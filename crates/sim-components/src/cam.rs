//! Electronic cam (a CAM table): the slave's position as a function of a master's, as a drive's cam technology runs it
//! (SIMOTION, TIA Portal's technology objects, the "Kurvenscheibe"). The master is a virtual axis or the web or conveyor
//! the slave works on; its speed sets how fast the slave moves: slave speed s'(m) m', acceleration s''(m) m'^2 + s'(m) m'',
//! jerk s'''(m) m'^3 at steady master speed. Knows no machine.
//!
//! A cam is periodic: over one master `period` the slave rises by `advance` (0 for a slave that swings back, one turn for
//! a knife that turns on). It is made of segments (`Segment`) with the motion laws of VDI 2143 (`Law`), or of a table of
//! points (`parse_table`, `Cam::from_table`) through which a cubic runs.
//!
//! Where two segments meet, the slave's slope over the master is that of the one that sets it: a straight segment
//! (`Law::Linear`, a synchronous run) or a dwell; between two motion laws the slave is at rest there (rest in rest). A
//! motion law between neighbours that move (velocity in velocity, VDI 2143) keeps their slopes v0 and v1 at its ends and
//! has no acceleration there: s(u) = dm (v0 u + (v1 - v0) g(u)) + c f(u), u the share of the segment run, f the law
//! normalised from rest to rest, g(u) = u^3 - u^4 / 2 (whose slope 3 u^2 - 2 u^3 blends v0 into v1), and
//! c = ds - dm (v0 + v1) / 2 what the law itself has to rise.
//!
//! The seam to `profile`: the polynomial laws (3-4-5, 4-5-6-7) and the sine laws are written here once more, normalised
//! in `Law::normalised`; once `profile` has them as functions of a share of the run, that is the one place to call them.

use sim_core::{Config, ConfigError};

/// A motion law of VDI 2143, from rest to rest over u in [0, 1].
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Law {
    /// The slave stands: its rise must be 0.
    Dwell,
    /// Constant slope (a synchronous run): rise / master span throughout.
    Linear,
    /// The 5th-order polynomial 10 u^3 - 15 u^4 + 6 u^5: peak slope 1.875, peak acceleration 5.774, jerk 60 at the ends.
    Poly345,
    /// The 7th-order polynomial 35 u^4 - 84 u^5 + 70 u^6 - 20 u^7: no jerk at the ends; peak slope 2.1875, acceleration
    /// 7.513.
    Poly4567,
    /// The cycloid (inclined sine) u - sin(2 pi u) / (2 pi): peak slope 2, acceleration 2 pi, no acceleration at the ends.
    Cycloid,
    /// The simple sine (harmonic) (1 - cos(pi u)) / 2: peak slope pi / 2, but an acceleration of pi^2 / 2 at both ends,
    /// a jump against a neighbour at rest.
    Harmonic,
}

impl Law {
    pub fn from_name(name: &str) -> Option<Self> {
        Some(match name {
            "dwell" => Self::Dwell,
            "linear" => Self::Linear,
            "3-4-5" => Self::Poly345,
            "4-5-6-7" => Self::Poly4567,
            "cycloid" => Self::Cycloid,
            "harmonic" => Self::Harmonic,
            _ => return None,
        })
    }

    /// Whether the law sets its own slope (straight or standing) rather than taking its neighbours'.
    fn straight(self) -> bool {
        matches!(self, Self::Dwell | Self::Linear)
    }

    /// The law from rest to rest at the share u: [f, f', f'', f'''] over u.
    pub fn normalised(self, u: f64) -> [f64; 4] {
        use std::f64::consts::PI;
        match self {
            Self::Dwell => [0.0; 4],
            Self::Linear => [u, 1.0, 0.0, 0.0],
            Self::Poly345 => {
                let (u2, u3) = (u * u, u * u * u);
                [
                    10.0 * u3 - 15.0 * u3 * u + 6.0 * u3 * u2,
                    30.0 * u2 - 60.0 * u3 + 30.0 * u2 * u2,
                    60.0 * u - 180.0 * u2 + 120.0 * u3,
                    60.0 - 360.0 * u + 360.0 * u2,
                ]
            }
            Self::Poly4567 => {
                let (u2, u3, u4) = (u * u, u * u * u, u * u * u * u);
                [
                    35.0 * u4 - 84.0 * u4 * u + 70.0 * u4 * u2 - 20.0 * u4 * u3,
                    140.0 * u3 - 420.0 * u4 + 420.0 * u4 * u - 140.0 * u4 * u2,
                    420.0 * u2 - 1680.0 * u3 + 2100.0 * u4 - 840.0 * u4 * u,
                    840.0 * u - 5040.0 * u2 + 8400.0 * u3 - 4200.0 * u4,
                ]
            }
            Self::Cycloid => {
                let w = 2.0 * PI;
                [u - (w * u).sin() / w, 1.0 - (w * u).cos(), w * (w * u).sin(), w * w * (w * u).cos()]
            }
            Self::Harmonic => [
                0.5 * (1.0 - (PI * u).cos()),
                0.5 * PI * (PI * u).sin(),
                0.5 * PI * PI * (PI * u).cos(),
                -0.5 * PI * PI * PI * (PI * u).sin(),
            ],
        }
    }
}

/// One segment of a cam: over `master` of the master the slave rises by `slave` along `law`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Segment {
    pub master: f64,
    pub slave: f64,
    pub law: Law,
}

#[derive(Debug, Clone)]
enum Piece {
    /// A segment of a motion law from `m0` over `dm`, the slave from `s0` by `ds`, its slopes `v0` and `v1` at the ends.
    Law { m0: f64, dm: f64, s0: f64, ds: f64, v0: f64, v1: f64, law: Law },
    /// A cubic from (m0, s0) with slope k0 to (m0 + h, s1) with slope k1.
    Cubic { m0: f64, h: f64, s0: f64, s1: f64, k0: f64, k1: f64 },
}

impl Piece {
    fn start(&self) -> f64 {
        match self {
            Self::Law { m0, .. } | Self::Cubic { m0, .. } => *m0,
        }
    }

    /// [s, s', s'', s'''] over the master at `m`, within the piece.
    fn at(&self, m: f64) -> [f64; 4] {
        match *self {
            Self::Law { m0, dm, s0, ds, v0, v1, law } => {
                let u = ((m - m0) / dm).clamp(0.0, 1.0);
                if law == Law::Linear {
                    return [s0 + ds * u, ds / dm, 0.0, 0.0];
                }
                let f = law.normalised(u);
                let (g, g1, g2, g3) = (
                    u * u * u - 0.5 * u * u * u * u,
                    3.0 * u * u - 2.0 * u * u * u,
                    6.0 * u - 6.0 * u * u,
                    6.0 - 12.0 * u,
                );
                let (dv, c) = (v1 - v0, ds - dm * (v0 + v1) / 2.0);
                [
                    s0 + dm * (v0 * u + dv * g) + c * f[0],
                    v0 + dv * g1 + c * f[1] / dm,
                    dv * g2 / dm + c * f[2] / (dm * dm),
                    dv * g3 / (dm * dm) + c * f[3] / (dm * dm * dm),
                ]
            }
            Self::Cubic { m0, h, s0, s1, k0, k1 } => {
                let u = ((m - m0) / h).clamp(0.0, 1.0);
                let (u2, u3) = (u * u, u * u * u);
                // Hermite basis: position and its rates over u, then over m.
                let b = [
                    [2.0 * u3 - 3.0 * u2 + 1.0, u3 - 2.0 * u2 + u, -2.0 * u3 + 3.0 * u2, u3 - u2],
                    [6.0 * u2 - 6.0 * u, 3.0 * u2 - 4.0 * u + 1.0, -6.0 * u2 + 6.0 * u, 3.0 * u2 - 2.0 * u],
                    [12.0 * u - 6.0, 6.0 * u - 4.0, -12.0 * u + 6.0, 6.0 * u - 2.0],
                    [12.0, 6.0, -12.0, 6.0],
                ];
                let at = |r: [f64; 4]| r[0] * s0 + r[1] * h * k0 + r[2] * s1 + r[3] * h * k1;
                [at(b[0]), at(b[1]) / h, at(b[2]) / (h * h), at(b[3]) / (h * h * h)]
            }
        }
    }
}

/// A periodic cam: its pieces over one master period from 0, and what the slave rises by over the period.
#[derive(Debug, Clone)]
pub struct Cam {
    pieces: Vec<Piece>,
    pub period: f64,
    pub advance: f64,
}

impl Cam {
    /// The cam of `segments`, the slave starting at 0. An error says which segment (from 0) is wrong: a span that is not
    /// positive, a dwell that rises, or two straight neighbours of different slopes (the slave's speed would jump).
    pub fn from_segments(segments: &[Segment]) -> Result<Self, usize> {
        let n = segments.len();
        if n == 0 {
            return Err(0);
        }
        for (i, s) in segments.iter().enumerate() {
            if !(s.master > 0.0 && s.master.is_finite() && s.slave.is_finite()) || s.law == Law::Dwell && s.slave != 0.0
            {
                return Err(i);
            }
        }
        let slope = |s: &Segment| if s.law == Law::Linear { s.slave / s.master } else { 0.0 };
        // The slope where segment i ends and i + 1 begins (the last one meets the first: the cam goes round).
        let mut junction = Vec::with_capacity(n);
        for i in 0..n {
            let (a, b) = (&segments[i], &segments[(i + 1) % n]);
            junction.push(match (a.law.straight(), b.law.straight()) {
                (true, true) if (slope(a) - slope(b)).abs() > 1e-9 * slope(a).abs().max(slope(b).abs()).max(1e-12) => {
                    return Err((i + 1) % n);
                }
                (true, _) => slope(a),
                (false, true) => slope(b),
                (false, false) => 0.0,
            });
        }
        let (mut m0, mut s0, mut pieces) = (0.0, 0.0, Vec::with_capacity(n));
        for (i, s) in segments.iter().enumerate() {
            let (v0, v1) = (junction[(i + n - 1) % n], junction[i]);
            pieces.push(Piece::Law { m0, dm: s.master, s0, ds: s.slave, v0, v1, law: s.law });
            m0 += s.master;
            s0 += s.slave;
        }
        Ok(Self { pieces, period: m0, advance: s0 })
    }

    /// The cam through a table of points (master, slave, slope over the master where given), the master rising; the
    /// last point closes the period (the master's span of the table is the period, the slave's its advance). Where a
    /// row has no slope, the slopes are those of the periodic cubic spline through the points (the slave's acceleration
    /// is then continuous), else a cubic runs between each two points with their slopes. An error names the row (from
    /// 0) that is wrong: fewer than three points, a master that does not rise, or slopes on some rows only.
    pub fn from_table(rows: &[(f64, f64, Option<f64>)]) -> Result<Self, usize> {
        let n = rows.len();
        if n < 3 {
            return Err(n);
        }
        if let Some(i) = (1..n).find(|&i| !(rows[i].0 > rows[i - 1].0)) {
            return Err(i);
        }
        let given = rows.iter().filter(|r| r.2.is_some()).count();
        if given != 0 && given != n {
            return Err(rows.iter().position(|r| r.2.is_none()).unwrap_or(0));
        }
        let (period, advance) = (rows[n - 1].0 - rows[0].0, rows[n - 1].1 - rows[0].1);
        let slopes: Vec<f64> = if given == n {
            rows.iter().map(|r| r.2.unwrap_or_default()).collect()
        } else {
            // Periodic spline over the n - 1 intervals: unknowns k_0 .. k_{n-2}, k_{n-1} = k_0. At each point the second
            // derivative from the left equals that from the right:
            // k_{i-1} / h_{i-1} + 2 k_i (1 / h_{i-1} + 1 / h_i) + k_{i+1} / h_i = 3 (d_{i-1} / h_{i-1} + d_i / h_i), d = rise / h.
            let p = n - 1;
            let h: Vec<f64> = (0..p).map(|i| rows[i + 1].0 - rows[i].0).collect();
            let d: Vec<f64> = (0..p).map(|i| (rows[i + 1].1 - rows[i].1) / h[i]).collect();
            let mut a = vec![0.0; p * p];
            let mut b = vec![0.0; p];
            for i in 0..p {
                let before = (i + p - 1) % p;
                a[i * p + before] += 1.0 / h[before];
                a[i * p + i] += 2.0 * (1.0 / h[before] + 1.0 / h[i]);
                a[i * p + (i + 1) % p] += 1.0 / h[i];
                b[i] = 3.0 * (d[before] / h[before] + d[i] / h[i]);
            }
            let mut k = crate::chain::solve(&a, &b).ok_or(0usize)?;
            k.push(k[0]);
            k
        };
        let pieces = (0..n - 1)
            .map(|i| Piece::Cubic {
                m0: rows[i].0 - rows[0].0,
                h: rows[i + 1].0 - rows[i].0,
                s0: rows[i].1 - rows[0].1,
                s1: rows[i + 1].1 - rows[0].1,
                k0: slopes[i],
                k1: slopes[i + 1],
            })
            .collect();
        Ok(Self { pieces, period, advance })
    }

    /// The cam of `<prefix>.table_file` (a CSV file of master, slave and an optional slope, see `parse_table`) where
    /// that is not empty, else of the array `[[<prefix>.segment]]`, each with `master`, `slave` (in the master's and the
    /// slave's units) and `law` (`Law::from_name`).
    pub fn from_config(cfg: &Config, prefix: &str) -> Result<Self, ConfigError> {
        let file = cfg.get(&format!("{prefix}.table_file"))?.as_str().unwrap_or_default().to_string();
        if !file.is_empty() {
            let bad = |detail: String| ConfigError::Invalid(format!("{prefix}.table_file: {file}: {detail}"));
            let text = std::fs::read_to_string(&file).map_err(|e| bad(e.to_string()))?;
            let rows = parse_table(&text).map_err(|line| bad(format!("line {line}")))?;
            return Self::from_table(&rows).map_err(|row| bad(format!("row {row}")));
        }
        let key = format!("{prefix}.segment");
        let number =
            |v: Option<&toml::Value>| v.and_then(|v| v.as_float().or_else(|| v.as_integer().map(|i| i as f64)));
        let segments = cfg
            .array(&key)?
            .iter()
            .enumerate()
            .map(|(i, s)| {
                let law = s.get("law").and_then(|l| l.as_str()).and_then(Law::from_name);
                match (number(s.get("master")), number(s.get("slave")), law) {
                    (Some(master), Some(slave), Some(law)) => Ok(Segment { master, slave, law }),
                    _ => Err(ConfigError::Invalid(format!("{key}[{i}]"))),
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
        Self::from_segments(&segments).map_err(|i| ConfigError::Invalid(format!("{key}[{i}]")))
    }

    /// [s, s', s'', s'''] over the master at any master position: the period repeats, each time `advance` higher.
    pub fn at(&self, m: f64) -> [f64; 4] {
        let turns = (m / self.period).floor();
        let r = m - turns * self.period;
        let i = self.pieces.partition_point(|p| p.start() <= r).saturating_sub(1);
        let mut out = self.pieces[i].at(r);
        out[0] += turns * self.advance;
        out
    }

    /// The highest slave speed, acceleration and jerk over a period at the steady master speed `speed`:
    /// max |s'| v, max |s''| v^2, max |s'''| v^3, sampled at 4000 points and at every piece's ends.
    pub fn peaks(&self, speed: f64) -> [f64; 3] {
        let n = 4000;
        let mut at: Vec<f64> = (0..=n).map(|i| self.period * i as f64 / n as f64).collect();
        for p in &self.pieces {
            // Just inside each end, where a law's jerk is highest.
            at.extend([p.start() + 1e-9 * self.period, p.start() - 1e-9 * self.period]);
        }
        let mut peak = [0f64; 3];
        for m in at.into_iter().filter(|m| (0.0..=self.period).contains(m)) {
            let s = self.at(m);
            for k in 0..3 {
                peak[k] = peak[k].max(s[k + 1].abs() * speed.powi(k as i32 + 1));
            }
        }
        peak
    }
}

/// Rows of a cam table in CSV: master, slave and optionally the slope of the slave over the master, comma separated. A
/// line that does not start with a number (a header, a comment beginning with `#`, an empty line) is skipped. An error
/// gives the number of the line (from 1) that has a number first and then something that is not one, or too many fields.
pub fn parse_table(text: &str) -> Result<Vec<(f64, f64, Option<f64>)>, usize> {
    let mut rows = Vec::new();
    for (i, line) in text.lines().enumerate() {
        let mut fields = line.split(',').map(|f| f.trim().parse::<f64>());
        let Some(Ok(master)) = fields.next() else { continue };
        let rest: Vec<f64> = fields.collect::<Result<_, _>>().map_err(|_| i + 1)?;
        match rest[..] {
            [slave] => rows.push((master, slave, None)),
            [slave, slope] => rows.push((master, slave, Some(slope))),
            _ => return Err(i + 1),
        }
    }
    Ok(rows)
}

/// The master's travel, speed and acceleration `t` seconds after it starts from rest: up to `speed` in `ramp` seconds by
/// a sin^2 speed ramp (no jump in acceleration), steady, and from `stop` seconds on down to rest by the mirror ramp.
/// A stop asked for during the run-up begins once the master is up to speed.
pub fn master_at(speed: f64, ramp: f64, t: f64, stop: Option<f64>) -> [f64; 3] {
    master_change(0.0, speed, ramp, t, stop)
}

/// The master's travel, speed and acceleration `t` seconds after its speed is set from `from` to `speed`: the same sin^2
/// ramp between the two over `ramp` seconds (`master_at` is the one from rest), travel from (from + speed) ramp / 2 at
/// its end; then steady, and from `stop` on down to rest from `speed`.
pub fn master_change(from: f64, speed: f64, ramp: f64, t: f64, stop: Option<f64>) -> [f64; 3] {
    use std::f64::consts::PI;
    // The ramp of unit height from rest: travel, speed and acceleration.
    let unit = |t: f64| -> [f64; 3] {
        if ramp <= 0.0 || t >= ramp {
            return [t - ramp / 2.0, 1.0, 0.0];
        }
        let w = PI / ramp;
        [t / 2.0 - (w * t).sin() / (2.0 * w), (w * t / 2.0).sin().powi(2), w / 2.0 * (w * t).sin()]
    };
    let up = |t: f64| -> [f64; 3] {
        let u = unit(t);
        [from * t + (speed - from) * u[0], from + (speed - from) * u[1], (speed - from) * u[2]]
    };
    let t = t.max(0.0);
    let Some(stop) = stop.map(|s| s.max(ramp)) else { return up(t) };
    if t <= stop {
        return up(t);
    }
    let base = up(stop)[0];
    let tau = (t - stop).min(ramp);
    if ramp <= 0.0 {
        return [base, 0.0, 0.0];
    }
    // The run-up turned round: travel ramp / 2 at most, speed falling as cos^2.
    let r = unit(tau);
    [base + speed * (tau - r[0]), speed * (1.0 - r[1]), -speed * r[2]]
}

/// The slave's position, speed and acceleration from the cam's [s, s', s'', s'''] and the master's [m, m', m''].
pub fn slave_motion(cam: [f64; 4], master: [f64; 3]) -> [f64; 3] {
    [cam[0], cam[1] * master[1], cam[2] * master[1] * master[1] + cam[1] * master[2]]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rest_to_rest(law: Law) -> Cam {
        Cam::from_segments(&[Segment { master: 1.0, slave: 1.0, law }, Segment { master: 1.0, slave: -1.0, law }])
            .unwrap()
    }

    /// 3-4-5 over a master span T at master speed V, rise h: the slave's peak speed is 1.875 h V / T, at mid-span, and its
    /// peak acceleration 10 / sqrt(3) h V^2 / T^2 = 5.7735 h V^2 / T^2, at u = (3 - sqrt 3) / 6 = 0.2113. With h = 0.1 m,
    /// T = 2 rad and V = 4 rad/s: 0.375 m/s and 5.7735 * 0.1 * 16 / 4 = 2.3094 m/s^2; the jerk at the start 60 h V^3 / T^3
    /// = 60 * 0.1 * 64 / 8 = 48 m/s^3.
    #[test]
    fn a_3_4_5_segment_peaks_at_its_hand_worked_speed_acceleration_and_jerk() {
        let cam = Cam::from_segments(&[
            Segment { master: 2.0, slave: 0.1, law: Law::Poly345 },
            Segment { master: 2.0, slave: -0.1, law: Law::Poly345 },
        ])
        .unwrap();
        let [v, a, j] = cam.peaks(4.0);
        assert!((v - 0.375).abs() < 1e-9, "{v}");
        assert!((a - 0.1 * 16.0 / 4.0 * 10.0 / 3f64.sqrt()).abs() < 1e-5, "{a}");
        assert!((a - 2.3094).abs() < 1e-4 && (j - 48.0).abs() < 1e-6, "{a} {j}");
        // At mid-span half the rise, and the slope there 1.875 h / T.
        let mid = cam.at(1.0);
        assert!((mid[0] - 0.05).abs() < 1e-12 && (mid[1] - 1.875 * 0.1 / 2.0).abs() < 1e-12);
    }

    /// Every law rises from 0 to 1 at rest at both ends (the harmonic with its end accelerations), its rates the
    /// differences of the one before.
    #[test]
    fn every_law_rises_by_one_from_rest_to_rest_with_consistent_rates() {
        for law in [Law::Poly345, Law::Poly4567, Law::Cycloid, Law::Harmonic] {
            let (start, end) = (law.normalised(0.0), law.normalised(1.0));
            assert!(start[0].abs() < 1e-12 && (end[0] - 1.0).abs() < 1e-12, "{law:?}");
            assert!(start[1].abs() < 1e-12 && end[1].abs() < 1e-12, "{law:?}");
            let h = 1e-6;
            for u in [0.1, 0.37, 0.5, 0.83] {
                let (a, b) = (law.normalised(u - h), law.normalised(u + h));
                for k in 0..3 {
                    let diff = (b[k] - a[k]) / (2.0 * h);
                    assert!((diff - law.normalised(u)[k + 1]).abs() < 1e-5 * (1.0 + diff.abs()), "{law:?} {k} {u}");
                }
            }
        }
        assert!((rest_to_rest(Law::Poly4567).peaks(1.0)[0] - 2.1875).abs() < 1e-9);
        assert!((rest_to_rest(Law::Cycloid).peaks(1.0)[1] - std::f64::consts::TAU).abs() < 1e-6);
    }

    /// A law between two synchronous runs of slope 1 / R (a knife that matches the web): its slopes at the ends are the
    /// neighbours', and with 3-4-5 the slope at mid-span is v + 1.875 (ds / dm - v).
    #[test]
    fn a_law_between_two_runs_keeps_their_slope_and_no_acceleration_at_its_ends() {
        let v = 10.0;
        let cam = Cam::from_segments(&[
            Segment { master: 0.01, slave: 0.1, law: Law::Linear },
            Segment { master: 0.5, slave: 6.0, law: Law::Poly345 },
            Segment { master: 0.01, slave: 0.1, law: Law::Linear },
        ])
        .unwrap();
        for m in [0.01 + 1e-12, 0.51 - 1e-12] {
            let s = cam.at(m);
            assert!((s[1] - v).abs() < 1e-6 && s[2].abs() < 1e-3, "{s:?}");
        }
        let mid = cam.at(0.26);
        assert!((mid[1] - (v + 1.875 * (6.0 / 0.5 - v))).abs() < 1e-9, "{mid:?}");
        assert!((cam.period - 0.52).abs() < 1e-12 && (cam.advance - 6.2).abs() < 1e-12);
        // The next period goes on 6.2 higher.
        assert!((cam.at(0.52 + 0.26)[0] - cam.at(0.26)[0] - 6.2).abs() < 1e-12);
    }

    #[test]
    fn segments_that_cannot_join_are_refused_by_their_index() {
        let lin = |master: f64, slave: f64| Segment { master, slave, law: Law::Linear };
        assert_eq!(Cam::from_segments(&[lin(1.0, 1.0), lin(1.0, 2.0)]).unwrap_err(), 1);
        assert_eq!(
            Cam::from_segments(&[lin(1.0, 1.0), Segment { master: 1.0, slave: 0.5, law: Law::Dwell }]).unwrap_err(),
            1
        );
        assert_eq!(Cam::from_segments(&[lin(0.0, 1.0)]).unwrap_err(), 0);
    }

    /// A table of a sine with its slopes runs through the sine; without slopes the periodic spline is near it too.
    #[test]
    fn a_table_imported_from_csv_runs_through_its_points() {
        let text: String = std::iter::once("master,slave,slope\n# a sine over one turn\n".to_string())
            .chain((0..=36).map(|i| {
                let m = i as f64 * 10f64.to_radians();
                format!("{m}, {}, {}\n", m.sin(), m.cos())
            }))
            .collect();
        let rows = parse_table(&text).unwrap();
        assert_eq!(rows.len(), 37);
        let with = Cam::from_table(&rows).unwrap();
        let without = Cam::from_table(&rows.iter().map(|r| (r.0, r.1, None)).collect::<Vec<_>>()).unwrap();
        for m in [0.3, 1.0, 2.5, 4.0, 6.0] {
            assert!((with.at(m)[0] - m.sin()).abs() < 1e-5 && (with.at(m)[1] - m.cos()).abs() < 1e-3, "{m}");
            assert!((without.at(m)[0] - m.sin()).abs() < 1e-4 && (without.at(m)[2] + m.sin()).abs() < 2e-2, "{m}");
        }
        assert!(with.advance.abs() < 1e-12 && (with.period - std::f64::consts::TAU).abs() < 1e-12);
        assert_eq!(parse_table("0, 1\n1, x\n"), Err(2));
        assert_eq!(Cam::from_table(&[(0.0, 0.0, None), (1.0, 1.0, Some(1.0)), (2.0, 2.0, None)]).unwrap_err(), 0);
    }

    /// A new master speed while the cam runs, from 2 to 4 m/s over a ramp of 0.5 s: the speed (V0 + V1) / 2 = 3 m/s at
    /// mid-ramp, with the ramp's highest acceleration (V1 - V0) pi / (2 ramp) = 2 pi m/s2 there; 4 m/s at its end,
    /// having gone (V0 + V1) ramp / 2 = 1.5 m; at no point a step in speed or acceleration (0 at both ends). 0.25 s later
    /// it has gone 1.5 + 4 * 0.25 = 2.5 m. Asked to stop at 1 s, it runs down from 4 m/s over 0.5 s: 1 m more.
    #[test]
    fn a_new_master_speed_ramps_from_the_old_one() {
        use std::f64::consts::PI;
        let (v0, v1, ramp) = (2.0, 4.0, 0.5);
        assert_eq!(master_change(v0, v1, ramp, 0.0, None), [0.0, 2.0, 0.0]);
        let mid = master_change(v0, v1, ramp, 0.25, None);
        assert!((mid[1] - 3.0).abs() < 1e-12 && (mid[2] - (v1 - v0) * PI / (2.0 * ramp)).abs() < 1e-12, "{mid:?}");
        let end = master_change(v0, v1, ramp, ramp, None);
        assert!((end[0] - 1.5).abs() < 1e-12 && (end[1] - 4.0).abs() < 1e-12 && end[2].abs() < 1e-12, "{end:?}");
        assert!((master_change(v0, v1, ramp, 0.75, None)[0] - 2.5).abs() < 1e-12);
        let stopped = master_change(v0, v1, ramp, 3.0, Some(1.0));
        assert!((stopped[0] - (1.5 + 4.0 * 0.5 + 1.0)).abs() < 1e-12 && stopped[1].abs() < 1e-12, "{stopped:?}");
        let h = 1e-6;
        for t in [0.1, 0.3, 0.49, 0.6, 1.2] {
            let (a, b) = (master_change(v0, v1, ramp, t - h, Some(1.0)), master_change(v0, v1, ramp, t + h, Some(1.0)));
            let at = master_change(v0, v1, ramp, t, Some(1.0));
            assert!(
                ((b[0] - a[0]) / (2.0 * h) - at[1]).abs() < 1e-6 && ((b[1] - a[1]) / (2.0 * h) - at[2]).abs() < 1e-5
            );
        }
    }

    /// The master's sin^2 ramp: up to speed in `ramp` seconds having gone ramp V / 2, down again over the same.
    #[test]
    fn the_master_runs_up_and_down_by_its_ramp() {
        let (v, ramp) = (2.0, 0.5);
        assert_eq!(master_at(v, ramp, 0.0, None), [0.0, 0.0, 0.0]);
        let up = master_at(v, ramp, ramp, None);
        assert!((up[0] - 0.5).abs() < 1e-12 && (up[1] - v).abs() < 1e-12);
        let half = master_at(v, ramp, ramp / 2.0, None);
        // Peak acceleration V pi / (2 ramp) at mid-ramp.
        assert!((half[1] - 1.0).abs() < 1e-12 && (half[2] - v * std::f64::consts::PI / (2.0 * ramp)).abs() < 1e-12);
        let stop = 2.0;
        let end = master_at(v, ramp, stop + ramp + 1.0, Some(stop));
        // 0.5 m up, 1.5 s steady at 2 m/s, 0.5 m down.
        assert!((end[0] - (0.5 + 3.0 + 0.5)).abs() < 1e-12 && end[1].abs() < 1e-12, "{end:?}");
        let h = 1e-6;
        for t in [0.1, 0.3, 1.0, 2.1, 2.3] {
            let (a, b) = (master_at(v, ramp, t - h, Some(stop)), master_at(v, ramp, t + h, Some(stop)));
            let mid = master_at(v, ramp, t, Some(stop));
            assert!(
                ((b[0] - a[0]) / (2.0 * h) - mid[1]).abs() < 1e-6 && ((b[1] - a[1]) / (2.0 * h) - mid[2]).abs() < 1e-5
            );
        }
    }
}
