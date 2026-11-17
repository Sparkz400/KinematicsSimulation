//! Input shapers: trains of impulses that, convolved with a setpoint, cancel the vibration it starts in a mode of known
//! frequency and damping (Singer and Seering 1990, Singhose 1994–1997). Times are in half damped periods of the mode,
//! T_d / 2 = 1 / (2 f sqrt(1 - zeta^2)); amplitudes add up to one.
//!
//! Undamped, each shaper is a sum of impulses at multiples of the half period: ZV 1 : 1, ZVD 1 : 2 : 1, ZVDD
//! 1 : 3 : 3 : 1, EI (1+V)/4 : (1-V)/2 : (1+V)/4, the two-hump EI A : 1/2 - A : 1/2 - A : A with
//! A = (3X^2 + 2X + 3V^2) / (16 X), X = (V^2 (sqrt(1 - V^2) + 1))^(1/3), V the vibration the EI shapers allow: EI leaves
//! V at the design frequency and none a little either side of it, the two-hump EI none at it and humps of V either
//! side, so both stay at or below V over a wider band than ZVD (Singhose, Seering and Singer 1994–1997).
//!
//! Damped, impulse i is weighted by K^i, K = exp(-zeta pi / sqrt(1 - zeta^2)), and all by their sum: that cancels the
//! mode's decay between the impulses. For ZV, ZVD and ZVDD it is the textbook damped shaper, which leaves nothing at
//! the design frequency.
//!
//! The damped EI shapers are solved, not weighted (weighting leaves V K^n / sum(a_i K^i) at the design frequency, 0.0424
//! for V = 0.05 at zeta = 0.05). With r the mode's frequency over the design frequency, g = zeta / sqrt(1 - zeta^2) and
//! the times t_i in half damped periods of the design, the vibration is V(r) = |G(r)|,
//! G(r) = sum A_i exp(pi r (t_i - t_n) g) exp(i pi r t_i) (the decay of the mode between the impulses and the phase of
//! each). The three-impulse EI has A_1, A_2 (A_3 = 1 - A_1 - A_2), t_2, t_3 and the two flank frequencies r_1, r_2
//! as unknowns, and six equations: G(r_1) = G(r_2) = 0 (two real each), |G(1)| = V and dV/dr = Re(conj(G) G') = 0 at
//! the design frequency. The two-hump EI has A_1..A_3 (A_4 = 1 - their sum), t_2..t_4, the frequencies r_l, r_h of
//! its two outer zeros and r_a, r_b of its humps as unknowns, and ten equations: G = 0 at 1, r_l and r_h (two each),
//! |G| = V and Re(conj(G) G') = 0 at each hump; undamped that is the closed form (three zeros, two humps of V between),
//! and damped it is Singhose's damped two-hump EI (its published curve fit for V = 5 % has the solved amplitudes to
//! 6e-4 and times to 2 % at zeta up to 0.2, the fit's own accuracy). Four impulses at a common spacing are no solution
//! with damping: at zeta 0.1 the closest such shaper misses the equations by 8e-4, at 0.2 by 3e-3.
//! Newton's method with a finite-difference Jacobian solves them, from the undamped closed form at zeta = 0
//! carried to the damping in steps of at most 0.01 (the solution moves continuously); the residual at the design
//! frequency is V (EI) or 0 (two-hump) to 1e-12. Where the iteration does not converge (damping near 1) the shaper
//! falls back to the undamped weights times K^i.

use std::f64::consts::PI;

/// The kinds of shaper (`profile.shaper_type`, `type` of a `profile.shaper_modes` entry).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Kind {
    Zv,
    #[default]
    Zvd,
    Zvdd,
    Ei,
    TwoHumpEi,
}

impl Kind {
    pub const NAMES: [&str; 5] = ["zv", "zvd", "zvdd", "ei", "ei2"];

    pub fn from_name(name: &str) -> Option<Self> {
        [Self::Zv, Self::Zvd, Self::Zvdd, Self::Ei, Self::TwoHumpEi].into_iter().find(|k| k.name() == name)
    }

    pub fn name(&self) -> &'static str {
        Self::NAMES[*self as usize]
    }
}

/// The impulses of a shaper of `kind` for a mode of damping ratio `damping`: (time in half damped periods, amplitude),
/// the amplitudes adding up to one. `tolerance`: the vibration V the EI shapers allow (0 to 1).
pub fn impulses(kind: Kind, damping: f64, tolerance: f64) -> Vec<(f64, f64)> {
    let v = tolerance;
    let undamped: Vec<f64> = match kind {
        Kind::Zv => vec![1.0, 1.0],
        Kind::Zvd => vec![1.0, 2.0, 1.0],
        Kind::Zvdd => vec![1.0, 3.0, 3.0, 1.0],
        Kind::Ei => vec![(1.0 + v) / 4.0, (1.0 - v) / 2.0, (1.0 + v) / 4.0],
        Kind::TwoHumpEi => {
            let x = (v * v * ((1.0 - v * v).sqrt() + 1.0)).cbrt();
            // At V = 0 it is ZVDD's 1/8 : 3/8 : 3/8 : 1/8.
            let a = if x > 0.0 { (3.0 * x * x + 2.0 * x + 3.0 * v * v) / (16.0 * x) } else { 0.125 };
            vec![a, 0.5 - a, 0.5 - a, a]
        }
    };
    if matches!(kind, Kind::Ei | Kind::TwoHumpEi)
        && damping > 0.0
        && v > 1e-6
        && let Some(solved) = damped_ei(kind, damping, v)
    {
        return solved;
    }
    let k = (-damping * PI / (1.0 - damping * damping).sqrt()).exp();
    let weighted: Vec<f64> = undamped.iter().enumerate().map(|(i, a)| a * k.powi(i as i32)).collect();
    let sum: f64 = weighted.iter().sum();
    weighted.iter().enumerate().map(|(i, a)| (i as f64, a / sum)).collect()
}

/// G(r) and dG/dr (as [re, im]) of impulses `amps` at `taus` (half damped periods of the design frequency, the last the
/// latest) at the frequency ratio `r`, for g = zeta / sqrt(1 - zeta^2): see the module's derivation.
fn vibration(amps: &[f64], taus: &[f64], g: f64, r: f64) -> ([f64; 2], [f64; 2]) {
    let last = taus.iter().fold(f64::MIN, |m, t| m.max(*t));
    let (mut v, mut d) = ([0.0; 2], [0.0; 2]);
    for (a, t) in amps.iter().zip(taus) {
        let w = a * (PI * r * (t - last) * g).exp();
        let (sin, cos) = (PI * r * t).sin_cos();
        v[0] += w * cos;
        v[1] += w * sin;
        let decay = (t - last) * g;
        d[0] += w * PI * (decay * cos - t * sin);
        d[1] += w * PI * (decay * sin + t * cos);
    }
    (v, d)
}

/// |G| at `r` less `level`, and Re(conj(G) G'): the two equations of a point where the vibration has `level` at a
/// stationary value.
fn stationary(amps: &[f64], taus: &[f64], g: f64, r: f64, level: f64) -> [f64; 2] {
    let (v, d) = vibration(amps, taus, g, r);
    [v[0].hypot(v[1]) - level, v[0] * d[0] + v[1] * d[1]]
}

/// Solves f(u) = 0 (as many equations as unknowns) by Newton's method from `u`, the Jacobian by central differences;
/// whether it converged to 1e-12.
fn newton(u: &mut [f64], f: &dyn Fn(&[f64]) -> Vec<f64>) -> bool {
    let n = u.len();
    let size = |r: &[f64]| r.iter().fold(0.0f64, |m, x| m.max(x.abs()));
    for _ in 0..60 {
        let r = f(u);
        if size(&r) < 1e-13 {
            return true;
        }
        let mut m = vec![vec![0.0; n + 1]; n];
        for c in 0..n {
            let h = 1e-7 * u[c].abs().max(1.0);
            let (mut up, mut down) = (u.to_vec(), u.to_vec());
            up[c] += h;
            down[c] -= h;
            let (fu, fd) = (f(&up), f(&down));
            for row in 0..n {
                m[row][c] = (fu[row] - fd[row]) / (2.0 * h);
            }
        }
        for row in 0..n {
            m[row][n] = -r[row];
        }
        for col in 0..n {
            let pivot = (col..n).max_by(|a, b| m[*a][col].abs().total_cmp(&m[*b][col].abs())).unwrap_or(col);
            if m[pivot][col].abs() < 1e-14 {
                return false;
            }
            m.swap(col, pivot);
            for row in 0..n {
                if row != col {
                    let factor = m[row][col] / m[col][col];
                    for k in col..=n {
                        m[row][k] -= factor * m[col][k];
                    }
                }
            }
        }
        for c in 0..n {
            u[c] += m[c][n] / m[c][c];
        }
        if u.iter().any(|x| !x.is_finite()) {
            return false;
        }
    }
    size(&f(u)) < 1e-12
}

/// The damped EI (three impulses) or two-hump EI (four) for `damping` > 0 and tolerance `v`, by the module's equations;
/// None where the iteration fails.
fn damped_ei(kind: Kind, damping: f64, v: f64) -> Option<Vec<(f64, f64)>> {
    let g_of = |zeta: f64| zeta / (1.0 - zeta * zeta).sqrt();
    let steps = (damping / 0.01).ceil().max(1.0) as usize;
    let three = kind == Kind::Ei;
    // The undamped solution: amplitudes, times, and the frequencies of the zeros (and humps). Undamped the two-hump
    // EI's vibration is |2A cos(3x) + (1 - 2A) cos x|, x = pi r / 2: zeros where cos^2 x = (8A - 1) / (8A), humps
    // where cos^2 x = (8A - 1) / (24A).
    let mut u: Vec<f64> = if three {
        let r1 = (-(1.0 - v) / (1.0 + v)).acos() / PI;
        vec![(1.0 + v) / 4.0, (1.0 - v) / 2.0, 1.0, 2.0, r1, 2.0 - r1]
    } else {
        let x = (v * v * ((1.0 - v * v).sqrt() + 1.0)).cbrt();
        let a = (3.0 * x * x + 2.0 * x + 3.0 * v * v) / (16.0 * x);
        let at = |c2: f64| 2.0 / PI * c2.sqrt().acos();
        let (zero, hump) = (at((8.0 * a - 1.0) / (8.0 * a)), at((8.0 * a - 1.0) / (24.0 * a)));
        vec![a, 0.5 - a, 0.5 - a, 1.0, 2.0, 3.0, zero, 2.0 - zero, hump, 2.0 - hump]
    };
    for step in 1..=steps {
        let g = g_of(damping * step as f64 / steps as f64);
        let ok = if three {
            newton(&mut u, &|u| {
                let (amps, taus) = ([u[0], u[1], 1.0 - u[0] - u[1]], [0.0, u[2], u[3]]);
                let (v1, v2) = (vibration(&amps, &taus, g, u[4]).0, vibration(&amps, &taus, g, u[5]).0);
                let at_design = stationary(&amps, &taus, g, 1.0, v);
                vec![v1[0], v1[1], v2[0], v2[1], at_design[0], at_design[1]]
            })
        } else {
            newton(&mut u, &|u| {
                let (amps, taus) = ([u[0], u[1], u[2], 1.0 - u[0] - u[1] - u[2]], [0.0, u[3], u[4], u[5]]);
                let zeros = [1.0, u[6], u[7]].map(|r| vibration(&amps, &taus, g, r).0);
                let (a, b) = (stationary(&amps, &taus, g, u[8], v), stationary(&amps, &taus, g, u[9], v));
                zeros.iter().flatten().copied().chain(a).chain(b).collect()
            })
        };
        if !ok {
            return None;
        }
    }
    let impulses: Vec<(f64, f64)> = if three {
        vec![(0.0, u[0]), (u[2], u[1]), (u[3], 1.0 - u[0] - u[1])]
    } else {
        vec![(0.0, u[0]), (u[3], u[1]), (u[4], u[2]), (u[5], 1.0 - u[0] - u[1] - u[2])]
    };
    impulses.iter().all(|(t, a)| t.is_finite() && *a > 0.0 && a.is_finite()).then_some(impulses)
}

/// The shaper of two shapers in a row: every pair of impulses, at the sum of their times with the product of their
/// amplitudes. Times must be in the same unit.
pub fn convolve(a: &[(f64, f64)], b: &[(f64, f64)]) -> Vec<(f64, f64)> {
    a.iter().flat_map(|(ta, aa)| b.iter().map(move |(tb, ab)| (ta + tb, aa * ab))).collect()
}

/// The vibration a shaper leaves in a mode of `hz` and `damping`, as a share of what a single impulse leaves
/// (Singer and Seering): e^(-zeta w t_n) sqrt(C^2 + S^2), C and S the sums of A_i e^(zeta w t_i) cos and sin of
/// w_d t_i. `impulses` in seconds.
pub fn residual(impulses: &[(f64, f64)], hz: f64, damping: f64) -> f64 {
    let w = 2.0 * PI * hz;
    let wd = w * (1.0 - damping * damping).sqrt();
    let last = impulses.iter().fold(0.0, |m: f64, (t, _)| m.max(*t));
    let (c, s) = impulses.iter().fold((0.0, 0.0), |(c, s), (t, a)| {
        let e = a * (damping * w * (t - last)).exp();
        (c + e * (wd * t).cos(), s + e * (wd * t).sin())
    });
    (c * c + s * s).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A shaper designed for 10 Hz, its impulses in seconds: half periods of 1 / (2 f sqrt(1 - zeta^2)).
    fn at_10_hz(kind: Kind, damping: f64) -> Vec<(f64, f64)> {
        let half = 1.0 / (2.0 * 10.0 * (1.0 - damping * damping).sqrt());
        impulses(kind, damping, 0.05).into_iter().map(|(t, a)| (t * half, a)).collect()
    }

    /// ZV, undamped: two halves half a period apart leave |1 + e^(i pi r)| / 2 = |cos(pi r / 2)| at r times the design
    /// frequency: 0 at it, cos(0.4 pi) = 0.3090 at 0.8 and at 1.2. ZVD the square: 0.0955.
    #[test]
    fn zv_cancels_its_frequency_and_leaves_a_third_at_20_percent_off() {
        let zv = at_10_hz(Kind::Zv, 0.0);
        assert!(residual(&zv, 10.0, 0.0) < 1e-12);
        for hz in [8.0, 12.0] {
            assert!((residual(&zv, hz, 0.0) - 0.309_017).abs() < 1e-6, "{hz}: {}", residual(&zv, hz, 0.0));
            assert!((residual(&at_10_hz(Kind::Zvd, 0.0), hz, 0.0) - 0.309_017f64.powi(2)).abs() < 1e-6);
        }
    }

    /// EI with V = 5 %, undamped: 0.2625 : 0.475 : 0.2625 at 0, T/2, T leave |0.475 + 0.525 cos(pi r)|: 0.05 at the design
    /// frequency, |0.475 + 0.525 cos(0.8 pi)| = |0.475 - 0.42473| = 0.05027 at 0.8 and 1.2: within the 5 % over +-20 %, where
    /// ZV leaves 31 %.
    #[test]
    fn ei_leaves_its_tolerance_at_its_frequency_and_about_that_at_20_percent_off() {
        let ei = at_10_hz(Kind::Ei, 0.0);
        assert!(ei.iter().zip([0.2625, 0.475, 0.2625]).all(|(i, a)| (i.1 - a).abs() < 1e-15), "{ei:?}");
        assert!((residual(&ei, 10.0, 0.0) - 0.05).abs() < 1e-12);
        for hz in [8.0, 12.0] {
            let expected = (0.475 + 0.525 * (0.8 * PI).cos()).abs();
            assert!((expected - 0.050_266).abs() < 1e-6);
            assert!((residual(&ei, hz, 0.0) - expected).abs() < 1e-9, "{hz}: {}", residual(&ei, hz, 0.0));
        }
        // Damped: ZV still leaves nothing at its frequency.
        assert!(residual(&at_10_hz(Kind::Zv, 0.05), 10.0, 0.05) < 1e-12);
    }

    /// The damped EI is solved (module doc): at zeta = 0.05, 0.1 and 0.2 it leaves V = 5 % at the design frequency (the
    /// weighted shaper left 0.04245 at 0.05, by hand 0.05 K^2 / (0.2625 + 0.475 K + 0.2625 K^2), K = 0.85447), has no
    /// slope there (|dV/df| < 1e-6 per Hz by central differences), leaves nothing at its flanks (the minimum over the
    /// band below the design is under 1e-3: a scan in steps of 2.9 mHz, half a step off a zero where |dV/df| is at most 0.3 per Hz) and its amplitudes add up to one.
    #[test]
    fn the_damped_ei_leaves_its_tolerance_with_no_slope_at_the_design_frequency() {
        for zeta in [0.05, 0.1, 0.2] {
            let ei = at_10_hz(Kind::Ei, zeta);
            assert!((ei.iter().map(|i| i.1).sum::<f64>() - 1.0).abs() < 1e-12 && ei.iter().all(|i| i.1 > 0.0));
            assert!((residual(&ei, 10.0, zeta) - 0.05).abs() < 1e-9, "{zeta}: {}", residual(&ei, 10.0, zeta));
            let slope = (residual(&ei, 10.0 + 1e-4, zeta) - residual(&ei, 10.0 - 1e-4, zeta)) / 2e-4;
            assert!(slope.abs() < 1e-6, "{zeta}: slope {slope}");
            for (lo, hi) in [(7.0, 9.9), (10.1, 13.0)] {
                let least =
                    (0..=1000).map(|i| residual(&ei, lo + (hi - lo) * i as f64 / 1000.0, zeta)).fold(1.0, f64::min);
                assert!(least < 1e-3, "{zeta} {lo}: {least}");
            }
        }
        // Without damping it is the closed form.
        let ei = impulses(Kind::Ei, 0.0, 0.05);
        assert!(ei.iter().zip([0.2625, 0.475, 0.2625]).all(|(i, a)| (i.1 - a).abs() < 1e-15));
    }

    /// The damped two-hump EI leaves nothing at the design frequency (to 1e-9), its two humps are V = 5 % each (the
    /// largest vibration over 7 to 9.99 and 10.01 to 13 Hz, sampled every 1.5 mHz), and it leaves nothing at an outer
    /// frequency on either side (the least over 5 to 7.5 and 13 to 16 Hz). At zeta = 0.1 its amplitudes are Singhose's
    /// curve fit for V = 5 % (A_1 = 0.16054 + 0.76699 z + 2.2656 z^2 - 1.2275 z^3,
    /// A_2 = 0.33911 + 0.45081 z - 2.5808 z^2 + 1.7365 z^3, A_3 = 0.34089 - 0.61533 z - 0.68765 z^2 + 0.42261 z^3:
    /// 0.25867, 0.36012, 0.27290, 0.10831) within the fit's accuracy, 1e-3 (the run: 5.4e-4 at most).
    #[test]
    fn the_damped_two_hump_ei_has_two_humps_of_its_tolerance_and_no_vibration_at_its_frequency() {
        for zeta in [0.05, 0.1, 0.2] {
            let two = at_10_hz(Kind::TwoHumpEi, zeta);
            assert!((two.iter().map(|i| i.1).sum::<f64>() - 1.0).abs() < 1e-12 && two.iter().all(|i| i.1 > 0.0));
            assert!(residual(&two, 10.0, zeta) < 1e-9, "{zeta}: {}", residual(&two, 10.0, zeta));
            let scan = |lo: f64, hi: f64, n: usize| {
                (0..=n)
                    .map(|i| residual(&two, lo + (hi - lo) * i as f64 / n as f64, zeta))
                    .fold([1.0f64, 0.0f64], |[a, b], x| [a.min(x), b.max(x)])
            };
            let (below, above) = (scan(7.0, 9.99, 2000)[1], scan(10.01, 13.0, 2000)[1]);
            assert!((below - 0.05).abs() < 1e-6 && (above - 0.05).abs() < 1e-6, "{zeta}: {below} {above}");
            let (low, high) = (scan(5.0, 7.5, 20000)[0], scan(13.0, 16.0, 20000)[0]);
            assert!(low < 1e-4 && high < 1e-4, "{zeta}: {low} {high}");
        }
        let two = impulses(Kind::TwoHumpEi, 0.1, 0.05);
        for (i, fit) in two.iter().zip([0.25867, 0.36012, 0.27290, 0.10831]) {
            assert!((i.1 - fit).abs() < 1e-3, "{two:?}");
        }
    }

    /// The two-hump EI at V = 5 %: X = (0.0025 (sqrt(0.9975) + 1))^(1/3) = 0.17096, A = (3 X^2 + 2 X + 0.0075) / (16 X)
    /// = 0.15980. Undamped it leaves |8 A c^3 + (1 - 8 A) c|, c = cos(pi r / 2): nothing at the design frequency (c = 0),
    /// and humps where 24 A c^2 = 8 A - 1, of (2/3) (8 A - 1) sqrt((8 A - 1) / (24 A)) = (2/3) 0.27838 * 0.26942 = 0.0500:
    /// V, at c = 0.26942: r = (2 / pi) acos(c) = 0.8263 and 1.1737. From 7 to 13 Hz (c = cos(0.35 pi) = 0.45399 at the
    /// ends, |1.27838 * 0.09357 - 0.27838 * 0.45399| = 0.0068) the humps are the most it leaves; at 5 Hz (c = 0.7071) it
    /// leaves 0.255, outside the band it is insensitive over. At 0 vibration it is ZVDD.
    #[test]
    fn the_two_hump_ei_leaves_its_tolerance_at_its_humps_and_is_zvdd_without_one() {
        let x = (0.0025f64 * (0.9975f64.sqrt() + 1.0)).cbrt();
        let a = (3.0 * x * x + 2.0 * x + 0.0075) / (16.0 * x);
        assert!((x - 0.17096).abs() < 1e-5 && (a - 0.15980).abs() < 1e-5, "{x} {a}");
        let two = at_10_hz(Kind::TwoHumpEi, 0.0);
        assert!((two[0].1 - a).abs() < 1e-12 && (two[1].1 - (0.5 - a)).abs() < 1e-12, "{two:?}");
        assert!(residual(&two, 10.0, 0.0) < 1e-12, "{}", residual(&two, 10.0, 0.0));
        let hump = (0..=2000).map(|i| residual(&two, 7.0 + 6.0 * i as f64 / 2000.0, 0.0)).fold(0.0, f64::max);
        assert!((hump - 0.05).abs() < 1e-4, "{hump}");
        let zvdd: Vec<f64> = impulses(Kind::TwoHumpEi, 0.0, 0.0).iter().map(|i| i.1).collect();
        assert_eq!(zvdd, [0.125, 0.375, 0.375, 0.125]);
    }

    /// Two shapers convolved cancel both modes: ZV at 10 Hz and ZV at 25 Hz leave nothing at either.
    #[test]
    fn convolved_shapers_cancel_every_mode() {
        let b: Vec<(f64, f64)> = impulses(Kind::Zv, 0.0, 0.0).into_iter().map(|(t, a)| (t / 50.0, a)).collect();
        let both = convolve(&at_10_hz(Kind::Zv, 0.0), &b);
        assert_eq!(both.len(), 4);
        assert!((both.iter().map(|i| i.1).sum::<f64>() - 1.0).abs() < 1e-15);
        assert!(residual(&both, 10.0, 0.0) < 1e-12 && residual(&both, 25.0, 0.0) < 1e-12);
    }
}
