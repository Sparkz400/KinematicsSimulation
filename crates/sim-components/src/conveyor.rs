//! Picking from a moving belt conveyor: parts ride on a belt through a pick window; the tool meets each part with the
//! belt's speed, moves with it while the gripper closes, lifts off and places it at a fixed place or onto a second
//! moving belt. Works on any machine whose tool axes are Cartesian positions (a delta, a gantry), through the tool
//! pose alone; all vectors are tool poses or tool velocities, one value per axis (0 on a rotation axis).
//!
//! Every move of a pass is a superposition: a jerk-limited straight move (`profile::Move`) from where the phase starts
//! to a point Q, plus a belt term whose velocity blends from the start velocity `va` to the end velocity `vb` as
//! `va + (vb - va) h(u)`, `h(u) = 3u^2 - 2u^3`, u = t / T: zero acceleration at both ends, so the phases join without a
//! jump in acceleration. The belt term covers `T (va + vb) / 2`; a target that moves with `vb` is met at T when
//! `Q = target(0) + T (vb - va) / 2`. The move is planned with each axis's limits less what the belt term takes
//! (`max(|va|, |vb|)` of the speed, `1.5 |vb - va| / T` of the acceleration, `6 |vb - va| / T^2` of the jerk), and
//! T is the shortest time in which the move to that Q fits (`intercept`).

use crate::profile::Move;
use sim_core::{AxisInfo, Config, ConfigError};

type Result<T> = std::result::Result<T, ConfigError>;

/// The conveyor of `[conveyor]` and how far it has got: the parts picked and missed so far.
#[derive(Debug, Clone)]
pub struct Conveyor {
    /// Unit vector along the belt in tool coordinates, and the belt's speed, m/s, which it reaches `ramp` seconds after
    /// the start at a constant acceleration.
    pub direction: Vec<f64>,
    pub speed: f64,
    pub ramp: f64,
    /// The tool pose that grips a part where it enters the belt (belt position 0), and the belt's length from there.
    pub origin: Vec<f64>,
    pub length: f64,
    /// Parts pass the origin at `n pitch + arrivals[j]`, n = 0, 1, ...
    pub pitch: f64,
    pub arrivals: Vec<f64>,
    /// The stretch of the belt, from the origin, in which the gripper may close: from its start to its end.
    pub window: [f64; 2],
    /// Added to a grip or place pose for the pose above it, which the tool comes from and lifts to.
    pub lift: Vec<f64>,
    /// Seconds the tool moves with the part while the gripper closes, and with the place while it opens.
    pub grip: f64,
    pub release: f64,
    /// The place pose, and the velocity of the belt it lies on (zero for a fixed place).
    pub place: Vec<f64>,
    pub place_velocity: Vec<f64>,
    /// The verdict `pick-tracking`: the most the tool may be from the part along the belt during a grip, m.
    pub tolerance: f64,
    /// The next part not yet picked or missed, counted from 0.
    next: u64,
    pub picked: u64,
    pub missed: u64,
}

/// One move of a pass, from `start` seconds after the pass began, for `duration`: the move `mv` plus the belt term from
/// `va` to `vb` (see the module).
#[derive(Debug, Clone)]
struct Phase {
    start: f64,
    duration: f64,
    mv: Move,
    va: Vec<f64>,
    vb: Vec<f64>,
}

impl Phase {
    fn at(&self, t: f64, pos: &mut [f64], vel: &mut [f64], acc: &mut [f64]) {
        let tt = self.duration;
        let t = t.clamp(0.0, tt);
        self.mv.at(t, pos, vel, acc);
        let u = if tt > 0.0 { t / tt } else { 1.0 };
        let (g, h, dh) = (u * u * u - u * u * u * u / 2.0, 3.0 * u * u - 2.0 * u * u * u, 6.0 * u - 6.0 * u * u);
        for i in 0..pos.len() {
            let dv = self.vb[i] - self.va[i];
            pos[i] += self.va[i] * t + dv * tt * g;
            vel[i] += self.va[i] + dv * h;
            acc[i] += if tt > 0.0 { dv * dh / tt } else { 0.0 };
        }
    }
}

/// One pick and place, from rest to rest, beginning at the absolute time `start`.
#[derive(Debug, Clone)]
pub struct Pass {
    pub start: f64,
    phases: Vec<Phase>,
    /// The grip, from its start to its end, seconds after the pass began, and the part it grips.
    grip: (f64, f64),
    part: u64,
    /// When the gripper lets go, seconds after the pass began.
    release_end: f64,
}

impl Pass {
    pub fn duration(&self) -> f64 {
        self.phases.last().map_or(0.0, |p| p.start + p.duration)
    }

    /// Tool pose, velocity and acceleration `t` seconds after the pass began.
    pub fn at(&self, t: f64, pos: &mut [f64], vel: &mut [f64], acc: &mut [f64]) {
        let i = self.phases.partition_point(|p| p.start <= t).saturating_sub(1);
        let phase = &self.phases[i];
        phase.at(t - phase.start, pos, vel, acc);
    }

    /// Whether the part is in the gripper `t` seconds after the pass began: from the end of the grip to the end of the
    /// release.
    pub fn holds(&self, t: f64) -> bool {
        t >= self.grip.1 && t < self.release_end
    }
}

fn number(v: &toml::Value) -> Option<f64> {
    v.as_float().or_else(|| v.as_integer().map(|i| i as f64))
}

impl Conveyor {
    /// The conveyor of the config's `[conveyor]` table, for a tool of `axes` axes; `None` without the table or with
    /// `conveyor.enabled = false`.
    pub fn from_config(cfg: &Config, axes: usize) -> Result<Option<Self>> {
        if cfg.get("conveyor").is_err() || !cfg.bool("conveyor.enabled")? {
            return Ok(None);
        }
        let pose = |key: &str| -> Result<Vec<f64>> {
            let path = format!("conveyor.{key}");
            let v = cfg.f64s(&path)?;
            if v.len() == axes && v.iter().all(|x| x.is_finite()) { Ok(v) } else { Err(ConfigError::Invalid(path)) }
        };
        let unit = |key: &str| -> Result<Vec<f64>> {
            let v = pose(key)?;
            let norm = v.iter().map(|x| x * x).sum::<f64>().sqrt();
            if norm > 0.0 {
                Ok(v.iter().map(|x| x / norm).collect())
            } else {
                Err(ConfigError::Invalid(format!("conveyor.{key}")))
            }
        };
        let direction = unit("direction")?;
        let place_direction = unit("place_direction")?;
        let place_speed = cfg.non_negative("conveyor.place_speed_m_s")?;
        let window = cfg.f64s("conveyor.window_m")?;
        let length = cfg.positive("conveyor.length_m")?;
        if window.len() != 2 || !(0.0 <= window[0] && window[0] <= window[1] && window[1] <= length) {
            return Err(ConfigError::Invalid("conveyor.window_m".into()));
        }
        let pitch = cfg.positive("conveyor.pitch_s")?;
        let bad = || ConfigError::Invalid("conveyor.arrivals_s".into());
        let mut arrivals: Vec<f64> =
            cfg.array("conveyor.arrivals_s")?.iter().map(number).collect::<Option<_>>().ok_or_else(bad)?;
        arrivals.sort_by(f64::total_cmp);
        if arrivals.is_empty() || arrivals[0] < 0.0 || arrivals[arrivals.len() - 1] >= pitch {
            return Err(bad());
        }
        Ok(Some(Self {
            direction,
            speed: cfg.non_negative("conveyor.speed_m_s")?,
            ramp: cfg.non_negative("conveyor.ramp_s")?,
            origin: pose("origin")?,
            length,
            pitch,
            arrivals,
            window: [window[0], window[1]],
            lift: pose("lift")?,
            grip: cfg.non_negative("conveyor.grip_s")?,
            release: cfg.non_negative("conveyor.release_s")?,
            place: pose("place")?,
            place_velocity: place_direction.iter().map(|d| d * place_speed).collect(),
            tolerance: cfg.positive("conveyor.tracking_tolerance_m")?,
            next: 0,
            picked: 0,
            missed: 0,
        }))
    }

    /// Goes on where `old` was: its next part, and the parts it picked and missed.
    pub fn carry_on(&mut self, old: &Self) {
        (self.next, self.picked, self.missed) = (old.next, old.picked, old.missed);
    }

    /// How far the belt has run since the start at time `t`.
    pub fn belt(&self, t: f64) -> f64 {
        let t = t.max(0.0);
        if t < self.ramp { self.speed * t * t / (2.0 * self.ramp) } else { self.speed * (t - self.ramp / 2.0) }
    }

    /// The time at which the belt has run `s`; infinite on a belt that stands.
    fn belt_time(&self, s: f64) -> f64 {
        if self.speed <= 0.0 {
            return f64::INFINITY;
        }
        if s < self.speed * self.ramp / 2.0 {
            (2.0 * self.ramp * s / self.speed).sqrt()
        } else {
            s / self.speed + self.ramp / 2.0
        }
    }

    /// When part `k` passes the origin.
    pub fn arrival(&self, k: u64) -> f64 {
        let n = self.arrivals.len() as u64;
        (k / n) as f64 * self.pitch + self.arrivals[(k % n) as usize]
    }

    /// Parts a minute the belt brings.
    pub fn offered_per_minute(&self) -> f64 {
        60.0 * self.arrivals.len() as f64 / self.pitch
    }

    /// Where part `k` is along the belt at time `t`, from the origin (negative before it gets there).
    pub fn position(&self, k: u64, t: f64) -> f64 {
        self.belt(t) - self.belt(self.arrival(k))
    }

    /// The pose that grips part `k` at time `t`.
    pub fn part_pose(&self, k: u64, t: f64) -> Vec<f64> {
        let s = self.position(k, t);
        self.origin.iter().zip(&self.direction).map(|(o, d)| o + d * s).collect()
    }

    fn belt_velocity(&self) -> Vec<f64> {
        self.direction.iter().map(|d| d * self.speed).collect()
    }

    /// The parts on the belt at time `t` that are not in the gripper, with their positions along it: those not yet
    /// planned for, and the one of `pass` until its grip ends. What the scene draws.
    pub fn parts_on_belt(&self, t: f64, pass: Option<&Pass>) -> Vec<(u64, f64)> {
        let waiting = pass.filter(|p| t < p.start + p.grip.1).map(|p| p.part);
        waiting
            .into_iter()
            .chain((self.next..).take_while(|k| self.arrival(*k) <= t).take(64))
            .map(|k| (k, self.position(k, t)))
            .filter(|(_, s)| (0.0..=self.length).contains(s))
            .collect()
    }

    /// The next pass from `from` at rest at time `now`: for the first part that can be gripped inside the window, the
    /// parts before it counted as missed. A part the tool would grip before the window waits for it (the pass starts
    /// later); one it would grip beyond the window, or whose pass leaves `reachable`, is missed. `None` where a
    /// thousand parts in a row are missed: no part can be picked.
    pub fn plan(
        &mut self,
        axes: &[AxisInfo],
        now: f64,
        from: &[f64],
        reachable: &dyn Fn(&[f64]) -> bool,
    ) -> Option<Pass> {
        // shortcut: passes start once the belt has its speed, so the belt term of a pass is that of a constant speed;
        // a pick during the ramp would need the part's acceleration in the intercept.
        let earliest = now.max(self.ramp);
        for _ in 0..1000 {
            let k = self.next;
            self.next += 1;
            let Some(pass) = self.pass_for(axes, k, earliest, from) else {
                self.missed += 1;
                continue;
            };
            let gripped = self.position(k, pass.start + pass.grip.1);
            let samples = (pass.duration() / 0.005).ceil() as usize;
            let n = from.len();
            let (mut pos, mut vel, mut acc) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
            let inside = (0..=samples).all(|i| {
                pass.at(pass.duration() * i as f64 / samples.max(1) as f64, &mut pos, &mut vel, &mut acc);
                reachable(&pos)
            });
            if gripped > self.window[1] + 1e-9 || !inside {
                self.missed += 1;
                continue;
            }
            self.picked += 1;
            return Some(pass);
        }
        None
    }

    /// The pass for part `k`, starting no earlier than `earliest`, and later where the grip would begin before the
    /// window or before the part has arrived.
    fn pass_for(&self, axes: &[AxisInfo], k: u64, earliest: f64, from: &[f64]) -> Option<Pass> {
        let mut pass = self.build(axes, k, earliest, from)?;
        let grip_at = |p: &Pass| p.start + p.grip.0;
        if self.position(k, grip_at(&pass)) < self.window[0] {
            if self.speed <= 0.0 {
                return None;
            }
            // Wait: by the time the part reaches the window's start, a pass started then grips it inside.
            let (mut lo, mut hi) = (earliest, self.belt_time(self.window[0] + self.belt(self.arrival(k))));
            let mut found = self.build(axes, k, hi, from)?;
            for _ in 0..50 {
                let mid = 0.5 * (lo + hi);
                match self.build(axes, k, mid, from) {
                    Some(p) if self.position(k, grip_at(&p)) >= self.window[0] => (hi, found) = (mid, p),
                    _ => lo = mid,
                }
            }
            pass = found;
        }
        // On a belt that stands the part is gripped once it is there.
        let early = self.arrival(k) - grip_at(&pass);
        if early > 0.0 {
            pass = self.build(axes, k, pass.start + early, from)?;
        }
        Some(pass)
    }

    /// The phases of a pass for part `k` starting at `t0` from `from` at rest: to above the part, down onto it, the
    /// grip, up, to above the place, down, the release, up, and, onto a moving place, to rest above the place.
    fn build(&self, axes: &[AxisInfo], k: u64, t0: f64, from: &[f64]) -> Option<Pass> {
        let n = from.len();
        let (zero, belt, placed) = (vec![0.0; n], self.belt_velocity(), self.place_velocity.clone());
        let plus = |a: &[f64], b: &[f64]| -> Vec<f64> { a.iter().zip(b).map(|(x, y)| x + y).collect() };
        let minus = |a: &[f64], b: &[f64]| -> Vec<f64> { a.iter().zip(b).map(|(x, y)| x - y).collect() };
        let mut phases: Vec<Phase> = Vec::new();
        let mut at = from.to_vec();
        let mut t = 0.0;
        // A phase from where the last ended to `target` (where it is at the phase's start, moving with `vb`); `dwell`
        // for a phase of a fixed time that stays with `va`.
        let mut add = |at: &mut Vec<f64>, t: &mut f64, va: &[f64], target: &[f64], vb: &[f64], dwell: Option<f64>| {
            let (duration, mv) = match dwell {
                Some(d) => (d, Move::new(at, at, axes)),
                None => intercept(axes, at, va, target, vb)?,
            };
            let phase = Phase { start: *t, duration, mv, va: va.to_vec(), vb: vb.to_vec() };
            let (mut pos, mut vel, mut acc) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
            phase.at(duration, &mut pos, &mut vel, &mut acc);
            *at = pos;
            *t += duration;
            phases.push(phase);
            Some(())
        };
        let above = plus(&self.part_pose(k, t0), &self.lift);
        add(&mut at, &mut t, &zero, &above, &belt, None)?;
        let part = self.part_pose(k, t0 + t);
        add(&mut at, &mut t, &belt, &part, &belt, None)?;
        let grip_start = t;
        add(&mut at, &mut t, &belt, &[], &belt, Some(self.grip))?;
        let grip_end = t;
        let up = plus(&at, &self.lift);
        add(&mut at, &mut t, &belt, &up, &belt, None)?;
        add(&mut at, &mut t, &belt, &plus(&self.place, &self.lift), &placed, None)?;
        let down = minus(&at, &self.lift);
        add(&mut at, &mut t, &placed, &down, &placed, None)?;
        add(&mut at, &mut t, &placed, &[], &placed, Some(self.release))?;
        let release_end = t;
        let up = plus(&at, &self.lift);
        add(&mut at, &mut t, &placed, &up, &placed, None)?;
        if placed.iter().any(|v| *v != 0.0) {
            add(&mut at, &mut t, &placed, &plus(&self.place, &self.lift), &zero, None)?;
        }
        Some(Pass { start: t0, phases, grip: (grip_start, grip_end), part: k, release_end })
    }

    /// The part in the gripper's grip at time `t` of `pass` (from the grip's start to its end): its pose.
    pub fn gripping(&self, pass: &Pass, t: f64) -> Option<Vec<f64>> {
        let into = t - pass.start;
        (into >= pass.grip.0 && into <= pass.grip.1).then(|| self.part_pose(pass.part, t))
    }
}

/// The shortest time T in which the tool, at `from` with velocity `va`, meets a target that is at `target` now and
/// moves with `vb`, arriving with its velocity: T, and the move to Q = target + T (vb - va) / 2 that the belt term goes
/// on top of (see the module). Searched by bisection on T, since the move's limits depend on T; at once where
/// `va == vb`. `None` where no T up to 1000 s fits, or the belt term alone breaks an axis's limits.
pub fn intercept(axes: &[AxisInfo], from: &[f64], va: &[f64], target: &[f64], vb: &[f64]) -> Option<(f64, Move)> {
    let fits = |t: f64| -> Option<Move> {
        let reduced: Vec<AxisInfo> = axes
            .iter()
            .enumerate()
            .map(|(i, a)| {
                let dv = (vb[i] - va[i]).abs();
                let blend = |limit: f64| if dv > 0.0 { limit } else { 0.0 };
                AxisInfo {
                    max_vel: a.max_vel - va[i].abs().max(vb[i].abs()),
                    max_acc: a.max_acc - blend(1.5 * dv / t),
                    max_jerk: a.max_jerk - blend(6.0 * dv / (t * t)),
                    ..a.clone()
                }
            })
            .collect();
        if reduced.iter().any(|a| a.max_vel <= 0.0 || a.max_acc <= 0.0 || a.max_jerk <= 0.0) {
            return None;
        }
        // Where the velocities agree the shift is none, also for the T = infinity of `va == vb`.
        let q: Vec<f64> =
            (0..from.len()).map(|i| target[i] + if vb[i] == va[i] { 0.0 } else { t * (vb[i] - va[i]) / 2.0 }).collect();
        let mv = Move::new(from, &q, &reduced);
        (mv.duration() <= t).then_some(mv)
    };
    if va == vb {
        // The belt term moves at a constant velocity: the move takes its own time, which then is T.
        let mv = fits(f64::INFINITY)?;
        return Some((mv.duration(), mv));
    }
    let mut hi = 1e-3;
    while fits(hi).is_none() {
        hi *= 2.0;
        if hi > 1000.0 {
            return None;
        }
    }
    let mut lo = hi / 2.0;
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if fits(mid).is_some() { hi = mid } else { lo = mid }
    }
    Some((hi, fits(hi)?))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axis(max_vel: f64, max_acc: f64, max_jerk: f64) -> AxisInfo {
        AxisInfo { id: "x", key: "", unit: "m", min: -10.0, max: 10.0, max_vel, max_acc, max_jerk }
    }

    /// A belt along the one axis at 0.5 m/s, a part every 2 s, the window from 0.5 to 1.0 m, the place at -0.5 m.
    fn conveyor() -> Conveyor {
        let cfg = Config::parse(
            "[conveyor]\nenabled = true\ndirection = [1.0]\nspeed_m_s = 0.5\nramp_s = 0.0\norigin = [0.0]\n\
             length_m = 2.0\npitch_s = 2.0\narrivals_s = [0.0]\nwindow_m = [0.5, 1.0]\nlift = [0.0]\ngrip_s = 0.1\n\
             release_s = 0.1\nplace = [-0.5]\nplace_direction = [1.0]\nplace_speed_m_s = 0.0\n\
             tracking_tolerance_m = 0.001\n",
        )
        .unwrap();
        Conveyor::from_config(&cfg, 1).unwrap().unwrap()
    }

    /// The intercept against the closed form of a trapezoidal move (jerk so high that the S-curve is one). The tool at
    /// 0 at rest meets a part at D0 = 1 m moving at v = 0.5 m/s, with v_max = 2 m/s and a = 20 m/s^2. The move goes to
    /// D = D0 + v T / 2 with v' = v_max - v = 1.5 m/s and a' = a - 1.5 v / T, so with cruise T = D / v' + v' / a', which
    /// is the quadratic a alpha T^2 - (alpha c + a beta + v') T + beta c = 0, alpha = 1 - v / (2 v'), beta = D0 / v',
    /// c = 1.5 v: T = 0.89395 s.
    #[test]
    fn the_intercept_of_a_trapezoidal_move_is_the_closed_form() {
        let (vm, a, v, d0): (f64, f64, f64, f64) = (2.0, 20.0, 0.5, 1.0);
        let vp = vm - v;
        let (alpha, beta, c) = (1.0 - v / (2.0 * vp), d0 / vp, 1.5 * v);
        let (qa, qb, qc) = (a * alpha, alpha * c + a * beta + vp, beta * c);
        let expected = (qb + (qb * qb - 4.0 * qa * qc).sqrt()) / (2.0 * qa);
        assert!((expected - 0.89395).abs() < 1e-4, "{expected}");
        let (t, mv) = intercept(&[axis(vm, a, 1e9)], &[0.0], &[0.0], &[d0], &[v]).unwrap();
        assert!((t - expected).abs() < 1e-5, "{t} against {expected}");
        // The move ends where the part is less what the belt term covers, v T / 2.
        let (mut p, mut vel, mut acc) = ([0.0], [0.0], [0.0]);
        mv.at(f64::INFINITY, &mut p, &mut vel, &mut acc);
        assert!((p[0] - (d0 + v * t / 2.0)).abs() < 1e-12);
    }

    /// The tool meets the part where it is, with its speed, and stays within the axis's limits on the way.
    #[test]
    fn the_tool_arrives_on_the_part_with_the_belts_speed_within_its_limits() {
        let ax = [axis(2.0, 20.0, 400.0)];
        let (t, mv) = intercept(&ax, &[0.0], &[0.0], &[0.3], &[0.5]).unwrap();
        let phase = Phase { start: 0.0, duration: t, mv, va: vec![0.0], vb: vec![0.5] };
        let (mut p, mut v, mut a) = ([0.0], [0.0], [0.0]);
        for i in 0..=1000 {
            phase.at(t * i as f64 / 1000.0, &mut p, &mut v, &mut a);
            assert!(v[0].abs() <= 2.0 + 1e-9 && a[0].abs() <= 20.0 + 1e-9, "{v:?} {a:?}");
        }
        phase.at(t, &mut p, &mut v, &mut a);
        assert!((p[0] - (0.3 + 0.5 * t)).abs() < 1e-9 && (v[0] - 0.5).abs() < 1e-12 && a[0].abs() < 1e-12);
    }

    /// During the grip the plan moves with the belt: its speed along the belt is the belt's, 0.5 m/s, and it is on the
    /// part.
    #[test]
    fn during_the_grip_the_plan_moves_with_the_part() {
        let mut c = conveyor();
        let pass = c.plan(&[axis(2.0, 20.0, 400.0)], 0.0, &[-0.5], &|_| true).unwrap();
        let (mut p, mut v, mut a) = ([0.0], [0.0], [0.0]);
        for i in 0..=10 {
            let t = pass.grip.0 + (pass.grip.1 - pass.grip.0) * i as f64 / 10.0;
            pass.at(t, &mut p, &mut v, &mut a);
            assert!((v[0] - 0.5).abs() < 1e-9, "{v:?}");
            let part = c.gripping(&pass, pass.start + t).unwrap();
            assert!((p[0] - part[0]).abs() < 1e-9, "{} against {}", p[0], part[0]);
        }
        // Part 0 passes the origin at 0 and is gripped as it reaches the window's start, 0.5 m on: 1 s in.
        assert!((pass.start + pass.grip.0 - 1.0).abs() < 1e-6, "{}", pass.start + pass.grip.0);
        assert_eq!((c.picked, c.missed), (1, 0));
    }

    /// A part the tool cannot grip before it leaves the window is missed. Parts pass the origin at 0, 2 and 4 s; at
    /// 5 s, where the tool starts, they are 2.5, 1.5 and 0.5 m along the belt. The window ends at 1.0 m: parts 0 and 1
    /// are missed, part 2 is picked inside it.
    #[test]
    fn a_part_beyond_the_window_counts_as_missed() {
        let mut c = conveyor();
        let pass = c.plan(&[axis(2.0, 20.0, 400.0)], 5.0, &[-0.5], &|_| true).unwrap();
        assert_eq!((pass.part, c.missed, c.picked), (2, 2, 1));
        let s = c.position(pass.part, pass.start + pass.grip.1);
        assert!((0.5..=1.0).contains(&s), "{s}");
    }

    /// A pass whose path leaves what the machine reaches is not run: its part counts as missed too.
    #[test]
    fn a_part_out_of_reach_counts_as_missed() {
        let mut c = conveyor();
        // Nothing at or behind -0.4 m is reachable, and the place at -0.5 m lies there: no part can be placed. Every
        // part is missed, and after a thousand in a row the planner gives up.
        assert!(c.plan(&[axis(2.0, 20.0, 400.0)], 0.0, &[-0.5], &|p: &[f64]| p[0] > -0.4).is_none());
        assert_eq!((c.picked, c.missed), (0, 1000));
    }
}
