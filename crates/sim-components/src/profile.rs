//! Motion profiles: the jerk-limited seven-segment S-curve, the sin² profile (Bestehorn), the polynomial motion laws
//! 3-4-5 and 4-5-6-7 of VDI 2143, braking slower or faster than speeding up, and synchronised multi-axis moves.
//!
//! A profile goes from rest to rest over a distance. The S-curve and the sin² profile speed up on a ramp, cruise and
//! brake on a ramp of their own (`Profile::Ramps`), each ramp within its own acceleration limit; the polynomial laws
//! run the whole distance in one law with no cruise. Any profile can be stretched in time (`Profile::stretched`):
//! speed, acceleration and jerk limits scaled by 1/k, 1/k^2 and 1/k^3 give the same shape k times as long, which is how
//! a planner synchronises moves to a common duration.

use sim_core::AxisInfo;

/// The shape of a move from rest to rest (`profile.shape`, or `shape` of a `[[cycle.segment]]`).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum Shape {
    /// Seven segments: jerk up, constant acceleration, jerk down, cruise, and the same braking.
    #[default]
    SCurve,
    /// The speed rises as sin² of time over the ramp (Bestehorn): peak acceleration (pi/2) v / T_ramp, peak jerk
    /// (pi^2/2) v / T_ramp^2, at the ramp's ends.
    SinSquared,
    /// s = 10 u^3 - 15 u^4 + 6 u^5 over the whole move (VDI 2143): speed and acceleration zero at both ends.
    Poly345,
    /// s = 35 u^4 - 84 u^5 + 70 u^6 - 20 u^7: jerk zero at both ends too.
    Poly4567,
}

impl Shape {
    pub const NAMES: [&str; 4] = ["s-curve", "sin2", "poly345", "poly4567"];

    pub fn from_name(name: &str) -> Option<Self> {
        [Self::SCurve, Self::SinSquared, Self::Poly345, Self::Poly4567].into_iter().find(|s| s.name() == name)
    }

    pub fn name(&self) -> &'static str {
        Self::NAMES[*self as usize]
    }

    /// For a polynomial law: the peaks of speed, acceleration and jerk over a move of 1 in a time of 1 (multiply by
    /// h/T, h/T^2 and h/T^3). 3-4-5: s' = 30 u^2 (1-u)^2, 1.875 at u = 1/2; s'' = 60 u (1-u) (1-2u), 10/sqrt 3 at
    /// u = (1 - 1/sqrt 3)/2; s''' 60 at the ends. 4-5-6-7: s' = 140 u^3 (1-u)^3, 2.1875; s'' = 420 u^2 (1-u)^2 (1-2u),
    /// with x = u - 1/2 largest at x^2 = 1/20: 420 * 2 / (25 sqrt 20) = 7.5132; s''' = 840 u (1-u) (1 - 5u + 5u^2),
    /// 52.5 at u = 1/2. `None` for the shapes with a cruise.
    pub fn peaks(&self) -> Option<[f64; 3]> {
        match self {
            Self::Poly345 => Some([1.875, 10.0 / 3f64.sqrt(), 60.0]),
            Self::Poly4567 => Some([2.1875, 33.6 / 20f64.sqrt(), 52.5]),
            _ => None,
        }
    }

    /// For a polynomial law: (s, s', s'', s''') at u in [0, 1] over a move of 1 in a time of 1.
    pub fn law(&self, u: f64) -> [f64; 4] {
        let u = u.clamp(0.0, 1.0);
        let (u2, u3) = (u * u, u * u * u);
        match self {
            Self::Poly4567 => [
                u3 * u * (35.0 - 84.0 * u + 70.0 * u2 - 20.0 * u3),
                140.0 * u3 * (1.0 - u).powi(3),
                420.0 * u2 * (1.0 - u).powi(2) * (1.0 - 2.0 * u),
                840.0 * u * (1.0 - u) * (1.0 - 5.0 * u + 5.0 * u2),
            ],
            _ => [
                u3 * (10.0 - 15.0 * u + 6.0 * u2),
                30.0 * u2 * (1.0 - u).powi(2),
                60.0 * u * (1.0 - u) * (1.0 - 2.0 * u),
                60.0 - 360.0 * u + 360.0 * u2,
            ],
        }
    }
}

/// The limits of a profile: speed, acceleration while speeding up, deceleration while braking (both positive), jerk.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    pub vel: f64,
    pub acc: f64,
    pub dec: f64,
    pub jerk: f64,
}

/// A ramp from rest to a speed `speed` in `time`: an S-curve's (jerk for `tj`, constant acceleration, jerk back for
/// `tj`) or a sin²'s. It covers speed * time / 2 (both are symmetric about their middle).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Ramp {
    sine: bool,
    time: f64,
    tj: f64,
}

impl Ramp {
    /// The shortest ramp to `speed` within the acceleration `acc` and the jerk `jerk`.
    fn new(sine: bool, speed: f64, acc: f64, jerk: f64) -> Self {
        if sine {
            let pi = std::f64::consts::PI;
            let time = (pi * speed / (2.0 * acc)).max(pi * (speed / (2.0 * jerk)).sqrt());
            return Self { sine, time, tj: 0.0 };
        }
        // As SCurve::new: the acceleration limit reached, or not.
        if speed * jerk < acc * acc {
            let tj = (speed / jerk).sqrt();
            Self { sine, time: 2.0 * tj, tj }
        } else {
            Self { sine, time: acc / jerk + speed / acc, tj: acc / jerk }
        }
    }

    /// (position, velocity, acceleration, jerk) at t in [0, time] on the way to `speed`.
    fn at(&self, speed: f64, t: f64) -> [f64; 4] {
        let tt = self.time;
        let t = t.clamp(0.0, tt);
        if tt <= 0.0 {
            return [0.0, speed, 0.0, 0.0];
        }
        if self.sine {
            let w = std::f64::consts::PI / tt;
            return [
                speed * (t / 2.0 - (w * t).sin() / (2.0 * w)),
                speed * (w * t / 2.0).sin().powi(2),
                speed * w / 2.0 * (w * t).sin(),
                speed * w * w / 2.0 * (w * t).cos(),
            ];
        }
        let (tj, ta) = (self.tj, tt - 2.0 * self.tj);
        let j = speed / (tj * (tj + ta));
        let (mut p, mut v, mut a, mut left) = (0.0, 0.0, 0.0, t);
        for (duration, jerk) in [(tj, j), (ta, 0.0), (tj, -j)] {
            let h = left.min(duration);
            p += v * h + a * h * h / 2.0 + jerk * h * h * h / 6.0;
            v += a * h + jerk * h * h / 2.0;
            a += jerk * h;
            left -= h;
            if left <= 0.0 {
                return [p, v, a, jerk];
            }
        }
        [p, v, a, 0.0]
    }
}

/// A move from rest to rest over `dist` in one of the shapes.
#[derive(Debug, Clone)]
pub enum Profile {
    /// The S-curve with the same acceleration and braking, in closed form.
    SCurve(SCurve),
    /// Speeding up to `speed`, cruising for `cruise` seconds, braking: the S-curve with braking of its own, and sin².
    Ramps { dist: f64, speed: f64, up: Ramp, cruise: f64, down: Ramp },
    /// A polynomial law over the whole move, `time` long.
    Law { dist: f64, shape: Shape, time: f64 },
}

impl Default for Profile {
    fn default() -> Self {
        Self::SCurve(SCurve::default())
    }
}

impl Profile {
    /// The shortest move over `dist` in `shape` within `limits`. A polynomial law, which has no separate braking, keeps
    /// the lower of the acceleration and the deceleration.
    pub fn new(dist: f64, shape: Shape, limits: Limits) -> Self {
        let Limits { vel, acc, dec, jerk } = limits;
        if dist <= 0.0 {
            return Self::default();
        }
        if let Some([cv, ca, cj]) = shape.peaks() {
            let time = (cv * dist / vel).max((ca * dist / acc.min(dec)).sqrt()).max((cj * dist / jerk).cbrt());
            return Self::Law { dist, shape, time };
        }
        let sine = shape == Shape::SinSquared;
        if !sine && acc == dec {
            return Self::SCurve(SCurve::new(dist, vel, acc, jerk));
        }
        let ramps = |v: f64| (Ramp::new(sine, v, acc, jerk), Ramp::new(sine, v, dec, jerk));
        let covered = |v: f64| {
            let (up, down) = ramps(v);
            v * (up.time + down.time) / 2.0
        };
        // The cruise speed: the limit where the ramps fit the distance, else the speed at which they cover it alone
        // (what they cover rises with the speed).
        let speed = if covered(vel) <= dist {
            vel
        } else {
            let (mut lo, mut hi) = (0.0, vel);
            for _ in 0..200 {
                let mid = 0.5 * (lo + hi);
                if covered(mid) <= dist { lo = mid } else { hi = mid }
            }
            lo
        };
        let (up, down) = ramps(speed);
        // What the ramps leave is cruised: on a short move a rounding of the bisection.
        let cruise = if speed > 0.0 { ((dist - covered(speed)) / speed).max(0.0) } else { 0.0 };
        Self::Ramps { dist, speed, up, cruise, down }
    }

    /// Time scaling: the move over `dist` in `shape` stretched to `duration` (no shorter than the shortest within
    /// `limits`), with the limits scaled by 1/k, 1/k^2 and 1/k^3 for k = duration / shortest: the same shape, slower.
    pub fn stretched(dist: f64, shape: Shape, limits: Limits, duration: f64) -> Self {
        let shortest = Self::new(dist, shape, limits);
        let k = duration / shortest.duration();
        if !(k > 1.0 && k.is_finite()) {
            return shortest;
        }
        let Limits { vel, acc, dec, jerk } = limits;
        Self::new(dist, shape, Limits { vel: vel / k, acc: acc / (k * k), dec: dec / (k * k), jerk: jerk / k.powi(3) })
    }

    pub fn dist(&self) -> f64 {
        match self {
            Self::SCurve(c) => c.dist,
            Self::Ramps { dist, .. } | Self::Law { dist, .. } => *dist,
        }
    }

    pub fn duration(&self) -> f64 {
        match self {
            Self::SCurve(c) => c.duration(),
            Self::Ramps { up, cruise, down, .. } => up.time + cruise + down.time,
            Self::Law { time, .. } => *time,
        }
    }

    /// Time to speed up to the highest speed (the first half of a polynomial law).
    pub fn ramp(&self) -> f64 {
        match self {
            Self::SCurve(c) => c.ramp(),
            Self::Ramps { up, .. } => up.time,
            Self::Law { time, .. } => time / 2.0,
        }
    }

    /// Time to brake from the highest speed.
    pub fn brake(&self) -> f64 {
        match self {
            Self::SCurve(c) => c.ramp(),
            Self::Ramps { down, .. } => down.time,
            Self::Law { time, .. } => time / 2.0,
        }
    }

    /// (position, velocity, acceleration, jerk) at time t.
    pub fn at(&self, t: f64) -> (f64, f64, f64, f64) {
        let end = (self.dist(), 0.0, 0.0, 0.0);
        if t >= self.duration() {
            return end;
        }
        let t = t.max(0.0);
        match self {
            Self::SCurve(c) => c.at(t),
            Self::Law { dist, shape, time } => {
                let [s, v, a, j] = shape.law(t / time);
                (dist * s, dist * v / time, dist * a / (time * time), dist * j / time.powi(3))
            }
            Self::Ramps { speed, up, cruise, down, .. } => {
                if t < up.time {
                    let [p, v, a, j] = up.at(*speed, t);
                    return (p, v, a, j);
                }
                let at_cruise = speed * up.time / 2.0;
                let t = t - up.time;
                if t < *cruise {
                    return (at_cruise + speed * t, *speed, 0.0, 0.0);
                }
                let t = t - cruise;
                let [p, v, a, j] = down.at(*speed, t);
                (at_cruise + speed * cruise + speed * t - p, speed - v, -a, -j)
            }
        }
    }
}

/// How the moves of a machine are shaped: the shape, the braking limit as a share of the acceleration limit
/// (`profile.deceleration_ratio`), and an axis that lifts against gravity (`profile.vertical_axis`): its index, the
/// sign of up along it, and the share of its acceleration limit it has upwards (`profile.up_acceleration_ratio`).
#[derive(Debug, Clone, PartialEq)]
pub struct MotionLaw {
    pub shape: Shape,
    pub deceleration_ratio: f64,
    pub vertical: Option<(usize, f64, f64)>,
}

impl Default for MotionLaw {
    fn default() -> Self {
        Self { shape: Shape::SCurve, deceleration_ratio: 1.0, vertical: None }
    }
}

/// Seven-segment S-curve over a distance, from rest to rest:
/// jerk up, constant acceleration, jerk down, cruise, and the mirror image.
#[derive(Debug, Clone, Default)]
pub struct SCurve {
    pub dist: f64,
    jerk: f64,
    /// Duration of each jerk segment, of each constant-acceleration segment, and of the cruise.
    tj: f64,
    ta: f64,
    tv: f64,
}

impl SCurve {
    pub fn new(dist: f64, max_vel: f64, max_acc: f64, max_jerk: f64) -> Self {
        if dist <= 0.0 {
            return Self::default();
        }
        let (v, a, j) = (max_vel, max_acc, max_jerk);
        // Ramp to cruise speed: with or without reaching the acceleration limit.
        let (mut tj, mut ta) = if v * j < a * a { ((v / j).sqrt(), 0.0) } else { (a / j, v / a - a / j) };
        let peak = |tj: f64, ta: f64| j * tj * (tj + ta);
        let ramps = |tj: f64, ta: f64| peak(tj, ta) * (2.0 * tj + ta);
        let mut tv = 0.0;
        if ramps(tj, ta) > dist {
            // Too short to reach cruise speed: lower the peak speed.
            let tja = a / j;
            if dist >= 2.0 * j * tja.powi(3) {
                tj = tja;
                ta = 0.5 * (-3.0 * tj + (tj * tj + 4.0 * dist / a).sqrt());
            } else {
                tj = (dist / (2.0 * j)).cbrt();
                ta = 0.0;
            }
        } else {
            tv = (dist - ramps(tj, ta)) / peak(tj, ta);
        }
        Self { dist, jerk: j, tj, ta, tv }
    }

    pub fn duration(&self) -> f64 {
        4.0 * self.tj + 2.0 * self.ta + self.tv
    }

    /// Time to accelerate to cruise speed, which is also the time to brake from it.
    pub fn ramp(&self) -> f64 {
        2.0 * self.tj + self.ta
    }

    /// (position, velocity, acceleration, jerk) at time t.
    pub fn at(&self, t: f64) -> (f64, f64, f64, f64) {
        if t >= self.duration() {
            return (self.dist, 0.0, 0.0, 0.0);
        }
        let j = self.jerk;
        let segments =
            [(self.tj, j), (self.ta, 0.0), (self.tj, -j), (self.tv, 0.0), (self.tj, -j), (self.ta, 0.0), (self.tj, j)];
        let (mut p, mut v, mut a, mut left) = (0.0, 0.0, 0.0, t.max(0.0));
        for (duration, jerk) in segments {
            let h = left.min(duration);
            p += v * h + a * h * h / 2.0 + jerk * h * h * h / 6.0;
            v += a * h + jerk * h * h / 2.0;
            a += jerk * h;
            left -= h;
            if left <= 0.0 {
                return (p, v, a, jerk);
            }
        }
        (self.dist, 0.0, 0.0, 0.0)
    }
}

/// A straight-line move of all axes that start and finish together.
/// One profile runs the path from 0 to 1; its limits are the tightest of all axes.
#[derive(Debug, Clone)]
pub struct Move {
    start: Vec<f64>,
    target: Vec<f64>,
    delta: Vec<f64>,
    // Boxed: two moves sit in `control::Motion`, whose other variants are small.
    curve: Box<Profile>,
}

impl Move {
    /// The S-curve within the axes' limits, braking as it speeds up.
    pub fn new(start: &[f64], target: &[f64], axes: &[AxisInfo]) -> Self {
        Self::with_law(start, target, axes, &MotionLaw::default())
    }

    /// The move in `law`. Speeding up, each axis accelerates towards its target; braking, away from it. The vertical
    /// axis has `up_ratio` of its limit where that is upwards, and braking has `deceleration_ratio` of every limit.
    pub fn with_law(start: &[f64], target: &[f64], axes: &[AxisInfo], law: &MotionLaw) -> Self {
        let delta: Vec<f64> = target.iter().zip(start).map(|(t, s)| t - s).collect();
        let tightest = |limit: &dyn Fn(usize, &AxisInfo, f64) -> f64| {
            axes.iter()
                .enumerate()
                .zip(&delta)
                .filter(|(_, d)| **d != 0.0)
                .map(|((i, a), d)| limit(i, a, d.signum()) / d.abs())
                .fold(f64::INFINITY, f64::min)
        };
        // The acceleration limit of axis i for an acceleration in the direction `sign`.
        let acc = |i: usize, a: &AxisInfo, sign: f64| match law.vertical {
            Some((axis, up, ratio)) if axis == i && sign == up => a.max_acc * ratio,
            _ => a.max_acc,
        };
        let moving = delta.iter().any(|d| *d != 0.0);
        let curve = if moving {
            let limits = Limits {
                vel: tightest(&|_, a, _| a.max_vel),
                acc: tightest(&|i, a, sign| acc(i, a, sign)),
                dec: tightest(&|i, a, sign| acc(i, a, -sign) * law.deceleration_ratio),
                jerk: tightest(&|_, a, _| a.max_jerk),
            };
            Profile::new(1.0, law.shape, limits)
        } else {
            Profile::default()
        };
        Self { start: start.to_vec(), target: target.to_vec(), delta, curve: Box::new(curve) }
    }

    pub fn duration(&self) -> f64 {
        self.curve.duration()
    }

    /// Time to speed up to the highest speed.
    pub fn ramp(&self) -> f64 {
        self.curve.ramp()
    }

    /// Time to brake from the highest speed.
    pub fn brake(&self) -> f64 {
        self.curve.brake()
    }

    pub fn delta(&self) -> &[f64] {
        &self.delta
    }

    pub fn at(&self, t: f64, pos: &mut [f64], vel: &mut [f64], acc: &mut [f64]) {
        let (s, v, a, _) = if self.curve.dist() > 0.0 { self.curve.at(t) } else { (0.0, 0.0, 0.0, 0.0) };
        for i in 0..pos.len() {
            // At the end the target itself: start + (target - start) can be a rounding beyond it, and beyond the end of
            // a stroke, where a path planned from there was refused (2.1000000000000005 rad for 2.1 on an arm's joint).
            pos[i] = if s >= 1.0 { self.target[i] } else { self.start[i] + self.delta[i] * s };
            vel[i] = self.delta[i] * v;
            acc[i] = self.delta[i] * a;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Profile limits: velocity, acceleration and jerk never exceed their limits,
    /// in every regime (cruise reached or not, acceleration limit reached or not).
    #[test]
    fn velocity_acceleration_and_jerk_stay_within_limits() {
        let cases = [
            (1.0, 0.8, 8.0, 100.0),   // all seven segments
            (0.05, 0.8, 8.0, 100.0),  // no cruise, acceleration limit reached
            (0.001, 0.8, 8.0, 100.0), // no cruise, acceleration limit not reached
            (1.0, 0.2, 8.0, 100.0),   // cruise, acceleration limit not reached
            (0.3, 2.0, 5.0, 20.0),
            (1e-6, 1.0, 10.0, 100.0),
        ];
        for (dist, v, a, j) in cases {
            let c = SCurve::new(dist, v, a, j);
            let (n, t_end) = (20_000, c.duration());
            let (mut v_peak, mut a_peak) = (0f64, 0f64);
            let mut last = c.at(0.0);
            for i in 1..=n {
                let now = c.at(t_end * i as f64 / n as f64);
                let h = t_end / n as f64;
                assert!(now.1 >= -1e-12 && now.1 <= v * (1.0 + 1e-9), "{dist}: v {}", now.1);
                assert!(now.2.abs() <= a * (1.0 + 1e-9), "{dist}: a {}", now.2);
                assert!(now.3.abs() <= j, "{dist}: j {}", now.3);
                // The reported derivatives are the real ones: check them against differences.
                assert!(
                    (now.0 - last.0 - 0.5 * (now.1 + last.1) * h).abs() <= 1e-9 * dist + j * h.powi(3),
                    "{dist}: p/v"
                );
                assert!(((now.2 - last.2) / h).abs() <= j * (1.0 + 1e-6), "{dist}: da/dt {}", (now.2 - last.2) / h);
                v_peak = v_peak.max(now.1);
                a_peak = a_peak.max(now.2.abs());
                last = now;
            }
            let end = c.at(t_end);
            assert!((end.0 - dist).abs() < 1e-12 && end.1 == 0.0 && end.2 == 0.0);
            let before_end = c.at(t_end * (1.0 - 1e-9));
            assert!((before_end.0 - dist).abs() < 1e-9 * dist && before_end.1.abs() < 1e-6 * v);
            assert!(v_peak > 0.0 && a_peak > 0.0);
        }
    }

    #[test]
    fn long_moves_use_the_limits_fully() {
        let c = SCurve::new(1.0, 0.8, 8.0, 100.0);
        let mid = c.at(c.duration() / 2.0);
        assert!((mid.1 - 0.8).abs() < 1e-12 && mid.2.abs() < 1e-12);
        // Hand calculation: tj = a/j = 0.08, ta = v/a - tj = 0.02, ramps cover v (2 tj + ta) = 0.144 m.
        assert!((c.duration() - (4.0 * 0.08 + 2.0 * 0.02 + (1.0 - 0.144) / 0.8)).abs() < 1e-12);
    }

    fn axis(max_vel: f64, max_acc: f64) -> AxisInfo {
        AxisInfo { id: "", key: "", unit: "m", min: 0.0, max: 1.0, max_vel, max_acc, max_jerk: 100.0 }
    }

    /// Synchronised moves: both axes finish together, on a straight line, each within its limits.
    #[test]
    fn synchronised_axes_finish_together_within_their_limits() {
        let axes = [axis(0.8, 8.0), axis(0.4, 6.0)];
        let m = Move::new(&[0.1, 0.1], &[0.9, 0.4], &axes);
        let (mut p, mut v, mut a) = ([0.0; 2], [0.0; 2], [0.0; 2]);
        let n = 5000;
        let mut peak_v = [0f64; 2];
        for i in 0..=n {
            m.at(m.duration() * i as f64 / n as f64, &mut p, &mut v, &mut a);
            for k in 0..2 {
                assert!(v[k].abs() <= axes[k].max_vel * (1.0 + 1e-9) && a[k].abs() <= axes[k].max_acc * (1.0 + 1e-9));
                peak_v[k] = peak_v[k].max(v[k]);
            }
            assert!(((p[0] - 0.1) * 0.3 - (p[1] - 0.1) * 0.8).abs() < 1e-12, "not on the straight line");
        }
        assert!((p[0] - 0.9).abs() < 1e-12 && (p[1] - 0.4).abs() < 1e-12 && v == [0.0; 2]);
        // X is the limiting axis here (0.8 m at 0.8 m/s against 0.3 m at 0.4 m/s).
        assert!((peak_v[0] - 0.8).abs() < 1e-9 && (peak_v[1] - 0.3).abs() < 1e-9, "{peak_v:?}");
    }

    /// Peaks of a profile over its duration, sampled: (speed, acceleration, deceleration, jerk), and its end.
    fn peaks(p: &Profile) -> ([f64; 4], (f64, f64, f64, f64)) {
        let (n, t_end) = (100_000, p.duration());
        let mut peak = [0f64; 4];
        for i in 0..=n {
            let (_, v, a, j) = p.at(t_end * i as f64 / n as f64 * (1.0 - 1e-12));
            peak = [peak[0].max(v), peak[1].max(a), peak[2].max(-a), peak[3].max(j.abs())];
        }
        (peak, p.at(t_end))
    }

    /// sin² (Bestehorn): the speed rises as v sin^2(pi t / (2 T)), so the acceleration v pi / (2 T) sin(pi t / T) peaks
    /// at (pi/2) v / T. With v = 0.8 m/s, a = 8 m/s^2 and a jerk of 1000 m/s^3 the ramp is the longer of
    /// pi v / (2 a) = 0.15708 s and pi sqrt(v / (2 j)) = 0.06283 s: 0.15708 s, and the peak (pi/2) 0.8 / 0.15708 = 8 m/s^2.
    #[test]
    fn the_sin2_profile_peaks_at_pi_half_v_over_its_ramp() {
        let limits = Limits { vel: 0.8, acc: 8.0, dec: 8.0, jerk: 1000.0 };
        let p = Profile::new(1.0, Shape::SinSquared, limits);
        let t_acc = p.ramp();
        assert!((t_acc - std::f64::consts::PI * 0.8 / 16.0).abs() < 1e-12, "{t_acc}");
        let (peak, end) = peaks(&p);
        assert!((peak[0] - 0.8).abs() < 1e-9, "{peak:?}");
        assert!((peak[1] - std::f64::consts::FRAC_PI_2 * 0.8 / t_acc).abs() < 1e-6 && (peak[1] - 8.0).abs() < 1e-6);
        // The ramps cover v T each half: 0.8 * 0.15708 = 0.12566 m, the rest is cruised at 0.8 m/s.
        assert!((p.duration() - (2.0 * t_acc + (1.0 - 0.8 * t_acc) / 0.8)).abs() < 1e-12);
        assert!((end.0 - 1.0).abs() < 1e-15 && end.1 == 0.0);
        // A short move does not reach the speed: ramps alone, v T(v) = dist.
        let short = Profile::new(0.01, Shape::SinSquared, limits);
        let (peak, end) = peaks(&short);
        assert!(peak[0] < 0.8 && peak[1] <= 8.0 * (1.0 + 1e-9) && peak[3] <= 1000.0 * (1.0 + 1e-9), "{peak:?}");
        assert!((end.0 - 0.01).abs() < 1e-15);
    }

    /// 3-4-5 (VDI 2143): s = h (10 u^3 - 15 u^4 + 6 u^5), u = t / T. Its speed 30 h u^2 (1 - u)^2 / T peaks at u = 1/2 at
    /// 30/16 h / T = 1.875 h / T. Speed bound here: h = 0.3 m at 0.5 m/s takes T = 1.875 * 0.3 / 0.5 = 1.125 s (the
    /// acceleration's sqrt(5.7735 * 0.3 / 8) = 0.465 s and the jerk's cbrt(60 * 0.3 / 1000) = 0.262 s are shorter).
    #[test]
    fn the_345_law_peaks_at_1_875_h_over_t() {
        let p = Profile::new(0.3, Shape::Poly345, Limits { vel: 0.5, acc: 8.0, dec: 8.0, jerk: 1000.0 });
        assert!((p.duration() - 1.125).abs() < 1e-12, "{}", p.duration());
        let (peak, end) = peaks(&p);
        assert!((peak[0] - 1.875 * 0.3 / 1.125).abs() < 1e-9 && (peak[0] - 0.5).abs() < 1e-9, "{peak:?}");
        // Acceleration 10 / sqrt 3 h / T^2 = 5.7735 * 0.3 / 1.265625 = 1.3685 m/s^2, jerk 60 h / T^3 = 12.642 m/s^3.
        assert!((peak[1] - 5.773_502_7 * 0.3 / 1.125f64.powi(2)).abs() < 1e-6, "{peak:?}");
        assert!((peak[3] - 60.0 * 0.3 / 1.125f64.powi(3)).abs() < 1e-6, "{peak:?}");
        assert!((end.0 - 0.3).abs() < 1e-15 && end.1 == 0.0 && end.2 == 0.0);
        // Velocity is the derivative of position (central difference at u = 0.3).
        let (t, h) = (0.3 * 1.125, 1e-6);
        assert!(((p.at(t + h).0 - p.at(t - h).0) / (2.0 * h) - p.at(t).1).abs() < 1e-6);
    }

    /// 4-5-6-7: speed 140 h u^3 (1-u)^3 / T, 2.1875 h / T at u = 1/2; acceleration 7.5132 h / T^2; jerk 52.5 h / T^3,
    /// zero at both ends. Acceleration bound: h = 0.1 m at 2 m/s^2 takes sqrt(7.5132 * 0.1 / 2) = 0.6129 s.
    #[test]
    fn the_4567_law_has_no_jerk_at_its_ends() {
        let p = Profile::new(0.1, Shape::Poly4567, Limits { vel: 5.0, acc: 2.0, dec: 3.0, jerk: 1000.0 });
        let t = (33.6 / 20f64.sqrt() * 0.1 / 2.0).sqrt();
        assert!((p.duration() - t).abs() < 1e-6, "{}", p.duration());
        let (peak, _) = peaks(&p);
        assert!((peak[0] - 2.1875 * 0.1 / t).abs() < 1e-6 && (peak[1] - 2.0).abs() < 1e-5, "{peak:?}");
        assert!((peak[3] - 52.5 * 0.1 / t.powi(3)).abs() < 1e-4, "{peak:?}");
        assert!(p.at(0.0).3.abs() < 1e-12 && p.at(t * (1.0 - 1e-9)).3.abs() < 1e-3);
    }

    /// Braking at half the acceleration: v = 0.8 m/s, a = 8, d = 4 m/s^2, j = 100 m/s^3. Speeding up reaches the
    /// acceleration: tj = 0.08 s, ramp 0.08 + 0.8 / 8 = 0.18 s. Braking: tj = 4 / 100 = 0.04 s, ramp 0.04 + 0.8 / 4 = 0.24 s.
    /// The ramps cover 0.8 (0.18 + 0.24) / 2 = 0.168 m of 1 m; the rest at 0.8 m/s takes 1.04 s: 1.46 s in all.
    #[test]
    fn braking_slower_than_speeding_up_takes_its_own_ramp() {
        let p = Profile::new(1.0, Shape::SCurve, Limits { vel: 0.8, acc: 8.0, dec: 4.0, jerk: 100.0 });
        assert!((p.ramp() - 0.18).abs() < 1e-12 && (p.brake() - 0.24).abs() < 1e-12);
        assert!((p.duration() - 1.46).abs() < 1e-12, "{}", p.duration());
        let (peak, end) = peaks(&p);
        assert!((peak[1] - 8.0).abs() < 1e-9 && (peak[2] - 4.0).abs() < 1e-9 && peak[3] <= 100.0 + 1e-9, "{peak:?}");
        assert!((end.0 - 1.0).abs() < 1e-15);
        // With the same braking the general ramps give the closed form's S-curve.
        let ramps =
            Profile::new(1.0, Shape::SCurve, Limits { vel: 0.8, acc: 8.0, dec: 8.0 * (1.0 - 1e-15), jerk: 100.0 });
        assert!((ramps.duration() - SCurve::new(1.0, 0.8, 8.0, 100.0).duration()).abs() < 1e-9);
    }

    /// Stretched to twice its time, a profile is the same shape at half the speed: the peak speed of a 3-4-5 law and of
    /// an S-curve halves, the end stays.
    #[test]
    fn a_stretched_profile_is_the_same_shape_slower() {
        let limits = Limits { vel: 0.8, acc: 8.0, dec: 8.0, jerk: 100.0 };
        for shape in [Shape::SCurve, Shape::SinSquared, Shape::Poly345, Shape::Poly4567] {
            let (fast, d) = (Profile::new(0.5, shape, limits), 0.5);
            let slow = Profile::stretched(d, shape, limits, 2.0 * fast.duration());
            assert!((slow.duration() - 2.0 * fast.duration()).abs() < 1e-9, "{shape:?}");
            for u in [0.1, 0.37, 0.5, 0.8] {
                let (a, b) = (fast.at(u * fast.duration()), slow.at(u * slow.duration()));
                assert!((a.0 - b.0).abs() < 1e-9 && (a.1 - 2.0 * b.1).abs() < 1e-9, "{shape:?} {u}");
            }
        }
    }

    /// Lifting against gravity: z (axis 1, up = +) has half its acceleration upwards. Up, the move speeds up at
    /// 3 m/s^2 and brakes (downwards) at 6; down it speeds up at 6 and brakes at 3. A trapezoid (high jerk) over 0.1 m
    /// at up to 1 m/s: up, ramps of 1/3 and 1/6 s would cover 0.25 m, more than 0.1: no cruise, and v^2 (1/3 + 1/6) / 2
    /// = 0.1 gives v = 0.6325 m/s, T = v (1/3 + 1/6) = 0.3162 s, the same down with the ramps swapped.
    #[test]
    fn a_vertical_axis_accelerates_slower_upwards() {
        let axes = [axis(1.0, 6.0), AxisInfo { max_jerk: 1e9, ..axis(1.0, 6.0) }];
        let law = MotionLaw { vertical: Some((1, 1.0, 0.5)), ..MotionLaw::default() };
        let (up, down) = (
            Move::with_law(&[0.0, 0.0], &[0.0, 0.1], &axes, &law),
            Move::with_law(&[0.0, 0.1], &[0.0, 0.0], &axes, &law),
        );
        let v = (0.2f64 / 0.5).sqrt();
        for m in [&up, &down] {
            assert!((m.duration() - v * 0.5).abs() < 1e-6, "{}", m.duration());
        }
        assert!(up.ramp() > up.brake() && down.ramp() < down.brake());
        assert!((up.ramp() - 2.0 * up.brake()).abs() < 1e-6);
        let (mut p, mut vel, mut acc) = ([0.0; 2], [0.0; 2], [0.0; 2]);
        up.at(up.ramp() / 2.0, &mut p, &mut vel, &mut acc);
        assert!((acc[1] - 3.0).abs() < 1e-3, "{acc:?}");
        down.at(down.ramp() / 2.0, &mut p, &mut vel, &mut acc);
        assert!((acc[1] + 6.0).abs() < 1e-3, "{acc:?}");
    }

    #[test]
    fn a_move_to_where_it_already_is_takes_no_time() {
        let m = Move::new(&[0.3, 0.2], &[0.3, 0.2], &[axis(0.8, 8.0), axis(0.4, 6.0)]);
        assert_eq!(m.duration(), 0.0);
    }
}
