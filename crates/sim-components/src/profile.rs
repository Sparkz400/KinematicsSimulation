//! Motion profiles: the jerk-limited seven-segment S-curve and synchronised multi-axis moves.

use sim_core::AxisInfo;

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
        let segments = [(self.tj, j), (self.ta, 0.0), (self.tj, -j), (self.tv, 0.0), (self.tj, -j), (self.ta, 0.0), (self.tj, j)];
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
/// One S-curve runs the path from 0 to 1; its limits are the tightest of all axes.
#[derive(Debug, Clone)]
pub struct Move {
    start: Vec<f64>,
    delta: Vec<f64>,
    curve: SCurve,
}

impl Move {
    pub fn new(start: &[f64], target: &[f64], axes: &[AxisInfo]) -> Self {
        let delta: Vec<f64> = target.iter().zip(start).map(|(t, s)| t - s).collect();
        let tightest = |limit: fn(&AxisInfo) -> f64| {
            axes.iter().zip(&delta).filter(|(_, d)| **d != 0.0).map(|(a, d)| limit(a) / d.abs()).fold(f64::INFINITY, f64::min)
        };
        let moving = delta.iter().any(|d| *d != 0.0);
        let curve = if moving {
            SCurve::new(1.0, tightest(|a| a.max_vel), tightest(|a| a.max_acc), tightest(|a| a.max_jerk))
        } else {
            SCurve::default()
        };
        Self { start: start.to_vec(), delta, curve }
    }

    pub fn duration(&self) -> f64 {
        self.curve.duration()
    }

    pub fn ramp(&self) -> f64 {
        self.curve.ramp()
    }

    pub fn delta(&self) -> &[f64] {
        &self.delta
    }

    pub fn at(&self, t: f64, pos: &mut [f64], vel: &mut [f64], acc: &mut [f64]) {
        let (s, v, a, _) = if self.curve.dist > 0.0 { self.curve.at(t) } else { (0.0, 0.0, 0.0, 0.0) };
        for i in 0..pos.len() {
            pos[i] = self.start[i] + self.delta[i] * s;
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
            (1.0, 0.8, 8.0, 100.0), // all seven segments
            (0.05, 0.8, 8.0, 100.0), // no cruise, acceleration limit reached
            (0.001, 0.8, 8.0, 100.0), // no cruise, acceleration limit not reached
            (1.0, 0.2, 8.0, 100.0), // cruise, acceleration limit not reached
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
                assert!((now.0 - last.0 - 0.5 * (now.1 + last.1) * h).abs() <= 1e-9 * dist + j * h.powi(3), "{dist}: p/v");
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

    #[test]
    fn a_move_to_where_it_already_is_takes_no_time() {
        let m = Move::new(&[0.3, 0.2], &[0.3, 0.2], &[axis(0.8, 8.0), axis(0.4, 6.0)]);
        assert_eq!(m.duration(), 0.0);
    }
}
