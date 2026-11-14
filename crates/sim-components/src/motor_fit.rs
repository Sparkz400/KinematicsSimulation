//! Identification that needs no machine model: one motor's drive train from its own trace, in the motor's
//! units (rad, rad/s, N m at the shaft). So it works on any imported drive trace, also of a machine the
//! simulator has no config for.
//!
//! - Inertia and friction: torque = J α + b ω + (Tc + (Ts - Tc) e^-(ω/ωs)²) sign ω + T0, with the Stribeck
//!   part kept only where the trace shows it, and the LuGre bristle stiffness σ0 from the reversals.
//! - Drive delays and filters (the fit's stage 3): a gain, a first-order lag and a dead time between a
//!   setpoint and what follows it (torque setpoint to torque, position setpoint to position).
//! - Stiffness and damping between motor and load, as a two-mass model: from a frequency response (measured
//!   by the drive, or computed here from a trace with an excitation), or from the ringing after a torque step.
//!
//! Every value comes with a standard error from the least-squares covariance, widened for residuals that
//! are correlated from row to row; `Estimate::interval` is the 95 % interval from it. Every model says how
//! well it reproduces the trace (`Quality`).

use crate::analysis::SweepPoint;
use crate::identify::{Resonance, resonance};
use std::f64::consts::TAU;

/// A fitted value and its standard error.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Estimate {
    pub value: f64,
    pub error: f64,
}

impl Estimate {
    /// The 95 % confidence interval, value ± 1.96 standard errors.
    pub fn interval(&self) -> (f64, f64) {
        (self.value - 1.96 * self.error, self.value + 1.96 * self.error)
    }
}

/// How well a model reproduces a measured signal: RMS of the difference, and the normalised fit
/// 100 % × (1 - |measured - model| / |measured - mean|): 100 % is exact, 0 % no better than the mean.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quality {
    pub rms: f64,
    pub fit_percent: f64,
}

pub fn quality(measured: &[f64], model: &[f64]) -> Quality {
    let n = measured.len().max(1) as f64;
    let mean = measured.iter().sum::<f64>() / n;
    let off: f64 = measured.iter().zip(model).map(|(m, s)| (m - s).powi(2)).sum();
    let spread: f64 = measured.iter().map(|m| (m - mean).powi(2)).sum();
    Quality { rms: (off / n).sqrt(), fit_percent: 100.0 * (1.0 - (off / spread.max(f64::MIN_POSITIVE)).sqrt()) }
}

/// Solves a x = b by elimination with partial pivoting; `None` where `a` is (nearly) singular.
fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    let size = a.iter().flatten().fold(0.0, |m: f64, v| m.max(v.abs()));
    for col in 0..n {
        let pivot = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if !(a[pivot][col].abs() > 1e-12 * size) {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..n {
            let f = a[row][col] / a[col][col];
            for j in col..n {
                a[row][j] -= f * a[col][j];
            }
            b[row] -= f * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        x[row] = (b[row] - (row + 1..n).map(|j| a[row][j] * x[j]).sum::<f64>()) / a[row][row];
    }
    Some(x)
}

/// Solves a x = b for a symmetric positive (semi)definite `a` whose diagonal spans many decades, by
/// solving the system scaled to a unit diagonal.
fn solve_scaled(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let d: Vec<f64> = (0..b.len()).map(|i| a[i][i].max(f64::MIN_POSITIVE).sqrt()).collect();
    let scaled = (0..b.len()).map(|i| (0..b.len()).map(|j| a[i][j] / (d[i] * d[j])).collect()).collect();
    let x = solve(scaled, b.iter().zip(&d).map(|(b, d)| b / d).collect())?;
    Some(x.iter().zip(&d).map(|(x, d)| x / d).collect())
}

/// A least-squares fit: the values, their standard errors and the residuals at the values.
struct Fitted {
    values: Vec<f64>,
    errors: Vec<f64>,
    residuals: Vec<f64>,
}

/// Levenberg-Marquardt on `residuals` from `start`, with a numeric Jacobian whose steps are 1e-6 of
/// `scale` (the size each value is expected to have). The covariance is σ² (JᵀJ)⁻¹, with σ² widened by
/// (1 + ρ) / (1 - ρ) for a lag-one correlation ρ of the residuals: rows of a filtered trace are not
/// independent measurements. `None` where the values cannot be told apart (singular normal equations).
fn least_squares(start: &[f64], scale: &[f64], residuals: &dyn Fn(&[f64]) -> Vec<f64>) -> Option<Fitted> {
    let n = start.len();
    let cost = |r: &[f64]| r.iter().map(|v| v * v).sum::<f64>();
    let mut p = start.to_vec();
    let mut r = residuals(&p);
    let mut c = cost(&r);
    if !c.is_finite() || r.len() <= n {
        return None;
    }
    let jacobian = |p: &[f64], r: &[f64]| -> Vec<Vec<f64>> {
        (0..n)
            .map(|j| {
                let h = 1e-6 * scale[j];
                let mut q = p.to_vec();
                q[j] += h;
                residuals(&q).iter().zip(r).map(|(a, b)| (a - b) / h).collect()
            })
            .collect()
    };
    let normal = |jac: &[Vec<f64>]| -> Vec<Vec<f64>> {
        (0..n).map(|a| (0..n).map(|b| jac[a].iter().zip(&jac[b]).map(|(x, y)| x * y).sum()).collect()).collect()
    };
    let mut lambda = 1e-3;
    for _ in 0..200 {
        let jac = jacobian(&p, &r);
        let a = normal(&jac);
        let g: Vec<f64> = jac.iter().map(|col| -col.iter().zip(&r).map(|(x, y)| x * y).sum::<f64>()).collect();
        let mut improved = false;
        while lambda < 1e12 {
            let mut damped = a.clone();
            (0..n).for_each(|i| damped[i][i] += lambda * a[i][i].max(1e-300));
            let Some(step) = solve_scaled(&damped, &g) else {
                lambda *= 10.0;
                continue;
            };
            let q: Vec<f64> = p.iter().zip(&step).map(|(a, b)| a + b).collect();
            let rq = residuals(&q);
            let cq = cost(&rq);
            if cq.is_finite() && cq < c {
                let done = c - cq < 1e-12 * c;
                (p, r, c) = (q, rq, cq);
                lambda = (lambda / 3.0).max(1e-12);
                improved = !done;
                break;
            }
            lambda *= 4.0;
        }
        if !improved {
            break;
        }
    }
    let a = normal(&jacobian(&p, &r));
    let m = r.len();
    let rho = (r.windows(2).map(|w| w[0] * w[1]).sum::<f64>() / c.max(f64::MIN_POSITIVE)).clamp(0.0, 0.98);
    let variance = c / (m - n) as f64 * (1.0 + rho) / (1.0 - rho);
    let errors = (0..n)
        .map(|j| {
            let unit: Vec<f64> = (0..n).map(|i| if i == j { 1.0 } else { 0.0 }).collect();
            solve_scaled(&a, &unit).map(|column| (variance * column[j]).max(0.0).sqrt())
        })
        .collect::<Option<Vec<f64>>>()?;
    Some(Fitted { values: p, errors, residuals: r })
}

/// The standard error of `f(values)` by first-order propagation of independent errors.
fn propagate(f: &dyn Fn(&[f64]) -> f64, inputs: &[Estimate]) -> Estimate {
    let values: Vec<f64> = inputs.iter().map(|e| e.value).collect();
    let value = f(&values);
    let error = (0..inputs.len())
        .map(|i| {
            let mut shifted = values.clone();
            shifted[i] += inputs[i].error;
            (f(&shifted) - value).powi(2)
        })
        .sum::<f64>()
        .sqrt();
    Estimate { value, error }
}

/// Rows on either side that a smoothing window spans: 5 ms, at least one row and at most the trace. `None` where
/// the times do not rise (a zero step made the window `usize::MAX` rows and `2 w + 1` overflowed).
fn half_window(time: &[f64]) -> Option<usize> {
    if time.windows(2).any(|p| !(p[1] > p[0])) {
        return None;
    }
    let step = (time[time.len() - 1] - time[0]) / (time.len() - 1) as f64;
    Some(((0.005 / step).round() as usize).clamp(1, time.len()))
}

/// Inertia and friction of one motor's drive train, at the motor shaft.
#[derive(Debug, Clone, PartialEq)]
pub struct Friction {
    /// Total inertia: rotor, couplings, gearbox and the load reflected to the motor. kg m².
    pub inertia: Estimate,
    /// N m s/rad.
    pub viscous: Estimate,
    /// Sliding (Coulomb) friction, N m.
    pub coulomb: Estimate,
    /// Breakaway torque and the Stribeck speed (rad/s) of the drop to Coulomb friction; `None` where the
    /// trace does not show a drop that is clearly above its noise.
    pub stiction: Option<Estimate>,
    pub stribeck_speed: Option<Estimate>,
    /// A torque that does not depend on the motion: weight on a vertical axis, a cable pull. N m.
    pub constant: Estimate,
    /// LuGre bristle stiffness σ0 from the reversals (N m/rad); `None` with fewer than three reversals whose
    /// presliding the trace resolves (a fast reversal crosses it within a row: see the measurement procedure).
    pub presliding_stiffness: Option<Estimate>,
    /// The model against the smoothed measured torque on the rows used.
    pub quality: Quality,
    /// The model's torque per row of the trace (from the smoothed speed and acceleration).
    pub model: Vec<f64>,
}

/// Inertia and friction from a trace's speed and torque at one motor (`angle` for the presliding).
/// Speed and torque are averaged over 5 ms on either side, the acceleration is the slope of the speed across
/// that window. Rows slower than 1 % of the top speed are left out, and so are rows whose window has a reversal. `None` if the motor does not turn both
/// ways (Coulomb friction and a constant torque cannot be told apart then) or the trace is too short.
pub fn friction(time: &[f64], angle: &[f64], speed: &[f64], torque: &[f64]) -> Option<Friction> {
    let rows = time.len();
    if rows < 20 {
        return None;
    }
    let w = half_window(time)?;
    let top = speed.iter().fold(0.0, |a: f64, b| a.max(b.abs()));
    let mean = |x: &[f64], i: usize| x[i - w..=i + w].iter().sum::<f64>() / (2 * w + 1) as f64;
    let mut smooth = vec![(0.0, 0.0, 0.0); rows];
    for i in w..rows.saturating_sub(w) {
        smooth[i] = (mean(speed, i), (speed[i + w] - speed[i - w]) / (time[i + w] - time[i - w]), mean(torque, i));
    }
    // A row whose window spans a reversal would average the two Coulomb levels: only windows of one sign.
    let used: Vec<usize> = (w..rows.saturating_sub(w))
        .filter(|&i| {
            smooth[i].0.abs() > 0.01 * top && speed[i - w..=i + w].iter().all(|v| v.signum() == smooth[i].0.signum())
        })
        .collect();
    let one_way = |sign: f64| used.iter().filter(|&&i| smooth[i].0.signum() == sign).count();
    if one_way(1.0).min(one_way(-1.0)) < 10 * (2 * w + 1) {
        return None;
    }
    let stribeck = |p: &[f64], v: f64, a: f64| {
        p[0] * a + p[1] * v + (p[2] + p.get(4).map_or(0.0, |ds| ds * (-(v / p[5]).powi(2)).exp())) * v.signum() + p[3]
    };
    let residuals =
        |p: &[f64]| -> Vec<f64> { used.iter().map(|&i| stribeck(p, smooth[i].0, smooth[i].1) - smooth[i].2).collect() };
    let level = torque.iter().fold(0.0, |a: f64, b| a.max(b.abs())).max(1e-9);
    let accel = used.iter().fold(0.0, |a: f64, &i| a.max(smooth[i].1.abs())).max(1e-9);
    let scale = [level / accel, level / top, level, level, level, top];
    let plain = least_squares(&[0.0; 4], &scale[..4], &residuals)?;
    // Stribeck: started from the plain fit with a drop of a fifth of the Coulomb friction at 5 % of top speed.
    let p = &plain.values;
    let start = [p[0], p[1], p[2], p[3], 0.2 * p[2].abs(), 0.05 * top];
    let curved = least_squares(&start, &scale, &residuals);
    let curved = curved.filter(|f| {
        let (drop, speed) = ((f.values[4], f.errors[4]), (f.values[5].abs(), f.errors[5]));
        // Kept only where the drop is real and the speed it happens at is known to better than half and lies
        // inside what the trace covers.
        let rss = |r: &[f64]| r.iter().map(|v| v * v).sum::<f64>();
        drop.0 > 3.0 * drop.1 && speed.1 < 0.5 * speed.0 && speed.0 < top && rss(&f.residuals) < rss(&plain.residuals)
    });
    let fit = curved.as_ref().unwrap_or(&plain);
    let at = |q: usize| Estimate { value: fit.values[q], error: fit.errors[q] };
    let model: Vec<f64> = smooth.iter().map(|&(v, a, _)| stribeck(&fit.values, v, a)).collect();
    let measured: Vec<f64> = used.iter().map(|&i| smooth[i].2).collect();
    let modelled: Vec<f64> = used.iter().map(|&i| model[i]).collect();
    let coulomb = at(2);
    let presliding = presliding(time, angle, speed, torque, &smooth, w, &fit.values, coulomb.value, 0.01 * top);
    Some(Friction {
        inertia: at(0),
        viscous: at(1),
        coulomb,
        stiction: curved
            .as_ref()
            .map(|f| propagate(&|v| v[0] + v[1], &[at(2), Estimate { value: f.values[4], error: f.errors[4] }])),
        stribeck_speed: curved.as_ref().map(|f| Estimate { value: f.values[5].abs(), error: f.errors[5] }),
        constant: at(3),
        presliding_stiffness: presliding,
        quality: quality(&measured, &modelled),
        model,
    })
}

/// σ0 from the reversals: after the smoothed speed changes sign, friction (torque less inertia and constant
/// torque) swings from one sliding level to the other over a small angle. In the LuGre model the slope of
/// friction over angle is σ0 (1 - F / g): twice σ0 at the turning point and σ0 where friction passes zero.
/// So the slope is taken over the middle half of the swing, |F| < Tc / 2, per reversal whose swing ends
/// within 0.5 s while the speed stays below `slow` (else it is a fast reversal, where the swing is the
/// loops' doing) and covers at least four rows; the median of them, with their spread as its error.
#[allow(clippy::too_many_arguments)]
fn presliding(
    time: &[f64],
    angle: &[f64],
    speed: &[f64],
    torque: &[f64],
    smooth: &[(f64, f64, f64)],
    w: usize,
    p: &[f64],
    coulomb: f64,
    slow: f64,
) -> Option<Estimate> {
    let rows = time.len();
    let half = 0.5 * coulomb.abs();
    let mut slopes = Vec::new();
    for i in w + 1..rows.saturating_sub(w) {
        let (before, after) = (smooth[i - 1].0, smooth[i].0);
        if before == 0.0 || after == 0.0 || before.signum() == after.signum() {
            continue;
        }
        let sign = after.signum();
        // The turning point: the raw row nearest the sign change of the smoothed speed.
        let start = (i - w..i + w).min_by(|&a, &b| speed[a].abs().total_cmp(&speed[b].abs()))?;
        let friction = |k: usize| sign * (torque[k] - p[0] * smooth[k].1 - p[3]);
        let mut points = Vec::new();
        let mut ended = false;
        for k in start..rows - w {
            if time[k] - time[start] > 0.5 || speed[k].abs() > slow {
                break;
            }
            let f = friction(k);
            if f > half {
                ended = true;
                break;
            }
            if f >= -half {
                points.push((sign * angle[k], f));
            }
        }
        if !ended || points.len() < 4 {
            continue;
        }
        let n = points.len() as f64;
        let (mx, my) = (points.iter().map(|p| p.0).sum::<f64>() / n, points.iter().map(|p| p.1).sum::<f64>() / n);
        let sxx: f64 = points.iter().map(|p| (p.0 - mx).powi(2)).sum();
        let sxy: f64 = points.iter().map(|p| (p.0 - mx) * (p.1 - my)).sum();
        if sxx > 0.0 && sxy > 0.0 {
            slopes.push(sxy / sxx);
        }
    }
    if slopes.len() < 3 {
        return None;
    }
    slopes.sort_by(f64::total_cmp);
    let median = slopes[slopes.len() / 2];
    let spread = (slopes.iter().map(|s| (s - median).powi(2)).sum::<f64>() / (slopes.len() - 1) as f64).sqrt();
    Some(Estimate { value: median, error: 1.25 * spread / (slopes.len() as f64).sqrt() })
}

/// A setpoint and what follows it, as gain × first-order lag × dead time: the drive's loops, filters and
/// bus delays lumped together (stage 3 of the fit).
#[derive(Debug, Clone, PartialEq)]
pub struct Lag {
    pub gain: Estimate,
    /// s.
    pub time_constant: Estimate,
    /// s. Below zero where the output leads the setpoint: the two signals' times are misaligned.
    pub dead_time: Estimate,
    /// The model against the measured output, and against the measured difference setpoint - output
    /// (the following error, which says more where the output follows closely).
    pub quality: Quality,
    pub error_quality: Quality,
    pub model: Vec<f64>,
}

/// The output of the lag model for an input, starting where the measured output starts and at rest. The
/// input is held from one row to the next, as a drive holds a setpoint for its cycle; delayed by a dead time
/// that is not a whole number of rows it changes once within a step, and both parts are solved exactly.
/// Rows have to be evenly spaced.
fn lag_response(time: &[f64], input: &[f64], output0: f64, gain: f64, tau: f64, dead: f64) -> Vec<f64> {
    let (rows, start) = (time.len(), time[0]);
    let step = (time[rows - 1] - start) / (rows - 1) as f64;
    // The held input at a time, and the time it changes next.
    let held = |t: f64| -> (f64, f64) {
        let k = ((t - start) / step).floor();
        let value = if k < 0.0 { input[0] } else { input[(k as usize).min(rows - 1)] };
        (value, start + (k + 1.0) * step)
    };
    let mut out = vec![output0; rows];
    for k in 1..rows {
        let (from, to) = (time[k - 1] - dead, time[k] - dead);
        let (first, change) = held(from + 1e-9 * step);
        let mut y = out[k - 1];
        let mut go = |value: f64, span: f64| {
            let a = (-span.max(0.0) / tau).exp();
            y = a * y + (1.0 - a) * (output0 + gain * (value - input[0]));
        };
        if change < to {
            go(first, change - from);
            go(held(change + 1e-9 * step).0, to - change);
        } else {
            go(first, to - from);
        }
        out[k] = y;
    }
    out
}

/// The input, linear between rows, delayed by `dead`, times `gain`, from where the output starts.
fn delayed(time: &[f64], input: &[f64], output0: f64, gain: f64, dead: f64) -> Vec<f64> {
    time.iter()
        .map(|&t| {
            let k = time.partition_point(|&x| x <= t - dead).clamp(1, time.len() - 1);
            let share = ((t - dead - time[k - 1]) / (time[k] - time[k - 1])).clamp(0.0, 1.0);
            output0 + gain * (input[k - 1] + share * (input[k] - input[k - 1]) - input[0])
        })
        .collect()
}

/// Fits gain, lag and dead time from `input` (a setpoint) to `output`. The trace has to start at rest.
/// The start is the best point of a grid (dead time 0 to 20 rows, time constant 0 and 1 to 100 rows). A
/// time constant the trace cannot resolve (shorter than a fraction of a row, or a following error too small
/// to show it) is reported as 0 ± one row, and gain and dead time are fitted without it.
pub fn lag(time: &[f64], input: &[f64], output: &[f64]) -> Option<Lag> {
    let rows = time.len();
    if rows < 20 {
        return None;
    }
    let step = (time[rows - 1] - time[0]) / (rows - 1) as f64;
    // The output at rest is fitted too ([gain, ln τ, dead time, rest]): one noisy first sample would bias all.
    let span =
        output.iter().fold(f64::NEG_INFINITY, |a, &b| a.max(b)) - output.iter().fold(f64::INFINITY, |a, &b| a.min(b));
    let rest = output[..rows.min(5)].iter().sum::<f64>() / rows.min(5) as f64;
    let residuals = |p: &[f64]| -> Vec<f64> {
        lag_response(time, input, p[3], p[0], p[1].exp(), p[2]).iter().zip(output).map(|(m, y)| m - y).collect()
    };
    let cost = |p: &[f64]| residuals(p).iter().map(|v| v * v).sum::<f64>();
    let none = (1e-6 * step).ln();
    let mut best = (f64::INFINITY, [1.0, step.ln(), 0.0, rest]);
    for d in 0..=20 {
        for t in 0..=21 {
            let tau = if t == 21 { none } else { (step * 100f64.powf(t as f64 / 20.0)).ln() };
            let p = [1.0, tau, d as f64 * step, rest];
            let c = cost(&p);
            if c < best.0 {
                best = (c, p);
            }
        }
    }
    // Resolved: the log of the time constant known to better than a factor e.
    let scale = [1.0, 1.0, step, span.max(1e-12)];
    let resolved = least_squares(&best.1, &scale, &residuals).filter(|f| f.errors[1] < 1.0 && f.values[1] > none + 1.0);
    let (gain, tau, dead, rest) = match resolved {
        Some(f) => {
            let tau = f.values[1].exp();
            (
                Estimate { value: f.values[0], error: f.errors[0] },
                Estimate { value: tau, error: tau * f.errors[1] },
                Estimate { value: f.values[2], error: f.errors[2] },
                f.values[3],
            )
        }
        // A held input delayed without a lag is a staircase that moves in whole rows, with no slope to fit the
        // delay on; between rows the input is taken as linear instead.
        None => {
            let f = least_squares(&[best.1[0], best.1[2], rest], &[1.0, step, scale[3]], &|p: &[f64]| {
                delayed(time, input, p[2], p[0], p[1]).iter().zip(output).map(|(m, y)| m - y).collect()
            })?;
            (
                Estimate { value: f.values[0], error: f.errors[0] },
                Estimate { value: 0.0, error: step },
                Estimate { value: f.values[1], error: f.errors[1] },
                f.values[2],
            )
        }
    };
    let model = match tau.value {
        0.0 => delayed(time, input, rest, gain.value, dead.value),
        tau => lag_response(time, input, rest, gain.value, tau, dead.value),
    };
    let following = |y: &[f64]| -> Vec<f64> { input.iter().zip(y).map(|(u, y)| u - y).collect() };
    Some(Lag {
        gain,
        time_constant: tau,
        dead_time: dead,
        quality: quality(output, &model),
        error_quality: quality(&following(output), &following(&model)),
        model,
    })
}

/// The frequency response from `input` to `output` of a trace with an excitation in it (noise, a chirp, a
/// PRBS), at the given frequencies: Welch's H1 estimate over Hann windows with half overlap, each window
/// long enough for four periods of the lowest frequency. With it the coherence, 0 to 1: below about 0.8
/// the point is mostly noise or the response is not linear there. `None` for a trace too short for eight
/// windows: with few windows the coherence of pure noise comes out near 1.
pub fn frequency_response(
    time: &[f64],
    input: &[f64],
    output: &[f64],
    frequencies: &[f64],
) -> Option<Vec<(SweepPoint, f64)>> {
    let rows = time.len();
    if rows < 32 || frequencies.is_empty() {
        return None;
    }
    let step = (time[rows - 1] - time[0]) / (rows - 1) as f64;
    let lowest = frequencies.iter().fold(f64::INFINITY, |a, &b| a.min(b));
    let length = ((4.0 / (lowest * step)).ceil() as usize).clamp(8, rows);
    let window: Vec<f64> = (0..length).map(|k| 0.5 - 0.5 * (TAU * k as f64 / length as f64).cos()).collect();
    let starts: Vec<usize> = (0..).map(|s| s * length / 2).take_while(|s| s + length <= rows).collect();
    if starts.len() < 8 {
        return None;
    }
    let response = frequencies
        .iter()
        .map(|&f| {
            let (mut pxy, mut pxx, mut pyy) = ((0.0, 0.0), 0.0, 0.0);
            for &s in &starts {
                let spectrum = |x: &[f64]| -> (f64, f64) {
                    let m = x[s..s + length].iter().sum::<f64>() / length as f64;
                    (0..length).fold((0.0, 0.0), |(re, im), k| {
                        let (phase, v) = (TAU * f * k as f64 * step, window[k] * (x[s + k] - m));
                        (re + v * phase.cos(), im - v * phase.sin())
                    })
                };
                let (x, y) = (spectrum(input), spectrum(output));
                // conj(X) Y
                pxy = (pxy.0 + x.0 * y.0 + x.1 * y.1, pxy.1 + x.0 * y.1 - x.1 * y.0);
                pxx += x.0 * x.0 + x.1 * x.1;
                pyy += y.0 * y.0 + y.1 * y.1;
            }
            let gain = pxy.0.hypot(pxy.1) / pxx;
            let coherence = (pxy.0 * pxy.0 + pxy.1 * pxy.1) / (pxx * pyy);
            (SweepPoint { frequency: f, gain, phase_deg: pxy.1.atan2(pxy.0).to_degrees() }, coherence)
        })
        .collect();
    Some(response)
}

/// Motor and load coupled by a spring with damping: the belt or the coupling, at the motor shaft.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TwoMass {
    /// kg m².
    pub total_inertia: Estimate,
    pub motor_inertia: Estimate,
    pub load_inertia: Estimate,
    /// N m/rad and N m s/rad.
    pub stiffness: Estimate,
    pub damping: Estimate,
    /// Hz: where the load swings against the motor (a peak in torque to speed), and where the motor stands
    /// while the load swings (the dip).
    pub resonance: Estimate,
    pub anti_resonance: Estimate,
}

/// The two-mass model from a frequency response of motor torque to motor speed: the dip (the lowest in
/// frequency, as `resonance` finds it), the highest peak above it up to five times its frequency, and the
/// total inertia from the part below a third of the dip (gain = 1 / (J ω) there, corrected for the two
/// frequencies). Then
/// J_motor = J (f_dip / f_peak)², J_load = J - J_motor, stiffness = J_load (2π f_dip)², and the damping from
/// the width of the dip. That is the start for a least-squares fit of the whole model (with a viscous
/// friction at the motor) through all points, whose values and errors are returned; where that fails, the
/// values read off the dip and peak.
/// `None` without a dip, a peak inside the response, or three points below the dip.
pub fn two_mass(sweep: &[SweepPoint]) -> Option<TwoMass> {
    let mut sweep = sweep.to_vec();
    sweep.sort_by(|a, b| a.frequency.total_cmp(&b.frequency));
    let Resonance { frequency: fa, damping: zeta, frequency_error: fa_error } = resonance(&sweep)?;
    let level: Vec<f64> = sweep.iter().map(|p| p.gain * p.frequency).collect();
    let above: Vec<usize> =
        (0..sweep.len()).filter(|&i| sweep[i].frequency > fa && sweep[i].frequency <= 5.0 * fa).collect();
    let peak = *above.iter().max_by(|&&a, &&b| level[a].total_cmp(&level[b]))?;
    if peak == 0 || peak + 1 >= sweep.len() || level[peak] < level[peak - 1] || level[peak] < level[peak + 1] {
        return None;
    }
    // A parabola through the three points in log frequency and log level.
    let (x, y) = (
        [peak - 1, peak, peak + 1].map(|i| sweep[i].frequency.ln()),
        [peak - 1, peak, peak + 1].map(|i| level[i].ln()),
    );
    let (d1, d2) = ((y[1] - y[0]) / (x[1] - x[0]), (y[2] - y[1]) / (x[2] - x[1]));
    let curve = (d2 - d1) / (0.5 * (x[2] - x[0]));
    let top = if curve < 0.0 { 0.5 * (x[0] + x[1]) - d1 / curve } else { x[1] };
    let fr = Estimate { value: top.exp(), error: 0.5 * (sweep[peak + 1].frequency - sweep[peak - 1].frequency) / 2.0 };
    // Below the dip the response is that of the total inertia times (1 - (f/fa)²) / (1 - (f/fr)²) (without
    // damping), which is taken out.
    let correction = |f: f64| (1.0 - (f / fa).powi(2)) / (1.0 - (f / fr.value).powi(2));
    let low: Vec<f64> = sweep
        .iter()
        .filter(|p| p.frequency < fa / 3.0)
        .map(|p| correction(p.frequency) / (TAU * p.gain * p.frequency))
        .collect();
    if low.len() < 3 {
        return None;
    }
    let n = low.len() as f64;
    let mean = low.iter().sum::<f64>() / n;
    let total =
        Estimate { value: mean, error: (low.iter().map(|j| (j - mean).powi(2)).sum::<f64>() / (n - 1.0) / n).sqrt() };
    let fa = Estimate { value: fa, error: fa_error };
    let inputs = [total, fa, fr, Estimate { value: zeta, error: 0.0 }];
    let motor = |v: &[f64]| v[0] * (v[1] / v[2]).powi(2);
    let load = |v: &[f64]| v[0] - motor(v);
    let stiffness = |v: &[f64]| load(v) * (TAU * v[1]).powi(2);
    // The dip is the load alone on the spring: its damping ratio is c / (2 sqrt(k J_load)).
    let damping = |v: &[f64]| 2.0 * v[3] * (stiffness(v) * load(v)).sqrt();
    let rough = TwoMass {
        total_inertia: total,
        motor_inertia: propagate(&motor, &inputs),
        load_inertia: propagate(&load, &inputs),
        stiffness: propagate(&stiffness, &inputs),
        damping: propagate(&damping, &inputs),
        resonance: fr,
        anti_resonance: fa,
    };
    // Then the whole model through every point, in log gain: total inertia, the motor's share of it,
    // stiffness, damping, and a viscous friction at the motor that bends the response at low frequencies.
    // Read off the dip alone, its width also has the slope towards the peak in it.
    let share = rough.motor_inertia.value / total.value;
    if !(share > 0.0 && share < 1.0 && rough.damping.value > 0.0) {
        return Some(rough);
    }
    let start = [total.value.ln(), share, rough.stiffness.value.ln(), rough.damping.value.ln(), 0.0];
    let b_scale = total.value * TAU * sweep[0].frequency;
    let residuals =
        |p: &[f64]| -> Vec<f64> { sweep.iter().map(|q| (two_mass_gain(p, q.frequency) / q.gain).ln()).collect() };
    let Some(fit) = least_squares(&start, &[1.0, 1.0, 1.0, 1.0, b_scale], &residuals) else { return Some(rough) };
    let at = |q: usize| Estimate { value: fit.values[q], error: fit.errors[q] };
    let exp = |q: usize| propagate(&|v| v[0].exp(), &[at(q)]);
    let (j, s, k) = (exp(0), at(1), exp(2));
    Some(TwoMass {
        total_inertia: j,
        motor_inertia: propagate(&|v| v[0] * v[1], &[j, s]),
        load_inertia: propagate(&|v| v[0] * (1.0 - v[1]), &[j, s]),
        stiffness: k,
        damping: exp(3),
        resonance: propagate(&|v| (v[2] / (v[0] * v[1] * (1.0 - v[1]))).sqrt() / TAU, &[j, s, k]),
        anti_resonance: propagate(&|v| (v[2] / (v[0] * (1.0 - v[1]))).sqrt() / TAU, &[j, s, k]),
    })
}

/// |motor speed / motor torque| of the two-mass model at `f` for [ln J, motor share, ln k, ln c, b]:
/// 1 / |J_m s + b + D J_l s / (J_l s + D)| with D = c + k / s, s = j 2π f.
fn two_mass_gain(p: &[f64], f: f64) -> f64 {
    let (j, w) = (p[0].exp(), TAU * f);
    let (jm, jl, k, c, b) = (j * p[1], j * (1.0 - p[1]), p[2].exp(), p[3].exp(), p[4]);
    let mul = |a: (f64, f64), b: (f64, f64)| (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0);
    let div = |a: (f64, f64), b: (f64, f64)| {
        let n = b.0 * b.0 + b.1 * b.1;
        ((a.0 * b.0 + a.1 * b.1) / n, (a.1 * b.0 - a.0 * b.1) / n)
    };
    let d = (c, -k / w);
    let load_s = (0.0, jl * w);
    let coupled = div(mul(d, load_s), (d.0 + load_s.0, d.1 + load_s.1));
    let total = (b + coupled.0, jm * w + coupled.1);
    1.0 / total.0.hypot(total.1)
}

/// A damped oscillation riding on a slow trend: what the motor speed does after a torque step.
#[derive(Debug, Clone, PartialEq)]
pub struct Ringing {
    /// Undamped natural frequency, Hz.
    pub frequency: Estimate,
    pub damping_ratio: Estimate,
    pub quality: Quality,
}

/// Fits A e^(-σt) sin(ω_d t + φ) + c0 + c1 t + c2 t² to `signal` between `from` and `until` (s), with t
/// from `from`. The quadratic takes the rigid motion under the step. The start for ω_d is the strongest
/// frequency of what is left after the trend; the start for φ the best of eight. `None` with fewer than
/// 20 rows in the span or no fit.
pub fn ringing(time: &[f64], signal: &[f64], from: f64, until: f64) -> Option<Ringing> {
    let rows: Vec<usize> = (0..time.len()).filter(|&i| time[i] >= from && time[i] <= until).collect();
    if rows.len() < 20 {
        return None;
    }
    let t: Vec<f64> = rows.iter().map(|&i| time[i] - from).collect();
    let y: Vec<f64> = rows.iter().map(|&i| signal[i]).collect();
    let span = t[t.len() - 1];
    let level = y.iter().fold(0.0, |a: f64, b| a.max(b.abs())).max(1e-12);
    let trend = least_squares(&[0.0; 3], &[level, level / span, level / span / span], &|p: &[f64]| {
        t.iter().zip(&y).map(|(t, y)| p[0] + p[1] * t + p[2] * t * t - y).collect()
    })?;
    let rest = &trend.residuals;
    let step = span / (t.len() - 1) as f64;
    let candidates = crate::analysis::log_spaced(2.0 / span, 0.25 / step, 200);
    let power = |f: f64| {
        let (re, im) = t
            .iter()
            .zip(rest)
            .fold((0.0, 0.0), |(re, im), (t, r)| (re + r * (TAU * f * t).cos(), im + r * (TAU * f * t).sin()));
        re * re + im * im
    };
    let fd = candidates.into_iter().max_by(|a, b| power(*a).total_cmp(&power(*b)))?;
    let amplitude = rest.iter().fold(0.0, |a: f64, b| a.max(b.abs()));
    let model =
        |p: &[f64], t: f64| p[0] * (-p[3] * t).exp() * (TAU * p[2] * t + p[1]).sin() + p[4] + p[5] * t + p[6] * t * t;
    let residuals = |p: &[f64]| -> Vec<f64> { t.iter().zip(&y).map(|(&t, y)| model(p, t) - y).collect() };
    let tp = &trend.values;
    let cost = |p: &[f64]| residuals(p).iter().map(|v| v * v).sum::<f64>();
    let start = (0..8)
        .map(|k| [amplitude, k as f64 * TAU / 8.0, fd, 0.1 * TAU * fd, tp[0], tp[1], tp[2]])
        .min_by(|a, b| cost(a).total_cmp(&cost(b)))?;
    let fit = least_squares(&start, &[level, 1.0, fd, TAU * fd, level, level / span, level / span / span], &residuals)?;
    let (wd, sigma) = (
        Estimate { value: TAU * fit.values[2], error: TAU * fit.errors[2] },
        Estimate { value: fit.values[3], error: fit.errors[3] },
    );
    let fitted: Vec<f64> = t.iter().map(|&t| model(&fit.values, t)).collect();
    Some(Ringing {
        frequency: propagate(&|v| v[0].hypot(v[1]) / TAU, &[wd, sigma]),
        damping_ratio: propagate(&|v| v[1] / v[0].hypot(v[1]), &[wd, sigma]),
        quality: quality(&y, &fitted),
    })
}

/// The two-mass model from the ringing after a torque step, which shows the resonance (motor and load
/// against each other), given the total inertia (from `friction`) and the motor's own (the rotor, from its
/// data sheet, plus what sits on the motor side of the spring). Its reduced inertia
/// μ = J_motor J_load / J gives stiffness = μ (2π f)² and damping = 2 ζ sqrt(stiffness μ).
pub fn two_mass_from_ringing(ringing: &Ringing, total: Estimate, motor: Estimate) -> TwoMass {
    let inputs = [total, motor, ringing.frequency, ringing.damping_ratio];
    let reduced = |v: &[f64]| v[1] * (v[0] - v[1]) / v[0];
    let stiffness = |v: &[f64]| reduced(v) * (TAU * v[2]).powi(2);
    TwoMass {
        total_inertia: total,
        motor_inertia: motor,
        load_inertia: propagate(&|v| v[0] - v[1], &inputs),
        stiffness: propagate(&stiffness, &inputs),
        damping: propagate(&|v| 2.0 * v[3] * (stiffness(v) * reduced(v)).sqrt(), &inputs),
        resonance: ringing.frequency,
        anti_resonance: propagate(&|v| v[2] * (v[1] / v[0]).sqrt(), &inputs),
    }
}

/// The names `Identified` gives its values and qualities, every one it can give.
pub const QUANTITIES: [&str; 19] = [
    "inertia_kg_m2",
    "viscous_nm_s_rad",
    "coulomb_nm",
    "constant_torque_nm",
    "stiction_nm",
    "stribeck_speed_rad_s",
    "presliding_stiffness_nm_rad",
    "current_loop_gain",
    "current_loop_time_constant_s",
    "current_loop_dead_time_s",
    "position_loop_gain",
    "position_loop_time_constant_s",
    "position_loop_dead_time_s",
    "friction_model_fit_percent",
    "friction_model_rms_nm",
    "current_loop_fit_percent",
    "current_loop_following_error_fit_percent",
    "position_loop_fit_percent",
    "position_loop_following_error_fit_percent",
];

/// Everything a trace of one motor gives without extra tests: inertia and friction, and the lags from torque
/// setpoint to torque (where the trace has the setpoint) and from position setpoint to position. What
/// `sim-cli identify` and the app show.
#[derive(Debug, Clone, PartialEq)]
pub struct Identified {
    pub friction: Option<Friction>,
    pub current_loop: Option<Lag>,
    pub position_loop: Option<Lag>,
}

impl Identified {
    pub fn of(time: &[f64], motor: &crate::trace::MotorTrace, torque_setpoint: Option<&[f64]>) -> Self {
        Self {
            friction: friction(time, &motor.angle, &motor.speed, &motor.torque),
            current_loop: torque_setpoint.and_then(|input| lag(time, input, &motor.torque)),
            position_loop: lag(time, &motor.setpoint, &motor.angle),
        }
    }

    /// The fitted values by name (quantity and unit, as `sim-cli identify` writes them), in a fixed order;
    /// what the trace did not show is left out.
    pub fn estimates(&self) -> Vec<(&'static str, Estimate)> {
        let mut out = Vec::new();
        if let Some(f) = &self.friction {
            out.extend([
                ("inertia_kg_m2", f.inertia),
                ("viscous_nm_s_rad", f.viscous),
                ("coulomb_nm", f.coulomb),
                ("constant_torque_nm", f.constant),
            ]);
            let optional = [
                ("stiction_nm", f.stiction),
                ("stribeck_speed_rad_s", f.stribeck_speed),
                ("presliding_stiffness_nm_rad", f.presliding_stiffness),
            ];
            out.extend(optional.into_iter().filter_map(|(name, e)| e.map(|e| (name, e))));
        }
        for (names, lag) in [
            (["current_loop_gain", "current_loop_time_constant_s", "current_loop_dead_time_s"], &self.current_loop),
            (["position_loop_gain", "position_loop_time_constant_s", "position_loop_dead_time_s"], &self.position_loop),
        ] {
            if let Some(l) = lag {
                out.extend(names.into_iter().zip([l.gain, l.time_constant, l.dead_time]));
            }
        }
        out
    }

    /// How well each model reproduces the trace, by name: fit in percent, and the friction model's RMS in N m.
    pub fn qualities(&self) -> Vec<(&'static str, f64)> {
        let mut out = Vec::new();
        if let Some(f) = &self.friction {
            out.extend([
                ("friction_model_fit_percent", f.quality.fit_percent),
                ("friction_model_rms_nm", f.quality.rms),
            ]);
        }
        for (names, lag) in [
            (["current_loop_fit_percent", "current_loop_following_error_fit_percent"], &self.current_loop),
            (["position_loop_fit_percent", "position_loop_following_error_fit_percent"], &self.position_loop),
        ] {
            if let Some(l) = lag {
                out.extend(names.into_iter().zip([l.quality.fit_percent, l.error_quality.fit_percent]));
            }
        }
        out
    }
}
