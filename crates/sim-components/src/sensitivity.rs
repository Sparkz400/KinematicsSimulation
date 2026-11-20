//! Global sensitivity of a cycle's results to the uncertain values of a config's `[bounds]`: which unknown parameters
//! matter, with all of them moving at once (the one-at-a-time `analysis::sensitivity` and `verdict_bands` move one).
//!
//! Two methods on the unit cube, each parameter's range mapped to [0, 1]:
//!
//! - Morris' elementary effects (Morris 1991, with the mean of the absolute effects of Campolongo et al. 2007): a cheap
//!   screening, `trajectories` (k + 1) runs. Per parameter μ* (how much it moves the result, over its whole range) and σ
//!   (how much that depends on the others: interaction or a curve).
//! - Sobol' indices (Saltelli 2002 sampling, Jansen 1999 estimators): first-order S_i (the share of the result's
//!   variance the parameter explains alone) and total S_Ti (with all its interactions), `samples` (k + 2) runs, with
//!   bootstrap 95 % confidence intervals.
//!
//! The results of a run are the verdicts' margins (`margin`: the room to the limit, in the check's unit) and per motor the peak and RMS torque and the winding's rise, and the tool's path error (peak and RMS) and its error in
//! the dwells (settling). Each result is ranked on its own: the parameters at its top are the ones to measure first.

use crate::analysis::{
    AT_LEAST, CycleLoad, Verdict, analyse_cycle_with, followed, on_all_cores, set_bounded, verdicts,
};
use crate::sim::Sim;
use sim_core::testkit::Noise;
use sim_core::{Config, ConfigError, MachineEntry};

/// The levels of Morris' grid: 0, 1/3, 2/3, 1 of each range; every step moves a parameter by `MORRIS_STEP`.
pub const MORRIS_LEVELS: usize = 4;
/// Morris' step, p / (2 (p - 1)) of the range: with it every level is visited equally often.
pub const MORRIS_STEP: f64 = MORRIS_LEVELS as f64 / (2.0 * (MORRIS_LEVELS as f64 - 1.0));
/// How many bootstrap resamples the Sobol' confidence intervals are taken from.
pub const BOOTSTRAP: usize = 200;

/// One parameter's elementary effects on one result, in the result's unit per the parameter's whole range.
#[derive(Debug, Clone, PartialEq)]
pub struct MorrisRow {
    pub parameter: String,
    /// Mean of the absolute effects: the ranking figure.
    pub mu_star: f64,
    /// Mean of the effects: near 0 with a large μ* where the sign changes over the range.
    pub mu: f64,
    /// Standard deviation of the effects: how much the effect depends on where the others are.
    pub sigma: f64,
}

/// One parameter's Sobol' indices on one result, with their bootstrap 95 % intervals (low, high).
#[derive(Debug, Clone, PartialEq)]
pub struct SobolRow {
    pub parameter: String,
    pub first: f64,
    pub first_ci: (f64, f64),
    /// The ranking figure.
    pub total: f64,
    pub total_ci: (f64, f64),
}

/// The rows of one result, the parameter that matters most first.
#[derive(Debug, Clone, PartialEq)]
pub struct Ranking<T> {
    pub result: String,
    pub rows: Vec<T>,
}

/// The points of `trajectories` Morris trajectories in [0, 1]^k, k + 1 points each: a random start on the grid's lower
/// half, then each parameter in a random order stepped once by `MORRIS_STEP`, up or down at random. Pseudo-random from
/// `seed`.
pub fn morris_points(k: usize, trajectories: usize, seed: u64) -> Vec<Vec<f64>> {
    let mut random = Noise(seed);
    let mut pick = |n: usize| ((random.uniform() * n as f64) as usize).min(n - 1);
    let mut points = Vec::with_capacity(trajectories * (k + 1));
    for _ in 0..trajectories {
        // Each parameter's two ends: the lower one from the levels a step still fits above.
        let (mut at, mut to) = (vec![0.0; k], vec![0.0; k]);
        for i in 0..k {
            let low = pick(MORRIS_LEVELS / 2) as f64 / (MORRIS_LEVELS - 1) as f64;
            (at[i], to[i]) = if pick(2) == 0 { (low, low + MORRIS_STEP) } else { (low + MORRIS_STEP, low) };
        }
        let mut order: Vec<usize> = (0..k).collect();
        for i in (1..k).rev() {
            order.swap(i, pick(i + 1));
        }
        points.push(at.clone());
        for i in order {
            at[i] = to[i];
            points.push(at.clone());
        }
    }
    points
}

/// (μ*, μ, σ) per parameter of one result, from its values `y` at `morris_points(k, ..)`. The parameter a step moved is
/// read from the points.
pub fn morris_indices(k: usize, points: &[Vec<f64>], y: &[f64]) -> Vec<(f64, f64, f64)> {
    let mut effects = vec![Vec::new(); k];
    for (pair, out) in points.chunks(k + 1).zip(y.chunks(k + 1)) {
        for s in 0..k {
            let i = (0..k).find(|&i| pair[s + 1][i] != pair[s][i]).expect("a Morris step moves one parameter");
            effects[i].push((out[s + 1] - out[s]) / (pair[s + 1][i] - pair[s][i]));
        }
    }
    effects
        .iter()
        .map(|e| {
            let r = e.len() as f64;
            let mu = e.iter().sum::<f64>() / r;
            let mu_star = e.iter().map(|d| d.abs()).sum::<f64>() / r;
            let sigma =
                if e.len() > 1 { (e.iter().map(|d| (d - mu).powi(2)).sum::<f64>() / (r - 1.0)).sqrt() } else { 0.0 };
            (mu_star, mu, sigma)
        })
        .collect()
}

/// Saltelli's points in [0, 1]^k: n rows of A, n rows of B (both uniform at random from `seed`), then for each
/// parameter i the n rows of A with column i taken from B. n (k + 2) points.
pub fn saltelli_points(k: usize, n: usize, seed: u64) -> Vec<Vec<f64>> {
    let mut random = Noise(seed);
    let mut draw = || (0..n).map(|_| (0..k).map(|_| random.uniform()).collect::<Vec<f64>>()).collect::<Vec<_>>();
    let (a, b) = (draw(), draw());
    let mut points = [a.clone(), b.clone()].concat();
    for i in 0..k {
        points.extend(a.iter().zip(&b).map(|(a, b)| {
            let mut ab = a.clone();
            ab[i] = b[i];
            ab
        }));
    }
    points
}

/// Jansen's first-order and total index of each parameter, from the values at the rows `pick` of `saltelli_points`.
fn jansen(k: usize, n: usize, y: &[f64], pick: &[usize]) -> Vec<(f64, f64)> {
    let m = pick.len() as f64;
    let (a, b) = (|j: usize| y[j], |j: usize| y[n + j]);
    let mean = pick.iter().map(|&j| a(j) + b(j)).sum::<f64>() / (2.0 * m);
    let variance = pick.iter().map(|&j| (a(j) - mean).powi(2) + (b(j) - mean).powi(2)).sum::<f64>() / (2.0 * m);
    (0..k)
        .map(|i| {
            if variance <= 0.0 {
                return (0.0, 0.0);
            }
            let ab = |j: usize| y[(2 + i) * n + j];
            // Jansen 1999: V_Ti = E[(f(A) - f(AB_i))^2] / 2, V - V_i = E[(f(B) - f(AB_i))^2] / 2.
            let total = pick.iter().map(|&j| (a(j) - ab(j)).powi(2)).sum::<f64>() / (2.0 * m);
            let rest = pick.iter().map(|&j| (b(j) - ab(j)).powi(2)).sum::<f64>() / (2.0 * m);
            ((variance - rest) / variance, total / variance)
        })
        .collect()
}

/// (S_i, its interval, S_Ti, its interval) per parameter of one result, from its values `y` at `saltelli_points(k, n,
/// ..)`. The intervals are the 2.5 % and 97.5 % percentiles of `bootstrap` resamples of the n rows (with replacement,
/// pseudo-random from `seed`). A result that does not vary has all indices 0.
pub fn sobol_indices(k: usize, n: usize, y: &[f64], bootstrap: usize, seed: u64) -> Vec<SobolIndex> {
    let all: Vec<usize> = (0..n).collect();
    let point = jansen(k, n, y, &all);
    let mut random = Noise(seed);
    let resamples: Vec<Vec<(f64, f64)>> = (0..bootstrap)
        .map(|_| {
            let pick: Vec<usize> = (0..n).map(|_| ((random.uniform() * n as f64) as usize).min(n - 1)).collect();
            jansen(k, n, y, &pick)
        })
        .collect();
    let interval = |mut v: Vec<f64>| -> (f64, f64) {
        if v.is_empty() {
            return (f64::NAN, f64::NAN);
        }
        v.sort_by(f64::total_cmp);
        let at = |q: f64| v[(q * (v.len() - 1) as f64).round() as usize];
        (at(0.025), at(0.975))
    };
    (0..k)
        .map(|i| SobolIndex {
            first: point[i].0,
            first_ci: interval(resamples.iter().map(|r| r[i].0).collect()),
            total: point[i].1,
            total_ci: interval(resamples.iter().map(|r| r[i].1).collect()),
        })
        .collect()
}

/// One parameter's Sobol' indices, without its name (`sobol_indices`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SobolIndex {
    pub first: f64,
    pub first_ci: (f64, f64),
    pub total: f64,
    pub total_ci: (f64, f64),
}

/// A verdict's margin: the room its value leaves to its limit, in the check's unit, negative where it fails. Not the
/// utilisation (value over limit): "belt-least-tension" has the limit 0, where that is 0 whatever the tension. A bound on a rating
/// moves the margin through the limit.
pub fn margin(v: &Verdict) -> f64 {
    if AT_LEAST.contains(&v.check) { v.value - v.limit } else { v.limit - v.value }
}

/// The results of one run, by name (see the module): verdicts as "check" or "check:motor", the figures as
/// "peak-torque:motor", "rms-torque:motor", "winding-rise:motor", "path-error-peak", "path-error-rms", "dwell-error-peak".
fn results(load: &CycleLoad, judged: &[Verdict], motors: &[String]) -> Vec<(String, f64)> {
    let named = |what: &str, motor: Option<usize>| match motor {
        Some(k) => format!("{what}:{}", motors[k]),
        None => what.to_string(),
    };
    let mut out: Vec<(String, f64)> = judged.iter().map(|v| (named(v.check, v.motor), margin(v))).collect();
    for (k, m) in load.motors.iter().enumerate() {
        out.push((named("peak-torque", Some(k)), m.peak_torque));
        out.push((named("rms-torque", Some(k)), m.rms_torque));
        out.push((named("winding-rise", Some(k)), m.winding_rise));
    }
    out.push(("path-error-peak".into(), load.path_error_peak));
    out.push(("path-error-rms".into(), load.path_error_rms));
    out.push(("dwell-error-peak".into(), load.dwell_error_peak));
    out
}

/// The cycle of `cfg` with the parameters of `bounds` at each point of the unit cube (0: low bound, 1: high), on all
/// processors. Per result that came up in every run with a finite value its values, in the order of the points; a
/// result missing from a run (a check only some values bring up) or not finite in one is left out.
fn run_points(
    entry: &MachineEntry,
    cfg: &Config,
    bounds: &[(String, f64, f64)],
    points: &[Vec<f64>],
) -> Result<Vec<(String, Vec<f64>)>, ConfigError> {
    let resolved = &crate::catalog::resolve(entry, cfg)?;
    let (bounds, cfg) = (&followed(cfg, resolved, bounds), resolved);
    let motors: Vec<String> = (entry.build)(cfg)?.motors().iter().map(|m| m.id.to_string()).collect();
    let runs = on_all_cores(points.len(), |r| -> Result<Vec<(String, f64)>, ConfigError> {
        let mut changed = cfg.clone();
        for ((path, low, high), u) in bounds.iter().zip(&points[r]) {
            set_bounded(&mut changed, path, low + (high - low) * u)?;
        }
        let load = analyse_cycle_with(&mut Sim::new(entry, &changed)?, false)
            .map_err(|e| ConfigError::Invalid(e.key.into()))?;
        Ok(results(&load, &verdicts(&changed, &load)?, &motors))
    });
    let mut table: Vec<(String, Vec<f64>)> = Vec::new();
    for (r, run) in runs.into_iter().enumerate() {
        for (name, value) in run? {
            match table.iter_mut().find(|(n, _)| *n == name) {
                Some((_, values)) => values[r] = value,
                None => {
                    let mut values = vec![f64::NAN; points.len()];
                    values[r] = value;
                    table.push((name, values));
                }
            }
        }
    }
    table.retain(|(_, values)| values.iter().all(|v| v.is_finite()));
    Ok(table)
}

/// Morris screening of the cycle's results (see the module) over `bounds` (path, low, high; see `analysis::bounds`):
/// `trajectories` (k + 1) runs on all processors. Per result the parameters by μ*, largest first. The same seed gives
/// the same ranking.
pub fn morris(
    entry: &MachineEntry,
    cfg: &Config,
    bounds: &[(String, f64, f64)],
    trajectories: usize,
    seed: u64,
) -> Result<Vec<Ranking<MorrisRow>>, ConfigError> {
    if trajectories == 0 || bounds.is_empty() {
        return Err(ConfigError::Invalid(if bounds.is_empty() { "bounds" } else { "samples" }.into()));
    }
    let k = bounds.len();
    let points = morris_points(k, trajectories, seed);
    Ok(run_points(entry, cfg, bounds, &points)?
        .into_iter()
        .map(|(result, y)| {
            let mut rows: Vec<MorrisRow> = morris_indices(k, &points, &y)
                .into_iter()
                .zip(bounds)
                .map(|((mu_star, mu, sigma), b)| MorrisRow { parameter: b.0.clone(), mu_star, mu, sigma })
                .collect();
            rows.sort_by(|a, b| b.mu_star.total_cmp(&a.mu_star));
            Ranking { result, rows }
        })
        .collect())
}

/// Sobol' indices of the cycle's results (see the module) over `bounds`: `samples` (k + 2) runs on all processors, the
/// intervals from `BOOTSTRAP` resamples. Per result the parameters by their total index, largest first. The same seed
/// gives the same indices.
pub fn sobol(
    entry: &MachineEntry,
    cfg: &Config,
    bounds: &[(String, f64, f64)],
    samples: usize,
    seed: u64,
) -> Result<Vec<Ranking<SobolRow>>, ConfigError> {
    if samples < 2 || bounds.is_empty() {
        return Err(ConfigError::Invalid(if bounds.is_empty() { "bounds" } else { "samples" }.into()));
    }
    let k = bounds.len();
    let points = saltelli_points(k, samples, seed);
    Ok(run_points(entry, cfg, bounds, &points)?
        .into_iter()
        .map(|(result, y)| {
            let mut rows: Vec<SobolRow> = sobol_indices(k, samples, &y, BOOTSTRAP, seed)
                .into_iter()
                .zip(bounds)
                .map(|(s, b)| SobolRow {
                    parameter: b.0.clone(),
                    first: s.first,
                    first_ci: s.first_ci,
                    total: s.total,
                    total_ci: s.total_ci,
                })
                .collect();
            rows.sort_by(|a, b| b.total.total_cmp(&a.total));
            Ranking { result, rows }
        })
        .collect())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Ishigami & Homma 1990 on [-pi, pi]^3 with a = 7, b = 0.1, the unit cube mapped onto it.
    fn ishigami(u: &[f64]) -> f64 {
        let x: Vec<f64> = u.iter().map(|u| -PI + 2.0 * PI * u).collect();
        x[0].sin() + 7.0 * x[1].sin().powi(2) + 0.1 * x[2].powi(4) * x[0].sin()
    }

    /// Its indices by hand (Sobol' & Levitan 1999; Saltelli et al. 2008, p. 179):
    /// V1 = 1/2 (1 + b pi^4 / 5)^2 = 1/2 (1 + 9.7409 / 5)^2 = 4.3459; V2 = a^2 / 8 = 6.125;
    /// V13 = b^2 pi^8 (1/18 - 1/50) = 0.01 * 9488.53 * 0.035556 = 3.3737; V = V1 + V2 + V13 = 13.8446 (V3 = 0).
    /// S1 = 0.3139, S2 = 0.4424, S3 = 0; ST1 = (V1 + V13) / V = 0.5576, ST2 = 0.4424, ST3 = V13 / V = 0.2437.
    const ISHIGAMI_FIRST: [f64; 3] = [0.3139, 0.4424, 0.0];
    const ISHIGAMI_TOTAL: [f64; 3] = [0.5576, 0.4424, 0.2437];

    #[test]
    fn sobol_finds_the_ishigami_indices() {
        let (k, n) = (3, 1 << 14);
        let y: Vec<f64> = saltelli_points(k, n, 7).iter().map(|u| ishigami(u)).collect();
        let s = sobol_indices(k, n, &y, 100, 7);
        for i in 0..k {
            assert!((s[i].first - ISHIGAMI_FIRST[i]).abs() < 0.03, "S{} = {}", i + 1, s[i].first);
            assert!((s[i].total - ISHIGAMI_TOTAL[i]).abs() < 0.03, "ST{} = {}", i + 1, s[i].total);
            assert!(s[i].first_ci.0 <= s[i].first_ci.1 && s[i].total_ci.0 <= s[i].total_ci.1);
            // Intervals of a few hundredths at 16384 rows, and around the estimate.
            assert!(s[i].total_ci.1 - s[i].total_ci.0 < 0.06, "{:?}", s[i].total_ci);
            assert!(s[i].total_ci.0 <= s[i].total + 0.01 && s[i].total - 0.01 <= s[i].total_ci.1);
        }
    }

    /// Y = sum a_i X_i with X_i uniform on [lo_i, hi_i]: Var(X_i) = (hi_i - lo_i)^2 / 12 and
    /// S_i = S_Ti = a_i^2 Var_i / sum_j a_j^2 Var_j. With a = (1, -2, 0.5, 0), ranges 1, 1, 4, 3:
    /// a_i^2 Var_i * 12 = 1, 4, 4, 0, sum 9, so S = (1/9, 4/9, 4/9, 0) = (0.1111, 0.4444, 0.4444, 0).
    const A: [f64; 4] = [1.0, -2.0, 0.5, 0.0];
    const RANGE: [(f64, f64); 4] = [(0.0, 1.0), (2.0, 3.0), (-2.0, 2.0), (5.0, 8.0)];
    fn linear(u: &[f64]) -> f64 {
        u.iter().zip(A).zip(RANGE).map(|((u, a), (lo, hi))| a * (lo + (hi - lo) * u)).sum()
    }

    #[test]
    fn sobol_finds_the_shares_of_a_linear_model() {
        let (k, n) = (4, 1 << 13);
        let y: Vec<f64> = saltelli_points(k, n, 3).iter().map(|u| linear(u)).collect();
        let s = sobol_indices(k, n, &y, 50, 3);
        for (i, want) in [1.0 / 9.0, 4.0 / 9.0, 4.0 / 9.0, 0.0].into_iter().enumerate() {
            assert!((s[i].first - want).abs() < 0.03, "S{} = {} against {want}", i + 1, s[i].first);
            // A model without interactions: the total index is the first-order one.
            assert!((s[i].total - want).abs() < 0.02, "ST{} = {} against {want}", i + 1, s[i].total);
        }
        // X4 does nothing: its total index is exactly 0 (f(A) = f(AB_4) row by row).
        assert_eq!(s[3].total, 0.0);
    }

    #[test]
    fn morris_on_a_linear_model_gives_the_slope_times_the_range_with_no_spread() {
        // Each effect is exact: a_i (hi_i - lo_i) per whole range, so mu* = |a_i| (hi_i - lo_i) = 1, 2, 2, 0, sigma = 0.
        let k = 4;
        let points = morris_points(k, 10, 11);
        assert_eq!(points.len(), 10 * (k + 1));
        let y: Vec<f64> = points.iter().map(|u| linear(u)).collect();
        for ((mu_star, mu, sigma), want) in morris_indices(k, &points, &y).into_iter().zip([1.0f64, -2.0, 2.0, 0.0]) {
            assert!((mu_star - want.abs()).abs() < 1e-9 && (mu - want).abs() < 1e-9 && sigma < 1e-9);
        }
    }

    #[test]
    fn morris_sees_the_interaction_of_the_ishigami_function() {
        // x3 explains nothing alone (S3 = 0) but acts through x1 (ST3 = 0.24): Morris gives it a mu* well above 0 and a
        // sigma as large, and x2 (no interaction, but a curve: sin^2) a mu* well above it.
        let k = 3;
        let points = morris_points(k, 200, 5);
        let y: Vec<f64> = points.iter().map(|u| ishigami(u)).collect();
        let m = morris_indices(k, &points, &y);
        assert!(m[2].0 > 1.0 && m[2].2 > 1.0, "x3: {:?}", m[2]);
        assert!(m[1].0 > m[2].0, "{m:?}");
    }

    #[test]
    fn the_same_seed_gives_the_same_points_and_another_seed_others() {
        assert_eq!(morris_points(5, 4, 1), morris_points(5, 4, 1));
        assert_ne!(morris_points(5, 4, 1), morris_points(5, 4, 2));
        assert_eq!(saltelli_points(3, 8, 1), saltelli_points(3, 8, 1));
        let y: Vec<f64> = saltelli_points(3, 256, 1).iter().map(|u| ishigami(u)).collect();
        assert_eq!(sobol_indices(3, 256, &y, 50, 1), sobol_indices(3, 256, &y, 50, 1));
    }

    #[test]
    fn every_morris_point_lies_on_the_grid_and_every_step_moves_one_parameter_by_the_step() {
        let k = 6;
        for t in morris_points(k, 20, 9).chunks(k + 1) {
            for p in t.iter().flatten() {
                assert!((p * 3.0 - (p * 3.0).round()).abs() < 1e-12 && (0.0..=1.0).contains(p));
            }
            let mut moved = vec![false; k];
            for s in 0..k {
                let changed: Vec<usize> = (0..k).filter(|&i| t[s + 1][i] != t[s][i]).collect();
                assert_eq!(changed.len(), 1);
                assert!(((t[s + 1][changed[0]] - t[s][changed[0]]).abs() - MORRIS_STEP).abs() < 1e-12);
                moved[changed[0]] = true;
            }
            assert!(moved.iter().all(|m| *m));
        }
    }
}
