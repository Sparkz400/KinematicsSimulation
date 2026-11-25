//! The catalogue search as a workflow: which drive works best with which path and kinematics. `catalog::search` tries
//! the parts; this module gives each candidate the figures of a wide table (`Figures`), judges it over a duty mix
//! (`mix_verdicts`), ranks by one criterion or a weighted mix of them (`rank_by`), marks the non-dominated ones
//! (`front`), sweeps a gear ratio or pulley radius for the optimum (`transmission_sweep`), ranks the best again in the
//! error bands (`robust_ranking`), and runs one tool-space task on several kinematics (`compare_kinematics`).

use crate::analysis::{CycleLoad, MotorLoad, Verdict, supply_energy, utilisation};
use crate::catalog::{self, Candidate, Options};
use crate::drive::DriveParams;
use sim_core::{Config, ConfigError, MachineEntry};

/// One candidate's figures beside its parts, the columns of the search's table. A figure the run does not give is NaN.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Figures {
    /// Per check (verdict id) its utilisation (`analysis::utilisation`, above 1 it fails), the highest over the motors
    /// (for one motor's search, `Candidate::motor`, that motor's and the machine's), in the order they come up.
    pub checks: Vec<(&'static str, f64)>,
    /// The checks that fail, in that order.
    pub failing: Vec<&'static str>,
    /// Energy from the supply per cycle, J (`analysis::supply_energy`); over a duty mix per cycle of the mix, weighted.
    pub energy_j: f64,
    /// RMS torque over continuous torque (`MotorLoad::thermal_utilisation`), the highest of the motors; over a duty mix
    /// the highest the motor's node takes it to (`mix_verdicts_with`, as "motor-s1").
    pub thermal: f64,
    /// The drive's RMS current over its rated current ("drive-current"; over a mix as `mix_verdicts_with` has it).
    pub drive_current: f64,
    /// The most the tool lagged behind its setpoint (`CycleLoad::tracking_error_peak`), m; the worst duty's.
    pub following_error_m: f64,
    /// The most the tool was off its target late in a dwell (`CycleLoad::dwell_error_peak`), m; the worst duty's.
    pub settling_error_m: f64,
    /// Load over rotor inertia (`catalog::inertia_ratios`), the highest of the motors (or the one motor's).
    pub inertia_ratio: f64,
    /// Cycle time, s; over a duty mix the mean per cycle, weighted.
    pub cycle_s: f64,
    /// What the motor and drive leave to the config's estimates (`catalog::estimated_data`); empty: all from the catalogue.
    pub estimated: Vec<&'static str>,
}

/// Whether a verdict concerns the motor `motor` of a load (by id), or the machine; every one for `motor` "".
fn concerns(load: &CycleLoad, motor: &str, v: &Verdict) -> bool {
    motor.is_empty() || v.motor.is_none_or(|k| load.motors.get(k).is_some_and(|m| m.id == motor))
}

/// The time-weighted RMS of a figure over a duty mix: (weight, the figure's RMS over the cycle, the cycle's time) per
/// duty, sqrt(sum w RMS^2 t / sum w t). Each duty runs its cycle w times as often as one of weight 1. NaN for no time.
pub fn mix_rms(parts: &[(f64, f64, f64)]) -> f64 {
    let time: f64 = parts.iter().map(|(w, _, t)| w * t).sum();
    (parts.iter().map(|(w, rms, t)| w * rms * rms * t).sum::<f64>() / time).sqrt()
}

/// The time-weighted mean of a figure over a duty mix, (weight, figure, time) per duty: sum w x t / sum w t.
pub fn mix_mean(parts: &[(f64, f64, f64)]) -> f64 {
    let time: f64 = parts.iter().map(|(w, _, t)| w * t).sum();
    parts.iter().map(|(w, x, t)| w * x * t).sum::<f64>() / time
}

/// One stretch of a sequence that repeats, on a linear model of two states: held for `len`, x' = a x + b.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stretch {
    pub len: f64,
    pub a: [[f64; 2]; 2],
    pub b: [f64; 2],
}

impl Stretch {
    /// A first-order node of time constant `tau` that settles at `target` (the second state is not used).
    pub fn node(len: f64, tau: f64, target: f64) -> Self {
        Self { len, a: [[-1.0 / tau, 0.0], [0.0, -2.0 / tau]], b: [target / tau, 0.0] }
    }
}

/// The two eigenvalues m + d and m - d of `a` (real, as a thermal network's are; d clamped at 0), and the weights of
/// e^{a t} = p I + q (a - m I) at `t`: p = (e^{(m+d) t} + e^{(m-d) t}) / 2, q = (e^{(m+d) t} - e^{(m-d) t}) / (2 d),
/// which is t e^{m t} for d t near 0 (Cayley-Hamilton).
fn modes(a: &[[f64; 2]; 2], t: f64) -> (f64, f64, f64, f64) {
    let m = 0.5 * (a[0][0] + a[1][1]);
    let d = (m * m - (a[0][0] * a[1][1] - a[0][1] * a[1][0])).max(0.0).sqrt();
    let (up, down) = (((m + d) * t).exp(), ((m - d) * t).exp());
    let q = if d * t > 1e-9 { (up - down) / (2.0 * d) } else { t * (m * t).exp() };
    (m, d, 0.5 * (up + down), q)
}

/// The peak of `out` . x over the periodic state of a linear model of two states driven through `parts` in order, over
/// and over (each a `Stretch`). Over a stretch x = x* + e^{a t} (x0 - x*), x* = -a^-1 b, so a period maps x0 to
/// M x0 + v and the periodic state is (I - M)^-1 v. Within a stretch out . x = y* + c+ e^{(m+d) t} + c- e^{(m-d) t}
/// (`modes`), which has at most one turning point, where e^{2 d t} = -c- (m-d) / (c+ (m+d)); the peak is at a stretch's
/// end or there. NaN for no stretch.
pub fn periodic_peak(parts: &[Stretch], out: [f64; 2]) -> f64 {
    let settled = |s: &Stretch| {
        let det = s.a[0][0] * s.a[1][1] - s.a[0][1] * s.a[1][0];
        [(s.a[0][1] * s.b[1] - s.a[1][1] * s.b[0]) / det, (s.a[1][0] * s.b[0] - s.a[0][0] * s.b[1]) / det]
    };
    let flow = |s: &Stretch, t: f64| {
        let (m, _, p, q) = modes(&s.a, t);
        [[p + q * (s.a[0][0] - m), q * s.a[0][1]], [q * s.a[1][0], p + q * (s.a[1][1] - m)]]
    };
    let times = |e: &[[f64; 2]; 2], x: [f64; 2]| [e[0][0] * x[0] + e[0][1] * x[1], e[1][0] * x[0] + e[1][1] * x[1]];
    // Over one period x_end = mm x0 + v.
    let (mut mm, mut v) = ([[1.0, 0.0], [0.0, 1.0]], [0.0, 0.0]);
    for s in parts {
        let (e, xs) = (flow(s, s.len), settled(s));
        let ev = times(&e, [v[0] - xs[0], v[1] - xs[1]]);
        let column = |j: usize| times(&e, [mm[0][j], mm[1][j]]);
        let (c0, c1) = (column(0), column(1));
        (mm, v) = ([[c0[0], c1[0]], [c0[1], c1[1]]], [ev[0] + xs[0], ev[1] + xs[1]]);
    }
    let (i00, i01, i10, i11) = (1.0 - mm[0][0], -mm[0][1], -mm[1][0], 1.0 - mm[1][1]);
    let det = i00 * i11 - i01 * i10;
    let mut x = [(i11 * v[0] - i01 * v[1]) / det, (i00 * v[1] - i10 * v[0]) / det];
    let y = |x: [f64; 2]| out[0] * x[0] + out[1] * x[1];
    let mut peak = if parts.is_empty() { f64::NAN } else { y(x) };
    for s in parts {
        let xs = settled(s);
        let dx = [x[0] - xs[0], x[1] - xs[1]];
        let (m, d, ..) = modes(&s.a, 0.0);
        let (u, w) = (y(dx), y(times(&[[s.a[0][0] - m, s.a[0][1]], [s.a[1][0], s.a[1][1] - m]], dx)));
        // The turning point of y* + p u + q w (`modes`), where it lies within the stretch.
        let turn = if d > 1e-12 * m.abs() {
            let (up, down) = (0.5 * (u + w / d) * (m + d), 0.5 * (u - w / d) * (m - d));
            (up != 0.0 && -down / up > 0.0).then(|| (-down / up).ln() / (2.0 * d))
        } else {
            (w != 0.0).then(|| -(m * u + w) / (m * w))
        };
        if let Some(t) = turn.filter(|t| *t > 0.0 && *t < s.len) {
            let at = times(&flow(s, t), dx);
            peak = peak.max(y([at[0] + xs[0], at[1] + xs[1]]));
        }
        let at = times(&flow(s, s.len), dx);
        x = [at[0] + xs[0], at[1] + xs[1]];
        peak = peak.max(y(x));
    }
    peak
}

/// The winding's model (`Drive::warm`) held for `len` at a duty that settles it at `rise` with the mean iron loss `iron`
/// (W, effects.iron_loss): in rises it would settle at alone, q the copper heating at no rise and h the iron's, the
/// copper's resistance rising by c per kelvin: one node tau T' = q (1 + c T) + h - T, or with effects.motor_thermal_nodes
/// the winding tau_w T_w' = s q (1 + c T_w) - (T_w - T_h) and the housing tau_h T_h' = (T_w - T_h) / s - T_h / (1 - s) + h,
/// tau_h = tau - tau_w / s. Both settle at T_w = (q + h_w) / (1 - c q) (`DriveParams::settled_rise`; h_w = h, or (1 - s) h
/// with two nodes), so q = (rise - h_w) / (1 + c rise).
pub fn winding_stretch(p: &DriveParams, len: f64, rise: f64, iron: f64) -> Stretch {
    let (c, h) = (crate::drive::COPPER, p.iron_heating(iron));
    match &p.thermal_nodes {
        None => {
            let (q, tau) = ((rise - h) / (1.0 + c * rise), p.thermal_time_constant);
            Stretch { len, a: [[(c * q - 1.0) / tau, 0.0], [0.0, -2.0 / tau]], b: [(q + h) / tau, 0.0] }
        }
        Some(n) => {
            let (s, tw) = (n.winding_share, n.winding_time_constant);
            let (q, th) = ((rise - (1.0 - s) * h) / (1.0 + c * rise), p.thermal_time_constant - tw / s);
            Stretch {
                len,
                a: [[(s * q * c - 1.0) / tw, 1.0 / tw], [1.0 / (s * th), -(1.0 / s + 1.0 / (1.0 - s)) / th]],
                b: [s * q / tw, h / th],
            }
        }
    }
}

/// What the heat of a duty mix is worked out with, per motor: its drive model (the winding's nodes, `winding_stretch`)
/// and the drive's thermal image as `drive::Overload` has it, [tau, chip share, chip tau]
/// (`ratings.drive_overload_time_constant_s`, `drive_chip_share`, `drive_chip_time_constant_s`; no chips where the
/// config lacks the share), `None` where the config has no time constant for it.
#[derive(Debug, Clone)]
pub struct MixHeat {
    pub drive: DriveParams,
    pub image: Option<[f64; 3]>,
}

impl MixHeat {
    /// Motor `id`'s (`catalog::motor_config`).
    pub fn of(cfg: &Config, id: &str) -> Result<Self, ConfigError> {
        let own = catalog::motor_config(cfg, id)?;
        let image = match own.positive("ratings.drive_overload_time_constant_s") {
            Ok(tau) if own.get("ratings.drive_chip_share").is_ok() => Some([
                tau,
                own.non_negative("ratings.drive_chip_share")?,
                own.positive("ratings.drive_chip_time_constant_s")?,
            ]),
            Ok(tau) => Some([tau, 0.0, tau]),
            Err(_) => None,
        };
        Ok(Self { drive: DriveParams::from_config(&own, 1.0, 1.0)?, image })
    }
}

/// The verdicts of a duty mix on `cfg` (the first duty's; the motors and drives are the same in all), (weight, cycle,
/// verdicts) per duty: `mix_verdicts_with` each motor's `MixHeat`.
pub fn mix_verdicts(cfg: &Config, runs: &[(f64, CycleLoad, Vec<Verdict>)]) -> Result<Vec<Verdict>, ConfigError> {
    let Some((_, first, _)) = runs.first() else { return Ok(Vec::new()) };
    let heats = first.motors.iter().map(|m| MixHeat::of(cfg, &m.id)).collect::<Result<Vec<_>, _>>()?;
    Ok(mix_verdicts_with(&heats, runs))
}

/// A figure of a mix whose square is a heat that a first-order node of time constant `tau` takes up, (duration, the
/// figure) per duty: the root of the node's periodic peak (`periodic_peak`). One duty: its own figure.
fn heat_peak(parts: &[(f64, f64)], tau: f64) -> f64 {
    let nodes: Vec<Stretch> = parts.iter().map(|&(len, x)| Stretch::node(len, tau, x * x)).collect();
    periodic_peak(&nodes, [1.0, 0.0]).sqrt()
}

/// The verdicts of a duty mix, (weight, cycle, verdicts) per duty, each motor's heat with `heats` (in the order of the
/// cycles' motors): per check and motor the worst duty's (failing before passing, then the highest utilisation), but the
/// heat over the mix, as the duties take turns, each for its weight times its cycle time and in the order given, over
/// and over (the periodic state, `periodic_peak`): "motor-thermal" the highest winding rise, the winding's model driven
/// by each duty's (`winding_stretch`, with its iron loss); "drive-current" the root of the highest of the drive's
/// thermal image driven by each duty's mean square current, where the config has its time constant (else the
/// time-weighted RMS, `mix_rms`); "motor-s1" the highest S1 utilisation, each duty's RMS torque over its S1 limit
/// squared into the motor's one node (`motor.thermal_time_constant_s`), shown as that share of the time-weighted mean
/// of the duties' limits. Duties much shorter than the time constants give the heat of the mix's mean, much longer
/// ones the worst duty's own. The values are the duties' verdicts', so worst-corner verdicts mix the same way
/// (`robust_ranking`).
pub fn mix_verdicts_with(heats: &[MixHeat], runs: &[(f64, CycleLoad, Vec<Verdict>)]) -> Vec<Verdict> {
    let mut out: Vec<Verdict> = Vec::new();
    for v in runs.iter().flat_map(|(_, _, verdicts)| verdicts) {
        // Failing before passing, then the higher utilisation.
        let worse = |v: &Verdict, o: &Verdict| (!v.ok && o.ok) || (v.ok == o.ok && utilisation(v) > utilisation(o));
        match out.iter_mut().find(|o| o.check == v.check && o.motor == v.motor) {
            Some(o) => {
                if worse(v, o) {
                    *o = v.clone();
                }
            }
            None => out.push(v.clone()),
        }
    }
    for o in &mut out {
        let Some((k, heat)) = o.motor.and_then(|k| Some((k, heats.get(k)?))) else { continue };
        // Per duty that judged this check: (duration, its verdict, the motor's load).
        let duties: Vec<(f64, &Verdict, &MotorLoad)> = runs
            .iter()
            .filter_map(|(w, load, verdicts)| {
                let v = verdicts.iter().find(|v| v.check == o.check && v.motor == Some(k))?;
                Some((w * load.duration, v, load.motors.get(k)?))
            })
            .collect();
        let p = &heat.drive;
        match o.check {
            "motor-thermal" => {
                o.value = if duties.iter().all(|(_, v, _)| v.value.is_finite()) {
                    let parts: Vec<Stretch> =
                        duties.iter().map(|(len, v, m)| winding_stretch(p, *len, v.value, m.iron_loss)).collect();
                    periodic_peak(&parts, [1.0, 0.0])
                } else {
                    f64::INFINITY
                };
            }
            "drive-current" => {
                o.value = match heat.image {
                    Some([tau, share, chip]) => {
                        let parts: Vec<Stretch> = duties
                            .iter()
                            .map(|(len, v, _)| {
                                let q = v.value * v.value;
                                Stretch {
                                    len: *len,
                                    a: [[-1.0 / tau, 0.0], [0.0, -1.0 / chip]],
                                    b: [(1.0 - share) * q / tau, share * q / chip],
                                }
                            })
                            .collect();
                        periodic_peak(&parts, [1.0, 1.0]).sqrt()
                    }
                    None => mix_rms(&duties.iter().map(|(len, v, _)| (1.0, v.value, *len)).collect::<Vec<_>>()),
                };
            }
            "motor-s1" => {
                o.limit = mix_mean(&duties.iter().map(|(len, v, _)| (1.0, v.limit, *len)).collect::<Vec<_>>());
                let shares: Vec<(f64, f64)> = duties.iter().map(|(len, v, _)| (*len, v.value / v.limit)).collect();
                o.value = heat_peak(&shares, p.thermal_time_constant) * o.limit;
            }
            _ => continue,
        }
        o.ok = o.value <= o.limit;
    }
    out
}

/// The figures of a candidate that ran: `cfg` its config (the first duty's), `motor` the one motor searched ("" all),
/// `runs` (weight, cycle, verdicts) per duty, `estimated` from `catalog::estimated_data`. Over a mix the verdicts are
/// `mix_verdicts`, and the thermal utilisation goes into each motor's one node as the S1 check's does.
pub fn figures(
    entry: &MachineEntry,
    cfg: &Config,
    motor: &str,
    runs: &[(f64, CycleLoad, Vec<Verdict>)],
    estimated: Vec<&'static str>,
) -> Result<Figures, ConfigError> {
    let Some((_, first, _)) = runs.first() else { return Ok(Figures::default()) };
    let verdicts = if runs.len() == 1 { runs[0].2.clone() } else { mix_verdicts(cfg, runs)? };
    let mut out = Figures { estimated, ..Figures::default() };
    for v in verdicts.iter().filter(|v| concerns(first, motor, v)) {
        let u = utilisation(v);
        match out.checks.iter_mut().find(|(check, _)| *check == v.check) {
            Some((_, most)) => *most = most.max(u),
            None => out.checks.push((v.check, u)),
        }
        if !v.ok && !out.failing.contains(&v.check) {
            out.failing.push(v.check);
        }
    }
    let ours = |k: &usize| motor.is_empty() || first.motors[*k].id == motor;
    let weight: f64 = runs.iter().map(|(w, ..)| w).sum();
    let highest = |values: Vec<f64>| values.into_iter().filter(|x| !x.is_nan()).reduce(f64::max).unwrap_or(f64::NAN);
    out.energy_j = runs.iter().map(|(w, load, _)| w * supply_energy(cfg, load)).sum::<f64>() / weight;
    out.cycle_s = runs.iter().map(|(w, load, _)| w * load.duration).sum::<f64>() / weight;
    let mut thermal = Vec::new();
    for (k, m) in first.motors.iter().enumerate().filter(|(k, _)| ours(k)) {
        let tau = catalog::motor_config(cfg, &m.id)?.positive("motor.thermal_time_constant_s")?;
        let parts: Vec<(f64, f64)> = runs
            .iter()
            .filter_map(|(w, load, _)| load.motors.get(k).map(|m| (w * load.duration, m.thermal_utilisation)))
            .collect();
        thermal.push(heat_peak(&parts, tau));
    }
    out.thermal = highest(thermal);
    out.drive_current = out.checks.iter().find(|(c, _)| *c == "drive-current").map_or(f64::NAN, |(_, u)| *u);
    out.following_error_m = runs.iter().map(|(_, load, _)| load.tracking_error_peak).fold(f64::NAN, f64::max);
    out.settling_error_m = runs.iter().map(|(_, load, _)| load.dwell_error_peak).fold(f64::NAN, f64::max);
    out.inertia_ratio = catalog::inertia_ratios(entry, cfg).map_or(f64::NAN, |ratios| {
        highest(ratios.into_iter().enumerate().filter(|(k, _)| ours(k)).map(|r| r.1).collect())
    });
    Ok(out)
}

/// What a search can be ranked by (`rank_by`), lower better: the cost (`Candidate::cost`), the margin (the tightest
/// check's utilisation, `Candidate::tightest`), the energy per cycle, the following error, the thermal utilisation, the
/// inertia ratio (`Figures`), and the drive's size, its rated output current (`catalog::drives_for`; the catalogue
/// has no weight of a drive, so the cost leaves it out).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Criterion {
    Cost,
    Margin,
    Energy,
    Error,
    Thermal,
    Inertia,
    Drive,
    /// The user's price (`Candidate::price`, from `--prices`); infinite for a candidate without one.
    Price,
}

/// The criteria by the names `--rank-by` takes.
pub const CRITERIA: [(&str, Criterion); 8] = [
    ("cost", Criterion::Cost),
    ("margin", Criterion::Margin),
    ("energy", Criterion::Energy),
    ("error", Criterion::Error),
    ("thermal", Criterion::Thermal),
    ("inertia", Criterion::Inertia),
    ("drive", Criterion::Drive),
    ("price", Criterion::Price),
];

impl Criterion {
    /// The candidate's figure for it; infinite where it has none.
    pub fn of(self, c: &Candidate) -> f64 {
        let figure = |f: &dyn Fn(&Figures) -> f64| c.figures.as_ref().map_or(f64::INFINITY, f);
        let value = match self {
            Criterion::Cost => c.cost,
            Criterion::Price => c.price.unwrap_or(f64::INFINITY),
            Criterion::Margin => c.tightest().map_or(f64::INFINITY, |t| t.1),
            Criterion::Energy => figure(&|f| f.energy_j),
            Criterion::Error => figure(&|f| f.following_error_m),
            Criterion::Thermal => figure(&|f| f.thermal),
            Criterion::Inertia => figure(&|f| f.inertia_ratio),
            Criterion::Drive => {
                catalog::drives_for(c.supply).into_iter().find(|d| d.0 == c.drive).map_or(f64::INFINITY, |d| d.1)
            }
        };
        if value.is_nan() { f64::INFINITY } else { value }
    }
}

/// A ranking as `--rank-by` names it: one criterion of `CRITERIA` ("energy"), or a weighted mix ("cost=1,energy=0.5";
/// a name alone weighs 1). `None` for a name it does not know or a weight that is no positive number.
pub fn parse_ranking(text: &str) -> Option<Vec<(Criterion, f64)>> {
    text.split(',')
        .map(|part| {
            let (name, weight) = part.split_once('=').unwrap_or((part, "1"));
            let criterion = CRITERIA.iter().find(|(n, _)| *n == name.trim())?.1;
            let weight: f64 = weight.trim().parse().ok().filter(|w: &f64| *w > 0.0 && w.is_finite())?;
            Some((criterion, weight))
        })
        .collect()
}

/// Ranks a search (`catalog::rank`: passing first, the others nearest to passing first) and orders the passing ones by
/// `by`: the sum of each criterion's weight times its figure over the best (lowest) figure of the passing ones, so
/// that criteria of different units weigh as their weights say (a figure at or below 0 is taken over 1, as it is). For
/// one criterion its figure; ties keep the cost order.
pub fn rank_by(found: &mut [Candidate], by: &[(Criterion, f64)]) {
    catalog::rank(found);
    let passing = found.iter().take_while(|c| c.passes()).count();
    let best: Vec<f64> = by
        .iter()
        .map(|(criterion, _)| {
            let low = found[..passing].iter().map(|c| criterion.of(c)).fold(f64::INFINITY, f64::min);
            if low > 0.0 && low.is_finite() { low } else { 1.0 }
        })
        .collect();
    let score =
        |c: &Candidate| by.iter().zip(&best).map(|((criterion, w), best)| w * criterion.of(c) / best).sum::<f64>();
    found[..passing].sort_by(|a, b| score(a).total_cmp(&score(b)));
}

/// The criteria the search's front is drawn over: cost, the drive's size, margin, energy and following error.
pub const FRONT: [Criterion; 5] =
    [Criterion::Cost, Criterion::Drive, Criterion::Margin, Criterion::Energy, Criterion::Error];

/// Which candidates are on the search's front: passing, and no other passing one is as good in every criterion of
/// `FRONT` and better in one (`analysis::non_dominated_indices`). One of two alike is on it.
pub fn front(found: &[Candidate]) -> Vec<bool> {
    let passing: Vec<usize> = (0..found.len()).filter(|&i| found[i].passes()).collect();
    let figures: Vec<Vec<f64>> = passing.iter().map(|&i| FRONT.iter().map(|c| c.of(&found[i])).collect()).collect();
    let mut out = vec![false; found.len()];
    for k in crate::analysis::non_dominated_indices(&figures) {
        out[passing[k]] = true;
    }
    out
}

/// The gear ratio at which a motor of rotor inertia `rotor` accelerates a pure inertia `load` (at the gearbox output)
/// with the least torque: the torque J_m i a + J_L a / i is least at i = sqrt(J_L / J_m), the classic optimum.
pub fn optimum_ratio(load: f64, rotor: f64) -> f64 {
    (load / rotor).sqrt()
}

/// One value of a sweep of a gear ratio or pulley radius (`transmission_sweep`): the value; the torque the motors need
/// per unit of acceleration of every axis in turn, summed (`catalog::torque_per_acceleration`, the rigid machine at
/// mid travel); the inertia ratio (`Figures::inertia_ratio`); and the cycle run with it, its tightest check and whether
/// it passes; `refused` the reason where it did not run.
#[derive(Debug, Clone, Default)]
pub struct SweepRow {
    pub value: f64,
    pub torque_per_acceleration: f64,
    pub inertia_ratio: f64,
    pub tightest: Option<(&'static str, f64)>,
    pub passes: bool,
    pub cycle_s: f64,
    pub refused: &'static str,
    /// For a catalogue gearbox snapped to (`TransmissionSweep::snapped`): its name; "" for a swept value.
    pub gearbox: String,
}

/// A sweep of one value of the config (`gearbox.ratio` or `geometry.pulley_radius_m`): the rows of the values swept
/// and of the two optima, in the order of their value; the classic optimum, the value at which the motors need the
/// least torque per unit of acceleration (for a pure inertia `optimum_ratio`); the best value for the cycle, the row
/// that passes with the lowest tightest check (else the lowest tightest check); and the catalogue gearboxes around the
/// best ratio, each run on the cycle (with a motor and a series given).
#[derive(Debug, Clone, Default)]
pub struct TransmissionSweep {
    pub rows: Vec<SweepRow>,
    pub classic: f64,
    pub best: f64,
    pub snapped: Vec<SweepRow>,
}

/// The torque per unit acceleration of every axis in turn, summed over axes and motors.
fn torque_need(entry: &MachineEntry, cfg: &Config) -> Result<f64, ConfigError> {
    Ok(catalog::torque_per_acceleration(entry, cfg)?.iter().flatten().map(|t| t.abs()).sum())
}

/// A row of the sweep: `cfg` with its parts in, judged on its cycle.
fn sweep_row(entry: &MachineEntry, cfg: Result<Config, ConfigError>, value: f64) -> SweepRow {
    let mut row = SweepRow {
        value,
        torque_per_acceleration: f64::NAN,
        inertia_ratio: f64::NAN,
        cycle_s: f64::NAN,
        ..SweepRow::default()
    };
    let Ok(cfg) = cfg else {
        row.refused = "config";
        return row;
    };
    row.torque_per_acceleration = torque_need(entry, &cfg).unwrap_or(f64::NAN);
    row.inertia_ratio = catalog::inertia_ratios(entry, &cfg)
        .map_or(f64::NAN, |r| r.into_iter().filter(|x| !x.is_nan()).fold(f64::NAN, f64::max));
    let judged = crate::sim::Sim::new(entry, &cfg).map_err(|_| "config").and_then(|mut sim| {
        let load = crate::analysis::analyse_cycle_with(&mut sim, false).map_err(|e| e.key)?;
        let v = crate::analysis::verdicts(&cfg, &load).map_err(|_| "config")?;
        Ok((load, v))
    });
    match judged {
        Ok(result) => {
            row.cycle_s = result.0.duration;
            let c = Candidate { result: Some(result), ..Candidate::default() };
            (row.tightest, row.passes) = (c.tightest(), c.passes());
        }
        Err(reason) => row.refused = reason,
    }
    row
}

/// `cfg` for a swept `value` of `path`: where that is the pulley radius (`geometry.pulley_radius_m`) the pulley's inertia
/// (`pulley.inertia_kgm2`) goes with it as a disc of the same width and material: J = pi rho w r^4 / 2, so J
/// scales as (r / r0)^4 from the config's pulley (radius r0). Any other path, or a config without a pulley inertia:
/// `cfg` as it is.
fn pulley_scaled(cfg: &Config, path: &str, value: f64) -> Result<Config, ConfigError> {
    let mut out = cfg.clone();
    if let ("geometry.pulley_radius_m", Ok(_)) = (path, cfg.get("pulley.inertia_kgm2")) {
        let inertia = cfg.positive("pulley.inertia_kgm2")? * (value / cfg.positive(path)?).powi(4);
        out.set_text(&format!("pulley.inertia_kgm2 = {inertia:?}"))?;
    }
    Ok(out)
}

/// Sweeps `path` (`gearbox.ratio` or `geometry.pulley_radius_m`) of a config over `count` values spaced logarithmically
/// from `from` to `to`, each with the velocity gain retuned for it (`catalog::with_value`) and run on the cycle; finds the
/// classic optimum by a golden-section search of the torque the motors need per unit acceleration over the same range
/// (unimodal for inertia: J_m i / r + J_L r / i), and runs it too. With `snap` = (series, motor article, supply) and
/// `path` the gear ratio: the catalogue gearboxes of that series that fit the motor around the best ratio
/// (`catalog::fitting_gearboxes`), each run as a row of `snapped`. The runs share the processors.
pub fn transmission_sweep(
    entry: &MachineEntry,
    cfg: &Config,
    path: &str,
    (from, to, count): (f64, f64, usize),
    snap: Option<(&str, &str, &str)>,
) -> Result<TransmissionSweep, ConfigError> {
    let cfg = &catalog::resolve(entry, cfg)?;
    if !(from > 0.0 && to > from && count >= 2) {
        return Err(ConfigError::Invalid(format!("{path} from {from} to {to}, {count} values")));
    }
    cfg.positive(path)?;
    let need = |value: f64| -> Result<f64, ConfigError> {
        let mut changed = pulley_scaled(cfg, path, value)?;
        changed.set_text(&format!("{path} = {value:?}"))?;
        torque_need(entry, &changed)
    };
    // Golden section on the logarithm of the value, to a millionth of the range's ratio.
    let (mut low, mut high) = (from.ln(), to.ln());
    let golden = (5f64.sqrt() - 1.0) / 2.0;
    while high - low > 1e-7 {
        let (a, b) = (high - golden * (high - low), low + golden * (high - low));
        if need(a.exp())? <= need(b.exp())? { high = b } else { low = a }
    }
    let classic = ((low + high) / 2.0).exp();
    let mut values = crate::analysis::log_spaced(from, to, count);
    values.push(classic);
    values.sort_by(f64::total_cmp);
    let rows = crate::analysis::on_all_cores(values.len(), |i| {
        sweep_row(
            entry,
            pulley_scaled(cfg, path, values[i]).and_then(|c| catalog::with_value(entry, &c, path, values[i])),
            values[i],
        )
    });
    let key = |r: &SweepRow| (!r.passes, r.tightest.map_or(f64::INFINITY, |t| t.1));
    let best = rows
        .iter()
        .min_by(|a, b| key(a).0.cmp(&key(b).0).then(key(a).1.total_cmp(&key(b).1)))
        .map_or(classic, |r| r.value);
    let mut snapped = Vec::new();
    if let (Some((series, article, supply)), "gearbox.ratio") = (snap, path) {
        let names = catalog::fitting_gearboxes(article, series, best);
        snapped = crate::analysis::on_all_cores(names.len(), |i| {
            let parts = catalog::Parts { motor: article, supply, gearbox: &names[i], ..catalog::Parts::default() };
            let named = catalog::with_named(entry, cfg, &parts);
            let ratio = named.as_ref().ok().and_then(|c| c.f64("gearbox.ratio").ok()).unwrap_or(f64::NAN);
            SweepRow { gearbox: names[i].clone(), ..sweep_row(entry, named, ratio) }
        });
    }
    Ok(TransmissionSweep { rows, classic, best, snapped })
}

/// A candidate judged at the worst corner of its config's bounds (`robust_ranking`): its place in the ranking given,
/// whether it passes there, its tightest check there, and the cycles the search for the corners took; `refused` where
/// its config could not be judged so.
#[derive(Debug, Clone, Default)]
pub struct RobustRow {
    pub index: usize,
    pub passes: bool,
    pub tightest: Option<(&'static str, f64)>,
    pub runs: usize,
    pub refused: String,
}

/// The first `top` passing candidates of a ranked search on `cfg` judged again at the worst corner of the bounds of
/// their config (`analysis::verdict_worst`; the bounds follow a part put in, see `catalog::set`): the rows in the robust
/// order, the ones that pass there first, each group in the order given; and whether that order differs from the one
/// given. With a duty mix (`duties`, each duty's config and weight; empty: `cfg` alone) every duty is judged at its own
/// worst corner and those verdicts are mixed as the search mixes the duties' own (`mix_verdicts`), with each duty's
/// cycle as the search ran it; the runs add up.
pub fn robust_ranking(
    entry: &MachineEntry,
    cfg: &Config,
    duties: &[(Config, f64)],
    found: &[Candidate],
    top: usize,
) -> (Vec<RobustRow>, bool) {
    let bases: Vec<&Config> = if duties.is_empty() { vec![cfg] } else { duties.iter().map(|d| &d.0).collect() };
    let chosen: Vec<usize> = (0..found.len()).filter(|&i| found[i].passes()).take(top).collect();
    let mut rows = crate::analysis::on_all_cores(chosen.len(), |n| {
        let (index, c) = (chosen[n], &found[chosen[n]]);
        let load = c.result.as_ref().map(|r| r.0.clone()).unwrap_or_else(|| unreachable!("a passing candidate ran"));
        let judged = || -> Result<RobustRow, ConfigError> {
            let (mut runs, mut mixed, mut first) = (0, Vec::new(), None);
            for (i, base) in bases.iter().enumerate() {
                let own = catalog::candidate_config(entry, base, c)?;
                let (worst, done) = crate::analysis::verdict_worst(entry, &own, &crate::analysis::bounds(&own)?)?;
                runs += done;
                let (weight, cycle) = c.duties.get(i).map_or((1.0, &load), |(w, cycle)| (*w, cycle));
                mixed.push((weight, cycle.clone(), worst.into_iter().map(|w| w.verdict).collect::<Vec<_>>()));
                first.get_or_insert(own);
            }
            let verdicts = match (&mixed[..], &first) {
                ([one], _) => one.2.clone(),
                (_, Some(own)) => mix_verdicts(own, &mixed)?,
                _ => Vec::new(),
            };
            let at_worst = Candidate { result: Some((load.clone(), verdicts)), ..c.clone() };
            Ok(RobustRow {
                index,
                passes: at_worst.passes(),
                tightest: at_worst.tightest(),
                runs,
                refused: String::new(),
            })
        };
        judged().unwrap_or_else(|e| RobustRow {
            index,
            refused: format!("{}: {}", e.key(), e.detail()),
            ..RobustRow::default()
        })
    });
    rows.sort_by_key(|r| (!r.passes, r.index));
    let changed = rows.iter().map(|r| r.index).ne(chosen.iter().copied());
    (rows, changed)
}

/// A task in tool space for comparing kinematics (`compare_kinematics`), read from a task file (`Task::parse`): pick and
/// place positions and via points of the way between them, x y z in m relative to each machine's origin (`origin`, by
/// default the middle of its axis ranges), z up; the lift above pick and place; the dwell of the gripper; the payload;
/// the target cycle time (or rate); assignments made to every machine's config (`set`, e.g. `cycle.timing = ...`); and
/// per machine id an origin of its own, in its tool coordinates (one value per axis).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Task {
    pub pick: [f64; 3],
    pub place: [f64; 3],
    pub via: Vec<[f64; 3]>,
    pub lift_m: f64,
    pub dwell_s: f64,
    pub payload_kg: Option<f64>,
    pub cycle_time_s: Option<f64>,
    pub set: Vec<String>,
    pub origin: Vec<(String, Vec<f64>)>,
}

/// Why a task cannot be mapped onto a machine: a key ("task-axis-missing": the task moves along an axis the machine has
/// not; "task-out-of-range": a pose beyond an axis's range; "task-origin": an origin of the wrong length;
/// "task-payload": a payload on a machine whose config has none; "task-set": an assignment the config refuses; or the
/// machine's own reason from `Machine::unreachable`) and what it concerns.
#[derive(Debug, Clone, PartialEq)]
pub struct Refusal {
    pub key: &'static str,
    pub detail: String,
}

impl Task {
    /// A task file (TOML): `pick = [x, y, z]`, `place = [x, y, z]`, optional `via = [[x, y, z], ...]`, `lift_m`,
    /// `dwell_s`, `payload_kg`, `cycle_time_s` or `rate_per_min`, `set = ["path = value", ...]` and a table `[origin]`
    /// of machine id = [one value per axis]. The error names the key that is wrong.
    pub fn parse(text: &str) -> Result<Task, String> {
        let table: toml::Table = text.parse().map_err(|e: toml::de::Error| e.message().to_string())?;
        let number = |v: &toml::Value| v.as_float().or_else(|| v.as_integer().map(|i| i as f64));
        let point = |key: &str, v: &toml::Value| -> Result<[f64; 3], String> {
            let list = v.as_array().filter(|a| a.len() == 3).ok_or(format!("{key}: three numbers x, y, z"))?;
            let mut out = [0.0; 3];
            for (o, v) in out.iter_mut().zip(list) {
                *o = number(v).filter(|x| x.is_finite()).ok_or(format!("{key}: three numbers x, y, z"))?;
            }
            Ok(out)
        };
        let scalar = |key: &str| -> Result<Option<f64>, String> {
            table
                .get(key)
                .map(|v| number(v).filter(|x| x.is_finite() && *x >= 0.0).ok_or(format!("{key}: a number")))
                .transpose()
        };
        let known = [
            "pick",
            "place",
            "via",
            "lift_m",
            "dwell_s",
            "payload_kg",
            "cycle_time_s",
            "rate_per_min",
            "set",
            "origin",
        ];
        if let Some(key) = table.keys().find(|k| !known.contains(&k.as_str())) {
            return Err(format!("{key}: not a key of a task"));
        }
        let cycle_time_s = match (scalar("cycle_time_s")?, scalar("rate_per_min")?) {
            (Some(_), Some(_)) => return Err("cycle_time_s, rate_per_min: one of them".into()),
            (time, rate) => time.or(rate.map(|r| 60.0 / r)),
        };
        Ok(Task {
            pick: point("pick", table.get("pick").ok_or("pick: missing")?)?,
            place: point("place", table.get("place").ok_or("place: missing")?)?,
            via: match table.get("via") {
                None => Vec::new(),
                Some(v) => v
                    .as_array()
                    .ok_or("via: a list of points")?
                    .iter()
                    .map(|p| point("via", p))
                    .collect::<Result<_, _>>()?,
            },
            lift_m: scalar("lift_m")?.unwrap_or(0.0),
            dwell_s: scalar("dwell_s")?.unwrap_or(0.0),
            payload_kg: scalar("payload_kg")?,
            cycle_time_s,
            set: match table.get("set") {
                None => Vec::new(),
                Some(v) => v
                    .as_array()
                    .ok_or("set: a list of assignments")?
                    .iter()
                    .map(|s| s.as_str().map(str::to_string).ok_or("set: a list of assignments"))
                    .collect::<Result<_, _>>()?,
            },
            origin: match table.get("origin") {
                None => Vec::new(),
                Some(v) => v
                    .as_table()
                    .ok_or("origin: a table of machine id = [values]")?
                    .iter()
                    .map(|(id, values)| {
                        let values = values.as_array().ok_or(format!("origin.{id}: a list of numbers"))?;
                        let values = values.iter().map(number).collect::<Option<Vec<f64>>>();
                        Ok((id.clone(), values.ok_or(format!("origin.{id}: a list of numbers"))?))
                    })
                    .collect::<Result<_, String>>()?,
            },
        })
    }
}

/// The config of a machine with a task's cycle in it: the start above the pick, then down to the pick (the gripper
/// closes: `payload` true, the dwell), up, across through the via points to above the place, down (it opens), up, and
/// back across. Each task position goes to the machine's axis of id "x", "y" or "z" in m, from the origin; the other
/// axes stay at the origin. Refused (`Refusal`) where a pose moves along an axis the machine has not, lies beyond an axis's
/// range, or is one the machine cannot go to (`Machine::unreachable`); and the payload set where the config has
/// `mass.payload_kg`. The assignments of `set` come last.
pub fn task_config(entry: &MachineEntry, cfg: &Config, task: &Task) -> Result<Config, Refusal> {
    let refuse = |key: &'static str, detail: String| Refusal { key, detail };
    let config_error = |e: ConfigError| refuse("task-set", format!("{}: {}", e.key(), e.detail()));
    let cfg = catalog::resolve(entry, cfg).map_err(config_error)?;
    let machine = (entry.build)(&cfg).map_err(config_error)?;
    let axes = machine.axes();
    let origin = match task.origin.iter().find(|(id, _)| id == entry.id) {
        Some((_, values)) if values.len() != axes.len() => {
            return Err(refuse(
                "task-origin",
                format!("{}: {} values for {} axes", entry.id, values.len(), axes.len()),
            ));
        }
        Some((_, values)) => values.clone(),
        None => sim_core::testkit::mid_pose(machine.as_ref()),
    };
    let column = |letter: &str| axes.iter().position(|a| a.id == letter && a.unit == "m");
    let columns: Vec<Option<usize>> = ["x", "y", "z"].iter().map(|l| column(l)).collect();
    let pose = |p: [f64; 3]| -> Result<Vec<f64>, Refusal> {
        let mut out = origin.clone();
        for (i, value) in p.iter().enumerate() {
            match columns[i] {
                Some(a) => out[a] += value,
                None if *value != 0.0 => return Err(refuse("task-axis-missing", ["x", "y", "z"][i].into())),
                None => {}
            }
        }
        for (a, (value, axis)) in out.iter().zip(axes).enumerate() {
            if *value < axis.min - 1e-9 || *value > axis.max + 1e-9 {
                return Err(refuse(
                    "task-out-of-range",
                    format!("{} = {value} {} beyond {} .. {} (axis {a})", axis.id, axis.unit, axis.min, axis.max),
                ));
            }
        }
        if let Some(key) = machine.unreachable(&out, 0.0) {
            return Err(refuse(key, format!("{out:?}")));
        }
        Ok(out)
    };
    let up = |p: [f64; 3]| [p[0], p[1], p[2] + task.lift_m];
    let (pick, place) = (pose(task.pick)?, pose(task.place)?);
    let (above_pick, above_place) = (pose(up(task.pick))?, pose(up(task.place))?);
    let via: Vec<Vec<f64>> = task.via.iter().map(|p| pose(*p)).collect::<Result<_, _>>()?;
    let list = |pose: &[f64]| toml::Value::Array(pose.iter().map(|v| toml::Value::Float(*v)).collect());
    let segment = |target: &[f64], dwell: f64, payload: bool, via: &[Vec<f64>]| {
        let mut s = toml::Table::new();
        s.insert("target".into(), list(target));
        s.insert("dwell_s".into(), toml::Value::Float(dwell));
        s.insert("payload".into(), toml::Value::Boolean(payload));
        if !via.is_empty() {
            s.insert("via".into(), toml::Value::Array(via.iter().map(|p| list(p)).collect()));
        }
        toml::Value::Table(s)
    };
    let back: Vec<Vec<f64>> = via.iter().rev().cloned().collect();
    let cycle = toml::Value::Array(vec![
        segment(&pick, task.dwell_s, true, &[]),
        segment(&above_pick, 0.0, true, &[]),
        segment(&above_place, 0.0, true, &via),
        segment(&place, task.dwell_s, false, &[]),
        segment(&above_place, 0.0, false, &[]),
        segment(&above_pick, 0.0, false, &back),
    ]);
    let mut out = cfg.clone();
    out.set("start.pose", list(&above_pick)).map_err(config_error)?;
    out.set("cycle.segment", cycle).map_err(config_error)?;
    match task.payload_kg {
        Some(kg) if out.get("mass.payload_kg").is_ok() => {
            out.set_text(&format!("mass.payload_kg = {kg:?}")).map_err(config_error)?
        }
        Some(kg) if kg > 0.0 => return Err(refuse("task-payload", format!("{kg} kg"))),
        _ => {}
    }
    for assignment in &task.set {
        out.set_text(assignment).map_err(config_error)?;
    }
    Ok(out)
}

/// One machine of a kinematics comparison: its id, why the task cannot be mapped onto it (`task_config`), else the
/// search's best candidate (the first of the ranking, passing or not), how many passed and how many were tried, and the
/// task's target cycle time.
#[derive(Debug, Clone)]
pub struct KinematicsRow {
    pub machine: String,
    pub refused: Option<Refusal>,
    pub best: Option<Candidate>,
    pub passing: usize,
    pub tried: usize,
    pub target_s: Option<f64>,
}

impl KinematicsRow {
    /// Whether the best candidate passes and its cycle (the mean of a mix) meets the target cycle time, where there is one.
    pub fn meets(&self) -> bool {
        let fast =
            |c: &Candidate| self.target_s.is_none_or(|t| c.figures.as_ref().is_some_and(|f| f.cycle_s <= t + 1e-9));
        self.best.as_ref().is_some_and(|c| c.passes() && fast(c))
    }
}

/// Runs a task on each machine (with its config): maps it (`task_config`), runs the catalogue search on it at its
/// supplies (`supplies` gives them per config, e.g. the config's own), ranked by `by` (`rank_by`), and reports per
/// machine the best combination.
pub fn compare_kinematics(
    task: &Task,
    machines: &[(&MachineEntry, Config)],
    supplies: &dyn Fn(&Config) -> Vec<&'static str>,
    wanted: &dyn Fn(&str) -> bool,
    options: &Options,
    by: &[(Criterion, f64)],
) -> Vec<KinematicsRow> {
    machines
        .iter()
        .map(|(entry, cfg)| {
            let mut row = KinematicsRow {
                machine: entry.id.to_string(),
                refused: None,
                best: None,
                passing: 0,
                tried: 0,
                target_s: task.cycle_time_s,
            };
            let mapped = match task_config(entry, cfg, task) {
                Ok(mapped) => mapped,
                Err(refusal) => return KinematicsRow { refused: Some(refusal), ..row },
            };
            match catalog::search(entry, &mapped, &supplies(&mapped), wanted, options) {
                Ok(mut found) => {
                    rank_by(&mut found, by);
                    row.passing = found.iter().filter(|c| c.passes()).count();
                    row.tried = found.len();
                    row.best = found.into_iter().next();
                }
                Err(e) => {
                    row.refused = Some(Refusal { key: "task-set", detail: format!("{}: {}", e.key(), e.detail()) })
                }
            }
            row
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two cycles of one motor: 2 N m RMS for 1 s run three times as often as 4 N m RMS for 2 s. sqrt((3 * 4 * 1 + 1 *
    /// 16 * 2) / (3 * 1 + 1 * 2)) = sqrt(44 / 5) = 2.9665 N m, below the 4 N m of the worse cycle alone: the heat of the
    /// mix, not of its worst cycle. The mean of 10 K and 40 K rises the same way: (3 * 10 + 80) / 5 = 22 K.
    #[test]
    fn a_two_cycle_duty_mix_has_the_time_weighted_rms() {
        let parts = [(3.0, 2.0, 1.0), (1.0, 4.0, 2.0)];
        assert!((mix_rms(&parts) - (44.0f64 / 5.0).sqrt()).abs() < 1e-12, "{}", mix_rms(&parts));
        assert!((mix_mean(&[(3.0, 10.0, 1.0), (1.0, 40.0, 2.0)]) - 22.0).abs() < 1e-12);
    }

    /// A motor of the drive tests' round numbers (rated rise 100 K, no iron loss) with the winding's time constant `tau`
    /// and the drive's image (`MixHeat`).
    fn heat(tau: f64, image: Option<[f64; 3]>) -> MixHeat {
        MixHeat { drive: DriveParams { thermal_time_constant: tau, ..crate::drive::tests::params() }, image }
    }

    /// The mix's verdicts: per check the worst duty's, but the winding rise and the drive's RMS current over the mix.
    /// Duty A (weight 3, 1 s): rise 10 K, 2 A RMS, peak utilisation 0.5; duty B (weight 1, 2 s): rise 40 K, 4 A RMS, peak
    /// 0.9. Far shorter than the time constants the mix heats as its mean: the copper heating at no rise of a duty is
    /// q = T / (1 + c T) (c = 0.0039 /K), the mean q = (3 * 1 * q_A + 1 * 2 * q_B) / 5 settles at q / (1 - c q) = 21.240 K
    /// (the mean of the rises, 22 K, leaves out that the resistance follows the mean's temperature), against 30 K it
    /// passes though B alone (40 K) fails; the current sqrt(44 / 5) = 2.966 A against 3 A passes. Far longer: B's own,
    /// 40 K and 4 A, and both fail. motor-peak is B's 0.9 either way.
    #[test]
    fn a_duty_mix_is_judged_on_the_worst_peak_and_the_heat_of_its_periodic_state() {
        let load = |duration: f64, rise: f64, current: f64| CycleLoad {
            duration,
            motors: vec![MotorLoad { winding_rise: rise, rms_current: current, ..MotorLoad::default() }],
            ..test_load()
        };
        let verdicts = |rise: f64, current: f64, peak: f64| {
            vec![
                Verdict { check: "motor-peak", motor: Some(0), value: peak, limit: 1.0, ok: peak <= 1.0 },
                Verdict { check: "motor-thermal", motor: Some(0), value: rise, limit: 30.0, ok: rise <= 30.0 },
                Verdict { check: "drive-current", motor: Some(0), value: current, limit: 3.0, ok: current <= 3.0 },
            ]
        };
        let runs = vec![
            (3.0, load(1.0, 10.0, 2.0), verdicts(10.0, 2.0, 0.5)),
            (1.0, load(2.0, 40.0, 4.0), verdicts(40.0, 4.0, 0.9)),
        ];
        let c = crate::drive::COPPER;
        let q = |t: f64| t / (1.0 + c * t);
        let mean = (3.0 * q(10.0) + 2.0 * q(40.0)) / 5.0;
        assert!((mean / (1.0 - c * mean) - 21.240).abs() < 1e-3);
        for (tau, rise, current, ok) in
            [(1e9, mean / (1.0 - c * mean), (44.0f64 / 5.0).sqrt(), true), (1e-6, 40.0, 4.0, false)]
        {
            let mixed = mix_verdicts_with(&[heat(tau, Some([tau, 0.0, tau]))], &runs);
            let get = |check: &str| mixed.iter().find(|v| v.check == check).unwrap().clone();
            assert_eq!(get("motor-peak").value, 0.9);
            let (thermal, drive) = (get("motor-thermal"), get("drive-current"));
            assert!((thermal.value - rise).abs() < 1e-6 && thermal.ok == ok, "{tau}: {thermal:?}");
            assert!((drive.value - current).abs() < 1e-6 && drive.ok == ok, "{tau}: {drive:?}");
        }
        // No time constant for the drive's image: the time-weighted RMS.
        let mixed = mix_verdicts_with(&[heat(1e-6, None)], &runs);
        assert!((mixed[2].value - (44.0f64 / 5.0).sqrt()).abs() < 1e-12, "{:?}", mixed[2]);
    }

    /// Two duties of 1 s each, settling at T1 = 10 K and T2 = 30 K, on one node of time constant tau = 1 s: the copper's
    /// resistance follows the temperature, so a duty settling at T approaches it at the rate (1 - c q) / tau =
    /// 1 / (tau (1 + c T)), e_i = exp(-L / (tau (1 + c T_i))). The periodic state ends duty 2 at
    /// x2 = (T2 (1 - e2) + T1 (1 - e1) e2) / (1 - e1 e2) = 24.017 K, its peak; without the copper's rise (c = 0) it would
    /// be (30 + 10 e) / (1 + e) = 24.621 K.
    #[test]
    fn a_duty_mix_judges_the_periodic_peak_of_the_winding_node() {
        let c = crate::drive::COPPER;
        let e = |t: f64| (-1.0 / (1.0 + c * t)).exp();
        let (e1, e2) = (e(10.0), e(30.0));
        let hand = (30.0 * (1.0 - e2) + 10.0 * (1.0 - e1) * e2) / (1.0 - e1 * e2);
        assert!((hand - 24.017).abs() < 1e-3, "{hand}");
        let p = heat(1.0, None).drive;
        let parts = [winding_stretch(&p, 1.0, 10.0, 0.0), winding_stretch(&p, 1.0, 30.0, 0.0)];
        assert!((periodic_peak(&parts, [1.0, 0.0]) - hand).abs() < 1e-12, "{}", periodic_peak(&parts, [1.0, 0.0]));
        // A single duty: its own rise.
        assert!((periodic_peak(&parts[1..], [1.0, 0.0]) - 30.0).abs() < 1e-12);
    }

    /// effects.motor_thermal_nodes (winding share 0.25, 60 s against the housing, 1800 s in all): duties of 300 s at
    /// 2.5 A and 600 s at 1 A, against the two nodes integrated step by step as `Drive::warm` has them (copper loss
    /// I^2 R(T_w), RK4 at 0.1 s over 30 periods, the peak sampled in the last): the closed form agrees within 1e-5 K.
    #[test]
    fn the_periodic_peak_of_the_two_winding_nodes_matches_the_integrated_nodes() {
        let p = DriveParams {
            thermal_nodes: Some(crate::drive::ThermalNodes { winding_share: 0.25, winding_time_constant: 60.0 }),
            ..heat(1800.0, None).drive
        };
        let duties = [(300.0, 2.5), (600.0, 1.0)];
        let parts: Vec<Stretch> =
            duties.iter().map(|&(len, i)| winding_stretch(&p, len, p.settled_rise(i, 0.0), 0.0)).collect();
        let closed = periodic_peak(&parts, [1.0, 0.0]);
        let (s, tw) = (0.25, 60.0);
        let th = 1800.0 - tw / s;
        let f = |[w, h]: [f64; 2], i: f64| {
            let heating = p.iron_heating(i * i * p.resistance_at(w));
            [(s * heating - (w - h)) / tw, ((w - h) / s - h / (1.0 - s)) / th]
        };
        let (dt, mut x, mut peak) = (0.1, [0.0, 0.0], 0.0f64);
        for period in 0..30 {
            for &(len, i) in &duties {
                for _ in 0..(len / dt) as usize {
                    let add = |x: [f64; 2], k: [f64; 2], h: f64| [x[0] + h * k[0], x[1] + h * k[1]];
                    let k1 = f(x, i);
                    let k2 = f(add(x, k1, dt / 2.0), i);
                    let k3 = f(add(x, k2, dt / 2.0), i);
                    let k4 = f(add(x, k3, dt), i);
                    x = [0, 1].map(|j| x[j] + dt / 6.0 * (k1[j] + 2.0 * k2[j] + 2.0 * k3[j] + k4[j]));
                    if period == 29 {
                        peak = peak.max(x[0]);
                    }
                }
            }
        }
        assert!((closed - peak).abs() < 1e-5, "closed {closed}, integrated {peak}");
    }

    /// The drive's image with chips (share 0.3, 2 s) on a heat sink of 100 s: duty A 2 A for 10 s, B 4 A for 5 s. Each
    /// node is first order and rises in B, falls in A, so the image peaks at B's end, the sum of the nodes there: with
    /// Q = share I^2 and e = exp(-L / tau) per node, x = (Q_B (1 - e_B) + Q_A (1 - e_A) e_B) / (1 - e_A e_B); the verdict
    /// is the root of the sum, 3.2012 A.
    #[test]
    fn a_duty_mix_judges_the_drives_image_with_its_chips() {
        let node = |share: f64, tau: f64| {
            let (ea, eb) = ((-10.0 / tau).exp(), (-5.0 / tau).exp());
            (share * 16.0 * (1.0 - eb) + share * 4.0 * (1.0 - ea) * eb) / (1.0 - ea * eb)
        };
        let hand = (node(0.7, 100.0) + node(0.3, 2.0)).sqrt();
        assert!((hand - 3.2012).abs() < 1e-4, "{hand}");
        let duty = |duration: f64, current: f64| {
            let v = Verdict { check: "drive-current", motor: Some(0), value: current, limit: 5.0, ok: true };
            (1.0, CycleLoad { duration, ..test_load() }, vec![v])
        };
        let mixed = mix_verdicts_with(&[heat(1800.0, Some([100.0, 0.3, 2.0]))], &[duty(10.0, 2.0), duty(5.0, 4.0)]);
        assert!((mixed[0].value - hand).abs() < 1e-9, "{:?}", mixed[0]);
    }

    /// The S1 check over a mix: duty A (weight 3, 1 s) 2 N m RMS against a limit of 4.5 N m at its mean speed, duty B
    /// (weight 1, 2 s) 4 N m against 3.5 N m. B alone fails. Far shorter than the winding's time constant the mix's
    /// utilisation is sqrt((3 * (2 / 4.5)^2 + 2 * (4 / 3.5)^2) / 5) = 0.80060 of the time-weighted mean limit
    /// (3 * 4.5 * 1 + 1 * 3.5 * 2) / 5 = 4.1 N m, and passes; far longer, B's 4 / 3.5, and fails.
    #[test]
    fn a_duty_mix_is_judged_on_s1_by_its_utilisation_in_the_winding_node() {
        let load = |duration: f64| CycleLoad { duration, ..test_load() };
        let verdict = |value: f64, limit: f64| {
            vec![Verdict { check: "motor-s1", motor: Some(0), value, limit, ok: value <= limit }]
        };
        let runs = vec![(3.0, load(1.0), verdict(2.0, 4.5)), (1.0, load(2.0), verdict(4.0, 3.5))];
        let u: f64 = ((3.0 * (2.0f64 / 4.5).powi(2) + 2.0 * (4.0f64 / 3.5).powi(2)) / 5.0).sqrt();
        assert!((u - 0.80060).abs() < 1e-5);
        let s1 = &mix_verdicts_with(&[heat(1e9, None)], &runs)[0];
        assert!((s1.value - u * 4.1).abs() < 1e-6 && (s1.limit - 4.1).abs() < 1e-12 && s1.ok, "{s1:?}");
        let s1 = &mix_verdicts_with(&[heat(1e-6, None)], &runs)[0];
        assert!((s1.value / s1.limit - 4.0 / 3.5).abs() < 1e-9 && !s1.ok, "{s1:?}");
    }

    /// A swept pulley radius takes the pulley's inertia with it as r^4: twice the radius, 16 times the inertia; a config
    /// without a pulley inertia, and any other path, stay as they are.
    #[test]
    fn a_swept_pulley_radius_scales_the_pulley_inertia_with_r4() {
        let cfg =
            Config::parse("format_version = 1\n[geometry]\npulley_radius_m = 0.01\n[pulley]\ninertia_kgm2 = 2.0e-5\n")
                .unwrap();
        let scaled = pulley_scaled(&cfg, "geometry.pulley_radius_m", 0.02).unwrap();
        assert!((scaled.f64("pulley.inertia_kgm2").unwrap() - 3.2e-4).abs() < 1e-15);
        assert_eq!(pulley_scaled(&cfg, "gearbox.ratio", 0.02).unwrap().f64("pulley.inertia_kgm2").unwrap(), 2.0e-5);
        let bare = Config::parse("format_version = 1\n[geometry]\npulley_radius_m = 0.01\n").unwrap();
        assert!(pulley_scaled(&bare, "geometry.pulley_radius_m", 0.02).unwrap().get("pulley.inertia_kgm2").is_err());
    }

    /// The classic optimum: J_L = 0.0025 kg m^2 at the output, J_m = 0.3e-4 kg m^2: sqrt(83.33) = 9.129.
    #[test]
    fn the_optimum_ratio_of_a_pure_inertia_is_the_root_of_the_inertia_ratio() {
        assert!((optimum_ratio(0.0025, 0.3e-4) - 9.128709).abs() < 1e-6);
    }

    /// `--rank-by` takes one name, or names with weights; anything else is refused.
    #[test]
    fn a_ranking_is_one_criterion_or_a_weighted_mix() {
        assert_eq!(parse_ranking("energy"), Some(vec![(Criterion::Energy, 1.0)]));
        assert_eq!(parse_ranking("cost=1, error=0.5"), Some(vec![(Criterion::Cost, 1.0), (Criterion::Error, 0.5)]));
        assert_eq!(parse_ranking("price"), Some(vec![(Criterion::Price, 1.0)]));
        assert_eq!(parse_ranking("weight"), None);
        assert_eq!(parse_ranking("cost=-1"), None);
    }

    /// Ranking by a weighted mix: three passing candidates, cost 10/20/30 kg and energy 300/100/150 J. By cost: 10, 20,
    /// 30. By energy: 100, 150, 300. By cost=1,energy=1, each over the best (10 kg, 100 J): 1 + 3 = 4, 2 + 1 = 3,
    /// 3 + 1.5 = 4.5, so 20, 10, 30. The front over cost, margin, energy and error: 10 (cheapest) and 20 (least energy);
    /// 30 is beaten by 20 in both.
    #[test]
    fn a_search_ranks_by_any_criterion_and_marks_its_front() {
        let pass = vec![Verdict { check: "motor-peak", motor: Some(0), value: 0.5, limit: 1.0, ok: true }];
        let candidate = |cost: f64, energy: f64| Candidate {
            cost,
            result: Some((test_load(), pass.clone())),
            figures: Some(Figures { energy_j: energy, following_error_m: 1e-4, ..Figures::default() }),
            ..Candidate::default()
        };
        let mut found = vec![candidate(30.0, 150.0), candidate(10.0, 300.0), candidate(20.0, 100.0)];
        let costs = |found: &[Candidate]| found.iter().map(|c| c.cost).collect::<Vec<_>>();
        rank_by(&mut found, &[(Criterion::Cost, 1.0)]);
        assert_eq!(costs(&found), [10.0, 20.0, 30.0]);
        rank_by(&mut found, &[(Criterion::Energy, 1.0)]);
        assert_eq!(costs(&found), [20.0, 30.0, 10.0]);
        rank_by(&mut found, &parse_ranking("cost=1,energy=1").unwrap());
        assert_eq!(costs(&found), [20.0, 10.0, 30.0]);
        assert_eq!(front(&found), [true, true, false]);
    }

    fn test_load() -> CycleLoad {
        CycleLoad { duration: 1.0, motors: vec![MotorLoad::default()], ..CycleLoad::default() }
    }
}
