//! Fitting a config to a measured machine, in stages: first the rigid values of each axis from a drive
//! trace (or, where the motors do not turn in proportion to the axes, of each motor: `fit_joints`), then the
//! load's resonance on the belt from a frequency sweep, or its stiffness from a move's trace
//! (`fit_move_stiffness`). Nothing here edits a config; the results say what the config would have to be.
//!
//! Every stage runs the same estimate on the measurement and on the config's own simulation of the same
//! run, and reports the two side by side. What the simple estimate gets wrong (gearbox losses that grow
//! with the torque, friction that depends on the load, a resonance that moves with the sweep's amplitude)
//! it gets wrong on both sides, so the difference between them is what the config is off by.

use crate::analysis::SweepPoint;
use crate::trace::{Trace, TraceError, replay};
use sim_core::{Config, ConfigError, Machine, MachineEntry};

/// Rows on either side of a row that are averaged with it: 11 rows in all, 11 ms in a trace from a run.
/// Long enough to average the belt's ringing and the noise, short against a ramp of the profile.
const WINDOW: usize = 5;

/// The rigid values of one axis, in this order: moved mass (with rotors and pulleys counted as mass at the
/// axis), viscous friction, Coulomb friction and constant force. In N, kg, s and the axis's unit.
#[derive(Debug, Clone, PartialEq)]
pub struct AxisFit {
    /// What the estimate gives on the config's simulation of the trace's setpoints.
    pub config: [f64; 4],
    /// What it gives on the trace, and the standard error of that.
    pub fitted: [f64; 4],
    pub error: [f64; 4],
    /// Where the payload was picked or dropped during the trace (`fit_rigid`), rising: the time and its standard error
    /// in s, then the change of the payload's factor there (+1 for the config's payload picked, −1 for it dropped) and
    /// its standard error. With steps, `fitted` holds the values before the first.
    pub payload_steps: Vec<(f64, f64, f64, f64)>,
}

impl AxisFit {
    /// What the fit says to change in the config, as translation keys with their argument: the axis named
    /// `axis` with the change of mass and constant force, and the factors on the two kinds of friction; then each
    /// payload step.
    pub fn advice(&self, axis: &str) -> Vec<(&'static str, String)> {
        let change = |q: usize, unit: &str| {
            format!("{axis}: {:+.3} {unit} ± {:.3}", self.fitted[q] - self.config[q], self.error[q])
        };
        let factor = |q: usize| {
            format!("{axis}: × {:.3} ± {:.3}", self.fitted[q] / self.config[q], (self.error[q] / self.config[q]).abs())
        };
        [
            ("fit-mass", change(0, "kg")),
            ("fit-viscous", factor(1)),
            ("fit-coulomb", factor(2)),
            ("fit-constant", change(3, "N")),
        ]
        .into_iter()
        .chain(self.payload_steps.iter().map(|(time, time_error, change, error)| {
            ("fit-axis-payload-step", format!("{axis}: {time:.3} s ± {time_error:.3}, {change:+.3} ± {error:.3}"))
        }))
        .collect()
    }
}

/// Per axis: the force the motors put on it and its speed, both from the trace's motor values through
/// the machine's transformation. Motor angle per axis travel turns motor torques into axis forces
/// (the same power on both sides), so the fit works per axis even where every motor moves every axis.
fn axis_view(machine: &dyn Machine, trace: &Trace) -> Vec<(Vec<f64>, Vec<f64>)> {
    let (axes, motors, h) = (machine.axes().len(), trace.motors.len(), 1e-3);
    let mut out = vec![(Vec::new(), Vec::new()); axes];
    let (mut pose, mut ahead) = (vec![0.0; axes], vec![0.0; axes]);
    let (mut base, mut shifted) = (vec![0.0; motors], vec![0.0; motors]);
    for row in 0..trace.time.len() {
        let angle: Vec<f64> = trace.motors.iter().map(|m| m.angle[row]).collect();
        let moved: Vec<f64> = trace.motors.iter().map(|m| m.angle[row] + h * m.speed[row]).collect();
        machine.forward(&angle, &mut pose);
        machine.forward(&moved, &mut ahead);
        machine.inverse(&pose, &mut base);
        for (i, (force, speed)) in out.iter_mut().enumerate() {
            speed.push((ahead[i] - pose[i]) / h);
            pose[i] += h;
            machine.inverse(&pose, &mut shifted);
            pose[i] -= h;
            force.push((0..motors).map(|k| trace.motors[k].torque[row] * (shifted[k] - base[k]) / h).sum());
        }
    }
    out
}

/// Least squares of force = mass * acceleration + viscous * speed + coulomb * sign(speed) + constant + Σ the `extra`
/// columns (one value per row each, by a value of their own) on the rows where the axis moves: values and standard
/// errors, the four first. Force, speed and the extra columns are averaged over the window, the acceleration is the
/// slope of the speed across it. Rows slower than 5 % of the top speed are left out, because there friction is neither
/// of the two terms. `None` if the axis does not move both ways, since one direction alone cannot tell Coulomb friction
/// from a constant force. The columns are scaled to unit length before the normal matrix is solved, as `solve` does:
/// accelerations and ones differ by orders of magnitude.
fn rigid_values(time: &[f64], force: &[f64], speed: &[f64], extra: &[Vec<f64>]) -> Option<(Vec<f64>, Vec<f64>)> {
    let (w, rows, p) = (WINDOW, time.len(), 4 + extra.len());
    let top = speed.iter().fold(0.0, |a: f64, b| a.max(b.abs()));
    let mean = |x: &[f64], i: usize| x[i - w..=i + w].iter().sum::<f64>() / (2 * w + 1) as f64;
    let mut samples: Vec<Sample> = Vec::new();
    for i in w..rows.saturating_sub(w) {
        let v = mean(speed, i);
        if v.abs() > 0.05 * top {
            let mut x = vec![(speed[i + w] - speed[i - w]) / (time[i + w] - time[i - w]), v, v.signum(), 1.0];
            x.extend(extra.iter().map(|column| mean(column, i)));
            samples.push((i, x, mean(force, i)));
        }
    }
    let one_way = |sign: f64| samples.iter().filter(|(_, x, _)| x[2] == sign).count();
    if one_way(1.0).min(one_way(-1.0)) < 10 * (2 * w + 1) {
        return None;
    }
    // A column that is zero on every row used (the payload's force on an axis it does not load) has no value.
    let basis: Vec<usize> = (0..p).filter(|&j| samples.iter().any(|(_, x, _)| x[j] != 0.0)).collect();
    let solved = solve(&samples, Some(&basis), (2 * w + 1) as f64)?;
    Some((solved.values, solved.errors))
}

/// Stage 1: the rigid values of every axis from a trace, next to the config's. `None` for an axis the
/// trace does not move in both directions.
/// The machine has to pass `trace::per_axis_fit` (a linear transformation and an inertia that does not change with the
/// pose); others are refused with `error-trace-not-linear` and go to `fit_joints`.
///
/// A payload picked or dropped during the trace is looked for as `fit_joints` does (`payload_steps`, in the motors'
/// coordinates, against the config's replay); at each step found an axis gets a column of its own, the payload's force
/// on it (the rigid model's with less without the payload, through the transformation) from the step on, whose value
/// is the change of the payload's factor there (`AxisFit::payload_steps`). The axis's values are then those before the
/// first step, the config's those of its replay, which holds the payload all along. Links (`Machine::links`): no
/// machine the fit per axis takes reports any, the per-axis values hold them.
pub fn fit_rigid(entry: &MachineEntry, cfg: &Config, trace: &Trace) -> Result<Vec<Option<AxisFit>>, TraceError> {
    let mut machine = (entry.build)(&crate::catalog::resolve(entry, cfg)?)?;
    if !crate::trace::per_axis_fit(machine.as_ref()) {
        return Err(TraceError { key: "error-trace-not-linear", detail: String::new() });
    }
    let simulated = replay(entry, cfg, trace)?;
    let (h, per) = smoothing(&trace.time);
    let steps = match (regressor(machine.as_mut(), trace, h), regressor(machine.as_mut(), &simulated, h)) {
        (Some(measured), Some(config)) => payload_steps(&measured, &config, &trace.time, h, per),
        _ => Vec::new(),
    };
    // The payload's torque at each motor, row by row, as a trace whose axis view is the payload's force on each axis.
    let mut payload = trace.clone();
    payload.motors.iter_mut().for_each(|m| m.torque.fill(0.0));
    if !steps.is_empty() {
        let n = trace.motors.len();
        let (mut with, mut without) = (vec![0.0; n], vec![0.0; n]);
        for i in 1..trace.time.len() - 1 {
            let t = &trace.time;
            let at = |k: usize, a: usize| trace.motors[k].angle[a];
            let angle: Vec<f64> = (0..n).map(|k| at(k, i)).collect();
            let speed: Vec<f64> = trace.motors.iter().map(|m| m.speed[i]).collect();
            let acc: Vec<f64> = (0..n)
                .map(|k| {
                    let slope = |a: usize| (at(k, a + 1) - at(k, a)) / (t[a + 1] - t[a]);
                    2.0 * (slope(i) - slope(i - 1)) / (t[i + 1] - t[i - 1])
                })
                .collect();
            machine.set_payload_attached(false);
            machine.rigid_motor_torque(&angle, &speed, &acc, &mut without);
            machine.set_payload_attached(true);
            machine.rigid_motor_torque(&angle, &speed, &acc, &mut with);
            for k in 0..n {
                payload.motors[k].torque[i] = with[k] - without[k];
            }
        }
    }
    let carried = axis_view(machine.as_ref(), &payload);
    let views = axis_view(machine.as_ref(), trace).into_iter().zip(axis_view(machine.as_ref(), &simulated));
    Ok(views
        .zip(&carried)
        .map(|((measured, config), (force, _))| {
            // From each step on, the payload's force on the axis.
            let columns: Vec<Vec<f64>> = steps
                .iter()
                .map(|&(step, _)| (0..trace.time.len()).map(|i| if i >= step { force[i] } else { 0.0 }).collect())
                .collect();
            let (values, errors) = rigid_values(&trace.time, &measured.0, &measured.1, &columns)?;
            let four = |v: &[f64]| -> [f64; 4] { std::array::from_fn(|q| v[q]) };
            Some(AxisFit {
                config: four(&rigid_values(&trace.time, &config.0, &config.1, &[])?.0),
                fitted: four(&values),
                error: four(&errors),
                payload_steps: steps
                    .iter()
                    .enumerate()
                    .map(|(j, &(row, time_error))| (trace.time[row], time_error, values[4 + j], errors[4 + j]))
                    .collect(),
            })
        })
        .collect())
}

/// One value of the fit in the motors' coordinates (`fit_joints`): its name as the CLI writes it, of what (a motor's
/// id, a link's name, or `payload`), and as in `AxisFit` the config's estimate, the measured one and its standard error.
/// Per motor in N m, kg m², s and rad at the motor shaft. A base parameter: where the trace cannot tell other values
/// from this one, `combines` names them (as `quantity of`) with the factor their change enters this one's change by, so
/// that fitted less config is the change of this value plus each factor times the change of that one.
#[derive(Debug, Clone, PartialEq)]
pub struct JointValue {
    pub quantity: &'static str,
    pub of: String,
    pub config: f64,
    pub fitted: f64,
    pub error: f64,
    pub combines: Vec<(String, f64)>,
}

impl JointValue {
    /// `quantity of`, as `combines` and the advice name a value.
    pub fn name(&self) -> String {
        format!("{} {}", self.quantity, self.of)
    }
}

/// Stage 1 in the motors' coordinates (`fit_joints`).
#[derive(Debug, Clone, PartialEq)]
pub struct JointFit {
    /// The base parameters: every value the trace tells apart from the others, with what it `combines`.
    pub values: Vec<JointValue>,
    /// Pairs of `values` (their indices) whose measured estimates correlate by more than `ALIKE`: the trace cannot
    /// tell them apart, only a sum (or difference) of them is known, and each one's error band is wide for that.
    pub alike: Vec<(usize, usize)>,
    /// Values (`quantity of`) that are not reported on their own: those a base parameter combines, and those the
    /// rigid torques do not depend on at all (a link's first moment along its own joint's axis).
    pub folded: Vec<String>,
    /// Where the payload was picked or dropped during the trace, rising: each time and its standard error, in s. The
    /// payload's factor is then fitted per stretch between them: before the first (`payload_factor_before`) and from each
    /// on (`payload_factor_after`, of `payload` for one step, `payload 1`, `payload 2`, ... for several).
    pub payload_steps: Vec<(f64, f64)>,
}

/// Above this correlation of two estimates the trace does not tell them apart.
pub const ALIKE: f64 = 0.95;

/// No value's variance may be inflated by more than this by the others' (1 / (1 − R²) of its column on all the others):
/// less than a tenth of its column outside what the others explain (and no less than 1, which rounding gives where the
/// columns are exactly dependent). A column that would make one so is not fitted on
/// its own but folded into those before it (`solve`): exactly where the rigid model only knows their sum (a link
/// turning about a fixed axis adds to its motor's inertia and nothing else), nearly where the trace does not move the
/// joints that would tell them apart (an elbow held still, a link never tilted against gravity).
pub const WEAK: f64 = 100.0;

/// The payload's factor has to differ before and after a step by this many of its standard errors for the step to
/// count (`JointFit::payload_steps`), more than the trace's after less before differs from the config's replay's. Well
/// above 3, as the search tries some hundred splits and keeps the best; on the swings with the payload held (delta,
/// SCARA, arm) none was found, on the delta's pick the difference was some 70 standard errors.
pub const PAYLOAD_STEP_SIGMAS: f64 = 6.0;

/// The quantities of `fit_joints` per motor, after the factor on the payload (`payload_factor`).
pub const JOINT_QUANTITIES: [&str; 4] =
    ["added_inertia_kg_m2", "viscous_friction_nm_s_rad", "coulomb_friction_nm", "constant_torque_nm"];

/// The quantities of `fit_joints` per link of a chain (`Machine::links`), after the motors': the factor on its mass,
/// first moment and inertia together, and the change of its first moment along x, y and z of its frame, its mass and
/// its inertia about its frame kept.
pub const LINK_QUANTITIES: [&str; 4] =
    ["link_mass_factor", "link_first_moment_x_kg_m", "link_first_moment_y_kg_m", "link_first_moment_z_kg_m"];

/// The payload's factor before the first step and from each step on (`JointFit::payload_steps`), in place of
/// `payload_factor`.
pub const PAYLOAD_STEP_QUANTITIES: [&str; 2] = ["payload_factor_before", "payload_factor_after"];

impl JointFit {
    /// What the fit says to change in the config, as translation keys with their argument: the factor on the payload
    /// (or its step), per motor the changes of inertia, friction and constant torque at its shaft, per link its mass
    /// factor and first moment; then what each base parameter combines, what has no effect and the pairs that cannot
    /// be told apart.
    pub fn advice(&self) -> Vec<(&'static str, String)> {
        let factor = |v: &JointValue| format!("× {:.3} ± {:.3}", v.fitted / v.config, (v.error / v.config).abs());
        let mut out: Vec<(&'static str, String)> = Vec::new();
        // The stretches' factors lead the values, in order of time.
        let stretches: Vec<&JointValue> =
            self.values.iter().filter(|v| PAYLOAD_STEP_QUANTITIES.contains(&v.quantity)).collect();
        for (j, (time, error)) in self.payload_steps.iter().enumerate() {
            let at = |s: usize| stretches.get(s).map(|v| factor(v)).unwrap_or_default();
            out.push(("fit-payload-step", format!("{time:.3} s ± {error:.3}: {} → {}", at(j), at(j + 1))));
        }
        out.extend(self.values.iter().filter_map(|v| {
            let change = |unit: &str| format!("{}: {:+.4} {unit} ± {:.4}", v.of, v.fitted - v.config, v.error);
            Some(match v.quantity {
                "payload_factor" => ("fit-payload", factor(v)),
                "payload_factor_before" | "payload_factor_after" => return None,
                "added_inertia_kg_m2" => {
                    ("fit-joint-inertia", format!("{}: {:+.3e} kg m² ± {:.1e}", v.of, v.fitted - v.config, v.error))
                }
                "viscous_friction_nm_s_rad" => {
                    ("fit-joint-viscous", format!("{}: {:+.2e} N m s/rad ± {:.1e}", v.of, v.fitted - v.config, v.error))
                }
                "coulomb_friction_nm" => ("fit-joint-coulomb", change("N m")),
                "constant_torque_nm" => ("fit-joint-constant", change("N m")),
                "link_mass_factor" => ("fit-link-mass", format!("{}: {}", v.of, factor(v))),
                axis => ("fit-link-moment", format!("{} {}", &axis[18..19], change("kg m"))),
            })
        }));
        for v in self.values.iter().filter(|v| !v.combines.is_empty()) {
            let parts: Vec<String> = v.combines.iter().map(|(name, c)| format!("{c:+.3e} × {name}")).collect();
            out.push(("fit-combined", format!("{}: {}", v.name(), parts.join(", "))));
        }
        let combined: Vec<&String> = self.values.iter().flat_map(|v| v.combines.iter().map(|(n, _)| n)).collect();
        let no_effect: Vec<&str> = self.folded.iter().filter(|f| !combined.contains(f)).map(String::as_str).collect();
        if !no_effect.is_empty() {
            out.push(("fit-no-effect", no_effect.join(", ")));
        }
        out.extend(
            self.alike
                .iter()
                .map(|&(a, b)| ("fit-alike", format!("{} / {}", self.values[a].name(), self.values[b].name()))),
        );
        out
    }
}

/// The first moment's change the links' columns are taken over (kg m): the torques are linear in it, so the size only
/// sets the rounding.
const MOMENT_STEP: f64 = 0.01;

/// One row of the stacked regressor: the trace's row, the regressor Y and the torque the rigid model leaves over.
type Sample = (usize, Vec<f64>, f64);

/// The span over which the regressor's rows are smoothed, s: a triangle of this half-width (`smoothing`). A gear's ringing
/// above 1 / 0.04 s = 25 Hz is all but gone from it (the triangle's response is sinc², below 0.05 beyond its first zero),
/// the slow swings of an exciting trajectory pass (0.995 at 1 Hz), and as the torque and every column are smoothed alike
/// the fit holds whatever passes.
// shortcut: one span for every machine, set by the six-axis arm's gears (their ringing at some 30 Hz); take it from
// `Machine::modes` once every machine reports its lowest mode (the chain machines report none).
pub const SMOOTHING_S: f64 = 0.04;

/// The triangle's half-width `h` in rows of a trace's step (the median), and how many rows of a series smoothed by it
/// count as one measurement: (Σ w)² / Σ w² for the weights w = h − |j|, |j| < h (h⁴ / (h² + 2 Σ (h − j)²), about 1.5 h;
/// for a moving average of 2w + 1 rows the same formula gives 2w + 1).
fn smoothing(time: &[f64]) -> (usize, f64) {
    let mut steps: Vec<f64> = time.windows(2).map(|p| p[1] - p[0]).collect();
    steps.sort_by(f64::total_cmp);
    let h = steps.get(steps.len() / 2).map_or(1, |step| ((SMOOTHING_S / step).round() as usize).max(1));
    let squares = (h * h) as f64 + 2.0 * (1..h).map(|j| ((h - j) * (h - j)) as f64).sum::<f64>();
    (h, (h * h) as f64 * (h * h) as f64 / squares)
}

/// The stacked regressor of the motor torques on the machine's rigid model, τ − τ_rigid = Y θ, in the motors' own
/// coordinates: per motor k a row
///
/// [τ_k,rigid − τ_k,rigid without payload, then per motor q̈_k, q̇_k, sign q̇_k, 1 at motor k's own place, then per
/// link (`Machine::links`) the change of τ_k,rigid with the link's inertial parameters doubled and with 0.01 kg m more
/// first moment along x, y and z, per unit],
///
/// the rigid torques from `Machine::rigid_split_torque` at each row's load motion with the rotors at their own
/// acceleration, linear in each of these. The load: each motor's angle less the twist of its train that its torque
/// holds (`Machine::twist`, zeros where the trains are rigid), a two-mass model per joint; its speed the measured one
/// less the twist's slope, its acceleration the second difference of the load's angle (an encoder's angle is far less
/// noisy than the speed differentiated, and noise on a regressor biases its value towards zero). Then the torque and
/// every column are smoothed alike along the rows by a triangle of half-width `h` (`smoothing`): the equation holds as
/// well smoothed, and what the rigid model does not have, the gears' ringing and the noise, is smoothed away. Before 14
/// November the rigid model was taken at the motors' angles, the acceleration was the second difference across ±5 rows
/// and the torque their mean: on the six-axis arm with its gears elastic a link's mass factor came out −11.8 for
/// +0.57 (smoothing alone: −9.4; with the twist: + 0.558). A motor's row only where its smoothed speed is above 5 % of its top speed and its speed keeps its sign across
/// the window (sign q̇ smoothed over a reversal is no longer ±1). A link's column that changes the stacked rigid torque
/// by less than 1e-9 of it is set to zeros: rounding, of a parameter that has no effect. `None` where a motor does not
/// turn both ways.
fn regressor(machine: &mut dyn Machine, trace: &Trace, h: usize) -> Option<Vec<Sample>> {
    let (rows, n, links) = (trace.time.len(), trace.motors.len(), machine.links().len());
    let p = 1 + 4 * n + 4 * links;
    if rows < 2 * h + 5 {
        return None;
    }
    let t = &trace.time;
    let top: Vec<f64> = trace.motors.iter().map(|m| m.speed.iter().fold(0.0, |a: f64, b| a.max(b.abs()))).collect();
    // Per motor, row by row as measured: the columns, the torque the rigid model leaves over and the rigid torque.
    let mut raw = vec![vec![0.0; rows * p]; n];
    let (mut left, mut stacked) = (vec![vec![0.0; rows]; n], vec![vec![0.0; rows]; n]);
    let (mut with, mut without, mut changed) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
    let steps = [
        (2.0, [0.0; 3]),
        (1.0, [MOMENT_STEP, 0.0, 0.0]),
        (1.0, [0.0, MOMENT_STEP, 0.0]),
        (1.0, [0.0, 0.0, MOMENT_STEP]),
    ];
    // The second difference of a series per motor at row a (rows 1 to rows − 2).
    let second = |x: &dyn Fn(usize) -> f64, a: usize| {
        2.0 * ((x(a + 1) - x(a)) / (t[a + 1] - t[a]) - (x(a) - x(a - 1)) / (t[a] - t[a - 1])) / (t[a + 1] - t[a - 1])
    };
    // Each motor's acceleration and the twist of its train (`Machine::twist`), row by row: the load is where the motor's
    // angle less the twist puts it, a two-mass model per joint whose spring's force the motor's torque tells.
    let rotor_acc: Vec<Vec<f64>> = (0..rows)
        .map(|i| {
            (0..n)
                .map(|k| if i == 0 || i == rows - 1 { 0.0 } else { second(&|a| trace.motors[k].angle[a], i) })
                .collect()
        })
        .collect();
    let twist: Vec<Vec<f64>> = (0..rows)
        .map(|i| {
            let mut out = vec![0.0; n];
            machine.twist(&trace.motors.iter().map(|m| m.torque[i]).collect::<Vec<f64>>(), &rotor_acc[i], &mut out);
            out
        })
        .collect();
    let load = |k: usize, a: usize| trace.motors[k].angle[a] - twist[a][k];
    for i in 2..rows - 2 {
        let angle: Vec<f64> = (0..n).map(|k| load(k, i)).collect();
        let speed: Vec<f64> = (0..n)
            .map(|k| trace.motors[k].speed[i] - (twist[i + 1][k] - twist[i - 1][k]) / (t[i + 1] - t[i - 1]))
            .collect();
        let acc: Vec<f64> = (0..n).map(|k| second(&|a| load(k, a), i)).collect();
        machine.set_payload_attached(false);
        machine.rigid_split_torque(&angle, &speed, &acc, &rotor_acc[i], &mut without);
        machine.set_payload_attached(true);
        machine.rigid_split_torque(&angle, &speed, &acc, &rotor_acc[i], &mut with);
        for k in 0..n {
            let x = &mut raw[k][i * p..(i + 1) * p];
            x[0] = with[k] - without[k];
            x[1 + 4 * k..5 + 4 * k].copy_from_slice(&[acc[k], speed[k], speed[k].signum(), 1.0]);
            (left[k][i], stacked[k][i]) = (trace.motors[k].torque[i] - with[k], with[k]);
        }
        for l in 0..links {
            for (c, (factor, moment)) in steps.iter().enumerate() {
                machine.set_link_change(l, *factor, *moment);
                machine.rigid_split_torque(&angle, &speed, &acc, &rotor_acc[i], &mut changed);
                let size = if c == 0 { 1.0 } else { MOMENT_STEP };
                for k in 0..n {
                    raw[k][i * p + 1 + 4 * n + 4 * l + c] = (changed[k] - with[k]) / size;
                }
            }
            machine.set_link_change(l, 1.0, [0.0; 3]);
        }
    }
    // The triangle (h − |j|) / h² about row i, |j| < h: its weights sum to 1.
    let smooth = |at: &dyn Fn(usize) -> f64, i: usize| {
        (1..h).map(|j| (h - j) as f64 * (at(i - j) + at(i + j))).sum::<f64>() / (h * h) as f64 + at(i) / h as f64
    };
    let mut samples: Vec<Sample> = Vec::new();
    let mut ways = vec![[0usize; 2]; n];
    // The stacked rigid torque's and each link column's change's sum of squares, for the columns of no effect.
    let (mut rigid, mut link_change) = (0.0, vec![0.0; 4 * links]);
    for i in h + 1..rows - h - 1 {
        for k in 0..n {
            let speed = &trace.motors[k].speed;
            let v = smooth(&|r| speed[r], i);
            if v.abs() <= 0.05 * top[k] || speed[i + 1 - h..i + h].iter().any(|s| s.signum() != v.signum()) {
                continue;
            }
            let x: Vec<f64> = (0..p).map(|c| smooth(&|r| raw[k][r * p + c], i)).collect();
            for (c, change) in link_change.iter_mut().enumerate() {
                let size = if c % 4 == 0 { 1.0 } else { MOMENT_STEP };
                *change += (x[1 + 4 * n + c] * size).powi(2);
            }
            rigid += smooth(&|r| stacked[k][r], i).powi(2);
            ways[k][usize::from(v > 0.0)] += 1;
            samples.push((i, x, smooth(&|r| left[k][r], i)));
        }
    }
    if ways.iter().any(|both| both[0].min(both[1]) < 10 * (2 * h + 1)) {
        return None;
    }
    for (c, change) in link_change.iter().enumerate() {
        if change.sqrt() <= 1e-9 * rigid.sqrt() {
            samples.iter_mut().for_each(|(_, x, _)| x[1 + 4 * n + c] = 0.0);
        }
    }
    Some(samples)
}

/// The regressor with the payload's column split at the trace's rows `steps`: one column for each stretch between them.
fn split(samples: &[Sample], steps: &[usize]) -> Vec<Sample> {
    samples
        .iter()
        .map(|(row, x, y)| {
            // One column per stretch between the steps (rising rows); a row in stretch s has the payload's in column s.
            let mut out = vec![0.0; steps.len() + 1];
            out[steps.partition_point(|s| s <= row)] = x[0];
            out.extend_from_slice(&x[1..]);
            (*row, out, *y)
        })
        .collect()
}

/// A least-squares fit of the stacked regressor on its base parameters.
struct Solved {
    /// Per column: the value (NaN outside the basis), its standard error, and for a column of the basis the columns it
    /// combines with their factors.
    values: Vec<f64>,
    errors: Vec<f64>,
    combines: Vec<Vec<(usize, f64)>>,
    covariance: Vec<Vec<f64>>,
    basis: Vec<usize>,
}

/// Least squares on the base parameters: the columns, in their order, that the trace tells apart from those already
/// taken (each column taken where with it no value's variance inflation, the diagonal of the inverse normal matrix of
/// the columns scaled to unit length, is above `WEAK`), or the `basis` given (the trace's, for the config's replay). The payload's and the motors' columns come first, so a link's
/// parameter that the motors' own values already hold is folded into those, and the links' parameters are reported
/// only for what the motors' cannot hold. Each dropped column j is X_j = Σ X_i B_ij + r_j over the basis, B the least
/// squares projection, r_j orthogonal to every column of the basis (zero where the model only knows the sum); so its
/// value enters each base value i with the factor B_ij (`combines`) and its r_j only the residual, not the base values.
/// Standard errors as `rigid_values`, the variance widened by `per` rows to a measurement (`smoothing`). `None` where
/// the normal matrix cannot be solved.
fn solve(samples: &[Sample], basis: Option<&[usize]>, per: f64) -> Option<Solved> {
    let p = samples.first()?.1.len();
    let norm: Vec<f64> = (0..p).map(|j| samples.iter().map(|(_, x, _)| x[j] * x[j]).sum::<f64>().sqrt()).collect();
    let live: Vec<usize> = (0..p).filter(|&j| norm[j] > 0.0).collect();
    let (mut gram, mut right) = (vec![vec![0.0; p]; p], vec![0.0; p]);
    for (_, x, y) in samples {
        for &a in &live {
            let za = x[a] / norm[a];
            right[a] += za * y;
            for &b in &live {
                gram[a][b] += za * x[b] / norm[b];
            }
        }
    }
    let basis: Vec<usize> = match basis {
        Some(b) => b.to_vec(),
        None => live.iter().fold(Vec::new(), |chosen: Vec<usize>, &j| {
            let mut trial = chosen.clone();
            trial.push(j);
            let told_apart = inverse_of(&gram, &trial)
                .is_some_and(|inv| (0..trial.len()).all(|a| (1.0 - 1e-6..=WEAK).contains(&inv[a][a])));
            if told_apart { trial } else { chosen }
        }),
    };
    let m = basis.len();
    let inverse = inverse_of(&gram, &basis)?;
    let mut values = vec![f64::NAN; p];
    for (a, &j) in basis.iter().enumerate() {
        values[j] = (0..m).map(|b| inverse[a][b] * right[basis[b]]).sum::<f64>() / norm[j];
    }
    let mut combines = vec![Vec::new(); p];
    for &j in live.iter().filter(|j| !basis.contains(j)) {
        for (a, &i) in basis.iter().enumerate() {
            let scaled: f64 = (0..m).map(|b| inverse[a][b] * gram[basis[b]][j]).sum();
            // Shares below 1 % of the folded column are left out of what is reported: rounding, or next to nothing.
            if scaled.abs() > 0.01 {
                combines[i].push((j, scaled * norm[j] / norm[i]));
            }
        }
    }
    let residual: f64 =
        samples.iter().map(|(_, x, y)| (y - basis.iter().map(|&j| values[j] * x[j]).sum::<f64>()).powi(2)).sum();
    // Neighbouring rows share most of their smoothing, so only every `per`th row counts as a measurement.
    let variance = residual / (samples.len() - m) as f64 * per;
    let mut covariance = vec![vec![f64::NAN; p]; p];
    for (a, &i) in basis.iter().enumerate() {
        for (b, &j) in basis.iter().enumerate() {
            covariance[i][j] = variance * inverse[a][b] / (norm[i] * norm[j]);
        }
    }
    let errors = (0..p).map(|j| covariance[j][j].sqrt()).collect();
    Some(Solved { values, errors, combines, covariance, basis })
}

/// The inverse of the normal matrix `gram` on the columns `of`; `None` where it has none.
fn inverse_of(gram: &[Vec<f64>], of: &[usize]) -> Option<Vec<Vec<f64>>> {
    let m = of.len();
    let normal: Vec<f64> = of.iter().flat_map(|&a| of.iter().map(move |&b| gram[a][b])).collect();
    let unit = |a: usize| (0..m).map(|b| if a == b { 1.0 } else { 0.0 }).collect::<Vec<f64>>();
    (0..m).map(|a| crate::chain::solve(&normal, &unit(a))).collect()
}

/// Where in the stretch `within` (rows, the ends kept out) the payload's column `column` of the regressor is best split
/// in two: the column split at every row in turn, on the base parameters `basis` of the fit with the steps found so far
/// (the column one of them), and the row whose split leaves the least sum of squares; every `coarse`th row, then every
/// row about the best (`coarse` = 2h + 1 of `smoothing`). Its standard error is how far from it the sum of squares stays
/// within one variance (as `solve` takes it, `per` rows to a measurement) of its least. In: the regressor split at the
/// steps so far, its basis, the column, the stretch, the trace's times; out: the row and the time's error.
///
/// A sum of squares per split from running sums along the rows: only the split column changes with the split, and it
/// is zero outside its stretch.
#[allow(clippy::too_many_arguments)]
fn best_split(
    samples: &[Sample],
    basis: &[usize],
    column: usize,
    within: std::ops::Range<usize>,
    time: &[f64],
    coarse: usize,
    per: f64,
) -> Option<(usize, f64)> {
    let at = basis.iter().position(|&j| j == column)?;
    let (m, rows) = (basis.len(), time.len());
    let norm: Vec<f64> =
        basis.iter().map(|&j| samples.iter().map(|(_, x, _)| x[j] * x[j]).sum::<f64>().sqrt()).collect();
    let (mut gram, mut right, mut yy) = (vec![0.0; m * m], vec![0.0; m], 0.0);
    // Per row of the trace: the split column's products with every column and with the torque, summed up to it.
    let mut running = vec![vec![0.0; m + 1]; rows + 1];
    let mut by_row = vec![vec![0.0; m + 1]; rows];
    for (row, x, y) in samples {
        let z: Vec<f64> = basis.iter().zip(&norm).map(|(&j, n)| x[j] / n).collect();
        for a in 0..m {
            right[a] += z[a] * y;
            for b in 0..m {
                gram[a * m + b] += z[a] * z[b];
            }
            by_row[*row][a] += z[at] * z[a];
        }
        by_row[*row][m] += z[at] * y;
        yy += y * y;
    }
    for row in 0..rows {
        running[row + 1] = (0..=m).map(|c| running[row][c] + by_row[row][c]).collect();
    }
    let others: Vec<usize> = (0..m).filter(|&a| a != at).collect();
    // The sum of squares left with the split before `row`: the column before, after, then the basis's others.
    let left = |row: usize| -> Option<f64> {
        let s = &running[row];
        let k = m + 1;
        let (mut normal, mut rhs) = (vec![0.0; k * k], vec![0.0; k]);
        normal[0] = s[at];
        normal[k + 1] = gram[at * m + at] - s[at];
        rhs[0] = s[m];
        rhs[1] = right[at] - s[m];
        for (o, &a) in others.iter().enumerate() {
            let r = o + 2;
            normal[r] = s[a];
            normal[r * k] = s[a];
            normal[k + r] = gram[at * m + a] - s[a];
            normal[r * k + 1] = gram[at * m + a] - s[a];
            rhs[r] = right[a];
            for (o2, &b) in others.iter().enumerate() {
                normal[r * k + o2 + 2] = gram[a * m + b];
            }
        }
        let theta = crate::chain::solve(&normal, &rhs)?;
        Some(yy - theta.iter().zip(&rhs).map(|(t, r)| t * r).sum::<f64>())
    };
    let (from, to) = (within.start, within.end);
    let best_of = |rows: &mut dyn Iterator<Item = usize>| {
        rows.filter_map(|r| left(r).map(|s| (r, s))).min_by(|a, b| a.1.total_cmp(&b.1))
    };
    let (rough, _) = best_of(&mut (from..to).step_by(coarse))?;
    let near = rough.saturating_sub(coarse).max(from)..(rough + coarse).min(to);
    let (row, least) = best_of(&mut near.clone())?;
    let variance = least / (samples.len() - m - 1) as f64 * per;
    let spread = near
        .filter(|&r| left(r).is_some_and(|s| s - least <= variance))
        .map(|r| (time[r] - time[row]).abs())
        .fold(0.0, f64::max);
    Some((row, spread.max(time[row] - time[row - 1])))
}

/// At most this many payload steps are looked for in one trace.
pub const MAX_PAYLOAD_STEPS: usize = 8;

/// Where the payload was picked or dropped during the trace, as rows with the time's standard error, rising. One step
/// at a time: in each stretch between the steps found so far (a tenth of the trace kept from its ends and from every
/// step) the best split (`best_split`); the regressor split there is fitted on the trace and, on that basis, on the
/// config's replay (which holds the payload all along), and the split counts where the factor's change across it on the
/// trace less that on the replay is more than `PAYLOAD_STEP_SIGMAS` of its standard error. Of the stretches' splits that
/// count, the one with the most standard errors is taken, then the search goes on, up to `MAX_PAYLOAD_STEPS`.
fn payload_steps(measured: &[Sample], config: &[Sample], time: &[f64], h: usize, per: f64) -> Vec<(usize, f64)> {
    let (rows, margin) = (time.len(), time.len() / 10);
    let mut steps: Vec<(usize, f64)> = Vec::new();
    // The stretches' columns all fitted on their own, ahead of every other.
    let leading = |fit: &Solved, stretches: usize| {
        fit.basis.len() >= stretches && (0..stretches).eq(fit.basis[..stretches].iter().copied())
    };
    while steps.len() < MAX_PAYLOAD_STEPS {
        let at: Vec<usize> = steps.iter().map(|s| s.0).collect();
        let current = split(measured, &at);
        let Some(fit) = solve(&current, None, per).filter(|f| leading(f, at.len() + 1)) else { break };
        let ends: Vec<usize> = std::iter::once(0).chain(at.iter().copied()).chain(std::iter::once(rows)).collect();
        let mut best: Option<(f64, usize, f64)> = None;
        for (stretch, pair) in ends.windows(2).enumerate() {
            let within = pair[0] + margin..pair[1].saturating_sub(margin);
            if within.is_empty() {
                continue;
            }
            let Some((row, error)) = best_split(&current, &fit.basis, stretch, within, time, 2 * h + 1, per) else {
                continue;
            };
            let mut trial = at.clone();
            trial.insert(stretch, row);
            let Some(split_fit) = solve(&split(measured, &trial), None, per).filter(|f| leading(f, trial.len() + 1))
            else {
                continue;
            };
            let Some(own) = solve(&split(config, &trial), Some(&split_fit.basis), per) else { continue };
            let (a, b, c) = (stretch, stretch + 1, &split_fit.covariance);
            let sigma = (c[a][a] + c[b][b] - 2.0 * c[a][b]).sqrt();
            let change = (split_fit.values[b] - split_fit.values[a]) - (own.values[b] - own.values[a]);
            let sigmas = change.abs() / sigma;
            if sigmas > PAYLOAD_STEP_SIGMAS && best.is_none_or(|(most, ..)| sigmas > most) {
                best = Some((sigmas, row, error));
            }
        }
        let Some((_, row, error)) = best else { break };
        steps.push((row, error));
        steps.sort_by_key(|s| s.0);
    }
    steps
}

/// Stage 1 in the motors' own coordinates, for every machine, and the only one for a machine whose motors do not turn
/// in proportion to its axes (the delta's arms, a SCARA, an arm): the motor torques regressed on the machine's own
/// rigid model (`regressor`), on the trace and on the config's replay of it, on the trace's base parameters (`solve`).
/// For the config the payload's factor and each link's come out near 1 and the added inertias and first moments near
/// 0, friction and constant torque as the config has them at the motor shaft, gearbox losses and all, so that the
/// difference is what the config is off by. `None` where a motor does not turn both ways in the trace.
///
/// Each motor's added inertia stands for what its rotor and its link add about the joint alone; a link's mass factor
/// and first moments are reported where they act otherwise (on other joints, against gravity, with the pose), else
/// folded into the motors' values (`JointValue::combines`). What else it cannot tell apart depends on the moves and is
/// folded or listed in `alike`; a trace that swings every axis over much of its range at several frequencies
/// (docs/measurement-procedure.md) leaves none of the motors'. Elastic gears (`effects.joint_elasticity`): their
/// ringing is smoothed out of the torque and of the rigid model alike (`regressor`, `SMOOTHING_S`); the slow twist
/// under the load is left out (on the six-axis arm 1.5 mrad of a joint's angle under 100 N m), a shortcut that holds
/// while the swings stay well below the gears' resonance.
///
/// Payloads picked or dropped during the trace (no gripper column in a trace, so from its torques: `payload_steps`)
/// are fitted per stretch between the steps: `payload_factor_before` up to the first step, `payload_factor_after` from
/// each step on (of `payload` for one step, `payload 1`, `payload 2`, ... for several). The config's replay keeps its
/// payload and is fitted with the same split; a step counts where the trace's change of the factor across it differs
/// from the replay's by more than `PAYLOAD_STEP_SIGMAS` standard errors, so that what the rigid model misses of the
/// machine over the trace, the same in both, is not taken for one.
///
/// A factor on the rest of the rigid model (arms, platform, rotors) is not fitted: on the delta it correlated by more
/// than 0.95 with the motors' added inertias and constant torques together (gravity is nearly constant over a trace,
/// and each arm's inertia sits on its own motor), so it made all of them unknown.
pub fn fit_joints(entry: &MachineEntry, cfg: &Config, trace: &Trace) -> Result<Option<JointFit>, TraceError> {
    let simulated = replay(entry, cfg, trace)?;
    let mut machine = (entry.build)(&crate::catalog::resolve(entry, cfg)?)?;
    let ids: Vec<String> = machine.motors().iter().map(|m| m.id.to_string()).collect();
    let links = machine.links();
    let (h, per) = smoothing(&trace.time);
    let (Some(measured), Some(config)) =
        (regressor(machine.as_mut(), trace, h), regressor(machine.as_mut(), &simulated, h))
    else {
        return Ok(None);
    };
    let steps = payload_steps(&measured, &config, &trace.time, h, per);
    let at: Vec<usize> = steps.iter().map(|s| s.0).collect();
    let Some(fitted) = solve(&split(&measured, &at), None, per) else { return Ok(None) };
    let Some(replayed) = solve(&split(&config, &at), Some(&fitted.basis), per) else { return Ok(None) };
    let payload: Vec<(&'static str, String)> = match steps.len() {
        0 => vec![("payload_factor", "payload".to_string())],
        1 => PAYLOAD_STEP_QUANTITIES.iter().map(|q| (*q, "payload".to_string())).collect(),
        many => std::iter::once((PAYLOAD_STEP_QUANTITIES[0], "payload".to_string()))
            .chain((1..=many).map(|j| (PAYLOAD_STEP_QUANTITIES[1], format!("payload {j}"))))
            .collect(),
    };
    let factors = payload.len();
    let names: Vec<(&'static str, String)> = payload
        .into_iter()
        .chain(ids.iter().flat_map(|id| JOINT_QUANTITIES.map(|q| (q, id.clone()))))
        .chain(links.iter().flat_map(|l| LINK_QUANTITIES.map(|q| (q, l.clone()))))
        .collect();
    // Factors are fitted as a change, as the rest; the config's own counts 1.
    let shift = |j: usize| if j < factors || names[j].0 == "link_mass_factor" { 1.0 } else { 0.0 };
    let label = |j: usize| format!("{} {}", names[j].0, names[j].1);
    let values: Vec<JointValue> = fitted
        .basis
        .iter()
        .map(|&j| JointValue {
            quantity: names[j].0,
            of: names[j].1.clone(),
            config: replayed.values[j] + shift(j),
            fitted: fitted.values[j] + shift(j),
            error: fitted.errors[j],
            combines: fitted.combines[j].iter().map(|&(i, c)| (label(i), c)).collect(),
        })
        .collect();
    let mut alike = Vec::new();
    let c = &fitted.covariance;
    for (a, &i) in fitted.basis.iter().enumerate() {
        for (b, &j) in fitted.basis.iter().enumerate().skip(a + 1) {
            if (c[i][j] / (c[i][i] * c[j][j]).sqrt()).abs() > ALIKE {
                alike.push((a, b));
            }
        }
    }
    let folded = (0..names.len()).filter(|j| !fitted.basis.contains(j)).map(label).collect();
    let payload_steps = steps.iter().map(|&(row, error)| (trace.time[row], error)).collect();
    Ok(Some(JointFit { values, alike, folded, payload_steps }))
}

// The load swinging on the belt against the motor, read from a frequency sweep.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Resonance {
    pub frequency: f64,
    /// Damping ratio from the width of the dip. Friction and the gearbox widen it too, so this is not
    /// the belt's damping alone.
    pub damping: f64,
    /// Half the distance between the sweep's points at the dip: the frequency is not known better.
    pub frequency_error: f64,
}

/// Stage 2: the resonance of the load on the belt, from a frequency sweep of motor torque to motor speed
/// (`frequency_sweep`, or a drive's measurement of the mechanical system). Where the load resonates the
/// motor hardly moves, which shows as a dip. Its frequency squared is stiffness over load mass, so with
/// the masses right, measured over config frequency, squared, is the factor the stiffness is off by.
///
/// The dip is not read off a move's trace: the closed loops and friction damp the ringing, and the belt's stiffness
/// changes along the way; `fit_move_stiffness` replays such a trace instead. A sweep that does not have the dip and
/// both its flanks gives `None`.
///
/// A machine with more than one spring shows more than one dip (the first default machine has three,
/// near 22, 62 and 107 Hz); this is the lowest in frequency, the load swinging on the whole drive train, so that the config's
/// sweep and the measured one are compared at the same dip. The points may come in any order.
pub fn resonance(sweep: &[SweepPoint]) -> Option<Resonance> {
    let mut sweep = sweep.to_vec();
    sweep.sort_by(|a, b| a.frequency.total_cmp(&b.frequency));
    // Gain times frequency is level for a pure inertia, so the dip is not hidden in the 1 / f slope.
    let level: Vec<f64> = sweep.iter().map(|p| p.gain * p.frequency).collect();
    // The flanks, where the response has twice the power of the lowest point, are steep: noise moves
    // them little, where it can move the lowest point by a whole step.
    let flanks = |lowest: usize| {
        let limit = level[lowest] * std::f64::consts::SQRT_2;
        let below = (0..lowest).rev().find(|&i| level[i] > limit)?;
        let above = (lowest + 1..level.len()).find(|&i| level[i] > limit)?;
        // Not a dip of its own where a deeper point lies between its flanks.
        level[below..=above].iter().all(|&l| l >= level[lowest]).then_some((limit, below, above))
    };
    let (limit, below, above) = (0..level.len()).find_map(flanks)?;
    let crossing = |outside: usize, inside: usize| {
        let share = (limit - level[inside]) / (level[outside] - level[inside]);
        sweep[inside].frequency + share * (sweep[outside].frequency - sweep[inside].frequency)
    };
    let (low, high) = (crossing(below, below + 1), crossing(above, above - 1));
    let frequency = (low * high).sqrt();
    let step =
        (sweep[below + 1].frequency - sweep[below].frequency).max(sweep[above].frequency - sweep[above - 1].frequency);
    Some(Resonance { frequency, damping: (high - low) / (2.0 * frequency), frequency_error: 0.5 * step })
}

/// A frequency response as `sim-cli sweep` writes it: the columns `frequency_hz` and `gain_rad_s_per_nm`. A
/// phase is not read; the fit does not use it.
pub fn parse_sweep(text: &str) -> Result<Vec<SweepPoint>, TraceError> {
    let table = crate::trace::columns(text, &["frequency_hz".to_string(), "gain_rad_s_per_nm".to_string()])?;
    Ok(table[0]
        .iter()
        .zip(&table[1])
        .map(|(&frequency, &gain)| SweepPoint { frequency, gain, phase_deg: 0.0 })
        .collect())
}

/// Stage 2 done: the resonance of the config's own sweep and that of the measured one.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SweepFit {
    pub config: Resonance,
    pub fitted: Resonance,
}

impl SweepFit {
    /// The factor the belt stiffness is off by, measured over config frequency squared, and its standard error.
    pub fn stiffness(&self) -> (f64, f64) {
        let factor = (self.fitted.frequency / self.config.frequency).powi(2);
        (factor, 2.0 * factor * self.fitted.frequency_error / self.fitted.frequency)
    }

    /// What to change in the config, as a translation key with its argument.
    pub fn advice(&self) -> (&'static str, String) {
        let (factor, error) = self.stiffness();
        ("fit-stiffness", format!("× {factor:.3} ± {error:.3}"))
    }
}

/// Stage 2 on a measured sweep, taken on `motor` with a torque `amplitude`: the config's own sweep at the same
/// frequencies, and the resonance of both. `None` where either shows no dip with both flanks.
pub fn fit_sweep(
    entry: &MachineEntry,
    cfg: &Config,
    motor: usize,
    measured: &[SweepPoint],
    amplitude: f64,
) -> Result<Option<SweepFit>, ConfigError> {
    let frequencies: Vec<f64> = measured.iter().map(|p| p.frequency).collect();
    let config = crate::analysis::frequency_sweep(entry, cfg, motor, &frequencies, amplitude)?;
    Ok(resonance(&config).zip(resonance(measured)).map(|(config, fitted)| SweepFit { config, fitted }))
}

/// Above this standard error, over the factor, a trace is taken not to excite the mode enough (`fit_move_stiffness`).
pub const MOVE_STIFFNESS_LIMIT: f64 = 0.1;

/// The config key `fit_move_stiffness` scales unless told another: the belts' stiffness, of every belt machine.
pub const BELT_STIFFNESS_KEY: &str = "belt.stiffness_ea_n";

/// Stage 2 from an ordinary move's trace (`fit_move_stiffness`): the factor on the stiffness and its standard error,
/// and the ringing the fit compares, RMS over the motors in N m: in the trace, and what the config's replay with the
/// factor leaves of it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MoveStiffness {
    pub factor: f64,
    pub error: f64,
    pub ringing: f64,
    pub residual: f64,
}

impl MoveStiffness {
    /// What to change in the config, as a translation key with its argument.
    pub fn advice(&self) -> (&'static str, String) {
        ("fit-move-stiffness", format!("× {:.3} ± {:.3}", self.factor, self.error))
    }
}

/// What is left of each motor torque after its mean over 10 ms on either side is taken off: the ringing of the load on
/// its springs (12 ms a period at 84 Hz), without the rigid motion (ramps of tens of ms and more), which a wrong mass or
/// friction would put in. Rows within the window of either end are left out.
fn ringing(trace: &Trace) -> Vec<f64> {
    let rows = trace.time.len();
    let step = (trace.time[rows - 1] - trace.time[0]) / (rows - 1) as f64;
    let w = ((0.01 / step).round() as usize).max(1);
    let mut out = Vec::new();
    for m in &trace.motors {
        out.extend(
            (w..rows.saturating_sub(w))
                .map(|i| m.torque[i] - m.torque[i - w..=i + w].iter().sum::<f64>() / (2 * w + 1) as f64),
        );
    }
    out
}

/// Stage 2 from an ordinary move's trace, no sweep: an output-error fit of the stiffness. The config is replayed with
/// the value at `key` (the belts' stiffness, `BELT_STIFFNESS_KEY`, or another spring between motor and load) times a
/// factor, and the factor is the one whose replay's motor torques ring as the trace's do (`ringing`: least squares over
/// every motor and row). The replay has the config's loops, filters, delays and friction, which damp the ringing of a
/// move in the trace as they do in the machine, so that they need not be fitted as well; a free two-mass fit of the
/// torque took the drive's own lag, at 51 Hz on the belt axis, for the belt. The masses have to be right first (stage
/// 1), as for a sweep.
///
/// The factor is searched from 1/4 to 4: nine replays a factor of 1.41 apart, then a parabola through seven replays
/// between the neighbours of the best, and another through seven within ±10 % of its vertex (24 replays in all). Its
/// standard error from the curvature 2 a of the last parabola: var(ln factor) = 2 σ² / (2 a), σ² = S / (N − 1) widened
/// by (1 + ρ) / (1 − ρ) for the lag-one correlation ρ of the residuals (the ringing of neighbouring rows is not
/// independent), plus the variance of the vertex from the points' scatter about the parabola (on a trace without noise
/// that is all there is: 0.70 on the belt axis came out 0.681 ± 0.001 without it).
///
/// Refused with `fit-move-not-excited` where that error is above `MOVE_STIFFNESS_LIMIT` of the factor or the best
/// factor lies at an end of the range: the trace's moves did not ring the load enough against its noise (a gentle
/// move, a low jerk, a heavily damped mode). With `fit-move-no-key` where the config has no number at `key`.
pub fn fit_move_stiffness(
    entry: &MachineEntry,
    cfg: &Config,
    trace: &Trace,
    key: &str,
) -> Result<MoveStiffness, TraceError> {
    let base = cfg.f64(key).map_err(|_| TraceError { key: "fit-move-no-key", detail: key.to_string() })?;
    let measured = ringing(trace);
    let cost = |u: f64| -> Result<(f64, Vec<f64>), TraceError> {
        let mut scaled = cfg.clone();
        scaled.set_text(&format!("{key} = {:?}", base * u.exp()))?;
        let residuals: Vec<f64> =
            ringing(&replay(entry, &scaled, trace)?).iter().zip(&measured).map(|(s, m)| m - s).collect();
        Ok((residuals.iter().map(|r| r * r).sum(), residuals))
    };
    let not_excited = |detail: String| TraceError { key: "fit-move-not-excited", detail };
    let grid: Vec<f64> = (0..9).map(|i| (i as f64 - 4.0) * 0.5 * std::f64::consts::LN_2).collect();
    let costs = grid.iter().map(|&u| cost(u).map(|c| c.0)).collect::<Result<Vec<f64>, _>>()?;
    let best = (0..grid.len()).min_by(|&a, &b| costs[a].total_cmp(&costs[b])).unwrap_or(0);
    if best == 0 || best + 1 == grid.len() {
        return Err(not_excited(format!("× {:.2}", grid[best].exp())));
    }
    // Parabolas through evenly spaced replays, not golden sections: the sum of squares is jagged on a small scale (a belt
    // tooth or the gearbox's play meets a little earlier or later), which a parabola through several points averages
    // over. First between the grid's neighbours of its best point, then ±0.1 about that vertex.
    // Vertex, curvature (twice the square term) and the vertex's standard error from how far the points scatter about
    // the parabola: σ_p² (AᵀA)⁻¹ for the linear term over twice the square term, squared.
    let parabola = |from: f64, to: f64| -> Result<Option<(f64, f64, f64)>, TraceError> {
        let mid = 0.5 * (from + to);
        let points = (0..7)
            .map(|i| from + (to - from) * i as f64 / 6.0)
            .map(|u| cost(u).map(|c| (u - mid, c.0)))
            .collect::<Result<Vec<_>, _>>()?;
        let (mut a, mut b) = (vec![0.0; 9], vec![0.0; 3]);
        for (x, c) in &points {
            let row = [1.0, *x, x * x];
            for r in 0..3 {
                b[r] += row[r] * c;
                (0..3).for_each(|k| a[3 * r + k] += row[r] * row[k]);
            }
        }
        let Some(v) = crate::chain::solve(&a, &b).filter(|v| v[2] > 0.0) else { return Ok(None) };
        let scatter = points.iter().map(|(x, c)| (c - v[0] - v[1] * x - v[2] * x * x).powi(2)).sum::<f64>()
            / (points.len() - 3) as f64;
        let linear = crate::chain::solve(&a, &[0.0, 1.0, 0.0]).map_or(f64::INFINITY, |column| column[1]);
        Ok(Some(((mid - 0.5 * v[1] / v[2]).clamp(from, to), 2.0 * v[2], (scatter * linear).sqrt() / (2.0 * v[2]))))
    };
    let curved =
        |found: Option<(f64, f64, f64)>| found.ok_or_else(|| not_excited(format!("× {:.2} ± inf", grid[best].exp())));
    let (first, ..) = curved(parabola(grid[best - 1], grid[best + 1])?)?;
    let (u, curvature, jagged) = curved(parabola(first - 0.1, first + 0.1)?)?;
    let (s, residuals) = cost(u)?;
    let rho = (residuals.windows(2).map(|w| w[0] * w[1]).sum::<f64>() / s.max(f64::MIN_POSITIVE)).clamp(0.0, 0.98);
    let variance = s / (residuals.len() - 1) as f64 * (1.0 + rho) / (1.0 - rho);
    let factor = u.exp();
    let error = factor * (2.0 * variance / curvature + jagged * jagged).sqrt();
    let rms = |x: &[f64]| (x.iter().map(|v| v * v).sum::<f64>() / x.len().max(1) as f64).sqrt();
    if !(error <= MOVE_STIFFNESS_LIMIT * factor) {
        return Err(not_excited(format!("× {factor:.2} ± {error:.2}")));
    }
    Ok(MoveStiffness { factor, error, ringing: rms(&measured), residual: rms(&residuals) })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The dip of a response with a known resonance: gain * f = |fa^2 - f^2 + 2 j d fa f| has its lowest
    /// point at fa and twice that power where f^2 = fa^2 (1 -+ 2 d).
    #[test]
    fn resonance_reads_frequency_and_damping_off_a_known_dip() {
        let (fa, d) = (80.0f64, 0.05);
        let sweep: Vec<SweepPoint> = crate::analysis::log_spaced(50.0, 130.0, 60)
            .into_iter()
            .map(|f| SweepPoint { frequency: f, gain: (fa * fa - f * f).hypot(2.0 * d * fa * f) / f, phase_deg: 0.0 })
            .collect();
        let found = resonance(&sweep).unwrap();
        assert!((found.frequency - fa).abs() < found.frequency_error && found.frequency_error < 0.01 * fa, "{found:?}");
        assert!((found.damping - d).abs() < 0.05 * d, "{found:?}");
        // Half a dip is none.
        assert_eq!(resonance(&sweep[..25]), None);
        // From high to low frequency, the same dip. The damping came out negative.
        let falling: Vec<SweepPoint> = sweep.iter().rev().copied().collect();
        assert_eq!(resonance(&falling), Some(found));
    }

    /// Of two dips the lowest in frequency is reported, also where the one above it is deeper: the first default machine's
    /// dips at 22 and 62 Hz are about as deep, and which was the deepest changed with the amplitude and the
    /// stiffness, so a fit could compare the config's one dip with the other one measured.
    #[test]
    fn of_two_dips_the_lowest_in_frequency_is_reported() {
        let dip = |f: f64, fa: f64, d: f64| (fa * fa - f * f).hypot(2.0 * d * fa * f) / (fa * fa);
        let sweep: Vec<SweepPoint> = crate::analysis::log_spaced(10.0, 200.0, 200)
            .into_iter()
            .map(|f| SweepPoint {
                frequency: f,
                gain: 1000.0 * dip(f, 22.0, 0.05).min(1.0) * dip(f, 62.0, 0.01).min(1.0) / f,
                phase_deg: 0.0,
            })
            .collect();
        // The dip at 62 Hz goes down to 0.02, the one at 22 Hz to 0.1.
        let found = resonance(&sweep).unwrap();
        assert!((found.frequency - 22.0).abs() < 0.5 && (found.damping - 0.05).abs() < 0.01, "{found:?}");
    }
}
