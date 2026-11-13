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
}

impl AxisFit {
    /// What the fit says to change in the config, as translation keys with their argument: the axis named
    /// `axis` with the change of mass and constant force, and the factors on the two kinds of friction.
    pub fn advice(&self, axis: &str) -> [(&'static str, String); 4] {
        let change = |q: usize, unit: &str| format!("{axis}: {:+.3} {unit} ± {:.3}", self.fitted[q] - self.config[q], self.error[q]);
        let factor = |q: usize| format!("{axis}: × {:.3} ± {:.3}", self.fitted[q] / self.config[q], (self.error[q] / self.config[q]).abs());
        [("fit-mass", change(0, "kg")), ("fit-viscous", factor(1)), ("fit-coulomb", factor(2)), ("fit-constant", change(3, "N"))]
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

/// Inverse of a 4 x 4 matrix by elimination; `None` if it has none.
fn invert(mut m: [[f64; 4]; 4]) -> Option<[[f64; 4]; 4]> {
    let mut inv = [[0.0; 4]; 4];
    (0..4).for_each(|i| inv[i][i] = 1.0);
    // Columns of very different size (accelerations, ones): judge a pivot against its own column.
    let scale: [f64; 4] = std::array::from_fn(|i| m[i][i]);
    for col in 0..4 {
        let pivot = (col..4).max_by(|&a, &b| m[a][col].abs().total_cmp(&m[b][col].abs()))?;
        if m[pivot][col].abs() < 1e-9 * scale[col].abs() {
            return None;
        }
        m.swap(col, pivot);
        inv.swap(col, pivot);
        let p = m[col][col];
        for j in 0..4 {
            m[col][j] /= p;
            inv[col][j] /= p;
        }
        for row in (0..4).filter(|&r| r != col) {
            let f = m[row][col];
            for j in 0..4 {
                m[row][j] -= f * m[col][j];
                inv[row][j] -= f * inv[col][j];
            }
        }
    }
    Some(inv)
}

/// Least squares of force = mass * acceleration + viscous * speed + coulomb * sign(speed) + constant on
/// the rows where the axis moves: values and standard errors. Force and speed are averaged over the
/// window, the acceleration is the slope of the speed across it. Rows slower than 5 % of the top speed
/// are left out, because there friction is neither of the two terms. `None` if the axis does not move
/// both ways, since one direction alone cannot tell Coulomb friction from a constant force.
fn rigid_values(time: &[f64], force: &[f64], speed: &[f64]) -> Option<([f64; 4], [f64; 4])> {
    let (w, rows) = (WINDOW, time.len());
    let top = speed.iter().fold(0.0, |a: f64, b| a.max(b.abs()));
    let mean = |x: &[f64], i: usize| x[i - w..=i + w].iter().sum::<f64>() / (2 * w + 1) as f64;
    let mut samples = Vec::new();
    for i in w..rows.saturating_sub(w) {
        let v = mean(speed, i);
        if v.abs() > 0.05 * top {
            samples.push(([(speed[i + w] - speed[i - w]) / (time[i + w] - time[i - w]), v, v.signum(), 1.0], mean(force, i)));
        }
    }
    let one_way = |sign: f64| samples.iter().filter(|(x, _)| x[2] == sign).count();
    if one_way(1.0).min(one_way(-1.0)) < 10 * (2 * w + 1) {
        return None;
    }
    let (mut normal, mut right) = ([[0.0; 4]; 4], [0.0; 4]);
    for (x, y) in &samples {
        for a in 0..4 {
            right[a] += x[a] * y;
            for b in 0..4 {
                normal[a][b] += x[a] * x[b];
            }
        }
    }
    let inverse = invert(normal)?;
    let values: [f64; 4] = std::array::from_fn(|a| (0..4).map(|b| inverse[a][b] * right[b]).sum());
    let residual: f64 = samples.iter().map(|(x, y)| (y - (0..4).map(|a| values[a] * x[a]).sum::<f64>()).powi(2)).sum();
    // Neighbouring rows share most of their window, so only every (2w + 1)th row counts as a measurement.
    let variance = residual / (samples.len() - 4) as f64 * (2 * w + 1) as f64;
    Some((values, std::array::from_fn(|a| (variance * inverse[a][a]).sqrt())))
}

/// Stage 1: the rigid values of every axis from a trace, next to the config's. `None` for an axis the
/// trace does not move in both directions.
/// The machine has to pass `trace::per_axis_fit` (a linear transformation and an inertia that does not change with the
/// pose); others are refused with `error-trace-not-linear` and go to `fit_joints`.
pub fn fit_rigid(entry: &MachineEntry, cfg: &Config, trace: &Trace) -> Result<Vec<Option<AxisFit>>, TraceError> {
    let machine = (entry.build)(&crate::catalog::resolve(entry, cfg)?)?;
    if !crate::trace::per_axis_fit(machine.as_ref()) {
        return Err(TraceError { key: "error-trace-not-linear", detail: String::new() });
    }
    let simulated = replay(entry, cfg, trace)?;
    let views = axis_view(machine.as_ref(), trace).into_iter().zip(axis_view(machine.as_ref(), &simulated));
    Ok(views
        .map(|(measured, config)| {
            let (fitted, error) = rigid_values(&trace.time, &measured.0, &measured.1)?;
            Some(AxisFit { config: rigid_values(&trace.time, &config.0, &config.1)?.0, fitted, error })
        })
        .collect())
}

/// One value of the fit in the motors' coordinates (`fit_joints`): its name as the CLI writes it, of what (a motor's
/// id, or `payload`), and as in `AxisFit` the config's estimate, the measured one and its standard error. Per motor in
/// N m, kg m², s and rad at the motor shaft.
#[derive(Debug, Clone, PartialEq)]
pub struct JointValue {
    pub quantity: &'static str,
    pub of: String,
    pub config: f64,
    pub fitted: f64,
    pub error: f64,
}

/// Stage 1 in the motors' coordinates (`fit_joints`).
#[derive(Debug, Clone, PartialEq)]
pub struct JointFit {
    pub values: Vec<JointValue>,
    /// Pairs of `values` (their indices) whose measured estimates correlate by more than `ALIKE`: the trace cannot
    /// tell them apart, only a sum (or difference) of them is known, and each one's error band is wide for that.
    pub alike: Vec<(usize, usize)>,
}

/// Above this correlation of two estimates the trace does not tell them apart.
pub const ALIKE: f64 = 0.95;

/// The quantities of `fit_joints` per motor, after the factor on the payload (`payload_factor`).
pub const JOINT_QUANTITIES: [&str; 4] = ["added_inertia_kg_m2", "viscous_friction_nm_s_rad", "coulomb_friction_nm", "constant_torque_nm"];

impl JointFit {
    /// What the fit says to change in the config, as translation keys with their argument: the factor on the payload,
    /// per motor the changes of inertia, friction and constant torque at its shaft, and the pairs that cannot be told
    /// apart.
    pub fn advice(&self) -> Vec<(&'static str, String)> {
        let mut out: Vec<(&'static str, String)> = self
            .values
            .iter()
            .map(|v| {
                let change = |unit: &str| format!("{}: {:+.4} {unit} ± {:.4}", v.of, v.fitted - v.config, v.error);
                match v.quantity {
                    "payload_factor" => ("fit-payload", format!("× {:.3} ± {:.3}", v.fitted / v.config, v.error / v.config)),
                    "added_inertia_kg_m2" => ("fit-joint-inertia", format!("{}: {:+.3e} kg m² ± {:.1e}", v.of, v.fitted - v.config, v.error)),
                    "viscous_friction_nm_s_rad" => ("fit-joint-viscous", format!("{}: {:+.2e} N m s/rad ± {:.1e}", v.of, v.fitted - v.config, v.error)),
                    "coulomb_friction_nm" => ("fit-joint-coulomb", change("N m")),
                    _ => ("fit-joint-constant", change("N m")),
                }
            })
            .collect();
        let name = |i: usize| format!("{} {}", self.values[i].quantity, self.values[i].of);
        out.extend(self.alike.iter().map(|&(a, b)| ("fit-alike", format!("{} / {}", name(a), name(b)))));
        out
    }
}

/// Least squares of the motor torques on the machine's rigid model in regressor form, τ = Y(q, q̇, q̈) θ, in the motors'
/// own coordinates: per motor k
///
/// τ_k − τ_k,rigid = (θ_p − 1) (τ_k,rigid − τ_k,rigid without payload) + ΔJ_k q̈_k + b_k q̇_k + c_k sign q̇_k + τ0_k,
///
/// the rigid torques from `Machine::rigid_motor_torque` at the row's angles, speeds and accelerations. Torque and speed
/// averaged over the window as `rigid_values` does; the acceleration from the angle, (q[i+w] − 2 q[i] + q[i−w]) / (w dt)²,
/// since an encoder's angle is far less noisy than the speed differentiated, and noise on a regressor biases its
/// value towards zero (the speed's slope gave 14 % too little payload on the delta). A motor's row only where it turns
/// faster than 5 % of its top speed. Values, standard errors and the covariance, in the order [θ_p, then per motor
/// ΔJ, b, c, τ0]; θ_p is NaN, and left out, where the payload adds nothing (none in the config). `None` where a motor
/// does not turn both ways or the values cannot be solved for.
///
/// A factor on the rest of the rigid model (arms, platform, rotors) is not fitted: on the delta it correlated by more
/// than 0.95 with the motors' added inertias and constant torques together (gravity is nearly constant over a trace,
/// and each arm's inertia sits on its own motor), so it made all of them unknown.
fn joint_values(machine: &mut dyn sim_core::Machine, trace: &Trace) -> Option<(Vec<f64>, Vec<f64>, Vec<Vec<f64>>)> {
    let (w, rows, n) = (WINDOW, trace.time.len(), trace.motors.len());
    let p = 1 + 4 * n;
    let mean = |x: &[f64], i: usize| x[i - w..=i + w].iter().sum::<f64>() / (2 * w + 1) as f64;
    let top: Vec<f64> = trace.motors.iter().map(|m| m.speed.iter().fold(0.0, |a: f64, b| a.max(b.abs()))).collect();
    let mut samples: Vec<(Vec<f64>, f64)> = Vec::new();
    let mut ways = vec![[0usize; 2]; n];
    let (mut with, mut without) = (vec![0.0; n], vec![0.0; n]);
    for i in w..rows.saturating_sub(w) {
        let angle: Vec<f64> = trace.motors.iter().map(|m| m.angle[i]).collect();
        let speed: Vec<f64> = trace.motors.iter().map(|m| mean(&m.speed, i)).collect();
        let half = 0.5 * (trace.time[i + w] - trace.time[i - w]);
        let acc: Vec<f64> = trace.motors.iter().map(|m| (m.angle[i + w] - 2.0 * m.angle[i] + m.angle[i - w]) / (half * half)).collect();
        machine.set_payload_attached(false);
        machine.rigid_motor_torque(&angle, &speed, &acc, &mut without);
        machine.set_payload_attached(true);
        machine.rigid_motor_torque(&angle, &speed, &acc, &mut with);
        for k in (0..n).filter(|&k| speed[k].abs() > 0.05 * top[k]) {
            let mut x = vec![0.0; p];
            x[0] = with[k] - without[k];
            x[1 + 4 * k..5 + 4 * k].copy_from_slice(&[acc[k], speed[k], speed[k].signum(), 1.0]);
            ways[k][usize::from(speed[k] > 0.0)] += 1;
            samples.push((x, mean(&trace.motors[k].torque, i) - with[k]));
        }
    }
    if ways.iter().any(|both| both[0].min(both[1]) < 10 * (2 * w + 1)) {
        return None;
    }
    // Columns scaled to unit length, as their sizes span decades; a payload column of zeros is left out.
    let norm: Vec<f64> = (0..p).map(|j| samples.iter().map(|(x, _)| x[j] * x[j]).sum::<f64>().sqrt()).collect();
    let used: Vec<usize> = (0..p).filter(|&j| j > 0 || norm[0] > 1e-12).collect();
    let m = used.len();
    let (mut normal, mut right) = (vec![0.0; m * m], vec![0.0; m]);
    for (x, y) in &samples {
        let z: Vec<f64> = used.iter().map(|&j| x[j] / norm[j]).collect();
        for a in 0..m {
            right[a] += z[a] * y;
            for b in 0..m {
                normal[a * m + b] += z[a] * z[b];
            }
        }
    }
    let unit = |a: usize| (0..m).map(|b| if a == b { 1.0 } else { 0.0 }).collect::<Vec<f64>>();
    let inverse: Vec<Vec<f64>> = (0..m).map(|a| crate::chain::solve(&normal, &unit(a))).collect::<Option<_>>()?;
    let mut values = vec![f64::NAN; p];
    for (a, &j) in used.iter().enumerate() {
        values[j] = (0..m).map(|b| inverse[a][b] * right[b]).sum::<f64>() / norm[j];
    }
    let residual: f64 = samples.iter().map(|(x, y)| (y - used.iter().map(|&j| values[j] * x[j]).sum::<f64>()).powi(2)).sum();
    // Neighbouring rows share most of their window, so only every (2w + 1)th row counts as a measurement.
    let variance = residual / (samples.len() - m) as f64 * (2 * w + 1) as f64;
    let mut covariance = vec![vec![f64::NAN; p]; p];
    for (a, &i) in used.iter().enumerate() {
        for (b, &j) in used.iter().enumerate() {
            covariance[i][j] = variance * inverse[a][b] / (norm[i] * norm[j]);
        }
    }
    // Fitted as a change, as the rest; the config's own payload counts 1.
    values[0] += 1.0;
    let errors = (0..p).map(|j| covariance[j][j].sqrt()).collect();
    Some((values, errors, covariance))
}

/// Stage 1 in the motors' own coordinates, for every machine, and the only one for a machine whose motors do not turn
/// in proportion to its axes (the delta's arms, a SCARA, an arm): the motor torques regressed on the machine's own
/// rigid model (`joint_values`), on the trace and on the config's replay of it. For the config the payload's factor
/// comes out near 1 and the added inertias near 0, friction and constant torque as the config has them at the motor
/// shaft, gearbox losses and all, so that the difference is what the config is off by. `None` where a motor does not
/// turn both ways in the trace.
///
/// Each motor's added inertia stands for its arm or link's own inertia about the joint as well as its rotor's: the
/// trace cannot tell those apart. What else it cannot tell apart depends on the moves and is listed in `alike`; a trace
/// that swings every axis over much of its range at several frequencies (docs/measurement-procedure.md) leaves none.
pub fn fit_joints(entry: &MachineEntry, cfg: &Config, trace: &Trace) -> Result<Option<JointFit>, TraceError> {
    let simulated = replay(entry, cfg, trace)?;
    let mut machine = (entry.build)(&crate::catalog::resolve(entry, cfg)?)?;
    let ids: Vec<&str> = machine.motors().iter().map(|m| m.id).collect();
    let (Some((fitted, error, covariance)), Some((config, ..))) = (joint_values(machine.as_mut(), trace), joint_values(machine.as_mut(), &simulated)) else {
        return Ok(None);
    };
    let names = std::iter::once(("payload_factor", "payload".to_string())).chain(ids.iter().flat_map(|id| JOINT_QUANTITIES.map(|q| (q, id.to_string()))));
    let kept: Vec<usize> = (0..fitted.len()).filter(|&j| fitted[j].is_finite()).collect();
    let values: Vec<JointValue> = names
        .enumerate()
        .filter(|(j, _)| fitted[*j].is_finite())
        .map(|(j, (quantity, of))| JointValue { quantity, of, config: config[j], fitted: fitted[j], error: error[j] })
        .collect();
    let mut alike = Vec::new();
    for (a, &i) in kept.iter().enumerate() {
        for (b, &j) in kept.iter().enumerate().skip(a + 1) {
            if (covariance[i][j] / (covariance[i][i] * covariance[j][j]).sqrt()).abs() > ALIKE {
                alike.push((a, b));
            }
        }
    }
    Ok(Some(JointFit { values, alike }))
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
    let step = (sweep[below + 1].frequency - sweep[below].frequency).max(sweep[above].frequency - sweep[above - 1].frequency);
    Some(Resonance { frequency, damping: (high - low) / (2.0 * frequency), frequency_error: 0.5 * step })
}

/// A frequency response as `sim-cli sweep` writes it: the columns `frequency_hz` and `gain_rad_s_per_nm`. A
/// phase is not read; the fit does not use it.
pub fn parse_sweep(text: &str) -> Result<Vec<SweepPoint>, TraceError> {
    let table = crate::trace::columns(text, &["frequency_hz".to_string(), "gain_rad_s_per_nm".to_string()])?;
    Ok(table[0].iter().zip(&table[1]).map(|(&frequency, &gain)| SweepPoint { frequency, gain, phase_deg: 0.0 }).collect())
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
pub fn fit_sweep(entry: &MachineEntry, cfg: &Config, motor: usize, measured: &[SweepPoint], amplitude: f64) -> Result<Option<SweepFit>, ConfigError> {
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
        out.extend((w..rows.saturating_sub(w)).map(|i| m.torque[i] - m.torque[i - w..=i + w].iter().sum::<f64>() / (2 * w + 1) as f64));
    }
    out
}

/// Stage 2 from an ordinary move's trace, no sweep: an output-error fit of the stiffness. The config is replayed with
/// the value at `key` (the belts' stiffness, `BELT_STIFFNESS_KEY`, or another spring between motor and load) times a
/// factor, and the factor is the one whose replay's motor torques ring as the trace's do (`ringing`: least squares over
/// every motor and row). The replay has the config's loops, filters, delays and friction, which damp the ringing of a
/// move in the trace as they do in the machine, so that they need not be fitted as well; a free two-mass fit of the
/// torque took the drive's own lag, at 51 Hz on the single axis, for the belt. The masses have to be right first (stage
/// 1), as for a sweep.
///
/// The factor is searched from 1/4 to 4: nine replays a factor of 1.41 apart, then a parabola through seven replays
/// between the neighbours of the best, and another through seven within ±10 % of its vertex (24 replays in all). Its
/// standard error from the curvature 2 a of the last parabola: var(ln factor) = 2 σ² / (2 a), σ² = S / (N − 1) widened
/// by (1 + ρ) / (1 − ρ) for the lag-one correlation ρ of the residuals (the ringing of neighbouring rows is not
/// independent), plus the variance of the vertex from the points' scatter about the parabola (on a trace without noise
/// that is all there is: 0.70 on the single axis came out 0.681 ± 0.001 without it).
///
/// Refused with `fit-move-not-excited` where that error is above `MOVE_STIFFNESS_LIMIT` of the factor or the best
/// factor lies at an end of the range: the trace's moves did not ring the load enough against its noise (a gentle
/// move, a low jerk, a heavily damped mode). With `fit-move-no-key` where the config has no number at `key`.
pub fn fit_move_stiffness(entry: &MachineEntry, cfg: &Config, trace: &Trace, key: &str) -> Result<MoveStiffness, TraceError> {
    let base = cfg.f64(key).map_err(|_| TraceError { key: "fit-move-no-key", detail: key.to_string() })?;
    let measured = ringing(trace);
    let cost = |u: f64| -> Result<(f64, Vec<f64>), TraceError> {
        let mut scaled = cfg.clone();
        scaled.set_text(&format!("{key} = {:?}", base * u.exp()))?;
        let residuals: Vec<f64> = ringing(&replay(entry, &scaled, trace)?).iter().zip(&measured).map(|(s, m)| m - s).collect();
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
        let points = (0..7).map(|i| from + (to - from) * i as f64 / 6.0).map(|u| cost(u).map(|c| (u - mid, c.0))).collect::<Result<Vec<_>, _>>()?;
        let (mut a, mut b) = (vec![0.0; 9], vec![0.0; 3]);
        for (x, c) in &points {
            let row = [1.0, *x, x * x];
            for r in 0..3 {
                b[r] += row[r] * c;
                (0..3).for_each(|k| a[3 * r + k] += row[r] * row[k]);
            }
        }
        let Some(v) = crate::chain::solve(&a, &b).filter(|v| v[2] > 0.0) else { return Ok(None) };
        let scatter = points.iter().map(|(x, c)| (c - v[0] - v[1] * x - v[2] * x * x).powi(2)).sum::<f64>() / (points.len() - 3) as f64;
        let linear = crate::chain::solve(&a, &[0.0, 1.0, 0.0]).map_or(f64::INFINITY, |column| column[1]);
        Ok(Some(((mid - 0.5 * v[1] / v[2]).clamp(from, to), 2.0 * v[2], (scatter * linear).sqrt() / (2.0 * v[2]))))
    };
    let curved = |found: Option<(f64, f64, f64)>| found.ok_or_else(|| not_excited(format!("× {:.2} ± inf", grid[best].exp())));
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
            .map(|f| SweepPoint { frequency: f, gain: 1000.0 * dip(f, 22.0, 0.05).min(1.0) * dip(f, 62.0, 0.01).min(1.0) / f, phase_deg: 0.0 })
            .collect();
        // The dip at 62 Hz goes down to 0.02, the one at 22 Hz to 0.1.
        let found = resonance(&sweep).unwrap();
        assert!((found.frequency - 22.0).abs() < 0.5 && (found.damping - 0.05).abs() < 0.01, "{found:?}");
    }
}
