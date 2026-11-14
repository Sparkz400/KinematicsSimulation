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
            samples.push((
                [(speed[i + w] - speed[i - w]) / (time[i + w] - time[i - w]), v, v.signum(), 1.0],
                mean(force, i),
            ));
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
    /// Where the payload was picked or dropped during the trace: the time and its standard error, in s. The payload's
    /// factor is then fitted before (`payload_factor_before`) and after (`payload_factor_after`) it.
    pub payload_step: Option<(f64, f64)>,
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
/// count (`JointFit::payload_step`), more than the trace's after less before differs from the config's replay's. Well
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

/// The payload's factor before and after a step (`JointFit::payload_step`), in place of `payload_factor`.
pub const PAYLOAD_STEP_QUANTITIES: [&str; 2] = ["payload_factor_before", "payload_factor_after"];

impl JointFit {
    /// What the fit says to change in the config, as translation keys with their argument: the factor on the payload
    /// (or its step), per motor the changes of inertia, friction and constant torque at its shaft, per link its mass
    /// factor and first moment; then what each base parameter combines, what has no effect and the pairs that cannot
    /// be told apart.
    pub fn advice(&self) -> Vec<(&'static str, String)> {
        let factor = |v: &JointValue| format!("× {:.3} ± {:.3}", v.fitted / v.config, (v.error / v.config).abs());
        let mut out: Vec<(&'static str, String)> = Vec::new();
        if let Some((time, error)) = self.payload_step {
            let at = |q: &str| self.values.iter().find(|v| v.quantity == q).map(factor).unwrap_or_default();
            let detail =
                format!("{time:.3} s ± {error:.3}: {} → {}", at("payload_factor_before"), at("payload_factor_after"));
            out.push(("fit-payload-step", detail));
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

/// The stacked regressor of the motor torques on the machine's rigid model, τ − τ_rigid = Y θ, in the motors' own
/// coordinates: per motor k a row
///
/// [τ_k,rigid − τ_k,rigid without payload, then per motor q̈_k, q̇_k, sign q̇_k, 1 at motor k's own place, then per
/// link (`Machine::links`) the change of τ_k,rigid with the link's inertial parameters doubled and with 0.01 kg m more
/// first moment along x, y and z, per unit],
///
/// the rigid torques from `Machine::rigid_motor_torque` at the row's angles, speeds and accelerations, linear in each
/// of these. Torque and speed averaged over the window as `rigid_values` does; the acceleration from the angle, (q[i+w]
/// − 2 q[i] + q[i−w]) / (w dt)², since an encoder's angle is far less noisy than the speed differentiated, and noise on
/// a regressor biases its value towards zero (the speed's slope gave 14 % too little payload on the delta). A motor's
/// row only where it turns faster than 5 % of its top speed. A link's column that changes the stacked rigid torque by
/// less than 1e-9 of it is set to zeros: rounding, of a parameter that has no effect. `None` where a motor does not
/// turn both ways.
fn regressor(machine: &mut dyn Machine, trace: &Trace) -> Option<Vec<Sample>> {
    let (w, rows, n, links) = (WINDOW, trace.time.len(), trace.motors.len(), machine.links().len());
    let p = 1 + 4 * n + 4 * links;
    let mean = |x: &[f64], i: usize| x[i - w..=i + w].iter().sum::<f64>() / (2 * w + 1) as f64;
    let top: Vec<f64> = trace.motors.iter().map(|m| m.speed.iter().fold(0.0, |a: f64, b| a.max(b.abs()))).collect();
    let mut samples: Vec<Sample> = Vec::new();
    let mut ways = vec![[0usize; 2]; n];
    let (mut with, mut without, mut changed) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
    let steps = [
        (2.0, [0.0; 3]),
        (1.0, [MOMENT_STEP, 0.0, 0.0]),
        (1.0, [0.0, MOMENT_STEP, 0.0]),
        (1.0, [0.0, 0.0, MOMENT_STEP]),
    ];
    // The stacked rigid torque's and each link column's change's sum of squares, for the columns of no effect.
    let (mut rigid, mut link_change) = (0.0, vec![0.0; 4 * links]);
    for i in w..rows.saturating_sub(w) {
        let moving: Vec<usize> = (0..n).filter(|&k| mean(&trace.motors[k].speed, i).abs() > 0.05 * top[k]).collect();
        if moving.is_empty() {
            continue;
        }
        let angle: Vec<f64> = trace.motors.iter().map(|m| m.angle[i]).collect();
        let speed: Vec<f64> = trace.motors.iter().map(|m| mean(&m.speed, i)).collect();
        let half = 0.5 * (trace.time[i + w] - trace.time[i - w]);
        let acc: Vec<f64> =
            trace.motors.iter().map(|m| (m.angle[i + w] - 2.0 * m.angle[i] + m.angle[i - w]) / (half * half)).collect();
        machine.set_payload_attached(false);
        machine.rigid_motor_torque(&angle, &speed, &acc, &mut without);
        machine.set_payload_attached(true);
        machine.rigid_motor_torque(&angle, &speed, &acc, &mut with);
        let mut per_link = vec![vec![0.0; n]; 4 * links];
        for l in 0..links {
            for (c, (factor, moment)) in steps.iter().enumerate() {
                machine.set_link_change(l, *factor, *moment);
                machine.rigid_motor_torque(&angle, &speed, &acc, &mut changed);
                let size = if c == 0 { 1.0 } else { MOMENT_STEP };
                per_link[4 * l + c] = (0..n).map(|k| (changed[k] - with[k]) / size).collect();
            }
            machine.set_link_change(l, 1.0, [0.0; 3]);
        }
        for k in moving {
            let mut x = vec![0.0; p];
            x[0] = with[k] - without[k];
            x[1 + 4 * k..5 + 4 * k].copy_from_slice(&[acc[k], speed[k], speed[k].signum(), 1.0]);
            for (c, column) in per_link.iter().enumerate() {
                x[1 + 4 * n + c] = column[k];
                let size = if c % 4 == 0 { 1.0 } else { MOMENT_STEP };
                link_change[c] += (column[k] * size).powi(2);
            }
            rigid += with[k] * with[k];
            ways[k][usize::from(speed[k] > 0.0)] += 1;
            samples.push((i, x, mean(&trace.motors[k].torque, i) - with[k]));
        }
    }
    if ways.iter().any(|both| both[0].min(both[1]) < 10 * (2 * w + 1)) {
        return None;
    }
    for (c, change) in link_change.iter().enumerate() {
        if change.sqrt() <= 1e-9 * rigid.sqrt() {
            samples.iter_mut().for_each(|(_, x, _)| x[1 + 4 * n + c] = 0.0);
        }
    }
    Some(samples)
}

/// The regressor with the payload's column split at the trace's row `step` (before it, from it on), where there is one.
fn split(samples: &[Sample], step: Option<usize>) -> Vec<Sample> {
    let Some(step) = step else { return samples.to_vec() };
    samples
        .iter()
        .map(|(row, x, y)| {
            let before = if *row < step { x[0] } else { 0.0 };
            let mut out = vec![before, x[0] - before];
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
/// Standard errors as `rigid_values`, the variance widened by the window. `None` where the normal matrix cannot be
/// solved.
fn solve(samples: &[Sample], basis: Option<&[usize]>) -> Option<Solved> {
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
    // Neighbouring rows share most of their window, so only every (2w + 1)th row counts as a measurement.
    let variance = residual / (samples.len() - m) as f64 * (2 * WINDOW + 1) as f64;
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

/// Where the payload was picked or dropped during the trace, if it was: the payload's column split at every row in
/// turn (`split`), on the base parameters of the fit without a step, and the row whose split leaves the least sum of
/// squares; every 11th row between a tenth and nine tenths of the trace, then every row about the best. Its standard
/// error is how far from it the sum of squares stays within one variance (as `solve` takes it, widened by the window)
/// of its least. The step counts where the two factors differ by more than `PAYLOAD_STEP_SIGMAS` of their difference's
/// standard error. In: the regressor, the basis without a step, the trace's times; out: the row and the time's error.
///
/// A sum of squares per split from running sums along the rows: only the payload's column changes with the split.
fn payload_step(samples: &[Sample], basis: &[usize], time: &[f64]) -> Option<(usize, f64)> {
    if basis.first() != Some(&0) {
        return None;
    }
    let (m, rows) = (basis.len(), time.len());
    let norm: Vec<f64> =
        basis.iter().map(|&j| samples.iter().map(|(_, x, _)| x[j] * x[j]).sum::<f64>().sqrt()).collect();
    let (mut gram, mut right, mut yy) = (vec![0.0; m * m], vec![0.0; m], 0.0);
    // Per row of the trace: the payload column's products with every column and with the torque, summed up to it.
    let mut running = vec![vec![0.0; m + 1]; rows + 1];
    let mut by_row = vec![vec![0.0; m + 1]; rows];
    for (row, x, y) in samples {
        let z: Vec<f64> = basis.iter().zip(&norm).map(|(&j, n)| x[j] / n).collect();
        for a in 0..m {
            right[a] += z[a] * y;
            for b in 0..m {
                gram[a * m + b] += z[a] * z[b];
            }
            by_row[*row][a] += z[0] * z[a];
        }
        by_row[*row][m] += z[0] * y;
        yy += y * y;
    }
    for row in 0..rows {
        running[row + 1] = (0..=m).map(|c| running[row][c] + by_row[row][c]).collect();
    }
    // The sum of squares left with the split before `row`: columns before, after, then the basis's others.
    let left = |row: usize| -> Option<f64> {
        let s = &running[row];
        let k = m + 1;
        let mut normal = vec![0.0; k * k];
        let mut rhs = vec![0.0; k];
        normal[0] = s[0];
        normal[k + 1] = gram[0] - s[0];
        rhs[0] = s[m];
        rhs[1] = right[0] - s[m];
        for a in 1..m {
            normal[a + 1] = s[a];
            normal[(a + 1) * k] = s[a];
            normal[k + a + 1] = gram[a] - s[a];
            normal[(a + 1) * k + 1] = gram[a] - s[a];
            rhs[a + 1] = right[a];
            for b in 1..m {
                normal[(a + 1) * k + b + 1] = gram[a * m + b];
            }
        }
        let theta = crate::chain::solve(&normal, &rhs)?;
        Some(yy - theta.iter().zip(&rhs).map(|(t, r)| t * r).sum::<f64>())
    };
    let coarse = 2 * WINDOW + 1;
    let (from, to) = (rows / 10, rows - rows / 10);
    let best_of = |rows: &mut dyn Iterator<Item = usize>| {
        rows.filter_map(|r| left(r).map(|s| (r, s))).min_by(|a, b| a.1.total_cmp(&b.1))
    };
    let (rough, _) = best_of(&mut (from..to).step_by(coarse))?;
    let near = rough.saturating_sub(coarse).max(from)..(rough + coarse).min(to);
    let (row, least) = best_of(&mut near.clone())?;
    let variance = least / (samples.len() - m - 1) as f64 * coarse as f64;
    let spread = near
        .filter(|&r| left(r).is_some_and(|s| s - least <= variance))
        .map(|r| (time[r] - time[row]).abs())
        .fold(0.0, f64::max);
    Some((row, spread.max(time[row] - time[row - 1])))
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
/// (docs/measurement-procedure.md) leaves none of the motors'.
///
/// A payload picked or dropped during the trace (no gripper column in a trace, so from its torques: `payload_step`) is
/// fitted before and after. The config's replay keeps its payload and is fitted with the same split; the step counts
/// where the trace's factor after less before differs from the replay's by more than `PAYLOAD_STEP_SIGMAS` standard
/// errors, so that what the rigid model misses of the machine over the trace, the same in both, is not taken for one.
///
/// A factor on the rest of the rigid model (arms, platform, rotors) is not fitted: on the delta it correlated by more
/// than 0.95 with the motors' added inertias and constant torques together (gravity is nearly constant over a trace,
/// and each arm's inertia sits on its own motor), so it made all of them unknown.
pub fn fit_joints(entry: &MachineEntry, cfg: &Config, trace: &Trace) -> Result<Option<JointFit>, TraceError> {
    let simulated = replay(entry, cfg, trace)?;
    let mut machine = (entry.build)(&crate::catalog::resolve(entry, cfg)?)?;
    let ids: Vec<String> = machine.motors().iter().map(|m| m.id.to_string()).collect();
    let links = machine.links();
    let (Some(measured), Some(config)) = (regressor(machine.as_mut(), trace), regressor(machine.as_mut(), &simulated))
    else {
        return Ok(None);
    };
    let Some(plain) = solve(&measured, None) else { return Ok(None) };
    let stepped = payload_step(&measured, &plain.basis, &trace.time).and_then(|(row, error)| {
        let fit = solve(&split(&measured, Some(row)), None).filter(|f| f.basis.starts_with(&[0, 1]))?;
        let own = solve(&split(&config, Some(row)), Some(&fit.basis))?;
        let c = &fit.covariance;
        let sigma = (c[0][0] + c[1][1] - 2.0 * c[0][1]).sqrt();
        let step = (fit.values[1] - fit.values[0]) - (own.values[1] - own.values[0]);
        (step.abs() > PAYLOAD_STEP_SIGMAS * sigma).then_some((row, error, fit, own))
    });
    let (step, fitted, replayed) = match stepped {
        Some((row, error, fit, own)) => (Some((row, error)), fit, own),
        None => match solve(&config, Some(&plain.basis)) {
            Some(own) => (None, plain, own),
            None => return Ok(None),
        },
    };
    let payload: Vec<&'static str> =
        if step.is_some() { PAYLOAD_STEP_QUANTITIES.to_vec() } else { vec!["payload_factor"] };
    let factors = payload.len();
    let names: Vec<(&'static str, String)> = payload
        .iter()
        .map(|q| (*q, "payload".to_string()))
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
    let payload_step = step.map(|(row, error)| (trace.time[row], error));
    Ok(Some(JointFit { values, alike, folded, payload_step }))
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
