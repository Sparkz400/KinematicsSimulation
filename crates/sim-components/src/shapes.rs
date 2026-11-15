//! What a machine is, beside what it does: its modes with their shapes (from the machine, or from its elastic model
//! linearised at a pose) and the stiffness each mode lives in; how well it moves its tool at a place of its workspace;
//! and the envelope of traces. For the app's views and `sim-cli modes --shapes` and `workspace`. No machine is named:
//! everything comes through the `Machine` interface and the config.

use sim_core::{Config, ConfigError, Machine, MachineEntry};

/// A complex number as [re, im].
type C = [f64; 2];

fn mul(a: C, b: C) -> C {
    [a[0] * b[0] - a[1] * b[1], a[0] * b[1] + a[1] * b[0]]
}
fn sub(a: C, b: C) -> C {
    [a[0] - b[0], a[1] - b[1]]
}
fn div(a: C, b: C) -> C {
    let d = b[0] * b[0] + b[1] * b[1];
    [(a[0] * b[0] + a[1] * b[1]) / d, (a[1] * b[0] - a[0] * b[1]) / d]
}
fn abs(a: C) -> f64 {
    a[0].hypot(a[1])
}

/// The machine at rest at `pose` (`Machine::equilibrium`) and its small-motion matrix there: A = d(derivatives) /
/// d(state), n x n row after row, by central differences with the motor torques held at those that hold it at rest.
/// The motors are free in it, as on a machine with its drives off: no control loop holds them.
pub fn linearised(machine: &dyn Machine, pose: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let n = machine.state_len();
    let (mut rest, mut torque) = (Vec::new(), vec![0.0; machine.motors().len()]);
    machine.equilibrium(pose, &mut rest, &mut torque);
    let (mut a, mut up, mut down, mut state) = (vec![0.0; n * n], vec![0.0; n], vec![0.0; n], rest.clone());
    for j in 0..n {
        // A step of 1e-7 of the state's size: forces of 0.05 N on a belt of 5e5 N per unit strain, far above rounding.
        let h = 1e-7 * rest[j].abs().max(1.0);
        state[j] = rest[j] + h;
        machine.derivatives(&state, &torque, &mut up);
        state[j] = rest[j] - h;
        machine.derivatives(&state, &torque, &mut down);
        state[j] = rest[j];
        (0..n).for_each(|i| a[i * n + j] = (up[i] - down[i]) / (2.0 * h));
    }
    (rest, a)
}

/// The eigenvalue of the n x n matrix `a` nearest `shift` and its eigenvector, by inverse iteration: (A - shift I) is
/// factorised (LU with partial pivoting) and solved fifty times, each solve multiplying the part of the nearest
/// eigenvector by 1 / |lambda - shift| more than any other; then once more from the eigenvalue found, as Rayleigh's
/// iteration does, since another eigenvalue (a free machine's rigid motion at 0) may lie nearly as near the shift and
/// fade slowly. The vector is scaled so that its largest entry is 1 (real). `None` where the matrix holds a number that
/// is not finite.
pub fn eigen_near(a: &[f64], n: usize, shift: [f64; 2]) -> Option<([f64; 2], Vec<[f64; 2]>)> {
    if n == 0 || a.iter().any(|x| !x.is_finite()) {
        return None;
    }
    // A start with a part of every eigenvector: no entry zero, and not all in phase.
    let start: Vec<C> = (0..n).map(|i| [1.0, 0.37 * (i as f64 + 1.0).sin()]).collect();
    let (lambda, x) = inverse_iteration(a, n, shift, start, 50)?;
    inverse_iteration(a, n, lambda, x, 10)
}

/// `count` solves of (A - shift I) y = x from `x`, each scaled so that its largest entry is 1, and the eigenvalue at the
/// end: (A x)_k / x_k at that entry.
fn inverse_iteration(a: &[f64], n: usize, shift: C, mut x: Vec<C>, count: usize) -> Option<(C, Vec<C>)> {
    let mut m: Vec<C> = a.iter().map(|&x| [x, 0.0]).collect();
    (0..n).for_each(|i| m[i * n + i] = sub(m[i * n + i], shift));
    let mut order: Vec<usize> = (0..n).collect();
    for k in 0..n {
        let p = (k..n).max_by(|&i, &j| abs(m[i * n + k]).total_cmp(&abs(m[j * n + k])))?;
        if p != k {
            (0..n).for_each(|c| m.swap(k * n + c, p * n + c));
            order.swap(k, p);
        }
        // A shift that is an eigenvalue exactly: the solve then gives that eigenvector at once.
        if abs(m[k * n + k]) == 0.0 {
            m[k * n + k] = [1e-300, 0.0];
        }
        for i in k + 1..n {
            let f = div(m[i * n + k], m[k * n + k]);
            m[i * n + k] = f;
            for c in k + 1..n {
                m[i * n + c] = sub(m[i * n + c], mul(f, m[k * n + c]));
            }
        }
    }
    let solve = |b: &[C]| -> Vec<C> {
        let mut y: Vec<C> = order.iter().map(|&p| b[p]).collect();
        for i in 0..n {
            for c in 0..i {
                y[i] = sub(y[i], mul(m[i * n + c], y[c]));
            }
        }
        for i in (0..n).rev() {
            for c in i + 1..n {
                y[i] = sub(y[i], mul(m[i * n + c], y[c]));
            }
            y[i] = div(y[i], m[i * n + i]);
        }
        y
    };
    let largest = |x: &[C]| (0..n).max_by(|&i, &j| abs(x[i]).total_cmp(&abs(x[j]))).unwrap_or(0);
    for _ in 0..count {
        x = solve(&x);
        let top = x[largest(&x)];
        if !(abs(top) > 0.0 && abs(top).is_finite()) {
            return None;
        }
        x.iter_mut().for_each(|v| *v = div(*v, top));
    }
    let k = largest(&x);
    let ax = (0..n).fold([0.0, 0.0], |sum: C, c| [sum[0] + a[k * n + c] * x[c][0], sum[1] + a[k * n + c] * x[c][1]]);
    Some((div(ax, x[k]), x))
}

/// An eigenvalue as a mode: its undamped natural frequency |lambda| / 2 pi in Hz and its damping ratio -Re / |lambda|.
pub fn as_mode(lambda: [f64; 2]) -> sim_core::Mode {
    let w = abs(lambda);
    sim_core::Mode { hz: w / std::f64::consts::TAU, damping: if w > 0.0 { -lambda[0] / w } else { 0.0 } }
}

/// The eigenvalue of a mode of `hz` and `damping` with its positive imaginary part, where inverse iteration starts.
fn shift_of(hz: f64, damping: f64) -> C {
    let w = std::f64::consts::TAU * hz;
    [-damping.clamp(0.0, 0.99) * w, w * (1.0 - damping.clamp(0.0, 0.99).powi(2)).sqrt()]
}

/// The kinds of stiffness a mode can live in, each with the label key `ui-modes-stiffness-<kind>`.
pub const STIFFNESS_KINDS: [&str; 6] = ["belt", "gearbox", "guide", "frame", "base", "link"];

/// The kind of stiffness a config value is, by its path: a number whose key names a stiffness (`*stiffness*`, or a
/// Young's modulus), in a table of belts, gearboxes, guides and what runs on them (friction contacts, racks, screws,
/// slides), the frame, the base or the links. End stops and a payload's swing are left out: they hold nothing at rest.
pub fn stiffness_kind(path: &str) -> Option<&'static str> {
    let (table, key) = (path.split('.').next()?, path.rsplit('.').next()?);
    let named = key.contains("stiffness") || key.contains("youngs_modulus");
    if !named || ["stop", "swing", "_t1", "_t2", "scale", "coefficient"].iter().any(|w| key.contains(w)) {
        return None;
    }
    Some(match table {
        t if t.starts_with("belt") => "belt",
        "gearbox" => "gearbox",
        t if t.starts_with("friction") || ["guide", "rack", "screw", "slide"].contains(&t) => "guide",
        "frame" => "frame",
        "base" => "base",
        t if t.starts_with("link") || t == "arm" => "link",
        _ => return None,
    })
}

/// A shape shown swinging in the 3D view (`session::mode_scene`): the pose the machine rests at, the mode's frequency
/// and its shape as a complex direction in the state, re and im. The state swings as rest + a (re sin wt + im cos wt).
#[derive(Debug, Clone, PartialEq)]
pub struct ShownMode {
    pub pose: Vec<f64>,
    pub hz: f64,
    pub re: Vec<f64>,
    pub im: Vec<f64>,
}

/// One mode of the machine at a pose, for the mode list.
#[derive(Debug, Clone)]
pub struct ModeView {
    /// As the machine reports it (`Machine::modes`): Hz and damping ratio.
    pub hz: f64,
    pub damping: f64,
    /// The mode of the linearised model nearest it, whose shape is shown where the machine gives none; `None` where
    /// the linearisation found none.
    pub linear: Option<sim_core::Mode>,
    /// The shape is the machine's own (`Machine::mode_shape`), not the linearised model's.
    pub from_machine: bool,
    pub shape: ShownMode,
    /// The share of the mode's strain energy in each kind of stiffness (`STIFFNESS_KINDS`), largest first: twice the
    /// change of ln f over that of ln k when the kind's stiffnesses in the config grow by 10 % (Rayleigh: d(w^2) / dk_i
    /// is the energy in k_i over the modal mass). Empty where the linearised model has no mode near it.
    pub stiffness: Vec<(&'static str, f64)>,
}

/// The machine `config` builds, with its parts in.
pub fn build(entry: &MachineEntry, config: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    (entry.build)(&crate::catalog::resolve(entry, config)?)
}

/// The modes the machine reports at `pose`, lowest first, each with a shape: the machine's own for the mode it gives one
/// of, else that of the mode of its linearised model nearest the reported one (`linearised`, `eigen_near`), scaled so
/// that the tool moves by at most 1 m per unit (or, where it does not move, the largest entry is 1).
pub fn mode_views(entry: &MachineEntry, config: &Config, pose: &[f64]) -> Result<Vec<ModeView>, ConfigError> {
    let machine = build(entry, config)?;
    let modes = machine.modes(pose);
    if modes.is_empty() {
        return Ok(Vec::new());
    }
    let n = machine.state_len();
    let (rest, a) = linearised(machine.as_ref(), pose);
    let own = machine.mode_shape(pose);
    let kinds = stiffness_leaves(config);
    let views = modes.iter().map(|mode| {
        let found = eigen_near(&a, n, shift_of(mode.hz, mode.damping));
        let own = own.as_ref().filter(|(hz, _)| (hz - mode.hz).abs() <= 1e-9 * mode.hz);
        let shape = match (own, &found) {
            (Some((_, shape)), _) => {
                ShownMode { pose: pose.to_vec(), hz: mode.hz, re: shape.clone(), im: vec![0.0; n] }
            }
            (None, Some((lambda, x))) => scaled(machine.as_ref(), pose, &rest, x, as_mode(*lambda).hz),
            (None, None) => ShownMode { pose: pose.to_vec(), hz: mode.hz, re: vec![0.0; n], im: vec![0.0; n] },
        };
        let stiffness = found.as_ref().map_or(Vec::new(), |(lambda, _)| shares(entry, config, pose, &kinds, *lambda));
        ModeView {
            hz: mode.hz,
            damping: mode.damping,
            linear: found.map(|(lambda, _)| as_mode(lambda)),
            from_machine: own.is_some(),
            shape,
            stiffness,
        }
    });
    Ok(views.collect())
}

/// The shape `x` of the linearised model as a `ShownMode`, scaled by how far it moves the tool (see `mode_views`).
fn scaled(machine: &dyn Machine, pose: &[f64], rest: &[f64], x: &[C], hz: f64) -> ShownMode {
    let axes = machine.axes().len();
    let tool = |part: usize| -> Vec<f64> {
        let state: Vec<f64> = rest.iter().zip(x).map(|(r, v)| r + 1e-6 * v[part]).collect();
        let (mut pos, mut vel) = (vec![0.0; axes], vec![0.0; axes]);
        machine.tool_state(&state, &mut pos, &mut vel);
        pos
    };
    let (at_rest, re, im) = (
        {
            let (mut pos, mut vel) = (vec![0.0; axes], vec![0.0; axes]);
            machine.tool_state(rest, &mut pos, &mut vel);
            pos
        },
        tool(0),
        tool(1),
    );
    let moved = (0..axes).map(|i| (re[i] - at_rest[i]).hypot(im[i] - at_rest[i]) / 1e-6).fold(0.0, f64::max);
    let scale = if moved > 1e-9 { 1.0 / moved } else { 1.0 };
    ShownMode {
        pose: pose.to_vec(),
        hz,
        re: x.iter().map(|v| v[0] * scale).collect(),
        im: x.iter().map(|v| v[1] * scale).collect(),
    }
}

/// The config's stiffness values by kind (`stiffness_kind`): positive numbers only.
fn stiffness_leaves(config: &Config) -> Vec<(&'static str, Vec<(String, f64)>)> {
    let mut kinds: Vec<(&'static str, Vec<(String, f64)>)> = Vec::new();
    for (path, value) in config.leaves() {
        let (Some(kind), Some(k)) = (stiffness_kind(&path), value.as_float().filter(|k| *k > 0.0)) else { continue };
        match kinds.iter_mut().find(|(have, _)| *have == kind) {
            Some((_, leaves)) => leaves.push((path, k)),
            None => kinds.push((kind, vec![(path, k)])),
        }
    }
    kinds
}

/// The share of the strain energy of the linearised mode `lambda` in each kind of stiffness (see `ModeView`).
fn shares(
    entry: &MachineEntry,
    config: &Config,
    pose: &[f64],
    kinds: &[(&'static str, Vec<(String, f64)>)],
    lambda: C,
) -> Vec<(&'static str, f64)> {
    const STEP: f64 = 1.1;
    let mut out: Vec<(&'static str, f64)> = kinds
        .iter()
        .filter_map(|(kind, leaves)| {
            let mut stiffer = config.clone();
            for (path, k) in leaves {
                crate::analysis::set_bounded(&mut stiffer, path, k * STEP).ok()?;
            }
            let machine = build(entry, &stiffer).ok()?;
            let (_, a) = linearised(machine.as_ref(), pose);
            let (moved, _) = eigen_near(&a, machine.state_len(), lambda)?;
            Some((*kind, 2.0 * (abs(moved) / abs(lambda)).ln() / STEP.ln()))
        })
        .collect();
    out.sort_by(|a, b| b.1.total_cmp(&a.1));
    out
}

/// The largest acceleration the motors can give the tool at `pose` in its worst direction, m/s^2: with T = A a + g the
/// rigid model's motor torques (`Machine::feedforward_torque`, linear in the acceleration a of the axes in metres) and
/// `peak` each motor's peak torque, motor k holds |g_k| + |A_k . d| a <= peak_k for a direction d; its worst is d along
/// its row A_k, so a = min over k of (peak_k - |g_k|) / |A_k|. 0 where a motor cannot even hold the tool there.
pub fn worst_acceleration(machine: &dyn Machine, pose: &[f64], peak: &[f64]) -> f64 {
    let (axes, motors) = (machine.axes(), machine.motors().len());
    let mut hold = vec![0.0; motors];
    machine.feedforward_torque(pose, &vec![0.0; axes.len()], &mut hold);
    let mut rows = vec![0.0; motors];
    let mut torque = vec![0.0; motors];
    for (j, _) in axes.iter().enumerate().filter(|(_, axis)| axis.unit == "m") {
        let mut acc = vec![0.0; axes.len()];
        acc[j] = 1.0;
        machine.feedforward_torque(pose, &acc, &mut torque);
        (0..motors).for_each(|k| rows[k] += (torque[k] - hold[k]).powi(2));
    }
    (0..motors)
        .filter(|&k| rows[k] > 0.0)
        .map(|k| (peak.get(k).copied().unwrap_or(0.0) - hold[k].abs()) / rows[k].sqrt())
        .fold(f64::INFINITY, f64::min)
        .max(0.0)
}

/// A horizontal slice through what the machine reaches, at height `z` of its third axis (`workspace_slice`).
#[derive(Debug, Clone, PartialEq)]
pub struct Slice {
    /// The span of the first two axes the grid covers, (low, high) each: their ranges and half of their width beyond
    /// each end, so that the reach shows where it goes past the travel range.
    pub x: (f64, f64),
    pub y: (f64, f64),
    /// Nodes per side: node (i, j) is at x.0 + i (x.1 - x.0) / (n - 1), and likewise in y with j.
    pub n: usize,
    /// Per node, row after row (j outer): the condition number (`Machine::condition`; `INFINITY` out of reach of the
    /// kinematics), whether the machine goes there (`Machine::unreachable` has nothing against it), and the largest
    /// acceleration in the worst direction (`worst_acceleration`, with each motor's `peak` torque; NaN out of reach).
    pub condition: Vec<f64>,
    pub goes: Vec<bool>,
    pub acceleration: Vec<f64>,
}

/// The slice at `z` with `n` nodes a side (see `Slice`); the other axes beyond the third at 0. `None` for a machine with
/// fewer than three axes or no condition to tell (one whose motors move the tool as its axes).
pub fn workspace_slice(machine: &dyn Machine, z: f64, n: usize, peak: &[f64]) -> Option<Slice> {
    let axes = machine.axes();
    axes.get(2)?;
    machine.condition(&[0.0, 0.0, z])?;
    let span = |a: &sim_core::AxisInfo| (a.min - (a.max - a.min) / 2.0, a.max + (a.max - a.min) / 2.0);
    let (x, y) = (span(&axes[0]), span(&axes[1]));
    let at = |(low, high): (f64, f64), i: usize| low + (high - low) * i as f64 / (n - 1) as f64;
    let mut pose = vec![0.0; axes.len()];
    let mut slice = Slice { x, y, n, condition: Vec::new(), goes: Vec::new(), acceleration: Vec::new() };
    for j in 0..n {
        for i in 0..n {
            (pose[0], pose[1], pose[2]) = (at(x, i), at(y, j), z);
            let condition = machine.condition(&pose).unwrap_or(f64::INFINITY);
            slice.condition.push(condition);
            slice.goes.push(machine.unreachable(&pose, 0.0).is_none());
            slice.acceleration.push(if condition.is_finite() {
                worst_acceleration(machine, &pose, peak)
            } else {
                f64::NAN
            });
        }
    }
    Some(slice)
}

/// The lowest and the highest value at each sample over `traces`, sample by sample from their starts; where a trace is
/// shorter than another, its end counts no further.
pub fn envelope<'a>(traces: impl IntoIterator<Item = &'a [f64]>) -> (Vec<f64>, Vec<f64>) {
    let (mut low, mut high): (Vec<f64>, Vec<f64>) = (Vec::new(), Vec::new());
    for trace in traces {
        for (i, &v) in trace.iter().enumerate() {
            if i < low.len() {
                (low[i], high[i]) = (low[i].min(v), high[i].max(v));
            } else {
                low.push(v);
                high.push(v);
            }
        }
    }
    (low, high)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two runs by hand: [1, 3, 2] and [2, 1, 2, 5] span [1, 1, 2, 5] to [2, 3, 2, 5].
    #[test]
    fn the_envelope_is_the_lowest_and_highest_of_the_runs_at_each_sample() {
        let (a, b) = ([1.0, 3.0, 2.0], [2.0, 1.0, 2.0, 5.0]);
        assert_eq!(envelope([&a[..], &b[..]]), (vec![1.0, 1.0, 2.0, 5.0], vec![2.0, 3.0, 2.0, 5.0]));
    }

    /// x'' = -w^2 x - 2 z w x' with w = 2 pi 10 and z = 0.1, as a first-order system: lambda = w (-z + i sqrt(1 - z^2)),
    /// the shape (1, lambda).
    #[test]
    fn inverse_iteration_finds_the_damped_oscillators_eigenvalue() {
        let (w, z) = (std::f64::consts::TAU * 10.0, 0.1);
        let a = [0.0, 1.0, -w * w, -2.0 * z * w];
        let (lambda, x) = eigen_near(&a, 2, [0.0, w]).unwrap();
        let exact = [-z * w, w * (1.0 - z * z).sqrt()];
        assert!(abs(sub(lambda, exact)) < 1e-9 * w, "{lambda:?} {exact:?}");
        assert!(abs(sub(div(x[1], x[0]), exact)) < 1e-9 * w);
        let mode = as_mode(lambda);
        assert!((mode.hz - 10.0).abs() < 1e-9 && (mode.damping - z).abs() < 1e-9);
    }

    #[test]
    fn stiffness_values_are_sorted_by_kind() {
        assert_eq!(stiffness_kind("belt.stiffness_ea_n"), Some("belt"));
        assert_eq!(stiffness_kind("gearbox.4.stiffness_nm_rad"), Some("gearbox"));
        assert_eq!(stiffness_kind("friction_x.bristle_stiffness_n_m"), Some("guide"));
        assert_eq!(stiffness_kind("link_bending.youngs_modulus_pa"), Some("link"));
        assert_eq!(stiffness_kind("base.tilt_stiffness_x_nm_rad"), Some("base"));
        for none in
            ["hard_stop.stiffness_n_m", "joint.stop_stiffness_nm_rad", "gearbox.stiffness_t1_nm", "mass.carriage_kg"]
        {
            assert_eq!(stiffness_kind(none), None, "{none}");
        }
    }
}
