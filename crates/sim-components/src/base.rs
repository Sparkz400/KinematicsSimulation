//! A compliant base under any machine (effects.base_compliance): the frame or stand the whole machine stands on, a
//! rigid body on springs in up to six directions, translation along x, y, z and a small tilt about x, y, z through
//! `base.pivot_m`, in the frame of the tool pose (`FrameMomentum`). Each direction with a stiffness above zero is a
//! mass (`base.mass_kg`, or `base.inertia_<x|y|z>_kgm2` for a tilt), a spring and a damper of its own:
//!
//!   M d'' + 2 zeta sqrt(k M) d' + k d = L
//!
//! where L is what the machine puts on the base: the rate of change of the momentum of what moves on it, taken back
//! (the motors push on their stators as they push the load), the weight of what moves and an outside force on it (a
//! process force from a workpiece on the floor), less those at rest at the start pose, at which the base was set up:
//!
//!   F = -dp/dt + m g + F_ext,   T = -dL/dt + c x g + r_ext x F_ext   (about the origin; about the pivot T - P x F)
//!
//! dp/dt is the momentum's change along the machine's own state rates, by central differences over +-1 us. The tool
//! rides on the base and the workpiece stands on the floor, so the base's move at the tool (`Machine::base_shift`) is
//! the tool's too; the motors' angles are measured on the base and do not see it.
//!
//! M is everything that sways with the base, the machine on it included: the base's natural frequency is
//! sqrt(k / M) / 2 pi as configured.
//!
//! Without effects.base_reaction the coupling is one-way: the machine's own dynamics do not feel the base's
//! acceleration (its motors neither see the sway in their following error nor hold against it); exact where the servo
//! holds the axes stiffly to the base or the base is much heavier than what moves.
//!
//! With effects.base_reaction it is two-way. The machine's momentum is linear in its speeds v (the states that move a
//! mass: `Reaction::speeds`), P = A(q) v (linear, then angular about the pivot), and A is the map that turns the
//! base's acceleration a (translation, then tilt) into generalised forces on them, -A^T a, as it turns gravity into
//! A_lin^T g. Its mass matrix over the speeds is the Hessian of its kinetic energy, M = d2E / dv2 (`Machine::energy`
//! at the state with v set to unit speeds, by polarisation: exact, E being quadratic in v). With the machine's own
//! accelerations v0'' (its derivatives, which do not know the base), the two are solved together each evaluation:
//!
//!   v'' = v0'' + M^-1 A^T (w - a),   (M_B - A M^-1 A^T) a = L - k d - c d' - A M^-1 A^T w
//!
//! L as above (its dp/dt along the machine's own rates), w = (g x theta, 0) the change of gravity in the machine's frame
//! by the base's tilt theta. The base's kinetic energy 1/2 M_B d'^2 + d' . P, its spring's 1/2 k d^2 and the weight's
//! work on its tilt, -theta . (c x g - c0 x g), are in `Machine::energy` and in the signals `energy.*`; its damping c d'^2
//! is the signal `power.base`. The process force's work on the moving base, the outside force's wrench (about the pivot,
//! less its value at the start pose) times the base's rates, is taken off that row (it is energy put in, so a negative
//! loss), which closes the budget while a process force works on a swaying base.
//!
//! The gyroscopic and centrifugal terms of the base's tilt rates on the machine (second order in the small tilt), and
//! the change of the base's inertia with the machine's pose, are left out. A gantry crate's and the machine files' own frame mode (effects.frame_compliance)
//! is the coupled one along one axis: the portal's posts swaying under its beam; the base is what the whole machine
//! stands on, in series with it.

use sim_core::{
    AxisInfo, BASE_AXES, BeltSection, Config, ConfigError, ENERGY_PARTS, FrameMomentum, FrictionContact, LOSS_PREFIX,
    Machine, Mode, MotorInfo, RatingCheck, Scene, SignalInfo, ThermalPart,
};

/// The base's directions: translation along x, y, z, tilt about x, y, z.
pub const DOFS: [&str; 6] = ["x", "y", "z", "rx", "ry", "rz"];

/// The step of the central difference that takes the momentum's rate of change, s.
const H: f64 = 1e-6;

#[derive(Debug, Clone, Copy)]
struct Dof {
    /// Which of `DOFS`.
    index: usize,
    mass: f64,
    stiffness: f64,
    damping: f64,
}

/// A machine on a compliant base. Its state is the machine's, then the base's deflection in each compliant direction,
/// then their rates.
pub struct Based {
    inner: Box<dyn Machine>,
    dofs: Vec<Dof>,
    pivot: [f64; 3],
    /// The static load at the start pose (about the origin), which the base was set up under.
    reference: [f64; 6],
    /// The outside force's wrench at the start pose, about the origin (the part of `reference` that is not weight).
    external_reference: [f64; 6],
    /// How many of the machine's signals are its base signals.
    inner_base_signals: usize,
    /// effects.base_reaction.
    reaction: Option<Reaction>,
}

/// The two-way coupling (effects.base_reaction, see the module).
struct Reaction {
    /// The machine's states that are its speeds: those its momentum moves with, and whose change moves something but
    /// speeds (a state held at zero, as a frame mode switched off, is none).
    speeds: Vec<usize>,
    /// Gravity in the machine's frame, and its weight as force and torque about the origin at the start pose.
    gravity: [f64; 3],
    weight: [f64; 6],
    /// Where `energy.<ENERGY_PARTS>` sit among the machine's effect signals.
    parts: Option<[usize; 3]>,
}

impl Reaction {
    /// Found at the machine's state at rest `state` under `torque`, its momentum there `at_rest`.
    fn new(m: &dyn Machine, state: &[f64], torque: &[f64], pivot: [f64; 3], at_rest: &FrameMomentum) -> Self {
        let momentum_at = |s: &[f64]| m.frame_momentum(s).map(|f| momentum(&f, pivot));
        let moved = |j: usize| {
            let mut s = state.to_vec();
            s[j] += 1e-3;
            s
        };
        // At rest the momentum is zero whatever the positions: only a speed moves it.
        let before = momentum_at(state);
        let moving: Vec<usize> = (0..state.len())
            .filter(|&j| {
                momentum_at(&moved(j))
                    .zip(before)
                    .is_some_and(|(a, b)| a.iter().zip(&b).any(|(a, b)| (a - b).is_finite() && a != b))
            })
            .collect();
        let mut rates = vec![0.0; state.len()];
        m.derivatives(state, torque, &mut rates);
        let speeds = moving
            .iter()
            .copied()
            .filter(|&j| {
                let mut out = vec![0.0; state.len()];
                m.derivatives(&moved(j), torque, &mut out);
                (0..state.len()).any(|i| !moving.contains(&i) && out[i] != rates[i])
            })
            .collect();
        let infos = m.signal_infos();
        let mut own = Vec::new();
        m.base_signals(state, torque, &mut own);
        let parts =
            ENERGY_PARTS.map(|part| infos.iter().position(|s| s.id == format!("energy.{part}")).map(|i| i - own.len()));
        Self {
            speeds,
            gravity: at_rest.gravity,
            weight: weight(at_rest),
            parts: parts.iter().all(Option::is_some).then(|| parts.map(Option::unwrap)),
        }
    }
}

/// Momentum and angular momentum about `pivot`.
fn momentum(f: &FrameMomentum, pivot: [f64; 3]) -> [f64; 6] {
    let about = cross(pivot, f.linear);
    let l = f.angular;
    [f.linear[0], f.linear[1], f.linear[2], l[0] - about[0], l[1] - about[1], l[2] - about[2]]
}

/// The weight of what moves, as force and torque about the origin.
fn weight(f: &FrameMomentum) -> [f64; 6] {
    let t = cross(f.first_moment, f.gravity);
    let g = f.gravity.map(|g| f.mass * g);
    [g[0], g[1], g[2], t[0], t[1], t[2]]
}

/// The machine on a compliant base with effects.base_compliance, as it is without. A config without the switch (one
/// written before 14 November 2026) has a rigid base.
pub fn mount(cfg: &Config, machine: Box<dyn Machine>) -> Result<Box<dyn Machine>, ConfigError> {
    if cfg.get("effects.base_compliance").is_err() || !cfg.bool("effects.base_compliance")? {
        return Ok(machine);
    }
    let mass = cfg.positive("base.mass_kg")?;
    let mut dofs = Vec::new();
    for (index, id) in DOFS.iter().enumerate() {
        let axis = ["x", "y", "z"][index % 3];
        let (m, k) = if index < 3 {
            (mass, cfg.non_negative(&format!("base.stiffness_{axis}_n_m"))?)
        } else {
            (
                cfg.positive(&format!("base.inertia_{axis}_kgm2"))?,
                cfg.non_negative(&format!("base.tilt_stiffness_{axis}_nm_rad"))?,
            )
        };
        let zeta = cfg.non_negative(&format!("base.damping_ratio_{id}"))?;
        if k > 0.0 {
            dofs.push(Dof { index, mass: m, stiffness: k, damping: 2.0 * zeta * (k * m).sqrt() });
        }
    }
    let pivot = match cfg.f64s("base.pivot_m")?[..] {
        [x, y, z] => [x, y, z],
        _ => return Err(ConfigError::Invalid("base.pivot_m".into())),
    };
    let pose = crate::sim::start_pose(cfg, machine.as_ref())?;
    let (mut state, mut torque) = (Vec::new(), vec![0.0; machine.motors().len()]);
    machine.equilibrium(&pose, &mut state, &mut torque);
    let at_rest =
        machine.frame_momentum(&state).ok_or_else(|| ConfigError::Invalid("effects.base_compliance".into()))?;
    let mut signals = Vec::new();
    machine.base_signals(&state, &torque, &mut signals);
    let reaction = (cfg.get("effects.base_reaction").is_ok() && cfg.bool("effects.base_reaction")?)
        .then(|| Reaction::new(machine.as_ref(), &state, &torque, pivot, &at_rest));
    let based = Based {
        reference: static_load(&at_rest),
        external_reference: external_wrench(&at_rest),
        inner_base_signals: signals.len(),
        inner: machine,
        dofs,
        pivot,
        reaction,
    };
    // What sways must be heavier than what the machine moves on it along each direction: M_B - A M^-1 A^T > 0.
    if let Some(r) = &based.reaction {
        let (_, g) = based.coupling(r, &state);
        if let Some(d) = based.dofs.iter().find(|d| !(d.mass - g[d.index][d.index] > 0.0)) {
            let axis = ["x", "y", "z"][d.index % 3];
            return Err(ConfigError::Invalid(if d.index < 3 {
                "base.mass_kg".into()
            } else {
                format!("base.inertia_{axis}_kgm2")
            }));
        }
    }
    Ok(Box::new(based))
}

/// Adds a point mass `m` at `r` moving at `v` to a machine's `FrameMomentum`.
pub fn add_point(f: &mut FrameMomentum, m: f64, r: [f64; 3], v: [f64; 3]) {
    let l = cross(r, v);
    for k in 0..3 {
        f.linear[k] += m * v[k];
        f.angular[k] += m * l[k];
        f.first_moment[k] += m * r[k];
    }
    f.mass += m;
}

/// Adds a body that turns about a fixed joint at `joint`, at `speed`: its centre of mass lies along `dir` (a unit
/// vector turning with it, `ddir` its derivative by the angle, so that the axis is dir x ddir), `moment` (kg m) its
/// mass times that distance and `inertia` its inertia about the axis through the joint. Its mass at the joint does not
/// change the load and is left out. Linear momentum moment speed ddir; angular inertia speed axis plus joint x that.
pub fn add_turning(
    f: &mut FrameMomentum,
    joint: [f64; 3],
    dir: [f64; 3],
    ddir: [f64; 3],
    speed: f64,
    inertia: f64,
    moment: f64,
) {
    let (axis, p) = (cross(dir, ddir), ddir.map(|d| moment * speed * d));
    let about = cross(joint, p);
    for k in 0..3 {
        f.linear[k] += p[k];
        f.angular[k] += inertia * speed * axis[k] + about[k];
        f.first_moment[k] += moment * dir[k];
    }
}

/// Adds a rotor (or any wheel whose centre stays put) spinning at `speed` with `inertia` about the unit `axis`.
pub fn add_spin(f: &mut FrameMomentum, axis: [f64; 3], inertia: f64, speed: f64) {
    (0..3).for_each(|k| f.angular[k] += inertia * speed * axis[k]);
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// The outside force alone, as force and torque about the origin.
fn external_wrench(m: &FrameMomentum) -> [f64; 6] {
    let t = cross(m.external_at, m.external);
    [m.external[0], m.external[1], m.external[2], t[0], t[1], t[2]]
}

/// The weight and the outside force of what moves, as force and torque about the origin.
fn static_load(m: &FrameMomentum) -> [f64; 6] {
    let force: [f64; 3] = std::array::from_fn(|k| m.mass * m.gravity[k] + m.external[k]);
    let (weight, outside) = (cross(m.first_moment, m.gravity), cross(m.external_at, m.external));
    [force[0], force[1], force[2], weight[0] + outside[0], weight[1] + outside[1], weight[2] + outside[2]]
}

impl Based {
    fn n(&self) -> usize {
        self.inner.state_len()
    }

    /// The base's deflection (or rate, `rates`) in all six directions, zero where it is rigid.
    fn full(&self, s: &[f64], rates: bool) -> [f64; 6] {
        let at = self.n() + if rates { self.dofs.len() } else { 0 };
        let mut d = [0.0; 6];
        for (k, dof) in self.dofs.iter().enumerate() {
            d[dof.index] = s[at + k];
        }
        d
    }

    /// What the machine at state `s` (its own), whose state changes at `rate`, puts on the base: force and torque
    /// about the pivot, less what it put on it at rest at the start pose.
    pub fn load(&self, s: &[f64], rate: &[f64]) -> [f64; 6] {
        let at = |h: f64| {
            let moved: Vec<f64> = s.iter().zip(rate).map(|(x, r)| x + h * r).collect();
            self.inner.frame_momentum(&moved).unwrap_or_default()
        };
        let (before, now, after) = (at(-H), at(0.0), at(H));
        let rest = static_load(&now);
        let mut load: [f64; 6] = std::array::from_fn(|j| {
            let (a, b) =
                if j < 3 { (after.linear[j], before.linear[j]) } else { (after.angular[j - 3], before.angular[j - 3]) };
            rest[j] - self.reference[j] - (a - b) / (2.0 * H)
        });
        let about_pivot = cross(self.pivot, [load[0], load[1], load[2]]);
        (0..3).for_each(|k| load[3 + k] -= about_pivot[k]);
        load
    }

    /// At the machine's state `s` (its own): X = M^-1 A^T (per speed, the six directions) and A M^-1 A^T (see the
    /// module), from its momentum and its energy at unit speeds.
    fn coupling(&self, r: &Reaction, s: &[f64]) -> (Vec<[f64; 6]>, [[f64; 6]; 6]) {
        let n = r.speeds.len();
        let mut at = s.to_vec();
        r.speeds.iter().for_each(|&i| at[i] = 0.0);
        let still = self.inner.energy(&at);
        let (mut a, mut single) = (Vec::with_capacity(n), Vec::with_capacity(n));
        for &i in &r.speeds {
            at[i] = 1.0;
            a.push(self.inner.frame_momentum(&at).map_or([f64::NAN; 6], |f| momentum(&f, self.pivot)));
            single.push(self.inner.energy(&at) - still);
            at[i] = 0.0;
        }
        let mut mass = vec![0.0; n * n];
        for k in 0..n {
            mass[k * n + k] = 2.0 * single[k];
            for l in 0..k {
                (at[r.speeds[k]], at[r.speeds[l]]) = (1.0, 1.0);
                let both = self.inner.energy(&at) - still - single[k] - single[l];
                (at[r.speeds[k]], at[r.speeds[l]]) = (0.0, 0.0);
                (mass[k * n + l], mass[l * n + k]) = (both, both);
            }
        }
        let mut x = vec![[0.0; 6]; n];
        for j in 0..6 {
            let rhs: Vec<f64> = a.iter().map(|c| c[j]).collect();
            let column = crate::chain::solve(&mass, &rhs).unwrap_or_else(|| vec![f64::NAN; n]);
            x.iter_mut().zip(column).for_each(|(x, c)| x[j] = c);
        }
        let g = std::array::from_fn(|i| std::array::from_fn(|j| (0..n).map(|k| a[k][i] * x[k][j]).sum()));
        (x, g)
    }

    /// The base's share of the energy at state `s` with effects.base_reaction, as [kinetic, potential, elastic]: its
    /// sway with the machine's momentum, the weight's work on its tilt, its springs.
    fn reaction_energy(&self, r: &Reaction, s: &[f64]) -> [f64; 3] {
        let (n, m) = (self.n(), self.dofs.len());
        let f = self.inner.frame_momentum(&s[..n]).unwrap_or_default();
        let (p, rate) = (momentum(&f, self.pivot), self.full(s, true));
        let about = |w: [f64; 6]| {
            let t = cross(self.pivot, [w[0], w[1], w[2]]);
            [w[0], w[1], w[2], w[3] - t[0], w[4] - t[1], w[5] - t[2]]
        };
        let (now, then) = (about(weight(&f)), about(r.weight));
        let mut parts = [(0..6).map(|j| rate[j] * p[j]).sum(), 0.0, 0.0];
        for (k, d) in self.dofs.iter().enumerate() {
            let (x, v) = (s[n + k], s[n + m + k]);
            parts[0] += 0.5 * d.mass * v * v;
            parts[1] -= x * (now[d.index] - then[d.index]);
            parts[2] += 0.5 * d.stiffness * x * x;
        }
        parts
    }

    /// The base's row of the loss budget, W: its damping loss less the work the process force does on it as it moves.
    fn damping_power(&self, s: &[f64]) -> f64 {
        let at = self.n() + self.dofs.len();
        let damping: f64 = self.dofs.iter().enumerate().map(|(k, d)| d.damping * s[at + k] * s[at + k]).sum();
        let f = self.inner.frame_momentum(&s[..self.n()]).unwrap_or_default();
        let now = external_wrench(&f);
        let mut w: [f64; 6] = std::array::from_fn(|j| now[j] - self.external_reference[j]);
        let about = cross(self.pivot, [w[0], w[1], w[2]]);
        (0..3).for_each(|k| w[3 + k] -= about[k]);
        let rate = self.full(s, true);
        damping - (0..6).map(|j| rate[j] * w[j]).sum::<f64>()
    }

    /// The base's deflection at rest under the machine at state `s` (its own).
    fn rest(&self, s: &[f64]) -> Vec<f64> {
        let load = self.load(s, &vec![0.0; s.len()]);
        self.dofs.iter().map(|d| load[d.index] / d.stiffness).collect()
    }

    /// The tool pose's move by the base's deflection `d` at the pose `pose` the machine alone has.
    fn tool_shift(&self, pose: &[f64], d: &[f64; 6]) -> Vec<f64> {
        let mut out = vec![0.0; pose.len()];
        self.inner.base_shift(pose, self.pivot, d, &mut out);
        out
    }

    /// The base's modes: frequency, damping, which direction.
    fn base_modes(&self) -> impl Iterator<Item = (Mode, usize)> + '_ {
        self.dofs.iter().enumerate().map(|(k, d)| {
            let w = (d.stiffness / d.mass).sqrt();
            (Mode { hz: w / std::f64::consts::TAU, damping: d.damping / (2.0 * d.mass * w) }, k)
        })
    }

    /// Where the base's move (`scale` times) takes a point `p` of the scene: by the base's translation plus its tilt
    /// about the pivot at the point's own place, p + t + theta x (p - pivot). The scene's axes are those of the
    /// machine's frame up to the map the machine's `scene_point` makes (a permutation or a reflection): it is read
    /// off from the scene move of a unit translation along each axis, and the tilt, a pseudovector, goes through it
    /// with the sign of its determinant. The pivot in the scene lies where the tool's point lies, less its distance to
    /// the pivot in the machine's frame. (A direction the scene point does not show is taken along the scene's own axis.)
    fn scene_mover(&self, s: &[f64], scale: f64) -> Option<impl Fn([f64; 3]) -> [f64; 3]> {
        let n = self.inner.axes().len();
        let (mut pose, mut vel) = (vec![0.0; n], vec![0.0; n]);
        self.inner.tool_state(&s[..self.n()], &mut pose, &mut vel);
        let from = self.inner.scene_point(&pose)?;
        // A millimetre along each axis, read back per metre.
        const STEP: f64 = 1e-3;
        let mut columns = [[0.0; 3]; 3];
        for (k, column) in columns.iter_mut().enumerate() {
            let mut unit = [0.0; 6];
            unit[k] = STEP;
            let moved: Vec<f64> = pose.iter().zip(self.tool_shift(&pose, &unit)).map(|(p, d)| p + d).collect();
            let to = self.inner.scene_point(&moved)?;
            *column = std::array::from_fn(|i| (to[i] - from[i]) / STEP);
        }
        // A direction the scene point does not show (a planar machine has no z axis): the scene's axis furthest from
        // the directions it shows.
        for k in 0..3 {
            if columns[k].iter().all(|c| c.abs() < 1e-9) {
                let shown = |i: usize| columns.iter().map(|c| c[i].abs()).sum::<f64>();
                let free = (0..3).min_by(|&a, &b| shown(a).total_cmp(&shown(b))).unwrap_or(k);
                columns[k] = std::array::from_fn(|i| if i == free { 1.0 } else { 0.0 });
            }
        }
        let map = |v: [f64; 3]| -> [f64; 3] { std::array::from_fn(|i| (0..3).map(|k| columns[k][i] * v[k]).sum()) };
        let det = {
            let c = columns;
            let x = cross(c[0], c[1]);
            (0..3).map(|i| x[i] * c[2][i]).sum::<f64>().signum()
        };
        let d = self.full(s, false);
        let t = map([d[0], d[1], d[2]]).map(|v| scale * v);
        let theta = map([d[3], d[4], d[5]]).map(|v| scale * v * if det < 0.0 { -1.0 } else { 1.0 });
        let axes = self.inner.axes();
        let found = BASE_AXES[0].map(|id| axes.iter().position(|a| a.id == id));
        let pivot: [f64; 3] = if found.iter().any(Option::is_some) {
            let at_tool: [f64; 3] = std::array::from_fn(|k| found[k].map_or(0.0, |i| pose[i]) - self.pivot[k]);
            let away = map(at_tool);
            std::array::from_fn(|k| from[k] - away[k])
        } else {
            // Tool axes that are joints tell nothing of where the tool is in the frame: the scene's origin is the
            // frame's (as the axes are where the scene shows no translation).
            map(self.pivot)
        };
        Some(move |p: [f64; 3]| {
            let turned = cross(theta, std::array::from_fn(|k| p[k] - pivot[k]));
            std::array::from_fn(|k| p[k] + t[k] + turned[k])
        })
    }
}

impl Machine for Based {
    fn frame_momentum(&self, state: &[f64]) -> Option<FrameMomentum> {
        self.inner.frame_momentum(&state[..self.n()])
    }
    fn base_shift(&self, tool_pose: &[f64], pivot: [f64; 3], shift: &[f64; 6], out: &mut [f64]) {
        self.inner.base_shift(tool_pose, pivot, shift, out)
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        self.inner.rating_checks()
    }
    fn axes(&self) -> &[AxisInfo] {
        self.inner.axes()
    }
    fn motors(&self) -> &[MotorInfo] {
        self.inner.motors()
    }
    fn motor_linear(&self, motor: usize) -> bool {
        self.inner.motor_linear(motor)
    }
    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        self.inner.inverse(tool, motors)
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        self.inner.forward(motors, tool)
    }
    fn state_len(&self) -> usize {
        self.n() + 2 * self.dofs.len()
    }
    /// The tool, not the machine on a rigid base, at `tool`: the machine stands off by the base's deflection under it.
    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let mut pose = tool.to_vec();
        for _ in 0..2 {
            self.inner.equilibrium(&pose, state, torque);
            let mut d = [0.0; 6];
            for (k, value) in self.rest(state).into_iter().enumerate() {
                d[self.dofs[k].index] = value;
            }
            let shift = self.tool_shift(&pose, &d);
            pose = tool.iter().zip(&shift).map(|(t, s)| t - s).collect();
        }
        self.inner.equilibrium(&pose, state, torque);
        let rest = self.rest(state);
        state.extend(rest);
        state.extend(vec![0.0; self.dofs.len()]);
    }
    fn derivatives(&self, state: &[f64], torque: &[f64], out: &mut [f64]) {
        let n = self.n();
        self.inner.derivatives(&state[..n], torque, &mut out[..n]);
        let load = self.load(&state[..n], &out[..n]);
        let m = self.dofs.len();
        let Some(r) = &self.reaction else {
            for (k, d) in self.dofs.iter().enumerate() {
                let (x, v) = (state[n + k], state[n + m + k]);
                out[n + k] = v;
                out[n + m + k] = (load[d.index] - d.stiffness * x - d.damping * v) / d.mass;
            }
            return;
        };
        // Base and machine together (see the module): (M_B - G) a = L - k d - c d' - G w, then v'' += X (w - a).
        let (x, g) = self.coupling(r, &state[..n]);
        let d = self.full(state, false);
        let tilt = cross(r.gravity, [d[3], d[4], d[5]]);
        let w = [tilt[0], tilt[1], tilt[2], 0.0, 0.0, 0.0];
        let (mut matrix, mut rhs) = (vec![0.0; m * m], vec![0.0; m]);
        for (k, dk) in self.dofs.iter().enumerate() {
            let (i, x, v) = (dk.index, state[n + k], state[n + m + k]);
            rhs[k] = load[i] - dk.stiffness * x - dk.damping * v - (0..6).map(|j| g[i][j] * w[j]).sum::<f64>();
            for (l, dl) in self.dofs.iter().enumerate() {
                matrix[k * m + l] = if k == l { dk.mass } else { 0.0 } - g[i][dl.index];
            }
        }
        let acc = crate::chain::solve(&matrix, &rhs).unwrap_or_else(|| vec![f64::NAN; m]);
        let mut push = w;
        for (k, dk) in self.dofs.iter().enumerate() {
            push[dk.index] -= acc[k];
            out[n + k] = state[n + m + k];
            out[n + m + k] = acc[k];
        }
        for (k, &i) in r.speeds.iter().enumerate() {
            out[i] += (0..6).map(|j| x[k][j] * push[j]).sum::<f64>();
        }
    }
    fn tool_state(&self, state: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        self.inner.tool_state(&state[..self.n()], pos, vel);
        let shift = self.tool_shift(pos, &self.full(state, false));
        let rate = self.tool_shift(pos, &self.full(state, true));
        pos.iter_mut().zip(shift).for_each(|(p, d)| *p += d);
        vel.iter_mut().zip(rate).for_each(|(v, d)| *v += d);
    }
    /// The scale stands on the machine, so the base's deflection is not in it either.
    fn scale_state(&self, state: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        self.inner.scale_state(&state[..self.n()], pos, vel)
    }
    fn set_tool_velocity(&self, state: &mut [f64], vel: &[f64]) {
        let n = self.n();
        self.inner.set_tool_velocity(&mut state[..n], vel)
    }
    fn motor_state(&self, state: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        self.inner.motor_state(&state[..self.n()], pos, vel)
    }
    fn feedforward_torque(&self, tool_pos: &[f64], tool_acc: &[f64], out: &mut [f64]) {
        self.inner.feedforward_torque(tool_pos, tool_acc, out)
    }
    fn feedforward_torque_moving(&self, tool_pos: &[f64], tool_vel: &[f64], tool_acc: &[f64], out: &mut [f64]) {
        self.inner.feedforward_torque_moving(tool_pos, tool_vel, tool_acc, out)
    }
    fn rigid_motor_torque(&self, angle: &[f64], speed: &[f64], acc: &[f64], out: &mut [f64]) {
        self.inner.rigid_motor_torque(angle, speed, acc, out)
    }
    fn twist(&self, torque: &[f64], acc: &[f64], out: &mut [f64]) {
        self.inner.twist(torque, acc, out)
    }
    fn twist_at(&self, angle: &[f64], torque: &[f64], acc: &[f64], out: &mut [f64]) {
        self.inner.twist_at(angle, torque, acc, out)
    }
    fn rigid_split_torque(&self, angle: &[f64], speed: &[f64], acc: &[f64], rotor_acc: &[f64], out: &mut [f64]) {
        self.inner.rigid_split_torque(angle, speed, acc, rotor_acc, out)
    }
    fn rigid_tensions(&self, tool_pos: &[f64], tool_acc: &[f64], out: &mut Vec<f64>) {
        self.inner.rigid_tensions(tool_pos, tool_acc, out)
    }
    fn rigid_tensions_moving(&self, tool_pos: &[f64], tool_vel: &[f64], tool_acc: &[f64], out: &mut Vec<f64>) {
        self.inner.rigid_tensions_moving(tool_pos, tool_vel, tool_acc, out)
    }
    fn tension_band(&self) -> Option<[f64; 2]> {
        self.inner.tension_band()
    }
    fn rigid_guide_loads(&self, tool_pos: &[f64], tool_acc: &[f64], out: &mut Vec<[f64; 2]>) {
        self.inner.rigid_guide_loads(tool_pos, tool_acc, out)
    }
    fn steady_friction(&self, tool_pos: &[f64], tool_vel: &[f64], out: &mut [f64]) {
        self.inner.steady_friction(tool_pos, tool_vel, out)
    }
    fn rigid_friction(&self, tool_pos: &[f64], tool_dir: &[f64], out: &mut Vec<[f64; 2]>) {
        self.inner.rigid_friction(tool_pos, tool_dir, out)
    }
    fn rigid_friction_rise(&self, tool_pos: &[f64], tool_dir: &[f64], out: &mut Vec<Vec<[f64; 2]>>) {
        self.inner.rigid_friction_rise(tool_pos, tool_dir, out)
    }
    fn rigid_checks(&self, tool_pos: &[f64], tool_vel: &[f64], tool_acc: &[f64], out: &mut Vec<f64>) {
        self.inner.rigid_checks(tool_pos, tool_vel, tool_acc, out)
    }
    /// The machine's, plus the base's kinetic and elastic energy (its potential energy in gravity left out: one-way the
    /// machine's own energy is not conserved with it anyway). With effects.base_reaction the base's share as the module
    /// has it, which the two together conserve.
    fn energy(&self, state: &[f64]) -> f64 {
        if let Some(r) = &self.reaction {
            return self.inner.energy(&state[..self.n()]) + self.reaction_energy(r, state).iter().sum::<f64>();
        }
        let (n, m) = (self.n(), self.dofs.len());
        let base: f64 = self
            .dofs
            .iter()
            .enumerate()
            .map(|(k, d)| 0.5 * d.mass * state[n + m + k].powi(2) + 0.5 * d.stiffness * state[n + k].powi(2))
            .sum();
        self.inner.energy(&state[..n]) + base
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.inner.set_payload_attached(attached)
    }
    fn valves(&self) -> Vec<(String, f64)> {
        self.inner.valves()
    }
    fn set_valve(&mut self, valve: usize, command: f64) {
        self.inner.set_valve(valve, command)
    }
    fn end_sensors(&self, valve: usize) -> [bool; 2] {
        self.inner.end_sensors(valve)
    }
    fn links(&self) -> Vec<String> {
        self.inner.links()
    }
    fn set_link_change(&mut self, link: usize, factor: f64, moment: [f64; 3]) {
        self.inner.set_link_change(link, factor, moment)
    }
    fn set_dwelling(&mut self, dwelling: bool) {
        self.inner.set_dwelling(dwelling)
    }
    fn set_tool_force(&mut self, force: &[f64]) -> bool {
        self.inner.set_tool_force(force)
    }
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.inner.set_brake_engaged(engaged)
    }
    fn after_step(&mut self, state: &mut [f64], dt: f64) {
        let n = self.n();
        self.inner.after_step(&mut state[..n], dt)
    }
    fn memory(&self) -> Vec<f64> {
        self.inner.memory()
    }
    fn set_memory(&mut self, memory: &[f64]) {
        self.inner.set_memory(memory)
    }
    fn set_part_temperatures(&mut self, rises: &[f64]) {
        self.inner.set_part_temperatures(rises)
    }
    /// The lower of the machine's resonance and the base's lowest mode that moves the tool along its first axis.
    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        let base = self.mode_shape(tool_pose).map(|(hz, _)| hz);
        match (base, self.inner.resonance_hz(tool_pose)) {
            (Some(b), Some(i)) => Some(b.min(i)),
            (b, i) => b.or(i),
        }
    }
    fn mode_shape(&self, tool_pose: &[f64]) -> Option<(f64, Vec<f64>)> {
        let len = self.state_len();
        let inner = self.inner.mode_shape(tool_pose).map(|(hz, mut shape)| {
            shape.resize(len, 0.0);
            (hz, shape)
        });
        let base = self
            .base_modes()
            .filter_map(|(mode, k)| {
                let mut unit = [0.0; 6];
                unit[self.dofs[k].index] = 1.0;
                let along = self.tool_shift(tool_pose, &unit)[0];
                (along.abs() > 1e-12).then(|| {
                    let mut shape = vec![0.0; len];
                    shape[self.n() + k] = 1.0 / along;
                    (mode.hz, shape)
                })
            })
            .min_by(|a, b| a.0.total_cmp(&b.0));
        match (inner, base) {
            (Some(i), Some(b)) => Some(if b.0 < i.0 { b } else { i }),
            (i, b) => i.or(b),
        }
    }
    fn modes(&self, tool_pose: &[f64]) -> Vec<Mode> {
        let mut modes = self.inner.modes(tool_pose);
        modes.extend(self.base_modes().map(|(mode, _)| mode));
        modes.sort_by(|a, b| a.hz.total_cmp(&b.hz));
        modes
    }
    /// The machine's own with the base's, as `modes`.
    fn sway_modes(&self, tool_pose: &[f64]) -> Vec<Mode> {
        let mut modes = self.inner.sway_modes(tool_pose);
        modes.extend(self.base_modes().map(|(mode, _)| mode));
        modes.sort_by(|a, b| a.hz.total_cmp(&b.hz));
        modes
    }
    fn trains(&self, tool_pose: &[f64]) -> Vec<Option<sim_core::Train>> {
        self.inner.trains(tool_pose)
    }
    fn belt_wraps(&self, tool_pose: &[f64]) -> Vec<f64> {
        self.inner.belt_wraps(tool_pose)
    }
    fn unreachable(&self, tool_pose: &[f64], slack: f64) -> Option<&'static str> {
        self.inner.unreachable(tool_pose, slack)
    }
    fn condition(&self, tool_pose: &[f64]) -> Option<f64> {
        self.inner.condition(tool_pose)
    }
    fn setpoint_slack(&self) -> f64 {
        self.inner.setpoint_slack()
    }
    fn cam(&self) -> Option<sim_core::CamMaster> {
        self.inner.cam()
    }
    fn cam_at(&self, master: f64, out: &mut [[f64; 4]]) {
        self.inner.cam_at(master, out)
    }
    fn cam_curve(&self) -> Option<sim_core::CamCurve> {
        self.inner.cam_curve()
    }
    fn gantry_motors(&self) -> Option<[usize; 2]> {
        self.inner.gantry_motors()
    }
    /// The machine's base signals, the base's deflections (`base.<dof>`, m or rad), then the machine's effect signals.
    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut infos = self.inner.signal_infos();
        let base = self.dofs.iter().map(|d| {
            let id = DOFS[d.index];
            SignalInfo::new(format!("base.{id}"), "base", &format!("quantity-base-{id}"), ["m", "rad"][d.index / 3])
        });
        infos.splice(self.inner_base_signals..self.inner_base_signals, base);
        if self.reaction.is_some() {
            infos.push(SignalInfo::new(format!("{LOSS_PREFIX}base"), "base", "quantity-damping-power", "W"));
        }
        infos
    }
    fn base_signals(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let n = self.n();
        self.inner.base_signals(&state[..n], torque, out);
        out.extend_from_slice(&state[n..n + self.dofs.len()]);
    }
    /// The machine's, with effects.base_reaction the base's energy in its `energy.*` and its damping loss last.
    fn effect_signals(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let start = out.len();
        self.inner.effect_signals(&state[..self.n()], torque, out);
        let Some(r) = &self.reaction else { return };
        if let Some(parts) = r.parts {
            let shares = self.reaction_energy(r, state);
            parts.iter().zip(shares).for_each(|(&at, share)| out[start + at] += share);
        }
        out.push(self.damping_power(state));
    }
    fn loss_powers(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.inner.loss_powers(&state[..self.n()], torque, out);
        if self.reaction.is_some() {
            out.push(self.damping_power(state));
        }
    }
    /// The machine's scene, moved with the base times the exaggeration: every point (the parts, the ends of a belt's
    /// strips, labels, arrows) by the base's translation and its tilt at its own place (`scene_mover`).
    fn scene(&self, state: &[f64], out: &mut Scene) {
        self.inner.scene(&state[..self.n()], out);
        let Some(mover) = self.scene_mover(state, out.exaggeration) else { return };
        for part in &mut out.parts {
            part.pos = mover(part.pos);
            if let sim_core::Shape::Segment { to, .. } = &mut part.shape {
                *to = mover(*to);
            }
        }
        out.labels.iter_mut().for_each(|l| l.pos = mover(l.pos));
        out.arrows.iter_mut().for_each(|a| a.pos = mover(a.pos));
    }
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        self.inner.scene_point(tool_pose)
    }
    fn collision_bodies(&self, state: &[f64]) -> Vec<sim_core::Aabb> {
        let mut bodies = self.inner.collision_bodies(&state[..self.n()]);
        // (A box keeps its axes: only its centre goes with the tilt.)
        if let Some(mover) = self.scene_mover(state, 1.0) {
            bodies.iter_mut().for_each(|b| b.centre = mover(b.centre));
        }
        bodies
    }
    fn collision_capsules(&self, state: &[f64]) -> Vec<sim_core::Capsule> {
        let mut capsules = self.inner.collision_capsules(&state[..self.n()]);
        if let Some(mover) = self.scene_mover(state, 1.0) {
            for c in &mut capsules {
                (c.from, c.to) = (mover(c.from), mover(c.to));
            }
        }
        capsules
    }
    fn friction_contacts(&self) -> Vec<FrictionContact> {
        self.inner.friction_contacts()
    }
    fn friction_curve(&self, i: usize, speed: f64, load_scale: f64) -> [f64; 3] {
        self.inner.friction_curve(i, speed, load_scale)
    }
    fn friction_rotary(&self, i: usize) -> bool {
        self.inner.friction_rotary(i)
    }
    fn friction_speed_limit(&self, i: usize, motor_speed: &[f64]) -> Option<f64> {
        self.inner.friction_speed_limit(i, motor_speed)
    }
    fn belt_sections(&self) -> Vec<BeltSection> {
        self.inner.belt_sections()
    }
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        self.inner.thermal_parts()
    }
    fn thermal_values(&self, id: &str) -> Option<(f64, f64)> {
        self.inner.thermal_values(id)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::{Rk4, advance};
    use std::f64::consts::FRAC_PI_2;

    /// A point mass on a frictionless guide along x, pushed by its motor's force (torque = force), with a process force
    /// while it dwells, and a spring of `spring` N/m that holds it at x = 0.5: the smallest machine that stands on a
    /// base.
    struct Slider {
        mass: f64,
        process: f64,
        spring: f64,
        dwelling: bool,
        resonance: Option<f64>,
        /// The scene shows x the other way (a reflection).
        flipped: bool,
        axes: [AxisInfo; 1],
        motors: [MotorInfo; 1],
    }

    fn slider() -> Slider {
        Slider {
            mass: 4.0,
            process: 50.0,
            spring: 0.0,
            dwelling: false,
            resonance: None,
            flipped: false,
            axes: [AxisInfo {
                id: "x",
                key: "",
                unit: "m",
                min: 0.0,
                max: 1.0,
                max_vel: 1.0,
                max_acc: 10.0,
                max_jerk: 100.0,
            }],
            motors: [MotorInfo { id: "a", key: "" }],
        }
    }

    impl Machine for Slider {
        fn frame_momentum(&self, s: &[f64]) -> Option<FrameMomentum> {
            Some(FrameMomentum {
                linear: [self.mass * s[1], 0.0, 0.0],
                mass: self.mass,
                first_moment: [self.mass * s[0], 0.0, 0.0],
                external: [if self.dwelling { self.process } else { 0.0 }, 0.0, 0.0],
                external_at: [s[0], 0.0, 0.0],
                ..Default::default()
            })
        }
        fn axes(&self) -> &[AxisInfo] {
            &self.axes
        }
        fn motors(&self) -> &[MotorInfo] {
            &self.motors
        }
        fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
            motors[0] = tool[0];
        }
        fn forward(&self, motors: &[f64], tool: &mut [f64]) {
            tool[0] = motors[0];
        }
        fn state_len(&self) -> usize {
            2
        }
        fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
            *state = vec![tool[0], 0.0];
            torque[0] = if self.dwelling { -self.process } else { 0.0 };
        }
        fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
            let process = if self.dwelling { self.process } else { 0.0 };
            out[0] = s[1];
            out[1] = (torque[0] + process - self.spring * (s[0] - 0.5)) / self.mass;
        }
        fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
            (pos[0], vel[0]) = (s[0], s[1]);
        }
        fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
            s[1] = vel[0];
        }
        fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
            (pos[0], vel[0]) = (s[0], s[1]);
        }
        fn feedforward_torque(&self, _: &[f64], acc: &[f64], out: &mut [f64]) {
            out[0] = self.mass * acc[0];
        }
        fn energy(&self, s: &[f64]) -> f64 {
            0.5 * self.mass * s[1] * s[1] + 0.5 * self.spring * (s[0] - 0.5).powi(2)
        }
        fn set_payload_attached(&mut self, _: bool) {}
        fn set_dwelling(&mut self, dwelling: bool) {
            self.dwelling = dwelling;
        }
        fn resonance_hz(&self, _: &[f64]) -> Option<f64> {
            self.resonance
        }
        fn motor_linear(&self, _: usize) -> bool {
            true
        }
        fn signal_infos(&self) -> Vec<SignalInfo> {
            Vec::new()
        }
        fn base_signals(&self, _: &[f64], _: &[f64], _: &mut Vec<f64>) {}
        fn scene(&self, _: &[f64], _: &mut Scene) {}
        fn scene_point(&self, tool: &[f64]) -> Option<[f64; 3]> {
            Some([if self.flipped { -tool[0] } else { tool[0] }, 0.0, 0.0])
        }
    }

    /// An arm in the x-z plane turning about the y axis at the origin, its angle a from x towards z: a point mass `mass`
    /// at `length`, a rotor of `rotor` kg m^2 on its shaft (ratio 1), gravity `g` down z. Its energy parts are signals,
    /// so that the loss budget can be checked on it.
    struct Arm {
        mass: f64,
        length: f64,
        rotor: f64,
        g: f64,
        axes: [AxisInfo; 1],
        motors: [MotorInfo; 1],
    }

    fn arm(g: f64) -> Arm {
        // Its middle hanging down, where it swings stably under a constant torque (the standard suite's checks hold it
        // so).
        let (min, max) = (-FRAC_PI_2 - 0.5, -FRAC_PI_2 + 0.5);
        let axes = [AxisInfo { id: "a", key: "", unit: "rad", min, max, max_vel: 1.0, max_acc: 10.0, max_jerk: 100.0 }];
        Arm { mass: 3.0, length: 0.4, rotor: 0.02, g, axes, motors: [MotorInfo { id: "a", key: "" }] }
    }

    impl Arm {
        fn inertia(&self) -> f64 {
            self.mass * self.length * self.length + self.rotor
        }
        fn parts(&self, s: &[f64]) -> [f64; 3] {
            [0.5 * self.inertia() * s[1] * s[1], self.mass * self.g * self.length * s[0].sin(), 0.0]
        }
    }

    impl Machine for Arm {
        fn frame_momentum(&self, s: &[f64]) -> Option<FrameMomentum> {
            let (sin, cos) = s[0].sin_cos();
            let mut f = FrameMomentum { gravity: [0.0, 0.0, -self.g], ..Default::default() };
            let (dir, ddir) = ([cos, 0.0, sin], [-sin, 0.0, cos]);
            let (l, m) = (self.length, self.mass);
            add_turning(&mut f, [0.0; 3], dir, ddir, s[1], m * l * l, m * l);
            add_spin(&mut f, [0.0, -1.0, 0.0], self.rotor, s[1]);
            Some(f)
        }
        fn axes(&self) -> &[AxisInfo] {
            &self.axes
        }
        fn motors(&self) -> &[MotorInfo] {
            &self.motors
        }
        fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
            motors[0] = tool[0];
        }
        fn forward(&self, motors: &[f64], tool: &mut [f64]) {
            tool[0] = motors[0];
        }
        fn state_len(&self) -> usize {
            2
        }
        fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
            *state = vec![tool[0], 0.0];
            torque[0] = self.mass * self.g * self.length * tool[0].cos();
        }
        fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
            out[0] = s[1];
            out[1] = (torque[0] - self.mass * self.g * self.length * s[0].cos()) / self.inertia();
        }
        fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
            (pos[0], vel[0]) = (s[0], s[1]);
        }
        fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
            s[1] = vel[0];
        }
        fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
            (pos[0], vel[0]) = (s[0], s[1]);
        }
        fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
            out[0] = self.inertia() * acc[0] + self.mass * self.g * self.length * pos[0].cos();
        }
        fn energy(&self, s: &[f64]) -> f64 {
            self.parts(s).iter().sum()
        }
        fn set_payload_attached(&mut self, _: bool) {}
        fn signal_infos(&self) -> Vec<SignalInfo> {
            ENERGY_PARTS.map(|p| SignalInfo::new(format!("energy.{p}"), "", "", "J")).to_vec()
        }
        fn base_signals(&self, _: &[f64], _: &[f64], _: &mut Vec<f64>) {}
        fn effect_signals(&self, s: &[f64], _: &[f64], out: &mut Vec<f64>) {
            out.extend(self.parts(s));
        }
        fn scene(&self, _: &[f64], _: &mut Scene) {}
    }

    /// A base config with effects.base_reaction as `reaction`: 200 kg, 10 kg m^2 about each axis, the stiffnesses of
    /// `DOFS`, damping ratio `zeta`, the start pose `pose`.
    fn base_config(reaction: bool, pose: f64, stiffness: [f64; 6], zeta: f64, pivot: [f64; 3]) -> Config {
        // `Config::set_text` only changes keys that are there, so every key is written into the text.
        let mut text = format!(
            "format_version = 1\n[effects]\nbase_compliance = true\nbase_reaction = {reaction}\n[start]\n\
             pose = [{pose:?}]\n[base]\npivot_m = {pivot:?}\nmass_kg = 200.0\n"
        );
        for (k, axis) in ["x", "y", "z"].iter().enumerate() {
            text += &format!(
                "stiffness_{axis}_n_m = {:?}\ntilt_stiffness_{axis}_nm_rad = {:?}\ninertia_{axis}_kgm2 = 10.0\n",
                stiffness[k],
                stiffness[3 + k]
            );
        }
        for id in DOFS {
            text += &format!("damping_ratio_{id} = {zeta:?}\n");
        }
        Config::parse(&text).unwrap()
    }

    /// A base along x of 200 kg on 8e5 N/m, damping ratio `zeta`, pivot at the origin.
    fn on_base(zeta: f64) -> Box<dyn Machine> {
        let stiffness = [8.0e5, 0.0, 0.0, 0.0, 0.0, 0.0];
        mount(&base_config(false, 0.5, stiffness, zeta, [0.0; 3]), Box::new(slider())).unwrap()
    }

    #[test]
    fn the_base_adds_a_deflection_and_its_rate_per_compliant_direction() {
        let m = on_base(0.0);
        assert_eq!(m.state_len(), 4);
        assert_eq!(m.signal_infos()[0].id, "base.x");
        // f = sqrt(k / M) / 2 pi = sqrt(8e5 / 200) / 2 pi = 63.25 / 2 pi = 10.07 Hz.
        let modes = m.modes(&[0.5]);
        assert!((modes[0].hz - 63.245_553 / std::f64::consts::TAU).abs() < 1e-6, "{modes:?}");
        assert_eq!(m.resonance_hz(&[0.5]), Some(modes[0].hz));
    }

    #[test]
    fn the_resonance_is_the_lower_of_the_machines_and_the_bases_and_the_motor_kind_is_forwarded() {
        // The base mode is 10.07 Hz (see above); a machine resonance of 0.35 Hz without a mode shape is the lower.
        let stiffness = [8.0e5, 0.0, 0.0, 0.0, 0.0, 0.0];
        let config = base_config(false, 0.5, stiffness, 0.0, [0.0; 3]);
        let m = mount(&config, Box::new(Slider { resonance: Some(0.35), ..slider() })).unwrap();
        assert_eq!(m.resonance_hz(&[0.5]), Some(0.35));
        assert!(m.motor_linear(0));
    }

    #[test]
    fn a_force_held_at_rest_deflects_the_base_and_the_tool_by_force_over_stiffness() {
        let mut m = on_base(0.3);
        let (mut state, mut torque) = (Vec::new(), vec![0.0]);
        m.equilibrium(&[0.5], &mut state, &mut torque);
        assert_eq!(state[2], 0.0);
        // Now the process force of 50 N pushes the slider while its motor holds it with -50 N: the motor's stator
        // takes +50 N, and the base gives way by 50 / 8e5 = 62.5 um once its sway has died down (damping ratio 0.3,
        // 10 Hz: after 2 s it is down by exp(-0.3 * 63.2 * 2), nothing).
        m.set_dwelling(true);
        let mut rk = Rk4::new(state.len());
        for _ in 0..2000 {
            advance(m.as_ref(), &mut rk, &mut state, &[-50.0], 1e-3, 4);
        }
        assert!((state[2] - 62.5e-6).abs() < 1e-9, "base {}", state[2]);
        let (mut pos, mut vel) = ([0.0], [0.0]);
        m.tool_state(&state, &mut pos, &mut vel);
        assert!((pos[0] - (0.5 + 62.5e-6)).abs() < 1e-9, "tool {}", pos[0]);
    }

    #[test]
    fn an_axis_stopping_on_a_soft_base_leaves_it_ringing_at_its_natural_frequency() {
        let m = on_base(0.0);
        let (mut state, mut torque) = (Vec::new(), vec![0.0]);
        m.equilibrium(&[0.5], &mut state, &mut torque);
        // 100 N for 50 ms, then -100 N for 50 ms: the slider (4 kg) moves and stops; the base, pushed back by -100 N
        // and then +100 N, rings on after. Undamped, its zero crossings come every half period, 1 / (2 f) =
        // pi sqrt(M / k) = pi sqrt(200 / 8e5) = 49.67 ms.
        let mut rk = Rk4::new(state.len());
        let dt = 1e-4;
        let mut crossings = Vec::new();
        let mut last = state[2];
        for step in 0..20_000 {
            let t = step as f64 * dt;
            let force = if t < 0.05 {
                100.0
            } else if t < 0.1 {
                -100.0
            } else {
                0.0
            };
            advance(m.as_ref(), &mut rk, &mut state, &[force], dt, 1);
            if t > 0.1 && last.signum() != state[2].signum() {
                // Where the line between the two samples crosses zero.
                crossings.push(t + dt * last / (last - state[2]));
            }
            last = state[2];
        }
        assert!((state[1]).abs() < 1e-12, "the slider stopped");
        let half = (crossings[crossings.len() - 1] - crossings[0]) / (crossings.len() - 1) as f64;
        assert!((half - std::f64::consts::PI * (200.0f64 / 8.0e5).sqrt()).abs() < 1e-6, "half period {half}");
    }

    /// Two-way (effects.base_reaction), the drive off and the belt replaced by a spring: the slider of m = 4 kg held by
    /// k_c = 2e5 N/m to the base, the base 200 kg in all (M0 = 196 kg of its own) on k_b = 8e5 N/m along x. A two-mass
    /// system, the base x0 and the slider x1 over ground: M0 x0'' = -k_b x0 + k_c (x1 - x0), m x1'' = -k_c (x1 - x0).
    /// Its modes w^2 = (p -+ sqrt(p^2 - 4 q)) / 2 with p = (k_b + k_c) / M0 + k_c / m = 5102.04 + 50000 = 55102.04 and
    /// q = k_b k_c / (M0 m) = 2.0408e8: w^2 = 3993.07 and 51108.97, 10.057 Hz and 35.981 Hz; in each the slider moves
    /// r = k_c / (k_c - m w^2) times as far as the base, 1.0868 and -45.09. Started at rest in either shape, the base
    /// swings as A cos(w t) and the slider keeps the shape, to 1e-6 of A over 0.2 s (2 and 7 periods). One-way the base
    /// rings at sqrt(k_b / 200) = 10.066 Hz whatever the slider does.
    #[test]
    fn a_slider_on_a_spring_on_a_coupled_base_swings_in_the_two_mass_modes() {
        let (m, kb, kc): (f64, f64, f64) = (4.0, 8.0e5, 2.0e5);
        let m0 = 200.0 - m;
        let (p, q) = ((kb + kc) / m0 + kc / m, kb * kc / (m0 * m));
        let modes = [-1.0, 1.0].map(|sign: f64| (0.5 * (p + sign * (p * p - 4.0 * q).sqrt())).sqrt());
        let hz = modes.map(|w| w / std::f64::consts::TAU);
        assert!((hz[0] - 10.057).abs() < 1e-3 && (hz[1] - 35.981).abs() < 1e-3, "{hz:?}");
        let config = base_config(true, 0.5, [kb, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, [0.0; 3]);
        for w in modes {
            let machine = mount(&config, Box::new(Slider { spring: kc, ..slider() })).unwrap();
            let (r, a, dt) = (kc / (kc - m * w * w), 1e-6, 1e-5);
            let mut state = vec![0.5 + (r - 1.0) * a, 0.0, a, 0.0];
            let mut rk = Rk4::new(state.len());
            let (mut swing, mut shape) = (0f64, 0f64);
            for step in 1..=20_000 {
                advance(machine.as_ref(), &mut rk, &mut state, &[0.0], dt, 1);
                swing = swing.max((state[2] - a * (w * step as f64 * dt).cos()).abs());
                shape = shape.max((state[0] - 0.5 - (r - 1.0) * state[2]).abs() / (r - 1.0).abs());
            }
            assert!(swing < 1e-6 * a && shape < 1e-6 * a, "{w} rad/s: off by {swing} m, shape by {shape} m");
        }
    }

    /// Two-way, the base free (springs at zero) along x and z and in tilt about y through the pivot (0.1, 0, -0.2),
    /// 20 kg and 2 kg m^2, no gravity and no outside force: a torque pulse swings the arm (3 kg at 0.4 m, rotor 0.02 kg
    /// m^2) from 0.3 rad. Nothing outside pushes base and arm, so their momentum stays what it was, zero: M_B d' + p along
    /// x and z and I_B theta' + L (about the pivot) about y, to 1e-9 of the arm's own while it swings. One-way keeps it
    /// too (the base takes dp/dt whatever the machine does); this checks that the two are solved as one: with the
    /// machine feeling the base but the base's equation left without A M^-1 A^T it fails.
    #[test]
    fn a_free_coupled_base_and_the_machine_on_it_keep_their_momentum() {
        let inner = arm(0.0);
        let (mut state, mut torque) = (Vec::new(), [0.0]);
        inner.equilibrium(&[0.3], &mut state, &mut torque);
        let pivot = [0.1, 0.0, -0.2];
        let at_rest = inner.frame_momentum(&state).unwrap();
        let reaction = Reaction::new(&inner, &state, &torque, pivot, &at_rest);
        assert_eq!(reaction.speeds, [1]);
        let free = |index, mass| Dof { index, mass, stiffness: 0.0, damping: 0.0 };
        let dofs = vec![free(0, 20.0), free(2, 20.0), free(4, 2.0)];
        let m = Based {
            inner: Box::new(inner),
            dofs: dofs.clone(),
            pivot,
            reference: static_load(&at_rest),
            external_reference: external_wrench(&at_rest),
            inner_base_signals: 0,
            reaction: Some(reaction),
        };
        state.extend([0.0; 6]);
        let mut rk = Rk4::new(state.len());
        let (dt, mut own, mut total) = (1e-4, 0f64, 0f64);
        for step in 0..5000 {
            let t = step as f64 * dt;
            let pulse = if t < 0.2 { 4.0 * (std::f64::consts::PI * t / 0.2).sin().powi(2) } else { 0.0 };
            advance(&m, &mut rk, &mut state, &[pulse], dt, 1);
            let p = momentum(&m.frame_momentum(&state).unwrap(), pivot);
            for (k, d) in dofs.iter().enumerate() {
                own = own.max(p[d.index].abs());
                total = total.max((d.mass * state[5 + k] + p[d.index]).abs());
            }
        }
        assert!(own > 0.1 && total < 1e-9 * own, "the arm's momentum up to {own}, with the base's {total}");
    }

    /// Two-way, losses off: the arm hanging in gravity on a base along x and z (8e5 N/m) and tilting about y through a
    /// pivot 0.3 m below the arm's joint (5e3 N m/rad, soft, so that the tilt turns gravity in the arm's frame by a
    /// measurable angle), undamped. The standard suite's energy check: the arm's energy and the base's (its sway with the
    /// arm's momentum, its springs, the arm's weight on its tilt) stay within 0.1 % of what the torque pulse put in over
    /// 10 s (without the tilt's turn of gravity on the arm 0.32 %; one-way more). With a damping ratio of 0.05 the loss
    /// budget closes with the base's damping loss `power.base` and its energy in the `energy.*` signals.
    #[test]
    fn a_coupled_base_keeps_the_energy_and_closes_the_loss_budget() {
        let stiffness = [8.0e5, 0.0, 8.0e5, 0.0, 5.0e3, 0.0];
        let config = |zeta: f64| base_config(true, -FRAC_PI_2, stiffness, zeta, [0.0, 0.0, -0.3]);
        let on = |zeta: f64| mount(&config(zeta), Box::new(arm(9.81)));
        sim_core::testkit::energy_conservation(on(0.0).unwrap().as_ref());
        let damped = on(0.05).unwrap();
        assert_eq!(damped.signal_infos().last().unwrap().id, "power.base");
        sim_core::testkit::loss_budget(damped.as_ref());
    }

    /// The process force's work on the moving base is in the base's row. The slider dwells under 50 N along x on a base
    /// of 200 kg and 8e5 N/m (damping ratio 0.05: c = 2 * 0.05 * sqrt(8e5 * 200) = 1264.911 N s/m) moving at 0.3 m/s:
    /// the row is c v^2 - F v = 113.842 - 15 = 98.842 W (the force was not on at the start pose, so none is taken off).
    #[test]
    fn the_process_force_works_on_the_moving_base_in_its_row() {
        let config = base_config(true, 0.5, [8.0e5, 0.0, 0.0, 0.0, 0.0, 0.0], 0.05, [0.0; 3]);
        let mut machine = mount(&config, Box::new(slider())).unwrap();
        machine.set_dwelling(true);
        let state = [0.5, 0.0, 1e-4, 0.3];
        let mut rows = Vec::new();
        machine.loss_powers(&state, &[0.0], &mut rows);
        assert!((rows.last().unwrap() - 98.842).abs() < 1e-3, "{rows:?}");
    }

    /// The budget closes with the process force on a moving base: the slider (dwelling under 50 N along x, its motor
    /// pushing 30 N back) on a coupled base along x (200 kg, 8e5 N/m, damping ratio 0.05), integrated by RK4 over 50 ms
    /// from rest. The motor's work and the process force's on the slider's own travel, int (tau + F) x' dt, equal the
    /// change of the energy plus the base's row, int (c d'^2 - F d') dt (trapezoid on 1 us steps).
    #[test]
    fn the_budget_closes_with_the_process_force_on_a_moving_base() {
        let config = base_config(true, 0.5, [8.0e5, 0.0, 0.0, 0.0, 0.0, 0.0], 0.05, [0.0; 3]);
        let mut machine = mount(&config, Box::new(slider())).unwrap();
        machine.set_dwelling(true);
        let (dt, tau, force) = (1e-6, [-30.0], 50.0);
        let mut s = vec![0.5, 0.0, 0.0, 0.0];
        let row = |s: &[f64]| {
            let mut rows = Vec::new();
            machine.loss_powers(s, &tau, &mut rows);
            *rows.last().unwrap()
        };
        let start = machine.energy(&s);
        let (mut work, mut lost, mut last, mut fastest) = (0.0, 0.0, ((tau[0] + force) * s[1], row(&s)), 0.0f64);
        let derivative = |s: &[f64]| {
            let mut out = vec![0.0; s.len()];
            machine.derivatives(s, &tau, &mut out);
            out
        };
        for _ in 0..50_000 {
            let k1 = derivative(&s);
            let at = |k: &[f64], h: f64| s.iter().zip(k).map(|(x, d)| x + h * d).collect::<Vec<_>>();
            let k2 = derivative(&at(&k1, 0.5 * dt));
            let k3 = derivative(&at(&k2, 0.5 * dt));
            let k4 = derivative(&at(&k3, dt));
            for i in 0..s.len() {
                s[i] += dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            }
            let now = ((tau[0] + force) * s[1], row(&s));
            (work, lost) = (work + 0.5 * dt * (now.0 + last.0), lost + 0.5 * dt * (now.1 + last.1));
            last = now;
            fastest = fastest.max(s[3].abs());
        }
        let stored = machine.energy(&s) - start;
        assert!(fastest > 1e-3 && work > 0.01, "the base moves ({fastest} m/s), the slider is pushed ({work} J)");
        assert!((work - stored - lost).abs() < 1e-9, "work {work} J, stored {stored} J, base row {lost} J");
    }

    /// The scene tilts with the base. The base is moved 2e-4 m along x and tilted 1e-3 rad about y through the origin;
    /// a part at (0.2, 0, 0.5) and a belt strip's far end at (0.8, 0, -0.25) move by t + theta x p, theta = (0, 1e-3, 0):
    /// (2e-4 + 1e-3 * 0.5, 0, -1e-3 * 0.2) = (7e-4, 0, -2e-4) and (2e-4 - 2.5e-4, 0, -8e-4) = (-5e-5, 0, -8e-4).
    #[test]
    fn the_scene_tilts_with_the_base_at_each_points_own_place() {
        let config = base_config(false, 0.5, [8.0e5, 0.0, 0.0, 0.0, 5.0e3, 0.0], 0.0, [0.0; 3]);
        let machine = mount(&config, Box::new(slider())).unwrap();
        let mut scene = Scene::default();
        let shape = sim_core::Shape::Segment { to: [0.8, 0.0, -0.25], width: 0.01 };
        scene.parts.push(sim_core::Part::new("belt", shape, [0.2, 0.0, 0.5], [1.0; 3]));
        // The slider at 0.5, then the base's x and tilt about y, then their rates.
        let state = [0.5, 0.0, 2e-4, 1e-3, 0.0, 0.0];
        machine.scene(&state, &mut scene);
        let part = &scene.parts[0];
        let sim_core::Shape::Segment { to, .. } = &part.shape else { unreachable!() };
        let (a, b) = ([7e-4, 0.0, -2e-4], [-5e-5, 0.0, -8e-4]);
        for k in 0..3 {
            assert!((part.pos[k] - [0.2, 0.0, 0.5][k] - a[k]).abs() < 1e-12, "{:?}", part.pos);
            assert!((to[k] - [0.8, 0.0, -0.25][k] - b[k]).abs() < 1e-12, "{to:?}");
        }
    }

    /// The same base move on a scene that shows x the other way (a reflection, x -> -x): the point the scene shows at
    /// (-0.2, 0, 0.5) is the machine's (0.2, 0, 0.5), which moves by (7e-4, 0, -2e-4) as above; in the scene that is
    /// (-7e-4, 0, -2e-4). (Mapping the tilt as a plain vector, not as a pseudovector, gives +2e-4 in z.)
    #[test]
    fn the_scene_tilts_with_the_base_on_a_flipped_scene() {
        let config = base_config(false, 0.5, [8.0e5, 0.0, 0.0, 0.0, 5.0e3, 0.0], 0.0, [0.0; 3]);
        let machine = mount(&config, Box::new(Slider { flipped: true, ..slider() })).unwrap();
        let mut scene = Scene::default();
        scene.parts.push(sim_core::Part::new(
            "box",
            sim_core::Shape::Box { size: [0.1; 3] },
            [-0.2, 0.0, 0.5],
            [1.0; 3],
        ));
        machine.scene(&[0.5, 0.0, 2e-4, 1e-3, 0.0, 0.0], &mut scene);
        let moved: [f64; 3] = std::array::from_fn(|k| scene.parts[0].pos[k] - [-0.2, 0.0, 0.5][k]);
        for (m, want) in moved.iter().zip([-7e-4, 0.0, -2e-4]) {
            assert!((m - want).abs() < 1e-12, "{moved:?}");
        }
    }

    /// A base lighter than what the machine moves on it cannot be: M_B - A M^-1 A^T would not be positive.
    #[test]
    fn a_coupled_base_must_be_heavier_than_what_moves_on_it() {
        let mut config = base_config(true, 0.5, [8.0e5, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, [0.0; 3]);
        config.set_text("base.mass_kg = 3.9").unwrap();
        assert!(matches!(mount(&config, Box::new(slider())), Err(ConfigError::Invalid(k)) if k == "base.mass_kg"));
        config.set_text("base.mass_kg = 4.1").unwrap();
        assert!(mount(&config, Box::new(slider())).is_ok());
    }
}
