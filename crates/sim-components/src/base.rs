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
//! shortcut: one-way coupling, the machine's own dynamics do not feel the base's acceleration (its motors neither see
//! the sway in their following error nor hold against it); exact where the servo holds the axes stiffly to the base or
//! the base is much heavier than what moves; upgrade by passing the base's acceleration to the machines as a
//! pseudo-gravity once one needs the sway in the motor traces. A gantry crate's and the machine files' own frame mode
//! (effects.frame_compliance) is the coupled one along one axis: the portal's posts swaying under its beam; the base is
//! what the whole machine stands on, in series with it.

use sim_core::{
    AxisInfo, BeltSection, Config, ConfigError, FrameMomentum, FrictionContact, Machine, Mode, MotorInfo, RatingCheck,
    Scene, SignalInfo, ThermalPart,
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
    /// How many of the machine's signals are its base signals.
    inner_base_signals: usize,
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
    Ok(Box::new(Based {
        reference: static_load(&at_rest),
        inner_base_signals: signals.len(),
        inner: machine,
        dofs,
        pivot,
    }))
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

    /// The machine's pose alone (without the base) and the base's move of the scene at state `s`, `scale` times.
    fn scene_shift(&self, s: &[f64], scale: f64) -> Option<[f64; 3]> {
        let n = self.inner.axes().len();
        let (mut pose, mut vel) = (vec![0.0; n], vec![0.0; n]);
        self.inner.tool_state(&s[..self.n()], &mut pose, &mut vel);
        let shift = self.tool_shift(&pose, &self.full(s, false));
        let moved: Vec<f64> = pose.iter().zip(&shift).map(|(p, d)| p + scale * d).collect();
        let (from, to) = (self.inner.scene_point(&pose)?, self.inner.scene_point(&moved)?);
        Some(std::array::from_fn(|k| to[k] - from[k]))
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
        for (k, d) in self.dofs.iter().enumerate() {
            let (x, v) = (state[n + k], state[n + m + k]);
            out[n + k] = v;
            out[n + m + k] = (load[d.index] - d.stiffness * x - d.damping * v) / d.mass;
        }
    }
    fn tool_state(&self, state: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        self.inner.tool_state(&state[..self.n()], pos, vel);
        let shift = self.tool_shift(pos, &self.full(state, false));
        let rate = self.tool_shift(pos, &self.full(state, true));
        pos.iter_mut().zip(shift).for_each(|(p, d)| *p += d);
        vel.iter_mut().zip(rate).for_each(|(v, d)| *v += d);
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
    fn rigid_tensions(&self, tool_pos: &[f64], tool_acc: &[f64], out: &mut Vec<f64>) {
        self.inner.rigid_tensions(tool_pos, tool_acc, out)
    }
    fn rigid_guide_loads(&self, tool_pos: &[f64], tool_acc: &[f64], out: &mut Vec<[f64; 2]>) {
        self.inner.rigid_guide_loads(tool_pos, tool_acc, out)
    }
    /// The machine's, plus the base's kinetic and elastic energy (its potential energy in gravity left out: the
    /// machine's own energy is not conserved with it anyway, see the module's shortcut).
    fn energy(&self, state: &[f64]) -> f64 {
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
    fn links(&self) -> Vec<String> {
        self.inner.links()
    }
    fn set_link_change(&mut self, link: usize, factor: f64, moment: [f64; 3]) {
        self.inner.set_link_change(link, factor, moment)
    }
    fn set_dwelling(&mut self, dwelling: bool) {
        self.inner.set_dwelling(dwelling)
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
        self.mode_shape(tool_pose).map(|(hz, _)| hz).or_else(|| self.inner.resonance_hz(tool_pose))
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
    /// The machine's base signals, the base's deflections (`base.<dof>`, m or rad), then the machine's effect signals.
    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut infos = self.inner.signal_infos();
        let base = self.dofs.iter().map(|d| {
            let id = DOFS[d.index];
            SignalInfo::new(format!("base.{id}"), "base", &format!("quantity-base-{id}"), ["m", "rad"][d.index / 3])
        });
        infos.splice(self.inner_base_signals..self.inner_base_signals, base);
        infos
    }
    fn base_signals(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let n = self.n();
        self.inner.base_signals(&state[..n], torque, out);
        out.extend_from_slice(&state[n..n + self.dofs.len()]);
    }
    fn effect_signals(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.inner.effect_signals(&state[..self.n()], torque, out)
    }
    fn loss_powers(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.inner.loss_powers(&state[..self.n()], torque, out)
    }
    /// The machine's scene, moved as far as the base moves its tool, times the exaggeration.
    // shortcut: the scene moves as a whole by the base's move at the tool, neither tilting nor moving the polylines of
    // belts; enough to see the base sway, upgrade if a tilting base has to be drawn.
    fn scene(&self, state: &[f64], out: &mut Scene) {
        self.inner.scene(&state[..self.n()], out);
        let Some(d) = self.scene_shift(state, out.exaggeration) else { return };
        let add = |p: &mut [f64; 3]| (0..3).for_each(|k| p[k] += d[k]);
        out.parts.iter_mut().for_each(|p| add(&mut p.pos));
        out.labels.iter_mut().for_each(|l| add(&mut l.pos));
        out.arrows.iter_mut().for_each(|a| add(&mut a.pos));
    }
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        self.inner.scene_point(tool_pose)
    }
    fn collision_bodies(&self, state: &[f64]) -> Vec<sim_core::Aabb> {
        let mut bodies = self.inner.collision_bodies(&state[..self.n()]);
        if let Some(d) = self.scene_shift(state, 1.0) {
            bodies.iter_mut().for_each(|b| (0..3).for_each(|k| b.centre[k] += d[k]));
        }
        bodies
    }
    fn friction_contacts(&self) -> Vec<FrictionContact> {
        self.inner.friction_contacts()
    }
    fn friction_curve(&self, i: usize, speed: f64, load_scale: f64) -> [f64; 3] {
        self.inner.friction_curve(i, speed, load_scale)
    }
    fn belt_sections(&self) -> Vec<BeltSection> {
        self.inner.belt_sections()
    }
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        self.inner.thermal_parts()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::{Rk4, advance};

    /// A point mass on a frictionless guide along x, pushed by its motor's force (torque = force), with a process force
    /// while it dwells: the smallest machine that stands on a base.
    struct Slider {
        mass: f64,
        process: f64,
        dwelling: bool,
        axes: [AxisInfo; 1],
        motors: [MotorInfo; 1],
    }

    fn slider() -> Slider {
        Slider {
            mass: 4.0,
            process: 50.0,
            dwelling: false,
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
            out[1] = (torque[0] + process) / self.mass;
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
            0.5 * self.mass * s[1] * s[1]
        }
        fn set_payload_attached(&mut self, _: bool) {}
        fn set_dwelling(&mut self, dwelling: bool) {
            self.dwelling = dwelling;
        }
        fn signal_infos(&self) -> Vec<SignalInfo> {
            Vec::new()
        }
        fn base_signals(&self, _: &[f64], _: &[f64], _: &mut Vec<f64>) {}
        fn scene(&self, _: &[f64], _: &mut Scene) {}
    }

    /// A base along x of 200 kg on 8e5 N/m, damping ratio `zeta`, pivot at the origin.
    fn on_base(zeta: f64) -> Box<dyn Machine> {
        // `Config::set_text` only changes keys that are there, so every key is written into the text.
        let mut text = "format_version = 1\n[effects]\nbase_compliance = true\n[start]\npose = [0.5]\n[base]\n\
                        pivot_m = [0.0, 0.0, 0.0]\nmass_kg = 200.0\n"
            .to_string();
        for axis in ["x", "y", "z"] {
            let k = if axis == "x" { 8.0e5 } else { 0.0 };
            text += &format!(
                "stiffness_{axis}_n_m = {k:?}\ntilt_stiffness_{axis}_nm_rad = 0.0\ninertia_{axis}_kgm2 = 10.0\n"
            );
        }
        for id in DOFS {
            text += &format!("damping_ratio_{id} = {zeta:?}\n");
        }
        mount(&Config::parse(&text).unwrap(), Box::new(slider())).unwrap()
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
}
