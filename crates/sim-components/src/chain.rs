//! A machine of joints, each driven by a motor through a transmission (`joint::RotaryJoint`): the machine for every
//! mechanism that can say its dynamics in joint coordinates, M(q) q'' + h(q, q') = tau. A joint's coordinate is an angle
//! (rad) or a travel (m); its gearbox's ratio is motor radians per unit of it, so a ball screw is a gearbox of ratio
//! 2 pi / lead (times any reduction before it), a rack 1 / pinion radius, a linear motor 1 (its "rotor" the forcer, its
//! torques forces). Stiffness, backlash, friction and stops are those at the joint, in its unit.
//!
//! Rigid: the rotors turn with their joints times the ratio, their inertia counts at the joint times the ratio squared,
//! and each gearbox loses its efficiency on what passes through it (two passes, as on the delta). Elastic
//! (`effects.joint_elasticity`): the rotors are states of their own behind the gearboxes' springs.
//!
//! A mechanism (`Mechanism`) gives the load's dynamics, its kinematics from joints to the tool, its own signals and
//! rating checks and its scene. Everything else (states, losses, energy, torque breakdown, friction view, heating
//! gearboxes) is here, the same for all.

use crate::joint::RotaryJoint;
use crate::mech::ThermalFeedback;
use sim_core::{
    AxisInfo, ENERGY_PARTS, FRICTION_SIGNALS, FrameMomentum, FrictionContact, LOSS_PREFIX, Machine, MotorInfo,
    RatingCheck, Scene, SignalInfo, TORQUE_PARTS, ThermalPart,
};

/// Solves the n x n system `m` (row by row) x = b by Gaussian elimination with partial pivoting; `None` where `m` is
/// singular.
pub fn solve(m: &[f64], b: &[f64]) -> Option<Vec<f64>> {
    let n = b.len();
    let mut a = m.to_vec();
    let mut x = b.to_vec();
    for c in 0..n {
        let p = (c..n).max_by(|&i, &j| a[i * n + c].abs().total_cmp(&a[j * n + c].abs()))?;
        if !(a[p * n + c].abs() > 1e-300) {
            return None;
        }
        if p != c {
            for k in 0..n {
                a.swap(p * n + k, c * n + k);
            }
            x.swap(p, c);
        }
        for r in c + 1..n {
            let f = a[r * n + c] / a[c * n + c];
            for k in c..n {
                a[r * n + k] -= f * a[c * n + k];
            }
            x[r] -= f * x[c];
        }
    }
    for c in (0..n).rev() {
        let s: f64 = (c + 1..n).map(|k| a[c * n + k] * x[k]).sum();
        x[c] = (x[c] - s) / a[c * n + c];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// What a mechanism tells the joint machine.
pub trait Mechanism: Send {
    /// The load's mass matrix M (n x n, row by row, without the rotors) and h with M q'' + h = the torques (forces) at
    /// the joints: velocity terms plus gravity. With the payload as it is attached. Where the mechanism has passive
    /// joints (`passive_joints`), q and q' hold them after the driven ones, and M and h are of them all.
    fn dynamics(&self, q: &[f64], qd: &[f64], mass: &mut [f64], bias: &mut [f64]);
    /// The load's potential energy (passive joints included, their springs not); its gravity torques are those of h at
    /// rest.
    fn potential(&self, q: &[f64]) -> f64;
    /// Joints that no motor drives, after the driven ones (a payload swinging on the flange): each with its spring
    /// (N m/rad) and damper (N m s/rad) towards zero. None by default.
    fn passive_joints(&self) -> Vec<(f64, f64)> {
        Vec::new()
    }
    fn set_payload_attached(&mut self, attached: bool);
    /// As `Machine::links` and `Machine::set_link_change`: the bodies whose inertial parameters a trace can be fitted for.
    fn links(&self) -> Vec<String> {
        Vec::new()
    }
    fn set_link_change(&mut self, _link: usize, _factor: f64, _moment: [f64; 3]) {}

    /// Joints to tool pose. The joints themselves by default.
    fn forward(&self, q: &[f64], tool: &mut [f64]) {
        tool.copy_from_slice(q);
    }
    /// Tool pose to joints; `false` where it cannot be reached.
    fn inverse(&self, tool: &[f64], q: &mut [f64]) -> bool {
        q.copy_from_slice(tool);
        true
    }
    /// What `inverse` carries from one call to the next (a numerical inverse's last solution, which the next starts
    /// from), as a fixed count of numbers, for `Machine::memory`. None by default.
    fn memory(&self) -> Vec<f64> {
        Vec::new()
    }
    /// Sets back what `memory` gave.
    fn set_memory(&mut self, _memory: &[f64]) {}
    fn unreachable(&self, _tool: &[f64], _slack: f64) -> Option<&'static str> {
        None
    }
    /// `Machine::condition` at a tool pose; `None` where the tool is the joints.
    fn condition(&self, _tool: &[f64]) -> Option<f64> {
        None
    }
    /// Whether the tool pose is the joints themselves (`forward` and `inverse` as by default): then the joint speeds and
    /// accelerations are the tool's, without going through the Jacobian.
    fn tool_is_joints(&self) -> bool {
        true
    }

    /// Whether the mechanism works on a process (a press force, a cut), which `process_torque` gives: then the machine
    /// reports the power given to it as `power.process`. None by default.
    fn has_process(&self) -> bool {
        false
    }
    /// Adds the torques (forces) the process puts on the joints at positions `q` and speeds `qd` to `out`, positive
    /// turning a joint up. Not conservative and not in `dynamics`: the torque breakdown counts it among the rest, and
    /// the feedforward leaves it out, as a drive does not know it.
    fn process_torque(&self, _q: &[f64], _qd: &[f64], _out: &mut [f64]) {}

    /// As `Machine::cam` and `Machine::cam_at`: an electronic cam the mechanism runs as its cycle, its slaves the tool
    /// axes. None by default.
    fn cam(&self) -> Option<sim_core::CamMaster> {
        None
    }
    fn cam_at(&self, _master: f64, _out: &mut [[f64; 4]]) {}

    /// The mechanism's own signals, after the machine's: their infos, and their values from joint positions, speeds,
    /// accelerations and the force (torque) each transmission passes to its joint.
    fn signal_infos(&self) -> Vec<SignalInfo> {
        Vec::new()
    }
    fn signals(&self, _q: &[f64], _qd: &[f64], _qdd: &[f64], _force: &[f64], _out: &mut Vec<f64>) {}
    fn rating_checks(&self) -> Vec<RatingCheck> {
        Vec::new()
    }
    /// As `Machine::rigid_guide_loads`: the loads of the guides among `signals`, at a tool pose and acceleration.
    fn rigid_guide_loads(&self, _tool: &[f64], _acc: &[f64], _out: &mut Vec<[f64; 2]>) {}

    /// The ids of the boxes `scene` draws that move with the load and can hit an obstacle (`Machine::collision_bodies`).
    fn collision_parts(&self) -> &'static [&'static str] {
        &[]
    }

    /// For effects.base_compliance (`Machine::frame_momentum`): the load's momentum at joints q and speeds qd (passive
    /// ones included), in the frame of the tool pose's x, y, z, each driven joint's rotor adding `rotor_spin` (its
    /// inertia times its speed: N m s, or N s for a linear motor's forcer) along its own axis. `None` by default.
    fn frame_momentum(&self, _q: &[f64], _qd: &[f64], _rotor_spin: &[f64]) -> Option<FrameMomentum> {
        None
    }
    /// Bending modes of the load's links (effects.link_bending): its motion drives them and they move the tool, but the
    /// joints do not feel them (one-way, as `base::Based`). Their count must not change. Empty by default.
    fn flex_modes(&self) -> Vec<FlexMode> {
        Vec::new()
    }
    /// The force on each of `flex_modes` at the driven joints' positions q, speeds qd and accelerations qdd, in its
    /// coordinate (N).
    fn flex_loads(&self, _q: &[f64], _qd: &[f64], _qdd: &[f64], _out: &mut [f64]) {}
    /// How far the tool pose moves at joints q when the modes deflect by `deflection`, into `out` (zeroed first).
    fn flex_shift(&self, _q: &[f64], _deflection: &[f64], _out: &mut [f64]) {}

    /// Draws the machine at joint positions `q` (passive ones included) with the motor shafts at `rotors` (rad) turning
    /// at `rotor_speeds`.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene);
    fn scene_point(&self, _tool: &[f64]) -> Option<[f64; 3]> {
        None
    }
}

/// A bending mode of a mechanism's load (`Mechanism::flex_modes`): its modal mass (kg), stiffness (N/m) and damping
/// (N s/m), in the coordinate of its deflection (a link's tip, m), and the link (0 the first) it bends.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlexMode {
    pub link: usize,
    pub mass: f64,
    pub stiffness: f64,
    pub damping: f64,
}

/// One joint as the machine sees it: its drive, the joint's id and translation key (for friction contacts and signals),
/// and where it has a gearbox ahead of a converter (screw, rack): the joint's travel per radian of the gearbox output,
/// so that the torque at the gearbox output is the joint force times it (signal `gearbox.<motor>.output_torque`).
#[derive(Debug, Clone)]
pub struct JointSpec {
    pub drive: RotaryJoint,
    pub id: &'static str,
    pub key: &'static str,
    pub gear_lever: Option<f64>,
    /// The joint's unit: "m" for a travel, "rad" for an angle (the axes may be the tool's, in other units).
    pub unit: &'static str,
}

pub struct JointMachine<M: Mechanism> {
    pub mechanism: M,
    pub joints: Vec<JointSpec>,
    pub elastic: bool,
    pub thermal: Option<ThermalFeedback>,
    pub axes: Vec<AxisInfo>,
    pub motors: Vec<MotorInfo>,
    brake_engaged: bool,
    /// `Machine::setpoint_slack`, worked out once (`rest_sag`).
    rest_slack: f64,
    /// `Mechanism::passive_joints`: their angles and speeds follow the joint machine's own states.
    passive: Vec<(f64, f64)>,
    /// How many bending modes the mechanism has (`Mechanism::flex_modes`); their deflections and rates end the state,
    /// after the passive joints'.
    flex: usize,
}

/// The losses of one moment: per joint its friction, its motor's friction with the brake, its gearbox; then the stops
/// and what the ripple gives off. And per joint the gearbox loss as a torque at the rotor.
struct Losses {
    powers: Vec<f64>,
    gear_at_rotor: Vec<f64>,
}

impl<M: Mechanism> JointMachine<M> {
    /// `axes` as many as joints, or fewer where the joints are redundant to the tool (a seven-joint arm whose tool has
    /// six axes: the mechanism's `inverse` resolves the redundancy); one motor per joint, in their order.
    pub fn new(
        mechanism: M,
        joints: Vec<JointSpec>,
        elastic: bool,
        thermal: Option<ThermalFeedback>,
        axes: Vec<AxisInfo>,
        motors: Vec<MotorInfo>,
    ) -> Self {
        assert!(axes.len() <= joints.len() && joints.len() == motors.len());
        let (passive, flex) = (mechanism.passive_joints(), mechanism.flex_modes().len());
        let mut machine = Self {
            mechanism,
            joints,
            elastic,
            thermal,
            axes,
            motors,
            brake_engaged: false,
            rest_slack: 0.0,
            passive,
            flex,
        };
        machine.rest_slack = machine.rest_sag();
        machine
    }

    /// How far the motors' pose can be from the tool's at rest, as the delta works it out: each gearbox twisted by the
    /// torque it holds against gravity over its stiffness, plus half its backlash, through the forward kinematics; at
    /// the corners and the centre of the axes' box, with the payload as it is (attached at the start); plus a tenth,
    /// at least the default 0.1 mm. That alone in the rigid model, whose gearboxes do not twist. An arm's gears under
    /// its weight: some mrad.
    fn rest_sag(&self) -> f64 {
        let n = self.n();
        if !self.elastic {
            return 1e-4;
        }
        let mut poses: Vec<Vec<f64>> = (0..1usize << n)
            .map(|c| self.axes.iter().enumerate().map(|(i, a)| if c >> i & 1 == 1 { a.max } else { a.min }).collect())
            .collect();
        poses.push(self.axes.iter().map(|a| 0.5 * (a.min + a.max)).collect());
        let m = self.axes.len();
        let (mut q, mut tool, mut sagged) = (vec![0.0; n], vec![0.0; m], vec![0.0; m]);
        let mut most = 0f64;
        for pose in poses {
            if !self.mechanism.inverse(&pose, &mut q) {
                continue;
            }
            let (_, gravity) = self.load(&q, &vec![0.0; n]);
            let twisted: Vec<f64> =
                (0..n).map(|k| q[k] + self.joints[k].drive.gearbox.static_twist(gravity[k])).collect();
            self.mechanism.forward(&q, &mut tool);
            self.mechanism.forward(&twisted, &mut sagged);
            most = most.max(tool.iter().zip(&sagged).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max));
        }
        f64::max(1e-4, 1.1 * most)
    }

    fn n(&self) -> usize {
        self.joints.len()
    }

    /// Where the bending modes' deflections start in the state, after the passive joints'; their rates follow them.
    fn flex_at(&self) -> usize {
        self.passive_at() + 2 * self.passive.len()
    }

    /// The bending modes' deflections and rates.
    fn flex_state<'a>(&self, s: &'a [f64]) -> (&'a [f64], &'a [f64]) {
        let (at, m) = (self.flex_at(), self.flex);
        (&s[at..at + m], &s[at + m..at + 2 * m])
    }

    /// The bending modes' rates into `out`, from the joints' accelerations `acc` (the driven ones first): m d'' + c d' +
    /// k d = load.
    fn flex_rates(&self, s: &[f64], acc: &[f64], out: &mut [f64]) {
        if self.flex == 0 {
            return;
        }
        let (q, qd) = self.arms(s);
        let mut load = vec![0.0; self.flex];
        self.mechanism.flex_loads(q, qd, &acc[..q.len()], &mut load);
        let ((d, rate), at, m) = (self.flex_state(s), self.flex_at(), self.flex);
        for (k, mode) in self.mechanism.flex_modes().iter().enumerate() {
            out[at + k] = rate[k];
            out[at + m + k] = (load[k] - mode.stiffness * d[k] - mode.damping * rate[k]) / mode.mass;
        }
    }

    /// The tool's move by the bending modes at joints q, deflections d: `Mechanism::flex_shift`.
    fn flex_shift(&self, q: &[f64], d: &[f64]) -> Vec<f64> {
        let mut out = vec![0.0; self.axes.len()];
        if self.flex > 0 {
            self.mechanism.flex_shift(q, d, &mut out);
        }
        out
    }

    /// Joint positions and speeds of a state.
    fn arms<'a>(&self, s: &'a [f64]) -> (&'a [f64], &'a [f64]) {
        let n = self.n();
        (&s[..n], &s[n..2 * n])
    }

    /// Rotor angles and speeds.
    fn rotors(&self, s: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = self.n();
        if self.elastic {
            (s[2 * n..3 * n].to_vec(), s[3 * n..4 * n].to_vec())
        } else {
            (0..n).map(|k| (s[k] * self.joints[k].drive.ratio(), s[n + k] * self.joints[k].drive.ratio())).unzip()
        }
    }

    fn brakes<'a>(&self, s: &'a [f64]) -> &'a [f64] {
        let n = self.n();
        let at = if self.elastic { 4 * n } else { 2 * n };
        &s[at..at + n]
    }

    /// The process's torques at the joints (`Mechanism::process_torque`).
    fn process(&self, q: &[f64], qd: &[f64]) -> Vec<f64> {
        let mut out = vec![0.0; self.n()];
        self.mechanism.process_torque(q, qd, &mut out);
        out
    }

    /// The driven joints' part of the load with the passive joints held at rest at zero: what the rigid feedforward
    /// and the poses at rest see (the payload as if fixed to the flange).
    fn load(&self, q: &[f64], qd: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let (n, p) = (self.n(), self.passive.len());
        if p == 0 {
            let (mut mass, mut bias) = (vec![0.0; n * n], vec![0.0; n]);
            self.mechanism.dynamics(q, qd, &mut mass, &mut bias);
            return (mass, bias);
        }
        let rest = |x: &[f64]| -> Vec<f64> { x.iter().copied().chain(std::iter::repeat_n(0.0, p)).collect() };
        let (mass, bias) = self.load_all(&rest(q), &rest(qd));
        let all = n + p;
        ((0..n * n).map(|rc| mass[rc / n * all + rc % n]).collect(), bias[..n].to_vec())
    }

    /// The load of all joints, passive ones included.
    fn load_all(&self, q: &[f64], qd: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let all = q.len();
        let (mut mass, mut bias) = (vec![0.0; all * all], vec![0.0; all]);
        self.mechanism.dynamics(q, qd, &mut mass, &mut bias);
        (mass, bias)
    }

    /// Where the passive joints' angles start in the state; their speeds follow them.
    fn passive_at(&self) -> usize {
        self.n() * if self.elastic { 5 } else { 3 }
    }

    /// All joints' positions and speeds, passive ones after the driven ones.
    fn all_joints(&self, s: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let (p, at) = (self.passive.len(), self.passive_at());
        let (q, qd) = self.arms(s);
        (q.iter().chain(&s[at..at + p]).copied().collect(), qd.iter().chain(&s[at + p..at + 2 * p]).copied().collect())
    }

    /// The passive joints' spring and damper torques.
    fn passive_torque(&self, qp: &[f64], qdp: &[f64]) -> Vec<f64> {
        self.passive.iter().zip(qp.iter().zip(qdp)).map(|((k, c), (q, qd))| -k * q - c * qd).collect()
    }

    /// Rigid model: the joints' accelerations (passive ones after the driven ones), and per driven joint the torque
    /// into its gearbox (after the rotor's own acceleration) and out of it, at the joint.
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let (n, p) = (self.n(), self.passive.len());
        let all = n + p;
        let (q, qd) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let (qa, qda) = self.all_joints(s);
        let (mass, bias) = self.load_all(&qa, &qda);
        let springs = self.passive_torque(&qa[n..], &qda[n..]);
        let drive: Vec<f64> = (0..n)
            .map(|k| {
                self.joints[k].drive.rigid_drive(torque[k], rotor[k], rotor_speed[k], brakes[k], self.brake_engaged).0
            })
            .collect();
        let process = self.process(q, qd);
        let rest: Vec<f64> = (0..n)
            .map(|k| self.joints[k].drive.output_torque(q[k], qd[k]) - bias[k] + process[k])
            .chain((0..p).map(|i| springs[i] - bias[n + i]))
            .collect();
        let solve_with = |loss: &[f64]| -> Vec<f64> {
            let mut m = mass.clone();
            for k in 0..n {
                m[k * all + k] += loss[k] * self.joints[k].drive.reflected_inertia();
            }
            let rhs: Vec<f64> = (0..all).map(|k| if k < n { loss[k] * drive[k] } else { 0.0 } + rest[k]).collect();
            solve(&m, &rhs).unwrap_or_else(|| vec![f64::NAN; all])
        };
        let first = solve_with(&vec![1.0; n]);
        let loss: Vec<f64> = (0..n)
            .map(|k| {
                self.joints[k]
                    .drive
                    .gearbox
                    .loss_factor(drive[k] - self.joints[k].drive.reflected_inertia() * first[k], qd[k])
            })
            .collect();
        let acc = solve_with(&loss);
        let into: Vec<f64> = (0..n).map(|k| drive[k] - self.joints[k].drive.reflected_inertia() * acc[k]).collect();
        let out = (0..n).map(|k| loss[k] * into[k]).collect();
        (acc, into, out)
    }

    /// Joint accelerations (passive ones after the driven ones) and the force (torque) each transmission passes to its
    /// joint, in either model.
    fn motion(&self, s: &[f64], torque: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = self.n();
        if self.elastic {
            let mut d = vec![0.0; self.state_len()];
            self.derivatives(s, torque, &mut d);
            let gear = (0..n)
                .map(|k| self.joints[k].drive.gearbox.torque(s[2 * n + k], s[3 * n + k], s[k], s[n + k]).output)
                .collect();
            let at = self.passive_at() + self.passive.len();
            (d[n..2 * n].iter().chain(&d[at..at + self.passive.len()]).copied().collect(), gear)
        } else {
            let (acc, _, out) = self.rigid_solve(s, torque);
            (acc, out)
        }
    }

    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        let n = self.n();
        let (q, qd) = self.all_joints(s);
        let all = q.len();
        let (mass, _) = self.load_all(&q, &vec![0.0; all]);
        let mut kinetic =
            0.5 * (0..all).map(|i| (0..all).map(|j| qd[i] * mass[i * all + j] * qd[j]).sum::<f64>()).sum::<f64>();
        let potential = self.mechanism.potential(&q);
        let mut elastic: f64 = (0..n).map(|k| self.joints[k].drive.stop_energy(q[k])).sum::<f64>()
            + self.passive.iter().zip(&q[n..]).map(|((k, _), q)| 0.5 * k * q * q).sum::<f64>();
        for k in 0..n {
            let j = &self.joints[k].drive;
            if self.elastic {
                kinetic += 0.5 * j.rotor_inertia * s[3 * n + k] * s[3 * n + k];
                elastic += j.gearbox.energy(s[2 * n + k], s[k]);
            } else {
                kinetic += 0.5 * j.reflected_inertia() * qd[k] * qd[k];
            }
        }
        [kinetic, potential, elastic]
    }

    fn losses(&self, s: &[f64], torque: &[f64]) -> Losses {
        let n = self.n();
        let (q, qd) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let p = self.passive.len();
        let (mut powers, mut gear_at_rotor) = (vec![0.0; 3 * n + 2 + p.min(1)], vec![0.0; n]);
        if p > 0 {
            // The swing's dampers.
            let (_, qd) = self.all_joints(s);
            powers[3 * n + 2] = self.passive.iter().zip(&qd[n..]).map(|((_, c), w)| c * w * w).sum();
        }
        let rigid = (!self.elastic).then(|| self.rigid_solve(s, torque));
        for k in 0..n {
            let j = &self.joints[k].drive;
            let [friction, stop, shaft, drag] = j.losses(q[k], qd[k], rotor_speed[k], brakes[k], self.brake_engaged);
            let gear = match &rigid {
                Some((_, into, out)) => {
                    gear_at_rotor[k] = (into[k] - out[k]) / j.ratio();
                    (into[k] - out[k]) * qd[k]
                }
                None => {
                    let g = j.gearbox.torque(s[2 * n + k], s[3 * n + k], s[k], s[n + k]);
                    gear_at_rotor[k] = g.rotor - g.output / j.ratio();
                    j.gearbox.loss_power(s[2 * n + k], s[3 * n + k], s[k], s[n + k])
                }
            };
            powers[3 * k..3 * k + 3].copy_from_slice(&[friction, shaft, gear + drag]);
            powers[3 * n] += stop;
            powers[3 * n + 1] += (torque[k] - j.ripple.apply(torque[k], rotor[k])) * rotor_speed[k];
        }
        if self.mechanism.has_process() {
            powers.push(-self.process(q, qd).iter().zip(qd).map(|(t, v)| t * v).sum::<f64>());
        }
        Losses { powers, gear_at_rotor }
    }

    /// The torque breakdown, loss powers, energies and friction signals of the effect views.
    fn view_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let n = self.n();
        let (_, qd) = self.arms(s);
        let (_, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let (qa, qda) = self.all_joints(s);
        let all = qa.len();
        let (mass, bias) = self.load_all(&qa, &qda);
        let (_, gravity) = self.load_all(&qa, &vec![0.0; all]);
        let (acc, _) = self.motion(s, torque);
        let losses = self.losses(s, torque);
        for k in 0..n {
            let j = &self.joints[k].drive;
            let i = j.ratio();
            let moving: f64 = (0..all).map(|c| mass[k * all + c] * acc[c]).sum();
            let held = j.brake.torque(self.brake_engaged, rotor_speed[k], brakes[k]).0;
            let parts = [
                (moving + bias[k] - gravity[k] + j.reflected_inertia() * acc[k]) / i,
                gravity[k] / i,
                j.shaft.torque(rotor_speed[k]) + held + losses.gear_at_rotor[k] + j.friction.torque(qd[k]) / i,
            ];
            out.extend(parts);
            out.push(torque[k] - parts.iter().sum::<f64>());
        }
        out.extend(&losses.powers);
        out.extend(self.energy_parts(s));
        for k in 0..n {
            let f = &self.joints[k].drive.friction;
            out.extend([qd[k], f.coulomb * (qd[k] / f.smoothing).tanh(), f.viscous * qd[k], 0.0, 1.0]);
        }
    }

    /// The joint's unit and the unit of what drives it: rad and N m, or m and N.
    fn units(&self, k: usize) -> (&'static str, &'static str) {
        if self.joints[k].unit == "m" { ("m/s", "N") } else { ("rad/s", "N m") }
    }

    fn effect_infos(&self, out: &mut Vec<SignalInfo>) {
        for motor in &self.motors {
            out.extend(TORQUE_PARTS.map(|part| {
                SignalInfo::new(
                    format!("motor.{}.torque_{part}", motor.id),
                    motor.key,
                    &format!("quantity-torque-{part}"),
                    "N m",
                )
            }));
        }
        for (k, motor) in self.motors.iter().enumerate() {
            let gear = if self.joints[k].drive.coupled {
                "quantity-coupling-loss-power"
            } else {
                "quantity-gearbox-loss-power"
            };
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}joint_{}", motor.id),
                self.joints[k].key,
                "quantity-friction-power",
                "W",
            ));
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}shaft_{}", motor.id),
                motor.key,
                "quantity-shaft-friction-power",
                "W",
            ));
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}gearbox_{}", motor.id), motor.key, gear, "W"));
        }
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}hard_stops"), "group-joint", "quantity-damping-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}other"), "loss-other", "quantity-power-given-off", "W"));
        // In the order of `losses`: the swing's dampers, then the process.
        if !self.passive.is_empty() {
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}payload_swing"),
                "group-payload-swing",
                "quantity-damping-power",
                "W",
            ));
        }
        if self.mechanism.has_process() {
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}process"),
                "group-process",
                "quantity-power-given-off",
                "W",
            ));
        }
        out.extend(ENERGY_PARTS.map(|part| {
            SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")
        }));
        for (k, contact) in self.friction_contacts().iter().enumerate() {
            let (speed, force) = self.units(k);
            for part in FRICTION_SIGNALS {
                let unit = match part {
                    "speed" => speed,
                    "load_scale" => "",
                    _ => force,
                };
                out.push(SignalInfo::new(
                    format!("{}.friction_{part}", contact.id),
                    contact.owner_key,
                    &format!("quantity-friction-{}", part.replace('_', "-")),
                    unit,
                ));
            }
        }
    }

    /// The lowest mode of a one-joint machine with an elastic transmission: the load (M) and the rotor seen at the joint
    /// (J i^2) on the transmission's spring k, f = sqrt(k (1/M + 1/(J i^2))) / (2 pi). `None` for more joints.
    pub fn two_mass_frequency(&self, q: &[f64]) -> Option<f64> {
        if self.n() != 1 {
            return None;
        }
        let (mass, _) = self.load(q, &[0.0]);
        let j = &self.joints[0].drive;
        Some(
            (j.gearbox.stiffness * (1.0 / mass[0] + 1.0 / j.reflected_inertia())).sqrt() / (2.0 * std::f64::consts::PI),
        )
    }
}

impl<M: Mechanism> Machine for JointMachine<M> {
    /// The mechanism's (`Mechanism::frame_momentum`, passive joints included), each rotor's spin from the state.
    fn frame_momentum(&self, s: &[f64]) -> Option<FrameMomentum> {
        let (q, qd) = self.all_joints(s);
        let spin: Vec<f64> =
            self.rotors(s).1.iter().zip(&self.joints).map(|(w, j)| j.drive.rotor_inertia * w).collect();
        self.mechanism.frame_momentum(&q, &qd, &spin)
    }
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    fn rating_checks(&self) -> Vec<RatingCheck> {
        self.mechanism.rating_checks()
    }
    fn cam(&self) -> Option<sim_core::CamMaster> {
        self.mechanism.cam()
    }
    fn cam_at(&self, master: f64, out: &mut [[f64; 4]]) {
        self.mechanism.cam_at(master, out);
    }

    /// NaN where the pose cannot be reached: the controller never asks for one (see `unreachable`).
    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        let mut q = vec![0.0; self.n()];
        let ok = self.mechanism.inverse(tool, &mut q);
        for k in 0..self.n() {
            motors[k] = if ok { self.joints[k].drive.ratio() * q[k] } else { f64::NAN };
        }
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        let q: Vec<f64> = (0..self.n()).map(|k| motors[k] / self.joints[k].drive.ratio()).collect();
        self.mechanism.forward(&q, tool);
    }

    fn state_len(&self) -> usize {
        self.flex_at() + 2 * self.flex
    }

    /// With bending links, the tool (not the joints' pose) at `tool`: the links bent by their load at rest.
    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let n = self.n();
        let (mut q, mut bent, mut pose) = (vec![0.0; n], vec![0.0; self.flex], tool.to_vec());
        for _ in 0..if self.flex > 0 { 3 } else { 1 } {
            if !self.mechanism.inverse(&pose, &mut q) {
                q.fill(f64::NAN);
            }
            if self.flex > 0 {
                let mut load = vec![0.0; self.flex];
                self.mechanism.flex_loads(&q, &vec![0.0; n], &vec![0.0; n], &mut load);
                let modes = self.mechanism.flex_modes();
                bent = load.iter().zip(&modes).map(|(l, m)| l / m.stiffness).collect();
                let shift = self.flex_shift(&q, &bent);
                pose = tool.iter().zip(&shift).map(|(t, s)| t - s).collect();
            }
        }
        let (_, gravity) = self.load(&q, &vec![0.0; n]);
        state.clear();
        state.extend(&q);
        state.extend(vec![0.0; n]);
        for k in 0..n {
            let j = &self.joints[k].drive;
            // At rest the transmission holds the load against gravity and the stops.
            let out = gravity[k] - j.output_torque(q[k], 0.0);
            let rotor = j.ratio() * (q[k] + if self.elastic { j.gearbox.static_twist(out) } else { 0.0 });
            if self.elastic {
                state.push(rotor);
            }
            torque[k] = j.ripple.invert(out / j.ratio(), rotor);
        }
        state.resize(self.flex_at(), 0.0);
        state.extend(bent);
        state.resize(self.state_len(), 0.0);
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let n = self.n();
        let (q, qd) = self.arms(s);
        out[..n].copy_from_slice(qd);
        let (p, at) = (self.passive.len(), self.passive_at());
        out[at..at + p].copy_from_slice(&s[at + p..at + 2 * p]);
        if !self.elastic {
            let (acc, ..) = self.rigid_solve(s, torque);
            let (_, rotor_speed) = self.rotors(s);
            let brakes = self.brakes(s);
            out[n..2 * n].copy_from_slice(&acc[..n]);
            out[at + p..at + 2 * p].copy_from_slice(&acc[n..]);
            for k in 0..n {
                out[2 * n + k] = self.joints[k].drive.brake.torque(self.brake_engaged, rotor_speed[k], brakes[k]).1;
            }
            return self.flex_rates(s, &acc, out);
        }
        let (qa, qda) = self.all_joints(s);
        let (mass, bias) = self.load_all(&qa, &qda);
        let springs = self.passive_torque(&qa[n..], &qda[n..]);
        let process = self.process(q, qd);
        let mut rhs: Vec<f64> = vec![0.0; n].into_iter().chain((0..p).map(|i| springs[i] - bias[n + i])).collect();
        for k in 0..n {
            let j = &self.joints[k].drive;
            let (gear, rotor_acc, brake_rate) =
                j.rotor(torque[k], s[2 * n + k], s[3 * n + k], q[k], qd[k], s[4 * n + k], self.brake_engaged);
            rhs[k] = gear.output - bias[k] + j.output_torque(q[k], qd[k]) + process[k];
            out[2 * n + k] = s[3 * n + k];
            out[3 * n + k] = rotor_acc;
            out[4 * n + k] = brake_rate;
        }
        let acc = solve(&mass, &rhs).unwrap_or_else(|| vec![f64::NAN; n + p]);
        out[n..2 * n].copy_from_slice(&acc[..n]);
        out[at + p..at + 2 * p].copy_from_slice(&acc[n..]);
        self.flex_rates(s, &acc, out);
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (q, qd) = self.arms(s);
        self.mechanism.forward(q, pos);
        let jac = self.jacobian(q);
        let n = self.n();
        for (i, v) in vel.iter_mut().enumerate().take(self.axes.len()) {
            *v = (0..n).map(|k| jac[i * n + k] * qd[k]).sum();
        }
        if self.flex > 0 {
            let (d, rate) = self.flex_state(s);
            pos.iter_mut().zip(self.flex_shift(q, d)).for_each(|(p, d)| *p += d);
            vel.iter_mut().zip(self.flex_shift(q, rate)).for_each(|(v, d)| *v += d);
        }
    }
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        let n = self.n();
        let mut pos = vec![0.0; self.axes.len()];
        self.mechanism.forward(&s[..n], &mut pos);
        let zero = vec![0.0; pos.len()];
        let (_, qd, _) = self.joint_motion(&pos, vel, &zero);
        s[n..2 * n].copy_from_slice(&qd);
        if self.elastic {
            for k in 0..n {
                s[3 * n + k] = self.joints[k].drive.ratio() * qd[k];
            }
        }
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (rotor, speed) = self.rotors(s);
        pos[..self.n()].copy_from_slice(&rotor);
        vel[..self.n()].copy_from_slice(&speed);
    }

    /// Rigid, loss-free, at rest: (M q'' + g) at the joint over the ratio, plus the rotor's own J i q'', with q'' what
    /// the tool's acceleration asks at rest.
    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        let zero = vec![0.0; pos.len()];
        let (q, qd, qdd) = self.joint_motion(pos, &zero, acc);
        let motor = |x: &[f64]| -> Vec<f64> { (0..self.n()).map(|k| x[k] * self.joints[k].drive.ratio()).collect() };
        self.rigid_motor_torque(&motor(&q), &motor(&qd), &motor(&qdd), out);
    }

    fn rigid_guide_loads(&self, pos: &[f64], acc: &[f64], out: &mut Vec<[f64; 2]>) {
        self.mechanism.rigid_guide_loads(pos, acc, out);
    }

    /// The joints' motion (`joint_motion`), then `rigid_motor_torque` with the velocity terms: the Coriolis and
    /// centripetal torques of M(q) changing with the pose, in joint space too.
    fn feedforward_torque_moving(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut [f64]) {
        let (q, qd, qdd) = self.joint_motion(pos, vel, acc);
        let motor = |x: &[f64]| -> Vec<f64> { (0..self.n()).map(|k| x[k] * self.joints[k].drive.ratio()).collect() };
        self.rigid_motor_torque(&motor(&q), &motor(&qd), &motor(&qdd), out);
    }

    /// Exact in joint coordinates, velocity terms included (`feedforward_torque` leaves them out): M q'' + h at the
    /// joint over the ratio, plus the rotor's own J_rotor i q''.
    fn rigid_motor_torque(&self, angle: &[f64], speed: &[f64], acc: &[f64], out: &mut [f64]) {
        self.rigid_split_torque(angle, speed, acc, acc, out);
    }

    /// Elastic: the gearbox's static twist under what reaches it, (torque − J_rotor acc) i at the joint (ripple, motor
    /// friction and the gearbox's losses and damping left out), times the ratio. Zeros where the joints are rigid.
    fn twist(&self, torque: &[f64], acc: &[f64], out: &mut [f64]) {
        for (k, joint) in self.joints.iter().enumerate() {
            let j = &joint.drive;
            out[k] = if self.elastic {
                j.ratio() * j.gearbox.static_twist((torque[k] - j.rotor_inertia * acc[k]) * j.ratio())
            } else {
                0.0
            };
        }
    }

    fn rigid_split_torque(&self, angle: &[f64], speed: &[f64], acc: &[f64], rotor_acc: &[f64], out: &mut [f64]) {
        let n = self.n();
        let joint = |x: &[f64]| -> Vec<f64> { (0..n).map(|k| x[k] / self.joints[k].drive.ratio()).collect() };
        let (q, qd, qdd) = (joint(angle), joint(speed), joint(acc));
        let (mass, bias) = self.load(&q, &qd);
        for k in 0..n {
            let j = &self.joints[k].drive;
            let at_joint: f64 = (0..n).map(|c| mass[k * n + c] * qdd[c]).sum::<f64>() + bias[k];
            out[k] = at_joint / j.ratio() + j.rotor_inertia * rotor_acc[k];
        }
    }

    fn energy(&self, s: &[f64]) -> f64 {
        self.energy_parts(s).iter().sum()
    }

    fn set_payload_attached(&mut self, attached: bool) {
        self.mechanism.set_payload_attached(attached);
    }
    fn links(&self) -> Vec<String> {
        self.mechanism.links()
    }
    fn set_link_change(&mut self, link: usize, factor: f64, moment: [f64; 3]) {
        self.mechanism.set_link_change(link, factor, moment);
    }
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.brake_engaged = engaged;
    }

    /// The gearboxes, in the order of `thermal_parts`.
    /// The gearboxes' hysteresis states move on to where the step ended (`Gearbox::commit`).
    fn after_step(&mut self, s: &mut [f64], _dt: f64) {
        let n = self.n();
        if self.elastic {
            for k in 0..n {
                self.joints[k].drive.gearbox.commit(s[2 * n + k], s[k]);
            }
        }
    }
    /// The mechanism's, then the gearboxes' hysteresis anchors.
    fn memory(&self) -> Vec<f64> {
        let gears = crate::mech::Gearbox::anchors(self.joints.iter().map(|j| &j.drive.gearbox));
        [self.mechanism.memory(), gears].concat()
    }
    fn set_memory(&mut self, memory: &[f64]) {
        let (own, gears) = memory.split_at(self.mechanism.memory().len().min(memory.len()));
        self.mechanism.set_memory(own);
        crate::mech::Gearbox::set_anchors(self.joints.iter_mut().map(|j| &mut j.drive.gearbox), gears)
    }

    fn set_part_temperatures(&mut self, rises: &[f64]) {
        let Some(law) = &self.thermal else { return };
        let geared = self.joints.iter_mut().filter(|j| !j.drive.coupled);
        for (joint, rise) in geared.zip(rises) {
            joint.drive.gearbox.efficiency = law.gearbox_efficiency(*rise) * joint.drive.extra_efficiency;
            joint.drive.shaft.set_oil(law.viscosity_factor(*rise));
        }
    }

    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        let mut q = vec![0.0; self.n()];
        self.mechanism.inverse(tool_pose, &mut q).then(|| self.two_mass_frequency(&q)).flatten()
    }

    /// The two-mass mode of `resonance_hz` and the links' bending modes (`Mechanism::flex_modes`), lowest first.
    fn modes(&self, tool_pose: &[f64]) -> Vec<sim_core::Mode> {
        let mut out: Vec<sim_core::Mode> =
            self.resonance_hz(tool_pose).map(|hz| sim_core::Mode { hz, damping: 0.0 }).into_iter().collect();
        out.extend(self.mechanism.flex_modes().iter().map(|m| {
            let w = (m.stiffness / m.mass).sqrt();
            sim_core::Mode { hz: w / std::f64::consts::TAU, damping: m.damping / (2.0 * m.mass * w) }
        }));
        out.sort_by(|a, b| a.hz.total_cmp(&b.hz));
        out
    }

    fn setpoint_slack(&self) -> f64 {
        self.rest_slack
    }

    fn condition(&self, tool: &[f64]) -> Option<f64> {
        self.mechanism.condition(tool)
    }

    fn unreachable(&self, tool: &[f64], slack: f64) -> Option<&'static str> {
        if let Some(why) = self.mechanism.unreachable(tool, slack) {
            return Some(why);
        }
        let mut q = vec![0.0; self.n()];
        if !self.mechanism.inverse(tool, &mut q) {
            return Some("error-target-out-of-range");
        }
        // A joint beyond its stops, by more than `slack` (taken in the joint's unit: where the tool is the joints, as on
        // the arms and slides, that is the controller's slack; an arm's gears twist under its weight beyond a stop).
        let beyond = self
            .joints
            .iter()
            .zip(&q)
            .any(|(j, q)| *q < j.drive.range.0 - slack - 1e-9 || *q > j.drive.range.1 + slack + 1e-9);
        beyond.then_some("error-joint-beyond-stop")
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out = Vec::new();
        for (k, motor) in self.motors.iter().enumerate() {
            let unit = self.joints[k].unit;
            out.push(SignalInfo::new(
                format!("joint.{}.position", self.joints[k].id),
                self.joints[k].key,
                "quantity-joint-position",
                unit,
            ));
            out.push(SignalInfo::new(
                format!("joint.{}.force", self.joints[k].id),
                self.joints[k].key,
                "quantity-joint-force",
                self.units(k).1,
            ));
            if self.joints[k].gear_lever.is_some() || self.joints[k].unit != "m" {
                out.push(SignalInfo::new(
                    format!("gearbox.{}.output_torque", motor.id),
                    motor.key,
                    "quantity-gearbox-torque",
                    "N m",
                ));
            }
        }
        for k in 0..self.passive.len() {
            out.push(SignalInfo::new(
                format!("swing.{}.angle", k + 1),
                "group-payload-swing",
                "quantity-swing-angle",
                "rad",
            ));
        }
        out.extend(self.mechanism.signal_infos());
        out.extend(self.mechanism.flex_modes().iter().map(|m| {
            SignalInfo::new(format!("link.{}.bend", m.link + 1), JOINT_KEYS[m.link], "quantity-link-bend", "m")
        }));
        self.effect_infos(&mut out);
        out
    }

    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (q, qd) = self.arms(s);
        let (acc, force) = self.motion(s, torque);
        for k in 0..self.n() {
            out.extend([q[k], force[k]]);
            match self.joints[k].gear_lever {
                // Ahead of the converter: more than the joint's force times the lever while it drives, less while the
                // load drives it back.
                Some(lever) => {
                    let e = self.joints[k].drive.extra_efficiency;
                    out.push(force[k] * lever * if force[k] * qd[k] >= 0.0 { 1.0 / e } else { e });
                }
                None if self.joints[k].unit != "m" => out.push(force[k]),
                None => {}
            }
        }
        let at = self.passive_at();
        out.extend(&s[at..at + self.passive.len()]);
        self.mechanism.signals(q, qd, &acc, &force, out);
        out.extend_from_slice(self.flex_state(s).0);
    }
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.view_signals(s, torque, out);
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(s, torque).powers);
    }

    /// Each joint's friction (Coulomb and viscous, no stiction), owned by the joint's axis: the friction view spans that
    /// axis's speed limit (with the joint's own key it found no axis and spanned 1 m/s or rad/s).
    fn friction_contacts(&self) -> Vec<FrictionContact> {
        self.joints
            .iter()
            .zip(&self.axes)
            .map(|(j, axis)| FrictionContact { id: j.id_signal(), owner_key: axis.key })
            .collect()
    }
    fn friction_curve(&self, i: usize, speed: f64, _load_scale: f64) -> [f64; 3] {
        let Some(joint) = self.joints.get(i) else { return [0.0; 3] };
        let f = &joint.drive.friction;
        [f.coulomb * (speed / f.smoothing).tanh(), f.viscous * speed, 0.0]
    }

    /// The gearboxes, where there are gearboxes (not on a direct drive).
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        self.joints
            .iter()
            .zip(&self.motors)
            .filter(|(j, _)| !j.drive.coupled)
            .enumerate()
            .map(|(k, (_, m))| ThermalPart {
                id: GEARBOX_IDS[MOTOR_IDS.iter().position(|id| *id == m.id).unwrap_or(0)],
                kind: "gearbox",
                owner_key: m.key,
                scene_part: "gearbox",
                scene_index: k,
            })
            .collect()
    }

    fn scene(&self, s: &[f64], out: &mut Scene) {
        let (q, _) = self.all_joints(s);
        let (rotor, speed) = self.rotors(s);
        out.parts.clear();
        out.labels.clear();
        self.mechanism.scene(&q, &rotor, &speed, out);
    }
    fn scene_point(&self, tool: &[f64]) -> Option<[f64; 3]> {
        self.mechanism.scene_point(tool)
    }

    /// The boxes of the scene that the mechanism names (`Mechanism::collision_parts`).
    fn collision_bodies(&self, s: &[f64]) -> Vec<sim_core::Aabb> {
        let mut scene = Scene::default();
        self.scene(s, &mut scene);
        sim_core::scene::scene_boxes(&scene.parts, self.mechanism.collision_parts())
    }
}

impl<M: Mechanism> JointMachine<M> {
    /// The joints' positions, speeds and accelerations for the tool's pose, speed and acceleration. Where the tool is
    /// the joints, those themselves. With as many axes as joints: q' = J^-1 v and q'' = J^-1 (a - J' q'), J' q' the
    /// second difference of the forward kinematics along q'. With fewer (a redundant arm): by central differences of
    /// the mechanism's `inverse`, which resolves the redundancy, along v and along a, so that speeds and accelerations
    /// are those of the very joints it gives: q' = (q(x + h v) - q(x - h v)) / 2h, q'' = (q(x + h v) - 2 q(x) +
    /// q(x - h v)) / h^2 + (q(x + k a) - q(x - k a)) / 2k, steps of 1e-4 of the tool's units.
    pub fn joint_motion(&self, pos: &[f64], vel: &[f64], acc: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let (n, m) = (self.n(), self.axes.len());
        if self.mechanism.tool_is_joints() {
            return (pos[..n].to_vec(), vel[..n].to_vec(), acc[..n].to_vec());
        }
        let solved = |x: &[f64]| -> Vec<f64> {
            let mut q = vec![0.0; n];
            if !self.mechanism.inverse(x, &mut q) {
                q.fill(f64::NAN);
            }
            q
        };
        let q = solved(pos);
        if m < n {
            let along = |d: &[f64]| -> Option<(f64, Vec<f64>, Vec<f64>)> {
                let size = d.iter().map(|v| v * v).sum::<f64>().sqrt();
                (size > 0.0).then(|| {
                    let h = 1e-4 / size;
                    let at = |s: f64| solved(&(0..m).map(|i| pos[i] + s * d[i]).collect::<Vec<_>>());
                    (h, at(h), at(-h))
                })
            };
            let (mut qd, mut qdd) = (vec![0.0; n], vec![0.0; n]);
            if let Some((h, ahead, behind)) = along(&vel[..m]) {
                for k in 0..n {
                    qd[k] = (ahead[k] - behind[k]) / (2.0 * h);
                    qdd[k] = (ahead[k] - 2.0 * q[k] + behind[k]) / (h * h);
                }
            }
            if let Some((h, ahead, behind)) = along(&acc[..m]) {
                for k in 0..n {
                    qdd[k] += (ahead[k] - behind[k]) / (2.0 * h);
                }
            }
            return (q, qd, qdd);
        }
        let jac = self.jacobian(&q);
        let qd = solve(&jac, &vel[..n]).unwrap_or_else(|| vec![f64::NAN; n]);
        let speed = qd.iter().map(|v| v * v).sum::<f64>().sqrt();
        let mut curve = vec![0.0; n];
        if speed > 0.0 {
            // A step of 1e-4 rad (m) along q': truncation ~1e-8, rounding ~1e-16 / 1e-8 of the tool's size.
            let h = 1e-4 / speed;
            let (mut a, mut b, mut c) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
            let at = |s: f64| -> Vec<f64> { (0..n).map(|k| q[k] + s * qd[k]).collect() };
            self.mechanism.forward(&at(h), &mut a);
            self.mechanism.forward(&q, &mut b);
            self.mechanism.forward(&at(-h), &mut c);
            curve = (0..n).map(|k| (a[k] - 2.0 * b[k] + c[k]) / (h * h)).collect();
        }
        let rhs: Vec<f64> = (0..n).map(|k| acc[k] - curve[k]).collect();
        let qdd = solve(&jac, &rhs).unwrap_or_else(|| vec![f64::NAN; n]);
        (q, qd, qdd)
    }

    /// d tool / d q (axes x joints, row by row), by central differences of `Mechanism::forward`; exact (to rounding)
    /// where the tool is the joints.
    pub fn jacobian(&self, q: &[f64]) -> Vec<f64> {
        let (n, rows) = (self.n(), self.axes.len());
        // Where the tool is the joints, the identity to the bit: the difference quotient of the copy is off by up to
        // 1e-10, and a model switch that rebuilt the joints changed the tool's speed by that.
        if self.mechanism.tool_is_joints() {
            return (0..rows * n).map(|i| if i / n == i % n { 1.0 } else { 0.0 }).collect();
        }
        let (mut jac, mut a, mut b) = (vec![0.0; rows * n], vec![0.0; rows], vec![0.0; rows]);
        let mut p = q.to_vec();
        for c in 0..n {
            let h = 1e-6 * q[c].abs().max(1.0);
            p[c] = q[c] + h;
            self.mechanism.forward(&p, &mut a);
            p[c] = q[c] - h;
            self.mechanism.forward(&p, &mut b);
            p[c] = q[c];
            for r in 0..rows {
                jac[r * n + c] = (a[r] - b[r]) / (2.0 * h);
            }
        }
        jac
    }
}

/// Motor ids "1" to "7" and the ids of their gearboxes' loss powers, so that `ThermalPart` gets `&'static str`s.
pub const MOTOR_IDS: [&str; 7] = ["1", "2", "3", "4", "5", "6", "7"];
const GEARBOX_IDS: [&str; 7] =
    ["gearbox_1", "gearbox_2", "gearbox_3", "gearbox_4", "gearbox_5", "gearbox_6", "gearbox_7"];
/// Translation keys of motors 1 to 7 and of joints 1 to 7.
pub const MOTOR_KEYS: [&str; 7] = [
    "motor-joint-1",
    "motor-joint-2",
    "motor-joint-3",
    "motor-joint-4",
    "motor-joint-5",
    "motor-joint-6",
    "motor-joint-7",
];
pub const JOINT_KEYS: [&str; 7] = ["joint-1", "joint-2", "joint-3", "joint-4", "joint-5", "joint-6", "joint-7"];
const JOINT_SIGNALS: [&str; 7] = ["joint.1", "joint.2", "joint.3", "joint.4", "joint.5", "joint.6", "joint.7"];

impl JointSpec {
    /// "joint.<id>" for the friction contact, where the id is one of `MOTOR_IDS`.
    fn id_signal(&self) -> &'static str {
        JOINT_SIGNALS[MOTOR_IDS.iter().position(|id| *id == self.id).unwrap_or(0)]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gaussian_elimination_solves_and_refuses_singular() {
        let x = solve(&[0.0, 2.0, 1.0, 1.0], &[4.0, 3.0]).unwrap();
        assert!((x[0] - 1.0).abs() < 1e-12 && (x[1] - 2.0).abs() < 1e-12);
        assert!(solve(&[1.0, 2.0, 2.0, 4.0], &[1.0, 2.0]).is_none());
    }
}
