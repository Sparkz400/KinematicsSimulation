//! A serial arm whose bodies, joints, masses and inertias come from a URDF file (`rigid::Chain`), its dynamics by the
//! recursive Newton-Euler algorithm. The tool pose is the joint positions themselves (rad, m for a prismatic joint):
//! the cycle is given in joints, and the sizing judges motors, gears and screws along it. With
//! `kinematics.tool_space = true` it is the flange's pose instead (`tool`): x, y, z and c on the SCARA, x, y, z and the
//! rotation vector rx, ry, rz on a six-joint arm, each axis's limits in `[axis.x]`, ..., its range from
//! `workspace.tool_min` and `tool_max`; poses near a singularity (`workspace.max_condition_number`) are refused.
//!
//! Config `[arm]`: `urdf_file` (a path, or "builtin:six-axis" / "builtin:scara" for the files in `urdf/`),
//! `gear_ratios` (one per joint, before any screw), `payload_kg` and `payload_offset_m` (from the URDF's tip frame).
//! Each joint's limits are `[axis.j1]`, `[axis.j2]`, ...: `max_velocity_rad_s` and so on (`_m_` for a prismatic joint). Every joint has
//! the `[motor]` and the `[gearbox]` (with its own ratio); a revolute joint the `[joint]` friction and stops (its stops
//! from the URDF's limits where it gives them). A prismatic joint is a `[screw]` coupled to its motor (ratio 1; the
//! `[gearbox]`'s stiffness and damping stand for the coupling, its efficiency and inertia do not count) with the
//! `[slide]` friction and stops. With a
//! `[strain_wave]` table, each revolute joint's gear is judged by its life.
//!
//! shortcut: every joint has the same motor, drive and gearbox but its ratio, as `drive.rs` reads one `[motor]` and
//! `[drive]` per machine; give joints their own tables once the drive reads one per motor.

use crate::tool::{Scara, Six, Tool};
use crate::{axis, gravity, machine, payload, required_life};
use sim_components::chain::{JOINT_KEYS, MOTOR_IDS, MOTOR_KEYS, Mechanism};
use sim_components::joint::RotaryJoint;
use sim_components::rigid::{Chain, V3, mul};
use sim_components::transmission::{BallScrew, StrainWave};
use sim_core::{Config, ConfigError, Label, Machine, Part, RatingCheck, Scene, Shape, SignalInfo};
use std::f64::consts::{PI, TAU};

const BUILTIN: [(&str, &str); 2] = [
    ("builtin:six-axis", include_str!("../urdf/six-axis.urdf")),
    ("builtin:scara", include_str!("../urdf/scara.urdf")),
];

const AXIS_IDS: [&str; 7] = ["j1", "j2", "j3", "j4", "j5", "j6", "j7"];
/// The tool's axes in tool space: the SCARA's, the six-joint arm's.
const SCARA_AXES: [(&str, &str); 4] = [("x", "axis-x"), ("y", "axis-y"), ("z", "axis-z"), ("c", "axis-tool-c")];
const SIX_AXES: [(&str, &str); 6] =
    [("x", "axis-x"), ("y", "axis-y"), ("z", "axis-z"), ("rx", "axis-rx"), ("ry", "axis-ry"), ("rz", "axis-rz")];

pub struct Arm {
    chain: Chain,
    g: f64,
    /// The payload's mass and where it sits in the last body's frame.
    payload: (f64, V3),
    attached: bool,
    /// Per joint: motor radians per joint unit.
    ratios: Vec<f64>,
    screw: Option<BallScrew>,
    strain_wave: Option<StrainWave>,
    required_life: Option<f64>,
    tool: Tool,
    /// `workspace.max_condition_number`: tool poses beyond it are refused.
    max_condition: f64,
}

/// The chain of `arm.urdf_file`.
pub fn chain(cfg: &Config) -> Result<Chain, ConfigError> {
    let name =
        cfg.get("arm.urdf_file")?.as_str().ok_or_else(|| ConfigError::Invalid("arm.urdf_file".into()))?.to_string();
    let text = match BUILTIN.iter().find(|(n, _)| *n == name) {
        Some((_, text)) => text.to_string(),
        None => {
            std::fs::read_to_string(&name).map_err(|e| ConfigError::Invalid(format!("arm.urdf_file: {name}: {e}")))?
        }
    };
    Chain::from_urdf(&text)
}

pub fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    let chain = chain(cfg)?;
    let n = chain.len();
    if n == 0 || n > MOTOR_IDS.len() {
        return Err(ConfigError::Invalid("arm.urdf_file".into()));
    }
    let per_joint = |key: &str| -> Result<Vec<f64>, ConfigError> {
        let v = cfg.f64s(&format!("arm.{key}"))?;
        if v.len() != n || v.iter().any(|x| !(*x > 0.0)) {
            return Err(ConfigError::Invalid(format!("arm.{key}")));
        }
        Ok(v)
    };
    let ratios = per_joint("gear_ratios")?;
    // shortcut: the gearbox life takes every joint's speed through `gearbox.ratio` (analysis::output_rpm), so it is
    // refused where the joints' ratios differ; report each motor's ratio to the analysis to allow it.
    if cfg.f64("ratings.gearbox_reference_speed_rpm").is_ok()
        && ratios.iter().any(|r| Some(*r) != cfg.f64("gearbox.ratio").ok())
    {
        return Err(ConfigError::Invalid("ratings.gearbox_reference_speed_rpm".into()));
    }
    let screw = if chain.bodies.iter().any(|b| b.prismatic) { Some(BallScrew::from_config(cfg)?) } else { None };
    let strain_wave = if cfg.get("strain_wave").is_ok() { Some(StrainWave::from_config(cfg)?) } else { None };
    let (mut joints, mut axes) = (Vec::new(), Vec::new());
    for (k, body) in chain.bodies.iter().enumerate() {
        let (mut joint, lever) = match (&screw, body.prismatic) {
            (Some(screw), true) => {
                // The screw sits on the motor's shaft through a coupling: no gear, no gear losses or inertia.
                if ratios[k] != 1.0 {
                    return Err(ConfigError::Invalid("arm.gear_ratios".into()));
                }
                let mut joint = RotaryJoint::linear_from_config(cfg)?;
                joint.gearbox.ratio = 1.0;
                joint.gearbox.efficiency = 1.0;
                joint.coupled = true;
                joint.rotor_inertia = cfg.positive("motor.rotor_inertia_kgm2")?;
                screw.converter(screw.softest_position()).apply(cfg, &mut joint)?;
                if cfg.bool("effects.joint_friction")? {
                    joint.friction.coulomb += screw.preload_torque() * screw.ratio();
                }
                (joint, Some(1.0 / screw.ratio()))
            }
            _ => {
                let mut joint = RotaryJoint::from_config(cfg)?;
                joint.gearbox.ratio = ratios[k];
                joint.coupled = ratios[k] == 1.0;
                (joint, None)
            }
        };
        if let Some(range) = body.limits {
            joint.range = range;
        }
        let unit = if body.prismatic { "m" } else { "rad" };
        axes.push(axis(cfg, AXIS_IDS[k], JOINT_KEYS[k], unit, joint.range)?);
        joints.push((joint, lever, unit));
    }
    let tool = if !cfg.bool("kinematics.tool_space")? {
        Tool::Joints
    } else {
        let refuse = || ConfigError::Invalid("kinematics.tool_space".into());
        let (ids, tool): (&[(&'static str, &'static str)], Tool) = if n == 4 {
            let elbow = match cfg.get("kinematics.elbow")?.as_str() {
                Some("right") => 1.0,
                Some("left") => -1.0,
                _ => return Err(ConfigError::Invalid("kinematics.elbow".into())),
            };
            (&SCARA_AXES, Tool::Scara(Scara::new(&chain, elbow).ok_or_else(refuse)?))
        } else {
            let home = cfg.f64s("kinematics.home_joints")?;
            (&SIX_AXES, Tool::Six(Six::new(&chain, home).ok_or_else(refuse)?))
        };
        let (min, max) = (cfg.f64s("workspace.tool_min")?, cfg.f64s("workspace.tool_max")?);
        if min.len() != n || max.len() != n || min.iter().zip(&max).any(|(a, b)| !(a < b)) {
            return Err(ConfigError::Invalid("workspace.tool_min".into()));
        }
        axes = ids
            .iter()
            .enumerate()
            .map(|(k, (id, key))| axis(cfg, id, key, if k < 3 { "m" } else { "rad" }, (min[k], max[k])))
            .collect::<Result<_, _>>()?;
        tool
    };
    let offset = cfg.f64s("arm.payload_offset_m")?;
    let &[x, y, z] = &offset[..] else { return Err(ConfigError::Invalid("arm.payload_offset_m".into())) };
    let at = sim_components::rigid::add(chain.tip_pos, mul(&chain.tip_rot, [x, y, z]));
    let arm = Arm {
        payload: (payload(cfg, "arm.payload_kg")?, at),
        g: gravity(cfg)?,
        attached: true,
        ratios: joints.iter().map(|(j, ..)| j.ratio()).collect(),
        screw,
        strain_wave,
        required_life: required_life(cfg),
        max_condition: cfg.positive("workspace.max_condition_number")?,
        tool,
        chain,
    };
    machine(cfg, arm, joints, axes)
}

impl Arm {
    fn extra(&self) -> Option<(f64, V3)> {
        (self.attached && self.payload.0 > 0.0).then_some(self.payload)
    }

    /// The joints at a tool pose and their Jacobian's condition number there (`tool::Scara::condition`,
    /// `tool::Six::condition`); `None` out of reach.
    fn solved(&self, tool: &[f64]) -> Option<(Vec<f64>, f64)> {
        let mut q = vec![0.0; self.chain.len()];
        match &self.tool {
            Tool::Joints => None,
            Tool::Scara(s) => s.inverse(tool, &mut q).then(|| (q.clone(), s.condition(&q))),
            Tool::Six(s) => s.inverse(&self.chain, tool, &mut q).then(|| (q.clone(), s.condition(&self.chain, &q))),
        }
    }
}

impl Mechanism for Arm {
    fn dynamics(&self, q: &[f64], qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        self.chain.dynamics(q, qd, self.g, self.extra(), mass, bias);
    }
    fn potential(&self, q: &[f64]) -> f64 {
        self.chain.potential(q, self.g, self.extra())
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.attached = attached;
    }

    fn forward(&self, q: &[f64], tool: &mut [f64]) {
        match &self.tool {
            Tool::Joints => tool.copy_from_slice(q),
            Tool::Scara(s) => s.forward(q, tool),
            Tool::Six(_) => Six::forward(&self.chain, q, tool),
        }
    }
    fn inverse(&self, tool: &[f64], q: &mut [f64]) -> bool {
        match &self.tool {
            Tool::Joints => {
                q.copy_from_slice(tool);
                true
            }
            Tool::Scara(s) => s.inverse(tool, q),
            Tool::Six(s) => s.inverse(&self.chain, tool, q),
        }
    }
    /// In tool space: out of reach, a turn of the flange near half a turn from the base frame's (where its rotation
    /// vector jumps), or too near a singularity; the joints' stops are the joint machine's to check.
    fn unreachable(&self, tool: &[f64], _slack: f64) -> Option<&'static str> {
        if matches!(self.tool, Tool::Joints) {
            return None;
        }
        if matches!(self.tool, Tool::Six(_)) && tool[3..6].iter().map(|v| v * v).sum::<f64>().sqrt() > PI - 0.01 {
            return Some("error-arm-orientation-half-turn");
        }
        match self.solved(tool) {
            None => Some("error-arm-out-of-reach"),
            Some((_, c)) if !(c <= self.max_condition) => Some("error-arm-singularity"),
            Some(_) => None,
        }
    }
    fn condition(&self, tool: &[f64]) -> Option<f64> {
        match self.tool {
            Tool::Joints => None,
            _ => Some(self.solved(tool).map_or(f64::INFINITY, |(_, c)| c)),
        }
    }
    fn tool_is_joints(&self) -> bool {
        matches!(self.tool, Tool::Joints)
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out = Vec::new();
        for (k, body) in self.chain.bodies.iter().enumerate() {
            match (body.prismatic, &self.screw, &self.strain_wave) {
                (true, Some(s), _) => out.extend(s.signal_infos(MOTOR_IDS[k], JOINT_KEYS[k])),
                (false, _, Some(g)) => out.push(g.signal_info(MOTOR_IDS[k], JOINT_KEYS[k])),
                _ => {}
            }
        }
        out
    }
    fn signals(&self, _q: &[f64], qd: &[f64], _qdd: &[f64], force: &[f64], out: &mut Vec<f64>) {
        for (k, body) in self.chain.bodies.iter().enumerate() {
            match (body.prismatic, &self.screw, &self.strain_wave) {
                (true, Some(s), _) => s.signals(qd[k], force[k], out),
                (false, _, Some(g)) => out.push(g.signal(force[k], qd[k] * self.ratios[k])),
                _ => {}
            }
        }
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        let mut out = Vec::new();
        for (k, body) in self.chain.bodies.iter().enumerate() {
            match (body.prismatic, &self.screw, &self.strain_wave, self.required_life) {
                (true, Some(s), ..) => out.extend(s.rating_checks(MOTOR_IDS[k], k, self.required_life)),
                (false, _, Some(g), Some(life)) => out.push(g.rating_check(MOTOR_IDS[k], k, life)),
                _ => {}
            }
        }
        out
    }

    fn collision_parts(&self) -> &'static [&'static str] {
        &["flange", "payload"]
    }

    /// Each body as a bar from its joint to the next, each joint with its motor's shaft bar and speed; the flange and
    /// the payload as boxes, for the collision with obstacles.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene) {
        let frames = self.chain.frames(q);
        let (_, tip) = self.chain.tip(q);
        out.parts.push(Part::new(
            "base",
            Shape::Box { size: [0.3, 0.3, 0.05] },
            [0.0, 0.0, -0.025],
            [0.62, 0.64, 0.68],
        ));
        let mut from = [0.0; 3];
        for (k, (_, at)) in frames.iter().enumerate() {
            out.parts.push(Part::new("link", Shape::Segment { to: *at, width: 0.05 }, from, [0.2, 0.22, 0.26]));
            out.parts.push(Part::new(
                sim_core::scene::MOTOR_PART,
                Shape::Cylinder { radius: 0.04, length: 0.08 },
                *at,
                [0.9, 0.45, 0.1],
            ));
            out.parts.push(
                Part::new(
                    "rotor-bar",
                    Shape::Box { size: [0.06, 0.004, 0.01] },
                    [at[0], at[1] - 0.05, at[2]],
                    [0.1, 0.1, 0.1],
                )
                .turned(rotors[k]),
            );
            if !self.chain.bodies[k].prismatic {
                out.parts.push(Part::new(
                    "gearbox",
                    Shape::Box { size: [0.07, 0.07, 0.05] },
                    [at[0], at[1], at[2] - 0.06],
                    [0.25, 0.25, 0.28],
                ));
            }
            out.labels.push(Label {
                pos: [at[0], at[1], at[2] + 0.08],
                owner_key: MOTOR_KEYS[k],
                value: rotor_speeds[k] * 60.0 / TAU,
                unit_key: "unit-rpm",
                decimals: 0,
            });
            from = *at;
        }
        out.parts.push(Part::new("link", Shape::Segment { to: tip, width: 0.03 }, from, [0.2, 0.22, 0.26]));
        out.parts.push(Part::new("flange", Shape::Box { size: [0.06, 0.06, 0.06] }, tip, [0.3, 0.3, 0.32]));
        if self.extra().is_some() {
            out.parts.push(Part::new("payload", Shape::Box { size: [0.08, 0.08, 0.08] }, tip, [0.2, 0.7, 0.3]));
        }
        out.labels.push(Label {
            pos: [tip[0], tip[1], tip[2] - 0.1],
            owner_key: "axis-z",
            value: tip[2],
            unit_key: "unit-m",
            decimals: 3,
        });
    }
    fn scene_point(&self, tool: &[f64]) -> Option<[f64; 3]> {
        match self.tool {
            Tool::Joints => Some(self.chain.tip(tool).1),
            _ => Some([tool[0], tool[1], tool[2]]),
        }
    }
}
