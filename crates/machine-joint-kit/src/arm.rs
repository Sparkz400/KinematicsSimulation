//! A serial arm whose bodies, joints, masses and inertias come from a URDF file (`rigid::Chain`), its dynamics by the
//! recursive Newton-Euler algorithm. The tool pose is the joint positions themselves (rad, m for a prismatic joint):
//! the cycle is given in joints, and the sizing judges motors, gears and screws along it. With
//! `kinematics.tool_space = true` it is the flange's pose instead (`tool`): x, y, z and c on the SCARA (closed form),
//! or those of x, y, z, rx, ry, rz that `kinematics.tool_axes` names on any other chain of 4 to 7 joints (numerical,
//! one joint to spare on a redundant arm), each axis's limits in `[axis.x]`, ..., its range from `workspace.tool_min`
//! and `tool_max`; poses near a singularity (`workspace.max_condition_number`) are refused.
//!
//! Config `[arm]`: `urdf_file` (a path, or "builtin:six-axis" / "builtin:scara" / "builtin:five-axis" /
//! "builtin:seven-axis" / "builtin:palletiser" for the files in `urdf/`), `gear_ratios` (one per motor, before any
//! screw), `payload_kg`, `payload_offset_m` (its centre from the URDF's tip frame) and `payload_inertia_kgm2` (about its
//! centre in the tip frame: xx, yy, zz, xy, xz, yz), `joint_coupling` (the URDF's joints as sums of the motors' joints,
//! one row per URDF joint: a parallelogram that keeps a palletiser's flange level; `[]` for one motor per joint).
//! Each motor joint's limits are `[axis.j1]`, `[axis.j2]`, ...: `max_velocity_rad_s` and so on (`_m_` for a prismatic
//! joint). Each joint has its own `[motor.<k>]`, `[drive.<k>]`, `[gearbox.<k>]`, `[ratings.<k>]` and `[parts.<k>]` over
//! the shared tables, `[strain_wave.<k>]` and `[joint.<k>]` too (`catalog::motor_config`, k = 1, 2, ...); its ratio
//! is `arm.gear_ratios`'. A revolute joint has the `[joint]` friction and stops (its stops from the URDF's limits where
//! it gives them). A prismatic joint is a `[screw]` coupled to its motor (ratio 1; the `[gearbox]`'s stiffness and
//! damping stand for the coupling, its efficiency and inertia do not count) with the `[slide]` friction and stops. With
//! a `[strain_wave]` table, each revolute joint's gear is judged by its life.
//!
//! With `effects.payload_swing` the payload hangs from the flange on a pivot about two axes across the line to its
//! centre (`arm.payload_offset_m`, its length), held by `arm.payload_swing_stiffness_nm_rad` and damped by
//! `arm.payload_swing_damping_nm_s_rad`: two joints no motor drives (`chain::Mechanism::passive_joints`), a sack or a
//! carton stack in a soft gripper that swings after each move.

use crate::tool::{POSE_AXES, Pose, Scara, Tool, turn_length};
use crate::{axis, gravity, machine, payload, required_life};
use sim_components::beam::{CANTILEVER_PARTICIPATION, CANTILEVER_TIP_SLOPE, cantilever_mode, tube_second_moment};
use sim_components::catalog::motor_config;
use sim_components::chain::{FlexMode, JOINT_KEYS, MOTOR_IDS, MOTOR_KEYS, Mechanism};
use sim_components::joint::RotaryJoint;
use sim_components::rigid::{Body, Chain, Inertial, M3, V3, add, cross, dot, exp, matmul, mul, transpose};
use sim_components::transmission::{BallScrew, StrainWave};
use sim_core::{Config, ConfigError, Label, Machine, Part, RatingCheck, Scene, Shape, SignalInfo};
use std::f64::consts::TAU;

const BUILTIN: [(&str, &str); 5] = [
    ("builtin:six-axis", include_str!("../urdf/six-axis.urdf")),
    ("builtin:scara", include_str!("../urdf/scara.urdf")),
    ("builtin:five-axis", include_str!("../urdf/five-axis.urdf")),
    ("builtin:seven-axis", include_str!("../urdf/seven-axis.urdf")),
    ("builtin:palletiser", include_str!("../urdf/palletiser.urdf")),
];

const AXIS_IDS: [&str; 7] = ["j1", "j2", "j3", "j4", "j5", "j6", "j7"];
/// The SCARA's tool axes in closed form.
const SCARA_AXES: [(&str, &str); 4] = [("x", "axis-x"), ("y", "axis-y"), ("z", "axis-z"), ("c", "axis-tool-c")];

pub struct Arm {
    /// The URDF's chain: the kinematics.
    chain: Chain,
    /// The chain the dynamics are of: the URDF's, with the swinging payload's two bodies after it.
    moving: Chain,
    /// Each body's inertial as built, which `set_link_change` changes from.
    built: Vec<Inertial>,
    g: f64,
    /// The payload in the last body's frame (fixed to it), or on the swing's last body.
    payload: Inertial,
    /// `effects.payload_swing`: the swing's spring and damper per axis.
    swing: Option<(f64, f64)>,
    attached: bool,
    /// The URDF's joints from the motors' (`arm.joint_coupling`), rows by URDF joint; `None` one to one.
    coupling: Option<Vec<Vec<f64>>>,
    /// Per motor joint: motor radians per joint unit.
    ratios: Vec<f64>,
    /// Per motor joint: whether it is a prismatic one.
    prismatic: Vec<bool>,
    screw: Option<BallScrew>,
    /// Per motor joint, its strain wave gear where the config has one.
    strain_waves: Vec<Option<StrainWave>>,
    required_life: Option<f64>,
    tool: Tool,
    /// `workspace.max_condition_number`: tool poses beyond it are refused.
    max_condition: f64,
    /// effects.link_bending: the links that bend, each in one mode.
    bending: Vec<BentLink>,
}

/// A link that bends (effects.link_bending) in its first Euler-Bernoulli mode as a cantilever from its joint, in the
/// plane its joint turns it in: from the joint's origin to the next joint (the flange after the last) along `dir`, by
/// its tip's deflection along `across`, so that it turns about `dir x across`; all in its body's frame. Its own mass
/// is the URDF link's, taken as spread evenly along it; what the chain carries beyond it sits at its tip.
// shortcut: one mode per link, in the plane its joint turns it in (none where the link lies along the joint's axis);
// out of that plane and in torsion it is rigid, and the bodies beyond move with its tip but their own turning inertia
// is left out of its mode. Add the second plane where a payload hangs off to the side.
#[derive(Debug, Clone, Copy)]
struct BentLink {
    body: usize,
    length: f64,
    dir: V3,
    across: V3,
    ei: f64,
    damping_ratio: f64,
}

/// The bending links of effects.link_bending: per joint from `link_bending.outer_diameter_m` and `wall_m` (a tube; a
/// diameter of 0 keeps that link rigid) and `youngs_modulus_pa`, all with `damping_ratio`.
fn bent_links(cfg: &Config, chain: &Chain) -> Result<Vec<BentLink>, ConfigError> {
    if !cfg.bool("effects.link_bending")? {
        return Ok(Vec::new());
    }
    let n = chain.len();
    let per_link = |key: &str| -> Result<Vec<f64>, ConfigError> {
        let v = cfg.f64s(&format!("link_bending.{key}"))?;
        if v.len() != n || v.iter().any(|x| !(*x >= 0.0)) {
            return Err(ConfigError::Invalid(format!("link_bending.{key}")));
        }
        Ok(v)
    };
    let (outer, wall) = (per_link("outer_diameter_m")?, per_link("wall_m")?);
    let (e, damping_ratio) =
        (cfg.positive("link_bending.youngs_modulus_pa")?, cfg.non_negative("link_bending.damping_ratio")?);
    let mut links = Vec::new();
    for (body, b) in chain.bodies.iter().enumerate() {
        let to = chain.bodies.get(body + 1).map_or(chain.tip_pos, |next| next.origin_pos);
        let length = dot(to, to).sqrt();
        if outer[body] == 0.0 || length < 1e-3 || b.prismatic {
            continue;
        }
        let dir = to.map(|v| v / length);
        let side = cross(b.axis, dir);
        let size = dot(side, side).sqrt();
        if size < 0.1 {
            continue;
        }
        links.push(BentLink {
            body,
            length,
            dir,
            across: side.map(|v| v / size),
            ei: e * tube_second_moment(outer[body], wall[body]),
            damping_ratio,
        });
    }
    Ok(links)
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

/// `arm.joint_coupling`: `None` for `[]`; else one row per URDF joint of one factor per motor joint, each motor joint in
/// at least one row.
fn coupling(cfg: &Config, urdf_joints: usize) -> Result<Option<Vec<Vec<f64>>>, ConfigError> {
    let bad = || ConfigError::Invalid("arm.joint_coupling".into());
    let rows = cfg.get("arm.joint_coupling")?.as_array().ok_or_else(bad)?.clone();
    if rows.is_empty() {
        return Ok(None);
    }
    let rows: Vec<Vec<f64>> = rows
        .iter()
        .map(|r| r.as_array().ok_or_else(bad)?.iter().map(|v| v.as_float().ok_or_else(bad)).collect())
        .collect::<Result<_, _>>()?;
    let motors = rows[0].len();
    let used = |k: usize| rows.iter().any(|r| r[k] != 0.0);
    if rows.len() != urdf_joints || motors == 0 || rows.iter().any(|r| r.len() != motors) || !(0..motors).all(used) {
        return Err(bad());
    }
    Ok(Some(rows))
}

/// The two bodies of a payload that swings on the flange: a pivot about the first axis across the line from the flange
/// to the payload's centre, then the payload about the second, its centre `length` along the line. `rot` and `pos` put
/// the frame whose z is that line onto the last body's frame.
fn swing_bodies(rot: M3, pos: V3, payload: &Inertial, length: f64) -> [Body; 2] {
    let body = |name: &str, origin_rot: M3, origin_pos: V3, axis: V3, inertial: Inertial| Body {
        name: name.into(),
        origin_rot,
        origin_pos,
        axis,
        prismatic: false,
        inertial,
        limits: None,
    };
    let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let along = Inertial {
        mass: payload.mass,
        com: [0.0, 0.0, length],
        inertia: matmul(&matmul(&transpose(&rot), &payload.inertia), &rot),
    };
    [
        body("swing-1", rot, pos, [1.0, 0.0, 0.0], Inertial::default()),
        body("swing-2", identity, [0.0; 3], [0.0, 1.0, 0.0], along),
    ]
}

/// A rotation whose third column is the unit vector `z`.
fn frame_along(z: V3) -> M3 {
    let helper = if z[0].abs() < 0.9 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let x = cross(helper, z);
    let x = x.map(|v| v / dot(x, x).sqrt());
    let y = cross(z, x);
    std::array::from_fn(|r| [x[r], y[r], z[r]])
}

pub fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    let chain = chain(cfg)?;
    let u = chain.len();
    let coupling = coupling(cfg, u)?;
    let n = coupling.as_ref().map_or(u, |c| c[0].len());
    if n == 0 || n > MOTOR_IDS.len() {
        return Err(ConfigError::Invalid("arm.urdf_file".into()));
    }
    // The URDF joint that a motor joint is alone in, with its factor: its limits and its kind are the motor joint's.
    let alone = |k: usize| -> Option<(usize, f64)> {
        match &coupling {
            None => Some((k, 1.0)),
            Some(rows) => rows
                .iter()
                .position(|r| r.iter().enumerate().all(|(c, v)| (c == k) == (*v != 0.0)))
                .map(|i| (i, rows[i][k])),
        }
    };
    let prismatic: Vec<bool> = (0..n).map(|k| alone(k).is_some_and(|(i, _)| chain.bodies[i].prismatic)).collect();
    if let Some(rows) = &coupling {
        // A prismatic joint has to be a motor's own.
        let shared = |i: usize| {
            rows[i].iter().filter(|v| **v != 0.0).count() > 1 || rows[i].iter().any(|v| *v != 0.0 && v.abs() != 1.0)
        };
        if (0..u).any(|i| chain.bodies[i].prismatic && shared(i)) {
            return Err(ConfigError::Invalid("arm.joint_coupling".into()));
        }
    }
    let ratios = cfg.f64s("arm.gear_ratios")?;
    if ratios.len() != n || ratios.iter().any(|x| !(*x > 0.0)) {
        return Err(ConfigError::Invalid("arm.gear_ratios".into()));
    }
    let owns: Vec<Config> = (0..n).map(|k| motor_config(cfg, MOTOR_IDS[k])).collect::<Result<_, _>>()?;
    // shortcut: the gearbox life takes each motor's speed through its own `gearbox.ratio` (analysis::output_rpm), so it
    // is refused where a motor's ratio is not the one its gearbox table says; report each motor's ratio to the analysis
    // to allow it.
    if cfg.f64("ratings.gearbox_reference_speed_rpm").is_ok()
        && (0..n).any(|k| !prismatic[k] && Some(ratios[k]) != owns[k].f64("gearbox.ratio").ok())
    {
        return Err(ConfigError::Invalid("ratings.gearbox_reference_speed_rpm".into()));
    }
    let screw = if prismatic.iter().any(|p| *p) { Some(BallScrew::from_config(cfg)?) } else { None };
    let strain_waves = (0..n)
        .map(|k| {
            if !prismatic[k] && owns[k].get("strain_wave").is_ok() {
                StrainWave::from_config(&owns[k]).map(Some)
            } else {
                Ok(None)
            }
        })
        .collect::<Result<Vec<_>, _>>()?;
    let (mut joints, mut axes) = (Vec::new(), Vec::new());
    for k in 0..n {
        let own = &owns[k];
        let (mut joint, lever) = match (&screw, prismatic[k]) {
            (Some(screw), true) => {
                // The screw sits on the motor's shaft through a coupling: no gear, no gear losses or inertia.
                if ratios[k] != 1.0 {
                    return Err(ConfigError::Invalid("arm.gear_ratios".into()));
                }
                let mut joint = RotaryJoint::linear_from_config(own)?;
                joint.gearbox.ratio = 1.0;
                joint.gearbox.efficiency = 1.0;
                joint.coupled = true;
                joint.rotor_inertia = own.positive("motor.rotor_inertia_kgm2")?;
                screw.converter(screw.softest_position()).apply(own, &mut joint)?;
                if own.bool("effects.joint_friction")? {
                    joint.friction.coulomb += screw.preload_torque() * screw.ratio();
                }
                (joint, Some(1.0 / screw.ratio()))
            }
            _ => {
                let mut joint = RotaryJoint::from_config(own)?;
                joint.gearbox.ratio = ratios[k];
                joint.coupled = ratios[k] == 1.0;
                (joint, None)
            }
        };
        if let Some((i, factor)) = alone(k)
            && let Some((low, high)) = chain.bodies[i].limits
        {
            joint.range = if factor > 0.0 { (low / factor, high / factor) } else { (high / factor, low / factor) };
        }
        let unit = if prismatic[k] { "m" } else { "rad" };
        axes.push(axis(cfg, AXIS_IDS[k], JOINT_KEYS[k], unit, joint.range)?);
        joints.push((joint, lever, unit));
    }

    let offset = cfg.f64s("arm.payload_offset_m")?;
    let &[x, y, z] = &offset[..] else { return Err(ConfigError::Invalid("arm.payload_offset_m".into())) };
    let inertia = cfg.f64s("arm.payload_inertia_kgm2")?;
    let &[ixx, iyy, izz, ixy, ixz, iyz] = &inertia[..] else {
        return Err(ConfigError::Invalid("arm.payload_inertia_kgm2".into()));
    };
    let on = cfg.bool("effects.payload")?;
    let tensor = [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]].map(|r| r.map(|v| if on { v } else { 0.0 }));
    // In the tip frame, then turned into the last body's.
    let in_tip = Inertial { mass: payload(cfg, "arm.payload_kg")?, com: [x, y, z], inertia: tensor };
    let payload = Inertial {
        mass: in_tip.mass,
        com: add(chain.tip_pos, mul(&chain.tip_rot, in_tip.com)),
        inertia: matmul(&matmul(&chain.tip_rot, &in_tip.inertia), &transpose(&chain.tip_rot)),
    };
    let mut moving = chain.clone();
    let swing = if cfg.bool("effects.payload_swing")? {
        let length = dot(offset_v(&offset), offset_v(&offset)).sqrt();
        if !(length > 0.0) {
            return Err(ConfigError::Invalid("arm.payload_offset_m (the swing's length)".into()));
        }
        let rot = matmul(&chain.tip_rot, &frame_along(offset_v(&offset).map(|v| v / length)));
        moving.bodies.extend(swing_bodies(rot, chain.tip_pos, &payload_about(&in_tip, &chain.tip_rot), length));
        Some((
            cfg.non_negative("arm.payload_swing_stiffness_nm_rad")?,
            cfg.non_negative("arm.payload_swing_damping_nm_s_rad")?,
        ))
    } else {
        None
    };

    let max_condition = cfg.positive("workspace.max_condition_number")?;
    let bending = bent_links(cfg, &chain)?;
    let mut arm = Arm {
        chain,
        moving,
        payload,
        swing,
        g: gravity(cfg)?,
        attached: true,
        coupling,
        ratios: joints.iter().map(|(j, ..)| j.ratio()).collect(),
        prismatic,
        screw,
        strain_waves,
        required_life: required_life(cfg),
        built: Vec::new(),
        max_condition,
        tool: Tool::Joints,
        bending,
    };
    arm.built = arm.chain.bodies.iter().map(|b| b.inertial).collect();
    if cfg.bool("kinematics.tool_space")? {
        let refuse = || ConfigError::Invalid("kinematics.tool_space".into());
        let names = cfg.get("kinematics.tool_axes")?.as_array().ok_or_else(refuse)?.clone();
        let picked: Vec<usize> = names
            .iter()
            .map(|v| v.as_str().and_then(|s| POSE_AXES.iter().position(|(id, ..)| *id == s)))
            .collect::<Option<_>>()
            .ok_or_else(|| ConfigError::Invalid("kinematics.tool_axes".into()))?;
        let scara = (picked == [0, 1, 2, 5] && arm.coupling.is_none())
            .then(|| -> Result<Option<Scara>, ConfigError> {
                let elbow = match cfg.get("kinematics.elbow")?.as_str() {
                    Some("right") => 1.0,
                    Some("left") => -1.0,
                    _ => return Err(ConfigError::Invalid("kinematics.elbow".into())),
                };
                Ok(Scara::new(&arm.chain, elbow))
            })
            .transpose()?
            .flatten();
        let ids: Vec<(&'static str, &'static str, &'static str)> = match scara {
            Some(s) => {
                arm.tool = Tool::Scara(s);
                SCARA_AXES
                    .iter()
                    .enumerate()
                    .map(|(k, (id, key))| (*id, *key, if k < 3 { "m" } else { "rad" }))
                    .collect()
            }
            None => {
                let home = cfg.f64s("kinematics.home_joints")?;
                if home.len() != n {
                    return Err(ConfigError::Invalid("kinematics.home_joints".into()));
                }
                let fk = |q: &[f64]| arm.tip(q);
                let full = arm.full(&home);
                let length = turn_length(arm.chain.tip(&full).1, arm.chain.frames(&full)[1.min(u - 1)].1);
                let pose = Pose::new(&fk, home, picked.clone(), length, max_condition).ok_or_else(refuse)?;
                arm.tool = Tool::Pose(pose);
                picked.iter().map(|&a| POSE_AXES[a]).collect()
            }
        };
        let m = ids.len();
        let (min, max) = (cfg.f64s("workspace.tool_min")?, cfg.f64s("workspace.tool_max")?);
        if min.len() != m || max.len() != m || min.iter().zip(&max).any(|(a, b)| !(a < b)) {
            return Err(ConfigError::Invalid("workspace.tool_min".into()));
        }
        axes = ids
            .iter()
            .enumerate()
            .map(|(k, (id, key, unit))| axis(cfg, id, key, unit, (min[k], max[k])))
            .collect::<Result<_, _>>()?;
    }
    machine(cfg, arm, joints, axes)
}

fn offset_v(offset: &[f64]) -> V3 {
    [offset[0], offset[1], offset[2]]
}

/// The payload's inertia about its centre turned from the tip frame into the last body's (its centre is the swing's).
fn payload_about(in_tip: &Inertial, tip_rot: &M3) -> Inertial {
    Inertial { inertia: matmul(&matmul(tip_rot, &in_tip.inertia), &transpose(tip_rot)), ..*in_tip }
}

impl Arm {
    /// The URDF's joints at the motors' joints `q` (passive ones after them are left out).
    fn full(&self, q: &[f64]) -> Vec<f64> {
        match &self.coupling {
            None => q[..self.chain.len()].to_vec(),
            Some(rows) => rows.iter().map(|r| r.iter().zip(q).map(|(c, v)| c * v).sum()).collect(),
        }
    }

    /// The flange's orientation and position at the motors' joints.
    fn tip(&self, q: &[f64]) -> (M3, V3) {
        self.chain.tip(&self.full(q))
    }

    /// The payload fixed to the last body; on the swing it is the swing's body instead.
    fn extra(&self) -> Option<Inertial> {
        (self.attached && self.payload.mass > 0.0 && self.swing.is_none()).then_some(self.payload)
    }

    /// The moving chain's joints at all of the mechanism's joints (the motors', then the swing's).
    fn moving_joints(&self, q: &[f64]) -> Vec<f64> {
        let n = self.ratios.len();
        let mut out = self.full(q);
        out.extend(&q[n..]);
        out
    }

    /// The chain the dynamics are of (with the swing's bodies while they carry the payload; `extra` is the payload
    /// otherwise) and its joints at the mechanism's joints `q`, the swing's at rest where `q` has the motors' only.
    fn loaded(&self, q: &[f64]) -> (&Chain, Vec<f64>) {
        let detached_swing = self.swing.is_some() && !(self.attached && self.payload.mass > 0.0);
        let chain = if detached_swing { &self.chain } else { &self.moving };
        let mut joints = self.moving_joints(q);
        joints.resize(chain.len(), 0.0);
        (chain, joints)
    }

    /// The URDF joint (body) whose axis motor joint `k`'s motor turns about: the first it moves.
    fn motor_body(&self, k: usize) -> usize {
        match &self.coupling {
            None => k,
            Some(rows) => rows.iter().position(|r| r[k] != 0.0).unwrap_or(0),
        }
    }

    /// The joints at a tool pose and their Jacobian's condition number there (`tool::Scara::condition`,
    /// `tool::Pose::condition`); `None` out of reach.
    fn solved(&self, tool: &[f64]) -> Option<(Vec<f64>, f64)> {
        let mut q = vec![0.0; self.ratios.len()];
        let fk = |q: &[f64]| self.tip(q);
        match &self.tool {
            Tool::Joints => None,
            Tool::Scara(s) => s.inverse(tool, &mut q).then(|| (q.clone(), s.condition(&q))),
            Tool::Pose(p) => p.inverse(&fk, tool, &mut q).then(|| (q.clone(), p.condition(&fk, &q))),
        }
    }
}

impl Mechanism for Arm {
    /// The bodies' (`Chain::momentum` of the chain the dynamics are of, the payload with the last or on the swing) and
    /// each motor's rotor spinning about the axis of the joint it turns (`motor_body`).
    fn frame_momentum(&self, q: &[f64], qd: &[f64], rotor_spin: &[f64]) -> Option<sim_core::FrameMomentum> {
        let ((chain, q), (_, qd)) = (self.loaded(q), self.loaded(qd));
        let (linear, mut angular) = chain.momentum(&q, &qd, self.extra());
        let (mut mass, mut first_moment) = (0.0, [0.0; 3]);
        let bodies = chain.body_states(&q, &qd, self.extra());
        for b in &bodies {
            mass += b.inertial.mass;
            first_moment = add(first_moment, b.com.map(|c| c * b.inertial.mass));
        }
        for (k, spin) in rotor_spin.iter().enumerate() {
            angular = add(angular, bodies[self.motor_body(k)].axis.map(|a| a * spin));
        }
        Some(sim_core::FrameMomentum {
            linear,
            angular,
            mass,
            first_moment,
            gravity: [0.0, 0.0, -self.g],
            ..Default::default()
        })
    }

    /// Per bending link its cantilever mode (`beam::cantilever_mode`): its own mass spread along it, everything the
    /// chain carries beyond it at its tip (a swinging payload too).
    fn flex_modes(&self) -> Vec<FlexMode> {
        let (chain, zero) = self.loaded(&vec![0.0; self.ratios.len()]);
        let masses: Vec<f64> = chain.body_states(&zero, &zero, self.extra()).iter().map(|b| b.inertial.mass).collect();
        self.bending
            .iter()
            .map(|l| {
                let (mass, stiffness) =
                    cantilever_mode(l.ei, masses[l.body], masses[l.body + 1..].iter().sum(), l.length);
                FlexMode { link: l.body, mass, stiffness, damping: 2.0 * l.damping_ratio * (mass * stiffness).sqrt() }
            })
            .collect()
    }

    /// Across each bending link: the force and the moment the bodies beyond it need from its tip to move as they do
    /// against gravity (their m (a - g), the accelerations by central differences over +-1 us along q' and q''), taken
    /// back, the moment through the tip's slope; and its own body's m (a - g), as a load spread evenly along it. A
    /// swinging payload as if fixed where it hangs at rest.
    fn flex_loads(&self, q: &[f64], qd: &[f64], qdd: &[f64], out: &mut [f64]) {
        let h = 1e-6;
        let at = |s: f64| {
            let q: Vec<f64> = q.iter().zip(qd).map(|(q, v)| q + s * v).collect();
            let qd: Vec<f64> = qd.iter().zip(qdd).map(|(v, a)| v + s * a).collect();
            let ((chain, q), (_, qd)) = (self.loaded(&q), self.loaded(&qd));
            chain.body_states(&q, &qd, self.extra())
        };
        let (now, before, after) = (at(0.0), at(-h), at(h));
        let need: Vec<V3> = (0..now.len())
            .map(|k| {
                let a: V3 = std::array::from_fn(|i| (after[k].velocity[i] - before[k].velocity[i]) / (2.0 * h));
                [a[0], a[1], a[2] + self.g].map(|v| v * now[k].inertial.mass)
            })
            .collect();
        for (link, load) in self.bending.iter().zip(out.iter_mut()) {
            let b = &now[link.body];
            let tip = add(b.origin, mul(&b.rot, link.dir.map(|v| v * link.length)));
            let (across, turn) = (mul(&b.rot, link.across), mul(&b.rot, cross(link.dir, link.across)));
            let (mut force, mut moment) = ([0.0; 3], [0.0; 3]);
            for k in link.body + 1..now.len() {
                force = add(force, need[k]);
                moment = add(moment, cross(std::array::from_fn(|i| now[k].com[i] - tip[i]), need[k]));
            }
            *load = -dot(across, force)
                - CANTILEVER_PARTICIPATION * dot(across, need[link.body])
                - dot(turn, moment) * CANTILEVER_TIP_SLOPE / link.length;
        }
    }

    /// The flange moves with each bending link's tip and turns with its slope: by d across it and by 1.3765 d / L about
    /// dir x across, about the tip. In joint space the tool is the joints, which the bending does not move.
    fn flex_shift(&self, q: &[f64], d: &[f64], out: &mut [f64]) {
        out.iter_mut().for_each(|o| *o = 0.0);
        if matches!(self.tool, Tool::Joints) {
            return;
        }
        let full = self.full(q);
        let frames = self.chain.frames(&full);
        let (flange_rot, flange) = self.chain.tip(&full);
        let (mut moved, mut turned) = ([0.0; 3], [0.0; 3]);
        for (link, d) in self.bending.iter().zip(d) {
            let (rot, origin) = frames[link.body];
            let tip = add(origin, mul(&rot, link.dir.map(|v| v * link.length)));
            let turn = mul(&rot, cross(link.dir, link.across)).map(|v| v * CANTILEVER_TIP_SLOPE * d / link.length);
            let swing = cross(turn, std::array::from_fn(|i| flange[i] - tip[i]));
            moved = add(moved, add(mul(&rot, link.across).map(|v| v * d), swing));
            turned = add(turned, turn);
        }
        match &self.tool {
            Tool::Scara(_) => {
                out[..3].copy_from_slice(&moved);
                out[3] = turned[2];
            }
            // The tool's axes of the bent flange less those of the straight one, whichever of the six they are.
            Tool::Pose(p) => {
                let (mut bent, mut straight) = (vec![0.0; out.len()], vec![0.0; out.len()]);
                p.forward(&|_: &[f64]| (matmul(&exp(turned), &flange_rot), add(flange, moved)), q, &mut bent);
                p.forward(&|_: &[f64]| (flange_rot, flange), q, &mut straight);
                out.iter_mut().zip(bent.iter().zip(&straight)).for_each(|(o, (b, s))| *o = b - s);
            }
            Tool::Joints => {}
        }
    }

    /// The moving chain's, taken to the motors' joints where the URDF's are coupled to them: with q_urdf = C q,
    /// M = C^T M_urdf C and h = C^T h_urdf (C constant, so no term of its own; the coupling's forces do no work).
    /// shortcut: the parallelogram's bars are in the links they move with, not bodies of their own; give them their own
    /// inertia where they are heavy against the arms.
    fn dynamics(&self, q: &[f64], qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        let all = q.len();
        let detached_swing = self.swing.is_some() && !(self.attached && self.payload.mass > 0.0);
        let moving = if detached_swing { &self.chain } else { &self.moving };
        let (qm, qdm) = (self.moving_joints(q), self.moving_joints(qd));
        let l = moving.len();
        let (mut ml, mut hl) = (vec![0.0; l * l], vec![0.0; l]);
        moving.dynamics(&qm[..l], &qdm[..l], self.g, self.extra(), &mut ml, &mut hl);
        let n = self.ratios.len();
        // T: the moving chain's joints per mechanism joint (l x all).
        let t = |r: usize, c: usize| -> f64 {
            match (&self.coupling, r < self.chain.len(), c < n) {
                (Some(rows), true, true) => rows[r][c],
                (None, true, true) => (r == c) as u8 as f64,
                (_, false, false) => (r - self.chain.len() == c - n) as u8 as f64,
                _ => 0.0,
            }
        };
        for i in 0..all {
            bias[i] = (0..l).map(|r| t(r, i) * hl[r]).sum();
            for j in 0..all {
                mass[i * all + j] =
                    (0..l).map(|r| t(r, i) * (0..l).map(|c| ml[r * l + c] * t(c, j)).sum::<f64>()).sum();
            }
        }
        if detached_swing {
            // shortcut: without the payload the swing's joints carry nothing: a unit inertia of their own, apart from the
            // arm, so that they stay solvable; drop them when the payload goes and back when it comes.
            for i in n..all {
                mass[i * all + i] = 1.0;
            }
        }
    }
    fn potential(&self, q: &[f64]) -> f64 {
        let qm = self.moving_joints(q);
        match self.swing.is_some() && self.attached {
            true => self.moving.potential(&qm, self.g, None),
            false => self.chain.potential(&qm[..self.chain.len()], self.g, self.extra()),
        }
    }
    fn passive_joints(&self) -> Vec<(f64, f64)> {
        self.swing.map_or_else(Vec::new, |s| vec![s; 2])
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.attached = attached;
    }
    fn links(&self) -> Vec<String> {
        self.chain.bodies.iter().map(|b| b.name.clone()).collect()
    }
    fn set_link_change(&mut self, link: usize, factor: f64, moment: [f64; 3]) {
        let changed = self.built[link].changed(factor, moment);
        self.chain.bodies[link].inertial = changed;
        self.moving.bodies[link].inertial = changed;
    }

    fn forward(&self, q: &[f64], tool: &mut [f64]) {
        match &self.tool {
            Tool::Joints => tool.copy_from_slice(&q[..tool.len()]),
            Tool::Scara(s) => s.forward(q, tool),
            Tool::Pose(p) => p.forward(&|q: &[f64]| self.tip(q), q, tool),
        }
    }
    fn inverse(&self, tool: &[f64], q: &mut [f64]) -> bool {
        match &self.tool {
            Tool::Joints => {
                q.copy_from_slice(tool);
                true
            }
            Tool::Scara(s) => s.inverse(tool, q),
            Tool::Pose(p) => p.inverse(&|q: &[f64]| self.tip(q), tool, q),
        }
    }
    /// In tool space: out of reach, a turn of the flange near half a turn from its orientation at zero joints (where
    /// its rotation vector jumps), or too near a singularity; the joints' stops are the joint machine's to check.
    fn unreachable(&self, tool: &[f64], _slack: f64) -> Option<&'static str> {
        match &self.tool {
            Tool::Joints => return None,
            Tool::Pose(p) if p.near_half_turn(tool) => return Some("error-arm-orientation-half-turn"),
            _ => {}
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
        for k in 0..self.ratios.len() {
            match (self.prismatic[k], &self.screw, &self.strain_waves[k]) {
                (true, Some(s), _) => out.extend(s.signal_infos(MOTOR_IDS[k], JOINT_KEYS[k])),
                (false, _, Some(g)) => out.push(g.signal_info(MOTOR_IDS[k], JOINT_KEYS[k])),
                _ => {}
            }
        }
        out
    }
    fn signals(&self, _q: &[f64], qd: &[f64], _qdd: &[f64], force: &[f64], out: &mut Vec<f64>) {
        for k in 0..self.ratios.len() {
            match (self.prismatic[k], &self.screw, &self.strain_waves[k]) {
                (true, Some(s), _) => s.signals(qd[k], force[k], out),
                (false, _, Some(g)) => out.push(g.signal(force[k], qd[k] * self.ratios[k])),
                _ => {}
            }
        }
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        let mut out = Vec::new();
        for k in 0..self.ratios.len() {
            match (self.prismatic[k], &self.screw, &self.strain_waves[k], self.required_life) {
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

    /// Each body as a bar from its joint to the next, each motor joint's motor with its shaft bar and speed at the first
    /// URDF joint it turns; the flange and the payload as boxes (the payload where it swings to, on a line from the
    /// flange), for the collision with obstacles.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene) {
        let n = self.ratios.len();
        let full = self.full(q);
        let frames = self.chain.frames(&full);
        let (_, tip) = self.chain.tip(&full);
        out.parts.push(Part::new(
            "base",
            Shape::Box { size: [0.3, 0.3, 0.05] },
            [0.0, 0.0, -0.025],
            [0.62, 0.64, 0.68],
        ));
        let mut from = [0.0; 3];
        for (_, at) in &frames {
            out.parts.push(Part::new("link", Shape::Segment { to: *at, width: 0.05 }, from, [0.2, 0.22, 0.26]));
            from = *at;
        }
        for k in 0..n {
            let at = frames[self.motor_body(k)].1;
            out.parts.push(Part::new(
                sim_core::scene::MOTOR_PART,
                Shape::Cylinder { radius: 0.04, length: 0.08 },
                at,
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
            if !self.prismatic[k] {
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
        }
        out.parts.push(Part::new("link", Shape::Segment { to: tip, width: 0.03 }, from, [0.2, 0.22, 0.26]));
        out.parts.push(Part::new("flange", Shape::Box { size: [0.06, 0.06, 0.06] }, tip, [0.3, 0.3, 0.32]));
        if self.attached && self.payload.mass > 0.0 {
            let centre = if self.swing.is_some() && q.len() > n {
                let swung = self.moving.frames(&self.moving_joints(q));
                let (rot, at) = swung[swung.len() - 1];
                let centre = add(at, mul(&rot, self.moving.bodies[self.moving.len() - 1].inertial.com));
                out.parts.push(Part::new("sling", Shape::Segment { to: centre, width: 0.01 }, tip, [0.1, 0.1, 0.1]));
                centre
            } else {
                tip
            };
            out.parts.push(Part::new("payload", Shape::Box { size: [0.08, 0.08, 0.08] }, centre, [0.2, 0.7, 0.3]));
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
            Tool::Joints => Some(self.tip(tool).1),
            _ => Some([tool[0], tool[1], tool[2]]),
        }
    }
}
