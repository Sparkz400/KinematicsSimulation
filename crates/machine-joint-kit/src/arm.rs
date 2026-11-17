//! A serial arm whose bodies, joints, masses and inertias come from a URDF file (`rigid::Chain`), its dynamics by the
//! recursive Newton-Euler algorithm. The tool pose is the joint positions themselves (rad, m for a prismatic joint):
//! the cycle is given in joints, and the sizing judges motors, gears and screws along it. With
//! `kinematics.tool_space = true` it is the flange's pose instead (`tool`): x, y, z and c on the SCARA (closed form),
//! or those of x, y, z, rx, ry, rz that `kinematics.tool_axes` names on any other chain of 4 to 7 joints (numerical,
//! one joint to spare on a redundant arm), each axis's limits in `[axis.x]`, ..., its range from `workspace.tool_min`
//! and `tool_max`; poses near a singularity (`workspace.max_condition_number`) are refused.
//!
//! Config `[arm]`: `urdf_file` (a path, or "builtin:six-axis" / "builtin:scara" / "builtin:five-axis" /
//! "builtin:seven-axis" / "builtin:palletiser" / "builtin:arm-on-track" for the files in `urdf/`), `gear_ratios` (one per motor, before any
//! screw), `payload_kg`, `payload_offset_m` (its centre from the URDF's tip frame) and `payload_inertia_kgm2` (about its
//! centre in the tip frame: xx, yy, zz, xy, xz, yz), `joint_coupling` (the URDF's joints as sums of the motors' joints,
//! one row per URDF joint: a parallelogram that keeps a palletiser's flange level; `[]` for one motor per joint).
//! Each motor joint's limits are `[axis.j1]`, `[axis.j2]`, ...: `max_velocity_rad_s` and so on (`_m_` for a prismatic
//! joint). Each joint has its own `[motor.<k>]`, `[drive.<k>]`, `[gearbox.<k>]`, `[ratings.<k>]` and `[parts.<k>]` over
//! the shared tables, `[strain_wave.<k>]` and `[joint.<k>]` too (`catalog::motor_config`, k = 1, 2, ...); its ratio
//! is `arm.gear_ratios`'. A revolute joint has the `[joint]` friction and stops (its stops from the URDF's limits where
//! it gives them). A prismatic joint is a `[screw]` coupled to its motor (ratio 1; the `[gearbox]`'s stiffness and
//! damping stand for the coupling, its efficiency and inertia do not count) with the `[slide]` friction and stops; where
//! the config has a `[rack]`, a pinion on a rack behind the joint's own gearbox instead (ratio `arm.gear_ratios`'), its
//! feed force and the pinion's radial load on the gearbox shaft (`ratings.shaft_max_radial_load_n` of its motor) judged.
//! With `[track]` the first joint is a carriage on four guide blocks (`track.block_spacing_m` along, `rail_spacing_m`
//! across), their load from the chain's reaction at that joint (`Arm::guide_load`, judged by the guide verdicts). With
//! a `[strain_wave]` table, each revolute joint's gear is judged by its life.
//!
//! With `effects.payload_swing` the payload hangs from the flange on a pivot about two axes across the line to its
//! centre (`arm.payload_offset_m`, its length), held by `arm.payload_swing_stiffness_nm_rad` and damped by
//! `arm.payload_swing_damping_nm_s_rad`: two joints no motor drives (`chain::Mechanism::passive_joints`), a sack or a
//! carton stack in a soft gripper that swings after each move.

use crate::tool::{FIRST_JOINT, POSE_AXES, Pose, Scara, Tool, turn_length};
use crate::{axis, gravity, machine, payload, required_life};
use sim_components::beam::{CANTILEVER_PARTICIPATION, CANTILEVER_TIP_SLOPE, cantilever_mode, tube_second_moment};
use sim_components::catalog::{GUIDE_CARRIAGES, motor_config};
use sim_components::chain::{FlexCoupling, FlexKind, FlexMode, JOINT_KEYS, MOTOR_IDS, MOTOR_KEYS, Mechanism};
use sim_components::joint::RotaryJoint;
use sim_components::rigid::{Body, BodyState, Chain, Inertial, M3, V3, add, cross, dot, exp, matmul, mul, transpose};
use sim_components::transmission::{BallScrew, RackPinion, StrainWave};
use sim_core::{Config, ConfigError, Label, Machine, Part, RatingCheck, Scene, Shape, SignalInfo};
use std::f64::consts::TAU;

const BUILTIN: [(&str, &str); 6] = [
    ("builtin:six-axis", include_str!("../urdf/six-axis.urdf")),
    ("builtin:scara", include_str!("../urdf/scara.urdf")),
    ("builtin:five-axis", include_str!("../urdf/five-axis.urdf")),
    ("builtin:seven-axis", include_str!("../urdf/seven-axis.urdf")),
    ("builtin:palletiser", include_str!("../urdf/palletiser.urdf")),
    ("builtin:arm-on-track", include_str!("../urdf/arm-on-track.urdf")),
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
    /// Per motor joint: its rotor's inertia (kg m^2, with the gearbox's at its input), which spins at `ratios` times the
    /// joint's speed (`guide_load`'s gyroscopic moment).
    rotor_inertia: Vec<f64>,
    /// Per motor joint: whether it is a prismatic one.
    prismatic: Vec<bool>,
    screw: Option<BallScrew>,
    /// Per motor joint, its rack and pinion where it is a prismatic one and the config has a `[rack]`: the rack, the
    /// efficiency in effect on its teeth, and its motor's `ratings.shaft_max_radial_load_n`.
    racks: Vec<Option<(RackPinion, f64, Option<f64>)>>,
    /// `[track]`: the first joint is a carriage on four guide blocks, `block_spacing_m` apart along its travel and
    /// `rail_spacing_m` across it, and the rack's side (+1 or -1 along the carriage's y) and its lever (the rack's line's
    /// distance from the carriage's centre line, m): `rack_side`, `rack_lever_m` (`guide_load`).
    track: Option<(f64, f64, f64, f64)>,
    /// Per motor joint, its strain wave gear where the config has one.
    strain_waves: Vec<Option<StrainWave>>,
    required_life: Option<f64>,
    tool: Tool,
    /// `workspace.max_condition_number`: tool poses beyond it are refused.
    max_condition: f64,
    /// effects.link_bending: the links' modes, per link in the order of `link_bending.directions`.
    bending: Vec<BentLink>,
    /// effects.link_bending_reaction: the modes two-way (`Mechanism::flex_coupling`).
    reaction: bool,
    /// With rx, ry or rz among the tool axes: the smallest of their velocity and of their acceleration limits, which the
    /// planned flange's angular speed and acceleration are held to (`tool.angular_speed`, `tool.angular_acceleration`:
    /// shown as measured; kept by both planners and judged on the plan's rigid motion, see `rating_checks`).
    angular_limits: Option<[f64; 2]>,
}

/// A link's mode (effects.link_bending), its first as a cantilever from its joint: from the joint's origin to the next
/// joint (the flange after the last) along `dir`. A bend (Euler-Bernoulli) moves its tip along `across` by the mode's
/// coordinate d and turns it about `turn = dir x across` by 1.3765 d / L (`slope / span`): in the plane its joint turns
/// it in (`across = axis x dir`), or across that plane (`across = dir x (axis x dir)`). A twist turns its tip about
/// `turn = dir` by d (`across` none). All in its body's frame. Its own mass is the URDF link's, taken as spread evenly
/// along it; what the chain carries beyond it sits on its tip.
// One mode per link and direction; a link along its joint's axis (the base column) has no bend. Every mode, the one-way
// bend in its joint's plane too, takes the bodies beyond whole (`field`): their mass and their turning inertia.
#[derive(Debug, Clone, Copy)]
struct BentLink {
    body: usize,
    length: f64,
    dir: V3,
    across: V3,
    turn: V3,
    slope: f64,
    span: f64,
    /// 1.875104^4 / 4 E I / L^3 (N/m) for a bend, G I_p / L (N m/rad) for a twist.
    stiffness: f64,
    damping_ratio: f64,
    kind: FlexKind,
}

/// The modes of effects.link_bending: per joint from `link_bending.outer_diameter_m` and `wall_m` (a tube; a diameter
/// of 0 keeps that link rigid), `youngs_modulus_pa` (and `shear_modulus_pa` for a twist), in each of
/// `link_bending.directions` ("in_plane", "out_of_plane", "torsion"), all with `damping_ratio`; and whether they act
/// back (effects.link_bending_reaction).
fn bent_links(cfg: &Config, chain: &Chain) -> Result<(Vec<BentLink>, bool), ConfigError> {
    if !cfg.bool("effects.link_bending")? {
        return Ok((Vec::new(), false));
    }
    let reaction = cfg.bool("effects.link_bending_reaction")?;
    let bad = || ConfigError::Invalid("link_bending.directions".into());
    let names = cfg.get("link_bending.directions")?.as_array().ok_or_else(bad)?.clone();
    let kinds: Vec<FlexKind> = names
        .iter()
        .map(|v| match v.as_str() {
            Some("in_plane") => Ok(FlexKind::Bend),
            Some("out_of_plane") => Ok(FlexKind::BendAcross),
            Some("torsion") => Ok(FlexKind::Twist),
            _ => Err(bad()),
        })
        .collect::<Result<_, _>>()?;
    if kinds.is_empty() || (0..kinds.len()).any(|i| kinds[..i].contains(&kinds[i])) {
        return Err(bad());
    }
    let shear = if kinds.contains(&FlexKind::Twist) { cfg.positive("link_bending.shear_modulus_pa")? } else { 0.0 };
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
        let second_moment = tube_second_moment(outer[body], wall[body]);
        let bend = cantilever_mode(e * second_moment, 0.0, 0.0, length).1;
        for kind in &kinds {
            let across = match kind {
                FlexKind::Twist => [0.0; 3],
                _ if size < 0.1 => continue,
                FlexKind::Bend => side.map(|v| v / size),
                FlexKind::BendAcross => cross(dir, side.map(|v| v / size)),
            };
            let (turn, slope, span, stiffness) = match kind {
                // G I_p / L, I_p = 2 I of the tube.
                FlexKind::Twist => (dir, 1.0, 1.0, shear * 2.0 * second_moment / length),
                _ => (cross(dir, across), CANTILEVER_TIP_SLOPE, length, bend),
            };
            links.push(BentLink {
                body,
                length,
                dir,
                across,
                turn,
                slope,
                span,
                stiffness,
                damping_ratio,
                kind: *kind,
            });
        }
    }
    Ok((links, reaction))
}

fn sub(a: V3, b: V3) -> V3 {
    std::array::from_fn(|i| a[i] - b[i])
}

/// Mode `link`'s field on body `k` (its own or one beyond it) of `states`: how far the body's centre moves and how far
/// it turns per unit of the mode's coordinate (base frame). Beyond the link the body rides on its tip: u + r x (c -
/// tip), turning by r. Its own body takes a share of the mode: a bend's evenly spread load drives it with 0.3915 of
/// its sum (`CANTILEVER_PARTICIPATION`, its turning left out), a twist (growing evenly along the link) with half of the
/// body's moment about the link's line.
fn field(link: &BentLink, states: &[BodyState], k: usize) -> (V3, V3) {
    let b = &states[link.body];
    let (u, r) = (mul(&b.rot, link.across), mul(&b.rot, link.turn).map(|v| v * link.slope / link.span));
    if k == link.body {
        return match link.kind {
            FlexKind::Twist => (cross(r, sub(b.com, b.origin)).map(|v| 0.5 * v), r.map(|v| 0.5 * v)),
            _ => (u.map(|v| CANTILEVER_PARTICIPATION * v), [0.0; 3]),
        };
    }
    let tip = add(b.origin, mul(&b.rot, link.dir.map(|v| v * link.length)));
    (add(u, cross(r, sub(states[k].com, tip))), r)
}

/// The rate of `field` as the joints move at the speeds of `states`: everything of the link turns with its spin w,
/// and the body beyond moves against its tip.
fn field_rate(link: &BentLink, states: &[BodyState], k: usize) -> V3 {
    let b = &states[link.body];
    let w = b.spin;
    let (t, _) = field(link, states, k);
    if k == link.body {
        return cross(w, t);
    }
    let (u, r) = (mul(&b.rot, link.across), mul(&b.rot, link.turn).map(|v| v * link.slope / link.span));
    let tip = add(b.origin, mul(&b.rot, link.dir.map(|v| v * link.length)));
    let tip_speed = add(b.velocity, cross(w, sub(tip, b.com)));
    add(add(cross(w, u), cross(cross(w, r), sub(states[k].com, tip))), cross(r, sub(states[k].velocity, tip_speed)))
}

/// What the link's own body adds to its mode's modal mass beyond the square of its share in `field`: of a bend's
/// m / 4 (the integral of the mode shape squared) the share gives 0.3915^2 m, of a twist's J / 3 (J about the link's
/// line) J / 4. Adding the rest keeps the modal masses positive definite with the joints'.
fn own_extra(link: &BentLink, states: &[BodyState]) -> f64 {
    let b = &states[link.body];
    match link.kind {
        FlexKind::Twist => {
            let axis = mul(&b.rot, link.dir);
            let turned = matmul(&matmul(&b.rot, &b.inertial.inertia), &transpose(&b.rot));
            let off = cross(axis, sub(b.com, b.origin));
            (dot(axis, mul(&turned, axis)) + b.inertial.mass * dot(off, off)) / 12.0
        }
        _ => b.inertial.mass * (0.25 - CANTILEVER_PARTICIPATION * CANTILEVER_PARTICIPATION),
    }
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

/// The parallelogram's bars (or any rigid piece the URDF leaves out) as bodies of their own: bar i is fixed to the link
/// of the URDF joint `arm.bars_on[i]` (1-based), its mass `bars_mass_kg[i]`, its centre `bars_com_m[3i..3i+3]` in that
/// link's frame and its inertia about its centre `bars_inertia_kgm2[6i..6i+6]` (xx, yy, zz, xy, xz, yz, in that frame;
/// the whole array may be empty: 0), merged into the link by the parallel axis theorem (`Inertial::merged`). A bar that
/// stays parallel to a link turns with it, so as a rigid piece of it it has exactly the bar's mass and turning inertia;
/// a coupler that only translates has inertia 0 and its mass at its centre. Empty arrays (the default): no bars.
fn bars(cfg: &Config, chain: &mut Chain) -> Result<(), ConfigError> {
    let Ok(on) = cfg.f64s("arm.bars_on") else { return Ok(()) };
    let bad = |key: &str| ConfigError::Invalid(format!("arm.{key}"));
    let (mass, com, inertia) =
        (cfg.f64s("arm.bars_mass_kg")?, cfg.f64s("arm.bars_com_m")?, cfg.f64s("arm.bars_inertia_kgm2")?);
    let n = on.len();
    if mass.len() != n || com.len() != 3 * n {
        return Err(bad("bars_mass_kg or arm.bars_com_m"));
    }
    if !inertia.is_empty() && inertia.len() != 6 * n {
        return Err(bad("bars_inertia_kgm2"));
    }
    let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for i in 0..n {
        if on[i].fract() != 0.0 || on[i] < 1.0 || on[i] > chain.len() as f64 {
            return Err(bad("bars_on"));
        }
        if !(mass[i] >= 0.0 && mass[i].is_finite()) || com[3 * i..3 * i + 3].iter().any(|v| !v.is_finite()) {
            return Err(bad("bars_mass_kg"));
        }
        let j = if inertia.is_empty() { [0.0; 6] } else { std::array::from_fn(|k| inertia[6 * i + k]) };
        let own = Inertial {
            mass: mass[i],
            com: [com[3 * i], com[3 * i + 1], com[3 * i + 2]],
            inertia: [[j[0], j[3], j[4]], [j[3], j[1], j[5]], [j[4], j[5], j[2]]],
        };
        let body = &mut chain.bodies[on[i] as usize - 1];
        body.inertial = body.inertial.merged(&own, &identity, [0.0; 3]);
    }
    Ok(())
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
    let mut chain = chain(cfg)?;
    bars(cfg, &mut chain)?;
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
    let rack = if cfg.get("rack").is_ok() { Some(RackPinion::from_config(cfg)?) } else { None };
    let screw = if rack.is_none() && prismatic.iter().any(|p| *p) { Some(BallScrew::from_config(cfg)?) } else { None };
    let mut racks = vec![None; n];
    let track = if cfg.get("track").is_ok() {
        // The guide loads take the first joint's frame as the carriage's: x along the rails, its origin between the
        // four blocks.
        if !prismatic[0] || coupling.is_some() || chain.bodies[0].axis != [1.0, 0.0, 0.0] {
            return Err(ConfigError::Invalid("track (the first joint slides along its frame's x)".into()));
        }
        // A config from before the rack's side (16 November 2026) has it on +y with its line through the centre.
        let side = if cfg.get("track.rack_side").is_ok() { cfg.f64("track.rack_side")? } else { 1.0 };
        if side.abs() != 1.0 {
            return Err(ConfigError::Invalid("track.rack_side".into()));
        }
        let lever = if cfg.get("track.rack_lever_m").is_ok() { cfg.non_negative("track.rack_lever_m")? } else { 0.0 };
        Some((cfg.positive("track.block_spacing_m")?, cfg.positive("track.rail_spacing_m")?, side, lever))
    } else {
        None
    };
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
        let (mut joint, lever) = match (&screw, &rack, prismatic[k]) {
            (_, Some(rack), true) => {
                // The pinion on the gearbox's output, as on the rack axis (`slide`), the ratio arm.gear_ratios'.
                let mut joint = RotaryJoint::linear_from_config(own)?;
                joint.gearbox.ratio = ratios[k];
                joint.coupled = ratios[k] == 1.0;
                rack.converter().apply(own, &mut joint)?;
                racks[k] =
                    Some((rack.clone(), joint.extra_efficiency, own.f64("ratings.shaft_max_radial_load_n").ok()));
                (joint, Some(rack.radius()))
            }
            (Some(screw), _, true) => {
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
                    if joint.friction.presliding > 0.0 {
                        joint.friction.stiction += screw.preload_torque() * screw.ratio();
                    }
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
    let (bending, reaction) = bent_links(cfg, &chain)?;
    let mut arm = Arm {
        chain,
        moving,
        payload,
        swing,
        g: gravity(cfg)?,
        attached: true,
        coupling,
        ratios: joints.iter().map(|(j, ..)| j.ratio()).collect(),
        rotor_inertia: joints.iter().map(|(j, ..)| j.rotor_inertia).collect(),
        prismatic,
        screw,
        racks,
        track,
        strain_waves,
        required_life: required_life(cfg),
        built: Vec::new(),
        max_condition,
        tool: Tool::Joints,
        bending,
        reaction,
        angular_limits: None,
    };
    arm.built = arm.chain.bodies.iter().map(|b| b.inertial).collect();
    if cfg.bool("kinematics.tool_space")? {
        let refuse = || ConfigError::Invalid("kinematics.tool_space".into());
        let names = cfg.get("kinematics.tool_axes")?.as_array().ok_or_else(refuse)?.clone();
        let picked: Vec<usize> = names
            .iter()
            .map(|v| match v.as_str() {
                Some("j1") => Some(FIRST_JOINT),
                s => s.and_then(|s| POSE_AXES.iter().position(|(id, ..)| *id == s)),
            })
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
                let mut pose = Pose::new(&fk, home, picked.clone(), length, max_condition).ok_or_else(refuse)?;
                match cfg.get("kinematics.redundancy")?.as_str() {
                    Some("home") => {}
                    Some("joint_limits") => {
                        pose = pose.away_from_limits(&joints.iter().map(|j| j.0.range).collect::<Vec<_>>());
                    }
                    _ => return Err(ConfigError::Invalid("kinematics.redundancy".into())),
                }
                arm.tool = Tool::Pose(pose);
                // The per-axis limits bound the rotation vector's rates; the spin they allow is bounded by these.
                let least = |key: &str| -> Result<Option<f64>, ConfigError> {
                    let turns = picked.iter().filter(|a| (3..6).contains(*a));
                    turns
                        .map(|&a| cfg.f64(&format!("axis.{}.{key}", POSE_AXES[a].0)))
                        .try_fold(None, |m: Option<f64>, v| v.map(|v| Some(m.map_or(v, |m| m.min(v)))))
                };
                if let (Some(v), Some(a)) = (least("max_velocity_rad_s")?, least("max_acceleration_rad_s2")?) {
                    arm.angular_limits = Some([v, a]);
                }
                let first = (AXIS_IDS[0], JOINT_KEYS[0], if arm.prismatic[0] { "m" } else { "rad" });
                picked.iter().map(|&a| if a == FIRST_JOINT { first } else { POSE_AXES[a] }).collect()
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

    /// The bodies of the chain the dynamics are of at the mechanism's joints `q` and speeds `qd` (`loaded`).
    fn states(&self, q: &[f64], qd: &[f64]) -> Vec<BodyState> {
        let ((chain, q), (_, qd)) = (self.loaded(q), self.loaded(qd));
        chain.body_states(&q, &qd, self.extra())
    }

    /// Over the bodies of `states`, each with its fields (`field`): per mode its momentum p = sum m t . v + r . I w,
    /// its load by gravity Q_g = sum m t . g, and the modal masses M_d = sum m t_i . t_j + r_i . I r_j (modes x modes,
    /// row by row, `own_extra` on the diagonal).
    fn modal(&self, states: &[BodyState]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        self.modal_in(states, &self.fields(states))
    }

    /// Every mode's `field` on every body of `states` (none before its link).
    fn fields(&self, states: &[BodyState]) -> Vec<Vec<(V3, V3)>> {
        self.bending
            .iter()
            .map(|l| {
                (0..states.len()).map(|k| if k < l.body { ([0.0; 3], [0.0; 3]) } else { field(l, states, k) }).collect()
            })
            .collect()
    }

    /// `modal` with the fields worked out already (the bodies where they were, at other speeds).
    fn modal_in(&self, states: &[BodyState], fields: &[Vec<(V3, V3)>]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let f = self.bending.len();
        let (mut p, mut gravity, mut mass) = (vec![0.0; f], vec![0.0; f], vec![0.0; f * f]);
        for (k, b) in states.iter().enumerate() {
            let inertia = matmul(&matmul(&b.rot, &b.inertial.inertia), &transpose(&b.rot));
            let m = b.inertial.mass;
            for i in 0..f {
                let (t, r) = fields[i][k];
                p[i] += m * dot(t, b.velocity) + dot(r, mul(&inertia, b.spin));
                gravity[i] -= m * self.g * t[2];
                for j in 0..f {
                    let (u, s) = fields[j][k];
                    mass[i * f + j] += m * dot(t, u) + dot(r, mul(&inertia, s));
                }
            }
        }
        for (i, link) in self.bending.iter().enumerate() {
            mass[i * f + i] += own_extra(link, states);
        }
        (p, gravity, mass)
    }

    /// The unit axis motor joint `k`'s rotor spins about in the base frame, for the bodies `bodies`: its joint's axis, a
    /// rack's motor standing upright on the carriage, its pinion turning about the vertical against the rack on the side
    /// of the rails.
    fn rotor_axis(&self, bodies: &[BodyState], k: usize) -> V3 {
        let body = &bodies[self.motor_body(k)];
        if self.racks[k].is_some() { mul(&body.rot, [0.0, 0.0, 1.0]) } else { body.axis }
    }

    /// The URDF joint (body) whose axis motor joint `k`'s motor turns about: the first it moves.
    fn motor_body(&self, k: usize) -> usize {
        match &self.coupling {
            None => k,
            Some(rows) => rows.iter().position(|r| r[k] != 0.0).unwrap_or(0),
        }
    }

    /// `[track]`: the load of the carriage's guides from the chain's reaction at its first joint, for the guide verdicts.
    /// What the base passes to the chain (carriage and all it carries, the payload with it) is the rate of the chain's
    /// momentum less gravity's pull: F = dP/dt + sum m g z, M_O = dH_O/dt + sum c x m g z about the base's origin (P and
    /// H by central differences of +-1 us along q' and q'', as `flex_loads`), taken to the carriage's origin between the
    /// blocks, M = M_O - o x F, and into its frame. Its x along the rails is the drive's; the rest the four blocks at
    /// (+-l/2, +-w/2) share as rigid supports: each carries F_z / 4 + M_x / (2 w) s_y - M_y / (2 l) s_x on its rail and
    /// F_y / 4 + M_z / (2 l) s_x across it (s the block's signs), its load the sum of both magnitudes (the catalogue's
    /// equivalent load with equal ratings both ways). Reported as the most loaded block's times the four blocks, so that
    /// the guide verdicts, which share the load equally among the set (`GUIDE_CARRIAGES`) and rate the set as one
    /// (`catalog` `parts.axis`), judge that block.
    /// The rack on the side s (`track.rack_side`, +1 or -1 along the carriage's y) at the lever l (`track.rack_lever_m`)
    /// takes part of what the base passes: the base's force F_y is the blocks' plus the rack's, and the rack's is the
    /// helical pinion's separating force F tan(alpha) / cos(beta) pushing the carriage off the rack, -s of it, so the
    /// blocks' F_y is the chain's plus s of it; the rack's push F_x, at y = s l, turns the carriage by -s l F_x about z,
    /// which the blocks carry as M_z + s l F_x (F_x the chain's reaction along the rails, the drive's).
    /// The motors' rotors are part of what the carriage carries: each spins at its ratio times its joint's speed about
    /// its axis (`rotor_axis`), its angular momentum J r q' along that axis is in H_O, so that the rate of H_O has the
    /// gyroscopic moment (the axis turning with its body) and the reaction of the rotor's own acceleration. The load is
    /// on the four blocks `[track]` describes (`GUIDE_CARRIAGES`), the first joint's carriage being the only guided one.
    fn guide_load(&self, q: &[f64], qd: &[f64], qdd: &[f64]) -> f64 {
        let Some((along, across, side, lever)) = self.track else { return 0.0 };
        let h = 1e-6;
        let momentum = |s: f64| {
            let q: Vec<f64> = q.iter().zip(qd).map(|(q, v)| q + s * v).collect();
            let qd: Vec<f64> = qd.iter().zip(qdd).map(|(v, a)| v + s * a).collect();
            let ((chain, q), (_, qm)) = (self.loaded(&q), self.loaded(&qd));
            let (p, mut h) = chain.momentum(&q, &qm, self.extra());
            let bodies = chain.body_states(&q, &qm, self.extra());
            for k in 0..self.ratios.len() {
                h = add(h, self.rotor_axis(&bodies, k).map(|a| a * self.rotor_inertia[k] * self.ratios[k] * qd[k]));
            }
            (p, h)
        };
        let ((p0, h0), (p1, h1)) = (momentum(-h), momentum(h));
        let rate = |a: V3, b: V3| -> V3 { std::array::from_fn(|i| (b[i] - a[i]) / (2.0 * h)) };
        let (mut force, mut moment) = (rate(p0, p1), rate(h0, h1));
        let states = self.states(q, qd);
        for b in &states {
            let lift = [0.0, 0.0, b.inertial.mass * self.g];
            force = add(force, lift);
            moment = add(moment, cross(b.com, lift));
        }
        let carriage = &states[0];
        let back = transpose(&carriage.rot);
        let (mut f, mut m) = (mul(&back, force), mul(&back, sub(moment, cross(carriage.origin, force))));
        m[2] += side * lever * f[0];
        // The helical pinion also pushes the carriage off the rack, across the rails (as `rack.<id>.radial_load` counts it).
        if let Some((r, efficiency, _)) = &self.racks[0] {
            let teeth = RackPinion::feed_force(f[0], qd[0], *efficiency);
            f[1] += side * (r.radial_load(teeth).powi(2) - teeth * teeth).max(0.0).sqrt();
        }
        let block = |sx: f64, sy: f64| {
            (f[2] / 4.0 + sy * m[0] / (2.0 * across) - sx * m[1] / (2.0 * along)).abs()
                + (f[1] / 4.0 + sx * m[2] / (2.0 * along)).abs()
        };
        let most =
            [block(1.0, 1.0), block(1.0, -1.0), block(-1.0, 1.0), block(-1.0, -1.0)].into_iter().fold(0.0, f64::max);
        GUIDE_CARRIAGES * most
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
            let axis = self.rotor_axis(&bodies, k);
            angular = add(angular, axis.map(|a| a * spin));
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

    /// Per mode the diagonal of `modal`'s M_d at zero joints: the link's own mass spread along it (a cantilever's m / 4)
    /// and everything the chain carries beyond it at its tip, with its turning inertia (a swinging payload too). The
    /// damping 2 zeta sqrt(m k).
    fn flex_modes(&self) -> Vec<FlexMode> {
        let (chain, zero) = self.loaded(&vec![0.0; self.ratios.len()]);
        let states = chain.body_states(&zero, &zero, self.extra());
        let whole = self.modal(&states).2;
        let f = self.bending.len();
        self.bending
            .iter()
            .enumerate()
            .map(|(i, l)| {
                let mass = whole[i * f + i];
                let stiffness = l.stiffness;
                let damping = 2.0 * l.damping_ratio * (mass * stiffness).sqrt();
                FlexMode { link: l.body, mass, stiffness, damping, kind: l.kind }
            })
            .collect()
    }

    /// Across each bending link the load on its mode: Q_g - dp/dt of `modal` (gravity's load less the rate of the mode's
    /// momentum, the bodies beyond it whole and its own body's share of the mode), p's rate by central differences over
    /// +-1 us along q' and q'' (`FlexCoupling`'s F with B q'' in it). A swinging payload as if fixed where it hangs.
    fn flex_loads(&self, q: &[f64], qd: &[f64], qdd: &[f64], out: &mut [f64]) {
        let h = 1e-6;
        let at = |s: f64| {
            let q: Vec<f64> = q.iter().zip(qd).map(|(q, v)| q + s * v).collect();
            let qd: Vec<f64> = qd.iter().zip(qdd).map(|(v, a)| v + s * a).collect();
            self.modal(&self.states(&q, &qd))
        };
        let ((_, gravity, _), (behind, ..), (ahead, ..)) = (at(0.0), at(-h), at(h));
        for (i, load) in out.iter_mut().enumerate() {
            *load = gravity[i] - (ahead[i] - behind[i]) / (2.0 * h);
        }
    }

    /// effects.link_bending_reaction: `FlexCoupling` from `modal` at the bodies' states. B's column j is p at unit speed
    /// of joint j; B' q' and M_d' by central differences over +-1 us along q'; G_j = sum_i d_i sum m g . dt_i/dq_j, the
    /// fields' rates (`field_rate`) at unit speed of joint j.
    fn flex_coupling(&self, q: &[f64], qd: &[f64], d: &[f64], dd: &[f64]) -> Option<FlexCoupling> {
        if !self.reaction {
            return None;
        }
        let (h, f, all) = (1e-6, self.bending.len(), q.len());
        let shifted = |s: f64| -> Vec<f64> { q.iter().zip(qd).map(|(q, v)| q + s * v).collect() };
        let now = self.states(q, qd);
        let fields = self.fields(&now);
        let (momentum, gravity, mass) = self.modal_in(&now, &fields);
        let ((behind, _, mass_behind), (ahead, _, mass_ahead)) =
            (self.modal(&self.states(&shifted(-h), qd)), self.modal(&self.states(&shifted(h), qd)));
        let force = (0..f)
            .map(|i| {
                let changing: f64 = (0..f).map(|k| (mass_ahead[i * f + k] - mass_behind[i * f + k]) * dd[k]).sum();
                gravity[i] - (ahead[i] - behind[i]) / (2.0 * h) - 0.5 * changing / (2.0 * h)
            })
            .collect();
        let (mut coupling, mut joint) = (vec![0.0; f * all], vec![0.0; all]);
        for j in 0..all {
            let unit: Vec<f64> = (0..all).map(|c| f64::from(u8::from(c == j))).collect();
            let states = self.states(q, &unit);
            let (column, ..) = self.modal_in(&states, &fields);
            for (i, link) in self.bending.iter().enumerate() {
                coupling[i * all + j] = column[i];
                if d[i] != 0.0 && self.g != 0.0 {
                    let lift: f64 = (link.body..states.len())
                        .map(|k| -states[k].inertial.mass * self.g * field_rate(link, &states, k)[2])
                        .sum();
                    joint[j] += d[i] * lift;
                }
            }
        }
        let potential = -(0..f).map(|i| d[i] * gravity[i]).sum::<f64>();
        Some(FlexCoupling { mass, coupling, momentum, force, joint, potential })
    }

    /// The flange moves with each bending link's tip and turns with its slope: by d across it and by 1.3765 d / L about
    /// dir x across, about the tip (a twist: about the link's line by d). In joint space the tool is the joints, which
    /// the bending does not move.
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
            let turn = mul(&rot, link.turn).map(|v| v * link.slope * d / link.span);
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
    /// M = C^T M_urdf C and h = C^T h_urdf (C constant, so no term of its own; the coupling's forces do no work). The
    /// parallelogram's bars are bodies of their own where the config has them (`arm.bars_*`, `bars`): their mass and
    /// inertia are merged into the links they move with.
    fn dynamics(&self, q: &[f64], qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        let all = q.len();
        let detached_swing = self.swing.is_some() && !(self.attached && self.payload.mass > 0.0);
        let moving = if detached_swing { &self.chain } else { &self.moving };
        // Uncoupled and without a swing the moving chain's joints are the mechanism's own.
        let joints;
        let (qm, qdm) = if self.coupling.is_none() && all == self.chain.len() {
            (q, qd)
        } else {
            joints = (self.moving_joints(q), self.moving_joints(qd));
            (&joints.0[..], &joints.1[..])
        };
        let l = moving.len();
        let (mut ml, mut hl) = (vec![0.0; l * l], vec![0.0; l]);
        moving.dynamics(&qm[..l], &qdm[..l], self.g, self.extra(), &mut ml, &mut hl);
        let n = self.ratios.len();
        // T: the moving chain's joints per mechanism joint (l x all).
        let tm: Vec<f64> = (0..l * all)
            .map(|rc| {
                let (r, c) = (rc / all, rc % all);
                match (&self.coupling, r < self.chain.len(), c < n) {
                    (Some(rows), true, true) => rows[r][c],
                    (None, true, true) => (r == c) as u8 as f64,
                    (_, false, false) => (r - self.chain.len() == c - n) as u8 as f64,
                    _ => 0.0,
                }
            })
            .collect();
        let t = |r: usize, c: usize| tm[r * all + c];
        // M_urdf T, once for every row of the result (l x all).
        let ml_t: Vec<f64> =
            (0..l * all).map(|rj| (0..l).map(|c| ml[rj / all * l + c] * t(c, rj % all)).sum()).collect();
        for i in 0..all {
            bias[i] = (0..l).map(|r| t(r, i) * hl[r]).sum();
            for j in 0..all {
                mass[i * all + j] = (0..l).map(|r| t(r, i) * ml_t[r * all + j]).sum();
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
    fn memory(&self) -> Vec<f64> {
        if let Tool::Pose(p) = &self.tool { p.memory() } else { Vec::new() }
    }
    fn set_memory(&self, memory: &[f64]) {
        if let Tool::Pose(p) = &self.tool {
            p.set_memory(memory)
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

    /// A rack's force on its teeth and the pinion's radial load on the gearbox shaft (`RackPinion::radial_load`), and a
    /// track's guide load (`guide_load`, as `axis.j1.guide_load`: the guide verdicts take the speed of the axis it names).
    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out = Vec::new();
        for k in 0..self.ratios.len() {
            let (id, key) = (MOTOR_IDS[k], JOINT_KEYS[k]);
            match (self.prismatic[k], &self.screw, &self.strain_waves[k], &self.racks[k]) {
                (_, _, _, Some((r, ..))) => out.extend([
                    r.signal_info(id, key),
                    SignalInfo::new(format!("rack.{id}.radial_load"), key, "quantity-radial-load", "N"),
                ]),
                (true, Some(s), ..) => out.extend(s.signal_infos(id, key)),
                (false, _, Some(g), _) => out.push(g.signal_info(id, key)),
                _ => {}
            }
        }
        if self.track.is_some() {
            out.push(SignalInfo::new("axis.j1.guide_load", JOINT_KEYS[0], "quantity-guide-load", "N"));
        }
        if self.angular_limits.is_some() {
            out.extend([
                SignalInfo::new("tool.angular_speed", "tool-orientation", "quantity-angular-speed", "rad/s"),
                SignalInfo::new(
                    "tool.angular_acceleration",
                    "tool-orientation",
                    "quantity-angular-acceleration",
                    "rad/s^2",
                ),
            ]);
        }
        out
    }
    fn signals(&self, q: &[f64], qd: &[f64], qdd: &[f64], force: &[f64], out: &mut Vec<f64>) {
        for k in 0..self.ratios.len() {
            match (self.prismatic[k], &self.screw, &self.strain_waves[k], &self.racks[k]) {
                (_, _, _, Some((r, efficiency, _))) => {
                    let teeth = RackPinion::feed_force(force[k], qd[k], *efficiency);
                    out.extend([teeth, r.radial_load(teeth)]);
                }
                (true, Some(s), ..) => s.signals(qd[k], force[k], out),
                (false, _, Some(g), _) => out.push(g.signal(force[k], qd[k] * self.ratios[k])),
                _ => {}
            }
        }
        if self.track.is_some() {
            out.push(self.guide_load(q, qd, qdd));
        }
        if let (Some(_), Tool::Pose(p)) = (self.angular_limits, &self.tool) {
            let (w, a) = p.angular_motion(&|q: &[f64]| self.tip(q), q, qd, qdd);
            out.extend([w, a]);
        }
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        let mut out = Vec::new();
        for k in 0..self.ratios.len() {
            if let Some((r, _, radial)) = &self.racks[k] {
                let id = MOTOR_IDS[k];
                out.extend(r.rating_checks(id, k));
                out.extend(radial.map(|limit| RatingCheck {
                    check: "gearbox-radial-load",
                    signal: format!("rack.{id}.radial_load"),
                    motor: Some(k),
                    limit,
                    life: false,
                }));
                continue;
            }
            match (self.prismatic[k], &self.screw, &self.strain_waves[k], self.required_life) {
                (true, Some(s), ..) => out.extend(s.rating_checks(MOTOR_IDS[k], k, self.required_life)),
                (false, _, Some(g), Some(life)) => out.push(g.rating_check(MOTOR_IDS[k], k, life)),
                _ => {}
            }
        }
        // Limits of the plan, as the rx, ry and rz limits they stand for: the time-optimal planner keeps them
        // (`rigid_checks` reads `plan.<signal>` as the rigid model's <signal>), a move timed by the axis limits too
        // (`MotionLaw::spin`), and the verdicts judge them on the setpoint's rigid motion (`analysis::Tally`). Judged on
        // the simulated pass they would count the gears' ringing: 22 rad/s2 on the arm's tool cycle, whose plan holds the
        // orientation (0 rad/s2), 6.4 with the joints rigid (the servo's steps).
        if let Some([speed, acceleration]) = self.angular_limits {
            let check = |check, signal: &str, limit| RatingCheck {
                check,
                signal: signal.into(),
                motor: None,
                limit,
                life: false,
            };
            out.extend([
                check("tool-angular-speed", "plan.tool.angular_speed", speed),
                check("tool-angular-acceleration", "plan.tool.angular_acceleration", acceleration),
            ]);
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
        if let (Some(_), Some((low, high))) = (self.track, self.chain.bodies[0].limits) {
            // The rails from one end of the travel to the other, under the carriage's origin.
            let end = |x: f64| add(self.chain.bodies[0].origin_pos, [x, 0.0, 0.0]);
            out.parts.push(Part::new(
                "rail",
                Shape::Segment { to: end(high), width: 0.08 },
                end(low),
                [0.62, 0.64, 0.68],
            ));
        }
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
        match &self.tool {
            Tool::Joints => Some(self.tip(tool).1),
            // After the first joint's own axis where it leads.
            Tool::Pose(p) if p.axes.first() == Some(&FIRST_JOINT) => Some([tool[1], tool[2], tool[3]]),
            _ => Some([tool[0], tool[1], tool[2]]),
        }
    }
}
