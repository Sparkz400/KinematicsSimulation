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
use sim_components::beam::{
    CANTILEVER_PARTICIPATION, CANTILEVER_TIP_SLOPE, cantilever_mode, cantilever_shape, tube_second_moment,
};
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
    /// Per motor joint: its rotors' angular momentum along `rotor_axis` per motor radian per second (kg m^2): the motor's
    /// rotor and the gearbox's input at the motor's speed, a rack's pinion at 1 / i of it (J_p / i, where the joint's
    /// inertia has it reflected, J_p / i^2), negative behind a strain wave gear, whose output turns against its input
    /// (circular spline fixed, flexspline output). `guide_load`'s gyroscopic moment and `frame_momentum`.
    spinning: Vec<f64>,
    /// Per motor joint: its rotor inertia as the joint has it (motor, gearbox input, a converter reflected), which
    /// `frame_momentum`'s rotor spin is the motor's speed times.
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
    /// The bars that are bodies of their own (`bars`), after the chain's bodies in `with_bars`.
    bars: Vec<Bar>,
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
// bend in its joint's plane too, takes the bodies beyond whole (`field`): their mass and their turning inertia, but for
// those a parallelogram holds (`held_from`).
#[derive(Debug, Clone, Copy)]
struct BentLink {
    body: usize,
    /// The first body whose orientation a parallelogram's bar holds against this mode (`parallelograms`): a bar pinned
    /// on a body before this link that turns with one beyond it drives that body's turn from before the link, so a bend
    /// in the joints' plane moves it and all beyond with the link's tip but does not turn them. `usize::MAX`: none.
    held_from: usize,
    /// Whether the mode bends in the plane the link's joint turns in (its turn about the joint's axis): the plane the
    /// parallelograms' pins turn in.
    in_plane: bool,
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
            let in_plane = matches!(kind, FlexKind::Bend);
            links.push(BentLink {
                body,
                held_from: usize::MAX,
                in_plane,
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
    if k >= link.held_from {
        return (u, [0.0; 3]);
    }
    let tip = add(b.origin, mul(&b.rot, link.dir.map(|v| v * link.length)));
    (add(u, cross(r, sub(states[k].com, tip))), r)
}

/// Mode `link`'s field on bar `bar` (its state `state` among `states`, the chain's bodies first): its pin, on its body,
/// moves as that body's points do (`field` there; on the bent link itself at the mode's shape where it sits along it,
/// `beam::cantilever_shape`, none behind the joint), and it turns as the body it turns with (`field`'s turn). A bar that
/// turns with the bent link itself while pinned beyond it is a parallelogram's side across from the link, its other end
/// before it: it turns with the link's chord, d / L about the mode's turn axis, in the joints' plane; pinned before the
/// link it does not turn.
fn bar_field(link: &BentLink, states: &[BodyState], bar: &Bar, state: &BodyState) -> (V3, V3) {
    let pin = state.origin;
    let b = &states[link.body];
    let shape = |at: V3| -> f64 {
        let along = dot(sub(at, b.origin), mul(&b.rot, link.dir)) / link.length;
        if along <= 0.0 { 0.0 } else { cantilever_shape(along.min(1.0)) }
    };
    let moved = |on: usize| -> V3 {
        match on.cmp(&link.body) {
            std::cmp::Ordering::Less => [0.0; 3],
            std::cmp::Ordering::Equal => match link.kind {
                FlexKind::Twist => {
                    let r = mul(&b.rot, link.turn);
                    let along = (dot(sub(pin, b.origin), mul(&b.rot, link.dir)) / link.length).clamp(0.0, 1.0);
                    cross(r.map(|v| v * along), sub(pin, b.origin))
                }
                _ => mul(&b.rot, link.across).map(|v| v * shape(pin)),
            },
            std::cmp::Ordering::Greater => {
                let (t, r) = field(link, states, on);
                add(t, cross(r, sub(pin, states[on].com)))
            }
        }
    };
    let at_pin = moved(bar.on);
    let turn = match bar.turns.cmp(&link.body) {
        std::cmp::Ordering::Less => [0.0; 3],
        std::cmp::Ordering::Equal if link.in_plane && bar.on > link.body => {
            mul(&b.rot, link.turn).map(|v| v / link.length)
        }
        std::cmp::Ordering::Equal => [0.0; 3],
        std::cmp::Ordering::Greater => field(link, states, bar.turns).1,
    };
    (add(at_pin, cross(turn, sub(state.com, pin))), turn)
}

/// The rate of `field` as the joints move at the speeds of `states`: everything of the link turns with its spin w,
/// and the body beyond moves against its tip.
fn field_rate(link: &BentLink, states: &[BodyState], k: usize) -> V3 {
    let b = &states[link.body];
    let w = b.spin;
    let (t, _) = field(link, states, k);
    if k == link.body || k >= link.held_from {
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

/// A bar of a parallelogram the URDF leaves out, which does not move as any one of its links does: pinned at `pin` of
/// body `on` (in its frame), it stays parallel to body `turns` and so turns with it; its centre `inertial.com` from the
/// pin and its inertia about its centre in the frame of `turns`. Its points move at v_pin + w_turns x (r - pin)
/// (`Arm::with_bars`): the rod that keeps a palletiser's forearm level is pinned on the forearm's rear lever and stays
/// parallel to the upper arm; the crank that drives it is pinned on the turret and turns with the forearm.
#[derive(Debug, Clone, Copy)]
struct Bar {
    on: usize,
    turns: usize,
    pin: V3,
    inertial: Inertial,
    /// A four-bar's coupler or follower (`four_bar`): its place and turn from the loop's closure, not from `turns`.
    closure: Option<Closure>,
}

/// A planar four-bar the URDF leaves out (`arm.four_bar_*`): a crank point C on body `crank` (at `crank_pin` of its
/// frame) drives a coupler of length l_c to the point T, which a follower of length l_f holds at l_f from its pivot F on
/// body `ground` (`Bar::on`, at `Bar::pin`), all in the plane square to `normal` (in the ground's frame). T from the
/// loop's closure: with d the in-plane part of C - F, D = |d|, e = d / D, cos a = (l_f^2 + D^2 - l_c^2) / (2 l_f D),
/// T = F + l_f (cos a e + branch sin a n x e). Its rates from the loop's Jacobian: in the ground's frame (turning at w_g)
/// C moves at c' = C' - F' - w_g x (C - F); the follower turns at lambda n with lambda = (T - C) . c' / ((T - C) . (n x
/// (T - F))) (the coupler keeps its length), the coupler at mu n with mu = (T'_rel - c') . (n x (T - C)) / l_c^2. Each
/// bar a uniform slender rod from its pin along its x (its mass at its middle, m l^2 / 12 across it).
#[derive(Debug, Clone, Copy)]
struct Closure {
    crank: usize,
    crank_pin: V3,
    normal: V3,
    coupler: f64,
    follower: f64,
    branch: f64,
    /// This bar is the coupler (from C to T); else the follower (from F to T).
    is_coupler: bool,
}

impl Closure {
    /// The loop's point T and the follower's and the coupler's turn rates about n relative to the ground (`Closure`),
    /// with C, F and n; `None` where it does not close.
    fn solve(&self, states: &[BodyState], ground: usize, pin: V3) -> Option<[V3; 7]> {
        let (g, c) = (&states[ground], &states[self.crank]);
        let n = mul(&g.rot, self.normal);
        let crank = add(c.origin, mul(&c.rot, self.crank_pin));
        let pivot = add(g.origin, mul(&g.rot, pin));
        let speed = |b: &BodyState, at: V3| add(b.velocity, cross(b.spin, sub(at, b.com)));
        let (cv, fv) = (speed(c, crank), speed(g, pivot));
        let raw = sub(crank, pivot);
        let d = sub(raw, n.map(|v| v * dot(n, raw)));
        let length = dot(d, d).sqrt();
        let e = d.map(|v| v / length);
        let cos = (self.follower.powi(2) + length * length - self.coupler.powi(2)) / (2.0 * self.follower * length);
        if !(cos.abs() <= 1.0) {
            return None;
        }
        let sin = self.branch * (1.0 - cos * cos).sqrt();
        let tip = add(pivot, add(e.map(|v| v * self.follower * cos), cross(n, e).map(|v| v * self.follower * sin)));
        let rel = sub(sub(cv, fv), cross(g.spin, sub(crank, pivot)));
        let (from_crank, from_pivot) = (sub(tip, crank), sub(tip, pivot));
        let lambda = dot(from_crank, rel) / dot(from_crank, cross(n, from_pivot));
        let tip_rel = cross(n, from_pivot).map(|v| v * lambda);
        let mu = dot(sub(tip_rel, rel), cross(n, from_crank)) / self.coupler.powi(2);
        Some([tip, crank, pivot, n, [lambda, mu, 0.0], cv, fv])
    }
}

/// The parallelogram's bars (or any rigid piece the URDF leaves out) as bodies of their own: bar i is pinned on the link
/// of the URDF joint `arm.bars_on[i]` (1-based) at `bars_pin_m[3i..3i+3]` of its frame (the array may be empty: the
/// link's origin) and turns with the link of `bars_turns_with[i]` (empty: with the link it is pinned on), its mass
/// `bars_mass_kg[i]`, its centre `bars_com_m[3i..3i+3]` from the pin and its inertia about its centre
/// `bars_inertia_kgm2[6i..6i+6]` (xx, yy, zz, xy, xz, yz; empty: 0), both in the frame of the link it turns with. A bar
/// that turns with the link it is pinned on is a rigid piece of it, merged into it by the parallel axis theorem
/// (`Inertial::merged`); the others are `Bar`s. Empty arrays (the default): no bars.
fn bars(cfg: &Config, chain: &mut Chain) -> Result<Vec<Bar>, ConfigError> {
    let Ok(on) = cfg.f64s("arm.bars_on") else { return four_bar(cfg, chain.len()) };
    let bad = |key: &str| ConfigError::Invalid(format!("arm.{key}"));
    let optional = |key: &str| if cfg.get(key).is_ok() { cfg.f64s(key) } else { Ok(Vec::new()) };
    let (mass, com, inertia) =
        (cfg.f64s("arm.bars_mass_kg")?, cfg.f64s("arm.bars_com_m")?, cfg.f64s("arm.bars_inertia_kgm2")?);
    let (turns, pin) = (optional("arm.bars_turns_with")?, optional("arm.bars_pin_m")?);
    let n = on.len();
    if mass.len() != n || com.len() != 3 * n {
        return Err(bad("bars_mass_kg or arm.bars_com_m"));
    }
    if !inertia.is_empty() && inertia.len() != 6 * n || inertia.iter().any(|v| !v.is_finite()) {
        return Err(bad("bars_inertia_kgm2"));
    }
    if !turns.is_empty() && turns.len() != n {
        return Err(bad("bars_turns_with"));
    }
    if !pin.is_empty() && pin.len() != 3 * n || pin.iter().any(|v| !v.is_finite()) {
        return Err(bad("bars_pin_m"));
    }
    let links = chain.len() as f64;
    let link = |v: f64, key: &str| {
        if v.fract() == 0.0 && v >= 1.0 && v <= links { Ok(v as usize - 1) } else { Err(bad(key)) }
    };
    let mut out = Vec::new();
    for i in 0..n {
        let on = link(on[i], "bars_on")?;
        let turns = if turns.is_empty() { on } else { link(turns[i], "bars_turns_with")? };
        if !(mass[i] >= 0.0 && mass[i].is_finite()) || com[3 * i..3 * i + 3].iter().any(|v| !v.is_finite()) {
            return Err(bad("bars_mass_kg"));
        }
        let j = if inertia.is_empty() { [0.0; 6] } else { std::array::from_fn(|k| inertia[6 * i + k]) };
        let pin = if pin.is_empty() { [0.0; 3] } else { [pin[3 * i], pin[3 * i + 1], pin[3 * i + 2]] };
        let own = Inertial {
            mass: mass[i],
            com: [com[3 * i], com[3 * i + 1], com[3 * i + 2]],
            inertia: [[j[0], j[3], j[4]], [j[3], j[1], j[5]], [j[4], j[5], j[2]]],
        };
        if turns == on {
            let body = &mut chain.bodies[on];
            body.inertial = body.inertial.merged(&own, &[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], pin);
        } else {
            out.push(Bar { on, turns, pin, inertial: own, closure: None });
        }
    }
    out.extend(four_bar(cfg, chain.len())?);
    Ok(out)
}

/// The four-bar of `arm.four_bar_*` (`Closure`), two `Bar`s: its coupler and its follower; none where
/// `arm.four_bar_ground_on` is left out. Keys: `four_bar_ground_on` and `four_bar_crank_on` (URDF joints, 1-based),
/// `four_bar_ground_pin_m` (F) and `four_bar_crank_pin_m` (C) in their links' frames, `four_bar_normal` (the loop's
/// plane's normal in the ground's frame), `four_bar_lengths_m` [l_c, l_f], `four_bar_masses_kg` [m_c, m_f],
/// `four_bar_branch` (+1 or -1: T on the side n x e of the line F to C, or the other); `four_bar_ground_on` 0: none.
fn four_bar(cfg: &Config, links: usize) -> Result<Vec<Bar>, ConfigError> {
    if cfg.get("arm.four_bar_ground_on").is_err() || cfg.f64("arm.four_bar_ground_on")? == 0.0 {
        return Ok(Vec::new());
    }
    let bad = |key: &str| ConfigError::Invalid(format!("arm.four_bar_{key}"));
    let link = |key: &str| -> Result<usize, ConfigError> {
        let v = cfg.f64(&format!("arm.four_bar_{key}"))?;
        if v.fract() == 0.0 && v >= 1.0 && v <= links as f64 { Ok(v as usize - 1) } else { Err(bad(key)) }
    };
    let point = |key: &str| -> Result<V3, ConfigError> {
        match cfg.f64s(&format!("arm.four_bar_{key}"))?[..] {
            [x, y, z] if [x, y, z].iter().all(|v| v.is_finite()) => Ok([x, y, z]),
            _ => Err(bad(key)),
        }
    };
    let (ground, crank, pin, crank_pin, normal) =
        (link("ground_on")?, link("crank_on")?, point("ground_pin_m")?, point("crank_pin_m")?, point("normal")?);
    let size = dot(normal, normal).sqrt();
    let (lengths, masses, branch) =
        (cfg.f64s("arm.four_bar_lengths_m")?, cfg.f64s("arm.four_bar_masses_kg")?, cfg.f64("arm.four_bar_branch")?);
    if !(size > 0.0) || branch.abs() != 1.0 {
        return Err(bad("normal or arm.four_bar_branch"));
    }
    let (&[coupler, follower], &[m_c, m_f]) = (&lengths[..], &masses[..]) else { return Err(bad("lengths_m")) };
    if !(coupler > 0.0 && follower > 0.0 && m_c >= 0.0 && m_f >= 0.0) {
        return Err(bad("lengths_m or arm.four_bar_masses_kg"));
    }
    let rod = |m: f64, l: f64| Inertial {
        mass: m,
        com: [0.5 * l, 0.0, 0.0],
        inertia: [[0.0; 3], [0.0, m * l * l / 12.0, 0.0], [0.0, 0.0, m * l * l / 12.0]],
    };
    let closure =
        Closure { crank, crank_pin, normal: normal.map(|v| v / size), coupler, follower, branch, is_coupler: true };
    Ok(vec![
        Bar { on: ground, turns: ground, pin, inertial: rod(m_c, coupler), closure: Some(closure) },
        Bar {
            on: ground,
            turns: ground,
            pin,
            inertial: rod(m_f, follower),
            closure: Some(Closure { is_coupler: false, ..closure }),
        },
    ])
}

/// effects.bar_elasticity: a parallelogram's rod stretches under the torque it passes. Bar i with an axial stiffness
/// `arm.bars_axial_stiffness_n_m[i]` k (0: rigid) carries the turn of motor joint `arm.bars_carries[i]` (1-based) on a
/// lever of c = |`bars_pin_m`| (its pin's distance from its link's joint), so its k c^2 (N m/rad at that joint) lies in
/// series with that joint's gearbox: K = 1 / (1 / k_g + 1 / (k c^2)), which the dynamics, the held modes and the
/// planner's trains then take. The rod's own mass stays on its bar; the gearbox's damper stays as it was (in parallel
/// with K, not in series with the rod), and so do a stiffness curve's upper segments. A config without the switch has
/// it off.
fn bar_springs(cfg: &Config, joints: &mut [(RotaryJoint, Option<f64>, &str)]) -> Result<(), ConfigError> {
    if !(cfg.get("effects.bar_elasticity").is_ok() && cfg.bool("effects.bar_elasticity")?) {
        return Ok(());
    }
    let bad = |key: &str| ConfigError::Invalid(format!("arm.{key}"));
    let (stiffness, carries, pins) =
        (cfg.f64s("arm.bars_axial_stiffness_n_m")?, cfg.f64s("arm.bars_carries")?, cfg.f64s("arm.bars_pin_m")?);
    if stiffness.is_empty() {
        return Ok(());
    }
    if carries.len() != stiffness.len() || pins.len() != 3 * stiffness.len() {
        return Err(bad("bars_carries"));
    }
    for (i, (k, motor)) in stiffness.iter().zip(&carries).enumerate() {
        if *k == 0.0 {
            continue;
        }
        let at = *motor as usize;
        if !(*k > 0.0) || motor.fract() != 0.0 || at == 0 || at > joints.len() {
            return Err(bad("bars_axial_stiffness_n_m or arm.bars_carries"));
        }
        let lever = pins[3 * i..3 * i + 3].iter().map(|v| v * v).sum::<f64>();
        if !(lever > 0.0) {
            return Err(bad("bars_pin_m"));
        }
        let gearbox = &mut joints[at - 1].0.gearbox;
        gearbox.stiffness = 1.0 / (1.0 / gearbox.stiffness + 1.0 / (k * lever));
    }
    Ok(())
}

/// The momentum of `bodies` about the base's origin: sum m v and sum c x m v + I w.
fn momentum_of(bodies: &[BodyState]) -> (V3, V3) {
    let (mut linear, mut angular) = ([0.0; 3], [0.0; 3]);
    for b in bodies {
        let p = b.velocity.map(|v| v * b.inertial.mass);
        let turned = matmul(&matmul(&b.rot, &b.inertial.inertia), &transpose(&b.rot));
        linear = add(linear, p);
        angular = add(angular, add(cross(b.com, p), mul(&turned, b.spin)));
    }
    (linear, angular)
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
    let bars = bars(cfg, &mut chain)?;
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
    let rack = if cfg.get("rack").is_ok() { Some(RackPinion::from_config(cfg)?) } else { None };
    let screw = if rack.is_none() && prismatic.iter().any(|p| *p) { Some(BallScrew::from_config(cfg)?) } else { None };
    // The gearbox lives take a motor's speed through its own `gearbox.ratio` (`analysis::output_rpm`, the motor's table
    // over the shared one), the arm its joint's `arm.gear_ratios`: where a joint with a gearbox (not a screw's coupling)
    // has another, a config that rates those lives is refused, as they would be worked out at another output speed.
    let rates_life = |own: &Config| {
        ["ratings.gearbox_reference_speed_rpm", "ratings.gearbox_gearing_wohler_exponent"]
            .iter()
            .any(|k| own.get(k).is_ok())
    };
    if (0..n).any(|k| {
        !(prismatic[k] && screw.is_some())
            && rates_life(&owns[k])
            && owns[k].f64("gearbox.ratio").ok() != Some(ratios[k])
    }) {
        return Err(ConfigError::Invalid("arm.gear_ratios (each joint's gearbox.<id>.ratio)".into()));
    }
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
        // What a prismatic joint moves at least: the bodies beyond it, without the payload (the damper of its screw's or
        // rack's spring, `Converter::apply_back`).
        let beyond: f64 = alone(k).map_or(0.0, |(i, _)| chain.bodies[i..].iter().map(|b| b.inertial.mass).sum());
        let (mut joint, lever) = match (&screw, &rack, prismatic[k]) {
            (_, Some(rack), true) => {
                // The pinion on the gearbox's output, as on the rack axis (`slide`), the ratio arm.gear_ratios'.
                let mut joint = RotaryJoint::linear_from_config(own)?;
                joint.gearbox.ratio = ratios[k];
                joint.coupled = ratios[k] == 1.0;
                rack.converter().apply_to(own, &mut joint, beyond)?;
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
                screw.converter(screw.softest_position()).apply_to(own, &mut joint, beyond)?;
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
    bar_springs(cfg, &mut joints)?;

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
    let (mut bending, reaction) = bent_links(cfg, &chain)?;
    // shortcut: a four-bar's bars have no field of the links' modes (`bar_field` takes parallel bars), so the two are
    // refused together; upgrade when a bent link carries a four-bar.
    if !bending.is_empty() && bars.iter().any(|b| b.closure.is_some()) {
        return Err(ConfigError::Invalid("arm.four_bar_* with effects.link_bending".into()));
    }
    // The parallelograms: a bar pinned on a body before a bent link that turns with a body beyond it drives that body's
    // turn from before the link, so the link's bend in the joints' plane moves the body and all beyond it with the
    // link's tip without turning them (`BentLink::held_from`, `field`); the bars' own fields `bar_field`.
    for link in bending.iter_mut().filter(|l| l.in_plane) {
        link.held_from = bars
            .iter()
            .filter(|b| b.on < link.body && link.body < b.turns)
            .map(|b| b.turns)
            .min()
            .unwrap_or(usize::MAX);
    }
    let mut arm = Arm {
        chain,
        moving,
        payload,
        swing,
        g: gravity(cfg)?,
        attached: true,
        coupling,
        ratios: joints.iter().map(|(j, ..)| j.ratio()).collect(),
        spinning: joints
            .iter()
            .enumerate()
            .map(|(k, (j, ..))| {
                let pinion = racks[k]
                    .as_ref()
                    .map_or(0.0, |(r, ..)| r.pinion_inertia * (1.0 / ratios[k] - 1.0 / ratios[k].powi(2)));
                (j.rotor_inertia + pinion) * if strain_waves[k].is_some() { -1.0 } else { 1.0 }
            })
            .collect(),
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
        bars,
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

    /// `states` (the chain's bodies) and after them each bar of its own (`Bar`): its pin on its body moving at
    /// v_pin = v_c + w x (pin - c) of that body, the bar turning with w' of the body it stays parallel to, its centre at
    /// pin + R' c_bar moving at v_pin + w' x (centre - pin).
    fn with_bars(&self, mut states: Vec<BodyState>) -> Vec<BodyState> {
        for bar in &self.bars {
            if let Some(c) = &bar.closure {
                let state = match c.solve(&states, bar.on, bar.pin) {
                    Some([tip, crank, pivot, n, [lambda, mu, _], cv, fv]) => {
                        let (start, speed, rate, length) =
                            if c.is_coupler { (crank, cv, mu, c.coupler) } else { (pivot, fv, lambda, c.follower) };
                        let x = sub(tip, start).map(|v| v / length);
                        let y = cross(n, x);
                        let rot = [[x[0], y[0], n[0]], [x[1], y[1], n[1]], [x[2], y[2], n[2]]];
                        let spin = add(states[bar.on].spin, n.map(|v| v * rate));
                        let com = add(start, x.map(|v| v * 0.5 * length));
                        let velocity = add(speed, cross(spin, sub(com, start)));
                        BodyState { inertial: bar.inertial, rot, origin: start, axis: [0.0; 3], com, velocity, spin }
                    }
                    None => BodyState {
                        inertial: bar.inertial,
                        rot: [[f64::NAN; 3]; 3],
                        origin: [f64::NAN; 3],
                        axis: [0.0; 3],
                        com: [f64::NAN; 3],
                        velocity: [f64::NAN; 3],
                        spin: [f64::NAN; 3],
                    },
                };
                states.push(state);
                continue;
            }
            let (a, b) = (&states[bar.on], &states[bar.turns]);
            let pin = add(a.origin, mul(&a.rot, bar.pin));
            let at_pin = add(a.velocity, cross(a.spin, sub(pin, a.com)));
            let com = add(pin, mul(&b.rot, bar.inertial.com));
            let velocity = add(at_pin, cross(b.spin, sub(com, pin)));
            let state = BodyState {
                inertial: bar.inertial,
                rot: b.rot,
                origin: pin,
                axis: [0.0; 3],
                com,
                velocity,
                spin: b.spin,
            };
            states.push(state);
        }
        states
    }

    /// The bars' share of the mass matrix and the bias (`dynamics`), by Kane's equations: a bar's v and w are linear in
    /// q' (J_j: at unit speed of joint j), so M_ij += m J_v,i . J_v,j + J_w,i . I J_w,j and h_i += J_v,i . m (a0 + g z) +
    /// J_w,i . (I a0_w + w x I w), with a0 and a0_w its accelerations at q'' = 0 (J' q': its speeds at q' while q moves
    /// by -+1 us of q', central differences) and I its inertia in the base frame.
    fn bar_dynamics(&self, q: &[f64], qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        let (all, h, n) = (q.len(), 1e-6, self.bars.len());
        let bars = |q: &[f64], qd: &[f64]| {
            let states = self.with_bars(self.states(q, qd));
            states[states.len() - n..].to_vec()
        };
        let shifted = |s: f64| -> Vec<f64> { q.iter().zip(qd).map(|(q, v)| q + s * v).collect() };
        let (now, behind, ahead) = (bars(q, qd), bars(&shifted(-h), qd), bars(&shifted(h), qd));
        let columns: Vec<Vec<BodyState>> =
            (0..all).map(|j| bars(q, &(0..all).map(|c| f64::from(u8::from(c == j))).collect::<Vec<_>>())).collect();
        for (b, bar) in now.iter().enumerate() {
            let m = bar.inertial.mass;
            let inertia = matmul(&matmul(&bar.rot, &bar.inertial.inertia), &transpose(&bar.rot));
            let rate = |x: V3, y: V3| -> V3 { std::array::from_fn(|i| (y[i] - x[i]) / (2.0 * h)) };
            let (a0, a0_w) = (rate(behind[b].velocity, ahead[b].velocity), rate(behind[b].spin, ahead[b].spin));
            let force = [m * a0[0], m * a0[1], m * (a0[2] + self.g)];
            let moment = add(mul(&inertia, a0_w), cross(bar.spin, mul(&inertia, bar.spin)));
            for i in 0..all {
                let (vi, wi) = (columns[i][b].velocity, columns[i][b].spin);
                bias[i] += dot(vi, force) + dot(wi, moment);
                for j in 0..all {
                    let (vj, wj) = (columns[j][b].velocity, columns[j][b].spin);
                    mass[i * all + j] += m * dot(vi, vj) + dot(wi, mul(&inertia, wj));
                }
            }
        }
    }

    /// Over the bodies of `states`, each with its fields (`field`): per mode its momentum p = sum m t . v + r . I w,
    /// its load by gravity Q_g = sum m t . g, and the modal masses M_d = sum m t_i . t_j + r_i . I r_j (modes x modes,
    /// row by row, `own_extra` on the diagonal).
    fn modal(&self, states: &[BodyState]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        self.modal_in(states, &self.fields(states))
    }

    /// Every mode's `field` on every body of `states` (none before its link), the bars' (`bar_field`) after them: `states`
    /// as `with_bars` gives them.
    fn fields(&self, states: &[BodyState]) -> Vec<Vec<(V3, V3)>> {
        let bodies = states.len() - self.bars.len();
        self.bending
            .iter()
            .map(|l| {
                (0..states.len())
                    .map(|k| match k {
                        _ if k >= bodies => bar_field(l, &states[..bodies], &self.bars[k - bodies], &states[k]),
                        _ if k < l.body => ([0.0; 3], [0.0; 3]),
                        _ => field(l, states, k),
                    })
                    .collect()
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

    /// The unit axis motor joint `k`'s rotor spins about in the base frame, for the bodies `bodies`, positive where the
    /// joint moves forward: its joint's axis; a rack's motor standing upright on the carriage, its pinion turning about
    /// the vertical against the rack on the side s of the rails (`track.rack_side`): the pinion's teeth on that side
    /// stand still against the rack, v + w z x (s r y) = 0, so w = s v / r about z.
    fn rotor_axis(&self, bodies: &[BodyState], k: usize) -> V3 {
        let body = &bodies[self.motor_body(k)];
        let side = self.track.map_or(1.0, |t| t.2);
        if self.racks[k].is_some() { mul(&body.rot, [0.0, 0.0, side]) } else { body.axis }
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
    /// its axis (`rotor_axis`), its angular momentum J r q' along that axis (J the rotors' `spinning`) is in H_O, so that the rate of H_O has the
    /// gyroscopic moment (the axis turning with its body) and the reaction of the rotor's own acceleration. The load is
    /// on the four blocks `[track]` describes (`GUIDE_CARRIAGES`), the first joint's carriage being the only guided one.
    fn guide_load(&self, q: &[f64], qd: &[f64], qdd: &[f64]) -> f64 {
        let Some((along, across, side, lever)) = self.track else { return 0.0 };
        let h = 1e-6;
        let momentum = |s: f64| {
            let q: Vec<f64> = q.iter().zip(qd).map(|(q, v)| q + s * v).collect();
            let qd: Vec<f64> = qd.iter().zip(qdd).map(|(v, a)| v + s * a).collect();
            let ((chain, q), (_, qm)) = (self.loaded(&q), self.loaded(&qd));
            let bodies = self.with_bars(chain.body_states(&q, &qm, self.extra()));
            let (p, mut h) = momentum_of(&bodies);
            for k in 0..self.ratios.len() {
                h = add(h, self.rotor_axis(&bodies, k).map(|a| a * self.spinning[k] * self.ratios[k] * qd[k]));
            }
            (p, h)
        };
        let ((p0, h0), (p1, h1)) = (momentum(-h), momentum(h));
        let rate = |a: V3, b: V3| -> V3 { std::array::from_fn(|i| (b[i] - a[i]) / (2.0 * h)) };
        let (mut force, mut moment) = (rate(p0, p1), rate(h0, h1));
        let states = self.with_bars(self.states(q, qd));
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
    /// each motor's rotor spinning about the axis of the joint it turns (`motor_body`, `rotor_axis`), its momentum the
    /// rotors' `spinning` at the motor's speed.
    fn frame_momentum(&self, q: &[f64], qd: &[f64], rotor_spin: &[f64]) -> Option<sim_core::FrameMomentum> {
        let ((chain, q), (_, qd)) = (self.loaded(q), self.loaded(qd));
        let bodies = self.with_bars(chain.body_states(&q, &qd, self.extra()));
        let (linear, mut angular) = momentum_of(&bodies);
        let (mut mass, mut first_moment) = (0.0, [0.0; 3]);
        for b in &bodies {
            mass += b.inertial.mass;
            first_moment = add(first_moment, b.com.map(|c| c * b.inertial.mass));
        }
        for (k, spin) in rotor_spin.iter().enumerate() {
            let axis = self.rotor_axis(&bodies, k);
            angular = add(angular, axis.map(|a| a * spin * self.spinning[k] / self.rotor_inertia[k]));
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
        let states = self.with_bars(chain.body_states(&zero, &zero, self.extra()));
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
            self.modal(&self.with_bars(self.states(&q, &qd)))
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
        let now = self.with_bars(self.states(q, qd));
        let fields = self.fields(&now);
        let (momentum, gravity, mass) = self.modal_in(&now, &fields);
        let ((behind, _, mass_behind), (ahead, _, mass_ahead)) = (
            self.modal(&self.with_bars(self.states(&shifted(-h), qd))),
            self.modal(&self.with_bars(self.states(&shifted(h), qd))),
        );
        let force = (0..f)
            .map(|i| {
                let changing: f64 = (0..f).map(|k| (mass_ahead[i * f + k] - mass_behind[i * f + k]) * dd[k]).sum();
                gravity[i] - (ahead[i] - behind[i]) / (2.0 * h) - 0.5 * changing / (2.0 * h)
            })
            .collect();
        let (mut coupling, mut joint) = (vec![0.0; f * all], vec![0.0; all]);
        let bodies = now.len() - self.bars.len();
        for j in 0..all {
            let unit: Vec<f64> = (0..all).map(|c| f64::from(u8::from(c == j))).collect();
            let states = self.with_bars(self.states(q, &unit));
            let (column, ..) = self.modal_in(&states, &fields);
            // The bars' fields' rates along joint j by central differences over +-1 us at its unit speed.
            let bar_rates = (!self.bars.is_empty() && self.g != 0.0).then(|| {
                let at = |s: f64| {
                    self.fields(&self.with_bars(
                        self.states(&q.iter().zip(&unit).map(|(q, u)| q + s * u).collect::<Vec<_>>(), &unit),
                    ))
                };
                (at(-h), at(h))
            });
            for (i, link) in self.bending.iter().enumerate() {
                coupling[i * all + j] = column[i];
                if d[i] != 0.0 && self.g != 0.0 {
                    let mut lift: f64 = (link.body..bodies)
                        .map(|k| -states[k].inertial.mass * self.g * field_rate(link, &states, k)[2])
                        .sum();
                    if let Some((behind, ahead)) = &bar_rates {
                        lift -= (bodies..states.len())
                            .map(|k| {
                                states[k].inertial.mass * self.g * (ahead[i][k].0[2] - behind[i][k].0[2]) / (2.0 * h)
                            })
                            .sum::<f64>();
                    }
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
            // A flange a parallelogram holds against the mode moves with the tip and does not turn (`held_from`).
            let held = frames.len() - 1 >= link.held_from;
            let turn = mul(&rot, link.turn).map(|v| if held { 0.0 } else { v * link.slope * d / link.span });
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
    /// parallelogram's bars where the config has them (`arm.bars_*`, `bars`): a rigid piece of a link merged into it,
    /// a bar of its own with its share (`bar_dynamics`).
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
        if !self.bars.is_empty() {
            self.bar_dynamics(q, qd, mass, bias);
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
        let bars = if self.bars.is_empty() {
            0.0
        } else {
            let states = self.with_bars(self.states(q, &vec![0.0; q.len()]));
            states[states.len() - self.bars.len()..].iter().map(|b| b.inertial.mass * self.g * b.com[2]).sum()
        };
        bars + match self.swing.is_some() && self.attached {
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
