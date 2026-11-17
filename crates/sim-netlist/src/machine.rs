//! `ConfigMachine`: a belt-driven machine of any routing, built from a netlist (`Netlist`).
//!
//! The machine has n axes (tool coordinates q) and as many motors (one more per drive of a gantry beyond the first),
//! and with a yawing gantry one more coordinate, its yaw, after the axes. Each motor turns one pulley through its gearbox.
//! Belt section s has the geometric length L_s = L0_s + sum_i c_si q_i (plus the teeth in mesh at each pulley it ends
//! at, see `Belt::tooth_length`), and pulley k feeds f_sk r_k of belt into it per radian (f = +1 or -1). So:
//!
//! * the belt pulls axis i with -sum_s c_si T_s, and turns pulley k with r_mat,k sum_s f_sk M_s (M the force on the belt
//!   material, `SectionForce::material`);
//! * in the rigid model the belt keeps its length: C q = F diag(r) a, so the pulley angles are a = K q with
//!   K = (F diag(r))^+ C, which must reproduce C exactly (else the routing is refused) and be invertible;
//! * bodies move with their axes, so the mass on axis i is the sum of the bodies that move with it; the axes are
//!   taken square to each other, so the bodies' mass matrix is diagonal. Gravity pulls axis i with m_i g sin(incline_i).
//!   A body that rides on rotary axes (or across a yawing gantry's bridge) is placed by `body_point` instead, and adds
//!   m J^T J, -m g J_z and -m J^T (dJ/dt v) (its place's Jacobian J by differences): Lagrange's equations.
//! * a gantry's yaw (`machine.gantry`, effects.gantry_yaw): the coordinate psi with its own inertia, spring and damper;
//!   the sections that pull the bridge's ends get longer by their yaw coefficient per rad, a drive at its side has the
//!   row e_axis + side e_psi in K. Where fewer motors than coordinates set it, the motors set the axes (`k_inv` from
//!   their columns) and the spring the yaw; where the belts cannot keep their length with it, the rigid model has none.
//! * a tool centre point (`machine.tcp`): the tool pose is the tip's, the axes stand at the tip less the tool turned by
//!   the rotaries (`joints`, `tool_of`).
//!
//! The effects of the shipped crates, written for any routing:
//!
//! * idlers (`machine.idler`): idler d passes belt at the speed s_d . v (s_d its `speed` per axis), so its inertia
//!   J_d / r_d^2 adds m_d s_d s_d^T to the axes' mass matrix (which is then no longer diagonal), and its bearing
//!   friction torque F_d r_d pulls the axes back with F_d tanh(s_d . v / (w_s r_d)) s_d (w_s the motors' friction
//!   smoothing speed, as for the gantry's idlers). They carry no weight.
//! * a cable chain on an axis (`cable_chain = true`): mass m(q) = mu (L0 + q / 2) that moves with the axis, which
//!   takes m dv/dt + m' v^2 / 2, and a drag that fades out below the axis guide's Stribeck speed (`mech::CableChain`).
//! * a process force per axis, `process.force_<axis>_n`, on the tool while the cycle dwells.
//! * the frame (`machine.frame`, elastic model only): it sways by u along a horizontal axis f on its spring. The axes
//!   are measured on the frame, so the bodies move at v + e_f du/dt over ground: M (a + e_f u'') = F, and the frame
//!   takes the forces its belt, guides, idlers, chain and stops put on them back, m_frame u'' = -k u - c u' - F_f
//!   (less the process force, which the workpiece takes, and plus the chain's -m' w^2 / 2, w the chain's speed over
//!   ground). Bodies that do not move along f ride on the frame: their mass sways with it.
//! * thermal feedback: the gearboxes' efficiency and the guides' viscous friction follow their temperature
//!   (`mech::ThermalFeedback`), the belt's stiffness the ambient one (in `Belt`).
//! * belt tooth jump (elastic model only): at each driven pulley between a section it feeds (+1) and one it takes from
//!   (-1), `mech::ToothJump`; the slip u of the belt over pulley k feeds u of belt into its +1 section.
//! * span vibration (`machine.span`): each free span's first sideways mode at its section's tension, against what the
//!   drives excite (`mech::span_frequency`, `drive_excitations`).
//!
//! Each motor's drive train (rotor, gearbox, motor friction, ripple, brake) is read from its own config
//! (`catalog::motor_config`): the shared `[motor]` and `[gearbox]` with its `[motor.<id>]`, `[gearbox.<id>]` over them.
//!
//! Two of the registered belt machines are this with C = [[1], [-1]] and F = [[1], [-1]], and with
//! C = [[1, 1], [0, -2], [-1, 1]] and F = [[1, 0], [-1, 1], [0, -1]]; `machines/*.toml` hold them, and
//! crates/sim-machines/tests/machine_files.rs checks them against their crates. The equations are those crates',
//! written for any C and F: their module comments derive them.
//!
//! A motor may drive an axis without a belt (`Direct`): a ball screw or a rack behind its gearbox, a linear motor, or a
//! rotary table (an axis in rad). Its drive train is the registered one-axis machines' (`RotaryJoint` with the screw's or rack's
//! `transmission::Converter` behind the gearbox): the gearbox is then one in the axis's unit, of ratio i / lever, its
//! stiffness the gearbox's and the converter's in series, its efficiency both, the screw's or pinion's inertia on the
//! rotor. Its "pulley" is the axis itself: K has a row e_i for it, the axis takes the gearbox's output as its force,
//! and in the elastic model the pulley's states follow the axis. So the registered screw, rack, linear motor and table axes are this with no
//! belt (crates/sim-machines/tests/machine_files.rs checks the screw axis against its crate). A rotary axis has no
//! body: its rotary's inertia (and payload's) is its mass, no gravity along it.

use crate::geometry::{Wheel, arc, contacts, flex_at, lean_reach, lean_weight, rise_coefficients, wrap};
use crate::netlist::{LeanNode, Netlist, SpanNode, TcpNode, Transmission, Waypoint, WormNode};
use sim_components::catalog::motor_config;
use sim_components::joint::RotaryJoint;
use sim_components::mech::{
    Belt, CableChain, GearTorque, Gearbox, HardStop, HoldingBrake, JUMP_STATES, Lugre, Ripple, SectionForce,
    ShaftFriction, ThermalFeedback, ToothJump, ToothMesh, drive_excitations, span_detuning, span_frequency,
};
use sim_components::transmission::{BallScrew, LinearMotor, RackPinion, StrainWave};
use sim_core::scene::{MOTOR_PART, drive_unit, idler};
use sim_core::{
    AxisInfo, BeltSection, Config, ConfigError, ENERGY_PARTS, FRICTION_SIGNALS, FrictionContact, LOSS_PREFIX, Label,
    Machine, MotorInfo, Part, RatingCheck, Scene, Shape, SignalInfo, TORQUE_PARTS, ThermalPart,
};

const G: f64 = 9.80665;
/// Straight pieces the 3D view draws a belt's arc round a wheel with.
const ARC_PIECES: usize = 8;

/// Where the 3D view puts a body with every axis at 0 (the bodies' places, `body_point`, start at the origin).
fn scene_origin() -> [f64; 3] {
    [0.0, -0.03, 0.06]
}

/// A `&'static str` for an id or key read from a file, which `AxisInfo` and the other machine descriptions take.
/// Each distinct text is leaked once, so rebuilding a machine on every config change does not grow memory.
pub fn intern(text: &str) -> &'static str {
    use std::collections::HashSet;
    use std::sync::{Mutex, OnceLock};
    static TEXTS: OnceLock<Mutex<HashSet<&'static str>>> = OnceLock::new();
    let mut texts = TEXTS.get_or_init(Default::default).lock().unwrap_or_else(|e| e.into_inner());
    if let Some(known) = texts.get(text) {
        return known;
    }
    let leaked: &'static str = Box::leak(text.to_string().into_boxed_str());
    texts.insert(leaked);
    leaked
}

struct Section {
    id: &'static str,
    key: &'static str,
    /// Length at q = 0 with the teeth in mesh, and that without them; per axis and per motor.
    length: f64,
    bare: f64,
    c: Vec<f64>,
    f: Vec<f64>,
    /// Unstretched belt at q = 0 with the pulleys at angle 0.
    mat0: f64,
    route: Vec<Waypoint>,
}

struct Body {
    mass: f64,
    payload: f64,
    axes: Vec<bool>,
    /// The rotary axes it rides on, outermost first, and its centre of mass from where their axes meet at angle 0.
    rotaries: Vec<usize>,
    offset: [f64; 3],
    /// Its place is not a sum of the axes' travels (it rides on a rotary, or across a yawing bridge): its share of the
    /// mass matrix, gravity and the forces of its speed come from its place's Jacobian (`body_point`).
    general: bool,
}

/// A gantry's bridge that yaws (`machine.gantry`, effects.gantry_yaw): its yaw is the coordinate `coord`, after the
/// axes. A body that moves with `axis` and along `across` sits `q_across + shift` from the bridge's centre, so the yaw
/// moves it along `axis` by that times the yaw angle (small angles).
struct Gantry {
    id: &'static str,
    coord: usize,
    axis: usize,
    across: Option<usize>,
    shift: f64,
    width: f64,
    stiffness: f64,
    damping: f64,
    tolerance: Option<f64>,
}

struct Guide {
    /// Its id, for its loss (`power.<id>`) and its warmth, `axis.<axis id>` for its friction signals.
    loss_id: &'static str,
    contact: &'static str,
    axis: usize,
    friction: Lugre,
    carries: Vec<usize>,
    overhang: Option<crate::netlist::Overhang>,
}

struct Idler {
    /// J / r^2 and the belt force its bearing takes, both 0 with effects.idlers off; the belt speed below which that
    /// fades out.
    mass: f64,
    drag: f64,
    smoothing: f64,
    radius: f64,
    speed: Vec<f64>,
    body: Option<usize>,
    at: Option<[f64; 3]>,
}

struct FrameMode {
    axis: usize,
    mass: f64,
    stiffness: f64,
    damping: f64,
}

/// The beam under the carriage's axis (effects.beam_vertical_mode, `machine.lean` names the axes): a simple span of `span`
/// m and bending stiffness `ei`, the carriage `start` m from the left post at 0, whose first vertical mode with the carriage
/// on it is one more coordinate w, the beam's deflection under the carriage, down. Shape: the span's static deflection
/// under a point load at the carriage, taken to 1 under it: stiffness k = 1 / `span_compliance` (so that m g / k is the
/// quasi-static sag, exactly), modal mass M = mu integral of the shape squared (`BeamMode::modal_mass`, 17 mu L / 35 in
/// the middle), damping c = 2 zeta sqrt(k M). The bodies that move along the beam's axis hang w lower with it (mass
/// sum m + M), and those that also move along the vertical axis have their speed over ground there as z' - w' (the axis's
/// own coordinate is relative to the carriage), so a push on the vertical axis drives the beam through -m_z between z and
/// w. V = 1/2 k(x) w^2 - m g w; T keeps 1/2 M(x) w'^2 and drops what the shape's change with x adds (second order in the
/// deflection, as in the crate of the T-shaped belt gantry), so on x: 1/2 M' w'^2 - 1/2 k' w^2, on w: -M' x' w'. The belt's
/// own runs' mass is not on w. The tool hangs w lower: pos_vertical -= w.
struct BeamMode {
    /// The coordinate along the beam, and the vertical one.
    axis: usize,
    vertical: usize,
    span: f64,
    start: f64,
    ei: f64,
    mass_per_m: f64,
    damping_ratio: f64,
}

impl BeamMode {
    /// The carriage's distance from the left post at x, kept off the posts where the span's stiffness is infinite.
    fn at(&self, x: f64) -> f64 {
        (self.start + x).clamp(1e-3 * self.span, (1.0 - 1e-3) * self.span)
    }
    fn stiffness(&self, x: f64) -> f64 {
        1.0 / sim_components::beam::span_compliance(self.ei, self.span, self.at(x))
    }
    /// The modal mass at x: with the load at a (b = L - a) the shape is phi = s (L^2 - b^2 - s^2) / (2 a^2 b) left of it,
    /// mirrored right of it; with I(a, c) = integral over 0..a of s^2 (c - s^2)^2 = c^2 a^3 / 3 - 2 c a^5 / 5 + a^7 / 7:
    /// M = mu (I(a, L^2 - b^2) / (4 a^4 b^2) + I(b, L^2 - a^2) / (4 a^2 b^4)). In the middle 17 mu L / 35.
    fn modal_mass(&self, x: f64) -> f64 {
        let (a, l) = (self.at(x), self.span);
        let b = l - a;
        let i = |a: f64, c: f64| c * c * a.powi(3) / 3.0 - 2.0 * c * a.powi(5) / 5.0 + a.powi(7) / 7.0;
        self.mass_per_m
            * (i(a, l * l - b * b) / (4.0 * a.powi(4) * b * b) + i(b, l * l - a * a) / (4.0 * a * a * b.powi(4)))
    }
    /// (stiffness k, modal mass M, damping c, dk/dx, dM/dx) at x; the slopes by central differences over 1e-6 m.
    fn at_x(&self, x: f64) -> [f64; 5] {
        let (k, mass) = (self.stiffness(x), self.modal_mass(x));
        let slope = |f: &dyn Fn(f64) -> f64| (f(x + 1e-6) - f(x - 1e-6)) / 2e-6;
        [
            k,
            mass,
            2.0 * self.damping_ratio * (k * mass).sqrt(),
            slope(&|x| self.stiffness(x)),
            slope(&|x| self.modal_mass(x)),
        ]
    }
}

struct Span {
    key: &'static str,
    section: usize,
    length: f64,
    c: Vec<f64>,
    /// Its belt's speed over ground along x, y, z per unit speed of each coordinate.
    velocity: Option<[Vec<f64>; 3]>,
}

/// What a motor drives without a belt: an axis, through its gearbox in the axis's unit (see the module comment).
struct Direct {
    axis: usize,
    /// Where it acts across a yawing gantry's bridge, from its centre (0 elsewhere): its output travels with the yaw too.
    side: f64,
    kind: DirectKind,
    /// Travel per radian of the gearbox's own output (screw, rack): the torque there is the force times it.
    lever: Option<f64>,
    /// `ratings.required_life_h` of its motor, where set.
    required_life: Option<f64>,
    /// `ratings.shaft_max_radial_load_n` of its motor, where set: what a rack's pinion may load the gearbox's output
    /// shaft with sideways (`RackPinion::radial_load`).
    radial_limit: Option<f64>,
}

enum DirectKind {
    Screw(BallScrew),
    Rack(RackPinion),
    /// The attraction between forcer and magnets and the forcer's mass, which press on the axis's guide.
    Linear {
        attraction: f64,
        forcer: f64,
    },
    Rotary(Option<StrainWave>),
}

/// The motor's config with its `[<layout>]` (`screw`, `rack`, ...) replaced by the file's table at `table`: what the
/// transmissions' `from_config` reads.
fn values_config(own: &Config, cfg: &Config, layout: &str, table: &str) -> Result<Config, ConfigError> {
    let mut all: toml::Table = own.to_text()?.parse().map_err(|_| ConfigError::Parse(table.into()))?;
    all.insert(layout.into(), cfg.get(table)?.clone());
    Config::parse(&toml::to_string(&all).map_err(|e| ConfigError::Parse(e.to_string()))?)
}

/// Per motor its worm stage while effects.gearbox_efficiency counts it. Its gearbox then keeps only what a screw or rack
/// behind it loses (`extra_efficiency`, 1 for none): `worm::factors` gives the worm's share. With
/// effects.thermal_feedback its efficiencies follow its temperature (`set_part_temperatures`, `worm::warmed`).
fn worm_stages(
    cfg: &Config,
    net: &Netlist,
    gears: &mut [Gearbox],
    extra: &[f64],
) -> Result<Vec<Option<WormNode>>, ConfigError> {
    let counts = cfg.bool("effects.gearbox_efficiency")?;
    Ok(net
        .motors
        .iter()
        .zip(gears.iter_mut().zip(extra))
        .map(|(d, (gear, extra))| {
            let worm = d.worm.filter(|_| counts)?;
            gear.efficiency = *extra;
            Some(worm)
        })
        .collect())
}

/// With effects.belt_wrap: per motor that drives a pulley the wheels next to it along the belt, the one before it in the
/// route of the section that ends at it and the one after it in the route of the section that starts at it
/// (`belt_wraps`); a motor with a screw, rack, linear motor or rotary has no wrap. Refused where a pulley does not end
/// one route and start another, or it or a wheel next to it has no place (`at_m`).
fn wrap_neighbours(net: &Netlist, direct: &[Option<Direct>]) -> Result<Vec<(usize, Waypoint, Waypoint)>, ConfigError> {
    let refuse = |why: String| ConfigError::Invalid(format!("effects.belt_wrap ({why})"));
    let (placed, name) = (
        |w: Waypoint| match w {
            Waypoint::Pulley(kk) => net.motors[kk].at.is_some(),
            Waypoint::Idler(d) => net.idlers[d].at.is_some(),
            Waypoint::Body(_) => true,
        },
        |w: Waypoint| match w {
            Waypoint::Pulley(kk) => net.motors[kk].pulley.clone(),
            Waypoint::Idler(d) => net.idlers[d].id.clone(),
            Waypoint::Body(b) => net.bodies[b].id.clone(),
        },
    );
    let mut out = Vec::new();
    for (kk, d) in net.motors.iter().enumerate().filter(|(kk, _)| direct[*kk].is_none()) {
        let me = Waypoint::Pulley(kk);
        let ends: Vec<&Vec<Waypoint>> =
            net.sections.iter().map(|s| &s.route).filter(|r| r.last() == Some(&me)).collect();
        let starts: Vec<&Vec<Waypoint>> =
            net.sections.iter().map(|s| &s.route).filter(|r| r.first() == Some(&me)).collect();
        let before = match ends[..] {
            [route] => route.iter().rev().nth(1),
            _ => None,
        };
        let after = match starts[..] {
            [route] => route.get(1),
            _ => None,
        };
        let (Some(&before), Some(&after)) = (before, after) else {
            return Err(refuse(format!(
                "pulley {} has to end the route of one section and start the route of another",
                d.pulley
            )));
        };
        if let Some(w) = [me, before, after].into_iter().find(|w| !placed(*w)) {
            return Err(refuse(format!("{} has no at_m", name(w))));
        }
        out.push((kk, before, after));
    }
    Ok(out)
}

/// An error about `<layout>.<key>` as one about the file's `<table>.<key>`.
fn renamed(e: ConfigError, layout: &str, table: &str) -> ConfigError {
    let fix = |d: String| match d.strip_prefix(&format!("{layout}.")) {
        Some(rest) => format!("{table}.{rest}"),
        None => d,
    };
    match e {
        ConfigError::Missing(d) => ConfigError::Missing(fix(d)),
        ConfigError::Type(d) => ConfigError::Type(fix(d)),
        ConfigError::Invalid(d) => ConfigError::Invalid(fix(d)),
        other => other,
    }
}

pub struct ConfigMachine {
    elastic: bool,
    axes: Vec<AxisInfo>,
    motors: Vec<MotorInfo>,
    /// Per coordinate: gravity along it (pulling towards -) and across it; and gravity itself (0 with effects.gravity off).
    g_along: Vec<f64>,
    g: f64,
    g_across: Vec<f64>,
    /// The closed belts: each the sections of one belt whose every pulley feeds it on one side and takes it out on
    /// another, so that the belt's material in them stays what it is. A belt that ends on a pulley (a drum) has
    /// its material wound on and off, and no such sum.
    loops: Vec<Vec<usize>>,
    sections: Vec<Section>,
    bodies: Vec<Body>,
    guides: Vec<Guide>,
    idlers: Vec<Idler>,
    /// Elastic model: a rotary axis's bearing friction at the output, per axis [Coulomb N m, viscous N m s/rad, smoothing
    /// rad/s]: it acts behind the gearbox's spring and efficiency (the rigid model puts it at the rotor, c / i + v w / i^2).
    bearing: Vec<[f64; 3]>,
    /// Pulley angle per metre of each axis (motors x axes), and its inverse.
    k: Vec<Vec<f64>>,
    k_inv: Vec<Vec<f64>>,
    /// Per motor: pulley radius, unstretched belt fed per radian, pulley inertia with its share of the belt, where the
    /// 3D view draws the pulley.
    r: Vec<f64>,
    r_mat: Vec<f64>,
    j_pulley: Vec<f64>,
    pulley_at: Vec<Option<[f64; 3]>>,
    /// Per motor: rotor and gearbox input inertia, the gearbox ratio, its gearbox, shaft friction, ripple and brake.
    j_rotor: Vec<f64>,
    ratio: Vec<f64>,
    gears: Vec<Gearbox>,
    shafts: Vec<ShaftFriction>,
    ripples: Vec<Ripple>,
    brakes: Vec<HoldingBrake>,
    mesh: ToothMesh,
    belt: Belt,
    stops: HardStop,
    /// The cable chain, the axes that drag one, and per axis the speed below which its drag fades out.
    chain: CableChain,
    chain_axes: Vec<bool>,
    chain_fade: Vec<f64>,
    /// Force on the tool along each axis while the cycle dwells; zero with effects.process_force off.
    process: Vec<f64>,
    /// An obstacle's push on the tool (`Machine::set_tool_force`), per axis, held until changed; it acts like the process
    /// force, whether or not the cycle dwells. Its power is not in the machine's rows: the simulation books it as the
    /// crash's.
    tool_push: Vec<f64>,
    dwelling: bool,
    /// The netlist has a frame, and with effects.frame_compliance in the elastic model it sways.
    frame_node: bool,
    frame: Option<FrameMode>,
    /// With effects.thermal_feedback: the law per motor (its gearbox's efficiency), and each guide's viscous
    /// coefficient at the reference temperature.
    thermal: Option<(Vec<ThermalFeedback>, Vec<f64>)>,
    /// With effects.belt_tooth_jump (elastic model only): per motor the law and the sections its pulley feeds (+1) and
    /// takes from (-1); the energy the last step's jumps took, J/s.
    /// Only the belt motors have one: `jump_motor` is the motor of each jump, `jump_of` the jump of each motor.
    jumps: Vec<(ToothJump, usize, usize)>,
    jump_motor: Vec<usize>,
    jump_of: Vec<Option<usize>>,
    jump_loss: f64,
    /// With effects.belt_span_vibration: the belt's mass per metre, and the spans.
    span_mass: Option<f64>,
    spans: Vec<Span>,
    /// The belt's mass per metre where the spans give their speeds over ground (`runs_mass`), else 0; the belt's mass
    /// lumped on each belt pulley (at its radius).
    run_mass: f64,
    lump: f64,
    payload_attached: bool,
    brake_engaged: bool,
    /// Per motor: the axis it drives without a belt, `None` for a belt motor.
    direct: Vec<Option<Direct>>,
    /// Per motor: its configured gearbox has ratio 1 (a coupling; a screw or rack behind it changes the ratio), and
    /// what a screw or rack puts on its efficiency (1 for none), which thermal feedback keeps.
    coupled: Vec<bool>,
    extra_efficiency: Vec<f64>,
    /// Per axis: the inertia a rotary turns with it and its payload's (0 on a straight axis), kg m^2.
    turned: Vec<(f64, f64)>,
    /// With effects.belt_wrap: per belt motor the wheels before and after its pulley along the belt (`belt_wraps`).
    wraps: Vec<(usize, Waypoint, Waypoint)>,
    /// Per motor and idler: the belt runs round it clockwise (`machine.pulley.clockwise`, `machine.idler.clockwise`).
    clockwise: (Vec<bool>, Vec<bool>),
    /// The netlist has a leaning body, and with effects.tool_flex in the elastic model it leans (`machine.lean`).
    lean_node: Option<LeanNode>,
    lean: Option<LeanNode>,
    /// With effects.beam_vertical_mode (elastic model, with a `machine.lean` node naming the axes): the beam's mode.
    beam: Option<BeamMode>,
    /// Per motor: its worm stage while effects.gearbox_efficiency counts it (`worm::factors`), at its temperature; and
    /// as configured, at the reference temperature.
    worms: Vec<Option<WormNode>>,
    worms_configured: Vec<Option<WormNode>>,
    /// Per coordinate (the axes, then a yawing gantry's yaw): the direction a straight axis moves the bodies in the 3D
    /// view (zero for the others), and the axis a rotary turns them about (zero for the others).
    dirs: Vec<[f64; 3]>,
    turn: Vec<[f64; 3]>,
    /// Per axis the stroke of its joint, where its hard stops are (with a tcp the tool's range is wider).
    stroke: Vec<f64>,
    gantry: Option<Gantry>,
    tcp: Option<TcpNode>,
    /// A dual-drive gantry's two motors, the first the master (`Machine::gantry_motors`), whether or not its bridge yaws.
    gantry_motors: Option<[usize; 2]>,
}

// ----------------------------------------------------------------------------------------------------------------
// Small dense linear algebra: the systems have as many unknowns as the machine has axes or belt sections.

/// Solves a x = b by Gaussian elimination with partial pivoting; `None` for a singular a.
fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    for col in 0..n {
        let pivot = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[pivot][col].abs() < 1e-300 {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..n {
            let factor = a[row][col] / a[col][col];
            if factor != 0.0 {
                for k in col..n {
                    a[row][k] -= factor * a[col][k];
                }
                b[row] -= factor * b[col];
            }
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let sum: f64 = (row + 1..n).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - sum) / a[row][row];
    }
    Some(x)
}

/// The least-squares solution of a x = b (a: rows x cols), the one of least norm where it is not unique.
fn lstsq(a: &[Vec<f64>], b: &[f64]) -> Option<Vec<f64>> {
    let (rows, cols) = (a.len(), a.first().map_or(0, Vec::len));
    if rows >= cols {
        let ata = (0..cols).map(|i| (0..cols).map(|j| (0..rows).map(|r| a[r][i] * a[r][j]).sum()).collect()).collect();
        let atb = (0..cols).map(|i| (0..rows).map(|r| a[r][i] * b[r]).sum()).collect();
        solve(ata, atb)
    } else {
        let aat = (0..rows).map(|i| (0..rows).map(|j| (0..cols).map(|c| a[i][c] * a[j][c]).sum()).collect()).collect();
        let y = solve(aat, b.to_vec())?;
        Some((0..cols).map(|c| (0..rows).map(|r| a[r][c] * y[r]).sum()).collect())
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// `v` turned by `angle` about the unit axis `u` (Rodrigues): v cos + (u x v) sin + u (u . v) (1 - cos).
fn rotate(u: [f64; 3], angle: f64, v: [f64; 3]) -> [f64; 3] {
    let (s, c) = angle.sin_cos();
    let cross = cross(u, v);
    let along = dot(&u, &v) * (1.0 - c);
    [0, 1, 2].map(|k| v[k] * c + cross[k] * s + u[k] * along)
}

fn cross(u: [f64; 3], v: [f64; 3]) -> [f64; 3] {
    [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
}

/// The eigenvalues and eigenvectors (columns) of a symmetric matrix, by cyclic Jacobi rotations.
fn symmetric_eigen(mut a: Vec<Vec<f64>>) -> (Vec<f64>, Vec<Vec<f64>>) {
    let d = a.len();
    let mut v: Vec<Vec<f64>> = (0..d).map(|i| (0..d).map(|j| if i == j { 1.0 } else { 0.0 }).collect()).collect();
    let scale: f64 = a.iter().flatten().map(|x| x * x).sum();
    for _ in 0..100 {
        let off: f64 = (0..d)
            .flat_map(|i| (0..d).filter(move |&j| j != i).map(move |j| (i, j)))
            .map(|(i, j)| a[i][j] * a[i][j])
            .sum();
        if off <= 1e-30 * scale {
            break;
        }
        for p in 0..d {
            for q in p + 1..d {
                if a[p][q] == 0.0 {
                    continue;
                }
                let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..d {
                    let (kp, kq) = (a[k][p], a[k][q]);
                    (a[k][p], a[k][q]) = (c * kp - s * kq, s * kp + c * kq);
                }
                for k in 0..d {
                    let (pk, qk) = (a[p][k], a[q][k]);
                    (a[p][k], a[q][k]) = (c * pk - s * qk, s * pk + c * qk);
                }
                for row in v.iter_mut() {
                    let (kp, kq) = (row[p], row[q]);
                    (row[p], row[q]) = (c * kp - s * kq, s * kp + c * kq);
                }
            }
        }
    }
    ((0..d).map(|i| a[i][i]).collect(), v)
}

/// The lowest mode of K x = w^2 M x (M symmetric positive definite, K symmetric) whose w^2 is above zero: (w^2, x).
/// Through the Cholesky factor M = L L^T: L^-1 K L^-T y = w^2 y, x = L^-T y.
fn lowest_mode(m: &[Vec<f64>], k: &[Vec<f64>]) -> Option<(f64, Vec<f64>)> {
    let d = m.len();
    let mut l = vec![vec![0.0; d]; d];
    for i in 0..d {
        for j in 0..=i {
            let sum = m[i][j] - (0..j).map(|p| l[i][p] * l[j][p]).sum::<f64>();
            if i == j {
                l[i][i] = if sum > 0.0 { sum.sqrt() } else { return None };
            } else {
                l[i][j] = sum / l[j][j];
            }
        }
    }
    let lower = |b: &[f64]| {
        let mut x = vec![0.0; d];
        for i in 0..d {
            x[i] = (b[i] - (0..i).map(|p| l[i][p] * x[p]).sum::<f64>()) / l[i][i];
        }
        x
    };
    // X = L^-1 K, column by column; A = L^-1 X^T.
    let x: Vec<Vec<f64>> = (0..d).map(|c| lower(&(0..d).map(|r| k[r][c]).collect::<Vec<_>>())).collect();
    let a: Vec<Vec<f64>> = (0..d).map(|c| lower(&x[c])).collect();
    let a: Vec<Vec<f64>> = (0..d).map(|i| (0..d).map(|j| 0.5 * (a[i][j] + a[j][i])).collect()).collect();
    let (values, vectors) = symmetric_eigen(a);
    let largest = values.iter().fold(0.0f64, |m, v| m.max(*v));
    let (index, w2) =
        values.iter().copied().enumerate().filter(|(_, v)| *v > 1e-9 * largest).min_by(|a, b| a.1.total_cmp(&b.1))?;
    // x = L^-T y.
    let mut out = vec![0.0; d];
    for i in (0..d).rev() {
        out[i] = (vectors[i][index] - (i + 1..d).map(|p| l[p][i] * out[p]).sum::<f64>()) / l[i][i];
    }
    Some((w2, out))
}

/// Generalised velocity of a rigid solve: accelerations, guide forces and states' rates, per motor the torque at the
/// gearbox output and the torque that goes into it before its loss.
#[derive(Clone)]
struct Rigid {
    acc: Vec<f64>,
    rates: Vec<f64>,
    out: Vec<f64>,
    into: Vec<f64>,
}

/// The closed belts among the sections (see `ConfigMachine::loops`): sections that share a pulley are one belt.
fn closed_belts(sections: &[Section], motors: usize) -> Vec<Vec<usize>> {
    let mut belt: Vec<usize> = (0..sections.len()).collect();
    let root = |belt: &[usize], mut s: usize| {
        while belt[s] != s {
            s = belt[s];
        }
        s
    };
    for k in 0..motors {
        let at: Vec<usize> = (0..sections.len()).filter(|&s| sections[s].f[k] != 0.0).collect();
        for pair in at.windows(2) {
            let (a, b) = (root(&belt, pair[0]), root(&belt, pair[1]));
            belt[a] = b;
        }
    }
    let mut out: Vec<Vec<usize>> = Vec::new();
    for s in 0..sections.len() {
        let r = root(&belt, s);
        match out.iter_mut().find(|l| root(&belt, l[0]) == r) {
            Some(l) => l.push(s),
            None => out.push(vec![s]),
        }
    }
    out.retain(|l| (0..motors).all(|k| l.iter().map(|&s| sections[s].f[k]).sum::<f64>() == 0.0));
    out
}

impl ConfigMachine {
    pub fn new(cfg: &Config, net: &Netlist) -> Result<Self, ConfigError> {
        let (n, m) = (net.axes.len(), net.motors.len());
        let elastic = cfg.bool("effects.belt_stretch")?;
        let gravity = if cfg.bool("effects.gravity")? { G } else { 0.0 };
        let belt = Belt::from_config(cfg)?;
        // Each motor's drive train from its own config; a worm stage's ratio is its gearbox's.
        let mut own: Vec<Config> = net.motors.iter().map(|d| motor_config(cfg, &d.id)).collect::<Result<_, _>>()?;
        for (own, d) in own.iter_mut().zip(&net.motors) {
            if let Some(worm) = d.worm {
                own.set("gearbox.ratio", toml::Value::Float(worm.ratio))?;
            }
        }
        let own = own;
        let mut gears = own.iter().map(Gearbox::from_config).collect::<Result<Vec<_>, _>>()?;
        let mut j_rotor = own
            .iter()
            .map(|c| Ok(c.positive("motor.rotor_inertia_kgm2")? + c.non_negative("gearbox.inertia_at_input_kgm2")?))
            .collect::<Result<Vec<f64>, ConfigError>>()?;
        let mut shafts = own.iter().map(ShaftFriction::from_config).collect::<Result<Vec<_>, _>>()?;
        let mut ripples = own.iter().map(Ripple::from_config).collect::<Result<Vec<_>, _>>()?;
        let mut brakes =
            own.iter().zip(&j_rotor).map(|(c, j)| HoldingBrake::from_config(c, *j)).collect::<Result<Vec<_>, _>>()?;
        let mut coupled: Vec<bool> = gears.iter().map(Gearbox::is_coupling).collect();
        let mut extra_efficiency = vec![1.0; m];
        // A screw, rack, linear motor or rotary: the drive train as the registered one-axis machines build it, the converter behind the
        // gearbox, which is then one in the axis's unit (see the module comment).
        let guide_friction = cfg.bool("effects.guide_friction")?;
        let mut direct = Vec::with_capacity(m);
        let mut bearing = vec![[0.0; 3]; n];
        for (kk, d) in net.motors.iter().enumerate() {
            let Some(axis) = d.axis else {
                direct.push(None);
                continue;
            };
            let own = &own[kk];
            let stroke = net.axes[axis].stroke;
            let free = ShaftFriction::NONE;
            let mut joint = RotaryJoint::with(own, free, HardStop { stiffness: 0.0, damping: 0.0 }, (0.0, stroke))?;
            let ratio = joint.ratio();
            // What the screw or rack moves at least: the bodies on its axis without their payload, shared with the
            // other drives of that axis (a gantry's). The mesh friction's bristle is capped by the lighter of it and the
            // rotor (`Converter::apply_to`); with no body on it, the rotor's alone.
            let on_axis: f64 = net.bodies.iter().filter(|b| b.axes.get(axis) == Some(&true)).map(|b| b.mass).sum();
            let output = match on_axis / net.motors.iter().filter(|o| o.axis == Some(axis)).count() as f64 {
                x if x > 0.0 => x,
                _ => f64::INFINITY,
            };
            let (kind, lever) = match &d.transmission {
                Transmission::Screw { values } => {
                    let screw = BallScrew::from_config(&values_config(own, cfg, "screw", values)?)
                        .map_err(|e| renamed(e, "screw", values))?;
                    if screw.length < stroke {
                        return Err(ConfigError::Invalid(format!(
                            "{values}.length_m (shorter than the stroke of axis {})",
                            net.axes[axis].id
                        )));
                    }
                    screw.converter(screw.softest_position()).apply_to(own, &mut joint, output)?;
                    // The nut's preload drags as friction at the screw, at the rotor through the gearbox.
                    if guide_friction {
                        joint.shaft.coulomb += screw.preload_torque() / ratio;
                    }
                    let lever = 1.0 / screw.ratio();
                    (DirectKind::Screw(screw), Some(lever))
                }
                Transmission::Rack { values } => {
                    let rack = RackPinion::from_config(&values_config(own, cfg, "rack", values)?)
                        .map_err(|e| renamed(e, "rack", values))?;
                    rack.converter().apply_to(own, &mut joint, output)?;
                    let lever = rack.radius();
                    (DirectKind::Rack(rack), Some(lever))
                }
                Transmission::LinearMotor { values } => {
                    let view = values_config(own, cfg, "linear_motor", values)?;
                    let motor = LinearMotor::from_config(&view).map_err(|e| renamed(e, "linear_motor", values))?;
                    if ratio != 1.0 {
                        let own_ratio = format!("gearbox.{}.ratio", d.id);
                        let at = if cfg.get(&own_ratio).is_ok() { own_ratio } else { "gearbox.ratio".into() };
                        return Err(ConfigError::Invalid(format!(
                            "{at} (motor {} is a linear motor: its gearbox is the forcer's mounting, ratio 1)",
                            d.id
                        )));
                    }
                    joint.ripple.cogging_order = motor.cogging_order();
                    // The attraction presses the guides: its friction, as the registered one-axis machines add it.
                    if guide_friction {
                        joint.shaft.coulomb += view
                            .non_negative("linear_motor.guide_friction_coefficient")
                            .map_err(|e| renamed(e, "linear_motor", values))?
                            * motor.attraction;
                    }
                    (DirectKind::Linear { attraction: motor.attraction, forcer: joint.rotor_inertia }, None)
                }
                Transmission::Rotary { strain_wave, bearing: [coulomb, viscous], .. } => {
                    // The table's bearing drags at the axis. The rigid model has it at the rotor, i times as fast, as
                    // c / i + v w / i^2; the elastic one at the output, behind the gearbox's spring and efficiency
                    // (the smoothing speed of the rotor's friction over i).
                    if guide_friction && elastic {
                        let smoothing = ShaftFriction::from_config(own)?.smoothing / ratio.abs();
                        for (b, x) in bearing[axis].iter_mut().zip([*coulomb, *viscous, smoothing]) {
                            *b += x;
                        }
                    } else if guide_friction {
                        joint.shaft.coulomb += coulomb / ratio;
                        joint.shaft.viscous += viscous / (ratio * ratio);
                    }
                    let read = |t: &String| {
                        StrainWave::from_config(&values_config(own, cfg, "strain_wave", t)?)
                            .map_err(|e| renamed(e, "strain_wave", t))
                    };
                    (DirectKind::Rotary(strain_wave.as_ref().map(read).transpose()?), None)
                }
                Transmission::Belt => return Err(ConfigError::Invalid(format!("machine.motor[{}]", d.id))),
            };
            let RotaryJoint {
                gearbox,
                rotor_inertia,
                shaft,
                ripple,
                brake,
                extra_efficiency: extra,
                coupled: coupling,
                ..
            } = joint;
            (gears[kk], j_rotor[kk], shafts[kk], ripples[kk], brakes[kk]) =
                (gearbox, rotor_inertia, shaft, ripple, brake);
            (coupled[kk], extra_efficiency[kk]) = (coupling, extra);
            let side = net.gantry.as_ref().and_then(|g| g.drives[kk]).unwrap_or(0.0);
            direct.push(Some(Direct {
                axis,
                side,
                kind,
                lever,
                required_life: own.f64("ratings.required_life_h").ok(),
                radial_limit: own.f64("ratings.shaft_max_radial_load_n").ok(),
            }));
        }
        let worms = worm_stages(cfg, net, &mut gears, &extra_efficiency)?;
        // With a tcp the tool's range is where its tip can be whatever the head's angles: the strokes shifted by the tool
        // turned by the head, per direction by the most and the least of it over the rotaries' strokes (sampled, 13
        // angles each). Beyond that a pose may still be reachable at some angles (`unreachable` judges it).
        let span = net.tcp.as_ref().map(|t| {
            let mut span = [[f64::INFINITY, f64::NEG_INFINITY]; 3];
            let samples = 13usize;
            for k in 0..samples.pow(t.rotaries.len() as u32) {
                let mut v = t.tool;
                for (j, &r) in t.rotaries.iter().enumerate().rev() {
                    let (a, step) = (&net.axes[r], (k / samples.pow(j as u32)) % samples);
                    let turn = [a.incline_deg.to_radians().sin(), 0.0, a.incline_deg.to_radians().cos()];
                    v = rotate(turn, a.stroke * step as f64 / (samples - 1) as f64, v);
                }
                for (s, x) in span.iter_mut().zip(v) {
                    *s = [s[0].min(x), s[1].max(x)];
                }
            }
            span
        });
        let axes = net
            .axes
            .iter()
            .enumerate()
            .map(|(i, a)| -> Result<AxisInfo, ConfigError> {
                // A rotary axis's limits are in rad: max_velocity_rad_s.
                let unit = if a.rotary { "rad" } else { "m" };
                let limit = |what: &str, rate: &str| cfg.positive(&format!("axis.{}.max_{what}_{unit}_{rate}", a.id));
                let (min, max) = match net.tcp.as_ref().and_then(|t| Some((t, t.axes.iter().position(|&x| x == i)?))) {
                    None => (0.0, a.stroke),
                    Some((t, k)) if t.table.is_empty() => {
                        let [low, high] = span.map_or([0.0; 2], |s| s[k]);
                        (high, a.stroke + low)
                    }
                    // On a turning table: the workpiece within the round the table can turn under the tool, the box
                    // about its pivot as wide as the pivot's least distance from an end of X's and Y's strokes, which the
                    // head's tool offset shifts as in the case above: the tip always reaches [high, stroke + low] of the span.
                    Some((t, k)) => {
                        let reach = |j: usize| {
                            let [low, high] = span.map_or([0.0; 2], |s| s[j]);
                            (high, net.axes[t.axes[j]].stroke + low)
                        };
                        // A direction every table rotary turns about (Z under a C table alone) is not turned: its own reach.
                        let about = |r: usize| net.axes[r].incline_deg.to_radians().sin_cos();
                        let unturned = t.table.iter().all(|&r| {
                            let (s, c) = about(r);
                            [s, 0.0, c][k].abs() > 1.0 - 1e-12
                        });
                        let half = (0..2)
                            .map(|j| (t.pivot[j] - reach(j).0).min(reach(j).1 - t.pivot[j]))
                            .fold(f64::INFINITY, f64::min)
                            .max(0.0);
                        if unturned { reach(k) } else { (t.pivot[k] - half, t.pivot[k] + half) }
                    }
                };
                Ok(AxisInfo {
                    id: intern(&a.id),
                    key: intern(&a.key),
                    unit,
                    min,
                    max,
                    max_vel: limit("velocity", "s")?,
                    max_acc: limit("acceleration", "s2")?,
                    max_jerk: limit("jerk", "s3")?,
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        let incline: Vec<f64> = net.axes.iter().map(|a| a.incline_deg.to_radians()).collect();
        let teeth = belt.tooth_length;
        let mut sections: Vec<Section> = net
            .sections
            .iter()
            .map(|s| {
                let length = s.length + teeth * s.pulleys.iter().filter(|f| **f != 0.0).count() as f64;
                Section {
                    id: intern(&format!("belt.{}", s.id)),
                    key: intern(&s.key),
                    length,
                    bare: s.length,
                    c: s.axes.clone(),
                    f: s.pulleys.clone(),
                    mat0: belt.rest_length(length),
                    route: s.route.clone(),
                }
            })
            .collect();
        let r: Vec<f64> = net.motors.iter().map(|d| d.radius).collect();
        // The rigid kinematics: F diag(r) K = C, over the motors that drive belts (the columns of F).
        let belted: Vec<usize> = (0..m).filter(|&kk| direct[kk].is_none()).collect();
        let fr: Vec<Vec<f64>> = sections.iter().map(|s| belted.iter().map(|&k| s.f[k] * r[k]).collect()).collect();
        // A gantry's yaw is a coordinate after the axes (effects.gantry_yaw); the sections that pull the bridge's ends get
        // longer with it.
        let yaw_on = match cfg.bool("effects.gantry_yaw") {
            Err(ConfigError::Missing(_)) => false,
            other => other?,
        };
        let gantry_node = net.gantry.as_ref().filter(|_| yaw_on);
        let mut columns: Vec<Vec<f64>> = (0..n).map(|i| sections.iter().map(|s| s.c[i]).collect()).collect();
        columns.extend(gantry_node.map(|g| g.sections.clone()));
        let mut yaw = gantry_node.is_some();
        let mut k = vec![vec![0.0; columns.len()]; m];
        // Without belt sections (every motor drives its axis without a belt) there is nothing to solve.
        for (i, column) in columns.iter().enumerate().filter(|_| !sections.is_empty()) {
            let solved = lstsq(&fr, column)
                .ok_or_else(|| ConfigError::Invalid("machine.section (the pulleys do not set every section)".into()))?;
            let wrong = (0..sections.len()).find(|&s| {
                let follows: f64 = fr[s].iter().zip(&solved).map(|(a, b)| a * b).sum();
                (follows - column[s]).abs() > 1e-9 * (1.0 + column[s].abs())
            });
            match wrong {
                Some(s) if i < n => {
                    return Err(ConfigError::Invalid(format!(
                        "machine.section[{}].axes (the belt cannot keep its length)",
                        &sections[s].id["belt.".len()..]
                    )));
                }
                // The belts hold the yaw where they cannot keep their length with it (a CoreXY's): the rigid model has
                // none, the elastic one takes their stretch.
                Some(_) if !elastic => yaw = false,
                _ => {}
            }
            for (j, &kk) in belted.iter().enumerate() {
                k[kk][i] = solved[j];
            }
        }
        if !yaw {
            k.iter_mut().for_each(|row| row.truncate(n));
        }
        let d = n + usize::from(yaw);
        // The yaw coordinate has no bearing of its own; the per-coordinate tables are indexed up to d.
        bearing.resize(d, [0.0; 3]);
        if let Some(g) = gantry_node.filter(|_| yaw) {
            sections.iter_mut().zip(&g.sections).for_each(|(s, c)| s.c.push(*c));
        }
        // A motor that drives an axis without a belt turns its "pulley" (the gearbox output, in the axis's unit) with it,
        // and at a yawing gantry's side with the yaw too.
        for (row, x) in k.iter_mut().zip(&direct) {
            if let Some(x) = x {
                *row = (0..d)
                    .map(|i| {
                        if i == x.axis {
                            1.0
                        } else if i == n {
                            x.side
                        } else {
                            0.0
                        }
                    })
                    .collect();
            }
        }
        // The coordinates the motors set: all of them, or the axes where a gantry's yaw is left to its spring. Two
        // drives on a gantry whose yaw is off share its axis: the least-squares inverse halves its force between them.
        let set: Vec<usize> = if m >= d { (0..d).collect() } else { (0..n).collect() };
        if set.len() > m {
            return Err(ConfigError::Invalid("machine.motor (the motors do not set every axis)".into()));
        }
        let k_set: Vec<Vec<f64>> = k.iter().map(|row| set.iter().map(|&i| row[i]).collect()).collect();
        let mut k_inv = vec![vec![0.0; m]; d];
        for kk in 0..m {
            let unit: Vec<f64> = (0..m).map(|j| if j == kk { 1.0 } else { 0.0 }).collect();
            let column = if m == set.len() { solve(k_set.clone(), unit) } else { lstsq(&k_set, &unit) }
                .ok_or_else(|| ConfigError::Invalid("machine.motor (the motors do not set every axis)".into()))?;
            for (j, &i) in set.iter().enumerate() {
                k_inv[i][kk] = column[j];
            }
        }
        // The belt runs at pulley surface speed: its mass is shared out on the driven pulleys.
        let belt_motors = direct.iter().filter(|d| d.is_none()).count().max(1);
        let belt_mass = belt.mass_per_m * sections.iter().map(|s| s.bare).sum::<f64>() / belt_motors as f64;
        let payload_on = cfg.bool("effects.payload")?;
        let mut turned = vec![(0.0, 0.0); d];
        // The bridge's own yaw inertia is the yaw's "mass".
        if let Some(g) = gantry_node.filter(|_| yaw) {
            turned[n] = (g.inertia, 0.0);
        }
        for d in &net.motors {
            if let (Transmission::Rotary { inertia, payload, .. }, Some(axis)) = (&d.transmission, d.axis) {
                turned[axis] = (*inertia, if payload_on { *payload } else { 0.0 });
            }
        }
        let guides = net
            .guides
            .iter()
            .map(|g| -> Result<Guide, ConfigError> {
                Ok(Guide {
                    loss_id: intern(&g.id),
                    contact: intern(&format!("axis.{}", net.axes[g.axis].id)),
                    axis: g.axis,
                    friction: Lugre::from_config(cfg, &g.friction)?,
                    carries: g.carries.clone(),
                    overhang: g.overhang.as_ref().map(|o| crate::netlist::Overhang { ..*o }),
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        // A file written before machine files had these effects may leave their switches out: off.
        let on = |effect: &str| match cfg.bool(&format!("effects.{effect}")) {
            Err(ConfigError::Missing(_)) => Ok(false),
            other => other,
        };
        let idlers_on = on("idlers")?;
        // A vector per axis with zeros for the yaw.
        let pad = |v: &[f64]| -> Vec<f64> { (0..d).map(|i| v.get(i).copied().unwrap_or(0.0)).collect() };
        let smoothing = ShaftFriction::from_config(cfg)?.smoothing;
        let idlers = net
            .idlers
            .iter()
            .map(|d| Idler {
                mass: if idlers_on { d.inertia / (d.radius * d.radius) } else { 0.0 },
                drag: if idlers_on { d.friction / d.radius } else { 0.0 },
                smoothing: smoothing * d.radius,
                radius: d.radius,
                speed: pad(&d.speed),
                body: d.body,
                at: d.at,
            })
            .collect();
        let mut chain_axes: Vec<bool> = net.axes.iter().map(|a| a.cable_chain).collect();
        chain_axes.resize(d, false);
        let chain = if chain_axes.contains(&true) {
            CableChain::from_config(cfg)?
        } else {
            CableChain { mass_per_m: 0.0, length0: 0.0, drag: 0.0 }
        };
        // shortcut: an axis without a guide fades its chain's drag out below 1 cm/s; give it a guide to set it.
        let chain_fade =
            (0..d).map(|i| guides.iter().find(|g| g.axis == i).map_or(0.01, |g| g.friction.stribeck_v)).collect();
        let process = if on("process_force")? {
            pad(&net
                .axes
                .iter()
                .map(|a| cfg.f64(&format!("process.force_{}_n", a.id)))
                .collect::<Result<Vec<_>, _>>()?)
        } else {
            vec![0.0; d]
        };
        let frame = match (&net.frame, on("frame_compliance")? && elastic) {
            (Some(f), true) => {
                Some(FrameMode { axis: f.axis, mass: f.mass, stiffness: f.stiffness, damping: f.damping })
            }
            (None, true) => return Err(ConfigError::Missing("machine.frame (effects.frame_compliance)".into())),
            (_, false) => None,
        };
        let thermal = own
            .iter()
            .map(ThermalFeedback::from_config)
            .collect::<Result<Option<Vec<_>>, _>>()?
            .map(|laws| (laws, guides.iter().map(|g| g.friction.viscous).collect()));
        // The jump states are laid out for the belt motors only (a screw, rack, linear motor or rotary has no tooth to jump).
        let mut jumps = Vec::new();
        let mut jump_motor = Vec::new();
        let mut jump_of = vec![None; m];
        if elastic && on("belt_tooth_jump")? {
            for (kk, d) in net.motors.iter().enumerate().filter(|(kk, _)| direct[*kk].is_none()) {
                let side = |f: f64| -> Result<usize, ConfigError> {
                    let found: Vec<usize> = (0..sections.len()).filter(|&s| sections[s].f[kk] == f).collect();
                    match found[..] {
                        [s] => Ok(s),
                        _ => Err(ConfigError::Invalid(format!(
                            "effects.belt_tooth_jump (pulley {} has no section on each side)",
                            d.pulley
                        ))),
                    }
                };
                let law = ToothJump::from_config(&own[kk], d.radius, belt_mass)?
                    .ok_or_else(|| ConfigError::Invalid("effects.belt_tooth_jump".into()))?;
                jump_of[kk] = Some(jumps.len());
                jump_motor.push(kk);
                jumps.push((law, side(1.0)?, side(-1.0)?));
            }
        }
        let span_mass = if on("belt_span_vibration")? {
            if net.spans.is_empty() {
                return Err(ConfigError::Missing("machine.span (effects.belt_span_vibration)".into()));
            }
            Some(cfg.positive("belt.mass_kg_m")?)
        } else {
            None
        };
        let spans: Vec<Span> = net
            .spans
            .iter()
            .map(|s| Span {
                key: intern(&format!("belt-span-{}", s.id)),
                section: s.section,
                length: s.length,
                c: pad(&s.axes),
                velocity: s.velocity.as_ref().map(|v| [pad(&v[0]), pad(&v[1]), pad(&v[2])]),
            })
            .collect();
        // The belt run by run (`runs_mass`): every span with its speed over ground, and the spans the whole belt.
        let runs = spans.iter().any(|s| s.velocity.is_some());
        if runs {
            let total =
                |length: f64, c: &[f64]| -> Vec<f64> { std::iter::once(length).chain(c.iter().copied()).collect() };
            let sum = |rows: Vec<Vec<f64>>| -> Vec<f64> {
                rows.iter().fold(vec![0.0; d + 1], |a, r| a.iter().zip(r).map(|(a, b)| a + b).collect())
            };
            let of_spans = sum(spans.iter().map(|s| total(s.length, &s.c)).collect());
            let of_sections = sum(sections.iter().map(|s| total(s.bare, &s.c)).collect());
            let whole = of_spans.iter().zip(&of_sections).all(|(a, b)| (a - b).abs() <= 1e-9 * of_sections[0]);
            if spans.iter().any(|s| s.velocity.is_none()) || !whole {
                return Err(ConfigError::Invalid(
                    "machine.span (with velocity_x, _y or _z every span needs one, and the spans together are the whole belt)"
                        .into(),
                ));
            }
        }
        let wraps = if on("belt_wrap")? { wrap_neighbours(net, &direct)? } else { Vec::new() };
        let lean = match (net.lean, on("tool_flex")?) {
            (None, true) => return Err(ConfigError::Missing("machine.lean (effects.tool_flex)".into())),
            // As in the machine crate with a leaning axis: the lean is a mode of the elastic model.
            (lean, flex) => lean.filter(|_| flex && elastic),
        };
        // A belt that moves the lean's lever axis holds the lean back by its vertical runs (`lean_runs`), which the spans
        // give: without one the geometric stiffness would silently be missing.
        if let Some(z) = lean.as_ref().and_then(|l| l.lever_axis) {
            let belted = sections.iter().any(|s| s.c.get(z).is_some_and(|c| *c != 0.0));
            let vertical = |s: &SpanNode| {
                s.axes.get(z).is_some_and(|c| c.abs() == 1.0)
                    && s.axes.iter().enumerate().all(|(i, c)| i == z || *c == 0.0)
            };
            if belted && !net.spans.iter().any(vertical) {
                return Err(ConfigError::Missing(format!(
                    "machine.span (the belt's runs along {} that hold the leaning body back, effects.tool_flex)",
                    net.axes[z].id
                )));
            }
        }
        let beam = if elastic && on("beam_vertical_mode")? {
            let lean =
                net.lean.ok_or_else(|| ConfigError::Missing("machine.lean (effects.beam_vertical_mode)".into()))?;
            let vertical = lean
                .lever_axis
                .ok_or_else(|| ConfigError::Missing("machine.lean.lever_axis (effects.beam_vertical_mode)".into()))?;
            Some(BeamMode {
                axis: lean.axis,
                vertical,
                span: cfg.positive("beam.span_m")?,
                start: cfg.non_negative("beam.carriage_at_zero_m")?,
                ei: cfg.positive("beam.bending_stiffness_nm2")?,
                mass_per_m: cfg.positive("beam.mass_kg_m")?,
                damping_ratio: cfg.non_negative("beam.damping_ratio")?,
            })
        } else {
            None
        };
        let gantry = gantry_node.filter(|_| yaw).map(|g| Gantry {
            id: intern(&g.id),
            coord: n,
            axis: g.axis,
            across: g.across,
            shift: g.across_shift,
            width: g.width,
            stiffness: g.stiffness,
            damping: g.damping,
            tolerance: g.tolerance,
        });
        // Rotary axes have no weight along or across them: what rides on them weighs through its place (`body_point`).
        let straight = |i: usize, v: f64| if i < n && !net.axes[i].rotary { v } else { 0.0 };
        let dirs = (0..d)
            .map(|i| {
                let (up, across) = incline.get(i).map_or((0.0, 0.0), |a| a.sin_cos());
                let (up, across) = (straight(i, up), straight(i, across));
                if i == 0 { [across, 0.0, up] } else { [0.0, across, up] }
            })
            .collect();
        let turn = (0..d)
            .map(|i| match net.axes.get(i) {
                Some(a) if a.rotary => [incline[i].sin(), 0.0, incline[i].cos()],
                _ => [0.0; 3],
            })
            .collect();
        let bodies = net
            .bodies
            .iter()
            .map(|b| {
                let mut axes = b.axes.clone();
                axes.resize(d, false);
                let across = gantry.as_ref().is_some_and(|g| axes[g.axis] && g.across.is_some_and(|a| axes[a]));
                Body {
                    mass: b.mass,
                    payload: if payload_on { b.payload } else { 0.0 },
                    general: across || !b.rotaries.is_empty(),
                    axes,
                    rotaries: b.rotaries.clone(),
                    offset: b.offset,
                }
            })
            .collect();
        Ok(Self {
            elastic,
            g: gravity,
            g_along: (0..d).map(|i| straight(i, gravity * incline[i.min(n - 1)].sin())).collect(),
            // cos(90 deg) is 6e-17, not 0: a vertical axis has no weight across it.
            g_across: (0..d)
                .map(|i| {
                    let a = incline[i.min(n - 1)];
                    straight(i, if (a.cos()).abs() < 1e-12 { 0.0 } else { gravity * a.cos() })
                })
                .collect(),
            loops: closed_belts(&sections, m),
            bodies,
            dirs,
            turn,
            stroke: net.axes.iter().map(|a| a.stroke).collect(),
            gantry,
            tcp: net.tcp.clone(),
            gantry_motors: net.gantry.as_ref().and_then(|g| {
                let mut drives = g.drives.iter().enumerate().filter(|(_, d)| d.is_some()).map(|(k, _)| k);
                Some([drives.next()?, drives.next()?])
            }),
            motors: net.motors.iter().map(|d| MotorInfo { id: intern(&d.id), key: intern(&d.key) }).collect(),
            r_mat: r.iter().map(|r| belt.rest_length(*r)).collect(),
            j_pulley: net
                .motors
                .iter()
                .zip(&direct)
                .map(|(d, x)| if x.is_some() { 0.0 } else { d.inertia + belt_mass * d.radius * d.radius })
                .collect(),
            pulley_at: net.motors.iter().map(|d| d.at).collect(),
            r,
            k,
            k_inv,
            ratio: gears.iter().map(|g| g.ratio).collect(),
            shafts,
            ripples,
            brakes,
            j_rotor,
            gears,
            mesh: ToothMesh::from_config(cfg)?,
            stops: HardStop::from_config(cfg)?,
            chain,
            chain_axes,
            chain_fade,
            process,
            tool_push: Vec::new(),
            dwelling: false,
            frame_node: net.frame.is_some(),
            frame,
            thermal,
            jumps,
            jump_motor,
            jump_of,
            jump_loss: 0.0,
            span_mass,
            spans,
            run_mass: if runs { belt.mass_per_m } else { 0.0 },
            lump: belt_mass,
            belt,
            sections,
            guides,
            idlers,
            bearing,
            axes,
            payload_attached: true,
            brake_engaged: false,
            direct,
            coupled,
            extra_efficiency,
            turned,
            wraps,
            clockwise: (
                net.motors.iter().map(|d| d.clockwise).collect(),
                net.idlers.iter().map(|d| d.clockwise).collect(),
            ),
            lean_node: net.lean,
            lean,
            beam,
            worms_configured: worms.clone(),
            worms,
        })
    }

    /// The angle and speed of motor kk's gearbox output in the elastic model: its pulley's, or the axis's that its
    /// screw, rack, linear motor or rotary drives.
    fn output(&self, s: &[f64], kk: usize) -> (f64, f64) {
        let (n, v) = (self.n(), self.vel());
        match &self.direct[kk] {
            Some(_) => (dot(&self.k[kk], &s[..n]), dot(&self.k[kk], &s[v..v + n])),
            None => (s[n + kk], s[v + n + kk]),
        }
    }
    /// The belt's tooth mesh force at motor kk's pulley; none where it drives no pulley.
    fn mesh_force(&self, kk: usize, x: f64) -> f64 {
        if self.direct[kk].is_some() { 0.0 } else { self.mesh.force(x) }
    }
    /// A screw, rack, linear motor or rotary drives coordinate i (a gantry's yaw where one stands at its side).
    fn driven(&self, i: usize) -> bool {
        self.direct.iter().zip(&self.k).any(|(d, k)| d.is_some() && k[i] != 0.0)
    }
    /// What presses on axis i's guide beside the weights: a linear motor's attraction and its forcer's weight.
    fn pressed(&self, i: usize) -> f64 {
        let on = |d: &&Direct| d.axis == i;
        self.direct
            .iter()
            .flatten()
            .filter(on)
            .map(|d| {
                if let DirectKind::Linear { attraction, forcer } = d.kind {
                    attraction + forcer * self.g_across[i]
                } else {
                    0.0
                }
            })
            .sum()
    }

    /// The coordinates: the axes, then a yawing gantry's yaw.
    fn n(&self) -> usize {
        self.dirs.len()
    }
    /// The axes, the tool's coordinates.
    fn nt(&self) -> usize {
        self.axes.len()
    }
    fn m(&self) -> usize {
        self.motors.len()
    }
    // Elastic state: [q, a (pulleys), p (rotors), u (the frame, with one), w (the leaning body, with effects.tool_flex)],
    // their rates, a friction state per axis, the slack time, a brake state per motor, and with effects.belt_tooth_jump
    // `JUMP_STATES` per belt motor.
    // Rigid state: [q, v, friction states, brakes].
    fn vel(&self) -> usize {
        if self.elastic { self.beam_at() + usize::from(self.beam.is_some()) } else { self.n() }
    }
    fn frame_at(&self) -> usize {
        self.n() + 2 * self.m()
    }
    fn lean_at(&self) -> usize {
        self.frame_at() + usize::from(self.frame.is_some())
    }
    fn beam_at(&self) -> usize {
        self.lean_at() + usize::from(self.lean.is_some())
    }
    /// The rows of the lean and the beam's deflection in the elastic mass matrix: after the axes and the frame's sway.
    fn lean_row(&self) -> usize {
        self.n() + usize::from(self.frame.is_some())
    }
    fn beam_row(&self) -> usize {
        self.lean_row() + usize::from(self.lean.is_some())
    }
    /// The beam's deflection under the carriage and its rate; zero without the beam mode.
    fn deflection(&self, s: &[f64]) -> (f64, f64) {
        if self.beam.is_some() { (s[self.beam_at()], s[self.vel() + self.beam_at()]) } else { (0.0, 0.0) }
    }
    /// The beam mode at the carriage's place: (k, M, c, dk/dx, dM/dx), zeros without it.
    fn beam_mode(&self, q: &[f64]) -> [f64; 5] {
        self.beam.as_ref().map_or([0.0; 5], |b| b.at_x(q[b.axis]))
    }
    /// The mass of the bodies that move along the beam's axis (the carriage and what rides on it, with the payload while
    /// attached), and of those of them that also move along the vertical axis: what hangs on w.
    fn beam_load(&self) -> (f64, f64) {
        let Some(b) = &self.beam else { return (0.0, 0.0) };
        let on = self.bodies.iter().filter(|body| body.axes[b.axis]);
        let riding = on.clone().filter(|body| body.axes[b.vertical]);
        (on.map(|body| self.body_mass(body)).sum(), riding.map(|body| self.body_mass(body)).sum())
    }
    /// With the beam mode, shifts `q` to where the Z axis stands on its belt above the beam's quasi-static sag
    /// w0 = (carried weight) / k (its own coordinate is relative to the carriage) and returns w0; 0 and no change without it.
    fn sag_at_rest(&self, q: &mut [f64]) -> f64 {
        let Some(b) = &self.beam else { return 0.0 };
        let w0 = self.beam_load().0 * self.g / b.stiffness(q[b.axis]);
        q[b.vertical] += w0;
        w0
    }
    /// The leaning body's sideways travel against its carriage and its rate; zero without it.
    fn leaning(&self, s: &[f64]) -> (f64, f64) {
        if self.lean.is_some() { (s[self.lean_at()], s[self.vel() + self.lean_at()]) } else { (0.0, 0.0) }
    }
    /// The tool tip's travel along the leaning body's axis per metre it leans (`geometry::flex_at`); 0 without it.
    fn tip(&self, q: &[f64]) -> f64 {
        self.lean.as_ref().map_or(0.0, |lean| flex_at(lean, q).2)
    }
    /// The belt runs' geometric stiffness, per section: a belt run of length l under tension T whose far end is out
    /// sideways by delta is longer by delta^2 / (2 l) (the chord of a right triangle), so it holds the end back with
    /// T delta / l. The runs are the spans that change length with the lean's lever axis alone, one metre per metre (the
    /// belt's vertical runs: the clamp's, whose far end is out with the leaning body at `-length` below the carriage's
    /// idlers, and the idler at the body's bottom, `length` below them). With g the end's travel per metre of lean,
    /// k reach (`lean_reach` at the end's distance d and the body's centre of mass h = com_drop - z below the carriage,
    /// k the spring's stiffness), each section is longer by 1/2 c u^2 with c = sum over its runs of g^2 / l; its tension,
    /// its force on the lean (sum T c u), on the lever axis (-1/2 u^2 sum T c') and its damping follow from the lengths. Zero
    /// without the lean or a span to give the runs (the crate has them as s + z, 2 times s + sz - z, s + z).
    fn lean_runs(&self, q: &[f64]) -> Vec<f64> {
        let mut c = vec![0.0; self.sections.len()];
        let Some((lean, z)) = self.lean.as_ref().and_then(|l| Some((l, l.lever_axis?))) else { return c };
        let (k, h) = (flex_at(lean, q).0, lean.com_drop - q[z]);
        for span in &self.spans {
            let up = span.c[z];
            if up.abs() != 1.0 || span.c.iter().enumerate().any(|(i, c)| i != z && *c != 0.0) {
                continue;
            }
            let length = span.length + up * q[z];
            let end = k * lean_reach(lean, -up * length, h);
            c[span.section] += end * end / length;
        }
        c
    }
    /// How much longer `lean_runs` make the sections at the state's lean u and height z, and the rate of that:
    /// 1/2 c u^2 and c u u' + 1/2 c' z' u^2 (c' by a central difference over 2e-6 m, as the crate takes it).
    fn lean_lengths(&self, s: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let zero = vec![0.0; self.sections.len()];
        let Some(z) = self.lean.as_ref().and_then(|l| l.lever_axis) else { return (zero.clone(), zero) };
        let (n, v) = (self.n(), self.vel());
        let ((u, du), q) = (self.leaning(s), &s[..n]);
        let (c, mut ahead, mut behind) = (self.lean_runs(q), q.to_vec(), q.to_vec());
        (ahead[z], behind[z]) = (q[z] + 1e-6, q[z] - 1e-6);
        let (ahead, behind) = (self.lean_runs(&ahead), self.lean_runs(&behind));
        let longer = c.iter().map(|c| 0.5 * c * u * u).collect();
        let rate =
            (0..c.len()).map(|i| c[i] * u * du + 0.5 * (ahead[i] - behind[i]) / 2e-6 * s[v + z] * u * u).collect();
        (longer, rate)
    }

    /// Motor kk's gearbox (`Gearbox::torque`), with its worm stage's share where it has one that turns back
    /// (`worm::factors`, at the power its spring's torque puts through the output). A self-locking worm passes the
    /// gearbox's torques on as they are; its mesh's friction is `worm_drag`, at the rotor.
    fn gear_torque(&self, kk: usize, phi: f64, w_phi: f64, theta: f64, w_theta: f64) -> GearTorque {
        let gear = self.gears[kk].torque(phi, w_phi, theta, w_theta);
        let Some(worm) = self.worms[kk].filter(|w| w.back_efficiency > 0.0) else { return gear };
        let (output, rotor) = crate::worm::factors(&worm, gear.rotor * self.ratio[kk] * w_theta);
        GearTorque { output: gear.output * output, rotor: gear.rotor * rotor }
    }
    /// The friction torque a self-locking worm's mesh puts on motor kk's rotor in the elastic model, against its speed
    /// `w_phi`, N m (0 without one): what would take the rotor's speed out over `worm::STICK_TIME_S` given its other
    /// torques `drive` (motor less shaft and brake), held to Coulomb's level of the flank the gear's torque at the rotor
    /// `rotor` loads (`worm::levels`) and never along the motion, so that the mesh only ever takes power
    /// (`F w_phi >= 0`). At speed that is the level (sliding); where the level would stop the rotor within the time,
    /// it stops it over that time (sticking), and at rest it holds anything within the levels. No speed band: the
    /// rotor's own acceleration on the level (16000 rad/s2 for the table's jam) crosses any band of a step's width.
    fn worm_drag(&self, kk: usize, rotor: f64, w_phi: f64, drive: f64) -> f64 {
        let Some(worm) = self.worms[kk].filter(|w| w.back_efficiency <= 0.0) else { return 0.0 };
        let (up, down) = crate::worm::levels(&worm, rotor);
        let stuck = drive - rotor + self.j_rotor[kk] * w_phi / crate::worm::STICK_TIME_S;
        let (low, high) = match w_phi.partial_cmp(&0.0) {
            Some(std::cmp::Ordering::Greater) => (0.0, up),
            Some(std::cmp::Ordering::Less) => (-down, 0.0),
            _ => (-down, up),
        };
        stuck.clamp(low, high)
    }
    /// The share of the load motor kk holds at rest: none behind a self-locking worm, all of it otherwise.
    fn held(&self, kk: usize) -> f64 {
        if self.worms[kk].is_some_and(|w| w.back_efficiency <= 0.0) { 0.0 } else { 1.0 }
    }
    /// What motor kk's gearbox loses, W (`Gearbox::loss_power`), with its worm stage's share, `drag` the self-locking
    /// worm's (`worm_drag`).
    fn gear_loss_power(&self, kk: usize, phi: f64, w_phi: f64, theta: f64, w_theta: f64, drag: f64) -> f64 {
        let (plain, worm) =
            (self.gears[kk].torque(phi, w_phi, theta, w_theta), self.gear_torque(kk, phi, w_phi, theta, w_theta));
        self.gears[kk].loss_power(phi, w_phi, theta, w_theta) + (worm.rotor - plain.rotor + drag) * w_phi
            - (worm.output - plain.output) * w_theta
    }
    /// The factors on motor kk's torque at its gearbox's output and at its rotor, in the rigid model, at the torque `into`
    /// that goes in (on the output's scale) and the output's speed: (`Gearbox::loss_factor`, 1), times the worm's share.
    fn rigid_factors(&self, kk: usize, into: f64, speed: f64) -> (f64, f64) {
        let loss = self.gears[kk].loss_factor(into, speed);
        match &self.worms[kk] {
            Some(worm) => {
                let (output, rotor) = crate::worm::factors(worm, into * speed);
                (loss * output, rotor)
            }
            None => (loss, 1.0),
        }
    }
    fn friction_at(&self) -> usize {
        2 * self.vel()
    }
    fn slack_at(&self) -> usize {
        self.friction_at() + self.n()
    }
    fn brake_at(&self) -> usize {
        self.slack_at() + usize::from(self.elastic)
    }
    fn jump_at(&self, j: usize) -> usize {
        self.brake_at() + self.m() + JUMP_STATES * j
    }

    fn body_mass(&self, b: &Body) -> f64 {
        b.mass + if self.payload_attached { b.payload } else { 0.0 }
    }
    /// The bodies' mass on each axis; on a rotary axis its rotary's inertia (with the payload's while attached), on a
    /// gantry's yaw the bridge's own. Bodies whose place is not a sum of the axes' travels are not in it (`general_mass`).
    fn mass(&self) -> Vec<f64> {
        let turned = |i: usize| self.turned[i].0 + if self.payload_attached { self.turned[i].1 } else { 0.0 };
        (0..self.n())
            .map(|i| {
                self.bodies.iter().filter(|b| b.axes[i] && !b.general).map(|b| self.body_mass(b)).sum::<f64>()
                    + turned(i)
            })
            .collect()
    }

    /// Where a body's centre of mass is at q, from where it is with every coordinate at 0: the straight axes' travels
    /// along their directions, on a yawing gantry's bridge its lever times the yaw along the bridge's axis, and its
    /// offset turned by the rotaries it rides on, the innermost first.
    fn body_point(&self, b: &Body, q: &[f64]) -> [f64; 3] {
        let mut p = [0.0; 3];
        let mut add = |dir: [f64; 3], t: f64| (0..3).for_each(|k| p[k] += dir[k] * t);
        for i in (0..self.n()).filter(|&i| b.axes[i]) {
            add(self.dirs[i], q[i]);
        }
        if let Some(g) = self.gantry.as_ref().filter(|g| b.axes[g.axis]) {
            let lever = g.across.filter(|&a| b.axes[a]).map_or(0.0, |a| q[a] + g.shift);
            add(self.dirs[g.axis], lever * q[g.coord]);
        }
        add(self.turned_by(&b.rotaries, q, b.offset), 1.0);
        p
    }
    /// `v` turned by the rotaries (outermost first) at their angles in q.
    fn turned_by(&self, rotaries: &[usize], q: &[f64], mut v: [f64; 3]) -> [f64; 3] {
        for &r in rotaries.iter().rev() {
            v = rotate(self.turn[r], q[r], v);
        }
        v
    }
    /// d body_point / d q_i, per coordinate, exactly: an axis's direction, the yaw's lever and the yaw on the lever's
    /// axis, and for rotary r the outer rotaries turning turn_r x (r and the inner ones turning the offset).
    fn body_jacobian(&self, b: &Body, q: &[f64]) -> Vec<[f64; 3]> {
        let mut out: Vec<[f64; 3]> = (0..self.n()).map(|i| if b.axes[i] { self.dirs[i] } else { [0.0; 3] }).collect();
        let mut add = |i: usize, dir: [f64; 3], t: f64| (0..3).for_each(|k| out[i][k] += dir[k] * t);
        if let Some(g) = self.gantry.as_ref().filter(|g| b.axes[g.axis]) {
            let across = g.across.filter(|&a| b.axes[a]);
            add(g.coord, self.dirs[g.axis], across.map_or(0.0, |a| q[a] + g.shift));
            if let Some(a) = across {
                add(a, self.dirs[g.axis], q[g.coord]);
            }
        }
        for (p, &r) in b.rotaries.iter().enumerate() {
            let inner = self.turned_by(&b.rotaries[p..], q, b.offset);
            add(r, self.turned_by(&b.rotaries[..p], q, cross(self.turn[r], inner)), 1.0);
        }
        out
    }
    /// The second derivative of body_point along the speeds v (dJ/dt v), exactly: 2 v_across v_yaw along the bridge's
    /// axis, and through each rotary R (speed w about u) of what it turns, x'' -> R x'' + 2 w u x (R x') + w^2 u x (u x R x).
    fn body_curve(&self, b: &Body, q: &[f64], v: &[f64]) -> [f64; 3] {
        let (mut x, mut d1, mut d2) = (b.offset, [0.0; 3], [0.0; 3]);
        for &r in b.rotaries.iter().rev() {
            let (u, w) = (self.turn[r], v[r]);
            let (y, y1) = (rotate(u, q[r], x), rotate(u, q[r], d1));
            let (uy, uy1, uuy) = (cross(u, y), cross(u, y1), cross(u, cross(u, y)));
            d2 = rotate(u, q[r], d2);
            (0..3).for_each(|k| d2[k] += 2.0 * w * uy1[k] + w * w * uuy[k]);
            d1 = [0, 1, 2].map(|k| y1[k] + w * uy[k]);
            x = y;
        }
        if let Some(g) = self.gantry.as_ref().filter(|g| b.axes[g.axis])
            && let Some(a) = g.across.filter(|&a| b.axes[a])
        {
            (0..3).for_each(|k| d2[k] += 2.0 * v[a] * v[g.coord] * self.dirs[g.axis][k]);
        }
        d2
    }
    /// The bodies whose place is not a sum of the axes' travels, as m J^T J at q (J = `body_jacobian`).
    fn general_mass(&self, q: &[f64], out: &mut [Vec<f64>]) {
        for b in self.bodies.iter().filter(|b| b.general) {
            let (j, m) = (self.body_jacobian(b, q), self.body_mass(b));
            for (i, row) in out.iter_mut().enumerate() {
                for (l, x) in row.iter_mut().enumerate() {
                    *x += m * (0..3).map(|k| j[i][k] * j[l][k]).sum::<f64>();
                }
            }
        }
    }
    /// What the speeds v ask of those bodies beside their mass matrix, as forces on the coordinates:
    /// -m J^T (dJ/dt v), the centripetal and Coriolis forces (`body_curve`). With the mass matrix m J^T J these are
    /// Lagrange's equations, so the energy keeps.
    fn inertial(&self, q: &[f64], v: &[f64]) -> Vec<f64> {
        let mut out = vec![0.0; self.n()];
        for b in self.bodies.iter().filter(|b| b.general) {
            let curve = self.body_curve(b, q, v);
            let (j, m) = (self.body_jacobian(b, q), self.body_mass(b));
            for (i, f) in out.iter_mut().enumerate() {
                *f -= m * (0..3).map(|k| j[i][k] * curve[k]).sum::<f64>();
            }
        }
        out
    }
    /// The yaw spring's and damper's force on the coordinates: -k psi - c psi' on a yawing gantry's yaw, none elsewhere.
    fn yaw_force(&self, q: &[f64], v: &[f64]) -> Vec<f64> {
        let mut out = vec![0.0; self.n()];
        if let Some(g) = &self.gantry {
            out[g.coord] = -g.stiffness * q[g.coord] - g.damping * v[g.coord];
        }
        out
    }
    /// The joint coordinates (the yaw 0) of a tool pose: with a tcp the straight axes stand off the tool's tip by the
    /// tool turned by the rotaries.
    fn joints(&self, tool: &[f64]) -> Vec<f64> {
        let mut q = tool.to_vec();
        q.resize(self.n(), 0.0);
        if let Some(t) = &self.tcp {
            // The tip on the workpiece where the table has turned it, less the tool turned by the head.
            let on = [0, 1, 2].map(|k| q[t.axes[k]] - t.pivot[k]);
            let at = self.turned_by(&t.table, &q, on);
            let r = self.turned_by(&t.rotaries, &q, t.tool);
            (0..3).for_each(|k| q[t.axes[k]] = t.pivot[k] + at[k] - r[k]);
        }
        q
    }
    /// `v` turned back by the rotaries (outermost first) at their angles in q: `turned_by` undone.
    fn unturned_by(&self, rotaries: &[usize], q: &[f64], mut v: [f64; 3]) -> [f64; 3] {
        for &r in rotaries {
            v = rotate(self.turn[r], -q[r], v);
        }
        v
    }
    /// The tool pose of the joint coordinates q.
    fn tool_of(&self, q: &[f64]) -> Vec<f64> {
        let mut tool = q[..self.nt()].to_vec();
        // The tool rides on the carriage across a yawing bridge: the yaw moves it along the bridge's axis.
        if let Some(g) = &self.gantry {
            tool[g.axis] += g.across.map_or(0.0, |a| q[a] + g.shift) * q[g.coord];
        }
        if let Some(t) = &self.tcp {
            let r = self.turned_by(&t.rotaries, q, t.tool);
            let tip = [0, 1, 2].map(|k| tool[t.axes[k]] + r[k] - t.pivot[k]);
            let on = self.unturned_by(&t.table, q, tip);
            (0..3).for_each(|k| tool[t.axes[k]] = t.pivot[k] + on[k]);
        }
        tool
    }
    /// The joints' speeds and accelerations for a tool pose, speed and acceleration (`joints` differentiated).
    fn joint_motion(&self, tool: &[f64], vel: &[f64], acc: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let q = self.joints(tool);
        let pad = |v: &[f64]| -> Vec<f64> { (0..self.n()).map(|i| v.get(i).copied().unwrap_or(0.0)).collect() };
        if self.tcp.is_none() {
            return (q, pad(vel), pad(acc));
        }
        let along = |dir: &[f64], h: f64| -> Vec<f64> {
            self.joints(&tool.iter().zip(dir).map(|(t, d)| t + h * d).collect::<Vec<_>>())
        };
        let size = |v: &[f64]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
        // The first derivative along a direction over 1 um, the second over 0.1 mm.
        let first = |dir: &[f64]| -> Vec<f64> {
            if size(dir) == 0.0 {
                return vec![0.0; self.n()];
            }
            let h = 1e-6 / size(dir);
            along(dir, h).iter().zip(along(dir, -h)).map(|(a, b)| (a - b) / (2.0 * h)).collect()
        };
        let (dq, mut ddq) = (first(vel), first(acc));
        if size(vel) > 0.0 {
            let h = 1e-4 / size(vel);
            let (a, b) = (along(vel, h), along(vel, -h));
            ddq.iter_mut().enumerate().for_each(|(i, x)| *x += (a[i] - 2.0 * q[i] + b[i]) / (h * h));
        }
        (q, dq, ddq)
    }
    /// The mass matrix of the axes at q: the bodies, the cable chains and the idlers.
    fn axis_mass(&self, q: &[f64]) -> Vec<Vec<f64>> {
        let n = self.n();
        let mut out = vec![vec![0.0; n]; n];
        for (i, m) in self.mass().into_iter().enumerate() {
            out[i][i] = m + if self.chain_axes[i] { self.chain.mass(q[i]).0 } else { 0.0 };
        }
        self.general_mass(q, &mut out);
        for (row, idle) in out.iter_mut().zip(self.idler_mass()) {
            row.iter_mut().zip(idle).for_each(|(a, b)| *a += b);
        }
        out
    }
    /// The idlers' share of the axes' mass matrix: each spins with the belt's speed past its carriage, s_d . v with v the
    /// axes' speeds on the frame, J / r^2 (s_d s_d').
    fn idler_mass(&self) -> Vec<Vec<f64>> {
        let n = self.n();
        let mut out = vec![vec![0.0; n]; n];
        for d in &self.idlers {
            for i in 0..n {
                for l in 0..n {
                    out[i][l] += d.mass * d.speed[i] * d.speed[l];
                }
            }
        }
        out
    }
    /// The frame's mass in its sway, with the bodies that do not move along its axis and so ride on it.
    fn frame_mass(&self, f: &FrameMode) -> f64 {
        f.mass + self.bodies.iter().filter(|b| !b.axes[f.axis]).map(|b| self.body_mass(b)).sum::<f64>()
    }
    /// Gravity's force on each coordinate at q: -m g along the straight axes, and through their places' Jacobian for
    /// the bodies `body_point` places.
    fn gravity(&self, q: &[f64]) -> Vec<f64> {
        let mut out: Vec<f64> = self.mass().iter().zip(&self.g_along).map(|(m, g)| -m * g).collect();
        for b in self.bodies.iter().filter(|b| b.general) {
            let (j, m) = (self.body_jacobian(b, q), self.body_mass(b));
            out.iter_mut().zip(&j).for_each(|(f, j)| *f -= m * self.g * j[2]);
        }
        out
    }
    /// The potential energy of gravity at q, from where it is with every coordinate at 0.
    fn potential(&self, q: &[f64]) -> f64 {
        let straight: f64 = self.mass().iter().zip(&self.g_along).zip(q).map(|((m, g), q)| m * g * q).sum();
        let general: f64 = self
            .bodies
            .iter()
            .filter(|b| b.general)
            .map(|b| self.body_mass(b) * self.g * self.body_point(b, q)[2])
            .sum();
        straight + general
    }
    /// Pulleys and rotors seen on the axes, J_pulley + i^2 J_rotor per motor, through K.
    fn drive_mass(&self) -> Vec<Vec<f64>> {
        let n = self.n();
        let mut out = vec![vec![0.0; n]; n];
        for (kk, row) in self.k.iter().enumerate() {
            let j = self.j_pulley[kk] + self.ratio[kk] * self.ratio[kk] * self.j_rotor[kk];
            for i in 0..n {
                for l in 0..n {
                    out[i][l] += j * row[i] * row[l];
                }
            }
        }
        out
    }
    /// The rigid model's mass matrix at q: bodies, chains, idlers, pulleys and rotors, and the belt's runs beyond its lumps.
    fn rigid_mass(&self, q: &[f64]) -> Vec<Vec<f64>> {
        let mut out = self.drive_mass();
        for ((row, axis), runs) in out.iter_mut().zip(self.axis_mass(q)).zip(self.runs_mass(q, [0.0; 3]).0) {
            row.iter_mut().zip(axis).zip(runs).for_each(|((a, b), c)| *a += b + c);
        }
        out
    }
    /// Torques at the motors for forces on the axes: (J^T)^-1 with J = diag(i) K.
    fn to_motors(&self, forces: &[f64]) -> Vec<f64> {
        (0..self.m())
            .map(|kk| (0..self.n()).map(|i| self.k_inv[i][kk] * forces[i]).sum::<f64>() / self.ratio[kk])
            .collect()
    }

    fn positions<'s>(&self, s: &'s [f64]) -> (&'s [f64], &'s [f64]) {
        let (n, v) = (self.n(), self.vel());
        (&s[..n], &s[v..v + n])
    }
    fn pulley_angle(&self, kk: usize, q: &[f64]) -> f64 {
        dot(&self.k[kk], q)
    }
    /// The motor angles at the coordinates q.
    fn motor_angles(&self, q: &[f64], out: &mut [f64]) {
        for (kk, out) in out.iter_mut().enumerate() {
            *out = self.ratio[kk] * self.pulley_angle(kk, q);
        }
    }
    /// The tool's speed at the coordinates q moving at v.
    fn tool_speed(&self, q: &[f64], v: &[f64]) -> Vec<f64> {
        if self.tcp.is_none() && self.gantry.is_none() {
            return v[..self.nt()].to_vec();
        }
        let size = v.iter().map(|x| x * x).sum::<f64>().sqrt();
        if size == 0.0 {
            return vec![0.0; self.nt()];
        }
        let h = 1e-6 / size;
        let at = |t: f64| self.tool_of(&q.iter().zip(v).map(|(q, v)| q + t * v).collect::<Vec<_>>());
        at(h).iter().zip(at(-h)).map(|(a, b)| (a - b) / (2.0 * h)).collect()
    }
    /// Where the 3D view draws a body at q.
    fn body_scene_at(&self, b: &Body, q: &[f64]) -> [f64; 3] {
        if !b.general {
            return self.scene_at(q, &b.axes);
        }
        let p = self.body_point(b, q);
        [p[0], p[1] - 0.03, p[2] + 0.06]
    }
    /// The frame's sway and its rate; zero without it.
    fn sway(&self, s: &[f64]) -> (f64, f64) {
        if self.frame.is_some() { (s[self.frame_at()], s[self.vel() + self.frame_at()]) } else { (0.0, 0.0) }
    }

    /// The pulley angles as far as the belt sections see them: each pulley's angle plus the belt's slip over it
    /// (effects.belt_tooth_jump) over the radius, and their rates. Elastic model only.
    fn fed(&self, s: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let (n, m, v) = (self.n(), self.m(), self.vel());
        let (mut angle, mut rate) = (s[n..n + m].to_vec(), s[v + n..v + n + m].to_vec());
        if !self.jumps.is_empty() {
            for (j, &kk) in self.jump_motor.iter().enumerate() {
                angle[kk] += s[self.jump_at(j)] / self.r[kk];
                rate[kk] += s[self.jump_at(j) + 1] / self.r[kk];
            }
        }
        (angle, rate)
    }
    fn sections(&self, s: &[f64]) -> Vec<SectionForce> {
        let (n, v) = (self.n(), self.vel());
        let (angle, rate) = self.fed(s);
        let (longer, longer_rate) = self.lean_lengths(s);
        self.sections
            .iter()
            .enumerate()
            .map(|(i, sec)| {
                let geo = sec.length + dot(&sec.c, &s[..n]) + longer[i];
                let mat = sec.mat0 + (0..self.m()).map(|kk| sec.f[kk] * self.r_mat[kk] * angle[kk]).sum::<f64>();
                let mat_rate: f64 = (0..self.m()).map(|kk| sec.f[kk] * self.r_mat[kk] * rate[kk]).sum();
                self.belt.section(geo, mat, dot(&sec.c, &s[v..v + n]) + longer_rate[i], mat_rate)
            })
            .collect()
    }
    fn geo(&self, q: &[f64]) -> Vec<f64> {
        self.sections.iter().map(|s| s.length + dot(&s.c, q)).collect()
    }
    fn mat(&self, s: &[f64]) -> Vec<f64> {
        let (angle, _) = self.fed(s);
        self.sections
            .iter()
            .map(|sec| sec.mat0 + (0..self.m()).map(|kk| sec.f[kk] * self.r_mat[kk] * angle[kk]).sum::<f64>())
            .collect()
    }
    /// The tension of each section: the belt's in the elastic model, pretension in the rigid one.
    fn tensions(&self, s: &[f64]) -> Vec<f64> {
        if self.elastic {
            self.sections(s).iter().map(|f| f.tension).collect()
        } else {
            vec![self.belt.pretension; self.sections.len()]
        }
    }

    /// The acceleration of each axis that the motor torques would give a loss-free rigid machine: what an overhang's
    /// tilt on a guide is worked out with. The true acceleration would make friction depend on itself, and its absolute
    /// value would turn every belt vibration into extra friction. Over the rigid model's whole mass matrix
    /// (`rigid_mass`: bodies, chains, idlers, pulleys, rotors and the belt's runs).
    fn commanded_acc(&self, q: &[f64], torque: &[f64]) -> Vec<f64> {
        let forces: Vec<f64> =
            (0..self.n()).map(|i| (0..self.m()).map(|kk| self.ratio[kk] * self.k[kk][i] * torque[kk]).sum()).collect();
        solve(self.rigid_mass(q), forces).unwrap_or_else(|| vec![0.0; self.n()])
    }

    /// The load on a guide: the weight across its axis of what it carries, and with an overhang the tilt
    /// 2 m a lever / spacing (and with side_load the overhang's inertia force m a).
    fn guide_load(&self, g: &Guide, q: &[f64], acc: &[f64]) -> f64 {
        let weight: f64 = g.carries.iter().map(|&b| self.body_mass(&self.bodies[b]) * self.g_across[g.axis]).sum();
        let Some(o) = &g.overhang else { return weight };
        let side = self.body_mass(&self.bodies[o.body]) * acc[o.axis];
        let lever = o.lever - o.lever_axis.map_or(0.0, |a| q[a]);
        let tilt = 2.0 * side * lever / o.spacing;
        weight + tilt.abs() + if o.side_load { side.abs() } else { 0.0 }
    }
    /// Friction level factor per guide: its load against that of what it carries without the payload.
    fn load_scales(&self, q: &[f64], torque: &[f64]) -> Vec<f64> {
        let acc = if self.guides.iter().any(|g| g.overhang.is_some()) {
            self.commanded_acc(q, torque)
        } else {
            vec![0.0; self.n()]
        };
        self.guides
            .iter()
            .map(|g| {
                let nominal: f64 = g.carries.iter().map(|&b| self.bodies[b].mass * self.g_across[g.axis]).sum();
                g.friction.load_scale(self.guide_load(g, q, &acc), nominal)
            })
            .collect()
    }
    /// Guide friction force and friction state rate per axis (zero where an axis has no guide).
    fn friction(&self, q: &[f64], v: &[f64], states: &[f64], torque: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let (mut force, mut rate) = (vec![0.0; self.n()], vec![0.0; self.n()]);
        for (g, scale) in self.guides.iter().zip(self.load_scales(q, torque)) {
            (force[g.axis], rate[g.axis]) = g.friction.eval(v[g.axis], states[g.axis], scale);
        }
        (force, rate)
    }
    /// The idlers' bearing friction as forces on the axes, and the power it takes.
    fn idler_friction(&self, v: &[f64]) -> (Vec<f64>, f64) {
        let (mut force, mut power) = (vec![0.0; self.n()], 0.0);
        for d in &self.idlers {
            let speed = dot(&d.speed, v);
            let drag = d.drag * (speed / d.smoothing).tanh();
            power += drag * speed;
            force.iter_mut().zip(&d.speed).for_each(|(f, s)| *f -= drag * s);
        }
        (force, power)
    }
    /// The rotary axes' bearing friction at the output (elastic model), the force against each axis's motion; the
    /// smoothing is the one of the shaft friction at the output, and the powers it takes per axis.
    fn bearing_force(&self, v: &[f64]) -> Vec<f64> {
        (0..self.n())
            .map(|i| {
                let [coulomb, viscous, smoothing] = self.bearing[i];
                if coulomb == 0.0 && viscous == 0.0 {
                    0.0
                } else {
                    coulomb * (v[i] / smoothing).tanh() + viscous * v[i]
                }
            })
            .collect()
    }
    /// The cable chains' drag on each axis (opposing it).
    fn chain_drag(&self, v: &[f64]) -> Vec<f64> {
        (0..self.n())
            .map(|i| if self.chain_axes[i] { self.chain.drag(v[i], self.chain_fade[i]) } else { 0.0 })
            .collect()
    }
    /// What a growing chain takes, m' w (w / 2 - v) on each axis with one (w its speed over ground, v along the axis), and
    /// on the frame -m' w^2 / 2 (see the module comment); m' v^2 / 2 against the motion without a frame.
    fn chain_inertia(&self, v: &[f64], sway_rate: f64) -> (Vec<f64>, f64) {
        let (mut force, mut on_frame) = (vec![0.0; self.n()], 0.0);
        for i in (0..self.n()).filter(|&i| self.chain_axes[i]) {
            let growth = self.chain.mass(0.0).1;
            let w = v[i] + if self.frame.as_ref().is_some_and(|f| f.axis == i) { sway_rate } else { 0.0 };
            force[i] = growth * w * (0.5 * w - v[i]);
            if self.frame.as_ref().is_some_and(|f| f.axis == i) {
                on_frame = -0.5 * growth * w * w;
            }
        }
        (force, on_frame)
    }
    /// The force on the tool along each axis: the process force while the cycle dwells, and an obstacle's push.
    fn process_force(&self) -> Vec<f64> {
        let mut force = self.dwelling_force();
        force.iter_mut().zip(&self.tool_push).for_each(|(f, p)| *f += p);
        force
    }
    /// The process force alone (its power is the budget's process row; a push's is the crash row, `sim::Sim::pushed`).
    fn dwelling_force(&self) -> Vec<f64> {
        if self.dwelling { self.process.clone() } else { vec![0.0; self.n()] }
    }
    /// The generalised force on the coordinates q of a force on the tool along its axes (`process_force`): its virtual
    /// work per unit of each coordinate, sum_i F_i d tool_i / d q_j. The tool's axes are the first coordinates, so
    /// that is F itself unless a tcp or a yawing bridge moves the tool with other coordinates too (by central
    /// differences of `tool_of` then).
    fn on_coordinates(&self, q: &[f64], tool: &[f64]) -> Vec<f64> {
        let (n, nt) = (self.n(), self.nt());
        let mut out: Vec<f64> = (0..n).map(|j| tool.get(j).copied().unwrap_or(0.0)).collect();
        if (self.tcp.is_none() && self.gantry.is_none()) || tool.iter().all(|f| *f == 0.0) {
            return out;
        }
        const STEP: f64 = 1e-7;
        for (j, out) in out.iter_mut().enumerate() {
            let (mut ahead, mut behind) = (q.to_vec(), q.to_vec());
            (ahead[j], behind[j]) = (q[j] + STEP, q[j] - STEP);
            let (a, b) = (self.tool_of(&ahead), self.tool_of(&behind));
            *out = (0..nt).map(|i| tool.get(i).copied().unwrap_or(0.0) * (a[i] - b[i]) / (2.0 * STEP)).sum();
        }
        out
    }
    /// The hard stops at the ends of axis i's joint; none on the yaw.
    fn stop_force(&self, i: usize, pos: f64, vel: f64) -> f64 {
        if i < self.nt() { self.stops.force(pos, vel, self.stroke[i]) } else { 0.0 }
    }

    /// The direction of the 3D view the frame sways in (`dirs` of its axis), zero without a swaying frame.
    fn sway_direction(&self) -> [f64; 3] {
        self.frame.as_ref().map_or([0.0; 3], |f| self.dirs[f.axis])
    }

    /// The belt run by run, where the spans give their speeds over ground (`machine.span` velocity_x, _y, _z; mu =
    /// `run_mass` per metre). Run s, L_s = L0_s + c_s . q long, moves at y_s = V_s v over ground (V_s its velocity rows),
    /// and where the frame sways at du along e all of the belt sways with it: y_s = V_s v + e du. Half the belt is lumped
    /// on each belt pulley already (`j_pulley`), 1/2 J_l (K_k v)^2 with J_l = lump r_k^2, so the runs add
    ///   T = 1/2 mu sum_s L_s |V_s v + e du|^2 - 1/2 sum_k J_l (K_k v)^2:
    /// over the coordinates mu sum L_s V_s' V_s - sum J_l K_k K_k', between them and the sway mu sum L_s V_s' e, and on the
    /// sway mu sum L_s (the belt's mass): (block, column, mass). All zero without runs.
    fn runs_mass(&self, q: &[f64], e: [f64; 3]) -> (Vec<Vec<f64>>, Vec<f64>, f64) {
        let n = self.n();
        let (mut block, mut column, mut total) = (vec![vec![0.0; n]; n], vec![0.0; n], 0.0);
        if self.run_mass == 0.0 {
            return (block, column, total);
        }
        for s in &self.spans {
            let Some(v) = &s.velocity else { continue };
            let weight = self.run_mass * (s.length + dot(&s.c, q));
            total += weight;
            for i in 0..n {
                column[i] += weight * (0..3).map(|k| v[k][i] * e[k]).sum::<f64>();
                for l in 0..n {
                    block[i][l] += weight * (0..3).map(|k| v[k][i] * v[k][l]).sum::<f64>();
                }
            }
        }
        for (kk, row) in self.k.iter().enumerate().filter(|(kk, _)| self.direct[*kk].is_none()) {
            let lumped = self.lump * self.r[kk] * self.r[kk];
            for i in 0..n {
                for l in 0..n {
                    block[i][l] -= lumped * row[i] * row[l];
                }
            }
        }
        (block, column, total)
    }

    /// The forces the runs' speeds ask for in Lagrange's equations of `runs_mass` (whose lengths change, dL_s/dt = c_s . v):
    /// on the coordinates -mu sum_s [(c_s . v) V_s' y_s - 1/2 c_s |y_s|^2], on the sway -mu sum_s (c_s . v) e . y_s.
    fn runs_speed(&self, v: &[f64], e: [f64; 3], du: f64) -> (Vec<f64>, f64) {
        let n = self.n();
        let (mut force, mut on_frame) = (vec![0.0; n], 0.0);
        if self.run_mass == 0.0 {
            return (force, on_frame);
        }
        let mu = self.run_mass;
        for s in &self.spans {
            let Some(rows) = &s.velocity else { continue };
            let y: [f64; 3] = std::array::from_fn(|k| dot(&rows[k], v) + e[k] * du);
            let (grows, squared) = (dot(&s.c, v), y.iter().map(|y| y * y).sum::<f64>());
            for (i, f) in force.iter_mut().enumerate() {
                *f -= mu * (grows * (0..3).map(|k| rows[k][i] * y[k]).sum::<f64>() - 0.5 * s.c[i] * squared);
            }
            on_frame -= mu * grows * (0..3).map(|k| e[k] * y[k]).sum::<f64>();
        }
        (force, on_frame)
    }

    /// The elastic model's mass matrix over its coordinates that carry mass: the axes (on the frame), then the frame's
    /// sway and the lean, where they are; T = 1/2 y' M y of all but the pulleys and rotors (on their own states). The
    /// bodies, chains and idlers move at their axes' speed plus the sway along its axis (`axis_mass` on the speeds over
    /// ground), the bodies that do not move along it and the frame with the sway (`frame_mass`), the leaning body at its
    /// axis's speed plus the lean's, and the belt's runs as `runs_mass` has them. The idlers' inertia rides on their axes'
    /// speeds alone: the belt's speed past a carriage is the difference of two speeds over ground, in which the frame's
    /// sway cancels, so the idlers are in the axes' block and not in the sway's row and column.
    fn elastic_mass(&self, q: &[f64]) -> Vec<Vec<f64>> {
        let n = self.n();
        let d =
            n + usize::from(self.frame.is_some()) + usize::from(self.lean.is_some()) + usize::from(self.beam.is_some());
        let mut mass = vec![vec![0.0; d]; d];
        let axis_mass = self.axis_mass(q);
        let (runs, column, belt) = self.runs_mass(q, self.sway_direction());
        for i in 0..n {
            for l in 0..n {
                mass[i][l] = axis_mass[i][l] + runs[i][l];
            }
        }
        if let Some(f) = &self.frame {
            let idle = self.idler_mass();
            for i in 0..n {
                (mass[i][n], mass[n][i]) = (
                    axis_mass[i][f.axis] - idle[i][f.axis] + column[i],
                    axis_mass[f.axis][i] - idle[f.axis][i] + column[i],
                );
            }
            mass[n][n] = axis_mass[f.axis][f.axis] - idle[f.axis][f.axis] + self.frame_mass(f) + belt;
        }
        if let Some(lean) = &self.lean {
            let (at, body) = (self.lean_row(), self.body_mass(&self.bodies[lean.body]));
            (mass[lean.axis][at], mass[at][lean.axis], mass[at][at]) = (body, body, body);
            if self.frame.as_ref().is_some_and(|f| f.axis == lean.axis) {
                (mass[n][at], mass[at][n]) = (body, body);
            }
        }
        // The beam's deflection carries the bodies that move along the beam and its own modal mass; the vertical axis's
        // coordinate is relative to the carriage, so the bodies that move along both have z' - w' over ground (`BeamMode`).
        if let Some(b) = &self.beam {
            let (at, (carried, riding)) = (self.beam_row(), self.beam_load());
            mass[at][at] = carried + self.beam_mode(q)[1];
            (mass[b.vertical][at], mass[at][b.vertical]) = (-riding, -riding);
        }
        mass
    }

    /// The axes' accelerations, the frame's and the friction states' rates of the elastic model, with the axes' own
    /// forces (all but the inertia) that go with them.
    fn elastic_derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let (n, m, v) = (self.n(), self.m(), self.vel());
        let sec = self.sections(s);
        let (fa, ba) = (self.friction_at(), self.brake_at());
        let (q, vq) = (&s[..n], &s[v..v + n]);
        let (friction, rates) = self.friction(q, vq, &s[fa..fa + n], torque);
        out[..v].copy_from_slice(&s[v..2 * v]);
        let (gravity, (idlers, _), drag) = (self.gravity(q), self.idler_friction(vq), self.chain_drag(vq));
        let bearing = self.bearing_force(vq);
        let (inertial, yaw) = (self.inertial(q, vq), self.yaw_force(q, vq));
        let (u, du) = self.sway(s);
        // What the screws, racks, linear motors and rotaries pass to their axes (and a gantry's yaw): their gearbox's
        // output.
        let mut driven = vec![0.0; n];
        for (kk, d) in self.direct.iter().enumerate() {
            if d.is_some() {
                let (theta, w_theta) = self.output(s, kk);
                let out = self.gear_torque(kk, s[n + m + kk], s[v + n + m + kk], theta, w_theta).output;
                driven.iter_mut().zip(&self.k[kk]).for_each(|(f, k)| *f += k * out);
            }
        }
        // What the frame and the things on it put on the axes: it takes all of it back.
        let mut force: Vec<f64> = (0..n)
            .map(|i| {
                let belt: f64 = self.sections.iter().zip(&sec).map(|(x, f)| -x.c[i] * f.tension).sum();
                belt - friction[i] + idlers[i] - drag[i] - bearing[i]
                    + self.stop_force(i, q[i], vq[i])
                    + gravity[i]
                    + driven[i]
                    + inertial[i]
                    + yaw[i]
            })
            .collect();
        let (chain, chain_on_frame) = self.chain_inertia(vq, du);
        let process = self.process_force();
        let on_q = self.on_coordinates(q, &process);
        let (runs, runs_on_frame) = self.runs_speed(vq, self.sway_direction(), du);
        for i in 0..n {
            force[i] += chain[i] + on_q[i] + runs[i];
        }
        out[fa..fa + n].copy_from_slice(&rates[..n]);
        // Lagrange's equations over the axes (on the frame), the frame's sway and the lean, M a = F (`elastic_mass`). The
        // frame takes its spring and damper and what the chain's and the belt's runs' speeds ask of it; the forces between
        // it and the axes act inside. The process force acts on the tool, which the sway and the lean move too (`tip` per
        // unit of lean). The lean's spring acts between the carriage and the body, and stiffens as the lever axis
        // travels: -w^2 / 2 dk/dz there; and the body's weight rises as it leans, a pendulum's stiffness c on top of k
        // (`lean_weight`), whose rate with the lever axis acts there too.
        if let Some(f) = &self.frame {
            force.push(
                -f.stiffness * u - f.damping * du + chain_on_frame + chain[f.axis] + process[f.axis] + runs_on_frame,
            );
        }
        //
        // The belt runs' geometric stiffness (`lean_runs`) is in the section tensions through the lengths: it holds the lean
        // back with sum T_i c_i u and pushes on the lever axis with -1/2 u^2 sum T_i c_i'. The process force's vertical part
        // works on the tip's rise rho = a u^2 (`rise_coefficients`): 2 a u on the lean, a' u^2 on the lever axis.
        if let Some(lean) = &self.lean {
            let (w, dw) = self.leaning(s);
            let (k, dk, tip) = flex_at(lean, q);
            let (c, dc) = lean_weight(lean, q, self.body_mass(&self.bodies[lean.body]), self.g);
            let mut on_lean = 0.0;
            let mut rise = 0.0;
            if let Some(z) = lean.lever_axis {
                let (runs, mut ahead, mut behind) = (self.lean_runs(q), q.to_vec(), q.to_vec());
                (ahead[z], behind[z]) = (q[z] + 1e-6, q[z] - 1e-6);
                let (ahead, behind) = (self.lean_runs(&ahead), self.lean_runs(&behind));
                on_lean = (0..runs.len()).map(|i| sec[i].tension * runs[i]).sum::<f64>() * w;
                let on_z: f64 =
                    (0..runs.len()).map(|i| sec[i].tension * (ahead[i] - behind[i]) / 2e-6).sum::<f64>() * 0.5 * w * w;
                let [a, da, _] = rise_coefficients(lean, q);
                force[z] += -0.5 * w * w * (dk + dc) - on_z + process[z] * da * w * w;
                rise = 2.0 * process[z] * a * w;
            }
            force.push(tip * process[lean.axis] + rise - on_lean - (k + c) * w - lean.damping * dw);
        }
        // The beam mode (`BeamMode`): M(x) w'' + M' x' w' + c w' + k w = (carried weight) - F_vertical on w, and on the beam's
        // axis the speed term 1/2 M' w'^2 less the stiffness's change with it, 1/2 k' w^2.
        if let Some(b) = &self.beam {
            let ((w, dw), [k, _, c, dk, dm]) = (self.deflection(s), self.beam_mode(q));
            force[b.axis] += 0.5 * dm * dw * dw - 0.5 * dk * w * w;
            force.push(self.beam_load().0 * self.g - process[b.vertical] - k * w - c * dw - dm * vq[b.axis] * dw);
        }
        let size = force.len();
        let acc = solve(self.elastic_mass(q), force).unwrap_or_else(|| vec![0.0; size]);
        out[v..v + n].copy_from_slice(&acc[..n]);
        if self.frame.is_some() {
            out[v + self.frame_at()] = acc[n];
        }
        if self.lean.is_some() {
            out[v + self.lean_at()] = acc[self.lean_row()];
        }
        if self.beam.is_some() {
            out[v + self.beam_at()] = acc[self.beam_row()];
        }
        for kk in 0..m {
            let (phi, w_phi) = (s[n + m + kk], s[v + n + m + kk]);
            let (theta, w_theta) = self.output(s, kk);
            let gear = self.gear_torque(kk, phi, w_phi, theta, w_theta);
            if self.direct[kk].is_some() {
                // Its pulley is the axis: the pulley's states follow it.
                out[v + n + kk] = dot(&self.k[kk], &acc);
            } else {
                let belt: f64 =
                    self.r_mat[kk] * self.sections.iter().zip(&sec).map(|(x, f)| x.f[kk] * f.material).sum::<f64>();
                let r = self.r[kk];
                // While its teeth are out of mesh the pulley feels only the ridge it holds the sliding belt back with.
                let (on_pulley, j_pulley) = match self.jump_of[kk].map(|j| (j, &self.jumps[j])) {
                    Some((j, (jump, ..))) => {
                        let slip = &s[self.jump_at(j)..];
                        (jump.pulley_torque(slip, belt, r), jump.pulley_inertia(slip, self.j_pulley[kk], r))
                    }
                    None => (belt, self.j_pulley[kk]),
                };
                out[v + n + kk] = (gear.output + on_pulley + r * self.mesh.force(r * theta)) / j_pulley;
                if let Some((j, (jump, ..))) = self.jump_of[kk].map(|j| (j, &self.jumps[j])) {
                    let at = self.jump_at(j);
                    jump.rates(&s[at..], belt / r, r * out[v + n + kk], &mut out[at..]);
                }
            }
            let (brake, brake_rate) = self.brakes[kk].torque(self.brake_engaged, w_phi, s[ba + kk]);
            let drive = self.ripples[kk].apply(torque[kk], phi) - self.shafts[kk].torque(w_phi) - brake;
            out[v + n + m + kk] =
                (drive - gear.rotor - self.worm_drag(kk, gear.rotor, w_phi, drive)) / self.j_rotor[kk];
            out[ba + kk] = brake_rate;
        }
        out[self.slack_at()] = if sec.iter().any(|f| f.tension <= 0.0) { 1.0 } else { 0.0 };
    }

    /// Rigid model. The gearbox loses torque only on what it transmits, which is the motor torque less what
    /// accelerates the rotor itself; that depends on the acceleration solved for, and three passes settle it.
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> Rigid {
        let (n, m) = (self.n(), self.m());
        let (q, v) = (&s[..n], &s[n..2 * n]);
        let (fa, ba) = (self.friction_at(), self.brake_at());
        let (friction, rates) = self.friction(q, v, &s[fa..fa + n], torque);
        let (angle, speed): (Vec<f64>, Vec<f64>) = (0..m)
            .map(|kk| (self.ratio[kk] * self.pulley_angle(kk, q), self.ratio[kk] * self.pulley_angle(kk, v)))
            .unzip();
        // The torque each rotor passes on, before its own inertia and the gearbox; a brake holds the rotor.
        let drive: Vec<f64> = (0..m)
            .map(|kk| {
                self.ripples[kk].apply(torque[kk], angle[kk])
                    - self.shafts[kk].torque(speed[kk])
                    - self.brakes[kk].torque(self.brake_engaged, speed[kk], s[ba + kk]).0
            })
            .collect();
        let (gravity, (idlers, _), drag, (chain, _), process) = (
            self.gravity(q),
            self.idler_friction(v),
            self.chain_drag(v),
            self.chain_inertia(v, 0.0),
            self.on_coordinates(q, &self.process_force()),
        );
        let (inertial, yaw, runs) = (self.inertial(q, v), self.yaw_force(q, v), self.runs_speed(v, [0.0; 3], 0.0).0);
        let base: Vec<f64> = (0..n)
            .map(|i| {
                let mesh: f64 = (0..m)
                    .map(|kk| self.r[kk] * self.mesh_force(kk, self.r[kk] * angle[kk] / self.ratio[kk]) * self.k[kk][i])
                    .sum();
                mesh - friction[i] + self.stop_force(i, q[i], v[i]) + gravity[i] + idlers[i] - drag[i]
                    + chain[i]
                    + process[i]
                    + inertial[i]
                    + yaw[i]
                    + runs[i]
            })
            .collect();
        let mut fixed = self.rigid_mass(q);
        for kk in 0..m {
            let rotor = self.ratio[kk] * self.ratio[kk] * self.j_rotor[kk];
            for i in 0..n {
                for l in 0..n {
                    fixed[i][l] -= rotor * self.k[kk][i] * self.k[kk][l];
                }
            }
        }
        let solved = Rigid { acc: vec![0.0; n], rates, out: vec![0.0; m], into: vec![0.0; m] };
        if self.worms.iter().flatten().any(|w| w.back_efficiency <= 0.0) {
            return self.rigid_locking(&fixed, &base, &drive, v, solved);
        }
        // One pass with the factors (output, rotor) of each motor's gear, flat: the solution, and the factors of the torques
        // it puts into the gears.
        let pass = |flat: &Vec<f64>| -> (Rigid, Vec<f64>) {
            let mut solved = solved.clone();
            let loss: Vec<f64> = flat.chunks(2).map(|f| f[0] / f[1]).collect();
            let mut a = fixed.clone();
            let mut b = base.clone();
            for kk in 0..m {
                let rotor = self.ratio[kk] * self.ratio[kk] * self.j_rotor[kk];
                for i in 0..n {
                    b[i] += loss[kk] * self.ratio[kk] * drive[kk] * self.k[kk][i];
                    for l in 0..n {
                        a[i][l] += loss[kk] * rotor * self.k[kk][i] * self.k[kk][l];
                    }
                }
            }
            let acc = solve(a, b).unwrap_or_else(|| vec![0.0; n]);
            for kk in 0..m {
                // What the gearbox output carries: what the rotor's own acceleration and the losses of this pass leave.
                let into =
                    self.ratio[kk] * (drive[kk] - self.j_rotor[kk] * self.ratio[kk] * self.pulley_angle(kk, &acc));
                (solved.out[kk], solved.into[kk]) = (loss[kk] * into, into);
            }
            solved.acc = acc;
            let next = (0..m)
                .flat_map(|kk| {
                    let (output, rotor) = self.rigid_factors(kk, solved.into[kk], self.pulley_angle(kk, v));
                    [output, rotor]
                })
                .collect();
            (solved, next)
        };
        // The factors at their fixed point (`mech::settled_loss`): a gear's loss fades in smoothly over its power, and so
        // does a worm's that turns back (`worm::factors`), so they settle as the other machines' do.
        // Three passes left up to 1.6e-2 of the acceleration on the shipped files' rigid models and 1.2e-2 on a worm that
        // turns back.
        pass(&sim_components::mech::settled_loss(vec![1.0; 2 * m], |flat| pass(flat).1)).0
    }

    /// `rigid_solve` with a self-locking worm (`worm::factors`), whose rotor takes no share of the load at rest and
    /// pushes against it while it is driven back: the outputs o are solved for beside the accelerations, so that a
    /// rotor factor of 0 stays solvable. With `fixed` the mass matrix less the rotors and `base` the axes' forces, the
    /// axes take fixed acc - K^T o = base, and each rotor puts into its gearbox what its motor's torque (`drive`) leaves
    /// after its own acceleration, into = i (drive - J i K acc), of which the output gets o = into * output factor /
    /// rotor factor: rotor factor o + i^2 J K acc = output factor i drive.
    ///
    /// A locking worm moving (`worm`, the module comment) drags D = into - o = s max((1 / e - 1) s o,
    /// (1 - eta_back) max(-s o_rest, 0)) at the wheel, s the sign of its speed, e its forward efficiency times its
    /// gearbox's, o_rest the torque its output holds at rest against the axes' forces (K^T o_rest = -base): its row is
    /// the driving flank's, o + e i^2 J K acc = e i drive, or the drag's, o + i^2 J K acc = i drive - s C with
    /// C = `worm::overhauling_drag`, whichever holds at the solution. That drag rises with o, so exactly one choice holds
    /// (both at the seam, where they agree): every choice is solved and the one that holds is taken, no starting guess.
    /// Its output at rest, or slower than `worm::STICK_SPEED`, sticks (K acc = -K v / `worm::STICK_TIME_S`, 0 at rest)
    /// while the drag that takes, o - i drive, lies between what the worm gives raising the load, -(1 / eta - 1) o_rest,
    /// and lowering it, (1 - eta_back) o_rest; beyond, it breaks loose with the drag at that bound.
    fn rigid_locking(&self, fixed: &[Vec<f64>], base: &[f64], drive: &[f64], v: &[f64], mut solved: Rigid) -> Rigid {
        let (n, m) = (self.n(), self.m());
        let kt: Vec<Vec<f64>> = (0..n).map(|i| (0..m).map(|kk| self.k[kk][i]).collect()).collect();
        let holds = lstsq(&kt, &base.iter().map(|f| -f).collect::<Vec<_>>()).unwrap_or_else(|| vec![0.0; m]);
        let locking: Vec<Option<WormNode>> =
            self.worms.iter().map(|w| w.filter(|w| w.back_efficiency <= 0.0)).collect();
        // Per motor: (output factor, rotor factor), the drag a locking worm broken loose from rest takes, (Stuck) it
        // sticks, or (Moving) a locking worm moving with its forward efficiency e and its overhauling flank's drag C.
        #[derive(Clone, Copy, PartialEq)]
        enum Row {
            Factors(f64, f64),
            Drag(f64),
            Stuck,
            Moving(f64, f64),
        }
        let speed = |kk: usize| self.pulley_angle(kk, v);
        let sticks = |kk: usize| speed(kk).abs() < crate::worm::STICK_SPEED;
        let moving_row = |kk: usize, w: &WormNode, out: f64| {
            let e = w.efficiency * self.gears[kk].loss_factor(out.abs(), speed(kk).abs());
            Row::Moving(e, crate::worm::overhauling_drag(w, holds[kk], speed(kk)))
        };
        let mut rows: Vec<Row> = (0..m)
            .map(|kk| match locking[kk] {
                Some(_) if sticks(kk) => Row::Stuck,
                Some(w) => moving_row(kk, &w, 0.0),
                None => Row::Factors(1.0, 1.0),
            })
            .collect();
        let moving: Vec<usize> = (0..m).filter(|&kk| matches!(rows[kk], Row::Moving(..))).collect();
        // The solution with each moving worm on its driving flank (bit set in `flanks`) or dragging.
        let solve_rows = |rows: &[Row], flanks: usize| -> Vec<f64> {
            let mut a = vec![vec![0.0; n + m]; n + m];
            let mut b = base.to_vec();
            for i in 0..n {
                a[i][..n].copy_from_slice(&fixed[i]);
                for kk in 0..m {
                    a[i][n + kk] = -self.k[kk][i];
                }
            }
            for (kk, row) in rows.iter().enumerate() {
                let inertia = self.ratio[kk] * self.ratio[kk] * self.j_rotor[kk];
                let into = self.ratio[kk] * drive[kk];
                let driving = moving.iter().position(|&j| j == kk).is_some_and(|bit| flanks >> bit & 1 == 1);
                let (at_acc, at_o, rhs) = match *row {
                    Row::Factors(output, rotor) => (output * inertia, rotor, output * into),
                    Row::Drag(drag) => (inertia, 1.0, into + drag),
                    Row::Stuck => (1.0, 0.0, -speed(kk) / crate::worm::STICK_TIME_S),
                    Row::Moving(e, _) if driving => (e * inertia, 1.0, e * into),
                    Row::Moving(_, c) => (inertia, 1.0, into - speed(kk).signum() * c),
                };
                for l in 0..n {
                    a[n + kk][l] = at_acc * self.k[kk][l];
                }
                a[n + kk][n + kk] = at_o;
                b.push(rhs);
            }
            solve(a, b).unwrap_or_else(|| vec![0.0; n + m])
        };
        // How far the moving worms' flanks miss the drag law at a solution: the driving flank's drag (1 / e - 1) s o
        // has to reach C, the dragging one's must not exceed it.
        let missed = |rows: &[Row], flanks: usize, x: &[f64]| -> f64 {
            moving
                .iter()
                .enumerate()
                .map(|(bit, &kk)| {
                    let Row::Moving(e, c) = rows[kk] else { return 0.0 };
                    let flank = (1.0 / e - 1.0) * speed(kk).signum() * x[n + kk];
                    if flanks >> bit & 1 == 1 { (c - flank).max(0.0) } else { (flank - c).max(0.0) }
                })
                .sum()
        };
        // Rounds until the rows settle: a gearbox's loss factor fades over its power (`mech::settled_loss`), and a
        // locking worm at rest that breaks loose takes its drag from the round that found it stuck.
        for _ in 0..12 {
            let x = (0..1usize << moving.len())
                .map(|flanks| {
                    let x = solve_rows(&rows, flanks);
                    (missed(&rows, flanks, &x), x)
                })
                .min_by(|a, b| a.0.total_cmp(&b.0))
                .map(|(_, x)| x)
                .unwrap_or_default();
            solved.acc = x[..n].to_vec();
            for kk in 0..m {
                solved.into[kk] = self.ratio[kk]
                    * (drive[kk] - self.j_rotor[kk] * self.ratio[kk] * self.pulley_angle(kk, &solved.acc));
                solved.out[kk] = x[n + kk];
            }
            let before = rows.clone();
            for kk in 0..m {
                rows[kk] = match (rows[kk], locking[kk]) {
                    (Row::Stuck | Row::Drag(_), Some(worm)) if sticks(kk) => {
                        // Stuck, or broken loose this step: the bounds of the drag at rest.
                        let ends =
                            [-(1.0 / worm.efficiency - 1.0) * holds[kk], (1.0 - worm.back_efficiency) * holds[kk]];
                        let (low, high) = (ends[0].min(ends[1]), ends[0].max(ends[1]));
                        let drag = solved.out[kk] - solved.into[kk];
                        match rows[kk] {
                            Row::Stuck if (low..=high).contains(&drag) => Row::Stuck,
                            Row::Stuck => Row::Drag(drag.clamp(low, high)),
                            kept => kept,
                        }
                    }
                    (_, Some(w)) => moving_row(kk, &w, solved.out[kk]),
                    (_, None) => {
                        let (output, rotor) = self.rigid_factors(kk, solved.into[kk], speed(kk));
                        Row::Factors(output, rotor)
                    }
                };
            }
            if rows == before {
                break;
            }
        }
        solved
    }

    /// Kinetic, potential and elastic energy (see `ENERGY_PARTS`).
    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        let (n, m, v) = (self.n(), self.m(), self.vel());
        let q = &s[..n];
        let potential = self.potential(q);
        let mut stops: f64 = (0..self.nt()).map(|i| self.stops.energy(q[i], self.stroke[i])).sum();
        // The gantry's yaw spring.
        stops += self.gantry.as_ref().map_or(0.0, |g| 0.5 * g.stiffness * q[g.coord] * q[g.coord]);
        let vq = &s[v..v + n];
        let quadratic = |mass: &[Vec<f64>], w: &[f64]| 0.5 * (0..n).map(|i| w[i] * dot(&mass[i], w)).sum::<f64>();
        if !self.elastic {
            return [quadratic(&self.rigid_mass(q), vq), potential, stops];
        }
        // Over the axes, the sway and the lean (`elastic_mass`).
        let (u, du) = self.sway(s);
        let frame = self.frame.as_ref().map_or(0.0, |f| 0.5 * f.stiffness * u * u);
        let mut speeds = vq.to_vec();
        speeds.extend(self.frame.as_ref().map(|_| du));
        speeds.extend(self.lean.as_ref().map(|_| self.leaning(s).1));
        speeds.extend(self.beam.as_ref().map(|_| self.deflection(s).1));
        let elastic_mass = self.elastic_mass(q);
        let kinetic = 0.5 * speeds.iter().zip(&elastic_mass).map(|(a, row)| a * dot(row, &speeds)).sum::<f64>();
        let mut turning: f64 = (0..m)
            .map(|kk| {
                0.5 * self.j_pulley[kk] * s[v + n + kk].powi(2) + 0.5 * self.j_rotor[kk] * s[v + n + m + kk].powi(2)
            })
            .sum();
        for (j, (jump, ..)) in self.jumps.iter().enumerate() {
            let kk = self.jump_motor[j];
            turning += jump.slip_energy(&s[self.jump_at(j)..], self.r[kk] * s[v + n + kk]);
        }
        let lean = self.lean.as_ref().map_or(0.0, |lean| 0.5 * flex_at(lean, q).0 * self.leaning(s).0.powi(2));
        // The leaning body's weight rises as it leans: its potential 1/2 c w^2 (`lean_weight`).
        let potential = potential
            + self.lean.as_ref().map_or(0.0, |lean| {
                let c = lean_weight(lean, q, self.body_mass(&self.bodies[lean.body]), self.g).0;
                0.5 * c * self.leaning(s).0.powi(2)
            });
        let (geo, mat, longer) = (self.geo(q), self.mat(s), self.lean_lengths(s).0);
        let belt: f64 = (0..geo.len()).map(|i| self.belt.energy(geo[i] + longer[i], mat[i])).sum();
        let gears: f64 = (0..m).map(|kk| self.gears[kk].energy(s[n + m + kk], self.output(s, kk).0)).sum();
        // The beam: the carried weight at w lower, and its spring.
        let (w, k) = (self.deflection(s).0, self.beam_mode(q)[0]);
        let potential = potential - self.beam_load().0 * self.g * w;
        let beam = 0.5 * k * w * w;
        [kinetic + turning, potential, stops + belt + gears + frame + lean + beam]
    }

    /// The loss powers in the order of `effect_infos`, and per motor the gearbox loss as a torque at the rotor and the
    /// brake torque, which the torque breakdown takes too.
    fn losses(&self, s: &[f64], torque: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let (n, m, v) = (self.n(), self.m(), self.vel());
        let (q, vq) = self.positions(s);
        let fa = self.friction_at();
        let (friction, _) = self.friction(q, vq, &s[fa..fa + n], torque);
        let (mut angle, mut speed) = (vec![0.0; m], vec![0.0; m]);
        self.motor_state(s, &mut angle, &mut speed);
        let (mut gear_loss, mut gear_power, mut brake, mut mesh_in) = (vec![0.0; m], vec![0.0; m], vec![0.0; m], 0.0);
        let (mut belt, mut tool_v) = (0.0, vq.to_vec());
        if self.elastic {
            for kk in 0..m {
                let (phi, w_phi) = (s[n + m + kk], s[v + n + m + kk]);
                let (theta, w_theta) = self.output(s, kk);
                let gear = self.gear_torque(kk, phi, w_phi, theta, w_theta);
                brake[kk] = self.brakes[kk].torque(self.brake_engaged, w_phi, s[self.brake_at() + kk]).0;
                let drive = self.ripples[kk].apply(torque[kk], phi) - self.shafts[kk].torque(w_phi) - brake[kk];
                let drag = self.worm_drag(kk, gear.rotor, w_phi, drive);
                gear_loss[kk] = gear.rotor + drag - gear.output / self.ratio[kk];
                gear_power[kk] = self.gear_loss_power(kk, phi, w_phi, theta, w_theta, drag);
                mesh_in += self.r[kk] * self.mesh_force(kk, self.r[kk] * theta) * w_theta;
            }
            let (geo, mat, (_, rate)) = (self.geo(q), self.mat(s), self.fed(s));
            let (longer, longer_rate) = self.lean_lengths(s);
            for (i, sec) in self.sections.iter().enumerate() {
                let mat_rate: f64 = (0..m).map(|kk| sec.f[kk] * self.r_mat[kk] * rate[kk]).sum();
                belt += self.belt.loss_power(geo[i] + longer[i], mat[i], dot(&sec.c, vq) + longer_rate[i], mat_rate);
            }
            if let Some(f) = &self.frame {
                tool_v[f.axis] += self.sway(s).1;
            }
            // The tool's velocity (`tool_state`): with the lean's tip, its rise and the beam's deflection.
            if let Some(lean) = &self.lean {
                let ((u, du), [a, da, _]) = (self.leaning(s), rise_coefficients(lean, q));
                tool_v[lean.axis] += self.tip(q) * du;
                if let Some(z) = lean.lever_axis {
                    tool_v[z] += 2.0 * a * u * du + da * u * u * vq[z];
                }
            }
            if let Some(b) = &self.beam {
                tool_v[b.vertical] -= self.deflection(s).1;
            }
        } else {
            let solved = self.rigid_solve(s, torque);
            for kk in 0..m {
                brake[kk] = self.brakes[kk].torque(self.brake_engaged, speed[kk], s[self.brake_at() + kk]).0;
                gear_loss[kk] = (solved.into[kk] - solved.out[kk]) / self.ratio[kk];
                gear_power[kk] = gear_loss[kk] * speed[kk];
                mesh_in += self.mesh_force(kk, self.r[kk] * angle[kk] / self.ratio[kk])
                    * self.r[kk]
                    * self.pulley_angle(kk, vq);
            }
        }
        let mut powers: Vec<f64> = self.guides.iter().map(|g| friction[g.axis] * vq[g.axis]).collect();
        // The gearboxes' drag acts on the rotors but is the gearboxes' loss.
        let drag: Vec<f64> = (0..m).map(|kk| self.shafts[kk].drag_power(speed[kk])).collect();
        // The rotaries' bearing friction at the output is booked with the shaft friction of the motor that turns the axis.
        let bearing = self.bearing_force(vq);
        let at_output = |kk: usize| match self.direct[kk].as_ref() {
            Some(d) => {
                bearing[d.axis] * vq[d.axis] / self.direct.iter().flatten().filter(|o| o.axis == d.axis).count() as f64
            }
            None => 0.0,
        };
        powers.extend(
            (0..m).map(|kk| (self.shafts[kk].torque(speed[kk]) + brake[kk]) * speed[kk] - drag[kk] + at_output(kk)),
        );
        powers.extend((0..m).map(|kk| gear_power[kk] + drag[kk]));
        if !self.idlers.is_empty() {
            powers.push(self.idler_friction(vq).1);
        }
        if self.chain_axes.contains(&true) {
            powers.push(dot(&self.chain_drag(vq), vq));
        }
        powers.push(belt);
        if self.frame_node {
            powers.push(self.frame.as_ref().map_or(0.0, |f| f.damping * self.sway(s).1.powi(2)));
        }
        if self.lean_node.is_some() {
            powers.push(self.lean.as_ref().map_or(0.0, |lean| lean.damping * self.leaning(s).1.powi(2)));
        }
        if let Some(g) = &self.gantry {
            powers.push(g.damping * vq[g.coord] * vq[g.coord]);
        }
        let ripple: f64 =
            (0..m).map(|kk| (torque[kk] - self.ripples[kk].apply(torque[kk], angle[kk])) * speed[kk]).sum();
        let stops: f64 = (0..self.nt()).map(|i| self.stops.loss_power(q[i], vq[i], self.stroke[i])).sum();
        powers.extend([stops, ripple - mesh_in - dot(&self.dwelling_force(), &tool_v)]);
        if !self.jumps.is_empty() {
            powers.push(self.jump_loss);
        }
        if self.beam.is_some() {
            powers.push(self.beam_mode(q)[2] * self.deflection(s).1.powi(2));
        }
        (powers, gear_loss, brake)
    }

    /// The signals of the effect views, in the order of `effect_infos`. Torque parts per motor, through (J^T)^-1:
    /// inertia from the mass matrices times the axes' accelerations (bodies, chains and idlers at their speed over
    /// ground, pulleys and rotors as a rigid machine would turn them, and what picking up chain costs), gravity, and
    /// friction (guides, idlers, chains, and at the motor its shaft friction, brake and gearbox loss); the rest is the
    /// motor torque less these, so belt and gearbox ringing are in it.
    fn view_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (n, m) = (self.n(), self.m());
        let (q, vq) = self.positions(s);
        let fa = self.friction_at();
        let states = &s[fa..fa + n];
        let (friction, _) = self.friction(q, vq, states, torque);
        let (mut angle, mut speed) = (vec![0.0; m], vec![0.0; m]);
        self.motor_state(s, &mut angle, &mut speed);
        let (powers, gear_loss, brake) = self.losses(s, torque);
        // The axes' rows of the mass matrix (`elastic_mass`, with the sway and the lean) times the accelerations, and the
        // pulleys and rotors at the axes' (`drive_mass`).
        let (acc, moved) = if self.elastic {
            let mut d = vec![0.0; self.state_len()];
            self.elastic_derivatives(s, torque, &mut d);
            let mut all = d[self.vel()..self.vel() + n].to_vec();
            all.extend(self.frame.as_ref().map(|_| d[self.vel() + self.frame_at()]));
            all.extend(self.lean.as_ref().map(|_| d[self.vel() + self.lean_at()]));
            all.extend(self.beam.as_ref().map(|_| d[self.vel() + self.beam_at()]));
            let mass = self.elastic_mass(q);
            (all[..n].to_vec(), (0..n).map(|i| dot(&mass[i], &all)).collect::<Vec<f64>>())
        } else {
            let acc = self.rigid_solve(s, torque).acc;
            let (axis_mass, runs) = (self.axis_mass(q), self.runs_mass(q, [0.0; 3]).0);
            let moved = (0..n).map(|i| dot(&axis_mass[i], &acc) + dot(&runs[i], &acc)).collect();
            (acc, moved)
        };
        let (drive_mass, (chain, _)) = (self.drive_mass(), self.chain_inertia(vq, self.sway(s).1));
        let (inertial, runs) = (self.inertial(q, vq), self.runs_speed(vq, self.sway_direction(), self.sway(s).1).0);
        let inertia: Vec<f64> =
            (0..n).map(|i| moved[i] + dot(&drive_mass[i], &acc) - chain[i] - inertial[i] - runs[i]).collect();
        let gravity: Vec<f64> = self.gravity(q).iter().map(|g| -g).collect();
        let (idlers, drag) = (self.idler_friction(vq).0, self.chain_drag(vq));
        let bearing = self.bearing_force(vq);
        let resisting: Vec<f64> = (0..n).map(|i| friction[i] - idlers[i] + drag[i] + bearing[i]).collect();
        let [inertia, gravity, friction_at] = [inertia, gravity, resisting].map(|f| self.to_motors(&f));
        for kk in 0..m {
            let parts = [
                inertia[kk],
                gravity[kk],
                self.shafts[kk].torque(speed[kk]) + brake[kk] + gear_loss[kk] + friction_at[kk],
            ];
            out.extend(parts);
            out.push(torque[kk] - parts.iter().sum::<f64>());
        }
        out.extend(powers);
        out.extend(self.energy_parts(s));
        for (g, scale) in self.guides.iter().zip(self.load_scales(q, torque)) {
            out.push(vq[g.axis]);
            out.extend(g.friction.split(vq[g.axis], states[g.axis], scale));
            out.push(scale);
        }
        let geo = self.geo(q);
        let mat = if self.elastic { self.mat(s) } else { geo.iter().map(|g| self.belt.rest_length(*g)).collect() };
        out.extend(geo.iter().zip(&mat).map(|(g, l)| g - l));
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
        for g in &self.guides {
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}{}", g.loss_id),
                self.axes[g.axis].key,
                "quantity-friction-power",
                "W",
            ));
        }
        for motor in &self.motors {
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}shaft_{}", motor.id),
                motor.key,
                "quantity-shaft-friction-power",
                "W",
            ));
        }
        for (motor, coupled) in self.motors.iter().zip(&self.coupled) {
            let quantity = if *coupled { "quantity-coupling-loss-power" } else { "quantity-gearbox-loss-power" };
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}gearbox_{}", motor.id), motor.key, quantity, "W"));
        }
        if !self.idlers.is_empty() {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}idlers"), "group-idler", "quantity-friction-power", "W"));
        }
        if self.chain_axes.contains(&true) {
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}cable_chain"),
                "group-cable_chain",
                "quantity-friction-power",
                "W",
            ));
        }
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}belt"), "group-belt", "quantity-damping-power", "W"));
        if self.frame_node {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}frame"), "frame", "quantity-damping-power", "W"));
        }
        if self.lean_node.is_some() {
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}tool_flex"),
                "group-tool_flex",
                "quantity-damping-power",
                "W",
            ));
        }
        if let Some(g) = &self.gantry {
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}gantry_{}", g.id),
                "group-gantry_yaw",
                "quantity-damping-power",
                "W",
            ));
        }
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}hard_stops"), "group-hard_stop", "quantity-damping-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}other"), "loss-other", "quantity-power-given-off", "W"));
        if !self.jumps.is_empty() {
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}tooth_jump"),
                "group-belt",
                "quantity-tooth-jump-power",
                "W",
            ));
        }
        if self.beam.is_some() {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}beam"), "group-beam", "quantity-damping-power", "W"));
        }
        out.extend(ENERGY_PARTS.map(|part| {
            SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")
        }));
        for g in &self.guides {
            for part in FRICTION_SIGNALS {
                let unit = match part {
                    "speed" => "m/s",
                    "load_scale" => "",
                    _ => "N",
                };
                out.push(SignalInfo::new(
                    format!("{}.friction_{part}", g.contact),
                    self.axes[g.axis].key,
                    &format!("quantity-friction-{}", part.replace('_', "-")),
                    unit,
                ));
            }
        }
        out.extend(
            self.sections
                .iter()
                .map(|s| SignalInfo::new(format!("{}.stretch", s.id), s.key, "quantity-section-stretch", "m")),
        );
    }

    /// The first-mode frequency of each span (`span_frequency`) at its section's tension, and how far the nearest of
    /// them is from what the drives excite (`span_detuning`).
    fn span_signals(&self, s: &[f64], mass: f64, out: &mut Vec<f64>) {
        let (n, m) = (self.n(), self.m());
        let (q, tension) = (&s[..n], self.tensions(s));
        let (mut angle, mut speed) = (vec![0.0; m], vec![0.0; m]);
        self.motor_state(s, &mut angle, &mut speed);
        let mut excitations = Vec::with_capacity(3 * m);
        for kk in (0..m).filter(|&kk| self.direct[kk].is_none()) {
            let pulley = if self.elastic { s[self.vel() + n + kk] } else { speed[kk] / self.ratio[kk] };
            drive_excitations(self.mesh.pitch, self.r[kk] * pulley, &self.ripples[kk], speed[kk], &mut excitations);
        }
        let frequencies: Vec<f64> =
            self.spans.iter().map(|p| span_frequency(p.length + dot(&p.c, q), tension[p.section], mass)).collect();
        out.extend(&frequencies);
        out.push(frequencies.iter().map(|f| span_detuning(*f, &excitations)).fold(1.0, f64::min));
    }

    /// The sections' tensions at rest at q and their unstretched belt, L_s / (1 + T_s / EA): the tensions hold every axis
    /// against gravity, sum_s c_si T_s = gravity_i, and each closed belt keeps its material, sum over its sections of
    /// L_s / (1 + T_s / EA) = what it had at q = 0. Newton from the pretension; the first equations are linear, the
    /// others nearly so. The frame stands at 0: nothing pulls along its horizontal axis. An axis a screw, rack, linear
    /// motor or rotary drives is held by that, not by the belt. A yaw left to its spring is not among the equations: its
    /// balance sets the yaw (`settled_yaw`), which q brings.
    fn rest_tensions(&self, q: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let (n, geo, gravity) = (self.n(), self.geo(q), self.gravity(q));
        let (ea, slack) = (self.belt.ea, self.belt.slack);
        let taut = |t: f64| if slack { t.max(0.0) } else { t };
        let mut tension = vec![self.belt.pretension; self.sections.len()];
        let sprung = self.gantry.as_ref().filter(|_| self.yaw_sprung()).map(|g| g.coord);
        let belt_held: Vec<usize> = (0..n).filter(|&i| !self.driven(i) && Some(i) != sprung).collect();
        for _ in 0..50 {
            let mut jac: Vec<Vec<f64>> =
                belt_held.iter().map(|&i| self.sections.iter().map(|s| s.c[i]).collect()).collect();
            let mut res: Vec<f64> =
                belt_held.iter().zip(&jac).map(|(&i, row)| gravity[i] - dot(row, &tension)).collect();
            for belt in &self.loops {
                let mut row = vec![0.0; self.sections.len()];
                let mut held = 0.0;
                for &s in belt {
                    let stretch = 1.0 + taut(tension[s]) / ea;
                    held += geo[s] / stretch - self.sections[s].mat0;
                    row[s] = if slack && tension[s] < 0.0 { 0.0 } else { -geo[s] / (ea * stretch * stretch) };
                }
                jac.push(row);
                res.push(-held);
            }
            let step = lstsq(&jac, &res).unwrap_or_else(|| vec![0.0; self.sections.len()]);
            tension.iter_mut().zip(&step).for_each(|(t, d)| *t += d);
            if step.iter().all(|d| d.abs() < 1e-12 * (1.0 + self.belt.pretension)) {
                break;
            }
        }
        let mat = geo.iter().zip(&tension).map(|(g, t)| g / (1.0 + taut(*t) / ea)).collect();
        (tension, mat)
    }

    /// The force each screw, rack, linear motor or rotary passes to its axis at rest at q with the belts at `tension`
    /// (zero for the belt motors): the balance of the driven coordinates, the gearboxes' outputs against the belt's pull
    /// and gravity, in least squares where two drives share an axis. A gantry's yaw left to its spring is not among
    /// them: the spring holds it (`settled_yaw`).
    fn held_forces(&self, q: &[f64], tension: &[f64]) -> Vec<f64> {
        let taut = |t: f64| if self.belt.slack { t.max(0.0) } else { t };
        let gravity = self.gravity(q);
        let drives: Vec<usize> = (0..self.m()).filter(|&kk| self.direct[kk].is_some()).collect();
        let sprung = self.yaw_sprung();
        let rows: Vec<usize> = (0..self.n()).filter(|&i| self.driven(i) && !(sprung && i >= self.nt())).collect();
        let a: Vec<Vec<f64>> = rows.iter().map(|&i| drives.iter().map(|&kk| self.k[kk][i]).collect()).collect();
        let b: Vec<f64> = rows
            .iter()
            .map(|&i| self.sections.iter().zip(tension).map(|(s, t)| s.c[i] * taut(*t)).sum::<f64>() - gravity[i])
            .collect();
        let mut held = vec![0.0; self.m()];
        if !drives.is_empty() {
            let solved = lstsq(&a, &b).unwrap_or_else(|| vec![0.0; drives.len()]);
            drives.iter().zip(solved).for_each(|(&kk, o)| held[kk] = o);
        }
        held
    }

    /// A yawing gantry whose yaw the motors do not set (fewer motors than coordinates): its spring holds it.
    fn yaw_sprung(&self) -> bool {
        self.gantry.is_some() && self.m() < self.n()
    }

    /// The yaw's resting deflection, which a yaw left to its spring takes: the sum of the forces on the coordinate
    /// vanishes, sum_drives side F_d - k psi = sum_s c_s,psi T_s - G_psi (the drives' moment, the spring's, the belts'
    /// pull and gravity's), the tensions at that yaw (`rest_tensions`: the belts keep their length with it) and the
    /// drives' forces from the axes' balance. Solved for psi by the secant method from psi = f(0) / k, 0 if it does not
    /// converge.
    fn settled_yaw(&self, q: &[f64]) -> f64 {
        let Some(g) = &self.gantry else { return 0.0 };
        let residual = |psi: f64| -> f64 {
            let mut at = q.to_vec();
            at[g.coord] = psi;
            if !self.elastic {
                // The rigid model: the motors' torques at the pulleys carry the other coordinates against gravity, K^T P =
                // -gravity there (least squares, the yaw's row left out), and what P then puts on the yaw less the spring's
                // and gravity's leaves the residual, as the belts' pull does above.
                let gravity = self.gravity(&at);
                let rows: Vec<usize> = (0..self.n()).filter(|&i| i != g.coord).collect();
                let kt: Vec<Vec<f64>> = rows.iter().map(|&i| (0..self.m()).map(|kk| self.k[kk][i]).collect()).collect();
                let pulley = lstsq(&kt, &rows.iter().map(|&i| -gravity[i]).collect::<Vec<_>>())
                    .unwrap_or_else(|| vec![0.0; self.m()]);
                let moment: f64 = (0..self.m()).map(|kk| self.k[kk][g.coord] * pulley[kk]).sum();
                return moment - g.stiffness * psi + gravity[g.coord];
            }
            let (tension, _) = self.rest_tensions(&at);
            let held = self.held_forces(&at, &tension);
            let taut = |t: f64| if self.belt.slack { t.max(0.0) } else { t };
            let moment: f64 =
                (0..self.m()).filter(|&kk| self.direct[kk].is_some()).map(|kk| self.k[kk][g.coord] * held[kk]).sum();
            let pull: f64 = self.sections.iter().zip(&tension).map(|(s, t)| s.c[g.coord] * taut(*t)).sum();
            moment - g.stiffness * psi - (pull - self.gravity(&at)[g.coord])
        };
        let (mut x0, mut f0) = (0.0, residual(0.0));
        if f0 == 0.0 || g.stiffness <= 0.0 {
            return 0.0;
        }
        let mut x1 = f0 / g.stiffness;
        for _ in 0..40 {
            let f1 = residual(x1);
            if !f1.is_finite() || f1 == f0 {
                return if f1.is_finite() { x1 } else { 0.0 };
            }
            let x2 = x1 - f1 * (x1 - x0) / (f1 - f0);
            if (x2 - x1).abs() <= 1e-14 + 1e-12 * x1.abs() {
                return x2;
            }
            (x0, f0, x1) = (x1, f1, x2);
        }
        0.0
    }

    /// The lowest belt mode at a pose (elastic model): the axes (and the frame, where it sways, and the leaning body,
    /// where it leans) on the belt's stiffness with the pulleys held, K = sum_s EA / l_s c_s c_s^T (l_s the section's
    /// unstretched belt at rest; plus the frame's and
    /// the lean's springs), against their mass matrix (see `energy_parts`): (w^2, the axes', frame's and lean's motion).
    fn belt_mode(&self, pose: &[f64]) -> Option<(f64, Vec<f64>)> {
        if !self.elastic {
            return None;
        }
        // dT/dL = EA / l of `Belt::section`, l the section's unstretched belt at rest at the pose (`rest_tensions`), as
        // the crates take it.
        let (n, rest) = (self.n(), self.rest_tensions(pose).1);
        let mass = self.elastic_mass(pose);
        let d = mass.len();
        let mut stiffness = vec![vec![0.0; d]; d];
        for i in 0..n {
            for (sec, length) in self.sections.iter().zip(&rest) {
                for l in 0..n {
                    stiffness[i][l] += self.belt.ea / length * sec.c[i] * sec.c[l];
                }
            }
        }
        // A screw's, rack's, linear motor's or rotary's gearbox (with its converter) holds its axis on the rotor.
        for (kk, d) in self.direct.iter().enumerate() {
            if d.is_some() {
                let k = &self.k[kk];
                for i in 0..n {
                    for l in 0..n {
                        stiffness[i][l] += self.gears[kk].stiffness * k[i] * k[l];
                    }
                }
            }
        }
        if let Some(g) = &self.gantry {
            stiffness[g.coord][g.coord] += g.stiffness;
        }
        if let Some(f) = &self.frame {
            stiffness[n][n] = f.stiffness;
        }
        if let Some(lean) = &self.lean {
            let (k, dk, tip) = flex_at(lean, pose);
            let (c, _) = lean_weight(lean, pose, self.body_mass(&self.bodies[lean.body]), self.g);
            let lr = self.lean_row();
            stiffness[lr][lr] = k + c;
            // The spring stiffens with the lever axis (k(z)), so at the lean u0 = tip F / k that the pose's sideways
            // process force F holds, K_uz = k' u0 and K_zz = k'' u0^2 / 2 (the crate's `modal`).
            if let Some(z) = lean.lever_axis {
                let u0 = tip * self.process_force()[lean.axis] / k;
                let step = 1e-6;
                let rate = |h: f64| {
                    let mut at = pose.to_vec();
                    at[z] += h;
                    flex_at(lean, &at).1
                };
                let ddk = (rate(step) - rate(-step)) / (2.0 * step);
                stiffness[lr][z] += dk * u0;
                stiffness[z][lr] += dk * u0;
                stiffness[z][z] += 0.5 * ddk * u0 * u0;
                // The belt runs' geometric stiffness, K_uu += sum T_i c_i (`lean_runs`, T_i the tensions at rest), and the
                // process force's vertical part on the tip's rise a u^2 (V = -F_z a u^2, `rise_coefficients`):
                // K_uu += -2 F_z a, K_uz += -2 F_z a' u0, K_zz += -F_z a'' u0^2.
                let (tension, runs) = (self.rest_tensions(pose).0, self.lean_runs(pose));
                stiffness[lr][lr] += tension.iter().zip(&runs).map(|(t, c)| t * c).sum::<f64>();
                let [a, da, dda] = rise_coefficients(lean, pose);
                let process_z = self.process_force()[z];
                stiffness[lr][lr] -= 2.0 * process_z * a;
                stiffness[lr][z] -= 2.0 * process_z * da * u0;
                stiffness[z][lr] -= 2.0 * process_z * da * u0;
                stiffness[z][z] -= process_z * dda * u0 * u0;
            }
        }
        // The beam's deflection w: V = 1/2 k(x) w^2 - m g w, so at the pose's w0 = m g / k: K_ww = k, K_xw = k' w0 and
        // K_xx += k'' w0^2 / 2 (k'' by a central difference over 1e-4 m, as the crate takes it).
        if let Some(b) = &self.beam {
            let (at, x) = (self.beam_row(), b.axis);
            let [k, _, _, dk, _] = self.beam_mode(pose);
            let w0 = self.beam_load().0 * self.g / k;
            let slope = |h: f64| {
                let mut moved = pose.to_vec();
                moved[x] += h;
                self.beam_mode(&moved)[3]
            };
            let ddk = (slope(1e-4) - slope(-1e-4)) / 2e-4;
            stiffness[at][at] += k;
            stiffness[at][x] += dk * w0;
            stiffness[x][at] += dk * w0;
            stiffness[x][x] += 0.5 * ddk * w0 * w0;
        }
        lowest_mode(&mass, &stiffness)
    }

    /// Where the scene draws a pose: each axis runs from the origin along its incline in the x-z plane.
    fn scene_at(&self, q: &[f64], moves: &[bool]) -> [f64; 3] {
        let mut p = scene_origin();
        // Up by the incline, and across: the first axis along x, the others along y (`dirs`). A rotary axis turns
        // its table in place.
        for i in (0..self.n()).filter(|i| moves[*i]) {
            (0..3).for_each(|k| p[k] += self.dirs[i][k] * q[i]);
        }
        p
    }
    /// Where the scene draws the pulley of motor `kk`: its `at_m`, else at the ends of the first axis, two by two
    /// further down.
    fn pulley_scene_at(&self, kk: usize) -> [f64; 3] {
        let end = self.stroke[0];
        self.pulley_at[kk].unwrap_or([
            if kk.is_multiple_of(2) { -0.1 } else { end + 0.1 },
            -0.05,
            -0.15 * (kk / 2) as f64,
        ])
    }
    /// A screw, rack or magnet track along its axis from 0 to its stroke, carried by what carries the axis (the first
    /// body it moves, without its own travel), with the motor at the 0 end; a rotary as a disc where the motor's pulley
    /// would be, its mark turned with the axis. Where the motor's label goes.
    fn direct_scene(&self, kk: usize, d: &Direct, q: &[f64], out: &mut Scene) -> [f64; 3] {
        let motor = [0.9, 0.45, 0.1];
        let id = match d.kind {
            DirectKind::Screw(_) => "screw",
            DirectKind::Rack(_) => "rack",
            DirectKind::Linear { .. } => "magnet-track",
            DirectKind::Rotary(_) => {
                // At the bodies that ride on it, where there are some.
                let at = self.bodies.iter().find(|b| b.rotaries.contains(&d.axis)).map_or_else(
                    || self.pulley_scene_at(kk),
                    |b| {
                        let carried: Vec<bool> = (0..self.n()).map(|i| b.axes[i] && i != d.axis).collect();
                        self.scene_at(q, &carried)
                    },
                );
                out.parts.push(Part::new(
                    "table",
                    Shape::Cylinder { radius: 0.15, length: 0.04 },
                    at,
                    [0.55, 0.55, 0.6],
                ));
                out.parts.push(
                    Part::new(
                        "index-mark",
                        Shape::Box { size: [0.27, 0.004, 0.02] },
                        [at[0], at[1] - 0.025, at[2]],
                        [0.1, 0.1, 0.1],
                    )
                    .turned(q[d.axis]),
                );
                out.parts.push(Part::new(
                    MOTOR_PART,
                    Shape::Cylinder { radius: 0.04, length: 0.1 },
                    [at[0], at[1], at[2] - 0.1],
                    motor,
                ));
                return at;
            }
        };
        let moves =
            self.bodies.iter().find(|b| b.axes[d.axis]).map_or_else(|| vec![false; self.n()], |b| b.axes.clone());
        let mut pose = q.to_vec();
        pose[d.axis] = 0.0;
        // At a yawing gantry's side: across the bridge from where its across axis puts the carriage at 0.
        let side = |p: [f64; 3]| match self.gantry.as_ref().and_then(|g| g.across.map(|a| (a, d.side - g.shift))) {
            Some((a, across)) if d.side != 0.0 => [0, 1, 2].map(|k| p[k] + self.dirs[a][k] * across),
            _ => p,
        };
        let start = side(self.scene_at(&pose, &moves));
        pose[d.axis] = self.stroke[d.axis];
        let end = side(self.scene_at(&pose, &moves));
        out.parts.push(Part::new(id, Shape::Segment { to: end, width: 0.02 }, start, [0.75, 0.75, 0.78]));
        // A linear motor's forcer rides on the carriage.
        let motor_at = if matches!(d.kind, DirectKind::Linear { .. }) { side(self.scene_at(q, &moves)) } else { start };
        out.parts.push(Part::new(MOTOR_PART, Shape::Cylinder { radius: 0.04, length: 0.1 }, motor_at, motor));
        motor_at
    }
    fn idler_scene_at(&self, d: &Idler, q: &[f64]) -> [f64; 3] {
        let at = d.at.unwrap_or_default();
        match d.body {
            Some(b) => {
                let body = self.scene_at(q, &self.bodies[b].axes);
                [body[0] + at[0], body[1] + at[1], body[2] + at[2]]
            }
            None => at,
        }
    }
    /// The belt's plane in the 3D view, as (its second coordinate, the depth): x-z seen from the front, or x-y seen from
    /// above where the belt lies flat, a section's length changing with axes along x and along y and none along z (an
    /// H-bot's, a CoreXY's).
    fn belt_plane(&self) -> (usize, usize) {
        let flat = self.sections.iter().any(|s| {
            let along = |k: usize| (0..self.n()).any(|i| s.c[i] != 0.0 && self.dirs[i][k].abs() > 1e-9);
            along(0) && along(1) && !along(2)
        });
        if flat { (1, 2) } else { (2, 1) }
    }
    /// A point of the belt's plane (`belt_plane`) at the depth `depth`, in the 3D view.
    fn in_view(&self, p: [f64; 2], depth: f64) -> [f64; 3] {
        let (second, deep) = self.belt_plane();
        let mut out = [p[0]; 3];
        (out[second], out[deep]) = (p[1], depth);
        out
    }
    /// A waypoint of a belt as a wheel in the belt's plane (`belt_plane`), its radius signed by the sense the belt runs
    /// round it, and where the 3D view puts it at the pose `q`.
    fn wheel(&self, w: Waypoint, q: &[f64]) -> (Wheel, [f64; 3]) {
        let sense = |clockwise: bool| if clockwise { -1.0 } else { 1.0 };
        let (at, radius) = match w {
            Waypoint::Pulley(kk) => (self.pulley_scene_at(kk), self.r[kk] * sense(self.clockwise.0[kk])),
            Waypoint::Idler(d) => {
                let idler = &self.idlers[d];
                (self.idler_scene_at(idler, q), idler.radius * sense(self.clockwise.1[d]))
            }
            Waypoint::Body(b) => (self.scene_at(q, &self.bodies[b].axes), 0.0),
        };
        (Wheel { centre: [at[0], at[self.belt_plane().0]], radius }, at)
    }

    /// Where each span (a straight run of the belt) lies at the pose `q`: the middle of the straight piece between the
    /// tangent points on the two wheels of its section's route (`contacts`), in the frame of the bodies' places (the 3D
    /// view's, less where `scene_at` starts). The section's spans are taken in order as the route's consecutive pairs of
    /// waypoints; `None` for a section whose span count is not the route's pairs, and for a pair with no straight piece.
    fn run_midpoints(&self, q: &[f64]) -> Vec<Option<[f64; 3]>> {
        let mut next = vec![0usize; self.sections.len()];
        let routes: Vec<Vec<Waypoint>> = (0..self.sections.len()).map(|s| self.route_waypoints(s)).collect();
        let mut count = vec![0usize; self.sections.len()];
        self.spans.iter().for_each(|span| count[span.section] += 1);
        let deep = self.belt_plane().1;
        self.spans
            .iter()
            .map(|span| {
                let (route, at) = (&routes[span.section], next[span.section]);
                next[span.section] += 1;
                if count[span.section] + 1 != route.len() {
                    return None;
                }
                let ((a, pa), (b, pb)) = (self.wheel(route[at], q), self.wheel(route[at + 1], q));
                let [from, to] = contacts(a, b);
                let middle =
                    self.in_view([0.5 * (from[0] + to[0]), 0.5 * (from[1] + to[1])], 0.5 * (pa[deep] + pb[deep]));
                let origin = scene_origin();
                let place: [f64; 3] = std::array::from_fn(|k| middle[k] - origin[k]);
                place.iter().all(|x| x.is_finite()).then_some(place)
            })
            .collect()
    }

    /// The belt of section `s` as the 3D view draws it, a polyline in scene coordinates: its straight runs from tangent
    /// to tangent between the wheels of its route (`geometry::contacts`), round each wheel inside the route in the
    /// wheel's sense (`geometry::arc`). Each point keeps the depth of its wheel off the belt's plane.
    fn belt_line(&self, s: usize, q: &[f64]) -> Vec<[f64; 3]> {
        let route = self.route_waypoints(s);
        let wheels: Vec<(Wheel, [f64; 3])> = route.iter().map(|&w| self.wheel(w, q)).collect();
        let deep = self.belt_plane().1;
        let mut line = Vec::new();
        for (i, pair) in wheels.windows(2).enumerate() {
            let [(a, pa), (b, pb)] = [pair[0], pair[1]];
            let [leave, meet] = contacts(a, b);
            if i > 0 {
                // Round wheel a from where the run before met it.
                let came = contacts(wheels[i - 1].0, a)[1];
                line.extend(arc(a, came, leave, ARC_PIECES).into_iter().skip(1).map(|p| self.in_view(p, pa[deep])));
            } else {
                line.push(self.in_view(leave, pa[deep]));
            }
            line.push(self.in_view(meet, pb[deep]));
        }
        line
    }

    /// The wraps round the belt motors' pulleys between the section that ends at each and the one that starts there
    /// (`wrap_neighbours`' rule), as polylines with the section that ends there; none where a pulley does not end one
    /// route and start another.
    fn pulley_wraps(&self, q: &[f64]) -> Vec<(usize, Vec<[f64; 3]>)> {
        let routes: Vec<Vec<Waypoint>> = (0..self.sections.len()).map(|s| self.route_waypoints(s)).collect();
        (0..self.m())
            .filter(|&kk| self.direct[kk].is_none())
            .filter_map(|kk| {
                let me = Waypoint::Pulley(kk);
                let ends: Vec<usize> = (0..routes.len()).filter(|&i| routes[i].last() == Some(&me)).collect();
                let starts: Vec<usize> = (0..routes.len()).filter(|&i| routes[i].first() == Some(&me)).collect();
                let (&[section], &[next]) = (&ends[..], &starts[..]) else { return None };
                let (before, after) = (*routes[section].iter().rev().nth(1)?, *routes[next].get(1)?);
                let ((wheel, at), from, to) = (self.wheel(me, q), self.wheel(before, q).0, self.wheel(after, q).0);
                let points = arc(wheel, contacts(from, wheel)[1], contacts(wheel, to)[0], ARC_PIECES);
                let deep = self.belt_plane().1;
                Some((section, points.into_iter().map(|p| self.in_view(p, at[deep])).collect()))
            })
            .collect()
    }

    /// The axis a wheel spins about per rad/s of the belt running along its section's route: the normal of the belt's
    /// plane (`belt_plane`), pointing as a clockwise turn seen from the front (the viewer at -y) or, for a flat belt, from
    /// above does, reversed for a wheel the belt runs round counter-clockwise.
    fn wheel_axis(&self, clockwise: bool) -> [f64; 3] {
        let cw = if self.belt_plane().1 == 2 { [0.0, 0.0, -1.0] } else { [0.0, 1.0, 0.0] };
        cw.map(|c| if clockwise { c } else { -c })
    }
    /// +1 where a positive turn of pulley kk, or a positive speed past an idler of section `sec` (fed by that pulley),
    /// moves the belt along the section's route, -1 against it: the belt goes from the pulley into a section it feeds
    /// (`f` = +1), so along the route where the route starts at the pulley and against it where it ends there; reversed
    /// for a section it takes from (`f` = -1).
    fn route_sense(&self, sec: usize, kk: usize) -> f64 {
        let f = self.sections[sec].f[kk];
        if self.route_waypoints(sec).last() == Some(&Waypoint::Pulley(kk)) { -f } else { f }
    }

    /// The waypoints a section's belt runs through: its `route`, else its pulleys and the body it is clamped to (the
    /// one that moves with most of the axes its length changes with, and the fewest others).
    fn route_waypoints(&self, s: usize) -> Vec<Waypoint> {
        let sec = &self.sections[s];
        let mut route = sec.route.clone();
        if route.is_empty() {
            route.extend((0..self.m()).filter(|&kk| sec.f[kk] != 0.0).map(Waypoint::Pulley));
            let fits = |b: &Body| {
                let shared = (0..self.n()).filter(|&i| sec.c[i] != 0.0 && b.axes[i]).count() as i64;
                (shared, -((0..self.n()).filter(|&i| sec.c[i] == 0.0 && b.axes[i]).count() as i64))
            };
            if let Some(b) =
                (0..self.bodies.len()).filter(|&b| fits(&self.bodies[b]).0 > 0).max_by_key(|&b| fits(&self.bodies[b]))
            {
                route.insert(
                    if sec.f.iter().position(|f| *f != 0.0).is_some_and(|kk| sec.f[kk] < 0.0) {
                        0
                    } else {
                        route.len()
                    },
                    Waypoint::Body(b),
                );
            }
        }
        route
    }
}

impl ConfigMachine {
    /// The racking moment of a yawing gantry (effects.gantry_yaw) at a tool pose and acceleration, N m: what the belts'
    /// pull on the bridge's ends (`rigid_tensions`), its drives at its sides and gravity put on its yaw beyond what
    /// turns the bodies on it with the bridge held square, in the loss-free rigid model. The yaw's spring takes it while
    /// the acceleration lasts (signal gantry.<id>.moment). Zero where two drives set the yaw; `None` without a yawing
    /// gantry.
    pub fn racking_moment(&self, pos: &[f64], acc: &[f64]) -> Option<f64> {
        let g = self.gantry.as_ref()?;
        let (q, _, ddq) = self.joint_motion(pos, &vec![0.0; pos.len()], acc);
        let mut tensions = Vec::new();
        self.rigid_tensions(pos, acc, &mut tensions);
        let (mass, gravity) = (self.axis_mass(&q), self.gravity(&q));
        let wanted: Vec<f64> = (0..self.n())
            .map(|i| {
                let belt: f64 = self.sections.iter().zip(&tensions).map(|(s, t)| s.c[i] * t).sum();
                dot(&mass[i], &ddq) - gravity[i] + belt
            })
            .collect();
        // The drives' forces: on the coordinates they set (the yaw too where two of them do).
        let drives: Vec<usize> = (0..self.m()).filter(|&kk| self.direct[kk].is_some()).collect();
        let rows: Vec<usize> =
            (0..self.n()).filter(|&i| self.driven(i) && (i < self.nt() || self.m() >= self.n())).collect();
        let a: Vec<Vec<f64>> = rows.iter().map(|&i| drives.iter().map(|&kk| self.k[kk][i]).collect()).collect();
        let b: Vec<f64> = rows.iter().map(|&i| wanted[i]).collect();
        let forces = if drives.is_empty() { Vec::new() } else { lstsq(&a, &b)? };
        let driving: f64 = drives.iter().zip(&forces).map(|(&kk, f)| self.k[kk][g.coord] * f).sum();
        Some(driving - wanted[g.coord])
    }

    /// `belt_wraps` at the coordinates q.
    fn wraps_at(&self, q: &[f64]) -> Vec<f64> {
        let wheel = |w: Waypoint| self.wheel(w, q).0;
        let mut out = vec![f64::NAN; self.m()];
        for &(kk, before, after) in &self.wraps {
            out[kk] = wrap(wheel(before), wheel(Waypoint::Pulley(kk)), wheel(after));
        }
        out
    }
}

impl Machine for ConfigMachine {
    /// Each body moving with its axes (an axis with the id x, y or z along that direction), over ground with the frame's
    /// sway and the lean; the frame's own mass in its sway; a rotary axis's inertia turning about its own axis; the process
    /// force on the tool, each along its axis's direction (`dirs`, whatever the axis's id). The belt drives' spin: a
    /// pulley, its rotor and the idlers turn about the normal of the belt's plane (`belt_plane`), a screw's rotor about
    /// its axis, I w each. The sense of a wheel: the belt along its section's route turns it as its `clockwise` says (seen
    /// from the front, from above for a flat belt), and a positive pulley turn and an idler's positive `speed` move the
    /// belt as the first pulley of the section feeds it (`route_sense`). A rack's pinion turns about e x up (e the rack's
    /// direction, the pinion above it; a vertical rack's e x y), a rotary's motor about the rotary's own axis, positive
    /// as the axis grows. The belt's runs carry angular momentum m x (mu L y) too, each about the middle of its straight
    /// piece (`run_midpoints`); the arcs round the wheels are in the wheels' spin. What does not move is left out (see
    /// `FrameMomentum`).
    fn frame_momentum(&self, s: &[f64]) -> Option<sim_core::FrameMomentum> {
        // The axes, not the coordinates: a yawing gantry's yaw is one more, which no tool axis has.
        let n = self.axes.len();
        let g = self.g_along.first().zip(self.g_across.first()).map_or(0.0, |(a, b)| a.hypot(*b));
        let mut f = sim_core::FrameMomentum { gravity: [0.0, 0.0, -g], ..Default::default() };
        let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
        self.tool_state(s, &mut pos, &mut vel);
        let (q, v) = self.positions(s);
        let (mut ground, mut place) = (v.to_vec(), q.to_vec());
        if let Some(fr) = &self.frame {
            let (u, du) = self.sway(s);
            ground[fr.axis] += du;
            place[fr.axis] += u;
            let e = self.sway_direction();
            for k in 0..3 {
                f.linear[k] += fr.mass * du * e[k];
                f.first_moment[k] += fr.mass * u * e[k];
            }
        }
        for (b, body) in self.bodies.iter().enumerate() {
            // Where the body is (straight axes, its offset turned by the rotaries it rides on) and how fast, by the
            // Jacobian of that place: a rotary's lever arm and omega x r come with it.
            let mut r = self.body_point(body, &place);
            let mut w = [0.0; 3];
            for (i, column) in self.body_jacobian(body, &place).iter().enumerate() {
                (0..3).for_each(|k| w[k] += column[k] * ground[i]);
            }
            // A body that does not move along the frame's axis rides on the frame.
            if self.frame.as_ref().is_some_and(|fr| !body.axes[fr.axis]) {
                let (e, (u, du)) = (self.sway_direction(), self.sway(s));
                (0..3).for_each(|k| (r[k], w[k]) = (r[k] + u * e[k], w[k] + du * e[k]));
            }
            if let Some(lean) = self.lean.as_ref().filter(|l| l.body == b) {
                let (e, (x, dx)) = (self.dirs[lean.axis], self.leaning(s));
                (0..3).for_each(|k| (r[k], w[k]) = (r[k] + x * e[k], w[k] + dx * e[k]));
            }
            // The bodies that move along the beam hang w lower with it.
            if let Some(beam) = self.beam.as_ref().filter(|beam| body.axes[beam.axis]) {
                let (e, (x, dx)) = (self.dirs[beam.vertical], self.deflection(s));
                (0..3).for_each(|k| (r[k], w[k]) = (r[k] - x * e[k], w[k] - dx * e[k]));
            }
            sim_components::base::add_point(&mut f, self.body_mass(body), r, w);
        }
        // Each rotary's own spin, about its axis as the rotaries outside it have turned it.
        let rotary_axis = |i: usize| {
            let outer =
                self.bodies.iter().find_map(|b| b.rotaries.iter().position(|&r| r == i).map(|at| &b.rotaries[..at]));
            self.turned_by(outer.unwrap_or(&[]), q, self.turn[i])
        };
        for i in (0..n).filter(|&i| self.axes[i].unit == "rad") {
            let turned = self.turned[i].0 + if self.payload_attached { self.turned[i].1 } else { 0.0 };
            let axis = rotary_axis(i);
            (0..3).for_each(|k| f.angular[k] += turned * v[i] * axis[k]);
        }
        // The belt drives' spin (see above): pulleys, rotors and idlers.
        let (mut turn, mut rotor_speed) = (vec![0.0; self.m()], vec![0.0; self.m()]);
        self.motor_state(s, &mut turn, &mut rotor_speed);
        let (nn, vv) = (self.n(), self.vel());
        for kk in 0..self.m() {
            let mut ax =
                |axis: [f64; 3], w: f64, inertia: f64| (0..3).for_each(|k| f.angular[k] += inertia * w * axis[k]);
            match (&self.direct[kk], self.sections.iter().position(|sec| sec.f[kk] != 0.0)) {
                (None, Some(sec)) => {
                    let w_pulley = if self.elastic { s[vv + nn + kk] } else { self.pulley_angle(kk, v) };
                    let sense = self.route_sense(sec, kk);
                    let (cw, r) = (self.clockwise.0[kk], self.r[kk]);
                    let spin = self.wheel_axis(cw);
                    ax(spin, sense * w_pulley, (self.j_pulley[kk] - self.lump * r * r).max(0.0));
                    ax(spin, sense * rotor_speed[kk], self.j_rotor[kk]);
                }
                (Some(Direct { kind: DirectKind::Screw(_), axis, .. }), _) => {
                    ax(self.dirs[*axis], rotor_speed[kk], self.j_rotor[kk]);
                }
                // A rack's pinion turns about the horizontal across its rack (the pinion above the rack: a positive turn
                // moves the rack along its direction e at the contact below, so the axis is e x up; a vertical rack's is
                // e x y). A rotary's rotor turns about the rotary's own axis, as the rotaries outside it have turned it.
                (Some(Direct { kind: DirectKind::Rack(_), axis, .. }), _) => {
                    let e = self.dirs[*axis];
                    let across = if e[2].abs() > 0.999 { [0.0, 1.0, 0.0] } else { [0.0, 0.0, 1.0] };
                    let a = cross(e, across);
                    let size = dot(&a, &a).sqrt();
                    ax(a.map(|a| a / size), rotor_speed[kk], self.j_rotor[kk]);
                }
                (Some(Direct { kind: DirectKind::Rotary(_), axis, .. }), _) => {
                    ax(rotary_axis(*axis), rotor_speed[kk], self.j_rotor[kk]);
                }
                _ => {}
            }
        }
        for (d, idler) in self.idlers.iter().enumerate() {
            let Some(sec) =
                (0..self.sections.len()).find(|&sec| self.route_waypoints(sec).contains(&Waypoint::Idler(d)))
            else {
                continue;
            };
            let Some(first) = (0..self.m()).find(|&kk| self.sections[sec].f[kk] != 0.0) else { continue };
            let w = self.route_sense(sec, first) * dot(&idler.speed, v) / idler.radius;
            let spin = self.wheel_axis(self.clockwise.1[d]);
            (0..3).for_each(|k| f.angular[k] += idler.mass * idler.radius * idler.radius * w * spin[k]);
        }
        // The belt's runs (`runs_mass`), each at V_s v over ground and the sway on top: momentum mu sum L_s y_s, and the
        // first moment mu sum (L0_s + c_s . q / 2) V_s q, whose rate that is (the belt's runs being one piece), with
        // mu L e u of the sway.
        // The runs' angular momentum, m_s x (mu L_s y_s) about the origin of the bodies' places, from where each run lies
        // (`run_midpoints`, the middle of its straight piece, the sway on top).
        if self.run_mass > 0.0 {
            let (e, (u, du)) = (self.sway_direction(), self.sway(s));
            let middles = self.run_midpoints(q);
            for (span, middle) in self.spans.iter().zip(middles) {
                let Some(rows) = &span.velocity else { continue };
                let weight = self.run_mass * (span.length + dot(&span.c, q));
                let moment = self.run_mass * (span.length + 0.5 * dot(&span.c, q));
                let y: [f64; 3] = std::array::from_fn(|k| dot(&rows[k], v) + e[k] * du);
                for k in 0..3 {
                    f.linear[k] += weight * y[k];
                    f.first_moment[k] += moment * dot(&rows[k], q) + weight * e[k] * u;
                }
                f.mass += weight;
                if let Some(m) = middle {
                    let at = std::array::from_fn(|k| m[k] + e[k] * u);
                    let turning = cross(at, y);
                    (0..3).for_each(|k| f.angular[k] += weight * turning[k]);
                }
            }
        }
        // The process force acts on the tool along the axes' directions, at the tool's place over ground.
        for (i, force) in self.process_force().iter().enumerate().take(n) {
            (0..3).for_each(|k| f.external[k] += *force * self.dirs[i][k]);
        }
        for i in (0..n).filter(|&i| self.axes[i].unit != "rad") {
            (0..3).for_each(|k| f.external_at[k] += pos[i] * self.dirs[i][k]);
        }
        Some(f)
    }
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }
    fn motor_linear(&self, motor: usize) -> bool {
        matches!(self.direct.get(motor), Some(Some(Direct { kind: DirectKind::Linear { .. }, .. })))
    }

    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        self.motor_angles(&self.joints(tool), motors);
    }
    /// The motors set the coordinates they can (`k_inv`); a gantry's yaw they leave to its spring stands at 0.
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        let q: Vec<f64> = (0..self.n())
            .map(|i| (0..self.m()).map(|kk| self.k_inv[i][kk] * motors[kk] / self.ratio[kk]).sum())
            .collect();
        tool.copy_from_slice(&self.tool_of(&q));
    }

    fn state_len(&self) -> usize {
        self.brake_at() + self.m() + JUMP_STATES * self.jumps.len()
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let (n, m) = (self.n(), self.m());
        *state = vec![0.0; self.state_len()];
        let mut q = self.joints(tool);
        // Where the motors leave a gantry's yaw to its spring, the drives' moment about it deflects the spring.
        if self.yaw_sprung() {
            q[self.gantry.as_ref().map_or(0, |g| g.coord)] = self.settled_yaw(&q);
        }
        // With the beam mode the Z axis stands on its belt above the beam's sag, which the beam rests at.
        let sag = self.sag_at_rest(&mut q);
        let tool = &q[..];
        state[..n].copy_from_slice(tool);
        if self.beam.is_some() {
            state[self.beam_at()] = sag;
        }
        let gravity = self.gravity(tool);
        if !self.elastic {
            // K^T (pulley torque + r mesh force) = -gravity.
            let kt: Vec<Vec<f64>> = (0..n).map(|i| (0..m).map(|kk| self.k[kk][i]).collect()).collect();
            // With the yaw on its spring (`settled_yaw`) the spring's force -k psi is among the forces to hold.
            let spring = self.yaw_force(tool, &vec![0.0; n]);
            let at_pulley = lstsq(&kt, &gravity.iter().zip(&spring).map(|(g, y)| -g - y).collect::<Vec<_>>())
                .unwrap_or_else(|| vec![0.0; m]);
            for kk in 0..m {
                let angle = self.ratio[kk] * self.pulley_angle(kk, tool);
                let shaft = (at_pulley[kk] - self.r[kk] * self.mesh_force(kk, self.r[kk] * angle / self.ratio[kk]))
                    / self.ratio[kk];
                torque[kk] = self.ripples[kk].invert(shaft * self.held(kk), angle);
            }
            return;
        }
        let geo = self.geo(tool);
        let (tension, mat) = self.rest_tensions(tool);
        // The belt motors' pulley angles that feed that material (a motor without a pulley feeds none).
        let belted: Vec<usize> = (0..m).filter(|&kk| self.direct[kk].is_none()).collect();
        let fed: Vec<Vec<f64>> =
            self.sections.iter().map(|s| belted.iter().map(|&kk| s.f[kk] * self.r_mat[kk]).collect()).collect();
        let change: Vec<f64> = self.sections.iter().zip(&mat).map(|(s, l)| l - s.mat0).collect();
        let mut angles = vec![0.0; m];
        for (&kk, a) in belted.iter().zip(lstsq(&fed, &change).unwrap_or_else(|| vec![0.0; belted.len()])) {
            angles[kk] = a;
        }
        let material: Vec<f64> =
            geo.iter().zip(&mat).map(|(g, l)| self.belt.section(*g, *l, 0.0, 0.0).material).collect();
        // The force balance of the coordinates the screws, racks, linear motors and rotaries drive: their gearboxes'
        // outputs, the belt's pull and gravity (two drives at a gantry's sides share it by their places).
        let held = self.held_forces(tool, &tension);
        for kk in 0..m {
            let (theta, gear) = match &self.direct[kk] {
                Some(_) => (self.pulley_angle(kk, tool), held[kk]),
                None => {
                    let belt: f64 =
                        self.r_mat[kk] * self.sections.iter().zip(&material).map(|(s, f)| s.f[kk] * f).sum::<f64>();
                    (angles[kk], -belt - self.r[kk] * self.mesh.force(self.r[kk] * angles[kk]))
                }
            };
            let phi = self.ratio[kk] * (theta + self.gears[kk].static_twist(gear));
            state[n + kk] = theta;
            state[n + m + kk] = phi;
            torque[kk] = self.ripples[kk].invert(gear * self.held(kk) / self.ratio[kk], phi);
        }
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        if self.elastic {
            return self.elastic_derivatives(s, torque, out);
        }
        let (n, m) = (self.n(), self.m());
        let solved = self.rigid_solve(s, torque);
        out[..n].copy_from_slice(&s[n..2 * n]);
        out[n..2 * n].copy_from_slice(&solved.acc);
        out[2 * n..3 * n].copy_from_slice(&solved.rates);
        for kk in 0..m {
            let speed = self.ratio[kk] * self.pulley_angle(kk, &s[n..2 * n]);
            out[3 * n + kk] = self.brakes[kk].torque(self.brake_engaged, speed, s[3 * n + kk]).1;
        }
    }

    /// The tool is where the axes put it on the frame, and the frame sways: the workpiece stands on the floor.
    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (q, v) = self.positions(s);
        pos.copy_from_slice(&self.tool_of(q));
        vel.copy_from_slice(&self.tool_speed(q, v));
        if let Some(f) = &self.frame {
            let (u, du) = self.sway(s);
            pos[f.axis] += u;
            vel[f.axis] += du;
        }
        // The tool sits at the tip of the leaning body.
        if let Some(lean) = &self.lean {
            let ((w, dw), tip) = (self.leaning(s), self.tip(q));
            pos[lean.axis] += tip * w;
            vel[lean.axis] += tip * dw;
            // The leaning body's tip rises by a u^2 along the lever axis (`rise_coefficients`).
            if let Some(z) = lean.lever_axis {
                let [rise, d_rise, _] = rise_coefficients(lean, q);
                pos[z] += rise * w * w;
                vel[z] += 2.0 * rise * w * dw + d_rise * w * w * v[z];
            }
        }
        // The carriage hangs w lower on the beam, and the tool with it.
        if let Some(b) = &self.beam {
            let (w, dw) = self.deflection(s);
            pos[b.vertical] -= w;
            vel[b.vertical] -= dw;
        }
    }
    /// The carriage's position against the frame: the frame's sway and the lean are not in it.
    fn scale_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (q, v) = self.positions(s);
        pos.copy_from_slice(&self.tool_of(q));
        vel.copy_from_slice(&self.tool_speed(q, v));
    }
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        let (n, m, v) = (self.n(), self.m(), self.vel());
        let tool = self.tool_of(&s[..n]);
        let (_, vel, _) = self.joint_motion(&tool, vel, &vec![0.0; tool.len()]);
        s[v..v + n].copy_from_slice(&vel);
        if self.elastic {
            for kk in 0..m {
                let w = self.pulley_angle(kk, &vel);
                (s[v + n + kk], s[v + n + m + kk]) = (w, self.ratio[kk] * w);
            }
        }
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (n, m, v) = (self.n(), self.m(), self.vel());
        if self.elastic {
            pos.copy_from_slice(&s[n + m..n + 2 * m]);
            vel.copy_from_slice(&s[v + n + m..v + n + 2 * m]);
        } else {
            self.motor_angles(&s[..n], pos);
            self.motor_angles(&s[n..2 * n], vel);
        }
    }

    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        self.feedforward_torque_moving(pos, &vec![0.0; pos.len()], acc, out);
    }
    /// With the joints' motion of the tool's (`joint_motion`) and the forces its speed asks of the bodies on rotaries
    /// (`inertial`). Where a gantry's yaw is driven, its row asks the two drives for the moment that keeps the bridge
    /// square: they share the bridge's load by the lever rule.
    fn feedforward_torque_moving(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut [f64]) {
        let (q, dq, ddq) = self.joint_motion(pos, vel, acc);
        let mass = self.rigid_mass(&q);
        let (gravity, inertial) = (self.gravity(&q), self.inertial(&q, &dq));
        // The rigid model's speed terms as `rigid_solve` has them: the belt's runs and the growing chains.
        let (runs, chain) = (self.runs_speed(&dq, [0.0; 3], 0.0).0, self.chain_inertia(&dq, 0.0).0);
        let forces: Vec<f64> =
            (0..self.n()).map(|i| dot(&mass[i], &ddq) - gravity[i] - inertial[i] - runs[i] - chain[i]).collect();
        out.copy_from_slice(&self.to_motors(&forces));
    }

    /// The belt pulls the bodies (with chains and idlers) with the force F their masses take, without pulleys and
    /// rotors, -C^T T = F, and each closed belt keeps its length: sum_s (T_s - P) L_s = 0 over its geometric section
    /// lengths (the elastic law to first order in T / EA).
    fn rigid_tensions(&self, pos: &[f64], acc: &[f64], out: &mut Vec<f64>) {
        let (q, _, acc) = self.joint_motion(pos, &vec![0.0; pos.len()], acc);
        let (mass, gravity) = (self.axis_mass(&q), self.gravity(&q));
        let geo = self.geo(&q);
        // An axis a screw, rack, linear motor or rotary drives takes its force from that; a gantry's yaw spring takes
        // the moment the belt leaves.
        let belt_held: Vec<usize> = (0..self.nt()).filter(|&i| !self.driven(i)).collect();
        let mut a: Vec<Vec<f64>> = belt_held.iter().map(|&i| self.sections.iter().map(|s| s.c[i]).collect()).collect();
        let mut b: Vec<f64> = belt_held.iter().map(|&i| -(dot(&mass[i], &acc) - gravity[i])).collect();
        // Each closed belt keeps its length.
        for belt in &self.loops {
            a.push((0..self.sections.len()).map(|s| if belt.contains(&s) { geo[s] } else { 0.0 }).collect());
            b.push(0.0);
        }
        let d = lstsq(&a, &b).unwrap_or_else(|| vec![0.0; self.sections.len()]);
        out.extend(d.iter().map(|d| self.belt.pretension + d));
    }

    /// Each guide's load (`guide_load`, with what a linear motor presses on it) at the axis positions `pos` and
    /// accelerations `acc`: at rest its weight across the axis and the press, moving the overhang's tilt (and side
    /// load), m a (2 |lever| / spacing (+ 1)), signed with the acceleration.
    /// Each guide's Coulomb and viscous friction (`Lugre::steady`) on its axis's coordinate, as torque at the motors by
    /// the map the rigid forces take (`to_motors`).
    /// Each guide's level takes its load at rest at the pose (the load scale `load_scales` has, 1 without
    /// effects.load_dependent_friction); a moving overhang's added load follows the acceleration, which this does not
    /// see (`rigid_guide_loads` gives it). The pair [c, b] of c + b v has no room for the Stribeck rise, which fades
    /// within a few v_s: `rigid_friction_rise` gives it.
    fn rigid_friction(&self, pos: &[f64], dir: &[f64], out: &mut Vec<[f64; 2]>) {
        let (q, rate, _) = self.joint_motion(pos, dir, &vec![0.0; pos.len()]);
        let (mut coulomb, mut viscous) = (vec![0.0; self.n()], vec![0.0; self.n()]);
        for (g, scale) in self.guides.iter().zip(self.load_scales(&q, &vec![0.0; self.m()])) {
            let [c, b, _] = g.friction.steady(rate[g.axis], scale);
            (coulomb[g.axis], viscous[g.axis]) = (coulomb[g.axis] + c, viscous[g.axis] + b);
        }
        out.extend(self.to_motors(&coulomb).into_iter().zip(self.to_motors(&viscous)).map(|(c, b)| [c, b]));
    }

    /// The guides' Stribeck rise per motor, [level, speed]: the friction beyond c + b v is level exp(-(v / speed)^2) at
    /// the speed v along the path, level the guides' (stiction - Coulomb) at their load at rest taken to the motor as
    /// `rigid_friction` does, signed by the way each axis turns, and its Stribeck speed along the path (the guide's over
    /// its axis's rate): a pair per guide that reaches the motor.
    fn rigid_friction_rise(&self, pos: &[f64], dir: &[f64], out: &mut Vec<Vec<[f64; 2]>>) {
        let (q, rate, _) = self.joint_motion(pos, dir, &vec![0.0; pos.len()]);
        let (n, m) = (self.n(), self.m());
        let mut rises = vec![Vec::new(); m];
        for (g, scale) in self.guides.iter().zip(self.load_scales(&q, &vec![0.0; m])) {
            let (axis, turning) = (g.axis, rate[g.axis]);
            if !g.friction.on || turning == 0.0 {
                continue;
            }
            let mut rise = vec![0.0; n];
            rise[axis] = scale * (g.friction.stiction - g.friction.coulomb) * turning.signum();
            let speed = g.friction.stribeck_v / turning.abs();
            for (kk, torque) in self.to_motors(&rise).into_iter().enumerate().filter(|(_, t)| *t != 0.0) {
                rises[kk].push([torque, speed]);
            }
        }
        out.extend(rises);
    }

    /// `rigid_solve` at a steady speed: the guides (`Lugre::feedforward`, at their load at rest), idlers and cable chains
    /// as forces on the axes, taken to the motors as the rigid forces are (`to_motors`), over each gearbox's loss factor
    /// for what it passes at rest (that friction less the weight; the loss on the weight counts as friction), plus each shaft's friction at its rotor's speed.
    fn steady_friction(&self, pos: &[f64], vel: &[f64], out: &mut [f64]) {
        let (q, v, _) = self.joint_motion(pos, vel, &vec![0.0; pos.len()]);
        let n = self.n();
        let mut force = vec![0.0; n];
        for (g, scale) in self.guides.iter().zip(self.load_scales(&q, &vec![0.0; self.m()])) {
            force[g.axis] += g.friction.feedforward(v[g.axis], scale);
        }
        let ((idlers, _), drag, gravity) = (self.idler_friction(&v), self.chain_drag(&v), self.gravity(&q));
        let friction: Vec<f64> = (0..n).map(|i| force[i] - idlers[i] + drag[i]).collect();
        let held: Vec<f64> = (0..n).map(|i| friction[i] - gravity[i]).collect();
        let (weight, passed) = (self.to_motors(&gravity), self.to_motors(&held));
        for (kk, out) in out.iter_mut().enumerate() {
            let pulley = self.pulley_angle(kk, &v);
            let (output, rotor) = self.rigid_factors(kk, self.ratio[kk] * passed[kk], pulley);
            // The loss factor acts on everything the gearbox passes, the weight included; the weight's own lossless torque
            // (-weight) is not friction, so it comes off again.
            *out = self.shafts[kk].torque(self.ratio[kk] * pulley) + passed[kk] * rotor / output + weight[kk];
        }
    }

    fn rigid_guide_loads(&self, pos: &[f64], acc: &[f64], out: &mut Vec<[f64; 2]>) {
        let still = vec![0.0; self.n()];
        for g in &self.guides {
            let moving = g.overhang.as_ref().map_or(0.0, |o| {
                let lever = o.lever - o.lever_axis.map_or(0.0, |a| pos[a]);
                let side = self.body_mass(&self.bodies[o.body]) * acc[o.axis];
                side * (2.0 * lever.abs() / o.spacing + if o.side_load { 1.0 } else { 0.0 })
            });
            out.push([self.guide_load(g, pos, &still) + self.pressed(g.axis), moving]);
        }
    }

    fn energy(&self, s: &[f64]) -> f64 {
        self.energy_parts(s).iter().sum()
    }

    /// The lowest belt mode (`belt_mode`), in the elastic model.
    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        let mut q = self.joints(tool_pose);
        self.sag_at_rest(&mut q);
        self.belt_mode(&q).map(|(w2, _)| w2.sqrt() / std::f64::consts::TAU)
    }

    /// The mode of `resonance_hz`: the axes and the frame move, pulleys and rotors stand. Scaled so that the tool's
    /// largest travel along an axis is 1 m, positive (with the frame the tool moves with it, see `tool_state`).
    fn mode_shape(&self, tool_pose: &[f64]) -> Option<(f64, Vec<f64>)> {
        let mut q = self.joints(tool_pose);
        self.sag_at_rest(&mut q);
        let (w2, x) = self.belt_mode(&q)?;
        let n = self.n();
        let mut tool = x[..self.nt()].to_vec();
        if let Some(f) = &self.frame {
            tool[f.axis] += x[n];
        }
        if let Some(lean) = &self.lean {
            tool[lean.axis] += self.tip(&q) * x[self.lean_row()];
        }
        if let Some(b) = &self.beam {
            tool[b.vertical] -= x[self.beam_row()];
        }
        let largest = tool.iter().copied().max_by(|a, b| a.abs().total_cmp(&b.abs())).filter(|t| t.abs() > 0.0)?;
        let mut shape = vec![0.0; self.state_len()];
        for i in 0..n {
            shape[i] = x[i] / largest;
        }
        if self.frame.is_some() {
            shape[self.frame_at()] = x[n] / largest;
        }
        if self.lean.is_some() {
            shape[self.lean_at()] = x[self.lean_row()] / largest;
        }
        if self.beam.is_some() {
            shape[self.beam_at()] = x[self.beam_row()] / largest;
        }
        Some((w2.sqrt() / std::f64::consts::TAU, shape))
    }

    /// With effects.belt_wrap: per motor the wrap at its pulley (`geometry::wrap`), from where the 3D view puts it and
    /// the wheels before and after it along the belt (`wrap_neighbours`) at this pose, in the belt's plane (elt_plane); NaN
    /// for a motor that drives no belt (a screw, rack, linear motor or rotary).
    fn belt_wraps(&self, tool_pose: &[f64]) -> Vec<f64> {
        if self.wraps.is_empty() {
            return Vec::new();
        }
        self.wraps_at(&self.joints(tool_pose))
    }

    fn set_payload_attached(&mut self, attached: bool) {
        self.payload_attached = attached;
    }
    fn set_dwelling(&mut self, dwelling: bool) {
        self.dwelling = dwelling;
    }
    fn set_tool_force(&mut self, force: &[f64]) -> bool {
        self.tool_push.clear();
        self.tool_push.extend_from_slice(force);
        true
    }
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.brake_engaged = engaged;
    }

    fn memory(&self) -> Vec<f64> {
        Gearbox::anchors(&self.gears)
    }
    fn set_memory(&mut self, memory: &[f64]) {
        Gearbox::set_anchors(&mut self.gears, memory)
    }

    /// Belt tooth jump at each motor pulley (`ToothJump`): a positive slip feeds the section the pulley feeds.
    fn after_step(&mut self, s: &mut [f64], dt: f64) {
        self.jump_loss = 0.0;
        // The gearboxes' hysteresis states move on to where the step ended (`Gearbox::commit`).
        if self.elastic {
            let (n, m) = (self.n(), self.m());
            for kk in 0..m {
                let (phi, theta) = (s[n + m + kk], self.output(s, kk).0);
                self.gears[kk].commit(phi, theta);
            }
        }
        let v = self.vel() + self.n();
        let wraps = self.wraps_at(&s[..self.n()]);
        for j in 0..self.jumps.len() {
            let kk = self.jump_motor[j];
            let (mut jump, feeds, takes) = self.jumps[j].clone();
            if let Some(&wrap) = wraps.get(kk).filter(|w| w.is_finite()) {
                jump.set_wrap(wrap);
            }
            let (before, t) = (self.energy(s), self.tensions(s));
            let (at, r) = (self.jump_at(j), self.r[kk]);
            let (rest, mut speed) = (self.j_pulley[kk] / (r * r) - jump.mass, r * s[v + kk]);
            if jump.update(&mut s[at..at + JUMP_STATES], t[feeds], t[takes], rest, &mut speed) {
                s[v + kk] = speed / r;
                self.jump_loss += (before - self.energy(s)) / dt;
            }
        }
    }

    /// The gearboxes (where there are some) and the guides, in the order of `thermal_parts`.
    fn set_part_temperatures(&mut self, rises: &[f64]) {
        let Some((laws, viscous)) = &self.thermal else { return };
        let mut next = rises.iter();
        // A screw's or rack's share of the efficiency stays what it is.
        for (kk, (gear, law)) in self.gears.iter_mut().zip(laws).enumerate() {
            if !self.coupled[kk] {
                let rise = *next.next().unwrap_or(&0.0);
                self.shafts[kk].set_oil(law.viscosity_factor(rise));
                // A worm stage's efficiencies are its own (`worm_stages`), its loss following the oil as the gearbox's.
                match &self.worms_configured[kk] {
                    Some(worm) => {
                        let s = law.gearbox_viscous_share;
                        self.worms[kk] = Some(crate::worm::warmed(worm, 1.0 - s + s * law.viscosity_factor(rise)));
                    }
                    None => gear.efficiency = law.gearbox_efficiency(rise) * self.extra_efficiency[kk],
                }
            }
        }
        for (g, base) in self.guides.iter_mut().zip(viscous) {
            g.friction.viscous = base * laws[0].viscosity_factor(*next.next().unwrap_or(&0.0));
        }
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out: Vec<SignalInfo> = self
            .sections
            .iter()
            .map(|s| SignalInfo::new(format!("{}.tension", s.id), s.key, "quantity-tension", "N"))
            .collect();
        for axis in &self.axes {
            out.push(SignalInfo::new(format!("axis.{}.belt_stretch", axis.id), axis.key, "quantity-belt-stretch", "m"));
            out.push(SignalInfo::new(format!("axis.{}.friction", axis.id), axis.key, "quantity-friction-force", "N"));
        }
        if self.frame_node {
            out.push(SignalInfo::new("frame.deflection", "frame", "quantity-deflection", "m"));
        }
        if let Some(lean) = &self.lean_node {
            let axis = &self.axes[lean.axis];
            out.push(SignalInfo::new(
                format!("axis.{}.tool_deflection", axis.id),
                axis.key,
                "quantity-tool-deflection",
                "m",
            ));
        }
        if self.beam.is_some() {
            out.push(SignalInfo::new("beam.deflection", "group-beam", "quantity-deflection", "m"));
        }
        if let Some(g) = &self.gantry {
            for (name, quantity, unit) in [
                ("yaw", "quantity-yaw-angle", "rad"),
                ("sync_error", "quantity-sync-error", "m"),
                ("moment", "quantity-yaw-moment", "N m"),
            ] {
                out.push(SignalInfo::new(format!("gantry.{}.{name}", g.id), "group-gantry_yaw", quantity, unit));
            }
        }
        out.push(SignalInfo::new("belt.slack_time", "group-belt", "quantity-slack-time", "s"));
        // Only a motor that turns a pulley has one: the verdicts judge belt teeth and pulley bearings where it does.
        for (motor, _) in self.motors.iter().zip(&self.direct).filter(|(_, d)| d.is_none()) {
            out.push(SignalInfo::new(
                format!("pulley.{}.radial_load", motor.id),
                motor.key,
                "quantity-radial-load",
                "N",
            ));
        }
        for g in &self.guides {
            out.push(SignalInfo::new(
                format!("{}.guide_load", g.contact),
                self.axes[g.axis].key,
                "quantity-guide-load",
                "N",
            ));
        }
        for motor in &self.motors {
            out.push(SignalInfo::new(
                format!("gearbox.{}.output_torque", motor.id),
                motor.key,
                "quantity-gearbox-torque",
                "N m",
            ));
        }
        if !self.jumps.is_empty() {
            for motor in self.jump_motor.iter().map(|&kk| &self.motors[kk]) {
                out.push(SignalInfo::new(
                    format!("pulley.{}.tooth_jumps", motor.id),
                    motor.key,
                    "quantity-tooth-jumps",
                    "",
                ));
                out.push(SignalInfo::new(
                    format!("pulley.{}.tooth_slip", motor.id),
                    motor.key,
                    "quantity-tooth-slip",
                    "m",
                ));
            }
        }
        if self.span_mass.is_some() {
            // Numbered from 1 in the file's order, as the crates number theirs (`belt.span.<i>.frequency`); the label names the node.
            out.extend(self.spans.iter().enumerate().map(|(i, p)| {
                SignalInfo::new(format!("belt.span.{}.frequency", i + 1), p.key, "quantity-span-frequency", "Hz")
            }));
            out.push(SignalInfo::new("belt.span_detuning", "group-belt", "quantity-span-detuning", ""));
        }
        // Named by the motor, as the registered one-axis machines name them by their joint (`joint.1.force`, `screw.1.speed`).
        for (kk, d) in self.direct.iter().enumerate() {
            let Some(d) = d else { continue };
            let (motor, axis) = (&self.motors[kk], &self.axes[d.axis]);
            out.push(SignalInfo::new(
                format!("joint.{}.force", motor.id),
                axis.key,
                "quantity-joint-force",
                if axis.unit == "m" { "N" } else { "N m" },
            ));
            match &d.kind {
                DirectKind::Screw(screw) => out.extend(screw.signal_infos(motor.id, motor.key)),
                DirectKind::Rack(rack) => out.extend([
                    rack.signal_info(motor.id, motor.key),
                    SignalInfo::new(format!("rack.{}.radial_load", motor.id), motor.key, "quantity-radial-load", "N"),
                ]),
                DirectKind::Rotary(Some(gear)) => out.push(gear.signal_info(motor.id, motor.key)),
                _ => {}
            }
        }
        self.effect_infos(&mut out);
        out
    }

    /// The checks of the screws (speed, d n, buckling, static load, life), racks (feed force) and strain wave gears
    /// (life, where the motor's ratings.required_life_h is set), as the registered one-axis machines judge them.
    fn gantry_motors(&self) -> Option<[usize; 2]> {
        self.gantry_motors
    }

    fn rating_checks(&self) -> Vec<RatingCheck> {
        let mut out = Vec::new();
        // The yaw's synchronisation error (width times yaw) against the gantry's tolerance, where it has one.
        if let Some(g) = &self.gantry {
            out.extend(g.tolerance.map(|limit| RatingCheck {
                check: "gantry-sync",
                signal: format!("gantry.{}.sync_error", g.id),
                motor: None,
                limit,
                life: false,
            }));
        }
        for (kk, d) in self.direct.iter().enumerate() {
            let Some(d) = d else { continue };
            let id = self.motors[kk].id;
            match &d.kind {
                DirectKind::Screw(screw) => out.extend(screw.rating_checks(id, kk, d.required_life)),
                DirectKind::Rack(rack) => {
                    out.extend(rack.rating_checks(id, kk));
                    out.extend(d.radial_limit.map(|limit| RatingCheck {
                        check: "gearbox-radial-load",
                        signal: format!("rack.{id}.radial_load"),
                        motor: Some(kk),
                        limit,
                        life: false,
                    }));
                }
                DirectKind::Rotary(Some(gear)) => {
                    out.extend(d.required_life.map(|life| gear.rating_check(id, kk, life)))
                }
                _ => {}
            }
        }
        out
    }

    fn friction_contacts(&self) -> Vec<FrictionContact> {
        self.guides.iter().map(|g| FrictionContact { id: g.contact, owner_key: self.axes[g.axis].key }).collect()
    }
    fn friction_curve(&self, i: usize, speed: f64, load_scale: f64) -> [f64; 3] {
        self.guides.get(i).map_or([0.0; 3], |g| g.friction.steady(speed, load_scale))
    }

    fn belt_sections(&self) -> Vec<BeltSection> {
        self.sections
            .iter()
            .map(|s| BeltSection { id: s.id, owner_key: s.key, pretension: self.belt.pretension })
            .collect()
    }

    /// The gearboxes (not on a direct drive, see `Gearbox::is_coupling`), then the guides, each shown by the first
    /// body it carries (or its overhang).
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        let mut parts = Vec::new();
        for (kk, motor) in self.motors.iter().enumerate().filter(|(kk, _)| !self.coupled[*kk]) {
            parts.push(ThermalPart {
                id: intern(&format!("gearbox_{}", motor.id)),
                kind: "gearbox",
                owner_key: motor.key,
                scene_part: "gearbox",
                scene_index: kk,
            });
        }
        for g in &self.guides {
            let body = g.carries.first().copied().or(g.overhang.as_ref().map(|o| o.body)).unwrap_or(0);
            parts.push(ThermalPart {
                id: g.loss_id,
                kind: "guide",
                owner_key: self.axes[g.axis].key,
                scene_part: "carriage",
                scene_index: body,
            });
        }
        parts
    }

    /// The rigid model has no belt stretch: it reports pretension, zero stretch and no slack. A section is taken to
    /// wrap the pulleys it ends at by half a turn, so the radial load on a pulley's shaft is the sum of the tensions
    /// of the sections that end there.
    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (n, m) = (self.n(), self.m());
        let (q, vq) = self.positions(s);
        let fa = self.friction_at();
        let (friction, _) = self.friction(q, vq, &s[fa..fa + n], torque);
        let tensions = self.tensions(s);
        let (stretch, slack, outputs) = if self.elastic {
            let angles: Vec<f64> = (0..m).map(|kk| self.output(s, kk).0).collect();
            let stretch: Vec<f64> = (0..n).map(|i| q[i] - dot(&self.k_inv[i], &angles)).collect();
            let outputs: Vec<f64> = (0..m)
                .map(|kk| {
                    let (theta, w_theta) = self.output(s, kk);
                    self.gear_torque(kk, s[n + m + kk], s[self.vel() + n + m + kk], theta, w_theta).output
                })
                .collect();
            (stretch, s[self.slack_at()], outputs)
        } else {
            (vec![0.0; n], 0.0, self.rigid_solve(s, torque).out)
        };
        out.extend(&tensions);
        for i in 0..self.nt() {
            out.extend([stretch[i], friction[i]]);
        }
        if self.frame_node {
            out.push(self.sway(s).0);
        }
        if self.lean_node.is_some() {
            out.push(self.tip(q) * self.leaning(s).0);
        }
        if self.beam.is_some() {
            out.push(self.deflection(s).0);
        }
        if let Some(g) = &self.gantry {
            let (yaw, rate) = (q[g.coord], vq[g.coord]);
            out.extend([yaw, g.width * yaw, g.stiffness * yaw + g.damping * rate]);
        }
        out.push(slack);
        for kk in (0..m).filter(|&kk| self.direct[kk].is_none()) {
            out.push(self.sections.iter().zip(&tensions).filter(|(sec, _)| sec.f[kk] != 0.0).map(|(_, t)| t).sum());
        }
        let acc =
            if self.guides.iter().any(|g| g.overhang.is_some()) { self.commanded_acc(q, torque) } else { vec![0.0; n] };
        out.extend(self.guides.iter().map(|g| self.guide_load(g, q, &acc) + self.pressed(g.axis)));
        // Behind a screw or rack the gearbox's own output, ahead of the converter: more than the force times the lever
        // while it drives, less while the load drives it back (as the registered one-axis machines).
        out.extend((0..m).map(|kk| match &self.direct[kk] {
            Some(Direct { lever: Some(lever), .. }) => {
                let (force, e) = (outputs[kk], self.extra_efficiency[kk]);
                force * lever * if force * dot(&self.k[kk], vq) >= 0.0 { 1.0 / e } else { e }
            }
            _ => outputs[kk],
        }));
        for j in 0..self.jumps.len() {
            out.extend([s[self.jump_at(j) + 4], s[self.jump_at(j)]]);
        }
        if let Some(mass) = self.span_mass {
            self.span_signals(s, mass, out);
        }
        // The screws', racks' and strain wave gears' own signals, from the force each passes to its axis.
        for (kk, d) in self.direct.iter().enumerate() {
            let Some(d) = d else { continue };
            let (force, speed) = (outputs[kk], dot(&self.k[kk], vq));
            out.push(force);
            match &d.kind {
                DirectKind::Screw(screw) => screw.signals(speed, force, out),
                DirectKind::Rack(rack) => {
                    let teeth = RackPinion::feed_force(force, speed, self.extra_efficiency[kk]);
                    out.extend([teeth, rack.radial_load(teeth)]);
                }
                DirectKind::Rotary(Some(gear)) => out.push(gear.signal(force, speed * self.ratio[kk])),
                _ => {}
            }
        }
    }
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.view_signals(s, torque, out);
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(s, torque).0);
    }

    /// A plain scene: a beam, a box per body, a drive unit per motor at its pulley's `at_m`, the idlers, and the belt
    /// round its pulleys and idlers: each section from tangent to tangent through its route (`belt_line`), shaded by its
    /// tension against the pretension, and the wraps round the motors' pulleys between sections (`pulley_wraps`).
    fn scene(&self, s: &[f64], out: &mut Scene) {
        use std::f64::consts::TAU;
        let (n, m) = (self.n(), self.m());
        let (q, _) = self.positions(s);
        let (mut rotor, mut rotor_speed) = (vec![0.0; m], vec![0.0; m]);
        self.motor_state(s, &mut rotor, &mut rotor_speed);
        out.parts.clear();
        out.labels.clear();
        let end = self.stroke[0];
        // The tool from where the rotaries' axes meet to its tip.
        if let Some(t) = &self.tcp {
            let pivot: Vec<bool> = (0..n).map(|i| t.axes.contains(&i)).collect();
            let from = self.scene_at(q, &pivot);
            let tip = self.turned_by(&t.rotaries, q, t.tool);
            let to = [0, 1, 2].map(|k| from[k] + tip[k]);
            out.parts.push(Part::new("tool", Shape::Segment { to, width: 0.015 }, from, [0.85, 0.2, 0.2]));
        }
        out.parts.push(Part::new(
            "beam",
            Shape::Box { size: [end + 0.2, 0.07, 0.09] },
            [end / 2.0, 0.045, 0.0],
            [0.62, 0.64, 0.68],
        ));
        for (b, body) in self.bodies.iter().enumerate() {
            let at = self.body_scene_at(body, q);
            let shade = 0.2 + 0.1 * b as f32;
            out.parts.push(Part::new(
                "carriage",
                Shape::Box { size: [0.16, 0.05, 0.03] },
                at,
                [shade, shade + 0.02, shade + 0.06],
            ));
        }
        // A wheel of a belt that lies flat is drawn upright at its place in the belt's plane and laid flat
        // (`Part::lay_flat`): it faces up, the motor below it.
        let flat = self.belt_plane().1 == 2;
        let upright = |at: [f64; 3]| if flat { ([at[0], at[1]], -at[2]) } else { ([at[0], at[2]], at[1]) };
        let lay = |parts: &mut [Part]| {
            if flat {
                parts.iter_mut().for_each(Part::lay_flat)
            }
        };
        for kk in 0..m {
            let at = match &self.direct[kk] {
                Some(d) => self.direct_scene(kk, d, q, out),
                None => {
                    let at = self.pulley_scene_at(kk);
                    let pulley = if self.elastic { s[n + kk] } else { rotor[kk] / self.ratio[kk] };
                    let (from, (centre, depth)) = (out.parts.len(), upright(at));
                    drive_unit(&mut out.parts, centre, depth, self.r[kk], -pulley, -rotor[kk], [0.9, 0.45, 0.1]);
                    lay(&mut out.parts[from..]);
                    at
                }
            };
            out.labels.push(Label {
                pos: [at[0], at[1], at[2] + 0.11],
                owner_key: self.motors[kk].key,
                value: rotor_speed[kk] * 60.0 / TAU,
                unit_key: "unit-rpm",
                decimals: 0,
            });
        }
        for d in &self.idlers {
            let (from, (centre, depth)) = (out.parts.len(), upright(self.idler_scene_at(d, q)));
            idler(&mut out.parts, centre, depth, d.radius, -dot(&d.speed, q) / d.radius);
            lay(&mut out.parts[from..]);
        }
        // A machine without idler nodes and one belt motor gets the idler at the far end the one-axis machine has.
        if m == 1 && self.idlers.is_empty() && self.direct[0].is_none() {
            idler(&mut out.parts, [end + 0.1, 0.0], -0.05, self.r[0], 0.0);
        }
        let tensions = self.tensions(s);
        let pretension = self.belt.pretension.max(1.0);
        let strip = |from: [f64; 3], to: [f64; 3]| {
            Part::new("belt", Shape::Segment { to, width: 0.02 }, from, [0.12, 0.12, 0.13])
        };
        let shade = |t: f64| ((t - self.belt.pretension) / pretension).clamp(-1.0, 1.0) as f32;
        for (i, t) in tensions.iter().enumerate() {
            for pair in self.belt_line(i, q).windows(2) {
                out.parts.push(strip(pair[0], pair[1]).highlighted(shade(*t)));
            }
        }
        // Shaded as the section that comes onto the pulley.
        for (section, wrap) in self.pulley_wraps(q) {
            out.parts.extend(wrap.windows(2).map(|pair| strip(pair[0], pair[1]).highlighted(shade(tensions[section]))));
        }
        for (i, axis) in self.axes.iter().enumerate() {
            let moves: Vec<bool> = (0..n).map(|l| l <= i).collect();
            let at = self.scene_at(q, &moves);
            let (value, unit_key, decimals) =
                if axis.unit == "m" { (q[i], "unit-m", 3) } else { (q[i].to_degrees(), "unit-deg", 1) };
            out.labels.push(Label {
                pos: [at[0], at[1], at[2] + 0.16 + 0.06 * i as f64],
                owner_key: axis.key,
                value,
                unit_key,
                decimals,
            });
        }
    }

    /// The tool's tip: where the straight axes put the rotaries, plus the tool turned by them.
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        let q = self.joints(tool_pose);
        let mut p = self.scene_at(&q, &vec![true; self.n()]);
        if let Some(t) = &self.tcp {
            let tip = self.turned_by(&t.rotaries, &q, t.tool);
            (0..3).for_each(|k| p[k] += tip[k]);
        }
        Some(p)
    }

    /// With a tcp: the tool's tip where the straight axes would have to stand beyond their strokes.
    fn unreachable(&self, tool_pose: &[f64], slack: f64) -> Option<&'static str> {
        let t = self.tcp.as_ref()?;
        let q = self.joints(tool_pose);
        t.axes.iter().any(|&i| q[i] < -slack || q[i] > self.stroke[i] + slack).then_some("error-tcp-joint-range")
    }
}

#[cfg(test)]
#[path = "wheel_sense_tests.rs"]
mod wheel_sense_tests;
