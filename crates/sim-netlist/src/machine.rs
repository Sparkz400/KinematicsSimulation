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

use crate::geometry::{Wheel, arc, contacts, flex_at, wrap};
use crate::netlist::{LeanNode, Netlist, TcpNode, Transmission, Waypoint, WormNode};
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

struct Span {
    key: &'static str,
    section: usize,
    length: f64,
    c: Vec<f64>,
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
    /// shaft with sideways (`pinion_radial_load`).
    radial_limit: Option<f64>,
}

/// The standard pressure angle of a rack and pinion's teeth (DIN 867), in the normal section.
/// shortcut: every rack is taken at 20 degrees; give `[rack]` a pressure angle if a file uses another.
const RACK_PRESSURE_ANGLE_DEG: f64 = 20.0;

/// The radial load the pinion puts on the gearbox's output shaft for the force `tooth_force` on its teeth: the tooth
/// force itself, tangential to the pinion and so across the shaft, and the force that pushes pinion and rack apart,
/// F tan(alpha_n) / cos(beta), at right angles to it: |F| sqrt(1 + (tan(alpha_n) / cos(beta))^2). The axial force of a
/// helical pinion, F tan(beta), and the tilt it puts on the shaft are left out.
fn pinion_radial_load(rack: &RackPinion, tooth_force: f64) -> f64 {
    let apart = RACK_PRESSURE_ANGLE_DEG.to_radians().tan() / rack.helix_angle.cos();
    tooth_force.abs() * (1.0 + apart * apart).sqrt()
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
    dwelling: bool,
    /// The netlist has a frame, and with effects.frame_compliance in the elastic model it sways.
    frame_node: bool,
    frame: Option<FrameMode>,
    /// With effects.thermal_feedback: the law per motor (its gearbox's efficiency), and each guide's viscous
    /// coefficient at the reference temperature.
    thermal: Option<(Vec<ThermalFeedback>, Vec<f64>)>,
    /// With effects.belt_tooth_jump (elastic model only): per motor the law and the sections its pulley feeds (+1) and
    /// takes from (-1); the energy the last step's jumps took, J/s.
    jumps: Vec<(ToothJump, usize, usize)>,
    jump_loss: f64,
    /// With effects.belt_span_vibration: the belt's mass per metre, and the spans.
    span_mass: Option<f64>,
    spans: Vec<Span>,
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
    let cross = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    let along = dot(&u, &v) * (1.0 - c);
    [0, 1, 2].map(|k| v[k] * c + cross[k] * s + u[k] * along)
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
                    screw.converter(screw.softest_position()).apply(own, &mut joint)?;
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
                    rack.converter().apply(own, &mut joint)?;
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
                    // The table's bearing drags at the axis: at the rotor, i times as fast, as c / i + v w / i^2.
                    // shortcut: the bearing's friction acts at the rotor, before the gearbox's spring and efficiency, as
                    // a screw's preload does; give the elastic model a friction at the output if a soft gear matters.
                    if guide_friction {
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
        // With a tcp the tool's tip reaches beyond the straight axes' strokes by its distance from the rotaries.
        let reach = net.tcp.as_ref().map_or(0.0, |t| t.tool.iter().map(|x| x * x).sum::<f64>().sqrt());
        let axes = net
            .axes
            .iter()
            .enumerate()
            .map(|(i, a)| -> Result<AxisInfo, ConfigError> {
                // A rotary axis's limits are in rad: max_velocity_rad_s.
                let unit = if a.rotary { "rad" } else { "m" };
                let limit = |what: &str, rate: &str| cfg.positive(&format!("axis.{}.max_{what}_{unit}_{rate}", a.id));
                let reach = if net.tcp.as_ref().is_some_and(|t| t.axes.contains(&i)) { reach } else { 0.0 };
                Ok(AxisInfo {
                    id: intern(&a.id),
                    key: intern(&a.key),
                    unit,
                    min: -reach,
                    max: a.stroke + reach,
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
        let mut jumps = Vec::new();
        // shortcut: the jumps' states are laid out per motor; a motor without a pulley would need a gap there. Give the
        // jump states to the belt motors only if a file needs tooth jump beside a screw.
        if elastic && on("belt_tooth_jump")? && direct.iter().any(Option::is_some) {
            return Err(ConfigError::Invalid(
                "effects.belt_tooth_jump (not with a screw, rack, linear motor or rotary yet)".into(),
            ));
        }
        if elastic && on("belt_tooth_jump")? {
            for (kk, d) in net.motors.iter().enumerate() {
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
        let spans = net
            .spans
            .iter()
            .map(|s| Span {
                key: intern(&format!("belt-span-{}", s.id)),
                section: s.section,
                length: s.length,
                c: pad(&s.axes),
            })
            .collect();
        let wraps = if on("belt_wrap")? { wrap_neighbours(net, &direct)? } else { Vec::new() };
        let lean = match (net.lean, on("tool_flex")?) {
            (None, true) => return Err(ConfigError::Missing("machine.lean (effects.tool_flex)".into())),
            // As in the machine crate with a leaning axis: the lean is a mode of the elastic model.
            (lean, flex) => lean.filter(|_| flex && elastic),
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
            dwelling: false,
            frame_node: net.frame.is_some(),
            frame,
            thermal,
            jumps,
            jump_loss: 0.0,
            span_mass,
            spans,
            belt,
            sections,
            guides,
            idlers,
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
    // `JUMP_STATES` per motor.
    // Rigid state: [q, v, friction states, brakes].
    fn vel(&self) -> usize {
        if self.elastic { self.lean_at() + usize::from(self.lean.is_some()) } else { self.n() }
    }
    fn frame_at(&self) -> usize {
        self.n() + 2 * self.m()
    }
    fn lean_at(&self) -> usize {
        self.frame_at() + usize::from(self.frame.is_some())
    }
    /// The leaning body's sideways travel against its carriage and its rate; zero without it.
    fn leaning(&self, s: &[f64]) -> (f64, f64) {
        if self.lean.is_some() { (s[self.lean_at()], s[self.vel() + self.lean_at()]) } else { (0.0, 0.0) }
    }
    /// The tool tip's travel along the leaning body's axis per metre it leans (`geometry::flex_at`); 0 without it.
    fn tip(&self, q: &[f64]) -> f64 {
        self.lean.as_ref().map_or(0.0, |lean| flex_at(lean, q).2)
    }

    /// Motor kk's gearbox (`Gearbox::torque`), with its worm stage's share where it has one (`worm::factors`, at the
    /// power its spring's torque puts through the output).
    fn gear_torque(&self, kk: usize, phi: f64, w_phi: f64, theta: f64, w_theta: f64) -> GearTorque {
        let gear = self.gears[kk].torque(phi, w_phi, theta, w_theta);
        let Some(worm) = &self.worms[kk] else { return gear };
        let (output, rotor) = crate::worm::factors(worm, gear.rotor * self.ratio[kk] * w_theta);
        GearTorque { output: gear.output * output, rotor: gear.rotor * rotor }
    }
    /// The share of the load motor kk holds at rest: none behind a self-locking worm, all of it otherwise.
    fn held(&self, kk: usize) -> f64 {
        self.worms[kk].as_ref().map_or(1.0, |worm| crate::worm::factors(worm, 0.0).1)
    }
    /// What motor kk's gearbox loses, W (`Gearbox::loss_power`), with its worm stage's share.
    fn gear_loss_power(&self, kk: usize, phi: f64, w_phi: f64, theta: f64, w_theta: f64) -> f64 {
        let (plain, worm) =
            (self.gears[kk].torque(phi, w_phi, theta, w_theta), self.gear_torque(kk, phi, w_phi, theta, w_theta));
        self.gears[kk].loss_power(phi, w_phi, theta, w_theta) + (worm.rotor - plain.rotor) * w_phi
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
    fn jump_at(&self, kk: usize) -> usize {
        self.brake_at() + self.m() + JUMP_STATES * kk
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
    /// d body_point / d q_i, per coordinate, by central differences (the place is smooth and cheap to work out).
    fn body_jacobian(&self, b: &Body, q: &[f64]) -> Vec<[f64; 3]> {
        let h = 1e-6;
        let mut at = q.to_vec();
        (0..self.n())
            .map(|i| {
                at[i] = q[i] + h;
                let ahead = self.body_point(b, &at);
                at[i] = q[i] - h;
                let behind = self.body_point(b, &at);
                at[i] = q[i];
                [0, 1, 2].map(|k| (ahead[k] - behind[k]) / (2.0 * h))
            })
            .collect()
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
    /// -m J^T (dJ/dt v), the centripetal and Coriolis forces, with dJ/dt v the second derivative of the body's place
    /// along v (by differences over 0.1 mm of travel). With the mass matrix m J^T J these are Lagrange's equations, so
    /// the energy keeps.
    fn inertial(&self, q: &[f64], v: &[f64]) -> Vec<f64> {
        let mut out = vec![0.0; self.n()];
        let speed = v.iter().map(|x| x * x).sum::<f64>().sqrt();
        if speed == 0.0 {
            return out;
        }
        let h = 1e-4 / speed;
        let shifted = |t: f64| -> Vec<f64> { q.iter().zip(v).map(|(q, v)| q + t * v).collect() };
        let (ahead, behind) = (shifted(h), shifted(-h));
        for b in self.bodies.iter().filter(|b| b.general) {
            let (p, a, z) = (self.body_point(b, q), self.body_point(b, &ahead), self.body_point(b, &behind));
            let curve = [0, 1, 2].map(|k| (a[k] - 2.0 * p[k] + z[k]) / (h * h));
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
            let r = self.turned_by(&t.rotaries, &q, t.tool);
            t.axes.iter().zip(r).for_each(|(&i, r)| q[i] -= r);
        }
        q
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
            t.axes.iter().zip(r).for_each(|(&i, r)| tool[i] += r);
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
    /// The rigid model's mass matrix at q: bodies, chains, idlers, pulleys and rotors.
    fn rigid_mass(&self, q: &[f64]) -> Vec<Vec<f64>> {
        let mut out = self.drive_mass();
        for (row, axis) in out.iter_mut().zip(self.axis_mass(q)) {
            row.iter_mut().zip(axis).for_each(|(a, b)| *a += b);
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
            for kk in 0..m {
                angle[kk] += s[self.jump_at(kk)] / self.r[kk];
                rate[kk] += s[self.jump_at(kk) + 1] / self.r[kk];
            }
        }
        (angle, rate)
    }
    fn sections(&self, s: &[f64]) -> Vec<SectionForce> {
        let (n, v) = (self.n(), self.vel());
        let (angle, rate) = self.fed(s);
        self.sections
            .iter()
            .map(|sec| {
                let geo = sec.length + dot(&sec.c, &s[..n]);
                let mat = sec.mat0 + (0..self.m()).map(|kk| sec.f[kk] * self.r_mat[kk] * angle[kk]).sum::<f64>();
                let mat_rate: f64 = (0..self.m()).map(|kk| sec.f[kk] * self.r_mat[kk] * rate[kk]).sum();
                self.belt.section(geo, mat, dot(&sec.c, &s[v..v + n]), mat_rate)
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
    /// value would turn every belt vibration into extra friction. Bodies, pulleys and rotors only, as the crates take it:
    /// chains and idlers change it by a few per mille.
    fn commanded_acc(&self, q: &[f64], torque: &[f64]) -> Vec<f64> {
        let forces: Vec<f64> =
            (0..self.n()).map(|i| (0..self.m()).map(|kk| self.ratio[kk] * self.k[kk][i] * torque[kk]).sum()).collect();
        let mut mass = self.drive_mass();
        for (i, m) in self.mass().into_iter().enumerate() {
            mass[i][i] += m;
        }
        self.general_mass(q, &mut mass);
        solve(mass, forces).unwrap_or_else(|| vec![0.0; self.n()])
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
    fn process_force(&self) -> Vec<f64> {
        if self.dwelling { self.process.clone() } else { vec![0.0; self.n()] }
    }
    /// The hard stops at the ends of axis i's joint; none on the yaw.
    fn stop_force(&self, i: usize, pos: f64, vel: f64) -> f64 {
        if i < self.nt() { self.stops.force(pos, vel, self.stroke[i]) } else { 0.0 }
    }

    /// With the body leaning (effects.tool_flex): the axes' accelerations and the lean's, w''. The body moves along its
    /// axis a at a_a + w'' (over the frame), so its mass m couples the two:
    /// [[M, m e_a], [m e_a^T, m]] [acc; w''] = [F; tip p_a - k w - c w'], M the axes' mass matrix and F their forces with
    /// the process force p on the tool: the guide's spring acts between carriage and body, inside, and the process
    /// force at the tool tip, which moves tip per w. The spring stiffens as the lever axis travels, which F there takes
    /// as -w^2 / 2 dk/dz. The machine crate with a leaning axis writes the same out for its one carriage.
    fn lean_acc(&self, lean: &LeanNode, s: &[f64], mut force: Vec<f64>, process: f64) -> (Vec<f64>, f64) {
        let (n, q) = (self.n(), &s[..self.n()]);
        let (w, dw) = self.leaning(s);
        let (k, dk, tip) = flex_at(lean, q);
        let body = self.body_mass(&self.bodies[lean.body]);
        let mut mass = self.axis_mass(q);
        for (i, row) in mass.iter_mut().enumerate() {
            row.push(if i == lean.axis { body } else { 0.0 });
        }
        let mut last = vec![0.0; n + 1];
        (last[lean.axis], last[n]) = (body, body);
        mass.push(last);
        if let Some(z) = lean.lever_axis {
            force[z] -= 0.5 * w * w * dk;
        }
        force.push(tip * process - k * w - lean.damping * dw);
        let mut x = solve(mass, force).unwrap_or_else(|| vec![0.0; n + 1]);
        let lean_acc = x.pop().unwrap_or(0.0);
        (x, lean_acc)
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
                belt - friction[i] + idlers[i] - drag[i]
                    + self.stop_force(i, q[i], vq[i])
                    + gravity[i]
                    + driven[i]
                    + inertial[i]
                    + yaw[i]
            })
            .collect();
        let (chain, chain_on_frame) = self.chain_inertia(vq, du);
        let frame_acc = self.frame.as_ref().map(|f| {
            (f.axis, (-f.stiffness * u - f.damping * du - force[f.axis] + chain_on_frame) / self.frame_mass(f))
        });
        let process = self.process_force();
        for i in 0..n {
            force[i] += chain[i] + process[i];
        }
        out[fa..fa + n].copy_from_slice(&rates[..n]);
        let mut acc = match &self.lean {
            None => solve(self.axis_mass(q), force).unwrap_or_else(|| vec![0.0; n]),
            Some(lean) => {
                let (acc, lean_acc) = self.lean_acc(lean, s, force, process[lean.axis]);
                out[v + self.lean_at()] = lean_acc;
                acc
            }
        };
        if let Some((axis, aq)) = frame_acc {
            acc[axis] -= aq;
            out[v + self.frame_at()] = aq;
        }
        out[v..v + n].copy_from_slice(&acc);
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
                let (on_pulley, j_pulley) = match self.jumps.get(kk) {
                    Some((jump, ..)) => {
                        let slip = &s[self.jump_at(kk)..];
                        (jump.pulley_torque(slip, belt, r), jump.pulley_inertia(slip, self.j_pulley[kk], r))
                    }
                    None => (belt, self.j_pulley[kk]),
                };
                out[v + n + kk] = (gear.output + on_pulley + r * self.mesh.force(r * theta)) / j_pulley;
                if let Some((jump, ..)) = self.jumps.get(kk) {
                    let at = self.jump_at(kk);
                    jump.rates(&s[at..], belt / r, r * out[v + n + kk], &mut out[at..]);
                }
            }
            let (brake, brake_rate) = self.brakes[kk].torque(self.brake_engaged, w_phi, s[ba + kk]);
            out[v + n + m + kk] =
                (self.ripples[kk].apply(torque[kk], phi) - gear.rotor - self.shafts[kk].torque(w_phi) - brake)
                    / self.j_rotor[kk];
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
            self.process_force(),
        );
        let (inertial, yaw) = (self.inertial(q, v), self.yaw_force(q, v));
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
        let (mut factors, mut solved) =
            (vec![(1.0, 1.0); m], Rigid { acc: vec![0.0; n], rates, out: vec![0.0; m], into: vec![0.0; m] });
        if self.worms.iter().flatten().any(|w| w.back_efficiency <= 0.0) {
            return self.rigid_locking(&fixed, &base, &drive, v, solved);
        }
        for _ in 0..3 {
            let loss: Vec<f64> = factors.iter().map(|(output, rotor)| output / rotor).collect();
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
            factors = (0..m).map(|kk| self.rigid_factors(kk, solved.into[kk], self.pulley_angle(kk, v))).collect();
        }
        solved
    }

    /// `rigid_solve` with a self-locking worm (`worm::factors`), whose rotor takes no share of the load at rest and
    /// pushes against it while it is driven back: the outputs o are solved for beside the accelerations, so that a
    /// rotor factor of 0 stays solvable. With `fixed` the mass matrix less the rotors and `base` the axes' forces, the
    /// axes take fixed acc - K^T o = base, and each rotor puts into its gearbox what its motor's torque (`drive`) leaves
    /// after its own acceleration, into = i (drive - J i K acc), of which the output gets o = into * output factor /
    /// rotor factor: rotor factor o + i^2 J K acc = output factor i drive.
    ///
    /// A locking worm driven back (its output's power o w below 0, the rotor factor below 0) takes a drag
    /// D = o - into = (1 - rotor factor) o_rest instead, o_rest the torque its output holds at rest against the axes'
    /// forces (K^T o_rest = -base): o + i^2 J K acc = i drive + D. Its output standing still sticks (K acc = 0) while
    /// the drag that takes, o - i drive, lies between what the worm gives raising the load, -(1 / eta - 1) o_rest, and
    /// lowering it, (1 - eta_back) o_rest; beyond, it breaks loose with the drag at that bound.
    fn rigid_locking(&self, fixed: &[Vec<f64>], base: &[f64], drive: &[f64], v: &[f64], mut solved: Rigid) -> Rigid {
        let (n, m) = (self.n(), self.m());
        let kt: Vec<Vec<f64>> = (0..n).map(|i| (0..m).map(|kk| self.k[kk][i]).collect()).collect();
        let holds = lstsq(&kt, &base.iter().map(|f| -f).collect::<Vec<_>>()).unwrap_or_else(|| vec![0.0; m]);
        let locking: Vec<Option<WormNode>> =
            self.worms.iter().map(|w| w.filter(|w| w.back_efficiency <= 0.0)).collect();
        // Per motor: (output factor, rotor factor), or the drag a locking worm takes, or (None) it sticks.
        #[derive(Clone, Copy)]
        enum Row {
            Factors(f64, f64),
            Drag(f64),
            Stuck,
        }
        let speed = |kk: usize| self.pulley_angle(kk, v);
        let mut rows: Vec<Row> = (0..m)
            .map(|kk| match locking[kk] {
                Some(_) if speed(kk) == 0.0 => Row::Stuck,
                Some(w) => Row::Factors(1.0, crate::worm::factors(&w, holds[kk] * speed(kk)).1),
                None => Row::Factors(1.0, 1.0),
            })
            .collect();
        for _ in 0..3 {
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
                let row = match *row {
                    Row::Factors(_, rotor) if locking[kk].is_some() && rotor <= 0.0 => {
                        Row::Drag((1.0 - rotor) * holds[kk])
                    }
                    other => other,
                };
                let (at_acc, at_o, rhs) = match row {
                    Row::Factors(output, rotor) => (output * inertia, rotor, output * into),
                    Row::Drag(drag) => (inertia, 1.0, into + drag),
                    Row::Stuck => (1.0, 0.0, 0.0),
                };
                for l in 0..n {
                    a[n + kk][l] = at_acc * self.k[kk][l];
                }
                a[n + kk][n + kk] = at_o;
                b.push(rhs);
            }
            let x = solve(a, b).unwrap_or_else(|| vec![0.0; n + m]);
            solved.acc = x[..n].to_vec();
            for kk in 0..m {
                solved.into[kk] = self.ratio[kk]
                    * (drive[kk] - self.j_rotor[kk] * self.ratio[kk] * self.pulley_angle(kk, &solved.acc));
                solved.out[kk] = x[n + kk];
            }
            for kk in 0..m {
                rows[kk] = match (rows[kk], locking[kk]) {
                    (Row::Stuck | Row::Drag(_), Some(worm)) if speed(kk) == 0.0 => {
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
                    // By the output's power: o w is below 0 while the load drives back, whatever the rotor does.
                    (_, Some(_)) => {
                        let (output, rotor) = self.rigid_factors(kk, solved.out[kk], speed(kk));
                        Row::Factors(output, rotor)
                    }
                    (_, None) => {
                        let (output, rotor) = self.rigid_factors(kk, solved.into[kk], speed(kk));
                        Row::Factors(output, rotor)
                    }
                };
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
        // The bodies move at v + e_f du/dt over ground.
        let (u, du) = self.sway(s);
        let mut ground = vq.to_vec();
        let mut frame = [0.0; 2];
        if let Some(f) = &self.frame {
            ground[f.axis] += du;
            frame = [0.5 * self.frame_mass(f) * du * du, 0.5 * f.stiffness * u * u];
        }
        let mut turning: f64 = (0..m)
            .map(|kk| {
                0.5 * self.j_pulley[kk] * s[v + n + kk].powi(2) + 0.5 * self.j_rotor[kk] * s[v + n + m + kk].powi(2)
            })
            .sum();
        for (kk, (jump, ..)) in self.jumps.iter().enumerate() {
            turning += jump.slip_energy(&s[self.jump_at(kk)..], self.r[kk] * s[v + n + kk]);
        }
        // The leaning body moves at its axis's speed over ground plus the lean's.
        let lean = self.lean.as_ref().map_or([0.0; 2], |lean| {
            let ((w, dw), body) = (self.leaning(s), self.body_mass(&self.bodies[lean.body]));
            [body * dw * (ground[lean.axis] + 0.5 * dw), 0.5 * flex_at(lean, q).0 * w * w]
        });
        let kinetic = quadratic(&self.axis_mass(q), &ground) + turning + frame[0] + lean[0];
        let (geo, mat) = (self.geo(q), self.mat(s));
        let belt: f64 = geo.iter().zip(&mat).map(|(g, l)| self.belt.energy(*g, *l)).sum();
        let gears: f64 = (0..m).map(|kk| self.gears[kk].energy(s[n + m + kk], self.output(s, kk).0)).sum();
        [kinetic, potential, stops + belt + gears + frame[1] + lean[1]]
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
                gear_loss[kk] = gear.rotor - gear.output / self.ratio[kk];
                gear_power[kk] = self.gear_loss_power(kk, phi, w_phi, theta, w_theta);
                brake[kk] = self.brakes[kk].torque(self.brake_engaged, w_phi, s[self.brake_at() + kk]).0;
                mesh_in += self.r[kk] * self.mesh_force(kk, self.r[kk] * theta) * w_theta;
            }
            let (geo, mat, (_, rate)) = (self.geo(q), self.mat(s), self.fed(s));
            for (i, sec) in self.sections.iter().enumerate() {
                let mat_rate: f64 = (0..m).map(|kk| sec.f[kk] * self.r_mat[kk] * rate[kk]).sum();
                belt += self.belt.loss_power(geo[i], mat[i], dot(&sec.c, vq), mat_rate);
            }
            if let Some(f) = &self.frame {
                tool_v[f.axis] += self.sway(s).1;
            }
            if let Some(lean) = &self.lean {
                tool_v[lean.axis] += self.tip(q) * self.leaning(s).1;
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
        powers.extend((0..m).map(|kk| (self.shafts[kk].torque(speed[kk]) + brake[kk]) * speed[kk] - drag[kk]));
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
        powers.extend([stops, ripple - mesh_in - dot(&self.process_force(), &tool_v)]);
        if !self.jumps.is_empty() {
            powers.push(self.jump_loss);
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
        // The leaning body's mass times its lean's acceleration, on its axis.
        let mut leaning = vec![0.0; n];
        let (acc, ground_acc) = if self.elastic {
            let mut d = vec![0.0; self.state_len()];
            self.elastic_derivatives(s, torque, &mut d);
            let acc = d[self.vel()..self.vel() + n].to_vec();
            let mut ground = acc.clone();
            if let Some(f) = &self.frame {
                ground[f.axis] += d[self.vel() + self.frame_at()];
            }
            if let Some(lean) = &self.lean {
                leaning[lean.axis] = self.body_mass(&self.bodies[lean.body]) * d[self.vel() + self.lean_at()];
            }
            (acc, ground)
        } else {
            let acc = self.rigid_solve(s, torque).acc;
            (acc.clone(), acc)
        };
        let (axis_mass, drive_mass, (chain, _)) =
            (self.axis_mass(q), self.drive_mass(), self.chain_inertia(vq, self.sway(s).1));
        let inertial = self.inertial(q, vq);
        let inertia: Vec<f64> = (0..n)
            .map(|i| dot(&axis_mass[i], &ground_acc) + dot(&drive_mass[i], &acc) - chain[i] + leaning[i] - inertial[i])
            .collect();
        let gravity: Vec<f64> = self.gravity(q).iter().map(|g| -g).collect();
        let (idlers, drag) = (self.idler_friction(vq).0, self.chain_drag(vq));
        let resisting: Vec<f64> = (0..n).map(|i| friction[i] - idlers[i] + drag[i]).collect();
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

    /// The lowest belt mode at a pose (elastic model): the axes (and the frame, where it sways, and the leaning body,
    /// where it leans) on the belt's stiffness with the pulleys held, K = sum_s EA / L_s c_s c_s^T (plus the frame's and
    /// the lean's springs), against their mass matrix (see `energy_parts`): (w^2, the axes', frame's and lean's motion).
    fn belt_mode(&self, pose: &[f64]) -> Option<(f64, Vec<f64>)> {
        if !self.elastic {
            return None;
        }
        let (n, geo) = (self.n(), self.geo(pose));
        let d = n + usize::from(self.frame.is_some()) + usize::from(self.lean.is_some());
        let (mut mass, mut stiffness) = (vec![vec![0.0; d]; d], vec![vec![0.0; d]; d]);
        let axis_mass = self.axis_mass(pose);
        for i in 0..n {
            mass[i][..n].copy_from_slice(&axis_mass[i]);
            for (sec, length) in self.sections.iter().zip(&geo) {
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
            // The frame's coordinate moves the bodies along its axis with it.
            for i in 0..n {
                (mass[i][n], mass[n][i]) = (axis_mass[i][f.axis], axis_mass[f.axis][i]);
            }
            mass[n][n] = axis_mass[f.axis][f.axis] + self.frame_mass(f);
            stiffness[n][n] = f.stiffness;
        }
        // The lean's coordinate moves the leaning body along its axis, and with the frame where that sways along it.
        if let Some(lean) = &self.lean {
            let (at, body) = (d - 1, self.body_mass(&self.bodies[lean.body]));
            (mass[lean.axis][at], mass[at][lean.axis], mass[at][at]) = (body, body, body);
            if self.frame.as_ref().is_some_and(|f| f.axis == lean.axis) {
                (mass[n][at], mass[at][n]) = (body, body);
            }
            stiffness[at][at] = flex_at(lean, pose).0;
        }
        lowest_mode(&mass, &stiffness)
    }

    /// Where the scene draws a pose: each axis runs from the origin along its incline in the x-z plane.
    fn scene_at(&self, q: &[f64], moves: &[bool]) -> [f64; 3] {
        let mut p = [0.0, -0.03, 0.06];
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
    /// A waypoint of a belt as a wheel in the belt's plane (x, z), its radius signed by the sense the belt runs round
    /// it, and where the 3D view puts it at the pose `q`.
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
        (Wheel { centre: [at[0], at[2]], radius }, at)
    }

    /// The belt of section `s` as the 3D view draws it, a polyline in scene coordinates: its straight runs from tangent
    /// to tangent between the wheels of its route (`geometry::contacts`), round each wheel inside the route in the
    /// wheel's sense (`geometry::arc`). Each point keeps the depth (y) of its wheel.
    fn belt_line(&self, s: usize, q: &[f64]) -> Vec<[f64; 3]> {
        let route = self.route_waypoints(s);
        let wheels: Vec<(Wheel, [f64; 3])> = route.iter().map(|&w| self.wheel(w, q)).collect();
        let at = |p: [f64; 2], y: f64| [p[0], y, p[1]];
        let mut line = Vec::new();
        for (i, pair) in wheels.windows(2).enumerate() {
            let [(a, pa), (b, pb)] = [pair[0], pair[1]];
            let [leave, meet] = contacts(a, b);
            if i > 0 {
                // Round wheel a from where the run before met it.
                let came = contacts(wheels[i - 1].0, a)[1];
                line.extend(arc(a, came, leave, ARC_PIECES).into_iter().skip(1).map(|p| at(p, pa[1])));
            } else {
                line.push(at(leave, pa[1]));
            }
            line.push(at(meet, pb[1]));
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
                Some((section, points.into_iter().map(|p| [p[0], at[1], p[1]]).collect()))
            })
            .collect()
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
    /// sway and the lean; the frame's own mass in its sway; a rotary axis's inertia turning about z; the process force on
    /// the tool. What does not move is left out (see `FrameMomentum`).
    // shortcut: the pulleys', idlers' and rotors' spin is left out (their axes are not in the netlist), and a linear
    // axis whose id is not x, y or z moves nothing; add them once the netlist places them.
    fn frame_momentum(&self, s: &[f64]) -> Option<sim_core::FrameMomentum> {
        let n = self.n();
        let along: Vec<Option<usize>> =
            self.axes.iter().map(|a| ["x", "y", "z"].iter().position(|id| *id == a.id)).collect();
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
            if let Some(k) = along[fr.axis] {
                f.linear[k] += fr.mass * du;
                f.first_moment[k] += fr.mass * u;
            }
        }
        for (b, body) in self.bodies.iter().enumerate() {
            let (mut r, mut w) = ([0.0; 3], [0.0; 3]);
            for i in (0..n).filter(|&i| body.axes[i]) {
                if let Some(k) = along[i] {
                    (r[k], w[k]) = (place[i], ground[i]);
                }
            }
            // A body that does not move along the frame's axis rides on the frame.
            if let Some(k) = self.frame.as_ref().filter(|fr| !body.axes[fr.axis]).and_then(|fr| along[fr.axis]) {
                let (u, du) = self.sway(s);
                (r[k], w[k]) = (r[k] + u, w[k] + du);
            }
            if let Some(k) = self.lean.as_ref().filter(|l| l.body == b).and_then(|lean| along[lean.axis]) {
                let (x, dx) = self.leaning(s);
                (r[k], w[k]) = (r[k] + x, w[k] + dx);
            }
            sim_components::base::add_point(&mut f, self.body_mass(body), r, w);
        }
        for i in (0..n).filter(|&i| self.axes[i].unit == "rad") {
            let turned = self.turned[i].0 + if self.payload_attached { self.turned[i].1 } else { 0.0 };
            f.angular[2] += turned * v[i];
        }
        for (i, force) in self.process_force().iter().enumerate() {
            if let Some(k) = along[i] {
                f.external[k] = *force;
            }
        }
        for (i, k) in along.iter().enumerate() {
            if let Some(k) = k {
                f.external_at[*k] = pos[i];
            }
        }
        Some(f)
    }
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
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
        let q = self.joints(tool);
        let tool = &q[..];
        state[..n].copy_from_slice(tool);
        let gravity = self.gravity(tool);
        if !self.elastic {
            // K^T (pulley torque + r mesh force) = -gravity.
            let kt: Vec<Vec<f64>> = (0..n).map(|i| (0..m).map(|kk| self.k[kk][i]).collect()).collect();
            let at_pulley = lstsq(&kt, &gravity.iter().map(|g| -g).collect::<Vec<_>>()).unwrap_or_else(|| vec![0.0; m]);
            for kk in 0..m {
                let angle = self.ratio[kk] * self.pulley_angle(kk, tool);
                let shaft = (at_pulley[kk] - self.r[kk] * self.mesh_force(kk, self.r[kk] * angle / self.ratio[kk]))
                    / self.ratio[kk];
                torque[kk] = self.ripples[kk].invert(shaft * self.held(kk), angle);
            }
            return;
        }
        // The tensions at rest: they hold every axis against gravity, sum_s c_si T_s = gravity_i, and each closed belt
        // keeps its material, sum over its sections of L_s / (1 + T_s / EA) = what it had at q = 0. Newton from the
        // pretension; the first equations are linear, the others nearly so. The frame stands at 0: nothing pulls along
        // its horizontal axis. An axis a screw, rack, linear motor or rotary drives is held by that, not by the belt.
        let geo = self.geo(tool);
        let (ea, slack) = (self.belt.ea, self.belt.slack);
        let taut = |t: f64| if slack { t.max(0.0) } else { t };
        let mut tension = vec![self.belt.pretension; self.sections.len()];
        let belt_held: Vec<usize> = (0..n).filter(|&i| !self.driven(i)).collect();
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
        let mat: Vec<f64> = geo.iter().zip(&tension).map(|(g, t)| g / (1.0 + taut(*t) / ea)).collect();
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
        // shortcut: where a gantry's yaw is left to its spring, a load on it at rest is shared in least squares, not
        // held by the spring; work the spring's deflection out if a file has one (none of the shipped do).
        let drives: Vec<usize> = (0..m).filter(|&kk| self.direct[kk].is_some()).collect();
        let rows: Vec<usize> = (0..n).filter(|&i| self.driven(i)).collect();
        let a: Vec<Vec<f64>> = rows.iter().map(|&i| drives.iter().map(|&kk| self.k[kk][i]).collect()).collect();
        let b: Vec<f64> = rows
            .iter()
            .map(|&i| self.sections.iter().zip(&tension).map(|(s, t)| s.c[i] * taut(*t)).sum::<f64>() - gravity[i])
            .collect();
        let mut held = vec![0.0; m];
        if !drives.is_empty() {
            let solved = lstsq(&a, &b).unwrap_or_else(|| vec![0.0; drives.len()]);
            drives.iter().zip(solved).for_each(|(&kk, o)| held[kk] = o);
        }
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
        }
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
        let forces: Vec<f64> = (0..self.n()).map(|i| dot(&mass[i], &ddq) - gravity[i] - inertial[i]).collect();
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
        self.belt_mode(&self.joints(tool_pose)).map(|(w2, _)| w2.sqrt() / std::f64::consts::TAU)
    }

    /// The mode of `resonance_hz`: the axes and the frame move, pulleys and rotors stand. Scaled so that the tool's
    /// largest travel along an axis is 1 m, positive (with the frame the tool moves with it, see `tool_state`).
    fn mode_shape(&self, tool_pose: &[f64]) -> Option<(f64, Vec<f64>)> {
        let q = self.joints(tool_pose);
        let (w2, x) = self.belt_mode(&q)?;
        let n = self.n();
        let mut tool = x[..self.nt()].to_vec();
        if let Some(f) = &self.frame {
            tool[f.axis] += x[n];
        }
        if let Some(lean) = &self.lean {
            tool[lean.axis] += self.tip(&q) * x[x.len() - 1];
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
            shape[self.lean_at()] = x[x.len() - 1] / largest;
        }
        Some((w2.sqrt() / std::f64::consts::TAU, shape))
    }

    /// With effects.belt_wrap: per motor the wrap at its pulley (`geometry::wrap`), from where the 3D view puts it and
    /// the wheels before and after it along the belt (`wrap_neighbours`) at this pose, in the belt's plane (x, z); NaN
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
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.brake_engaged = engaged;
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
        for kk in 0..self.jumps.len() {
            let (mut jump, feeds, takes) = self.jumps[kk].clone();
            if let Some(&wrap) = wraps.get(kk).filter(|w| w.is_finite()) {
                jump.set_wrap(wrap);
            }
            let (before, t) = (self.energy(s), self.tensions(s));
            let (at, r) = (self.jump_at(kk), self.r[kk]);
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
            for motor in &self.motors {
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
        for kk in 0..self.jumps.len() {
            out.extend([s[self.jump_at(kk) + 4], s[self.jump_at(kk)]]);
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
                    out.extend([teeth, pinion_radial_load(rack, teeth)]);
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
        for kk in 0..m {
            let at = match &self.direct[kk] {
                Some(d) => self.direct_scene(kk, d, q, out),
                None => {
                    let at = self.pulley_scene_at(kk);
                    let pulley = if self.elastic { s[n + kk] } else { rotor[kk] / self.ratio[kk] };
                    drive_unit(
                        &mut out.parts,
                        [at[0], at[2]],
                        at[1],
                        self.r[kk],
                        -pulley,
                        -rotor[kk],
                        [0.9, 0.45, 0.1],
                    );
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
            let at = self.idler_scene_at(d, q);
            idler(&mut out.parts, [at[0], at[2]], at[1], d.radius, -dot(&d.speed, q) / d.radius);
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
