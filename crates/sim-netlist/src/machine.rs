//! `ConfigMachine`: a belt-driven machine of any routing, built from a netlist (`Netlist`).
//!
//! The machine has n axes (tool coordinates q) and as many motors. Each motor turns one pulley through its gearbox.
//! Belt section s has the geometric length L_s = L0_s + sum_i c_si q_i (plus the teeth in mesh at each pulley it ends
//! at, see `Belt::tooth_length`), and pulley k feeds f_sk r_k of belt into it per radian (f = +1 or -1). So:
//!
//! * the belt pulls axis i with -sum_s c_si T_s, and turns pulley k with r_mat,k sum_s f_sk M_s (M the force on the belt
//!   material, `SectionForce::material`);
//! * in the rigid model the belt keeps its length: C q = F diag(r) a, so the pulley angles are a = K q with
//!   K = (F diag(r))^+ C, which must reproduce C exactly (else the routing is refused) and be invertible;
//! * bodies move with their axes, so the mass on axis i is the sum of the bodies that move with it; the axes are
//!   taken square to each other, so the bodies' mass matrix is diagonal. Gravity pulls axis i with m_i g sin(incline_i).
//!
//! The effects of the shipped crates, written for any routing:
//!
//! * idlers (`machine.idler`): idler d passes belt at the speed s_d . v (s_d its `speed` per axis), so its inertia
//!   J_d / r_d^2 adds m_d s_d s_d^T to the axes' mass matrix (which is then no longer diagonal), and its bearing
//!   friction torque F_d r_d pulls the axes back with F_d tanh(s_d . v / (w_s r_d)) s_d (w_s the motors' friction
//!   smoothing speed, as for the T-gantry's idlers). They carry no weight.
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

use crate::netlist::{Netlist, Waypoint};
use sim_components::catalog::motor_config;
use sim_components::mech::{Belt, CableChain, Gearbox, HardStop, HoldingBrake, JUMP_STATES, Lugre, Ripple, SectionForce, ShaftFriction, ThermalFeedback, ToothJump, ToothMesh, drive_excitations, span_detuning, span_frequency};
use sim_core::scene::{drive_unit, idler};
use sim_core::{AxisInfo, BeltSection, Config, ConfigError, ENERGY_PARTS, FRICTION_SIGNALS, FrictionContact, LOSS_PREFIX, Label, Machine, MotorInfo, Part, Scene, Shape, SignalInfo, TORQUE_PARTS, ThermalPart};

const G: f64 = 9.80665;

/// Effects the shipped machine crates have and machine files do not: switched on, a machine file is refused.
/// tool_flex is the T-gantry's Z axis leaning on its carriage: it needs where that axis's centre of mass and tool tip
/// sit below the guide and how that changes with its travel, geometry a netlist of belts and bodies does not have.
pub const UNSUPPORTED_EFFECTS: [&str; 1] = ["tool_flex"];

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
    id: &'static str,
    key: &'static str,
    section: usize,
    length: f64,
    c: Vec<f64>,
}

pub struct ConfigMachine {
    elastic: bool,
    axes: Vec<AxisInfo>,
    motors: Vec<MotorInfo>,
    /// Per axis: gravity along it (pulling towards -) and across it.
    g_along: Vec<f64>,
    g_across: Vec<f64>,
    /// Per axis: its angle above the horizontal, rad, for the scene.
    incline: Vec<f64>,
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

/// The eigenvalues and eigenvectors (columns) of a symmetric matrix, by cyclic Jacobi rotations.
fn symmetric_eigen(mut a: Vec<Vec<f64>>) -> (Vec<f64>, Vec<Vec<f64>>) {
    let d = a.len();
    let mut v: Vec<Vec<f64>> = (0..d).map(|i| (0..d).map(|j| if i == j { 1.0 } else { 0.0 }).collect()).collect();
    let scale: f64 = a.iter().flatten().map(|x| x * x).sum();
    for _ in 0..100 {
        let off: f64 = (0..d).flat_map(|i| (0..d).filter(move |&j| j != i).map(move |j| (i, j))).map(|(i, j)| a[i][j] * a[i][j]).sum();
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
    let (index, w2) = values.iter().copied().enumerate().filter(|(_, v)| *v > 1e-9 * largest).min_by(|a, b| a.1.total_cmp(&b.1))?;
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
        for effect in UNSUPPORTED_EFFECTS {
            if cfg.bool(&format!("effects.{effect}")).unwrap_or(false) {
                return Err(ConfigError::Invalid(format!("effects.{effect} (not in machine files yet)")));
            }
        }
        let (n, m) = (net.axes.len(), net.motors.len());
        let elastic = cfg.bool("effects.belt_stretch")?;
        let gravity = if cfg.bool("effects.gravity")? { G } else { 0.0 };
        let belt = Belt::from_config(cfg)?;
        // Each motor's drive train from its own config.
        let own: Vec<Config> = net.motors.iter().map(|d| motor_config(cfg, &d.id)).collect::<Result<_, _>>()?;
        let gears = own.iter().map(Gearbox::from_config).collect::<Result<Vec<_>, _>>()?;
        let j_rotor = own.iter().map(|c| Ok(c.positive("motor.rotor_inertia_kgm2")? + c.non_negative("gearbox.inertia_at_input_kgm2")?)).collect::<Result<Vec<f64>, ConfigError>>()?;
        let axes = net
            .axes
            .iter()
            .map(|a| -> Result<AxisInfo, ConfigError> {
                let limit = |name: &str| cfg.positive(&format!("axis.{}.{name}", a.id));
                Ok(AxisInfo {
                    id: intern(&a.id),
                    key: intern(&a.key),
                    unit: "m",
                    min: 0.0,
                    max: a.stroke,
                    max_vel: limit("max_velocity_m_s")?,
                    max_acc: limit("max_acceleration_m_s2")?,
                    max_jerk: limit("max_jerk_m_s3")?,
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        let incline: Vec<f64> = net.axes.iter().map(|a| a.incline_deg.to_radians()).collect();
        let teeth = belt.tooth_length;
        let sections: Vec<Section> = net
            .sections
            .iter()
            .map(|s| {
                let length = s.length + teeth * s.pulleys.iter().filter(|f| **f != 0.0).count() as f64;
                Section { id: intern(&format!("belt.{}", s.id)), key: intern(&s.key), length, bare: s.length, c: s.axes.clone(), f: s.pulleys.clone(), mat0: belt.rest_length(length), route: s.route.clone() }
            })
            .collect();
        let r: Vec<f64> = net.motors.iter().map(|d| d.radius).collect();
        // The rigid kinematics: F diag(r) K = C.
        let fr: Vec<Vec<f64>> = sections.iter().map(|s| (0..m).map(|k| s.f[k] * r[k]).collect()).collect();
        let mut k = vec![vec![0.0; n]; m];
        for i in 0..n {
            let column: Vec<f64> = sections.iter().map(|s| s.c[i]).collect();
            let solved = lstsq(&fr, &column).ok_or_else(|| ConfigError::Invalid("machine.section (the pulleys do not set every section)".into()))?;
            for kk in 0..m {
                k[kk][i] = solved[kk];
            }
        }
        for (s, row) in sections.iter().zip(&fr) {
            for i in 0..n {
                let follows: f64 = (0..m).map(|kk| row[kk] * k[kk][i]).sum();
                if (follows - s.c[i]).abs() > 1e-9 * (1.0 + s.c[i].abs()) {
                    return Err(ConfigError::Invalid(format!("machine.section[{}].axes (the belt cannot keep its length)", &s.id["belt.".len()..])));
                }
            }
        }
        let mut k_inv = vec![vec![0.0; m]; n];
        for kk in 0..m {
            let unit: Vec<f64> = (0..m).map(|j| if j == kk { 1.0 } else { 0.0 }).collect();
            let column = solve(k.clone(), unit).ok_or_else(|| ConfigError::Invalid("machine.motor (the motors do not set every axis)".into()))?;
            for i in 0..n {
                k_inv[i][kk] = column[i];
            }
        }
        // The belt runs at pulley surface speed: its mass is shared out on the driven pulleys.
        let belt_mass = belt.mass_per_m * sections.iter().map(|s| s.bare).sum::<f64>() / m as f64;
        let payload_on = cfg.bool("effects.payload")?;
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
        let smoothing = ShaftFriction::from_config(cfg)?.smoothing;
        let idlers = net
            .idlers
            .iter()
            .map(|d| Idler {
                mass: if idlers_on { d.inertia / (d.radius * d.radius) } else { 0.0 },
                drag: if idlers_on { d.friction / d.radius } else { 0.0 },
                smoothing: smoothing * d.radius,
                radius: d.radius,
                speed: d.speed.clone(),
                body: d.body,
                at: d.at,
            })
            .collect();
        let chain_axes: Vec<bool> = net.axes.iter().map(|a| a.cable_chain).collect();
        let chain = if chain_axes.contains(&true) { CableChain::from_config(cfg)? } else { CableChain { mass_per_m: 0.0, length0: 0.0, drag: 0.0 } };
        // shortcut: an axis without a guide fades its chain's drag out below 1 cm/s; give it a guide to set it.
        let chain_fade = (0..n).map(|i| guides.iter().find(|g| g.axis == i).map_or(0.01, |g| g.friction.stribeck_v)).collect();
        let process = if on("process_force")? { net.axes.iter().map(|a| cfg.f64(&format!("process.force_{}_n", a.id))).collect::<Result<Vec<_>, _>>()? } else { vec![0.0; n] };
        let frame = match (&net.frame, on("frame_compliance")? && elastic) {
            (Some(f), true) => Some(FrameMode { axis: f.axis, mass: f.mass, stiffness: f.stiffness, damping: f.damping }),
            (None, true) => return Err(ConfigError::Missing("machine.frame (effects.frame_compliance)".into())),
            (_, false) => None,
        };
        let thermal = match own.iter().map(ThermalFeedback::from_config).collect::<Result<Option<Vec<_>>, _>>()? {
            Some(laws) => Some((laws, guides.iter().map(|g| g.friction.viscous).collect())),
            None => None,
        };
        let mut jumps = Vec::new();
        if elastic && on("belt_tooth_jump")? {
            for (kk, d) in net.motors.iter().enumerate() {
                let side = |f: f64| -> Result<usize, ConfigError> {
                    let found: Vec<usize> = (0..sections.len()).filter(|&s| sections[s].f[kk] == f).collect();
                    match found[..] {
                        [s] => Ok(s),
                        _ => Err(ConfigError::Invalid(format!("effects.belt_tooth_jump (pulley {} has no section on each side)", d.pulley))),
                    }
                };
                let law = ToothJump::from_config(&own[kk], d.radius, belt_mass)?.ok_or_else(|| ConfigError::Invalid("effects.belt_tooth_jump".into()))?;
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
        let spans = net.spans.iter().map(|s| Span { id: intern(&s.id), key: intern(&format!("belt-span-{}", s.id)), section: s.section, length: s.length, c: s.axes.clone() }).collect();
        Ok(Self {
            elastic,
            g_along: incline.iter().map(|a| gravity * a.sin()).collect(),
            // cos(90 deg) is 6e-17, not 0: a vertical axis has no weight across it.
            g_across: incline.iter().map(|a| if (a.cos()).abs() < 1e-12 { 0.0 } else { gravity * a.cos() }).collect(),
            loops: closed_belts(&sections, m),
            incline,
            bodies: net.bodies.iter().map(|b| Body { mass: b.mass, payload: if payload_on { b.payload } else { 0.0 }, axes: b.axes.clone() }).collect(),
            motors: net.motors.iter().map(|d| MotorInfo { id: intern(&d.id), key: intern(&d.key) }).collect(),
            r_mat: r.iter().map(|r| belt.rest_length(*r)).collect(),
            j_pulley: net.motors.iter().map(|d| d.inertia + belt_mass * d.radius * d.radius).collect(),
            pulley_at: net.motors.iter().map(|d| d.at).collect(),
            r,
            k,
            k_inv,
            ratio: gears.iter().map(|g| g.ratio).collect(),
            shafts: own.iter().map(ShaftFriction::from_config).collect::<Result<_, _>>()?,
            ripples: own.iter().map(Ripple::from_config).collect::<Result<_, _>>()?,
            brakes: own.iter().zip(&j_rotor).map(|(c, j)| HoldingBrake::from_config(c, *j)).collect::<Result<_, _>>()?,
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
        })
    }

    fn n(&self) -> usize {
        self.axes.len()
    }
    fn m(&self) -> usize {
        self.motors.len()
    }
    // Elastic state: [q, a (pulleys), p (rotors), u (the frame, with one)], their rates, a friction state per axis, the
    // slack time, a brake state per motor, and with effects.belt_tooth_jump `JUMP_STATES` per motor.
    // Rigid state: [q, v, friction states, brakes].
    fn vel(&self) -> usize {
        if self.elastic { self.n() + 2 * self.m() + usize::from(self.frame.is_some()) } else { self.n() }
    }
    fn frame_at(&self) -> usize {
        self.n() + 2 * self.m()
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
    /// The bodies' mass on each axis.
    fn mass(&self) -> Vec<f64> {
        (0..self.n()).map(|i| self.bodies.iter().filter(|b| b.axes[i]).map(|b| self.body_mass(b)).sum()).collect()
    }
    /// The mass matrix of the axes at q: the bodies, the cable chains and the idlers.
    fn axis_mass(&self, q: &[f64]) -> Vec<Vec<f64>> {
        let n = self.n();
        let mut out = vec![vec![0.0; n]; n];
        for (i, m) in self.mass().into_iter().enumerate() {
            out[i][i] = m + if self.chain_axes[i] { self.chain.mass(q[i]).0 } else { 0.0 };
        }
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
    /// Gravity's force on each axis.
    fn gravity(&self) -> Vec<f64> {
        self.mass().iter().zip(&self.g_along).map(|(m, g)| -m * g).collect()
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
        (0..self.m()).map(|kk| (0..self.n()).map(|i| self.k_inv[i][kk] * forces[i]).sum::<f64>() / self.ratio[kk]).collect()
    }

    fn positions<'s>(&self, s: &'s [f64]) -> (&'s [f64], &'s [f64]) {
        let (n, v) = (self.n(), self.vel());
        (&s[..n], &s[v..v + n])
    }
    fn pulley_angle(&self, kk: usize, q: &[f64]) -> f64 {
        dot(&self.k[kk], q)
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
        self.sections.iter().map(|sec| sec.mat0 + (0..self.m()).map(|kk| sec.f[kk] * self.r_mat[kk] * angle[kk]).sum::<f64>()).collect()
    }
    /// The tension of each section: the belt's in the elastic model, pretension in the rigid one.
    fn tensions(&self, s: &[f64]) -> Vec<f64> {
        if self.elastic { self.sections(s).iter().map(|f| f.tension).collect() } else { vec![self.belt.pretension; self.sections.len()] }
    }

    /// The acceleration of each axis that the motor torques would give a loss-free rigid machine: what an overhang's
    /// tilt on a guide is worked out with. The true acceleration would make friction depend on itself, and its absolute
    /// value would turn every belt vibration into extra friction. Bodies, pulleys and rotors only, as the crates take it:
    /// chains and idlers change it by a few per mille.
    fn commanded_acc(&self, torque: &[f64]) -> Vec<f64> {
        let forces: Vec<f64> = (0..self.n()).map(|i| (0..self.m()).map(|kk| self.ratio[kk] * self.k[kk][i] * torque[kk]).sum()).collect();
        let mut mass = self.drive_mass();
        for (i, m) in self.mass().into_iter().enumerate() {
            mass[i][i] += m;
        }
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
        let acc = if self.guides.iter().any(|g| g.overhang.is_some()) { self.commanded_acc(torque) } else { vec![0.0; self.n()] };
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
        (0..self.n()).map(|i| if self.chain_axes[i] { self.chain.drag(v[i], self.chain_fade[i]) } else { 0.0 }).collect()
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
    fn stop_force(&self, i: usize, pos: f64, vel: f64) -> f64 {
        self.stops.force(pos, vel, self.axes[i].max)
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
        let (gravity, (idlers, _), drag) = (self.gravity(), self.idler_friction(vq), self.chain_drag(vq));
        let (u, du) = self.sway(s);
        // What the frame and the things on it put on the axes: it takes all of it back.
        let mut force: Vec<f64> = (0..n)
            .map(|i| {
                let belt: f64 = self.sections.iter().zip(&sec).map(|(x, f)| -x.c[i] * f.tension).sum();
                belt - friction[i] + idlers[i] - drag[i] + self.stop_force(i, q[i], vq[i]) + gravity[i]
            })
            .collect();
        let (chain, chain_on_frame) = self.chain_inertia(vq, du);
        let frame_acc = self.frame.as_ref().map(|f| (f.axis, (-f.stiffness * u - f.damping * du - force[f.axis] + chain_on_frame) / self.frame_mass(f)));
        let process = self.process_force();
        for i in 0..n {
            force[i] += chain[i] + process[i];
            out[fa + i] = rates[i];
        }
        let mut acc = solve(self.axis_mass(q), force).unwrap_or_else(|| vec![0.0; n]);
        if let Some((axis, aq)) = frame_acc {
            acc[axis] -= aq;
            out[v + self.frame_at()] = aq;
        }
        out[v..v + n].copy_from_slice(&acc);
        for kk in 0..m {
            let (theta, w_theta, phi, w_phi) = (s[n + kk], s[v + n + kk], s[n + m + kk], s[v + n + m + kk]);
            let gear = self.gears[kk].torque(phi, w_phi, theta, w_theta);
            let belt: f64 = self.r_mat[kk] * self.sections.iter().zip(&sec).map(|(x, f)| x.f[kk] * f.material).sum::<f64>();
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
            let (brake, brake_rate) = self.brakes[kk].torque(self.brake_engaged, w_phi, s[ba + kk]);
            out[v + n + m + kk] = (self.ripples[kk].apply(torque[kk], phi) - gear.rotor - self.shafts[kk].torque(w_phi) - brake) / self.j_rotor[kk];
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
        let (angle, speed): (Vec<f64>, Vec<f64>) = (0..m).map(|kk| (self.ratio[kk] * self.pulley_angle(kk, q), self.ratio[kk] * self.pulley_angle(kk, v))).unzip();
        // The torque each rotor passes on, before its own inertia and the gearbox; a brake holds the rotor.
        let drive: Vec<f64> = (0..m)
            .map(|kk| self.ripples[kk].apply(torque[kk], angle[kk]) - self.shafts[kk].torque(speed[kk]) - self.brakes[kk].torque(self.brake_engaged, speed[kk], s[ba + kk]).0)
            .collect();
        let (gravity, (idlers, _), drag, (chain, _), process) = (self.gravity(), self.idler_friction(v), self.chain_drag(v), self.chain_inertia(v, 0.0), self.process_force());
        let base: Vec<f64> = (0..n)
            .map(|i| {
                let mesh: f64 = (0..m).map(|kk| self.r[kk] * self.mesh.force(self.r[kk] * angle[kk] / self.ratio[kk]) * self.k[kk][i]).sum();
                mesh - friction[i] + self.stop_force(i, q[i], v[i]) + gravity[i] + idlers[i] - drag[i] + chain[i] + process[i]
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
        let (mut loss, mut solved) = (vec![1.0; m], Rigid { acc: vec![0.0; n], rates, out: vec![0.0; m], into: vec![0.0; m] });
        for _ in 0..3 {
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
                let into = self.ratio[kk] * (drive[kk] - self.j_rotor[kk] * self.ratio[kk] * self.pulley_angle(kk, &acc));
                (solved.out[kk], solved.into[kk]) = (loss[kk] * into, into);
            }
            solved.acc = acc;
            loss = (0..m).map(|kk| self.gears[kk].loss_factor(solved.into[kk], self.pulley_angle(kk, v))).collect();
        }
        solved
    }

    /// Kinetic, potential and elastic energy (see `ENERGY_PARTS`).
    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        let (n, m, v) = (self.n(), self.m(), self.vel());
        let q = &s[..n];
        let potential = -dot(&self.gravity(), q);
        let stops: f64 = (0..n).map(|i| self.stops.energy(q[i], self.axes[i].max)).sum();
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
        let mut turning: f64 = (0..m).map(|kk| 0.5 * self.j_pulley[kk] * s[v + n + kk].powi(2) + 0.5 * self.j_rotor[kk] * s[v + n + m + kk].powi(2)).sum();
        for (kk, (jump, ..)) in self.jumps.iter().enumerate() {
            turning += jump.slip_energy(&s[self.jump_at(kk)..], self.r[kk] * s[v + n + kk]);
        }
        let kinetic = quadratic(&self.axis_mass(q), &ground) + turning + frame[0];
        let (geo, mat) = (self.geo(q), self.mat(s));
        let belt: f64 = geo.iter().zip(&mat).map(|(g, l)| self.belt.energy(*g, *l)).sum();
        let gears: f64 = (0..m).map(|kk| self.gears[kk].energy(s[n + m + kk], s[n + kk])).sum();
        [kinetic, potential, stops + belt + gears + frame[1]]
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
                let (theta, w_theta, phi, w_phi) = (s[n + kk], s[v + n + kk], s[n + m + kk], s[v + n + m + kk]);
                let gear = self.gears[kk].torque(phi, w_phi, theta, w_theta);
                gear_loss[kk] = gear.rotor - gear.output / self.ratio[kk];
                gear_power[kk] = self.gears[kk].loss_power(phi, w_phi, theta, w_theta);
                brake[kk] = self.brakes[kk].torque(self.brake_engaged, w_phi, s[self.brake_at() + kk]).0;
                mesh_in += self.r[kk] * self.mesh.force(self.r[kk] * theta) * w_theta;
            }
            let (geo, mat, (_, rate)) = (self.geo(q), self.mat(s), self.fed(s));
            for (i, sec) in self.sections.iter().enumerate() {
                let mat_rate: f64 = (0..m).map(|kk| sec.f[kk] * self.r_mat[kk] * rate[kk]).sum();
                belt += self.belt.loss_power(geo[i], mat[i], dot(&sec.c, vq), mat_rate);
            }
            if let Some(f) = &self.frame {
                tool_v[f.axis] += self.sway(s).1;
            }
        } else {
            let solved = self.rigid_solve(s, torque);
            for kk in 0..m {
                brake[kk] = self.brakes[kk].torque(self.brake_engaged, speed[kk], s[self.brake_at() + kk]).0;
                gear_loss[kk] = (solved.into[kk] - solved.out[kk]) / self.ratio[kk];
                gear_power[kk] = gear_loss[kk] * speed[kk];
                mesh_in += self.mesh.force(self.r[kk] * angle[kk] / self.ratio[kk]) * self.r[kk] * self.pulley_angle(kk, vq);
            }
        }
        let mut powers: Vec<f64> = self.guides.iter().map(|g| friction[g.axis] * vq[g.axis]).collect();
        powers.extend((0..m).map(|kk| (self.shafts[kk].torque(speed[kk]) + brake[kk]) * speed[kk]));
        powers.extend(&gear_power);
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
        let ripple: f64 = (0..m).map(|kk| (torque[kk] - self.ripples[kk].apply(torque[kk], angle[kk])) * speed[kk]).sum();
        let stops: f64 = (0..n).map(|i| self.stops.loss_power(q[i], vq[i], self.axes[i].max)).sum();
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
        let (acc, ground_acc) = if self.elastic {
            let mut d = vec![0.0; self.state_len()];
            self.elastic_derivatives(s, torque, &mut d);
            let acc = d[self.vel()..self.vel() + n].to_vec();
            let mut ground = acc.clone();
            if let Some(f) = &self.frame {
                ground[f.axis] += d[self.vel() + self.frame_at()];
            }
            (acc, ground)
        } else {
            let acc = self.rigid_solve(s, torque).acc;
            (acc.clone(), acc)
        };
        let (axis_mass, drive_mass, (chain, _)) = (self.axis_mass(q), self.drive_mass(), self.chain_inertia(vq, self.sway(s).1));
        let inertia: Vec<f64> = (0..n).map(|i| dot(&axis_mass[i], &ground_acc) + dot(&drive_mass[i], &acc) - chain[i]).collect();
        let gravity: Vec<f64> = self.gravity().iter().map(|g| -g).collect();
        let (idlers, drag) = (self.idler_friction(vq).0, self.chain_drag(vq));
        let resisting: Vec<f64> = (0..n).map(|i| friction[i] - idlers[i] + drag[i]).collect();
        let [inertia, gravity, friction_at] = [inertia, gravity, resisting].map(|f| self.to_motors(&f));
        for kk in 0..m {
            let parts = [inertia[kk], gravity[kk], self.shafts[kk].torque(speed[kk]) + brake[kk] + gear_loss[kk] + friction_at[kk]];
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
            out.extend(TORQUE_PARTS.map(|part| SignalInfo::new(format!("motor.{}.torque_{part}", motor.id), motor.key, &format!("quantity-torque-{part}"), "N m")));
        }
        for g in &self.guides {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}{}", g.loss_id), self.axes[g.axis].key, "quantity-friction-power", "W"));
        }
        for motor in &self.motors {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}shaft_{}", motor.id), motor.key, "quantity-shaft-friction-power", "W"));
        }
        for (motor, gear) in self.motors.iter().zip(&self.gears) {
            let quantity = if gear.is_coupling() { "quantity-coupling-loss-power" } else { "quantity-gearbox-loss-power" };
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}gearbox_{}", motor.id), motor.key, quantity, "W"));
        }
        if !self.idlers.is_empty() {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}idlers"), "group-idler", "quantity-friction-power", "W"));
        }
        if self.chain_axes.contains(&true) {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}cable_chain"), "group-cable_chain", "quantity-friction-power", "W"));
        }
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}belt"), "group-belt", "quantity-damping-power", "W"));
        if self.frame_node {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}frame"), "frame", "quantity-damping-power", "W"));
        }
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}hard_stops"), "group-hard_stop", "quantity-damping-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}other"), "loss-other", "quantity-power-given-off", "W"));
        if !self.jumps.is_empty() {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}tooth_jump"), "group-belt", "quantity-tooth-jump-power", "W"));
        }
        out.extend(ENERGY_PARTS.map(|part| SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")));
        for g in &self.guides {
            for part in FRICTION_SIGNALS {
                let unit = match part {
                    "speed" => "m/s",
                    "load_scale" => "",
                    _ => "N",
                };
                out.push(SignalInfo::new(format!("{}.friction_{part}", g.contact), self.axes[g.axis].key, &format!("quantity-friction-{}", part.replace('_', "-")), unit));
            }
        }
        out.extend(self.sections.iter().map(|s| SignalInfo::new(format!("{}.stretch", s.id), s.key, "quantity-section-stretch", "m")));
    }

    /// The first-mode frequency of each span (`span_frequency`) at its section's tension, and how far the nearest of
    /// them is from what the drives excite (`span_detuning`).
    fn span_signals(&self, s: &[f64], mass: f64, out: &mut Vec<f64>) {
        let (n, m) = (self.n(), self.m());
        let (q, tension) = (&s[..n], self.tensions(s));
        let (mut angle, mut speed) = (vec![0.0; m], vec![0.0; m]);
        self.motor_state(s, &mut angle, &mut speed);
        let mut excitations = Vec::with_capacity(3 * m);
        for kk in 0..m {
            let pulley = if self.elastic { s[self.vel() + n + kk] } else { speed[kk] / self.ratio[kk] };
            drive_excitations(self.mesh.pitch, self.r[kk] * pulley, &self.ripples[kk], speed[kk], &mut excitations);
        }
        let frequencies: Vec<f64> = self.spans.iter().map(|p| span_frequency(p.length + dot(&p.c, q), tension[p.section], mass)).collect();
        out.extend(&frequencies);
        out.push(frequencies.iter().map(|f| span_detuning(*f, &excitations)).fold(1.0, f64::min));
    }

    /// The lowest belt mode at a pose (elastic model): the axes (and the frame, where it sways) on the belt's
    /// stiffness with the pulleys held, K = sum_s EA / L_s c_s c_s^T (plus the frame's spring), against their mass
    /// matrix (see `energy_parts`): (w^2, the axes' and the frame's motion in it).
    fn belt_mode(&self, pose: &[f64]) -> Option<(f64, Vec<f64>)> {
        if !self.elastic {
            return None;
        }
        let (n, geo) = (self.n(), self.geo(pose));
        let d = n + usize::from(self.frame.is_some());
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
        if let Some(f) = &self.frame {
            // The frame's coordinate moves the bodies along its axis with it.
            for i in 0..n {
                (mass[i][n], mass[n][i]) = (axis_mass[i][f.axis], axis_mass[f.axis][i]);
            }
            mass[n][n] = axis_mass[f.axis][f.axis] + self.frame_mass(f);
            stiffness[n][n] = f.stiffness;
        }
        lowest_mode(&mass, &stiffness)
    }

    /// Where the scene draws a pose: each axis runs from the origin along its incline in the x-z plane.
    fn scene_at(&self, q: &[f64], moves: &[bool]) -> [f64; 3] {
        let mut p = [0.0, -0.03, 0.06];
        for i in (0..self.n()).filter(|i| moves[*i]) {
            // Up by the incline, and across: the first axis along x, the others along y.
            let (up, across) = self.incline[i].sin_cos();
            p[if i == 0 { 0 } else { 1 }] += q[i] * across;
            p[2] += q[i] * up;
        }
        p
    }
    /// Where the scene draws the pulley of motor `kk`: its `at_m`, else at the ends of the first axis, two by two
    /// further down.
    fn pulley_scene_at(&self, kk: usize) -> [f64; 3] {
        let end = self.axes[0].max;
        self.pulley_at[kk].unwrap_or([if kk % 2 == 0 { -0.1 } else { end + 0.1 }, -0.05, -0.15 * (kk / 2) as f64])
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
    /// The points the 3D view draws a section's belt through: its `route`, else its pulleys and the body it is clamped
    /// to (the one that moves with most of the axes its length changes with, and the fewest others).
    fn route(&self, s: usize, q: &[f64]) -> Vec<[f64; 3]> {
        let sec = &self.sections[s];
        let mut route = sec.route.clone();
        if route.is_empty() {
            route.extend((0..self.m()).filter(|&kk| sec.f[kk] != 0.0).map(Waypoint::Pulley));
            let fits = |b: &Body| {
                let shared = (0..self.n()).filter(|&i| sec.c[i] != 0.0 && b.axes[i]).count() as i64;
                (shared, -((0..self.n()).filter(|&i| sec.c[i] == 0.0 && b.axes[i]).count() as i64))
            };
            if let Some(b) = (0..self.bodies.len()).filter(|&b| fits(&self.bodies[b]).0 > 0).max_by_key(|&b| fits(&self.bodies[b])) {
                route.insert(if sec.f.iter().position(|f| *f != 0.0).is_some_and(|kk| sec.f[kk] < 0.0) { 0 } else { route.len() }, Waypoint::Body(b));
            }
        }
        route
            .iter()
            .map(|w| match *w {
                Waypoint::Pulley(kk) => self.pulley_scene_at(kk),
                Waypoint::Idler(d) => self.idler_scene_at(&self.idlers[d], q),
                Waypoint::Body(b) => self.scene_at(q, &self.bodies[b].axes),
            })
            .collect()
    }
}

impl Machine for ConfigMachine {
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        for (kk, out) in motors.iter_mut().enumerate() {
            *out = self.ratio[kk] * self.pulley_angle(kk, tool);
        }
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        for (i, out) in tool.iter_mut().enumerate() {
            *out = (0..self.m()).map(|kk| self.k_inv[i][kk] * motors[kk] / self.ratio[kk]).sum();
        }
    }

    fn state_len(&self) -> usize {
        self.brake_at() + self.m() + JUMP_STATES * self.jumps.len()
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let (n, m) = (self.n(), self.m());
        *state = vec![0.0; self.state_len()];
        state[..n].copy_from_slice(tool);
        let gravity = self.gravity();
        if !self.elastic {
            // K^T (pulley torque + r mesh force) = -gravity.
            let kt: Vec<Vec<f64>> = (0..n).map(|i| (0..m).map(|kk| self.k[kk][i]).collect()).collect();
            let at_pulley = lstsq(&kt, &gravity.iter().map(|g| -g).collect::<Vec<_>>()).unwrap_or_else(|| vec![0.0; m]);
            for kk in 0..m {
                let angle = self.ratio[kk] * self.pulley_angle(kk, tool);
                let shaft = (at_pulley[kk] - self.r[kk] * self.mesh.force(self.r[kk] * angle / self.ratio[kk])) / self.ratio[kk];
                torque[kk] = self.ripples[kk].invert(shaft, angle);
            }
            return;
        }
        // The tensions at rest: they hold every axis against gravity, sum_s c_si T_s = gravity_i, and each closed belt
        // keeps its material, sum over its sections of L_s / (1 + T_s / EA) = what it had at q = 0. Newton from the
        // pretension; the first equations are linear, the others nearly so. The frame stands at 0: nothing pulls along
        // its horizontal axis.
        let geo = self.geo(tool);
        let (ea, slack) = (self.belt.ea, self.belt.slack);
        let taut = |t: f64| if slack { t.max(0.0) } else { t };
        let mut tension = vec![self.belt.pretension; self.sections.len()];
        for _ in 0..50 {
            let mut jac: Vec<Vec<f64>> = (0..n).map(|i| self.sections.iter().map(|s| s.c[i]).collect()).collect();
            let mut res: Vec<f64> = (0..n).map(|i| gravity[i] - dot(&jac[i], &tension)).collect();
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
        let fed: Vec<Vec<f64>> = self.sections.iter().map(|s| (0..m).map(|kk| s.f[kk] * self.r_mat[kk]).collect()).collect();
        let change: Vec<f64> = self.sections.iter().zip(&mat).map(|(s, l)| l - s.mat0).collect();
        let angles = lstsq(&fed, &change).unwrap_or_else(|| vec![0.0; m]);
        let material: Vec<f64> = geo.iter().zip(&mat).map(|(g, l)| self.belt.section(*g, *l, 0.0, 0.0).material).collect();
        for kk in 0..m {
            let theta = angles[kk];
            let belt: f64 = self.r_mat[kk] * self.sections.iter().zip(&material).map(|(s, f)| s.f[kk] * f).sum::<f64>();
            let gear = -belt - self.r[kk] * self.mesh.force(self.r[kk] * theta);
            let phi = self.ratio[kk] * (theta + self.gears[kk].static_twist(gear));
            state[n + kk] = theta;
            state[n + m + kk] = phi;
            torque[kk] = self.ripples[kk].invert(gear / self.ratio[kk], phi);
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
        pos.copy_from_slice(q);
        vel.copy_from_slice(v);
        if let Some(f) = &self.frame {
            let (u, du) = self.sway(s);
            pos[f.axis] += u;
            vel[f.axis] += du;
        }
    }
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        let (n, m, v) = (self.n(), self.m(), self.vel());
        s[v..v + n].copy_from_slice(vel);
        if self.elastic {
            for kk in 0..m {
                let w = self.pulley_angle(kk, vel);
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
            self.inverse(&s[..n], pos);
            self.inverse(&s[n..2 * n], vel);
        }
    }

    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        let mass = self.rigid_mass(pos);
        let gravity = self.gravity();
        let forces: Vec<f64> = (0..self.n()).map(|i| dot(&mass[i], acc) - gravity[i]).collect();
        out.copy_from_slice(&self.to_motors(&forces));
    }

    /// The belt pulls the bodies (with chains and idlers) with the force F their masses take, without pulleys and
    /// rotors, -C^T T = F, and each closed belt keeps its length: sum_s (T_s - P) L_s = 0 over its geometric section
    /// lengths (the elastic law to first order in T / EA).
    fn rigid_tensions(&self, pos: &[f64], acc: &[f64], out: &mut Vec<f64>) {
        let (n, mass, gravity) = (self.n(), self.axis_mass(pos), self.gravity());
        let geo = self.geo(pos);
        let mut a: Vec<Vec<f64>> = (0..n).map(|i| self.sections.iter().map(|s| s.c[i]).collect()).collect();
        let mut b: Vec<f64> = (0..n).map(|i| -(dot(&mass[i], acc) - gravity[i])).collect();
        // Each closed belt keeps its length.
        for belt in &self.loops {
            a.push((0..self.sections.len()).map(|s| if belt.contains(&s) { geo[s] } else { 0.0 }).collect());
            b.push(0.0);
        }
        let d = lstsq(&a, &b).unwrap_or_else(|| vec![0.0; self.sections.len()]);
        out.extend(d.iter().map(|d| self.belt.pretension + d));
    }

    fn energy(&self, s: &[f64]) -> f64 {
        self.energy_parts(s).iter().sum()
    }

    /// The lowest belt mode (`belt_mode`), in the elastic model.
    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        self.belt_mode(tool_pose).map(|(w2, _)| w2.sqrt() / std::f64::consts::TAU)
    }

    /// The mode of `resonance_hz`: the axes and the frame move, pulleys and rotors stand. Scaled so that the tool's
    /// largest travel along an axis is 1 m, positive (with the frame the tool moves with it, see `tool_state`).
    fn mode_shape(&self, tool_pose: &[f64]) -> Option<(f64, Vec<f64>)> {
        let (w2, x) = self.belt_mode(tool_pose)?;
        let n = self.n();
        let mut tool = x[..n].to_vec();
        if let Some(f) = &self.frame {
            tool[f.axis] += x[n];
        }
        let largest = tool.iter().copied().max_by(|a, b| a.abs().total_cmp(&b.abs())).filter(|t| t.abs() > 0.0)?;
        let mut shape = vec![0.0; self.state_len()];
        for i in 0..n {
            shape[i] = x[i] / largest;
        }
        if self.frame.is_some() {
            shape[self.frame_at()] = x[n] / largest;
        }
        Some((w2.sqrt() / std::f64::consts::TAU, shape))
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
        let v = self.vel() + self.n();
        for kk in 0..self.jumps.len() {
            let (jump, feeds, takes) = self.jumps[kk].clone();
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
        for (gear, law) in self.gears.iter_mut().zip(laws) {
            if !gear.is_coupling() {
                gear.efficiency = law.gearbox_efficiency(*next.next().unwrap_or(&0.0));
            }
        }
        for (g, base) in self.guides.iter_mut().zip(viscous) {
            g.friction.viscous = base * laws[0].viscosity_factor(*next.next().unwrap_or(&0.0));
        }
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out: Vec<SignalInfo> = self.sections.iter().map(|s| SignalInfo::new(format!("{}.tension", s.id), s.key, "quantity-tension", "N")).collect();
        for axis in &self.axes {
            out.push(SignalInfo::new(format!("axis.{}.belt_stretch", axis.id), axis.key, "quantity-belt-stretch", "m"));
            out.push(SignalInfo::new(format!("axis.{}.friction", axis.id), axis.key, "quantity-friction-force", "N"));
        }
        if self.frame_node {
            out.push(SignalInfo::new("frame.deflection", "frame", "quantity-deflection", "m"));
        }
        out.push(SignalInfo::new("belt.slack_time", "group-belt", "quantity-slack-time", "s"));
        for motor in &self.motors {
            out.push(SignalInfo::new(format!("pulley.{}.radial_load", motor.id), motor.key, "quantity-radial-load", "N"));
        }
        for g in &self.guides {
            out.push(SignalInfo::new(format!("{}.guide_load", g.contact), self.axes[g.axis].key, "quantity-guide-load", "N"));
        }
        for motor in &self.motors {
            out.push(SignalInfo::new(format!("gearbox.{}.output_torque", motor.id), motor.key, "quantity-gearbox-torque", "N m"));
        }
        if !self.jumps.is_empty() {
            for motor in &self.motors {
                out.push(SignalInfo::new(format!("pulley.{}.tooth_jumps", motor.id), motor.key, "quantity-tooth-jumps", ""));
                out.push(SignalInfo::new(format!("pulley.{}.tooth_slip", motor.id), motor.key, "quantity-tooth-slip", "m"));
            }
        }
        if self.span_mass.is_some() {
            out.extend(self.spans.iter().map(|p| SignalInfo::new(format!("belt.span.{}.frequency", p.id), p.key, "quantity-span-frequency", "Hz")));
            out.push(SignalInfo::new("belt.span_detuning", "group-belt", "quantity-span-detuning", ""));
        }
        self.effect_infos(&mut out);
        out
    }

    fn friction_contacts(&self) -> Vec<FrictionContact> {
        self.guides.iter().map(|g| FrictionContact { id: g.contact, owner_key: self.axes[g.axis].key }).collect()
    }
    fn friction_curve(&self, i: usize, speed: f64, load_scale: f64) -> [f64; 3] {
        self.guides.get(i).map_or([0.0; 3], |g| g.friction.steady(speed, load_scale))
    }

    fn belt_sections(&self) -> Vec<BeltSection> {
        self.sections.iter().map(|s| BeltSection { id: s.id, owner_key: s.key, pretension: self.belt.pretension }).collect()
    }

    /// The gearboxes (not on a direct drive, see `Gearbox::is_coupling`), then the guides, each shown by the first
    /// body it carries (or its overhang).
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        let mut parts = Vec::new();
        for (kk, motor) in self.motors.iter().enumerate().filter(|(kk, _)| !self.gears[*kk].is_coupling()) {
            parts.push(ThermalPart { id: intern(&format!("gearbox_{}", motor.id)), kind: "gearbox", owner_key: motor.key, scene_part: "gearbox", scene_index: kk });
        }
        for g in &self.guides {
            let body = g.carries.first().copied().or(g.overhang.as_ref().map(|o| o.body)).unwrap_or(0);
            parts.push(ThermalPart { id: g.loss_id, kind: "guide", owner_key: self.axes[g.axis].key, scene_part: "carriage", scene_index: body });
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
            let stretch: Vec<f64> = (0..n).map(|i| q[i] - dot(&self.k_inv[i], &s[n..n + m])).collect();
            let outputs = (0..m).map(|kk| self.gears[kk].torque(s[n + m + kk], s[self.vel() + n + m + kk], s[n + kk], s[self.vel() + n + kk]).output).collect();
            (stretch, s[self.slack_at()], outputs)
        } else {
            (vec![0.0; n], 0.0, self.rigid_solve(s, torque).out)
        };
        out.extend(&tensions);
        for i in 0..n {
            out.extend([stretch[i], friction[i]]);
        }
        if self.frame_node {
            out.push(self.sway(s).0);
        }
        out.push(slack);
        for kk in 0..m {
            out.push(self.sections.iter().zip(&tensions).filter(|(sec, _)| sec.f[kk] != 0.0).map(|(_, t)| t).sum());
        }
        let acc = if self.guides.iter().any(|g| g.overhang.is_some()) { self.commanded_acc(torque) } else { vec![0.0; n] };
        out.extend(self.guides.iter().map(|g| self.guide_load(g, q, &acc)));
        out.extend(outputs);
        for kk in 0..self.jumps.len() {
            out.extend([s[self.jump_at(kk) + 4], s[self.jump_at(kk)]]);
        }
        if let Some(mass) = self.span_mass {
            self.span_signals(s, mass, out);
        }
    }
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.view_signals(s, torque, out);
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(s, torque).0);
    }

    /// A plain scene: a beam, a box per body, a drive unit per motor at its pulley's `at_m`, the idlers, and the belt
    /// as straight strips through each section's route, shaded by its tension against the pretension.
    /// shortcut: the strips run from centre to centre, not tangent around the pulleys; upgrade with pulley wraps if the
    /// view has to show which side of a pulley a belt runs on.
    fn scene(&self, s: &[f64], out: &mut Scene) {
        use std::f64::consts::TAU;
        let (n, m) = (self.n(), self.m());
        let (q, _) = self.positions(s);
        let (mut rotor, mut rotor_speed) = (vec![0.0; m], vec![0.0; m]);
        self.motor_state(s, &mut rotor, &mut rotor_speed);
        out.parts.clear();
        out.labels.clear();
        let end = self.axes[0].max;
        out.parts.push(Part::new("beam", Shape::Box { size: [end + 0.2, 0.07, 0.09] }, [end / 2.0, 0.045, 0.0], [0.62, 0.64, 0.68]));
        for (b, body) in self.bodies.iter().enumerate() {
            let at = self.scene_at(q, &body.axes);
            let shade = 0.2 + 0.1 * b as f32;
            out.parts.push(Part::new("carriage", Shape::Box { size: [0.16, 0.05, 0.03] }, at, [shade, shade + 0.02, shade + 0.06]));
        }
        for kk in 0..m {
            let at = self.pulley_scene_at(kk);
            let pulley = if self.elastic { s[n + kk] } else { rotor[kk] / self.ratio[kk] };
            drive_unit(&mut out.parts, [at[0], at[2]], at[1], self.r[kk], -pulley, -rotor[kk], [0.9, 0.45, 0.1]);
            out.labels.push(Label { pos: [at[0], at[1], at[2] + 0.11], owner_key: self.motors[kk].key, value: rotor_speed[kk] * 60.0 / TAU, unit_key: "unit-rpm", decimals: 0 });
        }
        for d in &self.idlers {
            let at = self.idler_scene_at(d, q);
            idler(&mut out.parts, [at[0], at[2]], at[1], d.radius, -dot(&d.speed, q) / d.radius);
        }
        // A machine without idler nodes and one motor gets the idler at the far end the single axis has.
        if m == 1 && self.idlers.is_empty() {
            idler(&mut out.parts, [end + 0.1, 0.0], -0.05, self.r[0], 0.0);
        }
        let tensions = self.tensions(s);
        let pretension = self.belt.pretension.max(1.0);
        for (i, t) in tensions.iter().enumerate() {
            let shade = ((t - self.belt.pretension) / pretension).clamp(-1.0, 1.0) as f32;
            for pair in self.route(i, q).windows(2) {
                out.parts.push(Part::new("belt", Shape::Segment { to: pair[1], width: 0.02 }, pair[0], [0.12, 0.12, 0.13]).highlighted(shade));
            }
        }
        for (i, axis) in self.axes.iter().enumerate() {
            let moves: Vec<bool> = (0..n).map(|l| l <= i).collect();
            let at = self.scene_at(q, &moves);
            out.labels.push(Label { pos: [at[0], at[1], at[2] + 0.16 + 0.06 * i as f64], owner_key: axis.key, value: q[i], unit_key: "unit-m", decimals: 3 });
        }
    }

    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        Some(self.scene_at(tool_pose, &vec![true; self.n()]))
    }
}
