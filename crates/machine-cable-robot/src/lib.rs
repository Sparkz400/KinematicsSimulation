//! Cable-driven parallel robots: a platform hung in cables, each cable wound on a winch driven by a servo motor through
//! a gearbox. Two machines of the one model here:
//!
//! - `cable-planar`: a point-mass carriage in a vertical plane on 4 cables from the frame's corners; tool x, z (m).
//! - `cable-spatial`: a platform with 6 degrees of freedom on 8 cables from the corners of a box frame; tool x, y, z (m)
//!   and the orientation as a rotation vector rx, ry, rz (rad, in the base frame).
//!
//! Kinematics. Each cable leaves its winch along the axis `e_z` of a swivelling pulley of radius r at the point A
//! (`frame.exits_m`, `frame.winch_directions`), wraps it by the angle beta and runs straight from its exit point B to
//! the platform's anchor P = p + R a (`platform.anchors_m`). The pulley swivels about `e_z` into the plane of P:
//!
//! ```text
//! d = P - A, gamma = atan2(d . e_y, d . e_x), e_u = cos gamma e_x + sin gamma e_y   (e_x towards the frame's centre)
//! in the pulley's plane (u along e_u, w along e_z): centre M = (r, 0), P = (d . e_u, d . e_z)
//! m = |P - M|, l_f = sqrt(m^2 - r^2), theta_B = atan2(P_w, P_u - r) - acos(r / m), beta = (theta_B - pi) mod 2 pi
//! L = r beta + l_f, u = (B - P) / l_f (the unit vector from the anchor to the exit point); dL/dP = -u
//! ```
//!
//! Statics. A cable pulls the platform with t u at its anchor: the structure matrix A (n x m) has the columns
//! (u, b x u) restricted to the platform's degrees of freedom, b = R a, and the cables hold a wrench w when A t = w with
//! every tension between `cable.min_tension_n` and the rating `cable.breaking_load_n` / `cable.safety_factor`. Cables
//! can only pull, so the tensions are distributed by a small QP (`distribute`): those nearest `cable.reference_tension_n`
//! that hold the wrench within the band, which is the closed form t = t_ref + A^+ (w - A t_ref) wherever no cable sits
//! at a bound, and changes continuously with the pose where one does. A pose whose platform weight (with payload) no
//! tensions in the band hold is outside the wrench-feasible workspace and refused.
//! shortcut: the workspace is the static one (the weight at rest); the wrench of the cycle's accelerations is not
//! checked against the band ahead (the verdicts judge the tensions the run reached); add the dynamic workspace to the
//! planner where cycles run at the band's edge.
//!
//! Dynamics. The platform (mass, inertia R I R^T about its centre, payload a point mass there) moves by its twist
//! xi = (v, w): M xi' + c + g = A t, c = (0, w x I_w w), g = (0, 0, m g, 0, 0, 0). Each winch is a `joint::RotaryJoint`
//! in metres of cable: motor, gearbox with the drum's radius as a lever (ratio i / r rad per m), the drum's inertia as
//! a mass J / r^2 at the cable, friction in its bearings and stops at its capacity. Elastic (effects.cable_stretch),
//! the cable of free length l is a spring of potential V = EA (L - l)^2 / (2 l) while taut (none while slack): it pulls
//! the platform with t = EA (L - l) / l, of stiffness EA / l, so that the stiffness changes over the workspace, and the
//! drum with t (1 + (L - l) / (2 l)); with effects.cable_damping a damper of c / l (`cable.specific_damping_n_s`).
//! Drum and rotor are bodies of their own behind the gearbox's spring, as on every joint. Rigid, the cables do not
//! stretch: l = L(pose), the rotors turn with the drums, and the winches' inertia joins the platform's through
//! G = dL/dxi = -A^T: (M + G^T diag(J_k) G) xi' = G^T (drive + friction) - c - g - G^T diag(J_k) (G' xi).
//! shortcut: the cables' own mass and sag are left out (a few grams per metre against tensions of tens of N); add
//! them for long cables of a large robot.
//!
//! The motor angles of a pose (`inverse`) are those of the free lengths that hold the platform at rest there with the
//! tensions of the distribution, l = L / (1 + t / EA) (rigid: l = L), so that position setpoints and the torque
//! feedforward ask the same pretension of the cables. `forward` finds the pose by Gauss-Newton on the cable lengths.

use sim_components::chain::solve;
use sim_components::joint::RotaryJoint;
use sim_components::mech::{HardStop, ShaftFriction, ThermalFeedback};
use sim_components::rigid::{M3, V3, about, add, cross, dot, matmul, mul};
use sim_components::transmission::Converter;
use sim_core::{
    AxisInfo, Config, ConfigError, ENERGY_PARTS, FRICTION_SIGNALS, FrictionContact, LOSS_PREFIX, Label, Machine,
    MachineEntry, Mode, MotorInfo, Part, RatingCheck, Scene, Shape, SignalInfo, TORQUE_PARTS, ThermalPart,
};
use std::f64::consts::{PI, TAU};

const I18N: &[(&str, &str)] = &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))];

pub static PLANAR: MachineEntry = MachineEntry {
    id: "cable-planar",
    key: "machine-cable-planar",
    default_config: concat!(include_str!("../config/common.toml"), "\n", include_str!("../config/planar.toml")),
    i18n: I18N,
    build,
    suite_configs: || suite_configs(&PLANAR),
};

pub static SPATIAL: MachineEntry = MachineEntry {
    id: "cable-spatial",
    key: "machine-cable-spatial",
    default_config: concat!(include_str!("../config/common.toml"), "\n", include_str!("../config/spatial.toml")),
    i18n: I18N,
    build,
    suite_configs: || suite_configs(&SPATIAL),
};

const G: f64 = 9.80665;

/// Effects that take energy out or put it in; the standard suite runs with them off.
pub const LOSSY_EFFECTS: [&str; 8] = [
    "winch_friction",
    "motor_friction",
    "gearbox_drag",
    "gearbox_lost_motion",
    "gearbox_efficiency",
    "torque_ripple",
    "holding_brake",
    "cable_damping",
];

/// The default config with every loss switched off.
pub fn lossless_config(entry: &MachineEntry) -> Config {
    let mut cfg = entry.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    cfg.set_text("gearbox.damping_nm_s_rad = 0.0").unwrap();
    cfg.set_text("winch.stop_damping_n_s_m = 0.0").unwrap();
    cfg
}

/// Loss-free rigid and elastic models, with and without the payload. Held by constant tensions the platform is stable
/// (pulled cables stiffen it), so gravity stays on.
fn suite_configs(entry: &MachineEntry) -> Vec<Config> {
    [(false, true), (false, false), (true, true), (true, false)]
        .into_iter()
        .map(|(elastic, payload)| {
            let mut cfg = lossless_config(entry);
            cfg.set_text(&format!("effects.cable_stretch = {elastic}")).unwrap();
            cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
            cfg
        })
        .collect()
}

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    Ok(Box::new(CableRobot::new(cfg)?))
}

const MOTOR_IDS: [&str; 8] = ["1", "2", "3", "4", "5", "6", "7", "8"];
const MOTOR_KEYS: [&str; 8] = [
    "motor-winch-1",
    "motor-winch-2",
    "motor-winch-3",
    "motor-winch-4",
    "motor-winch-5",
    "motor-winch-6",
    "motor-winch-7",
    "motor-winch-8",
];
const WINCH_KEYS: [&str; 8] = ["winch-1", "winch-2", "winch-3", "winch-4", "winch-5", "winch-6", "winch-7", "winch-8"];
const WINCH_CONTACTS: [&str; 8] =
    ["winch.1", "winch.2", "winch.3", "winch.4", "winch.5", "winch.6", "winch.7", "winch.8"];
const CABLE_KEYS: [&str; 8] = ["cable-1", "cable-2", "cable-3", "cable-4", "cable-5", "cable-6", "cable-7", "cable-8"];
const GEARBOX_IDS: [&str; 8] =
    ["gearbox_1", "gearbox_2", "gearbox_3", "gearbox_4", "gearbox_5", "gearbox_6", "gearbox_7", "gearbox_8"];
const AXIS_IDS: [&str; 6] = ["x", "y", "z", "rx", "ry", "rz"];
const AXIS_KEYS: [&str; 6] = ["axis-x", "axis-y", "axis-z", "axis-rx", "axis-ry", "axis-rz"];

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: V3, s: f64) -> V3 {
    a.map(|v| v * s)
}

fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

fn transpose(m: &M3) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| m[j][i]))
}

/// The rotation of a rotation vector.
fn exp(r: V3) -> M3 {
    let angle = norm(r);
    if angle == 0.0 { about([0.0, 0.0, 1.0], 0.0) } else { about(scale(r, 1.0 / angle), angle) }
}

/// The rotation vector (axis times angle, angle up to pi) of a rotation matrix.
fn log(r: &M3) -> V3 {
    let vee = [r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]];
    let cos = ((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0).clamp(-1.0, 1.0);
    let sin = 0.5 * norm(vee);
    let angle = sin.atan2(cos);
    if angle < 1e-6 {
        return vee.map(|v| 0.5 * v * (1.0 + angle * angle / 6.0));
    }
    if angle < PI - 1e-6 {
        return vee.map(|v| v * angle / (2.0 * sin));
    }
    let i = (0..3).max_by(|&a, &b| r[a][a].total_cmp(&r[b][b])).unwrap_or(0);
    let col: V3 = std::array::from_fn(|k| r[k][i] + if k == i { 1.0 } else { 0.0 });
    let n = norm(col);
    let sign = if dot(col, vee) < 0.0 { -1.0 } else { 1.0 };
    col.map(|v| sign * v / n * angle)
}

/// The skew matrix [r]x and its square, and the angle.
fn skew(r: V3) -> (M3, M3, f64) {
    let k = [[0.0, -r[2], r[1]], [r[2], 0.0, -r[0]], [-r[1], r[0], 0.0]];
    (k, matmul(&k, &k), norm(r))
}

/// The left Jacobian of the rotation vector: w = J_l(r) r'. J_l = 1 + (1 - cos a) / a^2 [r]x + (a - sin a) / a^3 [r]x^2.
fn left_jacobian(r: V3) -> M3 {
    let (k, k2, a) = skew(r);
    let (c1, c2) = if a < 1e-4 {
        (0.5 - a * a / 24.0, 1.0 / 6.0 - a * a / 120.0)
    } else {
        ((1.0 - a.cos()) / (a * a), (a - a.sin()) / (a * a * a))
    };
    std::array::from_fn(|i| std::array::from_fn(|j| if i == j { 1.0 } else { 0.0 } + c1 * k[i][j] + c2 * k2[i][j]))
}

/// Its inverse: r' = J_l^-1 w, J_l^-1 = 1 - [r]x / 2 + (1 / a^2 - (1 + cos a) / (2 a sin a)) [r]x^2.
fn left_jacobian_inverse(r: V3) -> M3 {
    let (k, k2, a) = skew(r);
    let c2 = if a < 1e-4 { 1.0 / 12.0 + a * a / 720.0 } else { 1.0 / (a * a) - (1.0 + a.cos()) / (2.0 * a * a.sin()) };
    std::array::from_fn(|i| std::array::from_fn(|j| if i == j { 1.0 } else { 0.0 } - 0.5 * k[i][j] + c2 * k2[i][j]))
}

/// The eigenvalues of the symmetric n x n matrix `a` (row by row), by Jacobi rotations.
fn eigenvalues(mut a: Vec<f64>, n: usize) -> Vec<f64> {
    for _ in 0..100 {
        let off: f64 = (0..n * n).filter(|k| k / n != k % n).map(|k| a[k].powi(2)).sum();
        let diag: f64 = (0..n).map(|k| a[k * n + k].powi(2)).sum();
        if !(off > 1e-30 * diag) {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                if a[p * n + q] == 0.0 {
                    continue;
                }
                let theta = 0.5 * (a[q * n + q] - a[p * n + p]) / a[p * n + q];
                let t = if theta == 0.0 { 1.0 } else { theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt()) };
                let (c, s) = (1.0 / (t * t + 1.0).sqrt(), t / (t * t + 1.0).sqrt());
                for k in 0..n {
                    let (akp, akq) = (a[k * n + p], a[k * n + q]);
                    a[k * n + p] = c * akp - s * akq;
                    a[k * n + q] = s * akp + c * akq;
                }
                for k in 0..n {
                    let (apk, aqk) = (a[p * n + k], a[q * n + k]);
                    a[p * n + k] = c * apk - s * aqk;
                    a[q * n + k] = s * apk + c * aqk;
                }
            }
        }
    }
    let mut eig: Vec<f64> = (0..n).map(|k| a[k * n + k]).collect();
    eig.sort_by(f64::total_cmp);
    eig
}

/// min 1/2 t^T H t - c^T t over l <= t <= u (H symmetric positive definite, m x m, row by row): the primal
/// active-set method from t = clamp(start), finite for a strictly convex problem. Each round solves the free
/// coordinates' Newton step with the fixed ones held; a step that would cross a bound stops there and fixes it; with no
/// step left, a fixed coordinate whose gradient pulls it back inside is freed; none: done.
fn box_qp(h: &[f64], c: &[f64], l: f64, u: f64, start: &[f64]) -> Option<Vec<f64>> {
    let m = c.len();
    let mut t: Vec<f64> = start.iter().map(|v| v.clamp(l, u)).collect();
    let mut fixed: Vec<bool> = t.iter().map(|v| *v == l || *v == u).collect();
    let scale = c.iter().fold(1.0f64, |s, v| s.max(v.abs()));
    for _ in 0..20 * m + 20 {
        let grad = |t: &[f64]| -> Vec<f64> {
            (0..m).map(|i| (0..m).map(|j| h[i * m + j] * t[j]).sum::<f64>() - c[i]).collect()
        };
        let g = grad(&t);
        let free: Vec<usize> = (0..m).filter(|k| !fixed[*k]).collect();
        let f = free.len();
        let mut stationary = f == 0;
        if !stationary {
            // The free coordinates' exact minimum with the fixed ones held (a quadratic: one Newton step), as far as
            // the first bound it meets.
            let hff: Vec<f64> = (0..f * f).map(|rc| h[free[rc / f] * m + free[rc % f]]).collect();
            let step = solve(&hff, &free.iter().map(|k| -g[*k]).collect::<Vec<_>>())?;
            let (mut alpha, mut block) = (1.0f64, None);
            for (i, k) in free.iter().enumerate() {
                let limit = match step[i] {
                    s if s < 0.0 => (l - t[*k]) / s,
                    s if s > 0.0 => (u - t[*k]) / s,
                    _ => f64::INFINITY,
                };
                if limit < alpha {
                    (alpha, block) = (limit.max(0.0), Some((*k, step[i] < 0.0)));
                }
            }
            for (i, k) in free.iter().enumerate() {
                t[*k] += alpha * step[i];
            }
            match block {
                Some((k, low)) => {
                    t[k] = if low { l } else { u };
                    fixed[k] = true;
                }
                None => stationary = true,
            }
        }
        if stationary {
            // A fixed coordinate whose gradient points inside: at l it wants to grow (gradient < 0), at u to shrink.
            let g = grad(&t);
            let wrong = (0..m)
                .filter(|k| fixed[*k])
                .map(|k| (k, if t[k] == l { -g[k] } else { g[k] }))
                .filter(|(_, pull)| *pull > 1e-12 * scale)
                .max_by(|a, b| a.1.total_cmp(&b.1));
            match wrong {
                Some((k, _)) => fixed[k] = false,
                None => return Some(t),
            }
        }
    }
    None
}

/// The penalised distribution: min |t - t_ref|^2 / 2 + RHO |A t - w|^2 / 2 within [t_min, t_max], defined for every
/// wrench and continuous in it (a strictly convex problem over a box). Inside the wrench-feasible workspace it misses
/// the wrench by about |t - t_ref| / RHO, 1e-4 N; what the motors' setpoints are taken from, so that `forward` finds
/// a pose from any motor angles.
fn penalised(a: &[f64], w: &[f64], t_min: f64, t_max: f64, t_ref: f64) -> Option<Vec<f64>> {
    const RHO: f64 = 1e6;
    let n = w.len();
    let m = a.len() / n;
    let h: Vec<f64> = (0..m * m)
        .map(|rc| if rc / m == rc % m { 1.0 } else { 0.0 } + RHO * (0..n).map(|r| a[r * m + rc / m] * a[r * m + rc % m]).sum::<f64>())
        .collect();
    let c: Vec<f64> = (0..m).map(|k| t_ref + RHO * (0..n).map(|r| a[r * m + k] * w[r]).sum::<f64>()).collect();
    box_qp(&h, &c, t_min, t_max, &vec![t_ref; m])
}

/// The tension distribution: the tensions t (m) nearest t_ref that hold the wrench, A t = w for the structure matrix
/// `a` (n x m, row by row), each within [t_min, t_max]; `None` where there are none (outside the wrench-feasible
/// workspace). A small QP, min |t - t_ref|^2 subject to A t = w and the band: the penalised problem (`penalised`) finds
/// which cables sit at a bound, and the free ones are then solved exactly, t_F = t_ref + A_F^T (A_F A_F^T)^-1 (w - A_B
/// t_B - A_F t_ref). Where no cable is at a bound that is the closed form t_ref + A^+ (w - A t_ref); unlike the improved
/// closed-form method, which fixes one cable at a time and never frees it, the QP's tensions change continuously with
/// the pose wherever they exist.
pub fn distribute(a: &[f64], w: &[f64], t_min: f64, t_max: f64, t_ref: f64) -> Option<Vec<f64>> {
    let n = w.len();
    let m = a.len() / n;
    let mut t = penalised(a, w, t_min, t_max, t_ref)?;
    let free: Vec<usize> = (0..m).filter(|k| t[*k] > t_min && t[*k] < t_max).collect();
    let f = free.len();
    if f >= n {
        let rest: Vec<f64> = (0..n)
            .map(|r| w[r] - (0..m).filter(|k| !free.contains(k)).map(|k| a[r * m + k] * t[k]).sum::<f64>())
            .collect();
        let af = |r: usize, k: usize| a[r * m + free[k]];
        let aat: Vec<f64> = (0..n * n).map(|rc| (0..f).map(|k| af(rc / n, k) * af(rc % n, k)).sum()).collect();
        let miss: Vec<f64> = (0..n).map(|r| rest[r] - (0..f).map(|k| af(r, k) * t_ref).sum::<f64>()).collect();
        let y = solve(&aat, &miss)?;
        for k in 0..f {
            t[free[k]] = t_ref + (0..n).map(|r| af(r, k) * y[r]).sum::<f64>();
        }
    }
    let band = 1e-9 * t_max.abs().clamp(1.0, 1e12);
    let off = (0..n).map(|r| ((0..m).map(|k| a[r * m + k] * t[k]).sum::<f64>() - w[r]).abs()).fold(0.0, f64::max);
    let scale = w.iter().fold(t_max.abs().clamp(1.0, 1e12), |s, v| s.max(v.abs()));
    let inside = t.iter().all(|v| *v >= t_min - band && *v <= t_max + band);
    (inside && off <= 1e-9 * scale && t.iter().all(|v| v.is_finite())).then_some(t)
}

/// One cable's pulley and anchor.
#[derive(Debug, Clone)]
struct Cable {
    /// Where the cable leaves the winch's line onto the pulley, the pulley's swivel axis (towards the winch), and the
    /// pulley's direction at rest (towards the frame's centre, square to the axis).
    exit: V3,
    axis: V3,
    ref_x: V3,
    /// On the platform, in its frame.
    anchor: V3,
}

/// One cable at a pose: its length from the pulley's swivel point to the anchor (arc and straight part), the straight
/// part, the unit vector from the anchor to the exit point, the exit point, and the anchor's offset b = R a.
#[derive(Debug, Clone, Copy)]
pub struct Span {
    pub length: f64,
    pub straight: f64,
    pub u: V3,
    pub exit: V3,
    pub b: V3,
}

/// The pulley's geometry for an anchor at `p` (world).
fn span(c: &Cable, r: f64, p: V3) -> Option<(f64, f64, V3, V3)> {
    let d = sub(p, c.exit);
    let ey = cross(c.axis, c.ref_x);
    let gamma = dot(d, ey).atan2(dot(d, c.ref_x));
    let eu = add(scale(c.ref_x, gamma.cos()), scale(ey, gamma.sin()));
    let (pu, pw) = (dot(d, eu), dot(d, c.axis));
    let m = (pu - r).hypot(pw);
    if !(m > r * (1.0 + 1e-9)) || !(m > 1e-9) {
        return None;
    }
    let straight = (m * m - r * r).sqrt();
    let theta = pw.atan2(pu - r) - (r / m).acos();
    let beta = (theta - PI).rem_euclid(TAU);
    let exit = add(c.exit, add(scale(eu, r + r * theta.cos()), scale(c.axis, r * theta.sin())));
    Some((r * beta + straight, straight, scale(sub(exit, p), 1.0 / straight), exit))
}

pub struct CableRobot {
    spatial: bool,
    /// The platform's degrees of freedom among (x, y, z, wx, wy, wz).
    dofs: Vec<usize>,
    cables: Vec<Cable>,
    pulley: f64,
    ea: f64,
    damping: f64,
    t_min: f64,
    t_max: f64,
    t_ref: f64,
    mass: f64,
    payload: f64,
    attached: bool,
    inertia: M3,
    /// The anchors' largest distance from the centre: the length that weighs turns against travels.
    size: f64,
    g: f64,
    elastic: bool,
    winches: Vec<RotaryJoint>,
    drum_mass: f64,
    drum_radius: f64,
    thermal: Option<ThermalFeedback>,
    brake_engaged: bool,
    axes: Vec<AxisInfo>,
    motors: Vec<MotorInfo>,
    /// The pose Gauss-Newton starts from: the centre of the tool's box.
    home: Vec<f64>,
    frame: Vec<V3>,
}

/// Per cable of one moment: tension on the platform, on the drum, and the damper's share of it.
struct Tensions {
    platform: Vec<f64>,
    drum: Vec<f64>,
    damper: Vec<f64>,
}

impl CableRobot {
    pub fn new(cfg: &Config) -> Result<Self, ConfigError> {
        let spatial = cfg.bool("platform.spatial")?;
        let dofs: Vec<usize> = if spatial { (0..6).collect() } else { vec![0, 2] };
        let n = dofs.len();
        let rows = |key: &str| -> Result<Vec<V3>, ConfigError> {
            let invalid = || ConfigError::Invalid(key.into());
            cfg.array(key)?
                .iter()
                .map(|v| {
                    let row: Vec<f64> = v.as_array().ok_or_else(invalid)?.iter().filter_map(|x| x.as_float()).collect();
                    (row.len() == 3).then(|| [row[0], row[1], row[2]]).ok_or_else(invalid)
                })
                .collect()
        };
        let exits = rows("frame.exits_m")?;
        let directions = rows("frame.winch_directions")?;
        let m = exits.len();
        if !(n + 1..=8).contains(&m) || directions.len() != m {
            return Err(ConfigError::Invalid("frame.exits_m".into()));
        }
        let anchors = if spatial { rows("platform.anchors_m")? } else { vec![[0.0; 3]; m] };
        if anchors.len() != m {
            return Err(ConfigError::Invalid("platform.anchors_m".into()));
        }
        let centre = scale(exits.iter().fold([0.0; 3], |s, e| add(s, *e)), 1.0 / m as f64);
        let mut cables = Vec::new();
        for k in 0..m {
            let axis = scale(directions[k], 1.0 / norm(directions[k]));
            let inward = sub(centre, exits[k]);
            let flat = sub(inward, scale(axis, dot(inward, axis)));
            if !(norm(flat) > 1e-9) || !axis.iter().all(|v| v.is_finite()) {
                return Err(ConfigError::Invalid("frame.winch_directions".into()));
            }
            cables.push(Cable { exit: exits[k], axis, ref_x: scale(flat, 1.0 / norm(flat)), anchor: anchors[k] });
        }
        let t_min = cfg.non_negative("cable.min_tension_n")?;
        let t_max = cfg.positive("cable.breaking_load_n")? / cfg.positive("cable.safety_factor")?;
        let t_ref = cfg.positive("cable.reference_tension_n")?;
        if !(t_min < t_ref && t_ref < t_max) {
            return Err(ConfigError::Invalid("cable.reference_tension_n".into()));
        }
        let diag = if spatial { cfg.f64s("platform.inertia_kgm2")? } else { vec![1.0; 3] };
        if diag.len() != 3 || diag.iter().any(|v| !(*v > 0.0)) {
            return Err(ConfigError::Invalid("platform.inertia_kgm2".into()));
        }
        let inertia = std::array::from_fn(|i| std::array::from_fn(|j| if i == j { diag[i] } else { 0.0 }));
        // Each winch: the drum's radius as the lever behind the gearbox, no give of its own (the cable's is apart).
        let drum_radius = cfg.positive("winch.drum_radius_m")?;
        let on = cfg.bool("effects.winch_friction")?;
        let friction = ShaftFriction {
            coulomb: if on { cfg.non_negative("winch.friction_coulomb_n")? } else { 0.0 },
            viscous: if on { cfg.non_negative("winch.friction_viscous_n_s_m")? } else { 0.0 },
            smoothing: cfg.positive("winch.friction_smoothing_m_s")?,
            ..ShaftFriction::NONE
        };
        let stops = if cfg.bool("effects.hard_stops")? {
            HardStop {
                stiffness: cfg.positive("winch.stop_stiffness_n_m")?,
                damping: cfg.non_negative("winch.stop_damping_n_s_m")?,
            }
        } else {
            HardStop { stiffness: 0.0, damping: 0.0 }
        };
        let mut winch = RotaryJoint::with(cfg, friction, stops, (0.0, cfg.positive("winch.capacity_m")?))?;
        Converter { lever: drum_radius, stiffness: f64::INFINITY, backlash: 0.0, efficiency: 1.0, inertia: 0.0 }
            .apply(cfg, &mut winch)?;
        let (min, max) = (cfg.f64s("workspace.tool_min")?, cfg.f64s("workspace.tool_max")?);
        if min.len() != n || max.len() != n || (0..n).any(|k| !(min[k] < max[k])) {
            return Err(ConfigError::Invalid("workspace.tool_max".into()));
        }
        let axes = (0..n)
            .map(|k| {
                let id = AXIS_IDS[dofs[k]];
                let unit = if dofs[k] < 3 { "m" } else { "rad" };
                let get = |what: &str, rate: &str| cfg.positive(&format!("axis.{id}.max_{what}_{unit}_{rate}"));
                Ok(AxisInfo {
                    id,
                    key: AXIS_KEYS[dofs[k]],
                    unit,
                    min: min[k],
                    max: max[k],
                    max_vel: get("velocity", "s")?,
                    max_acc: get("acceleration", "s2")?,
                    max_jerk: get("jerk", "s3")?,
                })
            })
            .collect::<Result<Vec<_>, ConfigError>>()?;
        let mass = cfg.positive("platform.mass_kg")?;
        let payload = if cfg.bool("effects.payload")? { cfg.non_negative("platform.payload_kg")? } else { 0.0 };
        let robot = Self {
            spatial,
            size: anchors.iter().map(|a| norm(*a)).fold(0.0, f64::max).max(0.1),
            dofs,
            cables,
            pulley: cfg.positive("pulley.radius_m")?,
            ea: cfg.positive("cable.ea_n")?,
            damping: if cfg.bool("effects.cable_damping")? {
                cfg.non_negative("cable.specific_damping_n_s")?
            } else {
                0.0
            },
            t_min,
            t_max,
            t_ref,
            mass,
            payload,
            attached: true,
            inertia,
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            elastic: cfg.bool("effects.cable_stretch")?,
            winches: vec![winch; m],
            drum_mass: cfg.positive("winch.drum_inertia_kgm2")? / (drum_radius * drum_radius),
            drum_radius,
            thermal: ThermalFeedback::from_config(cfg)?,
            brake_engaged: false,
            axes,
            motors: (0..m).map(|k| MotorInfo { id: MOTOR_IDS[k], key: MOTOR_KEYS[k] }).collect(),
            home: (0..n).map(|k| 0.5 * (min[k] + max[k])).collect(),
            frame: exits,
        };
        let mut torque = vec![0.0; m];
        robot.equilibrium(&robot.home, &mut Vec::new(), &mut torque);
        if !torque.iter().all(|t| t.is_finite()) {
            return Err(ConfigError::Invalid("workspace.tool_min".into()));
        }
        Ok(robot)
    }

    fn n(&self) -> usize {
        self.dofs.len()
    }
    fn m(&self) -> usize {
        self.cables.len()
    }
    fn moving(&self) -> f64 {
        self.mass + if self.attached { self.payload } else { 0.0 }
    }

    /// Position and rotation of a pose.
    fn place(&self, pose: &[f64]) -> (V3, M3) {
        let mut full = [0.0; 6];
        for (k, d) in self.dofs.iter().enumerate() {
            full[*d] = pose[k];
        }
        ([full[0], full[1], full[2]], exp([full[3], full[4], full[5]]))
    }

    /// The pose moved by a twist step `step` (in the degrees of freedom): p + v, exp(w) R.
    fn moved(&self, pose: &[f64], step: &[f64]) -> Vec<f64> {
        if !self.spatial {
            return pose.iter().zip(step).map(|(a, b)| a + b).collect();
        }
        let r = log(&matmul(&exp([step[3], step[4], step[5]]), &exp([pose[3], pose[4], pose[5]])));
        vec![pose[0] + step[0], pose[1] + step[1], pose[2] + step[2], r[0], r[1], r[2]]
    }

    /// The full twist (v, w) of one in the degrees of freedom.
    fn full(&self, xi: &[f64]) -> (V3, V3) {
        let mut f = [0.0; 6];
        for (k, d) in self.dofs.iter().enumerate() {
            f[*d] = xi[k];
        }
        ([f[0], f[1], f[2]], [f[3], f[4], f[5]])
    }

    /// The cables at a pose; `None` where one cannot be drawn (the anchor at its pulley).
    pub fn spans(&self, pose: &[f64]) -> Option<Vec<Span>> {
        let (p, r) = self.place(pose);
        self.cables
            .iter()
            .map(|c| {
                let b = mul(&r, c.anchor);
                let (length, straight, u, exit) = span(c, self.pulley, add(p, b))?;
                Some(Span { length, straight, u, exit, b })
            })
            .collect()
    }

    /// The structure matrix A (n x m, row by row): column k is cable k's wrench per newton, (u, b x u) in the degrees
    /// of freedom.
    pub fn structure(&self, spans: &[Span]) -> Vec<f64> {
        let (n, m) = (self.n(), self.m());
        let mut a = vec![0.0; n * m];
        for (k, s) in spans.iter().enumerate() {
            let moment = cross(s.b, s.u);
            let full = [s.u[0], s.u[1], s.u[2], moment[0], moment[1], moment[2]];
            for (r, d) in self.dofs.iter().enumerate() {
                a[r * m + k] = full[*d];
            }
        }
        a
    }

    /// The platform's mass matrix over its twist, in the degrees of freedom (n x n).
    fn platform_mass(&self, pose: &[f64]) -> Vec<f64> {
        let (_, r) = self.place(pose);
        let iw = matmul(&matmul(&r, &self.inertia), &transpose(&r));
        let n = self.n();
        let mut out = vec![0.0; n * n];
        for (i, di) in self.dofs.iter().enumerate() {
            for (j, dj) in self.dofs.iter().enumerate() {
                out[i * n + j] = match (*di < 3, *dj < 3) {
                    (true, true) if di == dj => self.moving(),
                    (false, false) => iw[di - 3][dj - 3],
                    _ => 0.0,
                };
            }
        }
        out
    }

    /// What the cables have to give the platform besides its acceleration: c + g = (0, 0, m g, w x I_w w).
    fn platform_bias(&self, pose: &[f64], xi: &[f64]) -> Vec<f64> {
        let (_, r) = self.place(pose);
        let (_, w) = self.full(xi);
        let iw = matmul(&matmul(&r, &self.inertia), &transpose(&r));
        let gyro = cross(w, mul(&iw, w));
        let full = [0.0, 0.0, self.moving() * self.g, gyro[0], gyro[1], gyro[2]];
        self.dofs.iter().map(|d| full[*d]).collect()
    }

    /// The static wrench at a pose with the payload on, for the workspace: (0, 0, (m + payload) g).
    fn weight(&self) -> Vec<f64> {
        let full = [0.0, 0.0, (self.mass + self.payload) * self.g, 0.0, 0.0, 0.0];
        self.dofs.iter().map(|d| full[*d]).collect()
    }

    /// The tensions that hold the platform at rest at a pose, by the distribution; `None` outside the wrench-feasible
    /// workspace.
    pub fn static_tensions(&self, pose: &[f64]) -> Option<Vec<f64>> {
        let spans = self.spans(pose)?;
        let w = self.platform_bias(pose, &vec![0.0; self.n()]);
        distribute(&self.structure(&spans), &w, self.t_min, self.t_max, self.t_ref)
    }

    /// The tensions the motors' angles count on at rest at a pose: the closed form t_ref + A^+ (w - A t_ref) without
    /// the band, smooth wherever the cables can be drawn, so that `forward` finds the pose again; the distribution's
    /// own wherever no cable sits at a bound of its band.
    /// shortcut: where the distribution holds a cable at a bound (near the workspace's edge), the setpoints count on
    /// the closed form's tensions and the feedforward on the distribution's, and each cable's stretch differs by
    /// (t_qp - t) L / EA; the position loops make up for it. Compensate with the distribution itself once `forward` can
    /// follow its kinks.
    fn rest_tensions(&self, spans: &[Span], pose: &[f64]) -> Option<Vec<f64>> {
        let w = self.platform_bias(pose, &vec![0.0; self.n()]);
        let a = self.structure(spans);
        let (n, m) = (self.n(), self.m());
        let aat: Vec<f64> = (0..n * n).map(|rc| (0..m).map(|k| a[rc / n * m + k] * a[rc % n * m + k]).sum()).collect();
        let miss: Vec<f64> = (0..n).map(|r| w[r] - (0..m).map(|k| a[r * m + k] * self.t_ref).sum::<f64>()).collect();
        let y = solve(&aat, &miss)?;
        Some((0..m).map(|k| self.t_ref + (0..n).map(|r| a[r * m + k] * y[r]).sum::<f64>()).collect())
    }

    /// The cables' free lengths that the motors' angles stand for at a pose, l = L / (1 + t / EA) at `rest_tensions`
    /// where the cables stretch, l = L where they do not.
    fn rest_lengths(&self, pose: &[f64]) -> Option<Vec<f64>> {
        let spans = self.spans(pose)?;
        if !self.elastic {
            return Some(spans.iter().map(|s| s.length).collect());
        }
        let t = self.rest_tensions(&spans, pose)?;
        Some(spans.iter().zip(&t).map(|(s, t)| s.length / (1.0 + t / self.ea)).collect())
    }

    /// `rest_lengths` inside the wrench-feasible workspace, `None` outside it.
    fn free_lengths(&self, pose: &[f64]) -> Option<Vec<f64>> {
        self.static_tensions(pose)?;
        self.rest_lengths(pose)
    }

    /// The pose whose cables are `lengths` long (pulley to anchor): Gauss-Newton through G = -A^T from `start`, no step
    /// longer than 50 mm or 50 mrad, until a step is below 1e-14; `None` where it does not settle within 100 steps.
    /// Lengths that no pose fits exactly give the pose that fits them best.
    fn pose_of_lengths(&self, lengths: &[f64], start: &[f64]) -> Option<Vec<f64>> {
        let (n, m) = (self.n(), self.m());
        let mut pose = start.to_vec();
        for _ in 0..100 {
            let spans = self.spans(&pose)?;
            let a = self.structure(&spans);
            // G^T G step = G^T miss with G = -A^T: (A A^T) step = -A miss.
            let aat: Vec<f64> =
                (0..n * n).map(|rc| (0..m).map(|k| a[rc / n * m + k] * a[rc % n * m + k]).sum()).collect();
            let rhs: Vec<f64> =
                (0..n).map(|r| -(0..m).map(|k| a[r * m + k] * (lengths[k] - spans[k].length)).sum::<f64>()).collect();
            let mut step = solve(&aat, &rhs)?;
            let longest = step.iter().fold(0.0f64, |s, v| s.max(v.abs()));
            if longest > 0.05 {
                step.iter_mut().for_each(|v| *v *= 0.05 / longest);
            }
            pose = self.moved(&pose, &step);
            if longest < 1e-14 {
                return Some(pose);
            }
        }
        None
    }

    /// G' xi: the cables' length acceleration at the twist `xi` with no platform acceleration, per cable:
    /// -u . (w x (w x b)) + w_P^T H w_P, w_P = v + w x b the anchor's speed and H = -du/dP the Hessian of the length
    /// over the anchor's place (by central differences of u, 1 um).
    fn length_curvature(&self, spans: &[Span], pose: &[f64], xi: &[f64]) -> Vec<f64> {
        let (p, _) = self.place(pose);
        let (v, w) = self.full(xi);
        spans
            .iter()
            .zip(&self.cables)
            .map(|(s, c)| {
                let wp = add(v, cross(w, s.b));
                let anchor = add(p, s.b);
                let h = 1e-6;
                let mut hw = [0.0; 3];
                for i in 0..3 {
                    let mut e = [0.0; 3];
                    e[i] = h;
                    let up = span(c, self.pulley, add(anchor, e)).map_or([f64::NAN; 3], |s| s.2);
                    let down = span(c, self.pulley, sub(anchor, e)).map_or([f64::NAN; 3], |s| s.2);
                    for (j, out) in hw.iter_mut().enumerate() {
                        *out -= (up[j] - down[j]) / (2.0 * h) * wp[i];
                    }
                }
                -dot(s.u, cross(w, cross(w, s.b))) + dot(wp, hw)
            })
            .collect()
    }

    /// Pose and twist of a state.
    fn platform<'a>(&self, s: &'a [f64]) -> (&'a [f64], &'a [f64]) {
        let n = self.n();
        (&s[..n], &s[n..2 * n])
    }

    /// Free lengths and their rates, rotor angles and speeds of a state.
    fn winch_state(&self, s: &[f64], spans: &[Span]) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
        let (n, m) = (self.n(), self.m());
        if self.elastic {
            let at = |k: usize| s[2 * n + k * m..2 * n + (k + 1) * m].to_vec();
            return (at(0), at(1), at(2), at(3));
        }
        let (_, xi) = self.platform(s);
        let rates = self.length_rates(spans, xi);
        let lengths: Vec<f64> = spans.iter().map(|s| s.length).collect();
        let ratio = |k: usize| self.winches[k].ratio();
        let rotor = (0..m).map(|k| ratio(k) * lengths[k]).collect();
        let speed = (0..m).map(|k| ratio(k) * rates[k]).collect();
        (lengths, rates, rotor, speed)
    }

    fn brakes<'a>(&self, s: &'a [f64]) -> &'a [f64] {
        let (n, m) = (self.n(), self.m());
        let at = if self.elastic { 2 * n + 4 * m } else { 2 * n };
        &s[at..at + m]
    }

    /// L' = G xi = -A^T xi.
    fn length_rates(&self, spans: &[Span], xi: &[f64]) -> Vec<f64> {
        let (n, m) = (self.n(), self.m());
        let a = self.structure(spans);
        (0..m).map(|k| -(0..n).map(|r| a[r * m + k] * xi[r]).sum::<f64>()).collect()
    }

    /// The elastic cables' tensions at lengths L (rates L') and free lengths l (rates l').
    fn tensions(&self, spans: &[Span], rates: &[f64], free: &[f64], free_rates: &[f64]) -> Tensions {
        let m = self.m();
        let mut out = Tensions { platform: vec![0.0; m], drum: vec![0.0; m], damper: vec![0.0; m] };
        for k in 0..m {
            let e = spans[k].length - free[k];
            if e > 0.0 {
                let spring = self.ea * e / free[k];
                let total = (spring + self.damping / free[k] * (rates[k] - free_rates[k])).max(0.0);
                out.damper[k] = total - spring;
                out.platform[k] = total;
                out.drum[k] = spring * (1.0 + e / (2.0 * free[k])) + out.damper[k];
            }
        }
        out
    }

    /// Rigid model: the platform's acceleration (twist rate), the cables' length accelerations, and per winch the
    /// force into its gearbox (after the rotor's acceleration) and out of it, at the cable (N).
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
        let (n, m) = (self.n(), self.m());
        let (pose, xi) = self.platform(s);
        let nan = || (vec![f64::NAN; n], vec![f64::NAN; m], vec![f64::NAN; m], vec![f64::NAN; m]);
        let Some(spans) = self.spans(pose) else { return nan() };
        let (lengths, rates, rotor, speed) = self.winch_state(s, &spans);
        let brakes = self.brakes(s);
        let a = self.structure(&spans);
        let curve = self.length_curvature(&spans, pose, xi);
        let mass = self.platform_mass(pose);
        let bias = self.platform_bias(pose, xi);
        let drive: Vec<f64> = (0..m)
            .map(|k| self.winches[k].rigid_drive(torque[k], rotor[k], speed[k], brakes[k], self.brake_engaged).0)
            .collect();
        let out: Vec<f64> = (0..m).map(|k| self.winches[k].output_torque(lengths[k], rates[k])).collect();
        // G = -A^T: G^T x = -A x.
        let solve_with = |loss: &[f64]| -> Vec<f64> {
            let inertia: Vec<f64> =
                (0..m).map(|k| loss[k] * self.winches[k].reflected_inertia() + self.drum_mass).collect();
            let mut mm = mass.clone();
            for i in 0..n {
                for j in 0..n {
                    mm[i * n + j] += (0..m).map(|k| a[i * m + k] * inertia[k] * a[j * m + k]).sum::<f64>();
                }
            }
            let rhs: Vec<f64> = (0..n)
                .map(|i| {
                    -(0..m).map(|k| a[i * m + k] * (loss[k] * drive[k] + out[k] - inertia[k] * curve[k])).sum::<f64>()
                        - bias[i]
                })
                .collect();
            solve(&mm, &rhs).unwrap_or_else(|| vec![f64::NAN; n])
        };
        let length_acc = |acc: &[f64]| -> Vec<f64> {
            (0..m).map(|k| curve[k] - (0..n).map(|r| a[r * m + k] * acc[r]).sum::<f64>()).collect()
        };
        let first = length_acc(&solve_with(&vec![1.0; m]));
        let loss: Vec<f64> = (0..m)
            .map(|k| {
                let w = &self.winches[k];
                w.gearbox.loss_factor(drive[k] - w.reflected_inertia() * first[k], rates[k])
            })
            .collect();
        let acc = solve_with(&loss);
        let lacc = length_acc(&acc);
        let into: Vec<f64> = (0..m).map(|k| drive[k] - self.winches[k].reflected_inertia() * lacc[k]).collect();
        let outs = (0..m).map(|k| loss[k] * into[k]).collect();
        (acc, lacc, into, outs)
    }

    /// The tensions of either model at a state: the springs' where the cables stretch; where they do not, what the drums
    /// hold against the gearboxes and friction.
    fn state_tensions(&self, s: &[f64], torque: &[f64]) -> Vec<f64> {
        let (pose, xi) = self.platform(s);
        let Some(spans) = self.spans(pose) else { return vec![f64::NAN; self.m()] };
        if self.elastic {
            let (free, free_rates, ..) = self.winch_state(s, &spans);
            return self.tensions(&spans, &self.length_rates(&spans, xi), &free, &free_rates).platform;
        }
        let (lengths, rates, ..) = self.winch_state(s, &spans);
        let (_, lacc, _, out) = self.rigid_solve(s, torque);
        (0..self.m())
            .map(|k| self.drum_mass * lacc[k] - out[k] - self.winches[k].output_torque(lengths[k], rates[k]))
            .collect()
    }

    /// The platform's twist rate, in either model.
    fn twist_acceleration(&self, s: &[f64], torque: &[f64]) -> Vec<f64> {
        let n = self.n();
        if self.elastic {
            let mut d = vec![0.0; self.state_len()];
            self.derivatives(s, torque, &mut d);
            d[n..2 * n].to_vec()
        } else {
            self.rigid_solve(s, torque).0
        }
    }

    /// Pose rate from twist: p' = v, r' = J_l(r)^-1 w.
    fn pose_rate(&self, pose: &[f64], xi: &[f64]) -> Vec<f64> {
        if !self.spatial {
            return xi.to_vec();
        }
        let r = mul(&left_jacobian_inverse([pose[3], pose[4], pose[5]]), [xi[3], xi[4], xi[5]]);
        vec![xi[0], xi[1], xi[2], r[0], r[1], r[2]]
    }

    /// Twist from pose rate: w = J_l(r) r'.
    fn twist(&self, pose: &[f64], rate: &[f64]) -> Vec<f64> {
        if !self.spatial {
            return rate.to_vec();
        }
        let w = mul(&left_jacobian([pose[3], pose[4], pose[5]]), [rate[3], rate[4], rate[5]]);
        vec![rate[0], rate[1], rate[2], w[0], w[1], w[2]]
    }

    /// The cables' stiffness EA / l alone, summed over the cables as K = sum EA / l_k a_k a_k^T (n x n, in the degrees
    /// of freedom), at the free lengths of the platform held at rest at `pose`.
    pub fn cable_stiffness(&self, pose: &[f64]) -> Option<Vec<f64>> {
        let spans = self.spans(pose)?;
        let t = self.static_tensions(pose)?;
        let (n, m) = (self.n(), self.m());
        let a = self.structure(&spans);
        let k: Vec<f64> = (0..m).map(|c| self.ea * (1.0 + t[c] / self.ea) / spans[c].length).collect();
        Some((0..n * n).map(|rc| (0..m).map(|c| a[rc / n * m + c] * k[c] * a[rc % n * m + c]).sum()).collect())
    }

    /// The whole stiffness of the platform with the drums held at the free lengths of rest at `pose`: minus the
    /// derivative of the cables' wrench on it over a small twist (central differences of 1 um and 1 urad), so with
    /// the stiffness of the tensions turning with the cables (the geometric part) and the pulleys.
    pub fn stiffness(&self, pose: &[f64]) -> Option<Vec<f64>> {
        let (n, m) = (self.n(), self.m());
        let spans = self.spans(pose)?;
        let t = self.static_tensions(pose)?;
        let free: Vec<f64> = (0..m).map(|k| spans[k].length / (1.0 + t[k] / self.ea)).collect();
        let wrench = |p: &[f64]| -> Option<Vec<f64>> {
            let spans = self.spans(p)?;
            let a = self.structure(&spans);
            let tension = self.tensions(&spans, &vec![0.0; m], &free, &vec![0.0; m]).platform;
            Some((0..n).map(|r| (0..m).map(|k| a[r * m + k] * tension[k]).sum()).collect())
        };
        let mut k = vec![0.0; n * n];
        for c in 0..n {
            let mut step = vec![0.0; n];
            step[c] = 1e-6;
            let up = wrench(&self.moved(pose, &step))?;
            step[c] = -1e-6;
            let down = wrench(&self.moved(pose, &step))?;
            for r in 0..n {
                k[r * n + c] = -(up[r] - down[r]) / 2e-6;
            }
        }
        // Symmetric up to the differences' error and the non-conservative part of a turning moment.
        Some((0..n * n).map(|rc| 0.5 * (k[rc] + k[rc % n * n + rc / n])).collect())
    }

    /// The platform's natural frequencies on its cables at a pose, Hz, lowest first: the eigenvalues of
    /// L^-1 K L^-T with M = L L^T (Cholesky), the drums held.
    pub fn natural_frequencies(&self, pose: &[f64]) -> Option<Vec<f64>> {
        let n = self.n();
        let k = self.stiffness(pose)?;
        let mm = self.platform_mass(pose);
        let mut l = vec![0.0; n * n];
        for i in 0..n {
            for j in 0..=i {
                let s: f64 = mm[i * n + j] - (0..j).map(|c| l[i * n + c] * l[j * n + c]).sum::<f64>();
                l[i * n + j] = if i == j { s.sqrt() } else { s / l[j * n + j] };
            }
        }
        // C = L^-1 K L^-T, column by column through forward substitution.
        let lower = |b: &[f64]| -> Vec<f64> {
            let mut x = vec![0.0; n];
            for i in 0..n {
                x[i] = (b[i] - (0..i).map(|c| l[i * n + c] * x[c]).sum::<f64>()) / l[i * n + i];
            }
            x
        };
        let mut y = vec![0.0; n * n];
        for c in 0..n {
            let col = lower(&(0..n).map(|r| k[r * n + c]).collect::<Vec<_>>());
            for r in 0..n {
                y[r * n + c] = col[r];
            }
        }
        let mut cmat = vec![0.0; n * n];
        for r in 0..n {
            let row = lower(&y[r * n..r * n + n]);
            for c in 0..n {
                cmat[c * n + r] = row[c];
            }
        }
        let eig = eigenvalues(cmat, n);
        Some(eig.iter().map(|e| e.max(0.0).sqrt() / TAU).collect())
    }

    /// The motor torques of the rigid, loss-free model for a pose, twist and twist rate: the tensions of the
    /// distribution for the platform's wrench, and the winches' own inertia on the cables' length accelerations.
    fn torques_for(&self, pose: &[f64], xi: &[f64], xi_rate: &[f64], out: &mut [f64]) {
        let (n, m) = (self.n(), self.m());
        let Some(spans) = self.spans(pose) else {
            out.fill(f64::NAN);
            return;
        };
        let a = self.structure(&spans);
        let mass = self.platform_mass(pose);
        let bias = self.platform_bias(pose, xi);
        let w: Vec<f64> = (0..n).map(|r| (0..n).map(|c| mass[r * n + c] * xi_rate[c]).sum::<f64>() + bias[r]).collect();
        // Beyond the cables' band the feedforward still asks for the distribution's nearest: unbounded tensions.
        let t = distribute(&a, &w, self.t_min, self.t_max, self.t_ref)
            .or_else(|| distribute(&a, &w, -1e12, 1e12, self.t_ref))
            .unwrap_or_else(|| vec![f64::NAN; m]);
        let curve = self.length_curvature(&spans, pose, xi);
        for k in 0..m {
            let j = &self.winches[k];
            let lacc = curve[k] - (0..n).map(|r| a[r * m + k] * xi_rate[r]).sum::<f64>();
            out[k] = (self.drum_mass * lacc - t[k]) / j.ratio() + j.rotor_inertia * j.ratio() * lacc;
        }
    }

    /// The losses of one moment: per winch its bearings' friction, its motor's friction with the brake, its gearbox;
    /// per cable its damper; then the stops and what the ripple gives off. And per winch the gearbox's loss as a torque
    /// at the rotor.
    fn losses(&self, s: &[f64], torque: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let m = self.m();
        let (pose, xi) = self.platform(s);
        let mut powers = vec![0.0; 4 * m + 2];
        let mut at_rotor = vec![0.0; m];
        let Some(spans) = self.spans(pose) else { return (vec![f64::NAN; 4 * m + 2], at_rotor) };
        let (free, free_rates, rotor, speed) = self.winch_state(s, &spans);
        let brakes = self.brakes(s);
        let rigid = (!self.elastic).then(|| self.rigid_solve(s, torque));
        let damper = self.elastic.then(|| {
            let rates = self.length_rates(&spans, xi);
            let t = self.tensions(&spans, &rates, &free, &free_rates);
            (0..m).map(|k| t.damper[k] * (rates[k] - free_rates[k])).collect::<Vec<_>>()
        });
        for k in 0..m {
            let j = &self.winches[k];
            let [friction, stop, shaft, drag] = j.losses(free[k], free_rates[k], speed[k], brakes[k], self.brake_engaged);
            let gear = match &rigid {
                Some((_, _, into, out)) => {
                    at_rotor[k] = (into[k] - out[k]) / j.ratio();
                    (into[k] - out[k]) * free_rates[k]
                }
                None => {
                    let g = j.gearbox.torque(rotor[k], speed[k], free[k], free_rates[k]);
                    at_rotor[k] = g.rotor - g.output / j.ratio();
                    j.gearbox.loss_power(rotor[k], speed[k], free[k], free_rates[k])
                }
            };
            powers[3 * k..3 * k + 3].copy_from_slice(&[friction, shaft, gear + drag]);
            powers[3 * m + k] = damper.as_ref().map_or(0.0, |d| d[k]);
            powers[4 * m] += stop;
            powers[4 * m + 1] += (torque[k] - j.ripple.apply(torque[k], rotor[k])) * speed[k];
        }
        (powers, at_rotor)
    }

    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        let (n, m) = (self.n(), self.m());
        let (pose, xi) = self.platform(s);
        let Some(spans) = self.spans(pose) else { return [f64::NAN; 3] };
        let mass = self.platform_mass(pose);
        let mut kinetic = 0.5 * (0..n * n).map(|rc| xi[rc / n] * mass[rc] * xi[rc % n]).sum::<f64>();
        let (p, _) = self.place(pose);
        let potential = self.moving() * self.g * p[2];
        let (free, free_rates, rotor, speed) = self.winch_state(s, &spans);
        let mut elastic = 0.0;
        for k in 0..m {
            let j = &self.winches[k];
            elastic += j.stop_energy(free[k]);
            if self.elastic {
                kinetic += 0.5 * self.drum_mass * free_rates[k].powi(2) + 0.5 * j.rotor_inertia * speed[k].powi(2);
                elastic += j.gearbox.energy(rotor[k], free[k]);
                let e = spans[k].length - free[k];
                if e > 0.0 {
                    elastic += 0.5 * self.ea * e * e / free[k];
                }
            } else {
                kinetic += 0.5 * (self.drum_mass + j.reflected_inertia()) * free_rates[k].powi(2);
            }
        }
        [kinetic, potential, elastic]
    }

    /// The least-norm share of the platform's wrench `w` per cable, A^+ w: what the parts of the torque breakdown count
    /// for inertia and gravity (the pretension the motors also hold shows in the rest).
    fn least_norm(a: &[f64], w: &[f64]) -> Vec<f64> {
        let n = w.len();
        let m = a.len() / n;
        let aat: Vec<f64> = (0..n * n).map(|rc| (0..m).map(|k| a[rc / n * m + k] * a[rc % n * m + k]).sum()).collect();
        let y = solve(&aat, w).unwrap_or_else(|| vec![f64::NAN; n]);
        (0..m).map(|k| (0..n).map(|r| a[r * m + k] * y[r]).sum()).collect()
    }
}

impl Machine for CableRobot {
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    /// Per cable its highest tension against the rating (breaking load over the safety factor) and how far it fell
    /// below the least tension that keeps it taut (0 allowed).
    fn rating_checks(&self) -> Vec<RatingCheck> {
        (0..self.m())
            .flat_map(|k| {
                [
                    RatingCheck {
                        check: "cable-max-tension",
                        signal: format!("cable.{}.tension", MOTOR_IDS[k]),
                        motor: Some(k),
                        limit: self.t_max,
                        life: false,
                    },
                    RatingCheck {
                        check: "cable-min-tension",
                        signal: format!("cable.{}.shortfall", MOTOR_IDS[k]),
                        motor: Some(k),
                        limit: 0.0,
                        life: false,
                    },
                ]
            })
            .collect()
    }

    /// NaN where the pose cannot be held (see `unreachable`).
    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        match self.free_lengths(tool) {
            Some(l) => (0..self.m()).for_each(|k| motors[k] = self.winches[k].ratio() * l[k]),
            None => motors.fill(f64::NAN),
        }
    }

    /// The motors' cable to the pose. Rigid: the pose of those lengths (`pose_of_lengths`). Elastic: Gauss-Newton on the
    /// free lengths at rest (`rest_lengths`) from the pose of the lengths stretched at the reference tension; first
    /// with the steps through G = -A^T / (1 + t_ref / EA), cheap, as long as they shrink by a third each; then, where
    /// the lengths still miss (near the edge, where the tensions change fast with the pose), with the Jacobian by
    /// central differences (0.1 um, 0.1 urad), each step halved until the lengths miss by less. No step longer than
    /// 50 mm or 50 mrad. Settled at a step below 1e-11 m (rad) with the lengths missing by under 1e-10 m, or where no
    /// step helps any more: lengths no pose fits exactly (the motors of a run with a following error) give the pose
    /// that fits them best. NaN where it does not settle within 100 steps.
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        let (n, m) = (self.n(), self.m());
        let free: Vec<f64> = (0..m).map(|k| motors[k] / self.winches[k].ratio()).collect();
        let stretch = if self.elastic { 1.0 + self.t_ref / self.ea } else { 1.0 };
        let stretched: Vec<f64> = free.iter().map(|l| l * stretch).collect();
        let Some(mut pose) = self.pose_of_lengths(&stretched, &self.home) else {
            tool[..n].fill(f64::NAN);
            return;
        };
        if !self.elastic {
            tool[..n].copy_from_slice(&pose);
            return;
        }
        let miss_at = |pose: &[f64]| -> Option<Vec<f64>> {
            Some(self.rest_lengths(pose)?.iter().zip(&free).map(|(l, f)| f - l).collect())
        };
        let size = |v: &[f64]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
        let mut exact = false;
        let mut last = f64::INFINITY;
        for _ in 0..100 {
            let Some(miss) = miss_at(&pose) else { break };
            let Some(spans) = self.spans(&pose) else { break };
            // g (m x n, row by row): d free length / d twist.
            let mut g = vec![0.0; m * n];
            if exact {
                for c in 0..n {
                    let mut step = vec![0.0; n];
                    step[c] = 1e-7;
                    let up = self.rest_lengths(&self.moved(&pose, &step));
                    step[c] = -1e-7;
                    let down = self.rest_lengths(&self.moved(&pose, &step));
                    let (Some(up), Some(down)) = (up, down) else {
                        tool[..n].fill(f64::NAN);
                        return;
                    };
                    (0..m).for_each(|k| g[k * n + c] = (up[k] - down[k]) / 2e-7);
                }
            } else {
                let a = self.structure(&spans);
                (0..m * n).for_each(|kc| g[kc] = -a[kc % n * m + kc / n] / stretch);
            }
            let gtg: Vec<f64> =
                (0..n * n).map(|rc| (0..m).map(|k| g[k * n + rc / n] * g[k * n + rc % n]).sum()).collect();
            let rhs: Vec<f64> = (0..n).map(|r| (0..m).map(|k| g[k * n + r] * miss[k]).sum::<f64>()).collect();
            let Some(mut step) = solve(&gtg, &rhs) else { break };
            let longest = step.iter().fold(0.0f64, |s, v| s.max(v.abs()));
            if longest > 0.05 {
                step.iter_mut().for_each(|v| *v *= 0.05 / longest);
            }
            // The lengths come from solves to some 1e-14 m: below that a step is noise.
            if longest < 1e-11 && size(&miss) < 1e-10 {
                tool[..n].copy_from_slice(&pose);
                return;
            }
            if !exact {
                if longest < 1e-11 || longest > 0.67 * last {
                    exact = true;
                    continue;
                }
                last = longest;
                pose = self.moved(&pose, &step);
                continue;
            }
            let mut next = None;
            for _ in 0..30 {
                let trial = self.moved(&pose, &step);
                if miss_at(&trial).is_some_and(|m| size(&m) < size(&miss)) {
                    next = Some(trial);
                    break;
                }
                step.iter_mut().for_each(|v| *v *= 0.5);
            }
            match next {
                Some(next) => pose = next,
                // Settled, or the best fit of lengths no pose fits: no step helps any more.
                None => {
                    tool[..n].copy_from_slice(&pose);
                    return;
                }
            }
        }
        tool[..n].fill(f64::NAN);
    }

    fn state_len(&self) -> usize {
        2 * self.n() + self.m() * if self.elastic { 5 } else { 1 }
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let (n, m) = (self.n(), self.m());
        state.clear();
        state.extend_from_slice(&tool[..n]);
        state.extend(vec![0.0; n]);
        let (Some(spans), Some(t)) = (self.spans(tool), self.static_tensions(tool)) else {
            state.iter_mut().for_each(|v| *v = f64::NAN);
            state.resize(self.state_len(), f64::NAN);
            torque.fill(f64::NAN);
            return;
        };
        let (mut free, mut rotors) = (vec![0.0; m], vec![0.0; m]);
        for k in 0..m {
            let j = &self.winches[k];
            let (l, drum) = if self.elastic {
                let l = spans[k].length / (1.0 + t[k] / self.ea);
                (l, t[k] * (1.0 + (spans[k].length - l) / (2.0 * l)))
            } else {
                (spans[k].length, t[k])
            };
            // At rest the gearbox holds the drum against its cable and the stops.
            let out = -drum - j.output_torque(l, 0.0);
            let rotor = j.ratio() * (l + if self.elastic { j.gearbox.static_twist(out) } else { 0.0 });
            free[k] = l;
            rotors[k] = rotor;
            torque[k] = j.ripple.invert(out / j.ratio(), rotor);
        }
        if self.elastic {
            state.extend(&free);
            state.extend(vec![0.0; m]);
            state.extend(&rotors);
            state.extend(vec![0.0; m]);
        }
        state.resize(self.state_len(), 0.0);
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let (n, m) = (self.n(), self.m());
        let (pose, xi) = self.platform(s);
        out[..n].copy_from_slice(&self.pose_rate(pose, xi));
        if !self.elastic {
            let (acc, ..) = self.rigid_solve(s, torque);
            out[n..2 * n].copy_from_slice(&acc);
            let Some(spans) = self.spans(pose) else {
                out.fill(f64::NAN);
                return;
            };
            let (_, _, _, speed) = self.winch_state(s, &spans);
            let brakes = self.brakes(s);
            for k in 0..m {
                out[2 * n + k] = self.winches[k].brake.torque(self.brake_engaged, speed[k], brakes[k]).1;
            }
            return;
        }
        let Some(spans) = self.spans(pose) else {
            out.fill(f64::NAN);
            return;
        };
        let (free, free_rates, rotor, speed) = self.winch_state(s, &spans);
        let brakes = self.brakes(s);
        let rates = self.length_rates(&spans, xi);
        let t = self.tensions(&spans, &rates, &free, &free_rates);
        let a = self.structure(&spans);
        let bias = self.platform_bias(pose, xi);
        let rhs: Vec<f64> =
            (0..n).map(|r| (0..m).map(|k| a[r * m + k] * t.platform[k]).sum::<f64>() - bias[r]).collect();
        let acc = solve(&self.platform_mass(pose), &rhs).unwrap_or_else(|| vec![f64::NAN; n]);
        out[n..2 * n].copy_from_slice(&acc);
        for k in 0..m {
            let j = &self.winches[k];
            let (gear, rotor_acc, brake_rate) =
                j.rotor(torque[k], rotor[k], speed[k], free[k], free_rates[k], brakes[k], self.brake_engaged);
            out[2 * n + k] = free_rates[k];
            out[2 * n + m + k] = (gear.output + j.output_torque(free[k], free_rates[k]) + t.drum[k]) / self.drum_mass;
            out[2 * n + 2 * m + k] = speed[k];
            out[2 * n + 3 * m + k] = rotor_acc;
            out[2 * n + 4 * m + k] = brake_rate;
        }
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let n = self.n();
        let (pose, xi) = self.platform(s);
        pos[..n].copy_from_slice(pose);
        vel[..n].copy_from_slice(&self.pose_rate(pose, xi));
    }

    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        let (n, m) = (self.n(), self.m());
        let pose = s[..n].to_vec();
        let xi = self.twist(&pose, &vel[..n]);
        s[n..2 * n].copy_from_slice(&xi);
        if self.elastic {
            let Some(spans) = self.spans(&pose) else { return };
            let rates = self.length_rates(&spans, &xi);
            for k in 0..m {
                s[2 * n + m + k] = rates[k];
                s[2 * n + 3 * m + k] = self.winches[k].ratio() * rates[k];
            }
        }
    }

    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let m = self.m();
        let (pose, _) = self.platform(s);
        let Some(spans) = self.spans(pose) else {
            pos[..m].fill(f64::NAN);
            vel[..m].fill(f64::NAN);
            return;
        };
        let (_, _, rotor, speed) = self.winch_state(s, &spans);
        pos[..m].copy_from_slice(&rotor);
        vel[..m].copy_from_slice(&speed);
    }

    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        let n = self.n();
        self.feedforward_torque_moving(pos, &vec![0.0; n], acc, out);
    }

    /// The twist and its rate from the pose's speed and acceleration (w' = J_l r'' + (d/dt J_l) r', the latter by
    /// central differences along r'), then `torques_for`.
    fn feedforward_torque_moving(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut [f64]) {
        let n = self.n();
        let xi = self.twist(pos, &vel[..n]);
        let mut rate = self.twist(pos, &acc[..n]);
        if self.spatial {
            let rv = [vel[3], vel[4], vel[5]];
            let speed = norm(rv);
            if speed > 0.0 {
                let h = 1e-4 / speed;
                let at = |s: f64| left_jacobian([pos[3] + s * rv[0], pos[4] + s * rv[1], pos[5] + s * rv[2]]);
                let (up, down) = (mul(&at(h), rv), mul(&at(-h), rv));
                // (d/dt J_l) r' = (J_l(r + h r') - J_l(r - h r')) r' / (2 h).
                for i in 0..3 {
                    rate[3 + i] += (up[i] - down[i]) / (2.0 * h);
                }
            }
        }
        self.torques_for(pos, &xi, &rate, out);
    }

    fn energy(&self, s: &[f64]) -> f64 {
        self.energy_parts(s).iter().sum()
    }

    fn set_payload_attached(&mut self, attached: bool) {
        self.attached = attached;
    }
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.brake_engaged = engaged;
    }

    /// The gearboxes, in the order of `thermal_parts`.
    fn set_part_temperatures(&mut self, rises: &[f64]) {
        let Some(law) = &self.thermal else { return };
        for (winch, rise) in self.winches.iter_mut().filter(|w| !w.coupled).zip(rises) {
            winch.gearbox.efficiency = law.gearbox_efficiency(*rise) * winch.extra_efficiency;
            winch.shaft.set_oil(law.viscosity_factor(*rise));
        }
    }

    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        self.natural_frequencies(tool_pose)?.first().copied()
    }
    fn modes(&self, tool_pose: &[f64]) -> Vec<Mode> {
        self.natural_frequencies(tool_pose)
            .unwrap_or_default()
            .into_iter()
            .map(|hz| Mode { hz, damping: 0.0 })
            .collect()
    }

    /// Out of the cables' reach (an anchor at its pulley), outside the wrench-feasible workspace (no tensions within
    /// the band hold the platform with its payload at rest), or a winch beyond its capacity.
    fn unreachable(&self, tool: &[f64], slack: f64) -> Option<&'static str> {
        let Some(spans) = self.spans(tool) else { return Some("error-target-out-of-range") };
        let a = self.structure(&spans);
        if distribute(&a, &self.weight(), self.t_min, self.t_max, self.t_ref).is_none() {
            return Some("error-cable-wrench-infeasible");
        }
        let (min, max) = self.winches[0].range;
        let free = self.free_lengths(tool)?;
        free.iter().any(|l| *l < min - slack || *l > max + slack).then_some("error-joint-beyond-stop")
    }

    /// The structure matrix's condition number, the turns weighed by the platform's size.
    fn condition(&self, tool: &[f64]) -> Option<f64> {
        let Some(spans) = self.spans(tool) else { return Some(f64::INFINITY) };
        let (n, m) = (self.n(), self.m());
        let mut a = self.structure(&spans);
        for (r, d) in self.dofs.iter().enumerate() {
            if *d >= 3 {
                (0..m).for_each(|k| a[r * m + k] /= self.size);
            }
        }
        let aat: Vec<f64> = (0..n * n).map(|rc| (0..m).map(|k| a[rc / n * m + k] * a[rc % n * m + k]).sum()).collect();
        let eig = eigenvalues(aat, n);
        let (least, most) = (eig[0], eig[n - 1]);
        Some(if least > 1e-24 * most { (most / least).sqrt() } else { f64::INFINITY })
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let m = self.m();
        let mut out = Vec::new();
        for k in 0..m {
            out.push(SignalInfo::new(
                format!("cable.{}.tension", MOTOR_IDS[k]),
                CABLE_KEYS[k],
                "quantity-cable-tension",
                "N",
            ));
            out.push(SignalInfo::new(
                format!("cable.{}.shortfall", MOTOR_IDS[k]),
                CABLE_KEYS[k],
                "quantity-cable-shortfall",
                "N",
            ));
            out.push(SignalInfo::new(
                format!("cable.{}.length", MOTOR_IDS[k]),
                CABLE_KEYS[k],
                "quantity-cable-length",
                "m",
            ));
        }
        out.push(SignalInfo::new("platform.condition", "group-platform", "quantity-condition-number", ""));
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
        for k in 0..m {
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}winch_{}", MOTOR_IDS[k]),
                WINCH_KEYS[k],
                "quantity-friction-power",
                "W",
            ));
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}shaft_{}", MOTOR_IDS[k]),
                MOTOR_KEYS[k],
                "quantity-shaft-friction-power",
                "W",
            ));
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}gearbox_{}", MOTOR_IDS[k]),
                MOTOR_KEYS[k],
                "quantity-gearbox-loss-power",
                "W",
            ));
        }
        for k in 0..m {
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}cable_{}", MOTOR_IDS[k]),
                CABLE_KEYS[k],
                "quantity-damping-power",
                "W",
            ));
        }
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}hard_stops"), "group-winch", "quantity-damping-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}other"), "loss-other", "quantity-power-given-off", "W"));
        out.extend(ENERGY_PARTS.map(|part| {
            SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")
        }));
        for contact in self.friction_contacts() {
            for part in FRICTION_SIGNALS {
                let unit = match part {
                    "speed" => "m/s",
                    "load_scale" => "",
                    _ => "N",
                };
                out.push(SignalInfo::new(
                    format!("{}.friction_{part}", contact.id),
                    contact.owner_key,
                    &format!("quantity-friction-{}", part.replace('_', "-")),
                    unit,
                ));
            }
        }
        out
    }

    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (pose, _) = self.platform(s);
        let tension = self.state_tensions(s, torque);
        let lengths: Vec<f64> =
            self.spans(pose).map_or(vec![f64::NAN; self.m()], |sp| sp.iter().map(|s| s.length).collect());
        for k in 0..self.m() {
            out.extend([tension[k], (self.t_min - tension[k]).max(0.0), lengths[k]]);
        }
        out.push(self.condition(pose).unwrap_or(f64::INFINITY));
    }

    /// Torque breakdown (inertia and gravity as the least-norm shares of the platform's wrench, plus the winches' own
    /// inertia; friction; the rest with the pretension), loss powers, energies, winch friction.
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (n, m) = (self.n(), self.m());
        let (pose, xi) = self.platform(s);
        let Some(spans) = self.spans(pose) else {
            out.extend(vec![f64::NAN; 4 * m + 4 * m + 2 + 3 + 5 * m]);
            return;
        };
        let a = self.structure(&spans);
        let acc = self.twist_acceleration(s, torque);
        let mass = self.platform_mass(pose);
        let bias = self.platform_bias(pose, xi);
        let gravity_w = self.platform_bias(pose, &vec![0.0; n]);
        let inertia_w: Vec<f64> =
            (0..n).map(|r| (0..n).map(|c| mass[r * n + c] * acc[c]).sum::<f64>() + bias[r] - gravity_w[r]).collect();
        let (inertia_t, gravity_t) = (Self::least_norm(&a, &inertia_w), Self::least_norm(&a, &gravity_w));
        let curve = self.length_curvature(&spans, pose, xi);
        let (_, free_rates, _, speed) = self.winch_state(s, &spans);
        let brakes = self.brakes(s);
        let (powers, at_rotor) = self.losses(s, torque);
        for k in 0..m {
            let j = &self.winches[k];
            let lacc = curve[k] - (0..n).map(|r| a[r * m + k] * acc[r]).sum::<f64>();
            let held = j.brake.torque(self.brake_engaged, speed[k], brakes[k]).0;
            let parts = [
                (self.drum_mass * lacc - inertia_t[k]) / j.ratio() + j.rotor_inertia * j.ratio() * lacc,
                -gravity_t[k] / j.ratio(),
                j.shaft.torque(speed[k]) + held + at_rotor[k] + j.friction.torque(free_rates[k]) / j.ratio(),
            ];
            out.extend(parts);
            out.push(torque[k] - parts.iter().sum::<f64>());
        }
        out.extend(&powers);
        out.extend(self.energy_parts(s));
        for k in 0..m {
            let f = &self.winches[k].friction;
            out.extend([
                free_rates[k],
                f.coulomb * (free_rates[k] / f.smoothing).tanh(),
                f.viscous * free_rates[k],
                0.0,
                1.0,
            ]);
        }
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(s, torque).0);
    }

    fn friction_contacts(&self) -> Vec<FrictionContact> {
        (0..self.m()).map(|k| FrictionContact { id: WINCH_CONTACTS[k], owner_key: WINCH_KEYS[k] }).collect()
    }
    fn friction_curve(&self, i: usize, speed: f64, _load_scale: f64) -> [f64; 3] {
        let Some(w) = self.winches.get(i) else { return [0.0; 3] };
        [w.friction.coulomb * (speed / w.friction.smoothing).tanh(), w.friction.viscous * speed, 0.0]
    }

    fn thermal_parts(&self) -> Vec<ThermalPart> {
        (0..self.m())
            .filter(|k| !self.winches[*k].coupled)
            .enumerate()
            .map(|(i, k)| ThermalPart {
                id: GEARBOX_IDS[k],
                kind: "gearbox",
                owner_key: MOTOR_KEYS[k],
                scene_part: "gearbox",
                scene_index: i,
            })
            .collect()
    }

    /// The frame's posts, each winch (motor and gearbox) behind its pulley, the cables coloured by their tension against
    /// the reference, the platform.
    fn scene(&self, s: &[f64], out: &mut Scene) {
        out.parts.clear();
        out.labels.clear();
        let (pose, _) = self.platform(s);
        let Some(spans) = self.spans(pose) else { return };
        let (p, _) = self.place(pose);
        let tension = self.state_tensions(s, &vec![0.0; self.m()]);
        let (_, _, rotor, speed) = self.winch_state(s, &spans);
        let grey = [0.55, 0.57, 0.6];
        let floor = self.frame.iter().map(|e| e[2]).fold(f64::INFINITY, f64::min) - 0.1;
        for (k, c) in self.cables.iter().enumerate() {
            let post = [c.exit[0], c.exit[1], floor];
            if !self.frame[..k].iter().any(|e| (e[0] - c.exit[0]).abs() < 1e-6 && (e[1] - c.exit[1]).abs() < 1e-6) {
                let top =
                    self.frame.iter().filter(|e| (e[0] - c.exit[0]).abs() < 1e-6 && (e[1] - c.exit[1]).abs() < 1e-6);
                let height = top.map(|e| e[2]).fold(f64::NEG_INFINITY, f64::max) + 0.1;
                out.parts.push(Part::new(
                    "frame",
                    Shape::Segment { to: [c.exit[0], c.exit[1], height], width: 0.05 },
                    post,
                    grey,
                ));
            }
            let back = add(c.exit, scale(c.axis, 0.12));
            out.parts.push(Part::new(
                "pulley",
                Shape::Cylinder { radius: self.pulley, length: 0.02 },
                c.exit,
                [0.3, 0.3, 0.32],
            ));
            out.parts.push(
                Part::new(
                    sim_core::scene::MOTOR_PART,
                    Shape::Box { size: [0.08, 0.08, 0.08] },
                    add(back, scale(c.ref_x, -0.1)),
                    [0.9, 0.45, 0.1],
                )
                .turned(rotor[k]),
            );
            out.parts.push(Part::new(
                "gearbox",
                Shape::Box { size: [0.06, 0.06, 0.06] },
                add(back, scale(c.ref_x, -0.03)),
                [0.25, 0.25, 0.28],
            ));
            out.parts.push(Part::new(
                "drum",
                Shape::Cylinder { radius: self.drum_radius, length: 0.05 },
                add(back, scale(c.ref_x, 0.04)),
                [0.75, 0.75, 0.78],
            ));
            let level = ((tension[k] - self.t_ref) / self.t_ref).clamp(-1.0, 1.0) as f32;
            out.parts.push(
                Part::new(
                    "cable",
                    Shape::Segment { to: add(p, spans[k].b), width: 0.006 },
                    spans[k].exit,
                    sim_core::scene::BELT_COLOR,
                )
                .highlighted(level),
            );
            if self.spatial {
                out.parts.push(Part::new(
                    "platform",
                    Shape::Segment { to: add(p, spans[k].b), width: 0.02 },
                    p,
                    [0.3, 0.45, 0.7],
                ));
            }
            out.labels.push(Label {
                pos: add(back, [0.0, 0.0, 0.1]),
                owner_key: MOTOR_KEYS[k],
                value: speed[k] * 60.0 / TAU,
                unit_key: "unit-rpm",
                decimals: 0,
            });
            out.labels.push(Label {
                pos: scale(add(spans[k].exit, add(p, spans[k].b)), 0.5),
                owner_key: CABLE_KEYS[k],
                value: tension[k],
                unit_key: "unit-n",
                decimals: 0,
            });
        }
        let size = if self.spatial { 0.08 } else { 0.1 };
        out.parts.push(Part::new("tool", Shape::Box { size: [size, size, size] }, p, [0.2, 0.22, 0.26]));
        if self.attached && self.payload > 0.0 {
            out.parts.push(Part::new(
                "payload",
                Shape::Box { size: [0.08, 0.08, 0.06] },
                add(p, [0.0, 0.0, -0.08]),
                [0.2, 0.7, 0.3],
            ));
        }
    }
    fn scene_point(&self, tool: &[f64]) -> Option<[f64; 3]> {
        Some(self.place(tool).0)
    }
    fn collision_bodies(&self, s: &[f64]) -> Vec<sim_core::Aabb> {
        let mut scene = Scene::default();
        self.scene(s, &mut scene);
        sim_core::scene::scene_boxes(&scene.parts, &["tool", "payload"])
    }
}

#[cfg(test)]
mod tests;
