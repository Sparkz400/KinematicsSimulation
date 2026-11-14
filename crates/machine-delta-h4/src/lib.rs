//! Four-arm delta of the Quattro / H4 type (Pierrot's H4, the Adept/Omron Quattro): four motor-driven arms on the base
//! at 90 degrees, arm k along the azimuth (k - 1) 90 degrees, each carrying a parallelogram of two forearms down to a
//! travelling plate in two parts. Arms 1 and 2 carry part A, arms 3 and 4 part B. The tool is x, y, z and its turn c
//! about +z.
//!
//! Which arms share a part: two neighbouring ones. A parallelogram keeps its plate end parallel to its elbow's axis,
//! leaving the part one turn, about that axis; two neighbouring arms' axes are at right angles, so a part on them cannot
//! turn at all. Two opposite arms' axes are parallel and would leave a part free to turn about them.
//!
//! The two parts slide against each other along the horizontal d = (1, -1, 0) / sqrt 2, each by the half slide h from
//! the tool's point the other way: part A at p - h d, part B at p + h d. A pinion between a rack on each part rolls by
//! h / r_p, its centre on a carriage that the parts guide between them stays at the tool's point p, and a gear stage of
//! ratio i_p turns the tool: c = kappa h, kappa = i_p / r_p (`Plate`). d is across the line between the parts: along it,
//! d = (1, 1, 0) / sqrt 2, the slide would move the four forearms' ends as the plate's descent does at the centre, and
//! the Jacobian would be singular there.
//!
//! Kinematics in x = (p, h), leg k at azimuth phi_k, side sigma_k (-1 on part A, +1 on part B), the plate parts' radius
//! folded into e = base radius - plate radius as on the D3, theta_k the arm angle (0 horizontal, positive down):
//!
//! ```text
//! elbow     c_k = R_z(phi_k) (e + L_a cos theta, 0, -L_a sin theta)
//! end       q_k = p + sigma_k h d
//! constraint |q_k - c_k|^2 = L_b^2
//! inverse   per leg the D3's closed form at q_k (elbow out)
//! forward   Newton on the four constraints from the D3's trilateration of legs 1 to 3 at h = 0 (the lower point)
//! Jacobian  J_x x' = J_q theta', J_x row k (u_k, sigma_k u_k . d), u_k = q_k - c_k, J_q = diag(u_k . dc_k/dtheta_k)
//!           J_x x'' = J_q theta'' + w, w_k = theta_k'^2 (u_k . c_k'') - |q_k' - c_k' theta_k'|^2
//! ```
//!
//! Dynamics, by virtual work as the D3's (Codourey 1998, Olsson 2009): each forearm pair's mass split, a share r at the
//! elbow and 1 - r at its plate part (or, with effects.forearm_inertia, a rigid rod: a sixth at each end, two thirds
//! at its middle, and its spin about its own axis), and every body a point with its Jacobian P (3 x 4) in the arm
//! angles: the parts, m_part + 2 (1 - r) m_fb each, with P = J_p -+ d J_h; the carriage and the payload at p; the tool's
//! turn, I_t c'^2 / 2 = I_t kappa^2 h'^2 / 2; the arms about their joints. Each body adds m P^T P to M, m P^T a0 to
//! the velocity term (a0 its acceleration at no arm acceleration) and -m g P_z to the gravity:
//!
//! ```text
//! M(theta) theta'' = tau_gear + Q_g - v + tau_joint
//! ```
//!
//! With effects.amplifier_compliance the tool is a body of its own, its angle phi on the amplifier's spring
//! (`Amplifier`, with effects.amplifier_backlash a dead band in it): tau_a = k (kappa h - phi) + c (kappa h' - phi'),
//! I_t phi'' = tau_a, and the arms feel -kappa J_h^T tau_a. Each joint is a `RotaryJoint` (gearbox with stiffness,
//! backlash and efficiency, motor with friction, ripple and brake, joint friction, stops), as on the D3.
//!
//! The D3's bending upper arms (effects.link_elasticity) are not here.
// shortcut: no bending of the upper arms; add the D3's in- and out-of-plane bend states per arm if a ringing plate after
// a move ever matters on this machine.

use sim_components::joint::RotaryJoint;
use sim_components::mech::ThermalFeedback;
use sim_core::{
    AxisInfo, Config, ConfigError, ENERGY_PARTS, LOSS_PREFIX, Label, Machine, MachineEntry, Mode, MotorInfo, Part,
    Scene, Shape, SignalInfo, TORQUE_PARTS, ThermalPart,
};

pub static ENTRY: MachineEntry = MachineEntry {
    id: "delta-h4",
    key: "machine-delta-h4",
    default_config: include_str!("../default.toml"),
    i18n: &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))],
    build,
    suite_configs,
};

const G: f64 = 9.80665;

/// Effects that take energy out or put it in; the standard suite runs with them off.
pub const LOSSY_EFFECTS: [&str; 7] = [
    "joint_friction",
    "motor_friction",
    "gearbox_drag",
    "gearbox_lost_motion",
    "gearbox_efficiency",
    "torque_ripple",
    "holding_brake",
];

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    Ok(Box::new(H4::new(cfg)?))
}

/// The start pose and the cycle's targets moved by `dz` in z, the tool's turn kept.
fn lowered(cfg: &mut Config, dz: f64) {
    let s = cfg.f64s("start.pose").unwrap();
    cfg.set_text(&format!("start.pose = [{:?}, {:?}, {:?}, {:?}]", s[0], s[1], s[2] + dz, s[3])).unwrap();
    let segments: Vec<String> = cfg
        .array("cycle.segment")
        .unwrap()
        .iter()
        .map(|s| {
            let t: Vec<f64> = s["target"].as_array().unwrap().iter().map(|v| v.as_float().unwrap()).collect();
            format!(
                "{{ target = [{:?}, {:?}, {:?}, {:?}], dwell_s = {:?}, payload = {} }}",
                t[0],
                t[1],
                t[2] + dz,
                t[3],
                s["dwell_s"].as_float().unwrap(),
                s["payload"].as_bool().unwrap()
            )
        })
        .collect();
    cfg.set_text(&format!("cycle.segment = [{}]", segments.join(", "))).unwrap();
}

/// The default config with every loss switched off.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    for damping in ["gearbox.damping_nm_s_rad", "joint.stop_damping_nm_s_rad", "plate.amplifier_damping_nm_s_rad"] {
        cfg.set_text(&format!("{damping} = 0.0")).unwrap();
    }
    cfg
}

/// Loss-free rigid and elastic models, with and without the payload; the elastic ones with the amplifier's spring (and
/// its backlash) and, with the payload, part heating and the temperature feedback (no loss, so the gearboxes stay at
/// ambient).
///
/// As on the D3, their working range, start pose and cycle lie lower than the default's, here by 200 mm (top 950 mm
/// below the shoulders, the suite's middle pose 1175 mm down): held by constant motor torques, as the suite holds it
/// for 10 s after a pulse, the plate is unstable in its vertical mode in the upper part of the range, where its weight
/// pulls harder through the Jacobian the further it sinks (the potential's second derivative, by hand at the centre with
/// the lumped forearms, 9.3 kg on the plate: -4.3 N m/rad at 975 mm down, +5.7 at 1175 mm; see
/// `the_vertical_mode_under_constant_torques_is_unstable_high_up`). The drives' position loops hold it.
fn suite_configs() -> Vec<Config> {
    let mut out = Vec::new();
    for (elastic, payload) in [(false, true), (false, false), (true, true), (true, false)] {
        let mut cfg = lossless_config();
        cfg.set_text("workspace.top_z_m = -0.95").unwrap();
        lowered(&mut cfg, -0.2);
        cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
        cfg.set_text(&format!("effects.amplifier_compliance = {elastic}")).unwrap();
        cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
        cfg.set_text(&format!("effects.part_heating = {}", elastic && payload)).unwrap();
        cfg.set_text(&format!("effects.thermal_feedback = {}", elastic && payload)).unwrap();
        out.push(cfg);
    }
    out
}

type V3 = [f64; 3];
type V4 = [f64; 4];
type M4 = [[f64; 4]; 4];
/// Mass matrix, velocity term and gravity in the arm angles.
type Load = (M4, V4, V4);

fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

fn dot4(a: V4, b: V4) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
}

fn mul4(m: &M4, v: V4) -> V4 {
    std::array::from_fn(|r| dot4(m[r], v))
}

/// Solves m x = b by Gaussian elimination with partial pivoting; `None` where m is singular.
fn solve4(m: &M4, b: V4) -> Option<V4> {
    let (mut a, mut x) = (*m, b);
    for c in 0..4 {
        let p = (c..4).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if !(a[p][c].abs() > 1e-300) {
            return None;
        }
        a.swap(p, c);
        x.swap(p, c);
        for r in c + 1..4 {
            let f = a[r][c] / a[c][c];
            for k in c..4 {
                a[r][k] -= f * a[c][k];
            }
            x[r] -= f * x[c];
        }
    }
    for c in (0..4).rev() {
        let s: f64 = (c + 1..4).map(|k| a[c][k] * x[k]).sum();
        x[c] = (x[c] - s) / a[c][c];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

/// The inverse of a 4 x 4 matrix, column by column; `None` where it is singular.
fn inverse4(m: &M4) -> Option<M4> {
    let mut out = [[0.0; 4]; 4];
    for c in 0..4 {
        let col = solve4(m, std::array::from_fn(|r| if r == c { 1.0 } else { 0.0 }))?;
        for r in 0..4 {
            out[r][c] = col[r];
        }
    }
    Some(out)
}

/// A point mass m into `load`: its Jacobian jm (3 x 4), its acceleration am at no acceleration of the arms, its weight
/// down z at g.
fn add_point(load: &mut Load, m: f64, jm: &[V4; 3], am: V3, g: f64) {
    for r in 0..4 {
        for c in 0..4 {
            load.0[r][c] += m * (0..3).map(|k| jm[k][r] * jm[k][c]).sum::<f64>();
        }
        load.1[r] += m * (0..3).map(|k| jm[k][r] * am[k]).sum::<f64>();
        load.2[r] -= m * g * jm[2][r];
    }
}

/// A turn with inertia `inertia` at the rate row . theta' into `load`, its acceleration a0 at no acceleration of the
/// arms: no weight.
fn add_turn(load: &mut Load, inertia: f64, row: V4, a0: f64) {
    for r in 0..4 {
        for c in 0..4 {
            load.0[r][c] += inertia * row[r] * row[c];
        }
        load.1[r] += inertia * row[r] * a0;
    }
}

/// The eigenvalues and eigenvectors (columns) of a symmetric matrix, by cyclic Jacobi rotations.
fn symmetric_eigen(mut a: Vec<Vec<f64>>) -> (Vec<f64>, Vec<Vec<f64>>) {
    let n = a.len();
    let mut v: Vec<Vec<f64>> = (0..n).map(|i| (0..n).map(|j| if i == j { 1.0 } else { 0.0 }).collect()).collect();
    for _ in 0..50 {
        let off: f64 = (0..n).flat_map(|p| (p + 1..n).map(move |q| (p, q))).map(|(p, q)| a[p][q] * a[p][q]).sum();
        let diagonal: f64 = (0..n).map(|i| a[i][i] * a[i][i]).sum();
        if !(off > 1e-30 * diagonal) {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                if a[p][q] == 0.0 {
                    continue;
                }
                let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..n {
                    (a[k][p], a[k][q]) = (c * a[k][p] - s * a[k][q], s * a[k][p] + c * a[k][q]);
                }
                for k in 0..n {
                    (a[p][k], a[q][k]) = (c * a[p][k] - s * a[q][k], s * a[p][k] + c * a[q][k]);
                    (v[k][p], v[k][q]) = (c * v[k][p] - s * v[k][q], s * v[k][p] + c * v[k][q]);
                }
            }
        }
    }
    ((0..n).map(|i| a[i][i]).collect(), v)
}

/// The 2-norm condition number of the 4 x 4 Jacobian in (x, y, z, h): the square root of the largest over the smallest
/// eigenvalue of J^T J. In h, the half slide, a part moves as far as the plate does in x, y or z: the tool's turn c,
/// in rad, would weigh by the amplifier's ratio instead.
pub fn condition_number(j: &M4) -> f64 {
    let jtj: Vec<Vec<f64>> =
        (0..4).map(|r| (0..4).map(|c| (0..4).map(|k| j[k][r] * j[k][c]).sum()).collect()).collect();
    let (eigen, _) = symmetric_eigen(jtj);
    let (smallest, largest) = eigen.iter().fold((f64::INFINITY, 0f64), |(lo, hi), e| (lo.min(*e), hi.max(*e)));
    if smallest <= 0.0 || !smallest.is_finite() { f64::INFINITY } else { (largest / smallest).sqrt() }
}

/// Which side of the slide each leg's plate part is on: -1 for part A (arms 1 and 2), +1 for part B (arms 3 and 4).
pub const SIDE: [f64; 4] = [-1.0, -1.0, 1.0, 1.0];

/// The leg geometry and the kinematics, apart from masses and drives.
#[derive(Debug, Clone)]
pub struct Legs {
    /// Base radius less plate radius, upper arm, forearm.
    pub e: f64,
    pub la: f64,
    pub lb: f64,
    /// cos and sin of the legs' azimuths, 0, 90, 180 and 270 degrees.
    pub azimuth: [(f64, f64); 4],
    /// The direction the parts slide along, d.
    pub slide: V3,
}

impl Legs {
    pub fn new(e: f64, la: f64, lb: f64) -> Self {
        let azimuth = [0.0f64, 90.0, 180.0, 270.0].map(|deg| (deg.to_radians().cos(), deg.to_radians().sin()));
        let s = std::f64::consts::FRAC_1_SQRT_2;
        Self { e, la, lb, azimuth, slide: [s, -s, 0.0] }
    }

    /// A vector in leg i's frame turned into the base frame (R_z(phi_i) v), and back.
    fn to_base(&self, i: usize, v: V3) -> V3 {
        let (c, s) = self.azimuth[i];
        [c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]]
    }
    fn to_leg(&self, i: usize, v: V3) -> V3 {
        let (c, s) = self.azimuth[i];
        [c * v[0] + s * v[1], -s * v[0] + c * v[1], v[2]]
    }

    /// The elbow of leg i at arm angle t, and its first and second derivative by t.
    pub fn elbow(&self, i: usize, t: f64) -> [V3; 3] {
        let (s, c) = t.sin_cos();
        let la = self.la;
        [
            self.to_base(i, [self.e + la * c, 0.0, -la * s]),
            self.to_base(i, [-la * s, 0.0, -la * c]),
            self.to_base(i, [-la * c, 0.0, la * s]),
        ]
    }

    /// The direction of leg i's elbow axis: the rods of a pair are apart along it.
    pub fn tangent(&self, i: usize) -> V3 {
        self.to_base(i, [0.0, 1.0, 0.0])
    }

    /// Where leg i's forearm pair meets its plate part at x = (p, h), the plate radius folded into e.
    pub fn end(&self, i: usize, x: V4) -> V3 {
        std::array::from_fn(|k| x[k] + SIDE[i] * x[3] * self.slide[k])
    }

    /// Arm angles for x = (p, h), elbows out; `None` out of reach.
    pub fn inverse(&self, x: V4) -> Option<V4> {
        let mut out = [0.0; 4];
        for (i, t) in out.iter_mut().enumerate() {
            let [lx, ly, lz] = self.to_leg(i, self.end(i, x));
            let (a, b, c) = (
                -2.0 * self.la * (lx - self.e),
                2.0 * self.la * lz,
                (lx - self.e).powi(2) + ly * ly + lz * lz + self.la * self.la - self.lb * self.lb,
            );
            let d = a * a + b * b - c * c;
            if !(d >= 0.0) {
                return None;
            }
            // tan(t/2) = (-b - sqrt d) / (c - a) = (a + c) / (sqrt d - b): the second where the first's terms cancel
            // (b < 0, the plate below the arms; near a = c it lost 2e-11 rad).
            *t = 2.0 * if b < 0.0 { (a + c) / (d.sqrt() - b) } else { (-b - d.sqrt()) / (c - a) }.atan();
        }
        Some(out)
    }

    /// x = (p, h) for arm angles: Newton on the four constraints, from the lower point at a forearm's length from the
    /// first three elbows (h = 0). `None` where the legs cannot be assembled with the forearms down from the elbows.
    pub fn forward(&self, t: V4) -> Option<V4> {
        let c: [V3; 3] = std::array::from_fn(|i| self.elbow(i, t[i])[0]);
        let ex = sub(c[1], c[0]);
        let d = norm(ex);
        let ex = ex.map(|v| v / d);
        let to3 = sub(c[2], c[0]);
        let i = dot(ex, to3);
        let ey = sub(to3, ex.map(|v| v * i));
        let ey_len = norm(ey);
        if !(d > 1e-12 && ey_len > 1e-12) {
            return None;
        }
        let ey = ey.map(|v| v / ey_len);
        let ez = cross(ex, ey);
        let j = dot(ey, to3);
        let x = d / 2.0;
        let y = (i * i + j * j - 2.0 * i * x) / (2.0 * j);
        // Short of reach at h = 0 the start is the three elbows' plane; Newton goes on from there.
        let z = (self.lb * self.lb - x * x - y * y).max(0.0).sqrt();
        let base: V3 = std::array::from_fn(|k| c[0][k] + x * ex[k] + y * ey[k]);
        let (up, down): (V3, V3) =
            (std::array::from_fn(|k| base[k] + z * ez[k]), std::array::from_fn(|k| base[k] - z * ez[k]));
        let start = if up[2] < down[2] { up } else { down };
        self.forward_from(t, [start[0], start[1], start[2], 0.0])
    }

    /// `forward` by Newton from `x`: J_x dx = -(|u_k|^2 - L_b^2) / 2.
    pub fn forward_from(&self, t: V4, mut x: V4) -> Option<V4> {
        let lb2 = self.lb * self.lb;
        for _ in 0..50 {
            let (jx, _) = self.constraint(t, x);
            let r: V4 = std::array::from_fn(|k| {
                let u = [jx[k][0], jx[k][1], jx[k][2]];
                -0.5 * (dot(u, u) - lb2)
            });
            let dx = solve4(&jx, r)?;
            for k in 0..4 {
                x[k] += dx[k];
            }
            if dx.iter().all(|v| v.abs() < 1e-14) {
                break;
            }
        }
        let (jx, _) = self.constraint(t, x);
        (0..4)
            .all(|k| {
                let u = [jx[k][0], jx[k][1], jx[k][2]];
                (dot(u, u) - lb2).abs() < 1e-9 * lb2 && u[2] < 0.0
            })
            .then_some(x)
    }

    /// J_x (rows: the forearm vectors u_k = q_k - c_k and sigma_k u_k . d) and the diagonal of J_q (u_k . dc_k/dt).
    fn constraint(&self, t: V4, x: V4) -> (M4, V4) {
        let (mut jx, mut jq) = ([[0.0; 4]; 4], [0.0; 4]);
        for k in 0..4 {
            let [c, d1, _] = self.elbow(k, t[k]);
            let u = sub(self.end(k, x), c);
            jx[k] = [u[0], u[1], u[2], SIDE[k] * dot(u, self.slide)];
            jq[k] = dot(u, d1);
        }
        (jx, jq)
    }

    /// The Jacobian J[i][j] = dx_i / dtheta_j, x = (p, h), at arm angles t and x; `None` at a direct singularity.
    pub fn jacobian(&self, t: V4, x: V4) -> Option<M4> {
        let (jx, jq) = self.constraint(t, x);
        let inv = inverse4(&jx)?;
        Some(std::array::from_fn(|i| std::array::from_fn(|j| inv[i][j] * jq[j])))
    }

    /// J' theta' = J_x^-1 w: the acceleration of x at arm speeds `wt` and no arm acceleration, x moving at `xv`.
    pub fn velocity_term(&self, t: V4, x: V4, wt: V4, xv: V4) -> Option<V4> {
        let (jx, _) = self.constraint(t, x);
        let w: V4 = std::array::from_fn(|k| {
            let [_, d1, d2] = self.elbow(k, t[k]);
            let rel: V3 = std::array::from_fn(|i| xv[i] + SIDE[k] * xv[3] * self.slide[i] - d1[i] * wt[k]);
            wt[k] * wt[k] * dot([jx[k][0], jx[k][1], jx[k][2]], d2) - dot(rel, rel)
        });
        solve4(&jx, w)
    }

    /// The two rods of leg i's forearm pair, elbow end to plate end, at arm angle t and x, as drawn: `spacing` apart
    /// along the elbow's axis, their ends at the base radius (the legs' elbow is at e, `r` the plate radius added back)
    /// and at the plate part's.
    pub fn rods(&self, i: usize, t: f64, x: V4, r: f64, spacing: f64) -> [(V3, V3); 2] {
        let (c, sn) = self.azimuth[i];
        let (elbow, end) = (self.elbow(i, t)[0], self.end(i, x));
        let half = [-0.5 * spacing * sn, 0.5 * spacing * c, 0.0];
        let (arm_end, plate_end) =
            ([elbow[0] + r * c, elbow[1] + r * sn, elbow[2]], [end[0] + r * c, end[1] + r * sn, end[2]]);
        [-1.0, 1.0].map(|side| {
            (
                std::array::from_fn(|k| arm_end[k] + side * half[k]),
                std::array::from_fn(|k| plate_end[k] + side * half[k]),
            )
        })
    }
}

/// The amplifier's spring between the slide and the tool (effects.amplifier_compliance), at the tool: N m/rad, N m s/rad,
/// and half its play (effects.amplifier_backlash; 0 without), rad.
#[derive(Debug, Clone, Copy)]
pub struct Amplifier {
    pub stiffness: f64,
    pub damping: f64,
    pub half_backlash: f64,
}

impl Amplifier {
    /// The twist beyond the play, with its sign; 0 within the play.
    fn beyond(&self, twist: f64) -> f64 {
        (twist.abs() - self.half_backlash).max(0.0) * twist.signum()
    }
    fn contact(&self, twist: f64) -> bool {
        self.half_backlash == 0.0 || twist.abs() > self.half_backlash
    }
    /// The torque on the tool at a twist (kappa h - phi) and its rate.
    pub fn torque(&self, twist: f64, rate: f64) -> f64 {
        if self.contact(twist) { self.stiffness * self.beyond(twist) + self.damping * rate } else { 0.0 }
    }
    pub fn energy(&self, twist: f64) -> f64 {
        0.5 * self.stiffness * self.beyond(twist).powi(2)
    }
    pub fn loss_power(&self, twist: f64, rate: f64) -> f64 {
        if self.contact(twist) { self.damping * rate * rate } else { 0.0 }
    }
}

pub struct H4 {
    legs: Legs,
    /// The geometry as configured, for the scene: base and plate radius, the rods' spacing.
    base_radius: f64,
    plate_radius: f64,
    rod_spacing: f64,
    elastic: bool,
    g: f64,
    /// The arm's inertia about its joint and its m_LG, the elbow's share of the forearms included, rotor not.
    arm_inertia: f64,
    arm_moment: f64,
    /// The mass at each forearm's middle and its rods' spin inertia (both rods; effects.forearm_inertia, else 0).
    m_rod_middle: f64,
    rod_spin: f64,
    /// Each plate part with its forearms' share, the carriage, the payload.
    m_part: f64,
    m_carriage: f64,
    m_payload: f64,
    /// The tool's turn per half slide, c / h, rad/m.
    kappa: f64,
    /// The tool, and the payload's, inertia about the tool's axis.
    j_tool: f64,
    j_payload: f64,
    /// effects.amplifier_compliance.
    amplifier: Option<Amplifier>,
    payload_attached: bool,
    brake_engaged: bool,
    joints: [RotaryJoint; 4],
    thermal: Option<ThermalFeedback>,
    /// The working range: radius, top, bottom sphere's centre height and radius; the condition number refused above.
    radius: f64,
    top: f64,
    sphere: (f64, f64),
    max_condition: f64,
    axes: [AxisInfo; 4],
    motors: [MotorInfo; 4],
    /// `Machine::setpoint_slack`, worked out once (`rest_sag`).
    rest_slack: f64,
}

// Elastic state: [theta 1..4, omega 1..4, rotor angle 1..4, rotor speed 1..4, brake 1..4], then with
// effects.amplifier_compliance the tool's angle and speed. Rigid: [theta 1..4, omega 1..4, brake 1..4], then the tool's.
const N_ELASTIC: usize = 20;
const N_RIGID: usize = 12;

/// The losses of one moment: per joint its friction, its motor's friction with the brake, its gearbox; the stops, what
/// the ripple gives off, the amplifier's damping; and per joint the gearbox loss as a torque at the rotor.
struct Losses {
    powers: [f64; 15],
    gear_at_rotor: V4,
}

impl H4 {
    pub fn new(cfg: &Config) -> Result<Self, ConfigError> {
        let base_radius = cfg.positive("geometry.base_radius_m")?;
        let plate_radius = cfg.non_negative("geometry.plate_radius_m")?;
        let (la, lb) = (cfg.positive("geometry.upper_arm_m")?, cfg.positive("geometry.forearm_m")?);
        if lb <= la {
            return Err(ConfigError::Invalid("geometry.forearm_m".into()));
        }
        let share = cfg.non_negative("mass.forearm_elbow_share")?;
        if share > 1.0 {
            return Err(ConfigError::Invalid("mass.forearm_elbow_share".into()));
        }
        let (m_arm, cg, i_arm) = (
            cfg.positive("mass.upper_arm_kg")?,
            cfg.non_negative("mass.upper_arm_cg_m")?,
            cfg.positive("mass.upper_arm_inertia_kgm2")?,
        );
        let (m_elbow, m_fb) = (cfg.non_negative("mass.elbow_kg")?, cfg.non_negative("mass.forearm_pair_kg")?);
        // A rigid rod: a sixth at each end, two thirds at its middle (see the crate's doc).
        let rods = cfg.bool("effects.forearm_inertia")?;
        let (share, middle) = if rods { (1.0 / 6.0, 2.0 / 3.0) } else { (share, 0.0) };
        let elbow = m_elbow + share * m_fb;
        let spin = cfg.non_negative("mass.forearm_rod_spin_inertia_kgm2")?;
        let kappa = cfg.positive("plate.gear_ratio")? / cfg.positive("plate.pinion_radius_m")?;
        let amplifier = Amplifier {
            stiffness: cfg.positive("plate.amplifier_stiffness_nm_rad")?,
            damping: cfg.non_negative("plate.amplifier_damping_nm_s_rad")?,
            half_backlash: 0.5 * cfg.non_negative("plate.amplifier_backlash_rad")?,
        };
        let amplifier = Amplifier {
            half_backlash: if cfg.bool("effects.amplifier_backlash")? { amplifier.half_backlash } else { 0.0 },
            ..amplifier
        };
        let (radius, top) = (cfg.positive("workspace.radius_m")?, cfg.f64("workspace.top_z_m")?);
        let (edge, centre) = (cfg.positive("workspace.height_edge_m")?, cfg.positive("workspace.height_centre_m")?);
        if centre < edge {
            return Err(ConfigError::Invalid("workspace.height_centre_m".into()));
        }
        // The bottom: a sphere through the centre's depth and the edge's ring; flat where both heights are the same.
        let sag = centre - edge;
        let sphere = if sag > 0.0 {
            let r = (radius * radius + sag * sag) / (2.0 * sag);
            (top - centre + r, r)
        } else {
            (f64::INFINITY, f64::INFINITY)
        };
        let joint = RotaryJoint::from_config(cfg)?;
        let axis = |id: &'static str, key: &'static str, min: f64, max: f64| -> Result<AxisInfo, ConfigError> {
            Ok(AxisInfo {
                id,
                key,
                unit: "m",
                min,
                max,
                max_vel: cfg.positive(&format!("axis.{id}.max_velocity_m_s"))?,
                max_acc: cfg.positive(&format!("axis.{id}.max_acceleration_m_s2"))?,
                max_jerk: cfg.positive(&format!("axis.{id}.max_jerk_m_s3"))?,
            })
        };
        let (c_min, c_max) =
            (cfg.f64("axis.c.min_angle_deg")?.to_radians(), cfg.f64("axis.c.max_angle_deg")?.to_radians());
        if c_min >= c_max {
            return Err(ConfigError::Invalid("axis.c.max_angle_deg".into()));
        }
        let mut h4 = Self {
            legs: Legs::new(base_radius - plate_radius, la, lb),
            base_radius,
            plate_radius,
            rod_spacing: cfg.non_negative("geometry.forearm_spacing_m")?,
            elastic: cfg.bool("effects.joint_elasticity")?,
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            arm_inertia: i_arm + la * la * elbow,
            arm_moment: m_arm * cg + la * elbow,
            m_rod_middle: middle * m_fb,
            rod_spin: if rods { 2.0 * spin } else { 0.0 },
            m_part: cfg.non_negative("mass.plate_part_kg")? + 2.0 * (1.0 - share - middle) * m_fb,
            m_carriage: cfg.positive("mass.carriage_kg")?,
            m_payload: if cfg.bool("effects.payload")? { cfg.non_negative("mass.payload_kg")? } else { 0.0 },
            kappa,
            j_tool: cfg.positive("tool.inertia_kgm2")?,
            j_payload: if cfg.bool("effects.payload")? { cfg.non_negative("tool.payload_inertia_kgm2")? } else { 0.0 },
            amplifier: cfg.bool("effects.amplifier_compliance")?.then_some(amplifier),
            payload_attached: true,
            brake_engaged: false,
            joints: [joint.clone(), joint.clone(), joint.clone(), joint],
            thermal: ThermalFeedback::from_config(cfg)?,
            radius,
            top,
            sphere,
            max_condition: cfg.positive("workspace.max_condition_number")?,
            axes: [
                axis("x", "axis-x", -radius, radius)?,
                axis("y", "axis-y", -radius, radius)?,
                axis("z", "axis-z", top - centre, top)?,
                AxisInfo {
                    id: "c",
                    key: "axis-c",
                    unit: "rad",
                    min: c_min,
                    max: c_max,
                    max_vel: cfg.positive("axis.c.max_velocity_rad_s")?,
                    max_acc: cfg.positive("axis.c.max_acceleration_rad_s2")?,
                    max_jerk: cfg.positive("axis.c.max_jerk_rad_s3")?,
                },
            ],
            motors: [
                MotorInfo { id: "1", key: "motor-arm-1" },
                MotorInfo { id: "2", key: "motor-arm-2" },
                MotorInfo { id: "3", key: "motor-arm-3" },
                MotorInfo { id: "4", key: "motor-arm-4" },
            ],
            rest_slack: 0.0,
        };
        h4.rest_slack = h4.rest_sag();
        Ok(h4)
    }

    pub fn legs(&self) -> &Legs {
        &self.legs
    }

    /// The tool's turn per half slide, c / h, rad/m.
    pub fn kappa(&self) -> f64 {
        self.kappa
    }

    fn ratio(&self) -> f64 {
        self.joints[0].ratio()
    }

    /// The mass at the tool's point: carriage and, while held, the payload.
    pub fn carried(&self) -> f64 {
        self.m_carriage + if self.payload_attached { self.m_payload } else { 0.0 }
    }

    /// Each plate part with its forearms' share.
    pub fn part_mass(&self) -> f64 {
        self.m_part
    }

    /// What the tool's turn moves: tool and, while held, the payload.
    pub fn tool_inertia(&self) -> f64 {
        self.j_tool + if self.payload_attached { self.j_payload } else { 0.0 }
    }

    /// The arm's inertia about its joint and its m_LG (the gravity torque is g m_LG cos theta), rotor not included.
    pub fn arm_inertia(&self) -> f64 {
        self.arm_inertia
    }
    pub fn arm_moment(&self) -> f64 {
        self.arm_moment
    }

    /// x = (p, h) of a tool pose.
    fn coordinates(&self, tool: &[f64]) -> V4 {
        [tool[0], tool[1], tool[2], tool[3] / self.kappa]
    }

    /// Arm angles and speeds of a state: at the joints, the gearboxes' output side.
    fn arms(&self, s: &[f64]) -> (V4, V4) {
        ([s[0], s[1], s[2], s[3]], [s[4], s[5], s[6], s[7]])
    }

    /// Rotor angles and speeds of a state.
    fn rotors(&self, s: &[f64]) -> (V4, V4) {
        if self.elastic {
            ([s[8], s[9], s[10], s[11]], [s[12], s[13], s[14], s[15]])
        } else {
            let i = self.ratio();
            ([s[0] * i, s[1] * i, s[2] * i, s[3] * i], [s[4] * i, s[5] * i, s[6] * i, s[7] * i])
        }
    }

    fn brakes(&self, s: &[f64]) -> V4 {
        let at = if self.elastic { 16 } else { 8 };
        [s[at], s[at + 1], s[at + 2], s[at + 3]]
    }

    /// Where the tool's angle and speed sit in the state with effects.amplifier_compliance.
    fn turn_at(&self) -> usize {
        if self.elastic { N_ELASTIC } else { N_RIGID }
    }

    /// x and the Jacobian at arm angles t; NaN where the legs cannot be assembled.
    fn pose(&self, t: V4) -> (V4, M4) {
        self.legs
            .forward(t)
            .and_then(|x| Some((x, self.legs.jacobian(t, x)?)))
            .unwrap_or(([f64::NAN; 4], [[f64::NAN; 4]; 4]))
    }

    /// The amplifier's twist kappa h - phi and its rate; zero without effects.amplifier_compliance.
    fn twist(&self, s: &[f64]) -> (f64, f64) {
        if self.amplifier.is_none() {
            return (0.0, 0.0);
        }
        let (t, w) = self.arms(s);
        let (x, j) = self.pose(t);
        let at = self.turn_at();
        (self.kappa * x[3] - s[at], self.kappa * dot4(j[3], w) - s[at + 1])
    }

    /// The amplifier's torque on the tool; zero without effects.amplifier_compliance.
    fn amplifier_torque(&self, s: &[f64]) -> f64 {
        self.amplifier.map_or(0.0, |a| {
            let (twist, rate) = self.twist(s);
            a.torque(twist, rate)
        })
    }

    /// The load's mass matrix M, velocity term and gravity Q_g at arm angles t and speeds w: the two plate parts, the
    /// mass `carried` at the tool's point, the tool's turn with the inertia `spin` (0 where the tool is a body of its
    /// own), the rods' middles and spin (effects.forearm_inertia), the arms.
    pub fn load_at(&self, t: V4, w: V4, carried: f64, spin: f64) -> Load {
        let (x, j) = self.pose(t);
        self.load_with(t, x, j, w, carried, spin)
    }

    /// `load_at` with x and the Jacobian known.
    fn load_with(&self, t: V4, x: V4, j: M4, w: V4, carried: f64, spin: f64) -> Load {
        let xv = mul4(&j, w);
        let a0 = self.legs.velocity_term(t, x, w, xv).unwrap_or([f64::NAN; 4]);
        let d = self.legs.slide;
        // A point at p + side h d: its Jacobian and its acceleration at no arm acceleration.
        let point = |side: f64| -> ([V4; 3], V3) {
            (
                std::array::from_fn(|r| std::array::from_fn(|c| j[r][c] + side * d[r] * j[3][c])),
                std::array::from_fn(|r| a0[r] + side * d[r] * a0[3]),
            )
        };
        let mut load = ([[0.0; 4]; 4], [0.0; 4], [0.0; 4]);
        for side in [-1.0, 1.0] {
            let (jp, ap) = point(side);
            add_point(&mut load, self.m_part, &jp, ap, self.g);
        }
        let (jp, ap) = point(0.0);
        add_point(&mut load, carried, &jp, ap, self.g);
        add_turn(&mut load, spin * self.kappa * self.kappa, j[3], a0[3]);
        for i in 0..4 {
            load.0[i][i] += self.arm_inertia;
            load.2[i] += self.g * self.arm_moment * t[i].cos();
            if self.m_rod_middle == 0.0 && self.rod_spin == 0.0 {
                continue;
            }
            // The elbow's Jacobian: dc/dtheta on its own angle; c'' = d2 w^2 at no acceleration.
            let ([elbow, d1, d2], axis) = (self.legs.elbow(i, t[i]), self.legs.tangent(i));
            let (jq, aq) = point(SIDE[i]);
            let je: [V4; 3] = std::array::from_fn(|r| std::array::from_fn(|c| if c == i { d1[r] } else { 0.0 }));
            let ae = d2.map(|v| v * w[i] * w[i]);
            if self.m_rod_middle > 0.0 {
                let jm = std::array::from_fn(|r| std::array::from_fn(|c| 0.5 * (jq[r][c] + je[r][c])));
                add_point(&mut load, self.m_rod_middle, &jm, std::array::from_fn(|k| 0.5 * (aq[k] + ae[k])), self.g);
            }
            if self.rod_spin > 0.0 {
                let ju = std::array::from_fn(|r| std::array::from_fn(|c| jq[r][c] - je[r][c]));
                let accel = std::array::from_fn(|k| aq[k] - ae[k]);
                self.add_spin(&mut load, axis, sub(self.legs.end(i, x), elbow), &ju, &w, accel);
            }
        }
        load
    }

    /// The spin of a forearm pair's rods about their own axis into `load`, as the D3's: the parallelogram keeps the
    /// pair's plane through the elbow's axis a and the forearm u, so a rod turns about u as that plane does. With
    /// sigma = a . u, n = a . (u' x u) and L = |u|, the spin is s = -sigma n / (L (L^2 - sigma^2)) = g . theta',
    /// g = -sigma / (L (L^2 - sigma^2)) J_u^T (u x a); it adds I g g^T to the mass matrix and
    /// I (s0' g + s J_u^T (u' x u) / L^3) to the velocity term (s0' the spin's rate at no arm acceleration, u'' = `accel`).
    fn add_spin(&self, load: &mut Load, a: V3, u: V3, ju: &[V4; 3], w: &V4, accel: V3) {
        let l = self.legs.lb;
        let du: V3 = std::array::from_fn(|r| dot4(ju[r], *w));
        let (sigma, n) = (dot(a, u), dot(a, cross(du, u)));
        let d = l * l - sigma * sigma;
        let s = -sigma * n / (l * d);
        let axis = cross(u, a);
        let g: V4 = std::array::from_fn(|c| -sigma / (l * d) * (0..3).map(|k| ju[k][c] * axis[k]).sum::<f64>());
        let (dsigma, dn) = (dot(a, du), dot(a, cross(accel, u)));
        let ds = -(dsigma * n + sigma * dn) / (l * d) - 2.0 * sigma * sigma * dsigma * n / (l * d * d);
        let turn = cross(du, u).map(|v| v / l.powi(3));
        for r in 0..4 {
            for c in 0..4 {
                load.0[r][c] += self.rod_spin * g[r] * g[c];
            }
            load.1[r] += self.rod_spin * (ds * g[r] + s * (0..3).map(|k| ju[k][r] * turn[k]).sum::<f64>());
        }
    }

    /// The rigid model's load: the tool turning with the slide.
    fn rigid_load(&self, t: V4, w: V4) -> Load {
        self.load_at(t, w, self.carried(), self.tool_inertia())
    }

    /// What the joints carry at state s: the rigid model's load, or with effects.amplifier_compliance the load without
    /// the tool's turn and the amplifier's torque through the slide, kappa J_h^T tau_a, in the velocity term.
    fn joint_load(&self, s: &[f64]) -> Load {
        let (t, w) = self.arms(s);
        if self.amplifier.is_none() {
            return self.rigid_load(t, w);
        }
        let (x, j) = self.pose(t);
        let mut load = self.load_with(t, x, j, w, self.carried(), 0.0);
        let torque = self.amplifier_torque(s);
        for r in 0..4 {
            load.1[r] += self.kappa * j[3][r] * torque;
        }
        load
    }

    /// Rigid model: the arms' acceleration, and per joint the torque into its gearbox (after the rotor's own
    /// acceleration) and out of it, at the arm.
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> (V4, V4, V4) {
        let (t, w) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let (mass, velocity, gravity) = self.joint_load(s);
        let drive: V4 = std::array::from_fn(|k| {
            self.joints[k].rigid_drive(torque[k], rotor[k], rotor_speed[k], brakes[k], self.brake_engaged).0
        });
        let rest: V4 = std::array::from_fn(|k| gravity[k] - velocity[k] + self.joints[k].output_torque(t[k], w[k]));
        let solve = |loss: V4| -> V4 {
            let m: M4 = std::array::from_fn(|r| {
                std::array::from_fn(|c| {
                    mass[r][c] + if r == c { loss[r] * self.joints[r].reflected_inertia() } else { 0.0 }
                })
            });
            solve4(&m, std::array::from_fn(|k| loss[k] * drive[k] + rest[k])).unwrap_or([f64::NAN; 4])
        };
        let first = solve([1.0; 4]);
        let loss: V4 = std::array::from_fn(|k| {
            self.joints[k].gearbox.loss_factor(drive[k] - self.joints[k].reflected_inertia() * first[k], w[k])
        });
        let acc = solve(loss);
        let into: V4 = std::array::from_fn(|k| drive[k] - self.joints[k].reflected_inertia() * acc[k]);
        (acc, into, std::array::from_fn(|k| loss[k] * into[k]))
    }

    /// The arms' acceleration in either model.
    fn arm_acceleration(&self, s: &[f64], torque: &[f64]) -> V4 {
        if self.elastic {
            let mut d = vec![0.0; self.state_len()];
            self.derivatives(s, torque, &mut d);
            [d[4], d[5], d[6], d[7]]
        } else {
            self.rigid_solve(s, torque).0
        }
    }

    /// The torque each gearbox passes to its arm.
    fn gear_output(&self, s: &[f64], torque: &[f64]) -> V4 {
        if self.elastic {
            std::array::from_fn(|k| self.joints[k].gearbox.torque(s[8 + k], s[12 + k], s[k], s[4 + k]).output)
        } else {
            self.rigid_solve(s, torque).2
        }
    }

    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        let (t, w) = self.arms(s);
        let (x, j) = self.pose(t);
        let spin = if self.amplifier.is_some() { 0.0 } else { self.tool_inertia() };
        let (mass, ..) = self.load_with(t, x, j, w, self.carried(), spin);
        let mut kinetic = 0.5 * dot4(w, mul4(&mass, w));
        let middles: f64 = (0..4).map(|i| 0.5 * (self.legs.elbow(i, t[i])[0][2] + x[2])).sum();
        let sines: f64 = t.iter().map(|a| a.sin()).sum();
        let potential = self.g
            * ((2.0 * self.m_part + self.carried()) * x[2] + self.m_rod_middle * middles - self.arm_moment * sines);
        let mut elastic: f64 = (0..4).map(|k| self.joints[k].stop_energy(t[k])).sum();
        if self.elastic {
            for k in 0..4 {
                kinetic += 0.5 * self.joints[k].rotor_inertia * s[12 + k] * s[12 + k];
                elastic += self.joints[k].gearbox.energy(s[8 + k], s[k]);
            }
        } else {
            kinetic += 0.5 * (0..4).map(|k| self.joints[k].reflected_inertia() * w[k] * w[k]).sum::<f64>();
        }
        if let Some(a) = self.amplifier {
            let at = self.turn_at();
            kinetic += 0.5 * self.tool_inertia() * s[at + 1] * s[at + 1];
            elastic += a.energy(self.kappa * x[3] - s[at]);
        }
        [kinetic, potential, elastic]
    }

    fn losses(&self, s: &[f64], torque: &[f64]) -> Losses {
        let (t, w) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let mut powers = [0.0; 15];
        let mut gear_at_rotor = [0.0; 4];
        let rigid = (!self.elastic).then(|| self.rigid_solve(s, torque));
        let (mut stops, mut other) = (0.0, 0.0);
        for k in 0..4 {
            let joint = &self.joints[k];
            let [friction, stop, shaft, drag] = joint.losses(t[k], w[k], rotor_speed[k], brakes[k], self.brake_engaged);
            let gear = match &rigid {
                Some((_, into, out)) => {
                    gear_at_rotor[k] = (into[k] - out[k]) / joint.ratio();
                    (into[k] - out[k]) * w[k]
                }
                None => {
                    let g = joint.gearbox.torque(s[8 + k], s[12 + k], s[k], s[4 + k]);
                    gear_at_rotor[k] = g.rotor - g.output / joint.ratio();
                    joint.gearbox.loss_power(s[8 + k], s[12 + k], s[k], s[4 + k])
                }
            };
            powers[3 * k..3 * k + 3].copy_from_slice(&[friction, shaft, gear + drag]);
            stops += stop;
            other += (torque[k] - joint.ripple.apply(torque[k], rotor[k])) * rotor_speed[k];
        }
        powers[12] = stops;
        powers[13] = other;
        if let Some(a) = self.amplifier {
            let (twist, rate) = self.twist(s);
            powers[14] = a.loss_power(twist, rate);
        }
        Losses { powers, gear_at_rotor }
    }

    /// How many of `Losses::powers` are signals: the amplifier's damping only with effects.amplifier_compliance.
    fn loss_count(&self) -> usize {
        if self.amplifier.is_some() { 15 } else { 14 }
    }

    /// Whether a position is within the working range (cylinder, top, spherical bottom), widened by `slack`.
    fn in_envelope(&self, p: &[f64], slack: f64) -> bool {
        let (centre, r) = self.sphere;
        let below = if centre.is_finite() {
            (p[0] * p[0] + p[1] * p[1] + (p[2] - centre).powi(2)).sqrt() <= r + slack
        } else {
            p[2] >= self.axes[2].min - slack
        };
        p[0].hypot(p[1]) <= self.radius + slack && p[2] <= self.top + slack && below
    }

    /// How far the motors' pose can be from the tool's at rest anywhere in the working range, payload on or off, the
    /// tool's turn counted in rad as the settling check counts it: each gearbox twisted by the torque it holds against
    /// gravity over its stiffness, plus half its backlash either way (all sixteen combinations), through the forward
    /// kinematics. Over a grid of the range: 5 radii, 12 directions, 9 heights and the bottom, the turn at its ends and
    /// its middle; plus a tenth for what the grid steps over and the joints' friction. At least the default 0.1 mm;
    /// that alone in the rigid model, whose gearboxes do not twist. The amplifier's own play does not reach the motors.
    fn rest_sag(&self) -> f64 {
        if !self.elastic {
            return 1e-4;
        }
        let mut most = 0f64;
        let (bottom, top) = (self.axes[2].min, self.top);
        let turns = [self.axes[3].min, 0.5 * (self.axes[3].min + self.axes[3].max), self.axes[3].max];
        for ring in 0..=4 {
            let rho = self.radius * ring as f64 / 4.0;
            for turn in 0..12 {
                let (sin, cos) = (turn as f64 * std::f64::consts::TAU / 12.0).sin_cos();
                let (centre, r) = self.sphere;
                let lowest = if centre.is_finite() { centre - (r * r - rho * rho).max(0.0).sqrt() } else { bottom };
                for z in (0..=8).map(|k| lowest + (top - lowest) * k as f64 / 8.0) {
                    let p = [rho * cos, rho * sin, z];
                    if !self.in_envelope(&p, 1e-9) {
                        continue;
                    }
                    for c in turns {
                        let x = self.coordinates(&[p[0], p[1], p[2], c]);
                        let Some(t) = self.legs.inverse(x) else { continue };
                        let Some(j) = self.legs.jacobian(t, x) else { continue };
                        for m in [self.m_carriage, self.m_carriage + self.m_payload] {
                            let (.., gravity) = self.load_with(t, x, j, [0.0; 4], m, 0.0);
                            let twist: V4 = std::array::from_fn(|k| {
                                (-gravity[k] - self.joints[k].output_torque(t[k], 0.0))
                                    / self.joints[k].gearbox.stiffness
                            });
                            for signs in 0..16 {
                                let half = |k: usize| {
                                    let sign = if signs >> k & 1 == 1 { 1.0 } else { -1.0 };
                                    sign * self.joints[k].gearbox.half_backlash
                                };
                                let moved = std::array::from_fn(|k| t[k] + twist[k] + half(k));
                                if let Some(q) = self.legs.forward_from(moved, x) {
                                    let off: V4 =
                                        std::array::from_fn(|k| (q[k] - x[k]) * if k == 3 { self.kappa } else { 1.0 });
                                    most = most.max(dot4(off, off).sqrt());
                                }
                            }
                        }
                    }
                }
            }
        }
        (1.1 * most).max(1e-4)
    }

    /// The arms' modes with the motors held (by their position loops): the gearboxes' springs k (all alike) against
    /// the load's mass matrix M, the rotors still, so w^2 = k / mu over M's eigenvalues mu, the shape M's eigenvector
    /// v; the damping ratio c / (2 w mu) with the gearboxes' damping c. The tool counts with the slide unless it is on
    /// the amplifier's spring (`amplifier_mode`). Gravity's stiffness and the stops are left out. Frequency in Hz,
    /// damping ratio and shape in the arm angles, lowest first; none in the rigid model or out of reach.
    fn modes_at(&self, tool: &[f64]) -> Vec<(f64, f64, V4)> {
        if !self.elastic || tool.len() != 4 {
            return Vec::new();
        }
        let Some(t) = self.legs.inverse(self.coordinates(tool)) else { return Vec::new() };
        let spin = if self.amplifier.is_some() { 0.0 } else { self.tool_inertia() };
        let (mass, ..) = self.load_at(t, [0.0; 4], self.carried(), spin);
        let (mu, v) = symmetric_eigen(mass.iter().map(|row| row.to_vec()).collect());
        let (k, c) = (self.joints[0].gearbox.stiffness, self.joints[0].gearbox.damping);
        let mut out: Vec<(f64, f64, V4)> = (0..4)
            .filter(|&i| mu[i] > 0.0)
            .map(|i| {
                let w = (k / mu[i]).sqrt();
                (w / std::f64::consts::TAU, c / (2.0 * w * mu[i]), std::array::from_fn(|r| v[r][i]))
            })
            .collect();
        out.sort_by(|a, b| a.0.total_cmp(&b.0));
        out
    }

    /// The tool's turn on the amplifier's spring with the slide held, as the D4's tool: w^2 = k / I_t, damping ratio
    /// c / (2 sqrt(k I_t)). `None` without effects.amplifier_compliance.
    pub fn amplifier_mode(&self) -> Option<Mode> {
        let a = self.amplifier?;
        let j = self.tool_inertia();
        Some(Mode {
            hz: (a.stiffness / j).sqrt() / std::f64::consts::TAU,
            damping: a.damping / (2.0 * (a.stiffness * j).sqrt()),
        })
    }

    /// The two plate parts as drawn at x: each a bar between its two forearm pairs' ends (at the plate radius from the
    /// part's centre, at least 30 mm so that a bar is drawn).
    fn plate_bars(&self, x: V4) -> [(V3, V3); 2] {
        let r = self.plate_radius.max(0.03);
        let end = |i: usize| {
            let (q, (c, s)) = (self.legs.end(i, x), self.legs.azimuth[i]);
            [q[0] + r * c, q[1] + r * s, q[2]]
        };
        [(end(0), end(1)), (end(2), end(3))]
    }

    /// The carriage at the tool's point and the payload under it, as `scene` draws them.
    fn on_carriage(&self, p: V3) -> Vec<Part> {
        let mut parts = vec![Part::new("carriage", Shape::Box { size: [0.08, 0.08, 0.05] }, p, [0.55, 0.55, 0.6])];
        if self.payload_attached && self.m_payload > 0.0 {
            parts.push(Part::new(
                "payload",
                Shape::Box { size: [0.12, 0.12, 0.08] },
                [p[0], p[1], p[2] - 0.1],
                [0.2, 0.7, 0.3],
            ));
        }
        parts
    }

    /// Upper arm k from its shoulder to its end, as `scene` draws it: the legs' elbow is at e, the arm's end at the base
    /// radius (the plate's radius added back).
    fn arm(&self, k: usize, t: V4) -> (V3, V3) {
        let (c, sn) = self.legs.azimuth[k];
        let elbow = self.legs.elbow(k, t[k])[0];
        (
            [self.base_radius * c, self.base_radius * sn, 0.0],
            [elbow[0] + self.plate_radius * c, elbow[1] + self.plate_radius * sn, elbow[2]],
        )
    }

    /// The tool's turn and its speed at state s.
    fn turn(&self, s: &[f64]) -> (f64, f64) {
        if self.amplifier.is_some() {
            let at = self.turn_at();
            return (s[at], s[at + 1]);
        }
        let (t, w) = self.arms(s);
        let (x, j) = self.pose(t);
        (self.kappa * x[3], self.kappa * dot4(j[3], w))
    }

    /// The torque breakdown, loss powers, energies of the effect views.
    fn view_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (_, w) = self.arms(s);
        let (_, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let (mass, velocity, gravity) = self.joint_load(s);
        let acc = self.arm_acceleration(s, torque);
        let moving = mul4(&mass, acc);
        let losses = self.losses(s, torque);
        for k in 0..4 {
            let joint = &self.joints[k];
            let i = joint.ratio();
            let held = joint.brake.torque(self.brake_engaged, rotor_speed[k], brakes[k]).0;
            let parts = [
                (moving[k] + velocity[k] + joint.reflected_inertia() * acc[k]) / i,
                -gravity[k] / i,
                joint.shaft.torque(rotor_speed[k]) + held + losses.gear_at_rotor[k] + joint.friction.torque(w[k]) / i,
            ];
            out.extend(parts);
            out.push(torque[k] - parts.iter().sum::<f64>());
        }
        out.extend(&losses.powers[..self.loss_count()]);
        out.extend(self.energy_parts(s));
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
            let gear = if self.joints[k].gearbox.is_coupling() {
                "quantity-coupling-loss-power"
            } else {
                "quantity-gearbox-loss-power"
            };
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}joint_{}", motor.id),
                ARMS[k],
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
        if self.amplifier.is_some() {
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}amplifier"), "group-plate", "quantity-damping-power", "W"));
        }
        out.extend(ENERGY_PARTS.map(|part| {
            SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")
        }));
    }
}

/// The arms' translation keys, in the motors' order.
const ARMS: [&str; 4] = ["arm-1", "arm-2", "arm-3", "arm-4"];

impl Machine for H4 {
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    /// NaN where the pose is out of reach: the controller never asks for one (see `unreachable`).
    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        let t = self.legs.inverse(self.coordinates(tool)).unwrap_or([f64::NAN; 4]);
        for k in 0..4 {
            motors[k] = self.ratio() * t[k];
        }
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        let i = self.ratio();
        let x = self.legs.forward(std::array::from_fn(|k| motors[k] / i)).unwrap_or([f64::NAN; 4]);
        tool[..3].copy_from_slice(&x[..3]);
        tool[3] = self.kappa * x[3];
    }

    fn state_len(&self) -> usize {
        self.turn_at() + if self.amplifier.is_some() { 2 } else { 0 }
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let t = self.legs.inverse(self.coordinates(tool)).unwrap_or([f64::NAN; 4]);
        let (.., gravity) = self.rigid_load(t, [0.0; 4]);
        state.clear();
        state.extend(t);
        state.extend([0.0; 4]);
        for k in 0..4 {
            let joint = &self.joints[k];
            // At rest the gearbox holds the arm against gravity and the stops.
            let out = -gravity[k] - joint.output_torque(t[k], 0.0);
            let rotor = joint.ratio() * (t[k] + if self.elastic { joint.gearbox.static_twist(out) } else { 0.0 });
            if self.elastic {
                state.push(rotor);
            }
            torque[k] = joint.ripple.invert(out / joint.ratio(), rotor);
        }
        // Rotor speeds and brakes at rest; the tool at its turn, its spring unloaded (nothing turns it at rest).
        state.resize(self.turn_at(), 0.0);
        if self.amplifier.is_some() {
            state.extend([tool[3], 0.0]);
        }
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        if self.amplifier.is_some() {
            let at = self.turn_at();
            out[at] = s[at + 1];
            out[at + 1] = self.amplifier_torque(s) / self.tool_inertia();
        }
        if !self.elastic {
            let (acc, ..) = self.rigid_solve(s, torque);
            let (_, rotor_speed) = self.rotors(s);
            let brakes = self.brakes(s);
            out[..4].copy_from_slice(&s[4..8]);
            out[4..8].copy_from_slice(&acc);
            for k in 0..4 {
                out[8 + k] = self.joints[k].brake.torque(self.brake_engaged, rotor_speed[k], brakes[k]).1;
            }
            return;
        }
        let (t, w) = self.arms(s);
        let (mass, velocity, gravity) = self.joint_load(s);
        let mut rhs = [0.0; 4];
        for k in 0..4 {
            let joint = &self.joints[k];
            let (gear, rotor_acc, brake_rate) =
                joint.rotor(torque[k], s[8 + k], s[12 + k], t[k], w[k], s[16 + k], self.brake_engaged);
            rhs[k] = gear.output + gravity[k] - velocity[k] + joint.output_torque(t[k], w[k]);
            out[8 + k] = s[12 + k];
            out[12 + k] = rotor_acc;
            out[16 + k] = brake_rate;
        }
        out[..4].copy_from_slice(&s[4..8]);
        out[4..8].copy_from_slice(&solve4(&mass, rhs).unwrap_or([f64::NAN; 4]));
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (t, w) = self.arms(s);
        let (x, j) = self.pose(t);
        let v = mul4(&j, w);
        pos[..3].copy_from_slice(&x[..3]);
        vel[..3].copy_from_slice(&v[..3]);
        (pos[3], vel[3]) = self.turn(s);
    }
    /// The arms turn at the speed, the tool on the amplifier's spring with them.
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        let (t, _) = self.arms(s);
        let (_, j) = self.pose(t);
        let w = solve4(&j, self.coordinates(vel)).unwrap_or([f64::NAN; 4]);
        s[4..8].copy_from_slice(&w);
        if self.elastic {
            for k in 0..4 {
                s[12 + k] = self.joints[k].ratio() * w[k];
            }
        }
        if self.amplifier.is_some() {
            let at = self.turn_at();
            s[at + 1] = vel[3];
        }
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (rotor, speed) = self.rotors(s);
        pos[..4].copy_from_slice(&rotor);
        vel[..4].copy_from_slice(&speed);
    }

    /// Rigid, loss-free, at rest: M theta'' - Q_g at the arm over the ratio, plus the rotor's own J_rotor i theta'',
    /// theta'' = J^-1 (p'', c'' / kappa).
    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        let x = self.coordinates(pos);
        let t = self.legs.inverse(x).unwrap_or([f64::NAN; 4]);
        let j = self.legs.jacobian(t, x).unwrap_or([[f64::NAN; 4]; 4]);
        let (mass, _, gravity) = self.load_with(t, x, j, [0.0; 4], self.carried(), self.tool_inertia());
        let arm_acc = solve4(&j, self.coordinates(acc)).unwrap_or([f64::NAN; 4]);
        let at_arm = mul4(&mass, arm_acc);
        for k in 0..4 {
            let joint = &self.joints[k];
            out[k] = (at_arm[k] - gravity[k]) / joint.ratio() + joint.rotor_inertia * joint.ratio() * arm_acc[k];
        }
    }

    /// With the speed: theta' = J^-1 x', theta'' = J^-1 (x'' - J_x^-1 w), and at the arm M theta'' + v - Q_g, over the
    /// ratio, plus the rotor's own J_rotor i theta''. The tool turns with the slide.
    fn feedforward_torque_moving(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut [f64]) {
        let x = self.coordinates(pos);
        let t = self.legs.inverse(x).unwrap_or([f64::NAN; 4]);
        let j = self.legs.jacobian(t, x).unwrap_or([[f64::NAN; 4]; 4]);
        let (xv, xa) = (self.coordinates(vel), self.coordinates(acc));
        let w = solve4(&j, xv).unwrap_or([f64::NAN; 4]);
        let bend = self.legs.velocity_term(t, x, w, xv).unwrap_or([f64::NAN; 4]);
        let arm_acc = solve4(&j, std::array::from_fn(|i| xa[i] - bend[i])).unwrap_or([f64::NAN; 4]);
        let (mass, velocity, gravity) = self.load_with(t, x, j, w, self.carried(), self.tool_inertia());
        let at_arm = mul4(&mass, arm_acc);
        for k in 0..4 {
            let joint = &self.joints[k];
            out[k] = (at_arm[k] + velocity[k] - gravity[k]) / joint.ratio()
                + joint.rotor_inertia * joint.ratio() * arm_acc[k];
        }
    }

    fn energy(&self, s: &[f64]) -> f64 {
        self.energy_parts(s).iter().sum()
    }

    fn set_payload_attached(&mut self, attached: bool) {
        self.payload_attached = attached;
    }
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.brake_engaged = engaged;
    }

    /// The gearboxes' hysteresis states move on to where the step ended (`Gearbox::commit`).
    fn after_step(&mut self, s: &mut [f64], _dt: f64) {
        if self.elastic {
            for k in 0..4 {
                self.joints[k].gearbox.commit(s[8 + k], s[k]);
            }
        }
    }

    /// The gearboxes, in the order of `thermal_parts`.
    fn set_part_temperatures(&mut self, rises: &[f64]) {
        let Some(law) = &self.thermal else { return };
        if self.joints[0].gearbox.is_coupling() {
            return;
        }
        for (joint, rise) in self.joints.iter_mut().zip(rises) {
            joint.gearbox.efficiency = law.gearbox_efficiency(*rise);
        }
    }

    /// What the gearboxes can sag at rest anywhere in the working range (`rest_sag`).
    fn setpoint_slack(&self) -> f64 {
        self.rest_slack
    }

    fn unreachable(&self, tool: &[f64], slack: f64) -> Option<&'static str> {
        if tool.len() != 4 {
            return Some("error-target-out-of-range");
        }
        if !self.in_envelope(tool, slack) {
            return Some("error-delta-outside-envelope");
        }
        let x = self.coordinates(tool);
        let Some(t) = self.legs.inverse(x) else { return Some("error-delta-out-of-reach") };
        let (min, max) = self.joints[0].range;
        if t.iter().any(|a| *a < min || *a > max) {
            return Some("error-delta-joint-stop");
        }
        match self.legs.jacobian(t, x) {
            Some(j) if condition_number(&j) <= self.max_condition => None,
            _ => Some("error-delta-singularity"),
        }
    }

    /// Of the Jacobian in (x, y, z, h), elbows out (`condition_number`), as the signal `kinematics.condition` and the
    /// check against `workspace.max_condition_number`.
    fn condition(&self, tool: &[f64]) -> Option<f64> {
        let x = self.coordinates(tool);
        let j = self.legs.inverse(x).and_then(|t| self.legs.jacobian(t, x));
        Some(j.map_or(f64::INFINITY, |j| condition_number(&j)))
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out = Vec::new();
        for (k, motor) in self.motors.iter().enumerate() {
            out.push(SignalInfo::new(format!("arm.{}.angle", motor.id), ARMS[k], "quantity-arm-angle", "rad"));
            out.push(SignalInfo::new(
                format!("gearbox.{}.output_torque", motor.id),
                motor.key,
                "quantity-gearbox-torque",
                "N m",
            ));
        }
        out.push(SignalInfo::new("kinematics.condition", "group-kinematics", "quantity-condition-number", ""));
        out.push(SignalInfo::new("plate.slide", "group-plate", "quantity-plate-slide", "m"));
        if self.amplifier.is_some() {
            out.push(SignalInfo::new("plate.amplifier_twist", "group-plate", "quantity-amplifier-twist", "rad"));
        }
        self.effect_infos(&mut out);
        out
    }

    /// Per arm its angle and gearbox torque, the condition number, the parts' slide against each other (2 h) and with
    /// effects.amplifier_compliance the amplifier's twist.
    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (t, _) = self.arms(s);
        let gear = self.gear_output(s, torque);
        for k in 0..4 {
            out.extend([t[k], gear[k]]);
        }
        let (x, j) = self.pose(t);
        out.push(condition_number(&j));
        out.push(2.0 * x[3]);
        if self.amplifier.is_some() {
            out.push(self.twist(s).0);
        }
    }
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.view_signals(s, torque, out);
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(&self.losses(s, torque).powers[..self.loss_count()]);
    }

    // shortcut: the joints' friction is not offered to the friction view, as on the D3 (a torque over an angular speed
    // the view cannot draw yet).

    /// The four gearboxes, where there are gearboxes (not on a direct drive).
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        if self.joints[0].gearbox.is_coupling() {
            return Vec::new();
        }
        [
            ("gearbox_1", "motor-arm-1"),
            ("gearbox_2", "motor-arm-2"),
            ("gearbox_3", "motor-arm-3"),
            ("gearbox_4", "motor-arm-4"),
        ]
        .into_iter()
        .enumerate()
        .map(|(k, (id, owner_key))| ThermalPart {
            id,
            kind: "gearbox",
            owner_key,
            scene_part: "gearbox",
            scene_index: k,
        })
        .collect()
    }

    fn scene(&self, s: &[f64], out: &mut Scene) {
        let (t, _) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let (x, _) = self.pose(t);
        let (c, _) = self.turn(s);
        let p = [x[0], x[1], x[2]];
        out.parts.clear();
        out.labels.clear();
        let grey = [0.62, 0.64, 0.68];
        out.parts.push(Part::new(
            "base",
            Shape::Box { size: [2.4 * self.base_radius, 2.4 * self.base_radius, 0.04] },
            [0.0, 0.0, 0.1],
            grey,
        ));
        for k in 0..4 {
            let (cos, sin) = self.legs.azimuth[k];
            let (shoulder, arm_end) = self.arm(k, t);
            let at = |r: f64, z: f64| [r * cos, r * sin, z];
            out.parts.push(Part::new(
                sim_core::scene::MOTOR_PART,
                Shape::Box { size: [0.1, 0.1, 0.14] },
                at(self.base_radius, 0.17),
                [0.9, 0.45, 0.1],
            ));
            out.parts.push(Part::new(
                "gearbox",
                Shape::Box { size: [0.1, 0.1, 0.07] },
                at(self.base_radius, 0.055),
                [0.25, 0.25, 0.28],
            ));
            out.parts.push(
                Part::new(
                    "rotor-bar",
                    Shape::Box { size: [0.08, 0.004, 0.012] },
                    at(self.base_radius, 0.245),
                    [0.1, 0.1, 0.1],
                )
                .turned(rotor[k]),
            );
            out.parts.push(Part::new(
                "arm",
                Shape::Segment { to: arm_end, width: ARM_WIDTH },
                shoulder,
                [0.2, 0.22, 0.26],
            ));
            // The two rods of the forearm pair, apart along the elbow's axis (tangential to the base).
            for (from, to) in self.legs.rods(k, t[k], x, self.plate_radius, self.rod_spacing) {
                out.parts.push(Part::new("forearm", Shape::Segment { to, width: ROD_WIDTH }, from, [0.15, 0.15, 0.15]));
            }
            out.labels.push(Label {
                pos: at(self.base_radius, 0.32),
                owner_key: self.motors[k].key,
                value: rotor_speed[k] * 60.0 / std::f64::consts::TAU,
                unit_key: "unit-rpm",
                decimals: 0,
            });
        }
        for (from, to) in self.plate_bars(x) {
            out.parts.push(Part::new("plate-part", Shape::Segment { to, width: PLATE_WIDTH }, from, [0.55, 0.55, 0.6]));
        }
        out.parts.extend(self.on_carriage(p));
        // A bar under the carriage, turned with the tool.
        let half = [0.06 * c.cos(), 0.06 * c.sin(), 0.0];
        let under = [p[0], p[1], p[2] - 0.04];
        out.parts.push(Part::new(
            "tool",
            Shape::Segment { to: std::array::from_fn(|k| under[k] + half[k]), width: 0.015 },
            std::array::from_fn(|k| under[k] - half[k]),
            [0.85, 0.75, 0.2],
        ));
        out.labels.push(Label {
            pos: [p[0], p[1], p[2] - 0.2],
            owner_key: "axis-z",
            value: p[2],
            unit_key: "unit-m",
            decimals: 3,
        });
    }

    /// The carriage's centre, the tool's point: the machine's frame is the scene's.
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        Some([tool_pose[0], tool_pose[1], tool_pose[2]])
    }

    /// The carriage and the payload under it, the two plate parts, the eight forearm rods and the four upper arms,
    /// where `scene` draws them, without drawing the rest. Each segment is boxes along it, each the box around its piece
    /// of the drawn segment widened by the segment's width.
    // shortcut: a slanted segment's pieces are boxes along the scene's axes, as on the D3; the tool's bar is not a body.
    fn collision_bodies(&self, s: &[f64]) -> Vec<sim_core::Aabb> {
        let (t, _) = self.arms(s);
        let (x, _) = self.pose(t);
        let mut out = sim_core::scene::scene_boxes(&self.on_carriage([x[0], x[1], x[2]]), &["carriage", "payload"]);
        for (from, to) in self.plate_bars(x) {
            out.extend(segment_boxes(from, to, PLATE_WIDTH, PLATE_PIECES));
        }
        for k in 0..4 {
            for (from, to) in self.legs.rods(k, t[k], x, self.plate_radius, self.rod_spacing) {
                out.extend(segment_boxes(from, to, ROD_WIDTH, ROD_PIECES));
            }
            let (shoulder, arm_end) = self.arm(k, t);
            out.extend(segment_boxes(shoulder, arm_end, ARM_WIDTH, ARM_PIECES));
        }
        out
    }

    /// The lowest of the arms' modes (`modes_at`), with the gearboxes' springs (effects.joint_elasticity): what an input
    /// shaper on the path is for. The tool's turn on the amplifier is in `modes`.
    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        self.modes_at(tool_pose).first().map(|mode| mode.0)
    }

    /// The mode of `resonance_hz`: the joints' turns in the arm angles, rotors and tool still; scaled so that the tool
    /// point's largest travel along an axis is 1 m.
    fn mode_shape(&self, tool_pose: &[f64]) -> Option<(f64, Vec<f64>)> {
        let (hz, _, v) = self.modes_at(tool_pose).into_iter().next()?;
        let x = self.coordinates(tool_pose);
        let j = self.legs.jacobian(self.legs.inverse(x)?, x)?;
        let moved = mul4(&j, v);
        let largest = (0..3).fold(0f64, |m, k| m.max(moved[k].abs()));
        let mut shape = vec![0.0; self.state_len()];
        for k in 0..4 {
            shape[k] = v[k] / largest;
        }
        Some((hz, shape))
    }

    /// The arms' four modes (with effects.joint_elasticity) and the tool's turn on the amplifier (with
    /// effects.amplifier_compliance), lowest first.
    fn modes(&self, tool_pose: &[f64]) -> Vec<Mode> {
        let mut out: Vec<Mode> =
            self.modes_at(tool_pose).into_iter().map(|(hz, damping, _)| Mode { hz, damping }).collect();
        out.extend(self.amplifier_mode());
        out.sort_by(|a, b| a.hz.total_cmp(&b.hz));
        out
    }
}

/// `pieces` boxes along the segment from `from` to `to`, each the box around its piece widened by `width`.
fn segment_boxes(from: V3, to: V3, width: f64, pieces: usize) -> impl Iterator<Item = sim_core::Aabb> {
    (0..pieces).map(move |n| {
        let at = |f: f64| -> V3 { std::array::from_fn(|i| from[i] + f * (to[i] - from[i])) };
        let (a, b) = (at(n as f64 / pieces as f64), at((n + 1) as f64 / pieces as f64));
        sim_core::Aabb {
            centre: std::array::from_fn(|i| 0.5 * (a[i] + b[i])),
            size: std::array::from_fn(|i| (b[i] - a[i]).abs() + width),
        }
    })
}

/// The forearm rods', upper arms' and plate parts' width as drawn and as collision bodies, m, and the pieces their
/// collision boxes cut each into.
const ROD_WIDTH: f64 = 0.014;
const ROD_PIECES: usize = 10;
const ARM_WIDTH: f64 = 0.05;
const ARM_PIECES: usize = 10;
const PLATE_WIDTH: f64 = 0.03;
const PLATE_PIECES: usize = 3;

#[cfg(test)]
mod tests;
