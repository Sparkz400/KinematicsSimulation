//! Delta picker D3 (Clavel): three motor-driven arms at 120 degrees on a base, each carrying a parallelogram of two
//! forearms down to a platform that stays level. The tool is the platform's centre, x, y, z.
//!
//! Kinematics per leg i at azimuth phi_i, with e = base radius - platform radius (the platform's offset folded into the
//! base, as the platform never turns), the arm angle theta_i (0 horizontal, positive down), upper arm L_a, forearm L_b:
//!
//! ```text
//! elbow     c_i = R_z(phi_i) (e + L_a cos theta, 0, -L_a sin theta)
//! constraint |p - c_i|^2 = L_b^2
//! inverse   A cos theta + B sin theta + C = 0, A = -2 L_a (x' - e), B = 2 L_a z, C = (x' - e)^2 + y'^2 + z^2 + L_a^2 - L_b^2
//!           theta = 2 atan((-B - sqrt(A^2 + B^2 - C^2)) / (C - A))     (elbow out; (x', y', z) = R_z(-phi_i) p)
//! forward   p = the lower of the two points at L_b from all three elbows (trilateration)
//! Jacobian  J_x p' = J_q theta', J_x rows u_i = p - c_i, J_q = diag(u_i . dc_i/dtheta_i); p' = J theta', J = J_x^-1 J_q
//!           J_x p'' = J_q theta'' + w, w_i = theta_i'^2 (u_i . c_i'') - |p' - c_i' theta_i'|^2
//! ```
//!
//! Dynamics (Codourey 1998 as used by Olsson 2009): the forearms' rotational inertia is left out and each forearm pair's
//! mass m_fb split, a share r at the elbow (it turns with the arm) and 1 - r at the platform. Then the machine is three
//! arms about their joints plus a point mass at the platform, m_nt = platform + payload + 3 (1 - r) m_fb, and by virtual
//! work
//!
//! ```text
//! M(theta) theta'' = tau_gear + Q_g - m_nt J^T J_x^-1 w + tau_joint,   M = I_b 1 + m_nt J^T J
//! I_b = I_arm + L_a^2 (m_elbow + r m_fb),   Q_g,i = g m_LG cos theta_i - m_nt g J_zi,   m_LG = m_arm l_cg + L_a (m_elbow + r m_fb)
//! ```
//!
//! exact for that model: the platform's kinetic energy m_nt |J theta'|^2 / 2 is what M holds, and the velocity term is the
//! platform's m_nt J'theta' through J^T.
//!
//! With effects.forearm_inertia each forearm pair is a rigid slender rod instead (the two rods of a pair move alike, the
//! platform never turning): its kinetic energy (m/6)(|c'|^2 + c'.p' + |p'|^2) is exactly that of m/6 at each end and
//! 2m/3 at its middle, so the rod is m_fb/6 at the elbow, m_fb/6 at the platform and 2 m_fb/3 at its middle
//! (c_i + p)/2, whose Jacobian J_m,i = (J + dc_i/dtheta e_i^T)/2 adds m_mid J_m^T J_m to M, its acceleration at no
//! arm acceleration (J_x^-1 w + c_i'' theta_i'^2)/2 the velocity term and its weight to Q_g. The spin of a rod about its
//! own axis is left out.
//!
//! With effects.link_elasticity each upper arm bends (in the plane it turns in): the elbow turns by delta_i against the
//! arm's root, on a spring k_b = 3 E I / L_a (a cantilever's tip deflection as a turn about the shoulder) with a damper.
//! The arm's tube stays with the root (I_arm, m_arm l_cg), the elbow and all beyond move with the elbow angle
//! theta + delta: I_arm theta'' = tau_gear + g m_arm l_cg cos theta + k_b delta + c_b delta' + tau_joint, and the rest of
//! the machine as above at the elbow angles with the spring's -k_b delta - c_b delta' in place of the gearbox.
//! Each joint is a `RotaryJoint` (gearbox with stiffness, backlash and
//! efficiency, motor with friction, ripple and brake, joint friction, stops). Elastic (effects.joint_elasticity) the
//! rotors are states of their own behind the gearboxes' springs; rigid, each rotor turns with its arm times the ratio and
//! its inertia counts at the arm times the ratio squared, the gearbox losing its efficiency on what passes through it.

use sim_components::joint::RotaryJoint;
use sim_components::mech::{ShaftFriction, ThermalFeedback};
use sim_core::{
    AxisInfo, Config, ConfigError, ENERGY_PARTS, LOSS_PREFIX, Label, Machine, MachineEntry, Mode, MotorInfo, Part,
    Scene, Shape, SignalInfo, TORQUE_PARTS, ThermalPart,
};

pub static ENTRY: MachineEntry = MachineEntry {
    id: "delta-d3",
    key: "machine-delta-d3",
    default_config: include_str!("../default.toml"),
    i18n: &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))],
    build,
    suite_configs,
};

const G: f64 = 9.80665;

/// Effects that take energy out or put it in; the standard suite runs with them off. The stops stay, without their
/// damping: the suite's pulses would otherwise send the arms round beyond where the legs can be assembled.
pub const LOSSY_EFFECTS: [&str; 5] =
    ["joint_friction", "motor_friction", "gearbox_efficiency", "torque_ripple", "holding_brake"];

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    sim_components::base::mount(cfg, Box::new(Delta::new(cfg)?))
}

/// The start pose and the cycle's targets moved by `dz` in z, with the working range (see `suite_configs`).
fn lowered(cfg: &mut Config, dz: f64) {
    let start = cfg.f64s("start.pose").unwrap();
    cfg.set_text(&format!("start.pose = [{:?}, {:?}, {:?}]", start[0], start[1], start[2] + dz)).unwrap();
    let segments: Vec<String> = cfg
        .array("cycle.segment")
        .unwrap()
        .iter()
        .map(|s| {
            let t: Vec<f64> = s["target"].as_array().unwrap().iter().map(|v| v.as_float().unwrap()).collect();
            format!(
                "{{ target = [{:?}, {:?}, {:?}], dwell_s = {:?}, payload = {} }}",
                t[0],
                t[1],
                t[2] + dz,
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
    cfg.set_text("gearbox.damping_nm_s_rad = 0.0").unwrap();
    cfg.set_text("joint.stop_damping_nm_s_rad = 0.0").unwrap();
    cfg.set_text("upper_arm.bending_damping_nm_s_rad = 0.0").unwrap();
    cfg.set_text("upper_arm.lateral_damping_n_s_m = 0.0").unwrap();
    cfg
}

/// Loss-free rigid and elastic models, with and without the payload, and the elastic one with part heating and the
/// temperature feedback (no loss, so the gearboxes stay at ambient).
///
/// Their working range, start pose and cycle lie 175 mm lower than the default's (top 850 mm below the shoulders), so
/// that the suite's middle pose is 1043 mm down: held by constant motor torques, as the suite holds it for 10 s after a pulse, the delta
/// is unstable in its vertical mode in the upper part of its range, where the platform's weight pulls harder through
/// the Jacobian the further it sinks (negative stiffness; `the_vertical_mode_under_constant_torques_is_unstable_high_up`),
/// and stable below about 950 mm. A real property, not a numerical one: the drives' position loops hold it.
fn suite_configs() -> Vec<Config> {
    let mut out = Vec::new();
    for (elastic, payload) in [(false, true), (false, false), (true, true), (true, false)] {
        let mut cfg = lossless_config();
        cfg.set_text("workspace.top_z_m = -0.85").unwrap();
        lowered(&mut cfg, -0.175);
        cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
        cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
        cfg.set_text(&format!("effects.part_heating = {}", elastic && payload)).unwrap();
        cfg.set_text(&format!("effects.thermal_feedback = {}", elastic && payload)).unwrap();
        out.push(cfg);
    }
    out
}

type V3 = [f64; 3];
type M3 = [[f64; 3]; 3];
/// Mass matrix, velocity term and gravity in N coordinates.
type Load<const N: usize> = ([[f64; N]; N], [f64; N], [f64; N]);

/// A point mass m into `load`: its Jacobian jm (3 x N), its acceleration am at no acceleration of the coordinates, its
/// weight down z at g.
fn add_point<const N: usize>(load: &mut Load<N>, m: f64, jm: &[[f64; N]; 3], am: V3, g: f64) {
    for r in 0..N {
        for c in 0..N {
            load.0[r][c] += m * (0..3).map(|k| jm[k][r] * jm[k][c]).sum::<f64>();
        }
        load.1[r] += m * (0..3).map(|k| jm[k][r] * am[k]).sum::<f64>();
        load.2[r] -= m * g * jm[2][r];
    }
}

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

fn mul(m: &M3, v: V3) -> V3 {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}

fn transpose(m: &M3) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| m[j][i]))
}

/// The inverse of a 3 x 3 matrix by its adjugate; `None` where it is singular.
fn inverse(m: &M3) -> Option<M3> {
    let det = dot(m[0], cross(m[1], m[2]));
    if !(det.abs() > 1e-300) || !det.is_finite() {
        return None;
    }
    // The columns of the inverse are the cross products of the rows over the determinant.
    let (c0, c1, c2) = (cross(m[1], m[2]), cross(m[2], m[0]), cross(m[0], m[1]));
    Some(std::array::from_fn(|i| [c0[i] / det, c1[i] / det, c2[i] / det]))
}

/// The 2-norm condition number of a 3 x 3 matrix: the square root of the largest over the smallest eigenvalue of
/// J^T J, by the closed form for a symmetric 3 x 3 (trigonometric solution of the characteristic cubic).
pub fn condition_number(j: &M3) -> f64 {
    let jt = transpose(j);
    let a: M3 = std::array::from_fn(|r| std::array::from_fn(|c| dot(jt[r], jt[c])));
    let q = (a[0][0] + a[1][1] + a[2][2]) / 3.0;
    let p1 = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
    let p2 = (0..3).map(|i| (a[i][i] - q).powi(2)).sum::<f64>() + 2.0 * p1;
    let p = (p2 / 6.0).sqrt();
    if p < 1e-15 * q.abs() {
        return 1.0;
    }
    let b: M3 = std::array::from_fn(|r| std::array::from_fn(|c| (a[r][c] - if r == c { q } else { 0.0 }) / p));
    let half_det = (dot(b[0], cross(b[1], b[2])) / 2.0).clamp(-1.0, 1.0);
    let phi = half_det.acos() / 3.0;
    let (largest, smallest) = (q + 2.0 * p * phi.cos(), q + 2.0 * p * (phi + 2.0 * std::f64::consts::PI / 3.0).cos());
    if smallest <= 0.0 { f64::INFINITY } else { (largest / smallest).sqrt() }
}

/// The leg geometry and the kinematics, apart from masses and drives.
#[derive(Debug, Clone)]
pub struct Legs {
    /// Base radius less platform radius, upper arm, forearm.
    pub e: f64,
    pub la: f64,
    pub lb: f64,
    /// cos and sin of the legs' azimuths, 0, 120 and 240 degrees.
    pub azimuth: [(f64, f64); 3],
}

impl Legs {
    pub fn new(e: f64, la: f64, lb: f64) -> Self {
        let azimuth = [0.0f64, 120.0, 240.0].map(|deg| (deg.to_radians().cos(), deg.to_radians().sin()));
        Self { e, la, lb, azimuth }
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

    /// The direction of leg i's elbow axis: the rods of a pair are apart along it, and the upper arm bends out of the
    /// plane it turns in along it.
    pub fn tangent(&self, i: usize) -> V3 {
        self.to_base(i, [0.0, 1.0, 0.0])
    }

    /// The elbows at arm angles t, each moved by y_i along its tangent (the arms bent out of their planes).
    pub fn elbows_moved(&self, t: V3, y: V3) -> [V3; 3] {
        std::array::from_fn(|i| {
            let (c, tg) = (self.elbow(i, t[i])[0], self.tangent(i));
            std::array::from_fn(|k| c[k] + y[i] * tg[k])
        })
    }

    /// Arm angles for a platform position, elbows out; `None` out of reach.
    pub fn inverse(&self, p: V3) -> Option<V3> {
        self.inverse_moved(p, [0.0; 3])
    }

    /// `inverse` with the elbows moved by y along their tangents: in the leg's frame the platform is y_i nearer.
    pub fn inverse_moved(&self, p: V3, moved: V3) -> Option<V3> {
        let mut out = [0.0; 3];
        for (i, t) in out.iter_mut().enumerate() {
            let [x, y, z] = self.to_leg(i, p);
            let y = y - moved[i];
            let (a, b, c) = (
                -2.0 * self.la * (x - self.e),
                2.0 * self.la * z,
                (x - self.e).powi(2) + y * y + z * z + self.la * self.la - self.lb * self.lb,
            );
            let d = a * a + b * b - c * c;
            if !(d >= 0.0) {
                return None;
            }
            *t = 2.0 * ((-b - d.sqrt()) / (c - a)).atan();
        }
        Some(out)
    }

    /// The platform position for arm angles: the lower point at a forearm's length from all three elbows. `None` where
    /// the legs cannot be assembled.
    pub fn forward(&self, t: V3) -> Option<V3> {
        self.forward_moved(t, [0.0; 3])
    }

    /// `forward` with the elbows moved by y along their tangents.
    pub fn forward_moved(&self, t: V3, y: V3) -> Option<V3> {
        let c = self.elbows_moved(t, y);
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
        let z2 = self.lb * self.lb - x * x - y * y;
        if !(z2 >= 0.0) {
            return None;
        }
        let base: V3 = std::array::from_fn(|k| c[0][k] + x * ex[k] + y * ey[k]);
        let (up, down): (V3, V3) = (
            std::array::from_fn(|k| base[k] + z2.sqrt() * ez[k]),
            std::array::from_fn(|k| base[k] - z2.sqrt() * ez[k]),
        );
        Some(if up[2] < down[2] { up } else { down })
    }

    /// J_x (rows: the forearm vectors u_i = p - c_i), the diagonal of J_q (u_i . dc_i/dt) and that of the elbows'
    /// moves (u_i . tangent_i), with the elbows moved by y.
    fn constraint(&self, t: V3, y: V3, p: V3) -> (M3, V3, V3) {
        let jx = self.elbows_moved(t, y).map(|c| sub(p, c));
        (jx, [0, 1, 2].map(|i| dot(jx[i], self.elbow(i, t[i])[1])), [0, 1, 2].map(|i| dot(jx[i], self.tangent(i))))
    }

    /// The Jacobian J[i][j] = dp_i / dtheta_j at arm angles t and platform position p; `None` at a direct singularity.
    pub fn jacobian(&self, t: V3, p: V3) -> Option<M3> {
        self.jacobians(t, [0.0; 3], p).map(|(j, _)| j)
    }

    /// The platform's Jacobians by the arm angles and by the elbows' moves y along their tangents, with the elbows
    /// moved by y: J_x^-1 J_q and J_x^-1 diag(u_i . tangent_i).
    pub fn jacobians(&self, t: V3, y: V3, p: V3) -> Option<(M3, M3)> {
        let (jx, jq, jy) = self.constraint(t, y, p);
        let inv = inverse(&jx)?;
        let scaled = |d: V3| -> M3 { std::array::from_fn(|i| std::array::from_fn(|j| inv[i][j] * d[j])) };
        Some((scaled(jq), scaled(jy)))
    }

    /// J' theta' = J_x^-1 w: the platform's acceleration at arm speeds `w_t` and no arm acceleration.
    pub fn velocity_term(&self, t: V3, p: V3, wt: V3, pv: V3) -> Option<V3> {
        self.velocity_term_moved(t, [0.0; 3], p, wt, [0.0; 3], pv)
    }

    /// `velocity_term` with the elbows moved by y and moving at yv along their tangents, which stay as they are:
    /// w_i = theta_i'^2 (u_i . c_i'') - |p' - c_i'|^2 with c_i' = dc_i/dtheta theta_i' + tangent_i y_i'.
    pub fn velocity_term_moved(&self, t: V3, y: V3, p: V3, wt: V3, yv: V3, pv: V3) -> Option<V3> {
        let (jx, ..) = self.constraint(t, y, p);
        let inv = inverse(&jx)?;
        let w: V3 = std::array::from_fn(|i| {
            let ([_, d1, d2], tg) = (self.elbow(i, t[i]), self.tangent(i));
            let rel: V3 = std::array::from_fn(|k| pv[k] - d1[k] * wt[i] - tg[k] * yv[i]);
            wt[i] * wt[i] * dot(jx[i], d2) - dot(rel, rel)
        });
        Some(mul(&inv, w))
    }

    /// The two rods of leg i's forearm pair, elbow end to platform end, at elbow angle t and platform position p, as
    /// drawn: `spacing` apart along the elbow's axis, their ends at the base radius (the legs' elbow is at e, `r` the
    /// platform radius added back) and at the platform's rim; the elbow moved by y along its axis (the arm bent out of
    /// its plane).
    pub fn rods(&self, i: usize, t: f64, y: f64, p: V3, r: f64, spacing: f64) -> [(V3, V3); 2] {
        let (c, sn) = self.azimuth[i];
        let (at, tg) = (self.elbow(i, t)[0], self.tangent(i));
        let elbow: V3 = std::array::from_fn(|k| at[k] + y * tg[k]);
        let half = [-0.5 * spacing * sn, 0.5 * spacing * c, 0.0];
        let (arm_end, platform_end) =
            ([elbow[0] + r * c, elbow[1] + r * sn, elbow[2]], [p[0] + r * c, p[1] + r * sn, p[2]]);
        [-1.0, 1.0].map(|side| {
            (
                std::array::from_fn(|k| arm_end[k] + side * half[k]),
                std::array::from_fn(|k| platform_end[k] + side * half[k]),
            )
        })
    }
}

/// The arms' modes with the motors held (by their position loops): per arm the gearbox's spring k_g from the motor to
/// the arm's root (inertia `root`), the bending spring k_b from the root to the elbow, and the rest of the machine as
/// the mass matrix `distal` in the elbow angles and, where it is 6 x 6, the elbows' moves out of the arms' planes, each
/// on its lateral spring k_l. `None` for a rigid spring (infinite stiffness: the two ends one body). The coordinates
/// are (roots, elbow angles, elbow moves), the flexibility per arm K^-1 = [[1/k_g, 1/k_g], [1/k_g, 1/k_g + 1/k_b]]
/// (root, elbow) and 1/k_l; with M = L L^T (Cholesky) the eigenvalues mu = 1/w^2 of the symmetric L^T K^-1 L, by
/// Jacobi rotations, the shape v = K^-1 L z / mu. A rigid connection has mu = 0 and is no mode. Gravity's stiffness
/// (some N m/rad against 1e4) and the stops are left out. Returns w^2 and the shape per mode, lowest first.
pub fn arm_modes(
    root: f64,
    distal: &[Vec<f64>],
    k_gear: Option<f64>,
    k_bend: Option<f64>,
    k_lateral: Option<f64>,
) -> Vec<(f64, Vec<f64>)> {
    let flex = |k: Option<f64>| k.map_or(0.0, |k| 1.0 / k);
    let (fg, fb, fl) = (flex(k_gear), flex(k_bend), flex(k_lateral));
    let n = 3 + distal.len();
    let (mut m, mut f) = (vec![vec![0.0; n]; n], vec![vec![0.0; n]; n]);
    for i in 0..3 {
        m[i][i] = root;
        (f[i][i], f[i][3 + i], f[3 + i][i], f[3 + i][3 + i]) = (fg, fg, fg, fg + fb);
        if n == 9 {
            f[6 + i][6 + i] = fl;
        }
    }
    for (r, row) in distal.iter().enumerate() {
        m[3 + r][3..].copy_from_slice(row);
    }
    let mut l = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..=i {
            let s = m[i][j] - (0..j).map(|k| l[i][k] * l[j][k]).sum::<f64>();
            l[i][j] = if i == j { s.max(0.0).sqrt() } else { s / l[j][j] };
        }
    }
    let times = |a: &[Vec<f64>], b: &[Vec<f64>]| -> Vec<Vec<f64>> {
        (0..n).map(|r| (0..n).map(|c| (0..n).map(|k| a[r][k] * b[k][c]).sum()).collect()).collect()
    };
    let lt: Vec<Vec<f64>> = (0..n).map(|r| (0..n).map(|c| l[c][r]).collect()).collect();
    let fl_ = times(&f, &l);
    let (mu, z) = symmetric_eigen(times(&lt, &fl_));
    let largest = mu.iter().fold(0f64, |a, b| a.max(*b));
    let mut out: Vec<(f64, Vec<f64>)> = (0..n)
        .filter(|&k| mu[k] > 1e-9 * largest)
        .map(|k| (1.0 / mu[k], (0..n).map(|r| (0..n).map(|c| fl_[r][c] * z[c][k]).sum::<f64>() / mu[k]).collect()))
        .collect();
    out.sort_by(|a, b| a.0.total_cmp(&b.0));
    out
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

/// A force on the platform against its motion along the line from a fixed point to a point on the platform: the
/// sliding friction of a telescopic shaft hung from `pivot` and joined to the platform at `offset` from its centre (the
/// fourth axis of a four-axis delta built on this crate). The D3 itself has none.
#[derive(Debug, Clone)]
pub struct PlatformDrag {
    pub pivot: [f64; 3],
    pub offset: [f64; 3],
    /// Along the line, N, against the rate at which the line lengthens.
    pub friction: ShaftFriction,
}

impl PlatformDrag {
    /// The unit vector from the pivot to the platform's joint, with the platform's centre at p.
    pub fn direction(&self, p: [f64; 3]) -> [f64; 3] {
        let d: V3 = std::array::from_fn(|k| p[k] + self.offset[k] - self.pivot[k]);
        let l = norm(d);
        d.map(|v| v / l)
    }

    /// The force on the platform at p moving at v: -f(L') u, with u the line's direction and L' = u . v its rate.
    pub fn force(&self, p: [f64; 3], v: [f64; 3]) -> [f64; 3] {
        let u = self.direction(p);
        let f = self.friction.torque(dot(u, v));
        u.map(|c| -f * c)
    }
}

/// The upper arms' bending (effects.link_elasticity): in the plane an arm turns in, a turn of the elbow against the
/// root, N m/rad and N m s/rad; out of it, a move of the elbow along its axis, N/m and N s/m.
#[derive(Debug, Clone, Copy)]
struct Bending {
    k: f64,
    c: f64,
    k_lateral: f64,
    c_lateral: f64,
}

pub struct Delta {
    legs: Legs,
    /// `PlatformDrag`, set by a machine built on this one; `None` on the D3.
    drag: Option<PlatformDrag>,
    /// The geometry as configured, for the scene: base and platform radius, the rods' spacing.
    base_radius: f64,
    platform_radius: f64,
    rod_spacing: f64,
    elastic: bool,
    g: f64,
    /// I_b without the rotor and m_LG, each split into the arm's tube (root, about the joint) and what sits at the
    /// elbow (tip); the mass at each forearm's middle (effects.forearm_inertia, else 0); platform mass without and with
    /// payload (forearm share included).
    root_inertia: f64,
    root_moment: f64,
    tip_inertia: f64,
    tip_moment: f64,
    m_rod_middle: f64,
    /// What moves with an arm's bend out of its plane, at the elbow: the elbow block and the forearms' share, with
    /// effects.arm_tube_inertia the tube's own share and the elbow block's twist too.
    m_lateral: f64,
    /// The spin inertia of a forearm pair, both rods about their own axes (effects.forearm_inertia, else 0), kg m^2.
    rod_spin: f64,
    /// effects.link_elasticity: the upper arms' bending.
    bending: Option<Bending>,
    m_platform: f64,
    m_payload: f64,
    payload_attached: bool,
    brake_engaged: bool,
    joints: [RotaryJoint; 3],
    thermal: Option<ThermalFeedback>,
    /// The working range: radius, top, bottom sphere's centre height and radius; the condition number refused above.
    radius: f64,
    top: f64,
    sphere: (f64, f64),
    max_condition: f64,
    axes: [AxisInfo; 3],
    motors: [MotorInfo; 3],
    /// `Machine::setpoint_slack`, worked out once (`rest_sag`).
    rest_slack: f64,
}

// Elastic state: [theta 1..3, omega 1..3, rotor angle 1..3, rotor speed 1..3, brake 1..3].
const N_ELASTIC: usize = 15;
// Rigid state: [theta 1..3, omega 1..3, brake 1..3].
const N_RIGID: usize = 9;

/// The losses of one moment: per joint its friction, its motor's friction with the brake, its gearbox; the stops and
/// what the ripple gives off; the arms' bending damping (effects.link_elasticity); and per joint the gearbox loss as a
/// torque at the rotor.
struct Losses {
    powers: [f64; 12],
    gear_at_rotor: V3,
}

impl Delta {
    /// The platform at `p` and the payload under it, as `scene` draws them.
    fn on_platform(&self, p: [f64; 3]) -> Vec<Part> {
        let size = 2.4 * self.platform_radius.max(0.03);
        let mut parts = vec![Part::new("platform", Shape::Box { size: [size, size, 0.03] }, p, [0.55, 0.55, 0.6])];
        if self.payload_attached && self.m_payload > 0.0 {
            parts.push(Part::new(
                "payload",
                Shape::Box { size: [0.1, 0.1, 0.08] },
                [p[0], p[1], p[2] - 0.06],
                [0.2, 0.7, 0.3],
            ));
        }
        parts
    }

    /// Upper arm k from its shoulder to its end, as `scene` draws it: the legs' elbow is at e, the arm's end at the base
    /// radius (the platform's radius added back), moved by y out of the arm's plane.
    fn arm(&self, k: usize, t: V3, y: V3) -> (V3, V3) {
        let (c, sn) = self.legs.azimuth[k];
        let elbow = self.legs.elbows_moved(t, y)[k];
        (
            [self.base_radius * c, self.base_radius * sn, 0.0],
            [elbow[0] + self.platform_radius * c, elbow[1] + self.platform_radius * sn, elbow[2]],
        )
    }

    pub fn new(cfg: &Config) -> Result<Self, ConfigError> {
        let base_radius = cfg.positive("geometry.base_radius_m")?;
        let platform_radius = cfg.non_negative("geometry.platform_radius_m")?;
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
        let bending = Bending {
            k: cfg.positive("upper_arm.bending_stiffness_nm_rad")?,
            c: cfg.non_negative("upper_arm.bending_damping_nm_s_rad")?,
            k_lateral: cfg.positive("upper_arm.lateral_stiffness_n_m")?,
            c_lateral: cfg.non_negative("upper_arm.lateral_damping_n_s_m")?,
        };
        let spin = cfg.non_negative("mass.forearm_rod_spin_inertia_kgm2")?;
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
        // effects.arm_tube_inertia (with the bending arms): the tube bends in its first cantilever mode
        // (`beam`), so a quarter of its mass moves with the elbow's bend, m L^2 / 4 of its inertia about the shoulder
        // and 0.3915 m L of its first moment (what its weight drives the bend with) go from the root to the elbow; out
        // of its plane a quarter of its mass moves with the elbow, and the elbow block turns with the tube's tip slope,
        // 1.3765 / L per metre: its twist inertia (the block's, and the forearms' share at the rods' ends, half the
        // spacing off its middle) counts J (1.3765 / L)^2 there.
        let rod_spacing = cfg.non_negative("geometry.forearm_spacing_m")?;
        let (mut root_inertia, mut root_moment, mut tip_inertia, mut tip_moment, mut m_lateral) =
            (i_arm, m_arm * cg, la * la * elbow, la * elbow, elbow);
        if cfg.bool("effects.arm_tube_inertia")? && cfg.bool("effects.link_elasticity")? {
            use sim_components::beam::{CANTILEVER_PARTICIPATION, CANTILEVER_TIP_SLOPE};
            let (inertia, moment) = (0.25 * m_arm * la * la, CANTILEVER_PARTICIPATION * m_arm * la);
            if inertia >= root_inertia || moment > root_moment {
                return Err(ConfigError::Invalid("mass.upper_arm_inertia_kgm2".into()));
            }
            (root_inertia, root_moment, tip_inertia, tip_moment) =
                (root_inertia - inertia, root_moment - moment, tip_inertia + inertia, tip_moment + moment);
            let twist =
                cfg.non_negative("mass.elbow_twist_inertia_kgm2")? + share * m_fb * rod_spacing * rod_spacing / 4.0;
            m_lateral += 0.25 * m_arm + twist * (CANTILEVER_TIP_SLOPE / la).powi(2);
        }
        let mut delta = Self {
            legs: Legs::new(base_radius - platform_radius, la, lb),
            drag: None,
            base_radius,
            platform_radius,
            rod_spacing,
            elastic: cfg.bool("effects.joint_elasticity")?,
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            root_inertia,
            root_moment,
            tip_inertia,
            tip_moment,
            m_rod_middle: middle * m_fb,
            m_lateral,
            rod_spin: if rods { 2.0 * spin } else { 0.0 },
            bending: cfg.bool("effects.link_elasticity")?.then_some(bending),
            m_platform: cfg.positive("mass.platform_kg")? + 3.0 * (1.0 - share - middle) * m_fb,
            m_payload: if cfg.bool("effects.payload")? { cfg.non_negative("mass.payload_kg")? } else { 0.0 },
            payload_attached: true,
            brake_engaged: false,
            joints: [joint.clone(), joint.clone(), joint],
            thermal: ThermalFeedback::from_config(cfg)?,
            radius,
            top,
            sphere,
            max_condition: cfg.positive("workspace.max_condition_number")?,
            axes: [
                axis("x", "axis-x", -radius, radius)?,
                axis("y", "axis-y", -radius, radius)?,
                axis("z", "axis-z", top - centre, top)?,
            ],
            motors: [
                MotorInfo { id: "1", key: "motor-arm-1" },
                MotorInfo { id: "2", key: "motor-arm-2" },
                MotorInfo { id: "3", key: "motor-arm-3" },
            ],
            rest_slack: 0.0,
        };
        delta.rest_slack = delta.rest_sag();
        Ok(delta)
    }

    /// How far the motors' pose can be from the tool's at rest anywhere in the working range, payload on or off: each
    /// gearbox twisted by the torque it holds against gravity over its stiffness, plus half its backlash either way (an
    /// arm that holds little can rest anywhere in it, so all eight combinations are tried), through the forward
    /// kinematics. Over a grid of the range: 5 radii, 24 directions, 9 heights and the bottom; plus a tenth for what the
    /// grid steps over and the joints' friction. At least the default 0.1 mm; that alone in the rigid model, whose
    /// gearboxes do not twist.
    fn rest_sag(&self) -> f64 {
        let mut most = 0f64;
        if !self.elastic && self.bending.is_none() {
            return 1e-4;
        }
        let (bottom, top) = (self.axes[2].min, self.top);
        for ring in 0..=4 {
            let rho = self.radius * ring as f64 / 4.0;
            for turn in 0..24 {
                let (sin, cos) = (turn as f64 * std::f64::consts::TAU / 24.0).sin_cos();
                let (x, y) = (rho * cos, rho * sin);
                let (centre, r) = self.sphere;
                let lowest = if centre.is_finite() { centre - (r * r - rho * rho).max(0.0).sqrt() } else { bottom };
                let heights = (0..=8).map(|k| lowest + (top - lowest) * k as f64 / 8.0);
                for p in heights.map(|z| [x, y, z]).filter(|p| self.in_envelope(p, 1e-9)) {
                    if self.legs.inverse(p).is_none() {
                        continue;
                    }
                    for m in [self.m_platform, self.m_platform + self.m_payload] {
                        // The joints' angles at rest, the arms' bend taken off the elbows'.
                        let (t, gravity, ..) = self.rest_at(p, m);
                        let twist: V3 = std::array::from_fn(|k| {
                            let held = -gravity[k] - self.joints[k].output_torque(t[k], 0.0);
                            if self.elastic { held / self.joints[k].gearbox.stiffness } else { 0.0 }
                        });
                        for signs in 0..8 {
                            // No backlash in the rigid joint model.
                            let backlash =
                                |k: usize| if self.elastic { self.joints[k].gearbox.half_backlash } else { 0.0 };
                            let half = |k: usize| if signs >> k & 1 == 1 { 1.0 } else { -1.0 } * backlash(k);
                            if let Some(q) = self.legs.forward(std::array::from_fn(|k| t[k] + twist[k] + half(k))) {
                                most = most.max(norm(sub(q, p)));
                            }
                        }
                    }
                }
            }
        }
        (1.1 * most).max(1e-4)
    }

    pub fn legs(&self) -> &Legs {
        &self.legs
    }

    /// Puts a force on the platform against its motion (`PlatformDrag`), or takes it off. It enters the dynamics as a
    /// generalised force J^T F at the elbows (in `load_at`'s velocity term); the feedforward, the rigid loss-free model,
    /// leaves it out as it leaves out the joints' friction.
    pub fn set_platform_drag(&mut self, drag: Option<PlatformDrag>) {
        self.drag = drag;
    }

    /// The drag's generalised force J^T F at elbow angles t and speeds w; zero without a drag.
    fn drag_torque(&self, t: V3, w: V3) -> V3 {
        let Some(drag) = &self.drag else { return [0.0; 3] };
        let (p, j) = self.pose(t);
        mul(&transpose(&j), drag.force(p, mul(&j, w)))
    }

    fn ratio(&self) -> f64 {
        self.joints[0].ratio()
    }

    /// The point mass at the platform.
    pub fn m_nt(&self) -> f64 {
        self.m_platform + if self.payload_attached { self.m_payload } else { 0.0 }
    }

    /// The arm's inertia about its joint, elbow share of the forearms included, rotor not.
    pub fn arm_inertia(&self) -> f64 {
        self.root_inertia + self.tip_inertia
    }

    /// m_LG: the arm's gravity torque is g m_LG cos(theta).
    pub fn arm_moment(&self) -> f64 {
        self.root_moment + self.tip_moment
    }

    /// Arm angles and speeds of a state: at the joints, the gearboxes' output side.
    fn arms(&self, s: &[f64]) -> (V3, V3) {
        ([s[0], s[1], s[2]], [s[3], s[4], s[5]])
    }

    /// Where the arms' bend sits in the state, after the brakes.
    fn bend_at(&self) -> usize {
        if self.elastic { N_ELASTIC } else { N_RIGID }
    }

    /// The arms' bend (the elbow's turn against the arm's root) and its rate; zero with rigid links.
    fn bend(&self, s: &[f64]) -> (V3, V3) {
        if self.bending.is_none() {
            return ([0.0; 3], [0.0; 3]);
        }
        let at = self.bend_at();
        ([s[at], s[at + 1], s[at + 2]], [s[at + 3], s[at + 4], s[at + 5]])
    }

    /// The elbows' moves out of the arms' planes, along their axes, and their rates; zero with rigid links.
    fn lateral(&self, s: &[f64]) -> (V3, V3) {
        if self.bending.is_none() {
            return ([0.0; 3], [0.0; 3]);
        }
        let at = self.bend_at() + 6;
        ([s[at], s[at + 1], s[at + 2]], [s[at + 3], s[at + 4], s[at + 5]])
    }

    /// The elbow angles and speeds, which the kinematics take: the arms' plus their bend.
    fn elbows(&self, s: &[f64]) -> (V3, V3) {
        let ((t, w), (b, r)) = (self.arms(s), self.bend(s));
        (std::array::from_fn(|k| t[k] + b[k]), std::array::from_fn(|k| w[k] + r[k]))
    }

    /// Rotor angles and speeds of a state.
    fn rotors(&self, s: &[f64]) -> (V3, V3) {
        if self.elastic {
            ([s[6], s[7], s[8]], [s[9], s[10], s[11]])
        } else {
            let i = self.ratio();
            ([s[0] * i, s[1] * i, s[2] * i], [s[3] * i, s[4] * i, s[5] * i])
        }
    }

    fn brakes(&self, s: &[f64]) -> V3 {
        let at = if self.elastic { 12 } else { 6 };
        [s[at], s[at + 1], s[at + 2]]
    }

    /// Where the platform is and the Jacobian there, for arm angles; NaN where the legs cannot be assembled.
    fn pose(&self, t: V3) -> (V3, M3) {
        let (p, j, _) = self.pose_moved(t, [0.0; 3]);
        (p, j)
    }

    /// `pose` with the elbows moved by y out of the arms' planes, and the Jacobian by those moves.
    fn pose_moved(&self, t: V3, y: V3) -> (V3, M3, M3) {
        self.legs
            .forward_moved(t, y)
            .and_then(|p| {
                let (j, jy) = self.legs.jacobians(t, y, p)?;
                Some((p, j, jy))
            })
            .unwrap_or(([f64::NAN; 3], [[f64::NAN; 3]; 3], [[f64::NAN; 3]; 3]))
    }

    /// The platform's position and velocity at state s.
    fn platform(&self, s: &[f64]) -> (V3, V3) {
        let ((t, w), (y, yv)) = (self.elbows(s), self.lateral(s));
        let (p, j, jy) = self.pose_moved(t, y);
        let (a, b) = (mul(&j, w), mul(&jy, yv));
        (p, std::array::from_fn(|k| a[k] + b[k]))
    }

    /// The load's mass matrix M, the velocity term m_nt J^T J' theta' (and the forearms' middles'), and the generalised
    /// gravity Q_g at arm angles t and speeds w, the links rigid.
    fn load(&self, t: V3, w: V3) -> (M3, V3, V3) {
        self.load_at(t, w, self.m_nt(), true)
    }

    /// `load` at elbow angles t and speeds w with the platform's mass m: of everything beyond the gearboxes (`root`), or
    /// only of what lies beyond the arms' tubes (what the bending springs carry).
    pub fn load_at(&self, t: V3, w: V3, m: f64, root: bool) -> (M3, V3, V3) {
        self.load_n::<3>(t, [0.0; 3], w, [0.0; 3], m, root)
    }

    /// `load_at` in N coordinates: the elbow angles t (speeds w) and, with N = 6, the elbows' moves y out of the arms'
    /// planes along their axes (rates yv). Each point mass (platform, the rods' middles, on the moves the elbows) counts
    /// with its Jacobian P (3 x N): m P^T P in the mass matrix, m P^T a0 in the velocity term (a0 its acceleration at
    /// no acceleration of the coordinates), -m g P_z in the gravity; the elbows' on the angles are `tip_inertia` and
    /// `tip_moment`. With the rods also their spin about their own axes (`add_spin`).
    fn load_n<const N: usize>(&self, t: V3, y: V3, w: V3, yv: V3, m: f64, root: bool) -> Load<N> {
        let (p, j, jy) = self.pose_moved(t, y);
        let jp: [[f64; N]; 3] =
            std::array::from_fn(|r| std::array::from_fn(|c| if c < 3 { j[r][c] } else { jy[r][c - 3] }));
        let qv: [f64; N] = std::array::from_fn(|c| if c < 3 { w[c] } else { yv[c - 3] });
        let pv: V3 = std::array::from_fn(|r| (0..N).map(|c| jp[r][c] * qv[c]).sum());
        let a0 = self.legs.velocity_term_moved(t, y, p, w, yv, pv).unwrap_or([f64::NAN; 3]);
        let mut load = ([[0.0; N]; N], [0.0; N], [0.0; N]);
        add_point(&mut load, m, &jp, a0, self.g);
        let (inertia, moment) =
            if root { (self.arm_inertia(), self.arm_moment()) } else { (self.tip_inertia, self.tip_moment) };
        if let Some(drag) = &self.drag {
            let f = drag.force(p, pv);
            for r in 0..N {
                load.1[r] -= (0..3).map(|k| jp[k][r] * f[k]).sum::<f64>();
            }
        }
        let elbows = self.legs.elbows_moved(t, y);
        for i in 0..3 {
            load.0[i][i] += inertia;
            load.2[i] += self.g * moment * t[i].cos();
            // The elbow's Jacobian: dc/dtheta on its angle, its axis on its move; c'' = d2 w^2 at no acceleration.
            let ([_, d1, d2], axis) = (self.legs.elbow(i, t[i]), self.legs.tangent(i));
            let je: [[f64; N]; 3] = std::array::from_fn(|r| {
                std::array::from_fn(|c| {
                    if c == i {
                        d1[r]
                    } else if c == 3 + i {
                        axis[r]
                    } else {
                        0.0
                    }
                })
            });
            let ae = d2.map(|v| v * w[i] * w[i]);
            if N == 6 {
                // Along its axis the elbow mass has no acceleration at none of the coordinates', and no weight.
                load.0[3 + i][3 + i] += self.m_lateral;
            }
            if self.m_rod_middle > 0.0 {
                let jm = std::array::from_fn(|r| std::array::from_fn(|c| 0.5 * (jp[r][c] + je[r][c])));
                add_point(&mut load, self.m_rod_middle, &jm, std::array::from_fn(|k| 0.5 * (a0[k] + ae[k])), self.g);
            }
            if self.rod_spin > 0.0 {
                let ju = std::array::from_fn(|r| std::array::from_fn(|c| jp[r][c] - je[r][c]));
                let accel = std::array::from_fn(|k| a0[k] - ae[k]);
                self.add_spin(&mut load, axis, sub(p, elbows[i]), &ju, &qv, accel);
            }
        }
        load
    }

    /// The spin of a forearm pair's rods about their own axis into `load`, both rods alike: the parallelogram keeps the
    /// pair's plane through the elbow's axis a (fixed) and the forearm u = p - c, so a rod turns about u as that plane
    /// does. With sigma = a . u, n = a . (u' x u) and L = |u|, the spin is s = -sigma n / (L (L^2 - sigma^2)), linear in
    /// the speeds: s = g . q', g = -sigma / (L (L^2 - sigma^2)) J_u^T (u x a), J_u the forearm's Jacobian. The rod's
    /// angular momentum about its axis I s e (e = u / L) gives, by Kane's equations with the rod's angular velocity
    /// e x e' + s e, I g g^T in the mass matrix and I (s0' g + s J_u^T (u' x u) / L^3) in the velocity term, s0' the
    /// spin's rate at no acceleration of the coordinates (u'' = `accel`); its kinetic energy is I s^2 / 2.
    fn add_spin<const N: usize>(&self, load: &mut Load<N>, a: V3, u: V3, ju: &[[f64; N]; 3], qv: &[f64; N], accel: V3) {
        let l = self.legs.lb;
        let du: V3 = std::array::from_fn(|r| (0..N).map(|c| ju[r][c] * qv[c]).sum());
        let (sigma, n) = (dot(a, u), dot(a, cross(du, u)));
        let d = l * l - sigma * sigma;
        let s = -sigma * n / (l * d);
        let axis = cross(u, a);
        let g: [f64; N] = std::array::from_fn(|c| -sigma / (l * d) * (0..3).map(|k| ju[k][c] * axis[k]).sum::<f64>());
        let (dsigma, dn) = (dot(a, du), dot(a, cross(accel, u)));
        let ds = -(dsigma * n + sigma * dn) / (l * d) - 2.0 * sigma * sigma * dsigma * n / (l * d * d);
        let turn = cross(du, u).map(|v| v / l.powi(3));
        for r in 0..N {
            for c in 0..N {
                load.0[r][c] += self.rod_spin * g[r] * g[c];
            }
            load.1[r] += self.rod_spin * (ds * g[r] + s * (0..3).map(|k| ju[k][r] * turn[k]).sum::<f64>());
        }
    }

    /// The spin of leg i's rods about their own axis (see `add_spin`), rad/s, at elbow angles t and moves y, speeds w
    /// and yv.
    pub fn rod_spin(&self, i: usize, t: V3, y: V3, w: V3, yv: V3) -> f64 {
        let (p, j, jy) = self.pose_moved(t, y);
        let (a, b) = (mul(&j, w), mul(&jy, yv));
        let ([_, d1, _], axis) = (self.legs.elbow(i, t[i]), self.legs.tangent(i));
        let du: V3 = std::array::from_fn(|k| a[k] + b[k] - d1[k] * w[i] - axis[k] * yv[i]);
        let u = sub(p, self.legs.elbows_moved(t, y)[i]);
        let (l, sigma) = (self.legs.lb, dot(axis, u));
        -sigma * dot(axis, cross(du, u)) / (l * (l * l - sigma * sigma))
    }

    /// At rest with the platform at p and its mass m: the joints' angles, the torque gravity puts on each joint (all of
    /// it: with bending arms the tube's own and, through the spring, the rest), the arms' bend, Q_d / k_b, and the
    /// elbows' moves out of the arms' planes, Q_y / k_l (from none, three rounds: Q_y hardly changes with them).
    fn rest_at(&self, p: V3, m: f64) -> (V3, V3, V3, V3) {
        let Some(b) = self.bending else {
            let elbows = self.legs.inverse(p).unwrap_or([f64::NAN; 3]);
            return (elbows, self.load_at(elbows, [0.0; 3], m, true).2, [0.0; 3], [0.0; 3]);
        };
        let tip_at = |y: V3| {
            let elbows = self.legs.inverse_moved(p, y).unwrap_or([f64::NAN; 3]);
            (elbows, self.load_n::<6>(elbows, y, [0.0; 3], [0.0; 3], m, false).2)
        };
        let mut y = [0.0; 3];
        for _ in 0..3 {
            let (_, tip) = tip_at(y);
            y = std::array::from_fn(|i| tip[3 + i] / b.k_lateral);
        }
        let (elbows, tip) = tip_at(y);
        let bend: V3 = std::array::from_fn(|i| tip[i] / b.k);
        let t: V3 = std::array::from_fn(|i| elbows[i] - bend[i]);
        (t, std::array::from_fn(|i| self.g * self.root_moment * t[i].cos() + tip[i]), bend, y)
    }

    /// What the joints carry at state s, as `load` has it: mass matrix, velocity term, gravity. With bending arms only
    /// the tubes are on the joints, and the springs' torque stands in for the rest: its weight Q_d in the gravity, the
    /// remainder in the velocity term (so that the torque breakdown shows the weight as gravity).
    fn joint_load(&self, s: &[f64]) -> (M3, V3, V3) {
        let (t, w) = self.arms(s);
        let Some(b) = self.bending else { return self.load(t, w) };
        let ((elbows, _), (bend, rate), (y, _)) = (self.elbows(s), self.bend(s), self.lateral(s));
        let (_, _, tip) = self.load_n::<6>(elbows, y, [0.0; 3], [0.0; 3], self.m_nt(), false);
        let mass = std::array::from_fn(|r| std::array::from_fn(|col| if r == col { self.root_inertia } else { 0.0 }));
        let gravity = std::array::from_fn(|i| self.g * self.root_moment * t[i].cos() + tip[i]);
        (mass, std::array::from_fn(|i| tip[i] - b.k * bend[i] - b.c * rate[i]), gravity)
    }

    /// With bending arms, the rates of the bend states into `out`, from the joints' acceleration `acc`: the elbows'
    /// accelerations, of their angles and their moves, from M_d q'' = Q_d - v_d - (k_b delta + c_b delta', k_l y + c_l y')
    /// (`load_n` beyond the tubes).
    fn bend_rates(&self, s: &[f64], acc: V3, out: &mut [f64]) {
        let Some(b) = self.bending else { return };
        let ((elbows, speeds), (bend, rate), (y, yv)) = (self.elbows(s), self.bend(s), self.lateral(s));
        let (mass, velocity, gravity) = self.load_n::<6>(elbows, y, speeds, yv, self.m_nt(), false);
        let rhs: Vec<f64> = (0..6)
            .map(|i| {
                let spring = if i < 3 {
                    b.k * bend[i] + b.c * rate[i]
                } else {
                    b.k_lateral * y[i - 3] + b.c_lateral * yv[i - 3]
                };
                gravity[i] - velocity[i] - spring
            })
            .collect();
        let q = sim_components::chain::solve(mass.as_flattened(), &rhs).unwrap_or_else(|| vec![f64::NAN; 6]);
        let at = self.bend_at();
        out[at..at + 3].copy_from_slice(&rate);
        for i in 0..3 {
            out[at + 3 + i] = q[i] - acc[i];
        }
        out[at + 6..at + 9].copy_from_slice(&yv);
        out[at + 9..at + 12].copy_from_slice(&q[3..]);
    }

    /// The modes at a tool pose with the motors held (`arm_modes`), lowest first: frequency in Hz, damping ratio and
    /// shape (the joints' turns, the elbows' turns, with bending arms the elbows' moves). The damping of a shape v is
    /// v'Cv / (2 w v'Mv) with C the gearboxes', the bending's and the lateral dampers (the cross terms of a damping that
    /// is not proportional left out). None with rigid gearboxes and rigid arms, or out of reach.
    fn modes_at(&self, tool: &[f64]) -> Vec<(f64, f64, Vec<f64>)> {
        let k_gear = self.elastic.then(|| self.joints[0].gearbox.stiffness);
        let elbows = if tool.len() == 3 { self.legs.inverse([tool[0], tool[1], tool[2]]) } else { None };
        let (Some(elbows), true) = (elbows, k_gear.is_some() || self.bending.is_some()) else { return Vec::new() };
        let distal: Vec<Vec<f64>> = match self.bending {
            Some(_) => {
                self.load_n::<6>(elbows, [0.0; 3], [0.0; 3], [0.0; 3], self.m_nt(), false).0.map(Vec::from).into()
            }
            None => self.load_at(elbows, [0.0; 3], self.m_nt(), false).0.map(Vec::from).into(),
        };
        let c_gear = if self.elastic { self.joints[0].gearbox.damping } else { 0.0 };
        let b = self.bending;
        let modes = arm_modes(self.root_inertia, &distal, k_gear, b.map(|b| b.k), b.map(|b| b.k_lateral));
        modes
            .into_iter()
            .map(|(w2, v)| {
                let n = v.len() - 3;
                let distal_kinetic: f64 =
                    (0..n).map(|r| v[3 + r] * (0..n).map(|c| distal[r][c] * v[3 + c]).sum::<f64>()).sum();
                let kinetic = self.root_inertia * (0..3).map(|i| v[i] * v[i]).sum::<f64>() + distal_kinetic;
                let damped: f64 = (0..3)
                    .map(|i| {
                        c_gear * v[i] * v[i]
                            + b.map_or(0.0, |b| b.c * (v[3 + i] - v[i]).powi(2) + b.c_lateral * v[6 + i] * v[6 + i])
                    })
                    .sum();
                let w = w2.sqrt();
                (w / std::f64::consts::TAU, damped / (2.0 * w * kinetic), v)
            })
            .collect()
    }

    /// Rigid model: the arms' acceleration, and per joint the torque into its gearbox (after the rotor's own
    /// acceleration) and out of it, at the arm.
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> (V3, V3, V3) {
        let (t, w) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let (mass, velocity, gravity) = self.joint_load(s);
        let drive: V3 = std::array::from_fn(|k| {
            self.joints[k].rigid_drive(torque[k], rotor[k], rotor_speed[k], brakes[k], self.brake_engaged).0
        });
        let rest: V3 = std::array::from_fn(|k| gravity[k] - velocity[k] + self.joints[k].output_torque(t[k], w[k]));
        let solve = |loss: V3| -> V3 {
            let m: M3 = std::array::from_fn(|r| {
                std::array::from_fn(|c| {
                    mass[r][c] + if r == c { loss[r] * self.joints[r].reflected_inertia() } else { 0.0 }
                })
            });
            let rhs: V3 = std::array::from_fn(|k| loss[k] * drive[k] + rest[k]);
            inverse(&m).map_or([f64::NAN; 3], |inv| mul(&inv, rhs))
        };
        let first = solve([1.0; 3]);
        let loss: V3 = std::array::from_fn(|k| {
            self.joints[k].gearbox.loss_factor(drive[k] - self.joints[k].reflected_inertia() * first[k], w[k])
        });
        let acc = solve(loss);
        let into: V3 = std::array::from_fn(|k| drive[k] - self.joints[k].reflected_inertia() * acc[k]);
        (acc, into, std::array::from_fn(|k| loss[k] * into[k]))
    }

    /// The arms' acceleration in either model.
    fn arm_acceleration(&self, s: &[f64], torque: &[f64]) -> V3 {
        if self.elastic {
            let mut d = vec![0.0; self.state_len()];
            self.derivatives(s, torque, &mut d);
            [d[3], d[4], d[5]]
        } else {
            self.rigid_solve(s, torque).0
        }
    }

    /// The torque each gearbox passes to its arm.
    fn gear_output(&self, s: &[f64], torque: &[f64]) -> V3 {
        if self.elastic {
            std::array::from_fn(|k| self.joints[k].gearbox.torque(s[6 + k], s[9 + k], s[k], s[3 + k]).output)
        } else {
            self.rigid_solve(s, torque).2
        }
    }

    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        let (t, w) = self.arms(s);
        let ((elbows, speeds), (y, yv)) = (self.elbows(s), self.lateral(s));
        // Beyond the tubes at the elbows' speeds and moves, the tubes at the joints' (all of it at once with rigid
        // links).
        let mut kinetic = if self.bending.is_some() {
            let (mass, ..) = self.load_n::<6>(elbows, y, speeds, yv, self.m_nt(), false);
            let q: [f64; 6] = std::array::from_fn(|i| if i < 3 { speeds[i] } else { yv[i - 3] });
            let moving: f64 = (0..6).map(|r| q[r] * (0..6).map(|c| mass[r][c] * q[c]).sum::<f64>()).sum();
            0.5 * moving + 0.5 * self.root_inertia * dot(w, w)
        } else {
            0.5 * dot(speeds, mul(&self.load_at(elbows, speeds, self.m_nt(), true).0, speeds))
        };
        let (p, ..) = self.pose_moved(elbows, y);
        let middles: f64 = (0..3).map(|i| 0.5 * (self.legs.elbow(i, elbows[i])[0][2] + p[2])).sum();
        let sines = |a: V3| a[0].sin() + a[1].sin() + a[2].sin();
        let potential = self.g
            * (self.m_nt() * p[2] + self.m_rod_middle * middles
                - self.root_moment * sines(t)
                - self.tip_moment * sines(elbows));
        let mut elastic: f64 = (0..3).map(|k| self.joints[k].stop_energy(t[k])).sum();
        if let Some(b) = self.bending {
            let (bend, _) = self.bend(s);
            elastic += 0.5 * b.k * dot(bend, bend) + 0.5 * b.k_lateral * dot(y, y);
        }
        if self.elastic {
            for k in 0..3 {
                kinetic += 0.5 * self.joints[k].rotor_inertia * s[9 + k] * s[9 + k];
                elastic += self.joints[k].gearbox.energy(s[6 + k], s[k]);
            }
        } else {
            kinetic += 0.5 * (0..3).map(|k| self.joints[k].reflected_inertia() * w[k] * w[k]).sum::<f64>();
        }
        [kinetic, potential, elastic]
    }

    fn losses(&self, s: &[f64], torque: &[f64]) -> Losses {
        let (t, w) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let mut powers = [0.0; 12];
        let mut gear_at_rotor = [0.0; 3];
        let rigid = (!self.elastic).then(|| self.rigid_solve(s, torque));
        let mut stops = 0.0;
        let mut other = 0.0;
        for k in 0..3 {
            let joint = &self.joints[k];
            let [friction, stop, shaft] = joint.losses(t[k], w[k], rotor_speed[k], brakes[k], self.brake_engaged);
            let gear = match &rigid {
                Some((_, into, out)) => {
                    gear_at_rotor[k] = (into[k] - out[k]) / joint.ratio();
                    (into[k] - out[k]) * w[k]
                }
                None => {
                    let g = joint.gearbox.torque(s[6 + k], s[9 + k], s[k], s[3 + k]);
                    gear_at_rotor[k] = g.rotor - g.output / joint.ratio();
                    joint.gearbox.loss_power(s[6 + k], s[9 + k], s[k], s[3 + k])
                }
            };
            powers[3 * k..3 * k + 3].copy_from_slice(&[friction, shaft, gear]);
            stops += stop;
            other += (torque[k] - joint.ripple.apply(torque[k], rotor[k])) * rotor_speed[k];
        }
        powers[9] = stops;
        powers[10] = other;
        if let Some(b) = self.bending {
            let ((_, rate), (_, moving)) = (self.bend(s), self.lateral(s));
            powers[11] = b.c * dot(rate, rate) + b.c_lateral * dot(moving, moving);
        }
        Losses { powers, gear_at_rotor }
    }

    /// How many of `Losses::powers` are signals: the arms' bending damping only with effects.link_elasticity.
    fn loss_count(&self) -> usize {
        if self.bending.is_some() { 12 } else { 11 }
    }

    /// Whether a pose is within the working range (cylinder, top, spherical bottom), widened by `slack`.
    fn in_envelope(&self, p: &[f64], slack: f64) -> bool {
        let (centre, r) = self.sphere;
        let below = if centre.is_finite() {
            (p[0] * p[0] + p[1] * p[1] + (p[2] - centre).powi(2)).sqrt() <= r + slack
        } else {
            p[2] >= self.axes[2].min - slack
        };
        p[0].hypot(p[1]) <= self.radius + slack && p[2] <= self.top + slack && below
    }

    /// The torque breakdown, loss powers, energies, friction signals of the effect views.
    fn view_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (_, w) = self.arms(s);
        let (_, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let (mass, velocity, gravity) = self.joint_load(s);
        let acc = self.arm_acceleration(s, torque);
        let moving = mul(&mass, acc);
        let losses = self.losses(s, torque);
        // A drag on the platform is friction: with rigid links it is in the joints' velocity term (`load_at`), and is
        // moved over to the friction here. With bending arms it reaches the joints through the springs.
        let drag = if self.bending.is_none() { self.drag_torque(self.arms(s).0, w) } else { [0.0; 3] };
        for k in 0..3 {
            let joint = &self.joints[k];
            let i = joint.ratio();
            let held = joint.brake.torque(self.brake_engaged, rotor_speed[k], brakes[k]).0;
            let parts = [
                (moving[k] + velocity[k] + drag[k] + joint.reflected_inertia() * acc[k]) / i,
                -gravity[k] / i,
                joint.shaft.torque(rotor_speed[k])
                    + held
                    + losses.gear_at_rotor[k]
                    + (joint.friction.torque(w[k]) - drag[k]) / i,
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
            let arm = ["arm-1", "arm-2", "arm-3"][k];
            let gear = if self.joints[k].gearbox.is_coupling() {
                "quantity-coupling-loss-power"
            } else {
                "quantity-gearbox-loss-power"
            };
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}joint_{}", motor.id), arm, "quantity-friction-power", "W"));
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
        if self.bending.is_some() {
            out.push(SignalInfo::new(
                format!("{LOSS_PREFIX}arm_bending"),
                "group-upper-arms",
                "quantity-damping-power",
                "W",
            ));
        }
        out.extend(ENERGY_PARTS.map(|part| {
            SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")
        }));
    }
}

impl Machine for Delta {
    /// The platform (with the payload and the forearms' share), each arm's tube turning with its joint and what sits at
    /// its elbow turning with the elbow (the bend added), the elbow's move out of the arm's plane, the forearms' middles,
    /// and the rotors spinning about the joints' axes. The joints are where they are, at the base radius; the platform's
    /// centre is the tool. What does not move is left out (see `FrameMomentum`).
    // shortcut: the forearms' spin about their own axes is left out (effects.forearm_inertia; a few per cent of the
    // reaction torque about z); add it if the base's twist under the D3 has to be exact.
    fn frame_momentum(&self, s: &[f64]) -> Option<sim_core::FrameMomentum> {
        use sim_components::base::{add_point, add_spin, add_turning};
        let mut f = sim_core::FrameMomentum { gravity: [0.0, 0.0, -self.g], ..Default::default() };
        let ((t, w), (e, ew), (y, yv)) = (self.arms(s), self.elbows(s), self.lateral(s));
        let (p, pv) = self.platform(s);
        add_point(&mut f, self.m_nt(), p, pv);
        let rotor_speed = self.rotors(s).1;
        for i in 0..3 {
            let joint = self.legs.to_base(i, [self.base_radius, 0.0, 0.0]);
            let turn = |angle: f64| {
                let (sin, cos) = angle.sin_cos();
                (self.legs.to_base(i, [cos, 0.0, -sin]), self.legs.to_base(i, [-sin, 0.0, -cos]))
            };
            let (dir, ddir) = turn(t[i]);
            add_turning(&mut f, joint, dir, ddir, w[i], self.root_inertia, self.root_moment);
            let (tip, dtip) = turn(e[i]);
            add_turning(&mut f, joint, tip, dtip, ew[i], self.tip_inertia, self.tip_moment);
            // The elbow's move out of the plane; its turn is in the tip's share already.
            let (tangent, la) = (self.legs.tangent(i), self.legs.la);
            let in_plane: [f64; 3] = std::array::from_fn(|k| joint[k] + la * tip[k]);
            let elbow: [f64; 3] = std::array::from_fn(|k| in_plane[k] + y[i] * tangent[k]);
            add_point(&mut f, self.m_lateral, elbow, tangent.map(|v| v * yv[i]));
            add_point(&mut f, -self.m_lateral, in_plane, [0.0; 3]);
            let elbow_speed: [f64; 3] = std::array::from_fn(|k| la * ew[i] * dtip[k] + yv[i] * tangent[k]);
            let at_platform = self.legs.to_base(i, [self.platform_radius, 0.0, 0.0]);
            let middle: [f64; 3] = std::array::from_fn(|k| 0.5 * (elbow[k] + p[k] + at_platform[k]));
            add_point(&mut f, self.m_rod_middle, middle, std::array::from_fn(|k| 0.5 * (elbow_speed[k] + pv[k])));
            add_spin(&mut f, tangent, self.joints[i].rotor_inertia, rotor_speed[i]);
        }
        Some(f)
    }
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    /// NaN where the pose is out of reach: the controller never asks for one (see `unreachable`).
    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        let t = self.legs.inverse([tool[0], tool[1], tool[2]]).unwrap_or([f64::NAN; 3]);
        for k in 0..3 {
            motors[k] = self.ratio() * t[k];
        }
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        let i = self.ratio();
        let p = self.legs.forward([motors[0] / i, motors[1] / i, motors[2] / i]).unwrap_or([f64::NAN; 3]);
        tool[..3].copy_from_slice(&p);
    }

    fn state_len(&self) -> usize {
        self.bend_at() + if self.bending.is_some() { 12 } else { 0 }
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        // The elbows at the pose; the joints short of them by the arms' bend under the weight.
        let (t, gravity, bend, lateral) = self.rest_at([tool[0], tool[1], tool[2]], self.m_nt());
        state.clear();
        state.extend(t);
        state.extend([0.0; 3]);
        for k in 0..3 {
            let joint = &self.joints[k];
            // At rest the gearbox holds the arm against gravity and the stops.
            let out = -gravity[k] - joint.output_torque(t[k], 0.0);
            let rotor = joint.ratio() * (t[k] + if self.elastic { joint.gearbox.static_twist(out) } else { 0.0 });
            if self.elastic {
                state.push(rotor);
            }
            torque[k] = joint.ripple.invert(out / joint.ratio(), rotor);
        }
        // Rotor speeds and brakes at rest; the bend still.
        state.resize(self.bend_at(), 0.0);
        if self.bending.is_some() {
            state.extend(bend);
            state.extend([0.0; 3]);
            state.extend(lateral);
        }
        state.resize(self.state_len(), 0.0);
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        if !self.elastic {
            let (acc, ..) = self.rigid_solve(s, torque);
            let (_, rotor_speed) = self.rotors(s);
            let brakes = self.brakes(s);
            out[..3].copy_from_slice(&s[3..6]);
            out[3..6].copy_from_slice(&acc);
            for k in 0..3 {
                out[6 + k] = self.joints[k].brake.torque(self.brake_engaged, rotor_speed[k], brakes[k]).1;
            }
            self.bend_rates(s, acc, out);
            return;
        }
        let (t, w) = self.arms(s);
        let (mass, velocity, gravity) = self.joint_load(s);
        let mut rhs = [0.0; 3];
        for k in 0..3 {
            let joint = &self.joints[k];
            let (gear, rotor_acc, brake_rate) =
                joint.rotor(torque[k], s[6 + k], s[9 + k], t[k], w[k], s[12 + k], self.brake_engaged);
            rhs[k] = gear.output + gravity[k] - velocity[k] + joint.output_torque(t[k], w[k]);
            out[6 + k] = s[9 + k];
            out[9 + k] = rotor_acc;
            out[12 + k] = brake_rate;
        }
        let acc = inverse(&mass).map_or([f64::NAN; 3], |inv| mul(&inv, rhs));
        out[..3].copy_from_slice(&s[3..6]);
        out[3..6].copy_from_slice(&acc);
        self.bend_rates(s, acc, out);
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (p, v) = self.platform(s);
        pos[..3].copy_from_slice(&p);
        vel[..3].copy_from_slice(&v);
    }
    /// The arms turn at the speed, their bend (in and out of their planes) standing still.
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        let ((t, _), (y, _)) = (self.elbows(s), self.lateral(s));
        let (_, j, _) = self.pose_moved(t, y);
        let w = inverse(&j).map_or([f64::NAN; 3], |inv| mul(&inv, [vel[0], vel[1], vel[2]]));
        s[3..6].copy_from_slice(&w);
        if self.bending.is_some() {
            let at = self.bend_at();
            s[at + 3..at + 6].fill(0.0);
            s[at + 9..at + 12].fill(0.0);
        }
        if self.elastic {
            for k in 0..3 {
                s[9 + k] = self.joints[k].ratio() * w[k];
            }
        }
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (rotor, speed) = self.rotors(s);
        pos[..3].copy_from_slice(&rotor);
        vel[..3].copy_from_slice(&speed);
    }

    /// Rigid, loss-free, at rest: M theta'' - Q_g at the arm over the ratio, plus the rotor's own J_rotor i theta''.
    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        let t = self.legs.inverse([pos[0], pos[1], pos[2]]).unwrap_or([f64::NAN; 3]);
        let (mass, _, gravity) = self.load(t, [0.0; 3]);
        let (_, j) = self.pose(t);
        let arm_acc = inverse(&j).map_or([f64::NAN; 3], |inv| mul(&inv, [acc[0], acc[1], acc[2]]));
        let at_arm = mul(&mass, arm_acc);
        for k in 0..3 {
            let joint = &self.joints[k];
            out[k] = (at_arm[k] - gravity[k]) / joint.ratio() + joint.rotor_inertia * joint.ratio() * arm_acc[k];
        }
    }

    /// With the speed: θ̇ = J⁻¹ ṗ, θ̈ = J⁻¹ (p̈ − J_x⁻¹ w), and at the arm M θ̈ + v − Q_g (`load`: with the lumped
    /// forearms that is I_b θ̈ + m_nt Jᵀ p̈ − Q_g; with rods their middles' too), over the ratio, plus the rotor's own
    /// J_rotor i θ̈. The links rigid.
    fn feedforward_torque_moving(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut [f64]) {
        let p = [pos[0], pos[1], pos[2]];
        let t = self.legs.inverse(p).unwrap_or([f64::NAN; 3]);
        let (_, j) = self.pose(t);
        let inv = inverse(&j).unwrap_or([[f64::NAN; 3]; 3]);
        let (pv, pa) = ([vel[0], vel[1], vel[2]], [acc[0], acc[1], acc[2]]);
        let w = mul(&inv, pv);
        let bend = self.legs.velocity_term(t, p, w, pv).unwrap_or([f64::NAN; 3]);
        let arm_acc = mul(&inv, std::array::from_fn(|i| pa[i] - bend[i]));
        let (mass, velocity, gravity) = self.load(t, w);
        // The rigid model is loss-free: a platform drag is friction, which `load` puts in the velocity term for the
        // dynamics, and is taken out again here. What stays is quadratic in the speed, as the time-optimal planner reads
        // it: a sliding friction of 5 N read as d v^2 from its value at speed 1 grew ninefold at speed 3, and held the
        // tool's turn on a path to 15 rad/s where its motor allows 39.
        let drag = self.drag_torque(t, w);
        let at_arm = mul(&mass, arm_acc);
        for k in 0..3 {
            let joint = &self.joints[k];
            out[k] = (at_arm[k] + velocity[k] + drag[k] - gravity[k]) / joint.ratio()
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

    /// What the gearboxes can sag at rest anywhere in the working range (`rest_sag`): 2.30 mm with the catalogue's 9
    /// arcmin backlash (the simulation at rest puts the motors' pose up to 2.09 mm off the tool's at the rim's lower edge,
    /// between two arms, 0.75 mm at the centre, 0.64 mm at pick and place; its arms rest at one side of their backlash,
    /// the bound takes either). Until 13 November 2026 a fixed 1.5 mm, which 9 arcmin outgrew.
    fn setpoint_slack(&self) -> f64 {
        self.rest_slack
    }

    fn unreachable(&self, tool: &[f64], slack: f64) -> Option<&'static str> {
        if tool.len() != 3 {
            return Some("error-target-out-of-range");
        }
        let p = [tool[0], tool[1], tool[2]];
        if !self.in_envelope(&p, slack) {
            return Some("error-delta-outside-envelope");
        }
        let Some(t) = self.legs.inverse(p) else { return Some("error-delta-out-of-reach") };
        let (min, max) = self.joints[0].range;
        if t.iter().any(|a| *a < min || *a > max) {
            return Some("error-delta-joint-stop");
        }
        match self.legs.jacobian(t, p) {
            Some(j) if condition_number(&j) <= self.max_condition => None,
            _ => Some("error-delta-singularity"),
        }
    }

    /// Of the arm angles, elbows out, as the signal `kinematics.condition` and the check against
    /// `workspace.max_condition_number`.
    fn condition(&self, tool: &[f64]) -> Option<f64> {
        let p = [tool[0], tool[1], tool[2]];
        let j = self.legs.inverse(p).and_then(|t| self.legs.jacobian(t, p));
        Some(j.map_or(f64::INFINITY, |j| condition_number(&j)))
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out = Vec::new();
        for (k, motor) in self.motors.iter().enumerate() {
            let arm = ["arm-1", "arm-2", "arm-3"][k];
            out.push(SignalInfo::new(format!("arm.{}.angle", motor.id), arm, "quantity-arm-angle", "rad"));
            out.push(SignalInfo::new(
                format!("gearbox.{}.output_torque", motor.id),
                motor.key,
                "quantity-gearbox-torque",
                "N m",
            ));
        }
        out.push(SignalInfo::new("kinematics.condition", "group-kinematics", "quantity-condition-number", ""));
        self.effect_infos(&mut out);
        out
    }

    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (t, _) = self.arms(s);
        let gear = self.gear_output(s, torque);
        for k in 0..3 {
            out.extend([t[k], gear[k]]);
        }
        let (_, j) = self.pose(self.elbows(s).0);
        out.push(condition_number(&j));
    }
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.view_signals(s, torque, out);
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(&self.losses(s, torque).powers[..self.loss_count()]);
    }

    // shortcut: the joints' friction is not offered to the friction view, which draws a guide's force over its speed
    // along an axis (N over m/s); the joints' losses are in the energy budget and the torque breakdown. Offer them as
    // contacts once the view can draw a torque over an angular speed.

    /// The three gearboxes, where there are gearboxes (not on a direct drive).
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        if self.joints[0].gearbox.is_coupling() {
            return Vec::new();
        }
        [("gearbox_1", "motor-arm-1"), ("gearbox_2", "motor-arm-2"), ("gearbox_3", "motor-arm-3")]
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
        // The arms drawn straight to their elbows, bent or not.
        let ((t, _), (y, _)) = (self.elbows(s), self.lateral(s));
        let (rotor, rotor_speed) = self.rotors(s);
        let (p, ..) = self.pose_moved(t, y);
        out.parts.clear();
        out.labels.clear();
        let grey = [0.62, 0.64, 0.68];
        out.parts.push(Part::new(
            "base",
            Shape::Box { size: [2.4 * self.base_radius, 2.4 * self.base_radius, 0.04] },
            [0.0, 0.0, 0.1],
            grey,
        ));
        for k in 0..3 {
            let (c, sn) = self.legs.azimuth[k];
            let (shoulder, arm_end) = self.arm(k, t, y);
            let at = |r: f64, z: f64| [r * c, r * sn, z];
            out.parts.push(Part::new(
                sim_core::scene::MOTOR_PART,
                Shape::Box { size: [0.09, 0.09, 0.12] },
                at(self.base_radius, 0.16),
                [0.9, 0.45, 0.1],
            ));
            out.parts.push(Part::new(
                "gearbox",
                Shape::Box { size: [0.09, 0.09, 0.06] },
                at(self.base_radius, 0.05),
                [0.25, 0.25, 0.28],
            ));
            out.parts.push(
                Part::new(
                    "rotor-bar",
                    Shape::Box { size: [0.07, 0.004, 0.012] },
                    at(self.base_radius, 0.225),
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
            for (from, to) in self.legs.rods(k, t[k], y[k], p, self.platform_radius, self.rod_spacing) {
                out.parts.push(Part::new("forearm", Shape::Segment { to, width: ROD_WIDTH }, from, [0.15, 0.15, 0.15]));
            }
            out.labels.push(Label {
                pos: at(self.base_radius, 0.3),
                owner_key: self.motors[k].key,
                value: rotor_speed[k] * 60.0 / std::f64::consts::TAU,
                unit_key: "unit-rpm",
                decimals: 0,
            });
        }
        out.parts.extend(self.on_platform(p));
        out.labels.push(Label {
            pos: [p[0], p[1], p[2] - 0.15],
            owner_key: "axis-z",
            value: p[2],
            unit_key: "unit-m",
            decimals: 3,
        });
    }

    /// The platform's centre: the machine's frame is the scene's.
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        Some([tool_pose[0], tool_pose[1], tool_pose[2]])
    }

    /// The platform and the payload under it, the six forearm rods and the three upper arms, where `scene` draws them,
    /// without drawing the rest: the live check asks for them every base step. Each rod is `ROD_PIECES` boxes along it,
    /// each arm `ARM_PIECES`, each the box around its piece of the drawn segment widened by the segment's width.
    // shortcut: a slanted segment's pieces are boxes along the scene's axes, up to a piece's length (95 mm on a rod, 40
    // mm on an arm) wider than the segment; capsules if a near miss of a rod or an arm ever matters.
    fn collision_bodies(&self, s: &[f64]) -> Vec<sim_core::Aabb> {
        let ((t, _), (y, _)) = (self.elbows(s), self.lateral(s));
        let p = self.pose_moved(t, y).0;
        let mut out = sim_core::scene::scene_boxes(&self.on_platform(p), &["platform", "payload"]);
        for k in 0..3 {
            for (from, to) in self.legs.rods(k, t[k], y[k], p, self.platform_radius, self.rod_spacing) {
                out.extend(segment_boxes(from, to, ROD_WIDTH, ROD_PIECES));
            }
        }
        for k in 0..3 {
            let (shoulder, arm_end) = self.arm(k, t, y);
            out.extend(segment_boxes(shoulder, arm_end, ARM_WIDTH, ARM_PIECES));
        }
        out
    }

    /// The lowest of `modes`: the gearboxes' springs (effects.joint_elasticity) and the arms' bending in and out of
    /// their planes (effects.link_elasticity), in series where both are on. At the centre the lowest is sideways
    /// (twice).
    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        self.modes_at(tool_pose).first().map(|mode| mode.0)
    }

    /// The mode of `resonance_hz`: the joints' turns in the arm angles, the bend (elbow less joint) in the bend states,
    /// the elbows' moves out of the arms' planes in theirs, rotors still; scaled so that the platform's largest travel
    /// along an axis is 1 m.
    fn mode_shape(&self, tool_pose: &[f64]) -> Option<(f64, Vec<f64>)> {
        let (hz, _, v) = self.modes_at(tool_pose).into_iter().next()?;
        let p = [tool_pose[0], tool_pose[1], tool_pose[2]];
        let (j, jy) = self.legs.jacobians(self.legs.inverse(p)?, [0.0; 3], p)?;
        let (root, tip) = ([v[0], v[1], v[2]], [v[3], v[4], v[5]]);
        let moves = if v.len() == 9 { [v[6], v[7], v[8]] } else { [0.0; 3] };
        let (a, b) = (mul(&j, tip), mul(&jy, moves));
        let largest = (0..3).fold(0f64, |m, k| m.max((a[k] + b[k]).abs()));
        let mut shape = vec![0.0; self.state_len()];
        shape[..3].copy_from_slice(&root.map(|v| v / largest));
        if self.bending.is_some() {
            let at = self.bend_at();
            for k in 0..3 {
                shape[at + k] = (tip[k] - root[k]) / largest;
                shape[at + 6 + k] = moves[k] / largest;
            }
        }
        Some((hz, shape))
    }

    /// Every mode of `modes_at` with its damping ratio: 3 with the gearboxes' springs alone (one per arm), 6 with the
    /// arms' bending alone (in and out of their planes), 9 with both.
    fn modes(&self, tool_pose: &[f64]) -> Vec<Mode> {
        self.modes_at(tool_pose).into_iter().map(|(hz, damping, _)| Mode { hz, damping }).collect()
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

/// The forearm rods' width as drawn and as collision bodies, m, and the pieces a rod's collision boxes cut it into;
/// the same for the upper arms.
const ROD_WIDTH: f64 = 0.012;
const ROD_PIECES: usize = 10;
const ARM_WIDTH: f64 = 0.04;
const ARM_PIECES: usize = 10;

#[cfg(test)]
mod tests;
