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
//! platform's m_nt J'theta' through J^T. Each joint is a `RotaryJoint` (gearbox with stiffness, backlash and
//! efficiency, motor with friction, ripple and brake, joint friction, stops). Elastic (effects.joint_elasticity) the
//! rotors are states of their own behind the gearboxes' springs; rigid, each rotor turns with its arm times the ratio and
//! its inertia counts at the arm times the ratio squared, the gearbox losing its efficiency on what passes through it.

use sim_components::joint::RotaryJoint;
use sim_components::mech::ThermalFeedback;
use sim_core::{AxisInfo, Config, ConfigError, ENERGY_PARTS, FRICTION_SIGNALS, FrictionContact, LOSS_PREFIX, Label, Machine, MachineEntry, MotorInfo, Part, Scene, Shape, SignalInfo, TORQUE_PARTS, ThermalPart};

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
pub const LOSSY_EFFECTS: [&str; 5] = ["joint_friction", "motor_friction", "gearbox_efficiency", "torque_ripple", "holding_brake"];

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    Ok(Box::new(Delta::new(cfg)?))
}

/// The default config with every loss switched off.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    cfg.set_text("gearbox.damping_nm_s_rad = 0.0").unwrap();
    cfg.set_text("joint.stop_damping_nm_s_rad = 0.0").unwrap();
    cfg
}

/// Loss-free rigid and elastic models, with and without the payload, and the elastic one with part heating and the
/// temperature feedback (no loss, so the gearboxes stay at ambient).
///
/// Their working range lies 175 mm lower than the default's (top 850 mm below the shoulders), so that the suite's
/// middle pose is 1043 mm down: held by constant motor torques, as the suite holds it for 10 s after a pulse, the delta
/// is unstable in its vertical mode in the upper part of its range, where the platform's weight pulls harder through
/// the Jacobian the further it sinks (negative stiffness; `the_vertical_mode_under_constant_torques_is_unstable_high_up`),
/// and stable below about 950 mm. A real property, not a numerical one: the drives' position loops hold it.
fn suite_configs() -> Vec<Config> {
    let mut out = Vec::new();
    for (elastic, payload) in [(false, true), (false, false), (true, true), (true, false)] {
        let mut cfg = lossless_config();
        cfg.set_text("workspace.top_z_m = -0.85").unwrap();
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
        [self.to_base(i, [self.e + la * c, 0.0, -la * s]), self.to_base(i, [-la * s, 0.0, -la * c]), self.to_base(i, [-la * c, 0.0, la * s])]
    }

    /// Arm angles for a platform position, elbows out; `None` out of reach.
    pub fn inverse(&self, p: V3) -> Option<V3> {
        let mut out = [0.0; 3];
        for (i, t) in out.iter_mut().enumerate() {
            let [x, y, z] = self.to_leg(i, p);
            let (a, b, c) = (-2.0 * self.la * (x - self.e), 2.0 * self.la * z, (x - self.e).powi(2) + y * y + z * z + self.la * self.la - self.lb * self.lb);
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
        let c = [0, 1, 2].map(|i| self.elbow(i, t[i])[0]);
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
        let (up, down): (V3, V3) = (std::array::from_fn(|k| base[k] + z2.sqrt() * ez[k]), std::array::from_fn(|k| base[k] - z2.sqrt() * ez[k]));
        Some(if up[2] < down[2] { up } else { down })
    }

    /// J_x (rows: the forearm vectors u_i = p - c_i) and the diagonal of J_q (u_i . dc_i/dt).
    fn constraint(&self, t: V3, p: V3) -> (M3, V3) {
        let rows: [[V3; 3]; 3] = [0, 1, 2].map(|i| self.elbow(i, t[i]));
        let jx = rows.map(|e| sub(p, e[0]));
        (jx, [0, 1, 2].map(|i| dot(jx[i], rows[i][1])))
    }

    /// The Jacobian J[i][j] = dp_i / dtheta_j at arm angles t and platform position p; `None` at a direct singularity.
    pub fn jacobian(&self, t: V3, p: V3) -> Option<M3> {
        let (jx, jq) = self.constraint(t, p);
        let inv = inverse(&jx)?;
        Some(std::array::from_fn(|i| std::array::from_fn(|j| inv[i][j] * jq[j])))
    }

    /// J' theta' = J_x^-1 w: the platform's acceleration at arm speeds `w_t` and no arm acceleration.
    pub fn velocity_term(&self, t: V3, p: V3, wt: V3, pv: V3) -> Option<V3> {
        let (jx, _) = self.constraint(t, p);
        let inv = inverse(&jx)?;
        let w: V3 = std::array::from_fn(|i| {
            let [_, d1, d2] = self.elbow(i, t[i]);
            let rel = sub(pv, d1.map(|v| v * wt[i]));
            wt[i] * wt[i] * dot(jx[i], d2) - dot(rel, rel)
        });
        Some(mul(&inv, w))
    }
}

pub struct Delta {
    legs: Legs,
    /// The geometry as configured, for the scene: base and platform radius, the rods' spacing.
    base_radius: f64,
    platform_radius: f64,
    rod_spacing: f64,
    elastic: bool,
    g: f64,
    /// I_b without the rotor, m_LG, platform mass without and with payload (forearm share included).
    arm_inertia: f64,
    arm_moment: f64,
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
}

// Elastic state: [theta 1..3, omega 1..3, rotor angle 1..3, rotor speed 1..3, brake 1..3].
const N_ELASTIC: usize = 15;
// Rigid state: [theta 1..3, omega 1..3, brake 1..3].
const N_RIGID: usize = 9;

/// The losses of one moment: per joint its friction, its motor's friction with the brake, its gearbox; the stops and
/// what the ripple gives off; and per joint the gearbox loss as a torque at the rotor.
struct Losses {
    powers: [f64; 11],
    gear_at_rotor: V3,
}

impl Delta {
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
        let (m_arm, cg, i_arm) = (cfg.positive("mass.upper_arm_kg")?, cfg.non_negative("mass.upper_arm_cg_m")?, cfg.positive("mass.upper_arm_inertia_kgm2")?);
        let (m_elbow, m_fb) = (cfg.non_negative("mass.elbow_kg")?, cfg.non_negative("mass.forearm_pair_kg")?);
        let elbow = m_elbow + share * m_fb;
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
        Ok(Self {
            legs: Legs::new(base_radius - platform_radius, la, lb),
            base_radius,
            platform_radius,
            rod_spacing: cfg.non_negative("geometry.forearm_spacing_m")?,
            elastic: cfg.bool("effects.joint_elasticity")?,
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            arm_inertia: i_arm + la * la * elbow,
            arm_moment: m_arm * cg + la * elbow,
            m_platform: cfg.positive("mass.platform_kg")? + 3.0 * (1.0 - share) * m_fb,
            m_payload: if cfg.bool("effects.payload")? { cfg.non_negative("mass.payload_kg")? } else { 0.0 },
            payload_attached: true,
            brake_engaged: false,
            joints: [joint.clone(), joint.clone(), joint],
            thermal: ThermalFeedback::from_config(cfg)?,
            radius,
            top,
            sphere,
            max_condition: cfg.positive("workspace.max_condition_number")?,
            axes: [axis("x", "axis-x", -radius, radius)?, axis("y", "axis-y", -radius, radius)?, axis("z", "axis-z", top - centre, top)?],
            motors: [MotorInfo { id: "1", key: "motor-arm-1" }, MotorInfo { id: "2", key: "motor-arm-2" }, MotorInfo { id: "3", key: "motor-arm-3" }],
        })
    }

    pub fn legs(&self) -> &Legs {
        &self.legs
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
        self.arm_inertia
    }

    /// m_LG: the arm's gravity torque is g m_LG cos(theta).
    pub fn arm_moment(&self) -> f64 {
        self.arm_moment
    }

    /// Arm angles and speeds of a state.
    fn arms(&self, s: &[f64]) -> (V3, V3) {
        ([s[0], s[1], s[2]], [s[3], s[4], s[5]])
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
        match self.legs.forward(t).and_then(|p| Some((p, self.legs.jacobian(t, p)?))) {
            Some(found) => found,
            None => ([f64::NAN; 3], [[f64::NAN; 3]; 3]),
        }
    }

    /// The load's mass matrix M, the velocity term m_nt J^T J' theta', and the generalised gravity Q_g at arm angles t
    /// and speeds w.
    fn load(&self, t: V3, w: V3) -> (M3, V3, V3) {
        let (p, j) = self.pose(t);
        let jt = transpose(&j);
        let m = self.m_nt();
        let mass: M3 = std::array::from_fn(|r| std::array::from_fn(|c| m * dot(jt[r], jt[c]) + if r == c { self.arm_inertia } else { 0.0 }));
        let pv = mul(&j, w);
        let velocity = self.legs.velocity_term(t, p, w, pv).map_or([f64::NAN; 3], |a| mul(&jt, a.map(|v| m * v)));
        let gravity: V3 = std::array::from_fn(|k| self.g * self.arm_moment * t[k].cos() - m * self.g * j[2][k]);
        (mass, velocity, gravity)
    }

    /// Rigid model: the arms' acceleration, and per joint the torque into its gearbox (after the rotor's own
    /// acceleration) and out of it, at the arm.
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> (V3, V3, V3) {
        let (t, w) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let (mass, velocity, gravity) = self.load(t, w);
        let drive: V3 = std::array::from_fn(|k| self.joints[k].rigid_drive(torque[k], rotor[k], rotor_speed[k], brakes[k], self.brake_engaged).0);
        let rest: V3 = std::array::from_fn(|k| gravity[k] - velocity[k] + self.joints[k].output_torque(t[k], w[k]));
        let solve = |loss: V3| -> V3 {
            let m: M3 = std::array::from_fn(|r| std::array::from_fn(|c| mass[r][c] + if r == c { loss[r] * self.joints[r].reflected_inertia() } else { 0.0 }));
            let rhs: V3 = std::array::from_fn(|k| loss[k] * drive[k] + rest[k]);
            inverse(&m).map_or([f64::NAN; 3], |inv| mul(&inv, rhs))
        };
        let first = solve([1.0; 3]);
        let loss: V3 = std::array::from_fn(|k| self.joints[k].gearbox.loss_factor(drive[k] - self.joints[k].reflected_inertia() * first[k], w[k]));
        let acc = solve(loss);
        let into: V3 = std::array::from_fn(|k| drive[k] - self.joints[k].reflected_inertia() * acc[k]);
        (acc, into, std::array::from_fn(|k| loss[k] * into[k]))
    }

    /// The arms' acceleration in either model.
    fn arm_acceleration(&self, s: &[f64], torque: &[f64]) -> V3 {
        if self.elastic {
            let mut d = [0.0; N_ELASTIC];
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
        let (mass, ..) = self.load(t, w);
        let mut kinetic = 0.5 * dot(w, mul(&mass, w));
        let (p, _) = self.pose(t);
        let potential = self.g * (self.m_nt() * p[2] - self.arm_moment * (t[0].sin() + t[1].sin() + t[2].sin()));
        let mut elastic: f64 = (0..3).map(|k| self.joints[k].stop_energy(t[k])).sum();
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
        let mut powers = [0.0; 11];
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
        Losses { powers, gear_at_rotor }
    }

    /// Whether a pose is within the working range (cylinder, top, spherical bottom), widened by `slack`.
    fn in_envelope(&self, p: &[f64], slack: f64) -> bool {
        let (centre, r) = self.sphere;
        let below = if centre.is_finite() { (p[0] * p[0] + p[1] * p[1] + (p[2] - centre).powi(2)).sqrt() <= r + slack } else { p[2] >= self.axes[2].min - slack };
        p[0].hypot(p[1]) <= self.radius + slack && p[2] <= self.top + slack && below
    }

    /// The torque breakdown, loss powers, energies, friction signals of the effect views.
    fn view_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (t, w) = self.arms(s);
        let (_, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let (mass, velocity, gravity) = self.load(t, w);
        let acc = self.arm_acceleration(s, torque);
        let moving = mul(&mass, acc);
        let losses = self.losses(s, torque);
        for k in 0..3 {
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
        out.extend(losses.powers);
        out.extend(self.energy_parts(s));
        for k in 0..3 {
            let f = &self.joints[k].friction;
            out.extend([w[k], f.coulomb * (w[k] / f.smoothing).tanh(), f.viscous * w[k], 0.0, 1.0]);
        }
    }

    fn effect_infos(&self, out: &mut Vec<SignalInfo>) {
        for motor in &self.motors {
            out.extend(TORQUE_PARTS.map(|part| SignalInfo::new(format!("motor.{}.torque_{part}", motor.id), motor.key, &format!("quantity-torque-{part}"), "N m")));
        }
        for (k, motor) in self.motors.iter().enumerate() {
            let arm = ["arm-1", "arm-2", "arm-3"][k];
            let gear = if self.joints[k].gearbox.is_coupling() { "quantity-coupling-loss-power" } else { "quantity-gearbox-loss-power" };
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}joint_{}", motor.id), arm, "quantity-friction-power", "W"));
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}shaft_{}", motor.id), motor.key, "quantity-shaft-friction-power", "W"));
            out.push(SignalInfo::new(format!("{LOSS_PREFIX}gearbox_{}", motor.id), motor.key, gear, "W"));
        }
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}hard_stops"), "group-joint", "quantity-damping-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}other"), "loss-other", "quantity-power-given-off", "W"));
        out.extend(ENERGY_PARTS.map(|part| SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")));
        for contact in self.friction_contacts() {
            for part in FRICTION_SIGNALS {
                let unit = match part {
                    "speed" => "rad/s",
                    "load_scale" => "",
                    _ => "N m",
                };
                out.push(SignalInfo::new(format!("{}.friction_{part}", contact.id), contact.owner_key, &format!("quantity-friction-{}", part.replace('_', "-")), unit));
            }
        }
    }
}

impl Machine for Delta {
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
        if self.elastic { N_ELASTIC } else { N_RIGID }
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let t = self.legs.inverse([tool[0], tool[1], tool[2]]).unwrap_or([f64::NAN; 3]);
        let (_, _, gravity) = self.load(t, [0.0; 3]);
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
        // Rotor speeds and brakes at rest.
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
            return;
        }
        let (t, w) = self.arms(s);
        let (mass, velocity, gravity) = self.load(t, w);
        let mut rhs = [0.0; 3];
        for k in 0..3 {
            let joint = &self.joints[k];
            let (gear, rotor_acc, brake_rate) = joint.rotor(torque[k], s[6 + k], s[9 + k], t[k], w[k], s[12 + k], self.brake_engaged);
            rhs[k] = gear.output + gravity[k] - velocity[k] + joint.output_torque(t[k], w[k]);
            out[6 + k] = s[9 + k];
            out[9 + k] = rotor_acc;
            out[12 + k] = brake_rate;
        }
        let acc = inverse(&mass).map_or([f64::NAN; 3], |inv| mul(&inv, rhs));
        out[..3].copy_from_slice(&s[3..6]);
        out[3..6].copy_from_slice(&acc);
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (t, w) = self.arms(s);
        let (p, j) = self.pose(t);
        pos[..3].copy_from_slice(&p);
        vel[..3].copy_from_slice(&mul(&j, w));
    }
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        let (t, _) = self.arms(s);
        let (_, j) = self.pose(t);
        let w = inverse(&j).map_or([f64::NAN; 3], |inv| mul(&inv, [vel[0], vel[1], vel[2]]));
        s[3..6].copy_from_slice(&w);
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

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out = Vec::new();
        for (k, motor) in self.motors.iter().enumerate() {
            let arm = ["arm-1", "arm-2", "arm-3"][k];
            out.push(SignalInfo::new(format!("arm.{}.angle", motor.id), arm, "quantity-arm-angle", "rad"));
            out.push(SignalInfo::new(format!("gearbox.{}.output_torque", motor.id), motor.key, "quantity-gearbox-torque", "N m"));
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
        let (_, j) = self.pose(t);
        out.push(condition_number(&j));
    }
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.view_signals(s, torque, out);
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(s, torque).powers);
    }

    /// The arms' joint friction, as friction contacts of the friction view (Coulomb and viscous, no stiction).
    fn friction_contacts(&self) -> Vec<FrictionContact> {
        vec![FrictionContact { id: "joint.1", owner_key: "arm-1" }, FrictionContact { id: "joint.2", owner_key: "arm-2" }, FrictionContact { id: "joint.3", owner_key: "arm-3" }]
    }
    fn friction_curve(&self, i: usize, speed: f64, _load_scale: f64) -> [f64; 3] {
        let Some(joint) = self.joints.get(i) else { return [0.0; 3] };
        let f = &joint.friction;
        [f.coulomb * (speed / f.smoothing).tanh(), f.viscous * speed, 0.0]
    }

    /// The three gearboxes, where there are gearboxes (not on a direct drive).
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        if self.joints[0].gearbox.is_coupling() {
            return Vec::new();
        }
        [("gearbox_1", "motor-arm-1"), ("gearbox_2", "motor-arm-2"), ("gearbox_3", "motor-arm-3")]
            .into_iter()
            .enumerate()
            .map(|(k, (id, owner_key))| ThermalPart { id, kind: "gearbox", owner_key, scene_part: "gearbox", scene_index: k })
            .collect()
    }

    fn scene(&self, s: &[f64], out: &mut Scene) {
        let (t, _) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let (p, _) = self.pose(t);
        out.parts.clear();
        out.labels.clear();
        let grey = [0.62, 0.64, 0.68];
        out.parts.push(Part::new("base", Shape::Box { size: [2.4 * self.base_radius, 2.4 * self.base_radius, 0.04] }, [0.0, 0.0, 0.1], grey));
        for k in 0..3 {
            let (c, sn) = self.legs.azimuth[k];
            let shoulder = [self.base_radius * c, self.base_radius * sn, 0.0];
            // The elbow drawn where the arm ends: the legs' elbow is at e, the arm's end at the base radius.
            let [elbow, ..] = self.legs.elbow(k, t[k]);
            let arm_end = [elbow[0] + self.platform_radius * c, elbow[1] + self.platform_radius * sn, elbow[2]];
            let at = |r: f64, z: f64| [r * c, r * sn, z];
            out.parts.push(Part::new(sim_core::scene::MOTOR_PART, Shape::Box { size: [0.09, 0.09, 0.12] }, at(self.base_radius, 0.16), [0.9, 0.45, 0.1]));
            out.parts.push(Part::new("gearbox", Shape::Box { size: [0.09, 0.09, 0.06] }, at(self.base_radius, 0.05), [0.25, 0.25, 0.28]));
            out.parts.push(Part::new("rotor-bar", Shape::Box { size: [0.07, 0.004, 0.012] }, at(self.base_radius, 0.225), [0.1, 0.1, 0.1]).turned(rotor[k]));
            out.parts.push(Part::new("arm", Shape::Segment { to: arm_end, width: 0.04 }, shoulder, [0.2, 0.22, 0.26]));
            // The two rods of the forearm pair, apart along the elbow's axis (tangential to the base).
            let half = [-0.5 * self.rod_spacing * sn, 0.5 * self.rod_spacing * c, 0.0];
            let platform_end = [p[0] + self.platform_radius * c, p[1] + self.platform_radius * sn, p[2]];
            for side in [-1.0, 1.0] {
                let from = std::array::from_fn(|i| arm_end[i] + side * half[i]);
                let to = std::array::from_fn(|i| platform_end[i] + side * half[i]);
                out.parts.push(Part::new("forearm", Shape::Segment { to, width: 0.012 }, from, [0.15, 0.15, 0.15]));
            }
            out.labels.push(Label { pos: at(self.base_radius, 0.3), owner_key: self.motors[k].key, value: rotor_speed[k] * 60.0 / std::f64::consts::TAU, unit_key: "unit-rpm", decimals: 0 });
        }
        let size = 2.4 * self.platform_radius.max(0.03);
        out.parts.push(Part::new("platform", Shape::Box { size: [size, size, 0.03] }, p, [0.55, 0.55, 0.6]));
        if self.payload_attached && self.m_payload > 0.0 {
            out.parts.push(Part::new("payload", Shape::Box { size: [0.1, 0.1, 0.08] }, [p[0], p[1], p[2] - 0.06], [0.2, 0.7, 0.3]));
        }
        out.labels.push(Label { pos: [p[0], p[1], p[2] - 0.15], owner_key: "axis-z", value: p[2], unit_key: "unit-m", decimals: 3 });
    }

    /// The platform's centre: the machine's frame is the scene's.
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        Some([tool_pose[0], tool_pose[1], tool_pose[2]])
    }
}

#[cfg(test)]
mod tests;
