//! Wafer handler: the R-theta-Z arm of semiconductor and flat-panel handling. A column lifts (Z, motor 3, on a ball
//! screw) and turns (theta, motor 1) an arm of two links of equal length L in the horizontal plane. The upper arm turns
//! on the column (motor 2, riding on the column); inside it a band from a pulley fixed to the column to one on the
//! forearm turns the forearm against the upper arm at 2:1, and inside the forearm a band from a pulley fixed to the upper
//! arm to one on the end effector turns the effector against the forearm at 1:2. So the one motor 2 runs the wrist along
//! the radial line and the effector keeps pointing outwards along it. The tool pose is (r, theta, z): r the wafer
//! centre's distance from the column axis along the effector's direction, theta that direction, z the lift.
//!
//! Angle convention: phi is the upper arm's angle from the radial line (the column's direction theta), positive
//! anticlockwise; the links' absolute angles are
//!
//! ```text
//! upper arm  a1 = theta + phi
//! forearm    a2 = theta - phi + beta      (2:1 against the upper arm: a2 - a1 = -2 phi)
//! effector   a3 = theta + gamma           (1:2 against the forearm: a3 - a2 = phi)
//! wrist      W = L u(a1) + L u(a2) = 2 L cos(phi) u(theta)          (beta = gamma = 0)
//! wafer      C = W + e u(a3),  r = 2 L cos(phi) + e
//! ```
//!
//! with u(a) = (cos a, sin a) and e the effector's length from the wrist to the wafer centre. beta and gamma are the
//! bands' stretch (effects.band_elasticity), zero where they are rigid. dr/dphi = -2 L sin(phi): zero at full extension
//! (phi = 0, r = 2 L + e), the singular pose; largest at phi = 90 degrees (the wrist over the column). The working range
//! keeps phi above the angle where 1 / sin(phi) (`condition`) passes `workspace.max_condition_number`.
//!
//! Dynamics: four bodies in the plane (upper arm, forearm, effector, wafer) and the column's turn, each body's kinetic
//! energy 1/2 m |v|^2 + 1/2 I w^2 with v = J_v q' and w = a_k . q' over q = [theta, phi, beta, gamma] (Kane's method):
//!
//! ```text
//! M(q) q'' + h(q, q') = Q,   M = J_c + sum m J_v^T J_v + I a_k a_k^T,   h = sum m J_v^T (dJ_v/dt q')
//! ```
//!
//! h carries the centrifugal and Coriolis coupling of theta and phi: turning at theta' pushes the arm outwards (the
//! classic handler error, the wafer pushed out in a fast turn, held by motor 2), and extending at phi' during a turn
//! drags on motor 1. Gravity acts along -z, on the joints' bearings: it loads the Z motor alone. Each band is a loop of
//! two spans of stiffness k and pretension T0 over a distal pulley of radius r_b (the proximal one 2 r_b at the elbow
//! band, r_b / 2 at the wrist band); its stretch s1 = -r_b1 beta, s2 = r_b2 (beta / 2 - gamma), the spans' tensions
//! T0 +- k s (none once a span runs slack), and its torques on beta and gamma r_b1 dT1 - r_b2 dT2 / 2 and r_b2 dT2.
//! Motors 1 and 2 drive theta and phi through `RotaryJoint`s, elastic or rigid (effects.joint_elasticity); the Z stroke
//! is a rigid `RotaryJoint` on a `BallScrew` (motor 3's own config) that lifts the column, the links and the wafer.

use sim_components::catalog::motor_config;
use sim_components::joint::RotaryJoint;
use sim_components::mech::ThermalFeedback;
use sim_components::transmission::BallScrew;
use sim_core::{
    AxisInfo, Config, ConfigError, ENERGY_PARTS, LOSS_PREFIX, Label, Machine, MachineEntry, Mode, MotorInfo, Part,
    RatingCheck, Scene, Shape, SignalInfo, TORQUE_PARTS, ThermalPart,
};

const I18N: &[(&str, &str)] = &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))];

pub static ENTRY: MachineEntry = MachineEntry {
    id: "wafer-handler",
    key: "machine-wafer-handler",
    default_config: include_str!("../default.toml"),
    i18n: I18N,
    build,
    suite_configs,
};

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
    sim_components::base::mount(cfg, Box::new(WaferHandler::new(cfg)?))
}

/// The default config with every loss switched off: the effects above, the dampers of the gearboxes, the stops and
/// the bands.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    for key in [
        "gearbox.damping_nm_s_rad",
        "joint.stop_damping_nm_s_rad",
        "slide.stop_damping_n_s_m",
        "elbow_band.span_damping_n_s_m",
        "wrist_band.span_damping_n_s_m",
    ] {
        cfg.set_text(&format!("{key} = 0.0")).unwrap();
    }
    cfg
}

/// Loss-free models: rigid, elastic gearboxes, elastic bands and both, with and without the wafer; the elastic one
/// with part heating and the temperature feedback (no loss, so the gearboxes stay at ambient).
fn suite_configs() -> Vec<Config> {
    let mut out = Vec::new();
    for (gears, bands, payload) in
        [(false, false, true), (false, false, false), (true, true, true), (true, true, false), (false, true, true)]
    {
        let mut cfg = lossless_config();
        cfg.set_text(&format!("effects.joint_elasticity = {gears}")).unwrap();
        cfg.set_text(&format!("effects.band_elasticity = {bands}")).unwrap();
        cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
        cfg.set_text(&format!("effects.part_heating = {}", gears && payload)).unwrap();
        cfg.set_text(&format!("effects.thermal_feedback = {}", gears && payload)).unwrap();
        out.push(cfg);
    }
    out
}

/// A vector in the plane, (x, y).
pub type V2 = [f64; 2];
/// The arm's coordinates [theta, phi, beta, gamma], or rates of them.
pub type Q = [f64; 4];
/// A 4 x 4 matrix over them.
pub type Mat = [[f64; 4]; 4];

fn unit(a: f64) -> V2 {
    [a.cos(), a.sin()]
}

fn normal(a: f64) -> V2 {
    [-a.sin(), a.cos()]
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

/// Solves the leading `n` x `n` system m x = b by Gaussian elimination with partial pivoting; `None` where it is
/// singular. The entries beyond `n` come out zero.
pub fn solve(m: &Mat, b: &Q, n: usize) -> Option<Q> {
    let (mut a, mut x) = (*m, *b);
    for c in 0..n {
        let p = (c..n).max_by(|&i, &j| a[i][c].abs().total_cmp(&a[j][c].abs()))?;
        if !(a[p][c].abs() > 1e-300) {
            return None;
        }
        a.swap(c, p);
        x.swap(c, p);
        for r in c + 1..n {
            let f = a[r][c] / a[c][c];
            for k in c..n {
                a[r][k] -= f * a[c][k];
            }
            x[r] -= f * x[c];
        }
    }
    let mut out = [0.0; 4];
    for c in (0..n).rev() {
        let rest: f64 = (c + 1..n).map(|k| a[c][k] * out[k]).sum();
        out[c] = (x[c] - rest) / a[c][c];
    }
    out.iter().all(|v| v.is_finite()).then_some(out)
}

/// The eigenvalues of the symmetric generalised problem K v = lambda M v over the leading `n` coordinates, lowest
/// first: M = L L^T (Cholesky), then the symmetric L^-1 K L^-T by Jacobi rotations.
pub fn generalised_eigenvalues(m: &Mat, k: &Mat, n: usize) -> Option<Vec<f64>> {
    let mut l = [[0.0; 4]; 4];
    for i in 0..n {
        for j in 0..=i {
            let s: f64 = (0..j).map(|p| l[i][p] * l[j][p]).sum();
            if i == j {
                let d = m[i][i] - s;
                if !(d > 0.0) {
                    return None;
                }
                l[i][i] = d.sqrt();
            } else {
                l[i][j] = (m[i][j] - s) / l[j][j];
            }
        }
    }
    // X = L^-1 K, then A = X L^-T, row by row with forward substitution.
    let lower_solve = |b: [f64; 4]| {
        let mut y = [0.0; 4];
        for i in 0..n {
            y[i] = (b[i] - (0..i).map(|p| l[i][p] * y[p]).sum::<f64>()) / l[i][i];
        }
        y
    };
    let columns: Vec<[f64; 4]> = (0..n).map(|c| lower_solve(std::array::from_fn(|r| k[r][c]))).collect();
    let mut x = [[0.0; 4]; 4];
    for c in 0..n {
        for r in 0..n {
            x[r][c] = columns[c][r];
        }
    }
    let mut a = [[0.0; 4]; 4];
    for r in 0..n {
        let row = lower_solve(x[r]);
        a[r][..n].copy_from_slice(&row[..n]);
    }
    for _ in 0..100 {
        let off: f64 = (0..n)
            .flat_map(|i| (0..n).filter(move |&j| j != i).map(move |j| (i, j)))
            .map(|(i, j)| a[i][j] * a[i][j])
            .sum();
        if off < 1e-24 * (0..n).map(|i| a[i][i] * a[i][i]).sum::<f64>().max(1e-300) {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                if a[p][q].abs() < 1e-300 {
                    continue;
                }
                let theta = 0.5 * (2.0 * a[p][q]).atan2(a[q][q] - a[p][p]);
                let (s, c) = theta.sin_cos();
                for k in 0..n {
                    let (kp, kq) = (a[k][p], a[k][q]);
                    a[k][p] = c * kp - s * kq;
                    a[k][q] = s * kp + c * kq;
                }
                for k in 0..n {
                    let (pk, qk) = (a[p][k], a[q][k]);
                    a[p][k] = c * pk - s * qk;
                    a[q][k] = s * pk + c * qk;
                }
            }
        }
    }
    let mut out: Vec<f64> = (0..n).map(|i| a[i][i]).collect();
    out.sort_by(f64::total_cmp);
    Some(out)
}

/// The links' absolute angles over q = [theta, phi, beta, gamma]: upper arm, forearm, effector.
pub const LINK_ANGLES: [Q; 3] = [[1.0, 1.0, 0.0, 0.0], [1.0, -1.0, 1.0, 0.0], [1.0, 0.0, 0.0, 1.0]];

/// A rigid body of the arm: its mass, its inertia about its own centre (about z), and its centre's distance from the
/// joint it turns about, along its link.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Body {
    pub mass: f64,
    pub inertia: f64,
    pub centre: f64,
}

/// The arm's geometry and masses, apart from drives and bands.
#[derive(Debug, Clone)]
pub struct Arm {
    /// Both links' length L, and the effector's from the wrist to the wafer centre.
    pub link: f64,
    pub effector: f64,
    /// What turns with the column about its axis (the column's top, motor 2, the shoulder pulley), kg m^2.
    pub column_inertia: f64,
    /// Upper arm, forearm, effector.
    pub bodies: [Body; 3],
    /// The wafer, its centre at the effector's length.
    pub wafer: Body,
}

impl Arm {
    /// The links' absolute angles at q.
    pub fn angles(q: &Q) -> [f64; 3] {
        LINK_ANGLES.map(|a| dot(&a, q))
    }

    /// r(phi) = 2 L cos(phi) + e, and its first and second derivative by phi.
    pub fn reach(&self, phi: f64) -> [f64; 3] {
        let (s, c) = phi.sin_cos();
        let two_l = 2.0 * self.link;
        [two_l * c + self.effector, -two_l * s, -two_l * c]
    }

    /// The upper arm's angle phi from the radial line for a reach r; NaN beyond 2 L + e or behind -2 L + e.
    pub fn phi(&self, r: f64) -> f64 {
        let c = (r - self.effector) / (2.0 * self.link);
        if (-1.0..=1.0).contains(&c) { c.acos() } else { f64::NAN }
    }

    /// The segments from the column axis to a point on link `k` at `d` from its joint: (length, link).
    fn segments(&self, k: usize, d: f64) -> impl Iterator<Item = (f64, usize)> {
        (0..k).map(|j| (self.link, j)).chain(std::iter::once((d, k)))
    }

    /// The point on link `k` at `d` from its joint.
    pub fn point(&self, q: &Q, k: usize, d: f64) -> V2 {
        let a = Self::angles(q);
        self.segments(k, d).fold([0.0; 2], |p, (l, j)| {
            let u = unit(a[j]);
            [p[0] + l * u[0], p[1] + l * u[1]]
        })
    }

    /// That point's Jacobian dp/dq (rows x and y) and its acceleration at q'' = 0 (the centripetal terms).
    pub fn point_motion(&self, q: &Q, qd: &Q, k: usize, d: f64) -> ([Q; 2], V2) {
        let a = Self::angles(q);
        let w = LINK_ANGLES.map(|row| dot(&row, qd));
        let (mut jac, mut bias) = ([[0.0; 4]; 2], [0.0; 2]);
        for (l, j) in self.segments(k, d) {
            let (n, u) = (normal(a[j]), unit(a[j]));
            for r in 0..2 {
                for c in 0..4 {
                    jac[r][c] += l * n[r] * LINK_ANGLES[j][c];
                }
                bias[r] -= l * w[j] * w[j] * u[r];
            }
        }
        (jac, bias)
    }

    /// The wafer centre at q.
    pub fn wafer_centre(&self, q: &Q) -> V2 {
        self.point(q, 2, self.effector)
    }

    /// The bodies with their link and centre: upper arm, forearm, effector, and the wafer while it is held.
    fn moving(&self, wafer: bool) -> impl Iterator<Item = (usize, Body)> + '_ {
        let mut list = vec![(0, self.bodies[0]), (1, self.bodies[1]), (2, self.bodies[2])];
        if wafer {
            list.push((2, Body { centre: self.effector, ..self.wafer }));
        }
        list.into_iter()
    }

    /// The mass matrix M (column, links, effector, and the wafer while held) and the velocity term h = sum m J_v^T
    /// (dJ_v/dt q') at q and q', over all four coordinates.
    pub fn dynamics(&self, q: &Q, qd: &Q, wafer: bool) -> (Mat, Q) {
        let (mut m, mut h) = ([[0.0; 4]; 4], [0.0; 4]);
        m[0][0] = self.column_inertia;
        for (k, body) in self.moving(wafer) {
            let (jac, bias) = self.point_motion(q, qd, k, body.centre);
            for r in 0..4 {
                for c in 0..4 {
                    m[r][c] += body.mass * (jac[0][r] * jac[0][c] + jac[1][r] * jac[1][c])
                        + body.inertia * LINK_ANGLES[k][r] * LINK_ANGLES[k][c];
                }
                h[r] += body.mass * (jac[0][r] * bias[0] + jac[1][r] * bias[1]);
            }
        }
        (m, h)
    }

    /// The mass the arm moves in the plane (links, effector, wafer while held), kg.
    pub fn mass(&self, wafer: bool) -> f64 {
        self.moving(wafer).map(|(_, b)| b.mass).sum()
    }
}

/// A band in a link: a loop of two spans over a distal pulley of radius `radius` (the proximal pulley's radius is the
/// coupling's ratio times it), each span of `stiffness` and `damping`, tensioned to `pretension`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Band {
    pub radius: f64,
    pub stiffness: f64,
    pub damping: f64,
    pub pretension: f64,
}

impl Band {
    fn from_config(cfg: &Config, table: &str) -> Result<Self, ConfigError> {
        Ok(Self {
            radius: cfg.positive(&format!("{table}.pulley_radius_m"))?,
            stiffness: cfg.positive(&format!("{table}.span_stiffness_n_m"))?,
            damping: cfg.non_negative(&format!("{table}.span_damping_n_s_m"))?,
            pretension: cfg.positive(&format!("{table}.pretension_n"))?,
        })
    }

    /// A span's tension at an elongation `e` beyond its pretensioned length and its rate: T0 + k e + c e' while its
    /// spring holds tension, none once it runs slack.
    pub fn span_tension(&self, e: f64, rate: f64) -> f64 {
        if self.pretension + self.stiffness * e > 0.0 {
            self.pretension + self.stiffness * e + self.damping * rate
        } else {
            0.0
        }
    }

    /// The tension of the span stretched by `s` less that of the other, shortened by it.
    pub fn difference(&self, s: f64, rate: f64) -> f64 {
        self.span_tension(s, rate) - self.span_tension(-s, -rate)
    }

    /// The work stored in both spans from their pretensioned length: the integral of T0 + k x from 0 to +-s, constant
    /// once a span is slack: k s^2 while both hold tension.
    pub fn energy(&self, s: f64) -> f64 {
        let span = |e: f64| {
            let (t, k) = (self.pretension, self.stiffness);
            if t + k * e > 0.0 { t * e + 0.5 * k * e * e } else { -0.5 * t * t / k }
        };
        span(s) + span(-s)
    }

    /// The power its spans' dampers take: c s'^2 per span that holds tension.
    pub fn loss(&self, s: f64, rate: f64) -> f64 {
        let taut = |e: f64| self.pretension + self.stiffness * e > 0.0;
        (taut(s) as u8 + taut(-s) as u8) as f64 * self.damping * rate * rate
    }

    /// Its torsional stiffness at the distal pulley while both spans hold tension: 2 k r^2.
    pub fn torsional_stiffness(&self) -> f64 {
        2.0 * self.stiffness * self.radius * self.radius
    }
}

/// How the effector holds the wafer (`[wafer]`), for the verdict "wafer-slip".
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Hold {
    /// On friction pads, by its weight: it slips when |a_xy| > mu (g + a_z).
    Pads { mu: f64 },
    /// On vacuum pads: by its weight and the vacuum's force F over its mass m, |a_xy| > mu (g + a_z + F / m).
    Vacuum { mu: f64, lift: f64 },
    /// Clamped at its edge: up to an acceleration the grip takes, whatever its weight.
    Edge { acceleration: f64 },
}

impl Hold {
    fn from_config(cfg: &Config, wafer_kg: f64) -> Result<Self, ConfigError> {
        Ok(match cfg.get("wafer.hold")?.as_str() {
            Some("pads") => Self::Pads { mu: cfg.positive("wafer.friction_coefficient")? },
            Some("vacuum") => Self::Vacuum {
                mu: cfg.positive("wafer.friction_coefficient")?,
                lift: cfg.positive("wafer.vacuum_area_m2")? * cfg.positive("wafer.vacuum_pa")? / wafer_kg.max(1e-9),
            },
            Some("edge") => Self::Edge { acceleration: cfg.positive("wafer.edge_grip_acceleration_m_s2")? },
            _ => return Err(ConfigError::Invalid("wafer.hold".into())),
        })
    }

    /// The highest in-plane acceleration it holds at rest under gravity `g`, m/s^2: mu g, mu (g + F / m), or the grip's.
    pub fn at_rest(&self, g: f64) -> f64 {
        match *self {
            Self::Pads { mu } => mu * g,
            Self::Vacuum { mu, lift } => mu * (g + lift),
            Self::Edge { acceleration } => acceleration,
        }
    }

    /// The wafer's in-plane acceleration `a_xy` at the vertical acceleration `a_z` as the in-plane acceleration that asks
    /// the same share of the hold at rest: |a_xy| (g + F / m) / (g + a_z + F / m) on pads, |a_xy| clamped at the edge. A
    /// wafer lifted off its pads (g + a_z + F / m <= 0) asks without bound: 1e9 m/s^2.
    pub fn equivalent(&self, a_xy: f64, a_z: f64, g: f64) -> f64 {
        let lift = match *self {
            Self::Pads { .. } => 0.0,
            Self::Vacuum { lift, .. } => lift,
            Self::Edge { .. } => return a_xy.abs(),
        };
        let (rest, now) = (g + lift, g + a_z + lift);
        if a_xy == 0.0 || rest <= 0.0 {
            a_xy.abs()
        } else if now <= 0.0 {
            1e9
        } else {
            a_xy.abs() * rest / now
        }
    }
}

/// The Z stroke: the column on a ball screw (in N and m) behind its coupling. Its state is [position, speed, brake].
// shortcut: rigid, as the template's stroke: the screw's and the coupling's stiffness are left out (the column on its
// screw is a mode of several hundred Hz); give it states as the arm's joints have once that mode matters.
struct Lift {
    joint: RotaryJoint,
    /// What it lifts besides the wafer (column, motors 1 and 2, links, effector), kg; the wafer while it is held.
    load: f64,
    payload: f64,
    g: f64,
    screw: BallScrew,
}

impl Lift {
    fn mass(&self, attached: bool) -> f64 {
        self.load + if attached { self.payload } else { 0.0 }
    }

    /// The weight along the stroke (negative: down).
    fn weight(&self, attached: bool) -> f64 {
        -self.g * self.mass(attached)
    }

    fn ratio(&self) -> f64 {
        self.joint.ratio()
    }

    /// The acceleration, and the force into the screw after the rotor's own acceleration and out of it:
    /// (m + eta i^2 J_rotor) q'' = eta drive + weight + stops and friction.
    fn solve(&self, x: &[f64], torque: f64, engaged: bool, attached: bool) -> (f64, f64, f64) {
        let (j, i) = (&self.joint, self.ratio());
        let drive = j.rigid_drive(torque, x[0] * i, x[1] * i, x[2], engaged).0;
        let rest = self.weight(attached) + j.output_torque(x[0], x[1]);
        let (m, r) = (self.mass(attached), j.reflected_inertia());
        let first = (drive + rest) / (m + r);
        let loss = j.gearbox.loss_factor(drive - r * first, x[1]);
        let acc = (loss * drive + rest) / (m + loss * r);
        let into = drive - r * acc;
        (acc, into, loss * into)
    }

    /// Kinetic, potential and elastic (stops) energy.
    fn energy(&self, x: &[f64], attached: bool) -> [f64; 3] {
        let kinetic = 0.5 * (self.mass(attached) + self.joint.reflected_inertia()) * x[1] * x[1];
        [kinetic, -self.weight(attached) * x[0], self.joint.stop_energy(x[0])]
    }

    /// Powers lost in its guides, its motor's friction with the brake, its coupling and screw, its stops and what the
    /// ripple gives off; and the coupling's loss as a torque at the rotor.
    fn losses(&self, x: &[f64], torque: f64, engaged: bool, attached: bool) -> ([f64; 5], f64) {
        let i = self.ratio();
        let [friction, stop, shaft, drag] = self.joint.losses(x[0], x[1], x[1] * i, x[2], engaged);
        let (_, into, out) = self.solve(x, torque, engaged, attached);
        let other = (torque - self.joint.ripple.apply(torque, x[0] * i)) * x[1] * i;
        ([friction, shaft, (into - out) * x[1] + drag, stop, other], (into - out) / i)
    }

    /// The motor torque's parts (`TORQUE_PARTS`).
    fn torque_parts(&self, x: &[f64], torque: f64, engaged: bool, attached: bool) -> [f64; 4] {
        let (j, i) = (&self.joint, self.ratio());
        let (acc, ..) = self.solve(x, torque, engaged, attached);
        let (_, gear_at_rotor) = self.losses(x, torque, engaged, attached);
        let held = j.brake.torque(engaged, x[1] * i, x[2]).0;
        let parts = [
            (self.mass(attached) + j.reflected_inertia()) * acc / i,
            -self.weight(attached) / i,
            j.shaft.torque(x[1] * i) + held + gear_at_rotor + j.friction.torque(x[1]) / i,
        ];
        [parts[0], parts[1], parts[2], torque - parts.iter().sum::<f64>()]
    }

    /// Rigid, loss-free: (m q'' - weight) / i + J_rotor i q''.
    fn feedforward(&self, acc: f64, attached: bool) -> f64 {
        (self.mass(attached) * acc - self.weight(attached)) / self.ratio()
            + self.joint.rotor_inertia * self.ratio() * acc
    }
}

/// The column's height in the scene below the arm's plane, m (view only).
const COLUMN_DROP: f64 = 0.35;

pub struct WaferHandler {
    arm: Arm,
    /// Motor 1 turns the column (theta), motor 2 the upper arm on it (phi).
    joints: [RotaryJoint; 2],
    thermal: [Option<ThermalFeedback>; 2],
    /// The elbow's band and the wrist's.
    bands: [Band; 2],
    /// effects.joint_elasticity and effects.band_elasticity.
    elastic: bool,
    bands_elastic: bool,
    lift: Lift,
    /// What the Z stroke lifts besides the arm (column, motors 1 and 2), for the frame's momentum.
    carriage: f64,
    required_life: Option<f64>,
    g: f64,
    hold: Hold,
    slip_limit: f64,
    direction_limit: f64,
    band_limit: f64,
    payload_attached: bool,
    brake_engaged: bool,
    dwelling: bool,
    max_condition: f64,
    axes: Vec<AxisInfo>,
    motors: Vec<MotorInfo>,
    rest_slack: f64,
}

/// The losses of one moment: per joint its friction, its motor's friction with the brake, its gearbox; the stops, what
/// the ripple gives off, the elbow's and the wrist's band; and per joint the gearbox loss as a torque at the rotor.
struct Losses {
    powers: [f64; 10],
    gear_at_rotor: V2,
}

impl WaferHandler {
    pub fn new(cfg: &Config) -> Result<Self, ConfigError> {
        let link = cfg.positive("geometry.link_m")?;
        let effector = cfg.positive("geometry.effector_m")?;
        let body = |name: &str| -> Result<Body, ConfigError> {
            Ok(Body {
                mass: cfg.positive(&format!("mass.{name}_kg"))?,
                inertia: cfg.non_negative(&format!("mass.{name}_inertia_kgm2"))?,
                centre: cfg.non_negative(&format!("mass.{name}_centre_m"))?,
            })
        };
        let payload = cfg.bool("effects.payload")?;
        let wafer_kg = cfg.positive("mass.payload_kg")?;
        let wafer = if payload {
            Body { mass: wafer_kg, inertia: cfg.non_negative("mass.payload_inertia_kgm2")?, centre: effector }
        } else {
            Body { mass: 0.0, inertia: 0.0, centre: effector }
        };
        let arm = Arm {
            link,
            effector,
            column_inertia: cfg.positive("mass.column_inertia_kgm2")?,
            bodies: [body("upper_arm")?, body("forearm")?, body("effector")?],
            wafer,
        };
        let g = sim_components::gripper::gravity(cfg)?;
        let theta = RotaryJoint::from_config(cfg)?;
        let own = motor_config(cfg, "2")?;
        let phi = RotaryJoint::from_config(&own)?;
        if !(phi.range.0 > 0.0 && phi.range.1 < std::f64::consts::PI) {
            return Err(ConfigError::Invalid("joint.2.min_angle_deg".into()));
        }
        let (r_min, r_max) = (cfg.f64("workspace.min_reach_m")?, cfg.positive("workspace.max_reach_m")?);
        if !(r_min > 0.0 && r_min < r_max) {
            return Err(ConfigError::Invalid("workspace.min_reach_m".into()));
        }
        let span = |id: &str| -> Result<(f64, f64), ConfigError> {
            let (min, max) =
                (cfg.f64(&format!("axis.{id}.min_angle_deg"))?, cfg.f64(&format!("axis.{id}.max_angle_deg"))?);
            if min >= max {
                return Err(ConfigError::Invalid(format!("axis.{id}.max_angle_deg")));
            }
            Ok((min.to_radians(), max.to_radians()))
        };
        let mut axes = vec![
            Self::axis_info(cfg, "r", "axis-reach", "m", (r_min, r_max))?,
            Self::axis_info(cfg, "theta", "axis-turn", "rad", span("theta")?)?,
        ];
        let carriage = cfg.non_negative("z_axis.carriage_kg")?;
        let z = motor_config(cfg, "3")?;
        let mut joint = RotaryJoint::linear_from_config(&z)?;
        let screw = BallScrew::from_config(&z)?;
        if screw.length < joint.range.1 {
            return Err(ConfigError::Invalid("screw.length_m".into()));
        }
        screw.converter(screw.softest_position()).apply(&z, &mut joint)?;
        // The nut's preload drags like friction in the guides, as a force at the nut.
        if z.bool("effects.joint_friction")? {
            joint.friction.coulomb += screw.preload_torque() * screw.ratio();
        }
        axes.push(Self::axis_info(cfg, "z", "axis-lift", "m", joint.range)?);
        let lift = Lift { joint, load: carriage + arm.mass(false), payload: arm.wafer.mass, g, screw };
        let hold = Hold::from_config(cfg, wafer_kg)?;
        let factor = cfg.positive("ratings.wafer_slip_safety_factor")?;
        let mut handler = Self {
            arm,
            joints: [theta, phi],
            thermal: [ThermalFeedback::from_config(cfg)?, ThermalFeedback::from_config(&own)?],
            bands: [Band::from_config(cfg, "elbow_band")?, Band::from_config(cfg, "wrist_band")?],
            elastic: cfg.bool("effects.joint_elasticity")?,
            bands_elastic: cfg.bool("effects.band_elasticity")?,
            lift,
            carriage,
            required_life: cfg.f64("ratings.required_life_h").ok(),
            g,
            hold,
            slip_limit: hold.at_rest(g) / factor,
            direction_limit: cfg.positive("ratings.direction_tolerance_rad")?,
            band_limit: cfg.positive("ratings.band_max_pretension_share")?,
            payload_attached: true,
            brake_engaged: false,
            dwelling: false,
            max_condition: cfg.positive("workspace.max_condition_number")?,
            axes,
            motors: vec![
                MotorInfo { id: "1", key: "motor-turn" },
                MotorInfo { id: "2", key: "motor-reach" },
                MotorInfo { id: "3", key: "motor-lift" },
            ],
            rest_slack: 0.0,
        };
        // The reach's range within the upper arm's stops and the condition number.
        for r in [r_min, r_max] {
            if handler.unreachable(&[r, 0.0, handler.axes[2].min], 0.0).is_some() {
                return Err(ConfigError::Invalid("workspace.max_reach_m".into()));
            }
        }
        handler.rest_slack = handler.rest_sag();
        Ok(handler)
    }

    /// An axis from `[axis.<id>]`, its limits in m or rad.
    fn axis_info(
        cfg: &Config,
        id: &'static str,
        key: &'static str,
        unit: &'static str,
        (min, max): (f64, f64),
    ) -> Result<AxisInfo, ConfigError> {
        Ok(AxisInfo {
            id,
            key,
            unit,
            min,
            max,
            max_vel: cfg.positive(&format!("axis.{id}.max_velocity_{unit}_s"))?,
            max_acc: cfg.positive(&format!("axis.{id}.max_acceleration_{unit}_s2"))?,
            max_jerk: cfg.positive(&format!("axis.{id}.max_jerk_{unit}_s3"))?,
        })
    }

    /// How far the motors' pose can be from the wafer's at rest: each gearbox at either side of its backlash (nothing
    /// loads them at rest, gravity being out of the plane): the column's half play in rad on theta, the upper arm's
    /// times the most dr/dphi, 2 L, on r; plus a tenth. At least the default 0.1 mm; that alone in the rigid model.
    fn rest_sag(&self) -> f64 {
        if !self.elastic {
            return 1e-4;
        }
        let [theta, phi] = [&self.joints[0], &self.joints[1]].map(|j| j.gearbox.half_backlash);
        (1.1 * theta.max(2.0 * self.arm.link * phi)).max(1e-4)
    }

    pub fn arm(&self) -> &Arm {
        &self.arm
    }

    pub fn bands(&self) -> &[Band; 2] {
        &self.bands
    }

    /// The highest in-plane acceleration "wafer-slip" allows, m/s^2.
    pub fn slip_limit(&self) -> f64 {
        self.slip_limit
    }

    /// The number of the arm's coordinates in the state: theta and phi, and beta and gamma with elastic bands.
    fn nq(&self) -> usize {
        if self.bands_elastic { 4 } else { 2 }
    }

    fn planar_len(&self) -> usize {
        2 * self.nq() + if self.elastic { 6 } else { 2 }
    }

    /// The arm's coordinates and rates of a state, beta and gamma zero where the bands are rigid.
    fn coordinates(&self, s: &[f64]) -> (Q, Q) {
        let n = self.nq();
        let (mut q, mut qd) = ([0.0; 4], [0.0; 4]);
        q[..n].copy_from_slice(&s[..n]);
        qd[..n].copy_from_slice(&s[n..2 * n]);
        (q, qd)
    }

    /// Rotor angles and speeds of a state.
    fn rotors(&self, s: &[f64]) -> (V2, V2) {
        let n = self.nq();
        if self.elastic {
            ([s[2 * n], s[2 * n + 1]], [s[2 * n + 2], s[2 * n + 3]])
        } else {
            let i = [self.joints[0].ratio(), self.joints[1].ratio()];
            ([s[0] * i[0], s[1] * i[1]], [s[n] * i[0], s[n + 1] * i[1]])
        }
    }

    fn brakes(&self, s: &[f64]) -> V2 {
        let at = self.planar_len() - 2;
        [s[at], s[at + 1]]
    }

    /// The Z stroke's state: [position, speed, brake].
    fn lift_state<'a>(&self, s: &'a [f64]) -> &'a [f64] {
        let at = self.planar_len();
        &s[at..at + 3]
    }

    /// The bands' stretches and their rates: s1 = -r1 beta, s2 = r2 (beta / 2 - gamma).
    fn stretches(&self, q: &Q, qd: &Q) -> [V2; 2] {
        let [r1, r2] = [self.bands[0].radius, self.bands[1].radius];
        [[-r1 * q[2], -r1 * qd[2]], [r2 * (0.5 * q[2] - q[3]), r2 * (0.5 * qd[2] - qd[3])]]
    }

    /// The bands' generalised forces on beta and gamma (zero on theta and phi).
    fn band_forces(&self, q: &Q, qd: &Q) -> Q {
        if !self.bands_elastic {
            return [0.0; 4];
        }
        let [[s1, v1], [s2, v2]] = self.stretches(q, qd);
        let (d1, d2) = (self.bands[0].difference(s1, v1), self.bands[1].difference(s2, v2));
        [0.0, 0.0, self.bands[0].radius * d1 - 0.5 * self.bands[1].radius * d2, self.bands[1].radius * d2]
    }

    /// The bands' tension swings over their pretension (k s / T0, signed): `from_stretch`, from their stretch (elastic
    /// bands), else from the forces the rigid model asks of them at the arm's acceleration `qdd`: dT2 = G_gamma / r2,
    /// dT1 = (G_beta + G_gamma / 2) / r1 of G = M q'' + h over all four coordinates, half of each per span.
    fn band_shares(&self, q: &Q, qd: &Q, qdd: &Q, from_stretch: bool) -> V2 {
        if from_stretch {
            let [[s1, _], [s2, _]] = self.stretches(q, qd);
            return [0, 1].map(|k| self.bands[k].stiffness * [s1, s2][k] / self.bands[k].pretension);
        }
        let (m, h) = self.arm.dynamics(q, qd, self.payload_attached);
        let g: Q = std::array::from_fn(|r| dot(&m[r], qdd) + h[r]);
        let [b1, b2] = self.bands;
        let (d2, d1) = (g[3] / b2.radius, (g[2] + 0.5 * g[3]) / b1.radius);
        [0.5 * d1 / b1.pretension, 0.5 * d2 / b2.pretension]
    }

    /// The tool's (r, theta) and their rates at q and q': in the rigid-band model r(phi) and the column's angle; with
    /// elastic bands from the wafer centre C, r signed along the column's direction and theta its polar angle, so that C
    /// = r u(theta), and r' = u . C', theta' = n . C' / r.
    fn plane_tool(&self, q: &Q, qd: &Q) -> (V2, V2) {
        if !self.bands_elastic {
            let [r, dr, _] = self.arm.reach(q[1]);
            return ([r, q[0]], [dr * qd[1], qd[0]]);
        }
        let c = self.arm.wafer_centre(q);
        let (jac, _) = self.arm.point_motion(q, qd, 2, self.arm.effector);
        let v = [dot(&jac[0], qd), dot(&jac[1], qd)];
        let (s, k) = q[0].sin_cos();
        let local = [k * c[0] + s * c[1], -s * c[0] + k * c[1]];
        let sign = if local[0] >= 0.0 { 1.0 } else { -1.0 };
        let r = sign * local[0].hypot(local[1]);
        let theta = q[0] + (sign * local[1]).atan2(sign * local[0]);
        let (u, n) = (unit(theta), normal(theta));
        ([r, theta], [dot(&u, &v), dot(&n, &v) / r])
    }

    /// The effector's direction less the wafer centre's polar angle: the turn of the effector out of the radial line
    /// through its wafer, which the bands' stretch gives; zero with rigid bands.
    fn direction_error(&self, q: &Q, qd: &Q) -> f64 {
        if !self.bands_elastic {
            return 0.0;
        }
        Arm::angles(q)[2] - self.plane_tool(q, qd).0[1]
    }

    /// The arm's coordinates, rates and accelerations for a tool pose, speed and acceleration in the rigid model: theta
    /// = theta, phi = acos((r - e) / 2 L), phi' = r' / r_phi, phi'' = (r'' - r_phiphi phi'^2) / r_phi.
    fn joint_motion(&self, pos: &[f64], vel: &[f64], acc: &[f64]) -> (Q, Q, Q) {
        let phi = self.arm.phi(pos[0]);
        let [_, d1, d2] = self.arm.reach(phi);
        let w = vel[0] / d1;
        ([pos[1], phi, 0.0, 0.0], [vel[1], w, 0.0, 0.0], [acc[1], (acc[0] - d2 * w * w) / d1, 0.0, 0.0])
    }

    /// Rigid gearboxes: the arm's acceleration, and per joint the torque into its gearbox (after the rotor's own
    /// acceleration) and out of it.
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> (Q, V2, V2) {
        let n = self.nq();
        let (q, qd) = self.coordinates(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let (mass, h) = self.arm.dynamics(&q, &qd, self.payload_attached);
        let bands = self.band_forces(&q, &qd);
        let drive: V2 = std::array::from_fn(|k| {
            self.joints[k].rigid_drive(torque[k], rotor[k], rotor_speed[k], brakes[k], self.brake_engaged).0
        });
        let rest: Q = std::array::from_fn(|k| {
            bands[k] - h[k] + if k < 2 { self.joints[k].output_torque(q[k], qd[k]) } else { 0.0 }
        });
        let at = |loss: V2| -> Q {
            let mut m = mass;
            let mut rhs = rest;
            for k in 0..2 {
                m[k][k] += loss[k] * self.joints[k].reflected_inertia();
                rhs[k] += loss[k] * drive[k];
            }
            solve(&m, &rhs, n).unwrap_or([f64::NAN; 4])
        };
        let first = at([1.0; 2]);
        let loss: V2 = std::array::from_fn(|k| {
            self.joints[k].gearbox.loss_factor(drive[k] - self.joints[k].reflected_inertia() * first[k], qd[k])
        });
        let acc = at(loss);
        let into: V2 = std::array::from_fn(|k| drive[k] - self.joints[k].reflected_inertia() * acc[k]);
        (acc, into, std::array::from_fn(|k| loss[k] * into[k]))
    }

    /// The arm's accelerations in either model.
    fn arm_acceleration(&self, s: &[f64], torque: &[f64]) -> Q {
        if self.elastic {
            let mut d = vec![0.0; self.state_len()];
            self.derivatives(s, torque, &mut d);
            let n = self.nq();
            std::array::from_fn(|k| if k < n { d[n + k] } else { 0.0 })
        } else {
            self.rigid_solve(s, torque).0
        }
    }

    /// The torque each gearbox passes to its link.
    fn gear_output(&self, s: &[f64], torque: &[f64]) -> V2 {
        if self.elastic {
            let n = self.nq();
            std::array::from_fn(|k| {
                self.joints[k].gearbox.torque(s[2 * n + k], s[2 * n + 2 + k], s[k], s[n + k]).output
            })
        } else {
            self.rigid_solve(s, torque).2
        }
    }

    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        let n = self.nq();
        let (q, qd) = self.coordinates(s);
        let (mass, _) = self.arm.dynamics(&q, &qd, self.payload_attached);
        let mut kinetic = 0.5 * (0..n).map(|r| qd[r] * dot(&mass[r][..n], &qd[..n])).sum::<f64>();
        let mut elastic: f64 = (0..2).map(|k| self.joints[k].stop_energy(q[k])).sum();
        if self.bands_elastic {
            let [[s1, _], [s2, _]] = self.stretches(&q, &qd);
            elastic += self.bands[0].energy(s1) + self.bands[1].energy(s2);
        }
        if self.elastic {
            for k in 0..2 {
                kinetic += 0.5 * self.joints[k].rotor_inertia * s[2 * n + 2 + k].powi(2);
                elastic += self.joints[k].gearbox.energy(s[2 * n + k], q[k]);
            }
        } else {
            kinetic += 0.5 * (0..2).map(|k| self.joints[k].reflected_inertia() * qd[k] * qd[k]).sum::<f64>();
        }
        // Nothing in the plane rises or falls: the potential energy is the Z stroke's alone.
        let own = self.lift.energy(self.lift_state(s), self.payload_attached);
        [kinetic + own[0], own[1], elastic + own[2]]
    }

    fn losses(&self, s: &[f64], torque: &[f64]) -> Losses {
        let n = self.nq();
        let (q, qd) = self.coordinates(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let mut powers = [0.0; 10];
        let mut gear_at_rotor = [0.0; 2];
        let rigid = (!self.elastic).then(|| self.rigid_solve(s, torque));
        let (mut stops, mut other) = (0.0, 0.0);
        for k in 0..2 {
            let joint = &self.joints[k];
            let [friction, stop, shaft, drag] =
                joint.losses(q[k], qd[k], rotor_speed[k], brakes[k], self.brake_engaged);
            let gear = match &rigid {
                Some((_, into, out)) => {
                    gear_at_rotor[k] = (into[k] - out[k]) / joint.ratio();
                    (into[k] - out[k]) * qd[k]
                }
                None => {
                    let (r, w) = (s[2 * n + k], s[2 * n + 2 + k]);
                    let g = joint.gearbox.torque(r, w, q[k], qd[k]);
                    gear_at_rotor[k] = g.rotor - g.output / joint.ratio();
                    joint.gearbox.loss_power(r, w, q[k], qd[k])
                }
            };
            powers[3 * k..3 * k + 3].copy_from_slice(&[friction, shaft, gear + drag]);
            stops += stop;
            other += (torque[k] - joint.ripple.apply(torque[k], rotor[k])) * rotor_speed[k];
        }
        let ([.., stop, ripple], _) =
            self.lift.losses(self.lift_state(s), torque[2], self.brake_engaged, self.payload_attached);
        powers[6] = stops + stop;
        powers[7] = other + ripple;
        if self.bands_elastic {
            let [[s1, v1], [s2, v2]] = self.stretches(&q, &qd);
            powers[8] = self.bands[0].loss(s1, v1);
            powers[9] = self.bands[1].loss(s2, v2);
        }
        Losses { powers, gear_at_rotor }
    }

    /// The wafer centre's acceleration in the plane at a state's arm accelerations `qdd`.
    fn wafer_acceleration(&self, q: &Q, qd: &Q, qdd: &Q) -> V2 {
        let (jac, bias) = self.arm.point_motion(q, qd, 2, self.arm.effector);
        [dot(&jac[0], qdd) + bias[0], dot(&jac[1], qdd) + bias[1]]
    }

    /// The torque breakdown, loss powers, energies of the effect views.
    fn view_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (q, qd) = self.coordinates(s);
        let (_, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let (mass, h) = self.arm.dynamics(&q, &qd, self.payload_attached);
        let acc = self.arm_acceleration(s, torque);
        let losses = self.losses(s, torque);
        for k in 0..2 {
            let joint = &self.joints[k];
            let i = joint.ratio();
            let held = joint.brake.torque(self.brake_engaged, rotor_speed[k], brakes[k]).0;
            let parts = [
                (dot(&mass[k], &acc) + h[k] + joint.reflected_inertia() * acc[k]) / i,
                // Gravity acts out of the plane, on the joints' bearings.
                0.0,
                joint.shaft.torque(rotor_speed[k]) + held + losses.gear_at_rotor[k] + joint.friction.torque(qd[k]) / i,
            ];
            out.extend(parts);
            out.push(torque[k] - parts.iter().sum::<f64>());
        }
        out.extend(losses.powers);
        out.extend(self.energy_parts(s));
        let (state, at) = (self.lift_state(s), (torque[2], self.brake_engaged, self.payload_attached));
        out.extend(self.lift.torque_parts(state, at.0, at.1, at.2));
        out.extend(&self.lift.losses(state, at.0, at.1, at.2).0[..3]);
    }

    /// The torque parts and loss powers of motor `motor`.
    fn motor_infos(motor: &MotorInfo, joint: &str, coupled: bool) -> (Vec<SignalInfo>, Vec<SignalInfo>) {
        let parts = TORQUE_PARTS.map(|part| {
            SignalInfo::new(
                format!("motor.{}.torque_{part}", motor.id),
                motor.key,
                &format!("quantity-torque-{part}"),
                "N m",
            )
        });
        let gear = if coupled { "quantity-coupling-loss-power" } else { "quantity-gearbox-loss-power" };
        let losses = vec![
            SignalInfo::new(format!("{LOSS_PREFIX}joint_{}", motor.id), joint, "quantity-friction-power", "W"),
            SignalInfo::new(
                format!("{LOSS_PREFIX}shaft_{}", motor.id),
                motor.key,
                "quantity-shaft-friction-power",
                "W",
            ),
            SignalInfo::new(format!("{LOSS_PREFIX}gearbox_{}", motor.id), motor.key, gear, "W"),
        ];
        (parts.to_vec(), losses)
    }

    fn effect_infos(&self, out: &mut Vec<SignalInfo>) {
        let arms: Vec<_> = (0..2)
            .map(|k| {
                Self::motor_infos(&self.motors[k], ["column", "upper-arm"][k], self.joints[k].gearbox.is_coupling())
            })
            .collect();
        out.extend(arms.iter().flat_map(|(parts, _)| parts.clone()));
        out.extend(arms.into_iter().flat_map(|(_, losses)| losses));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}hard_stops"), "group-joint", "quantity-damping-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}other"), "loss-other", "quantity-power-given-off", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}band_elbow"), "elbow-band", "quantity-band-damping-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}band_wrist"), "wrist-band", "quantity-band-damping-power", "W"));
        out.extend(ENERGY_PARTS.map(|part| {
            SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")
        }));
        let (parts, losses) = Self::motor_infos(&self.motors[2], self.axes[2].key, self.lift.joint.coupled);
        out.extend(parts);
        out.extend(losses);
    }

    /// The scene's parts that move with the wafer: the effector and the wafer at the arm's coordinates.
    fn on_effector(&self, q: &Q, z: f64) -> Vec<Part> {
        let wrist = self.arm.point(q, 1, self.arm.link);
        let centre = self.arm.wafer_centre(q);
        let mut parts = vec![Part::new(
            "effector",
            Shape::Segment { to: [centre[0], centre[1], z], width: 0.06 },
            [wrist[0], wrist[1], z],
            [0.85, 0.85, 0.8],
        )];
        if self.payload_attached && self.arm.wafer.mass > 0.0 {
            parts.push(Part::new(
                "wafer",
                Shape::Box { size: [0.3, 0.3, 0.002] },
                [centre[0], centre[1], z + 0.005],
                [0.35, 0.4, 0.6],
            ));
        }
        parts
    }
}

impl Machine for WaferHandler {
    /// In the machine's frame (x, y in the plane, z up, the column's axis at the origin): each link, the effector and
    /// the wafer at their centres moving in the plane and up with the stroke, turning at their links' rates; the column
    /// and the rotors spinning about z; the rest of what the stroke lifts at the axis.
    fn frame_momentum(&self, s: &[f64]) -> Option<sim_core::FrameMomentum> {
        use sim_components::base::{add_point, add_spin};
        let mut f = sim_core::FrameMomentum { gravity: [0.0, 0.0, -self.g], ..Default::default() };
        let (q, qd) = self.coordinates(s);
        let [z, vz, _] = self.lift_state(s)[..] else { unreachable!() };
        for (k, body) in self.arm.moving(self.payload_attached) {
            let p = self.arm.point(&q, k, body.centre);
            let (jac, _) = self.arm.point_motion(&q, &qd, k, body.centre);
            let v = [dot(&jac[0], &qd), dot(&jac[1], &qd)];
            add_point(&mut f, body.mass, [p[0], p[1], z], [v[0], v[1], vz]);
            add_spin(&mut f, [0.0, 0.0, 1.0], body.inertia, dot(&LINK_ANGLES[k], &qd));
        }
        add_point(&mut f, self.carriage, [0.0, 0.0, z], [0.0, 0.0, vz]);
        add_spin(&mut f, [0.0, 0.0, 1.0], self.arm.column_inertia, qd[0]);
        let rotor_speed = self.rotors(s).1;
        for k in 0..2 {
            add_spin(&mut f, [0.0, 0.0, 1.0], self.joints[k].rotor_inertia, rotor_speed[k]);
        }
        add_spin(&mut f, [0.0, 0.0, 1.0], self.lift.joint.rotor_inertia, self.lift.ratio() * vz);
        Some(f)
    }

    /// The wafer centre at (r cos theta, r sin theta, z) moves with the base by its translation and tilt (as the
    /// default moves a point), told along and across the radial line: r by the move along u(theta), theta by the move
    /// across it over r, z by the move up.
    fn base_shift(&self, tool_pose: &[f64], pivot: [f64; 3], shift: &[f64; 6], out: &mut [f64]) {
        let (r, theta) = (tool_pose[0], tool_pose[1]);
        let (u, n) = (unit(theta), normal(theta));
        let p = [r * u[0] - pivot[0], r * u[1] - pivot[1], tool_pose[2] - pivot[2]];
        let t = [shift[3], shift[4], shift[5]];
        let moved = [t[1] * p[2] - t[2] * p[1], t[2] * p[0] - t[0] * p[2], t[0] * p[1] - t[1] * p[0]];
        let d: [f64; 3] = std::array::from_fn(|k| shift[k] + moved[k]);
        let across = d[0] * n[0] + d[1] * n[1];
        out[0] = d[0] * u[0] + d[1] * u[1];
        out[1] = if r.abs() > 1e-9 { across / r } else { t[2] };
        out[2] = d[2];
    }

    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    /// NaN where the reach is beyond the arm's: the controller never asks for one (see `unreachable`).
    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        motors[0] = self.joints[0].ratio() * tool[1];
        motors[1] = self.joints[1].ratio() * self.arm.phi(tool[0]);
        motors[2] = self.lift.ratio() * tool[2];
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        tool[0] = self.arm.reach(motors[1] / self.joints[1].ratio())[0];
        tool[1] = motors[0] / self.joints[0].ratio();
        tool[2] = motors[2] / self.lift.ratio();
    }

    fn state_len(&self) -> usize {
        self.planar_len() + 3
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let n = self.nq();
        let q = [tool[1], self.arm.phi(tool[0])];
        state.clear();
        state.extend(q);
        state.resize(2 * n, 0.0);
        let mut rotors = Vec::new();
        for k in 0..2 {
            let joint = &self.joints[k];
            // At rest the gearbox holds the link against its stops; gravity gives it nothing to hold, nor the bands.
            let out = -joint.output_torque(q[k], 0.0);
            let rotor = joint.ratio() * (q[k] + if self.elastic { joint.gearbox.static_twist(out) } else { 0.0 });
            rotors.push(rotor);
            torque[k] = joint.ripple.invert(out / joint.ratio(), rotor);
        }
        if self.elastic {
            state.extend(rotors);
            state.extend([0.0; 2]);
        }
        state.extend([0.0; 2]);
        // The stroke holds what it lifts.
        let (z, i) = (tool[2], self.lift.ratio());
        let out = -self.lift.weight(self.payload_attached) - self.lift.joint.output_torque(z, 0.0);
        torque[2] = self.lift.joint.ripple.invert(out / i, z * i);
        state.extend([z, 0.0, 0.0]);
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let n = self.nq();
        let at = self.planar_len();
        let state = self.lift_state(s);
        out[at] = state[1];
        out[at + 1] = self.lift.solve(state, torque[2], self.brake_engaged, self.payload_attached).0;
        out[at + 2] = self.lift.joint.brake.torque(self.brake_engaged, state[1] * self.lift.ratio(), state[2]).1;
        out[..n].copy_from_slice(&s[n..2 * n]);
        if !self.elastic {
            let (acc, ..) = self.rigid_solve(s, torque);
            let (_, rotor_speed) = self.rotors(s);
            let brakes = self.brakes(s);
            out[n..2 * n].copy_from_slice(&acc[..n]);
            for k in 0..2 {
                out[2 * n + k] = self.joints[k].brake.torque(self.brake_engaged, rotor_speed[k], brakes[k]).1;
            }
            return;
        }
        let (q, qd) = self.coordinates(s);
        let (mass, h) = self.arm.dynamics(&q, &qd, self.payload_attached);
        let mut rhs = self.band_forces(&q, &qd);
        for r in 0..4 {
            rhs[r] -= h[r];
        }
        for k in 0..2 {
            let joint = &self.joints[k];
            let (gear, rotor_acc, brake_rate) = joint.rotor(
                torque[k],
                s[2 * n + k],
                s[2 * n + 2 + k],
                q[k],
                qd[k],
                s[2 * n + 4 + k],
                self.brake_engaged,
            );
            rhs[k] += gear.output + joint.output_torque(q[k], qd[k]);
            out[2 * n + k] = s[2 * n + 2 + k];
            out[2 * n + 2 + k] = rotor_acc;
            out[2 * n + 4 + k] = brake_rate;
        }
        let acc = solve(&mass, &rhs, n).unwrap_or([f64::NAN; 4]);
        out[n..2 * n].copy_from_slice(&acc[..n]);
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (q, qd) = self.coordinates(s);
        let (p, v) = self.plane_tool(&q, &qd);
        pos[..2].copy_from_slice(&p);
        vel[..2].copy_from_slice(&v);
        (pos[2], vel[2]) = (self.lift_state(s)[0], self.lift_state(s)[1]);
    }
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        let n = self.nq();
        let d1 = self.arm.reach(s[1])[1];
        let w = [vel[1], vel[0] / d1];
        s[n..n + 2].copy_from_slice(&w);
        if self.elastic {
            for k in 0..2 {
                s[2 * n + 2 + k] = self.joints[k].ratio() * w[k];
            }
        }
        s[self.planar_len() + 1] = vel[2];
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (rotor, speed) = self.rotors(s);
        pos[..2].copy_from_slice(&rotor);
        vel[..2].copy_from_slice(&speed);
        let state = self.lift_state(s);
        (pos[2], vel[2]) = (state[0] * self.lift.ratio(), state[1] * self.lift.ratio());
    }

    /// Rigid, loss-free, at rest: M q'' at the joints over the ratio, plus the rotor's own J_rotor i q''; the stroke's
    /// (m q'' - weight) / i + J_rotor i q''.
    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        self.feedforward_torque_moving(pos, &[0.0; 3], acc, out);
    }

    /// With the speed: the joints' motion (`joint_motion`), M q'' + h at the joints over the ratio, plus the rotor's own
    /// J_rotor i q''.
    fn feedforward_torque_moving(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut [f64]) {
        let (q, qd, qdd) = self.joint_motion(pos, vel, acc);
        let (mass, h) = self.arm.dynamics(&q, &qd, self.payload_attached);
        for k in 0..2 {
            let joint = &self.joints[k];
            let at_joint = mass[k][0] * qdd[0] + mass[k][1] * qdd[1] + h[k];
            out[k] = at_joint / joint.ratio() + joint.rotor_inertia * joint.ratio() * qdd[k];
        }
        out[2] = self.lift.feedforward(acc[2], self.payload_attached);
    }

    /// The rigid model at q'' = 0: per joint the motor shaft's friction (with the gearbox's drag) at i q' and the joint's
    /// own friction (bearings, the Z guides and the nut's preload) through the gearbox: f(q') / (i loss).
    fn steady_friction(&self, pos: &[f64], vel: &[f64], out: &mut [f64]) {
        let (_, qd, _) = self.joint_motion(pos, vel, &[0.0; 3]);
        let joints = [(&self.joints[0], qd[0]), (&self.joints[1], qd[1]), (&self.lift.joint, vel[2])];
        for (k, (joint, v)) in joints.into_iter().enumerate() {
            let (i, friction) = (joint.ratio(), joint.friction.torque(v));
            out[k] = joint.shaft.torque(i * v) + friction / (i * joint.gearbox.loss_factor(friction, v));
        }
    }

    /// The rating checks' values in the rigid model at a tool pose, speed and acceleration (`rating_checks`' order):
    /// the screw's speed, force and d n (its life NaN); "wafer-slip" NaN: the plan's rows are linear in the acceleration
    /// and the speed squared along the path, and the size of the wafer's acceleration, a vector's length, is not (it
    /// is judged, and `sim-cli optimise` keeps it, the cycle's axis limits are sized for it); "effector-direction" NaN
    /// (the rigid model has none); the bands' tension swings (`band_shares`), linear in the acceleration.
    fn rigid_checks(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut Vec<f64>) {
        let force = self.lift.mass(self.payload_attached) * (acc[2] + self.g);
        let mut screw = Vec::new();
        self.lift.screw.signals(vel[2], force, &mut screw);
        for check in self.lift.screw.rating_checks("z", 2, self.required_life) {
            out.push(match check.check {
                "screw-speed" => screw[0],
                "screw-dn" => screw[1],
                "screw-buckling" | "screw-static" => force,
                _ => f64::NAN,
            });
        }
        let (q, qd, qdd) = self.joint_motion(pos, vel, acc);
        let shares = self.band_shares(&q, &qd, &qdd, false);
        out.extend([f64::NAN, f64::NAN, shares[0], shares[1]]);
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
    fn set_dwelling(&mut self, dwelling: bool) {
        self.dwelling = dwelling;
    }

    /// The joints' gearboxes' hysteresis states move on to where the step ended (`Gearbox::commit`).
    fn after_step(&mut self, s: &mut [f64], _dt: f64) {
        if self.elastic {
            let n = self.nq();
            for k in 0..2 {
                self.joints[k].gearbox.commit(s[2 * n + k], s[k]);
            }
        }
    }

    /// As `Machine::adopt_state`, but where the model keeps its states (a changed ratio, not rigid against elastic) the
    /// links, the bands' stretch and the stroke stay as they were, and each elastic gearbox keeps its twist and its
    /// rotor's speed at the link (scaled by the new over the old dr/dmotor): rebuilt at rest, the bands' lag during a
    /// move (0.14 mm at the wafer) was dropped and rang anew, 0.134 mm off a run left alone after a 1 % change of the
    /// ratio.
    fn adopt_state(&self, old: &dyn Machine, old_state: &[f64]) -> Vec<f64> {
        let (m, n) = (self.axes.len(), self.motors.len());
        let (mut pos, mut vel) = (vec![0.0; m], vec![0.0; m]);
        old.tool_state(old_state, &mut pos, &mut vel);
        if old_state.len() != self.state_len() {
            let mut state = Vec::new();
            self.equilibrium(&pos, &mut state, &mut vec![0.0; n]);
            self.set_tool_velocity(&mut state, &vel);
            return state;
        }
        let mut state = old_state.to_vec();
        if self.elastic {
            let (mut rotor, mut speed) = (vec![0.0; n], vec![0.0; n]);
            old.motor_state(old_state, &mut rotor, &mut speed);
            // Motor k against the tool axis it drives (motor 1 theta, axis 1; motor 2 r, axis 0), new and old.
            let at = |machine: &dyn Machine, shift: f64, axis: usize| {
                let mut p = pos.clone();
                p[axis] += shift;
                let mut out = vec![0.0; n];
                machine.inverse(&p, &mut out);
                out
            };
            let (h, nq) = (1e-6, self.nq());
            for (k, axis) in [(0, 1), (1, 0)] {
                let (before, after) = (at(old, 0.0, axis)[k], at(self, 0.0, axis)[k]);
                let scale =
                    (at(self, h, axis)[k] - at(self, -h, axis)[k]) / (at(old, h, axis)[k] - at(old, -h, axis)[k]);
                state[2 * nq + k] = after + scale * (rotor[k] - before);
                state[2 * nq + 2 + k] = scale * speed[k];
            }
        }
        state
    }

    /// The gearboxes, in the order of `thermal_parts`.
    fn set_part_temperatures(&mut self, rises: &[f64]) {
        let mut rises = rises.iter();
        for k in 0..2 {
            if self.joints[k].gearbox.is_coupling() {
                continue;
            }
            if let (Some(law), Some(rise)) = (&self.thermal[k], rises.next()) {
                self.joints[k].gearbox.efficiency = law.gearbox_efficiency(*rise);
            }
        }
    }

    /// The lowest modes at a tool pose, with the motors held where they are: the bands' (beta, gamma) and, with elastic
    /// gearboxes, the column's and the upper arm's on their gearboxes' stiffness; the rigid model's joints held fast.
    fn modes(&self, tool_pose: &[f64]) -> Vec<Mode> {
        let phi = self.arm.phi(tool_pose[0]);
        let q = [tool_pose[1], phi, 0.0, 0.0];
        let (mass, _) = self.arm.dynamics(&q, &[0.0; 4], self.payload_attached);
        let mut k = [[0.0; 4]; 4];
        if self.bands_elastic {
            let (k1, k2) = (self.bands[0].torsional_stiffness(), self.bands[1].torsional_stiffness());
            k[2][2] = k1 + 0.25 * k2;
            k[2][3] = -0.5 * k2;
            k[3][2] = -0.5 * k2;
            k[3][3] = k2;
        }
        // The coordinates that move: theta and phi only on elastic gearboxes.
        let free: Vec<usize> = (0..4).filter(|&c| if c < 2 { self.elastic } else { self.bands_elastic }).collect();
        if free.is_empty() {
            return Vec::new();
        }
        for c in 0..2 {
            k[c][c] = self.joints[c].gearbox.stiffness;
        }
        let (mut m, mut kk) = ([[0.0; 4]; 4], [[0.0; 4]; 4]);
        for (a, &i) in free.iter().enumerate() {
            for (b, &j) in free.iter().enumerate() {
                m[a][b] = mass[i][j];
                kk[a][b] = k[i][j];
            }
        }
        generalised_eigenvalues(&m, &kk, free.len()).map_or(Vec::new(), |values| {
            values
                .into_iter()
                .filter(|v| *v > 0.0)
                .map(|v| Mode { hz: v.sqrt() / std::f64::consts::TAU, damping: 0.0 })
                .collect()
        })
    }

    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        self.modes(tool_pose).first().map(|m| m.hz)
    }

    /// How far the reach's motor is from the singular full extension: 1 / sin(phi), the most dr/dphi over this one (1
    /// with the wrist over the column, without bound at full extension); infinite out of reach.
    fn condition(&self, tool_pose: &[f64]) -> Option<f64> {
        let phi = self.arm.phi(tool_pose[0]);
        Some(if phi.is_finite() { 1.0 / phi.sin().abs() } else { f64::INFINITY })
    }

    /// What the gearboxes can sag at rest (`rest_sag`).
    fn setpoint_slack(&self) -> f64 {
        self.rest_slack
    }

    fn unreachable(&self, tool: &[f64], _slack: f64) -> Option<&'static str> {
        if tool.len() != self.axes.len() {
            return Some("error-target-out-of-range");
        }
        let phi = self.arm.phi(tool[0]);
        if !phi.is_finite() {
            return Some("error-handler-out-of-reach");
        }
        let (min, max) = self.joints[1].range;
        let (tmin, tmax) = self.joints[0].range;
        if phi < min || phi > max || tool[1] < tmin || tool[1] > tmax {
            return Some("error-handler-joint-stop");
        }
        match self.condition(tool) {
            Some(c) if c <= self.max_condition => None,
            _ => Some("error-handler-singularity"),
        }
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out = Vec::new();
        for (k, (owner, quantity)) in
            [("column", "quantity-column-angle"), ("upper-arm", "quantity-shoulder-angle")].into_iter().enumerate()
        {
            let motor = &self.motors[k];
            out.push(SignalInfo::new(format!("arm.{}.angle", motor.id), owner, quantity, "rad"));
            out.push(SignalInfo::new(
                format!("gearbox.{}.output_torque", motor.id),
                motor.key,
                "quantity-gearbox-torque",
                "N m",
            ));
        }
        out.push(SignalInfo::new("kinematics.condition", "group-kinematics", "quantity-condition-number", ""));
        out.push(SignalInfo::new("effector.direction_error", "effector", "quantity-direction-error", "rad"));
        out.push(SignalInfo::new(
            "effector.direction_error_at_station",
            "effector",
            "quantity-direction-error-at-station",
            "rad",
        ));
        out.push(SignalInfo::new("wafer.acceleration", "wafer", "quantity-wafer-acceleration", "m/s^2"));
        out.push(SignalInfo::new("band.elbow.pretension_share", "elbow-band", "quantity-band-share", ""));
        out.push(SignalInfo::new("band.wrist.pretension_share", "wrist-band", "quantity-band-share", ""));
        let axis = &self.axes[2];
        out.push(SignalInfo::new(format!("joint.{}.force", axis.id), axis.key, "quantity-joint-force", "N"));
        out.extend(self.lift.screw.signal_infos(axis.id, axis.key));
        self.effect_infos(&mut out);
        out
    }

    /// The stroke's ball screw (speed, d n, buckling, static load, life); the wafer's hold ("wafer-slip", against the
    /// hold at rest over `ratings.wafer_slip_safety_factor`); the effector's direction at a station
    /// ("effector-direction", against `ratings.direction_tolerance_rad`); each band's tension swing against
    /// `ratings.band_max_pretension_share` of its pretension ("elbow-band-slack", "wrist-band-slack").
    fn rating_checks(&self) -> Vec<RatingCheck> {
        let mut out = self.lift.screw.rating_checks("z", 2, self.required_life);
        let own = |check: &'static str, signal: &str, limit: f64| RatingCheck {
            check,
            signal: signal.into(),
            motor: None,
            limit,
            life: false,
        };
        out.push(own("wafer-slip", "wafer.acceleration", self.slip_limit));
        out.push(own("effector-direction", "effector.direction_error_at_station", self.direction_limit));
        out.push(own("elbow-band-slack", "band.elbow.pretension_share", self.band_limit));
        out.push(own("wrist-band-slack", "band.wrist.pretension_share", self.band_limit));
        out
    }

    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (q, qd) = self.coordinates(s);
        let gear = self.gear_output(s, torque);
        for k in 0..2 {
            out.extend([q[k], gear[k]]);
        }
        out.push(self.condition(&[self.arm.reach(q[1])[0]]).unwrap_or(f64::NAN));
        let direction = self.direction_error(&q, &qd);
        out.extend([direction, if self.dwelling { direction } else { 0.0 }]);
        let qdd = self.arm_acceleration(s, torque);
        let state = self.lift_state(s);
        let (a_z, _, force) = self.lift.solve(state, torque[2], self.brake_engaged, self.payload_attached);
        let a = self.wafer_acceleration(&q, &qd, &qdd);
        let held = self.payload_attached && self.arm.wafer.mass > 0.0;
        out.push(if held { self.hold.equivalent(a[0].hypot(a[1]), a_z, self.g) } else { 0.0 });
        out.extend(self.band_shares(&q, &qd, &qdd, self.bands_elastic));
        out.push(force);
        self.lift.screw.signals(state[1], force, out);
    }
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.view_signals(s, torque, out);
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(s, torque).powers);
        out.extend(&self.lift.losses(self.lift_state(s), torque[2], self.brake_engaged, self.payload_attached).0[..3]);
    }

    /// The two gearboxes, where there are gearboxes (not on a direct drive); the stroke's coupling warms nothing.
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        let ids = [("gearbox_1", "motor-turn"), ("gearbox_2", "motor-reach")];
        (0..2)
            .filter(|&k| !self.joints[k].gearbox.is_coupling())
            .enumerate()
            .map(|(scene_index, k)| ThermalPart {
                id: ids[k].0,
                kind: "gearbox",
                owner_key: ids[k].1,
                scene_part: "gearbox",
                scene_index,
            })
            .collect()
    }

    /// The machine with the arm's plane at the stroke's height z: the column below it with motor 1 and its gearbox at
    /// its foot, motor 2 and its gearbox at its top, the Z motor beside it; the upper arm, the forearm, the effector and
    /// the wafer; a bar on each motor turned by its rotor angle and its speed above it.
    fn scene(&self, s: &[f64], out: &mut Scene) {
        let (q, _) = self.coordinates(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let z = self.lift_state(s)[0];
        out.parts.clear();
        out.labels.clear();
        let grey = [0.62, 0.64, 0.68];
        out.parts.push(Part::new("base", Shape::Box { size: [0.4, 0.4, 0.05] }, [0.0, 0.0, -COLUMN_DROP - 0.1], grey));
        out.parts.push(Part::new(
            "column",
            Shape::Box { size: [0.16, 0.16, COLUMN_DROP] },
            [0.0, 0.0, z - 0.5 * COLUMN_DROP],
            [0.8, 0.8, 0.82],
        ));
        let elbow = self.arm.point(&q, 0, self.arm.link);
        let wrist = self.arm.point(&q, 1, self.arm.link);
        out.parts.push(Part::new(
            "upper-arm",
            Shape::Segment { to: [elbow[0], elbow[1], z], width: 0.07 },
            [0.0, 0.0, z],
            [0.2, 0.22, 0.26],
        ));
        out.parts.push(Part::new(
            "forearm",
            Shape::Segment { to: [wrist[0], wrist[1], z + 0.04], width: 0.06 },
            [elbow[0], elbow[1], z + 0.04],
            [0.3, 0.32, 0.36],
        ));
        out.parts.extend(self.on_effector(&q, z + 0.07));
        let lift = self.lift_state(s);
        let places = [[0.0, 0.0, z - COLUMN_DROP + 0.05], [0.0, 0.0, z - 0.06], [0.25, 0.0, -COLUMN_DROP - 0.02]];
        let turns = [rotor[0], rotor[1], lift[0] * self.lift.ratio()];
        let speeds = [rotor_speed[0], rotor_speed[1], lift[1] * self.lift.ratio()];
        for k in 0..3 {
            let at = places[k];
            out.parts.push(Part::new(
                sim_core::scene::MOTOR_PART,
                Shape::Box { size: [0.07, 0.07, 0.09] },
                at,
                [0.9, 0.45, 0.1],
            ));
            if k < 2 && !self.joints[k].gearbox.is_coupling() {
                out.parts.push(Part::new(
                    "gearbox",
                    Shape::Box { size: [0.08, 0.08, 0.04] },
                    [at[0], at[1] - 0.08, at[2]],
                    [0.25, 0.25, 0.28],
                ));
            }
            out.parts.push(
                Part::new(
                    "rotor-bar",
                    Shape::Box { size: [0.06, 0.004, 0.01] },
                    [at[0], at[1] - 0.04, at[2]],
                    [0.1, 0.1, 0.1],
                )
                .turned(turns[k]),
            );
            out.labels.push(Label {
                pos: [at[0] + 0.1, at[1], at[2]],
                owner_key: self.motors[k].key,
                value: speeds[k] * 60.0 / std::f64::consts::TAU,
                unit_key: "unit-rpm",
                decimals: 0,
            });
        }
    }

    /// The wafer centre at the effector's height.
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        let c = unit(tool_pose[1]);
        Some([tool_pose[0] * c[0], tool_pose[0] * c[1], tool_pose[2] + 0.075])
    }

    /// The wafer, where `scene` draws it, and the effector as the box around its strip.
    fn collision_bodies(&self, s: &[f64]) -> Vec<sim_core::Aabb> {
        let (q, _) = self.coordinates(s);
        let z = self.lift_state(s)[0] + 0.07;
        let mut out = sim_core::scene::scene_boxes(&self.on_effector(&q, z), &["wafer"]);
        let (wrist, centre) = (self.arm.point(&q, 1, self.arm.link), self.arm.wafer_centre(&q));
        out.push(sim_core::Aabb {
            centre: [0.5 * (wrist[0] + centre[0]), 0.5 * (wrist[1] + centre[1]), z],
            size: [(wrist[0] - centre[0]).abs() + 0.06, (wrist[1] - centre[1]).abs() + 0.06, 0.01],
        });
        out
    }
}

#[cfg(test)]
mod tests;
