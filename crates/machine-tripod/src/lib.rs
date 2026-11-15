//! Tripod (3-RPS parallel mechanism): a platform on three electric cylinders of variable length, each a ball screw
//! driven by its own servo motor through a coupling, with a revolute joint at the base and a spherical joint at the
//! platform. Three degrees of freedom: the tool pose is the platform centre's height z (m) and the platform's tilts rx
//! about x and ry about y (rad), as tilting tables, tool heads and motion bases use them.
//!
//! Why 3-RPS: the legs are electric cylinders (one per motor, the motor in line with the screw), each
//! on a trunnion at the base that lets it swing in the vertical plane through the base's centre, and on a ball joint at
//! the platform. The other common form, 3-PRS (three vertical slides carrying struts of fixed length, the Sprint Z3
//! head), has the same constraints and dependent motion; the cylinders of variable length are this crate's form.
//!
//! Geometry (`[tripod]`): base joints B_i = rb (cos t_i, sin t_i, 0) at t_i = 0, 120, 240 degrees, each revolute about
//! the tangent e_i = (-sin t_i, cos t_i, 0); platform joints a_i = rp (cos t_i, sin t_i, 0) in the platform frame. Leg
//! i's length is L_i = `leg.closed_length_m` + q_i with q_i the cylinder's extension, its joint coordinate.
//!
//! ```text
//! orientation  R = Rx(rx) Ry(ry) Rz(g): the pure tilt R0 = Rx(rx) Ry(ry), then a turn g about the platform's normal
//! constraints  e_i . (p + R a_i) = 0 (each platform joint stays in its leg's plane), which for t_i = 0, 120, 240 are
//!              R12 = R21, p_x = rp (R11 - R22) / 2, p_y = -rp R21, so in closed form (the dependent coordinates):
//!              tan g = -sin rx sin ry / (cos rx + cos ry), p_x = rp (cos ry cos g + sin rx sin ry sin g - cos rx cos g) / 2,
//!              p_y = rp cos ry sin g, p_z = z
//! inverse      d_i = p + R a_i - B_i, L_i = |d_i|, q_i = L_i - L_closed                                    (closed form)
//! twist        xi = (v, w): G xi = (q', 0), G's rows (u_i, b_i x u_i) (legs) and (e_i, b_i x e_i) (constraints),
//!              b_i = R a_i, u_i = d_i / L_i; A = the first three columns of G^-1: xi = A q'
//! rates        x' = (z', rx', ry') = S xi: z' = v_z, rx' = w_x + tan ry (sin rx w_y - cos rx w_z),
//!              ry' = cos rx w_y + sin rx w_z (w = rx' e_x + ry' Rx e_y + g' R0 e_z solved); K^-1 = S A
//! forward      chord iterations x += K(home)^-1 (q - q(x)) from the centre of the box, which contract over the working
//!              range (`chord_contraction`, docs/mechanisms.md); after the first, steps with K(x1)^-1 at that iterate
//!              while each halves the legs' miss, else the chord again; until |q - q(x)| < 1e-13 m, refused (NaN)
//!              where that takes more than 100 steps.
//! dynamics     G xi' = (q'' - a0, -c0), a0_i = (|w_i|^2 - (u_i . w_i)^2) / L_i + u_i . (w x (w x b_i)),
//!              w_i = v + w x b_i, c0_i = e_i . (w x (w x b_i)), so xi' = A q'' + beta, beta = G^-1 (-a0, -c0);
//!              M_xi xi' + c + g = J^T f + C^T lambda with M_xi = [[m 1, -[s]x], [[s]x, I_p]], s = R s_b the first
//!              moment about the centre, c = (w x (w x s), w x I_p w), g = (0, 0, m g, g s_y, -g s_x, 0); A^T takes
//!              the constraint forces out (C A = 0) and A^T J^T = 1: M_q = A^T M_xi A, h = A^T (M_xi beta + c + g)
//! ```
//!
//! f_i is the force leg i pushes the platform with: what its screw passes on (`chain::JointMachine`, one
//! `joint::RotaryJoint` with a `transmission::BallScrew` per leg). Each leg's mass is split between
//! its ends: the share `leg.platform_share` moves with the platform joint (a point mass there), the rest sits at the
//! base joint and does not move. The payload is a body of its own mass and inertia with its centre at
//! `mass.payload_offset_m` in the platform frame.
//! shortcut: the legs' swing about their base joints (their inertia about it) is left out; add it
//! where the legs weigh as much as the platform.

use sim_components::chain::{
    JOINT_KEYS, JointMachine, JointSpec, MOTOR_IDS, MOTOR_KEYS, Mechanism, condition_number, invert,
};
use sim_components::joint::RotaryJoint;
use sim_components::mech::{HardStop, ShaftFriction, ThermalFeedback};
use sim_components::rigid::{M3, V3, add, cross, dot, matmul, mul, transpose};
use sim_components::transmission::BallScrew;
use sim_core::{
    AxisInfo, Config, ConfigError, Label, Machine, MachineEntry, MotorInfo, Part, RatingCheck, Scene, Shape, SignalInfo,
};
use std::f64::consts::TAU;

pub static ENTRY: MachineEntry = MachineEntry {
    id: "tripod",
    key: "machine-tripod",
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

const AXIS_IDS: [&str; 3] = ["z", "rx", "ry"];
const AXIS_KEYS: [&str; 3] = ["axis-z", "axis-rx", "axis-ry"];

/// The default config with every loss switched off.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    cfg.set_text("gearbox.damping_nm_s_rad = 0.0").unwrap();
    cfg.set_text("leg.stop_damping_n_s_m = 0.0").unwrap();
    cfg
}

/// Loss-free rigid and elastic models, with and without the payload, all without gravity: held by constant leg forces
/// the platform under its weight is unstable (pushing legs of constant force, d^2/dz^2 of m g z - sum f_i L_i =
/// -sum f_i (1 - u_iz^2) / L_i < 0), a real property that the position loops hold. Gravity's part is checked on the
/// default config (static hold, loss budget) and by the hand-calculated leg forces.
fn suite_configs() -> Vec<Config> {
    [(false, true), (false, false), (true, true), (true, false)]
        .into_iter()
        .map(|(elastic, payload)| {
            let mut cfg = lossless_config();
            cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
            cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
            cfg.set_text("effects.gravity = false").unwrap();
            cfg
        })
        .collect()
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

fn scale(a: V3, k: f64) -> V3 {
    a.map(|v| v * k)
}

/// Rotation about x, y and z.
fn rot_x(a: f64) -> M3 {
    let (s, c) = a.sin_cos();
    [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
}
fn rot_y(a: f64) -> M3 {
    let (s, c) = a.sin_cos();
    [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]
}
fn rot_z(a: f64) -> M3 {
    let (s, c) = a.sin_cos();
    [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]
}

/// m (row by row, rows x cols) times v.
fn apply(m: &[f64], v: &[f64]) -> Vec<f64> {
    let cols = v.len();
    (0..m.len() / cols).map(|r| (0..cols).map(|c| m[r * cols + c] * v[c]).sum()).collect()
}

/// The LU factors of G (6 x 6) with partial pivoting, to solve G x = b for several b at the price of one elimination.
pub struct Lu {
    a: [f64; 36],
    perm: [usize; 6],
}

impl Lu {
    /// `None` where G is singular.
    pub fn new(m: &[f64]) -> Option<Self> {
        let mut a = [0.0; 36];
        a.copy_from_slice(&m[..36]);
        let mut perm = [0, 1, 2, 3, 4, 5];
        for c in 0..6 {
            let p = (c..6).max_by(|&i, &j| a[i * 6 + c].abs().total_cmp(&a[j * 6 + c].abs()))?;
            if !(a[p * 6 + c].abs() > 1e-300) {
                return None;
            }
            if p != c {
                for k in 0..6 {
                    a.swap(p * 6 + k, c * 6 + k);
                }
                perm.swap(p, c);
            }
            for r in c + 1..6 {
                let f = a[r * 6 + c] / a[c * 6 + c];
                a[r * 6 + c] = f;
                for k in c + 1..6 {
                    a[r * 6 + k] -= f * a[c * 6 + k];
                }
            }
        }
        Some(Self { a, perm })
    }

    pub fn solve(&self, b: &[f64; 6]) -> [f64; 6] {
        let mut x: [f64; 6] = std::array::from_fn(|i| b[self.perm[i]]);
        for r in 1..6 {
            for k in 0..r {
                x[r] -= self.a[r * 6 + k] * x[k];
            }
        }
        for r in (0..6).rev() {
            for k in r + 1..6 {
                x[r] -= self.a[r * 6 + k] * x[k];
            }
            x[r] /= self.a[r * 6 + r];
        }
        x
    }

    /// A, the first three columns of G^-1: the twist per leg rate, column by column.
    pub fn columns(&self) -> [[f64; 6]; 3] {
        std::array::from_fn(|c| self.solve(&std::array::from_fn(|r| if r == c { 1.0 } else { 0.0 })))
    }
}

/// Where the platform is at a tool pose (z, rx, ry): its centre p, its rotation R and the dependent turn g about its
/// normal (the parasitic x, y are p's).
#[derive(Debug, Clone, Copy)]
pub struct Place {
    pub p: V3,
    pub r: M3,
    pub turn: f64,
}

/// The legs at a platform place: unit vectors u_i from base to platform joint, lengths L_i, and the platform joints'
/// offsets b_i = R a_i from the platform centre (base frame).
#[derive(Debug, Clone, Copy)]
pub struct Legs {
    pub u: [V3; 3],
    pub length: [f64; 3],
    pub b: [V3; 3],
}

/// The tripod's geometry and kinematics, apart from masses and drives.
#[derive(Debug, Clone)]
pub struct Geometry {
    /// Base joints (base frame), platform joints (platform frame), the base joints' revolute axes (tangents) and the
    /// legs' radial directions.
    pub base: [V3; 3],
    pub anchors: [V3; 3],
    pub tangents: [V3; 3],
    pub radial: [V3; 3],
    pub platform_radius: f64,
    pub closed: f64,
    pub stroke: f64,
    /// The largest swing of a leg in its plane about its base joint, from the base's normal, and the largest cone
    /// angle of a spherical joint (the leg against the platform's normal), rad.
    pub max_revolute: f64,
    pub max_cone: f64,
    pub max_condition: f64,
    /// The pose the forward kinematics starts from (the centre of the tool's box) and K^-1 there, row by row.
    pub home: [f64; 3],
    pub chord: Vec<f64>,
}

impl Geometry {
    pub fn new(base_radius: f64, platform_radius: f64) -> Self {
        let azimuth = [0.0f64, 120.0, 240.0].map(|deg| deg.to_radians().sin_cos());
        let radial = azimuth.map(|(s, c)| [c, s, 0.0]);
        Self {
            base: radial.map(|r| scale(r, base_radius)),
            anchors: radial.map(|r| scale(r, platform_radius)),
            tangents: azimuth.map(|(s, c)| [-s, c, 0.0]),
            radial,
            platform_radius,
            closed: 0.0,
            stroke: 1.0,
            max_revolute: std::f64::consts::FRAC_PI_2,
            max_cone: std::f64::consts::FRAC_PI_2,
            max_condition: f64::INFINITY,
            home: [0.0; 3],
            chord: Vec::new(),
        }
    }

    /// Sets the start of the forward kinematics and its chord matrix K(home)^-1; `false` where home is singular.
    pub fn set_home(&mut self, home: [f64; 3]) -> bool {
        self.home = home;
        let legs = self.legs(&home);
        match self.rates(&home, &legs) {
            Some(k) => {
                self.chord = k;
                true
            }
            None => false,
        }
    }

    /// The platform's place at a tool pose, in closed form.
    pub fn place(&self, tool: &[f64]) -> Place {
        let (z, a, b) = (tool[0], tool[1], tool[2]);
        let (sa, ca) = a.sin_cos();
        let (sb, cb) = b.sin_cos();
        let turn = (-sa * sb).atan2(ca + cb);
        let (sg, cg) = turn.sin_cos();
        let rp = self.platform_radius;
        let p = [rp * (cb * cg + sa * sb * sg - ca * cg) / 2.0, rp * cb * sg, z];
        Place { p, r: matmul(&matmul(&rot_x(a), &rot_y(b)), &rot_z(turn)), turn }
    }

    pub fn legs_at(&self, place: &Place) -> Legs {
        let mut legs = Legs { u: [[0.0; 3]; 3], length: [0.0; 3], b: [[0.0; 3]; 3] };
        for i in 0..3 {
            let b = mul(&place.r, self.anchors[i]);
            let d = sub(add(place.p, b), self.base[i]);
            let l = norm(d);
            legs.u[i] = scale(d, 1.0 / l);
            legs.length[i] = l;
            legs.b[i] = b;
        }
        legs
    }

    pub fn legs(&self, tool: &[f64]) -> Legs {
        self.legs_at(&self.place(tool))
    }

    /// G (6 x 6, row by row): the legs' extension rates and the constraints' rates over the platform's twist (v, w).
    pub fn constraint_matrix(&self, legs: &Legs) -> Vec<f64> {
        let mut g = vec![0.0; 36];
        for i in 0..3 {
            let (m, n) = (cross(legs.b[i], legs.u[i]), cross(legs.b[i], self.tangents[i]));
            let e = self.tangents[i];
            g[i * 6..i * 6 + 6].copy_from_slice(&[legs.u[i][0], legs.u[i][1], legs.u[i][2], m[0], m[1], m[2]]);
            g[(3 + i) * 6..(3 + i) * 6 + 6].copy_from_slice(&[e[0], e[1], e[2], n[0], n[1], n[2]]);
        }
        g
    }

    /// The tool's rates (z', rx', ry') per twist: S (3 x 6, row by row), at the tool pose's tilts.
    fn selection(tool: &[f64]) -> [f64; 18] {
        let (sa, ca) = tool[1].sin_cos();
        let tb = tool[2].tan();
        let mut s = [0.0; 18];
        s[2] = 1.0;
        s[6 + 3] = 1.0;
        s[6 + 4] = tb * sa;
        s[6 + 5] = -tb * ca;
        s[12 + 4] = ca;
        s[12 + 5] = sa;
        s
    }

    /// K^-1 = S A (3 x 3, row by row): the tool's rates per leg rates, at a tool pose and its legs; `None` where G is
    /// singular.
    pub fn rates(&self, tool: &[f64], legs: &Legs) -> Option<Vec<f64>> {
        let a = Lu::new(&self.constraint_matrix(legs))?.columns();
        let s = Self::selection(tool);
        Some((0..9).map(|rc| (0..6).map(|k| s[rc / 3 * 6 + k] * a[rc % 3][k]).sum()).collect())
    }

    /// The Jacobian K (3 x 3, row by row) of the legs' extension rates over the tool's rates (z', rx', ry').
    pub fn jacobian(&self, tool: &[f64]) -> Option<Vec<f64>> {
        invert(&self.rates(tool, &self.legs(tool))?, 3)
    }

    /// Leg extensions for a tool pose (closed form); `false` where a leg would be of no length.
    pub fn inverse(&self, tool: &[f64], q: &mut [f64]) -> bool {
        let legs = self.legs(tool);
        for i in 0..3 {
            q[i] = legs.length[i] - self.closed;
        }
        q[..3].iter().all(|v| v.is_finite()) && legs.length.iter().all(|l| *l > 0.0)
    }

    /// The legs' miss q + L_closed - L(x) at a tool pose and its largest magnitude.
    fn miss(&self, q: &[f64], x: &[f64; 3]) -> (Vec<f64>, f64) {
        let legs = self.legs(x);
        let miss: Vec<f64> = (0..3).map(|i| q[i] + self.closed - legs.length[i]).collect();
        let worst = miss.iter().fold(0.0f64, |m, v| m.max(v.abs()));
        (miss, worst)
    }

    /// The tool pose for leg extensions; NaN where it does not converge. Chord steps x += K(home)^-1 (q - q(x)) from
    /// `home`, which contract over the working range (`chord_contraction`); after the first of them the steps take
    /// K(x1)^-1 at that first iterate instead, which near the solution contract far faster, as long as each at least
    /// halves the legs' miss; at the first that does not, the chord steps take over for good. Until the legs miss by
    /// less than 1e-13 m, within 100 steps.
    pub fn forward(&self, q: &[f64], tool: &mut [f64]) {
        let mut x = self.home;
        let (mut miss, mut worst) = self.miss(q, &x);
        let (mut fast, mut fell_back, mut converged) = (None::<Vec<f64>>, false, false);
        for _ in 0..100 {
            if !worst.is_finite() || x[1].abs() > 1.5 || x[2].abs() > 1.5 {
                break;
            }
            if worst < 1e-13 {
                converged = true;
                break;
            }
            let step = apply(fast.as_deref().unwrap_or(&self.chord), &miss);
            let next = [x[0] + step[0], x[1] + step[1], x[2] + step[2]];
            let (next_miss, next_worst) = self.miss(q, &next);
            if fast.is_some() && !(next_worst < 0.5 * worst) {
                (fast, fell_back) = (None, true);
                continue;
            }
            (x, miss, worst) = (next, next_miss, next_worst);
            if fast.is_none() && !fell_back {
                fast = self.rates(&x, &self.legs(&x));
                fell_back = fast.is_none();
            }
        }
        if converged {
            tool[..3].copy_from_slice(&x);
        } else {
            tool.iter_mut().for_each(|v| *v = f64::NAN);
        }
    }

    /// Per leg its swing about the base joint from the base's normal (signed, outwards positive) and the spherical
    /// joint's cone angle against the platform's normal, rad.
    pub fn joint_angles(&self, tool: &[f64]) -> ([f64; 3], [f64; 3]) {
        let place = self.place(tool);
        let legs = self.legs_at(&place);
        let normal = [place.r[0][2], place.r[1][2], place.r[2][2]];
        (
            std::array::from_fn(|i| dot(legs.u[i], self.radial[i]).atan2(legs.u[i][2])),
            std::array::from_fn(|i| dot(legs.u[i], normal).clamp(-1.0, 1.0).acos()),
        )
    }

    /// The condition number of K with the tilts weighed by the platform radius (the platform joints' speed for a
    /// tilt): that of diag(1, rp, rp) K^-1, as a matrix and its inverse share it.
    pub fn condition(&self, tool: &[f64]) -> f64 {
        let Some(mut k) = self.rates(tool, &self.legs(tool)) else { return f64::INFINITY };
        for v in &mut k[3..] {
            *v *= self.platform_radius;
        }
        condition_number(&k, 3, 3)
    }

    /// The parasitic motion at a tool pose: the centre's x and y, the turn about the normal, and the pose's error
    /// against the pure tilt about (0, 0, z): the farthest any point of the joint circle (radius rp) lies from where the
    /// pure tilt puts it, sqrt(|d|^2 + c^2 + 2 c |(R0^T d)_xy|), d = (p_x, p_y, 0), c = 2 rp |sin(g / 2)|.
    pub fn parasitic(&self, tool: &[f64]) -> [f64; 4] {
        let place = self.place(tool);
        let d = [place.p[0], place.p[1], 0.0];
        let pure = matmul(&rot_x(tool[1]), &rot_y(tool[2]));
        let local = mul(&transpose(&pure), d);
        let c = 2.0 * self.platform_radius * (0.5 * place.turn).sin().abs();
        let error = (dot(d, d) + c * c + 2.0 * c * local[0].hypot(local[1])).sqrt();
        [place.p[0], place.p[1], place.turn, error]
    }

    /// The chord iteration's contraction at a tool pose: the max-row-sum norm of W (1 - K(home)^-1 K(x)) W^-1, W =
    /// diag(1, rp, rp), the factor by which one chord step shrinks the error (weighed so) near x.
    pub fn chord_contraction(&self, tool: &[f64]) -> f64 {
        let Some(k) = self.jacobian(tool) else { return f64::INFINITY };
        let w = [1.0, self.platform_radius, self.platform_radius];
        (0..3)
            .map(|r| {
                (0..3)
                    .map(|c| {
                        let prod: f64 = (0..3).map(|m| self.chord[r * 3 + m] * k[m * 3 + c]).sum();
                        (w[r] * (if r == c { 1.0 } else { 0.0 } - prod) / w[c]).abs()
                    })
                    .sum::<f64>()
            })
            .fold(0.0, f64::max)
    }
}

/// The moving body: mass, first moment about the platform centre (platform frame) and inertia about the centre
/// (platform frame).
#[derive(Debug, Clone, Copy)]
struct Body {
    mass: f64,
    moment: V3,
    inertia: M3,
}

impl Body {
    /// A body of mass m with its centre at c and its inertia `own` about its centre, about the origin.
    fn at(m: f64, c: V3, own: M3) -> Self {
        let inertia = std::array::from_fn(|i| {
            std::array::from_fn(|j| own[i][j] + m * (if i == j { dot(c, c) } else { 0.0 } - c[i] * c[j]))
        });
        Self { mass: m, moment: scale(c, m), inertia }
    }
    fn plus(self, o: Self) -> Self {
        Self {
            mass: self.mass + o.mass,
            moment: add(self.moment, o.moment),
            inertia: std::array::from_fn(|i| std::array::from_fn(|j| self.inertia[i][j] + o.inertia[i][j])),
        }
    }
}

fn diagonal(cfg: &Config, key: &str) -> Result<M3, ConfigError> {
    let d = cfg.f64s(key)?;
    if d.len() != 3 || d.iter().any(|v| !(*v >= 0.0)) {
        return Err(ConfigError::Invalid(key.into()));
    }
    Ok(std::array::from_fn(|i| std::array::from_fn(|j| if i == j { d[i] } else { 0.0 })))
}

#[derive(Clone)]
pub struct Tripod {
    pub geo: Geometry,
    screw: BallScrew,
    /// The platform with the legs' platform shares, and with the payload too.
    empty: Body,
    loaded: Body,
    attached: bool,
    g: f64,
    required_life: Option<f64>,
    max_force: f64,
    parasitic_tolerance: Option<f64>,
}

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    let (machine, _) = Tripod::from_config(cfg)?;
    sim_components::base::mount(cfg, machine)
}

impl Tripod {
    /// The machine, and its mechanism alone for the tests.
    pub fn from_config(cfg: &Config) -> Result<(Box<dyn Machine>, Tripod), ConfigError> {
        let mut geo = Geometry::new(cfg.positive("tripod.base_radius_m")?, cfg.positive("tripod.platform_radius_m")?);
        geo.closed = cfg.positive("leg.closed_length_m")?;
        geo.stroke = cfg.positive("leg.stroke_m")?;
        geo.max_revolute = cfg.positive("workspace.max_revolute_angle_deg")?.to_radians();
        geo.max_cone = cfg.positive("workspace.max_cone_angle_deg")?.to_radians();
        geo.max_condition = cfg.positive("workspace.max_condition_number")?;
        let (min, max) = (cfg.f64s("workspace.tool_min")?, cfg.f64s("workspace.tool_max")?);
        if min.len() != 3 || max.len() != 3 {
            return Err(ConfigError::Invalid("workspace.tool_min".into()));
        }
        if (0..3).any(|k| !(min[k] < max[k])) || (1..3).any(|k| !(min[k] > -1.0 && max[k] < 1.0)) {
            return Err(ConfigError::Invalid("workspace.tool_max".into()));
        }
        if !geo.set_home(std::array::from_fn(|k| 0.5 * (min[k] + max[k]))) {
            return Err(ConfigError::Invalid("workspace.tool_min".into()));
        }
        let share = cfg.non_negative("leg.platform_share")?;
        if share > 1.0 {
            return Err(ConfigError::Invalid("leg.platform_share".into()));
        }
        let at_joint = share * cfg.non_negative("leg.mass_kg")?;
        let platform_inertia = diagonal(cfg, "mass.platform_inertia_kgm2")?;
        let mut empty = Body::at(cfg.positive("mass.platform_kg")?, [0.0; 3], platform_inertia);
        for a in geo.anchors {
            empty = empty.plus(Body::at(at_joint, a, [[0.0; 3]; 3]));
        }
        let offset = cfg.f64s("mass.payload_offset_m")?;
        if offset.len() != 3 || offset.iter().any(|v| !v.is_finite()) {
            return Err(ConfigError::Invalid("mass.payload_offset_m".into()));
        }
        let payload = if cfg.bool("effects.payload")? { cfg.non_negative("mass.payload_kg")? } else { 0.0 };
        let own = if payload > 0.0 { diagonal(cfg, "mass.payload_inertia_kgm2")? } else { [[0.0; 3]; 3] };
        let loaded = empty.plus(Body::at(payload, [offset[0], offset[1], offset[2]], own));
        let screw = BallScrew::from_config(cfg)?;
        if screw.length < geo.stroke {
            return Err(ConfigError::Invalid("screw.length_m".into()));
        }
        let on = cfg.bool("effects.joint_friction")?;
        let friction = ShaftFriction {
            coulomb: if on { cfg.non_negative("leg.friction_coulomb_n")? } else { 0.0 },
            viscous: if on { cfg.non_negative("leg.friction_viscous_n_s_m")? } else { 0.0 },
            smoothing: cfg.positive("leg.friction_smoothing_m_s")?,
            ..ShaftFriction::NONE
        };
        let stops = if cfg.bool("effects.hard_stops")? {
            HardStop {
                stiffness: cfg.positive("leg.stop_stiffness_n_m")?,
                damping: cfg.non_negative("leg.stop_damping_n_s_m")?,
            }
        } else {
            HardStop { stiffness: 0.0, damping: 0.0 }
        };
        let mut joint = RotaryJoint::with(cfg, friction, stops, (0.0, geo.stroke))?;
        screw.converter(screw.softest_position()).apply(cfg, &mut joint)?;
        // The nut's preload drags like friction in the rod's guide, as a force at the nut.
        if on {
            joint.friction.coulomb += screw.preload_torque() * screw.ratio();
        }
        let lever = 1.0 / screw.ratio();
        let mech = Tripod {
            geo,
            screw,
            empty,
            loaded,
            attached: true,
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            required_life: cfg.f64("ratings.required_life_h").ok(),
            max_force: cfg.positive("leg.max_force_n")?,
            parasitic_tolerance: cfg.f64("ratings.parasitic_tolerance_m").ok(),
        };
        let axes = (0..3)
            .map(|k| {
                let unit = if k < 1 { "m" } else { "rad" };
                let get =
                    |what: &str, rate: &str| cfg.positive(&format!("axis.{}.max_{what}_{unit}_{rate}", AXIS_IDS[k]));
                Ok(AxisInfo {
                    id: AXIS_IDS[k],
                    key: AXIS_KEYS[k],
                    unit,
                    min: min[k],
                    max: max[k],
                    max_vel: get("velocity", "s")?,
                    max_acc: get("acceleration", "s2")?,
                    max_jerk: get("jerk", "s3")?,
                })
            })
            .collect::<Result<Vec<_>, ConfigError>>()?;
        let specs = (0..3)
            .map(|k| JointSpec {
                drive: joint.clone(),
                id: MOTOR_IDS[k],
                key: JOINT_KEYS[k],
                gear_lever: Some(lever),
                unit: "m",
            })
            .collect();
        let motors = (0..3).map(|k| MotorInfo { id: MOTOR_IDS[k], key: MOTOR_KEYS[k] }).collect();
        let machine = JointMachine::new(
            mech.clone(),
            specs,
            cfg.bool("effects.joint_elasticity")?,
            ThermalFeedback::from_config(cfg)?,
            axes,
            motors,
        );
        Ok((Box::new(machine), mech))
    }

    fn body(&self) -> Body {
        if self.attached { self.loaded } else { self.empty }
    }

    /// The pose for leg extensions, its place and legs; `None` where the forward kinematics fails.
    fn pose(&self, q: &[f64]) -> Option<([f64; 3], Place, Legs)> {
        let mut tool = [0.0; 3];
        self.geo.forward(q, &mut tool);
        if !tool.iter().all(|v| v.is_finite()) {
            return None;
        }
        let place = self.geo.place(&tool);
        Some((tool, place, self.geo.legs_at(&place)))
    }

    /// The body in the base frame at rotation R: mass, first moment s = R s_b, inertia about the centre R I_b R^T.
    fn turned(&self, r: &M3) -> (f64, V3, M3) {
        let b = self.body();
        (b.mass, mul(r, b.moment), matmul(&matmul(r, &b.inertia), &transpose(r)))
    }

    /// The twist's mass matrix (6 x 6, row by row): [[m 1, -[s]x], [[s]x, I_p]].
    fn twist_mass(m: f64, s: V3, ip: &M3) -> Vec<f64> {
        let sx = [[0.0, -s[2], s[1]], [s[2], 0.0, -s[0]], [-s[1], s[0], 0.0]];
        let mut out = vec![0.0; 36];
        for r in 0..3 {
            out[r * 6 + r] = m;
            for c in 0..3 {
                out[r * 6 + 3 + c] = -sx[r][c];
                out[(3 + r) * 6 + c] = sx[r][c];
                out[(3 + r) * 6 + 3 + c] = ip[r][c];
            }
        }
        out
    }

    /// The weight as the wrench the legs must hold: g = (0, 0, m g, g s_y, -g s_x, 0).
    fn weight(&self, m: f64, s: V3) -> [f64; 6] {
        [0.0, 0.0, m * self.g, self.g * s[1], -self.g * s[0], 0.0]
    }

    /// The forces the legs push the platform with to hold it at rest at a tool pose: A^T g, N.
    pub fn static_leg_forces(&self, tool: &[f64]) -> Option<[f64; 3]> {
        let place = self.geo.place(tool);
        let legs = self.geo.legs_at(&place);
        let a = Lu::new(&self.geo.constraint_matrix(&legs))?.columns();
        let (m, s, _) = self.turned(&place.r);
        let g = self.weight(m, s);
        Some(std::array::from_fn(|i| (0..6).map(|k| a[i][k] * g[k]).sum()))
    }
}

impl Mechanism for Tripod {
    fn dynamics(&self, q: &[f64], qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        let fail = |mass: &mut [f64], bias: &mut [f64]| {
            mass.fill(f64::NAN);
            bias.fill(f64::NAN);
        };
        let Some((_, place, legs)) = self.pose(q) else { return fail(mass, bias) };
        let Some(lu) = Lu::new(&self.geo.constraint_matrix(&legs)) else { return fail(mass, bias) };
        // A = the first three columns of G^-1.
        let columns = lu.columns();
        let a = |r: usize, c: usize| columns[c][r];
        let xi: Vec<f64> = (0..6).map(|r| (0..3).map(|c| a(r, c) * qd[c]).sum()).collect();
        let (v, w) = ([xi[0], xi[1], xi[2]], [xi[3], xi[4], xi[5]]);
        let mut rest = [0.0; 6];
        for i in 0..3 {
            let spin = cross(w, cross(w, legs.b[i]));
            let wi = add(v, cross(w, legs.b[i]));
            let along = dot(legs.u[i], wi);
            rest[i] = -((dot(wi, wi) - along * along) / legs.length[i] + dot(legs.u[i], spin));
            rest[3 + i] = -dot(self.geo.tangents[i], spin);
        }
        let beta = lu.solve(&rest);
        let (m, s, ip) = self.turned(&place.r);
        let mx = Self::twist_mass(m, s, &ip);
        let (lin, gyro) = (cross(w, cross(w, s)), cross(w, mul(&ip, w)));
        let weight = self.weight(m, s);
        let held = [lin[0], lin[1], lin[2], gyro[0], gyro[1], gyro[2]];
        let mbeta = apply(&mx, &beta);
        let wrench: Vec<f64> = (0..6).map(|k| mbeta[k] + held[k] + weight[k]).collect();
        for i in 0..3 {
            bias[i] = (0..6).map(|k| a(k, i) * wrench[k]).sum();
            for j in 0..3 {
                mass[i * 3 + j] = (0..6).map(|k| a(k, i) * (0..6).map(|l| mx[k * 6 + l] * a(l, j)).sum::<f64>()).sum();
            }
        }
    }

    /// The platform (its joints' share of the legs and the payload with it) at its centre p with its twist (v, w):
    /// L = m v + w x s, about the origin s x v + I_p w + p x L, its first moment m p + s. Each leg's rotor spins along
    /// the leg's u_i. The legs' shares at the base joints do not move.
    fn frame_momentum(&self, q: &[f64], qd: &[f64], rotor_spin: &[f64]) -> Option<sim_core::FrameMomentum> {
        let (_, place, legs) = self.pose(q)?;
        let a = Lu::new(&self.geo.constraint_matrix(&legs))?.columns();
        let xi: Vec<f64> = (0..6).map(|r| (0..3).map(|c| a[c][r] * qd[c]).sum()).collect();
        let (v, w) = ([xi[0], xi[1], xi[2]], [xi[3], xi[4], xi[5]]);
        let (m, s, ip) = self.turned(&place.r);
        let p = place.p;
        let linear = add(scale(v, m), cross(w, s));
        let angular = add(add(cross(s, v), mul(&ip, w)), cross(p, linear));
        let mut f = sim_core::FrameMomentum { gravity: [0.0, 0.0, -self.g], ..Default::default() };
        f.linear = linear;
        f.angular = angular;
        f.mass = m;
        f.first_moment = add(scale(p, m), s);
        for (u, spin) in legs.u.iter().zip(rotor_spin) {
            sim_components::base::add_spin(&mut f, *u, 1.0, *spin);
        }
        Some(f)
    }
    fn potential(&self, q: &[f64]) -> f64 {
        self.pose(q).map_or(f64::NAN, |(_, place, _)| {
            let (m, s, _) = self.turned(&place.r);
            self.g * (m * place.p[2] + s[2])
        })
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.attached = attached;
    }

    fn forward(&self, q: &[f64], tool: &mut [f64]) {
        self.geo.forward(q, tool);
    }
    fn inverse(&self, tool: &[f64], q: &mut [f64]) -> bool {
        self.geo.inverse(tool, q)
    }
    fn unreachable(&self, tool: &[f64], _slack: f64) -> Option<&'static str> {
        let (swing, cone) = self.geo.joint_angles(tool);
        if !swing.iter().all(|a| a.abs() <= self.geo.max_revolute) {
            return Some("error-tripod-revolute-angle");
        }
        if !cone.iter().all(|a| *a <= self.geo.max_cone) {
            return Some("error-tripod-cone-angle");
        }
        if !(self.geo.condition(tool) <= self.geo.max_condition) {
            return Some("error-tripod-singularity");
        }
        None
    }
    fn condition(&self, tool: &[f64]) -> Option<f64> {
        Some(self.geo.condition(tool))
    }
    fn tool_is_joints(&self) -> bool {
        false
    }

    /// Per leg its screw's speed, d n and wear, its stroke from the middle, its swing at the base joint and its cone
    /// angle at the platform joint; then the condition number and the parasitic motion at the pose.
    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out = Vec::new();
        for i in 0..3 {
            out.extend(self.screw.signal_infos(MOTOR_IDS[i], JOINT_KEYS[i]));
        }
        for i in 0..3 {
            let id = MOTOR_IDS[i];
            out.push(SignalInfo::new(format!("leg.{id}.from_middle"), JOINT_KEYS[i], "quantity-leg-from-middle", "m"));
            out.push(SignalInfo::new(format!("leg.{id}.swing"), JOINT_KEYS[i], "quantity-revolute-swing", "rad"));
            out.push(SignalInfo::new(format!("leg.{id}.cone"), JOINT_KEYS[i], "quantity-cone-angle", "rad"));
        }
        out.push(SignalInfo::new("platform.condition", "group-tripod", "quantity-condition-number", ""));
        out.push(SignalInfo::new("platform.x", "group-tripod", "quantity-parasitic-x", "m"));
        out.push(SignalInfo::new("platform.y", "group-tripod", "quantity-parasitic-y", "m"));
        out.push(SignalInfo::new("platform.turn", "group-tripod", "quantity-parasitic-turn", "rad"));
        out.push(SignalInfo::new("platform.pose_error", "group-tripod", "quantity-pose-error", "m"));
        out
    }
    fn signals(&self, q: &[f64], qd: &[f64], _qdd: &[f64], force: &[f64], out: &mut Vec<f64>) {
        for i in 0..3 {
            self.screw.signals(qd[i], force[i], out);
        }
        let tool = self.pose(q).map(|(tool, ..)| tool);
        let (swing, cone) = tool.map_or(([f64::NAN; 3], [f64::NAN; 3]), |t| self.geo.joint_angles(&t));
        for i in 0..3 {
            out.extend([q[i] - 0.5 * self.geo.stroke, swing[i], cone[i]]);
        }
        out.push(tool.map_or(f64::INFINITY, |t| self.geo.condition(&t)));
        out.extend(tool.map_or([f64::NAN; 4], |t| self.geo.parasitic(&t)));
    }
    /// Each leg's screw (speed, d n, buckling and static load under the leg's force, life), the leg's force against
    /// the cylinder's, its stroke, its joints' angles; the parasitic motion where a tolerance is given.
    fn rating_checks(&self) -> Vec<RatingCheck> {
        let mut out: Vec<RatingCheck> =
            (0..3).flat_map(|i| self.screw.rating_checks(MOTOR_IDS[i], i, self.required_life)).collect();
        for i in 0..3 {
            let id = MOTOR_IDS[i];
            let check = |check: &'static str, signal: String, limit: f64| RatingCheck {
                check,
                signal,
                motor: Some(i),
                limit,
                life: false,
            };
            out.push(check("tripod-leg-force", format!("joint.{id}.force"), self.max_force));
            out.push(check("tripod-leg-stroke", format!("leg.{id}.from_middle"), 0.5 * self.geo.stroke));
            out.push(check("tripod-revolute-angle", format!("leg.{id}.swing"), self.geo.max_revolute));
            out.push(check("tripod-cone-angle", format!("leg.{id}.cone"), self.geo.max_cone));
        }
        if let Some(limit) = self.parasitic_tolerance {
            out.push(RatingCheck {
                check: "tripod-parasitic",
                signal: "platform.pose_error".into(),
                motor: None,
                limit,
                life: false,
            });
        }
        out
    }

    fn collision_parts(&self) -> &'static [&'static str] {
        &["tool", "payload"]
    }

    /// Base and platform as triangles through their joints, each leg as its cylinder and rod, the motors below the
    /// base joints, the payload at its centre.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene) {
        let Some((_, place, legs)) = self.pose(q) else { return };
        let p = place.p;
        let top: Vec<V3> = (0..3).map(|i| add(p, legs.b[i])).collect();
        for i in 0..3 {
            let next = (i + 1) % 3;
            out.parts.push(Part::new(
                "base",
                Shape::Segment { to: self.geo.base[next], width: 0.04 },
                self.geo.base[i],
                [0.55, 0.57, 0.6],
            ));
            out.parts.push(Part::new(
                "platform",
                Shape::Segment { to: top[next], width: 0.04 },
                top[i],
                [0.3, 0.45, 0.7],
            ));
        }
        let body = 0.6 * self.geo.closed;
        for i in 0..3 {
            let (b, u) = (self.geo.base[i], legs.u[i]);
            let end = add(b, scale(u, body));
            out.parts.push(Part::new("cylinder", Shape::Segment { to: end, width: 0.06 }, b, [0.75, 0.75, 0.78]));
            out.parts.push(Part::new(
                "rod",
                Shape::Segment { to: add(b, scale(u, 0.9 * body)), width: 0.03 },
                top[i],
                [0.85, 0.86, 0.9],
            ));
            out.parts.push(Part::new(
                sim_core::scene::MOTOR_PART,
                Shape::Box { size: [0.08, 0.08, 0.1] },
                add(b, [0.0, 0.0, -0.07]),
                [0.9, 0.45, 0.1],
            ));
            // The shaft's bar below the motor turns with the rotor, as on every machine.
            out.parts.push(
                Part::new("rotor-bar", Shape::Box { size: [0.08, 0.004, 0.012] }, add(b, [0.0, 0.0, -0.13]), [0.1; 3])
                    .turned(rotors[i]),
            );
            out.labels.push(Label {
                pos: add(b, [0.0, 0.0, -0.16]),
                owner_key: MOTOR_KEYS[i],
                value: rotor_speeds[i] * 60.0 / TAU,
                unit_key: "unit-rpm",
                decimals: 0,
            });
            out.labels.push(Label {
                pos: add(b, scale(u, 0.5 * legs.length[i])),
                owner_key: JOINT_KEYS[i],
                value: legs.length[i],
                unit_key: "unit-m",
                decimals: 3,
            });
        }
        let normal = [place.r[0][2], place.r[1][2], place.r[2][2]];
        out.parts.push(Part::new(
            "tool",
            Shape::Box { size: [0.08, 0.08, 0.03] },
            add(p, scale(normal, 0.015)),
            [0.2, 0.22, 0.26],
        ));
        let b = self.body();
        if self.attached && (b.mass - self.empty.mass) > 0.0 {
            let own = sub(b.moment, self.empty.moment);
            let centre = add(p, mul(&place.r, scale(own, 1.0 / (b.mass - self.empty.mass))));
            out.parts.push(Part::new("payload", Shape::Box { size: [0.2, 0.2, 0.12] }, centre, [0.2, 0.7, 0.3]));
        }
        out.labels.push(Label {
            pos: add(p, [0.0, 0.0, 0.25]),
            owner_key: "axis-z",
            value: p[2],
            unit_key: "unit-m",
            decimals: 3,
        });
    }
    fn scene_point(&self, tool: &[f64]) -> Option<[f64; 3]> {
        Some(self.geo.place(tool).p)
    }
}

#[cfg(test)]
mod tests;
