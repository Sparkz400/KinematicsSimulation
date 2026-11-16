//! A six-axis platform on six cranks (6-RSS): six servo motors with gearboxes on the base, each turning a crank in its
//! own plane (R), each crank's tip joined to the platform by a rod with a ball joint at both ends (S, S). The build of
//! motion simulators and fast six-axis positioners. Six degrees of freedom: the tool pose is the centre of the platform
//! joints' plane x, y, z (m, z above the plane of the crank axes) and the platform's orientation as a rotation vector
//! rx, ry, rz (rad, axis times angle, in the base frame). Each joint coordinate is a crank angle theta_i (rad).
//!
//! Geometry (`[platform]`, `[crank]`, `[rod]`): the crank axes' centres B_i on a circle of `base_radius_m` in pairs at
//! 120 k -+ `base_half_angle_deg`, the platform joints a_i (platform frame) on a circle of `platform_radius_m` at
//! 120 k -+ (60 - `platform_half_angle_deg`), so that each leg runs from a base pair to the neighbouring platform pair.
//! Each crank axis k_i is horizontal: the base radius through B_i turned about z by -+ `crank.axis_turn_deg` (mirrored
//! in a pair). The crank turns in the vertical plane across it, spanned by d_i = k_i x z... written the other way round:
//! d_i is the horizontal direction in the crank's plane that points away from the pair's partner, and k_i = d_i x z, so
//! that a positive theta lifts the tip:
//!
//! ```text
//! tip      C_i = B_i + r (cos theta d_i + sin theta z),  t_i = dC/dtheta = r (-sin theta d_i + cos theta z),
//!          t2_i = d^2C/dtheta^2 = -r (cos theta d_i + sin theta z)
//! joint    P_i = p + b_i,  b_i = R a_i;  the rod  |P_i - C_i| = L,  u_i = (P_i - C_i) / L
//! inverse  e = P_i - B_i:  a cos theta + b sin theta = c,  a = e . d_i,  b = e . z,  c = (|e|^2 + r^2 - L^2) / (2 r),
//!          theta = atan2(b, a) -+ acos(c / sqrt(a^2 + b^2)): "out" takes -, "in" takes + (`crank.branch`, once for all
//!          legs); no solution where |c| > sqrt(a^2 + b^2) (the rod cannot reach)
//! rates    A xi = diag(m) theta',  A row i = (u_i, b_i x u_i),  m_i = u_i . t_i (the crank's torque arm, m),
//!          xi = (v, w) the platform's twist: xi = K theta', K = A^-1 diag(m)
//!          A xi' = diag(m) theta'' + s,  s_i = u_i . t2_i theta_i'^2 - |v_i - t_i theta_i'|^2 / L - u_i . (w x (w x b_i)),
//!          v_i = v + w x b_i
//! forward  Newton on the twist: xi = A^-1 (L - |P_i - C_i|), p += v, R = exp(w) R, from the home pose, until the rods'
//!          lengths are within 1e-12 m, and one step more; refused (NaN) where it does not converge within 40 steps
//! ```
//!
//! The dead centre: the rod in the plane of the crank and its axis, u_i . t_i = 0, where the motor's torque arm m_i
//! vanishes; the angle between the rod and the crank's tangent, acos(|u_i . t_i| / r), is 90 degrees there.
//!
//! Dynamics by virtual work in the crank angles (M theta'' + h = tau at the cranks):
//!
//! - the platform with the payload, one rigid body of mass m, first moment S = R s and inertia I = R I_p R^T about p:
//!   M_xi = [m 1, -[S]x; [S]x, I], its required wrench W = M_xi xi' + (w x (w x S), w x I w) - (m g, S x g), through K:
//!   K^T M_xi K and K^T (M_xi xi0' + c - G), xi0' = A^-1 s the twist's acceleration at theta'' = 0;
//! - each rod a slender rigid rod (ball joints at both ends, its spin about its axis left out): a sixth of its mass at
//!   each end, two thirds at its middle, which is exact for a uniform rod's mass, centre and inertia across; each a point
//!   with its Jacobian P (3 x 6) in theta: m P^T P and m P^T (a0 - g);
//! - each crank its inertia about its axis and its weight at `crank.cg_m` from the axis.
//!
//! The motor, its rotor through the ratio, the gearbox (stiffness, backlash, efficiency, drag) and the crank bearings'
//! friction are each a `joint::RotaryJoint` (`chain::JointMachine`), as on the arms and the delta pickers.
//!
//! The force along each rod (`rod_forces`, compression positive) follows from the platform: A^T f = W plus the rods'
//! upper ends, the rod's own mass carried by its nodes (its upper sixth and half its middle at the platform joint).
//! shortcut: the rods' spin about their own axes (free between two ball joints) is left out, as is the ball joints'
//! friction; parts that cross are checked only for the rods among each other.

use sim_components::chain::{
    JOINT_KEYS, JointMachine, JointSpec, MOTOR_IDS, MOTOR_KEYS, Mechanism, condition_number, solve,
};
use sim_components::joint::RotaryJoint;
use sim_components::mech::ThermalFeedback;
use sim_components::rigid::{M3, V3, add, cross, dot, exp, log, matmul, mul, transpose};
use sim_core::{
    AxisInfo, Config, ConfigError, Label, Machine, MachineEntry, MotorInfo, Part, RatingCheck, Scene, Shape, SignalInfo,
};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

pub static ENTRY: MachineEntry = MachineEntry {
    id: "hexapod-rotary",
    key: "machine-hexapod-rotary",
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

const AXIS_IDS: [&str; 6] = ["x", "y", "z", "rx", "ry", "rz"];
const AXIS_KEYS: [&str; 6] = ["axis-x", "axis-y", "axis-z", "axis-rx", "axis-ry", "axis-rz"];
const Z: V3 = [0.0, 0.0, 1.0];

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

/// Loss-free rigid and elastic models, with and without the payload, all without gravity: held by constant crank
/// torques, as the suite holds it after its pulse, the platform under its weight is not held in place (the cranks'
/// torques at rest keep the rods' forces, not the pose), so it runs away from the pose, a real property that the
/// position loops hold and not a numerical one (tried: the rigid loss-free config with gravity leaves the pose within the
/// suite's 10 s, its invariant off by 80 times the pulse's energy). Gravity's part is checked on the default config
/// (static hold, loss budget) and by the hand-calculated holding torques. Without gravity nothing holds the gearboxes against a flank
/// either: the suite's pulse sends them across their backlash, and each crossing's impact (whether it falls before or
/// after a step's end) changed the travel after a second by 1.2 % when the step was halved. So the backlash is off
/// here too; the default config, which runs the suite's static hold and loss budget, keeps it.
fn suite_configs() -> Vec<Config> {
    [(false, true), (false, false), (true, true), (true, false)]
        .into_iter()
        .map(|(elastic, payload)| {
            let mut cfg = lossless_config();
            cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
            cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
            cfg.set_text("effects.gravity = false").unwrap();
            cfg.set_text("effects.gearbox_backlash = false").unwrap();
            cfg
        })
        .collect()
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: V3, k: f64) -> V3 {
    a.map(|v| v * k)
}

fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

/// A 6 x 6 matrix factored by Gaussian elimination with partial pivoting (P A = L U), to solve with it more than once.
struct Lu {
    a: [f64; 36],
    pivot: [usize; 6],
}

impl Lu {
    /// The factors of `m` (row by row); `None` where it is singular.
    fn of(m: &[f64]) -> Option<Lu> {
        let mut a: [f64; 36] = std::array::from_fn(|k| m[k]);
        let mut pivot = [0; 6];
        for c in 0..6 {
            let p = (c..6).max_by(|&i, &j| a[i * 6 + c].abs().total_cmp(&a[j * 6 + c].abs()))?;
            if !(a[p * 6 + c].abs() > 1e-300) {
                return None;
            }
            pivot[c] = p;
            if p != c {
                for k in 0..6 {
                    a.swap(p * 6 + k, c * 6 + k);
                }
            }
            for r in c + 1..6 {
                let f = a[r * 6 + c] / a[c * 6 + c];
                a[r * 6 + c] = f;
                for k in c + 1..6 {
                    a[r * 6 + k] -= f * a[c * 6 + k];
                }
            }
        }
        Some(Lu { a, pivot })
    }

    /// x with A x = b.
    fn solve(&self, mut b: [f64; 6]) -> [f64; 6] {
        // The rows' interchanges first, in their order: the multipliers moved with every later interchange.
        for c in 0..6 {
            b.swap(c, self.pivot[c]);
        }
        for c in 0..6 {
            for r in c + 1..6 {
                b[r] -= self.a[r * 6 + c] * b[c];
            }
        }
        for c in (0..6).rev() {
            let s: f64 = (c + 1..6).map(|k| self.a[c * 6 + k] * b[k]).sum();
            b[c] = (b[c] - s) / self.a[c * 6 + c];
        }
        b
    }
}

/// The least distance between the segments p0-p1 and q0-q1.
pub fn segment_distance(p0: V3, p1: V3, q0: V3, q1: V3) -> f64 {
    let (d1, d2, r) = (sub(p1, p0), sub(q1, q0), sub(p0, q0));
    let (a, e, f) = (dot(d1, d1), dot(d2, d2), dot(d2, r));
    let (c, b) = (dot(d1, r), dot(d1, d2));
    let denom = a * e - b * b;
    let mut s = if denom > 1e-18 { ((b * f - c * e) / denom).clamp(0.0, 1.0) } else { 0.0 };
    let mut t = (b * s + f) / e;
    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }
    norm(sub(add(p0, scale(d1, s)), add(q0, scale(d2, t))))
}

/// One leg at a pose: the crank's tip C, its tangent t and second derivative t2 by the crank angle, the platform joint
/// P and its offset b = R a from the platform centre, the rod's unit vector u from C to P, and the torque arm u . t.
#[derive(Debug, Clone, Copy, Default)]
pub struct Leg {
    pub tip: V3,
    pub t: V3,
    pub t2: V3,
    pub joint: V3,
    pub b: V3,
    pub u: V3,
    pub arm: f64,
}

/// The kinematics, apart from masses and drives.
#[derive(Debug, Clone)]
pub struct Geometry {
    /// Crank axes' centres (base frame), the cranks' zero directions d_i, their axes k_i, and the platform joints
    /// (platform frame).
    pub base: [V3; 6],
    pub dir: [V3; 6],
    pub axis: [V3; 6],
    pub anchors: [V3; 6],
    pub platform_radius: f64,
    pub crank: f64,
    pub rod: f64,
    /// -1 for the branch "out", +1 for "in".
    pub branch: f64,
    /// Working range: the least angle from the dead centre (rad), the steepest rod at the crank's ball joint (out of the
    /// crank's plane) and at the platform's (against the platform's normal), rad, the least distance between two rods
    /// (m), the highest condition number.
    pub min_margin: f64,
    pub max_crank_joint: f64,
    pub max_platform_joint: f64,
    pub min_clearance: f64,
    pub max_condition: f64,
    /// The pose Newton's forward kinematics starts from: the centre of the tool's box.
    pub home: [f64; 6],
    /// The cranks' angles at `home` and the twist per crank angle there (K, 6 x 6): Newton starts from home moved by
    /// K (q - q_home), a guess that depends on q alone (`set_home`); it saves about two of its steps.
    home_q: [f64; 6],
    home_k: [f64; 36],
}

impl Geometry {
    /// Joints at the azimuths 120 k -+ the base half angle and 120 k -+ (60 - the platform half angle), degrees; each
    /// crank axis the base radius turned by -+ `axis_turn` degrees; cranks of length `crank`, rods of `rod`.
    pub fn new(
        base_radius: f64,
        base_half: f64,
        platform_radius: f64,
        platform_half: f64,
        axis_turn: f64,
        crank: f64,
        rod: f64,
        out: bool,
    ) -> Self {
        let at = |radius: f64, deg: f64| [radius * deg.to_radians().cos(), radius * deg.to_radians().sin(), 0.0];
        let side = |i: usize| if i.is_multiple_of(2) { -1.0 } else { 1.0 };
        let base = std::array::from_fn(|i| at(base_radius, 120.0 * (i / 2) as f64 + side(i) * base_half));
        let anchors =
            std::array::from_fn(|i| at(platform_radius, 120.0 * (i / 2) as f64 + side(i) * (60.0 - platform_half)));
        let dir: [V3; 6] = std::array::from_fn(|i| {
            let n = at(1.0, 120.0 * (i / 2) as f64 + side(i) * (base_half + axis_turn));
            // z x n, the tangent turning counter-clockwise, towards the partner for the first of a pair.
            scale([-n[1], n[0], 0.0], side(i))
        });
        let axis = std::array::from_fn(|i| cross(dir[i], Z));
        Self {
            base,
            dir,
            axis,
            anchors,
            platform_radius,
            crank,
            rod,
            branch: if out { -1.0 } else { 1.0 },
            min_margin: 0.0,
            max_crank_joint: FRAC_PI_2,
            max_platform_joint: PI,
            min_clearance: 0.0,
            max_condition: f64::INFINITY,
            home: [0.0; 6],
            home_q: [0.0; 6],
            home_k: [0.0; 36],
        }
    }

    /// Sets the pose the forward kinematics starts from, and its first-order guess there (none where home is out of
    /// reach or singular).
    pub fn set_home(&mut self, home: [f64; 6]) {
        (self.home, self.home_q, self.home_k) = (home, [0.0; 6], [0.0; 36]);
        let mut q = [0.0; 6];
        if !self.inverse(&home, &mut q) {
            return;
        }
        let legs = self.legs_at(&q, [home[0], home[1], home[2]], &Self::rotation(&home));
        let Some(lu) = Lu::of(&Self::constraint(&legs)) else { return };
        for c in 0..6 {
            let col = lu.solve(std::array::from_fn(|i| if i == c { legs[c].arm } else { 0.0 }));
            (0..6).for_each(|r| self.home_k[r * 6 + c] = col[r]);
        }
        self.home_q = q;
    }

    pub fn rotation(tool: &[f64]) -> M3 {
        exp([tool[3], tool[4], tool[5]])
    }

    /// Crank i's tip, tangent and second derivative at the angle theta.
    pub fn tip(&self, i: usize, theta: f64) -> [V3; 3] {
        let (s, c) = theta.sin_cos();
        let (r, d) = (self.crank, self.dir[i]);
        [
            add(self.base[i], scale(add(scale(d, c), scale(Z, s)), r)),
            scale(add(scale(d, -s), scale(Z, c)), r),
            scale(add(scale(d, c), scale(Z, s)), -r),
        ]
    }

    /// The crank angle that puts crank i's rod onto the platform joint at `joint` (closed form); `None` out of reach.
    pub fn crank_angle(&self, i: usize, joint: V3) -> Option<f64> {
        let e = sub(joint, self.base[i]);
        let (a, b) = (dot(e, self.dir[i]), e[2]);
        let c = (dot(e, e) + self.crank * self.crank - self.rod * self.rod) / (2.0 * self.crank);
        let rho = a.hypot(b);
        if !(c.abs() <= rho) {
            return None;
        }
        Some(b.atan2(a) + self.branch * (c / rho).acos())
    }

    /// The legs at crank angles `q` with the platform at `p` turned by `r`.
    pub fn legs_at(&self, q: &[f64], p: V3, r: &M3) -> [Leg; 6] {
        std::array::from_fn(|i| {
            let [tip, t, t2] = self.tip(i, q[i]);
            let b = mul(r, self.anchors[i]);
            let joint = add(p, b);
            let d = sub(joint, tip);
            let u = scale(d, 1.0 / norm(d));
            Leg { tip, t, t2, joint, b, u, arm: dot(u, t) }
        })
    }

    /// Crank angles for a tool pose (closed form); `false` where a rod cannot reach.
    pub fn inverse(&self, tool: &[f64], q: &mut [f64]) -> bool {
        let (p, r) = ([tool[0], tool[1], tool[2]], Self::rotation(tool));
        for i in 0..6 {
            match self.crank_angle(i, add(p, mul(&r, self.anchors[i]))) {
                Some(theta) => q[i] = theta,
                None => return false,
            }
        }
        true
    }

    /// The legs at a tool pose, by the inverse; `None` out of reach.
    pub fn legs(&self, tool: &[f64]) -> Option<[Leg; 6]> {
        let mut q = [0.0; 6];
        self.inverse(tool, &mut q).then(|| self.legs_at(&q, [tool[0], tool[1], tool[2]], &Self::rotation(tool)))
    }

    /// A (6 x 6, row by row): row i (u_i, b_i x u_i), the rate of rod i's length over the platform's twist.
    pub fn constraint(legs: &[Leg; 6]) -> [f64; 36] {
        let mut a = [0.0; 36];
        for (i, leg) in legs.iter().enumerate() {
            let m = cross(leg.b, leg.u);
            a[i * 6..i * 6 + 6].copy_from_slice(&[leg.u[0], leg.u[1], leg.u[2], m[0], m[1], m[2]]);
        }
        a
    }

    /// The inverse Jacobian (6 x 6, row by row): the crank rates per platform twist, theta' = diag(m)^-1 A xi.
    pub fn inverse_jacobian(legs: &[Leg; 6]) -> Vec<f64> {
        let mut j = Self::constraint(legs);
        for (i, leg) in legs.iter().enumerate() {
            j[i * 6..i * 6 + 6].iter_mut().for_each(|v| *v /= leg.arm);
        }
        j.to_vec()
    }

    /// The platform's centre and rotation for crank angles, by Newton's method from `home`; `None` where it does not
    /// converge.
    pub fn pose(&self, q: &[f64]) -> Option<(V3, M3)> {
        let dq: [f64; 6] = std::array::from_fn(|c| q[c] - self.home_q[c]);
        let guess: [f64; 6] = std::array::from_fn(|r| (0..6).map(|c| self.home_k[r * 6 + c] * dq[c]).sum());
        // Far from home (or out of reach of the guess) Newton starts from home itself.
        let ok = guess.iter().all(|v| v.is_finite() && v.abs() < 0.5);
        let guess = if ok { guess } else { [0.0; 6] };
        let p = add([self.home[0], self.home[1], self.home[2]], [guess[0], guess[1], guess[2]]);
        let r = matmul(&exp([guess[3], guess[4], guess[5]]), &Self::rotation(&self.home));
        self.pose_from(q, p, r)
    }

    /// `pose`'s Newton from the platform at `p` turned by `r`.
    fn pose_from(&self, q: &[f64], mut p: V3, mut r: M3) -> Option<(V3, M3)> {
        for _ in 0..40 {
            let legs = self.legs_at(q, p, &r);
            let miss: [f64; 6] = std::array::from_fn(|i| self.rod - norm(sub(legs[i].joint, legs[i].tip)));
            let worst = miss.iter().fold(0.0f64, |m, v| m.max(v.abs()));
            if !worst.is_finite() {
                return None;
            }
            let step = Lu::of(&Self::constraint(&legs))?.solve(miss);
            p = add(p, [step[0], step[1], step[2]]);
            r = matmul(&exp([step[3], step[4], step[5]]), &r);
            // Within 1e-12 m one more step: Newton's square takes the rest to the rounding (stopping at 1e-13 left some
            // poses unconverged by their rounding, stopping at 1e-12 left 1e-13 of noise in the pose, which the
            // controller's differences of the forward kinematics amplify).
            if worst < 1e-12 {
                return Some((p, r));
            }
        }
        None
    }

    /// The tool pose for crank angles (`pose`); NaN where it does not converge.
    pub fn forward(&self, q: &[f64], tool: &mut [f64]) {
        match self.pose(q) {
            Some((p, r)) => {
                let rv = log(&r);
                tool[..6].copy_from_slice(&[p[0], p[1], p[2], rv[0], rv[1], rv[2]]);
            }
            None => tool.iter_mut().for_each(|v| *v = f64::NAN),
        }
    }

    /// The condition number of the inverse Jacobian with the turns weighed by the platform radius (the platform
    /// joints' speed for w); infinite out of reach and where one crank is at its dead centre (all six at once scale the
    /// rows alike, which the dead centre's margin in `unreachable` catches).
    pub fn condition(&self, tool: &[f64]) -> f64 {
        let Some(legs) = self.legs(tool) else { return f64::INFINITY };
        let mut j = Self::inverse_jacobian(&legs);
        for i in 0..6 {
            for c in 3..6 {
                j[i * 6 + c] /= self.platform_radius;
            }
        }
        if !j.iter().all(|v| v.is_finite()) {
            return f64::INFINITY;
        }
        condition_number(&j, 6, 6)
    }

    /// Per leg: the rod's angle from the crank's tangent (90 degrees at the dead centre), its angle out of the crank's
    /// plane at the crank's ball joint, and its angle against the platform's normal at the platform's, rad.
    pub fn angles(&self, legs: &[Leg; 6], r: &M3) -> [[f64; 3]; 6] {
        let normal = mul(r, Z);
        std::array::from_fn(|i| {
            let leg = &legs[i];
            [
                (leg.arm.abs() / self.crank).clamp(0.0, 1.0).acos(),
                dot(leg.u, self.axis[i]).abs().clamp(0.0, 1.0).asin(),
                dot(leg.u, normal).clamp(-1.0, 1.0).acos(),
            ]
        })
    }

    /// The least distance between the axes of two rods, m.
    pub fn clearance(legs: &[Leg; 6]) -> f64 {
        let mut least = f64::INFINITY;
        for i in 0..6 {
            for j in i + 1..6 {
                least = least.min(segment_distance(legs[i].tip, legs[i].joint, legs[j].tip, legs[j].joint));
            }
        }
        least
    }
}

/// The platform with what it carries, as one rigid body in the platform frame: mass, first moment (kg m) and inertia
/// about the platform centre.
#[derive(Debug, Clone, Copy)]
struct Body {
    mass: f64,
    moment: V3,
    inertia: M3,
}

/// A body's inertia about a point `c` from its centre of mass: I_cg + m (|c|^2 1 - c c^T).
fn shifted(diag: V3, m: f64, c: V3) -> M3 {
    std::array::from_fn(|i| {
        std::array::from_fn(|j| if i == j { diag[i] + m * dot(c, c) } else { 0.0 } - m * c[i] * c[j])
    })
}

impl Body {
    fn plus(&self, other: &Body) -> Body {
        Body {
            mass: self.mass + other.mass,
            moment: add(self.moment, other.moment),
            inertia: std::array::from_fn(|i| std::array::from_fn(|j| self.inertia[i][j] + other.inertia[i][j])),
        }
    }
}

/// The mechanism: geometry, platform, payload, cranks and rods.
#[derive(Clone)]
pub struct RotaryPlatform {
    pub geo: Geometry,
    platform: Body,
    payload: Body,
    attached: bool,
    /// The rods' shares the dynamics counts with the platform: a third of each rod's mass at its platform joint for the
    /// inertia (the m/3 H^T H of its upper end and middle), a half for the weight (m/6 and m/3).
    rods_inertia: Body,
    rods_weight: Body,
    pub crank_mass: f64,
    pub crank_cg: f64,
    pub crank_inertia: f64,
    pub rod_mass: f64,
    g: f64,
    /// Rating limits: the rod's angle from the crank's tangent (rad), the rods' buckling load over the safety factor
    /// (N); the ball joints' from the working range.
    dead_centre_limit: f64,
    buckling_limit: f64,
    /// The grid point's pose the dynamics' Newton starts from (`warm_motion`).
    last: LastFound,
}

/// What the platform's motion is at crank angles and rates: the pose, the legs, A^-1, K = A^-1 diag(m), the twist and
/// its acceleration at theta'' = 0.
struct Motion {
    tool: [f64; 6],
    r: M3,
    legs: [Leg; 6],
    k: [f64; 36],
    xi: [f64; 6],
    xi0: [f64; 6],
}

/// The pose at a point of the crank angles' grid (`ANCHOR_STEP`) the dynamics' Newton starts from: the grid point's
/// index, its crank angles, the platform's centre and rotation, and K there.
#[derive(Debug, Clone, Copy)]
struct Found {
    key: [i64; 6],
    q: [f64; 6],
    p: V3,
    r: M3,
    k: [f64; 36],
}

/// The grid of crank angles the dynamics' Newton starts from, rad: from the nearest point, so the pose is a function of
/// the angles alone, whatever was called before (a run, its replay and a perturbed run give the same numbers).
const ANCHOR_STEP: f64 = 1e-3;

/// The last grid point's pose behind a lock (`Mechanism` is `Send`, and the integrator calls `dynamics` through `&self`).
#[derive(Debug, Default)]
struct LastFound(std::sync::Mutex<Option<Found>>);

impl LastFound {
    fn get(&self) -> Option<Found> {
        self.0.lock().ok().and_then(|f| *f)
    }
    fn set(&self, found: Option<Found>) {
        if let Ok(mut f) = self.0.lock() {
            *f = found;
        }
    }
}

impl Clone for LastFound {
    fn clone(&self) -> Self {
        Self(std::sync::Mutex::new(self.get()))
    }
}

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    let (machine, _) = RotaryPlatform::from_config(cfg)?;
    sim_components::base::mount(cfg, machine)
}

fn vec3(cfg: &Config, key: &str, positive: bool) -> Result<V3, ConfigError> {
    let v = cfg.f64s(key)?;
    if v.len() != 3 || v.iter().any(|x| !x.is_finite() || (positive && !(*x > 0.0))) {
        return Err(ConfigError::Invalid(key.into()));
    }
    Ok([v[0], v[1], v[2]])
}

impl RotaryPlatform {
    /// The machine, and its mechanism alone for the tests.
    pub fn from_config(cfg: &Config) -> Result<(Box<dyn Machine>, RotaryPlatform), ConfigError> {
        let out = match cfg.get("crank.branch")?.as_str() {
            Some("out") => true,
            Some("in") => false,
            _ => return Err(ConfigError::Invalid("crank.branch".into())),
        };
        let (crank, rod) = (cfg.positive("crank.length_m")?, cfg.positive("rod.length_m")?);
        let mut geo = Geometry::new(
            cfg.positive("platform.base_radius_m")?,
            cfg.non_negative("platform.base_half_angle_deg")?,
            cfg.positive("platform.platform_radius_m")?,
            cfg.non_negative("platform.platform_half_angle_deg")?,
            cfg.f64("crank.axis_turn_deg")?,
            crank,
            rod,
            out,
        );
        geo.min_margin = cfg.non_negative("workspace.min_dead_centre_margin_deg")?.to_radians();
        geo.max_crank_joint = cfg.positive("workspace.crank_joint_max_angle_deg")?.to_radians();
        geo.max_platform_joint = cfg.positive("workspace.platform_joint_max_angle_deg")?.to_radians();
        geo.min_clearance = cfg.non_negative("workspace.min_rod_clearance_m")?;
        geo.max_condition = cfg.positive("workspace.max_condition_number")?;
        let (min, max) = (cfg.f64s("workspace.tool_min")?, cfg.f64s("workspace.tool_max")?);
        if min.len() != 6 || max.len() != 6 {
            return Err(ConfigError::Invalid("workspace.tool_min".into()));
        }
        if (0..6).any(|k| !(min[k] < max[k])) {
            return Err(ConfigError::Invalid("workspace.tool_max".into()));
        }
        geo.set_home(std::array::from_fn(|k| 0.5 * (min[k] + max[k])));
        let body =
            |mass: f64, cg: V3, diag: V3| Body { mass, moment: scale(cg, mass), inertia: shifted(diag, mass, cg) };
        let platform = body(
            cfg.positive("mass.platform_kg")?,
            vec3(cfg, "mass.platform_cg_m", false)?,
            vec3(cfg, "mass.platform_inertia_kgm2", true)?,
        );
        let payload = if cfg.bool("effects.payload")? {
            body(
                cfg.non_negative("mass.payload_kg")?,
                vec3(cfg, "mass.payload_cg_m", false)?,
                vec3(cfg, "mass.payload_inertia_kgm2", false)?,
            )
        } else {
            body(0.0, [0.0; 3], [0.0; 3])
        };
        // Euler's case 2 (both ends free to turn): F = pi^2 E I / L^2 of the tube's section.
        let (d_out, wall) = (cfg.positive("rod.outer_diameter_m")?, cfg.positive("rod.wall_m")?);
        if 2.0 * wall > d_out {
            return Err(ConfigError::Invalid("rod.wall_m".into()));
        }
        let second_moment = PI / 64.0 * (d_out.powi(4) - (d_out - 2.0 * wall).powi(4));
        let euler = PI * PI * cfg.positive("rod.youngs_modulus_pa")? * second_moment / (rod * rod);
        let rod_mass = cfg.non_negative("rod.mass_kg")?;
        let at_joints =
            |m: f64| geo.anchors.iter().fold(body(0.0, [0.0; 3], [0.0; 3]), |sum, a| sum.plus(&body(m, *a, [0.0; 3])));
        let (rods_inertia, rods_weight) = (at_joints(rod_mass / 3.0), at_joints(rod_mass / 2.0));
        let mech = RotaryPlatform {
            geo,
            platform,
            payload,
            attached: true,
            rods_inertia,
            rods_weight,
            crank_mass: cfg.non_negative("crank.mass_kg")?,
            crank_cg: cfg.non_negative("crank.cg_m")?,
            crank_inertia: cfg.positive("crank.inertia_kgm2")?,
            rod_mass,
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            dead_centre_limit: FRAC_PI_2 - cfg.non_negative("ratings.crank_dead_centre_margin_deg")?.to_radians(),
            buckling_limit: euler / cfg.positive("ratings.rod_buckling_safety_factor")?,
            last: LastFound::default(),
        };
        let joint = RotaryJoint::from_config(cfg)?;
        let axes = (0..6)
            .map(|k| {
                let unit = if k < 3 { "m" } else { "rad" };
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
        let specs = (0..6)
            .map(|k| JointSpec {
                drive: joint.clone(),
                id: MOTOR_IDS[k],
                key: JOINT_KEYS[k],
                gear_lever: None,
                unit: "rad",
            })
            .collect();
        let motors = (0..6).map(|k| MotorInfo { id: MOTOR_IDS[k], key: MOTOR_KEYS[k] }).collect();
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

    /// The platform with the payload as it is attached.
    fn body(&self) -> Body {
        if self.attached { self.platform.plus(&self.payload) } else { self.platform }
    }

    /// The Euler buckling load of a rod over its safety factor, N.
    pub fn buckling_limit(&self) -> f64 {
        self.buckling_limit
    }

    /// The platform's motion at crank angles q and rates qd; `None` where the forward kinematics fails or A is
    /// singular.
    fn motion(&self, q: &[f64], qd: &[f64]) -> Option<Motion> {
        self.motion_at(q, qd, self.geo.pose(q)?)
    }

    /// `motion` for the dynamics, which runs four times a sub-step: Newton starts from the pose at the nearest point of
    /// the `ANCHOR_STEP` grid moved by K (q - q_grid), which takes it two steps instead of about four from `forward`'s
    /// guess; as `motion` where that does not converge. The grid point's pose (by `motion`) is kept until the cranks
    /// leave its cell. The pose differs from `forward`'s by Newton's tolerance (1e-12 m on the rods, and one step more).
    fn warm_motion(&self, q: &[f64], qd: &[f64]) -> Option<Motion> {
        let key: [i64; 6] = std::array::from_fn(|i| (q[i] / ANCHOR_STEP).round() as i64);
        let anchor = self.last.get().filter(|f| f.key == key).or_else(|| {
            let g: [f64; 6] = std::array::from_fn(|i| key[i] as f64 * ANCHOR_STEP);
            let (p, r) = self.geo.pose(&g)?;
            let found = Found { key, q: g, p, r, k: self.motion_at(&g, &[0.0; 6], (p, r))?.k };
            self.last.set(Some(found));
            Some(found)
        });
        let warm = anchor.and_then(|f| {
            let xi: [f64; 6] = std::array::from_fn(|r| (0..6).map(|c| f.k[r * 6 + c] * (q[c] - f.q[c])).sum());
            self.geo.pose_from(q, add(f.p, [xi[0], xi[1], xi[2]]), matmul(&exp([xi[3], xi[4], xi[5]]), &f.r))
        });
        let pose = match warm {
            Some(pose) => pose,
            None => self.geo.pose(q)?,
        };
        self.motion_at(q, qd, pose)
    }

    /// `motion` with the platform's centre and rotation known.
    fn motion_at(&self, q: &[f64], qd: &[f64], (p, r): (V3, M3)) -> Option<Motion> {
        let rv = log(&r);
        let tool = [p[0], p[1], p[2], rv[0], rv[1], rv[2]];
        let legs = self.geo.legs_at(q, p, &r);
        let lu = Lu::of(&Geometry::constraint(&legs))?;
        let mut k = [0.0; 36];
        for c in 0..6 {
            let col = lu.solve(std::array::from_fn(|i| if i == c { legs[c].arm } else { 0.0 }));
            (0..6).for_each(|r| k[r * 6 + c] = col[r]);
        }
        let xi = lu.solve(std::array::from_fn(|i| legs[i].arm * qd[i]));
        let (v, w) = ([xi[0], xi[1], xi[2]], [xi[3], xi[4], xi[5]]);
        let s: [f64; 6] = std::array::from_fn(|i| {
            let leg = &legs[i];
            let slip = sub(add(v, cross(w, leg.b)), scale(leg.t, qd[i]));
            dot(leg.u, leg.t2) * qd[i] * qd[i] - dot(slip, slip) / self.geo.rod - dot(leg.u, cross(w, cross(w, leg.b)))
        });
        let xi0 = lu.solve(s);
        Some(Motion { tool, r, legs, k, xi, xi0 })
    }

    /// The platform joint's velocity per crank rate (3 x 6, row by row): K_v + K_w x b.
    fn joint_jacobian(m: &Motion, b: V3) -> [[f64; 6]; 3] {
        let mut out = [[0.0; 6]; 3];
        for c in 0..6 {
            let v = [m.k[c], m.k[6 + c], m.k[12 + c]];
            let w = [m.k[18 + c], m.k[24 + c], m.k[30 + c]];
            let col = add(v, cross(w, b));
            (0..3).for_each(|r| out[r][c] = col[r]);
        }
        out
    }

    /// The body's mass matrix over the twist (6 x 6), its speed terms c and its weight G, in the base frame.
    /// `body` moves, `weight` weighs (the same but where the rods' shares differ).
    fn twist_terms(&self, m: &Motion, body: &Body, weight: &Body) -> ([f64; 36], [f64; 6], [f64; 6]) {
        let s = mul(&m.r, body.moment);
        let inertia = matmul(&matmul(&m.r, &body.inertia), &transpose(&m.r));
        let mut mx = [0.0; 36];
        for i in 0..3 {
            mx[i * 6 + i] = body.mass;
            for j in 0..3 {
                mx[(3 + i) * 6 + 3 + j] = inertia[i][j];
            }
        }
        // [S]x in the lower left, -[S]x in the upper right.
        let sx = [[0.0, -s[2], s[1]], [s[2], 0.0, -s[0]], [-s[1], s[0], 0.0]];
        for i in 0..3 {
            for j in 0..3 {
                mx[(3 + i) * 6 + j] = sx[i][j];
                mx[i * 6 + 3 + j] = -sx[i][j];
            }
        }
        let w = [m.xi[3], m.xi[4], m.xi[5]];
        let (f, t) = (cross(w, cross(w, s)), cross(w, mul(&inertia, w)));
        let gw = cross(mul(&m.r, weight.moment), [0.0, 0.0, -self.g]);
        (mx, [f[0], f[1], f[2], t[0], t[1], t[2]], [0.0, 0.0, -weight.mass * self.g, gw[0], gw[1], gw[2]])
    }

    /// The force along each rod (compression positive, N) at crank angles, rates and accelerations: A^T f = W, the
    /// platform's required wrench with the rods' upper sixth and half their middle carried at the platform joints.
    pub fn rod_forces(&self, q: &[f64], qd: &[f64], qdd: &[f64]) -> Option<[f64; 6]> {
        let m = self.motion(q, qd)?;
        let xi_acc: [f64; 6] = std::array::from_fn(|r| m.xi0[r] + (0..6).map(|c| m.k[r * 6 + c] * qdd[c]).sum::<f64>());
        let body = self.body();
        let (mx, c, g) = self.twist_terms(&m, &body, &body);
        let mut wrench: [f64; 6] =
            std::array::from_fn(|r| (0..6).map(|k| mx[r * 6 + k] * xi_acc[k]).sum::<f64>() + c[r] - g[r]);
        let (v_acc, w_acc, w) =
            ([xi_acc[0], xi_acc[1], xi_acc[2]], [xi_acc[3], xi_acc[4], xi_acc[5]], [m.xi[3], m.xi[4], m.xi[5]]);
        for (i, leg) in m.legs.iter().enumerate() {
            let a_high = add(add(v_acc, cross(w_acc, leg.b)), cross(w, cross(w, leg.b)));
            let a_low = add(scale(leg.t, qdd[i]), scale(leg.t2, qd[i] * qd[i]));
            let a_mid = scale(add(a_low, a_high), 0.5);
            let need = |a: V3| [a[0], a[1], a[2] + self.g];
            let f = add(scale(need(a_high), self.rod_mass / 6.0), scale(need(a_mid), self.rod_mass / 3.0));
            let t = cross(leg.b, f);
            for k in 0..3 {
                wrench[k] += f[k];
                wrench[3 + k] += t[k];
            }
        }
        let a = Geometry::constraint(&m.legs);
        let at: Vec<f64> = (0..36).map(|k| a[(k % 6) * 6 + k / 6]).collect();
        let f = solve(&at, &wrench)?;
        Some(std::array::from_fn(|i| f[i]))
    }
}

impl Mechanism for RotaryPlatform {
    fn dynamics(&self, q: &[f64], qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        let Some(m) = self.warm_motion(q, qd) else {
            mass.fill(f64::NAN);
            bias.fill(f64::NAN);
            return;
        };
        // Each rod's three points (m/6 at its tip C with Jacobian L, m/6 at its joint P with H, 2m/3 at its middle with
        // (L + H) / 2) give m/3 (L^T L + H^T H) + m/6 (L^T H + H^T L): the H^T H part a point of m/3 on the platform, the
        // L^T L part m/3 r^2 on the crank (L only in crank i's column, t_i). Their accelerations at theta'' = 0 likewise:
        // m/3 (L^T a_C + H^T a_P) + m/6 (L^T a_P + H^T a_C), t . a_C = t . t2 theta'^2 = 0. Their weight m/2 (L_z + H_z).
        let (inertial, weighing) = (self.body().plus(&self.rods_inertia), self.body().plus(&self.rods_weight));
        let (mx, c, g) = self.twist_terms(&m, &inertial, &weighing);
        let need: [f64; 6] =
            std::array::from_fn(|r| (0..6).map(|k| mx[r * 6 + k] * m.xi0[k]).sum::<f64>() + c[r] - g[r]);
        // M_xi K, then K^T of it.
        let mk: [f64; 36] = std::array::from_fn(|rc| (0..6).map(|k| mx[rc / 6 * 6 + k] * m.k[k * 6 + rc % 6]).sum());
        for i in 0..6 {
            bias[i] = (0..6).map(|r| m.k[r * 6 + i] * need[r]).sum();
            for j in 0..6 {
                mass[i * 6 + j] = (0..6).map(|r| m.k[r * 6 + i] * mk[r * 6 + j]).sum();
            }
        }
        let (rod, w) = (self.rod_mass, [m.xi[3], m.xi[4], m.xi[5]]);
        for (i, leg) in m.legs.iter().enumerate() {
            let high = Self::joint_jacobian(&m, leg.b);
            let a_high = add(add([m.xi0[0], m.xi0[1], m.xi0[2]], cross([m.xi0[3], m.xi0[4], m.xi0[5]], leg.b)), {
                cross(w, cross(w, leg.b))
            });
            let a_low = scale(leg.t2, qd[i] * qd[i]);
            for c in 0..6 {
                let col = [high[0][c], high[1][c], high[2][c]];
                let cross_term = rod / 6.0 * dot(leg.t, col);
                mass[i * 6 + c] += cross_term;
                mass[c * 6 + i] += cross_term;
                bias[c] += rod / 6.0 * dot(col, a_low);
            }
            mass[i * 6 + i] += self.crank_inertia + rod / 3.0 * dot(leg.t, leg.t);
            bias[i] += rod / 6.0 * dot(leg.t, a_high)
                + self.g * (self.crank_mass * self.crank_cg + 0.5 * rod * self.geo.crank) * q[i].cos();
        }
    }

    /// The body at its centre of mass with its inertia about it, the rods' three points each, the cranks turning about
    /// their axes, each rotor spinning along its crank's axis.
    fn frame_momentum(&self, q: &[f64], qd: &[f64], rotor_spin: &[f64]) -> Option<sim_core::FrameMomentum> {
        use sim_components::base::{add_point, add_spin, add_turning};
        let m = self.motion(q, qd)?;
        let mut f = sim_core::FrameMomentum { gravity: [0.0, 0.0, -self.g], ..Default::default() };
        let body = self.body();
        let (p, v, w) = ([m.tool[0], m.tool[1], m.tool[2]], [m.xi[0], m.xi[1], m.xi[2]], [m.xi[3], m.xi[4], m.xi[5]]);
        if body.mass > 0.0 {
            let rc = scale(mul(&m.r, body.moment), 1.0 / body.mass);
            add_point(&mut f, body.mass, add(p, rc), add(v, cross(w, rc)));
            let about_p = matmul(&matmul(&m.r, &body.inertia), &transpose(&m.r));
            let own: M3 = std::array::from_fn(|i| {
                std::array::from_fn(|j| {
                    about_p[i][j] - body.mass * (if i == j { dot(rc, rc) } else { 0.0 } - rc[i] * rc[j])
                })
            });
            let spin = mul(&own, w);
            (0..3).for_each(|k| f.angular[k] += spin[k]);
        }
        for (i, leg) in m.legs.iter().enumerate() {
            let (v_low, v_high) = (scale(leg.t, qd[i]), add(v, cross(w, leg.b)));
            add_point(&mut f, self.rod_mass / 6.0, leg.tip, v_low);
            add_point(&mut f, self.rod_mass / 6.0, leg.joint, v_high);
            add_point(
                &mut f,
                2.0 * self.rod_mass / 3.0,
                scale(add(leg.tip, leg.joint), 0.5),
                scale(add(v_low, v_high), 0.5),
            );
            let r = self.geo.crank;
            let dir = scale(sub(leg.tip, self.geo.base[i]), 1.0 / r);
            add_turning(
                &mut f,
                self.geo.base[i],
                dir,
                scale(leg.t, 1.0 / r),
                qd[i],
                self.crank_inertia,
                self.crank_mass * self.crank_cg,
            );
            add_spin(&mut f, self.geo.axis[i], 1.0, rotor_spin[i]);
        }
        Some(f)
    }

    fn potential(&self, q: &[f64]) -> f64 {
        let mut tool = [0.0; 6];
        self.geo.forward(q, &mut tool);
        let r = Geometry::rotation(&tool);
        let legs = self.geo.legs_at(q, [tool[0], tool[1], tool[2]], &r);
        let body = self.body();
        let rods: f64 = legs.iter().map(|l| 0.5 * self.rod_mass * (l.tip[2] + l.joint[2])).sum();
        let cranks: f64 = q[..6].iter().map(|t| self.crank_mass * self.crank_cg * t.sin()).sum();
        self.g * (body.mass * tool[2] + mul(&r, body.moment)[2] + rods + cranks)
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
    /// Out of reach is told by the joint machine (`inverse`); here a crank too near its dead centre, a ball joint
    /// beyond its angle, two rods too near each other, the kinematics near a singularity.
    fn unreachable(&self, tool: &[f64], _slack: f64) -> Option<&'static str> {
        let legs = self.geo.legs(tool)?;
        let angles = self.geo.angles(&legs, &Geometry::rotation(tool));
        if angles.iter().any(|a| !(FRAC_PI_2 - a[0] >= self.geo.min_margin)) {
            return Some("error-rss-dead-centre");
        }
        if angles.iter().any(|a| !(a[1] <= self.geo.max_crank_joint && a[2] <= self.geo.max_platform_joint)) {
            return Some("error-rss-ball-joint");
        }
        if !(Geometry::clearance(&legs) >= self.geo.min_clearance) {
            return Some("error-rss-rods-cross");
        }
        if !(self.geo.condition(tool) <= self.geo.max_condition) {
            return Some("error-rss-singularity");
        }
        None
    }
    fn condition(&self, tool: &[f64]) -> Option<f64> {
        Some(self.geo.condition(tool))
    }
    fn tool_is_joints(&self) -> bool {
        false
    }

    /// Per leg: the rod's angle from the crank's tangent, its force and compression, its angles at both ball joints;
    /// then the condition number at the pose.
    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out = Vec::new();
        for i in 0..6 {
            let id = MOTOR_IDS[i];
            out.push(SignalInfo::new(
                format!("crank.{id}.dead_centre_angle"),
                JOINT_KEYS[i],
                "quantity-dead-centre-angle",
                "rad",
            ));
            out.push(SignalInfo::new(format!("rod.{id}.force"), JOINT_KEYS[i], "quantity-rod-force", "N"));
            out.push(SignalInfo::new(format!("rod.{id}.compression"), JOINT_KEYS[i], "quantity-rod-compression", "N"));
            out.push(SignalInfo::new(
                format!("crank.{id}.ball_joint_angle"),
                JOINT_KEYS[i],
                "quantity-crank-ball-joint-angle",
                "rad",
            ));
            out.push(SignalInfo::new(
                format!("platform.{id}.ball_joint_angle"),
                JOINT_KEYS[i],
                "quantity-platform-ball-joint-angle",
                "rad",
            ));
        }
        out.push(SignalInfo::new("kinematics.condition", "group-platform", "quantity-condition-number", ""));
        out
    }
    fn signals(&self, q: &[f64], qd: &[f64], qdd: &[f64], _force: &[f64], out: &mut Vec<f64>) {
        let Some(m) = self.motion(q, qd) else {
            out.extend(std::iter::repeat_n(f64::NAN, 31));
            return;
        };
        let angles = self.geo.angles(&m.legs, &m.r);
        let forces = self.rod_forces(q, qd, qdd).unwrap_or([f64::NAN; 6]);
        for i in 0..6 {
            out.extend([angles[i][0], forces[i], forces[i].max(0.0), angles[i][1], angles[i][2]]);
        }
        out.push(self.geo.condition(&m.tool));
    }
    /// Per leg: the dead centre's margin, the rod's buckling, both ball joints' angles.
    fn rating_checks(&self) -> Vec<RatingCheck> {
        (0..6)
            .flat_map(|i| {
                let id = MOTOR_IDS[i];
                [
                    ("crank-dead-centre", format!("crank.{id}.dead_centre_angle"), self.dead_centre_limit),
                    ("rod-buckling", format!("rod.{id}.compression"), self.buckling_limit),
                    ("crank-ball-joint", format!("crank.{id}.ball_joint_angle"), self.geo.max_crank_joint),
                    ("platform-ball-joint", format!("platform.{id}.ball_joint_angle"), self.geo.max_platform_joint),
                ]
                .map(|(check, signal, limit)| RatingCheck {
                    check,
                    signal,
                    motor: Some(i),
                    limit,
                    life: false,
                })
            })
            .collect()
    }

    fn collision_parts(&self) -> &'static [&'static str] {
        &["tool", "payload"]
    }

    /// Base and platform as hexagons through their joints, each crank and rod, the motor with its gearbox behind each
    /// crank along its axis, the payload at its centre of mass.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene) {
        let mut tool = [0.0; 6];
        self.geo.forward(q, &mut tool);
        if !tool.iter().all(|v| v.is_finite()) {
            return;
        }
        let r = Geometry::rotation(&tool);
        let p = [tool[0], tool[1], tool[2]];
        let legs = self.geo.legs_at(q, p, &r);
        for i in 0..6 {
            let next = (i + 1) % 6;
            out.parts.push(Part::new(
                "base",
                Shape::Segment { to: self.geo.base[next], width: 0.04 },
                self.geo.base[i],
                [0.55, 0.57, 0.6],
            ));
            out.parts.push(Part::new(
                "platform",
                Shape::Segment { to: legs[next].joint, width: 0.04 },
                legs[i].joint,
                [0.3, 0.45, 0.7],
            ));
        }
        for (i, leg) in legs.iter().enumerate() {
            let (b, k) = (self.geo.base[i], self.geo.axis[i]);
            out.parts.push(Part::new("crank", Shape::Segment { to: leg.tip, width: 0.04 }, b, [0.45, 0.47, 0.5]));
            out.parts.push(Part::new("rod", Shape::Segment { to: leg.joint, width: 0.02 }, leg.tip, [0.85, 0.86, 0.9]));
            out.parts.push(Part::new(
                "gearbox",
                Shape::Segment { to: add(b, scale(k, -0.12)), width: 0.08 },
                add(b, scale(k, -0.03)),
                [0.25, 0.25, 0.28],
            ));
            out.parts.push(Part::new(
                sim_core::scene::MOTOR_PART,
                Shape::Segment { to: add(b, scale(k, -0.25)), width: 0.09 },
                add(b, scale(k, -0.12)),
                [0.9, 0.45, 0.1],
            ));
            out.parts.push(
                Part::new("rotor-bar", Shape::Box { size: [0.07, 0.004, 0.012] }, add(b, scale(k, -0.27)), [0.1; 3])
                    .turned(rotors[i]),
            );
            out.labels.push(Label {
                pos: add(b, add(scale(k, -0.3), [0.0, 0.0, -0.05])),
                owner_key: MOTOR_KEYS[i],
                value: rotor_speeds[i] * 60.0 / TAU,
                unit_key: "unit-rpm",
                decimals: 0,
            });
            out.labels.push(Label {
                pos: add(leg.tip, [0.0, 0.0, 0.05]),
                owner_key: JOINT_KEYS[i],
                value: q[i].to_degrees(),
                unit_key: "unit-deg",
                decimals: 1,
            });
        }
        out.parts.push(Part::new(
            "tool",
            Shape::Box { size: [0.1, 0.1, 0.03] },
            add(p, mul(&r, [0.0, 0.0, 0.015])),
            [0.2, 0.22, 0.26],
        ));
        if self.attached && self.payload.mass > 0.0 {
            let at = scale(self.payload.moment, 1.0 / self.payload.mass);
            out.parts.push(Part::new(
                "payload",
                Shape::Box { size: [0.3, 0.3, 0.3] },
                add(p, mul(&r, at)),
                [0.2, 0.7, 0.3],
            ));
        }
        out.labels.push(Label {
            pos: add(p, [0.0, 0.0, 0.5]),
            owner_key: "axis-z",
            value: p[2],
            unit_key: "unit-m",
            decimals: 3,
        });
    }
    fn scene_point(&self, tool: &[f64]) -> Option<[f64; 3]> {
        Some([tool[0], tool[1], tool[2]])
    }
}

#[cfg(test)]
mod tests;
