//! Hexapod (Stewart-Gough platform): six electric cylinders between a base and a moving platform, each a ball screw
//! driven by its own servo motor through a coupling. Six degrees of freedom: the tool pose is the platform centre x, y,
//! z (m) and the platform's orientation as a rotation vector rx, ry, rz (rad, axis times angle, in the base frame).
//!
//! Geometry (generic proportions, `[hexapod]`): the base joints B_i on a circle of `base_radius_m` in pairs at
//! 120 k +- `base_half_angle_deg`, the platform joints a_i (platform frame) on a circle of `platform_radius_m` at
//! 120 k +- (60 - `platform_half_angle_deg`), so that each leg runs from a base pair to the neighbouring platform pair
//! and the six legs form the usual triangles. Leg i's length is L_i = `leg.closed_length_m` + q_i with q_i the
//! cylinder's extension, its joint coordinate (0 to `leg.stroke_m`).
//!
//! ```text
//! inverse   P_i = p + R a_i,  d_i = P_i - B_i,  L_i = |d_i|,  q_i = L_i - L_closed                   (closed form)
//! twist     xi = (v, w): q_i' = u_i . v + (b_i x u_i) . w,  u_i = d_i / L_i,  b_i = R a_i,  rows of J
//! forward   Newton on the twist: xi = J^-1 (q - q(x)), p += v, R = exp(w) R, from the home pose, until |q - q(x)|
//!           < 1e-13 m; refused (NaN) where it does not converge within 40 steps
//! dynamics  M_xi xi' + c + g = J^T f,  M_xi = diag(m 1, R I R^T),  c = (0, w x I_w w),  g = (0, 0, m g, 0, 0, 0)
//!           q'' = J xi' + a0,  a0_i = (|w_i|^2 - (u_i . w_i)^2) / L_i + u_i . (w x (w x b_i)),  w_i = v + w x b_i
//!           so in leg space: M_q = J^-T M_xi J^-1,  h = J^-T (c + g - M_xi J^-1 a0),  M_q q'' + h = f
//! ```
//!
//! f_i is the force leg i pushes the platform with: what its screw passes on (`chain::JointMachine`, one
//! `joint::RotaryJoint` with a `transmission::BallScrew` per leg: the screw and nut stiffness, the coupling, backlash,
//! friction in the screw and the rod's guide, the motor). Each leg's mass is split between its ends: the share
//! `leg.platform_share` moves with the platform joint (a point mass there, in the platform's mass and inertia), the rest
//! sits at the base joint and does not move.
//! shortcut: the legs' turning about their base joints (their inertia about it) is left out; add it where the legs
//! weigh as much as the platform.
//! shortcut: the payload is a point mass at the platform centre (no inertia of its own); give it an inertia where a
//! large workpiece turns with the platform.

use sim_components::chain::{
    JOINT_KEYS, JointMachine, JointSpec, Lu, MOTOR_IDS, MOTOR_KEYS, Mechanism, condition_number, solve,
};
use sim_components::joint::RotaryJoint;
use sim_components::mech::{HardStop, ShaftFriction, ThermalFeedback};
use sim_components::rigid::{M3, V3, about, add, cross, dot, matmul, mul};
use sim_components::transmission::BallScrew;
use sim_core::{
    AxisInfo, Config, ConfigError, Label, Machine, MachineEntry, MotorInfo, Part, RatingCheck, Scene, Shape, SignalInfo,
};
use std::f64::consts::{PI, TAU};

pub static ENTRY: MachineEntry = MachineEntry {
    id: "hexapod",
    key: "machine-hexapod",
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

/// Loss-free rigid and elastic models, with and without the payload, all without gravity: held by constant leg forces,
/// as the suite holds it after its pulse, the platform under its weight is unstable (pushing legs of constant force:
/// d^2/dz^2 of m g z - sum f_i L_i = -sum f_i (1 - u_iz^2) / L_i < 0), and it runs away from the pose, a real property
/// that the position loops hold and not a numerical one. Gravity's part is checked on the default config (static
/// hold, loss budget) and by the hand-calculated leg forces.
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

fn transpose(m: &M3) -> M3 {
    std::array::from_fn(|i| std::array::from_fn(|j| m[j][i]))
}

/// The rotation of a rotation vector.
pub fn exp(r: V3) -> M3 {
    let angle = norm(r);
    if angle == 0.0 { about([0.0, 0.0, 1.0], 0.0) } else { about(r.map(|v| v / angle), angle) }
}

/// The rotation vector (axis times angle, angle up to pi) of a rotation matrix.
pub fn log(r: &M3) -> V3 {
    let vee = [r[2][1] - r[1][2], r[0][2] - r[2][0], r[1][0] - r[0][1]];
    let cos = ((r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0).clamp(-1.0, 1.0);
    let sin = 0.5 * norm(vee);
    let angle = sin.atan2(cos);
    if angle < 1e-6 {
        // sin a / a = 1 - a^2/6: below 1e-6 rad the correction is under 1e-12 of the angle.
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

/// The inverse of J (rows), column by column; `None` where it is singular.
fn inverse(j: &[[f64; 6]; 6]) -> Option<[[f64; 6]; 6]> {
    Lu::new(j)?.inverse()
}

/// The legs at a platform pose: unit vectors u_i from base to platform joint, lengths L_i, and the platform joints'
/// offsets b_i = R a_i from the platform centre (base frame).
#[derive(Debug, Clone, Copy)]
pub struct Legs {
    pub u: [V3; 6],
    pub length: [f64; 6],
    pub b: [V3; 6],
}

/// The hexapod's geometry and kinematics, apart from masses and drives.
#[derive(Debug, Clone)]
pub struct Geometry {
    /// Base joints (base frame) and platform joints (platform frame).
    pub base: [V3; 6],
    pub anchors: [V3; 6],
    pub platform_radius: f64,
    pub closed: f64,
    pub stroke: f64,
    /// The steepest angle a leg may make with the base's normal at its base joint and with the platform's normal at
    /// its platform joint, rad.
    pub max_joint_angle: f64,
    pub max_condition: f64,
    /// The pose Newton's forward kinematics starts from: the centre of the tool's box.
    pub home: [f64; 6],
}

impl Geometry {
    /// Joints at the azimuths 120 k -+ the base half angle and 120 k -+ (60 - the platform half angle), degrees.
    pub fn new(base_radius: f64, base_half: f64, platform_radius: f64, platform_half: f64) -> Self {
        let at = |radius: f64, deg: f64| [radius * deg.to_radians().cos(), radius * deg.to_radians().sin(), 0.0];
        let base = std::array::from_fn(|i| {
            let side = if i % 2 == 0 { -1.0 } else { 1.0 };
            at(base_radius, 120.0 * (i / 2) as f64 + side * base_half)
        });
        let anchors = std::array::from_fn(|i| {
            let side = if i % 2 == 0 { -1.0 } else { 1.0 };
            at(platform_radius, 120.0 * (i / 2) as f64 + side * (60.0 - platform_half))
        });
        Self {
            base,
            anchors,
            platform_radius,
            closed: 0.0,
            stroke: 1.0,
            max_joint_angle: PI / 2.0,
            max_condition: f64::INFINITY,
            home: [0.0; 6],
        }
    }

    fn rotation(tool: &[f64]) -> M3 {
        exp([tool[3], tool[4], tool[5]])
    }

    /// The legs with the platform at `p` turned by `r`.
    pub fn legs_at(&self, p: V3, r: &M3) -> Legs {
        let mut legs = Legs { u: [[0.0; 3]; 6], length: [0.0; 6], b: [[0.0; 3]; 6] };
        for i in 0..6 {
            let b = mul(r, self.anchors[i]);
            let d = sub(add(p, b), self.base[i]);
            let l = norm(d);
            legs.u[i] = d.map(|v| v / l);
            legs.length[i] = l;
            legs.b[i] = b;
        }
        legs
    }

    pub fn legs(&self, tool: &[f64]) -> Legs {
        self.legs_at([tool[0], tool[1], tool[2]], &Self::rotation(tool))
    }

    /// The Jacobian J (6 x 6, row by row) of the legs' extension rates over the platform's twist (v, w).
    pub fn jacobian(legs: &Legs) -> Vec<f64> {
        Self::jacobian_rows(legs).concat()
    }

    /// `jacobian` by rows.
    fn jacobian_rows(legs: &Legs) -> [[f64; 6]; 6] {
        std::array::from_fn(|i| {
            let m = cross(legs.b[i], legs.u[i]);
            [legs.u[i][0], legs.u[i][1], legs.u[i][2], m[0], m[1], m[2]]
        })
    }

    /// Leg extensions for a tool pose (closed form); `false` where a leg would be of no length.
    pub fn inverse(&self, tool: &[f64], q: &mut [f64]) -> bool {
        let legs = self.legs(tool);
        for i in 0..6 {
            q[i] = legs.length[i] - self.closed;
        }
        q.iter().all(|v| v.is_finite())
    }

    /// The tool pose for leg extensions, by Newton's method from `home`; NaN where it does not converge.
    pub fn forward(&self, q: &[f64], tool: &mut [f64]) {
        let home = ([self.home[0], self.home[1], self.home[2]], Self::rotation(&self.home));
        let Some((p, r, _)) = self.solve_from(q, home.0, home.1) else {
            tool.iter_mut().for_each(|v| *v = f64::NAN);
            return;
        };
        let rv = log(&r);
        tool[..6].copy_from_slice(&[p[0], p[1], p[2], rv[0], rv[1], rv[2]]);
    }

    /// Newton's method on the twist from the platform at `p` turned by `r` until every leg is within 1e-13 m of its
    /// length for `q`: the platform's centre, rotation and its legs there; `None` where it does not converge within 40
    /// steps.
    fn solve_from(&self, q: &[f64], mut p: V3, mut r: M3) -> Option<(V3, M3, Legs)> {
        for _ in 0..40 {
            let legs = self.legs_at(p, &r);
            let miss: [f64; 6] = std::array::from_fn(|i| q[i] + self.closed - legs.length[i]);
            let worst = miss.iter().fold(0.0f64, |m, v| m.max(v.abs()));
            if !worst.is_finite() {
                return None;
            }
            if worst < 1e-13 {
                return Some((p, r, legs));
            }
            let step = Lu::new(&Self::jacobian_rows(&legs)).and_then(|lu| lu.solve(miss))?;
            p = add(p, [step[0], step[1], step[2]]);
            r = matmul(&exp([step[3], step[4], step[5]]), &r);
        }
        None
    }

    /// The steepest leg angle at the joints, rad: against the base's normal and against the platform's.
    pub fn joint_angle(&self, tool: &[f64]) -> f64 {
        let legs = self.legs(tool);
        let normal = mul(&Self::rotation(tool), [0.0, 0.0, 1.0]);
        (0..6)
            .map(|i| legs.u[i][2].clamp(-1.0, 1.0).acos().max(dot(legs.u[i], normal).clamp(-1.0, 1.0).acos()))
            .fold(0.0, f64::max)
    }

    /// The condition number of J with the turns weighed by the platform radius (the platform joints' speed for w).
    pub fn condition(&self, tool: &[f64]) -> f64 {
        let mut j = Self::jacobian(&self.legs(tool));
        for i in 0..6 {
            for c in 3..6 {
                j[i * 6 + c] /= self.platform_radius;
            }
        }
        condition_number(&j, 6, 6)
    }
}

/// The pose at a point of the leg extensions' grid (`ANCHOR_STEP`) the dynamics' Newton starts from: the grid point's
/// index, its leg extensions, the platform's centre and rotation, and J^-1 there.
#[derive(Debug, Clone, Copy)]
struct Found {
    key: [i64; 6],
    q: [f64; 6],
    p: V3,
    r: M3,
    jinv: [[f64; 6]; 6],
}

/// The grid of leg extensions the dynamics' Newton starts from, m: from the nearest point, so the pose is a function
/// of the extensions alone, whatever was called before (a run, its replay and a perturbed run give the same numbers).
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

#[derive(Clone)]
pub struct Hexapod {
    pub geo: Geometry,
    screw: BallScrew,
    /// Platform with the legs' platform shares; payload; inertia about the centre in the platform frame.
    mass: f64,
    payload: f64,
    attached: bool,
    inertia: M3,
    g: f64,
    required_life: Option<f64>,
    /// The grid point's pose the dynamics' Newton starts from (`warm_pose`).
    last: LastFound,
}

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    let (machine, _) = Hexapod::from_config(cfg)?;
    sim_components::base::mount(cfg, machine)
}

impl Hexapod {
    /// The machine, and its mechanism alone for the tests.
    pub fn from_config(cfg: &Config) -> Result<(Box<dyn Machine>, Hexapod), ConfigError> {
        let mut geo = Geometry::new(
            cfg.positive("hexapod.base_radius_m")?,
            cfg.non_negative("hexapod.base_half_angle_deg")?,
            cfg.positive("hexapod.platform_radius_m")?,
            cfg.non_negative("hexapod.platform_half_angle_deg")?,
        );
        geo.closed = cfg.positive("leg.closed_length_m")?;
        geo.stroke = cfg.positive("leg.stroke_m")?;
        geo.max_joint_angle = cfg.positive("workspace.max_joint_angle_deg")?.to_radians();
        geo.max_condition = cfg.positive("workspace.max_condition_number")?;
        let (min, max) = (cfg.f64s("workspace.tool_min")?, cfg.f64s("workspace.tool_max")?);
        if min.len() != 6 || max.len() != 6 {
            return Err(ConfigError::Invalid("workspace.tool_min".into()));
        }
        if (0..6).any(|k| !(min[k] < max[k])) {
            return Err(ConfigError::Invalid("workspace.tool_max".into()));
        }
        geo.home = std::array::from_fn(|k| 0.5 * (min[k] + max[k]));
        let share = cfg.non_negative("leg.platform_share")?;
        if share > 1.0 {
            return Err(ConfigError::Invalid("leg.platform_share".into()));
        }
        let at_joint = share * cfg.non_negative("leg.mass_kg")?;
        let diag = cfg.f64s("mass.platform_inertia_kgm2")?;
        if diag.len() != 3 || diag.iter().any(|v| !(*v > 0.0)) {
            return Err(ConfigError::Invalid("mass.platform_inertia_kgm2".into()));
        }
        let mut inertia: M3 = std::array::from_fn(|i| std::array::from_fn(|j| if i == j { diag[i] } else { 0.0 }));
        for a in geo.anchors {
            for i in 0..3 {
                for j in 0..3 {
                    inertia[i][j] += at_joint * (if i == j { dot(a, a) } else { 0.0 } - a[i] * a[j]);
                }
            }
        }
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
        let mech = Hexapod {
            geo,
            screw,
            mass: cfg.positive("mass.platform_kg")? + 6.0 * at_joint,
            payload: if cfg.bool("effects.payload")? { cfg.non_negative("mass.payload_kg")? } else { 0.0 },
            attached: true,
            inertia,
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            required_life: cfg.f64("ratings.required_life_h").ok(),
            last: LastFound::default(),
        };
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
                gear_lever: Some(lever),
                unit: "m",
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

    fn moving(&self) -> f64 {
        self.mass + if self.attached { self.payload } else { 0.0 }
    }

    /// The pose for leg extensions, its legs and rotation; `None` where the forward kinematics fails.
    fn pose(&self, q: &[f64]) -> Option<([f64; 6], Legs, M3)> {
        let mut tool = [0.0; 6];
        self.geo.forward(q, &mut tool);
        if !tool.iter().all(|v| v.is_finite()) {
            return None;
        }
        Some((tool, self.geo.legs(&tool), Geometry::rotation(&tool)))
    }

    /// The legs, the platform's rotation and J^-1 for leg extensions, for the dynamics, which runs four times a
    /// sub-step: Newton starts from the pose at the nearest point of the `ANCHOR_STEP` grid moved by J^-1 (q - q_grid),
    /// which takes it two steps instead of about five from home (`forward`); from home where that does not converge.
    /// The grid point's pose (from home) is kept until the legs leave its cell. The pose differs from `forward`'s by
    /// Newton's tolerance (1e-13 m on the legs). `None` where the forward kinematics fails or J is singular.
    fn warm_pose(&self, q: &[f64]) -> Option<(Legs, M3, [[f64; 6]; 6])> {
        let key: [i64; 6] = std::array::from_fn(|i| (q[i] / ANCHOR_STEP).round() as i64);
        let anchor = self.last.get().filter(|f| f.key == key).or_else(|| {
            let g: [f64; 6] = std::array::from_fn(|i| key[i] as f64 * ANCHOR_STEP);
            let (tool, legs, r) = self.pose(&g)?;
            let jinv = inverse(&Geometry::jacobian_rows(&legs))?;
            let found = Found { key, q: g, p: [tool[0], tool[1], tool[2]], r, jinv };
            self.last.set(Some(found));
            Some(found)
        });
        let warm = anchor.and_then(|f| {
            let xi: [f64; 6] = std::array::from_fn(|r| (0..6).map(|c| f.jinv[r][c] * (q[c] - f.q[c])).sum());
            self.geo.solve_from(q, add(f.p, [xi[0], xi[1], xi[2]]), matmul(&exp([xi[3], xi[4], xi[5]]), &f.r))
        });
        let (r, legs) = match warm {
            Some((_, r, legs)) => (r, legs),
            None => {
                let (_, legs, r) = self.pose(q)?;
                (r, legs)
            }
        };
        Some((legs, r, inverse(&Geometry::jacobian_rows(&legs))?))
    }

    /// The platform's mass matrix over its twist (6 x 6).
    fn twist_mass(&self, r: &M3) -> [[f64; 6]; 6] {
        let iw = matmul(&matmul(r, &self.inertia), &transpose(r));
        let mut m = [[0.0; 6]; 6];
        for k in 0..3 {
            m[k][k] = self.moving();
            for c in 0..3 {
                m[3 + k][3 + c] = iw[k][c];
            }
        }
        m
    }

    /// The forces the legs push the platform with to hold it at rest at a tool pose: J^-T (0, 0, m g, 0, 0, 0), N.
    pub fn static_leg_forces(&self, tool: &[f64]) -> Option<[f64; 6]> {
        let jt: Vec<f64> = {
            let j = Geometry::jacobian(&self.geo.legs(tool));
            (0..36).map(|k| j[(k % 6) * 6 + k / 6]).collect()
        };
        let f = solve(&jt, &[0.0, 0.0, self.moving() * self.g, 0.0, 0.0, 0.0])?;
        Some(std::array::from_fn(|i| f[i]))
    }
}

impl Mechanism for Hexapod {
    fn dynamics(&self, q: &[f64], qd: &[f64], mass: &mut [f64], bias: &mut [f64]) {
        let Some((legs, r, jinv)) = self.warm_pose(q) else {
            mass.fill(f64::NAN);
            bias.fill(f64::NAN);
            return;
        };
        let xi: [f64; 6] = std::array::from_fn(|r| (0..6).map(|c| jinv[r][c] * qd[c]).sum());
        let (v, w) = ([xi[0], xi[1], xi[2]], [xi[3], xi[4], xi[5]]);
        let a0: [f64; 6] = std::array::from_fn(|i| {
            let wi = add(v, cross(w, legs.b[i]));
            let along = dot(legs.u[i], wi);
            (dot(wi, wi) - along * along) / legs.length[i] + dot(legs.u[i], cross(w, cross(w, legs.b[i])))
        });
        let mx = self.twist_mass(&r);
        let iw = [[mx[3][3], mx[3][4], mx[3][5]], [mx[4][3], mx[4][4], mx[4][5]], [mx[5][3], mx[5][4], mx[5][5]]];
        let gyro = cross(w, mul(&iw, w));
        let jinv_a0: [f64; 6] = std::array::from_fn(|r| (0..6).map(|c| jinv[r][c] * a0[c]).sum());
        let wrench: [f64; 6] = std::array::from_fn(|k| {
            let held = [0.0, 0.0, self.moving() * self.g, gyro[0], gyro[1], gyro[2]][k];
            held - (0..6).map(|c| mx[k][c] * jinv_a0[c]).sum::<f64>()
        });
        // M_xi J^-1, once for every column of M_q.
        let mx_jinv: [[f64; 6]; 6] =
            std::array::from_fn(|k| std::array::from_fn(|j| (0..6).map(|l| mx[k][l] * jinv[l][j]).sum::<f64>()));
        for i in 0..6 {
            bias[i] = (0..6).map(|k| jinv[k][i] * wrench[k]).sum();
            for j in 0..6 {
                mass[i * 6 + j] = (0..6).map(|k| jinv[k][i] * mx_jinv[k][j]).sum();
            }
        }
    }
    /// The platform (its joints' share of the legs and the payload with it) at its centre p with its twist (v, w):
    /// m v, p x m v + R I R^T w; the platform joints lie symmetric about p, so their first moment is m p. Each leg's
    /// rotor spins along the leg's u_i. The legs' shares at the base joints do not move.
    fn frame_momentum(&self, q: &[f64], qd: &[f64], rotor_spin: &[f64]) -> Option<sim_core::FrameMomentum> {
        let (tool, legs, r) = self.pose(q)?;
        let jinv = inverse(&Geometry::jacobian_rows(&legs))?;
        let xi: [f64; 6] = std::array::from_fn(|r| (0..6).map(|c| jinv[r][c] * qd[c]).sum());
        let mut f = sim_core::FrameMomentum { gravity: [0.0, 0.0, -self.g], ..Default::default() };
        sim_components::base::add_point(&mut f, self.moving(), [tool[0], tool[1], tool[2]], [xi[0], xi[1], xi[2]]);
        let iw = matmul(&matmul(&r, &self.inertia), &transpose(&r));
        let turning = mul(&iw, [xi[3], xi[4], xi[5]]);
        (0..3).for_each(|k| f.angular[k] += turning[k]);
        for (u, spin) in legs.u.iter().zip(rotor_spin) {
            sim_components::base::add_spin(&mut f, *u, 1.0, *spin);
        }
        Some(f)
    }
    fn potential(&self, q: &[f64]) -> f64 {
        self.pose(q).map_or(f64::NAN, |(tool, ..)| self.moving() * self.g * tool[2])
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
        if !(self.geo.joint_angle(tool) <= self.geo.max_joint_angle) {
            return Some("error-hexapod-joint-angle");
        }
        if !(self.geo.condition(tool) <= self.geo.max_condition) {
            return Some("error-hexapod-singularity");
        }
        None
    }
    fn condition(&self, tool: &[f64]) -> Option<f64> {
        Some(self.geo.condition(tool))
    }
    fn tool_is_joints(&self) -> bool {
        false
    }

    /// Per leg its screw's speed, d n and wear; then the condition number at the pose.
    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out: Vec<SignalInfo> =
            (0..6).flat_map(|i| self.screw.signal_infos(MOTOR_IDS[i], JOINT_KEYS[i])).collect();
        out.push(SignalInfo::new("platform.condition", "group-hexapod", "quantity-condition-number", ""));
        out
    }
    fn signals(&self, q: &[f64], qd: &[f64], _qdd: &[f64], force: &[f64], out: &mut Vec<f64>) {
        for i in 0..6 {
            self.screw.signals(qd[i], force[i], out);
        }
        out.push(self.pose(q).map_or(f64::INFINITY, |(tool, ..)| self.geo.condition(&tool)));
    }
    /// Each leg's screw: speed, d n, buckling and static load under the leg's force, life.
    fn rating_checks(&self) -> Vec<RatingCheck> {
        (0..6).flat_map(|i| self.screw.rating_checks(MOTOR_IDS[i], i, self.required_life)).collect()
    }

    fn collision_parts(&self) -> &'static [&'static str] {
        &["tool", "payload"]
    }

    /// Base and platform as hexagons through their joints, each leg as its cylinder and rod, the motors below the
    /// base joints.
    fn scene(&self, q: &[f64], rotors: &[f64], rotor_speeds: &[f64], out: &mut Scene) {
        let Some((tool, legs, _)) = self.pose(q) else { return };
        let p = [tool[0], tool[1], tool[2]];
        let top: Vec<V3> = (0..6).map(|i| add(p, legs.b[i])).collect();
        for i in 0..6 {
            let next = (i + 1) % 6;
            out.parts.push(Part::new(
                "base",
                Shape::Segment { to: self.geo.base[next], width: 0.03 },
                self.geo.base[i],
                [0.55, 0.57, 0.6],
            ));
            out.parts.push(Part::new(
                "platform",
                Shape::Segment { to: top[next], width: 0.03 },
                top[i],
                [0.3, 0.45, 0.7],
            ));
        }
        let body = 0.6 * self.geo.closed;
        for i in 0..6 {
            let (b, u) = (self.geo.base[i], legs.u[i]);
            let end = add(b, u.map(|v| v * body));
            out.parts.push(Part::new("cylinder", Shape::Segment { to: end, width: 0.05 }, b, [0.75, 0.75, 0.78]));
            out.parts.push(Part::new(
                "rod",
                Shape::Segment { to: add(b, u.map(|v| v * 0.9 * body)), width: 0.025 },
                top[i],
                [0.85, 0.86, 0.9],
            ));
            out.parts.push(Part::new(
                sim_core::scene::MOTOR_PART,
                Shape::Box { size: [0.07, 0.07, 0.09] },
                add(b, [0.0, 0.0, -0.06]),
                [0.9, 0.45, 0.1],
            ));
            // The shaft's bar below the motor turns with the rotor, as on every machine.
            out.parts.push(
                Part::new("rotor-bar", Shape::Box { size: [0.07, 0.004, 0.012] }, add(b, [0.0, 0.0, -0.11]), [0.1; 3])
                    .turned(rotors[i]),
            );
            out.labels.push(Label {
                pos: add(b, [0.0, 0.0, -0.14]),
                owner_key: MOTOR_KEYS[i],
                value: rotor_speeds[i] * 60.0 / TAU,
                unit_key: "unit-rpm",
                decimals: 0,
            });
            out.labels.push(Label {
                pos: add(b, u.map(|v| v * 0.5 * legs.length[i])),
                owner_key: JOINT_KEYS[i],
                value: legs.length[i],
                unit_key: "unit-m",
                decimals: 3,
            });
        }
        out.parts.push(Part::new(
            "tool",
            Shape::Box { size: [0.06, 0.06, 0.03] },
            add(p, [0.0, 0.0, 0.015]),
            [0.2, 0.22, 0.26],
        ));
        if self.attached && self.payload > 0.0 {
            out.parts.push(Part::new(
                "payload",
                Shape::Box { size: [0.12, 0.12, 0.08] },
                add(p, [0.0, 0.0, 0.07]),
                [0.2, 0.7, 0.3],
            ));
        }
        out.labels.push(Label {
            pos: add(p, [0.0, 0.0, 0.2]),
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
