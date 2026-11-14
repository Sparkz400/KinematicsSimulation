//! Parallel SCARA: two motor-driven arms turning about vertical axes in the horizontal x-y plane, one at each side of
//! the base, each carrying a pair of forearms to a plate that keeps its orientation (a five-bar whose outer links are
//! parallelograms), as Mitsubishi's RP series or the parallel SCARAs with a five-bar. At the plate an optional Z stroke
//! on a ball screw (`[z_axis]`, motor 3) and an optional turn of the tool about z (`[tool_turn]`, motor 4). The tool
//! pose is (x, y[, z][, c]).
//!
//! The five-bar follows the D2's (the planar delta picker), laid flat: the same equations, copied, as a machine crate
//! may not use another's. Leg i points along s_i x (s_1 = +1, s_2 = -1); e = base radius - platform radius (the
//! plate's offset folded into the base, as it never turns), the arm angle theta_i (0 along x, positive towards -y),
//! upper arm L_a, forearm L_b:
//!
//! ```text
//! elbow     c_i = (s_i (e + L_a cos theta), -L_a sin theta)
//! constraint |p - c_i|^2 = L_b^2
//! inverse   A cos theta + B sin theta + C = 0, A = -2 L_a (x' - e), B = 2 L_a y, C = (x' - e)^2 + y^2 + L_a^2 - L_b^2
//!           theta = 2 atan((-B - sqrt(A^2 + B^2 - C^2)) / (C - A))     (elbow out; x' = s_i x)
//! forward   p = the one of the two points at L_b from both elbows further from the shoulders (two circles)
//! Jacobian  J_x p' = J_q theta', J_x rows u_i = p - c_i, J_q = diag(u_i . dc_i/dtheta_i); p' = J theta', J = J_x^-1 J_q
//!           J_x p'' = J_q theta'' + w, w_i = theta_i'^2 (u_i . c_i'') - |p' - c_i' theta_i'|^2
//! ```
//!
//! Dynamics (Codourey 1998, Olsson 2009): each forearm pair's mass split, a share r at the elbow and 1 - r at the
//! plate; two arms about their joints plus a point mass at the plate, m_nt = plate + payload + 2 (1 - r) m_fb + what
//! the plate carries (the Z and C units, the carriage), and
//!
//! ```text
//! M(theta) theta'' = tau_gear - m_nt J^T J_x^-1 w + tau_joint,   M = I_b 1 + m_nt J^T J
//! I_b = I_arm + L_a^2 (m_elbow + r m_fb)
//! ```
//!
//! Gravity acts along -z, out of the plane, on the joints' bearings: it gives the arms no torque. Each arm's joint is a
//! `RotaryJoint`, elastic or rigid (effects.joint_elasticity). The Z stroke and the tool's turn are rigid
//! `RotaryJoint`s from their motors' own config (`[motor.<id>]`, `[gearbox.<id>]`): the stroke's nut lifts the
//! carriage, m_z q'' = F - m_z g, the turn spins the tool, J_c c'' = T. Both are perpendicular to the plane and about
//! its normal, so the plane's motors feel their masses (in m_nt) and nothing else.

use sim_components::catalog::motor_config;
use sim_components::joint::RotaryJoint;
use sim_components::mech::{HardStop, ShaftFriction, ThermalFeedback};
use sim_components::transmission::BallScrew;
use sim_core::{
    AxisInfo, Config, ConfigError, ENERGY_PARTS, LOSS_PREFIX, Label, Machine, MachineEntry, MotorInfo, Part,
    RatingCheck, Scene, Shape, SignalInfo, TORQUE_PARTS, ThermalPart,
};

const I18N: &[(&str, &str)] = &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))];

pub static ENTRY: MachineEntry = MachineEntry {
    id: "parallel-scara",
    key: "machine-parallel-scara",
    default_config: include_str!("../default.toml"),
    i18n: I18N,
    build,
    suite_configs,
};

const G: f64 = 9.80665;

/// Effects that take energy out or put it in; the standard suite runs with them off. The stops stay, without their
/// damping.
pub const LOSSY_EFFECTS: [&str; 5] =
    ["joint_friction", "motor_friction", "gearbox_efficiency", "torque_ripple", "holding_brake"];

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    Ok(Box::new(ParallelScara::new(cfg)?))
}

/// The default config with every loss switched off.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    for key in ["gearbox.damping_nm_s_rad", "joint.stop_damping_nm_s_rad", "slide.stop_damping_n_s_m"] {
        cfg.set_text(&format!("{key} = 0.0")).unwrap();
    }
    cfg
}

/// Loss-free rigid and elastic models, with and without the payload, and the elastic one with part heating and the
/// temperature feedback (no loss, so the gearboxes stay at ambient).
fn suite_configs() -> Vec<Config> {
    let mut out = Vec::new();
    for (elastic, payload) in [(false, true), (false, false), (true, true), (true, false)] {
        let mut cfg = lossless_config();
        cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
        cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
        cfg.set_text(&format!("effects.part_heating = {}", elastic && payload)).unwrap();
        cfg.set_text(&format!("effects.thermal_feedback = {}", elastic && payload)).unwrap();
        out.push(cfg);
    }
    out
}

/// A vector in the plane, (x, y).
type V2 = [f64; 2];
type M2 = [[f64; 2]; 2];

fn dot(a: V2, b: V2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn sub(a: V2, b: V2) -> V2 {
    [a[0] - b[0], a[1] - b[1]]
}

fn norm(a: V2) -> f64 {
    dot(a, a).sqrt()
}

fn mul(m: &M2, v: V2) -> V2 {
    [dot(m[0], v), dot(m[1], v)]
}

fn transpose(m: &M2) -> M2 {
    [[m[0][0], m[1][0]], [m[0][1], m[1][1]]]
}

/// The inverse of a 2 x 2 matrix; `None` where it is singular.
fn inverse(m: &M2) -> Option<M2> {
    let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    if !(det.abs() > 1e-300) || !det.is_finite() {
        return None;
    }
    Some([[m[1][1] / det, -m[0][1] / det], [-m[1][0] / det, m[0][0] / det]])
}

/// The 2-norm condition number of a 2 x 2 matrix: the square root of the larger over the smaller eigenvalue of
/// J^T J, m +- sqrt(((a - c) / 2)^2 + b^2) with m = (a + c) / 2 for J^T J = [[a, b], [b, c]].
pub fn condition_number(j: &M2) -> f64 {
    let (a, b, c) = (
        j[0][0] * j[0][0] + j[1][0] * j[1][0],
        j[0][0] * j[0][1] + j[1][0] * j[1][1],
        j[0][1] * j[0][1] + j[1][1] * j[1][1],
    );
    let (mean, half) = ((a + c) / 2.0, ((a - c) / 2.0).hypot(b));
    if mean - half <= 0.0 { f64::INFINITY } else { ((mean + half) / (mean - half)).sqrt() }
}

/// The leg geometry and the kinematics, apart from masses and drives.
#[derive(Debug, Clone)]
pub struct Legs {
    /// Base radius less platform radius, upper arm, forearm.
    pub e: f64,
    pub la: f64,
    pub lb: f64,
}

/// The side each leg points to along x.
pub const SIDE: [f64; 2] = [1.0, -1.0];

impl Legs {
    pub fn new(e: f64, la: f64, lb: f64) -> Self {
        Self { e, la, lb }
    }

    /// The elbow of leg i at arm angle t, and its first and second derivative by t, as (x, y).
    pub fn elbow(&self, i: usize, t: f64) -> [V2; 3] {
        let (s, c) = t.sin_cos();
        let (la, side) = (self.la, SIDE[i]);
        [[side * (self.e + la * c), -la * s], [-side * la * s, -la * c], [-side * la * c, la * s]]
    }

    /// Arm angles for a plate position (x, y), elbows out; `None` out of reach.
    pub fn inverse(&self, p: V2) -> Option<V2> {
        let mut out = [0.0; 2];
        for (i, t) in out.iter_mut().enumerate() {
            let (x, y) = (SIDE[i] * p[0], p[1]);
            let (a, b, c) = (
                -2.0 * self.la * (x - self.e),
                2.0 * self.la * y,
                (x - self.e).powi(2) + y * y + self.la * self.la - self.lb * self.lb,
            );
            let d = a * a + b * b - c * c;
            if !(d >= 0.0) {
                return None;
            }
            *t = 2.0 * ((-b - d.sqrt()) / (c - a)).atan();
        }
        Some(out)
    }

    /// The plate position for arm angles: the point at a forearm's length from both elbows further from the shoulders.
    /// `None` where the legs cannot be assembled.
    pub fn forward(&self, t: V2) -> Option<V2> {
        let (c0, c1) = (self.elbow(0, t[0])[0], self.elbow(1, t[1])[0]);
        let along = sub(c1, c0);
        let d = norm(along);
        let h2 = self.lb * self.lb - d * d / 4.0;
        if !(d > 1e-12 && h2 >= 0.0) {
            return None;
        }
        // Midway between the elbows, then h along the normal to the line joining them.
        let (mid, h) = ([0.5 * (c0[0] + c1[0]), 0.5 * (c0[1] + c1[1])], h2.sqrt());
        let normal = [-along[1] / d, along[0] / d];
        let (one, other) =
            ([mid[0] + h * normal[0], mid[1] + h * normal[1]], [mid[0] - h * normal[0], mid[1] - h * normal[1]]);
        Some(if one[1] < other[1] { one } else { other })
    }

    /// J_x (rows: the forearm vectors u_i = p - c_i) and the diagonal of J_q (u_i . dc_i/dt).
    fn constraint(&self, t: V2, p: V2) -> (M2, V2) {
        let rows = [self.elbow(0, t[0]), self.elbow(1, t[1])];
        let jx = rows.map(|e| sub(p, e[0]));
        (jx, [dot(jx[0], rows[0][1]), dot(jx[1], rows[1][1])])
    }

    /// The Jacobian J[i][j] = dp_i / dtheta_j (rows x and y) at arm angles t and plate position p; `None` at a direct
    /// singularity.
    pub fn jacobian(&self, t: V2, p: V2) -> Option<M2> {
        let (jx, jq) = self.constraint(t, p);
        let inv = inverse(&jx)?;
        Some(std::array::from_fn(|i| std::array::from_fn(|j| inv[i][j] * jq[j])))
    }

    /// J' theta' = J_x^-1 w: the plate's acceleration at arm speeds `wt` and no arm acceleration.
    pub fn velocity_term(&self, t: V2, p: V2, wt: V2, pv: V2) -> Option<V2> {
        let (jx, _) = self.constraint(t, p);
        let inv = inverse(&jx)?;
        let w: V2 = std::array::from_fn(|i| {
            let [_, d1, d2] = self.elbow(i, t[i]);
            let rel = sub(pv, d1.map(|v| v * wt[i]));
            wt[i] * wt[i] * dot(jx[i], d2) - dot(rel, rel)
        });
        Some(mul(&inv, w))
    }
}

/// An axis at the plate beside the plane's two: the Z stroke on its ball screw (in N and m), or the
/// tool's turn about z. Its state is [position, speed, brake].
// shortcut: rigid, as the arms are in the rigid model: the screw's and the turn gearbox's twist and backlash are left
// out (the carriage on its screw is a mode near 570 Hz); give them states as the arms have once their modes matter.
struct Extra {
    joint: RotaryJoint,
    /// What it moves (kg on the stroke, kg m^2 on the turn), and the payload's share while it is held.
    load: f64,
    payload: f64,
    /// Gravity along it: g on the stroke, 0 on the turn.
    g: f64,
    /// The stroke's screw; `None` on the turn.
    screw: Option<BallScrew>,
    thermal: Option<ThermalFeedback>,
}

impl Extra {
    fn mass(&self, attached: bool) -> f64 {
        self.load + if attached { self.payload } else { 0.0 }
    }

    /// The weight along the joint (negative: down).
    fn weight(&self, attached: bool) -> f64 {
        -self.g * self.mass(attached)
    }

    fn ratio(&self) -> f64 {
        self.joint.ratio()
    }

    /// The acceleration, and the force (torque) into the gearbox after the rotor's own acceleration and out of it, as
    /// the arms' `rigid_solve`: (m + eta i^2 J_rotor) q'' = eta drive + weight + stops and friction.
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

    /// Powers lost in its friction, its motor's friction with the brake, its gearbox (coupling and screw), its stops
    /// and what the ripple gives off; and the gearbox's loss as a torque at the rotor.
    fn losses(&self, x: &[f64], torque: f64, engaged: bool, attached: bool) -> ([f64; 5], f64) {
        let i = self.ratio();
        let [friction, stop, shaft] = self.joint.losses(x[0], x[1], x[1] * i, x[2], engaged);
        let (_, into, out) = self.solve(x, torque, engaged, attached);
        let other = (torque - self.joint.ripple.apply(torque, x[0] * i)) * x[1] * i;
        ([friction, shaft, (into - out) * x[1], stop, other], (into - out) / i)
    }

    /// The motor torque's parts (`TORQUE_PARTS`), as the arms'.
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

/// How far the tool's flange hangs below the plate with the stroke at its top (or without a stroke), m (view only).
const TOOL_DROP: f64 = 0.1;

pub struct ParallelScara {
    legs: Legs,
    /// The geometry as configured, for the scene: base and platform radius, the rods' spacing.
    base_radius: f64,
    platform_radius: f64,
    rod_spacing: f64,
    elastic: bool,
    /// The Z stroke and the tool's turn, where there are, in that order: axes and motors 3 and 4 on.
    extras: Vec<Extra>,
    required_life: Option<f64>,
    /// I_b without the rotor, plate mass without and with payload (forearm share and what it carries included).
    arm_inertia: f64,
    m_platform: f64,
    m_payload: f64,
    payload_attached: bool,
    brake_engaged: bool,
    joints: [RotaryJoint; 2],
    thermal: Option<ThermalFeedback>,
    /// The working range: half width, its near edge (y), its far arc's centre and radius; the condition number refused
    /// above.
    half_width: f64,
    top: f64,
    circle: (f64, f64),
    max_condition: f64,
    axes: Vec<AxisInfo>,
    motors: Vec<MotorInfo>,
    /// `Machine::setpoint_slack`, worked out once (`rest_sag`).
    rest_slack: f64,
}

// Elastic state: [theta 1, 2, omega 1, 2, rotor angle 1, 2, rotor speed 1, 2, brake 1, 2].
const N_ELASTIC: usize = 10;
// Rigid state: [theta 1, 2, omega 1, 2, brake 1, 2].
const N_RIGID: usize = 6;

/// The losses of one moment: per joint its friction, its motor's friction with the brake, its gearbox; the stops and
/// what the ripple gives off; and per joint the gearbox loss as a torque at the rotor.
struct Losses {
    powers: [f64; 8],
    gear_at_rotor: V2,
}

impl ParallelScara {
    /// The plate at the tool pose, the quill hanging from it down to the tool's flange, and the payload under that, as
    /// `scene` draws them.
    fn on_platform(&self, tool: &[f64]) -> Vec<Part> {
        let (p, size) = ([tool[0], tool[1]], 2.4 * self.platform_radius.max(0.03));
        {
            let flange = self.flange_z(tool);
            let mut parts = vec![
                Part::new(
                    "platform",
                    Shape::Box { size: [size, 0.06, self.rod_spacing.max(0.06) + 0.03] },
                    [p[0], p[1], 0.0],
                    [0.55, 0.55, 0.6],
                ),
                Part::new("quill", Shape::Box { size: [0.025, 0.025, -flange] }, [p[0], p[1], 0.5 * flange], [0.7; 3]),
                Part::new("tool", Shape::Box { size: [0.06, 0.06, 0.02] }, [p[0], p[1], flange - 0.01], [0.3; 3]),
            ];
            if self.payload_attached && self.m_payload > 0.0 {
                parts.push(Part::new(
                    "payload",
                    Shape::Box { size: [0.12, 0.12, 0.1] },
                    [p[0], p[1], flange - 0.07],
                    [0.2, 0.7, 0.3],
                ));
            }
            parts
        }
    }

    /// The height of the tool's flange in the scene, the stroke's top `TOOL_DROP` below the plate.
    fn flange_z(&self, tool: &[f64]) -> f64 {
        match self.extras.iter().position(|x| x.screw.is_some()) {
            Some(e) => tool[2 + e] - self.axes[2 + e].max - TOOL_DROP,
            None => -TOOL_DROP,
        }
    }

    /// The tool pose of a state: the plate's, then the stroke's and the turn's positions.
    fn tool_pose(&self, s: &[f64]) -> Vec<f64> {
        let (p, _) = self.pose(self.arms(s).0);
        let mut tool = p.to_vec();
        tool.extend((0..self.extras.len()).map(|e| self.extra_state(s, e)[0]));
        tool
    }

    /// The state of extra axis `e`: [position, speed, brake], after the arms'.
    fn extra_state<'a>(&self, s: &'a [f64], e: usize) -> &'a [f64] {
        let at = self.planar_len() + 3 * e;
        &s[at..at + 3]
    }

    fn planar_len(&self) -> usize {
        if self.elastic { N_ELASTIC } else { N_RIGID }
    }

    /// The Z stroke (motor 3): the carriage on a ball screw behind a coupling, `[slide]` and `[screw]` as for a screw
    /// axis, with motor 3's own config; the unit (Z motor, screw, housing) and the carriage at the plate.
    fn z_axis(cfg: &Config, payload: f64, g: f64) -> Result<(Extra, AxisInfo, f64), ConfigError> {
        let own = motor_config(cfg, "3")?;
        let mut joint = RotaryJoint::linear_from_config(&own)?;
        let screw = BallScrew::from_config(&own)?;
        if screw.length < joint.range.1 {
            return Err(ConfigError::Invalid("screw.length_m".into()));
        }
        screw.converter(screw.softest_position()).apply(&own, &mut joint)?;
        // The nut's preload drags like friction in the guides, as a force at the nut.
        if own.bool("effects.joint_friction")? {
            joint.friction.coulomb += screw.preload_torque() * screw.ratio();
        }
        let carriage = cfg.non_negative("z_axis.carriage_kg")?;
        let axis = Self::axis_info(cfg, "z", "axis-z", "m", joint.range)?;
        let carried = cfg.non_negative("z_axis.unit_kg")? + carriage;
        Ok((Extra { joint, load: carriage, payload, g, screw: Some(screw), thermal: None }, axis, carried))
    }

    /// The tool's turn (motor 4) through its gearbox, with no stops: the tool, and the payload's inertia while held;
    /// the unit (C motor and gearbox) at the plate.
    fn tool_turn(cfg: &Config, payload: bool) -> Result<(Extra, AxisInfo, f64), ConfigError> {
        let own = motor_config(cfg, "4")?;
        let on = cfg.bool("effects.joint_friction")?;
        let friction = ShaftFriction {
            coulomb: if on { cfg.non_negative("tool_turn.friction_coulomb_nm")? } else { 0.0 },
            viscous: if on { cfg.non_negative("tool_turn.friction_viscous_nm_s_rad")? } else { 0.0 },
            smoothing: cfg.positive("tool_turn.friction_smoothing_rad_s")?,
        };
        let range = (cfg.f64("axis.c.min_angle_deg")?.to_radians(), cfg.f64("axis.c.max_angle_deg")?.to_radians());
        if range.0 >= range.1 {
            return Err(ConfigError::Invalid("axis.c.max_angle_deg".into()));
        }
        let joint = RotaryJoint::with(&own, friction, HardStop { stiffness: 0.0, damping: 0.0 }, range)?;
        let extra = Extra {
            joint,
            load: cfg.positive("tool_turn.inertia_kgm2")?,
            payload: if payload { cfg.non_negative("tool_turn.payload_inertia_kgm2")? } else { 0.0 },
            g: 0.0,
            screw: None,
            thermal: ThermalFeedback::from_config(&own)?,
        };
        Ok((extra, Self::axis_info(cfg, "c", "axis-c", "rad", range)?, cfg.non_negative("tool_turn.unit_kg")?))
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
        let i_arm = cfg.positive("mass.upper_arm_inertia_kgm2")?;
        let (m_elbow, m_fb) = (cfg.non_negative("mass.elbow_kg")?, cfg.non_negative("mass.forearm_pair_kg")?);
        let elbow = m_elbow + share * m_fb;
        let (half_width, top) = (cfg.positive("workspace.half_width_m")?, cfg.f64("workspace.near_y_m")?);
        let (edge, centre) = (cfg.positive("workspace.depth_edge_m")?, cfg.positive("workspace.depth_centre_m")?);
        if centre < edge {
            return Err(ConfigError::Invalid("workspace.depth_centre_m".into()));
        }
        // The far edge ("bottom", "top" the near edge): a circle through the centre's depth and both edges; straight
        // where both depths are the same.
        let sag = centre - edge;
        let circle = if sag > 0.0 {
            let r = (half_width * half_width + sag * sag) / (2.0 * sag);
            (top - centre + r, r)
        } else {
            (f64::INFINITY, f64::INFINITY)
        };
        let joint = RotaryJoint::from_config(cfg)?;
        let g = if cfg.bool("effects.gravity")? { G } else { 0.0 };
        let payload = cfg.bool("effects.payload")?;
        let m_payload = if payload { cfg.non_negative("mass.payload_kg")? } else { 0.0 };
        let mut axes = vec![
            Self::axis_info(cfg, "x", "axis-x", "m", (-half_width, half_width))?,
            Self::axis_info(cfg, "y", "axis-y", "m", (top - centre, top))?,
        ];
        let mut motors = vec![MotorInfo { id: "1", key: "motor-arm-1" }, MotorInfo { id: "2", key: "motor-arm-2" }];
        let (mut extras, mut carried) = (Vec::new(), 0.0);
        let mut add = |(extra, axis, mass): (Extra, AxisInfo, f64), motor: MotorInfo| {
            extras.push(extra);
            axes.push(axis);
            motors.push(motor);
            carried += mass;
        };
        if cfg.bool("z_axis.enabled")? {
            add(Self::z_axis(cfg, m_payload, g)?, MotorInfo { id: "3", key: "motor-z" });
        }
        if cfg.bool("tool_turn.enabled")? {
            add(Self::tool_turn(cfg, payload)?, MotorInfo { id: "4", key: "motor-tool" });
        }
        let mut scara = Self {
            legs: Legs::new(base_radius - platform_radius, la, lb),
            base_radius,
            platform_radius,
            rod_spacing: cfg.non_negative("geometry.forearm_spacing_m")?,
            elastic: cfg.bool("effects.joint_elasticity")?,
            extras,
            required_life: cfg.f64("ratings.required_life_h").ok(),
            arm_inertia: i_arm + la * la * elbow,
            m_platform: cfg.positive("mass.platform_kg")? + 2.0 * (1.0 - share) * m_fb + carried,
            m_payload,
            payload_attached: true,
            brake_engaged: false,
            joints: [joint.clone(), joint],
            thermal: ThermalFeedback::from_config(cfg)?,
            half_width,
            top,
            circle,
            max_condition: cfg.positive("workspace.max_condition_number")?,
            axes,
            motors,
            rest_slack: 0.0,
        };
        scara.rest_slack = scara.rest_sag();
        Ok(scara)
    }

    /// How far the motors' pose can be from the tool's at rest anywhere in the working range: each gearbox at either
    /// side of its backlash (all four combinations; nothing loads them at rest, gravity being out of the plane),
    /// through the forward kinematics. Over a grid of the range, 41 columns of 9 depths and the far edge; plus a tenth for what
    /// the grid steps over and the joints' friction. At least the default 0.1 mm; that alone in the rigid model.
    fn rest_sag(&self) -> f64 {
        if !self.elastic {
            return 1e-4;
        }
        let mut most = 0f64;
        for column in 0..=40 {
            let x = self.half_width * (column as f64 / 20.0 - 1.0);
            let lowest = self.bottom(x);
            for z in (0..=8).map(|k| lowest + (self.top - lowest) * k as f64 / 8.0) {
                let p = [x, z];
                if !self.in_envelope(&p, 1e-9) {
                    continue;
                }
                let Some(t) = self.legs.inverse(p) else { continue };
                for signs in 0..4 {
                    let half =
                        |k: usize| if signs >> k & 1 == 1 { 1.0 } else { -1.0 } * self.joints[k].gearbox.half_backlash;
                    if let Some(q) = self.legs.forward(std::array::from_fn(|k| t[k] + half(k))) {
                        most = most.max(norm(sub(q, p)));
                    }
                }
            }
        }
        (1.1 * most).max(1e-4)
    }

    pub fn legs(&self) -> &Legs {
        &self.legs
    }

    fn ratio(&self) -> f64 {
        self.joints[0].ratio()
    }

    /// The point mass at the plate.
    pub fn m_nt(&self) -> f64 {
        self.m_platform + if self.payload_attached { self.m_payload } else { 0.0 }
    }

    /// The arm's inertia about its joint, elbow share of the forearms included, rotor not.
    pub fn arm_inertia(&self) -> f64 {
        self.arm_inertia
    }

    /// Arm angles and speeds of a state.
    fn arms(&self, s: &[f64]) -> (V2, V2) {
        ([s[0], s[1]], [s[2], s[3]])
    }

    /// Rotor angles and speeds of a state.
    fn rotors(&self, s: &[f64]) -> (V2, V2) {
        if self.elastic {
            ([s[4], s[5]], [s[6], s[7]])
        } else {
            let i = self.ratio();
            ([s[0] * i, s[1] * i], [s[2] * i, s[3] * i])
        }
    }

    fn brakes(&self, s: &[f64]) -> V2 {
        let at = if self.elastic { 8 } else { 4 };
        [s[at], s[at + 1]]
    }

    /// Where the plate is and the Jacobian there, for arm angles; NaN where the legs cannot be assembled.
    fn pose(&self, t: V2) -> (V2, M2) {
        self.legs
            .forward(t)
            .and_then(|p| Some((p, self.legs.jacobian(t, p)?)))
            .unwrap_or(([f64::NAN; 2], [[f64::NAN; 2]; 2]))
    }

    /// The load's mass matrix M and the velocity term m_nt J^T J' theta' at arm angles t and speeds w.
    fn load(&self, t: V2, w: V2) -> (M2, V2) {
        let (p, j) = self.pose(t);
        let jt = transpose(&j);
        let m = self.m_nt();
        let mass: M2 = std::array::from_fn(|r| {
            std::array::from_fn(|c| m * dot(jt[r], jt[c]) + if r == c { self.arm_inertia } else { 0.0 })
        });
        let pv = mul(&j, w);
        let velocity = self.legs.velocity_term(t, p, w, pv).map_or([f64::NAN; 2], |a| mul(&jt, a.map(|v| m * v)));
        (mass, velocity)
    }

    /// Rigid model: the arms' acceleration, and per joint the torque into its gearbox (after the rotor's own
    /// acceleration) and out of it, at the arm.
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> (V2, V2, V2) {
        let (t, w) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let (mass, velocity) = self.load(t, w);
        let drive: V2 = std::array::from_fn(|k| {
            self.joints[k].rigid_drive(torque[k], rotor[k], rotor_speed[k], brakes[k], self.brake_engaged).0
        });
        let rest: V2 = std::array::from_fn(|k| self.joints[k].output_torque(t[k], w[k]) - velocity[k]);
        let solve = |loss: V2| -> V2 {
            let m: M2 = std::array::from_fn(|r| {
                std::array::from_fn(|c| {
                    mass[r][c] + if r == c { loss[r] * self.joints[r].reflected_inertia() } else { 0.0 }
                })
            });
            let rhs: V2 = std::array::from_fn(|k| loss[k] * drive[k] + rest[k]);
            inverse(&m).map_or([f64::NAN; 2], |inv| mul(&inv, rhs))
        };
        let first = solve([1.0; 2]);
        let loss: V2 = std::array::from_fn(|k| {
            self.joints[k].gearbox.loss_factor(drive[k] - self.joints[k].reflected_inertia() * first[k], w[k])
        });
        let acc = solve(loss);
        let into: V2 = std::array::from_fn(|k| drive[k] - self.joints[k].reflected_inertia() * acc[k]);
        (acc, into, std::array::from_fn(|k| loss[k] * into[k]))
    }

    /// The arms' acceleration in either model.
    fn arm_acceleration(&self, s: &[f64], torque: &[f64]) -> V2 {
        if self.elastic {
            let mut d = vec![0.0; self.state_len()];
            self.derivatives(s, torque, &mut d);
            [d[2], d[3]]
        } else {
            self.rigid_solve(s, torque).0
        }
    }

    /// The torque each gearbox passes to its arm.
    fn gear_output(&self, s: &[f64], torque: &[f64]) -> V2 {
        if self.elastic {
            std::array::from_fn(|k| self.joints[k].gearbox.torque(s[4 + k], s[6 + k], s[k], s[2 + k]).output)
        } else {
            self.rigid_solve(s, torque).2
        }
    }

    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        let (t, w) = self.arms(s);
        let (mass, ..) = self.load(t, w);
        let mut kinetic = 0.5 * dot(w, mul(&mass, w));
        // Nothing in the plane rises or falls: the potential energy is the Z stroke's alone.
        let potential = 0.0;
        let mut elastic: f64 = (0..2).map(|k| self.joints[k].stop_energy(t[k])).sum();
        if self.elastic {
            for k in 0..2 {
                kinetic += 0.5 * self.joints[k].rotor_inertia * s[6 + k] * s[6 + k];
                elastic += self.joints[k].gearbox.energy(s[4 + k], s[k]);
            }
        } else {
            kinetic += 0.5 * (0..2).map(|k| self.joints[k].reflected_inertia() * w[k] * w[k]).sum::<f64>();
        }
        let mut parts = [kinetic, potential, elastic];
        for (e, x) in self.extras.iter().enumerate() {
            let own = x.energy(self.extra_state(s, e), self.payload_attached);
            (0..3).for_each(|i| parts[i] += own[i]);
        }
        parts
    }

    fn losses(&self, s: &[f64], torque: &[f64]) -> Losses {
        let (t, w) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let mut powers = [0.0; 8];
        let mut gear_at_rotor = [0.0; 2];
        let rigid = (!self.elastic).then(|| self.rigid_solve(s, torque));
        let (mut stops, mut other) = (0.0, 0.0);
        for k in 0..2 {
            let joint = &self.joints[k];
            let [friction, stop, shaft] = joint.losses(t[k], w[k], rotor_speed[k], brakes[k], self.brake_engaged);
            let gear = match &rigid {
                Some((_, into, out)) => {
                    gear_at_rotor[k] = (into[k] - out[k]) / joint.ratio();
                    (into[k] - out[k]) * w[k]
                }
                None => {
                    let g = joint.gearbox.torque(s[4 + k], s[6 + k], s[k], s[2 + k]);
                    gear_at_rotor[k] = g.rotor - g.output / joint.ratio();
                    joint.gearbox.loss_power(s[4 + k], s[6 + k], s[k], s[2 + k])
                }
            };
            powers[3 * k..3 * k + 3].copy_from_slice(&[friction, shaft, gear]);
            stops += stop;
            other += (torque[k] - joint.ripple.apply(torque[k], rotor[k])) * rotor_speed[k];
        }
        // The extra axes' own losses are their signals; their stops and ripple join the arms'.
        for (e, x) in self.extras.iter().enumerate() {
            let ([.., stop, ripple], _) =
                x.losses(self.extra_state(s, e), torque[2 + e], self.brake_engaged, self.payload_attached);
            stops += stop;
            other += ripple;
        }
        powers[6] = stops;
        powers[7] = other;
        Losses { powers, gear_at_rotor }
    }

    /// The working range's bottom below x: on the circle, or flat.
    fn bottom(&self, x: f64) -> f64 {
        let (centre, r) = self.circle;
        if centre.is_finite() { centre - (r * r - x * x).max(0.0).sqrt() } else { self.axes[1].min }
    }

    /// Whether a pose is within the working range (the band |x| <= half width below the top, above the bottom
    /// circle), widened by `slack`.
    fn in_envelope(&self, p: &[f64], slack: f64) -> bool {
        let (centre, r) = self.circle;
        let above =
            if centre.is_finite() { p[0].hypot(p[1] - centre) <= r + slack } else { p[1] >= self.axes[1].min - slack };
        p[0].abs() <= self.half_width + slack && p[1] <= self.top + slack && above
    }

    /// The torque breakdown, loss powers, energies of the effect views.
    fn view_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (t, w) = self.arms(s);
        let (_, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let (mass, velocity) = self.load(t, w);
        let acc = self.arm_acceleration(s, torque);
        let moving = mul(&mass, acc);
        let losses = self.losses(s, torque);
        for k in 0..2 {
            let joint = &self.joints[k];
            let i = joint.ratio();
            let held = joint.brake.torque(self.brake_engaged, rotor_speed[k], brakes[k]).0;
            let parts = [
                (moving[k] + velocity[k] + joint.reflected_inertia() * acc[k]) / i,
                // Gravity acts out of the plane, on the joints' bearings.
                0.0,
                joint.shaft.torque(rotor_speed[k]) + held + losses.gear_at_rotor[k] + joint.friction.torque(w[k]) / i,
            ];
            out.extend(parts);
            out.push(torque[k] - parts.iter().sum::<f64>());
        }
        out.extend(losses.powers);
        out.extend(self.energy_parts(s));
        for (e, x) in self.extras.iter().enumerate() {
            let (state, at) = (self.extra_state(s, e), (torque[2 + e], self.brake_engaged, self.payload_attached));
            out.extend(x.torque_parts(state, at.0, at.1, at.2));
            out.extend(&x.losses(state, at.0, at.1, at.2).0[..3]);
        }
    }

    /// The extra axes' loss powers, in the order of their signals.
    fn extra_losses(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        for (e, x) in self.extras.iter().enumerate() {
            let state = self.extra_state(s, e);
            out.extend(&x.losses(state, torque[2 + e], self.brake_engaged, self.payload_attached).0[..3]);
        }
    }

    /// The torque parts and loss powers of motor `motor` (the arms' with `joint` their arm's key).
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
            .map(|k| Self::motor_infos(&self.motors[k], ["arm-1", "arm-2"][k], self.joints[k].gearbox.is_coupling()))
            .collect();
        out.extend(arms.iter().flat_map(|(parts, _)| parts.clone()));
        out.extend(arms.into_iter().flat_map(|(_, losses)| losses));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}hard_stops"), "group-joint", "quantity-damping-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}other"), "loss-other", "quantity-power-given-off", "W"));
        out.extend(ENERGY_PARTS.map(|part| {
            SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")
        }));
        for (e, x) in self.extras.iter().enumerate() {
            let (parts, losses) = Self::motor_infos(&self.motors[2 + e], self.axes[2 + e].key, x.joint.coupled);
            out.extend(parts);
            out.extend(losses);
        }
    }
}

impl Machine for ParallelScara {
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    /// NaN where the pose is out of reach: the controller never asks for one (see `unreachable`).
    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        let t = self.legs.inverse([tool[0], tool[1]]).unwrap_or([f64::NAN; 2]);
        for k in 0..2 {
            motors[k] = self.ratio() * t[k];
        }
        for (e, x) in self.extras.iter().enumerate() {
            motors[2 + e] = x.ratio() * tool[2 + e];
        }
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        let i = self.ratio();
        let p = self.legs.forward([motors[0] / i, motors[1] / i]).unwrap_or([f64::NAN; 2]);
        tool[..2].copy_from_slice(&p);
        for (e, x) in self.extras.iter().enumerate() {
            tool[2 + e] = motors[2 + e] / x.ratio();
        }
    }

    fn state_len(&self) -> usize {
        self.planar_len() + 3 * self.extras.len()
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let t = self.legs.inverse([tool[0], tool[1]]).unwrap_or([f64::NAN; 2]);
        state.clear();
        state.extend(t);
        state.extend([0.0; 2]);
        for k in 0..2 {
            let joint = &self.joints[k];
            // At rest the gearbox holds the arm against the stops; gravity gives it nothing to hold.
            let out = -joint.output_torque(t[k], 0.0);
            let rotor = joint.ratio() * (t[k] + if self.elastic { joint.gearbox.static_twist(out) } else { 0.0 });
            if self.elastic {
                state.push(rotor);
            }
            torque[k] = joint.ripple.invert(out / joint.ratio(), rotor);
        }
        // Rotor speeds and brakes at rest.
        state.resize(self.planar_len(), 0.0);
        for (e, x) in self.extras.iter().enumerate() {
            // The stroke holds the carriage's weight; the turn holds nothing.
            let (q, i) = (tool[2 + e], x.ratio());
            let out = -x.weight(self.payload_attached) - x.joint.output_torque(q, 0.0);
            torque[2 + e] = x.joint.ripple.invert(out / i, q * i);
            state.extend([q, 0.0, 0.0]);
        }
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        for (e, x) in self.extras.iter().enumerate() {
            let (state, at) = (self.extra_state(s, e), self.planar_len() + 3 * e);
            out[at] = state[1];
            out[at + 1] = x.solve(state, torque[2 + e], self.brake_engaged, self.payload_attached).0;
            out[at + 2] = x.joint.brake.torque(self.brake_engaged, state[1] * x.ratio(), state[2]).1;
        }
        if !self.elastic {
            let (acc, ..) = self.rigid_solve(s, torque);
            let (_, rotor_speed) = self.rotors(s);
            let brakes = self.brakes(s);
            out[..2].copy_from_slice(&s[2..4]);
            out[2..4].copy_from_slice(&acc);
            for k in 0..2 {
                out[4 + k] = self.joints[k].brake.torque(self.brake_engaged, rotor_speed[k], brakes[k]).1;
            }
            return;
        }
        let (t, w) = self.arms(s);
        let (mass, velocity) = self.load(t, w);
        let mut rhs = [0.0; 2];
        for k in 0..2 {
            let joint = &self.joints[k];
            let (gear, rotor_acc, brake_rate) =
                joint.rotor(torque[k], s[4 + k], s[6 + k], t[k], w[k], s[8 + k], self.brake_engaged);
            rhs[k] = gear.output - velocity[k] + joint.output_torque(t[k], w[k]);
            out[4 + k] = s[6 + k];
            out[6 + k] = rotor_acc;
            out[8 + k] = brake_rate;
        }
        let acc = inverse(&mass).map_or([f64::NAN; 2], |inv| mul(&inv, rhs));
        out[..2].copy_from_slice(&s[2..4]);
        out[2..4].copy_from_slice(&acc);
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (t, w) = self.arms(s);
        let (p, j) = self.pose(t);
        pos[..2].copy_from_slice(&p);
        vel[..2].copy_from_slice(&mul(&j, w));
        for e in 0..self.extras.len() {
            (pos[2 + e], vel[2 + e]) = (self.extra_state(s, e)[0], self.extra_state(s, e)[1]);
        }
    }
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        let (t, _) = self.arms(s);
        let (_, j) = self.pose(t);
        let w = inverse(&j).map_or([f64::NAN; 2], |inv| mul(&inv, [vel[0], vel[1]]));
        s[2..4].copy_from_slice(&w);
        if self.elastic {
            for k in 0..2 {
                s[6 + k] = self.joints[k].ratio() * w[k];
            }
        }
        for e in 0..self.extras.len() {
            s[self.planar_len() + 3 * e + 1] = vel[2 + e];
        }
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (rotor, speed) = self.rotors(s);
        pos[..2].copy_from_slice(&rotor);
        vel[..2].copy_from_slice(&speed);
        for (e, x) in self.extras.iter().enumerate() {
            let state = self.extra_state(s, e);
            (pos[2 + e], vel[2 + e]) = (state[0] * x.ratio(), state[1] * x.ratio());
        }
    }

    /// Rigid, loss-free, at rest: M theta'' at the arm over the ratio, plus the rotor's own J_rotor i theta''; the Z
    /// stroke's and the turn's (m q'' - weight) / i + J_rotor i q''.
    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        let t = self.legs.inverse([pos[0], pos[1]]).unwrap_or([f64::NAN; 2]);
        let (mass, _) = self.load(t, [0.0; 2]);
        let (_, j) = self.pose(t);
        let arm_acc = inverse(&j).map_or([f64::NAN; 2], |inv| mul(&inv, [acc[0], acc[1]]));
        let at_arm = mul(&mass, arm_acc);
        for k in 0..2 {
            let joint = &self.joints[k];
            out[k] = at_arm[k] / joint.ratio() + joint.rotor_inertia * joint.ratio() * arm_acc[k];
        }
        for (e, x) in self.extras.iter().enumerate() {
            out[2 + e] = x.feedforward(acc[2 + e], self.payload_attached);
        }
    }

    /// With the speed: θ̇ = J⁻¹ ṗ, θ̈ = J⁻¹ (p̈ − J_x⁻¹ w), and at the arm I_b θ̈ + m_nt Jᵀ p̈, over the ratio,
    /// plus the rotor's own J_rotor i θ̈.
    fn feedforward_torque_moving(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut [f64]) {
        let p = [pos[0], pos[1]];
        let t = self.legs.inverse(p).unwrap_or([f64::NAN; 2]);
        let (_, j) = self.pose(t);
        let inv = inverse(&j).unwrap_or([[f64::NAN; 2]; 2]);
        let (pv, pa) = ([vel[0], vel[1]], [acc[0], acc[1]]);
        let w = mul(&inv, pv);
        let bend = self.legs.velocity_term(t, p, w, pv).unwrap_or([f64::NAN; 2]);
        let arm_acc = mul(&inv, [pa[0] - bend[0], pa[1] - bend[1]]);
        let force = transpose(&j).map(|column| self.m_nt() * dot(column, pa));
        for k in 0..2 {
            let joint = &self.joints[k];
            out[k] = (self.arm_inertia * arm_acc[k] + force[k]) / joint.ratio()
                + joint.rotor_inertia * joint.ratio() * arm_acc[k];
        }
        // The stroke and the turn are linear: nothing for the speed.
        for (e, x) in self.extras.iter().enumerate() {
            out[2 + e] = x.feedforward(acc[2 + e], self.payload_attached);
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
        let mut rises = rises.iter();
        if !self.joints[0].gearbox.is_coupling() {
            for (joint, rise) in self.joints.iter_mut().zip(rises.by_ref()) {
                joint.gearbox.efficiency = law.gearbox_efficiency(*rise);
            }
        }
        for x in self.extras.iter_mut().filter(|x| !x.joint.coupled) {
            if let (Some(law), Some(rise)) = (&x.thermal, rises.next()) {
                x.joint.gearbox.efficiency = law.gearbox_efficiency(*rise) * x.joint.extra_efficiency;
            }
        }
    }

    /// What the gearboxes can sag at rest anywhere in the working range (`rest_sag`).
    fn setpoint_slack(&self) -> f64 {
        self.rest_slack
    }

    fn unreachable(&self, tool: &[f64], slack: f64) -> Option<&'static str> {
        if tool.len() != self.axes.len() {
            return Some("error-target-out-of-range");
        }
        let p = [tool[0], tool[1]];
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
        for (k, motor) in self.motors[..2].iter().enumerate() {
            let arm = ["arm-1", "arm-2"][k];
            out.push(SignalInfo::new(format!("arm.{}.angle", motor.id), arm, "quantity-arm-angle", "rad"));
            out.push(SignalInfo::new(
                format!("gearbox.{}.output_torque", motor.id),
                motor.key,
                "quantity-gearbox-torque",
                "N m",
            ));
        }
        out.push(SignalInfo::new("kinematics.condition", "group-kinematics", "quantity-condition-number", ""));
        // The stroke's force on its nut and its screw's speed, d n and life; the turn's gearbox torque.
        for (e, x) in self.extras.iter().enumerate() {
            let (axis, motor) = (&self.axes[2 + e], &self.motors[2 + e]);
            match &x.screw {
                Some(screw) => {
                    let force = format!("joint.{}.force", axis.id);
                    out.push(SignalInfo::new(force, axis.key, "quantity-joint-force", "N"));
                    out.extend(screw.signal_infos(axis.id, axis.key));
                }
                None => out.push(SignalInfo::new(
                    format!("gearbox.{}.output_torque", motor.id),
                    motor.key,
                    "quantity-gearbox-torque",
                    "N m",
                )),
            }
        }
        self.effect_infos(&mut out);
        out
    }

    /// The stroke's ball screw: its speed, d n, buckling, static load and life.
    fn rating_checks(&self) -> Vec<RatingCheck> {
        let screws = self.extras.iter().enumerate().filter_map(|(e, x)| Some((e, x.screw.as_ref()?)));
        screws.flat_map(|(e, screw)| screw.rating_checks(self.axes[2 + e].id, 2 + e, self.required_life)).collect()
    }

    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (t, _) = self.arms(s);
        let gear = self.gear_output(s, torque);
        for k in 0..2 {
            out.extend([t[k], gear[k]]);
        }
        let (_, j) = self.pose(t);
        out.push(condition_number(&j));
        for (e, x) in self.extras.iter().enumerate() {
            let state = self.extra_state(s, e);
            let (.., force) = x.solve(state, torque[2 + e], self.brake_engaged, self.payload_attached);
            out.push(force);
            if let Some(screw) = &x.screw {
                screw.signals(state[1], force, out);
            }
        }
    }
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.view_signals(s, torque, out);
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(s, torque).powers);
        self.extra_losses(s, torque, out);
    }

    // shortcut: the joints' friction is not offered to the friction view (as on the D3), which draws a guide's force
    // over its speed; offer them once the view can draw a torque over an angular speed.

    /// The two gearboxes, where there are gearboxes (not on a direct drive); then the extra axes' (not behind a
    /// coupling), drawn after the arms'.
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        let planar = if self.joints[0].gearbox.is_coupling() {
            &[][..]
        } else {
            &[("gearbox_1", "motor-arm-1"), ("gearbox_2", "motor-arm-2")][..]
        };
        let extras = self.extras.iter().zip(&self.motors[2..]).filter(|(x, _)| !x.joint.coupled);
        let extras = extras.map(|(_, m)| (if m.id == "3" { "gearbox_3" } else { "gearbox_4" }, m.key));
        planar
            .iter()
            .copied()
            .zip(0..)
            .chain(extras.zip(2..))
            .map(|((id, owner_key), scene_index)| ThermalPart {
                id,
                kind: "gearbox",
                owner_key,
                scene_part: "gearbox",
                scene_index,
            })
            .collect()
    }

    /// The machine in the plane z = 0 of the scene: base beam, per arm a motor and a gearbox box above its shoulder with
    /// a bar turned by the rotor angle, the arm, the forearm pair's two rods (apart along z, the joints' axis), the
    /// plate; at the plate the quill down to the tool's flange, the payload under it, a bar across the flange turned
    /// with the tool, and on top the Z and C motors.
    fn scene(&self, s: &[f64], out: &mut Scene) {
        let (t, _) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let tool = self.tool_pose(s);
        let p = [tool[0], tool[1]];
        out.parts.clear();
        out.labels.clear();
        let grey = [0.62, 0.64, 0.68];
        out.parts.push(Part::new(
            "base",
            Shape::Box { size: [2.0 * self.base_radius + 0.2, 0.2, 0.04] },
            [0.0, 0.0, 0.1],
            grey,
        ));
        for k in 0..2 {
            let side = SIDE[k];
            let shoulder = [side * self.base_radius, 0.0, 0.0];
            // The elbow drawn where the arm ends: the legs' elbow is at e, the arm's end at the base radius.
            let [elbow, ..] = self.legs.elbow(k, t[k]);
            let arm_end = [elbow[0] + side * self.platform_radius, elbow[1], 0.0];
            let at = |z: f64| [side * self.base_radius, 0.0, z];
            out.parts.push(Part::new(
                sim_core::scene::MOTOR_PART,
                Shape::Box { size: [0.09, 0.09, 0.12] },
                at(0.16),
                [0.9, 0.45, 0.1],
            ));
            out.parts.push(Part::new("gearbox", Shape::Box { size: [0.09, 0.09, 0.06] }, at(0.05), [0.25, 0.25, 0.28]));
            out.parts.push(
                Part::new("rotor-bar", Shape::Box { size: [0.07, 0.004, 0.012] }, at(0.225), [0.1, 0.1, 0.1])
                    .turned(rotor[k]),
            );
            out.parts.push(Part::new("arm", Shape::Segment { to: arm_end, width: 0.04 }, shoulder, [0.2, 0.22, 0.26]));
            for half in [-0.5 * self.rod_spacing, 0.5 * self.rod_spacing] {
                let (from, to) = ([arm_end[0], arm_end[1], half], [p[0] + side * self.platform_radius, p[1], half]);
                out.parts.push(Part::new("forearm", Shape::Segment { to, width: 0.012 }, from, [0.15, 0.15, 0.15]));
            }
            out.labels.push(Label {
                pos: at(0.3),
                owner_key: self.motors[k].key,
                value: rotor_speed[k] * 60.0 / std::f64::consts::TAU,
                unit_key: "unit-rpm",
                decimals: 0,
            });
        }
        out.parts.extend(self.on_platform(&tool));
        let below = [p[0], p[1], self.flange_z(&tool) - 0.17];
        out.labels.push(Label {
            pos: below,
            owner_key: self.axes[1].key,
            value: p[1],
            unit_key: "unit-m",
            decimals: 3,
        });
        // The Z motor behind the plate and the C motor in front of it, on top, each with its gearbox where
        // it has one; the bar across the flange shows the tool's turn.
        for (e, x) in self.extras.iter().enumerate() {
            let k = 2 + e;
            let (state, i) = (self.extra_state(s, e), x.ratio());
            let at = |z: f64| [p[0], p[1] + if x.screw.is_some() { 0.08 } else { -0.08 }, z];
            out.parts.push(Part::new(
                sim_core::scene::MOTOR_PART,
                Shape::Box { size: [0.05, 0.05, 0.08] },
                at(0.13),
                [0.9, 0.45, 0.1],
            ));
            if !x.joint.coupled {
                out.parts.push(Part::new("gearbox", Shape::Box { size: [0.05, 0.05, 0.04] }, at(0.07), [0.25; 3]));
            }
            out.parts.push(
                Part::new("rotor-bar", Shape::Box { size: [0.04, 0.004, 0.01] }, at(0.18), [0.1; 3])
                    .turned(state[0] * i),
            );
            out.labels.push(Label {
                pos: at(0.23),
                owner_key: self.motors[k].key,
                value: state[1] * i * 60.0 / std::f64::consts::TAU,
                unit_key: "unit-rpm",
                decimals: 0,
            });
            if x.screw.is_none() {
                let (c, flange) = (state[0], self.flange_z(&tool) - 0.025);
                let (dx, dy) = (0.05 * c.cos(), 0.05 * c.sin());
                let to = [p[0] + dx, p[1] + dy, flange];
                out.parts.push(Part::new(
                    "tool-bar",
                    Shape::Segment { to, width: 0.01 },
                    [p[0] - dx, p[1] - dy, flange],
                    [0.9, 0.8, 0.1],
                ));
            }
        }
    }

    /// The underside of the tool's flange.
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        Some([tool_pose[0], tool_pose[1], self.flange_z(tool_pose) - 0.02])
    }

    /// The plate, the quill, the tool's flange and the payload under it, where `scene` draws them. The forearms are left
    /// out.
    fn collision_bodies(&self, s: &[f64]) -> Vec<sim_core::Aabb> {
        sim_core::scene::scene_boxes(&self.on_platform(&self.tool_pose(s)), &["platform", "quill", "tool", "payload"])
    }
}

#[cfg(test)]
mod tests;
