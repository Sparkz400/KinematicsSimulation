//! `LinkageMachine`: a planar parallel linkage built from a machine file's `[[machine.linkage]]` node: two motor-driven
//! arms in the vertical x-z plane, one at each side of the base, each carrying a pair of forearms down to a plate that
//! keeps its orientation (a five-bar whose lower links are parallelograms: the planar delta). The tool is the plate's
//! centre; the node's two axes are its x (across) and z (up).
//!
//! The registered planar delta's equations, for any lengths and masses the node names (the crate's module comment
//! derives them). Arm k points along s_k x (s_1 = +1, s_2 = -1); e = base radius - platform radius, the arm angle
//! theta_k (0 horizontal, positive down), upper arm L_a, forearm L_b:
//!
//! ```text
//! elbow     c_k = (s_k (e + L_a cos theta), -L_a sin theta)
//! inverse   A cos theta + B sin theta + C = 0, A = -2 L_a (x' - e), B = 2 L_a z, C = (x' - e)^2 + z^2 + L_a^2 - L_b^2
//!           theta = 2 atan((-B - sqrt(A^2 + B^2 - C^2)) / (C - A))     (elbow out; x' = s_k x)
//! forward   p = the lower of the two points at L_b from both elbows
//! Jacobian  J_x p' = J_q theta', J_x rows u_k = p - c_k, J_q = diag(u_k . dc_k/dtheta_k); J = J_x^-1 J_q
//! dynamics  M(theta) theta'' = tau_gear + Q_g - m_nt J^T J_x^-1 w + tau_joint,   M = I_b 1 + m_nt J^T J
//!           I_b = I_arm + L_a^2 (m_elbow + r m_fb),   Q_g,k = g m_LG cos theta_k - m_nt g J_zk
//!           m_LG = m_arm l_cg + L_a (m_elbow + r m_fb),   m_nt = plate + payload + 2 (1 - r) m_fb
//! ```
//!
//! Each arm's joint is a `RotaryJoint` read from its motor's own config (`catalog::motor_config`: `[motor]`,
//! `[gearbox]` with `[motor.<id>]`, `[gearbox.<id>]` over them) and the shared `[joint]` (bearing friction, stops),
//! elastic or rigid with effects.joint_elasticity. The machine is the crate's, line for line, so that the file
//! `machines/planar-delta.toml` gives the crate's numbers (crates/sim-machines/tests/machine_file_linkage.rs).

use crate::machine::intern;
use crate::netlist::{AxisNode, Kind, MotorNode, NODES, Netlist, Node, Transmission, number};
use sim_components::catalog::motor_config;
use sim_components::joint::RotaryJoint;
use sim_components::mech::ThermalFeedback;
use sim_core::{
    AxisInfo, Config, ConfigError, ENERGY_PARTS, LOSS_PREFIX, Label, Machine, MotorInfo, Part, Scene, Shape,
    SignalInfo, TORQUE_PARTS, ThermalPart,
};
use toml::Value;

const G: f64 = 9.80665;

/// Values of the node that have to be above 0; `top_z_m` may be anything, every other value at least 0. The range's
/// half width is the planar linkage's, its radius the spatial one's.
const POSITIVE: [&str; 11] = [
    "base_radius_m",
    "upper_arm_m",
    "forearm_m",
    "upper_arm_kg",
    "upper_arm_inertia_kgm2",
    "platform_kg",
    "half_width_m",
    "radius_m",
    "height_edge_m",
    "height_centre_m",
    "max_condition_number",
];

/// Values only the spatial linkage (three arms) reads: the range's radius, the rods' spin, the upper arms' bending.
pub const SPATIAL_ONLY: [&str; 6] = [
    "radius_m",
    "forearm_rod_spin_inertia_kgm2",
    "bending_stiffness_nm_rad",
    "bending_damping_nm_s_rad",
    "lateral_stiffness_n_m",
    "lateral_damping_n_s_m",
];

/// The linkage node, read and checked: its number fields by name (an optional one left out is 0), and per arm the
/// gearbox node's id (arm k's motor is the netlist's motor k). Two arms: the planar linkage (`LinkageMachine`); three:
/// the spatial one (`crate::spatial::Delta`).
pub struct LinkageNode {
    pub id: String,
    pub gearboxes: Vec<String>,
    /// The node's path in the file, `machine.linkage[<id>]`, for errors.
    path: String,
    values: Vec<(&'static str, f64)>,
}

impl LinkageNode {
    /// The value of a number field of the node.
    pub fn value(&self, name: &str) -> f64 {
        self.values.iter().find(|(n, _)| *n == name).map_or(0.0, |(_, v)| *v)
    }
}

/// Where a linkage reads its numbers: a crate's tables by their paths (`geometry.forearm_m`), or a file's linkage node
/// by the path's last part (`forearm_m`), so that one constructor serves both.
pub trait Numbers {
    /// Any finite number.
    fn number(&self, path: &str) -> Result<f64, ConfigError>;
    /// The error that refuses the value at `path`.
    fn invalid(&self, path: &str) -> ConfigError;
    fn positive(&self, path: &str) -> Result<f64, ConfigError> {
        let v = self.number(path)?;
        if v > 0.0 { Ok(v) } else { Err(self.invalid(path)) }
    }
    fn non_negative(&self, path: &str) -> Result<f64, ConfigError> {
        let v = self.number(path)?;
        if v >= 0.0 { Ok(v) } else { Err(self.invalid(path)) }
    }
}

impl Numbers for Config {
    fn number(&self, path: &str) -> Result<f64, ConfigError> {
        Config::f64(self, path)
    }
    fn invalid(&self, path: &str) -> ConfigError {
        ConfigError::Invalid(path.into())
    }
}

impl Numbers for LinkageNode {
    fn number(&self, path: &str) -> Result<f64, ConfigError> {
        Ok(self.value(path.rsplit('.').next().unwrap_or(path)))
    }
    fn invalid(&self, path: &str) -> ConfigError {
        ConfigError::Invalid(format!("{}.{}", self.path, path.rsplit('.').next().unwrap_or(path)))
    }
}

/// A linkage's drives and names: per arm its joint, its motor and its gearbox's id for its warmth (`gearbox_<motor>`, as
/// its loss is named); per axis of the tool its id and key.
pub struct Arms<const N: usize> {
    pub joints: [RotaryJoint; N],
    pub motors: [MotorInfo; N],
    pub gearboxes: [&'static str; N],
    pub axes: [(&'static str, &'static str); N],
}

impl<const N: usize> Arms<N> {
    /// A crate's: one joint for every arm from `[joint]`, `[motor]` and `[gearbox]`; motors `1`, `2`, ... (keys
    /// `motor-arm-k`), gearboxes `gearbox_k`; the axes as given.
    pub fn shared(cfg: &Config, axes: [(&'static str, &'static str); N]) -> Result<Self, ConfigError> {
        let joint = RotaryJoint::from_config(cfg)?;
        Ok(Self {
            joints: std::array::from_fn(|_| joint.clone()),
            motors: std::array::from_fn(|k| MotorInfo {
                id: ["1", "2", "3"][k],
                key: ["motor-arm-1", "motor-arm-2", "motor-arm-3"][k],
            }),
            gearboxes: std::array::from_fn(|k| ["gearbox_1", "gearbox_2", "gearbox_3"][k]),
            axes,
        })
    }

    /// A file's: per arm the joint of its motor's own config (`catalog::motor_config`), the netlist's motors and axes.
    pub fn of_file(cfg: &Config, net: &Netlist) -> Result<Self, ConfigError> {
        let joints = (0..N)
            .map(|k| RotaryJoint::from_config(&motor_config(cfg, &net.motors[k].id)?))
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self {
            joints: joints.try_into().unwrap_or_else(|_| unreachable!("N joints")),
            motors: std::array::from_fn(|k| MotorInfo {
                id: intern(&net.motors[k].id),
                key: intern(&net.motors[k].key),
            }),
            // The warmth's id is that of the gearbox's loss, `power.gearbox_<motor>`.
            gearboxes: std::array::from_fn(|k| intern(&format!("gearbox_{}", net.motors[k].id))),
            axes: std::array::from_fn(|k| (intern(&net.axes[k].id), intern(&net.axes[k].key))),
        })
    }

    /// Axis k's limits from `[axis.<id>]`, its range `min` to `max`, in m.
    pub fn axis(&self, cfg: &Config, k: usize, min: f64, max: f64) -> Result<AxisInfo, ConfigError> {
        let (id, key) = self.axes[k];
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
    }
}

/// The netlist of a file with a linkage: its axes (across, then up; or x, y, z), its arms' motors in the order of the
/// node's `arms`, and the linkage. Two arms make the planar linkage, three the spatial one. Such a file has axes,
/// motors, gearboxes and one linkage, nothing else (`others`: the other node lists), and no belt effect switched on.
pub(crate) fn read(
    cfg: &Config,
    axes: &[Node],
    motors: &[Node],
    gearboxes: &[Node],
    linkages: &[Node],
    others: &[&[Node]],
) -> Result<Netlist, ConfigError> {
    let node = match linkages {
        [node] => node,
        _ => return Err(ConfigError::Invalid(format!("{} (one linkage per file)", linkages[1].path))),
    };
    if let Some(other) = others.iter().find_map(|nodes| nodes.first()) {
        return Err(ConfigError::Invalid(format!(
            "{} (a file with a linkage has axes, motors, gearboxes and the linkage only)",
            other.path
        )));
    }
    // A linkage has no belt: a belt effect switched on would otherwise be ignored without a word.
    let effects = cfg.get("effects").ok().and_then(Value::as_table);
    for (effect, on) in effects.into_iter().flatten() {
        if effect.starts_with("belt_") && on.as_bool() == Some(true) {
            return Err(ConfigError::Invalid(format!("effects.{effect} (a linkage has no belt: switch it off)")));
        }
    }
    let id_of = |n: &Node| n.table["id"].as_str().unwrap_or_default().to_string();
    let by_id = |nodes: &'_ [Node], field: &str, id: &str| -> Result<usize, ConfigError> {
        nodes
            .iter()
            .position(|n| n.table["id"].as_str() == Some(id))
            .ok_or_else(|| ConfigError::Invalid(format!("{} = \"{id}\"", node.at(field))))
    };
    // Two or three of each, as many as the arms, every one named once.
    let arm_count = node.id_list("arms")?.len();
    let all = |field: &str, nodes: &[Node], list: &str| -> Result<Vec<usize>, ConfigError> {
        let ids = node.id_list(field)?;
        let found = ids.iter().map(|id| by_id(nodes, field, id)).collect::<Result<Vec<_>, _>>()?;
        let distinct = found.iter().enumerate().all(|(i, a)| !found[..i].contains(a));
        if (2..=3).contains(&found.len()) && found.len() == arm_count && distinct && nodes.len() == found.len() {
            Ok(found)
        } else {
            Err(ConfigError::Invalid(format!(
                "{} (two or three machine.{list} nodes, as many as the arms, each once; the file has {})",
                node.at(field),
                nodes.len()
            )))
        }
    };
    let spatial = arm_count == 3;

    let fields = NODES.iter().find(|(list, ..)| *list == "linkage").map_or(&[][..], |(_, _, f)| *f);
    let mut refs = Vec::new();
    let mut values = Vec::new();
    for f in fields.iter().filter(|f| f.kind == Kind::Number) {
        // The planar linkage's range has a half width, the spatial one's a radius; each refuses the other's values.
        let own = if spatial { f.name != "half_width_m" } else { !SPATIAL_ONLY.contains(&f.name) };
        if !own {
            if node.get(f.name).is_some() {
                let arms = if spatial { "three" } else { "two" };
                return Err(ConfigError::Invalid(format!("{} (not for a linkage of {arms} arms)", node.at(f.name))));
            }
            continue;
        }
        let v = node.get(f.name).map(|v| number(cfg, v, &node.at(f.name), &mut refs)).transpose()?;
        // Required by one kind of linkage only, so not marked required in `NODES`.
        let wanted =
            ["half_width_m", "radius_m", "bending_stiffness_nm_rad", "lateral_stiffness_n_m"].contains(&f.name);
        let v = match v {
            Some(v) => v,
            None if wanted => return Err(ConfigError::Missing(node.at(f.name))),
            None => 0.0,
        };
        let ok = if POSITIVE.contains(&f.name) { v > 0.0 } else { f.name == "top_z_m" || v >= 0.0 };
        if !ok {
            return Err(ConfigError::Invalid(node.at(f.name)));
        }
        values.push((f.name, v));
    }
    let value = |name: &str| values.iter().find(|(n, _)| *n == name).map_or(0.0, |(_, v)| *v);
    for (field, bad, why) in [
        ("forearm_m", value("forearm_m") <= value("upper_arm_m"), "longer than the upper arm"),
        ("forearm_elbow_share", value("forearm_elbow_share") > 1.0, "a share, 0 to 1"),
        ("height_centre_m", value("height_centre_m") < value("height_edge_m"), "at least height_edge_m"),
    ] {
        if bad {
            return Err(ConfigError::Invalid(format!("{} ({why})", node.at(field))));
        }
    }

    let mut axis_nodes = Vec::new();
    let tool_axes = all("axes", axes, "axis")?;
    let count = tool_axes.len();
    for (k, i) in tool_axes.into_iter().enumerate() {
        let axis = &axes[i];
        for field in ["stroke_m", "stroke_rad", "incline_deg", "cable_chain"] {
            if axis.get(field).is_some() {
                return Err(ConfigError::Invalid(format!(
                    "{} (a linkage's axes take their range from the linkage's working range)",
                    axis.at(field)
                )));
            }
        }
        let id = id_of(axis);
        let key = axis.text("key")?.map_or_else(|| format!("axis-{id}"), str::to_string);
        // The range across (each way across on the spatial linkage), and up from the bottom at the centre.
        let stroke = match (k + 1 == count, spatial) {
            (true, _) => value("height_centre_m"),
            (false, false) => 2.0 * value("half_width_m"),
            (false, true) => 2.0 * value("radius_m"),
        };
        axis_nodes.push(AxisNode { id, key, stroke, incline_deg: 0.0, cable_chain: false, rotary: false });
    }

    let mut motor_nodes: Vec<MotorNode> = Vec::new();
    let arms = all("arms", gearboxes, "gearbox")?;
    for &i in &arms {
        let gearbox = &gearboxes[i];
        let motor = gearbox.text("motor")?.unwrap_or_default();
        let m = motors
            .iter()
            .position(|n| n.table["id"].as_str() == Some(motor))
            .ok_or_else(|| ConfigError::Invalid(format!("{} = \"{motor}\"", gearbox.at("motor"))))?;
        let id = id_of(&motors[m]);
        let key = motors[m].text("key")?.map_or_else(|| format!("motor-{id}"), str::to_string);
        if motor_nodes.iter().any(|o| o.id == id) {
            break;
        }
        // What drives the arm is the linkage, which the netlist's transmissions do not describe: nothing reads these.
        motor_nodes.push(MotorNode {
            id,
            key,
            radius: 1.0,
            inertia: 0.0,
            pulley: String::new(),
            at: None,
            clockwise: false,
            transmission: Transmission::Belt,
            axis: None,
            worm: None,
        });
    }
    if motors.len() != arms.len() || motor_nodes.len() != arms.len() {
        return Err(ConfigError::Invalid(format!("{} (one motor per arm, each with its gearbox)", node.at("arms"))));
    }
    Ok(Netlist {
        axes: axis_nodes,
        motors: motor_nodes,
        sections: Vec::new(),
        bodies: Vec::new(),
        guides: Vec::new(),
        idlers: Vec::new(),
        frame: None,
        spans: Vec::new(),
        lean: None,
        refs,
        values: Vec::new(),
        linkage: Some(LinkageNode {
            id: id_of(node),
            gearboxes: arms.iter().map(|&i| id_of(&gearboxes[i])).collect(),
            path: node.path.clone(),
            values,
        }),
        scissor: None,
    })
}

/// Effects that take energy out or put it in, as on the registered planar delta: the standard suite runs with them
/// off.
pub const LOSSY_EFFECTS: [&str; 7] = [
    "joint_friction",
    "motor_friction",
    "gearbox_drag",
    "gearbox_lost_motion",
    "gearbox_efficiency",
    "torque_ripple",
    "holding_brake",
];

/// A vector in the x-z plane, (x, z).
pub type V2 = [f64; 2];
pub type M2 = [[f64; 2]; 2];

pub fn dot(a: V2, b: V2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

pub fn sub(a: V2, b: V2) -> V2 {
    [a[0] - b[0], a[1] - b[1]]
}

pub fn norm(a: V2) -> f64 {
    dot(a, a).sqrt()
}

pub fn mul(m: &M2, v: V2) -> V2 {
    [dot(m[0], v), dot(m[1], v)]
}

pub fn transpose(m: &M2) -> M2 {
    [[m[0][0], m[1][0]], [m[0][1], m[1][1]]]
}

/// The inverse of a 2 x 2 matrix; `None` where it is singular.
pub fn inverse(m: &M2) -> Option<M2> {
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

/// The side each arm points to along x.
pub const SIDE: [f64; 2] = [1.0, -1.0];

/// The legs' geometry and the kinematics, apart from masses and drives.
#[derive(Debug, Clone)]
pub struct Legs {
    /// Base radius less platform radius, upper arm, forearm.
    pub e: f64,
    pub la: f64,
    pub lb: f64,
}

impl Legs {
    pub fn new(e: f64, la: f64, lb: f64) -> Self {
        Self { e, la, lb }
    }

    /// The elbow of leg i at arm angle t, and its first and second derivative by t, as (x, z).
    pub fn elbow(&self, i: usize, t: f64) -> [V2; 3] {
        let (s, c) = t.sin_cos();
        let (la, side) = (self.la, SIDE[i]);
        [[side * (self.e + la * c), -la * s], [-side * la * s, -la * c], [-side * la * c, la * s]]
    }

    /// Arm angles for a plate position (x, z), elbows out; `None` out of reach.
    pub fn inverse(&self, p: V2) -> Option<V2> {
        let mut out = [0.0; 2];
        for (i, t) in out.iter_mut().enumerate() {
            let (x, z) = (SIDE[i] * p[0], p[1]);
            let (a, b, c) = (
                -2.0 * self.la * (x - self.e),
                2.0 * self.la * z,
                (x - self.e).powi(2) + z * z + self.la * self.la - self.lb * self.lb,
            );
            let d = a * a + b * b - c * c;
            if !(d >= 0.0) {
                return None;
            }
            *t = 2.0 * ((-b - d.sqrt()) / (c - a)).atan();
        }
        Some(out)
    }

    /// The plate position for arm angles: the lower point at a forearm's length from both elbows. `None` where the
    /// legs cannot be assembled.
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

    /// The Jacobian J[i][j] = dp_i / dtheta_j (rows x and z); `None` at a direct singularity.
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

pub struct LinkageMachine {
    legs: Legs,
    /// The geometry as configured, for the scene: base and platform radius, the rods' spacing.
    base_radius: f64,
    platform_radius: f64,
    rod_spacing: f64,
    elastic: bool,
    g: f64,
    /// I_b without the rotor, m_LG, plate mass without and with payload (forearm share included).
    arm_inertia: f64,
    arm_moment: f64,
    m_platform: f64,
    m_payload: f64,
    payload_attached: bool,
    brake_engaged: bool,
    joints: [RotaryJoint; 2],
    thermal: Option<ThermalFeedback>,
    /// The working range: half width, top, bottom circle's centre height and radius; the condition number refused above.
    half_width: f64,
    top: f64,
    circle: (f64, f64),
    max_condition: f64,
    axes: [AxisInfo; 2],
    motors: [MotorInfo; 2],
    /// The gearbox nodes' ids, for their warmth.
    gearbox_ids: [&'static str; 2],
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

impl LinkageMachine {
    /// The plate at `p` (x, z) and the payload under it, as `scene` draws them.
    fn on_platform(&self, p: V2) -> Vec<Part> {
        let size = 2.4 * self.platform_radius.max(0.03);
        let mut parts = vec![Part::new(
            "platform",
            Shape::Box { size: [size, self.rod_spacing.max(0.06) + 0.03, 0.03] },
            [p[0], 0.0, p[1]],
            [0.55, 0.55, 0.6],
        )];
        if self.payload_attached && self.m_payload > 0.0 {
            parts.push(Part::new(
                "payload",
                Shape::Box { size: [0.12, 0.12, 0.1] },
                [p[0], 0.0, p[1] - 0.07],
                [0.2, 0.7, 0.3],
            ));
        }
        parts
    }

    /// The planar linkage of a crate's tables: `[geometry]`, `[mass]`, `[workspace]`, one `[joint]` for both arms, the
    /// axes `x` and `z`.
    pub fn new(cfg: &Config) -> Result<Self, ConfigError> {
        Self::build(cfg, cfg, Arms::shared(cfg, [("x", "axis-x"), ("z", "axis-z")])?)
    }

    /// The planar linkage of a machine file's node.
    pub fn from_node(cfg: &Config, net: &Netlist, node: &LinkageNode) -> Result<Self, ConfigError> {
        Self::build(cfg, node, Arms::of_file(cfg, net)?)
    }

    /// The machine from its numbers (`n`, by their paths in a crate's tables) and its drives; the effects from `cfg`.
    pub fn build(cfg: &Config, n: &dyn Numbers, arms: Arms<2>) -> Result<Self, ConfigError> {
        let base_radius = n.positive("geometry.base_radius_m")?;
        let platform_radius = n.non_negative("geometry.platform_radius_m")?;
        let (la, lb) = (n.positive("geometry.upper_arm_m")?, n.positive("geometry.forearm_m")?);
        if lb <= la {
            return Err(n.invalid("geometry.forearm_m"));
        }
        let share = n.non_negative("mass.forearm_elbow_share")?;
        if share > 1.0 {
            return Err(n.invalid("mass.forearm_elbow_share"));
        }
        let (m_arm, cg, i_arm) = (
            n.positive("mass.upper_arm_kg")?,
            n.non_negative("mass.upper_arm_cg_m")?,
            n.positive("mass.upper_arm_inertia_kgm2")?,
        );
        let (m_elbow, m_fb) = (n.non_negative("mass.elbow_kg")?, n.non_negative("mass.forearm_pair_kg")?);
        let elbow = m_elbow + share * m_fb;
        let (half_width, top) = (n.positive("workspace.half_width_m")?, n.number("workspace.top_z_m")?);
        let (edge, centre) = (n.positive("workspace.height_edge_m")?, n.positive("workspace.height_centre_m")?);
        if centre < edge {
            return Err(n.invalid("workspace.height_centre_m"));
        }
        // The bottom: a circle through the centre's depth and both edges; flat where both heights are the same.
        let sag = centre - edge;
        let circle = if sag > 0.0 {
            let r = (half_width * half_width + sag * sag) / (2.0 * sag);
            (top - centre + r, r)
        } else {
            (f64::INFINITY, f64::INFINITY)
        };
        let mut machine = Self {
            legs: Legs::new(base_radius - platform_radius, la, lb),
            base_radius,
            platform_radius,
            rod_spacing: n.non_negative("geometry.forearm_spacing_m")?,
            elastic: cfg.bool("effects.joint_elasticity")?,
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            arm_inertia: i_arm + la * la * elbow,
            arm_moment: m_arm * cg + la * elbow,
            m_platform: n.positive("mass.platform_kg")? + 2.0 * (1.0 - share) * m_fb,
            m_payload: if cfg.bool("effects.payload")? { n.non_negative("mass.payload_kg")? } else { 0.0 },
            payload_attached: true,
            brake_engaged: false,
            thermal: ThermalFeedback::from_config(cfg)?,
            half_width,
            top,
            circle,
            max_condition: n.positive("workspace.max_condition_number")?,
            axes: [arms.axis(cfg, 0, -half_width, half_width)?, arms.axis(cfg, 1, top - centre, top)?],
            motors: arms.motors,
            gearbox_ids: arms.gearboxes,
            joints: arms.joints,
            rest_slack: 0.0,
        };
        machine.rest_slack = machine.rest_sag();
        Ok(machine)
    }

    /// How far the motors' pose can be from the tool's at rest anywhere in the working range, payload on or off: each
    /// gearbox twisted by the torque it holds against gravity over its stiffness, plus half its backlash either way
    /// (all four combinations), through the forward kinematics. Over a grid of the range, 41 columns of 9 heights and
    /// the bottom; plus a tenth for what the grid steps over and the joints' friction. At least 0.1 mm; that alone in
    /// the rigid model, whose gearboxes do not twist.
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
                let Some(j) = self.legs.jacobian(t, p) else { continue };
                for m in [self.m_platform, self.m_platform + self.m_payload] {
                    let twist: V2 = std::array::from_fn(|k| {
                        let held = -(self.g * self.arm_moment * t[k].cos() - m * self.g * j[1][k])
                            - self.joints[k].output_torque(t[k], 0.0);
                        self.joints[k].gearbox.spring_twist_signed(held)
                    });
                    for signs in 0..4 {
                        let half = |k: usize| {
                            (if signs >> k & 1 == 1 { 1.0 } else { -1.0 }) * self.joints[k].gearbox.half_backlash
                        };
                        if let Some(q) = self.legs.forward(std::array::from_fn(|k| t[k] + twist[k] + half(k))) {
                            most = most.max(norm(sub(q, p)));
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

    /// The point mass at the plate.
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
    fn arms(&self, s: &[f64]) -> (V2, V2) {
        ([s[0], s[1]], [s[2], s[3]])
    }

    /// Rotor angles and speeds of a state.
    fn rotors(&self, s: &[f64]) -> (V2, V2) {
        if self.elastic {
            ([s[4], s[5]], [s[6], s[7]])
        } else {
            let i: V2 = std::array::from_fn(|k| self.joints[k].ratio());
            ([s[0] * i[0], s[1] * i[1]], [s[2] * i[0], s[3] * i[1]])
        }
    }

    fn brakes(&self, s: &[f64]) -> V2 {
        let at = if self.elastic { 8 } else { 4 };
        [s[at], s[at + 1]]
    }

    /// Where the plate is and the Jacobian there, for arm angles; NaN where the legs cannot be assembled.
    pub fn pose(&self, t: V2) -> (V2, M2) {
        self.legs
            .forward(t)
            .and_then(|p| Some((p, self.legs.jacobian(t, p)?)))
            .unwrap_or(([f64::NAN; 2], [[f64::NAN; 2]; 2]))
    }

    /// The load's mass matrix M, the velocity term m_nt J^T J' theta', and the generalised gravity Q_g at arm angles t
    /// and speeds w.
    fn load(&self, t: V2, w: V2) -> (M2, V2, V2) {
        let (p, j) = self.pose(t);
        let jt = transpose(&j);
        let m = self.m_nt();
        let mass: M2 = std::array::from_fn(|r| {
            std::array::from_fn(|c| m * dot(jt[r], jt[c]) + if r == c { self.arm_inertia } else { 0.0 })
        });
        let pv = mul(&j, w);
        let velocity = self.legs.velocity_term(t, p, w, pv).map_or([f64::NAN; 2], |a| mul(&jt, a.map(|v| m * v)));
        let gravity: V2 = std::array::from_fn(|k| self.g * self.arm_moment * t[k].cos() - m * self.g * j[1][k]);
        (mass, velocity, gravity)
    }

    /// Rigid model: the arms' acceleration, and per joint the torque into its gearbox (after the rotor's own
    /// acceleration) and out of it, at the arm.
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> (V2, V2, V2) {
        let (t, w) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let brakes = self.brakes(s);
        let (mass, velocity, gravity) = self.load(t, w);
        let drive: V2 = std::array::from_fn(|k| {
            self.joints[k].rigid_drive(torque[k], rotor[k], rotor_speed[k], brakes[k], self.brake_engaged).0
        });
        let rest: V2 = std::array::from_fn(|k| gravity[k] - velocity[k] + self.joints[k].output_torque(t[k], w[k]));
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
            let mut d = [0.0; N_ELASTIC];
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

    pub fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        let (t, w) = self.arms(s);
        let (mass, ..) = self.load(t, w);
        let mut kinetic = 0.5 * dot(w, mul(&mass, w));
        let (p, _) = self.pose(t);
        let potential = self.g * (self.m_nt() * p[1] - self.arm_moment * (t[0].sin() + t[1].sin()));
        let mut elastic: f64 = (0..2).map(|k| self.joints[k].stop_energy(t[k])).sum();
        if self.elastic {
            for k in 0..2 {
                kinetic += 0.5 * self.joints[k].rotor_inertia * s[6 + k] * s[6 + k];
                elastic += self.joints[k].gearbox.energy(s[4 + k], s[k]);
            }
        } else {
            kinetic += 0.5 * (0..2).map(|k| self.joints[k].reflected_inertia() * w[k] * w[k]).sum::<f64>();
        }
        [kinetic, potential, elastic]
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
        let (mass, velocity, gravity) = self.load(t, w);
        let acc = self.arm_acceleration(s, torque);
        let moving = mul(&mass, acc);
        let losses = self.losses(s, torque);
        for k in 0..2 {
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
            let arm = ["arm-1", "arm-2"][k];
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
        out.extend(ENERGY_PARTS.map(|part| {
            SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")
        }));
    }

    /// The arms whose gearbox is one (not a coupling), in the order of `thermal_parts`.
    fn geared(&self) -> impl Iterator<Item = usize> + '_ {
        (0..2).filter(|&k| !self.joints[k].gearbox.is_coupling())
    }
}

impl Machine for LinkageMachine {
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
            motors[k] = self.joints[k].ratio() * t[k];
        }
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        let t = std::array::from_fn(|k| motors[k] / self.joints[k].ratio());
        let p = self.legs.forward(t).unwrap_or([f64::NAN; 2]);
        tool[..2].copy_from_slice(&p);
    }

    fn state_len(&self) -> usize {
        if self.elastic { N_ELASTIC } else { N_RIGID }
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let t = self.legs.inverse([tool[0], tool[1]]).unwrap_or([f64::NAN; 2]);
        let (_, _, gravity) = self.load(t, [0.0; 2]);
        state.clear();
        state.extend(t);
        state.extend([0.0; 2]);
        for k in 0..2 {
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
            out[..2].copy_from_slice(&s[2..4]);
            out[2..4].copy_from_slice(&acc);
            for k in 0..2 {
                out[4 + k] = self.joints[k].brake.torque(self.brake_engaged, rotor_speed[k], brakes[k]).1;
            }
            return;
        }
        let (t, w) = self.arms(s);
        let (mass, velocity, gravity) = self.load(t, w);
        let mut rhs = [0.0; 2];
        for k in 0..2 {
            let joint = &self.joints[k];
            let (gear, rotor_acc, brake_rate) =
                joint.rotor(torque[k], s[4 + k], s[6 + k], t[k], w[k], s[8 + k], self.brake_engaged);
            rhs[k] = gear.output + gravity[k] - velocity[k] + joint.output_torque(t[k], w[k]);
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
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let (rotor, speed) = self.rotors(s);
        pos[..2].copy_from_slice(&rotor);
        vel[..2].copy_from_slice(&speed);
    }

    /// Rigid, loss-free, at rest: M theta'' - Q_g at the arm over the ratio, plus the rotor's own J_rotor i theta''.
    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        let t = self.legs.inverse([pos[0], pos[1]]).unwrap_or([f64::NAN; 2]);
        let (mass, _, gravity) = self.load(t, [0.0; 2]);
        let (_, j) = self.pose(t);
        let arm_acc = inverse(&j).map_or([f64::NAN; 2], |inv| mul(&inv, [acc[0], acc[1]]));
        let at_arm = mul(&mass, arm_acc);
        for k in 0..2 {
            let joint = &self.joints[k];
            out[k] = (at_arm[k] - gravity[k]) / joint.ratio() + joint.rotor_inertia * joint.ratio() * arm_acc[k];
        }
    }

    /// With the speed: theta' = J^-1 p', theta'' = J^-1 (p'' - J_x^-1 w), and at the arm I_b theta'' + m_nt J^T p'' -
    /// Q_g, over the ratio, plus the rotor's own J_rotor i theta''.
    fn feedforward_torque_moving(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut [f64]) {
        let p = [pos[0], pos[1]];
        let t = self.legs.inverse(p).unwrap_or([f64::NAN; 2]);
        let (_, _, gravity) = self.load(t, [0.0; 2]);
        let (_, j) = self.pose(t);
        let inv = inverse(&j).unwrap_or([[f64::NAN; 2]; 2]);
        let (pv, pa) = ([vel[0], vel[1]], [acc[0], acc[1]]);
        let w = mul(&inv, pv);
        let bend = self.legs.velocity_term(t, p, w, pv).unwrap_or([f64::NAN; 2]);
        let arm_acc = mul(&inv, [pa[0] - bend[0], pa[1] - bend[1]]);
        let force = transpose(&j).map(|column| self.m_nt() * dot(column, pa));
        for k in 0..2 {
            let joint = &self.joints[k];
            out[k] = (self.arm_inertia * arm_acc[k] + force[k] - gravity[k]) / joint.ratio()
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
        let geared: Vec<usize> = self.geared().collect();
        for (k, rise) in geared.into_iter().zip(rises) {
            self.joints[k].gearbox.efficiency = law.gearbox_efficiency(*rise);
            self.joints[k].shaft.set_oil(law.viscosity_factor(*rise));
        }
    }

    /// What the gearboxes can sag at rest anywhere in the working range (`rest_sag`).
    fn setpoint_slack(&self) -> f64 {
        self.rest_slack
    }

    fn unreachable(&self, tool: &[f64], slack: f64) -> Option<&'static str> {
        if tool.len() != 2 {
            return Some("error-target-out-of-range");
        }
        let p = [tool[0], tool[1]];
        if !self.in_envelope(&p, slack) {
            return Some("error-delta-outside-envelope");
        }
        let Some(t) = self.legs.inverse(p) else { return Some("error-delta-out-of-reach") };
        if (0..2).any(|k| t[k] < self.joints[k].range.0 || t[k] > self.joints[k].range.1) {
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
        self.effect_infos(&mut out);
        out
    }

    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (t, _) = self.arms(s);
        let gear = self.gear_output(s, torque);
        for k in 0..2 {
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

    // shortcut: the joints' friction is not offered to the friction view, which draws a guide's force over its speed;
    // offer them once the view can draw a torque over an angular speed.

    /// The gearboxes that are not couplings, by their nodes' ids.
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        self.geared()
            .map(|k| ThermalPart {
                id: self.gearbox_ids[k],
                kind: "gearbox",
                owner_key: self.motors[k].key,
                scene_part: "gearbox",
                scene_index: k,
            })
            .collect()
    }

    /// The machine in the plane y = 0 of the scene: base beam, per arm a motor and a gearbox box with a bar turned by
    /// the rotor angle, the arm, the forearm pair's two rods (apart along y, the joints' axis), the plate, the payload.
    fn scene(&self, s: &[f64], out: &mut Scene) {
        let (t, _) = self.arms(s);
        let (rotor, rotor_speed) = self.rotors(s);
        let (p, _) = self.pose(t);
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
            let arm_end = [elbow[0] + side * self.platform_radius, 0.0, elbow[1]];
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
            let platform_end = [p[0] + side * self.platform_radius, 0.0, p[1]];
            for half in [-0.5 * self.rod_spacing, 0.5 * self.rod_spacing] {
                let (from, to) = ([arm_end[0], half, arm_end[2]], [platform_end[0], half, platform_end[2]]);
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
        out.parts.extend(self.on_platform(p));
        out.labels.push(Label {
            pos: [p[0], 0.0, p[1] - 0.17],
            owner_key: self.axes[1].key,
            value: p[1],
            unit_key: "unit-m",
            decimals: 3,
        });
    }

    /// The plate's centre, in the plane y = 0.
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        Some([tool_pose[0], 0.0, tool_pose[1]])
    }

    /// The plate and the payload under it, where `scene` draws them. The forearms are left out.
    fn collision_bodies(&self, s: &[f64]) -> Vec<sim_core::Aabb> {
        let (t, _) = self.arms(s);
        sim_core::scene::scene_boxes(&self.on_platform(self.pose(t).0), &["platform", "payload"])
    }
}
