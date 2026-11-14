//! `ScissorMachine`: a scissor lift from a machine file's `[[machine.scissor]]` node. n stages of crossed arm pairs,
//! each arm L from pin to pin, stand on the base; a screw, turned by the motor through its gearbox, pulls the sliding
//! foot of the lowest stage towards the fixed one and so raises the platform. The tool is the platform's height z.
//!
//! With u the span between the feet, a stage stands s = sqrt(L^2 - u^2) high and the platform h = n s. The gearbox's
//! output turns the screw by theta, which moves the foot by c theta, c = lead / (2 pi): u = u_0 - c theta, u_0 the span
//! at the lowest height. So
//!
//! ```text
//! h = n s,   dh/dtheta = n c u / s,   d2h/dtheta2 = -n c^2 L^2 / s^3
//! ```
//!
//! and by virtual work the screw pulls the foot with F = W dh/du = W n u / s = W n cot(alpha) for a weight W on the
//! platform, alpha the arms' angle to the base: the force ratio grows without bound as the lift folds flat.
//!
//! Dynamics in theta, as the linkage's: M(theta) theta'' + m h' h'' theta'^2 = tau_gear + Q_g, M = J_screw + m h'^2,
//! Q_g = -g m_g h'. m counts the platform, the payload and the arms' vertical motion, arms (4 n^2 - 1) / (12 n^2) (the
//! centres of stage k's arms at (k - 1/2) h / n); m_g the platform, the payload and half the arms.
//! shortcut: the arms' turning and the sideways motion of their centres and of the sliding feet are left out of the
//! kinetic energy (a few per cent of the arms' share at mid height); add them to M if light platforms on heavy arms
//! matter.
//!
//! The joint is a `RotaryJoint` on the motor's own config (motor, gearbox, brake), without joint friction or stops: the
//! screw's efficiency counts with the gearbox's (effects.gearbox_efficiency); elastic or rigid with
//! effects.joint_elasticity, as the linkage.

use crate::machine::intern;
use crate::netlist::{AxisNode, Kind, MotorNode, NODES, Netlist, Node, Transmission, number};
use sim_components::catalog::motor_config;
use sim_components::joint::RotaryJoint;
use sim_components::mech::{HardStop, ShaftFriction, ThermalFeedback};
use sim_core::{
    AxisInfo, Config, ConfigError, ENERGY_PARTS, LOSS_PREFIX, Label, Machine, MotorInfo, Part, Scene, Shape,
    SignalInfo, TORQUE_PARTS, ThermalPart,
};
use toml::Value;

const G: f64 = 9.80665;

/// The scissor node, read and checked: its numbers by name (an optional one left out is 0, the screw's efficiency 1)
/// and its gearbox's id.
pub struct ScissorNode {
    pub id: String,
    pub gearbox: String,
    values: Vec<(&'static str, f64)>,
}

impl ScissorNode {
    pub fn value(&self, name: &str) -> f64 {
        self.values.iter().find(|(n, _)| *n == name).map_or(0.0, |(_, v)| *v)
    }
}

/// The netlist of a file with a scissor lift: one axis (its height), one motor, the gearbox and the scissor node; no
/// other node and no belt effect.
pub(crate) fn read(
    cfg: &Config,
    axes: &[Node],
    motors: &[Node],
    gearboxes: &[Node],
    scissors: &[Node],
    others: &[&[Node]],
) -> Result<Netlist, ConfigError> {
    let node = match scissors {
        [node] => node,
        _ => return Err(ConfigError::Invalid(format!("{} (one scissor per file)", scissors[1].path))),
    };
    if let Some(other) = others.iter().find_map(|nodes| nodes.first()) {
        return Err(ConfigError::Invalid(format!(
            "{} (a file with a scissor lift has an axis, a motor, a gearbox and the scissor only)",
            other.path
        )));
    }
    let effects = cfg.get("effects").ok().and_then(Value::as_table);
    for (effect, on) in effects.into_iter().flatten() {
        if effect.starts_with("belt_") && on.as_bool() == Some(true) {
            return Err(ConfigError::Invalid(format!("effects.{effect} (a scissor lift has no belt: switch it off)")));
        }
    }
    let id_of = |n: &Node| n.table["id"].as_str().unwrap_or_default().to_string();
    let one = |nodes: &[Node], field: &str, list: &str| -> Result<usize, ConfigError> {
        let id = node.text(field)?.unwrap_or_default();
        match nodes.iter().position(|n| n.table["id"].as_str() == Some(id)) {
            Some(i) if nodes.len() == 1 => Ok(i),
            Some(_) => Err(ConfigError::Invalid(format!("{} (one machine.{list} node)", node.at(field)))),
            None => Err(ConfigError::Invalid(format!("{} = \"{id}\"", node.at(field)))),
        }
    };

    let fields = NODES.iter().find(|(list, ..)| *list == "scissor").map_or(&[][..], |(_, _, f)| *f);
    let (mut refs, mut values) = (Vec::new(), Vec::new());
    for f in fields.iter().filter(|f| f.kind == Kind::Number) {
        let v = node.get(f.name).map(|v| number(cfg, v, &node.at(f.name), &mut refs)).transpose()?;
        let v = v.unwrap_or(if f.name == "screw_efficiency" { 1.0 } else { 0.0 });
        let positive = !["payload_kg", "screw_inertia_kgm2", "arms_kg"].contains(&f.name);
        if if positive { !(v > 0.0) } else { !(v >= 0.0) } {
            return Err(ConfigError::Invalid(node.at(f.name)));
        }
        values.push((f.name, v));
    }
    let value = |name: &str| values.iter().find(|(n, _)| *n == name).map_or(0.0, |(_, v)| *v);
    let stages = value("stages");
    for (field, bad, why) in [
        ("stages", stages.fract() != 0.0, "a whole number"),
        ("screw_efficiency", value("screw_efficiency") > 1.0, "at most 1"),
        ("max_height_m", value("max_height_m") <= value("min_height_m"), "above min_height_m"),
        ("max_height_m", value("max_height_m") >= stages * value("arm_m"), "below stages times arm_m"),
    ] {
        if bad {
            return Err(ConfigError::Invalid(format!("{} ({why})", node.at(field))));
        }
    }

    let axis = &axes[one(axes, "axis", "axis")?];
    for field in ["stroke_m", "stroke_rad", "incline_deg", "cable_chain"] {
        if axis.get(field).is_some() {
            return Err(ConfigError::Invalid(format!(
                "{} (a scissor lift's axis takes its range from min_height_m and max_height_m)",
                axis.at(field)
            )));
        }
    }
    let axis_id = id_of(axis);
    let axis_key = axis.text("key")?.map_or_else(|| format!("axis-{axis_id}"), str::to_string);
    let gearbox = &gearboxes[one(gearboxes, "gearbox", "gearbox")?];
    let motor_id = gearbox.text("motor")?.unwrap_or_default();
    let motor = match motors.iter().position(|n| n.table["id"].as_str() == Some(motor_id)) {
        Some(m) if motors.len() == 1 => &motors[m],
        _ => return Err(ConfigError::Invalid(format!("{} = \"{motor_id}\" (the one motor)", gearbox.at("motor")))),
    };
    let key = motor.text("key")?.map_or_else(|| format!("motor-{motor_id}"), str::to_string);
    Ok(Netlist {
        axes: vec![AxisNode {
            id: axis_id,
            key: axis_key,
            stroke: value("max_height_m") - value("min_height_m"),
            incline_deg: 90.0,
            cable_chain: false,
            rotary: false,
        }],
        // What drives the platform is the scissor, which the netlist's transmissions do not describe.
        motors: vec![MotorNode {
            id: motor_id.to_string(),
            key,
            radius: 1.0,
            inertia: 0.0,
            pulley: String::new(),
            at: None,
            clockwise: false,
            transmission: Transmission::Belt,
            axis: None,
            worm: None,
        }],
        sections: Vec::new(),
        bodies: Vec::new(),
        guides: Vec::new(),
        idlers: Vec::new(),
        frame: None,
        spans: Vec::new(),
        lean: None,
        refs,
        values: Vec::new(),
        linkage: None,
        scissor: Some(ScissorNode { id: id_of(node), gearbox: id_of(gearbox), values }),
        gantry: None,
        tcp: None,
    })
}

/// Effects that take energy out or put it in: the standard suite runs with them off.
pub const LOSSY_EFFECTS: [&str; 4] = ["motor_friction", "gearbox_efficiency", "torque_ripple", "holding_brake"];

// Elastic state: [theta, omega, rotor angle, rotor speed, brake]; rigid: [theta, omega, brake].
const N_ELASTIC: usize = 5;
const N_RIGID: usize = 3;

/// The stages' geometry: their number n, the arms' length L, the screw's lever c = lead / (2 pi) (m of the foot per
/// rad of the screw) and the feet's span u_0 at the angle 0, the lowest height.
#[derive(Debug, Clone, Copy)]
pub struct Stages {
    pub stages: f64,
    pub arm: f64,
    pub lever: f64,
    pub span_low: f64,
}

impl Stages {
    /// The platform's height and its first and second derivative by the screw's angle theta.
    pub fn height(&self, theta: f64) -> [f64; 3] {
        let (n, c, l) = (self.stages, self.lever, self.arm);
        let u = self.span_low - c * theta;
        let s = (l * l - u * u).max(0.0).sqrt();
        [n * s, n * c * u / s, -n * c * c * l * l / s.powi(3)]
    }

    /// The screw's angle for a height; NaN out of reach.
    pub fn angle(&self, h: f64) -> f64 {
        let s = h / self.stages;
        let u = (self.arm * self.arm - s * s).sqrt();
        if h > 0.0 && u.is_finite() { (self.span_low - u) / self.lever } else { f64::NAN }
    }

    /// The force the screw pulls the foot with to hold the weight `w` (N) at the height h: W n u / s = W n cot(alpha).
    pub fn actuator_force(&self, w: f64, h: f64) -> f64 {
        let s = h / self.stages;
        w * self.stages * (self.arm * self.arm - s * s).sqrt() / s
    }
}

pub struct ScissorMachine {
    geometry: Stages,
    min_height: f64,
    max_height: f64,
    /// The screw's inertia at the gearbox output; the moving mass m (without the payload) and the weight's mass m_g.
    screw_inertia: f64,
    m_moving: f64,
    m_weight: f64,
    m_payload: f64,
    payload_attached: bool,
    brake_engaged: bool,
    /// The screw's efficiency (1 with effects.gearbox_efficiency off), which the gearbox's carries.
    screw_efficiency: f64,
    joint: RotaryJoint,
    thermal: Option<ThermalFeedback>,
    elastic: bool,
    g: f64,
    axes: [AxisInfo; 1],
    motors: [MotorInfo; 1],
    gearbox_id: &'static str,
    id: &'static str,
    rest_slack: f64,
}

impl ScissorMachine {
    pub fn from_node(cfg: &Config, net: &Netlist, node: &ScissorNode) -> Result<Self, ConfigError> {
        let v = |name: &str| node.value(name);
        let (stages, arm, lever) = (v("stages"), v("arm_m"), v("lead_m") / std::f64::consts::TAU);
        let (min_height, max_height) = (v("min_height_m"), v("max_height_m"));
        let span_low = (arm * arm - (min_height / stages).powi(2)).sqrt();
        let own = motor_config(cfg, &net.motors[0].id)?;
        let theta_max = (span_low - (arm * arm - (max_height / stages).powi(2)).sqrt()) / lever;
        let free = ShaftFriction::NONE;
        let mut joint = RotaryJoint::with(&own, free, HardStop { stiffness: 0.0, damping: 0.0 }, (0.0, theta_max))?;
        let counts = cfg.bool("effects.gearbox_efficiency")?;
        let screw_efficiency = if counts { v("screw_efficiency") } else { 1.0 };
        joint.gearbox.efficiency *= screw_efficiency;
        let axis = &net.axes[0];
        let arms = v("arms_kg");
        let mut machine = Self {
            geometry: Stages { stages, arm, lever, span_low },
            min_height,
            max_height,
            screw_inertia: v("screw_inertia_kgm2"),
            m_moving: v("platform_kg") + arms * (4.0 * stages * stages - 1.0) / (12.0 * stages * stages),
            m_weight: v("platform_kg") + 0.5 * arms,
            m_payload: if cfg.bool("effects.payload")? { v("payload_kg") } else { 0.0 },
            payload_attached: true,
            brake_engaged: false,
            screw_efficiency,
            joint,
            thermal: ThermalFeedback::from_config(cfg)?,
            elastic: cfg.bool("effects.joint_elasticity")?,
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            axes: [AxisInfo {
                id: intern(&axis.id),
                key: intern(&axis.key),
                unit: "m",
                min: min_height,
                max: max_height,
                max_vel: cfg.positive(&format!("axis.{}.max_velocity_m_s", axis.id))?,
                max_acc: cfg.positive(&format!("axis.{}.max_acceleration_m_s2", axis.id))?,
                max_jerk: cfg.positive(&format!("axis.{}.max_jerk_m_s3", axis.id))?,
            }],
            motors: [MotorInfo { id: intern(&net.motors[0].id), key: intern(&net.motors[0].key) }],
            // The warmth's id is that of the gearbox's loss.
            gearbox_id: intern(&format!("gearbox_{}", net.motors[0].id)),
            id: intern(&node.id),
            rest_slack: 0.0,
        };
        machine.rest_slack = machine.rest_sag();
        Ok(machine)
    }

    fn height(&self, theta: f64) -> [f64; 3] {
        self.geometry.height(theta)
    }
    fn angle(&self, h: f64) -> f64 {
        self.geometry.angle(h)
    }

    fn m_nt(&self) -> f64 {
        self.m_moving + if self.payload_attached { self.m_payload } else { 0.0 }
    }
    fn m_g(&self) -> f64 {
        self.m_weight + if self.payload_attached { self.m_payload } else { 0.0 }
    }

    /// M, the velocity term m h' h'' w^2 and Q_g at theta and its speed w.
    fn load(&self, theta: f64, w: f64) -> (f64, f64, f64) {
        let [_, d1, d2] = self.height(theta);
        (self.screw_inertia + self.m_nt() * d1 * d1, self.m_nt() * d1 * d2 * w * w, -self.g * self.m_g() * d1)
    }

    fn rotor(&self, s: &[f64]) -> (f64, f64) {
        if self.elastic { (s[2], s[3]) } else { (self.joint.ratio() * s[0], self.joint.ratio() * s[1]) }
    }
    fn brake(&self, s: &[f64]) -> f64 {
        s[if self.elastic { 4 } else { 2 }]
    }

    /// Rigid model: theta'', the torque into the gearbox (after the rotor's own acceleration) and out of it.
    fn rigid_solve(&self, s: &[f64], torque: f64) -> (f64, f64, f64) {
        let (theta, w) = (s[0], s[1]);
        let (rotor, rotor_speed) = self.rotor(s);
        let (mass, velocity, gravity) = self.load(theta, w);
        let j = &self.joint;
        let drive = j.rigid_drive(torque, rotor, rotor_speed, self.brake(s), self.brake_engaged).0;
        let rest = gravity - velocity + j.output_torque(theta, w);
        let first = (drive + rest) / (mass + j.reflected_inertia());
        let loss = j.gearbox.loss_factor(drive - j.reflected_inertia() * first, w);
        let acc = (loss * drive + rest) / (mass + loss * j.reflected_inertia());
        let into = drive - j.reflected_inertia() * acc;
        (acc, into, loss * into)
    }

    fn acceleration(&self, s: &[f64], torque: f64) -> f64 {
        let mut d = [0.0; N_ELASTIC];
        self.derivatives(s, &[torque], &mut d);
        d[1]
    }

    fn gear(&self, s: &[f64], torque: f64) -> f64 {
        if self.elastic {
            self.joint.gearbox.torque(s[2], s[3], s[0], s[1]).output
        } else {
            self.rigid_solve(s, torque).2
        }
    }

    /// The shaft's friction with the brake, the gearbox's loss, and what the ripple gives off (W); and the gearbox's
    /// loss as a torque at the rotor.
    fn losses(&self, s: &[f64], torque: f64) -> ([f64; 3], f64) {
        let (theta, w) = (s[0], s[1]);
        let (rotor, rotor_speed) = self.rotor(s);
        let j = &self.joint;
        let [_, _, shaft] = j.losses(theta, w, rotor_speed, self.brake(s), self.brake_engaged);
        let (gear, at_rotor) = if self.elastic {
            let g = j.gearbox.torque(s[2], s[3], theta, w);
            (j.gearbox.loss_power(s[2], s[3], theta, w), g.rotor - g.output / j.ratio())
        } else {
            let (_, into, out) = self.rigid_solve(s, torque);
            ((into - out) * w, (into - out) / j.ratio())
        };
        ([shaft, gear, (torque - j.ripple.apply(torque, rotor)) * rotor_speed], at_rotor)
    }

    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        let (theta, w) = (s[0], s[1]);
        let (mass, ..) = self.load(theta, w);
        let mut kinetic = 0.5 * mass * w * w;
        let potential = self.g * self.m_g() * self.height(theta)[0];
        let mut elastic = 0.0;
        if self.elastic {
            kinetic += 0.5 * self.joint.rotor_inertia * s[3] * s[3];
            elastic += self.joint.gearbox.energy(s[2], s[0]);
        } else {
            kinetic += 0.5 * self.joint.reflected_inertia() * w * w;
        }
        [kinetic, potential, elastic]
    }

    /// How far the motor's pose can be from the platform's at rest: the gearbox twisted by what it holds over its
    /// stiffness and half its backlash, through dh/dtheta, over 21 heights, payload on; plus a tenth. At least 0.1 mm;
    /// that alone in the rigid model.
    fn rest_sag(&self) -> f64 {
        if !self.elastic {
            return 1e-4;
        }
        let gear = &self.joint.gearbox;
        let most = (0..=20)
            .map(|k| self.min_height + (self.max_height - self.min_height) * k as f64 / 20.0)
            .map(|h| {
                let [_, d1, _] = self.height(self.angle(h));
                let held = self.g * (self.m_weight + self.m_payload) * d1;
                d1 * (held / gear.stiffness + gear.half_backlash)
            })
            .fold(0.0, f64::max);
        (1.1 * most).max(1e-4)
    }
}

impl Machine for ScissorMachine {
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }
    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        motors[0] = self.joint.ratio() * self.angle(tool[0]);
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        tool[0] = self.height(motors[0] / self.joint.ratio())[0];
    }
    fn state_len(&self) -> usize {
        if self.elastic { N_ELASTIC } else { N_RIGID }
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let theta = self.angle(tool[0]);
        let (_, _, gravity) = self.load(theta, 0.0);
        let j = &self.joint;
        let out = -gravity - j.output_torque(theta, 0.0);
        let rotor = j.ratio() * (theta + if self.elastic { j.gearbox.static_twist(out) } else { 0.0 });
        state.clear();
        state.extend([theta, 0.0]);
        if self.elastic {
            state.extend([rotor, 0.0]);
        }
        state.push(0.0);
        torque[0] = j.ripple.invert(out / j.ratio(), rotor);
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let j = &self.joint;
        if !self.elastic {
            let (acc, ..) = self.rigid_solve(s, torque[0]);
            let rotor_speed = self.rotor(s).1;
            out[..3].copy_from_slice(&[s[1], acc, j.brake.torque(self.brake_engaged, rotor_speed, s[2]).1]);
            return;
        }
        let (theta, w) = (s[0], s[1]);
        let (mass, velocity, gravity) = self.load(theta, w);
        let (gear, rotor_acc, brake_rate) = j.rotor(torque[0], s[2], s[3], theta, w, s[4], self.brake_engaged);
        let acc = (gear.output + gravity - velocity + j.output_torque(theta, w)) / mass;
        out[..5].copy_from_slice(&[w, acc, s[3], rotor_acc, brake_rate]);
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let [h, d1, _] = self.height(s[0]);
        (pos[0], vel[0]) = (h, d1 * s[1]);
    }
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        s[1] = vel[0] / self.height(s[0])[1];
        if self.elastic {
            s[3] = self.joint.ratio() * s[1];
        }
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        (pos[0], vel[0]) = self.rotor(s);
    }

    /// Rigid, loss-free, at rest: theta'' = z'' / h', M theta'' - Q_g over the ratio, plus the rotor's J i theta''.
    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        let theta = self.angle(pos[0]);
        let (mass, _, gravity) = self.load(theta, 0.0);
        let acc = acc[0] / self.height(theta)[1];
        let j = &self.joint;
        out[0] = (mass * acc - gravity) / j.ratio() + j.rotor_inertia * j.ratio() * acc;
    }

    /// With the speed: theta' = z' / h', theta'' = (z'' - h'' theta'^2) / h', and J_screw theta'' + m h' z'' - Q_g at
    /// the screw.
    fn feedforward_torque_moving(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut [f64]) {
        let theta = self.angle(pos[0]);
        let [_, d1, d2] = self.height(theta);
        let (_, _, gravity) = self.load(theta, 0.0);
        let w = vel[0] / d1;
        let theta_acc = (acc[0] - d2 * w * w) / d1;
        let j = &self.joint;
        out[0] = (self.screw_inertia * theta_acc + self.m_nt() * d1 * acc[0] - gravity) / j.ratio()
            + j.rotor_inertia * j.ratio() * theta_acc;
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
    fn set_part_temperatures(&mut self, rises: &[f64]) {
        if let (Some(law), Some(rise), false) = (&self.thermal, rises.first(), self.joint.gearbox.is_coupling()) {
            self.joint.gearbox.efficiency = law.gearbox_efficiency(*rise) * self.screw_efficiency;
        }
    }
    fn setpoint_slack(&self) -> f64 {
        self.rest_slack
    }
    fn unreachable(&self, tool: &[f64], slack: f64) -> Option<&'static str> {
        let inside = tool.len() == 1 && tool[0] >= self.min_height - slack && tool[0] <= self.max_height + slack;
        (!inside).then_some("error-target-out-of-range")
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let (motor, axis) = (&self.motors[0], &self.axes[0]);
        let mut out = vec![
            SignalInfo::new(format!("gearbox.{}.output_torque", motor.id), motor.key, "quantity-gearbox-torque", "N m"),
            SignalInfo::new(format!("scissor.{}.actuator_force", self.id), axis.key, "quantity-joint-force", "N"),
        ];
        out.extend(TORQUE_PARTS.map(|part| {
            SignalInfo::new(
                format!("motor.{}.torque_{part}", motor.id),
                motor.key,
                &format!("quantity-torque-{part}"),
                "N m",
            )
        }));
        let gear = if self.joint.gearbox.is_coupling() {
            "quantity-coupling-loss-power"
        } else {
            "quantity-gearbox-loss-power"
        };
        out.extend([
            SignalInfo::new(
                format!("{LOSS_PREFIX}shaft_{}", motor.id),
                motor.key,
                "quantity-shaft-friction-power",
                "W",
            ),
            SignalInfo::new(format!("{LOSS_PREFIX}gearbox_{}", motor.id), motor.key, gear, "W"),
            SignalInfo::new(format!("{LOSS_PREFIX}other"), "loss-other", "quantity-power-given-off", "W"),
        ]);
        out.extend(ENERGY_PARTS.map(|part| {
            SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")
        }));
        out
    }

    /// The gearbox's output torque, and the force on the sliding foot, the screw's torque over its lever c.
    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let gear = self.gear(s, torque[0]);
        out.extend([gear, gear / self.geometry.lever]);
    }
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (theta, w) = (s[0], s[1]);
        let j = &self.joint;
        let (mass, velocity, gravity) = self.load(theta, w);
        let acc = self.acceleration(s, torque[0]);
        let (powers, gear_at_rotor) = self.losses(s, torque[0]);
        let rotor_speed = self.rotor(s).1;
        let held = j.brake.torque(self.brake_engaged, rotor_speed, self.brake(s)).0;
        let parts = [
            (mass * acc + velocity + j.reflected_inertia() * acc) / j.ratio(),
            -gravity / j.ratio(),
            j.shaft.torque(rotor_speed) + held + gear_at_rotor,
        ];
        out.extend(parts);
        out.push(torque[0] - parts.iter().sum::<f64>());
        out.extend(powers);
        out.extend(self.energy_parts(s));
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(s, torque[0]).0);
    }

    fn thermal_parts(&self) -> Vec<ThermalPart> {
        if self.joint.gearbox.is_coupling() {
            return Vec::new();
        }
        vec![ThermalPart {
            id: self.gearbox_id,
            kind: "gearbox",
            owner_key: self.motors[0].key,
            scene_part: "gearbox",
            scene_index: 0,
        }]
    }

    /// The lift in the plane y = 0: the base, the motor and gearbox at the fixed foot, the screw to the sliding foot,
    /// the crossed arms of every stage, the platform and the payload on it.
    fn scene(&self, s: &[f64], out: &mut Scene) {
        let h = self.height(s[0])[0];
        let (rotor, rotor_speed) = self.rotor(s);
        let Stages { stages, arm, .. } = self.geometry;
        let (n, u) = (stages as usize, (arm * arm - (h / stages).powi(2)).max(0.0).sqrt());
        let grey = [0.62, 0.64, 0.68];
        out.parts.clear();
        out.labels.clear();
        let width = arm.max(0.2);
        out.parts.push(Part::new("base", Shape::Box { size: [width + 0.2, 0.4, 0.04] }, [0.0, 0.0, -0.02], grey));
        let foot = [-0.5 * u, 0.0, 0.0];
        out.parts.push(Part::new(
            sim_core::scene::MOTOR_PART,
            Shape::Box { size: [0.12, 0.09, 0.09] },
            [-0.5 * width - 0.2, 0.0, 0.06],
            [0.9, 0.45, 0.1],
        ));
        out.parts.push(Part::new(
            "gearbox",
            Shape::Box { size: [0.06, 0.09, 0.09] },
            [-0.5 * width - 0.11, 0.0, 0.06],
            [0.25, 0.25, 0.28],
        ));
        out.parts.push(
            Part::new(
                "rotor-bar",
                Shape::Box { size: [0.004, 0.07, 0.012] },
                [-0.5 * width - 0.27, 0.0, 0.06],
                [0.1, 0.1, 0.1],
            )
            .turned(rotor),
        );
        out.parts.push(Part::new(
            "screw",
            Shape::Segment { to: [0.5 * u, 0.0, 0.06], width: 0.016 },
            [-0.5 * width - 0.08, 0.0, 0.06],
            [0.5, 0.5, 0.55],
        ));
        let stage = h / stages;
        for k in 0..n {
            let (low, high) = (k as f64 * stage, (k + 1) as f64 * stage);
            for (from, to) in [(foot[0], 0.5 * u), (0.5 * u, foot[0])] {
                out.parts.push(Part::new(
                    "arm",
                    Shape::Segment { to: [to, 0.0, high], width: 0.03 },
                    [from, 0.0, low],
                    [0.2, 0.22, 0.26],
                ));
            }
        }
        out.parts.push(Part::new(
            "platform",
            Shape::Box { size: [width, 0.4, 0.03] },
            [0.0, 0.0, h + 0.015],
            [0.55, 0.55, 0.6],
        ));
        if self.payload_attached && self.m_payload > 0.0 {
            out.parts.push(Part::new(
                "payload",
                Shape::Box { size: [0.2, 0.2, 0.15] },
                [0.0, 0.0, h + 0.105],
                [0.2, 0.7, 0.3],
            ));
        }
        out.labels.push(Label {
            pos: [-0.5 * width - 0.2, 0.0, 0.16],
            owner_key: self.motors[0].key,
            value: rotor_speed * 60.0 / std::f64::consts::TAU,
            unit_key: "unit-rpm",
            decimals: 0,
        });
        out.labels.push(Label {
            pos: [0.5 * width + 0.1, 0.0, h],
            owner_key: self.axes[0].key,
            value: h,
            unit_key: "unit-m",
            decimals: 3,
        });
    }

    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        Some([0.0, 0.0, tool_pose[0] + 0.03])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two stages of 1 m arms, lead 10 mm: at h = 0.6 m a stage stands 0.3 m, the feet u = sqrt(1 - 0.09) = 0.953939 m
    /// apart; a weight W = 1000 N on the platform takes F = W n u / s = 1000 * 2 * 0.953939 / 0.3 = 6359.59 N at the
    /// foot (cot alpha = 3.17980). At h = 1.6 m (s = 0.8, u = 0.6): F = 1000 * 2 * 0.6 / 0.8 = 1500 N, cot alpha = 0.75.
    /// The ratio of the two, 4.23973, is the price of lifting from low down. By virtual work, F = W dh/du: the
    /// derivative of `height` by theta over the lever c gives the same.
    #[test]
    fn the_scissor_force_ratio_is_n_cot_alpha_at_two_heights() {
        let lift = Stages { stages: 2.0, arm: 1.0, lever: 0.01 / std::f64::consts::TAU, span_low: 0.99f64.sqrt() };
        for (h, by_hand, cot) in [(0.6, 6359.59, 3.179_797), (1.6, 1500.0, 0.75)] {
            let force = lift.actuator_force(1000.0, h);
            assert!((force - by_hand).abs() < 0.01, "h = {h}: {force} N against {by_hand}");
            assert!((force - 1000.0 * 2.0 * cot).abs() < 0.001, "h = {h}: n cot alpha");
            let theta = lift.angle(h);
            let [height, d1, _] = lift.height(theta);
            assert!((height - h).abs() < 1e-12, "{height}");
            assert!((1000.0 * d1 / lift.lever - force).abs() < 1e-6, "virtual work at h = {h}");
        }
        let ratio = lift.actuator_force(1000.0, 0.6) / lift.actuator_force(1000.0, 1.6);
        assert!((ratio - 4.239_73).abs() < 1e-5, "{ratio}");
        // d2h/dtheta2 against central differences of dh/dtheta.
        let theta = lift.angle(1.0);
        let step = 1e-3;
        let numeric = (lift.height(theta + step)[1] - lift.height(theta - step)[1]) / (2.0 * step);
        assert!((lift.height(theta)[2] - numeric).abs() < 1e-9 * numeric.abs().max(1e-9), "{numeric}");
    }
}
