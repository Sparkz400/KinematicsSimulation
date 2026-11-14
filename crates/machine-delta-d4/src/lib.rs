//! Delta picker D4: the three-arm delta of `machine-delta-d3` with a fourth axis that turns the tool about the
//! vertical, as in the four-axis pickers (ABB IRB 360 with its fourth axis, Autonox's four-axis deltas): a fourth motor
//! with its gearbox on the base, in its centre, drives a telescopic shaft through a cardan (universal) joint, and the
//! shaft drives the tool's shaft in a bearing of the platform through a second cardan joint. The tool is the
//! platform's centre and the tool's turn, x, y, z, c.
//!
//! The arms are the D3's (`machine_delta_d3::Delta`, its kinematics, dynamics, rods, bending arms, modes, collision
//! rods), built from this config with the shaft's share of mass at the platform added to it. What the fourth axis adds:
//!
//! ```text
//! shaft     d = p + (0, 0, h) - (0, 0, z_u)      upper cardan on the base's axis at z_u, lower one h above the platform
//!           L = |d|,  beta = angle(d, -z),  psi = atan2(d_y, d_x)    (length, cardan angle at both joints, bending plane)
//! cardan    tan chi = tan phi / cos beta     one joint, phi and chi from the bending plane (the input yoke's pin in it
//!           chi' / phi' = cos beta / (1 - sin^2 beta cos^2 phi)       at phi = 0): the shaft turns unevenly
//! pair      tan phi_out = cos beta tan chi = tan phi                 the second joint in phase undoes the first
//! ```
//!
//! The pair cancels exactly here: the motor's shaft and the tool's shaft are both vertical (the platform never tilts),
//! so the two joints are bent by the same beta in one plane, the Z arrangement; with the intermediate shaft's yokes
//! in phase the tool turns as the gearbox's output, c = theta_out, at any pose. Only the intermediate shaft turns
//! unevenly, between cos beta and 1 / cos beta of the tool's speed (the signal shaft.speed).
//!
//! Dynamics. The tool's turn is a rotary joint (`RotaryJoint`: motor, gearbox, brake, the tool bearing's friction), its
//! load the tool and gripper, the payload (with effects.payload) and the shaft's spin, J_c = J_tool + J_payload + J_s,
//! with no stops (the turn is endless):
//!
//! ```text
//! J_c c'' = tau_gear + tau_bearing
//! ```
//!
//! Nothing turns the arms' parallelograms, so the turn and the arms are coupled only through the shaft: its weight at
//! the platform (a share of its mass, effects.shaft_mass, added to the D3's platform mass) and the telescope's sliding
//! friction along the shaft against its lengthening, F = -f(L') u (effects.telescope_friction, a `PlatformDrag` on
//! the D3, J^T F at the arms). The torque the shaft passes and the cardans' bending moments on the platform do no work
//! on the arms: the parallelograms hold the platform level.
//!
//! With effects.joint_elasticity the tool's gearbox twists, and with effects.shaft_torsion the shaft twists in series
//! with it: one spring 1 / (1/k_gear + 1/k_shaft) between the gearbox and the tool, with the shaft's damping.

use machine_delta_d3::{Delta, PlatformDrag};
use sim_components::catalog::motor_config;
use sim_components::joint::RotaryJoint;
use sim_components::mech::{HardStop, ShaftFriction, ThermalFeedback};
use sim_core::{
    AxisInfo, Config, ConfigError, LOSS_PREFIX, Label, Machine, MachineEntry, Mode, MotorInfo, Part, Scene, Shape,
    SignalInfo, TORQUE_PARTS, ThermalPart,
};

pub static ENTRY: MachineEntry = MachineEntry {
    id: "delta-d4",
    key: "machine-delta-d4",
    default_config: include_str!("../default.toml"),
    i18n: &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))],
    build,
    suite_configs,
};

/// Effects that take energy out or put it in; the standard suite runs with them off: the D3's and the telescope's
/// sliding friction.
pub const LOSSY_EFFECTS: [&str; 6] =
    ["joint_friction", "motor_friction", "gearbox_efficiency", "torque_ripple", "holding_brake", "telescope_friction"];

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    Ok(Box::new(Delta4::new(cfg)?))
}

/// The default config with every loss switched off.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    for damping in [
        "gearbox.damping_nm_s_rad",
        "joint.stop_damping_nm_s_rad",
        "upper_arm.bending_damping_nm_s_rad",
        "upper_arm.lateral_damping_n_s_m",
        "gearbox.4.damping_nm_s_rad",
        "shaft.torsional_damping_nm_s_rad",
    ] {
        cfg.set_text(&format!("{damping} = 0.0")).unwrap();
    }
    cfg
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

/// As the D3's (see `machine_delta_d3`): loss-free rigid and elastic models, with and without the payload, the working
/// range 175 mm lower, where the delta held by constant torques is stable in its vertical mode. The shaft's torsion
/// is on in the elastic ones.
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

fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// `a` moved by whole turns to lie within half a turn of `near`.
fn unwrap_near(a: f64, near: f64) -> f64 {
    a + std::f64::consts::TAU * ((near - a) / std::f64::consts::TAU).round()
}

/// One cardan joint bent by `beta`: the output's angle for the input's `phi`, both from the bending plane (the plane of
/// the two shafts), the input yoke's pin lying in that plane at phi = 0: tan chi = tan phi / cos beta.
pub fn cardan(beta: f64, phi: f64) -> f64 {
    unwrap_near(phi.sin().atan2(beta.cos() * phi.cos()), phi)
}

/// The second joint of a pair whose intermediate shaft has its yokes in phase, bent by the same `beta` in the same
/// plane: the output's angle for the intermediate shaft's `chi`, tan phi_out = cos beta tan chi (the first joint
/// backwards).
pub fn cardan_back(beta: f64, chi: f64) -> f64 {
    unwrap_near((beta.cos() * chi.sin()).atan2(chi.cos()), chi)
}

/// The speed of a cardan joint's output over its input's at the input's angle `phi` from the bending plane:
/// cos beta / (1 - sin^2 beta cos^2 phi), between cos beta (phi = 90 degrees) and 1 / cos beta (phi = 0).
pub fn cardan_speed_ratio(beta: f64, phi: f64) -> f64 {
    beta.cos() / (1.0 - (beta.sin() * phi.cos()).powi(2))
}

/// The telescopic shaft between its two cardan joints.
#[derive(Debug, Clone)]
pub struct Shaft {
    /// The upper joint's height on the base's axis, and the lower joint's above the platform's centre, m.
    pub upper_z: f64,
    pub lower_offset: f64,
    /// Shortest and longest the telescope can be, m; the largest bend of a joint, rad.
    pub min_length: f64,
    pub max_length: f64,
    pub max_angle: f64,
}

impl Shaft {
    /// The upper and the lower joint with the platform's centre at p.
    pub fn joints(&self, p: V3) -> (V3, V3) {
        ([0.0, 0.0, self.upper_z], [p[0], p[1], p[2] + self.lower_offset])
    }

    /// Length, the cardan angle (both joints) and the bending plane's azimuth with the platform's centre at p.
    pub fn geometry(&self, p: V3) -> (f64, f64, f64) {
        let d = [p[0], p[1], p[2] + self.lower_offset - self.upper_z];
        let across = d[0].hypot(d[1]);
        (dot(d, d).sqrt(), across.atan2(-d[2]), d[1].atan2(d[0]))
    }
}

pub struct Delta4 {
    /// The three arms: the D3 with the shaft's share at its platform and the telescope's friction on it.
    arms: Delta,
    /// How many of the state's entries and of the base signals are the arms'.
    n3: usize,
    arm_base_signals: usize,
    /// The tool's turn: motor, gearbox (twisted together with the shaft), brake, the tool bearing's friction.
    tool: RotaryJoint,
    elastic: bool,
    /// Tool and gripper with the shaft's spin (kg m^2), and the payload's about the tool's axis.
    j_tool: f64,
    j_payload: f64,
    payload_attached: bool,
    brake_engaged: bool,
    shaft: Shaft,
    /// The telescope's friction as the D3 has it, for its loss power; `None` with effects.telescope_friction off.
    drag: Option<PlatformDrag>,
    thermal: Option<ThermalFeedback>,
    axes: [AxisInfo; 4],
    motors: [MotorInfo; 4],
}

/// The tool's losses of one moment: its bearing (and stops: none), its motor's friction with the brake, its gearbox,
/// the telescope's sliding, the ripple's; and the gearbox's loss as a torque at the rotor.
struct ToolLosses {
    powers: [f64; 5],
    gear_at_rotor: f64,
}

impl Delta4 {
    pub fn new(cfg: &Config) -> Result<Self, ConfigError> {
        let shaft = Shaft {
            upper_z: cfg.f64("shaft.upper_joint_z_m")?,
            lower_offset: cfg.non_negative("shaft.lower_joint_offset_m")?,
            min_length: cfg.positive("shaft.min_length_m")?,
            max_length: cfg.positive("shaft.max_length_m")?,
            max_angle: cfg.positive("shaft.max_cardan_angle_deg")?.to_radians(),
        };
        if shaft.max_length <= shaft.min_length {
            return Err(ConfigError::Invalid("shaft.max_length_m".into()));
        }
        let share = cfg.non_negative("shaft.platform_share")?;
        if share > 1.0 {
            return Err(ConfigError::Invalid("shaft.platform_share".into()));
        }
        let with_mass = cfg.bool("effects.shaft_mass")?;
        // The arms: the D3 with the shaft's share of mass on its platform (the share of the inner shaft and the lower
        // joint, which the platform carries; the outer tube hangs from the upper joint).
        let mut inner = cfg.clone();
        if with_mass {
            let platform = cfg.positive("mass.platform_kg")? + share * cfg.non_negative("shaft.mass_kg")?;
            inner.set_text(&format!("mass.platform_kg = {platform:?}"))?;
        }
        // The arms are the D3's, which reads the shared [motor] and [gearbox]: an arm's own table there would size its
        // drive and verdicts but not the arm it turns, so it is refused.
        // shortcut: the three arms share one motor and gearbox; tables per arm if an arm ever needs another.
        for id in ["1", "2", "3"] {
            for table in ["parts", "motor", "gearbox"] {
                if cfg.get(&format!("{table}.{id}")).is_ok_and(|v| v.is_table()) {
                    return Err(ConfigError::Invalid(format!("{table}.{id} (the three arms share [{table}])")));
                }
            }
        }
        let mut arms = Delta::new(&inner)?;
        let drag = if cfg.bool("effects.telescope_friction")? {
            let friction = ShaftFriction {
                coulomb: cfg.non_negative("shaft.slide_friction_n")?,
                viscous: cfg.non_negative("shaft.slide_friction_viscous_n_s_m")?,
                smoothing: cfg.positive("shaft.slide_friction_smoothing_m_s")?,
            };
            Some(PlatformDrag { pivot: [0.0, 0.0, shaft.upper_z], offset: [0.0, 0.0, shaft.lower_offset], friction })
        } else {
            None
        };
        arms.set_platform_drag(drag.clone());
        // The tool's joint: motor 4's own motor and gearbox, `[motor.4]` and `[gearbox.4]` over the shared tables
        // (`catalog::motor_config`), as its drive and its verdicts read them.
        let tcfg = motor_config(cfg, "4")?;
        let bearing = cfg.bool("effects.joint_friction")?;
        let friction = ShaftFriction {
            coulomb: if bearing { cfg.non_negative("tool.friction_coulomb_nm")? } else { 0.0 },
            viscous: if bearing { cfg.non_negative("tool.friction_viscous_nm_s_rad")? } else { 0.0 },
            smoothing: cfg.positive("tool.friction_smoothing_rad_s")?,
        };
        let (min, max) = (cfg.f64("axis.c.min_angle_deg")?.to_radians(), cfg.f64("axis.c.max_angle_deg")?.to_radians());
        if min >= max {
            return Err(ConfigError::Invalid("axis.c.max_angle_deg".into()));
        }
        // No stops: the turn is endless, the axis range is the controller's.
        let mut tool = RotaryJoint::with(&tcfg, friction, HardStop { stiffness: 0.0, damping: 0.0 }, (min, max))?;
        let (k_shaft, c_shaft) =
            (cfg.positive("shaft.torsional_stiffness_nm_rad")?, cfg.non_negative("shaft.torsional_damping_nm_s_rad")?);
        if cfg.bool("effects.shaft_torsion")? {
            // shortcut: the shaft's spin inertia sits at the tool and the gearbox's output has none, so gearbox and shaft
            // are one spring in series, damped by the shaft (the softer by far); a body between the two springs if the
            // gearbox's own mode (above 1 kHz against the shaft's spin) ever matters.
            tool.gearbox.stiffness = 1.0 / (1.0 / tool.gearbox.stiffness + 1.0 / k_shaft);
            tool.gearbox.damping = c_shaft;
        }
        let j_shaft = if with_mass { cfg.non_negative("shaft.spin_inertia_kgm2")? } else { 0.0 };
        let mut axes: Vec<AxisInfo> = arms.axes().to_vec();
        axes.push(AxisInfo {
            id: "c",
            key: "axis-c",
            unit: "rad",
            min,
            max,
            max_vel: cfg.positive("axis.c.max_velocity_rad_s")?,
            max_acc: cfg.positive("axis.c.max_acceleration_rad_s2")?,
            max_jerk: cfg.positive("axis.c.max_jerk_rad_s3")?,
        });
        let motors = [
            MotorInfo { id: "1", key: "motor-arm-1" },
            MotorInfo { id: "2", key: "motor-arm-2" },
            MotorInfo { id: "3", key: "motor-arm-3" },
            MotorInfo { id: "4", key: "motor-tool" },
        ];
        // The arms' share of the base signals, counted at rest in the middle of the range.
        let mid: Vec<f64> = arms.axes().iter().map(|a| 0.5 * (a.min + a.max)).collect();
        let (mut state, mut torque, mut signals) = (Vec::new(), [0.0; 3], Vec::new());
        arms.equilibrium(&[0.0, 0.0, mid[2]], &mut state, &mut torque);
        arms.base_signals(&state, &torque, &mut signals);
        Ok(Self {
            n3: arms.state_len(),
            arm_base_signals: signals.len(),
            arms,
            elastic: cfg.bool("effects.joint_elasticity")?,
            j_tool: cfg.positive("tool.inertia_kgm2")? + j_shaft,
            j_payload: if cfg.bool("effects.payload")? { cfg.non_negative("tool.payload_inertia_kgm2")? } else { 0.0 },
            payload_attached: true,
            brake_engaged: false,
            shaft,
            drag,
            thermal: ThermalFeedback::from_config(&tcfg)?,
            tool,
            axes: axes.try_into().map_err(|_| ConfigError::Invalid("axis".into()))?,
            motors,
        })
    }

    /// The arms, a D3.
    pub fn arms(&self) -> &Delta {
        &self.arms
    }

    pub fn shaft(&self) -> &Shaft {
        &self.shaft
    }

    pub fn tool_joint(&self) -> &RotaryJoint {
        &self.tool
    }

    /// What the tool's joint turns: tool, gripper, shaft spin and, while held, the payload.
    pub fn tool_inertia(&self) -> f64 {
        self.j_tool + if self.payload_attached { self.j_payload } else { 0.0 }
    }

    /// The tool's turn and its speed.
    fn turn(&self, s: &[f64]) -> (f64, f64) {
        (s[self.n3], s[self.n3 + 1])
    }

    /// The tool motor's rotor angle and speed.
    fn rotor(&self, s: &[f64]) -> (f64, f64) {
        if self.elastic {
            (s[self.n3 + 2], s[self.n3 + 3])
        } else {
            let (c, w) = self.turn(s);
            (self.tool.ratio() * c, self.tool.ratio() * w)
        }
    }

    fn brake(&self, s: &[f64]) -> f64 {
        s[self.n3 + if self.elastic { 4 } else { 2 }]
    }

    /// The platform's centre and speed.
    fn platform(&self, s: &[f64]) -> (V3, V3) {
        let (mut p, mut v) = ([0.0; 3], [0.0; 3]);
        self.arms.tool_state(&s[..self.n3], &mut p, &mut v);
        (p, v)
    }

    /// Rigid model: the turn's acceleration, and the torque into the gearbox (after the rotor's own acceleration)
    /// and out of it, as the D3's arms (`Delta::rigid_solve`).
    fn rigid_solve(&self, s: &[f64], torque: f64) -> (f64, f64, f64) {
        let ((c, w), (rotor, speed)) = (self.turn(s), self.rotor(s));
        let j = &self.tool;
        let drive = j.rigid_drive(torque, rotor, speed, self.brake(s), self.brake_engaged).0;
        let rest = j.output_torque(c, w);
        let solve = |loss: f64| (loss * drive + rest) / (self.tool_inertia() + loss * j.reflected_inertia());
        let loss = j.gearbox.loss_factor(drive - j.reflected_inertia() * solve(1.0), w);
        let acc = solve(loss);
        let into = drive - j.reflected_inertia() * acc;
        (acc, into, loss * into)
    }

    /// The tool's derivatives into `out` (its part of the state).
    fn tool_rates(&self, s: &[f64], torque: f64, out: &mut [f64]) {
        let (c, w) = self.turn(s);
        out[0] = w;
        if self.elastic {
            let o = self.n3;
            let (gear, rotor_acc, brake_rate) =
                self.tool.rotor(torque, s[o + 2], s[o + 3], c, w, s[o + 4], self.brake_engaged);
            out[1] = (gear.output + self.tool.output_torque(c, w)) / self.tool_inertia();
            out[2] = s[o + 3];
            out[3] = rotor_acc;
            out[4] = brake_rate;
        } else {
            out[1] = self.rigid_solve(s, torque).0;
            let (_, speed) = self.rotor(s);
            out[2] = self.tool.brake.torque(self.brake_engaged, speed, self.brake(s)).1;
        }
    }

    /// The torque the tool's gearbox passes to the tool.
    fn tool_gear_output(&self, s: &[f64], torque: f64) -> f64 {
        let ((c, w), (rotor, speed)) = (self.turn(s), self.rotor(s));
        if self.elastic { self.tool.gearbox.torque(rotor, speed, c, w).output } else { self.rigid_solve(s, torque).2 }
    }

    /// The tool's kinetic, potential (none) and elastic energy, the rotor's included.
    fn tool_energy(&self, s: &[f64]) -> [f64; 3] {
        let ((c, w), (rotor, speed)) = (self.turn(s), self.rotor(s));
        let mut kinetic = 0.5 * self.tool_inertia() * w * w;
        let mut elastic = self.tool.stop_energy(c);
        if self.elastic {
            kinetic += 0.5 * self.tool.rotor_inertia * speed * speed;
            elastic += self.tool.gearbox.energy(rotor, c);
        } else {
            kinetic += 0.5 * self.tool.reflected_inertia() * w * w;
        }
        [kinetic, 0.0, elastic]
    }

    /// The telescope's sliding power, W: its friction times the rate at which it lengthens.
    pub fn telescope_power(&self, s: &[f64]) -> f64 {
        let Some(drag) = &self.drag else { return 0.0 };
        let (p, v) = self.platform(s);
        -dot(drag.force(p, v), v)
    }

    fn tool_losses(&self, s: &[f64], torque: f64) -> ToolLosses {
        let ((c, w), (rotor, speed)) = (self.turn(s), self.rotor(s));
        let j = &self.tool;
        let [friction, stop, shaft] = j.losses(c, w, speed, self.brake(s), self.brake_engaged);
        let (gear, gear_at_rotor) = if self.elastic {
            let g = j.gearbox.torque(rotor, speed, c, w);
            (j.gearbox.loss_power(rotor, speed, c, w), g.rotor - g.output / j.ratio())
        } else {
            let (_, into, out) = self.rigid_solve(s, torque);
            ((into - out) * w, (into - out) / j.ratio())
        };
        let other = (torque - j.ripple.apply(torque, rotor)) * speed;
        ToolLosses { powers: [friction + stop, shaft, gear, self.telescope_power(s), other], gear_at_rotor }
    }

    /// The torsional mode of the tool with its motor held: the gearbox's spring (with the shaft's in series,
    /// effects.shaft_torsion) against J_c, w^2 = k / J_c, damping ratio c / (2 sqrt(k J_c)). `None` in the rigid model.
    pub fn tool_mode(&self) -> Option<Mode> {
        if !self.elastic {
            return None;
        }
        let (k, j) = (self.tool.gearbox.stiffness, self.tool_inertia());
        Some(Mode {
            hz: (k / j).sqrt() / std::f64::consts::TAU,
            damping: self.tool.gearbox.damping / (2.0 * (k * j).sqrt()),
        })
    }

    /// The telescope as drawn with the platform's centre at p: the outer tube down from the upper joint and the inner
    /// shaft up from the lower one, each as long as the shortest shaft less 50 mm (they overlap), with their widths.
    fn telescope(&self, p: V3) -> [(V3, V3, f64); 2] {
        let (upper, lower) = self.shaft.joints(p);
        let (length, ..) = self.shaft.geometry(p);
        let u: V3 = std::array::from_fn(|k| (lower[k] - upper[k]) / length);
        let part = (self.shaft.min_length - 0.05).min(length);
        [
            (upper, std::array::from_fn(|k| upper[k] + part * u[k]), TUBE_WIDTH),
            (lower, std::array::from_fn(|k| lower[k] - part * u[k]), INNER_WIDTH),
        ]
    }

    /// The telescope's tube and inner shaft as collision boxes, `PIECES` along each, each piece widened by its width.
    fn shaft_boxes(&self, p: V3) -> Vec<sim_core::Aabb> {
        let mut out = Vec::new();
        for (from, to, width) in self.telescope(p) {
            out.extend((0..PIECES).map(|n| {
                let at = |f: f64| -> V3 { std::array::from_fn(|i| from[i] + f * (to[i] - from[i])) };
                let (a, b) = (at(n as f64 / PIECES as f64), at((n + 1) as f64 / PIECES as f64));
                sim_core::Aabb {
                    centre: std::array::from_fn(|i| 0.5 * (a[i] + b[i])),
                    size: std::array::from_fn(|i| (b[i] - a[i]).abs() + width),
                }
            }));
        }
        out
    }
}

/// The telescope's outer tube and inner shaft as drawn and as collision bodies, m, and the pieces the collision boxes
/// cut each into.
const TUBE_WIDTH: f64 = 0.03;
const INNER_WIDTH: f64 = 0.016;
const PIECES: usize = 6;

impl Machine for Delta4 {
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        self.arms.inverse(&tool[..3], &mut motors[..3]);
        motors[3] = self.tool.ratio() * tool[3];
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        self.arms.forward(&motors[..3], &mut tool[..3]);
        tool[3] = motors[3] / self.tool.ratio();
    }

    /// The arms' state (the D3's), then the turn and its speed, the rotor's angle and speed (elastic) and the brake.
    fn state_len(&self) -> usize {
        self.n3 + if self.elastic { 5 } else { 3 }
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        self.arms.equilibrium(&tool[..3], state, &mut torque[..3]);
        // Nothing holds the turn at rest: no weight turns it.
        let rotor = self.tool.ratio() * tool[3];
        state.extend([tool[3], 0.0]);
        if self.elastic {
            state.extend([rotor, 0.0]);
        }
        state.push(0.0);
        torque[3] = self.tool.ripple.invert(0.0, rotor);
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let n = self.n3;
        self.arms.derivatives(&s[..n], &torque[..3], &mut out[..n]);
        self.tool_rates(s, torque[3], &mut out[n..]);
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        self.arms.tool_state(&s[..self.n3], &mut pos[..3], &mut vel[..3]);
        (pos[3], vel[3]) = self.turn(s);
    }
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        let n = self.n3;
        self.arms.set_tool_velocity(&mut s[..n], &vel[..3]);
        s[n + 1] = vel[3];
        if self.elastic {
            s[n + 3] = self.tool.ratio() * vel[3];
        }
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        self.arms.motor_state(&s[..self.n3], &mut pos[..3], &mut vel[..3]);
        (pos[3], vel[3]) = self.rotor(s);
    }

    /// The arms' (the D3's), and the turn's J_c c'' / i + J_rotor i c''.
    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        self.arms.feedforward_torque(&pos[..3], &acc[..3], &mut out[..3]);
        let i = self.tool.ratio();
        out[3] = self.tool_inertia() * acc[3] / i + self.tool.rotor_inertia * i * acc[3];
    }
    /// The arms' with the speed (the D3's, loss-free: without the telescope's friction), the turn's as without it.
    fn feedforward_torque_moving(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut [f64]) {
        self.arms.feedforward_torque_moving(&pos[..3], &vel[..3], &acc[..3], &mut out[..3]);
        let i = self.tool.ratio();
        out[3] = self.tool_inertia() * acc[3] / i + self.tool.rotor_inertia * i * acc[3];
    }

    fn energy(&self, s: &[f64]) -> f64 {
        self.arms.energy(&s[..self.n3]) + self.tool_energy(s).iter().sum::<f64>()
    }

    fn set_payload_attached(&mut self, attached: bool) {
        self.payload_attached = attached;
        self.arms.set_payload_attached(attached);
    }
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.brake_engaged = engaged;
        self.arms.set_brake_engaged(engaged);
    }

    /// The arms' three gearboxes, then the tool's.
    fn set_part_temperatures(&mut self, rises: &[f64]) {
        self.arms.set_part_temperatures(&rises[..rises.len().min(3)]);
        if let (Some(law), Some(rise)) = (&self.thermal, rises.get(3))
            && !self.tool.gearbox.is_coupling()
        {
            self.tool.gearbox.efficiency = law.gearbox_efficiency(*rise);
        }
    }

    /// The arms' modes (the D3's) and the tool's torsion (`tool_mode`), lowest first.
    fn modes(&self, tool_pose: &[f64]) -> Vec<Mode> {
        let mut out = self.arms.modes(&tool_pose[..3]);
        out.extend(self.tool_mode());
        out.sort_by(|a, b| a.hz.total_cmp(&b.hz));
        out
    }
    /// The arms' lowest mode, which an input shaper on the path is for; the tool's torsion is in `modes`.
    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        self.arms.resonance_hz(&tool_pose[..3])
    }
    fn mode_shape(&self, tool_pose: &[f64]) -> Option<(f64, Vec<f64>)> {
        let (hz, mut shape) = self.arms.mode_shape(&tool_pose[..3])?;
        shape.resize(self.state_len(), 0.0);
        Some((hz, shape))
    }

    fn unreachable(&self, tool: &[f64], slack: f64) -> Option<&'static str> {
        if tool.len() != 4 {
            return Some("error-target-out-of-range");
        }
        if let Some(why) = self.arms.unreachable(&tool[..3], slack) {
            return Some(why);
        }
        let (length, beta, _) = self.shaft.geometry([tool[0], tool[1], tool[2]]);
        if length < self.shaft.min_length - slack || length > self.shaft.max_length + slack {
            return Some("error-delta-shaft-stroke");
        }
        (beta > self.shaft.max_angle).then_some("error-delta-cardan-angle")
    }

    fn condition(&self, tool: &[f64]) -> Option<f64> {
        self.arms.condition(&tool[..3])
    }

    fn setpoint_slack(&self) -> f64 {
        self.arms.setpoint_slack()
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let arms = self.arms.signal_infos();
        let (base, effects) = arms.split_at(self.arm_base_signals);
        let mut out = base.to_vec();
        out.push(SignalInfo::new("tool.angle", "group-tool", "quantity-tool-angle", "rad"));
        out.push(SignalInfo::new("gearbox.4.output_torque", "motor-tool", "quantity-gearbox-torque", "N m"));
        out.push(SignalInfo::new("shaft.length", "group-shaft", "quantity-shaft-length", "m"));
        out.push(SignalInfo::new("shaft.cardan_angle", "group-shaft", "quantity-cardan-angle", "rad"));
        out.push(SignalInfo::new("shaft.speed", "group-shaft", "quantity-shaft-speed", "rad/s"));
        out.extend_from_slice(effects);
        out.extend(TORQUE_PARTS.map(|part| {
            SignalInfo::new(format!("motor.4.torque_{part}"), "motor-tool", &format!("quantity-torque-{part}"), "N m")
        }));
        let gear = if self.tool.gearbox.is_coupling() {
            "quantity-coupling-loss-power"
        } else {
            "quantity-gearbox-loss-power"
        };
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}joint_4"), "group-tool", "quantity-friction-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}shaft_4"), "motor-tool", "quantity-shaft-friction-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}gearbox_4"), "motor-tool", gear, "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}telescope"), "group-shaft", "quantity-friction-power", "W"));
        out.push(SignalInfo::new(format!("{LOSS_PREFIX}other_4"), "loss-other", "quantity-power-given-off", "W"));
        out
    }

    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.arms.base_signals(&s[..self.n3], &torque[..3], out);
        let (c, w) = self.turn(s);
        let (length, beta, azimuth) = self.shaft.geometry(self.platform(s).0);
        out.extend([c, self.tool_gear_output(s, torque[3]), length, beta, w * cardan_speed_ratio(beta, c - azimuth)]);
    }

    /// The arms' effect signals with the tool's energy added to theirs, then the tool motor's torque parts and the
    /// tool's losses.
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.arms.effect_signals(&s[..self.n3], &torque[..3], out);
        let at = out.len() - 3;
        for (k, e) in self.tool_energy(s).into_iter().enumerate() {
            out[at + k] += e;
        }
        let j = &self.tool;
        let ((_, w), (_, speed)) = (self.turn(s), self.rotor(s));
        let mut rates = [0.0; 5];
        self.tool_rates(s, torque[3], &mut rates);
        let losses = self.tool_losses(s, torque[3]);
        let held = j.brake.torque(self.brake_engaged, speed, self.brake(s)).0;
        let parts = [
            (self.tool_inertia() + j.reflected_inertia()) * rates[1] / j.ratio(),
            0.0,
            j.shaft.torque(speed) + held + losses.gear_at_rotor + j.friction.torque(w) / j.ratio(),
        ];
        out.extend(parts);
        out.push(torque[3] - parts.iter().sum::<f64>());
        out.extend(losses.powers);
    }

    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.arms.loss_powers(&s[..self.n3], &torque[..3], out);
        out.extend(self.tool_losses(s, torque[3]).powers);
    }

    /// The arms' three gearboxes and the tool's (none on direct drives).
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        let mut out = self.arms.thermal_parts();
        if !self.tool.gearbox.is_coupling() {
            out.push(ThermalPart {
                id: "gearbox_4",
                kind: "gearbox",
                owner_key: "motor-tool",
                scene_part: "gearbox",
                scene_index: 3,
            });
        }
        out
    }

    /// The D3's scene, then the tool's motor and gearbox in the base's centre, the telescope (outer tube from the upper
    /// joint, inner shaft from the lower), the two joints, and a bar under the platform turned with the tool.
    fn scene(&self, s: &[f64], out: &mut Scene) {
        self.arms.scene(&s[..self.n3], out);
        let (p, _) = self.platform(s);
        let (c, _) = self.turn(s);
        let (rotor, speed) = self.rotor(s);
        let (upper, lower) = self.shaft.joints(p);
        let [(tube_from, tube_to, tube), (inner_from, inner_to, inner)] = self.telescope(p);
        let dark = [0.25, 0.25, 0.28];
        out.parts.push(Part::new(
            sim_core::scene::MOTOR_PART,
            Shape::Box { size: [0.09, 0.09, 0.12] },
            [0.0, 0.0, 0.16],
            [0.9, 0.45, 0.1],
        ));
        out.parts.push(Part::new("gearbox", Shape::Box { size: [0.08, 0.08, 0.05] }, [0.0, 0.0, 0.065], dark));
        out.parts.push(
            Part::new("rotor-bar", Shape::Box { size: [0.07, 0.004, 0.012] }, [0.0, 0.0, 0.225], [0.1, 0.1, 0.1])
                .turned(rotor),
        );
        out.parts.push(Part::new(
            "shaft-tube",
            Shape::Segment { to: tube_to, width: tube },
            tube_from,
            [0.7, 0.72, 0.75],
        ));
        out.parts.push(Part::new(
            "shaft-inner",
            Shape::Segment { to: inner_to, width: inner },
            inner_from,
            [0.5, 0.5, 0.52],
        ));
        for at in [upper, lower] {
            out.parts.push(Part::new("cardan", Shape::Box { size: [0.035, 0.035, 0.035] }, at, dark));
        }
        let half = [0.06 * c.cos(), 0.06 * c.sin(), 0.0];
        let under = [p[0], p[1], p[2] - 0.025];
        out.parts.push(Part::new(
            "tool",
            Shape::Segment { to: std::array::from_fn(|k| under[k] + half[k]), width: 0.015 },
            std::array::from_fn(|k| under[k] - half[k]),
            [0.85, 0.75, 0.2],
        ));
        // The tool motor's speed after the arms' three (the D3's z label is not in rpm).
        let rpm = Label {
            pos: [0.0, 0.0, 0.3],
            owner_key: "motor-tool",
            value: speed * 60.0 / std::f64::consts::TAU,
            unit_key: "unit-rpm",
            decimals: 0,
        };
        let at = out.labels.iter().rposition(|l| l.unit_key == "unit-rpm").map_or(out.labels.len(), |i| i + 1);
        out.labels.insert(at, rpm);
    }

    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        self.arms.scene_point(&tool_pose[..3])
    }

    /// The D3's bodies (platform, payload, forearm rods) and the telescopic shaft between its joints.
    // shortcut: the shaft's pieces are boxes along the scene's axes, as the D3's rods; the motor and gearbox above the
    // base are not bodies.
    fn collision_bodies(&self, s: &[f64]) -> Vec<sim_core::Aabb> {
        let mut out = self.arms.collision_bodies(&s[..self.n3]);
        out.extend(self.shaft_boxes(self.platform(s).0));
        out
    }
}

#[cfg(test)]
mod tests;
