//! Single-axis test machine: a motor at the left end drives a carriage through a belt loop around an
//! idler at the right end. x = r * pulley angle. Component checks run here once and then hold for
//! every machine that uses the component.
//!
//! The motor pulley cuts the belt into two sections of uniform tension:
//! section 1 runs from the pulley straight to the carriage (length const + x, belt fed in by + r a),
//! section 2 from the pulley around the idler to the carriage (length const - x, fed by - r a).
//! Belt force on the carriage: T2 - T1. Belt torque on the pulley: r (T1 - T2).

use sim_components::mech::{Belt, CableChain, Gearbox, HardStop, HoldingBrake, Lugre, Ripple, SectionForce, ShaftFriction, ToothMesh};
use sim_core::{AxisInfo, BeltSection, Config, ConfigError, ENERGY_PARTS, FRICTION_SIGNALS, FrictionContact, LOSS_PREFIX, Label, Machine, MachineEntry, MotorInfo, Part, Scene, Shape, SignalInfo, TORQUE_PARTS, ThermalPart};

pub static ENTRY: MachineEntry = MachineEntry {
    id: "single-axis",
    key: "machine-single-axis",
    default_config: include_str!("../default.toml"),
    i18n: &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))],
    build,
    suite_configs,
};

const G: f64 = 9.80665;

/// Effects that take energy out or put it in; the standard suite runs with them off.
pub const LOSSY_EFFECTS: [&str; 10] = [
    "belt_damping",
    "guide_friction",
    "load_dependent_friction",
    "motor_friction",
    "gearbox_efficiency",
    "torque_ripple",
    "tooth_meshing",
    "temperature",
    "process_force",
    "holding_brake",
];

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    Ok(Box::new(SingleAxis::new(cfg)?))
}

/// The default config with every loss switched off.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    cfg.set_text("gearbox.damping_nm_s_rad = 0.0").unwrap();
    cfg.set_text("hard_stop.damping_n_s_m = 0.0").unwrap();
    cfg.set_text("cable_chain.drag_n = 0.0").unwrap();
    cfg
}

/// Loss-free rigid and full models, level and inclined (so that gravity takes part).
fn suite_configs() -> Vec<Config> {
    let mut out = Vec::new();
    for (elastic, incline) in [(false, 0.0), (false, 30.0), (true, 0.0), (true, 30.0)] {
        let mut cfg = lossless_config();
        // The inclined runs have the tooth stiffness and the cable chain in, so the energy check covers the
        // belt with and without, and the chain where its weight counts.
        cfg.set_text(&format!("effects.belt_tooth_stiffness = {}", incline > 0.0)).unwrap();
        cfg.set_text(&format!("effects.cable_chain = {}", incline > 0.0)).unwrap();
        cfg.set_text(&format!("effects.part_heating = {}", incline > 0.0)).unwrap();
        cfg.set_text(&format!("effects.belt_stretch = {elastic}")).unwrap();
        cfg.set_text(&format!("geometry.incline_deg = {incline:.1}")).unwrap();
        out.push(cfg);
    }
    out
}

pub struct SingleAxis {
    elastic: bool,
    r: f64,
    /// Unstretched belt fed per radian of pulley. The pulley radius holds at pretension, where the
    /// belt is already stretched, so that the kinematics are exact for an evenly tensioned belt.
    r_mat: f64,
    span_min: f64,
    m_carriage: f64,
    m_payload: f64,
    j_rotor: f64,
    /// Pulley inertia including the belt mass.
    j_pulley: f64,
    /// Gravity along the axis (pulling towards -x) and into the guide.
    g_along: f64,
    g_normal: f64,
    belt: Belt,
    /// Unstretched belt per section at x = 0 with the pulley at angle 0.
    mat0: [f64; 2],
    friction: Lugre,
    shaft: ShaftFriction,
    ripple: Ripple,
    mesh: ToothMesh,
    gearbox: Gearbox,
    stops: HardStop,
    chain: CableChain,
    /// Force on the carriage towards +x while the cycle dwells.
    process: f64,
    dwelling: bool,
    brake: HoldingBrake,
    brake_engaged: bool,
    payload_attached: bool,
    axes: [AxisInfo; 1],
    motors: [MotorInfo; 1],
}

// Elastic state: [x, a, p, vx, va, wp, s, slack, brake]. a: pulley angle, p: motor shaft angle, s: friction
// state, slack: seconds a belt section has spent without tension, brake: deflection of the holding brake.
const N_ELASTIC: usize = 9;
// Rigid state: [x, v, s, brake].
const N_RIGID: usize = 4;

impl SingleAxis {
    pub fn new(cfg: &Config) -> Result<Self, ConfigError> {
        let g = if cfg.bool("effects.gravity")? { G } else { 0.0 };
        let incline = cfg.f64("geometry.incline_deg")?.to_radians();
        let r = cfg.positive("geometry.pulley_radius_m")?;
        let belt = Belt::from_config(cfg)?;
        // The gearbox's own inertia turns with the rotor.
        let j_rotor = cfg.positive("motor.rotor_inertia_kgm2")? + cfg.non_negative("gearbox.inertia_at_input_kgm2")?;
        let process = cfg.f64("process.force_x_n")?;
        let mut m = Self {
            elastic: cfg.bool("effects.belt_stretch")?,
            r,
            r_mat: belt.rest_length(r),
            span_min: cfg.positive("geometry.span_min_m")?,
            m_carriage: cfg.positive("mass.carriage_kg")?,
            m_payload: if cfg.bool("effects.payload")? { cfg.non_negative("mass.payload_kg")? } else { 0.0 },
            j_rotor,
            j_pulley: cfg.positive("pulley.inertia_kgm2")?,
            g_along: g * incline.sin(),
            g_normal: g * incline.cos(),
            belt,
            mat0: [0.0; 2],
            friction: Lugre::from_config(cfg, "friction_x")?,
            shaft: ShaftFriction::from_config(cfg)?,
            ripple: Ripple::from_config(cfg)?,
            mesh: ToothMesh::from_config(cfg)?,
            gearbox: Gearbox::from_config(cfg)?,
            stops: HardStop::from_config(cfg)?,
            chain: CableChain::from_config(cfg)?,
            process: if cfg.bool("effects.process_force")? { process } else { 0.0 },
            dwelling: false,
            brake: HoldingBrake::from_config(cfg, j_rotor)?,
            brake_engaged: false,
            payload_attached: true,
            axes: [AxisInfo {
                id: "x",
                key: "axis-x",
                unit: "m",
                min: 0.0,
                max: cfg.positive("geometry.stroke_m")?,
                max_vel: cfg.positive("axis.x.max_velocity_m_s")?,
                max_acc: cfg.positive("axis.x.max_acceleration_m_s2")?,
                max_jerk: cfg.positive("axis.x.max_jerk_m_s3")?,
            }],
            motors: [MotorInfo { id: "m", key: "motor-m" }],
        };
        let geo = m.geo(0.0);
        m.mat0 = geo.map(|g| m.belt.rest_length(g));
        // The whole belt runs at carriage speed, so all of its mass sits on the pulley.
        m.j_pulley += m.belt.mass_per_m * (geo[0] + geo[1] - 2.0 * m.belt.tooth_length) * r * r;
        Ok(m)
    }

    fn m(&self) -> f64 {
        self.m_carriage + if self.payload_attached { self.m_payload } else { 0.0 }
    }
    /// Inertia at the pulley: pulley plus rotor seen through the gearbox.
    fn j(&self) -> f64 {
        self.j_pulley + self.gearbox.ratio * self.gearbox.ratio * self.j_rotor
    }
    fn m_total(&self) -> f64 {
        self.m() + self.j() / (self.r * self.r)
    }

    /// Geometric length of the two belt sections. Each ends at the motor pulley once, where the teeth
    /// in mesh count as more belt (see `Belt::tooth_length`).
    fn geo(&self, x: f64) -> [f64; 2] {
        let (stroke, s, teeth) = (self.axes[0].max, self.span_min, self.belt.tooth_length);
        [s + x + teeth, 3.0 * s + 2.0 * stroke - x + teeth]
    }
    fn mat(&self, a: f64) -> [f64; 2] {
        [self.mat0[0] + self.r_mat * a, self.mat0[1] - self.r_mat * a]
    }
    fn sections(&self, s: &[f64]) -> [SectionForce; 2] {
        let (geo, mat, r) = (self.geo(s[0]), self.mat(s[1]), self.r_mat);
        [
            self.belt.section(geo[0], mat[0], s[3], r * s[4]),
            self.belt.section(geo[1], mat[1], -s[3], -r * s[4]),
        ]
    }

    /// The guide carries the weight; the friction levels are given for the carriage without payload.
    fn load_scale(&self) -> f64 {
        self.friction.load_scale(self.m() * self.g_normal, self.m_carriage * self.g_normal)
    }

    /// Force of the end stops on the carriage.
    fn stop_force(&self, x: f64, v: f64) -> f64 {
        self.stops.force(x, v, self.axes[0].max)
    }

    /// What the cable chain and the process put on the carriage at x and v, beside the chain's inertia and
    /// weight: the chain's drag (it fades out at rest like the guide friction's Stribeck speed), what picking
    /// up chain costs (see `CableChain::mass`), and the process force while the cycle dwells.
    fn extra_force(&self, x: f64, v: f64) -> f64 {
        let process = if self.dwelling { self.process } else { 0.0 };
        process - self.chain.drag(v, self.friction.stribeck_v) - 0.5 * self.chain.mass(x).1 * v * v
    }

    fn elastic_derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let r = self.r;
        let sec = self.sections(s);
        let (ff, ds) = self.friction.eval(s[3], s[6], self.load_scale());
        let gear = self.gearbox.torque(s[2], s[5], s[1], s[4]);
        out[..3].copy_from_slice(&s[3..6]);
        // The chain's moving run has inertia and, on an incline, weight like the carriage; it does not load the guide.
        let m = self.m() + self.chain.mass(s[0]).0;
        out[3] = (sec[1].tension - sec[0].tension - ff + self.stop_force(s[0], s[3]) + self.extra_force(s[0], s[3])) / m - self.g_along;
        let belt_torque = self.r_mat * (sec[0].material - sec[1].material);
        out[4] = (gear.output + belt_torque + r * self.mesh.force(r * s[1])) / self.j_pulley;
        let (brake, brake_rate) = self.brake.torque(self.brake_engaged, s[5], s[8]);
        out[5] = (self.ripple.apply(torque[0], s[2]) - gear.rotor - self.shaft.torque(s[5]) - brake) / self.j_rotor;
        out[8] = brake_rate;
        out[6] = ds;
        out[7] = if sec.iter().any(|f| f.tension <= 0.0) { 1.0 } else { 0.0 };
    }

    /// Rigid model: acceleration, guide friction force, friction state rate, the torque at the gearbox output and
    /// the torque that goes into it before its loss.
    ///
    /// The gearbox loses torque only on what it transmits, which is the motor torque minus what
    /// accelerates the rotor itself; the second pass uses the acceleration from the first for that.
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> (f64, f64, f64, f64, f64) {
        let (r, ratio) = (self.r, self.gearbox.ratio);
        let (angle, speed) = (ratio * s[0] / r, ratio * s[1] / r);
        // Belt force the motor would give with a loss-free gearbox and a massless rotor; a brake holds the rotor.
        let brake = self.brake.torque(self.brake_engaged, speed, s[3]).0;
        let drive = ratio * (self.ripple.apply(torque[0], angle) - self.shaft.torque(speed) - brake) / r;
        let (ff, ds) = self.friction.eval(s[1], s[2], self.load_scale());
        let m = self.m() + self.chain.mass(s[0]).0;
        let load = self.mesh.force(s[0]) - ff - m * self.g_along + self.stop_force(s[0], s[1]) + self.extra_force(s[0], s[1]);
        let rotor = ratio * ratio * self.j_rotor / (r * r);
        let solve = |loss: f64| (loss * drive + load) / (m + self.j_pulley / (r * r) + loss * rotor);
        let through_gear = drive - rotor * solve(1.0);
        let loss = self.gearbox.loss_factor(through_gear * r, s[1] / r);
        let a = solve(loss);
        // The gearbox output carries what the rotor's own acceleration and the losses leave of the motor's torque.
        (a, ff, ds, loss * (drive - rotor * a) * r, (drive - rotor * a) * r)
    }

    /// Kinetic, potential and elastic energy (see `ENERGY_PARTS`).
    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        // The chain's moving run weighs mass_per_m (length0 + x / 2): lifted from 0 to x, that integrates to this.
        let chain_height = self.chain.mass_per_m * (self.chain.length0 * s[0] + 0.25 * s[0] * s[0]);
        let potential = (self.m() * s[0] + chain_height) * self.g_along;
        let (chain, stops) = (self.chain.mass(s[0]).0, self.stops.energy(s[0], self.axes[0].max));
        if !self.elastic {
            return [0.5 * (self.m_total() + chain) * s[1] * s[1], potential, stops];
        }
        let (geo, mat) = (self.geo(s[0]), self.mat(s[1]));
        let kinetic = 0.5 * (self.m() + chain) * s[3] * s[3] + 0.5 * self.j_pulley * s[4] * s[4] + 0.5 * self.j_rotor * s[5] * s[5];
        [kinetic, potential, stops + self.belt.energy(geo[0], mat[0]) + self.belt.energy(geo[1], mat[1]) + self.gearbox.energy(s[2], s[1])]
    }

    /// The loss powers in the order of `effect_infos`, the gearbox loss as a torque at the rotor and the brake torque,
    /// which the torque breakdown takes too. No derivatives: cheap enough for every step.
    fn losses(&self, s: &[f64], torque: &[f64]) -> ([f64; 7], f64, f64) {
        let (r, ratio) = (self.r, self.gearbox.ratio);
        let (x, v, state) = if self.elastic { (s[0], s[3], s[6]) } else { (s[0], s[1], s[2]) };
        let ff = self.friction.eval(v, state, self.load_scale()).0;
        let drag = self.chain.drag(v, self.friction.stribeck_v);
        let (mut angle, mut speed) = ([0.0], [0.0]);
        self.motor_state(s, &mut angle, &mut speed);
        let (angle, speed) = (angle[0], speed[0]);
        // Gearbox loss at the rotor and as power, brake, power of the tooth meshing put in, belt damping.
        let (gear_loss, gear_power, brake, mesh_in, belt) = if self.elastic {
            let gear = self.gearbox.torque(s[2], s[5], s[1], s[4]);
            let (geo, mat) = (self.geo(x), self.mat(s[1]));
            let belt = self.belt.loss_power(geo[0], mat[0], v, self.r_mat * s[4]) + self.belt.loss_power(geo[1], mat[1], -v, -self.r_mat * s[4]);
            let brake = self.brake.torque(self.brake_engaged, s[5], s[8]).0;
            (gear.rotor - gear.output / ratio, self.gearbox.loss_power(s[2], s[5], s[1], s[4]), brake, r * self.mesh.force(r * s[1]) * s[4], belt)
        } else {
            let (_, _, _, out, into) = self.rigid_solve(s, torque);
            let loss = (into - out) / ratio;
            (loss, loss * speed, self.brake.torque(self.brake_engaged, speed, s[3]).0, self.mesh.force(x) * v, 0.0)
        };
        let process_in = if self.dwelling { self.process * v } else { 0.0 };
        let other = (torque[0] - self.ripple.apply(torque[0], angle)) * speed - mesh_in - process_in;
        let stops = self.stops.loss_power(x, v, self.axes[0].max);
        ([ff * v, drag * v, (self.shaft.torque(speed) + brake) * speed, gear_power, belt, stops, other], gear_loss, brake)
    }

    /// The signals of the effect views, in the order of `effect_infos`, with the transpose r / i:
    /// inertia = r / i ((m + J / r^2) a + m' v^2 / 2), a the carriage's acceleration and J pulley and rotor seen at the
    /// pulley, as a rigid machine would turn them; gravity = r / i * m g sin(incline); friction = shaft friction +
    /// brake + gearbox loss at the rotor + r / i (guide friction + chain drag); rest = motor torque less these.
    fn view_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (r, ratio) = (self.r, self.gearbox.ratio);
        let (x, v, state) = if self.elastic { (s[0], s[3], s[6]) } else { (s[0], s[1], s[2]) };
        let scale = self.load_scale();
        let ff = self.friction.eval(v, state, scale).0;
        let drag = self.chain.drag(v, self.friction.stribeck_v);
        let (chain_mass, chain_growth) = self.chain.mass(x);
        let (mut angle, mut speed) = ([0.0], [0.0]);
        self.motor_state(s, &mut angle, &mut speed);
        let speed = speed[0];
        let (powers, gear_loss, brake) = self.losses(s, torque);
        // Carriage acceleration.
        let acc = if self.elastic {
            let mut d = [0.0; N_ELASTIC];
            self.elastic_derivatives(s, torque, &mut d);
            d[3]
        } else {
            self.rigid_solve(s, torque).0
        };
        let m = self.m() + chain_mass;
        let parts = [
            r / ratio * ((m + self.j() / (r * r)) * acc + 0.5 * chain_growth * v * v),
            r / ratio * m * self.g_along,
            self.shaft.torque(speed) + brake + gear_loss + r / ratio * (ff + drag),
        ];
        out.extend(parts);
        out.push(torque[0] - parts.iter().sum::<f64>());
        out.extend(powers);
        out.extend(self.energy_parts(s));
        out.push(v);
        out.extend(self.friction.split(v, state, scale));
        out.push(scale);
        // Stretch per belt section: its length less its unstretched belt; in the rigid model that of pretension.
        let geo = self.geo(x);
        let mat = if self.elastic { self.mat(s[1]) } else { geo.map(|g| self.belt.rest_length(g)) };
        out.extend([geo[0] - mat[0], geo[1] - mat[1]]);
    }

    /// The signal list of `effect_signals`.
    fn effect_infos(&self, out: &mut Vec<SignalInfo>) {
        let (motor, axis) = (&self.motors[0], self.axes[0].key);
        out.extend(TORQUE_PARTS.map(|part| SignalInfo::new(format!("motor.{}.torque_{part}", motor.id), motor.key, &format!("quantity-torque-{part}"), "N m")));
        let powers = [
            ("guide_x", axis, "quantity-friction-power"),
            ("cable_chain", "group-cable_chain", "quantity-friction-power"),
            ("shaft_m", motor.key, "quantity-shaft-friction-power"),
            ("gearbox_m", motor.key, if self.gearbox.is_coupling() { "quantity-coupling-loss-power" } else { "quantity-gearbox-loss-power" }),
            ("belt", "group-belt", "quantity-damping-power"),
            ("hard_stops", "group-hard_stop", "quantity-damping-power"),
            ("other", "loss-other", "quantity-power-given-off"),
        ];
        out.extend(powers.map(|(id, owner, quantity)| SignalInfo::new(format!("{LOSS_PREFIX}{id}"), owner, quantity, "W")));
        out.extend(ENERGY_PARTS.map(|part| SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")));
        for part in FRICTION_SIGNALS {
            let unit = match part {
                "speed" => "m/s",
                "load_scale" => "",
                _ => "N",
            };
            out.push(SignalInfo::new(format!("axis.x.friction_{part}"), axis, &format!("quantity-friction-{}", part.replace('_', "-")), unit));
        }
        out.extend(self.belt_sections().iter().map(|b| SignalInfo::new(format!("{}.stretch", b.id), b.owner_key, "quantity-section-stretch", "m")));
    }
}

impl Machine for SingleAxis {
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        motors[0] = self.gearbox.ratio * tool[0] / self.r;
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        tool[0] = motors[0] * self.r / self.gearbox.ratio;
    }

    fn state_len(&self) -> usize {
        if self.elastic { N_ELASTIC } else { N_RIGID }
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let (x, r, ratio) = (tool[0], self.r, self.gearbox.ratio);
        let weight = (self.m() + self.chain.mass(x).0) * self.g_along;
        if !self.elastic {
            *state = vec![x, 0.0, 0.0, 0.0];
            let shaft = (weight - self.mesh.force(x)) * r / ratio;
            torque[0] = self.ripple.invert(shaft, ratio * x / r);
            return;
        }
        // Carriage balance: T2 - T1 = m g sin(incline). The belt total fixes the level.
        let geo = self.geo(x);
        let mat = self.belt.static_material(&geo, &[0.0, weight], self.mat0.iter().sum());
        let a = (mat[0] - self.mat0[0]) / self.r_mat;
        let m: [f64; 2] = std::array::from_fn(|i| self.belt.section(geo[i], mat[i], 0.0, 0.0).material);
        let gear = -self.r_mat * (m[0] - m[1]) - r * self.mesh.force(r * a);
        let p = ratio * (a + self.gearbox.static_twist(gear));
        *state = vec![x, a, p, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        torque[0] = self.ripple.invert(gear / ratio, p);
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        if self.elastic {
            return self.elastic_derivatives(s, torque, out);
        }
        let (a, _, ds, ..) = self.rigid_solve(s, torque);
        let brake_rate = self.brake.torque(self.brake_engaged, self.gearbox.ratio * s[1] / self.r, s[3]).1;
        out.copy_from_slice(&[s[1], a, ds, brake_rate]);
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        pos[0] = s[0];
        vel[0] = if self.elastic { s[3] } else { s[1] };
    }
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        if self.elastic {
            (s[3], s[4], s[5]) = (vel[0], vel[0] / self.r, self.gearbox.ratio * vel[0] / self.r);
        } else {
            s[1] = vel[0];
        }
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        if self.elastic {
            pos[0] = s[2];
            vel[0] = s[5];
        } else {
            self.inverse(&s[0..1], pos);
            self.inverse(&s[1..2], vel);
        }
    }

    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        let chain = self.chain.mass(pos[0]).0;
        out[0] = ((self.m_total() + chain) * acc[0] + (self.m() + chain) * self.g_along) * self.r / self.gearbox.ratio;
    }

    fn energy(&self, s: &[f64]) -> f64 {
        self.energy_parts(s).iter().sum()
    }

    fn set_payload_attached(&mut self, attached: bool) {
        self.payload_attached = attached;
    }
    fn set_dwelling(&mut self, dwelling: bool) {
        self.dwelling = dwelling;
    }
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.brake_engaged = engaged;
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let key = self.axes[0].key;
        let mut out = vec![
            SignalInfo::new("belt.1.tension", "belt-section-1", "quantity-tension", "N"),
            SignalInfo::new("belt.2.tension", "belt-section-2", "quantity-tension", "N"),
            SignalInfo::new("axis.x.belt_stretch", key, "quantity-belt-stretch", "m"),
            SignalInfo::new("axis.x.friction", key, "quantity-friction-force", "N"),
            SignalInfo::new("belt.slack_time", "group-belt", "quantity-slack-time", "s"),
            SignalInfo::new("pulley.m.radial_load", self.motors[0].key, "quantity-radial-load", "N"),
            SignalInfo::new("axis.x.guide_load", key, "quantity-guide-load", "N"),
            SignalInfo::new("gearbox.m.output_torque", self.motors[0].key, "quantity-gearbox-torque", "N m"),
        ];
        self.effect_infos(&mut out);
        out
    }

    fn friction_contacts(&self) -> Vec<FrictionContact> {
        vec![FrictionContact { id: "axis.x", owner_key: self.axes[0].key }]
    }
    fn friction_curve(&self, i: usize, speed: f64, load_scale: f64) -> [f64; 3] {
        if i == 0 { self.friction.steady(speed, load_scale) } else { [0.0; 3] }
    }

    /// The two belt sections, in the order of the signals `belt.<i>.tension`.
    fn belt_sections(&self) -> Vec<BeltSection> {
        let pretension = self.belt.pretension;
        vec![BeltSection { id: "belt.1", owner_key: "belt-section-1", pretension }, BeltSection { id: "belt.2", owner_key: "belt-section-2", pretension }]
    }

    /// The gearbox, where there is one (not on a direct drive, see `Gearbox::is_coupling`), and the guide, shown by
    /// the carriage.
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        let gearbox = ThermalPart { id: "gearbox_m", kind: "gearbox", owner_key: self.motors[0].key, scene_part: "gearbox", scene_index: 0 };
        let guide = ThermalPart { id: "guide_x", kind: "guide", owner_key: self.axes[0].key, scene_part: "carriage", scene_index: 0 };
        if self.gearbox.is_coupling() { vec![guide] } else { vec![gearbox, guide] }
    }

    /// The rigid model has no belt stretch: it reports pretension and zero stretch. The belt wraps the motor
    /// pulley by half a turn, so both sections pull its shaft the same way: the radial load is the sum of
    /// their tensions. The guide carries the weight of what it moves. The gearbox torque is the one at its
    /// output, positive where it drives the carriage towards +x: in the elastic model what the gear's spring
    /// passes on to the pulley, in the rigid one what its solver works out.
    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        if self.elastic {
            let sec = self.sections(s);
            let (ff, _) = self.friction.eval(s[3], s[6], self.load_scale());
            out.extend([sec[0].tension, sec[1].tension, s[0] - self.r * s[1], ff, s[7], sec[0].tension + sec[1].tension]);
        } else {
            let (ff, _) = self.friction.eval(s[1], s[2], self.load_scale());
            out.extend([self.belt.pretension, self.belt.pretension, 0.0, ff, 0.0, 2.0 * self.belt.pretension]);
        }
        out.push(self.m() * self.g_normal);
        out.push(if self.elastic { self.gearbox.torque(s[2], s[5], s[1], s[4]).output } else { self.rigid_solve(s, torque).3 });
    }
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.view_signals(s, torque, out);
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(s, torque).0);
    }

    fn scene(&self, s: &[f64], out: &mut Scene) {
        use sim_core::scene::{belt, drive_unit, idler, on_circle};
        use std::f64::consts::{FRAC_PI_4, TAU};
        let (x, r, ratio, span) = (s[0], self.r, self.gearbox.ratio, self.span_min);
        let (mut rotor, mut rotor_speed) = ([0.0], [0.0]);
        self.motor_state(s, &mut rotor, &mut rotor_speed);
        let pulley = if self.elastic { s[1] } else { rotor[0] / ratio };
        let tension = if self.elastic { self.sections(s).map(|f| f.tension) } else { [self.belt.pretension; 2] };
        let shade = tension.map(|t| ((t - self.belt.pretension) / self.belt.pretension.max(1.0)).clamp(-1.0, 1.0) as f32);

        // The belt loop runs in one plane in front of the beam, the motor sits behind it at the left end.
        let y = -0.05;
        let (end, cx, floor) = (2.0 * span + self.axes[0].max, span + x, -0.4);
        let part = Part::new;
        out.labels.clear();
        out.parts.clear();
        out.parts.extend([
            part("beam", Shape::Box { size: [end - 0.14, 0.07, 0.09] }, [end / 2.0, 0.045, 0.0], [0.62, 0.64, 0.68]),
            part("post", Shape::Box { size: [0.06, 0.06, -0.045 - floor] }, [0.13, 0.045, (floor - 0.045) / 2.0], [0.45, 0.47, 0.5]),
            part("post", Shape::Box { size: [0.06, 0.06, -0.045 - floor] }, [end - 0.13, 0.045, (floor - 0.045) / 2.0], [0.45, 0.47, 0.5]),
            // The tool: drawn where `scene_point` says.
            part("carriage", Shape::Box { size: [0.16, 0.05, 0.03] }, self.scene_point(&[x]).unwrap_or_default(), [0.2, 0.22, 0.26]),
            part("clamp", Shape::Box { size: [0.06, 0.035, 0.03] }, [cx, y, r + 0.017], [0.85, 0.3, 0.2]),
        ]);
        if self.payload_attached && self.m_payload > 0.0 {
            out.parts.push(part("payload", Shape::Box { size: [0.1, 0.1, 0.1] }, [cx, -0.03, 0.125], [0.2, 0.7, 0.3]));
        }
        // A positive pulley angle moves the carriage to the right: the top of the pulley moves
        // right, which is clockwise seen from the front.
        drive_unit(&mut out.parts, [0.0, 0.0], y, r, -pulley, -rotor[0], [0.9, 0.45, 0.1]);
        idler(&mut out.parts, [end, 0.0], y, r, -x / r);

        // The belt loop, starting and ending at the clamp on the carriage.
        let (a, b) = ([0.0, 0.0], [end, 0.0]);
        let path = [
            [cx, r],
            [0.0, r],
            on_circle(a, r, 3.0 * FRAC_PI_4),
            on_circle(a, r, 4.0 * FRAC_PI_4),
            on_circle(a, r, 5.0 * FRAC_PI_4),
            [0.0, -r],
            [end, -r],
            on_circle(b, r, -FRAC_PI_4),
            on_circle(b, r, 0.0),
            on_circle(b, r, FRAC_PI_4),
            [end, r],
            [cx, r],
        ];
        // Strips up to the middle of the motor pulley belong to the section straight to the carriage.
        belt(&mut out.parts, &path, y, 0.025, 0.04, |i| shade[if i < 3 { 0 } else { 1 }]);

        out.labels.extend([
            Label { pos: [0.0, y, 0.11], owner_key: self.motors[0].key, value: rotor_speed[0] * 60.0 / TAU, unit_key: "unit-rpm", decimals: 0 },
            Label { pos: [cx, y, 0.22], owner_key: self.axes[0].key, value: x, unit_key: "unit-m", decimals: 3 },
        ]);
    }

    /// The carriage on the beam.
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        Some([self.span_min + tool_pose[0], -0.03, 0.06])
    }
}

#[cfg(test)]
mod tests;
