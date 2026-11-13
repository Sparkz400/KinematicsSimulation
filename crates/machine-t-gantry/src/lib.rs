//! T-gantry: transformation, belt routing, assembly, scene description.
//!
//! Two fixed motors A (left) and B (right) at the ends of the X beam drive one belt. The belt ends are
//! clamped at the top of the Z axis and it runs around an idler at the bottom of the Z axis:
//!
//! clamp -> carriage idler -> motor A -> carriage idler -> Z-bottom idler -> carriage idler -> motor B
//! -> carriage idler -> clamp
//!
//! With pulley angles a and b after the gearbox: x = r/2 (a + b), z = r/2 (a - b).
//! Both motors the same way moves X, against each other moves Z.
//!
//! The two motor pulleys cut the belt into three sections of uniform tension:
//!
//! | section | from -> to            | geometric length      | belt fed in by |
//! | ------- | --------------------- | --------------------- | -------------- |
//! | 1       | clamp -> motor A      | const + x + z         | + r a          |
//! | 2       | motor A -> motor B    | const - 2 z           | - r a + r b    |
//! | 3       | motor B -> clamp      | const - x + z         | - r b          |
//!
//! Belt forces: Fx = T3 - T1, Fz = 2 T2 - T1 - T3, pulley A: r (T1 - T2), pulley B: r (T2 - T3).
//!
//! Belt speed past the idlers, as each idler sees it. The top of pulley A moves at vx + vz, the carriage
//! at vx, so belt passes the two upper carriage idlers at vz (it only follows the clamp up and down). The
//! lower run comes back from the pulleys at vx + vz and vx - vz against the carriage, so belt passes the
//! lower carriage idlers at 2 vx + vz (left) and 2 vx - vz (right), and the Z-bottom idler, which rises
//! with z, at 2 vx.

use sim_components::mech::{Belt, CableChain, Gearbox, HardStop, HoldingBrake, Lugre, Ripple, SectionForce, ShaftFriction, ToothMesh};
use sim_core::{AxisInfo, BeltSection, Config, ConfigError, ENERGY_PARTS, FRICTION_SIGNALS, FrictionContact, LOSS_PREFIX, Label, Machine, MachineEntry, MotorInfo, Part, Scene, Shape, SignalInfo, TORQUE_PARTS, ThermalPart};

pub static ENTRY: MachineEntry = MachineEntry {
    id: "t-gantry",
    key: "machine-t-gantry",
    default_config: include_str!("../default.toml"),
    i18n: &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))],
    build,
    suite_configs,
};

const G: f64 = 9.80665;

/// A real machine as a config for `--config`: a Festo EXCT-30 with Siemens 1FK2 motors on SINAMICS S210.
pub const EXCT_30_CONFIG: &str = include_str!("../exct-30.toml");

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
    Ok(Box::new(TGantry::new(cfg)?))
}

/// The default config with every loss switched off.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    cfg.set_text("gearbox.damping_nm_s_rad = 0.0").unwrap();
    cfg.set_text("frame.damping_n_s_m = 0.0").unwrap();
    cfg.set_text("tool_flex.damping_n_s_m = 0.0").unwrap();
    cfg.set_text("idler.friction_nm = 0.0").unwrap();
    cfg.set_text("cable_chain.drag_n = 0.0").unwrap();
    cfg.set_text("hard_stop.damping_n_s_m = 0.0").unwrap();
    cfg
}

/// Loss-free rigid reference, loss-free full model, and the full model with the loss-free parts of the
/// belt details and loads on top.
fn suite_configs() -> Vec<Config> {
    let mut rigid = lossless_config();
    rigid.set_text("effects.belt_stretch = false").unwrap();
    let mut loaded = lossless_config();
    for effect in ["belt_tooth_stiffness", "idlers", "cable_chain", "part_heating"] {
        loaded.set_text(&format!("effects.{effect} = true")).unwrap();
    }
    vec![rigid, lossless_config(), loaded]
}

pub struct TGantry {
    elastic: bool,
    r: f64,
    /// Unstretched belt fed per radian of pulley. The pulley radius holds at pretension, where the
    /// belt is already stretched, so that the kinematics are exact for an evenly tensioned belt.
    r_mat: f64,
    span_min: f64,
    z_com_drop: f64,
    z_tool_drop: f64,
    guide_spacing: f64,
    m_carriage: f64,
    m_z_axis: f64,
    m_payload: f64,
    j_rotor: f64,
    /// Pulley inertia including its share of the belt mass.
    j_pulley: f64,
    g: f64,
    belt: Belt,
    /// Unstretched belt per section at x = z = 0 with the pulleys at angle 0.
    mat0: [f64; 3],
    friction_x: Lugre,
    friction_z: Lugre,
    shaft: ShaftFriction,
    ripple: Ripple,
    mesh: ToothMesh,
    gearbox: Gearbox,
    frame: Option<Frame>,
    flex: Option<ToolFlex>,
    /// Idler inertia as mass on [X, Z]: with m = J / r^2 per idler and the belt speeds of the module
    /// comment, 1/2 m (2 vz^2 + (2 vx + vz)^2 + (2 vx - vz)^2 + (2 vx)^2) = 1/2 (12 m) vx^2 + 1/2 (4 m) vz^2.
    m_idlers: [f64; 2],
    /// Belt force that one idler's bearing friction takes, and the belt speed below which it fades out.
    idler_drag: f64,
    idler_smoothing: f64,
    chain: CableChain,
    /// Force on the tool in [X, Z] while the cycle dwells.
    process: [f64; 2],
    dwelling: bool,
    stops: HardStop,
    /// Holding brake of one motor shaft.
    brake: HoldingBrake,
    brake_engaged: bool,
    payload_attached: bool,
    axes: [AxisInfo; 2],
    motors: [MotorInfo; 2],
}

struct Frame {
    mass: f64,
    stiffness: f64,
    damping: f64,
}

/// The Z axis leaning against the carriage: its centre of mass moves sideways by u against a spring.
/// The guide gives way sideways (lateral) and in tilt; a force at the distance h from the carriage
/// tilts with the arm h, so the compliance at the centre of mass is 1 / lateral + h^2 / tilt.
struct ToolFlex {
    lateral: f64,
    tilt: f64,
    damping: f64,
}

// Elastic state: positions [x, z, a, b, pa, pb, q, u], their rates, the friction states [sx, sz], the
// seconds a belt section has spent without tension, and the friction states of the two brakes.
// a, b: pulley angles. pa, pb: motor shaft angles. q: frame deflection in X; x is the carriage relative to
// the frame. u: the Z axis's centre of mass in X relative to the carriage.
const X: usize = 0;
const Z: usize = 1;
const A: usize = 2;
const PA: usize = 4;
const Q: usize = 6;
const U: usize = 7;
const V: usize = 8;
const S: usize = 16;
const SLACK: usize = 18;
const BRAKE: usize = 19;
const N_ELASTIC: usize = 21;
// Rigid state: [x, z, vx, vz, sx, sz], and the deflections of the two brakes.
const RIGID_BRAKE: usize = 6;
const N_RIGID: usize = 8;

impl TGantry {
    pub fn new(cfg: &Config) -> Result<Self, ConfigError> {
        let axis = |id: &'static str, key: &'static str, stroke: f64| -> Result<AxisInfo, ConfigError> {
            Ok(AxisInfo {
                id,
                key,
                unit: "m",
                min: 0.0,
                max: stroke,
                max_vel: cfg.positive(&format!("axis.{id}.max_velocity_m_s"))?,
                max_acc: cfg.positive(&format!("axis.{id}.max_acceleration_m_s2"))?,
                max_jerk: cfg.positive(&format!("axis.{id}.max_jerk_m_s3"))?,
            })
        };
        let belt = Belt::from_config(cfg)?;
        let r = cfg.positive("geometry.pulley_radius_m")?;
        let frame = if cfg.bool("effects.frame_compliance")? {
            Some(Frame {
                mass: cfg.positive("frame.mass_kg")?,
                stiffness: cfg.positive("frame.stiffness_n_m")?,
                damping: cfg.non_negative("frame.damping_n_s_m")?,
            })
        } else {
            None
        };
        let flex = if cfg.bool("effects.tool_flex")? {
            Some(ToolFlex {
                lateral: cfg.positive("tool_flex.lateral_stiffness_n_m")?,
                tilt: cfg.positive("tool_flex.tilt_stiffness_nm_rad")?,
                damping: cfg.non_negative("tool_flex.damping_n_s_m")?,
            })
        } else {
            None
        };
        let on = |effect: &str, value: f64| -> Result<f64, ConfigError> { Ok(if cfg.bool(&format!("effects.{effect}"))? { value } else { 0.0 }) };
        let idler_radius = cfg.positive("idler.radius_m")?;
        let shaft = ShaftFriction::from_config(cfg)?;
        // The gearbox's own inertia turns with the rotor.
        let j_rotor = cfg.positive("motor.rotor_inertia_kgm2")? + cfg.non_negative("gearbox.inertia_at_input_kgm2")?;
        let brake = HoldingBrake::from_config(cfg, j_rotor)?;
        let idler_mass = on("idlers", cfg.non_negative("idler.inertia_kgm2")? / (idler_radius * idler_radius))?;
        let mut m = Self {
            elastic: cfg.bool("effects.belt_stretch")?,
            r,
            r_mat: belt.rest_length(r),
            span_min: cfg.positive("geometry.span_min_m")?,
            z_com_drop: cfg.f64("geometry.z_com_drop_m")?,
            z_tool_drop: cfg.f64("geometry.z_tool_drop_m")?,
            guide_spacing: cfg.positive("geometry.guide_spacing_m")?,
            m_carriage: cfg.positive("mass.carriage_kg")?,
            m_z_axis: cfg.positive("mass.z_axis_kg")?,
            m_payload: if cfg.bool("effects.payload")? { cfg.non_negative("mass.payload_kg")? } else { 0.0 },
            j_rotor,
            j_pulley: cfg.positive("pulley.inertia_kgm2")?,
            g: if cfg.bool("effects.gravity")? { G } else { 0.0 },
            mat0: [0.0; 3],
            friction_x: Lugre::from_config(cfg, "friction_x")?,
            friction_z: Lugre::from_config(cfg, "friction_z")?,
            m_idlers: [12.0 * idler_mass, 4.0 * idler_mass],
            idler_drag: on("idlers", cfg.non_negative("idler.friction_nm")? / idler_radius)?,
            idler_smoothing: shaft.smoothing * idler_radius,
            chain: CableChain::from_config(cfg)?,
            process: [on("process_force", cfg.f64("process.force_x_n")?)?, on("process_force", cfg.f64("process.force_z_n")?)?],
            dwelling: false,
            stops: HardStop::from_config(cfg)?,
            brake,
            brake_engaged: false,
            shaft,
            ripple: Ripple::from_config(cfg)?,
            mesh: ToothMesh::from_config(cfg)?,
            gearbox: Gearbox::from_config(cfg)?,
            frame,
            flex,
            payload_attached: true,
            axes: [
                axis("x", "axis-x", cfg.positive("geometry.stroke_x_m")?)?,
                axis("z", "axis-z", cfg.positive("geometry.stroke_z_m")?)?,
            ],
            motors: [MotorInfo { id: "a", key: "motor-a" }, MotorInfo { id: "b", key: "motor-b" }],
            belt,
        };
        let geo = m.geo(0.0, 0.0);
        m.mat0 = geo.map(|g| m.belt.rest_length(g));
        // Belt mass: the belt runs at pulley surface speed, so half of it is lumped on each pulley.
        m.j_pulley += 0.5 * m.belt.mass_per_m * (geo.iter().sum::<f64>() - 4.0 * m.belt.tooth_length) * r * r;
        Ok(m)
    }

    fn payload(&self) -> f64 {
        if self.payload_attached { self.m_payload } else { 0.0 }
    }
    /// Everything that moves in Z: the vertical axis and what it carries.
    fn m_z(&self) -> f64 {
        self.m_z_axis + self.payload()
    }
    /// Everything that moves in X: the carriage and the whole Z axis on it.
    fn m_x(&self) -> f64 {
        self.m_carriage + self.m_z()
    }
    /// Inertia per motor at the pulley: pulley plus rotor seen through the gearbox.
    fn j(&self) -> f64 {
        self.j_pulley + self.gearbox.ratio * self.gearbox.ratio * self.j_rotor
    }
    /// Both motors reflected onto one tool axis: 2 J / r^2.
    fn m_motors(&self) -> f64 {
        2.0 * self.j() / (self.r * self.r)
    }

    /// The spring that holds the Z axis upright at height z: (stiffness k at the centre of mass, dk/dz,
    /// tool-tip travel per u). All zero with the effect off.
    fn flex_at(&self, z: f64) -> (f64, f64, f64) {
        let Some(f) = &self.flex else { return (0.0, 0.0, 0.0) };
        let (h, h_tool) = (self.z_com_drop - z, self.z_tool_drop - z);
        let k = 1.0 / (1.0 / f.lateral + h * h / f.tilt);
        // The force k u shifts the guide by k u / lateral and tilts it by k u h / tilt.
        (k, 2.0 * k * k * h / f.tilt, k * (1.0 / f.lateral + h * h_tool / f.tilt))
    }

    /// Bearing friction of the idlers as forces opposing (vx, vz): each idler holds back the belt that
    /// passes it (see the module comment for the speeds).
    fn idler_friction(&self, vx: f64, vz: f64) -> (f64, f64) {
        let sign = |v: f64| (v / self.idler_smoothing).tanh();
        let (left, right) = (sign(2.0 * vx + vz), sign(2.0 * vx - vz));
        (2.0 * self.idler_drag * (left + right + sign(2.0 * vx)), self.idler_drag * (2.0 * sign(vz) + left - right))
    }

    /// Chain drag opposing vx; it fades out at rest like the guide friction's Stribeck speed.
    fn chain_drag(&self, vx: f64) -> f64 {
        self.chain.drag(vx, self.friction_x.stribeck_v)
    }

    fn process_force(&self) -> [f64; 2] {
        if self.dwelling { self.process } else { [0.0; 2] }
    }

    /// Force of the end stops on an axis.
    fn stop_force(&self, axis: usize, pos: f64, vel: f64) -> f64 {
        self.stops.force(pos, vel, self.axes[axis].max)
    }
    fn stop_energy(&self, x: f64, z: f64) -> f64 {
        self.stops.energy(x, self.axes[0].max) + self.stops.energy(z, self.axes[1].max)
    }

    /// Geometric length of the three belt sections. Sections 1 and 3 end at one motor pulley, section 2
    /// at both; the teeth in mesh there count as more belt (see `Belt::tooth_length`).
    fn geo(&self, x: f64, z: f64) -> [f64; 3] {
        let (sx, sz, s, teeth) = (self.axes[0].max, self.axes[1].max, self.span_min, self.belt.tooth_length);
        [2.0 * s + x + z + teeth, 4.0 * s + sx + 2.0 * sz - 2.0 * z + 2.0 * teeth, 2.0 * s + sx - x + z + teeth]
    }

    /// Unstretched belt in the three sections for pulley angles a and b.
    fn mat(&self, a: f64, b: f64) -> [f64; 3] {
        let r = self.r_mat;
        [self.mat0[0] + r * a, self.mat0[1] - r * a + r * b, self.mat0[2] - r * b]
    }

    fn sections(&self, s: &[f64]) -> [SectionForce; 3] {
        let (geo, mat, r) = (self.geo(s[X], s[Z]), self.mat(s[A], s[A + 1]), self.r_mat);
        let (vx, vz, va, vb) = (s[V + X], s[V + Z], s[V + A], s[V + A + 1]);
        let geo_rate = [vx + vz, -2.0 * vz, vz - vx];
        let mat_rate = [r * va, r * (vb - va), -r * vb];
        std::array::from_fn(|i| self.belt.section(geo[i], mat[i], geo_rate[i], mat_rate[i]))
    }

    /// Loads on the X and the Z guide at an X acceleration of the Z axis.
    ///
    /// The X guide carries the weight and the tilting moment of the overhanging Z axis; the Z guide
    /// carries the side force that drags the Z axis along and the same moment.
    fn guide_loads(&self, z: f64, ax: f64) -> (f64, f64) {
        let side = self.m_z() * ax;
        let tilt = 2.0 * side * (self.z_com_drop - z) / self.guide_spacing;
        (self.m_x() * self.g + tilt.abs(), side.abs() + tilt.abs())
    }

    /// Friction level factors for the X and Z guides at an X acceleration of the Z axis.
    fn load_scales(&self, z: f64, ax: f64) -> (f64, f64) {
        let (load_x, load_z) = self.guide_loads(z, ax);
        (self.friction_x.load_scale(load_x, (self.m_carriage + self.m_z_axis) * self.g), self.friction_z.load_scale(load_z, 0.0))
    }

    /// The X acceleration that the motor torques would give a loss-free rigid machine: what the guide
    /// loads are worked out with.
    fn commanded_ax(&self, torque: &[f64]) -> f64 {
        self.gearbox.ratio * (torque[0] + torque[1]) / self.r / (self.m_x() + self.m_motors())
    }

    /// Guide friction forces and friction state rates: (fx, fz, dsx, dsz).
    ///
    /// The guide load uses the X acceleration that the motor torques would give a loss-free rigid
    /// machine. The true acceleration would make friction depend on itself, and its absolute value
    /// would turn every belt vibration into extra friction.
    fn guide_friction(&self, z: f64, vx: f64, vz: f64, sx: f64, sz: f64, torque: &[f64]) -> (f64, f64, f64, f64) {
        let (scale_x, scale_z) = self.load_scales(z, self.commanded_ax(torque));
        let (fx, dsx) = self.friction_x.eval(vx, sx, scale_x);
        let (fz, dsz) = self.friction_z.eval(vz, sz, scale_z);
        (fx, fz, dsx, dsz)
    }

    fn elastic_derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let r = self.r;
        let sec = self.sections(s);
        let fx_belt = sec[2].tension - sec[0].tension;
        let fz_belt = 2.0 * sec[1].tension - sec[0].tension - sec[2].tension;
        let (ffx, ffz, dsx, dsz) = self.guide_friction(s[Z], s[V + X], s[V + Z], s[S], s[S + 1], torque);
        out[..V].copy_from_slice(&s[V..S]);
        let (vx, vz) = (s[V + X], s[V + Z]);
        let (idler_x, idler_z) = self.idler_friction(vx, vz);
        let [process_x, process_z] = self.process_force();
        // Frame mode: belt, guide, idlers, chain and end stop push back on the beam with the force they
        // put on the carriage.
        let fx = fx_belt - ffx - idler_x - self.chain_drag(vx) + self.stop_force(0, s[X], vx);
        // shortcut: idler and chain inertia ride on the carriage's speed over ground w, not on its speed along
        // the beam; the two differ by the frame's own motion, which is a thousand times smaller.
        // The chain's mass m(x) grows as the carriage moves: d/dt (m w) - m' w^2 / 2 = force, and the beam
        // feels d/dt (m w).
        let (chain_mass, chain_growth) = self.chain.mass(s[X]);
        let w = vx + s[V + Q];
        let chain_x = chain_growth * w * (0.5 * w - vx);
        let m_carriage = self.m_x() - self.m_z() + self.m_idlers[0] + chain_mass;
        let aq = match &self.frame {
            Some(f) => (-f.stiffness * s[Q] - f.damping * s[V + Q] - fx - 0.5 * chain_growth * w * w) / f.mass,
            None => {
                out[Q] = 0.0;
                0.0
            }
        };
        let mut fz_flex = 0.0;
        match &self.flex {
            Some(f) => {
                let (k, dk_dz, tip) = self.flex_at(s[Z]);
                let spring = k * s[U] + f.damping * s[V + U];
                // The process force acts at the tool tip, which moves `tip` times as far as u: that share
                // of it bends the Z axis, the rest pushes the carriage.
                let ax = (fx + chain_x + (1.0 - tip) * process_x + spring) / m_carriage - aq;
                out[V + X] = ax;
                // The Z axis rides on the carriage and the frame: its own acceleration is ax + aq plus that of u.
                out[V + U] = (tip * process_x - spring) / self.m_z() - (ax + aq);
                // The spring gets stiffer as the Z axis rises, which pushes back on Z: -dV/dz of k u^2 / 2.
                fz_flex = -0.5 * s[U] * s[U] * dk_dz;
            }
            None => {
                out[V + X] = (fx + chain_x + process_x) / (m_carriage + self.m_z()) - aq;
                out[U] = 0.0;
                out[V + U] = 0.0;
            }
        }
        // The idlers add inertia in Z but no weight.
        let m_z = self.m_z() + self.m_idlers[1];
        out[V + Z] = (fz_belt - ffz + fz_flex - idler_z + process_z + self.stop_force(1, s[Z], vz)) / m_z - self.g * (self.m_z() / m_z);
        out[V + Q] = aq;
        let belt_torque =
            [self.r_mat * (sec[0].material - sec[1].material), self.r_mat * (sec[1].material - sec[2].material)];
        for k in 0..2 {
            let (theta, w_theta, phi, w_phi) = (s[A + k], s[V + A + k], s[PA + k], s[V + PA + k]);
            let gear = self.gearbox.torque(phi, w_phi, theta, w_theta);
            let (brake, brake_rate) = self.brake.torque(self.brake_engaged, w_phi, s[BRAKE + k]);
            out[V + A + k] = (gear.output + belt_torque[k] + r * self.mesh.force(r * theta)) / self.j_pulley;
            out[V + PA + k] =
                (self.ripple.apply(torque[k], phi) - gear.rotor - self.shaft.torque(w_phi) - brake) / self.j_rotor;
            out[BRAKE + k] = brake_rate;
        }
        out[S] = dsx;
        out[S + 1] = dsz;
        out[SLACK] = if sec.iter().any(|f| f.tension <= 0.0) { 1.0 } else { 0.0 };
    }

    /// Rigid model: accelerations, guide friction forces, friction state rates, the torques at the two
    /// gearbox outputs and the torques that go into them before their losses,
    /// [ax, az, ffx, ffz, dsx, dsz, gear a, gear b, into gear a, into gear b].
    ///
    /// The gearbox loses torque only on what it transmits, which is the motor torque minus what
    /// accelerates the rotor itself. That depends on the acceleration being solved for; three passes
    /// settle it.
    fn rigid_solve(&self, s: &[f64], torque: &[f64]) -> [f64; 10] {
        let (r, ratio) = (self.r, self.gearbox.ratio);
        let (mut pos, mut vel) = ([0.0; 2], [0.0; 2]);
        self.inverse(&s[0..2], &mut pos);
        self.inverse(&s[2..4], &mut vel);
        // Belt force each motor would give with a loss-free gearbox and a massless rotor; a brake holds the rotor.
        let brake: [f64; 2] = std::array::from_fn(|k| self.brake.torque(self.brake_engaged, vel[k], s[RIGID_BRAKE + k]).0);
        let drive: [f64; 2] =
            std::array::from_fn(|k| ratio * (self.ripple.apply(torque[k], pos[k]) - self.shaft.torque(vel[k]) - brake[k]) / r);
        let mesh: [f64; 2] = std::array::from_fn(|k| self.mesh.force(r * pos[k] / ratio));
        let rotor = ratio * ratio * self.j_rotor / (r * r);
        let pulleys = 2.0 * self.j_pulley / (r * r);
        let (chain_mass, chain_growth) = self.chain.mass(s[0]);
        let (mx, mz) = (self.m_x() + pulleys + self.m_idlers[0] + chain_mass, self.m_z() + pulleys + self.m_idlers[1]);
        let (ffx, ffz, dsx, dsz) = self.guide_friction(s[1], s[2], s[3], s[4], s[5], torque);
        // Idlers, chain, process and end stops; the growing chain takes m' vx^2 / 2 (see the elastic model).
        let (idler_x, idler_z) = self.idler_friction(s[2], s[3]);
        let [process_x, process_z] = self.process_force();
        let extra_x = process_x + self.stop_force(0, s[0], s[2]) - idler_x - self.chain_drag(s[2]) - 0.5 * chain_growth * s[2] * s[2];
        let extra_z = process_z + self.stop_force(1, s[1], s[3]) - idler_z;
        let (mut loss, mut out) = ([1.0; 2], [0.0; 10]);
        for _ in 0..3 {
            let bx = loss[0] * drive[0] + loss[1] * drive[1] + mesh[0] + mesh[1] - ffx + extra_x;
            let bz = loss[0] * drive[0] - loss[1] * drive[1] + mesh[0] - mesh[1] - ffz - self.m_z() * self.g + extra_z;
            // (mx + p) ax + q az = bx,  q ax + (mz + p) az = bz
            let (p, q) = (rotor * (loss[0] + loss[1]), rotor * (loss[0] - loss[1]));
            let det = (mx + p) * (mz + p) - q * q;
            let (ax, az) = ((bx * (mz + p) - q * bz) / det, ((mx + p) * bz - q * bx) / det);
            let through_gear = [drive[0] - rotor * (ax + az), drive[1] - rotor * (ax - az)];
            // The gearbox outputs carry what the rotors' own acceleration and the losses of this pass leave.
            out = [ax, az, ffx, ffz, dsx, dsz, loss[0] * through_gear[0] * r, loss[1] * through_gear[1] * r, through_gear[0] * r, through_gear[1] * r];
            loss = std::array::from_fn(|k| self.gearbox.loss_factor(through_gear[k] * r, vel[k] / ratio));
        }
        out
    }

    fn rigid_derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let [ax, az, _, _, dsx, dsz, ..] = self.rigid_solve(s, torque);
        let (mut vel, mut brake) = ([0.0; 2], [0.0; 2]);
        self.inverse(&s[2..4], &mut vel);
        for k in 0..2 {
            brake[k] = self.brake.torque(self.brake_engaged, vel[k], s[RIGID_BRAKE + k]).1;
        }
        out.copy_from_slice(&[s[2], s[3], ax, az, dsx, dsz, brake[0], brake[1]]);
    }

    /// Kinetic, potential and elastic energy (see `ENERGY_PARTS`).
    fn energy_parts(&self, s: &[f64]) -> [f64; 3] {
        let potential = self.m_z() * self.g * s[Z];
        if !self.elastic {
            let kinetic = 0.5 * (self.m_x() + self.m_motors() + self.m_idlers[0] + self.chain.mass(s[0]).0) * s[2] * s[2]
                + 0.5 * (self.m_z() + self.m_motors() + self.m_idlers[1]) * s[3] * s[3];
            return [kinetic, potential, self.stop_energy(s[0], s[1])];
        }
        let (geo, mat) = (self.geo(s[X], s[Z]), self.mat(s[A], s[A + 1]));
        let belt: f64 = (0..3).map(|i| self.belt.energy(geo[i], mat[i])).sum();
        let gears: f64 = (0..2).map(|k| self.gearbox.energy(s[PA + k], s[A + k])).sum();
        let turning: f64 = (0..2).map(|k| 0.5 * self.j_pulley * s[V + A + k].powi(2) + 0.5 * self.j_rotor * s[V + PA + k].powi(2)).sum();
        let frame = self.frame.as_ref().map_or([0.0; 2], |f| [0.5 * f.mass * s[V + Q].powi(2), 0.5 * f.stiffness * s[Q].powi(2)]);
        let v_carriage = s[V + X] + s[V + Q];
        let kinetic = 0.5 * (self.m_x() - self.m_z() + self.m_idlers[0] + self.chain.mass(s[X]).0) * v_carriage.powi(2)
            + 0.5 * self.m_z() * (v_carriage + s[V + U]).powi(2)
            + 0.5 * (self.m_z() + self.m_idlers[1]) * s[V + Z].powi(2)
            + turning
            + frame[0];
        let elastic = 0.5 * self.flex_at(s[Z]).0 * s[U] * s[U] + self.stop_energy(s[X], s[Z]) + belt + gears + frame[1];
        [kinetic, potential, elastic]
    }

    /// The loss powers in the order of `effect_infos`, and per motor the gearbox loss as a torque at the rotor and the
    /// brake torque, which the torque breakdown takes too. No derivatives: cheap enough for every step.
    fn losses(&self, s: &[f64], torque: &[f64]) -> ([f64; 13], [f64; 2], [f64; 2]) {
        let (r, ratio) = (self.r, self.gearbox.ratio);
        let v = if self.elastic { V } else { 2 };
        let (z, vx, vz) = (s[Z], s[v + X], s[v + Z]);
        let (sx, sz) = if self.elastic { (s[S], s[S + 1]) } else { (s[4], s[5]) };
        let (ffx, ffz, ..) = self.guide_friction(z, vx, vz, sx, sz, torque);
        let (idler_x, idler_z) = self.idler_friction(vx, vz);
        let drag = self.chain_drag(vx);
        let [process_x, process_z] = self.process_force();
        let (mut angle, mut speed) = ([0.0; 2], [0.0; 2]);
        self.motor_state(s, &mut angle, &mut speed);
        // Per motor: gearbox loss as a torque at the rotor and as a power, brake torque, and the power the tooth
        // meshing puts in; the damping losses.
        let (mut gear_loss, mut gear_power, mut brake, mut mesh_in) = ([0.0; 2], [0.0; 2], [0.0; 2], 0.0);
        let (process_in, damping) = if self.elastic {
            for k in 0..2 {
                let (theta, w_theta, phi, w_phi) = (s[A + k], s[V + A + k], s[PA + k], s[V + PA + k]);
                let gear = self.gearbox.torque(phi, w_phi, theta, w_theta);
                gear_loss[k] = gear.rotor - gear.output / ratio;
                gear_power[k] = self.gearbox.loss_power(phi, w_phi, theta, w_theta);
                brake[k] = self.brake.torque(self.brake_engaged, w_phi, s[BRAKE + k]).0;
                mesh_in += r * self.mesh.force(r * theta) * w_theta;
            }
            // The rates of `sections`.
            let (rm, (geo, mat)) = (self.r_mat, (self.geo(s[X], z), self.mat(s[A], s[A + 1])));
            let (geo_rate, mat_rate) = ([vx + vz, -2.0 * vz, vz - vx], [rm * s[V + A], rm * (s[V + A + 1] - s[V + A]), -rm * s[V + A + 1]]);
            let belt: f64 = (0..3).map(|i| self.belt.loss_power(geo[i], mat[i], geo_rate[i], mat_rate[i])).sum();
            let frame = self.frame.as_ref().map_or(0.0, |f| f.damping * s[V + Q].powi(2));
            let flex = self.flex.as_ref().map_or(0.0, |f| f.damping * s[V + U].powi(2));
            let tool_vx = vx + s[V + Q] + self.flex_at(z).2 * s[V + U];
            (process_x * tool_vx + process_z * vz, [belt, frame, flex])
        } else {
            let solved = self.rigid_solve(s, torque);
            for k in 0..2 {
                let sign = [1.0, -1.0][k];
                brake[k] = self.brake.torque(self.brake_engaged, speed[k], s[RIGID_BRAKE + k]).0;
                gear_loss[k] = (solved[8 + k] - solved[6 + k]) / ratio;
                gear_power[k] = gear_loss[k] * speed[k];
                mesh_in += self.mesh.force(r * angle[k] / ratio) * (vx + sign * vz);
            }
            (process_x * vx + process_z * vz, [0.0; 3])
        };
        let shaft: [f64; 2] = std::array::from_fn(|k| (self.shaft.torque(speed[k]) + brake[k]) * speed[k]);
        let ripple: f64 = (0..2).map(|k| (torque[k] - self.ripple.apply(torque[k], angle[k])) * speed[k]).sum();
        let stops = self.stops.loss_power(s[X], vx, self.axes[0].max) + self.stops.loss_power(z, vz, self.axes[1].max);
        let [belt, frame, flex] = damping;
        let powers = [ffx * vx, ffz * vz, idler_x * vx + idler_z * vz, drag * vx, shaft[0], shaft[1], gear_power[0], gear_power[1], belt, frame, flex, stops, ripple - mesh_in - process_in];
        (powers, gear_loss, brake)
    }

    /// The signals of the effect views, in the order of `effect_infos`: per motor the parts of its torque, the
    /// loss powers, the energy parts and per guide its friction split.
    ///
    /// Torque parts. Per motor k = A, B with s_k = +1, -1 and the rigid model's transpose k_r = r / (2 i):
    /// inertia = k_r (I_x + s_k I_z), with I what accelerates every moved mass at its own acceleration (carriage,
    /// leaning Z axis, idlers, chain with what picking it up costs) and both rotors and pulleys at the acceleration
    /// of the carriage along the beam and of the Z axis, as a rigid machine would turn them; gravity = s_k k_r m_z g;
    /// friction = shaft friction + brake + gearbox loss at the rotor + k_r (F_x + s_k F_z), F the guide, idler and
    /// chain drag forces; rest = motor torque less these. So rotors and pulleys ringing against the load on the belt
    /// and in the gearbox backlash are in the rest.
    fn view_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let kr = self.r / (2.0 * self.gearbox.ratio);
        let v = if self.elastic { V } else { 2 };
        let (z, vx, vz) = (s[Z], s[v + X], s[v + Z]);
        let (sx, sz) = if self.elastic { (s[S], s[S + 1]) } else { (s[4], s[5]) };
        let (scales, (ffx, ffz, ..)) = (self.load_scales(z, self.commanded_ax(torque)), self.guide_friction(z, vx, vz, sx, sz, torque));
        let (idler_x, idler_z) = self.idler_friction(vx, vz);
        let drag = self.chain_drag(vx);
        let (chain_mass, chain_growth) = self.chain.mass(s[X]);
        let (mut angle, mut speed) = ([0.0; 2], [0.0; 2]);
        self.motor_state(s, &mut angle, &mut speed);
        let (powers, gear_loss, brake) = self.losses(s, torque);
        // The inertia forces in X and Z.
        let (motors, idlers) = (self.m_motors(), self.m_idlers);
        let inertia = if self.elastic {
            let mut d = [0.0; N_ELASTIC];
            self.elastic_derivatives(s, torque, &mut d);
            let (ax, az, aq, au) = (d[V + X], d[V + Z], d[V + Q], d[V + U]);
            let w = vx + s[V + Q];
            let m_carriage = self.m_x() - self.m_z() + idlers[0] + chain_mass;
            let inertia_x = m_carriage * (ax + aq) + self.m_z() * (ax + aq + au) + motors * ax + chain_growth * w * (vx - 0.5 * w);
            [inertia_x, (self.m_z() + idlers[1] + motors) * az]
        } else {
            let solved = self.rigid_solve(s, torque);
            let inertia_x = (self.m_x() + motors + idlers[0] + chain_mass) * solved[0] + 0.5 * chain_growth * vx * vx;
            [inertia_x, (self.m_z() + motors + idlers[1]) * solved[1]]
        };
        let friction = [ffx + idler_x + drag, ffz + idler_z];
        for k in 0..2 {
            let sign = [1.0, -1.0][k];
            let parts = [
                kr * (inertia[0] + sign * inertia[1]),
                sign * kr * self.m_z() * self.g,
                self.shaft.torque(speed[k]) + brake[k] + gear_loss[k] + kr * (friction[0] + sign * friction[1]),
            ];
            out.extend(parts);
            out.push(torque[k] - parts.iter().sum::<f64>());
        }
        out.extend(powers);
        out.extend(self.energy_parts(s));
        for (friction, v, state, scale) in [(&self.friction_x, vx, sx, scales.0), (&self.friction_z, vz, sz, scales.1)] {
            out.push(v);
            out.extend(friction.split(v, state, scale));
            out.push(scale);
        }
        // Stretch per belt section: its length less its unstretched belt; in the rigid model that of pretension.
        let geo = self.geo(s[X], z);
        let mat = if self.elastic { self.mat(s[A], s[A + 1]) } else { geo.map(|g| self.belt.rest_length(g)) };
        out.extend((0..3).map(|i| geo[i] - mat[i]));
    }

    /// The signal list of `effect_signals`.
    fn effect_infos(&self, out: &mut Vec<SignalInfo>) {
        for motor in &self.motors {
            out.extend(TORQUE_PARTS.map(|part| SignalInfo::new(format!("motor.{}.torque_{part}", motor.id), motor.key, &format!("quantity-torque-{part}"), "N m")));
        }
        let friction = "quantity-friction-power";
        let mut powers = vec![("guide_x", "axis-x", friction), ("guide_z", "axis-z", friction), ("idlers", "group-idler", friction), ("cable_chain", "group-cable_chain", friction)];
        powers.extend([("shaft_a", "motor-a", "quantity-shaft-friction-power"), ("shaft_b", "motor-b", "quantity-shaft-friction-power")]);
        let gear = if self.gearbox.is_coupling() { "quantity-coupling-loss-power" } else { "quantity-gearbox-loss-power" };
        powers.extend([("gearbox_a", "motor-a", gear), ("gearbox_b", "motor-b", gear)]);
        let damping = "quantity-damping-power";
        powers.extend([("belt", "group-belt", damping), ("frame", "frame", damping), ("tool_flex", "group-tool_flex", damping), ("hard_stops", "group-hard_stop", damping)]);
        powers.push(("other", "loss-other", "quantity-power-given-off"));
        out.extend(powers.into_iter().map(|(id, owner, quantity)| SignalInfo::new(format!("{LOSS_PREFIX}{id}"), owner, quantity, "W")));
        out.extend(ENERGY_PARTS.map(|part| SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")));
        for contact in self.friction_contacts() {
            for part in FRICTION_SIGNALS {
                let unit = match part {
                    "speed" => "m/s",
                    "load_scale" => "",
                    _ => "N",
                };
                out.push(SignalInfo::new(format!("{}.friction_{part}", contact.id), contact.owner_key, &format!("quantity-friction-{}", part.replace('_', "-")), unit));
            }
        }
        out.extend(self.belt_sections().iter().map(|b| SignalInfo::new(format!("{}.stretch", b.id), b.owner_key, "quantity-section-stretch", "m")));
    }
}

impl Machine for TGantry {
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        let k = self.gearbox.ratio / self.r;
        motors[0] = k * (tool[0] + tool[1]);
        motors[1] = k * (tool[0] - tool[1]);
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        let k = self.r / (2.0 * self.gearbox.ratio);
        tool[0] = k * (motors[0] + motors[1]);
        tool[1] = k * (motors[0] - motors[1]);
    }

    fn state_len(&self) -> usize {
        if self.elastic { N_ELASTIC } else { N_RIGID }
    }

    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let (x, z, r, ratio) = (tool[0], tool[1], self.r, self.gearbox.ratio);
        if !self.elastic {
            *state = vec![x, z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
            let mut angle = [0.0; 2];
            self.inverse(tool, &mut angle);
            let lift = self.m_z() * self.g * r / 2.0;
            for (k, at_pulley) in [lift, -lift].into_iter().enumerate() {
                let shaft = (at_pulley - r * self.mesh.force(r * angle[k] / ratio)) / ratio;
                torque[k] = self.ripple.invert(shaft, angle[k]);
            }
            return;
        }
        // X balance: T1 = T3. Z balance: 2 T2 - T1 - T3 = m_z g. The belt total fixes the level.
        let geo = self.geo(x, z);
        let mat = self.belt.static_material(&geo, &[0.0, self.m_z() * self.g / 2.0, 0.0], self.mat0.iter().sum());
        let (a, b) = ((mat[0] - self.mat0[0]) / self.r_mat, (self.mat0[2] - mat[2]) / self.r_mat);
        let m: [f64; 3] = std::array::from_fn(|i| self.belt.section(geo[i], mat[i], 0.0, 0.0).material);
        *state = vec![0.0; N_ELASTIC];
        state[X] = x;
        state[Z] = z;
        for (k, (theta, belt_torque)) in [(a, self.r_mat * (m[0] - m[1])), (b, self.r_mat * (m[1] - m[2]))].into_iter().enumerate() {
            let gear = -belt_torque - r * self.mesh.force(r * theta);
            let phi = ratio * (theta + self.gearbox.static_twist(gear));
            state[A + k] = theta;
            state[PA + k] = phi;
            torque[k] = self.ripple.invert(gear / ratio, phi);
        }
    }

    fn derivatives(&self, state: &[f64], torque: &[f64], out: &mut [f64]) {
        if self.elastic { self.elastic_derivatives(state, torque, out) } else { self.rigid_derivatives(state, torque, out) }
    }

    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let v = if self.elastic { V } else { 2 };
        pos.copy_from_slice(&s[0..2]);
        vel.copy_from_slice(&s[v..v + 2]);
        if self.elastic {
            // The tool is at the tip of the leaning Z axis, not at the carriage, and the carriage rides on the
            // frame: the workpiece stands on the floor, so the frame's sway is the tool's too.
            let tip = self.flex_at(s[Z]).2;
            pos[0] += tip * s[U] + s[Q];
            vel[0] += tip * s[V + U] + s[V + Q];
        }
    }
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        if !self.elastic {
            return s[2..4].copy_from_slice(vel);
        }
        s[V + X..V + X + 2].copy_from_slice(vel);
        let mut rotors = [0.0; 2];
        self.inverse(vel, &mut rotors);
        for k in 0..2 {
            (s[V + A + k], s[V + PA + k]) = (rotors[k] / self.gearbox.ratio, rotors[k]);
        }
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        if self.elastic {
            pos.copy_from_slice(&s[PA..PA + 2]);
            vel.copy_from_slice(&s[V + PA..V + PA + 2]);
        } else {
            self.inverse(&s[0..2], pos);
            self.inverse(&s[2..4], vel);
        }
    }

    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        let fx = (self.m_x() + self.m_motors() + self.m_idlers[0] + self.chain.mass(pos[0]).0) * acc[0];
        let fz = (self.m_z() + self.m_motors() + self.m_idlers[1]) * acc[1] + self.m_z() * self.g;
        let k = self.r / (2.0 * self.gearbox.ratio);
        out[0] = k * (fx + fz);
        out[1] = k * (fx - fz);
    }

    fn energy(&self, s: &[f64]) -> f64 {
        self.energy_parts(s).iter().sum()
    }

    /// The Z axis leaning on its guide while the belt holds the carriage: two masses in a row. The carriage
    /// sits on the belt's stiffness in X (the two sections that end at the clamp, EA / L each, with the
    /// pulleys taken as held), the Z axis with what it carries on the spring of its guide at this height.
    /// The lower of the two modes, from w^4 - (kb / mc + kz / mc + kz / mz) w^2 + kb kz / (mc mz) = 0.
    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        if !self.elastic || self.flex.is_none() {
            return None;
        }
        let geo = self.geo(tool_pose[0], tool_pose[1]);
        let (belt, lean) = (self.belt.ea / geo[0] + self.belt.ea / geo[2], self.flex_at(tool_pose[1]).0);
        let m_carriage = self.m_x() - self.m_z() + self.m_idlers[0] + self.chain.mass(tool_pose[0]).0;
        let (sum, product) = ((belt + lean) / m_carriage + lean / self.m_z(), belt * lean / (m_carriage * self.m_z()));
        Some((0.5 * (sum - (sum * sum - 4.0 * product).sqrt())).sqrt() / std::f64::consts::TAU)
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
        let mut out: Vec<SignalInfo> = (1..=3)
            .map(|i| SignalInfo::new(format!("belt.{i}.tension"), &format!("belt-section-{i}"), "quantity-tension", "N"))
            .collect();
        for axis in &self.axes {
            out.push(SignalInfo::new(format!("axis.{}.belt_stretch", axis.id), axis.key, "quantity-belt-stretch", "m"));
            out.push(SignalInfo::new(format!("axis.{}.friction", axis.id), axis.key, "quantity-friction-force", "N"));
        }
        out.push(SignalInfo::new("frame.deflection", "frame", "quantity-deflection", "m"));
        out.push(SignalInfo::new("axis.x.tool_deflection", "axis-x", "quantity-tool-deflection", "m"));
        out.push(SignalInfo::new("belt.slack_time", "group-belt", "quantity-slack-time", "s"));
        for motor in &self.motors {
            out.push(SignalInfo::new(format!("pulley.{}.radial_load", motor.id), motor.key, "quantity-radial-load", "N"));
        }
        for axis in &self.axes {
            out.push(SignalInfo::new(format!("axis.{}.guide_load", axis.id), axis.key, "quantity-guide-load", "N"));
        }
        for motor in &self.motors {
            out.push(SignalInfo::new(format!("gearbox.{}.output_torque", motor.id), motor.key, "quantity-gearbox-torque", "N m"));
        }
        self.effect_infos(&mut out);
        out
    }

    /// The two guides. Their speeds are those of the carriage along the beam and of the Z axis.
    fn friction_contacts(&self) -> Vec<FrictionContact> {
        vec![FrictionContact { id: "axis.x", owner_key: "axis-x" }, FrictionContact { id: "axis.z", owner_key: "axis-z" }]
    }
    fn friction_curve(&self, i: usize, speed: f64, load_scale: f64) -> [f64; 3] {
        [&self.friction_x, &self.friction_z].get(i).map_or([0.0; 3], |f| f.steady(speed, load_scale))
    }

    /// The three belt sections, in the order of the signals `belt.<i>.tension`.
    fn belt_sections(&self) -> Vec<BeltSection> {
        ["belt-section-1", "belt-section-2", "belt-section-3"].iter().zip(["belt.1", "belt.2", "belt.3"]).map(|(&owner_key, id)| BeltSection { id, owner_key, pretension: self.belt.pretension }).collect()
    }

    /// Both gearboxes, where there are gearboxes (not on a direct drive, see `Gearbox::is_coupling`), and both guides.
    /// The X guide is shown by the carriage, the Z guide by the Z axis's column.
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        let part = |id, kind, owner_key, scene_part, scene_index| ThermalPart { id, kind, owner_key, scene_part, scene_index };
        let gearboxes = [part("gearbox_a", "gearbox", "motor-a", "gearbox", 0), part("gearbox_b", "gearbox", "motor-b", "gearbox", 1)];
        let mut parts = if self.gearbox.is_coupling() { Vec::new() } else { gearboxes.to_vec() };
        parts.extend([part("guide_x", "guide", "axis-x", "carriage", 0), part("guide_z", "guide", "axis-z", "z-axis", 0)]);
        parts
    }

    /// The rigid model has no belt stretch: it reports pretension, zero stretch, zero deflection and no slack.
    /// The belt wraps each motor pulley by half a turn, so the two sections that end there pull its shaft
    /// the same way: the radial load is the sum of their tensions. The gearbox torques are those at the
    /// outputs, positive where they turn the pulley forwards: in the elastic model what the gear's spring
    /// passes on to the pulley, in the rigid one what its solver works out.
    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let (load_x, load_z) = self.guide_loads(s[Z], self.commanded_ax(torque));
        if !self.elastic {
            let [_, _, ffx, ffz, _, _, gear_a, gear_b, ..] = self.rigid_solve(s, torque);
            out.extend([self.belt.pretension; 3]);
            out.extend([0.0, ffx, 0.0, ffz, 0.0, 0.0, 0.0]);
            out.extend([2.0 * self.belt.pretension, 2.0 * self.belt.pretension, load_x, load_z, gear_a, gear_b]);
            return;
        }
        let sec = self.sections(s);
        let (ffx, ffz, ..) = self.guide_friction(s[Z], s[V + X], s[V + Z], s[S], s[S + 1], torque);
        let (a, b) = (s[A], s[A + 1]);
        out.extend(sec.iter().map(|f| f.tension));
        out.extend([s[X] - self.r / 2.0 * (a + b), ffx, s[Z] - self.r / 2.0 * (a - b), ffz, s[Q], self.flex_at(s[Z]).2 * s[U], s[SLACK]]);
        out.extend([sec[0].tension + sec[1].tension, sec[1].tension + sec[2].tension, load_x, load_z]);
        out.extend((0..2).map(|k| self.gearbox.torque(s[PA + k], s[V + PA + k], s[A + k], s[V + A + k]).output));
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
        let (x, z, r, ratio) = (s[0], s[1], self.r, self.gearbox.ratio);
        let (sx, sz, span) = (self.axes[0].max, self.axes[1].max, self.span_min);
        let (mut rotor, mut rotor_speed) = ([0.0; 2], [0.0; 2]);
        self.motor_state(s, &mut rotor, &mut rotor_speed);
        let pulley = if self.elastic { [s[A], s[A + 1]] } else { rotor.map(|p| p / ratio) };
        // Belt colour: tension relative to pretension, per section.
        let tension = if self.elastic { self.sections(s).map(|f| f.tension) } else { [self.belt.pretension; 3] };
        let shade = tension.map(|t| ((t - self.belt.pretension) / self.belt.pretension.max(1.0)).clamp(-1.0, 1.0) as f32);

        // The belt runs in one plane in front of the beam; the motors sit behind it at both ends.
        // The carriage rides on the beam at height 0; the Z axis hangs through it and rises with z.
        let y = -0.05;
        let (end, cx, half) = (2.0 * span + sx, span + x, 0.03);
        let (left, right, top, bottom) = (cx - half, cx + half, span + z, z - sz - span);
        let floor = -(sz + 2.0 * span + 0.2);
        let part = Part::new;
        out.labels.clear();
        out.parts.clear();
        out.parts.extend([
            part("beam", Shape::Box { size: [end - 0.14, 0.07, 0.09] }, [end / 2.0, 0.045, 0.0], [0.62, 0.64, 0.68]),
            part("post", Shape::Box { size: [0.06, 0.06, -0.045 - floor] }, [0.13, 0.045, (floor - 0.045) / 2.0], [0.45, 0.47, 0.5]),
            part("post", Shape::Box { size: [0.06, 0.06, -0.045 - floor] }, [end - 0.13, 0.045, (floor - 0.045) / 2.0], [0.45, 0.47, 0.5]),
            part("carriage", Shape::Box { size: [0.16, 0.02, 0.16] }, [cx, -0.02, 0.0], [0.2, 0.22, 0.26]),
            part("z-axis", Shape::Box { size: [0.04, 0.03, top - bottom] }, [cx, y, (top + bottom) / 2.0], [0.78, 0.8, 0.84]),
            part("clamp", Shape::Box { size: [0.09, 0.035, 0.03] }, [cx, y, top + 0.015], [0.85, 0.3, 0.2]),
            // The tool: drawn where `scene_point` says, at the carriage's place on the beam.
            part("flange", Shape::Box { size: [0.07, 0.05, 0.02] }, self.scene_point(&[x, z]).unwrap_or_default(), [0.3, 0.3, 0.33]),
        ]);
        if self.payload() > 0.0 {
            out.parts.push(part("payload", Shape::Box { size: [0.1, 0.1, 0.1] }, [cx, y, bottom - half - 0.072], [0.2, 0.7, 0.3]));
        }
        // Positive pulley angles turn clockwise seen from the front (the top of the pulley moves right).
        drive_unit(&mut out.parts, [0.0, 0.0], y, r, -pulley[0], -rotor[0], [0.9, 0.45, 0.1]);
        drive_unit(&mut out.parts, [end, 0.0], y, r, -pulley[1], -rotor[1], [0.1, 0.45, 0.9]);
        // Carriage idlers turn the belt from the beam into the Z axis; the idler at the bottom of
        // the Z axis turns it around. Each rolls with the belt that passes it (see the module comment).
        let ri = 0.012;
        for (side, lower_travel) in [(-1.0, z + 2.0 * x), (1.0, z - 2.0 * x)] {
            let ix = cx + side * (half + ri);
            idler(&mut out.parts, [ix, r + ri], y, ri, side * z / ri);
            idler(&mut out.parts, [ix, -r - ri], y, ri, side * lower_travel / ri);
        }
        idler(&mut out.parts, [cx, bottom], y, half - 0.002, -2.0 * x / half);

        // The belt path from the clamp on the left to the clamp on the right. Its length does not
        // depend on the pose, so teeth at fixed belt length travel around the pulleys by themselves.
        let (a, b, bend) = ([0.0, 0.0], [end, 0.0], [cx, bottom]);
        let path = [
            [left, top],
            [left, r],
            [0.0, r],
            on_circle(a, r, 3.0 * FRAC_PI_4),
            on_circle(a, r, 4.0 * FRAC_PI_4),
            on_circle(a, r, 5.0 * FRAC_PI_4),
            [0.0, -r],
            [left, -r],
            [left, bottom],
            on_circle(bend, half, 5.0 * FRAC_PI_4),
            on_circle(bend, half, 6.0 * FRAC_PI_4),
            on_circle(bend, half, 7.0 * FRAC_PI_4),
            [right, bottom],
            [right, -r],
            [end, -r],
            on_circle(b, r, -FRAC_PI_4),
            on_circle(b, r, 0.0),
            on_circle(b, r, FRAC_PI_4),
            [end, r],
            [right, r],
            [right, top],
        ];
        // Strips up to the middle of each motor pulley belong to the section before it.
        belt(&mut out.parts, &path, y, 0.025, 0.04, |i| shade[if i < 4 { 0 } else if i < 16 { 1 } else { 2 }]);

        let rpm = |speed: f64| speed * 60.0 / TAU;
        out.labels.extend([
            Label { pos: [0.0, y, 0.11], owner_key: self.motors[0].key, value: rpm(rotor_speed[0]), unit_key: "unit-rpm", decimals: 0 },
            Label { pos: [end, y, 0.11], owner_key: self.motors[1].key, value: rpm(rotor_speed[1]), unit_key: "unit-rpm", decimals: 0 },
            Label { pos: [cx, y, top + 0.08], owner_key: self.axes[0].key, value: x, unit_key: "unit-m", decimals: 3 },
            Label { pos: [cx + 0.12, y, bottom], owner_key: self.axes[1].key, value: z, unit_key: "unit-m", decimals: 3 },
        ]);
    }

    /// The flange under the idler at the bottom of the Z axis, in the belt plane.
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        Some([self.span_min + tool_pose[0], -0.05, tool_pose[1] - self.axes[1].max - self.span_min - 0.042])
    }
}

#[cfg(test)]
mod tests;
