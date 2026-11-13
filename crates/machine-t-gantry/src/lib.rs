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

use sim_components::mech::{Belt, CableChain, Gearbox, HardStop, HoldingBrake, JUMP_STATES, Lugre, Ripple, SectionForce, ShaftFriction, ThermalFeedback, ToothJump, ToothMesh, drive_excitations, span_detuning, span_frequency, wrap_angle};
use sim_core::{Arrow, AxisInfo, BeltSection, Config, ConfigError, ENERGY_PARTS, FRICTION_SIGNALS, FrictionContact, LOSS_PREFIX, Label, Machine, MachineEntry, Mode, MotorInfo, Part, Scene, Shape, SignalInfo, TORQUE_PARTS, ThermalPart};

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
    for effect in ["belt_tooth_stiffness", "idlers", "cable_chain", "part_heating", "thermal_feedback", "belt_tooth_jump", "belt_span_vibration", "belt_wrap"] {
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
    /// The gearbox of each motor: `gearbox` with its own efficiency once that follows its temperature
    /// (effects.thermal_feedback).
    gears: [Gearbox; 2],
    /// With effects.thermal_feedback: the law, and the X and Z guides' viscous coefficients at the reference temperature.
    thermal: Option<(ThermalFeedback, [f64; 2])>,
    /// With effects.belt_tooth_jump (elastic model only): the law, and the energy the last step's jumps took, J/s.
    jump: Option<ToothJump>,
    jump_loss: f64,
    /// With effects.belt_span_vibration: the belt's mass per metre, for the spans' string frequencies.
    span_mass: Option<f64>,
    /// With effects.belt_wrap: the carriage idlers' radius, for the belt's wrap at the motor pulleys (`belt_wraps`).
    wrap_idler: Option<f64>,
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
// With effects.belt_tooth_jump the elastic state goes on with the slip states of pulley A, then of pulley B
// (`JUMP_STATES` each).
const JUMP: usize = N_ELASTIC;
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
            gears: [Gearbox::from_config(cfg)?, Gearbox::from_config(cfg)?],
            thermal: match ThermalFeedback::from_config(cfg)? {
                Some(law) => Some((law, [cfg.non_negative("friction_x.viscous_n_s_m")?, cfg.non_negative("friction_z.viscous_n_s_m")?])),
                None => None,
            },
            jump: None,
            jump_loss: 0.0,
            span_mass: if cfg.bool("effects.belt_span_vibration")? { Some(cfg.positive("belt.mass_kg_m")?) } else { None },
            wrap_idler: if cfg.bool("effects.belt_wrap")? { Some(idler_radius) } else { None },
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
        if m.elastic {
            // Half the belt's mass is lumped on each motor pulley (below).
            m.jump = ToothJump::from_config(cfg, r, 0.5 * m.belt.mass_per_m * (geo.iter().sum::<f64>() - 4.0 * m.belt.tooth_length))?;
        }
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

    /// The Z axis and what hangs from it, as `scene` draws them before the frame sways, and their lean: with
    /// effects.tool_flex they turn about the carriage so that the tool moves by its flex travel, `exaggeration` times
    /// (`Scene::exaggeration`; deflections are micrometres).
    fn hanging(&self, s: &[f64], exaggeration: f64) -> (Vec<Part>, f64) {
        let (x, z, span) = (s[0], s[1], self.span_min);
        let (y, cx, half) = (-0.05, span + x, 0.03);
        let (top, bottom) = (span + z, z - self.axes[1].max - span);
        let part = Part::new;
        let mut hanging = vec![
            part("z-axis", Shape::Box { size: [0.04, 0.03, top - bottom] }, [cx, y, (top + bottom) / 2.0], [0.78, 0.8, 0.84]),
            part("clamp", Shape::Box { size: [0.09, 0.035, 0.03] }, [cx, y, top + 0.015], [0.85, 0.3, 0.2]),
            // The tool: drawn where `scene_point` says, at the carriage's place on the beam.
            part("flange", Shape::Box { size: [0.07, 0.05, 0.02] }, self.scene_point(&[x, z]).unwrap_or_default(), [0.3, 0.3, 0.33]),
        ];
        if self.payload() > 0.0 {
            hanging.push(part("payload", Shape::Box { size: [0.1, 0.1, 0.1] }, [cx, y, bottom - half - 0.072], [0.2, 0.7, 0.3]));
        }
        sim_core::scene::idler(&mut hanging, [cx, bottom], y, half - 0.002, -2.0 * x / half);
        let u = if self.elastic { s[U] } else { 0.0 };
        let tool_arm = -self.scene_point(&[x, z]).unwrap_or_default()[2];
        let lean = (exaggeration * self.flex_at(z).2 * u / tool_arm.max(0.05)).clamp(-0.5, 0.5).asin();
        let (sin, cos) = lean.sin_cos();
        for p in &mut hanging {
            [p.pos[0], p.pos[2]] = [cx + (p.pos[0] - cx) * cos - p.pos[2] * sin, (p.pos[0] - cx) * sin + p.pos[2] * cos];
            p.angle += lean;
        }
        (hanging, lean)
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

    /// The pulley angles as far as the belt sections see them: each pulley's angle plus the belt's slip over it
    /// (effects.belt_tooth_jump) over the radius, and their rates.
    fn fed_angles(&self, s: &[f64]) -> ([f64; 2], [f64; 2]) {
        let mut out = ([s[A], s[A + 1]], [s[V + A], s[V + A + 1]]);
        if self.jump.is_some() {
            for k in 0..2 {
                out.0[k] += s[JUMP + JUMP_STATES * k] / self.r;
                out.1[k] += s[JUMP + JUMP_STATES * k + 1] / self.r;
            }
        }
        out
    }

    fn sections(&self, s: &[f64]) -> [SectionForce; 3] {
        let ([a, b], [va, vb]) = self.fed_angles(s);
        let (geo, mat, r) = (self.geo(s[X], s[Z]), self.mat(a, b), self.r_mat);
        let (vx, vz) = (s[V + X], s[V + Z]);
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
            let gear = self.gears[k].torque(phi, w_phi, theta, w_theta);
            let (brake, brake_rate) = self.brake.torque(self.brake_engaged, w_phi, s[BRAKE + k]);
            // While its teeth are out of mesh the pulley feels only the ridge it holds the sliding belt back with.
            let on_pulley = self.jump.as_ref().map_or(belt_torque[k], |j| j.pulley_torque(&s[JUMP + JUMP_STATES * k..], belt_torque[k], r));
            let j_pulley = self.jump.as_ref().map_or(self.j_pulley, |j| j.pulley_inertia(&s[JUMP + JUMP_STATES * k..], self.j_pulley, r));
            out[V + A + k] = (gear.output + on_pulley + r * self.mesh.force(r * theta)) / j_pulley;
            if let Some(jump) = &self.jump {
                let at = JUMP + JUMP_STATES * k;
                jump.rates(&s[at..], belt_torque[k] / r, r * out[V + A + k], &mut out[at..]);
            }
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
            loss = std::array::from_fn(|k| self.gears[k].loss_factor(through_gear[k] * r, vel[k] / ratio));
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
        let ([a, b], _) = self.fed_angles(s);
        let (geo, mat) = (self.geo(s[X], s[Z]), self.mat(a, b));
        let belt: f64 = (0..3).map(|i| self.belt.energy(geo[i], mat[i])).sum();
        let gears: f64 = (0..2).map(|k| self.gearbox.energy(s[PA + k], s[A + k])).sum();
        let slip = self.jump.as_ref().map_or(0.0, |j| (0..2).map(|k| j.slip_energy(&s[JUMP + JUMP_STATES * k..], self.r * s[V + A + k])).sum());
        let turning: f64 = slip + (0..2).map(|k| 0.5 * self.j_pulley * s[V + A + k].powi(2) + 0.5 * self.j_rotor * s[V + PA + k].powi(2)).sum::<f64>();
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
                let gear = self.gears[k].torque(phi, w_phi, theta, w_theta);
                gear_loss[k] = gear.rotor - gear.output / ratio;
                gear_power[k] = self.gears[k].loss_power(phi, w_phi, theta, w_theta);
                brake[k] = self.brake.torque(self.brake_engaged, w_phi, s[BRAKE + k]).0;
                mesh_in += r * self.mesh.force(r * theta) * w_theta;
            }
            // The rates of `sections`.
            let ([a, b], [va, vb]) = self.fed_angles(s);
            let (rm, (geo, mat)) = (self.r_mat, (self.geo(s[X], z), self.mat(a, b)));
            let (geo_rate, mat_rate) = ([vx + vz, -2.0 * vz, vz - vx], [rm * va, rm * (vb - va), -rm * vb]);
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
            let mut d = [0.0; N_ELASTIC + 2 * JUMP_STATES];
            self.elastic_derivatives(s, torque, &mut d[..self.state_len()]);
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
        if self.jump.is_some() {
            out.push(self.jump_loss);
        }
        out.extend(self.energy_parts(s));
        for (friction, v, state, scale) in [(&self.friction_x, vx, sx, scales.0), (&self.friction_z, vz, sz, scales.1)] {
            out.push(v);
            out.extend(friction.split(v, state, scale));
            out.push(scale);
        }
        // Stretch per belt section: its length less its unstretched belt; in the rigid model that of pretension.
        let geo = self.geo(s[X], z);
        let mat = if self.elastic {
            let ([a, b], _) = self.fed_angles(s);
            self.mat(a, b)
        } else {
            geo.map(|g| self.belt.rest_length(g))
        };
        out.extend((0..3).map(|i| geo[i] - mat[i]));
    }

    /// The first-mode frequency of each free belt span (`span_frequency`) and how far the nearest of them is from what
    /// the two drives excite (`span_detuning`). The spans run between the clamps, the idlers and the motor pulleys, in
    /// the order of the belt from the left clamp: 1 and 2 are section 1, 3 to 6 section 2, 7 and 8 section 3; their
    /// lengths are those of `geo` without the teeth in mesh.
    fn span_signals(&self, s: &[f64], mass: f64, out: &mut Vec<f64>) {
        let (x, z, sx, sz, span) = (s[X], s[Z], self.axes[0].max, self.axes[1].max, self.span_min);
        let t = if self.elastic { self.sections(s).map(|f| f.tension) } else { [self.belt.pretension; 3] };
        let (down, left, right) = (span + sz - z, span + x, span + sx - x);
        let spans = [(span + z, t[0]), (left, t[0]), (left, t[1]), (down, t[1]), (down, t[1]), (right, t[1]), (right, t[2]), (span + z, t[2])];
        let (mut angle, mut speed) = ([0.0; 2], [0.0; 2]);
        self.motor_state(s, &mut angle, &mut speed);
        let mut excitations = Vec::with_capacity(6);
        for k in 0..2 {
            let pulley = if self.elastic { s[V + A + k] } else { speed[k] / self.gearbox.ratio };
            drive_excitations(self.mesh.pitch, self.r * pulley, &self.ripple, speed[k], &mut excitations);
        }
        let frequencies = spans.map(|(length, tension)| span_frequency(length, tension, mass));
        out.extend(frequencies);
        out.push(frequencies.iter().map(|f| span_detuning(*f, &excitations)).fold(1.0, f64::min));
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
        if self.jump.is_some() {
            powers.push(("tooth_jump", "group-belt", "quantity-tooth-jump-power"));
        }
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
        if !self.elastic {
            N_RIGID
        } else if self.jump.is_some() {
            N_ELASTIC + 2 * JUMP_STATES
        } else {
            N_ELASTIC
        }
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
        *state = vec![0.0; self.state_len()];
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

    /// F_x = T3 - T1 on the carriage and F_z = 2 T2 - T1 - T3 on the Z axis carry the moved masses (without the pulleys
    /// and rotors of `feedforward_torque`), and the belt keeps its length: sum (T_i - P) L_i = 0 over the geometric
    /// section lengths L_i (the elastic law T = EA (L - l) / l to first order in T / EA). With d_i = T_i - P:
    /// d1 = -((F_x + F_z) L2 / 2 + F_x L3) / (L1 + L2 + L3), d3 = d1 + F_x, d2 = d1 + (F_x + F_z) / 2.
    fn rigid_tensions(&self, pos: &[f64], acc: &[f64], out: &mut Vec<f64>) {
        let fx = (self.m_x() + self.m_idlers[0] + self.chain.mass(pos[0]).0) * acc[0];
        let fz = (self.m_z() + self.m_idlers[1]) * acc[1] + self.m_z() * self.g;
        let l = self.geo(pos[0], pos[1]);
        let d1 = -((fx + fz) * l[1] / 2.0 + fx * l[2]) / (l[0] + l[1] + l[2]);
        let p = self.belt.pretension;
        out.extend([p + d1, p + d1 + (fx + fz) / 2.0, p + d1 + fx]);
    }

    fn energy(&self, s: &[f64]) -> f64 {
        self.energy_parts(s).iter().sum()
    }

    /// The lower of the two modes of `modes`.
    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        self.modes(tool_pose).first().map(|mode| mode.hz)
    }

    /// The Z axis leaning on its guide while the belt holds the carriage: two masses in a row. The carriage
    /// sits on the belt's stiffness in X (the two sections that end at the clamp, EA / L each, with the
    /// pulleys taken as held), the Z axis with what it carries on the spring of its guide at this height.
    /// Both modes, from w^4 - (kb / mc + kz / mc + kz / mz) w^2 + kb kz / (mc mz) = 0: in the lower one carriage and
    /// Z axis swing together, in the upper one against each other. Each one's damping ratio is that of its shape v
    /// (`mode_shape`) in (x, u), v'Cv / (2 w v'Mv) with C = [[cb, 0], [0, cz]]: cb the damping of the two sections
    /// (`belt.damping_n_s_m` each, or with effects.belt_material_damping `belt.damping_time_s` EA / L), cz that of the
    /// guide (`tool_flex.damping_n_s_m`); the cross terms of a damping that is not proportional are left out.
    fn modes(&self, tool_pose: &[f64]) -> Vec<Mode> {
        let Some(flex) = self.flex.as_ref().filter(|_| self.elastic) else { return Vec::new() };
        let geo = self.geo(tool_pose[0], tool_pose[1]);
        let (belt, lean) = (self.belt.ea / geo[0] + self.belt.ea / geo[2], self.flex_at(tool_pose[1]).0);
        let belt_damping = 2.0 * self.belt.damping + self.belt.damping_time * belt;
        let (mc, mz) = (self.m_x() - self.m_z() + self.m_idlers[0] + self.chain.mass(tool_pose[0]).0, self.m_z());
        let (sum, product) = ((belt + lean) / mc + lean / mz, belt * lean / (mc * mz));
        let root = (sum * sum - 4.0 * product).sqrt();
        [sum - root, sum + root]
            .map(|twice| {
                let w2 = 0.5 * twice;
                let (vx, vu) = (lean - w2 * mz, w2 * mz);
                let (kinetic, damped) = (mc * vx * vx + mz * (vx + vu) * (vx + vu), belt_damping * vx * vx + flex.damping * vu * vu);
                Mode { hz: w2.sqrt() / std::f64::consts::TAU, damping: damped / (2.0 * w2.sqrt() * kinetic) }
            })
            .to_vec()
    }

    /// With effects.belt_wrap: the wrap at each motor pulley, from where `scene` puts the wheels next to it, the carriage
    /// idlers (`idler.radius_m`) beside the carriage (0.03 m either side of its middle) at the heights r + ri and
    /// -(r + ri). The spans to them are inner tangents that leave level, atan2(r + ri, d) = asin((r + ri) / D), at every
    /// pose: half a turn on each pulley wherever the carriage is (`mech::wrap_angle`). Only a wheel at another height
    /// would make it change with x.
    fn belt_wraps(&self, tool_pose: &[f64]) -> Vec<f64> {
        let Some(ri) = self.wrap_idler else { return Vec::new() };
        let (r, end, cx) = (self.r, 2.0 * self.span_min + self.axes[0].max, self.span_min + tool_pose[0]);
        [cx - 0.03 - ri, end - cx - 0.03 - ri].iter().map(|&d| wrap_angle(r, [d, r + ri, ri], [d, r + ri, ri])).collect()
    }

    /// The mode of `resonance_hz`. What moves: the carriage on the frame, x (with the pulleys held, all of it is
    /// belt stretch), and the Z axis's centre of mass against the carriage, u. Nothing else: pulleys, motors and
    /// frame stay. In (x, u), with the Z axis at x + u over ground:
    ///   M = [[mc + mz, mz], [mz, mz]], K = [[kb, 0], [0, kz]],
    /// kb, kz, mc, mz as in `modes`. The second row of (K - w^2 M) v = 0, -w^2 mz vx + (kz - w^2 mz) vu = 0,
    /// gives v = (kz - w^2 mz, w^2 mz), scaled so that the larger of the carriage's travel x and the tool's travel
    /// x + tip u (`tool_state`) is 1 m. Not to the tool's alone: high on the Z axis tip turns negative and the tool
    /// stands still in the mode at one height, where that scale would be infinite. In the lower mode x and u have
    /// the same sign. Low on the Z axis u is the larger: at x = 0.9 m, z = 0.05 m of the default config,
    /// w^2 = 2.394e4, kz = 1.902e5 N/m, mz = 7 kg: v = (2.26e4, 1.676e5) before scaling, tip = 2.585, so
    /// (0.050, 0.368), the tool moving 1 m: it swings mostly by the Z axis leaning.
    fn mode_shape(&self, tool_pose: &[f64]) -> Option<(f64, Vec<f64>)> {
        let hz = self.resonance_hz(tool_pose)?;
        let (w2, kz, mz, tip) = ((std::f64::consts::TAU * hz).powi(2), self.flex_at(tool_pose[1]).0, self.m_z(), self.flex_at(tool_pose[1]).2);
        let (vx, vu) = (kz - w2 * mz, w2 * mz);
        let largest = vx.abs().max((vx + tip * vu).abs());
        let mut shape = vec![0.0; self.state_len()];
        (shape[X], shape[U]) = (vx / largest, vu / largest);
        Some((hz, shape))
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

    /// Belt tooth jump at each motor pulley (`ToothJump`): a positive slip of pulley A feeds section 1, of pulley B
    /// section 2.
    fn after_step(&mut self, s: &mut [f64], dt: f64) {
        self.jump_loss = 0.0;
        let Some(mut jump) = self.jump.clone() else { return };
        let wraps = self.belt_wraps(&[s[X], s[Z]]);
        for k in 0..2 {
            if let Some(&wrap) = wraps.get(k) {
                jump.set_wrap(wrap);
            }
            let (before, t) = (self.energy(s), self.sections(s).map(|f| f.tension));
            let at = JUMP + JUMP_STATES * k;
            let (rest, mut v) = ((self.j_pulley / (self.r * self.r)) - jump.mass, self.r * s[V + A + k]);
            if jump.update(&mut s[at..at + JUMP_STATES], t[k], t[k + 1], rest, &mut v) {
                s[V + A + k] = v / self.r;
                self.jump_loss += (before - self.energy(s)) / dt;
            }
        }
    }

    /// The gearboxes (where there are some) and the X and Z guides, in the order of `thermal_parts`.
    fn set_part_temperatures(&mut self, rises: &[f64]) {
        let Some((law, viscous)) = &self.thermal else { return };
        let gearboxes = if self.gearbox.is_coupling() { 0 } else { 2 };
        for k in 0..gearboxes {
            self.gears[k].efficiency = law.gearbox_efficiency(rises[k]);
        }
        self.friction_x.viscous = viscous[0] * law.viscosity_factor(rises[gearboxes]);
        self.friction_z.viscous = viscous[1] * law.viscosity_factor(rises[gearboxes + 1]);
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
        if self.jump.is_some() {
            for motor in &self.motors {
                out.push(SignalInfo::new(format!("pulley.{}.tooth_jumps", motor.id), motor.key, "quantity-tooth-jumps", ""));
                out.push(SignalInfo::new(format!("pulley.{}.tooth_slip", motor.id), motor.key, "quantity-tooth-slip", "m"));
            }
        }
        if self.span_mass.is_some() {
            out.extend((1..=8).map(|i| SignalInfo::new(format!("belt.span.{i}.frequency"), &format!("belt-span-{i}"), "quantity-span-frequency", "Hz")));
            out.push(SignalInfo::new("belt.span_detuning", "group-belt", "quantity-span-detuning", ""));
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
            if let Some(mass) = self.span_mass {
                self.span_signals(s, mass, out);
            }
            return;
        }
        let sec = self.sections(s);
        let (ffx, ffz, ..) = self.guide_friction(s[Z], s[V + X], s[V + Z], s[S], s[S + 1], torque);
        let (a, b) = (s[A], s[A + 1]);
        out.extend(sec.iter().map(|f| f.tension));
        out.extend([s[X] - self.r / 2.0 * (a + b), ffx, s[Z] - self.r / 2.0 * (a - b), ffz, s[Q], self.flex_at(s[Z]).2 * s[U], s[SLACK]]);
        out.extend([sec[0].tension + sec[1].tension, sec[1].tension + sec[2].tension, load_x, load_z]);
        out.extend((0..2).map(|k| self.gears[k].torque(s[PA + k], s[V + PA + k], s[A + k], s[V + A + k]).output));
        if self.jump.is_some() {
            for k in 0..2 {
                out.extend([s[JUMP + JUMP_STATES * k + 4], s[JUMP + JUMP_STATES * k]]);
            }
        }
        if let Some(mass) = self.span_mass {
            self.span_signals(s, mass, out);
        }
    }
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.view_signals(s, torque, out);
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(s, torque).0);
        if self.jump.is_some() {
            out.push(self.jump_loss);
        }
    }

    fn scene(&self, s: &[f64], out: &mut Scene) {
        use sim_core::scene::{belt_along, drive_unit, idler, on_circle};
        use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, FRAC_PI_8, TAU};
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
        // The beam has the height of the EXCT's profile, 170 mm; the posts stand under the ends of the carriage's
        // travel, where the EXCT's mounting plates are.
        let beam_half = 0.085;
        let part = Part::new;
        out.labels.clear();
        out.parts.clear();
        out.arrows.clear();
        let post = Shape::Box { size: [0.06, 0.06, -beam_half - floor] };
        out.parts.extend([
            part("beam", Shape::Box { size: [end - 0.14, 0.07, 2.0 * beam_half] }, [end / 2.0, 0.045, 0.0], [0.62, 0.64, 0.68]),
            part("post", post.clone(), [span, 0.045, (floor - beam_half) / 2.0], [0.45, 0.47, 0.5]),
            part("post", post, [end - span, 0.045, (floor - beam_half) / 2.0], [0.45, 0.47, 0.5]),
            part("carriage", Shape::Box { size: [0.16, 0.02, 0.16] }, [cx, -0.02, 0.0], [0.2, 0.22, 0.26]),
        ]);
        cable_chain(&mut out.parts, end, x, cx, beam_half);
        let (hanging, lean) = self.hanging(s, out.exaggeration);
        let q = if self.elastic { s[Q] } else { 0.0 };
        let turn = |p: [f64; 2]| {
            let (sin, cos) = lean.sin_cos();
            [cx + (p[0] - cx) * cos - p[1] * sin, (p[0] - cx) * sin + p[1] * cos]
        };
        out.parts.extend(hanging);
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

        // The belt path from the clamp on the left to the clamp on the right. Its length does not
        // depend on the pose, so teeth at fixed belt length travel around the pulleys by themselves.
        let (a, b, bend) = ([0.0, 0.0], [end, 0.0], [cx, bottom]);
        // Each point with the section of the strip that ends at it, and whether that strip is a free horizontal span
        // (between a motor pulley and a carriage idler), which sags. Strips up to the middle of each motor pulley
        // belong to the section before it. Points on the Z axis (its ends and the bend at its bottom) lean with it.
        // The wraps on the motor pulleys get 8 strips per half turn, so the teeth there sit close to the pitch circle
        // (a chord is 0.994 of its arc and lies r (1 - cos(pi / 16)) = 0.02 r inside it at the middle).
        let wrap = |centre: [f64; 2], from: f64, sections: [usize; 2]| {
            (1..=8).map(move |k| (on_circle(centre, r, from + k as f64 * FRAC_PI_8), sections[(k > 4) as usize], false))
        };
        let mut key = vec![(turn([left, top]), 0, false), ([left, r], 0, false), ([0.0, r], 0, true)];
        key.extend(wrap(a, FRAC_PI_2, [0, 1]));
        key.extend([([left, -r], 1, true), (turn([left, bottom]), 1, false)]);
        key.extend((5..=7).map(|k| (turn(on_circle(bend, half, k as f64 * FRAC_PI_4)), 1, false)));
        key.extend([(turn([right, bottom]), 1, false), ([right, -r], 1, false), ([end, -r], 1, true)]);
        key.extend(wrap(b, -FRAC_PI_2, [1, 2]));
        key.extend([([right, r], 2, true), (turn([right, top]), 2, false)]);
        // Sagging spans get points along their sag, drawn larger like the other deflections, but never deeper than a
        // quarter of the span. Each span has a pulley or idler under one end (the motor pulley under the upper spans,
        // the lower carriage idler under the lower ones) and one above or level at the other. It leaves the end with
        // the wheel under it level, as a belt does off a wrap, so the sag cannot cut into that wheel: the shape is
        // 27/4 t^2 (1 - t), deepest at a third of the span from the other end, at the depth of the parabola's middle.
        // The belt length of every point is that of the straight path, so the teeth keep their places on the belt
        // (`belt_along`).
        let weight = self.belt.mass_per_m * self.g;
        let (mut path, mut along, mut section) = (vec![key[0].0], vec![0.0], Vec::new());
        for pair in key.windows(2) {
            let ((p, _, _), (q, sec, sags)) = (pair[0], pair[1]);
            let length = (q[0] - p[0]).hypot(q[1] - p[1]);
            // The motor pulleys stand at the ends, x = 0 and x = end: an upper span has its pulley under the end at
            // a pulley, a lower span its idler under the end away from it.
            let at_pulley = |point: [f64; 2]| point[0].abs() < 1e-9 || (point[0] - end).abs() < 1e-9;
            let level_at_q = if p[1] > 0.0 { at_pulley(q) } else { !at_pulley(q) };
            let sag = if sags { (out.exaggeration * span_sag(weight, length, tension[sec])).min(length / 4.0) } else { 0.0 };
            let pieces = if sags { SAG_PIECES } else { 1 };
            let start = *along.last().unwrap();
            for k in 1..=pieces {
                let t = k as f64 / pieces as f64;
                let shape = if level_at_q { 6.75 * (1.0 - t) * (1.0 - t) * t } else { 6.75 * t * t * (1.0 - t) };
                path.push([p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]) - sag * shape]);
                along.push(start + t * length);
                section.push(sec);
            }
        }
        belt_along(&mut out.parts, &path, &along, y, 0.025, 0.04, |i| shade[section[i]]);

        // The belt's pull: on the carriage along X, on the Z axis along Z, and on each motor pulley's shaft, where
        // both sections that leave it pull the same way.
        let [t1, t2, t3] = tension;
        out.arrows.extend([
            Arrow { pos: [cx, y, 0.0], force: [t3 - t1, 0.0, 0.0], owner_key: self.axes[0].key },
            Arrow { pos: [cx, y, bottom], force: [0.0, 0.0, 2.0 * t2 - t1 - t3], owner_key: self.axes[1].key },
            Arrow { pos: [0.0, y, 0.0], force: [t1 + t2, 0.0, 0.0], owner_key: self.motors[0].key },
            Arrow { pos: [end, y, 0.0], force: [-(t2 + t3), 0.0, 0.0], owner_key: self.motors[1].key },
        ]);

        let rpm = |speed: f64| speed * 60.0 / TAU;
        out.labels.extend([
            Label { pos: [0.0, y, 0.11], owner_key: self.motors[0].key, value: rpm(rotor_speed[0]), unit_key: "unit-rpm", decimals: 0 },
            Label { pos: [end, y, 0.11], owner_key: self.motors[1].key, value: rpm(rotor_speed[1]), unit_key: "unit-rpm", decimals: 0 },
            Label { pos: [cx, y, top + 0.08], owner_key: self.axes[0].key, value: x, unit_key: "unit-m", decimals: 3 },
            Label { pos: [cx + 0.12, y, bottom], owner_key: self.axes[1].key, value: z, unit_key: "unit-m", decimals: 3 },
        ]);

        // The frame sways by q (x is the carriage on the frame): all of it moves, and the posts lean from the floor.
        let sway = out.exaggeration * q;
        let post_tilt = -(sway / (-beam_half - floor)).atan();
        for p in &mut out.parts {
            if p.id == "post" {
                p.pos[0] += sway / 2.0;
                p.angle = post_tilt;
            } else {
                p.pos[0] += sway;
                if let Shape::Segment { to, .. } = &mut p.shape {
                    to[0] += sway;
                }
            }
        }
        out.labels.iter_mut().for_each(|l| l.pos[0] += sway);
        out.arrows.iter_mut().for_each(|a| a.pos[0] += sway);
    }

    /// The flange under the idler at the bottom of the Z axis, in the belt plane.
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        Some([self.span_min + tool_pose[0], -0.05, tool_pose[1] - self.axes[1].max - self.span_min - 0.042])
    }

    /// What hangs from the carriage and moves through the work area: the Z axis, the flange and the payload, where
    /// `scene` draws them (unexaggerated), without drawing the rest: the live check asks for them every base step.
    fn collision_bodies(&self, s: &[f64]) -> Vec<sim_core::Aabb> {
        let sway = if self.elastic { s[Q] } else { 0.0 };
        let mut boxes = sim_core::scene::scene_boxes(&self.hanging(s, 1.0).0, &["z-axis", "flange", "payload"]);
        boxes.iter_mut().for_each(|b| b.centre[0] += sway);
        boxes
    }
}

/// How deep a slack belt span is drawn (before `Scene::exaggeration`), and the most any span sags in the drawing.
const SLACK_SAG: f64 = 0.015;
/// Strips per sagging span: their lengthening over the chord is a little short of the smooth sag's.
const SAG_PIECES: usize = 6;

/// The sag at the middle of a horizontal belt span of `span` metres under `tension`, its weight `weight` N/m: the
/// shallow catenary's w L^2 / (8 T). A span without tension is drawn at `SLACK_SAG`.
fn span_sag(weight: f64, span: f64, tension: f64) -> f64 {
    if tension <= 0.0 { SLACK_SAG } else { (weight * span * span / (8.0 * tension)).min(SLACK_SAG) }
}

/// The cable chain on top of the beam, link by link, as on the EXCT: fixed near the right end, its lower run lies
/// on the beam, turns up in a loop and comes back on top to a bracket on the carriage. Its length stays, so the
/// loop moves half as far as the carriage (its upper run is 0.02 m + x / 2 long), and each link keeps its place
/// along the chain from the fixed end.
/// shortcut: drawn only. Its force and mass follow effects.cable_chain, whose model has the chain fixed at x = 0;
/// both are the same to the carriage (half the travel joins the moving run).
fn cable_chain(parts: &mut Vec<Part>, end: f64, x: f64, cx: f64, beam_half: f64) {
    use std::f64::consts::PI;
    // The EXCT-30's chain: 49 mm wide, 24 mm links, a loop of 90 mm radius just left of the bracket at x = 0.
    let (radius, height, width, pitch, depth) = (0.09, 0.024, 0.049, 0.03, 0.05);
    let (fixed, moving) = (end - 0.07, cx - 0.1);
    let upper = 0.02 + x / 2.0;
    let turn = moving - upper;
    let (low, high) = (beam_half + height / 2.0, beam_half + height / 2.0 + 2.0 * radius);
    let lower = fixed - turn;
    let colour = [0.16, 0.16, 0.17];
    // The links share the chain's length evenly, so the last one ends at the bracket.
    let length = lower + PI * radius + upper;
    let links = (length / pitch).round().max(1.0) as usize;
    let pitch = length / links as f64;
    parts.extend((0..links).map(|i| {
        let along = (i as f64 + 0.5) * pitch;
        let (pos, angle) = if along < lower {
            ([fixed - along, low], 0.0)
        } else if along < lower + PI * radius {
            let round = (along - lower) / radius;
            ([turn - radius * round.sin(), (low + high) / 2.0 - radius * round.cos()], -round)
        } else {
            ([turn + along - lower - PI * radius, high], 0.0)
        };
        Part::new("cable-chain", Shape::Box { size: [0.9 * pitch, width, height] }, [pos[0], depth, pos[1]], colour).turned(angle)
    }));
    // The bracket that carries the chain's end on the carriage.
    let (base, lift) = (0.08, high - height / 2.0);
    parts.push(Part::new("chain-bracket", Shape::Box { size: [0.02, width, lift - base] }, [moving, depth, (base + lift) / 2.0], colour));
}

#[cfg(test)]
mod scene_tests;
#[cfg(test)]
mod tests;
