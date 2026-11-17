//! A winder and unwinder with a dancer, as in film, paper and foil converting: a web runs from an unwind roll down
//! over a dancer roller and up to a rewind roll, at a line speed, under a tension. Both rolls are centre-driven, each by
//! a servo motor through a gearbox of ratio i; the dancer is a roller on a vertical slide, pushed down by a pneumatic
//! cylinder (its force the tension setpoint, `dancer`), and its position is the measured value of the tension loop.
//!
//! The tool axis is the line position L (m): the web wound onto the rewind roll since it was empty, which is the web
//! paid out by the unwind roll since it was full when the dancer stands in the middle. So L = 0 has the unwind full and
//! the rewind on its core, L = the roll's capacity the other way round, and every pose is a pair of radii. The axis runs
//! over a stretch of the roll (`roll.run_from_m`, `roll.run_m`): a run takes metres of a roll of hundreds.
//!
//! Rolls: a roll of web of thickness h on a core of radius r_c gains h per turn, so its radius is linear in its angle
//! (an Archimedean spiral), r = r_c + h phi / (2 pi) for the rewind and r = r_f - h phi / (2 pi) for the unwind, and
//! r^2 = r_c^2 + h s / pi with s the web on it: s(phi) = phi (r_c + r) / 2. Its inertia is the web's, a thick-walled
//! cylinder, 1/2 rho w pi (r^4 - r_c^4), plus core and shaft, plus the motor's rotor and the gearbox's input through
//! the ratio, i^2 J_m: the reflected inertia changes over a run by r^4, the torque for a tension by r.
//!
//! A roll that gains or loses web at its surface, where the web moves with the roll's surface, gets no torque from the
//! web's mass flow: J dw/dt = sum of the torques (not d(J w)/dt). Unwind: J_u w_u' = i (tau_u - friction) + T1 r_u;
//! rewind: J_r w_r' = i (tau_r - friction) - T2 r_r. Kinetic energy is carried from one roll to the other with the
//! web (1/2 lambda v^3 per second each way, lambda = rho w h), booked with the web's transport (`power.web_transport`).
//!
//! The web (effects.web_stretch): two spans, unwind to dancer and dancer to rewind, of length L_k0 + y (y the dancer's
//! travel down from the middle; each span is vertical, so both lengthen by y), each holding M_k of web unstretched.
//! Its tension T = EA (L / M - 1) while L > M, plus the material damping EA tau dE/dt, and 0 while slack. The web
//! carries its strain from span to span: what leaves a span leaves at its strain, so
//!   dM1/dt = v_u / (1 + eps_0) - v_d / (1 + eps_1),   dM2/dt = v_d / (1 + eps_1) - v_r / (1 + eps_2),
//! eps_0 the strain the unwind roll was wound at (`roll.wound_tension_n`), v_d the dancer roller's surface speed, and
//! upwind in the other direction (backwards the rewind gives off web at the tension setpoint's strain). To first order
//! in the strains this is the span equation L dT2/dt = EA (v2 - v1) + v1 T1 - v2 T2 plus EA dL/dt; the form here keeps
//! the web's length exactly: what leaves the unwind arrives at the rewind plus what the spans store. The span's
//! length change with the rolls' radii (the web leaves each roll over a fixed idler, not drawn) is left out, and the
//! web's own mass in the spans.
//!
//! The dancer roller turns with the web (no slip) with its own inertia and bearing drag: J_d w_d' = R (T2 - T1) -
//! drag; the dancer's slide carries it with its mass, its friction and its stops (effects.hard_stops):
//! m y'' = F_cyl - k_air y + m g - T1 - T2 - friction. The cylinder's force is set so that it holds the tension
//! setpoint, F_cyl = 2 T_set - m g (the dancer's own weight taken off, as dancer controls do), with the taper
//! T_set(r) = T0 (1 - k (1 - r_c / r)) over the rewind's radius (effects.tension_taper).
//!
//! The rigid model (effects.web_stretch off): the web does not stretch, so the dancer stands at half of what the unwind
//! paid out more than the rewind took up, y = (s_u - s_r) / 2, and the roller turns at (v_u + v_r) / (2 R); the two
//! tensions follow from the dancer's and the roller's balance (`Winder::eval`).
//!
//! Control, as such drives are run, with the controller's own loops: both motors follow the line position by
//! electronic gearing (the inverse kinematics: the angles of the line position at the radii computed from the wound
//! length, so the speed setpoint is i v / r), with the tension's torque and the roll's inertia fed forward
//! (T r / i and J(r) w' / i plus the rotor's, `feedforward_torque_moving`). The dancer's position loop (`dancer.position
//! _loop`), a PI as a winder's technology controller runs it, trims the unwind: its speed trim, integrated, is web the
//! unwind pays out beyond the line position, which `inverse` adds to the unwind's wound length, so the unwind runs on
//! the line speed over its radius plus the trim, its radius following the web it gave, and its position loop holds the
//! integral of that. Of a state the line position is the mean of the two rolls' (`tool_state`), the unwind's less the
//! trim: in the rigid model the web's travel at the dancer roller.
//!
//! Gearboxes: rigid, with their efficiency (effects.gearbox_efficiency), no-load drag (effects.gearbox_drag) and, with
//! effects.thermal_feedback, both at their temperature; each motor has a holding brake (effects.holding_brake) that keeps
//! the roll, and with it the tension, while the drives are off.

use sim_components::mech::{GearDrag, Gearbox, HoldingBrake, ThermalFeedback};
use sim_core::{
    AxisInfo, Config, ConfigError, ENERGY_PARTS, FrameMomentum, LOSS_PREFIX, Label, Machine, MachineEntry, Mode,
    MotorInfo, Part, RatingCheck, Scene, Shape, SignalInfo, TORQUE_PARTS, ThermalPart,
};
use std::f64::consts::{PI, TAU};

pub static ENTRY: MachineEntry = MachineEntry {
    id: "winder",
    key: "machine-winder",
    default_config: include_str!("../default.toml"),
    i18n: &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))],
    build,
    suite_configs,
};

const G: f64 = 9.80665;
/// Below this power a gearbox's efficiency fades out, so a gearbox at rest has no torque step (as `mech::Gearbox`).
const EFFICIENCY_FADE_W: f64 = 0.5;

/// Effects that take energy out or put it in; the standard suite runs with them off. The taper changes the cylinder's
/// force with the rewind's radius, which puts energy in or takes it out.
pub const LOSSY_EFFECTS: [&str; 6] =
    ["motor_friction", "gearbox_efficiency", "gearbox_drag", "dancer_friction", "web_damping", "tension_taper"];

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    sim_components::base::mount(cfg, Box::new(Winder::new(cfg)?))
}

/// The default config with every loss switched off.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    cfg.set_text("dancer.stop_damping_n_s_m = 0.0").unwrap();
    cfg
}

/// Loss-free rigid and elastic models, with and without the weight on the dancer (payload), run in the middle of the roll
/// (`on_the_roll`). There the two radii are equal: under the constant holding torques of the suite's runs the tension
/// stays where the unwind was wound, and the web carries no energy in or out (`power.web_transport`). At the roll's start
/// the radii grow apart over the run, the tension the torques hold drifts off the wound tension, and the strain the web
/// carries in and out is the energy the elastic runs lose there (0.36 % of what the pulse puts in over the 10 s, as
/// `power.web_transport` books it, which the loss budget of the default config closes).
fn suite_configs() -> Vec<Config> {
    [(false, true), (false, false), (true, true), (true, false)]
        .into_iter()
        .map(|(elastic, payload)| {
            let mut cfg = lossless_config();
            cfg.set_text(&format!("effects.web_stretch = {elastic}")).unwrap();
            cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
            on_the_roll(&mut cfg, 633.0);
            cfg
        })
        .collect()
}

/// `cfg` with its axis over the 20 m of the roll from `from` on, its start 7 m in and its cycle 3 m forward and back as
/// shipped.
pub fn on_the_roll(cfg: &mut Config, from: f64) {
    let (start, end) = (from + 7.0, from + 10.0);
    cfg.set_text(&format!("roll.run_from_m = {from:?}")).unwrap();
    cfg.set_text("roll.run_m = 20.0").unwrap();
    cfg.set_text(&format!("start.pose = [{start:?}]")).unwrap();
    cfg.set_text(&format!(
        "cycle.segment = [{{ target = [{end:?}], dwell_s = 0.3, payload = false }}, \
         {{ target = [{start:?}], dwell_s = 0.3, payload = true }}]"
    ))
    .unwrap();
}

/// A roll of web: core, full radius, the web's thickness, density times width and the inertia of core and shaft.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Roll {
    pub core: f64,
    pub full: f64,
    pub thickness: f64,
    /// rho w, kg/m^2 (per metre of radius and of length: the web's mass per area of its cross-section is rho).
    pub density_width: f64,
    pub core_inertia: f64,
}

impl Roll {
    /// The radius with `wound` metres of web on the core: r^2 = r_c^2 + h s / pi.
    pub fn radius(&self, wound: f64) -> f64 {
        (self.core * self.core + self.thickness * wound / PI).max(0.0).sqrt()
    }
    /// The web a full roll holds, m: pi (r_f^2 - r_c^2) / h.
    pub fn capacity(&self) -> f64 {
        PI * (self.full * self.full - self.core * self.core) / self.thickness
    }
    /// The web's mass on the roll at radius `r`, kg: rho w pi (r^2 - r_c^2).
    pub fn web_mass(&self, r: f64) -> f64 {
        let r = r.max(self.core);
        self.density_width * PI * (r * r - self.core * self.core)
    }
    /// The roll's inertia about its axis at radius `r`: the web, 1/2 rho w pi (r^4 - r_c^4), plus core and shaft.
    pub fn inertia(&self, r: f64) -> f64 {
        let r = r.max(self.core);
        0.5 * self.density_width * PI * (r.powi(4) - self.core.powi(4)) + self.core_inertia
    }
    /// dJ/dr = 2 pi rho w r^3 (0 inside the core).
    fn inertia_slope(&self, r: f64) -> f64 {
        if r > self.core { TAU * self.density_width * r.powi(3) } else { 0.0 }
    }
}

/// Coulomb (smoothed by tanh), viscous: [level, viscous, smoothing speed]; all zero with its effect off.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct Friction([f64; 3]);

impl Friction {
    fn from_config(cfg: &Config, on: bool, keys: [&str; 3]) -> Result<Self, ConfigError> {
        let smoothing = cfg.positive(keys[2])?;
        if !on {
            return Ok(Self([0.0, 0.0, smoothing]));
        }
        Ok(Self([cfg.non_negative(keys[0])?, cfg.non_negative(keys[1])?, smoothing]))
    }
    /// The force or torque opposing a positive speed.
    fn at(&self, speed: f64) -> f64 {
        let [c, b, s] = self.0;
        c * (speed / s).tanh() + b * speed
    }
}

/// What `Winder::eval` works out of a state: kinematics, the web's tensions, the rates and the forces.
#[derive(Debug, Clone, Copy, Default)]
pub struct Eval {
    /// Radii, roll speeds (rad/s), surface speeds and inertias at the rolls (web, core, rotor through the ratio),
    /// unwind first.
    pub r: [f64; 2],
    pub w: [f64; 2],
    pub v: [f64; 2],
    pub j: [f64; 2],
    /// The dancer's travel down from the middle, its speed, and its roller's speed (rad/s).
    pub y: f64,
    pub vy: f64,
    pub wd: f64,
    /// The spans' tensions (unwind to dancer, dancer to rewind), the spring's share of them, their strains, their
    /// lengths, the web they hold and its rate (the elastic model; the rigid one has the tensions alone).
    pub t: [f64; 2],
    pub spring: [f64; 2],
    pub strain: [f64; 2],
    pub length: [f64; 2],
    pub material: [f64; 2],
    pub material_rate: [f64; 2],
    /// The web crossing into span 1 from the unwind, from span 1 to span 2 over the roller, and out of span 2 onto the
    /// rewind, m/s unstretched (the elastic model).
    pub flow: [f64; 3],
    /// The rolls' angular accelerations, the dancer's and its roller's.
    pub acc: [f64; 2],
    pub ydd: f64,
    pub wdd: f64,
    /// Shaft friction, holding brake and the gearbox's drag at each motor (N m, opposing its speed), the drag alone, the
    /// rate of each brake's deflection, the dancer slide's friction (N) and its roller's (N m).
    pub shaft: [f64; 2],
    pub drag: [f64; 2],
    pub brake_rate: [f64; 2],
    /// Each gearbox's factor on what it passes (`Winder::loss_factor`) and the torque at its input, N m.
    pub factor: [f64; 2],
    pub gear_input: [f64; 2],
    pub slide: f64,
    pub roller: f64,
    /// The stops' force on the dancer (N, along y), their stored energy and their loss.
    pub stop: [f64; 3],
    /// The cylinder's force setpoint and its force at the dancer's travel (less the air spring's k y).
    pub cylinder: f64,
    pub load: f64,
}

pub struct Winder {
    pub roll: Roll,
    pub ratio: f64,
    /// Each gearbox's efficiency now (1 with effects.gearbox_efficiency off; with effects.thermal_feedback at its
    /// temperature), its no-load running torque (effects.gearbox_drag) and its lubricant's viscosity over the reference.
    pub efficiency: [f64; 2],
    drag: GearDrag,
    oil: [f64; 2],
    thermal: Option<ThermalFeedback>,
    /// Motor rotor and gearbox input, kg m^2 at the motor.
    pub j_rotor: f64,
    pub ea: f64,
    /// The web's width, m (drawn).
    pub width: f64,
    /// The web's material damping time constant (0 with effects.web_damping off), s.
    pub damping_time: f64,
    /// The tension setpoint at the rewind's core, N, and the taper k (0 with effects.tension_taper off).
    pub tension: f64,
    pub taper: f64,
    /// The strain the unwind roll was wound at.
    pub wound_strain: f64,
    pub spans: [f64; 2],
    pub dancer_mass: f64,
    pub payload: f64,
    pub attached: bool,
    pub roller_radius: f64,
    pub roller_inertia: f64,
    pub air_spring: f64,
    pub stroke: f64,
    slide_friction: Friction,
    roller_friction: Friction,
    motor_friction: Friction,
    /// The motors' holding brakes, engaged while the drives are off (`set_brake_engaged`).
    brake: HoldingBrake,
    /// Stops at +-stroke (effects.hard_stops): stiffness and damping; stiffness 0 without.
    stop: [f64; 2],
    pub g: f64,
    pub elastic: bool,
    /// The dancer's position loop: on, Kp (1/s), Tn (s), the most it trims (m/s), and its state: the web it has had the
    /// unwind pay out beyond the line position (m), the integrator (m/s), the trim of the last step (m/s).
    position_loop: bool,
    trim_gains: [f64; 3],
    pub trim_length: f64,
    trim_integral: f64,
    pub trim_speed: f64,
    drives_off: bool,
    /// Limits of the own verdicts: web's highest and lowest tension, dancer travel, inertia ratio, the motor's peak
    /// torque, its rated torque and its highest speed.
    limits: [f64; 7],
    axes: [AxisInfo; 1],
    motors: [MotorInfo; 2],
}

// Elastic state: [phi_u, phi_r, y, w_u, w_r, v_y, w_d, M1, M2, slack, brake_u, brake_r]: roll angles (rad, unwind in
// its paying-out sense), the dancer's travel, the speeds, the dancer roller's speed, the web in each span (m
// unstretched), the seconds a span has spent slack, and the holding brakes' deflections.
const N_ELASTIC: usize = 12;
// Rigid state: [phi_u, phi_r, w_u, w_r, brake_u, brake_r].
const N_RIGID: usize = 6;
/// The rolls by motor, for signal ids.
const ROLLS: [&str; 2] = ["unwind", "rewind"];

impl Winder {
    pub fn new(cfg: &Config) -> Result<Self, ConfigError> {
        let on = |effect: &str| cfg.bool(&format!("effects.{effect}"));
        let (thickness, width) = (cfg.positive("web.thickness_m")?, cfg.positive("web.width_m")?);
        let roll = Roll {
            core: cfg.positive("roll.core_radius_m")?,
            full: cfg.positive("roll.full_radius_m")?,
            thickness,
            density_width: cfg.positive("web.density_kg_m3")? * width,
            core_inertia: cfg.non_negative("roll.core_inertia_kgm2")?,
        };
        if roll.full <= roll.core {
            return Err(ConfigError::Invalid("roll.full_radius_m".into()));
        }
        let ea = cfg.positive("web.modulus_pa")? * thickness * width;
        let taper = cfg.non_negative("web.taper")?;
        if taper >= 1.0 {
            return Err(ConfigError::Invalid("web.taper".into()));
        }
        let stroke = cfg.positive("dancer.stroke_m")?;
        let (kp, tn) = (cfg.non_negative("dancer.trim_kp_1_s")?, cfg.positive("dancer.trim_tn_s")?);
        let thermal = ThermalFeedback::from_config(cfg)?;
        // What a catalogue gearbox writes beside its ratio: checked, not used (the gearboxes are rigid here).
        cfg.positive("gearbox.stiffness_nm_rad")?;
        cfg.non_negative("gearbox.backlash_rad")?;
        // The stretch of the roll the axis runs over: from where on the roll, how long.
        let (from, run) = (cfg.non_negative("roll.run_from_m")?, cfg.positive("roll.run_m")?);
        if from + run > roll.capacity() {
            return Err(ConfigError::Invalid("roll.run_m".into()));
        }
        let get = |what: &str, rate: &str| cfg.positive(&format!("axis.web.max_{what}_m_{rate}"));
        let rating = |key: &str| cfg.f64(key).unwrap_or(f64::INFINITY);
        Ok(Self {
            roll,
            ratio: cfg.positive("gearbox.ratio")?,
            // The teeth's efficiency (the catalogue's less what the drag takes, `Gearbox::configured_efficiency`); with
            // the temperature feedback that at ambient.
            efficiency: [thermal.as_ref().map_or(Gearbox::configured_efficiency(cfg)?, |t| t.gearbox_efficiency(0.0));
                2],
            drag: GearDrag::from_config(cfg)?,
            oil: [thermal.as_ref().map_or(1.0, |t| t.viscosity_factor(0.0)); 2],
            thermal,
            j_rotor: cfg.positive("motor.rotor_inertia_kgm2")? + cfg.non_negative("gearbox.inertia_at_input_kgm2")?,
            ea,
            width,
            damping_time: if on("web_damping")? { cfg.non_negative("web.damping_time_s")? } else { 0.0 },
            tension: cfg.positive("web.tension_n")?,
            taper: if on("tension_taper")? { taper } else { 0.0 },
            wound_strain: cfg.non_negative("roll.wound_tension_n")? / ea,
            spans: [cfg.positive("span.unwind_m")?, cfg.positive("span.rewind_m")?],
            dancer_mass: cfg.positive("mass.dancer_kg")?,
            payload: if on("payload")? { cfg.non_negative("mass.payload_kg")? } else { 0.0 },
            attached: true,
            roller_radius: cfg.positive("dancer.roller_radius_m")?,
            roller_inertia: cfg.positive("dancer.roller_inertia_kgm2")?,
            air_spring: cfg.non_negative("dancer.air_spring_n_m")?,
            stroke,
            slide_friction: Friction::from_config(
                cfg,
                on("dancer_friction")?,
                ["dancer.friction_coulomb_n", "dancer.friction_viscous_n_s_m", "dancer.friction_smoothing_m_s"],
            )?,
            roller_friction: Friction::from_config(
                cfg,
                on("dancer_friction")?,
                [
                    "dancer.roller_friction_coulomb_nm",
                    "dancer.roller_friction_viscous_nm_s_rad",
                    "dancer.roller_friction_smoothing_rad_s",
                ],
            )?,
            motor_friction: Friction::from_config(
                cfg,
                on("motor_friction")?,
                ["motor.friction_coulomb_nm", "motor.friction_viscous_nm_s_rad", "motor.friction_smoothing_rad_s"],
            )?,
            brake: HoldingBrake::from_config(
                cfg,
                cfg.positive("motor.rotor_inertia_kgm2")? + cfg.non_negative("gearbox.inertia_at_input_kgm2")?,
            )?,
            stop: if on("hard_stops")? {
                [cfg.positive("dancer.stop_stiffness_n_m")?, cfg.non_negative("dancer.stop_damping_n_s_m")?]
            } else {
                [0.0, 0.0]
            },
            g: if on("gravity")? { G } else { 0.0 },
            elastic: on("web_stretch")?,
            position_loop: cfg.bool("dancer.position_loop")?,
            trim_gains: [kp, tn, cfg.non_negative("dancer.trim_max_m_s")?],
            trim_length: 0.0,
            trim_integral: 0.0,
            trim_speed: 0.0,
            drives_off: false,
            limits: [
                rating("ratings.web_max_tension_n"),
                rating("ratings.web_min_tension_n"),
                rating("ratings.dancer_max_travel_m"),
                rating("ratings.max_inertia_ratio"),
                cfg.positive("drive.peak_torque_nm")?,
                cfg.positive("drive.rated_torque_nm")?,
                if cfg.get("drive.max_speed_rad_s").is_ok() {
                    cfg.positive("drive.max_speed_rad_s")?
                } else {
                    cfg.positive("drive.no_load_speed_rad_s")?
                },
            ],
            axes: [AxisInfo {
                id: "web",
                key: "axis-web",
                unit: "m",
                min: from,
                max: from + run,
                max_vel: get("velocity", "s")?,
                max_acc: get("acceleration", "s2")?,
                max_jerk: get("jerk", "s3")?,
            }],
            motors: [MotorInfo { id: "unwind", key: "motor-unwind" }, MotorInfo { id: "rewind", key: "motor-rewind" }],
        })
    }

    /// The dancer with what it carries.
    pub fn dancer_total_mass(&self) -> f64 {
        self.dancer_mass + if self.attached { self.payload } else { 0.0 }
    }

    /// Radii at the line position `l`: the unwind with the capacity less `l` on its core, the rewind with `l`.
    pub fn radii_at(&self, l: f64) -> [f64; 2] {
        let c = &self.roll;
        let h = c.thickness * l / PI;
        [(c.full * c.full - h).max(c.core * c.core).sqrt(), (c.core * c.core + h).max(0.0).sqrt()]
    }
    /// The roll angles at the line position `l` with the dancer in the middle: s = phi (r0 + r) / 2 solved for phi.
    pub fn angles_at(&self, l: f64) -> [f64; 2] {
        let [ru, rr] = self.radii_at(l);
        [2.0 * l / (self.roll.full + ru), 2.0 * l / (self.roll.core + rr)]
    }
    /// The unwind's radius at its angle (paid out from full) and the rewind's at its angle (wound from the core).
    pub fn unwind_radius(&self, phi: f64) -> f64 {
        (self.roll.full - self.roll.thickness * phi / TAU).max(self.roll.core)
    }
    pub fn rewind_radius(&self, phi: f64) -> f64 {
        (self.roll.core + self.roll.thickness * phi / TAU).max(self.roll.core)
    }
    /// The web the unwind has paid out at its angle, and the rewind wound up at its, m: phi (r0 + r) / 2.
    pub fn paid_out(&self, phi: f64) -> f64 {
        0.5 * phi * (self.roll.full + self.unwind_radius(phi))
    }
    pub fn wound_up(&self, phi: f64) -> f64 {
        0.5 * phi * (self.roll.core + self.rewind_radius(phi))
    }
    /// A roll's inertia at radius `r` as its motor turns it, at the roll: roll plus i^2 J_rotor.
    pub fn roll_inertia(&self, r: f64) -> f64 {
        self.roll.inertia(r) + self.ratio * self.ratio * self.j_rotor
    }

    /// The tension setpoint at the rewind's radius: T0 (1 - k (1 - r_c / r)).
    pub fn tension_setpoint(&self, r_rewind: f64) -> f64 {
        self.tension * (1.0 - self.taper * (1.0 - self.roll.core / r_rewind.max(self.roll.core)))
    }
    /// The cylinder's force setpoint for that tension: 2 T - m g, the dancer's weight taken off.
    pub fn cylinder_force(&self, r_rewind: f64) -> f64 {
        2.0 * self.tension_setpoint(r_rewind) - self.dancer_total_mass() * self.g
    }

    /// The stops' force on the dancer along y, their stored energy and their loss power.
    fn stop(&self, y: f64, vy: f64) -> [f64; 3] {
        let [k, c] = self.stop;
        let depth = y.abs() - self.stroke;
        if k == 0.0 || depth <= 0.0 {
            return [0.0; 3];
        }
        let (side, rate) = (y.signum(), y.signum() * vy);
        let push = (k * depth + c * rate).max(0.0);
        [-side * push, 0.5 * k * depth * depth, (push - k * depth) * rate]
    }

    /// The motor torque that turns roll `k` (0 unwind, 1 rewind) at radius `r` so that its surface runs at `v` and
    /// speeds up by `a`, against the web's tension `tension`: tau = (J(r) w' -+ T r) / i, w' = (a -+ h w^2 / (2 pi)) / r
    /// (the radius shrinks on the unwind, grows on the rewind, so at a steady line speed the unwind speeds up).
    pub fn roll_torque(&self, k: usize, r: f64, v: f64, a: f64, tension: f64) -> f64 {
        let w = v / r;
        let (sign, growth) = if k == 0 { (-1.0, -1.0) } else { (1.0, 1.0) };
        let acc = (a - growth * self.roll.thickness * w * w / TAU) / r;
        (self.roll_inertia(r) * acc + sign * tension * r) / self.ratio
    }

    /// The two spans' tensions of the rigid model at rest for the line's acceleration `a`: the roller's inertia takes
    /// J_d a / R^2 between them, the dancer at rest their sum 2 T_set: T_set -+ J_d a / (2 R^2).
    pub fn rigid_span_tensions(&self, l: f64, a: f64) -> [f64; 2] {
        let t = self.tension_setpoint(self.radii_at(l)[1]);
        let d = self.roller_inertia * a / (2.0 * self.roller_radius * self.roller_radius);
        [t - d, t + d]
    }

    /// Everything of a state (see `Eval`).
    pub fn eval(&self, s: &[f64], torque: &[f64]) -> Eval {
        let (h, i, rd) = (self.roll.thickness, self.ratio, self.roller_radius);
        let mut e = Eval::default();
        let (phi, w) = if self.elastic { ([s[0], s[1]], [s[3], s[4]]) } else { ([s[0], s[1]], [s[2], s[3]]) };
        e.r = [self.unwind_radius(phi[0]), self.rewind_radius(phi[1])];
        e.w = w;
        e.v = [e.r[0] * w[0], e.r[1] * w[1]];
        e.j = e.r.map(|r| self.roll_inertia(r));
        if self.elastic {
            (e.y, e.vy, e.wd) = (s[2], s[5], s[6]);
        } else {
            e.y = 0.5 * (self.paid_out(phi[0]) - self.wound_up(phi[1]));
            e.vy = 0.5 * (e.v[0] - e.v[1]);
            e.wd = 0.5 * (e.v[0] + e.v[1]) / rd;
        }
        // The shaft's friction and, while the drives are off, its holding brake on its deflection.
        let first_brake = if self.elastic { N_ELASTIC - 2 } else { N_RIGID - 2 };
        for k in 0..2 {
            let (brake, rate) = self.brake.torque(self.drives_off, i * w[k], s[first_brake + k]);
            e.drag[k] = self.drag.torque(i * w[k], self.oil[k]);
            (e.shaft[k], e.brake_rate[k]) = (self.motor_friction.at(i * w[k]) + brake + e.drag[k], rate);
        }
        let drive: [f64; 2] = std::array::from_fn(|k| i * (torque[k] - e.shaft[k]));
        e.slide = self.slide_friction.at(e.vy);
        e.roller = self.roller_friction.at(e.wd);
        e.stop = self.stop(e.y, e.vy);
        e.cylinder = self.cylinder_force(e.r[1]);
        e.load = e.cylinder - self.air_spring * e.y;
        let m = self.dancer_total_mass();
        // What pushes the dancer down but the web.
        let push = e.load + m * self.g - e.slide + e.stop[0];
        if self.elastic {
            let vd = rd * e.wd;
            for k in 0..2 {
                e.length[k] = self.spans[k] + e.y;
                e.material[k] = s[7 + k];
                e.strain[k] = e.length[k] / e.material[k] - 1.0;
            }
            let taut = e.strain.map(|x| x.max(0.0));
            // The web crossing each boundary at the strain of the side it comes from.
            let back = self.tension_setpoint(e.r[1]) / self.ea;
            let flow = |v: f64, ahead: f64, behind: f64| v / (1.0 + if v >= 0.0 { ahead } else { behind });
            let q = [flow(e.v[0], self.wound_strain, taut[0]), flow(vd, taut[0], taut[1]), flow(e.v[1], taut[1], back)];
            e.material_rate = [q[0] - q[1], q[1] - q[2]];
            e.flow = q;
            for k in 0..2 {
                let rate = (e.vy - (1.0 + e.strain[k]) * e.material_rate[k]) / e.material[k];
                if e.strain[k] > 0.0 {
                    e.spring[k] = self.ea * e.strain[k];
                    e.t[k] = (e.spring[k] + self.ea * self.damping_time * rate).max(0.0);
                }
            }
            let web = [e.t[0] * e.r[0], -e.t[1] * e.r[1]];
            // The gearbox's loss factor of the torque going in, which depends on the roll's acceleration it gives, at its
            // fixed point (`mech::settled_loss`), then the roll with it.
            for k in 0..2 {
                let acc = |factor: f64| (factor * drive[k] + web[k]) / self.geared_inertia(e.r[k], factor);
                [e.factor[k]] = sim_components::mech::settled_loss([1.0], |&[factor]| {
                    [self.loss_factor(k, torque[k] - e.shaft[k] - self.j_rotor * i * acc(factor), w[k])]
                });
                e.acc[k] = acc(e.factor[k]);
            }
            e.ydd = (push - e.t[0] - e.t[1]) / m;
            e.wdd = (rd * (e.t[1] - e.t[0]) - e.roller) / self.roller_inertia;
        } else {
            // a_u = b_u + k_u T1, a_r = b_r - k_r T2 (the surfaces' accelerations), y'' = (a_u - a_r) / 2 and
            // R w_d' = (a_u + a_r) / 2 into the dancer's and the roller's balance: two equations in T1 and T2. Solved
            // with the gearboxes' loss factors at the torques the solution passes (to the fixed point, from loss-free).
            let rate = [-h * w[0] * w[0] / TAU, h * w[1] * w[1] / TAU];
            let (md, jd) = (0.5 * m, self.roller_inertia / (2.0 * rd * rd));
            let solve = |factor: [f64; 2]| {
                let j: [f64; 2] = std::array::from_fn(|k| self.geared_inertia(e.r[k], factor[k]));
                let kk = [e.r[0] * e.r[0] / j[0], e.r[1] * e.r[1] / j[1]];
                let b =
                    [e.r[0] * factor[0] * drive[0] / j[0] + rate[0], e.r[1] * factor[1] * drive[1] / j[1] + rate[1]];
                let (a11, a12, b1) = (1.0 + md * kk[0], 1.0 + md * kk[1], push - md * (b[0] - b[1]));
                let (a21, a22, b2) = (1.0 + jd * kk[0], -(1.0 + jd * kk[1]), -e.roller / rd - jd * (b[0] + b[1]));
                let det = a11 * a22 - a12 * a21;
                let t = [(b1 * a22 - a12 * b2) / det, (a11 * b2 - a21 * b1) / det];
                let acc =
                    [(factor[0] * drive[0] + t[0] * e.r[0]) / j[0], (factor[1] * drive[1] - t[1] * e.r[1]) / j[1]];
                let surface = [b[0] + kk[0] * t[0], b[1] - kk[1] * t[1]];
                (t, acc, surface)
            };
            // The loss factors at their fixed point, as above.
            e.factor = sim_components::mech::settled_loss([1.0; 2], |factor| {
                let (_, acc, _) = solve(*factor);
                std::array::from_fn(|k| self.loss_factor(k, torque[k] - e.shaft[k] - self.j_rotor * i * acc[k], w[k]))
            });
            let (t, acc, surface) = solve(e.factor);
            (e.t, e.spring, e.acc) = (t, t, acc);
            e.ydd = 0.5 * (surface[0] - surface[1]);
            e.wdd = 0.5 * (surface[0] + surface[1]) / rd;
        }
        e.gear_input = std::array::from_fn(|k| torque[k] - e.shaft[k] - self.j_rotor * i * e.acc[k]);
        e
    }

    /// Gearbox `k`'s factor on the torque `input` at its input while the roll turns at `w`: its efficiency while the
    /// motor drives, 1 / efficiency while the roll drives the motor back, faded smoothly through zero power
    /// (eta^tanh(P / 0.5 W)), as the other machines' gearboxes (`mech::Gearbox::loss_factor`).
    fn loss_factor(&self, k: usize, input: f64, w: f64) -> f64 {
        if self.efficiency[k] == 1.0 {
            return 1.0;
        }
        self.efficiency[k].powf((input * self.ratio * w / EFFICIENCY_FADE_W).tanh())
    }
    /// What the roll's angular acceleration meets with the gearbox's loss factor `factor`: J_roll + factor i^2 J_m, the
    /// rotor's own inertia seen through the gearbox as its torque is.
    fn geared_inertia(&self, r: f64, factor: f64) -> f64 {
        self.roll.inertia(r) + factor * self.ratio * self.ratio * self.j_rotor
    }

    /// The line's speed and acceleration: the rewind's surface, d(r w)/dt = r w' + (h w / 2 pi) w.
    fn line(&self, e: &Eval) -> (f64, f64) {
        let w = e.w[1];
        (e.v[1], e.r[1] * e.acc[1] + self.roll.thickness * w * w / TAU)
    }

    /// Kinetic, potential and elastic energy (see `ENERGY_PARTS`): rolls with rotors, dancer and roller; the dancer's
    /// weight and the cylinder (its force setpoint as it is now, and the air spring); the web's spans and the stops.
    fn energy_parts(&self, e: &Eval) -> [f64; 3] {
        let m = self.dancer_total_mass();
        let kinetic = 0.5 * (e.j[0] * e.w[0] * e.w[0] + e.j[1] * e.w[1] * e.w[1])
            + 0.5 * m * e.vy * e.vy
            + 0.5 * self.roller_inertia * e.wd * e.wd;
        let potential = -(m * self.g + e.cylinder) * e.y + 0.5 * self.air_spring * e.y * e.y;
        let web: f64 = (0..2)
            .filter(|&k| self.elastic && e.length[k] > e.material[k])
            .map(|k| self.ea * (e.length[k] - e.material[k]).powi(2) / (2.0 * e.material[k]))
            .sum();
        [kinetic, potential, web + e.stop[1]]
    }

    /// The loss powers in the order of `effect_infos`: each motor's shaft, the dancer's slide and roller, the web's
    /// damping, the stops, what the web carries in and out of the machine's energy, the cylinder's taper.
    ///
    /// The web's transport is the balance of what the tensions do on the bodies, what the spans store and what the
    /// damping takes: P = -(1/2 J_u' w_u^2 + 1/2 J_r' w_r^2) - P_bodies - dU/dt - P_damping, which is the kinetic energy
    /// the web carries from roll to roll (1/2 lambda (v_u^3 - v_r^3)) and the strain energy it brings into the spans and
    /// takes out of them (T v (eps_in - eps) to first order, nothing where the strains are equal). The cylinder's power
    /// is dF/dt y: the setpoint changes with the rewind's radius, and the potential with it.
    fn losses(&self, e: &Eval) -> [f64; 9] {
        let (i, rd, h) = (self.ratio, self.roller_radius, self.roll.thickness);
        // The gearbox's drag acts on the rotor with the shaft's friction, but its loss is the gearbox's.
        let shaft = [0, 1].map(|k| (e.shaft[k] - e.drag[k]) * i * e.w[k]);
        // What goes into a gearbox less what comes out: (1 - factor) times the input's power, either way positive.
        let gearbox = [0, 1].map(|k| ((1.0 - e.factor[k]) * e.gear_input[k] + e.drag[k]) * i * e.w[k]);
        let dancer = e.slide * e.vy + e.roller * e.wd;
        let growth = [-h * e.w[0] / TAU, h * e.w[1] / TAU];
        let mass_flow: f64 = (0..2).map(|k| 0.5 * self.roll.inertia_slope(e.r[k]) * growth[k] * e.w[k] * e.w[k]).sum();
        let bodies = e.t[0] * e.v[0] - e.t[1] * e.v[1] + rd * e.wd * (e.t[1] - e.t[0]) - (e.t[0] + e.t[1]) * e.vy;
        let (mut stored, mut damping) = (0.0, 0.0);
        if self.elastic {
            for k in 0..2 {
                stored += e.spring[k] * (e.vy - (1.0 + 0.5 * e.strain[k]) * e.material_rate[k]);
                damping += (e.t[k] - e.spring[k]) * (e.vy - (1.0 + e.strain[k]) * e.material_rate[k]);
            }
        }
        let r = e.r[1].max(self.roll.core);
        let slope = -2.0 * self.tension * self.taper * self.roll.core / (r * r);
        let cylinder = if e.r[1] > self.roll.core { slope * growth[1] * e.y } else { 0.0 };
        [
            shaft[0],
            shaft[1],
            gearbox[0],
            gearbox[1],
            dancer,
            damping,
            e.stop[2],
            -mass_flow - bodies - stored - damping,
            cylinder,
        ]
    }

    /// The radii a roll has at its two ends: [full, core].
    fn ends(&self) -> [f64; 2] {
        [self.roll.full, self.roll.core]
    }

    /// The inertia ratio at roll radius `r`: what the motor sees of roll, core and shaft over rotor and gearbox input.
    pub fn inertia_ratio(&self, r: f64) -> f64 {
        self.roll.inertia(r) / (self.ratio * self.ratio * self.j_rotor)
    }

    /// The figures the own verdicts judge, from the line's speed `v`, its acceleration `a` and the spans' tensions `t`
    /// at the rewind's radius `r_rewind`: per motor its torque at the full roll and at the core (`roll_torque`), its
    /// speed at the core i v / r_c, the tension's torque at the full roll T r_f / i, and the inertia ratio at the full
    /// roll. The tension at a roll's end is the setpoint there (with the taper: the rewind at its core while the unwind is
    /// full, and full while the unwind is on its core) plus what the span's tension is off its setpoint now.
    pub fn end_figures(&self, v: f64, a: f64, t: [f64; 2], r_rewind: f64) -> [[f64; 5]; 2] {
        let now = self.tension_setpoint(r_rewind);
        let [full, core] = self.ends();
        std::array::from_fn(|k| {
            // The rewind's radius when roll k is full and when it is on its core.
            let rewind = if k == 0 { [core, full] } else { [full, core] };
            let tension = rewind.map(|r| self.tension_setpoint(r) + t[k] - now);
            [
                self.roll_torque(k, full, v, a, tension[0]),
                self.roll_torque(k, core, v, a, tension[1]),
                self.ratio * v / core,
                tension[0] * full / self.ratio,
                self.inertia_ratio(full),
            ]
        })
    }

    /// The lowest tension's shortfall: the web's minimum tension over the lower span's (above 1: below the minimum; a
    /// slack span counts as 1e9).
    fn slack_ratio(&self, t: [f64; 2]) -> f64 {
        let low = t[0].min(t[1]);
        if low > 0.0 { (self.limits[1] / low).min(1e9) } else { 1e9 }
    }

    /// Every own check, whether or not the config rates it (`rating_checks` leaves out those it does not): the web's
    /// highest tension, its lowest (as the minimum over it, at most 1), the dancer's travel, and per motor the inertia
    /// ratio at the full roll, its torque at the full roll and at the core against its peak torque, its speed at the core
    /// against its highest speed, and the tension's torque at the full roll against its rated torque.
    fn all_checks(&self) -> Vec<RatingCheck> {
        let check = |check: &'static str, signal: String, motor: Option<usize>, limit: f64| RatingCheck {
            check,
            signal,
            motor,
            limit,
            life: false,
        };
        let [tension, minimum, travel, ratio, peak, rated, speed] = self.limits;
        let mut out = vec![
            check("web-tension", "web.tension_peak".into(), None, tension),
            check("web-slack", "web.slack_ratio".into(), None, if minimum.is_finite() { 1.0 } else { minimum }),
            check("dancer-travel", "dancer.position".into(), None, travel),
        ];
        for (k, roll) in ROLLS.iter().enumerate() {
            out.extend([
                check("inertia-ratio", format!("{roll}.inertia_ratio_at_full"), Some(k), ratio),
                check("torque-at-full-roll", format!("{roll}.torque_at_full"), Some(k), peak),
                check("torque-at-core", format!("{roll}.torque_at_core"), Some(k), peak),
                check("speed-at-core", format!("{roll}.speed_at_core"), Some(k), speed),
                check("tension-torque-at-full-roll", format!("{roll}.tension_torque_at_full"), Some(k), rated),
            ]);
        }
        out
    }

    /// The dancer at its travel `y`, as `scene` draws it: its roller, its slide, the weight on it while it carries one, and
    /// its cylinder below.
    fn dancer_parts(&self, y: f64) -> Vec<Part> {
        let (width, rd) = (self.width, self.roller_radius);
        let depth = 0.5 * (self.spans[0] + self.spans[1]) + y;
        let mut parts = Vec::with_capacity(4);
        if self.attached && self.payload > 0.0 {
            parts.push(Part::new(
                "payload",
                Shape::Box { size: [0.08, 0.05, 0.05] },
                [0.0, 0.5 * width + 0.04, -depth + 0.09],
                [0.2, 0.7, 0.3],
            ));
        }
        parts.extend([
            Part::new(
                "dancer-roller",
                Shape::Cylinder { radius: rd, length: width + 0.02 },
                [0.0, 0.0, -depth],
                [0.62, 0.64, 0.68],
            ),
            Part::new(
                "dancer-slide",
                Shape::Box { size: [0.06, 0.04, 0.12] },
                [0.0, 0.5 * width + 0.04, -depth],
                [0.2, 0.22, 0.26],
            ),
            Part::new(
                "dancer-cylinder",
                Shape::Segment { to: [0.0, 0.5 * width + 0.04, -depth - 0.35], width: 0.05 },
                [0.0, 0.5 * width + 0.04, -depth - 0.06],
                [0.3, 0.45, 0.7],
            ),
        ]);
        parts
    }

    /// The signal list of `effect_signals`.
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
        let gearbox = if self.ratio == 1.0 { "quantity-coupling-loss-power" } else { "quantity-gearbox-loss-power" };
        let powers = [
            ("shaft_unwind", "motor-unwind", "quantity-shaft-friction-power"),
            ("shaft_rewind", "motor-rewind", "quantity-shaft-friction-power"),
            ("gearbox_unwind", "motor-unwind", gearbox),
            ("gearbox_rewind", "motor-rewind", gearbox),
            ("dancer", "group-dancer", "quantity-friction-power"),
            ("web_damping", "group-web", "quantity-damping-power"),
            ("dancer_stops", "group-dancer", "quantity-damping-power"),
            ("web_transport", "group-web", "quantity-web-transport-power"),
            ("cylinder", "group-dancer", "quantity-cylinder-power"),
        ];
        out.extend(
            powers.map(|(id, owner, quantity)| SignalInfo::new(format!("{LOSS_PREFIX}{id}"), owner, quantity, "W")),
        );
        out.extend(ENERGY_PARTS.map(|part| {
            SignalInfo::new(format!("energy.{part}"), "group-energy", &format!("quantity-energy-{part}"), "J")
        }));
    }
}

impl Machine for Winder {
    fn axes(&self) -> &[AxisInfo] {
        &self.axes
    }
    fn motors(&self) -> &[MotorInfo] {
        &self.motors
    }

    /// The line position's roll angles through the ratio, each at the radius of its wound length: the rewind's the line
    /// position, the unwind's the line position plus the web the dancer's trim has had it pay out more.
    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        motors[0] = self.ratio * self.angles_at(tool[0] + self.trim_length)[0];
        motors[1] = self.ratio * self.angles_at(tool[0])[1];
    }
    /// The line position from the motors' angles, as `tool_state` takes it from the rolls'.
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        let (u, r) = (motors[0] / self.ratio, motors[1] / self.ratio);
        tool[0] = 0.5 * (self.paid_out(u) - self.trim_length + self.wound_up(r));
    }

    fn state_len(&self) -> usize {
        if self.elastic { N_ELASTIC } else { N_RIGID }
    }

    /// At rest at the line position, the dancer in the middle and each span at the tension setpoint: M = L / (1 + T / EA);
    /// the motors hold the tension's torque, -T r_u / i on the unwind and T r_r / i on the rewind.
    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        let l = tool[0];
        let ([u, r], [ru, rr]) = (self.angles_at(l), self.radii_at(l));
        let t = self.tension_setpoint(rr);
        *state = if self.elastic {
            let m = self.spans.map(|span| span / (1.0 + t / self.ea));
            vec![u, r, 0.0, 0.0, 0.0, 0.0, 0.0, m[0], m[1], 0.0, 0.0, 0.0]
        } else {
            vec![u, r, 0.0, 0.0, 0.0, 0.0]
        };
        torque[0] = -t * ru / self.ratio;
        torque[1] = t * rr / self.ratio;
    }

    fn derivatives(&self, s: &[f64], torque: &[f64], out: &mut [f64]) {
        let e = self.eval(s, torque);
        if self.elastic {
            let slack = if e.t.iter().any(|t| *t <= 0.0) { 1.0 } else { 0.0 };
            out.copy_from_slice(&[
                e.w[0],
                e.w[1],
                e.vy,
                e.acc[0],
                e.acc[1],
                e.ydd,
                e.wdd,
                e.material_rate[0],
                e.material_rate[1],
                slack,
                e.brake_rate[0],
                e.brake_rate[1],
            ]);
        } else {
            out.copy_from_slice(&[e.w[0], e.w[1], e.acc[0], e.acc[1], e.brake_rate[0], e.brake_rate[1]]);
        }
    }

    /// The line position the rolls stand at: the mean of the web the unwind has paid out, less what the dancer's trim had
    /// it pay out more, and the web the rewind has wound up; with the dancer in the middle both. In the rigid model the
    /// web's travel at the dancer roller, (s_u + s_r) / 2.
    fn tool_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let w = if self.elastic { [s[3], s[4]] } else { [s[2], s[3]] };
        pos[0] = 0.5 * (self.paid_out(s[0]) - self.trim_length + self.wound_up(s[1]));
        vel[0] = 0.5 * (self.unwind_radius(s[0]) * w[0] - self.trim_speed + self.rewind_radius(s[1]) * w[1]);
    }
    fn set_tool_velocity(&self, s: &mut [f64], vel: &[f64]) {
        let (phi, v) = ([s[0], s[1]], vel[0]);
        let w = [v / self.unwind_radius(phi[0]), v / self.rewind_radius(phi[1])];
        if self.elastic {
            (s[3], s[4], s[5], s[6]) = (w[0], w[1], 0.0, v / self.roller_radius);
        } else {
            (s[2], s[3]) = (w[0], w[1]);
        }
    }
    fn motor_state(&self, s: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        let w = if self.elastic { [s[3], s[4]] } else { [s[2], s[3]] };
        for k in 0..2 {
            pos[k] = self.ratio * s[k];
            vel[k] = self.ratio * w[k];
        }
    }

    fn feedforward_torque(&self, pos: &[f64], acc: &[f64], out: &mut [f64]) {
        self.feedforward_torque_moving(pos, &[0.0], acc, out);
    }
    /// The tension's torque and the inertia compensation at the radii of the line position (`roll_torque`), the
    /// spans' tensions those of the rigid model at rest (`rigid_span_tensions`).
    fn feedforward_torque_moving(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut [f64]) {
        let (r, t) = (self.radii_at(pos[0]), self.rigid_span_tensions(pos[0], acc[0]));
        for k in 0..2 {
            out[k] = self.roll_torque(k, r[k], vel[0], acc[0], t[k]);
        }
    }
    /// The rigid model's inverse dynamics at motor angles, speeds and accelerations of their own: the dancer then moves
    /// at half the surfaces' difference, and the two tensions follow from its balance and the roller's.
    fn rigid_motor_torque(&self, angle: &[f64], speed: &[f64], acc: &[f64], out: &mut [f64]) {
        let (i, h, rd) = (self.ratio, self.roll.thickness, self.roller_radius);
        let phi = [angle[0] / i, angle[1] / i];
        let (w, wdot) = ([speed[0] / i, speed[1] / i], [acc[0] / i, acc[1] / i]);
        let r = [self.unwind_radius(phi[0]), self.rewind_radius(phi[1])];
        let v = [r[0] * w[0], r[1] * w[1]];
        let a = [r[0] * wdot[0] - h * w[0] * w[0] / TAU, r[1] * wdot[1] + h * w[1] * w[1] / TAU];
        let y = 0.5 * (self.paid_out(phi[0]) - self.wound_up(phi[1]));
        let vy = 0.5 * (v[0] - v[1]);
        let m = self.dancer_total_mass();
        let push = self.cylinder_force(r[1]) - self.air_spring * y + m * self.g + self.stop(y, vy)[0];
        let sum = push - m * 0.5 * (a[0] - a[1]);
        let diff = self.roller_inertia * 0.5 * (a[0] + a[1]) / (rd * rd);
        let t = [0.5 * (sum - diff), 0.5 * (sum + diff)];
        out[0] = (self.roll_inertia(r[0]) * wdot[0] - t[0] * r[0]) / i;
        out[1] = (self.roll_inertia(r[1]) * wdot[1] + t[1] * r[1]) / i;
    }

    /// The motors' shaft friction and the gearboxes' drag at their speeds, and the dancer roller's drag through the web:
    /// it takes R (T2 - T1),
    /// so T1 = T - drag / (2 R) and T2 = T + drag / (2 R), r / i of each at the motors.
    fn steady_friction(&self, pos: &[f64], vel: &[f64], out: &mut [f64]) {
        let (r, v, rd) = (self.radii_at(pos[0]), vel[0], self.roller_radius);
        let drag = self.roller_friction.at(v / rd);
        for k in 0..2 {
            let speed = self.ratio * v / r[k];
            out[k] = self.motor_friction.at(speed)
                + self.drag.torque(speed, self.oil[k])
                + drag * r[k] / (2.0 * rd * self.ratio);
        }
    }
    /// The roller's drag of `steady_friction` as Coulomb and viscous parts; the motors' shafts are their own.
    fn rigid_friction(&self, pos: &[f64], dir: &[f64], out: &mut Vec<[f64; 2]>) {
        let (r, rd) = (self.radii_at(pos[0]), self.roller_radius);
        let [c, b, _] = self.roller_friction.0;
        for rk in r {
            let share = rk / (2.0 * rd * self.ratio);
            out.push([c * dir[0].signum() * share, b * dir[0] / rd * share]);
        }
    }
    /// The own checks in the rigid model (`rating_checks`): the spans' tensions of `rigid_span_tensions`, the figures
    /// at the rolls' ends, and no dancer travel (the rigid model holds it in the middle): NaN.
    fn rigid_checks(&self, pos: &[f64], vel: &[f64], acc: &[f64], out: &mut Vec<f64>) {
        let t = self.rigid_span_tensions(pos[0], acc[0]);
        let mut all = vec![t[0].max(t[1]), self.slack_ratio(t), f64::NAN];
        for f in self.end_figures(vel[0], acc[0], t, self.radii_at(pos[0])[1]) {
            all.extend([f[4], f[0], f[1], f[2], f[3]]);
        }
        // In the order of `rating_checks`, which leaves out a check whose rating the config has none of.
        out.extend(self.all_checks().iter().zip(all).filter(|(c, _)| c.limit.is_finite()).map(|(_, v)| v));
    }

    fn energy(&self, s: &[f64]) -> f64 {
        self.energy_parts(&self.eval(s, &[0.0, 0.0])).iter().sum()
    }

    /// What moves and can hit an obstacle: the dancer's slide and the weight on it, as `scene` draws them (the rolls turn
    /// in place).
    fn collision_bodies(&self, s: &[f64]) -> Vec<sim_core::Aabb> {
        let y = self.eval(s, &[0.0, 0.0]).y;
        sim_core::scene::scene_boxes(&self.dancer_parts(y), &["dancer-slide", "payload"])
    }

    fn set_payload_attached(&mut self, attached: bool) {
        self.attached = attached;
    }
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.drives_off = engaged;
    }

    /// The dancer's position loop, every base step while the drives are on: the trim v = -(Kp e + integral), e the
    /// dancer's travel from the middle (down: too much web between the rolls, so the unwind slows), held to the most it
    /// may trim, the integrator Kp / Tn e (held while the trim is at its limit); the unwind's setpoint moves on by
    /// i v / r_u. While the drives are off the trim tracks what the rolls stand at: the web the unwind paid out more than
    /// the rewind took up (the dancer's travel, two spans of it), so that enabled again both motors' setpoints are where
    /// they stand. Before 15 November 2026 a re-enable after a coast set both rolls to the mean of the two, yanked them
    /// by the dancer's travel and tripped a weak DC link again.
    fn after_step(&mut self, s: &mut [f64], dt: f64) {
        if self.drives_off {
            (self.trim_speed, self.trim_integral) = (0.0, 0.0);
            self.trim_length = self.paid_out(s[0]) - self.wound_up(s[1]);
            return;
        }
        if !self.position_loop {
            self.trim_speed = 0.0;
            return;
        }
        let e = self.eval(s, &[0.0, 0.0]);
        let [kp, tn, most] = self.trim_gains;
        let wanted = -(kp * e.y + self.trim_integral);
        self.trim_speed = wanted.clamp(-most, most);
        if self.trim_speed == wanted {
            self.trim_integral += kp / tn * e.y * dt;
        }
        self.trim_length += self.trim_speed * dt;
    }
    fn memory(&self) -> Vec<f64> {
        vec![self.trim_length, self.trim_integral, self.trim_speed]
    }
    fn set_memory(&mut self, memory: &[f64]) {
        (self.trim_length, self.trim_integral, self.trim_speed) = (memory[0], memory[1], memory[2]);
    }

    /// The dancer on the web with the rolls held: the two spans and the air spring, k = EA / L1 + EA / L2 + k_air, on the
    /// dancer and what it carries, f = sqrt(k / m) / (2 pi), damped by the web's material damping (EA tau / L each) and
    /// the slide's viscous friction; and the roller turning on the spans, one stretching as the other gives,
    /// k = R^2 (EA / L1 + EA / L2) on J_d. The dancer in the middle; none in the rigid model.
    fn modes(&self, _tool_pose: &[f64]) -> Vec<Mode> {
        if !self.elastic {
            return Vec::new();
        }
        let web = self.ea / self.spans[0] + self.ea / self.spans[1];
        let m = self.dancer_total_mass();
        let (k, c) = (web + self.air_spring, web * self.damping_time + self.slide_friction.0[1]);
        let w = (k / m).sqrt();
        let rd = self.roller_radius;
        let (kr, cr) = (rd * rd * web, rd * rd * web * self.damping_time + self.roller_friction.0[1]);
        let wr = (kr / self.roller_inertia).sqrt();
        let mut modes = vec![
            Mode { hz: w / TAU, damping: c / (2.0 * m * w) },
            Mode { hz: wr / TAU, damping: cr / (2.0 * self.roller_inertia * wr) },
        ];
        modes.sort_by(|a, b| a.hz.total_cmp(&b.hz));
        modes
    }
    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        self.modes(tool_pose).first().map(|m| m.hz)
    }

    /// In the frame of the scene, the rolls' axes along y at z = 0: the dancer and what it carries moving along z (down
    /// as y grows), the rolls with their rotors and the roller spinning about y. The web's weight moving from one roll
    /// to the other over the run is left out (minutes, not a vibration).
    fn frame_momentum(&self, s: &[f64]) -> Option<FrameMomentum> {
        use sim_components::base::{add_point, add_spin};
        let e = self.eval(s, &[0.0, 0.0]);
        let mut f = FrameMomentum { gravity: [0.0, 0.0, -self.g], ..Default::default() };
        let depth = 0.5 * (self.spans[0] + self.spans[1]);
        add_point(&mut f, self.dancer_total_mass(), [0.0, 0.0, -depth - e.y], [0.0, 0.0, -e.vy]);
        let rotor = self.ratio * self.ratio * self.j_rotor;
        // Both rolls turn clockwise seen from the front (about -y) as the line runs forward; the roller the other way.
        for k in 0..2 {
            add_spin(&mut f, [0.0, -1.0, 0.0], e.j[k] - rotor + self.ratio * self.j_rotor, e.w[k]);
        }
        add_spin(&mut f, [0.0, 1.0, 0.0], self.roller_inertia, e.wd);
        Some(f)
    }

    fn rating_checks(&self) -> Vec<RatingCheck> {
        self.all_checks().into_iter().filter(|c| c.limit.is_finite()).collect()
    }

    /// With effects.thermal_feedback: each gearbox's efficiency and its lubricant's viscosity (its drag) at its rise, in the
    /// order of `thermal_parts`.
    fn set_part_temperatures(&mut self, rises: &[f64]) {
        let Some(law) = &self.thermal else { return };
        for (k, rise) in rises.iter().enumerate().take(2) {
            (self.efficiency[k], self.oil[k]) = (law.gearbox_efficiency(*rise), law.viscosity_factor(*rise));
        }
    }

    /// The two gearboxes, which warm with their losses (`power.gearbox_<motor>`) with effects.part_heating or
    /// effects.thermal_feedback; none where the ratio is 1 (a coupling).
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        if self.ratio == 1.0 {
            return Vec::new();
        }
        [("gearbox_unwind", "motor-unwind", 0), ("gearbox_rewind", "motor-rewind", 1)]
            .map(|(id, owner_key, scene_index)| ThermalPart {
                id,
                kind: "gearbox",
                owner_key,
                scene_part: "gearbox",
                scene_index,
            })
            .into()
    }

    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out: Vec<SignalInfo> = [
            ("web.span.1.tension", "web-span-1", "quantity-tension", "N"),
            ("web.span.2.tension", "web-span-2", "quantity-tension", "N"),
            ("web.tension_peak", "group-web", "quantity-tension-peak", "N"),
            ("web.slack_ratio", "group-web", "quantity-slack-ratio", ""),
            ("web.tension_setpoint", "group-web", "quantity-tension-setpoint", "N"),
            ("web.slack_time", "group-web", "quantity-web-slack-time", "s"),
            ("dancer.position", "group-dancer", "quantity-position", "m"),
            ("dancer.speed", "group-dancer", "quantity-speed", "m/s"),
            ("dancer.load_force", "group-dancer", "quantity-cylinder-force", "N"),
            ("dancer.trim_speed", "group-dancer", "quantity-trim-speed", "m/s"),
            ("rewind.radius_from_speeds", "motor-rewind", "quantity-radius-from-speeds", "m"),
            ("gearbox.unwind.output_torque", "motor-unwind", "quantity-gearbox-torque", "N m"),
            ("gearbox.rewind.output_torque", "motor-rewind", "quantity-gearbox-torque", "N m"),
        ]
        .into_iter()
        .map(|(id, owner, quantity, unit)| SignalInfo::new(id, owner, quantity, unit))
        .collect();
        for (roll, motor) in ROLLS.iter().zip(&self.motors) {
            for (what, quantity, unit) in [
                ("radius", "quantity-radius", "m"),
                ("inertia_ratio", "quantity-inertia-ratio", ""),
                ("inertia_ratio_at_full", "quantity-inertia-ratio-at-full", ""),
                ("torque_at_full", "quantity-torque-at-full", "N m"),
                ("torque_at_core", "quantity-torque-at-core", "N m"),
                ("speed_at_core", "quantity-speed-at-core", "rad/s"),
                ("tension_torque_at_full", "quantity-tension-torque-at-full", "N m"),
            ] {
                out.push(SignalInfo::new(format!("{roll}.{what}"), motor.key, quantity, unit));
            }
        }
        self.effect_infos(&mut out);
        out
    }

    fn base_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let e = self.eval(s, torque);
        let (v, a) = self.line(&e);
        // The rewind's radius as a drive works it out from the speeds: the unwind's radius times its speed over the
        // rewind's (equal surface speeds), where the rewind turns.
        let from_speeds = if e.w[1].abs() > 1e-9 { e.r[0] * e.w[0] / e.w[1] } else { e.r[1] };
        out.extend([
            e.t[0],
            e.t[1],
            e.t[0].max(e.t[1]),
            self.slack_ratio(e.t),
            self.tension_setpoint(e.r[1]),
            if self.elastic { s[9] } else { 0.0 },
            e.y,
            e.vy,
            e.load,
            self.trim_speed,
            from_speeds,
        ]);
        // What each gearbox passes on to its roll: its factor times the ratio times the torque at its input.
        out.extend((0..2).map(|k| e.factor[k] * self.ratio * e.gear_input[k]));
        let figures = self.end_figures(v, a, e.t, e.r[1]);
        for k in 0..2 {
            let [full, core, speed, tension, ratio] = figures[k];
            out.extend([e.r[k], self.inertia_ratio(e.r[k]), ratio, full, core, speed, tension]);
        }
    }

    /// Per motor: inertia J w' / i (roll, core and rotor), no gravity (the dancer's weight is taken off by its cylinder),
    /// its shaft friction and its gearbox's loss at the rotor ((1 - factor) times its input torque), and the rest (the
    /// web's tension); the losses; the energy parts.
    fn effect_signals(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        let e = self.eval(s, torque);
        for k in 0..2 {
            let gearbox = (1.0 - e.factor[k]) * e.gear_input[k];
            let parts = [e.j[k] * e.acc[k] / self.ratio, 0.0, e.shaft[k] + gearbox];
            out.extend(parts);
            out.push(torque[k] - parts.iter().sum::<f64>());
        }
        out.extend(self.losses(&e));
        out.extend(self.energy_parts(&e));
    }
    fn loss_powers(&self, s: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        out.extend(self.losses(&self.eval(s, torque)));
    }

    /// Seen from the front: the unwind roll on the left, the rewind on the right, their motors behind them, the dancer's
    /// roller below in the middle on its slide with the cylinder under it, and the web from roll to roller to roll,
    /// shaded by its tension against the setpoint.
    fn scene(&self, s: &[f64], out: &mut Scene) {
        let e = self.eval(s, &[0.0, 0.0]);
        let (x, width, rd) = (0.35, self.width, self.roller_radius);
        let depth = 0.5 * (self.spans[0] + self.spans[1]) + e.y;
        let phi = [s[0], s[1]];
        out.labels.clear();
        out.parts.clear();
        let grey = [0.62, 0.64, 0.68];
        let (mut angle, mut speed) = ([0.0; 2], [0.0; 2]);
        self.motor_state(s, &mut angle, &mut speed);
        out.parts.push(Part::new("frame", Shape::Box { size: [1.2, 0.04, 0.06] }, [0.0, 0.3, 0.25], grey));
        for (k, centre) in [-x, x].into_iter().enumerate() {
            let turn = -phi[k];
            out.parts.extend([
                Part::new(
                    "roll",
                    Shape::Cylinder { radius: e.r[k], length: width },
                    [centre, 0.0, 0.0],
                    [0.85, 0.82, 0.7],
                ),
                Part::new(
                    "core",
                    Shape::Cylinder { radius: self.roll.core, length: width + 0.04 },
                    [centre, 0.0, 0.0],
                    [0.55, 0.4, 0.25],
                ),
                Part::new(
                    "roll-spoke",
                    Shape::Box { size: [1.8 * self.roll.core, 0.004, 0.2 * self.roll.core] },
                    [centre, -0.5 * width - 0.03, 0.0],
                    [0.1, 0.1, 0.1],
                )
                .turned(turn),
                Part::new("gearbox", Shape::Box { size: [0.09, 0.09, 0.09] }, [centre, 0.5 * width + 0.1, 0.0], grey),
                Part::new(
                    sim_core::scene::MOTOR_PART,
                    Shape::Box { size: [0.1, 0.16, 0.1] },
                    [centre, 0.5 * width + 0.23, 0.0],
                    [0.9, 0.45, 0.1],
                ),
                // The motor shaft's bar, at the motor's far end, turning with it.
                Part::new(
                    "rotor-bar",
                    Shape::Box { size: [0.08, 0.004, 0.015] },
                    [centre, 0.5 * width + 0.315, 0.0],
                    [0.1, 0.1, 0.1],
                )
                .turned(-angle[k]),
            ]);
        }
        out.parts.extend(self.dancer_parts(e.y));
        let set = self.tension_setpoint(e.r[1]).max(1.0);
        let shade = e.t.map(|t| ((t - set) / set).clamp(-1.0, 1.0) as f32);
        let spans = [([-x + e.r[0], 0.0], [-rd, -depth], shade[0]), ([rd, -depth], [x - e.r[1], 0.0], shade[1])];
        for (from, to, shade) in spans {
            out.parts.push(
                Part::new(
                    "web",
                    Shape::Segment { to: [to[0], 0.0, to[1]], width },
                    [from[0], 0.0, from[1]],
                    sim_core::scene::BELT_COLOR,
                )
                .highlighted(shade),
            );
        }
        for (k, centre) in [-x, x].into_iter().enumerate() {
            out.labels.push(Label {
                pos: [centre, 0.0, self.roll.full + 0.08],
                owner_key: self.motors[k].key,
                value: speed[k] * 60.0 / TAU,
                unit_key: "unit-rpm",
                decimals: 0,
            });
        }
        out.labels.extend([
            Label { pos: [0.15, 0.0, -depth], owner_key: "group-dancer", value: e.y, unit_key: "unit-m", decimals: 4 },
            Label {
                pos: [0.0, 0.0, 0.25],
                owner_key: "group-web",
                value: e.t[0].max(e.t[1]),
                unit_key: "unit-n",
                decimals: 1,
            },
        ]);
    }
}

#[cfg(test)]
mod tests;
