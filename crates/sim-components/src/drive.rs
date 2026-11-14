//! Servo drive: cascaded position (P, the Kv factor), velocity (PI) and current (PI) control on the
//! motor's own encoder, with feedforward, measurement effects and torque limits. One switch each.
//!
//! The current controller works on the motor winding as a resistance and an inductance with the
//! back-EMF of the turning rotor: L di/dt = u - R i - ke w, torque = kt i, and |u| limited by the
//! DC link. With Kp = L / T and Tn = L / R the current follows its setpoint with time constant T.
//! The controller holds its voltage over a base step, as an inverter holds it over a PWM period; the
//! winding current is integrated over the step together with the mechanics (`Sim::step`), so that the
//! electrical power and the work on the shaft come from the same integration.
//!
//! The one winding stands for the three phases: its current is the RMS phase current and u i is the
//! electrical power, so u is three times the RMS phase voltage. A DC link U gives a phase amplitude of
//! U / sqrt(3), that is U / sqrt(6) RMS: the winding gets sqrt(3/2) U at most (`WINDING_PER_DC_LINK`).

use sim_core::testkit::Noise;
use sim_core::{Config, ConfigError};
use std::collections::VecDeque;
use std::f64::consts::TAU;

type Result<T> = std::result::Result<T, ConfigError>;

#[derive(Debug, Clone)]
pub struct DriveParams {
    pub kv: f64,
    pub kp: f64,
    pub ki: f64,
    /// Feedforward weights; 0 when switched off.
    pub velocity_ff: f64,
    pub torque_ff: f64,
    /// Motor winding: torque per current and resistance (the inductance is below).
    pub kt: f64,
    pub resistance: f64,
    /// Back-EMF per speed; 0 when switched off. In SI units it equals the torque constant.
    pub ke: f64,
    /// Highest voltage at the winding, `WINDING_PER_DC_LINK` times the DC link voltage; infinite when the
    /// limit is switched off.
    pub max_voltage: f64,
    /// Current controller gains, and its cycle (the base step).
    pub current_kp: f64,
    pub current_ki: f64,
    pub current_cycle: f64,
    pub peak_torque: f64,
    /// What the motor can give continuously: the limit for the RMS torque of a cycle.
    pub rated_torque: f64,
    pub rated_speed: f64,
    pub no_load_speed: f64,
    pub limit: bool,
    /// Encoder step in rad; 0 for an ideal encoder.
    pub encoder_step: f64,
    /// Low-pass gains per velocity cycle; 1 when the filter is off.
    pub velocity_filter: f64,
    pub torque_filter: f64,
    pub notch: Option<[f64; 5]>,
    pub velocity_cycle: f64,
    /// Cycles between computing a value and it acting: of the current loop for the voltage, of the
    /// velocity loop for the torque setpoint. 0 when the delay is switched off.
    pub current_delay: usize,
    pub velocity_delay: usize,
    /// Winding inductance, for the winding's time constant once its resistance changes with temperature.
    pub inductance: f64,
    /// Heating of the winding: whether it is on, the temperature rise at the rated current, the rated
    /// current, the thermal time constant and the rise the simulation starts with.
    pub heating: bool,
    pub rated_rise: f64,
    pub rated_current: f64,
    pub thermal_time_constant: f64,
    pub start_rise: f64,
    /// Saturation: the share of the torque constant lost at the maximum current; 0 when switched off.
    pub saturation: f64,
    pub max_current: f64,
    /// effects.drive_current_limit: the drive's maximum output current, `ratings.drive_max_current_a`, A. The torque
    /// setpoint is held to the torque of this current (`torque_range`), as the drive holds its current setpoint.
    /// Infinite with the effect off or without the rating.
    pub drive_max_current: f64,
    /// DC link with a braking chopper: whether it is simulated, the voltage the rectifier holds, the
    /// capacitance, the voltage at which the chopper switches the braking resistor in, the most the
    /// resistor takes, and the voltage at which the drive trips.
    pub dc_link: bool,
    /// The drives of the machine share one DC link (`shared_dc_link`); only with `dc_link`.
    pub shared_dc_link: bool,
    pub bus_voltage: f64,
    pub dc_capacitance: f64,
    pub braking_threshold: f64,
    pub resistor_peak: f64,
    pub overvoltage: f64,
    /// Time constant R C with which a disabled drive's DC link discharges through its bleeder and the drive's own
    /// consumption, down to the bus level the rectifier holds.
    pub discharge_time_constant: f64,
    /// Regenerative supply (`regenerative_supply`, only with `dc_link`): above the bus level the line module feeds
    /// the DC link's energy back into the mains, up to `line_max_power`, of which `line_efficiency` arrives there.
    pub regenerative: bool,
    pub line_max_power: f64,
    pub line_efficiency: f64,
    /// Sensor noise, as standard deviations: on the measured position in rad and on the measured current.
    /// 0 when switched off.
    pub encoder_noise: f64,
    pub current_noise: f64,
    pub noise_seed: u64,
    /// effects.field_weakening: the winding as the q axis of a d-q model, with a d current (`FieldWeakening`).
    pub field_weakening: Option<FieldWeakening>,
    /// effects.drive_overload: the drive's thermal image, which lowers its current limit (`Overload`).
    pub overload: Option<Overload>,
    /// effects.motor_thermal_nodes: winding and housing as two heat capacities (`ThermalNodes`); without it one.
    pub thermal_nodes: Option<ThermalNodes>,
    /// Fine interpolation, the torque feedforward's model, friction feedforward and balancing (`Precontrol`).
    pub precontrol: Precontrol,
}

/// What the drive does with the setpoints between and beside the gains, as a SINAMICS S120/S210 in DSC is set up.
/// The default is the drive before 14 November 2026: setpoint held, rigid model, no friction, no balancing.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Precontrol {
    /// effects.fine_interpolation: between profile cycles the position loop's setpoint runs on from the last
    /// setpoints as pos + v t + a t^2 / 2, as the speed feedforward runs on as v + a t (its derivative), the drive's
    /// fine interpolator. Off it is held for the profile cycle, as a position loop closed in the controller at the
    /// IPO cycle sees it.
    pub fine_interpolation: bool,
    /// drive.torque_feedforward_model = "inertia": the torque feedforward is one moment of inertia per motor times the
    /// motor's acceleration setpoint (S120 p1498 with p1428 speed/torque precontrol), the inertia the rigid model has
    /// at the start pose; no gravity, no speed terms, no coupling. "rigid" (false): the rigid model's inverse dynamics.
    pub constant_inertia: bool,
    /// effects.friction_feedforward: the friction characteristic, (speed rad/s, torque N m) points at rising speeds
    /// (S120 p3820-p3839); empty when off. See `friction_at`.
    pub friction: Vec<[f64; 2]>,
    /// drive.precontrol_balance_s: the position setpoint passes a first-order lag of this time constant before the
    /// position loop compares it, so that it arrives with the motor that the speed precontrol moves through the
    /// closed speed loop (S120 p1429 balancing); 0 off.
    pub balance_time: f64,
}

impl Precontrol {
    /// The friction feedforward at a speed setpoint: linear between the points, from (0, 0) to the first, held
    /// beyond the last, with the sign of the speed.
    pub fn friction_at(&self, speed: f64) -> f64 {
        let (s, mut last) = (speed.abs(), [0.0, 0.0]);
        for &[v, t] in &self.friction {
            if s <= v {
                return (last[1] + (t - last[1]) * (s - last[0]) / (v - last[0])).copysign(speed);
            }
            last = [v, t];
        }
        last[1].copysign(speed)
    }
}

/// effects.field_weakening: the permanent-magnet synchronous motor in d-q axes. The one winding is the q axis, scaled as
/// the module comment says (current the RMS phase current, voltage three times the RMS phase voltage); the d axis is
/// scaled alike, so that R, L and the voltage limit are the winding's and the flux linkage is ke / pole pairs. With the
/// electrical speed w = pole pairs * shaft speed:
///   u_q = R i_q + L_q di_q/dt + ke speed + w L_d i_d,   u_d = R i_d - w L_q i_q,   u_q^2 + u_d^2 <= U^2,
///   torque = kt i_q + pole pairs (L_d - L_q) i_d i_q.
/// Above base speed the drive sets a negative d current, the least that brings the voltage within the circle
/// (`DriveParams::d_current_for`); i_q^2 + i_d^2 is the RMS phase current squared.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FieldWeakening {
    pub pole_pairs: f64,
    /// d-axis inductance as the winding has it (three times the phase value); the q axis's is `DriveParams::inductance`.
    pub d_inductance: f64,
}

/// effects.drive_overload: the drive's thermal image (i^2 t) of its power unit, x' = ((i / I_rated)^2 - x) / tau, 1 at
/// the rated current held. At the trip level the drive lowers its current limit to the base load current, and holds it
/// there until the image is back at 1. The trip level follows from the time the drive gives its maximum current from
/// the rated load: trip = r^2 - (r^2 - 1) exp(-t_max / tau) for r = I_max / I_rated.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Overload {
    pub rated_current: f64,
    pub base_current: f64,
    pub max_current: f64,
    pub time_constant: f64,
    pub trip: f64,
}

impl Overload {
    /// With effects.drive_overload on (off where the config has no switch), from the `[ratings]`.
    pub fn from_config(cfg: &Config) -> Result<Option<Self>> {
        if !(cfg.get("effects.drive_overload").is_ok() && cfg.bool("effects.drive_overload")?) {
            return Ok(None);
        }
        let rating = |leaf: &str| cfg.positive(&format!("ratings.{leaf}"));
        let (rated_current, base_current) = (rating("drive_rated_current_a")?, rating("drive_base_load_current_a")?);
        let max_current = rating("drive_max_current_a")?;
        if base_current >= rated_current {
            return Err(ConfigError::Invalid("ratings.drive_base_load_current_a".into()));
        }
        if max_current <= rated_current {
            return Err(ConfigError::Invalid("ratings.drive_max_current_a".into()));
        }
        let time_constant = rating("drive_overload_time_constant_s")?;
        let r2 = (max_current / rated_current).powi(2);
        let trip = r2 - (r2 - 1.0) * (-rating("drive_max_current_time_s")? / time_constant).exp();
        Ok(Some(Self { rated_current, base_current, max_current, time_constant, trip }))
    }

    /// The image after `dt` at a constant `current`, exactly.
    pub fn image_after(&self, image: f64, current: f64, dt: f64) -> f64 {
        let target = (current / self.rated_current).powi(2);
        target + (image - target) * (-dt / self.time_constant).exp()
    }
}

/// effects.motor_thermal_nodes: three nodes, winding, stator/housing and ambient, as two heat capacities in a chain.
/// The resistances add up to the one-node model's, rated rise over rated loss, so the settled rises stay; the winding
/// has the share `winding_share` of it (to the housing) and the time constant `winding_time_constant` against the
/// housing. The heat capacities add up to the one-node model's too (thermal time constant over the resistance).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThermalNodes {
    pub winding_share: f64,
    pub winding_time_constant: f64,
}

/// Resistance of copper rises by this share per kelvin.
const COPPER: f64 = 0.0039;

/// Winding voltage per DC link voltage, sqrt(3/2): see the module comment.
pub const WINDING_PER_DC_LINK: f64 = 1.224_744_871_391_589;

impl DriveParams {
    /// `dt` is the base step, which is also the current-loop cycle.
    pub fn from_config(cfg: &Config, dt: f64, velocity_cycle: f64) -> Result<Self> {
        let on = |effect: &str| cfg.bool(&format!("effects.{effect}"));
        let low_pass = |effect: &str, leaf: &str| -> Result<f64> {
            let tau = cfg.positive(&format!("drive.{leaf}"))?;
            Ok(if on(effect)? { 1.0 - (-velocity_cycle / tau).exp() } else { 1.0 })
        };
        let kp = cfg.positive("drive.velocity_kp_nm_s_rad")?;
        let delay = |leaf: &str| -> Result<usize> {
            let path = format!("drive.{leaf}");
            let cycles = cfg.f64(&path)?;
            if cycles < 0.0 || cycles.fract() != 0.0 || cycles > 1000.0 {
                return Err(ConfigError::Invalid(path));
            }
            Ok(if on("drive_delay")? { cycles as usize } else { 0 })
        };
        let notch = if on("notch_filter")? {
            let (f, bw) = (cfg.positive("drive.notch_frequency_hz")?, cfg.positive("drive.notch_bandwidth_hz")?);
            Some(notch_coefficients(f, bw, velocity_cycle))
        } else {
            None
        };
        let (kt, resistance) = (cfg.positive("motor.torque_constant_nm_a")?, cfg.positive("motor.resistance_ohm")?);
        // The torque fades out between the two speeds; with the no-load speed at or below the rated one it
        // would have no slope to fade on.
        let (rated_speed, no_load_speed) =
            (cfg.positive("drive.rated_speed_rad_s")?, cfg.positive("drive.no_load_speed_rad_s")?);
        if no_load_speed <= rated_speed {
            return Err(ConfigError::Invalid("drive.no_load_speed_rad_s".into()));
        }
        let kt_at_max = cfg.positive("motor.torque_constant_at_max_current_nm_a")?;
        if kt_at_max > kt {
            return Err(ConfigError::Invalid("motor.torque_constant_at_max_current_nm_a".into()));
        }
        let current_kp = cfg.positive("drive.current_kp_v_a")?;
        let bus_voltage = cfg.positive("drive.bus_voltage_v")?;
        // The simulated DC link is built from the drive's ratings, which it then needs.
        let dc_link = on("dc_link")?;
        let rating = |leaf: &str| if dc_link { cfg.positive(&format!("ratings.{leaf}")) } else { Ok(0.0) };
        let (braking_threshold, overvoltage) = (rating("braking_threshold_v")?, rating("dc_link_overvoltage_v")?);
        if dc_link && !(bus_voltage < braking_threshold && braking_threshold < overvoltage) {
            return Err(ConfigError::Invalid("ratings.braking_threshold_v".into()));
        }
        let noise = |leaf: &str| -> Result<f64> {
            let path = format!("drive.{leaf}");
            let value = cfg.f64(&path)?;
            if !(value >= 0.0) {
                return Err(ConfigError::Invalid(path));
            }
            Ok(if on("sensor_noise")? { value } else { 0.0 })
        };
        let seed = cfg.f64("drive.noise_seed")?;
        if seed < 0.0 || seed.fract() != 0.0 {
            return Err(ConfigError::Invalid("drive.noise_seed".into()));
        }
        // The line module's ratings are needed only where it feeds back.
        let regenerative = dc_link && on("regenerative_supply")?;
        let line = |leaf: &str| if regenerative { cfg.positive(&format!("ratings.{leaf}")) } else { Ok(1.0) };
        let line_efficiency = line("line_module_efficiency")?;
        // A config written before 13 November 2026 has no switch: the drive's limit is judged, not simulated.
        let drive_limit = cfg.get("effects.drive_current_limit").is_ok() && on("drive_current_limit")?;
        if line_efficiency > 1.0 {
            return Err(ConfigError::Invalid("ratings.line_module_efficiency".into()));
        }
        // The three switches of 14 November 2026 read as off where a config has none; their keys only when on.
        let switched =
            |effect: &str| -> Result<bool> { Ok(cfg.get(&format!("effects.{effect}")).is_ok() && on(effect)?) };
        let field_weakening = if switched("field_weakening")? {
            Some(FieldWeakening {
                pole_pairs: cfg.positive("motor.pole_pairs")?,
                d_inductance: cfg.positive("motor.d_inductance_h")?,
            })
        } else {
            None
        };
        let thermal_time_constant = cfg.positive("motor.thermal_time_constant_s")?;
        let thermal_nodes = if switched("motor_thermal_nodes")? {
            let share = cfg.positive("motor.winding_resistance_share")?;
            let winding_time_constant = cfg.positive("motor.winding_time_constant_s")?;
            if share >= 1.0 {
                return Err(ConfigError::Invalid("motor.winding_resistance_share".into()));
            }
            // The housing's heat capacity, (tau - tau_w / share) / R, has to be left over.
            if winding_time_constant >= share * thermal_time_constant {
                return Err(ConfigError::Invalid("motor.winding_time_constant_s".into()));
            }
            Some(ThermalNodes { winding_share: share, winding_time_constant })
        } else {
            None
        };
        // The keys of 14 November 2026 for the loops: a config without them gets the drive as it was (`Precontrol`).
        let model = cfg.get("drive.torque_feedforward_model").map(|v| v.as_str());
        let constant_inertia = match model {
            Err(_) | Ok(Some("rigid")) => false,
            Ok(Some("inertia")) => true,
            Ok(_) => return Err(ConfigError::Invalid("drive.torque_feedforward_model".into())),
        };
        let friction = if switched("friction_feedforward")? {
            let (speeds, torques) = (
                cfg.f64s("drive.friction_feedforward_speed_rad_s")?,
                cfg.f64s("drive.friction_feedforward_torque_nm")?,
            );
            if speeds.is_empty() || speeds.len() != torques.len() || torques.iter().any(|t| !(*t >= 0.0)) {
                return Err(ConfigError::Invalid("drive.friction_feedforward_torque_nm".into()));
            }
            if !(speeds[0] > 0.0) || speeds.windows(2).any(|w| !(w[1] > w[0])) {
                return Err(ConfigError::Invalid("drive.friction_feedforward_speed_rad_s".into()));
            }
            speeds.iter().zip(&torques).map(|(v, t)| [*v, *t]).collect()
        } else {
            Vec::new()
        };
        let precontrol = Precontrol {
            fine_interpolation: switched("fine_interpolation")?,
            constant_inertia,
            friction,
            balance_time: if cfg.get("drive.precontrol_balance_s").is_ok() {
                cfg.non_negative("drive.precontrol_balance_s")?
            } else {
                0.0
            },
        };
        Ok(Self {
            precontrol,
            field_weakening,
            overload: Overload::from_config(cfg)?,
            thermal_nodes,
            dc_link,
            shared_dc_link: dc_link && on("shared_dc_link")?,
            bus_voltage,
            dc_capacitance: rating("dc_link_capacitance_uf")? * 1e-6,
            braking_threshold,
            resistor_peak: rating("braking_resistor_peak_w")?,
            overvoltage,
            discharge_time_constant: rating("dc_link_discharge_time_constant_s")?,
            regenerative,
            line_max_power: line("line_module_max_power_w")?,
            line_efficiency,
            encoder_noise: noise("encoder_noise_counts")? * TAU / cfg.positive("drive.encoder_counts_per_rev")?,
            current_noise: noise("current_noise_a")?,
            noise_seed: seed as u64,
            kt,
            resistance,
            ke: if on("back_emf")? { kt } else { 0.0 },
            max_voltage: if on("voltage_limit")? { WINDING_PER_DC_LINK * bus_voltage } else { f64::INFINITY },
            current_kp,
            current_ki: current_kp / cfg.positive("drive.current_tn_s")?,
            current_cycle: dt,
            kv: cfg.positive("drive.kv_1_s")?,
            kp,
            ki: kp / cfg.positive("drive.velocity_tn_s")?,
            velocity_ff: if on("velocity_feedforward")? {
                cfg.non_negative("drive.velocity_feedforward_factor")?
            } else {
                0.0
            },
            torque_ff: if on("torque_feedforward")? {
                cfg.non_negative("drive.torque_feedforward_factor")?
            } else {
                0.0
            },
            peak_torque: cfg.positive("drive.peak_torque_nm")?,
            rated_torque: cfg.positive("drive.rated_torque_nm")?,
            rated_speed,
            no_load_speed,
            limit: on("torque_limit")?,
            encoder_step: if on("encoder_resolution")? {
                TAU / cfg.positive("drive.encoder_counts_per_rev")?
            } else {
                0.0
            },
            velocity_filter: low_pass("velocity_filter", "velocity_filter_s")?,
            torque_filter: low_pass("torque_filter", "torque_filter_s")?,
            notch,
            velocity_cycle,
            current_delay: delay("current_delay_cycles")?,
            velocity_delay: delay("velocity_delay_cycles")?,
            inductance: cfg.positive("motor.inductance_h")?,
            heating: on("winding_heating")?,
            rated_rise: cfg.positive("motor.rated_temperature_rise_k")?,
            rated_current: cfg.positive("motor.rated_current_a")?,
            thermal_time_constant,
            start_rise: cfg.non_negative("motor.start_temperature_rise_k")?,
            saturation: if on("torque_saturation")? { 1.0 - kt_at_max / kt } else { 0.0 },
            max_current: cfg.positive("motor.max_current_a")?,
            drive_max_current: if drive_limit && cfg.get("ratings.drive_max_current_a").is_ok() {
                cfg.positive("ratings.drive_max_current_a")?
            } else {
                f64::INFINITY
            },
        })
    }

    /// Torque limits (lowest, highest) at a speed: the current limit everywhere, and in the
    /// direction of rotation the torque-speed curve that the voltage limit leaves. With the drive's current limit
    /// (effects.drive_current_limit) also the torque of the drive's maximum current, where that is the lower: the
    /// current setpoint, the torque's, then stays within min(motor, drive maximum current).
    pub fn torque_range(&self, speed: f64) -> (f64, f64) {
        let (low, high) = if self.limit { self.torque_curve(speed) } else { (f64::NEG_INFINITY, f64::INFINITY) };
        if self.drive_max_current.is_finite() {
            let most = self.torque_of(self.drive_max_current);
            (low.max(-most), high.min(most))
        } else {
            (low, high)
        }
    }

    /// Winding resistance at a temperature rise.
    pub fn resistance_at(&self, rise: f64) -> f64 {
        self.resistance * (1.0 + COPPER * rise)
    }

    /// Torque of a current: the torque constant, less what saturation takes at high current.
    pub fn torque_of(&self, current: f64) -> f64 {
        self.kt * current * (1.0 - self.saturation * (current / self.max_current).powi(2).min(1.0))
    }

    /// The current that gives a torque, the inverse of `torque_of`, up to the maximum current. Newton's
    /// method from below: the curve bends down, so every step stays below the answer and none overshoots.
    pub fn current_for(&self, torque: f64) -> f64 {
        if self.saturation == 0.0 {
            return torque / self.kt;
        }
        // In units of the maximum current and of the torque it would give unsaturated: y = x - s x^3.
        // A curve that loses more than a third turns down before the maximum current; its top is the limit.
        let (s, y) = (self.saturation, torque.abs() / (self.kt * self.max_current));
        let top = (1.0 / (3.0 * s).sqrt()).min(1.0);
        let mut x = top;
        if y < top - s * top.powi(3) {
            x = y;
            for _ in 0..8 {
                x += (y - x + s * x.powi(3)) / (1.0 - 3.0 * s * x * x);
            }
        }
        (x * self.max_current).copysign(torque)
    }

    /// Back-EMF at a current and speed. The torque constant that saturation leaves is the back-EMF
    /// constant too, so that the electrical power is the shaft power plus the copper loss.
    pub fn back_emf(&self, current: f64, speed: f64) -> f64 {
        self.ke * speed * (1.0 - self.saturation * (current / self.max_current).powi(2).min(1.0))
    }

    /// The winding's rates at a moment of a base step, for the integrator (`Sim::step`), with the voltage and the
    /// resistance held over the step: [di/dt, electrical power u i, copper loss R i^2, shaft power torque w]. With
    /// the back-EMF on, u i = R i^2 + L i di/dt + torque w at every moment, so the winding's energy closes.
    ///
    /// With field weakening the d current `d_current` is held over the step as the controller set it (`FieldWeakening`):
    /// it adds w L_d i_d to what the q axis meets, u_d i_d to the electrical power, R i_d^2 to the copper loss and the
    /// reluctance torque to the shaft. u_q i_q + u_d i_d = R (i_q^2 + i_d^2) + L i_q di_q/dt + torque w still closes.
    pub fn winding_rates(&self, voltage: f64, resistance: f64, current: f64, speed: f64, d_current: f64) -> [f64; 4] {
        let (coupling, d_power) = match &self.field_weakening {
            Some(fw) => {
                let w = fw.pole_pairs * speed;
                (w * fw.d_inductance * d_current, (resistance * d_current - w * self.inductance * current) * d_current)
            }
            None => (0.0, 0.0),
        };
        let rate = (voltage - resistance * current - self.back_emf(current, speed) - coupling) / self.inductance;
        [
            rate,
            voltage * current + d_power,
            resistance * (current * current + d_current * d_current),
            self.torque_with(current, d_current) * speed,
        ]
    }

    /// Torque of a q current and a d current: `torque_of`, plus the reluctance torque pole pairs (L_d - L_q) i_d i_q
    /// with field weakening.
    pub fn torque_with(&self, current: f64, d_current: f64) -> f64 {
        let reluctance = self
            .field_weakening
            .map_or(0.0, |fw| fw.pole_pairs * (fw.d_inductance - self.inductance) * d_current * current);
        self.torque_of(current) + reluctance
    }

    /// The d current the drive sets for a q current at a speed with the voltage `max_voltage`: none while
    /// u_q^2 + u_d^2 (steady, `FieldWeakening`) stays within it, else the least negative one that brings it there, at
    /// most `limit`. Where none does, the one that needs the least voltage. 0 without field weakening.
    ///
    /// shortcut: the d current follows its setpoint at once (no d-axis dynamics) and the back-EMF here is unsaturated;
    /// give the d axis a winding state of its own if current-loop transients in field weakening are ever studied.
    pub fn d_current_for(&self, q: f64, speed: f64, max_voltage: f64, limit: f64) -> f64 {
        let Some(fw) = &self.field_weakening else { return 0.0 };
        let w = fw.pole_pairs * speed;
        let (a, b, r) = (w * fw.d_inductance, w * self.inductance, self.resistance);
        let c0 = r * q + self.ke * speed;
        // (c0 + a d)^2 + (r d - b q)^2 = U^2 as a d^2 + b d + c = 0.
        let (qa, qb, qc) =
            (a * a + r * r, 2.0 * (a * c0 - r * b * q), c0 * c0 + b * b * q * q - max_voltage * max_voltage);
        if !(qc > 0.0) {
            return 0.0;
        }
        let disc = qb * qb - 4.0 * qa * qc;
        let d = if disc >= 0.0 { (-qb + disc.sqrt()) / (2.0 * qa) } else { -qb / (2.0 * qa) };
        d.clamp(-limit, 0.0)
    }

    /// With field weakening: the most torque, motoring or braking (as a magnitude), at a speed of at least 0 within the
    /// current circle of the motor's maximum current and the voltage circle of `max_voltage`, over every d current
    /// from 0 to minus the maximum current (golden section; the torque rises to one top and falls).
    fn field_weakened_torque(&self, fw: &FieldWeakening, speed: f64, motoring: bool) -> f64 {
        let (imax, umax, r) = (self.max_current, self.max_voltage, self.resistance);
        let w = fw.pole_pairs * speed;
        let (a, b) = (w * fw.d_inductance, w * self.inductance);
        // The q current the two circles leave at a d current, as a magnitude.
        let q_at = |d: f64| -> f64 {
            let by_current = (imax * imax - d * d).max(0.0).sqrt();
            if !umax.is_finite() {
                return by_current;
            }
            // (r q + c1)^2 + (r d - b q)^2 <= U^2 as qa q^2 + qb q + qc <= 0.
            let c1 = self.ke * speed + a * d;
            let (qa, qb, qc) = (r * r + b * b, 2.0 * r * (c1 - b * d), c1 * c1 + r * r * d * d - umax * umax);
            let disc = qb * qb - 4.0 * qa * qc;
            if disc < 0.0 {
                return 0.0;
            }
            let q = if motoring { (-qb + disc.sqrt()) / (2.0 * qa) } else { -(-qb - disc.sqrt()) / (2.0 * qa) };
            q.clamp(0.0, by_current)
        };
        let torque = |d: f64| {
            let q = if motoring { q_at(d) } else { -q_at(d) };
            let t = self.torque_with(q, d);
            (if motoring { t } else { -t }).max(0.0)
        };
        let (mut lo, mut hi) = (-imax, 0.0);
        let g = 0.5 * (5f64.sqrt() - 1.0);
        let (mut x1, mut x2) = (hi - g * (hi - lo), lo + g * (hi - lo));
        let (mut f1, mut f2) = (torque(x1), torque(x2));
        for _ in 0..40 {
            if f1 < f2 {
                (lo, x1, f1) = (x1, x2, f2);
                x2 = lo + g * (hi - lo);
                f2 = torque(x2);
            } else {
                (hi, x2, f2) = (x2, x1, f1);
                x1 = hi - g * (hi - lo);
                f1 = torque(x1);
            }
        }
        f1.max(f2).max(torque(0.0))
    }

    /// The temperature rise the winding settles at when a cycle with this RMS current repeats for long:
    /// the copper loss, with the resistance at that temperature, against the loss at the rated current.
    pub fn settled_rise(&self, rms_current: f64) -> f64 {
        let q = self.rated_rise / (1.0 + COPPER * self.rated_rise) * (rms_current / self.rated_current).powi(2);
        if COPPER * q < 1.0 { q / (1.0 - COPPER * q) } else { f64::INFINITY }
    }

    /// The same limits, whether or not the limit is switched on: what the motor and drive can give.
    /// With field weakening the curve is the d-q model's (`FieldWeakening`), up to the peak torque; without it the
    /// torque fades out linearly between the rated and the no-load speed.
    pub fn torque_curve(&self, speed: f64) -> (f64, f64) {
        if let Some(fw) = &self.field_weakening {
            let (motoring, braking) = (
                self.field_weakened_torque(fw, speed.abs(), true).min(self.peak_torque),
                self.field_weakened_torque(fw, speed.abs(), false).min(self.peak_torque),
            );
            return if speed >= 0.0 { (-braking, motoring) } else { (-motoring, braking) };
        }
        let fade = ((self.no_load_speed - speed.abs()) / (self.no_load_speed - self.rated_speed)).clamp(0.0, 1.0);
        let (peak, motoring) = (self.peak_torque, self.peak_torque * fade);
        if speed >= 0.0 { (-peak, motoring) } else { (-motoring, peak) }
    }
}

/// Biquad notch (b0, b1, b2, a1, a2), normalised by a0.
fn notch_coefficients(frequency: f64, bandwidth: f64, cycle: f64) -> [f64; 5] {
    let w0 = TAU * frequency * cycle;
    let alpha = w0.sin() / (2.0 * frequency / bandwidth);
    let a0 = 1.0 + alpha;
    [1.0 / a0, -2.0 * w0.cos() / a0, 1.0 / a0, -2.0 * w0.cos() / a0, (1.0 - alpha) / a0]
}

/// The loops and filters of one drive.
#[derive(Debug, Clone, Default)]
pub struct Drive {
    pub pos_set: f64,
    /// Speed setpoint at the last profile cycle, and its rate until the next one.
    pub vel_set: f64,
    pub acc_set: f64,
    /// Time since the profile generator last wrote the setpoints; the controller adds a base step after each.
    pub since_setpoint: f64,
    pub torque_ff: f64,
    /// The setpoint the position loop compared with at its last cycle: `pos_set` run on between profile cycles
    /// (`Precontrol::fine_interpolation`) and balanced (`Precontrol::balance_time`). `following_error` is from it.
    pub position_setpoint: f64,
    pub following_error: f64,
    /// The position loop's share of the speed command, Kv times the following error, held between its cycles.
    pub position_part: f64,
    /// The velocity loop's output at its last cycle in its two parts, before filters and limits: the feedforward
    /// (`torque_ff` weighted, and friction) and the PI controller's.
    pub feedforward_torque: f64,
    pub controller_torque: f64,
    pub speed_command: f64,
    /// What the velocity loop asks for, before the torque limit; `torque_command` is what the limit leaves of it.
    pub torque_demand: f64,
    pub torque_command: f64,
    /// A test signal added to what the velocity loop asks for, before the limit: the loop is measured by it
    /// (`analysis::velocity_loop_response`). Zero otherwise.
    pub torque_injection: f64,
    pub torque: f64,
    /// Winding current and the voltage the current controller applies.
    pub current: f64,
    pub voltage: f64,
    /// Electrical power into the winding, the mean u i over the last base step; negative while the motor feeds back.
    pub power: f64,
    /// Copper loss and the power the torque gave the shaft, means over the last step like `power`.
    pub copper_loss: f64,
    pub shaft_power: f64,
    /// Temperature of the winding above its surroundings.
    pub temperature_rise: f64,
    /// With effects.motor_thermal_nodes: temperature of the stator and housing above the surroundings; 0 without.
    pub housing_rise: f64,
    /// With effects.field_weakening: the d current the drive sets (`DriveParams::d_current_for`), held over the base
    /// step like the voltage; 0 without. The RMS phase current is the hypotenuse of it and `current` (`phase_current`).
    pub d_current: f64,
    /// With effects.drive_overload: the drive's thermal image (`Overload`), and whether it has lowered its current
    /// limit to the base load current.
    pub overload_image: f64,
    pub overloaded: bool,
    /// DC link voltage, the power the braking resistor burns, and whether the drive has tripped on
    /// overvoltage. With the DC link switched off the voltage stays at the bus level.
    pub dc_voltage: f64,
    pub resistor_power: f64,
    pub tripped: bool,
    /// With the DC link simulated: the power drawn from the line into the DC link, and with the regenerative supply
    /// the power fed back into the mains and what the line module lost doing so, over the last step.
    pub rectifier_power: f64,
    pub mains_power: f64,
    pub line_loss: f64,
    /// With the DC link simulated and the drive off: what its discharge through the bleeder took out of the DC link over
    /// the last step, as a mean power (the rectifier draws it back while the link is held at the bus level).
    pub discharge_power: f64,
    /// The drive's own stream of sensor noise.
    pub noise: Noise,
    current_integral: f64,
    integral: f64,
    encoder: f64,
    last_encoder: f64,
    speed: f64,
    filtered: f64,
    notch: [f64; 4],
    /// Values computed but not yet acting, oldest first.
    voltage_queue: VecDeque<f64>,
    torque_queue: VecDeque<f64>,
}

/// `value` goes into the queue; what was computed `cycles` ago comes out.
pub(crate) fn delayed<T: Clone>(queue: &mut VecDeque<T>, value: T, cycles: usize) -> T {
    queue.push_back(value);
    while queue.len() > cycles + 1 {
        queue.pop_front();
    }
    queue[0].clone()
}

impl Drive {
    /// A drive at rest at `pos`, already delivering `torque` (so a hanging axis does not sag at start).
    pub fn at_rest(p: &DriveParams, pos: f64, torque: f64, torque_ff: f64) -> Self {
        let encoder = quantise(pos, p.encoder_step);
        Self {
            pos_set: pos,
            position_setpoint: pos,
            torque_ff,
            feedforward_torque: p.torque_ff * torque_ff,
            controller_torque: torque - p.torque_ff * torque_ff,
            torque_demand: torque,
            torque_command: torque,
            torque,
            current: p.current_for(torque),
            voltage: p.resistance * p.current_for(torque),
            current_integral: p.resistance * p.current_for(torque),
            temperature_rise: if p.heating { p.start_rise } else { 0.0 },
            // A winding started warm has its housing where that rise settles it.
            housing_rise: match (p.heating, &p.thermal_nodes) {
                (true, Some(n)) => p.start_rise * (1.0 - n.winding_share),
                _ => 0.0,
            },
            dc_voltage: p.bus_voltage,
            noise: Noise(p.noise_seed),
            integral: torque - p.torque_ff * torque_ff,
            encoder,
            last_encoder: encoder,
            filtered: torque,
            notch: [torque; 4],
            voltage_queue: std::iter::repeat_n(p.resistance * p.current_for(torque), p.current_delay).collect(),
            torque_queue: std::iter::repeat_n(torque, p.velocity_delay).collect(),
            ..Self::default()
        }
    }

    pub fn encoder(&self) -> f64 {
        self.encoder
    }

    /// The RMS phase current: of the q current `current` and the d current together.
    pub fn phase_current(&self) -> f64 {
        self.current.hypot(self.d_current)
    }

    /// The speed the velocity loop measured at its last cycle, filtered.
    pub fn measured_speed(&self) -> f64 {
        self.speed
    }

    /// Moves the encoder's readings so far by `by`, for a motor that was set elsewhere without turning there.
    pub fn shift_encoder(&mut self, by: f64) {
        self.encoder += by;
        self.last_encoder += by;
    }

    /// Moves the position setpoints by `by`, for a motor that was set elsewhere without turning there.
    pub fn shift_setpoint(&mut self, by: f64) {
        self.pos_set += by;
        self.position_setpoint += by;
    }

    /// Reads the motor encoder: the position as it reports it, with its noise, in whole counts. Once per base step in
    /// which a loop is due, so that the position and the velocity loop see the same sample (`encoder`).
    pub fn read_encoder(&mut self, p: &DriveParams, pos: f64) {
        let noise = if p.encoder_noise > 0.0 { p.encoder_noise * self.noise.next() } else { 0.0 };
        self.encoder = quantise(pos + noise, p.encoder_step);
    }

    /// Position loop, every `cycle`: following error times Kv against the position setpoint (`position_setpoint`),
    /// with the measured position `measured`: the motor encoder's (`encoder`) or a load-side one's.
    pub fn position_loop(&mut self, p: &DriveParams, measured: f64, cycle: f64) {
        let (c, t) = (&p.precontrol, self.since_setpoint);
        let target = if c.fine_interpolation {
            self.pos_set + (self.vel_set + 0.5 * self.acc_set * t) * t
        } else {
            self.pos_set
        };
        let share = if c.balance_time > 0.0 { 1.0 - (-cycle / c.balance_time).exp() } else { 1.0 };
        self.position_setpoint += share * (target - self.position_setpoint);
        self.following_error = self.position_setpoint - measured;
        self.position_part = p.kv * self.following_error;
        self.speed_command = self.position_part + p.velocity_ff * (self.vel_set + self.acc_set * t);
    }

    /// Velocity loop: PI on the measured speed (from `read_encoder`), plus torque feedforward, then filters and limits.
    pub fn velocity_loop(&mut self, p: &DriveParams, true_speed: f64) {
        // A real encoder gives positions only; the speed is their difference per cycle.
        let real = p.encoder_step > 0.0 || p.encoder_noise > 0.0;
        let raw = if real { (self.encoder - self.last_encoder) / p.velocity_cycle } else { true_speed };
        self.last_encoder = self.encoder;
        self.speed += p.velocity_filter * (raw - self.speed);

        // Velocity feedforward, interpolated between profile cycles. Holding it for a whole profile
        // cycle would make the speed command a staircase and the torque a sawtooth.
        let speed_set = self.vel_set + self.acc_set * self.since_setpoint;
        self.speed_command = self.position_part + p.velocity_ff * speed_set;

        let error = self.speed_command - self.speed;
        self.controller_torque = p.kp * error + self.integral;
        self.feedforward_torque = p.torque_ff * self.torque_ff + p.precontrol.friction_at(speed_set);
        let wanted = self.controller_torque + self.feedforward_torque;
        self.filtered += p.torque_filter * (wanted - self.filtered);
        let mut out = self.filtered;
        if let Some([b0, b1, b2, a1, a2]) = p.notch {
            let [x1, x2, y1, y2] = self.notch;
            out = b0 * self.filtered + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            self.notch = [self.filtered, x1, out, y1];
        }
        let (mut low, mut high) = p.torque_range(self.speed);
        // An overloaded drive gives its base load current at most.
        if let (true, Some(o)) = (self.overloaded, &p.overload) {
            let most = p.torque_of(o.base_current);
            (low, high) = (low.max(-most), high.min(most));
        }
        self.torque_demand = out;
        self.torque_command =
            delayed(&mut self.torque_queue, (out + self.torque_injection).clamp(low, high), p.velocity_delay);
        // Anti-windup: the integrator stops while it would push further into the limit.
        if (out <= high || error < 0.0) && (out >= low || error > 0.0) {
            self.integral += p.ki * error * p.velocity_cycle;
        }
    }

    /// Current loop: PI on the winding current. The voltage it sets is held over the next base step, in which the
    /// integrator takes the winding through it (`DriveParams::winding_rates`); then `winding_stepped`.
    pub fn current_control(&mut self, p: &DriveParams, speed: f64) {
        let measured = self.current + if p.current_noise > 0.0 { p.current_noise * self.noise.next() } else { 0.0 };
        let setpoint = p.current_for(self.torque_command);
        let error = setpoint - measured;
        // The voltage limit is what the DC link has at this moment.
        let max_voltage =
            if p.dc_link && p.max_voltage.is_finite() { WINDING_PER_DC_LINK * self.dc_voltage } else { p.max_voltage };
        // Field weakening: the d current for this setpoint; the q axis gets what the d axis leaves of the voltage.
        self.d_current = p.d_current_for(setpoint, speed, max_voltage, p.max_current.min(p.drive_max_current));
        let (coupling, q_voltage) = match &p.field_weakening {
            Some(fw) => {
                let w = fw.pole_pairs * speed;
                let u_d = p.resistance * self.d_current - w * p.inductance * self.current;
                (w * fw.d_inductance * self.d_current, (max_voltage * max_voltage - u_d * u_d).max(0.0).sqrt())
            }
            None => (0.0, max_voltage),
        };
        // The drive adds the back-EMF it expects at this speed, as real drives do. Without that the
        // integrator would have to follow every speed change, at the slow pace of L / R.
        let back_emf = p.back_emf(self.current, speed) + coupling;
        let wanted = p.current_kp * error + self.current_integral + back_emf;
        let limited = wanted.clamp(-q_voltage, q_voltage);
        self.voltage = delayed(&mut self.voltage_queue, limited, p.current_delay);
        // Anti-windup: the integrator stops while the bus voltage is used up.
        if wanted == limited {
            self.current_integral += p.current_ki * error * p.current_cycle;
        }
    }

    /// The winding's resistance over the next base step: at the temperature the step starts with.
    pub fn winding_resistance(&self, p: &DriveParams) -> f64 {
        if p.heating { p.resistance_at(self.temperature_rise) } else { p.resistance }
    }

    /// The base step done: the integrator has taken the winding through it (`DriveParams::winding_rates`) and left
    /// the current at its end and the electrical energy, the copper loss and the work on the shaft over it. Then the
    /// DC link of its own and the winding's heating.
    pub fn winding_stepped(&mut self, p: &DriveParams, current: f64, [electrical, copper, work]: [f64; 3]) {
        let dt = p.current_cycle;
        (self.current, self.torque) = (current, p.torque_with(current, self.d_current));
        (self.power, self.copper_loss, self.shaft_power, self.discharge_power) =
            (electrical / dt, copper / dt, work / dt, 0.0);
        // A DC link of its own. A shared one is stepped once all drives have had this step (`shared_dc_link`).
        if p.dc_link && !p.shared_dc_link {
            self.own_dc_link(p);
        }
        self.warm(p);
        self.load_power_unit(p);
    }

    /// effects.drive_overload: the step's phase current on the drive's thermal image; at the trip level the drive lowers
    /// its limit to the base load current, back at the level of the rated current it lifts it again.
    fn load_power_unit(&mut self, p: &DriveParams) {
        if let Some(o) = &p.overload {
            self.overload_image = o.image_after(self.overload_image, self.phase_current(), p.current_cycle);
            if self.overload_image >= o.trip {
                self.overloaded = true;
            } else if self.overload_image <= 1.0 {
                self.overloaded = false;
            }
        }
    }

    /// A base step with the drive disabled: no current, no torque. The winding cools and the DC link discharges
    /// through its bleeder, a shared one once all drives have had this step (`shared_dc_link`).
    ///
    /// A current still in the winding when the pulses stop freewheels through the inverter's diodes into the DC link,
    /// against its voltage: it is gone within microseconds, well inside the step, and its energy L i^2 / 2 goes to
    /// the DC link (as power fed back over this step; without the DC link simulated, back to the supply).
    pub fn disabled_step(&mut self, p: &DriveParams) {
        let freewheeling = 0.5 * p.inductance * self.current * self.current;
        (self.torque, self.torque_command, self.torque_demand, self.current, self.voltage) = (0.0, 0.0, 0.0, 0.0, 0.0);
        (self.power, self.copper_loss, self.shaft_power, self.d_current) =
            (-freewheeling / p.current_cycle, 0.0, 0.0, 0.0);
        self.warm(p);
        self.load_power_unit(p);
        if p.dc_link {
            // C dU/dt = -U / R_b: the voltage falls by exp(-dt / (R_b C)) per step, exactly. The rectifier catches it
            // at the bus level, a regenerative supply takes what is above.
            let before = self.dc_voltage;
            self.dc_voltage *= (-p.current_cycle / p.discharge_time_constant).exp();
            self.discharge_power =
                0.5 * p.dc_capacitance * (before * before - self.dc_voltage * self.dc_voltage) / p.current_cycle;
            if !p.shared_dc_link {
                self.own_dc_link(p);
            }
        }
    }

    /// One step of a DC link of its own, with the electrical power of the step.
    fn own_dc_link(&mut self, p: &DriveParams) {
        let stored = 0.5 * p.dc_capacitance * self.dc_voltage * self.dc_voltage;
        self.set_dc_link(p, dc_link_step(p, p.dc_capacitance, stored, self.power));
    }

    fn set_dc_link(&mut self, p: &DriveParams, step: DcLinkStep) {
        (self.dc_voltage, self.resistor_power, self.rectifier_power, self.mains_power, self.line_loss) =
            (step.voltage, step.resistor, step.drawn, step.mains, step.line_loss);
        self.tripped |= self.dc_voltage > p.overvoltage;
    }

    /// The copper loss of the step warms the winding; at the rated current it balances at the rated rise. Without
    /// current (a disabled drive) the same model cools it.
    ///
    /// With effects.motor_thermal_nodes (`ThermalNodes`), in units of the rise the loss would settle at (P R): the winding
    /// tau_w dT_w/dt = s P R - (T_w - T_h), the housing (tau - tau_w / s) dT_h/dt = (T_w - T_h) / s - T_h / (1 - s).
    fn warm(&mut self, p: &DriveParams) {
        if p.heating {
            let rated_loss = p.resistance_at(p.rated_rise) * p.rated_current * p.rated_current;
            let heating = p.rated_rise * self.copper_loss / rated_loss;
            let dt = p.current_cycle;
            match &p.thermal_nodes {
                None => self.temperature_rise += dt / p.thermal_time_constant * (heating - self.temperature_rise),
                Some(n) => {
                    let (s, across) = (n.winding_share, self.temperature_rise - self.housing_rise);
                    let housing_time = p.thermal_time_constant - n.winding_time_constant / s;
                    self.temperature_rise += dt / n.winding_time_constant * (s * heating - across);
                    self.housing_rise += dt / housing_time * (across / s - self.housing_rise / (1.0 - s));
                }
            }
        }
    }
}

/// What a DC link did over one base step: its voltage after it, and the powers over it of the braking resistor, of
/// the rectifier (drawn from the line), fed back into the mains and lost in the line module while feeding back.
#[derive(Debug, Clone, Copy)]
struct DcLinkStep {
    voltage: f64,
    resistor: f64,
    drawn: f64,
    mains: f64,
    line_loss: f64,
}

/// One base step of a DC link of `capacitance` that holds the energy `stored` and takes `power` out (negative:
/// a motor feeds back).
fn dc_link_step(p: &DriveParams, capacitance: f64, stored: f64, power: f64) -> DcLinkStep {
    // The capacitors take what the motors feed back and give what they draw: C U dU/dt = -P.
    // In energy, C U^2 / 2, that is exact for any step; `at` is the energy at a voltage.
    let (at, dt) = (|voltage: f64| 0.5 * capacitance * voltage * voltage, p.current_cycle);
    let energy = stored - power * dt;
    // A regenerative supply holds the bus level from above too: it feeds the excess back into the mains as far as its
    // power goes, and of that its efficiency arrives there.
    let fed = if p.regenerative { (energy - at(p.bus_voltage)).clamp(0.0, p.line_max_power * dt) } else { 0.0 };
    // Above the threshold the chopper switches the braking resistor in, which burns the excess
    // as far as its peak power goes. Beyond that the voltage keeps rising.
    let burned = (energy - fed - at(p.braking_threshold)).clamp(0.0, p.resistor_peak * dt);
    // The rectifier feeds from the line as soon as the voltage would fall below the bus level.
    let left = energy - fed - burned;
    let (voltage, drawn) = if left > at(p.bus_voltage) {
        ((2.0 * left / capacitance).sqrt(), 0.0)
    } else {
        (p.bus_voltage, at(p.bus_voltage) - left)
    };
    DcLinkStep {
        voltage,
        resistor: burned / dt,
        drawn: drawn / dt,
        mains: p.line_efficiency * fed / dt,
        line_loss: (1.0 - p.line_efficiency) * fed / dt,
    }
}

/// One base step of the DC link that all `drives` share, after their current loops: one Line Module, one DC
/// link, as on a SINAMICS S120. What a braking motor feeds back, a motoring one draws; only the net power
/// charges the capacitors. The link has the capacitance of all drives together (each brings
/// `ratings.dc_link_capacitance_uf`), one chopper with one braking resistor of the `[ratings]` powers, and one
/// trip. Every drive shows the link's voltage, the powers of resistor, rectifier and line module and the trip as its own.
///
/// Links that were apart until now (the switch came on a running machine) put their energies together.
pub fn shared_dc_link(p: &DriveParams, drives: &mut [Drive]) {
    if drives.is_empty() {
        return;
    }
    let stored: f64 = drives.iter().map(|d| 0.5 * p.dc_capacitance * d.dc_voltage * d.dc_voltage).sum();
    let power: f64 = drives.iter().map(|d| d.power).sum();
    let step = dc_link_step(p, p.dc_capacitance * drives.len() as f64, stored, power);
    for drive in drives {
        drive.set_dc_link(p, step);
    }
}

pub(crate) fn quantise(pos: f64, step: f64) -> f64 {
    if step > 0.0 { (pos / step).round() * step } else { pos }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> DriveParams {
        DriveParams {
            kv: 50.0,
            kp: 0.08,
            ki: 8.0,
            velocity_ff: 0.0,
            torque_ff: 0.0,
            kt: 0.45,
            resistance: 4.0,
            ke: 0.45,
            max_voltage: 190.0,
            // Kp = L / T and Tn = L / R for T = 0.25 ms.
            current_kp: 40.0,
            current_ki: 40.0 / 2.5e-3,
            current_cycle: 62.5e-6,
            peak_torque: 3.8,
            rated_torque: 1.27,
            rated_speed: 314.0,
            no_load_speed: 420.0,
            limit: true,
            encoder_step: 0.0,
            velocity_filter: 1.0,
            torque_filter: 1.0,
            notch: None,
            velocity_cycle: 125e-6,
            current_delay: 0,
            velocity_delay: 0,
            inductance: 0.010,
            heating: false,
            rated_rise: 100.0,
            rated_current: 1.27 / 0.45,
            thermal_time_constant: 1800.0,
            start_rise: 0.0,
            saturation: 0.0,
            max_current: 9.5,
            drive_max_current: f64::INFINITY,
            dc_link: false,
            shared_dc_link: false,
            bus_voltage: 190.0,
            dc_capacitance: 470e-6,
            braking_threshold: 250.0,
            resistor_peak: 500.0,
            overvoltage: 280.0,
            discharge_time_constant: 130.0,
            regenerative: false,
            line_max_power: 35000.0,
            line_efficiency: 0.98,
            encoder_noise: 0.0,
            current_noise: 0.0,
            noise_seed: 1,
            field_weakening: None,
            overload: None,
            thermal_nodes: None,
            precontrol: Precontrol::default(),
        }
    }

    impl Drive {
        /// The current loop and the winding at a speed held, as `Sim::step` takes the winding through a base step
        /// with the mechanics: four RK4 sub-steps of the same rates.
        fn current_loop(&mut self, p: &DriveParams, speed: f64) {
            self.current_control(p, speed);
            let (voltage, resistance, d_current) = (self.voltage, self.winding_resistance(p), self.d_current);
            let mut y = [self.current, 0.0, 0.0, 0.0];
            let mut rk = sim_core::Rk4::new(4);
            for _ in 0..4 {
                rk.step(
                    |y, dy| dy.copy_from_slice(&p.winding_rates(voltage, resistance, y[0], speed, d_current)),
                    &mut y,
                    p.current_cycle / 4.0,
                );
            }
            self.winding_stepped(p, y[0], [y[1], y[2], y[3]]);
        }
    }

    /// A shared DC link against a hand calculation. Two motors turn at 100 rad/s. One brakes with 0.45 N m: 1 A
    /// against a back-EMF of 45 V, of which the winding keeps 4 V, so it feeds back 41 W, in 0.1 s 4.1 J. The
    /// other drives with 0.225 N m: 0.5 A at 45 V + 2 V = 47 V, 23.5 W, in 0.1 s 2.35 J.
    /// Each with a link of its own: the braking drive's 470 uF go from 190 V (8.4835 J) to
    /// sqrt(2 * 12.5835 J / 470 uF) = 231.40 V, and the motoring drive's link stays at 190 V, fed from the line.
    /// Sharing one link of 2 * 470 uF, the motoring drive takes its 2.35 J from what the other feeds back: the
    /// net 1.75 J on 2 * 8.4835 J give sqrt(2 * 18.717 J / 940 uF) = 199.56 V. With the chopper at 195 V
    /// (940 uF hold 17.872 J there) the one resistor burns 18.717 - 17.872 = 0.845 J, and a trip level of
    /// 198 V with a resistor too small for the 17.5 W trips both drives.
    #[test]
    fn a_shared_dc_link_rises_by_the_net_energy_of_its_drives() {
        let run = |p: &DriveParams| {
            let mut drives = [Drive::at_rest(p, 0.0, -0.45, 0.0), Drive::at_rest(p, 0.0, 0.225, 0.0)];
            let mut burned = [0.0; 2];
            for _ in 0..(0.1 / p.current_cycle).round() as usize {
                drives.iter_mut().for_each(|d| d.current_loop(p, 100.0));
                if p.shared_dc_link {
                    shared_dc_link(p, &mut drives);
                }
                for (burned, d) in burned.iter_mut().zip(&drives) {
                    *burned += d.resistor_power * p.current_cycle;
                }
            }
            (drives, burned)
        };
        let apart = DriveParams { dc_link: true, ..params() };
        let (drives, burned) = run(&apart);
        assert!(
            (drives[0].dc_voltage - 231.40).abs() < 0.01 && drives[1].dc_voltage == 190.0 && burned == [0.0; 2],
            "{} V and {} V",
            drives[0].dc_voltage,
            drives[1].dc_voltage
        );

        let shared = DriveParams { shared_dc_link: true, ..apart.clone() };
        let (drives, burned) = run(&shared);
        let by_hand = (2.0 * (2.0 * 8.4835 + 4.1 - 2.35) / 940e-6f64).sqrt();
        assert!(
            (by_hand - 199.56).abs() < 0.01 && (drives[0].dc_voltage - by_hand).abs() < 0.01,
            "{} V, by hand {by_hand} V",
            drives[0].dc_voltage
        );
        assert!(
            drives[1].dc_voltage == drives[0].dc_voltage
                && burned == [0.0; 2]
                && !drives[0].tripped
                && !drives[1].tripped
        );
        println!("separate 231.40 V and 190 V; shared {} V, by hand {by_hand} V", drives[0].dc_voltage);

        // One chopper, one resistor: both drives report the same 0.845 J.
        let (drives, burned) = run(&DriveParams { braking_threshold: 195.0, ..shared.clone() });
        assert!(
            (drives[0].dc_voltage - 195.0).abs() < 1e-9 && drives[1].dc_voltage == drives[0].dc_voltage,
            "{} V",
            drives[0].dc_voltage
        );
        assert!((burned[0] - 0.845).abs() < 0.002 && burned[1] == burned[0], "{burned:?} J burned");
        // One trip.
        let (drives, _) =
            run(&DriveParams { braking_threshold: 195.0, resistor_peak: 5.0, overvoltage: 198.0, ..shared.clone() });
        assert!(drives[0].tripped && drives[1].tripped);
        // Links that are joined while they differ put their energies together: 231.40 V and 190 V on 470 uF
        // each are 12.5835 J + 8.4835 J on 940 uF, 211.72 V.
        let (mut drives, _) = run(&apart);
        drives.iter_mut().for_each(|d| d.power = 0.0);
        shared_dc_link(&shared, &mut drives);
        assert!(
            (drives[0].dc_voltage - 211.72).abs() < 0.01 && drives[1].dc_voltage == drives[0].dc_voltage,
            "{} V",
            drives[0].dc_voltage
        );
    }

    /// The DC link against a hand calculation. The motor turns at 100 rad/s and brakes with 0.45 N m: 1 A
    /// against a back-EMF of 45 V, of which the winding keeps R i = 4 V. The drive feeds back 41 V * 1 A =
    /// 41 W, in 0.1 s 4.1 J. 470 uF at 190 V hold C U^2 / 2 = 8.4835 J; with 4.1 J more the voltage is
    /// sqrt(2 * 12.5835 J / 470 uF) = 231.40 V.
    #[test]
    fn braking_energy_raises_the_dc_link_by_the_hand_calculated_voltage() {
        let p = DriveParams { dc_link: true, ..params() };
        let brake = |p: &DriveParams, seconds: f64| {
            let mut d = Drive::at_rest(p, 0.0, -0.45, 0.0);
            let (mut fed_back, mut burned) = (0.0, 0.0);
            for _ in 0..(seconds / p.current_cycle).round() as usize {
                d.current_loop(p, 100.0);
                fed_back -= d.power * p.current_cycle;
                burned += d.resistor_power * p.current_cycle;
            }
            (d, fed_back, burned)
        };
        let (d, fed_back, burned) = brake(&p, 0.1);
        assert!((fed_back - 4.1).abs() < 1e-9 && burned == 0.0, "{fed_back} J fed back, {burned} J burned");
        assert!((d.dc_voltage - 231.40).abs() < 0.01 && !d.tripped, "{} V", d.dc_voltage);
        assert!((0.5 * 470e-6 * (d.dc_voltage * d.dc_voltage - 190.0 * 190.0) - 4.1).abs() < 1e-9);
        // Motoring with the same current draws R i + ke w = 49 V * 1 A from the capacitors, and the rectifier
        // holds the voltage at the bus level from below: after 0.1 s (4.9 J, more than was stored) it is 190 V.
        let mut d = d;
        d.torque_command = 0.45;
        for _ in 0..1600 {
            d.current_loop(&p, 100.0);
        }
        assert_eq!(d.dc_voltage, 190.0);
        // Between 190 V and the threshold of 250 V the capacitors hold 470 uF (250^2 - 190^2) / 2 = 6.204 J.
        // In 0.3 s the motor feeds back 12.3 J: the chopper holds 250 V and the resistor burns the other 6.096 J.
        let (d, fed_back, burned) = brake(&p, 0.3);
        assert!(
            (d.dc_voltage - 250.0).abs() < 1e-9
                && (burned - (fed_back - 6.204)).abs() < 1e-9
                && (burned - 6.096).abs() < 1e-3,
            "{} V, {burned} J",
            d.dc_voltage
        );
        // A resistor that takes 20 W at most leaves 21 W to charge on: 280 V, 3.736 J above the threshold,
        // are reached 0.178 s after the 0.151 s that the threshold takes, and the drive trips.
        let weak = DriveParams { resistor_peak: 20.0, ..p.clone() };
        assert!(!brake(&weak, 0.32).0.tripped && brake(&weak, 0.34).0.tripped);
        // The voltage limit follows the DC link: at 231.4 V the winding gets sqrt(3/2) * 231.4 V = 283.4 V.
        let (mut d, ..) = brake(&p, 0.1);
        d.torque_command = 1000.0;
        d.current_loop(&p, 100.0);
        assert!((d.voltage - 283.41).abs() < 0.01, "{} V", d.voltage);
        // Switched off, the voltage stays at the bus level and nothing trips.
        let (d, ..) = brake(&DriveParams { dc_link: false, resistor_peak: 20.0, ..params() }, 0.34);
        assert!(d.dc_voltage == 190.0 && d.resistor_power == 0.0 && !d.tripped);
    }

    /// The regenerative supply against a hand calculation. The motor brakes as in the test above and feeds back 41 W. The
    /// line module holds the DC link at 190 V and feeds them into the mains: 98 % of 41 W for 0.1 s is 4.018 J there and
    /// 0.082 J lost, and neither the capacitors nor the resistor take anything. With the line module's power limited to
    /// 20 W it feeds back 20 W (1.96 J of 2 J arrive) and the other 21 W charge the capacitors: 2.1 J more than 8.4835 J
    /// is sqrt(2 * 10.5835 J / 470 uF) = 212.217 V. Motoring, the rectifier draws what the motor takes: 49 W.
    #[test]
    fn a_regenerative_supply_feeds_back_into_the_mains_up_to_its_power() {
        let run = |p: &DriveParams, torque: f64| {
            let mut d = Drive::at_rest(p, 0.0, torque, 0.0);
            let (mut mains, mut lost, mut burned, mut drawn) = (0.0, 0.0, 0.0, 0.0);
            for _ in 0..1600 {
                d.current_loop(p, 100.0);
                (mains, lost, burned, drawn) = (
                    mains + d.mains_power * p.current_cycle,
                    lost + d.line_loss * p.current_cycle,
                    burned + d.resistor_power * p.current_cycle,
                    drawn + d.rectifier_power * p.current_cycle,
                );
            }
            (d, mains, lost, burned, drawn)
        };
        let p = DriveParams { dc_link: true, regenerative: true, ..params() };
        let (d, mains, lost, burned, drawn) = run(&p, -0.45);
        assert!(
            d.dc_voltage == 190.0
                && (mains - 4.018).abs() < 1e-9
                && (lost - 0.082).abs() < 1e-9
                && burned == 0.0
                && drawn == 0.0,
            "{} V, {mains} J, {lost} J",
            d.dc_voltage
        );
        let (d, mains, _, burned, _) = run(&DriveParams { line_max_power: 20.0, ..p.clone() }, -0.45);
        assert!(
            (d.dc_voltage - 212.217).abs() < 1e-3 && (mains - 1.96).abs() < 1e-9 && burned == 0.0,
            "{} V, {mains} J",
            d.dc_voltage
        );
        let (d, mains, _, _, drawn) = run(&p, 0.45);
        assert!(d.dc_voltage == 190.0 && mains == 0.0 && (drawn - 4.9).abs() < 1e-9, "{drawn} J drawn");
    }

    /// Sensor noise against a hand calculation. Each encoder reading is off by an independent error with
    /// standard deviation s. The speed is the difference of two readings over the velocity cycle T, so its
    /// error has s sqrt(2) / T. The velocity filter y += a (x - y) turns the readings n into
    /// y = (a / T) (n[k] - a sum over j >= 1 of (1 - a)^(j - 1) n[k - j]), with the variance
    /// (a s / T)^2 (1 + a^2 / (1 - (1 - a)^2)) = (a s / T)^2 * 2 / (2 - a): the filtered speed has the
    /// standard deviation (s / T) a sqrt(2 / (2 - a)).
    /// Two counts of a 2^20 encoder are s = 1.198e-5 rad, s / T = 0.0959 rad/s at T = 125 us: 0.1356 rad/s
    /// unfiltered, and with a = 1 - exp(-125 / 200) = 0.4647 filtered 0.0959 * 0.4647 * 1.1414 = 0.0509 rad/s.
    #[test]
    fn speed_from_a_noisy_encoder_has_the_hand_calculated_rms() {
        let step = TAU / 1048576.0;
        let rms = |filter: f64| {
            let p = DriveParams { encoder_noise: 2.0 * step, velocity_filter: filter, ..params() };
            let mut d = Drive::at_rest(&p, 0.0, 0.0, 0.0);
            let mut sum = 0.0;
            for _ in 0..200_000 {
                d.read_encoder(&p, 0.0);
                d.velocity_loop(&p, 0.0);
                sum += d.speed * d.speed;
            }
            (sum / 200_000.0f64).sqrt()
        };
        let (raw, filtered) = (rms(1.0), rms(1.0 - (-125.0f64 / 200.0).exp()));
        assert!((raw - 0.1356).abs() < 0.02 * 0.1356, "{raw} rad/s unfiltered");
        assert!((filtered - 0.0509).abs() < 0.02 * 0.0509, "{filtered} rad/s filtered");
        // Current noise reaches the voltage through the current controller's gain: 0.01 A * 40 V/A.
        let p = DriveParams { current_noise: 0.01, ..params() };
        let mut d = Drive::at_rest(&p, 0.0, 0.0, 0.0);
        d.current_loop(&p, 0.0);
        assert!(d.voltage != 0.0 && d.voltage.abs() < 6.0 * 0.4, "{} V", d.voltage);
        // Without noise the encoder is exact and nothing is drawn from the generator.
        let mut d = Drive::at_rest(&params(), 0.0, 0.0, 0.0);
        d.read_encoder(&params(), 0.0);
        d.velocity_loop(&params(), 0.0);
        assert!(d.speed == 0.0 && d.noise.0 == 1);
    }

    #[test]
    fn position_loop_is_kv_times_following_error_plus_feedforward() {
        let mut p = params();
        let mut d = Drive { pos_set: 1.0, vel_set: 10.0, ..Drive::default() };
        d.position_loop(&p, 0.9, 1e-3);
        assert!((d.following_error - 0.1).abs() < 1e-15 && (d.speed_command - 5.0).abs() < 1e-12);
        p.velocity_ff = 1.0;
        d.position_loop(&p, 0.9, 1e-3);
        assert!((d.speed_command - 15.0).abs() < 1e-12);
    }

    /// pos 1 rad, 10 rad/s, 100 rad/s^2, 3 ms after the profile cycle: 1 + 10 * 0.003 + 100 * 0.003^2 / 2 = 1.03045 rad
    /// with fine interpolation, the setpoint 1 rad held without. Measured at 1 rad: following error 0.03045 rad and
    /// Kv 50 * 0.03045 = 1.5225 rad/s; the speed feedforward at 1.0 runs on to 10 + 100 * 0.003 = 10.3 rad/s.
    #[test]
    fn fine_interpolation_runs_the_setpoint_on_between_profile_cycles() {
        let mut p = DriveParams { velocity_ff: 1.0, ..params() };
        let set = Drive { pos_set: 1.0, vel_set: 10.0, acc_set: 100.0, since_setpoint: 3e-3, ..Drive::default() };
        let mut held = set.clone();
        held.position_loop(&p, 1.0, 1e-3);
        assert_eq!((held.position_setpoint, held.following_error), (1.0, 0.0));
        p.precontrol.fine_interpolation = true;
        let mut d = set.clone();
        d.position_loop(&p, 1.0, 1e-3);
        assert!((d.position_setpoint - 1.03045).abs() < 1e-12 && (d.following_error - 0.03045).abs() < 1e-12);
        assert!((d.position_part - 1.5225).abs() < 1e-10 && (d.speed_command - (1.5225 + 10.3)).abs() < 1e-10);
        // At the profile cycle itself the two agree: nothing changes where the position and profile cycles are equal.
        let (mut a, mut b) = (Drive { since_setpoint: 0.0, ..set.clone() }, Drive { since_setpoint: 0.0, ..set });
        a.position_loop(&p, 0.9, 1e-3);
        b.position_loop(&DriveParams { velocity_ff: 1.0, ..params() }, 0.9, 1e-3);
        assert_eq!(a.speed_command, b.speed_command);
    }

    /// A characteristic of (10 rad/s, 0.2 N m) and (100 rad/s, 0.3 N m): 5 rad/s gives 0.2 * 5 / 10 = 0.1 N m (from
    /// (0, 0)), -55 rad/s gives -(0.2 + 0.1 * 45 / 90) = -0.25 N m, 200 rad/s the last point's 0.3 N m, standstill 0.
    /// In the velocity loop it is the feedforward at the speed setpoint, 4 + 1000 * 1 ms = 5 rad/s: 0.1 N m, beside the
    /// rigid model's 0.3 N m.
    #[test]
    fn the_friction_feedforward_is_the_characteristic_at_the_speed_setpoint() {
        let c = Precontrol { friction: vec![[10.0, 0.2], [100.0, 0.3]], ..Precontrol::default() };
        assert!((c.friction_at(5.0) - 0.1).abs() < 1e-15 && (c.friction_at(-55.0) + 0.25).abs() < 1e-15);
        assert_eq!(
            (c.friction_at(200.0), c.friction_at(0.0), Precontrol::default().friction_at(50.0)),
            (0.3, 0.0, 0.0)
        );
        let p = DriveParams { torque_ff: 1.0, precontrol: c, ..params() };
        let mut d = Drive { vel_set: 4.0, acc_set: 1000.0, since_setpoint: 1e-3, torque_ff: 0.3, ..Drive::default() };
        d.velocity_loop(&p, 0.0);
        assert!((d.feedforward_torque - 0.4).abs() < 1e-12, "{}", d.feedforward_torque);
        assert!((d.controller_torque - 0.0).abs() < 1e-12 && (d.torque_demand - 0.4).abs() < 1e-12);
    }

    /// Balancing with 1 ms at a 1 ms position cycle: a setpoint step from 0 to 1 rad arrives as 1 - e^-1 = 0.632121 rad
    /// after one cycle and 1 - e^-2 = 0.864665 rad after two.
    #[test]
    fn balancing_lags_the_position_setpoint_by_its_time_constant() {
        let mut p = params();
        p.precontrol.balance_time = 1e-3;
        let mut d = Drive { pos_set: 1.0, ..Drive::default() };
        d.position_loop(&p, 0.0, 1e-3);
        assert!((d.position_setpoint - 0.632_120_558_8).abs() < 1e-9 && d.following_error == d.position_setpoint);
        d.position_loop(&p, 0.0, 1e-3);
        assert!((d.position_setpoint - 0.864_664_716_8).abs() < 1e-9);
    }

    #[test]
    fn velocity_loop_is_pi_with_torque_feedforward() {
        let mut p = params();
        p.torque_ff = 1.0;
        let mut d = Drive { position_part: 10.0, torque_ff: 0.3, ..Drive::default() };
        d.velocity_loop(&p, 4.0);
        assert!((d.torque_command - (0.08 * 6.0 + 0.3)).abs() < 1e-12);
        d.velocity_loop(&p, 4.0);
        assert!((d.torque_command - (0.08 * 6.0 + 8.0 * 6.0 * 125e-6 + 0.3)).abs() < 1e-12);
    }

    #[test]
    fn torque_limit_follows_the_torque_speed_curve() {
        let p = params();
        assert_eq!(p.torque_range(100.0), (-3.8, 3.8));
        // Half way between rated and no-load speed, half the torque is left for accelerating.
        let (low, high) = p.torque_range(367.0);
        assert!((high - 1.9).abs() < 1e-12 && low == -3.8);
        assert_eq!(p.torque_range(-500.0), (-0.0, 3.8));
        let mut d = Drive { position_part: 1000.0, ..Drive::default() };
        d.velocity_loop(&p, 0.0);
        assert_eq!(d.torque_command, 3.8);
        let before = d.integral;
        d.velocity_loop(&p, 0.0);
        assert_eq!(d.integral, before, "the integrator winds up at the limit");
        let unlimited = DriveParams { limit: false, ..params() };
        let mut d = Drive { position_part: 1000.0, ..Drive::default() };
        d.velocity_loop(&unlimited, 0.0);
        assert!(d.torque_command > 3.8);
        assert_eq!(unlimited.torque_curve(100.0), (-3.8, 3.8));
    }

    /// effects.drive_current_limit: a drive of 5 A holds the 0.45 N m/A motor to 0.45 * 5 = 2.25 N m, below the
    /// motor's 3.8 N m, and its velocity loop stops integrating there; the current setpoint is then 2.25 / 0.45 = 5 A.
    /// A drive of 10.9 A would allow 4.905 N m: the motor's 3.8 N m stays the limit. With the motor's limit off the
    /// drive's still holds.
    #[test]
    fn the_drive_current_limit_holds_the_torque_to_the_lower_of_motor_and_drive() {
        let small = DriveParams { drive_max_current: 5.0, ..params() };
        assert_eq!(small.torque_range(100.0), (-2.25, 2.25));
        assert_eq!(small.torque_range(367.0), (-2.25, 1.9), "the torque-speed curve below the drive's limit");
        let mut d = Drive { position_part: 1000.0, ..Drive::default() };
        d.velocity_loop(&small, 0.0);
        let before = d.integral;
        d.velocity_loop(&small, 0.0);
        assert!(
            d.torque_command == 2.25
                && d.integral == before
                && (small.current_for(d.torque_command) - 5.0).abs() < 1e-12
        );
        assert_eq!(DriveParams { drive_max_current: 10.9, ..params() }.torque_range(100.0), (-3.8, 3.8));
        assert_eq!(DriveParams { limit: false, ..small.clone() }.torque_range(100.0), (-2.25, 2.25));
        // What the motor and drive can give, which "motor-peak" judges against, is the motor's.
        assert_eq!(small.torque_curve(100.0), (-3.8, 3.8));
    }

    #[test]
    fn velocity_feedforward_is_interpolated_between_profile_cycles() {
        let mut p = params();
        p.velocity_ff = 1.0;
        let mut d = Drive { vel_set: 10.0, acc_set: 1000.0, ..Drive::default() };
        d.velocity_loop(&p, 0.0);
        assert!((d.speed_command - 10.0).abs() < 1e-12);
        // The controller advances the time since the setpoints, a base step at a time (two per velocity cycle).
        d.since_setpoint += 125e-6;
        d.velocity_loop(&p, 0.0);
        assert!((d.speed_command - (10.0 + 1000.0 * 125e-6)).abs() < 1e-12);
    }

    #[test]
    fn current_follows_its_setpoint_with_the_tuned_time_constant() {
        let p = params();
        let mut d = Drive { torque_command: 0.45, ..Drive::default() };
        // Kp = L / T cancels the winding's own time constant: after T = 0.25 ms (4 steps) the
        // current is at about 63 % of its 1 A setpoint, and it settles there without overshoot.
        let mut peak = 0f64;
        for step in 1..=200 {
            d.current_loop(&p, 0.0);
            peak = peak.max(d.current);
            if step == 4 {
                assert!((d.current - 0.632).abs() < 0.05, "{} A after one time constant", d.current);
            }
        }
        assert!((d.current - 1.0).abs() < 1e-4 && peak < 1.01, "ends at {} A, peak {peak} A", d.current);
        assert!((d.torque - 0.45).abs() < 1e-4 && (d.voltage - 4.0).abs() < 1e-3);
    }

    #[test]
    fn computation_delay_makes_the_current_loop_unstable_at_the_calculated_limit() {
        // With Kp = L / T the open loop is 1 / (T s): crossover at 1 / T with 90 degrees of phase left. A dead
        // time d takes d / T rad of it, and holding the voltage for a cycle another half cycle's worth. The
        // loop is stable up to d = (pi / 2) T - dt / 2 = 0.36 ms, which is 5.8 cycles of 62.5 us.
        let largest_current = |cycles: usize| {
            let p = DriveParams { current_delay: cycles, max_voltage: f64::INFINITY, ..params() };
            let mut d = Drive { torque_command: 0.45, ..Drive::default() };
            let mut late = 0f64;
            for step in 0..4000 {
                d.current_loop(&p, 0.0);
                if step >= 3000 {
                    late = late.max((d.current - 1.0).abs());
                }
            }
            late
        };
        assert!(largest_current(1) < 1e-6 && largest_current(4) < 1e-3, "{} A off with 4 cycles", largest_current(4));
        assert!(largest_current(7) > 10.0, "only {} A off with 7 cycles", largest_current(7));
    }

    #[test]
    fn the_torque_setpoint_acts_one_velocity_cycle_late() {
        let p = DriveParams { velocity_delay: 1, ..params() };
        let mut d = Drive { position_part: 10.0, ..Drive::default() };
        d.velocity_loop(&p, 4.0);
        let first = d.torque_command;
        d.velocity_loop(&p, 4.0);
        // The first cycle has nothing older to act; the second acts what the first computed.
        assert!(
            (first - 0.08 * 6.0).abs() < 1e-12 && (d.torque_command - 0.08 * 6.0).abs() < 1e-12,
            "{first}, {}",
            d.torque_command
        );
        d.velocity_loop(&p, 4.0);
        assert!((d.torque_command - (0.08 * 6.0 + 8.0 * 6.0 * 125e-6)).abs() < 1e-12);
    }

    #[test]
    fn the_winding_warms_to_the_rated_rise_at_the_rated_current() {
        // A short thermal time constant, so the test runs in a moment: 0.5 s is 8000 current cycles.
        let p = DriveParams { heating: true, thermal_time_constant: 0.5, max_voltage: f64::INFINITY, ..params() };
        let run = |current: f64, seconds: f64| {
            let mut d = Drive { torque_command: p.kt * current, ..Drive::default() };
            for _ in 0..(seconds / p.current_cycle) as usize {
                d.current_loop(&p, 0.0);
            }
            d
        };
        // The cold winding has less resistance and so less loss: the rise follows tau dT/dt = 100 (1 + 0.0039 T) / 1.39 - T,
        // which approaches 100 K with the time constant 1.39 tau. After one tau that is 100 (1 - exp(-1 / 1.39)) = 51.3 K.
        let (early, late) = (run(p.rated_current, 0.5), run(p.rated_current, 6.0));
        assert!((early.temperature_rise - 51.3).abs() < 0.5, "{} K after one time constant", early.temperature_rise);
        assert!((late.temperature_rise - 100.0).abs() < 0.2, "{} K in the end", late.temperature_rise);
        // The warm winding has 39 % more resistance and needs that much more voltage for the same current.
        assert!((late.voltage / (4.0 * p.rated_current) - 1.39).abs() < 0.005, "{} V", late.voltage);
        // Half the current: a quarter of the loss, less the resistance that did not rise as far. By hand:
        // q = 100 / 1.39 / 4 = 17.99 K, settled at q / (1 - 0.0039 q) = 19.34 K.
        let half = run(0.5 * p.rated_current, 6.0);
        assert!(
            (half.temperature_rise - 19.34).abs() < 0.05
                && (p.settled_rise(0.5 * p.rated_current) - 19.34).abs() < 0.01,
            "{} K",
            half.temperature_rise
        );
        // Switched off, the winding stays as it is.
        let cold = DriveParams { heating: false, ..p.clone() };
        let mut d = Drive { torque_command: 1.27, ..Drive::default() };
        for _ in 0..8000 {
            d.current_loop(&cold, 0.0);
        }
        assert_eq!(d.temperature_rise, 0.0);
    }

    #[test]
    fn saturation_takes_torque_at_high_current_only() {
        let p = DriveParams { saturation: 1.0 - 0.40 / 0.45, ..params() };
        assert!((p.torque_of(9.5) - 0.40 * 9.5).abs() < 1e-12 && (p.torque_of(-9.5) + 0.40 * 9.5).abs() < 1e-12);
        // At a tenth of the maximum current a hundredth of the loss.
        assert!((p.torque_of(0.95) / (0.45 * 0.95) - (1.0 - (1.0 - 0.40 / 0.45) / 100.0)).abs() < 1e-12);
        assert_eq!(params().torque_of(9.5), 0.45 * 9.5);
    }

    /// The 1FK2205-4AF of the catalogue: 6 N m at 4.7 A at standstill, 18 N m at its maximum current of 15.1 A.
    fn saturating() -> DriveParams {
        let kt = 6.0 / 4.7;
        DriveParams {
            kt,
            ke: kt,
            saturation: 1.0 - 18.0 / 15.1 / kt,
            max_current: 15.1,
            max_voltage: f64::INFINITY,
            peak_torque: 18.0,
            ..params()
        }
    }

    /// The current setpoint is the inverse of the saturation curve. Commanding the maximum torque takes the
    /// maximum current; with the setpoint at torque / kt it was 18 / 1.277 = 14.1 A, which gives 16.9 N m.
    #[test]
    fn the_maximum_torque_takes_the_maximum_current() {
        let p = saturating();
        let mut d = Drive { torque_command: 18.0, ..Drive::default() };
        for _ in 0..2000 {
            d.current_loop(&p, 0.0);
        }
        assert!((d.current - 15.1).abs() < 1e-6 && (d.torque - 18.0).abs() < 1e-6, "{} A, {} N m", d.current, d.torque);
        // More than the motor has is limited to the maximum current.
        assert!((p.current_for(30.0) - 15.1).abs() < 1e-12 && (p.current_for(-30.0) + 15.1).abs() < 1e-12);
        // A drive that starts holding a load starts with the current that holds it.
        assert!((Drive::at_rest(&p, 0.0, -18.0, 0.0).current + 15.1).abs() < 1e-9);
        for current in [0.5, 4.7, 10.0, -12.0, 15.1] {
            assert!((p.current_for(p.torque_of(current)) - current).abs() < 1e-9, "{current} A");
        }
        // A curve that loses more than a third of its torque constant turns down before the maximum current:
        // with 40 % at 1 / sqrt(3 * 0.4) = 0.913 of it. More current would give less torque, so that is the limit.
        let steep = DriveParams { saturation: 0.4, ..p.clone() };
        assert!((steep.current_for(1e3) - 15.1 / 1.2f64.sqrt()).abs() < 1e-12);
        assert!(steep.torque_of(steep.current_for(1e3)) >= steep.torque_of(15.1));
    }

    /// With saturation the back-EMF constant falls with the torque constant. At the maximum current and
    /// 100 rad/s the winding needs R i + (18 / 15.1) * 100 = 60.4 + 119.2 V, and it draws the copper loss
    /// R i^2 = 912 W plus the shaft power 18 N m * 100 rad/s. With the back-EMF at kt w it drew 128 W more
    /// that went nowhere.
    #[test]
    fn under_saturation_the_electrical_power_is_copper_loss_plus_shaft_power() {
        let p = saturating();
        let mut d = Drive { torque_command: 18.0, ..Drive::default() };
        for _ in 0..4000 {
            d.current_loop(&p, 100.0);
        }
        assert!((d.voltage - (60.4 + 1800.0 / 15.1)).abs() < 1e-3, "{} V", d.voltage);
        assert!(
            (d.power - (4.0 * 15.1 * 15.1 + d.torque * 100.0)).abs() < 1e-2 && (d.torque - 18.0).abs() < 1e-6,
            "{} W, {} N m",
            d.power,
            d.torque
        );
    }

    #[test]
    fn back_emf_costs_voltage_and_the_bus_limits_torque_at_speed() {
        let p = params();
        // At 100 rad/s the same 1 A needs R i + ke w = 4 V + 45 V.
        let mut d = Drive { torque_command: 0.45, ..Drive::default() };
        for _ in 0..2000 {
            d.current_loop(&p, 100.0);
        }
        assert!((d.current - 1.0).abs() < 1e-4 && (d.voltage - 49.0).abs() < 1e-3, "{} A, {} V", d.current, d.voltage);
        // At 400 rad/s the back-EMF is 180 V of the 190 V bus: 10 V / 4 Ohm = 2.5 A is all that is left.
        let mut d = Drive { torque_command: 3.8, ..Drive::default() };
        for _ in 0..2000 {
            d.current_loop(&p, 400.0);
        }
        assert!((d.current - 2.5).abs() < 1e-6 && d.voltage == 190.0, "{} A, {} V", d.current, d.voltage);
        // The integrator did not wind up meanwhile: with the setpoint gone the current is back within 2 ms.
        d.torque_command = 0.0;
        for _ in 0..32 {
            d.current_loop(&p, 400.0);
        }
        assert!(d.current.abs() < 0.2, "{} A", d.current);
        // Without the limit and without back-EMF the full current flows at any speed.
        let free = DriveParams { ke: 0.0, max_voltage: f64::INFINITY, ..params() };
        let mut d = Drive { torque_command: 3.8, ..Drive::default() };
        for _ in 0..2000 {
            d.current_loop(&free, 400.0);
        }
        assert!((d.torque - 3.8).abs() < 1e-6);
    }

    #[test]
    fn encoder_resolution_quantises_position_and_speed() {
        let mut p = params();
        p.encoder_step = TAU / 4096.0;
        let mut d = Drive::at_rest(&p, 0.0, 0.0, 0.0);
        d.pos_set = 0.4 * p.encoder_step;
        d.read_encoder(&p, 0.0);
        d.position_loop(&p, d.encoder(), 1e-3);
        assert_eq!(d.following_error, 0.4 * p.encoder_step);
        d.read_encoder(&p, 0.3 * p.encoder_step);
        d.position_loop(&p, d.encoder(), 1e-3);
        assert_eq!(d.following_error, 0.4 * p.encoder_step, "a move below one count is invisible");
        // One count per velocity cycle reads as step / cycle, whatever the true speed is.
        d.read_encoder(&p, 1.2 * p.encoder_step);
        d.velocity_loop(&p, 123.0);
        assert!((d.speed - p.encoder_step / 125e-6).abs() < 1e-9);
    }

    #[test]
    fn notch_removes_its_centre_frequency_and_passes_the_rest() {
        let gain_at = |frequency: f64| {
            let [b0, b1, b2, a1, a2] = notch_coefficients(60.0, 20.0, 125e-6);
            let (mut x, mut y, mut peak) = ([0.0; 2], [0.0; 2], 0f64);
            for n in 0..80_000 {
                let input = (TAU * frequency * n as f64 * 125e-6).sin();
                let out = b0 * input + b1 * x[0] + b2 * x[1] - a1 * y[0] - a2 * y[1];
                (x, y) = ([input, x[0]], [out, y[0]]);
                if n > 60_000 {
                    peak = peak.max(out.abs());
                }
            }
            peak
        };
        assert!(gain_at(60.0) < 0.01, "{}", gain_at(60.0));
        assert!((gain_at(5.0) - 1.0).abs() < 0.02 && (gain_at(600.0) - 1.0).abs() < 0.02);
    }

    #[test]
    fn a_drive_starts_holding_its_load() {
        let mut p = params();
        p.torque_ff = 1.0;
        let mut d = Drive::at_rest(&p, 2.0, 0.14, 0.137);
        d.read_encoder(&p, 2.0);
        d.position_loop(&p, d.encoder(), 1e-3);
        d.velocity_loop(&p, 0.0);
        d.current_loop(&p, 0.0);
        assert!((d.torque - 0.14).abs() < 1e-12 && (d.torque_command - 0.14).abs() < 1e-12);
    }

    /// The test motor with field weakening: 4 pole pairs, L_d = L_q = 10 mH, and a winding resistance small enough to
    /// leave out of the hand calculation.
    fn weakening() -> DriveParams {
        DriveParams {
            resistance: 1e-6,
            field_weakening: Some(FieldWeakening { pole_pairs: 4.0, d_inductance: 0.010 }),
            ..params()
        }
    }

    /// effects.field_weakening against a hand calculation. At 500 rad/s the back-EMF is 0.45 * 500 = 225 V, above the
    /// 190 V the winding gets: without field weakening there is no torque (the fade ends at 420 rad/s). With it, w L =
    /// 4 * 500 * 0.010 = 20 Ohm on both axes, and R = 0, the voltage circle is (225 + 20 i_d)^2 + (20 i_q)^2 <= 190^2,
    /// the current circle i_q^2 + i_d^2 <= 9.5^2. i_q is largest where they cross: 225^2 + 9000 i_d + 400 * 9.5^2 = 190^2,
    /// i_d = -50625 / 9000 = -5.625 A, i_q = sqrt(90.25 - 31.640625) = 7.6557 A, 0.45 * 7.6557 = 3.4451 N m. The current
    /// loop gets there: the drive sets i_d = -5.625 A for that setpoint and the voltage vector sits on the circle.
    #[test]
    fn field_weakening_gives_the_hand_calculated_torque_above_the_voltage_limit() {
        let p = weakening();
        assert_eq!(DriveParams { field_weakening: None, ..p.clone() }.torque_curve(500.0).1, 0.0);
        let (low, high) = p.torque_curve(500.0);
        assert!((high - 3.4451).abs() < 1e-3, "{high} N m motoring at 500 rad/s");
        // Braking the same: with R = 0 the voltage circle does not depend on the sign of i_q. With R = 4 Ohm the back-EMF
        // is partly spent in the resistance while braking, and braking has more torque than motoring.
        let back = p.torque_curve(-500.0);
        assert!((low + high).abs() < 1e-6 && back == (-high, -low), "{low} N m braking, {back:?} backwards");
        let resistive = DriveParams { resistance: 4.0, ..p.clone() };
        let (braking, motoring) = resistive.torque_curve(500.0);
        assert!(-braking > motoring, "{braking} N m braking, {motoring} N m motoring");
        // Below base speed field weakening changes nothing.
        assert_eq!(p.torque_curve(100.0), (-3.8, 3.8));
        assert!((p.d_current_for(7.6557, 500.0, 190.0, 9.5) + 5.625).abs() < 1e-3);
        assert_eq!(p.d_current_for(1.0, 100.0, 190.0, 9.5), 0.0, "45 V and 4 V reach");

        let mut d = Drive { torque_command: high, ..Drive::default() };
        for _ in 0..4000 {
            d.current_loop(&p, 500.0);
        }
        let u_d = -20.0 * d.current;
        assert!(
            (d.current - 7.6557).abs() < 1e-3
                && (d.d_current + 5.625).abs() < 1e-3
                && (d.voltage.hypot(u_d) - 190.0).abs() < 0.1
                && (d.torque - high).abs() < 1e-3,
            "{} A, {} A, {} V, {} V",
            d.current,
            d.d_current,
            d.voltage,
            u_d
        );
        // The RMS phase current is the hypotenuse of both: 9.5 A, the current circle.
        assert!((d.phase_current() - 9.5).abs() < 1e-3);
        // The winding's energy closes with the d axis: electrical power = copper loss + shaft power (steady, R = 0).
        assert!((d.power - d.copper_loss - d.shaft_power).abs() < 1e-3 * d.shaft_power, "{} W", d.power);
    }

    /// The d-q model on a motor Siemens publishes R, L, ke and the pole pairs of: the 1FK7042-2AF7 on S120
    /// (catalog/siemens/motors_s120.csv and motors_s120_electrical.csv), at the DC link of 600 V its data sheet holds
    /// for: kt = 3.0 N m / 2.2 A = 1.364 N m/A, R = 3 * 4.67 Ohm, L = 3 * 35 mH, 4 pole pairs, 7.6 A, 10.5 N m at most.
    ///
    /// The S1 and peak torque-speed curves Siemens prints only as graphs (in the data sheets and manuals, not in the
    /// catalogue files), so the points checked are those the catalogue has. The S1 point, 2.6 N m at the rated 2.0 A
    /// and 314.16 rad/s: the model gives kt * 2.0 A = 2.73 N m, 5 % above (the tolerance stated: 6 %, about the
    /// difference between the static and the rated torque constant, 1.364 against 1.3 N m/A), and needs no field weakening
    /// there: R i + kt w = 456.6 V and w L i = 263.9 V make 527 V of the sqrt(3/2) * 600 = 734.8 V. Without field weakening
    /// the torque is gone at 734.8 / 1.364 = 538.9 rad/s; Siemens gives the motor up to 670.2 rad/s on S120 (n_max,inv)
    /// and the model with field weakening still has torque there. Towards high speed the best d current tends to the
    /// characteristic current -psi / L_d = -kt / (p L_d) = -1.364 / (4 * 0.105) = -3.247 A.
    /// n_max,inv itself is where the motor's no-load voltage, sqrt(2) * 0.8594 V s/rad * 670.2 rad/s = 814.6 V peak line to
    /// line, reaches the 820 V at which the S120 trips: the speed above which field weakening must not be lost.
    #[test]
    fn the_d_q_model_holds_the_published_points_of_a_1fk7() {
        let kt = 3.0 / 2.2;
        let fw = FieldWeakening { pole_pairs: 4.0, d_inductance: 3.0 * 0.035 };
        let p = DriveParams {
            kt,
            ke: kt,
            resistance: 3.0 * 4.67,
            inductance: 3.0 * 0.035,
            max_voltage: WINDING_PER_DC_LINK * 600.0,
            max_current: 7.6,
            peak_torque: 10.5,
            rated_speed: 314.16,
            no_load_speed: WINDING_PER_DC_LINK * 600.0 / kt,
            field_weakening: Some(fw),
            ..params()
        };
        let rated = p.torque_of(2.0);
        assert!((rated / 2.6 - 1.0).abs() < 0.06 && p.torque_curve(314.16).1 >= rated, "{rated} N m");
        assert_eq!(p.d_current_for(2.0, 314.16, p.max_voltage, 7.6), 0.0);
        let old = DriveParams { field_weakening: None, ..p.clone() };
        assert!(old.torque_curve(670.2).1 == 0.0 && p.torque_curve(670.2).1 > rated, "{:?}", p.torque_curve(670.2));
        // At 942.5 rad/s, the mechanical limit: the largest q current on both circles for each d current, by bisection
        // on the voltage, and the d current where it is largest, by stepping through them.
        let w = 4.0 * 942.5;
        let q_at = |d: f64| {
            let (mut lo, mut hi) = (0.0, (7.6f64 * 7.6 - d * d).max(0.0).sqrt());
            for _ in 0..40 {
                let mid = 0.5 * (lo + hi);
                let uq = p.resistance * mid + kt * 942.5 + w * fw.d_inductance * d;
                let ud = p.resistance * d - w * p.inductance * mid;
                if uq.hypot(ud) <= p.max_voltage { lo = mid } else { hi = mid }
            }
            lo
        };
        let best = (0..=7600).map(|k| -(k as f64) * 1e-3).max_by(|a, b| q_at(*a).total_cmp(&q_at(*b))).unwrap();
        assert!((best + 3.247).abs() < 0.01 * 3.247, "{best} A");
        assert!((p.torque_curve(942.5).1 - kt * q_at(best)).abs() < 1e-3, "{:?}", p.torque_curve(942.5));
        assert!((2f64.sqrt() * 0.8594 * 670.2 / 820.0 - 1.0).abs() < 0.01);
    }

    /// effects.drive_overload against a hand calculation, with the 3 A S120 Motor Module: rated 3 A, base load 2.6 A,
    /// 9 A for 0.25 s from the rated load, and a thermal image of 10 s. The trip level is 9 - 8 exp(-0.025) = 1.19752.
    /// A cold drive asked for 6 A, twice its rated current, fills its image as 4 (1 - exp(-t / 10 s)): it reaches the
    /// trip after -10 s ln(1 - 1.19752 / 4) = 3.558 s, and holds 2.6 A from then. The image then falls towards
    /// (2.6 / 3)^2 = 0.7511 and is back at 1 after -10 s ln((1 - 0.7511) / (1.19752 - 0.7511)) = 5.842 s, when the drive
    /// gives the 6 A again.
    #[test]
    fn a_step_at_twice_the_rated_current_clamps_after_the_hand_calculated_time() {
        let trip = 9.0 - 8.0 * (-0.025f64).exp();
        assert!((trip - 1.19752).abs() < 1e-5);
        let o = Overload { rated_current: 3.0, base_current: 2.6, max_current: 9.0, time_constant: 10.0, trip };
        let p = DriveParams { overload: Some(o), torque_ff: 1.0, drive_max_current: 9.0, ..params() };
        let mut d = Drive { torque_ff: 0.45 * 6.0, ..Drive::default() };
        let (mut clamped, mut released, mut t) = (None, None, 0.0);
        while t < 10.0 {
            d.velocity_loop(&p, 0.0);
            for _ in 0..2 {
                d.current_loop(&p, 0.0);
                t += p.current_cycle;
            }
            if clamped.is_none() && d.torque_command < 0.45 * 6.0 - 1e-9 {
                clamped = Some(t);
                assert!((d.torque_command - 0.45 * 2.6).abs() < 1e-12);
            }
            if clamped.is_some() && released.is_none() && d.torque_command == 0.45 * 6.0 {
                released = Some(t);
            }
        }
        let (clamped, released) = (clamped.unwrap(), released.unwrap());
        assert!((clamped - 3.558).abs() < 2e-3, "clamps after {clamped} s");
        // The image passes the trip by up to a velocity cycle (125 us at 0.28 /s), worth 22 s per unit on the way down.
        assert!((released - clamped - 5.842).abs() < 3e-3, "released {} s later", released - clamped);
        // Off, nothing clamps.
        let off = DriveParams { overload: None, ..p.clone() };
        let mut d = Drive { torque_ff: 0.45 * 6.0, ..Drive::default() };
        for _ in 0..40_000 {
            d.velocity_loop(&off, 0.0);
            d.current_loop(&off, 0.0);
        }
        assert!(d.torque_command == 0.45 * 6.0 && !d.overloaded && d.overload_image == 0.0);
    }

    /// The image's peak when a cycle repeats (`analysis::repeating_peak`), by hand: 1.5 times the rated current held for
    /// a 1 s pass from a cold image settles at 2.25. Twice the rated current for 0.5 s and nothing for 0.5 s, tau 10 s:
    /// the repeating start is x0 = 4 (1 - e) e / (1 - e^2) with e = exp(-0.05), 1.95001, and the peak, at 0.5 s,
    /// x0 e + 4 (1 - e) = 2.04999, a little above the mean of (i / I)^2, 2.
    #[test]
    fn the_overload_image_of_a_repeated_cycle_peaks_at_the_hand_calculated_level() {
        let o = Overload { rated_current: 1.0, base_current: 0.85, max_current: 3.0, time_constant: 10.0, trip: 1.2 };
        let pass = |current: &dyn Fn(f64) -> f64| {
            let (mut x, mut samples) = (0.0, Vec::new());
            for k in 1..=1000 {
                x = o.image_after(x, current(k as f64 * 1e-3 - 0.5e-3), 1e-3);
                samples.push((k as f64 * 1e-3, x));
            }
            let end = samples.pop().unwrap();
            crate::analysis::repeating_peak(&o, 0.0, &samples, end)
        };
        assert!((pass(&|_| 1.5) - 2.25).abs() < 1e-9);
        assert!((pass(&|t| if t < 0.5 { 2.0 } else { 0.0 }) - 2.04999).abs() < 1e-4);
    }

    /// effects.motor_thermal_nodes against a hand calculation. The winding has a quarter of the resistance (s = 0.25) and
    /// 60 s against the housing; the motor 1800 s, as the one node. At a loss that the one node settles at 100 K the
    /// winding settles at P (R_w + R_h) = 100 K too and the housing at P R_h = 75 K. On the way, with
    /// k = 1800 - 60 / 0.25 = 1560 s, the matrix [[-1/60, 1/60], [1/(0.25 k), -(1/0.25 + 1/0.75)/k]] has the eigenvalues
    /// -7.36204e-4 and -1.934927e-2 1/s (1358 s and 51.7 s), and from cold the winding is at
    /// 100 - 81.5696 exp(-t / 1358.3 s) - 18.4304 exp(-t / 51.68 s): 16.18 K after 60 s (the one node: 3.28 K) and
    /// 78.32 K after 1800 s (the one node: 63.21 K).
    #[test]
    fn three_thermal_nodes_settle_at_the_sum_of_the_resistances_and_step_as_by_hand() {
        let p = DriveParams {
            heating: true,
            thermal_time_constant: 1800.0,
            current_cycle: 1e-3,
            thermal_nodes: Some(ThermalNodes { winding_share: 0.25, winding_time_constant: 60.0 }),
            ..params()
        };
        let rated_loss = p.resistance_at(p.rated_rise) * p.rated_current * p.rated_current;
        let mut d = Drive { copper_loss: rated_loss, ..Drive::default() };
        let by_hand = |t: f64| 100.0 - 81.5696 * (-7.362_044e-4 * t).exp() - 18.4304 * (-1.934_927e-2 * t).exp();
        // 20 000 s: 81.6 K exp(-20000 / 1358) = 3e-5 K are left of the slow part.
        for step in 1..=20_000_000 {
            d.warm(&p);
            let t = step as f64 * 1e-3;
            if step == 60_000 || step == 1_800_000 {
                assert!((d.temperature_rise - by_hand(t)).abs() < 1e-3, "{} K at {t} s", d.temperature_rise);
            }
        }
        assert!((by_hand(60.0) - 16.18).abs() < 0.01 && (by_hand(1800.0) - 78.32).abs() < 0.01);
        assert!(
            (d.temperature_rise - 100.0).abs() < 1e-3 && (d.housing_rise - 75.0).abs() < 1e-3,
            "{} K, {} K",
            d.temperature_rise,
            d.housing_rise
        );
        // The settled rise the verdicts judge is the winding's, and the same as the one node's.
        assert!((p.settled_rise(p.rated_current) - 100.0).abs() < 1e-9);
        // A winding started warm starts with its housing settled for it.
        let warm = DriveParams { start_rise: 40.0, ..p.clone() };
        assert_eq!(Drive::at_rest(&warm, 0.0, 0.0, 0.0).housing_rise, 30.0);
    }
}
