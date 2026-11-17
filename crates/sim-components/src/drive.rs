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

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
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
    /// Back-EMF per speed; 0 when switched off. The torque constant unless `drive.back_emf_constant_v_s_rad` gives
    /// another (the maker's printed voltage constant, in the one winding's scale: sqrt(3) times the line-to-line one).
    pub ke: f64,
    /// Highest voltage at the winding, `WINDING_PER_DC_LINK` times the DC link voltage; infinite when the
    /// limit is switched off.
    pub max_voltage: f64,
    /// `drive.overmodulation`: the fundamental of the six-step (square) wave over the sinusoidal limit, else 1: 2 sqrt 3 / pi
    /// = 1.1027 for the three-phase bridge of a servo or induction motor, 4 / pi for a stepper's H-bridges. The model then
    /// carries the fundamental only, without the square wave's harmonics (and their loss). `dc_gain` is it times
    /// `WINDING_PER_DC_LINK`.
    pub fundamental_gain: f64,
    /// Current controller gains, and its cycle (the base step).
    pub current_kp: f64,
    pub current_ki: f64,
    pub current_cycle: f64,
    pub peak_torque: f64,
    /// What the motor can give continuously: the limit for the RMS torque of a cycle.
    pub rated_torque: f64,
    pub rated_speed: f64,
    pub no_load_speed: f64,
    /// The highest speed the motor may turn at, `drive.max_speed_rad_s` (the catalogue's mechanical or inverter limit);
    /// the no-load speed where the config leaves it out. The time-optimal plan keeps within it (`path`), "motor-speed"
    /// judges a cycle against it where the config gives it, and `check_limits` warns of axis limits beyond it.
    pub max_speed: f64,
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
    /// Pole pairs `motor.pole_pairs`, where the config has them (0 where it has none): the electrical speed w L i of the
    /// servo winding's reactance takes the q axis's voltage (`current_control`, `voltage_limited_current`).
    pub pole_pairs: f64,
    /// Heating of the winding: whether it is on, the temperature rise at the rated current, the rated
    /// current, the thermal time constant and the rise the simulation starts with.
    pub heating: bool,
    pub rated_rise: f64,
    /// The housing's highest rise, K (`ratings.housing_max_rise_k`; absent: `rated_rise`), what the utilisation chart holds
    /// its rise against.
    pub housing_max_rise: f64,
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
    /// effects.iron_loss: hysteresis and eddy-current loss in the stator iron at the rated speed, W (`iron_loss_at`), and
    /// the stall current `motor.stall_current_a`, at which the copper loss alone brings the winding to its rated rise.
    /// The iron loss is what the copper loss at the rated speed leaves of that, R (I_0^2 - I_N^2): Siemens' rated and
    /// static data both hold for the rated rise. 0 and the rated current with the effect off. This is the speed losses'
    /// heat as the winding feels it, in copper-loss watts (`iron_heating`), not the power they take from the shaft
    /// (`iron_drag_rated`): on the 144 1FK7/1FT7 rows with a printed resistance the printed torque gap (k_t I_N - M_N)
    /// times the rated speed is a median 2.9 times it, and the printed efficiency's losses beyond copper side with the gap
    /// (median 0.75 of it, 2.3 times this): iron and bearings heat the winding less than its own copper does.
    pub iron_loss_rated: f64,
    /// effects.iron_loss, a servo: the drag torque of the speed losses at the rated speed, N m (`iron_drag`): what the
    /// printed rated point leaves of the torque constant times the rated current, k_t I_N (with saturation) - M_N, less
    /// the shaft friction the model has of its own there (`motor.friction_*` with effects.motor_friction), at least 0;
    /// so the shaft gives the printed rated torque at the rated current and speed. 0 with the effect off.
    pub iron_drag_rated: f64,
    pub stall_current: f64,
    /// The static torque k_t I_0 of the S1 curve, from `motor.stall_current_a` whether or not effects.iron_loss is on; 0
    /// where the config has no stall current (`s1_torque`).
    pub static_torque: f64,
    /// The S1 curve the config prints (`motor.s1_curve_speed_rad_s`, `motor.s1_curve_torque_nm`) as (speed, torque) points,
    /// speeds rising; empty where it has none (`s1_torque` then takes the straight line).
    pub s1_curve: Vec<[f64; 2]>,
    /// effects.magnet_temperature: the magnets' flux per kelvin, `motor.magnet_temperature_coefficient_1_k` (negative); the
    /// torque constant and back-EMF are the catalogue's at the rated rise and change with it (`Drive::flux`). 0 off.
    pub magnet_coefficient: f64,
    /// effects.braking_resistor_heating: the braking resistor's temperature (`ResistorHeating`); None without.
    pub resistor_heating: Option<ResistorHeating>,
    /// Fine interpolation, the torque feedforward's model, friction feedforward and balancing (`Precontrol`).
    pub precontrol: Precontrol,
    /// motor.kind: the motor the drive turns (`Motor`); the servo motor this module models by default.
    pub motor: Motor,
}

/// motor.kind: "servo" (the default, the permanent-magnet servo motor of this module), "stepper" (`stepper::Stepper`
/// behind an open-loop step and direction drive) or "induction" (`induction::Induction` on a frequency converter).
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Motor {
    #[default]
    Servo,
    Stepper(crate::stepper::Stepper),
    Induction(crate::induction::Induction),
}

impl Motor {
    fn from_config(cfg: &Config) -> Result<Self> {
        match cfg.get("motor.kind").ok().map(|v| v.as_str()) {
            None | Some(Some("servo")) => Ok(Self::Servo),
            Some(Some("stepper")) => Ok(Self::Stepper(crate::stepper::Stepper::from_config(cfg)?)),
            Some(Some("induction")) => Ok(Self::Induction(crate::induction::Induction::from_config(cfg)?)),
            _ => Err(ConfigError::Invalid("motor.kind".into())),
        }
    }

    /// Whether the drive runs the motor without its speed loop: the stepper's step generator and the induction motor's
    /// V/f control follow the speed setpoint open loop, and take no torque setpoint.
    pub fn open_loop(&self) -> bool {
        match self {
            Self::Servo => false,
            Self::Stepper(_) => true,
            Self::Induction(m) => !m.field_oriented,
        }
    }
}

/// The electrical states of a winding in the integration of a base step (`Sim::step`), per drive: the first state, the
/// electrical energy, the copper loss and the work on the shaft, then three more. The servo's one winding has its current
/// first and the last three at rest; the other motors keep theirs in `Phases::x`.
pub const WINDING_STATES: usize = 7;

/// The state of a stepper's or an induction motor's windings and drive beside the servo's (`Motor`); unused for the servo.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Phases {
    /// The electrical states (stepper: i_a, i_b; induction: stator and rotor flux in d and q), as integrated.
    pub x: [f64; 4],
    /// The voltages held over the base step (stepper: phases a and b; induction: d and q).
    pub u: [f64; 2],
    /// The current controllers' integrators (induction, field-oriented).
    pub integral: [f64; 2],
    /// The rotor at the end of the last step: its angle and speed at the motor, rad and rad/s.
    pub angle: f64,
    pub speed: f64,
    /// Stepper: the angle the step generator commands, rad at the motor; the angle the windings count from (where the
    /// drive was enabled, on a full step); the electrical angle of the current vector, in whole microsteps; and the
    /// load angle, that less the rotor's electrical angle (beyond the pull-out angle the rotor falls out of step).
    pub command: f64,
    pub origin: f64,
    /// Stepper: how far the drive's own count stands from the controller's setpoint (rad at the motor), left by an
    /// enable that pulled the rotor to the nearest full step of the grid (`Stepper::re_enabled`); 0 otherwise.
    #[serde(default)]
    pub offset: f64,
    pub electrical: f64,
    pub load_angle: f64,
    /// Induction: the electrical speed of the frame the voltages are set in, rad/s, and the drive's estimate of the rotor
    /// flux (field-oriented) or its estimate of the torque (V/f, for the slip compensation).
    pub frame_speed: f64,
    pub estimate: f64,
}

/// What the drive does with the setpoints between and beside the gains, as a SINAMICS S120/S210 in DSC is set up.
/// The default is the drive before 14 November 2026: setpoint held, rigid model, no friction, no balancing.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
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
    /// effects.friction_feedforward with drive.friction_feedforward_model = "machine": in place of the characteristic,
    /// the friction of the machine's own model at the planned speed (`Machine::steady_friction`, `Drive::friction_ff`).
    #[serde(default)]
    pub machine_friction: bool,
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

/// effects.braking_resistor_heating: the braking resistor as one heat capacity, rise' = (rated_rise P / P_continuous -
/// rise) / tau: at its continuous power it settles at its rated rise, and a peak of its peak power for the duty cycle the
/// catalogue gives (`braking_s120.csv`) warms it within the time constant.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ResistorHeating {
    pub continuous_power: f64,
    pub time_constant: f64,
    pub rated_rise: f64,
}

impl ResistorHeating {
    /// The rise after `dt` at a constant `power`, exactly.
    pub fn rise_after(&self, rise: f64, power: f64, dt: f64) -> f64 {
        let target = self.rated_rise * power / self.continuous_power;
        target + (rise - target) * (-dt / self.time_constant).exp()
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
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct FieldWeakening {
    pub pole_pairs: f64,
    /// d-axis inductance as the winding has it (three times the phase value); the q axis's is `DriveParams::inductance`.
    pub d_inductance: f64,
}

/// effects.drive_overload: the drive's thermal image (i^2 t) of its power unit in two nodes, as the SINAMICS power units
/// model a slow heat sink and the fast chips on it: x = x_s + x_c with x_s' = ((1 - w) q - x_s) / tau and
/// x_c' = (w q - x_c) / tau_c, q = (i / I_rated)^2, w the chips' share of the rise (`chip_share`). x is 1 at the rated
/// current held. At the trip level the drive lowers its current limit to the base load current, and holds it there until
/// the image is back at 1. The trip level follows from the time the drive gives its maximum current from the rated load:
/// trip = r^2 - (r^2 - 1) ((1 - w) exp(-t_max / tau) + w exp(-t_max / tau_c)) for r = I_max / I_rated. With w = 0 the
/// one node of 14 November 2026.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Overload {
    pub rated_current: f64,
    pub base_current: f64,
    pub max_current: f64,
    pub time_constant: f64,
    pub trip: f64,
    pub chip_share: f64,
    pub chip_time_constant: f64,
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
        let chip_share = cfg.non_negative("ratings.drive_chip_share")?;
        if chip_share >= 1.0 {
            return Err(ConfigError::Invalid("ratings.drive_chip_share".into()));
        }
        let chip_time_constant = rating("drive_chip_time_constant_s")?;
        let mut o =
            Self { rated_current, base_current, max_current, time_constant, trip: 0.0, chip_share, chip_time_constant };
        let r2 = (max_current / rated_current).powi(2);
        let t_max = rating("drive_max_current_time_s")?;
        o.trip = r2 - (r2 - 1.0) * o.decay(t_max).iter().sum::<f64>();
        Ok(Some(o))
    }

    /// The share of a step's difference each node keeps after `dt`: the slow node's weighted by 1 - w, the chips' by w.
    fn decay(&self, dt: f64) -> [f64; 2] {
        [
            (1.0 - self.chip_share) * (-dt / self.time_constant).exp(),
            self.chip_share * (-dt / self.chip_time_constant).exp(),
        ]
    }

    /// The nodes [slow, chips] after `dt` at a constant `current`, exactly.
    pub fn nodes_after(&self, [slow, chip]: [f64; 2], current: f64, dt: f64) -> [f64; 2] {
        let q = (current / self.rated_current).powi(2);
        let (a, b) = ((1.0 - self.chip_share) * q, self.chip_share * q);
        [a + (slow - a) * (-dt / self.time_constant).exp(), b + (chip - b) * (-dt / self.chip_time_constant).exp()]
    }

    /// The time constant of node 0 (the slow one) or 1 (the chips).
    pub fn node_time_constant(&self, node: usize) -> f64 {
        if node == 0 { self.time_constant } else { self.chip_time_constant }
    }

    /// The time from the rated load held (image 1) until the image reaches the trip level at a constant `current`; infinite
    /// where it never does. Bisection on the closed form x(t) = q - (q - 1) (decays summed).
    pub fn trip_time(&self, current: f64) -> f64 {
        let q = (current / self.rated_current).powi(2);
        let at = |t: f64| q - (q - 1.0) * self.decay(t).iter().sum::<f64>();
        if at(1e9) < self.trip {
            return f64::INFINITY;
        }
        let (mut lo, mut hi) = (0.0, 1e9);
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if at(mid) < self.trip { lo = mid } else { hi = mid }
        }
        hi
    }
}

/// effects.motor_thermal_nodes: three nodes, winding, stator/housing and ambient, as two heat capacities in a chain.
/// The resistances add up to the one-node model's, rated rise over rated loss, so the settled rises stay; the winding
/// has the share `winding_share` of it (to the housing) and the time constant `winding_time_constant` against the
/// housing. The heat capacities add up to the one-node model's too (thermal time constant over the resistance).
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ThermalNodes {
    pub winding_share: f64,
    pub winding_time_constant: f64,
}

/// Resistance of copper rises by this share per kelvin.
pub(crate) const COPPER: f64 = 0.0039;

/// Winding voltage per DC link voltage, sqrt(3/2): see the module comment.
pub const WINDING_PER_DC_LINK: f64 = 1.224_744_871_391_589;

/// Phase amplitude per winding voltage: sqrt 2 / 3 (the winding's voltage is three times the RMS phase voltage).
const PHASE_PER_WINDING: f64 = 0.471_404_520_791_031_7;

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
        // The back-EMF per speed of a servo winding, where the maker prints a voltage constant apart from the torque
        // constant; none, or 0, leaves it equal to kt. A stepper's winding keeps one constant for both (`stepper`).
        let emf_constant = if cfg.get("drive.back_emf_constant_v_s_rad").is_ok() {
            cfg.non_negative("drive.back_emf_constant_v_s_rad")?
        } else {
            0.0
        };
        // The torque fades out between the two speeds; with the no-load speed at or below the rated one it
        // would have no slope to fade on.
        let (rated_speed, no_load_speed) =
            (cfg.positive("drive.rated_speed_rad_s")?, cfg.positive("drive.no_load_speed_rad_s")?);
        if no_load_speed <= rated_speed {
            return Err(ConfigError::Invalid("drive.no_load_speed_rad_s".into()));
        }
        let max_speed = if cfg.get("drive.max_speed_rad_s").is_ok() {
            cfg.positive("drive.max_speed_rad_s")?
        } else {
            no_load_speed
        };
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
        // The line module's power is needed only where it feeds back; its efficiency wherever the DC link is simulated,
        // which it fills from the line through it.
        let regenerative = dc_link && on("regenerative_supply")?;
        let line = |leaf: &str| if regenerative { cfg.positive(&format!("ratings.{leaf}")) } else { Ok(1.0) };
        let line_efficiency = if dc_link { cfg.positive("ratings.line_module_efficiency")? } else { 1.0 };
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
        // Which friction the feedforward adds: the drive's recorded characteristic (no key: as before 15 November 2026),
        // or the machine's own model at the planned speed.
        let machine_friction = match cfg.get("drive.friction_feedforward_model").map(|v| v.as_str()) {
            Err(_) | Ok(Some("characteristic")) => false,
            Ok(Some("machine")) => true,
            Ok(_) => return Err(ConfigError::Invalid("drive.friction_feedforward_model".into())),
        } && switched("friction_feedforward")?;
        let friction = if switched("friction_feedforward")? && !machine_friction {
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
            machine_friction,
            balance_time: if cfg.get("drive.precontrol_balance_s").is_ok() {
                cfg.non_negative("drive.precontrol_balance_s")?
            } else {
                0.0
            },
        };
        // The switches below read as off where a config has none, as those above.
        let stall_current = if switched("iron_loss")? { cfg.positive("motor.stall_current_a")? } else { 0.0 };
        let rated_current = cfg.positive("motor.rated_current_a")?;
        let rated_rise = cfg.positive("motor.rated_temperature_rise_k")?;
        let magnet_coefficient =
            if switched("magnet_temperature")? { cfg.f64("motor.magnet_temperature_coefficient_1_k")? } else { 0.0 };
        if !(-0.01..=0.0).contains(&magnet_coefficient) {
            return Err(ConfigError::Invalid("motor.magnet_temperature_coefficient_1_k".into()));
        }
        let resistor_heating = if dc_link && switched("braking_resistor_heating")? {
            Some(ResistorHeating {
                continuous_power: cfg.positive("ratings.braking_resistor_continuous_w")?,
                time_constant: cfg.positive("ratings.braking_resistor_time_constant_s")?,
                rated_rise: cfg.positive("ratings.braking_resistor_rated_rise_k")?,
            })
        } else {
            None
        };
        let motor = Motor::from_config(cfg)?;
        let servo = matches!(motor, Motor::Servo);
        let saturation = if on("torque_saturation")? { 1.0 - kt_at_max / kt } else { 0.0 };
        let max_current = cfg.positive("motor.max_current_a")?;
        // The printed rated point's torque gap holds the iron's drag and the shaft's friction (`iron_drag_rated`).
        let iron_drag_rated = if servo && stall_current > 0.0 {
            let air_gap = kt * rated_current * (1.0 - saturation * (rated_current / max_current).powi(2).min(1.0));
            let friction = if cfg.bool("effects.motor_friction")? {
                cfg.non_negative("motor.friction_coulomb_nm")?
                    * (rated_speed / cfg.positive("motor.friction_smoothing_rad_s")?).tanh()
                    + cfg.non_negative("motor.friction_viscous_nm_s_rad")? * rated_speed
            } else {
                0.0
            };
            (air_gap - cfg.positive("drive.rated_torque_nm")? - friction).max(0.0)
        } else {
            0.0
        };
        // Absent: the sinusoidal limit, as before. A square wave's fundamental is 4 / pi of its height. A stepper's
        // H-bridge gives a phase +-U, so sinusoidally U at most: 4 / pi more. A three-phase bridge's sinusoidal limit
        // (space vector, `WINDING_PER_DC_LINK`) is U / sqrt 3 per phase and six-step's fundamental 2 U / pi: 2 sqrt 3 / pi.
        let fundamental_gain = match cfg.get("drive.overmodulation").is_ok() && cfg.bool("drive.overmodulation")? {
            false => 1.0,
            true if matches!(motor, Motor::Stepper(_)) => 4.0 / std::f64::consts::PI,
            true => 2.0 * 3f64.sqrt() / std::f64::consts::PI,
        };
        Ok(Self {
            // Copper at the rated rise; a rated current above the stall current (a misprint, see catalog) gives no iron loss.
            iron_loss_rated: resistance
                * (1.0 + COPPER * rated_rise)
                * (stall_current * stall_current - rated_current * rated_current).max(0.0),
            iron_drag_rated,
            // An induction motor's rotor heats it too: at the rated point the copper loss of both brings it to its rated
            // rise, as the stall current's alone would (Drive::thermal_current).
            stall_current: match &motor {
                Motor::Induction(m) => {
                    let warm = resistance * (1.0 + COPPER * rated_rise);
                    (m.rated_copper_loss(warm) / warm).sqrt()
                }
                _ if stall_current > 0.0 => stall_current,
                _ => rated_current,
            },
            magnet_coefficient,
            resistor_heating,
            precontrol,
            motor,
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
            ke: match on("back_emf")? {
                false => 0.0,
                true if servo && emf_constant > 0.0 => emf_constant,
                true => kt,
            },
            s1_curve: s1_curve_of(cfg)?,
            static_torque: if cfg.get("motor.stall_current_a").is_ok() {
                kt * cfg.positive("motor.stall_current_a")?
            } else {
                0.0
            },
            max_voltage: if on("voltage_limit")? {
                WINDING_PER_DC_LINK * fundamental_gain * bus_voltage
            } else {
                f64::INFINITY
            },
            fundamental_gain,
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
            max_speed,
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
            pole_pairs: match (&field_weakening, cfg.get("motor.pole_pairs")) {
                (Some(fw), _) => fw.pole_pairs,
                (None, Ok(_)) => cfg.positive("motor.pole_pairs")?,
                (None, Err(_)) => 0.0,
            },
            heating: on("winding_heating")?,
            rated_rise,
            housing_max_rise: if cfg.get("ratings.housing_max_rise_k").is_ok() {
                cfg.positive("ratings.housing_max_rise_k")?
            } else {
                rated_rise
            },
            rated_current,
            thermal_time_constant,
            start_rise: cfg.non_negative("motor.start_temperature_rise_k")?,
            saturation,
            max_current,
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
        self.torque_range_at(speed, self.max_voltage)
    }

    /// `torque_range` with the winding getting `max_voltage` at most (the DC link's at the moment, `Drive::max_voltage`).
    pub fn torque_range_at(&self, speed: f64, max_voltage: f64) -> (f64, f64) {
        let (low, high) =
            if self.limit { self.torque_curve_at(speed, max_voltage) } else { (f64::NEG_INFINITY, f64::INFINITY) };
        // The induction motor has no servo torque constant: its curve holds the drive's current itself
        // (`Induction::torque_curve`), as the field-weakened curve does (`field_weakened_torque`, the current circle).
        let curve_has_it = matches!(self.motor, Motor::Induction(_)) || (self.limit && self.field_weakening.is_some());
        if self.drive_max_current.is_finite() && !curve_has_it {
            let most = self.torque_of(self.drive_max_current);
            (low.max(-most), high.min(most))
        } else {
            (low, high)
        }
    }

    /// The winding's voltage limit per DC link voltage: `WINDING_PER_DC_LINK`, times `fundamental_gain`.
    pub fn dc_gain(&self) -> f64 {
        WINDING_PER_DC_LINK * self.fundamental_gain
    }

    /// effects.iron_loss: the iron loss at a speed, `iron_loss_rated` times (|speed| / rated speed)^1.5: hysteresis grows
    /// with the frequency, eddy currents with its square. One exponent stands for both parts: no maker prints the split.
    /// This is the heat the winding's thermal model takes (`Drive::iron_loss`, `iron_heating`); the power the shaft loses
    /// to the same losses follows the same law from the printed torque gap (`iron_drag`).
    pub fn iron_loss_at(&self, speed: f64) -> f64 {
        if self.iron_loss_rated == 0.0 {
            0.0
        } else {
            self.iron_loss_rated * (speed.abs() / self.rated_speed).powf(1.5)
        }
    }

    /// effects.iron_loss: the drag torque of the speed losses on a servo's shaft, against the motion: `iron_drag_rated` at
    /// the rated speed and P(w) / w with P = c |w|^1.5 over speed, written T_N w |w|^-0.5 / w_N^0.5 so that it is smooth
    /// through 0 (0 at standstill, no sign jump). The shaft gets the air-gap torque less this, so shaft power = air-gap
    /// power - this loss, with the drive disabled too.
    pub fn iron_drag(&self, speed: f64) -> f64 {
        if self.iron_drag_rated == 0.0 || speed == 0.0 {
            0.0
        } else {
            self.iron_drag_rated * speed / (speed.abs() * self.rated_speed).sqrt()
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
    ///
    /// With effects.iron_loss the shaft power is the air-gap power less the iron loss (`iron_drag` w): what the electrical
    /// power leaves beyond copper, stored energy and shaft power is `Drive::emf_loss`, which then holds the iron loss.
    ///
    /// `flux` scales the torque constant and the back-EMF for the magnets' temperature (`Drive::flux`, 1 without
    /// effects.magnet_temperature).
    pub fn winding_rates(
        &self,
        voltage: f64,
        resistance: f64,
        current: f64,
        speed: f64,
        d_current: f64,
        flux: f64,
    ) -> [f64; 4] {
        let (coupling, d_power) = match &self.field_weakening {
            Some(fw) => {
                let w = fw.pole_pairs * speed;
                (w * fw.d_inductance * d_current, (resistance * d_current - w * self.inductance * current) * d_current)
            }
            None => (0.0, 0.0),
        };
        let rate = (voltage - resistance * current - flux * self.back_emf(current, speed) - coupling) / self.inductance;
        [
            rate,
            voltage * current + d_power,
            resistance * (current * current + d_current * d_current),
            (self.torque_with(current, d_current, flux) - self.iron_drag(speed)) * speed,
        ]
    }

    /// Torque of a q current and a d current: `torque_of` times the magnets' `flux`, plus the reluctance torque pole
    /// pairs (L_d - L_q) i_d i_q with field weakening.
    pub fn torque_with(&self, current: f64, d_current: f64, flux: f64) -> f64 {
        let reluctance = self
            .field_weakening
            .map_or(0.0, |fw| fw.pole_pairs * (fw.d_inductance - self.inductance) * d_current * current);
        flux * self.torque_of(current) + reluctance
    }

    /// The d current the drive sets for a q current at a speed with the voltage `max_voltage`: none while
    /// u_q^2 + u_d^2 (steady, `FieldWeakening`) stays within it, else the least negative one that brings it there, at
    /// most `limit`. Where none does, the one that needs the least voltage. 0 without field weakening. With the winding's
    /// `resistance` at its temperature and the magnets' `flux` (`Drive::flux`), as the winding has them.
    ///
    /// The back-EMF is the unsaturated k_E w: the drive's own model of the motor, which the winding's saturated back-EMF
    /// (`back_emf`) undercuts, so the d current leaves a voltage reserve (at most the saturation's share of the
    /// back-EMF: 11 % at the maximum current of the test motor, 17 % for the 1FK2106-6AF). Taken saturated (tried
    /// 17 November 2026) the steady point sits on the voltage circle, where braking beyond the curve is unstable: the q
    /// voltage that the circle leaves falls as the braking current rises (u_d = R i_d - w L i_q grows), faster than R and
    /// the back-EMF's fall hold it; the test motor at 400 rad/s and -3.8 N m ran away to 18.8 A of its 9.5 A
    /// (`braking_at_the_voltage_limit_does_not_run_away`). The torque-speed curve takes the same model, so it is what the
    /// drive reaches: 3.6 % below the saturated optimum at 380 rad/s there (3.3255 against 3.4514 N m).
    pub fn d_current_for(&self, q: f64, speed: f64, max_voltage: f64, limit: f64, resistance: f64, flux: f64) -> f64 {
        let Some(fw) = &self.field_weakening else { return 0.0 };
        let w = fw.pole_pairs * speed;
        let (a, b, r) = (w * fw.d_inductance, w * self.inductance, resistance);
        let c0 = r * q + flux * self.ke * speed;
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
    /// `imax` is the radius of the current circle, the phase current's: the motor's maximum current, or the drive's where
    /// that is lower (`torque_curve_at`).
    fn field_weakened_torque(&self, fw: &FieldWeakening, speed: f64, umax: f64, motoring: bool, imax: f64) -> f64 {
        let r = self.resistance;
        let w = fw.pole_pairs * speed;
        let (a, b) = (w * fw.d_inductance, w * self.inductance);
        // The q current the two circles leave at a d current, as a magnitude.
        let q_at = |d: f64| -> f64 {
            let by_current = (imax * imax - d * d).max(0.0).sqrt();
            if !umax.is_finite() {
                return by_current;
            }
            // (r q + c1)^2 + (r d - b q)^2 <= U^2 as qa q^2 + qb q + qc <= 0. The back-EMF unsaturated, as the drive's
            // d current takes it (`d_current_for`): the curve is what the drive reaches.
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
            let t = self.torque_with(q, d, 1.0);
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

    /// Without field weakening: the largest current magnitude the voltage `umax` leaves at a speed of at least 0, within the
    /// motor's maximum current, steady and cold. The q axis meets e = kE speed (motoring: R i + e; braking: e - R i) and
    /// the d axis the reactance x i, x = p speed L (`current_control`): (R i + e)^2 + (x i)^2 <= U^2, so with
    /// a = R^2 + x^2 and b = +-R e the root of a i^2 + 2 b i + e^2 - U^2 = 0, i = (-b + sqrt(b^2 - a (e^2 - U^2))) / a.
    /// Without reactance that is (U - e) / R motoring and (U + e) / R braking.
    ///
    /// Braking above the speed where e alone exceeds U the same circle is solved, for the braking current i against e:
    /// (e - R i)^2 + (x i)^2 <= U^2, an interval of currents [`forced_braking_current`, the root returned here]; the
    /// smaller one is what the inverter's diodes force at a speed the voltage cannot hold (the uncontrolled generator).
    /// Where the circle leaves none (x e > sqrt(a) U) the nearest current is returned, the circle's closest approach R e / a.
    pub fn voltage_limited_current(&self, speed: f64, umax: f64, motoring: bool) -> f64 {
        let (r, e) = (self.resistance, self.ke * speed);
        let x = self.pole_pairs * speed * self.inductance;
        let i = if x == 0.0 {
            (if motoring { umax - e } else { umax + e }) / r
        } else {
            let (a, b) = (r * r + x * x, if motoring { r * e } else { -r * e });
            (-b + (b * b - a * (e * e - umax * umax)).max(0.0).sqrt()) / a
        };
        i.clamp(0.0, self.max_current)
    }

    /// Braking above the speed where the back-EMF alone exceeds `umax`: the smallest braking current the voltage circle
    /// (e - R i)^2 + (x i)^2 <= U^2 leaves, the one the rectifier diodes force, i = (R e - sqrt(a U^2 - x^2 e^2)) / a with
    /// a = R^2 + x^2; the nearest current R e / a where the circle leaves none. 0 while e <= U (nothing is forced).
    pub fn forced_braking_current(&self, speed: f64, umax: f64) -> f64 {
        let (r, e) = (self.resistance, self.ke * speed);
        if !(e > umax) {
            return 0.0;
        }
        let x = self.pole_pairs * speed * self.inductance;
        let a = r * r + x * x;
        ((r * e - (a * umax * umax - x * x * e * e).max(0.0).sqrt()) / a).clamp(0.0, self.max_current)
    }

    /// The temperature rise the winding settles at when a cycle with this RMS current and this mean iron loss
    /// (effects.iron_loss, W) repeats for long: the copper loss, with the resistance at that temperature, against the loss
    /// at the rated current (with the iron loss, at the stall current), plus the iron loss's share. That share is all of
    /// it with one thermal node, and with effects.motor_thermal_nodes the housing's, 1 - s: the iron is in the stator.
    pub fn settled_rise(&self, rms_current: f64, iron_loss: f64) -> f64 {
        let q = self.rated_rise / (1.0 + COPPER * self.rated_rise) * (rms_current / self.stall_current).powi(2);
        let iron = self.iron_heating(iron_loss) * self.thermal_nodes.map_or(1.0, |n| 1.0 - n.winding_share);
        if COPPER * q < 1.0 { (q + iron) / (1.0 - COPPER * q) } else { f64::INFINITY }
    }

    /// The copper loss at which the winding settles at its rated rise: at the stall current with effects.iron_loss (the
    /// iron loss is apart), else at the rated current, which then stands for both.
    pub fn rated_loss(&self) -> f64 {
        self.resistance_at(self.rated_rise) * self.stall_current * self.stall_current
    }

    /// The S1 torque at a speed, as the motor-s1 verdict has it (`analysis::s1_torque`) and the torque-speed chart draws it:
    /// `s1_line` from the static torque k_t I_0 (`motor.stall_current_a`, read with effects.iron_loss off too); flat at the
    /// rated torque where the config has no stall current.
    pub fn s1_torque(&self, speed: f64) -> f64 {
        if !self.s1_curve.is_empty() {
            return s1_interpolated(&self.s1_curve, speed);
        }
        let still = if self.static_torque > 0.0 { self.static_torque } else { self.rated_torque };
        s1_line(still, self.rated_torque, self.rated_speed, speed)
    }

    /// A loss in the rise it would settle at alone, K: rated rise per `rated_loss`.
    pub fn iron_heating(&self, loss: f64) -> f64 {
        self.rated_rise * loss / self.rated_loss()
    }

    /// The same limits, whether or not the limit is switched on: what the motor and drive can give.
    /// With field weakening the curve is the d-q model's (`FieldWeakening`), up to the peak torque; without it the
    /// torque fades out linearly between the rated and the no-load speed.
    pub fn torque_curve(&self, speed: f64) -> (f64, f64) {
        self.torque_curve_at(speed, self.max_voltage)
    }

    /// `torque_curve` with the winding getting `max_voltage` at most. Without field weakening the fade's two speeds
    /// scale with it against `max_voltage` (the back-EMF the voltage leaves room for).
    pub fn torque_curve_at(&self, speed: f64, max_voltage: f64) -> (f64, f64) {
        // A stepper's pull-out curve, an induction motor's breakdown curve.
        match &self.motor {
            Motor::Stepper(s) => return s.torque_curve(self, speed, max_voltage / WINDING_PER_DC_LINK),
            Motor::Induction(m) => return m.torque_curve(self, speed, max_voltage / WINDING_PER_DC_LINK),
            Motor::Servo => {}
        }
        if let Some(fw) = &self.field_weakening {
            let imax = self.max_current.min(self.drive_max_current);
            let (motoring, braking) = (
                self.field_weakened_torque(fw, speed.abs(), max_voltage, true, imax).min(self.peak_torque),
                self.field_weakened_torque(fw, speed.abs(), max_voltage, false, imax).min(self.peak_torque),
            );
            return if speed >= 0.0 { (-braking, motoring) } else { (-motoring, braking) };
        }
        // The printed fade at the config's voltage; at another one each of its two speeds moves to where the voltage
        // circle leaves the current of that point (`fade_speed`): the ratio of those speeds at the new and the printed
        // voltage, so R, L and kE enter. The printed voltage is the sinusoidal limit (without drive.overmodulation).
        let printed = self.max_voltage / self.fundamental_gain;
        let (rated_speed, no_load_speed) = if max_voltage.is_finite() && printed.is_finite() && max_voltage != printed {
            (self.fade_speed(self.rated_speed, max_voltage), self.fade_speed(self.no_load_speed, max_voltage))
        } else {
            (self.rated_speed, self.no_load_speed)
        };
        let fade = ((no_load_speed - speed.abs()) / (no_load_speed - rated_speed).max(1e-12)).clamp(0.0, 1.0);
        // Braking holds the peak torque at every speed, as far as the voltage circle (the winding's reactance) lets a
        // current of that size be controlled: beyond it the q voltage cannot oppose the back-EMF and the current runs away.
        let braking = if self.pole_pairs > 0.0 && max_voltage.is_finite() {
            self.peak_torque.min(self.torque_of(self.voltage_limited_current(speed.abs(), max_voltage, false)))
        } else {
            self.peak_torque
        };
        let motoring = self.peak_torque * fade;
        if speed >= 0.0 { (-braking, motoring) } else { (-motoring, braking) }
    }

    /// A speed of the printed fade (at the sinusoidal limit `max_voltage / fundamental_gain`) moved to the voltage `umax`:
    /// the speed at which the voltage circle (`voltage_limited_current`, motoring) leaves the current it leaves at `speed`
    /// with the printed voltage, times `speed` over the printed voltage's own such speed. That speed is the circle solved
    /// for w at the current i: (R i + kE w)^2 + (p w L i)^2 = U^2, A w^2 + B w + C = 0 with A = kE^2 + (p L i)^2,
    /// B = 2 R i kE, C = (R i)^2 - U^2 (the highest speed that still leaves i; where the current is clamped at the maximum,
    /// the end of that plateau). Without a back-EMF or where that current is 0 (the point sits at e = U) the speed
    /// scales with the voltage.
    fn fade_speed(&self, speed: f64, umax: f64) -> f64 {
        let printed = self.max_voltage / self.fundamental_gain;
        let proportional = speed * umax / printed;
        let i = self.voltage_limited_current(speed, printed, true);
        if self.ke <= 0.0 || i <= 1e-9 * self.max_current {
            return proportional;
        }
        let (r, x) = (self.resistance, self.pole_pairs * self.inductance * i);
        let at = |u: f64| {
            let (a, b, c) = (self.ke * self.ke + x * x, 2.0 * r * i * self.ke, r * r * i * i - u * u);
            if c >= 0.0 { 0.0 } else { (-b + (b * b - 4.0 * a * c).sqrt()) / (2.0 * a) }
        };
        let at_printed = at(printed);
        if at_printed > 0.0 { speed * at(umax) / at_printed } else { proportional }
    }

    /// The torque (lowest, highest) the winding's voltage leaves at a speed with a DC link of `dc_voltage`, within the
    /// motor's maximum current, steady and cold, whether or not the limit is switched on: the voltage-limit curve of the
    /// torque-speed chart. With field weakening the d-q model's (`torque_curve`'s, before the peak torque caps it);
    /// without it the winding has no d current, but its reactance is there: the steady voltage vector is
    /// (R i + kE speed, p speed L i) and the limit (R i + kE speed)^2 + (p speed L i)^2 <= U^2 in the direction of motion
    /// (`voltage_limited_current`; the pole pairs `motor.pole_pairs`, none without them: R i + kE speed <= U).
    pub fn voltage_limited_torque(&self, speed: f64, dc_voltage: f64) -> (f64, f64) {
        match &self.motor {
            Motor::Stepper(s) => return s.torque_curve(self, speed, self.fundamental_gain * dc_voltage),
            Motor::Induction(m) => return m.torque_curve(self, speed, self.fundamental_gain * dc_voltage),
            Motor::Servo => {}
        }
        let umax = self.dc_gain() * dc_voltage;
        let (motoring, braking) = match &self.field_weakening {
            Some(fw) => (
                self.field_weakened_torque(fw, speed.abs(), umax, true, self.max_current),
                self.field_weakened_torque(fw, speed.abs(), umax, false, self.max_current),
            ),
            None => (
                self.torque_of(self.voltage_limited_current(speed.abs(), umax, true)),
                self.torque_of(self.voltage_limited_current(speed.abs(), umax, false)),
            ),
        };
        if speed >= 0.0 { (-braking, motoring) } else { (-motoring, braking) }
    }

    /// The rates of drive `d`'s electrical states `w` (`WINDING_STATES`, as `Drive::winding_states` gives them) at a
    /// moment of a base step, with the rotor at `angle` and `speed` at the motor, into `rates`; returns the motor torque.
    /// The servo's are `winding_rates` and `torque_with`.
    pub fn motor_rates(&self, d: &Drive, w: &[f64], angle: f64, speed: f64, rates: &mut [f64]) -> f64 {
        match &self.motor {
            Motor::Servo => {
                let (i, flux) = (w[0], d.flux(self));
                rates[..4].copy_from_slice(&self.winding_rates(
                    d.voltage,
                    d.winding_resistance(self),
                    i,
                    speed,
                    d.d_current,
                    flux,
                ));
                rates[4..].fill(0.0);
                self.torque_with(i, d.d_current, flux) - self.iron_drag(speed)
            }
            Motor::Stepper(s) => s.rates(self, d, w, angle, speed, rates),
            Motor::Induction(m) => m.rates(self, d, w, speed, rates),
        }
    }
}

/// The printed S1 curve of a config (`motor.s1_curve_speed_rad_s`, `motor.s1_curve_torque_nm`): the same number of points
/// each, at least two, finite, speeds rising from 0 or above, torques not negative. Empty where the config has neither
/// key or both empty: the straight line.
pub fn s1_curve_of(cfg: &Config) -> Result<Vec<[f64; 2]>> {
    let (sk, tk) = ("motor.s1_curve_speed_rad_s", "motor.s1_curve_torque_nm");
    if cfg.get(sk).is_err() && cfg.get(tk).is_err() {
        return Ok(Vec::new());
    }
    let (speeds, torques) = (cfg.f64s(sk)?, cfg.f64s(tk)?);
    if speeds.is_empty() && torques.is_empty() {
        return Ok(Vec::new());
    }
    if speeds.len() != torques.len()
        || speeds.len() < 2
        || speeds.iter().chain(&torques).any(|x| !x.is_finite())
        || speeds[0] < 0.0
        || speeds.windows(2).any(|w| w[1] <= w[0])
        || torques.iter().any(|t| *t < 0.0)
    {
        return Err(ConfigError::Invalid(sk.into()));
    }
    Ok(speeds.into_iter().zip(torques).map(|(w, t)| [w, t]).collect())
}

/// The S1 torque at `speed` (either direction) from a printed curve: linear between its points, the first torque below
/// the first speed and the last one beyond the last (the curve ends where the maker's graph does).
pub fn s1_interpolated(curve: &[[f64; 2]], speed: f64) -> f64 {
    let w = speed.abs();
    match curve.iter().position(|p| p[0] >= w) {
        Some(0) => curve[0][1],
        Some(k) => {
            let (a, b) = (curve[k - 1], curve[k]);
            a[1] + (b[1] - a[1]) * (w - a[0]) / (b[0] - a[0])
        }
        None => curve[curve.len() - 1][1],
    }
}

/// The S1 line: from the static torque at standstill straight to the rated torque at the rated speed, and on along that
/// line above it, never below 0.
pub fn s1_line(static_torque: f64, rated_torque: f64, rated_speed: f64, speed: f64) -> f64 {
    (static_torque + (rated_torque - static_torque) * speed.abs() / rated_speed).max(0.0)
}

/// Biquad notch (b0, b1, b2, a1, a2), normalised by a0.
fn notch_coefficients(frequency: f64, bandwidth: f64, cycle: f64) -> [f64; 5] {
    let w0 = TAU * frequency * cycle;
    let alpha = w0.sin() / (2.0 * frequency / bandwidth);
    let a0 = 1.0 + alpha;
    [1.0 / a0, -2.0 * w0.cos() / a0, 1.0 / a0, -2.0 * w0.cos() / a0, (1.0 - alpha) / a0]
}

/// How a drive takes its command, as a servo drive's operating modes. `Position`: the controller's plan through the
/// position and velocity loops (the default). `Velocity`: a speed setpoint from outside, rad/s at the motor, for the
/// velocity loop; the position loop is bypassed. `Torque`: a torque setpoint from outside, N m at the motor, in place of
/// the velocity loop's output; both loops are bypassed. Either way the setpoint still passes the torque filter, notch,
/// torque limit and delay, the current loop, the winding and the DC link. Outside position mode the position setpoint
/// tracks the measured position (no following error), as a drive's tracking mode does.
#[derive(Debug, Clone, Copy, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum DriveMode {
    #[default]
    Position,
    Velocity(f64),
    Torque(f64),
}

/// The loops and filters of one drive.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct Drive {
    pub pos_set: f64,
    /// Speed setpoint at the last profile cycle, and its rate until the next one.
    pub vel_set: f64,
    pub acc_set: f64,
    /// Time since the profile generator last wrote the setpoints; the controller adds a base step after each.
    pub since_setpoint: f64,
    pub torque_ff: f64,
    /// With `drive.friction_feedforward_model = "machine"` (`Precontrol::machine_friction`): the friction the machine's
    /// own model has at the planned speed (`Machine::steady_friction`), written with the setpoints; 0 otherwise.
    #[serde(default)]
    pub friction_ff: f64,
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
    /// With effects.field_weakening: the d current, held over the base step like the voltage; 0 without. It follows the
    /// setpoint the drive works out (`DriveParams::d_current_for`) as the tuned current loop does, with the time constant
    /// L_d / Kp (the controller's Tn = L / R cancels the winding's own), exactly per step. The RMS phase current is the
    /// hypotenuse of it and `current` (`phase_current`).
    pub d_current: f64,
    /// The servo winding's d-axis voltage the current loop set with `voltage` (`current_control`), in the winding's units;
    /// 0 for the other motors and while disabled. `phase_voltage` is the vector of both.
    #[serde(default)]
    pub d_voltage: f64,
    /// With effects.drive_overload: the drive's thermal image (`Overload`), both nodes together, the chips' node alone,
    /// and whether it has lowered its current limit to the base load current.
    pub overload_image: f64,
    pub overload_chip: f64,
    pub overloaded: bool,
    /// With effects.iron_loss: the stator's iron loss over the last step, W, at the speed the step started with.
    pub iron_loss: f64,
    /// With a back-EMF constant other than the torque constant (`DriveParams::ke`): the electrical power the winding takes
    /// beyond the copper loss, the winding's stored energy and the shaft power over the last step, (kE - kt) i w, W. It
    /// is booked as a loss of its own in the energy budget (the magnets' and iron's, which the maker's two constants
    /// differ by); signed, as i w is. 0 for equal constants, a stepper and an induction motor.
    #[serde(default)]
    pub emf_loss: f64,
    /// The torque the machine's rigid model needs to hold the planned pose, where the feedforward is the constant inertia
    /// (`drive.torque_feedforward_model = "inertia"`: no gravity in `torque_ff`) and the torque-band monitor is on, plus
    /// with that monitor, on a machine that does not report its friction torque, the friction at the planned speed
    /// (`fault::Settings::torque_band`); 0 otherwise. What the monitor
    /// expects beside `torque_ff` (`fault::Watch::check`).
    #[serde(default)]
    pub torque_hold: f64,
    /// With effects.braking_resistor_heating: the braking resistor's temperature above its surroundings, K.
    pub resistor_rise: f64,
    /// The energy L i^2 / 2 a drive enabled without the DC link simulated put into its winding (`Control::hold_here`):
    /// drawn from the supply in the first step, booked there with the step's electrical power.
    pub enable_energy: f64,
    /// DC link voltage, the power the braking resistor burns, and whether the drive has tripped on
    /// overvoltage. With the DC link switched off the voltage stays at the bus level.
    pub dc_voltage: f64,
    pub resistor_power: f64,
    pub tripped: bool,
    /// With the DC link simulated: the power drawn from the line for the DC link, and with the regenerative supply the
    /// power fed back into the mains, over the last step; and what the line module lost either way
    /// (`ratings.line_module_efficiency`: of what it draws, the efficiency arrives in the DC link).
    pub rectifier_power: f64,
    pub mains_power: f64,
    pub line_loss: f64,
    /// With the DC link simulated and the drive off: what its discharge through the bleeder took out of the DC link over
    /// the last step, as a mean power (the rectifier draws it back while the link is held at the bus level).
    pub discharge_power: f64,
    /// The drive's own stream of sensor noise.
    pub noise: Noise,
    /// Whether the velocity loop's output was beyond the torque limit at its last cycle (`fault::Watch`).
    pub at_limit: bool,
    /// While the drive stops on a fault (`fault::Reaction`): the speed setpoint of its stop ramp, which the velocity
    /// loop follows without the position loop and without feedforward. And once its pulses are off: no voltage, no
    /// current, no torque, while the others may still be stopping.
    pub fault_speed: Option<f64>,
    pub pulses_off: bool,
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
    mode: DriveMode,
    /// Set when the mode changes between position, velocity and torque: the next velocity loop that runs its PI sets
    /// the integrator so that its output is the torque last asked for (bumpless transfer, `set_mode`).
    retune: bool,
    /// A stepper's or an induction motor's windings and drive (`Phases`); unused for the servo.
    pub phases: Phases,
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
        let mut drive = Self {
            pos_set: pos,
            position_setpoint: pos,
            torque_ff,
            feedforward_torque: p.torque_ff * torque_ff,
            controller_torque: torque - p.torque_ff * torque_ff,
            torque_demand: torque,
            torque_command: torque,
            torque,
            current: p.current_for(torque),
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
            voltage_queue: std::iter::repeat_n(0.0, p.current_delay).collect(),
            torque_queue: std::iter::repeat_n(torque, p.velocity_delay).collect(),
            ..Self::default()
        };
        match &p.motor {
            Motor::Stepper(s) => s.at_rest(&mut drive, p, pos),
            Motor::Induction(m) => m.at_rest(&mut drive, p, pos, torque),
            Motor::Servo => {}
        }
        drive.hold_current(p);
        drive
    }

    /// The electrical states this drive's winding starts a base step with, in the order of `WINDING_STATES`.
    pub fn winding_states(&self, p: &DriveParams) -> [f64; WINDING_STATES] {
        match p.motor {
            Motor::Servo => [self.current, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            _ => {
                let x = self.phases.x;
                [x[0], 0.0, 0.0, 0.0, x[1], x[2], x[3]]
            }
        }
    }

    /// The magnetic energy in the winding: L i^2 / 2 for the servo and the stepper (both phases), the induction motor's
    /// own (`induction::Induction::stored_energy`).
    pub fn winding_energy(&self, p: &DriveParams) -> f64 {
        match &p.motor {
            Motor::Induction(m) => m.stored_energy(&self.phases.x),
            _ => 0.5 * p.inductance * self.current * self.current,
        }
    }

    /// The energy stored in the drive's field: `winding_energy`, and for an enabled stepper the detent torque's potential
    /// (`Stepper::detent_potential`), so that the budget closes inside a step.
    pub fn magnetic_energy(&self, p: &DriveParams) -> f64 {
        match &p.motor {
            Motor::Stepper(s) if self.current > 0.0 => {
                self.winding_energy(p) + s.detent_potential(self.phases.angle, self.phases.origin)
            }
            _ => self.winding_energy(p),
        }
    }

    /// The current loop holding its current at rest: the voltage R i, with the winding's resistance at its temperature,
    /// in its integrator, its output and its delay. Before 14 November 2026 the cold resistance, so a winding started
    /// or enabled warm began with a voltage step of (R(rise) - R) i (found by the closed-loop reference written in
    /// Python for one belt axis, see docs/validation.md).
    pub fn hold_current(&mut self, p: &DriveParams) {
        let voltage = self.winding_resistance(p) * self.current;
        (self.voltage, self.current_integral) = (voltage, voltage);
        self.voltage_queue.iter_mut().for_each(|v| *v = voltage);
    }

    pub fn encoder(&self) -> f64 {
        self.encoder
    }

    pub fn mode(&self) -> DriveMode {
        self.mode
    }

    /// Switches the operating mode. A change between position, velocity and torque mode does not make the torque jump
    /// where the drive's own loops take over: the next velocity loop sets the PI integrator so that it asks for the
    /// torque it asked for last (`retune`). Into torque mode the outside setpoint acts at once, and the integrator is
    /// held meanwhile; a jump there is the outside controller's.
    pub fn set_mode(&mut self, mode: DriveMode) {
        if std::mem::discriminant(&mode) != std::mem::discriminant(&self.mode) {
            self.retune = true;
        }
        self.mode = mode;
    }

    /// The next velocity loop starts its PI from the torque last asked for (see `set_mode`).
    pub fn retune(&mut self) {
        self.retune = true;
    }

    /// The RMS phase current: of the q current `current` and the d current together.
    pub fn phase_current(&self) -> f64 {
        self.current.hypot(self.d_current)
    }

    /// `motor.<id>.voltage`, one meaning for every motor: the amplitude of the phase voltage's space vector, V. The
    /// induction motor's and the stepper's vectors are phase amplitudes already (`voltage`); the servo's one winding has
    /// three times the RMS phase voltage in each axis, so its vector (q `voltage`, d `d_voltage`) is multiplied by
    /// sqrt 2 / 3 (the DC link's sqrt(3/2) U gives U / sqrt 3).
    pub fn phase_voltage(&self, p: &DriveParams) -> f64 {
        match p.motor {
            Motor::Servo => self.voltage.hypot(self.d_voltage) * PHASE_PER_WINDING,
            _ => self.voltage,
        }
    }

    /// The current that heats the winding as the step's copper loss does, against `DriveParams::stall_current`: the phase
    /// current; for an induction motor, whose rotor is heated too, sqrt(copper loss / R_s).
    pub fn thermal_current(&self, p: &DriveParams) -> f64 {
        match p.motor {
            Motor::Induction(_) => (self.copper_loss / self.winding_resistance(p)).sqrt(),
            _ => self.phase_current(),
        }
    }

    /// The speed the velocity loop measured at its last cycle, filtered.
    pub fn measured_speed(&self) -> f64 {
        self.speed
    }

    /// The thermal image's nodes [slow, chips] (effects.drive_overload).
    pub fn overload_nodes(&self) -> [f64; 2] {
        [self.overload_image - self.overload_chip, self.overload_chip]
    }

    /// The magnets' flux against the catalogue's (effects.magnet_temperature): 1 + coefficient (rise - rated rise), the
    /// catalogue's torque constant holding at the rated rise. The magnets sit in the rotor next to the stator: at the
    /// housing's rise with effects.motor_thermal_nodes, else at the winding's. 1 without the effect or without winding
    /// heating (the motor then stands at its rated rise throughout).
    /// shortcut: the rotor has no temperature of its own; give it a node if magnet heating by eddy currents matters.
    pub fn flux(&self, p: &DriveParams) -> f64 {
        if p.magnet_coefficient == 0.0 || !p.heating {
            return 1.0;
        }
        // With the nodes the housing settles at (1 - s) of the rise the winding has (`ThermalNodes`), so the rated state
        // has the housing at (1 - s) times the rated rise.
        let (rise, rated) = match &p.thermal_nodes {
            Some(n) => (self.housing_rise, (1.0 - n.winding_share) * p.rated_rise),
            None => (self.temperature_rise, p.rated_rise),
        };
        1.0 + p.magnet_coefficient * (rise - rated)
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
        if self.mode != DriveMode::Position {
            // Tracking: the setpoint follows the actual position, so the loop asks for nothing.
            (self.position_setpoint, self.following_error, self.position_part) = (measured, 0.0, 0.0);
            return;
        }
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

    /// The voltage limit of the winding: what the DC link has at this moment with effects.dc_link, else the nominal one.
    pub fn max_voltage(&self, p: &DriveParams) -> f64 {
        if p.dc_link && p.max_voltage.is_finite() { p.dc_gain() * self.dc_voltage } else { p.max_voltage }
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
        // Open loop (a stepper, V/f): the speed setpoint goes to the step generator or the frequency, no PI.
        if p.motor.open_loop() {
            self.speed_command = match (self.fault_speed, self.mode) {
                (Some(speed), _) | (None, DriveMode::Velocity(speed)) => speed,
                _ => speed_set,
            };
            return;
        }
        // A fault's stop ramp wins over every mode.
        let mode = if self.fault_speed.is_some() { DriveMode::Velocity(0.0) } else { self.mode };
        self.speed_command = match (self.fault_speed, mode) {
            (Some(speed), _) | (None, DriveMode::Velocity(speed)) => speed,
            _ => self.position_part + p.velocity_ff * speed_set,
        };

        let error = self.speed_command - self.speed;
        let outside_torque = match mode {
            DriveMode::Torque(torque) => Some(torque),
            _ => None,
        };
        let feedforward = match mode {
            DriveMode::Position => {
                p.torque_ff * self.torque_ff + p.precontrol.friction_at(speed_set) + self.friction_ff
            }
            _ => 0.0,
        };
        if self.retune && outside_torque.is_none() {
            self.integral = self.controller_torque + self.feedforward_torque - feedforward - p.kp * error;
            self.retune = false;
        }
        self.controller_torque = outside_torque.unwrap_or(p.kp * error + self.integral);
        self.feedforward_torque = feedforward;
        let wanted = self.controller_torque + self.feedforward_torque;
        self.filtered += p.torque_filter * (wanted - self.filtered);
        let mut out = self.filtered;
        if let Some([b0, b1, b2, a1, a2]) = p.notch {
            let [x1, x2, y1, y2] = self.notch;
            out = b0 * self.filtered + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            self.notch = [self.filtered, x1, out, y1];
        }
        // The torque-speed curve at the DC link's voltage of the moment: a sagging DC link limits the torque at speed.
        let (mut low, mut high) = p.torque_range_at(self.speed, self.max_voltage(p));
        // An overloaded drive gives its base load current at most.
        // (The induction motor's converter holds it in its current setpoints, `Induction::control`.)
        if let (true, Some(o), false) = (self.overloaded, &p.overload, matches!(p.motor, Motor::Induction(_))) {
            let most = p.torque_of(o.base_current);
            (low, high) = (low.max(-most), high.min(most));
        }
        self.torque_demand = out;
        self.at_limit = out + self.torque_injection > high || out + self.torque_injection < low;
        self.torque_command =
            delayed(&mut self.torque_queue, (out + self.torque_injection).clamp(low, high), p.velocity_delay);
        // Anti-windup: the integrator stops while it would push further into the limit; in torque mode it is held.
        if outside_torque.is_none() && (out <= high || error < 0.0) && (out >= low || error > 0.0) {
            self.integral += p.ki * error * p.velocity_cycle;
        }
    }

    /// Current loop: PI on the winding current. The voltage it sets is held over the next base step, in which the
    /// integrator takes the winding through it (`DriveParams::winding_rates`); then `winding_stepped`.
    pub fn current_control(&mut self, p: &DriveParams, speed: f64) {
        match &p.motor {
            Motor::Stepper(s) => return s.control(self, p),
            Motor::Induction(m) => return m.control(self, p, speed),
            Motor::Servo => {}
        }
        let measured = self.current + if p.current_noise > 0.0 { p.current_noise * self.noise.next() } else { 0.0 };
        let mut setpoint = p.current_for(self.torque_command);
        let max_voltage = self.max_voltage(p);
        // The d axis first, the q axis gets what it leaves of the voltage circle. The d axis's steady voltage is
        // R i_d - w L i_q (w = p speed), the reactance of the q current with no d current (i_d = 0, the winding without
        // field weakening) and the reactance of the d current too with it.
        let (resistance, flux, w) = (self.winding_resistance(p), self.flux(p), p.pole_pairs * speed);
        let steady_d = resistance * self.d_current - w * p.inductance * self.current;
        let (coupling, u_d) = match &p.field_weakening {
            Some(fw) => {
                // The d current for this setpoint, with the winding warm as it is, for the larger of the setpoint and the
                // q current there is (an overshoot needs its weakening too).
                let limit = p.max_current.min(p.drive_max_current);
                let mut q_ref = if self.current.abs() > setpoint.abs() { self.current } else { setpoint };
                // A q current beyond what the two circles leave together (a setpoint above the curve) would take a d current
                // that fills the current circle and leaves no q at all: the d current is the one of the most q that fits.
                let fits = |q: f64| {
                    let (d, w) =
                        (p.d_current_for(q, speed, max_voltage, limit, resistance, flux), fw.pole_pairs * speed);
                    let (u_q, u_d) = (
                        resistance * q + flux * p.ke * speed + w * fw.d_inductance * d,
                        resistance * d - w * p.inductance * q,
                    );
                    q * q + d * d <= limit * limit && u_q * u_q + u_d * u_d <= max_voltage * max_voltage * (1.0 + 1e-9)
                };
                if !fits(q_ref) {
                    let (mut lo, mut hi) = (0.0, q_ref.abs());
                    for _ in 0..40 {
                        let mid = 0.5 * (lo + hi);
                        if fits(mid.copysign(q_ref)) { lo = mid } else { hi = mid }
                    }
                    q_ref = lo.copysign(q_ref);
                }
                let wanted_d = p.d_current_for(q_ref, speed, max_voltage, limit, resistance, flux);
                // The d axis follows it as the current loop does, with the voltage that takes i_d the lag's share this step
                // on top of the steady one: held within the circle, i_d moves as far as that voltage takes it, and the q
                // axis has no more than the rest of the circle, so that no state needs more than U.
                let lag = 1.0 - (-p.current_cycle * p.current_kp / fw.d_inductance).exp();
                let free = steady_d + fw.d_inductance * lag * (wanted_d - self.d_current) / p.current_cycle;
                let u_d = free.clamp(-max_voltage, max_voltage);
                if u_d == free {
                    self.d_current += lag * (wanted_d - self.d_current);
                } else {
                    self.d_current += (u_d - steady_d) * p.current_cycle / fw.d_inductance;
                }
                // The phase current's circle: i_q within what the d current leaves of it.
                let q_most = (limit * limit - self.d_current * self.d_current).max(0.0).sqrt();
                setpoint = setpoint.clamp(-q_most, q_most);
                (w * fw.d_inductance * self.d_current, u_d)
            }
            None => {
                // Braking where e alone exceeds U: the diodes force the current the circle's smaller root gives, and the
                // drive's current limiting cannot command less (`forced_braking_current`).
                if p.limit && speed != 0.0 && max_voltage.is_finite() {
                    let forced = p.forced_braking_current(speed.abs(), max_voltage);
                    if forced > 0.0 && setpoint * speed.signum() > -forced {
                        setpoint = -forced * speed.signum();
                    }
                }
                (0.0, steady_d)
            }
        };
        let q_voltage = (max_voltage * max_voltage - u_d * u_d).max(0.0).sqrt();
        self.d_voltage = u_d;
        let error = setpoint - measured;
        self.iron_loss = p.iron_loss_at(speed);
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
        match &p.motor {
            Motor::Servo => {
                // What the winding took and neither the copper, the shaft nor the inductance's energy accounts for:
                // exact, as the integrator took all four through the step together.
                self.emf_loss = if p.ke > 0.0 && (p.ke != p.kt || p.iron_drag_rated > 0.0) {
                    let stored = 0.5 * p.inductance * (current * current - self.current * self.current);
                    (electrical - copper - work - stored) / dt
                } else {
                    0.0
                };
                (self.current, self.torque) =
                    (current, p.torque_with(current, self.d_current, self.flux(p)) - p.iron_drag(self.phases.speed))
            }
            Motor::Stepper(s) => s.stepped(self, p),
            Motor::Induction(m) => m.stepped(self, p),
        }
        // A drive just enabled without the DC link simulated draws its winding's first L i^2 / 2 from the supply here.
        (self.power, self.copper_loss, self.shaft_power, self.discharge_power) =
            ((electrical + std::mem::take(&mut self.enable_energy)) / dt, copper / dt, work / dt, 0.0);
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
            let [slow, chip] = o.nodes_after(self.overload_nodes(), self.phase_current(), p.current_cycle);
            (self.overload_image, self.overload_chip) = (slow + chip, chip);
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
    ///
    /// `speed` is the rotor's, at the motor: a turning rotor has its iron loss and drag with the drive off (effects.iron_loss,
    /// `DriveParams::iron_drag`): `torque` is the drag, the shaft takes `-drag w` and the same power is heat in the iron
    /// (`iron_loss`; and `emf_loss`, the budget's row for it).
    pub fn disabled_step(&mut self, p: &DriveParams, speed: f64) {
        let freewheeling = self.winding_energy(p);
        (self.phases.x, self.phases.u, self.phases.integral) = ([0.0; 4], [0.0; 2], [0.0; 2]);
        (self.torque, self.torque_command, self.torque_demand, self.current, self.voltage, self.d_voltage) =
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let drag = p.iron_drag(speed);
        (self.power, self.copper_loss, self.shaft_power, self.d_current, self.iron_loss) = (
            -freewheeling / p.current_cycle,
            0.0,
            -drag * speed,
            0.0,
            if matches!(p.motor, Motor::Servo) { p.iron_loss_at(speed) } else { 0.0 },
        );
        (self.torque, self.emf_loss) = (-drag, drag * speed);
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
        if let Some(r) = &p.resistor_heating {
            self.resistor_rise = r.rise_after(self.resistor_rise, self.resistor_power, p.current_cycle);
        }
    }

    /// The copper loss of the step warms the winding; at the rated current it balances at the rated rise. Without
    /// current (a disabled drive) the same model cools it.
    ///
    /// With effects.motor_thermal_nodes (`ThermalNodes`), in units of the rise the loss would settle at (P R): the winding
    /// tau_w dT_w/dt = s P R - (T_w - T_h), the housing (tau - tau_w / s) dT_h/dt = (T_w - T_h) / s - T_h / (1 - s)
    /// + P_iron R. With one node the iron loss (effects.iron_loss) adds to the copper loss.
    fn warm(&mut self, p: &DriveParams) {
        if p.heating {
            let (heating, iron) = (p.iron_heating(self.copper_loss), p.iron_heating(self.iron_loss));
            let dt = p.current_cycle;
            match &p.thermal_nodes {
                None => {
                    self.temperature_rise += dt / p.thermal_time_constant * (heating + iron - self.temperature_rise)
                }
                // The iron loss heats the stator, the housing's node.
                Some(n) => {
                    let (s, across) = (n.winding_share, self.temperature_rise - self.housing_rise);
                    let housing_time = p.thermal_time_constant - n.winding_time_constant / s;
                    self.temperature_rise += dt / n.winding_time_constant * (s * heating - across);
                    self.housing_rise += dt / housing_time * (across / s - self.housing_rise / (1.0 - s) + iron);
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
    // The line module (or an S210's rectifier) loses its share either way: it draws drawn / efficiency from the line, and
    // of what it feeds back the efficiency arrives there. A step does one or the other, never both.
    DcLinkStep {
        voltage,
        resistor: burned / dt,
        drawn: drawn / p.line_efficiency / dt,
        mains: p.line_efficiency * fed / dt,
        line_loss: ((1.0 - p.line_efficiency) * fed + (1.0 / p.line_efficiency - 1.0) * drawn) / dt,
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
pub(crate) mod tests {
    use super::*;

    /// A drive of round numbers for the unit tests (the charts' too).
    pub(crate) fn params() -> DriveParams {
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
            fundamental_gain: 1.0,
            // Kp = L / T and Tn = L / R for T = 0.25 ms.
            current_kp: 40.0,
            current_ki: 40.0 / 2.5e-3,
            current_cycle: 62.5e-6,
            peak_torque: 3.8,
            rated_torque: 1.27,
            rated_speed: 314.0,
            no_load_speed: 420.0,
            max_speed: 420.0,
            limit: true,
            encoder_step: 0.0,
            velocity_filter: 1.0,
            torque_filter: 1.0,
            notch: None,
            velocity_cycle: 125e-6,
            current_delay: 0,
            velocity_delay: 0,
            inductance: 0.010,
            pole_pairs: 0.0,
            heating: false,
            rated_rise: 100.0,
            housing_max_rise: 100.0,
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
            motor: Motor::Servo,
            iron_loss_rated: 0.0,
            iron_drag_rated: 0.0,
            stall_current: 1.27 / 0.45,
            static_torque: 0.0,
            s1_curve: Vec::new(),
            magnet_coefficient: 0.0,
            resistor_heating: None,
        }
    }

    impl Drive {
        /// The current loop and the winding at a speed held, as `Sim::step` takes the winding through a base step
        /// with the mechanics: four RK4 sub-steps of the same rates.
        fn current_loop(&mut self, p: &DriveParams, speed: f64) {
            self.current_control(p, speed);
            let (voltage, resistance, d_current, flux) =
                (self.voltage, self.winding_resistance(p), self.d_current, self.flux(p));
            let mut y = [self.current, 0.0, 0.0, 0.0];
            let mut rk = sim_core::Rk4::new(4);
            for _ in 0..4 {
                rk.step(
                    |y, dy| dy.copy_from_slice(&p.winding_rates(voltage, resistance, y[0], speed, d_current, flux)),
                    &mut y,
                    p.current_cycle / 4.0,
                );
            }
            self.phases.speed = speed;
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
    /// is sqrt(2 * 10.5835 J / 470 uF) = 212.217 V. Motoring, the motor takes 49 W out of the DC link and the line module
    /// loses its 2 % drawing them too: 49 W / 0.98 = 50 W from the line, 5 J in 0.1 s, of which 0.1 J are lost.
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
        let (d, mains, lost, _, drawn) = run(&p, 0.45);
        assert!(
            d.dc_voltage == 190.0 && mains == 0.0 && (drawn - 5.0).abs() < 1e-9 && (lost - 0.1).abs() < 1e-9,
            "{drawn} J drawn, {lost} J lost"
        );
        // A line module without loss draws what the motor takes, 4.9 J (as before 14 November 2026).
        let (_, _, lost, _, drawn) = run(&DriveParams { line_efficiency: 1.0, ..p.clone() }, 0.45);
        assert!((drawn - 4.9).abs() < 1e-9 && lost == 0.0, "{drawn} J drawn");
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
                && (p.settled_rise(0.5 * p.rated_current, 0.0) - 19.34).abs() < 0.01,
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

    /// The velocity loop's torque limit follows the DC link's voltage of the moment (the field weakening test's motor, R
    /// left out, at 500 rad/s). The DC link 10 % low gives the winding 171 V of 190: on the current circle of 9.5 A,
    /// (225 + 20 i_d)^2 + (20 i_q)^2 = 171^2 is 50625 + 9000 i_d + 400 * 90.25 = 29241, i_d = -57484 / 9000 = -6.38711 A,
    /// i_q = sqrt(90.25 - 40.79522) = 7.03241 A, 0.45 * 7.03241 = 3.16459 N m (3.4451 N m at the full 190 V). Without
    /// field weakening the fade's speeds scale by 0.9, to 282.6 and 378 rad/s: at 330 rad/s 3.8 * 48 / 95.4 =
    /// 1.91195 N m where the full voltage gives 3.8 * 90 / 106 = 3.22642 N m.
    #[test]
    fn a_dc_link_ten_percent_low_lowers_the_velocity_loops_torque_limit_at_speed() {
        let bus = 190.0 / WINDING_PER_DC_LINK;
        let limit = |p: &DriveParams, dc: f64, speed: f64| {
            // An integrator far beyond the limit: the loop asks for more than it may give.
            let mut d = Drive { dc_voltage: dc, integral: 100.0, ..Drive::default() };
            d.velocity_loop(p, speed);
            d.torque_command
        };
        let p = DriveParams { dc_link: true, bus_voltage: bus, ..weakening() };
        let (full, low) = (limit(&p, bus, 500.0), limit(&p, 0.9 * bus, 500.0));
        assert!((full - 3.4451).abs() < 1e-3 && (low - 3.16459).abs() < 1e-3, "{full} N m, {low} N m");
        let fade = DriveParams { field_weakening: None, ..p.clone() };
        let (full, low) = (limit(&fade, bus, 330.0), limit(&fade, 0.9 * bus, 330.0));
        assert!((full - 3.22642).abs() < 1e-4 && (low - 1.91195).abs() < 1e-4, "{full} N m, {low} N m");
        // Without the DC link simulated the nominal voltage holds, whatever the drive's DC link field says.
        let nominal = DriveParams { dc_link: false, ..p.clone() };
        assert!((limit(&nominal, 0.9 * bus, 500.0) - 3.4451).abs() < 1e-3);
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
        assert!((p.d_current_for(7.6557, 500.0, 190.0, 9.5, p.resistance, 1.0) + 5.625).abs() < 1e-3);
        assert_eq!(p.d_current_for(1.0, 100.0, 190.0, 9.5, p.resistance, 1.0), 0.0, "45 V and 4 V reach");

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

    /// Without field weakening the winding's reactance is in the steady voltage: (R i + kE w)^2 + (p w L i)^2 <= U^2,
    /// the test motor (kt = kE = 0.45, R = 4 Ohm, L = 10 mH, 4 pole pairs, U = 190 V). At 300 rad/s kE w = 135 V and
    /// p w L = 12 Ohm: 160 i^2 + 1080 i + 18225 - 36100 = 0, i = (-540 + sqrt(291600 + 160 * 17875)) / 160 = 7.7206 A,
    /// 3.474 N m (without the reactance (190 - 135) / 4 = 13.75 A, held at 9.5 A: 4.275 N m). At 330 rad/s 2.709 N m,
    /// at 360 rad/s 1.956 N m. The loop gets there and its voltage vector sits on the circle: u_q = 4 * 7.7206 + 135 =
    /// 165.88 V, u_d = -12 * 7.7206 = -92.65 V, |u| = 190 V, a phase amplitude of 190 * sqrt 2 / 3 = 89.57 V
    /// (`phase_voltage`: U_dc / sqrt 3 for the DC link 190 / sqrt(3/2)).
    #[test]
    fn the_winding_reactance_limits_the_torque_without_field_weakening() {
        let p = DriveParams { pole_pairs: 4.0, ..params() };
        let at = |speed: f64| p.voltage_limited_torque(speed, 190.0 / WINDING_PER_DC_LINK).1;
        for (speed, torque) in [(300.0, 3.474), (330.0, 2.709), (360.0, 1.956)] {
            assert!((at(speed) - torque).abs() < 1e-3, "{speed} rad/s: {} N m", at(speed));
        }
        assert!((params().voltage_limited_torque(300.0, 190.0 / WINDING_PER_DC_LINK).1 - 4.275).abs() < 1e-9);
        let mut d = Drive { torque_command: 3.474, ..Drive::default() };
        for _ in 0..4000 {
            d.current_loop(&p, 300.0);
        }
        assert!(
            (d.current - 7.7206).abs() < 0.05 && (d.voltage.hypot(d.d_voltage) - 190.0).abs() < 0.2,
            "{} A",
            d.current
        );
        assert!((d.d_voltage + 92.65).abs() < 0.5 && (d.phase_voltage(&p) - 89.57).abs() < 0.1, "{} V", d.d_voltage);
        // The time simulation at constant speed, asked for the peak torque, settles at the limit at each speed (0.25 s).
        for (speed, torque) in [(300.0, 3.474), (330.0, 2.709), (360.0, 1.956)] {
            let mut d = Drive { torque_command: 3.8, ..Drive::default() };
            for _ in 0..4000 {
                d.current_loop(&p, speed);
            }
            assert!((d.torque - torque).abs() < 0.01 * torque, "{speed} rad/s: {} N m", d.torque);
        }
    }

    /// With field weakening the phase current, i_q and i_d together, stays within the drive's maximum current. The test
    /// motor with R left out on a 5 A drive at 500 rad/s: the circles (225 + 20 i_d)^2 + (20 i_q)^2 = 190^2 and
    /// i_q^2 + i_d^2 = 25 cross at 50625 + 9000 i_d + 10000 = 36100, i_d = -2.725 A, i_q = sqrt(25 - 7.4256) = 4.1922 A,
    /// 0.45 * 4.1922 = 1.8865 N m (the q current alone held to 5 A gave 2.25 N m and a phase current of about 5.9 A).
    #[test]
    fn the_drives_current_limit_holds_the_phase_current_with_field_weakening() {
        let p = DriveParams { drive_max_current: 5.0, ..weakening() };
        let (_, high) = p.torque_range(500.0);
        assert!((high - 1.8865).abs() < 1e-3, "{high} N m");
        let mut d = Drive { torque_command: 2.25, ..Drive::default() };
        for _ in 0..4000 {
            d.current_loop(&p, 500.0);
        }
        assert!(d.phase_current() < 5.0 + 1e-3, "{} A", d.phase_current());
    }

    /// Braking or motoring at the peak torque near the top speed does not run away. The audit's case, the T-shaped belt gantry's
    /// motor (this test motor with field weakening, 190 V) with saturation off, a step to -3.8 N m held at 400 and
    /// 420 rad/s, gave -19.1 N m at 42.5 A of phase current before the d axis was held within the voltage circle (commit
    /// 1d098351). Now, stepped from rest to +-3.8 N m and to the curve's two torques (the velocity loop's clamp left out)
    /// at 380 to 450 rad/s, saturation on and off, for 0.5 s: the torque stays within the peak 3.8 N m (the step overshoots
    /// by 0.06 %), the phase current within the motor's 9.5 A and the voltage vector within the 190 V circle at every
    /// step, and the torque settles at the command or, beyond the curve, at the curve (3.612, 3.470, 3.335, 3.145 N m
    /// motoring without saturation: a command above the curve took a d current filling the current circle and fell to
    /// nil before the d current was set for the most q current that fits).
    #[test]
    fn braking_at_the_voltage_limit_does_not_run_away() {
        let fw = FieldWeakening { pole_pairs: 4.0, d_inductance: 0.010 };
        for saturation in [0.0, 1.0 - 0.40 / 0.45] {
            let p = DriveParams { pole_pairs: 4.0, field_weakening: Some(fw), saturation, ..params() };
            for speed in [380.0, 400.0, 420.0, 450.0] {
                let (low, high) = p.torque_curve(speed);
                for command in [-3.8, 3.8, low, high] {
                    let mut d = Drive { torque_command: command, ..Drive::default() };
                    let (mut torque, mut current, mut voltage) = (0.0f64, 0.0f64, 0.0f64);
                    for _ in 0..8000 {
                        d.current_loop(&p, speed);
                        torque = torque.max(d.torque.abs());
                        current = current.max(d.phase_current());
                        voltage = voltage.max(d.voltage.hypot(d.d_voltage));
                    }
                    // It settles at the command, or at the curve where the command is beyond it.
                    let settled = command.clamp(low, high);
                    assert!(
                        torque <= 3.8 * 1.001
                            && current <= 9.5 + 1e-6
                            && voltage <= 190.0 + 1e-9
                            && (d.torque - settled).abs() < 1e-3 * settled.abs(),
                        "{saturation} {speed} rad/s {command} N m: {torque} N m, {current} A, {voltage} V, {} N m",
                        d.torque
                    );
                }
            }
        }
    }

    /// The magnets' flux with the thermal nodes is 1 at the rated state: the housing then stands at (1 - s) of the
    /// winding's rated rise, 0.75 * 100 K = 75 K for s = 0.25 (the winding's rise 100 K was what the flux compared).
    #[test]
    fn the_magnet_flux_is_one_at_the_rated_state_with_thermal_nodes() {
        let p = DriveParams {
            heating: true,
            magnet_coefficient: -1e-3,
            thermal_nodes: Some(ThermalNodes { winding_share: 0.25, winding_time_constant: 60.0 }),
            ..params()
        };
        let d = Drive { temperature_rise: 100.0, housing_rise: 75.0, ..Drive::default() };
        assert!((d.flux(&p) - 1.0).abs() < 1e-12, "{}", d.flux(&p));
        let one_node = DriveParams { thermal_nodes: None, ..p };
        assert!((d.flux(&one_node) - 1.0).abs() < 1e-12);
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
        assert_eq!(p.d_current_for(2.0, 314.16, p.max_voltage, 7.6, p.resistance, 1.0), 0.0);
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
        let o = Overload {
            rated_current: 3.0,
            base_current: 2.6,
            max_current: 9.0,
            time_constant: 10.0,
            trip,
            chip_share: 0.0,
            chip_time_constant: 1.0,
        };
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
        let o = Overload {
            rated_current: 1.0,
            base_current: 0.85,
            max_current: 3.0,
            time_constant: 10.0,
            trip: 1.2,
            chip_share: 0.0,
            chip_time_constant: 1.0,
        };
        let pass = |current: &dyn Fn(f64) -> f64| {
            let (mut x, mut samples) = ([0.0; 2], Vec::new());
            for k in 1..=1000 {
                x = o.nodes_after(x, current(k as f64 * 1e-3 - 0.5e-3), 1e-3);
                samples.push((k as f64 * 1e-3, x));
            }
            let end = samples.pop().unwrap();
            crate::analysis::repeating_peak(&o, [0.0; 2], &samples, end)
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
        assert!((p.settled_rise(p.rated_current, 0.0) - 100.0).abs() < 1e-9);
        // A winding started warm starts with its housing settled for it.
        let warm = DriveParams { start_rise: 40.0, ..p.clone() };
        assert_eq!(Drive::at_rest(&warm, 0.0, 0.0, 0.0).housing_rise, 30.0);
    }

    /// The voltage-limit torque of the d-q model against a hand calculation, for the 1FK2205-4AF as the catalogue puts it
    /// on a 400 V line: kt = 6 N m / 4.7 A = 1.27660 N m/A, R = 2 Ohm (the config's estimate), 4 pole pairs, L_d = L_q =
    /// kt^2 / (2 * 4 * 13 N m) = 15.670 mH from Siemens' maximum short-circuit braking torque (`catalog::apply_motor`),
    /// 15.1 A, and the DC link of 540 V: the winding gets sqrt(3/2) * 540 = 661.38 V. At 600 rad/s the back-EMF alone,
    /// 765.96 V, is above it: without a d current no torque (and the old fade from 314.16 to 518.07 rad/s none either).
    /// By hand, stepping i_d from 0 to -15.1 A in 0.1 mA and taking for each the largest i_q on both circles (the
    /// quadratic (R i_q + kt w + w L i_d)^2 + (R i_d - w L i_q)^2 = U^2 with w = 4 * 600): the best is i_d = -8.855 A,
    /// i_q = 12.2309 A, 15.614 N m. At 500 rad/s it is 17.818 N m where the fade gave 1.595 N m.
    #[test]
    fn the_voltage_limit_gives_the_hand_calculated_torque_from_r_l_and_the_dc_link() {
        let kt: f64 = 6.0 / 4.7;
        let inductance = kt * kt / (2.0 * 4.0 * 13.0);
        assert!((inductance - 0.015670).abs() < 1e-6);
        let p = DriveParams {
            kt,
            ke: kt,
            resistance: 2.0,
            inductance,
            max_voltage: WINDING_PER_DC_LINK * 540.0,
            max_current: 15.1,
            peak_torque: 18.0,
            rated_speed: 314.159,
            no_load_speed: WINDING_PER_DC_LINK * 540.0 / kt,
            field_weakening: Some(FieldWeakening { pole_pairs: 4.0, d_inductance: inductance }),
            ..params()
        };
        let (at_600, at_500) = (p.torque_curve(600.0).1, p.torque_curve(500.0).1);
        assert!((at_600 - 15.614).abs() < 2e-3 && (at_500 - 17.818).abs() < 2e-3, "{at_600} N m, {at_500} N m");
        let fade = DriveParams { field_weakening: None, ..p.clone() };
        assert!(fade.torque_curve(600.0).1 == 0.0 && (fade.torque_curve(500.0).1 - 1.595).abs() < 1e-3);
        // The drive's d current for that q current at 600 rad/s is the hand calculation's.
        let d = p.d_current_for(12.2309, 600.0, p.max_voltage, 15.1, p.resistance, 1.0);
        assert!((d + 8.855).abs() < 2e-3, "{d} A");
    }

    /// The d axis follows its setpoint as the tuned current loop does: with Kp = 40 V/A and L_d = 10 mH the time constant
    /// is L_d / Kp = 0.25 ms, four current cycles of 62.5 us. At 500 rad/s the setpoint is -5.625 A (the field weakening
    /// test); unbounded, from 0 the d current would be -5.625 (1 - exp(-1)) = -3.5557 A after four cycles, and -5.625
    /// (1 - exp(-10)) after forty. Before 14 November 2026 it was at -5.625 A in the first. The d voltage is held within the
    /// 190 V circle (the q current is 0 here, so the steady d voltage is R i_d, nil): the first cycle's share
    /// e = 1 - exp(-1/4) = 0.22120 of 5.625 A asks for L_d e 5.625 / 62.5 us = 199.1 V, so it moves 190 V * 62.5 us / 10 mH
    /// = 1.1875 A; then 0.22120 of what is left each cycle (157 V, 122 V, 96 V, within it): -2.1692, -2.9336, -3.5289 A.
    #[test]
    fn the_d_current_follows_its_setpoint_with_the_current_loop_time_constant() {
        let p = weakening();
        let mut d = Drive { torque_command: p.torque_curve(500.0).1, ..Drive::default() };
        d.current_control(&p, 500.0);
        assert!((d.d_current + 1.1875).abs() < 1e-4 && (d.d_voltage + 190.0).abs() < 1e-9, "{} A", d.d_current);
        for _ in 1..4 {
            d.current_control(&p, 500.0);
        }
        assert!((d.d_current + 3.5289).abs() < 2e-4, "{} A", d.d_current);
        for _ in 4..40 {
            d.current_control(&p, 500.0);
        }
        assert!((d.d_current + 5.625 * (1.0 - (-10f64).exp())).abs() < 2e-3, "{} A", d.d_current);
    }

    /// The d current takes the winding warm as it is: at 500 rad/s with R = 4 Ohm cold and 5.56 Ohm 100 K warm, the
    /// voltage circle asks for more d current warm, and the drive sets that one.
    #[test]
    fn the_d_current_takes_the_warm_resistance() {
        let p = DriveParams { resistance: 4.0, heating: true, ..weakening() };
        let (cold, warm) = (
            p.d_current_for(5.0, 500.0, 190.0, 9.5, 4.0, 1.0),
            p.d_current_for(5.0, 500.0, 190.0, 9.5, p.resistance_at(100.0), 1.0),
        );
        assert!(warm < cold - 0.01, "{warm} A warm, {cold} A cold");
        let mut d = Drive { torque_command: 0.45 * 5.0, temperature_rise: 100.0, ..Drive::default() };
        for _ in 0..400 {
            d.current_control(&p, 500.0);
        }
        assert!((d.d_current - warm).abs() < 1e-6, "{} A against {warm} A", d.d_current);
    }

    /// effects.iron_loss against a hand calculation. R = 4 Ohm, warm at the rated 100 K: 5.56 Ohm. Stall current 3 A and
    /// rated current 2.5 A: the iron loss at the rated speed is 5.56 * (9 - 6.25) = 15.29 W, and at a quarter of the speed
    /// 15.29 * 0.25^1.5 = 1.911 W. At the rated current and speed the winding settles at the rated 100 K, copper and iron
    /// together, as Siemens' rated data say; at the stall current at standstill too.
    #[test]
    fn iron_loss_at_a_speed_is_the_hand_calculated_share_of_the_rated_loss() {
        let p = DriveParams {
            resistance: 4.0,
            rated_current: 2.5,
            stall_current: 3.0,
            iron_loss_rated: 4.0 * (1.0 + COPPER * 100.0) * (9.0 - 6.25),
            heating: true,
            current_cycle: 1e-3,
            thermal_time_constant: 100.0,
            ..params()
        };
        assert!((p.iron_loss_rated - 15.29).abs() < 1e-9);
        assert!((p.iron_loss_at(p.rated_speed / 4.0) - 1.911_25).abs() < 1e-9 && p.iron_loss_at(0.0) == 0.0);
        let at_rated = p.settled_rise(2.5, p.iron_loss_rated);
        assert!((at_rated - 100.0).abs() < 1e-9 && (p.settled_rise(3.0, 0.0) - 100.0).abs() < 1e-9, "{at_rated} K");
        // The one node integrated to its end: copper at the rated current with its resistance warm, and the iron loss.
        let mut d = Drive::default();
        for _ in 0..2_000_000 {
            d.copper_loss = p.resistance_at(d.temperature_rise) * 2.5 * 2.5;
            d.iron_loss = p.iron_loss_at(p.rated_speed);
            d.warm(&p);
        }
        assert!((d.temperature_rise - 100.0).abs() < 1e-3, "{} K", d.temperature_rise);
        // Off, the rated current alone brings the winding to 100 K (the iron loss lumped in), as before.
        let off = DriveParams { iron_loss_rated: 0.0, stall_current: 2.5, ..p.clone() };
        assert!((off.settled_rise(2.5, 0.0) - 100.0).abs() < 1e-9);
    }

    /// effects.magnet_temperature: the catalogue's torque constant holds at the rated rise; a cold motor (0 K) with
    /// -0.12 %/K has 1 + 0.0012 * 100 = 1.12 times its flux: 12 % more torque per amp, and back-EMF.
    #[test]
    fn cold_magnets_give_the_hand_calculated_torque_per_amp() {
        let p = DriveParams { heating: true, magnet_coefficient: -0.0012, ..params() };
        let cold = Drive::default();
        assert!((cold.flux(&p) - 1.12).abs() < 1e-12);
        assert!((p.torque_with(2.0, 0.0, cold.flux(&p)) - 1.12 * 0.9).abs() < 1e-12);
        assert_eq!(Drive { temperature_rise: 100.0, ..Drive::default() }.flux(&p), 1.0);
        assert_eq!(cold.flux(&DriveParams { magnet_coefficient: 0.0, ..p.clone() }), 1.0);
    }

    /// The two-node overload image against a hand calculation, on the 3 A S120 Motor Module of the one-node test: 9 A for
    /// 0.25 s from the rated load, slow node 10 s, chips 30 % of the rise with 1 s. Trip level
    /// 9 - 8 (0.7 exp(-0.025) + 0.3 exp(-0.25)) = 1.66914. From the rated load held (image 1: 0.7 + 0.3), 6 A fill it as
    /// 4 - 3 (0.7 exp(-t / 10 s) + 0.3 exp(-t / 1 s)): by bisection it reaches the trip after 0.8203 s, where the one
    /// node (trip 1.19752, 4 - 3 exp(-t / 10 s)) takes 0.6811 s.
    #[test]
    fn the_two_node_overload_image_trips_after_the_hand_calculated_time() {
        let mut o = Overload {
            rated_current: 3.0,
            base_current: 2.6,
            max_current: 9.0,
            time_constant: 10.0,
            trip: 0.0,
            chip_share: 0.3,
            chip_time_constant: 1.0,
        };
        o.trip = 9.0 - 8.0 * o.decay(0.25).iter().sum::<f64>();
        assert!((o.trip - 1.66914).abs() < 1e-5, "{}", o.trip);
        assert!((o.trip_time(6.0) - 0.8203).abs() < 1e-4, "{} s", o.trip_time(6.0));
        let one = Overload { chip_share: 0.0, trip: 9.0 - 8.0 * (-0.025f64).exp(), ..o };
        assert!((one.trip_time(6.0) - 0.6811).abs() < 1e-4 && o.trip_time(2.9) == f64::INFINITY);
        // The drive: the image starts at the rated load, the torque asks for 6 A; it clamps to 2.6 A at the trip.
        let p = DriveParams { overload: Some(o), torque_ff: 1.0, drive_max_current: 9.0, ..params() };
        let mut d = Drive { torque_ff: 0.45 * 6.0, overload_image: 1.0, overload_chip: 0.3, ..Drive::default() };
        let mut t = 0.0;
        while d.torque_command != 0.45 * 2.6 && t < 2.0 {
            d.velocity_loop(&p, 0.0);
            for _ in 0..2 {
                d.current_loop(&p, 0.0);
                t += p.current_cycle;
            }
        }
        // The current reaches 6 A within a few cycles, and the clamp comes at the velocity cycle after the trip.
        assert!((t - 0.8203).abs() < 2e-3, "clamps after {t} s");
    }

    /// effects.braking_resistor_heating against a hand calculation: the S120 Braking Module's resistor (1.5 kW continuous,
    /// 100 kW peak for 1 s in 68 s), a time constant of 60 s and 200 K at the continuous power. A pulse of its peak power
    /// for 1 s from cold: 200 * 100 / 1.5 * (1 - exp(-1 / 60)) = 220.38 K; 67 s later, the rest of its cycle, 72.15 K.
    #[test]
    fn the_braking_resistor_warms_by_the_hand_calculated_rise_in_a_pulse() {
        let r = ResistorHeating { continuous_power: 1500.0, time_constant: 60.0, rated_rise: 200.0 };
        let p = DriveParams { dc_link: true, resistor_heating: Some(r), ..params() };
        let mut d = Drive::default();
        let pulse = |d: &mut Drive, power: f64, seconds: f64| {
            for _ in 0..(seconds / p.current_cycle).round() as usize {
                let step = DcLinkStep { voltage: 190.0, resistor: power, drawn: 0.0, mains: 0.0, line_loss: 0.0 };
                d.set_dc_link(&p, step);
            }
        };
        pulse(&mut d, 100e3, 1.0);
        assert!((d.resistor_rise - 220.38).abs() < 0.01, "{} K", d.resistor_rise);
        pulse(&mut d, 0.0, 67.0);
        assert!((d.resistor_rise - 72.15).abs() < 0.01, "{} K", d.resistor_rise);
        // Held at the continuous power it settles at the rated rise.
        assert!((r.rise_after(0.0, 1500.0, 1e6) - 200.0).abs() < 1e-9);
    }

    /// A back-EMF constant of 0.5 V s/rad beside the torque constant 0.45 N m/A (R = 4 Ohm, winding limit 190 V, no
    /// reactance without pole pairs). The voltage left at 340 rad/s is 190 - 0.5 * 340 = 20 V, so i = 20 / 4 = 5 A and
    /// 0.45 * 5 = 2.25 N m (with kE = kt: 190 - 153 = 37 V, 9.25 A, 4.1625 N m). Holding 2 A (0.9 N m) at 100 rad/s the
    /// winding needs u = R i + kE w = 8 + 50 = 58 V and takes 116 W: copper 4 * 4 = 16 W, shaft 0.45 * 2 * 100 = 90 W, and
    /// (kE - kt) i w = 0.05 * 2 * 100 = 10 W is the loss row's. With equal constants that row is exactly 0.
    #[test]
    fn a_back_emf_constant_apart_from_the_torque_constant_costs_voltage_and_books_the_difference() {
        let p = DriveParams { ke: 0.5, ..params() };
        let limited = p.voltage_limited_torque(340.0, 190.0 / WINDING_PER_DC_LINK);
        assert!((limited.1 - 2.25).abs() < 1e-9, "{limited:?}");
        let same = params().voltage_limited_torque(340.0, 190.0 / WINDING_PER_DC_LINK);
        assert!((same.1 - 4.1625).abs() < 1e-9, "{same:?}");
        let mut d = Drive::at_rest(&p, 0.0, 0.9, 0.0);
        d.current_loop(&p, 100.0);
        assert!((d.emf_loss - 10.0).abs() < 1e-6, "{} W", d.emf_loss);
        assert!((d.power - 116.0).abs() < 1e-6 && (d.power - d.copper_loss - d.shaft_power - d.emf_loss).abs() < 1e-6);
        let mut d = Drive::at_rest(&params(), 0.0, 0.9, 0.0);
        d.current_loop(&params(), 100.0);
        assert_eq!(d.emf_loss, 0.0);
    }

    /// The printed S1 curve (0, 2.0), (100, 1.5), (300, 0.5): linear between its points, the last beyond the last, the
    /// same both ways: at 200 rad/s 1.5 - 0.5 * 100 / 200 = 1.0 N m; one starting at 50 rad/s holds its first torque below.
    /// The config's arrays are read so (`s1_curve_of`), become the drive's `s1_torque`; empty ones are the straight line,
    /// and arrays of different lengths, falling speeds or a negative torque are refused.
    #[test]
    fn the_printed_s1_curve_is_read_linearly() {
        let c = [[0.0, 2.0], [100.0, 1.5], [300.0, 0.5]];
        assert_eq!(s1_interpolated(&c, 0.0), 2.0);
        assert!((s1_interpolated(&c, -200.0) - 1.0).abs() < 1e-12);
        assert_eq!(s1_interpolated(&c, 400.0), 0.5);
        assert_eq!(s1_interpolated(&[[50.0, 2.0], [100.0, 1.0]], 20.0), 2.0);
        let read = |speeds: &str, torques: &str| {
            s1_curve_of(
                &Config::parse(&format!(
                    "format_version = 1\n[motor]\ns1_curve_speed_rad_s = {speeds}\ns1_curve_torque_nm = {torques}\n"
                ))
                .unwrap(),
            )
        };
        let curve = read("[0.0, 100.0, 300.0]", "[2.0, 1.5, 0.5]").unwrap();
        let p = DriveParams { s1_curve: curve, static_torque: 1.8, ..params() };
        assert!((p.s1_torque(-200.0) - 1.0).abs() < 1e-12 && p.s1_torque(0.0) == 2.0);
        assert!(read("[]", "[]").unwrap().is_empty());
        assert!(s1_curve_of(&Config::parse("format_version = 1\n").unwrap()).unwrap().is_empty());
        for (speeds, torques) in
            [("[0.0, 100.0]", "[2.0]"), ("[100.0, 50.0]", "[2.0, 1.0]"), ("[0.0, 1.0]", "[1.0, -1.0]")]
        {
            assert!(read(speeds, torques).is_err(), "{speeds} {torques}");
        }
    }

    /// The S1 line from the static torque (the stall current's, 1.8 N m here) to the rated torque at the rated speed:
    /// at the rated speed 314 rad/s the rated 1.27 N m, halfway 1.535 N m, and flat at the rated torque without a static one.
    #[test]
    fn the_s1_torque_runs_from_the_static_torque_to_the_rated_one() {
        let p = DriveParams { static_torque: 1.8, ..params() };
        assert!((p.s1_torque(0.0) - 1.8).abs() < 1e-12 && (p.s1_torque(-314.0) - 1.27).abs() < 1e-12);
        assert!((p.s1_torque(157.0) - 1.535).abs() < 1e-12);
        assert_eq!(params().s1_torque(0.0), 1.27);
    }

    /// effects.iron_loss as a drag on the shaft. k_t 0.45 N m/A at the rated current 2.5 A gives 1.125 N m in the air gap;
    /// a printed rated torque of 1.0 N m and a shaft friction of 0.025 N m at the rated speed 314 rad/s leave the drag
    /// T_N = 0.1 N m there (P = 31.4 W), at a quarter of the speed 0.1 * sqrt(0.25) = 0.05 N m (P = 3.925 W = 31.4 / 8),
    /// odd in the speed and 0 at standstill. At the rated current the shaft gets 1.125 - 0.1 = 1.025 N m, the printed
    /// 1.0 N m once the friction has its 0.025. The winding takes u i = R i^2 + L i di/dt + air-gap power: with the drag in
    /// the shaft's power, electrical power = copper + stored + shaft + 31.4 W, the last being `emf_loss` (equal
    /// constants). The heat the winding feels stays the thermal data's R (I_0^2 - I_N^2) = 15.29 W (I_0 = 3 A, R = 4 Ohm
    /// warm at 100 K). A disabled drive turning at the rated speed: torque -0.1 N m, shaft power -31.4 W, `emf_loss` 31.4 W.
    #[test]
    fn the_iron_loss_drags_the_shaft_and_the_budget_closes_with_it() {
        let p = DriveParams {
            resistance: 4.0,
            rated_current: 2.5,
            stall_current: 3.0,
            iron_loss_rated: 4.0 * (1.0 + COPPER * 100.0) * (9.0 - 6.25),
            iron_drag_rated: 1.125 - 1.0 - 0.025,
            rated_speed: 314.0,
            ..params()
        };
        let (heat, drag) = (p.iron_loss_rated, 31.4);
        assert!((p.iron_drag(314.0) - 0.1).abs() < 1e-12 && (p.iron_drag(314.0) * 314.0 - drag).abs() < 1e-9);
        assert!((p.iron_drag(78.5) - 0.05).abs() < 1e-12 && (p.iron_drag(78.5) * 78.5 - 3.925).abs() < 1e-9);
        assert_eq!((p.iron_drag(-314.0), p.iron_drag(0.0)), (-p.iron_drag(314.0), 0.0));
        let mut rates = [0.0; WINDING_STATES];
        let drive = Drive { voltage: 30.0, ..Drive::default() };
        let shaft = p.motor_rates(&drive, &[2.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.0, 314.0, &mut rates);
        assert!((shaft - 1.025).abs() < 1e-9, "{shaft} N m");
        // The same instant's rates: u i - R i^2 - L i di/dt - shaft power = the drag's power.
        let r = p.winding_rates(30.0, 4.0, 2.5, 314.0, 0.0, 1.0);
        assert!((r[1] - r[2] - p.inductance * 2.5 * r[0] - r[3] - drag).abs() < 1e-9);
        // Held for a step through the winding: the budget row holds the drag's power; the heat signal the thermal one.
        let mut d = Drive::at_rest(&p, 0.0, 1.125, 0.0);
        d.current_loop(&p, 314.0);
        assert!((d.power - d.copper_loss - d.shaft_power - d.emf_loss).abs() < 1e-6, "{} W", d.power);
        assert!((d.emf_loss - drag).abs() < 1e-3 * drag && d.iron_loss == heat, "{} W", d.emf_loss);
        let mut off = Drive { current: 1.0, ..Drive::default() };
        off.disabled_step(&p, 314.0);
        assert!((off.torque + 0.1).abs() < 1e-12, "{} N m", off.torque);
        assert!((off.shaft_power + drag).abs() < 1e-9 && (off.emf_loss - drag).abs() < 1e-9 && off.iron_loss == heat);
        off.disabled_step(&p, 0.0);
        assert_eq!((off.torque, off.shaft_power, off.emf_loss), (0.0, 0.0, 0.0));
    }

    /// The d current takes the drive's unsaturated back-EMF (`d_current_for`): q = 5 A at 500 rad/s, e = 0.45 * 500 =
    /// 225 V, w L_d = 20 Ohm, w L q = 100 V, R = 0: the circle (e + 20 d)^2 + 100^2 = 190^2 gives e + 20 d =
    /// sqrt(26100) = 161.5549 V, d = (161.5549 - 225) / 20 = -3.17225 A, with 20 % saturation as without (the winding's
    /// saturated 212.53 V would want -2.54898 A: the 0.623 A more is the reserve).
    #[test]
    fn the_d_current_keeps_the_unsaturated_back_emf() {
        let p = DriveParams { saturation: 0.2, ..weakening() };
        let d = p.d_current_for(5.0, 500.0, 190.0, 9.5, p.resistance, 1.0);
        assert!((d + 3.17225).abs() < 1e-4, "{d} A");
        let plain = weakening().d_current_for(5.0, 500.0, 190.0, 9.5, 1e-6, 1.0);
        assert!((plain + 3.17225).abs() < 1e-4, "{plain} A");
    }

    /// Braking above e = U. R = 4 Ohm, kE = 0.45, p = 2, L = 10 mH, U = 190 V, 20 A allowed. At 450 rad/s e = 202.5 V
    /// and x = 2 * 450 * 0.01 = 9 Ohm, a = 16 + 81 = 97: (202.5 - 4 i)^2 + (9 i)^2 = 190^2 is 97 i^2 - 1620 i + 4906.25 = 0,
    /// i = (1620 +- sqrt(2624400 - 1903625)) / 194 = (1620 +- 848.985) / 194 = 12.7267 A or 3.9743 A: the interval
    /// of braking currents the voltage leaves, the diodes force the smaller one. (Before: (U + e) / R = 98 A.) At 500 rad/s
    /// (e = 225 V, x = 10, a = 116) the circle leaves none: the nearest current is R e / a = 7.7586 A, both ways. The
    /// current loop asked for no torque at 450 rad/s brakes with the forced current.
    #[test]
    fn braking_above_the_back_emf_equal_to_the_voltage_solves_the_voltage_circle() {
        let p = DriveParams { pole_pairs: 2.0, max_current: 20.0, ..params() };
        let (most, forced) = (p.voltage_limited_current(450.0, 190.0, false), p.forced_braking_current(450.0, 190.0));
        assert!((most - 12.7267).abs() < 1e-3 && (forced - 3.9743).abs() < 1e-3, "{most} A, {forced} A");
        for i in [most, forced] {
            assert!(((202.5 - 4.0 * i).powi(2) + (9.0 * i).powi(2) - 190.0 * 190.0).abs() < 1e-6);
        }
        let none = 4.0 * 225.0 / 116.0;
        assert!((p.voltage_limited_current(500.0, 190.0, false) - none).abs() < 1e-9);
        assert!((p.forced_braking_current(500.0, 190.0) - none).abs() < 1e-9);
        // Below e = U nothing is forced.
        assert_eq!(p.forced_braking_current(300.0, 190.0), 0.0);
        let mut d = Drive { torque_command: 0.0, ..Drive::default() };
        for _ in 0..4000 {
            d.current_loop(&p, 450.0);
        }
        assert!((d.current + forced).abs() < 0.05, "{} A against -{forced} A", d.current);
    }

    /// The linear fade's speeds at another voltage. R = 4 Ohm, kE = 0.45, no reactance, 9.5 A allowed, printed fade 314 to
    /// 420 rad/s at 190 V. Rated point: the voltage leaves (190 - 141.3) / 4 = 12.2 A, clamped to 9.5 A, which it holds
    /// up to (190 - 38) / 0.45 = 337.78 rad/s; at 152 V (0.8) up to (152 - 38) / 0.45 = 253.33: ratio 0.75, so 235.5 rad/s.
    /// No-load point: (190 - 189) / 4 = 0.25 A, left up to 420 rad/s; at 152 V (152 - 1) / 0.45 = 335.56 rad/s.
    /// At 300 rad/s the torque is 3.8 (335.56 - 300) / (335.56 - 235.5) = 1.3504 N m (at 190 V the full 3.8 N m).
    #[test]
    fn the_linear_fade_follows_r_and_ke_at_another_voltage() {
        let p = params();
        assert_eq!(p.torque_curve_at(300.0, 190.0).1, 3.8);
        let low = p.torque_curve_at(300.0, 152.0).1;
        let by_hand = 3.8 * (151.0 / 0.45 - 300.0) / (151.0 / 0.45 - 235.5);
        assert!((low - by_hand).abs() < 1e-6 && (low - 1.3504).abs() < 1e-3, "{low} N m");
    }

    /// drive.overmodulation on a three-phase bridge: the winding's limit is 2 sqrt 3 / pi = 1.10266 times the space-vector
    /// one (six-step's fundamental 2 U / pi over U / sqrt 3). At 440 rad/s, e = 198 V, R = 4 Ohm, without reactance:
    /// (190 * 1.10266 - 198) / 4 = (209.505 - 198) / 4 = 2.87625 A, 0.45 times that = 1.29431 N m (sinusoidal: 190 < 198,
    /// none).
    #[test]
    fn overmodulation_raises_the_voltage_limit_to_the_six_step_fundamental() {
        let gain = 2.0 * 3f64.sqrt() / std::f64::consts::PI;
        let p = DriveParams { fundamental_gain: gain, ..params() };
        let dc = 190.0 / WINDING_PER_DC_LINK;
        assert!((p.dc_gain() - gain * WINDING_PER_DC_LINK).abs() < 1e-15);
        let by_hand = 0.45 * (190.0 * gain - 198.0) / 4.0;
        assert!((p.voltage_limited_torque(440.0, dc).1 - by_hand).abs() < 1e-9 && (by_hand - 1.29431).abs() < 1e-5);
        assert_eq!(params().voltage_limited_torque(440.0, dc).1, 0.0);
    }
}
