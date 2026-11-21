//! Pneumatic cylinders beside the servo axes: a double-acting cylinder moved by a directional valve, not by a drive.
//! `attach` puts the cylinders of a config on any machine; each brings states of its own after the machine's, its
//! signals (`cylinder.<id>.*`) and its checks (`cushion-energy`, `load-factor`). The cycle commands it by its valve
//! (a segment's `valves`) and waits for its end-position sensors (a segment's `wait`), see `control`.
//!
//! The model (docs/model-reference.md, "Pneumatic cylinder"):
//!
//! - Two chambers, cap side (1, area A1 = pi D^2 / 4) and rod side (2, A2 = A1 - pi d^2 / 4), each a control volume of
//!   an ideal gas (R = 287.058 J/(kg K), kappa = 1.4) with its mass m and internal energy U as states:
//!   V1 = V01 + A1 x, V2 = V02 + A2 (s - x), p = (kappa - 1) U / V, T = p V / (m R),
//!   dm/dt = mdot, dU/dt = h_in mdot + Q - p dV/dt, h = cp T of the gas that flows (the port's going in, the chamber's
//!   going out), Q = alpha A (T_wall - T) (`heat_transfer_w_k`). alpha A = 0 is adiabatic; alpha A large against
//!   m cv / (the process's time) is isothermal: the two limits.
//! - Every flow path an ISO 6358 element of sonic conductance C and critical pressure ratio b: mdot = C p_up rho0
//!   sqrt(T0 / T_up) psi, psi = 1 at p_down / p_up <= b (choked), sqrt(1 - ((r - b) / (1 - b))^2) above; rho0 =
//!   1.185 kg/m3 and T0 = 293.15 K (ANR, ISO 8778). Elements in series (valve, tube, throttle) combine to
//!   C = (sum C_i^-3)^(-1/3) and 1 - b = C^2 sum (1 - b_i) / C_i^2: the second keeps the sum of the pressure drops at
//!   small drops, where the elliptic law is dp ~ mdot^2 (1 - b) / C^2 (`Flow::series`).
//! - A 5/2 valve (or a 5/3 with a closed centre) that switches `valve_switch_time_s` after its command; inflow through
//!   valve and tube (the throttles' and the cushions' check valves open), outflow also through the meter-out throttle
//!   of that chamber, or within the cushion length of its end through the cushion's bleed screw behind that throttle.
//! - The piston: (m + payload) dv/dt = p1 A1 - p2 A2 - p_atm (A1 - A2) - F_load - m g sin(incline) - F_seal + F_stop,
//!   the seal's friction a Stribeck curve whose Coulomb level rises with the chambers' mean gauge pressure, the end
//!   stops `mech::HardStop` (elastic bumpers).
//!
//! shortcut: the flow law's last 0.5 % before the pressures meet is a straight line through zero (`LINEAR`), not
//! the root: the root's slope is infinite there, which no fixed step integrates. ISO 6358-3 has a laminar region of
//! its own there; take it in if a test against measured data ever needs the last 0.5 % of a pressure.
//! The cylinder rides on the machine without acting back on it: its weight and its reaction are in the masses of the
//! machine's body that carries it. That is exact only for a cylinder at right angles to the linear axes that carry it
//! and on no rotary one, so `attach` checks it (`along_carrying_axis`): a cylinder extends in the x-z plane at
//! `incline_deg` above x, direction (cos, 0, sin), and a file whose cylinder has a component along the direction of a
//! linear axis that carries it, or rides on a rotary axis, is refused.

use crate::mech::HardStop;
use sim_core::{AxisInfo, BeltSection, FrictionContact, MotorInfo, ThermalPart};
use sim_core::{Config, ConfigError, Machine, RatingCheck, Scene, SignalInfo};

/// Gas constant of air, J/(kg K), and its ratio of specific heats.
pub const R: f64 = 287.058;
pub const KAPPA: f64 = 1.4;
pub const CV: f64 = R / (KAPPA - 1.0);
pub const CP: f64 = KAPPA * CV;
/// ANR reference (ISO 8778, the reference of ISO 6358): density and temperature.
pub const RHO0: f64 = 1.185;
pub const T0: f64 = 293.15;
/// A catalogue's dm^3/(s bar) in m^3/(s Pa).
pub const DM3_S_BAR: f64 = 1e-3 / 1e5;
/// The flow law is a straight line over the last `LINEAR` of the pressure ratio before 1 (see the module's shortcut).
const LINEAR: f64 = 0.005;
/// The most an eigenvalue (rate, or angular frequency) times the integrator's sub-step may be: RK4 is stable to 2.78 on
/// the real axis and 2.83 on the imaginary one; 2 leaves room for what the estimates leave out.
const STABLE: f64 = 2.0;
const G: f64 = 9.80665;
/// States per cylinder: position, speed, mass and internal energy of the cap and the rod chamber, then running totals
/// for the balances: air drawn from the supply (kg), enthalpy from the supply and to the exhaust (J), heat from the
/// wall (J), work lost to seal friction and the stops' dampers (J).
pub const STATES: usize = 11;
const X: usize = 0;
const V: usize = 1;
const M1: usize = 2;
const U1: usize = 3;
const M2: usize = 4;
const U2: usize = 5;
pub const AIR: usize = 6;
pub const H_IN: usize = 7;
pub const H_OUT: usize = 8;
pub const HEAT: usize = 9;
pub const LOSS: usize = 10;

/// The keys of a cylinder's table (`values` of a `[[machine.cylinder]]`), with placeholder values: what a machine
/// file's schema adds for each cylinder. docs/config-reference.md tells them.
pub const TABLE: &str = r#"
piston_diameter_m = 0.032
rod_diameter_m = 0.012
stroke_m = 0.1
moved_mass_kg = 0.5
payload_kg = 0.0
incline_deg = 0.0
dead_volume_cap_m3 = 5.0e-6
dead_volume_rod_m3 = 5.0e-6
supply_pressure_bar = 6.0
ambient_pressure_pa = 101325.0
ambient_temperature_k = 293.15
heat_transfer_w_k = 0.05
valve = "5/2"
valve_c_dm3_s_bar = 1.0
valve_b = 0.3
valve_switch_time_s = 0.015
tube_c_dm3_s_bar = 2.0
tube_b = 0.4
throttle_extend_c_dm3_s_bar = 0.3
throttle_retract_c_dm3_s_bar = 0.3
throttle_b = 0.3
cushion_length_m = 0.017
cushion_bleed_c_dm3_s_bar = 0.05
cushion_bleed_b = 0.3
stop_stiffness_n_m = 1.0e6
stop_damping_n_s_m = 500.0
friction_coulomb_n = 10.0
friction_coulomb_n_bar = 2.0
friction_stiction_ratio = 1.5
friction_stribeck_m_s = 0.01
friction_viscous_n_s_m = 20.0
friction_smoothing_m_s = 0.002
load_force_n = 0.0
sensor_band_m = 0.002
sensor_delay_s = 0.005
start = "retracted"
rated_cushion_energy_j = 0.4
load_factor_limit = 0.7
compressor_energy_kwh_m3 = 0.12
"#;

/// The effect switches a file with cylinders has beside the shared ones.
pub const EFFECTS: [&str; 2] = ["cylinder_friction", "cylinder_cushion"];

/// One ISO 6358 flow element: sonic conductance C, m^3/(s Pa) (ANR), and critical pressure ratio b.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Flow {
    pub c: f64,
    pub b: f64,
}

/// The ISO 6358 flow function at the pressure ratio r = p_down / p_up (at most 1).
pub fn psi(r: f64, b: f64) -> f64 {
    let ellipse = |r: f64| (1.0 - ((r - b) / (1.0 - b)).powi(2)).max(0.0).sqrt();
    if r <= b {
        1.0
    } else if r < 1.0 - LINEAR {
        ellipse(r)
    } else {
        ellipse(1.0 - LINEAR) * (1.0 - r).max(0.0) / LINEAR
    }
}

impl Flow {
    pub fn new(c_dm3_s_bar: f64, b: f64) -> Self {
        Self { c: c_dm3_s_bar * DM3_S_BAR, b }
    }

    /// Elements one after the other (see the module's text).
    pub fn series(parts: &[Flow]) -> Flow {
        let c = parts.iter().map(|f| f.c.powi(-3)).sum::<f64>().powf(-1.0 / 3.0);
        let b = 1.0 - c * c * parts.iter().map(|f| (1.0 - f.b) / (f.c * f.c)).sum::<f64>();
        Flow { c, b: b.max(0.0) }
    }

    /// Mass flow from `p_up` at `t_up` down to `p_down`, kg/s; zero where `p_down` is not lower.
    pub fn mass_flow(&self, p_up: f64, t_up: f64, p_down: f64) -> f64 {
        if p_up <= p_down {
            return 0.0;
        }
        self.c * p_up * RHO0 * (T0 / t_up).sqrt() * psi(p_down / p_up, self.b)
    }

    /// The slope of the flow law's straight end, d psi / d r, for the stability estimate.
    fn end_slope(&self) -> f64 {
        psi(1.0 - LINEAR, self.b) / LINEAR
    }
}

/// The seal's friction: a Stribeck curve, smoothed through zero speed, whose levels rise with the pressure.
#[derive(Debug, Clone, Copy)]
struct Seal {
    coulomb: f64,
    per_pa: f64,
    stiction_ratio: f64,
    stribeck: f64,
    viscous: f64,
    smoothing: f64,
}

impl Seal {
    /// The force against the speed `v` with the chambers' mean gauge pressure `gauge`, N.
    fn force(&self, v: f64, gauge: f64) -> f64 {
        let level = self.coulomb + self.per_pa * gauge.max(0.0);
        let breakaway = level * (1.0 + (self.stiction_ratio - 1.0) * (-(v / self.stribeck).powi(2)).exp());
        breakaway * (v / self.smoothing).tanh() + self.viscous * v
    }
}

/// What the flows are at a state: into chamber 1 and 2 (kg/s), and the supply's and the exhaust's share.
#[derive(Debug, Clone, Copy, Default)]
pub struct Flows {
    pub into: [f64; 2],
    pub supply: f64,
    pub supply_enthalpy: f64,
    pub exhaust: f64,
    pub exhaust_enthalpy: f64,
}

/// What a cylinder keeps from step to step outside the state: the valve, the sensors and what it measured on its
/// strokes. Numbers only, so that it is the machine's memory as it is (`to_vec`).
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct Track {
    command: f64,
    spool: f64,
    /// The command the valve switches to and the time left until it does; below 0: none pending.
    pending: f64,
    pending_left: f64,
    sensor_time: [f64; 2],
    /// Since the spool last switched, s.
    since_switch: f64,
    arrived: f64,
    cushioned: f64,
    contact_time: f64,
    stroke_time: [f64; 2],
    impact_speed: f64,
    impact_energy: f64,
    end_speed: f64,
    bounce: f64,
    settling: f64,
    air_mark: f64,
    air_cycle: f64,
}

const TRACK: usize = 19;

impl Track {
    fn to_vec(self) -> [f64; TRACK] {
        let t = self;
        [
            t.command,
            t.spool,
            t.pending,
            t.pending_left,
            t.sensor_time[0],
            t.sensor_time[1],
            t.since_switch,
            t.arrived,
            t.cushioned,
            t.contact_time,
            t.stroke_time[0],
            t.stroke_time[1],
            t.impact_speed,
            t.impact_energy,
            t.end_speed,
            t.bounce,
            t.settling,
            t.air_mark,
            t.air_cycle,
        ]
    }
    fn from_slice(m: &[f64]) -> Self {
        Self {
            command: m[0],
            spool: m[1],
            pending: m[2],
            pending_left: m[3],
            sensor_time: [m[4], m[5]],
            since_switch: m[6],
            arrived: m[7],
            cushioned: m[8],
            contact_time: m[9],
            stroke_time: [m[10], m[11]],
            impact_speed: m[12],
            impact_energy: m[13],
            end_speed: m[14],
            bounce: m[15],
            settling: m[16],
            air_mark: m[17],
            air_cycle: m[18],
        }
    }
}

/// A double-acting cylinder with its valve, throttles and cushions (see the module's text).
#[derive(Debug, Clone)]
pub struct Cylinder {
    pub id: String,
    pub key: String,
    pub a1: f64,
    pub a2: f64,
    pub stroke: f64,
    pub mass: f64,
    pub payload: f64,
    payload_on: bool,
    /// Gravity's component against +x per kg, m/s^2 (g sin incline, 0 with effects.gravity off).
    pub gravity: f64,
    incline: f64,
    pub dead: [f64; 2],
    /// Supply and ambient pressure (absolute), Pa; ambient (and supply, and wall) temperature, K.
    pub supply: f64,
    pub ambient: f64,
    pub temperature: f64,
    pub heat_transfer: f64,
    pub closed_centre: bool,
    pub switch_time: f64,
    /// Into a chamber: valve and tube. Out of chamber 1 and 2: valve, tube and that chamber's throttle; and within
    /// the cushion: valve, tube and the bleed.
    pub inflow: Flow,
    pub outflow: [Flow; 2],
    pub bleed: [Flow; 2],
    /// Cushion length, m; 0 with effects.cylinder_cushion off.
    pub cushion: f64,
    stops: HardStop,
    seal: Option<Seal>,
    pub load: f64,
    sensor_band: f64,
    sensor_delay: f64,
    start_extended: bool,
    rated_cushion_energy: f64,
    load_factor_limit: f64,
    compressor_j_per_kg: f64,
    track: Track,
}

/// The cylinder's signals, after `cylinder.<id>.`, with their quantity keys (`quantity-cylinder-<name>`) and units.
pub const SIGNALS: [(&str, &str); 21] = [
    ("position", "m"),
    ("velocity", "m/s"),
    ("pressure_cap", "Pa"),
    ("pressure_rod", "Pa"),
    ("temperature_cap", "K"),
    ("temperature_rod", "K"),
    ("spool", ""),
    ("sensor_retracted", ""),
    ("sensor_extended", ""),
    ("supply_flow", "l/min"),
    ("air", "l"),
    ("air_cycle", "l"),
    ("compressor_energy", "J"),
    ("stroke_time_extend", "s"),
    ("stroke_time_retract", "s"),
    ("impact_speed", "m/s"),
    ("impact_energy", "J"),
    ("end_speed", "m/s"),
    ("bounce", "m"),
    ("settling_time", "s"),
    ("load_factor", ""),
];

fn text<'a>(cfg: &'a Config, path: &str) -> Result<&'a str, ConfigError> {
    cfg.get(path)?.as_str().ok_or_else(|| ConfigError::Type(path.into()))
}

impl Cylinder {
    /// The cylinder `id` from its table at `path`, integrated in sub-steps of `h` seconds: refused where that step
    /// would not be stable (`error-cylinder-unstable`).
    pub fn from_config(cfg: &Config, id: &str, key: &str, path: &str, h: f64) -> Result<Self, ConfigError> {
        let at = |k: &str| format!("{path}.{k}");
        let pos = |k: &str| cfg.positive(&at(k));
        let nonneg = |k: &str| cfg.non_negative(&at(k));
        let ratio = |k: &str| -> Result<f64, ConfigError> {
            let b = cfg.non_negative(&at(k))?;
            if b < 0.95 { Ok(b) } else { Err(ConfigError::Invalid(at(k))) }
        };
        let flow = |c: &str, b: &str| -> Result<Flow, ConfigError> { Ok(Flow::new(pos(c)?, ratio(b)?)) };
        let (d, rod) = (pos("piston_diameter_m")?, pos("rod_diameter_m")?);
        if rod >= d {
            return Err(ConfigError::Invalid(at("rod_diameter_m")));
        }
        let a1 = std::f64::consts::PI * d * d / 4.0;
        let a2 = a1 - std::f64::consts::PI * rod * rod / 4.0;
        let stroke = pos("stroke_m")?;
        let ambient = pos("ambient_pressure_pa")?;
        let supply = ambient + nonneg("supply_pressure_bar")? * 1e5;
        let incline = cfg.f64(&at("incline_deg"))?.to_radians();
        let gravity = if cfg.bool("effects.gravity")? { G * incline.sin() } else { 0.0 };
        let payload = if cfg.bool("effects.payload")? { nonneg("payload_kg")? } else { 0.0 };
        let closed_centre = match text(cfg, &at("valve"))? {
            "5/2" => false,
            "5/3" => true,
            _ => return Err(ConfigError::Invalid(at("valve"))),
        };
        let start_extended = match text(cfg, &at("start"))? {
            "retracted" => false,
            "extended" => true,
            _ => return Err(ConfigError::Invalid(at("start"))),
        };
        let (valve, tube) = (flow("valve_c_dm3_s_bar", "valve_b")?, flow("tube_c_dm3_s_bar", "tube_b")?);
        let throttle_b = ratio("throttle_b")?;
        // During the extend stroke the rod chamber exhausts: its throttle is the one that sets the extend speed.
        let throttles = [
            Flow::new(pos("throttle_retract_c_dm3_s_bar")?, throttle_b),
            Flow::new(pos("throttle_extend_c_dm3_s_bar")?, throttle_b),
        ];
        let bleed = flow("cushion_bleed_c_dm3_s_bar", "cushion_bleed_b")?;
        let cushion = if cfg.bool("effects.cylinder_cushion")? { nonneg("cushion_length_m")? } else { 0.0 };
        if cushion > 0.5 * stroke {
            return Err(ConfigError::Invalid(at("cushion_length_m")));
        }
        let stiction_ratio = cfg.f64(&at("friction_stiction_ratio"))?;
        if stiction_ratio < 1.0 {
            return Err(ConfigError::Invalid(at("friction_stiction_ratio")));
        }
        let seal = Seal {
            coulomb: nonneg("friction_coulomb_n")?,
            per_pa: nonneg("friction_coulomb_n_bar")? / 1e5,
            stiction_ratio,
            stribeck: pos("friction_stribeck_m_s")?,
            viscous: nonneg("friction_viscous_n_s_m")?,
            smoothing: pos("friction_smoothing_m_s")?,
        };
        let seal = cfg.bool("effects.cylinder_friction")?.then_some(seal);
        let temperature = pos("ambient_temperature_k")?;
        let cylinder = Self {
            id: id.to_string(),
            key: key.to_string(),
            a1,
            a2,
            stroke,
            mass: pos("moved_mass_kg")?,
            payload,
            payload_on: true,
            gravity,
            incline,
            dead: [pos("dead_volume_cap_m3")?, pos("dead_volume_rod_m3")?],
            supply,
            ambient,
            temperature,
            heat_transfer: nonneg("heat_transfer_w_k")?,
            closed_centre,
            switch_time: nonneg("valve_switch_time_s")?,
            inflow: Flow::series(&[valve, tube]),
            outflow: throttles.map(|t| Flow::series(&[valve, tube, t])),
            // The cushion's bleed screw sits behind the port's meter-out throttle, so that is in series too.
            bleed: throttles.map(|t| Flow::series(&[valve, tube, bleed, t])),
            cushion,
            stops: HardStop { stiffness: pos("stop_stiffness_n_m")?, damping: nonneg("stop_damping_n_s_m")? },
            seal,
            load: cfg.f64(&at("load_force_n"))?,
            sensor_band: pos("sensor_band_m")?,
            sensor_delay: nonneg("sensor_delay_s")?,
            start_extended,
            rated_cushion_energy: pos("rated_cushion_energy_j")?,
            load_factor_limit: pos("load_factor_limit")?,
            compressor_j_per_kg: nonneg("compressor_energy_kwh_m3")? * 3.6e6 / RHO0,
            track: Track::default(),
        };
        if let Some((what, rate)) = cylinder.stiffest().into_iter().find(|(_, rate)| rate * h > STABLE) {
            return Err(ConfigError::Part(
                "error-cylinder-unstable",
                format!("{path}: {what} {rate:.0} 1/s, sim.dt_s / sim.substeps at most {:.3e} s", STABLE / rate),
            ));
        }
        let mut cylinder = cylinder;
        let spool = cylinder.start_command();
        // At rest in the end it starts at: arrived there, nothing to measure until the valve switches.
        cylinder.track = Track {
            command: spool,
            spool,
            pending: spool,
            pending_left: -1.0,
            arrived: 1.0,
            cushioned: 1.0,
            ..Track::default()
        };
        Ok(cylinder)
    }

    fn start_command(&self) -> f64 {
        if self.start_extended { 1.0 } else { -1.0 }
    }

    fn moved(&self) -> f64 {
        self.mass + if self.payload_on { self.payload } else { 0.0 }
    }

    /// The stiffest rates of the model at its worst (smallest chamber, highest pressure), 1/s: the flow into the
    /// smallest dead volume where the law is a straight line, the air spring of it, the stops, the seal's smoothing
    /// and the heat transfer. What each is, for the message.
    fn stiffest(&self) -> Vec<(&'static str, f64)> {
        let small = self.dead[0].min(self.dead[1]);
        let widest = self.inflow.c.max(self.outflow[0].c).max(self.outflow[1].c);
        let slope = [self.inflow, self.outflow[0], self.outflow[1]].iter().map(Flow::end_slope).fold(0.0, f64::max);
        let m = self.mass;
        let mut out = vec![
            ("flow", KAPPA * R * self.temperature * RHO0 * widest * slope / small),
            (
                "air spring",
                (KAPPA * self.supply * (self.a1 * self.a1 / self.dead[0]).max(self.a2 * self.a2 / self.dead[1]) / m)
                    .sqrt(),
            ),
            ("end stop", (self.stops.stiffness / m).sqrt().max(self.stops.damping / m)),
            ("heat transfer", self.heat_transfer * (KAPPA - 1.0) * self.temperature / (self.ambient * small)),
        ];
        if let Some(s) = self.seal {
            let level = (s.coulomb + s.per_pa * (self.supply - self.ambient)) * s.stiction_ratio;
            out.push(("seal friction", (level / s.smoothing + s.viscous) / m));
        }
        out
    }

    pub fn volumes(&self, x: f64) -> [f64; 2] {
        [self.dead[0] + self.a1 * x, self.dead[1] + self.a2 * (self.stroke - x)]
    }

    /// Pressure and temperature of both chambers at state `s` (this cylinder's).
    pub fn gas(&self, s: &[f64]) -> [[f64; 2]; 2] {
        let v = self.volumes(s[X]);
        let p = [(KAPPA - 1.0) * s[U1] / v[0], (KAPPA - 1.0) * s[U2] / v[1]];
        [p, [p[0] * v[0] / (s[M1] * R), p[1] * v[1] / (s[M2] * R)]]
    }

    /// The net force on the piston but the seal's and the stops', N, towards +x (extending).
    fn pressure_force(&self, p: [f64; 2]) -> f64 {
        p[0] * self.a1 - p[1] * self.a2 - self.ambient * (self.a1 - self.a2) - self.load - self.moved() * self.gravity
    }

    fn seal_force(&self, v: f64, p: [f64; 2]) -> f64 {
        self.seal.map_or(0.0, |s| s.force(v, 0.5 * (p[0] + p[1]) - self.ambient))
    }

    /// The flows at state `s` with the spool at `spool` (+1, -1, 0).
    pub fn flows(&self, s: &[f64], spool: f64) -> Flows {
        let [p, t] = self.gas(s);
        let x = s[X];
        let mut f = Flows::default();
        for i in 0..2 {
            // Chamber 1 is on the supply while extending, chamber 2 while retracting.
            let on_supply = if i == 0 { spool > 0.5 } else { spool < -0.5 };
            if spool.abs() < 0.5 {
                continue;
            }
            let (port, port_t) =
                if on_supply { (self.supply, self.temperature) } else { (self.ambient, self.temperature) };
            let mdot = if port > p[i] {
                self.inflow.mass_flow(port, port_t, p[i])
            } else {
                // Within the cushion length of this chamber's end the spear closes the main bore: the bleed alone, blended
                // in over the first millimetre so that the rates stay continuous.
                let to_end = if i == 0 { x } else { self.stroke - x };
                let ramp = (0.001f64).min(self.cushion);
                let w = if self.cushion > 0.0 { ((self.cushion - to_end) / ramp).clamp(0.0, 1.0) } else { 0.0 };
                let out = (1.0 - w) * self.outflow[i].mass_flow(p[i], t[i], port)
                    + w * self.bleed[i].mass_flow(p[i], t[i], port);
                -out
            };
            let enthalpy = CP * mdot * if mdot > 0.0 { port_t } else { t[i] };
            f.into[i] = mdot;
            if on_supply {
                f.supply += mdot;
                f.supply_enthalpy += enthalpy;
            } else {
                f.exhaust -= mdot;
                f.exhaust_enthalpy -= enthalpy;
            }
        }
        f
    }

    /// The rates of this cylinder's states at `s` (`STATES` of each).
    pub fn derivatives(&self, s: &[f64], out: &mut [f64]) {
        self.derivatives_with(s, self.track.spool, out)
    }

    pub fn derivatives_with(&self, s: &[f64], spool: f64, out: &mut [f64]) {
        let [p, t] = self.gas(s);
        let (x, v) = (s[X], s[V]);
        let f = self.flows(s, spool);
        let dv = [self.a1 * v, -self.a2 * v];
        let heat = [0, 1].map(|i| self.heat_transfer * (self.temperature - t[i]));
        let enthalpy = [0, 1].map(|i| CP * f.into[i] * if f.into[i] > 0.0 { self.temperature } else { t[i] });
        let seal = self.seal_force(v, p);
        let stop = self.stops.force(x, v, self.stroke);
        out[X] = v;
        out[V] = (self.pressure_force(p) - seal + stop) / self.moved();
        out[M1] = f.into[0];
        out[U1] = enthalpy[0] + heat[0] - p[0] * dv[0];
        out[M2] = f.into[1];
        out[U2] = enthalpy[1] + heat[1] - p[1] * dv[1];
        out[AIR] = f.supply;
        out[H_IN] = f.supply_enthalpy;
        out[H_OUT] = f.exhaust_enthalpy;
        out[HEAT] = heat[0] + heat[1];
        out[LOSS] = seal * v + self.stops.loss_power(x, v, self.stroke);
    }

    /// At rest in the end the valve holds it at: the driven chamber at the supply pressure, the other at ambient, the
    /// piston pressed into its stop as far as the net force deflects it, the gas at ambient temperature. Appended to
    /// `state`.
    pub fn rest(&self, spool: f64, state: &mut Vec<f64>) {
        let extended = spool > 0.0;
        let p = if extended { [self.supply, self.ambient] } else { [self.ambient, self.supply] };
        let force = self.pressure_force(p);
        let x = if extended {
            self.stroke + force.max(0.0) / self.stops.stiffness
        } else {
            force.min(0.0) / self.stops.stiffness
        };
        let v = self.volumes(x);
        let mut s = [0.0; STATES];
        s[X] = x;
        s[M1] = p[0] * v[0] / (R * self.temperature);
        s[U1] = p[0] * v[0] / (KAPPA - 1.0);
        s[M2] = p[1] * v[1] / (R * self.temperature);
        s[U2] = p[1] * v[1] / (KAPPA - 1.0);
        state.extend(s);
    }

    /// Mechanical energy (kinetic, gravity, the load's and the atmosphere's work on the rod, the stops' springs), J.
    pub fn mechanical_energy(&self, s: &[f64]) -> f64 {
        let (x, v) = (s[X], s[V]);
        0.5 * self.moved() * v * v
            + (self.moved() * self.gravity + self.load + self.ambient * (self.a1 - self.a2)) * x
            + self.stops.energy(x, self.stroke)
    }

    /// The gas's internal energy in both chambers, J.
    pub fn gas_energy(&self, s: &[f64]) -> f64 {
        s[U1] + s[U2]
    }

    /// The load over the theoretical force at the supply's gauge pressure, the worse of the two strokes: what the
    /// cylinder has to push against gravity and the load, over p_g A1 extending and p_g A2 retracting.
    pub fn load_factor(&self) -> f64 {
        let gauge = self.supply - self.ambient;
        let against = self.load + (self.mass + self.payload) * self.gravity;
        (against.max(0.0) / (gauge * self.a1)).max((-against).max(0.0) / (gauge * self.a2))
    }

    /// The command of the valve, from the cycle: switches after `switch_time` where it differs from where the valve
    /// goes. A 5/2 valve has no centre: 0 keeps it where it is.
    pub fn command(&mut self, command: f64) {
        let command = if !self.closed_centre && command.abs() < 0.5 { self.track.command } else { command.signum() };
        let going = if self.track.pending_left < 0.0 { self.track.spool } else { self.track.pending };
        self.track.command = command;
        if command != going {
            (self.track.pending, self.track.pending_left) = (command, self.switch_time);
        }
    }

    /// The step of `dt` done: the valve's delay, the sensors' delay, and what the stroke measured.
    fn stepped(&mut self, s: &[f64], dt: f64) {
        let t = &mut self.track;
        t.since_switch += dt;
        if t.pending_left >= 0.0 {
            t.pending_left -= dt;
            if t.pending_left <= 1e-12 {
                t.spool = t.pending;
                t.pending_left = -1.0;
                (t.since_switch, t.arrived, t.cushioned, t.contact_time) = (0.0, 0.0, 0.0, -1.0);
                (t.bounce, t.settling) = (0.0, 0.0);
                if t.spool > 0.5 {
                    t.air_cycle = s[AIR] / RHO0 * 1e3 - t.air_mark;
                    t.air_mark = s[AIR] / RHO0 * 1e3;
                }
            }
        }
        let (x, v) = (s[X], s[V]);
        for (k, at_end) in [x <= self.sensor_band, x >= self.stroke - self.sensor_band].into_iter().enumerate() {
            t.sensor_time[k] = if at_end { t.sensor_time[k] + dt } else { 0.0 };
        }
        // Measured on the stroke under way: toward the end the spool drives to.
        let extending = t.spool > 0.5;
        if t.spool.abs() < 0.5 {
            return;
        }
        let to_end = if extending { self.stroke - x } else { x };
        let moved = self.mass + if self.payload_on { self.payload } else { 0.0 };
        if t.cushioned == 0.0 && self.cushion > 0.0 && to_end <= self.cushion {
            t.cushioned = 1.0;
            // The kinetic energy alone, the vendor's rule for the cylinder's own cushioning (Festo DSBC data sheet: v = sqrt(2 E /
            // (m1 + m2))); the weight's work over the cushion length belongs to the rule for external shock absorbers.
            (t.impact_speed, t.impact_energy) = (v.abs(), 0.5 * moved * v * v);
        }
        if t.arrived == 0.0 && to_end <= self.sensor_band {
            t.arrived = 1.0;
            t.stroke_time[extending as usize] = t.since_switch;
        }
        if t.contact_time < 0.0 && to_end <= 0.0 {
            t.contact_time = t.since_switch;
            t.end_speed = v.abs();
            if self.cushion == 0.0 {
                (t.impact_speed, t.impact_energy) = (v.abs(), 0.5 * moved * v * v);
            }
        }
        if t.contact_time >= 0.0 {
            t.bounce = t.bounce.max(to_end);
            if to_end > self.sensor_band {
                t.settling = t.since_switch - t.contact_time;
            }
        }
    }

    fn sensors(&self) -> [bool; 2] {
        self.track.sensor_time.map(|time| time > 0.0 && time >= self.sensor_delay - 1e-12)
    }

    fn signals(&self, s: &[f64], out: &mut Vec<f64>) {
        let [p, t] = self.gas(s);
        let f = self.flows(s, self.track.spool);
        let tr = &self.track;
        let sensors = self.sensors();
        let air = s[AIR] / RHO0 * 1e3;
        out.extend([
            s[X],
            s[V],
            p[0],
            p[1],
            t[0],
            t[1],
            tr.spool,
            sensors[0] as u8 as f64,
            sensors[1] as u8 as f64,
            f.supply / RHO0 * 6e4,
            air,
            tr.air_cycle,
            s[AIR] * self.compressor_j_per_kg,
            tr.stroke_time[1],
            tr.stroke_time[0],
            tr.impact_speed,
            tr.impact_energy,
            tr.end_speed,
            tr.bounce,
            tr.settling,
            self.load_factor(),
        ]);
    }
}

/// A machine with pneumatic cylinders beside its axes. Its state is the machine's, then `STATES` per cylinder.
pub struct Pneumatic {
    inner: Box<dyn Machine>,
    pub cylinders: Vec<Cylinder>,
    inner_base_signals: usize,
}

/// The cylinder nodes of a machine file: id, translation key and the path of its table.
pub struct CylinderNode {
    pub id: String,
    pub key: String,
    pub values: String,
}

/// The machine with the cylinders `nodes` attached, as it is without any. `carrying`: the axes that carry the tool,
/// where the cylinders sit, each with its direction (a unit vector) or None for a rotary axis.
pub fn attach(
    cfg: &Config,
    nodes: &[CylinderNode],
    carrying: &[(String, Option<[f64; 3]>)],
    machine: Box<dyn Machine>,
) -> Result<Box<dyn Machine>, ConfigError> {
    if nodes.is_empty() {
        return Ok(machine);
    }
    let h = cfg.positive("sim.dt_s")? / cfg.positive("sim.substeps")?;
    let cylinders =
        nodes.iter().map(|n| Cylinder::from_config(cfg, &n.id, &n.key, &n.values, h)).collect::<Result<Vec<_>, _>>()?;
    for (c, n) in cylinders.iter().zip(nodes) {
        if let Some(axis) = along_carrying_axis(c.incline, carrying) {
            return Err(ConfigError::Invalid(format!(
                "{}.incline_deg (the cylinder {} pushes on the machine's axis {axis}, which carries it; the model \
                 lets the cylinder ride without acting back and holds only at right angles to the linear axes that \
                 carry it and on none that turns it)",
                n.values, n.id
            )));
        }
    }
    let pose = crate::sim::start_pose(cfg, machine.as_ref())?;
    let (mut state, mut torque, mut signals) = (Vec::new(), vec![0.0; machine.motors().len()], Vec::new());
    machine.equilibrium(&pose, &mut state, &mut torque);
    machine.base_signals(&state, &torque, &mut signals);
    Ok(Box::new(Pneumatic { inner_base_signals: signals.len(), inner: machine, cylinders }))
}

/// The id of a carrying axis that a cylinder extending at `incline` (rad, above x in the x-z plane) pushes on: a
/// linear one whose direction has more than rounding (1e-9) of the cylinder's, or a rotary one (it would turn the
/// cylinder, whose incline is fixed, and the push has a moment about it). None where it pushes on none.
fn along_carrying_axis(incline: f64, carrying: &[(String, Option<[f64; 3]>)]) -> Option<&str> {
    let (sin, cos) = incline.sin_cos();
    carrying.iter().find(|(_, d)| d.is_none_or(|d| (cos * d[0] + sin * d[2]).abs() > 1e-9)).map(|(id, _)| id.as_str())
}

impl Pneumatic {
    fn n(&self) -> usize {
        self.inner.state_len()
    }
    fn of<'a>(&self, state: &'a [f64], k: usize) -> &'a [f64] {
        let at = self.n() + STATES * k;
        &state[at..at + STATES]
    }
}

impl Machine for Pneumatic {
    fn frame_momentum(&self, state: &[f64]) -> Option<sim_core::FrameMomentum> {
        self.inner.frame_momentum(&state[..self.n()])
    }
    fn base_shift(&self, tool_pose: &[f64], pivot: [f64; 3], shift: &[f64; 6], out: &mut [f64]) {
        self.inner.base_shift(tool_pose, pivot, shift, out)
    }
    fn twist(&self, torque: &[f64], acc: &[f64], out: &mut [f64]) {
        self.inner.twist(torque, acc, out)
    }
    fn twist_at(&self, angle: &[f64], torque: &[f64], acc: &[f64], out: &mut [f64]) {
        self.inner.twist_at(angle, torque, acc, out)
    }
    fn rigid_split_torque(&self, angle: &[f64], speed: &[f64], acc: &[f64], rotor_acc: &[f64], out: &mut [f64]) {
        self.inner.rigid_split_torque(angle, speed, acc, rotor_acc, out)
    }
    fn links(&self) -> Vec<String> {
        self.inner.links()
    }
    fn set_link_change(&mut self, link: usize, factor: f64, moment: [f64; 3]) {
        self.inner.set_link_change(link, factor, moment)
    }
    fn cam(&self) -> Option<sim_core::CamMaster> {
        self.inner.cam()
    }
    fn cam_at(&self, master: f64, out: &mut [[f64; 4]]) {
        self.inner.cam_at(master, out)
    }
    fn cam_curve(&self) -> Option<sim_core::CamCurve> {
        self.inner.cam_curve()
    }
    fn gantry_motors(&self) -> Option<[usize; 2]> {
        self.inner.gantry_motors()
    }
    fn collision_capsules(&self, state: &[f64]) -> Vec<sim_core::Capsule> {
        self.inner.collision_capsules(&state[..self.n()])
    }
    fn rating_checks(&self) -> Vec<RatingCheck> {
        let mut checks = self.inner.rating_checks();
        for c in &self.cylinders {
            let check = |check, signal: &str, limit| RatingCheck {
                check,
                signal: format!("cylinder.{}.{signal}", c.id),
                motor: None,
                limit,
                life: false,
            };
            checks.push(check("cushion-energy", "impact_energy", c.rated_cushion_energy));
            checks.push(check("load-factor", "load_factor", c.load_factor_limit));
        }
        checks
    }
    fn axes(&self) -> &[AxisInfo] {
        self.inner.axes()
    }
    fn motors(&self) -> &[MotorInfo] {
        self.inner.motors()
    }
    fn inverse(&self, tool: &[f64], motors: &mut [f64]) {
        self.inner.inverse(tool, motors)
    }
    fn forward(&self, motors: &[f64], tool: &mut [f64]) {
        self.inner.forward(motors, tool)
    }
    fn state_len(&self) -> usize {
        self.n() + STATES * self.cylinders.len()
    }
    fn equilibrium(&self, tool: &[f64], state: &mut Vec<f64>, torque: &mut [f64]) {
        self.inner.equilibrium(tool, state, torque);
        for c in &self.cylinders {
            c.rest(c.track.spool, state);
        }
    }
    fn derivatives(&self, state: &[f64], torque: &[f64], out: &mut [f64]) {
        let n = self.n();
        self.inner.derivatives(&state[..n], torque, &mut out[..n]);
        for (k, c) in self.cylinders.iter().enumerate() {
            let at = n + STATES * k;
            c.derivatives(&state[at..at + STATES], &mut out[at..at + STATES]);
        }
    }
    fn tool_state(&self, state: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        self.inner.tool_state(&state[..self.n()], pos, vel)
    }
    fn set_tool_velocity(&self, state: &mut [f64], vel: &[f64]) {
        let n = self.n();
        self.inner.set_tool_velocity(&mut state[..n], vel)
    }
    fn motor_state(&self, state: &[f64], pos: &mut [f64], vel: &mut [f64]) {
        self.inner.motor_state(&state[..self.n()], pos, vel)
    }
    fn feedforward_torque(&self, tool_pos: &[f64], tool_acc: &[f64], out: &mut [f64]) {
        self.inner.feedforward_torque(tool_pos, tool_acc, out)
    }
    fn feedforward_torque_moving(&self, tool_pos: &[f64], tool_vel: &[f64], tool_acc: &[f64], out: &mut [f64]) {
        self.inner.feedforward_torque_moving(tool_pos, tool_vel, tool_acc, out)
    }
    fn rigid_motor_torque(&self, angle: &[f64], speed: &[f64], acc: &[f64], out: &mut [f64]) {
        self.inner.rigid_motor_torque(angle, speed, acc, out)
    }
    fn rigid_tensions(&self, tool_pos: &[f64], tool_acc: &[f64], out: &mut Vec<f64>) {
        self.inner.rigid_tensions(tool_pos, tool_acc, out)
    }
    fn rigid_tensions_moving(&self, tool_pos: &[f64], tool_vel: &[f64], tool_acc: &[f64], out: &mut Vec<f64>) {
        self.inner.rigid_tensions_moving(tool_pos, tool_vel, tool_acc, out)
    }
    fn tension_band(&self) -> Option<[f64; 2]> {
        self.inner.tension_band()
    }
    fn rigid_guide_loads(&self, tool_pos: &[f64], tool_acc: &[f64], out: &mut Vec<[f64; 2]>) {
        self.inner.rigid_guide_loads(tool_pos, tool_acc, out)
    }
    fn rigid_friction(&self, tool_pos: &[f64], tool_dir: &[f64], out: &mut Vec<[f64; 2]>) {
        self.inner.rigid_friction(tool_pos, tool_dir, out)
    }
    fn rigid_friction_rise(&self, tool_pos: &[f64], tool_dir: &[f64], out: &mut Vec<Vec<[f64; 2]>>) {
        self.inner.rigid_friction_rise(tool_pos, tool_dir, out)
    }
    /// The machine's alone: a cylinder is an open system, fed by the supply, and keeps its own balance (the signals
    /// `cylinder.<id>.*` and `Cylinder::mechanical_energy`, `gas_energy`).
    fn energy(&self, state: &[f64]) -> f64 {
        self.inner.energy(&state[..self.n()])
    }
    fn set_payload_attached(&mut self, attached: bool) {
        self.inner.set_payload_attached(attached);
        self.cylinders.iter_mut().for_each(|c| c.payload_on = attached);
    }
    fn set_dwelling(&mut self, dwelling: bool) {
        self.inner.set_dwelling(dwelling)
    }
    fn set_brake_engaged(&mut self, engaged: bool) {
        self.inner.set_brake_engaged(engaged)
    }
    fn valves(&self) -> Vec<(String, f64)> {
        self.cylinders.iter().map(|c| (c.id.clone(), c.track.command)).collect()
    }
    fn set_valve(&mut self, valve: usize, command: f64) {
        if let Some(c) = self.cylinders.get_mut(valve) {
            c.command(command);
        }
    }
    fn end_sensors(&self, valve: usize) -> [bool; 2] {
        self.cylinders.get(valve).map_or([false; 2], Cylinder::sensors)
    }
    fn after_step(&mut self, state: &mut [f64], dt: f64) {
        let n = self.n();
        self.inner.after_step(&mut state[..n], dt);
        for (k, c) in self.cylinders.iter_mut().enumerate() {
            let at = n + STATES * k;
            c.stepped(&state[at..at + STATES], dt);
        }
    }
    fn memory(&self) -> Vec<f64> {
        let mut m = self.inner.memory();
        self.cylinders.iter().for_each(|c| m.extend(c.track.to_vec()));
        m
    }
    fn set_memory(&mut self, memory: &[f64]) {
        let own = TRACK * self.cylinders.len();
        let (inner, own) = memory.split_at(memory.len() - own);
        self.inner.set_memory(inner);
        for (c, m) in self.cylinders.iter_mut().zip(own.chunks(TRACK)) {
            c.track = Track::from_slice(m);
        }
    }
    fn set_part_temperatures(&mut self, rises: &[f64]) {
        self.inner.set_part_temperatures(rises)
    }
    fn resonance_hz(&self, tool_pose: &[f64]) -> Option<f64> {
        self.inner.resonance_hz(tool_pose)
    }
    fn mode_shape(&self, tool_pose: &[f64]) -> Option<(f64, Vec<f64>)> {
        let len = self.state_len();
        self.inner.mode_shape(tool_pose).map(|(hz, mut shape)| {
            shape.resize(len, 0.0);
            (hz, shape)
        })
    }
    fn modes(&self, tool_pose: &[f64]) -> Vec<sim_core::Mode> {
        self.inner.modes(tool_pose)
    }
    fn sway_modes(&self, tool_pose: &[f64]) -> Vec<sim_core::Mode> {
        self.inner.sway_modes(tool_pose)
    }
    fn trains(&self, tool_pose: &[f64]) -> Vec<Option<sim_core::Train>> {
        self.inner.trains(tool_pose)
    }
    fn belt_wraps(&self, tool_pose: &[f64]) -> Vec<f64> {
        self.inner.belt_wraps(tool_pose)
    }
    fn unreachable(&self, tool_pose: &[f64], slack: f64) -> Option<&'static str> {
        self.inner.unreachable(tool_pose, slack)
    }
    fn condition(&self, tool_pose: &[f64]) -> Option<f64> {
        self.inner.condition(tool_pose)
    }
    fn setpoint_slack(&self) -> f64 {
        self.inner.setpoint_slack()
    }
    fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut infos = self.inner.signal_infos();
        let own = self.cylinders.iter().flat_map(|c| {
            SIGNALS.iter().map(move |(name, unit)| {
                SignalInfo::new(
                    format!("cylinder.{}.{name}", c.id),
                    &c.key,
                    &format!("quantity-cylinder-{}", name.replace('_', "-")),
                    unit,
                )
            })
        });
        infos.splice(self.inner_base_signals..self.inner_base_signals, own.collect::<Vec<_>>());
        infos
    }
    fn base_signals(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.inner.base_signals(&state[..self.n()], torque, out);
        for (k, c) in self.cylinders.iter().enumerate() {
            c.signals(self.of(state, k), out);
        }
    }
    fn effect_signals(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.inner.effect_signals(&state[..self.n()], torque, out)
    }
    fn loss_powers(&self, state: &[f64], torque: &[f64], out: &mut Vec<f64>) {
        self.inner.loss_powers(&state[..self.n()], torque, out)
    }
    /// The machine's scene, and each cylinder at the tool: its body and its rod along its stroke (along x tilted by its
    /// incline, in the x-z plane), the rod out by the piston's position.
    fn scene(&self, state: &[f64], out: &mut Scene) {
        let n = self.n();
        self.inner.scene(&state[..n], out);
        let axes = self.inner.axes().len();
        let (mut pose, mut vel) = (vec![0.0; axes], vec![0.0; axes]);
        self.inner.tool_state(&state[..n], &mut pose, &mut vel);
        let Some(tool) = self.inner.scene_point(&pose) else { return };
        for (k, c) in self.cylinders.iter().enumerate() {
            let (dir, x) = ([c.incline.cos(), 0.0, c.incline.sin()], self.of(state, k)[X]);
            let at = |along: f64| std::array::from_fn(|i| tool[i] + along * dir[i]);
            let size = |length: f64, thick: f64| {
                let s: [f64; 3] = std::array::from_fn(|i| (length * dir[i]).abs().max(thick));
                sim_core::scene::Shape::Box { size: s }
            };
            let width = 2.0 * (c.a1 / std::f64::consts::PI).sqrt() + 0.02;
            out.parts.push(sim_core::scene::Part::new(
                "cylinder-body",
                size(c.stroke + 0.05, width),
                at(-0.5 * (c.stroke + 0.05)),
                [0.6, 0.65, 0.7],
            ));
            out.parts.push(sim_core::scene::Part::new(
                "cylinder-rod",
                size(x.max(0.0) + 0.01, 0.4 * width),
                at(0.5 * x.max(0.0)),
                [0.85, 0.85, 0.85],
            ));
        }
    }
    fn scene_point(&self, tool_pose: &[f64]) -> Option<[f64; 3]> {
        self.inner.scene_point(tool_pose)
    }
    fn collision_bodies(&self, state: &[f64]) -> Vec<sim_core::Aabb> {
        self.inner.collision_bodies(&state[..self.n()])
    }
    fn friction_contacts(&self) -> Vec<FrictionContact> {
        self.inner.friction_contacts()
    }
    fn friction_curve(&self, i: usize, speed: f64, load_scale: f64) -> [f64; 3] {
        self.inner.friction_curve(i, speed, load_scale)
    }
    fn friction_rotary(&self, i: usize) -> bool {
        self.inner.friction_rotary(i)
    }
    fn friction_speed_limit(&self, i: usize, motor_speed: &[f64]) -> Option<f64> {
        self.inner.friction_speed_limit(i, motor_speed)
    }
    fn belt_sections(&self) -> Vec<BeltSection> {
        self.inner.belt_sections()
    }
    fn thermal_parts(&self) -> Vec<ThermalPart> {
        self.inner.thermal_parts()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sim_core::Rk4;

    /// A cylinder hanging down (-90 degrees) from a carriage on a horizontal x pushes at right angles to it (cos =
    /// 6e-17), and to a horizontal y: carried. At 0 degrees it pushes along x, at 45 degrees partly along it; -90 pushes
    /// along a vertical z, and along an x inclined by 30 degrees (sin 30 of it); any rotary axis is refused.
    #[test]
    fn a_cylinder_along_a_carrying_axis_is_found() {
        let axis = |id: &str, d: Option<[f64; 3]>| (id.to_string(), d);
        let (x, y, z) =
            (axis("x", Some([1.0, 0.0, 0.0])), axis("y", Some([0.0, 1.0, 0.0])), axis("z", Some([0.0, 0.0, 1.0])));
        let down = -std::f64::consts::FRAC_PI_2;
        assert_eq!(along_carrying_axis(down, &[x.clone(), y.clone()]), None);
        assert_eq!(along_carrying_axis(0.0, &[y.clone(), x.clone()]), Some("x"));
        assert_eq!(along_carrying_axis(0.25 * std::f64::consts::PI, std::slice::from_ref(&x)), Some("x"));
        assert_eq!(along_carrying_axis(down, &[x, y.clone(), z]), Some("z"));
        let tilted = axis("x", Some([30f64.to_radians().cos(), 0.0, 30f64.to_radians().sin()]));
        assert_eq!(along_carrying_axis(down, &[tilted]), Some("x"));
        assert_eq!(along_carrying_axis(down, &[y, axis("c", None)]), Some("c"));
    }

    /// A DSBC-32-like cylinder from `TABLE` (32 mm bore, 12 mm rod: A1 = 8.0425e-4 m2, A2 = 6.9115e-4 m2, 100 mm
    /// stroke, 0.5 kg, dead volumes 5 cm3, 6 bar gauge), horizontal, without friction and cushions unless `changes` say.
    fn cylinder(changes: &[&str], h: f64) -> Cylinder {
        Cylinder::from_config(&config(false, changes), "z", "cylinder", "c", h).unwrap()
    }

    /// The config of `cylinder`, with seal friction and cushions on or off.
    fn config(on: bool, changes: &[&str]) -> Config {
        let mut cfg = Config::parse(&format!(
            "format_version = 1\n[effects]\ngravity = true\npayload = true\ncylinder_friction = {on}\ncylinder_cushion = {on}\n[c]\n{TABLE}"
        ))
        .unwrap();
        for change in changes {
            cfg.set_text(change).unwrap();
        }
        cfg
    }

    /// Integrates the cylinder alone with the spool at `spool` for `seconds` in steps of `h`, calling `each` after every
    /// step with the time and the state.
    fn run(c: &Cylinder, s: &mut [f64], spool: f64, h: f64, seconds: f64, mut each: impl FnMut(f64, &[f64])) {
        let mut rk = Rk4::new(STATES);
        for k in 0..(seconds / h).round() as usize {
            rk.step(|y, dy| c.derivatives_with(y, spool, dy), s, h);
            each((k + 1) as f64 * h, s);
        }
    }

    fn rest(c: &Cylinder, spool: f64) -> Vec<f64> {
        let mut s = Vec::new();
        c.rest(spool, &mut s);
        s
    }

    fn near(a: f64, b: f64, tol: f64, what: &str) {
        assert!((a - b).abs() <= tol * b.abs(), "{what}: {a} against {b} (tolerance {tol})");
    }

    /// The theoretical force p A at 6 bar gauge: extending 6e5 Pa * pi 0.016^2 = 482.5 N, retracting 6e5 Pa *
    /// (8.0425e-4 - 1.1310e-4) m2 = 414.7 N; Festo prints 483 N and 415 N for the DSBC-32 (catalog/festo/cylinders_dsbc.csv).
    #[test]
    fn the_theoretical_force_is_p_a_on_both_sides() {
        let c = cylinder(&[], 1e-5);
        let (ps, pa) = (c.supply, c.ambient);
        near(c.pressure_force([ps, pa]), 482.549, 1e-5, "extending");
        near(-c.pressure_force([pa, ps]), 414.690, 1e-5, "retracting");
        near(c.a1, 8.04248e-4, 1e-5, "A1");
        near(c.a2, 6.91150e-4, 1e-5, "A2");
    }

    /// ISO 6358 by hand: C = 1 dm3/(s bar) = 1e-8 m3/(s Pa) from 7.01325 bar into the atmosphere (ratio 0.144 < b = 0.3,
    /// choked): 1e-8 * 7.01325e5 * 1.185 = 8.3107e-3 kg/s at 293.15 K; at a ratio of 0.65, psi = sqrt(1 - (0.35 /
    /// 0.7)^2) = 0.86603 of it; at 0.9975 (the straight end) psi(0.995) / 2.
    #[test]
    fn the_flow_through_a_conductance_is_choked_below_b_and_elliptic_above() {
        let f = Flow::new(1.0, 0.3);
        near(f.mass_flow(7.01325e5, T0, 1.01325e5), 8.31070e-3, 1e-5, "choked");
        near(f.mass_flow(1e6, T0, 6.5e5) / f.mass_flow(1e6, T0, 1e5), 0.866025, 1e-6, "subsonic");
        near(psi(0.9975, 0.3), 0.5 * psi(0.995, 0.3), 1e-12, "straight end");
        assert_eq!(f.mass_flow(1e5, T0, 2e5), 0.0);
        // Series: two equal elements C = (2 C^-3)^(-1/3) = 0.7937 C, b = 1 - 0.63 * (0.7 + 0.7) = 0.1181.
        let s = Flow::series(&[f, f]);
        near(s.c / f.c, 0.793701, 1e-6, "series C");
        near(s.b, 1.0 - 0.793701f64.powi(2) * 1.4, 1e-5, "series b");
    }

    /// The meter-out throttled steady speed by hand: with the rod side's throttle choked, the mass that leaves it,
    /// C p2 rho0 sqrt(T0 / T2), is the mass the shrinking chamber pushes out, p2 A2 v / (R T2), so
    /// v = C rho0 R sqrt(T0 T2) / A2, whatever p2: at T2 = T0, 1e-9 * 1.185 * 287.058 * 293.15 / 6.9115e-4 = 0.14428
    /// m/s for C = 0.1 dm3/(s bar). Here the throttle is in series with a valve and a tube of 5 (C 0.1 times
    /// (1 + 2 0.1^3 / 5^3)^(-1/3), 5e-6 less) and works against 50 N: p2 = (p1 A1 - p_atm (A1 - A2) - 50) / A2 = 7.27 bar, the
    /// ratio 0.14 to the atmosphere, choked. The heat transfer (20 W/K) holds the rod side near T0. Measured over the
    /// middle of a 300 mm stroke, within 1 %.
    #[test]
    fn a_meter_out_throttled_cylinder_runs_at_the_choked_flow_s_speed() {
        let h = 1e-5;
        let c = cylinder(
            &[
                "c.stroke_m = 0.3",
                "c.valve_c_dm3_s_bar = 5.0",
                "c.tube_c_dm3_s_bar = 5.0",
                "c.throttle_extend_c_dm3_s_bar = 0.1",
                "c.load_force_n = 50.0",
                "c.heat_transfer_w_k = 20.0",
            ],
            h,
        );
        let mut s = rest(&c, -1.0);
        let (mut at, mut speeds) = (Vec::new(), Vec::new());
        run(&c, &mut s, 1.0, h, 3.0, |_, s| {
            if (0.1..0.2).contains(&s[X]) {
                at.push(s[X]);
                speeds.push(s[V]);
            }
        });
        assert!(!speeds.is_empty(), "the piston never got to the middle");
        let mean = speeds.iter().sum::<f64>() / speeds.len() as f64;
        let expected = c.outflow[1].c * RHO0 * R * T0 / c.a2;
        near(expected, 0.144279, 1e-4, "hand speed");
        near(mean, expected, 0.01, "steady speed");
    }

    /// The piston on its two air springs, both chambers closed (a 5/3 valve's centre) at mid-stroke: the stiffness
    /// n (p1 A1^2 / V1 + p2 A2^2 / V2), with p2 = 5 bar and p1 = (p2 A2 + p_atm (A1 - A2)) / A1 = 4.4394 bar balancing it,
    /// V1 = 5e-6 + A1 0.05 = 4.5212e-5 m3, V2 = 3.9558e-5 m3: 12389 N/m times n. Adiabatic (no heat transfer) n = 1.4,
    /// sqrt(1.4 * 12389 / 0.5) / 2 pi = 29.64 Hz; with a heat transfer quick against the period (500 W/K: a time
    /// constant of 0.34 ms) isothermal, n = 1, 25.05 Hz. Measured from the zero crossings of the speed over 1 s.
    #[test]
    fn the_air_springs_ring_at_the_hand_frequency_adiabatic_and_isothermal() {
        let h = 1e-5;
        for (heat, n, hz) in [(0.0, KAPPA, 29.64), (500.0, 1.0, 25.05)] {
            let c = cylinder(&["c.valve = \"5/3\"", &format!("c.heat_transfer_w_k = {heat}")], h);
            let x = 0.5 * c.stroke;
            let v = c.volumes(x);
            let p2 = 5e5;
            let p1 = (p2 * c.a2 + c.ambient * (c.a1 - c.a2)) / c.a1;
            let stiffness = n * (p1 * c.a1 * c.a1 / v[0] + p2 * c.a2 * c.a2 / v[1]);
            let expected = (stiffness / c.mass).sqrt() / std::f64::consts::TAU;
            near(expected, hz, 2e-3, "hand frequency");
            let mut s = vec![0.0; STATES];
            (s[X], s[V]) = (x, 1e-3);
            (s[M1], s[U1]) = (p1 * v[0] / (R * T0), p1 * v[0] / (KAPPA - 1.0));
            (s[M2], s[U2]) = (p2 * v[1] / (R * T0), p2 * v[1] / (KAPPA - 1.0));
            let (mut crossings, mut last, mut before) = (Vec::new(), s[V], 0.0);
            run(&c, &mut s, 0.0, h, 1.0, |t, s| {
                if last > 0.0 && s[V] <= 0.0 {
                    crossings.push(before + h * last / (last - s[V]));
                }
                (last, before) = (s[V], t);
            });
            let measured = (crossings.len() - 1) as f64 / (crossings[crossings.len() - 1] - crossings[0]);
            near(measured, expected, 0.02, &format!("frequency with {heat} W/K"));
        }
    }

    /// Air for a double stroke by hand: each chamber goes from the atmosphere at its dead volume to the supply at its
    /// full volume, m = (p_s (V0 + A s) - p_atm V0) / (R T): cap side (7.01325e5 * 8.5425e-5 - 1.01325e5 * 5e-6) /
    /// (287.058 * 293.15) = 7.0591e-4 kg, rod side (7.01325e5 * 7.4115e-5 - 0.5066) / 84152 = 6.1166e-4 kg; 1.31757e-3 kg,
    /// 1.1119 l (ANR, 1.185 kg/m3). The air settles back to the wall's temperature in the dwells; stiff stops keep
    /// their give out of the volumes.
    #[test]
    fn a_double_stroke_draws_the_hand_air_consumption() {
        let h = 1e-5;
        let c = cylinder(
            &["c.heat_transfer_w_k = 2.0", "c.stop_stiffness_n_m = 1.0e8", "c.stop_damping_n_s_m = 2000.0"],
            h,
        );
        let mut s = rest(&c, -1.0);
        run(&c, &mut s, 1.0, h, 2.0, |_, _| {});
        assert!(s[X] > c.stroke - 1e-4, "not extended: {}", s[X]);
        run(&c, &mut s, -1.0, h, 2.0, |_, _| {});
        assert!(s[X] < 1e-4, "not retracted: {}", s[X]);
        let full = |a: f64, dead: f64| (c.supply * (dead + a * c.stroke) - c.ambient * dead) / (R * T0);
        let expected = full(c.a1, c.dead[0]) + full(c.a2, c.dead[1]);
        near(expected, 1.31757e-3, 1e-4, "hand mass");
        near(s[AIR], expected, 0.005, "air drawn");
        near(s[AIR] / RHO0 * 1e3, 1.1119, 0.006, "normal litres");
    }

    /// A closed volume filled through a conductance, adiabatic: while choked (p <= b p_s) the flow is the constant
    /// C p_s rho0 and dp/dt = kappa R T_s mdot / V, so it takes t = (b p_s - p_atm) V / (kappa R T_s mdot). The piston
    /// is held in its stop by a load of 1e5 N, far beyond the force: the stop (1e8 N/m) gives 1 mm under it, so the cap
    /// side's volume is the dead volume, 50 cm3, less A1 * 1 mm: 4.920e-5 m3 (the pressure moves it by 0.01 % more).
    /// The path is valve and tube (C 1 and 2, b 0.3 and 0.4: C = 0.9615 dm3/(s bar), b = 1 - 0.9245 (0.7 + 0.6 / 4) =
    /// 0.2142): (0.2142 * 7.01325e5 - 1.01325e5) * 4.920e-5 / (1.4 * 287.058 * 293.15 * 7.9906e-3 kg/s) = 2.555 ms.
    #[test]
    fn filling_a_closed_volume_takes_the_closed_form_time_while_choked() {
        let h = 2e-6;
        let c = cylinder(
            &[
                "c.dead_volume_cap_m3 = 5.0e-5",
                "c.load_force_n = 1.0e5",
                "c.stop_stiffness_n_m = 1.0e8",
                "c.heat_transfer_w_k = 0.0",
            ],
            h,
        );
        let mut s = rest(&c, -1.0);
        let volume = c.volumes(s[X])[0];
        let mdot = c.inflow.mass_flow(c.supply, T0, c.ambient);
        let expected = (c.inflow.b * c.supply - c.ambient) * volume / (KAPPA * R * T0 * mdot);
        near(expected, 2.555e-3, 2e-3, "hand time");
        let (mut found, mut before) = (None, (0.0, c.gas(&s)[0][0]));
        run(&c, &mut s, 1.0, h, 0.01, |t, s| {
            let p = c.gas(s)[0][0];
            let target = c.inflow.b * c.supply;
            if found.is_none() && p >= target {
                found = Some(before.0 + h * (target - before.1) / (p - before.1));
            }
            before = (t, p);
        });
        near(found.unwrap(), expected, 0.005, "time to b p_s");
    }

    /// Energy conservation with the chambers closed (a 5/3 valve's centre), adiabatic and without friction: the gas's
    /// internal energy and the mechanical energy (kinetic, gravity, the atmosphere's work on the rod, the stops)
    /// together stay within 1e-9 of the gas's energy over 1 s of the piston bouncing on its air springs, vertical. The
    /// stops' dampers are off: they take energy whenever the piston touches a stop.
    #[test]
    fn a_closed_cylinder_keeps_its_energy() {
        let h = 1e-5;
        let changes =
            ["c.valve = \"5/3\"", "c.heat_transfer_w_k = 0.0", "c.incline_deg = 90.0", "c.stop_damping_n_s_m = 0.0"];
        let c = cylinder(&changes, h);
        // Mid-stroke at 3 and 4 bar, at 0.5 m/s: it swings on its air springs, into the stops and out.
        let mut s = vec![0.0; STATES];
        let v = c.volumes(0.5 * c.stroke);
        (s[X], s[V]) = (0.5 * c.stroke, 0.5);
        (s[M1], s[U1]) = (3e5 * v[0] / (R * T0), 3e5 * v[0] / (KAPPA - 1.0));
        (s[M2], s[U2]) = (4e5 * v[1] / (R * T0), 4e5 * v[1] / (KAPPA - 1.0));
        let total = |s: &[f64]| c.mechanical_energy(s) + c.gas_energy(s);
        let start = total(&s);
        let (mut worst, mut lowest) = (0f64, c.stroke);
        run(&c, &mut s, 0.0, h, 1.0, |_, s| {
            worst = worst.max((total(s) - start).abs());
            lowest = lowest.min(s[X]);
        });
        assert!(worst < 1e-9 * c.gas_energy(&s), "drifted by {worst} J");
        assert!(lowest < c.stroke - 1e-3, "the piston never left the stop");
    }

    /// The open system's balance in place of energy conservation (the suite's check is for a closed one): over an extend
    /// and a retract stroke with friction, cushions and heat transfer, the change of the gas's and the mechanical
    /// energy equals the enthalpy from the supply less that to the exhaust, plus the heat from the wall, less what the
    /// seals and the stops' dampers took, within 1e-6 of the enthalpy drawn.
    #[test]
    fn the_enthalpy_flows_balance_the_energy_of_an_open_cylinder() {
        let h = 1e-5;
        let cfg = config(true, &["c.incline_deg = -90.0", "c.load_force_n = 30.0", "c.heat_transfer_w_k = 0.3"]);
        let c = Cylinder::from_config(&cfg, "z", "cylinder", "c", h).unwrap();
        assert!(c.seal.is_some() && c.cushion > 0.0);
        let mut s = rest(&c, -1.0);
        let stored = |s: &[f64]| c.mechanical_energy(s) + c.gas_energy(s);
        let start = stored(&s);
        for spool in [1.0, -1.0] {
            run(&c, &mut s, spool, h, 1.5, |_, _| {});
        }
        let flows = s[H_IN] - s[H_OUT] + s[HEAT] - s[LOSS];
        let off = stored(&s) - start - flows;
        assert!(s[LOSS] > 0.0 && s[H_IN] > 0.0);
        assert!(off.abs() < 1e-6 * s[H_IN], "off by {off} J of {} J drawn", s[H_IN]);
    }

    /// Halving the step changes the stroke time by less than 1 % (and much less): a full extend stroke with cushions and
    /// friction at the simulation's sub-step (62.5 us / 4) and at half of it.
    #[test]
    fn the_stroke_time_converges_with_the_step() {
        let time = |h: f64| {
            let c = Cylinder::from_config(&config(true, &["c.incline_deg = -90.0"]), "z", "cylinder", "c", h).unwrap();
            let mut s = rest(&c, -1.0);
            let mut arrived = None;
            run(&c, &mut s, 1.0, h, 2.0, |t, s| {
                if arrived.is_none() && s[X] >= c.stroke - c.sensor_band {
                    arrived = Some(t);
                }
            });
            arrived.expect("never arrived")
        };
        let (coarse, fine) = (time(62.5e-6 / 4.0), time(62.5e-6 / 8.0));
        assert!((coarse - fine).abs() < 0.01 * fine, "{coarse} s against {fine} s");
    }

    /// A dead volume so small that the flow law's straight end is stiffer than the step can integrate is refused by
    /// name, not run into NaN: 1e-9 m3 with the default flows needs a sub-step below about 0.1 us.
    #[test]
    fn a_cylinder_too_stiff_for_the_step_is_refused() {
        let cfg = config(true, &["c.dead_volume_rod_m3 = 1.0e-9"]);
        let error = Cylinder::from_config(&cfg, "z", "cylinder", "c", 62.5e-6 / 4.0).unwrap_err();
        assert_eq!(error.key(), "error-cylinder-unstable");
        assert!(error.detail().contains("flow"), "{}", error.detail());
    }

    /// The energy judged against the cushion's rating at 0.2 m/s into the 17 mm cushion with m = moved mass is the
    /// kinetic m v^2 / 2 alone (the vendor's rule), whichever way the weight acts: hanging 1 kg gives 0.02 J (m g L =
    /// 0.1667 J of weight work is not added), extending upwards and retracting down likewise.
    #[test]
    fn the_cushion_energy_is_the_kinetic_energy_whichever_way_the_weight_acts() {
        let entering = |incline: &str, spool: f64, x: f64| {
            let mut c = cylinder(&[incline], 1e-5);
            c.cushion = 0.017;
            c.track.spool = spool;
            c.track.cushioned = 0.0;
            let mut s = rest(&c, spool);
            (s[X], s[V]) = (x, 0.2 * spool);
            c.stepped(&s, 1e-5);
            (c.track.impact_energy, c.mass + c.payload)
        };
        let (down, m) = entering("c.incline_deg = -90.0", 1.0, 0.1 - 0.0169);
        near(down, 0.5 * m * 0.04, 1e-9, "extending down");
        let (up, m) = entering("c.incline_deg = 90.0", 1.0, 0.1 - 0.0169);
        near(up, 0.5 * m * 0.04, 1e-9, "extending up");
        let (retract, m) = entering("c.incline_deg = 90.0", -1.0, 0.0169);
        near(retract, 0.5 * m * 0.04, 1e-9, "retracting down");
    }
}
