//! Transmissions that turn a motor's rotation into the motion of a load, beside belts and planetary gearboxes: ball
//! screw, rack and pinion, linear motor, worm gear and strain wave gear. Each gives what the simulation needs (ratio
//! from motor radians to the load's coordinate, efficiency, stiffness, inertia, drag) and the ratings its maker checks
//! a cycle against, with the maker's published formulas. They know no machine; `chain::JointMachine` puts them on a
//! joint.
//!
//! Units: lengths in m, forces in N, torques in N m, speeds of rotation in r/min where a catalogue gives them so.

use crate::joint::RotaryJoint;
use crate::mech::{BRISTLE_RATE_MAX, HoldingBrake, TravelCompliance};
use sim_core::{Config, ConfigError, RatingCheck, SignalInfo};
use std::f64::consts::PI;

type Result<T> = std::result::Result<T, ConfigError>;

/// Young's modulus and density of screw steel, Pa and kg/m^3.
pub const STEEL_E: f64 = 2.06e11;
pub const STEEL_DENSITY: f64 = 7850.0;

/// How a screw shaft is held at its two ends, for its critical speed and buckling load.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Mounting {
    FixedFree,
    SupportedSupported,
    FixedSupported,
    FixedFixed,
}

impl Mounting {
    pub fn parse(name: &str) -> Option<Self> {
        Some(match name {
            "fixed-free" => Self::FixedFree,
            "supported-supported" => Self::SupportedSupported,
            "fixed-supported" => Self::FixedSupported,
            "fixed-fixed" => Self::FixedFixed,
            _ => return None,
        })
    }

    /// lambda L of the first bending mode of a uniform beam with these ends (Euler-Bernoulli): 1.875, pi, 3.927, 4.730.
    pub fn mode_factor(self) -> f64 {
        match self {
            Self::FixedFree => 1.875_104,
            Self::SupportedSupported => PI,
            Self::FixedSupported => 3.926_602,
            Self::FixedFixed => 4.730_041,
        }
    }

    /// The factor on the pin-ended Euler load pi^2 E I / L^2: 1/4, 1, 2.046, 4.
    pub fn buckling_factor(self) -> f64 {
        match self {
            Self::FixedFree => 0.25,
            Self::SupportedSupported => 1.0,
            Self::FixedSupported => 2.046,
            Self::FixedFixed => 4.0,
        }
    }
}

/// What turns a gearbox's output rotation into a joint's travel (ball screw, rack and pinion): the travel per radian of
/// its input (`lever`, m/rad), its own stiffness along the travel (N/m), its backlash (m), efficiency, the inertia
/// that turns with its input (screw shaft, pinion; kg m^2), and the damping ratio of its spring's own mode (nut,
/// bearings and guides; `apply_back`).
#[derive(Debug, Clone, PartialEq)]
pub struct Converter {
    pub lever: f64,
    pub stiffness: f64,
    pub backlash: f64,
    pub efficiency: f64,
    pub inertia: f64,
    pub damping_ratio: f64,
}

impl Converter {
    /// Puts the converter behind `joint`'s gearbox (read in N m and rad at the converter's input): the joint's gearbox
    /// becomes gearbox and converter together, in N and m. Ratio i / lever; stiffness of the gearbox seen at the joint
    /// (over lever^2) in series with the converter's, the gearbox's damping across its own spring alone (a Zener solid,
    /// `Gearbox::arm`); backlash and efficiency added and multiplied as
    /// effects.gearbox_backlash and effects.gearbox_efficiency say; the converter's inertia joins the rotor over i^2.
    ///
    /// With effects.thermal_feedback the gearbox's temperature changes the gearbox's share of the efficiency, not the
    /// converter's (`RotaryJoint::extra_efficiency`).
    ///
    /// The output's own inertia is not known here: the mesh friction's bristle is capped by the rotor's (`apply_to`).
    pub fn apply(&self, cfg: &Config, joint: &mut RotaryJoint) -> Result<()> {
        self.apply_to(cfg, joint, f64::INFINITY)
    }

    /// `apply` for an output of the given inertia at the joint's travel (kg behind a screw, rack or drum): the mesh
    /// friction's bristle, between the gear's output and the housing, is capped by the smaller of it and the rotor's
    /// inertia seen at the load, so that a light output (a winch drum) does not stick to the housing at a frequency the
    /// step cannot follow.
    pub fn apply_to(&self, cfg: &Config, joint: &mut RotaryJoint, output: f64) -> Result<()> {
        self.apply_back(cfg, joint, output, self.efficiency)
    }

    /// `apply_to` for a converter that loses `back_efficiency` where the load drives it back (`self.efficiency` driving):
    /// the gearbox's back-driving efficiency becomes its own times back_efficiency / efficiency (`Gearbox::back_ratio`).
    ///
    /// The converter's spring is in series with the whole gear. The lost-motion band is the gearbox's angle times the
    /// lever, and over it the converter's spring stretches with the force as it does beyond (`Gearbox::band_compliance`);
    /// the line beyond the band keeps its offset, b lever / 2. The hysteresis, a friction T_h in parallel with the
    /// flanks' spring K1 (lever^2 / K1 in m/N at the joint): sliding, the force is K1' (x -+ lever T_h / K1) with K1' the
    /// series stiffness, so its friction at the joint is T_h / lever times K1' lever^2 / K1, and its loop as wide as the
    /// gear's, 2 lever T_h / K1; at a reversal the gear is K1 + sigma, in series K1' + sigma' with
    /// sigma' = series(K1 + sigma) - series(K1).
    pub fn apply_back(&self, cfg: &Config, joint: &mut RotaryJoint, output: f64, back_efficiency: f64) -> Result<()> {
        let (g, l) = (&mut joint.gearbox, self.lever);
        let i = g.ratio;
        g.ratio = i / l;
        let converter = self.stiffness;
        let mesh_factor = g.mesh_stiffness / g.stiffness;
        let series = move |k: f64| 1.0 / (l * l / k + 1.0 / converter);
        let (k1, sigma) = (g.stiffness, g.dahl_stiffness);
        // The three segments are given in torque, so each is in series with the converter as K1 is; the knees are
        // torques at the gearbox output, forces over the lever at the joint.
        [g.stiffness, g.upper[0], g.upper[1]] = [g.stiffness, g.upper[0], g.upper[1]].map(series);
        g.dahl_stiffness = series(k1 + sigma) - g.stiffness;
        g.knees = g.knees.map(|t| t / l);
        g.hysteresis *= g.stiffness * l / k1;
        g.band *= l;
        g.band_compliance = g.band_compliance * l * l + 1.0 / converter;
        // Two damped springs in series, the node between them massless: the gear's K_g = K1 / l^2 with its damper c_g
        // across it alone, and the converter's K_c with c_c = 2 zeta sqrt(K_c m) across it alone, zeta its
        // `damping_ratio` and m the mass behind it (`output`; none where it is not known). Their K_s + b s + K_m tau s /
        // (1 + tau s) holds it exactly (`Gearbox::arm`): b = c_g c_c / C, tau = C / S, K_m = (K_g c_c - K_c c_g)^2 /
        // (S C^2), with S = K_g + K_c, C = c_g + c_c. An arm that would settle faster than the integrator follows
        // (`BRISTLE_RATE_MAX`, tau under 10 us) is its slow limit, u following x: a damper K_m tau beside b, off by
        // (omega tau)^2 at a ring of omega (0.4 % at 1 kHz). A rigid converter leaves the gear's damper across it.
        let damping = g.damping / (l * l);
        let own = if output.is_finite() && converter.is_finite() {
            2.0 * self.damping_ratio * (converter * output).sqrt()
        } else {
            0.0
        };
        let (sum, dampers) = (converter + k1 / (l * l), damping + own);
        let (across, tau) = (if dampers > 0.0 { damping * own / dampers } else { 0.0 }, dampers / sum);
        let arm = if dampers > 0.0 {
            (k1 / (l * l) * own - converter * damping).powi(2) / (sum * dampers * dampers)
        } else {
            0.0
        };
        if !converter.is_finite() {
            g.damping = damping;
        } else if arm > 0.0 && tau * BRISTLE_RATE_MAX >= 1.0 {
            (g.relax_stiffness, g.relax_time, g.damping) = (arm, tau, across);
        } else {
            g.damping = across + arm * tau;
        }
        g.half_backlash =
            g.half_backlash * l + if cfg.bool("effects.gearbox_backlash")? { 0.5 * self.backlash } else { 0.0 };
        if cfg.bool("effects.gearbox_efficiency")? {
            g.efficiency *= self.efficiency;
            joint.extra_efficiency *= self.efficiency;
            g.back_ratio *= back_efficiency / self.efficiency;
        }
        joint.rotor_inertia += self.inertia / (i * i);
        // The mesh friction's bristle is `gearbox.mesh_stiffness_factor` (MESH_STIFFNESS) times the converted series stiffness (N/m now, not N m/rad). No
        // shipped machine reaches the cap below: rack, screw and both machines hung in cables run the full 2.6e7 to 5.6e7 N/m, and
        // their gearbox loss per cycle is the same at a tenth and at ten times it to 4e-4 J (rack 16.741 J, screw
        // 16.762 J, cable-planar 38.167 J, cable-spatial 53.544 to 53.548 J), with their energy budgets closed to 2e-5 J.
        // A gear far stiffer than any shipped (1e12 N m/rad behind a rigid lever) is capped where the bristle on the
        // lighter of the rotor's inertia seen at the load and the output's own stays at a quarter of BRISTLE_RATE_MAX
        // (omega h = 0.39 at the 15.6 us step against RK4's 2.8). A winch drum of 0.32 kg at its radius puts the cap at
        // 2e8 N/m, above what the winches' gears give.
        if g.mesh_stiffness > 0.0 {
            let lighter = (joint.rotor_inertia * g.ratio * g.ratio).min(output);
            let omega = BRISTLE_RATE_MAX / 4.0;
            g.mesh_stiffness = (mesh_factor * g.stiffness).min(lighter * omega * omega);
        }
        joint.brake = HoldingBrake::from_config(cfg, joint.rotor_inertia)?;
        Ok(())
    }
}

/// A converter with its own back-driving efficiency (`BallScrew::converter`): derefs to the `Converter`, and applies like
/// it (`Converter::apply`, `apply_to`) with the back efficiency in the gearbox, and the screw's compliance along the
/// travel where it follows the nut (`Gearbox::travel`).
#[derive(Debug, Clone, PartialEq)]
pub struct BackDriven {
    pub converter: Converter,
    pub back_efficiency: f64,
    pub travel: Option<TravelCompliance>,
}

impl std::ops::Deref for BackDriven {
    type Target = Converter;
    fn deref(&self) -> &Converter {
        &self.converter
    }
}

impl BackDriven {
    pub fn apply(&self, cfg: &Config, joint: &mut RotaryJoint) -> Result<()> {
        self.apply_to(cfg, joint, f64::INFINITY)
    }

    pub fn apply_to(&self, cfg: &Config, joint: &mut RotaryJoint, output: f64) -> Result<()> {
        self.converter.apply_back(cfg, joint, output, self.back_efficiency)?;
        joint.gearbox.travel = self.travel;
        Ok(())
    }
}

/// A ball screw: shaft, nut and the bearings that hold the shaft. The nut moves the load by `lead` per turn.
///
/// Config `[screw]`: `lead_m`, `nominal_diameter_m`, `root_diameter_m`, `length_m` (between the bearings, or from the
/// fixed bearing to the free end), `mounting` (as `Mounting::parse`), `efficiency`, `friction_coefficient` (with
/// effects.screw_lead_efficiency, `lead_efficiency`), `nut_stiffness_n_m`,
/// `bearing_stiffness_n_m`, `preload_n`, `dynamic_load_rating_n` (Ca), `static_load_rating_n` (C0a), `load_factor`
/// (fw), `static_safety_factor`, `critical_speed_factor`, `buckling_safety_factor`, `max_dn` (d n with d in mm and n in
/// r/min; 0 for none), and optionally `axial_damping_ratio` (the damping ratio of the mass behind the nut on the screw's
/// axial spring, from the nut, the bearings and the guides: `Converter::apply_back`; 0 where it is absent). With
/// effects.screw_stiffness_with_position (off where absent), `fixed_bearing_at_m`: the joint's travel at which the fixed
/// bearing stands (`TravelCompliance`).
#[derive(Debug, Clone, PartialEq)]
pub struct BallScrew {
    pub lead: f64,
    pub nominal_diameter: f64,
    pub root_diameter: f64,
    pub length: f64,
    pub mounting: Mounting,
    pub efficiency: f64,
    /// Driven back by the load, with effects.screw_lead_efficiency (`lead_back_efficiency`); the forward one otherwise.
    pub back_efficiency: f64,
    pub nut_stiffness: f64,
    pub bearing_stiffness: f64,
    pub preload: f64,
    pub dynamic_rating: f64,
    pub static_rating: f64,
    pub load_factor: f64,
    pub static_safety: f64,
    pub speed_factor: f64,
    pub buckling_safety: f64,
    pub max_dn: f64,
    pub axial_damping_ratio: f64,
    /// With effects.screw_stiffness_with_position: the joint's travel at the fixed bearing, m. None: the stiffness is
    /// the one at the position `converter` is given for the whole stroke.
    pub fixed_bearing_at: Option<f64>,
}

/// An optional damping ratio, 0 where the key is absent (a config written before 20 November 2026), below 1.
fn damping_ratio(cfg: &Config, key: &str) -> Result<f64> {
    if cfg.get(key).is_err() {
        return Ok(0.0);
    }
    let zeta = cfg.non_negative(key)?;
    if zeta >= 1.0 {
        return Err(ConfigError::Invalid(key.into()));
    }
    Ok(zeta)
}

impl BallScrew {
    pub fn from_config(cfg: &Config) -> Result<Self> {
        let p = |name: &str| cfg.positive(&format!("screw.{name}"));
        let mounting = cfg
            .get("screw.mounting")?
            .as_str()
            .and_then(Mounting::parse)
            .ok_or_else(|| ConfigError::Invalid("screw.mounting".into()))?;
        let (nominal, root) = (p("nominal_diameter_m")?, p("root_diameter_m")?);
        if root > nominal {
            return Err(ConfigError::Invalid("screw.root_diameter_m".into()));
        }
        let efficiency = p("efficiency")?;
        if efficiency > 1.0 {
            return Err(ConfigError::Invalid("screw.efficiency".into()));
        }
        let lead = p("lead_m")?;
        // A config without the switch (one written before 16 November 2026) has it off.
        let (efficiency, back_efficiency) =
            if cfg.get("effects.screw_lead_efficiency").is_ok() && cfg.bool("effects.screw_lead_efficiency")? {
                let mu = cfg.positive("screw.friction_coefficient")?;
                let back = Self::lead_back_efficiency(lead, nominal, mu);
                if back <= 0.0 {
                    return Err(ConfigError::Invalid(
                        "screw.friction_coefficient (the screw locks itself driven back: its efficiency there is \
                         not positive, and this gearbox does not model a self-locking screw)"
                            .into(),
                    ));
                }
                (Self::lead_efficiency(lead, nominal, mu), back)
            } else {
                (efficiency, efficiency)
            };
        Ok(Self {
            lead,
            nominal_diameter: nominal,
            root_diameter: root,
            length: p("length_m")?,
            mounting,
            efficiency,
            back_efficiency,
            nut_stiffness: p("nut_stiffness_n_m")?,
            bearing_stiffness: p("bearing_stiffness_n_m")?,
            preload: cfg.non_negative("screw.preload_n")?,
            dynamic_rating: p("dynamic_load_rating_n")?,
            static_rating: p("static_load_rating_n")?,
            load_factor: p("load_factor")?,
            static_safety: p("static_safety_factor")?,
            speed_factor: p("critical_speed_factor")?,
            buckling_safety: p("buckling_safety_factor")?,
            max_dn: cfg.non_negative("screw.max_dn")?,
            axial_damping_ratio: damping_ratio(cfg, "screw.axial_damping_ratio")?,
            // A config without the switch (one written before 21 November 2026) has it off.
            fixed_bearing_at: if cfg.get("effects.screw_stiffness_with_position").is_ok()
                && cfg.bool("effects.screw_stiffness_with_position")?
            {
                Some(cfg.f64("screw.fixed_bearing_at_m")?)
            } else {
                None
            },
        })
    }

    /// The rolling efficiency of a ball screw driving its nut, from its lead angle beta = atan(lead / (pi d)) and the
    /// balls' friction coefficient mu (THK's formula): (1 - mu tan beta) / (1 + mu / tan beta), tan beta / tan(beta + rho)
    /// with mu = tan rho. effects.screw_lead_efficiency puts it in place of `screw.efficiency`, which is a catalogue's
    /// lumped figure that holds the nut's preload drag too; the preload drags apart (`preload_torque`).
    pub fn lead_efficiency(lead: f64, nominal_diameter: f64, mu: f64) -> f64 {
        let tan_beta = lead / (PI * nominal_diameter);
        (1.0 - mu * tan_beta) / (1.0 + mu / tan_beta)
    }

    /// The efficiency of the load driving the screw back (THK): (1 - mu / tan beta) / (1 + mu tan beta). Zero or less
    /// where tan beta <= mu: the screw locks itself (`from_config` refuses that). It is the driving one with force and
    /// motion swapped, tan(beta - rho) / tan(beta) with mu = tan rho.
    pub fn lead_back_efficiency(lead: f64, nominal_diameter: f64, mu: f64) -> f64 {
        let tan_beta = lead / (PI * nominal_diameter);
        (1.0 - mu / tan_beta) / (1.0 + mu * tan_beta)
    }

    /// Screw radians per metre of travel: 2 pi / lead.
    pub fn ratio(&self) -> f64 {
        2.0 * PI / self.lead
    }

    /// The shaft's moment of inertia about its axis, a solid steel cylinder of the nominal diameter: pi rho L d^4 / 32.
    pub fn inertia(&self) -> f64 {
        PI * STEEL_DENSITY * self.length * self.nominal_diameter.powi(4) / 32.0
    }

    /// Second moment of area and area of the root section.
    fn section(&self) -> (f64, f64) {
        let d = self.root_diameter;
        (PI * d.powi(4) / 64.0, PI * d * d / 4.0)
    }

    /// The first bending mode of the shaft as a speed, r/min: n = 60 / (2 pi) (lambda L)^2 / L^2 sqrt(E I / (rho A)),
    /// a uniform beam of the root diameter over `length`.
    pub fn critical_speed_rpm(&self) -> f64 {
        let (i, a) = self.section();
        let omega = (self.mounting.mode_factor() / self.length).powi(2) * (STEEL_E * i / (STEEL_DENSITY * a)).sqrt();
        omega * 60.0 / (2.0 * PI)
    }

    /// The highest screw speed the cycle may reach, r/min: the critical speed times `critical_speed_factor` (makers
    /// ask for 0.8).
    pub fn allowed_speed_rpm(&self) -> f64 {
        self.speed_factor * self.critical_speed_rpm()
    }

    /// Euler's buckling load of the shaft, N: factor pi^2 E I / L^2 with the mounting's factor.
    pub fn buckling_load(&self) -> f64 {
        self.mounting.buckling_factor() * PI * PI * STEEL_E * self.section().0 / (self.length * self.length)
    }

    /// The highest axial force allowed against buckling: the buckling load over `buckling_safety_factor`.
    pub fn allowed_buckling_force(&self) -> f64 {
        self.buckling_load() / self.buckling_safety
    }

    /// The highest axial force allowed by the static rating: C0a over the static safety factor.
    pub fn allowed_static_force(&self) -> f64 {
        self.static_rating / self.static_safety
    }

    /// Axial stiffness between the motor's shaft angle and the nut, N/m, with the nut `x` from the fixed bearing (the
    /// fixed-fixed shaft held at both ends: A E L / (x (L - x))): shaft, nut and bearings in series. The shaft's
    /// share at least that of its whole length.
    pub fn stiffness(&self, x: f64) -> f64 {
        let ae = STEEL_E * self.section().1;
        let x = x.clamp(1e-3 * self.length, self.length);
        let shaft = match self.mounting {
            Mounting::FixedFixed => ae * self.length / (x * (self.length - x).max(1e-3 * self.length)),
            _ => ae / x,
        };
        1.0 / (1.0 / shaft + 1.0 / self.nut_stiffness + 1.0 / self.bearing_stiffness)
    }

    /// The drag torque of the nut's preload, N m (THK's formula): K Fpr l / (2 pi), K = 0.05 / sqrt(tan beta), beta the
    /// lead angle atan(l / (pi d)).
    pub fn preload_torque(&self) -> f64 {
        let tan_beta = self.lead / (PI * self.nominal_diameter);
        0.05 / tan_beta.sqrt() * self.preload * self.lead / (2.0 * PI)
    }

    /// The load each ball track of a preloaded double nut carries under an axial force: Fpr (1 + Fa / (2.83 Fpr))^(3/2)
    /// until the force lifts the preload off the other nut (at 2.83 Fpr), the force itself beyond (Hiwin's catalogue).
    pub fn equivalent_load(&self, force: f64) -> f64 {
        let f = force.abs();
        let lift = 2.83 * self.preload;
        if self.preload <= 0.0 || f >= lift { f } else { self.preload * (1.0 + f / lift).powf(1.5) }
    }

    /// The share of the nominal life (L10 = (Ca / (fw Fm))^3 10^6 turns, ISO 3408-5) used per hour at an axial force
    /// and a screw speed (r/min): 60 |n| (fw F_eq / Ca)^3 / 10^6. Its mean over a cycle is one over the life in hours.
    pub fn damage_rate(&self, force: f64, rpm: f64) -> f64 {
        60.0 * rpm.abs() * (self.load_factor * self.equivalent_load(force) / self.dynamic_rating).powi(3) / 1e6
    }

    /// d n, the nominal diameter in mm times the speed in r/min, against `max_dn`.
    pub fn dn(&self, rpm: f64) -> f64 {
        self.nominal_diameter * 1e3 * rpm.abs()
    }

    /// The screw as a converter, stiff as it is with the nut at `x` from the fixed bearing. Every builder passes
    /// `softest_position()`. Without `fixed_bearing_at` that is the joint gearbox's stiffness for the whole stroke, which
    /// judges resonance and settling on the safe side; with it the gearbox follows the nut from there (`travel`).
    pub fn converter(&self, x: f64) -> BackDriven {
        BackDriven {
            converter: Converter {
                lever: 1.0 / self.ratio(),
                stiffness: self.stiffness(x),
                backlash: 0.0,
                efficiency: self.efficiency,
                inertia: self.inertia(),
                damping_ratio: self.axial_damping_ratio,
            },
            back_efficiency: self.back_efficiency,
            travel: self.travel(x),
        }
    }

    /// The shaft's compliance along the travel with `x` as the reference (`TravelCompliance`), where it follows the nut.
    pub fn travel(&self, x: f64) -> Option<TravelCompliance> {
        let fixed_bearing_at = self.fixed_bearing_at?;
        let mut t = TravelCompliance {
            ae: STEEL_E * self.section().1,
            length: self.length,
            both_ends: self.mounting == Mounting::FixedFixed,
            fixed_bearing_at,
            reference: 0.0,
        };
        t.reference = t.shaft(x).0;
        Some(t)
    }

    /// Where the shaft is softest: its far end, or its middle where both ends are fixed.
    pub fn softest_position(&self) -> f64 {
        if self.mounting == Mounting::FixedFixed { 0.5 * self.length } else { self.length }
    }
}

/// The standard pressure angle of a rack and pinion's teeth (DIN 867), in the normal section: the default of
/// `rack.pressure_angle_deg`.
const RACK_PRESSURE_ANGLE_DEG: f64 = 20.0;

/// A pinion on a rack. Config `[rack]`: `module_m`, `teeth`, `helix_angle_deg`, `efficiency`, `stiffness_n_m` (mesh and
/// rack mounting), `backlash_m`, `pinion_inertia_kgm2`, `rated_feed_force_n` (the maker's permissible feed force of the
/// pair), `service_factor` (K_A), and optionally `pressure_angle_deg` (normal section, 20 where absent) and
/// `damping_ratio` (the damping ratio of the mass behind the pinion on the mesh's spring: `Converter::apply_back`; 0
/// where absent).
#[derive(Debug, Clone, PartialEq)]
pub struct RackPinion {
    pub module: f64,
    pub teeth: f64,
    pub helix_angle: f64,
    pub efficiency: f64,
    pub stiffness: f64,
    pub backlash: f64,
    pub pinion_inertia: f64,
    pub rated_force: f64,
    pub service_factor: f64,
    /// Normal pressure angle, rad.
    pub pressure_angle: f64,
    pub damping_ratio: f64,
}

impl RackPinion {
    pub fn from_config(cfg: &Config) -> Result<Self> {
        let p = |name: &str| cfg.positive(&format!("rack.{name}"));
        let helix = cfg.non_negative("rack.helix_angle_deg")?;
        let efficiency = p("efficiency")?;
        if helix >= 45.0 || efficiency > 1.0 {
            return Err(ConfigError::Invalid(
                if helix >= 45.0 { "rack.helix_angle_deg" } else { "rack.efficiency" }.into(),
            ));
        }
        let alpha =
            if cfg.get("rack.pressure_angle_deg").is_ok() { p("pressure_angle_deg")? } else { RACK_PRESSURE_ANGLE_DEG };
        if alpha >= 45.0 {
            return Err(ConfigError::Invalid("rack.pressure_angle_deg".into()));
        }
        Ok(Self {
            pressure_angle: alpha.to_radians(),
            module: p("module_m")?,
            teeth: p("teeth")?,
            helix_angle: helix.to_radians(),
            efficiency,
            stiffness: p("stiffness_n_m")?,
            backlash: cfg.non_negative("rack.backlash_m")?,
            pinion_inertia: p("pinion_inertia_kgm2")?,
            rated_force: p("rated_feed_force_n")?,
            service_factor: p("service_factor")?,
            damping_ratio: damping_ratio(cfg, "rack.damping_ratio")?,
        })
    }

    /// The pinion's pitch radius: m z / (2 cos beta), the normal module m.
    pub fn radius(&self) -> f64 {
        self.module * self.teeth / (2.0 * self.helix_angle.cos())
    }

    /// The highest feed force the cycle may reach: the rated feed force over the service factor.
    pub fn allowed_force(&self) -> f64 {
        self.rated_force / self.service_factor
    }

    pub fn converter(&self) -> Converter {
        Converter {
            lever: self.radius(),
            stiffness: self.stiffness,
            backlash: self.backlash,
            efficiency: self.efficiency,
            inertia: self.pinion_inertia,
            damping_ratio: self.damping_ratio,
        }
    }
}

/// An ironcore linear motor: the forcer on the carriage, the magnet track along the axis. Config `[linear_motor]`:
/// `attraction_force_n` (between forcer and magnets, on the guide), `pole_pitch_m` (north to north, the period of the
/// cogging). The force constant, resistance and the rest are the drive's `[motor]` values, read with N for N m and m for
/// rad.
#[derive(Debug, Clone, PartialEq)]
pub struct LinearMotor {
    pub attraction: f64,
    pub pole_pitch: f64,
}

impl LinearMotor {
    pub fn from_config(cfg: &Config) -> Result<Self> {
        Ok(Self {
            attraction: cfg.non_negative("linear_motor.attraction_force_n")?,
            pole_pitch: cfg.positive("linear_motor.pole_pitch_m")?,
        })
    }

    /// The cogging's order for `mech::Ripple`, whose angle is here the forcer's position: one period per pole pitch.
    pub fn cogging_order(&self) -> f64 {
        2.0 * PI / self.pole_pitch
    }
}

/// A worm gear. Config `[worm]`: `lead_angle_deg` (gamma), `friction_coefficient` (mu), `pressure_angle_deg` (alpha_n).
/// The ratio is the gearbox's.
#[derive(Debug, Clone, PartialEq)]
pub struct WormGear {
    pub lead_angle: f64,
    pub friction: f64,
    pub pressure_angle: f64,
}

impl WormGear {
    pub fn from_config(cfg: &Config) -> Result<Self> {
        let gamma = cfg.positive("worm.lead_angle_deg")?;
        let alpha = cfg.non_negative("worm.pressure_angle_deg")?;
        if gamma >= 45.0 || alpha >= 45.0 {
            return Err(ConfigError::Invalid(
                if gamma >= 45.0 { "worm.lead_angle_deg" } else { "worm.pressure_angle_deg" }.into(),
            ));
        }
        Ok(Self {
            lead_angle: gamma.to_radians(),
            friction: cfg.non_negative("worm.friction_coefficient")?,
            pressure_angle: alpha.to_radians(),
        })
    }

    /// The friction angle on the flank: rho' = atan(mu / cos alpha_n).
    fn friction_angle(&self) -> f64 {
        (self.friction / self.pressure_angle.cos()).atan()
    }

    /// Driven by the worm: tan gamma / tan(gamma + rho').
    pub fn efficiency(&self) -> f64 {
        self.lead_angle.tan() / (self.lead_angle + self.friction_angle()).tan()
    }

    /// Driven by the wheel: tan(gamma - rho') / tan gamma; zero or less where the gear locks itself.
    pub fn back_efficiency(&self) -> f64 {
        (self.lead_angle - self.friction_angle()).tan() / self.lead_angle.tan()
    }
}

/// A strain wave gear (Harmonic Drive's catalogue method). Config `[strain_wave]`: `rated_torque_nm` (Tr, at the rated
/// input speed), `rated_input_speed_rpm` (nr, 2000 in the catalogues), `rated_life_h` (Ln: 7000 h for CSF, 10000 h for
/// CSG). The life is L10 = Ln (Tr / Tav)^3 (nr / nav), Tav the cubic mean of the output torque weighted with the input
/// speed and nav the mean input speed.
#[derive(Debug, Clone, PartialEq)]
pub struct StrainWave {
    pub rated_torque: f64,
    pub rated_speed: f64,
    pub rated_life: f64,
}

impl StrainWave {
    pub fn from_config(cfg: &Config) -> Result<Self> {
        let p = |name: &str| cfg.positive(&format!("strain_wave.{name}"));
        Ok(Self {
            rated_torque: p("rated_torque_nm")?,
            rated_speed: p("rated_input_speed_rpm")?,
            rated_life: p("rated_life_h")?,
        })
    }

    /// The share of the rated life used per hour at an output torque and input speed (r/min): |n| / nr (|T| / Tr)^3 / Ln.
    pub fn damage_rate(&self, torque: f64, input_rpm: f64) -> f64 {
        input_rpm.abs() / self.rated_speed * (torque.abs() / self.rated_torque).powi(3) / self.rated_life
    }
}

/// The signals and rating checks of a ball screw on joint `id` (one of `chain::MOTOR_IDS`, its motor `motor`): screw
/// speed (r/min), d n, and the share of its life used per hour, from the joint's speed and force (signal
/// `joint.<id>.force`). Checks: "screw-speed" (against `allowed_speed_rpm`), "screw-dn" (where `max_dn` is set),
/// "screw-buckling" (|force| against `allowed_buckling_force`; pulling too, on the safe side), "screw-static" (against
/// `allowed_static_force`) and "screw-life" (against `required_life_h`, where set).
impl BallScrew {
    pub fn signal_infos(&self, id: &str, key: &'static str) -> Vec<SignalInfo> {
        vec![
            SignalInfo::new(format!("screw.{id}.speed"), key, "quantity-screw-speed", "rpm"),
            SignalInfo::new(format!("screw.{id}.dn"), key, "quantity-screw-dn", "mm rpm"),
            SignalInfo::new(format!("screw.{id}.damage_rate"), key, "quantity-damage-rate", "1/h"),
        ]
    }

    /// Joint speed (m/s) and force (N) to the values of `signal_infos`.
    pub fn signals(&self, speed: f64, force: f64, out: &mut Vec<f64>) {
        let rpm = speed * self.ratio() * 60.0 / (2.0 * PI);
        out.extend([rpm, self.dn(rpm), self.damage_rate(force, rpm)]);
    }

    pub fn rating_checks(&self, id: &str, motor: usize, required_life_h: Option<f64>) -> Vec<RatingCheck> {
        let check = |check: &'static str, signal: &str, limit: f64, life: bool| RatingCheck {
            check,
            signal: signal.into(),
            motor: Some(motor),
            limit,
            life,
        };
        let force = format!("joint.{id}.force");
        let mut out = vec![
            check("screw-speed", &format!("screw.{id}.speed"), self.allowed_speed_rpm(), false),
            check("screw-buckling", &force, self.allowed_buckling_force(), false),
            check("screw-static", &force, self.allowed_static_force(), false),
        ];
        if self.max_dn > 0.0 {
            out.push(check("screw-dn", &format!("screw.{id}.dn"), self.max_dn, false));
        }
        if let Some(life) = required_life_h {
            out.push(check("screw-life", &format!("screw.{id}.damage_rate"), life, true));
        }
        out
    }
}

impl RackPinion {
    pub fn signal_info(&self, id: &str, key: &'static str) -> SignalInfo {
        SignalInfo::new(format!("rack.{id}.feed_force"), key, "quantity-rack-feed-force", "N")
    }

    /// The force on the teeth for the force `force` the rack passes to the load at `speed` and the efficiency
    /// `efficiency` in effect (1 with effects.gearbox_efficiency off): more than it while the pinion drives, less while
    /// the load drives it back.
    pub fn feed_force(force: f64, speed: f64, efficiency: f64) -> f64 {
        if force * speed >= 0.0 { force / efficiency } else { force * efficiency }
    }

    /// The radial load the pinion puts on the gearbox's output shaft for the force `tooth_force` on its teeth: the tooth
    /// force itself, tangential to the pinion and so across the shaft, and the force that pushes pinion and rack apart,
    /// F tan(alpha_n) / cos(beta), at right angles to it: |F| sqrt(1 + (tan(alpha_n) / cos(beta))^2). The axial force of
    /// a helical pinion, F tan(beta), and the tilt it puts on the shaft are left out.
    pub fn radial_load(&self, tooth_force: f64) -> f64 {
        let apart = self.pressure_angle.tan() / self.helix_angle.cos();
        tooth_force.abs() * (1.0 + apart * apart).sqrt()
    }

    /// "rack-feed-force": the force on the teeth of joint `id` (signal `rack.<id>.feed_force`) against `allowed_force`.
    pub fn rating_checks(&self, id: &str, motor: usize) -> Vec<RatingCheck> {
        vec![RatingCheck {
            check: "rack-feed-force",
            signal: format!("rack.{id}.feed_force"),
            motor: Some(motor),
            limit: self.allowed_force(),
            life: false,
        }]
    }
}

/// The share of a strain wave gear's life used per hour on joint `id`, from its output torque and input speed (rad/s),
/// and the check "strain-wave-life" against `required_life_h`.
impl StrainWave {
    pub fn signal_info(&self, id: &str, key: &'static str) -> SignalInfo {
        SignalInfo::new(format!("strain_wave.{id}.damage_rate"), key, "quantity-damage-rate", "1/h")
    }

    pub fn signal(&self, torque: f64, input_speed: f64) -> f64 {
        self.damage_rate(torque, input_speed * 60.0 / (2.0 * PI))
    }

    pub fn rating_check(&self, id: &str, motor: usize, required_life_h: f64) -> RatingCheck {
        RatingCheck {
            check: "strain-wave-life",
            signal: format!("strain_wave.{id}.damage_rate"),
            motor: Some(motor),
            limit: required_life_h,
            life: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A winch: a gear of ratio 20 and 2e5 N m/rad (5 N m s/rad, efficiency 0.9, mesh friction on; rotor and input
    /// 2e-4 kg m^2) behind a drum of 25 mm radius that weighs 0.32 kg at its radius. At the cable the gear is
    /// 2e5 / 0.025^2 = 3.2e8 N/m and the rotor 2e-4 * 800^2 = 128 kg: capped by the rotor alone (`apply`) the bristle is
    /// the full 3.2e8 N/m, by the drum (`apply_to`) 0.32 kg * (BRISTLE_RATE_MAX / 4)^2 = 2e8 N/m. Driven by a torque of
    /// sin(2 pi 2 t) N m for a second at the base step's quarter (15.625 us, RK4), reversing four times, it stays
    /// bounded, and its energy closes: the motor's work is the change of the rotor's, the drum's and the gear's energy and
    /// the gear's loss (0.0636 J against 252 J that pass back and forth), to 1e-6 of the loss (seen: 4e-11 J).
    #[test]
    fn a_gear_at_the_bristle_cap_of_a_light_drum_stays_stable_and_closes_its_energy() {
        let text = "format_version = 1\n[effects]\njoint_friction = false\nhard_stops = false\nmotor_friction = false\n\
            torque_ripple = false\ngearbox_backlash = false\ngearbox_efficiency = true\ngearbox_drag = false\n\
            motor_stiction = false\ngearbox_lost_motion = false\ngearbox_stiffness_curve = false\nthermal_feedback = false\n\
            holding_brake = false\ngearbox_mesh_friction = true\n\
            [motor]\nrotor_inertia_kgm2 = 1e-4\nfriction_coulomb_nm = 0.0\nfriction_viscous_nm_s_rad = 0.0\n\
            friction_smoothing_rad_s = 1e-3\nfriction_breakaway_nm = 0.0\nfriction_stribeck_rad_s = 2.0\nripple_fraction = 0.0\n\
            ripple_order = 6.0\ncogging_nm = 0.0\ncogging_order = 6.0\nbrake_torque_nm = 5.0\n\
            [gearbox]\nratio = 20.0\nefficiency = 0.9\nbacklash_rad = 0.0\nstiffness_nm_rad = 2e5\ndamping_nm_s_rad = 5.0\n\
            inertia_at_input_kgm2 = 1e-4\nefficiency_torque_nm = 100.0\nno_load_torque_nm = 0.0\nno_load_speed_rpm = 3000.0\n\
            lost_motion_band_rad = 0.0\nhysteresis_rad = 0.0\nstiffness_t1_nm = 100.0\nstiffness_t2_nm = 100.0\n\
            stiffness_k2_nm_rad = 2e5\nstiffness_k3_nm_rad = 2e5\n\
            [joint]\nfriction_coulomb_nm = 0.0\nfriction_viscous_nm_s_rad = 0.0\nfriction_smoothing_rad_s = 1e-3\n\
            min_angle_deg = -45.0\nmax_angle_deg = 90.0\nstop_stiffness_nm_rad = 1e5\nstop_damping_nm_s_rad = 0.0\n";
        let cfg = Config::parse(text).unwrap();
        let drum = Converter {
            lever: 0.025,
            stiffness: f64::INFINITY,
            backlash: 0.0,
            efficiency: 1.0,
            inertia: 0.0,
            damping_ratio: 0.0,
        };
        let mass = 0.32;
        let joint = |output: f64| {
            let mut j = RotaryJoint::from_config(&cfg).unwrap();
            drum.apply_to(&cfg, &mut j, output).unwrap();
            j
        };
        assert!((joint(f64::INFINITY).gearbox.mesh_stiffness / 3.2e8 - 1.0).abs() < 1e-12);
        let mut j = joint(mass);
        let cap = mass * (BRISTLE_RATE_MAX / 4.0).powi(2);
        assert!((j.gearbox.mesh_stiffness / cap - 1.0).abs() < 1e-12 && (cap / 2e8 - 1.0).abs() < 1e-12);
        let command = |t: f64| (std::f64::consts::TAU * 2.0 * t).sin();
        let rates = |j: &RotaryJoint, s: [f64; 4], t: f64| {
            let (gear, acc, _) = j.rotor(command(t), s[0], s[1], s[2], s[3], 0.0, false);
            [s[1], acc, s[3], (gear.output + j.output_torque(s[2], s[3])) / mass]
        };
        let energy = |j: &RotaryJoint, s: [f64; 4]| {
            0.5 * j.rotor_inertia * s[1] * s[1] + 0.5 * mass * s[3] * s[3] + j.gearbox.energy(s[0], s[2])
        };
        let power =
            |j: &RotaryJoint, s: [f64; 4], t: f64| (command(t) * s[1], j.gearbox.loss_power(s[0], s[1], s[2], s[3]));
        let (dt, mut s, mut t) = (15.625e-6, [0.0; 4], 0.0);
        let (start, mut work, mut through, mut lost, mut peak) = (energy(&j, s), 0.0, 0.0, 0.0, 0.0f64);
        for _ in 0..64000 {
            let add = |s: [f64; 4], k: [f64; 4], h: f64| std::array::from_fn(|i| s[i] + h * k[i]);
            let k1 = rates(&j, s, t);
            let k2 = rates(&j, add(s, k1, 0.5 * dt), t + 0.5 * dt);
            let k3 = rates(&j, add(s, k2, 0.5 * dt), t + 0.5 * dt);
            let k4 = rates(&j, add(s, k3, dt), t + dt);
            let (p0, l0) = power(&j, s, t);
            s = std::array::from_fn(|i| s[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]));
            t += dt;
            let (p1, l1) = power(&j, s, t);
            (work, through, lost) =
                (work + 0.5 * dt * (p0 + p1), through + 0.5 * dt * (p0.abs() + p1.abs()), lost + 0.5 * dt * (l0 + l1));
            j.gearbox.commit(s[0], s[2]);
            peak = peak.max(s[3].abs());
        }
        let off = work - (energy(&j, s) - start) - lost;
        println!(
            "peak speed {peak:.4} m/s, work {work:.6} J, lost {lost:.6} J, off {off:.3e} J of {through:.6} J through"
        );
        assert!(s.iter().all(|x| x.is_finite()) && peak > 0.1 && peak < 2.0, "{s:?}, peak {peak} m/s");
        assert!(lost > 0.0 && off.abs() < 1e-6 * lost, "off by {off} J, lost {lost} J of {through} J through");
    }

    fn screw() -> BallScrew {
        BallScrew {
            lead: 0.01,
            nominal_diameter: 0.025,
            root_diameter: 0.021,
            length: 1.0,
            mounting: Mounting::FixedSupported,
            efficiency: 0.9,
            back_efficiency: 0.9,
            nut_stiffness: 5e8,
            bearing_stiffness: 1e9,
            preload: 500.0,
            dynamic_rating: 15000.0,
            static_rating: 30000.0,
            load_factor: 1.2,
            static_safety: 2.0,
            speed_factor: 0.8,
            buckling_safety: 2.0,
            max_dn: 70000.0,
            axial_damping_ratio: 0.0,
            fixed_bearing_at: None,
        }
    }

    fn close(a: f64, b: f64, what: &str) {
        assert!((a / b - 1.0).abs() < 1e-4, "{what}: {a} against {b} by hand");
    }

    /// By hand, d = 21 mm, L = 1 m, steel E = 206 GPa, rho = 7850 kg/m^3:
    /// I = pi 0.021^4 / 64 = 9.5466e-9 m^4, A = pi 0.021^2 / 4 = 3.4636e-4 m^2, sqrt(E I / (rho A)) = sqrt(206e9 9.5466e-9
    /// / (7850 3.4636e-4)) = 26.894 m^2/s; fixed-supported (3.9266)^2 = 15.418: omega = 414.66 rad/s, 3959.7 r/min.
    /// Buckling: 2.046 pi^2 206e9 9.5466e-9 / 1 = 39 711 N. Shaft inertia pi 7850 1 0.025^4 / 32 = 3.0104e-4 kg m^2.
    /// Stiffness at 0.5 m: shaft A E / x = 1.4270e8 N/m; with 5e8 and 1e9 in series: 1 / (7.0077e-9 + 2e-9 + 1e-9) = 9.9923e7.
    /// Preload drag: tan beta = 0.01 / (pi 0.025) = 0.12732, K = 0.05 / 0.35682 = 0.14013, 0.14013 500 0.01 / 2 pi = 0.11151 N m.
    #[test]
    fn a_ball_screw_has_the_hand_calculated_limits() {
        let s = screw();
        close(s.ratio(), 628.3185, "ratio");
        close(s.critical_speed_rpm(), 3959.7, "critical speed");
        close(s.allowed_speed_rpm(), 0.8 * 3959.7, "allowed speed");
        close(s.buckling_load(), 39711.0, "buckling");
        close(s.allowed_buckling_force(), 19855.5, "buckling allowed");
        close(s.allowed_static_force(), 15000.0, "static");
        close(s.inertia(), 3.0104e-4, "inertia");
        close(s.stiffness(0.5), 9.9923e7, "stiffness");
        close(s.preload_torque(), 0.11151, "preload drag");
        close(s.dn(2000.0), 50000.0, "dn");
    }

    /// THK's rolling efficiency at mu = 0.003. The test screw: tan beta = 0.01 / (pi 0.025) = 0.127324,
    /// (1 - 0.003 0.127324) / (1 + 0.003 / 0.127324) = 0.999618 / 1.023562 = 0.976607. The ISW bench's 40 x 40:
    /// tan beta = 0.318310, 0.999045 / 1.009425 = 0.989717. From a config: the switch off keeps screw.efficiency to the
    /// bit, on takes the lead's.
    #[test]
    fn the_screw_efficiency_follows_the_lead_angle_with_its_switch() {
        close(BallScrew::lead_efficiency(0.01, 0.025, 0.003), 0.976607, "test screw");
        close(BallScrew::lead_efficiency(0.04, 0.04, 0.003), 0.989717, "ISW screw");
        let text = |on: bool| {
            format!(
                "format_version = 1\n[effects]\nscrew_lead_efficiency = {on}\n[screw]\nlead_m = 0.04\nnominal_diameter_m = 0.04\n\
                 root_diameter_m = 0.0337\nlength_m = 1.8\nmounting = \"fixed-supported\"\nefficiency = 0.9\n\
                 friction_coefficient = 0.003\nnut_stiffness_n_m = 5e8\nbearing_stiffness_n_m = 1e9\npreload_n = 500.0\n\
                 dynamic_load_rating_n = 15000.0\nstatic_load_rating_n = 30000.0\nload_factor = 1.2\n\
                 static_safety_factor = 2.0\ncritical_speed_factor = 0.8\nbuckling_safety_factor = 2.0\nmax_dn = 0.0\n"
            )
        };
        let screw = |on| BallScrew::from_config(&Config::parse(&text(on)).unwrap()).unwrap();
        assert_eq!(screw(false).efficiency, 0.9);
        close(screw(true).efficiency, 0.989717, "from the config");
    }

    /// Preload 500 N lifts off at 1415 N. At 1000 N: 500 (1 + 1000 / 1415)^1.5 = 500 1.70671^1.5 = 1114.8 N; beyond, the
    /// force itself. At 1000 r/min and 2000 N: 60 1000 (1.2 2000 / 15000)^3 / 1e6 = 2.4576e-4 per hour, 4069 h.
    #[test]
    fn the_screw_life_takes_preload_and_load_factor() {
        let s = screw();
        close(s.equivalent_load(1000.0), 1114.8, "preloaded");
        close(s.equivalent_load(-2000.0), 2000.0, "lifted");
        close(s.damage_rate(2000.0, -1000.0), 2.4576e-4, "damage");
        assert_eq!(BallScrew { preload: 0.0, ..s }.equivalent_load(300.0), 300.0);
    }

    /// Module 2 mm, 30 teeth, 19.5283 degrees helix: r = 0.002 30 / (2 cos 19.5283) = 31.831 mm (100 mm travel per turn).
    #[test]
    fn a_helical_pinion_has_its_pitch_radius() {
        let r = RackPinion {
            module: 0.002,
            teeth: 30.0,
            helix_angle: 19.5283f64.to_radians(),
            efficiency: 0.97,
            stiffness: 1e8,
            backlash: 0.0,
            pinion_inertia: 1e-4,
            rated_force: 3000.0,
            service_factor: 1.5,
            pressure_angle: 20f64.to_radians(),
            damping_ratio: 0.0,
        };
        close(r.radius() * 2.0 * PI, 0.2, "travel per turn");
        close(r.allowed_force(), 2000.0, "allowed force");
    }

    /// gamma 5 degrees, mu 0.05, alpha 20 degrees: rho' = atan(0.05 / 0.93969) = 3.0459 degrees; tan 5 / tan 8.0459 =
    /// 0.087489 / 0.14136 = 0.61891; back tan 1.9541 / tan 5 = 0.034118 / 0.087489 = 0.38997. At 2 degrees the gear locks.
    #[test]
    fn a_worm_gear_has_the_hand_calculated_efficiencies() {
        let w = WormGear { lead_angle: 5f64.to_radians(), friction: 0.05, pressure_angle: 20f64.to_radians() };
        close(w.efficiency(), 0.61891, "forward");
        close(w.back_efficiency(), 0.38997, "back");
        assert!(WormGear { lead_angle: 2f64.to_radians(), ..w }.back_efficiency() < 0.0);
    }

    /// Tr 50 N m at 2000 r/min, Ln 7000 h: at 100 N m and 1000 r/min 0.5 8 / 7000 = 5.7143e-4 per hour (1750 h).
    #[test]
    fn a_strain_wave_gear_wears_by_the_cube_of_its_torque() {
        let g = StrainWave { rated_torque: 50.0, rated_speed: 2000.0, rated_life: 7000.0 };
        close(g.damage_rate(-100.0, 1000.0), 5.7143e-4, "damage");
    }

    #[test]
    fn a_linear_motor_cogs_once_per_pole_pitch() {
        close(LinearMotor { attraction: 0.0, pole_pitch: 0.024 }.cogging_order(), 261.799, "order");
    }
    /// At 0.5 m/s and 2000 N on the 10 mm lead: 3000 r/min, d n 25 * 3000 = 75000, 60 * 3000 * (1.2 * 2000 / 15000)^3 /
    /// 1e6 = 7.3728e-4 per hour. The checks' limits are those of the hand-calculated test above; life only where
    /// required, d n only where limited.
    #[test]
    fn the_screw_reports_and_checks_by_hand() {
        let s = screw();
        let mut out = Vec::new();
        s.signals(0.5, 2000.0, &mut out);
        close(out[0], 3000.0, "rpm");
        close(out[1], 75000.0, "dn");
        close(out[2], 7.3728e-4, "damage");
        assert_eq!(
            s.signal_infos("3", "joint-3").iter().map(|i| i.id.as_str()).collect::<Vec<_>>(),
            ["screw.3.speed", "screw.3.dn", "screw.3.damage_rate"]
        );
        let checks = s.rating_checks("3", 2, Some(20000.0));
        let limit =
            |id: &str| checks.iter().find(|c| c.check == id).map(|c| (c.limit, c.signal.clone(), c.life, c.motor));
        assert_eq!(limit("screw-life"), Some((20000.0, "screw.3.damage_rate".into(), true, Some(2))));
        close(limit("screw-speed").unwrap().0, 0.8 * 3959.7, "speed limit");
        close(limit("screw-buckling").unwrap().0, 19855.5, "buckling limit");
        assert_eq!(limit("screw-static").unwrap().1, "joint.3.force");
        assert_eq!(limit("screw-dn").unwrap().0, 70000.0);
        let plain = BallScrew { max_dn: 0.0, ..s }.rating_checks("3", 2, None);
        assert!(plain.iter().all(|c| c.check != "screw-dn" && c.check != "screw-life"));
        for c in &checks {
            assert_eq!(c.life, crate::analysis::AT_LEAST.contains(&c.check), "{}", c.check);
        }
    }

    /// 100 N m at 1000 r/min in (104.72 rad/s): 5.7143e-4 per hour, as by the catalogue's formula above.
    #[test]
    fn the_strain_wave_gear_reports_its_wear_from_the_input_speed() {
        let g = StrainWave { rated_torque: 50.0, rated_speed: 2000.0, rated_life: 7000.0 };
        close(g.signal(100.0, 1000.0 * 2.0 * PI / 60.0), 5.7143e-4, "damage");
        let c = g.rating_check("2", 1, 15000.0);
        assert!(
            c.life && c.check == "strain-wave-life" && c.signal == "strain_wave.2.damage_rate" && c.limit == 15000.0
        );
    }

    #[test]
    fn a_rack_is_checked_against_its_feed_force_over_the_service_factor() {
        let r = RackPinion {
            module: 0.002,
            teeth: 30.0,
            helix_angle: 0.0,
            efficiency: 0.97,
            stiffness: 1e8,
            backlash: 0.0,
            pinion_inertia: 1e-4,
            rated_force: 3000.0,
            service_factor: 1.25,
            pressure_angle: 20f64.to_radians(),
            damping_ratio: 0.0,
        };
        let c = &r.rating_checks("1", 0)[0];
        assert!(c.check == "rack-feed-force" && c.signal == "rack.1.feed_force" && c.limit == 2400.0 && !c.life);
        // 970 N to the load at 97 %: 1000 N on the teeth driving, 940.9 N braking.
        close(RackPinion::feed_force(970.0, 1.0, 0.97), 1000.0, "driving");
        close(RackPinion::feed_force(970.0, -1.0, 0.97), 940.9, "braking");
        // Straight teeth: r = m z / 2 = 30 mm.
        close(r.converter().lever, 0.03, "lever");
    }

    /// THK's back-driving efficiency (1 - mu / tan beta) / (1 + mu tan beta) at mu = 0.003. The ISW bench (tan beta =
    /// 0.318310): 0.9905752 / 1.0009549 = 0.9896302 against 0.9897172 driving (8.7e-5 apart); at a lead angle of 7 degrees
    /// (tan beta = 0.122785): 0.975567 / 1.000368 = 0.975208 against 0.975790 (6e-4). tan beta = mu locks: 0.
    #[test]
    fn the_screw_driven_back_loses_by_thks_back_efficiency_and_a_locking_one_is_refused() {
        assert!((BallScrew::lead_back_efficiency(0.04, 0.04, 0.003) - 0.989_630_2).abs() < 1e-7);
        let lead7 = 7f64.to_radians().tan() * PI * 0.04;
        assert!((BallScrew::lead_efficiency(lead7, 0.04, 0.003) - 0.975_790_1).abs() < 1e-7);
        assert!((BallScrew::lead_back_efficiency(lead7, 0.04, 0.003) - 0.975_207_7).abs() < 1e-7);
        assert!(BallScrew::lead_back_efficiency(0.003 * PI * 0.04, 0.04, 0.003).abs() < 1e-12);
        let text = |mu: f64| {
            format!(
                "format_version = 1\n[effects]\nscrew_lead_efficiency = true\n[screw]\nlead_m = 0.04\nnominal_diameter_m = 0.04\n\
                 root_diameter_m = 0.0337\nlength_m = 1.8\nmounting = \"fixed-supported\"\nefficiency = 0.9\n\
                 friction_coefficient = {mu}\nnut_stiffness_n_m = 5e8\nbearing_stiffness_n_m = 1e9\npreload_n = 500.0\n\
                 dynamic_load_rating_n = 15000.0\nstatic_load_rating_n = 30000.0\nload_factor = 1.2\n\
                 static_safety_factor = 2.0\ncritical_speed_factor = 0.8\nbuckling_safety_factor = 2.0\nmax_dn = 0.0\n"
            )
        };
        let screw = BallScrew::from_config(&Config::parse(&text(0.003)).unwrap()).unwrap();
        assert!((screw.back_efficiency - 0.989_630_2).abs() < 1e-7);
        // mu 0.4 over tan beta 0.318: the screw holds its load.
        let err = BallScrew::from_config(&Config::parse(&text(0.4)).unwrap()).unwrap_err();
        assert!(format!("{err:?}").contains("locks itself"), "{err:?}");
    }

    /// The test gearbox of the winch (efficiency 0.9, mesh friction off), behind a converter of efficiency 0.95 that loses
    /// 0.9 driven back: driving it loses 0.9 x 0.95 = 0.855, driven back the gear's own 1 / 0.9 and the converter's
    /// 1 / 0.9: 1 / 0.81 = 1.234568 (a gear that loses alike both ways would give 1 / 0.855 = 1.169591), and the loss power
    /// of a steady twist of 100 N m at 2 m/s backwards is 200 (1 / 0.81 - 1) = 46.9136 W against 200 (1 / 0.855 - 1) =
    /// 33.9181 W alike, whatever the rotor's side of the budget (`Gearbox::loss_power`).
    #[test]
    fn a_converter_with_its_own_back_efficiency_loses_so_driven_back_and_the_budget_has_it() {
        let text = "format_version = 1\n[effects]\njoint_friction = false\nhard_stops = false\nmotor_friction = false\n\
            torque_ripple = false\ngearbox_backlash = false\ngearbox_efficiency = true\ngearbox_drag = false\n\
            motor_stiction = false\ngearbox_lost_motion = false\ngearbox_stiffness_curve = false\nthermal_feedback = false\n\
            holding_brake = false\ngearbox_mesh_friction = false\n\
            [motor]\nrotor_inertia_kgm2 = 1e-4\nfriction_coulomb_nm = 0.0\nfriction_viscous_nm_s_rad = 0.0\n\
            friction_smoothing_rad_s = 1e-3\nfriction_breakaway_nm = 0.0\nfriction_stribeck_rad_s = 2.0\nripple_fraction = 0.0\n\
            ripple_order = 6.0\ncogging_nm = 0.0\ncogging_order = 6.0\nbrake_torque_nm = 5.0\n\
            [gearbox]\nratio = 20.0\nefficiency = 0.9\nbacklash_rad = 0.0\nstiffness_nm_rad = 2e5\ndamping_nm_s_rad = 5.0\n\
            inertia_at_input_kgm2 = 1e-4\nefficiency_torque_nm = 100.0\nno_load_torque_nm = 0.0\nno_load_speed_rpm = 3000.0\n\
            lost_motion_band_rad = 0.0\nhysteresis_rad = 0.0\nstiffness_t1_nm = 100.0\nstiffness_t2_nm = 100.0\n\
            stiffness_k2_nm_rad = 2e5\nstiffness_k3_nm_rad = 2e5\n\
            [joint]\nfriction_coulomb_nm = 0.0\nfriction_viscous_nm_s_rad = 0.0\nfriction_smoothing_rad_s = 1e-3\n\
            min_angle_deg = -45.0\nmax_angle_deg = 90.0\nstop_stiffness_nm_rad = 1e5\nstop_damping_nm_s_rad = 0.0\n";
        let cfg = Config::parse(text).unwrap();
        let converter = Converter {
            lever: 1.0,
            stiffness: f64::INFINITY,
            backlash: 0.0,
            efficiency: 0.95,
            inertia: 0.0,
            damping_ratio: 0.0,
        };
        let make = |back: f64| {
            let mut j = RotaryJoint::from_config(&cfg).unwrap();
            converter.apply_back(&cfg, &mut j, f64::INFINITY, back).unwrap();
            j.gearbox
        };
        let (own, alike) = (make(0.9), make(0.95));
        assert!(
            (own.loss_factor(1.0, 1e3) - 0.855).abs() < 1e-12 && (alike.loss_factor(1.0, 1e3) - 0.855).abs() < 1e-12
        );
        assert!((own.loss_factor(1.0, -1e3) - 1.0 / 0.81).abs() < 1e-12, "{}", own.loss_factor(1.0, -1e3));
        assert!((alike.loss_factor(1.0, -1e3) - 1.0 / 0.855).abs() < 1e-12);
        // A twist of 0.0005 rad at 2e5 N m/rad holds 100 N m at the output; the output runs back at 2 rad/s, the rotor with
        // it (twist constant).
        let (ratio, out) = (own.ratio, -0.3);
        let loss = |g: &crate::mech::Gearbox| g.loss_power(ratio * (out + 5e-4), -2.0 * ratio, out, -2.0);
        assert!((loss(&own) - 200.0 * (1.0 / 0.81 - 1.0)).abs() < 1e-6, "{}", loss(&own));
        assert!((loss(&alike) - 200.0 * (1.0 / 0.855 - 1.0)).abs() < 1e-6, "{}", loss(&alike));
    }

    /// The hysteresis loop's and the mesh friction's presliding stiffness are factors on K1, 1 where the key is absent
    /// (the mesh one with the converted stiffness, as the K1 itself: 2e5 / 0.025^2 = 3.2e8 N/m behind the drum).
    /// `hysteresis_stiffness_factor = 3`: 3 x 3.2e8 = 9.6e8 N/m; `mesh_stiffness_factor = 0.5`: 1.6e8 N/m.
    #[test]
    fn the_presliding_stiffnesses_are_factors_on_k1_that_default_to_one() {
        let base = "format_version = 1\n[effects]\njoint_friction = false\nhard_stops = false\nmotor_friction = false\n\
            torque_ripple = false\ngearbox_backlash = false\ngearbox_efficiency = true\ngearbox_drag = false\n\
            motor_stiction = false\ngearbox_lost_motion = false\ngearbox_stiffness_curve = false\nthermal_feedback = false\n\
            holding_brake = false\ngearbox_mesh_friction = true\n\
            [motor]\nrotor_inertia_kgm2 = 1e-4\nfriction_coulomb_nm = 0.0\nfriction_viscous_nm_s_rad = 0.0\n\
            friction_smoothing_rad_s = 1e-3\nfriction_breakaway_nm = 0.0\nfriction_stribeck_rad_s = 2.0\nripple_fraction = 0.0\n\
            ripple_order = 6.0\ncogging_nm = 0.0\ncogging_order = 6.0\nbrake_torque_nm = 5.0\n\
            [gearbox]\nratio = 20.0\nefficiency = 0.9\nbacklash_rad = 0.0\nstiffness_nm_rad = 2e5\ndamping_nm_s_rad = 5.0\n\
            inertia_at_input_kgm2 = 1e-4\nefficiency_torque_nm = 100.0\nno_load_torque_nm = 0.0\nno_load_speed_rpm = 3000.0\n\
            lost_motion_band_rad = 0.0\nhysteresis_rad = 0.0\nstiffness_t1_nm = 100.0\nstiffness_t2_nm = 100.0\n\
            stiffness_k2_nm_rad = 2e5\nstiffness_k3_nm_rad = 2e5\n{extra}\
            [joint]\nfriction_coulomb_nm = 0.0\nfriction_viscous_nm_s_rad = 0.0\nfriction_smoothing_rad_s = 1e-3\n\
            min_angle_deg = -45.0\nmax_angle_deg = 90.0\nstop_stiffness_nm_rad = 1e5\nstop_damping_nm_s_rad = 0.0\n";
        let drum = Converter {
            lever: 0.025,
            stiffness: f64::INFINITY,
            backlash: 0.0,
            efficiency: 1.0,
            inertia: 0.0,
            damping_ratio: 0.0,
        };
        let make = |extra: &str| {
            let cfg = Config::parse(&base.replace("{extra}", extra)).unwrap();
            let mut j = RotaryJoint::from_config(&cfg).unwrap();
            drum.apply(&cfg, &mut j).unwrap();
            j.gearbox
        };
        let plain = make("");
        assert!(
            (plain.dahl_stiffness / 3.2e8 - 1.0).abs() < 1e-12 && (plain.mesh_stiffness / 3.2e8 - 1.0).abs() < 1e-12
        );
        let set = make("hysteresis_stiffness_factor = 3.0\nmesh_stiffness_factor = 0.5\n");
        assert!((set.dahl_stiffness / 9.6e8 - 1.0).abs() < 1e-12, "{}", set.dahl_stiffness);
        assert!((set.mesh_stiffness / 1.6e8 - 1.0).abs() < 1e-12, "{}", set.mesh_stiffness);
    }

    /// The gear behind a converter is the gear and the converter's spring in series, band and hysteresis too. K1 = 2e5
    /// N m/rad behind a lever of 0.025 m/rad (3.2e8 N/m) and a spring of 5e7 N/m: K1' = 1 / (3.125e-9 + 2e-8) =
    /// 4.324324e7 N/m. A band of 1e-4 rad: the flanks leave it at T_b = 10 N m (400 N); at 100 N (2.5 N m) they twist
    /// sqrt(2 x 1e-4 x 2.5 / 2e5) = 5e-5 rad = 1.25e-6 m and the spring 2e-6 m: 3.25e-6 m; at 800 N (20 N m) 1e-4 +
    /// 5e-5 rad = 3.75e-6 m and 1.6e-5 m: 1.975e-5 m, an offset of the line of 1.25e-6 m, the gear's 5e-5 rad x lever.
    /// hysteresis_rad 2e-4: T_h = 20 N m; sliding, the force at the joint is K1' (x -+ lever T_h / K1), so the friction
    /// is 4.324324e7 x 0.025 x 20 / 2e5 = 108.108 N (not 20 / 0.025 = 800 N), and the loop at zero force 2 x 108.108 /
    /// K1' = 5e-6 m wide, the gear's 2 T_h / K1 x lever; sigma' = series(4e5) - K1' = 1 / (1.5625e-9 + 2e-8) - K1' =
    /// 3.133595e6 N/m. No band, none.
    #[test]
    fn the_gear_behind_a_converter_is_in_series_with_its_spring_over_the_band_and_the_hysteresis() {
        let text = "format_version = 1\n[effects]\njoint_friction = false\nhard_stops = false\nmotor_friction = false\n\
            torque_ripple = false\ngearbox_backlash = false\ngearbox_efficiency = false\ngearbox_drag = false\n\
            motor_stiction = false\ngearbox_lost_motion = true\ngearbox_stiffness_curve = false\nthermal_feedback = false\n\
            holding_brake = false\ngearbox_mesh_friction = false\n\
            [motor]\nrotor_inertia_kgm2 = 1e-4\nfriction_coulomb_nm = 0.0\nfriction_viscous_nm_s_rad = 0.0\n\
            friction_smoothing_rad_s = 1e-3\nfriction_breakaway_nm = 0.0\nfriction_stribeck_rad_s = 2.0\nripple_fraction = 0.0\n\
            ripple_order = 6.0\ncogging_nm = 0.0\ncogging_order = 6.0\nbrake_torque_nm = 5.0\n\
            [gearbox]\nratio = 20.0\nefficiency = 0.9\nbacklash_rad = 0.0\nstiffness_nm_rad = 2e5\ndamping_nm_s_rad = 5.0\n\
            inertia_at_input_kgm2 = 1e-4\nefficiency_torque_nm = 100.0\nno_load_torque_nm = 0.0\nno_load_speed_rpm = 3000.0\n\
            lost_motion_band_rad = BAND\nhysteresis_rad = 2e-4\nstiffness_t1_nm = 100.0\nstiffness_t2_nm = 100.0\n\
            stiffness_k2_nm_rad = 2e5\nstiffness_k3_nm_rad = 2e5\n\
            [joint]\nfriction_coulomb_nm = 0.0\nfriction_viscous_nm_s_rad = 0.0\nfriction_smoothing_rad_s = 1e-3\n\
            min_angle_deg = -45.0\nmax_angle_deg = 90.0\nstop_stiffness_nm_rad = 1e5\nstop_damping_nm_s_rad = 0.0\n";
        let screw = Converter {
            lever: 0.025,
            stiffness: 5e7,
            backlash: 0.0,
            efficiency: 1.0,
            inertia: 0.0,
            damping_ratio: 0.0,
        };
        let gear = |b: &str| {
            let cfg = Config::parse(&text.replace("BAND", b)).unwrap();
            let mut j = RotaryJoint::from_config(&cfg).unwrap();
            screw.apply(&cfg, &mut j).unwrap();
            j.gearbox
        };
        let g = gear("1e-4");
        let rel = |a: f64, b: f64| (a / b - 1.0).abs() < 1e-12;
        assert!(rel(g.stiffness, 1.0 / 2.3125e-8) && rel(g.band, 2.5e-6), "{} {}", g.stiffness, g.band);
        for (force, twist) in [(100.0, 3.25e-6), (400.0, 2.5e-6 + 8e-6), (800.0, 1.975e-5)] {
            assert!(rel(g.static_twist(force), twist), "{force}: {}", g.static_twist(force));
            assert!(rel(g.spring(twist).0, force), "{twist}: {}", g.spring(twist).0);
        }
        assert!(rel(g.hysteresis, 108.108_108_108_108_1) && rel(2.0 * g.hysteresis / g.stiffness, 5e-6));
        assert!(rel(g.dahl_stiffness, 1.0 / 2.15625e-8 - 1.0 / 2.3125e-8), "{}", g.dahl_stiffness);
        assert_eq!(gear("0.0").band, 0.0);
    }

    /// The gear's damper bridges its own spring, not the converter's (a Zener solid, `Gearbox::arm`). A gear of
    /// K1 = 100 N m/rad and 0.2828 N m s/rad behind a lever of 0.01 m/rad is K_g = 1e6 N/m and c = 2828 N s/m at the
    /// joint, a converter of K_c = 1e6 N/m in series: K_s = 5e5 N/m at rest (the arm then carries nothing). A mass of
    /// 1 kg on it, the rotor held, rings as the roots of m c s^3 + m (K_c + K_g) s^2 + K_c c s + K_c K_g = 0 say:
    /// s = -152.099260 +- 924.266861i and -403.015059 (numpy.roots), so successive peaks fall by
    /// exp(-2 pi 152.099260 / 924.266861) = 0.355591 (log decrement 1.033974), the real root's share gone after a few
    /// periods. With the damper across the whole series spring (as the converter had it) zeta would be
    /// 2828 / (2 sqrt(5e5)) = 2.0: no ring at all. The energy closes: the mass's and the springs' (the arm's with them)
    /// and the arm's damper's loss stay the start's. Without damping, or behind a rigid converter, there is no arm and
    /// the damping stays as it was, c / lever^2 across the spring; an arm faster than the integrator is its slow limit.
    ///
    /// With the screw's own damper too, zeta = 0.04 on the 1 kg: c_c = 2 x 0.04 sqrt(1e6 x 1) = 80 N s/m across K_c
    /// alone. The pair's m C s^3 + (m S + c_g c_c) s^2 + (K_g c_c + K_c c_g) s + K_g K_c = 0 (S = K_g + K_c, C = c_g +
    /// c_c) has s = -181.415607 +- 906.072216i: peaks fall by 0.284213; b = c_g c_c / C = 77.799175 N s/m beside the
    /// arm's K_m = (K_g c_c - K_c c_g)^2 / (S C^2) = 446493.00 N/m and tau = C / S = 1.454 ms. Behind a near-rigid gear
    /// (K1 = 1e8 N m/rad, no damper) the screw's own mode keeps its 0.04.
    #[test]
    fn the_gear_damper_behind_a_converter_bridges_the_gear_alone() {
        let text = "format_version = 1\n[effects]\njoint_friction = false\nhard_stops = false\nmotor_friction = false\n\
            torque_ripple = false\ngearbox_backlash = false\ngearbox_efficiency = false\ngearbox_drag = false\n\
            motor_stiction = false\ngearbox_lost_motion = false\ngearbox_stiffness_curve = false\nthermal_feedback = false\n\
            holding_brake = false\ngearbox_mesh_friction = false\n\
            [motor]\nrotor_inertia_kgm2 = 1e-4\nfriction_coulomb_nm = 0.0\nfriction_viscous_nm_s_rad = 0.0\n\
            friction_smoothing_rad_s = 1e-3\nfriction_breakaway_nm = 0.0\nfriction_stribeck_rad_s = 2.0\nripple_fraction = 0.0\n\
            ripple_order = 6.0\ncogging_nm = 0.0\ncogging_order = 6.0\nbrake_torque_nm = 5.0\n\
            [gearbox]\nratio = 1.0\nefficiency = 0.9\nbacklash_rad = 0.0\nstiffness_nm_rad = K1\ndamping_nm_s_rad = DAMPING\n\
            inertia_at_input_kgm2 = 1e-4\nefficiency_torque_nm = 100.0\nno_load_torque_nm = 0.0\nno_load_speed_rpm = 3000.0\n\
            lost_motion_band_rad = 0.0\nhysteresis_rad = 0.0\nstiffness_t1_nm = 100.0\nstiffness_t2_nm = 100.0\n\
            stiffness_k2_nm_rad = 100.0\nstiffness_k3_nm_rad = 100.0\n\
            [joint]\nfriction_coulomb_nm = 0.0\nfriction_viscous_nm_s_rad = 0.0\nfriction_smoothing_rad_s = 1e-3\n\
            min_angle_deg = -45.0\nmax_angle_deg = 90.0\nstop_stiffness_nm_rad = 1e5\nstop_damping_nm_s_rad = 0.0\n";
        let geared = |k1: &str, damping: &str, stiffness: f64, damping_ratio: f64| {
            let cfg = Config::parse(&text.replace("K1", k1).replace("DAMPING", damping)).unwrap();
            let mut j = RotaryJoint::from_config(&cfg).unwrap();
            let converter =
                Converter { lever: 0.01, stiffness, backlash: 0.0, efficiency: 1.0, inertia: 0.0, damping_ratio };
            converter.apply_to(&cfg, &mut j, 1.0).unwrap();
            j.gearbox
        };
        let gear = |damping: &str, stiffness: f64| geared("100.0", damping, stiffness, 0.0);
        let rel = |a: f64, b: f64, tol: f64| (a / b - 1.0).abs() < tol;
        // The limits: no damping, no arm (the series spring alone); a rigid converter, the damper across the gear.
        let plain = gear("0.0", 1e6);
        assert!(plain.relax_stiffness == 0.0 && plain.damping == 0.0 && rel(plain.stiffness, 5e5, 1e-12));
        let rigid = gear("0.2828", f64::INFINITY);
        assert!(rigid.relax_stiffness == 0.0 && rel(rigid.damping, 2828.0, 1e-12) && rel(rigid.stiffness, 1e6, 1e-12));
        // 0.001 N m s/rad, c = 10 N s/m: tau = 10 / 2e6 = 5 us, faster than the integrator follows; its slow limit is
        // a damper of c (K_c / (K_c + K_g))^2 = 2.5 N s/m across the series spring.
        let fast = gear("0.001", 1e6);
        assert!(fast.relax_stiffness == 0.0 && rel(fast.damping, 2.5, 1e-12), "{}", fast.damping);
        let g = gear("0.2828", 1e6);
        assert_eq!(g.damping, 0.0);
        assert!(rel(g.relax_stiffness, 5e5, 1e-12) && rel(g.relax_time, 1.414e-3, 1e-12));
        // At rest, the series stiffness: the arm relaxed carries nothing.
        let x = 1e-3;
        assert!(rel(g.static_twist(500.0), 1e-3, 1e-12));
        assert!(rel(g.torque_with(0.0, 0.0, -x, 0.0, g.arm_rest(0.0, -x)).output, 500.0, 1e-12));
        // Free decay from there, the rotor held: state [out, speed, arm]. The fall of the peaks after the real root's
        // share is gone, and what the energy misses.
        let decay = |g: &crate::mech::Gearbox| {
            let (mass, dt) = (1.0, 1e-6);
            let rates = |s: [f64; 3]| {
                let force = g.torque_with(0.0, 0.0, s[0], s[1], s[2]).output;
                [s[1], force / mass, g.arm(0.0, s[0], s[2]).1]
            };
            let energy = |s: [f64; 3]| 0.5 * mass * s[1] * s[1] + g.energy_with(0.0, s[0], s[2]);
            let u0 = g.arm_rest(0.0, -x);
            let (mut s, mut lost, start) = ([-x, 0.0, u0], 0.0, energy([-x, 0.0, u0]));
            let mut peaks = Vec::new();
            for _ in 0..80000 {
                let add = |s: [f64; 3], k: [f64; 3], h: f64| std::array::from_fn(|i| s[i] + h * k[i]);
                let k1 = rates(s);
                let k2 = rates(add(s, k1, 0.5 * dt));
                let k3 = rates(add(s, k2, 0.5 * dt));
                let k4 = rates(add(s, k3, dt));
                let l0 = g.loss_power_with(0.0, 0.0, s[0], s[1], s[2]);
                let next: [f64; 3] =
                    std::array::from_fn(|i| s[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]));
                lost += 0.5 * dt * (l0 + g.loss_power_with(0.0, 0.0, next[0], next[1], next[2]));
                if s[1] > 0.0 && next[1] <= 0.0 {
                    peaks.push(next[0]);
                }
                s = next;
            }
            assert!(peaks.len() >= 10, "{}", peaks.len());
            println!("peaks {peaks:?}");
            (peaks[8] / peaks[7], (start - energy(s) - lost) / start)
        };
        let (fall, off) = decay(&g);
        assert!(rel(fall, 0.355591, 2e-5) && off.abs() < 1e-9, "{fall} {off}");
        let both = geared("100.0", "0.2828", 1e6, 0.04);
        assert!(rel(both.damping, 77.799175, 1e-7) && rel(both.relax_stiffness, 446493.00, 1e-7), "{both:?}");
        assert!(rel(both.relax_time, 1.454e-3, 1e-12));
        let (fall, off) = decay(&both);
        assert!(rel(fall, 0.284213, 2e-5) && off.abs() < 1e-9, "{fall} {off}");
        let screw = geared("1e8", "0.0", 1e6, 0.04);
        let zeta = screw.damping_at(0.0) / (2.0 * screw.stiffness.sqrt());
        assert!(screw.relax_stiffness == 0.0 && rel(zeta, 0.04, 1e-5), "{zeta}");
        // The modes' damping at the ring: K_m tau / (1 + (w tau)^2) at w = sqrt(K_s / m).
        let w = 5e5f64.sqrt();
        assert!(rel(g.damping_at(w), 5e5 * 1.414e-3 / (1.0 + (w * 1.414e-3).powi(2)), 1e-12));
    }

    /// The screw's stiffness is judged at the softest nut position: the far end of a fixed-supported shaft (1 m):
    /// A E / x with A E = 7.1350e7 N, in series with 5e8 and 1e9 N/m: 1 / (1.4015e-8 + 2e-9 + 1e-9) = 5.8770e7 N/m; the
    /// middle of a fixed-fixed one: A E L / (x (L - x)) = 2.8540e8 N/m, 1 / (3.5037e-9 + 3e-9) = 1.5376e8 N/m, which is
    /// the least of the stroke there (the ends are stiffer).
    #[test]
    fn the_screw_converter_takes_the_softest_position_of_either_bearing_arrangement() {
        let free = screw();
        assert_eq!(free.softest_position(), 1.0);
        close(free.converter(free.softest_position()).stiffness, 5.8770e7, "fixed-supported far end");
        assert!((0..=100).all(|i| free.stiffness(0.01 * i as f64) >= free.stiffness(1.0) * (1.0 - 1e-12)));
        let fixed = BallScrew { mounting: Mounting::FixedFixed, ..screw() };
        assert_eq!(fixed.softest_position(), 0.5);
        close(fixed.converter(fixed.softest_position()).stiffness, 1.5376e8, "fixed-fixed middle");
        assert!((1..100).all(|i| fixed.stiffness(0.01 * i as f64) >= fixed.stiffness(0.5) * (1.0 - 1e-12)));
    }

    /// With the stiffness following the nut (`travel`), the shaft's compliance along the travel is the one `stiffness`
    /// puts in series, for either bearing arrangement: with the fixed bearing at travel -0.1, the nut 0.25 from it at
    /// 0.15 has 1 / stiffness(0.25) - 1 / stiffness(1) more than the far end (fixed-supported: 0.25 / A E - 1 / A E =
    /// -1.0512e-8 m/N), and its slope is 1 / A E = 1.4015e-8 m/N per m. The switch and `fixed_bearing_at_m` from a
    /// config; without the switch there is none.
    #[test]
    fn the_screw_compliance_along_the_travel_is_the_stiffness_at_the_nut() {
        for mounting in [Mounting::FixedSupported, Mounting::FixedFixed] {
            let s = BallScrew { mounting, fixed_bearing_at: Some(-0.1), ..screw() };
            let t = s.travel(s.softest_position()).unwrap();
            for x in [0.05, 0.25, 0.5, 0.9] {
                let by_stiffness = 1.0 / s.stiffness(x) - 1.0 / s.stiffness(s.softest_position());
                assert!((t.delta(x - 0.1).0 - by_stiffness).abs() < 1e-20, "{mounting:?} at {x}");
                let h = 1e-6;
                let slope = (t.delta(x - 0.1 + h).0 - t.delta(x - 0.1 - h).0) / (2.0 * h);
                assert!((t.delta(x - 0.1).1 - slope).abs() < 1e-14, "{mounting:?} slope at {x}");
            }
        }
        let t = BallScrew { fixed_bearing_at: Some(-0.1), ..screw() }.travel(1.0).unwrap();
        close(t.delta(0.15).0, -1.0512e-8, "compliance less the far end's");
        close(t.delta(0.15).1, 1.4015e-8, "slope");
        assert!(screw().converter(1.0).travel.is_none());
        let text = |on: bool| {
            format!(
                "format_version = 1\n[effects]\nscrew_stiffness_with_position = {on}\n[screw]\nlead_m = 0.01\n\
                 nominal_diameter_m = 0.025\nroot_diameter_m = 0.021\nlength_m = 1.0\nmounting = \"fixed-supported\"\n\
                 efficiency = 0.9\nnut_stiffness_n_m = 5e8\nbearing_stiffness_n_m = 1e9\npreload_n = 0.0\n\
                 dynamic_load_rating_n = 1e4\nstatic_load_rating_n = 2e4\nload_factor = 1.2\nstatic_safety_factor = 2.0\n\
                 critical_speed_factor = 0.8\nbuckling_safety_factor = 2.0\nmax_dn = 0.0\nfixed_bearing_at_m = -0.1\n"
            )
        };
        let read = |on| BallScrew::from_config(&Config::parse(&text(on)).unwrap()).unwrap().fixed_bearing_at;
        assert_eq!((read(true), read(false)), (Some(-0.1), None));
    }

    /// The pressure angle of the teeth: the force pushing pinion and rack apart is F tan(alpha) (straight teeth), the
    /// radial load F / cos(alpha): 1000 N gives 1064.18 N at the default 20 degrees (as before) and 1103.38 N at 25.
    #[test]
    fn the_racks_pressure_angle_is_a_key_that_defaults_to_twenty_degrees() {
        let text = |extra: &str| {
            format!(
                "format_version = 1\n[rack]\nmodule_m = 0.002\nteeth = 30.0\nhelix_angle_deg = 0.0\nefficiency = 0.97\n\
                 stiffness_n_m = 1e8\nbacklash_m = 0.0\npinion_inertia_kgm2 = 1e-4\nrated_feed_force_n = 3000.0\n\
                 service_factor = 1.5\n{extra}"
            )
        };
        let rack = |extra: &str| RackPinion::from_config(&Config::parse(&text(extra)).unwrap()).unwrap();
        close(rack("").radial_load(1000.0), 1064.18, "default 20 degrees");
        close(rack("pressure_angle_deg = 25.0\n").radial_load(-1000.0), 1103.38, "25 degrees");
        assert!(RackPinion::from_config(&Config::parse(&text("pressure_angle_deg = 60.0\n")).unwrap()).is_err());
    }
}
