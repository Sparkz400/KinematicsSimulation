//! Mechanical components: belt, guide friction, shaft friction, gearbox, torque ripple, tooth meshing, hard stops,
//! cable chain, holding brake.
//! Each reads its own parameters and its own switch from the config and knows no machine.

use sim_core::{Config, ConfigError};

type Result<T> = std::result::Result<T, ConfigError>;

/// The hand-set factor of effects.temperature. With effects.thermal_feedback it is not used: the temperatures then
/// come from `ThermalFeedback`.
fn temperature_scale(cfg: &Config, leaf: &str) -> Result<f64> {
    Ok(if cfg.bool("effects.temperature")? && !cfg.bool("effects.thermal_feedback")? { cfg.positive(&format!("temperature.{leaf}"))? } else { 1.0 })
}

// ---------------------------------------------------------------------------------------------
// Temperature feedback

const KELVIN: f64 = 273.15;

/// With effects.thermal_feedback: what the temperatures of the parts do to friction, gearbox losses and belt
/// stiffness. The config's values hold at `reference` (gearbox makers quote their no-load torque at 20 °C); the
/// machine stands in air at `ambient`, and a part is at ambient plus its rise (effects.part_heating; without it the
/// rises stay 0).
///
/// - Lubricant viscosity (Arrhenius/Andrade): nu(T) / nu_ref = exp(B (1/T - 1/T_ref)), T in K. A mineral oil of
///   viscosity index about 100 has B = 3470 K (ISO VG 32: 32 and 5.4 mm²/s at 40 and 100 °C) to 4730 K (VG 220:
///   220 and 19.4 mm²/s); VG 68 (68 and 8.7) gives 4000 K.
/// - Guide: its viscous coefficient follows the viscosity; Coulomb and stiction levels (rolling contact, seals)
///   stay as they are.
/// - Gearbox: a share s of its loss (1 - eta_ref) is churning and bearing drag, which follows the viscosity; the
///   rest, tooth friction, does not: 1 - eta = (1 - eta_ref) (1 - s + s nu / nu_ref).
/// - Belt: polyurethane loses modulus as it warms, but the steel cords carry EA: EA (1 - alpha (T - T_ref)), alpha a
///   few 1e-4 per K (steel's own modulus falls by about 2.5e-4 per K). The belt is taken at ambient: it has no loss
///   of its own in the model, and a stiffness that changed while running would change its stored energy.
#[derive(Debug, Clone)]
pub struct ThermalFeedback {
    /// K.
    pub ambient: f64,
    pub reference: f64,
    /// B of the viscosity law, K.
    pub viscosity_slope: f64,
    pub gearbox_viscous_share: f64,
    /// alpha, 1/K.
    pub belt_coefficient: f64,
    /// The gearbox's efficiency at the reference temperature (1 with effects.gearbox_efficiency off).
    pub efficiency: f64,
}

impl ThermalFeedback {
    /// `None` with the effect off.
    pub fn from_config(cfg: &Config) -> Result<Option<Self>> {
        if !cfg.bool("effects.thermal_feedback")? {
            return Ok(None);
        }
        let celsius = |leaf: &str| -> Result<f64> {
            let t = cfg.f64(&format!("thermal_feedback.{leaf}"))? + KELVIN;
            if t > 0.0 { Ok(t) } else { Err(ConfigError::Invalid(format!("thermal_feedback.{leaf}"))) }
        };
        let share = cfg.non_negative("thermal_feedback.gearbox_viscous_loss_share")?;
        if share > 1.0 {
            return Err(ConfigError::Invalid("thermal_feedback.gearbox_viscous_loss_share".into()));
        }
        Ok(Some(Self {
            ambient: celsius("ambient_c")?,
            reference: celsius("reference_c")?,
            viscosity_slope: cfg.non_negative("thermal_feedback.viscosity_slope_k")?,
            gearbox_viscous_share: share,
            belt_coefficient: cfg.f64("thermal_feedback.belt_stiffness_coefficient_1_k")?,
            efficiency: Gearbox::configured_efficiency(cfg)?,
        }))
    }

    /// nu / nu_ref of a lubricant `rise` above ambient.
    pub fn viscosity_factor(&self, rise: f64) -> f64 {
        (self.viscosity_slope * (1.0 / (self.ambient + rise) - 1.0 / self.reference)).exp()
    }

    /// The gearbox's efficiency `rise` above ambient. Not below 0.01: a gearbox far below its reference temperature
    /// would otherwise lose more than it transmits.
    pub fn gearbox_efficiency(&self, rise: f64) -> f64 {
        let s = self.gearbox_viscous_share;
        (1.0 - (1.0 - self.efficiency) * (1.0 - s + s * self.viscosity_factor(rise))).max(0.01)
    }

    /// Factor on the belt's EA at ambient.
    pub fn belt_stiffness_factor(&self) -> f64 {
        1.0 - self.belt_coefficient * (self.ambient - self.reference)
    }
}

// ---------------------------------------------------------------------------------------------
// Belt tooth jump

/// States per driven pulley with effects.belt_tooth_jump: the slip of the belt over the pulley u (m of belt, a
/// whole number of pitches while the teeth are in mesh), its rate, the direction of a jump under way (+1, -1;
/// 0 while in mesh), u where the jump began, the jumps counted, and the ridge force of the jump under way.
pub const JUMP_STATES: usize = 6;

/// With effects.belt_tooth_jump: the belt's teeth jump over a driven pulley.
///
/// The teeth climb out when the effective pull at the pulley, |T_a - T_b|, exceeds `climb` times the slack-side
/// tension min(T_a, T_b): each tooth in mesh is held in its groove by the tension's radial pull, T p / r over one
/// pitch, and pushed out by its share of the pull times the tangent of an effective flank angle (flank, friction
/// and the tooth's give together). Summed over the z_m teeth in mesh: climb = k z_m p / r, k = 1 / tan(effective
/// angle) an estimate (`belt.tooth_jump_factor`). The pull a pulley can take before the teeth jump is then
/// proportional to the tension the belt is set up with: the torque at which timing belts jump is reported to rise
/// with their installation tension. No maker's chart was read for the factor.
///
/// Once out, the teeth ride over the pulley's: the belt, with its mass, moves on its own, and the pulley holds it
/// back with a ridge force F_s sin(2 pi d / p), d how far it has slid, F_s the pull it climbed out at (climb times
/// the slack side's tension then), held over the jump so that the ridge is a spring with the energy
/// F_s p / (2 pi) (1 - cos(2 pi d / p)). That force is what the pulley passes on meanwhile. Where the belt crosses the
/// next groove (d = p) the teeth fall in: a jump, counted; where it slides back over the groove it left (d < 0),
/// they fall back in, uncounted. Either way belt and pulley run on together, at the speed that keeps their momentum
/// (a plastic impact): what that takes is what a jump dissipates. Where the pull settles below the ridge the belt
/// stays half out, held by it.
///
/// The belt's mass is the share the machine lumps on that pulley (effects.belt_mass, which the effect needs): while
/// the teeth are out it moves with the belt and not with the pulley.
#[derive(Debug, Clone)]
pub struct ToothJump {
    pub pitch: f64,
    pub climb: f64,
    /// The belt's mass lumped on the pulley, kg.
    pub mass: f64,
}

impl ToothJump {
    /// `None` with the effect off. `radius` is the driven pulley's, `mass` the belt's mass the machine lumps on it;
    /// none (effects.belt_mass off) is refused.
    pub fn from_config(cfg: &Config, radius: f64, mass: f64) -> Result<Option<Self>> {
        if !cfg.bool("effects.belt_tooth_jump")? {
            return Ok(None);
        }
        if mass <= 0.0 {
            return Err(ConfigError::Invalid("effects.belt_tooth_jump".into()));
        }
        let pitch = cfg.positive("belt.tooth_pitch_m")?;
        let climb = cfg.positive("belt.tooth_jump_factor")? * cfg.positive("ratings.belt_teeth_in_mesh")? * pitch / radius;
        Ok(Some(Self { pitch, climb, mass }))
    }

    /// The pulley's inertia: `lumped` (with the belt's share) while the teeth are in mesh, without it while they are out.
    pub fn pulley_inertia(&self, slip: &[f64], lumped: f64, radius: f64) -> f64 {
        if slip[2] == 0.0 { lumped } else { lumped - self.mass * radius * radius }
    }

    /// Whether the teeth climb out at these tensions of the two sections that meet at the pulley.
    pub fn climbs(&self, a: f64, b: f64) -> bool {
        (a - b).abs() > self.climb * a.min(b).max(0.0)
    }

    /// The force between belt and pulley along the belt, towards +u on the pulley: the sections' pull `force` while
    /// the teeth are in mesh, the ridge force while they are out.
    pub fn pulley_force(&self, slip: &[f64], force: f64) -> f64 {
        if slip[2] == 0.0 { force } else { slip[5] * (std::f64::consts::TAU * (slip[0] - slip[3]) / self.pitch).sin() }
    }

    /// `pulley_force` as a torque on a pulley of `radius`: the belt's torque as it is while in mesh, so that an effect
    /// that never climbs out changes no bit.
    pub fn pulley_torque(&self, slip: &[f64], belt_torque: f64, radius: f64) -> f64 {
        if slip[2] == 0.0 { belt_torque } else { radius * self.pulley_force(slip, 0.0) }
    }

    /// The rates of a pulley's slip states: none while in mesh. While the teeth are out, the belt moves on its own at
    /// r da/dt + du/dt (a the pulley's angle) under `force`, the sections' pull towards +u, less the ridge force:
    /// d2u/dt2 = (force - ridge) / mass - `surface_acc`, the pulley's surface acceleration r d2a/dt2.
    pub fn rates(&self, slip: &[f64], force: f64, surface_acc: f64, out: &mut [f64]) {
        out[..JUMP_STATES].fill(0.0);
        if slip[2] != 0.0 {
            (out[0], out[1]) = (slip[1], (force - self.pulley_force(slip, force)) / self.mass - surface_acc);
        }
    }

    /// What a sliding belt adds to the energy of a machine that counts its mass with the pulley, whose surface moves at
    /// v: 1/2 m (v + du/dt)^2 - 1/2 m v^2, and the ridge's energy; 0 while the teeth are in mesh.
    pub fn slip_energy(&self, slip: &[f64], surface_speed: f64) -> f64 {
        if slip[2] == 0.0 {
            return 0.0;
        }
        let ridge = slip[5] * self.pitch / std::f64::consts::TAU * (1.0 - (std::f64::consts::TAU * (slip[0] - slip[3]) / self.pitch).cos());
        0.5 * self.mass * slip[1] * (2.0 * surface_speed + slip[1]) + ridge
    }

    /// After a step: starts a jump where the teeth climb out at the tensions `a`, `b` (the section that +u feeds
    /// and the other one), or ends one where the belt has crossed the next groove (a jump) or the one it left: belt
    /// and pulley then share their momentum, `surface_speed` the pulley's and `rest_mass` its inertia without the belt
    /// over r^2. Returns whether it changed anything: the machine's energy then changes at once.
    pub fn update(&self, slip: &mut [f64], a: f64, b: f64, rest_mass: f64, surface_speed: &mut f64) -> bool {
        let dir = slip[2];
        if dir == 0.0 {
            if !self.climbs(a, b) {
                return false;
            }
            (slip[1], slip[2], slip[3], slip[5]) = (0.0, (a - b).signum(), slip[0], self.climb * a.min(b).max(0.0));
            return true;
        }
        let travel = (slip[0] - slip[3]) * dir;
        if (0.0..self.pitch).contains(&travel) {
            return false;
        }
        let jumped = travel >= self.pitch;
        *surface_speed += self.mass * slip[1] / (rest_mass + self.mass);
        slip[0] = slip[3] + if jumped { dir * self.pitch } else { 0.0 };
        (slip[1], slip[2], slip[5]) = (0.0, 0.0, 0.0);
        if jumped {
            slip[4] += 1.0;
        }
        true
    }
}

// ---------------------------------------------------------------------------------------------
// Sideways vibration of the free belt spans

/// First transverse mode of a free belt span as a string: f1 = sqrt(T / mu) / (2 L). The belt's own speed lowers it
/// by the factor 1 - v² / c² (c = sqrt(T / mu)), below 0.2 % at the speeds and tensions here, left out. 0 for a slack
/// span.
pub fn span_frequency(length: f64, tension: f64, mass_per_m: f64) -> f64 {
    if tension > 0.0 && length > 0.0 { (tension / mass_per_m).sqrt() / (2.0 * length) } else { 0.0 }
}

/// How far a span's first mode is from the nearest excitation, as a share of the mode: min |f1 - f_e| / f1. 1 for
/// a slack span or none.
pub fn span_detuning(f1: f64, excitations: &[f64]) -> f64 {
    if f1 <= 0.0 {
        return 1.0;
    }
    excitations.iter().filter(|f| **f > 0.0).map(|f| (f1 - f).abs() / f1).fold(1.0, f64::min)
}

/// The frequencies one drive excites the belt with, Hz: the tooth mesh |v| / p at the pulley (v its belt speed), and
/// with effects.torque_ripple the motor's ripple and cogging orders times its turns per second.
pub fn drive_excitations(pitch: f64, belt_speed: f64, ripple: &Ripple, motor_speed: f64, out: &mut Vec<f64>) {
    let turns = motor_speed.abs() / std::f64::consts::TAU;
    out.push(belt_speed.abs() / pitch);
    if ripple.ripple != 0.0 {
        out.push(ripple.ripple_order.abs() * turns);
    }
    if ripple.cogging != 0.0 {
        out.push(ripple.cogging_order.abs() * turns);
    }
}

// ---------------------------------------------------------------------------------------------
// Belt

/// A toothed belt. A *section* is the belt between two points that hold it (a driven pulley or a
/// clamp); idlers in between are free, so the tension is the same along the whole section.
///
/// A section has a geometric length `geo` (set by the pose) and holds `mat` of unstretched belt
/// (changed by the pulleys feeding belt in or out). Stretch e = geo - mat, stiffness k = EA / mat,
/// tension T = max(0, k e + c de/dt).
#[derive(Debug, Clone)]
pub struct Belt {
    pub ea: f64,
    pub damping: f64,
    /// Material loss: a section damps with its stiffness times this time, so a short section damps more.
    pub damping_time: f64,
    pub pretension: f64,
    pub slack: bool,
    pub mass_per_m: f64,
    /// The teeth in mesh on a driven pulley give way like this much more belt: EA / tooth-mesh stiffness.
    /// A machine adds it to the geometric length of a section once per driven pulley the section ends at.
    /// `mat` then holds it too, and k = EA / mat is the belt and the teeth in series.
    pub tooth_length: f64,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct SectionForce {
    /// Pulls on whatever the section's ends are attached to.
    pub tension: f64,
    /// Force on the belt material fed into the section, -dV/d(mat) plus damping. It exceeds the
    /// tension by the factor (1 + strain/2), because stretched belt carries its strain energy along.
    /// Using it at the pulleys makes the model conserve energy exactly.
    pub material: f64,
}

impl Belt {
    pub fn from_config(cfg: &Config) -> Result<Self> {
        let feedback = ThermalFeedback::from_config(cfg)?.map_or(1.0, |t| t.belt_stiffness_factor());
        let ea = cfg.positive("belt.stiffness_ea_n")? * temperature_scale(cfg, "belt_stiffness_scale")? * feedback;
        if ea <= 0.0 {
            return Err(ConfigError::Invalid("thermal_feedback.belt_stiffness_coefficient_1_k".into()));
        }
        let damping = cfg.bool("effects.belt_damping")?;
        let material = cfg.bool("effects.belt_material_damping")? && damping;
        let teeth = cfg.bool("effects.belt_tooth_stiffness")?;
        Ok(Self {
            ea,
            damping: if damping && !material { cfg.non_negative("belt.damping_n_s_m")? } else { 0.0 },
            damping_time: if material { cfg.non_negative("belt.damping_time_s")? } else { 0.0 },
            tooth_length: if teeth { ea / cfg.positive("belt.tooth_mesh_stiffness_n_m")? } else { 0.0 },
            // Not negative: at minus EA the belt would have no length at rest, and a belt cannot push.
            pretension: cfg.non_negative("belt.pretension_n")?,
            slack: cfg.bool("effects.belt_slack")?,
            mass_per_m: if cfg.bool("effects.belt_mass")? { cfg.non_negative("belt.mass_kg_m")? } else { 0.0 },
        })
    }

    /// Unstretched belt in a section of length `geo` at pretension.
    pub fn rest_length(&self, geo: f64) -> f64 {
        geo / (1.0 + self.pretension / self.ea)
    }

    pub fn section(&self, geo: f64, mat: f64, geo_rate: f64, mat_rate: f64) -> SectionForce {
        let e = geo - mat;
        let elastic = self.ea * e / mat;
        let total = elastic + (self.damping + self.damping_time * self.ea / mat) * (geo_rate - mat_rate);
        if self.slack && e < 0.0 {
            return SectionForce { tension: total.max(0.0), material: total.max(0.0) };
        }
        let tension = if self.slack { total.max(0.0) } else { total };
        SectionForce { tension, material: tension + elastic * e / (2.0 * mat) }
    }

    pub fn energy(&self, geo: f64, mat: f64) -> f64 {
        let e = geo - mat;
        if self.slack && e < 0.0 { 0.0 } else { 0.5 * self.ea * e * e / mat }
    }

    /// Power the damping of a section takes, W: what goes into it (the tension on the length, less the material
    /// force on the belt fed in) less what its stored energy gains. c (rate)^2 while it is taut.
    pub fn loss_power(&self, geo: f64, mat: f64, geo_rate: f64, mat_rate: f64) -> f64 {
        let (f, e) = (self.section(geo, mat, geo_rate, mat_rate), geo - mat);
        let stored = if self.slack && e < 0.0 { 0.0 } else { self.ea * e / mat * (geo_rate - mat_rate) - 0.5 * self.ea * e * e / (mat * mat) * mat_rate };
        f.tension * geo_rate - f.material * mat_rate - stored
    }

    /// Material per section at rest, when section i carries `base + offsets[i]` and the sections
    /// share `total` of belt. The base tension follows from the total; found by bisection.
    pub fn static_material(&self, geo: &[f64], offsets: &[f64], total: f64) -> Vec<f64> {
        let material = |base: f64| -> Vec<f64> {
            let tension = |o: f64| if self.slack { (base + o).max(0.0) } else { base + o };
            geo.iter().zip(offsets).map(|(g, o)| g / (1.0 + tension(*o) / self.ea)).collect()
        };
        let (mut lo, mut hi) = (-0.5 * self.ea, self.ea);
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if material(mid).iter().sum::<f64>() > total { lo = mid } else { hi = mid }
        }
        material(0.5 * (lo + hi))
    }
}

// ---------------------------------------------------------------------------------------------
// Hard stops

/// End stops of an axis whose stroke runs from 0 to `stroke`: a spring and a damper at both ends that act
/// only beyond it.
#[derive(Debug, Clone)]
pub struct HardStop {
    pub stiffness: f64,
    pub damping: f64,
}

impl HardStop {
    pub fn from_config(cfg: &Config) -> Result<Self> {
        let (stiffness, damping) = (cfg.positive("hard_stop.stiffness_n_m")?, cfg.non_negative("hard_stop.damping_n_s_m")?);
        Ok(if cfg.bool("effects.hard_stops")? { Self { stiffness, damping } } else { Self { stiffness: 0.0, damping: 0.0 } })
    }

    /// How far a position is beyond the stroke: negative below 0, positive above the stroke.
    fn beyond(pos: f64, stroke: f64) -> f64 {
        pos - pos.clamp(0.0, stroke)
    }

    /// Force of the stops on the axis. They push back, they never hold on.
    pub fn force(&self, pos: f64, vel: f64, stroke: f64) -> f64 {
        let beyond = Self::beyond(pos, stroke);
        let force = -self.stiffness * beyond - self.damping * vel;
        if force * beyond < 0.0 { force } else { 0.0 }
    }

    pub fn energy(&self, pos: f64, stroke: f64) -> f64 {
        0.5 * self.stiffness * Self::beyond(pos, stroke).powi(2)
    }

    /// Power the stops take, W: what the axis loses to them less what their springs store.
    pub fn loss_power(&self, pos: f64, vel: f64, stroke: f64) -> f64 {
        -(self.force(pos, vel, stroke) + self.stiffness * Self::beyond(pos, stroke)) * vel
    }
}

// ---------------------------------------------------------------------------------------------
// Cable chain

/// A cable chain along an axis, fixed at the x = 0 end: half of every metre the carriage travels joins the
/// run that moves with it. It drags with a constant force. All zero with the effect off.
#[derive(Debug, Clone)]
pub struct CableChain {
    pub mass_per_m: f64,
    /// Chain that moves with the carriage at x = 0.
    pub length0: f64,
    pub drag: f64,
}

impl CableChain {
    pub fn from_config(cfg: &Config) -> Result<Self> {
        let on = cfg.bool("effects.cable_chain")?;
        let value = |leaf: &str| cfg.non_negative(&format!("cable_chain.{leaf}"));
        let (mass_per_m, length0, drag) = (value("mass_kg_m")?, value("length_at_zero_m")?, value("drag_n")?);
        Ok(Self { mass_per_m: if on { mass_per_m } else { 0.0 }, length0, drag: if on { drag } else { 0.0 } })
    }

    /// (mass that moves with the carriage at x, its growth per metre). A mass m(x) moving at v takes
    /// m dv/dt + m' v^2 / 2 of force: the second part is what picking up chain costs.
    pub fn mass(&self, x: f64) -> (f64, f64) {
        (self.mass_per_m * (self.length0 + 0.5 * x), 0.5 * self.mass_per_m)
    }

    /// Drag opposing v; it fades out at rest, below `fade_v`.
    pub fn drag(&self, v: f64, fade_v: f64) -> f64 {
        self.drag * (v / fade_v).tanh()
    }
}

// ---------------------------------------------------------------------------------------------
// Holding brake

/// A brake at its full torque has given way by this angle.
const BRAKE_DEFLECTION_RAD: f64 = 1e-3;
/// A released brake forgets its deflection this fast.
const BRAKE_RELEASE_S: f64 = 1e-3;
/// A slipping brake forgets this fast what it is deflected beyond its full torque. Well above the integration
/// step (a quarter of 62.5 us by default), so that a brake that engages at any speed is stable.
const BRAKE_SLIP_S: f64 = 5e-5;

/// Holding brake on a motor shaft, engaged while the drives are disabled. A machine keeps one state per
/// brake, its deflection s.
#[derive(Debug, Clone)]
pub struct HoldingBrake {
    /// (its torque, the damping of the shaft held in it); `None` with the effect off.
    held: Option<(f64, f64)>,
}

impl HoldingBrake {
    /// `j_rotor` is the inertia the brake holds directly: rotor and gearbox input.
    pub fn from_config(cfg: &Config, j_rotor: f64) -> Result<Self> {
        if !cfg.bool("effects.holding_brake")? {
            return Ok(Self { held: None });
        }
        let torque = cfg.positive("motor.brake_torque_nm")?;
        // Critical damping of the rotor on the brake's stiffness, so that it stops without ringing.
        Ok(Self { held: Some((torque, 2.0 * (torque / BRAKE_DEFLECTION_RAD * j_rotor).sqrt())) })
    }

    /// (torque opposing the speed, rate of the deflection s).
    ///
    /// Held, the shaft sits on a damped spring that reaches the brake torque at `BRAKE_DEFLECTION_RAD`. The
    /// torque never exceeds the brake torque: beyond it the brake slips, and s stops following the shaft.
    /// A friction contact with bristle damping (LuGre) gave many times the brake torque when it engaged at
    /// speed, and its state was too stiff for the step above 178 rad/s.
    pub fn torque(&self, engaged: bool, speed: f64, s: f64) -> (f64, f64) {
        match self.held {
            Some((torque, damping)) if engaged => {
                let rate = speed - (s - s.clamp(-BRAKE_DEFLECTION_RAD, BRAKE_DEFLECTION_RAD)) / BRAKE_SLIP_S;
                ((torque / BRAKE_DEFLECTION_RAD * s + damping * rate).clamp(-torque, torque), rate)
            }
            _ => (0.0, -s / BRAKE_RELEASE_S),
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Guide friction

/// LuGre friction contact: Coulomb, viscous, stiction and the Stribeck drop between them,
/// with one internal state s (the bristle deflection).
#[derive(Debug, Clone)]
pub struct Lugre {
    pub on: bool,
    pub sigma0: f64,
    pub sigma1: f64,
    pub viscous: f64,
    pub coulomb: f64,
    pub stiction: f64,
    pub stribeck_v: f64,
    /// Guide preload; with `load_dependent` the Coulomb and stiction levels scale with preload + load.
    pub preload: f64,
    pub load_dependent: bool,
}

impl Lugre {
    pub fn from_config(cfg: &Config, table: &str) -> Result<Self> {
        let scale = temperature_scale(cfg, "friction_scale")?;
        let get = |leaf: &str| cfg.non_negative(&format!("{table}.{leaf}"));
        let coulomb = cfg.positive(&format!("{table}.coulomb_n"))? * scale;
        Ok(Self {
            on: cfg.bool("effects.guide_friction")?,
            sigma0: cfg.positive(&format!("{table}.bristle_stiffness_n_m"))?,
            sigma1: get("bristle_damping_n_s_m")?,
            // With the feedback, at ambient; the machine follows the guide's temperature (`ThermalFeedback`).
            viscous: get("viscous_n_s_m")? * scale * ThermalFeedback::from_config(cfg)?.map_or(1.0, |t| t.viscosity_factor(0.0)),
            coulomb,
            // Breaking away takes at least what sliding takes. A value below the Coulomb level counts as that
            // level: uncertainty bounds of the two that overlap would otherwise give runs that slide harder
            // than they break away.
            stiction: (cfg.positive(&format!("{table}.stiction_n"))? * scale).max(coulomb),
            stribeck_v: cfg.positive(&format!("{table}.stribeck_m_s"))?,
            preload: cfg.positive(&format!("{table}.preload_n"))?,
            load_dependent: cfg.bool("effects.load_dependent_friction")?,
        })
    }

    /// Factor on the friction levels for a guide load, relative to the load the levels were given for.
    pub fn load_scale(&self, load: f64, nominal_load: f64) -> f64 {
        if self.load_dependent { (self.preload + load.abs()) / (self.preload + nominal_load.abs()) } else { 1.0 }
    }

    /// Sliding level g(v) = Fc + (Fs - Fc) exp(-(v/vs)^2), times the load scale.
    pub fn level(&self, v: f64, load_scale: f64) -> f64 {
        load_scale * (self.coulomb + (self.stiction - self.coulomb) * (-(v / self.stribeck_v).powi(2)).exp())
    }

    /// Returns (friction force opposing positive v, ds/dt).
    pub fn eval(&self, v: f64, s: f64, load_scale: f64) -> (f64, f64) {
        if !self.on {
            return (0.0, 0.0);
        }
        let level = self.level(v, load_scale);
        let rate = self.sigma0 * v.abs() / level;
        if rate <= BRISTLE_RATE_MAX {
            let s_dot = v - self.sigma0 * v.abs() * s / level;
            return (self.sigma0 * s + self.sigma1 * s_dot + self.viscous * v, s_dot);
        }
        // shortcut: bristles that would settle faster than `BRISTLE_RATE_MAX` settle at that rate, with their
        // force scaled so that sliding still takes the level; step the friction state implicitly if what
        // bristles do within 10 us ever matters.
        let s_dot = v - BRISTLE_RATE_MAX * s;
        (self.sigma0 * s * BRISTLE_RATE_MAX / rate + self.sigma1 * s_dot + self.viscous * v, s_dot)
    }

    /// Steady sliding at v, g(v) sign(v) + b v, split into [Coulomb, viscous, stiction]: the Coulomb level, b v,
    /// and what the Stribeck curve adds to the Coulomb level, (Fs - Fc) exp(-(v/vs)^2), each times the load scale.
    /// Zero with the effect off.
    pub fn steady(&self, v: f64, load_scale: f64) -> [f64; 3] {
        if !self.on {
            return [0.0; 3];
        }
        let (sign, coulomb) = (if v == 0.0 { 0.0 } else { v.signum() }, load_scale * self.coulomb);
        [coulomb * sign, self.viscous * v, (self.level(v, load_scale) - coulomb) * sign]
    }

    /// The force of `eval` split into [Coulomb, viscous, stiction]: b v, and the bristle force (all the rest) shared
    /// in the proportion of the Coulomb level to what the Stribeck curve adds to it at this speed. In steady
    /// sliding these are the parts of `steady`; at rest the bristles hold with the shares of the stiction level.
    pub fn split(&self, v: f64, s: f64, load_scale: f64) -> [f64; 3] {
        if !self.on {
            return [0.0; 3];
        }
        let (viscous, level) = (self.viscous * v, self.level(v, load_scale));
        // No level (a load scale of 0): nothing for the bristles to hold, and no share to split by.
        if level == 0.0 {
            return [0.0, viscous, 0.0];
        }
        let coulomb = load_scale * self.coulomb / level;
        let bristles = self.eval(v, s, load_scale).0 - viscous;
        [bristles * coulomb, viscous, bristles * (1.0 - coulomb)]
    }
}

/// The fastest the bristles of a friction contact settle, 1/s. The bristle state settles at the rate
/// sigma0 |v| / level, which a stiff bristle on a light contact at speed takes beyond what the integrator can
/// follow (1e6 N/m, 12 N and 2.5 m/s: 2.1e5 1/s, against a limit of 2.8 over the 15.6 us step = 1.8e5 1/s for
/// RK4): the friction force then grew without bound. No config shipped reaches the cap with its own values.
const BRISTLE_RATE_MAX: f64 = 1.0e5;

// ---------------------------------------------------------------------------------------------
// Motor shaft

/// Motor and bearing friction: Coulomb plus viscous on the shaft. The Coulomb step is smoothed
/// with tanh so a shaft at rest sees no chattering torque.
#[derive(Debug, Clone)]
pub struct ShaftFriction {
    pub coulomb: f64,
    pub viscous: f64,
    pub smoothing: f64,
}

impl ShaftFriction {
    pub fn from_config(cfg: &Config) -> Result<Self> {
        let on = cfg.bool("effects.motor_friction")?;
        Ok(Self {
            coulomb: if on { cfg.non_negative("motor.friction_coulomb_nm")? } else { 0.0 },
            viscous: if on { cfg.non_negative("motor.friction_viscous_nm_s_rad")? } else { 0.0 },
            smoothing: cfg.positive("motor.friction_smoothing_rad_s")?,
        })
    }

    /// Torque opposing a positive speed.
    pub fn torque(&self, speed: f64) -> f64 {
        self.coulomb * (speed / self.smoothing).tanh() + self.viscous * speed
    }
}

/// Position-dependent motor torque: ripple proportional to the torque, cogging independent of it.
#[derive(Debug, Clone)]
pub struct Ripple {
    pub ripple: f64,
    pub ripple_order: f64,
    pub cogging: f64,
    pub cogging_order: f64,
}

impl Ripple {
    pub fn from_config(cfg: &Config) -> Result<Self> {
        let on = cfg.bool("effects.torque_ripple")?;
        // Below one: at one the torque would vanish at an angle, and `invert` divide by zero there.
        let ripple = cfg.non_negative("motor.ripple_fraction")?;
        if ripple >= 1.0 {
            return Err(ConfigError::Invalid("motor.ripple_fraction".into()));
        }
        Ok(Self {
            ripple: if on { ripple } else { 0.0 },
            ripple_order: cfg.f64("motor.ripple_order")?,
            cogging: if on { cfg.f64("motor.cogging_nm")? } else { 0.0 },
            cogging_order: cfg.f64("motor.cogging_order")?,
        })
    }

    /// Torque at the shaft for a torque produced by the current.
    pub fn apply(&self, torque: f64, angle: f64) -> f64 {
        torque * (1.0 + self.ripple * (self.ripple_order * angle).sin()) + self.cogging * (self.cogging_order * angle).sin()
    }

    /// The current torque that gives `shaft` at the shaft: the inverse of `apply`.
    pub fn invert(&self, shaft: f64, angle: f64) -> f64 {
        (shaft - self.cogging * (self.cogging_order * angle).sin()) / (1.0 + self.ripple * (self.ripple_order * angle).sin())
    }
}

/// Tooth meshing: a small periodic force at the tooth-pass frequency.
#[derive(Debug, Clone)]
pub struct ToothMesh {
    pub force: f64,
    pub pitch: f64,
}

impl ToothMesh {
    pub fn from_config(cfg: &Config) -> Result<Self> {
        Ok(Self {
            force: if cfg.bool("effects.tooth_meshing")? { cfg.f64("belt.tooth_force_n")? } else { 0.0 },
            pitch: cfg.positive("belt.tooth_pitch_m")?,
        })
    }

    /// Force along the belt for a belt travel at the pulley.
    pub fn force(&self, travel: f64) -> f64 {
        self.force * (std::f64::consts::TAU * travel / self.pitch).sin()
    }
}

// ---------------------------------------------------------------------------------------------
// Gearbox

/// Gearbox between motor shaft and pulley: ratio, torsional stiffness, backlash as a dead zone,
/// efficiency. Stiffness, damping and backlash are given at the output side.
#[derive(Debug, Clone)]
pub struct Gearbox {
    pub ratio: f64,
    pub stiffness: f64,
    pub damping: f64,
    pub half_backlash: f64,
    pub efficiency: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct GearTorque {
    /// Torque driving the output shaft.
    pub output: f64,
    /// Reaction that the motor shaft has to overcome.
    pub rotor: f64,
}

/// Below this power the efficiency fades out, so a gearbox at rest has no torque step.
const EFFICIENCY_FADE_W: f64 = 0.5;

impl Gearbox {
    pub fn from_config(cfg: &Config) -> Result<Self> {
        // A negative backlash would turn the dead zone inside out (and `clamp` panics on that).
        let backlash = cfg.non_negative("gearbox.backlash_rad")?;
        Ok(Self {
            ratio: cfg.positive("gearbox.ratio")?,
            stiffness: cfg.positive("gearbox.stiffness_nm_rad")?,
            damping: cfg.non_negative("gearbox.damping_nm_s_rad")?,
            half_backlash: if cfg.bool("effects.gearbox_backlash")? { 0.5 * backlash } else { 0.0 },
            // With the feedback, at ambient; the machine follows the gearbox's temperature (`ThermalFeedback`).
            efficiency: match ThermalFeedback::from_config(cfg)? {
                Some(t) => t.gearbox_efficiency(0.0),
                None => Self::configured_efficiency(cfg)?,
            },
        })
    }

    /// The efficiency as configured: 1 with effects.gearbox_efficiency off.
    pub fn configured_efficiency(cfg: &Config) -> Result<f64> {
        let efficiency = cfg.positive("gearbox.efficiency")?;
        if efficiency > 1.0 {
            return Err(ConfigError::Invalid("gearbox.efficiency".into()));
        }
        Ok(if cfg.bool("effects.gearbox_efficiency")? { efficiency } else { 1.0 })
    }

    /// A ratio of 1 is a direct drive: the table describes the coupling between motor and pulley, there is no
    /// gearbox, and what `loss_power` gives is the coupling's damping.
    pub fn is_coupling(&self) -> bool {
        self.ratio == 1.0
    }

    /// Twist beyond the dead zone.
    fn engaged_twist(&self, twist: f64) -> f64 {
        twist - twist.clamp(-self.half_backlash, self.half_backlash)
    }

    /// Factor on a torque at the output: efficiency when driving, 1/efficiency when driven back.
    pub fn loss_factor(&self, output_torque: f64, output_speed: f64) -> f64 {
        self.efficiency.powf((output_torque * output_speed / EFFICIENCY_FADE_W).tanh())
    }

    pub fn torque(&self, rotor_angle: f64, rotor_speed: f64, out_angle: f64, out_speed: f64) -> GearTorque {
        let twist = self.engaged_twist(rotor_angle / self.ratio - out_angle);
        if twist == 0.0 {
            return GearTorque { output: 0.0, rotor: 0.0 };
        }
        let torque = self.stiffness * twist + self.damping * (rotor_speed / self.ratio - out_speed);
        // Tooth flanks can push but not pull: while they separate fast, the damping must not reverse the torque.
        if self.half_backlash > 0.0 && torque * twist < 0.0 {
            return GearTorque { output: 0.0, rotor: 0.0 };
        }
        GearTorque { output: torque * self.loss_factor(torque, out_speed), rotor: torque / self.ratio }
    }

    /// Twist at rest that transmits `output_torque`, on the matching side of the backlash.
    pub fn static_twist(&self, output_torque: f64) -> f64 {
        if output_torque == 0.0 { 0.0 } else { output_torque / self.stiffness + self.half_backlash.copysign(output_torque) }
    }

    pub fn energy(&self, rotor_angle: f64, out_angle: f64) -> f64 {
        0.5 * self.stiffness * self.engaged_twist(rotor_angle / self.ratio - out_angle).powi(2)
    }

    /// Power the gearbox takes, W: what the rotor puts in less what the output gives the pulley and what the spring
    /// stores. The efficiency loss and the damping.
    pub fn loss_power(&self, rotor_angle: f64, rotor_speed: f64, out_angle: f64, out_speed: f64) -> f64 {
        let t = self.torque(rotor_angle, rotor_speed, out_angle, out_speed);
        let twist = self.engaged_twist(rotor_angle / self.ratio - out_angle);
        t.rotor * rotor_speed - t.output * out_speed - self.stiffness * twist * (rotor_speed / self.ratio - out_speed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn belt() -> Belt {
        Belt { ea: 5e5, damping: 0.0, damping_time: 0.0, pretension: 300.0, slack: true, mass_per_m: 0.0, tooth_length: 0.0 }
    }

    #[test]
    fn belt_tension_is_ea_strain_and_pretension_at_rest() {
        let b = belt();
        let mat = b.rest_length(1.2);
        assert!((b.section(1.2, mat, 0.0, 0.0).tension - 300.0).abs() < 1e-9);
        // k = EA / L: 1 mm more on about 1.2 m of belt.
        let more = b.section(1.201, mat, 0.0, 0.0).tension - 300.0;
        assert!((more - 5e5 * 0.001 / mat).abs() < 1e-6);
    }

    #[test]
    fn belt_tension_cannot_go_below_zero_unless_slack_is_off() {
        let mut b = belt();
        assert_eq!(b.section(0.99, 1.0, 0.0, 0.0).tension, 0.0);
        assert_eq!(b.energy(0.99, 1.0), 0.0);
        b.slack = false;
        assert!(b.section(0.99, 1.0, 0.0, 0.0).tension < 0.0);
    }

    #[test]
    fn belt_damping_acts_on_the_stretch_rate() {
        let mut b = belt();
        b.damping = 100.0;
        let mat = b.rest_length(1.0);
        let still = b.section(1.0, mat, 0.0, 0.0).tension;
        assert!((b.section(1.0, mat, 0.5, 0.2).tension - still - 100.0 * 0.3).abs() < 1e-9);
    }

    /// Material damping c = k tau with k = EA / L: 0.5 m of belt is four times as stiff as 2 m and damps
    /// four times as hard.
    #[test]
    fn material_damping_scales_with_the_section_stiffness() {
        let mut b = belt();
        b.damping_time = 2e-4;
        // Stretching at 0.3 m/s. Short: k = 5e5 / 0.5 = 1e6 N/m, c = 200 N s/m, 60 N.
        // Long: k = 5e5 / 2 = 2.5e5 N/m, c = 50 N s/m, 15 N.
        for (mat, force) in [(0.5, 60.0), (2.0, 15.0)] {
            let still = b.section(mat + 0.001, mat, 0.0, 0.0).tension;
            let moving = b.section(mat + 0.001, mat, 0.5, 0.2).tension;
            assert!((moving - still - force).abs() < 1e-9, "{mat} m: {} N", moving - still);
        }
    }

    /// Tooth stiffness in series: a 1 m section that ends at one driven pulley with 2e6 N/m of tooth
    /// stiffness has 1 / (1 / 5e5 + 1 / 2e6) = 4e5 N/m, the same as 1.25 m of belt.
    #[test]
    fn teeth_in_mesh_are_a_spring_in_series_with_the_section() {
        let mut b = belt();
        b.tooth_length = b.ea / 2e6;
        let geo = 1.0 + b.tooth_length;
        let mat = b.rest_length(geo);
        assert!((b.section(geo, mat, 0.0, 0.0).tension - 300.0).abs() < 1e-9);
        let more = b.section(geo + 1e-4, mat, 0.0, 0.0).tension - 300.0;
        // The belt is already stretched by the pretension, hence the 0.1 % tolerance.
        assert!((more - 4e5 * 1e-4).abs() < 1e-3 * 40.0, "{more} N");
    }

    #[test]
    fn belt_forces_are_the_gradient_of_its_energy() {
        let b = belt();
        let (geo, mat, h) = (1.003, 1.0, 1e-7);
        let f = b.section(geo, mat, 0.0, 0.0);
        let d_geo = (b.energy(geo + h, mat) - b.energy(geo - h, mat)) / (2.0 * h);
        let d_mat = (b.energy(geo, mat + h) - b.energy(geo, mat - h)) / (2.0 * h);
        assert!((f.tension - d_geo).abs() < 1e-3, "{} vs {d_geo}", f.tension);
        assert!((f.material + d_mat).abs() < 1e-3, "{} vs {d_mat}", f.material);
    }

    #[test]
    fn static_material_meets_the_tension_offsets() {
        let b = belt();
        let geo = [1.0, 2.0, 1.5];
        let total: f64 = geo.iter().map(|g| b.rest_length(*g)).sum();
        let mat = b.static_material(&geo, &[0.0, 40.0, 0.0], total);
        assert!((mat.iter().sum::<f64>() - total).abs() < 1e-12);
        let t: Vec<f64> = geo.iter().zip(&mat).map(|(g, m)| b.section(*g, *m, 0.0, 0.0).tension).collect();
        assert!((t[1] - t[0] - 40.0).abs() < 1e-6 && (t[2] - t[0]).abs() < 1e-6, "{t:?}");
    }

    fn lugre() -> Lugre {
        Lugre {
            on: true,
            sigma0: 5e5,
            sigma1: 1500.0,
            viscous: 20.0,
            coulomb: 15.0,
            stiction: 20.0,
            stribeck_v: 0.01,
            preload: 200.0,
            load_dependent: true,
        }
    }

    #[test]
    fn lugre_steady_sliding_is_coulomb_plus_viscous() {
        let f = lugre();
        // Steady state: ds/dt = 0, so s = g(v) sign(v) / sigma0.
        for v in [0.5, -0.5] {
            let s = f.level(v, 1.0) * v.signum() / f.sigma0;
            let (force, s_dot) = f.eval(v, s, 1.0);
            assert!(s_dot.abs() < 1e-12);
            assert!((force - v.signum() * (15.0 + 20.0 * 0.5)).abs() < 1e-9, "{force}");
        }
    }

    /// Stiff bristles on a light contact at speed. By hand: 1e6 N/m, a level of 12 N and 2.5 m/s make the
    /// bristle state settle at 1e6 * 2.5 / 12 = 2.1e5 1/s. RK4 with the simulator's 15.625 us is stable up to
    /// 2.785 / 15.625 us = 1.78e5 1/s: beyond it the friction force grew without bound (seen with these two
    /// values at their bounds together in a real-machine config). Capped at 1e5 1/s the state settles, and
    /// sliding takes the level plus the viscous part as before: 12 N + 20 N s/m * 2.5 m/s = 62 N.
    #[test]
    fn lugre_with_stiff_bristles_at_speed_stays_stable_and_slides_at_its_level() {
        let l = Lugre { sigma0: 1e6, coulomb: 12.0, stiction: 20.0, ..lugre() };
        let (v, h, mut s) = (2.5, 15.625e-6, 0.0);
        for _ in 0..2000 {
            let rate = |s: f64| l.eval(v, s, 1.0).1;
            let (k1, k2) = (rate(s), rate(s + 0.5 * h * rate(s)));
            let k3 = rate(s + 0.5 * h * k2);
            s += h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + rate(s + h * k3));
        }
        let (force, s_dot) = l.eval(v, s, 1.0);
        assert!((force - 62.0).abs() < 1e-6 && s_dot.abs() < 1e-9, "{force} N, bristles moving at {s_dot} m/s");
        // Below the cap nothing has changed: the force is sigma0 s + sigma1 ds/dt + viscous v with ds/dt = v - rate s.
        let (slow, s) = (0.1, 1e-5);
        let s_dot = slow - 1e6 * slow * s / l.level(slow, 1.0);
        assert_eq!(l.eval(slow, s, 1.0), (1e6 * s + 1500.0 * s_dot + 20.0 * slow, s_dot));
    }

    /// The friction view's curve, by hand for the test contact (Fc 15 N, Fs 20 N, b 20 N s/m, vs 0.01 m/s), as
    /// [Coulomb, viscous, stiction]: at 0.5 m/s 15, 10 and 5 exp(-2500) = 0 N; at 0.01 m/s 15, 0.2 and 5 / e =
    /// 1.83940 N; at -0.005 m/s -15, -0.1 and -5 exp(-0.25) = -3.89400 N. With the load scale 2 both levels double:
    /// 30, 0.2 and 3.67879 N at 0.01 m/s. In steady sliding the live split is the curve; held at rest by a bristle
    /// deflection of 1e-5 m (5 N) it is 5 * 15 / 20 = 3.75 N Coulomb and 1.25 N stiction.
    #[test]
    fn lugre_steady_curve_and_live_split_follow_the_formula() {
        let f = lugre();
        let close = |a: [f64; 3], b: [f64; 3]| a.iter().zip(&b).all(|(a, b)| (a - b).abs() < 1e-5);
        for (v, scale, expected) in [(0.5, 1.0, [15.0, 10.0, 0.0]), (0.01, 1.0, [15.0, 0.2, 1.83940]), (-0.005, 1.0, [-15.0, -0.1, -3.89400]), (0.01, 2.0, [30.0, 0.2, 3.67879])] {
            let steady = f.steady(v, scale);
            assert!(close(steady, expected), "{v} m/s: {steady:?}");
            let s = f.level(v, scale) * v.signum() / f.sigma0;
            assert!(close(f.split(v, s, scale), steady), "{v} m/s: {:?}", f.split(v, s, scale));
            assert!((f.split(v, s, scale).iter().sum::<f64>() - f.eval(v, s, scale).0).abs() < 1e-9);
        }
        assert!(close(f.split(0.0, 1e-5, 1.0), [3.75, 0.0, 1.25]), "{:?}", f.split(0.0, 1e-5, 1.0));
        assert_eq!(Lugre { on: false, ..lugre() }.steady(0.5, 1.0), [0.0; 3]);
    }

    /// A load scale of 0 leaves no friction level: the split is the viscous part alone, not NaN from 0 / 0.
    #[test]
    fn the_split_without_a_friction_level_is_the_viscous_part() {
        let f = lugre();
        assert_eq!(f.split(0.0, 0.0, 0.0), [0.0; 3]);
        assert_eq!(f.split(0.5, 1e-5, 0.0), [0.0, 10.0, 0.0]);
    }

    /// Loss powers by hand. Belt damping 100 N s/m stretched at 0.3 m/s: 100 * 0.3^2 = 9 W, whatever the stretch.
    /// Gearbox 5:1 at 90 % passing 2 N m at 50 rad/s out: 2 * 50 * 0.1 = 10 W; driven back, 2 / 0.9 * 50 - 2 * 50
    /// = 11.11 W; loss-free with 5 N m s/rad damping and the rotor 1 rad/s ahead at the output: 5 W. End stop with
    /// 200 N s/m, 1 mm beyond and moving on out at 0.1 m/s: 200 * 0.01 = 2 W.
    #[test]
    fn loss_powers_are_the_hand_calculated_ones() {
        let mut b = belt();
        b.damping = 100.0;
        let mat = b.rest_length(1.0);
        assert!((b.loss_power(1.0, mat, 0.5, 0.2) - 9.0).abs() < 1e-9 && (b.loss_power(1.003, mat, -0.1, 0.2) - 9.0).abs() < 1e-9);
        let mut g = gearbox();
        g.efficiency = 0.9;
        let twist = 5.0 * g.static_twist(2.0);
        assert!((g.loss_power(twist, 250.0, 0.0, 50.0) - 10.0).abs() < 1e-9, "{}", g.loss_power(twist, 250.0, 0.0, 50.0));
        assert!((g.loss_power(twist, -250.0, 0.0, -50.0) - (2.0 / 0.9 - 2.0) * 50.0).abs() < 1e-9);
        let g = Gearbox { damping: 5.0, half_backlash: 0.0, ..gearbox() };
        assert!((g.loss_power(5.0 * 1e-4, 5.0, 0.0, 0.0) - 5.0).abs() < 1e-9);
        let stop = HardStop { stiffness: 1e6, damping: 200.0 };
        assert!((stop.loss_power(1.001, 0.1, 1.0) - 2.0).abs() < 1e-9 && stop.loss_power(0.5, 0.1, 1.0) == 0.0);
    }

    #[test]
    fn lugre_level_falls_from_stiction_to_coulomb() {
        let f = lugre();
        assert!((f.level(0.0, 1.0) - 20.0).abs() < 1e-12);
        assert!((f.level(1.0, 1.0) - 15.0).abs() < 1e-9);
        assert!(f.level(0.005, 1.0) < 20.0 && f.level(0.005, 1.0) > 15.0);
    }

    /// A stiction level given below the Coulomb level counts as the Coulomb level: the contact never slides
    /// harder than it breaks away. A stiction level above it is taken as given.
    #[test]
    fn lugre_stiction_is_never_below_coulomb() {
        let level = |stiction: f64| {
            let text = format!("format_version = 1
[effects]
guide_friction = true
temperature = false
thermal_feedback = false
load_dependent_friction = false
[guide]
bristle_stiffness_n_m = 1e5
bristle_damping_n_s_m = 300.0
viscous_n_s_m = 20.0
coulomb_n = 26.0
stiction_n = {stiction:?}
stribeck_m_s = 0.01
preload_n = 200.0
");
            let f = Lugre::from_config(&Config::parse(&text).unwrap(), "guide").unwrap();
            (f.level(0.0, 1.0), f.level(1.0, 1.0))
        };
        assert_eq!(level(20.0), (26.0, 26.0));
        assert_eq!(level(30.0), (30.0, 26.0));
    }

    #[test]
    fn lugre_levels_scale_with_guide_load_and_switch_off() {
        let mut f = lugre();
        assert!((f.load_scale(300.0, 100.0) - 500.0 / 300.0).abs() < 1e-12);
        assert!((f.level(0.0, 2.0) - 40.0).abs() < 1e-12);
        f.load_dependent = false;
        assert_eq!(f.load_scale(300.0, 100.0), 1.0);
        f.on = false;
        assert_eq!(f.eval(0.3, 1e-5, 1.0), (0.0, 0.0));
    }

    #[test]
    fn shaft_friction_is_coulomb_plus_viscous_away_from_rest() {
        let f = ShaftFriction { coulomb: 0.02, viscous: 1e-4, smoothing: 0.5 };
        assert!((f.torque(100.0) - (0.02 + 0.01)).abs() < 1e-12);
        assert!((f.torque(-100.0) + (0.02 + 0.01)).abs() < 1e-12);
        assert_eq!(f.torque(0.0), 0.0);
    }

    #[test]
    fn ripple_and_cogging_are_periodic_and_invertible() {
        let r = Ripple { ripple: 0.02, ripple_order: 24.0, cogging: 0.01, cogging_order: 24.0 };
        let quarter = std::f64::consts::TAU / 24.0 / 4.0;
        assert!((r.apply(1.0, quarter) - (1.02 + 0.01)).abs() < 1e-12);
        assert!((r.apply(1.0, 0.3) - r.apply(1.0, 0.3 + std::f64::consts::TAU / 24.0)).abs() < 1e-12);
        assert!((r.apply(r.invert(0.7, 0.3), 0.3) - 0.7).abs() < 1e-12);
    }

    #[test]
    fn tooth_mesh_repeats_once_per_tooth() {
        let t = ToothMesh { force: 2.0, pitch: 0.005 };
        assert!((t.force(0.00125) - 2.0).abs() < 1e-12);
        assert!((t.force(0.001) - t.force(0.006)).abs() < 1e-9);
    }

    fn gearbox() -> Gearbox {
        Gearbox { ratio: 5.0, stiffness: 1e4, damping: 0.0, half_backlash: 4e-4, efficiency: 1.0 }
    }

    #[test]
    fn gearbox_transmits_nothing_inside_the_backlash() {
        let g = gearbox();
        // Output-side twist of +-4e-4 rad is free: that is 8e-4 rad of lost motion in total.
        assert_eq!(g.torque(5.0 * 3.9e-4, 0.0, 0.0, 0.0).output, 0.0);
        assert_eq!(g.torque(-5.0 * 3.9e-4, 0.0, 0.0, 0.0).output, 0.0);
        let t = g.torque(5.0 * 5e-4, 0.0, 0.0, 0.0);
        assert!((t.output - 1e4 * 1e-4).abs() < 1e-9 && (t.rotor - t.output / 5.0).abs() < 1e-12);
        assert!((g.static_twist(1.0) - (1e-4 + 4e-4)).abs() < 1e-15);
        assert!((g.torque(5.0 * g.static_twist(-2.0), 0.0, 0.0, 0.0).output + 2.0).abs() < 1e-9);
    }

    #[test]
    fn gearbox_flanks_in_contact_never_pull() {
        let mut g = gearbox();
        g.damping = 5.0;
        // Engaged on the positive side by 1e-4 rad (1 N m of spring torque), output running away at 1 rad/s.
        let t = g.torque(5.0 * 5e-4, 0.0, 0.0, 1.0);
        assert_eq!((t.output, t.rotor), (0.0, 0.0));
        // Closing instead, the damping adds to the spring.
        assert!((g.torque(5.0 * 5e-4, 0.0, 0.0, -1.0).output - 6.0).abs() < 1e-9);
        // Without backlash the gearbox is a plain spring and damper, in both directions.
        g.half_backlash = 0.0;
        assert!((g.torque(5.0 * 1e-4, 0.0, 0.0, 1.0).output + 4.0).abs() < 1e-9);
    }

    fn feedback() -> ThermalFeedback {
        ThermalFeedback { ambient: 293.15, reference: 293.15, viscosity_slope: 4000.0, gearbox_viscous_share: 0.5, belt_coefficient: 3e-4, efficiency: 0.95 }
    }

    /// The temperature laws at fixed temperatures, by hand. B 4000 K, ambient and reference 20 °C. 30 K above:
    /// nu / nu_ref = exp(4000 (1 / 323.15 - 1 / 293.15)) = exp(-1.26672) = 0.281749; the gearbox of 95 % with half its
    /// loss viscous then has 1 - 0.05 (0.5 + 0.5 * 0.281749) = 0.967956. At -10 °C ambient the oil is 4.73776 times
    /// as thick and the gearbox has 1 - 0.05 (0.5 + 2.36888) = 0.856556. At ambient = reference nothing changes; at
    /// 40 °C ambient the belt has 1 - 3e-4 * 20 = 0.994 of its EA.
    #[test]
    fn the_temperature_laws_follow_their_formulas() {
        let t = feedback();
        let close = |a: f64, b: f64| assert!((a - b).abs() < 1e-6, "{a} against {b}");
        close(t.viscosity_factor(0.0), 1.0);
        close(t.gearbox_efficiency(0.0), 0.95);
        close(t.belt_stiffness_factor(), 1.0);
        close(t.viscosity_factor(30.0), 0.281749);
        close(t.gearbox_efficiency(30.0), 0.967956);
        let cold = ThermalFeedback { ambient: 263.15, ..feedback() };
        close(cold.viscosity_factor(0.0), 4.737759);
        close(cold.gearbox_efficiency(0.0), 0.856556);
        close(ThermalFeedback { ambient: 313.15, ..feedback() }.belt_stiffness_factor(), 0.994);
        // A gearbox without losses keeps none, warm or cold.
        close(ThermalFeedback { efficiency: 1.0, ..cold }.gearbox_efficiency(0.0), 1.0);
    }

    /// The teeth climb out above climb times the slack side's tension: with climb 6, 100 N and 700 N climb, 100 N and
    /// 600 N do not, and a slack side climbs at any pull. A jump runs towards the tight side against the ridge, 600 N
    /// (6 * 100 N) times sin(2 pi d / p): at d = p / 4 the full 600 N, its energy 600 N * 5 mm / pi = 0.954930 J at
    /// the top; it ends where the belt crosses the next groove (counted) or the one it left (not counted).
    #[test]
    fn a_tooth_jump_starts_at_the_climb_limit_and_ends_one_pitch_on() {
        let jump = ToothJump { pitch: 0.005, climb: 6.0, mass: 0.26 };
        assert!(jump.climbs(100.0, 700.1) && jump.climbs(700.1, 100.0) && !jump.climbs(100.0, 600.0) && jump.climbs(0.0, 1.0));
        let mut slip = [0.0; JUMP_STATES];
        let mut v = 1.0;
        assert!(!jump.update(&mut slip, 100.0, 600.0, 1.0, &mut v) && slip == [0.0; JUMP_STATES]);
        assert_eq!(jump.pulley_force(&slip, -500.0), -500.0);
        // Section b is the tight one: the belt slides towards it, -u.
        assert!(jump.update(&mut slip, 100.0, 701.0, 1.0, &mut v) && slip == [0.0, 0.0, -1.0, 0.0, 0.0, 600.0] && v == 1.0);
        assert_eq!((jump.pulley_inertia(&slip, 1e-3, 0.02), jump.pulley_inertia(&[0.0; JUMP_STATES], 1e-3, 0.02)), (1e-3 - 0.26 * 4e-4, 1e-3));
        slip[0] = -0.00125;
        assert!((jump.pulley_force(&slip, -601.0) + 600.0).abs() < 1e-9);
        let mut rates = [0.0; JUMP_STATES];
        jump.rates(&slip, -601.0, 2.0, &mut rates);
        assert!((rates[1] - (-1.0 / 0.26 - 2.0)).abs() < 1e-9 && rates[0] == 0.0, "{rates:?}");
        slip[0] = -0.0025;
        slip[1] = -1.0;
        // 1/2 0.26 ((1 - 1)^2 - 1^2) = -0.13 J, and the ridge's top.
        assert!((jump.slip_energy(&slip, 1.0) - (-0.13 + 0.954930)).abs() < 1e-6, "{}", jump.slip_energy(&slip, 1.0));
        assert_eq!(jump.slip_energy(&[0.0; JUMP_STATES], 1.0), 0.0);
        (slip[0], slip[1]) = (-0.0049, -1.0);
        assert!(!jump.update(&mut slip, 100.0, 701.0, 1.0, &mut v));
        // Landed one pitch on: a pulley of 1 kg at its surface at 1 m/s and the belt of 0.26 kg at 0 m/s run on at
        // 1 / 1.26 = 0.793651 m/s.
        slip[0] = -0.0051;
        assert!(jump.update(&mut slip, 100.0, 701.0, 1.0, &mut v) && slip == [-0.005, 0.0, 0.0, 0.0, 1.0, 0.0], "{slip:?}");
        assert!((v - 0.793651).abs() < 1e-6, "{v}");
        // Back over the groove it left: in again where it was, not counted.
        let mut back = [0.0101, -0.1, 1.0, 0.01, 2.0, 50.0];
        let mut v = 0.0;
        assert!(!jump.update(&mut back, 0.0, 0.0, 1.0, &mut v));
        back[0] = 0.0099;
        assert!(jump.update(&mut back, 0.0, 0.0, 1.0, &mut v) && back == [0.01, 0.0, 0.0, 0.01, 2.0, 0.0], "{back:?}");
    }

    /// A span of 0.25 m at 300 N with 0.1 kg/m: f1 = sqrt(3000) / 0.5 = 109.545 Hz. Excited at 100 Hz it is 8.71 %
    /// off; with nothing exciting, or slack, it counts as 1. The drive at 1 m/s of belt with a 5 mm pitch meshes at
    /// 200 Hz, and a motor at 250 rad/s with ripple and cogging of order 24 adds 954.930 Hz.
    #[test]
    fn a_span_vibrates_as_a_string_and_is_judged_against_the_excitations() {
        let f = span_frequency(0.25, 300.0, 0.1);
        assert!((f - 109.544512).abs() < 1e-6, "{f}");
        assert!((span_detuning(f, &[0.0, 100.0, 300.0]) - 0.087129).abs() < 1e-6);
        assert_eq!((span_detuning(f, &[0.0]), span_detuning(0.0, &[100.0]), span_frequency(0.25, 0.0, 0.1)), (1.0, 1.0, 0.0));
        let ripple = Ripple { ripple: 0.02, ripple_order: 24.0, cogging: 0.01, cogging_order: 24.0 };
        let mut out = Vec::new();
        drive_excitations(0.005, -1.0, &ripple, -250.0, &mut out);
        assert!(out.len() == 3 && (out[0] - 200.0).abs() < 1e-9 && (out[1] - 954.929659).abs() < 1e-6 && out[1] == out[2], "{out:?}");
        out.clear();
        drive_excitations(0.005, 1.0, &Ripple { ripple: 0.0, cogging: 0.0, ..ripple }, 250.0, &mut out);
        assert_eq!(out.len(), 1);
    }

    #[test]
    fn gearbox_efficiency_loses_torque_in_the_direction_of_power_flow() {
        let mut g = gearbox();
        g.efficiency = 0.9;
        let twist = 5.0 * g.static_twist(2.0);
        let driving = g.torque(twist, 0.0, 0.0, 50.0);
        let driven = g.torque(twist, 0.0, 0.0, -50.0);
        assert!((driving.output - 2.0 * 0.9).abs() < 1e-9, "{}", driving.output);
        assert!((driven.output - 2.0 / 0.9).abs() < 1e-9, "{}", driven.output);
        assert!((g.torque(twist, 0.0, 0.0, 0.0).output - 2.0).abs() < 1e-12);
    }
}
