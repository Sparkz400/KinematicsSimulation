//! Mechanical components: belt, guide friction, shaft friction, gearbox, torque ripple, tooth meshing.
//! Each reads its own parameters and its own switch from the config and knows no machine.

use sim_core::{Config, ConfigError};

type Result<T> = std::result::Result<T, ConfigError>;

fn temperature_scale(cfg: &Config, leaf: &str) -> Result<f64> {
    Ok(if cfg.bool("effects.temperature")? { cfg.positive(&format!("temperature.{leaf}"))? } else { 1.0 })
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
    pub pretension: f64,
    pub slack: bool,
    pub mass_per_m: f64,
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
        Ok(Self {
            ea: cfg.positive("belt.stiffness_ea_n")? * temperature_scale(cfg, "belt_stiffness_scale")?,
            damping: if cfg.bool("effects.belt_damping")? { cfg.f64("belt.damping_n_s_m")? } else { 0.0 },
            pretension: cfg.f64("belt.pretension_n")?,
            slack: cfg.bool("effects.belt_slack")?,
            mass_per_m: if cfg.bool("effects.belt_mass")? { cfg.f64("belt.mass_kg_m")? } else { 0.0 },
        })
    }

    /// Unstretched belt in a section of length `geo` at pretension.
    pub fn rest_length(&self, geo: f64) -> f64 {
        geo / (1.0 + self.pretension / self.ea)
    }

    pub fn section(&self, geo: f64, mat: f64, geo_rate: f64, mat_rate: f64) -> SectionForce {
        let e = geo - mat;
        let elastic = self.ea * e / mat;
        let total = elastic + self.damping * (geo_rate - mat_rate);
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
        let get = |leaf: &str| cfg.f64(&format!("{table}.{leaf}"));
        Ok(Self {
            on: cfg.bool("effects.guide_friction")?,
            sigma0: cfg.positive(&format!("{table}.bristle_stiffness_n_m"))?,
            sigma1: get("bristle_damping_n_s_m")?,
            viscous: get("viscous_n_s_m")? * scale,
            coulomb: cfg.positive(&format!("{table}.coulomb_n"))? * scale,
            stiction: cfg.positive(&format!("{table}.stiction_n"))? * scale,
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
        let s_dot = v - self.sigma0 * v.abs() * s / self.level(v, load_scale);
        (self.sigma0 * s + self.sigma1 * s_dot + self.viscous * v, s_dot)
    }
}

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
            coulomb: if on { cfg.f64("motor.friction_coulomb_nm")? } else { 0.0 },
            viscous: if on { cfg.f64("motor.friction_viscous_nm_s_rad")? } else { 0.0 },
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
        Ok(Self {
            ripple: if on { cfg.f64("motor.ripple_fraction")? } else { 0.0 },
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
        Ok(Self {
            ratio: cfg.positive("gearbox.ratio")?,
            stiffness: cfg.positive("gearbox.stiffness_nm_rad")?,
            damping: cfg.f64("gearbox.damping_nm_s_rad")?,
            half_backlash: if cfg.bool("effects.gearbox_backlash")? { 0.5 * cfg.f64("gearbox.backlash_rad")? } else { 0.0 },
            efficiency: if cfg.bool("effects.gearbox_efficiency")? { cfg.positive("gearbox.efficiency")? } else { 1.0 },
        })
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
        GearTorque { output: torque * self.loss_factor(torque, out_speed), rotor: torque / self.ratio }
    }

    /// Twist at rest that transmits `output_torque`, on the matching side of the backlash.
    pub fn static_twist(&self, output_torque: f64) -> f64 {
        if output_torque == 0.0 { 0.0 } else { output_torque / self.stiffness + self.half_backlash.copysign(output_torque) }
    }

    pub fn energy(&self, rotor_angle: f64, out_angle: f64) -> f64 {
        0.5 * self.stiffness * self.engaged_twist(rotor_angle / self.ratio - out_angle).powi(2)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn belt() -> Belt {
        Belt { ea: 5e5, damping: 0.0, pretension: 300.0, slack: true, mass_per_m: 0.0 }
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

    #[test]
    fn lugre_level_falls_from_stiction_to_coulomb() {
        let f = lugre();
        assert!((f.level(0.0, 1.0) - 20.0).abs() < 1e-12);
        assert!((f.level(1.0, 1.0) - 15.0).abs() < 1e-9);
        assert!(f.level(0.005, 1.0) < 20.0 && f.level(0.005, 1.0) > 15.0);
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
