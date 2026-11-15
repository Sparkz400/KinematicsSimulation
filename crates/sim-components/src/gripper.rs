//! The gripper of a pick-and-place cycle (effects.gripper_time, `[gripper]`): how long it takes to grip and to let go,
//! and for a vacuum gripper what it holds (the verdict "gripper-hold").
//!
//! - `kind = "mechanical"`: jaws that close in `close_s` and open in `open_s`.
//! - `kind = "vacuum"`: a cup of volume V evacuated by a generator of suction flow Q builds up its vacuum as a first-order
//!   lag towards the generator's `vacuum_pa`, dp(t) = p_v (1 - exp(-t Q / V)): the grip is made when the vacuum switch
//!   sees `holding_pressure_pa`, after t = V / Q ln(p_v / (p_v - p_hold)). Vented with the flow `vent_flow_m3_s`, the
//!   vacuum falls as p_v exp(-t Q_vent / V) and lets go at `release_pressure_pa`, after V / Q_vent ln(p_v / p_release).
//!   The cup of area A then holds A p_hold along its axis, the least the switch lets the move start with.
//!
//! The cup sits above the part, its axis vertical (z up). The part of mass m at the tool's acceleration a pulls off the
//! cup with m (a_z + g) and pushes sideways with m |a_xy|, which the cup holds by the friction coefficient mu on what the
//! pull leaves of its force: it slips when m |a_xy| > mu (A p_hold - m (a_z + g)), that is when the force the part asks,
//! m (a_z + g) + m |a_xy| / mu, exceeds A p_hold (`demand`). The verdict judges the most the part asked over the cycle
//! against A p_hold / `ratings.gripper_safety_factor`.

use sim_core::{Config, ConfigError};

/// Standard gravity, m/s^2.
const G: f64 = 9.80665;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Gripper {
    /// Time to grip and to let go, s.
    pub close_s: f64,
    pub open_s: f64,
    /// A vacuum gripper's hold: (A p_hold in N, the cup's friction coefficient on the part). `None` for jaws.
    pub hold: Option<(f64, f64)>,
}

impl Gripper {
    /// With effects.gripper_time (off where the config has no switch), from `[gripper]`.
    pub fn from_config(cfg: &Config) -> Result<Option<Self>, ConfigError> {
        if !(cfg.get("effects.gripper_time").is_ok() && cfg.bool("effects.gripper_time")?) {
            return Ok(None);
        }
        let get = |leaf: &str| cfg.positive(&format!("gripper.{leaf}"));
        match cfg.get("gripper.kind")?.as_str() {
            Some("mechanical") => Ok(Some(Self { close_s: get("close_s")?, open_s: get("open_s")?, hold: None })),
            Some("vacuum") => {
                let (volume, vacuum, hold) = (get("cup_volume_m3")?, get("vacuum_pa")?, get("holding_pressure_pa")?);
                let release = get("release_pressure_pa")?;
                if hold >= vacuum {
                    return Err(ConfigError::Invalid("gripper.holding_pressure_pa".into()));
                }
                if release >= vacuum {
                    return Err(ConfigError::Invalid("gripper.release_pressure_pa".into()));
                }
                Ok(Some(Self {
                    close_s: volume / get("generator_flow_m3_s")? * (vacuum / (vacuum - hold)).ln(),
                    open_s: volume / get("vent_flow_m3_s")? * (vacuum / release).ln(),
                    hold: Some((get("cup_area_m2")? * hold, get("friction_coefficient")?)),
                }))
            }
            _ => Err(ConfigError::Invalid("gripper.kind".into())),
        }
    }
}

/// Gravity for the hold: g with effects.gravity, else 0.
pub fn gravity(cfg: &Config) -> Result<f64, ConfigError> {
    Ok(if cfg.bool("effects.gravity")? { G } else { 0.0 })
}

/// The force a part of `mass` at the tool's acceleration `acc` ([x, y, z], m/s^2) asks of a cup above it with the
/// friction coefficient `mu`, N: m (a_z + g) + m |a_xy| / mu (see the module).
pub fn demand(mass: f64, acc: [f64; 3], g: f64, mu: f64) -> f64 {
    mass * (acc[2] + g) + mass * acc[0].hypot(acc[1]) / mu
}

/// The payload's mass, from the table the machine keeps it in (`mass`, `arm` or `platform`); 0 with effects.payload off.
pub fn payload_mass(cfg: &Config) -> Result<f64, ConfigError> {
    if !cfg.bool("effects.payload")? {
        return Ok(0.0);
    }
    ["mass", "arm", "platform"]
        .iter()
        .find_map(|table| cfg.f64(&format!("{table}.payload_kg")).ok())
        .ok_or_else(|| ConfigError::Missing("mass.payload_kg".into()))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg(text: &str) -> Config {
        Config::parse(&format!("format_version = 1\n[effects]\ngripper_time = true\n[gripper]\n{text}")).unwrap()
    }

    /// Vacuum by hand: V = 20 cm^3, Q = 10 l/min = 1.6667e-4 m^3/s, tau = 0.12 s; to 60 of the generator's 80 kPa:
    /// 0.12 ln(80 / 20) = 0.166355 s. Vented at 40 l/min (tau 0.03 s) to 10 kPa: 0.03 ln 8 = 0.062383 s. A cup of
    /// 30 mm (7.0686e-4 m^2) at 60 kPa holds 42.4116 N. Jaws: as configured.
    #[test]
    fn the_vacuum_gripper_takes_the_hand_calculated_times_and_holds_area_times_pressure() {
        let v = Gripper::from_config(&cfg(
            "kind = \"vacuum\"\ncup_volume_m3 = 2.0e-5\ngenerator_flow_m3_s = 1.6666666666666666e-4\nvacuum_pa = 80000.0\n\
             holding_pressure_pa = 60000.0\nvent_flow_m3_s = 6.666666666666667e-4\nrelease_pressure_pa = 10000.0\n\
             cup_area_m2 = 7.0686e-4\nfriction_coefficient = 0.5\n",
        ))
        .unwrap()
        .unwrap();
        assert!((v.close_s - 0.12 * 4f64.ln()).abs() < 1e-12 && (v.open_s - 0.03 * 8f64.ln()).abs() < 1e-12);
        assert!((v.close_s - 0.166355).abs() < 1e-6 && (v.open_s - 0.062383).abs() < 1e-6);
        assert_eq!(v.hold, Some((7.0686e-4 * 60000.0, 0.5)));
        let jaws = Gripper::from_config(&cfg("kind = \"mechanical\"\nclose_s = 0.05\nopen_s = 0.04\n")).unwrap();
        assert_eq!(jaws, Some(Gripper { close_s: 0.05, open_s: 0.04, hold: None }));
        assert!(Gripper::from_config(&cfg("kind = \"magnetic\"\n")).is_err());
    }

    /// A 2 kg part under a cup holding 42.4116 N with mu 0.5, accelerated sideways: it slips beyond
    /// a = mu (A dp - m g) / m = 0.5 (42.4116 - 19.6133) / 2 = 5.69958 m/s^2, where it asks exactly A dp.
    #[test]
    fn the_part_asks_the_hold_exactly_at_the_hand_calculated_slip_acceleration() {
        let (hold, m, mu) = (7.0686e-4 * 60000.0, 2.0, 0.5);
        let slip = mu * (hold - m * G) / m;
        assert!((slip - 5.69958).abs() < 1e-5);
        assert!((demand(m, [slip, 0.0, 0.0], G, mu) - hold).abs() < 1e-9);
        assert!(demand(m, [0.0, slip * 0.999, 0.0], G, mu) < hold && demand(m, [slip * 1.001, 0.0, 0.0], G, mu) > hold);
    }
}
