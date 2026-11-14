//! Delta picker D3 (Clavel): three motor-driven arms at 120 degrees on a base, each carrying a parallelogram of two
//! forearms down to a platform that stays level. The tool is the platform's centre, x, y, z.
//!
//! The machine is the spatial delta of machine files (`sim_netlist::spatial`, its module comment derives the
//! kinematics and dynamics), read from this crate's tables: `[geometry]`, `[mass]`, `[upper_arm]`, `[workspace]`, one
//! `[joint]`/`[motor]`/`[gearbox]` for all three arms. The file machines/delta-d3.toml is the same machine written as a
//! netlist, so a fix to the arms reaches both.

use sim_core::{Config, ConfigError, Machine, MachineEntry};
pub use sim_netlist::spatial::*;

pub static ENTRY: MachineEntry = MachineEntry {
    id: "delta-d3",
    key: "machine-delta-d3",
    default_config: include_str!("../default.toml"),
    i18n: &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))],
    build,
    suite_configs,
};

/// Effects that take energy out or put it in; the standard suite runs with them off. The stops stay, without their
/// damping: the suite's pulses would otherwise send the arms round beyond where the legs can be assembled.
pub const LOSSY_EFFECTS: [&str; 7] = [
    "joint_friction",
    "motor_friction",
    "gearbox_drag",
    "gearbox_lost_motion",
    "gearbox_efficiency",
    "torque_ripple",
    "holding_brake",
];

fn build(cfg: &Config) -> Result<Box<dyn Machine>, ConfigError> {
    Ok(Box::new(Delta::new(cfg)?))
}

/// The start pose and the cycle's targets moved by `dz` in z, with the working range (see `suite_configs`).
fn lowered(cfg: &mut Config, dz: f64) {
    let start = cfg.f64s("start.pose").unwrap();
    cfg.set_text(&format!("start.pose = [{:?}, {:?}, {:?}]", start[0], start[1], start[2] + dz)).unwrap();
    let segments: Vec<String> = cfg
        .array("cycle.segment")
        .unwrap()
        .iter()
        .map(|s| {
            let t: Vec<f64> = s["target"].as_array().unwrap().iter().map(|v| v.as_float().unwrap()).collect();
            format!(
                "{{ target = [{:?}, {:?}, {:?}], dwell_s = {:?}, payload = {} }}",
                t[0],
                t[1],
                t[2] + dz,
                s["dwell_s"].as_float().unwrap(),
                s["payload"].as_bool().unwrap()
            )
        })
        .collect();
    cfg.set_text(&format!("cycle.segment = [{}]", segments.join(", "))).unwrap();
}

/// The default config with every loss switched off.
pub fn lossless_config() -> Config {
    let mut cfg = ENTRY.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    cfg.set_text("gearbox.damping_nm_s_rad = 0.0").unwrap();
    cfg.set_text("joint.stop_damping_nm_s_rad = 0.0").unwrap();
    cfg.set_text("upper_arm.bending_damping_nm_s_rad = 0.0").unwrap();
    cfg.set_text("upper_arm.lateral_damping_n_s_m = 0.0").unwrap();
    cfg
}

/// Loss-free rigid and elastic models, with and without the payload, and the elastic one with part heating and the
/// temperature feedback (no loss, so the gearboxes stay at ambient).
///
/// Their working range, start pose and cycle lie 175 mm lower than the default's (top 850 mm below the shoulders), so
/// that the suite's middle pose is 1043 mm down: held by constant motor torques, as the suite holds it for 10 s after a pulse, the delta
/// is unstable in its vertical mode in the upper part of its range, where the platform's weight pulls harder through
/// the Jacobian the further it sinks (negative stiffness; `the_vertical_mode_under_constant_torques_is_unstable_high_up`),
/// and stable below about 950 mm. A real property, not a numerical one: the drives' position loops hold it.
fn suite_configs() -> Vec<Config> {
    let mut out = Vec::new();
    for (elastic, payload) in [(false, true), (false, false), (true, true), (true, false)] {
        let mut cfg = lossless_config();
        cfg.set_text("workspace.top_z_m = -0.85").unwrap();
        lowered(&mut cfg, -0.175);
        cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
        cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
        cfg.set_text(&format!("effects.part_heating = {}", elastic && payload)).unwrap();
        cfg.set_text(&format!("effects.thermal_feedback = {}", elastic && payload)).unwrap();
        out.push(cfg);
    }
    out
}

#[cfg(test)]
mod tests;
