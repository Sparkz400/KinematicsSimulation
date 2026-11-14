//! Machines built from one motor-driven joint per axis (`sim_components::chain::JointMachine`), one for each kind of
//! transmission and mechanism the components offer, so that each can be run and sized:
//!
//! - `screw-axis`: a carriage on a ball screw (`slide`).
//! - `rack-axis`: a carriage driven by a pinion on a rack through a planetary gearbox (`slide`).
//! - `linear-motor-axis`: a carriage on an ironcore linear motor, no transmission (`slide`).
//! - `rotary-table`: an indexing table behind a strain wave or worm gear (`table`).
//! - `urdf-arm`: a serial six-axis arm whose bodies come from a URDF file (`arm`).
//! - `scara`: a SCARA arm from a URDF file, its vertical joint on a ball screw (`arm`).
//!
//! Each has its own default config in `config/`: `config/common.toml` with what all share, then the machine's file.

mod arm;
mod slide;
mod table;
mod tool;

use sim_components::chain::{JOINT_KEYS, JointMachine, JointSpec, MOTOR_IDS, MOTOR_KEYS, Mechanism};
use sim_components::joint::RotaryJoint;
use sim_components::mech::ThermalFeedback;
use sim_core::{AxisInfo, Config, ConfigError, Machine, MachineEntry, MotorInfo};

const I18N: &[(&str, &str)] = &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))];

macro_rules! entry {
    ($name:ident, $id:literal, $file:literal, $build:expr) => {
        pub static $name: MachineEntry = MachineEntry {
            id: $id,
            key: concat!("machine-", $id),
            default_config: concat!(
                include_str!("../config/common.toml"),
                "\n",
                include_str!(concat!("../config/", $file))
            ),
            i18n: I18N,
            build: |cfg| sim_components::base::mount(cfg, ($build)(cfg)?),
            suite_configs: || suite_configs(&$name),
        };
    };
}

entry!(SCREW, "screw-axis", "screw.toml", |cfg| slide::build(cfg, slide::Kind::Screw));
entry!(RACK, "rack-axis", "rack.toml", |cfg| slide::build(cfg, slide::Kind::Rack));
entry!(LINEAR_MOTOR, "linear-motor-axis", "linear-motor.toml", |cfg| slide::build(cfg, slide::Kind::LinearMotor));
entry!(ROTARY_TABLE, "rotary-table", "table.toml", table::build);
entry!(URDF_ARM, "urdf-arm", "arm.toml", arm::build);
entry!(SCARA, "scara", "scara.toml", arm::build);

/// The full configs of the SCARA and the six-axis arm in tool space (`kinematics.tool_space = true`), with a cycle of
/// straight lines of the tool: for `--config`.
pub const SCARA_TOOL_CONFIG: &str = include_str!("../config/scara-tool.toml");
pub const ARM_TOOL_CONFIG: &str = include_str!("../config/arm-tool.toml");

const G: f64 = 9.80665;

/// Effects that take energy out or put it in; the standard suite runs with them off.
pub const LOSSY_EFFECTS: [&str; 7] = [
    "joint_friction",
    "motor_friction",
    "gearbox_drag",
    "gearbox_lost_motion",
    "gearbox_efficiency",
    "torque_ripple",
    "holding_brake",
];

/// The default config with every loss switched off.
pub fn lossless_config(entry: &MachineEntry) -> Config {
    let mut cfg = entry.config();
    for effect in LOSSY_EFFECTS {
        cfg.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    cfg.set_text("gearbox.damping_nm_s_rad = 0.0").unwrap();
    let stops = if cfg.get("slide.stop_damping_n_s_m").is_ok() {
        "slide.stop_damping_n_s_m"
    } else {
        "joint.stop_damping_nm_s_rad"
    };
    cfg.set_text(&format!("{stops} = 0.0")).unwrap();
    cfg
}

/// Loss-free rigid and elastic models, with and without the payload; the elastic one with the payload also with part
/// heating and the temperature feedback (no loss, so nothing warms).
///
/// The six-axis arm without gravity: held by constant torques, as the suite holds it after its pulse, an arm whose
/// upper arm stands up is an inverted pendulum and falls away from the pose, and the energy it then trades between
/// height and speed dwarfs the pulse's (a real property, not a numerical one: the position loops hold it). Gravity's
/// part of the dynamics is checked on the default config (static hold, loss budget) and in `rigid`'s tests.
fn suite_configs(entry: &MachineEntry) -> Vec<Config> {
    [(false, true), (false, false), (true, true), (true, false)]
        .into_iter()
        .map(|(elastic, payload)| {
            let mut cfg = lossless_config(entry);
            cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
            cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
            cfg.set_text(&format!("effects.part_heating = {}", elastic && payload)).unwrap();
            cfg.set_text(&format!("effects.thermal_feedback = {}", elastic && payload)).unwrap();
            if entry.id == "urdf-arm" {
                cfg.set_text("effects.gravity = false").unwrap();
            }
            cfg
        })
        .collect()
}

fn gravity(cfg: &Config) -> Result<f64, ConfigError> {
    Ok(if cfg.bool("effects.gravity")? { G } else { 0.0 })
}

fn payload(cfg: &Config, key: &str) -> Result<f64, ConfigError> {
    Ok(if cfg.bool("effects.payload")? { cfg.non_negative(key)? } else { 0.0 })
}

/// An axis whose limits are `axis.<id>.max_velocity_<unit>_s`, `max_acceleration_<unit>_s2` and `max_jerk_<unit>_s3`.
fn axis(
    cfg: &Config,
    id: &'static str,
    key: &'static str,
    unit: &'static str,
    (min, max): (f64, f64),
) -> Result<AxisInfo, ConfigError> {
    let get = |what: &str, rate: &str| cfg.positive(&format!("axis.{id}.max_{what}_{unit}_{rate}"));
    Ok(AxisInfo {
        id,
        key,
        unit,
        min,
        max,
        max_vel: get("velocity", "s")?,
        max_acc: get("acceleration", "s2")?,
        max_jerk: get("jerk", "s3")?,
    })
}

/// A joint machine of `joints` (each with its gear lever, `None` where the joint is driven without a converter, and
/// its unit, "m" or "rad") and `axes`, one motor per joint.
fn machine<M: Mechanism + 'static>(
    cfg: &Config,
    mechanism: M,
    joints: Vec<(RotaryJoint, Option<f64>, &'static str)>,
    axes: Vec<AxisInfo>,
) -> Result<Box<dyn Machine>, ConfigError> {
    let specs = joints
        .into_iter()
        .enumerate()
        .map(|(k, (drive, gear_lever, unit))| JointSpec {
            drive,
            id: MOTOR_IDS[k],
            key: JOINT_KEYS[k],
            gear_lever,
            unit,
        })
        .collect::<Vec<_>>();
    let motors = (0..specs.len()).map(|k| MotorInfo { id: MOTOR_IDS[k], key: MOTOR_KEYS[k] }).collect();
    Ok(Box::new(JointMachine::new(
        mechanism,
        specs,
        cfg.bool("effects.joint_elasticity")?,
        ThermalFeedback::from_config(cfg)?,
        axes,
        motors,
    )))
}

/// `ratings.required_life_h`, where it is set.
fn required_life(cfg: &Config) -> Option<f64> {
    cfg.f64("ratings.required_life_h").ok()
}

#[cfg(test)]
mod tests;
