//! Crank and cam mechanisms, the core cases of a servo sizing tool, each a motor-driven joint
//! (`sim_components::chain::JointMachine`) that can run an electronic cam (`sim_components::cam`) as its cycle:
//!
//! - `slider-crank`: a crank, a connecting rod and a slider on a guide, as in presses and pushers (`crank`).
//! - `rotary-knife`: a knife roll cutting a moving web to format (`knife`).
//! - `flying-saw`: a saw on a carriage that runs with the material while it cuts and returns (`saw`).
//! - `disc-cam`: a cam disc turned by the servo lifting a roller follower on a slide against its spring (`disc`): the
//!   mechanical cam, its lift law, contact force and pressure angle.
//!
//! With `cam.run = false` (as shipped) the cycle is the config's `[[cycle.segment]]`, as on every machine: the crank
//! swings through its working stroke and back as a servo press in pendulum mode does, the knife indexes, the saw's
//! carriage moves. With `cam.run = true` the controller runs the mechanism's cam instead: the master (virtual, or the
//! web or material) comes up to speed in `cam.ramp_s` and the slave follows; one pass is one master period.
//!
//! Each has its config in `config/`: `config/common.toml` with what all share, then the machine's file; beside it
//! `<id>-cam.toml`, the same config with `cam.run = true`, a whole config to load.

mod crank;
mod disc;
mod knife;
mod saw;

use sim_components::chain::{JOINT_KEYS, JointMachine, JointSpec, MOTOR_IDS, MOTOR_KEYS, Mechanism};
use sim_components::joint::RotaryJoint;
use sim_components::mech::ThermalFeedback;
use sim_core::{AxisInfo, Config, ConfigError, Machine, MachineEntry, MotorInfo};

pub use crank::SliderCrank;
pub use disc::DiscCam;
pub use knife::RotaryKnife;
pub use saw::FlyingSaw;

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
            build: $build,
            suite_configs: || suite_configs(&$name),
        };
    };
}

entry!(SLIDER_CRANK, "slider-crank", "slider-crank.toml", crank::build);
entry!(ROTARY_KNIFE, "rotary-knife", "rotary-knife.toml", knife::build);
entry!(FLYING_SAW, "flying-saw", "flying-saw.toml", saw::build);
entry!(DISC_CAM, "disc-cam", "disc-cam.toml", disc::build);

const G: f64 = 9.80665;

/// Effects that take energy out or put it in; the standard suite runs with them off.
pub const LOSSY_EFFECTS: [&str; 6] =
    ["joint_friction", "motor_friction", "gearbox_efficiency", "torque_ripple", "holding_brake", "process_force"];

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

/// Loss-free rigid and elastic models, with and without the payload.
fn suite_configs(entry: &MachineEntry) -> Vec<Config> {
    [(false, true), (false, false), (true, true), (true, false)]
        .into_iter()
        .map(|(elastic, payload)| {
            let mut cfg = lossless_config(entry);
            cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
            cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
            cfg
        })
        .collect()
}

fn gravity(cfg: &Config) -> Result<f64, ConfigError> {
    Ok(if cfg.bool("effects.gravity")? { G } else { 0.0 })
}

fn payload(cfg: &Config) -> Result<f64, ConfigError> {
    Ok(if cfg.bool("effects.payload")? { cfg.non_negative("mass.payload_kg")? } else { 0.0 })
}

/// The master of `[cam]` where `cam.run` is on: its speed, the period of `cam`, and `cam.ramp_s`.
fn master(cfg: &Config, speed: f64, period: f64) -> Result<Option<sim_core::CamMaster>, ConfigError> {
    if !cfg.bool("cam.run")? {
        return Ok(None);
    }
    Ok(Some(sim_core::CamMaster { speed, period, ramp_s: cfg.positive("cam.ramp_s")? }))
}

/// The axis `id` over `range`, its limits `axis.<id>.max_velocity_<unit>_s`, `max_acceleration_<unit>_s2` and
/// `max_jerk_<unit>_s3`.
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

/// A joint machine of one joint (with its gear lever, `None` where it has no converter, and its unit) and its axis.
/// Where a cam runs its slave on and on (`endless`), the joint's stops go out of its way: the axis range stays what
/// moves and paths may use.
fn machine<M: Mechanism + 'static>(
    cfg: &Config,
    mechanism: M,
    (mut drive, gear_lever, unit): (RotaryJoint, Option<f64>, &'static str),
    axis: AxisInfo,
    endless: bool,
) -> Result<Box<dyn Machine>, ConfigError> {
    if endless {
        // shortcut: a crank or knife that turns on under the cam has no stops at all; model a limit switch if one matters.
        drive.range = (-1e12, 1e12);
    }
    let spec = JointSpec { drive, id: MOTOR_IDS[0], key: JOINT_KEYS[0], gear_lever, unit };
    Ok(Box::new(JointMachine::new(
        mechanism,
        vec![spec],
        cfg.bool("effects.joint_elasticity")?,
        ThermalFeedback::from_config(cfg)?,
        vec![axis],
        vec![MotorInfo { id: MOTOR_IDS[0], key: MOTOR_KEYS[0] }],
    )))
}

/// A table of `<prefix>.<at>` against `<prefix>.<value>`, two arrays of the same length with `at` rising: linear between
/// its points and 0 outside them (a press force that starts and ends with the table).
#[derive(Debug, Clone, Default)]
struct Table {
    at: Vec<f64>,
    value: Vec<f64>,
}

impl Table {
    fn from_config(cfg: &Config, at: &str, value: &str) -> Result<Self, ConfigError> {
        let read = |key: &str| -> Result<Vec<f64>, ConfigError> {
            cfg.array(key)?
                .iter()
                .map(|v| v.as_float().or_else(|| v.as_integer().map(|i| i as f64)).filter(|f| f.is_finite()))
                .collect::<Option<Vec<_>>>()
                .ok_or_else(|| ConfigError::Type(key.into()))
        };
        let (a, v) = (read(at)?, read(value)?);
        if a.is_empty() || a.len() != v.len() || a.windows(2).any(|w| w[1] <= w[0]) {
            return Err(ConfigError::Invalid(at.into()));
        }
        Ok(Self { at: a, value: v })
    }

    fn at(&self, x: f64) -> f64 {
        let (a, v) = (&self.at, &self.value);
        if a.is_empty() || x < a[0] || x > a[a.len() - 1] {
            return 0.0;
        }
        let i = a.partition_point(|p| *p <= x).clamp(1, a.len().max(2) - 1);
        if a.len() == 1 {
            return v[0];
        }
        let u = (x - a[i - 1]) / (a[i] - a[i - 1]);
        v[i - 1] + u * (v[i] - v[i - 1])
    }
}

/// The share of a process force that acts at speed `v` in the direction it works in: 0 at rest, below `smoothing` and
/// backwards, 1 from twice `smoothing` on, linear between: the force fades out where the motion turns, not in a step,
/// and a tool at rest in the material does not chatter against it on the speed loop's noise.
fn forwards(v: f64, smoothing: f64) -> f64 {
    (v / smoothing - 1.0).clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests;
