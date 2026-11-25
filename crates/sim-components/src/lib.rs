//! Components: belt span, friction contact, gearbox, rotary joint, motor, drive, profile generator, encoder. Know no machine.

/// (language code, translation file contents) for everything the components name.
pub static I18N: &[(&str, &str)] = &[
    ("en", include_str!("../i18n/en.ftl")),
    ("de", include_str!("../i18n/de.ftl")),
    ("en", include_str!("../i18n/import.en.ftl")),
    ("de", include_str!("../i18n/import.de.ftl")),
];

pub mod accuracy;
pub mod analysis;
pub mod base;
pub mod beam;
pub mod cam;
pub mod catalog;
pub mod chain;
pub mod charts;
pub mod collision;
pub mod control;
pub mod conveyor;
pub mod dataset;
pub mod drive;
pub mod fault;
pub mod gripper;
pub mod identify;
pub mod import;
pub mod induction;
pub mod joint;
pub mod mech;
pub mod motor_fit;
pub mod parameters;
pub mod path;
pub mod pneumatic;
pub mod profile;
pub mod program;
pub mod report;
pub mod rigid;
pub mod search;
pub mod sensitivity;
pub mod shaper;
pub mod shapes;
pub mod sim;
pub mod stepper;
pub mod stop;
pub mod trace;
pub mod transmission;
pub mod tune;

pub use control::{Command, CommandError, Controller, LimitViolation, StopCategory, check_limits};
pub use drive::DriveMode;
pub use sim::{Contact, Sim, Snapshot, load_config, schema};

/// Refuses each of `effects` (`effects.<name>`) switched on in `cfg` (`error-effect-not-modelled`): switches a machine's
/// config has with the others of its crate but that this machine does not model, so that one turned on is not silently
/// without effect.
pub fn refuse_unmodelled(cfg: &sim_core::Config, effects: &[&str]) -> Result<(), sim_core::ConfigError> {
    match effects.iter().find(|e| cfg.bool(&format!("effects.{e}")) == Ok(true)) {
        Some(e) => Err(sim_core::ConfigError::Part("error-effect-not-modelled", format!("effects.{e}"))),
        None => Ok(()),
    }
}
