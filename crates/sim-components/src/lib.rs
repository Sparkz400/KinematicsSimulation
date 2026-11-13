//! Components: belt span, friction contact, gearbox, rotary joint, motor, drive, profile generator, encoder. Know no machine.

/// (language code, translation file contents) for everything the components name.
pub static I18N: &[(&str, &str)] = &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))];

pub mod analysis;
pub mod catalog;
pub mod control;
pub mod drive;
pub mod identify;
pub mod joint;
pub mod mech;
pub mod path;
pub mod profile;
pub mod sim;
pub mod trace;

pub use control::{Command, CommandError, Controller, LimitViolation, check_limits};
pub use sim::{Sim, load_config};
