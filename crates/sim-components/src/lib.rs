//! Components: belt span, friction contact, gearbox, motor, drive, profile generator, encoder. Know no machine.

/// (language code, translation file contents) for everything the components name.
pub static I18N: &[(&str, &str)] = &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))];

pub mod mech;
