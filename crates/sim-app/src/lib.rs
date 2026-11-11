//! The app's logic without its window: the simulation session and the interface model.
//! `main.rs` and `ui.rs` draw it with Bevy and egui.

pub mod model;
pub mod session;

/// (language code, translation file contents) for the app's own labels.
pub static I18N: &[(&str, &str)] = &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))];
