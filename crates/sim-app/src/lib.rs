//! The app's logic without its window: the simulation session and the interface model.
//! `main.rs` and `ui.rs` draw it with Bevy and egui.

pub mod model;
pub mod runs;
pub mod session;
pub mod video;

/// (language code, translation file contents) for the app's own labels.
pub static I18N: &[(&str, &str)] = &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))];

/// The panels' typeface: Atkinson Hyperlegible Next, the Braille Institute's, under the SIL Open Font License
/// (`assets/fonts/OFL.txt`). Its default weight, Regular, is the one drawn.
pub static FONT: &[u8] = include_bytes!("../assets/fonts/AtkinsonHyperlegibleNext.ttf");
