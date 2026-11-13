//! The registration list: the one place outside a machine crate that names a machine.
//! App, CLI and PLC link pick every machine up from here.

use sim_core::{Config, ConfigError, MachineEntry, Translations};
use std::path::Path;

pub use sim_netlist::FileError;

pub static MACHINES: &[&MachineEntry] = &[
    &machine_t_gantry::ENTRY,
    &machine_single_axis::ENTRY,
    &machine_delta_d3::ENTRY,
    &machine_joint_kit::SCREW,
    &machine_joint_kit::RACK,
    &machine_joint_kit::LINEAR_MOTOR,
    &machine_joint_kit::ROTARY_TABLE,
    &machine_joint_kit::URDF_ARM,
    &machine_joint_kit::SCARA,
];

pub const LANGUAGES: &[&str] = &["en", "de"];

pub fn find(id: &str) -> Option<&'static MachineEntry> {
    MACHINES.iter().copied().find(|m| m.id == id)
}

/// Everything core, components and machines name, in one language. Front ends add their own labels.
/// `machine` selects whose axis and parameter names apply; machines may reuse keys such as `axis-x`.
pub fn translations(lang: &str, machine: &MachineEntry) -> Translations {
    let mut out = Translations::default();
    let sources = [sim_core::i18n::I18N, sim_components::I18N].into_iter();
    // The selected machine comes last, so its own wording wins where keys repeat.
    let machines = MACHINES.iter().filter(|m| m.id != machine.id).chain([&machine]).map(|m| m.i18n);
    for (_, text) in sources.chain(machines).flatten().filter(|(l, _)| *l == lang) {
        out.add(text);
    }
    out
}

/// A machine from a machine file (TOML, or JSON by its extension), checked in full against the tables the registered
/// machines have; see docs/machine-files.md. Front ends take it like an entry of `MACHINES`. An error names the line
/// in a TOML file; in a JSON file the key alone.
pub fn load_file(path: &Path) -> Result<&'static MachineEntry, FileError> {
    let fail = |error, line| FileError { file: path.into(), line, error };
    let text = std::fs::read_to_string(path).map_err(|e| fail(ConfigError::Parse(e.to_string()), None))?;
    let json = path.extension().is_some_and(|e| e.eq_ignore_ascii_case("json"));
    let toml_text = if json { json_to_toml(&text).map_err(|e| fail(e, None))? } else { text };
    let known: Vec<Config> = MACHINES.iter().map(|m| m.config()).collect();
    let stem = path.file_stem().and_then(|s| s.to_str()).unwrap_or("machine");
    sim_netlist::from_text(&toml_text, !json, stem, &known).map_err(|(error, line)| fail(error, line))
}

/// A JSON machine file as the TOML text of the same config.
fn json_to_toml(text: &str) -> Result<String, ConfigError> {
    let mut value: serde_json::Value = serde_json::from_str(text).map_err(|e| ConfigError::Parse(format!("line {}: {e}", e.line())))?;
    // JSON has one kind of number: a version written 1.0 is the version 1 the JSON Schema accepts.
    for version in ["/format_version", "/machine/version"] {
        if let Some(v) = value.pointer_mut(version).filter(|v| v.as_f64().is_some_and(|f| f.fract() == 0.0 && f.abs() < 1e9)) {
            *v = serde_json::Value::from(v.as_f64().unwrap_or_default() as i64);
        }
    }
    let toml::Value::Table(table) = toml::Value::try_from(value).map_err(|e| ConfigError::Parse(e.to_string()))? else {
        return Err(ConfigError::Parse("a machine file is a JSON object".into()));
    };
    toml::to_string(&table).map_err(|e| ConfigError::Parse(e.to_string()))
}

/// The JSON Schema of a machine file (`sim_netlist::json_schema`) as text: what schema/machine-file.schema.json holds.
pub fn json_schema() -> String {
    serde_json::to_string_pretty(&sim_netlist::json_schema()).expect("a TOML value is written as JSON") + "
"
}
