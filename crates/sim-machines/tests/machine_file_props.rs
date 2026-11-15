//! Property tests of machine files (`load_file`, TOML and JSON; `load_text`) and of the registered machines' configs as
//! a front end loads them (`sim::load_config`, then `Sim::new`): whatever the text, a machine or an error value, never a
//! panic or a hang. Seeds: machines/*.toml, the same as JSON, and each machine's default config.

#[path = "../../sim-core/tests/edits/mod.rs"]
mod edits;

use proptest::prelude::*;
use sim_components::Sim;
use sim_components::sim::load_config;
use sim_machines::{MACHINES, load_file, load_text};
use std::sync::atomic::{AtomicUsize, Ordering};

fn as_json(toml_text: &str) -> String {
    let value: toml::Table = toml_text.parse().unwrap();
    serde_json::to_string_pretty(&value).unwrap()
}

fn json_files() -> Vec<(String, String)> {
    edits::files("machines", ".toml").into_iter().map(|(name, text)| (name, as_json(&text))).collect()
}

/// `text` written to a file of its own with `extension`, loaded as a front end loads it.
fn load(text: &str, extension: &str) {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    let n = COUNT.fetch_add(1, Ordering::Relaxed);
    let path = std::env::temp_dir().join(format!("machine-file-props-{}-{n}.{extension}", std::process::id()));
    std::fs::write(&path, text).unwrap();
    if let Ok(entry) = load_file(&path) {
        let _ = Sim::new(entry, &entry.config());
    }
    let _ = std::fs::remove_file(&path);
}

fn defaults() -> Vec<(String, String)> {
    MACHINES.iter().map(|m| (m.id.to_string(), m.default_config.to_string())).collect()
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    #[test]
    fn edited_toml_machine_files_are_loaded_or_refused((_, text) in edits::edited(edits::files("machines", ".toml"))) {
        load(&text, "toml");
        let _ = load_text(&text, "machine");
    }

    #[test]
    fn edited_json_machine_files_are_loaded_or_refused((_, text) in edits::edited(json_files())) {
        load(&text, "json");
    }

    #[test]
    fn any_text_as_a_machine_file_is_refused(text in ".{0,300}") {
        load(&text, "toml");
        load(&text, "json");
    }

    #[test]
    fn edited_machine_configs_are_loaded_or_refused((id, text) in edits::edited(defaults())) {
        let entry = sim_machines::find(&id).unwrap();
        if let Ok(cfg) = load_config(entry, &text) {
            let _ = Sim::new(entry, &cfg);
        }
    }
}
