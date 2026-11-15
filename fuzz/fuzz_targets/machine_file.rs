//! Machine files: the input as the TOML text of a machine file (`load_text`), and as a JSON file (`load_file`).
#![no_main]

use libfuzzer_sys::fuzz_target;
use std::sync::OnceLock;

fuzz_target!(|text: &str| {
    if let Ok(entry) = sim_machines::load_text(text, "fuzz") {
        let _ = sim_components::Sim::new(entry, &entry.config());
    }
    static JSON: OnceLock<std::path::PathBuf> = OnceLock::new();
    let path = JSON.get_or_init(|| std::env::temp_dir().join(format!("fuzz-machine-{}.json", std::process::id())));
    if text.trim_start().starts_with('{') && std::fs::write(path, text).is_ok() {
        let _ = sim_machines::load_file(path);
    }
});
