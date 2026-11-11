//! No text for the user is written in code: every machine, axis, motor, signal, parameter, group and
//! effect has a key with a German and an English entry. This fails when one is missing.

use sim_machines::{LANGUAGES, MACHINES, translations};

#[test]
fn every_key_has_an_entry_in_every_language() {
    assert!(LANGUAGES.contains(&"de") && LANGUAGES.contains(&"en"));
    for entry in MACHINES {
        let config = entry.config();
        let machine = (entry.build)(&config).unwrap();
        let mut keys = vec![entry.key.to_string()];
        keys.extend(machine.axes().iter().map(|a| a.key.to_string()));
        keys.extend(machine.motors().iter().map(|m| m.key.to_string()));
        for signal in sim_components::Sim::new(entry, &config).unwrap().signal_infos() {
            keys.extend([signal.owner_key, signal.quantity_key]);
        }
        for (path, _) in config.leaves() {
            let (group, leaf) = path.rsplit_once('.').unwrap_or(("", &path));
            keys.push(if group == "effects" { format!("effect-{leaf}") } else { format!("param-{leaf}") });
            if !group.is_empty() {
                keys.push(format!("group-{}", group.replace('.', "-")));
            }
        }
        keys.extend(["parse", "version", "missing", "type", "invalid"].map(|e| format!("error-config-{e}")));
        keys.extend(["error-target-out-of-range", "error-busy", "error-no-cycle", "limit-motor-speed", "limit-motor-torque"].map(String::from));
        for lang in LANGUAGES {
            let tr = translations(lang, entry);
            for key in &keys {
                assert!(tr.has(key), "{}: no '{lang}' entry for '{key}'", entry.id);
            }
        }
    }
}
