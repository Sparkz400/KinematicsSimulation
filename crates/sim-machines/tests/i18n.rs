//! No text for the user is written in code: every machine, axis, motor, signal, parameter, group and
//! effect has a key with a German and an English entry. This fails when one is missing, and when a machine's label
//! comes from another machine's files rather than its own or the shared components' (before 15 November 2026 the last
//! registered machine's wording won wherever a machine had none of its own: the arms' joints read "Leg 1" or "Crank 1").

use sim_core::Translations;
use sim_machines::{LANGUAGES, MACHINES, translations};

/// What a machine may name its labels from: core, the shared components and its own crate.
fn own(lang: &str, entry: &sim_core::MachineEntry) -> Translations {
    let mut out = Translations::default();
    let sources = [sim_core::i18n::I18N, sim_components::I18N, entry.i18n].into_iter().flatten();
    sources.filter(|(l, _)| *l == lang).for_each(|(_, text)| out.add(text));
    out
}

#[test]
fn every_key_has_an_entry_in_every_language() {
    assert!(LANGUAGES.contains(&"de") && LANGUAGES.contains(&"en"));
    let mut wrong = Vec::new();
    for entry in MACHINES {
        let config = entry.config();
        let machine = (entry.build)(&config).unwrap();
        let mut keys = vec![entry.key.to_string()];
        keys.extend(machine.axes().iter().map(|a| a.key.to_string()));
        keys.extend(machine.motors().iter().map(|m| m.key.to_string()));
        keys.extend(machine.rating_checks().iter().map(|c| format!("check-{}", c.check)));
        for signal in sim_components::Sim::new(entry, &config).unwrap().signal_infos() {
            keys.extend([signal.owner_key, signal.quantity_key]);
        }
        let mut scene = sim_core::Scene::default();
        sim_components::Sim::new(entry, &config).unwrap().scene(&mut scene);
        assert!(!scene.labels.is_empty(), "{}: the scene shows no live values", entry.id);
        for label in &scene.labels {
            keys.extend([label.owner_key.to_string(), label.unit_key.to_string()]);
        }
        for (path, _) in config.leaves() {
            let (group, leaf) = path.rsplit_once('.').unwrap_or(("", &path));
            keys.push(if group == "effects" { format!("effect-{leaf}") } else { format!("param-{leaf}") });
            if !group.is_empty() {
                keys.push(format!("group-{}", group.replace('.', "-")));
            }
        }
        keys.extend(
            ["parse", "version", "missing", "type", "invalid", "unknown", "part-motor", "part-gearbox"]
                .map(|e| format!("error-config-{e}")),
        );
        keys.extend(
            [
                "error-target-out-of-range",
                "error-busy",
                "error-no-cycle",
                "error-disabled",
                "error-fault",
                "monitor-following-error",
                "monitor-standstill",
                "monitor-blocked",
                "monitor-torque-band",
                "reaction-off1",
                "reaction-off3",
                "reaction-sto",
                "reaction-current-limit",
                "error-setpoint-too-far",
                "error-path",
                "error-path-first-arc",
                "error-path-range",
                "error-path-torque",
                "error-path-guide",
                "limit-motor-speed",
                "limit-motor-torque",
            ]
            .map(String::from),
        );
        for lang in LANGUAGES {
            let (tr, own) = (translations(lang, entry), own(lang, entry));
            for key in &keys {
                assert!(tr.has(key), "{}: no '{lang}' entry for '{key}'", entry.id);
                if !own.has(key) || tr.tr(key) != own.tr(key) {
                    wrong.push(format!(
                        "{} {lang} {key} {}: '{}'",
                        entry.id,
                        if own.has(key) { "differs" } else { "missing" },
                        tr.tr(key)
                    ));
                }
            }
        }
    }
    wrong.dedup();
    assert!(wrong.is_empty(), "labels from another machine's files:\n{}", wrong.join("\n"));
}
