//! Property tests of config loading (`Config::parse`) and of `--set path=value` (`Config::set_text`): whatever the text,
//! an `Ok` or a `ConfigError`, never a panic; and what `to_text` writes, `parse` reads back to the same text.

#[path = "edits/mod.rs"]
mod edits;

use proptest::prelude::*;
use sim_core::Config;

/// Every machine config the repository ships: the machine crates' TOML files and the machine files.
fn configs() -> Vec<(String, String)> {
    let mut out = edits::files("machines", ".toml");
    for dir in std::fs::read_dir(edits::root().join("crates")).unwrap() {
        let dir = dir.unwrap().path();
        for file in std::fs::read_dir(&dir).unwrap() {
            let p = file.unwrap().path();
            if p.extension().is_some_and(|e| e == "toml") && p.file_name().is_some_and(|n| n != "Cargo.toml") {
                out.push((p.display().to_string(), std::fs::read_to_string(&p).unwrap()));
            }
        }
    }
    out
}

/// Reads every value a config has in every way a machine may ask for it, and writes it back.
fn exercise(cfg: &Config) {
    for (path, _) in cfg.leaves() {
        let _ = (cfg.f64(&path), cfg.positive(&path), cfg.non_negative(&path), cfg.bool(&path), cfg.f64s(&path));
    }
    let text = cfg.to_text().expect("a parsed config is written");
    let again = Config::parse(&text).expect("what to_text writes is read");
    assert_eq!(again.to_text().unwrap(), text);
}

proptest! {
    #[test]
    fn any_bytes_are_read_or_refused(bytes in prop::collection::vec(any::<u8>(), 0..2000)) {
        if let Ok(cfg) = Config::parse(&String::from_utf8_lossy(&bytes)) {
            exercise(&cfg);
        }
    }

    #[test]
    fn edited_configs_are_read_or_refused((_, text) in edits::edited(configs())) {
        if let Ok(cfg) = Config::parse(&text) {
            exercise(&cfg);
        }
    }

    #[test]
    fn set_text_takes_or_refuses_any_assignment(
        (_, text) in prop::sample::select(configs()),
        leaf in any::<prop::sample::Index>(),
        value in prop_oneof![edits::number(), ".{0,30}"],
        junk in ".{0,30}",
    ) {
        let Ok(mut cfg) = Config::parse(&text) else { return Ok(()) };
        let leaves = cfg.leaves();
        prop_assume!(!leaves.is_empty());
        let path = &leaves[leaf.index(leaves.len())].0;
        for assignment in [format!("{path}={value}"), format!("{path} = {junk}"), junk.clone(), format!("{junk}={value}")] {
            let before = cfg.clone();
            match cfg.set_text(&assignment) {
                Ok(()) => exercise(&cfg),
                Err(_) => prop_assert_eq!(cfg.to_text().unwrap(), before.to_text().unwrap()),
            }
        }
    }
}
