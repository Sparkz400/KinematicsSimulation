//! Config loading and `--set path=value`: the input as a config, then each of its lines as an assignment to it.
#![no_main]

use libfuzzer_sys::fuzz_target;
use sim_core::Config;

fuzz_target!(|text: &str| {
    let Ok(mut cfg) = Config::parse(text) else { return };
    for (path, _) in cfg.leaves() {
        let _ = (cfg.f64(&path), cfg.positive(&path), cfg.bool(&path), cfg.f64s(&path));
    }
    let written = cfg.to_text().expect("a parsed config is written");
    assert_eq!(Config::parse(&written).expect("what to_text writes is read").to_text().unwrap(), written);
    for line in text.lines() {
        let _ = cfg.set_text(line);
    }
});
