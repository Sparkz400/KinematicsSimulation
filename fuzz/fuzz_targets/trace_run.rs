//! Traces as `sim-cli run` writes them (`Trace::parse`) and frequency responses (`identify::parse_sweep`).
#![no_main]

use libfuzzer_sys::fuzz_target;
use sim_components::identify::parse_sweep;
use sim_components::trace::Trace;
use sim_core::MotorInfo;

fuzz_target!(|text: &str| {
    let motors = [MotorInfo { id: "x", key: "" }, MotorInfo { id: "y", key: "" }];
    if let Ok(trace) = Trace::parse(text, &motors) {
        assert!(trace.time.windows(2).all(|w| w[1] > w[0]));
        assert!(trace.motors.iter().all(|m| m.angle.len() == trace.time.len()));
    }
    let _ = parse_sweep(text);
});
