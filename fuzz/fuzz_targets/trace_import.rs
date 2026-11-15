//! Drive traces through mapping files: the input (bytes, as `decode` reads a file) through a mapping for each vendor
//! layout (CSV, SINAMICS, TwinCAT, CODESYS), and as a mapping file itself.
#![no_main]

use libfuzzer_sys::fuzz_target;
use sim_components::import::{Mapping, decode, import};
use std::sync::OnceLock;

const MAPS: [&str; 4] = [
    r#"format = "csv"
time = "time"
[[motor]]
setpoint = { signal = "position setpoint", per_rev = 1048576 }
angle = { signal = "position actual", per_rev = 1048576, wrap = 4294967296 }
speed = "speed actual"
torque = "torque actual"
torque_setpoint = "torque setpoint"
"#,
    r#"format = "sinamics"
[[motor]]
setpoint = { signal = "r2665", per_rev = 10000 }
angle = { signal = "r2521", per_rev = 10000 }
speed = "r61"
torque = "r80"
torque_setpoint = "r79"
"#,
    r#"format = "twincat"
[[motor]]
setpoint = { signal = "SetPos", per_rev = 10.0 }
angle = { signal = "MAIN.Axis.NcToPlc.ActPos", per_rev = 10.0 }
speed = { signal = "ActVelo", per_rev = 10.0 }
torque = { signal = "TorqueActual", reference = 2.0 }
"#,
    r#"format = "codesys"
[[motor]]
setpoint = { signal = "Axis1.fSetPosition", unit = "mm", per_rev = 5.0 }
angle = { signal = "Axis1.fActPosition", unit = "mm", per_rev = 5.0 }
speed = { signal = "Axis1.fActVelocity", unit = "mm/s", per_rev = 5.0 }
torque = { signal = "Axis1.fActTorque", unit = "Nm" }
"#,
];

fuzz_target!(|bytes: &[u8]| {
    static PARSED: OnceLock<Vec<Mapping>> = OnceLock::new();
    let maps = PARSED.get_or_init(|| MAPS.iter().map(|m| Mapping::parse(m).unwrap()).collect());
    let text = decode(bytes);
    for map in maps {
        if let Ok(imported) = import(&text, map) {
            let t = &imported.trace;
            assert!(t.time.windows(2).all(|w| w[1] > w[0]));
            assert!(t.motors.iter().all(|m| m.angle.iter().chain(&m.torque).all(|v| v.is_finite())));
        }
    }
    if let Ok(map) = Mapping::parse(&text) {
        for sample in ["time,a\n0,1\n0.001,2\n", "t;x\n0;1\n1;2\n"] {
            let _ = import(sample, &map);
        }
    }
});
