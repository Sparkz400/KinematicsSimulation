//! Cam tables (`cam::parse_table`, `Cam::from_table`) and trajectories of poses at times (`path::parse_trajectory`).
#![no_main]

use libfuzzer_sys::fuzz_target;
use sim_components::cam::{Cam, parse_table};
use sim_components::path::parse_trajectory;

fuzz_target!(|text: &str| {
    if let Ok(rows) = parse_table(text)
        && let Ok(cam) = Cam::from_table(&rows)
    {
        for k in 0..=8 {
            assert!(cam.at(k as f64 * cam.period / 8.0).iter().all(|v| v.is_finite()));
        }
    }
    if let Ok(points) = parse_trajectory(text) {
        assert!(points.iter().all(|(t, p)| t.is_finite() && p.iter().all(|x| x.is_finite())));
    }
});
