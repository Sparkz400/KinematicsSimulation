//! The URDF reader machine-joint-kit builds its arms with (`Chain::from_urdf`).
#![no_main]

use libfuzzer_sys::fuzz_target;
use sim_components::rigid::Chain;

fuzz_target!(|text: &str| {
    if let Ok(chain) = Chain::from_urdf(text) {
        for b in &chain.bodies {
            assert!(b.inertial.inertia.iter().flatten().chain(&b.inertial.com).all(|v| v.is_finite()));
            assert!((b.axis.iter().map(|a| a * a).sum::<f64>() - 1.0).abs() < 1e-9);
        }
    }
});
