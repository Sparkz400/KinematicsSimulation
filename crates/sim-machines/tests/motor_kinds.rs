//! The other motors (motor.kind, crates/sim-components/src/stepper.rs and induction.rs) on machine files of their own:
//! each runs its cycle and is judged with the verdicts that apply to it.

use sim_components::Sim;
use sim_components::analysis::{Verdict, analyse_cycle, verdicts};
use sim_core::{Config, MachineEntry};
use sim_machines::load_file;
use std::path::Path;

fn file(name: &str) -> &'static MachineEntry {
    load_file(&Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines").join(name))
        .unwrap_or_else(|e| panic!("{e}"))
}

fn judged(entry: &'static MachineEntry, cfg: &Config) -> Vec<Verdict> {
    let load = analyse_cycle(&mut Sim::new(entry, cfg).unwrap()).unwrap();
    verdicts(cfg, &load).unwrap()
}

fn check<'a>(v: &'a [Verdict], name: &str) -> &'a Verdict {
    v.iter().find(|v| v.check == name).unwrap_or_else(|| panic!("no {name}: {v:?}"))
}

/// The stepper belt axis (machines/stepper-axis.toml) runs its cycle in step: the load angle stays within the pull-out
/// angle, the torque within the pull-out curve, every check passes, and Siemens' S1 check, a servo motor's, is not made.
/// As shipped: load angle 0.42 rad (24 degrees) at most, 44 % of the pull-out curve, the winding at 59 K of its 80 K.
/// With a 25 kg payload the 5 m/s^2 alone ask for 26 kg * 5 m/s^2 * 0.016 m = 2.08 N m, over the holding torque
/// 0.318 N m/A * 3.5 A = 1.11 N m: the rotor falls out of step, the drive goes on counting, and "stepper-synchronism"
/// fails by more than pi (steps lost).
#[test]
fn the_stepper_axis_runs_its_cycle_in_step_and_a_heavy_payload_makes_it_lose_steps() {
    let entry = file("stepper-axis.toml");
    let cfg = entry.config();
    let v = judged(entry, &cfg);
    let failed: Vec<_> = v.iter().filter(|v| !v.ok).collect();
    assert!(failed.is_empty(), "{failed:?}");
    let sync = check(&v, "stepper-synchronism");
    assert!(sync.value > 0.1 && sync.value < std::f64::consts::FRAC_PI_2, "{sync:?}");
    assert!(check(&v, "motor-peak").value < 1.0);
    assert!(v.iter().all(|v| v.check != "motor-s1"));
    let mut heavy = cfg.clone();
    heavy.set_text("mass.payload_kg = 25.0").unwrap();
    let sync = check(&judged(entry, &heavy), "stepper-synchronism").clone();
    assert!(!sync.ok && sync.value > std::f64::consts::PI, "{sync:?}");
}

/// The induction motor lift (machines/induction-lift.toml), field-oriented under the drive's speed and position loops,
/// runs its cycle and passes every check; Siemens' S1 check is not made. Holding the 50 kg at rest asks for
/// 50 kg * 9.81 m/s^2 * 0.05 m / 10 = 2.45 N m at the motor (the gearbox's loss on top), a quarter of the rated torque.
#[test]
fn the_induction_lift_runs_its_cycle_field_oriented() {
    let entry = file("induction-lift.toml");
    let cfg = entry.config();
    let v = judged(entry, &cfg);
    let failed: Vec<_> = v.iter().filter(|v| !v.ok).collect();
    assert!(failed.is_empty(), "{failed:?}");
    assert!(v.iter().all(|v| v.check != "motor-s1" && v.check != "stepper-synchronism"));
}
