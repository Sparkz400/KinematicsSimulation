//! The other motors (motor.kind, crates/sim-components/src/stepper.rs and induction.rs) on machine files of their own:
//! each runs its cycle and is judged with the verdicts that apply to it.

use sim_components::Sim;
use sim_components::analysis::{BUDGET_TOLERANCE, ELECTRICAL_TOLERANCE, Verdict, analyse_cycle, verdicts};
use sim_core::{Config, MachineEntry};
use sim_machines::load_file;
use std::path::Path;

fn file(name: &str) -> &'static MachineEntry {
    load_file(&Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines").join(name))
        .unwrap_or_else(|e| panic!("{e}"))
}

/// The cycle's verdicts; and its energy budget closes, the electrical side (windings, DC link) as for the servo.
fn judged(entry: &'static MachineEntry, cfg: &Config) -> Vec<Verdict> {
    let load = analyse_cycle(&mut Sim::new(entry, cfg).unwrap()).unwrap();
    let budget = load.budget.as_ref().unwrap();
    assert!(budget.electrical_unaccounted().abs() < ELECTRICAL_TOLERANCE * budget.supply, "{budget:?}");
    assert!(budget.unaccounted().abs() < BUDGET_TOLERANCE * budget.supply, "{budget:?}");
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

/// The same lift laid flat as a conveyor and run open loop on V/f (drive.induction_control = "vf"): the cycle runs, the
/// converter following the plan's speed with the slip compensated, and nothing holds the position, so the carriage
/// stops off its targets by what the slip and the speed ramps leave. (Its warm start, "thermal-warm-start", settles 0.4 K
/// from where the pass started: the settled rise assumes a current that stays as the winding warms, and on V/f the
/// voltage is set, not the current.)
#[test]
fn the_induction_axis_runs_its_cycle_on_v_f() {
    let entry = file("induction-lift.toml");
    let mut cfg = entry.config();
    cfg.set_text("geometry.incline_deg = 0.0").unwrap();
    cfg.set_text("drive.induction_control = \"vf\"").unwrap();
    let v = judged(entry, &cfg);
    for ok in ["motor-peak", "motor-thermal", "drive-current", "drive-peak-current"] {
        assert!(check(&v, ok).ok, "{:?}", check(&v, ok));
    }
    // 9.4 mm off its target in the dwells, against 0.2 mm field-oriented: open loop.
    let settling = check(&v, "settling");
    assert!(settling.value > 2e-3 && settling.value < 0.02, "{settling:?}");
}
