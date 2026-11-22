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

/// The stepper's detent (T_d = 0.04 N m, N = 50: its potential swings by T_d / (2N) = 0.4 mJ) across a disable and an
/// enable. The stepper axis without its losses moves, is disabled mid-move and coasts 50 ms on the detent alone (which
/// its magnets hold without current), is enabled again off its grid's full step and pulled there: over all of it the
/// drive's electrical energy less copper, less what the magnets gave (`emf`), less the change of the winding's stored
/// energy, is the machine's change of energy and its own losses (the belt's tooth mesh and so on, the trapezoid over each
/// step). Before 22 November the potential sat in the store while enabled only and the detent let go when disabled:
/// the books were out by 9.8e-5 J at the end of these moves, against 2.3e-6 J now (2.1e-6 J with the detent off).
#[test]
fn the_steppers_detent_neither_appears_nor_vanishes_at_a_disable_or_an_enable() {
    use sim_components::{Command, Sim};
    let entry = file("stepper-axis.toml");
    let mut cfg = entry.config();
    for off in ["dc_link", "guide_friction", "motor_friction", "belt_damping", "winding_heating"] {
        cfg.set_text(&format!("effects.{off} = false")).unwrap();
    }
    let mut sim = Sim::new(entry, &cfg).unwrap();
    let stored = |sim: &Sim| {
        let d = &sim.control.drives[0];
        d.magnetic_energy(sim.control.drive_params_of(0)) + sim.machine.energy(&sim.state)
    };
    let start = stored(&sim);
    let lossy: Vec<usize> = (sim.machine.signal_infos().iter().enumerate())
        .filter(|(_, s)| s.id.starts_with(sim_core::LOSS_PREFIX))
        .map(|(i, _)| i)
        .collect();
    // The drive's electrical energy less copper, what the magnets gave, the machine's own losses, and the largest
    // torque while disabled.
    let mut books = [0.0, 0.0, 0.0, 0f64];
    let loss = |sim: &Sim| {
        let mut signals = Vec::new();
        sim.machine.signals(&sim.state, &sim.machine_torque(), &mut signals);
        lossy.iter().map(|&i| signals[i]).sum::<f64>()
    };
    let step = |sim: &mut Sim, books: &mut [f64; 4], n: usize| {
        for _ in 0..n {
            let before = loss(sim);
            sim.step();
            let d = &sim.control.drives[0];
            books[0] += (d.power - d.copper_loss) * sim.dt;
            books[1] += d.emf_loss * sim.dt;
            // The trapezoid over the step.
            books[2] += 0.5 * (before + loss(sim)) * sim.dt;
            if !sim.control.enabled() {
                books[3] = books[3].max(d.torque.abs());
            }
        }
    };
    let open = |sim: &Sim, books: &[f64; 4]| books[0] - books[1] - books[2] - (stored(sim) - start);
    sim.command(Command::MoveTo(vec![sim.control.planned_pos()[0] + 0.2])).unwrap();
    step(&mut sim, &mut books, 1600);
    sim.command(Command::Disable).unwrap();
    step(&mut sim, &mut books, 800);
    sim.command(Command::Enable).unwrap();
    step(&mut sim, &mut books, 3200);
    let ([drawn, gave, lost, detent_disabled], open) = (books, open(&sim, &books));
    println!(
        "drawn {drawn} J, the magnets {gave} J, lost {lost} J, open {open:e} J; the detent disabled up to \
         {detent_disabled} N m"
    );
    assert!(detent_disabled > 0.5 * 0.04, "the detent while disabled: {detent_disabled} N m");
    // The same moves with the detent off leave 2.1e-6 J open (the machine's own integration): within a twentieth of
    // the potential's T_d / (4 N) = 2e-4 J.
    assert!(open.abs() < 0.05 * 0.04 / 200.0, "{open} J open of {drawn} J");
}
