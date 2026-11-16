//! A time-optimal plan passes the verdicts it keeps within (15 November 2026): every machine with paths, its cycle timed
//! as fast as the motors allow (`profile.cycle_timing = "time_optimal"`), simulated with the config as shipped.

use sim_components::analysis::{analyse_cycle_with, verdicts};
use sim_components::path::Piece;
use sim_components::{Sim, load_config};
use sim_core::{Config, MachineEntry};
use sim_machines::MACHINES;

/// The verdicts whose limits the time-optimal plan reads (`path::plan_time_optimal`): the motor's curve, speed and the
/// drive's current, the gearbox's and the pulley's torque and the gearbox's input speed, the guides, the belt and the
/// braking resistor; besides them the machine's own checks that the rigid model gives (`Machine::rigid_checks`).
const KEPT: [&str; 10] = [
    "motor-peak",
    "motor-speed",
    "drive-peak-current",
    "gearbox-peak",
    "pulley-torque",
    "gearbox-input-speed",
    "guide-load",
    "belt-tension",
    "belt-slack",
    "braking-resistor-peak",
];

/// The machine's cycle, each segment timed as fast as the motors allow, run and judged: every verdict the plan keeps
/// within passes. A machine whose cycle is a cam has no path, and is left out.
fn keeps_its_verdicts(entry: &'static MachineEntry) {
    let mut cfg = entry.config();
    cfg.set_text("profile.cycle_timing = \"time_optimal\"").unwrap();
    let sim = Sim::new(entry, &cfg).unwrap_or_else(|e| panic!("{}: {e:?}", entry.id));
    if sim.machine.cam().is_some() {
        return;
    }
    // The machine's own checks that the rigid model gives a value for, at its start pose.
    let (start, zero) = (sim.control.planned_pos().to_vec(), vec![0.0; sim.machine.axes().len()]);
    let mut values = Vec::new();
    sim.machine.rigid_checks(&start, &zero, &zero, &mut values);
    let own: Vec<&'static str> =
        sim.machine.rating_checks().iter().zip(&values).filter(|(_, v)| v.is_finite()).map(|(c, _)| c.check).collect();
    let judged = |cfg: &Config| {
        let mut sim = Sim::new(entry, cfg).unwrap();
        let load = analyse_cycle_with(&mut sim, false).unwrap_or_else(|e| panic!("{}: {}", entry.id, e.key));
        (load.duration, verdicts(cfg, &load).unwrap())
    };
    let (duration, mut judgement) = judged(&cfg);
    // A winding that overheats on the cycle repeated ("motor-thermal", the RMS torque over the cycle, which no bound at
    // a point of the path keeps) runs away in temperature, and its resistance takes the torque the plan counts on: the
    // plan's own verdicts are then judged on the winding at its starting temperature.
    let overheats = judgement.iter().any(|v| v.check == "motor-thermal" && !v.ok);
    if overheats {
        cfg.set_text("effects.winding_heating = false").unwrap();
        judgement = judged(&cfg).1;
    }
    let failed: Vec<String> = judgement
        .into_iter()
        .filter(|v| !v.ok && (KEPT.contains(&v.check) || own.contains(&v.check)))
        .map(|v| format!("{} (motor {:?}): {} against {}", v.check, v.motor, v.value, v.limit))
        .collect();
    eprintln!("{}: cycle {duration:.4} s{}", entry.id, if overheats { ", overheats" } else { "" });
    assert!(failed.is_empty(), "{}: {failed:?}", entry.id);
}

/// One test per machine of `MACHINES` by its place in the list, as in the command line's tests.
macro_rules! per_machine {
    ($($test:ident $index:literal),*) => {
        $(#[test]
        fn $test() {
            keeps_its_verdicts(MACHINES[$index]);
        })*
        const PER_MACHINE_TESTS: usize = [$($index),*].len();
    };
}

per_machine!(machine_00 0, machine_01 1, machine_02 2, machine_03 3, machine_04 4, machine_05 5, machine_06 6,
    machine_07 7, machine_08 8, machine_09 9, machine_10 10, machine_11 11, machine_12 12, machine_13 13, machine_14 14,
    machine_15 15, machine_16 16, machine_17 17, machine_18 18, machine_19 19, machine_20 20, machine_21 21,
    machine_22 22,
    machine_23 23,
    machine_24 24,
    machine_25 25,
    machine_26 26,
    machine_27 27,
    machine_28 28,
    machine_29 29);

#[test]
fn every_machine_has_its_test() {
    assert_eq!(MACHINES.len(), PER_MACHINE_TESTS, "add or take out a `machine_NN` in `per_machine!`");
}

/// A turn about one rotary tool axis alone, a tenth of its range from the start pose, is timed as fast as the motors
/// allow on every machine that has one (the platforms' rx, ry, rz, the tripod's tilts, the four-axis delta's turn, the
/// arms' joints, and the arms in tool space): until 15 November a turn about z alone of the six-axis platform on cranks
/// was refused as a crawl (`error-path-singular`), because the speed curve fell faster than the jerk could follow.
#[test]
fn a_turn_about_one_rotary_axis_alone_is_timed_on_every_machine() {
    let root = std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../.."));
    let mut configs: Vec<(&MachineEntry, Config)> = MACHINES.iter().map(|e| (*e, e.config())).collect();
    for (id, file) in [("urdf-arm", "arm-tool.toml"), ("scara", "scara-tool.toml")] {
        let entry = sim_machines::find(id).unwrap();
        let text = std::fs::read_to_string(root.join("crates/machine-joint-kit/config").join(file)).unwrap();
        configs.push((entry, load_config(entry, &text).unwrap()));
    }
    let mut failed = Vec::new();
    for (entry, cfg) in &configs {
        let sim = Sim::new(entry, cfg).unwrap();
        if sim.machine.cam().is_some() {
            continue;
        }
        let start = sim.control.planned_pos().to_vec();
        for (i, axis) in sim.machine.axes().iter().enumerate().filter(|(_, a)| a.unit == "rad") {
            let mut target = start.clone();
            let turn = 0.1 * (axis.max - axis.min);
            target[i] = if start[i] + turn <= axis.max { start[i] + turn } else { start[i] - turn };
            if sim.machine.unreachable(&target, 0.0).is_some() {
                continue;
            }
            if let Err(e) = sim.plan_path(&[Piece::Line(target)], true) {
                failed.push(format!("{} {}: {}", entry.id, axis.id, e.key));
            }
        }
    }
    assert!(failed.is_empty(), "{failed:?}");
}
