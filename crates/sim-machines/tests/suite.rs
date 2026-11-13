//! Every registered machine passes the same standard suite, unchanged.

use sim_machines::MACHINES;

#[test]
fn every_machine_passes_the_standard_suite() {
    for entry in MACHINES {
        sim_core::testkit::run_standard_suite(entry);
    }
}

#[test]
fn every_machine_describes_a_scene() {
    for entry in MACHINES {
        let machine = (entry.build)(&entry.config()).unwrap();
        let (mut state, mut scene) = (Vec::new(), sim_core::Scene::default());
        machine.equilibrium(&sim_core::testkit::mid_pose(machine.as_ref()), &mut state, &mut vec![0.0; machine.motors().len()]);
        machine.scene(&state, &mut scene);
        assert!(scene.parts.len() >= 3, "{}", entry.id);
    }
}

#[test]
fn ids_are_unique_and_findable() {
    for entry in MACHINES {
        assert_eq!(MACHINES.iter().filter(|m| m.id == entry.id).count(), 1);
        assert!(sim_machines::find(entry.id).is_some());
    }
    assert!(sim_machines::find("no-such-machine").is_none());
}

/// The scene shows the motion: when the machine moves, pulley spokes and the motor shaft bars
/// turn, belt teeth travel, and the speed labels show the motor speeds in revolutions per minute. A machine without
/// belts (it reports no belt sections) has no pulleys and teeth to show; its motor shaft bars and labels are checked.
#[test]
fn the_scene_shows_pulleys_turning_teeth_travelling_and_motor_speeds() {
    use sim_components::{Command, Sim};
    for entry in MACHINES {
        let mut sim = Sim::new(entry, &entry.config()).unwrap();
        let (mut before, mut after) = (sim_core::Scene::default(), sim_core::Scene::default());
        sim.scene(&mut before);
        assert!(before.labels.iter().filter(|l| l.unit_key == "unit-rpm").all(|l| l.value.abs() < 1.0), "{}", entry.id);
        sim.command(Command::StartCycle).unwrap();
        sim.run(0.4, |_| {});
        // On to where an axis moves at a third of its speed limit, so that the motors' labels have speeds to show.
        while !sim.control.set_vel.iter().zip(sim.machine.axes()).any(|(v, a)| v.abs() > a.max_vel / 3.0) {
            sim.step();
        }
        sim.scene(&mut after);
        let belted = !sim.machine.belt_sections().is_empty();
        let angles = |scene: &sim_core::Scene, id: &str| scene.parts.iter().filter(|p| p.id == id).map(|p| p.angle).collect::<Vec<_>>();
        for id in if belted { &["pulley-spoke", "rotor-bar", "idler-bar"][..] } else { &["rotor-bar"][..] } {
            let (a, b) = (angles(&before, id), angles(&after, id));
            assert!(!a.is_empty() && a.len() == b.len(), "{}: no {id}", entry.id);
            assert!(a.iter().zip(&b).any(|(a, b)| (a - b).abs() > 0.5), "{}: {id} did not turn", entry.id);
        }
        if belted {
            // The motor shaft turns faster than the pulley by the gear ratio.
            let ratio = entry.config().f64("gearbox.ratio").unwrap();
            let turned = |id: &str| angles(&after, id)[0] - angles(&before, id)[0];
            assert!((turned("rotor-bar") / turned("pulley-spoke") - ratio).abs() < 0.05 * ratio, "{}", entry.id);
            let teeth = |scene: &sim_core::Scene| scene.parts.iter().filter(|p| p.id == "belt-tooth").map(|p| p.pos).collect::<Vec<_>>();
            let (a, b) = (teeth(&before), teeth(&after));
            assert!(a.len() > 40 && a.len().abs_diff(b.len()) <= 1, "{}: {} teeth before, {} after", entry.id, a.len(), b.len());
            assert!(a.iter().zip(&b).filter(|(a, b)| (a[0] - b[0]).abs() + (a[2] - b[2]).abs() > 0.01).count() > 20, "{}: the teeth did not travel", entry.id);
        }
        // The labels show what the signals say, in rpm.
        let mut sample = Vec::new();
        sim.sample(&mut sample);
        for (motor, label) in sim.machine.motors().iter().zip(after.labels.iter().filter(|l| l.unit_key == "unit-rpm")) {
            let speed = sample[sim.signal_index(&format!("motor.{}.speed", motor.id)).unwrap()];
            assert!((label.value - speed * 60.0 / std::f64::consts::TAU).abs() < 1e-9 && label.value.abs() > 50.0, "{}: {} rpm", entry.id, label.value);
            assert_eq!(label.owner_key, motor.key);
        }
    }
}
