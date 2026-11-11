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
