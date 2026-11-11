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

/// One switch per effect, and each one does something: flipping it changes the response to a push.
#[test]
fn every_effect_switch_changes_the_response() {
    use sim_core::testkit::{DT, PulseRun};
    for entry in MACHINES {
        let mut base = entry.config();
        // Low pretension, so the push takes one belt section to zero tension and the slack switch matters.
        base.set_text("belt.pretension_n = 20.0").unwrap();
        let response = |cfg: &sim_core::Config| {
            let machine = (entry.build)(cfg).unwrap();
            let mut run = PulseRun::new(machine.as_ref(), DT, 1.0, 0.05);
            run.run(0.3, |_| {});
            (run.state.clone(), run.hold.clone())
        };
        let reference = response(&base);
        let effects: Vec<String> = base.leaves().into_iter().map(|(p, _)| p).filter(|p| p.starts_with("effects.")).collect();
        assert!(effects.len() >= 10, "{}: only {} effect switches", entry.id, effects.len());
        for effect in effects {
            let mut flipped = base.clone();
            flipped.set_text(&format!("{effect} = {}", !base.bool(&effect).unwrap())).unwrap();
            assert!(response(&flipped) != reference, "{}: flipping {effect} changed nothing", entry.id);
        }
    }
}
