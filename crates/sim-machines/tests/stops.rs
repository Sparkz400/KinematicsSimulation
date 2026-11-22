//! Every machine with stops, driven into one: the steps that meet it are cut finer (`mech::STOP_SUBSTEPS`), so the
//! energy across the contact holds to the integrator's level.

use sim_core::MachineEntry;
use sim_core::testkit::{DT, stop_run};
use sim_machines::{MACHINES, load_file};
use std::path::Path;

/// The machines with their stops on whose runs here meet none (22 November 2026): the cable robots, the crank and cam
/// drives but the flying saw and the toggle press, the scissor lift (its joint's stops are built without stiffness),
/// the mover track and the winder.
const NO_STOP_MET: [&str; 10] = [
    "cable-planar",
    "cable-spatial",
    "cam-indexer",
    "disc-cam",
    "geneva-drive",
    "mover-track",
    "rotary-knife",
    "scissor-lift",
    "slider-crank",
    "winder",
];

/// Each registered machine and machine file whose loss-free config has its stops on (`effects.hard_stops`), at rest
/// near the end of one axis after the other (`testkit::stop_run`) and pushed towards it, either way, at 0.3 of its
/// acceleration limit for 1.5 s (then at 1.0 for 4 s, which the crane's long travels need), and 0.25 s past the first
/// step cut for a stop: the first run that meets one without leaving the kinematics keeps the energy less the
/// torques' work to 1e-6 of the push's work (6e-7 at most, the toggle press; without the cuts the toggle press's was
/// 2e-4, the linear delta's 4e-6). All but those of `NO_STOP_MET` meet one.
#[test]
fn a_machine_driven_into_a_stop_keeps_its_energy() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines");
    let mut entries: Vec<&'static MachineEntry> = MACHINES.to_vec();
    let mut files: Vec<_> = std::fs::read_dir(&root).unwrap().map(|e| e.unwrap().path()).collect();
    files.sort();
    entries.extend(files.iter().filter(|f| f.extension().is_some_and(|e| e == "toml")).map(|f| load_file(f).unwrap()));
    let next = std::sync::atomic::AtomicUsize::new(0);
    let rows = std::sync::Mutex::new(Vec::new());
    std::thread::scope(|scope| {
        for _ in 0..4 {
            scope.spawn(|| {
                while let Some(entry) = entries.get(next.fetch_add(1, std::sync::atomic::Ordering::SeqCst)) {
                    let cfg = (entry.suite_configs)().into_iter().next().unwrap_or_else(|| file_without_losses(entry));
                    if !cfg.bool("effects.hard_stops").unwrap_or(false) {
                        continue;
                    }
                    let m = (entry.build)(&cfg).unwrap();
                    let n = m.axes().len();
                    let pushes = [(0.3, 1.5), (1.0, 4.0)].into_iter().flat_map(|(acc, seconds)| {
                        (0..n).flat_map(move |axis| [(axis, acc, seconds), (axis, -acc, seconds)])
                    });
                    let met = pushes
                        .map(|(axis, acc, seconds)| (axis, acc, stop_run(m.as_ref(), axis, acc, DT, seconds, 0.25)))
                        .find(|(.., r)| r.cut > 0 && !r.broke);
                    rows.lock().unwrap().push((entry.id, met));
                }
            });
        }
    });
    let mut missed = Vec::new();
    for (id, met) in rows.into_inner().unwrap() {
        let Some((axis, acc, r)) = met else {
            missed.push(id);
            continue;
        };
        println!(
            "{id}: axis {axis} at {acc}: {} of {} steps cut, drift {:e} J of {:e} J",
            r.cut, r.steps, r.drift, r.push_work
        );
        assert!(r.drift < 1e-6 * r.push_work, "{id}: {} J of {} J", r.drift, r.push_work);
    }
    missed.sort();
    assert_eq!(missed, NO_STOP_MET, "the machines whose runs meet no stop");
}

/// A machine file, which has no loss-free configs of its own: every effect off but its stops, and every damper off
/// (the stops' among them).
fn file_without_losses(entry: &MachineEntry) -> sim_core::Config {
    let mut cfg = entry.config();
    for (key, value) in cfg.leaves() {
        if key.starts_with("effects.") && value.as_bool().is_some() {
            cfg.set_text(&format!("{key} = {}", key == "effects.hard_stops")).unwrap();
        } else if key.rsplit('.').next().is_some_and(|leaf| leaf.contains("damping")) && value.as_float().is_some() {
            cfg.set_text(&format!("{key} = 0.0")).unwrap();
        }
    }
    cfg
}
