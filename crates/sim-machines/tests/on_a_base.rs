//! Machines on a compliant base (effects.base_compliance, `sim_components::base::Based`) keep what the wrapper
//! forwards from the machine on it: the resonance (the lower of the machine's and the base's), the cam, the gantry's
//! motor pair, the train's twist, the valves and end sensors.

use sim_components::Sim;
use sim_components::analysis::{analyse_cycle, verdicts};
use sim_core::{Config, Machine, MachineEntry};
use sim_machines::{find, load_file};
use std::path::Path;

fn file(name: &str) -> &'static MachineEntry {
    load_file(&Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines").join(name)).unwrap()
}

/// The config with `edits` and the base on or off.
fn on_base(entry: &MachineEntry, on: bool, edits: &[&str]) -> Config {
    let mut cfg = entry.config();
    for edit in edits {
        cfg.set_text(edit).unwrap();
    }
    cfg.set_text(&format!("effects.base_compliance = {on}")).unwrap();
    cfg
}

fn build(entry: &MachineEntry, on: bool, edits: &[&str]) -> Box<dyn Machine> {
    (entry.build)(&on_base(entry, on, edits)).unwrap_or_else(|e| panic!("{}: {e:?}", entry.id))
}

/// The crane's sway, 0.352 Hz, stays its resonance on the base (whose own mode is near 30 Hz, the audit's 29.99 Hz that
/// the wrapper reported before), so the shaper aims at the sway and the shaped cycle (effects.input_shaping) leaves the load as still as it does
/// off the base (5.3 mm in the audit's run off it, 325 mm on it with the base's mode in the shaper).
#[test]
fn the_crane_on_a_base_keeps_its_sway_as_its_resonance() {
    let entry = find("overhead-crane").unwrap();
    let pose = entry.config().f64s("start.pose").unwrap();
    let resonance = |on: bool| build(entry, on, &[]).resonance_hz(&pose).unwrap();
    let (off, on) = (resonance(false), resonance(true));
    assert!((off - 0.352).abs() < 1e-3 && on == off, "{off} Hz off the base, {on} Hz on it");
    let sway = |on: bool| {
        let cfg = on_base(entry, on, &["effects.input_shaping = true", "effects.thermal_warm_start = false"]);
        let load = analyse_cycle(&mut Sim::new(entry, &cfg).unwrap()).unwrap();
        verdicts(&cfg, &load).unwrap().into_iter().find(|v| v.check == "residual-sway").unwrap().value
    };
    let (off, on) = (sway(false), sway(true));
    println!("crane residual sway: {off} m off the base, {on} m on it");
    assert!(on < 1.5 * off.max(1e-3), "residual sway {on} m on the base, {off} m off it");
}

/// The cam machines keep their cam (cam.run on), the dual gantry its motor pair, the screw axis its twist, and the pneumatic axis its
/// valves and end sensors, on the base as off it.
#[test]
fn the_wrapper_forwards_cam_gantry_twist_and_valves() {
    for id in ["toggle-press", "flying-saw", "slider-crank"] {
        let entry = find(id).unwrap();
        let cam = |on: bool| build(entry, on, &["cam.run = true"]).cam().is_some();
        assert!(cam(false) && cam(true), "{id}");
    }
    let dual = file("dual-gantry.toml");
    assert_eq!(
        (build(dual, false, &[]).gantry_motors(), build(dual, true, &[]).gantry_motors()),
        (Some([0, 1]), Some([0, 1]))
    );

    let screw = find("screw-axis").unwrap();
    let twist = |on: bool| {
        let mut out = vec![0.0; 1];
        build(screw, on, &[]).twist(&[1.0], &[0.0], &mut out);
        out[0]
    };
    assert!(twist(false) != 0.0 && twist(true) == twist(false), "{} {}", twist(false), twist(true));

    let pneumatic = file("pneumatic-axis.toml");
    let (off, on) = (build(pneumatic, false, &[]), build(pneumatic, true, &[]));
    assert!(!off.valves().is_empty() && on.valves() == off.valves());
    for valve in 0..off.valves().len() {
        assert_eq!(on.end_sensors(valve), off.end_sensors(valve));
    }
}
