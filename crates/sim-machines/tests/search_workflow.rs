//! The catalogue search as a workflow (`sim_components::search`): the inertia ratio, the drive as a search dimension, a
//! duty mix, the sweep of the gear ratio and pulley radius, the per-motor radial load and the kinematics comparison of
//! one tool-space task, each against a figure worked out by hand.

use sim_components::analysis::{analyse_cycle_with, verdicts};
use sim_components::catalog::{Drives, Options, drive_neighbours, drives_for, inertia_ratios, paired_drive, search};
use sim_components::search::{Task, mix_rms, task_config, transmission_sweep};
use sim_components::{Sim, catalog};
use sim_core::Config;
use sim_machines::find;

/// The single axis without its cable chain: a rigid machine whose feedforward is (m + J / r^2) a r / i, J = J_p + J_belt
/// + i^2 J_m, the whole belt's mass lumped on the pulley (it runs at carriage speed): J_belt = m' (4 s + 2 L) r^2.
fn single_axis() -> Config {
    let mut cfg = find("single-axis").unwrap().config();
    cfg.set_text("effects.cable_chain = false").unwrap();
    cfg
}

/// The single axis's load at the pulley, kg m^2: carriage and payload (4 + 2) kg * (0.02 m)^2 = 0.0024, the pulley 1e-4,
/// the belt 0.1 kg/m * (4 * 0.15 + 2 * 1.0) m * (0.02 m)^2 = 1.04e-4; 0.002604 in all. And the moved mass with the belt,
/// 6 + 0.26 = 6.26 kg.
fn load_at_pulley(cfg: &Config) -> (f64, f64) {
    let get = |path: &str| cfg.f64(path).unwrap();
    let belt = get("belt.mass_kg_m") * (4.0 * get("geometry.span_min_m") + 2.0 * get("geometry.stroke_m"));
    let (m, r) = (get("mass.carriage_kg") + get("mass.payload_kg") + belt, get("geometry.pulley_radius_m"));
    let load = m * r * r + get("pulley.inertia_kgm2");
    assert!((load - 0.002604).abs() < 1e-12 && (m - 6.26).abs() < 1e-12, "{load}, {m}: not the comment's numbers");
    (load, m)
}

/// The load the single axis's motor turns at its shaft, J_L / i^2 = 0.002604 / 5^2 = 1.0416e-4 kg m^2, over the rotor's
/// 0.3e-4 kg m^2: 3.472.
#[test]
fn the_inertia_ratio_of_a_belt_axis_is_the_load_at_the_shaft_over_the_rotor() {
    let cfg = single_axis();
    let (load, _) = load_at_pulley(&cfg);
    let by_hand = load / 25.0 / 0.3e-4;
    assert!((by_hand - 3.472).abs() < 1e-9 && cfg.f64("gearbox.ratio").unwrap() == 5.0);
    assert_eq!(cfg.f64("motor.rotor_inertia_kgm2").unwrap(), 0.3e-4);
    let ratios = inertia_ratios(find("single-axis").unwrap(), &cfg).unwrap();
    assert!(ratios.len() == 1 && (ratios[0] / by_hand - 1.0).abs() < 1e-6, "{ratios:?} against {by_hand}");
}

/// The classic optimum of a pure inertia: J_m i / r + J_L / (i r) is least at i = sqrt(J_L / J_m) = sqrt(0.002604 /
/// 0.3e-4) = 9.3167, where it is 2 sqrt(J_L J_m) / r = 0.027951 N m per m/s^2. And the pulley radius at the ratio 5:
/// (m + (J_p + i^2 J_m) / r^2) r / i is least at r^2 = (J_p + i^2 J_m) / m = (1e-4 + 25 * 0.3e-4) / 6.26 = 1.3578e-4,
/// r = 11.65 mm (the belt's mass on the pulley goes with r^2, as a moved mass). Each sweep runs its values and the
/// optimum on the cycle.
#[test]
fn the_ratio_sweep_finds_the_classic_optimum_of_a_pure_inertia() {
    let (entry, cfg) = (find("single-axis").unwrap(), single_axis());
    let (load, m) = load_at_pulley(&cfg);
    let ratio = transmission_sweep(entry, &cfg, "gearbox.ratio", (2.0, 40.0, 2), None).unwrap();
    assert!((ratio.classic / (load / 0.3e-4).sqrt() - 1.0).abs() < 1e-5, "{}", ratio.classic);
    assert!((ratio.classic - 9.3167).abs() < 1e-3, "{}", ratio.classic);
    assert_eq!(ratio.rows.len(), 3);
    let at = |v: f64| ratio.rows.iter().find(|r| (r.value - v).abs() < 1e-9).unwrap().torque_per_acceleration;
    assert!((at(ratio.classic) - 2.0 * (load * 0.3e-4).sqrt() / 0.02).abs() < 1e-9, "{}", at(ratio.classic));
    assert!(at(2.0) > at(ratio.classic) && at(40.0) > at(ratio.classic));
    let radius = transmission_sweep(entry, &cfg, "geometry.pulley_radius_m", (0.005, 0.05, 2), None).unwrap();
    assert!((radius.classic / (8.5e-4 / m).sqrt() - 1.0).abs() < 1e-5, "{}", radius.classic);
}

/// The S210 at 400 V pairs the 1FK2104-6AF with the 6SL3210-5HE11-0UF0 (3 A rated, `motors_s210.csv`). Among the 3 AC
/// converters of `converters_s210.csv` the next size down is the 5HE10-8UF0 (2.3 A), the next up the 5HE11-5UF0 (5 A).
/// The smallest (5HE10-4UF0, 1.2 A) has none below it; the 1 AC converters are not at 400 V.
#[test]
fn the_next_drive_sizes_are_chosen_by_rated_current() {
    assert_eq!(paired_drive("1FK2104-6AF", "400v"), "6SL3210-5HE11-0UF0");
    assert_eq!(
        drive_neighbours("1FK2104-6AF", "400v"),
        ["6SL3210-5HE10-8UF0", "6SL3210-5HE11-0UF0", "6SL3210-5HE11-5UF0"]
    );
    let all = drives_for("400v");
    assert_eq!(all.len(), 8, "{all:?}");
    assert!(all.windows(2).all(|w| w[0].1 <= w[1].1) && all[0] == ("6SL3210-5HE10-4UF0", 1.2, 4.2), "{all:?}");
    assert_eq!(drives_for("230v").len(), 12, "the 1 AC and 3 AC converters at 230 V");
    // The S120's single Motor Modules: 3 A (5 A next), whose smallest has no smaller one.
    let modules = drives_for("s120");
    let (smallest, next) = (modules[0], modules.iter().find(|d| d.1 > modules[0].1).unwrap());
    assert!(smallest.1 == 3.0 && next.1 == 5.0, "{modules:?}");
}

/// A search with `Drives::Neighbours` tries each motor with up to three drives, and says what the motor and drive leave
/// to the config's estimates: a 1FK2 on an S210 has no winding data or pole pairs in the catalogue, and the S210 no base
/// load current.
#[test]
fn a_search_tries_the_neighbouring_drives_and_names_what_is_estimated() {
    let (entry, cfg) = (find("single-axis").unwrap(), single_axis());
    let options = Options { drives: Drives::Neighbours, ..Options::default() };
    let found = search(entry, &cfg, &["400v"], &|a| a == "1FK2104-6AF", &options).unwrap();
    let mut drives: Vec<&str> = found.iter().map(|c| c.drive).collect();
    drives.sort();
    assert_eq!(drives, drive_neighbours("1FK2104-6AF", "400v"));
    let figures = found[0].figures.as_ref().unwrap();
    assert_eq!(figures.estimated, ["motor-winding", "motor-pole-pairs", "drive-base-load"]);
    assert!(figures.inertia_ratio.is_finite() && figures.checks.iter().any(|(c, _)| *c == "motor-peak"));
}

/// A duty mix of two cycles, the config as it is (weight 3) and without its payload (weight 1): the candidate runs both,
/// and its thermal utilisation is the mix's, sqrt(sum w u^2 t / sum w t) of the two cycles' (`mix_rms`), its cycle time
/// the weighted mean (3 t1 + t2) / 4.
#[test]
fn a_duty_mix_judges_the_heat_of_the_mix() {
    let (entry, cfg) = (find("single-axis").unwrap(), single_axis());
    let mut light = cfg.clone();
    light.set_text("mass.payload_kg = 0.0").unwrap();
    let duties = [(cfg.clone(), 3.0), (light, 1.0)];
    let options = Options { duties: &duties, ..Options::default() };
    let found = search(entry, &cfg, &["400v"], &|a| a == "1FK2104-6AF", &options).unwrap();
    let c = &found[0];
    assert_eq!(c.duties.len(), 2);
    let parts: Vec<(f64, f64, f64)> =
        c.duties.iter().map(|(w, load)| (*w, load.motors[0].thermal_utilisation, load.duration)).collect();
    let f = c.figures.as_ref().unwrap();
    assert!((f.thermal - mix_rms(&parts)).abs() < 1e-12, "{} against {}", f.thermal, mix_rms(&parts));
    assert!(parts[1].1 < parts[0].1, "the cycle without payload heats less");
    let mean = (3.0 * c.duties[0].1.duration + c.duties[1].1.duration) / 4.0;
    assert!((f.cycle_s - mean).abs() < 1e-12);
}

/// A gearbox is offered on a direct drive too: around the classic optimum, beside the drive as it is. The single axis
/// with its gearbox taken out (ratio 1) and the 1FK2104-6AF: J_L = 0.002604 kg m^2 at the pulley, the rotor 0.76e-4 kg m^2
/// (`motors_s210.csv`): the optimum sqrt(J_L / J_m) is 5.85, and the NRK gearboxes of the frame 04 within half to twice
/// it (2.9 to 11.7) are tried.
#[test]
fn a_direct_drive_is_offered_gearboxes_around_its_optimum() {
    let (entry, mut cfg) = (find("single-axis").unwrap(), single_axis());
    cfg.set_text("gearbox.ratio = 1.0").unwrap();
    let options = Options { gearboxes: Some("NRK"), ..Options::default() };
    let found = search(entry, &cfg, &["400v"], &|a| a == "1FK2104-6AF", &options).unwrap();
    let gearboxes: Vec<&str> = found.iter().map(|c| c.gearbox.as_str()).collect();
    assert!(
        gearboxes.contains(&"") && gearboxes.iter().any(|g| g.starts_with("NRK") && g.contains("-04-")),
        "{gearboxes:?}"
    );
}

/// The radial load on a motor's shaft is judged against that motor's own rating: the single axis's belt pulls its pulley
/// shaft with 2 * 300 N (pretension, rigid model), against 1500 N shared but 500 N for motor "m" alone. Before, the check
/// was judged once for the machine against the shared rating, and passed.
#[test]
fn the_radial_load_is_judged_per_motor() {
    let entry = find("single-axis").unwrap();
    let mut cfg = single_axis();
    cfg.set_text("effects.belt_stretch = false").unwrap();
    // A table of motor "m"'s own ratings (`catalog::motor_config`).
    let own = "\n[ratings.m]\nshaft_max_radial_load_n = 500.0\n";
    let cfg = Config::parse(&format!("{}{own}", cfg.to_text().unwrap())).unwrap();
    let load = analyse_cycle_with(&mut Sim::new(entry, &cfg).unwrap(), false).unwrap();
    let radial: Vec<_> =
        verdicts(&cfg, &load).unwrap().into_iter().filter(|v| v.check == "gearbox-radial-load").collect();
    assert_eq!(radial.len(), 1, "{radial:?}");
    assert!(
        radial[0].motor == Some(0)
            && radial[0].limit == 500.0
            && (radial[0].value - 600.0).abs() < 1e-6
            && !radial[0].ok
    );
}

/// A task in tool space mapped onto machines: on the gantry (axes x and z) and the D3 delta (x, y, z) the cycle is pick,
/// up, across, down, up, back, from the middle of the axis ranges; a task that moves along y is refused on the gantry
/// ("task-axis-missing"); a pose beyond an axis's range is refused ("task-out-of-range"), and one inside the box of the
/// axis ranges but outside the delta's working range with the machine's own reason (`Machine::unreachable`).
#[test]
fn the_kinematics_comparison_refuses_an_unreachable_pose() {
    let task = Task::parse(include_str!("../../../examples/pick-place-task.toml")).unwrap();
    let (gantry, delta) = (find("t-gantry").unwrap(), find("delta-d3").unwrap());
    for entry in [gantry, delta] {
        let mapped = task_config(entry, &entry.config(), &task).unwrap();
        assert_eq!(mapped.array("cycle.segment").unwrap().len(), 6, "{}", entry.id);
        Sim::new(entry, &mapped).unwrap();
    }
    let sideways = Task { place: [0.15, 0.1, -0.04], ..task.clone() };
    assert_eq!(task_config(gantry, &gantry.config(), &sideways).unwrap_err().key, "task-axis-missing");
    let far = Task { place: [2.0, 0.0, -0.04], ..task.clone() };
    assert_eq!(task_config(delta, &delta.config(), &far).unwrap_err().key, "task-out-of-range");
    // The corner of the delta's box at mid height: inside every axis range, outside its round working range.
    let machine = (delta.build)(&catalog::resolve(delta, &delta.config()).unwrap()).unwrap();
    let axes = machine.axes();
    let corner =
        [axes[0].max - 0.5 * (axes[0].min + axes[0].max), axes[1].max - 0.5 * (axes[1].min + axes[1].max), 0.0];
    let mid: Vec<f64> = axes.iter().map(|a| 0.5 * (a.min + a.max)).collect();
    let reason =
        machine.unreachable(&[axes[0].max, axes[1].max, mid[2]], 0.0).expect("the box's corner is out of reach");
    let outside = Task { place: corner, lift_m: 0.0, ..task.clone() };
    let refused = task_config(delta, &delta.config(), &outside).unwrap_err();
    assert_eq!(refused.key, reason, "{refused:?}");
}
