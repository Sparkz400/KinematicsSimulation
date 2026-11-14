//! Per-motor limits (14 November 2026): the time-optimal plan holds each motor to its own drive and motor, up to its
//! maximum speed along its torque-speed curve; the analysis gives each motor's peak current by its own motor; the
//! `[bounds]` follow a part swapped in by name.

use sim_components::Sim;
use sim_components::analysis::{analyse_cycle, bounds, verdict_bands};
use sim_components::path::Piece;
use sim_core::{Config, MachineEntry};
use sim_machines::load_file;
use std::path::{Path, PathBuf};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn file(name: &str) -> &'static MachineEntry {
    load_file(&root().join("machines").join(name)).unwrap_or_else(|e| panic!("{e}"))
}

/// The highest speed of an axis in a plan, from the poses one profile cycle apart.
fn peak_speed(points: &[(f64, Vec<f64>)], axis: usize) -> f64 {
    points.windows(2).map(|w| ((w[1].1[axis] - w[0].1[axis]) / (w[1].0 - w[0].0)).abs()).fold(0.0, f64::max)
}

/// The time-optimal plan of a straight move of `axis` to whichever end of its travel is farther.
fn plan_across(sim: &Sim, axis: usize) -> Vec<(f64, Vec<f64>)> {
    let info = &sim.machine.axes()[axis];
    let mut target = sim.control.set_pos.clone();
    let here = target[axis];
    target[axis] = if info.max - here > here - info.min { info.max } else { info.min };
    sim.plan_path(&[Piece::Line(target)], true).unwrap()
}

/// The single axis turns its motor 250 rad per metre (gearbox 5 over a 20 mm pulley radius). With its maximum speed at
/// 200 rad/s, below the rated 314 rad/s where the curve is full, a move across the 1 m stroke cruises at 200 / 250 =
/// 0.8 m/s. Before, the plan capped every motor at its rated speed and read no maximum: it cruised at 314 / 250 =
/// 1.256 m/s.
#[test]
fn the_time_optimal_plan_keeps_a_motor_within_its_maximum_speed() {
    let entry = &machine_single_axis::ENTRY;
    let mut cfg = entry.config();
    cfg.set_text("drive.max_speed_rad_s = 200.0").unwrap();
    let sim = Sim::new(entry, &cfg).unwrap();
    let peak = peak_speed(&plan_across(&sim, 0), 0);
    assert!(peak <= 0.8 * (1.0 + 1e-6) && peak > 0.8 * (1.0 - 1e-3), "{peak} m/s");
}

/// Above its rated speed a motor gives less than its full torque, and the plan may use it there: the single axis's curve
/// falls from 3.8 N m at 314 rad/s to 0 at 420 rad/s (its maximum speed), and the plan keeps back (1 - 0.8) * 3.8 =
/// 0.76 N m at every speed for friction. That leaves room up to 420 - 106 * 0.76 / 3.8 = 398.8 rad/s, 1.5952 m/s. Before,
/// it stopped at the rated speed, 1.256 m/s.
#[test]
fn the_time_optimal_plan_turns_a_motor_above_its_rated_speed_along_its_curve() {
    let entry = &machine_single_axis::ENTRY;
    let sim = Sim::new(entry, &entry.config()).unwrap();
    let peak = peak_speed(&plan_across(&sim, 0), 0);
    assert!(peak > 314.0 / 250.0 * 1.01 && peak <= 398.8 / 250.0 * (1.0 + 1e-6), "{peak} m/s");
}

/// The XYZ portal's Z motor has its own drive table: given a maximum speed of 100 rad/s there, a Z move cruises at
/// 100 rad/s through r / i = 2 mm (0.2 m/s), while X and Y keep the shared drive. Before, the plan read the shared
/// `[drive]` for every motor and cruised Z at the rated 314 rad/s, 0.628 m/s.
#[test]
fn the_time_optimal_plan_holds_each_motor_to_its_own_drive() {
    let text = std::fs::read_to_string(root().join("machines/xyz-portal.toml")).unwrap();
    assert!(text.contains("\n[drive.mz]\n"));
    let dir = std::env::temp_dir().join(format!("limits-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("xyz-portal-z-slow.toml");
    std::fs::write(&path, text.replace("\n[drive.mz]\n", "\n[drive.mz]\nmax_speed_rad_s = 100.0\n")).unwrap();
    let entry = load_file(&path).unwrap_or_else(|e| panic!("{e}"));
    let sim = Sim::new(entry, &entry.config()).unwrap();
    assert_eq!((sim.control.drive_params_of(2).max_speed, sim.control.drive_params_of(0).max_speed), (100.0, 420.0));
    let z = sim.machine.axes().iter().position(|a| a.id == "z").unwrap();
    let peak = peak_speed(&plan_across(&sim, z), z);
    assert!(peak <= 0.2 * (1.0 + 1e-6) && peak > 0.2 * (1.0 - 1e-3), "{peak} m/s");
    // X keeps the shared maximum: faster than the Z motor's 100 rad/s, 0.2 m/s through its 2 mm, would allow.
    let x = sim.machine.axes().iter().position(|a| a.id == "x").unwrap();
    assert!(peak_speed(&plan_across(&sim, x), x) > 0.5);
    std::fs::remove_dir_all(&dir).ok();
}

/// A motor's peak current is its own motor's: on the XYZ portal with the Z motor held to 0.15 N m, its velocity loop asks
/// for far more than 0.889 * 0.45 N m/A * 4.75 A = 1.90 N m, where its current curve tops out (torque constant 0.45, 11 %
/// lost to saturation at its own maximum current of 4.75 A), so the current it asks for is that maximum, 4.75 A. Before,
/// the shared motor's curve (maximum current 9.5 A) gave 9.5 A.
#[test]
fn the_peak_current_is_each_motors_own() {
    let entry = file("xyz-portal.toml");
    let mut cfg = entry.config();
    cfg.set_text("drive.mz.peak_torque_nm = 0.15").unwrap();
    let load = analyse_cycle(&mut Sim::new(entry, &cfg).unwrap()).unwrap();
    let z = load.motors.iter().find(|m| m.id == "mz").unwrap();
    assert!((z.peak_current - 4.75).abs() < 1e-9, "{}", z.peak_current);
}

/// EXCT-30's braking threshold has bounds of 750 to 790 V, written for its 540 V bus and 770 V threshold. A 230 V motor
/// and drive by name put a 325 V bus in and move the threshold with it (770 * 325 / 540 = 463.4 V, then the S210's own
/// rule where that is at or above its trip level); the bounds follow by the same factor, 750 and 790 V times the new
/// threshold over 770 V. Before, they stayed at 750 and 790 V and `verdicts --bands` aborted: out of range.
#[test]
fn the_bounds_follow_a_part_swapped_in_by_name() {
    let entry = &machine_t_gantry::ENTRY;
    let path = root().join("crates/machine-t-gantry/exct-30.toml");
    let mut cfg = Config::parse(&std::fs::read_to_string(path).unwrap()).unwrap();
    assert_eq!(cfg.f64("ratings.braking_threshold_v").unwrap(), 770.0);
    for (key, value) in
        [("motor", "\"1FK2104-5AF\""), ("motor_supply", "\"230v\""), ("drive", "\"6SL3210-5HB10-4UF0\"")]
    {
        cfg.set_text(&format!("parts.{key} = {value}")).unwrap();
    }
    let resolved = sim_components::catalog::resolve(entry, &cfg).unwrap();
    let threshold = resolved.f64("ratings.braking_threshold_v").unwrap();
    assert!(threshold < 470.0, "{threshold}");
    let scale = threshold / 770.0;
    let followed = bounds(&resolved).unwrap();
    let (_, low, high) = followed.iter().find(|(p, ..)| p == "ratings.braking_threshold_v").unwrap();
    assert!((low - 750.0 * scale).abs() < 1e-9 && (high - 790.0 * scale).abs() < 1e-9, "{low} {high}");
    // The bands run with the bounds as the resolved config has them, whatever list they were given.
    let one = [("ratings.braking_threshold_v".to_string(), 750.0, 790.0)];
    assert!(verdict_bands(entry, &cfg, &one).is_ok());
}
