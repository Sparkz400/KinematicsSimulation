//! Paths in the verdicts, the search and the optimiser; a program's feed as the speed along the path; friction in the
//! time-optimal plan (14 November 2026).

use sim_components::Sim;
use sim_components::analysis::{analyse_trajectory, verdict_bands, verdicts};
use sim_components::path::Piece;
use sim_components::program;
use sim_components::sim::with_path;
use sim_core::MachineEntry;
use sim_machines::load_file;
use std::path::Path;

fn file(name: &str) -> &'static MachineEntry {
    load_file(&Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines").join(name))
        .unwrap_or_else(|e| panic!("{e}"))
}

/// The highest speed along the path of a plan: the length of the step over all axes from one pose to the next, per
/// time between them.
fn peak_path_speed(points: &[(f64, Vec<f64>)]) -> f64 {
    points
        .windows(2)
        .map(|w| w[1].1.iter().zip(&w[0].1).map(|(b, a)| (b - a).powi(2)).sum::<f64>().sqrt() / (w[1].0 - w[0].0))
        .fold(0.0, f64::max)
}

/// `verdicts --path FILE` judges the path's run in place of the cycle: on the single axis, a path out by 0.3 m and back
/// to 0.1 m, the verdicts of the config with the path (`with_path`, as the command line makes it) are those of the same
/// path planned and measured by hand (`plan_path`, `analyse_trajectory`, `verdicts`), check by check, value for value.
/// The measured run is the plan and 0.3 s of rest: its duration is the plan's last time plus 0.3 s.
#[test]
fn verdicts_of_a_path_judge_the_run_the_analysis_measures() {
    let entry = &machine_single_axis::ENTRY;
    let cfg = entry.config();
    let mut sim = Sim::new(entry, &cfg).unwrap();
    let start = sim.control.planned_pos()[0];
    let text = format!("line,{}\nline,{}\n", start + 0.3, start + 0.1);
    let points = sim.plan_path(&sim_components::path::parse(&text).unwrap(), false).unwrap();
    let end = points[points.len() - 1].0;
    let load = analyse_trajectory(&mut sim, points).unwrap();
    assert!((load.duration - (end + 0.3)).abs() < 1e-9, "{} {end}", load.duration);
    let by_hand = verdicts(&cfg, &load).unwrap();
    let bands = verdict_bands(entry, &with_path(&cfg, "out-and-back.csv", &text, false).unwrap(), &[]).unwrap();
    assert_eq!(bands.len(), by_hand.len());
    for (band, v) in bands.iter().zip(&by_hand) {
        assert_eq!((band.verdict.check, band.verdict.motor, band.verdict.ok), (v.check, v.motor, v.ok));
        assert!(band.verdict.value == v.value || (band.verdict.value.is_nan() && v.value.is_nan()), "{}", v.check);
    }
}

/// G-code F is the speed along the path: G1 X250 Y250 F6000 from the XYZ portal's start (0.1, 0.1) is a diagonal of
/// 150 sqrt(2) = 212.1 mm at 6000 mm/min = 0.1 m/s along it, each axis at 0.1 / sqrt(2) = 0.0707 m/s. Before, F capped
/// each axis at 0.1 m/s: the diagonal ran at 0.1 sqrt(2) = 0.1414 m/s.
#[test]
fn a_g_code_diagonal_at_f_runs_at_f_along_the_path() {
    let entry = file("xyz-portal.toml");
    let sim = Sim::new(entry, &entry.config()).unwrap();
    let runs =
        program::read("diagonal.nc", "G90 G1 X250 Y250 F6000", sim.machine.axes(), sim.control.planned_pos()).unwrap();
    assert!((runs[0].feed.unwrap() - 0.1).abs() < 1e-15);
    let points = sim.plan_program(&runs, false).unwrap();
    let peak = peak_path_speed(&points);
    assert!((peak - 0.1).abs() < 1e-9, "{peak} m/s");
    let x = points.windows(2).map(|w| (w[1].1[0] - w[0].1[0]) / (w[1].0 - w[0].0)).fold(0.0, f64::max);
    assert!((x - 0.1 / 2f64.sqrt()).abs() < 1e-9, "{x} m/s");
}

/// A RAPID MoveL at v100 runs at 100 mm/s along the path: from the XYZ portal's start (100, 100, 250 mm) to (250, 200,
/// 250 mm) the move is 180.3 mm long and cruises at 0.1 m/s. Before, robot moves ran at the axes' limits (0.8 and 0.6 m/s
/// on X and Y). `vmax` keeps the axes' limits: faster.
#[test]
fn a_rapid_move_at_v100_runs_at_100_mm_per_second() {
    let entry = file("xyz-portal.toml");
    let sim = Sim::new(entry, &entry.config()).unwrap();
    let module = |speed: &str| {
        format!(
            "MODULE m\n PROC main()\n  MoveL [[250,200,250],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], {speed}, \
             fine, tool0;\n ENDPROC\nENDMODULE"
        )
    };
    let plan = |speed: &str| {
        let runs = program::read("p.mod", &module(speed), sim.machine.axes(), sim.control.planned_pos()).unwrap();
        sim.plan_program(&runs, false).unwrap()
    };
    let peak = peak_path_speed(&plan("v100"));
    assert!((peak - 0.1).abs() < 1e-9, "{peak} m/s");
    assert!(peak_path_speed(&plan("vmax")) > 0.3);
}

/// The time-optimal plan takes each motor's friction at the speed it turns off its torque. The XYZ portal's Z motor held
/// to 0.2 N m (`drive.mz.peak_torque_nm`) on the path down to (0.1, 0.1, 0.02) and across to (0.3, 0.2, 0.2): its shaft
/// friction is 0.02 tanh(w / 0.5) + 1e-4 w N m (0.06 N m at 400 rad/s), and the Z guide's 10 N Coulomb and 15 N s/m
/// through r / i = 2 mm add 0.02 N m and 3e-5 N m s/rad. The reserve keeps (1 - 0.8) 0.2 = 0.04 N m: less than the
/// friction. Planned with the friction the path is slower than with motor and guide friction switched off, and at the
/// plan's fastest Z speed upwards w (Z's rate times 500 rad/m) the torque that holds Z against gravity (0.0686 N m) and
/// the friction fit under the motor's curve less the reserve (downwards the friction helps the motor hold). The run then
/// keeps the Z motor's demand within its peak.
#[test]
fn a_motor_whose_friction_exceeds_its_reserve_is_planned_slower_within_its_peak() {
    let entry = file("xyz-portal.toml");
    let mut cfg = entry.config();
    cfg.set_text("drive.mz.peak_torque_nm = 0.2").unwrap();
    let pieces = [Piece::Line(vec![0.1, 0.1, 0.02]), Piece::Line(vec![0.3, 0.2, 0.2])];
    let mut sim = Sim::new(entry, &cfg).unwrap();
    let points = sim.plan_path(&pieces, true).unwrap();
    let mut frictionless = cfg.clone();
    for effect in ["motor_friction", "guide_friction"] {
        frictionless.set_text(&format!("effects.{effect} = false")).unwrap();
    }
    let quicker = Sim::new(entry, &frictionless).unwrap().plan_path(&pieces, true).unwrap();
    let (duration, without) = (points[points.len() - 1].0, quicker[quicker.len() - 1].0);
    assert!(duration > without + 1e-3, "{duration} s, without friction {without} s");
    // The fastest the Z motor turns in the plan, and what the hand formulae leave it there.
    let w = 500.0 * points.windows(2).map(|p| (p[1].1[2] - p[0].1[2]) / (p[1].0 - p[0].0)).fold(0.0, f64::max);
    let friction = 0.02 * (w / 0.5).tanh() + 1e-4 * w + 0.02 + 15.0 * 0.002 * 0.002 * w;
    let curve = sim.control.drive_params_of(2).torque_curve(w).1.min(0.2);
    assert!(0.0686 + friction <= curve - 0.04 + 1e-6, "at {w} rad/s: {} > {}", 0.0686 + friction, curve - 0.04);
    let load = analyse_trajectory(&mut sim, points).unwrap();
    let z = load.motors.iter().position(|m| m.id == "mz").unwrap();
    let peak = verdicts(&cfg, &load).unwrap().into_iter().find(|v| v.check == "motor-peak" && v.motor == Some(z));
    assert!(peak.as_ref().is_some_and(|v| v.ok), "{peak:?}");
}

/// `optimise --path FILE` shortens the path's time: on the single axis the path out by 0.3 m and back to 0.1 m, run
/// within the axis limits, as configured takes the plan and 0.3 s of rest (the measured run); the optimiser raises the
/// axis limits as long as every verdict passes, so the result is quicker, and its changes are the limits, not the cycle.
#[test]
fn the_optimiser_shortens_a_paths_time() {
    let entry = &machine_single_axis::ENTRY;
    let cfg = entry.config();
    let sim = Sim::new(entry, &cfg).unwrap();
    let start = sim.control.planned_pos()[0];
    let text = format!("line,{}\nline,{}\n", start + 0.3, start + 0.1);
    let points = sim.plan_path(&sim_components::path::parse(&text).unwrap(), false).unwrap();
    let found =
        sim_components::analysis::optimise_cycle(entry, &with_path(&cfg, "p.csv", &text, false).unwrap(), 6).unwrap();
    assert!((found.before.duration - (points[points.len() - 1].0 + 0.3)).abs() < 1e-9, "{}", found.before.duration);
    assert!(found.after.duration < found.before.duration - 1e-3, "{} {}", found.after.duration, found.before.duration);
    assert!(found.changes.iter().any(|(path, ..)| path.starts_with("axis.")) && found.lifts.is_empty());
}

/// On the hexapod and the spatial cable robot (x, y, z, rx, ry, rz: a line is a slerp), a line from the start to the
/// middle of the travel and one to three quarters of it rise to one top speed each and fall from it as fast as the
/// motors allow, and the two lines one after the other are planned too. Before 14 November the slerp's derivatives by
/// differences gave the hexapod's line to the middle bends of rounding noise that its jerk left no room for: its
/// places' top speeds alternated between 0, 0.0016 and 0.99 m/s, and every line was refused (`error-path-torque`).
#[test]
fn lines_on_six_axes_are_planned_time_optimal() {
    for entry in [&machine_hexapod::ENTRY, &machine_cable_robot::SPATIAL] {
        let sim = Sim::new(entry, &entry.config()).unwrap();
        let point =
            |share: f64| -> Vec<f64> { sim.machine.axes().iter().map(|a| a.min + share * (a.max - a.min)).collect() };
        let line = |p: Vec<f64>| format!("line,{}\n", p.iter().map(f64::to_string).collect::<Vec<_>>().join(","));
        let plan = |text: &str| {
            let points = sim.plan_path(&sim_components::path::parse(text).unwrap(), true);
            points.unwrap_or_else(|e| panic!("{}: {}", entry.id, e.key))
        };
        // The cable robot starts in the middle: its line there is a few micrometres, and only the hexapod's is timed.
        for share in [0.5, 0.75] {
            let points = plan(&line(point(share)));
            let apart = |b: &[f64], a: &[f64]| b.iter().zip(a).map(|(b, a)| (b - a).powi(2)).sum::<f64>().sqrt();
            if apart(&points[points.len() - 1].1, &points[0].1) < 0.01 {
                continue;
            }
            let speeds: Vec<f64> = points.windows(2).map(|w| apart(&w[1].1, &w[0].1) / (w[1].0 - w[0].0)).collect();
            let top = speeds.iter().fold(0.0, |a: f64, b| a.max(*b));
            let peak = speeds.iter().position(|v| *v == top).unwrap();
            assert!(
                top > 0.05
                    && speeds[..=peak].windows(2).all(|w| w[1] >= w[0] - 1e-3 * top)
                    && speeds[peak..].windows(2).all(|w| w[1] <= w[0] + 1e-3 * top),
                "{} to {share}: {speeds:?}",
                entry.id
            );
        }
        let end = plan(&(line(point(0.5)) + &line(point(0.75)))).pop().unwrap().1;
        assert!(end.iter().zip(point(0.75)).all(|(a, b)| (a - b).abs() < 1e-9), "{}: {end:?}", entry.id);
    }
}
