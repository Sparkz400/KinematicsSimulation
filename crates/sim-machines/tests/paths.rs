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

/// The XYZ portal planned for a G-code program in mm (`G91` steps from wherever it stands), with the config's `set`
/// assignments: the plan, and where the portal started.
fn portal_plan(program: &str, set: &[&str]) -> (Vec<(f64, Vec<f64>)>, Vec<f64>) {
    let entry = file("xyz-portal.toml");
    let mut cfg = entry.config();
    for assignment in set {
        cfg.set_text(assignment).unwrap();
    }
    let sim = Sim::new(entry, &cfg).unwrap();
    let start = sim.control.planned_pos().to_vec();
    let runs = program::read("p.nc", program, sim.machine.axes(), &start).unwrap();
    (sim.plan_program(&runs, false).unwrap(), start)
}

/// The plan's point nearest `at`: its distance (m) and the path speed there (the faster of the two steps beside it).
fn pass(points: &[(f64, Vec<f64>)], at: &[f64]) -> (f64, f64) {
    let dist = |p: &[f64]| p.iter().zip(at).map(|(a, b)| (a - b).powi(2)).sum::<f64>().sqrt();
    let i = (0..points.len()).min_by(|&a, &b| dist(&points[a].1).total_cmp(&dist(&points[b].1))).unwrap();
    let step = |a: usize, b: usize| {
        points[a].1.iter().zip(&points[b].1).map(|(x, y)| (x - y).powi(2)).sum::<f64>().sqrt()
            / (points[b].0 - points[a].0)
    };
    // (a step of a point to itself is 0 / 0 = NaN, which `max` passes over)
    let speed = step(i.saturating_sub(1), i).max(step(i, (i + 1).min(points.len() - 1)));
    (dist(&points[i].1), speed)
}

/// The farthest any point of the plan is from the lines start-a, a-b (m).
fn off_lines(points: &[(f64, Vec<f64>)], lines: [&[f64]; 3]) -> f64 {
    let to_segment = |p: &[f64], a: &[f64], b: &[f64]| {
        let ab: Vec<f64> = a.iter().zip(b).map(|(x, y)| y - x).collect();
        let t = (p.iter().zip(a).zip(&ab).map(|((p, a), d)| (p - a) * d).sum::<f64>()
            / ab.iter().map(|d| d * d).sum::<f64>())
        .clamp(0.0, 1.0);
        p.iter().zip(a).zip(&ab).map(|((p, a), d)| (p - a - t * d).powi(2)).sum::<f64>().sqrt()
    };
    points
        .iter()
        .map(|(_, p)| to_segment(p, lines[0], lines[1]).min(to_segment(p, lines[1], lines[2])))
        .fold(0.0, f64::max)
}

/// G61 stops at a corner and G64 P rounds it, whatever the config's `profile.corner_tolerance_m`. From the start s,
/// `G91 F6000` (0.1 m/s) runs 100 mm in X and then 100 mm in Y: a right angle at c = s + (0.1, 0, 0).
/// - Config tolerance 2 mm: G64 (the config's) rounds it (the curve's middle is 3 d / 16 = 2 mm off each line, on the
///   diagonal, sqrt(2) x 2 = 1.41 mm from c: the plan passes more than 1 mm from c), G61 reaches c (within 0.1 um: a
///   tick may miss the stop by 100 (1 ms)^3 / 6 = 1.7e-8 m) at a speed of about 0 (1.7e-5 m/s in the last step).
/// - Config tolerance 0, G64 P0.5: the curve's middle is 0.5 mm off each line, 0.5 sqrt(2) = 0.707 mm from c, and no
///   point is more than 0.5 mm off the two lines; the path passes there at speed.
#[test]
fn g61_stops_and_g64_p_rounds_a_corner() {
    let corner = |start: &[f64]| [start[0] + 0.1, start[1], start[2]];
    let go = |mode: &str, set: &[&str]| portal_plan(&format!("G91 F6000\n{mode}\nG1 X100\nG1 Y100"), set);
    let (rounded, s) = go("G64", &["profile.corner_tolerance_m = 0.002"]);
    let (dist, _) = pass(&rounded, &corner(&s));
    assert!(dist > 1.0e-3, "G64 at the config's 2 mm passes {dist} m from the corner");
    let (stopped, s) = go("G61", &["profile.corner_tolerance_m = 0.002"]);
    let (dist, speed) = pass(&stopped, &corner(&s));
    assert!(dist < 1e-7 && speed < 1e-3, "G61: {dist} m from the corner at {speed} m/s");

    let (blended, s) = go("G64 P0.5", &[]);
    let c = corner(&s);
    let end = [c[0], c[1] + 0.1, c[2]];
    let off = off_lines(&blended, [&s, &c, &end]);
    assert!(off <= 0.5e-3 + 1e-9 && off > 0.4e-3, "{off} m off the lines");
    let (dist, speed) = pass(&blended, &c);
    assert!((0.69e-3..0.80e-3).contains(&dist) && speed > 0.01, "{dist} m from the corner at {speed} m/s");
}

/// The mode of the block after a corner governs it. `G64 P0.5` for X100 and Y100, then G61 for X-100 and Y-100 from s:
/// the corner after X100 (both blocks G64) is rounded, the one after Y100 (the next block is G61) is reached and
/// stopped at, so is the last. The other way round (G61, X100, then G64 P0.5 for Y100) the corner is stopped at too:
/// a change of mode ends the run (documented: RS-274 would round it there).
#[test]
fn the_mode_of_the_block_after_a_corner_governs_it() {
    let (points, s) = portal_plan("G91 F6000\nG64 P0.5\nG1 X100\nG1 Y100\nG61\nG1 X-100\nG1 Y-100", &[]);
    let at = |dx: f64, dy: f64| [s[0] + dx, s[1] + dy, s[2]];
    let (dist, speed) = pass(&points, &at(0.1, 0.0));
    assert!(dist > 0.6e-3 && speed > 0.01, "G64 to G64: {dist} m from the corner at {speed} m/s");
    for (dx, dy) in [(0.1, 0.1), (0.0, 0.1)] {
        let (dist, speed) = pass(&points, &at(dx, dy));
        assert!(dist < 1e-7 && speed < 1e-3, "G64 to G61 at ({dx}, {dy}): {dist} m, {speed} m/s");
    }
    let (points, s) = portal_plan("G91 F6000\nG61\nG1 X100\nG64 P0.5\nG1 Y100", &[]);
    let (dist, speed) = pass(&points, &[s[0] + 0.1, s[1], s[2]]);
    assert!(dist < 1e-7 && speed < 1e-3, "G61 to G64: {dist} m, {speed} m/s");
}

/// G4 P0.5 holds the portal where the move before ended for 0.5 s: 500 profile cycles of 1 ms, the poses equal, and
/// the plan is 0.5 s longer than the same program with `G4 P0` (which splits the run in the same way, so the two plans
/// differ in the dwell only). A longer dwell moves no pose.
#[test]
fn g4_holds_the_pose_for_its_time() {
    let program = |p: &str| format!("G91 F6000\nG1 X100\nG4 P{p}\nG1 Y100");
    let (with, _) = portal_plan(&program("0.5"), &[]);
    let (without, _) = portal_plan(&program("0"), &[]);
    let (a, b) = (with[with.len() - 1].0, without[without.len() - 1].0);
    assert!((a - b - 0.5).abs() < 1e-9, "{a} s against {b} s");
    assert_eq!(with.len(), without.len() + 500);
    // The first two equal poses in a row are the end of the move before and the dwell's first: 500 more follow.
    let i = (0..with.len() - 1).find(|&i| with[i].1 == with[i + 1].1).expect("no pose is held");
    let j = (i..with.len()).take_while(|&j| with[j].1 == with[i].1).last().unwrap();
    assert_eq!(j - i, 500);
    assert!((with[j].0 - with[i].0 - 0.5).abs() < 1e-9, "{} s to {} s", with[i].0, with[j].0);
}

/// `profile.feed_override` scales the feed of G1 moves, never the machine's limits, and G0 not at all. G91 F3000 G1
/// X400 (0.4 m at 0.05 m/s): the cruise at 0.05 m/s lasts (0.4 - 0.05 t) / 0.05 = 7.96 s (t = 2 sqrt(0.05 / 100) = 45 ms,
/// the ramp up and down to a speed at the jerk 100 m/s^3), at override 0.5 and 0.025 m/s (0.4 - 0.025 t) / 0.025 =
/// 15.97 s: twice as long within 1 %. 1.0 is the plan of the config's own 1.0 (the factor is a multiplication by 1). At
/// F60000 (1 m/s, above X's 0.8 m/s) override 2.0 asks 2 m/s and the plan stays at the axis limit. A G0 plan is the
/// same at every override. A RAPID MoveL at v100 (0.1 m/s) runs at 0.05 m/s at 0.5. A value of 0 or below, or above
/// 2, is refused naming the key.
#[test]
fn feed_override_scales_the_feed_within_the_machines_limits() {
    let cruise = |points: &[(f64, Vec<f64>)], speed: f64| {
        let steps = points.windows(2).filter(|w| {
            let d = w[1].1.iter().zip(&w[0].1).map(|(b, a)| (b - a).powi(2)).sum::<f64>().sqrt();
            (d / (w[1].0 - w[0].0) - speed).abs() < 1e-6
        });
        steps.count() as f64 * 1e-3
    };
    let straight = "G91 F3000\nG1 X400";
    let (whole, _) = portal_plan(straight, &[]);
    let (same, _) = portal_plan(straight, &["profile.feed_override = 1.0"]);
    assert_eq!(whole, same);
    let (half, _) = portal_plan(straight, &["profile.feed_override = 0.5"]);
    let (full, slow) = (cruise(&whole, 0.05), cruise(&half, 0.025));
    assert!(
        (full - 7.96).abs() < 0.05 && (slow - 15.97).abs() < 0.05 && (slow / full - 2.0).abs() < 0.02,
        "{full} {slow}"
    );
    let (fast, _) = portal_plan("G91 F60000\nG1 X400", &["profile.feed_override = 2.0"]);
    let peak = peak_path_speed(&fast);
    assert!(peak <= 0.8 + 1e-6 && peak > 0.79, "{peak} m/s");
    let (rapid, _) = portal_plan("G91 G0 X400", &[]);
    assert_eq!(rapid, portal_plan("G91 G0 X400", &["profile.feed_override = 0.5"]).0);
    let entry = file("xyz-portal.toml");
    let mut cfg = entry.config();
    cfg.set_text("profile.feed_override = 0.5").unwrap();
    let sim = Sim::new(entry, &cfg).unwrap();
    let module = "MODULE m\n PROC main()\n  MoveL [[250,200,250],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]], v100, \
                  fine, tool0;\n ENDPROC\nENDMODULE";
    let runs = program::read("p.mod", module, sim.machine.axes(), sim.control.planned_pos()).unwrap();
    let peak = peak_path_speed(&sim.plan_program(&runs, false).unwrap());
    assert!((peak - 0.05).abs() < 1e-9, "{peak} m/s");
    for bad in ["0.0", "-1.0", "2.5"] {
        let mut cfg = entry.config();
        cfg.set_text(&format!("profile.feed_override = {bad}")).unwrap();
        let error = Sim::new(entry, &cfg).err().unwrap_or_else(|| panic!("{bad} was taken"));
        assert!(format!("{error:?}").contains("profile.feed_override"), "{bad}: {error:?}");
    }
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
