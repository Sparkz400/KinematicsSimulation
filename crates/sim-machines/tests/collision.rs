//! Obstacles and crash detection against hand calculations: the clearance of a box beside a straight move, a move into
//! a box with effects.crash (blocked by the box's push, tripped by the drive's monitoring, stopped by its fault
//! reaction), the check "collision" on a cycle, and the refused time-optimal path.

use sim_components::analysis::{analyse_cycle, verdicts};
use sim_components::collision::check_cycle;
use sim_components::fault::Monitor;
use sim_components::path::Piece;
use sim_components::{Command, Sim};
use sim_core::Config;
use sim_machines::{MACHINES, find, load_file};

fn with(entry: &sim_core::MachineEntry, settings: &[&str]) -> Config {
    let mut cfg = entry.config();
    for s in settings {
        cfg.set_text(s).unwrap();
    }
    cfg
}

/// The single axis draws its carriage, 0.16 x 0.05 x 0.03 m, at (0.15 + x, -0.03, 0.06) and its payload, a 0.1 m
/// cube, at (0.15 + x, -0.03, 0.125): y from -0.055 to -0.005 and -0.08 to 0.02, z from 0.045 to 0.075 and 0.075 to
/// 0.175. A 0.1 m cube at (0.5, 0.2, 0.06) spans y 0.15 to 0.25 and z 0.01 to 0.11. Moving from x = 0.1 to 0.6 the
/// carriage passes it in x, and then the payload is nearest, straight across in y: 0.15 - 0.02 = 0.13 m (the
/// carriage 0.155 m). Planned and run, the clearance is 0.13 m, since the axis moves in x alone.
#[test]
fn a_box_beside_a_straight_move_keeps_the_clearance_by_hand() {
    let entry = find("single-axis").unwrap();
    let cfg =
        with(entry, &[r#"obstacles = [{ name = "beside", centre_m = [0.5, 0.2, 0.06], size_m = [0.1, 0.1, 0.1] }]"#]);
    let mut sim = Sim::new(entry, &cfg).unwrap();
    let planned = sim.check_plan(&[vec![0.1], vec![0.6]]).unwrap();
    assert!((planned.clearance - 0.13).abs() < 1e-9 && planned.contact.is_none(), "{planned:?}");
    sim.command(Command::MoveTo(vec![0.6])).unwrap();
    sim.run(1.5, |_| {});
    assert!((sim.least_clearance - 0.13).abs() < 1e-9 && sim.contact.is_none(), "{}", sim.least_clearance);
    // At x = 0.6 the carriage's near face is at 0.67, the box's at 0.55: 0.12 m in x and 0.155 m in y, 0.19601 m, a
    // little nearer than the payload's 0.15 and 0.13 m (0.19849 m). The tool settles within `setpoint_slack` of its
    // target, so to 0.1 mm.
    assert!((sim.clearance - (0.12f64.powi(2) + 0.155f64.powi(2)).sqrt()).abs() < 1e-4, "{}", sim.clearance);
    let mut scene = sim_core::Scene::default();
    sim.scene(&mut scene);
    assert_eq!(scene.obstacles.len(), 1);
    assert!(!scene.obstacles[0].contact && scene.first_contact.is_none() && scene.clearance.is_some());
}

/// A box across the way: its face at x = 0.75 - 0.05 = 0.70, the carriage's front at 0.15 + x + 0.08, so they meet at
/// x = 0.47. The move from 0.1 to 0.9 at 1 m/s, 10 m/s² and 100 m/s³ is at its top speed from 0.1 m after the start
/// (a jerk-limited ramp of 0.2 s covers v T / 2 = 0.1 m) until 0.1 m before the end: at 0.37 m from the start it is
/// cruising at 1 m/s. With effects.crash the box pushes back and blocks the axis. With effects.drive_monitoring the
/// setpoint runs on at 1 m/s * i / r = 1 * 5 / 0.02 = 250 rad/s at the motor; the box stops the carriage
/// (`Machine::set_tool_force`) and the motor turns on into the belt's and the gearbox's give, so its following error
/// crosses the 1 rad window (1 + give) / 250 s after the contact; the position loop sees it at its next cycle (1 ms),
/// and the drive trips on its following error within 2 ms of that. It stops (OFF3) and the carriage comes to rest at
/// the face, at most 20 mm back where the box pushed it.
/// Without the crash the carriage runs through.
#[test]
fn a_move_into_a_box_is_blocked_at_its_face_and_trips_on_the_following_error() {
    let entry = find("single-axis").unwrap();
    let wall = r#"obstacles = [{ name = "wall", centre_m = [0.75, -0.03, 0.06], size_m = [0.1, 0.2, 0.2] }]"#;
    let planned = Sim::new(entry, &with(entry, &[wall])).unwrap().check_plan(&[vec![0.1], vec![0.9]]).unwrap();
    let contact_pose = planned.contact.as_ref().unwrap()[0];
    assert!(planned.clearance < 0.0 && (contact_pose - 0.47).abs() <= 1e-3, "{planned:?}");

    let monitored = [wall, "effects.crash = true", "effects.drive_monitoring = true"];
    let mut sim = Sim::new(entry, &with(entry, &monitored)).unwrap();
    sim.command(Command::MoveTo(vec![0.9])).unwrap();
    let mut planned_speed = None;
    while sim.contact.is_none() && sim.time() < 2.0 {
        let set_vel = sim.control.set_vel[0];
        sim.step();
        if sim.contact.is_some() {
            planned_speed = Some(set_vel);
        }
    }
    let contact = sim.contact.clone().expect("the carriage reaches the wall");
    assert_eq!(contact.obstacle, 0);
    assert!((planned_speed.unwrap() - 1.0).abs() < 1e-9, "planned {planned_speed:?}");
    assert!((contact.speed - 1.0).abs() < 0.03, "impact at {} m/s", contact.speed);
    // From rest at 0.1: 0.1 m in 0.2 s, then 0.27 m at 1 m/s, and the tool lags by the following error.
    assert!((contact.time - 0.47).abs() < 0.03, "contact at {} s", contact.time);
    // The motor's turn from the contact to the trip: into the belt's and the gearbox's give.
    let (mut angle, mut rate) = ([0.0], [0.0]);
    sim.machine.motor_state(&sim.state, &mut angle, &mut rate);
    let at_contact = angle[0];
    while sim.control.fault.is_none() && sim.time() < contact.time + 0.5 {
        sim.step();
        sim.machine.motor_state(&sim.state, &mut angle, &mut rate);
    }
    let give = angle[0] - at_contact;
    sim.run(0.5, |_| {});
    let fault = sim.control.fault.clone().expect("the drive trips");
    let after = fault.time - (contact.time + (1.0 + give) / 250.0);
    println!("the motor gave {give} rad after the contact");
    assert!(fault.monitor == Monitor::FollowingError && fault.drive == 0, "{fault:?}");
    assert!((0.0..0.002).contains(&after), "tripped {after} s after the window crossing");
    assert!(sim.crashed && !sim.control.enabled() && fault.stopped.is_some(), "{fault:?}");
    let (_, front) = carriage_front(&sim);
    let (mut pos, mut vel) = ([0.0], [0.0]);
    sim.machine.tool_state(&sim.state, &mut pos, &mut vel);
    assert!(front < 0.703 && front > 0.68 && vel[0].abs() < 1e-3, "front {front}, speed {}", vel[0]);
    // The impact is harder than the push at the current limit (950 N, below).
    let impact = sim.contact.clone().unwrap();
    println!("{impact:?}");
    assert!(impact.peak_force > 950.0 && impact.impact_energy > 0.0, "{impact:?}");

    let mut through = Sim::new(entry, &with(entry, &[wall])).unwrap();
    through.command(Command::MoveTo(vec![0.9])).unwrap();
    through.run(2.0, |_| {});
    let mut scene = sim_core::Scene::default();
    through.scene(&mut scene);
    assert!(through.control.enabled() && (through.control.set_pos[0] - 0.9).abs() < 1e-9 && through.contact.is_some());
    // Through and past it: at x = 0.9 the carriage's back is at 0.97, beyond the wall's far face at 0.80.
    assert!(!scene.obstacles[0].contact && scene.first_contact == through.contact.as_ref().map(|c| c.time));

    // The energy budget of the same move with the crash (the drive unmonitored, pushing on) closes, to
    // `BUDGET_TOLERANCE` as on the shipped cycles: what the box's push took out of the machine is its own row, `crash`,
    // integrated with the motion; the electrical side closes to `ELECTRICAL_TOLERANCE`.
    let mut sim = Sim::new(entry, &with(entry, &[wall, "effects.crash = true"])).unwrap();
    let points = sim.plan_path(&[Piece::Line(vec![0.9])], false).unwrap();
    let load = sim_components::analysis::analyse_trajectory(&mut sim, points).unwrap();
    let budget = load.budget.unwrap();
    let crash = budget.rows.iter().find(|r| r.id == "crash").map(|r| r.energy);
    println!("crash {crash:?} J, unaccounted {} J of {} J", budget.unaccounted(), budget.supply);
    assert!(load.contact.is_some() && crash.is_some_and(|e| e > 0.0), "{crash:?}");
    assert!(
        budget.unaccounted().abs() < sim_components::analysis::BUDGET_TOLERANCE * budget.supply,
        "{} J of {} J not accounted for",
        budget.unaccounted(),
        budget.supply
    );
    assert!(
        budget.electrical_unaccounted().abs() < sim_components::analysis::ELECTRICAL_TOLERANCE * budget.supply,
        "electrical: {} J",
        budget.electrical_unaccounted()
    );
}

/// The same move into the box without drive monitoring: nothing trips, and the blocked drive pushes on at its torque
/// limit, the peak torque 3.8 N m (at the motor's maximum current 9.5 A, within the drive's 10.9 A). At rest the box
/// holds it with F = T_max i / r = 3.8 N m * 5 / 0.02 m = 950 N, less what the motor's own Coulomb friction (at most
/// 0.02 N m, 5 N at the box) takes. Without the torque ripple: it scales the drive's torque by up to 2 % at the angle
/// the motor stands at, which with the push at the carriage is another than with the push at the motor (958.0 N with
/// it; with the guide's, the teeth's, the motor's friction and the gearbox's efficiency off as well 958.0 N still).
#[test]
fn a_blocked_axis_pushes_with_the_hand_calculated_force_at_the_current_limit() {
    let entry = find("single-axis").unwrap();
    let wall = r#"obstacles = [{ name = "wall", centre_m = [0.75, -0.03, 0.06], size_m = [0.1, 0.2, 0.2] }]"#;
    let settings = [wall, "effects.crash = true", "effects.torque_ripple = false"];
    let mut sim = Sim::new(entry, &with(entry, &settings)).unwrap();
    sim.command(Command::MoveTo(vec![0.9])).unwrap();
    sim.run(1.5, |_| {});
    let torque = sim.control.drives[0].torque;
    println!("torque {torque} N m, contact force {} N", sim.contact_force);
    assert!(sim.control.enabled() && sim.control.fault.is_none() && !sim.crashed);
    assert!((torque - 3.8).abs() < 1e-3, "{torque} N m");
    assert!((sim.contact_force - 950.0).abs() < 5.0, "{} N", sim.contact_force);
}

/// A machine file's belt axis takes the obstacle's push at its tool (`Machine::set_tool_force`), so the blocked carriage
/// stands in the obstacle by what its contact spring gives, F / k with k = 1e7 N/m, and not by that plus the belt's
/// stretch F / k_belt as when the push holds the motor (k_belt about EA / L = 5e5 N / (0.5 to 1 m): with F near 1 kN
/// that is millimetres against the contact's 0.1 mm). The wall is wide and tall enough to meet any part of the carriage.
#[test]
fn a_push_at_the_tool_stops_a_load_on_a_belt_at_the_obstacles_face() {
    let entry = load_file(&std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines/single-axis.toml"))
        .unwrap_or_else(|e| panic!("{e}"));
    let wall = r#"obstacles = [{ name = "wall", centre_m = [1.1, 0.0, 0.0], size_m = [1.0, 10.0, 10.0] }]"#;
    let mut sim = Sim::new(entry, &with(entry, &[wall, "effects.crash = true"])).unwrap();
    sim.command(Command::MoveTo(vec![0.9])).unwrap();
    sim.run(2.5, |_| {});
    let (force, depth) = (sim.contact_force, -sim.clearance);
    println!("contact force {force} N, depth {depth} m");
    assert!(sim.contact.is_some() && force > 100.0, "{force} N");
    assert!((depth - force / 1e7).abs() < 0.05 * depth, "depth {depth} m against F / k = {} m", force / 1e7);

    // The budget closes with the push at the tool: what it took is the crash row, within `BUDGET_TOLERANCE`.
    let mut sim = Sim::new(entry, &with(entry, &[wall, "effects.crash = true"])).unwrap();
    let points = sim.plan_path(&[Piece::Line(vec![0.9])], false).unwrap();
    let load = sim_components::analysis::analyse_trajectory(&mut sim, points).unwrap();
    let budget = load.budget.unwrap();
    let crash = budget.rows.iter().find(|r| r.id == "crash").unwrap().energy;
    println!("crash {crash} J, {} J of {} J not accounted for", budget.unaccounted(), budget.supply);
    assert!(load.contact.is_some() && crash > 0.0, "crash {crash} J");
    assert!(
        budget.unaccounted().abs() < sim_components::analysis::BUDGET_TOLERANCE * budget.supply,
        "{} J of {} J not accounted for",
        budget.unaccounted(),
        budget.supply
    );
}

/// On the five-axis file the tool's coordinates are its tip's, which the tilted head moves with every joint: the push
/// at the tip goes to the joints as its virtual work on each (`ConfigMachine::on_coordinates`), and the budget closes
/// with it, within `BUDGET_TOLERANCE`, while the head tilts its tip down onto a slab.
#[test]
fn a_push_at_a_tilted_tool_tip_keeps_the_budget() {
    let entry = load_file(&std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines/five-axis.toml"))
        .unwrap_or_else(|e| panic!("{e}"));
    let slab = r#"obstacles = [{ name = "slab", centre_m = [0.5, 0.25, -0.52], size_m = [2.0, 2.0, 1.0] }]"#;
    let mut sim = Sim::new(entry, &with(entry, &[slab, "effects.crash = true"])).unwrap();
    let points = sim.plan_path(&[Piece::Line(vec![0.5, 0.25, -0.1, 0.6, 0.5])], false).unwrap();
    let load = sim_components::analysis::analyse_trajectory(&mut sim, points).unwrap();
    let budget = load.budget.unwrap();
    let crash = budget.rows.iter().find(|r| r.id == "crash").unwrap().energy;
    println!(
        "contact {:?}, crash {crash} J, {} J of {} J not accounted for",
        load.contact,
        budget.unaccounted(),
        budget.supply
    );
    assert!(load.contact.is_some() && crash > 0.0, "crash {crash} J");
    assert!(
        budget.unaccounted().abs() < sim_components::analysis::BUDGET_TOLERANCE * budget.supply,
        "{} J of {} J not accounted for",
        budget.unaccounted(),
        budget.supply
    );
}

/// The xyz portal lowers onto a slab while it traverses: from (0.1, 0.1, 0.25) to (0.9, 0.1, 0.0), X cruising at its
/// 0.8 m/s, Z at 0.25 m/s (the move takes as long on both). The parts' lowest point is 0.06 m below the tool, so the
/// slab's top at z = 0.16 meets it at z = 0.10. Z is blocked there and trips on its following error: the setpoint runs
/// on at 0.25 m/s * 10 / 0.02 m = 125 rad/s at the motor, the slab stops the slide and the motor turns on into the
/// belt's and the gearbox's give (0.22 rad), so the error reaches 1 rad after (1 + 0.22) / 125 = 9.8 ms (one position
/// cycle, 1 ms, more at most). X is not touched and runs on until
/// its fault reaction stops it. OFF3: the rated speed 314 rad/s in 0.1 s at the motor is 3140 rad/s², times
/// r / i = 0.02 / 5 m at the carriage 12.56 m/s²; from v = 0.8 m/s that takes v² / (2a) = 25.48 mm, give or take the
/// speed loop's lag behind the ramp (about a millisecond, under 1.6 mm at 0.8 m/s). OFF1 (1 s from the rated speed,
/// 1.256 m/s²) takes ten times as far, 254.8 mm. STO, without a holding brake, leaves the motor to coast on its
/// friction, further than OFF3 by far.
#[test]
fn the_axis_not_in_contact_stops_on_its_fault_reaction_ramp() {
    let entry = load_file(&std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines/xyz-portal.toml"))
        .unwrap_or_else(|e| panic!("{e}"));
    let slab = r#"obstacles = [{ name = "slab", centre_m = [0.0, 0.0, -0.34], size_m = [10.0, 10.0, 1.0] }]"#;
    let run = |reaction: &str| {
        let reaction = format!("drive.fault_reaction = {reaction:?}");
        let settings = [
            slab,
            "effects.crash = true",
            "effects.drive_monitoring = true",
            "effects.holding_brake = false",
            &reaction,
        ];
        let mut sim = Sim::new(entry, &with(entry, &settings)).unwrap();
        sim.command(Command::MoveTo(vec![0.9, 0.1, 0.0])).unwrap();
        // The tool's speed along x at the trip: the drives trip at the start of a step, on what the step before left.
        let (mut pos, mut vel, mut speed) = ([0.0; 3], [0.0; 3], 0.0);
        let motors = sim.machine.motors().len();
        let (mut angle, mut rate, mut at_contact) = (vec![0.0; motors], vec![0.0; motors], None);
        while sim.control.fault.is_none() && sim.time() < 2.0 {
            sim.machine.tool_state(&sim.state, &mut pos, &mut vel);
            speed = vel[0];
            sim.step();
            sim.machine.motor_state(&sim.state, &mut angle, &mut rate);
            if sim.contact.is_some() && at_contact.is_none() {
                at_contact = Some(angle[2]);
            }
        }
        let give = (angle[2] - at_contact.unwrap()).abs();
        sim.run(1.5, |_| {});
        (sim, speed, give)
    };
    let (sim, v, give) = run("off3");
    let (contact, fault) = (sim.contact.clone().unwrap(), sim.control.fault.clone().unwrap());
    println!("contact {contact:?}\n{fault:?}, {v} m/s at the trip, Z motor {give} rad on after the contact");
    let after = fault.time - contact.time;
    assert!(fault.monitor == Monitor::FollowingError && fault.drive == 2, "{fault:?}");
    let due = (1.0 + give) / (0.25 * 10.0 / 0.02);
    assert!(
        give < 0.5 && (due - 1e-9..due + 1e-3).contains(&after),
        "tripped {after} s after the contact, not {due} s"
    );
    assert!((v - 0.8).abs() < 0.01 && sim.crashed && !sim.control.enabled() && fault.stopped.is_some(), "{v} m/s");
    let off3 = v * v / (2.0 * 3140.0 * 0.02 / 5.0);
    // The ramp starts from the speed the drive measured, and the speed loop lags behind it: within 2 ms at v either way.
    assert!((fault.stopping[0] - off3).abs() < 0.002 * v, "{} m against {off3} m", fault.stopping[0]);
    let (sim, v, _) = run("off1");
    let (stopping, off1) = (sim.control.fault.unwrap().stopping[0], v * v / (2.0 * 314.0 * 0.02 / 5.0));
    assert!((stopping - off1).abs() < 0.002 * v, "OFF1: {stopping} m against {off1} m");
    let (sim, _, _) = run("sto");
    let coasted = sim.control.fault.unwrap().stopping[0];
    assert!(coasted > 3.0 * off3, "STO: {coasted} m");
}

/// The T-gantry draws its payload, a 0.1 m cube, centred 0.752 m below the tool's z in the 3D view's z and 0.692 m
/// below for the tool itself: on the traverse at z = 0.35 its bottom is at 0.35 - 0.802 = -0.452. A box 0.1 m high
/// with its top at -0.5, at x = 0.65 (the middle of the traverse, 0.15 + 0.5), is 0.048 m below it. The return
/// traverse goes without the payload: the flange, 0.02 m high at -0.342, is 0.148 m above the box. The cycle's lowers
/// are at x = 0.25 and 1.05 in the view, 0.3 m and more beside the box. So 0.048 m, against `profile.min_clearance_m`
/// 0.3 (fails) and 0.04 (passes); without obstacles the check is not made.
#[test]
fn a_box_under_the_traverse_is_judged_by_the_collision_check() {
    let entry = find("t-gantry").unwrap();
    let low = r#"obstacles = [{ name = "fixture", centre_m = [0.65, -0.05, -0.55], size_m = [0.1, 0.2, 0.1] }]"#;
    let cfg = with(entry, &[low]);
    let planned = check_cycle(entry, &cfg).unwrap().unwrap();
    assert!((planned.clearance - 0.048).abs() < 1e-9 && planned.contact.is_none(), "{planned:?}");
    let load = analyse_cycle(&mut Sim::new(entry, &cfg).unwrap()).unwrap();
    let clearance = load.clearance.unwrap();
    assert!((clearance - 0.048).abs() < 1e-3 && load.contact.is_none(), "{clearance}");
    let judged = verdicts(&cfg, &load).unwrap();
    let collision = judged.iter().find(|v| v.check == "collision").unwrap();
    assert!(!collision.ok && collision.limit == 0.3 && collision.value == clearance);
    let relaxed = with(entry, &[low, "profile.min_clearance_m = 0.04"]);
    assert!(verdicts(&relaxed, &load).unwrap().iter().any(|v| v.check == "collision" && v.ok));
    // A box in the way of the traverse is a contact before the cycle runs.
    let hit =
        with(entry, &[r#"obstacles = [{ name = "post", centre_m = [0.65, -0.05, -0.45], size_m = [0.1, 0.2, 0.1] }]"#]);
    let planned = check_cycle(entry, &hit).unwrap().unwrap();
    assert!(planned.clearance < 0.0 && planned.contact.as_ref().is_some_and(|p| p[1] == 0.35), "{planned:?}");
    // No obstacles: nothing is checked, and no verdict can fail on it.
    for entry in MACHINES {
        let cfg = entry.config();
        assert_eq!(check_cycle(entry, &cfg).unwrap(), None);
        let load = analyse_cycle(&mut Sim::new(entry, &cfg).unwrap()).unwrap();
        assert!(
            load.clearance.is_none() && !verdicts(&cfg, &load).unwrap().iter().any(|v| v.check == "collision"),
            "{}",
            entry.id
        );
    }
}

/// The optimiser lifts each traverse of the T-gantry's cycle over a box across it, and no higher. With
/// `profile.min_clearance_m` = 0.05 m and pick and place at z = 0.05, the lifts go down to 0.05 + 0.05 = 0.1 m without
/// an obstacle, as before. A box 0.1 m high at x = 0.65 in the view (the middle of the traverse), its top at -0.6: the
/// lowest body over it is the payload on the way there, its bottom 0.802 m below the tool (centre 0.752 m, half its
/// 0.1 m), and the flange on the way back, 0.702 m below (centre 0.692 m, half its 0.02 m). So the lifts go to
/// -0.6 + 0.05 + 0.802 = 0.252 m and -0.6 + 0.05 + 0.702 = 0.152 m. Planned at rest that keeps 0.05 m exactly; the run
/// sways the payload some 0.05 mm nearer, so the optimiser runs the setting again with the lifts higher by that (three
/// cycles: the cycle as configured, the lifts on the box, a little higher), within 0.2 mm of the hand values. The
/// optimised cycle runs with effects.crash on, touches nothing and passes every verdict.
#[test]
fn the_optimiser_lifts_each_traverse_over_a_box_across_it_by_hand() {
    use sim_components::analysis::optimise_cycle;
    let entry = find("t-gantry").unwrap();
    let clearance = "profile.min_clearance_m = 0.05";
    let found = optimise_cycle(entry, &with(entry, &[clearance]), 2).unwrap();
    assert!(
        found.lifts.len() == 2 && found.lifts.iter().all(|l| l.before == 0.35 && (l.after - 0.1).abs() < 1e-12),
        "{:?}",
        found.lifts
    );
    let boxed = r#"obstacles = [{ name = "box", centre_m = [0.65, -0.05, -0.65], size_m = [0.1, 0.2, 0.1] }]"#;
    let found = optimise_cycle(entry, &with(entry, &[clearance, boxed]), 3).unwrap();
    assert_eq!(found.lifts.iter().map(|l| l.segments).collect::<Vec<_>>(), [[1, 2, 3], [4, 5, 0]]);
    for (lift, by_hand) in found.lifts.iter().zip([-0.6 + 0.05 + 0.802, -0.6 + 0.05 + 0.702]) {
        assert!(
            lift.after >= by_hand - 1e-12 && lift.after < by_hand + 2e-4 && lift.via.is_none(),
            "{lift:?}, by hand {by_hand} m"
        );
    }
    let mut applied = with(entry, &[clearance, boxed, "effects.crash = true"]);
    for (path, _, after) in &found.changes {
        applied.set_text(&format!("{path} = {after}")).unwrap();
    }
    let load = analyse_cycle(&mut Sim::new(entry, &applied).unwrap()).unwrap();
    assert!(load.contact.is_none() && load.clearance.is_some_and(|c| c >= 0.05), "{:?}", load.clearance);
    assert!(verdicts(&applied, &load).unwrap().iter().all(|v| v.ok));
    assert!((load.duration - found.after.duration).abs() < 1e-9);
}

/// The height profile of a lift follows the obstacles' shape. T-gantry, clearance 0.05 m (the lifts go down to 0.1 m
/// without obstacles), two boxes 0.1 m wide across the traverse: at x = 0.45 in the view (0.3 m of the axis, the view is
/// 0.15 m to the right) with its top at -0.7, and at 0.85 (0.7 m) with its top at -0.6. On the way to the place the
/// payload hangs lowest, its bottom 0.802 m below the tool: the via points pass the first box at
/// -0.7 + 0.05 + 0.802 = 0.152 m and the second at -0.6 + 0.05 + 0.802 = 0.252 m, two points each (where the traverse at
/// 0.1 m begins and ends to come nearer than the clearance), the first pair before the second. On the way back the
/// flange is lowest, 0.702 m below the tool: at 0.1 m it keeps 0.1 - 0.702 + 0.7 = 0.098 m from the first box, so only
/// the second is in the way and there is no profile (one box: the lift at -0.6 + 0.05 + 0.702 = 0.152 m, as before).
/// With the first box alone there is none either way. The optimised cycle runs with effects.crash on, touches nothing
/// and passes every verdict.
#[test]
fn the_lift_passes_two_boxes_each_at_its_own_height() {
    use sim_components::analysis::{lift_profiles, optimise_cycle};
    let entry = find("t-gantry").unwrap();
    let clearance = "profile.min_clearance_m = 0.05";
    let boxes = r#"obstacles = [
        { name = "low", centre_m = [0.45, -0.05, -0.75], size_m = [0.1, 0.2, 0.1] },
        { name = "high", centre_m = [0.85, -0.05, -0.65], size_m = [0.1, 0.2, 0.1] },
    ]"#;
    let cfg = with(entry, &[clearance, boxes]);
    let profiles = lift_profiles(entry, &cfg).unwrap();
    assert_eq!(profiles.len(), 1, "{profiles:?}");
    let (segments, points, raised) = &profiles[0];
    assert_eq!(*segments, [1, 2, 3]);
    let heights: Vec<f64> = points.iter().map(|p| p[1]).collect();
    let (low, high) = (-0.7 + 0.05 + 0.802, -0.6 + 0.05 + 0.802);
    assert!(heights.len() == 4, "{points:?}");
    for (height, by_hand) in heights.iter().zip([low, low, high, high]) {
        assert!((height - by_hand).abs() < 1e-6, "{points:?}, by hand {by_hand} m");
    }
    // Along the traverse in order, the first pair around the first box (0.3 m), the second around the second (0.7 m).
    let x: Vec<f64> = points.iter().map(|p| p[0]).collect();
    assert!(x.windows(2).all(|w| w[0] < w[1]) && x[0] < 0.3 && x[1] > 0.3 && x[2] < 0.7 && x[3] > 0.7, "{x:?}");
    let one = r#"obstacles = [{ name = "low", centre_m = [0.45, -0.05, -0.75], size_m = [0.1, 0.2, 0.1] }]"#;
    assert!(lift_profiles(entry, &with(entry, &[clearance, one])).unwrap().is_empty());

    // The cycle through the profile, its points raised as the optimiser raises them for the spline's sag (about 12 mm
    // here, never more than the 0.1 m between the two heights): the lift, traverse and lower are one segment from the
    // pick at 0.05 m up to 0.1 m, through the points, down at the place.
    assert!(*raised >= 0.0 && *raised < 0.1, "{raised} m");
    let mut via = vec![vec![0.1, 0.1]];
    via.extend(points.iter().map(|p| vec![p[0], p[1] + raised]));
    via.push(vec![0.9, 0.1]);
    let profiled = format!(
        "cycle.segment = [{{ target = [0.1, 0.05], dwell_s = 0.1, payload = true }}, \
         {{ target = [0.9, 0.05], dwell_s = 0.1, payload = false, via = {via:?} }}, \
         {{ target = [0.9, 0.35], dwell_s = 0.0, payload = false }}, \
         {{ target = [0.1, 0.35], dwell_s = 0.0, payload = false }}]"
    );
    let crashing = with(entry, &[clearance, boxes, "effects.crash = true", &profiled]);
    let load = analyse_cycle(&mut Sim::new(entry, &crashing).unwrap()).unwrap();
    // At rest the spline keeps the clearance; the run sways the payload some hundredths of a millimetre nearer.
    assert!(load.contact.is_none() && load.clearance.is_some_and(|c| c > 0.05 - 1e-3), "{:?}", load.clearance);

    let found = optimise_cycle(entry, &cfg, 40).unwrap();
    let mut applied = with(entry, &[clearance, boxes, "effects.crash = true"]);
    for (path, _, after) in &found.changes {
        applied.set_text(&format!("{path} = {after}")).unwrap();
    }
    let load = analyse_cycle(&mut Sim::new(entry, &applied).unwrap()).unwrap();
    assert!(load.contact.is_none() && load.clearance.is_some_and(|c| c >= 0.05), "{:?}", load.clearance);
    assert!(verdicts(&applied, &load).unwrap().iter().all(|v| v.ok));
}

/// The time-optimal plan respects the obstacles: a path that comes nearer than `profile.min_clearance_m` is refused,
/// the same path within the axis limits is planned (and `check_plan` says how near it comes).
#[test]
fn a_time_optimal_path_into_an_obstacle_is_refused() {
    let entry = find("single-axis").unwrap();
    let wall = r#"obstacles = [{ name = "wall", centre_m = [0.75, -0.03, 0.06], size_m = [0.1, 0.2, 0.2] }]"#;
    let path = [Piece::Line(vec![0.9])];
    let sim = Sim::new(entry, &with(entry, &[wall])).unwrap();
    assert_eq!(sim.plan_path(&path, true).unwrap_err().key, "error-path-collision");
    assert!(sim.plan_path(&path, false).is_ok());
    // Short of the wall by 0.1 m: refused at a clearance of 0.15 m, planned at 0.05 m.
    let short = [Piece::Line(vec![0.37])];
    assert!(
        Sim::new(entry, &with(entry, &[wall, "profile.min_clearance_m = 0.15"]))
            .unwrap()
            .plan_path(&short, true)
            .is_err()
    );
    assert!(
        Sim::new(entry, &with(entry, &[wall, "profile.min_clearance_m = 0.05"]))
            .unwrap()
            .plan_path(&short, true)
            .is_ok()
    );
    assert!(Sim::new(entry, &entry.config()).unwrap().plan_path(&path, true).is_ok());
}

/// The collision bodies are built without the rest of the scene (it was drawn every base step), and are still the
/// boxes the scene draws, to the last bit, and capsules along segments it draws (the deltas' rods, arms and shaft): on
/// every machine, during its cycle (the T-gantry's Z axis leaning with tool flex and its frame swaying), with and
/// without the payload.
#[test]
fn the_collision_bodies_are_boxes_of_the_scene() {
    for entry in MACHINES {
        let mut sim = Sim::new(entry, &entry.config()).unwrap();
        sim.command(Command::StartCycle).unwrap();
        let mut checked = 0;
        for k in 0..12 {
            sim.run(0.1, |_| {});
            sim.machine.set_payload_attached(k % 2 == 0);
            let mut scene = sim_core::Scene::default();
            sim.machine.scene(&sim.state, &mut scene);
            let bodies = sim.machine.collision_bodies(&sim.state);
            assert!(!bodies.is_empty(), "{}", entry.id);
            // A capsule is a drawn segment with half its width as radius.
            for capsule in sim.machine.collision_capsules(&sim.state) {
                let drawn = scene.parts.iter().any(|p| match p.shape {
                    sim_core::Shape::Segment { to, width } => {
                        let ends = |a: [f64; 3], b: [f64; 3]| (0..3).all(|i| (a[i] - b[i]).abs() < 1e-12);
                        ends(p.pos, capsule.from) && ends(to, capsule.to) && capsule.radius == width / 2.0
                    }
                    _ => false,
                });
                assert!(drawn, "{}: {capsule:?} is not drawn", entry.id);
            }
            for body in bodies {
                // A box drawn as such, or the box around a piece of a drawn segment (the delta's forearm rods): its
                // centre on the segment, no wider than the segment's extent plus its width.
                let along = |from: [f64; 3], to: [f64; 3], width: f64| {
                    let d: Vec<f64> = (0..3).map(|i| to[i] - from[i]).collect();
                    let f = (0..3).map(|i| (body.centre[i] - from[i]) * d[i]).sum::<f64>()
                        / d.iter().map(|v| v * v).sum::<f64>();
                    (0.0..=1.0).contains(&f)
                        && (0..3).all(|i| {
                            (from[i] + f * d[i] - body.centre[i]).abs() < 1e-9
                                && body.size[i] <= d[i].abs() + width + 1e-12
                        })
                };
                let drawn = scene.parts.iter().any(|p| match p.shape {
                    sim_core::Shape::Box { size } => size.map(f64::abs) == body.size && p.pos == body.centre,
                    sim_core::Shape::Segment { to, width } => along(p.pos, to, width),
                    _ => false,
                });
                assert!(drawn, "{}: {body:?} is not drawn", entry.id);
                checked += 1;
            }
        }
        // A body each sample and the payload's every other: the rotary knife's one blade and no payload give 12, as the
        // flying saw's carriage with its saw drawn as a disc.
        let least = if ["rotary-knife", "flying-saw"].contains(&entry.id) { 12 } else { 18 };
        assert!(checked >= least, "{}: {checked}", entry.id);
    }
}

/// An obstacle entry with a misspelt key, a size of zero or two numbers is refused by name.
#[test]
fn a_bad_obstacle_is_refused() {
    let entry = find("single-axis").unwrap();
    for bad in [
        r#"obstacles = [{ name = "a", centre = [0.5, 0.0, 0.0], size_m = [0.1, 0.1, 0.1] }]"#,
        r#"obstacles = [{ name = "a", centre_m = [0.5, 0.0, 0.0], size_m = [0.1, 0.0, 0.1] }]"#,
        r#"obstacles = [{ name = "a", centre_m = [0.5, 0.0], size_m = [0.1, 0.1, 0.1] }]"#,
    ] {
        let err = Sim::new(entry, &with(entry, &[bad])).err().unwrap();
        assert_eq!(err, sim_core::ConfigError::Invalid("obstacles[0]".into()));
    }
}

/// The tool's position on a one-axis machine, and where the single axis's carriage front is then (0.15 + x + 0.08).
fn carriage_front(sim: &Sim) -> (f64, f64) {
    let (mut pos, mut vel) = ([0.0], [0.0]);
    sim.machine.tool_state(&sim.state, &mut pos, &mut vel);
    (pos[0], 0.15 + pos[0] + 0.08)
}

/// After a crash into the wall of `a_move_into_a_box_is_blocked_at_its_face_and_trips_on_the_following_error` the drives
/// are enabled again (which acknowledges the trip) and take the carriage back to 0.1; a move into the wall again is
/// blocked at the face and trips them again. Held where the crash left them, they do not trip.
#[test]
fn after_a_crash_the_drives_are_enabled_again_and_move_out() {
    let entry = find("single-axis").unwrap();
    let wall = r#"obstacles = [{ name = "wall", centre_m = [0.75, -0.03, 0.06], size_m = [0.1, 0.2, 0.2] }]"#;
    let monitored = [wall, "effects.crash = true", "effects.drive_monitoring = true"];
    let mut sim = Sim::new(entry, &with(entry, &monitored)).unwrap();
    sim.command(Command::MoveTo(vec![0.9])).unwrap();
    sim.run(1.0, |_| {});
    assert!(sim.crashed && !sim.control.enabled());
    sim.command(Command::Enable).unwrap();
    assert!(sim.control.fault.is_none());
    sim.run(0.25, |_| {});
    assert!(sim.control.enabled() && !sim.crashed, "the drives tripped at rest at {:?}", carriage_front(&sim));
    sim.command(Command::MoveTo(vec![0.1])).unwrap();
    sim.run(0.01, |_| {});
    assert!(sim.control.enabled() && !sim.crashed, "the drives tripped again at {:?}", carriage_front(&sim));
    sim.run(1.5, |_| {});
    let (at, _) = carriage_front(&sim);
    assert!(sim.control.enabled() && (at - 0.1).abs() < 1e-3, "at {at}");
    sim.command(Command::MoveTo(vec![0.9])).unwrap();
    sim.run(1.5, |_| {});
    let (_, front) = carriage_front(&sim);
    assert!(sim.crashed && !sim.control.enabled() && front < 0.703 && front > 0.68, "front {front}");
}

/// With effects.crash the obstacles push with the drives off too. The single axis coasting at 1 m/s from x = 0.44
/// with its drives off and no holding brake is stopped by the wall and pushed back from it, at most 3 mm in (the
/// carriage swings on its belt; the push holds the motor) and no more than 0.1 m back, where its friction stops it.
/// The T-gantry lowers its payload onto a table at the pick position (top at -0.70, the payload's bottom 0.802 m below
/// the tool's z in the view, so at z = 0.102) without a holding brake: blocked, its drives trip on their following
/// error and stop, and the Z axis rests on the table, no more than 0.1 mm in (its weight on the belts' give, as the
/// push holds the motors, and the push's own depth). The energy budget of the T-gantry's run closes.
#[test]
fn obstacles_stop_the_machine_with_its_drives_off() {
    let entry = find("single-axis").unwrap();
    let wall = r#"obstacles = [{ name = "wall", centre_m = [0.75, -0.03, 0.06], size_m = [0.1, 0.2, 0.2] }]"#;
    let mut sim =
        Sim::new(entry, &with(entry, &[wall, "effects.crash = true", "effects.holding_brake = false"])).unwrap();
    sim.command(Command::MoveTo(vec![0.9])).unwrap();
    while carriage_front(&sim).0 < 0.44 {
        sim.step();
    }
    sim.command(Command::Disable).unwrap();
    sim.run(0.5, |_| {});
    let (_, front) = carriage_front(&sim);
    assert!(sim.contact.is_some() && !sim.crashed && sim.crash_energy > 0.0, "{:?}", sim.contact);
    assert!(front < 0.703 && front > 0.6, "front {front}");

    let entry = find("t-gantry").unwrap();
    let table = r#"obstacles = [{ name = "table", centre_m = [0.25, -0.05, -0.75], size_m = [0.2, 0.2, 0.1] }]"#;
    let settings = [table, "effects.crash = true", "effects.holding_brake = false", "effects.drive_monitoring = true"];
    let mut sim = Sim::new(entry, &with(entry, &settings)).unwrap();
    let points = sim.plan_path(&[Piece::Line(vec![0.1, 0.05])], false).unwrap();
    let load = sim_components::analysis::analyse_trajectory(&mut sim, points).unwrap();
    sim.run(0.5, |_| {});
    let (mut pos, mut vel) = ([0.0; 2], [0.0; 2]);
    sim.machine.tool_state(&sim.state, &mut pos, &mut vel);
    println!("z {} m, clearance {} m, speed {} m/s", pos[1], sim.clearance, vel[1]);
    assert!(
        sim.crashed
            && sim.clearance > -1e-4
            && sim.clearance < 1e-3
            && (pos[1] - 0.102).abs() < 1e-4
            && vel[1].abs() < 1e-3,
        "z {} m, clearance {} m",
        pos[1],
        sim.clearance
    );
    // The drives draw 0.18 J for a lower that gravity drives: 17 J of potential energy go through the losses, which is
    // what the budget's error (the losses summed step by step) scales with; to `BUDGET_TOLERANCE` of the two.
    let budget = load.budget.unwrap();
    let potential = budget.rows.iter().find(|r| r.id == "energy.potential").unwrap().energy.abs();
    let crash = budget.rows.iter().find(|r| r.id == "crash").unwrap().energy;
    assert!(load.contact.is_some() && crash > 0.0, "crash {crash} J");
    assert!(
        budget.unaccounted().abs() < sim_components::analysis::BUDGET_TOLERANCE * (budget.supply + potential),
        "{} J of {} J and {potential} J not accounted for",
        budget.unaccounted(),
        budget.supply
    );
}

/// The machines that keep an obstacle's push at their motors (`Machine::set_tool_force` false): the crane's tool is its
/// bridge, trolley and hoist, not the load that swings under them; the winder's is the web.
const PUSHED_AT_THE_MOTORS: [&str; 2] = ["overhead-crane", "winder"];

/// Every machine but those of `PUSHED_AT_THE_MOTORS` takes the push at its tool, and its power goes into the motion: on
/// its lossless suite config at the start pose, the tool moving at 5 % of each axis's range per second
/// (`set_tool_velocity`), a push F on the tool changes the rate of the machine's energy, grad E . (s' with F - s'
/// without), by F . v_tool, what the simulation books as the obstacles' work (the crash row). By central differences of
/// `energy`, to 1e-5.
#[test]
fn a_push_at_the_tool_works_on_the_machine_at_the_tools_speed() {
    let mut wrong = Vec::new();
    for entry in MACHINES {
        let cfg = (entry.suite_configs)().into_iter().next().unwrap_or_else(|| entry.config());
        let Sim { mut machine, state, .. } = Sim::new(entry, &cfg).unwrap();
        let (n, len) = (machine.axes().len(), machine.state_len());
        let force: Vec<f64> = (0..n).map(|i| 50.0 * (1.0 + i as f64)).collect();
        let takes = machine.set_tool_force(&force);
        if takes == PUSHED_AT_THE_MOTORS.contains(&entry.id) {
            wrong.push(format!("{}: takes the push at its tool {takes}", entry.id));
        }
        if !takes {
            continue;
        }
        let mut s = state[..len].to_vec();
        let speed: Vec<f64> = machine.axes().iter().map(|a| 0.05 * (a.max - a.min)).collect();
        machine.set_tool_velocity(&mut s, &speed);
        let torque = vec![0.0; machine.motors().len()];
        let (mut pushed, mut free) = (vec![0.0; len], vec![0.0; len]);
        machine.derivatives(&s, &torque, &mut pushed);
        machine.set_tool_force(&vec![0.0; n]);
        machine.derivatives(&s, &torque, &mut free);
        let rate: f64 = (0..len)
            .map(|i| {
                let h = 1e-6 * s[i].abs().max(1.0);
                let (mut up, mut down) = (s.clone(), s.clone());
                (up[i], down[i]) = (s[i] + h, s[i] - h);
                (machine.energy(&up) - machine.energy(&down)) / (2.0 * h) * (pushed[i] - free[i])
            })
            .sum();
        let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
        machine.tool_state(&s, &mut pos, &mut vel);
        let power: f64 = force.iter().zip(&vel).map(|(f, v)| f * v).sum();
        println!("{}: {rate} W against F . v = {power} W", entry.id);
        if !(power.abs() > 1e-3 && (rate - power).abs() < 1e-5 * power.abs()) {
            wrong.push(format!("{}: {rate} W, not {power} W", entry.id));
        }
    }
    assert!(wrong.is_empty(), "{wrong:#?}");
}

/// What a move into a slab with effects.crash leaves (`push_into_a_slab`).
#[derive(Debug)]
struct PushRun {
    /// Whether the machine takes the push at its tool (`Machine::set_tool_force`).
    takes: bool,
    /// At rest against the slab: how deep the parts stand in it (m) and its force (N).
    depth: f64,
    force: f64,
    /// The most of each over the run: the impact's (with the push at the motors a load on a soft transmission swings
    /// on into the slab by the transmission's give and springs back).
    peak_depth: f64,
    peak_force: f64,
    /// The planned move into it: the budget's crash row, what is not accounted for, and the supply plus the potential
    /// energy's change, which the budget's error scales with (J).
    crash: f64,
    unaccounted: f64,
    scale: f64,
}

/// Machine `id` moves from where it starts to `target` (tool coordinates) into a slab that the obstacle's contact
/// stiffness k = 1e7 N/m holds: its face (normal to the scene axis the tool moves most along) is where the furthest of
/// the moving parts are at 70 % of the move, so that the drives push on into it at their limit. Run for `seconds` with
/// the move commanded, then once more as a planned trajectory with the energy budget.
fn push_into_a_slab(id: &str, target: &[f64], seconds: f64) -> PushRun {
    let entry = find(id).unwrap();
    let mut probe = Sim::new(entry, &entry.config()).unwrap();
    let n = probe.machine.axes().len();
    let takes = probe.machine.set_tool_force(&vec![0.0; n]);
    let machine = probe.machine.as_ref();
    let (mut start, mut vel) = (vec![0.0; n], vec![0.0; n]);
    machine.tool_state(&probe.state, &mut start, &mut vel);
    let (a, b) = (machine.scene_point(&start).unwrap(), machine.scene_point(target).unwrap());
    let axis = (0..3).max_by(|&i, &j| (b[i] - a[i]).abs().total_cmp(&(b[j] - a[j]).abs())).unwrap();
    let sign = (b[axis] - a[axis]).signum();
    // Each part's bounds at a pose; the slab is that of the parts the move moves: its face where the furthest of them
    // is, across the axis 0.1 m beyond them all (so that it does not meet a part the move leaves where it is).
    let bounds = |pose: &[f64]| -> Vec<[[f64; 3]; 2]> {
        let (mut state, mut torque) = (Vec::new(), vec![0.0; machine.motors().len()]);
        machine.equilibrium(pose, &mut state, &mut torque);
        let bodies = sim_components::collision::Bodies::of(machine, &state);
        let boxes = bodies.boxes.iter().map(|b| {
            [
                std::array::from_fn(|i| b.centre[i] - 0.5 * b.size[i]),
                std::array::from_fn(|i| b.centre[i] + 0.5 * b.size[i]),
            ]
        });
        let rods = bodies.capsules.iter().map(|c| {
            [
                std::array::from_fn(|i| c.from[i].min(c.to[i]) - c.radius),
                std::array::from_fn(|i| c.from[i].max(c.to[i]) + c.radius),
            ]
        });
        boxes.chain(rods).collect()
    };
    let at: Vec<f64> = start.iter().zip(target).map(|(s, t)| s + 0.7 * (t - s)).collect();
    let (before, after) = (bounds(&start), bounds(&at));
    let moved: Vec<[[f64; 3]; 2]> =
        before.iter().zip(&after).filter(|(b, a)| (a[1][axis] - b[1][axis]).abs() > 1e-6).map(|(_, a)| *a).collect();
    let low = |i: usize| moved.iter().map(|b| b[0][i]).fold(f64::INFINITY, f64::min);
    let high = |i: usize| moved.iter().map(|b| b[1][i]).fold(f64::NEG_INFINITY, f64::max);
    let face = if sign > 0.0 { high(axis) } else { low(axis) };
    let mut centre: [f64; 3] = std::array::from_fn(|i| 0.5 * (low(i) + high(i)));
    let mut size: [f64; 3] = std::array::from_fn(|i| high(i) - low(i) + 0.2);
    (centre[axis], size[axis]) = (face + sign * 5.0, 10.0);
    let slab = format!("obstacles = [{{ name = \"slab\", centre_m = {centre:?}, size_m = {size:?} }}]");
    let cfg = with(entry, &[&slab, "effects.crash = true"]);

    let mut sim = Sim::new(entry, &cfg).unwrap();
    sim.command(Command::MoveTo(target.to_vec())).unwrap();
    let (mut peak_depth, mut peak_force) = (0.0f64, 0.0f64);
    while sim.time() < seconds {
        sim.step();
        (peak_depth, peak_force) = (peak_depth.max(-sim.clearance), peak_force.max(sim.contact_force));
    }
    let (depth, force) = (-sim.clearance, sim.contact_force);
    assert!(sim.contact.is_some(), "{id}: no contact with {slab}");

    let mut sim = Sim::new(entry, &cfg).unwrap();
    let points = sim.plan_path(&[Piece::Line(target.to_vec())], false).unwrap();
    let load = sim_components::analysis::analyse_trajectory(&mut sim, points).unwrap();
    let budget = load.budget.unwrap();
    let row = |id: &str| budget.rows.iter().find(|r| r.id == id).map_or(0.0, |r| r.energy);
    let run = PushRun {
        takes,
        depth,
        force,
        peak_depth,
        peak_force,
        crash: row("crash"),
        unaccounted: budget.unaccounted(),
        scale: budget.supply + row("energy.potential").abs(),
    };
    println!("{id}: {run:?}, F / k = {} m", force / 1e7);
    run
}

/// The push at the tool: the parts stand in the slab by its contact spring's give alone, F / k (within 5 %, as on the
/// machine file's belt axis), not by that and the transmission's give as when the push holds the motors; and the
/// budget of the planned move into it closes with the crash row, within `BUDGET_TOLERANCE` of the supply and the
/// potential energy's change.
fn assert_stops_at_the_face(id: &str, target: &[f64], seconds: f64) {
    let run = push_into_a_slab(id, target, seconds);
    assert!(run.takes, "{id} does not take the push at its tool");
    assert!(run.force > 10.0, "{id}: {} N", run.force);
    let hand = run.force / 1e7;
    assert!((run.depth - hand).abs() < 0.05 * hand, "{id}: depth {} m against F / k = {hand} m", run.depth);
    assert!(run.crash > 0.0, "{id}: crash {} J", run.crash);
    assert!(
        run.unaccounted.abs() < sim_components::analysis::BUDGET_TOLERANCE * run.scale,
        "{id}: {} J of {} J not accounted for",
        run.unaccounted,
        run.scale
    );
}

#[test]
fn the_single_axis_stops_at_the_face() {
    assert_stops_at_the_face("single-axis", &[0.9], 2.5);
}
#[test]
fn the_t_gantry_stops_at_the_face() {
    assert_stops_at_the_face("t-gantry", &[0.9, 0.35], 2.5);
}
#[test]
fn the_screw_axis_stops_at_the_face() {
    assert_stops_at_the_face("screw-axis", &[0.7], 2.5);
}
#[test]
fn the_rack_axis_stops_at_the_face() {
    assert_stops_at_the_face("rack-axis", &[2.5], 3.0);
}
#[test]
fn the_linear_motor_axis_stops_at_the_face() {
    assert_stops_at_the_face("linear-motor-axis", &[0.9], 2.5);
}
#[test]
fn the_palletiser_stops_at_the_face() {
    assert_stops_at_the_face("palletiser", &[1.2, -0.5, 0.8, 0.0], 3.0);
}
#[test]
fn the_urdf_arm_stops_at_the_face() {
    assert_stops_at_the_face("urdf-arm", &[0.0, 0.2, -1.0, -0.3, -0.5, -1.0], 3.0);
}
#[test]
fn the_scara_stops_at_the_face() {
    assert_stops_at_the_face("scara", &[0.0, 0.5, 0.18, 0.0], 2.5);
}
#[test]
fn the_arm_on_track_stops_at_the_face() {
    assert_stops_at_the_face("arm-on-track", &[2.0, 1.3, 0.2, -1.0, -0.3, -0.5, -1.0], 4.0);
}
#[test]
fn the_mover_track_stops_at_the_face() {
    assert_stops_at_the_face("mover-track", &[0.0, 0.628116, 1.131233, 1.634349, 2.137465, 2.640582], 2.5);
}
#[test]
fn the_delta_linear_stops_at_the_face() {
    assert_stops_at_the_face("delta-linear", &[0.15, 0.0, -1.21], 2.5);
}
#[test]
fn the_planar_cable_robot_stops_at_the_face() {
    assert_stops_at_the_face("cable-planar", &[0.0, 0.4], 2.5);
}
#[test]
fn the_spatial_cable_robot_stops_at_the_face() {
    assert_stops_at_the_face("cable-spatial", &[0.0, 0.0, 0.85, 0.0, 0.0, 0.0], 2.5);
}
// The D2's and D3's arms are the machine files' linkage and spatial delta (`sim_netlist::linkage`,
// `sim_netlist::spatial::Delta`), the D4's the D3's with the push about c on its turn.
#[test]
fn the_delta_d2_stops_at_the_face() {
    assert_stops_at_the_face("delta-d2", &[0.15, -0.8], 2.5);
}
#[test]
fn the_delta_d3_stops_at_the_face() {
    assert_stops_at_the_face("delta-d3", &[0.15, 0.0, -1.0], 2.5);
}
#[test]
fn the_delta_d4_stops_at_the_face() {
    assert_stops_at_the_face("delta-d4", &[0.15, 0.0, -1.0, 0.0], 2.5);
}
#[test]
fn the_delta_h4_stops_at_the_face() {
    assert_stops_at_the_face("delta-h4", &[0.15, 0.0, -1.1, 0.0], 2.5);
}
#[test]
fn the_tripod_stops_at_the_face() {
    assert_stops_at_the_face("tripod", &[0.505, 0.0, 0.0], 2.5);
}
#[test]
fn the_rotary_hexapod_stops_at_the_face() {
    assert_stops_at_the_face("hexapod-rotary", &[0.0, 0.0, 0.35, 0.0, 0.0, 0.0], 2.5);
}
#[test]
fn the_hexapod_stops_at_the_face() {
    assert_stops_at_the_face("hexapod", &[0.0, 0.0, 0.445, 0.0, 0.0, 0.0], 2.5);
}
#[test]
fn the_parallel_scara_stops_at_the_face() {
    assert_stops_at_the_face("parallel-scara", &[0.15, -0.8, 0.075, 0.0], 2.5);
}
#[test]
fn the_wafer_handler_stops_at_the_face() {
    assert_stops_at_the_face("wafer-handler", &[0.7, 0.0, 0.06], 2.5);
}
