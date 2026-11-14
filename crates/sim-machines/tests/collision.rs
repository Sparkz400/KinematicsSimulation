//! Obstacles and crash detection against hand calculations: the clearance of a box beside a straight move, the stop of
//! a move into a box with effects.crash, the check "collision" on a cycle, and the refused time-optimal path.

use sim_components::analysis::{analyse_cycle, verdicts};
use sim_components::collision::check_cycle;
use sim_components::path::Piece;
use sim_components::{Command, Sim};
use sim_core::Config;
use sim_machines::{MACHINES, find};

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
/// cruising at 1 m/s. With effects.crash the carriage stops at the face (put back along the step that went in, where
/// it was up to one base step's travel inside, 62.5 um at 1 m/s) and the drives go off; the impact speed is the
/// planned 1 m/s. Without the crash the carriage runs through.
#[test]
fn a_move_into_a_box_stops_at_its_face_at_the_planned_speed() {
    let entry = find("single-axis").unwrap();
    let wall = r#"obstacles = [{ name = "wall", centre_m = [0.75, -0.03, 0.06], size_m = [0.1, 0.2, 0.2] }]"#;
    let planned = Sim::new(entry, &with(entry, &[wall])).unwrap().check_plan(&[vec![0.1], vec![0.9]]).unwrap();
    let contact_pose = planned.contact.as_ref().unwrap()[0];
    assert!(planned.clearance < 0.0 && (contact_pose - 0.47).abs() <= 1e-3, "{planned:?}");

    let mut sim = Sim::new(entry, &with(entry, &[wall, "effects.crash = true"])).unwrap();
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
    assert!(!sim.control.enabled());
    sim.run(0.5, |_| {});
    let (mut pos, mut vel) = ([0.0], [0.0]);
    sim.machine.tool_state(&sim.state, &mut pos, &mut vel);
    let front = 0.15 + pos[0] + 0.08;
    assert!((front - 0.70).abs() < 1e-9 && vel[0].abs() < 1e-3, "front {front}, speed {}", vel[0]);

    let mut through = Sim::new(entry, &with(entry, &[wall])).unwrap();
    through.command(Command::MoveTo(vec![0.9])).unwrap();
    through.run(2.0, |_| {});
    let mut scene = sim_core::Scene::default();
    through.scene(&mut scene);
    assert!(through.control.enabled() && (through.control.set_pos[0] - 0.9).abs() < 1e-9 && through.contact.is_some());
    // Through and past it: at x = 0.9 the carriage's back is at 0.97, beyond the wall's far face at 0.80.
    assert!(!scene.obstacles[0].contact && scene.first_contact == through.contact.as_ref().map(|c| c.time));

    // The energy budget of the same move with the crash closes, to `BUDGET_TOLERANCE` as on the shipped cycles: what the
    // stop took out of the machine is its own row, `crash` (it was not booked, and the budget was off by it), and so is
    // what the DC link of the drives, off from the stop on, lost through its bleeder, `discharge` (0.07 J, 0.3 %); the
    // electrical side closes to `ELECTRICAL_TOLERANCE`.
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
/// boxes the scene draws, to the last bit, or boxes around pieces of a segment it draws (the delta's forearms): on
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
        assert!(checked >= 18, "{}: {checked}", entry.id);
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

/// After a crash into the wall of `a_move_into_a_box_stops_at_its_face_at_the_planned_speed` the drives are enabled
/// again and take the carriage back to 0.1; a move into the wall again stops it at the face and trips them again. (The
/// stop used to leave the carriage up to one step's travel inside, and every step after `Enable` found the contact and
/// tripped the drives: the machine could not be powered up again.)
#[test]
fn after_a_crash_the_drives_are_enabled_again_and_move_out() {
    let entry = find("single-axis").unwrap();
    let wall = r#"obstacles = [{ name = "wall", centre_m = [0.75, -0.03, 0.06], size_m = [0.1, 0.2, 0.2] }]"#;
    let mut sim = Sim::new(entry, &with(entry, &[wall, "effects.crash = true"])).unwrap();
    sim.command(Command::MoveTo(vec![0.9])).unwrap();
    sim.run(1.0, |_| {});
    assert!(sim.crashed && !sim.control.enabled());
    sim.command(Command::Enable).unwrap();
    // Held at the face, the drives stay on (they settle some 25 nm in; rounding alone moved it 4e-16 m in after 69
    // steps, which tripped them).
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
    assert!(sim.crashed && !sim.control.enabled() && (front - 0.70).abs() < 1e-9, "front {front}");
}

/// With effects.crash the obstacles are rigid with the drives off too. The single axis coasting at 1 m/s from x = 0.44
/// with its drives off and no holding brake stops at the wall's face (it went 51 mm in). The T-gantry crashes its payload
/// into a table at the pick position (top at -0.70, the payload's bottom 0.802 m below the tool's z in the view, so at
/// z = 0.102) without a holding brake: the Z axis rests on the table (it fell the whole 0.1 m through). Both stop
/// at the face (the T-gantry within `CREEP`, 1 um, of it), and the energy budget of the T-gantry's run closes.
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
    // Stopped at the face, it is free to leave it: with no torque to hold it there it drifts a few um back.
    assert!(front <= 0.70 + 1e-9 && front > 0.70 - 1e-4, "front {front}");

    let entry = find("t-gantry").unwrap();
    let table = r#"obstacles = [{ name = "table", centre_m = [0.25, -0.05, -0.75], size_m = [0.2, 0.2, 0.1] }]"#;
    let mut sim =
        Sim::new(entry, &with(entry, &[table, "effects.crash = true", "effects.holding_brake = false"])).unwrap();
    let points = sim.plan_path(&[Piece::Line(vec![0.1, 0.05])], false).unwrap();
    let load = sim_components::analysis::analyse_trajectory(&mut sim, points).unwrap();
    sim.run(0.5, |_| {});
    let (mut pos, mut vel) = ([0.0; 2], [0.0; 2]);
    sim.machine.tool_state(&sim.state, &mut pos, &mut vel);
    println!("z {} m, clearance {} m, speed {} m/s", pos[1], sim.clearance, vel[1]);
    // It may creep `CREEP` (1 um) in before it is put back where it stood.
    assert!(
        sim.crashed
            && sim.clearance > -2e-6
            && sim.clearance < 1e-3
            && (pos[1] - 0.102).abs() < 2e-6
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
