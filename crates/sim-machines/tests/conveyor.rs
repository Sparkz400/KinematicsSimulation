//! Picking from a moving conveyor (`[conveyor]`) against hand calculations: picks a minute from the pass's time, the
//! tool's speed along the belt during the grip, the RMS torque of a belt pick against a stationary one, parts missed by
//! a belt faster than the picks, the belt in the scene, and an obstacle in the pass.

use sim_components::analysis::{analyse_cycle, verdicts};
use sim_components::{Command, Sim, load_config};
use sim_core::Config;
use sim_machines::find;

/// The delta's example config, with `settings` on top.
fn example(settings: &[&str]) -> Config {
    let entry = find("delta-d3").unwrap();
    let text =
        std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../machine-delta-d3/conveyor.toml")).unwrap();
    let mut cfg = load_config(entry, &text).unwrap();
    for s in settings {
        cfg.set_text(s).unwrap();
    }
    cfg
}

fn verdict(cfg: &Config, load: &sim_components::analysis::CycleLoad, check: &str) -> sim_components::analysis::Verdict {
    verdicts(cfg, load).unwrap().into_iter().find(|v| v.check == check).unwrap()
}

/// The example's belt brings a part every 1.0 s; the delta's own pass takes 0.94 s, so it waits for each part to reach
/// the window's start, and its passes come a pitch apart: 1.0 s, 60 picks a minute = 60 / cycle time. The belt brings
/// 60 a minute: none missed. The tool stays within the 1 mm tolerance of the part along the belt during the grip.
#[test]
fn the_example_picks_a_part_each_pitch_and_misses_none() {
    let cfg = example(&[]);
    let mut sim = Sim::new(find("delta-d3").unwrap(), &cfg).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    let c = load.conveyor.unwrap();
    assert!((load.duration - 1.0).abs() < 1e-3, "{}", load.duration);
    assert!((c.picks_per_minute - 60.0 / load.duration).abs() < 1e-12);
    assert!((c.picks_per_minute - 60.0).abs() < 0.1, "{}", c.picks_per_minute);
    assert_eq!((c.missed, c.missed_per_minute), (0, 0.0));
    let tracking = verdict(&cfg, &load, "pick-tracking");
    assert!(tracking.ok && tracking.limit == 0.001 && tracking.value > 0.0, "{tracking:?}");
    assert!(verdict(&cfg, &load, "parts-missed").ok);
}

/// During the grip the plan moves with the belt at 0.3 m/s along x, exactly; the tool follows within the tolerance:
/// a tool at 0.3 m/s + tolerance / grip time = 0.3 + 0.001 / 0.035 = 0.3286 m/s for the whole grip would end the
/// tolerance off the part, so its speed along the belt stays within 0.3 +- 0.0286 m/s.
#[test]
fn during_the_grip_the_tool_moves_along_the_belt_at_its_speed() {
    let cfg = example(&[]);
    let mut sim = Sim::new(find("delta-d3").unwrap(), &cfg).unwrap();
    sim.command(Command::StartCycle).unwrap();
    let (mut samples, mut tool, mut speed) = (0, vec![0.0; 3], vec![0.0; 3]);
    for _ in 0..(2.2 / sim.dt) as usize {
        sim.step();
        let (conveyor, pass) = sim.control.conveyor().unwrap();
        if pass.and_then(|p| conveyor.gripping(p, sim.time())).is_none() {
            continue;
        }
        sim.machine.tool_state(&sim.state, &mut tool, &mut speed);
        assert!((sim.control.set_vel[0] - 0.3).abs() < 1e-9 && sim.control.set_vel[1].abs() < 1e-9);
        assert!((speed[0] - 0.3).abs() <= 0.001 / 0.035, "{} m/s at {} s", speed[0], sim.time());
        samples += 1;
    }
    // Two grips of 35 ms in 2.2 s (from 1.0 and 2.0 s on), each 35 ms / 62.5 us = 560 steps.
    assert!((1100..=1130).contains(&samples), "{samples}");
}

/// A belt that brings a part every 0.5 s, 120 a minute, is faster than the delta's 0.94 s pass: the pass no longer
/// waits, and at least 120 - 60 / 0.94 = 56 parts a minute pass the window unpicked. `parts-missed` fails.
#[test]
fn a_belt_faster_than_the_picks_misses_parts() {
    let cfg = example(&["conveyor.pitch_s = 0.5"]);
    let mut sim = Sim::new(find("delta-d3").unwrap(), &cfg).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    let c = load.conveyor.unwrap();
    assert!(load.duration < 0.99, "{}", load.duration);
    assert!(c.missed_per_minute >= 120.0 - c.picks_per_minute - 1e-9, "{c:?}");
    let missed = verdict(&cfg, &load, "parts-missed");
    assert!(!missed.ok && missed.value > 50.0, "{missed:?}");
}

/// The RMS torque of a belt pick against a stationary one, on the single axis with nothing but inertia acting (the
/// friction, the gearbox's losses and the payload switched off; jerk 1e5 m/s^3, so the moves are bang-bang): the motor
/// torque is k a, and over the same 1 s cycle (both wait for their parts) RMS_v / RMS_0 = sqrt(sum int a^2 dt, belt /
/// the same, stationary).
///
/// Stationary: pick at 0.4 m, place at 0.6 m, D = 0.2 m each way at a = 10 m/s^2: T = 2 sqrt(D / a) = 0.28284 s,
/// int a^2 = a^2 T = 28.284 each, 56.569 together.
///
/// Belt at v = 0.3 m/s, gripped at 0.4 m: each move runs with a' = a - 1.5 v / T (the belt term takes the rest), and
/// its blend adds 1.2 v^2 / T to int a^2 (int (v h'(u) / T)^2 dt = 36 / 30 v^2 / T; no cross term, the move's
/// acceleration being odd about T / 2 and the blend's even). The approach from 0.6 m meets the part at the window's
/// start, D = 0.2 + v T / 2 back against the belt: T^2 a' / 4 = D gives a T^2 - 3.5 v T - 4 * 0.2 = 0, T = 0.34018 s,
/// a' = 8.6772: int a^2 = a'^2 T + 1.2 v^2 / T = 25.613 + 0.317 = 25.931. The way back starts 0.05 s of grip later,
/// at 0.415 m, and ends at rest on 0.6 m: D = 0.185 - v T / 2, a T^2 + 0.5 v T - 4 * 0.185 = 0, T = 0.26463 s,
/// a' = 8.2995: 18.228 + 0.408 = 18.636. Together 44.567: RMS_v / RMS_0 = sqrt(44.567 / 56.569) = 0.8876. The faster
/// belt lowers the RMS torque here, by 11 %: the moves give the belt term its share of the limits and stretch.
#[test]
fn a_belt_pick_changes_the_rms_torque_by_the_hand_estimated_share() {
    let entry = find("single-axis").unwrap();
    let rms = |speed: f64, origin: f64, window: &str| {
        let text = format!(
            "{}\n[conveyor]\nenabled = true\ndirection = [1.0]\nspeed_m_s = {speed:?}\nramp_s = 0.0\norigin = [{origin:?}]\n\
             length_m = 0.6\npitch_s = 1.0\narrivals_s = [0.0]\nwindow_m = {window}\nlift = [0.0]\ngrip_s = 0.05\n\
             release_s = 0.05\nplace = [0.6]\nplace_direction = [1.0]\nplace_speed_m_s = 0.0\n\
             tracking_tolerance_m = 0.001\n",
            entry.config().to_text().unwrap()
        );
        let mut cfg = load_config(entry, &text).unwrap();
        for s in [
            "effects.payload = false",
            "effects.guide_friction = false",
            "effects.load_dependent_friction = false",
            "effects.motor_friction = false",
            "effects.gearbox_efficiency = false",
            "effects.torque_ripple = false",
            "axis.x.max_velocity_m_s = 3.0",
            "axis.x.max_jerk_m_s3 = 100000.0",
        ] {
            cfg.set_text(s).unwrap();
        }
        let load = analyse_cycle(&mut Sim::new(entry, &cfg).unwrap()).unwrap();
        assert!((load.duration - 1.0).abs() < 1e-3, "{}", load.duration);
        load.motors[0].rms_torque
    };
    let (stationary, belt) = (rms(0.0, 0.4, "[0.0, 0.5]"), rms(0.3, 0.3, "[0.1, 0.5]"));
    let (a, v): (f64, f64) = (10.0, 0.3);
    let root = |b: f64, c: f64| (-b + (b * b - 4.0 * a * c).sqrt()) / (2.0 * a);
    let squared = |t: f64| (a - 1.5 * v / t).powi(2) * t + 1.2 * v * v / t;
    let (approach, back) = (root(-3.5 * v, -0.8), root(0.5 * v, -0.74));
    let expected = ((squared(approach) + squared(back)) / (2.0 * a * a * 2.0 * (0.2f64 / a).sqrt())).sqrt();
    assert!((expected - 0.8876).abs() < 1e-3, "{expected}");
    assert!((belt / stationary - expected).abs() < 0.02, "{belt} / {stationary} against {expected}");
}

/// The scene draws the belt as one band, ten slats across it (1.0 m long, a slat every 0.1 m) and the parts on it:
/// at 0.5 s part 0, which passed the origin at 0, is 0.15 m along and the only one.
#[test]
fn the_scene_draws_the_belt_its_slats_and_its_parts() {
    let mut sim = Sim::new(find("delta-d3").unwrap(), &example(&[])).unwrap();
    sim.command(Command::StartCycle).unwrap();
    sim.run(0.5, |_| {});
    let mut scene = sim_core::Scene::default();
    sim.scene(&mut scene);
    let count = |id: &str| scene.parts.iter().filter(|p| p.id == id).count();
    assert_eq!((count("conveyor-belt"), count("conveyor-slat"), count("conveyor-part")), (1, 10, 1));
    let part = scene.parts.iter().find(|p| p.id == "conveyor-part").unwrap();
    // The delta's scene is its tool frame: the grip pose (-0.5 + 0.15, 0.25, -0.8), the 50 mm cube under it.
    let s = 0.3 * sim.time();
    assert!((part.pos[0] - (-0.5 + s)).abs() < 1e-9 && (part.pos[2] - -0.825).abs() < 1e-9, "{:?}", part.pos);
}

/// The conveyor cycle runs with the obstacles' live check like any other: a box around the place pose is hit by the
/// tool placing there, and the verdict "collision" fails.
#[test]
fn an_obstacle_at_the_place_fails_the_collision_check() {
    let cfg = example(&[
        r#"obstacles = [{ name = "at the place", centre_m = [0.0, -0.3, -0.8], size_m = [0.04, 0.04, 0.04] }]"#,
    ]);
    let mut sim = Sim::new(find("delta-d3").unwrap(), &cfg).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    let collision = verdict(&cfg, &load, "collision");
    assert!(!collision.ok && collision.value < 0.0, "{collision:?}");
}
