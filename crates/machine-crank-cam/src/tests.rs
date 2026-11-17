use super::*;
use crate::crank::SliderCrank;
use crate::knife::knife_cam;
use crate::saw::saw_cam;
use sim_components::analysis::{CycleLoad, Verdict};
use sim_components::cam::Law;
use sim_components::{Command, Sim};
use std::f64::consts::{PI, TAU};

const ENTRIES: [&MachineEntry; 7] =
    [&SLIDER_CRANK, &ROTARY_KNIFE, &FLYING_SAW, &DISC_CAM, &TOGGLE_PRESS, &GENEVA_DRIVE, &CAM_INDEXER];

#[test]
fn slider_crank_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&SLIDER_CRANK);
}
#[test]
fn rotary_knife_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ROTARY_KNIFE);
}
#[test]
fn flying_saw_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&FLYING_SAW);
}
#[test]
fn disc_cam_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&DISC_CAM);
}

/// The press of the shipped config without gravity, its process force 1 kN over the whole stroke (0 to 2 r = 100 mm from
/// the ram's end: at -90 degrees it is l + r - sqrt(l^2 - r^2) = 56.4 mm out).
fn press() -> SliderCrank {
    let mut cfg = SLIDER_CRANK.config();
    cfg.set_text("effects.gravity = false").unwrap();
    let mut crank = SliderCrank::from_config(&cfg).unwrap();
    crank.process = Some(Table { at: vec![0.0, 0.1], value: vec![1000.0, 1000.0] });
    crank
}

/// r = 50 mm, l = 200 mm, theta = 60 degrees: w = sqrt(0.04 - 0.0025 * 0.75) = sqrt(0.038125) = 0.1952562;
/// x = r cos + w = 0.025 + 0.1952562 = 0.2202562 m; x' = -r sin - r^2 sin cos / w = -0.0433013 - 0.00108253 / 0.1952562
/// = -0.0488454 m/rad; x'' = -r cos - r^2 cos(120 deg) / w - r^4 sin^2 cos^2 / w^3 = -0.025 + 0.0064017 - 0.0001573
/// = -0.0187556 m/rad^2. At the dead centres x = l + r and l - r, and x' = 0.
#[test]
fn the_slider_crank_kinematics_match_the_hand_worked_formula() {
    let crank = press();
    let [x, x1, x2] = crank.slider(60f64.to_radians());
    assert!((x - 0.2202562).abs() < 1e-7, "{x}");
    assert!((x1 + 0.0488454).abs() < 1e-7, "{x1}");
    assert!((x2 + 0.0187556).abs() < 1e-7, "{x2}");
    let (outer, inner) = (crank.slider(0.0), crank.slider(std::f64::consts::PI));
    assert!((outer[0] - 0.25).abs() < 1e-12 && (inner[0] - 0.15).abs() < 1e-12);
    assert!(outer[1].abs() < 1e-12 && inner[1].abs() < 1e-12);
    // The rates are the differences of the position.
    let h = 1e-6;
    let (a, b) = (crank.slider(1.0 - h), crank.slider(1.0 + h));
    assert!(((b[0] - a[0]) / (2.0 * h) - crank.slider(1.0)[1]).abs() < 1e-8);
    assert!(((b[1] - a[1]) / (2.0 * h) - crank.slider(1.0)[2]).abs() < 1e-8);
}

/// At theta = 60 degrees: J = J_crank + m_pin r^2 + m_x x'^2 = 0.02 + 1.5 * 0.0025 + 26.5 * 0.0488454^2 = 0.0869757 kg m^2
/// (m_x: ram 15 kg, die 10 kg, half the rod's 3 kg). Turning back at 2 rad/s (the ram moves out at 0.0977 m/s) and
/// speeding up by 10 rad/s^2 against 1 kN of press force: tau = J theta'' + m_x x' x'' theta'^2 + F x'
/// = 0.869757 + 26.5 * 0.0488454 * 0.0187556 * 4 - 48.8454 = 0.869757 + 0.097109 - 48.845430 = -47.878563 N m at the crank.
#[test]
fn the_reflected_inertia_and_torque_at_a_crank_angle_match_the_hand_calculation() {
    let crank = press();
    let theta = 60f64.to_radians();
    assert!((crank.inertia(theta) - 0.0869757).abs() < 1e-7, "{}", crank.inertia(theta));
    let (mut mass, mut bias, mut process) = ([0.0], [0.0], [0.0]);
    crank.dynamics(&[theta], &[-2.0], &mut mass, &mut bias);
    crank.process_torque(&[theta], &[-2.0], &mut process);
    let tau = mass[0] * 10.0 + bias[0] - process[0];
    assert!((tau + 47.878563).abs() < 1e-5, "{tau}");
    // Moving inwards the press force does not act.
    crank.process_torque(&[theta], &[2.0], &mut process);
    assert!((process[0] - 48.845430).abs() < 1e-5, "{process:?}");
}

/// The loads at rest at 90 degrees without gravity and process: no rod force, guide and bearing free. With 1 kN on the
/// ram moving out (at -90 degrees x' = r: theta' = 0.1 rad/s > 0): the rod carries F / cos(phi), cos(phi) = w / l = sqrt(0.04 - 0.0025) / 0.2
/// = 0.9682458, so -1032.796 N (pushed); the guide takes T sin(phi) = 1032.796 * 0.25 = 258.199 N; the bearing the rod's
/// whole force less the pin's centripetal pull at 0.1 rad/s (1.5 kg * 0.05 m * 0.01 = 7.5e-4 N, here along the rod's
/// normal): about 1032.796 N.
#[test]
fn rod_guide_and_bearing_loads_follow_the_rod_angle() {
    let crank = press();
    let mut quiet = press();
    quiet.process = None;
    assert!(quiet.loads(std::f64::consts::FRAC_PI_2, 0.0, 0.0).iter().all(|f| f.abs() < 1e-12));
    let [rod, guide, bearing] = crank.loads(-std::f64::consts::FRAC_PI_2, 0.1, 0.0);
    assert!((rod + 1032.796).abs() < 1e-2, "{rod}");
    assert!((guide - 258.199).abs() < 1e-2, "{guide}");
    assert!((bearing - 1032.796).abs() < 1e-2, "{bearing}");
}

/// R = 0.1 m, one blade, +-10 degrees synchronous: over the rest of a 0.5 m format the roll turns 2 pi - 0.349066 =
/// 5.934119 rad while the web runs 0.5 - 0.034907 = 0.465093 m: 12.758984 rad/m on average against 1 / R = 10 in the zone,
/// and with 3-4-5 at mid-span 10 + 1.875 * 2.758984 = 15.173095 rad/m: at 1 m/s of web 15.17 rad/s, the blade's tip at
/// 1.517 m/s. A format of 0.8 m (longer than the circumference) slows it: 5.934119 / 0.765093 = 7.756072 rad/m.
#[test]
fn the_rotary_knife_speeds_up_between_cuts_for_a_short_format_and_slows_for_a_long_one() {
    let sync = 10f64.to_radians();
    let short = knife_cam(0.1, 1.0, sync, 0.5, Law::Poly345).unwrap();
    assert!((short.period - 0.5).abs() < 1e-12 && (short.advance - std::f64::consts::TAU).abs() < 1e-12);
    // In the zone the tip runs with the web.
    for m in [0.0, 0.01, 0.49] {
        assert!((short.at(m)[1] - 10.0).abs() < 1e-9, "{m}");
    }
    let rest = (std::f64::consts::TAU - 2.0 * sync) / (0.5 - 0.2 * sync);
    assert!((rest - 12.758984).abs() < 1e-6);
    assert!((short.at(0.25)[1] - 15.173095).abs() < 1e-6, "{:?}", short.at(0.25));
    let long = knife_cam(0.1, 1.0, sync, 0.8, Law::Poly345).unwrap();
    let mean = (long.at(0.8 - 0.1 * sync)[0] - long.at(0.1 * sync)[0]) / (0.8 - 0.2 * sync);
    assert!((mean - 7.756072).abs() < 1e-6 && long.at(0.4)[1] < 10.0, "{mean}");
    // A format no longer than the zone has no room between cuts.
    assert!(knife_cam(0.1, 1.0, sync, 0.03, Law::Poly345).is_none());
}

/// Material at v = 1 m/s, mean synchronising acceleration a = 5 m/s^2: the material runs v^2 / a = 0.2 m while the
/// carriage comes up to speed over v^2 / (2 a) = 0.1 m, at a peak acceleration of 1.5 a = 7.5 m/s^2. Then it runs with the
/// material through the 0.2 m window, 0.2 s, from 0.1 m to 0.3 m, leaves it over 0.1 m more (stroke 0.4 m) and returns
/// over the rest of the 1.5 m cut length, 1.5 - 0.4 - 0.2 = 0.9 m of material, 0.9 s: with 3-4-5 at a peak of
/// 1.875 * 0.4 / 0.9 = 0.8333 m/s.
#[test]
fn the_flying_saw_synchronises_over_v_squared_over_2a_and_cuts_within_its_window() {
    let cam = saw_cam(1.0, 5.0, 0.2, 1.5, Law::Poly345).unwrap();
    let synced = cam.at(0.2);
    assert!((synced[0] - 0.1).abs() < 1e-12 && (synced[1] - 1.0).abs() < 1e-12, "{synced:?}");
    for m in [0.2, 0.3, 0.4 - 1e-12] {
        assert!((cam.at(m)[1] - 1.0).abs() < 1e-9 && cam.at(m)[2].abs() < 1e-9, "{m}");
    }
    assert!((cam.at(0.4 - 1e-12)[0] - 0.3).abs() < 1e-9);
    assert!((cam.at(0.6)[0] - 0.4).abs() < 1e-12 && cam.at(0.6)[1].abs() < 1e-12);
    assert!((cam.at(0.1)[2] - 7.5).abs() < 1e-9, "{:?}", cam.at(0.1));
    assert!((cam.at(1.05)[1] + 1.875 * 0.4 / 0.9).abs() < 1e-9);
    assert!(cam.at(1.5 - 1e-12)[0].abs() < 1e-9 && cam.advance.abs() < 1e-12);
    // A cut length that leaves no time to return is refused.
    assert!(saw_cam(1.0, 5.0, 0.2, 0.6, Law::Poly345).is_none());
}

/// Runs a machine's cycle and judges it.
fn judged(entry: &MachineEntry, cfg: &Config) -> (CycleLoad, Vec<Verdict>) {
    let mut sim = Sim::new(entry, cfg).unwrap();
    assert!(sim.limit_violations.is_empty(), "{}: {:?}", entry.id, sim.limit_violations);
    let load = sim_components::analysis::analyse_cycle(&mut sim).unwrap();
    let verdicts = sim_components::analysis::verdicts(cfg, &load).unwrap();
    (load, verdicts)
}

fn with_cam(entry: &MachineEntry) -> Config {
    let mut cfg = entry.config();
    cfg.set_text("cam.run = true").unwrap();
    cfg
}

/// The cam-running example beside the machine's config (`config/<id>-cam.toml`), loaded as a front end loads a file.
fn cam_example(entry: &MachineEntry) -> Config {
    let path = format!("{}/config/{}-cam.toml", env!("CARGO_MANIFEST_DIR"), entry.id);
    sim_components::load_config(entry, &std::fs::read_to_string(&path).unwrap()).unwrap()
}

/// Each cam-running example is the shipped config with `cam.run = true` and nothing else changed, so it keeps the keys
/// the shipped one gains; it passes every verdict (`every_default_cycle_and_every_cam_passes_its_verdicts`).
#[test]
fn every_cam_example_is_the_shipped_config_running_its_cam() {
    for entry in ENTRIES {
        assert_eq!(cam_example(entry), with_cam(entry), "{}", entry.id);
    }
}

/// Each machine's default config and its cam example (`config/<id>-cam.toml`) pass every verdict, the cam's limits
/// ("cam-speed", "cam-acceleration", "cam-jerk") among them: the defaults are a sized example.
#[test]
fn every_default_cycle_and_every_cam_passes_its_verdicts() {
    for entry in ENTRIES {
        for cfg in [entry.config(), cam_example(entry)] {
            let (load, verdicts) = judged(entry, &cfg);
            assert!(!load.tripped, "{}", entry.id);
            for v in verdicts {
                assert!(v.ok, "{}: {} of motor {:?} is {} against {}", entry.id, v.check, v.motor, v.value, v.limit);
            }
        }
    }
}

/// The cam as the cycle: one pass is one master period at speed (the knife's 0.5 s of web, the saw's 1.5 s, a press
/// stroke of 2 s); a stop runs the master down over its ramp and ends the cycle, the slave at rest.
#[test]
fn a_cam_cycle_runs_one_master_period_per_pass_and_stops_by_its_ramp() {
    for (entry, period) in [(&SLIDER_CRANK, 2.0), (&ROTARY_KNIFE, 0.5), (&FLYING_SAW, 1.5)] {
        let mut sim = Sim::new(entry, &with_cam(entry)).unwrap();
        sim.command(Command::StartCycle).unwrap();
        let mut starts = Vec::new();
        let mut last = sim.control.cycle_passes();
        while starts.len() < 3 {
            sim.step();
            if sim.control.cycle_passes() != last {
                (last, starts) = (sim.control.cycle_passes(), [starts, vec![sim.time()]].concat());
            }
            assert!(sim.time() < 20.0, "{}", entry.id);
        }
        assert!((starts[2] - starts[1] - period).abs() < 2e-3, "{}: {starts:?}", entry.id);
        sim.command(Command::Stop).unwrap();
        let stopped = sim.time();
        while sim.control.busy() {
            sim.step();
            assert!(sim.time() < stopped + 1.0, "{}", entry.id);
        }
        // The ramp of 0.5 s, the profile cycle that picks the stop up and the one that ends the cycle (to rounding: the
        // knife ends at 0.502 s, 8032 base steps).
        assert!(sim.time() - stopped < 0.5 + 2e-3 + 1e-9, "{}: {}", entry.id, sim.time() - stopped);
        assert!(sim.control.set_vel.iter().all(|v| *v == 0.0) && sim.control.cycle_segment().is_none());
    }
}

/// A cam that asks more of the slave than its axis allows is reported: the knife's 3-4-5 at 1 m/s of web peaks at
/// 15.17 rad/s, over a limit of 10.
#[test]
fn a_cam_beyond_the_axis_limits_is_reported() {
    let mut cfg = with_cam(&ROTARY_KNIFE);
    cfg.set_text("axis.knife.max_velocity_rad_s = 10.0").unwrap();
    let sim = Sim::new(&ROTARY_KNIFE, &cfg).unwrap();
    let speed = sim.limit_violations.iter().find(|v| v.key == "limit-cam-speed").expect("a speed violation");
    assert!((speed.needed - 15.173095).abs() < 1e-3 && speed.allowed == 10.0, "{speed:?}");
}

/// A new web speed while the knife's cam runs: the master ramps from 1 m/s to 0.8 m/s over the cam's 0.5 s ramp by the
/// sin^2 ramp, (1 + 0.8) / 2 = 0.9 m/s at mid-ramp, 0.8 m/s at its end, having gone (1 + 0.8) / 2 * 0.5 = 0.45 m; and
/// at no profile cycle a step in its speed (the ramp's highest acceleration is 0.2 pi / (2 * 0.5) = 0.628 m/s2, 0.6 mm/s
/// per millisecond).
#[test]
fn a_new_master_speed_under_a_running_cam_ramps_to_it() {
    let mut sim = Sim::new(&ROTARY_KNIFE, &with_cam(&ROTARY_KNIFE)).unwrap();
    sim.command(Command::StartCycle).unwrap();
    sim.run(1.5, |_| {});
    let cycle = with_cam(&ROTARY_KNIFE).f64("profile.cycle_s").unwrap();
    let speed = |sim: &mut Sim| {
        let before = sim.control.cam_master();
        let steps = (cycle / sim.dt).round() as usize;
        (0..steps).for_each(|_| sim.step());
        (sim.control.cam_master() - before) / cycle
    };
    assert!((speed(&mut sim) - 1.0).abs() < 1e-9);
    let mut slower = with_cam(&ROTARY_KNIFE);
    slower.set_text("web.speed_m_s = 0.8").unwrap();
    sim.reconfigure(&ROTARY_KNIFE, &slower).unwrap();
    let from = sim.control.cam_master();
    let (mut last, mut speeds) = (1.0, Vec::new());
    for _ in 0..(0.5 / cycle).round() as usize {
        let v = speed(&mut sim);
        assert!((v - last).abs() < 0.628 * cycle * 1.01, "a step in the master's speed: {last} to {v}");
        (last, speeds) = (v, [speeds, vec![v]].concat());
    }
    // Each speed is the mean over its cycle: the one that ends at mid-ramp is half a cycle short of it.
    let mid = speeds[speeds.len() / 2 - 1];
    assert!((mid - 0.9).abs() < 0.628 * cycle, "{mid}");
    assert!((last - 0.8).abs() < 1e-5 && (speed(&mut sim) - 0.8).abs() < 1e-9, "{last}");
    assert!((sim.control.cam_master() - cycle * 0.8 - from - 0.45).abs() < 1e-9);
}

/// The cam taken away while it runs (a config with cam.run = false): the flying saw's carriage, caught at its top speed
/// along with the material, ramps down from its speed then, v0, at its acceleration limit a, at rest after v0 / a, its
/// speed falling by a T every profile cycle T from the first on, with no step and no cycle standing still. (A rotary
/// knife's roll, which the cam turns on past its range, meets the stops the new config gives its joint.)
#[test]
fn a_cam_taken_away_under_way_ramps_the_slave_down_within_its_limit() {
    let mut sim = Sim::new(&FLYING_SAW, &with_cam(&FLYING_SAW)).unwrap();
    sim.command(Command::StartCycle).unwrap();
    sim.run(1.0, |_| {});
    let cycle = with_cam(&FLYING_SAW).f64("profile.cycle_s").unwrap();
    let steps = (cycle / sim.dt).round() as usize;
    let a = sim.machine.axes()[0].max_acc;
    let top = sim.machine.axes()[0].max_vel;
    while sim.control.set_vel[0].abs() < 0.5 * top || sim.control.set_acc[0] != 0.0 {
        sim.step();
        assert!(sim.time() < 10.0, "the carriage runs with the material");
    }
    let mut off = with_cam(&FLYING_SAW);
    off.set_text("cam.run = false").unwrap();
    let v0 = sim.control.set_vel[0];
    sim.reconfigure(&FLYING_SAW, &off).unwrap();
    let (mut last, mut cycles) = (v0, 0);
    while sim.control.set_vel[0] != 0.0 || cycles == 0 {
        (0..steps).for_each(|_| sim.step());
        let v = sim.control.set_vel[0];
        // The first cycle may fall short of a whole a T: the profile tick lies somewhere in the cycle.
        assert!((last - v).abs() <= a * cycle * (1.0 + 1e-9), "cycle {cycles}: {last} to {v} rad/s");
        assert!(
            cycles == 0 || v == 0.0 || (last - v).abs() >= a * cycle * (1.0 - 1e-9),
            "cycle {cycles}: {last} to {v}"
        );
        (last, cycles) = (v, cycles + 1);
        assert!(cycles < 10_000);
    }
    assert!((cycles as f64 * cycle - v0.abs() / a).abs() <= 2.0 * cycle, "{cycles} cycles for {} s", v0.abs() / a);
    (0..10 * steps).for_each(|_| sim.step());
    assert_eq!(sim.control.set_vel[0], 0.0);
}

/// The cam's limits are verdicts too: the knife's 3-4-5 at 1 m/s of web asks 15.17 rad/s of the roll (by hand in
/// `the_rotary_knife_speeds_up_between_cuts_for_a_short_format_and_slows_for_a_long_one`); with the axis's speed limit at
/// 10 rad/s "cam-speed" fails with 15.17 against 10, the acceleration and jerk pass. As shipped (20 rad/s) it passes.
#[test]
fn a_cam_beyond_the_axis_limits_fails_its_verdict() {
    let judge = |limit: f64| {
        let mut cfg = with_cam(&ROTARY_KNIFE);
        cfg.set_text(&format!("axis.knife.max_velocity_rad_s = {limit}")).unwrap();
        let mut sim = Sim::new(&ROTARY_KNIFE, &cfg).unwrap();
        let load = sim_components::analysis::analyse_cycle(&mut sim).unwrap();
        sim_components::analysis::verdicts(&cfg, &load).unwrap()
    };
    let slow = judge(10.0);
    let speed = slow.iter().find(|v| v.check == "cam-speed").expect("a cam-speed verdict");
    assert!(!speed.ok && (speed.value - 15.173095).abs() < 1e-3 && speed.limit == 10.0, "{speed:?}");
    assert_eq!(speed.motor, Some(0));
    assert!(slow.iter().filter(|v| v.check.starts_with("cam-")).count() == 3);
    assert!(slow.iter().filter(|v| v.check == "cam-acceleration" || v.check == "cam-jerk").all(|v| v.ok));
    let shipped = ROTARY_KNIFE.config().f64("axis.knife.max_velocity_rad_s").unwrap();
    assert!(judge(shipped).iter().filter(|v| v.check.starts_with("cam-")).all(|v| v.ok), "{shipped}");
}

/// The shipped disc cam at mid-rise by hand: base circle 60 mm, roller 15 mm, offset 10 mm: s0 = sqrt(0.075^2 - 0.01^2)
/// = 0.0743303 m. A 3-4-5 rise of h = 30 mm over beta = 120 degrees is at theta = 60 degrees halfway up, s = 15 mm, at
/// its steepest, s' = 1.875 h / beta = 0.0268574 m/rad, and without acceleration (s'' = 0). The pressure angle:
/// tan(phi) = (0.0268574 - 0.01) / (0.0743303 + 0.015) = 0.188709, phi = 0.186515 rad = 10.687 degrees. At a steady
/// 2 pi rad/s the follower (2 kg and the 1 kg tool) is pushed down by the spring's 100 + 2000 * 0.015 = 130 N and its
/// weight 29.42 N, 159.420 N: the contact force 159.420 / cos(phi) = 162.234 N, the guide's load across
/// 159.420 * 0.188709 = 30.084 N, and the disc's torque 159.420 * s' = 4.28161 N m (no speed term where s'' = 0).
#[test]
fn the_disc_cam_s_pressure_angle_and_contact_force_at_mid_rise_match_the_hand_calculation() {
    let disc = DiscCam::from_config(&DISC_CAM.config()).unwrap();
    assert!((disc.s0() - 0.0743303).abs() < 1e-7, "{}", disc.s0());
    let theta = std::f64::consts::PI / 3.0;
    let [s, s1, s2, _] = disc.lift.at(theta);
    // (The segments' spans are given to 8 digits: halfway up is 3e-11 rad from pi / 3.)
    assert!((s - 0.015).abs() < 1e-9 && (s1 - 0.0268574).abs() < 1e-7 && s2.abs() < 1e-6, "{s} {s1} {s2}");
    let phi = disc.pressure_angle(theta);
    assert!((phi - 0.186515).abs() < 1e-6 && (phi.to_degrees() - 10.687).abs() < 1e-3, "{phi}");
    let [contact, guide] = disc.loads(theta, std::f64::consts::TAU, 0.0);
    assert!((contact - 162.234).abs() < 1e-3 && (guide - 30.084).abs() < 1e-3, "{contact} {guide}");
    let (mut mass, mut bias) = ([0.0], [0.0]);
    disc.dynamics(&[theta], &[std::f64::consts::TAU], &mut mass, &mut bias);
    assert!((bias[0] - 4.28161).abs() < 1e-5, "{bias:?}");
    assert!((mass[0] - (0.0101 + 3.0 * s1 * s1)).abs() < 1e-12);
    // On the base circle the follower stands: no pressure angle but what the offset leaves, atan(-e / s0).
    assert!((disc.pressure_angle(5.5) + (0.01f64 / 0.0743303).atan()).abs() < 1e-6);
}

/// The flying saw's friction characteristic (drive.friction_feedforward_*) is its friction at the motor, by hand: pinion
/// radius r = m z / (2 cos(beta)) = 0.002 * 30 / (2 cos(19.5283 deg)) = 0.0318310 m (200 mm a turn), ratio 5, rack and
/// gearbox 0.97 each (eta = 0.9409, the motor drives). Coulomb: 80 N * r / (5 eta) + the motor's 0.05 N m =
/// 0.541287 + 0.05 = 0.591287 N m, from the guides' smoothing speed 2 mm/s = 0.002 * 5 / r = 0.314159 rad/s on.
/// Viscous: 100 N s/m * r^2 / (25 eta) + the motor's 2e-4 = 0.00450743 N m s/rad, so at 1.2 m/s (188.495 rad/s)
/// 0.591287 + 0.849649 = 1.440936 N m.
#[test]
fn the_flying_saw_s_friction_characteristic_is_its_friction_at_the_motor() {
    let cfg = FLYING_SAW.config();
    assert!(cfg.bool("effects.friction_feedforward").unwrap());
    let speed = cfg.f64s("drive.friction_feedforward_speed_rad_s").unwrap();
    let torque = cfg.f64s("drive.friction_feedforward_torque_nm").unwrap();
    assert_eq!(speed.len(), 2);
    assert!((speed[0] - 0.314159).abs() < 1e-3 && (speed[1] - 188.495).abs() < 0.01, "{speed:?}");
    assert!((torque[0] - 0.591287).abs() < 1e-3 && (torque[1] - 1.440936).abs() < 1e-3, "{torque:?}");
    // The hand calculation's inputs are the config's.
    let get = |k: &str| cfg.f64(k).unwrap();
    assert_eq!(
        [get("slide.friction_coulomb_n"), get("slide.friction_viscous_n_s_m"), get("slide.friction_smoothing_m_s")],
        [80.0, 100.0, 0.002]
    );
    assert_eq!([get("rack.module_m"), get("rack.teeth"), get("rack.helix_angle_deg")], [0.002, 30.0, 19.5283]);
    assert_eq!([get("gearbox.ratio"), get("gearbox.efficiency"), get("rack.efficiency")], [5.0, 0.97, 0.97]);
    assert_eq!([get("motor.friction_coulomb_nm"), get("motor.friction_viscous_nm_s_rad")], [0.05, 2.0e-4]);
    assert_eq!(get("axis.x.max_velocity_m_s"), 1.2);
}

/// Friction feedforward from the machine's own model, by hand, as the characteristic above
/// (`the_flying_saw_s_friction_characteristic_is_its_friction_at_the_motor`, 1.440936 N m to its rounding): at 1.2 m/s
/// the guides slide at 80 N + 100 N s/m x 1.2 m/s = 200 N (tanh(1.2 / 0.002) = 1), through r / (i eta) = 0.0318310 /
/// (5 x 0.9409): 1.353218 N m (the gearbox's power 240 W far beyond its fade, so eta whole); the shaft at 5 x 1.2 /
/// 0.0318310 = 188.4956 rad/s: 0.05 tanh(377) + 2e-4 x 188.4956 = 0.0876991 N m. So 1.440917 N m, and backwards its
/// negative. In the cruise at 1.2 m/s of the move 0.1 -> 0.7 m (the cut off, so that only friction is left) the drive
/// gets it, and the velocity loop's integrator carries nothing (under 2 % of it); without it, it carries the whole
/// friction.
#[test]
fn the_flying_saw_s_machine_friction_feedforward_is_its_characteristic_and_the_integrator_carries_nothing() {
    let expected = 1.440917;
    let saw = (FLYING_SAW.build)(&FLYING_SAW.config()).unwrap();
    let mut out = [0.0];
    for v in [1.2, -1.2] {
        saw.steady_friction(&[0.4], &[v], &mut out);
        assert!((out[0] - expected * v.signum()).abs() < 1e-5, "at {v} m/s: {} N m, by hand {expected}", out[0]);
    }
    let cruise = |model: bool| {
        let mut cfg = FLYING_SAW.config();
        cfg.set_text("effects.process_force = false").unwrap();
        cfg.set_text("drive.friction_feedforward_torque_nm = [0.0, 0.0]").unwrap();
        let source = if model { "machine" } else { "characteristic" };
        cfg.set_text(&format!("drive.friction_feedforward_model = \"{source}\"")).unwrap();
        let mut sim = Sim::new(&FLYING_SAW, &cfg).unwrap();
        sim.command(Command::MoveTo(vec![0.7])).unwrap();
        let (mut ff, mut integrator, mut n) = (0.0, 0.0, 0);
        sim.run(0.6, |s| {
            if s.time() > 0.3 && s.control.set_acc[0] == 0.0 && (s.control.set_vel[0] - 1.2).abs() < 1e-12 {
                (ff, integrator, n) =
                    (s.control.drives[0].friction_ff, integrator + s.control.drives[0].controller_torque, n + 1);
            }
        });
        assert!(n > 1000, "{n} samples in the cruise");
        (ff, integrator / n as f64)
    };
    let (ff, integrator) = cruise(true);
    assert!((ff - expected).abs() < 1e-4 * expected, "feedforward {ff} N m, by hand {expected}");
    assert!(integrator.abs() < 0.02 * expected, "the integrator carries {integrator} N m");
    let (ff, integrator) = cruise(false);
    assert!(ff == 0.0 && (integrator - expected).abs() < 0.02 * expected, "without: {integrator} N m");
}

/// A joint that turns on (here the knife's roll) has no stops of its own: the cam turns it on past the axis range
/// (-30 to 400 degrees), and when the cam is taken away under way the roll ramps down from its speed to rest where it
/// is, beyond that range. Its speed never rises above the speed it had (with the config's stops it met its upper stop
/// 3.4 turns out at 1e6 N m/rad and was flung back at 38000 rad/s).
#[test]
fn a_knife_roll_whose_cam_is_taken_away_ramps_down_beyond_its_axis_range_without_stops() {
    let mut sim = Sim::new(&ROTARY_KNIFE, &with_cam(&ROTARY_KNIFE)).unwrap();
    sim.command(Command::StartCycle).unwrap();
    sim.run(2.0, |_| {});
    let (from, v0) = (sim.state[0], sim.state[1]);
    assert!(from > 400f64.to_radians() + TAU && v0 > 10.0, "{from} rad at {v0} rad/s");
    let mut off = with_cam(&ROTARY_KNIFE);
    off.set_text("cam.run = false").unwrap();
    sim.reconfigure(&ROTARY_KNIFE, &off).unwrap();
    let mut fastest = 0.0f64;
    sim.run(3.0, |s| fastest = fastest.max(s.state[1].abs()));
    assert!(fastest < v0 * 1.01, "{fastest} rad/s against {v0}");
    assert!(sim.state[1].abs() < 1e-3 && sim.state[0] > from, "at rest at {} rad", sim.state[0]);
}

/// End stops where a config file puts them (`joint.stop_min_deg`, `joint.stop_max_deg`, either alone; the schema a file
/// loads against has them), none where it does not (the knife above); a lower stop not below the upper is refused. With
/// the upper stop at 720 degrees the running cam brings the disc up against it: it stays there but for the stop's give
/// (the drive's peak torque, 150 N m at the disc, over 1e6 N m/rad is 0.15 mrad; the run, with the speed it comes in
/// at, 0.42 mrad).
#[test]
fn a_joint_that_turns_on_has_end_stops_only_where_the_config_puts_them() {
    let config = |extra: &str| {
        let text = with_cam(&DISC_CAM).to_text().unwrap().replacen("[joint]", &format!("[joint]\n{extra}"), 1);
        sim_components::load_config(&DISC_CAM, &text)
    };
    let builds = |extra: &str| config(extra).and_then(|cfg| (DISC_CAM.build)(&cfg).map(|_| ())).is_ok();
    assert!(builds("stop_max_deg = 720.0"));
    assert!(builds("stop_min_deg = -90.0"));
    assert!(builds("stop_min_deg = -90.0\nstop_max_deg = 720.0"));
    assert!(!builds("stop_min_deg = 90.0\nstop_max_deg = 45.0"));
    let mut sim = Sim::new(&DISC_CAM, &config("stop_max_deg = 720.0").unwrap()).unwrap();
    sim.command(Command::StartCycle).unwrap();
    let mut furthest = 0.0f64;
    sim.run(4.0, |s| furthest = furthest.max(s.state[0]));
    assert!(furthest > 4.0 * PI && furthest < 4.0 * PI + 1e-3, "the disc went to {furthest} rad");
}
