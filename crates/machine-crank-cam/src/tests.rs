use super::*;
use crate::crank::SliderCrank;
use crate::knife::knife_cam;
use crate::saw::saw_cam;
use sim_components::analysis::{CycleLoad, Verdict};
use sim_components::cam::Law;
use sim_components::{Command, Sim};

const ENTRIES: [&MachineEntry; 3] = [&SLIDER_CRANK, &ROTARY_KNIFE, &FLYING_SAW];

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

/// The press of the shipped config without gravity, its process force 1 kN wherever the table reaches (0 to 50 mm).
fn press() -> SliderCrank {
    let mut cfg = SLIDER_CRANK.config();
    cfg.set_text("effects.gravity = false").unwrap();
    let mut crank = SliderCrank::from_config(&cfg).unwrap();
    crank.process = Some(Table { at: vec![0.0, 0.05], value: vec![1000.0, 1000.0] });
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

/// Each machine's default config and its cam pass every verdict: the defaults are a sized example.
#[test]
fn every_default_cycle_and_every_cam_passes_its_verdicts() {
    for entry in ENTRIES {
        for cfg in [entry.config(), with_cam(entry)] {
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
        // The ramp of 0.5 s, and a profile cycle.
        assert!(sim.time() - stopped < 0.5 + 2e-3, "{}: {}", entry.id, sim.time() - stopped);
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
