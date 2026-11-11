//! Machine-specific checks, and the component checks that the plan runs once on this machine:
//! friction levels, backlash, belt natural frequency.

use super::*;
use sim_core::testkit::{DT, PulseRun};
use sim_components::Sim;
use sim_components::analysis::{analyse_cycle, bandwidth, closed_loop_response, frequency_sweep, log_spaced, step_response};
use sim_core::{Rk4, advance};

fn machine(mut cfg: Config, edit: &[&str]) -> SingleAxis {
    for e in edit {
        cfg.set_text(e).unwrap();
    }
    SingleAxis::new(&cfg).unwrap()
}

fn rigid(edit: &[&str]) -> SingleAxis {
    let mut all = vec!["effects.belt_stretch = false"];
    all.extend(edit);
    machine(lossless_config(), &all)
}

/// Runs from `state` with a torque that may depend on time; `each` sees (t, state, torque).
fn simulate(m: &SingleAxis, state: &mut [f64], seconds: f64, torque: impl Fn(f64) -> f64, mut each: impl FnMut(f64, &[f64], f64)) {
    let mut rk = Rk4::new(state.len());
    for step in 0..(seconds / DT).round() as usize {
        let t = step as f64 * DT;
        advance(m, &mut rk, state, &[torque(t)], DT, 4);
        each(t + DT, state, torque(t));
    }
}

fn signal(m: &SingleAxis, state: &[f64], torque: f64, id: &str) -> f64 {
    let mut values = Vec::new();
    m.signals(state, &[torque], &mut values);
    values[m.signal_infos().iter().position(|s| s.id == id).unwrap()]
}

#[test]
fn constant_torque_gives_force_over_effective_mass() {
    let m = rigid(&["effects.belt_mass = false"]);
    let mut state = vec![0.1, 0.0, 0.0];
    let t_end = 0.25;
    simulate(&m, &mut state, t_end, |_| 0.2, |_, _, _| {});
    let a = (0.2 * 5.0 / 0.02) / (6.0 + (1.0e-4 + 25.0 * 0.3e-4) / (0.02 * 0.02));
    assert!((state[0] - 0.1 - 0.5 * a * t_end * t_end).abs() < 1e-9, "x {}", state[0]);
    assert!((state[1] - a * t_end).abs() < 1e-9);
}

#[test]
fn inclined_axis_needs_holding_torque() {
    let expected = 6.0 * G * 0.02 / 5.0;
    let (mut state, mut hold) = (Vec::new(), [0.0]);
    rigid(&["geometry.incline_deg = 90.0"]).equilibrium(&[0.5], &mut state, &mut hold);
    assert!((hold[0] - expected).abs() < 1e-12);
    // The elastic model agrees to within the belt strain (see Belt::section).
    machine(lossless_config(), &["geometry.incline_deg = 90.0"]).equilibrium(&[0.5], &mut state, &mut hold);
    assert!((hold[0] - expected).abs() < 1e-3 * expected, "{}", hold[0]);
}

#[test]
fn belt_mass_adds_to_the_pulley_inertia() {
    let (with, without) = (rigid(&[]), rigid(&["effects.belt_mass = false"]));
    // 0.1 kg/m on 0.15 + 0.45 + 2.0 m of belt, all moving at carriage speed.
    assert!((with.m_total() - without.m_total() - 0.1 * 2.6).abs() < 1e-12);
}

/// Friction levels: breakaway force equals the stiction level.
#[test]
fn breakaway_force_equals_the_stiction_level() {
    let m = rigid(&["effects.guide_friction = true"]);
    let mut state = vec![0.5, 0.0, 0.0];
    // Push with a force that rises by 5 N/s and watch the friction force until the carriage slides.
    let force_to_torque = 0.02 / 5.0;
    let mut peak = 0f64;
    simulate(&m, &mut state, 6.0, |t| 5.0 * t * force_to_torque, |_, s, torque| {
        if s[1] < m.friction.stribeck_v {
            peak = peak.max(signal(&m, s, torque, "axis.x.friction"));
        }
    });
    assert!(state[1] > 0.1, "the carriage never broke away");
    assert!((peak - 20.0).abs() < 0.02 * 20.0, "breakaway at {peak} N, stiction level is 20 N");
}

/// Friction levels: steady sliding force equals Coulomb plus viscous.
#[test]
fn steady_sliding_force_is_coulomb_plus_viscous() {
    let m = rigid(&["effects.guide_friction = true"]);
    for direction in [1.0, -1.0] {
        let mut state = vec![0.5, 0.0, 0.0];
        let torque = direction * 30.0 * 0.02 / 5.0;
        simulate(&m, &mut state, 6.0, |_| torque, |_, _, _| {});
        // 30 N = 15 N Coulomb + 20 N s/m * v  =>  v = 0.75 m/s.
        assert!((state[1] - direction * 0.75).abs() < 1e-3 * 0.75, "v {}", state[1]);
        let friction = signal(&m, &state, torque, "axis.x.friction");
        assert!((friction - direction * (15.0 + 20.0 * 0.75)).abs() < 1e-3 * 30.0, "friction {friction}");
    }
}

#[test]
fn friction_grows_with_payload_when_load_dependent() {
    let edit = ["effects.guide_friction = true", "effects.load_dependent_friction = true"];
    let mut m = machine(ENTRY.config(), &edit);
    // Levels are given for the empty carriage: preload 200 N plus 4 kg; the payload adds 2 kg.
    let expected = (200.0 + 6.0 * G) / (200.0 + 4.0 * G);
    assert!((m.load_scale() - expected).abs() < 1e-12);
    m.set_payload_attached(false);
    assert_eq!(m.load_scale(), 1.0);
}

/// Backlash: lost motion on reversal equals the configured backlash.
#[test]
fn lost_motion_on_reversal_equals_the_backlash() {
    let m = machine(
        lossless_config(),
        &[
            "effects.guide_friction = true",
            "effects.motor_friction = true",
            "effects.belt_damping = true",
            "gearbox.damping_nm_s_rad = 0.5",
        ],
    );
    let (mut state, mut hold) = (Vec::new(), [0.0]);
    m.equilibrium(&[0.5], &mut state, &mut hold);
    // 0.04 N m at the motor is 10 N at the carriage: below stiction, so the load stays put.
    let twist = |torque: f64, state: &mut Vec<f64>| {
        simulate(&m, state, 1.5, |_| torque, |_, _, _| {});
        assert!(state[5].abs() < 1e-6, "motor still turning at {} rad/s", state[5]);
        state[2] / 5.0 - state[1]
    };
    let forward = twist(0.04, &mut state);
    let x_forward = state[0];
    let backward = twist(-0.04, &mut state);
    assert!((state[0] - x_forward).abs() < 1e-4, "the load moved by {}", state[0] - x_forward);
    // The gear also winds up elastically by torque / stiffness on each side.
    let lost = forward - backward - 2.0 * (0.04 * 5.0) / 1.0e4;
    assert!((lost - 8.7e-4).abs() < 0.01 * 8.7e-4, "lost motion {lost} rad, backlash 8.7e-4 rad");
}

/// Belt natural frequency matches the hand calculation from k and m within 5 %.
#[test]
fn belt_natural_frequency_matches_hand_calculation() {
    // A very stiff gearbox without backlash, so motor and pulley turn as one inertia.
    let m = machine(lossless_config(), &["gearbox.stiffness_nm_rad = 1.0e7", "effects.gearbox_backlash = false"]);
    let mut run = PulseRun::new(&m, DT, 0.1, 0.004);
    run.substeps = 16;
    run.run(0.004, |_| {});
    let (mut crossings, mut last) = (0, 0.0);
    let seconds = 0.5;
    run.run(seconds, |r| {
        // Carriage against pulley: the rate of the belt stretch.
        let rate = r.state[3] - m.r * r.state[4];
        if rate * last < 0.0 {
            crossings += 1;
        }
        last = rate;
    });
    let measured = crossings as f64 / 2.0 / seconds;

    // Hand calculation at x = 0.5 m: carriage and motor inertia joined by the two belt sections.
    let k = 5.0e5 * (1.0 / 0.65 + 1.0 / 1.95);
    let m_load = 6.0;
    let m_motor = (1.0e-4 + 0.1 * 2.6 * 0.02 * 0.02 + 25.0 * 0.3e-4) / (0.02 * 0.02);
    let expected = f64::sqrt(k * (1.0 / m_load + 1.0 / m_motor)) / std::f64::consts::TAU;
    assert!((measured - expected).abs() < 0.05 * expected, "measured {measured} Hz, hand calculation {expected} Hz");
}

/// Rigid limit: with very high belt stiffness the full model matches the rigid reference within 1 %.
#[test]
fn stiff_belt_converges_to_the_rigid_reference() {
    let travel = |edit: &[&str], substeps: u32| {
        let mut all = vec!["effects.gearbox_backlash = false"];
        all.extend(edit);
        let m = machine(ENTRY.config(), &all);
        let mut run = PulseRun::new(&m, DT, 1.0, 0.1);
        run.substeps = substeps;
        let start = run.tool_pos()[0];
        run.run(1.0, |_| {});
        run.tool_pos()[0] - start
    };
    let rigid = travel(&["effects.belt_stretch = false"], 4);
    let stiff = travel(&["belt.stiffness_ea_n = 5.0e7", "gearbox.stiffness_nm_rad = 1.0e6"], 16);
    let soft = travel(&[], 4);
    assert!(rigid > 0.02, "the reference move is too small: {rigid}");
    assert!((stiff - rigid).abs() < 0.01 * rigid, "stiff {stiff} m, rigid {rigid} m");
    // The default belt is visibly softer, otherwise the check above would prove nothing.
    assert!((soft - rigid).abs() > (stiff - rigid).abs(), "soft {soft}, stiff {stiff}, rigid {rigid}");
}

/// A rigid, loss-free config with an ideal drive: what a hand calculation can follow exactly.
fn ideal_rigid() -> Config {
    let mut cfg = lossless_config();
    for e in [
        "effects.belt_stretch = false",
        "effects.belt_mass = false",
        "effects.payload = false",
        "effects.encoder_resolution = false",
        "effects.velocity_filter = false",
        "effects.torque_filter = false",
    ] {
        cfg.set_text(e).unwrap();
    }
    cfg
}

/// Analysis: peak and RMS torque of a reference cycle match a hand calculation.
#[test]
fn reference_cycle_torque_matches_hand_calculation() {
    let mut sim = Sim::new(&ENTRY, &ideal_rigid()).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();

    // The cycle: 0.8 m there, 0.1 s dwell, 0.8 m back, 0.1 s dwell, at 1 m/s, 10 m/s^2, 100 m/s^3.
    let (v, a, j, dist, dwell) = (1.0, 10.0, 100.0, 0.8, 0.1);
    let tj: f64 = a / j;
    let ta = v / a - tj;
    let tv = (dist - v * (2.0 * tj + ta)) / v;
    let duration = 2.0 * (4.0 * tj + 2.0 * ta + tv + dwell);
    // Each of the four acceleration ramps: a rises over tj, holds for ta, falls over tj.
    let a_squared = 4.0 * a * a * (ta + 2.0 * tj / 3.0);
    // Torque per acceleration: carriage plus pulley and rotor inertia, through radius and gear ratio.
    let m_total = 4.0 + (1.0e-4 + 25.0 * 0.3e-4) / (0.02 * 0.02);
    let per_acc = m_total * 0.02 / 5.0;
    let (peak, rms) = (per_acc * a, per_acc * (a_squared / duration).sqrt());

    assert!((load.duration - duration).abs() < 2e-3, "cycle {} s, by hand {duration} s", load.duration);
    let m = &load.motors[0];
    assert!((m.peak_torque - peak).abs() < 0.02 * peak, "peak {} N m, by hand {peak} N m", m.peak_torque);
    assert!((m.rms_torque - rms).abs() < 0.02 * rms, "RMS {} N m, by hand {rms} N m", m.rms_torque);
    // Operating points: top speed is v through radius and ratio, and the torque-speed curve
    // (3.8 N m up to 314 rad/s) is used to peak / 3.8.
    assert!((m.peak_speed - v * 5.0 / 0.02).abs() < 0.01 * 250.0, "peak speed {}", m.peak_speed);
    assert!((m.utilisation - peak / 3.8).abs() < 0.02 * peak / 3.8, "utilisation {}", m.utilisation);
    assert!(m.points.len() as f64 > 0.99 * duration * 1000.0);
}

#[test]
fn a_cycle_beyond_the_torque_speed_curve_shows_as_utilisation_above_one() {
    let mut cfg = ideal_rigid();
    // A weak drive: 0.25 N m up to 100 rad/s, falling to nothing at 260 rad/s. The cycle needs
    // 0.245 N m at half speed (125 rad/s), where only 0.25 * 135 / 160 = 0.21 N m is left, and is
    // still accelerating close to its top speed of 250 rad/s, where almost nothing is left.
    cfg.set_text("drive.rated_speed_rad_s = 100.0").unwrap();
    cfg.set_text("drive.no_load_speed_rad_s = 260.0").unwrap();
    cfg.set_text("drive.peak_torque_nm = 0.25").unwrap();
    cfg.set_text("effects.torque_limit = false").unwrap();
    let load = analyse_cycle(&mut Sim::new(&ENTRY, &cfg).unwrap()).unwrap();
    let utilisation = load.motors[0].utilisation;
    assert!(utilisation > 0.245 / 0.21, "{utilisation}");
}

/// Frequency sweep: a rigid machine is a pure inertia, gain 1 / (J w) at -90 degrees.
#[test]
fn frequency_sweep_of_the_rigid_machine_is_a_pure_inertia() {
    let j_motor_side = (6.0 * 0.02 * 0.02 + 1.0e-4) / 25.0 + 0.3e-4;
    let mut cfg = ideal_rigid();
    cfg.set_text("effects.payload = true").unwrap();
    for p in frequency_sweep(&ENTRY, &cfg, 0, &[5.0, 20.0, 80.0, 320.0], 0.05).unwrap() {
        let expected = 1.0 / (j_motor_side * std::f64::consts::TAU * p.frequency);
        assert!((p.gain - expected).abs() < 0.01 * expected, "{} Hz: gain {}, by hand {expected}", p.frequency, p.gain);
        assert!((p.phase_deg + 90.0).abs() < 1.0, "{} Hz: phase {}", p.frequency, p.phase_deg);
    }
}

/// Frequency sweep: with a stretching belt the load swings against the motor. Where the load
/// resonates on the belt with the motor held, the motor hardly moves (a dip in the response).
#[test]
fn frequency_sweep_shows_the_belt_resonance() {
    // A very stiff gearbox without backlash, so that the belt is the only spring (as in the
    // natural-frequency check); that stiffness needs a finer integration step.
    let mut cfg = lossless_config();
    for e in ["gearbox.stiffness_nm_rad = 1.0e7", "effects.gearbox_backlash = false", "sim.substeps = 16"] {
        cfg.set_text(e).unwrap();
    }
    // By hand, at the start pose x = 0.1 m: belt sections of 0.25 m and 2.35 m.
    let k = 5.0e5 * (1.0 / 0.25 + 1.0 / 2.35);
    let dip = f64::sqrt(k / 6.0) / std::f64::consts::TAU;
    let frequencies: Vec<f64> = (0..41).map(|i| dip * (0.8 + 0.01 * i as f64)).collect();
    let sweep = frequency_sweep(&ENTRY, &cfg, 0, &frequencies, 0.05).unwrap();
    let lowest = sweep.iter().min_by(|a, b| a.gain.total_cmp(&b.gain)).unwrap();
    assert!((lowest.frequency - dip).abs() < 0.05 * dip, "dip at {} Hz, by hand {dip} Hz", lowest.frequency);
    let far_below = frequency_sweep(&ENTRY, &cfg, 0, &[dip / 20.0], 0.05).unwrap()[0];
    // Far below the dip the machine still looks like its rigid inertia (within the belt's give).
    let j_motor_side = (6.0 * 0.02 * 0.02 + 1.0e-4 + 0.1 * 2.6 * 0.02 * 0.02) / 25.0 + 0.3e-4;
    let rigid = 1.0 / (j_motor_side * std::f64::consts::TAU * far_below.frequency);
    assert!((far_below.gain - rigid).abs() < 0.03 * rigid, "gain {} at {} Hz, rigid {rigid}", far_below.gain, far_below.frequency);
    assert!(lowest.gain * lowest.frequency < 0.1 * far_below.gain * far_below.frequency, "no clear dip");
}

/// Tuning: with a fast velocity loop the position loop is a first-order lag with time constant
/// 1 / Kv. The step response and the frequency response must both say so.
#[test]
fn step_and_frequency_response_show_the_position_loop_gain() {
    // Gains well below the velocity loop's bandwidth, where the first-order picture holds.
    for kv in [25.0, 50.0] {
        let mut cfg = ideal_rigid();
        cfg.set_text(&format!("drive.kv_1_s = {kv:?}")).unwrap();
        let step = step_response(&ENTRY, &cfg, 0, 0.001, 0.4).unwrap();
        // By hand: 63.2 % after 1 / Kv, 10 % to 90 % in ln(9) / Kv.
        let at_tau = step.trace.iter().find(|(t, _)| *t >= 1.0 / kv).unwrap().1 / step.size;
        assert!((at_tau - 0.632).abs() < 0.05, "Kv {kv}: {at_tau} of the step after 1/Kv");
        let rise = 9f64.ln() / kv;
        assert!((step.rise_time - rise).abs() < 0.1 * rise, "Kv {kv}: rise time {} s, by hand {rise} s", step.rise_time);
        assert!(step.overshoot < 0.05 && step.settling_time < 6.0 / kv, "Kv {kv}: {} overshoot, settled after {} s", step.overshoot, step.settling_time);
        assert!((step.trace.last().unwrap().1 - step.size).abs() < 1e-3 * step.size);

        // By hand: -3 dB and -45 degrees at Kv / 2 pi; gain 1 far below.
        let corner = kv / std::f64::consts::TAU;
        let response = closed_loop_response(&ENTRY, &cfg, 0, &log_spaced(corner / 8.0, corner * 4.0, 25), 0.0005).unwrap();
        let measured = bandwidth(&response).unwrap();
        assert!((measured - corner).abs() < 0.1 * corner, "Kv {kv}: bandwidth {measured} Hz, by hand {corner} Hz");
        assert!((response[0].gain - 1.0).abs() < 0.02, "Kv {kv}: gain {} at {} Hz", response[0].gain, response[0].frequency);
        let at_corner = closed_loop_response(&ENTRY, &cfg, 0, &[corner], 0.0005).unwrap()[0];
        assert!((at_corner.phase_deg + 45.0).abs() < 10.0, "Kv {kv}: phase {} at the corner", at_corner.phase_deg);
    }
}

#[test]
fn too_much_gain_shows_as_overshoot_in_the_step_response() {
    let step = |edit: &str| {
        let mut cfg = ENTRY.config();
        cfg.set_text(edit).unwrap();
        step_response(&ENTRY, &cfg, 0, 0.002, 0.4).unwrap()
    };
    let (tame, hot) = (step("drive.kv_1_s = 50.0"), step("drive.kv_1_s = 400.0"));
    assert!(hot.overshoot > tame.overshoot + 0.1, "overshoot {} at Kv 400, {} at Kv 50", hot.overshoot, tame.overshoot);
    assert!(hot.rise_time < tame.rise_time);
}
