//! Machine-specific checks, and the component checks that the plan runs once on this machine:
//! friction levels, backlash, belt natural frequency.

use super::*;
use sim_components::Sim;
use sim_components::analysis::{
    analyse_cycle, bandwidth, closed_loop_response, frequency_sweep, log_spaced, step_response,
};
use sim_components::identify::{fit_rigid, resonance};
use sim_components::trace::Trace;
use sim_core::testkit::{DT, Noise, PulseRun};
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
fn simulate(
    m: &SingleAxis,
    state: &mut [f64],
    seconds: f64,
    torque: impl Fn(f64) -> f64,
    mut each: impl FnMut(f64, &[f64], f64),
) {
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
    let mut state = vec![0.1, 0.0, 0.0, 0.0];
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
    // The carriage runs on beyond the stroke here, so the end stops are off.
    let m = rigid(&["effects.guide_friction = true", "effects.hard_stops = false"]);
    let mut state = vec![0.5, 0.0, 0.0, 0.0];
    // Push with a force that rises by 5 N/s and watch the friction force until the carriage slides.
    let force_to_torque = 0.02 / 5.0;
    let mut peak = 0f64;
    simulate(
        &m,
        &mut state,
        6.0,
        |t| 5.0 * t * force_to_torque,
        |_, s, torque| {
            if s[1] < m.friction.stribeck_v {
                peak = peak.max(signal(&m, s, torque, "axis.x.friction"));
            }
        },
    );
    assert!(state[1] > 0.1, "the carriage never broke away");
    assert!((peak - 20.0).abs() < 0.02 * 20.0, "breakaway at {peak} N, stiction level is 20 N");
}

/// Friction levels: steady sliding force equals Coulomb plus viscous.
#[test]
fn steady_sliding_force_is_coulomb_plus_viscous() {
    // The carriage runs on beyond the stroke here, so the end stops are off.
    let m = rigid(&["effects.guide_friction = true", "effects.hard_stops = false"]);
    for direction in [1.0, -1.0] {
        let mut state = vec![0.5, 0.0, 0.0, 0.0];
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

/// The frequency at which the carriage swings against the pulley on the belt, at x = 0.5 m.
fn belt_frequency(edit: &[&str]) -> f64 {
    // A very stiff gearbox without backlash, so motor and pulley turn as one inertia.
    let mut all = vec!["gearbox.stiffness_nm_rad = 1.0e7", "effects.gearbox_backlash = false"];
    all.extend(edit);
    let m = machine(lossless_config(), &all);
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
    crossings as f64 / 2.0 / seconds
}

/// Hand calculation: carriage (6 kg) and motor inertia joined by a belt of stiffness k.
fn two_mass_frequency(k: f64) -> f64 {
    let m_motor = (1.0e-4 + 0.1 * 2.6 * 0.02 * 0.02 + 25.0 * 0.3e-4) / (0.02 * 0.02);
    f64::sqrt(k * (1.0 / 6.0 + 1.0 / m_motor)) / std::f64::consts::TAU
}

/// Belt natural frequency matches the hand calculation from k and m within 5 %.
#[test]
fn belt_natural_frequency_matches_hand_calculation() {
    // At x = 0.5 m the two belt sections are 0.65 m and 1.95 m long and act side by side.
    let (measured, expected) = (belt_frequency(&[]), two_mass_frequency(5.0e5 * (1.0 / 0.65 + 1.0 / 1.95)));
    assert!((measured - expected).abs() < 0.05 * expected, "measured {measured} Hz, hand calculation {expected} Hz");
}

/// The same with the tooth stiffness: each section ends at the motor pulley once, so each has the
/// teeth in mesh in series, 1 / k = L / EA + 1 / k_tooth. With 1e6 N/m of tooth stiffness the belt
/// stiffness falls from 1.03e6 to 0.64e6 N/m and the frequency by a fifth.
#[test]
fn belt_natural_frequency_with_tooth_stiffness_matches_hand_calculation() {
    let measured = belt_frequency(&["effects.belt_tooth_stiffness = true", "belt.tooth_mesh_stiffness_n_m = 1.0e6"]);
    let k = 1.0 / (0.65 / 5.0e5 + 1.0 / 1.0e6) + 1.0 / (1.95 / 5.0e5 + 1.0 / 1.0e6);
    let expected = two_mass_frequency(k);
    assert!((measured - expected).abs() < 0.05 * expected, "measured {measured} Hz, hand calculation {expected} Hz");
    assert!(expected < 0.85 * two_mass_frequency(5.0e5 * (1.0 / 0.65 + 1.0 / 1.95)));
}

/// Tooth-jump flag. Moving at 1 m/s takes 15 N of Coulomb and 20 N of viscous friction, accelerating 6 kg
/// at 10 m/s2 another 60 N. The two belt sections share that force by stiffness, and the short one (0.25 m
/// at the start, against 2.35 m) takes nine tenths of it. A belt pretensioned to 20 N therefore has a section
/// without tension for most of the 1 s move; at 300 N the lowest tension stays above 200 N.
#[test]
fn a_slack_belt_section_is_counted_and_a_tight_one_is_not() {
    let slack_time = |pretension: &str| {
        let mut cfg = ENTRY.config();
        cfg.set_text(pretension).unwrap();
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        sim.command(sim_components::Command::MoveTo(vec![0.9])).unwrap();
        sim.run(1.5, |_| {});
        let mut sample = Vec::new();
        sim.sample(&mut sample);
        sample[sim.signal_index("belt.slack_time").unwrap()]
    };
    let (loose, tight) = (slack_time("belt.pretension_n = 20.0"), slack_time("belt.pretension_n = 300.0"));
    assert!(loose > 0.5 && loose < 1.0, "{loose} s without tension");
    assert_eq!(tight, 0.0);

    // The cycle analysis reports the time, and "belt-slack" fails on it. On the way back the long section is
    // the one that slackens and takes less of the force, so the cycle has less than twice the move's time.
    let judged = |pretension: &str| {
        let mut cfg = ENTRY.config();
        cfg.set_text(pretension).unwrap();
        let load = analyse_cycle(&mut Sim::new(&ENTRY, &cfg).unwrap()).unwrap();
        let ok = sim_components::analysis::verdicts(&cfg, &load)
            .unwrap()
            .iter()
            .find(|v| v.check == "belt-slack")
            .unwrap()
            .ok;
        (load.belt_slack_time.unwrap(), ok)
    };
    let (loose_cycle, ok) = judged("belt.pretension_n = 20.0");
    assert!(loose_cycle > loose && loose_cycle < 2.0 * loose && !ok, "{loose_cycle} s without tension in the cycle");
    assert_eq!(judged("belt.pretension_n = 300.0"), (0.0, true));
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
    // Electrical energy: the cycle starts and ends at rest and this config has no friction, so what the motor
    // draws and does not feed back is the copper loss, R I^2 over the cycle (R rises by less than 1 % here).
    let copper = 4.0 * m.rms_current * m.rms_current * load.duration;
    assert!(m.energy_regenerated > 0.0 && m.largest_regeneration > 0.0 && m.peak_regenerated_power > 0.0);
    assert!(
        (m.energy_in - m.energy_regenerated - copper).abs() < 0.03 * copper,
        "{} J in, {} J back, {copper} J of copper loss",
        m.energy_in,
        m.energy_regenerated
    );
    // Each braking gives back the kinetic energy, less the copper loss on the way: at most 1/2 m_eff v^2.
    assert!(
        m.largest_regeneration < 0.5 * m_total * v * v && m.largest_regeneration > 0.3 * 0.5 * m_total * v * v,
        "{} J",
        m.largest_regeneration
    );
    // The winding settles where the copper loss of the RMS current balances: far below the rated 100 K.
    let rated_current = 2.822;
    let q = 100.0 / 1.39 * (m.rms_current / rated_current).powi(2);
    assert!((m.winding_rise - q / (1.0 - 0.0039 * q)).abs() < 1e-9 && m.winding_rise < 100.0, "{} K", m.winding_rise);
    // The thermal verdict: RMS torque over the rated torque of the config.
    assert!(
        (m.thermal_utilisation - m.rms_torque / 1.27).abs() < 1e-12,
        "thermal utilisation {}",
        m.thermal_utilisation
    );
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
    assert!(
        (far_below.gain - rigid).abs() < 0.03 * rigid,
        "gain {} at {} Hz, rigid {rigid}",
        far_below.gain,
        far_below.frequency
    );
    assert!(lowest.gain * lowest.frequency < 0.1 * far_below.gain * far_below.frequency, "no clear dip");
}

/// The velocity loop's open-loop response on the rigid loss-free axis against the loop by hand: PI on a pure
/// inertia, the position loop in parallel, the current loop a first-order lag of T = L / Kp_current = 0.25 ms.
/// |L| = Kp |1 + 1/(j w Tn)| |1 + Kv e^(-j w 0.5 ms)/(j w)| / (J w |1 + j w T|), the 0.5 ms the hold of the 1 ms
/// position cycle; delays move only the phase (and higher up the discrete current loop leaves the first-order lag:
/// 2.9 % off at 400 Hz). Measured 0.3976 against 0.3973 at 100 Hz, 0.1914 against 0.1898 at 200 Hz. J = 3e-5 + (1e-4 + 4 kg * 0.02^2) / 5^2 = 9.8e-5 kg m2. And the
/// gain margin follows the gain: Kp twice as high takes 20 log 2 = 6.02 dB off it.
#[test]
fn the_velocity_loop_response_is_the_hand_calculated_loop() {
    use sim_components::analysis::{loop_margins, velocity_loop_response};
    let mut cfg = ideal_rigid();
    for e in [
        "effects.drive_delay = false",
        "effects.back_emf = false",
        "effects.torque_saturation = false",
        "drive.velocity_kp_nm_s_rad = 0.025",
    ] {
        cfg.set_text(e).unwrap();
    }
    let (kp, tn, kv, t, j) = (0.025, 0.01, 50.0, 0.010 / 40.0, 3e-5 + (1e-4 + 4.0 * 0.02 * 0.02) / 25.0);
    for f in [100.0, 200.0] {
        let w: f64 = std::f64::consts::TAU * f;
        let position = ((1.0 + kv * (-w * 0.5e-3).sin() / w).powi(2) + (kv * (-w * 0.5e-3).cos() / w).powi(2)).sqrt();
        let by_hand = kp * (1.0 + 1.0 / (w * tn).powi(2)).sqrt() * position / (j * w * (1.0 + (w * t).powi(2)).sqrt());
        let p = velocity_loop_response(&ENTRY, &cfg, 0, &[f], 0.1).unwrap()[0];
        let gain = p.re.hypot(p.im);
        assert!((gain - by_hand).abs() < 0.02 * by_hand, "{f} Hz: |L| {gain}, by hand {by_hand}");
    }
    let frequencies = log_spaced(50.0, 2000.0, 40);
    let margin = |kp: f64| {
        let mut cfg = cfg.clone();
        cfg.set_text(&format!("drive.velocity_kp_nm_s_rad = {kp:?}")).unwrap();
        loop_margins(&velocity_loop_response(&ENTRY, &cfg, 0, &frequencies, 0.1).unwrap()).gain_margin_db
    };
    let (low, high) = (margin(0.025), margin(0.05));
    assert!((low - high - 20.0 * 2f64.log10()).abs() < 0.1, "gain margin {low} dB at Kp 0.025, {high} dB at 0.05");
}

/// The default config with a velocity gain of 0.3 N m s/rad, twelve times the shipped 0.025: the loop is unstable, and
/// the margins say so (not stable: a margin negative, or the measurement not valid as the loop ran into its torque
/// limit or moved on its own). The shipped gain is stable.
#[test]
fn an_unstable_velocity_loop_is_reported_unstable() {
    use sim_components::analysis::velocity_loop_margins;
    let mut cfg = ENTRY.config();
    assert!(velocity_loop_margins(&ENTRY, &cfg, &[]).unwrap()[0].1[0].stable());
    cfg.set_text("drive.velocity_kp_nm_s_rad = 0.3").unwrap();
    let m = velocity_loop_margins(&ENTRY, &cfg, &[]).unwrap()[0].1[0];
    println!("{m:?}");
    assert!(!m.stable(), "{m:?}");
}

/// Tuning: with a fast velocity loop the position loop is a first-order lag with time constant
/// 1 / Kv. The step response and the frequency response must both say so.
#[test]
fn step_and_frequency_response_show_the_position_loop_gain() {
    // Gains well below the velocity loop's bandwidth, where the first-order picture holds. The config's velocity gain
    // of 0.025, tuned for margin on the elastic axis, puts that bandwidth at Kp / J = 0.025 / 9.8e-5 = 255 rad/s, too
    // close to Kv = 50 (71 % of the step after 1 / Kv); 0.08 is stable on this rigid loss-free axis and puts it at 816.
    for kv in [25.0, 50.0] {
        let mut cfg = ideal_rigid();
        cfg.set_text("drive.velocity_kp_nm_s_rad = 0.08").unwrap();
        cfg.set_text(&format!("drive.kv_1_s = {kv:?}")).unwrap();
        let step = step_response(&ENTRY, &cfg, 0, 0.001, 0.4).unwrap();
        // By hand: 63.2 % after 1 / Kv, 10 % to 90 % in ln(9) / Kv.
        let at_tau = step.trace.iter().find(|(t, _)| *t >= 1.0 / kv).unwrap().1 / step.size;
        assert!((at_tau - 0.632).abs() < 0.05, "Kv {kv}: {at_tau} of the step after 1/Kv");
        let rise = 9f64.ln() / kv;
        assert!(
            (step.rise_time - rise).abs() < 0.1 * rise,
            "Kv {kv}: rise time {} s, by hand {rise} s",
            step.rise_time
        );
        assert!(
            step.overshoot < 0.05 && step.settling_time < 6.0 / kv,
            "Kv {kv}: {} overshoot, settled after {} s",
            step.overshoot,
            step.settling_time
        );
        assert!((step.trace.last().unwrap().1 - step.size).abs() < 1e-3 * step.size);

        // By hand: -3 dB and -45 degrees at Kv / 2 pi; gain 1 far below.
        let corner = kv / std::f64::consts::TAU;
        let response =
            closed_loop_response(&ENTRY, &cfg, 0, &log_spaced(corner / 8.0, corner * 4.0, 25), 0.0005).unwrap();
        let measured = bandwidth(&response).unwrap();
        assert!((measured - corner).abs() < 0.1 * corner, "Kv {kv}: bandwidth {measured} Hz, by hand {corner} Hz");
        assert!(
            (response[0].gain - 1.0).abs() < 0.02,
            "Kv {kv}: gain {} at {} Hz",
            response[0].gain,
            response[0].frequency
        );
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

/// Verdicts of the reference cycle against a hand calculation, and what happens when a part is too small.
#[test]
fn verdicts_match_the_hand_calculation_and_fail_where_a_part_is_too_small() {
    use sim_components::analysis::{Verdict, verdicts};
    let judged = |edit: &[&str]| -> (sim_components::analysis::CycleLoad, Vec<Verdict>) {
        let mut cfg = ideal_rigid();
        // The braking checks as estimated from what the motor feeds back; the simulated DC link has a test of its own.
        cfg.set_text("effects.dc_link = false").unwrap();
        for e in edit {
            cfg.set_text(e).unwrap();
        }
        let load = analyse_cycle(&mut Sim::new(&ENTRY, &cfg).unwrap()).unwrap();
        let verdicts = verdicts(&cfg, &load).unwrap();
        (load, verdicts)
    };
    let find = |all: &[Verdict], check: &str| all.iter().find(|v| v.check == check).cloned();
    let (load, all) = judged(&[]);
    let m = &load.motors[0];
    assert!(all.iter().all(|v| v.ok), "{all:?}");
    // The gearbox output turns the pulley and moves the carriage, (m + J_pulley / r^2) r a = 4.25 kg * 0.02 m *
    // 10 m/s^2 = 0.85 N m, against 40 N m rated and 64 N m maximum. The rotor's own 25 J_rotor / r^2 = 1.875 kg
    // never pass through it: the motor torque times the ratio of 5 is 1.225 N m.
    let peak = find(&all, "gearbox-peak").unwrap();
    assert!(
        (peak.value - 0.85).abs() < 0.02 * 0.85
            && peak.limit == 64.0
            && (5.0 * m.peak_torque - 1.225).abs() < 0.02 * 1.225,
        "{} N m",
        peak.value
    );
    // Continuous: the cubic mean of the torque, weighted with the speed. Each of the four ramps is 0.1 s of
    // rising and 0.1 s of falling acceleration (a = 100 t, v = 50 t^2, and mirrored from v = 1 m/s), so the
    // integral of a^3 v dt over a ramp is 5e7 t^6 / 6 = 8.33 plus 1e6 u^4 / 4 - 5e7 u^6 / 6 = 16.67, 25 in all,
    // 100 over the cycle. The integral of v dt is the way, 1.6 m: the cubic mean acceleration is
    // (100 / 1.6)^(1/3) = 3.969 m/s^2, against an RMS of 3.4. Torque at the gearbox output per acceleration is
    // (m + J_pulley / r^2) r = 4.25 kg * 0.02 m; the motor's own cubic mean has the rotor in it, 6.125 kg.
    let continuous = find(&all, "gearbox-continuous").unwrap();
    let by_hand = 4.25 * 0.02 * (100.0f64 / 1.6).cbrt();
    assert!(
        (continuous.value - by_hand).abs() < 0.02 * by_hand && continuous.limit == 40.0,
        "{} N m, by hand {by_hand} N m",
        continuous.value
    );
    assert!(
        m.gearbox_torque == Some((peak.value, continuous.value))
            && (5.0 * m.cubic_mean_torque - by_hand * 6.125 / 4.25).abs() < 0.02 * by_hand
            && m.cubic_mean_torque > 1.1 * m.rms_torque
    );
    // The gearbox input turns at v i / r = 1 * 5 / 0.02 = 250 rad/s = 2387 r/min at most, against 13000 r/min.
    let speed = find(&all, "gearbox-input-speed").unwrap();
    assert!((speed.value - 2387.3).abs() < 0.01 * 2387.3 && speed.limit == 13000.0, "{} r/min", speed.value);
    // The rigid model's belt does not stretch: both sections pull the pulley shaft with the pretension, 2 * 300 N.
    assert_eq!(
        find(&all, "gearbox-radial-load").unwrap(),
        Verdict { check: "gearbox-radial-load", motor: None, value: 600.0, limit: 1500.0, ok: true }
    );
    // The guide carries the carriage of 4 kg (this config has no payload): 39.23 N, against 2000 N.
    let guide = find(&all, "guide-load").unwrap();
    assert!((guide.value - 4.0 * G).abs() < 1e-9 && guide.limit == 2000.0, "{} N", guide.value);
    assert_eq!(
        find(&all, "motor-thermal").unwrap(),
        Verdict { check: "motor-thermal", motor: Some(0), value: m.winding_rise, limit: 100.0, ok: true }
    );
    // The DC link: 470 uF between 155.13 V and 224.65 V hold C (U2^2 - U1^2) / 2 = 6.2 J, more than one braking gives back.
    let link = find(&all, "dc-link-energy").unwrap();
    assert!((link.limit - 6.204).abs() < 1e-3 && link.value == m.largest_regeneration && link.value < link.limit);
    assert!(find(&all, "braking-resistor-continuous").is_none());

    // A tenth of the capacitance cannot hold a braking: the resistor has to, and is judged on mean and peak power.
    let (load, all) = judged(&["ratings.dc_link_capacitance_uf = 47.0"]);
    let m = &load.motors[0];
    assert!(find(&all, "dc-link-energy").unwrap().value > 0.62 && all.iter().all(|v| v.ok), "{all:?}");
    assert!(
        (find(&all, "braking-resistor-continuous").unwrap().value - m.energy_regenerated / load.duration).abs() < 1e-12
    );
    assert_eq!(find(&all, "braking-resistor-peak").unwrap().value, m.peak_regenerated_power);
    // A resistor that is too small for the mean power fails; so does a gearbox below the peak torque.
    let (_, all) = judged(&[
        "ratings.dc_link_capacitance_uf = 47.0",
        "ratings.braking_resistor_continuous_w = 0.5",
        "ratings.gearbox_max_torque_nm = 0.8",
    ]);
    let failed: Vec<&str> = all.iter().filter(|v| !v.ok).map(|v| v.check).collect();
    assert_eq!(failed, ["gearbox-peak", "braking-resistor-continuous"]);
    // So do a gearbox that may not turn that fast, a shaft and a guide that are too weak.
    let (_, all) = judged(&[
        "ratings.gearbox_max_input_speed_rpm = 2000.0",
        "ratings.shaft_max_radial_load_n = 500.0",
        "ratings.guide_max_load_n = 30.0",
    ]);
    let failed: Vec<&str> = all.iter().filter(|v| !v.ok).map(|v| v.check).collect();
    assert_eq!(failed, ["gearbox-input-speed", "gearbox-radial-load", "guide-load"]);

    // Guide life. With the payload the guide carries 6 kg on the way out and through the dwell that follows
    // the way back, and 4 kg for the other half of the cycle: 58.84 N and 39.23 N for the same time each.
    // The cubic mean is ((58.84^3 + 39.23^3) / 2)^(1/3) = 50.92 N, and with C = 5000 N the life is
    // (5000 / 50.92)^3 * 50 km = 47.3 million km.
    let (load, all) = judged(&["effects.payload = true"]);
    let (peak, mean) = load.guide_load.unwrap();
    assert!((peak - 6.0 * G).abs() < 1e-9 && find(&all, "guide-load").unwrap().value == peak);
    assert!((mean - 50.92).abs() < 0.02, "cubic mean {mean} N");
    let life = sim_components::analysis::guide_life_km(&ideal_rigid(), &load).unwrap();
    assert!((life - 47.3e6).abs() < 0.01 * 47.3e6, "{life} km");
}

/// The DC link over a cycle. Each braking feeds back the kinetic energy, 6.125 kg * (1 m/s)^2 / 2 = 3.0625 J,
/// less the copper loss on the way: the current is 0.0245 N m s^2/m / 0.45 N m/A times the acceleration, whose
/// square over a ramp integrates to 2 * 100^2 * 0.1^3 / 3 = 6.67 m^2/s^3, so 4 Ohm * 0.0544^2 * 6.67 = 0.079 J.
/// The capacitors take the 2.983 J and the voltage rises from U1 = 155.13 V to U2 with C (U2^2 - U1^2) / 2 =
/// energy: with 470 uF to sqrt(155.13^2 + 2 * 2.983 / 470e-6) = 191.7 V, below the 224.65 V at which the
/// chopper switches on. With 4.7 uF there is room for 4.7 uF (224.65^2 - 155.13^2) / 2 = 0.062 J only, which
/// the capacitors give back to the next acceleration: of the two brakings of a cycle the resistor burns all but
/// 2 * 0.062 J, and its mean power is that over the cycle time. A resistor that takes 5 W cannot hold the
/// voltage against a braking of some 50 W, so the drive trips at 257.6 V.
#[test]
fn the_dc_link_takes_braking_energy_the_resistor_the_rest_and_too_small_a_resistor_trips() {
    use sim_components::analysis::verdicts;
    let run = |edit: &[&str]| {
        let mut cfg = ideal_rigid();
        cfg.set_text("effects.dc_link = true").unwrap();
        for e in edit {
            cfg.set_text(e).unwrap();
        }
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        let load = analyse_cycle(&mut sim).unwrap();
        let verdicts = verdicts(&cfg, &load).unwrap();
        (sim, load, verdicts)
    };
    let value =
        |all: &[sim_components::analysis::Verdict], check: &str| all.iter().find(|v| v.check == check).map(|v| v.value);

    let (_, load, all) = run(&[]);
    let m = &load.motors[0];
    let stored = 0.5 * 470e-6 * (m.peak_dc_link_voltage.powi(2) - 155.13 * 155.13);
    assert!(
        (stored - m.largest_regeneration).abs() < 1e-3 && (stored - 2.983).abs() < 0.01,
        "{stored} J stored, {} J fed back in one braking",
        m.largest_regeneration
    );
    assert!((m.peak_dc_link_voltage - 191.7).abs() < 0.1 && m.resistor_energy == 0.0, "{} V", m.peak_dc_link_voltage);
    assert!(
        all.iter().all(|v| v.ok)
            && (value(&all, "dc-link-energy").unwrap() - stored).abs() < 1e-9
            && value(&all, "dc-link-overvoltage") == Some(m.peak_dc_link_voltage)
    );
    assert!(value(&all, "braking-resistor-continuous").is_none());

    let (_, load, all) = run(&["ratings.dc_link_capacitance_uf = 4.7"]);
    let m = &load.motors[0];
    let by_hand = (m.energy_regenerated - 2.0 * 0.062) / load.duration;
    let mean = m.resistor_energy / load.duration;
    assert!((mean - by_hand).abs() < 0.01 * by_hand && mean > 1.0, "resistor {mean} W, by hand {by_hand} W");
    assert!(
        (m.peak_dc_link_voltage - 224.65).abs() < 0.01 && m.peak_resistor_power > 30.0 && m.peak_resistor_power < 62.0,
        "{} V, {} W",
        m.peak_dc_link_voltage,
        m.peak_resistor_power
    );
    assert!(all.iter().all(|v| v.ok), "{all:?}");
    assert_eq!(
        (value(&all, "braking-resistor-continuous"), value(&all, "braking-resistor-peak")),
        (Some(mean), Some(m.peak_resistor_power))
    );

    let (mut sim, load, all) = run(&["ratings.dc_link_capacitance_uf = 4.7", "ratings.braking_resistor_peak_w = 5.0"]);
    sim.step();
    assert!(sim.control.drives[0].tripped && !sim.control.enabled() && sim.torque[0] == 0.0);
    assert!(
        load.motors[0].peak_dc_link_voltage > 257.6 && load.motors[0].peak_dc_link_voltage < 258.6,
        "{} V",
        load.motors[0].peak_dc_link_voltage
    );
    let failed: Vec<&str> = all.iter().filter(|v| !v.ok).map(|v| v.check).collect();
    assert_eq!(failed, ["dc-link-overvoltage"]);
    // Tripped drives refuse commands like disabled ones. Enabling them keeps the DC link's charge (the horizontal axis
    // holds no current, so none of it goes to the winding), still above the trip level; in the next step the chopper's
    // 5 W take it below (0.26 V of 4.7 uF at 257.8 V), and the drive runs.
    assert_eq!(sim.command(sim_components::Command::StartCycle).unwrap_err().key, "error-disabled");
    let charged = sim.control.drives[0].dc_voltage;
    sim.command(sim_components::Command::Enable).unwrap();
    assert!(
        sim.control.enabled()
            && !sim.control.drives[0].tripped
            && sim.control.drives[0].dc_voltage == charged
            && charged > 257.6,
        "{charged} V"
    );
    sim.step();
    assert!(
        !sim.control.drives[0].tripped && sim.control.enabled() && sim.control.drives[0].dc_voltage < 257.6,
        "{charged} V, then {} V",
        sim.control.drives[0].dc_voltage
    );
    // A threshold that is not between the bus voltage and the trip level is refused.
    let mut cfg = ideal_rigid();
    cfg.set_text("effects.dc_link = true").unwrap();
    cfg.set_text("drive.bus_voltage_v = 260.0").unwrap();
    assert!(Sim::new(&ENTRY, &cfg).is_err());
}

/// Fieldbus delay: the drives get the setpoints three profile cycles of 1 ms late, so with everything
/// else alike the carriage does at t + 3 ms what it did at t without the delay. At the cruise speed of
/// 1 m/s that is 3 mm behind.
#[test]
fn a_fieldbus_delay_shifts_the_move_by_its_cycles() {
    let positions = |cycles: usize| {
        let mut cfg = ideal_rigid();
        cfg.set_text(&format!("effects.fieldbus_delay = {}", cycles > 0)).unwrap();
        cfg.set_text(&format!("profile.fieldbus_delay_cycles = {cycles}")).unwrap();
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        sim.command(sim_components::Command::MoveTo(vec![0.9])).unwrap();
        let mut trace = Vec::new();
        sim.run(1.5, |s| trace.push(s.state[0]));
        trace
    };
    let (plain, late) = (positions(0), positions(3));
    let shift = (3.0 * 1e-3 / DT).round() as usize;
    let off = |shift: usize| plain.iter().zip(&late[shift..]).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
    assert!(off(shift) < 1e-12, "{} m off the shifted trace", off(shift));
    assert!((off(0) - 3e-3).abs() < 0.01 * 3e-3, "{} m behind", off(0));
    assert!((plain[plain.len() - 1] - 0.9).abs() < 1e-6 && (late[late.len() - 1] - 0.9).abs() < 1e-6);
}

/// Fieldbus jitter against a hand calculation. Each frame takes the delay of 1 cycle plus j more, j drawn evenly from
/// 0 to N; the drive uses the newest frame that has arrived. So the frame it uses is older than the delay by more
/// than i cycles only if each of the frames i + 1 cycles late or less has not arrived: P(more than i) =
/// prod over k = 0..i of (N - k) / (N + 1). N = 1: 0 or 1 cycle late, 1/2 each. N = 2: 0, 1 and 2 cycles with 1/3, 4/9
/// and 2/9. At a constant motor speed w = 1 m/s * 5 / 0.02 m = 250 rad/s, a frame c cycles old is w c 1 ms = 0.25 c rad
/// behind, so the RMS of the position setpoint against that without jitter is 0.25 rad sqrt(E[c^2]): sqrt(1/2) * 0.25 =
/// 0.1768 rad for N = 1, sqrt(4/3) * 0.25 = 0.2887 rad for N = 2. The ~600 cruise cycles are a sample of this, so 5 %.
#[test]
fn a_fieldbus_jitter_spreads_the_setpoints_by_the_hand_calculated_rms() {
    let setpoints = |on: bool, jitter: usize| {
        let mut cfg = ideal_rigid();
        for e in [
            "effects.fieldbus_delay = true".to_string(),
            format!("effects.fieldbus_jitter = {on}"),
            format!("profile.fieldbus_jitter_cycles = {jitter}"),
        ] {
            cfg.set_text(&e).unwrap();
        }
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        sim.command(sim_components::Command::MoveTo(vec![0.9])).unwrap();
        let mut out = Vec::new();
        sim.run(1.5, |s| {
            if s.step % 16 == 0 {
                out.push((s.control.drives[0].pos_set, s.control.drives[0].vel_set));
            }
        });
        out
    };
    let plain = setpoints(false, 3);
    for (jitter, shares, rms) in [
        (1, vec![0.5, 0.5], 0.5f64.sqrt() * 0.25),
        (2, vec![1.0 / 3.0, 4.0 / 9.0, 2.0 / 9.0], (4.0f64 / 3.0).sqrt() * 0.25),
    ] {
        let late = setpoints(true, jitter);
        let mut counts = vec![0.0; jitter + 1];
        let (mut sum, mut n) = (0.0, 0.0);
        for i in jitter + 1..plain.len() {
            // A cruise: the frames that can be used here all at 250 rad/s.
            if plain[i - jitter - 1..=i].iter().all(|(_, v)| (v - 250.0).abs() < 1e-6) {
                let behind = plain[i].0 - late[i].0;
                let cycles = (behind / 0.25).round();
                assert!(
                    (behind - 0.25 * cycles).abs() < 1e-9 && (0.0..=jitter as f64).contains(&cycles),
                    "{behind} rad behind"
                );
                counts[cycles as usize] += 1.0;
                (sum, n) = (sum + behind * behind, n + 1.0);
            }
        }
        let measured = (sum / n).sqrt();
        assert!(
            n > 500.0 && (measured - rms).abs() < 0.05 * rms,
            "N = {jitter}: RMS {measured} rad over {n} cycles, by hand {rms}"
        );
        for (count, share) in counts.iter().zip(&shares) {
            assert!(
                (count / n - share).abs() < 0.05,
                "N = {jitter}: shares {:?}, by hand {shares:?}",
                counts.iter().map(|c| c / n).collect::<Vec<_>>()
            );
        }
    }
    // Switched on with no cycles of jitter, the setpoints are those with the switch off.
    assert!(setpoints(true, 0) == plain);
}

/// A second input shaper against the ZVD residual by hand. A machine with two undamped modes, at 25 and 62 Hz (the
/// two-motor machine has its belt mode near the first and its sweep one near the second): each is set swinging by the setpoint
/// acceleration a(t), and what it keeps swinging with after the move is |sum of a(t) e^(-j w t) dt| / w. A ZVD shaper
/// with no damping (K = 1, copies at 0, T/2 and T, T = 1 / f_s, weights 1 : 2 : 1) multiplies that by
/// |H| = (1 + cos(w T / 2)) / 2 = cos^2(pi f / (2 f_s)). One shaper at 25 Hz: 0 at 25 Hz, cos^2(pi 62 / 50) = 0.5314
/// at 62 Hz. One at 62 Hz: cos^2(pi 25 / 124) = 0.6503 at 25 Hz. Both convolved: the product, 0 at both. The move is
/// 0.75 m: a jerk-limited move has zeros in its spectrum (at multiples of 1 / 0.8 s for 0.8 m, 25 Hz among them, where
/// the plain move starts no swing to compare with).
#[test]
fn a_second_shaper_suppresses_a_second_resonance_by_the_zvd_formula() {
    let accelerations = |shaping: bool, first: f64, second: f64| {
        let mut cfg = ideal_rigid();
        for e in [
            format!("effects.input_shaping = {shaping}"),
            format!("profile.shaper_frequency_hz = {first:?}"),
            format!("profile.shaper2_frequency_hz = {second:?}"),
            "profile.shaper_damping = 0.0".into(),
            "profile.shaper2_damping = 0.0".into(),
        ] {
            cfg.set_text(&e).unwrap();
        }
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        sim.command(sim_components::Command::MoveTo(vec![0.85])).unwrap();
        let mut out = Vec::new();
        sim.run(1.5, |s| {
            if s.step % 16 == 1 {
                out.push(s.control.set_acc[0]);
            }
        });
        // The setpoint has arrived, with both shapers: the move ends at rest on the target.
        assert!(
            (sim.control.set_pos[0] - 0.85).abs() < 1e-12 && sim.control.set_acc[0] == 0.0,
            "{} m",
            sim.control.set_pos[0]
        );
        out
    };
    let swing = |acc: &[f64], f: f64| {
        let w = std::f64::consts::TAU * f;
        let (c, s) = acc.iter().enumerate().fold((0.0, 0.0), |(c, s), (n, a)| {
            (c + a * (w * n as f64 * 1e-3).cos(), s + a * (w * n as f64 * 1e-3).sin())
        });
        c.hypot(s) * 1e-3 / w
    };
    let plain = accelerations(false, 25.0, 0.0);
    let left = |acc: &[f64], f: f64| swing(acc, f) / swing(&plain, f);
    let (at_25, at_62, both) =
        (accelerations(true, 25.0, 0.0), accelerations(true, 62.0, 0.0), accelerations(true, 25.0, 62.0));
    println!(
        "left: shaper at 25 Hz {:.4} / {:.4}, at 62 Hz {:.4} / {:.4}, both {:.4} / {:.4}",
        left(&at_25, 25.0),
        left(&at_25, 62.0),
        left(&at_62, 25.0),
        left(&at_62, 62.0),
        left(&both, 25.0),
        left(&both, 62.0)
    );
    assert!(swing(&plain, 25.0) > 0.0 && swing(&plain, 62.0) > 0.0);
    assert!(left(&at_25, 25.0) < 0.01 && (left(&at_25, 62.0) - 0.5314).abs() < 0.02, "one shaper at 25 Hz");
    assert!(left(&at_62, 62.0) < 0.03 && (left(&at_62, 25.0) - 0.6503).abs() < 0.02, "one shaper at 62 Hz");
    assert!(left(&both, 25.0) < 0.1 && left(&both, 62.0) < 0.1, "both shapers");
}

/// Radial load on the pulley shaft against a hand calculation. On a vertical axis the 6 kg hang on the long
/// belt section: T2 - T1 = 58.84 N. The belt as a whole keeps its length, so L1 T1 + L2 T2 stays
/// (L1 + L2) * 300 N; with L1 = 0.65 m and L2 = 1.95 m at x = 0.5 m that gives T1 = 300 - 58.84 * 0.75 and
/// T2 = 300 + 58.84 * 0.25. Both pull the shaft the same way: T1 + T2 = 600 - 58.84 * 0.5 = 570.58 N.
#[test]
fn radial_load_on_the_pulley_shaft_is_the_sum_of_both_section_tensions() {
    let m = machine(lossless_config(), &["geometry.incline_deg = 90.0"]);
    let (mut state, mut hold) = (Vec::new(), [0.0]);
    m.equilibrium(&[0.5], &mut state, &mut hold);
    let radial = signal(&m, &state, hold[0], "pulley.m.radial_load");
    assert!((radial - 570.58).abs() < 0.02, "{radial} N");
    assert!((signal(&m, &state, hold[0], "belt.2.tension") - 300.0 - 58.84 * 0.25).abs() < 0.02);
    // Level, nothing hangs on the belt, and the guide carries the weight instead.
    let level = machine(lossless_config(), &[]);
    level.equilibrium(&[0.5], &mut state, &mut hold);
    assert!((signal(&level, &state, hold[0], "pulley.m.radial_load") - 600.0).abs() < 1e-6);
    assert!(
        (signal(&level, &state, hold[0], "axis.x.guide_load") - 6.0 * G).abs() < 1e-12
            && signal(&m, &state, hold[0], "axis.x.guide_load").abs() < 1e-12
    );
}

/// Hard stops against a hand calculation. The motor pushes the carriage into the stop at x = 0 with 0.1 N m,
/// which is 0.1 * 5 / 0.02 = 25 N; a stop of 1e6 N/m gives way by 25 um. The same at the far end. Friction
/// would hold part of the force, so it is off.
#[test]
fn a_carriage_pushed_into_the_hard_stop_rests_at_the_hand_calculated_depth() {
    let edit = [
        "effects.guide_friction = false",
        "effects.motor_friction = false",
        "effects.torque_ripple = false",
        "effects.tooth_meshing = false",
    ];
    // (how far beyond the end of the stroke, speed) after 2 s of pushing.
    let pushed = |m: &SingleAxis, end: f64, push: f64| {
        let (mut state, mut hold) = (Vec::new(), [0.0]);
        m.equilibrium(&[end], &mut state, &mut hold);
        simulate(m, &mut state, 2.0, |_| hold[0] + push, |_, _, _| {});
        (state[0] - end, state[3])
    };
    let m = machine(ENTRY.config(), &edit);
    for (end, push, by_hand) in [(0.0, -0.1, -25e-6), (1.0, 0.1, 25e-6)] {
        let (beyond, speed) = pushed(&m, end, push);
        assert!((beyond - by_hand).abs() < 0.01 * by_hand.abs(), "{beyond} m beyond the stroke, by hand {by_hand} m");
        assert!(speed.abs() < 1e-6, "still moving at {speed} m/s");
    }
    // Without stops nothing holds the carriage: in 2 s it is far beyond the end.
    let mut open = edit.to_vec();
    open.push("effects.hard_stops = false");
    assert!(pushed(&machine(ENTRY.config(), &open), 0.0, -0.1).0 < -1.0);
}

/// Error bands on the verdicts: with a bound on the moved mass, the band on the gearbox's peak torque is the
/// hand calculation at the low and at the high mass, and a verdict that passes with the config as it is
/// but not at a bound fails with bands.
#[test]
fn a_bound_on_the_moved_mass_gives_the_hand_calculated_band_and_fails_the_verdict_it_reaches() {
    use sim_components::analysis::{Band, bounds, verdict_bands};
    let mut cfg = ideal_rigid();
    let mass = [("mass.carriage_kg".to_string(), 3.0, 6.0)];
    // Torque is in proportion to the inertia moved. At the gearbox output, at 10 m/s^2:
    // (m + J_pulley / r^2) r a = 0.2 (m + 0.25) N m, so 0.65, 0.85 and 1.25 N m for 3, 4 and 6 kg.
    let by_hand = |m: f64| 0.2 * (m + 1.0e-4 / (0.02 * 0.02));
    let peak = |bands: Vec<Band>| bands.into_iter().find(|b| b.verdict.check == "gearbox-peak").unwrap();
    let band = peak(verdict_bands(&ENTRY, &cfg, &mass).unwrap());
    for (value, m) in [(band.low, 3.0), (band.verdict.value, 4.0), (band.high, 6.0)] {
        assert!((value - by_hand(m)).abs() < 0.02 * by_hand(m), "{value} N m with {m} kg, by hand {} N m", by_hand(m));
    }
    assert!(band.verdict.ok && band.ok);

    // A gearbox that allows 1 N m is enough for 4 kg and not for 6 kg.
    cfg.set_text("ratings.gearbox_max_torque_nm = 1.0").unwrap();
    let band = peak(verdict_bands(&ENTRY, &cfg, &mass).unwrap());
    assert!(band.verdict.ok && !band.ok, "{band:?}");
    // Without bounds the band is the value itself.
    let alone = peak(verdict_bands(&ENTRY, &cfg, &[]).unwrap());
    assert!(alone.ok && (alone.low, alone.high) == (alone.verdict.value, alone.verdict.value));

    // The bounds of a config are read from its [bounds]; one on a value the config does not have is refused.
    assert!(bounds(&ENTRY.config()).unwrap().contains(&("friction_x.coulomb_n".to_string(), 10.0, 20.0)));
    let typo =
        Config::parse(&ENTRY.default_config.replace("[bounds]\n", "[bounds]\nmass.cariage_kg = [3.0, 5.0]\n")).unwrap();
    assert_eq!(bounds(&typo).unwrap_err().detail(), "mass.cariage_kg");
}

/// The optimiser finds a shorter cycle that still passes every verdict, and refuses a cycle that fails
/// one to begin with.
#[test]
fn the_optimiser_shortens_the_cycle_within_the_verdicts() {
    use sim_components::analysis::{optimise_cycle, verdicts};
    let cfg = ENTRY.config();
    let found = optimise_cycle(&ENTRY, &cfg, 14).unwrap();
    assert!(
        found.runs <= 14 && found.after.duration < 0.95 * found.before.duration,
        "{} s to {} s",
        found.before.duration,
        found.after.duration
    );
    // The settings it reports give that cycle, and the cycle passes.
    let mut faster = cfg.clone();
    for (path, _, after) in &found.changes {
        faster.set_text(&format!("{path} = {after}")).unwrap();
    }
    assert!(!found.changes.is_empty());
    let load = analyse_cycle(&mut Sim::new(&ENTRY, &faster).unwrap()).unwrap();
    assert!(
        (load.duration - found.after.duration).abs() < 1e-9 && verdicts(&faster, &load).unwrap().iter().all(|v| v.ok)
    );
    assert!(load.dwell_error_peak <= cfg.f64("ratings.settling_tolerance_m").unwrap());

    // With a tolerance no machine meets, the cycle as configured fails and there is nothing to start from.
    let mut strict = cfg.clone();
    strict.set_text("ratings.settling_tolerance_m = 1.0e-9").unwrap();
    assert!(optimise_cycle(&ENTRY, &strict, 14).is_err());
}

/// The robust optimiser steps back to the fastest setting that passes in the error bands. With one bound, the payload
/// between 2 and 20 kg, each setting judged in the bands costs 2 * 1 + 1 = 3 cycles. The fastest setting of the 14
/// cycles of search on the config as written leaves the belt slack with 20 kg on, the cycle as configured does not (so
/// nothing is excused), and the step before the last passes: three settings judged, 9 cycles. The nominal search has no
/// robust part, and its result fails in the bands where the robust one passes.
#[test]
fn the_robust_optimiser_steps_back_to_the_fastest_setting_that_passes_in_the_bands() {
    use sim_components::analysis::{
        OptimiseOptions, Optimised, bounds, optimise_cycle, optimise_cycle_with, verdict_bands,
    };
    let mut cfg = ENTRY.config();
    cfg.set_text("bounds = { mass = { payload_kg = [2.0, 20.0] } }").unwrap();
    let nominal = optimise_cycle(&ENTRY, &cfg, 14).unwrap();
    let found =
        optimise_cycle_with(&ENTRY, &cfg, &OptimiseOptions { max_runs: 14, robust: true, pareto: false, worst: false })
            .unwrap();
    let robust = found.robust.clone().unwrap();
    assert_eq!(
        (robust.nominal_fails.as_slice(), robust.excused.len(), robust.steps_back, robust.runs),
        (["belt-slack"].as_slice(), 0, 1, 9),
        "{robust:?}"
    );
    assert!(
        nominal.robust.is_none() && robust.nominal_duration == nominal.after.duration,
        "{} s",
        robust.nominal_duration
    );
    assert!(
        found.after.duration > nominal.after.duration && found.after.duration < found.before.duration,
        "{} s to {} s",
        found.before.duration,
        found.after.duration
    );
    let applied = |found: &Optimised| {
        let mut out = cfg.clone();
        for (path, _, after) in &found.changes {
            out.set_text(&format!("{path} = {after}")).unwrap();
        }
        out
    };
    let passes = |cfg: &Config| verdict_bands(&ENTRY, cfg, &bounds(cfg).unwrap()).unwrap().iter().all(|b| b.ok);
    assert!(passes(&applied(&found)) && !passes(&applied(&nominal)));
}

/// The optimiser judged at the worst corner steps back to a setting that passes there, where the bands pass the fastest.
/// Two bounds, the belt's pretension 270 to 300 N and the payload 2 to 18 kg: each alone keeps the belt taut at the
/// fastest setting of the 14 cycles of search (the bands pass it), both at once, low pretension and heavy payload, do
/// not. With two bounds a worst-corner search runs the cycle as configured and each bound alone at its low and high
/// bound (2 * 2 + 1 = 5) and then corners, of which there are 2^2 = 4: 9 cycles at most. The search took three steps
/// (velocity x1.2, acceleration x1.5, jerk x1.5); judged are the fastest (5 + 3: its verdicts' sign corners are three of
/// the four, one of them leaves the belt slack and the search stops there), the cycle as configured (5 + 4, whole, to
/// find what it fails itself: nothing), then by bisection step 1 of 0 to 3 (5 + 4, passes) and step 2 of 1 to 3 (5 + 4,
/// fails): 8 + 9 + 9 + 9 = 35 cycles, the result step 1, two steps back. The bands' result fails `verdict_worst`, this
/// one passes it.
#[test]
fn the_optimiser_at_the_worst_corner_steps_back_to_a_setting_that_passes_there() {
    use sim_components::analysis::{OptimiseOptions, Optimised, bounds, optimise_cycle_with, verdict_worst};
    let mut cfg = ENTRY.config();
    cfg.set_text("bounds = { belt = { pretension_n = [270.0, 300.0] }, mass = { payload_kg = [2.0, 18.0] } }").unwrap();
    let options = OptimiseOptions { max_runs: 14, robust: true, pareto: false, worst: false };
    let bands = optimise_cycle_with(&ENTRY, &cfg, &options).unwrap();
    let worst = optimise_cycle_with(&ENTRY, &cfg, &OptimiseOptions { worst: true, ..options }).unwrap();
    let (in_bands, at_worst) = (bands.robust.clone().unwrap(), worst.robust.clone().unwrap());
    assert!(in_bands.nominal_fails.is_empty() && in_bands.steps_back == 0 && !in_bands.worst, "{in_bands:?}");
    assert_eq!(
        (at_worst.nominal_fails.as_slice(), at_worst.excused.len(), at_worst.steps_back, at_worst.runs, at_worst.worst),
        (["belt-slack"].as_slice(), 0, 2, 35, true),
        "{at_worst:?}"
    );
    assert!(
        worst.after.duration > bands.after.duration && worst.after.duration < worst.before.duration,
        "{} s, {} s in the bands",
        worst.after.duration,
        bands.after.duration
    );
    let applied = |found: &Optimised| {
        let mut out = cfg.clone();
        for (path, _, after) in &found.changes {
            out.set_text(&format!("{path} = {after}")).unwrap();
        }
        out
    };
    let passes =
        |cfg: &Config| verdict_worst(&ENTRY, cfg, &bounds(cfg).unwrap()).unwrap().0.iter().all(|w| w.verdict.ok);
    assert!(passes(&applied(&worst)) && !passes(&applied(&bands)));
}

/// Energy against cycle time (`sim-cli optimise --pareto`). The 0.8 m moves cannot reach 1 m/s at these accelerations:
/// each is a jerk-limited triangle that peaks where v^2/a + v a/j = d, at 0.88944 m/s with a = 1 m/s^2 and 0.63121 m/s
/// with 0.5 m/s^2 (j = 100 m/s^3; v above a^2/j, so a is reached). Each of the cycle's two stops gives back 1/2 m v^2,
/// m = 6.125 kg (carriage, pulley and rotor through radius and ratio, as above): 2.4228 J and 1.2202 J, less the copper
/// loss on the way (4 Ohm * (0.0245 N m s^2/m * 1 m/s^2 / 0.45 N m/A)^2 = 0.012 W over the 0.9 s of a stop: 0.01 J). So
/// the lower acceleration gives back 2 * 1.2026 = 2.405 J less per cycle, within 2 %. With the DC link at 4.7 uF the
/// braking resistor burns all but 0.062 J of each stop, which the supply gives again at the next start: the energy from
/// the supply falls by as much, within 3 % (the copper loss of the starts falls too, some 0.03 J).
#[test]
fn a_lower_acceleration_takes_the_hand_calculated_braking_energy_less_from_the_supply() {
    use sim_components::analysis::supply_energy;
    let run = |a: f64| {
        let mut cfg = ideal_rigid();
        for e in [
            "effects.dc_link = true".to_string(),
            "ratings.dc_link_capacitance_uf = 4.7".into(),
            format!("axis.x.max_acceleration_m_s2 = {a:?}"),
        ] {
            cfg.set_text(&e).unwrap();
        }
        let load = analyse_cycle(&mut Sim::new(&ENTRY, &cfg).unwrap()).unwrap();
        (load.motors[0].energy_regenerated, supply_energy(&cfg, &load))
    };
    // The peak speed of the triangle: v^2 + (a^2/j) v - a d = 0.
    let kinetic = |a: f64| {
        let (j, d) = (100.0, 0.8);
        let v: f64 = (-a * a / j + (a.powi(4) / (j * j) + 4.0 * a * d).sqrt()) / 2.0;
        0.5 * 6.125 * v * v
    };
    assert!((kinetic(1.0) - 2.4228).abs() < 1e-4 && (kinetic(0.5) - 1.2202).abs() < 1e-4);
    let ((back_fast, supply_fast), (back_slow, supply_slow)) = (run(1.0), run(0.5));
    for (back, a) in [(back_fast, 1.0), (back_slow, 0.5)] {
        assert!(
            back < 2.0 * kinetic(a) && back > 0.98 * 2.0 * kinetic(a),
            "{back} J fed back at {a} m/s^2, by hand {} J",
            2.0 * kinetic(a)
        );
    }
    let less = 2.0 * (kinetic(1.0) - kinetic(0.5));
    assert!(
        ((back_fast - back_slow) / less - 1.0).abs() < 0.02,
        "{} J less fed back, by hand {less} J",
        back_fast - back_slow
    );
    assert!(
        ((supply_fast - supply_slow) / less - 1.0).abs() < 0.03,
        "{supply_fast} J and {supply_slow} J from the supply, by hand {less} J less"
    );
}

/// The front of `sim-cli optimise --pareto` on the default config: the settings of the search that pass, and the
/// slower ones of the sweep (the six of them come on top of the 14 cycles of search). No row is beaten by another in
/// cycle time, energy and peak utilisation at once, fastest first; it reaches below the cycle as configured (the
/// sweep) and up to the result.
#[test]
fn every_row_of_the_energy_front_is_beaten_by_no_other() {
    use sim_components::analysis::{OptimiseOptions, optimise_cycle_with};
    let found = optimise_cycle_with(
        &ENTRY,
        &ENTRY.config(),
        &OptimiseOptions { max_runs: 14, robust: false, pareto: true, worst: false },
    )
    .unwrap();
    assert!(found.runs <= 14 + 6 && found.front.len() >= 3, "{} runs, {:?}", found.runs, found.front);
    let figures = |p: &sim_components::analysis::FrontPoint| [p.duration, p.energy, p.utilisation];
    for p in &found.front {
        for q in &found.front {
            let beats = figures(q).iter().zip(figures(p)).all(|(x, y)| *x <= y) && figures(q) != figures(p);
            assert!(!beats, "{q:?} beats {p:?}");
        }
    }
    assert!(found.front.windows(2).all(|w| w[0].duration <= w[1].duration));
    assert!(
        found.front[0].duration == found.after.duration && found.front.last().unwrap().duration > found.before.duration,
        "{:?}",
        found.front
    );
}

/// A drive trace of the axis going to the far end and back, one row per millisecond as a run writes it,
/// with measurement noise on torque (0.005 N m, about 1 % of the peak), speed (0.1 rad/s) and angle.
fn noisy_trace(cfg: &Config, noise: &mut Noise) -> Trace {
    let mut sim = Sim::new(&ENTRY, cfg).unwrap();
    let mut trace = Trace::default();
    for target in [0.9, 0.1] {
        sim.command(sim_components::Command::MoveTo(vec![target])).unwrap();
        for _ in 0..1300 {
            trace.record(&sim);
            sim.run(1e-3, |_| {});
        }
    }
    let m = &mut trace.motors[0];
    for row in 0..m.torque.len() {
        m.torque[row] += 0.005 * noise.next();
        m.speed[row] += 0.1 * noise.next();
        m.angle[row] += 1e-5 * noise.next();
    }
    trace
}

/// Fitting: measurements of a machine that is 1.5 kg heavier than the config, with 1.4 times the Coulomb
/// friction, 1.3 times the viscous friction and 0.7 times the belt stiffness, with noise on them, are
/// fitted against the unchanged config. The fit has to find these within its own error bands.
#[test]
fn the_fit_recovers_changed_masses_friction_and_belt_stiffness_from_noisy_measurements() {
    let coulomb = ["friction_x.coulomb_n", "friction_x.stiction_n", "motor.friction_coulomb_nm"];
    let viscous = ["friction_x.viscous_n_s_m", "motor.friction_viscous_nm_s_rad"];
    let scale = |cfg: &mut Config, paths: &[&str], factor: f64| {
        for path in paths {
            cfg.set_text(&format!("{path} = {:?}", cfg.f64(path).unwrap() * factor)).unwrap();
        }
    };
    let mut truth = ENTRY.config();
    scale(&mut truth, &["mass.carriage_kg", "mass.payload_kg"], 1.25);
    scale(&mut truth, &coulomb, 1.4);
    scale(&mut truth, &viscous, 1.3);
    scale(&mut truth, &["belt.stiffness_ea_n"], 0.7);
    let trace = noisy_trace(&truth, &mut Noise(1));

    // Stage 1, the rigid values. The lumped mass of the config: 6 kg plus pulley, rotor and belt.
    let mut cfg = ENTRY.config();
    let fit = fit_rigid(&ENTRY, &cfg, &trace).unwrap()[0].clone().unwrap();
    let m_total = 6.0 + (1.0e-4 + 25.0 * 0.3e-4 + 0.1 * 2.6 * 0.02 * 0.02) / (0.02 * 0.02);
    assert!((fit.config[0] - m_total).abs() < 0.03 * m_total, "config mass {} kg, by hand {m_total} kg", fit.config[0]);
    let errors = [fit.error[0], fit.error[1] / fit.config[1], fit.error[2] / fit.config[2], fit.error[3]];
    let found = [
        fit.fitted[0] - fit.config[0],
        fit.fitted[1] / fit.config[1],
        fit.fitted[2] / fit.config[2],
        fit.fitted[3] - fit.config[3],
    ];
    // Mass in kg, friction as factors, and no constant force on a level axis.
    for (q, expected) in [1.5, 1.3, 1.4, 0.0].into_iter().enumerate() {
        assert!(
            (found[q] - expected).abs() < 3.0 * errors[q],
            "value {q}: {} ± {}, expected {expected}",
            found[q],
            errors[q]
        );
    }
    // The bands are narrow enough to be of use: 1 % of the mass, 2 % of the friction.
    assert!(errors[0] < 0.01 * fit.fitted[0] && errors[1] < 0.02 && errors[2] < 0.02, "{errors:?}");

    // The changes as the command line advises them, then a second pass. Through the gearbox's losses a
    // wrong friction looks a little like mass and the other way round; the second pass takes that out.
    let apply = |cfg: &mut Config, fit: &sim_components::identify::AxisFit| {
        cfg.set_text(&format!(
            "mass.carriage_kg = {:?}",
            cfg.f64("mass.carriage_kg").unwrap() + fit.fitted[0] - fit.config[0]
        ))
        .unwrap();
        scale(cfg, &viscous, fit.fitted[1] / fit.config[1]);
        scale(cfg, &coulomb, fit.fitted[2] / fit.config[2]);
    };
    apply(&mut cfg, &fit);
    let second = fit_rigid(&ENTRY, &cfg, &trace).unwrap()[0].clone().unwrap();
    apply(&mut cfg, &second);
    let (mass, drag, dry) =
        (cfg.f64("mass.carriage_kg").unwrap() + 2.0, cfg.f64(viscous[0]).unwrap(), cfg.f64(coulomb[0]).unwrap());
    // Two standard errors, as for the friction below: with the winding integrated with the mechanics the second pass
    // lands 0.03 kg off (it was within one before).
    assert!((mass - 7.5).abs() < 2.0 * errors[0], "carriage and payload {mass} kg, in truth 7.5 kg ± {}", errors[0]);
    assert!(
        (drag / 26.0 - 1.0).abs() < 2.0 * errors[1] && (dry / 21.0 - 1.0).abs() < 2.0 * errors[2],
        "viscous {drag} N s/m of 26, Coulomb {dry} N of 21"
    );

    // Stage 2, the stiffness, from a frequency sweep of the measured machine with 2 % noise on its gains,
    // against the sweep of the config with the masses now right. The torque has to be large enough to
    // break the friction loose; the gearbox's play then lowers the dip, in both sweeps alike.
    let frequencies = log_spaced(50.0, 130.0, 40);
    let mut noise = Noise(2);
    let mut measured = frequency_sweep(&ENTRY, &truth, 0, &frequencies, 1.5).unwrap();
    measured.iter_mut().for_each(|p| p.gain *= 1.0 + 0.02 * noise.next());
    let (config, fitted) = (
        resonance(&frequency_sweep(&ENTRY, &cfg, 0, &frequencies, 1.5).unwrap()).unwrap(),
        resonance(&measured).unwrap(),
    );
    // By hand for the config: belt sections of 0.25 m and 2.35 m and the gearbox in series, 7.5 kg on them.
    // The play makes the spring softer than that, so the dip lies lower, but not by much.
    let k = 1.0 / (1.0 / (5.0e5 * (1.0 / 0.25 + 1.0 / 2.35)) + 0.02 * 0.02 / 1.0e4);
    let by_hand = f64::sqrt(k / 7.5) / std::f64::consts::TAU;
    assert!(
        config.frequency < by_hand && config.frequency > 0.85 * by_hand,
        "dip at {} Hz, by hand {by_hand} Hz",
        config.frequency
    );
    let stiffness = (fitted.frequency / config.frequency).powi(2);
    let band = 2.0 * stiffness * fitted.frequency_error / fitted.frequency;
    // The belt is not the only spring: with the gearbox in series, 0.7 on the belt is 0.72 on the whole.
    assert!(
        (stiffness - 0.7).abs() < 3.0 * band && band < 0.03,
        "stiffness factor {stiffness} ± {band}, belt in truth 0.7"
    );
    assert!(fitted.damping > 0.03 && fitted.damping < 0.3, "damping {}", fitted.damping);
}

/// Time-optimal timing against a hand calculation: on a straight line the motor gives its share of the peak
/// torque (0.8 of 0.5 N m) until it turns at its rated speed, and the same to brake. The acceleration a comes
/// and goes at the jerk limit j, which adds a / j: distance / v + v / a + a / j. With a jerk limit of
/// 1e5 m/s^3 that is 0.714 s, 0.16 ms more than without one. With the config's 100 m/s^3 a ramp to
/// v = 1.256 m/s is over before the acceleration reaches the 16.3 m/s^2 the torque allows (that takes a
/// change of speed of a^2 / j = 2.67 m/s): each ramp takes 2 sqrt(v / j) and reaches sqrt(v j) = 11.2 m/s^2,
/// so the line takes distance / v + 2 sqrt(v / j) = 0.861 s.
#[test]
fn a_time_optimal_line_takes_the_hand_calculated_time() {
    let mut cfg = ideal_rigid();
    cfg.set_text("drive.peak_torque_nm = 0.5").unwrap();
    cfg.set_text("axis.x.max_jerk_m_s3 = 1.0e5").unwrap();
    // Carriage plus pulley and rotor inertia, through radius and gear ratio (as in the reference cycle).
    let m_total = 4.0 + (1.0e-4 + 25.0 * 0.3e-4) / (0.02 * 0.02);
    let (a, v, dist) = (0.8 * 0.5 * 5.0 / 0.02 / m_total, 314.0 * 0.02 / 5.0, 0.8);
    let by_hand = dist / v + v / a + a / 1e5;
    let line = [sim_components::path::Piece::Line(vec![0.9])];
    let duration = |cfg: &Config| {
        let points = Sim::new(&ENTRY, cfg).unwrap().plan_path(&line, true).unwrap();
        // To the last bit or the one before it: the end is the start plus the length of the line.
        assert!((points[points.len() - 1].1[0] - 0.9).abs() < 1e-15, "ends at {:?}", points[points.len() - 1].1);
        // The top speed and the hardest acceleration of the plan, by differences.
        let speed = points.windows(2).map(|w| (w[1].1[0] - w[0].1[0]) / 1e-3).fold(0.0, f64::max);
        let acc =
            points.windows(3).map(|w| ((w[2].1[0] - 2.0 * w[1].1[0] + w[0].1[0]) / 1e-6).abs()).fold(0.0, f64::max);
        (points[points.len() - 1].0, speed, acc)
    };
    let (time, top, hardest) = duration(&cfg);
    assert!((time - by_hand).abs() < 1.5e-3, "{time} s, by hand {by_hand} s");
    assert!(
        (top - v).abs() < 1e-3 * v && (hardest - a).abs() < 1e-3 * a,
        "{top} m/s, {hardest} m/s^2, by hand {v} m/s, {a} m/s^2"
    );
    cfg.set_text("axis.x.max_jerk_m_s3 = 100.0").unwrap();
    let (by_hand, gentler) = (dist / v + 2.0 * (v / 100.0f64).sqrt(), (v * 100.0f64).sqrt());
    let (time, top, hardest) = duration(&cfg);
    assert!((time - by_hand).abs() < 1.5e-3, "{time} s, by hand {by_hand} s");
    // The acceleration peaks for an instant, which differences over a cycle of 1 ms round off by 0.3 %.
    assert!(
        (top - v).abs() < 1e-3 * v && (hardest - gentler).abs() < 5e-3 * gentler,
        "{top} m/s, {hardest} m/s^2, by hand {v} m/s, {gentler} m/s^2"
    );
    // The plan within the axis limits (1 m/s, 10 m/s^2) is slower, and an arc needs two axes.
    cfg.set_text("drive.peak_torque_nm = 3.8").unwrap();
    let sim = Sim::new(&ENTRY, &cfg).unwrap();
    let (fast, slow) = (sim.plan_path(&line, true).unwrap(), sim.plan_path(&line, false).unwrap());
    assert!(fast[fast.len() - 1].0 < slow[slow.len() - 1].0);
    let arc = sim_components::path::parse("arc,0.9,0.5,0.5,0.5,cw").unwrap();
    assert_eq!(sim.plan_path(&arc, false).unwrap_err().key, "error-path");
}

/// The rigid single axis at 1e5 m/s^3 with the belt's ratings out of the way (at a hundred m/s^2 the carriage's pull
/// would take the slack side's room).
fn ideal_rigid_quick() -> Config {
    let mut cfg = ideal_rigid();
    for e in
        ["axis.x.max_jerk_m_s3 = 1.0e5", "ratings.belt_min_tension_n = -1.0e6", "ratings.belt_max_tension_n = 1.0e6"]
    {
        cfg.set_text(e).unwrap();
    }
    cfg
}

/// The time-optimal line 0.1 -> 0.9 m on the rigid single axis: the planned duration, the top speed and the
/// acceleration up and down (the hardest of each, by differences over the 1 ms poses).
fn time_optimal_line(cfg: &Config) -> (f64, f64, f64, f64) {
    let points =
        Sim::new(&ENTRY, cfg).unwrap().plan_path(&sim_components::path::parse("line,0.9").unwrap(), true).unwrap();
    assert!((points[points.len() - 1].1[0] - 0.9).abs() < 1e-15, "ends at {:?}", points[points.len() - 1].1);
    let speed = points.windows(2).map(|w| (w[1].1[0] - w[0].1[0]) / 1e-3).fold(0.0, f64::max);
    let acc: Vec<f64> = points.windows(3).map(|w| (w[2].1[0] - 2.0 * w[1].1[0] + w[0].1[0]) / 1e-6).collect();
    (points[points.len() - 1].0, speed, acc.iter().fold(0.0, |m, a| m.max(*a)), acc.iter().fold(0.0, |m, a| m.max(-a)))
}

/// With effects.drive_current_limit the drive's maximum current limits the time-optimal plan where its torque is below
/// the motor's: 2 A through k_t 0.45 N m/A (no saturation) is 0.9 N m against the motor's 3.8 N m. Up a 30 degree
/// incline the carriage's weight along the axis, F = 4 kg * 9.80665 * 0.5 = 19.613 N, takes from the torque on the way
/// up and gives on the way down: a = (0.8 * 0.9 N m * 5 / 0.02 m -+ F) / 6.125 kg = (180 -+ 19.613) / 6.125 = 26.186 and
/// 32.590 m/s^2 (6.125 kg: the carriage, the pulley and the rotor through radius and ratio). The duration is the cruise
/// at the rated speed 1.256 m/s plus half of each ramp: 0.8 / v + (v / a1 + a1 / j) / 2 + (v / a2 + a2 / j) / 2 =
/// 0.68049 s. Without the effect the motor's torque limits it again.
#[test]
fn the_drive_current_limits_the_time_optimal_acceleration_to_the_hand_calculated_one() {
    let mut cfg = ideal_rigid_quick();
    for e in ["effects.torque_saturation = false", "geometry.incline_deg = 30.0", "ratings.drive_max_current_a = 2.0"] {
        cfg.set_text(e).unwrap();
    }
    let (m_total, v, j, weight) =
        (4.0 + (1.0e-4 + 25.0 * 0.3e-4) / (0.02 * 0.02), 314.0 * 0.02 / 5.0, 1e5, 4.0 * G * 0.5);
    let (up, down) =
        ((0.8 * 0.45 * 2.0 * 5.0 / 0.02 - weight) / m_total, (0.8 * 0.45 * 2.0 * 5.0 / 0.02 + weight) / m_total);
    let by_hand = 0.8 / v + 0.5 * (v / up + up / j) + 0.5 * (v / down + down / j);
    assert!((up - 26.186).abs() < 1e-3 && (down - 32.590).abs() < 1e-3 && (by_hand - 0.68049).abs() < 1e-5);
    let (time, top, rising, falling) = time_optimal_line(&cfg);
    assert!((time - by_hand).abs() < 1.5e-3, "{time} s, by hand {by_hand} s");
    assert!(
        (top - v).abs() < 1e-3 * v && (rising - up).abs() < 1e-3 * up && (falling - down).abs() < 1e-3 * down,
        "{top} m/s, {rising} and {falling} m/s^2, by hand {v} m/s, {up} and {down} m/s^2"
    );
    // The motor's own 0.8 * 3.8 N m gives far more without the drive's limit.
    cfg.set_text("effects.drive_current_limit = false").unwrap();
    let (_, _, rising, _) = time_optimal_line(&cfg);
    let motor = (0.8 * 3.8 * 5.0 / 0.02 - weight) / m_total;
    assert!((rising - motor).abs() < 1e-3 * motor, "{rising} m/s^2, by hand {motor} m/s^2");
}

/// The gearbox's peak output torque rating (M2max) and the pulley's torque rating limit the time-optimal plan through
/// the transmission: the output drives the carriage and the pulley, 4 kg * 0.02 m + 1e-4 kg m^2 / 0.02 m = 0.085 N m
/// per m/s^2 (the motor's 0.0245 N m per m/s^2 less the rotor's 0.3e-4 kg m^2 * 250 rad/m, times the ratio 5). With
/// M2max 10 N m: a = 0.8 * 10 / 0.085 = 94.118 m/s^2 (the motor alone allows 0.8 * 3.8 / 0.0245 = 124.1 m/s^2); with the
/// pulley rated 8 N m below it, a = 0.8 * 8 / 0.085 = 75.294 m/s^2. Durations 0.8 / v + v / a + a / j.
#[test]
fn a_gearbox_or_pulley_rating_that_binds_limits_the_time_optimal_acceleration() {
    let mut cfg = ideal_rigid_quick();
    let (v, j) = (314.0 * 0.02 / 5.0, 1e5);
    let per_acc: f64 = 5.0 * (6.125 * 0.02 / 5.0 - 0.3e-4 * 5.0 / 0.02);
    assert!((per_acc - 0.085).abs() < 1e-12);
    for (rating, rated) in [("gearbox_max_torque_nm", 10.0), ("pulley_max_torque_nm", 8.0)] {
        // The single axis's config has no pulley rating of its own (a catalogue axis brings one): added to its text.
        if cfg.set_text(&format!("ratings.{rating} = {rated:.1}")).is_err() {
            let text = cfg.to_text().unwrap().replace(
                "[ratings]
",
                &format!(
                    "[ratings]
{rating} = {rated:.1}
"
                ),
            );
            cfg = Config::parse(&text).unwrap();
        }
        let a = 0.8 * rated / per_acc;
        let by_hand = 0.8 / v + v / a + a / j;
        let (time, _, rising, falling) = time_optimal_line(&cfg);
        assert!((time - by_hand).abs() < 1.5e-3, "{rating}: {time} s, by hand {by_hand} s");
        assert!(
            (rising - a).abs() < 1e-3 * a && (falling - a).abs() < 1e-3 * a,
            "{rating}: {rising} and {falling} m/s^2, by hand {a} m/s^2"
        );
    }
    assert!((0.8 * 10.0 / per_acc - 94.118).abs() < 1e-3 && (0.8 * 8.0 / per_acc - 75.294).abs() < 1e-3);
}

/// A drive on its torque limit fails "motor-peak". With 120 kg of payload, accelerating at 10 m/s^2 takes
/// (124 kg + 2.125 kg of pulley and rotor) * 10 m/s^2 * 0.02 m / 5 = 5.05 N m, which the 3.8 N m motor does
/// not have: the drive sits on its limit and the carriage falls behind. The delivered torque never exceeds
/// the curve it is clamped to, so judged by it the cycle passed with 0.97; the torque the velocity loop
/// asks for shows the 5.05 / 3.8 = 1.33 and more. (Without the simulated DC link: braking 124 kg is more than
/// the placeholder braking resistor takes, and a drive that trips shows nothing of its torque limit.)
#[test]
fn a_cycle_on_the_torque_limit_fails_the_peak_verdict() {
    let mut cfg = ENTRY.config();
    cfg.set_text("mass.payload_kg = 120.0").unwrap();
    cfg.set_text("effects.dc_link = false").unwrap();
    let load = analyse_cycle(&mut Sim::new(&ENTRY, &cfg).unwrap()).unwrap();
    let m = &load.motors[0];
    assert!(
        m.peak_torque < 1.03 * 3.8 && load.tracking_error_peak > 0.01,
        "{} N m, {} m behind",
        m.peak_torque,
        load.tracking_error_peak
    );
    assert!(m.utilisation > 5.05 / 3.8, "utilisation {}", m.utilisation);
    let peak =
        sim_components::analysis::verdicts(&cfg, &load).unwrap().into_iter().find(|v| v.check == "motor-peak").unwrap();
    assert!(!peak.ok && peak.value == m.utilisation);
    // The default cycle asks for what it gets: the two ways of counting agree within the lag of the current loop.
    let load = analyse_cycle(&mut Sim::new(&ENTRY, &ENTRY.config()).unwrap()).unwrap();
    let m = &load.motors[0];
    assert!(
        (m.utilisation - m.peak_torque / 3.8).abs() < 0.05 * m.utilisation,
        "utilisation {}, peak {} N m",
        m.utilisation,
        m.peak_torque
    );
}

/// effects.drive_current_limit: a move that asks more than a 0.5 A drive gives. Rigid model, payload off, the guide's
/// 15 N Coulomb friction the only loss (its viscous part set to 0, its stiction gone above the 0.01 m/s Stribeck speed).
/// Held at the drive's current, the motor gives 0.45 N m/A * 0.5 A = 0.225 N m, through the gearbox (5) and the pulley
/// (0.02 m) 56.25 N, of which friction takes 15 N, on the carriage and the rotor and pulley seen at the belt,
/// 4 + (1e-4 + 25 * 0.3e-4) / 0.02^2 = 6.125 kg: a = (k_t I_max i / r - F_friction) / m_total = 41.25 / 6.125 =
/// 6.7347 m/s^2, where the plan asks for 10. With the motor's peak torque at 0.18 N m, below the drive's 0.225 N m, the
/// motor's limit holds: a = (0.18 * 250 - 15) / 6.125 = 4.8980 m/s^2. Measured over 50 ms in the middle of the
/// acceleration, while the delivered torque sits on the limit. On the shipped cycle a drive of 70 % of the current the
/// cycle needs leaves the carriage further behind and fails "drive-peak-current" with the current asked for.
#[test]
fn a_move_held_at_the_drive_current_limit_accelerates_at_the_hand_calculated_rate() {
    let limited = |edits: &[&str]| {
        let mut cfg = ideal_rigid();
        let base = [
            "effects.drive_current_limit = true",
            "ratings.drive_max_current_a = 0.5",
            "effects.guide_friction = true",
            "friction_x.viscous_n_s_m = 0.0",
            "effects.torque_saturation = false",
        ];
        for e in base.iter().chain(edits) {
            cfg.set_text(e).unwrap();
        }
        cfg
    };
    for (edits, torque, by_hand) in
        [(&[][..], 0.225, 41.25 / 6.125), (&["drive.peak_torque_nm = 0.18"][..], 0.18, 30.0 / 6.125)]
    {
        let mut sim = Sim::new(&ENTRY, &limited(edits)).unwrap();
        sim.command(sim_components::Command::MoveTo(vec![0.9])).unwrap();
        let speed = |sim: &Sim| {
            let (mut pos, mut vel) = ([0.0], [0.0]);
            sim.machine.tool_state(&sim.state, &mut pos, &mut vel);
            vel[0]
        };
        sim.run(0.12, |_| {});
        let before = speed(&sim);
        sim.run(0.05, |sim| {
            assert!(
                (sim.control.drives[0].torque - torque).abs() < 5e-3 * torque,
                "{} N m, not on the limit",
                sim.control.drives[0].torque
            )
        });
        let a = (speed(&sim) - before) / 0.05;
        assert!((a - by_hand).abs() < 0.01 * by_hand, "{a} m/s^2, by hand {by_hand} m/s^2");
    }
    // The shipped cycle against a drive of 70 % of its peak current: the check fails with what the loop asks for, and
    // the carriage falls further behind.
    let free = analyse_cycle(&mut Sim::new(&ENTRY, &ENTRY.config()).unwrap()).unwrap();
    let mut cfg = ENTRY.config();
    cfg.set_text(&format!("ratings.drive_max_current_a = {}", 0.7 * free.motors[0].peak_current)).unwrap();
    let held = analyse_cycle(&mut Sim::new(&ENTRY, &cfg).unwrap()).unwrap();
    let check = sim_components::analysis::verdicts(&cfg, &held)
        .unwrap()
        .into_iter()
        .find(|v| v.check == "drive-peak-current")
        .unwrap();
    assert!(
        !check.ok && check.value > 0.95 * free.motors[0].peak_current,
        "{} A against {} A",
        check.value,
        check.limit
    );
    assert!(
        held.tracking_error_peak > 1.2 * free.tracking_error_peak,
        "{} m behind, {} m without the limit",
        held.tracking_error_peak,
        free.tracking_error_peak
    );
}

/// The gearbox verdicts judge the torque at the gearbox output, not the motor torque times the ratio. On a
/// vertical axis with a gearbox of 0.9 efficiency the two differ both ways.
/// (a) Rigid model, 4 kg at a steady 0.5 m/s. The output holds the weight whichever way it moves:
/// 4 kg * 9.80665 m/s^2 * 0.02 m = 0.7845 N m. Lifting, the motor gives 0.7845 / (5 * 0.9) = 0.1743 N m, and
/// times the ratio that reads 0.8717 N m. Lowering, the load drives the gearbox back, the motor holds back
/// with 0.9 * 0.7845 / 5 = 0.1412 N m, and times the ratio that reads 0.7061 N m, a tenth too little.
/// (b) Elastic model, 6 kg lowered: the gear's spring passes 6 * 9.80665 * 0.02 = 1.1768 N m on to the pulley
/// while the motor holds back with 0.9 of it.
/// (c) The cycle of the vertical axis, 0.8 m up and down at 10 m/s^2: the output's peak is weight plus
/// carriage and pulley, (4 * 9.80665 + 4.25 * 10) N * 0.02 m = 1.6345 N m, reached lifting off and again
/// braking the fall. The motor's peak is the lift-off: 1.6345 / 0.9 through the gearbox and 25 J_rotor / r^2 =
/// 1.875 kg of its own rotor, 0.375 N m, together 2.191 N m at the output side, a third more than is there.
#[test]
fn the_gearbox_verdicts_judge_the_torque_at_the_gearbox_output() {
    let lossy = ["geometry.incline_deg = 90.0", "effects.gearbox_efficiency = true", "gearbox.efficiency = 0.9"];
    let m = rigid(&[&lossy[..], &["effects.payload = false", "effects.belt_mass = false"]].concat());
    let weight = 4.0 * G * 0.02;
    for (speed, motor) in [(0.5, weight / (5.0 * 0.9)), (-0.5, 0.9 * weight / 5.0)] {
        let (state, mut d) = ([0.5, speed, 0.0, 0.0], [0.0; N_RIGID]);
        m.derivatives(&state, &[motor], &mut d);
        let output = signal(&m, &state, motor, "gearbox.m.output_torque");
        assert!(
            d[1].abs() < 1e-9 && (output - weight).abs() < 1e-9 && (weight - 0.7845).abs() < 1e-4,
            "at {speed} m/s: {output} N m, {} m/s^2",
            d[1]
        );
        assert!((5.0 * motor / weight - if speed > 0.0 { 1.0 / 0.9 } else { 0.9 }).abs() < 1e-12);
    }

    let m = machine(
        lossless_config(),
        &[&lossy[..], &["effects.belt_damping = true", "gearbox.damping_nm_s_rad = 0.2"]].concat(),
    );
    let (mut state, mut hold) = (Vec::new(), [0.0]);
    m.equilibrium(&[0.6], &mut state, &mut hold);
    m.set_tool_velocity(&mut state, &[-0.5]);
    simulate(&m, &mut state, 0.4, |_| 0.9 * hold[0], |_, _, _| {});
    let output = signal(&m, &state, 0.9 * hold[0], "gearbox.m.output_torque");
    assert!((state[3] + 0.5).abs() < 2e-3 && (output - 1.1768).abs() < 0.01 * 1.1768, "{} m/s, {output} N m", state[3]);

    let mut cfg = ideal_rigid();
    for e in lossy.iter().chain(&["effects.dc_link = false"]) {
        cfg.set_text(e).unwrap();
    }
    let load = analyse_cycle(&mut Sim::new(&ENTRY, &cfg).unwrap()).unwrap();
    let peak = sim_components::analysis::verdicts(&cfg, &load)
        .unwrap()
        .into_iter()
        .find(|v| v.check == "gearbox-peak")
        .unwrap();
    let (by_hand, motor) = ((4.0 * G + 4.25 * 10.0) * 0.02, 5.0 * load.motors[0].peak_torque);
    assert!((peak.value - by_hand).abs() < 0.02 * by_hand, "{} N m at the output, by hand {by_hand} N m", peak.value);
    assert!((motor - (by_hand / 0.9 + 0.375)).abs() < 0.02 * motor, "{motor} N m by the motor torque");
    println!("output {} N m, by hand {by_hand} N m; motor torque times ratio {motor} N m", peak.value);
}

/// Error bands with an interaction: two values that each pass alone at their bound and fail together.
/// With the payload on, the gearbox output carries (carriage + payload + J_pulley / r^2) r a =
/// 0.2 (m_c + m_p + 0.25) N m at 10 m/s^2: 1.25 N m for 4 kg and 2 kg. The carriage may weigh up to 6 kg and
/// the payload up to 4 kg: either alone at its bound gives 0.2 * 8.25 = 1.65 N m, both together 0.2 * 10.25 =
/// 2.05 N m. A gearbox that allows 1.75 N m passes with each value alone at its bounds and fails as soon as the
/// two together are more than 2.5 kg over. Spread evenly over 2 kg by 2 kg that is the corner triangle with
/// sides of 1.5 kg, 1.125 / 4 = 28 % of the runs: 72 % pass.
#[test]
fn sampled_bands_find_the_two_masses_that_fail_only_together() {
    use sim_components::analysis::{Band, verdict_bands, verdict_bands_sampled};
    let mut cfg = ideal_rigid();
    cfg.set_text("effects.payload = true").unwrap();
    cfg.set_text("ratings.gearbox_max_torque_nm = 1.75").unwrap();
    let masses = [("mass.carriage_kg".to_string(), 4.0, 6.0), ("mass.payload_kg".to_string(), 2.0, 4.0)];
    let peak = |bands: Vec<Band>| bands.into_iter().find(|b| b.verdict.check == "gearbox-peak").unwrap();
    let near = |value: f64, by_hand: f64| (value - by_hand).abs() < 0.02 * by_hand;

    let alone = peak(verdict_bands(&ENTRY, &cfg, &masses).unwrap());
    assert!(
        alone.ok
            && alone.share == 1.0
            && near(alone.verdict.value, 1.25)
            && near(alone.low, 1.25)
            && near(alone.high, 1.65),
        "{alone:?}"
    );

    let together = peak(verdict_bands_sampled(&ENTRY, &cfg, &masses, 40, 1).unwrap());
    assert!(together.verdict.ok && !together.ok, "{together:?}");
    // No run is beyond the corner, and with 40 values spread over each range some come close to it.
    assert!(together.low > 0.98 * 1.25 && together.high < 1.02 * 2.05 && together.high > 1.85, "{together:?}");
    assert!((together.share - 0.72).abs() < 0.1, "{} of the runs pass, by hand 0.72", together.share);
    // The same seed gives the same runs, another seed others.
    assert_eq!(together, peak(verdict_bands_sampled(&ENTRY, &cfg, &masses, 40, 1).unwrap()));
    let other = peak(verdict_bands_sampled(&ENTRY, &cfg, &masses, 40, 2).unwrap());
    assert!(other.high != together.high && !other.ok && (other.share - 0.72).abs() < 0.1, "{other:?}");
    assert_eq!(verdict_bands_sampled(&ENTRY, &cfg, &masses, 0, 1).unwrap_err().detail(), "samples");
    println!(
        "alone {:.4} to {:.4} N m; together {:.4} to {:.4} N m, {:.3} pass; seed 2: {:.4} N m, {:.3} pass",
        alone.low, alone.high, together.low, together.high, together.share, other.high, other.share
    );
}

/// The search for the worst corner of the bounds (`verdict_worst`), against a hand calculation and against samples.
/// On the loss-free rigid axis with the payload on and a gearbox whose efficiency lies between 0.85 and 0.95, the
/// gearbox output carries 0.2 (m_c + m_p + 0.25) N m at 10 m/s^2 (see the test above): `gearbox-peak` is worst
/// with carriage and payload at their high bounds, 0.2 * (6 + 4 + 0.25) = 2.05 N m, whatever the efficiency. The
/// motor gives that through the gearbox, 2.05 / eta, plus its own rotor at the output side, 0.375 N m (see
/// `the_gearbox_verdicts_judge_the_torque_at_the_gearbox_output`), over the ratio 5: `motor-peak` is worst with both
/// masses high and the efficiency LOW, (2.05 / 0.85 + 0.375) / 5 = 0.5574 N m, 0.1467 of the 3.8 N m peak torque
/// (the motor runs at 250 rad/s at most, below the rated speed). A bound on the belt damping does nothing in the
/// rigid model. The search finds both corners, within 2 % of the hand calculation, and no sample of 40 (seeds 1
/// and 2) is worse. The bands it gives back from its first runs (`verdict_worst_and_bands`) are those of
/// `verdict_bands`, to the last bit.
#[test]
fn the_worst_corner_search_finds_the_hand_calculated_corner() {
    use sim_components::analysis::{verdict_bands, verdict_bands_sampled, verdict_worst, verdict_worst_and_bands};
    let mut cfg = ideal_rigid();
    for e in [
        "effects.payload = true",
        "effects.gearbox_efficiency = true",
        "gearbox.efficiency = 0.9",
        "effects.dc_link = false",
    ] {
        cfg.set_text(e).unwrap();
    }
    let bounds = [
        ("mass.carriage_kg".to_string(), 4.0, 6.0),
        ("mass.payload_kg".to_string(), 2.0, 4.0),
        ("gearbox.efficiency".to_string(), 0.85, 0.95),
        ("belt.damping_n_s_m".to_string(), 50.0, 200.0),
    ];
    let (worst, runs) = verdict_worst(&ENTRY, &cfg, &bounds).unwrap();
    let (again, bands, _) = verdict_worst_and_bands(&ENTRY, &cfg, &bounds).unwrap();
    assert!(again == worst && bands == verdict_bands(&ENTRY, &cfg, &bounds).unwrap());
    let find = |check: &str| worst.iter().find(|w| w.verdict.check == check).unwrap().clone();
    let (gearbox, motor) = (find("gearbox-peak"), find("motor-peak"));
    assert!(
        gearbox.high[..2] == [Some(true), Some(true)] && (gearbox.verdict.value / 2.05 - 1.0).abs() < 0.02,
        "{gearbox:?}"
    );
    assert!(
        motor.high[..3] == [Some(true), Some(true), Some(false)]
            && (motor.verdict.value / ((2.05 / 0.85 + 0.375) / 5.0 / 3.8) - 1.0).abs() < 0.02,
        "{motor:?}"
    );
    for seed in [1, 2] {
        let sampled = verdict_bands_sampled(&ENTRY, &cfg, &bounds, 40, seed).unwrap();
        for w in &worst {
            let band = sampled
                .iter()
                .find(|b| (b.verdict.check, b.verdict.motor) == (w.verdict.check, w.verdict.motor))
                .unwrap();
            // A check that has to stay above its limit (the belt's slack, the lives) is worst at its lowest.
            let below = sim_components::analysis::AT_LEAST.contains(&w.verdict.check);
            let sampled_worst = if below { band.low } else { band.high };
            let worse = if below { w.verdict.value <= sampled_worst } else { w.verdict.value >= sampled_worst };
            assert!(
                worse,
                "{}: {} at the corner, {sampled_worst} in a sample (seed {seed})",
                w.verdict.check, w.verdict.value
            );
        }
    }
    println!(
        "{runs} cycles; gearbox-peak {} N m, motor-peak {} at {:?}",
        gearbox.verdict.value, motor.verdict.value, motor.high
    );
}

/// Carriage with payload, pulley and rotor as mass at the belt, without belt mass:
/// 6 kg + (J_pulley + 25 J_rotor) / r^2 = 8.125 kg.
const M_RIGID: f64 = 6.0 + (1.0e-4 + 25.0 * 0.3e-4) / (0.02 * 0.02);

/// Cable chain against a hand calculation at two positions. The chain is fixed at x = 0, so 0.3 m + x / 2
/// of it moves with the carriage: 0.5 kg/m * 0.35 m = 0.175 kg at x = 0.1 m, 0.5 * 0.75 = 0.375 kg at 0.9 m.
/// Accelerating at 8 m/s2 takes 1.4 N and 3.0 N more. At 0.5 m/s the chain drags with 3 N, and picking up
/// 0.25 kg per metre takes another 0.25 * 0.5^2 / 2 N. On an axis inclined by 30 degrees the motor also holds
/// half the weight of the moving run: 0.175 kg * 9.80665 m/s^2 / 2 = 0.858 N at x = 0.1 m, 1.839 N at 0.9 m.
#[test]
fn cable_chain_adds_the_hand_calculated_force_at_two_positions() {
    let chained = ["effects.cable_chain = true", "cable_chain.drag_n = 3.0", "effects.belt_mass = false"];
    let (m, plain) = (rigid(&chained), rigid(&["effects.belt_mass = false"]));
    for (x, chain) in [(0.1, 0.175), (0.9, 0.375)] {
        let (mut with, mut without) = ([0.0], [0.0]);
        m.feedforward_torque(&[x], &[8.0], &mut with);
        plain.feedforward_torque(&[x], &[8.0], &mut without);
        // Force to motor torque: r / i.
        let extra_force = (with[0] - without[0]) * 5.0 / 0.02;
        assert!((extra_force - chain * 8.0).abs() < 1e-9, "x = {x}: {extra_force} N");
        // The model answers that torque with that acceleration.
        let mut d = [0.0; N_RIGID];
        m.derivatives(&[x, 0.0, 0.0, 0.0], &with, &mut d);
        assert!((d[1] - 8.0).abs() < 1e-9, "x = {x}: {d:?}");
        // Coasting, the drag and the chain it picks up brake.
        m.derivatives(&[x, 0.5, 0.0, 0.0], &[0.0], &mut d);
        assert!((d[1] + (3.0 + 0.25 * 0.25 / 2.0) / (M_RIGID + chain)).abs() < 1e-9, "x = {x}: {}", d[1]);

        // The elastic model: the carriage alone (6 kg with payload) carries the chain.
        let elastic = machine(lossless_config(), &chained);
        let (mut state, mut hold, mut d) = (Vec::new(), [0.0], [0.0; N_ELASTIC]);
        elastic.equilibrium(&[x], &mut state, &mut hold);
        state[3] = 0.5;
        elastic.derivatives(&state, &hold, &mut d);
        assert!((d[3] + (3.0 + 0.25 * 0.25 / 2.0) / (6.0 + chain)).abs() < 1e-6, "x = {x}: {}", d[3]);

        // Inclined by 30 degrees, both models hold the moving run's weight along the axis as well.
        let incline = [&chained[..], &["geometry.incline_deg = 30.0"]].concat();
        let extra = chain * G * 0.5 * 0.02 / 5.0;
        for (model, tolerance) in [(rigid(&incline), 1e-12), (machine(lossless_config(), &incline), 2e-6)] {
            let bare = machine(
                lossless_config(),
                &[
                    "geometry.incline_deg = 30.0",
                    "effects.belt_mass = false",
                    if model.elastic { "effects.belt_stretch = true" } else { "effects.belt_stretch = false" },
                ],
            );
            let (mut with, mut without) = ([0.0], [0.0]);
            model.equilibrium(&[x], &mut state, &mut with);
            bare.equilibrium(&[x], &mut state, &mut without);
            assert!(
                (with[0] - without[0] - extra).abs() < tolerance,
                "x = {x}: {} N m more to hold, by hand {extra} N m",
                with[0] - without[0]
            );
        }
    }
}

/// Process force against a hand calculation. While the cycle dwells the carriage is pushed with 10 N towards
/// +x. The motor holds against it with r / i * F = 0.004 m * 10 N = 0.04 N m less. Friction would take a share
/// of a force this small, so it is off.
#[test]
fn process_force_changes_the_holding_torque_by_the_hand_calculated_amount() {
    let dwell_torque = |process: bool| {
        let mut cfg = Config::parse(&ENTRY.default_config.replace("dwell_s = 0.1", "dwell_s = 1.0")).unwrap();
        for effect in ["guide_friction", "motor_friction", "gearbox_efficiency", "torque_ripple", "tooth_meshing"] {
            cfg.set_text(&format!("effects.{effect} = false")).unwrap();
        }
        cfg.set_text(&format!("effects.process_force = {process}")).unwrap();
        cfg.set_text("process.force_x_n = 10.0").unwrap();
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        let index = sim.signal_index("motor.m.torque").unwrap();
        sim.command(sim_components::Command::StartCycle).unwrap();
        // Mean torque over the second half of the first dwell.
        let (mut sum, mut n, mut sample) = (0.0, 0.0, Vec::new());
        loop {
            sim.step();
            match sim.control.dwell_progress() {
                Some(done) if done > 0.5 => {
                    sim.sample(&mut sample);
                    (sum, n) = (sum + sample[index], n + 1.0);
                }
                None if n > 0.0 => return sum / n,
                _ => assert!(sim.time() < 10.0, "the cycle never dwells"),
            }
        }
    };
    let (with, without) = (dwell_torque(true), dwell_torque(false));
    assert!((with - without + 0.04).abs() < 0.02 * 0.04, "{} N m", with - without);
}

/// Holding brake against a hand calculation, on the axis set upright. Disabled, the motor has to hold
/// m g r / i = 6 * 9.80665 * 0.02 / 5 = 0.2354 N m. A brake of 0.26 N m holds the carriage. One of 0.2 N m slips
/// with its torque: it carries 0.2 * 5 / 0.02 = 50 N of the 58.84 N weight, and the rest accelerates carriage,
/// pulley and rotor, a = 8.84 N / 8.125 kg = 1.088 m/s2, so it falls 136 mm in half a second. The same in the elastic
/// and in the rigid model (`effects.belt_stretch = false`), where the brake acts on the rotor as it does in the elastic one.
#[test]
fn the_holding_brake_holds_the_upright_axis_when_it_is_strong_enough() {
    use sim_components::Command;
    for elastic in [true, false] {
        let disabled = |brake_torque: f64| {
            let mut cfg = lossless_config();
            for e in [
                "effects.holding_brake = true",
                "effects.belt_mass = false",
                "geometry.incline_deg = 90.0",
                "start.pose = [0.5]",
                &format!("motor.brake_torque_nm = {brake_torque:?}"),
                &format!("effects.belt_stretch = {elastic}"),
            ] {
                cfg.set_text(e).unwrap();
            }
            let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
            sim.command(Command::Disable).unwrap();
            sim.run(0.5, |_| {});
            sim
        };
        let mut held = disabled(0.26);
        assert!(
            (held.state[0] - 0.5).abs() < 1e-4 && held.torque[0] == 0.0,
            "elastic {elastic}: held at {} m",
            held.state[0]
        );
        // Nothing moves a disabled machine; enabled again, the drive holds where it stands.
        assert_eq!(held.command(Command::MoveTo(vec![0.6])).unwrap_err().key, "error-disabled");
        held.command(Command::Enable).unwrap();
        held.run(0.3, |_| {});
        assert!((held.state[0] - 0.5).abs() < 1e-4 && held.command(Command::MoveTo(vec![0.6])).is_ok());

        let fell = 0.5 - disabled(0.2).state[0];
        let by_hand = 0.5 * (6.0 * G - 0.2 * 5.0 / 0.02) / M_RIGID * 0.5 * 0.5;
        assert!((fell - by_hand).abs() < 0.03 * by_hand, "elastic {elastic}: fell {fell} m, by hand {by_hand} m");
        println!("elastic {elastic}: fell {fell} m, by hand {by_hand} m");
    }
}

/// The mapping from a tool pose into the scene, for drawing paths there: it is where the scene draws the
/// carriage. By hand at x = 0.3 m: the beam starts 0.15 m left of x = 0, so 0.45 m across, 0.03 m in front of
/// the beam and 0.06 m above its middle. Moving the carriage moves the point by as much.
#[test]
fn the_scene_point_of_a_tool_pose_is_where_the_scene_draws_the_carriage() {
    for m in [rigid(&[]), machine(ENTRY.config(), &[])] {
        assert_eq!(m.scene_point(&[0.3]).unwrap().map(|v| (v * 1e12).round() / 1e12), [0.45, -0.03, 0.06]);
        for x in [0.0, 0.1, 0.3, 0.5, 1.0] {
            let (mut state, mut scene) = (Vec::new(), Scene::default());
            m.equilibrium(&[x], &mut state, &mut [0.0]);
            m.scene(&state, &mut scene);
            let carriage = scene.parts.iter().find(|p| p.id == "carriage").unwrap().pos;
            let (mut tool, mut speed) = ([0.0], [0.0]);
            m.tool_state(&state, &mut tool, &mut speed);
            let mapped = m.scene_point(&tool).unwrap();
            assert!(
                (0..3).all(|i| (mapped[i] - carriage[i]).abs() < 1e-12) && (mapped[0] - 0.15 - x).abs() < 1e-12,
                "at {x}: mapped {mapped:?}, drawn {carriage:?}"
            );
        }
    }
}

/// The vertical axis held, by hand: m g r / i = 6 * 9.80665 * 0.02 / 5 = 0.235360 N m is the gravity part, and at
/// rest it is the motor torque to within 0.2 %; inertia, friction and the rest (the controller dithering on the encoder
/// counts, the guide holding a little) stay below 0.5 % of it. Ripple, cogging
/// and tooth meshing are off: they are part of the rest, and at the start pose they take 0.7 % of the torque.
#[test]
fn holding_the_vertical_axis_is_all_gravity() {
    let by_hand = 6.0 * G * 0.02 / 5.0;
    assert!((by_hand - 0.235360).abs() < 1e-6);
    let mut cfg = ENTRY.config();
    for edit in ["geometry.incline_deg = 90.0", "effects.torque_ripple = false", "effects.tooth_meshing = false"] {
        cfg.set_text(edit).unwrap();
    }
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    sim.run(0.2, |_| {});
    let part = |p: &str| {
        let mut values = Vec::new();
        sim.machine.signals(&sim.state, &sim.torque, &mut values);
        values[sim.machine.signal_infos().iter().position(|s| s.id == format!("motor.m.torque_{p}")).unwrap()]
    };
    assert!((part("gravity") - by_hand).abs() < 1e-12, "{} N m", part("gravity"));
    assert!((sim.torque[0] - by_hand).abs() < 2e-3 * by_hand, "torque {} N m", sim.torque[0]);
    let (inertia, friction, rest) = (part("inertia"), part("friction"), part("rest"));
    assert!(
        inertia.abs() < 5e-3 * by_hand && friction.abs() < 5e-3 * by_hand && rest.abs() < 5e-3 * by_hand,
        "{inertia}, {friction}, {rest}"
    );
}

/// The friction view's curve is the formula with the config's values (Fc 15 N, Fs 20 N, b 20 N s/m, vs 0.01 m/s):
/// at -0.01 m/s -15, -0.2 and -5 / e = -1.83940 N; at 1 m/s with the levels at 1.5 times 22.5, 20 and 0 N.
#[test]
fn the_friction_curve_is_the_formula() {
    let m = machine(ENTRY.config(), &[]);
    assert_eq!(m.friction_contacts().len(), 1);
    for (speed, scale, expected) in [(-0.01, 1.0, [-15.0, -0.2, -1.83940]), (1.0, 1.5, [22.5, 20.0, 0.0])] {
        let curve = m.friction_curve(0, speed, scale);
        assert!(curve.iter().zip(&expected).all(|(a, b)| (a - b).abs() < 1e-5), "{speed} m/s: {curve:?}");
    }
    assert_eq!(m.friction_curve(1, 0.5, 1.0), [0.0; 3]);
}

/// The carriage on the belt with the pulley held (`modes`): m = 4 + 2 kg, at x = 0.5 m the sections are 0.65 m and
/// 1.95 m long, k = 5e5 / 0.65 + 5e5 / 1.95 = 1.025641e6 N/m, f = sqrt(k / m) / (2 pi) = 65.8025 Hz; at x = 0.1 m,
/// 0.25 m and 2.35 m, k = 2.212766e6 N/m, 96.6523 Hz. Damping ratio 2 * 100 N s/m / (2 m w): 0.040311 and 0.027445.
/// The rigid model tells none.
#[test]
fn the_carriage_mode_is_the_hand_calculated_one_at_two_places() {
    let m = machine(ENTRY.config(), &[]);
    for (x, hz, damping) in [(0.5, 65.8025, 0.040311), (0.1, 96.6523, 0.027445)] {
        let modes = m.modes(&[x]);
        assert!(
            modes.len() == 1 && (modes[0].hz - hz).abs() < 1e-3 && (modes[0].damping - damping).abs() < 1e-5,
            "x = {x}: {modes:?}"
        );
        assert_eq!(m.resonance_hz(&[x]), Some(modes[0].hz));
    }
    assert!(rigid(&[]).modes(&[0.5]).is_empty());
    // The clamp holds the belt at the pulley's height, the idler at the far end is as large as the pulley: half a turn.
    let wraps = machine(ENTRY.config(), &["effects.belt_wrap = true"]).belt_wraps(&[0.3]);
    assert!(wraps.len() == 1 && (wraps[0] - std::f64::consts::PI).abs() < 1e-12, "{wraps:?}");
}

/// The axis on a soft base (effects.base_compliance), along x only and undamped: 100 kg on 2.5e6 N/m. After a move
/// the axis settles, but the base rings on at f = sqrt(k / M) / 2 pi = sqrt(25000) / 2 pi = 25.165 Hz, and the tool
/// with it: it crosses its mean every half period, pi sqrt(100 / 2.5e6) = 19.869 ms.
#[test]
fn a_move_on_a_soft_base_leaves_the_tool_ringing_at_the_base_s_natural_frequency() {
    let mut cfg = ENTRY.config();
    for edit in ["effects.base_compliance = true", "base.damping_ratio_x = 0.0"] {
        cfg.set_text(edit).unwrap();
    }
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    sim.command(sim_components::Command::MoveTo(vec![0.5])).unwrap();
    sim.run(1.5, |_| {});
    let (x, base) = (sim.signal_index("axis.x.position").unwrap(), sim.signal_index("base.x").unwrap());
    let mut trace = Vec::new();
    let mut sample = Vec::new();
    sim.run(1.0, |s| {
        s.sample(&mut sample);
        trace.push((s.time(), sample[x], sample[base]));
    });
    let peak = trace.iter().map(|(.., b)| b.abs()).fold(0.0, f64::max);
    assert!(peak > 1e-6, "the base hardly moved: {peak} m");
    let mean = trace.iter().map(|(_, p, _)| p).sum::<f64>() / trace.len() as f64;
    let crossings: Vec<f64> = trace
        .windows(2)
        .filter(|w| (w[0].1 - mean).signum() != (w[1].1 - mean).signum())
        .map(|w| w[0].0 + (w[1].0 - w[0].0) * (w[0].1 - mean) / (w[0].1 - w[1].1))
        .collect();
    let half = (crossings[crossings.len() - 1] - crossings[0]) / (crossings.len() - 1) as f64;
    assert!((half / 0.019_869_18 - 1.0).abs() < 0.005, "half period {half} s from {} crossings", crossings.len());
}

/// A force held at rest moves the base, and the tool on it, by force over stiffness: the rigid axis on a base of
/// 2.5e6 N/m along x holds a process force of 50 N while it dwells; its motor's stator takes the 50 N, the base gives
/// way 50 / 2.5e6 = 20 um once its sway has died down (damping ratio 0.3 at 25 Hz: 2 s take it down by
/// exp(-0.3 x 157 x 2), nothing), and the tool, which the motor holds on the base, moves with it.
#[test]
fn a_held_force_moves_the_base_and_the_tool_by_force_over_stiffness() {
    let mut cfg = ENTRY.config();
    for edit in [
        "effects.base_compliance = true",
        "base.damping_ratio_x = 0.3",
        "effects.belt_stretch = false",
        "effects.process_force = true",
        "process.force_x_n = 50.0",
    ] {
        cfg.set_text(edit).unwrap();
    }
    let mut m = (ENTRY.build)(&cfg).unwrap();
    let (mut state, mut torque, mut held) = (Vec::new(), [0.0], [0.0]);
    m.equilibrium(&[0.3], &mut state, &mut torque);
    let at = m.signal_infos().iter().position(|s| s.id == "base.x").unwrap();
    m.set_dwelling(true);
    m.equilibrium(&[0.3], &mut Vec::new(), &mut held);
    let mut rk = Rk4::new(state.len());
    for _ in 0..20_000 {
        advance(m.as_ref(), &mut rk, &mut state, &held, 1e-4, 1);
    }
    let mut values = Vec::new();
    m.signals(&state, &held, &mut values);
    assert!((values[at] - 2.0e-5).abs() < 1e-9, "the base gave way {} m", values[at]);
    let (mut pos, mut vel) = ([0.0], [0.0]);
    m.tool_state(&state, &mut pos, &mut vel);
    assert!((pos[0] - 0.3 - 2.0e-5).abs() < 1e-8, "the tool moved {} m", pos[0] - 0.3);
}
