//! Machine-specific checks, and the component checks that the plan runs once on this machine:
//! friction levels, backlash, belt natural frequency.

use super::*;
use sim_core::testkit::{DT, PulseRun};
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
