//! Machine-specific checks: kinematics, static hold on Z, constant torque, rigid limit,
//! and the comparison against the independent SciPy implementation.

use super::*;
use sim_core::testkit::{DT, PulseRun};
use sim_core::{Rk4, advance};

fn machine(mut cfg: Config, edit: &[&str]) -> TGantry {
    for e in edit {
        cfg.set_text(e).unwrap();
    }
    TGantry::new(&cfg).unwrap()
}

fn rigid(edit: &[&str]) -> TGantry {
    let mut all = vec!["effects.belt_stretch = false"];
    all.extend(edit);
    machine(lossless_config(), &all)
}

#[test]
fn kinematics_follow_the_plan() {
    let m = rigid(&["gearbox.ratio = 1.0"]);
    let mut tool = [0.0; 2];
    // Both motors the same way: X only. Against each other: Z only.
    m.forward(&[1.0, 1.0], &mut tool);
    assert!((tool[0] - m.r).abs() < 1e-15 && tool[1].abs() < 1e-15);
    m.forward(&[1.0, -1.0], &mut tool);
    assert!(tool[0].abs() < 1e-15 && (tool[1] - m.r).abs() < 1e-15);
}

/// Static hold on Z: each motor holds m_z g r / 2 at the pulley, with opposite signs.
#[test]
fn static_hold_on_z_needs_opposite_torques() {
    let expected = (5.0 + 2.0) * G * 0.02 / 2.0 / 5.0;
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    let m = rigid(&[]);
    m.equilibrium(&[0.5, 0.25], &mut state, &mut hold);
    assert!((hold[0] - expected).abs() < 1e-12 && (hold[1] + expected).abs() < 1e-12, "{hold:?}");
    let mut d = [0.0; N_RIGID];
    m.derivatives(&state, &hold, &mut d);
    assert!(d.iter().all(|v| v.abs() < 1e-12), "{d:?}");

    // The elastic model agrees to within the belt strain (see Belt::section) and is at rest too.
    let m = machine(lossless_config(), &[]);
    m.equilibrium(&[0.5, 0.25], &mut state, &mut hold);
    assert!((hold[0] - expected).abs() < 1e-3 * expected && (hold[1] + expected).abs() < 1e-3 * expected, "{hold:?}");
    let mut d = [0.0; N_ELASTIC];
    m.derivatives(&state, &hold, &mut d);
    assert!(d.iter().all(|v| v.abs() < 1e-6), "{d:?}");
    // Belt tensions at rest: sections 1 and 3 equal, section 2 higher by m_z g / 2.
    let t = m.sections(&state).map(|f| f.tension);
    assert!((t[0] - t[2]).abs() < 1e-6 && (t[1] - t[0] - 7.0 * G / 2.0).abs() < 1e-6, "{t:?}");
}

/// Constant torque: acceleration equals force divided by effective mass.
#[test]
fn constant_torque_gives_force_over_effective_mass() {
    let m = rigid(&["effects.belt_mass = false"]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&[0.1, 0.1], &mut state, &mut hold);
    let mut rk = Rk4::new(N_RIGID);
    // Extra torque: 0.1 N m on both for X, and +-0.05 N m for Z on top.
    let torque = [hold[0] + 0.1 + 0.05, hold[1] + 0.1 - 0.05];
    let t_end = 0.25;
    for _ in 0..(t_end / DT) as usize {
        advance(&m, &mut rk, &mut state, &torque, DT, 4);
    }
    let m_motors = 2.0 * (1.0e-4 + 25.0 * 0.3e-4) / (0.02 * 0.02);
    let ax = (2.0 * 0.1 * 5.0 / 0.02) / (11.0 + m_motors);
    let az = (2.0 * 0.05 * 5.0 / 0.02) / (7.0 + m_motors);
    assert!((state[0] - 0.1 - 0.5 * ax * t_end * t_end).abs() < 1e-9, "x {}", state[0]);
    assert!((state[1] - 0.1 - 0.5 * az * t_end * t_end).abs() < 1e-9, "z {}", state[1]);
    assert!((state[2] - ax * t_end).abs() < 1e-9 && (state[3] - az * t_end).abs() < 1e-9);
}

#[test]
fn payload_switch_and_release_change_the_load() {
    let mut m = rigid(&[]);
    assert_eq!(m.m_z(), 7.0);
    m.set_payload_attached(false);
    assert_eq!(m.m_z(), 5.0);
    assert_eq!(rigid(&["effects.payload = false"]).m_z(), 5.0);
}

#[test]
fn belt_length_does_not_depend_on_the_pose() {
    let m = rigid(&[]);
    let total = |x, z| m.geo(x, z).iter().sum::<f64>();
    assert!((total(0.0, 0.0) - total(1.0, 0.5)).abs() < 1e-12);
    assert!((total(0.0, 0.5) - total(1.0, 0.0)).abs() < 1e-12);
}

#[test]
fn guide_friction_grows_with_acceleration_and_overhang() {
    let m = machine(ENTRY.config(), &[]);
    let (x_rest, z_rest) = m.load_scales(0.25, 0.0);
    let (x_acc, z_acc) = m.load_scales(0.25, 8.0);
    // At rest the X guide carries the payload on top of the nominal weight; the Z guide only preload.
    assert!((x_rest - (200.0 + 11.0 * G) / (200.0 + 9.0 * G)).abs() < 1e-12);
    assert_eq!(z_rest, 1.0);
    assert!(x_acc > x_rest && z_acc > 1.0);
    // Less overhang (z raised to the centre of mass) means less tilting moment.
    assert!(m.load_scales(0.3, 8.0).0 < x_acc);
}

/// Rigid limit: with very high belt stiffness the full model matches the rigid reference within 1 %.
#[test]
fn stiff_belt_converges_to_the_rigid_reference() {
    let travel = |edit: &[&str], substeps: u32| {
        let mut all = vec!["effects.gearbox_backlash = false", "effects.frame_compliance = false"];
        all.extend(edit);
        let m = machine(ENTRY.config(), &all);
        let mut run = PulseRun::new(&m, DT, 1.0, 0.2);
        run.substeps = substeps;
        let start = run.tool_pos();
        run.run(1.0, |_| {});
        let end = run.tool_pos();
        [end[0] - start[0], end[1] - start[1]]
    };
    let rigid = travel(&["effects.belt_stretch = false"], 4);
    let stiff = travel(&["belt.stiffness_ea_n = 5.0e7", "gearbox.stiffness_nm_rad = 1.0e6"], 16);
    let soft = travel(&[], 4);
    for axis in 0..2 {
        assert!(rigid[axis] > 0.02, "the reference move is too small: {rigid:?}");
        assert!((stiff[axis] - rigid[axis]).abs() < 0.01 * rigid[axis], "stiff {stiff:?}, rigid {rigid:?}");
    }
    // The default belt is visibly softer, otherwise the check above would prove nothing.
    let gap = |a: [f64; 2]| (a[0] - rigid[0]).abs() + (a[1] - rigid[1]).abs();
    assert!(gap(soft) > gap(stiff), "soft {soft:?}, stiff {stiff:?}, rigid {rigid:?}");
}

/// The torque pattern of the reference move: a push, a pause, a pull back.
fn reference_pulse(t: f64) -> f64 {
    let bump = |t0: f64| if t > t0 && t < t0 + 0.15 { (std::f64::consts::PI * (t - t0) / 0.15).sin().powi(2) } else { 0.0 };
    bump(0.0) - bump(0.3)
}

/// Independent reference: one move matches a SciPy implementation of the same equations.
/// The trace comes from `reference/scipy_reference.py`, which reads the same default config.
#[test]
fn reference_move_matches_scipy() {
    let m = machine(ENTRY.config(), &[]);
    let pose = [0.5, 0.25];
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&pose, &mut state, &mut hold);
    let (mut with, mut without) = ([0.0; 2], [0.0; 2]);
    m.feedforward_torque(&pose, &[m.axes[0].max_acc, m.axes[1].max_acc], &mut with);
    m.feedforward_torque(&pose, &[0.0; 2], &mut without);

    let mut rk = Rk4::new(N_ELASTIC);
    let mut trace = vec![(0.0, pose[0], pose[1])];
    for step in 0..(0.6 / DT).round() as usize {
        let p = reference_pulse((step as f64 + 0.5) * DT);
        let torque = [hold[0] + (with[0] - without[0]) * p, hold[1] + (with[1] - without[1]) * p];
        advance(&m, &mut rk, &mut state, &torque, DT, 4);
        if (step + 1) % 16 == 0 {
            trace.push(((step + 1) as f64 * DT, state[X], state[Z]));
        }
    }

    let reference: Vec<Vec<f64>> = include_str!("../reference/scipy_trace.csv")
        .lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with('t'))
        .map(|l| l.split(',').map(|v| v.trim().parse().unwrap()).collect())
        .collect();
    assert_eq!(reference.len(), trace.len(), "the reference trace has a different length");
    let travel = |i: usize| reference.iter().map(|r| (r[i] - pose[i - 1]).abs()).fold(0.0, f64::max);
    let (travel_x, travel_z) = (travel(1), travel(2));
    assert!(travel_x > 0.02 && travel_z > 0.01, "the reference move is too small");
    for (ours, theirs) in trace.iter().zip(&reference) {
        assert!((ours.0 - theirs[0]).abs() < 1e-9);
        assert!((ours.1 - theirs[1]).abs() < 0.002 * travel_x, "x at t={}: {} vs SciPy {}", ours.0, ours.1, theirs[1]);
        assert!((ours.2 - theirs[2]).abs() < 0.002 * travel_z, "z at t={}: {} vs SciPy {}", ours.0, ours.2, theirs[2]);
    }
}
