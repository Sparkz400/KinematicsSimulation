//! The D3's rigid, loss-free dynamics against an independent reference: the model of docs/model-reference.md written
//! again in Python as a Lagrangian in the arm angles and the platform's position, differentiated by SymPy, with the
//! legs' constraints through Lagrange multipliers (`reference/lagrange_reference.py`). The crate eliminates the
//! platform through the Jacobian instead. Both the lumped forearms (Codourey/Olsson) and the forearms as rods.

use machine_delta_d3::{Delta, lossless_config};
use sim_core::testkit::DT;
use sim_core::{Config, Machine, Rk4, advance};

/// The rows of a committed reference CSV, comments and the header left out.
fn rows(text: &str) -> Vec<Vec<f64>> {
    text.lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with(|c: char| c.is_ascii_alphabetic()))
        .map(|l| l.split(',').map(|v| v.trim().parse().unwrap()).collect())
        .collect()
}

/// The rigid, loss-free model: the crate's `lossless_config` with rigid gearboxes, the forearms as `rods` says.
fn rigid(rods: bool) -> Config {
    let mut cfg = lossless_config();
    cfg.set_text("effects.joint_elasticity = false").unwrap();
    cfg.set_text(&format!("effects.forearm_inertia = {rods}")).unwrap();
    cfg
}

/// Motor torques of the rigid model (`feedforward_torque_moving`) at 60 random poses, speeds and accelerations in the
/// working range (up to the axes' 3 m/s and 50 m/s^2) against the Lagrangian's, to 1e-8 of the largest torque at
/// each state.
#[test]
fn delta_inverse_dynamics_matches_the_lagrangian() {
    let reference = rows(include_str!("../reference/inverse_dynamics.csv"));
    assert_eq!(reference.len(), 60);
    for (rods, column) in [(false, 9), (true, 12)] {
        let d = Delta::new(&rigid(rods)).unwrap();
        for row in &reference {
            assert!(d.unreachable(&row[..3], 0.0).is_none(), "{:?} is outside the working range", &row[..3]);
            let mut ours = [0.0; 3];
            d.feedforward_torque_moving(&row[..3], &row[3..6], &row[6..9], &mut ours);
            let theirs = &row[column..column + 3];
            let scale = theirs.iter().fold(0.0f64, |m, t| m.max(t.abs()));
            for k in 0..3 {
                sim_core::testkit::record_check(
                    &format!("rods {rods}, motor {k} torque, N m"),
                    theirs[k],
                    ours[k],
                    1e-8 * scale,
                );
                assert!(
                    (ours[k] - theirs[k]).abs() <= 1e-8 * scale,
                    "rods {rods}, state {:?}: motor {k} {} N m against {} N m",
                    &row[..9],
                    ours[k],
                    theirs[k]
                );
            }
        }
    }
}

/// The shipped model (rods) under the reference's motor torques: its holding torques at the start, plus its push
/// that accelerates the platform at (5, 3, -4) m/s^2 there, as a sin^2 pulse of 0.15 s and back at 0.25 s. The
/// simulator's RK4 at its step against SciPy's DOP853 at rtol 1e-10, to 0.2 % of the travel in each axis, every
/// millisecond. The torques are the reference's, not the crate's feedforward: a model wrong by a factor would
/// otherwise push itself just as far.
#[test]
fn open_loop_move_matches_scipy() {
    let m = Delta::new(&rigid(true)).unwrap();
    let start = [0.05, 0.02, -0.92];
    let mut state = Vec::new();
    m.equilibrium(&start, &mut state, &mut [0.0; 3]);
    let torques = &rows(include_str!("../reference/open_loop_torques.csv"))[0];
    let (hold, push) = (&torques[..3], &torques[3..]);
    let pulse = |t: f64| {
        let bump = |t0: f64| {
            if t > t0 && t < t0 + 0.15 { (std::f64::consts::PI * (t - t0) / 0.15).sin().powi(2) } else { 0.0 }
        };
        bump(0.0) - bump(0.25)
    };

    let mut rk = Rk4::new(m.state_len());
    let mut trace = vec![start];
    let (mut pos, mut vel) = ([0.0; 3], [0.0; 3]);
    for step in 0..(0.5 / DT).round() as usize {
        let p = pulse((step as f64 + 0.5) * DT);
        let torque: [f64; 3] = std::array::from_fn(|k| hold[k] + push[k] * p);
        advance(&m, &mut rk, &mut state, &torque, DT, 4);
        if (step + 1) % 16 == 0 {
            m.tool_state(&state, &mut pos, &mut vel);
            trace.push(pos);
        }
    }

    let reference = rows(include_str!("../reference/open_loop_move.csv"));
    assert_eq!(reference.len(), trace.len(), "the reference trace has a different length");
    let travel: [f64; 3] =
        std::array::from_fn(|a| reference.iter().map(|r| (r[a + 1] - start[a]).abs()).fold(0.0, f64::max));
    assert!(travel.iter().all(|&t| t > 0.01), "the reference move is too small: {travel:?}");
    for (i, (ours, theirs)) in trace.iter().zip(&reference).enumerate() {
        assert!((i as f64 * 1e-3 - theirs[0]).abs() < 1e-9);
        for a in 0..3 {
            sim_core::testkit::record_check(
                &format!("axis {a} position, m"),
                theirs[a + 1],
                ours[a],
                0.002 * travel[a],
            );
            assert!(
                (ours[a] - theirs[a + 1]).abs() < 0.002 * travel[a],
                "axis {a} at t={}: {} against SciPy {}",
                theirs[0],
                ours[a],
                theirs[a + 1]
            );
        }
    }
}
