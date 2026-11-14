//! The D2's rigid, loss-free inverse dynamics against an independent reference: the model of
//! docs/model-reference.md written again in Python as a Lagrangian in the arm angles and the plate's position, with
//! the legs' constraints through Lagrange multipliers (`reference/lagrange_reference.py`).

use machine_delta_d2::{Delta2, lossless_config};
use sim_core::Machine;

/// Motor torques of the rigid model (`feedforward_torque_moving`) at 60 random poses, speeds and accelerations in the
/// working range (up to the axes' limits) against the Lagrangian's, to 1e-8 of the largest torque at each state.
#[test]
fn d2_inverse_dynamics_matches_the_lagrangian() {
    let mut cfg = lossless_config();
    cfg.set_text("effects.joint_elasticity = false").unwrap();
    let d = Delta2::new(&cfg).unwrap();
    let reference: Vec<Vec<f64>> = include_str!("../reference/inverse_dynamics.csv")
        .lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with('x'))
        .map(|l| l.split(',').map(|v| v.trim().parse().unwrap()).collect())
        .collect();
    assert_eq!(reference.len(), 60);
    for row in &reference {
        assert!(d.unreachable(&row[..2], 0.0).is_none(), "{:?} is outside the working range", &row[..2]);
        let mut ours = [0.0; 2];
        d.feedforward_torque_moving(&row[..2], &row[2..4], &row[4..6], &mut ours);
        let theirs = &row[6..8];
        let scale = theirs[0].abs().max(theirs[1].abs());
        for k in 0..2 {
            assert!(
                (ours[k] - theirs[k]).abs() <= 1e-8 * scale,
                "state {:?}: motor {k} {} N m against {} N m",
                &row[..6],
                ours[k],
                theirs[k]
            );
        }
    }
}
