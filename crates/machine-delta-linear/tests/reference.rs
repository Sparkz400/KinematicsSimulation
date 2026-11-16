//! The linear delta's rigid, loss-free inverse dynamics against an independent reference: the model of
//! docs/model-reference.md written again in Python as a Lagrangian in the carriages' travels and the plate's position,
//! with the legs' constraints through Lagrange multipliers (`reference/lagrange_reference.py`).

use machine_delta_linear::{LinearDelta, lossless_config};
use sim_core::Machine;

/// Motor torques of the rigid model (`feedforward_torque_moving`) at 30 random states in the working range with the
/// columns upright and 30 with them leaning 10 degrees inwards, speeds and accelerations up to the axes' limits,
/// against the Lagrangian's, to 1e-8 of the largest torque at each state.
#[test]
fn linear_delta_inverse_dynamics_matches_the_lagrangian() {
    let reference: Vec<Vec<f64>> = include_str!("../reference/inverse_dynamics.csv")
        .lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with('i'))
        .map(|l| l.split(',').map(|v| v.trim().parse().unwrap()).collect())
        .collect();
    assert_eq!(reference.len(), 60);
    for incline in [0.0, 10.0] {
        let mut cfg = lossless_config();
        cfg.set_text("effects.belt_stretch = false").unwrap();
        cfg.set_text(&format!("geometry.column_incline_deg = {incline:?}")).unwrap();
        let d = LinearDelta::new(&cfg).unwrap();
        let rows: Vec<_> = reference.iter().filter(|row| row[0] == incline).collect();
        assert_eq!(rows.len(), 30);
        for row in rows {
            assert!(d.unreachable(&row[1..4], 0.0).is_none(), "{:?} is outside the working range", &row[..4]);
            let mut ours = [0.0; 3];
            d.feedforward_torque_moving(&row[1..4], &row[4..7], &row[7..10], &mut ours);
            let theirs = &row[10..13];
            let scale = theirs.iter().fold(0.0f64, |m, t| m.max(t.abs()));
            for k in 0..3 {
                sim_core::testkit::record_check(
                    &format!("incline {incline}, motor {k} torque, N m"),
                    theirs[k],
                    ours[k],
                    1e-8 * scale,
                );
                assert!(
                    (ours[k] - theirs[k]).abs() <= 1e-8 * scale,
                    "state {:?}: motor {k} {} N m against {} N m",
                    &row[..10],
                    ours[k],
                    theirs[k]
                );
            }
        }
    }
}
