use super::*;
use std::f64::consts::{PI, TAU};

/// The disc cam with `effects.follower_lift_off`, the return spring as given (`spring_n_m`, `spring_preload_n`).
fn lifting(spring: f64, preload: f64) -> DiscCam {
    let mut cfg = DISC_CAM.config();
    cfg.set_text(&format!(
        "effects.follower_lift_off = true\nfollower.spring_n_m = {spring}\nfollower.spring_preload_n = {preload}"
    ))
    .unwrap();
    DiscCam::from_config(&cfg).unwrap()
}

/// The rates of [theta, g, theta', g', lost energy] with the disc's torque `torque` (N m) and, where `turn` is given, the
/// disc's angle held to a speed (theta'' = 0, a constraint: the budget does not close then).
fn rates(d: &DiscCam, y: [f64; 5], torque: f64, turn: Option<f64>) -> [f64; 5] {
    let (mut m, mut b) = ([0.0; 4], [0.0; 2]);
    d.dynamics(&[y[0], y[1]], &[y[2], y[3]], &mut m, &mut b);
    let loss = d.loss_power(&[y[0], y[1]], &[y[2], y[3]]);
    match turn {
        // m g'' + b_g = 0 with theta'' = 0
        Some(_) => [y[2], y[3], 0.0, -b[1] / m[3], loss],
        None => {
            let (r0, r1) = (torque - b[0], -b[1]);
            let det = m[0] * m[3] - m[1] * m[2];
            [y[2], y[3], (r0 * m[3] - m[1] * r1) / det, (m[0] * r1 - m[2] * r0) / det, loss]
        }
    }
}

/// One RK4 step of `rates`.
fn rk4(d: &DiscCam, y: [f64; 5], dt: f64, turn: Option<f64>) -> [f64; 5] {
    let at = |y: [f64; 5], k: [f64; 5], h: f64| -> [f64; 5] { std::array::from_fn(|i| y[i] + h * k[i]) };
    let k1 = rates(d, y, 0.0, turn);
    let k2 = rates(d, at(y, k1, 0.5 * dt), 0.0, turn);
    let k3 = rates(d, at(y, k2, 0.5 * dt), 0.0, turn);
    let k4 = rates(d, at(y, k3, dt), 0.0, turn);
    std::array::from_fn(|i| y[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]))
}

/// The follower of 3 kg (2 kg and the 1 kg tool) with no spring on a disc turning at 20 rad/s (kept there): it leaves the
/// disc where the cam's downward acceleration exceeds g, m s'' theta'^2 + m g = 0 (F_spring = 0). The 3-4-5 rise of h = 30 mm
/// over beta = 2.0944 rad has s'' = (h / beta^2) 60 xi (1 - xi) (1 - 2 xi), xi = theta / beta, with h / beta^2 * 60 =
/// 0.410347 m/rad^2: xi (1 - xi) (1 - 2 xi) = -g / (0.410347 * 400) = -0.059745, the root past the rise's middle (where s''
/// turns negative) xi = 0.62785, theta = 1.3150 rad (75.3 degrees). The flag comes on at that angle, to the step's angle
/// (20 rad/s * 62.5 us = 1.25 mrad) and the contact's own give (1 um of the 1e8 N/m spring). The contact force is never below
/// zero (the one-sided spring and damper), and the follower's gap opens.
#[test]
fn a_spring_sized_too_short_lets_the_follower_leave_the_disc_where_the_hand_calculation_says() {
    let d = lifting(0.0, 0.0);
    let (omega, dt) = (20.0, 62.5e-6 / 16.0);
    let mut y = [0.5, 0.0, omega, 0.0, 0.0];
    let mut flag = None;
    let mut peak_gap = 0.0f64;
    while y[0] < 1.8 && flag.is_none() {
        y = rk4(&d, y, dt, Some(omega));
        assert!(d.contact_force(y[1], y[3])[0] >= 0.0);
        if y[1] > 0.0 && y[0] > 1.0 {
            flag = Some(y[0]);
        }
    }
    let theta = flag.expect("the follower never left the disc");
    assert!((theta - 1.3150).abs() < 2e-3, "the flag came on at {theta} rad, the hand value is 1.3150");
    // On: the gap opens as the cam falls away faster than the follower, and the signals show it.
    while y[0] < 1.6 {
        y = rk4(&d, y, dt, Some(omega));
        peak_gap = peak_gap.max(y[1]);
    }
    assert!(peak_gap > 1e-4, "{peak_gap}");
    let mut out = Vec::new();
    d.signals(&[y[0], y[1]], &[y[2], y[3]], &[0.0, 0.0], &[0.0], &mut out);
    assert_eq!(out.len(), d.signal_infos().len());
    assert!(out[3] >= 0.0 && out[8] == f64::from(y[1] > 0.0), "{out:?}");
}

/// A spring strong enough (the shipped one, 100 N + 2000 N/m, at one turn a second) keeps the follower on the disc: the gap
/// stays below zero (pressed in by F / c_s, some microns), the lift is the disc's to 5 um, and the contact force along
/// the axis is the switch-off model's held + inertia force: halfway up the rise (theta = pi / 3, s'' = 0) the spring's
/// 100 + 2000 * 0.015 = 130 N and the weight 29.42 N, 159.420 N, which is the contact spring's c_s d.
#[test]
fn with_a_spring_strong_enough_the_follower_never_leaves_and_the_result_is_the_switch_off_one() {
    let d = lifting(2000.0, 100.0);
    let (omega, dt) = (TAU, 62.5e-6 / 16.0);
    let mut y = [0.0, 0.0, omega, 0.0, 0.0];
    let mut worst = 0.0f64;
    while y[0] < PI / 3.0 {
        y = rk4(&d, y, dt, Some(omega));
        assert!(y[1] <= 0.0 || y[0] < 0.05, "left the disc at {y:?}");
        worst = worst.max(y[1].abs());
    }
    assert!(worst < 5e-6, "{worst}");
    let off = DiscCam::from_config(&DISC_CAM.config()).unwrap();
    let [along, _] = off.loads(y[0], omega, 0.0);
    let held = along * off.pressure_angle(y[0]).cos();
    let on = d.contact_force(y[1], y[3])[0];
    assert!((on - held).abs() < 0.5 && (held - 159.42).abs() < 0.05, "{on} {held}");
}

/// The budget closes through impacts: the disc coasts (no torque) at 20 rad/s from the middle of the rise, the 3 kg
/// follower with no spring leaves it near theta = 1.315 rad each turn and comes back with a bounce, the contact damper's
/// and the guide's power integrated beside (`Mechanism::loss_power`): T + V + the heat equals the start's to a thousandth, and the heat is not nothing (the impacts took energy).
#[test]
fn the_energy_budget_closes_over_the_impacts() {
    let d = lifting(0.0, 0.0);
    let energy = |y: &[f64; 5]| {
        let (mut m, mut b) = ([0.0; 4], [0.0; 2]);
        d.dynamics(&[y[0], y[1]], &[y[2], y[3]], &mut m, &mut b);
        0.5 * (y[2] * (m[0] * y[2] + m[1] * y[3]) + y[3] * (m[2] * y[2] + m[3] * y[3])) + d.potential(&[y[0], y[1]])
    };
    let mut y = [0.5, 0.0, 20.0, 0.0, 0.0];
    let start = energy(&y);
    let (mut off, dt) = (false, 2e-6);
    for _ in 0..150_000 {
        y = rk4(&d, y, dt, None);
        off |= y[1] > 1e-5;
    }
    assert!(off, "the follower never left the disc");
    let drift = energy(&y) + y[4] - start;
    assert!(
        y[4] > 1e-3 && drift.abs() < 1e-3 * start.abs().max(1.0),
        "start {start} J, now {} J, heat {} J",
        energy(&y),
        y[4]
    );
}

/// With the switch on the machine has the gap as a passive joint (two more states, its row in the loss budget and the two
/// signals) and the standard budget closes through its own run; off, nothing of it.
#[test]
fn the_lift_off_is_a_passive_joint_with_its_own_loss_row_and_the_standard_budget_closes() {
    let mut cfg = lossless_config(&DISC_CAM);
    cfg.set_text("effects.follower_lift_off = true\neffects.joint_elasticity = false").unwrap();
    let on = (DISC_CAM.build)(&cfg).unwrap();
    let mut plain = lossless_config(&DISC_CAM);
    plain.set_text("effects.joint_elasticity = false").unwrap();
    let off = (DISC_CAM.build)(&plain).unwrap();
    assert_eq!(on.state_len(), off.state_len() + 2);
    assert!(DiscCam::from_config(&DISC_CAM.config()).unwrap().passive_joints().is_empty());
    assert_eq!(lifting(2000.0, 100.0).passive_joints(), vec![(0.0, 0.0)]);
    let ids: Vec<String> = on.signal_infos().iter().map(|s| s.id.to_string()).collect();
    for id in ["follower.gap", "follower.off_disc", "loss.follower_contact"] {
        assert!(ids.iter().any(|i| i == id), "{id} in {ids:?}");
    }
    sim_core::testkit::loss_budget(on.as_ref());
}
