use super::*;
use std::f64::consts::{PI, TAU};

/// The disc cam's config with `effects.follower_lift_off` and the assignments `extra`, one by one.
fn lifting_config(extra: &[&str]) -> Config {
    let mut cfg = DISC_CAM.config();
    cfg.set_text("effects.follower_lift_off = true").unwrap();
    for line in extra {
        cfg.set_text(line).unwrap();
    }
    cfg
}

fn lifting(extra: &[&str]) -> DiscCam {
    DiscCam::from_config(&lifting_config(extra)).unwrap()
}

/// The rates of [theta, g, theta', g', lost energy] with no torque on the disc, or with the disc's angle held to its speed
/// (`held`: theta'' = 0, a constraint: the budget does not close then).
fn rates(d: &DiscCam, y: [f64; 5], held: bool) -> [f64; 5] {
    let (mut m, mut b) = ([0.0; 4], [0.0; 2]);
    d.dynamics(&[y[0], y[1]], &[y[2], y[3]], &mut m, &mut b);
    let loss = d.loss_power(&[y[0], y[1]], &[y[2], y[3]]);
    if held {
        // m g'' + b_g = 0 with theta'' = 0
        return [y[2], y[3], 0.0, -b[1] / m[3], loss];
    }
    let det = m[0] * m[3] - m[1] * m[2];
    [y[2], y[3], (-b[0] * m[3] + m[1] * b[1]) / det, (-m[0] * b[1] + m[2] * b[0]) / det, loss]
}

/// One RK4 step of `rates`.
fn rk4(d: &DiscCam, y: [f64; 5], dt: f64, held: bool) -> [f64; 5] {
    let at = |y: [f64; 5], k: [f64; 5], h: f64| -> [f64; 5] { std::array::from_fn(|i| y[i] + h * k[i]) };
    let k1 = rates(d, y, held);
    let k2 = rates(d, at(y, k1, 0.5 * dt), held);
    let k3 = rates(d, at(y, k2, 0.5 * dt), held);
    let k4 = rates(d, at(y, k3, dt), held);
    std::array::from_fn(|i| y[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]))
}

/// Palmgren's line contact by hand: 160 N on a roller of 14 mm presses in 3.84e-5 * 160^0.9 / 14^0.8 = 4.4785e-4 mm, its
/// stiffness Q / (0.9 d) = 3.9695e8 N/m; the energy 9 / 19 d Q; nothing in tension. At rest halfway up the rise (s =
/// 15 mm) the shipped follower (3 kg with the tool) is pressed in by 100 + 2000 * 0.015 + 3 * 9.80665 = 159.420 N
/// (less 2000 N/m times the 0.45 um it sinks: 9e-4 N): 4.4639e-7 m.
#[test]
fn the_roller_s_line_contact_presses_in_as_palmgren_s_formula_says() {
    let d = lifting(&[]);
    let lo = d.lift_off.unwrap();
    let depth = lo.depth(160.0);
    assert!((depth / 4.4785e-7 - 1.0).abs() < 1e-4, "{depth}");
    let [q, k] = lo.load(depth);
    assert!((q - 160.0).abs() < 1e-9 && (k / 3.9695e8 - 1.0).abs() < 1e-4, "{q} {k}");
    assert_eq!(lo.load(-1e-6), [0.0, 0.0]);
    // At theta = 0 (s = 0) the follower sinks by the depth: its preload's and weight's work, the spring's, the contact's.
    let sunk = -(100.0 + 3.0 * 9.80665) * depth + 0.5 * 2000.0 * depth * depth + 9.0 / 19.0 * depth * 160.0;
    assert!((d.potential(&[0.0, -depth]) - d.potential(&[0.0, 0.0]) - sunk).abs() < 1e-15);
    let rest = d.passive_rest(&[PI / 3.0])[0];
    assert!((rest / -4.4639e-7 - 1.0).abs() < 1e-4, "{rest}");
    // There the contact carries the spring and the weight: nothing moves.
    let (mut m, mut b) = ([0.0; 4], [0.0; 2]);
    d.dynamics(&[PI / 3.0, rest], &[0.0, 0.0], &mut m, &mut b);
    assert!(b[1].abs() < 1e-9, "{b:?}");
}

/// The follower of 3 kg (2 kg and the 1 kg tool) with no spring on a disc held at 20 rad/s: it leaves the disc where the
/// cam's downward acceleration exceeds what its weight and the guide's friction (5 N against its rise) give, m s''
/// theta'^2 + m g + F_f = 0. The 3-4-5 rise of h = 30 mm over beta = 2.0944 rad has s'' = (h / beta^2) 60 xi (1 - xi)
/// (1 - 2 xi), xi = theta / beta, h / beta^2 * 60 = 0.410347 m/rad^2: xi (1 - xi) (1 - 2 xi) = -(9.80665 + 5 / 3) /
/// (0.410347 * 400) = -0.069900, the root past the rise's middle xi = 0.654571, theta = 1.37093 rad. The flag (the
/// contact carries nothing) comes on within a sample of that angle (20 rad/s * 62.5 us = 1.25 mrad; the run: 0.16 mrad
/// after it, the contact unloading quasi-statically; the gap itself turns positive 2.3 mrad later, the last nm of depth
/// springing back under the cut-off damper). The contact force is never below zero, the gap opens, the signals show it.
#[test]
fn a_spring_sized_too_short_lets_the_follower_leave_the_disc_where_the_hand_calculation_says() {
    let d = lifting(&["follower.spring_n_m = 0.0", "follower.spring_preload_n = 0.0"]);
    let (omega, dt) = (20.0, 62.5e-6 / 16.0);
    let mut y = [0.5, d.passive_rest(&[0.5])[0], omega, 0.0, 0.0];
    let off_disc = |y: &[f64; 5]| {
        let mut out = Vec::new();
        d.signals(&[y[0], y[1]], &[y[2], y[3]], &[0.0, 0.0], &[0.0], &mut out);
        assert_eq!(out.len(), d.signal_infos().len());
        assert!(out[3] >= 0.0, "{out:?}");
        out[7] == 1.0
    };
    let mut flag = None;
    while y[0] < 1.8 && flag.is_none() {
        y = rk4(&d, y, dt, true);
        if off_disc(&y) {
            flag = Some(y[0]);
        }
    }
    let theta = flag.expect("the follower never left the disc");
    assert!((theta - 1.37093).abs() < 1.25e-3, "the flag came on at {theta} rad, the hand value is 1.37093");
    let mut peak_gap = 0.0f64;
    while y[0] < 1.6 {
        y = rk4(&d, y, dt, true);
        peak_gap = peak_gap.max(y[1]);
    }
    assert!(peak_gap > 1e-4, "{peak_gap}");
    let mut out = Vec::new();
    d.signals(&[y[0], y[1]], &[y[2], y[3]], &[0.0, 0.0], &[0.0], &mut out);
    assert_eq!(out.len(), d.signal_infos().len());
    assert!(out[3] == 0.0 && out[7] == 1.0, "{out:?}");
}

/// A spring strong enough (the shipped one, 100 N + 2000 N/m, at one turn a second) keeps the follower on the disc, and
/// without the guide's friction (which only this model has) the contact force along the axis is the switch-off
/// model's: halfway up the rise (theta = pi / 3, s'' = 0) the spring's 100 + 2000 * 0.015 = 130 N and the weight 29.42
/// N, 159.420 N. From rest pressed in (`passive_rest`) the follower stays within 1 nm of the disc's lift plus its
/// static depth, and the force within 0.01 N (the run: 6e-11 m and 7.7e-3 N, the contact ringing at 1.8 kHz as the load changes).
#[test]
fn with_a_spring_strong_enough_the_follower_never_leaves_and_the_result_is_the_switch_off_one() {
    let d = lifting(&["follower.guide_friction_n = 0.0"]);
    let lo = d.lift_off.unwrap();
    let (omega, dt) = (TAU, 62.5e-6 / 16.0);
    let mut y = [0.0, d.passive_rest(&[0.0])[0], omega, 0.0, 0.0];
    let off = DiscCam::from_config(&DISC_CAM.config()).unwrap();
    let mut worst = (0.0f64, 0.0f64);
    while y[0] < PI / 3.0 {
        y = rk4(&d, y, dt, true);
        let along = off.loads(y[0], omega, 0.0)[0] * off.pressure_angle(y[0]).cos();
        worst.0 = worst.0.max((y[1] + lo.depth(along)).abs());
        worst.1 = worst.1.max((d.contact_force(y[1], y[3])[0] - along).abs());
    }
    assert!(worst.0 < 1e-9 && worst.1 < 0.01, "{worst:?}");
    let along = off.loads(y[0], omega, 0.0)[0] * off.pressure_angle(y[0]).cos();
    assert!((along - 159.42).abs() < 0.05, "{along}");
}

/// The budget closes through impacts: the disc coasts (no torque) at 20 rad/s from the middle of the rise, the 3 kg
/// follower with no spring leaves it each turn and comes back with impacts; the contact damper's and the guide's power
/// integrated beside (`Mechanism::loss_power`): T + V + the heat equals the start's. The run: 2.1e-8 of the start's
/// energy at a step of 2 us, 4.2e-7 at 4 us (RK4's h^4); the bound 1e-6 holds the 2 us run with room.
#[test]
fn the_energy_budget_closes_over_the_impacts() {
    let d = lifting(&["follower.spring_n_m = 0.0", "follower.spring_preload_n = 0.0"]);
    let energy = |y: &[f64; 5]| {
        let (mut m, mut b) = ([0.0; 4], [0.0; 2]);
        d.dynamics(&[y[0], y[1]], &[y[2], y[3]], &mut m, &mut b);
        0.5 * (y[2] * (m[0] * y[2] + m[1] * y[3]) + y[3] * (m[2] * y[2] + m[3] * y[3])) + d.potential(&[y[0], y[1]])
    };
    let mut y = [0.5, d.passive_rest(&[0.5])[0], 20.0, 0.0, 0.0];
    let start = energy(&y);
    let (mut off, mut lands, dt) = (false, 0, 2e-6);
    for _ in 0..1_000_000 {
        let before = y[1];
        y = rk4(&d, y, dt, false);
        off |= y[1] > 1e-5;
        lands += usize::from(before > 0.0 && y[1] <= 0.0);
    }
    assert!(off && lands > 1, "{lands} impacts");
    let drift = energy(&y) + y[4] - start;
    assert!(
        y[4] > 1e-3 && drift.abs() < 1e-6 * start,
        "start {start} J, now {} J, heat {} J, drift {drift} J",
        energy(&y),
        y[4]
    );
}

/// With the switch on the machine has the gap as a passive joint (two more states, the gap's signal `follower.gap`,
/// the contact's loss row and no row for a damper the gap joint does not have), rests pressed in (the standard static
/// hold), and the standard budget closes through its own runs, rigid and elastic; off, nothing of it.
#[test]
fn the_lift_off_is_a_passive_joint_with_its_own_loss_row_and_the_standard_checks_hold() {
    for elastic in [false, true] {
        let mut cfg = lossless_config(&DISC_CAM);
        cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
        let off = (DISC_CAM.build)(&cfg).ok().unwrap();
        cfg.set_text("effects.follower_lift_off = true").unwrap();
        let on = (DISC_CAM.build)(&cfg).ok().unwrap();
        assert_eq!(on.state_len(), off.state_len() + 2);
        let ids: Vec<String> = on.signal_infos().iter().map(|s| s.id.to_string()).collect();
        for id in ["follower.gap", "follower.off_disc", "power.follower_contact"] {
            assert!(ids.iter().any(|i| i == id), "{id} in {ids:?}");
        }
        assert!(!ids.iter().any(|i| i.starts_with("swing.") || i == "power.payload_swing"), "{ids:?}");
        sim_core::testkit::static_hold(on.as_ref());
        sim_core::testkit::loss_budget(on.as_ref());
    }
    assert!(DiscCam::from_config(&DISC_CAM.config()).unwrap().passive_joints().is_empty());
}
