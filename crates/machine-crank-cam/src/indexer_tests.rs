use super::*;
use std::f64::consts::{PI, TAU};

#[test]
fn geneva_drive_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&GENEVA_DRIVE);
}
#[test]
fn cam_indexer_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&CAM_INDEXER);
}

fn geneva(n: usize) -> Indexer {
    let mut cfg = GENEVA_DRIVE.config();
    cfg.set_text(&format!("indexer.stations = {n}")).unwrap();
    Indexer::from_config(&cfg, false).unwrap()
}

/// Largest phi' and |phi''| over the index, on a grid of 200001 points.
fn peaks(ix: &Indexer) -> (f64, f64) {
    (0..=200_000).map(|i| -ix.half + 2.0 * ix.half * i as f64 / 200_000.0).fold((0.0f64, 0.0f64), |(v, a), x| {
        let [_, p1, p2] = ix.motion(x);
        (v.max(p1), a.max(p2.abs()))
    })
}

/// The Geneva wheel of n slots by hand. lambda = sin(pi / n). The pin is in a slot over 2 h = pi - 2 pi / n of the
/// crank's turn: the wheel moves over (n - 2) / (2 n) of it and dwells over (n + 2) / (2 n) (the "(n - 2) / (2 n)" one
/// sometimes reads as the dwell is the index's share): n = 4: 1/4 and 3/4; 6: 1/3, 2/3; 8: 3/8, 5/8.
/// Peak speed ratio at x = 0, lambda / (1 - lambda): 4: 0.70711 / 0.29289 = 2.41421; 6: 0.5 / 0.5 = 1; 8: 0.38268 /
/// 0.61732 = 0.61991. Peak acceleration ratio where cos(x) = sqrt(k^2 + 2) - k, k = (1 + lambda^2) / (4 lambda):
/// n = 4: k = 0.53033, cos x = 0.98005 (11.46 degrees), lambda (1 - lambda^2) sin x / (1 - 2 lambda cos x +
/// lambda^2)^2 = 0.70711 * 0.5 * 0.19875 / 0.11401^2 = 5.4070; n = 6: x = 22.90 degrees, 1.3496; n = 8: x = 31.64
/// degrees, 0.69976 (the textbooks' 5.41, 1.35, 0.700).
#[test]
fn the_geneva_s_dwell_share_and_peak_speed_and_acceleration_ratios_match_the_textbook() {
    for (n, moving, speed, acc) in
        [(4, 0.25, 2.414_213_56, 5.406_981), (6, 1.0 / 3.0, 1.0, 1.349_637), (8, 0.375, 0.619_914_40, 0.699_757)]
    {
        let ix = geneva(n);
        assert!((2.0 * ix.half / TAU - moving).abs() < 1e-12, "{n}");
        assert!((1.0 - 2.0 * ix.half / TAU - (n as f64 + 2.0) / (2.0 * n as f64)).abs() < 1e-12, "{n}");
        // In the dwell the wheel stands, a station on per turn.
        let after = ix.motion(ix.half + 0.1);
        assert_eq!([after[1], after[2]], [0.0, 0.0]);
        assert!((after[0] - TAU / n as f64).abs() < 1e-12 && ix.motion(-ix.half - 0.1)[0].abs() < 1e-12);
        assert!((ix.motion(ix.half + TAU + 0.1)[0] - 2.0 * TAU / n as f64).abs() < 1e-12);
        let (v, a) = peaks(&ix);
        assert!((v - speed).abs() < 1e-8, "{n}: {v}");
        assert!((a - acc).abs() < 1e-5, "{n}: {a}");
        // The wheel's angle is continuous into and out of the dwell, and phi' is the slope of phi.
        let x = 0.3;
        let slope = (ix.motion(x + 1e-6)[0] - ix.motion(x - 1e-6)[0]) / 2e-6;
        assert!((slope - ix.motion(x)[1]).abs() < 1e-8);
        assert!((ix.motion(ix.half - 1e-12)[0] - TAU / n as f64).abs() < 1e-9);
    }
}

/// At the pin's entry the wheel is at rest (phi' = cos(x) - lambda = 0 at cos(h) = sin(pi / n) = lambda) but its
/// acceleration jumps from 0 to lambda (1 - lambda^2) cos(pi / n) / cos^4(pi / n) = tan(pi / n): 1 for n = 4, 0.57735 for
/// n = 6. The shipped wheel at 60 turns a minute (theta' = 2 pi): J_t = 0.03 + 0.4 + 3 * 0.2^2 = 0.55 kg m^2, its torque
/// steps to 0.55 * 0.57735 * 39.4784 = 12.536 N m, the pin's force on the slot (r = C cos(pi / n) = 0.12 * 0.86603 =
/// 0.103923 m) to 120.63 N; the crank's torque J phi' phi'' theta'^2 stays 0.
#[test]
fn the_geneva_s_acceleration_jumps_at_the_pin_s_entry_and_the_torque_on_the_wheel_with_it() {
    let ix = Indexer::from_config(&GENEVA_DRIVE.config(), false).unwrap();
    let w = TAU;
    let before = ix.loads(-ix.half - 1e-9, w, 0.0);
    let after = ix.loads(-ix.half + 1e-9, w, 0.0);
    assert_eq!(before, [0.0, 0.0]);
    assert!((ix.motion(-ix.half + 1e-9)[2] - (PI / 6.0).tan()).abs() < 1e-7);
    assert!((after[0] - 12.536).abs() < 1e-3, "{after:?}");
    assert!((after[1] - 120.63).abs() < 1e-2, "{after:?}");
    let (mut mass, mut bias) = ([0.0], [0.0]);
    ix.dynamics(&[-ix.half + 1e-9], &[w], &mut mass, &mut bias);
    assert!(bias[0].abs() < 1e-6 && (mass[0] - 0.01).abs() < 1e-12, "{bias:?} {mass:?}");
    // Leaving, the acceleration falls from -tan(pi / n) to 0: the step that sets the table ringing.
    assert!((ix.motion(ix.half - 1e-9)[2] + (PI / 6.0).tan()).abs() < 1e-7);
}

/// The modified sine (VDI 2143): peak acceleration 4 pi^2 / (4 + pi) = 39.4784 / 7.14159 = 5.52796 at u = 1/8, peak speed
/// 4 pi / (4 + pi) = 1.75960 at u = 1/2; continuous in s, s', s'' at 1/8 and 7/8, 0 to 1, no acceleration at the ends.
#[test]
fn the_modified_sine_s_peak_acceleration_is_5_53_and_its_peak_speed_1_76() {
    let [s0, v0, a0] = modified_sine(0.0);
    let [s1, v1, a1] = modified_sine(1.0);
    assert!(s0.abs() < 1e-15 && (s1 - 1.0).abs() < 1e-15 && v0.abs() < 1e-15 && v1.abs() < 1e-15);
    assert!(a0.abs() < 1e-14 && a1.abs() < 1e-12);
    assert!((modified_sine(0.125)[2] - 5.527_957).abs() < 1e-6);
    assert!((modified_sine(0.5)[1] - 1.759_603).abs() < 1e-6);
    assert!((modified_sine(0.5)[0] - 0.5).abs() < 1e-15);
    for u in [0.125, 0.875] {
        let (a, b) = (modified_sine(u - 1e-12), modified_sine(u + 1e-12));
        for k in 0..3 {
            assert!((a[k] - b[k]).abs() < 1e-9, "{u} {k}");
        }
    }
    let peak = (0..=100_000).map(|i| modified_sine(i as f64 / 100_000.0)[2].abs()).fold(0.0, f64::max);
    assert!((peak - 5.527_957).abs() < 1e-6);
}

/// The shipped cam indexer: a station (60 degrees, 1.0472 rad) over 120 degrees of the cam (2.0944 rad): peak phi'' =
/// 1.047198 * 5.527957 / 2.094395^2 = 1.319707, peak phi' = 1.0472 * 1.75960 / 2.0944 = 0.87980; against the six-slot Geneva's
/// 1.3496 and 1.0, over the same third of the turn.
#[test]
fn the_cam_indexer_s_peaks_are_the_modified_sine_s_over_its_index_angle() {
    let ix = Indexer::from_config(&CAM_INDEXER.config(), true).unwrap();
    let (v, a) = peaks(&ix);
    assert!((v - 0.879_802).abs() < 1e-6, "{v}");
    assert!((a - 1.319_707).abs() < 1e-5, "{a}");
    assert_eq!(ix.motion(-ix.half - 1e-9)[2], 0.0);
    assert!(ix.motion(-ix.half + 1e-9)[2].abs() < 1e-6);
}

/// The table's ring after an index at 60 turns a minute: the table side (0.4 + 3 * 0.2^2 = 0.52 kg m^2) twists against
/// the wheel the cam holds, w_n = sqrt(2e5 / 0.52) = 620.17 rad/s, zeta 0.03. The Geneva's step of -tan(pi / 6)
/// theta'^2 = -22.794 rad/s^2 at the exit leaves the table's twist ringing by 22.794 / 620.17^2 = 5.927e-5 rad (the
/// entry's step has died by exp(-0.03 * 620 / 3) = 0.002 by then): 11.9 um at 0.2 m, beyond the 10 um tolerance (5e-5
/// rad) for ln(5.927 / 5) / (0.03 * 620.17) = 9.1 ms. The modified sine ends without a step in acceleration, with its
/// jerk s''' = 16 pi^3 / (4 + pi) = 69.466: phi''' theta'^3 = 69.466 * 1.047198 / 2.094395^3 * 248.050 = 1964.1 rad/s^3
/// leaves j / w_n^3 = 1964.1 / 2.3853e8 = 8.23e-6 rad, 1.6 um: within the tolerance when the index ends.
#[test]
fn the_geneva_leaves_the_table_ringing_by_its_acceleration_step_and_the_cam_indexer_does_not() {
    let wheel = Indexer::from_config(&GENEVA_DRIVE.config(), false).unwrap();
    let a = wheel.residual(TAU);
    assert!((a / 5.927e-5 - 1.0).abs() < 0.03, "{a}");
    let t = wheel.settling_time(TAU);
    assert!((t - 0.0091).abs() < 0.002, "{t}");
    // Twice as fast: four times the step, ln(4 * 5.927 / 5) / 18.605 = 83.7 ms.
    assert!((wheel.settling_time(2.0 * TAU) - 0.0837).abs() < 0.003, "{}", wheel.settling_time(2.0 * TAU));
    let cam = Indexer::from_config(&CAM_INDEXER.config(), true).unwrap();
    assert!((cam.residual(TAU) / 8.23e-6 - 1.0).abs() < 0.05, "{}", cam.residual(TAU));
    assert_eq!(cam.settling_time(TAU), 0.0);
}

/// The cam's pressure angle by hand, shipped cam indexer (r_p 0.08 m, R_c 0.10 m, J_t = 0.02 + 0.4 + 3 * 0.2^2 = 0.54 kg m2):
/// in the middle of the index phi' = (2 pi / 6) * (4 pi / (4 + pi)) / (2 pi / 3) = 2 pi / (4 + pi) = 0.87980, so tan(alpha)
/// = 0.08 * 0.87980 / 0.10 = 0.70384, cos(alpha) = 0.81775 (alpha = 35.1 degrees). At theta'' = 100 rad/s2 (phi'' = 0 there)
/// the wheel's torque is 0.54 * 0.87980 * 100 = 47.509 N m, along the tangent 47.509 / 0.08 = 593.9 N, along the normal
/// 593.9 / 0.81775 = 726.2 N. Without the key the force stays 593.9 N.
#[test]
fn the_cam_s_pressure_angle_raises_the_contact_force_by_one_over_its_cosine() {
    let ix = Indexer::from_config(&CAM_INDEXER.config(), true).unwrap();
    assert!((ix.pressure_angle(0.0).to_degrees() - 35.1).abs() < 0.05, "{}", ix.pressure_angle(0.0).to_degrees());
    let [torque, force] = ix.loads(0.0, 0.0, 100.0);
    assert!((torque - 47.509).abs() < 0.01 && (force - 726.2).abs() < 0.5, "{torque} {force}");
    let mut cfg = CAM_INDEXER.config();
    cfg.set_text("indexer.cam_radius_m = 1e9").unwrap();
    let flat = Indexer::from_config(&cfg, true).unwrap().loads(0.0, 0.0, 100.0)[1];
    assert!((flat - 593.9).abs() < 0.1, "{flat}");
    // The Geneva has no cam: no angle, and no change.
    assert_eq!(geneva(6).pressure_angle(0.0), 0.0);
    // At 60 turns a minute (theta'' = 0) the peak moves to where phi'' and the angle together are largest, u = 0.84696
    // in the deceleration: |phi''| / cos(alpha) = 1.34716 against phi'' 1.31970 at u = 7/8 (both on a grid of 1e5 by
    // hand), 0.54 * 39.478 / 0.08 times them: 358.99 N against 351.67 N, 2.08 % more (the shipped cam cycle's verdict
    // moves from 352.5 to 359.8 N, the segment cycle's from 888.8 to 906.3 N, of 2000).
    let peak = |ix: &Indexer| {
        (0..=100_000)
            .map(|i| ix.loads(-ix.half + 2.0 * ix.half * i as f64 / 1e5, TAU, 0.0)[1].abs())
            .fold(0.0, f64::max)
    };
    let flat = Indexer::from_config(&cfg, true).unwrap();
    assert!((peak(&ix) - 358.99).abs() < 0.01 && (peak(&flat) - 351.67).abs() < 0.01, "{} {}", peak(&ix), peak(&flat));
}

/// effects.table_twist: the mass matrix of the doc comment by hand at the middle of the index (J_d 0.012, J_w 0.02, J_s =
/// 0.4 + 0.12 = 0.52, J_t = 0.54, phi' = 0.87980): M00 = 0.012 + 0.54 * 0.77405 = 0.42999, M01 = 0.52 * 0.87980 = 0.45750,
/// M11 = 0.52; phi'' = 0 there, so no speed terms. The joint's spring is 2e5 N m/rad and its damper 2 * 0.03 * sqrt(2e5 *
/// 0.52) = 19.35 N m s/rad, and the state has the joint's angle and speed beyond the rigid machine's.
#[test]
fn the_table_twist_is_a_joint_of_its_own_with_the_matrix_of_the_two_inertias() {
    let mut cfg = CAM_INDEXER.config();
    cfg.set_text("effects.table_twist = true").unwrap();
    let ix = Indexer::from_config(&cfg, true).unwrap();
    let (mut mass, mut bias) = ([0.0; 4], [0.0; 2]);
    ix.dynamics(&[0.0, 0.0], &[3.0, 0.0], &mut mass, &mut bias);
    for (got, want) in mass.iter().zip([0.42999, 0.45750, 0.45750, 0.52]) {
        assert!((got - want).abs() < 2e-5, "{mass:?}");
    }
    assert!(bias.iter().all(|b| b.abs() < 1e-9), "{bias:?}");
    let (k, c) = ix.passive_joints()[0];
    assert!((k - 2e5).abs() < 1e-6 && (c - 19.35).abs() < 0.01, "{k} {c}");
    // Off, the machine has no such joint; on, two more states.
    let on = (CAM_INDEXER.build)(&cfg).ok().unwrap();
    let off = (CAM_INDEXER.build)(&CAM_INDEXER.config()).ok().unwrap();
    assert_eq!(on.state_len(), off.state_len() + 2);
    assert!(Indexer::from_config(&CAM_INDEXER.config(), true).unwrap().passive_joints().is_empty());
    // The cam turns the wheel alone, the shaft's k x + c x' on it: at theta'' = 100 rad/s2 with x = 1e-4, x' = 0, J_w a_w
    // - k x = 0.02 * 87.980 - 20 = -18.240 N m (J_t a_w = 47.509 N m with the twist held, as the rigid plan sees it).
    let mut out = Vec::new();
    ix.signals(&[0.0, 1e-4], &[0.0, 0.0], &[100.0, 0.0], &[0.0], &mut out);
    assert!((out[3] + 18.240).abs() < 1e-3, "{out:?}");
    out.clear();
    ix.signals(&[0.0], &[0.0], &[100.0], &[0.0], &mut out);
    assert!((out[3] - 47.509).abs() < 1e-3, "{out:?}");
}

/// The twist joint in the machine, rigid and elastic chains, lossless and with the payload: it rests at zero (the
/// standard static hold), the standard energy and loss budgets close (its damper's power the row `power.table_twist`,
/// its spring's energy elastic), its signal is `table.twist`. The mode by hand: in the dwell (phi' = 0) the joint is
/// free of the driver, J_s x'' = -k x - c x', w_n = sqrt(2e5 / 0.52) = 620.17 rad/s, zeta 0.03: a twist of 1e-4 rad
/// rings at w_d = 619.89 rad/s (period 10.136 ms) and its peaks fall by exp(-2 pi 0.03 / sqrt(1 - 0.03^2)) = 0.82814
/// a period.
#[test]
fn the_table_twist_rings_at_its_mode_by_hand_and_its_budgets_close_rigid_and_elastic() {
    for (entry, cam) in [(&GENEVA_DRIVE, false), (&CAM_INDEXER, true)] {
        for elastic in [false, true] {
            let mut cfg = lossless_config(entry);
            cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
            cfg.set_text("effects.table_twist = true").unwrap();
            let m = (entry.build)(&cfg).ok().unwrap();
            let ids: Vec<String> = m.signal_infos().iter().map(|s| s.id.to_string()).collect();
            for id in ["table.twist", "power.table_twist"] {
                assert!(ids.iter().filter(|i| *i == id).count() == 1, "{id} in {ids:?}");
            }
            assert!(!ids.iter().any(|i| i.starts_with("swing.") || i == "power.payload_swing"), "{ids:?}");
            sim_core::testkit::static_hold(m.as_ref());
            sim_core::testkit::energy_conservation(m.as_ref());
            sim_core::testkit::loss_budget(m.as_ref());
            // From the middle of a dwell (theta = pi), the twist let go at 1e-4 rad.
            let (mut s, mut hold) = (Vec::new(), vec![0.0; 1]);
            m.equilibrium(&[PI], &mut s, &mut hold);
            let at = if elastic { 5 } else { 3 };
            s[at] = 1e-4;
            let mut rk = sim_core::Rk4::new(s.len());
            let h = 62.5e-6 / 4.0;
            let (mut t, mut last, mut ups, mut peaks) = (0.0, 1e-4, Vec::new(), Vec::new());
            let mut slope = 0.0;
            while t < 0.06 {
                rk.step(|y, out| m.derivatives(y, &hold, out), &mut s, h);
                t += h;
                let x = s[at];
                if last < 0.0 && x >= 0.0 {
                    ups.push(t - h * x / (x - last));
                }
                if slope > 0.0 && x < last {
                    peaks.push(last);
                }
                (slope, last) = (x - last, x);
            }
            let period = (ups[ups.len() - 1] - ups[0]) / (ups.len() - 1) as f64;
            let fall = (peaks[peaks.len() - 1] / peaks[0]).powf(1.0 / (peaks.len() - 1) as f64);
            assert!((period / 10.136e-3 - 1.0).abs() < 1e-4, "{} {cam} {elastic}: period {period}", entry.id);
            assert!((fall / 0.82814 - 1.0).abs() < 1e-4, "{} {cam} {elastic}: fall {fall}", entry.id);
        }
    }
}

/// The ring beside the dynamics against the joint: the Geneva at 60 turns a minute under its cam with the twist as a
/// joint (the shipped, elastic chain) leaves the table ringing after an index by what `residual` works out beside at
/// that speed, 5.93e-5 rad (by hand in the test above): its amplitude sqrt(x^2 + ((x' + zeta w_n x) / w_d)^2) at the
/// first sample of the dwell, where the table's speed is the twist's x'. The run: 5.930e-5 against 5.940e-5 beside.
#[test]
fn the_geneva_s_ring_as_a_joint_is_the_ring_worked_out_beside() {
    let mut cfg = GENEVA_DRIVE.config();
    cfg.set_text("cam.run = true").unwrap();
    cfg.set_text("effects.table_twist = true").unwrap();
    let mut sim = sim_components::Sim::new(&GENEVA_DRIVE, &cfg).unwrap();
    sim.command(sim_components::Command::StartCycle).unwrap();
    sim.run(1.6, |_| {});
    let at = |id: &str| sim.signal_index(id).unwrap();
    let (twist, speed, theta) = (at("table.twist"), at("table.speed"), at("joint.1.position"));
    let ix = Indexer::from_config(&cfg, false).unwrap();
    let (w, zeta) = ix.mode;
    let wd = w * (1.0 - zeta * zeta).sqrt();
    let dwell = |x: f64| (x + ix.half).rem_euclid(TAU) >= 2.0 * ix.half;
    let mut out = Vec::new();
    let (mut moving, mut ring) = (false, None);
    while ring.is_none() {
        sim.step();
        out.clear();
        sim.sample(&mut out);
        if moving && dwell(out[theta]) {
            ring = Some((out[twist].hypot((out[speed] + zeta * w * out[twist]) / wd), out[theta]));
        }
        moving = !dwell(out[theta]);
        assert!(sim.time() < 4.0);
    }
    let (ring, x) = ring.unwrap();
    let beside = ix.residual(TAU);
    assert!((ring / beside - 1.0).abs() < 0.01, "joint {ring} rad, beside {beside} rad (at {x} rad)");
}

/// effects.table_twist among the machine's modes (`Machine::modes`: the app's marks, `identify`, a shaper following its
/// mode k; it listed the gearbox's alone). In the dwell (theta = pi, phi' = 0) the twist is free of the driver: w_n =
/// sqrt(2e5 / 0.52) = 620.17 rad/s, 98.704 Hz, zeta 0.03 (c = 2 zeta sqrt(k J_s) over 2 sqrt(k J_s)), the lowest, so
/// `resonance_hz` too; beside it the gearbox's two-mass mode as without the twist, with the gearbox's damper c = 5 N m
/// s/rad: zeta = c / (2 sqrt(k mu)) = c w / (2 k), mu = k / w^2 (reported 0 before).
#[test]
fn the_table_twist_is_among_the_modes_with_its_damping() {
    let mut cfg = CAM_INDEXER.config();
    let off = (CAM_INDEXER.build)(&cfg).ok().unwrap().modes(&[PI]);
    cfg.set_text("effects.table_twist = true").unwrap();
    let m = (CAM_INDEXER.build)(&cfg).ok().unwrap();
    let on = m.modes(&[PI]);
    assert!(off.len() == 1 && on.len() == 2, "{off:?} {on:?}");
    let gear = 5.0 * TAU * off[0].hz / (2.0 * 35752.6);
    assert!((off[0].damping / gear - 1.0).abs() < 1e-9, "{off:?} against zeta {gear}");
    let twist = (2e5f64 / 0.52).sqrt() / TAU;
    assert!((on[0].hz / twist - 1.0).abs() < 1e-6 && (on[0].damping - 0.03).abs() < 1e-6, "{on:?} against {twist} Hz");
    assert!((on[1].hz / off[0].hz - 1.0).abs() < 1e-9 && (on[1].damping / gear - 1.0).abs() < 1e-9, "{on:?} {off:?}");
    assert_eq!(m.resonance_hz(&[PI]), Some(on[0].hz));
}
