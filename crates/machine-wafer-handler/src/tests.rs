//! The wafer handler against hand calculations: the kinematics at an angle, the inertia the R motor and the theta
//! motor feel retracted and extended, the centrifugal and Coriolis acceleration of the wafer and the turn rate at which
//! it slips, the bands' stretch under a torque and the direction error it gives, the forces the rigid bands carry, the Z
//! holding torque, the modes, the hold's verdict and the default cycle.
//!
//! The numbers: L = 0.32 m, e = 0.18 m; upper arm 1.5 kg, 0.0128 kg m^2 about its centre 0.16 m out; forearm 1.2 kg,
//! 0.01024 kg m^2, 0.16 m; effector 0.4 kg, 0.004 kg m^2, 0.12 m; wafer 0.1276 kg, 1.4355e-3 kg m^2; column 0.05 kg m^2;
//! gearboxes 50, rotor and gearbox input 9.3e-6 + 1.93e-5 = 2.86e-5 kg m^2.

use super::*;
use sim_components::Sim;

fn handler(edits: &[&str]) -> WaferHandler {
    let mut cfg = ENTRY.config();
    for e in edits {
        cfg.set_text(e).unwrap();
    }
    WaferHandler::new(&cfg).unwrap()
}

/// The rigid, loss-free model: no friction, ripple, gearbox loss or drag, rigid gearboxes and bands.
const RIGID: [&str; 7] = [
    "effects.joint_friction = false",
    "effects.gearbox_drag = false",
    "effects.motor_friction = false",
    "effects.torque_ripple = false",
    "effects.gearbox_efficiency = false",
    "effects.joint_elasticity = false",
    "effects.band_elasticity = false",
];

#[track_caller]
fn close(a: f64, b: f64, tolerance: f64, what: &str) {
    sim_core::testkit::record_check(what, b, a, tolerance * b.abs().max(1e-12));
    assert!((a - b).abs() <= tolerance * b.abs().max(1e-12), "{what}: {a} against {b} by hand");
}

/// A signal by its id.
fn signal(m: &dyn Machine, state: &[f64], torque: &[f64], id: &str) -> f64 {
    let mut out = Vec::new();
    m.signals(state, torque, &mut out);
    out[m.signal_infos().iter().position(|s| s.id == id).unwrap()]
}

/// The rotor and gearbox input at the motor, kg m^2.
const ROTOR: f64 = 9.3e-6 + 1.93e-5;

#[test]
fn the_wafer_handler_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ENTRY);
}

/// At phi = 60 degrees, theta = 30 degrees, by hand: the elbow at L u(90) = (0, 0.32), the wrist at L u(90) + L u(-30) =
/// (0.277128, 0.16) = 2 L cos 60 u(30) on the radial line, the wafer centre 0.18 further at (0.433013, 0.25), r = 0.5;
/// the effector points along theta. dr/dphi = -2 L sin 60 = -0.554256 m/rad, and the condition number 1 / sin 60 =
/// 1.154701. Forward of inverse is the pose to 1e-12, the motors at 50 times theta and phi, and with elastic bands at
/// rest the tool state is the pose.
#[test]
fn the_wrist_runs_on_the_radial_line_and_the_effector_keeps_its_direction() {
    let d = handler(&[]);
    let (theta, phi) = (30f64.to_radians(), 60f64.to_radians());
    let q = [theta, phi, 0.0, 0.0];
    let elbow = d.arm.point(&q, 0, 0.32);
    let wrist = d.arm.point(&q, 1, 0.32);
    let centre = d.arm.wafer_centre(&q);
    assert!(elbow[0].abs() < 1e-12 && (elbow[1] - 0.32).abs() < 1e-12, "{elbow:?}");
    assert!((wrist[0] - 0.277128).abs() < 1e-6 && (wrist[1] - 0.16).abs() < 1e-12, "{wrist:?}");
    assert!((centre[0] - 0.433013).abs() < 1e-6 && (centre[1] - 0.25).abs() < 1e-12, "{centre:?}");
    assert!((Arm::angles(&q)[2] - theta).abs() < 1e-15);
    let [r, dr, _] = d.arm.reach(phi);
    close(r, 0.5, 1e-12, "r");
    close(dr, -0.554256, 1e-6, "dr/dphi");
    close(d.condition(&[0.5, theta, 0.1]).unwrap(), 1.154701, 1e-6, "condition");
    let pose = [0.5, theta, 0.1];
    let (mut motors, mut back) = ([0.0; 3], [0.0; 3]);
    d.inverse(&pose, &mut motors);
    d.forward(&motors, &mut back);
    assert!((0..3).all(|k| (back[k] - pose[k]).abs() < 1e-12), "{back:?}");
    close(motors[0], 50.0 * theta, 1e-12, "motor 1");
    close(motors[1], 50.0 * phi, 1e-12, "motor 2");
    close(motors[2], 0.1 * std::f64::consts::TAU / 0.01, 1e-12, "motor 3");
    let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
    d.equilibrium(&pose, &mut state, &mut torque);
    let (mut pos, mut vel) = ([0.0; 3], [0.0; 3]);
    d.tool_state(&state, &mut pos, &mut vel);
    assert!((0..3).all(|k| (pos[k] - pose[k]).abs() < 1e-12 && vel[k] == 0.0), "{pos:?}");
}

/// The reach and the turn's range: r beyond 2 L + e is out of reach, phi beyond its stops (10 and 110 degrees: r =
/// 0.810303 and -0.038890) and the column beyond its +-200 degrees are refused, and a reach nearer the full extension
/// than the condition number 5 allows (phi below 11.537 degrees, r above 0.806926).
#[test]
fn targets_beyond_the_arm_its_stops_or_near_the_full_extension_are_refused() {
    let d = handler(&[]);
    assert_eq!(d.unreachable(&[0.83, 0.0, 0.1], 0.0), Some("error-handler-out-of-reach"));
    assert_eq!(d.unreachable(&[-0.05, 0.0, 0.1], 0.0), Some("error-handler-joint-stop"));
    assert_eq!(d.unreachable(&[0.5, 201f64.to_radians(), 0.1], 0.0), Some("error-handler-joint-stop"));
    assert_eq!(d.unreachable(&[0.808, 0.0, 0.1], 0.0), Some("error-handler-singularity"));
    assert_eq!(d.unreachable(&[0.806, 0.0, 0.1], 0.0), None);
    assert_eq!(d.unreachable(&[0.5, 0.0], 0.0), Some("error-target-out-of-range"));
}

/// The inertia the R motor feels, by hand (beta = gamma = 0, theta at rest): turning phi alone, the upper arm turns
/// about the shoulder (m1 c1^2 + I1), the forearm's centre moves at L n1 - c2 n2 (adding m2 (L^2 + c2^2 - 2 L c2 cos
/// 2 phi) and I2), the effector and the wafer slide along the radial line without turning at 2 L sin phi per rad. So
/// M_phiphi = m1 c1^2 + I1 + m2 (L^2 + c2^2 - 2 L c2 cos 2 phi) + I2 + (m3 + mw) 4 L^2 sin^2 phi. Retracted (r = 0.12,
/// phi = 95.379379 degrees): 0.5499656 kg m^2, 1.354595 kg along r; at the motor M / 50^2 + J_rotor = 2.485862e-4 kg
/// m^2. Extended (r = 0.75, phi = 27.048111): 0.1876677 kg m^2 but 2.215676 kg along r (dr/dphi 0.291033), at the
/// motor 1.036671e-4. So r'' = 1 m/s^2 from rest takes 2.485862e-4 * 50 / (-0.637181) = -0.019506 N m and
/// 1.036671e-4 * 50 / (-0.291033) = -0.017810 N m at motor 2, and M_thetaphi / 50 / (dr/dphi) at motor 1, M_thetaphi being
/// 0.13312 at every pose.
#[test]
fn the_r_motor_feels_the_hand_calculated_inertia_retracted_and_extended() {
    let d = handler(&RIGID);
    let (l, c2) = (0.32, 0.16);
    for (r, m_by_hand, at_motor, along) in
        [(0.12, 0.5499656, 2.485862e-4, 1.354595), (0.75, 0.1876677, 1.036671e-4, 2.215676)]
    {
        let phi = d.arm.phi(r);
        let m = 1.5 * 0.16 * 0.16
            + 0.0128
            + 1.2 * (l * l + c2 * c2 - 2.0 * l * c2 * (2.0 * phi).cos())
            + 0.01024
            + (0.4 + 0.1276) * 4.0 * l * l * phi.sin().powi(2);
        close(m, m_by_hand, 1e-6, "M_phiphi by hand");
        let (mass, _) = d.arm.dynamics(&[0.0, phi, 0.0, 0.0], &[0.0; 4], true);
        close(mass[1][1], m, 1e-12, "M_phiphi");
        close(mass[0][1], 0.13312, 1e-9, "M_thetaphi");
        let dr = d.arm.reach(phi)[1];
        close(m / (dr * dr), along, 1e-5, "mass along r");
        close(m / 2500.0 + ROTOR, at_motor, 1e-5, "at motor 2");
        let mut torque = [0.0; 3];
        d.feedforward_torque(&[r, 0.0, 0.1], &[1.0, 0.0, 0.0], &mut torque);
        close(torque[1], (m / 2500.0 + ROTOR) * 50.0 / dr, 1e-9, "motor 2 for 1 m/s^2");
        close(torque[0], 0.13312 / 50.0 / dr, 1e-9, "motor 1 for 1 m/s^2");
    }
}

/// The inertia the theta motor feels, by hand: the arm turned whole at fixed phi, J_c + sum (I + m |centre|^2): upper
/// arm m1 c1^2 + I1, forearm m2 (L^2 + c2^2 + 2 L c2 cos 2 phi) + I2, effector m3 (2 L cos phi + c3)^2 + I3, wafer
/// mw r^2 + Iw. Retracted (r = 0.12): 0.153033 kg m^2, at the motor 8.981318e-5; extended (r = 0.75): 0.604751, 2.705002e-4
/// kg m^2. theta'' = 1 rad/s^2 takes at motor 1 50 times that, and at motor 2 M_thetaphi / 50 = 0.0026624 N m.
#[test]
fn the_theta_motor_feels_an_inertia_growing_with_the_reach() {
    let d = handler(&RIGID);
    let (l, c2) = (0.32, 0.16);
    for (r, m_by_hand, at_motor) in [(0.12, 0.153033, 8.981318e-5), (0.75, 0.604751, 2.705002e-4)] {
        let phi = d.arm.phi(r);
        let m = 0.05
            + 1.5 * 0.16 * 0.16
            + 0.0128
            + 1.2 * (l * l + c2 * c2 + 2.0 * l * c2 * (2.0 * phi).cos())
            + 0.01024
            + 0.4 * (2.0 * l * phi.cos() + 0.12).powi(2)
            + 0.004
            + 0.1276 * r * r
            + 1.4355e-3;
        close(m, m_by_hand, 1e-5, "M_thetatheta by hand");
        let mut torque = [0.0; 3];
        d.feedforward_torque(&[r, 0.0, 0.1], &[0.0, 1.0, 0.0], &mut torque);
        close(torque[0], 50.0 * (m / 2500.0 + ROTOR), 1e-9, "motor 1 for 1 rad/s^2");
        close(m / 2500.0 + ROTOR, at_motor, 1e-6, "at motor 1");
        close(torque[1], 0.13312 / 50.0, 1e-6, "motor 2 for 1 rad/s^2");
    }
}

/// Turning at the reach r = 0.75 at the rate the wafer slips at on pads of mu = 0.3, by hand omega = sqrt(mu g / r) =
/// sqrt(0.3 * 9.80665 / 0.75) = 1.980571 rad/s, the wafer accelerates at r omega^2 = mu g = 2.941995 m/s^2 towards the
/// axis, 1.2 times the verdict's limit mu g / 1.2 = 2.451663 m/s^2. Motor 2 holds the arm against the pull outwards:
/// h_phi = -1/2 dM_thetatheta/dphi omega^2, dM_thetatheta/dphi = -4 m2 L c2 sin 2 phi - 4 L sin phi (m3 (2 L cos
/// phi + c3) + mw (2 L cos phi + e)) = -0.415420 kg m^2/rad, so 0.814776 N m at the joint, 0.0162955 N m at the motor. The
/// rigid model under the feedforward of that steady turn keeps r and theta' (no acceleration), and the signal is the
/// wafer's acceleration.
#[test]
fn a_fast_turn_pushes_the_wafer_outwards_and_it_slips_at_the_hand_calculated_rate() {
    let d = handler(&RIGID);
    let (r, mu, g): (f64, f64, f64) = (0.75, 0.3, 9.80665);
    let omega = (mu * g / r).sqrt();
    close(omega, 1.980571, 1e-6, "slip turn rate");
    close(d.slip_limit(), mu * g / 1.2, 1e-12, "limit");
    close(d.slip_limit(), 2.451663, 1e-6, "limit by hand");
    let pose = [r, 0.0, 0.1];
    let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
    d.equilibrium(&pose, &mut state, &mut torque);
    d.set_tool_velocity(&mut state, &[0.0, omega, 0.0]);
    d.feedforward_torque_moving(&pose, &[0.0, omega, 0.0], &[0.0; 3], &mut torque);
    close(torque[1], 0.0162955, 1e-5, "motor 2 holds the arm");
    let mut rates = vec![0.0; d.state_len()];
    d.derivatives(&state, &torque, &mut rates);
    assert!(rates[2].abs() < 1e-9 && rates[3].abs() < 1e-9, "{rates:?}");
    close(signal(&d, &state, &torque, "wafer.acceleration"), mu * g, 1e-9, "the wafer's acceleration");
    let check = d.rating_checks().into_iter().find(|c| c.check == "wafer-slip").unwrap();
    assert_eq!((check.signal.as_str(), check.limit), ("wafer.acceleration", d.slip_limit()));
}

/// Extending during a turn: at r = 0.5 moving out at 0.5 m/s while turning at 1 rad/s, no acceleration of r or theta,
/// the wafer accelerates at r'' - r theta'^2 = -0.5 m/s^2 along the radial line and 2 r' theta' + r theta'' = 1 m/s^2
/// across it (Coriolis): 1.118034 m/s^2.
#[test]
fn extending_during_a_turn_gives_the_wafer_the_coriolis_acceleration() {
    let d = handler(&RIGID);
    let (pose, vel) = ([0.5, 0.0, 0.1], [0.5, 1.0, 0.0]);
    let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
    d.equilibrium(&pose, &mut state, &mut torque);
    d.set_tool_velocity(&mut state, &vel);
    d.feedforward_torque_moving(&pose, &vel, &[0.0; 3], &mut torque);
    let acc = d.rigid_solve(&state, &torque).0;
    let (q, qd) = d.coordinates(&state);
    let a = d.wafer_acceleration(&q, &qd, &acc);
    assert!((a[0] + 0.5).abs() < 1e-9 && (a[1] - 1.0).abs() < 1e-9, "{a:?}");
    close(signal(&d, &state, &torque, "wafer.acceleration"), 1.118034, 1e-6, "|a|");
}

/// The bands under a torque T = 1 N m on the effector at its wrist, with the column and the upper arm held, by hand:
/// K1 = 2 k r1^2 = 2 * 9.4e5 * 0.03^2 = 1692 N m/rad, K2 = 2 * 9.4e5 * 0.035^2 = 2303 N m/rad. The wrist band takes T:
/// beta / 2 - gamma = -T / K2; the elbow band passes it on to the column, r1 dT1 = r2 dT2 / 2: beta = T / (2 K1) =
/// 2.955083e-4 rad, gamma = T / (4 K1) + T / K2 = 5.819704e-4 rad. Then the bands push back with exactly -T on gamma and
/// nothing on beta; their tension swings are k s / T0 = -T / (4 r1 T0) = -0.0555556 and -T / (2 r2 T0) = -0.0952381.
/// At r = 0.75, theta = 0 the wafer centre is at (0.750043, 1.889809e-4): the effector turned out of its radial line by
/// 0.18 * 5.819704e-4 / 0.75 less, 3.300103e-4 rad (by the geometry of the bands' stretch, `C = L u(phi) + L u(-phi +
/// beta) + e u(gamma)`).
#[test]
fn the_bands_stretch_under_a_torque_and_turn_the_effector_out_of_its_line() {
    let d = handler(&[]);
    let (t, k) = (1.0, 9.4e5);
    let (k1, k2) = (2.0 * k * 0.03 * 0.03, 2.0 * k * 0.035 * 0.035);
    close(d.bands()[0].torsional_stiffness(), 1692.0, 1e-12, "K1");
    close(d.bands()[1].torsional_stiffness(), 2303.0, 1e-12, "K2");
    let (beta, gamma) = (t / (2.0 * k1), t / (4.0 * k1) + t / k2);
    close(beta, 2.955083e-4, 1e-6, "beta");
    close(gamma, 5.819704e-4, 1e-6, "gamma");
    let phi = d.arm.phi(0.75);
    let q = [0.0, phi, beta, gamma];
    let forces = d.band_forces(&q, &[0.0; 4]);
    assert!(forces[2].abs() < 1e-12 && (forces[3] + t).abs() < 1e-12, "{forces:?}");
    let shares = d.band_shares(&q, &[0.0; 4], &[0.0; 4], true);
    close(shares[0], -0.0555556, 1e-6, "elbow band");
    close(shares[1], -0.0952381, 1e-6, "wrist band");
    let centre = d.arm.wafer_centre(&q);
    assert!((centre[0] - 0.750043).abs() < 1e-6 && (centre[1] - 1.889809e-4).abs() < 1e-9, "{centre:?}");
    close(d.direction_error(&q, &[0.0; 4]), 3.300103e-4, 1e-6, "direction error");
    // The signal at a station is that error while the cycle dwells, none while it moves.
    let mut m = handler(&[]);
    let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
    m.equilibrium(&[0.75, 0.0, 0.1], &mut state, &mut torque);
    (state[2], state[3]) = (beta, gamma);
    close(signal(&m, &state, &torque, "effector.direction_error"), 3.300103e-4, 1e-6, "signal");
    assert_eq!(signal(&m, &state, &torque, "effector.direction_error_at_station"), 0.0);
    m.set_dwelling(true);
    close(signal(&m, &state, &torque, "effector.direction_error_at_station"), 3.300103e-4, 1e-6, "at a station");
}

/// A span's tension T0 + k s: 1e-4 m stretches one span to 150 + 94 = 244 N and leaves the other at 56 N, a
/// difference of 2 k s = 188 N and k s^2 = 9.4e-3 J in both; at 2e-4 m the other runs slack (150 - 188 < 0): 338 N
/// against none, and its energy stays at -T0^2 / (2 k) = -0.0119681 J from there on: 150 * 2e-4 + 0.5 * 9.4e5 * 4e-8
/// - 0.0119681 = 0.0488 - 0.0119681 = 0.0368319 J.
#[test]
fn a_band_span_runs_slack_beyond_its_pretension() {
    let band = handler(&[]).bands()[0];
    close(band.difference(1e-4, 0.0), 188.0, 1e-12, "both spans taut");
    close(band.energy(1e-4), 9.4e-3, 1e-12, "energy taut");
    close(band.difference(2e-4, 0.0), 338.0, 1e-12, "one span slack");
    assert_eq!(band.span_tension(-2e-4, 0.0), 0.0);
    close(band.energy(2e-4), 0.0368319, 1e-6, "energy with one slack");
}

/// The forces the rigid bands carry, by hand, with the wrist over the column's axis (r = e = 0.18, phi = 90 degrees) at
/// rest and theta'' = 10 rad/s^2: the effector and the wafer turn about the wrist, M_gammatheta = m3 c3^2 + I3 + mw e^2 +
/// Iw = 0.01532974 kg m^2, so the wrist band gives G_gamma = 0.1532974 N m; the forearm's centre moves at c2 n2 per beta
/// and at L n1 + c2 n2 per theta with n1 . n2 = cos 180 = -1: M_betatheta = m2 (c2^2 - L c2) + I2 = -0.02048 kg m^2,
/// G_beta = -0.2048 N m (the effector's share is nil, n2 . n3 = cos 90). dT2 = G_gamma / r2 = 4.379926 N, dT1 = (G_beta +
/// G_gamma / 2) / r1 = -4.271710 N; per span half of it over 150 N: -0.0142390 and 0.0145998. The plan's own rows
/// (`rigid_checks`) and the signals of the rigid model say the same.
#[test]
fn the_rigid_bands_carry_the_hand_calculated_forces() {
    let d = handler(&RIGID);
    let (alpha, r1, r2, t0) = (10.0, 0.03, 0.035, 150.0);
    let (g_gamma, g_beta) = (0.01532974 * alpha, -0.02048 * alpha);
    let (d2, d1) = (g_gamma / r2, (g_beta + 0.5 * g_gamma) / r1);
    close(d2, 4.379926, 1e-6, "dT2");
    close(d1, -4.271710, 1e-6, "dT1");
    let (elbow, wrist) = (0.5 * d1 / t0, 0.5 * d2 / t0);
    close(elbow, -0.0142390, 1e-5, "elbow share");
    close(wrist, 0.0145998, 1e-5, "wrist share");
    let mut out = Vec::new();
    d.rigid_checks(&[0.18, 0.0, 0.1], &[0.0; 3], &[0.0, alpha, 0.0], &mut out);
    let checks = d.rating_checks();
    assert_eq!(out.len(), checks.len());
    let at = |id: &str| out[checks.iter().position(|c| c.check == id).unwrap()];
    close(at("elbow-band-slack"), elbow, 1e-9, "elbow row");
    close(at("wrist-band-slack"), wrist, 1e-9, "wrist row");
    assert!(at("wafer-slip").is_nan() && at("effector-direction").is_nan());
    let pose = [0.18, 0.0, 0.1];
    let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
    d.equilibrium(&pose, &mut state, &mut torque);
    d.feedforward_torque(&pose, &[0.0, alpha, 0.0], &mut torque);
    close(signal(&d, &state, &torque, "band.elbow.pretension_share"), elbow, 1e-9, "elbow signal");
    close(signal(&d, &state, &torque, "band.wrist.pretension_share"), wrist, 1e-9, "wrist signal");
}

/// The Z motor holds all it lifts, by hand m g lead / (2 pi): the carriage 8 kg, the links 1.5 + 1.2 kg, the effector
/// 0.4 kg and the wafer 0.1276 kg, 11.2276 kg * 9.80665 m/s^2 * 0.01 m / (2 pi) = 0.1752378 N m, without the wafer
/// 0.1732462 N m. The arm's motors hold nothing (gravity is out of the plane), and held so nothing moves, rigid or
/// elastic, with friction on.
#[test]
fn the_z_motor_holds_what_it_lifts_and_the_arm_motors_nothing() {
    let by_hand = |m: f64| m * 9.80665 * 0.01 / std::f64::consts::TAU;
    close(by_hand(11.2276), 0.1752378, 1e-6, "by hand");
    close(by_hand(11.1), 0.1732462, 1e-6, "without the wafer");
    // The far end's screw stiffness for the whole stroke: with it following the nut the rotor holds W^2 / (2 E A)
    // less (`Gearbox::held_at`), 1.9e-6 of it.
    for edits in
        [vec!["effects.torque_ripple = false", "effects.screw_stiffness_with_position = false"], RIGID.to_vec()]
    {
        let mut d = handler(&edits);
        for pose in [[0.12, 0.0, 0.1], [0.75, 3.0, 0.2], [0.4, -2.0, 0.05]] {
            let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
            d.equilibrium(&pose, &mut state, &mut torque);
            assert_eq!(&torque[..2], &[0.0, 0.0]);
            close(torque[2], by_hand(11.2276), 1e-9, "Z motor");
            let mut rates = vec![0.0; d.state_len()];
            d.derivatives(&state, &torque, &mut rates);
            assert!(rates.iter().all(|r| r.abs() < 1e-7), "{pose:?}: {rates:?}");
        }
        d.set_payload_attached(false);
        let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
        d.equilibrium(&[0.12, 0.0, 0.1], &mut state, &mut torque);
        close(torque[2], by_hand(11.1), 1e-9, "without the wafer");
    }
}

/// The modes with the motors held, against numpy's eigenvalues of the same M and K (scratch script, not committed:
/// M from the four bodies, K the gearboxes' 2.5e4 N m/rad on theta and phi, the bands' [[K1 + K2 / 4, -K2 / 2], [-K2 / 2,
/// K2]] on beta and gamma). Retracted (r = 0.12): 18.1952, 51.3514, 70.6454, 186.4465 Hz; the bands alone (rigid
/// gearboxes) 21.0252, 62.4203 Hz. Extended (r = 0.75): 16.7646, 42.3221, 83.6696, 175.3323 Hz; bands alone 18.5831,
/// 83.1553 Hz. The rigid model has none.
#[test]
fn the_modes_are_those_of_the_gearboxes_and_bands() {
    let both = handler(&[]);
    let bands = handler(&["effects.joint_elasticity = false"]);
    for (r, all, alone) in [
        (0.12, [18.1952, 51.3514, 70.6454, 186.4465], [21.0252, 62.4203]),
        (0.75, [16.7646, 42.3221, 83.6696, 175.3323], [18.5831, 83.1553]),
    ] {
        // The column on its screw is a fifth mode, far above these (`the_column_rings_on_its_screw_and_coupling`).
        let mut modes: Vec<f64> = both.modes(&[r, 0.0, 0.1]).iter().map(|m| m.hz).collect();
        assert_eq!(modes.len(), 5);
        modes.pop();
        (0..4).for_each(|k| close(modes[k], all[k], 1e-5, "mode"));
        let modes: Vec<f64> = bands.modes(&[r, 0.0, 0.1]).iter().map(|m| m.hz).collect();
        (0..2).for_each(|k| close(modes[k], alone[k], 1e-5, "band mode"));
        close(both.resonance_hz(&[r, 0.0, 0.1]).unwrap(), all[0], 1e-5, "lowest");
    }
    assert!(handler(&RIGID).modes(&[0.5, 0.0, 0.1]).is_empty());
}

/// The modes' damping ratios. With damping proportional to stiffness, C = alpha K (each band span 94 N s/m against its
/// 9.4e5 N/m, the arm's gearboxes 2.5 N m s/rad against 2.5e4 N m/rad: alpha = 1e-4 s), every mode of M and K has the
/// ratio alpha w / 2 = 1e-4 pi f, whatever its shape. The column, by hand: the coupling's 0.2 N m s/rad over the
/// screw's lever (1.591549e-3 m/rad) squared is c_g = 78956.8 N s/m across the coupling's spring alone, K_g =
/// 1.184353e9 N/m; the screw's K_c = 4.747102e7 N/m in series has its own c_c = 2 x 0.04 sqrt(K_c x 11.1 kg) = 1836.394
/// N s/m (`screw.axial_damping_ratio`, the column without the wafer). Together (`Gearbox::arm`): b = c_g c_c / C =
/// 1794.653 N s/m, K_m = (K_g c_c - K_c c_g)^2 / (S C^2) = 307810.0 N/m and tau = C / S = 6.558831e-5 s (S, C the sums),
/// and at the ring w = sqrt(4.564162e7 / 11.2276) = 2016.216 rad/s they damp b + K_m tau / (1 + (w tau)^2) =
/// 1814.495 N s/m, against 2 sqrt(4.564162e7 N/m x 11.2276 kg) = 45274.5 N s/m critical: 0.0400776. The coupling's
/// damper alone gave 0.0025474; across the whole series spring, as the converter had it, 1.743957.
#[test]
fn the_modes_are_damped_by_the_gearboxes_and_bands() {
    // At the far end, as the column's figures (`the_column_rings_stiffer_with_the_nut_near_its_bearing` has it
    // follow the nut).
    let d = handler(&[
        "effects.screw_stiffness_with_position = false",
        "elbow_band.span_damping_n_s_m = 94.0",
        "wrist_band.span_damping_n_s_m = 94.0",
        "gearbox.damping_nm_s_rad = 2.5",
    ]);
    for r in [0.12, 0.75] {
        let modes = d.modes(&[r, 0.0, 0.1]);
        assert_eq!(modes.len(), 5);
        for m in &modes[..4] {
            close(m.damping, 1e-4 * std::f64::consts::PI * m.hz, 1e-6, "proportional damping");
        }
        close(modes[4].damping, 0.0400776, 1e-5, "column damping");
    }
    assert!(handler(&[]).modes(&[0.4, 0.0, 0.1]).iter().all(|m| m.damping > 0.0));
}

/// The column on its screw and coupling (effects.joint_elasticity), by hand: the coupling's 3000 N m/rad at the screw's
/// lever lead / (2 pi) = 1.591549e-3 m/rad is K_c = 3000 / (1.591549e-3)^2 = 1.184353e9 N/m. The screw, fixed-free, is
/// softest with the nut at its far end, 0.4 m from the fixed bearing: the shaft of 13.5 mm root diameter, A = pi
/// 0.0135^2 / 4 = 1.431388e-4 m^2, is E A / L = 2.06e11 x 1.431388e-4 / 0.4 = 7.371649e7 N/m, in series with the nut's
/// 2e8 and the bearings' 4e8 N/m: K_s = 4.747102e7 N/m, and with the coupling K = 4.564162e7 N/m. The column carries
/// 8 + 1.5 + 1.2 + 0.4 kg and the 0.1276 kg wafer, 11.2276 kg, so with the rotor held it rings at
/// sqrt(4.564162e7 / 11.2276) / 2 pi = 320.8907 Hz, the fifth mode, above the arm's four (at most 186 Hz). At rest the
/// spring is compressed by m g / K = 2.412 um (the rotor that far ahead of the column, in rotor radians (2 pi / lead)
/// m g / K), and the state derivative there is zero.
#[test]
fn the_column_rings_on_its_screw_and_coupling() {
    // One stiffness for the stroke, the far end's (`the_column_rings_stiffer_with_the_nut_near_its_bearing` has it
    // follow the nut).
    let d = handler(&["effects.screw_stiffness_with_position = false"]);
    let lever = 0.01 / std::f64::consts::TAU;
    let k = 4.564162e7;
    close(d.lift.joint.gearbox.stiffness, k, 1e-6, "stiffness in series");
    let modes: Vec<f64> = d.modes(&[0.4, 0.0, 0.1]).iter().map(|m| m.hz).collect();
    assert_eq!(modes.len(), 5);
    close(*modes.last().unwrap(), 320.8907, 1e-6, "column mode");
    assert!(handler(&RIGID).modes(&[0.4, 0.0, 0.1]).is_empty());
    let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
    d.equilibrium(&[0.4, 0.0, 0.1], &mut state, &mut torque);
    let x = d.planar_len();
    let twist = state[x + 3] - state[x] / lever;
    close(twist, 11.2276 * 9.80665 / k / lever, 1e-6, "compression");
    let mut rates = vec![0.0; d.state_len()];
    d.derivatives(&state, &torque, &mut rates);
    assert!(rates.iter().all(|r| r.abs() < 1e-6), "{rates:?}");
}

/// The column's ring with the screw's stiffness following the nut (effects.screw_stiffness_with_position), by hand as
/// above with the fixed bearing at z = 0: the shaft E A / a = 2.948660e7 / a N/m with the nut a = z from it, in
/// series with nut, bearings and coupling. z = 0.1: K = 1 / (0.1 / 2.948660e7 + 5e-9 + 2.5e-9 + 8.443430e-10) =
/// 8.520998e7 N/m, sqrt(K / 11.2276) / 2 pi = 438.4518 Hz; z = 0.2: 6.610658e7, 386.1882 Hz; z = 0.3: 5.400018e7,
/// 349.0392 Hz (at the far end 0.4 the 320.8907 Hz above). At rest at z = 0.2 the spring carries the weight W = m g,
/// the nut where the rotor has turned it, 0.2 m plus the compression x: x = W / K (1 + W / E A) (6 pm more than
/// W / K), and the rotor holds W less the place's own W^2 / (2 E A) (`Gearbox::travel_force`): the state derivative is
/// zero.
///
/// The damping follows too (`the_modes_are_damped_by_the_gearboxes_and_bands` at the far end): the screw's own damper
/// keeps its ratio 0.04 on the 11.1 kg, c_c = 2 x 0.04 sqrt(K_c x 11.1) with K_c = 1 / (z / 2.948660e7 + 7.5e-9) =
/// 9.181580e7, 7.001456e7, 5.657992e7 N/m at z = 0.1, 0.2, 0.3; with the coupling's c_g = 78956.8 N s/m across
/// K_g = 1.184353e9 N/m in series, b + K_m tau / (1 + (w tau)^2) over the critical 2 sqrt(K m) is 0.0420995,
/// 0.0409748, 0.0404020 (0.0400776 at the far end): the coupling's damper takes a larger share where the screw is
/// stiffer. The far end's dampers held gave 0.0293, 0.0333, 0.0368.
#[test]
fn the_column_rings_stiffer_with_the_nut_near_its_bearing() {
    let d = handler(&["effects.screw_stiffness_with_position = true", "screw.fixed_bearing_at_m = 0.0"]);
    for (z, hz, zeta) in
        [(0.1, 438.4518, 0.0420995), (0.2, 386.1882, 0.0409748), (0.3, 349.0392, 0.0404020), (0.4, 320.8907, 0.0400776)]
    {
        let column = *d.modes(&[0.4, 0.0, z]).last().unwrap();
        close(column.hz, hz, 1e-6, "column mode along the stroke");
        close(column.damping, zeta, 1e-5, "column damping along the stroke");
    }
    let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
    d.equilibrium(&[0.4, 0.0, 0.2], &mut state, &mut torque);
    let (x, lever) = (d.planar_len(), 0.01 / std::f64::consts::TAU);
    let w: f64 = 11.2276 * 9.80665;
    let compression = w / 6.610658e7 * (1.0 + w / 2.948660e7);
    close(state[x + 3] - state[x] / lever, compression / lever, 1e-6, "compression at 0.2");
    let mut rates = vec![0.0; d.state_len()];
    d.derivatives(&state, &torque, &mut rates);
    assert!(rates.iter().all(|r| r.abs() < 1e-7), "{rates:?}");
}

/// The rest sag's allowance (`rest_sag`) holds the whole stroke with the screw's stiffness following the nut: the lift's
/// half play and its weight m g = 11.2276 x 9.80665 N over the screw where it is softest on the stroke 0..0.3 m, plus a
/// tenth. With the shaft fixed at one end (the shipped fixed bearing at -0.05 m) that is the stroke's top, the nut
/// a = 0.35 m from it: K = 1 / (0.35 / 2.948660e7 + 5e-9 + 2.5e-9 + 8.443430e-10) = 4.946592e7 N/m, not the shaft's
/// far end a = 0.4 m beyond the stroke (4.564162e7, `the_column_rings_on_its_screw_and_coupling`); held at both ends
/// the shaft's middle a = 0.2 m (z = 0.15), a (L - a) / (E A L) = 0.1 / 2.948660e7. The coupling's play of 0.2 rad
/// [test value] is 0.1 x 0.01 / 2 pi = 1.591549e-4 m at the column, which lifts the allowance over the 0.1 mm floor;
/// the shipped gears have none (0.1 mm before and after).
#[test]
fn the_rest_sag_takes_the_screw_where_it_is_softest_on_the_stroke() {
    let (ea, weight, half) = (2.948660e7, 11.2276 * 9.80665, 0.1 * 0.01 / std::f64::consts::TAU);
    let series = |shaft: f64| 1.0 / (shaft + 5e-9 + 2.5e-9 + 8.443430e-10);
    for (mounting, k) in [("fixed-free", series(0.35 / ea)), ("fixed-fixed", series(0.1 / ea))] {
        let d = handler(&[
            "effects.screw_stiffness_with_position = true",
            "gearbox.3.backlash_rad = 0.2",
            &format!("screw.mounting = \"{mounting}\""),
        ]);
        close(d.rest_sag(), 1.1 * (half + weight / k), 1e-6, mounting);
    }
}

/// The planner's trains (`Machine::trains`) are the machine's own held modes, one motor at a time: the lift's is the
/// column's ring on its screw at the stroke's height (438.4518 and 349.0392 Hz at z = 0.1 and 0.3 by hand above, the
/// fixed bearing at z = 0); the turn's and the reach's are the arm's mode on that joint's gearbox with the other one
/// rigid (its stiffness raised to 1e12 N m/rad) and the bands rigid, retracted and extended. The rigid model has none.
#[test]
fn the_trains_are_the_held_modes_one_motor_at_a_time() {
    let edits = ["effects.band_elasticity = false", "screw.fixed_bearing_at_m = 0.0"];
    let d = handler(&edits);
    for (r, z, column) in [(0.12, 0.1, 438.4518), (0.75, 0.3, 349.0392)] {
        let pose = [r, 0.0, z];
        let trains = d.trains(&pose);
        assert_eq!(trains.len(), 3);
        close(trains[2].unwrap().hz, column, 1e-6, "lift's train");
        for k in 0..2 {
            let mut held = handler(&edits);
            held.joints[1 - k].gearbox.stiffness = 1e12;
            let own = held.modes(&pose)[0].hz;
            close(trains[k].unwrap().hz, own, 1e-6, "joint's train");
        }
    }
    assert!(handler(&RIGID).trains(&[0.4, 0.0, 0.1]).is_empty());
}

/// With the screw's stiffness following the nut, the loss-free configs keep their energy and the default config's
/// budget closes (`testkit`): the spring's energy at the nut's travel and the force that travel takes from it agree.
#[test]
fn the_energy_closes_with_the_screw_stiffness_following_the_nut() {
    for mut cfg in suite_configs().into_iter().chain([ENTRY.config()]) {
        cfg.set_text("effects.screw_stiffness_with_position = true").unwrap();
        let m = WaferHandler::new(&cfg).unwrap();
        if !cfg.bool("effects.joint_friction").unwrap() {
            sim_core::testkit::energy_conservation(&m);
        }
        sim_core::testkit::loss_budget(&m);
    }
}

/// With a hysteresis in the gears and the lift's coupling (`gearbox.hysteresis_rad` 1e-3, which the coupling takes from
/// [gearbox]; the standard suite cannot tell, it never commits the gears' states) the default cycle's budget closes as
/// without it: the hysteresis's state behind the screw lives on the gear's own twist (`Gearbox::at`), and the work the
/// nut's place does on it is booked as stored. Measured: 6.4e-6 J of 48.13 J unaccounted (4.7e-6 J without the hysteresis,
/// 5.1e-6 J on the twist through gear and screw before 21 November), the gearbox rows 1.7e-3 J more.
#[test]
fn the_budget_closes_with_a_hysteresis_behind_the_screw() {
    let budget = |hysteresis: bool| {
        let mut cfg = ENTRY.config();
        if hysteresis {
            cfg.set_text("gearbox.hysteresis_rad = 1e-3").unwrap();
            assert!(WaferHandler::new(&cfg).unwrap().lift.joint.gearbox.hysteresis > 0.0);
        }
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        sim_components::analysis::analyse_cycle(&mut sim).unwrap().budget.unwrap()
    };
    let (plain, looped) = (budget(false), budget(true));
    let gears = |b: &sim_components::analysis::EnergyBudget| -> f64 {
        b.rows.iter().filter(|r| r.id.contains("gearbox")).map(|r| r.energy).sum()
    };
    println!(
        "{} J of {} J unaccounted, gearboxes {} J against {} J",
        looped.unaccounted(),
        looped.supply,
        gears(&looped),
        gears(&plain)
    );
    assert!(looped.unaccounted().abs() < 1e-6 * looped.supply, "{} J of {} J", looped.unaccounted(), looped.supply);
    assert!(gears(&looped) > gears(&plain));
}

/// The hold at rest and on the move, by hand: pads mu g = 0.3 * 9.80665 = 2.941995 m/s^2, and while the lift
/// accelerates down at 1 m/s^2 an in-plane 2 m/s^2 asks as much as 2 * 9.80665 / 8.80665 = 2.227100 at rest. Vacuum
/// pads of 3 cm^2 at 60 kPa pull 18 N on the 0.1276 kg wafer, 141.065831 m/s^2: mu (g + 141.065831) = 45.261744 m/s^2.
/// An edge grip holds its 9.81 m/s^2 whatever the lift does.
#[test]
fn the_hold_asks_the_hand_calculated_share_on_pads_vacuum_and_edge() {
    let g = 9.80665;
    let pads = handler(&[]).hold;
    close(pads.at_rest(g), 2.941995, 1e-6, "pads at rest");
    close(pads.equivalent(2.0, -1.0, g), 2.227100, 1e-6, "pads lowering");
    let vacuum = handler(&["wafer.hold = \"vacuum\""]).hold;
    close(vacuum.at_rest(g), 45.261744, 1e-6, "vacuum at rest");
    let edge = handler(&["wafer.hold = \"edge\""]).hold;
    assert_eq!((edge.at_rest(g), edge.equivalent(2.0, -1.0, g)), (9.81, 2.0));
    assert!(
        WaferHandler::new(&{
            let mut cfg = ENTRY.config();
            cfg.set_text("wafer.hold = \"glue\"").unwrap();
            cfg
        })
        .is_err()
    );
}

/// The default cycle passes every verdict (`sim-cli verdicts wafer-handler`: the wafer at 1.83 m/s^2 of the 2.45 its
/// pads hold over 1.2, settling 0.033 mm of 0.15 and 0.113 mrad of 0.2 (the column ringing after the first turn), the
/// effector's direction 12.7 urad of 0.5 mrad, the bands' swings 6.5 % and 1.2 % of their pretension); the reach speeding up at 3 m/s^2 instead of
/// 1.5 (its jerk 30 m/s^3) slips the wafer: 3 m/s^2 along the radial line, more than the 2.45 allowed.
#[test]
fn the_default_cycle_passes_its_verdicts_and_a_faster_reach_slips_the_wafer() {
    let judge = |edits: &[&str]| {
        let mut cfg = ENTRY.config();
        for e in edits {
            cfg.set_text(e).unwrap();
        }
        let load = sim_components::analysis::analyse_cycle(&mut Sim::new(&ENTRY, &cfg).unwrap()).unwrap();
        sim_components::analysis::verdicts(&cfg, &load).unwrap()
    };
    let verdicts = judge(&[]);
    let failed: Vec<_> = verdicts.iter().filter(|v| !v.ok).collect();
    assert!(failed.is_empty(), "{failed:?}");
    for check in ["wafer-slip", "effector-direction", "elbow-band-slack", "wrist-band-slack", "screw-speed", "settling"]
    {
        assert!(verdicts.iter().any(|v| v.check == check), "no {check}");
    }
    let fast = judge(&["axis.r.max_acceleration_m_s2 = 3.0", "axis.r.max_jerk_m_s3 = 30.0"]);
    let slip = fast.iter().find(|v| v.check == "wafer-slip").unwrap();
    assert!(!slip.ok && slip.value > 2.9, "{slip:?}");
}
