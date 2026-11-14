//! Stage 1 of the fit in the motors' own coordinates (`identify::fit_joints`) on the machines whose transformation is
//! not linear: a trace of a machine with another payload and more friction than the config, with noise on it, fitted
//! against the unchanged config.

use sim_components::Command;
use sim_components::identify::{JointFit, fit_joints};
use sim_components::sim::Sim;
use sim_components::trace::{Trace, per_axis_fit};
use sim_core::testkit::Noise;
use sim_core::{Config, MachineEntry};

/// An exciting trajectory, as robot identification takes it: every axis swings about the start pose with its own
/// amplitude and frequency (`swing`: amplitude, Hz), faded in and out over a second, for `seconds`; recorded every
/// millisecond, then noise on what a drive measures (torque, speed, angle) of the size the single axis's fit takes,
/// the torque's scaled to the motor.
fn noisy_trace(
    entry: &MachineEntry,
    cfg: &Config,
    swing: &[(f64, f64)],
    seconds: f64,
    torque_noise: f64,
    noise: &mut Noise,
) -> Trace {
    let mut sim = Sim::new(entry, cfg).unwrap();
    let n = swing.len();
    // About where the setpoint stands, which a trajectory has to start from.
    let setpoint: Vec<f64> = sim.control.drives.iter().map(|d| d.pos_set).collect();
    let mut pose0 = vec![0.0; n];
    sim.machine.forward(&setpoint, &mut pose0);
    let points: Vec<(f64, Vec<f64>)> = (0..=(seconds * 100.0) as usize)
        .map(|i| {
            let t = i as f64 * 0.01;
            let fade = (t.min(seconds - t).min(1.0) * std::f64::consts::PI / 2.0).sin().powi(2);
            (t, (0..n).map(|a| pose0[a] + fade * swing[a].0 * (std::f64::consts::TAU * swing[a].1 * t).sin()).collect())
        })
        .collect();
    sim.command(Command::Trajectory(points)).unwrap();
    let mut trace = Trace::default();
    for _ in 0..(seconds * 1000.0) as usize {
        trace.record(&sim);
        sim.run(1e-3, |_| {});
    }
    for m in &mut trace.motors {
        for row in 0..m.torque.len() {
            m.torque[row] += torque_noise * noise.next();
            m.speed[row] += 0.1 * noise.next();
            m.angle[row] += 1e-5 * noise.next();
        }
    }
    trace
}

fn scale(cfg: &mut Config, paths: &[&str], factor: f64) {
    for path in paths {
        cfg.set_text(&format!("{path} = {:?}", cfg.f64(path).unwrap() * factor)).unwrap();
    }
}

/// The value named `quantity` of `of`: (config, fitted, error).
fn value(fit: &JointFit, quantity: &str, of: &str) -> (f64, f64, f64) {
    let v = fit
        .values
        .iter()
        .find(|v| v.quantity == quantity && v.of == of)
        .unwrap_or_else(|| panic!("{quantity} {of} missing"));
    (v.config, v.fitted, v.error)
}

/// Asserts that the change the fit finds in `quantity` of motor `of` (fitted less config) is `expected` within three of
/// its standard errors.
fn change_is(fit: &JointFit, quantity: &str, of: &str, expected: f64) {
    let (config, fitted, error) = value(fit, quantity, of);
    assert!(
        (fitted - config - expected).abs() < 3.0 * error,
        "{quantity} {of}: {:.4e} ± {error:.2e}, by hand {expected:.4e}",
        fitted - config
    );
}

/// The payload's factor, fitted over the config's, is `expected` within three standard errors, and the error is below
/// 2 % of it.
fn payload_is(fit: &JointFit, expected: f64) {
    let (config, fitted, error) = value(fit, "payload_factor", "payload");
    let (factor, band) = (fitted / config, error / config);
    assert!(
        (factor - expected).abs() < 3.0 * band && band < 0.02 * factor,
        "payload × {factor:.4} ± {band:.4}, in truth × {expected:.4}"
    );
}

/// The delta picker with all its shipped effects (elastic gearboxes with backlash and losses, ripple, the motor's
/// winding): a trace of an exciting trajectory of a machine with 4 kg of payload instead of 3 and twice the joints'
/// Coulomb and viscous friction (0.3 to 0.6 N m, 0.05 to 0.1 N m s/rad), with noise, fitted against the config.
///
/// By hand, at the motor, behind the gearbox of ratio 20 and efficiency 0.96 that the friction torque passes on its way
/// from the motor: payload × 4 / 3; Coulomb friction + 0.3 / 20 / 0.96 = 0.0156 N m; viscous + 0.05 / 20² / 0.96 =
/// 1.30e-4 N m s/rad; no added inertia and no constant torque. Measured (13 November): × 1.329 ± 0.014; Coulomb + 0.021,
/// 0.019, 0.020 ± 0.003 (the heavier payload loads the joints' friction a little too); viscous + 0.6e-4 to 1.0e-4 ±
/// 0.9e-4 to 1.3e-4. Nothing the trace cannot tell apart.
#[test]
fn the_delta_fit_finds_a_heavier_payload_and_more_joint_friction() {
    let entry = sim_machines::find("delta-d3").unwrap();
    let cfg = entry.config();
    // Neither the per-axis fit nor its refusal: the arms' angles are not linear in the platform's pose.
    assert!(!per_axis_fit((entry.build)(&cfg).unwrap().as_ref()));
    assert_eq!(
        sim_components::identify::fit_rigid(entry, &cfg, &Trace::default()).err().map(|e| e.key),
        Some("error-trace-not-linear")
    );
    let mut truth = cfg.clone();
    scale(&mut truth, &["mass.payload_kg"], 4.0 / 3.0);
    scale(&mut truth, &["joint.friction_coulomb_nm", "joint.friction_viscous_nm_s_rad"], 2.0);
    // 0.2 m across at 0.6 and 0.9 Hz, 0.06 m up and down at 1.1 Hz.
    let trace = noisy_trace(entry, &truth, &[(0.2, 0.6), (0.2, 0.9), (0.06, 1.1)], 8.0, 0.01, &mut Noise(1));
    let fit = fit_joints(entry, &cfg, &trace).unwrap().unwrap();
    payload_is(&fit, 4.0 / 3.0);
    for motor in ["1", "2", "3"] {
        change_is(&fit, "coulomb_friction_nm", motor, 0.3 / 20.0 / 0.96);
        change_is(&fit, "viscous_friction_nm_s_rad", motor, 0.05 / 400.0 / 0.96);
        change_is(&fit, "added_inertia_kg_m2", motor, 0.0);
        change_is(&fit, "constant_torque_nm", motor, 0.0);
    }
    assert!(fit.alike.is_empty(), "{:?}", fit.advice());
    // The advice says the payload's factor first, then per motor.
    assert_eq!(fit.advice()[0].0, "fit-payload");
}

/// The SCARA (joints 1, 2 and 4 behind strain wave gears of ratio 100, 80 and 50, the quill on a screw): 4 kg of
/// payload instead of 3, twice the joints' Coulomb and viscous friction (0.5 to 1 N m, 0.2 to 0.4 N m s/rad), the
/// quill's guide unchanged. Its tool pose is its joints, linear, but the inertia joint 1 turns changes with the elbow,
/// so the per-axis fit refuses it.
///
/// Without the gearboxes' losses, which on these gears (efficiency 0.75) take a quarter of what passes and grow with
/// the heavier payload's torque, so that joint 1's friction came out twice too large (they cannot be told apart from
/// friction, as on the belt axes): by hand at the motor, payload × 4 / 3, Coulomb + 0.5 / ratio (0.005, 0.00625, 0.01
/// N m), viscous + 0.2 / ratio² (2.0e-5, 3.1e-5, 8.0e-5 N m s/rad), the quill 0. Measured (13 November): × 1.335 ±
/// 0.006, Coulomb + 0.00514, 0.00621, 0.00999, viscous + 1.9e-5, 3.3e-5, 7.9e-5. With the losses the payload still
/// comes out × 1.331 ± 0.017.
#[test]
fn the_scara_fit_finds_a_heavier_payload_and_more_joint_friction() {
    let entry = sim_machines::find("scara").unwrap();
    let mut cfg = entry.config();
    assert!(!per_axis_fit((entry.build)(&cfg).unwrap().as_ref()));
    cfg.set_text("effects.gearbox_efficiency = false").unwrap();
    let mut truth = cfg.clone();
    scale(&mut truth, &["arm.payload_kg"], 4.0 / 3.0);
    scale(&mut truth, &["joint.friction_coulomb_nm", "joint.friction_viscous_nm_s_rad"], 2.0);
    // About the start pose [0, 0.5, 0.05, 0]; the elbow stays bent, between 0.15 and 0.85 rad.
    let trace =
        noisy_trace(entry, &truth, &[(0.8, 0.5), (0.35, 0.7), (0.04, 0.9), (0.8, 1.1)], 8.0, 0.01, &mut Noise(1));
    let fit = fit_joints(entry, &cfg, &trace).unwrap().unwrap();
    payload_is(&fit, 4.0 / 3.0);
    for (motor, ratio) in [("1", 100.0), ("2", 80.0), ("4", 50.0)] {
        change_is(&fit, "coulomb_friction_nm", motor, 0.5 / ratio);
        change_is(&fit, "viscous_friction_nm_s_rad", motor, 0.2 / (ratio * ratio));
    }
    change_is(&fit, "coulomb_friction_nm", "3", 0.0);
    change_is(&fit, "viscous_friction_nm_s_rad", "3", 0.0);
    for motor in ["1", "2", "3", "4"] {
        change_is(&fit, "added_inertia_kg_m2", motor, 0.0);
        change_is(&fit, "constant_torque_nm", motor, 0.0);
    }
    assert!(fit.alike.is_empty(), "{:?}", fit.advice());
}

/// A trace in which a motor turns one way only cannot tell its Coulomb friction from a constant torque: no fit.
#[test]
fn a_joint_fit_needs_every_motor_to_turn_both_ways() {
    let entry = sim_machines::find("delta-d3").unwrap();
    let cfg = entry.config();
    let mut trace = noisy_trace(entry, &cfg, &[(0.2, 0.6), (0.2, 0.9), (0.06, 1.1)], 3.0, 0.01, &mut Noise(3));
    let m = &mut trace.motors[0];
    m.speed.iter_mut().for_each(|v| *v = v.abs());
    assert_eq!(fit_joints(entry, &cfg, &trace).unwrap(), None);
}
