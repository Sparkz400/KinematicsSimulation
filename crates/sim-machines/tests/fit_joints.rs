//! Stage 1 of the fit in the motors' own coordinates (`identify::fit_joints`) on the machines whose transformation is
//! not linear: a trace of a machine with another payload, more friction or heavier links than the config, or with a
//! payload picked during it, with noise on it, fitted against the unchanged config.

use sim_components::Command;
use sim_components::identify::{JointFit, JointValue, fit_joints};
use sim_components::sim::Sim;
use sim_components::trace::{Trace, per_axis_fit};
use sim_core::testkit::Noise;
use sim_core::{Config, MachineEntry};

/// An exciting trajectory, as robot identification takes it: every axis swings about the start pose with its own
/// amplitude and frequency (`swing`: amplitude, Hz), faded in and out over a second, for `seconds`; recorded every
/// millisecond, then noise on what a drive measures (torque, speed, angle) of the size the single axis's fit takes,
/// the torque's scaled to the motor. With `held`, the payload is held at the times it says.
fn noisy_trace(
    entry: &MachineEntry,
    cfg: &Config,
    swing: &[(f64, f64)],
    seconds: f64,
    torque_noise: f64,
    noise: &mut Noise,
    held: Option<fn(f64) -> bool>,
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
        if let Some(held) = held {
            sim.control.payload_attached = held(sim.time());
        }
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
    let trace = noisy_trace(entry, &truth, &[(0.2, 0.6), (0.2, 0.9), (0.06, 1.1)], 8.0, 0.01, &mut Noise(1), None);
    let fit = fit_joints(entry, &cfg, &trace).unwrap().unwrap();
    payload_is(&fit, 4.0 / 3.0);
    for motor in ["1", "2", "3"] {
        change_is(&fit, "coulomb_friction_nm", motor, 0.3 / 20.0 / 0.96);
        change_is(&fit, "viscous_friction_nm_s_rad", motor, 0.05 / 400.0 / 0.96);
        change_is(&fit, "added_inertia_kg_m2", motor, 0.0);
        change_is(&fit, "constant_torque_nm", motor, 0.0);
    }
    assert!(fit.alike.is_empty(), "{:?}", fit.advice());
    assert_eq!(fit.payload_steps, vec![]);
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
        noisy_trace(entry, &truth, &[(0.8, 0.5), (0.35, 0.7), (0.04, 0.9), (0.8, 1.1)], 8.0, 0.01, &mut Noise(1), None);
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
    let mut trace = noisy_trace(entry, &cfg, &[(0.2, 0.6), (0.2, 0.9), (0.06, 1.1)], 3.0, 0.01, &mut Noise(3), None);
    let m = &mut trace.motors[0];
    m.speed.iter_mut().for_each(|v| *v = v.abs());
    assert_eq!(fit_joints(entry, &cfg, &trace).unwrap(), None);
}

/// The config with the URDF's links named in `changes` made heavier: each one's mass and inertia (about its centre,
/// which stays) times its factor, so that its mass, first moment and inertia about its frame all change by the factor,
/// as `link_mass_factor` takes it. The built-in file `urdf` changed so and written to the temporary directory as
/// `file`. Every quoted single number in the link's block is scaled: in these files, its mass and inertia only (an
/// origin has three).
fn with_links(cfg: &Config, urdf: &str, changes: &[(&str, f64)], file: &str) -> Config {
    let mut text = urdf.to_string();
    for (link, factor) in changes {
        let at = text.find(&format!("<link name=\"{link}\">")).unwrap();
        let end = at + text[at..].find("</link>").unwrap();
        let block: Vec<String> = text[at..end]
            .split('"')
            .enumerate()
            .map(|(i, part)| match part.parse::<f64>() {
                Ok(v) if i % 2 == 1 => format!("{:?}", v * factor),
                _ => part.to_string(),
            })
            .collect();
        text.replace_range(at..end, &block.join("\""));
    }
    let path = std::env::temp_dir().join(format!("{file}-{}.urdf", std::process::id()));
    std::fs::write(&path, text).unwrap();
    let mut cfg = cfg.clone();
    cfg.set_text(&format!("arm.urdf_file = {:?}", path.to_str().unwrap())).unwrap();
    cfg
}

/// Asserts that each base parameter changed (fitted less config) by what `changes` (`quantity of`, its change in
/// truth; all others 0) make of it through what it combines, within three of its standard errors.
fn base_changes_are(fit: &JointFit, changes: &[(&str, f64)]) {
    let truth = |name: &str| changes.iter().find(|(n, _)| *n == name).map_or(0.0, |c| c.1);
    for v in &fit.values {
        let expected = truth(&v.name()) + v.combines.iter().map(|(n, c)| c * truth(n)).sum::<f64>();
        assert!(
            (v.fitted - v.config - expected).abs() < 3.0 * v.error,
            "{}: {:+.4e} ± {:.2e}, in truth {expected:+.4e}; {:?}",
            v.name(),
            v.fitted - v.config,
            v.error,
            v.combines
        );
    }
}

/// The value named `quantity of` of the fit, if it is one of its own (not folded into others).
fn own<'a>(fit: &'a JointFit, name: &str) -> Option<&'a JointValue> {
    fit.values.iter().find(|v| v.name() == name)
}

/// The factor `folded` (a `quantity of`) enters the value `into` by.
fn folded_into(fit: &JointFit, folded: &str, into: &str) -> f64 {
    let v = own(fit, into).unwrap_or_else(|| panic!("{into} missing"));
    v.combines.iter().find(|(n, _)| n == folded).unwrap_or_else(|| panic!("{folded} not in {into}: {:?}", v.combines)).1
}

/// The SCARA with its second arm 1.25 times and its quill 1.5 times as heavy as the config says (mass and inertia),
/// gearbox losses off as in the friction test, the same swing and noise.
///
/// By hand, what the rigid model itself cannot tell apart: arm 1 turns about joint 1's fixed vertical axis only, so its
/// mass factor adds its inertia about that axis, I_zz + m (c_x² + c_y²) = 0.1 + 8 · 0.16² = 0.3048 kg m², over the
/// ratio squared, 0.3048 / 100² = 3.048e-5 kg m², to motor 1's inertia, and nothing else. Quill, flange and payload sit
/// on the quill's vertical axis: each one's mass acts as the payload's does (their small inertias about that axis
/// aside), so their factors enter the payload's with their masses over the payload's, 1.5 / 3 = 0.5 and 1 / 3. The arm's
/// own mass and first moment act at the elbow's distance l1 = 0.325 m and with the elbow's angle, as the end's do; the
/// trace does not tell them from the payload with the motors' values (they would inflate its variance more than
/// `WEAK`), so they are folded into those. Measured (14 November): the payload's value + 0.445 ± 0.007 for 0.441 from the
/// truth through what it combines (0.764 × 0.25 + 0.501 × 0.5); quill 0.5010, flange 0.3297.
#[test]
fn the_scara_fit_finds_heavier_links_in_the_values_they_combine_with() {
    let entry = sim_machines::find("scara").unwrap();
    let mut cfg = entry.config();
    cfg.set_text("effects.gearbox_efficiency = false").unwrap();
    let urdf = include_str!("../../machine-joint-kit/urdf/scara.urdf");
    let truth = with_links(&cfg, urdf, &[("arm2", 1.25), ("quill", 1.5)], "scara-links");
    let trace =
        noisy_trace(entry, &truth, &[(0.8, 0.5), (0.35, 0.7), (0.04, 0.9), (0.8, 1.1)], 8.0, 0.01, &mut Noise(1), None);
    let fit = fit_joints(entry, &cfg, &trace).unwrap().unwrap();
    let coefficient = folded_into(&fit, "link_mass_factor arm1", "added_inertia_kg_m2 1");
    assert!((coefficient / 3.048e-5 - 1.0).abs() < 1e-6, "{coefficient:e}");
    for (link, share) in [("quill", 0.5), ("flange", 1.0 / 3.0)] {
        let coefficient = folded_into(&fit, &format!("link_mass_factor {link}"), "payload_factor payload");
        assert!((coefficient / share - 1.0).abs() < 0.02, "{link}: {coefficient}");
    }
    for name in ["link_mass_factor arm1", "link_mass_factor arm2", "link_mass_factor quill"] {
        assert!(own(&fit, name).is_none() && fit.folded.iter().any(|f| f == name), "{name}: {:?}", fit.advice());
    }
    base_changes_are(&fit, &[("link_mass_factor arm2", 0.25), ("link_mass_factor quill", 0.5)]);
    assert!(own(&fit, "payload_factor payload").unwrap().error < 0.01);
    assert_eq!(fit.payload_steps, vec![]);
    // The advice names what each value combines.
    assert!(
        fit.advice().iter().any(|(key, detail)| *key == "fit-combined" && detail.contains("link_mass_factor quill"))
    );
}

/// The six-axis arm with link 2 1.2 times and link 3 1.3 times as heavy (mass and inertia), every joint swung about the
/// start pose, gearbox losses off; its gears rigid (`joint_elasticity` off; the shipped elastic gears in
/// `the_arm_fit_with_elastic_gears_finds_heavier_links`).
///
/// By hand: link 1 turns about joint 1's fixed vertical axis, its centre on it: its factor adds I_zz = 0.25 kg m² over
/// 120² = 1.736e-5 kg m² to motor 1's inertia. Link 2 swings about joint 2's horizontal axis under gravity, so its factor
/// is a value of its own; link 3's factor is folded into link 2's and its own first moments. Measured (14 November):
/// link 2's value + 0.577 ± 0.013 for 0.2 + 1.217 × 0.3 = 0.565 through what it combines; link 3's first moment along
/// x + 0.197 ± 0.017 for 0.665 × 0.3 = 0.200.
///
/// The same arm with its shoulder (joint 2) held within 0.01 rad: link 2's weight then pulls on joint 2 nearly the
/// same all the time, like a constant torque on motor 2, and its factor is folded, not fitted.
#[test]
fn the_arm_fit_finds_heavier_links_and_folds_one_its_trace_does_not_excite() {
    let entry = sim_machines::find("urdf-arm").unwrap();
    let mut cfg = entry.config();
    cfg.set_text("effects.gearbox_efficiency = false").unwrap();
    cfg.set_text("effects.joint_elasticity = false").unwrap();
    let urdf = include_str!("../../machine-joint-kit/urdf/six-axis.urdf");
    let truth = with_links(&cfg, urdf, &[("link2", 1.2), ("link3", 1.3)], "arm-links");
    let mut swing = vec![(0.5, 0.5), (0.3, 0.7), (0.4, 0.6), (0.5, 0.8), (0.4, 0.9), (0.5, 1.0)];
    let trace = noisy_trace(entry, &truth, &swing, 8.0, 0.02, &mut Noise(1), None);
    let fit = fit_joints(entry, &cfg, &trace).unwrap().unwrap();
    let coefficient = folded_into(&fit, "link_mass_factor link1", "added_inertia_kg_m2 1");
    assert!((coefficient / (0.25 / 14400.0) - 1.0).abs() < 1e-6, "{coefficient:e}");
    let link2 = own(&fit, "link_mass_factor link2").unwrap();
    assert!(link2.error < 0.02, "{link2:?}");
    base_changes_are(&fit, &[("link_mass_factor link2", 0.2), ("link_mass_factor link3", 0.3)]);
    assert_eq!(fit.payload_steps, vec![]);

    swing[1].0 = 0.01;
    let held =
        fit_joints(entry, &cfg, &noisy_trace(entry, &cfg, &swing, 4.0, 0.02, &mut Noise(2), None)).unwrap().unwrap();
    assert!(own(&held, "link_mass_factor link2").is_none(), "{:?}", held.advice());
    assert!(held.folded.iter().any(|f| f == "link_mass_factor link2"));
}

/// The delta picker with every shipped effect, the config's own: the gripper empty for the first 3.5 s of the swing and
/// holding its 3 kg from then on. No trace carries a gripper column, so the step is found in the torques.
///
/// By hand: the payload's factor 0 before, 1 after; the step at 3.5 s (the payload is held from the step that starts
/// there, which the next row of a trace every millisecond shows). Measured (14 November): 3.503 s ± 0.001; × −0.014 ±
/// 0.010 before, × 0.988 ± 0.010 after. The joints' Coulomb friction comes out 0.004 to 0.007 N m lower than the
/// config's at the motor: their friction grows with the load they carry, and the config's replay carries the payload
/// all the time.
#[test]
fn a_payload_picked_during_the_trace_is_found_with_when() {
    let entry = sim_machines::find("delta-d3").unwrap();
    let cfg = entry.config();
    let trace = noisy_trace(
        entry,
        &cfg,
        &[(0.2, 0.6), (0.2, 0.9), (0.06, 1.1)],
        8.0,
        0.01,
        &mut Noise(1),
        Some(|t| t >= 3.5 - 1e-9),
    );
    let fit = fit_joints(entry, &cfg, &trace).unwrap().unwrap();
    let [(time, error)] = fit.payload_steps[..] else { panic!("{:?}", fit.payload_steps) };
    assert!((time - 3.5).abs() <= 0.005 && error <= 0.005, "{time} ± {error}");
    for (quantity, expected) in [("payload_factor_before", 0.0), ("payload_factor_after", 1.0)] {
        let (config, fitted, error) = value(&fit, quantity, "payload");
        let (factor, band) = (fitted / config, error / config);
        assert!((factor - expected).abs() < 3.0 * band && band < 0.02, "{quantity}: × {factor:.4} ± {band:.4}");
    }
    assert!(own(&fit, "payload_factor payload").is_none());
    assert_eq!(fit.advice()[0].0, "fit-payload-step");
}

/// The six-axis arm as shipped, its gears elastic (`joint_elasticity` on, 67 000 N m/rad at each joint), with link 2
/// 1.2 and link 3 1.3 times as heavy, the rigid arm's swing and noise, gearbox losses off. Before 14 November this
/// fitted link 2's factor −9.4 ± 0.9 and found a payload step that was not there: the rigid model at the motors' angles
/// took the gears' twist and ringing (joint 2's at some 9 Hz, 0.67 N m RMS at its motor) for the links' motion. Now the
/// load is put where the twist that the motor's torque holds puts it (`Machine::twist`, a two-mass model per joint).
///
/// By hand as for the rigid arm: link 1's factor into motor 1's inertia with 0.25 / 120² (to 1e-6); link 2's value up by
/// 0.2 + 1.217 × 0.3 = 0.565 through what it combines. Measured (14 November): + 0.558 ± 0.025. Tolerance: every base
/// value within three of its standard errors of the truth, link 2's error below 0.04, no payload step.
#[test]
fn the_arm_fit_with_elastic_gears_finds_heavier_links() {
    let entry = sim_machines::find("urdf-arm").unwrap();
    let mut cfg = entry.config();
    assert!(cfg.bool("effects.joint_elasticity").unwrap());
    cfg.set_text("effects.gearbox_efficiency = false").unwrap();
    let urdf = include_str!("../../machine-joint-kit/urdf/six-axis.urdf");
    let truth = with_links(&cfg, urdf, &[("link2", 1.2), ("link3", 1.3)], "elastic-arm-links");
    let swing = [(0.5, 0.5), (0.3, 0.7), (0.4, 0.6), (0.5, 0.8), (0.4, 0.9), (0.5, 1.0)];
    let trace = noisy_trace(entry, &truth, &swing, 8.0, 0.02, &mut Noise(1), None);
    let fit = fit_joints(entry, &cfg, &trace).unwrap().unwrap();
    let coefficient = folded_into(&fit, "link_mass_factor link1", "added_inertia_kg_m2 1");
    assert!((coefficient / (0.25 / 14400.0) - 1.0).abs() < 1e-6, "{coefficient:e}");
    let link2 = own(&fit, "link_mass_factor link2").unwrap();
    assert!(link2.error < 0.04, "{link2:?}");
    base_changes_are(&fit, &[("link_mass_factor link2", 0.2), ("link_mass_factor link3", 0.3)]);
    assert!(fit.payload_steps.is_empty(), "{:?}", fit.payload_steps);
}

/// The delta picker as shipped, its gripper empty for the first 2.5 s of the swing, holding its 3 kg until 5.5 s and
/// empty again from then on: two steps in one trace. By hand: the steps at 2.5 and 5.5 s, each within three of its
/// standard errors, which stay below 20 ms (the regressor is smoothed over ±40 ms), the payload's factor 0 before the first, 1 between (`payload_factor_after` of `payload 1`) and 0 after
/// the second (of `payload 2`), each within three standard errors and with a band below 0.02. Measured (14 November):
/// 2.489 ± 0.019 s and 5.502 ± 0.002 s; × 0.006, 0.990, 0.007, each ± 0.015.
#[test]
fn two_payload_steps_in_one_trace_are_found_with_when() {
    let entry = sim_machines::find("delta-d3").unwrap();
    let cfg = entry.config();
    let held: fn(f64) -> bool = |t| (2.5 - 1e-9..5.5 - 1e-9).contains(&t);
    let trace = noisy_trace(entry, &cfg, &[(0.2, 0.6), (0.2, 0.9), (0.06, 1.1)], 8.0, 0.01, &mut Noise(1), Some(held));
    let fit = fit_joints(entry, &cfg, &trace).unwrap().unwrap();
    assert_eq!(fit.payload_steps.len(), 2, "{:?}", fit.payload_steps);
    for ((time, error), truth) in fit.payload_steps.iter().zip([2.5, 5.5]) {
        assert!((time - truth).abs() <= 3.0 * error.max(0.002) && *error <= 0.02, "{time} ± {error}, in truth {truth}");
    }
    for (quantity, of, expected) in [
        ("payload_factor_before", "payload", 0.0),
        ("payload_factor_after", "payload 1", 1.0),
        ("payload_factor_after", "payload 2", 0.0),
    ] {
        let (config, fitted, error) = value(&fit, quantity, of);
        let (factor, band) = (fitted / config, error / config);
        assert!((factor - expected).abs() < 3.0 * band && band < 0.02, "{quantity} {of}: × {factor:.4} ± {band:.4}");
    }
    let steps: Vec<&str> = fit.advice().iter().filter(|(key, _)| *key == "fit-payload-step").map(|_| "step").collect();
    assert_eq!(steps.len(), 2);
}
