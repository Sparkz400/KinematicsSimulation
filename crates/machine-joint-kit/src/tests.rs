use super::*;
use sim_components::analysis::{CycleLoad, Verdict};

#[test]
fn screw_axis_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&SCREW);
}
#[test]
fn rack_axis_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&RACK);
}
#[test]
fn linear_motor_axis_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&LINEAR_MOTOR);
}
#[test]
fn rotary_table_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ROTARY_TABLE);
}
#[test]
fn urdf_arm_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&URDF_ARM);
}
#[test]
fn scara_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&SCARA);
}

/// Runs a machine's default cycle and judges it.
fn judged(entry: &MachineEntry, cfg: &Config) -> (CycleLoad, Vec<Verdict>) {
    let mut sim = sim_components::Sim::new(entry, cfg).unwrap();
    let load = sim_components::analysis::analyse_cycle(&mut sim).unwrap();
    let verdicts = sim_components::analysis::verdicts(cfg, &load).unwrap();
    (load, verdicts)
}

/// Each machine's default config passes every verdict of its own cycle: the defaults are a sized example.
#[test]
fn every_default_cycle_passes_its_verdicts() {
    for entry in [&SCREW, &RACK, &LINEAR_MOTOR, &ROTARY_TABLE, &URDF_ARM, &SCARA] {
        let (load, verdicts) = judged(entry, &entry.config());
        assert!(!load.tripped, "{}", entry.id);
        for v in verdicts {
            assert!(v.ok, "{}: {} of motor {:?} is {} against {}", entry.id, v.check, v.motor, v.value, v.limit);
        }
    }
}

/// The rigid, loss-free model.
fn rigid(entry: &MachineEntry) -> Config {
    let mut cfg = lossless_config(entry);
    cfg.set_text("effects.joint_elasticity = false").unwrap();
    cfg
}

fn value(verdicts: &[Verdict], check: &str) -> f64 {
    verdicts.iter().find(|v| v.check == check).unwrap_or_else(|| panic!("no verdict {check}")).value
}

fn near(a: f64, b: f64, share: f64, what: &str) {
    assert!((a / b - 1.0).abs() < share, "{what}: {a}, by hand {b}");
}

/// Runs two passes of the cycle and returns the highest (base + payload while attached) * |acceleration| of axis 0, the
/// acceleration taken from the axis's speed step by step, and the highest |signal|.
fn inertial_peak(entry: &MachineEntry, cfg: &Config, base: f64, payload: f64, signal: &str) -> (f64, f64) {
    use sim_components::{Command, Sim};
    let mut sim = Sim::new(entry, cfg).unwrap();
    let axis = sim.machine.axes()[0].id;
    let (speed, attached, at) = (
        sim.signal_index(&format!("axis.{axis}.velocity")).unwrap(),
        sim.signal_index("payload").unwrap(),
        sim.signal_index(signal).unwrap(),
    );
    let dt = cfg.f64("sim.dt_s").unwrap();
    sim.command(Command::StartCycle).unwrap();
    let (mut sample, mut last, mut peaks): (Vec<f64>, Option<f64>, (f64, f64)) = (Vec::new(), None, (0.0, 0.0));
    let duration = judged(entry, cfg).0.duration;
    sim.run(2.0 * duration, |s| {
        sample.clear();
        s.sample(&mut sample);
        if let Some(before) = last {
            let mass = base + payload * sample[attached];
            peaks.0 = peaks.0.max(mass * ((sample[speed] - before) / dt).abs());
        }
        peaks.1 = peaks.1.max(sample[at].abs());
        last = Some(sample[speed]);
    });
    peaks
}

/// By hand: 0.8 m/s on a 20 mm lead is 2400 r/min (d n 25 * 2400 = 60000), within 3 % for the loops' overshoot. On the
/// rigid, loss-free, horizontal axis the nut pushes the carriage (15 kg, 25 with the payload) by m a: the force the
/// checks judge is the mass times the acceleration the run reached, to 1 %; that acceleration is the profile's 8 m/s^2
/// and what the loops add, below 15 % more.
#[test]
fn the_screw_axis_reaches_the_hand_calculated_speed_and_force() {
    let cfg = rigid(&SCREW);
    let (load, v) = judged(&SCREW, &cfg);
    assert!(!load.tripped);
    near(value(&v, "screw-speed"), 2400.0, 0.03, "screw speed");
    near(value(&v, "screw-dn"), 60000.0, 0.03, "d n");
    let (inertial, force) = inertial_peak(&SCREW, &cfg, 15.0, 10.0, "joint.1.force");
    near(value(&v, "screw-buckling"), inertial, 0.01, "axial force");
    near(force, inertial, 0.01, "axial force");
    assert!(inertial > 200.0 && inertial < 230.0, "{inertial} N for 25 kg at 8 m/s^2");
    assert_eq!(value(&v, "screw-static"), value(&v, "screw-buckling"));
    assert!(v.iter().all(|v| v.ok), "{v:?}");
    // A screw 1.6 m long whirls at (1 / 1.6)^2 of the speed: 0.8 * 4000 / 2.56 = 1250 r/min allowed, below 2400; and a
    // life asked for beyond what it gives fails.
    let mut long = cfg.clone();
    long.set_text("screw.length_m = 1.6").unwrap();
    long.set_text("ratings.required_life_h = 1.0e12").unwrap();
    let (_, v) = judged(&SCREW, &long);
    let failed = |id: &str| v.iter().any(|v| v.check == id && !v.ok);
    assert!(failed("screw-speed") && failed("screw-life") && !failed("screw-buckling"), "{v:?}");
}

/// By hand: on the rack m a of 60 kg (80 with the payload), at the gearbox output that force times the pinion's
/// 31.831 mm; 0.95 m/s is 0.95 / 0.031831 = 29.85 rad/s of the pinion, 2850 r/min at the motor behind ratio 10.
#[test]
fn the_rack_axis_reaches_the_hand_calculated_force_torque_and_speed() {
    let cfg = rigid(&RACK);
    let (_, v) = judged(&RACK, &cfg);
    let (inertial, _) = inertial_peak(&RACK, &cfg, 60.0, 20.0, "joint.1.force");
    near(value(&v, "rack-feed-force"), inertial, 0.01, "feed force");
    near(value(&v, "gearbox-peak"), inertial * 0.031831, 0.01, "gearbox torque");
    assert!(inertial > 400.0 && inertial < 460.0, "{inertial} N for 80 kg at 5 m/s^2");
    near(value(&v, "gearbox-input-speed"), 2850.0, 0.03, "motor speed");
}

/// By hand: the guides carry the attraction and the weight of carriage, forcer and payload, 2500 + (5 + 3 + 5) * 9.80665 =
/// 2627.49 N, at any speed; the motor pushes the carriage, its payload and the forcer's own 3 kg (the rotor's inertia)
/// by m a, 13 kg in all.
#[test]
fn the_linear_motor_loads_its_guides_with_its_attraction() {
    let cfg = rigid(&LINEAR_MOTOR);
    let (load, v) = judged(&LINEAR_MOTOR, &cfg);
    near(value(&v, "guide-load"), 2627.486, 1e-6, "guide load");
    let (inertial, motor) = inertial_peak(&LINEAR_MOTOR, &cfg, 8.0, 5.0, "motor.1.torque");
    near(motor, inertial, 0.01, "motor force");
    assert!(!load.tripped);
    assert!(inertial > 260.0 && inertial < 300.0, "{inertial} N for 13 kg at 20 m/s^2");
}

/// By hand: the table and fixture (3 kg m^2) and the work (1.5) take J alpha at the gear's output.
#[test]
fn the_rotary_table_takes_the_hand_calculated_torque() {
    let cfg = rigid(&ROTARY_TABLE);
    let (_, v) = judged(&ROTARY_TABLE, &cfg);
    let (inertial, _) = inertial_peak(&ROTARY_TABLE, &cfg, 3.0, 1.5, "gearbox.1.output_torque");
    near(value(&v, "gearbox-peak"), inertial, 0.01, "gear torque");
    assert!(inertial > 67.5 && inertial < 78.0, "{inertial} N m for 4.5 kg m^2 at 15 rad/s^2");
    assert!(v.iter().any(|v| v.check == "strain-wave-life"));
}

/// Joint 2 at 90 degrees: the arm reaches out along x. Moments of the weights about joint 2, from urdf/six-axis.urdf:
/// link2 12 kg at 0.3 m, link3 7 kg at 0.68 m, links 4 to 6 (4 + 2 + 1.5 kg) and the 6 kg payload at 0.72 m: 9.80665 *
/// (3.6 + 4.76 + 9.72) = 177.304 N m, held against by -177.304 / 120 = -1.47753 N m at motor 2. About joint 3 (0.6 m
/// out): 9.80665 * (7 * 0.08 + 13.5 * 0.12) = 21.3785 N m, -0.178154 N m at motor 3. Joint 1 turns about the vertical:
/// nothing.
#[test]
fn the_six_axis_arm_holds_the_hand_calculated_gravity_torques() {
    let m = (URDF_ARM.build)(&rigid(&URDF_ARM)).unwrap();
    let mut torque = [0.0; 6];
    m.feedforward_torque(&[0.0, std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0], &[0.0; 6], &mut torque);
    assert!(torque[0].abs() < 1e-9, "{torque:?}");
    near(torque[1], -1.47753, 1e-5, "motor 2");
    near(torque[2], -0.178154, 1e-5, "motor 3");
}

/// The SCARA's quill, flange and payload (1.5 + 1 + 3 kg) hang on the screw: 5.5 * 9.80665 = 53.937 N, its motor
/// holds -53.937 * 0.02 / 2 pi = -0.171686 N m (the quill's coordinate counts down). The arms turn about the vertical.
#[test]
fn the_scara_quill_hangs_on_its_screw() {
    let m = (SCARA.build)(&rigid(&SCARA)).unwrap();
    let mut torque = [0.0; 4];
    m.feedforward_torque(&[0.3, -0.5, 0.1, 1.0], &[0.0; 4], &mut torque);
    assert!(torque[0].abs() < 1e-9 && torque[1].abs() < 1e-9 && torque[3].abs() < 1e-9, "{torque:?}");
    near(torque[2], -0.171686, 1e-5, "motor 3");
}

/// A URDF that cannot be read is a config error, not a panic; a file path is read from disk.
#[test]
fn the_arm_reads_its_urdf_from_a_file() {
    let mut cfg = URDF_ARM.config();
    cfg.set_text("arm.urdf_file = \"no/such/file.urdf\"").unwrap();
    assert!((URDF_ARM.build)(&cfg).is_err());
    let path = std::env::temp_dir().join("joint-kit-test-scara.urdf");
    std::fs::write(&path, include_str!("../urdf/scara.urdf")).unwrap();
    let mut cfg = SCARA.config();
    cfg.set_text(&format!("arm.urdf_file = {:?}", path.to_str().unwrap())).unwrap();
    assert_eq!((SCARA.build)(&cfg).unwrap().axes().len(), 4);
    cfg.set_text("arm.gear_ratios = [100.0, 80.0, 1.0]").unwrap();
    assert!((SCARA.build)(&cfg).is_err(), "one ratio per joint");
}

/// The acceleration of a machine at rest at its start pose under one motor torque, before and after its gearboxes are
/// told a temperature rise of zero: the efficiency of screw, rack or worm must stay in.
fn accelerations_around_feedback(entry: &MachineEntry, cfg: &mut Config) -> (f64, f64) {
    cfg.set_text("effects.thermal_feedback = true").unwrap();
    cfg.set_text("effects.part_heating = true").unwrap();
    cfg.set_text("effects.joint_elasticity = false").unwrap();
    let mut m = (entry.build)(cfg).unwrap();
    let pose = cfg.f64s("start.pose").unwrap();
    let (mut state, mut torque) = (Vec::new(), vec![0.0; m.motors().len()]);
    m.equilibrium(&pose, &mut state, &mut torque);
    // Moving, so that the gearbox passes power and loses its efficiency on it.
    let mut speed = vec![0.0; pose.len()];
    speed[0] = 0.2;
    m.set_tool_velocity(&mut state, &speed);
    torque[0] += 3.0;
    let mut d = vec![0.0; state.len()];
    m.derivatives(&state, &torque, &mut d);
    let before = d[pose.len()];
    m.set_part_temperatures(&vec![0.0; m.thermal_parts().len()]);
    m.derivatives(&state, &torque, &mut d);
    (before, d[pose.len()])
}

#[test]
fn the_temperature_feedback_keeps_the_efficiency_of_screw_rack_and_worm() {
    for (entry, worm) in [(&SCREW, false), (&RACK, false), (&ROTARY_TABLE, true)] {
        let mut cfg = entry.config();
        if worm {
            cfg.set_text("table.gear = \"worm\"").unwrap();
        }
        let (before, after) = accelerations_around_feedback(entry, &mut cfg);
        assert!((after / before - 1.0).abs() < 1e-9, "{}: {before} before, {after} after", entry.id);
    }
}

/// The screw's and the strain wave gear's lives are L10s: at 99 % reliability they shrink by a1 = 0.25 (ISO 281's
/// Weibull form, as for every bearing life here).
#[test]
fn the_life_checks_take_the_reliability() {
    let mut cfg = SCREW.config();
    let (load, at_90) = judged(&SCREW, &cfg);
    cfg.set_text("ratings.bearing_reliability_percent = 99.0").unwrap();
    let at_99 = sim_components::analysis::verdicts(&cfg, &load).unwrap();
    let a1 = sim_components::analysis::reliability_factor(&cfg).unwrap();
    assert!((a1 - 0.25).abs() < 0.01);
    near(value(&at_99, "screw-life"), a1 * value(&at_90, "screw-life"), 1e-12, "screw life");
}

/// With the rack's 97 %, the gearbox output and the teeth carry more than the force the load gets while the pinion
/// drives: the peak gearbox torque is the peak force at the load times the pinion radius over 0.97, the peak feed force
/// that force over 0.97 (the peak comes while accelerating, where the pinion drives).
#[test]
fn the_gearbox_and_the_rack_carry_the_rack_losses() {
    let mut cfg = RACK.config();
    cfg.set_text("effects.joint_elasticity = false").unwrap();
    let (_, force) = inertial_peak(&RACK, &cfg, 0.0, 0.0, "joint.1.force");
    let (_, torque) = inertial_peak(&RACK, &cfg, 0.0, 0.0, "gearbox.1.output_torque");
    let (_, teeth) = inertial_peak(&RACK, &cfg, 0.0, 0.0, "rack.1.feed_force");
    near(torque, force * 0.031831 / 0.97, 1e-3, "gearbox torque");
    near(teeth, force / 0.97, 1e-3, "feed force");
}

/// A screw joint of an arm is coupled to its motor: another ratio is refused, and so is a gearbox life the analysis
/// would take through the one `gearbox.ratio` for joints of other ratios.
#[test]
fn an_arm_refuses_what_it_cannot_size() {
    let mut cfg = SCARA.config();
    cfg.set_text("arm.gear_ratios = [100.0, 80.0, 2.0, 50.0]").unwrap();
    assert!((SCARA.build)(&cfg).is_err());
    let mut cfg = Config::parse(&URDF_ARM.default_config.replace(
        "[ratings]
",
        "[ratings]
gearbox_reference_speed_rpm = 400.0
",
    ))
    .unwrap();
    assert!((URDF_ARM.build)(&cfg).is_err());
    cfg.set_text("arm.gear_ratios = [120.0, 120.0, 120.0, 120.0, 120.0, 120.0]").unwrap();
    assert!((URDF_ARM.build)(&cfg).is_ok());
}

/// The mass matrix from the bodies' Jacobians is Newton-Euler's, one unit acceleration at a time, to rounding: on
/// both URDF examples, with the payload, at poses away from zero.
#[test]
fn the_jacobian_mass_matrix_is_newton_eulers() {
    for entry in [&URDF_ARM, &SCARA] {
        let chain = arm::chain(&entry.config()).unwrap();
        let n = chain.len();
        let q: Vec<f64> = (0..n).map(|k| 0.3 - 0.17 * k as f64).collect();
        let extra = Some((3.0, [0.05, 0.01, -0.02]));
        let mut fast = vec![0.0; n * n];
        chain.mass_matrix(&q, extra, &mut fast);
        let (mut unit, mut tau) = (vec![0.0; n], vec![0.0; n]);
        for j in 0..n {
            unit.iter_mut().enumerate().for_each(|(k, u)| *u = if k == j { 1.0 } else { 0.0 });
            chain.inverse_dynamics(&q, &vec![0.0; n], &unit, 0.0, extra, &mut tau);
            for i in 0..n {
                assert!(
                    (fast[i * n + j] - tau[i]).abs() < 1e-12 * tau[i].abs().max(1.0),
                    "{} M[{i}][{j}]: {} against {}",
                    entry.id,
                    fast[i * n + j],
                    tau[i]
                );
            }
        }
    }
}
