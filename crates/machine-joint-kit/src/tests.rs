use super::*;
use sim_components::analysis::{CycleLoad, Verdict};
use sim_components::{Command, Sim};

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

// ---------------------------------------------------------------------------------------------
// Tool space (`kinematics.tool_space`, `tool`).

fn tool_config(entry: &MachineEntry, text: &str) -> Config {
    sim_components::load_config(entry, text).unwrap()
}

/// The SCARA's arms from `urdf/scara.urdf`: 325 and 275 mm, the flange 0.2 m above the floor at joint 3 = 0.
const L1: f64 = 0.325;
const L2: f64 = 0.275;

#[test]
fn the_tool_configs_are_the_defaults_but_for_tool_space_start_cycle_and_settling() {
    for (entry, text) in [(&SCARA, SCARA_TOOL_CONFIG), (&URDF_ARM, ARM_TOOL_CONFIG)] {
        let (default, mut tool) = (entry.config(), tool_config(entry, text));
        assert!(tool.bool("kinematics.tool_space").unwrap());
        tool.set_text("kinematics.tool_space = false").unwrap();
        // The arm's settling tolerance is in m in tool space, in rad in joints.
        for key in ["start.pose", "cycle", "ratings.settling_tolerance_m"] {
            tool.set(key, default.get(key).unwrap().clone()).unwrap();
        }
        assert_eq!(tool.to_text().unwrap(), default.to_text().unwrap(), "{}", entry.id);
    }
}

/// At x = 0.325, y = 0.275 the cosine law gives cos(elbow) = (0.18125 - 0.325^2 - 0.275^2) / (2 0.325 0.275) = 0: the
/// right-handed elbow at +pi/2 with joint 1 at atan2(0.275, 0.325) - atan2(0.275, 0.325) = 0 (arm 1 along x, arm 2
/// along y); the left-handed at -pi/2 with joint 1 at 2 atan2(0.275, 0.325) = 1.404514. Joint 3 = (0.2 - z) / 1 m,
/// joint 4 = c - joint 1 - joint 2.
#[test]
fn the_scara_closed_form_gives_the_joints_of_a_pose_by_hand() {
    let chain = arm::chain(&SCARA.config()).unwrap();
    let tool = [L1, L2, 0.1, 0.3];
    let half = std::f64::consts::FRAC_PI_2;
    let left_j1 = 2.0 * L2.atan2(L1);
    assert!((left_j1 - 1.404514).abs() < 1e-6);
    for (elbow, by_hand) in [(1.0, [0.0, half, 0.1, 0.3 - half]), (-1.0, [left_j1, -half, 0.1, 0.3 - left_j1 + half])] {
        let scara = tool::Scara::new(&chain, elbow).unwrap();
        let mut q = [0.0; 4];
        assert!(scara.inverse(&tool, &mut q));
        for k in 0..4 {
            assert!((q[k] - by_hand[k]).abs() < 1e-12, "elbow {elbow}: joints {q:?}, by hand {by_hand:?}");
        }
        // And the URDF chain itself puts the flange there.
        let tip = chain.tip(&q).1;
        assert!((tip[0] - L1).abs() < 1e-12 && (tip[1] - L2).abs() < 1e-12 && (tip[2] - 0.1).abs() < 1e-12);
    }
}

/// Forward of inverse returns the pose within 1e-9 over the SCARA's tool range (the grid of the standard suite), and
/// the six-axis arm's at poses sampled around its home, orientations turned up to 0.3 rad; each arm holds still at rest
/// in tool space. The arm's wrist stays on the home's side (joint 5 > 0): the branch the inverse starts on.
#[test]
fn inverse_then_forward_returns_sampled_tool_poses_within_1e_9() {
    let scara = (SCARA.build)(&tool_config(&SCARA, SCARA_TOOL_CONFIG)).unwrap();
    sim_core::testkit::round_trip(scara.as_ref());
    sim_core::testkit::static_hold(scara.as_ref());

    let arm = (URDF_ARM.build)(&tool_config(&URDF_ARM, ARM_TOOL_CONFIG)).unwrap();
    let (mut motors, mut back, mut tried) = ([0.0; 6], [0.0; 6], 0);
    for k in 0..40 {
        let f = |i: usize| ((k * 7 + i * 13) % 11) as f64 / 10.0 - 0.5;
        let pose = [
            0.9 + 0.5 * f(0),
            0.8 * f(1),
            0.7 + 0.5 * f(2),
            0.6 * f(3),
            std::f64::consts::FRAC_PI_2 + 0.6 * f(4),
            0.6 * f(5),
        ];
        if arm.unreachable(&pose, 0.0).is_some() {
            continue;
        }
        tried += 1;
        arm.inverse(&pose, &mut motors);
        arm.forward(&motors, &mut back);
        for i in 0..6 {
            assert!((pose[i] - back[i]).abs() < 1e-9, "round trip at {pose:?} gave {back:?}");
        }
        assert!(motors[4] > 0.0, "wrist flipped at {pose:?}: motors {motors:?}");
    }
    assert!(tried >= 20, "only {tried} of 40 poses reachable");
    sim_core::testkit::static_hold(arm.as_ref());
}

/// A line of the tool, planned within the axis limits and as fast as the motors allow: every planned pose lies on it
/// within 1 um (and the turn c along it as y says), and the joints the inverse gives there put the flange on it too.
/// The six-axis arm's line holds its orientation within 1e-6 rad, read back from the joints through the URDF chain.
#[test]
fn every_planned_pose_of_a_straight_tool_line_lies_within_1_um_of_it() {
    let sim = Sim::new(&SCARA, &tool_config(&SCARA, SCARA_TOOL_CONFIG)).unwrap();
    let line = sim_components::path::parse("line, 0.4, 0.25, 0.15, 1.5").unwrap();
    let (mut motors, mut back) = ([0.0; 4], [0.0; 4]);
    for time_optimal in [false, true] {
        let points = sim.plan_path(&line, time_optimal).unwrap();
        assert!(points.len() > 100);
        for (_, p) in &points {
            // From the start (0.4, -0.25, 0.15, 0): x and z fixed, c = 1.5 (y + 0.25) / 0.5.
            let off = [p[0] - 0.4, p[2] - 0.15, p[3] - 3.0 * (p[1] + 0.25)];
            assert!(off.iter().all(|o| o.abs() < 1e-6), "{p:?} is off the line by {off:?}");
            sim.machine.inverse(p, &mut motors);
            sim.machine.forward(&motors, &mut back);
            assert!(p.iter().zip(&back).all(|(a, b)| (a - b).abs() < 1e-6), "{p:?}: joints put the flange at {back:?}");
        }
    }

    // Rigid: the elastic arm's setpoint at rest is where its motors stand, 2.3 mm above the sagging flange.
    let mut cfg = tool_config(&URDF_ARM, ARM_TOOL_CONFIG);
    cfg.set_text("effects.joint_elasticity = false").unwrap();
    let sim = Sim::new(&URDF_ARM, &cfg).unwrap();
    let down = std::f64::consts::FRAC_PI_2;
    let line = sim_components::path::parse(&format!("line, 0.9, 0.3, 0.7, 0.0, {down}, 0.0")).unwrap();
    let (mut motors, mut back) = ([0.0; 6], [0.0; 6]);
    let points = sim.plan_path(&line, false).unwrap();
    assert!(points.len() > 100);
    for (_, p) in &points {
        sim.machine.inverse(p, &mut motors);
        sim.machine.forward(&motors, &mut back);
        // From (0.9, -0.3, 0.7) along y.
        assert!((back[0] - 0.9).abs() < 1e-6 && (back[2] - 0.7).abs() < 1e-6, "{back:?} off the line");
        let turn = [back[3], back[4] - down, back[5]];
        assert!(turn.iter().map(|t| t * t).sum::<f64>().sqrt() < 1e-6, "orientation {back:?}");
    }
}

/// At x = 0.325, y = 0.275 (joint 1 at 0, joint 2 at pi/2) the arm's Jacobian in the plane is
/// [[-L1 sin 0 - L2 sin pi/2, -L2 sin pi/2], [L1 cos 0 + L2 cos pi/2, L2 cos pi/2]] = [[-0.275, -0.275], [0.325, 0]]:
/// along y at 0.5 m/s, joint 1 turns 0.5 / 0.325 = 1.538462 rad/s and joint 2 the opposite (the first row is 0), and
/// joint 4 turns back what they turn together (0), so that c stays. The motors: those times the ratios 100, 80.
/// Its condition number: from the eigenvalues of J^T J = [[0.18125, 0.075625], [0.075625, 0.075625]], 2.4691.
#[test]
fn the_joint_speeds_along_a_tool_line_are_the_inverse_jacobian_times_the_tool_speed() {
    let machine = (SCARA.build)(&tool_config(&SCARA, SCARA_TOOL_CONFIG)).unwrap();
    let pose = [L1, L2, 0.1, 0.3];
    let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
    machine.equilibrium(&pose, &mut state, &mut torque);
    machine.set_tool_velocity(&mut state, &[0.0, 0.5, 0.0, 0.0]);
    let (mut motors, mut speeds) = ([0.0; 4], [0.0; 4]);
    machine.motor_state(&state, &mut motors, &mut speeds);
    let by_hand = 0.5 / L1;
    assert!((by_hand - 1.538462).abs() < 1e-6);
    for (k, want) in [100.0 * by_hand, -80.0 * by_hand, 0.0, 0.0].into_iter().enumerate() {
        assert!((speeds[k] - want).abs() < 1e-4, "motor {k}: {} rad/s, by hand {want}", speeds[k]);
    }
    let (tr, det): (f64, f64) = (0.18125 + 0.075625, 0.18125 * 0.075625 - 0.075625 * 0.075625);
    let root = (tr * tr / 4.0 - det).sqrt();
    let cond = ((tr / 2.0 + root) / (tr / 2.0 - root)).sqrt();
    assert!((cond - 2.4691).abs() < 1e-4);
    assert!((machine.condition(&pose).unwrap() - cond).abs() < 1e-9);
}

/// The stretched SCARA arm is a singularity: at x = 0.598 (elbow acos((0.598^2 - L1^2 - L2^2) / (2 L1 L2)) =
/// 0.1639 rad) the condition number is above the config's 25, and a move there is refused with that reason; at 0.59
/// (elbow 0.3669 rad) it goes. A line of the tool that ends at the stretched arm (x = 0.6) is refused as running out of
/// what the machine can go to, as on the deltas; beyond the reach (0.61 m) the reason is the reach.
#[test]
fn a_tool_path_into_the_stretched_scara_arm_is_refused_with_its_reason() {
    let mut sim = Sim::new(&SCARA, &tool_config(&SCARA, SCARA_TOOL_CONFIG)).unwrap();
    let elbow = |x: f64| ((x * x - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)).acos();
    assert!((elbow(0.598) - 0.1639).abs() < 1e-4 && (elbow(0.59) - 0.3669).abs() < 1e-4);
    assert!(sim.machine.condition(&[0.598, 0.0, 0.1, 0.0]).unwrap() > 25.0);
    assert!(sim.machine.condition(&[0.6, 0.0, 0.1, 0.0]).unwrap() > 1e6);
    let refused = |sim: &mut Sim, x: f64| sim.command(Command::MoveTo(vec![x, 0.0, 0.1, 0.0])).err().map(|e| e.key);
    assert_eq!(refused(&mut sim, 0.598), Some("error-arm-singularity"));
    assert_eq!(sim.machine.unreachable(&[0.61, 0.0, 0.1, 0.0], 0.0), Some("error-arm-out-of-reach"));
    let line = sim_components::path::parse("line, 0.6, 0.0, 0.15, 0.0").unwrap();
    for time_optimal in [false, true] {
        assert_eq!(sim.plan_path(&line, time_optimal).err().map(|e| e.key), Some("error-path-range"));
    }
    assert_eq!(refused(&mut sim, 0.59), None);
}

/// The time-optimal plan of a line of the SCARA's tool takes the motors' limits through the inverse kinematics: no
/// motor turns faster than its rated 314.159 rad/s anywhere along it (speeds by differences of the joints the planned
/// poses ask for, over the planning cycle), and the fastest gets near it, faster than in the plan within the axis
/// limits (1 m/s along y), which so takes longer.
#[test]
fn the_time_optimal_plan_keeps_the_scara_motors_within_their_rated_speed_along_a_tool_line() {
    let cfg = tool_config(&SCARA, SCARA_TOOL_CONFIG);
    let sim = Sim::new(&SCARA, &cfg).unwrap();
    let rated = cfg.positive("drive.rated_speed_rad_s").unwrap();
    let line = sim_components::path::parse("line, 0.4, 0.25, 0.15, 1.5").unwrap();
    let fastest = |time_optimal: bool| {
        let points = sim.plan_path(&line, time_optimal).unwrap();
        let motors: Vec<Vec<f64>> = points
            .iter()
            .map(|(_, p)| {
                let mut m = vec![0.0; 4];
                sim.machine.inverse(p, &mut m);
                m
            })
            .collect();
        let most = motors
            .windows(2)
            .zip(points.windows(2))
            .flat_map(|(m, t)| (0..4).map(move |k| (m[1][k] - m[0][k]).abs() / (t[1].0 - t[0].0)))
            .fold(0.0, f64::max);
        (most, points.last().unwrap().0)
    };
    let ((optimal, optimal_time), (limited, limited_time)) = (fastest(true), fastest(false));
    assert!(optimal <= rated * 1.001, "{optimal} rad/s above the rated {rated}");
    assert!(optimal > 0.9 * rated && optimal > limited, "{optimal} rad/s, within the axis limits {limited}");
    assert!(optimal_time < limited_time, "{optimal_time} s against {limited_time} s");
}

/// The tool-space configs' cycles of straight lines pass every verdict, as the defaults' do: the SCARA's in 2.542 s
/// (settling 0.04 mm), the arm's in 3.774 s (settling 4.0 mm of its 5: the gears' twist under the payload).
#[test]
fn the_tool_space_cycles_pass_their_verdicts() {
    for (entry, text) in [(&SCARA, SCARA_TOOL_CONFIG), (&URDF_ARM, ARM_TOOL_CONFIG)] {
        let cfg = tool_config(entry, text);
        let (load, verdicts) = judged(entry, &cfg);
        assert!(!load.tripped, "{}", entry.id);
        for v in verdicts {
            assert!(v.ok, "{}: {} of motor {:?} is {} against {}", entry.id, v.check, v.motor, v.value, v.limit);
        }
    }
}

/// One steel tube of 60 x 4 mm, 0.8 m long and 4.5 kg, on a joint about y, lying along x: as a link that bends
/// (effects.link_bending) it is a cantilever from its joint, with nothing at its tip.
fn one_bending_link() -> Box<dyn Machine> {
    let path = std::env::temp_dir().join("joint-kit-test-one-link.urdf");
    std::fs::write(
        &path,
        r#"<?xml version="1.0"?>
<robot name="one-link">
  <link name="base_link"/>
  <link name="tube">
    <inertial>
      <origin xyz="0.4 0 0" rpy="0 0 0"/>
      <mass value="4.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.24" iyz="0" izz="0.24"/>
    </inertial>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="tube"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-3.0" upper="3.0" effort="0" velocity="0"/>
  </joint>
  <link name="tool0"/>
  <joint name="flange" type="fixed">
    <parent link="tube"/>
    <child link="tool0"/>
    <origin xyz="0.8 0 0" rpy="0 0 0"/>
  </joint>
</robot>"#,
    )
    .unwrap();
    let mut cfg = URDF_ARM.config();
    for edit in [
        format!("arm.urdf_file = {:?}", path.to_str().unwrap()),
        "arm.gear_ratios = [120.0]".into(),
        "effects.payload = false".into(),
        "effects.link_bending = true".into(),
        "link_bending.outer_diameter_m = [0.06]".into(),
        "link_bending.wall_m = [0.004]".into(),
        "link_bending.youngs_modulus_pa = 2.1e11".into(),
        "link_bending.damping_ratio = 0.0".into(),
    ] {
        cfg.set_text(&edit).unwrap();
    }
    (URDF_ARM.build)(&cfg).unwrap()
}

/// The tube's bending mode is the Euler-Bernoulli cantilever's: E I = 2.1e11 x pi / 64 (0.06^4 - 0.052^4) =
/// 58225.5 N m^2, f = 1.875104^2 / (2 pi) sqrt(E I / (m L^3)) = 0.559597 sqrt(58225.5 / (4.5 x 0.512)) = 88.958 Hz.
#[test]
fn a_link_bends_at_the_euler_bernoulli_frequency_of_a_cantilever() {
    let m = one_bending_link();
    let modes = m.modes(&[0.0]);
    let euler_bernoulli =
        1.875_104_068_711_961f64.powi(2) / std::f64::consts::TAU * (58_225.524f64 / (4.5 * 0.512)).sqrt();
    assert!((euler_bernoulli - 88.958).abs() < 1e-3);
    assert!(modes.iter().any(|mode| (mode.hz / euler_bernoulli - 1.0).abs() < 1e-6), "{modes:?}");
}

/// Under its own weight, lying level, the tube sags in its mode by 0.3915 m g / (3.0906 E I / L^3) =
/// 0.391496 x 44.130 / (3.0906 x 58225.5 / 0.512) = 49.16 um: within 1.5 % of the exact q L^4 / (8 E I) =
/// 44.130 x 0.512 / (8 x 58225.5) = 48.51 um of a beam loaded along its length. Downwards: the signal is positive.
#[test]
fn a_level_link_sags_under_its_weight_as_a_cantilever_does() {
    let m = one_bending_link();
    let (mut state, mut torque) = (Vec::new(), vec![0.0]);
    m.equilibrium(&[0.0], &mut state, &mut torque);
    let mut values = Vec::new();
    m.signals(&state, &torque, &mut values);
    let bend = values[m.signal_infos().iter().position(|s| s.id == "link.1.bend").unwrap()];
    assert!((bend - 49.156e-6).abs() < 0.01e-6, "{bend}");
    assert!((bend / 48.506e-6 - 1.0).abs() < 0.015);
}
