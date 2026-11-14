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
#[test]
fn palletiser_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&PALLETISER);
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
    for entry in [&SCREW, &RACK, &LINEAR_MOTOR, &ROTARY_TABLE, &URDF_ARM, &SCARA, &PALLETISER] {
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

/// The time-optimal plan reads the same guide load (`Machine::rigid_guide_loads`): 2627.49 N at rest, and nothing more
/// from accelerating along the rail, which the motor takes.
#[test]
fn the_linear_motor_tells_the_time_optimal_plan_its_guide_load() {
    let m = (LINEAR_MOTOR.build)(&rigid(&LINEAR_MOTOR)).unwrap();
    let mut loads = Vec::new();
    m.rigid_guide_loads(&[0.2], &[15.0], &mut loads);
    assert_eq!(loads.len(), 1);
    near(loads[0][0], 2500.0 + 13.0 * 9.80665, 1e-12, "guide load at rest");
    assert_eq!(loads[0][1], 0.0);
}

/// A path whose motors may hardly turn crawls as near a singularity: the SCARA's motors at 1e-4 rad/s turn some rad per
/// metre of the tool, so 0.1 m along x would take some 1000 s and more, a pose per millisecond. The time-optimal plan
/// refuses a plan longer than 1000 s instead of handing on that many poses. (The plan's speed limit is the motors'
/// maximum speed since 14 November, no longer the rated speed; joints 1 and 2, which the line turns, take the shared one.)
#[test]
fn the_time_optimal_plan_refuses_a_crawl_that_would_take_unbounded_poses() {
    let mut cfg = SCARA.config();
    cfg.set_text("drive.max_speed_rad_s = 1.0e-4").unwrap();
    let sim = Sim::new(&SCARA, &cfg).unwrap();
    let start = sim.control.planned_pos().to_vec();
    let mut far = start.clone();
    far[0] -= 0.1;
    let limits = sim_components::path::PlanLimits::new(0.8, [f64::NEG_INFINITY, f64::INFINITY, 1.0]);
    let planned = sim_components::path::plan_time_optimal(
        sim.machine.as_ref(),
        &sim.control.motor_drives(sim.machine.as_ref()),
        &limits,
        &start,
        &[sim_components::path::Piece::Line(far)],
        1e-3,
        0.0,
        1e-4,
    );
    assert_eq!(planned.err().map(|e| e.key), Some("error-path-singular"));
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
        let extra = Some(sim_components::rigid::Inertial {
            mass: 3.0,
            com: [0.05, 0.01, -0.02],
            inertia: [[0.02, 0.001, 0.0], [0.001, 0.03, 0.002], [0.0, 0.002, 0.015]],
        });
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
        // The settling tolerance in rad is the joints' in joint space, the orientation's in tool space.
        for key in ["start.pose", "cycle", "ratings.settling_tolerance_rad"] {
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
    let mut cfg = tool_config(&SCARA, SCARA_TOOL_CONFIG);
    // The maximum speed of joints 1 and 2 (the shared drive) at the rated speed: above it the plan would follow their
    // curves (limits.rs). Joints 3 and 4, motors by name, keep the catalogue's maximum and turn slowly here.
    cfg.set_text("drive.max_speed_rad_s = 314.159").unwrap();
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
/// (settling 0.04 mm), the arm's in 3.858 s (settling 2.4 mm of its 5 mm and 3.2 mrad of its 5 mrad: the gears' twist
/// under the payload).
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

// ---------------------------------------------------------------------------------------------
// The arms' dynamics, tool space and palletiser (14 November).

/// Two links turning about vertical axes (no gravity torque), point masses: 2 kg at 1 m on link 1, 1.5 kg at 0.5 m on
/// link 2, whose joint is at the end of link 1.
const TWO_LINKS: &str = r#"<robot name="two"><link name="b"/>
    <link name="l1"><inertial><origin xyz="1 0 0"/><mass value="2"/><inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>
    <link name="l2"><inertial><origin xyz="0.5 0 0"/><mass value="1.5"/><inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/></inertial></link>
    <joint name="a" type="revolute"><parent link="b"/><child link="l1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
    <joint name="c" type="revolute"><parent link="l1"/><child link="l2"/><origin xyz="1 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint></robot>"#;

/// The Coriolis and centripetal torques of a two-link arm in joint space, which the feedforward left out before. By
/// hand (Craig, the planar two-link arm), h = m2 l1 lc2 sin q2 = 1.5 * 1 * 0.5 * sin(pi/2) = 0.75 kg m^2; at
/// q' = (1, 2) rad/s and no acceleration tau1 = -h (2 q1' q2' + q2'^2) = -0.75 * (4 + 4) = -6 N m and
/// tau2 = h q1'^2 = 0.75 N m; through gears of ratio 1 the motors give those. Without the speed `feedforward_torque` is
/// zero there.
#[test]
fn the_joint_space_feedforward_has_the_coriolis_torques_of_a_two_link_arm() {
    let path = std::env::temp_dir().join("joint-kit-test-two-links.urdf");
    std::fs::write(&path, TWO_LINKS).unwrap();
    let mut cfg = rigid(&URDF_ARM);
    for edit in [
        format!("arm.urdf_file = {:?}", path.to_str().unwrap()),
        "arm.gear_ratios = [1.0, 1.0]".into(),
        "effects.payload = false".into(),
    ] {
        cfg.set_text(&edit).unwrap();
    }
    let m = (URDF_ARM.build)(&cfg).unwrap();
    let q = [0.0, std::f64::consts::FRAC_PI_2];
    let mut torque = [0.0; 2];
    m.feedforward_torque_moving(&q, &[1.0, 2.0], &[0.0; 2], &mut torque);
    near(torque[0], -6.0, 1e-12, "tau1");
    near(torque[1], 0.75, 1e-12, "tau2");
    m.feedforward_torque(&q, &[0.0; 2], &mut torque);
    assert!(torque.iter().all(|t| t.abs() < 1e-12), "{torque:?}");
}

/// Each joint of the shipped arms has its own motor and drive: the six-axis arm's wrist (joints 4 to 6) a 1FK2104-5AF
/// on a 6SL3210-5HE10-8UF0 ([catalogue] 7.5 N m maximum, rotor 0.56 kg cm^2, 2.3 A rated), joints 1 to 3 the shared
/// 1FK2105-4AF (15 N m); the SCARA's quill and flange (joints 3 and 4) the 1FK2104-5AF too. The per-joint gear life
/// rating is the wrist's 40 N m.
#[test]
fn the_arms_have_their_own_motor_per_joint() {
    let sim = Sim::new(&URDF_ARM, &URDF_ARM.config()).unwrap();
    for (k, peak) in [(0, 15.0), (2, 15.0), (3, 7.5), (5, 7.5)] {
        assert_eq!(sim.control.drive_params_of(k).peak_torque, peak, "motor {}", k + 1);
    }
    let wrist = sim_components::catalog::motor_config(&URDF_ARM.config(), "5").unwrap();
    assert_eq!(wrist.f64("motor.rotor_inertia_kgm2").unwrap(), 5.6e-5);
    assert_eq!(wrist.f64("ratings.drive_rated_current_a").unwrap(), 2.3);
    assert_eq!(wrist.f64("strain_wave.rated_torque_nm").unwrap(), 40.0);
    let sim = Sim::new(&SCARA, &SCARA.config()).unwrap();
    assert_eq!(sim.control.drive_params_of(1).peak_torque, 15.0);
    assert_eq!(sim.control.drive_params_of(2).peak_torque, 7.5);
    assert_eq!(sim.control.drive_params_of(3).peak_torque, 7.5);
}

/// The palletiser's parallelogram keeps its flange level: over a grid of the motors' joints (each joint's range and the
/// turret and flange turned), the flange's z axis points straight down to rounding, and the URDF's wrist joint is
/// minus the forearm's angle against the base (joint 2 + joint 3 + joint 4 = 0, by the coupling's rows).
#[test]
fn the_palletiser_keeps_its_flange_level_over_its_range() {
    let cfg = PALLETISER.config();
    let chain = arm::chain(&cfg).unwrap();
    assert_eq!(chain.len(), 5);
    let mut cfg_joints = cfg.clone();
    cfg_joints.set_text("kinematics.tool_space = false").unwrap();
    let m = (PALLETISER.build)(&cfg_joints).unwrap();
    assert_eq!(m.axes().len(), 4);
    let (a2, a3) = (&m.axes()[1], &m.axes()[2]);
    let mut seen = 0;
    for i in 0..=6 {
        for j in 0..=6 {
            let p2 = a2.min + (a2.max - a2.min) * i as f64 / 6.0;
            let p3 = a3.min + (a3.max - a3.min) * j as f64 / 6.0;
            let full = [0.7, p2, p3 - p2, -p3, -1.1];
            let (rot, _) = chain.tip(&full);
            let down = [rot[0][2], rot[1][2], rot[2][2]];
            assert!(down[0].abs() < 1e-12 && down[1].abs() < 1e-12 && (down[2] + 1.0).abs() < 1e-12, "{down:?}");
            seen += 1;
        }
    }
    assert_eq!(seen, 49);
    // In tool space the flange's turn rz is the turret's and the flange's together, as nothing else turns it.
    let m = (PALLETISER.build)(&cfg).unwrap();
    let mut motors = [0.0; 4];
    m.inverse(&[1.4, 0.3, 1.0, 0.5], &mut motors);
    let (turret, flange) = (motors[0] / 160.0, motors[3] / 50.0);
    near(turret + flange, 0.5, 1e-9, "rz");
    near(turret, 0.3f64.atan2(1.4), 1e-9, "turret towards the flange");
}

/// The palletiser's carton swinging on the flange (effects.payload_swing), at rest with the arm still, the flange level
/// and the carton hanging 0.2 m below it. By hand: turned by 0.1 rad about the swing's first axis (across the carton's
/// 0.6 m side, the flange's y) its weight rises by m g l (1 - cos 0.1) = 20 * 9.80665 * 0.2 * 0.0049958 = 0.19597 J and
/// the bellows' spring holds 120 * 0.1^2 / 2 = 0.6 J; swinging at 1 rad/s about it, it has (m l^2 + I) / 2 =
/// (20 * 0.2^2 + 0.75) / 2 = 0.775 J (I = 20 (0.6^2 + 0.3^2) / 12 = 0.75 kg m^2 about the flange's y).
#[test]
fn the_palletisers_swinging_carton_has_the_hand_calculated_energies() {
    let mut cfg = PALLETISER.config();
    cfg.set_text("effects.payload_swing = true").unwrap();
    let m = (PALLETISER.build)(&cfg).unwrap();
    let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
    m.equilibrium(&[1.4, 0.0, 1.0, 0.0], &mut state, &mut torque);
    let at = state.len() - 4;
    let rest = m.energy(&state);
    let mut swung = state.clone();
    swung[at] = 0.1;
    let by_hand = 20.0 * 9.80665 * 0.2 * (1.0 - 0.1f64.cos()) + 0.5 * 120.0 * 0.01;
    near(m.energy(&swung) - rest, by_hand, 1e-9, "turned");
    assert!((by_hand - 0.79597).abs() < 1e-5);
    let mut swinging = state.clone();
    swinging[at + 2] = 1.0;
    near(m.energy(&swinging) - rest, 0.775, 1e-9, "swinging");
    // At rest it stays at rest, and the swing has its signal.
    let mut d = vec![0.0; state.len()];
    m.derivatives(&state, &torque, &mut d);
    assert!(d[at + 2].abs() < 1e-9 && d[at + 3].abs() < 1e-9, "{:?}", &d[at..]);
    assert!(m.signal_infos().iter().any(|s| s.id == "swing.1.angle"));
}

/// Warm-started from the last solution along a line, the generic inverse gives the joints a walk from the home gives,
/// to 1e-9 rad (the same branch, settled to the same point), on the six-axis arm's line of the tool across its
/// workspace, its orientation turning.
#[test]
fn the_warm_started_inverse_stays_on_the_home_branch() {
    let cfg = tool_config(&URDF_ARM, ARM_TOOL_CONFIG);
    let chain = arm::chain(&cfg).unwrap();
    let fk = |q: &[f64]| chain.tip(q);
    let home = cfg.f64s("kinematics.home_joints").unwrap();
    let pose = || tool::Pose::new(&fk, home.clone(), vec![0, 1, 2, 3, 4, 5], 0.86, 30.0).unwrap();
    let warm = pose();
    let (mut a, mut b) = ([0.0; 6], [0.0; 6]);
    for k in 0..=40 {
        let s = k as f64 / 40.0;
        let tool = [0.9, -0.3 + 0.6 * s, 0.7 - 0.1 * s, 0.2 * s, std::f64::consts::FRAC_PI_2 - 0.3 * s, -0.2 * s];
        assert!(warm.inverse(&fk, &tool, &mut a));
        assert!(pose().inverse(&fk, &tool, &mut b));
        assert!(a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-9), "at {s}: {a:?} against {b:?}");
    }
}

/// Generic damped least squares on the seven-joint URDF (config/arm7-tool.toml): a pose 0.2 m off the home's, turned
/// by 0.3 rad, is reached to 1e-9, and the redundant joint is spent on staying near the home: the joints' offset from
/// the home has no part along the Jacobian's null space (the direction n with J n = 0, here by the null space projector
/// of J's 6 x 7 rows), so no move that keeps the pose brings them nearer the home.
#[test]
fn the_seven_joint_arm_reaches_a_pose_nearest_its_home_posture() {
    let cfg = tool_config(&URDF_ARM, ARM7_TOOL_CONFIG);
    let m = (URDF_ARM.build)(&cfg).unwrap();
    assert_eq!((m.axes().len(), m.motors().len()), (6, 7));
    let chain = arm::chain(&cfg).unwrap();
    let home = cfg.f64s("kinematics.home_joints").unwrap();
    let fk = |q: &[f64]| chain.tip(q);
    let pose = tool::Pose::new(&fk, home.clone(), vec![0, 1, 2, 3, 4, 5], 0.86, 30.0).unwrap();
    let mut at_home = [0.0; 6];
    pose.forward(&fk, &home, &mut at_home);
    let target =
        [at_home[0] - 0.1, at_home[1] + 0.15, at_home[2] - 0.08, at_home[3] + 0.3, at_home[4], at_home[5] - 0.1];
    let mut q = [0.0; 7];
    assert!(pose.inverse(&fk, &target, &mut q));
    let mut back = [0.0; 6];
    pose.forward(&fk, &q, &mut back);
    assert!(back.iter().zip(&target).all(|(a, b)| (a - b).abs() < 1e-9), "{back:?} against {target:?}");
    // The null space: J (6 x 7, of the tool's axes, by central differences), n = (I - J^T (J J^T)^-1 J) e for the e of
    // the joint the null space is largest along.
    let jac: Vec<f64> = {
        let mut j = vec![0.0; 42];
        for c in 0..7 {
            let (mut up, mut down) = (q, q);
            up[c] += 1e-6;
            down[c] -= 1e-6;
            let (mut a, mut b) = ([0.0; 6], [0.0; 6]);
            pose.forward(&fk, &up, &mut a);
            pose.forward(&fk, &down, &mut b);
            for r in 0..6 {
                j[r * 7 + c] = (a[r] - b[r]) / 2e-6;
            }
        }
        j
    };
    let jjt: Vec<f64> = (0..36).map(|rc| (0..7).map(|k| jac[rc / 6 * 7 + k] * jac[rc % 6 * 7 + k]).sum()).collect();
    let null = |e: usize| -> Vec<f64> {
        let je: Vec<f64> = (0..6).map(|r| jac[r * 7 + e]).collect();
        let y = sim_components::chain::solve(&jjt, &je).unwrap();
        (0..7).map(|k| if k == e { 1.0 } else { 0.0 } - (0..6).map(|r| jac[r * 7 + k] * y[r]).sum::<f64>()).collect()
    };
    let n = (0..7)
        .map(null)
        .max_by(|a, b| a.iter().map(|v| v * v).sum::<f64>().total_cmp(&b.iter().map(|v| v * v).sum()))
        .unwrap();
    let size = n.iter().map(|v| v * v).sum::<f64>().sqrt();
    assert!(size > 0.1, "a null space of {size}");
    let along: f64 = (0..7).map(|k| n[k] / size * (q[k] - home[k])).sum();
    let off: f64 = (0..7).map(|k| (q[k] - home[k]).powi(2)).sum::<f64>().sqrt();
    assert!(off > 0.05 && along.abs() < 1e-6 * off.max(1.0), "{along} rad of {off} along the null space");
}

/// A URDF whose flange points down at zero joints (the five-axis example) works around pointing down: the orientation
/// is the turn from the flange's at zero, 0 there, not half a turn from the base frame as the rotation vector of the
/// orientation itself would have it (refused before). Its tool's five axes round-trip within 1e-9 over a grid of the
/// working range around the home.
#[test]
fn a_flange_pointing_down_at_zero_is_at_no_turn() {
    let cfg = tool_config(&URDF_ARM, ARM5_TOOL_CONFIG);
    let m = (URDF_ARM.build)(&cfg).unwrap();
    assert_eq!(m.axes().iter().map(|a| a.id).collect::<Vec<_>>(), ["x", "y", "z", "rx", "ry"]);
    let (mut motors, mut back, mut tried) = ([0.0; 5], [0.0; 5], 0);
    for k in 0..27 {
        let f = |i: usize| ((k / 3usize.pow(i as u32)) % 3) as f64 - 1.0;
        let pose = [0.9 + 0.1 * f(0), 0.2 * f(1), 0.7 + 0.1 * f(2), 0.2 * f(0) * f(1), 0.2 * f(2)];
        assert_eq!(m.unreachable(&pose, 0.0), None, "{pose:?}");
        m.inverse(&pose, &mut motors);
        m.forward(&motors, &mut back);
        assert!(pose.iter().zip(&back).all(|(a, b)| (a - b).abs() < 1e-9), "{pose:?}: {back:?}");
        tried += 1;
    }
    assert_eq!(tried, 27);
}

/// A line of the six-axis arm's tool from pointing down to a turn of 0.3 rad about the vertical after it: the two
/// rotation vectors, (0, pi/2, 0) and that of Rz(0.3) Ry(pi/2), are not parallel, and halfway along the planned
/// orientation is their slerp midpoint (`rigid`'s hand calculation of a slerp: one fixed axis, half the angle), not the
/// straight line's midpoint between the rotation vectors, which is off it by more than a milliradian. The place moves
/// straight.
#[test]
fn a_tool_line_turns_its_orientation_by_slerp() {
    use sim_components::rigid::{exp, log, matmul, slerp};
    let mut cfg = tool_config(&URDF_ARM, ARM_TOOL_CONFIG);
    cfg.set_text("effects.joint_elasticity = false").unwrap();
    let sim = Sim::new(&URDF_ARM, &cfg).unwrap();
    let from = sim.control.planned_pos().to_vec();
    let a = [from[3], from[4], from[5]];
    let b = log(&matmul(&exp([0.0, 0.0, 0.3]), &exp(a)));
    let text = format!("line, {}, {}, {}, {}, {}, {}", from[0] + 0.1, from[1] + 0.2, from[2], b[0], b[1], b[2]);
    let points = sim.plan_path(&sim_components::path::parse(&text).unwrap(), false).unwrap();
    let halfway =
        points.iter().min_by(|p, q| (p.1[1] - from[1] - 0.1).abs().total_cmp(&(q.1[1] - from[1] - 0.1).abs())).unwrap();
    let share = (halfway.1[1] - from[1]) / 0.2;
    assert!((share - 0.5).abs() < 0.02, "{share}");
    let want = slerp(a, b, share);
    for i in 0..3 {
        assert!((halfway.1[3 + i] - want[i]).abs() < 1e-6, "{:?} against the slerp {want:?}", halfway.1);
    }
    let linear: Vec<f64> = (0..3).map(|i| a[i] + share * (b[i] - a[i])).collect();
    let apart: f64 = (0..3).map(|i| (want[i] - linear[i]).powi(2)).sum::<f64>().sqrt();
    assert!(apart > 1e-3, "the slerp is {apart} rad from the straight line: the test tells them apart");
    assert!((halfway.1[0] - from[0] - 0.1 * share).abs() < 1e-6, "{:?}", halfway.1);
    assert!((halfway.1[2] - from[2]).abs() < 1e-6);
}

/// The time-optimal plan's points are bounded (`path::PLAN_POINTS_MOST`, ten million): where the motors allow a crawl
/// only (a pose near a singularity, where their speed limit through J^-1 goes to zero: a line onto the stretched SCARA
/// arm once asked for some 64 GB of points), the plan is refused, not allocated. Here the crawl by a motor of 0.001
/// rad/s at most (`drive.max_speed_rad_s`, which the plan keeps within): 0.1 m along y at about 0.6 m reach turns joint
/// 1 by some 0.17 rad, times its ratio 100, at 0.001 rad/s 1.7e4 s, 1.7e7 points of 1 ms. Beyond the tighter bound of
/// 1000 s (`path::PLAN_LONGEST`) too, which refuses it as a crawl near a singularity.
#[test]
fn a_time_optimal_plan_of_a_crawl_is_refused_beyond_its_bound_of_points() {
    let mut cfg = tool_config(&SCARA, SCARA_TOOL_CONFIG);
    cfg.set_text("drive.max_speed_rad_s = 0.001").unwrap();
    let sim = Sim::new(&SCARA, &cfg).unwrap();
    let start = sim.control.planned_pos().to_vec();
    let text = format!("line, {}, {}, {}, {}", start[0], start[1] + 0.1, start[2], start[3]);
    let limits = sim_components::path::PlanLimits::new(0.8, [f64::NEG_INFINITY, f64::INFINITY, 1.0]);
    let planned = sim_components::path::plan_time_optimal(
        sim.machine.as_ref(),
        &sim.control.motor_drives(sim.machine.as_ref()),
        &limits,
        &start,
        &sim_components::path::parse(&text).unwrap(),
        1e-3,
        0.0,
        1e-4,
    );
    assert_eq!(planned.err().map(|e| e.key), Some("error-path-singular"));
}

/// The swinging payload's two joints keep the loss-free machine's energy, rigid and elastic: the arm's dynamics with
/// them (Newton-Euler over the chain with the swing's bodies, taken to the motors' joints) are those of its energy.
/// Without gravity, as the standard suite runs the arms (held by constant torques after the pulse, an arm falls away
/// from its pose under gravity and trades far more energy than the pulse put in); the swing's spring holds the carton.
#[test]
fn the_swinging_payload_conserves_energy() {
    for elastic in [false, true] {
        let mut cfg = lossless_config(&PALLETISER);
        for edit in [
            "effects.payload_swing = true".to_string(),
            "effects.gravity = false".into(),
            "arm.payload_swing_damping_nm_s_rad = 0.0".into(),
            format!("effects.joint_elasticity = {elastic}"),
        ] {
            cfg.set_text(&edit).unwrap();
        }
        let m = (PALLETISER.build)(&cfg).unwrap();
        sim_core::testkit::energy_conservation(m.as_ref());
    }
}
