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
#[test]
fn arm_on_track_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ARM_ON_TRACK);
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
    for entry in [&SCREW, &RACK, &LINEAR_MOTOR, &ROTARY_TABLE, &URDF_ARM, &SCARA, &PALLETISER, &ARM_ON_TRACK] {
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

/// effects.slide_presliding: the screw axis inclined by 5°, its drive disabled, holding brake and motor friction off, so
/// that only the slide's friction holds the carriage. By hand: 25 kg (with the payload) pull along it with
/// 25 * 9.80665 * sin 5° = 21.37 N; the guides' 30 N and the nut's preload drag (THK: K = 0.05 / sqrt(0.02 / (pi 0.025))
/// = 0.0991, 0.0991 * 600 N * 0.02 m / 2 pi = 0.1893 N m, 59.47 N at the nut) slide at g = 89.47 N, break away at
/// 45 + 59.47 = 104.47 N. With the presliding state (sigma 1e7 N/m) the carriage takes up the pull within
/// -g / sigma ln(1 - F / g) = -1.0447e-5 ln(1 - 21.37 / 104.47) = 2.40 um, swings through it on the screw with the free
/// rotor, and each swing moves the state on a little (Dahl's law closes no small loop): 12.1 um in the first half second,
/// within 20 um, then it stops, less than 1 um in the second. With the smoothed Coulomb step it creeps at
/// 0.002 m/s * atanh(21.37 / 89.47) = 0.487 mm/s, 0.24 mm in the second half second.
#[test]
fn a_presliding_slide_holds_its_load_without_creeping() {
    let travel = |presliding: bool| {
        let mut cfg = SCREW.config();
        for set in [
            "slide.incline_deg = 5.0",
            "effects.holding_brake = false",
            "effects.motor_friction = false",
            "effects.base_compliance = false",
        ] {
            cfg.set_text(set).unwrap();
        }
        cfg.set_text(&format!("effects.slide_presliding = {presliding}")).unwrap();
        let mut sim = Sim::new(&SCREW, &cfg).unwrap();
        sim.command(Command::Disable).unwrap();
        let at = sim.signal_index("axis.x.position").unwrap();
        let mut sample = Vec::new();
        let mut position = |sim: &mut Sim, seconds: f64| {
            sim.run(seconds, |_| {});
            sim.sample(&mut sample);
            sample[at]
        };
        let start = position(&mut sim, 0.0);
        let settled = position(&mut sim, 0.5);
        let end = position(&mut sim, 0.5);
        (settled - start, end - settled)
    };
    let (taken_up, crept) = travel(true);
    assert!(taken_up < -2.40e-6 && taken_up > -20e-6, "{taken_up}");
    assert!(crept.abs() < 1e-6, "{crept}");
    let (_, crept) = travel(false);
    assert!((crept + 0.2435e-3).abs() < 0.03e-3, "{crept}");
}

/// The screw axis's drive train for the planner (`Machine::trains`) is the elastic model's own mode: the coupling and
/// the screw in series, K1 (`Converter::apply_back` puts the screw's compliance into it), between the rotor and the
/// carriage, referred to the motor. The train's frequency with the motor held, over sqrt(1 - the load's share), is the
/// two-mass mode with the rotor free (`resonance_hz`), with the nut 0.1, 0.4 and 0.7 m along. The train put the screw's
/// compliance in series a second time (`band_compliance`): 0.73 of the mode here.
#[test]
fn the_screw_axis_train_is_its_two_mass_mode() {
    let sim = Sim::new(&SCREW, &SCREW.config()).unwrap();
    for x in [0.1, 0.4, 0.7] {
        let train = sim.machine.trains(&[x])[0].unwrap();
        let free = train.hz / (1.0 - train.load_share).sqrt();
        let mode = sim.machine.resonance_hz(&[x]).unwrap();
        assert!((free / mode - 1.0).abs() < 1e-9, "{x}: {free} against {mode} Hz");
    }
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

/// The top speed and the hardest acceleration of a time-optimal line, by differences over its 1 ms poses.
fn time_optimal_line(entry: &MachineEntry, cfg: &Config, to: f64) -> (f64, f64) {
    let sim = Sim::new(entry, cfg).unwrap();
    let points = sim.plan_path(&sim_components::path::parse(&format!("line,{to}")).unwrap(), true).unwrap();
    let x = |k: usize| points[k].1[0];
    let top = (1..points.len()).map(|k| ((x(k) - x(k - 1)) / 1e-3).abs()).fold(0.0, f64::max);
    let acc = (1..points.len() - 1).map(|k| ((x(k + 1) - 2.0 * x(k) + x(k - 1)) / 1e-6).abs()).fold(0.0, f64::max);
    (top, acc)
}

/// The time-optimal plan keeps the screw's speed rating (`Machine::rigid_checks`, "screw-speed"): the 1.6 m screw of
/// the test above whirls at 1250 r/min, which on the 20 mm lead is 0.41667 m/s, less the 1 % the plan keeps back of a
/// speed rating, 0.41250 m/s; the motor alone would run the carriage at far more.
#[test]
fn the_time_optimal_plan_keeps_the_screws_speed_rating() {
    let mut cfg = rigid(&SCREW);
    let (free, _) = time_optimal_line(&SCREW, &cfg, 0.7);
    cfg.set_text("screw.length_m = 1.6").unwrap();
    let (top, _) = time_optimal_line(&SCREW, &cfg, 0.7);
    let by_hand: f64 = 0.99 * 1250.0 / 60.0 * 0.02;
    assert!((by_hand - 0.41250).abs() < 1e-5 && free > 1.5 * by_hand, "{free} m/s without the long screw");
    assert!((top - by_hand).abs() < 1e-3 * by_hand, "{top} m/s, by hand {by_hand} m/s");
}

/// The time-optimal plan keeps the rack's feed force ("rack-feed-force", the rated 500 N over the service factor 1.25:
/// 400 N) with the torque reserve 0.8: the rigid, loss-free horizontal axis pushes its 60 kg carriage and 20 kg payload
/// at no more than 0.8 * 400 / 80 = 4 m/s^2; the motor's 0.8 * 15 N m * 10 / 31.831 mm would push it at 47 m/s^2.
#[test]
fn the_time_optimal_plan_keeps_the_racks_feed_force() {
    let mut cfg = rigid(&RACK);
    cfg.set_text("rack.rated_feed_force_n = 500.0").unwrap();
    let (_, acc) = time_optimal_line(&RACK, &cfg, 1.5);
    assert!((acc - 4.0).abs() < 2e-3 * 4.0, "{acc} m/s^2, by hand 4 m/s^2");
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

/// A screw joint of an arm is coupled to its motor: another ratio is refused. The gearbox lives take each motor's own
/// `gearbox.ratio`: a config that rates them (`ratings.gearbox_reference_speed_rpm`) is taken where every joint's gearbox
/// table says the arm's ratio (the shipped arm: 120 and 80), and refused where the arm turns joints 4 to 6 through 120
/// (`arm.gear_ratios`) while their tables say 80; without the rating the arm is taken either way.
#[test]
fn an_arm_refuses_what_it_cannot_size() {
    let mut cfg = SCARA.config();
    cfg.set_text("arm.gear_ratios = [100.0, 80.0, 2.0, 50.0]").unwrap();
    assert!((SCARA.build)(&cfg).is_err());
    let all_120 = "arm.gear_ratios = [120.0, 120.0, 120.0, 120.0, 120.0, 120.0]";
    let mut unrated = URDF_ARM.config();
    unrated.set_text(all_120).unwrap();
    assert!((URDF_ARM.build)(&unrated).is_ok());
    let rated = URDF_ARM.default_config.replace("[ratings]\n", "[ratings]\ngearbox_reference_speed_rpm = 400.0\n");
    let mut cfg = Config::parse(&rated).unwrap();
    assert!(cfg.get("ratings.gearbox_reference_speed_rpm").is_ok() && (URDF_ARM.build)(&cfg).is_ok());
    cfg.set_text(all_120).unwrap();
    assert!((URDF_ARM.build)(&cfg).is_err());
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
    for (entry, text) in
        [(&SCARA, SCARA_TOOL_CONFIG), (&URDF_ARM, ARM_TOOL_CONFIG), (&ARM_ON_TRACK, ARM_ON_TRACK_TOOL_CONFIG)]
    {
        let (default, mut tool) = (entry.config(), tool_config(entry, text));
        assert!(tool.bool("kinematics.tool_space").unwrap());
        tool.set_text("kinematics.tool_space = false").unwrap();
        // The settling tolerances are the joints' in joint space, the flange's place and orientation in tool space.
        for key in ["start.pose", "cycle", "ratings.settling_tolerance_rad", "ratings.settling_tolerance_m"] {
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
    for (entry, text) in
        [(&SCARA, SCARA_TOOL_CONFIG), (&URDF_ARM, ARM_TOOL_CONFIG), (&ARM_ON_TRACK, ARM_ON_TRACK_TOOL_CONFIG)]
    {
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

/// A steel tube 60 x 4 mm, 0.8 m long along x, on joint 1 about y (its URDF mass and inertia `tube`: kg, then ixx, iyy,
/// izz about its centre in the middle), and at its end a body on joint 2 about x, the tube's own axis, with its centre
/// on that axis (`tip`, likewise): the load beyond the tube as a tip mass and a tip inertia. Joint 1 between `lower`
/// and `upper`. Bending with E = 2.1e11 Pa and G = 8.1e10 Pa, undamped, plus `edits`.
fn tube_with_tip(
    name: &str,
    tube: (f64, [f64; 3]),
    tip: (f64, [f64; 3]),
    lower: f64,
    edits: &[&str],
) -> Box<dyn Machine> {
    let inertial = |(mass, [xx, yy, zz]): (f64, [f64; 3]), at: f64| {
        format!(
            r#"<inertial><origin xyz="{at} 0 0"/><mass value="{mass}"/><inertia ixx="{xx}" ixy="0" ixz="0" iyy="{yy}" iyz="0" izz="{zz}"/></inertial>"#
        )
    };
    let path = std::env::temp_dir().join(format!("joint-kit-test-{name}.urdf"));
    std::fs::write(
        &path,
        format!(
            r#"<?xml version="1.0"?>
<robot name="{name}">
  <link name="base_link"/>
  <link name="tube">{}</link>
  <joint name="joint1" type="revolute"><parent link="base_link"/><child link="tube"/><axis xyz="0 1 0"/><limit lower="{lower}" upper="{}"/></joint>
  <link name="tip">{}</link>
  <joint name="joint2" type="revolute"><parent link="tube"/><child link="tip"/><origin xyz="0.8 0 0"/><axis xyz="1 0 0"/><limit lower="-3" upper="3"/></joint>
</robot>"#,
            inertial(tube, 0.4),
            lower + 1.14,
            inertial(tip, 0.0)
        ),
    )
    .unwrap();
    let mut cfg = lossless_config(&URDF_ARM);
    for edit in [
        format!("arm.urdf_file = {:?}", path.to_str().unwrap()),
        "arm.gear_ratios = [120.0, 120.0]".into(),
        "effects.payload = false".into(),
        "effects.link_bending = true".into(),
        "link_bending.outer_diameter_m = [0.06, 0.0]".into(),
        "link_bending.wall_m = [0.004, 0.0]".into(),
        "link_bending.youngs_modulus_pa = 2.1e11".into(),
        "link_bending.shear_modulus_pa = 8.1e10".into(),
        "link_bending.damping_ratio = 0.0".into(),
    ]
    .into_iter()
    .chain(edits.iter().map(|e| e.to_string()))
    {
        cfg.set_text(&edit).unwrap();
    }
    (URDF_ARM.build)(&cfg).unwrap()
}

const ALL_DIRECTIONS: &str = r#"link_bending.directions = ["in_plane", "out_of_plane", "torsion"]"#;

/// The tube twists as a shaft with a tip inertia J = 0.5 kg m^2: k = G I_p / L = 8.1e10 x 2 x 2.772644e-7 / 0.8 =
/// 56146.0 N m/rad, I_p = pi / 32 (0.06^4 - 0.052^4); the tube's own share J_own / 3, J_own = m (D^2 + d^2) / 8 =
/// 4.5 x (0.0036 + 0.002704) / 8 = 3.546e-3 kg m^2 (its URDF ixx), 1.182e-3: f = sqrt(56146.0 / 0.501182) / 2 pi =
/// 53.2699 Hz, sqrt(G I_p / (L J)) / 2 pi = 53.3328 Hz without the tube's share (the exact root of beta tan beta =
/// J_own / J gives 53.26988 Hz). It bends in both planes alike, a round tube with 10 kg at its tip: k = 1.875104^4 / 4 x
/// E I / L^3 = 351467 N/m, m / 4 + M = 11.125 kg, 28.2887 Hz.
#[test]
fn a_tube_twists_at_the_frequency_of_its_tip_inertia_and_bends_alike_in_both_planes() {
    let m = tube_with_tip(
        "twist",
        (4.5, [3.546e-3, 0.24, 0.24]),
        (10.0, [0.5, 0.0, 0.0]),
        -0.57,
        &["effects.joint_elasticity = false", ALL_DIRECTIONS],
    );
    let modes = m.modes(&[0.0, 0.0]);
    assert_eq!(modes.len(), 3, "{modes:?}");
    near(modes[0].hz, 28.2887, 1e-5, "bend in the joint's plane");
    near(modes[1].hz, 28.2887, 1e-5, "bend across it");
    near(modes[2].hz, 53.2699, 1e-5, "twist");
    near(modes[2].hz, 53.26988, 1e-5, "twist, exact");
    let ids: Vec<String> = m.signal_infos().into_iter().map(|s| s.id).filter(|id| id.starts_with("link.")).collect();
    assert_eq!(ids, ["link.1.bend", "link.1.bend_across", "link.1.twist"]);
}

/// Two-way (effects.link_bending_reaction): the tube, here 0.45 kg (iyy 0.024 kg m^2 about its centre), carries 20 kg
/// at its tip (0.001 kg m^2 about each axis) and hangs level on joint 1's gearbox, k_g = 2e5 N m/rad, its motor held
/// (a rotor of 1e4 kg m^2). Joint and tube are two springs in series for the tip: L^2 / k_g = 3.2e-6 m/N and
/// 1 / k_b = 1 / 351467 N/m, together k_s = 165420 N/m.
///
/// The mode: in q (joint 1) and d (the tube's tip) the masses are M_qq = 0.024 + 0.45 x 0.4^2 + 20 x 0.8^2 + 0.001 =
/// 12.897 kg m^2, B = 0.391496 x 0.45 x 0.4 + 20 x 0.8 + 1.37651 / 0.8 x 0.001 = 16.07219 kg m, M_d = 0.45 / 4 + 20 +
/// (1.37651 / 0.8)^2 x 0.001 = 20.11546 kg; det(diag(k_g, k_b) - w^2 M) = 0 gives 14.43363 Hz, the series spring on the
/// modal mass sqrt(k_s / M_d) / 2 pi = 14.43276 Hz (and 440.88 Hz). One-way the joint would ring alone at
/// sqrt(k_g / M_qq) / 2 pi = 19.82 Hz and the tube clamped at 21.04 Hz.
///
/// Statically the tip sinks by L tau / k_g + Q_g / k_b, tau = g (20 x 0.8 + 0.45 x 0.4) = 158.6716 N m, Q_g =
/// g (20 + 0.391496 x 0.45) = 197.8609 N: 0.6346864 + 0.5629561 = 1.197642 mm, within 1 % of the series springs under
/// the tip mass alone, M g / k_s = 1.185666 mm (the tube's own weight is the rest). Let go from straight (gear untwisted,
/// tube unbent) it rings about that at the mode's frequency.
#[test]
fn a_bending_link_on_an_elastic_joint_rings_as_the_two_springs_in_series() {
    let m = tube_with_tip(
        "series",
        (0.45, [4.0e-4, 0.024, 0.024]),
        (20.0, [0.001, 0.001, 0.001]),
        -0.57,
        &[
            "effects.joint_elasticity = true",
            "effects.link_bending_reaction = true",
            "effects.gearbox_stiffness_curve = false",
            "effects.gearbox_backlash = false",
            "gearbox.stiffness_nm_rad = 2.0e5",
            "motor.rotor_inertia_kgm2 = 1.0e4",
        ],
    );
    // The lowest mode the machine reports (the input shaper's) is the coupled one.
    let modes = m.modes(&[0.0, 0.0]);
    near(modes[0].hz, 14.43363, 1e-5, "coupled mode");
    near(modes[0].hz, 14.43276, 1e-4, "series spring on the modal mass");
    near(modes[1].hz, 440.88, 1e-4, "second mode");
    let (mut state, mut hold) = (Vec::new(), vec![0.0; 2]);
    m.equilibrium(&[0.0, 0.0], &mut state, &mut hold);
    let at = state.len() - 2;
    let tip = |s: &[f64]| 0.8 * s[0] + s[at];
    let rest = tip(&state);
    // Straight: joint 1 where its rotor is (state: joints, their speeds, then the rotors), the tube unbent.
    state[0] = state[4] / 120.0;
    state[at] = 0.0;
    near(rest - tip(&state), 1.197642e-3, 1e-4, "static deflection");
    near(rest - tip(&state), 1.185666e-3, 0.011, "series springs under the tip mass");
    let (dt, mut rk) = (sim_core::testkit::DT, sim_core::Rk4::new(state.len()));
    let (mut low, mut high, mut ups, mut was) = (f64::INFINITY, f64::NEG_INFINITY, Vec::new(), tip(&state) - rest);
    for step in 1..=(1.0 / dt) as usize {
        sim_core::advance(m.as_ref(), &mut rk, &mut state, &hold, dt, sim_core::testkit::SUBSTEPS);
        let now = tip(&state) - rest;
        (low, high) = (low.min(now), high.max(now));
        if was < 0.0 && now >= 0.0 {
            ups.push((step as f64 - now / (now - was)) * dt);
        }
        was = now;
    }
    let hz = (ups.len() - 1) as f64 / (ups[ups.len() - 1] - ups[0]);
    near(hz, 14.43363, 2e-3, "ringing");
    near(0.5 * (high - low), 1.197642e-3, 0.01, "amplitude");
    assert!((0.5 * (high + low)).abs() < 0.01 * 1.197642e-3, "it rings about {} m from rest", 0.5 * (high + low));
}

/// With the links two-way and no losses the energy stays within the suite's 0.1 % of what its pulse puts in: the tube
/// with its tip hanging under gravity (where the bent links shift the weights), bending and twisting, on its elastic
/// and on rigid joints.
#[test]
fn two_way_bending_keeps_the_energy_of_a_hanging_link() {
    std::thread::scope(|scope| {
        for elastic in [true, false] {
            scope.spawn(move || {
                let elasticity = format!("effects.joint_elasticity = {elastic}");
                let hanging = tube_with_tip(
                    &format!("hanging-{elastic}"),
                    (4.5, [3.546e-3, 0.24, 0.24]),
                    (10.0, [0.05, 0.02, 0.02]),
                    1.0,
                    &[&elasticity, "effects.link_bending_reaction = true", ALL_DIRECTIONS],
                );
                sim_core::testkit::energy_conservation(hanging.as_ref());
            });
        }
    });
}

/// Two-way, at every instant: on the six-axis arm under gravity, loss-free and with no motor torque, its links bent by
/// 1 mm (1 mrad twisted) and moving, the joints turning, the machine's energy changes along its own rates by nothing:
/// dE/dt by central differences over +-1e-7 s of the state's rates, within 1e-4 W (1e-5 W is left, rounding). Without
/// G (the bent links' weight shift at the joints) it is off by 0.22 W, without 1/2 M_d' d' by 0.012 W (both tried).
#[test]
fn two_way_bending_trades_energy_without_loss_at_every_instant() {
    for elastic in [false, true] {
        let mut cfg = lossless_config(&URDF_ARM);
        for edit in [
            format!("effects.joint_elasticity = {elastic}"),
            "effects.link_bending = true".into(),
            "effects.link_bending_reaction = true".into(),
            ALL_DIRECTIONS.into(),
            "link_bending.damping_ratio = 0.0".into(),
        ] {
            cfg.set_text(&edit).unwrap();
        }
        let m = (URDF_ARM.build)(&cfg).unwrap();
        let n = 6;
        let (mut state, mut hold) = (Vec::new(), vec![0.0; n]);
        m.equilibrium(&sim_core::testkit::mid_pose(m.as_ref()), &mut state, &mut hold);
        let speeds = [0.5, -0.4, 0.3, 0.6, -0.5, 0.4];
        state[n..2 * n].copy_from_slice(&speeds);
        if elastic {
            let ratios = cfg.f64s("arm.gear_ratios").unwrap();
            (0..n).for_each(|k| state[3 * n + k] = ratios[k] * speeds[k]);
        }
        let modes = m.signal_infos().iter().filter(|s| s.id.starts_with("link.")).count();
        let at = state.len() - 2 * modes;
        for i in 0..modes {
            (state[at + i], state[at + modes + i]) = (1e-3 * (1.0 - 0.3 * i as f64 / modes as f64), 0.05);
        }
        let mut rates = vec![0.0; state.len()];
        m.derivatives(&state, &[0.0; 6], &mut rates);
        let h = 1e-7;
        let along = |s: f64| -> Vec<f64> { state.iter().zip(&rates).map(|(x, r)| x + s * r).collect() };
        let change = (m.energy(&along(h)) - m.energy(&along(-h))) / (2.0 * h);
        assert!(change.abs() < 1e-4, "elastic {elastic}: the energy changes by {change} W");
    }
}

/// The same on the six-axis arm, the suite's loss-free elastic config with the payload (without gravity, as the suite
/// runs it), its links bending two-way in their joints' planes.
#[test]
fn two_way_bending_keeps_the_six_axis_arms_energy() {
    let mut cfg = suite_configs(&URDF_ARM).remove(2);
    assert!(cfg.bool("effects.joint_elasticity").unwrap() && cfg.bool("effects.payload").unwrap());
    for edit in
        ["effects.link_bending = true", "effects.link_bending_reaction = true", "link_bending.damping_ratio = 0.0"]
    {
        cfg.set_text(edit).unwrap();
    }
    sim_core::testkit::energy_conservation((URDF_ARM.build)(&cfg).unwrap().as_ref());
}

/// The shipped six-axis arm with its links two-way in all three directions holds still under its weight at rest (the
/// bent links' weight shift held by the gears) and accounts for the links' damping among its losses.
#[test]
fn two_way_bending_holds_still_and_balances_the_losses() {
    let mut cfg = URDF_ARM.config();
    for edit in ["effects.link_bending = true", "effects.link_bending_reaction = true", ALL_DIRECTIONS] {
        cfg.set_text(edit).unwrap();
    }
    let m = (URDF_ARM.build)(&cfg).unwrap();
    assert!(m.signal_infos().iter().any(|s| s.id == "power.link_bending"));
    sim_core::testkit::static_hold(m.as_ref());
    sim_core::testkit::loss_budget(m.as_ref());
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

/// The palletiser's forearm drive parallelogram as shipped ([estimate] `arm.bars_*`): the crank (6 kg) turns with the
/// forearm about the upper arm's pivot, its centre 0.15 m behind it; the rod (8 kg) stays parallel to the upper arm,
/// pinned on the forearm's rear lever 0.3 m behind the elbow. With the upper arm upright and the forearm level (the
/// motors' joints 2 and 3 at 0) both centres lie behind the pivots: by virtual work the rod's weight acts on the
/// forearm's angle (motor 3) at its pin, 8 g x 0.3 = 23.536 N m, the crank's at its centre, 6 g x 0.15 = 8.826 N m,
/// 32.362 N m against the forearm's weight, and nothing on the upper arm's angle (its turn moves the rod's centre
/// across, not up). The motors hold that through their 160:1 gears: 0.20226 N m less on motor 3, none on motor 2.
#[test]
fn the_palletisers_rod_and_crank_hold_by_hand() {
    let mut cfg = rigid(&PALLETISER);
    cfg.set_text("kinematics.tool_space = false").unwrap();
    cfg.set_text("effects.gravity = true").unwrap();
    let hold = |cfg: &Config| {
        let (mut state, mut hold) = (Vec::new(), vec![0.0; 4]);
        (PALLETISER.build)(cfg).unwrap().equilibrium(&[0.0; 4], &mut state, &mut hold);
        hold
    };
    let with = hold(&cfg);
    for key in ["on", "turns_with", "pin_m", "mass_kg", "com_m", "inertia_kgm2"] {
        cfg.set_text(&format!("arm.bars_{key} = []")).unwrap();
    }
    let without = hold(&cfg);
    let g = 9.80665;
    let by_hand = (8.0 * 0.3 + 6.0 * 0.15) * g / 160.0;
    println!("with {with:?}, without {without:?}, by hand {by_hand} N m");
    assert!((with[1] - without[1]).abs() < 1e-9, "{with:?} {without:?}");
    near((without[2] - with[2]).abs(), by_hand, 1e-9, "motor 3");
    assert!(with[2].abs() < without[2].abs(), "the bars balance the forearm: {with:?} {without:?}");
}

/// The palletiser's upper arm bending in its joints' plane while its parallelogram holds the forearm (`held_from`,
/// `bar_field`): a steel tube 150 x 10 mm, E I = 2.1e11 x pi / 64 (0.15^4 - 0.13^4) = 2.6122e6 N m^2, its own 25 kg
/// over 1 m. The crank, pinned on the turret, turns with the forearm and so holds its turn against the arm's bend: the
/// forearm, wrist, flange and carton (20 + 5 + 8 + 20 = 53 kg) move with the arm's tip and do not turn; the rod (8 kg),
/// pinned on the forearm's lever and parallel to the arm's chord, swings about its lower end, m / 3 at the tip; the crank
/// stays. The mode's mass by hand 25 / 4 + 53 + 8 / 3 = 61.91667 kg on 1.875104^4 / 4 E I / L^3: 53.626 Hz. The exact
/// Euler-Bernoulli cantilever of 25 kg with 55.6667 kg at its tip and no tip inertia (the forearm held level,
/// `beam::cantilever_first_frequency`) rings at 52.980 Hz, 1.22 % lower: the bend-mode method's known overestimate (1.2 to 2.6 %
/// in the test case). Refused before 22 November. Without the bars the forearm and all beyond turn with
/// the tip's slope 1.3765 d / L, and their turning inertia about the tip lowers the mode to 35.17 Hz.
#[test]
fn the_parallelogram_holds_the_forearm_against_the_upper_arms_bend() {
    let bent = |bars: bool| (PALLETISER.build)(&bent_config(bars, false)).unwrap();
    let ei = 2.1e11 * std::f64::consts::PI / 64.0 * (0.15f64.powi(4) - 0.13f64.powi(4));
    let k = 1.875_104_068_711_961f64.powi(4) / 4.0 * ei;
    let by_hand = (k / (25.0 / 4.0 + 53.0 + 8.0 / 3.0)).sqrt() / std::f64::consts::TAU;
    let exact =
        sim_components::beam::cantilever_first_frequency(ei, 25.0, 53.0 + 8.0 / 3.0, 0.0, 1.0) / std::f64::consts::TAU;
    let (held, free) = (bent(true).modes(&[0.0; 4]), bent(false).modes(&[0.0; 4]));
    let nearest = |modes: &[sim_core::Mode], f: f64| {
        modes.iter().map(|m| m.hz).fold(0.0, |b, x| if (x - f).abs() < (b - f).abs() { x } else { b })
    };
    let (found, unheld) = (nearest(&held, by_hand), nearest(&free, by_hand));
    println!("held {found} Hz, by hand {by_hand} Hz, exact beam {exact} Hz; without the bars {unheld} Hz");
    near(found, by_hand, 1e-6, "the held mode");
    assert!(found > exact && found / exact - 1.0 < 0.026, "{found} against the exact {exact}");
    assert!((unheld - by_hand).abs() > 0.05 * by_hand, "without the bars {unheld} Hz");
    // Acting back on the joints (effects.link_bending_reaction), the bars' fields in the coupling with their rates: its
    // energy holds over the suite's run with gravity (`testkit::energy_conservation`): 1.8e-8 J of the 2.8 mJ put in over
    // 10 s with the joints' stops off, the arm without its bars 6.9e-9 J of 3.2 mJ (the integrator's level). With the
    // bars the swinging upper arm reaches its stop at 0.9 rad: the steps that meet it cut 16 times finer
    // (`mech::STOP_SUBSTEPS`), 5.3e-9 J. Before 22 November 1.3e-5 J, in steps at each contact (1.3e-6 J at half the
    // step): the stop's damper taking hold within a step, integrated across.
    let cfg = bent_config(true, true);
    let (drift, injected) = sim_core::testkit::energy_drift((PALLETISER.build)(&cfg).unwrap().as_ref());
    println!("with the stops: {drift} J of {injected} J");
    assert!(injected > 0.0 && drift < 1e-5 * injected, "{drift} J of {injected} J");
}

/// The palletiser rigid and loss-free in joint space, its upper arm bending in its plane (a steel tube 150 x 10 mm,
/// undamped), with or without its parallelogram's bars, with or without the bend acting back on the joints.
/// The palletiser's rod stretching under the torque it passes (effects.bar_elasticity, [estimate] E A / L = 2.14e8 N/m
/// on the forearm's 0.3 m lever, carrying motor 3): k c^2 = 2.14e8 x 0.09 = 1.926e7 N m/rad in series with the
/// gearbox's 2e5, K = 1 / (1 / 2e5 + 1 / 1.926e7) = 197944.6 N m/rad. Motor 3's train (the planner's) then rings
/// sqrt(K / 2e5) = 0.994834 times as high, the other motors' as before; the held modes see it too (one lower, none
/// higher).
#[test]
fn the_parallelogram_rod_stretches_in_series_with_motor_threes_gearbox() {
    let machine = |on: bool| {
        let mut cfg = PALLETISER.config();
        cfg.set_text(&format!("effects.bar_elasticity = {on}")).unwrap();
        (PALLETISER.build)(&cfg).unwrap()
    };
    let (off, on) = (machine(false), machine(true));
    let pose = [1.2, -0.5, 1.3, 0.0];
    let (a, b) = (off.trains(&pose), on.trains(&pose));
    let k: f64 = 1.0 / (1.0 / 2e5 + 1.0 / (2.14e8 * 0.09));
    assert!((k - 197_944.6).abs() < 0.1, "{k}");
    for m in 0..4 {
        let (a, b) = (a[m].unwrap().hz, b[m].unwrap().hz);
        let want = if m == 2 { (k / 2e5).sqrt() } else { 1.0 };
        assert!((b / a - want).abs() < 1e-9, "motor {}: {} against {want}", m + 1, b / a);
    }
    let (a, b) = (off.modes(&pose), on.modes(&pose));
    assert!(a.len() == b.len() && a.iter().zip(&b).all(|(a, b)| b.hz <= a.hz * (1.0 + 1e-12)), "{a:?} {b:?}");
    assert!(a.iter().zip(&b).any(|(a, b)| b.hz < a.hz * (1.0 - 1e-4)), "{a:?} {b:?}");
}

/// A forearm on a rocking rod (effects.bar_rocking) with one motor: a post (URDF joint 1, held by a zero row of
/// `arm.joint_coupling`) and on it, 0.5 m up, the forearm about y (joint 2, the motor's): 20 kg, its centre 0.5 m out,
/// 1 kg m^2 about it. The crank (6 kg, its centre 0.15 m behind the pivot, 0.045 kg m^2 about it) pinned on the post at
/// the pivot turns with the forearm; the rod (8 kg) pinned on the forearm 0.3 m behind the pivot stays parallel to the
/// post, its axial stiffness 2.14e8 N/m carrying the motor's joint. Loss-free, without gravity, plus `edits`.
fn rocker(edits: &[&str]) -> Box<dyn Machine> {
    let path = std::env::temp_dir().join("joint-kit-test-rocker.urdf");
    std::fs::write(
        &path,
        r#"<?xml version="1.0"?>
<robot name="rocker">
  <link name="base_link"/>
  <link name="post"><inertial><origin xyz="0 0 0.25"/><mass value="10"/><inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.05"/></inertial></link>
  <joint name="joint1" type="revolute"><parent link="base_link"/><child link="post"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <link name="forearm"><inertial><origin xyz="0.5 0 0"/><mass value="20"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="1.0"/></inertial></link>
  <joint name="joint2" type="revolute"><parent link="post"/><child link="forearm"/><origin xyz="0 0 0.5"/><axis xyz="0 1 0"/><limit lower="-1.5" upper="1.5"/></joint>
</robot>"#,
    )
    .unwrap();
    let mut cfg = lossless_config(&URDF_ARM);
    for edit in [
        format!("arm.urdf_file = {:?}", path.to_str().unwrap()),
        "arm.gear_ratios = [160.0]".into(),
        "arm.joint_coupling = [[0.0], [1.0]]".into(),
        "arm.bars_on = [1.0, 2.0]".into(),
        "arm.bars_turns_with = [2.0, 1.0]".into(),
        "arm.bars_pin_m = [0.0, 0.0, 0.5, -0.3, 0.0, 0.0]".into(),
        "arm.bars_mass_kg = [6.0, 8.0]".into(),
        "arm.bars_com_m = [-0.15, 0.0, 0.0, 0.0, 0.0, -0.5]".into(),
        "arm.bars_inertia_kgm2 = [0.004, 0.045, 0.045, 0.0, 0.0, 0.0, 0.66667, 0.66667, 0.006, 0.0, 0.0, 0.0]".into(),
        "arm.bars_axial_stiffness_n_m = [0.0, 2.14e8]".into(),
        "arm.bars_axial_damping_n_s_m = [0.0, 0.0]".into(),
        "arm.bars_carries = [0.0, 1.0]".into(),
        "effects.bar_rocking = true".into(),
        "effects.payload = false".into(),
        "effects.gravity = false".into(),
    ]
    .into_iter()
    .chain(edits.iter().map(|e| e.to_string()))
    {
        cfg.set_text(&edit).unwrap();
    }
    (URDF_ARM.build)(&cfg).unwrap()
}

/// The rocking forearm's two-mass mode (effects.bar_rocking), the motor held (a rotor of 1e4 kg m^2 behind 160:1): the
/// crank on the gearbox's spring k_g = 2e5 N m/rad, the forearm on the rod's k c^2 = 2.14e8 x 0.3^2 = 1.926e7 N m/rad
/// against the crank. The motor's joint q turns the crank, q + r the forearm: I_c = 0.045 + 6 x 0.15^2 = 0.18 kg m^2,
/// I_f = 1 + 20 x 0.5^2 + 8 x 0.3^2 = 6.72 kg m^2 (the rod moves with its pin without turning), so M = [[I_c + I_f, I_f],
/// [I_f, I_f]] and det(diag(k_g, k c^2) - w^2 M) = 0: I_c I_f w^4 - (k_g I_f + k c^2 (I_c + I_f)) w^2 + k_g k c^2 = 0,
/// 26.96 Hz (the arm on the gearbox, a little below sqrt(k_g / (I_c + I_f)) / 2 pi = 27.10 Hz) and 1676 Hz (the crank
/// between the two springs). Without the crank the two columns of M would be equal and it singular.
#[test]
fn the_rocking_forearm_rings_as_the_two_masses_by_hand() {
    let m = rocker(&[
        "effects.joint_elasticity = true",
        "effects.gearbox_stiffness_curve = false",
        "effects.gearbox_backlash = false",
        "gearbox.stiffness_nm_rad = 2.0e5",
        "motor.rotor_inertia_kgm2 = 1.0e4",
    ]);
    let (ic, i_f, kg, kr): (f64, f64, f64, f64) =
        (0.045 + 6.0 * 0.15 * 0.15, 1.0 + 20.0 * 0.25 + 8.0 * 0.09, 2e5, 2.14e8 * 0.09);
    let (a, b, c) = (ic * i_f, kg * i_f + kr * (ic + i_f), kg * kr);
    let root = (b * b - 4.0 * a * c).sqrt();
    let hz = |l: f64| l.sqrt() / std::f64::consts::TAU;
    let (low, high) = (hz(2.0 * c / (b + root)), hz((b + root) / (2.0 * a)));
    let modes = m.modes(&[0.0]);
    println!("by hand {low} Hz and {high} Hz; the machine's {modes:?}");
    assert!((low - 26.96).abs() < 0.01 && (high - 1676.0).abs() < 1.0, "{low} {high}");
    let nearest =
        |f: f64| modes.iter().map(|m| m.hz).fold(0.0, |b, x| if (x - f).abs() < (b - f).abs() { x } else { b });
    near(nearest(low), low, 1e-6, "the arm on the gearbox");
    near(nearest(high), high, 1e-6, "the crank between the springs");
    assert!(m.signal_infos().iter().any(|s| s.id == "bar.rocking"));
    // Without the crank the rocking is refused.
    let mut cfg = URDF_ARM.config();
    cfg.set_text("effects.bar_rocking = true").unwrap();
    assert!((URDF_ARM.build)(&cfg).is_err(), "the six-axis arm has no rod to rock on");
}

/// The palletiser's forearm at rest on its rod (effects.bar_rocking), the upper arm upright and the forearm level
/// (motors' joints 0), its carton on: by virtual work the rocking r turns the forearm about the elbow (20 kg, its centre
/// 0.55 m out) and moves the wrist, flange and carton (5 + 8 + 20 = 33 kg) with the forearm's tip 1.2 m out, without
/// turning them (the flange stays level), and the rod (8 kg) with its pin 0.3 m behind the elbow; the crank stays. Their
/// weights turn it by g (20 x 0.55 + 33 x 1.2 - 8 x 0.3) = 48.2 g = 472.68 N m against k c^2 = 1.926e7 N m/rad: it rests
/// at 2.4542e-5 rad, the forearm's tip down (positive about y), and stays there.
#[test]
fn the_palletisers_forearm_sags_on_its_rod_by_hand() {
    let mut cfg = PALLETISER.config();
    cfg.set_text("effects.bar_rocking = true").unwrap();
    cfg.set_text("kinematics.tool_space = false").unwrap();
    let m = (PALLETISER.build)(&cfg).unwrap();
    let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
    m.equilibrium(&[0.0; 4], &mut state, &mut torque);
    let at = state.len() - 2;
    let by_hand = 48.2 * 9.80665 / (2.14e8 * 0.09);
    println!("rests at {} rad, by hand {by_hand} rad", state[at]);
    near(state[at], by_hand, 1e-6, "the rod's sag");
    let mut d = vec![0.0; state.len()];
    m.derivatives(&state, &torque, &mut d);
    assert!(d[at + 1].abs() < 1e-6, "the rocking accelerates at {} rad/s^2 at rest", d[at + 1]);
    // The rod's spring is the rocking's: not in series with motor 3's gearbox as well, though bar_elasticity is on.
    cfg.set_text("effects.bar_elasticity = true").unwrap();
    let both = (PALLETISER.build)(&cfg).unwrap();
    let pose = [1.2, -0.5, 1.3, 0.0];
    assert_eq!(both.trains(&pose)[2].unwrap().hz, m.trains(&pose)[2].unwrap().hz);
}

/// With the rocking on and no losses the palletiser keeps its energy within the suite's 0.1 % of what its pulse puts in,
/// rigid and elastic, with gravity; with the rod's damper on its loss row closes the budget.
#[test]
fn the_rocking_forearm_keeps_the_energy() {
    for elastic in [false, true] {
        let mut cfg = rigid(&PALLETISER);
        for edit in [
            "kinematics.tool_space = false",
            "effects.bar_rocking = true",
            "arm.bars_axial_damping_n_s_m = [0.0, 0.0]",
            &format!("effects.joint_elasticity = {elastic}"),
        ] {
            cfg.set_text(edit).unwrap();
        }
        let m = (PALLETISER.build)(&cfg).unwrap();
        let (drift, injected) = sim_core::testkit::energy_drift(m.as_ref());
        println!("elastic {elastic}: {drift} J of {injected} J");
        assert!(injected > 0.0 && drift < 1e-3 * injected, "{drift} J of {injected} J");
        cfg.set_text("arm.bars_axial_damping_n_s_m = [0.0, 7.35e3]").unwrap();
        let m = (PALLETISER.build)(&cfg).unwrap();
        assert!(m.signal_infos().iter().any(|s| s.id == "power.bar_rocking"));
        sim_core::testkit::loss_budget(m.as_ref());
    }
}

fn bent_config(bars: bool, reaction: bool) -> Config {
    {
        let mut cfg = rigid(&PALLETISER);
        cfg.set_text(&format!("effects.link_bending_reaction = {reaction}")).unwrap();
        for edit in [
            "kinematics.tool_space = false",
            "effects.link_bending = true",
            r#"link_bending.directions = ["in_plane"]"#,
            "link_bending.outer_diameter_m = [0.0, 0.15, 0.0, 0.0, 0.0]",
            "link_bending.wall_m = [0.0, 0.01, 0.0, 0.0, 0.0]",
            "link_bending.youngs_modulus_pa = 2.1e11",
            "link_bending.damping_ratio = 0.0",
        ] {
            cfg.set_text(edit).unwrap();
        }
        if !bars {
            for key in ["on", "turns_with", "pin_m", "mass_kg", "com_m", "inertia_kgm2"] {
                cfg.set_text(&format!("arm.bars_{key} = []")).unwrap();
            }
        }
        cfg
    }
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
    let n = null_direction(&pose, &fk, &q);
    let size = n.iter().map(|v| v * v).sum::<f64>().sqrt();
    assert!(size > 0.1, "a null space of {size}");
    let along: f64 = (0..7).map(|k| n[k] / size * (q[k] - home[k])).sum();
    let off: f64 = (0..7).map(|k| (q[k] - home[k]).powi(2)).sum::<f64>().sqrt();
    assert!(off > 0.05 && along.abs() < 1e-6 * off.max(1.0), "{along} rad of {off} along the null space");
}

/// The direction of the seven-joint arm's null space at joints `q`: J (6 x 7, of the tool's axes, by central
/// differences), n = (I - J^T (J J^T)^-1 J) e for the e of the joint the null space is largest along.
fn null_direction(pose: &tool::Pose, fk: tool::Forward, q: &[f64; 7]) -> Vec<f64> {
    let q = *q;
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
    (0..7)
        .map(null)
        .max_by(|a, b| a.iter().map(|v| v * v).sum::<f64>().total_cmp(&b.iter().map(|v| v * v).sum()))
        .unwrap()
}
/// `kinematics.redundancy = "joint_limits"` on the seven-joint URDF: the pose of
/// `the_seven_joint_arm_reaches_a_pose_nearest_its_home_posture` is reached to 1e-9 as well, and the spare joint now
/// keeps the joints nearest the middles m of their ranges s, weighed (s_min / s)^2 (s_min = 3.8 rad of joint 4): the
/// weighted offset W (q - m) has no part along the null space, and its sum sum w (q - m)^2 is below the home solution's.
/// The machine built with the key solves the same joints (its motors over the gear ratios).
#[test]
// The URDF's limits as written, 6.28 among them.
#[allow(clippy::approx_constant)]
fn the_seven_joint_arm_can_spend_its_spare_joint_on_staying_off_its_stops() {
    let mut cfg = tool_config(&URDF_ARM, ARM7_TOOL_CONFIG);
    let chain = arm::chain(&cfg).unwrap();
    let home = cfg.f64s("kinematics.home_joints").unwrap();
    let fk = |q: &[f64]| chain.tip(q);
    // urdf/seven-axis.urdf's limits.
    let ranges = [(-2.967, 2.967), (-1.571, 2.356), (-2.9, 2.9), (-2.6, 1.2), (-3.3, 3.3), (-2.1, 2.1), (-6.28, 6.28)];
    let by_home = tool::Pose::new(&fk, home.clone(), vec![0, 1, 2, 3, 4, 5], 0.86, 30.0).unwrap();
    let off_stops =
        tool::Pose::new(&fk, home.clone(), vec![0, 1, 2, 3, 4, 5], 0.86, 30.0).unwrap().away_from_limits(&ranges);
    let mut at_home = [0.0; 6];
    by_home.forward(&fk, &home, &mut at_home);
    let target =
        [at_home[0] - 0.1, at_home[1] + 0.15, at_home[2] - 0.08, at_home[3] + 0.3, at_home[4], at_home[5] - 0.1];
    let (mut a, mut b) = ([0.0; 7], [0.0; 7]);
    assert!(by_home.inverse(&fk, &target, &mut a));
    assert!(off_stops.inverse(&fk, &target, &mut b));
    let mut back = [0.0; 6];
    off_stops.forward(&fk, &b, &mut back);
    assert!(back.iter().zip(&target).all(|(x, y)| (x - y).abs() < 1e-9), "{back:?} against {target:?}");
    let w: Vec<f64> = ranges.iter().map(|(l, h)| (3.8 / (h - l)).powi(2)).collect();
    let mid: Vec<f64> = ranges.iter().map(|(l, h)| 0.5 * (l + h)).collect();
    let cost = |q: &[f64; 7]| (0..7).map(|k| w[k] * (q[k] - mid[k]).powi(2)).sum::<f64>();
    assert!(cost(&b) < cost(&a) - 1e-3, "{} against the home solution's {}", cost(&b), cost(&a));
    let n = null_direction(&off_stops, &fk, &b);
    let size = n.iter().map(|v| v * v).sum::<f64>().sqrt();
    let along: f64 = (0..7).map(|k| n[k] / size * w[k] * (b[k] - mid[k])).sum();
    assert!(along.abs() < 1e-6, "{along} along the null space");
    // The machine reads the key and solves the same.
    cfg.set_text("kinematics.redundancy = \"joint_limits\"").unwrap();
    let m = (URDF_ARM.build)(&cfg).unwrap();
    let ratios = cfg.f64s("arm.gear_ratios").unwrap();
    let mut motors = [0.0; 7];
    m.inverse(&target, &mut motors);
    assert!((0..7).all(|k| (motors[k] / ratios[k] - b[k]).abs() < 1e-9), "{motors:?} against {b:?}");
    cfg.set_text("kinematics.redundancy = \"nearest\"").unwrap();
    assert!((URDF_ARM.build)(&cfg).is_err());
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

/// A machine built at rest stays there: its drives start with the torque that holds the load where the tool stands,
/// with the cogging, 0.02 N m sin(24 rotor angle), at the angle the rotor stands at. The palletiser's motors start at
/// angles where it is up to 0.02 N m (asserted below: at least 5 mN m on one). The hold used to be worked out at the
/// setpoint, where the motors' angles put the tool without its gears' twist (motor 3 0.39 rad further on, the tool
/// 4 mm off), with that pose's gravity and cogging: the machine moved 1.2 um in its first 0.2 s while its drives
/// settled. Without the encoder's counts it stays within 1e-9 m over 0.2 s (seen: 1.2e-13 m); with its 22 bits the
/// position loop corrects to the nearest count, 2 pi / 2^22 = 1.5e-6 rad at a motor, 9.4e-9 rad at a 160:1 joint, at
/// most 1.7 m from the turret: within half of 16 nm (seen: 3.2 nm).
#[test]
fn a_machine_built_at_rest_stays_there() {
    for (resolution, limit) in [(false, 1e-9), (true, 0.5 * 1.7 * std::f64::consts::TAU / 4194304.0 / 160.0)] {
        let mut cfg = PALLETISER.config();
        cfg.set_text(&format!("effects.encoder_resolution = {resolution}")).unwrap();
        let mut sim = sim_components::Sim::new(&PALLETISER, &cfg).unwrap();
        let n = sim.machine.axes().len();
        let pose = |s: &sim_components::Sim| {
            let (mut p, mut v) = (vec![0.0; n], vec![0.0; n]);
            s.machine.tool_state(&s.state, &mut p, &mut v);
            p
        };
        let (mut angle, mut speed) = (vec![0.0; 4], vec![0.0; 4]);
        sim.machine.motor_state(&sim.state, &mut angle, &mut speed);
        assert!(angle.iter().any(|a| (0.02 * (24.0 * a).sin()).abs() > 5e-3), "no cogging at {angle:?}");
        let start = pose(&sim);
        let mut most = 0f64;
        sim.run(0.2, |s| most = pose(s).iter().zip(&start).map(|(a, b)| (a - b).abs()).fold(most, f64::max));
        assert!(most < limit, "resolution {resolution}: moved {most} m, limit {limit} m");
    }
}

/// Friction feedforward from the machine's own model, by hand, on the screw axis in its cruise at 0.8 m/s (the move
/// 0.1 -> 0.7 m). The guides slide at 30 N plus the nut's preload, 0.05 / sqrt(tan b) Fpr l / 2 pi with tan b =
/// 0.02 / (pi 0.025) = 0.254648: 0.099083 x 600 x 0.02 / 2 pi = 0.189235 N m at the screw, times 2 pi / 0.02 = 314.159:
/// 59.4499 N; plus 40 N s/m x 0.8 m/s: 121.4499 N (tanh(0.8 / 0.002) = 1). Through the screw's ratio 314.159 rad/m and
/// its efficiency 0.9 (the coupling's 1): 121.4499 / (314.159 x 0.9) = 0.429541 N m. The motor shaft at 251.327 rad/s:
/// 0.05 tanh(502.65) + 2e-4 x 251.327 = 0.100265 N m. So 0.529807 N m. With it the velocity loop's integrator carries
/// nothing in the cruise (under 1 % of it); without it, it carries the whole friction.
#[test]
fn the_machine_friction_feedforward_is_the_steady_friction_and_the_integrator_carries_nothing() {
    let cruise = |model: bool| {
        let mut cfg = SCREW.config();
        cfg.set_text("effects.friction_feedforward = true").unwrap();
        let source = if model { "machine" } else { "characteristic" };
        cfg.set_text(&format!("drive.friction_feedforward_model = \"{source}\"")).unwrap();
        let mut sim = Sim::new(&SCREW, &cfg).unwrap();
        sim.command(Command::MoveTo(vec![0.7])).unwrap();
        let (mut ff, mut integrator, mut n) = (0.0, 0.0, 0);
        sim.run(0.8, |s| {
            if s.time() > 0.4 && s.control.set_acc[0] == 0.0 && (s.control.set_vel[0] - 0.8).abs() < 1e-12 {
                (ff, integrator, n) =
                    (s.control.drives[0].friction_ff, integrator + s.control.drives[0].controller_torque, n + 1);
            }
        });
        assert!(n > 1000, "{n} samples in the cruise");
        (ff, integrator / n as f64)
    };
    let preload = 0.05 / (0.02 / (std::f64::consts::PI * 0.025)).sqrt() * 600.0 * 0.02 / std::f64::consts::TAU;
    let ratio = std::f64::consts::TAU / 0.02;
    let expected = 0.05 + 2e-4 * 0.8 * ratio + (30.0 + preload * ratio + 40.0 * 0.8) / (ratio * 0.9);
    assert!((expected - 0.529807).abs() < 1e-6, "by hand {expected}");
    let (ff, integrator) = cruise(true);
    assert!((ff - expected).abs() < 1e-4 * expected, "feedforward {ff} N m, by hand {expected}");
    assert!(integrator.abs() < 0.01 * expected, "the integrator carries {integrator} N m");
    let (ff, integrator) = cruise(false);
    assert!(ff == 0.0 && (integrator - expected).abs() < 0.02 * expected, "without: {integrator} N m");
}

/// The palletiser's turret on its gear, as it rings after a long move in x (its arm out to 1.79 m): the machine reports
/// its joints' modes with the motors held, K x = w^2 M x (`JointMachine::held_modes`), 8.21 Hz at the move's start (8.30
/// before the parallelogram's rod and crank were added, 22 November: their 14 kg turn with the turret too) and
/// 5.95 Hz at its end, the turret's (4 mm of its sway in x 0.3 s after the move, `a_move_arrives_and_comes_to_rest`).
/// The ring measured from the tool's speed rings lower, at 5.30 Hz: the velocity loop does not hold the motor rigidly.
/// Its cascade (Kv, then PI with Kp and Tn) holds the rotor with Z = Kp (1 + 1 / (j w Tn)) (Kv + j w) - w^2 J at the
/// motor, i^2 Z at the joint, in series with the gear's k: the mode then rings at the held one times sqrt(Re(k_s) / k),
/// k_s = k i^2 Z / (k + i^2 Z), worked out at its own frequency: at 5.3 Hz Z = 0.4 (1 - 1.0 j)(50 + 33.3 j) = 33.3 -
/// 6.7 j N m/rad, i^2 Z about 8.5e5 N m/rad against k = 2e5: 5.95 sqrt(0.81) = 5.36 Hz, within 2 % of the ring (1.0 %).
#[test]
fn the_palletisers_turret_rings_at_its_held_mode_softened_by_the_velocity_loop() {
    use sim_components::{Command, Sim};
    // Without the gears' mesh friction, which sticks the turret to its housing more and more as the ring dies away and
    // so raises its frequency (5.60 Hz over the 3 s with it).
    let mut cfg = PALLETISER.config();
    cfg.set_text("effects.gearbox_mesh_friction = false").unwrap();
    let mut sim = Sim::new(&PALLETISER, &cfg).unwrap();
    let axis = sim.machine.axes()[0].clone();
    let (start, mut end) = (sim.control.set_pos.clone(), sim.control.set_pos.clone());
    end[0] = axis.min + 0.9 * (axis.max - axis.min);
    let held = sim.machine.resonance_hz(&end).unwrap();
    let at_start = sim.machine.resonance_hz(&start).unwrap();
    assert!(
        (held - 5.95).abs() < 0.02 && (at_start - 8.21).abs() < 0.02,
        "{at_start} Hz at the start, {held} at the end"
    );
    sim.command(Command::MoveTo(end)).unwrap();
    while sim.control.busy() || sim.step == 0 {
        sim.step();
    }
    // The free ring, from 0.5 s after the move (when the move's own transient has passed) over 3 s.
    sim.run(0.5, |_| {});
    let n = sim.machine.axes().len();
    let (mut last, mut crossings) = (0.0, Vec::new());
    sim.run(3.0, |s| {
        let (mut p, mut v) = (vec![0.0; n], vec![0.0; n]);
        s.machine.tool_state(&s.state, &mut p, &mut v);
        if last * v[1] < 0.0 {
            crossings.push(s.time());
        }
        last = v[1];
    });
    let ring = (crossings.len() - 1) as f64 / (2.0 * (crossings[crossings.len() - 1] - crossings[0]));
    // The turret's motor, drive and gear as resolved (its named motor's rotor, the gear's inertia at its input).
    let resolved = sim_components::catalog::resolve(&PALLETISER, &cfg).unwrap();
    let own = sim_components::catalog::motor_config(&resolved, "1").unwrap();
    let (kp, tn, kv) = (
        own.f64("drive.velocity_kp_nm_s_rad").unwrap(),
        own.f64("drive.velocity_tn_s").unwrap(),
        own.f64("drive.kv_1_s").unwrap(),
    );
    let rotor = own.f64("motor.rotor_inertia_kgm2").unwrap() + own.f64("gearbox.inertia_at_input_kgm2").unwrap();
    let (ratio, k) = (own.f64("gearbox.ratio").unwrap(), own.f64("gearbox.stiffness_nm_rad").unwrap());
    let mut f = held;
    for _ in 0..20 {
        let w = std::f64::consts::TAU * f;
        // (1 - j / (w Tn)) (Kv + j w), times Kp, less w^2 J: (re, im).
        let (a, b) = (1.0, -1.0 / (w * tn));
        let (re, im) = (kp * (a * kv - b * w) - w * w * rotor, kp * (a * w + b * kv));
        let (zr, zi) = (ratio * ratio * re, ratio * ratio * im);
        // k z / (k + z) for complex z.
        let (dr, di) = (k + zr, zi);
        let d2 = dr * dr + di * di;
        let series_re = k * (zr * dr + zi * di) / d2;
        f = held * (series_re / k).sqrt();
    }
    assert!(ring < held, "the ring {ring} Hz above the held mode {held} Hz");
    assert!((f / ring - 1.0).abs() < 0.02, "the loop's estimate {f} Hz against the ring {ring} Hz (held {held} Hz)");
}

// ---------------------------------------------------------------------------------------------
// The arm on its track (`arm-on-track`): the track a joint of the chain, its rack, its guide blocks.

/// The track's ratio, motor radians per metre: the gearbox's 10 over the pinion's pitch radius, 2 mm 30 teeth /
/// (2 cos 19.5283 deg) = 31.831 mm (200 mm a turn).
fn track_ratio() -> f64 {
    10.0 / (0.002 * 30.0 / (2.0 * 19.5283f64.to_radians().cos()))
}

/// From urdf/arm-on-track.urdf with the arm turned to stretch along +y (joint 2 at pi/2, the others at 0): the moving
/// masses' distances from the track's line, y, the arm's x of urdf/six-axis.urdf at zero: link2 12 kg at 0.15 m, link3
/// 7 kg at 0.23, link4 4 kg at 0.60, link5 2 kg at 0.83, link6 1.5 kg at 0.89 and the 6 kg payload 0.05 m beyond the
/// flange at 0.95; link1 and the carriage on the line. Sum m y = 1.8 + 1.61 + 2.4 + 1.66 + 1.335 + 5.7 = 14.505 kg m.
const SUM_MY: f64 = 14.505;
/// Their heights above the carriage's frame (the guide blocks' plane): the carriage 80 kg at 0.1, link1 18 kg at 0.70,
/// link2 at 1.15, link3 at 1.53, links 4 to 6 and the payload (13.5 kg) at 1.57: sum m z = 8 + 12.6 + 13.8 + 10.71 +
/// 21.195 = 66.305 kg m. All of it 80 + 18 + 12 + 7 + 4 + 2 + 1.5 + 6 = 130.5 kg.
const SUM_MZ: f64 = 66.305;
const TRACK_MASS: f64 = 130.5;

fn sideways() -> [f64; 7] {
    [1.0, std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0, 0.0]
}

/// The coupling, by hand, from the rigid model's torques at rest (no speed terms):
/// - joint 2 (the arm's base) turns at 3 rad/s^2 with the arm stretched across the track: each mass moves along the
///   track by -3 y (alpha z x y = -alpha y x), so the track's joint must push -3 * 14.505 = -43.515 N (motor 1: that over
///   the ratio); nothing pushes along the track with joint 2 still;
/// - the track accelerates at 2 m/s^2 under the same arm: joint 2 holds the masses beyond it against turning, the
///   moment of m a x about the vertical, -2 * 14.505 = -29.01 N m (motor 2: over its 120), and the track pushes all
///   130.5 kg by 261 N.
#[test]
fn the_track_and_the_arm_act_on_each_other_as_by_hand() {
    let m = (ARM_ON_TRACK.build)(&rigid(&ARM_ON_TRACK)).unwrap();
    let mut torque = [0.0; 7];
    let mut acc = [0.0; 7];
    acc[1] = 3.0;
    m.feedforward_torque(&sideways(), &acc, &mut torque);
    near(torque[0] * track_ratio(), -3.0 * SUM_MY, 1e-6, "track force while joint 2 swings");
    acc = [0.0; 7];
    m.feedforward_torque(&sideways(), &acc, &mut torque);
    assert!(torque[0].abs() < 1e-9, "the arm at rest pushes on the track by {}", torque[0] * track_ratio());
    acc[0] = 2.0;
    m.feedforward_torque(&sideways(), &acc, &mut torque);
    // Motor 1 turns too: its rotor, gearbox and pinion (1.7 + 0.359 + 1 / 100 kg cm^2) at 2 * track_ratio() rad/s^2.
    let rotor = (1.7e-4 + 3.59e-5 + 1.0e-4 / 100.0) * 2.0 * track_ratio();
    near((torque[0] - rotor) * track_ratio(), TRACK_MASS * 2.0, 1e-6, "track force");
    near(torque[1] * 120.0, -2.0 * SUM_MY, 1e-6, "joint 2 while the track accelerates");
}

fn track_signal(m: &dyn Machine, pose: &[f64], acc: &[f64], id: &str) -> f64 {
    let (mut state, mut torque) = (Vec::new(), vec![0.0; pose.len()]);
    m.equilibrium(pose, &mut state, &mut torque);
    m.feedforward_torque(pose, acc, &mut torque);
    let mut values = Vec::new();
    m.signals(&state, &torque, &mut values);
    values[m.signal_infos().iter().position(|s| s.id == id).unwrap_or_else(|| panic!("no {id}"))]
}

/// The guide blocks by hand, four of them 0.4 m apart along the rails and 0.5 m across, the arm stretched across the
/// track with its payload. At rest they carry the weight, 130.5 * 9.80665 = 1279.768 N, a quarter each, and its moment
/// about the track's line, M_x = 9.80665 * 14.505 = 142.2455 N m, as +-M_x / (2 * 0.5) on the two rails: the most
/// loaded block 319.942 + 142.2455 = 462.188 N, reported times four, 1848.75 N. The track accelerating at 2 m/s^2 adds
/// the inertia's moments about the carriage: pitch M_y = 2 * 66.305 = 132.61 N m on the blocks front and back, +-M_y /
/// (2 * 0.4), and yaw M_z = -2 * 14.505 = -29.01 N m across the rails, +-M_z / (2 * 0.4); the push along the rails is
/// the rack's: 462.188 + 165.7625 + 36.2625 = 664.213 N, times four 2656.85 N, and the pinion's separating force on top (below). The rack's teeth then carry the 261 N,
/// and the pinion loads the gearbox's shaft with 261 sqrt(1 + (tan 20 deg / cos 19.5283 deg)^2) = 261 * 1.071978 =
/// 279.786 N.
#[test]
fn the_guide_blocks_carry_the_weight_and_the_tilting_moments_by_hand() {
    let m = (ARM_ON_TRACK.build)(&rigid(&ARM_ON_TRACK)).unwrap();
    let g = 9.80665;
    let block = |extra: f64| 4.0 * (TRACK_MASS * g / 4.0 + g * SUM_MY / (2.0 * 0.5) + extra);
    let at_rest = track_signal(m.as_ref(), &sideways(), &[0.0; 7], "axis.j1.guide_load");
    assert!((block(0.0) - 1848.75).abs() < 0.01);
    near(at_rest, block(0.0), 1e-7, "guide load at rest");
    let mut acc = [0.0; 7];
    acc[0] = 2.0;
    let pushed = track_signal(m.as_ref(), &sideways(), &acc, "axis.j1.guide_load");
    // The helical pinion's separating force across the rails, 261 N * tan 20 deg / cos 19.5283 deg = 261 * 0.386 = 100.8 N,
    // on the four blocks: the total grows by it (F_y / 4 on each, all four on the loaded side).
    let apart = TRACK_MASS * 2.0 * (1.071978f64.powi(2) - 1.0).sqrt();
    assert!((apart - 100.8).abs() < 0.1, "{apart}");
    // The carriage's rotors spin about +z (the rack on +y: the pinion's teeth there stand still, w = v / r): the motor's
    // and the gearbox's input at r a = 2 x 314.159 rad/s^2, the pinion at a tenth of it, 1.7e-4 + 3.59e-5 + 1e-4 / 10 =
    // 2.159e-4 kg m^2 of momentum per motor rad/s. Its rate, 0.135654 N m about +z, is part of the yaw moment the base
    // passes, M_z = -2 SUM_MY + 0.135654: the most loaded block's yaw term falls by 0.135654 / 0.8 (4 x that = 0.678 N).
    let gyro = (1.7e-4 + 3.59e-5 + 1.0e-4 / 10.0) * track_ratio() * 2.0;
    let by_hand = block(2.0 * SUM_MZ / (2.0 * 0.4) + (2.0 * SUM_MY - gyro) / (2.0 * 0.4) + apart / 4.0);
    assert!((by_hand - 2756.965).abs() < 0.01, "{by_hand}");
    near(pushed, by_hand, 1e-6, "guide load accelerating");
    let teeth = track_signal(m.as_ref(), &sideways(), &acc, "rack.1.feed_force");
    near(teeth, TRACK_MASS * 2.0, 1e-6, "feed force");
    let radial = track_signal(m.as_ref(), &sideways(), &acc, "rack.1.radial_load");
    let apart = 20f64.to_radians().tan() / 19.5283f64.to_radians().cos();
    assert!(((1.0 + apart * apart).sqrt() - 1.071978).abs() < 1e-6);
    near(radial, teeth * 1.071978, 1e-6, "radial load");
}

/// The rack's side and lever (`track.rack_side`, `track.rack_lever_m`) at the track's 2 m/s^2 above, F_x = 261 N, the
/// separating force F_sep = 261 * 0.386 = 100.8 N: the blocks' F_y is the chain's plus side * F_sep, and the yaw moment
/// M_z = -2 * 14.505 + side * lever * 261 N m. The yaw term of the most loaded block, which the pitch moment keeps at
/// s_x = -1, is |F_y / 4 - M_z / (2 * 0.4)|: 25.2 + 36.2625 = 61.4625 N at side +1 and no lever (as shipped), 25.2 + 3.6375
/// at lever 0.1 m (M_z = -2.91 N m), |-25.2 + 36.2625| = 11.0625 N at side -1, 25.2 + 68.8875 at side -1 and lever 0.1 m;
/// and in M_z the rotors' momentum rate, side * 0.135654 N m (`the_guide_blocks_carry_the_weight_...`).
#[test]
fn the_racks_side_and_lever_move_the_guide_blocks_load_as_by_hand() {
    let g = 9.80665;
    let push = TRACK_MASS * 2.0;
    let apart = push * (1.071978f64.powi(2) - 1.0).sqrt();
    let by_hand = |side: f64, lever: f64| {
        // The rotors' momentum rate (above), about +z with the rack on +y and about -z with it on -y.
        let gyro = (1.7e-4 + 3.59e-5 + 1.0e-4 / 10.0) * track_ratio() * 2.0;
        let yaw = -2.0 * SUM_MY + side * lever * push + side * gyro;
        4.0 * (TRACK_MASS * g / 4.0 + g * SUM_MY + 2.0 * SUM_MZ / 0.8 + (side * apart / 4.0 - yaw / 0.8).abs())
    };
    let mut acc = [0.0; 7];
    acc[0] = 2.0;
    for (side, lever) in [(1.0, 0.0), (1.0, 0.1), (-1.0, 0.0), (-1.0, 0.1)] {
        let mut cfg = rigid(&ARM_ON_TRACK);
        cfg.set_text(&format!("track.rack_side = {side}")).unwrap();
        cfg.set_text(&format!("track.rack_lever_m = {lever}")).unwrap();
        let m = (ARM_ON_TRACK.build)(&cfg).unwrap();
        let load = track_signal(m.as_ref(), &sideways(), &acc, "axis.j1.guide_load");
        near(load, by_hand(side, lever), 1e-6, &format!("guide load, side {side}, lever {lever}"));
    }
}

/// The flange's angular speed and acceleration, not the rotation vector's rates. The six-axis arm with joint 1 about z
/// and joints 2 and 3 about y (all frames unrotated), at q1 = 0: w = q1' z + s y with s = q2' + q3', and dw/dt =
/// q1'' z + s' y + s q1' (z x y) = (-s q1', s', q1''). With q' = (0.5, 0.3, 0.4) and q'' = (2, 1, 2): s = 0.7, s' = 3,
/// w = (0, 0.7, 0.5), |w| = 0.86023, dw/dt = (-0.35, 3, 2), |dw/dt| = 3.62244.
#[test]
fn the_flanges_angular_speed_and_acceleration_follow_from_the_joint_axes() {
    let cfg = tool_config(&URDF_ARM, ARM_TOOL_CONFIG);
    let chain = arm::chain(&cfg).unwrap();
    let fk = |q: &[f64]| chain.tip(q);
    let home = cfg.f64s("kinematics.home_joints").unwrap();
    let pose = tool::Pose::new(&fk, home, vec![0, 1, 2, 3, 4, 5], 0.86, 30.0).unwrap();
    let q = [0.0, 0.2, -0.3, 0.0, 0.0, 0.0];
    let (w, a) = pose.angular_motion(&fk, &q, &[0.5, 0.3, 0.4, 0.0, 0.0, 0.0], &[2.0, 1.0, 2.0, 0.0, 0.0, 0.0]);
    near(w, 0.74f64.sqrt(), 1e-6, "angular speed");
    near(a, 13.1225f64.sqrt(), 1e-4, "angular acceleration");
}

/// A time-optimal line that turns the six-axis arm's flange 0.5 rad about the vertical and tilts it 0.2 rad (0.538 rad
/// in all, the rotation vector staying in the tool range of +-0.6 rad) while it moves 0.05 m: along the plan, the flange's spin from its planned orientations, log(R(t + dt) R(t - dt)^T) / 2 dt, and
/// its rate are what `Pose::angular_motion` gives from the joints (their rates differenced over the plan's 1 ms), and
/// the plan keeps both within the smallest rx, ry, rz limits (1.5 rad/s, 8 rad/s^2), which bind: 1.4997 rad/s and
/// 7.7354 rad/s^2 (a limit of the plan itself, judged on the planned motion, so no torque reserve: 6.3987, 0.8 of 8,
/// before 17 November; less what the controller's cubics may add between the plan's points, `path::piece_limits`), the
/// spin to 3e-7 rad/s, its rate to 0.01 rad/s^2.
#[test]
fn a_reorienting_time_optimal_line_keeps_the_flanges_angular_limits() {
    use sim_components::rigid::{exp, log, matmul, transpose};
    let mut cfg = tool_config(&URDF_ARM, ARM_TOOL_CONFIG);
    cfg.set_text("effects.joint_elasticity = false").unwrap();
    let sim = Sim::new(&URDF_ARM, &cfg).unwrap();
    let from = sim.control.planned_pos().to_vec();
    let b = log(&matmul(&exp([0.0, 0.0, 0.5]), &matmul(&exp([0.2, 0.0, 0.0]), &exp([from[3], from[4], from[5]]))));
    let text = format!("line, {}, {}, {}, {}, {}, {}", from[0], from[1] + 0.05, from[2], b[0], b[1], b[2]);
    let points = sim.plan_path(&sim_components::path::parse(&text).unwrap(), true).unwrap();
    let chain = arm::chain(&cfg).unwrap();
    let fk = |q: &[f64]| chain.tip(q);
    let pose =
        tool::Pose::new(&fk, cfg.f64s("kinematics.home_joints").unwrap(), vec![0, 1, 2, 3, 4, 5], 0.86, 30.0).unwrap();
    let ratios = cfg.f64s("arm.gear_ratios").unwrap();
    let joints: Vec<Vec<f64>> = points
        .iter()
        .map(|(_, p)| {
            let mut m = vec![0.0; 6];
            sim.machine.inverse(p, &mut m);
            m.iter().zip(&ratios).map(|(m, r)| m / r).collect()
        })
        .collect();
    let spin = |i: usize, j: usize| {
        let (ri, rj) = (
            exp([points[i].1[3], points[i].1[4], points[i].1[5]]),
            exp([points[j].1[3], points[j].1[4], points[j].1[5]]),
        );
        log(&matmul(&rj, &transpose(&ri))).map(|w| w / (points[j].0 - points[i].0))
    };
    let (mut fastest, mut hardest, mut worst) = (0.0f64, 0.0f64, [0.0f64; 2]);
    for k in 2..points.len() - 2 {
        let dt = points[k + 1].0 - points[k].0;
        let w = spin(k - 1, k + 1);
        let (w_ahead, w_behind) = (spin(k, k + 2), spin(k - 2, k));
        let dw: Vec<f64> = (0..3).map(|i| (w_ahead[i] - w_behind[i]) / (2.0 * dt)).collect();
        let qd: Vec<f64> = (0..6).map(|c| (joints[k + 1][c] - joints[k - 1][c]) / (2.0 * dt)).collect();
        let qdd: Vec<f64> =
            (0..6).map(|c| (joints[k + 1][c] - 2.0 * joints[k][c] + joints[k - 1][c]) / (dt * dt)).collect();
        let (speed, acc) = pose.angular_motion(&fk, &joints[k], &qd, &qdd);
        let (w_size, dw_size) =
            (w.iter().map(|v| v * v).sum::<f64>().sqrt(), dw.iter().map(|v| v * v).sum::<f64>().sqrt());
        worst = [worst[0].max((speed - w_size).abs()), worst[1].max((acc - dw_size).abs())];
        (fastest, hardest) = (fastest.max(w_size), hardest.max(dw_size));
    }
    println!(
        "{} points: |w| {fastest:.4} of 1.5 rad/s, |dw/dt| {hardest:.4} of 8 rad/s^2, differences {worst:?}",
        points.len()
    );
    assert!(worst[0] < 1e-5 * fastest && worst[1] < 5e-3 * hardest, "{worst:?}");
    assert!(fastest <= 1.5 * 1.001 && hardest <= 8.0 * 1.001, "{fastest} rad/s, {hardest} rad/s^2");
    assert!(fastest > 0.9 * 1.5 || hardest > 0.9 * 8.0, "the limits bind: {fastest} rad/s, {hardest} rad/s^2");
    // The palletiser's quarter turn across to the pallet (its cycle's third segment), time-optimal: its flange turns
    // about the vertical alone, so the spin is rz' and its rate rz'', within 2.5 rad/s and 10 rad/s^2.
    let mut cfg = PALLETISER.config();
    cfg.set_text("effects.joint_elasticity = false").unwrap();
    let sim = Sim::new(&PALLETISER, &cfg).unwrap();
    let text = "line, 1.6, 0.5, 1.3, 1.5707963267948966";
    let points = sim.plan_path(&sim_components::path::parse(text).unwrap(), true).unwrap();
    let rate = |i: usize, j: usize| (points[j].1[3] - points[i].1[3]) / (points[j].0 - points[i].0);
    let (mut fastest, mut hardest) = (0.0f64, 0.0f64);
    for k in 2..points.len() - 2 {
        let dt = 0.5 * (points[k + 1].0 - points[k - 1].0);
        fastest = fastest.max(rate(k - 1, k + 1).abs());
        hardest = hardest.max(((rate(k, k + 2) - rate(k - 2, k)) / (2.0 * dt)).abs());
    }
    println!("palletiser: |w| {fastest:.4} of 2.5 rad/s, |dw/dt| {hardest:.4} of 10 rad/s^2");
    assert!(fastest <= 2.5 * 1.001 && hardest <= 10.0 * 1.001, "{fastest} rad/s, {hardest} rad/s^2");
}

/// A path planned within the axis limits on the arm on a track in tool space (track, x, y, z, rx, ry, rz): a line that
/// moves the track 0.1 m and the flange 0.05 m along y and its rotation vector from (0, pi/2, 0) to (0.5, pi/2, 0.5).
/// Its orientation turns by slerp, about one axis (the turn from the start stays along the whole turn's axis, to 6e-16),
/// and the flange's spin from the planned orientations, log(R(t + dt) R(t - dt)^T) / 2 dt, and its rate keep the
/// flange's limits (1.5 rad/s, 8 rad/s^2) and reach them (1.5000, 8.0000: about a fixed axis the spin is the angle's
/// rate). Before 17 November only an arm with exactly x, y, z, rx, ry, rz slerped and kept them; the track arm's rotation
/// vector ran straight within its axes' limits alone. The same line timed by the axis limits without the flange's
/// (`path::plan`) spins it at 1.91 rad/s, in 0.575 s against 0.654 s.
#[test]
fn a_reorienting_line_on_the_track_arm_keeps_the_flanges_angular_limits() {
    use sim_components::rigid::{cross, exp, log, matmul, transpose};
    let mut cfg = tool_config(&ARM_ON_TRACK, ARM_ON_TRACK_TOOL_CONFIG);
    cfg.set_text("effects.joint_elasticity = false").unwrap();
    let sim = Sim::new(&ARM_ON_TRACK, &cfg).unwrap();
    let from = sim.control.planned_pos().to_vec();
    let to = vec![from[0] + 0.1, from[1], from[2] + 0.05, from[3], from[4] + 0.5, from[5], from[6] + 0.5];
    let pieces = [sim_components::path::Piece::Line(to.clone())];
    let rotation = |p: &[f64]| exp([p[4], p[5], p[6]]);
    let whole = log(&matmul(&rotation(&to), &transpose(&rotation(&from))));
    let size = |v: [f64; 3]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
    let measured = |points: &[(f64, Vec<f64>)]| {
        let spin = |i: usize, j: usize| {
            log(&matmul(&rotation(&points[j].1), &transpose(&rotation(&points[i].1))))
                .map(|w| w / (points[j].0 - points[i].0))
        };
        let (mut fastest, mut hardest, mut off_axis) = (0.0f64, 0.0f64, 0.0f64);
        for k in 2..points.len() - 2 {
            let dt = points[k + 1].0 - points[k].0;
            let (w_ahead, w_behind) = (spin(k, k + 2), spin(k - 2, k));
            fastest = fastest.max(size(spin(k - 1, k + 1)));
            hardest = hardest.max(size(std::array::from_fn(|i| (w_ahead[i] - w_behind[i]) / (2.0 * dt))));
            let turned = log(&matmul(&rotation(&points[k].1), &transpose(&rotation(&from))));
            off_axis = off_axis.max(size(cross(turned, whole)) / size(whole));
        }
        (fastest, hardest, off_axis)
    };
    let planned = sim.plan_path(&pieces, false).unwrap();
    let (fastest, hardest, off_axis) = measured(&planned);
    let (cycle, tolerance) = (cfg.f64("profile.cycle_s").unwrap(), cfg.f64("profile.path_tolerance_m").unwrap());
    let axes = sim.machine.axes();
    let free =
        sim_components::path::plan(axes, &from, &pieces, cycle, tolerance, sim.machine.setpoint_slack()).unwrap();
    let (free_fastest, ..) = measured(&free);
    println!(
        "track arm: {} s, |w| {fastest:.4} of 1.5 rad/s, |dw/dt| {hardest:.4} of 8 rad/s^2, off the axis {off_axis:.2e} rad; \
         without the flange's limits {} s, |w| {free_fastest:.4} rad/s",
        planned.last().unwrap().0,
        free.last().unwrap().0
    );
    assert!(off_axis < 1e-9, "the turn leaves its axis by {off_axis} rad");
    assert!(fastest <= 1.5 * 1.001 && hardest <= 8.0 * 1.001, "{fastest} rad/s, {hardest} rad/s^2");
    assert!(fastest > 0.99 * 1.5 || hardest > 0.99 * 8.0, "the limits bind: {fastest} rad/s, {hardest} rad/s^2");
    assert!(free_fastest > 1.5 * 1.01, "without the flange's limits: {free_fastest} rad/s");
}

/// A pure reorientation timed by the axis limits (`cycle_timing = "axis_limits"`): at the six-axis arm's home, flange
/// down (r = (0, pi/2, 0)), 0.5 rad more on rz. The move keeps the flange's own limits too (`MotionLaw::spin`), so its
/// spin from the setpoint, |log(exp(r + h r') exp(r - h r')^T)| / 2h, stays within 1.5 rad/s and its rate within 8
/// rad/s^2. Here rz's own limit of 1.5 rad/s binds: across the rotation vector the flange turns at 2 sqrt(2) / pi = 0.9003
/// of its rate, 1.3505 rad/s (0.9012 at the move's end, 1.3518; `profile::spin_rates`); turns the flange's limits bind
/// use them in full (`profile::tests::the_flanges_spin_on_a_move_uses_its_limits`). A move of the place alone takes as
/// long as before.
#[test]
fn a_reorientation_timed_by_the_axis_limits_keeps_the_flanges_angular_limits() {
    use sim_components::rigid::{exp, log, matmul, transpose};
    let cfg = tool_config(&URDF_ARM, ARM_TOOL_CONFIG);
    let spin = |r: [f64; 3], rate: [f64; 3]| -> [f64; 3] {
        let h = 1e-6;
        let at = |s: f64| exp(std::array::from_fn(|i| r[i] + s * rate[i]));
        log(&matmul(&at(h), &transpose(&at(-h)))).map(|w| w / (2.0 * h))
    };
    let size = |v: [f64; 3]| v.iter().map(|x| x * x).sum::<f64>().sqrt();
    let run = |target: Vec<f64>| {
        let mut sim = Sim::new(&URDF_ARM, &cfg).unwrap();
        sim.command(sim_components::Command::MoveTo(target)).unwrap();
        // The setpoint at each profile cycle (it holds between them): time and the flange's angular velocity.
        let mut samples: Vec<(f64, [f64; 3])> = Vec::new();
        let (mut last, mut end) = (Vec::new(), 0.0);
        sim.run(2.0, |s| {
            let c = &s.control;
            if c.set_pos != last {
                let w = spin([c.set_pos[3], c.set_pos[4], c.set_pos[5]], [c.set_vel[3], c.set_vel[4], c.set_vel[5]]);
                samples.push((s.time(), w));
                (last, end) = (c.set_pos.clone(), s.time());
            }
        });
        let fastest = samples.iter().map(|(_, w)| size(*w)).fold(0.0, f64::max);
        let hardest = samples
            .windows(3)
            .map(|p| size(std::array::from_fn(|i| (p[2].1[i] - p[0].1[i]) / (p[2].0 - p[0].0))))
            .fold(0.0, f64::max);
        (fastest, hardest, end)
    };
    let start = cfg.f64s("start.pose").unwrap();
    let mut turned = start.clone();
    turned[5] += 0.5;
    let (fastest, hardest, _) = run(turned);
    println!("pure reorientation: |w| {fastest:.4} of 1.5 rad/s, |dw/dt| {hardest:.4} of 8 rad/s^2");
    assert!(fastest > 1.3505 * (1.0 - 1e-3) && fastest < 1.3518 * (1.0 + 1e-3), "{fastest} rad/s");
    assert!(hardest <= 8.0 * 1.01, "{hardest} rad/s^2");
    // The place alone: the same timing with and without the flange's limits (no turn, nothing to keep).
    let mut moved = start.clone();
    moved[1] += 0.2;
    let axes = (URDF_ARM.build)(&cfg).unwrap().axes().to_vec();
    let law = |spin| sim_components::profile::MotionLaw { spin, ..Default::default() };
    let duration = |spin| sim_components::profile::Move::with_law(&start, &moved, &axes, &law(spin)).duration();
    assert_eq!(duration(Some([1.5, 8.0])), duration(None));
}

/// The verdicts "tool-angular-speed" and "tool-angular-acceleration" judge the flange's spin on the setpoint's rigid
/// motion (`Machine::rigid_checks` along the plan, once per millisecond), not on the simulated flange, whose gears ring
/// beyond any plan. The palletiser's shipped cycle turns the flange a quarter turn about the vertical within its rz
/// limits (2.5 rad/s, 10 rad/s^2), and passes; on the six-axis arm a trajectory that turns it 0.5 rad about the
/// vertical in 0.25 s (2 rad/s on average) is planned faster than its 1.5 rad/s, and fails.
#[test]
fn the_flanges_angular_limits_are_judged_on_the_planned_motion() {
    use sim_components::analysis::{analyse_cycle_with, analyse_trajectory, verdicts};
    let angular = |cfg: &Config, load| -> Vec<sim_components::analysis::Verdict> {
        verdicts(cfg, load).unwrap().into_iter().filter(|v| v.check.starts_with("tool-angular")).collect()
    };
    let cfg = PALLETISER.config();
    let load = analyse_cycle_with(&mut Sim::new(&PALLETISER, &cfg).unwrap(), false).unwrap();
    let judged = angular(&cfg, &load);
    println!("palletiser: {judged:?}");
    assert!(judged.len() == 2 && judged.iter().all(|v| v.ok && v.value > 0.5 * v.limit), "{judged:?}");
    let cfg = tool_config(&URDF_ARM, ARM_TOOL_CONFIG);
    let start = cfg.f64s("start.pose").unwrap();
    let mut turned = start.clone();
    turned[5] += 0.5;
    let load = analyse_trajectory(&mut Sim::new(&URDF_ARM, &cfg).unwrap(), vec![(0.0, start), (0.25, turned)]).unwrap();
    let judged = angular(&cfg, &load);
    println!("six-axis arm, too fast: {judged:?}");
    let speed = judged.iter().find(|v| v.check == "tool-angular-speed").unwrap();
    assert!(!speed.ok && speed.value > 2.0 && speed.limit == 1.5, "{speed:?}");
    // The five-axis arm: what is judged is the turn of where the tool points, which its rx, ry stand for; the turn about
    // it is the joints'. Its cycle's tilt of 0.3 rad about x runs at the rx limits (1.5 rad/s, 8 rad/s^2): the plan
    // reaches 8.000 rad/s^2 and 1.350 rad/s (the S-curve's top speed on 0.3 rad). Judged whole, the flange's spin about
    // where it points as the arm swings across (17 November 2026: 2.86 rad/s, 23.4 rad/s^2) failed both.
    let cfg = tool_config(&URDF_ARM, ARM5_TOOL_CONFIG);
    let load = analyse_cycle_with(&mut Sim::new(&URDF_ARM, &cfg).unwrap(), false).unwrap();
    let judged = angular(&cfg, &load);
    println!("five-axis arm: {judged:?}");
    let value = |check: &str| judged.iter().find(|v| v.check == check).unwrap();
    let (speed, acceleration) = (value("tool-angular-speed"), value("tool-angular-acceleration"));
    assert!(speed.ok && (speed.value - 1.350).abs() < 1e-3, "{speed:?}");
    assert!(acceleration.ok && (acceleration.value - 8.0).abs() < 1e-3, "{acceleration:?}");
}

/// The five-axis arm's rx, ry are where the tool points: the direction d = R R_0^T e_z, which is e_z at zero joints, is
/// exp(rx, ry, 0) e_z at every pose, whatever the arm's turn about d (by hand: the shortest turn from e_z to d has its
/// axis e_z x d in the x-y plane). Over the grid of `a_flange_pointing_down_at_zero_is_at_no_turn`, to 1e-9.
#[test]
fn the_five_axis_arms_turns_are_where_the_tool_points() {
    use sim_components::rigid::{exp, matmul, transpose};
    let cfg = tool_config(&URDF_ARM, ARM5_TOOL_CONFIG);
    let m = (URDF_ARM.build)(&cfg).unwrap();
    let chain = arm::chain(&cfg).unwrap();
    let ratios = cfg.f64s("arm.gear_ratios").unwrap();
    let zero = chain.tip(&[0.0; 5]).0;
    let mut motors = [0.0; 5];
    for k in 0..27 {
        let f = |i: usize| ((k / 3usize.pow(i as u32)) % 3) as f64 - 1.0;
        let pose = [0.9 + 0.1 * f(0), 0.2 * f(1), 0.7 + 0.1 * f(2), 0.2 * f(0) * f(1), 0.2 * f(2)];
        m.inverse(&pose, &mut motors);
        let joints: Vec<f64> = motors.iter().zip(&ratios).map(|(m, r)| m / r).collect();
        let turned = matmul(&chain.tip(&joints).0, &transpose(&zero));
        let want = exp([pose[3], pose[4], 0.0]);
        assert!((0..3).all(|i| (turned[i][2] - want[i][2]).abs() < 1e-9), "{pose:?}");
    }
}

/// The seven-joint arm resolves its spare joint by a walk from the last solution, settled to 1e-13, so the feedforward
/// at a pose (its joint rates by differences of the inverse over 1e-3 of the tool's units) is the same whatever the
/// inverse solved before: with `kinematics.redundancy = "joint_limits"` the motors' torques after four different
/// earlier calls agree to 1e-3 (they spread by 3.7 % on motor 1 before the tight settle). The step's truncation: a
/// quarter of it changes them by less than 1e-3 too. And the inverse converges (round trip to 1e-9) over a box of
/// +-0.15 m and +-0.3 rad about the start.
#[test]
fn the_seven_joint_arms_second_difference_does_not_depend_on_the_call_before() {
    let mut cfg = tool_config(&URDF_ARM, ARM7_TOOL_CONFIG);
    cfg.set_text("kinematics.redundancy = \"joint_limits\"").unwrap();
    let m = (URDF_ARM.build)(&cfg).unwrap();
    let start = cfg.f64s("start.pose").unwrap();
    let at = |d: [f64; 6]| -> Vec<f64> { start.iter().zip(d).map(|(s, d)| s + d).collect() };
    let pose = at([0.05, 0.1, -0.08, 0.1, -0.2, 0.15]);
    let (vel, acc) = ([0.4, -0.3, 0.2, 0.3, 0.2, -0.4], [3.0, 2.0, -2.5, 2.0, -1.5, 3.0]);
    let ff = |m: &dyn Machine| {
        let mut t = vec![0.0; 7];
        m.feedforward_torque_moving(&pose, &vel, &acc, &mut t);
        t
    };
    let mut torques = Vec::new();
    for before in
        [start.clone(), at([0.15, -0.15, 0.1, -0.3, 0.3, 0.0]), at([-0.15, 0.15, -0.15, 0.3, 0.0, -0.3]), pose.clone()]
    {
        let mut motors = [0.0; 7];
        m.inverse(&before, &mut motors);
        torques.push(ff(m.as_ref()));
    }
    let size = torques[0].iter().map(|t| t.abs()).fold(0.0, f64::max);
    for t in &torques[1..] {
        for k in 0..7 {
            assert!((t[k] - torques[0][k]).abs() < 1e-3 * size, "motor {k}: {} against {} N m", t[k], torques[0][k]);
        }
    }
    println!("motor 1 after four calls: {:?} N m", torques.iter().map(|t| t[0]).collect::<Vec<_>>());
    // The reference with a quarter of the step: the motors' angles differenced by hand.
    let ratios = cfg.f64s("arm.gear_ratios").unwrap();
    let motors_at = |x: &[f64]| {
        let mut q = vec![0.0; 7];
        m.inverse(x, &mut q);
        q
    };
    let along = |d: &[f64], s: f64| -> Vec<f64> { pose.iter().zip(d).map(|(p, d)| p + s * d).collect() };
    let (hv, ha) = (
        0.25e-3 / vel.iter().map(|v| v * v).sum::<f64>().sqrt(),
        0.25e-3 / acc.iter().map(|v| v * v).sum::<f64>().sqrt(),
    );
    let (q, ahead, behind) = (motors_at(&pose), motors_at(&along(&vel, hv)), motors_at(&along(&vel, -hv)));
    let (a_ahead, a_behind) = (motors_at(&along(&acc, ha)), motors_at(&along(&acc, -ha)));
    let speed: Vec<f64> = (0..7).map(|k| (ahead[k] - behind[k]) / (2.0 * hv)).collect();
    let accel: Vec<f64> = (0..7)
        .map(|k| (ahead[k] - 2.0 * q[k] + behind[k]) / (hv * hv) + (a_ahead[k] - a_behind[k]) / (2.0 * ha))
        .collect();
    let mut fine = vec![0.0; 7];
    m.rigid_motor_torque(&q, &speed, &accel, &mut fine);
    let coarse = ff(m.as_ref());
    for k in 0..7 {
        assert!(
            (coarse[k] - fine[k]).abs() < 1e-3 * size,
            "motor {k}: {} against {} N m with a quarter step",
            coarse[k],
            fine[k]
        );
    }
    println!("step 1e-3 against a quarter: {:?}", (0..7).map(|k| (coarse[k] - fine[k]) / size).collect::<Vec<_>>());
    assert_eq!(ratios.len(), 7);
    // Convergence over the box: every corner and the middles of its faces round trip.
    for i in 0..6 {
        for s in [-1.0, 1.0] {
            let mut d = [0.0; 6];
            d[i] = s * if i < 3 { 0.15 } else { 0.3 };
            let x = at(d);
            let (q, mut back) = (motors_at(&x), [0.0; 6]);
            assert!(q.iter().all(|v| v.is_finite()), "{x:?} not solved");
            m.forward(&q, &mut back);
            assert!(back.iter().zip(&x).all(|(a, b)| (a - b).abs() < 1e-9), "{x:?} came back as {back:?}");
        }
    }
}

/// The guide verdicts judge that load: a set rated just below the cycle's highest guide load fails "guide-load", just
/// above passes; and a rated life just above what the blocks give fails "guide-life".
#[test]
fn the_guide_verdicts_judge_the_track_at_their_limits() {
    let cfg = ARM_ON_TRACK.config();
    let (load, verdicts) = judged(&ARM_ON_TRACK, &cfg);
    let peak = load.guide_load.unwrap().0;
    assert!(peak > 1800.0, "{peak} N: at least the weight and the moment of the stretched arm");
    let life = value(&verdicts, "guide-life");
    for (key, limit, check, fails) in [
        ("ratings.guide_max_load_n", 0.999 * peak, "guide-load", true),
        ("ratings.guide_max_load_n", 1.001 * peak, "guide-load", false),
        ("ratings.required_life_h", 1.001 * life, "guide-life", true),
    ] {
        // The same pass, judged against the changed rating.
        let mut cfg = cfg.clone();
        cfg.set_text(&format!("{key} = {limit}")).unwrap();
        let v = sim_components::analysis::verdicts(&cfg, &load).unwrap();
        assert_eq!(v.iter().find(|v| v.check == check).unwrap().ok, !fails, "{check} at {limit}");
    }
}

/// In tool space the track is a given axis beside the flange's pose: the inverse puts the track exactly where the tool
/// pose says and solves the arm's six joints for the pose from there, and forward of it returns the pose within 1e-9,
/// at poses along the whole track; the flange held while only the track moves needs the arm to move back by as much.
#[test]
fn the_track_is_a_given_axis_beside_the_tool_pose() {
    let m = (ARM_ON_TRACK.build)(&tool_config(&ARM_ON_TRACK, ARM_ON_TRACK_TOOL_CONFIG)).unwrap();
    let down = std::f64::consts::FRAC_PI_2;
    let (mut q, mut back) = ([0.0; 7], [0.0; 7]);
    for track in [0.3, 1.5, 2.8, 3.7] {
        let pose = [track, track + 0.9, 0.3, 0.8, 0.1, down, -0.2];
        assert_eq!(m.unreachable(&pose, 0.0), None, "{pose:?}");
        m.inverse(&pose, &mut q);
        assert!((q[0] / track_ratio() - track).abs() < 1e-12, "track at {} for {track}", q[0] / track_ratio());
        m.forward(&q, &mut back);
        assert!(pose.iter().zip(&back).all(|(a, b)| (a - b).abs() < 1e-9), "{pose:?} came back as {back:?}");
    }
    // The same flange from the track 0.2 m further on: the arm's base joint turns to reach back.
    let (mut here, mut there) = ([0.0; 7], [0.0; 7]);
    m.inverse(&[1.5, 2.4, 0.3, 0.8, 0.0, down, 0.0], &mut here);
    m.inverse(&[1.7, 2.4, 0.3, 0.8, 0.0, down, 0.0], &mut there);
    assert!((there[0] - here[0] - 0.2 * track_ratio()).abs() < 1e-9 && (there[1] - here[1]).abs() > 1.0);
    sim_core::testkit::static_hold(m.as_ref());
}

/// The linear motor axis's frame momentum counts the forcer in the moving mass as it does in the first moment: every
/// moving part sits on the x axis at the carriage's travel x, so by hand at 0.2 m the first moment is 0.2 times the
/// mass, whatever the payload's share, and the mass holds at least the carriage's 5 kg and the forcer's 3 kg.
#[test]
fn the_linear_motor_frame_momentum_has_the_forcers_mass() {
    let m = (LINEAR_MOTOR.build)(&rigid(&LINEAR_MOTOR)).unwrap();
    let (mut state, mut torque) = (Vec::new(), [0.0; 1]);
    m.equilibrium(&[0.2], &mut state, &mut torque);
    let f = m.frame_momentum(&state).unwrap();
    assert!(f.mass >= 5.0 + 3.0, "{} kg is less than carriage and forcer", f.mass);
    near(f.first_moment[0], f.mass * 0.2, 1e-9, "first moment");
}

/// The mesh friction takes energy out as the other gearbox losses do, so the lossless config has it off.
#[test]
fn the_lossless_config_lists_the_mesh_friction() {
    assert!(LOSSY_EFFECTS.contains(&"gearbox_mesh_friction"));
}

/// The condition number is that of the geometric Jacobian, its turn rows the angular velocity (times the length 0.86 m):
/// by hand from the flange's rotation at the joints one microradian either side, the angular velocity the vector of the
/// antisymmetric part of R+ R-^T, whose vee is 2 sin(turn) w, the turn 2e-6 rad; the position rows as they are.
#[test]
fn the_condition_number_is_that_of_the_geometric_jacobian() {
    let cfg = tool_config(&URDF_ARM, ARM_TOOL_CONFIG);
    let chain = arm::chain(&cfg).unwrap();
    let fk = |q: &[f64]| chain.tip(q);
    let home = cfg.f64s("kinematics.home_joints").unwrap();
    let pose = tool::Pose::new(&fk, home.clone(), vec![0, 1, 2, 3, 4, 5], 0.86, 30.0).unwrap();
    let mut jac = vec![0.0; 36];
    let mut p = home.clone();
    for c in 0..6 {
        p[c] = home[c] + 1e-6;
        let (ra, pa) = fk(&p);
        p[c] = home[c] - 1e-6;
        let (rb, pb) = fk(&p);
        p[c] = home[c];
        let rel = sim_components::rigid::matmul(&ra, &sim_components::rigid::transpose(&rb));
        let w = [rel[2][1] - rel[1][2], rel[0][2] - rel[2][0], rel[1][0] - rel[0][1]];
        for r in 0..3 {
            jac[r * 6 + c] = (pa[r] - pb[r]) / 2e-6;
            jac[(3 + r) * 6 + c] = 0.86 * w[r] / (2.0 * (2e-6f64).sin());
        }
    }
    near(pose.condition(&fk, &home), tool::condition_number(&jac, 6, 6), 1e-3, "condition");
}

/// The one-way bend in its joint's plane takes the body beyond the link whole, its turning inertia too: the tube of
/// 4.5 kg with 10 kg and 0.5 kg m^2 (about y) at its tip, k = 351467.3 N/m, modal mass m / 4 + M + (1.3765 / L)^2 J =
/// 1.125 + 10 + (1.3765 / 0.8)^2 x 0.5 = 12.60528 kg (the tip turns by 1.3765 d / L): f = sqrt(k / M_d) / 2 pi =
/// 26.5758 Hz, against 28.2887 Hz of the mass alone.
#[test]
fn the_one_way_bend_carries_the_turning_inertia_of_the_tip() {
    let m = tube_with_tip(
        "tip-inertia",
        (4.5, [3.546e-3, 0.24, 0.24]),
        (10.0, [0.5, 0.5, 0.5]),
        -0.57,
        &["effects.joint_elasticity = false", r#"link_bending.directions = ["in_plane"]"#],
    );
    let modes = m.modes(&[0.0, 0.0]);
    assert_eq!(modes.len(), 1, "{modes:?}");
    near(modes[0].hz, 26.5758, 1e-4, "bend with the tip's inertia");
}

/// A bar of its own (`arm.bars_*`): 5 kg on joint 2's link at (0, 0, 0.3) of its frame, 0.3 m from the joint's axis with
/// the arm stretched along x (q2 = pi / 2): the gravity torque about joint 2 grows by 5 x 9.80665 x 0.3 = 14.70998 N m,
/// held by 14.70998 / 120 = 0.122583 N m more at motor 2 (-1.47753 - 0.122583 = -1.600113); joint 3 and beyond do not
/// carry it. A bar on a joint that is not there is refused.
#[test]
fn a_parallelogram_bar_is_a_body_of_its_own() {
    let mut cfg = rigid(&URDF_ARM);
    for edit in ["arm.bars_on = [2.0]", "arm.bars_mass_kg = [5.0]", "arm.bars_com_m = [0.0, 0.0, 0.3]"] {
        cfg.set_text(edit).unwrap();
    }
    let m = (URDF_ARM.build)(&cfg).unwrap();
    let mut torque = [0.0; 6];
    m.feedforward_torque(&[0.0, std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0], &[0.0; 6], &mut torque);
    near(torque[1], -1.600113, 1e-5, "motor 2 with the bar");
    near(torque[2], -0.178154, 1e-5, "motor 3 without it");
    cfg.set_text("arm.bars_on = [7.0]").unwrap();
    assert!((URDF_ARM.build)(&cfg).is_err());
}

/// A four-bar of unequal links on the SCARA (`arm.four_bar_*`): the follower's pivot F at (0.15, 0.06, 0) of link 1, the
/// crank point C 0.08 m out along link 2 from joint 2 (at (0.325, 0) of link 1), a 0.2 m coupler (0.5 kg) from C to T
/// and a 0.12 m follower (0.3 kg) from F to T, in link 1's plane. The exact loop by hand in link 1's frame:
/// C = (0.325 + 0.08 cos q2, 0.08 sin q2), d = C - F, cos a = (l_f^2 + |d|^2 - l_c^2) / (2 l_f |d|),
/// T = F + l_f (cos a e + sin a e_perp); each bar a slender rod, its centre at its middle, m l^2 / 12 about it. Its share
/// of the mass matrix over (q1, q2) is sum m dc/dq_i . dc/dq_j + I db/dq_i db/dq_j (c the centre, b the bar's angle in
/// the base frame, by central differences of the closed form), which the machine's (the feedforward's torques at unit
/// accelerations, at rest, less the SCARA's own, times the ratios 100 and 80) matches to 1e-7 at q2 = 0.3 and 1.2
/// (q1 = 0.4). Its energy holds over the suite's pulse and ring (loss-free).
#[test]
fn a_four_bar_of_unequal_links_closes_and_carries_its_bars_mass_by_hand() {
    let keys = [
        "arm.four_bar_ground_on = 1",
        "arm.four_bar_ground_pin_m = [0.15, 0.06, 0.0]",
        "arm.four_bar_crank_on = 2",
        "arm.four_bar_crank_pin_m = [0.08, 0.0, 0.0]",
        "arm.four_bar_normal = [0.0, 0.0, 1.0]",
        "arm.four_bar_lengths_m = [0.2, 0.12]",
        "arm.four_bar_masses_kg = [0.5, 0.3]",
        "arm.four_bar_branch = 1",
    ];
    let mut with = rigid(&SCARA);
    keys.iter().for_each(|k| with.set_text(k).unwrap());
    let (plain, looped) = ((SCARA.build)(&rigid(&SCARA)).unwrap(), (SCARA.build)(&with).unwrap());
    // The bars' centres and angles in the base frame at (q1, q2), by the closed form.
    let bars = |q1: f64, q2: f64| -> [(f64, [f64; 2], f64); 2] {
        let (lc, lf, f) = (0.2f64, 0.12f64, [0.15f64, 0.06]);
        let c = [0.325 + 0.08 * q2.cos(), 0.08 * q2.sin()];
        let d = [c[0] - f[0], c[1] - f[1]];
        let dd = d[0].hypot(d[1]);
        let e = [d[0] / dd, d[1] / dd];
        let cos = (lf * lf + dd * dd - lc * lc) / (2.0 * lf * dd);
        let sin = (1.0 - cos * cos).sqrt();
        let t = [f[0] + lf * (cos * e[0] - sin * e[1]), f[1] + lf * (cos * e[1] + sin * e[0])];
        let world = |p: [f64; 2]| [p[0] * q1.cos() - p[1] * q1.sin(), p[0] * q1.sin() + p[1] * q1.cos()];
        let mid = |a: [f64; 2], b: [f64; 2]| world([0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])]);
        let angle = |a: [f64; 2], b: [f64; 2]| (b[1] - a[1]).atan2(b[0] - a[0]) + q1;
        [(0.5, mid(c, t), angle(c, t)), (0.3, mid(f, t), angle(f, t))]
    };
    for q2 in [0.3, 1.2] {
        let q = [0.4, q2];
        let h = 1e-6;
        let rate = |j: usize| {
            let (mut a, mut b) = (q, q);
            a[j] -= h;
            b[j] += h;
            let (lo, hi) = (bars(a[0], a[1]), bars(b[0], b[1]));
            std::array::from_fn::<_, 2, _>(|k| {
                (
                    [(hi[k].1[0] - lo[k].1[0]) / (2.0 * h), (hi[k].1[1] - lo[k].1[1]) / (2.0 * h)],
                    (hi[k].2 - lo[k].2) / (2.0 * h),
                )
            })
        };
        let (r0, r1) = (rate(0), rate(1));
        let by_hand = |i: usize, j: usize| -> f64 {
            let (ri, rj) = (if i == 0 { r0 } else { r1 }, if j == 0 { r0 } else { r1 });
            (0..2)
                .map(|k| {
                    let (m, l) = ([0.5, 0.3][k], [0.2, 0.12][k]);
                    m * (ri[k].0[0] * rj[k].0[0] + ri[k].0[1] * rj[k].0[1]) + m * l * l / 12.0 * ri[k].1 * rj[k].1
                })
                .sum()
        };
        let pose = [q[0], q[1], 0.1, 0.0];
        for j in 0..2 {
            let mut acc = [0.0; 4];
            acc[j] = 1.0;
            let (mut a, mut b) = ([0.0; 4], [0.0; 4]);
            looped.feedforward_torque(&pose, &acc, &mut a);
            plain.feedforward_torque(&pose, &acc, &mut b);
            for i in 0..2 {
                let got = [100.0, 80.0][i] * (a[i] - b[i]);
                let want = by_hand(i, j);
                assert!((got - want).abs() < 1e-7 * want.abs().max(1e-3), "q2 {q2}: M[{i}][{j}] {got} against {want}");
            }
        }
    }
    let mut lossless = (SCARA.suite_configs)()[0].clone();
    keys.iter().for_each(|k| lossless.set_text(k).unwrap());
    sim_core::testkit::energy_conservation((SCARA.build)(&lossless).unwrap().as_ref());
}

/// A four-bar on a bending link (`bar_field` with a `Closure`): its pins move with the mode as their bodies' points, its
/// bars turn as the loop's Jacobian has them at the deflected pins. Its limit, the parallelogram: on the SCARA the
/// follower's pivot at joint 1's axis (link 1's origin), the crank point 0.08 m out on link 2, the follower 0.08 m and the
/// coupler 0.325 m, link 1's length, the crank square to link 2 (0.08 m along its y, so that the loop is no flat
/// parallelogram at zero joints, where the modes' masses are taken; at a dead point a four-bar with bending is refused):
/// the follower stays parallel to link 2's crank and the coupler to link 1's chord,
/// as two parallel bars do (`bars_turns_with` [2, 1]: the follower turning with link 2, the coupler with link 1, the bent
/// link, by its chord). Link 1 bending in its plane (a 120 x 8 mm aluminium tube), both give the same modes to 1e-9 at
/// two poses. And the energy holds with the bend acting back on the joints, the unequal four-bar of
/// `a_four_bar_of_unequal_links_closes_and_carries_its_bars_mass_by_hand` on links 1 and 2 bending (refused before).
#[test]
fn a_four_bar_on_a_bending_link_has_the_parallelograms_modes_in_its_limit() {
    let bent = |extra: &[&str]| {
        let mut cfg = rigid(&SCARA);
        for edit in [
            "effects.link_bending = true",
            r#"link_bending.directions = ["in_plane"]"#,
            "link_bending.outer_diameter_m = [0.12, 0.0, 0.0, 0.0]",
            "link_bending.wall_m = [0.008, 0.0, 0.0, 0.0]",
            "link_bending.damping_ratio = 0.0",
        ]
        .iter()
        .chain(extra)
        {
            cfg.set_text(edit).unwrap();
        }
        cfg
    };
    let (lc, lf) = (0.5 * 0.325f64.powi(2) / 12.0, 0.3 * 0.08f64.powi(2) / 12.0);
    let parallel = bent(&[
        "arm.bars_on = [1, 2]",
        "arm.bars_turns_with = [2, 1]",
        "arm.bars_pin_m = [0.0, 0.0, 0.0, 0.0, 0.08, 0.0]",
        "arm.bars_mass_kg = [0.3, 0.5]",
        "arm.bars_com_m = [0.0, 0.04, 0.0, -0.1625, 0.0, 0.0]",
        &format!("arm.bars_inertia_kgm2 = [{lf}, 0.0, {lf}, 0.0, 0.0, 0.0, 0.0, {lc}, {lc}, 0.0, 0.0, 0.0]"),
    ]);
    let looped = bent(&[
        "arm.four_bar_ground_on = 1",
        "arm.four_bar_ground_pin_m = [0.0, 0.0, 0.0]",
        "arm.four_bar_crank_on = 2",
        "arm.four_bar_crank_pin_m = [0.0, 0.08, 0.0]",
        "arm.four_bar_normal = [0.0, 0.0, 1.0]",
        "arm.four_bar_lengths_m = [0.325, 0.08]",
        "arm.four_bar_masses_kg = [0.5, 0.3]",
        "arm.four_bar_branch = 1",
    ]);
    let (a, b) = ((SCARA.build)(&parallel).unwrap(), (SCARA.build)(&looped).unwrap());
    for pose in [[0.4, 1.0, 0.1, 0.0], [-0.3, 1.3, 0.1, 0.0]] {
        let (ma, mb) = (a.modes(&pose), b.modes(&pose));
        assert!(!ma.is_empty() && ma.len() == mb.len(), "{ma:?} {mb:?}");
        for (x, y) in ma.iter().zip(&mb) {
            assert!((x.hz / y.hz - 1.0).abs() < 1e-9, "{pose:?}: parallel {} Hz, four-bar {} Hz", x.hz, y.hz);
        }
    }
    let mut lossless = (SCARA.suite_configs)()[0].clone();
    for edit in [
        "effects.link_bending = true",
        "effects.link_bending_reaction = true",
        r#"link_bending.directions = ["in_plane"]"#,
        "link_bending.damping_ratio = 0.0",
        "arm.four_bar_ground_on = 1",
        "arm.four_bar_ground_pin_m = [0.15, 0.06, 0.0]",
        "arm.four_bar_crank_on = 2",
        "arm.four_bar_crank_pin_m = [0.08, 0.0, 0.0]",
        "arm.four_bar_normal = [0.0, 0.0, 1.0]",
        "arm.four_bar_lengths_m = [0.2, 0.12]",
        "arm.four_bar_masses_kg = [0.5, 0.3]",
        "arm.four_bar_branch = 1",
    ] {
        lossless.set_text(edit).unwrap();
    }
    sim_core::testkit::energy_conservation((SCARA.build)(&lossless).unwrap().as_ref());
}

/// The carriage's motor rotor is part of what its guides carry: it spins about the vertical at the ratio times the
/// carriage's speed, and the carriage accelerating at 2 m/s^2 turns its momentum J r a about z, a yaw moment the four
/// blocks carry as +-M_z / (2 x 0.4) each. 1.7e-4 kg m^2 more at the motor (on the gearbox's input: the track's motor is
/// named, its rotor the catalogue's) changes nothing else of the guide load at rest, so it moves by four times the most
/// loaded block's share, 4 x 1.7e-4 x 314.159 x 2 / 0.8 = 0.534070 N: down, as the rotor's yaw (+z, the rack on +y)
/// works against the chain's (-2 SUM_MY), up with the rack on -y.
#[test]
fn the_rotor_of_the_track_adds_its_gyroscopic_moment_to_the_guide_load() {
    let load = |side: f64, input: f64| {
        let mut cfg = rigid(&ARM_ON_TRACK);
        cfg.set_text(&format!("track.rack_side = {side}")).unwrap();
        cfg.set_text(&format!("gearbox.1.inertia_at_input_kgm2 = {input}")).unwrap();
        let m = (ARM_ON_TRACK.build)(&cfg).unwrap();
        let mut acc = [0.0; 7];
        acc[0] = 2.0;
        track_signal(m.as_ref(), &sideways(), &acc, "axis.j1.guide_load")
    };
    let by_hand = 4.0 * 1.7e-4 * track_ratio() * 2.0 / 0.8;
    for side in [1.0, -1.0] {
        let moved = load(side, 3.59e-5 + 1.7e-4) - load(side, 3.59e-5);
        assert!((moved + side * by_hand).abs() < 1e-6 * by_hand, "side {side}: {moved} against {by_hand}");
    }
}

/// A walk of one step that crosses the wrist's singularity is refused although both its ends are clear of it: the
/// six-axis arm at its home with joint 5 at +0.03 rad, walked to the tool pose of joint 5 at -0.03 (0.06 rad of turn
/// about joint 5's axis and the flange's offset times that: one step of the walk), the condition number 128.8 and
/// 130.0 at its ends, below a limit of 200; the determinant of the geometric Jacobian changes its sign between them. Unguarded the
/// walk gets there (through the singular pose), and a step of the same size that stays on one side (0.33 to 0.27 rad)
/// passes. (With the shipped limit of 30 the condition number is 3.86 / q5 here: the ends of a crossing would be 0.26
/// rad apart, beyond a step's 0.05 rad, so the ends alone would tell there; the sign tells at any limit.)
#[test]
fn a_walk_of_one_step_through_a_singular_pose_is_refused() {
    let cfg = tool_config(&URDF_ARM, ARM_TOOL_CONFIG);
    let chain = arm::chain(&cfg).unwrap();
    let fk = |q: &[f64]| chain.tip(q);
    let home = cfg.f64s("kinematics.home_joints").unwrap();
    let pose = tool::Pose::new(&fk, home.clone(), vec![0, 1, 2, 3, 4, 5], 0.86, 200.0).unwrap();
    let at = |q5: f64| {
        let mut q = home.clone();
        q[4] = q5;
        q
    };
    let walk = |from: f64, to: f64, guard: bool| {
        let mut tool = vec![0.0; 6];
        pose.forward(&fk, &at(to), &mut tool);
        let mut q = vec![0.0; 6];
        let reached = pose.walk(&fk, &at(from), &tool, &mut q, guard);
        (reached, q)
    };
    for q5 in [0.03, -0.03] {
        let c = pose.condition(&fk, &at(q5));
        assert!(c > 128.0 && c < 131.0, "condition {c} at joint 5 = {q5}");
    }
    assert!(!walk(0.03, -0.03, true).0, "the guarded walk crossed the singular pose");
    let (reached, q) = walk(0.03, -0.03, false);
    assert!(reached && (q[4] + 0.03).abs() < 1e-6, "{q:?}");
    assert!(walk(0.33, 0.27, true).0);
}

/// Every shipped arm states each motor's ratio twice, as `arm.gear_ratios` (the simulation's) and as its gearbox's
/// `gearbox.ratio` with its own `[gearbox.<id>]` over it (the gearbox life, its drag, the drives' tuning): the same.
#[test]
fn the_shipped_arms_give_each_motor_one_gear_ratio() {
    let configs = [
        URDF_ARM.config(),
        SCARA.config(),
        PALLETISER.config(),
        ARM_ON_TRACK.config(),
        tool_config(&SCARA, SCARA_TOOL_CONFIG),
        tool_config(&URDF_ARM, ARM_TOOL_CONFIG),
        tool_config(&URDF_ARM, ARM5_TOOL_CONFIG),
        tool_config(&URDF_ARM, ARM7_TOOL_CONFIG),
        tool_config(&ARM_ON_TRACK, ARM_ON_TRACK_TOOL_CONFIG),
    ];
    for cfg in &configs {
        for (k, ratio) in cfg.f64s("arm.gear_ratios").unwrap().iter().enumerate() {
            let own = sim_components::catalog::motor_config(cfg, sim_components::chain::MOTOR_IDS[k]).unwrap();
            assert_eq!(
                own.f64("gearbox.ratio").unwrap(),
                *ratio,
                "{}: motor {}",
                cfg.f64s("arm.gear_ratios").unwrap().len(),
                k + 1
            );
        }
    }
}

/// A bar of its own (`arm.bars_turns_with`): 5 kg pinned at joint 3's origin (the elbow) and turning with link 1 (the
/// turret), its centre 0.3 m along the turret's x from the pin, its inertia 0.1, 0.2, 0.3 kg m^2 about its centre. The
/// arm stretched along x (q2 = pi / 2): the bar hangs on the elbow and does not turn with link 2 or 3, so it loads joint
/// 2 by its weight at the elbow's lever, 5 g (x_elbow - x_2) (not at its centre's, 0.3 m further), and joint 3 not at
/// all; turning the turret it adds I_zz + 5 r^2 (r its centre's distance from the vertical axis), swinging joint 2 it
/// moves with the elbow alone, 5 d^2 (d the elbow's distance from joint 2's axis), and joint 3 not at all. All over the
/// ratio 120 at the motors. Moving at speed with no torque the energy holds (the bar's speed terms are its mass
/// matrix's): its rate along the state's from `derivatives` is 0 to 1e-6 W (runs give 0; the differences' rounding is
/// about 5e-8 W; leaving out the bar's J' q' gives -0.11 W). A bar with the links bending is refused.
#[test]
fn a_parallelogram_bar_of_its_own_moves_with_its_pin_and_turns_with_its_link() {
    let edits = [
        "arm.bars_on = [3.0]",
        "arm.bars_turns_with = [1.0]",
        "arm.bars_mass_kg = [5.0]",
        "arm.bars_com_m = [0.3, 0.0, 0.0]",
        "arm.bars_inertia_kgm2 = [0.1, 0.2, 0.3, 0.0, 0.0, 0.0]",
    ];
    let plain = rigid(&URDF_ARM);
    let mut cfg = plain.clone();
    edits.iter().for_each(|e| cfg.set_text(e).unwrap());
    let (with, without) = ((URDF_ARM.build)(&cfg).unwrap(), (URDF_ARM.build)(&plain).unwrap());
    let pose = [0.0, std::f64::consts::FRAC_PI_2, 0.0, 0.0, 0.0, 0.0];
    let added = |acc: [f64; 6]| -> Vec<f64> {
        let (mut a, mut b) = ([0.0; 6], [0.0; 6]);
        with.feedforward_torque(&pose, &acc, &mut a);
        without.feedforward_torque(&pose, &acc, &mut b);
        a.iter().zip(&b).map(|(a, b)| (a - b) * 120.0).collect()
    };
    let frames = arm::chain(&cfg).unwrap().frames(&pose);
    let (turret, shoulder, elbow) = (frames[0], frames[1].1, frames[2].1);
    let centre: Vec<f64> = (0..3).map(|i| elbow[i] + turret.0[i][0] * 0.3).collect();
    let g = 9.80665;
    let held = added([0.0; 6]);
    near(held[1].abs(), 5.0 * g * (elbow[0] - shoulder[0]).abs(), 1e-9, "joint 2 holds the bar at the elbow");
    assert!(held[2].abs() < 1e-9 && held[0].abs() < 1e-9, "{held:?}");
    let r2 = (centre[0] - turret.1[0]).powi(2) + (centre[1] - turret.1[1]).powi(2);
    let zz = (0..3).map(|k| turret.0[2][k] * turret.0[2][k] * [0.1, 0.2, 0.3][k]).sum::<f64>();
    near(added([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])[0], zz + 5.0 * r2, 1e-9, "turret");
    let axis2 = (0..3).map(|i| frames[1].0[i][1]).collect::<Vec<f64>>();
    let arm: Vec<f64> = (0..3).map(|i| elbow[i] - shoulder[i]).collect();
    let along = (0..3).map(|i| axis2[i] * arm[i]).sum::<f64>();
    let d2 = (0..3).map(|i| arm[i] * arm[i]).sum::<f64>() - along * along;
    let swung = added([0.0, 1.0, 0.0, 0.0, 0.0, 0.0]);
    near(swung[1] - held[1], 5.0 * d2, 1e-9, "joint 2");
    assert!((added([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])[2] - held[2]).abs() < 1e-9);
    // Free, at speed.
    let (mut state, mut hold) = (Vec::new(), vec![0.0; 6]);
    with.equilibrium(&[0.3, 1.2, -0.6, 0.4, 0.8, -0.2], &mut state, &mut hold);
    state[6..12].copy_from_slice(&[0.5, -0.4, 0.6, 0.3, -0.5, 0.4]);
    let mut rates = vec![0.0; state.len()];
    with.derivatives(&state, &[0.0; 6], &mut rates);
    let h = 1e-7;
    let along = |s: f64| -> Vec<f64> { state.iter().zip(&rates).map(|(x, r)| x + s * r).collect() };
    let change = (with.energy(&along(h)) - with.energy(&along(-h))) / (2.0 * h);
    assert!(change.abs() < 1e-6, "the energy changes by {change} W");
    // Bending links with the bar build since 22 November (refused before; `the_parallelogram_holds_the_forearm_against_the_upper_arms_bend`).
    cfg.set_text("effects.link_bending = true").unwrap();
    assert!((URDF_ARM.build)(&cfg).is_ok());
    // The palletiser, its joints coupled: a 4 kg rod pinned on the forearm (URDF joint 3) 0.1 m behind the elbow,
    // parallel to the upper arm (joint 2), and its energy holds alike.
    let mut cfg = rigid(&PALLETISER);
    for edit in [
        "arm.bars_on = [3.0]",
        "arm.bars_turns_with = [2.0]",
        "arm.bars_pin_m = [-0.1, 0.0, 0.0]",
        "arm.bars_mass_kg = [4.0]",
        "arm.bars_com_m = [0.0, 0.0, 0.3]",
        "arm.bars_inertia_kgm2 = [0.12, 0.12, 0.001, 0.0, 0.0, 0.0]",
    ] {
        cfg.set_text(edit).unwrap();
    }
    let m = (PALLETISER.build)(&cfg).unwrap();
    let (mut state, mut hold) = (Vec::new(), vec![0.0; 4]);
    m.equilibrium(&[1.4, 0.0, 1.0, 0.0], &mut state, &mut hold);
    state[4..8].copy_from_slice(&[0.3, -0.4, 0.5, 0.6]);
    let mut rates = vec![0.0; state.len()];
    m.derivatives(&state, &[0.0; 4], &mut rates);
    let along = |s: f64| -> Vec<f64> { state.iter().zip(&rates).map(|(x, r)| x + s * r).collect() };
    // The palletiser's energy is larger: its last bit, 2.3e-13 J, is 2.3e-6 W of the rate over 1e-7 s (as a run shows),
    // 2.3e-8 W over 1e-5 s.
    let h = 1e-5;
    let change = (m.energy(&along(h)) - m.energy(&along(-h))) / (2.0 * h);
    assert!(change.abs() < 1e-6, "palletiser: the energy changes by {change} W");
}
