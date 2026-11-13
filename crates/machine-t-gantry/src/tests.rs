//! Machine-specific checks: kinematics, static hold on Z, constant torque, rigid limit,
//! and the comparison against the independent SciPy implementation.

use super::*;
use sim_core::testkit::{DT, PulseRun};
use sim_core::{Rk4, advance};

fn machine(mut cfg: Config, edit: &[&str]) -> TGantry {
    for e in edit {
        cfg.set_text(e).unwrap();
    }
    TGantry::new(&cfg).unwrap()
}

fn rigid(edit: &[&str]) -> TGantry {
    let mut all = vec!["effects.belt_stretch = false"];
    all.extend(edit);
    machine(lossless_config(), &all)
}

#[test]
fn kinematics_follow_the_plan() {
    let m = rigid(&["gearbox.ratio = 1.0"]);
    let mut tool = [0.0; 2];
    // Both motors the same way: X only. Against each other: Z only.
    m.forward(&[1.0, 1.0], &mut tool);
    assert!((tool[0] - m.r).abs() < 1e-15 && tool[1].abs() < 1e-15);
    m.forward(&[1.0, -1.0], &mut tool);
    assert!(tool[0].abs() < 1e-15 && (tool[1] - m.r).abs() < 1e-15);
}

/// Static hold on Z: each motor holds m_z g r / 2 at the pulley, with opposite signs.
#[test]
fn static_hold_on_z_needs_opposite_torques() {
    let expected = (5.0 + 2.0) * G * 0.02 / 2.0 / 5.0;
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    let m = rigid(&[]);
    m.equilibrium(&[0.5, 0.25], &mut state, &mut hold);
    assert!((hold[0] - expected).abs() < 1e-12 && (hold[1] + expected).abs() < 1e-12, "{hold:?}");
    let mut d = [0.0; N_RIGID];
    m.derivatives(&state, &hold, &mut d);
    assert!(d.iter().all(|v| v.abs() < 1e-12), "{d:?}");

    // The elastic model agrees to within the belt strain (see Belt::section) and is at rest too.
    let m = machine(lossless_config(), &[]);
    m.equilibrium(&[0.5, 0.25], &mut state, &mut hold);
    assert!((hold[0] - expected).abs() < 1e-3 * expected && (hold[1] + expected).abs() < 1e-3 * expected, "{hold:?}");
    let mut d = [0.0; N_ELASTIC];
    m.derivatives(&state, &hold, &mut d);
    assert!(d.iter().all(|v| v.abs() < 1e-6), "{d:?}");
    // Belt tensions at rest: sections 1 and 3 equal, section 2 higher by m_z g / 2.
    let t = m.sections(&state).map(|f| f.tension);
    assert!((t[0] - t[2]).abs() < 1e-6 && (t[1] - t[0] - 7.0 * G / 2.0).abs() < 1e-6, "{t:?}");
}

/// Constant torque: acceleration equals force divided by effective mass.
#[test]
fn constant_torque_gives_force_over_effective_mass() {
    let m = rigid(&["effects.belt_mass = false"]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&[0.1, 0.1], &mut state, &mut hold);
    let mut rk = Rk4::new(N_RIGID);
    // Extra torque: 0.1 N m on both for X, and +-0.05 N m for Z on top.
    let torque = [hold[0] + 0.1 + 0.05, hold[1] + 0.1 - 0.05];
    let t_end = 0.25;
    for _ in 0..(t_end / DT) as usize {
        advance(&m, &mut rk, &mut state, &torque, DT, 4);
    }
    let m_motors = 2.0 * (1.0e-4 + 25.0 * 0.3e-4) / (0.02 * 0.02);
    let ax = (2.0 * 0.1 * 5.0 / 0.02) / (11.0 + m_motors);
    let az = (2.0 * 0.05 * 5.0 / 0.02) / (7.0 + m_motors);
    assert!((state[0] - 0.1 - 0.5 * ax * t_end * t_end).abs() < 1e-9, "x {}", state[0]);
    assert!((state[1] - 0.1 - 0.5 * az * t_end * t_end).abs() < 1e-9, "z {}", state[1]);
    assert!((state[2] - ax * t_end).abs() < 1e-9 && (state[3] - az * t_end).abs() < 1e-9);
}

#[test]
fn payload_switch_and_release_change_the_load() {
    let mut m = rigid(&[]);
    assert_eq!(m.m_z(), 7.0);
    m.set_payload_attached(false);
    assert_eq!(m.m_z(), 5.0);
    assert_eq!(rigid(&["effects.payload = false"]).m_z(), 5.0);
}

#[test]
fn belt_length_does_not_depend_on_the_pose() {
    let m = rigid(&[]);
    let total = |x, z| m.geo(x, z).iter().sum::<f64>();
    assert!((total(0.0, 0.0) - total(1.0, 0.5)).abs() < 1e-12);
    assert!((total(0.0, 0.5) - total(1.0, 0.0)).abs() < 1e-12);
}

#[test]
fn guide_friction_grows_with_acceleration_and_overhang() {
    let m = machine(ENTRY.config(), &[]);
    let (x_rest, z_rest) = m.load_scales(0.25, 0.0);
    let (x_acc, z_acc) = m.load_scales(0.25, 8.0);
    // At rest the X guide carries the payload on top of the nominal weight; the Z guide only preload.
    assert!((x_rest - (200.0 + 11.0 * G) / (200.0 + 9.0 * G)).abs() < 1e-12);
    assert_eq!(z_rest, 1.0);
    assert!(x_acc > x_rest && z_acc > 1.0);
    // Less overhang (z raised to the centre of mass) means less tilting moment.
    assert!(m.load_scales(0.3, 8.0).0 < x_acc);
}

/// Rigid limit: with very high belt stiffness the full model matches the rigid reference within 1 %.
#[test]
fn stiff_belt_converges_to_the_rigid_reference() {
    let travel = |edit: &[&str], substeps: u32| {
        let mut all = vec!["effects.gearbox_backlash = false", "effects.frame_compliance = false", "effects.tool_flex = false"];
        all.extend(edit);
        let m = machine(ENTRY.config(), &all);
        let mut run = PulseRun::new(&m, DT, 1.0, 0.2);
        run.substeps = substeps;
        let start = run.tool_pos();
        run.run(1.0, |_| {});
        let end = run.tool_pos();
        [end[0] - start[0], end[1] - start[1]]
    };
    let rigid = travel(&["effects.belt_stretch = false"], 4);
    let stiff = travel(&["belt.stiffness_ea_n = 5.0e7", "gearbox.stiffness_nm_rad = 1.0e6"], 16);
    let soft = travel(&[], 4);
    for axis in 0..2 {
        assert!(rigid[axis] > 0.02, "the reference move is too small: {rigid:?}");
        assert!((stiff[axis] - rigid[axis]).abs() < 0.01 * rigid[axis], "stiff {stiff:?}, rigid {rigid:?}");
    }
    // The default belt is visibly softer, otherwise the check above would prove nothing.
    let gap = |a: [f64; 2]| (a[0] - rigid[0]).abs() + (a[1] - rigid[1]).abs();
    assert!(gap(soft) > gap(stiff), "soft {soft:?}, stiff {stiff:?}, rigid {rigid:?}");
}

/// The torque pattern of the reference move: a push, a pause, a pull back.
fn reference_pulse(t: f64) -> f64 {
    let bump = |t0: f64| if t > t0 && t < t0 + 0.15 { (std::f64::consts::PI * (t - t0) / 0.15).sin().powi(2) } else { 0.0 };
    bump(0.0) - bump(0.3)
}

/// Independent reference: one move matches a SciPy implementation of the same equations.
/// The trace comes from `reference/scipy_reference.py`, which reads the same default config.
#[test]
fn reference_move_matches_scipy() {
    // The SciPy implementation has no leaning Z axis.
    let m = machine(ENTRY.config(), &["effects.tool_flex = false"]);
    let pose = [0.5, 0.25];
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&pose, &mut state, &mut hold);
    let (mut with, mut without) = ([0.0; 2], [0.0; 2]);
    m.feedforward_torque(&pose, &[m.axes[0].max_acc, m.axes[1].max_acc], &mut with);
    m.feedforward_torque(&pose, &[0.0; 2], &mut without);

    let mut rk = Rk4::new(N_ELASTIC);
    let mut trace = vec![(0.0, pose[0], pose[1])];
    for step in 0..(0.6 / DT).round() as usize {
        let p = reference_pulse((step as f64 + 0.5) * DT);
        let torque = [hold[0] + (with[0] - without[0]) * p, hold[1] + (with[1] - without[1]) * p];
        advance(&m, &mut rk, &mut state, &torque, DT, 4);
        if (step + 1) % 16 == 0 {
            trace.push(((step + 1) as f64 * DT, state[X], state[Z]));
        }
    }

    let reference: Vec<Vec<f64>> = include_str!("../reference/scipy_trace.csv")
        .lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with('t'))
        .map(|l| l.split(',').map(|v| v.trim().parse().unwrap()).collect())
        .collect();
    assert_eq!(reference.len(), trace.len(), "the reference trace has a different length");
    let travel = |i: usize| reference.iter().map(|r| (r[i] - pose[i - 1]).abs()).fold(0.0, f64::max);
    let (travel_x, travel_z) = (travel(1), travel(2));
    assert!(travel_x > 0.02 && travel_z > 0.01, "the reference move is too small");
    for (ours, theirs) in trace.iter().zip(&reference) {
        assert!((ours.0 - theirs[0]).abs() < 1e-9);
        assert!((ours.1 - theirs[1]).abs() < 0.002 * travel_x, "x at t={}: {} vs SciPy {}", ours.0, ours.1, theirs[1]);
        assert!((ours.2 - theirs[2]).abs() < 0.002 * travel_z, "z at t={}: {} vs SciPy {}", ours.0, ours.2, theirs[2]);
    }
}

/// The EXCT-30 config against the catalogue files it was derived from, and against Festo's own
/// formula for the motor torque at 45 degrees travel, where motor A carries both axes.
#[test]
fn exct_30_config_agrees_with_the_catalogue() {
    let cfg = Config::parse(EXCT_30_CONFIG).unwrap();
    let close = |a: f64, b: f64, tolerance: f64| (a - b).abs() <= tolerance * b.abs();
    // A row of a catalogue table (schema 1, SI units) by its first columns, as a lookup of its numbers.
    let row = |text: &'static str, starts: &str| -> Box<dyn Fn(&str) -> f64> {
        let mut lines = text.lines();
        let header: Vec<&str> = lines.next().unwrap().split(',').collect();
        let row: Vec<&str> = lines.find(|l| l.starts_with(starts)).unwrap().split(',').collect();
        Box::new(move |column| row[header.iter().position(|h| *h == column).unwrap()].parse().unwrap())
    };
    let f = row(include_str!("../../../catalog/festo/belt_axes.csv"), "EXCT,30,");
    assert!(close(cfg.f64("geometry.pulley_radius_m").unwrap(), f("pulley_effective_diameter_m") / 2.0, 1e-3));
    let ea = f("belt_elongation_force_n") / f("belt_elongation");
    assert!(close(cfg.f64("belt.stiffness_ea_n").unwrap(), ea, 1e-3), "belt stiffness {ea}");
    assert!(close(cfg.f64("belt.tooth_pitch_m").unwrap(), f("belt_pitch_m"), 1e-9));
    assert_eq!(cfg.f64("drive.peak_torque_nm").unwrap(), f("max_pulley_torque_nm"));
    assert_eq!(cfg.f64("mass.payload_kg").unwrap(), f("rated_payload_kg"));
    assert!(cfg.f64("axis.x.max_velocity_m_s").unwrap() <= f("max_speed_m_s"));
    assert!(cfg.f64("axis.x.max_acceleration_m_s2").unwrap() <= f("max_acceleration_m_s2"));
    assert_eq!(cfg.f64("gearbox.ratio").unwrap(), 1.0);

    // The motor, from the row of the 1FK2205-4AF in the Siemens file.
    let motor = row(include_str!("../../../catalog/siemens/motors_s210.csv"), "1FK2205-4AF,");
    assert!(close(cfg.f64("motor.rotor_inertia_kgm2").unwrap(), motor("rotor_inertia_kgm2"), 1e-9));
    assert!(close(cfg.f64("motor.torque_constant_nm_a").unwrap(), motor("static_torque_nm") / motor("stall_current_a"), 1e-3));
    assert!(cfg.f64("drive.peak_torque_nm").unwrap() <= motor("max_torque_nm"));
    assert!(close(cfg.f64("drive.rated_speed_rad_s").unwrap(), motor("rated_speed_400v_rad_s"), 1e-3));

    // Festo: M = a (c_load m_L + c_motor J_m + c) + c_g (m_g + m_L), for the Z stroke of the config (SI: J_m in kg m2).
    let stroke = cfg.f64("geometry.stroke_z_m").unwrap();
    let c = row(include_str!("../../../catalog/festo/belt_axis_torque_formula.csv"), &format!("EXCT,30,{stroke:?},"));
    assert_eq!(cfg.f64("mass.z_axis_kg").unwrap(), c("gravity_mass_kg"));
    for (a, load) in [(5.0, 0.0), (25.0, 0.0), (5.0, 3.0), (25.0, 3.0), (40.0, 6.0)] {
        let gantry = machine(cfg.clone(), &[&format!("mass.payload_kg = {load:?}")]);
        let mut torque = [0.0; 2];
        gantry.feedforward_torque(&[0.5, 0.1], &[a / 2f64.sqrt(), a / 2f64.sqrt()], &mut torque);
        let festo_torque = a * (c("load_factor_m") * load + c("motor_inertia_factor_1_m") * motor("rotor_inertia_kgm2") + c("constant_kg_m"))
            + c("gravity_factor_nm_kg") * (c("gravity_mass_kg") + load);
        assert!(close(torque[0], festo_torque, 0.02), "a = {a}, load = {load}: {} against Festo's {festo_torque}", torque[0]);
    }
}

/// The leaning Z axis against a hand calculation at two heights: carriage and Z axis are two masses on a
/// spring k = 1 / (1 / lateral + h^2 / tilt), so a deflection u swings back with u k (1 / m_z + 1 / m_c),
/// and the tool tip is off by k u (1 / lateral + h h_tool / tilt).
#[test]
fn leaning_z_axis_matches_the_two_mass_hand_calculation() {
    let m = machine(lossless_config(), &["effects.frame_compliance = false"]);
    let (lateral, tilt, m_z, m_c, u) = (2.0e7, 1.2e4, 7.0, 4.0, 1e-4);
    let mut softer = 0.0;
    for z in [0.05, 0.25] {
        let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
        m.equilibrium(&[0.5, z], &mut state, &mut hold);
        state[U] = u;
        let mut d = [0.0; N_ELASTIC];
        m.derivatives(&state, &hold, &mut d);
        let (h, h_tool) = (0.3 - z, 0.7 - z);
        let k = 1.0 / (1.0 / lateral + h * h / tilt);
        assert!((d[V + U] + u * k * (1.0 / m_z + 1.0 / m_c)).abs() < 1e-6 * u * k, "z = {z}: {} m/s2", d[V + U]);
        let (mut pos, mut vel) = ([0.0; 2], [0.0; 2]);
        m.tool_state(&state, &mut pos, &mut vel);
        assert!((pos[0] - 0.5 - k * u * (1.0 / lateral + h * h_tool / tilt)).abs() < 1e-12, "tip at z = {z}: {}", pos[0] - 0.5);
        // Lower down the arm is longer and the axis leans more easily.
        let hz = (k * (1.0 / m_z + 1.0 / m_c)).sqrt() / std::f64::consts::TAU;
        assert!(hz > softer && hz > 20.0 && hz < 400.0, "{hz} Hz at z = {z}");
        softer = hz;
    }
    // Switched off, the Z axis is rigid on the carriage again.
    let rigid = machine(lossless_config(), &["effects.tool_flex = false"]);
    let (mut state, mut hold, mut d) = (Vec::new(), [0.0; 2], [0.0; N_ELASTIC]);
    rigid.equilibrium(&[0.5, 0.25], &mut state, &mut hold);
    state[U] = u;
    rigid.derivatives(&state, &hold, &mut d);
    assert_eq!((d[U], d[V + U]), (0.0, 0.0));
}

/// Sensitivity: more mass to lift needs more torque and less needs less, and it matters more to the
/// cycle than the tooth pitch of the belt does.
#[test]
fn sensitivity_shows_that_the_lifted_mass_matters_and_the_tooth_pitch_hardly() {
    let paths = ["mass.z_axis_kg".to_string(), "belt.tooth_pitch_m".to_string()];
    let (nominal, rows) = sim_components::analysis::sensitivity(&ENTRY, &ENTRY.config(), &paths, 0.2).unwrap();
    let peak = |load: &sim_components::analysis::CycleLoad| load.motors[0].peak_torque / nominal.motors[0].peak_torque - 1.0;
    let (mass, pitch) = (&rows[0], &rows[1]);
    assert!(peak(&mass.low) < -0.03 && peak(&mass.high) > 0.03, "{} and {}", peak(&mass.low), peak(&mass.high));
    assert!(peak(&pitch.low).abs() < peak(&mass.high) && peak(&pitch.high).abs() < peak(&mass.high));
}

/// Path error against a hand calculation: on a circle at constant speed, a position loop with gain Kv and
/// no feedforward follows each axis with the gain 1 / sqrt(1 + (w / Kv)^2), so the tool runs a smaller
/// circle. It is off the path by R (1 - gain) and behind its setpoint by R (w / Kv) gain.
#[test]
fn path_error_on_a_circle_matches_the_hand_calculation() {
    let mut cfg = lossless_config();
    for e in [
        "effects.belt_stretch = false",
        "effects.velocity_feedforward = false",
        "effects.encoder_resolution = false",
        "effects.velocity_filter = false",
        "effects.torque_filter = false",
        "effects.drive_delay = false",
        "start.pose = [0.55, 0.25]",
    ] {
        cfg.set_text(e).unwrap();
    }
    // The circle: radius 50 mm around (0.5, 0.25), sped up over half a second, two turns at 5 rad/s, slowed down.
    let (radius, w, ramp, turns) = (0.05, 5.0, 0.5, 2.0);
    let steady = turns * std::f64::consts::TAU / w;
    let (mut points, mut angle, dt) = (Vec::new(), 0.0, 1e-3);
    let steps = ((2.0 * ramp + steady) / dt).round() as usize;
    for i in 0..=steps {
        let t = i as f64 * dt;
        points.push((t, vec![0.5 + radius * f64::cos(angle), 0.25 + radius * f64::sin(angle)]));
        let share = if t < ramp { t / ramp } else if t > ramp + steady { (2.0 * ramp + steady - t) / ramp } else { 1.0 };
        angle += w * (std::f64::consts::FRAC_PI_2 * share.clamp(0.0, 1.0)).sin().powi(2) * dt;
    }
    let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    let load = sim_components::analysis::analyse_trajectory(&mut sim, points).unwrap();
    let gain = 1.0 / (1.0 + (w / 50.0f64).powi(2)).sqrt();
    let (off_path, behind) = (radius * (1.0 - gain), radius * (w / 50.0) * gain);
    assert!((load.path_error_peak - off_path).abs() < 0.1 * off_path, "{} m off the path, by hand {off_path} m", load.path_error_peak);
    assert!((load.tracking_error_peak - behind).abs() < 0.1 * behind, "{} m behind, by hand {behind} m", load.tracking_error_peak);
    assert!(load.path_error_rms < load.path_error_peak && load.path_error_peak < 0.1 * load.tracking_error_peak);

    // A path that starts elsewhere, or runs backwards in time, is refused.
    let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    let refused = |sim: &mut sim_components::Sim, points| sim.command(sim_components::Command::Trajectory(points)).unwrap_err().key;
    assert_eq!(refused(&mut sim, vec![(0.0, vec![0.3, 0.25]), (1.0, vec![0.4, 0.25])]), "error-trajectory-start");
    assert_eq!(refused(&mut sim, vec![(0.0, vec![0.55, 0.25]), (0.0, vec![0.5, 0.25])]), "error-trajectory");
    // A spline that swings out of the travel range between via points inside it is refused as a path that
    // leaves the range, by both planners.
    let sim = sim_components::Sim::new(&ENTRY, &ENTRY.config()).unwrap();
    let bulge = sim_components::path::parse("spline,0.3,0.5
spline,0.5,0.5
spline,0.7,0.35").unwrap();
    assert_eq!([false, true].map(|fast| sim.plan_path(&bulge, fast).unwrap_err().key), ["error-path-range"; 2]);
    // Its deviation from the path as given is refused the same way: it was reported for a path whose transition
    // curves are allowed 0.1 mm and whose arc leaves the range, and the plan refused afterwards.
    let mut cfg = ENTRY.config();
    cfg.set_text("profile.path_tolerance_m = 0.0001").unwrap();
    let sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    let out = sim_components::path::parse("line,0.5,0.35
arc,0.25,0.1,0.5,0.1,cw
line,0.25,0.3").unwrap();
    for (pieces, fast) in [(&bulge, false), (&bulge, true), (&out, false), (&out, true)] {
        assert_eq!((sim.plan_path(pieces, fast).unwrap_err().key, sim.path_deviation(pieces, fast).unwrap_err().key), ("error-path-range", "error-path-range"), "time-optimal {fast}");
    }
}

/// The resonance of the Z axis with payload (7 kg) leaning on its guide at the height z, by hand: the
/// spring is k = 1 / (1 / lateral + h^2 / tilt) with h = 0.3 m - z.
fn leaning_resonance(z: f64) -> f64 {
    let k = 1.0 / (1.0 / 2.0e7 + (0.3 - z) * (0.3 - z) / 1.2e4);
    (k / 7.0f64).sqrt() / std::f64::consts::TAU
}

/// The largest tool-tip deflection in the 0.3 s after a fast X traverse at the height z, with the shaper as
/// `edit` sets it up.
fn swing_after_a_traverse(z: f64, edit: &[String]) -> f64 {
    let mut cfg = ENTRY.config();
    // A sharp profile, so that the plain move does start the swing.
    for e in ["effects.frame_compliance = false".to_string(), format!("start.pose = [0.1, {z:?}]"), "axis.x.max_jerk_m_s3 = 2000.0".to_string()].iter().chain(edit) {
        cfg.set_text(e).unwrap();
    }
    let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    let deflection = sim.signal_index("axis.x.tool_deflection").unwrap();
    sim.command(sim_components::Command::MoveTo(vec![0.9, z])).unwrap();
    while sim.control.busy() {
        sim.step();
    }
    // The shaper finishes one period after the plan, at most that of the lowest height; then the swing
    // that is left shows.
    sim.run(1.2 / leaning_resonance(0.05), |_| {});
    let (mut sample, mut largest) = (Vec::new(), 0f64);
    sim.run(0.3, |s| {
        s.sample(&mut sample);
        largest = largest.max(sample[deflection].abs());
    });
    largest
}

/// Input shaping: after a fast traverse low down, where the Z axis leans most easily, the tool tip keeps
/// swinging. With the shaper set to the hand-calculated resonance of the Z axis on its guide, less than a
/// tenth of that swing is left.
#[test]
fn input_shaping_leaves_under_a_tenth_of_the_tool_swing() {
    let resonance = leaning_resonance(0.05);
    assert!((resonance - 26.2).abs() < 0.2, "{resonance} Hz");
    let swing = |shaping: bool| swing_after_a_traverse(0.05, &[format!("effects.input_shaping = {shaping}"), format!("profile.shaper_frequency_hz = {resonance:?}")]);
    let (plain, shaped) = (swing(false), swing(true));
    assert!(plain > 2e-6, "the plain move hardly swings: {plain} m");
    assert!(shaped < 0.1 * plain, "{shaped} m left of {plain} m");
}

/// The shaper that follows the pose. The machine reports the lower mode of two masses in a row: the carriage
/// (4 kg) on the belt, kb = EA / L1 + EA / L3, and the Z axis with payload (7 kg) on its guide, kz. At the end
/// of the traverse, x = 0.9 m:
/// z = 0.05 m: kb = 5e5 / 1.25 + 5e5 / 0.45 = 1.511e6 N/m, kz = 1.902e5 N/m. With s = (kb + kz) / 4 + kz / 7 =
///   4.525e5 and p = kb kz / 28 = 1.026e10, w^2 = (s - sqrt(s^2 - 4 p)) / 2 = 2.394e4: 24.6 Hz.
/// z = 0.15 m: kb = 5e5 / 1.35 + 5e5 / 0.55 = 1.280e6 N/m, kz = 1 / (1 / 2e7 + 0.15^2 / 1.2e4) = 5.195e5 N/m,
///   s = 5.240e5, p = 2.374e10, w^2 = 5.010e4: 35.6 Hz.
/// A shaper fixed at 35.6 Hz suits the higher pose, but at 24.6 Hz its three copies are only 24.6 / 35.6 =
/// 0.69 half periods apart and leave cos^2(pi / 2 * 0.69) = 0.22 of the swing. The shaper that takes the
/// frequency from the machine leaves about a tenth at both heights: what is left (about 1e-5 m at z = 0.15 m) hardly
/// changes with the velocity gain, the plain swing does (1.15e-4 m at 0.08 N m s/rad, 9.6e-5 m at the config's 0.04),
/// so 0.077 of it is left at 0.08 and 0.102 at 0.04; 0.11 is allowed.
#[test]
fn a_shaper_that_follows_the_height_leaves_about_a_tenth_at_both_heights() {
    let m = machine(ENTRY.config(), &[]);
    for (z, by_hand) in [(0.05, 24.63), (0.15, 35.62)] {
        let told = m.resonance_hz(&[0.9, z]).unwrap();
        assert!((told - by_hand).abs() < 0.05, "{told} Hz at z = {z}, by hand {by_hand} Hz");
    }
    // The rigid model has no leaning Z axis, and neither has the full one with the effect off.
    assert!(machine(ENTRY.config(), &["effects.tool_flex = false"]).resonance_hz(&[0.9, 0.05]).is_none());
    assert!(rigid(&[]).resonance_hz(&[0.9, 0.05]).is_none());

    let fixed = m.resonance_hz(&[0.9, 0.15]).unwrap();
    let swing = |z: f64, shaping: bool, follows: bool| {
        swing_after_a_traverse(z, &[format!("effects.input_shaping = {shaping}"), format!("effects.shaper_follows_pose = {follows}"), format!("profile.shaper_frequency_hz = {fixed:?}")])
    };
    for (z, fixed_is_enough) in [(0.15, true), (0.05, false)] {
        let (plain, fixed, following) = (swing(z, false, false), swing(z, true, false), swing(z, true, true));
        println!("z = {z} m: swing {plain:.3e} m plain, {fixed:.3e} m with the shaper fixed at 35.6 Hz, {following:.3e} m with the shaper following");
        assert!(plain > 2e-6, "z = {z}: the plain move hardly swings: {plain} m");
        assert!(following < 0.11 * plain, "z = {z}: following, {following} m left of {plain} m");
        assert!(if fixed_is_enough { fixed < 0.11 * plain } else { fixed > 0.15 * plain }, "z = {z}: fixed, {fixed} m left of {plain} m");
    }

    // A move that changes the height changes the shaper's period on the way. Its weights still add up to
    // one, so the setpoint ends on the target exactly.
    let mut cfg = ENTRY.config();
    for e in ["effects.input_shaping = true", "effects.shaper_follows_pose = true", "start.pose = [0.1, 0.05]"] {
        cfg.set_text(e).unwrap();
    }
    let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    sim.command(sim_components::Command::MoveTo(vec![0.9, 0.3])).unwrap();
    while sim.control.busy() {
        sim.step();
    }
    sim.run(0.1, |_| {});
    let c = &sim.control;
    assert!((c.set_pos[0] - 0.9).abs() < 1e-15 && (c.set_pos[1] - 0.3).abs() < 1e-15 && c.set_vel == [0.0, 0.0] && c.set_acc == [0.0, 0.0], "{:?}", c.set_pos);
}

/// Guide loads and the radial load on the pulley shafts against a hand calculation. 0.1 N m more on both
/// motors is 50 N in X, which would accelerate the loss-free rigid machine (11 kg and 4.25 kg of motors) at
/// 3.279 m/s^2. The Z axis with payload (7 kg) then pulls sideways with 22.95 N, at z = 0.05 m on an arm of
/// 0.25 m: on guide blocks 0.12 m apart that is 2 * 22.95 * 0.25 / 0.12 = 95.63 N per block pair. The X guide
/// carries that and the weight of 11 kg, 107.87 N; the Z guide carries it and the side force.
/// At rest at (0.5, 0.05) the three belt sections are 0.85 m, 2.5 m and 0.85 m long and the middle one holds
/// half the weight of the Z axis more, 34.32 N. The belt keeps its length, so the outer ones are at
/// 300 - 34.32 * 2.5 / 4.2 = 279.57 N, and each pulley shaft is pulled by 2 * 279.57 + 34.32 = 593.46 N.
#[test]
fn guide_loads_and_shaft_loads_match_the_hand_calculation() {
    let value = |m: &TGantry, state: &[f64], torque: &[f64], id: &str| {
        let mut values = Vec::new();
        m.signals(state, torque, &mut values);
        values[m.signal_infos().iter().position(|s| s.id == id).unwrap()]
    };
    let m = rigid(&["effects.belt_mass = false"]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&[0.5, 0.05], &mut state, &mut hold);
    assert!((value(&m, &state, &hold, "axis.x.guide_load") - 11.0 * G).abs() < 1e-9 && value(&m, &state, &hold, "axis.z.guide_load").abs() < 1e-9);
    let pushed = [hold[0] + 0.1, hold[1] + 0.1];
    let (x, z) = (value(&m, &state, &pushed, "axis.x.guide_load"), value(&m, &state, &pushed, "axis.z.guide_load"));
    assert!((x - (107.87 + 95.63)).abs() < 0.01 && (z - (22.95 + 95.63)).abs() < 0.01, "X guide {x} N, Z guide {z} N");
    // The rigid model's belt stays at pretension.
    assert_eq!(value(&m, &state, &hold, "pulley.a.radial_load"), 600.0);

    let m = machine(lossless_config(), &["effects.belt_mass = false"]);
    m.equilibrium(&[0.5, 0.05], &mut state, &mut hold);
    for id in ["pulley.a.radial_load", "pulley.b.radial_load"] {
        let radial = value(&m, &state, &hold, id);
        assert!((radial - 593.46).abs() < 0.02, "{id}: {radial} N");
    }
}

/// One idler as mass at the belt: J / r^2 = 2e-6 / 0.012^2 kg.
const M_IDLER: f64 = 2.0e-6 / (0.012 * 0.012);
/// Both motors and pulleys as mass on one axis, without belt mass: 2 (J_pulley + i^2 J_rotor) / r^2.
const M_MOTORS: f64 = 2.0 * (1.0e-4 + 25.0 * 0.3e-4) / (0.02 * 0.02);

/// Idlers against a hand calculation. Belt passes the two upper carriage idlers at vz, the two lower ones
/// at 2 vx + vz and 2 vx - vz and the Z-bottom idler at 2 vx. In an X move three idlers turn at twice the
/// carriage speed: 3 * 2^2 = 12 idler masses and 3 * 2 = 6 friction forces. In a Z move four turn at the
/// speed of the Z axis: 4 masses, 4 friction forces. One friction force is 0.002 N m / 0.012 m = 1/6 N.
#[test]
fn idlers_add_the_hand_calculated_mass_and_drag() {
    let edit = ["effects.idlers = true", "idler.friction_nm = 0.002", "effects.belt_mass = false"];
    let m = rigid(&edit);
    let (mut state, mut hold, mut d) = (Vec::new(), [0.0; 2], [0.0; N_RIGID]);
    m.equilibrium(&[0.5, 0.25], &mut state, &mut hold);
    // 0.1 N m more on both motors is 50 N in X; 0.05 N m more on A and less on B is 25 N in Z.
    m.derivatives(&state, &[hold[0] + 0.1, hold[1] + 0.1], &mut d);
    assert!((d[2] - 50.0 / (11.0 + M_MOTORS + 12.0 * M_IDLER)).abs() < 1e-9 && d[3].abs() < 1e-9, "{d:?}");
    m.derivatives(&state, &[hold[0] + 0.05, hold[1] - 0.05], &mut d);
    assert!((d[3] - 25.0 / (7.0 + M_MOTORS + 4.0 * M_IDLER)).abs() < 1e-9 && d[2].abs() < 1e-9, "{d:?}");
    // Coasting under the holding torques, only the bearings brake: 1 N in X, 2/3 N in Z.
    state[2] = 0.5;
    m.derivatives(&state, &hold, &mut d);
    assert!((d[2] + 1.0 / (11.0 + M_MOTORS + 12.0 * M_IDLER)).abs() < 1e-9 && d[3].abs() < 1e-9, "{d:?}");
    (state[2], state[3]) = (0.0, 0.3);
    m.derivatives(&state, &hold, &mut d);
    assert!((d[3] + (2.0 / 3.0) / (7.0 + M_MOTORS + 4.0 * M_IDLER)).abs() < 1e-9 && d[2].abs() < 1e-9, "{d:?}");
    // The feedforward knows the mass.
    let (mut with, mut without) = ([0.0; 2], [0.0; 2]);
    m.feedforward_torque(&[0.5, 0.25], &[8.0, 0.0], &mut with);
    rigid(&["effects.belt_mass = false"]).feedforward_torque(&[0.5, 0.25], &[8.0, 0.0], &mut without);
    assert!((with[0] - without[0] - 12.0 * M_IDLER * 8.0 * 0.02 / 10.0).abs() < 1e-12);

    // The elastic model: the carriage alone (11 kg with Z axis and payload) carries the idlers.
    let mut all = vec!["effects.frame_compliance = false", "effects.tool_flex = false"];
    all.extend(edit);
    let m = machine(lossless_config(), &all);
    let mut d = [0.0; N_ELASTIC];
    m.equilibrium(&[0.5, 0.25], &mut state, &mut hold);
    state[V + X] = 0.5;
    m.derivatives(&state, &hold, &mut d);
    assert!((d[V + X] + 1.0 / (11.0 + 12.0 * M_IDLER)).abs() < 1e-6, "{}", d[V + X]);
    (state[V + X], state[V + Z]) = (0.0, 0.3);
    m.derivatives(&state, &hold, &mut d);
    assert!((d[V + Z] + (2.0 / 3.0) / (7.0 + 4.0 * M_IDLER)).abs() < 1e-6, "{}", d[V + Z]);
}

/// Cable chain against a hand calculation at two positions. The chain is fixed at x = 0, so 0.3 m + x / 2
/// of it moves with the carriage: 0.5 kg/m * 0.35 m = 0.175 kg at x = 0.1 m, 0.5 * 0.75 = 0.375 kg at 0.9 m.
/// Accelerating at 8 m/s2 takes 1.4 N and 3.0 N more. At 0.5 m/s the chain drags with 3 N, and picking up
/// 0.25 kg per metre takes another 0.25 * 0.5^2 / 2 N.
#[test]
fn cable_chain_adds_the_hand_calculated_force_at_two_positions() {
    let m = rigid(&["effects.cable_chain = true", "cable_chain.drag_n = 3.0", "effects.belt_mass = false"]);
    let plain = rigid(&["effects.belt_mass = false"]);
    for (x, chain) in [(0.1, 0.175), (0.9, 0.375)] {
        let (mut with, mut without) = ([0.0; 2], [0.0; 2]);
        m.feedforward_torque(&[x, 0.25], &[8.0, 0.0], &mut with);
        plain.feedforward_torque(&[x, 0.25], &[8.0, 0.0], &mut without);
        // Force to motor torque: r / (2 i) per motor.
        let extra_force = (with[0] - without[0]) * 10.0 / 0.02;
        assert!((extra_force - chain * 8.0).abs() < 1e-9, "x = {x}: {extra_force} N");
        // The model answers that torque (it holds the Z axis too) with that acceleration.
        let (mut state, mut hold, mut d) = (Vec::new(), [0.0; 2], [0.0; N_RIGID]);
        m.equilibrium(&[x, 0.25], &mut state, &mut hold);
        m.derivatives(&state, &with, &mut d);
        assert!((d[2] - 8.0).abs() < 1e-9 && d[3].abs() < 1e-9, "x = {x}: {d:?}");
        state[2] = 0.5;
        m.derivatives(&state, &hold, &mut d);
        assert!((d[2] + (3.0 + 0.25 * 0.25 / 2.0) / (11.0 + M_MOTORS + chain)).abs() < 1e-9, "x = {x}: {}", d[2]);
    }
}

/// Process force against a hand calculation. While the cycle dwells the tool is pushed with 10 N in X and
/// 20 N in Z. The motors hold against it: A with r / (2 i) (Fx + Fz) = 0.002 m * 30 N = 0.06 N m less, B with
/// r / (2 i) (Fx - Fz) = 0.02 N m more. Friction would take a share of a force this small, so it is off.
#[test]
fn process_force_changes_the_holding_torque_by_the_hand_calculated_amount() {
    let dwell_torque = |process: bool| {
        let mut cfg = Config::parse(&ENTRY.default_config.replace("dwell_s = 0.1", "dwell_s = 1.0")).unwrap();
        for effect in ["guide_friction", "motor_friction", "gearbox_efficiency", "torque_ripple", "tooth_meshing"] {
            cfg.set_text(&format!("effects.{effect} = false")).unwrap();
        }
        cfg.set_text(&format!("effects.process_force = {process}")).unwrap();
        cfg.set_text("process.force_x_n = 10.0").unwrap();
        let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
        let index = ["a", "b"].map(|id| sim.signal_index(&format!("motor.{id}.torque")).unwrap());
        sim.command(sim_components::Command::StartCycle).unwrap();
        // Mean torque over the second half of the first dwell.
        let (mut sum, mut n, mut sample) = ([0.0; 2], 0.0, Vec::new());
        loop {
            sim.step();
            match sim.control.dwell_progress() {
                Some(done) if done > 0.5 => {
                    sim.sample(&mut sample);
                    sum = [sum[0] + sample[index[0]], sum[1] + sample[index[1]]];
                    n += 1.0;
                }
                None if n > 0.0 => return [sum[0] / n, sum[1] / n],
                _ => assert!(sim.time() < 10.0, "the cycle never dwells"),
            }
        }
    };
    let (with, without) = (dwell_torque(true), dwell_torque(false));
    assert!((with[0] - without[0] + 0.06).abs() < 0.02 * 0.06, "motor A: {} N m", with[0] - without[0]);
    assert!((with[1] - without[1] - 0.02).abs() < 0.02 * 0.06, "motor B: {} N m", with[1] - without[1]);
}

/// Hard stops against a hand calculation. Both motors push the carriage into the stop at x = 0 with 0.1 N m
/// each, which is 2 * 0.1 * 5 / 0.02 = 50 N; a stop of 1e6 N/m gives way by 50 um. Against the upper Z stop
/// 0.05 N m on each motor is 25 N and 25 um. Friction would hold part of the force, so it is off.
#[test]
fn a_carriage_pushed_into_the_hard_stop_rests_at_the_hand_calculated_depth() {
    let m = machine(ENTRY.config(), &["effects.guide_friction = false", "effects.motor_friction = false", "effects.torque_ripple = false", "effects.tooth_meshing = false"]);
    for (pose, push, axis, depth) in [([0.0, 0.25], [-0.1, -0.1], X, -50e-6), ([0.5, 0.5], [0.05, -0.05], Z, 25e-6)] {
        let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
        m.equilibrium(&pose, &mut state, &mut hold);
        let torque = [hold[0] + push[0], hold[1] + push[1]];
        let mut rk = Rk4::new(N_ELASTIC);
        for _ in 0..(2.0 / DT) as usize {
            advance(&m, &mut rk, &mut state, &torque, DT, 4);
        }
        let beyond = state[axis] - pose[axis];
        assert!((beyond - depth).abs() < 0.01 * depth.abs(), "axis {axis}: {beyond} m beyond the stroke, by hand {depth} m");
        assert!(state[V + axis].abs() < 1e-6, "axis {axis} still moves at {} m/s", state[V + axis]);
    }
}

/// Holding brakes against a hand calculation. Disabled, each motor has to hold m_z g r / (2 i) =
/// 7 * 9.80665 * 0.02 / 10 = 0.137 N m. A brake of 0.15 N m holds the Z axis. One of 0.12 N m slips with its
/// torque: the two brakes carry 2 * 0.12 * 5 / 0.02 = 60 N of the 68.6 N weight, and the rest accelerates the
/// Z axis and both motors, a = 8.65 N / (7 + 4.25) kg = 0.77 m/s2, so it falls 96 mm in half a second. The same in
/// the elastic and in the rigid model (`effects.belt_stretch = false`), where the brakes act on the rotors as they do
/// in the elastic one.
#[test]
fn holding_brakes_hold_the_z_axis_when_they_are_strong_enough() {
    use sim_components::Command;
    for elastic in [true, false] {
        let disabled = |brake_torque: f64| {
            let mut cfg = lossless_config();
            for e in ["effects.holding_brake = true", "effects.belt_mass = false", &format!("motor.brake_torque_nm = {brake_torque:?}"), &format!("effects.belt_stretch = {elastic}")] {
                cfg.set_text(e).unwrap();
            }
            let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
            sim.command(Command::Disable).unwrap();
            sim.run(0.5, |_| {});
            sim
        };
        let z = |sim: &sim_components::Sim| sim.state[Z];
        let mut held = disabled(0.15);
        assert!((z(&held) - 0.35).abs() < 1e-4, "elastic {elastic}: held at {} m", z(&held));
        // Nothing moves a disabled machine; enabled again, the drives hold where it stands.
        assert_eq!(held.command(Command::MoveTo(vec![0.5, 0.2])).unwrap_err().key, "error-disabled");
        held.command(Command::Enable).unwrap();
        held.run(0.3, |_| {});
        assert!((z(&held) - 0.35).abs() < 1e-4 && held.command(Command::MoveTo(vec![0.5, 0.2])).is_ok());

        let fell = 0.35 - z(&disabled(0.12));
        let by_hand = 0.5 * (7.0 * G - 2.0 * 0.12 * 5.0 / 0.02) / (7.0 + M_MOTORS) * 0.5 * 0.5;
        assert!((fell - by_hand).abs() < 0.03 * by_hand, "elastic {elastic}: fell {fell} m, by hand {by_hand} m");
        println!("elastic {elastic}: fell {fell} m, by hand {by_hand} m");
    }
}

/// Fitting: both motors move both axes, yet the fit tells the axes apart. The measured machine has a
/// carriage and a payload 1 kg heavier each (2 kg more for X, 1 kg more for Z and 9.81 N more to hold)
/// and 1.3 times the Coulomb friction; its trace, with noise, is fitted against the unchanged config.
#[test]
fn the_fit_tells_the_two_axes_apart() {
    let mut truth = ENTRY.config();
    for (path, value) in [("mass.carriage_kg", 5.0), ("mass.payload_kg", 3.0), ("friction_x.coulomb_n", 19.5), ("friction_x.stiction_n", 26.0), ("friction_z.coulomb_n", 13.0), ("friction_z.stiction_n", 18.2), ("motor.friction_coulomb_nm", 0.026)] {
        truth.set_text(&format!("{path} = {value:?}")).unwrap();
    }
    // One axis at a time, each there and back: moving both at once leaves too little to tell them apart.
    let mut sim = sim_components::Sim::new(&ENTRY, &truth).unwrap();
    let mut trace = sim_components::trace::Trace::default();
    for target in [[0.6, 0.35], [0.6, 0.1], [0.1, 0.1], [0.1, 0.35]] {
        sim.command(sim_components::Command::MoveTo(target.to_vec())).unwrap();
        for _ in 0..1300 {
            trace.record(&sim);
            sim.run(1e-3, |_| {});
        }
    }
    let mut noise = sim_core::testkit::Noise(1);
    for m in &mut trace.motors {
        for row in 0..m.torque.len() {
            m.torque[row] += 0.005 * noise.next();
            m.speed[row] += 0.1 * noise.next();
            m.angle[row] += 1e-5 * noise.next();
        }
    }
    let fits = sim_components::identify::fit_rigid(&ENTRY, &ENTRY.config(), &trace).unwrap();
    // Per axis: more mass in kg, the factor on the Coulomb friction, more constant force in N.
    for (fit, expected) in fits.iter().zip([[2.0, 1.3, 0.0], [1.0, 1.3, G]]) {
        let fit = fit.as_ref().unwrap();
        let found = [fit.fitted[0] - fit.config[0], fit.fitted[2] / fit.config[2], fit.fitted[3] - fit.config[3]];
        let errors = [fit.error[0], fit.error[2] / fit.config[2], fit.error[3]];
        for q in 0..3 {
            assert!((found[q] - expected[q]).abs() < 3.0 * errors[q], "value {q}: {} ± {}, expected {}", found[q], errors[q], expected[q]);
        }
        assert!(errors[0] < 0.2 && errors[1] < 0.05 && errors[2] < 0.6, "{errors:?}");
    }
}

/// The circle of `path_error_on_a_circle_matches_the_hand_calculation`, given as two full turns (after a `line`
/// to the start, which a full turn as the first piece needs) and timed by
/// the path planner instead of by hand. With both axes limited to 0.25 m/s the planner runs the circle of
/// radius 50 mm at that speed, which is the 5 rad/s of the hand calculation, so the path error and the
/// distance behind the setpoint are the same as there.
#[test]
fn a_circle_timed_by_the_planner_gives_the_hand_calculated_path_error() {
    let mut cfg = lossless_config();
    for e in [
        "effects.belt_stretch = false",
        "effects.velocity_feedforward = false",
        "effects.encoder_resolution = false",
        "effects.velocity_filter = false",
        "effects.torque_filter = false",
        "effects.drive_delay = false",
        "start.pose = [0.55, 0.25]",
        "axis.x.max_velocity_m_s = 0.25",
        "axis.z.max_velocity_m_s = 0.25",
    ] {
        cfg.set_text(e).unwrap();
    }
    let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    let turns = sim_components::path::parse("line,0.55,0.25\narc,0.55,0.25,0.5,0.25,ccw\narc,0.55,0.25,0.5,0.25,ccw").unwrap();
    let points = sim.plan_path(&turns, false).unwrap();
    // Two turns at 0.25 m/s take 2.51 s; the planner adds the two ramps, and every pose is on the circle.
    let (radius, w) = (0.05, 0.25 / 0.05);
    let steady = 2.0 * std::f64::consts::TAU / w;
    let duration = points[points.len() - 1].0;
    assert!(duration > steady && duration < steady + 0.4, "{duration} s");
    assert!(points.iter().all(|(_, p)| ((p[0] - 0.5).hypot(p[1] - 0.25) - radius).abs() < 1e-12));
    let fastest = points.windows(2).map(|w| (w[1].1[0] - w[0].1[0]).hypot(w[1].1[1] - w[0].1[1]) / 1e-3).fold(0.0, f64::max);
    assert!((fastest - 0.25).abs() < 1e-4, "{fastest} m/s");

    let load = sim_components::analysis::analyse_trajectory(&mut sim, points).unwrap();
    let gain = 1.0 / (1.0 + (w / 50.0f64).powi(2)).sqrt();
    let (off_path, behind) = (radius * (1.0 - gain), radius * (w / 50.0) * gain);
    assert!((load.path_error_peak - off_path).abs() < 0.1 * off_path, "{} m off the path, by hand {off_path} m", load.path_error_peak);
    assert!((load.tracking_error_peak - behind).abs() < 0.1 * behind, "{} m behind, by hand {behind} m", load.tracking_error_peak);
}

/// Time-optimal timing of a path: a line, an arc of radius 0.1 m and a line, tangent to each other.
/// (a) It is shorter than the plan within the axis limits: 1.43 s against 1.83 s. (The plan without a jerk
/// limit took 0.69 s and asked the motors for more than they have on a tight arc.) (b) No pose uses more
/// than the plan promises, by differences (`plan_use`, margin 0.1 %): the motors' share of the torque-speed
/// curve (`profile.torque_reserve`, 0.8), their rated speed and the jerk limits of the axes. (c) A limit is
/// active, used to 97 % or more, at over 60 % of the poses. With the config's motor that is the jerk at 57 %
/// and the rated speed at 10 %; the torque stays at 0.16 of its share, because at 100 m/s^3 the lines are
/// too short to reach it. With a motor of 0.6 N m and a jerk limit of 2000 m/s^3 it is the torque at 30 %
/// of the poses, the speed at 32 % and the jerk at 4 %. The other poses are on the arc, where the planner
/// shares speed, acceleration and jerk out between the directions along the path and towards the centre and
/// so uses none in full, and the steady cycles on either side of a junction. (d) Run on the full model with
/// all default effects, the plan stays on the torque-speed curve.
#[test]
fn the_time_optimal_plan_is_shorter_keeps_a_limit_active_and_stays_on_the_torque_speed_curve() {
    let pieces = sim_components::path::parse("line,0.5,0.35
arc,0.6,0.25,0.5,0.25,cw
line,0.6,0.05").unwrap();
    // The plan of a config, its duration, the share of its poses with a limit active and the most each limit is used.
    let plan = |edits: &[&str]| {
        let mut cfg = ENTRY.config();
        for e in edits {
            cfg.set_text(e).unwrap();
        }
        let sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
        let points = sim.plan_path(&pieces, true).unwrap();
        let used = plan_use(&sim, &cfg, &points);
        let active = |limit: usize| used.iter().filter(|u| u[limit] >= 0.97).count() as f64 / used.len() as f64;
        let share = used.iter().filter(|u| u.iter().any(|u| *u >= 0.97)).count() as f64 / used.len() as f64;
        let most = most_used(&used);
        println!("{edits:?}: {} s, a limit active at {share} of the poses (torque {}, speed {}, jerk {}), most used {most:?}", points[points.len() - 1].0, active(0), active(1), active(2));
        assert!(share > 0.6 && most.iter().all(|m| *m <= 1.001), "{edits:?}: a limit active at {share} of the poses, used to {most:?}");
        (sim, points, most)
    };
    let (mut sim, points, most) = plan(&[]);
    assert!(most[1] > 0.999 && most[2] > 0.999, "speed and jerk used to {most:?}");
    let (duration, within_axis_limits) = (points[points.len() - 1].0, sim.plan_path(&pieces, false).unwrap());
    let slow = within_axis_limits[within_axis_limits.len() - 1].0;
    assert!(duration < 0.8 * slow, "{duration} s as fast as the motors allow, {slow} s within the axis limits");
    let most = plan(&["drive.peak_torque_nm = 0.6", "axis.x.max_jerk_m_s3 = 2000.0", "axis.z.max_jerk_m_s3 = 2000.0"]).2;
    assert!(most.iter().all(|m| *m > 0.999), "a small motor: torque, speed and jerk used to {most:?}");

    let load = sim_components::analysis::analyse_trajectory(&mut sim, points).unwrap();
    for m in &load.motors {
        assert!(m.utilisation < 0.6 && m.peak_speed < 1.03 * 314.0, "utilisation {}, {} rad/s", m.utilisation, m.peak_speed);
    }
    assert!(load.path_error_peak < 1e-3, "{} m off the path", load.path_error_peak);
}

/// With `profile.path_tolerance_m` both planners time the same path: the one with transition curves, whose arc
/// has moved in by the tolerance. Each plan's poses are nowhere further from the given line, arc and line
/// than the 0.1 mm the simulator reports, reach that on the arc, and end on the target; within the axis
/// limits the path takes 1.59 s where it took 1.83 s, and as fast as the motors allow 1.20 s.
#[test]
fn both_planners_time_the_path_with_its_transition_curves() {
    let pieces = sim_components::path::parse("line,0.5,0.35\narc,0.6,0.25,0.5,0.25,cw\nline,0.6,0.05").unwrap();
    let mut cfg = ENTRY.config();
    let given = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    assert_eq!(given.path_deviation(&pieces, false).unwrap(), 0.0);
    cfg.set_text("profile.path_tolerance_m = 1.0e-4").unwrap();
    let sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    let reported = sim.path_deviation(&pieces, false).unwrap();
    assert!((reported - 1e-4).abs() < 1e-9, "{reported} m");
    let off = |p: &[f64]| if p[0] <= 0.5 { (p[1] - 0.35).abs() } else if p[1] <= 0.25 { (p[0] - 0.6).abs() } else { ((p[0] - 0.5).hypot(p[1] - 0.25) - 0.1).abs() };
    let mut durations = Vec::new();
    for time_optimal in [false, true] {
        let points = sim.plan_path(&pieces, time_optimal).unwrap();
        let furthest = points.iter().map(|p| off(&p.1)).fold(0.0, f64::max);
        // The setpoint the path starts from is a few micrometres from the configured start pose.
        assert!(furthest <= reported + 1e-5 && furthest > 0.99 * reported, "time-optimal {time_optimal}: {furthest} m off the given path");
        let last = &points[points.len() - 1];
        assert!((last.1[0] - 0.6).abs() < 1e-9 && (last.1[1] - 0.05).abs() < 1e-9, "ends at {:?}", last.1);
        durations.push(last.0);
    }
    let slow = given.plan_path(&pieces, false).unwrap();
    let slow = slow[slow.len() - 1].0;
    assert!(durations[0] < 0.9 * slow && durations[1] < 0.8 * durations[0], "{durations:?} s with transition curves, {slow} s without");
}

/// How much each pose of a plan uses of what the time-optimal planner promises, from the planned poses by
/// differences over one cycle, at rest before and after: the largest motor torque of the rigid model over
/// `profile.torque_reserve` times the torque-speed curve, motor speed over the rated speed, and axis jerk
/// over the axis's limit.
fn plan_use(sim: &sim_components::Sim, cfg: &Config, points: &[(f64, Vec<f64>)]) -> Vec<[f64; 3]> {
    let (c, reserve, rated) = (1e-3, cfg.f64("profile.torque_reserve").unwrap(), cfg.f64("drive.rated_speed_rad_s").unwrap());
    let pose = |i: isize| &points[i.clamp(0, points.len() as isize - 1) as usize].1;
    let mut all = Vec::new();
    for i in 0..points.len() as isize {
        let mut used = [0f64; 3];
        let (a, p, b, b2) = (pose(i - 1), pose(i), pose(i + 1), pose(i + 2));
        let acc = [(b[0] - 2.0 * p[0] + a[0]) / (c * c), (b[1] - 2.0 * p[1] + a[1]) / (c * c)];
        let (mut torque, mut ahead, mut here) = ([0.0; 2], [0.0; 2], [0.0; 2]);
        sim.machine.feedforward_torque(p, &acc, &mut torque);
        sim.machine.inverse(b, &mut ahead);
        sim.machine.inverse(p, &mut here);
        for k in 0..2 {
            let speed = (ahead[k] - here[k]) / c;
            let (low, high) = sim.control.torque_curve(speed);
            let jerk = (b2[k] - 3.0 * b[k] + 3.0 * p[k] - a[k]) / (c * c * c);
            used = [used[0].max(torque[k] / (reserve * if torque[k] >= 0.0 { high } else { low })), used[1].max(speed.abs() / rated), used[2].max(jerk.abs() / sim.machine.axes()[k].max_jerk)];
        }
        all.push(used);
    }
    all
}

/// The belt's tension is a limit of the time-optimal plan. Default config, jerk 300 m/s^3, a line along X from
/// (0.7, 0.35) to (0.95, 0.35), and `ratings.belt_max_tension_n` = 400 N. The rigid model: the belt pulls the
/// carriage, Z axis and payload, m = 4 + 5 + 2 = 11 kg, F_x = m a, and holds the Z axis, F_z = 7 kg * g = 68.65 N; the
/// sections are L1 = 0.3 + x + z, L2 = 2.6 - 2 z, L3 = 1.3 - x + z long (4.2 m together at z = 0.35), and with
/// sum (T_i - P) L_i = 0 section 3 carries T = 300 - 68.65 * 0.95 / 4.2 + m a (L1 + 0.95) / 4.2 accelerating. At rest
/// that is 284.47 N; at x = 0.95 (L1 = 1.6, the pose of the line where it rises most) by 6.681 N per m/s^2, and with
/// `profile.tension_reserve` = 0.8 the plan may use 0.8 * (400 - 284.47) = 92.42 N of it: a = 13.84 m/s^2. The plan
/// without the belt as a limit goes to 19.3 m/s^2 and fails `belt-tension` on the full model (425 N); the plan with it
/// accelerates at 13.84 m/s^2 within 0.5 %, its rigid tension reaches the 376.9 N the plan allows within 3 % (where it
/// accelerates, x = 0.7 to 0.8, the rise per m/s^2 is a little less), and the full model, friction and belt vibration
/// on top, passes `belt-tension` within 3 % of the 400 N.
#[test]
fn the_time_optimal_plan_keeps_the_belt_within_its_tension_limit() {
    let pieces = sim_components::path::parse("line,0.95,0.35").unwrap();
    let run = |rated: f64| {
        let mut cfg = ENTRY.config();
        for e in ["axis.x.max_jerk_m_s3 = 300.0", "axis.z.max_jerk_m_s3 = 300.0", "start.pose = [0.7, 0.35]"] {
            cfg.set_text(e).unwrap();
        }
        cfg.set_text(&format!("ratings.belt_max_tension_n = {rated:?}")).unwrap();
        let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
        let points = sim.plan_path(&pieces, true).unwrap();
        // The most acceleration along X and the most tension of the rigid model, by differences over the planned poses.
        let (c, mut most_acc, mut most_tension) = (1e-3, 0f64, 0f64);
        for w in points.windows(3) {
            let acc = [(w[2].1[0] - 2.0 * w[1].1[0] + w[0].1[0]) / (c * c), 0.0];
            let mut tensions = Vec::new();
            sim.machine.rigid_tensions(&w[1].1, &acc, &mut tensions);
            most_acc = most_acc.max(acc[0].abs());
            most_tension = tensions.iter().fold(most_tension, |m, t| m.max(*t));
        }
        let load = sim_components::analysis::analyse_trajectory(&mut sim, points).unwrap();
        let mut judged = cfg.clone();
        judged.set_text("ratings.belt_max_tension_n = 400.0").unwrap();
        let verdict = sim_components::analysis::verdicts(&judged, &load).unwrap().into_iter().find(|v| v.check == "belt-tension").unwrap();
        println!("limit {rated} N: {most_acc} m/s^2, rigid {most_tension} N, simulated {} N", verdict.value);
        (most_acc, most_tension, verdict)
    };
    // The tension law of the rigid model, by hand, at rest and accelerating at x = 0.9.
    let machine = (ENTRY.build)(&ENTRY.config()).unwrap();
    let mut at = Vec::new();
    machine.rigid_tensions(&[0.9, 0.35], &[10.0, 0.0], &mut at);
    // At x = 0.9: L1 = 1.55, L3 = 0.75.
    let weight = 7.0 * 9.80665;
    assert!((at[2] - (300.0 - weight * 0.95 / 4.2 + 110.0 * 1.55 / 4.2 + 110.0 * 0.95 / 4.2)).abs() < 1e-9, "{at:?}");
    assert!((at[0] - (300.0 - weight * 0.95 / 4.2 - 110.0 * (0.95 + 0.75) / 4.2)).abs() < 1e-9, "{at:?}");
    let (still, room) = (300.0 - weight * 0.95 / 4.2, 0.8 * (400.0 - (300.0 - weight * 0.95 / 4.2)));
    let allowed = room / (11.0 * (1.6 + 0.95) / 4.2);
    assert!((allowed - 13.84).abs() < 5e-3 && (still + room - 376.89).abs() < 0.01, "{allowed}");
    let (old_acc, _, old) = run(1e12);
    assert!(old_acc > 1.3 * allowed && !old.ok, "{old_acc} m/s^2, {old:?}");
    let (acc, tension, new) = run(400.0);
    assert!((acc / allowed - 1.0).abs() < 0.005, "{acc} m/s^2 against {allowed}");
    assert!(tension <= (still + room) * 1.001 && tension >= 0.97 * (still + room), "{tension} N");
    assert!(new.ok && new.value >= 0.97 * 400.0, "{new:?}");
}

/// The most that any pose of `plan_use` uses of each of the three limits.
fn most_used(used: &[[f64; 3]]) -> [f64; 3] {
    used.iter().fold([0.0; 3], |most, u| [most[0].max(u[0]), most[1].max(u[1]), most[2].max(u[2])])
}

/// The third review's scenario: a line, a quarter turn of 5 mm radius and a line, as fast as the motors allow,
/// as given and with transition curves. The plan's moving mean carried the speed of the lines through the
/// arc: 10.5 times the torque-speed curve on the full model. Now every planned pose keeps the motors within
/// the reserve and the rated speed and the axes within their jerk limits (by differences, margin 1 %), the
/// path ends on its end point, and on the full model with all default effects the motors stay on the curve.
#[test]
fn a_time_optimal_plan_through_a_tight_arc_stays_within_torque_speed_and_jerk() {
    let pieces = sim_components::path::parse("line,0.5,0.35\narc,0.505,0.345,0.5,0.345,cw\nline,0.505,0.05").unwrap();
    for tolerance in [0.0, 1e-4, 1e-3] {
        let mut cfg = ENTRY.config();
        cfg.set_text(&format!("profile.path_tolerance_m = {tolerance:?}")).unwrap();
        let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
        let points = sim.plan_path(&pieces, true).unwrap();
        let (used, last) = (most_used(&plan_use(&sim, &cfg, &points)), &points[points.len() - 1]);
        assert!(used.iter().all(|u| *u <= 1.001), "tolerance {tolerance} m: torque, speed and jerk used to {used:?}");
        assert!((last.1[0] - 0.505).abs() < 1e-9 && (last.1[1] - 0.05).abs() < 1e-9, "ends at {:?}", last.1);
        let duration = last.0;
        let load = sim_components::analysis::analyse_trajectory(&mut sim, points).unwrap();
        let utilisation = load.motors.iter().map(|m| m.utilisation).fold(0.0, f64::max);
        assert!(utilisation < 1.0, "tolerance {tolerance} m: utilisation {utilisation}");
        println!("tolerance {tolerance} m: {duration} s, used {used:?}, utilisation {utilisation}");
    }
}

/// 200 paths of two to five lines, arcs and splines that leave each other tangentially, inside the workspace:
/// arcs of 2 mm to 300 mm radius, pieces of 1 mm to 400 mm, splines of one to three via points, each up to
/// 0.6 rad off the direction to the one before. Half of them have a path tolerance, and half a motor of
/// 0.6 N m with a jerk limit of 2000 m/s^3, so that the torque limits the plan (with the config's values the
/// jerk does). Every plan as fast as the motors allow keeps torque, speed and jerk within their limits at
/// every planned pose (`plan_use`, margin 0.1 %) and ends on the end of the path.
#[test]
fn time_optimal_plans_of_random_paths_stay_within_torque_speed_and_jerk() {
    use sim_components::path::Piece;
    let mut noise = sim_core::testkit::Noise(3);
    // The generator's own last draw: evenly spread, which the sum that `next` returns is not.
    let mut uniform = |low: f64, high: f64| {
        noise.next();
        low + (high - low) * (noise.0 >> 11) as f64 / (1u64 << 53) as f64
    };
    let (mut worst, mut planned, mut longest) = ([0f64; 3], 0, 0f64);
    while planned < 200 {
        let mut cfg = ENTRY.config();
        cfg.set_text(&format!("start.pose = [{:?}, {:?}]", uniform(0.05, 0.95), uniform(0.05, 0.45))).unwrap();
        let tolerance = if planned % 2 == 1 { 10f64.powf(uniform(-5.0, -3.0)) } else { 0.0 };
        cfg.set_text(&format!("profile.path_tolerance_m = {tolerance:?}")).unwrap();
        if planned / 2 % 2 == 1 {
            for e in ["drive.peak_torque_nm = 0.6", "axis.x.max_jerk_m_s3 = 2000.0", "axis.z.max_jerk_m_s3 = 2000.0"] {
                cfg.set_text(e).unwrap();
            }
        }
        let sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
        let (mut here, mut heading) = (sim.control.planned_pos().to_vec(), uniform(0.0, std::f64::consts::TAU));
        let inside = |p: &[f64]| p.iter().zip(sim.machine.axes()).all(|(p, a)| *p > a.min + 1e-3 && *p < a.max - 1e-3);
        let (mut pieces, mut fits) = (Vec::new(), true);
        for _ in 0..uniform(2.0, 6.0) as usize {
            let length = uniform(1e-3f64.ln(), 0.4f64.ln()).exp();
            let kind = uniform(0.0, 3.0);
            if kind < 1.0 {
                here = vec![here[0] + length * heading.cos(), here[1] + length * heading.sin()];
                pieces.push(Piece::Line(here.clone()));
            } else if kind < 2.0 {
                // Via points of a spline, each `length / count` on from the last; the piece after the spline
                // leaves in a direction of its own, which the spline then ends with.
                let count = uniform(1.0, 4.0) as usize;
                for _ in 0..count {
                    heading += uniform(-0.6, 0.6);
                    here = vec![here[0] + length / count as f64 * heading.cos(), here[1] + length / count as f64 * heading.sin()];
                    fits &= inside(&here);
                    pieces.push(Piece::Spline(here.clone()));
                }
                heading += uniform(-0.6, 0.6);
            } else {
                let (radius, turn) = (uniform(2e-3f64.ln(), 0.3f64.ln()).exp(), if uniform(0.0, 1.0) < 0.5 { 1.0 } else { -1.0 });
                let sweep = (length / radius).min(4.5);
                // The centre is to the left of the heading for a turn counter-clockwise.
                let centre = vec![here[0] - turn * radius * heading.sin(), here[1] + turn * radius * heading.cos()];
                let on = |angle: f64| vec![centre[0] + turn * radius * angle.sin(), centre[1] - turn * radius * angle.cos()];
                fits &= (1..16).all(|i| inside(&on(heading + turn * sweep * i as f64 / 16.0)));
                heading += turn * sweep;
                here = on(heading);
                pieces.push(Piece::Arc { to: here.clone(), centre, clockwise: turn < 0.0 });
            }
            fits &= inside(&here);
        }
        if !fits {
            continue;
        }
        let points = sim.plan_path(&pieces, true).unwrap();
        let (used, last) = (most_used(&plan_use(&sim, &cfg, &points)), &points[points.len() - 1]);
        assert!(used.iter().all(|u| *u <= 1.001), "path {planned}, tolerance {tolerance} m, {pieces:?}: torque, speed and jerk used to {used:?}");
        assert!((last.1[0] - here[0]).hypot(last.1[1] - here[1]) < 1e-9, "path {planned}: ends at {:?}, not at {here:?}", last.1);
        for (worst, used) in worst.iter_mut().zip(used) {
            *worst = worst.max(used);
        }
        (planned, longest) = (planned + 1, longest.max(last.0));
    }
    println!("most used of torque, speed and jerk: {worst:?}; longest plan {longest} s");
}

/// The holding brake gives its torque and no more, also when it engages at speed. By hand: a rotor of
/// 0.3 kg cm2 under a brake of 1 N m slows down with 1 / 0.3e-4 = 33333 rad/s2 more than without it.
/// In a traverse at 0.8 m/s both motors turn at 200 rad/s. Two brakes of 1 N m are 2 * 1 * 5 / 0.02 = 500 N
/// at the belt, which stop 11 kg and the motors' 4.25 kg with 32.8 m/s2: over 0.8^2 / (2 * 32.8) = 9.8 mm.
/// Gravity is off, because brakes that slip carry nothing of the Z axis's weight, and the frame is rigid,
/// because a frame that gives way takes a share of the carriage's momentum. As a friction contact with
/// bristle damping the brake gave many times its torque here (244 to 153 rad/s in one step on the EXCT-30),
/// and above 178 rad/s its state was too stiff for the step.
#[test]
fn a_brake_that_engages_at_speed_stops_the_axis_with_its_torque_and_no_more() {
    use sim_components::Command;
    let mut cfg = lossless_config();
    for e in ["effects.holding_brake = true", "effects.belt_mass = false", "effects.gravity = false", "effects.belt_damping = true", "effects.frame_compliance = false", "start.pose = [0.1, 0.25]"] {
        cfg.set_text(e).unwrap();
    }
    let mut m = machine(cfg.clone(), &[]);
    let (mut state, mut hold, mut free, mut braked) = (Vec::new(), [0.0; 2], [0.0; N_ELASTIC], [0.0; N_ELASTIC]);
    m.equilibrium(&[0.5, 0.25], &mut state, &mut hold);
    state[V + PA] = 244.0;
    m.derivatives(&state, &hold, &mut free);
    m.set_brake_engaged(true);
    m.derivatives(&state, &hold, &mut braked);
    assert!((braked[V + PA] - free[V + PA] + 1.0 / 0.3e-4).abs() < 1e-6, "{} rad/s2", braked[V + PA] - free[V + PA]);
    for speed in [-400.0, -200.0, -1e-3, 0.0, 1e-3, 200.0, 400.0] {
        for deflection in [-0.02, -1e-3, 0.0, 5e-4, 0.02] {
            assert!(m.brake.torque(m.brake_engaged, speed, deflection).0.abs() <= 1.0, "{} N m at {speed} rad/s, {deflection} rad", m.brake.torque(m.brake_engaged, speed, deflection).0);
        }
    }

    let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    sim.command(Command::MoveTo(vec![0.9, 0.25])).unwrap();
    while sim.control.set_vel[0] < 0.8 - 1e-9 {
        sim.step();
    }
    sim.run(0.05, |_| {});
    let (x, v, w) = (sim.state[X], sim.state[V + X], sim.state[V + PA]);
    assert!((v - 0.8).abs() < 0.01 && (w - 200.0).abs() < 2.0, "{v} m/s, {w} rad/s");
    sim.command(Command::Disable).unwrap();
    // The fastest a rotor can lose speed: the brake and, wound up against it, the gearbox at its full twist.
    let mut last = w;
    sim.run(0.5, |s| {
        let now = s.state[V + PA];
        assert!(now.is_finite() && (now - last).abs() < 10.0, "{last} to {now} rad/s in one step at {} s", s.time());
        last = now;
    });
    let (stopped_after, by_hand) = (sim.state[X] - x, v * v / (2.0 * 500.0 / (11.0 + M_MOTORS)));
    assert!((stopped_after - by_hand).abs() < 0.05 * by_hand, "stopped after {stopped_after} m, by hand {by_hand} m");
    assert!(sim.state[V + X].abs() < 1e-3 && sim.state[V + PA].abs() < 0.1, "still moving: {} m/s, {} rad/s", sim.state[V + X], sim.state[V + PA]);
}

/// The reported tool position counts the frame's sway: the workpiece stands on the floor, the carriage rides
/// on the frame. x of the state is the carriage along the beam; the tool is at x + q + the lean of the Z axis.
/// In the default cycle the frame sways by some tenths of a millimetre, which path error, tracking error
/// and the settling verdict now see.
#[test]
fn the_tool_position_includes_the_frame_deflection() {
    let m = machine(lossless_config(), &[]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&[0.5, 0.25], &mut state, &mut hold);
    (state[Q], state[V + Q]) = (1e-4, 0.01);
    let (mut pos, mut vel) = ([0.0; 2], [0.0; 2]);
    m.tool_state(&state, &mut pos, &mut vel);
    assert!((pos[0] - 0.5001).abs() < 1e-15 && (vel[0] - 0.01).abs() < 1e-15 && pos[1] == 0.25 && vel[1] == 0.0, "{pos:?}, {vel:?}");

    let mut sim = sim_components::Sim::new(&ENTRY, &ENTRY.config()).unwrap();
    let [position, frame, lean] = ["axis.x.position", "frame.deflection", "axis.x.tool_deflection"].map(|id| sim.signal_index(id).unwrap());
    sim.command(sim_components::Command::StartCycle).unwrap();
    let (mut sample, mut sway) = (Vec::new(), 0f64);
    sim.run(4.0, |s| {
        s.sample(&mut sample);
        assert!((sample[position] - (s.state[X] + sample[frame] + sample[lean])).abs() < 1e-15);
        sway = sway.max(sample[frame].abs());
    });
    assert!(sway > 1e-4, "the frame sways by {sway} m only");
}

/// The optimiser switches input shaping on where it is what a faster cycle needs. A traverse low down,
/// where the Z axis leans most easily, with a dwell at both ends; the axes start at 0.5 m/s and 0.25 m/s.
/// At 1.5 times the speed the tool is 0.19 mm off its target in the second half of the dwell, beyond the
/// 0.18 mm this config allows, and nothing else fails: with the shaper it is 0.07 mm off, and the cycle is
/// a quarter shorter than at the start although every stop waits one period of the shaper. The optimiser
/// used to try the shaper only at unchanged limits, where it makes the cycle longer and never was taken.
#[test]
fn the_optimiser_switches_input_shaping_on_where_a_faster_cycle_fails_settling() {
    use sim_components::analysis::{analyse_cycle, optimise_cycle, verdicts};
    let text = ENTRY.default_config;
    let cycle = "[[cycle.segment]]\ntarget = [0.9, 0.05]\ndwell_s = 0.1\npayload = true\n\n[[cycle.segment]]\ntarget = [0.1, 0.05]\ndwell_s = 0.1\npayload = true\n";
    let mut cfg = Config::parse(&format!("{}{cycle}", &text[..text.find("[[cycle.segment]]").unwrap()])).unwrap();
    for e in ["start.pose = [0.1, 0.05]", "axis.x.max_velocity_m_s = 0.5", "axis.z.max_velocity_m_s = 0.25", "ratings.settling_tolerance_m = 1.8e-4"] {
        cfg.set_text(e).unwrap();
    }
    let found = optimise_cycle(&ENTRY, &cfg, 12).unwrap();
    assert!(found.after.duration < 0.8 * found.before.duration, "{} s to {} s", found.before.duration, found.after.duration);
    assert!(found.changes.contains(&("effects.input_shaping".to_string(), "false".to_string(), "true".to_string())), "{:?}", found.changes);
    // The settings it reports pass, and the same limits without the shaper fail on settling alone.
    let mut faster = cfg.clone();
    for (path, _, after) in &found.changes {
        faster.set_text(&format!("{path} = {after}")).unwrap();
    }
    let failing = |cfg: &Config| -> Vec<&'static str> {
        let load = analyse_cycle(&mut sim_components::Sim::new(&ENTRY, cfg).unwrap()).unwrap();
        verdicts(cfg, &load).unwrap().iter().filter(|v| !v.ok).map(|v| v.check).collect()
    };
    assert!(failing(&faster).is_empty());
    faster.set_text("effects.input_shaping = false").unwrap();
    assert_eq!(failing(&faster), ["settling"]);
}

/// The optimiser lowers the lifts of the cycle to the clearance. Default cycle (blending and shaping off), with
/// `profile.min_clearance_m` = 0.1 m: both lifts go from 0.35 m to 0.05 + 0.1 = 0.15 m, the first thing it tries (two
/// runs: the cycle as configured, then this). Each of the four Z moves (lift and lower at pick and place) is jerk-limited
/// with v = 0.4 m/s, a = 6 m/s^2, j = 100 m/s^3 (v j = 40 >= a^2 = 36, and both distances above v (v/a + a/j) = 0.0507 m,
/// so the speed is reached): t = d/v + v/a + a/j, 0.8767 s for 0.3 m and 0.3767 s for 0.1 m. The cycle shortens by
/// 4 * 0.2 m / 0.4 m/s = 2.000 s, within 5 ms (each segment arrives on a whole profile cycle).
#[test]
fn the_optimiser_lowers_the_lifts_to_the_clearance_and_saves_the_hand_calculated_time() {
    use sim_components::analysis::optimise_cycle;
    let mut cfg = ENTRY.config();
    cfg.set_text("profile.min_clearance_m = 0.1").unwrap();
    let found = optimise_cycle(&ENTRY, &cfg, 2).unwrap();
    assert_eq!(found.lifts.len(), 2, "{:?}", found.lifts);
    for lift in &found.lifts {
        assert!(lift.before == 0.35 && (lift.after - 0.15).abs() < 1e-12 && lift.via.is_none(), "{lift:?}");
    }
    assert_eq!(found.lifts.iter().map(|l| l.segments).collect::<Vec<_>>(), [[1, 2, 3], [4, 5, 0]]);
    let t = |d: f64| d / 0.4 + 0.4 / 6.0 + 6.0 / 100.0;
    let saved = 4.0 * (t(0.3) - t(0.1));
    assert!((saved - 2.0).abs() < 1e-12);
    let shorter = found.before.duration - found.after.duration;
    assert!((shorter - saved).abs() < 5e-3, "{} s to {} s: {shorter} s saved", found.before.duration, found.after.duration);
    // "Apply" writes the cycle with the lower lifts; it is the one the optimiser ran.
    let (path, _, after) = found.changes.iter().find(|c| c.0 == "cycle.segment").unwrap();
    cfg.set_text(&format!("{path} = {after}")).unwrap();
    assert_eq!(cfg.f64s("start.pose").unwrap().len(), 2);
    let targets: Vec<Vec<f64>> = cfg.array("cycle.segment").unwrap().iter().map(|s| s["target"].as_array().unwrap().iter().map(|v| v.as_float().unwrap()).collect()).collect();
    assert_eq!(targets.iter().filter(|t| (t[1] - 0.15).abs() < 1e-12).count(), 4, "{targets:?}");
    let load = sim_components::analysis::analyse_cycle(&mut sim_components::Sim::new(&ENTRY, &cfg).unwrap()).unwrap();
    assert!((load.duration - found.after.duration).abs() < 1e-12);
}

/// A lift rounded through a via point: the segment with `via` runs the planned path up from the pick, through the
/// via point and down to the place (`control::via_pieces`), without a stop at the corners, and arrives where its
/// target is before the dwell. A via point outside the stroke is refused by the config.
#[test]
fn a_cycle_segment_runs_its_path_through_via_points() {
    let cfg = |via: &str| {
        let text = ENTRY.default_config;
        let cycle = format!("[[cycle.segment]]
target = [0.1, 0.05]
dwell_s = 0.1
payload = true

[[cycle.segment]]
target = [0.9, 0.05]
dwell_s = 0.1
payload = false
via = {via}
");
        let mut cfg = Config::parse(&format!("{}{cycle}", &text[..text.find("[[cycle.segment]]").unwrap()])).unwrap();
        cfg.set_text("start.pose = [0.1, 0.05]").unwrap();
        cfg
    };
    let rounded = cfg("[[0.1, 0.15], [0.5, 0.3], [0.9, 0.15]]");
    let mut sim = sim_components::Sim::new(&ENTRY, &rounded).unwrap();
    sim.command(sim_components::Command::StartCycle).unwrap();
    let (mut highest, mut stopped_above, mut arrived) = (0f64, false, false);
    sim.run(6.0, |s| {
        let c = &s.control;
        highest = highest.max(c.set_pos[1]);
        // Between the two vertical lines the plan never stands still.
        if c.set_pos[0] > 0.11 && c.set_pos[0] < 0.89 && c.set_vel.iter().all(|v| v.abs() < 1e-9) {
            stopped_above = true;
        }
        if c.dwell_progress().is_some() && c.cycle_segment() == Some(1) {
            arrived |= (c.set_pos[0] - 0.9).abs() < 1e-9 && (c.set_pos[1] - 0.05).abs() < 1e-9 && !c.payload_attached;
        }
    });
    assert!(highest > 0.29 && highest < 0.35 && !stopped_above && arrived, "highest {highest}, stopped {stopped_above}, arrived {arrived}");
    // Outside the stroke the config is refused, naming the via points.
    let outside = sim_components::Sim::new(&ENTRY, &cfg("[[0.1, 0.15], [0.5, 9.0], [0.9, 0.15]]"));
    assert_eq!(outside.err().map(|e| e.detail().to_string()), Some("cycle.segment[1].via".to_string()));
}

/// The torque at the gearbox outputs against a hand calculation. Each pulley carries half the weight of the Z
/// axis with payload, 7 kg * 9.80665 m/s^2 * 0.02 m / 2 = 0.6865 N m, A lifting and B the other way round.
/// With a gearbox of 0.9 efficiency a motor gives 0.6865 / (5 * 0.9) N m to lift at a steady speed and holds
/// back with 0.9 * 0.6865 / 5 N m to lower; the output carries 0.6865 N m both times, which the motor torque
/// times the ratio reads a ninth too high or a tenth too low. At rest the elastic model's gear springs pass
/// on the same torque.
#[test]
fn gearbox_torque_is_the_load_at_the_pulleys_whichever_way_it_is_driven() {
    let value = |m: &TGantry, state: &[f64], torque: &[f64], id: &str| {
        let mut values = Vec::new();
        m.signals(state, torque, &mut values);
        values[m.signal_infos().iter().position(|s| s.id == id).unwrap()]
    };
    let lossy = ["effects.gearbox_efficiency = true", "gearbox.efficiency = 0.9", "effects.belt_mass = false"];
    let (m, by_hand) = (rigid(&lossy), 7.0 * G * 0.02 / 2.0);
    for (vz, motor) in [(0.3, by_hand / (5.0 * 0.9)), (-0.3, 0.9 * by_hand / 5.0)] {
        let (state, torque, mut d) = ([0.5, 0.25, 0.0, vz, 0.0, 0.0, 0.0, 0.0], [motor, -motor], [0.0; N_RIGID]);
        m.derivatives(&state, &torque, &mut d);
        let (a, b) = (value(&m, &state, &torque, "gearbox.a.output_torque"), value(&m, &state, &torque, "gearbox.b.output_torque"));
        assert!(d[2].abs() < 1e-9 && d[3].abs() < 1e-9, "vz = {vz}: {d:?}");
        assert!((a - by_hand).abs() < 1e-9 && (b + by_hand).abs() < 1e-9 && (by_hand - 0.6865).abs() < 1e-4, "vz = {vz}: {a} N m and {b} N m");
    }
    let m = machine(lossless_config(), &lossy);
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&[0.5, 0.25], &mut state, &mut hold);
    let (a, b) = (value(&m, &state, &hold, "gearbox.a.output_torque"), value(&m, &state, &hold, "gearbox.b.output_torque"));
    assert!((a - by_hand).abs() < 1e-3 * by_hand && (b + by_hand).abs() < 1e-3 * by_hand, "at rest: {a} N m and {b} N m");
}

/// The mapping from a tool pose into the scene, for drawing paths there: it is where the scene draws the
/// tool (the flange under the Z axis). By hand at (0.3, 0.1): the beam starts 0.15 m left of x = 0, so
/// 0.45 m across; in the belt plane at -0.05 m; and 0.1 - 0.5 - 0.15 m to the idler at the bottom of the Z
/// axis and 0.042 m more to the flange, -0.592 m. Moving the tool moves the point by as much.
#[test]
fn the_scene_point_of_a_tool_pose_is_where_the_scene_draws_the_tool() {
    for m in [rigid(&[]), machine(ENTRY.config(), &[])] {
        let at = m.scene_point(&[0.3, 0.1]).unwrap();
        assert!((at[0] - 0.45).abs() < 1e-12 && at[1] == -0.05 && (at[2] + 0.592).abs() < 1e-12, "{at:?}");
        for pose in [[0.0, 0.0], [0.3, 0.1], [0.5, 0.25], [1.0, 0.5], [0.9, 0.03]] {
            let (mut state, mut scene) = (Vec::new(), Scene::default());
            m.equilibrium(&pose, &mut state, &mut [0.0; 2]);
            m.scene(&state, &mut scene);
            let flange = scene.parts.iter().find(|p| p.id == "flange").unwrap().pos;
            let (mut tool, mut speed) = ([0.0; 2], [0.0; 2]);
            m.tool_state(&state, &mut tool, &mut speed);
            let mapped = m.scene_point(&tool).unwrap();
            assert!((0..3).all(|i| (mapped[i] - flange[i]).abs() < 1e-12), "at {pose:?}: mapped {mapped:?}, drawn {flange:?}");
            assert!((mapped[0] - at[0] - (pose[0] - 0.3)).abs() < 1e-12 && (mapped[2] - at[2] - (pose[1] - 0.1)).abs() < 1e-12);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Effect views

fn machine_config(edit: &[&str]) -> Config {
    let mut cfg = ENTRY.config();
    for e in edit {
        cfg.set_text(e).unwrap();
    }
    cfg
}

fn effect_signal(m: &dyn Machine, state: &[f64], torque: &[f64], id: &str) -> f64 {
    let mut values = Vec::new();
    m.signals(state, torque, &mut values);
    values[m.signal_infos().iter().position(|s| s.id == id).unwrap()]
}

/// The four parts of one motor's torque, in the order of `TORQUE_PARTS`.
fn torque_parts(m: &dyn Machine, state: &[f64], torque: &[f64], motor: &str) -> [f64; 4] {
    TORQUE_PARTS.map(|part| effect_signal(m, state, torque, &format!("motor.{motor}.torque_{part}")))
}

/// Pure inertia, by hand. Loss-free rigid model without gravity and belt mass; 0.1 N m on both motors accelerates
/// X at 2 * 0.1 * 5 / 0.02 / (11 + 4.25) = 3.27869 m/s^2 (both motors reflected: 2 (1e-4 + 25 * 0.3e-4) / 0.02^2 =
/// 4.25 kg), so each rotor at 5 / 0.02 * 3.27869 = 819.672 rad/s^2. Per motor the inertia seen is
/// J = (0.02 / 5)^2 (11 + 2 * 1e-4 / 0.02^2) / 2 + 0.3e-4 = 1.22e-4 kg m^2, and J * alpha = 0.1000 N m: all of the
/// torque is inertia, gravity and friction are zero. With 0.15 and 0.05 N m (Z as well) the parts are the torques.
#[test]
fn a_pure_inertia_move_is_all_inertia() {
    let m = rigid(&["effects.belt_mass = false", "effects.gravity = false"]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&[0.4, 0.2], &mut state, &mut hold);
    state[2] = 0.3;
    let mut d = [0.0; N_RIGID];
    m.derivatives(&state, &[0.1, 0.1], &mut d);
    let alpha = 5.0 / 0.02 * d[2];
    assert!((d[2] - 3.27869).abs() < 1e-5 && (alpha - 819.672).abs() < 1e-3, "{} m/s^2", d[2]);
    for motor in ["a", "b"] {
        let [inertia, gravity, friction, rest] = torque_parts(&m, &state, &[0.1, 0.1], motor);
        assert!((inertia - 1.22e-4 * alpha).abs() < 1e-12 && (inertia - 0.1).abs() < 1e-12, "{motor}: {inertia} N m");
        assert!(gravity == 0.0 && friction == 0.0 && rest.abs() < 1e-12, "{motor}: {gravity}, {friction}, {rest}");
    }
    let torque = [0.15, 0.05];
    for (motor, expected) in [("a", 0.15), ("b", 0.05)] {
        let parts = torque_parts(&m, &state, &torque, motor);
        assert!((parts[0] - expected).abs() < 1e-12 && parts[1..].iter().all(|p| p.abs() < 1e-12), "{motor}: {parts:?}");
    }
}

/// Holding Z, by hand: each motor holds m_z g r / (2 i) = 7 * 9.80665 * 0.02 / 10 = 0.137293 N m, A up and B down,
/// which is the gravity part of the full elastic model at rest. Over 50 ms of standing (the controller dithers on
/// the encoder counts, so one step alone is off by up to 0.5 %) the motor torque is the gravity part to within
/// 0.2 %, and inertia, friction and the rest are below 0.2 % of it. Ripple, cogging and tooth meshing are off: they
/// are part of the rest. The copper loss of that hold is R i^2 = 4 * (0.137293 / 0.45)^2 = 0.37235 W per motor.
#[test]
fn holding_z_is_all_gravity() {
    let by_hand = 7.0 * G * 0.02 / 10.0;
    assert!((by_hand - 0.137293).abs() < 1e-6);
    let sim_config = machine_config(&["effects.torque_ripple = false", "effects.tooth_meshing = false"]);
    let mut sim = sim_components::Sim::new(&ENTRY, &sim_config).unwrap();
    sim.run(0.2, |_| {});
    let (mut mean, mut torque, mut copper, steps) = ([[0.0; 4]; 2], [0.0; 2], [0.0; 2], 800.0);
    sim.run(steps * sim.dt, |sim| {
        for (k, motor) in ["a", "b"].into_iter().enumerate() {
            let parts = torque_parts(sim.machine.as_ref(), &sim.state, &sim.torque, motor);
            (0..4).for_each(|i| mean[k][i] += parts[i] / steps);
            torque[k] += sim.torque[k] / steps;
            copper[k] += sim.control.drives[k].copper_loss / steps;
        }
    });
    for (k, (motor, sign)) in [("a", 1.0), ("b", -1.0)].into_iter().enumerate() {
        let [inertia, gravity, friction, rest] = mean[k];
        assert!((gravity - sign * by_hand).abs() < 1e-12, "{motor}: {gravity} N m");
        assert!((torque[k] - gravity).abs() < 2e-3 * by_hand, "{motor}: torque {} N m, parts {inertia} {friction} {rest}", torque[k]);
        assert!([inertia, friction, rest].iter().all(|p| p.abs() < 2e-3 * by_hand), "{motor}: {inertia}, {friction}, {rest}");
        assert!((copper[k] - 0.37235).abs() < 2e-3 * 0.37235, "{motor}: {} W", copper[k]);
    }
}

/// Constant speed, by hand: on the rigid model with guide friction alone (not load dependent), X at 0.8 m/s takes
/// 15 N Coulomb and 20 * 0.8 = 16 N viscous at the X guide (the Stribeck part is 5 exp(-6400), nothing). Both
/// motors together give r / i (15 + 20 v) = 0.004 * 31 = 0.124 N m to it, and in the cruise that is all the motors
/// give beside holding Z: the Z guide's part and gravity cancel between the two motors.
#[test]
fn a_constant_speed_move_is_coulomb_plus_viscous_friction() {
    let mut cfg = lossless_config();
    cfg.set_text("effects.belt_stretch = false").unwrap();
    cfg.set_text("effects.guide_friction = true").unwrap();
    let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    sim.command(sim_components::Command::MoveTo(vec![0.9, 0.35])).unwrap();
    while sim.state[0] < 0.5 {
        sim.step();
    }
    let v = sim.state[2];
    assert!((v - 0.8).abs() < 1e-3, "{v} m/s");
    let m = sim.machine.as_ref();
    let (a, b) = (torque_parts(m, &sim.state, &sim.torque, "a"), torque_parts(m, &sim.state, &sim.torque, "b"));
    assert!((a[2] + b[2] - 0.004 * (15.0 + 20.0 * v)).abs() < 1e-9, "{} N m", a[2] + b[2]);
    assert!((a[2] + b[2] - 0.124).abs() < 1e-4);
    assert!((sim.torque[0] + sim.torque[1] - 0.124).abs() < 0.02 * 0.124, "{:?}", sim.torque);
    let split = ["coulomb", "viscous", "stiction"].map(|part| effect_signal(m, &sim.state, &sim.torque, &format!("axis.x.friction_{part}")));
    assert!((split[0] - 15.0).abs() < 1e-6 && (split[1] - 20.0 * v).abs() < 1e-9 && split[2].abs() < 1e-6, "{split:?}");
    assert!((split.iter().sum::<f64>() - effect_signal(m, &sim.state, &sim.torque, "axis.x.friction")).abs() < 1e-9);
}

/// The friction view's curves are the formula with the config's values. X guide: Fc 15 N, Fs 20 N, b 20 N s/m,
/// vs 0.01 m/s; at 0.5 m/s 15, 10 and 0 N, at 0.01 m/s 15, 0.2 and 5 / e = 1.83940 N. Z guide: Fc 10 N, Fs 14 N,
/// b 15 N s/m; at -0.2 m/s with the levels doubled -20, -3 and 0 N, at 0.005 m/s 10, 0.075 and 4 exp(-0.25) =
/// 3.11520 N.
#[test]
fn the_friction_curves_are_the_formula() {
    let m = machine(ENTRY.config(), &[]);
    assert_eq!(m.friction_contacts().iter().map(|c| c.id).collect::<Vec<_>>(), ["axis.x", "axis.z"]);
    for (contact, speed, scale, expected) in [(0, 0.5, 1.0, [15.0, 10.0, 0.0]), (0, 0.01, 1.0, [15.0, 0.2, 1.83940]), (1, -0.2, 2.0, [-20.0, -3.0, 0.0]), (1, 0.005, 1.0, [10.0, 0.075, 3.11520])] {
        let curve = m.friction_curve(contact, speed, scale);
        assert!(curve.iter().zip(&expected).all(|(a, b)| (a - b).abs() < 1e-5), "contact {contact} at {speed} m/s: {curve:?}");
    }
}
