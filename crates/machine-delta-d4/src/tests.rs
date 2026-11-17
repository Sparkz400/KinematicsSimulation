//! The D4 against hand calculations: the shaft's kinematics and cardans, the tool's torque, the arms' holding torques
//! with the shaft's weight, the telescope's friction, the tool's torsional mode, the standard suite, the cycle.

use super::*;
use sim_components::{Command, Sim};

fn d4(edits: &[&str]) -> Delta4 {
    let mut cfg = ENTRY.config();
    for e in edits {
        cfg.set_text(e).unwrap();
    }
    Delta4::new(&cfg).unwrap()
}

#[track_caller]
fn close(a: f64, b: f64, tolerance: f64, what: &str) {
    sim_core::testkit::record_check(what, b, a, tolerance * b.abs().max(1e-12));
    assert!((a - b).abs() <= tolerance * b.abs().max(1e-12), "{what}: {a} against {b} by hand");
}

/// The shaft at x = 0.3, y = 0.4, z = -0.85: from the upper joint at the shoulder plane to the lower joint 50 mm above
/// the platform, d = (0.3, 0.4, -0.8), so L = sqrt(0.09 + 0.16 + 0.64) = 0.943398 m, both joints bent by
/// beta = atan(0.5 / 0.8) = 32.0054 degrees, in the plane at azimuth atan2(0.4, 0.3) = 53.1301 degrees. Over the working
/// range (top 675 mm, sphere bottom) the shaft is 0.625 m at the top of the centre, 1.020 m at the edge's bottom, and
/// bends 43.83 degrees at the top of the edge (atan(0.6 / 0.625)): every pose the arms reach the shaft reaches too.
#[test]
fn the_shafts_length_and_cardan_angles_are_the_hand_calculated() {
    let m = d4(&[]);
    let (length, beta, azimuth) = m.shaft().geometry([0.3, 0.4, -0.85]);
    close(length, 0.943398, 1e-6, "length");
    close(beta.to_degrees(), 32.0054, 1e-5, "cardan angle");
    close(azimuth.to_degrees(), 53.1301, 1e-5, "bending plane");
    let (top, edge_top, edge_bottom) = (
        m.shaft().geometry([0.0, 0.0, -0.675]),
        m.shaft().geometry([0.6, 0.0, -0.675]),
        m.shaft().geometry([0.6, 0.0, -0.875]),
    );
    close(top.0, 0.625, 1e-12, "shortest");
    close(edge_bottom.0, 1.020110, 1e-6, "longest");
    close(edge_top.1.to_degrees(), 43.8309, 1e-5, "largest bend");
    let mut tried = 0;
    for i in 0..=12 {
        for j in 0..=12 {
            for k in 0..=8 {
                let p = [-0.6 + 0.1 * i as f64, -0.6 + 0.1 * j as f64, -1.061 + 0.386 * k as f64 / 8.0];
                if m.arms().unreachable(&p, 0.0).is_none() {
                    tried += 1;
                    assert_eq!(m.unreachable(&[p[0], p[1], p[2], 1.0], 0.0), None, "{p:?}");
                }
            }
        }
    }
    assert!(tried > 500, "{tried} poses");
    // Beyond the telescope's stroke or the joints' bend the target is refused with its reason.
    assert_eq!(
        d4(&["shaft.max_length_m = 0.9"]).unreachable(&[0.3, 0.4, -0.85, 0.0], 0.0),
        Some("error-delta-shaft-stroke")
    );
    assert_eq!(
        d4(&["shaft.max_cardan_angle_deg = 30.0"]).unreachable(&[0.3, 0.4, -0.85, 0.0], 0.0),
        Some("error-delta-cardan-angle")
    );
}

/// One cardan joint bent by beta passes its input's angle phi (from the bending plane) on as tan chi = tan phi / cos beta:
/// at beta = 32.0054 degrees and phi = 30 degrees, chi = atan(0.57735 / 0.848) = 34.2486 degrees. The pair with its
/// yokes in phase gives the angle back, to 1e-12 rad over a full turn: the tool turns as the gearbox's output.
///
/// So at the pose above with the tool at c = 0.3 rad turning at 10 rad/s, the motor turns at 8 * 10 = 80 rad/s and the
/// tool at 10 rad/s, whatever the bend, while the shaft between the joints turns at
/// 10 cos beta / (1 - sin^2 beta cos^2 phi) with phi = c - 53.1301 degrees = -0.627295 rad: cos beta = 0.848, sin^2 beta
/// = 0.25 / 0.89 = 0.280899, cos phi = 0.809618, so 10 * 0.848 / (1 - 0.280899 * 0.655481) = 10.3937 rad/s; over a turn
/// it swings between 10 cos beta = 8.480 and 10 / cos beta = 11.792 rad/s.
#[test]
fn the_tool_turns_with_the_motor_through_the_cardan_pair_and_the_shaft_between_unevenly() {
    let beta = 32.0054f64.to_radians();
    close(cardan(beta, 30f64.to_radians()).to_degrees(), 34.2486, 1e-5, "one joint");
    for k in 0..=72 {
        let phi = -std::f64::consts::PI + std::f64::consts::TAU * k as f64 / 72.0;
        assert!((cardan_back(beta, cardan(beta, phi)) - phi).abs() < 1e-12, "{phi}");
        // The single joint's speed ratio is the derivative of its angle.
        let h = 1e-6;
        close(
            cardan_speed_ratio(beta, phi),
            (cardan(beta, phi + h) - cardan(beta, phi - h)) / (2.0 * h),
            1e-7,
            "speed ratio",
        );
    }
    close(cardan_speed_ratio(beta, 0.0), 1.0 / beta.cos(), 1e-12, "fastest");
    close(cardan_speed_ratio(beta, std::f64::consts::FRAC_PI_2), beta.cos(), 1e-12, "slowest");
    for elastic in [false, true] {
        let m = d4(&[&format!("effects.joint_elasticity = {elastic}")]);
        let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
        m.equilibrium(&[0.3, 0.4, -0.85, 0.3], &mut state, &mut torque);
        m.set_tool_velocity(&mut state, &[0.0, 0.0, 0.0, 10.0]);
        let (mut pos, mut vel) = ([0.0; 4], [0.0; 4]);
        m.motor_state(&state, &mut pos, &mut vel);
        close(vel[3], 80.0, 1e-12, "motor speed");
        close(pos[3], 2.4, 1e-12, "motor angle");
        m.tool_state(&state, &mut pos, &mut vel);
        close(vel[3], 10.0, 1e-12, "tool speed");
        let mut signals = Vec::new();
        m.base_signals(&state, &torque, &mut signals);
        let at = |id: &str| signals[m.signal_infos().iter().position(|s| s.id == id).unwrap()];
        close(at("shaft.speed"), 10.3937, 1e-5, "shaft speed");
        close(at("shaft.cardan_angle").to_degrees(), 32.0054, 1e-5, "cardan angle signal");
        close(at("shaft.length"), 0.943398, 1e-6, "length signal");
    }
}

/// The tool's motor torque for 100 rad/s^2 of the tool's turn, rigid and loss-free: J_c = tool 2e-3 + shaft spin 8e-5 +
/// payload 5e-3 = 7.08e-3 kg m^2, at the motor 7.08e-3 * 100 / 8 = 0.0885 N m, plus motor 4's rotor and its gearbox's
/// input ([motor.4], [gearbox.4]: 3.5e-5 + 8.8e-6) * 8 * 100 = 0.03504 N m: 0.12354 N m. Without the payload
/// 0.0260 + 0.03504 = 0.06104 N m.
/// Given that torque the model turns the tool at 100 rad/s^2, and the arms' torques do not change.
#[test]
fn the_tool_motors_torque_for_a_turn_is_the_hand_calculated() {
    let m = d4(&[
        "effects.joint_friction = false",
        "effects.motor_friction = false",
        "effects.torque_ripple = false",
        "effects.gearbox_efficiency = false",
        "effects.joint_elasticity = false",
    ]);
    close(m.tool_inertia(), 7.08e-3, 1e-12, "J_c");
    let pose = [0.1, -0.1, -0.8, 0.5];
    let (mut moving, mut still) = ([0.0; 4], [0.0; 4]);
    m.feedforward_torque(&pose, &[0.0, 0.0, 0.0, 100.0], &mut moving);
    m.feedforward_torque(&pose, &[0.0; 4], &mut still);
    close(moving[3] - still[3], 0.12354, 1e-9, "tool motor torque");
    assert!((0..3).all(|k| moving[k] == still[k]));
    let (mut state, mut hold) = (Vec::new(), [0.0; 4]);
    m.equilibrium(&pose, &mut state, &mut hold);
    let mut rates = vec![0.0; m.state_len()];
    m.derivatives(&state, &moving, &mut rates);
    close(rates[m.n3 + 1], 100.0, 1e-9, "tool acceleration");
    let mut m = m;
    m.set_payload_attached(false);
    m.feedforward_torque(&pose, &[0.0, 0.0, 0.0, 100.0], &mut moving);
    close(moving[3], 0.06104, 1e-9, "without the payload");
}

/// The arms at rest at the centre, 100 mm below the top, hold the D3's torques plus the shaft's weight share: half of
/// its 1.4 kg on the platform, 0.7 kg, through dz/dtheta = -0.13343 m/rad per arm (the D3's hand calculation:
/// (e + L_a cos t) L_a sin t / (z + L_a sin t) - L_a cos t at t = 0.0577 degrees, a third), each gearbox
/// 0.7 * 9.80665 * (-0.13343) = -0.9160 N m more, each motor -0.04580 N m more. The tool's motor holds nothing. Without
/// effects.shaft_mass the D4's arms hold what the D3's do.
#[test]
fn the_arms_hold_the_d3s_torques_plus_the_shafts_weight_share() {
    let z = -0.775;
    let edits = ["effects.torque_ripple = false"];
    let d3 = machine_delta_d3::Delta::new(&{
        let mut cfg = ENTRY.config();
        cfg.set_text(edits[0]).unwrap();
        cfg
    })
    .unwrap();
    let (mut state, mut d3_torque) = (Vec::new(), [0.0; 3]);
    d3.equilibrium(&[0.0, 0.0, z], &mut state, &mut d3_torque);
    // dz/dtheta of one arm when all three turn together, by halving the leg's equation as the D3's tests do.
    let (e, la, lb) = (d3.legs().e, d3.legs().la, d3.legs().lb);
    let f = |t: f64| (e + la * t.cos()).powi(2) + (z + la * t.sin()).powi(2) - lb * lb;
    let (mut lo, mut hi) = (-60f64.to_radians(), 90f64.to_radians());
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        if f(lo) * f(mid) <= 0.0 { hi = mid } else { lo = mid }
    }
    let t = 0.5 * (lo + hi);
    let dz = (-la * t.cos() + (e + la * t.cos()) * la * t.sin() / (z + la * t.sin())) / 3.0;
    close(dz, -0.13343, 1e-4, "dz/dtheta per arm");
    let by_hand = 0.7 * 9.80665 * dz / 20.0;
    for (with, extra) in [(true, by_hand), (false, 0.0)] {
        let m = d4(&[edits[0], &format!("effects.shaft_mass = {with}")]);
        let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
        m.equilibrium(&[0.0, 0.0, z, 0.7], &mut state, &mut torque);
        for k in 0..3 {
            close(torque[k] - d3_torque[k], extra, 1e-6, "the shaft's share");
        }
        assert_eq!(torque[3], 0.0);
    }
    assert!((by_hand + 0.04580).abs() < 1e-5, "{by_hand}");
}

/// The telescope's friction at the pose of the shaft test, the platform sinking at 1 m/s: the shaft lengthens at
/// u . v = 0.8 / 0.943398 = 0.848 m/s, against which the spline's 5 N (smoothed over 5 mm/s, all of it here) and
/// 2 N s/m * 0.848 = 1.696 N push: 6.696 N along the shaft, 5.678 W. Given the torques that hold it there, the platform
/// at rest feels none.
#[test]
fn the_telescopes_friction_is_the_hand_calculated() {
    let m = d4(&[]);
    let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
    m.equilibrium(&[0.3, 0.4, -0.85, 0.0], &mut state, &mut torque);
    assert_eq!(m.telescope_power(&state), 0.0);
    m.set_tool_velocity(&mut state, &[0.0, 0.0, -1.0, 0.0]);
    close(m.telescope_power(&state), 5.678194, 1e-5, "sliding power");
    let drag = m.drag.as_ref().unwrap();
    let force = drag.force([0.3, 0.4, -0.85], [0.0, 0.0, -1.0]);
    let u = [0.3 / 0.943398, 0.4 / 0.943398, -0.8 / 0.943398];
    for k in 0..3 {
        close(force[k], -6.695997 * u[k], 1e-5, "force along the shaft");
    }
    assert!(d4(&["effects.telescope_friction = false"]).drag.is_none());
}

/// The tool's torsion with the motor held: the gearbox's 8250.592 N m/rad in series with the shaft's 500 N m/rad,
/// 1 / (1/8250.592 + 1/500) = 471.43 N m/rad, against J_c = 7.08e-3 kg m^2: sqrt(471.43 / 7.08e-3) / 2 pi = 41.0688 Hz,
/// without the payload (2.08e-3) 75.77 Hz; damping 0.075 / (2 sqrt(471.43 * 7.08e-3)) = 2.05 % of critical. It is among
/// the modes, the arms' first; the rigid model has none of it.
#[test]
fn the_tools_torsional_mode_is_the_hand_calculated() {
    let m = d4(&[]);
    let mode = m.tool_mode().unwrap();
    close(mode.hz, 41.0688, 1e-5, "with the payload");
    close(mode.damping, 0.020526, 1e-4, "damping ratio");
    let mut empty = d4(&[]);
    empty.set_payload_attached(false);
    close(empty.tool_mode().unwrap().hz, 75.7700, 1e-5, "without");
    assert!(m.modes(&[0.0, 0.0, -0.8, 0.0]).iter().any(|x| x.hz == mode.hz));
    assert!(d4(&["effects.joint_elasticity = false"]).tool_mode().is_none());
    close(d4(&["effects.shaft_torsion = false"]).tool_joint().gearbox.stiffness, 8250.592, 1e-12, "the gearbox alone");
}

#[test]
fn the_d4_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ENTRY);
}

/// The default cycle, the Adept cycle with the tool turned by 90 degrees on each traverse: the tool stands at 90
/// degrees during the pick (to within 1.5 mrad), and every verdict passes, "settling" with 0.37 mm (its distance to the
/// target counts the turn's error in rad with the position's in m, as on every machine), the tool's gearbox peaking at
/// 2.2 N m of its 29 (motor 4's own gearbox, `[ratings.4]`). With the catalogue's backlash on the arms' NRB080 (9 arcmin) and the tool's NRB060 (10 arcmin)
/// "settling" fails alone, 1.4 mm: 0.6 mm of it the arms' sag under the platform's weight (the D3's finding) and 1 mrad
/// the tool resting in its backlash.
#[test]
fn the_default_cycle_turns_the_tool_and_passes_its_verdicts() {
    let cfg = ENTRY.config();
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    sim.command(Command::StartCycle).unwrap();
    sim.run(0.45, |_| {});
    let mut out = Vec::new();
    sim.sample(&mut out);
    let c = out[sim.signal_index("axis.c.position").unwrap()];
    assert!((c - std::f64::consts::FRAC_PI_2).abs() < 1.5e-3, "{c} rad at the pick");
    let verdicts = |cfg: &Config| {
        let load = sim_components::analysis::analyse_cycle_with(&mut Sim::new(&ENTRY, cfg).unwrap(), false).unwrap();
        sim_components::analysis::verdicts(cfg, &load).unwrap()
    };
    let all = verdicts(&cfg);
    assert!(all.iter().all(|v| v.ok), "{:?}", all.iter().filter(|v| !v.ok).collect::<Vec<_>>());
    // The tool's gearbox judged by the common check of motor 4, against its own [ratings.4].
    let tool = all.iter().find(|v| v.check == "gearbox-peak" && v.motor == Some(3)).unwrap();
    assert!(tool.value > 1.0 && tool.value < 5.0 && tool.limit == 29.0, "{tool:?}");
    let mut catalogue = cfg.clone();
    catalogue.set_text("gearbox.backlash_rad = 2.618e-3").unwrap();
    catalogue.set_text("gearbox.4.backlash_rad = 2.909e-3").unwrap();
    let failed: Vec<_> = verdicts(&catalogue).into_iter().filter(|v| !v.ok).map(|v| v.check).collect();
    assert_eq!(failed, ["settling"]);
}

/// The tool's place and its turn are judged apart: metres and radians are not added. The default cycle turns the tool
/// by 90 degrees on each traverse; with those turns taken out of the targets (the same cycle, c = 0 all along) the
/// place's path, tracking and dwell errors are the same (the tool's own motor turns it, the arms see none of it: 1.53 mm
/// of path error both ways, where the place and the turn's 3.3 mrad counted together gave 3.53 mm), and the turn's own
/// errors, in rad, are there with the turn and nil without.
#[test]
fn the_tools_place_error_is_the_same_with_and_without_its_turn() {
    let turned = ENTRY.config();
    let still = Config::parse(&ENTRY.default_config.replace(", 1.5707963267948966]", ", 0.0]")).unwrap();
    let analysed = |cfg: &Config| {
        sim_components::analysis::analyse_cycle_with(&mut Sim::new(&ENTRY, cfg).unwrap(), false).unwrap()
    };
    let (a, b) = (analysed(&turned), analysed(&still));
    for (what, x, y) in [
        ("path", a.path_error_peak, b.path_error_peak),
        ("tracking", a.tracking_error_peak, b.tracking_error_peak),
        ("dwell", a.dwell_error_peak, b.dwell_error_peak),
    ] {
        assert!(x > 1e-5 && (x - y).abs() < 1e-3 * x, "{what}: {x} m with the turn, {y} m without");
    }
    assert!(a.path_angle_error_peak > 1e-3 && a.dwell_angle_error_peak > 0.0, "{a:?}");
    assert_eq!((b.path_angle_error_peak, b.dwell_angle_error_peak), (0.0, 0.0));
}

/// Motor 4 reads its own tables ([motor.4], [gearbox.4], [drive.4], [ratings.4] over the shared ones, as
/// `catalog::motor_config` puts them): the tool's joint turns a rotor of 3.5e-5 + 8.8e-6 kg m^2 behind ratio 8 at 97 %,
/// its drive gives 3.75 N m (the arms' 15 N m), and its gearbox's limit is 29 N m against the arms' 192 N m. Before, the
/// tool's joint had the arms' 1FK2105-4AF and its gearbox checks the arms' NRB080.
#[test]
fn the_tool_motors_own_tables_are_read() {
    let m = d4(&[]);
    close(m.tool_joint().rotor_inertia, 4.38e-5, 1e-12, "rotor and gearbox input");
    close(m.tool_joint().ratio(), 8.0, 1e-12, "ratio");
    close(m.tool_joint().gearbox.efficiency, 0.97, 1e-12, "efficiency");
    close(m.arms().axes().len() as f64, 3.0, 0.0, "arms");
    let sim = Sim::new(&ENTRY, &ENTRY.config()).unwrap();
    close(sim.control.drive_params_of(3).peak_torque, 3.75, 1e-12, "tool drive's peak");
    close(sim.control.drive_params_of(0).peak_torque, 15.0, 1e-12, "arm drive's peak");
    let own = motor_config(&ENTRY.config(), "4").unwrap();
    close(own.f64("ratings.gearbox_max_torque_nm").unwrap(), 29.0, 0.0, "tool gearbox limit");
    close(
        motor_config(&ENTRY.config(), "1").unwrap().f64("ratings.gearbox_max_torque_nm").unwrap(),
        192.0,
        0.0,
        "arms",
    );
}

/// The tool motor's values are the catalogue's written out: naming the same parts for motor 4
/// (`catalog::with_motor_part`: 1FK2104-4AF at 400 V, NRB060-04-8, 6SL3210-5HE10-4UF0) over the shared ones gives the
/// same numbers, the backlash apart (an estimate: 1 arcmin against the catalogue's 10). The velocity gain it retunes is
/// 0.25 times the square root of the torque per tool acceleration over the ratio, (7.08e-3 / 8 + 4.38e-5 * 8) / 8 =
/// 1.54425e-4 against the shared motor's (7.08e-3 / 20 + 2.106e-4 * 20) / 20 = 2.283e-4, 0.676413, times the rotors'
/// inertia, 4.38e-5 / 2.106e-4 = 0.207977: 0.25 * sqrt(0.676413 * 0.207977) = 0.093768, the one [drive.4] explains it
/// left (27.5 degrees, 4.1 dB); the shipped 0.05 lies below it.
#[test]
fn the_tool_motors_values_are_the_catalogues() {
    let shared = ENTRY.config();
    // The config with no tables of motor 4's own, as before it had them; then the catalogue's parts named for it.
    let mut all: toml::Table = shared.to_text().unwrap().parse().unwrap();
    for table in ["motor", "gearbox", "drive", "ratings"] {
        all.get_mut(table).unwrap().as_table_mut().unwrap().remove("4");
    }
    let base = Config::parse(&toml::to_string(&all).unwrap()).unwrap();
    let named = sim_components::catalog::with_motor_part(
        &ENTRY,
        &base,
        "4",
        "1FK2104-4AF",
        "400v",
        "NRB060-04-8",
        "6SL3210-5HE10-4UF0",
    )
    .unwrap();
    let (mine, catalogue) = (motor_config(&shared, "4").unwrap(), motor_config(&named, "4").unwrap());
    for path in [
        "motor.rotor_inertia_kgm2",
        "motor.torque_constant_nm_a",
        "motor.rated_current_a",
        "motor.thermal_time_constant_s",
        "motor.max_current_a",
        "motor.torque_constant_at_max_current_nm_a",
        "gearbox.ratio",
        "gearbox.efficiency",
        "gearbox.stiffness_nm_rad",
        "gearbox.inertia_at_input_kgm2",
        "drive.peak_torque_nm",
        "drive.rated_torque_nm",
        "drive.rated_speed_rad_s",
        "drive.no_load_speed_rad_s",
        "ratings.gearbox_rated_torque_nm",
        "ratings.gearbox_max_torque_nm",
        "ratings.drive_rated_current_a",
        "ratings.drive_max_current_a",
        "ratings.braking_resistor_continuous_w",
        "ratings.braking_resistor_peak_w",
    ] {
        close(mine.f64(path).unwrap(), catalogue.f64(path).unwrap(), 1e-6, path);
    }
    let gain = "drive.velocity_kp_nm_s_rad";
    close(catalogue.f64(gain).unwrap(), 0.093768, 1e-5, "retuned velocity gain");
    assert!(mine.f64(gain).unwrap() < catalogue.f64(gain).unwrap());
}

/// An arm's own `[gearbox.2]` is read by that arm alone: its ratio 10 against the shared 20 halves its motor's angle
/// for the same arm angle (motor angle = ratio x arm angle, the pose being the same), the other two arms unchanged.
#[test]
fn an_arms_own_gearbox_table_is_read_by_that_arm_alone() {
    let pose = [0.1, 0.05, -0.8, 0.0];
    let motors = |own: bool| {
        let mut all: toml::Table = ENTRY.config().to_text().unwrap().parse().unwrap();
        if own {
            let mut t = toml::Table::new();
            t.insert("ratio".into(), toml::Value::Float(10.0));
            all.get_mut("gearbox").unwrap().as_table_mut().unwrap().insert("2".into(), toml::Value::Table(t));
        }
        let cfg = Config::parse(&toml::to_string(&all).unwrap()).unwrap();
        let (m, mut out) = (Delta4::new(&cfg).unwrap(), [0.0; 4]);
        m.inverse(&pose, &mut out);
        out
    };
    let (shared, own) = (motors(false), motors(true));
    assert_eq!((own[0], own[2]), (shared[0], shared[2]));
    assert!((own[1] - 0.5 * shared[1]).abs() < 1e-12 * shared[1].abs(), "{own:?} against {shared:?}");
}

/// The tool motor and its gearbox above the base's centre are collision boxes where `scene` draws them: the pair at
/// (0, 0, 0.16) and (0, 0, 0.065), whatever the pose.
#[test]
fn the_motor_and_gearbox_above_the_base_are_collision_bodies() {
    let m = Delta4::new(&ENTRY.config()).unwrap();
    let (mut state, mut torque) = (Vec::new(), vec![0.0; 4]);
    m.equilibrium(&[0.0, 0.0, -0.75, 0.0], &mut state, &mut torque);
    let boxes = m.collision_bodies(&state);
    for at in [[0.0, 0.0, 0.16], [0.0, 0.0, 0.065]] {
        assert!(boxes.iter().any(|b| b.centre == at), "no body at {at:?}");
    }
}

/// Friction feedforward from the machine's own model (`steady_friction`), by hand, with the platform on the axis at
/// z = -0.85 m going down at 0.5 m/s and the tool turning at 5 rad/s. Each arm turns at t' = 1.128251 rad/s (the D3's
/// geometry, see its test of the same name): the joint 0.3 tanh(t' / 0.01) + 0.05 t' = 0.356413 N m. The telescope
/// lengthens at 0.5 m/s and slides at 5 tanh(100) + 2 x 0.5 = 6 N, upward on the platform; each arm moves the platform
/// by dz/dt_k = z' / (3 t') = -0.147721 m/rad (the three alike), so it feels 6 x -0.147721 = -0.886328 N m against its
/// turn: 1.242740 N m through the gearbox. Each arm holds G = 12.045 N m of weight (platform, payload, shaft and arms,
/// pulling the arm down, the way it turns), so its gearbox passes 1.242740 - 12.045 N m, is driven back, and the loss
/// is on all of it, as on the linkage (the weight's lossless G / 20 is the rigid feedforward's): ((1.242740 - 12.045)
/// x 0.96 + 12.045) / 20 = 0.083742 N m (without the loss on the weight it was 1.242740 x 0.96 / 20 = 0.059652, and
/// 0.114165 in all), plus the shaft at 20 t' = 22.565 rad/s, 0.05 tanh(45) + 2e-4 x 22.565 = 0.054513 N m: 0.138255
/// N m. The loss is on the speed's own torque at the arm too (the loss-free feedforward's): the arms slow down,
/// t'' = -0.557725 rad/s^2 (the constraint differentiated), and what turns with them, the arm 0.048 + 0.3 x 0.4^2 +
/// 0.5 / 6 x 0.4^2 = 0.109333 kg m^2 and the rod pair's middle 2/3 x 0.5 kg at half the elbow's plus half the
/// platform's velocity, gives back d(kinetic energy)/dt / t' = -0.079857 N m per arm, which the gearbox passes driven
/// back too: -0.079857 x (0.96 - 1) / 20 = +0.000160, 0.138415 N m in all. The tool: the bearing 0.02 tanh(500) + 1e-4 x 5 = 0.0205 N m through ratio 8 at the efficiency
/// 0.97^tanh(0.0205 x 5 / 0.5 W) = 0.993861 (the fade over 0.5 W), 0.002578 N m, plus the shaft at 40 rad/s,
/// 0.02 tanh(80) + 8e-5 x 40 = 0.0232 N m: 0.025778 N m. In a vertical cruise of the shipped delta at 0.5 m/s the
/// feedforward takes its own size off the arms' integrators.
#[test]
fn the_machine_friction_feedforward_is_the_steady_friction_and_the_integrator_carries_nothing() {
    let m = (ENTRY.build)(&ENTRY.config()).unwrap();
    let mut out = [0.0; 4];
    m.steady_friction(&[0.0, 0.0, -0.85, 0.0], &[0.0, 0.0, -0.5, 5.0], &mut out);
    let expected = [0.138415, 0.138415, 0.138415, 0.025778];
    for k in 0..4 {
        assert!((out[k] - expected[k]).abs() < 1e-5, "{out:?}, by hand {expected:?}");
    }
    let cruise = |model: bool| {
        let mut cfg = ENTRY.config();
        cfg.set_text("effects.friction_feedforward = true").unwrap();
        let source = if model { "machine" } else { "characteristic" };
        cfg.set_text(&format!("drive.friction_feedforward_model = \"{source}\"")).unwrap();
        cfg.set_text("start.pose = [0.0, 0.0, -0.7, 0.0]").unwrap();
        cfg.set_text("axis.z.max_velocity_m_s = 0.5").unwrap();
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        let at = sim.control.set_pos.clone();
        sim.command(Command::MoveTo(vec![at[0], at[1], -1.0, at[3]])).unwrap();
        let (mut ff, mut integrator, mut n) = (0.0, 0.0, 0);
        sim.run(0.6, |s| {
            if s.time() > 0.15 && s.control.set_acc[2] == 0.0 && (s.control.set_vel[2] + 0.5).abs() < 1e-12 {
                n += 1;
                for k in 0..3 {
                    ff += s.control.drives[k].friction_ff;
                    integrator += s.control.drives[k].controller_torque;
                }
            }
        });
        assert!(n > 1000, "{n} samples in the cruise");
        (ff / (3 * n) as f64, integrator / (3 * n) as f64)
    };
    let ((ff, with), (_, without)) = (cruise(true), cruise(false));
    assert!(ff > 0.1, "feedforward {ff} N m");
    assert!((without - with - ff).abs() < 0.03 * ff, "without: {without} N m, with {with}, of {ff}");
}

/// The friction feedforward (`steady_friction`) carries the arms' gearbox loss on the weight too, as the planar
/// linkage's (the trait's rule: the load goes through the efficiencies where the motor drives): on the rigid D4 with
/// its losses, at z = -0.85 m going down and up at 0.5 m/s with the tool turning, and sideways, the torques that leave
/// platform and tool without acceleration (a Newton solve of the rigid model: the tool's acceleration from its velocity a
/// microsecond along the model's rates, the Jacobian by differences) are the loss-free feedforward plus
/// `steady_friction`, to 5e-5 of the torque (seen: 2e-12 vertically, 1.3e-5 sideways, 7e-8 on the tool; 3.5e-4 before
/// `steady_friction` took the loss on the velocity terms too; what is left sideways is the efficiency's fade over the
/// power, which the rigid model takes at its loss-free acceleration).
#[test]
fn the_friction_feedforward_takes_the_loss_on_the_weight() {
    let machine = (ENTRY.build)(&{
        let mut cfg = ENTRY.config();
        for line in ["effects.joint_elasticity = false", "effects.torque_ripple = false"] {
            cfg.set_text(line).unwrap();
        }
        cfg
    })
    .unwrap();
    let pos = [0.0, 0.0, -0.85, 0.0];
    for vel in [[0.0, 0.0, -0.5, 5.0], [0.0, 0.0, 0.5, -5.0], [0.2, -0.1, 0.0, 2.0]] {
        let (mut state, mut held) = (Vec::new(), [0.0; 4]);
        machine.equilibrium(&pos, &mut state, &mut held);
        machine.set_tool_velocity(&mut state, &vel);
        let acc = |t: [f64; 4]| -> [f64; 4] {
            let mut rates = vec![0.0; machine.state_len()];
            machine.derivatives(&state, &t, &mut rates);
            let velocity = |h: f64| {
                let s: Vec<f64> = state.iter().zip(&rates).map(|(s, r)| s + h * r).collect();
                let (mut p, mut v) = ([0.0; 4], [0.0; 4]);
                machine.tool_state(&s, &mut p, &mut v);
                v
            };
            let (ahead, behind) = (velocity(1e-6), velocity(-1e-6));
            std::array::from_fn(|i| (ahead[i] - behind[i]) / 2e-6)
        };
        let mut t = held;
        for _ in 0..20 {
            let a = acc(t);
            // a + J dt = 0 by Gauss elimination, J[r][c] = d a_r / d t_c.
            let mut j: [[f64; 5]; 4] = [[0.0; 5]; 4];
            for c in 0..4 {
                let mut s = t;
                s[c] += 1e-6;
                let b = acc(s);
                (0..4).for_each(|r| j[r][c] = (b[r] - a[r]) / 1e-6);
            }
            (0..4).for_each(|r| j[r][4] = -a[r]);
            for p in 0..4 {
                let pivot = (p..4).max_by(|&x, &y| j[x][p].abs().total_cmp(&j[y][p].abs())).unwrap();
                j.swap(p, pivot);
                for r in p + 1..4 {
                    let f = j[r][p] / j[p][p];
                    (p..5).for_each(|c| j[r][c] -= f * j[p][c]);
                }
            }
            let mut step = [0.0; 4];
            for p in (0..4).rev() {
                step[p] = (j[p][4] - (p + 1..4).map(|c| j[p][c] * step[c]).sum::<f64>()) / j[p][p];
            }
            (0..4).for_each(|r| t[r] += step[r]);
        }
        let a = acc(t);
        assert!(a.iter().all(|a| a.abs() < 1e-7), "{vel:?}: {a:?}");
        let (mut ff, mut friction) = ([0.0; 4], [0.0; 4]);
        machine.feedforward_torque_moving(&pos, &vel, &[0.0; 4], &mut ff);
        machine.steady_friction(&pos, &vel, &mut friction);
        for k in 0..4 {
            let off = t[k] - ff[k] - friction[k];
            println!(
                "{vel:?}: motor {k}: Newton {:.6} N m, feedforward {:.6} + friction {:.6}, off {off:.2e}",
                t[k], ff[k], friction[k]
            );
            assert!(off.abs() < 5e-5 * t[k].abs(), "{vel:?}: motor {k}: {t:?} against {ff:?} + {friction:?}");
        }
    }
}

/// On a compliant base (`effects.base_compliance`) the machine keeps its collision capsules (11 of them, carried
/// along with the base) and its resonance is the lower of its own and the base's: the lowest of its modes.
#[test]
fn on_a_compliant_base_the_capsules_stay_and_the_resonance_is_the_lower() {
    let pose = [0.1525, 0.0, -0.75, 0.0];
    let build = |on: bool| {
        let mut cfg = ENTRY.config();
        cfg.set_text(&format!("effects.base_compliance = {on}")).unwrap();
        (ENTRY.build)(&cfg).unwrap()
    };
    let (plain, based) = (build(false), build(true));
    let capsules = |m: &dyn sim_core::Machine| {
        let (mut state, mut torque) = (Vec::new(), vec![0.0; m.motors().len()]);
        m.equilibrium(&pose, &mut state, &mut torque);
        m.collision_capsules(&state).len()
    };
    assert_eq!((capsules(plain.as_ref()), capsules(based.as_ref())), (11, 11));
    let (own, on_base) = (plain.resonance_hz(&pose).unwrap(), based.resonance_hz(&pose).unwrap());
    let lowest = based.modes(&pose)[0].hz;
    assert!(
        on_base <= own && (on_base - lowest).abs() < 1e-9 * lowest,
        "{own} Hz alone, {on_base} on the base, lowest mode {lowest}"
    );
}
