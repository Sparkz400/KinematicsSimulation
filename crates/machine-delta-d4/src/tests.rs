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

fn close(a: f64, b: f64, tolerance: f64, what: &str) {
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
    let (top, edge_top, edge_bottom) = (m.shaft().geometry([0.0, 0.0, -0.675]), m.shaft().geometry([0.6, 0.0, -0.675]), m.shaft().geometry([0.6, 0.0, -0.875]));
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
    assert_eq!(d4(&["shaft.max_length_m = 0.9"]).unreachable(&[0.3, 0.4, -0.85, 0.0], 0.0), Some("error-delta-shaft-stroke"));
    assert_eq!(d4(&["shaft.max_cardan_angle_deg = 30.0"]).unreachable(&[0.3, 0.4, -0.85, 0.0], 0.0), Some("error-delta-cardan-angle"));
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
        close(cardan_speed_ratio(beta, phi), (cardan(beta, phi + h) - cardan(beta, phi - h)) / (2.0 * h), 1e-7, "speed ratio");
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

/// The tool's motor torque for 100 rad/s^2 of the tool's turn, rigid and loss-free: J_c = tool 2e-3 + shaft spin 8e-5
/// + payload 5e-3 = 7.08e-3 kg m^2, at the motor 7.08e-3 * 100 / 8 = 0.0885 N m, plus the rotor and the gearbox's
/// input (1.7e-4 + 3.47e-5) * 8 * 100 = 0.16376 N m: 0.25226 N m. Without the payload 0.0260 + 0.16376 = 0.18976 N m.
/// Given that torque the model turns the tool at 100 rad/s^2, and the arms' torques do not change.
#[test]
fn the_tool_motors_torque_for_a_turn_is_the_hand_calculated() {
    let m = d4(&["effects.joint_friction = false", "effects.motor_friction = false", "effects.torque_ripple = false", "effects.gearbox_efficiency = false", "effects.joint_elasticity = false"]);
    close(m.tool_inertia(), 7.08e-3, 1e-12, "J_c");
    let pose = [0.1, -0.1, -0.8, 0.5];
    let (mut moving, mut still) = ([0.0; 4], [0.0; 4]);
    m.feedforward_torque(&pose, &[0.0, 0.0, 0.0, 100.0], &mut moving);
    m.feedforward_torque(&pose, &[0.0; 4], &mut still);
    close(moving[3] - still[3], 0.25226, 1e-9, "tool motor torque");
    assert!((0..3).all(|k| moving[k] == still[k]));
    let (mut state, mut hold) = (Vec::new(), [0.0; 4]);
    m.equilibrium(&pose, &mut state, &mut hold);
    let mut rates = vec![0.0; m.state_len()];
    m.derivatives(&state, &moving, &mut rates);
    close(rates[m.n3 + 1], 100.0, 1e-9, "tool acceleration");
    let mut m = m;
    m.set_payload_attached(false);
    m.feedforward_torque(&pose, &[0.0, 0.0, 0.0, 100.0], &mut moving);
    close(moving[3], 0.18976, 1e-9, "without the payload");
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

/// The tool's torsion with the motor held: the gearbox's 8594.4 N m/rad in series with the shaft's 500 N m/rad,
/// 1 / (1/8594.4 + 1/500) = 472.51 N m/rad, against J_c = 7.08e-3 kg m^2: sqrt(472.51 / 7.08e-3) / 2 pi = 41.12 Hz,
/// without the payload (2.08e-3) 75.86 Hz; damping 0.075 / (2 sqrt(472.51 * 7.08e-3)) = 2.05 % of critical. It is among
/// the modes, the arms' first; the rigid model has none of it.
#[test]
fn the_tools_torsional_mode_is_the_hand_calculated() {
    let m = d4(&[]);
    let mode = m.tool_mode().unwrap();
    close(mode.hz, 41.1158, 1e-5, "with the payload");
    close(mode.damping, 0.020503, 1e-4, "damping ratio");
    let mut empty = d4(&[]);
    empty.set_payload_attached(false);
    close(empty.tool_mode().unwrap().hz, 75.8567, 1e-5, "without");
    assert!(m.modes(&[0.0, 0.0, -0.8, 0.0]).iter().any(|x| x.hz == mode.hz));
    assert!(d4(&["effects.joint_elasticity = false"]).tool_mode().is_none());
    close(d4(&["effects.shaft_torsion = false"]).tool_joint().gearbox.stiffness, 8594.4, 1e-12, "the gearbox alone");
}

#[test]
fn the_d4_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ENTRY);
}

/// The default cycle, the Adept cycle with the tool turned by 90 degrees on each traverse: the tool stands at 90
/// degrees during the pick (to within 1.5 mrad), and every verdict passes, "settling" with 0.39 mm (its distance to the
/// target counts the turn's error in rad with the position's in m, as on every machine), the tool's gearbox peaking at
/// 2.3 N m of its 29. With the catalogue's backlash on the arms' NRB080 (9 arcmin) and the tool's NRB060 (10 arcmin)
/// "settling" fails alone, 1.2 mm: 0.6 mm of it the arms' sag under the platform's weight (the D3's finding) and 1 mrad
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
    let tool = all.iter().find(|v| v.check == "tool-gearbox-peak").unwrap();
    assert!(tool.value > 1.0 && tool.value < 5.0 && tool.limit == 29.0, "{tool:?}");
    let mut catalogue = cfg.clone();
    catalogue.set_text("gearbox.backlash_rad = 2.618e-3").unwrap();
    catalogue.set_text("tool_gearbox.backlash_rad = 2.909e-3").unwrap();
    let failed: Vec<_> = verdicts(&catalogue).into_iter().filter(|v| !v.ok).map(|v| v.check).collect();
    assert_eq!(failed, ["settling"]);
}
