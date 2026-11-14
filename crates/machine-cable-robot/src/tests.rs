use super::*;
use sim_components::analysis::{analyse_cycle, verdicts};

#[test]
fn the_planar_cable_robot_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&PLANAR);
}

#[test]
fn the_spatial_cable_robot_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&SPATIAL);
}

fn planar() -> CableRobot {
    CableRobot::new(&PLANAR.config()).unwrap()
}

const CENTRE: [f64; 2] = [0.0, 0.8];

/// Each cable at the centre, by hand: the carriage 1 m across and 0.8 m down (or up) from the pulley's swivel point.
/// In the pulley's plane, centre M = (0.025, 0), so m = |(0.975, -0.8)| = 1.261200 m, the straight part
/// l_f = sqrt(m^2 - r^2) = 1.260952 m, theta_B = atan2(-0.8, 0.975) - acos(0.025 / m) = -2.238097 rad, the wrap
/// beta = theta_B + pi = 0.903496 rad, L = 0.025 beta + l_f = 1.283539 m; B = M + r (cos, sin) theta_B = (0.009528,
/// -0.019637), u = (B - P) / l_f = (-0.785495, 0.618868) (towards the pulley, up for the top cables, mirrored for the
/// others).
fn centre_cable(k: usize, spans: &[Span]) {
    let s = spans[k];
    assert!((s.length - 1.283539).abs() < 1e-6 && (s.straight - 1.260952).abs() < 1e-6, "{s:?}");
    assert!((s.u[0].abs() - 0.785495).abs() < 1e-6 && (s.u[2].abs() - 0.618868).abs() < 1e-6, "{s:?}");
}

/// The centre's distribution, by hand: the four cables are mirror images, so A A^T = diag(4 u_x^2, 4 u_z^2) and A 1 = 0.
/// The closed form t = t_ref + A^T (A A^T)^-1 (w - A t_ref) is then t_ref + u_z,k m g / (4 u_z^2): with m = 3 + 2 kg,
/// t_ref = 120 N: 120 +- 49.033 / (4 x 0.618868) = 120 +- 19.808 N: 139.808 N on the top cables, 100.192 N on the
/// bottom ones, all within 10 N to 400 N, so the distribution is the closed form.
#[test]
fn the_planar_tensions_at_the_centre_are_the_hand_solved_ones() {
    let robot = planar();
    let spans = robot.spans(&CENTRE).unwrap();
    for k in 0..4 {
        centre_cable(k, &spans);
    }
    let t = robot.static_tensions(&CENTRE).unwrap();
    for (k, want) in [139.807643, 139.807643, 100.192357, 100.192357].iter().enumerate() {
        assert!((t[k] - want).abs() < 1e-5, "{t:?}");
    }
    // The machine holds it with the drums' torque over the gearbox: rigid, t r / i at each motor, pulling in.
    let mut cfg = lossless_config(&PLANAR);
    cfg.set_text("effects.cable_stretch = false").unwrap();
    let machine = (PLANAR.build)(&cfg).unwrap();
    let mut torque = vec![0.0; 4];
    machine.equilibrium(&CENTRE, &mut Vec::new(), &mut torque);
    for k in 0..4 {
        assert!((torque[k] + t[k] * 0.025 / 5.0).abs() < 1e-9, "{torque:?}");
    }
}

/// The cables' stiffness at the centre, by hand: cable k of free length l_k = L / (1 + t_k / EA) is a spring of
/// EA / l_k = (EA + t_k) / L. The mirror images cancel the cross terms, and the top and bottom tensions add up to
/// 2 t_ref: K_xx = u_x^2 (4 EA + 4 t_ref) / L = 0.617003 x 400480 / 1.283539 = 192512 N/m, K_zz = 0.382997 x 312014
/// = 119500 N/m. The lowest natural frequency is the vertical one, sqrt(119500 / 5) / (2 pi) = 24.605 Hz; the
/// tensions turning with the cables add their geometric stiffness, sum t (1 - u_z^2) / l_f = 0.617 x 480 / 1.261 =
/// 235 N/m, 0.1 % up.
#[test]
fn the_cable_stiffness_and_natural_frequency_at_the_centre_are_the_hand_calculated_ones() {
    let robot = planar();
    let k = robot.cable_stiffness(&CENTRE).unwrap();
    assert!((k[0] - 192512.4).abs() < 1.0 && (k[3] - 119499.9).abs() < 1.0 && k[1].abs() < 1e-6, "{k:?}");
    let f = robot.natural_frequencies(&CENTRE).unwrap();
    assert!(f[0] > 24.6047 && f[0] < 24.6047 * 1.002, "{f:?}");
    let whole = robot.stiffness(&CENTRE).unwrap();
    assert!((whole[3] - 119499.9 - 234.9).abs() < 2.0, "{whole:?}");
    // The stiffness changes over the workspace: 0.5 m to the right the right cables are shorter and steeper, and the
    // vertical stiffness is half as much again (176883 N/m).
    assert!(robot.cable_stiffness(&[0.5, 0.8]).unwrap()[3] > 1.4 * k[3]);
}

/// 5 cm below the top pulleys the top cables pull almost flat. By hand, at (0, 1.55): each top cable rises 0.0256 per
/// unit of its length, each bottom one falls 0.8391; the top ones have to lift the 49.03 N and pull down the bottom
/// ones' 10 N at the least: t_1 + t_2 >= (49.03 + 2 x 10 x 0.8391) / 0.0256, 1284 N each, beyond the 400 N rating.
/// The pose is refused; the centre is not.
#[test]
fn a_pose_outside_the_wrench_feasible_workspace_is_refused() {
    let robot = planar();
    assert_eq!(robot.unreachable(&CENTRE, 0.0), None);
    assert!(robot.static_tensions(&[0.0, 1.55]).is_none());
    assert_eq!(robot.unreachable(&[0.0, 1.55], 0.0), Some("error-cable-wrench-infeasible"));
    let mut motors = [0.0; 4];
    robot.inverse(&[0.0, 1.55], &mut motors);
    assert!(motors.iter().all(|v| v.is_nan()));
}

/// Where the closed form takes a cable below its least tension, the distribution holds that cable at the bound and
/// shares the rest out again. A point held by three cables, up-left, up-right and straight down (unit columns (-s, s),
/// (s, s), (0, -1), s = 1 / sqrt 2), carries w = (0, 10). By hand from t_ref = 5: A t_ref = (0, 5 (2 s - 1)) = (0, 2.071),
/// A A^T = diag(1, 2), so y = (0, 7.929 / 2) and t = 5 + (s, s, -1) 3.964 = (7.803, 7.803, 1.036). With a least tension
/// of 2 N the down cable is fixed at 2 N, the two up ones carry (0, 12): 12 / (2 s) = 8.485 N each. With a band of 2 N
/// to 6 N no tensions hold it.
#[test]
fn the_distribution_fixes_a_cable_at_its_bound_and_shares_out_the_rest() {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let a = [-s, s, 0.0, s, s, -1.0];
    let t = distribute(&a, &[0.0, 10.0], 1.0, 100.0, 5.0).unwrap();
    assert!((t[0] - (5.0 + s * 3.964466)).abs() < 1e-6 && (t[2] - (5.0 - 3.964466)).abs() < 1e-6, "{t:?}");
    let t = distribute(&a, &[0.0, 10.0], 2.0, 100.0, 5.0).unwrap();
    assert!((t[2] - 2.0).abs() < 1e-12 && (t[0] - 12.0 * s).abs() < 1e-9 && (t[1] - 12.0 * s).abs() < 1e-9, "{t:?}");
    assert!(distribute(&a, &[0.0, 10.0], 2.0, 6.0, 5.0).is_none());
}

/// The spatial robot's inverse and Gauss-Newton forward kinematics undo each other at three poses.
#[test]
fn the_spatial_forward_kinematics_undoes_the_inverse() {
    let machine = (SPATIAL.build)(&SPATIAL.config()).unwrap();
    for pose in
        [[0.0, 0.0, 1.1, 0.0, 0.0, 0.0], [0.3, -0.2, 1.3, 0.05, -0.03, 0.08], [-0.3, 0.2, 0.9, -0.05, 0.05, -0.03]]
    {
        assert_eq!(machine.unreachable(&pose, 0.0), None);
        let (mut motors, mut back) = ([0.0; 8], [0.0; 6]);
        machine.inverse(&pose, &mut motors);
        machine.forward(&motors, &mut back);
        for k in 0..6 {
            assert!((pose[k] - back[k]).abs() < 1e-10, "{pose:?} came back as {back:?}");
        }
    }
}

/// Each default cycle passes every verdict: the defaults are sized examples.
#[test]
fn the_default_cycles_pass_their_verdicts() {
    for entry in [&PLANAR, &SPATIAL] {
        let cfg = entry.config();
        let mut sim = sim_components::Sim::new(entry, &cfg).unwrap();
        let load = analyse_cycle(&mut sim).unwrap();
        assert!(!load.tripped, "{}", entry.id);
        for v in verdicts(&cfg, &load).unwrap() {
            assert!(v.ok, "{}: {} of motor {:?} is {} against {}", entry.id, v.check, v.motor, v.value, v.limit);
        }
    }
}
