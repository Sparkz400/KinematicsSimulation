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

/// The same three cables with the least tension just above the down cable's exact share: t_2 = 5 - (10 - 5 (2 s - 1)) / 2
/// = 1.0355339 N, the band from t_2 + 1e-6 N. The penalised problem misses the wrench by 1 / RHO of the correction and
/// leaves t_2 2e-6 N higher, inside the band, so the exact solve over the free cables took it to 1e-6 N below the band
/// and refused tensions that exist: the down cable at the bound, the up ones (10 + t_min) / (2 s) each. On the spatial
/// robot at (0.3774, 0, 1.1) m turned 0.047175 rad about z (on its way to the end of x) its cable 7 came to 19.97 N in
/// the exact solve against 20 N, and its motors were given no setpoint there (14 November 2026).
#[test]
fn a_cable_the_exact_solve_takes_out_of_the_band_goes_to_its_bound() {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let a = [-s, s, 0.0, s, s, -1.0];
    let low = 5.0 - (10.0 - 5.0 * (2.0 * s - 1.0)) / 2.0 + 1e-6;
    let t = distribute(&a, &[0.0, 10.0], low, 100.0, 5.0).unwrap();
    let up = (10.0 + low) / (2.0 * s);
    assert!((t[2] - low).abs() < 1e-12 && (t[0] - up).abs() < 1e-9 && (t[1] - up).abs() < 1e-9, "{t:?}");
    let robot = CableRobot::new(&SPATIAL.config()).unwrap();
    let t = robot.static_tensions(&[0.3774, 0.0, 1.1, 0.0, 0.0, 0.047175]).unwrap();
    assert!(t.iter().all(|v| (20.0 - 1e-6..=1000.0).contains(v)) && t.iter().any(|v| (v - 20.0).abs() < 1e-9), "{t:?}");
}

/// Motor angles that no pose fits, as a run with a following error has them: the spatial robot's motors 0.163 s and
/// 1.703 s into its cycle on a weak drive (sim-app's `demanding` config, 14 November 2026), 2.5 rad behind on cable 1
/// and 1.9 rad on cable 7, 12.7 and 9.6 mm of cable at 200 rad/m, and the planar robot's 0.148 s and 1.271 s in, its
/// cable 2 26 mm and its cable 4 250 mm slack. The forward kinematics fitted the lengths by Gauss-Newton, which slowed
/// to a ratio of 0.72 and then 0.97 a step there and ran out of steps, or (on the platform's equilibrium) stepped where
/// every cable was slack: the following error was shown as NaN. Elastic, the platform hangs where its weight and the
/// cables' tensions EA (L - l) / l balance, A t = w (to 1e-6 N and N m, every cable pulling or slack); rigid, it is
/// where the lengths fit best: their misfit grows when the pose moves 1 um or 1 urad either way along any axis.
#[test]
fn motor_angles_no_pose_fits_give_the_pose_the_platform_takes() {
    let spatial: [&[f64]; 2] = [
        &[
            218.1766718244078,
            236.09204543034912,
            246.82031457745563,
            228.18806261401326,
            230.0101951015795,
            248.74408977052855,
            257.55184642979265,
            241.0469591701435,
        ],
        &[
            217.81652339558013,
            236.09872513770574,
            247.05701949764565,
            228.0179794981002,
            229.8651170709315,
            249.02085196246475,
            257.9467071660587,
            241.16067748027044,
        ],
    ];
    let planar: [&[f64]; 2] = [
        &[262.9357534764116, 237.12395011188107, 252.04079274904026, 280.0630114959582],
        &[252.72452723453748, 351.53586672957255, 293.2643243221405, 211.62183409918603],
    ];
    for (entry, all) in [(&SPATIAL, spatial), (&PLANAR, planar)] {
        for elastic in [true, false] {
            let mut cfg = entry.config();
            cfg.set_text(&format!("effects.cable_stretch = {elastic}")).unwrap();
            let robot = CableRobot::new(&cfg).unwrap();
            let (n, m) = (robot.n(), robot.m());
            for motors in all {
                let mut pose = vec![0.0; n];
                robot.forward(motors, &mut pose);
                assert!(pose.iter().all(|v| v.is_finite()), "{} elastic {elastic}: {pose:?}", entry.id);
                let free: Vec<f64> = (0..m).map(|k| motors[k] / robot.winches[k].ratio()).collect();
                if elastic {
                    let spans = robot.spans(&pose).unwrap();
                    let t: Vec<f64> =
                        spans.iter().zip(&free).map(|(s, l)| (robot.ea * (s.length - l) / l).max(0.0)).collect();
                    let (a, w) = (robot.structure(&spans), robot.platform_bias(&pose, &vec![0.0; n]));
                    for r in 0..n {
                        let off = (0..m).map(|k| a[r * m + k] * t[k]).sum::<f64>() - w[r];
                        assert!(off.abs() < 1e-6, "{} {pose:?}: {off} N or N m left on axis {r}, {t:?}", entry.id);
                    }
                    continue;
                }
                let misfit = |pose: &[f64]| -> f64 {
                    let lengths = robot.rest_lengths(pose).unwrap();
                    lengths.iter().zip(&free).map(|(l, f)| (l - f).powi(2)).sum::<f64>().sqrt()
                };
                let best = misfit(&pose);
                for axis in 0..n {
                    for by in [-1e-6, 1e-6] {
                        let mut step = vec![0.0; n];
                        step[axis] = by;
                        let moved = misfit(&robot.moved(&pose, &step));
                        assert!(moved > best, "{} {pose:?} fits by {best} m, {by} along {axis}: {moved} m", entry.id);
                    }
                }
            }
        }
    }
}

/// The motors' angles of a pose ask the cables for the distribution's tensions, so the platform hangs there on them:
/// at (0.3, -0.2, 1.3) m turned (0.05, -0.03, 0.08) the closed form t_ref + A^+ (w - A t_ref) asked -8.7 N of cable 5,
/// a slack cable, and the platform hung 1.5 mrad off. The free lengths of `inverse` stretch each cable to a tension
/// EA (L - l) / l within the band (20 N to 1000 N) that holds the platform's weight, A t = w (to 1e-6 N and N m).
/// And the machine set up at rest at the end of x turned 0.05 rad about z has its setpoint (where the motors' angles
/// put the tool) within its setpoint slack of the pose: 0.42 mrad off about z, the gearboxes' twist, within 0.50 mrad;
/// it had been 0.49 mrad against the default 0.1.
#[test]
fn the_platform_hangs_where_the_motors_put_it() {
    let robot = CableRobot::new(&SPATIAL.config()).unwrap();
    let pose = [0.3, -0.2, 1.3, 0.05, -0.03, 0.08];
    let mut motors = vec![0.0; 8];
    robot.inverse(&pose, &mut motors);
    let spans = robot.spans(&pose).unwrap();
    let t: Vec<f64> = (0..8)
        .map(|k| {
            let l = motors[k] / robot.winches[k].ratio();
            robot.ea * (spans[k].length - l) / l
        })
        .collect();
    assert!(t.iter().all(|v| (20.0 - 1e-6..=1000.0).contains(v)), "{t:?}");
    let (a, w) = (robot.structure(&spans), robot.platform_bias(&pose, &[0.0; 6]));
    for r in 0..6 {
        let off = (0..8).map(|k| a[r * 8 + k] * t[k]).sum::<f64>() - w[r];
        assert!(off.abs() < 1e-6, "{off} N or N m left on axis {r}, tensions {t:?}");
    }
    let end = [0.4, 0.0, 1.1, 0.0, 0.0, 0.05];
    let mut cfg = SPATIAL.config();
    cfg.set_text(&format!("start.pose = {end:?}")).unwrap();
    let sim = sim_components::Sim::new(&SPATIAL, &cfg).unwrap();
    let slack = sim.machine.setpoint_slack();
    let off: Vec<f64> = sim.control.planned_pos().iter().zip(&end).map(|(p, e)| p - e).collect();
    assert!(slack < 1e-3 && off.iter().all(|d| d.abs() < slack) && off[5].abs() > 1e-4, "{off:?} against {slack}");
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

/// What the planar carriage puts on a compliant base at the centre (x 0, z 0.8 m), moving at (0.5, -0.2) m/s: m = 3 + 2
/// kg, so m v = (2.5, 0, -1.0) N s, and about the origin p x m v = (0, 0.8 x 2.5, 0) = (0, 2.0, 0) N m s; the first
/// moment m p = (0, 0, 4.0) kg m.
#[test]
fn the_planar_carriage_has_the_hand_calculated_momentum() {
    let robot = planar();
    let (mut state, mut torque) = (Vec::new(), vec![0.0; 4]);
    robot.equilibrium(&CENTRE, &mut state, &mut torque);
    robot.set_tool_velocity(&mut state, &[0.5, -0.2]);
    let f = robot.frame_momentum(&state).unwrap();
    for (got, want) in
        f.linear.iter().chain(&f.angular).chain(&f.first_moment).zip([2.5, 0.0, -1.0, 0.0, 2.0, 0.0, 0.0, 0.0, 4.0])
    {
        assert!((got - want).abs() < 1e-9, "{f:?}");
    }
    assert!((f.mass - 5.0).abs() < 1e-12 && (f.gravity[2] + 9.80665).abs() < 1e-12);
}

/// The planar winches' steady friction at the centre, moving across at 1 m/s, by hand: each cable runs at |L'| =
/// |u_x| x 1 m/s = 0.785495 m/s (`centre_cable`), the left ones (1 and 4) paying out, the right ones hauling in; the
/// winch's ratio 5 / 0.025 m = 200 rad/m turns the rotor at 157.0990 rad/s. Shaft: 0.05 tanh(314.2) + 2e-4 x 157.0990 =
/// 0.0814198 N m; the drum's bearings 2 N + 1 N s/m x 0.785495 m/s = 2.785495 N at the cable. The cable's tension
/// (100 N to 140 N) loads every gearbox the same way, against the drum: hauling in, the motor drives, f / (200 x 0.95)
/// = 0.0146605 N m, so -(0.0814198 + 0.0146605) = -0.0960803 N m; paying out, the cable drives the gearbox back, which
/// takes eta off the friction, f x 0.95 / 200 = 0.0132311 N m, so 0.0946509 N m.
#[test]
fn the_planar_winches_steady_friction_at_the_centre_is_the_hand_calculated_one() {
    let mut out = [0.0; 4];
    planar().steady_friction(&CENTRE, &[1.0, 0.0], &mut out);
    for (got, want) in out.iter().zip([0.0946509, -0.0960803, -0.0960803, 0.0946509]) {
        assert!((got - want).abs() < 1e-6, "{out:?}");
    }
}

/// Friction feedforward from the machine's own model in a sim of the planar robot, the gearboxes' efficiency off (which
/// the integrators keep on the cables' tensions by design, some per cent of them): in the cruise at 1.5 m/s across
/// (0, 0.8) -> (0.6, 0.8) the drives get the winches' friction (0.10 N m to 0.12 N m), and the velocity loops'
/// integrators carry under 3 % of it; without it, they carry the whole friction. Both besides a part common to the four
/// winches (0.009 N m each): the cables' internal tension, which A t = 0 leaves to the position loops, not friction.
#[test]
fn the_planar_machine_friction_feedforward_takes_the_friction_off_the_integrators() {
    let cruise = |model: bool| {
        let mut cfg = PLANAR.config();
        cfg.set_text("effects.friction_feedforward = true").unwrap();
        cfg.set_text("effects.gearbox_efficiency = false").unwrap();
        let source = if model { "machine" } else { "characteristic" };
        cfg.set_text(&format!("drive.friction_feedforward_model = \"{source}\"")).unwrap();
        let mut sim = sim_components::Sim::new(&PLANAR, &cfg).unwrap();
        sim.command(sim_components::Command::MoveTo(vec![0.6, 0.8])).unwrap();
        let (mut ff, mut integrator, mut n, mut start) = ([0.0; 4], [0.0; 4], 0, f64::NAN);
        sim.run(0.6, |s| {
            let c = &s.control;
            if c.set_acc.iter().all(|a| *a == 0.0) && (c.set_vel[0] - 1.5).abs() < 1e-12 {
                // The integrators settle for 0.1 s after the speed-up.
                start = start.min(s.time());
                if s.time() > start + 0.1 {
                    for k in 0..4 {
                        ff[k] += c.drives[k].friction_ff;
                        integrator[k] += c.drives[k].controller_torque;
                    }
                    n += 1;
                }
            }
        });
        assert!(n > 1000, "{n} samples in the cruise");
        (ff.map(|x| x / n as f64), integrator.map(|x| x / n as f64))
    };
    let ((ff, with), (none, without)) = (cruise(true), cruise(false));
    for k in 0..4 {
        let common = with.iter().sum::<f64>() / 4.0;
        let (carried, friction) = (with[k] - common, without[k] - common);
        assert!(none[k] == 0.0 && friction.abs() > 0.09, "winch {k}: without, {friction} N m");
        assert!(carried.abs() < 0.03 * friction.abs(), "winch {k}: the integrator carries {carried} N m of {friction}");
        assert!((ff[k] - friction).abs() < 0.03 * friction.abs(), "winch {k}: feedforward {} N m of {friction}", ff[k]);
    }
}

/// Both default cycles' figures as they were before the speed work of 16 November 2026, recorded then on Windows. The
/// warm-started elastic `forward` feeds only the tool's following error (the signal), not the motion, so on the
/// platform they were recorded on they are the same to 1e-9. Another platform's maths library rounds sine and arc
/// tangent differently in the last bit, which the closed loop amplifies to some 1e-4 (as on the hexapods): 1e-3 there.
#[test]
fn the_default_cycles_keep_their_figures() {
    const PLANAR_BEFORE: [f64; 22] = [
        1.0389606581361805,
        0.6479861361877007,
        55.15605952445517,
        37.66381165862248,
        1.6484450662868255,
        0.7751408942240309,
        216.53514897364323,
        119.22439518511554,
        0.9741805671230831,
        0.5074452311248481,
        44.49918884238062,
        28.429049654761798,
        1.1529379398008017,
        0.636068238322625,
        193.46466438048614,
        98.69438046364186,
        0.0006009987347714806,
        0.00014488979459075676,
        0.0006471198258464821,
        0.0,
        0.0,
        0.0,
    ];
    const SPATIAL_BEFORE: [f64; 38] = [
        2.4412936559147154,
        1.5461626889253295,
        170.00768422234148,
        114.70237297650282,
        2.0888749855699738,
        1.6332665649124543,
        27.146001260943798,
        15.930176850698514,
        1.6766752878582951,
        1.2416166492776037,
        47.139055592409754,
        40.61165201783822,
        2.0727370567209973,
        1.6599916758200133,
        88.41374837354466,
        66.15660061604721,
        1.7845207139881991,
        1.1656503598153982,
        50.93796101943463,
        32.05191462719961,
        1.9409129125355142,
        1.4520758435848597,
        83.60517595710246,
        56.1473698625616,
        2.2976306546130973,
        1.4069747101344625,
        148.96721978856755,
        106.63558187173287,
        1.7055648786260489,
        1.3957958869653693,
        25.3525102592258,
        15.065209084660706,
        0.00022184312648876797,
        6.842195194345995e-5,
        0.00026880527283743965,
        0.0015966339615177524,
        0.0004910623584291974,
        0.0015966339615177524,
    ];
    let (planar, spatial) = (cycle_figures(&PLANAR), cycle_figures(&SPATIAL));
    let tol = if cfg!(windows) { 1e-9 } else { 1e-3 };
    assert_figures("cable-planar", &planar, &PLANAR_BEFORE, tol);
    assert_figures("cable-spatial", &spatial, &SPATIAL_BEFORE, tol);
}

/// The default cycle's key figures: per motor its peak and RMS torque and the energy it drew and fed back, then the
/// tool's path error (peak, RMS), tracking error, and the same of its turn.
fn cycle_figures(entry: &MachineEntry) -> Vec<f64> {
    let cfg = entry.config();
    let mut sim = sim_components::Sim::new(entry, &cfg).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    let mut out: Vec<f64> =
        load.motors.iter().flat_map(|m| [m.peak_torque, m.rms_torque, m.energy_in, m.energy_regenerated]).collect();
    out.extend([
        load.path_error_peak,
        load.path_error_rms,
        load.tracking_error_peak,
        load.path_angle_error_peak,
        load.path_angle_error_rms,
        load.tracking_angle_error_peak,
    ]);
    out
}

/// Every figure within `tol` of the recorded one, relative (absolute below 1e-12).
fn assert_figures(id: &str, got: &[f64], want: &[f64], tol: f64) {
    assert_eq!(got.len(), want.len(), "{id}: {got:?}");
    for (k, (g, w)) in got.iter().zip(want).enumerate() {
        assert!((g - w).abs() <= tol * w.abs().max(1e-12), "{id}: figure {k} is {g:e}, was {w:e}; all: {got:?}");
    }
}
