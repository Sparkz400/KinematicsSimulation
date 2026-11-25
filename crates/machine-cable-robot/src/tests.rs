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

/// The planar robot without its cables' own mass: the hand calculations below are those of massless cables.
fn planar() -> CableRobot {
    let mut cfg = PLANAR.config();
    cfg.set_text("cable.mass_per_m_kg = 0.0").unwrap();
    CableRobot::new(&cfg).unwrap()
}

/// The planar robot with its cables of `rope` kg/m.
fn planar_with_rope(rope: f64) -> CableRobot {
    let mut cfg = PLANAR.config();
    cfg.set_text(&format!("cable.mass_per_m_kg = {rope:?}")).unwrap();
    CableRobot::new(&cfg).unwrap()
}

const CENTRE: [f64; 2] = [0.0, 0.8];

/// The stack lists the derivative's vectors are made of read, iterate and compare as the `Vec`s they replace did: the
/// entries in their order, to the bit, and only up to their length.
#[test]
fn a_short_list_holds_what_was_collected_in_order() {
    let values = [0.1, -0.0, f64::MIN_POSITIVE, 3.0e-310, 1.0 / 3.0];
    let list: Short<f64, 8> = values.iter().copied().collect();
    assert_eq!(list.len(), values.len());
    assert!(list.iter().zip(values).all(|(a, b)| a.to_bits() == b.to_bits()));
    assert_eq!(list.into_iter().map(f64::to_bits).collect::<Vec<_>>(), values.map(f64::to_bits));
    assert!(Short::<f64, 8>::default().is_empty());
}

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
    cfg.set_text("cable.mass_per_m_kg = 0.0").unwrap();
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

/// The modes' damping from the cables' dampers (effects.cable_damping, c = 60 N s): cable k's c / l_k on its stretch's
/// rate, l_k = L / (1 + t_k / EA), so at the centre C_zz = sum c u_z^2 (1 + t_k / EA) / L = (c / EA) K_zz of the cables
/// alone, 6e-4 x 119499.9 = 71.700 N s/m. The vertical mode (x and z apart by the mirror images) with the whole stiffness
/// 119734.8 N/m on the 5 kg: w = 154.748 rad/s, 24.629 Hz, zeta = C_zz / (2 m w) = 0.046333 (it was reported 0).
#[test]
fn the_modes_carry_the_damping_of_the_cables() {
    let robot = planar();
    let modes = robot.modes(&CENTRE);
    let w = (119734.8f64 / 5.0).sqrt();
    let zeta = 6e-4 * 119499.9 / (2.0 * 5.0 * w);
    assert!((modes[0].hz / (w / TAU) - 1.0).abs() < 2e-5, "{modes:?} against {} Hz", w / TAU);
    assert!((modes[0].damping / zeta - 1.0).abs() < 2e-5, "{modes:?} against zeta {zeta}");
    assert_eq!(modes.len(), 2);
    assert!((modes[0].hz / robot.resonance_hz(&CENTRE).unwrap() - 1.0).abs() < 1e-12, "{modes:?}");
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
/// cables' tensions EA (L - l) / l (with their sag, and its pull beyond the chords, `sag_pull`) balance, A t + pull = w (to
/// 1e-6 N and N m, every cable pulling or slack); rigid, it is
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
                    let (spans, rest) = (robot.spans(&pose).unwrap(), vec![0.0; m]);
                    let t = robot.tensions(&spans, &rest, &free, &rest).platform;
                    let rot = robot.place(&pose).1;
                    let (a, w) =
                        (robot.structure(&spans), robot.platform_bias_at(&rot, &spans, Some(&free), &vec![0.0; n]));
                    let pull = robot.sag_pull(&spans, &free, &t);
                    for r in 0..n {
                        let off = (0..m).map(|k| a[r * m + k] * t[k]).sum::<f64>() + pull[r] - w[r];
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
/// EA (L - l) / l (with its sag) within the band (20 N to 1000 N) that holds the platform's weight, A t + pull = w (to 1e-6
/// N and N m; `sag_pull`).
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
    let free: Vec<f64> = (0..8).map(|k| motors[k] / robot.winches[k].ratio()).collect();
    let t = robot.tensions(&spans, &[0.0; 8], &free, &[0.0; 8]).platform;
    assert!(t.iter().all(|v| (20.0 - 1e-6..=1000.0).contains(v)), "{t:?}");
    let rot = robot.place(&pose).1;
    let (a, w) = (robot.structure(&spans), robot.platform_bias_at(&rot, &spans, Some(&free), &[0.0; 6]));
    let pull = robot.sag_pull(&spans, &free, &t);
    for r in 0..6 {
        let off = (0..8).map(|k| a[r * 8 + k] * t[k]).sum::<f64>() + pull[r] - w[r];
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

/// The scene's cable labels are the tensions of the signals (those of the platform's wrench, 139.808 N on the top cable
/// at the centre by hand, above), not the rigid model's at zero motor torque.
#[test]
fn the_scene_labels_the_tensions_the_signals_show() {
    let robot = planar();
    let (mut state, mut torque) = (Vec::new(), vec![0.0; 4]);
    robot.equilibrium(&CENTRE, &mut state, &mut torque);
    let mut signals = Vec::new();
    robot.base_signals(&state, &torque, &mut signals);
    let mut scene = Scene::default();
    robot.scene(&state, &mut scene);
    let label = scene.labels.iter().find(|l| l.owner_key == CABLE_KEYS[0]).unwrap();
    assert!((label.value - signals[0]).abs() < 1e-9 && label.value > 100.0, "{} {}", label.value, signals[0]);
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

/// The cables' tensions in motion (what the time-optimal plan keeps within the band): they hold the wrench M a + g. At
/// the centre with the carriage accelerating at (2, -1) m/s^2, m = 5 kg: sum t_k u_k = (m a_x, m (a_z + g)) = (10 N,
/// 5 x (9.80665 - 1) = 44.03325 N), by the unit vectors u_k of the spans (`centre_cable`). The tensions are linear in
/// the acceleration (twice the acceleration moves them twice as far from the tensions at rest) and the point mass's
/// do not change with its speed.
#[test]
fn the_cables_tensions_in_motion_hold_the_wrench_and_are_linear_in_the_acceleration() {
    let robot = planar();
    let tensions = |vel: [f64; 2], acc: [f64; 2]| {
        let mut out = Vec::new();
        robot.rigid_tensions_moving(&CENTRE, &vel, &acc, &mut out);
        out
    };
    let spans = robot.spans(&CENTRE).unwrap();
    let (rest, moving) = (tensions([0.0; 2], [0.0; 2]), tensions([0.0; 2], [2.0, -1.0]));
    for (k, want) in [139.807643, 139.807643, 100.192357, 100.192357].iter().enumerate() {
        assert!((rest[k] - want).abs() < 1e-5, "{rest:?}");
    }
    let held = |t: &[f64], axis: usize| (0..4).map(|k| t[k] * spans[k].u[axis]).sum::<f64>();
    assert!((held(&moving, 0) - 10.0).abs() < 1e-7 && (held(&moving, 2) - 44.03325).abs() < 1e-5, "{moving:?}");
    let double = tensions([0.0; 2], [4.0, -2.0]);
    (0..4).for_each(|k| assert!(((double[k] - rest[k]) - 2.0 * (moving[k] - rest[k])).abs() < 1e-9));
    let fast = tensions([0.5, 0.3], [0.0; 2]);
    (0..4).for_each(|k| assert!((fast[k] - rest[k]).abs() < 1e-9));
    let mut by_acc = Vec::new();
    robot.rigid_tensions(&CENTRE, &[2.0, -1.0], &mut by_acc);
    assert_eq!(by_acc, moving);
}

/// The cables' own weight at the centre, by hand: the potential V = rho g L (z_P - z_E) / 2 per cable has the force
/// rho g / 2 (L e_z - (z_P - z_E) u) at the anchor. The four cables at the centre have L = 1.283539 m and (z_P - z_E)
/// u_z = (-0.8)(0.618868) = (0.8)(-0.618868) = -0.495094 m, so each puts rho g / 2 (1.283539 + 0.495094) =
/// 0.04903325 x 1.778633 = 0.087213 N on the platform at rho = 0.01 kg/m, 0.348850 N in all (the platform's own 5 kg
/// weigh 49.03325 N); the mirror images cancel the horizontal part. And the wrench is the gradient of V in general:
/// central differences of the potential over the pose (1 um) at an off-centre pose.
#[test]
fn the_cables_own_weight_is_the_gradient_of_its_potential() {
    let robot = planar_with_rope(0.01);
    let bias = robot.platform_bias(&CENTRE, &[0.0; 2]);
    assert!(bias[0].abs() < 1e-9 && (bias[1] - (49.03325 + 0.348850)).abs() < 1e-5, "{bias:?}");
    assert_eq!(planar().platform_bias(&CENTRE, &[0.0; 2])[1], 5.0 * G);
    let pose = [0.3, 0.9];
    // Rigid and, at fixed free lengths, elastic.
    let free = [1.5, 1.4, 1.3, 1.2];
    for lengths in [None, Some(&free[..])] {
        let potential = |pose: &[f64]| {
            let (p, r) = robot.place(pose);
            robot.cable_weight(&robot.spans_at(p, &r).unwrap(), lengths).1
        };
        let (p, r) = robot.place(&pose);
        let wrench = robot.cable_weight(&robot.spans_at(p, &r).unwrap(), lengths).0;
        for (i, h) in [(0, [1e-6, 0.0]), (1, [0.0, 1e-6])] {
            let (up, down) = ([pose[0] + h[0], pose[1] + h[1]], [pose[0] - h[0], pose[1] - h[1]]);
            let gradient = (potential(&up) - potential(&down)) / 2e-6;
            assert!((wrench[i] - gradient).abs() < 1e-7 * wrench[i].abs().max(1.0), "{i}: {} {gradient}", wrench[i]);
        }
    }
}

/// The sag. By hand, a cable of 0.01 kg/m over 1 m horizontally at EA = 1e5 N: w l_h = 0.0980665 N, c = (w l_h)^2 EA /
/// 12 = 80.1420 N^3. Its chord tension T solves T - c / (2 T^2) = t0: at a straight-cable tension t0 of 100 N
/// T = 100 + 40.071 / 100.004^2 = 100.0040068 N; at t0 = 0 (the cable as long as its chord) T = (c / 2)^(1/3) = 3.4219740
/// N; slack by 100 N it still hangs at 0.6310287 N. Its slope dT/dt0 = 1 / (1 + c / T^3) is Ernst's modulus. The inverse
/// gives t0 back, and the work W, the integral of T over t0, is that of Simpson's rule over -50 N to 50 N. The drum's pull
/// is minus the derivative of the spring's energy over the free length, which also checks W against it, and the
/// equilibrium holds the platform and the drums with it.
#[test]
fn the_sag_softens_the_cable_by_ernsts_modulus() {
    let c = (0.0980665f64).powi(2) * 1e5 / 12.0;
    assert!((c - 80.1420).abs() < 1e-3);
    for (t0, want) in [(100.0, 100.0040068), (0.0, 3.4219740), (-100.0, 0.6310287)] {
        assert!((sagged(t0, c) - want).abs() < 1e-7, "{t0}: {}", sagged(t0, c));
    }
    assert_eq!(sagged(100.0, 0.0), 100.0);
    assert_eq!(sagged(-1.0, 0.0), 0.0);
    for t0 in [-1e4, -100.0, -1.0, 0.5, 3.0, 20.0, 100.0, 5000.0] {
        let t = sagged(t0, c);
        assert!((unsagged(t, c) - t0).abs() < 1e-12 * t0.abs().max(t), "{t0}");
        // Ernst: the slope of T over t0 by central differences.
        let slope = (sagged(t0 + 1e-4, c) - sagged(t0 - 1e-4, c)) / 2e-4;
        assert!((slope - 1.0 / (1.0 + c / t.powi(3))).abs() < 1e-6, "{t0}: {slope}");
    }
    let steps = 5000;
    let h = 100.0 / steps as f64;
    let simpson: f64 = (0..=steps)
        .map(|i| {
            let w = if i == 0 || i == steps {
                1.0
            } else if i % 2 == 1 {
                4.0
            } else {
                2.0
            };
            w * sagged(-50.0 + i as f64 * h, c)
        })
        .sum::<f64>()
        * h
        / 3.0;
    let work = pull_work(50.0, c) - pull_work(-50.0, c);
    assert!((work - simpson).abs() < 1e-9 * simpson, "{work} {simpson}");
    assert_eq!((pull_work(50.0, 0.0), pull_work(-50.0, 0.0)), (1250.0, 0.0));

    let robot = planar_with_rope(0.01);
    let spans = robot.spans(&CENTRE).unwrap();
    let s = &spans[0];
    let (l, h) = (s.length / 1.01, 1e-7);
    let (e, c) = (s.length - l, robot.sag(s));
    let t0 = robot.ea * e / l;
    assert!((t0 - 1000.0).abs() < 1e-9 * 1000.0);
    // c at the horizontal part of the straight span, 1.260952 m x 0.785495 = 0.990472 m, times l_f / L = 1.260952 /
    // 1.283539: 77.2386 N^3; T - 1000 N = c / (2 T^2) = 3.86193e-5 N.
    assert!((c - 77.2386).abs() < 2e-3, "{c}");
    assert!((sagged(t0, c) - 1000.0 - 3.86193e-5).abs() < 1e-9);
    let pull = robot.drum_share(sagged(t0, c), t0, c, e, l);
    let energy = |l: f64| robot.spring_energy(s, l);
    assert!((pull + (energy(l + h) - energy(l - h)) / (2.0 * h)).abs() < 1e-5, "{pull}");
    // And slack by 1 % it still hangs: the drum's pull is the energy's derivative there too.
    let l = s.length * 1.01;
    let (e, t0) = (s.length - l, robot.ea * (s.length - l) / l);
    let pull = robot.drum_share(sagged(t0, c), t0, c, e, l);
    assert!(pull > 0.0 && (pull + (energy(l + h) - energy(l - h)) / (2.0 * h)).abs() < 1e-6, "{pull}");
    // At rest the platform and the drums hold with the sagged tensions (the elastic model's equilibrium).
    let (mut state, mut torque) = (Vec::new(), vec![0.0; 4]);
    robot.equilibrium(&CENTRE, &mut state, &mut torque);
    let mut rates = vec![0.0; robot.state_len()];
    robot.derivatives(&state, &torque, &mut rates);
    let (n, m) = (2, 4);
    assert!(rates[n..2 * n].iter().all(|r| r.abs() < 1e-6), "platform: {rates:?}");
    assert!(rates[2 * n + m..2 * n + 2 * m].iter().all(|r| r.abs() < 1e-3), "drums: {rates:?}");
}

/// The sagging cable's pull on the platform against the exact elastic catenary (Irvine's equations, solved apart by
/// Newton in a script of 17 November 2026): cable 1 of the planar robot from its pulley at (-1, 1.6) to the platform at
/// (0.3, 0.9), 1.3 m across and 0.7 m up, 0.01 kg/m, EA 1e5 N, the pulley shrunk to 1 nm so the cable runs straight from
/// the swivel point. With a free length that stretches the straight cable to 20 N (1.47618707 m) the catenary pulls the
/// anchor with (17.7559591, 9.4885599) N (across towards the pulley, up), at 5 N (1.47640849 m) with (5.7838546, 3.0421162) N. The model
/// pulls with T u - rho g l / 2 e_z: T = 20.1665 N and 6.5692 N, within 8.7e-5 N and 1.7e-4 N of the catenary (the
/// parabola's error); the straight cable's 20 N and 5 N would miss it by 0.17 N and 1.6 N, and Ernst's modulus taken as
/// the secant, t0 / (1 + c / t0^3), gave 19.67 N and 2.40 N.
#[test]
fn the_sagging_cable_pulls_as_the_elastic_catenary() {
    let mut cfg = PLANAR.config();
    cfg.set_text("cable.mass_per_m_kg = 0.01").unwrap();
    cfg.set_text("pulley.radius_m = 1.0e-9").unwrap();
    let robot = CableRobot::new(&cfg).unwrap();
    let pose = [0.3, 0.9];
    let (p, r) = robot.place(&pose);
    let spans = robot.spans_at(p, &r).unwrap();
    for (l, want, tol) in [
        (1.4761870686096181, [17.75595909793299, 9.488559851365673], 1e-4),
        (1.47640848559906, [5.783854604182082, 3.04211620717419], 2e-4),
    ] {
        let mut free: Vec<f64> = spans.iter().map(|s| s.length).collect();
        free[0] = l;
        let t = robot.tensions(&spans, &[0.0; 4], &free, &[0.0; 4]).platform[0];
        let weight = robot.cable_weight(&spans[..1], Some(&free[..1])).0;
        // Towards the pulley (-x) and up.
        let pull = [-t * spans[0].u[0], t * spans[0].u[2] - weight[1]];
        assert!((pull[0] - want[0]).abs() < tol && (pull[1] - want[1]).abs() < tol, "{l}: {pull:?} against {want:?}");
    }
}

/// The sagging cables' pull on the platform is the gradient of their energy over its place, the sag's weight moving
/// with the pose included (`sag_pull`): the planar robot elastic at 0.01 kg/m, at rest at (0.3, 0.9) m on free lengths
/// of about 100 N, the platform's weight added back to the dynamics' acceleration times the mass so that it is the
/// cables' force alone; against the fourth-order central difference (10 um) of the springs' energy and the cables'
/// weight at those free lengths (the second-order one is off by 4e-8 N at 1 um, the springs' third derivative, and by
/// 1.3e-7 N at 0.1 um, the energy's rounding). Without the pull it is off by the sag's change with the pose, 1.9e-5 N.
#[test]
fn the_sagging_cables_pull_is_the_gradient_of_their_energy() {
    let mut cfg = PLANAR.config();
    cfg.set_text("cable.mass_per_m_kg = 0.01").unwrap();
    let robot = CableRobot::new(&cfg).unwrap();
    let pose = [0.3, 0.9];
    let spans = robot.spans(&pose).unwrap();
    let free: Vec<f64> = spans.iter().map(|s| s.length / (1.0 + 100.0 / robot.ea)).collect();
    let energy = |pose: &[f64]| {
        let spans = robot.spans(pose).unwrap();
        let springs: f64 = spans.iter().zip(&free).map(|(s, l)| robot.spring_energy(s, *l)).sum();
        springs + robot.cable_weight(&spans, Some(&free)).1
    };
    // The state at rest at the pose on those free lengths, the rotors where the drums are; gravity's share taken off.
    let (mut state, mut torque) = (Vec::new(), vec![0.0; 4]);
    robot.equilibrium(&pose, &mut state, &mut torque);
    state[4..8].copy_from_slice(&free);
    let mut rates = vec![0.0; state.len()];
    robot.derivatives(&state, &torque, &mut rates);
    let mass = robot.moving();
    let force = [mass * rates[2], mass * (rates[3] + G)];
    let mut off = 0f64;
    for i in 0..2 {
        let at = |by: f64| {
            let mut moved = pose;
            moved[i] += by * 1e-5;
            energy(&moved)
        };
        let gradient = (8.0 * (at(1.0) - at(-1.0)) - (at(2.0) - at(-2.0))) / 12e-5;
        off = off.max((force[i] + gradient).abs());
    }
    let t = robot.tensions(&spans, &[0.0; 4], &free, &[0.0; 4]).platform;
    let pull = robot.sag_pull(&spans, &free, &t);
    println!("the force off the energy's gradient by {off:e} N; the sag's pull {pull:?} N");
    assert!(pull.iter().any(|f| f.abs() > 5e-6), "{pull:?}");
    assert!(off < 1e-8, "{off} N");
}

/// The sag constant's gradient over the anchor's place (`sag_gradient`, in closed form) against its central differences
/// (1 um) for every cable of the spatial robot at 202 poses over its box, turned and not, and the planar one at two: as
/// the pull it gives a cable of 100 N at its free length, l / (2 EA T) grad c, within 1e-9 N (measured 1.2e-14 N).
#[test]
fn the_sags_gradient_is_its_differences() {
    let mut worst = 0f64;
    for (entry, poses) in [
        (&SPATIAL, {
            let mut v = vec![vec![0.3, -0.2, 1.3, 0.05, -0.03, 0.08], vec![0.0, 0.0, 1.0, 0.0, 0.0, 0.0]];
            for i in 0..200 {
                let f = |k: f64| (i as f64 * k).sin();
                v.push(vec![0.4 * f(1.3), 0.4 * f(2.1), 1.0 + 0.3 * f(0.7), 0.1 * f(3.3), 0.1 * f(1.9), 0.1 * f(2.9)]);
            }
            v
        }),
        (&PLANAR, vec![vec![0.3, 0.9], vec![-0.4, 0.5]]),
    ] {
        let mut cfg = entry.config();
        cfg.set_text("cable.mass_per_m_kg = 0.01").unwrap();
        let robot = CableRobot::new(&cfg).unwrap();
        for pose in poses {
            let Some(spans) = robot.spans(&pose) else { continue };
            for (k, s) in spans.iter().enumerate() {
                let anchor = sub(s.exit, scale(s.u, s.straight));
                let at = |p: V3| {
                    let (length, straight, u, exit) = span(&robot.cables[k], robot.pulley, p).unwrap();
                    robot.sag(&Span { length, straight, u, exit, b: s.b, dz: 0.0 })
                };
                let exact = robot.sag_gradient(k, s).unwrap();
                let pull = s.length / (2.0 * robot.ea * 100.0);
                for i in 0..3 {
                    let (mut up, mut down) = (anchor, anchor);
                    (up[i], down[i]) = (anchor[i] + 1e-6, anchor[i] - 1e-6);
                    let differences = (at(up) - at(down)) / 2e-6;
                    let off = pull * (exact[i] - differences).abs();
                    worst = worst.max(off);
                }
            }
        }
    }
    println!("the closed form off the differences by {worst:e} N");
    assert!(worst < 1e-9, "{worst} N");
}

/// The cables' swing across themselves (effects.cable_inertia). Planar at the centre: each cable's straight span l_f
/// carries rho l_f / 3 at the anchor across its direction, so the platform's mass grows by sum rho l_f / 3 u_z^2 along x
/// and sum rho l_f / 3 u_x^2 along z (the four alike by symmetry). Its velocity terms are Lagrange's: along any motion at
/// a held twist the swing's kinetic energy changes at the rate twist . (its velocity terms), by central differences
/// over the pose (1e-6 s), spatial at a pose turned and moving and turning, within 1e-9 of it; and the energy closes
/// with it (`the_cables_run_along_with_the_drums`, `testkit::loss_budget`).
#[test]
fn the_cables_swing_with_their_anchors() {
    let mut cfg = PLANAR.config();
    cfg.set_text("effects.cable_inertia = true").unwrap();
    let robot = CableRobot::new(&cfg).unwrap();
    let plain = planar_with_rope(0.003);
    let spans = robot.spans(&CENTRE).unwrap();
    let mu = |s: &Span| 0.003 * s.straight / 3.0;
    let (xx, zz): (f64, f64) =
        (spans.iter().map(|s| mu(s) * s.u[2] * s.u[2]).sum(), spans.iter().map(|s| mu(s) * s.u[0] * s.u[0]).sum());
    let (with, without) = (robot.platform_mass(&CENTRE), plain.platform_mass(&CENTRE));
    assert!((with[0] - without[0] - xx).abs() < 1e-12 && (with[3] - without[3] - zz).abs() < 1e-12, "{with:?}");
    assert!(with[1].abs() < 1e-15 && xx > 1e-3, "{with:?} {xx}");
    let mut cfg = SPATIAL.config();
    cfg.set_text("effects.cable_inertia = true").unwrap();
    let robot = CableRobot::new(&cfg).unwrap();
    let (pose, xi) = (vec![0.2, -0.1, 1.2, 0.05, -0.03, 0.08], vec![0.4, -0.3, 0.2, 0.3, -0.2, 0.5]);
    let swing = |pose: &[f64]| {
        let (all, plain) = (robot.platform_mass(pose), {
            let (_, r) = robot.place(pose);
            robot.platform_mass_at(&r, &[])
        });
        0.5 * (0..36).map(|rc| xi[rc / 6] * (all[rc] - plain[rc]) * xi[rc % 6]).sum::<f64>()
    };
    let h = 1e-6;
    let step = |s: f64| robot.moved(&pose, &xi.iter().map(|x| x * s).collect::<Vec<_>>());
    let rate = (swing(&step(h)) - swing(&step(-h))) / (2.0 * h);
    let power: f64 = robot.swing_bias(&robot.spans(&pose).unwrap(), &xi).iter().zip(&xi).map(|(f, x)| f * x).sum();
    println!("the swing's energy changes at {rate} W, its velocity terms' power {power} W");
    assert!(rate.abs() > 1e-4 && (rate - power).abs() < 1e-6 * rate.abs(), "{rate} {power}");
}

/// effects.cable_inertia: the cable runs along itself at the drum's speed, all of the drum's 5 m and its run of 0.15 m
/// from the drum to the pulley (`frame.run_lengths_m`). Spatial: the drum's 4e-4 / 0.03^2 = 0.444444 kg at the cable and
/// 0.01 x 5.15 = 0.0515 kg of cable, 0.495944 kg. Planar, rigid, at (0.3, 0.9) moving at (0.5, -0.2) m/s: the cable
/// still on each drum, 0.003 (5 - L), turns with it (its spin about h, (J / r^2 + rho (5 - L)) r L'), and the cables
/// are in the frame's mass, 4 x 0.003 x 5.15 = 0.0618 kg more, and momentum: against the material's own, summed over 4000
/// points of each span (velocity l' u at the pulley to the anchor's at the anchor, the span straight from the pulley's
/// swivel point) and the run's (rho run l' along it, at its middle), to 1e-9. The first moment moves at the momentum and
/// the weight sits at it (testkit), rigid and elastic. The energy closes with it (`testkit::loss_budget`), as with the
/// lift of the cable from the drums 0.5 m under the bottom pulleys and over the top ones (runs of 0.5 m along the
/// winches' directions, -z and +z), which pulls each bottom drum back by 0.01 x 9.80665 x 0.5 = 0.0490333 N and each top
/// one on by as much.
#[test]
fn the_cables_run_along_with_the_drums() {
    let mut cfg = SPATIAL.config();
    cfg.set_text("effects.cable_inertia = true").unwrap();
    let spatial = CableRobot::new(&cfg).unwrap();
    assert!(spatial.drum_mass.iter().all(|m| (m - 0.495_944).abs() < 1e-6), "{:?}", spatial.drum_mass);
    assert!(CableRobot::new(&SPATIAL.config()).unwrap().drum_mass.iter().all(|m| (m - 0.444_444).abs() < 1e-6));
    for stretch in [false, true] {
        let mut cfg = PLANAR.config();
        cfg.set_text(&format!("effects.cable_stretch = {stretch}")).unwrap();
        let plain = CableRobot::new(&cfg).unwrap();
        cfg.set_text("effects.cable_inertia = true").unwrap();
        let robot = CableRobot::new(&cfg).unwrap();
        let (pose, vel) = ([0.3, 0.9], [0.5, -0.2]);
        let state = |robot: &CableRobot| {
            let (mut state, mut torque) = (Vec::new(), vec![0.0; 4]);
            robot.equilibrium(&pose, &mut state, &mut torque);
            robot.set_tool_velocity(&mut state, &vel);
            state
        };
        let s = state(&robot);
        let (with, without) = (robot.frame_momentum(&s).unwrap(), plain.frame_momentum(&state(&plain)).unwrap());
        assert!((with.mass - without.mass - 0.0618).abs() < 1e-12, "{} kg", with.mass - without.mass);
        let spans = robot.spans(&pose).unwrap();
        let (free, rates, ..) = robot.winch_state(&s, &spans);
        let (p, v) = ([pose[0], 0.0, pose[1]], [vel[0], 0.0, vel[1]]);
        let (mut linear, mut angular) = ([0.0; 3], [0.0; 3]);
        for k in 0..4 {
            let (e, axis, rho) = (robot.cables[k].exit, robot.cables[k].axis, 0.003);
            angular[1] += robot.spin[k][1] * rho * (5.0 - free[k]) * 0.025 * rates[k];
            // The run: rho 0.15 moving at l' from the drum towards the pulley, at its middle.
            let (mid, along) = (add(e, scale(axis, 0.075)), scale(axis, -rates[k]));
            let run = cross(mid, scale(along, rho * 0.15));
            (0..3).for_each(|i| (linear[i], angular[i]) = (linear[i] + rho * 0.15 * along[i], angular[i] + run[i]));
            // The span: 4000 points from the pulley's swivel point to the anchor, each rho l / 4000.
            let d = sub(p, e);
            let enter = scale(d, rates[k] / free[k]);
            for j in 0..4000 {
                let f = (j as f64 + 0.5) / 4000.0;
                let (at, speed) = (add(e, scale(d, f)), add(scale(enter, 1.0 - f), scale(v, f)));
                let piece = rho * free[k] / 4000.0;
                let turn = cross(at, scale(speed, piece));
                (0..3).for_each(|i| (linear[i], angular[i]) = (linear[i] + piece * speed[i], angular[i] + turn[i]));
            }
        }
        for i in 0..3 {
            assert!((with.linear[i] - without.linear[i] - linear[i]).abs() < 1e-9, "{stretch} {i}: {with:?}");
            assert!((with.angular[i] - without.angular[i] - angular[i]).abs() < 1e-9, "{stretch} {i}: {with:?}");
        }
        assert!(linear[0].abs() > 1e-4 && angular[1].abs() > 1e-4, "{linear:?} {angular:?}");
        sim_core::testkit::frame_momentum_moves_its_first_moment(&robot);
        sim_core::testkit::weight_is_at_the_first_moment(&robot);
        // Tighter: the weight is the first moment's exactly, the span's mass rho l in both (elastic, l the free length,
        // a state; rigid l = L): V + gravity . c is the same at another pose with other free lengths, to rounding.
        let held = |robot: &CableRobot, s: &[f64]| {
            let f = robot.frame_momentum(s).unwrap();
            robot.energy_parts(s)[1] + (0..3).map(|i| f.gravity[i] * f.first_moment[i]).sum::<f64>()
        };
        let mut other = Vec::new();
        robot.equilibrium(&[-0.2, 0.6], &mut other, &mut [0.0; 4]);
        if stretch {
            let n = robot.n();
            (0..4).for_each(|k| other[2 * n + k] *= 1.001);
        }
        let (a, b) = (held(&robot, &s), held(&robot, &other));
        assert!((a - b).abs() < 1e-12 * a.abs().max(1.0), "{stretch}: {a} against {b}");
    }
    let mut cfg = PLANAR.config();
    cfg.set_text("cable.mass_per_m_kg = 0.01").unwrap();
    let flat = CableRobot::new(&cfg).unwrap();
    cfg.set_text("effects.cable_inertia = true").unwrap();
    cfg.set_text("frame.run_lengths_m = [0.5, 0.5, 0.5, 0.5]").unwrap();
    let dropped = CableRobot::new(&cfg).unwrap();
    let spans = flat.spans(&[0.3, 0.9]).unwrap();
    let free: Vec<f64> = spans.iter().map(|s| s.length / (1.0 + 100.0 / flat.ea)).collect();
    let drum = |robot: &CableRobot| robot.tensions(&spans, &[0.0; 4], &free, &[0.0; 4]).drum;
    for (k, (a, b)) in drum(&dropped).iter().zip(drum(&flat)).enumerate() {
        let up = dropped.cables[k].axis[2];
        assert!((a - b - 0.049_033_25 * up).abs() < 1e-12, "{k}: {a} N against {b} N");
    }
    for stretch in [false, true] {
        let mut cfg = SPATIAL.config();
        for edit in ["effects.cable_inertia = true", &format!("effects.cable_stretch = {stretch}")] {
            cfg.set_text(edit).unwrap();
        }
        sim_core::testkit::loss_budget(&CableRobot::new(&cfg).unwrap());
    }
}

/// The winches' spin in the frame's load: the platform's moment about y at (0.3, 0.9) moving at (0.5, -0.2) m/s (m p x
/// v) plus each winch's drum and rotor, h (m_drum r L' + J_rotor i L'), with the default axes: horizontal and square to
/// the cable's direction at the winch, in the planar plane about y: +y for the top left and the bottom right cable, -y
/// for the others (h = ref_y = winch direction x the pulley's direction towards the centre; mirror-image winches spin
/// mirror-image ways, so at the centre they cancel). The cables' rates by hand (each length r beta + l_f in its pulley's
/// plane, by central differences in a script of 17 November 2026): L' = (0.535585, -0.215854, -0.466109, 0.300346) m/s,
/// so sum h_y L' = 0.535585 + 0.215854 - 0.466109 - 0.300346 = -0.0150160 m/s, times the drum's J / r and the rotor's
/// J i per metre of cable. The spatial robot's axes are horizontal and square to the winch direction.
#[test]
fn the_winches_spin_is_in_the_frames_momentum() {
    // Rigid, so that the drums pay out exactly L' (elastic, l' differs from it by the strain's t / EA, 1e-3).
    let mut cfg = PLANAR.config();
    cfg.set_text("effects.cable_stretch = false").unwrap();
    let robot = CableRobot::new(&cfg).unwrap();
    for (k, want) in [1.0, -1.0, 1.0, -1.0].into_iter().enumerate() {
        assert_eq!(robot.spin[k], [0.0, want, 0.0], "{k}");
    }
    let (pose, vel) = ([0.3, 0.9], [0.5, -0.2]);
    let (mut state, mut torque) = (Vec::new(), vec![0.0; 4]);
    robot.equilibrium(&pose, &mut state, &mut torque);
    robot.set_tool_velocity(&mut state, &vel);
    let f = robot.frame_momentum(&state).unwrap();
    let w = &robot.winches[0];
    let spin = (robot.drum_mass[0] * 0.025 + w.rotor_inertia * w.ratio()) * -0.0150160;
    let platform = 5.0 * (pose[1] * vel[0] - pose[0] * vel[1]);
    // The four rates' six digits: 2e-6 m/s.
    assert!((f.angular[1] - platform - spin).abs() < 2e-6 * spin.abs() / 0.015016, "{f:?} {platform} {spin}");
    let spatial = CableRobot::new(&SPATIAL.config()).unwrap();
    for (h, c) in spatial.spin.iter().zip(&spatial.cables) {
        assert!(h[2].abs() < 1e-12 && dot(*h, c.axis).abs() < 1e-12 && (norm(*h) - 1.0).abs() < 1e-12, "{h:?}");
    }
}

/// The planar winches' steady friction at the centre, moving across at 1 m/s, by hand: each cable runs at |L'| =
/// |u_x| x 1 m/s = 0.785495 m/s (`centre_cable`), the left ones (1 and 4) paying out, the right ones hauling in; the
/// winch's ratio 5 / 0.025 m = 200 rad/m turns the rotor at 157.0990 rad/s. Shaft: 0.05 tanh(314.2) + 2e-4 x 157.0990 =
/// 0.0814198 N m; the drum's bearings 2 N + 1 N s/m x 0.785495 m/s = 2.785495 N at the cable. The cable's tension
/// (100 N to 140 N) loads every gearbox the same way, against the drum: hauling in, the motor drives, f / (200 x 0.95)
/// = 0.0146605 N m, so -(0.0814198 + 0.0146605) = -0.0960803 N m; paying out, the cable drives the gearbox back, which
/// takes eta off the friction, f x 0.95 / 200 = 0.0132311 N m, so 0.0946509 N m. The gears' mesh friction
/// (effects.gearbox_mesh_friction) adds (1 - 0.95) of the torque each gear holds, t / 200 at the rotor, along the motion:
/// 0.0349519 N m on the top cables (139.807643 N), 0.0250481 N m on the bottom ones (100.192357 N).
#[test]
fn the_planar_winches_steady_friction_at_the_centre_is_the_hand_calculated_one() {
    let mut out = [0.0; 4];
    planar().steady_friction(&CENTRE, &[1.0, 0.0], &mut out);
    for (got, want) in out.iter().zip([0.1296028, -0.1310322, -0.1211284, 0.1196990]) {
        assert!((got - want).abs() < 1e-6, "{out:?}");
    }
}

/// The friction view's winch contacts are in N against the cable's m/s (the winch is a joint in metres of cable): at
/// 0.5 m/s the planar winch's bearings and pulley take 2 N tanh(0.5 / 0.002) = 2 N and 1 N s/m x 0.5 m/s = 0.5 N, and
/// the contact is a sliding one, as its signals' units say.
#[test]
fn the_winch_contacts_are_sliding_ones_in_newtons_at_the_cable() {
    let robot = planar();
    assert_eq!(robot.friction_curve(0, 0.5, 1.0), [2.0, 0.5, 0.0]);
    assert!((0..4).all(|i| !robot.friction_rotary(i)));
    let infos = robot.signal_infos();
    let unit = |id: &str| infos.iter().find(|s| s.id == id).map(|s| s.unit.clone());
    let id = robot.friction_contacts()[0].id;
    assert_eq!(unit(&format!("{id}.friction_speed")).as_deref(), Some("m/s"));
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
/// elastic `forward` now starts Newton from a grid point's pose (`ANCHOR_STEP`) and stops within the same tolerance, a
/// change at the rounding of the pose, which the closed loop amplifies, as on other parallel machines with a
/// warm-started pose: it moved these by up to 1.5e-5 (the planar machine's fourth motor's peak torque). Another
/// platform's maths library rounds sine and arc tangent differently in the last bit, which moves them the same way.
/// Hence 1e-3. Recorded again on 16 November 2026 after the gears' mesh-friction bristle took its stiffness in N/m
/// (`Converter::apply`; it had K1's number in N m/rad, 1/r^2 = 1600 times too soft): the stiff bristle sticks the
/// drums, which moves torques and energies by up to 2.4 % (cable-spatial's motor 2 peak torque 2.0888 to 2.1381 N m)
/// and the path and tracking errors by up to 12 % (its tracking error 0.269 to 0.300 mm; the planar machine's
/// tracking error 0.647 to 0.663 mm). Recorded again the same day for cable-spatial's speed loops at Tn 5 ms (was 30 ms;
/// config/spatial.toml), which break the stuck meshes away sooner, and the machine friction feedforward with the
/// meshes' friction on the cables' tension at rest (`steady_friction`): torques and energies move by up to 1.3 %
/// (cable-spatial's motor 7 peak torque 2.2962 to 2.2755 N m; the planar machine's by up to 0.2 %), cable-spatial's
/// tracking error falls from 0.300 to 0.204 mm, the turn's path error from 1.65 to 1.58 mrad, its tracking error stays
/// 1.65 to 1.67 mrad. Recorded again on 17 November 2026 with the cables' own weight and sag (3 g/m and 10 g/m): with
/// the mass at 0 the figures stay within 0.07 % of the former ones (what the other merged work moved); the cables' weight,
/// 0.08 N and 0.6 N against the platforms' 49 N and 78 N and a pretension of 120 N and 250 N, moves them by up to 0.11 %
/// (cable-spatial's motor 5 energy fed back 31.994 to 31.959 J), the planar machine's by up to 0.05 %.
#[test]
fn the_default_cycles_keep_their_figures() {
    const PLANAR_BEFORE: [f64; 22] = [
        1.0370485242274168,
        0.6508117633620196,
        55.17074132648752,
        37.62218302428801,
        1.6490574996012395,
        0.7725141315186166,
        216.69221782653418,
        119.30271639252666,
        0.9710668455137978,
        0.5069421387189742,
        44.46119540264424,
        28.350038445318305,
        1.1531464199209551,
        0.6344471451377474,
        193.45184955935562,
        98.69696825893328,
        0.0005993141046528103,
        0.00014536686574640457,
        0.0006498910195019717,
        0.0,
        0.0,
        0.0,
    ];
    const SPATIAL_BEFORE: [f64; 38] = [
        2.454133613315348,
        1.5503603381666788,
        169.73192527089893,
        114.97304826816891,
        2.1125494742619084,
        1.6356823599823862,
        27.33119601811953,
        15.92704194375928,
        1.68730677963562,
        1.2404732469425892,
        47.32061500545322,
        40.57375227106037,
        2.0738007655320354,
        1.654344101628969,
        88.52814522830276,
        66.02519414487313,
        1.7958799380787411,
        1.1634477024704413,
        50.99647866203367,
        31.959193635043288,
        1.9531031622521784,
        1.4512157360016786,
        83.70309210762065,
        55.91233507264243,
        2.2750673054987782,
        1.4010697752139403,
        148.41568393016928,
        106.67087268260978,
        1.7210925703692952,
        1.4050289879979878,
        25.5564783335091,
        15.007474038025647,
        0.00020446879027836694,
        6.644530169754617e-5,
        0.00020446879027836694,
        0.0015769142437619267,
        0.00046632771898633764,
        0.001672342092489491,
    ];
    let (planar, spatial) = (cycle_figures(&PLANAR), cycle_figures(&SPATIAL));
    let tol = 1e-3;
    assert_figures("cable-planar", &planar, &PLANAR_BEFORE, tol);
    assert_figures("cable-spatial", &spatial, &SPATIAL_BEFORE, tol);
}

/// The time-optimal plan keeps every cable in its band (`tension_band`, `profile.tension_reserve` 0.8 of the room): the
/// planar default cycle timed as fast as the motors allow takes 1.684 s (2.148 s within the axis limits), and no cable
/// falls below its 10 N or passes its 400 N. Planned without the band it took 1.653 s, and cable 4 went down to 0.63 N
/// (17 November 2026): the band costs 1.9 % of the cycle here, 5.6 % on cable-spatial (1.143 s to 1.207 s).
#[test]
fn the_time_optimal_cycle_keeps_the_cables_in_their_band() {
    let mut cfg = PLANAR.config();
    cfg.set_text("profile.cycle_timing = \"time_optimal\"").unwrap();
    let mut sim = sim_components::Sim::new(&PLANAR, &cfg).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    assert!(!load.tripped && (load.duration - 1.684).abs() < 0.01, "{}", load.duration);
    for v in verdicts(&cfg, &load).unwrap() {
        assert!(v.ok, "{} of motor {:?} is {} against {}", v.check, v.motor, v.value, v.limit);
    }
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
