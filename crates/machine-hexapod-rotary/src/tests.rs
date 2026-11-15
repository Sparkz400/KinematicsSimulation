use super::*;
use sim_components::analysis::{CycleLoad, analyse_cycle, verdicts};

#[test]
fn the_platform_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ENTRY);
}

fn mechanism(cfg: &Config) -> RotaryPlatform {
    RotaryPlatform::from_config(cfg).unwrap().1
}

const HOME: [f64; 6] = [0.0, 0.0, 0.4, 0.0, 0.0, 0.0];

/// Leg 1 at the level pose (0, 0, 0.4 m): its crank axis' centre B at 0.5 m and -5 degrees, its zero direction d the
/// clockwise tangent of the base radius turned by -30 degrees, d = (-0.573576, -0.819152, 0); its platform joint P at
/// 0.3 m and -55 degrees, 0.4 m up. e = P - B = (-0.326024, -0.202168, 0.4): a = e . d = 0.352606, b = 0.4,
/// c = (|e|^2 + 0.2^2 - 0.55^2) / (2 x 0.2) = 0.111659, sqrt(a^2 + b^2) = 0.533227, so theta = atan2(0.4, 0.352606) -
/// acos(0.111659 / 0.533227) = 0.848288 - 1.359832 = -0.5115438 rad (-29.309 degrees), the same for all six by
/// symmetry, and the rod from the tip to P is 0.55 m long.
#[test]
fn every_crank_has_the_hand_calculated_angle_at_the_level_pose() {
    let geo = mechanism(&ENTRY.config()).geo;
    let mut q = [0.0; 6];
    assert!(geo.inverse(&HOME, &mut q));
    for (i, v) in q.iter().enumerate() {
        assert!((v + 0.5115438).abs() < 1e-7, "crank {i} at {v}");
    }
    for leg in geo.legs(&HOME).unwrap() {
        assert!((norm(sub(leg.joint, leg.tip)) - 0.55).abs() < 1e-12);
    }
    // The other branch ("in") turns the crank up past the line to P: 0.848288 + 1.359832 = 2.208120 rad.
    let mut cfg = ENTRY.config();
    cfg.set_text("crank.branch = \"in\"").unwrap();
    assert!(mechanism(&cfg).geo.inverse(&HOME, &mut q));
    assert!((q[0] - 2.208120).abs() < 1e-6, "{q:?}");
}

/// The closed-form inverse and the Newton forward kinematics undo each other at three poses: level, a shift with a
/// tilt, and a far corner with all three turns.
#[test]
fn the_forward_kinematics_undoes_the_inverse_at_three_poses() {
    let geo = mechanism(&ENTRY.config()).geo;
    for pose in [HOME, [0.04, -0.02, 0.42, 0.05, 0.0, -0.03], [-0.06, 0.06, 0.37, -0.1, 0.08, 0.15]] {
        let (mut q, mut back) = ([0.0; 6], [0.0; 6]);
        assert!(geo.inverse(&pose, &mut q));
        geo.forward(&q, &mut back);
        for k in 0..6 {
            assert!((pose[k] - back[k]).abs() < 1e-12, "{pose:?} came back as {back:?}");
        }
    }
    // 0.3 m up the rods cannot reach: |c| > sqrt(a^2 + b^2).
    let mut q = [0.0; 6];
    assert!(!geo.inverse(&[0.0, 0.0, 0.7, 0.0, 0.0, 0.0], &mut q));
}

/// Holding the level platform with its centred load at rest: the platform 35 kg and the seat 80 kg (115 kg, their
/// centres of mass on the platform's axis) and of each rod the upper sixth and half its middle, 0.4 / 2 = 0.2 kg,
/// all carried by the rods' forces alike: f = (115 + 6 x 0.2) g / (6 u_z), u_z = 0.905281 (u = (-0.410898, -0.107835,
/// 0.905281) from the tip C = B + 0.2 (cos theta d + sin theta z) to P): 1139.53 / 5.431687 = 209.7935 N of
/// compression. The crank takes it with its torque arm u . t = 0.189602 m, t = 0.2 (-sin theta d + cos theta z), plus
/// its own weight, 2 kg at 0.08 m, and the rod's lower sixth and half its middle at its tip, 0.2 kg at 0.2 m, both
/// cos theta = 0.871990 times g: 209.7935 x 0.189602 + 0.871990 x 9.80665 x (0.16 + 0.04) = 39.7772 + 1.7103 =
/// 41.4875 N m at the crank, 41.4875 / 50 = 0.829750 N m at each motor (nothing lost at rest; the cogging off).
#[test]
fn the_cranks_hold_the_level_platform_with_the_hand_calculated_torque() {
    let mut cfg = ENTRY.config();
    cfg.set_text("effects.torque_ripple = false").unwrap();
    let mech = mechanism(&cfg);
    let mut q = [0.0; 6];
    assert!(mech.geo.inverse(&HOME, &mut q));
    let f = mech.rod_forces(&q, &[0.0; 6], &[0.0; 6]).unwrap();
    for v in f {
        assert!((v - 209.7935).abs() < 1e-3, "{f:?}");
    }
    let machine = (ENTRY.build)(&cfg).unwrap();
    let (mut state, mut torque) = (Vec::new(), vec![0.0; 6]);
    machine.equilibrium(&HOME, &mut state, &mut torque);
    for t in &torque {
        assert!((t - 0.829750).abs() < 1e-5, "{torque:?}");
    }
}

/// The torque arm m = u . t vanishes at the dead centre. With theta = atan2(b, a) - delta, the tip leaves e = P - B
/// at the angle delta = acos(c / sqrt(a^2 + b^2)) in the crank's plane, and u . t = e . t / L = r sqrt(a^2 + b^2)
/// sin(delta) / L. Lifting the level platform, leg 1 keeps a = 0.352606 and |e|^2 = 0.147164 + z^2 (as above): at
/// z = 0.5 m, c = 0.336659, sqrt(a^2 + b^2) = 0.611826, delta = 56.6156 degrees and m = 0.2 x 0.611826 x
/// sin(56.6156) / 0.55 = 0.185772 m; it reaches its dead centre where c = sqrt(a^2 + b^2), z^2 the root of
/// (w + K)^2 = 4 r^2 (a^2 + w), K = 0.147164 + 0.04 - 0.3025 = -0.115336: z = 0.6378641 m, where m = 0 (the arm grows as
/// the root of the distance below it). All six reach it at once at this pose, so the inverse Jacobian's rows grow
/// alike and its condition number does not see it: the working range refuses the pose by its margin from the dead
/// centre (`workspace.min_dead_centre_margin_deg`).
#[test]
fn the_torque_arm_goes_to_zero_at_the_dead_centre() {
    let mech = mechanism(&ENTRY.config());
    let arm = |z: f64| mech.geo.legs(&[0.0, 0.0, z, 0.0, 0.0, 0.0]).unwrap()[0].arm;
    assert!((arm(0.5) - 0.185772).abs() < 1e-6, "{}", arm(0.5));
    let top = 0.6378640947122;
    assert!(arm(top).abs() < 1e-5, "{}", arm(top));
    assert!(arm(top - 0.01) > 4.0 * arm(top - 0.0005).abs());
    let mut q = [0.0; 6];
    assert!(!mech.geo.inverse(&[0.0, 0.0, top + 1e-4, 0.0, 0.0, 0.0], &mut q));
    let pose = [0.0, 0.0, top - 0.002, 0.0, 0.0, 0.0];
    assert_eq!(Mechanism::unreachable(&mech, &pose, 0.0), Some("error-rss-dead-centre"));
    assert_eq!(Mechanism::unreachable(&mech, &HOME, 0.0), None);
}

/// The inverse Jacobian at the level pose, row 1: (u, b x u) / (u . t) with u and the arm 0.189602 m as above and
/// b = (0.172073, -0.245746, 0): (-2.167162, -0.568745, 4.774645, -1.173348, -0.821587, -0.630436). A heave of 1 m/s
/// turns every crank at 0.905281 / 0.189602 = 4.774645 rad/s; the closed form's slope along z agrees.
#[test]
fn the_jacobian_at_the_level_pose_is_the_hand_calculated_one() {
    let geo = mechanism(&ENTRY.config()).geo;
    let j = Geometry::inverse_jacobian(&geo.legs(&HOME).unwrap());
    for (got, want) in j[..6].iter().zip([-2.167162, -0.568745, 4.774645, -1.173348, -0.821587, -0.630436]) {
        assert!((got - want).abs() < 2e-6, "{:?}", &j[..6]);
    }
    let h = 1e-6;
    let (mut up, mut down) = ([0.0; 6], [0.0; 6]);
    geo.inverse(&[0.0, 0.0, 0.4 + h, 0.0, 0.0, 0.0], &mut up);
    geo.inverse(&[0.0, 0.0, 0.4 - h, 0.0, 0.0, 0.0], &mut down);
    for i in 0..6 {
        assert!(((up[i] - down[i]) / (2.0 * h) - 4.774645).abs() < 1e-5);
        assert!((j[i * 6 + 2] - 4.774645).abs() < 1e-6);
    }
}

/// A heave of 1 m/s^2 from rest at the level pose: each crank at theta'' = u_z / m = 4.774645 rad/s^2 (above). The
/// rods' compression grows by (115 + 6 (0.4/6 + 0.4/3 x (1 + t_z theta'') / 2)) / (6 u_z), t_z = r cos theta =
/// 0.174398 m: (115 + 6 (0.066667 + 0.133333 x 0.916349)) / 5.431687 = 21.3807 N; the crank's torque by 0.02 x
/// 4.774645 (its inertia) + t . (0.4/6 a_C + 0.4/3 a_mid) (a_C = t theta'', a_mid = (a_C + (0, 0, 1)) / 2) +
/// 21.3807 x 0.189602 = 0.095493 + 0.037088 + 4.053815 = 4.186396 N m; at the motor 4.186396 / 50 plus the rotor and
/// the gearbox's input, (0.56 + 0.54) kg cm^2 = 1.1e-4 kg m^2 x 50 x 4.774645 = 0.026261: 0.109989 N m more than at rest.
#[test]
fn a_heave_acceleration_takes_the_hand_calculated_torque() {
    let machine = (ENTRY.build)(&ENTRY.config()).unwrap();
    let (mut moving, mut resting) = ([0.0; 6], [0.0; 6]);
    machine.feedforward_torque(&HOME, &[0.0, 0.0, 1.0, 0.0, 0.0, 0.0], &mut moving);
    machine.feedforward_torque(&HOME, &[0.0; 6], &mut resting);
    for k in 0..6 {
        assert!((moving[k] - resting[k] - 0.109989).abs() < 2e-5, "motor {k}: {}", moving[k] - resting[k]);
    }
}

/// The rod's Euler load, both ends in ball joints: an aluminium tube 20 x 2 mm, I = pi (0.02^4 - 0.016^4) / 64 =
/// 4.636991e-9 m^4, E = 70 GPa, L = 0.55 m: F = pi^2 E I / L^2 = 10590.31 N, over the safety factor 3: 3530.10 N.
#[test]
fn the_rods_buckle_at_the_hand_calculated_euler_load() {
    assert!((mechanism(&ENTRY.config()).buckling_limit() - 3530.103).abs() < 1e-2);
}

/// The new verdicts at their limits, and the signals they judge at the level pose by hand. Leg 1 (as above): the rod's
/// angle from the crank's tangent acos(0.189602 / 0.2) = 18.5567 degrees; out of the crank's plane, against its axis
/// k = d x z = (-0.819152, 0.573576, 0), asin(|u . k|) = 15.9463 degrees; against the platform's normal acos(0.905281) =
/// 25.1388 degrees. The limits: 90 - 20 = 70 degrees from the tangent, the Euler load over 3, the ball joints' 35 and
/// 40 degrees. A verdict passes at its limit and fails just beyond it.
#[test]
fn the_new_verdicts_pass_at_their_limits_and_fail_beyond() {
    let cfg = ENTRY.config();
    let (machine, mech) = RotaryPlatform::from_config(&cfg).unwrap();
    let legs = mech.geo.legs(&HOME).unwrap();
    let a = mech.geo.angles(&legs, &Geometry::rotation(&HOME));
    for (got, want) in a[0].iter().zip([18.5567, 15.9463, 25.1388]) {
        assert!((got.to_degrees() - want).abs() < 1e-4, "{:?}", a[0]);
    }
    let checks = machine.rating_checks();
    assert_eq!(checks.len(), 24);
    let limits = [70f64.to_radians(), 3530.103, 35f64.to_radians(), 40f64.to_radians()];
    for (check, limit) in checks.iter().take(4).zip(limits) {
        assert!((check.limit - limit).abs() < 1e-2 * limit.min(1.0) * 1e-3 + 1e-2, "{check:?}");
    }
    for factor in [1.0, 1.0 + 1e-9] {
        let load = CycleLoad {
            checks: checks.iter().map(|c| (c.clone(), c.limit * factor)).collect(),
            ..CycleLoad::default()
        };
        let judged = verdicts(&cfg, &load).unwrap();
        for name in ["crank-dead-centre", "rod-buckling", "crank-ball-joint", "platform-ball-joint"] {
            let ours: Vec<_> = judged.iter().filter(|v| v.check == name).collect();
            assert_eq!(ours.len(), 6, "{name}");
            assert!(ours.iter().all(|v| v.ok == (factor == 1.0)), "{name} at {factor}");
        }
    }
}

/// The dynamics by virtual work against Newton and Euler on the crank: in a pose off centre, moving and accelerating,
/// each crank's torque M theta'' + h is its inertia times theta'', its weight, the rod's lower sixth and half its
/// middle at its tip (t . m (a - g)) and the rod's compression times the torque arm (`rod_forces`, from the platform's
/// side alone).
#[test]
fn the_virtual_work_agrees_with_the_forces_along_the_rods() {
    let mech = mechanism(&ENTRY.config());
    let pose = [0.03, -0.02, 0.42, 0.06, -0.04, 0.08];
    let mut q = [0.0; 6];
    assert!(mech.geo.inverse(&pose, &mut q));
    let qd = [0.4, -0.3, 0.2, 0.5, -0.6, 0.1];
    let qdd = [3.0, -2.0, 1.0, -4.0, 2.5, 0.5];
    let (mut mass, mut bias) = (vec![0.0; 36], vec![0.0; 6]);
    mech.dynamics(&q, &qd, &mut mass, &mut bias);
    let f = mech.rod_forces(&q, &qd, &qdd).unwrap();
    let m = mech.motion(&q, &qd).unwrap();
    let xi_acc: Vec<f64> = (0..6).map(|r| m.xi0[r] + (0..6).map(|c| m.k[r * 6 + c] * qdd[c]).sum::<f64>()).collect();
    let (w, w_acc) = ([m.xi[3], m.xi[4], m.xi[5]], [xi_acc[3], xi_acc[4], xi_acc[5]]);
    for (i, leg) in m.legs.iter().enumerate() {
        let tau: f64 = (0..6).map(|c| mass[i * 6 + c] * qdd[c]).sum::<f64>() + bias[i];
        let a_low = add(scale(leg.t, qdd[i]), scale(leg.t2, qd[i] * qd[i]));
        let a_high = add(add([xi_acc[0], xi_acc[1], xi_acc[2]], cross(w_acc, leg.b)), cross(w, cross(w, leg.b)));
        let a_mid = scale(add(a_low, a_high), 0.5);
        let need = |a: V3| [a[0], a[1], a[2] + G];
        let node = add(scale(need(a_low), mech.rod_mass / 6.0), scale(need(a_mid), mech.rod_mass / 3.0));
        let newton = mech.crank_inertia * qdd[i]
            + mech.crank_mass * G * mech.crank_cg * q[i].cos()
            + dot(leg.t, node)
            + f[i] * leg.arm;
        assert!((tau - newton).abs() < 1e-9 * tau.abs().max(1.0), "crank {i}: {tau} against {newton}");
    }
}

/// A pose that takes a rod beyond its reach is refused, one that tilts the ball joints beyond their limits too.
#[test]
fn poses_beyond_the_working_range_are_refused() {
    let machine = (ENTRY.build)(&ENTRY.config()).unwrap();
    assert_eq!(machine.unreachable(&HOME, 0.0), None);
    assert_eq!(machine.unreachable(&[0.0, 0.0, 0.7, 0.0, 0.0, 0.0], 0.0), Some("error-target-out-of-range"));
    assert_eq!(machine.unreachable(&[0.2, 0.0, 0.35, 0.0, 0.0, 0.0], 0.0), Some("error-rss-ball-joint"));
}

/// The default cycle passes every verdict: the default is a sized example.
#[test]
fn the_default_cycle_passes_its_verdicts() {
    let cfg = ENTRY.config();
    let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    assert!(!load.tripped);
    for v in verdicts(&cfg, &load).unwrap() {
        println!("{} {:?}: {} against {}", v.check, v.motor, v.value, v.limit);
    }
    for v in verdicts(&cfg, &load).unwrap() {
        assert!(v.ok, "{} of motor {:?} is {} against {}", v.check, v.motor, v.value, v.limit);
    }
}
