use super::*;
use sim_components::analysis::analyse_cycle;

#[test]
fn the_hexapod_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ENTRY);
}

fn mechanism(cfg: &Config) -> Hexapod {
    Hexapod::from_config(cfg).unwrap().1
}

/// At the home pose (0, 0, 0.48 m, no turn) every leg spans the same: base joint at 0.4 m and -15 degrees, platform
/// joint at 0.2 m and -50 degrees, 35 degrees apart. Seen from above they are d^2 = 0.4^2 + 0.2^2 - 2 0.4 0.2 cos 35 =
/// 0.0689357 m^2 apart, so L = sqrt(0.48^2 + 0.0689357) = 0.5471158 m and the cylinder is out by L - 0.45 = 0.0971158 m.
#[test]
fn every_leg_has_the_hand_calculated_length_at_the_home_pose() {
    let geo = mechanism(&ENTRY.config()).geo;
    let mut q = [0.0; 6];
    assert!(geo.inverse(&[0.0, 0.0, 0.48, 0.0, 0.0, 0.0], &mut q));
    for (i, v) in q.iter().enumerate() {
        assert!((v - 0.0971158).abs() < 1e-7, "leg {i} is out by {v}");
    }
}

/// The closed-form inverse and the Newton forward kinematics undo each other at three poses: home, a shift with a
/// tilt, and the far corner of the box with all three turns.
#[test]
fn the_forward_kinematics_undoes_the_inverse_at_three_poses() {
    let geo = mechanism(&ENTRY.config()).geo;
    for pose in
        [[0.0, 0.0, 0.48, 0.0, 0.0, 0.0], [0.04, -0.02, 0.5, 0.05, 0.0, -0.03], [-0.06, 0.06, 0.44, -0.08, 0.08, 0.12]]
    {
        let (mut q, mut back) = ([0.0; 6], [0.0; 6]);
        assert!(geo.inverse(&pose, &mut q));
        geo.forward(&q, &mut back);
        for k in 0..6 {
            assert!((pose[k] - back[k]).abs() < 1e-12, "{pose:?} came back as {back:?}");
        }
    }
    // Legs that no pose fits (one 1 m out, the others closed) are refused, not answered with a wrong pose.
    let mut back = [0.0; 6];
    geo.forward(&[1.0, 0.0, 0.0, 0.0, 0.0, 0.0], &mut back);
    assert!(back.iter().all(|v| v.is_nan()));
}

/// Holding the platform at rest at home, J^T f = (0, 0, m g, 0, 0, 0). By symmetry the six legs push alike, each with
/// its vertical share u_z = 0.48 / 0.5471158: f = m g / (6 u_z), m = 25 kg platform + 6 x 0.3 x 4 kg of legs + 30 kg
/// payload = 62.2 kg: 62.2 x 9.80665 / (6 x 0.877323) = 115.8772 N. The machine holds it with f lead / (2 pi) =
/// 0.184424 N m on each motor (coupling ratio 1, nothing lost at rest; the cogging switched off, which would add its
/// ripple at the rotor's angle).
#[test]
fn the_legs_carry_the_hand_calculated_static_forces() {
    let mut cfg = ENTRY.config();
    cfg.set_text("effects.torque_ripple = false").unwrap();
    let home = [0.0, 0.0, 0.48, 0.0, 0.0, 0.0];
    let f = mechanism(&cfg).static_leg_forces(&home).unwrap();
    for v in f {
        assert!((v - 115.8772).abs() < 1e-3, "{f:?}");
    }
    let machine = (ENTRY.build)(&cfg).unwrap();
    let (mut state, mut torque) = (Vec::new(), vec![0.0; 6]);
    machine.equilibrium(&home, &mut state, &mut torque);
    for t in &torque {
        assert!((t - 0.184424).abs() < 1e-5, "{torque:?}");
    }
    // Off centre the forces differ, but still carry the weight and no moment: the sum of f_i u_i is (0, 0, m g).
    let pose = [0.05, -0.03, 0.5, 0.04, -0.06, 0.1];
    let f = mechanism(&cfg).static_leg_forces(&pose).unwrap();
    let legs = mechanism(&cfg).geo.legs(&pose);
    let lift: f64 = (0..6).map(|i| f[i] * legs.u[i][2]).sum();
    assert!((lift - 62.2 * 9.80665).abs() < 1e-9 && f.iter().any(|v| (v - f[0]).abs() > 1.0));
}

/// A pose that takes a leg beyond its stroke is refused, one that tilts the legs beyond the joints' 40 degrees too.
#[test]
fn poses_beyond_the_strokes_and_the_joints_are_refused() {
    let machine = (ENTRY.build)(&ENTRY.config()).unwrap();
    assert_eq!(machine.unreachable(&[0.0, 0.0, 0.48, 0.0, 0.0, 0.0], 0.0), None);
    // 0.3 m up: L = sqrt(0.78^2 + 0.0689) = 0.8231 m, beyond 0.45 + 0.2.
    assert_eq!(machine.unreachable(&[0.0, 0.0, 0.78, 0.0, 0.0, 0.0], 0.0), Some("error-joint-beyond-stop"));
    // Low and far out: the legs lie flat.
    assert_eq!(machine.unreachable(&[0.25, 0.0, 0.25, 0.0, 0.0, 0.0], 0.0), Some("error-hexapod-joint-angle"));
}

/// The default cycle passes every verdict: the default is a sized example.
#[test]
fn the_default_cycle_passes_its_verdicts() {
    let cfg = ENTRY.config();
    let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    assert!(!load.tripped);
    for v in sim_components::analysis::verdicts(&cfg, &load).unwrap() {
        assert!(v.ok, "{} of motor {:?} is {} against {}", v.check, v.motor, v.value, v.limit);
    }
}
