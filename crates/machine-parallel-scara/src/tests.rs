//! The parallel SCARA against hand calculations: kinematics, the stroke's and the turn's ratios, an inertia torque in
//! the plane, the holding torques (none in the plane, the carriage's weight on the Z motor), the scene, the verdicts.

use super::*;
use sim_components::Sim;

fn scara(edits: &[&str]) -> ParallelScara {
    let mut cfg = ENTRY.config();
    for e in edits {
        cfg.set_text(e).unwrap();
    }
    ParallelScara::new(&cfg).unwrap()
}

fn close(a: f64, b: f64, tolerance: f64, what: &str) {
    assert!((a - b).abs() <= tolerance * b.abs().max(1e-12), "{what}: {a} against {b} by hand");
}

#[test]
fn the_parallel_scara_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ENTRY);
}

/// Forward of inverse is the plate's position to 1e-12 m over a grid of the working range in steps of 10 mm, and the
/// Jacobian is the derivative of the forward kinematics, by central differences of 1e-6 rad, to 1e-7 m/rad.
#[test]
fn the_five_bar_round_trips_and_its_jacobian_is_the_derivative_of_the_forward_kinematics() {
    let d = scara(&[]);
    let mut tried = 0;
    for i in 0..=80 {
        for k in 0..=48 {
            let p = [-0.4 + 0.01 * i as f64, -0.877 + 0.01 * k as f64];
            if d.unreachable(&[p[0], p[1], 0.1, 0.0], 0.0).is_some() {
                continue;
            }
            tried += 1;
            let t = d.legs.inverse(p).unwrap();
            let back = d.legs.forward(t).unwrap();
            assert!((0..2).all(|a| (back[a] - p[a]).abs() < 1e-12), "{p:?} back as {back:?}");
            let jac = d.legs.jacobian(t, p).unwrap();
            for c in 0..2 {
                let (mut up, mut down) = (t, t);
                up[c] += 1e-6;
                down[c] -= 1e-6;
                let (pu, pd) = (d.legs.forward(up).unwrap(), d.legs.forward(down).unwrap());
                for r in 0..2 {
                    assert!((jac[r][c] - (pu[r] - pd[r]) / 2e-6).abs() < 1e-7, "{p:?}: J[{r}][{c}]");
                }
            }
        }
    }
    assert!(tried > 2500, "{tried} poses");
}

/// The tool pose is (x, y, z, c): forward of inverse is the pose to 1e-12 at poses across the range, the stroke at both
/// ends and the tool turned both ways. The extra motors turn by their ratios, by hand: the Z motor 2 pi / 20 mm =
/// 314.159 rad per m of stroke behind its coupling (ratio 1), the C motor 8 rad per rad.
#[test]
fn the_kinematics_round_trip_and_the_stroke_and_turn_go_by_their_ratios() {
    let d = scara(&[]);
    assert_eq!(d.axes().iter().map(|a| a.id).collect::<Vec<_>>(), ["x", "y", "z", "c"]);
    assert_eq!(d.motors().iter().map(|m| m.id).collect::<Vec<_>>(), ["1", "2", "3", "4"]);
    for pose in [[0.1525, -0.5, 0.05, 0.0], [-0.3, -0.6, 0.0, -2.0], [0.0, -0.85, 0.2, 6.0], [0.4, -0.45, 0.1, 1.0]] {
        assert_eq!(d.unreachable(&pose, 0.0), None, "{pose:?}");
        let (mut motors, mut back) = ([0.0; 4], [0.0; 4]);
        d.inverse(&pose, &mut motors);
        d.forward(&motors, &mut back);
        assert!((0..4).all(|a| (back[a] - pose[a]).abs() < 1e-12), "{pose:?} back as {back:?}");
        let t = d.legs.inverse([pose[0], pose[1]]).unwrap();
        close(motors[0], 20.0 * t[0], 1e-12, "motor 1");
        close(motors[2], pose[2] * std::f64::consts::TAU / 0.02, 1e-12, "Z motor");
        close(motors[3], 8.0 * pose[3], 1e-12, "C motor");
    }
    // A target needs all four values.
    assert_eq!(d.unreachable(&[0.0, -0.5], 0.0), Some("error-target-out-of-range"));
}

/// The Z stroke and the tool turn are switched by the config: without them the tool pose is (x, y), with the stroke
/// alone (x, y, z) and motors 1, 2, 3; and the plate carries less, by the unit and carriage (2.95 + 1.0 kg) or the C
/// unit (2.43 kg) left out.
#[test]
fn the_stroke_and_the_turn_are_switched_by_the_config() {
    let full = scara(&[]).m_nt();
    let plane = scara(&["z_axis.enabled = false", "tool_turn.enabled = false"]);
    assert_eq!(plane.axes().iter().map(|a| a.id).collect::<Vec<_>>(), ["x", "y"]);
    close(plane.m_nt(), full - 2.95 - 1.0 - 2.43, 1e-12, "m_nt without both");
    let stroke = scara(&["tool_turn.enabled = false"]);
    assert_eq!(stroke.axes().iter().map(|a| a.id).collect::<Vec<_>>(), ["x", "y", "z"]);
    assert_eq!(stroke.motors().iter().map(|m| m.id).collect::<Vec<_>>(), ["1", "2", "3"]);
    close(stroke.m_nt(), full - 2.43, 1e-12, "m_nt without the turn");
    let turn = scara(&["z_axis.enabled = false"]);
    assert_eq!(turn.motors().iter().map(|m| m.id).collect::<Vec<_>>(), ["1", "2", "4"]);
}

/// The arm angle at the centre, from the leg's own equation solved by halving: (e + L_a cos t)^2 + (y + L_a sin t)^2 =
/// L_b^2.
fn centre_by_hand(d: &ParallelScara, y: f64) -> f64 {
    let (e, la, lb) = (d.legs.e, d.legs.la, d.legs.lb);
    let f = |t: f64| (e + la * t.cos()).powi(2) + (y + la * t.sin()).powi(2) - lb * lb;
    let (mut lo, mut hi) = (-60f64.to_radians(), 90f64.to_radians());
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        if f(lo) * f(mid) <= 0.0 { hi = mid } else { lo = mid }
    }
    0.5 * (lo + hi)
}

/// An acceleration of 10 m/s^2 along x at the centre, 500 mm in front of the shoulders (y = -0.5), at rest. By hand:
/// both arms at t = 1.42898 degrees (`centre_by_hand`); A = e + L_a cos t = 0.499876 m, B = y + L_a sin t = -0.490025 m,
/// so the forearms u_1 = (-A, B), u_2 = (A, B) and the elbows' motion c_1' = (-L_a sin t, -L_a cos t), c_2' =
/// (L_a sin t, -L_a cos t), D = u_i . c_i' = A L_a sin t - B L_a cos t = 0.200935 m^2 for both. The arms turn as
/// u_i . p'' = D t_i'': t_1'' = -A a / D = -24.8774 rad/s^2, t_2'' = +24.8774. The plate's force m a along x shares out
/// along the forearms, l_1 u_1 + l_2 u_2 = (m a, 0): l_2 = -l_1 = m a / (2 A), at the arms D l_i (the Jacobian's
/// transpose). m_nt = 1.0 + 2 (1/3) 0.6 + 2.95 + 1.0 + 2.43 + 5 = 12.780 kg (plate, forearms' share, Z unit, carriage,
/// C unit, payload), I_b = 0.064 + 0.4^2 (0.4 + 0.6667 * 0.6) = 0.192003 kg m^2: arm 1 takes I_b t_1'' - D m a / (2 A) =
/// -4.7765 - 25.6859 = -30.462 N m, at its motor over 20 plus the rotor's (1.7 + 0.406) 1e-4 * 20 t_1'': -1.6279 N m;
/// arm 2 the opposite. No gravity in the plane. The Z motor holds its 0.18729 N m (see below), the C motor nothing. The
/// rigid model given that torque accelerates the plate at 10 m/s^2 along x and not along y.
#[test]
fn an_acceleration_along_x_at_the_centre_takes_the_hand_calculated_torque() {
    let d = scara(&[
        "effects.joint_friction = false",
        "effects.motor_friction = false",
        "effects.torque_ripple = false",
        "effects.gearbox_efficiency = false",
        "effects.joint_elasticity = false",
    ]);
    let y = -0.5;
    let t = centre_by_hand(&d, y);
    let (e, la, a) = (0.1, 0.4, 10.0);
    let (big_a, big_b) = (e + la * t.cos(), y + la * t.sin());
    let dd = big_a * la * t.sin() - big_b * la * t.cos();
    let m = 1.0 + 2.0 * (1.0 - 0.6667) * 0.6 + 2.95 + 1.0 + 2.43 + 5.0;
    close(d.m_nt(), m, 1e-12, "m_nt");
    close(d.arm_inertia(), 0.064 + 0.16 * (0.4 + 0.6667 * 0.6), 1e-12, "I_b");
    let acc = -big_a * a / dd;
    let at_arm = d.arm_inertia() * acc - dd * m * a / (2.0 * big_a);
    let at_motor = at_arm / 20.0 + (1.7e-4 + 4.06e-5) * 20.0 * acc;
    println!(
        "A = {big_a:.6}, B = {big_b:.6}, D = {dd:.6}, t1'' = {acc:.4}, {at_arm:.4} N m at arm 1, {at_motor:.5} N m"
    );
    assert!((t.to_degrees() - 1.42898).abs() < 1e-4 && (dd - 0.200935).abs() < 1e-6, "{t} {dd}");
    assert!((at_arm + 30.462).abs() < 1e-3 && (at_motor + 1.6279).abs() < 1e-4, "{at_arm} {at_motor}");
    let pose = [0.0, y, 0.1, 0.0];
    let mut torque = [0.0; 4];
    d.feedforward_torque(&pose, &[a, 0.0, 0.0, 0.0], &mut torque);
    close(torque[0], at_motor, 1e-9, "motor 1");
    close(torque[1], -at_motor, 1e-9, "motor 2");
    close(torque[2], 6.0 * G * 0.02 / std::f64::consts::TAU, 1e-9, "Z motor");
    assert_eq!(torque[3], 0.0);
    let (mut state, mut hold) = (Vec::new(), [0.0; 4]);
    d.equilibrium(&pose, &mut state, &mut hold);
    let mut rates = vec![0.0; d.state_len()];
    d.derivatives(&state, &torque, &mut rates);
    let (_, j) = d.pose([state[0], state[1]]);
    let plate = mul(&j, [rates[2], rates[3]]);
    assert!((plate[0] - a).abs() < 1e-9 && plate[1].abs() < 1e-9, "{plate:?}");
    assert!(rates[N_RIGID + 1].abs() < 1e-12 && rates[N_RIGID + 4].abs() < 1e-12, "{rates:?}");
}

/// Gravity acts along -z, out of the plane: the arms' motors hold nothing anywhere in the range (their bearings carry
/// the weight), the tool's turn holds nothing, and the Z motor holds the carriage and the payload, by hand
/// m g lead / (2 pi) / ratio = (1 + 5) kg * 9.80665 m/s^2 * 0.02 m / (2 pi) / 1 = 0.187293 N m, 0.031216 N m without the
/// payload. Held so, nothing moves, rigid or elastic, with friction on. The torque breakdown says the same: the Z
/// motor's gravity part is that torque, the arms' none, and the nut carries 6 g = 58.84 N.
#[test]
fn gravity_loads_no_arm_motor_and_the_z_motor_holds_the_carriage() {
    let by_hand = |payload: f64| (1.0 + payload) * G * 0.02 / std::f64::consts::TAU;
    assert!((by_hand(5.0) - 0.187293).abs() < 1e-6 && (by_hand(0.0) - 0.031216).abs() < 1e-6);
    for elastic in [false, true] {
        let d = scara(&["effects.torque_ripple = false", &format!("effects.joint_elasticity = {elastic}")]);
        for pose in [[0.0, -0.5, 0.1, 0.0], [0.4, -0.5, 0.05, 1.0], [-0.3, -0.75, 0.15, -3.0]] {
            let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
            d.equilibrium(&pose, &mut state, &mut torque);
            assert_eq!(&torque[..2], &[0.0, 0.0], "{pose:?}");
            assert_eq!(torque[3], 0.0);
            close(torque[2], by_hand(5.0), 1e-12, "Z motor");
            let mut rates = vec![0.0; d.state_len()];
            d.derivatives(&state, &torque, &mut rates);
            assert!(rates.iter().all(|r| r.abs() < 1e-9), "elastic {elastic} at {pose:?}: {rates:?}");
            let mut signals = Vec::new();
            d.signals(&state, &torque, &mut signals);
            let infos = d.signal_infos();
            let at = |id: &str| signals[infos.iter().position(|s| s.id == id).unwrap()];
            assert_eq!([at("motor.1.torque_gravity"), at("motor.2.torque_gravity")], [0.0, 0.0]);
            close(at("motor.3.torque_gravity"), by_hand(5.0), 1e-12, "Z gravity part");
            close(at("joint.z.force"), 6.0 * G, 1e-12, "the nut's force");
        }
    }
    let mut d = scara(&["effects.torque_ripple = false"]);
    d.set_payload_attached(false);
    let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
    d.equilibrium(&[0.0, -0.5, 0.1, 0.0], &mut state, &mut torque);
    close(torque[2], by_hand(0.0), 1e-12, "Z motor without the payload");
}

/// The scene shows the plate at (x, y, 0) and the quill down to the tool's flange, the stroke's top `TOOL_DROP` = 0.1 m
/// below the plate: at z = 0.075 of the 0.2 m stroke the flange is at 0.075 - 0.2 - 0.1 = -0.225 m, the flange's box
/// centred 10 mm below it, the payload 70 mm below it, the quill 0.225 m long centred at -0.1125. Those four are the
/// collision bodies; the scene has four motors, three gearboxes (the arms' and the turn's; the Z motor has a coupling)
/// and four speed labels.
#[test]
fn the_scene_and_the_collision_bodies_hang_the_tool_under_the_plate() {
    let d = scara(&[]);
    let pose = [0.1525, -0.5, 0.075, 0.0];
    let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
    d.equilibrium(&pose, &mut state, &mut torque);
    let bodies = d.collision_bodies(&state);
    let by_hand = [[0.1525, -0.5, 0.0], [0.1525, -0.5, -0.1125], [0.1525, -0.5, -0.235], [0.1525, -0.5, -0.295]];
    assert_eq!(bodies.len(), 4);
    for (b, h) in bodies.iter().zip(by_hand) {
        assert!((0..3).all(|i| (b.centre[i] - h[i]).abs() < 1e-9), "{bodies:?}");
    }
    assert!((bodies[1].size[2] - 0.225).abs() < 1e-12);
    let point = d.scene_point(&pose).unwrap();
    assert!((point[2] + 0.245).abs() < 1e-12, "{point:?}");
    let mut scene = Scene::default();
    d.scene(&state, &mut scene);
    let count = |id: &str| scene.parts.iter().filter(|p| p.id == id).count();
    assert_eq!((count(sim_core::scene::MOTOR_PART), count("gearbox"), count("forearm")), (4, 3, 4));
    assert_eq!(scene.labels.iter().filter(|l| l.unit_key == "unit-rpm").count(), 4);
    // The forearm rods lie in the planes z = +-0.05, apart along the joints' axis.
    assert!(scene.parts.iter().filter(|p| p.id == "forearm").all(|p| (p.pos[2].abs() - 0.05).abs() < 1e-12));
    let gearbox_4 = d.thermal_parts().into_iter().find(|p| p.id == "gearbox_4").unwrap();
    assert_eq!((gearbox_4.scene_index, gearbox_4.owner_key), (2, "motor-tool"));
}

/// Targets outside the working range are refused with the reason: beyond the band's half width, beyond the far arc
/// (its centre 877 mm in front of the shoulders), nearer than the near edge (400 mm).
#[test]
fn targets_outside_the_working_range_are_refused() {
    let d = scara(&[]);
    assert_eq!(d.unreachable(&[0.0, -0.87699, 0.1, 0.0], 0.0), None);
    for p in [[0.40001, -0.5], [0.0, -0.87701], [0.0, -0.39999]] {
        assert_eq!(d.unreachable(&[p[0], p[1], 0.1, 0.0], 0.0), Some("error-delta-outside-envelope"), "{p:?}");
    }
}

/// The default cycle, the Adept cycle laid flat with the stroke and the turn, passes every verdict, the ball screw's
/// among them (`sim-cli verdicts parallel-scara`: settling 0.19 mm of 0.5, the arms' gearboxes at most 145 N m of
/// 192, the screw at 1535 r/min of the 5160 allowed).
#[test]
fn the_default_cycle_passes_its_verdicts() {
    let cfg = ENTRY.config();
    let load = sim_components::analysis::analyse_cycle(&mut Sim::new(&ENTRY, &cfg).unwrap()).unwrap();
    let verdicts = sim_components::analysis::verdicts(&cfg, &load).unwrap();
    let failed: Vec<_> = verdicts.iter().filter(|v| !v.ok).collect();
    assert!(failed.is_empty(), "{failed:?}");
    assert!(verdicts.iter().any(|v| v.check == "screw-speed"));
}
