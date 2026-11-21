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

#[track_caller]
fn close(a: f64, b: f64, tolerance: f64, what: &str) {
    sim_core::testkit::record_check(what, b, a, tolerance * b.abs().max(1e-12));
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
/// -4.7765 - 25.6859 = -30.462 N m, at its motor over 20 plus the rotor's and the gearbox's (the SP100's 0.70 kg cm^2)
/// (1.7 + 0.70) 1e-4 * 20 t_1'': -1.6425 N m; arm 2 the opposite. No gravity in the plane. The Z motor holds its
/// 0.18729 N m (see below), the C motor nothing. The rigid model given that torque accelerates the plate at 10 m/s^2
/// along x and not along y.
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
    let at_motor = at_arm / 20.0 + (1.7e-4 + 7.0e-5) * 20.0 * acc;
    println!(
        "A = {big_a:.6}, B = {big_b:.6}, D = {dd:.6}, t1'' = {acc:.4}, {at_arm:.4} N m at arm 1, {at_motor:.5} N m"
    );
    assert!((t.to_degrees() - 1.42898).abs() < 1e-4 && (dd - 0.200935).abs() < 1e-6, "{t} {dd}");
    assert!((at_arm + 30.462).abs() < 1e-3 && (at_motor + 1.6425).abs() < 1e-4, "{at_arm} {at_motor}");
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

/// Every rate of an equilibrium state is nothing: below 1e-9, but the elastic stroke's and turn's rotor accelerations,
/// judged as the torque J_rotor w' they stand for, below 1e-11 N m. The rotor's angle, i (q + twist) up to 47 rad, holds
/// the twist to its rounding, 7e-15 rad, and the spring at the rotor (the stroke's 4.75e7 N/m / i^2 = 481 N m/rad) turns
/// that into 3.4e-12 N m: 2.4e-8 rad/s^2 on the Z rotor's 1.43e-5 kg m2 (seen: 1.4e-8 and 2.4e-8).
fn assert_at_rest(d: &ParallelScara, state: &[f64], torque: &[f64], what: &str) {
    let mut rates = vec![0.0; d.state_len()];
    d.derivatives(state, torque, &mut rates);
    let rotor_acc = |i: usize| d.elastic && i >= d.planar_len() && (i - d.planar_len()) % 5 == 4;
    for (i, r) in rates.iter().enumerate() {
        let (size, bound) = match rotor_acc(i) {
            true => ((r * d.extras[(i - d.planar_len()) / 5].joint.rotor_inertia).abs(), 1e-11),
            false => (r.abs(), 1e-9),
        };
        assert!(size < bound, "{what}: rate {i} is {r}: {rates:?}");
    }
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
        // The far end's screw stiffness: with it following the nut the rotor holds F^2 / (2 E A) less.
        let d = scara(&[
            "effects.torque_ripple = false",
            "effects.screw_stiffness_with_position = false",
            &format!("effects.joint_elasticity = {elastic}"),
        ]);
        for pose in [[0.0, -0.5, 0.1, 0.0], [0.4, -0.5, 0.05, 1.0], [-0.3, -0.75, 0.15, -3.0]] {
            let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
            d.equilibrium(&pose, &mut state, &mut torque);
            assert_eq!(&torque[..2], &[0.0, 0.0], "{pose:?}");
            assert_eq!(torque[3], 0.0);
            close(torque[2], by_hand(5.0), 1e-12, "Z motor");
            assert_at_rest(&d, &state, &torque, &format!("elastic {elastic} at {pose:?}"));
            let mut signals = Vec::new();
            d.signals(&state, &torque, &mut signals);
            let infos = d.signal_infos();
            let at = |id: &str| signals[infos.iter().position(|s| s.id == id).unwrap()];
            assert_eq!([at("motor.1.torque_gravity"), at("motor.2.torque_gravity")], [0.0, 0.0]);
            close(at("motor.3.torque_gravity"), by_hand(5.0), 1e-12, "Z gravity part");
            // Elastic, the spring's force K (rotor / i - q): the rotor angle's rounding (3.6e-15 rad at 31 rad) over i =
            // 314 rad/m times K = 4.75e7 N/m is 5.4e-10 N, 9e-12 of the force (seen: 2.2e-12).
            close(at("joint.z.force"), 6.0 * G, if elastic { 1e-11 } else { 1e-12 }, "the nut's force");
        }
    }
    let mut d = scara(&["effects.torque_ripple = false", "effects.screw_stiffness_with_position = false"]);
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
/// among them (`sim-cli verdicts parallel-scara`: settling 0.30 mm of 0.5, the arms' gearboxes at most 137 N m of
/// 278, the screw at 1536 r/min of the 5160 allowed, the tool turn's motor at 0.39 of its peak torque).
#[test]
fn the_default_cycle_passes_its_verdicts() {
    let cfg = ENTRY.config();
    let load = sim_components::analysis::analyse_cycle(&mut Sim::new(&ENTRY, &cfg).unwrap()).unwrap();
    let verdicts = sim_components::analysis::verdicts(&cfg, &load).unwrap();
    let failed: Vec<_> = verdicts.iter().filter(|v| !v.ok).collect();
    assert!(failed.is_empty(), "{failed:?}");
    assert!(verdicts.iter().any(|v| v.check == "screw-speed"));
}

/// The tool turn's velocity loop at 0.08 N m s/rad hammers from flank to flank of its gearbox's 1 arcmin play, which
/// the small-signal measurement inside the play does not see: there it reads 78.2 degrees and 10.54 dB, but after the
/// swept kick at rest (`backlash_hammering`) the loop keeps swinging at 650 Hz with 0.77 N m peak of the 1.95 N m the
/// motor has (`sim-cli margins`, run 21 November 2026), so `velocity_loop_margins` calls it not stable at the start
/// pose. At 0.1 it hammers at 705 to 710 Hz, the 710 Hz the config's note found in a traverse, hardest (2.63 N m) over
/// the pick, the cycle's third pose (`margin_poses`: [-0.1525, -0.5, 0.095, pi / 2], the tool turned by 90 degrees);
/// at the shipped 0.033, and at 0.06, it does not at any of the cycle's four poses.
#[test]
fn the_tool_turn_hammering_across_its_play_is_found_by_the_margins() {
    use sim_components::analysis::{backlash_hammering, velocity_loop_margins};
    let at = |kv: f64| {
        let mut cfg = ENTRY.config();
        cfg.set_text(&format!("drive.4.velocity_kp_nm_s_rad = {kv}")).unwrap();
        cfg
    };
    let m = velocity_loop_margins(&ENTRY, &at(0.08), &[]).unwrap()[0].1[3];
    let h = m.hammering.expect("0.08 hammers");
    assert!(m.valid && m.gain_margin_db > 0.0 && !m.stable(), "{m:?}");
    // The start pose's run (the first).
    assert!((h.frequency_hz - 650.0).abs() < 15.0 && h.torque_nm > 0.3 && h.pose == 0, "{h:?}");
    let h = backlash_hammering(&ENTRY, &at(0.1), 3).unwrap().expect("0.1 hammers");
    assert!((h.frequency_hz - 710.0).abs() < 15.0 && h.pose == 2, "{h:?}");
    for kv in [0.033, 0.06] {
        assert_eq!(backlash_hammering(&ENTRY, &at(kv), 3).unwrap(), None, "{kv}");
    }
}

/// Friction feedforward from the machine's own model, by hand, on the tool turn (motor 4) in its cruise at 30 rad/s
/// (the turn 0 -> 6 rad, the plate at rest). The quill's friction 0.02 tanh(30 / 0.01) + 1e-4 x 30 = 0.023 N m passes
/// the gearbox 8, whose teeth (its 97 % at 259 N m with the drag 0.93 N m at the input taken out) pass 1 / (1 / 0.97 -
/// 8 x 0.93 / 259) = 0.997803, at part load 0.997803^tanh(0.023 x 30 / 0.5) = 0.997803^0.880951 = 0.998064:
/// 0.023 / (8 x 0.998064) = 0.002881 N m. The motor at 240 rad/s: its seals 0.05 (the breakaway's Stribeck part
/// exp(-(240 / 2)^2) is nothing) + 2e-4 x 240 = 0.098 N m, and the gearbox's drag 0.93 (240 / 314.159)^(2/3) = 0.777184
/// N m. So 0.878065 N m. With it the velocity loop's integrator carries nothing in the cruise (under 1 % of it; 0.04 %
/// seen); without it, it carries the whole friction.
#[test]
fn the_machine_friction_feedforward_is_the_steady_friction_and_the_integrator_carries_nothing() {
    let cruise = |model: bool| {
        let mut cfg = ENTRY.config();
        cfg.set_text("effects.friction_feedforward = true").unwrap();
        let source = if model { "machine" } else { "characteristic" };
        cfg.set_text(&format!("drive.friction_feedforward_model = \"{source}\"")).unwrap();
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        sim.command(sim_components::Command::MoveTo(vec![0.1525, -0.5, 0.075, 6.0])).unwrap();
        let (mut ff, mut integrator, mut n) = (0.0, 0.0, 0);
        sim.run(0.3, |s| {
            if s.control.set_acc[3] == 0.0 && (s.control.set_vel[3] - 30.0).abs() < 1e-12 {
                (ff, integrator, n) =
                    (s.control.drives[3].friction_ff, integrator + s.control.drives[3].controller_torque, n + 1);
            }
        });
        assert!(n > 1000, "{n} samples in the cruise");
        (ff, integrator / n as f64)
    };
    let quill: f64 = 0.02 + 1e-4 * 30.0;
    let tooth: f64 = 1.0 / (1.0 / 0.97 - 8.0 * 0.93 / 259.0);
    let loss = tooth.powf((quill * 30.0 / 0.5).tanh());
    let expected = 0.05 + 2e-4 * 240.0 + 0.93 * (240.0f64 / 314.159265).powf(2.0 / 3.0) + quill / (8.0 * loss);
    assert!((expected - 0.878065).abs() < 1e-6, "by hand {expected}");
    let (ff, integrator) = cruise(true);
    assert!((ff - expected).abs() < 1e-5 * expected, "feedforward {ff} N m, by hand {expected}");
    assert!(integrator.abs() < 0.01 * expected, "the integrator carries {integrator} N m");
    let (ff, integrator) = cruise(false);
    assert!(ff == 0.0 && (integrator - expected).abs() < 0.02 * expected, "without: {integrator} N m");
}

/// The arm's first moment about its joint (`arm_moment`): the tube's m c, `mass.upper_arm_kg` and `upper_arm_cg_m`, and
/// the elbow's l_a (m_elbow + share m_pair). Shipped, the uniform tube of the configured inertia, 1.2 kg x 0.2 m = 3 I /
/// (2 l_a) = 0.24 kg m, plus 0.4 (0.4 + 0.6667 x 0.6) = 0.320016 kg m; 2 kg at 0.3 m gives 0.6 for the tube's part.
#[test]
fn the_arm_tubes_first_moment_takes_its_mass_and_centre_where_given() {
    let elbow = 0.4 * (0.4 + 0.6667 * 0.6);
    close(scara(&[]).arm_moment, 1.5 * 0.064 / 0.4 + elbow, 1e-12, "the uniform tube");
    close(scara(&["mass.upper_arm_kg = 2.0", "mass.upper_arm_cg_m = 0.3"]).arm_moment, 0.6 + elbow, 1e-12, "m c");
}

/// The forearms are rods: in the frame momentum their mass is at their middles, whatever share of their inertia the
/// lumped model puts at the elbow, so that the plane's momentum is the rate of where their mass is. 0.1 kg more on each
/// forearm pair moves the first moment by 0.1 kg times the sum of the rods' middles, (elbow + plate joint) / 2 with
/// the elbow at (+-(R + L_a cos t), -L_a sin t) and the plate joint at (x +- r, y): at x = 0.1 m, y = -0.6 m,
/// 0.1 ((elbow_1 + elbow_2) / 2 + (x, y)); the lumped split had 0.1 (2/3 (elbow_1 + elbow_2) + 2/3 (x, y)) there.
#[test]
fn the_forearms_first_moment_is_at_their_middles() {
    let (x, y) = (0.1, -0.6);
    for share in ["mass.forearm_elbow_share = 0.6667", "mass.forearm_elbow_share = 0.5"] {
        let moment = |m: &str| {
            let s = scara(&[share, m]);
            let (mut state, mut torque) = (Vec::new(), vec![0.0; s.motors().len()]);
            s.equilibrium(&[x, y, 0.0, 0.0][..s.axes().len()], &mut state, &mut torque);
            s.frame_momentum(&state).unwrap().first_moment
        };
        let (a, b) = (moment("mass.forearm_pair_kg = 0.6"), moment("mass.forearm_pair_kg = 0.7"));
        let s = scara(&[]);
        let t = s.legs.inverse([x, y]).unwrap();
        let (base, la) = (s.base_radius, s.legs.la);
        let elbows = [0, 1].map(|k| [SIDE[k] * (base + la * t[k].cos()), -la * t[k].sin()]);
        for i in 0..2 {
            let by_hand = 0.1 * (0.5 * (elbows[0][i] + elbows[1][i]) + [x, y][i]);
            close(b[i] - a[i], by_hand, 1e-9, "the first moment per 0.1 kg of forearm");
        }
        assert!((b[2] - a[2]).abs() < 1e-12);
    }
}

/// The loss-free scara with `edits`.
fn lossless(edits: &[&str]) -> ParallelScara {
    let mut cfg = lossless_config();
    for e in edits {
        cfg.set_text(e).unwrap();
    }
    ParallelScara::new(&cfg).unwrap()
}

/// The stroke's gearbox stiffness at the joint by hand, N/m: the screw (16 x 20, root 13.5 mm, 0.3 m, fixed-free so
/// softest at the far end) has the shaft's E A / x = 2.06e11 pi / 4 0.0135^2 / 0.3 = 9.82887e7 N/m, the nut's 2e8 and the
/// bearings' 4e8 in series: 1 / (1.017411e-8 + 5e-9 + 2.5e-9) = 5.65797e7 N/m; the coupling's 3000 N m/rad over the
/// lever l = 0.02 / 2 pi = 3.18310e-3 m per rad, 3000 / l^2 = 2.96088e8 N/m, in series with it: 1 / (1.767410e-8 +
/// 3.377373e-9) = 4.75026e7 N/m.
fn stroke_stiffness_by_hand() -> f64 {
    let shaft = 2.06e11 * std::f64::consts::FRAC_PI_4 * 0.0135f64.powi(2) / 0.3;
    let screw = 1.0 / (1.0 / shaft + 1.0 / 2e8 + 1.0 / 4e8);
    let lever = 0.02 / std::f64::consts::TAU;
    1.0 / (1.0 / screw + lever * lever / 3000.0)
}

/// The rest sag's allowance (`rest_sag`) holds the whole stroke with the screw's stiffness following the nut: the
/// stroke's half play and the 6 kg's weight over the screw where it is softest on the stroke 0..0.2 m, plus a tenth
/// (the stroke's term the largest here). With the shaft fixed at one end (the shipped fixed bearing at 0.25 m) that is
/// the stroke's bottom, the nut a = 0.25 m from it, not the shaft's far end a = 0.3 m beyond the stroke
/// (`stroke_stiffness_by_hand`); held at both ends the shaft's middle a = 0.15 m (z = 0.1), a (L - a) / (E A L). The
/// coupling's play of 1 rad [test value] is 0.5 x 0.02 / 2 pi = 1.591549e-3 m at the carriage. The flanks' band off, so
/// the spring is linear; the shipped allowance is the arms' play's (unchanged).
#[test]
fn the_rest_sag_takes_the_screw_where_it_is_softest_on_the_stroke() {
    let ea = 2.06e11 * std::f64::consts::FRAC_PI_4 * 0.0135f64.powi(2);
    let lever = 0.02 / std::f64::consts::TAU;
    let series = |shaft: f64| 1.0 / (shaft + 1.0 / 2e8 + 1.0 / 4e8 + lever * lever / 3000.0);
    let (weight, half) = (6.0 * 9.80665, 0.5 * lever);
    for (mounting, k) in [("fixed-free", series(0.25 / ea)), ("fixed-fixed", series(0.15 * 0.15 / (0.3 * ea)))] {
        let d = scara(&[
            "effects.screw_stiffness_with_position = true",
            "effects.gearbox_lost_motion = false",
            "gearbox.3.backlash_rad = 1.0",
            &format!("screw.mounting = \"{mounting}\""),
        ]);
        close(d.rest_sag(), 1.1 * (half + weight / k), 1e-6, mounting);
    }
}

/// The carriage on its screw rings with the rotor held at sqrt(K / m) / 2 pi, K = 4.75026e7 N/m (`stroke_stiffness_by_hand`),
/// m = 1 kg carriage + 5 kg payload: sqrt(7.91710e6) / 2 pi = 447.8 Hz (not the 570 Hz the rigid model's comment guessed;
/// with the rotor free, its 1.43e-5 kg m2 as (2 pi / 0.02)^2 1.43e-5 = 1.41 kg at the nut, the pair rings at
/// sqrt(K (1 / 6 + 1 / 1.41)) / 2 pi = 1.03 kHz); the tool on the turn gearbox (8250.592 N m/rad, 2e-3 + 8.33e-3 kg m2):
/// sqrt(7.98703e5) / 2 pi = 142.2 Hz. Found two ways: from the model's own accelerations (the carriage moved 1 um from
/// where its rotor is: m a = -K dx), and the reported modes.
///
/// The plane's two at the centre (y = -0.5, `an_acceleration_along_x_at_the_centre_takes_the_hand_calculated_torque`
/// has A, B, D): J = D J_x^-1 = [[-D / 2A, D / 2A], [D / 2B, D / 2B]], so the arms turning against each other move the
/// plate along x, M_x = I_b + m_nt D^2 / (2 A^2) = 0.192003 + 12.78 x 0.080791 = 1.22451 kg m2, and together along y,
/// M_y = I_b + m_nt D^2 / (2 B^2) = 1.26643 kg m2; each on its gearbox's 106570.15 N m/rad: 46.95 Hz along x, 46.17 Hz
/// along y, damping ratios 5 / (2 sqrt(k M)). The lowest, y's, is `resonance_hz`.
#[test]
fn the_carriage_rings_on_its_screw_at_the_hand_calculated_frequency() {
    // One screw stiffness for the stroke, the far end's, as `stroke_stiffness_by_hand`.
    let d = lossless(&["effects.screw_stiffness_with_position = false"]);
    // Each extra axis six states: the stroke's coupling has a Maxwell arm (`Gearbox::arm`), and so all of them a place.
    assert_eq!(d.state_len(), N_ELASTIC + 12);
    assert_eq!(scara(&["effects.joint_elasticity = false"]).state_len(), N_RIGID + 6);
    let pose = [0.0, -0.5, 0.1, 0.0];
    let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
    d.equilibrium(&pose, &mut state, &mut torque);
    let (at, delta) = (d.planar_len(), 1e-6);
    let mut rates = vec![0.0; d.state_len()];
    d.derivatives(&state, &torque, &mut rates);
    let rest = rates[at + 1];
    assert!(rest.abs() < 1e-9, "at rest the carriage accelerates by {rest}");
    // The static stiffness: the coupling's damper (its Maxwell arm after the extra axes' five states, `Gearbox::arm`)
    // relaxed at the new twist, which the carriage's move takes down by delta; held, the arm would add its K_m.
    state[at] += delta;
    state[at + 5] -= delta;
    d.derivatives(&state, &torque, &mut rates);
    let hz = (-(rates[at + 1] - rest) / delta).sqrt() / std::f64::consts::TAU;
    let (k, m) = (stroke_stiffness_by_hand(), 6.0);
    let by_hand = (k / m).sqrt() / std::f64::consts::TAU;
    close(k, 4.75026e7, 1e-5, "the stroke's stiffness");
    close(by_hand, 447.8, 1e-3, "the carriage's mode by hand");
    close(hz, by_hand, 1e-6, "the carriage's mode from the model");
    let turn = (8250.592f64 / (2.0e-3 + 8.33e-3)).sqrt() / std::f64::consts::TAU;
    close(turn, 142.2, 1e-3, "the tool's mode by hand");
    // The plane's, by hand.
    let t = centre_by_hand(&d, -0.5);
    let (big_a, big_b) = (0.1 + 0.4 * t.cos(), -0.5 + 0.4 * t.sin());
    let dd = big_a * 0.4 * t.sin() - big_b * 0.4 * t.cos();
    let (i_b, m) = (0.064 + 0.16 * (0.4 + 0.6667 * 0.6), 1.0 + 2.0 * (1.0 - 0.6667) * 0.6 + 2.95 + 1.0 + 2.43 + 5.0);
    let [along_x, along_y] = [big_a, big_b].map(|l| i_b + m * dd * dd / (2.0 * l * l));
    close(along_x, 1.22451, 1e-5, "M_x");
    close(along_y, 1.26643, 1e-5, "M_y");
    let k_arm = 106570.15;
    let [x_hz, y_hz] = [along_x, along_y].map(|inertia| (k_arm / inertia).sqrt() / std::f64::consts::TAU);
    close(x_hz, 46.95, 1e-3, "along x by hand");
    close(y_hz, 46.17, 1e-3, "along y by hand");
    // The shipped damping (the loss-free config has none), the far end's stiffness.
    let d = scara(&["effects.screw_stiffness_with_position = false"]);
    let modes = d.modes(&pose);
    let found: Vec<f64> = modes.iter().map(|m| m.hz).collect();
    assert_eq!(found.len(), 4, "{found:?}");
    close(modes[0].hz, y_hz, 1e-9, "the plane's mode along y");
    close(modes[0].damping, 5.0 / (2.0 * (k_arm * along_y).sqrt()), 1e-9, "its damping");
    close(modes[1].hz, x_hz, 1e-9, "the plane's mode along x");
    close(modes[1].damping, 5.0 / (2.0 * (k_arm * along_x).sqrt()), 1e-9, "its damping");
    close(modes[2].hz, turn, 1e-9, "the turn's reported mode");
    close(modes[3].hz, by_hand, 1e-9, "the stroke's reported mode");
    assert_eq!(d.resonance_hz(&pose), Some(modes[0].hz));
    let rigid = scara(&["effects.joint_elasticity = false"]);
    assert!(rigid.modes(&pose).is_empty() && rigid.resonance_hz(&pose).is_none());
}

/// The shipped config, its screw's stiffness following the nut: the fixed bearing at the plate, z = 0.25, so the nut is
/// 0.25 - z from it, and the shaft's E A / a with E A = 2.06e11 pi / 4 0.0135^2 = 2.948660e7 N in place of the far
/// end's: 1 / K = 1 / 4.75026e7 - (0.3 - a) / E A. z = 0.1 (a = 0.15): K = 6.26393e7 N/m, sqrt(K / 6) / 2 pi = 514.24
/// Hz; z = 0.2 (a = 0.05): K = 7.95349e7 N/m, 579.46 Hz; z = 0 (a = 0.25): 467.02 Hz. The stroke's is the highest mode.
#[test]
fn the_carriage_rings_higher_with_the_nut_near_the_plate() {
    let d = scara(&[]);
    let ea = 2.06e11 * std::f64::consts::FRAC_PI_4 * 0.0135f64.powi(2);
    for z in [0.0, 0.1, 0.2] {
        let k = 1.0 / (1.0 / stroke_stiffness_by_hand() - (0.3 - (0.25 - z)) / ea);
        let by_hand = (k / 6.0).sqrt() / std::f64::consts::TAU;
        let found = d.modes(&[0.0, -0.5, z, 0.0]).iter().map(|m| m.hz).fold(0.0, f64::max);
        close(found, by_hand, 1e-9, "the stroke's mode along the stroke");
    }
    close((6.26393e7f64 / 6.0).sqrt() / std::f64::consts::TAU, 514.24, 1e-5, "z = 0.1 by hand");
}

/// At rest under the weight the screw carries 6 kg x g = 58.8399 N and is twisted by F / K = 58.8399 / 4.75020e7 =
/// 1.23869e-6 m, the rotor ahead of the carriage by i times that (i = 2 pi / 0.02 = 314.159 rad/m): 3.8914e-4 rad. The
/// turn holds nothing: its rotor is at 8 times the tool's angle. Without the payload the twist is 1 / 6 of it.
#[test]
fn the_screw_is_twisted_by_the_weight_at_rest() {
    // One screw stiffness for the stroke, the far end's, as `stroke_stiffness_by_hand`.
    let mut d = lossless(&["effects.screw_stiffness_with_position = false"]);
    let (at, i) = (d.planar_len(), std::f64::consts::TAU / 0.02);
    for (attached, kg) in [(true, 6.0), (false, 1.0)] {
        d.set_payload_attached(attached);
        let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
        d.equilibrium(&[0.0, -0.5, 0.1, 1.0], &mut state, &mut torque);
        let twist = kg * G / stroke_stiffness_by_hand();
        close(state[at + 3] - i * state[at], i * twist, 1e-6, "the screw's twist");
        close(state[at + 3], i * (0.1 + twist), 1e-9, "the stroke's rotor");
        close(state[at + 9], 8.0 * 1.0, 1e-12, "the turn's rotor");
        assert_eq!((state[at + 4], state[at + 10]), (0.0, 0.0));
        assert_at_rest(&d, &state, &torque, &format!("payload {attached}"));
    }
}

/// Energy is conserved with the drives holding their torque: the stored energy (kinetic, potential, springs) less the
/// work of the motors plus what the loss rows take stays where it started, with the carriage 2 um and the tool 1 mrad
/// off the positions of their rotors, over 8 ms (3.6 periods of the carriage, 1.1 of the tool), RK4 at 2 us. The
/// stored energy at the start is K dx^2 / 2 = 9.5e-5 J on the screw and 8250.592 x 1e-6 / 2 = 4.1e-3 J on the turn;
/// without and with the gearboxes' play (the turn's 1 arcmin, which it then crosses from flank to flank).
#[test]
fn the_screw_and_the_turn_conserve_energy_with_their_losses_booked() {
    for backlash in [false, true] {
        budget_of_the_screw_and_the_turn(backlash);
    }
}

fn budget_of_the_screw_and_the_turn(backlash: bool) {
    let d = lossless(&[&format!("effects.gearbox_backlash = {backlash}")]);
    let n = d.state_len();
    let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
    d.equilibrium(&[0.0, -0.5, 0.1, 0.0], &mut state, &mut torque);
    let at = d.planar_len();
    state[at] += 2e-6;
    state[at + 6] += 1e-3;
    let f = |s: &[f64]| {
        let mut r = vec![0.0; n];
        d.derivatives(s, &torque, &mut r);
        r
    };
    let advance = |s: &[f64], k: &[f64], h: f64| s.iter().zip(k).map(|(x, y)| x + h * y).collect::<Vec<f64>>();
    // What the motors put in less what the loss rows take, W.
    let power = |s: &[f64]| {
        let (mut rotor, mut speed) = ([0.0; 4], [0.0; 4]);
        d.motor_state(s, &mut rotor, &mut speed);
        let mut loss = Vec::new();
        d.loss_powers(s, &torque, &mut loss);
        (0..4).map(|k| torque[k] * speed[k]).sum::<f64>() - loss.iter().sum::<f64>()
    };
    // At 1 us: the coupling's damper bridges its own spring alone (`Gearbox::arm`), and its arm relaxes in 25 us, so the
    // trapezoid's work is off by 8.6e-9 J at 2 us, 2.1e-9 at 1 us, 5.4e-10 at 0.5 us without the play (second order in
    // the step, not a booking error).
    let (dt, e0) = (1e-6, d.energy(&state));
    let (mut work, mut last) = (0.0, power(&state));
    for _ in 0..8000 {
        let k1 = f(&state);
        let k2 = f(&advance(&state, &k1, dt / 2.0));
        let k3 = f(&advance(&state, &k2, dt / 2.0));
        let k4 = f(&advance(&state, &k3, dt));
        state = (0..n).map(|i| state[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])).collect();
        let now = power(&state);
        work += 0.5 * (last + now) * dt;
        last = now;
    }
    let drift = d.energy(&state) - e0 - work;
    // With the play the step crosses the flanks' kink, and the budget closes to first order in the step: 1.71e-6 J at
    // 1 us, 0.85e-6 at 0.5 us (a booking error would not shrink).
    let bound = if backlash { 1e-3 } else { 1e-6 } * 4.2e-3;
    assert!(drift.abs() < bound, "backlash {backlash}: the budget is off by {drift} J of 4.2e-3");
}
