//! The four-arm delta against hand calculations: kinematics at turned tools, the plate's ratio, the holding torque at
//! the centre, the feedforward, the energy of a free motion, the amplifier's mode.

use super::*;

fn h4(edits: &[&str]) -> H4 {
    let mut cfg = ENTRY.config();
    for e in edits {
        cfg.set_text(e).unwrap();
    }
    H4::new(&cfg).unwrap()
}

fn close(a: f64, b: f64, tolerance: f64, what: &str) {
    assert!((a - b).abs() <= tolerance * b.abs().max(1e-12), "{what}: {a} against {b} by hand");
}

/// A leg's arm angle by hand, elbow out: its forearm's end at (x', y', z) in the leg's own frame (the plate radius
/// folded into e), from (e + L_a cos t - x')^2 + y'^2 + (z + L_a sin t)^2 = L_b^2 solved by halving between -60 and 90
/// degrees, where the left side rises with t at these heights (default geometry: e = 0.18, L_a = 0.5, L_b = 1.1 m).
fn angle_by_hand(xl: f64, yl: f64, z: f64) -> f64 {
    let (e, la, lb) = (0.18, 0.5, 1.1);
    let f = |t: f64| (e + la * t.cos() - xl).powi(2) + yl * yl + (z + la * t.sin()).powi(2) - lb * lb;
    let (mut lo, mut hi) = (-60f64.to_radians(), 90f64.to_radians());
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if f(lo) * f(mid) <= 0.0 { hi = mid } else { lo = mid }
    }
    0.5 * (lo + hi)
}

/// Forward of inverse is the pose to 1e-12 (m, and rad of the turn) over a grid of the working range at five turns of
/// the tool, and the Jacobian in (x, y, z, h) is the derivative of the forward kinematics, by central differences of
/// 1e-6 rad, to 1e-7.
#[test]
fn the_kinematics_round_trip_at_any_turn_and_the_jacobian_is_the_derivative_of_the_forward_kinematics() {
    let d = h4(&[]);
    let mut tried = 0;
    for i in 0..=10 {
        for j in 0..=10 {
            for k in 0..=6 {
                for c in [-std::f64::consts::PI, -1.0, 0.0, 2.0, std::f64::consts::PI] {
                    let tool = [-0.65 + 0.13 * i as f64, -0.65 + 0.13 * j as f64, -1.2 + 0.45 * k as f64 / 6.0, c];
                    if d.unreachable(&tool, 0.0).is_some() {
                        continue;
                    }
                    tried += 1;
                    let (mut motors, mut back) = ([0.0; 4], [0.0; 4]);
                    d.inverse(&tool, &mut motors);
                    d.forward(&motors, &mut back);
                    for a in 0..4 {
                        assert!((back[a] - tool[a]).abs() < 1e-12, "{tool:?} back as {back:?}");
                    }
                    let x = d.coordinates(&tool);
                    let t = d.legs.inverse(x).unwrap();
                    let jac = d.legs.jacobian(t, x).unwrap();
                    let h = 1e-6;
                    for col in 0..4 {
                        let (mut up, mut down) = (t, t);
                        up[col] += h;
                        down[col] -= h;
                        let (xu, xd) = (d.legs.forward(up).unwrap(), d.legs.forward(down).unwrap());
                        for r in 0..4 {
                            assert!(
                                (jac[r][col] - (xu[r] - xd[r]) / (2.0 * h)).abs() < 1e-7,
                                "{tool:?}: J[{r}][{col}]"
                            );
                        }
                    }
                }
            }
        }
    }
    assert!(tried > 1000, "{tried} poses");
}

/// The plate's ratio. At the centre, 850 mm down, the parts slid by h = 10 mm each (20 mm against each other): leg 1's
/// forearm ends at p - h d = (-h / sqrt 2, h / sqrt 2, z), in its own frame 7.07 mm in and 7.07 mm aside; leg 3's at
/// p + h d, in its frame (turned by 180 degrees) the same; legs 2 and 4 end 7.07 mm out and 7.07 mm aside. By hand
/// (`angle_by_hand`): arms 1 and 3 at -1.0391 degrees, arms 2 and 4 at -2.3438 (at h = 0 all four at -1.6975): the
/// slide is arms 1 and 3 up and 2 and 4 down by the same 0.65 degrees. The pinion between the racks rolls by
/// h / r_p = 0.010 / 0.04 = 0.25 rad, the gear stage of 1.2 turns the tool by 0.3 rad. The machine's forward kinematics
/// of those hand angles (times the gearbox's 20) give the tool at the centre turned by 0.3 rad, and its inverse of that
/// pose the hand angles; the signal plate.slide is the 20 mm.
#[test]
fn the_tool_turns_by_the_gear_ratio_over_the_pinion_radius_times_the_half_slide() {
    let d = h4(&[]);
    let (z, h) = (-0.85, 0.010);
    let q = h / 2f64.sqrt();
    let (t13, t24, t0) = (angle_by_hand(-q, q, z), angle_by_hand(q, q, z), angle_by_hand(0.0, 0.0, z));
    close(t13.to_degrees(), -1.0391, 1e-4, "arms 1 and 3");
    close(t24.to_degrees(), -2.3438, 1e-4, "arms 2 and 4");
    close(t0.to_degrees(), -1.6975, 1e-4, "at h = 0");
    let c = 1.2 * h / 0.04;
    close(c, 0.3, 1e-12, "turn");
    close(d.kappa(), 30.0, 1e-12, "kappa");
    let mut tool = [0.0; 4];
    d.forward(&[20.0 * t13, 20.0 * t24, 20.0 * t13, 20.0 * t24], &mut tool);
    for (k, by_hand) in [0.0, 0.0, z, c].into_iter().enumerate() {
        assert!((tool[k] - by_hand).abs() < 1e-9, "{tool:?}");
    }
    let mut motors = [0.0; 4];
    d.inverse(&[0.0, 0.0, z, c], &mut motors);
    for (k, by_hand) in [t13, t24, t13, t24].into_iter().enumerate() {
        assert!((motors[k] / 20.0 - by_hand).abs() < 1e-9, "{motors:?}");
    }
    // The parts' ends: part B less part A is 2 h along d.
    let x = d.coordinates(&[0.0, 0.0, z, c]);
    let apart = sub(d.legs.end(2, x), d.legs.end(0, x));
    close(dot(apart, d.legs.slide), 2.0 * h, 1e-12, "slide");
    let (mut state, mut hold, mut signals) = (Vec::new(), [0.0; 4], Vec::new());
    d.equilibrium(&[0.0, 0.0, z, c], &mut state, &mut hold);
    d.base_signals(&state, &hold, &mut signals);
    let at = d.signal_infos().iter().position(|s| s.id == "plate.slide").unwrap();
    close(signals[at], 0.02, 1e-9, "plate.slide");
}

/// Static hold at the centre, 850 mm down, the tool at c = 0, the forearms lumped (effects.forearm_inertia off): by the
/// four-fold symmetry each arm carries a quarter of the plate. By hand: t = -1.6975 degrees (`angle_by_hand`); the
/// plate's descent per arm angle, all four turning together, from the leg's equation differentiated,
/// dz/dt = L_a (e sin t - z cos t) / (z + L_a sin t), a quarter of it per arm: -0.12203 m/rad. On the plate
/// m = 2 (0.7 + 2 (1/3) 0.7) + 1.0 + 6.0 = 9.3332 kg (the two parts with their forearms' third, carriage, payload);
/// m_LG = 1.4 * 0.25 + 0.5 (0.35 + 0.6667 * 0.7) = 0.75835 kg m. Each gearbox holds -g m_LG cos t + m g dz/dt
/// = -7.43356 - 11.16956 = -18.60312 N m (negative: the arm is held up), the motor that over 20: -0.930156 N m. The
/// same in the rigid model and without the amplifier's spring.
#[test]
fn the_holding_torque_at_the_centre_is_a_quarter_of_the_plates_weight_and_the_arms_own() {
    let z = -0.85;
    let t = angle_by_hand(0.0, 0.0, z);
    let (e, la) = (0.18, 0.5);
    let dz = la * (e * t.sin() - z * t.cos()) / (z + la * t.sin()) / 4.0;
    close(dz, -0.12203, 1e-4, "dz/dt per arm");
    let (m, m_lg) = (2.0 * (0.7 + 2.0 * (1.0 - 0.6667) * 0.7) + 1.0 + 6.0, 1.4 * 0.25 + 0.5 * (0.35 + 0.6667 * 0.7));
    let by_hand = -G * m_lg * t.cos() + m * G * dz;
    assert!((by_hand + 18.603_12).abs() < 1e-5, "{by_hand} N m");
    for model in [
        ["effects.joint_elasticity = true", "effects.amplifier_compliance = true"],
        ["effects.joint_elasticity = false", "effects.amplifier_compliance = false"],
    ] {
        let d = h4(&[&["effects.torque_ripple = false", "effects.forearm_inertia = false"][..], &model[..]].concat());
        close(2.0 * d.part_mass() + d.carried(), m, 1e-9, "plate mass");
        close(d.arm_moment(), m_lg, 1e-9, "m_LG");
        let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
        d.equilibrium(&[0.0, 0.0, z, 0.0], &mut state, &mut torque);
        for k in 0..4 {
            close(state[k], t, 1e-9, "arm angle");
            close(torque[k], by_hand / 20.0, 1e-6, "motor torque");
        }
    }
}

/// The feedforward with the speed is the rigid, loss-free model's torque: given it at a pose where the tool moves at
/// (2, -1, 0.5) m/s turning at 10 rad/s, the model accelerates the tool at the asked (10, 5, -20) m/s^2 and 100 rad/s^2,
/// the velocity term included (to 1e-9; the turn's to 1e-7, kappa = 30 times the slide's). Without the speed it misses
/// by more than 1 m/s^2. At rest the two are the same.
#[test]
fn the_feedforward_with_the_speed_gives_the_asked_acceleration() {
    let d = h4(&[
        "effects.joint_friction = false",
        "effects.motor_friction = false",
        "effects.gearbox_drag = false",
        "effects.torque_ripple = false",
        "effects.gearbox_efficiency = false",
        "effects.joint_elasticity = false",
        "effects.amplifier_compliance = false",
    ]);
    let (p, v, a) = ([0.2, -0.1, -0.9, 0.5], [2.0, -1.0, 0.5, 10.0], [10.0, 5.0, -20.0, 100.0]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 4]);
    d.equilibrium(&p, &mut state, &mut hold);
    d.set_tool_velocity(&mut state, &v);
    let tool_acc = |torque: &[f64]| {
        let mut rates = vec![0.0; d.state_len()];
        d.derivatives(&state, torque, &mut rates);
        let (t, w) = d.arms(&state);
        let x = d.coordinates(&p);
        let j = d.legs.jacobian(t, x).unwrap();
        let bend = d.legs.velocity_term(t, x, w, d.coordinates(&v)).unwrap();
        let along = mul4(&j, [rates[4], rates[5], rates[6], rates[7]]);
        let xa: V4 = std::array::from_fn(|i| along[i] + bend[i]);
        [xa[0], xa[1], xa[2], d.kappa() * xa[3]]
    };
    let (mut moving, mut still) = ([0.0; 4], [0.0; 4]);
    d.feedforward_torque_moving(&p, &v, &a, &mut moving);
    d.feedforward_torque(&p, &a, &mut still);
    let (with, without) = (tool_acc(&moving), tool_acc(&still));
    assert!((0..3).all(|i| (with[i] - a[i]).abs() < 1e-9) && (with[3] - a[3]).abs() < 1e-7, "{with:?}");
    assert!((0..3).map(|i| (without[i] - a[i]).powi(2)).sum::<f64>().sqrt() > 1.0, "{without:?}");
    d.feedforward_torque_moving(&p, &[0.0; 4], &a, &mut moving);
    assert!((0..4).all(|k| (moving[k] - still[k]).abs() < 1e-12));
}

/// Free and loss-free (no friction, damping, backlash or ripple), held by the constant motor torques of the pose and
/// started at (1, -0.5, 0.3) m/s turning at 5 rad/s, the energy less the motors' work, E - sum tau_k rotor_k, stays what
/// it was: over 0.1 s of RK4 at 10 us to 1e-7 of the kinetic energy at the start, with and without the rods, on elastic
/// and rigid joints, with the tool on the amplifier's spring and with it, and with the rods' spin inertia made
/// 0.05 kg m^2 (some 3000 times the config's, so that its share of the energy shows). A velocity term or a gravity that
/// does not belong to the mass matrix shows as a drift.
#[test]
fn a_free_loss_free_motion_keeps_its_energy() {
    for (rods, elastic, spring, spin) in [
        (true, true, true, 1.8e-5),
        (false, false, true, 1.8e-5),
        (true, false, false, 0.05),
        (false, true, false, 1.8e-5),
    ] {
        let mut cfg = lossless_config();
        for e in [
            format!("effects.forearm_inertia = {rods}"),
            format!("effects.joint_elasticity = {elastic}"),
            format!("effects.amplifier_compliance = {spring}"),
            format!("mass.forearm_rod_spin_inertia_kgm2 = {spin:?}"),
            "effects.gearbox_backlash = false".into(),
            "effects.amplifier_backlash = false".into(),
        ] {
            cfg.set_text(&e).unwrap();
        }
        let d = H4::new(&cfg).unwrap();
        let (mut s, mut torque) = (Vec::new(), [0.0; 4]);
        d.equilibrium(&[0.1, -0.05, -0.9, 0.3], &mut s, &mut torque);
        d.set_tool_velocity(&mut s, &[1.0, -0.5, 0.3, 5.0]);
        let budget = |s: &[f64]| {
            let (mut rotor, mut speed) = ([0.0; 4], [0.0; 4]);
            d.motor_state(s, &mut rotor, &mut speed);
            d.energy(s) - (0..4).map(|k| torque[k] * rotor[k]).sum::<f64>()
        };
        let (start, kinetic) = (budget(&s), d.energy_parts(&s)[0]);
        let n = s.len();
        let rate = |s: &[f64]| {
            let mut r = vec![0.0; n];
            d.derivatives(s, &torque, &mut r);
            r
        };
        let h = 1e-5;
        for _ in 0..10_000 {
            let k1 = rate(&s);
            let k2 = rate(&s.iter().zip(&k1).map(|(x, k)| x + 0.5 * h * k).collect::<Vec<_>>());
            let k3 = rate(&s.iter().zip(&k2).map(|(x, k)| x + 0.5 * h * k).collect::<Vec<_>>());
            let k4 = rate(&s.iter().zip(&k3).map(|(x, k)| x + h * k).collect::<Vec<_>>());
            for i in 0..n {
                s[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            }
        }
        let drift = (budget(&s) - start).abs();
        let (mut at, mut vel) = ([0.0; 4], [0.0; 4]);
        d.tool_state(&s, &mut at, &mut vel);
        println!("rods {rods}, elastic {elastic}, spring {spring}: drift {drift:.3e} J of {kinetic:.4} J, at {at:?}");
        assert!(drift < 1e-7 * kinetic, "rods {rods}, elastic {elastic}, spring {spring}: {drift} J of {kinetic} J");
    }
}

/// Held by constant torques (the holding torques of the pose), the stiffness of the potential gravity less torque times
/// angle, by second differences, the rigid model with the lumped forearms: in the symmetric mode (all four arms
/// together, the plate up and down) the Hessian's v^T H v / 4 with v = (1, 1, 1, 1). Worked out by hand (the potential
/// g (m z(theta) - m_LG sum sin theta) by second differences of the forward kinematics, m = 9.333 kg, m_LG = 0.758 kg m)
/// -4.3 N m/rad at the centre 975 mm down, the default range's middle, unstable; +5.7 N m/rad 1175 mm down, the suite's
/// middle, stable. The other three modes are stable at both.
#[test]
fn the_vertical_mode_under_constant_torques_is_unstable_high_up() {
    let mut cfg = lossless_config();
    for e in
        ["effects.joint_elasticity = false", "effects.forearm_inertia = false", "effects.amplifier_compliance = false"]
    {
        cfg.set_text(e).unwrap();
    }
    let m = H4::new(&cfg).unwrap();
    let hessian = |z: f64| {
        let (mut s, mut hold) = (Vec::new(), [0.0; 4]);
        m.equilibrium(&[0.0, 0.0, z, 0.0], &mut s, &mut hold);
        let tau: Vec<f64> = hold.iter().map(|t| t * 20.0).collect();
        let at = |t: V4| {
            let mut st = s.clone();
            st[..4].copy_from_slice(&t);
            m.energy(&st) - (0..4).map(|k| tau[k] * t[k]).sum::<f64>()
        };
        let (t0, h) = ([s[0], s[1], s[2], s[3]], 1e-4);
        let second = |i: usize, j: usize| {
            let f = |a: f64, b: f64| {
                let mut t = t0;
                t[i] += a;
                t[j] += b;
                at(t)
            };
            (f(h, h) - f(h, -h) - f(-h, h) + f(-h, -h)) / (4.0 * h * h)
        };
        let rows: Vec<Vec<f64>> = (0..4).map(|i| (0..4).map(|j| second(i, j)).collect()).collect();
        let vertical = rows.iter().flatten().sum::<f64>() / 4.0;
        let (mut eigen, _) = symmetric_eigen(rows);
        eigen.sort_by(f64::total_cmp);
        (vertical, eigen)
    };
    let ((high, high_modes), (low, low_modes)) = (hessian(-0.975), hessian(-1.175));
    println!("975 mm: {high:.3} N m/rad, {high_modes:?}; 1175 mm: {low:.3}, {low_modes:?}");
    assert!((high + 4.3).abs() < 0.2 && (low - 5.7).abs() < 0.2, "{high}, {low}");
    // The lowest of each is the vertical; the other three are stable.
    assert!((high_modes[0] - high).abs() < 1e-3 && high_modes[1] > 0.0 && low_modes[0] > 0.0);
}

/// The tool on the amplifier's spring, the slide held: w^2 = k / I_t. With the payload I_t = 0.003 + 0.01 kg m^2:
/// sqrt(3000 / 0.013) / 2 pi = 76.46 Hz, damping 0.25 / (2 sqrt(3000 * 0.013)) = 0.0200; without it 159.15 Hz. It is
/// among the modes, not the arms' resonance the shaper is set by; the rigid model has no amplifier mode.
#[test]
fn the_amplifier_mode_is_the_hand_calculated_one() {
    let mut d = h4(&[]);
    let mode = d.amplifier_mode().unwrap();
    close(mode.hz, (3000.0f64 / 0.013).sqrt() / std::f64::consts::TAU, 1e-12, "with the payload");
    close(mode.hz, 76.46, 1e-3, "with the payload, Hz");
    close(mode.damping, 0.25 / (2.0 * 39f64.sqrt()), 1e-12, "damping");
    let pose = [0.0, 0.0, -0.9, 0.0];
    assert!(d.modes(&pose).iter().any(|m| (m.hz - mode.hz).abs() < 1e-9) && d.modes(&pose).len() == 5);
    assert!(d.resonance_hz(&pose).is_some_and(|hz| (hz - mode.hz).abs() > 1.0));
    d.set_payload_attached(false);
    close(d.amplifier_mode().unwrap().hz, 159.15, 1e-3, "without the payload");
    assert!(h4(&["effects.amplifier_compliance = false"]).amplifier_mode().is_none());
}

/// Targets are refused with their reason: beyond the cylinder, with too few numbers; the centre and the cycle's
/// corners at either end of the turn are not.
#[test]
fn targets_outside_the_working_range_are_refused_with_their_reason() {
    let d = h4(&[]);
    assert_eq!(d.unreachable(&[0.7, 0.0, -0.9, 0.0], 0.0), Some("error-delta-outside-envelope"));
    assert_eq!(d.unreachable(&[0.0, 0.0, -0.9], 0.0), Some("error-target-out-of-range"));
    assert_eq!(d.unreachable(&[0.0, 0.0, -1.3, 0.0], 0.0), Some("error-delta-outside-envelope"));
    let half_turn = std::f64::consts::PI;
    for tool in [[0.0, 0.0, -0.9, 0.0], [0.1525, 0.0, -0.875, -half_turn], [-0.1525, 0.0, -0.85, half_turn]] {
        assert_eq!(d.unreachable(&tool, 0.0), None, "{tool:?}");
    }
}
