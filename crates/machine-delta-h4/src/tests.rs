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

#[track_caller]
fn close(a: f64, b: f64, tolerance: f64, what: &str) {
    sim_core::testkit::record_check(what, b, a, tolerance * b.abs().max(1e-12));
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
/// dz/dt = L_a (e sin t - z cos t) / (z + L_a sin t), a quarter of it per arm: -0.12203 m/rad. The inertia has the
/// plate's m = 2 (0.7 + 2 (1/3) 0.7) + 1.0 + 6.0 = 9.3332 kg (the two parts with their forearms' third, carriage,
/// payload) and m_LG = 1.4 * 0.25 + 0.5 (0.35 + 0.6667 * 0.7) = 0.75835 kg m; the weight has each forearm pair's at its
/// middle, half at each end: on the plate 2 (0.7 + 0.7) + 7.0 = 9.8 kg, m_LG = 1.4 * 0.25 + 0.5 (0.35 + 0.35) =
/// 0.70 kg m. Each gearbox holds -g m_LG cos t + m g dz/dt = -6.86164 - 11.72815 = -18.58980 N m (negative: the arm
/// is held up), the motor that over 20: -0.929490 N m (with the weight at the inertia's split it held -18.60312). The
/// same in the rigid model and without the amplifier's spring.
#[test]
fn the_holding_torque_at_the_centre_is_a_quarter_of_the_plates_weight_and_the_arms_own() {
    let z = -0.85;
    let t = angle_by_hand(0.0, 0.0, z);
    let (e, la) = (0.18, 0.5);
    let dz = la * (e * t.sin() - z * t.cos()) / (z + la * t.sin()) / 4.0;
    close(dz, -0.12203, 1e-4, "dz/dt per arm");
    let (m, m_lg) = (2.0 * (0.7 + 2.0 * (1.0 - 0.6667) * 0.7) + 1.0 + 6.0, 1.4 * 0.25 + 0.5 * (0.35 + 0.6667 * 0.7));
    let (weight, weight_lg) = (2.0 * (0.7 + 0.7) + 1.0 + 6.0, 1.4 * 0.25 + 0.5 * (0.35 + 0.35));
    let by_hand = -G * weight_lg * t.cos() + weight * G * dz;
    assert!((by_hand + 18.589_80).abs() < 1e-5, "{by_hand} N m");
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

/// The forearms' weight is at the frame momentum's first moment, lumped or rods, rigid or bending arms: the potential
/// changes along a motion as -g . dc/dt (`testkit::weight_is_at_the_first_moment`). And along the default cycle with
/// the forearms lumped (effects.forearm_inertia off), V + g . c stays what it was to 1e-6 J. Lumped, the weight and the
/// frame momentum both had the forearms at the inertia's split (2/3 at the elbow), where a rod's weight is at its middle.
#[test]
fn the_forearms_weight_is_at_their_middles() {
    for rods in [false, true] {
        for bending in [false, true] {
            sim_core::testkit::weight_is_at_the_first_moment(&h4(&[
                &format!("effects.forearm_inertia = {rods}"),
                &format!("effects.arm_bending = {bending}"),
            ]));
        }
    }
    let mut cfg = ENTRY.config();
    cfg.set_text("effects.forearm_inertia = false").unwrap();
    let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
    sim.command(sim_components::Command::StartCycle).unwrap();
    let at = sim.machine.signal_infos().iter().position(|s| s.id == "energy.potential").unwrap();
    let (start, mut out, mut sum) = (sim.control.cycle_passes(), Vec::new(), Vec::new());
    while sim.control.cycle_passes() < start + 2 {
        sim.step();
        out.clear();
        sim.machine.signals(&sim.state, &[0.0; 4], &mut out);
        let f = sim.machine.frame_momentum(&sim.state).unwrap();
        sum.push(out[at] + (0..3).map(|k| f.gravity[k] * f.first_moment[k]).sum::<f64>());
    }
    let spread =
        sum.iter().copied().fold(f64::NEG_INFINITY, f64::max) - sum.iter().copied().fold(f64::INFINITY, f64::min);
    assert!(sum.len() > 1000 && spread < 1e-6, "V + g . c spans {spread} J over {} steps", sum.len());
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
/// g (m z(theta) - m_LG sum sin theta) by second differences of the forward kinematics, the forearms' weight at their
/// middles: m = 9.8 kg, m_LG = 0.70 kg m) -4.71 N m/rad at the centre 975 mm down, the default range's middle, unstable
/// (-4.30 with the weight at the inertia's split, 9.333 kg and 0.758 kg m); +5.45 N m/rad 1175 mm down, the suite's
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
    assert!((high + 4.71).abs() < 0.2 && (low - 5.45).abs() < 0.2, "{high}, {low}");
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

/// With hysteresis in the gearboxes (`gearbox_lost_motion`, hysteresis_rad > 0) a small vibration sees K1 + sigma, and
/// sigma is K1: the arms' modes stand sqrt 2 above the large-amplitude ones (the gears are their only spring here, the
/// amplifier off). The amplifier's mode has no hysteresis and stays. Without hysteresis (as shipped) both lists agree.
#[test]
fn the_small_amplitude_modes_stand_sqrt_two_above_the_large_ones_with_hysteresis() {
    let pose = [0.0, 0.0, -0.9, 0.0];
    let d = h4(&["effects.amplifier_compliance = false", "gearbox.hysteresis_rad = 0.0003"]);
    let (large, small) = (d.modes(&pose), d.modes_small_amplitude(&pose));
    assert!(large.len() == 4 && small.len() == 4, "{large:?} {small:?}");
    for (l, s) in large.iter().zip(&small) {
        close(s.hz, l.hz * 2f64.sqrt(), 1e-9, "small-amplitude mode");
        close(s.damping, l.damping / 2f64.sqrt(), 1e-9, "its damping ratio, c / (2 w mu)");
    }
    let amplified = h4(&["gearbox.hysteresis_rad = 0.0003"]);
    let mode = amplified.amplifier_mode().unwrap();
    assert!(amplified.modes_small_amplitude(&pose).iter().any(|m| m.hz == mode.hz));
    let shipped = h4(&[]);
    assert_eq!(shipped.modes(&pose), shipped.modes_small_amplitude(&pose));
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

#[test]
fn the_four_arm_delta_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ENTRY);
}

/// A machine rebuilt with the gearbox ratio 1 % higher during a move (`Sim::reconfigure`) takes the state over with
/// the tool where it was, at its speed, on its amplifier's spring as twisted as before (here 1 mrad, the tool turning
/// 2 rad/s slower than the slide), and each gearbox twisted at its motor as before (here 0.5 mrad at the arm, 10 mrad
/// at the motor): rebuilt at rest, the spring was unloaded and the gears laid in the middle of their play, a jump of
/// 0.19 mrad on the tool's turn in the first 10 ms after the change in the default config's move (0.02 kept).
#[test]
fn a_changed_ratio_keeps_the_tool_on_its_spring_and_the_gears_twisted() {
    let (old, new) = (h4(&[]), h4(&["gearbox.ratio = 20.2"]));
    let (tool, vel) = ([0.1, -0.05, -0.9, 0.3], [1.0, -0.5, 0.3, 5.0]);
    let (mut s, mut torque) = (Vec::new(), [0.0; 4]);
    old.equilibrium(&tool, &mut s, &mut torque);
    old.set_tool_velocity(&mut s, &vel);
    let at = old.turn_at();
    (s[at], s[at + 1]) = (tool[3] - 1e-3, vel[3] - 2.0);
    for k in 0..4 {
        s[8 + k] = 20.0 * (s[k] + 5e-4);
    }
    let adopted = new.adopt_state(&old, &s);
    let state = |m: &H4, s: &[f64]| {
        let (mut p, mut v) = ([0.0; 4], [0.0; 4]);
        m.tool_state(s, &mut p, &mut v);
        (p, v, m.twist(s).0)
    };
    let ((p0, v0, twist0), (p1, v1, twist1)) = (state(&old, &s), state(&new, &adopted));
    for k in 0..4 {
        assert!((p1[k] - p0[k]).abs() < 1e-9 && (v1[k] - v0[k]).abs() < 1e-9, "{p1:?} {v1:?} against {p0:?} {v0:?}");
    }
    close(twist0, 1e-3, 1e-6, "twist before");
    close(twist1, twist0, 1e-6, "twist after");
    let (mut before, mut after) = ([0.0; 4], [0.0; 4]);
    old.inverse(&[p0[0], p0[1], p0[2], p0[3] + twist0], &mut before);
    new.inverse(&[p0[0], p0[1], p0[2], p0[3] + twist0], &mut after);
    for k in 0..4 {
        close(adopted[8 + k] - after[k], s[8 + k] - before[k], 1e-6, "twist at the motor");
        close(s[8 + k] - before[k], 0.01, 1e-6, "twist at the motor, before");
    }
}

/// The fit's window (`identify::smoothing_span`) on traces of 6 s at 1 ms whose four motors turn at +-1 rad/s in runs
/// of L rows, the arms standing at the start pose. A row counts for a direction where the 2 h - 1 rows about it keep
/// its sign: L - 2h + 2 rows in each whole run; the loop over rows h + 1 to 6000 - h - 1 loses 2 of the first run and 1
/// of the last; a direction needs 10 (2h + 1). L = 100: 30 runs each way, 30 (102 - 2h) - 2 >= 20 h + 10 holds up to
/// h = 38 (778 of 770; 39: 718 of 790), so 38 ms, where the default 40 ms leaves 30 * 22 - 2 = 658 of 810. L = 40: 75
/// runs, 75 (42 - 2h) - 2 >= 20 h + 10 up to h = 18, shorter than one period of the machine's lowest mode allows
/// (below 54 Hz: over 18.5 rows), so the trace keeps 40 ms and is not fitted.
#[test]
fn the_fit_window_is_the_longest_its_lifts_allow_down_to_its_lowest_mode() {
    use sim_components::identify::{SMOOTHING_S, smoothing_span, turns_both_ways};
    use sim_components::trace::{MotorTrace, Trace};
    let d = h4(&[]);
    let pose = [0.1525, 0.0, -0.85, 0.0];
    let mut motors = [0.0; 4];
    d.inverse(&pose, &mut motors);
    let lowest = d.modes(&pose)[0].hz;
    assert!(lowest < 54.0, "{lowest} Hz");
    let trace = |run: usize| Trace {
        time: (0..=6000).map(|i| i as f64 * 1e-3).collect(),
        motors: motors
            .iter()
            .map(|&angle| {
                let speed: Vec<f64> =
                    (0..=6000).map(|i| if (i / run).is_multiple_of(2) { 1.0 } else { -1.0 }).collect();
                MotorTrace { setpoint: vec![angle; 6001], angle: vec![angle; 6001], speed, torque: vec![0.0; 6001] }
            })
            .collect(),
        ..Trace::default()
    };
    let long = trace(100);
    close(smoothing_span(&d, &long), 0.038, 1e-9, "100 ms runs");
    assert!(turns_both_ways(&d, &long));
    let short = trace(40);
    close(smoothing_span(&d, &short), SMOOTHING_S, 1e-12, "40 ms runs");
    assert!(!turns_both_ways(&d, &short));
}

/// On a compliant base with effects.base_reaction (x, y 2.7e6 N/m and z 1.9e7 N/m as shipped, 300 kg; tilts about x
/// and y of 5e5 N m/rad added, 100 kg m^2), the base undamped and the losses off (the standard suite's rigid config): the
/// arms, the plate and the base keep their energy as the suite's check has it, the base's sway with the machine's
/// momentum and the weight's work on its tilt included. With the shipped damping ratio (0.02) and losses the loss budget
/// closes with the base's damping loss in it.
#[test]
fn on_a_coupled_base_the_four_arm_delta_keeps_its_energy_and_budget() {
    let on = |mut cfg: Config, zeta: f64| {
        for e in ["effects.base_compliance = true", "effects.base_reaction = true"] {
            cfg.set_text(e).unwrap();
        }
        for axis in ["x", "y"] {
            cfg.set_text(&format!("base.tilt_stiffness_{axis}_nm_rad = 5.0e5")).unwrap();
        }
        for id in sim_components::base::DOFS {
            cfg.set_text(&format!("base.damping_ratio_{id} = {zeta:?}")).unwrap();
        }
        build(&cfg).unwrap()
    };
    sim_core::testkit::energy_conservation(on((ENTRY.suite_configs)()[0].clone(), 0.0).as_ref());
    sim_core::testkit::loss_budget(on(ENTRY.config(), 0.02).as_ref());
}

/// Friction feedforward from the machine's own model (`steady_friction`), by hand, with the tool on the axis at
/// z = -0.95 m, unturned, going down at 0.5 m/s: the slide stands (h = 0), so each of the four legs is the D3's
/// (e = 0.25 - 0.07 = 0.18, L_a = 0.5, L_b = 1.1): 2 e L_a cos t + 2 z L_a sin t + e^2 + z^2 + L_a^2 - L_b^2 = 0, elbow
/// out t = 0.161292 rad, t' = -(2 L_a sin t + 2 z) z' / (2 z L_a cos t - 2 e L_a sin t) = 0.899777 rad/s. The joint:
/// 0.4 tanh(t' / 0.01) + 0.06 t' = 0.453987 N m. The rotor at 20 t' = 17.99554 rad/s: 0.06 (the breakaway's Stribeck
/// part exp(-81) nothing) + 2.5e-4 x 17.996 = 0.004499, and the gearbox's drag 0.93 (17.996 / 314.159)^(2/3) =
/// 0.138195: 0.202694 N m. The teeth lose nothing (the drag takes the catalogue's whole loss), so 0.202694 + 0.453987 /
/// 20 = 0.225393 N m per arm. In a vertical cruise of the shipped (elastic) machine at 0.5 m/s the velocity loops'
/// integrators then carry almost nothing; without the feedforward they carry the friction.
#[test]
fn the_machine_friction_feedforward_is_the_steady_friction_and_the_integrator_carries_nothing() {
    let m = (ENTRY.build)(&ENTRY.config()).unwrap();
    let mut out = [0.0; 4];
    m.steady_friction(&[0.0, 0.0, -0.95, 0.0], &[0.0, 0.0, -0.5, 0.0], &mut out);
    for f in out {
        assert!((f - 0.225393).abs() < 1e-5, "{out:?}");
    }
    let cruise = |model: bool| {
        let mut cfg = ENTRY.config();
        cfg.set_text("effects.friction_feedforward = true").unwrap();
        let source = if model { "machine" } else { "characteristic" };
        cfg.set_text(&format!("drive.friction_feedforward_model = \"{source}\"")).unwrap();
        cfg.set_text("start.pose = [0.0, 0.0, -0.8, 0.0]").unwrap();
        cfg.set_text("axis.z.max_velocity_m_s = 0.5").unwrap();
        let mut sim = sim_components::Sim::new(&ENTRY, &cfg).unwrap();
        let at = sim.control.set_pos.clone();
        sim.command(sim_components::Command::MoveTo(vec![at[0], at[1], -1.1, at[3]])).unwrap();
        let (mut ff, mut integrator, mut n) = (0.0, 0.0, 0);
        sim.run(0.6, |s| {
            if s.time() > 0.15 && s.control.set_acc[2] == 0.0 && (s.control.set_vel[2] + 0.5).abs() < 1e-12 {
                n += 1;
                for k in 0..4 {
                    ff += s.control.drives[k].friction_ff;
                    integrator += s.control.drives[k].controller_torque;
                }
            }
        });
        assert!(n > 1000, "{n} samples in the cruise");
        (ff / (4 * n) as f64, integrator / (4 * n) as f64)
    };
    let ((ff, with), (_, without)) = (cruise(true), cruise(false));
    assert!(with.abs() < 0.03 * ff, "the integrators carry {with} N m of {ff}");
    assert!((without - with - ff).abs() < 0.03 * ff, "without: {without} N m, with {with}, of {ff}");
}

/// The rods' spin about their own axes is in the frame momentum (effects.forearm_inertia): with the spin inertia
/// 1.8e-5 kg m^2 per rod (3.6e-5 per pair) against none, the angular momentum differs by the sum over the four pairs of
/// I s u / L. Each pair's plane (the elbow's axis a and the forearm u) rotates rigidly, so s is the turning rate of
/// the frame (e, u x e, u) with e = a less its part along u, normalised: s = e' . (u x e), here by central differences
/// of 1e-6 s along the tool's straight path at (0.3, -0.2, 0.4) m/s, an independent route from the closed form.
#[test]
fn the_rods_spin_is_in_the_frame_momentum_by_a_finite_difference() {
    let (with, without) = (
        h4(&["effects.forearm_inertia = true"]),
        h4(&["effects.forearm_inertia = true", "mass.forearm_rod_spin_inertia_kgm2 = 0.0"]),
    );
    let (mut state, mut torque) = (Vec::new(), vec![0.0; 4]);
    with.equilibrium(&[0.1, 0.05, -0.9, 0.0], &mut state, &mut torque);
    with.set_tool_velocity(&mut state, &[0.3, -0.2, 0.4, 0.0]);
    let (t, w) = with.arms(&state);
    let (x, j) = with.pose(t);
    let xv = mul4(&j, w);
    let frame = |at: f64, i: usize| -> (V3, V3) {
        let xt: V4 = std::array::from_fn(|k| x[k] + at * xv[k]);
        let tt = with.legs.inverse(xt).unwrap();
        let u = sub(with.legs.end(i, xt), with.legs.elbow(i, tt[i])[0]);
        let a = with.legs.tangent(i);
        let uh = u.map(|v| v / norm(u));
        let e = sub(a, uh.map(|v| v * dot(a, uh)));
        (e.map(|v| v / norm(e)), uh)
    };
    let mut expected = [0.0; 3];
    for i in 0..4 {
        let (h, (e, uh)) = (1e-6, frame(0.0, i));
        let (plus, minus) = (frame(h, i).0, frame(-h, i).0);
        let de: V3 = std::array::from_fn(|k| (plus[k] - minus[k]) / (2.0 * h));
        let spin = dot(de, cross(uh, e));
        for k in 0..3 {
            expected[k] += 3.6e-5 * spin * uh[k];
        }
    }
    let (a, b) = (with.frame_momentum(&state).unwrap(), without.frame_momentum(&state).unwrap());
    let seen: V3 = std::array::from_fn(|k| a.angular[k] - b.angular[k]);
    assert!(norm(expected) > 1e-8, "{expected:?}");
    for k in 0..3 {
        close(seen[k], expected[k], 1e-5, &format!("rods' spin momentum, axis {k}"));
    }
}

/// The slanted pieces are capsules along their own direction and the tool's bar is one: the two plate parts, eight
/// forearm rods, four upper arms and the bar make 15, the bar of radius 7.5 mm from p - 0.06 (cos c, sin c, 0) to
/// p + 0.06 (cos c, sin c, 0), 40 mm under the carriage's point (c = 0.5 rad here); the carriage and the payload stay
/// boxes.
#[test]
fn the_slanted_pieces_and_the_tool_bar_are_capsules() {
    let m = h4(&[]);
    let (mut state, mut torque) = (Vec::new(), vec![0.0; 4]);
    m.equilibrium(&[0.1, 0.05, -0.9, 0.5], &mut state, &mut torque);
    let capsules = m.collision_capsules(&state);
    assert_eq!(capsules.len(), 15);
    let (c, p) = (m.turn(&state).0, [0.1, 0.05, -0.9]);
    close(c, 0.5, 1e-12, "the tool's turn");
    let ends = [
        [p[0] - 0.06 * c.cos(), p[1] - 0.06 * c.sin(), p[2] - 0.04],
        [p[0] + 0.06 * c.cos(), p[1] + 0.06 * c.sin(), p[2] - 0.04],
    ];
    let near = |a: V3, b: V3| norm(sub(a, b)) < 1e-9;
    assert!(
        capsules.iter().any(|k| k.radius == 0.0075
            && (near(k.from, ends[0]) && near(k.to, ends[1]) || near(k.from, ends[1]) && near(k.to, ends[0]))),
        "no bar capsule at {ends:?}"
    );
    let boxes = m.collision_bodies(&state);
    assert!(boxes.len() <= 2, "{} boxes", boxes.len());
}

/// The shipped tube's bending stiffness by hand: a carbon tube 60 x 2.5 mm, E 100 GPa, I = pi (0.06^4 - 0.055^4) / 64 =
/// 1.869923e-7 m^4, as a cantilever of L_a = 0.5 m: k_b = 3 E I / L_a = 112195.35 N m/rad in its plane and
/// k_l = 3 E I / L_a^3 = 448781.4 N/m out of it.
const K_BEND: f64 = 112195.35;

#[test]
fn the_arms_bending_stiffnesses_are_the_tubes() {
    let d = h4(&["effects.arm_bending = true"]);
    let b = d.bending.unwrap();
    close(b.k, K_BEND, 1e-7, "in the plane");
    close(b.k_lateral, 448781.4, 1e-7, "out of it");
    let mut cfg = ENTRY.config();
    cfg.set_text("effects.arm_bending = true").unwrap();
    cfg.set_text("upper_arm.tube_wall_m = 0.031").unwrap();
    assert!(H4::new(&cfg).is_err(), "a wall thicker than the tube's radius");
}

/// The bend of an arm at rest under a hand-calculated tip load (effects.arm_bending). At the centre, 850 mm down, the
/// tool at c = 0, the forearms lumped: the arm's elbow is at t = -1.6975 degrees (`angle_by_hand`), by the four-fold
/// symmetry each arm's forearm pair carries a quarter of the plate, and there is no force along the elbow's axis (the
/// forearms lie in the arm's plane), so the elbows do not move out of it. What bends the arm is the torque the elbow
/// carries about the shoulder: the weight of what sits at the elbow, g m_tip cos t with the forearms' weight half at
/// each end, m_tip = L_a (m_elbow + 1/2 m_fb) = 0.5 (0.35 + 0.35) = 0.35 kg m, and the plate's, m g |dz/dt| (m = 9.8 kg
/// of weight, dz/dt = -0.12203 m/rad per arm as in the holding-torque test): Q = 3.4308 + 11.7282 = 15.159 N m, so
/// delta = Q / k_b = 15.159 / 112195.35 = 1.3511e-4 rad (`K_BEND`). The joint then stands short of the elbow by that, t - delta, and its
/// gearbox holds the tube's weight and Q: -(g 0.35 cos(t - delta) + Q) / 20 at the motor. The plate is where the pose says.
#[test]
fn an_arm_bends_by_the_tip_torque_over_its_stiffness_at_rest() {
    let z = -0.85;
    let t = angle_by_hand(0.0, 0.0, z);
    let (e, la) = (0.18, 0.5);
    let dz = la * (e * t.sin() - z * t.cos()) / (z + la * t.sin()) / 4.0;
    let m = 2.0 * (0.7 + 0.7) + 1.0 + 6.0;
    let tip_moment = la * (0.35 + 0.35);
    let q = G * tip_moment * t.cos() - m * G * dz;
    assert!((q - 15.159).abs() < 2e-3, "{q} N m");
    let mut cfg = lossless_config();
    for e in [
        "effects.arm_bending = true",
        "effects.forearm_inertia = false",
        "effects.gearbox_backlash = false",
        "effects.amplifier_backlash = false",
    ] {
        cfg.set_text(e).unwrap();
    }
    let d = H4::new(&cfg).unwrap();
    let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
    d.equilibrium(&[0.0, 0.0, z, 0.0], &mut state, &mut torque);
    assert_eq!(state.len(), d.state_len());
    let delta = q / K_BEND;
    let at = d.bend_at();
    for k in 0..4 {
        close(state[at + k], delta, 1e-6, "bend");
        assert!(state[at + 8 + k].abs() < 1e-12, "{}", state[at + 8 + k]);
        close(state[k], t - delta, 1e-9, "joint angle");
        close(torque[k], -(G * 0.35 * (t - delta).cos() + q) / 20.0, 1e-6, "motor torque");
    }
    let (mut pos, mut vel) = ([0.0; 4], [0.0; 4]);
    d.tool_state(&state, &mut pos, &mut vel);
    for (k, by_hand) in [0.0, 0.0, z, 0.0].into_iter().enumerate() {
        assert!((pos[k] - by_hand).abs() < 1e-9, "{pos:?}");
    }
    // The state is at rest: nothing accelerates it.
    let mut rate = vec![0.0; state.len()];
    d.derivatives(&state, &torque, &mut rate);
    for k in 4..8 {
        assert!(rate[k].abs() < 1e-6, "arm {k}: {} rad/s^2", rate[k]);
        assert!(rate[at + k].abs() < 1e-6, "bend {k}: {} rad/s^2", rate[at + k]);
    }
}

/// The bend mode by hand. With rigid gearboxes (the roots held by the position loops) and the arms lumped, at the
/// centre the four elbows moving together move the plate straight down (the symmetry of the four arms: no x, y or
/// slide), z' = 4 J_z per rad, J_z = dz/dt per arm = -0.12203 m/rad. The elbows carry I_tip = L_a^2 (m_elbow + 2/3 m_fb)
/// = 0.25 * 0.81669 = 0.204173 kg m^2 each and the plate m = 9.3332 kg, so with the springs k_b on the four elbows
/// v = (1, 1, 1, 1) is an eigenvector of M with M v = (I_tip + 4 m J_z^2) v = 0.76009 v, and w^2 = k_b / 0.76009
/// = 147607 1/s^2, 61.15 Hz. The forearms lie in the arms' planes, so the elbows' moves out of them are decoupled
/// there. The eight modes of the bending alone include it.
#[test]
fn the_bend_mode_of_the_four_elbows_together_is_the_hand_calculated_one() {
    let z = -0.85;
    let t = angle_by_hand(0.0, 0.0, z);
    let (e, la) = (0.18, 0.5);
    let jz = la * (e * t.sin() - z * t.cos()) / (z + la * t.sin()) / 4.0;
    let m = 2.0 * (0.7 + 2.0 * (1.0 - 0.6667) * 0.7) + 1.0 + 6.0;
    let i_tip = la * la * (0.35 + 0.6667 * 0.7);
    let w2 = K_BEND / (i_tip + 4.0 * m * jz * jz);
    let by_hand = w2.sqrt() / std::f64::consts::TAU;
    close(by_hand, 61.15, 2e-4, "the mode's frequency by hand");
    let d = h4(&[
        "effects.arm_bending = true",
        "effects.forearm_inertia = false",
        "effects.joint_elasticity = false",
        "effects.amplifier_compliance = false",
    ]);
    let modes = d.modes_at(&[0.0, 0.0, z, 0.0], false);
    let all: Vec<f64> = modes.iter().map(|m| m.0).collect();
    assert_eq!(modes.len(), 8, "{all:?}");
    let found = modes.iter().find(|mode| (mode.0 - by_hand).abs() < 1e-6 * by_hand);
    let (hz, _, shape) = found.unwrap_or_else(|| panic!("{by_hand} Hz not in {all:?}"));
    // Its shape: the four elbows alike, the roots still.
    for k in 0..4 {
        assert!(shape[k].abs() < 1e-9 * shape[4].abs(), "root {k} moves: {shape:?}");
        assert!((shape[4 + k] - shape[4]).abs() < 1e-6 * shape[4].abs(), "{hz} Hz: {shape:?}");
    }
}

/// Free and loss-free (no friction, damping, backlash or ripple, the bending's dampers made zero), held by the constant
/// motor torques of the pose and started at (1, -0.5, 0.3) m/s turning at 5 rad/s, the energy less the motors' work stays
/// what it was with the arms bending in and out of their planes: over 0.1 s of RK4 at 10 us to 1e-12 of the kinetic
/// energy at the start, with and without the rods (their spin made 0.05 kg m^2 so that its share shows), on elastic and
/// rigid joints, with the tool on the amplifier's spring and with it turning with the slide, and with the tube's own
/// share and the elbow's twist (effects.arm_tube_inertia). A velocity term or a gravity that does not belong to the mass
/// matrix of the 8 coordinates shows as a drift.
#[test]
fn a_free_loss_free_motion_with_bending_arms_keeps_its_energy() {
    for (rods, elastic, spring, spin, tube) in [
        (true, true, true, 0.05, false),
        (false, false, false, 1.8e-5, false),
        (true, false, true, 1.8e-5, false),
        (true, true, true, 0.05, true),
    ] {
        let mut cfg = lossless_config();
        for e in [
            "effects.arm_bending = true".to_string(),
            format!("effects.arm_tube_inertia = {tube}"),
            format!("effects.forearm_inertia = {rods}"),
            format!("effects.joint_elasticity = {elastic}"),
            format!("effects.amplifier_compliance = {spring}"),
            format!("mass.forearm_rod_spin_inertia_kgm2 = {spin:?}"),
            "effects.gearbox_backlash = false".into(),
            "effects.amplifier_backlash = false".into(),
            "upper_arm.bending_damping_nm_s_rad = 0.0".into(),
            "upper_arm.lateral_damping_n_s_m = 0.0".into(),
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
        let (mut bent, at, rest) = (0f64, d.bend_at(), s.clone());
        for _ in 0..10_000 {
            let k1 = rate(&s);
            let k2 = rate(&s.iter().zip(&k1).map(|(x, k)| x + 0.5 * h * k).collect::<Vec<_>>());
            let k3 = rate(&s.iter().zip(&k2).map(|(x, k)| x + 0.5 * h * k).collect::<Vec<_>>());
            let k4 = rate(&s.iter().zip(&k3).map(|(x, k)| x + h * k).collect::<Vec<_>>());
            for i in 0..n {
                s[i] += h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
            }
            bent = bent.max((0..4).map(|k| (s[at + k] - rest[at + k]).abs()).fold(0.0, f64::max));
        }
        let drift = (budget(&s) - start).abs();
        println!(
            "rods {rods}, elastic {elastic}, spring {spring}: drift {drift:.3e} J of {kinetic:.4} J, bend up to {bent:.2e} rad"
        );
        assert!(drift < 1e-12 * kinetic, "rods {rods}, elastic {elastic}, spring {spring}: {drift} J of {kinetic} J");
        // The arms did bend further (the motion's own acceleration rings them), so the test sees the bending's terms.
        assert!(bent > 1e-6, "{bent} rad");
    }
}

/// With the switch on the machine's signals add up: the base and effect values are as many as the infos, the bending's
/// loss power among them, and with it off the state and the signals are as before (no bend states, no `arm_bending`).
#[test]
fn the_bending_adds_its_states_and_its_loss_signal_and_nothing_with_the_switch_off() {
    let (on, off) = (h4(&["effects.arm_bending = true"]), h4(&[]));
    assert_eq!(on.state_len(), off.state_len() + 16);
    for (d, loss) in [(&on, true), (&off, false)] {
        let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
        d.equilibrium(&[0.1, 0.05, -0.9, 0.2], &mut state, &mut torque);
        assert_eq!(state.len(), d.state_len());
        let mut values = Vec::new();
        d.base_signals(&state, &torque, &mut values);
        d.effect_signals(&state, &torque, &mut values);
        let infos = d.signal_infos();
        assert_eq!(values.len(), infos.len());
        assert_eq!(infos.iter().any(|i| i.id.ends_with("arm_bending")), loss);
    }
}

/// The tube's own share (effects.arm_tube_inertia) by hand, at the centre 850 mm down with rigid gearboxes (the roots
/// held), the forearms lumped (m_elbow + 2/3 m_fb = 0.816690 kg at each elbow). Out of the planes the forearms lie in
/// the arms' planes, so each elbow's move is a mode of its own on k_l = 448781.4 N/m: the elbow, a quarter of the tube
/// (0.35 kg) and the elbow's twist, J = 2.9e-4 + 2/3 0.7 (0.1 / 2)^2 = 1.456723e-3 kg m^2 times (1.37651 / 0.5)^2 =
/// 7.579165: m = 0.816690 + 0.35 + 0.011041 = 1.177731 kg, 98.25 Hz (without the share 0.816690 kg, 117.98 Hz). In the
/// planes the four elbows together (as `the_bend_mode_of_the_four_elbows_together_is_the_hand_calculated_one`) carry a
/// quarter of the tube's inertia about the shoulder more, 1.4 0.25 / 4 = 0.0875 kg m^2: k_b / (0.76009 + 0.0875) =
/// 132369 1/s^2, 57.91 Hz.
#[test]
fn the_tubes_share_moves_the_bend_modes_by_hand() {
    let lateral = |tube: bool| 448781.4 / (0.816690 + if tube { 0.35 + 1.456723e-3 * 7.579165 } else { 0.0 });
    let z = -0.85;
    let t = angle_by_hand(0.0, 0.0, z);
    let (e, la) = (0.18, 0.5);
    let jz = la * (e * t.sin() - z * t.cos()) / (z + la * t.sin()) / 4.0;
    let m = 2.0 * (0.7 + 2.0 * (1.0 - 0.6667) * 0.7) + 1.0 + 6.0;
    let i_tip = la * la * (0.35 + 0.6667 * 0.7);
    let together = |tube: bool| K_BEND / (i_tip + if tube { 0.0875 } else { 0.0 } + 4.0 * m * jz * jz);
    let hz = |w2: f64| w2.sqrt() / std::f64::consts::TAU;
    close(hz(lateral(true)), 98.25, 2e-4, "out of the planes, by hand");
    close(hz(lateral(false)), 117.98, 2e-4, "without the share");
    close(hz(together(true)), 57.91, 2e-4, "in the planes, by hand");
    for tube in [false, true] {
        let d = h4(&[
            "effects.arm_bending = true",
            &format!("effects.arm_tube_inertia = {tube}"),
            "effects.forearm_inertia = false",
            "effects.joint_elasticity = false",
            "effects.amplifier_compliance = false",
        ]);
        let modes = d.modes_at(&[0.0, 0.0, z, 0.0], false);
        let all: Vec<f64> = modes.iter().map(|m| m.0).collect();
        let out = modes.iter().filter(|m| (m.0 - hz(lateral(tube))).abs() < 1e-6 * m.0).count();
        assert_eq!(out, 4, "{tube}: {} Hz four times in {all:?}", hz(lateral(tube)));
        assert!(all.iter().any(|f| (f - hz(together(tube))).abs() < 1e-6 * f), "{tube}: {all:?}");
    }
}

/// With bending arms, the tube's share and the rods, the frame momentum's first moment moves at its linear momentum
/// (`testkit::frame_momentum_moves_its_first_moment`): the elbow at its moved place with its whole speed, the tube's
/// sections out of the plane (0.3915 of its mass) and the rods' middles at their true places.
#[test]
fn the_frame_momentum_follows_the_bending_arms() {
    for tube in [false, true] {
        let d = h4(&["effects.arm_bending = true", &format!("effects.arm_tube_inertia = {tube}")]);
        sim_core::testkit::frame_momentum_moves_its_first_moment(&d);
    }
}

/// The tube's kinetic energy with effects.arm_tube_inertia by hand. Its sections move at s theta' + phi(s) L delta':
/// T = m L^2 (theta'^2 / 3 + 2 0.284413 theta' delta' + delta'^2 / 4) / 2 per arm, m L^2 = 1.4 * 0.25 = 0.35 kg m^2 and
/// I_arm = 0.117 (the config's, a little above the uniform tube's 0.116667). The roots turning at 1 rad/s with the
/// elbows still (delta' = -1): the tube 0.117 - 2 * 0.284413 * 0.35 + 0.35 / 4 = 5.4109e-3 kg m^2 per arm, so
/// 4 * 5.4109e-3 / 2 = 0.0108218 J and nothing else moves. Turning whole (delta' = 0) the arms are the rigid model's: the
/// same kinetic energy as without bending, to the bend's 1e-5 rad in the pose (9.4e-7 of it measured).
#[test]
fn the_tubes_kinetic_energy_is_the_cantilever_modes() {
    let bent = h4(&["effects.arm_bending = true", "effects.arm_tube_inertia = true"]);
    let (mut s, mut torque) = (Vec::new(), [0.0; 4]);
    bent.equilibrium(&[0.1, -0.05, -0.9, 0.0], &mut s, &mut torque);
    let at = bent.bend_at();
    for k in 0..4 {
        s[4 + k] = 1.0;
        s[at + 4 + k] = -1.0;
    }
    close(bent.energy_parts(&s)[0], 0.0108218, 1e-5, "the roots turning, the elbows still");
    let rigid = h4(&[]);
    let (mut r, mut rt) = (Vec::new(), [0.0; 4]);
    rigid.equilibrium(&[0.1, -0.05, -0.9, 0.0], &mut r, &mut rt);
    bent.equilibrium(&[0.1, -0.05, -0.9, 0.0], &mut s, &mut torque);
    for (m, state) in [(&rigid, &mut r), (&bent, &mut s)] {
        m.set_tool_velocity(state, &[0.5, 0.2, -0.3, 0.0]);
        for k in 12..16 {
            state[k] = 0.0;
        }
    }
    let (whole, split) = (rigid.energy_parts(&r)[0], bent.energy_parts(&s)[0]);
    assert!((whole - split).abs() < 1e-5 * whole, "{whole} J rigid, {split} J bending");
    // An arm lighter in inertia than a uniform tube of its mass: the tube is 3 I / L^2 = 0.96 kg of it, the rest a hub
    // on the axis, and the root keeps 0.08 - 0.25 * 0.96 * 0.25 - 2 * 0.034413 * 0.96 * 0.25 = 3.4818e-3 kg m^2.
    let light =
        h4(&["effects.arm_bending = true", "effects.arm_tube_inertia = true", "mass.upper_arm_inertia_kgm2 = 0.08"]);
    close(light.root_inertia, 3.4818e-3, 1e-4, "a hub on the axis");
    close(light.tube_mass, 0.96, 1e-12, "the tube mass");
}
