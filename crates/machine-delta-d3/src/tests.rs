//! The delta against hand calculations: kinematics, holding torques, an inertia torque, the Adept cycle's time.

use super::*;
use sim_components::{Command, Sim};

fn delta(edits: &[&str]) -> Delta {
    let mut cfg = ENTRY.config();
    for e in edits {
        cfg.set_text(e).unwrap();
    }
    Delta::new(&cfg).unwrap()
}

fn close(a: f64, b: f64, tolerance: f64, what: &str) {
    assert!((a - b).abs() <= tolerance * b.abs().max(1e-12), "{what}: {a} against {b} by hand");
}

/// Forward of inverse is the pose to 1e-12 m over a grid of the working range, and the Jacobian is the derivative of
/// the forward kinematics, by central differences of 1e-6 rad, to 1e-7 m/rad.
#[test]
fn the_kinematics_round_trip_and_the_jacobian_is_the_derivative_of_the_forward_kinematics() {
    let d = delta(&[]);
    let mut tried = 0;
    for i in 0..=12 {
        for j in 0..=12 {
            for k in 0..=8 {
                let p = [-0.6 + 0.1 * i as f64, -0.6 + 0.1 * j as f64, -1.061 + 0.386 * k as f64 / 8.0];
                if d.unreachable(&p, 0.0).is_some() {
                    continue;
                }
                tried += 1;
                let t = d.legs.inverse(p).unwrap();
                let back = d.legs.forward(t).unwrap();
                for a in 0..3 {
                    assert!((back[a] - p[a]).abs() < 1e-12, "{p:?} back as {back:?}");
                }
                let jac = d.legs.jacobian(t, p).unwrap();
                let h = 1e-6;
                for c in 0..3 {
                    let (mut up, mut down) = (t, t);
                    up[c] += h;
                    down[c] -= h;
                    let (pu, pd) = (d.legs.forward(up).unwrap(), d.legs.forward(down).unwrap());
                    for r in 0..3 {
                        assert!((jac[r][c] - (pu[r] - pd[r]) / (2.0 * h)).abs() < 1e-7, "{p:?}: J[{r}][{c}]");
                    }
                }
            }
        }
    }
    assert!(tried > 500, "{tried} poses");
}

/// The arm angle at the centre of the base, from the leg's own equation solved by halving:
/// (e + L_a cos t)^2 + (z + L_a sin t)^2 = L_b^2, and the platform's height per arm angle when all three turn together:
/// differentiating that, dz/dt = -L_a cos t + (e + L_a cos t) L_a sin t / (z + L_a sin t), a third of it per arm.
fn centre_by_hand(d: &Delta, z: f64) -> (f64, f64) {
    let (e, la, lb) = (d.legs.e, d.legs.la, d.legs.lb);
    let f = |t: f64| (e + la * t.cos()).powi(2) + (z + la * t.sin()).powi(2) - lb * lb;
    // The elbow-out root lies between -60 and 90 degrees, where f rises with t at this height.
    let (mut lo, mut hi) = (-60f64.to_radians(), 90f64.to_radians());
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        if f(lo) * f(mid) <= 0.0 { hi = mid } else { lo = mid }
    }
    let t = 0.5 * (lo + hi);
    let dz = -la * t.cos() + (e + la * t.cos()) * la * t.sin() / (z + la * t.sin());
    (t, dz / 3.0)
}

/// Static hold at the centre, 100 mm below the top, the forearms lumped (effects.forearm_inertia off): the platform's
/// weight shares out evenly. By hand (default config): t = 0.0577 degrees, dz/dt per arm = -0.13343 m/rad,
/// m_nt = 1.2 + 3 + 3 (1/3) 0.5 = 4.7 kg, m_LG = 0.9 * 0.2 + 0.4 (0.3 + 2/3 0.5) = 0.4333 kg m. Each gearbox holds -(g m_LG cos t) + m_nt g dz/dt
/// = -4.249 - 6.150 = -10.40 N m (negative: the arm is held up), the motor that over 20: -0.520 N m.
#[test]
fn the_holding_torque_at_the_centre_is_the_platforms_weight_shared_out_and_the_arms_own() {
    let d = delta(&["effects.torque_ripple = false", "effects.forearm_inertia = false"]);
    let z = -0.775;
    let (t, dz) = centre_by_hand(&d, z);
    let (m_nt, m_lg) = (1.2 + 3.0 + 0.5 / 3.0 * 3.0, 0.9 * 0.2 + 0.4 * (0.3 + 0.6667 * 0.5));
    close(m_nt, d.m_nt(), 1e-3, "m_nt");
    let by_hand = -G * m_lg * t.cos() + m_nt * G * dz;
    println!(
        "centre: t = {:.4} deg, dz/dt per arm {dz:.5} m/rad, holding {by_hand:.4} N m at each gearbox",
        t.to_degrees()
    );
    let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
    d.equilibrium(&[0.0, 0.0, z], &mut state, &mut torque);
    for k in 0..3 {
        close(state[k], t, 1e-9, "arm angle");
        close(torque[k], by_hand / 20.0, 1e-4, "motor torque");
    }
    assert!((by_hand + 10.40).abs() < 0.01, "{by_hand} N m");
}

/// Static hold at the edge, x = 0.6 m: by the Jacobian's transpose with the Jacobian from central differences of the
/// forward kinematics (not the machine's own), tau_k = -g m_LG cos t_k + m_nt g dz/dt_k, at each gearbox. The arm
/// pointing at the edge carries most: -22.2 N m against -1.35 N m on the other two (by hand with the script of the
/// research note).
#[test]
fn the_holding_torque_at_the_edge_is_the_weight_through_the_jacobians_transpose() {
    let d = delta(&["effects.torque_ripple = false", "effects.forearm_inertia = false"]);
    let p = [0.6, 0.0, -0.775];
    let t = d.legs.inverse(p).unwrap();
    let m_lg = 0.9 * 0.2 + 0.4 * (0.3 + 0.6667 * 0.5);
    let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
    d.equilibrium(&p, &mut state, &mut torque);
    let mut by_hand = [0.0; 3];
    for k in 0..3 {
        let (mut up, mut down) = (t, t);
        up[k] += 1e-6;
        down[k] -= 1e-6;
        let dz = (d.legs.forward(up).unwrap()[2] - d.legs.forward(down).unwrap()[2]) / 2e-6;
        by_hand[k] = -G * m_lg * t[k].cos() + d.m_nt() * G * dz;
        close(torque[k] * 20.0, by_hand[k], 1e-6, "gearbox torque");
    }
    println!("edge: {by_hand:?} N m");
    assert!(
        (by_hand[0] + 22.2).abs() < 0.1 && (by_hand[1] + 1.35).abs() < 0.05 && (by_hand[1] - by_hand[2]).abs() < 1e-6
    );
    // The rigid model held there stays put; so does the elastic one (the standard suite checks the middle).
    for elastic in [false, true] {
        let d = delta(&[&format!("effects.joint_elasticity = {elastic}")]);
        let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
        d.equilibrium(&p, &mut state, &mut torque);
        let mut rates = vec![0.0; d.state_len()];
        d.derivatives(&state, &torque, &mut rates);
        assert!(rates.iter().all(|r| r.abs() < 1e-6), "elastic {elastic}: {rates:?}");
    }
}

/// A vertical acceleration of 10 m/s^2 at the centre: the arms turn together, t'' = a / (3 dz/dt per arm) =
/// -24.98 rad/s^2. At the arm I_b t'' + m_nt a dz/dt = 0.1493 (-24.98) + 4.7 * 10 (-0.13343) = -3.730 - 6.271 =
/// -10.00 N m, at the motor over 20 plus the rotor's own (1.7 + 0.406) 1e-4 * 20 * t'' = -0.1052: -0.6053 N m. The
/// feedforward says so, and the rigid model given that torque accelerates the platform at 10 m/s^2 with no sideways part.
#[test]
fn a_vertical_acceleration_at_the_centre_takes_the_hand_calculated_inertia_torque() {
    let d = delta(&[
        "effects.joint_friction = false",
        "effects.motor_friction = false",
        "effects.torque_ripple = false",
        "effects.gearbox_efficiency = false",
        "effects.joint_elasticity = false",
        "effects.forearm_inertia = false",
    ]);
    let z = -0.775;
    let (_, dz) = centre_by_hand(&d, z);
    let i_b = 0.048 + 0.16 * (0.3 + 0.6667 * 0.5);
    close(i_b, d.arm_inertia(), 1e-12, "I_b");
    let acc = 10.0 / (3.0 * dz);
    let at_arm = i_b * acc + d.m_nt() * 10.0 * dz;
    let at_motor = at_arm / 20.0 + (1.7e-4 + 4.06e-5) * 20.0 * acc;
    println!("t'' = {acc:.3} rad/s^2, {at_arm:.4} N m at the arm, {at_motor:.4} N m at the motor");
    let (mut moving, mut still) = ([0.0; 3], [0.0; 3]);
    d.feedforward_torque(&[0.0, 0.0, z], &[0.0, 0.0, 10.0], &mut moving);
    d.feedforward_torque(&[0.0, 0.0, z], &[0.0; 3], &mut still);
    for k in 0..3 {
        close(moving[k] - still[k], at_motor, 1e-6, "inertia torque");
    }
    assert!((at_motor + 0.6053).abs() < 1e-3, "{at_motor}");
    let (mut state, mut hold) = (Vec::new(), [0.0; 3]);
    d.equilibrium(&[0.0, 0.0, z], &mut state, &mut hold);
    let mut rates = vec![0.0; d.state_len()];
    d.derivatives(&state, &moving, &mut rates);
    let (_, j) = d.pose([state[0], state[1], state[2]]);
    let platform = mul(&j, [rates[3], rates[4], rates[5]]);
    assert!(platform[0].abs() < 1e-9 && platform[1].abs() < 1e-9 && (platform[2] - 10.0).abs() < 1e-9, "{platform:?}");
}

/// Targets the delta cannot go to are refused with the reason: outside the cylinder, below its spherical bottom, near
/// the edge's bottom where the cylinder still has room but the sphere has not, in reach but beyond an arm's stop.
#[test]
fn targets_outside_the_working_range_are_refused_with_their_reason() {
    let mut sim = Sim::new(&ENTRY, &ENTRY.config()).unwrap();
    let refused = |sim: &mut Sim, p: [f64; 3]| sim.command(Command::MoveTo(p.to_vec())).err().map(|e| e.key);
    assert_eq!(refused(&mut sim, [0.45, 0.45, -0.75]), Some("error-delta-outside-envelope"));
    // At the edge the bottom is 200 mm below the top, at the centre 386 mm.
    assert_eq!(refused(&mut sim, [0.6, 0.0, -0.9]), Some("error-delta-outside-envelope"));
    assert_eq!(refused(&mut sim, [0.0, 0.0, -0.7]), None);
    let d = delta(&["joint.max_angle_deg = 60.0"]);
    assert_eq!(d.unreachable(&[0.6, 0.0, -0.87], 0.0), Some("error-delta-joint-stop"));
    let d = delta(&["geometry.forearm_m = 0.6"]);
    assert_eq!(d.unreachable(&[0.0, 0.0, -1.05], 0.0), Some("error-delta-out-of-reach"));
    let d = delta(&["workspace.max_condition_number = 1.5"]);
    assert_eq!(d.unreachable(&[0.5, 0.0, -0.8], 0.0), Some("error-delta-singularity"));
}

/// The Adept cycle's time against the S-curve by hand, at 3 m/s, 50 m/s^2 and 2500 m/s^3 along each axis.
/// The 305 mm traverse reaches cruise: t = d/v + v/a + a/j = 0.10167 + 0.06 + 0.02 = 0.18167 s. The 25 mm lifts
/// do not even reach the acceleration (d < 2 j (a/j)^3 = 40 mm): t = 4 (d / (2 j))^(1/3) = 4 * 0.017100 = 0.068399 s.
/// A pass: 4 lifts, 2 traverses, 2 gripper dwells of 35 ms = 0.27360 + 0.36333 + 0.07 = 0.70693 s.
///
/// The controller runs on its 1 ms profile cycle: a move arrives on the first cycle at or after its end (69 and 182
/// cycles), a dwell of n cycles hands over to the next segment n cycles later and that one starts a cycle after,
/// so a segment without a dwell stands still for 2 cycles at its corner. 4 * 69 + 2 * 182 + 4 * 2 + 2 * (35 + 1) =
/// 720 cycles: 0.720 s exactly, 13 ms more than the continuous figure.
///
/// The vendors' figures for 25/305/25 (ABB IRB 360-3/1130: 0.40 s at 1 kg, 0.52 s at 3 kg; Omron's 0.30 to 0.43 s for
/// the Quattro) are for rounded gate paths, without stops at the corners, and higher accelerations. They are
/// printed beside this, not asserted: the geometry here is an estimate and the cycle stops at each corner.
#[test]
fn the_adept_cycle_takes_the_hand_calculated_time() {
    let (v, a, j): (f64, f64, f64) = (3.0, 50.0, 2500.0);
    let traverse = 0.305 / v + v / a + a / j;
    let lift = 4.0 * (0.025 / (2.0 * j)).cbrt();
    let by_hand = 4.0 * lift + 2.0 * traverse + 2.0 * 0.035;
    let mut sim = Sim::new(&ENTRY, &ENTRY.config()).unwrap();
    let load = sim_components::analysis::analyse_cycle_with(&mut sim, false).unwrap();
    println!(
        "Adept cycle 25/305/25: {:.4} s simulated, {by_hand:.4} s by hand (traverse {traverse:.5} s, lift {lift:.5} s); ABB IRB 360-3/1130: 0.40 s at 1 kg, 0.52 s at 3 kg",
        load.duration
    );
    let cycles = |t: f64| (t / 1e-3 - 1e-9).ceil();
    let ticked = 4.0 * (cycles(lift) + 2.0) + 2.0 * (cycles(traverse) + 2.0) + 2.0 * (35.0 + 1.0 - 2.0);
    assert!((load.duration - ticked * 1e-3).abs() < 1e-9 && ticked == 720.0, "{} s, {ticked} cycles", load.duration);
    assert!((by_hand - 0.70693).abs() < 1e-4);
}

/// The singularity check: the cycle's highest condition number against the rating, from the signal.
#[test]
fn the_cycle_is_judged_against_the_condition_number_limit() {
    let cfg = ENTRY.config();
    let load = sim_components::analysis::analyse_cycle_with(&mut Sim::new(&ENTRY, &cfg).unwrap(), false).unwrap();
    let peak = load.condition_peak.unwrap();
    let verdicts = sim_components::analysis::verdicts(&cfg, &load).unwrap();
    let v = verdicts.iter().find(|v| v.check == "singularity").unwrap();
    assert!(v.ok && v.value == peak && v.limit == 5.0 && peak > 1.5 && peak < 2.5, "{v:?}");
    let mut tight = cfg.clone();
    tight.set_text("ratings.max_condition_number = 1.5").unwrap();
    assert!(
        !sim_components::analysis::verdicts(&tight, &load)
            .unwrap()
            .iter()
            .find(|v| v.check == "singularity")
            .unwrap()
            .ok
    );
}

/// The condition number of a matrix whose singular values are known: diag(1, 2, 4) turned has 4.
#[test]
fn the_condition_number_is_the_ratio_of_the_singular_values() {
    let (c, s) = (0.6f64, 0.8f64);
    let turn = [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]];
    let m: M3 = std::array::from_fn(|r| std::array::from_fn(|k| turn[r][k] * [1.0, 2.0, 4.0][k]));
    close(condition_number(&m), 4.0, 1e-12, "condition");
    close(condition_number(&[[2.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]]), 1.0, 1e-12, "identity");
}

/// Held by constant torques (the holding torques of the pose), the stiffness of the potential gravity less torque
/// times angle, by second differences: in the symmetric mode (all arms together, the platform up and down) it is
/// a + 2 b of the matrix [[a, b, b], [b, a, b], [b, b, a]]. At the middle of the default range, 868 mm down: a = 9.91,
/// b = -5.79 N m/rad, so -1.68 N m/rad, unstable; 1000 mm down a = 14.90, b = -6.13, +2.64 N m/rad, stable. The
/// sideways modes (a - b) are stable at both.
#[test]
fn the_vertical_mode_under_constant_torques_is_unstable_high_up() {
    let mut cfg = lossless_config();
    cfg.set_text("effects.joint_elasticity = false").unwrap();
    cfg.set_text("effects.forearm_inertia = false").unwrap();
    let m = Delta::new(&cfg).unwrap();
    let stiffness = |z: f64| {
        let (mut s, mut hold) = (Vec::new(), [0.0; 3]);
        m.equilibrium(&[0.0, 0.0, z], &mut s, &mut hold);
        let tau: Vec<f64> = hold.iter().map(|t| t * 20.0).collect();
        let at = |t: V3| {
            let mut st = s.clone();
            st[..3].copy_from_slice(&t);
            m.energy(&st) - (0..3).map(|k| tau[k] * t[k]).sum::<f64>()
        };
        let (t0, h) = ([s[0], s[1], s[2]], 1e-4);
        let second = |i: usize, j: usize| {
            let f = |a: f64, b: f64| {
                let mut t = t0;
                t[i] += a;
                t[j] += b;
                at(t)
            };
            (f(h, h) - f(h, -h) - f(-h, h) + f(-h, -h)) / (4.0 * h * h)
        };
        (second(0, 0), second(0, 1))
    };
    let ((a, b), (c, d)) = (stiffness(-0.868), stiffness(-1.0));
    println!("868 mm: a {a:.3}, b {b:.3}; 1000 mm: a {c:.3}, b {d:.3}");
    assert!(a + 2.0 * b < -1.5 && a - b > 0.0 && c + 2.0 * d > 2.0 && c - d > 0.0);
}

/// The feedforward with the speed is the rigid, loss-free model's torque: given it at a pose where the platform moves
/// at (2, -1, 0.5) m/s, the model accelerates the platform at the asked (10, 5, -20) m/s^2, the velocity term
/// m_nt J^T J_x^-1 w included (to 1e-9 m/s^2). Without the speed (the old feedforward) it misses by that term's
/// effect, here more than 1 m/s^2.
#[test]
fn the_feedforward_with_the_speed_gives_the_asked_acceleration() {
    let d = delta(&[
        "effects.joint_friction = false",
        "effects.motor_friction = false",
        "effects.torque_ripple = false",
        "effects.gearbox_efficiency = false",
        "effects.joint_elasticity = false",
        "effects.gravity = true",
    ]);
    let (p, v, a) = ([0.2, -0.1, -0.85], [2.0, -1.0, 0.5], [10.0, 5.0, -20.0]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 3]);
    d.equilibrium(&p, &mut state, &mut hold);
    d.set_tool_velocity(&mut state, &v);
    let platform = |torque: &[f64]| {
        let mut rates = vec![0.0; d.state_len()];
        d.derivatives(&state, torque, &mut rates);
        let t = [state[0], state[1], state[2]];
        let (_, j) = d.pose(t);
        let w = [state[3], state[4], state[5]];
        let bend = d.legs.velocity_term(t, p, w, v).unwrap();
        let along = mul(&j, [rates[3], rates[4], rates[5]]);
        [0, 1, 2].map(|i| along[i] + bend[i])
    };
    let (mut moving, mut still) = ([0.0; 3], [0.0; 3]);
    d.feedforward_torque_moving(&p, &v, &a, &mut moving);
    d.feedforward_torque(&p, &a, &mut still);
    let (with, without) = (platform(&moving), platform(&still));
    assert!((0..3).all(|i| (with[i] - a[i]).abs() < 1e-9), "{with:?}");
    assert!((0..3).map(|i| (without[i] - a[i]).powi(2)).sum::<f64>().sqrt() > 1.0, "{without:?}");
    // At rest the two are the same.
    d.feedforward_torque_moving(&p, &[0.0; 3], &a, &mut moving);
    assert!((0..3).all(|k| (moving[k] - still[k]).abs() < 1e-12));
}

/// The setpoint at rest is where the motors stand; the platform's weight twists the gearboxes and sits them at a side
/// of their backlash, so the tool is off it. `setpoint_slack` (`rest_sag`: twist and half the backlash either way at
/// every arm, over the range) covers what the simulation shows after 0.5 s at rest at five poses, the rim's lower edge
/// between two arms among them (2.09 mm there with the catalogue's 9 arcmin, the most found over the rim in steps of
/// 30 degrees), and is not far above it; without backlash and with stiff gearboxes it is the default 0.1 mm.
#[test]
fn the_setpoint_slack_covers_the_sag_at_rest() {
    let slack = delta(&[]).setpoint_slack();
    let mut most = 0f64;
    for pose in [[-0.6, 0.0, -0.7], [0.0, 0.0, -0.9], [0.3, 0.4, -0.8], [0.0, -0.55, -0.75], [0.29995, 0.51953, -0.874]]
    {
        let mut cfg = ENTRY.config();
        cfg.set_text(&format!("start.pose = {pose:?}")).unwrap();
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        sim.run(0.5, |_| {});
        let mut out = Vec::new();
        sim.sample(&mut out);
        let at = |id: &str| out[sim.signal_index(id).unwrap()];
        let off =
            ["x", "y", "z"].map(|a| at(&format!("axis.{a}.position_setpoint")) - at(&format!("axis.{a}.position")));
        let off = norm(off);
        println!("{pose:?}: setpoint {:.3} mm off the tool", off * 1e3);
        most = most.max(off);
    }
    println!("slack {:.3} mm", slack * 1e3);
    assert!(most <= slack && slack <= 1.4 * most, "{most} m at rest, slack {slack} m");
    assert_eq!(delta(&["effects.gearbox_backlash = false", "gearbox.stiffness_nm_rad = 1e12"]).setpoint_slack(), 1e-4);
}

/// A first arc from rest is laid out from the setpoint, which the sag puts 0.6 mm from the start pose: its end may be
/// off the circle through there by the machine's slack, not the gantries' 0.1 mm (refused before as "cannot be laid
/// out"). An end further off than the slack is still refused.
#[test]
fn a_first_arc_from_rest_is_laid_out_with_the_machines_slack() {
    let sim = Sim::new(&ENTRY, &ENTRY.config()).unwrap();
    let arc = |end: f64| sim_components::path::parse(&format!("arc,{end},0.0,-0.85,0.1525,0.0,-0.85,cw")).unwrap();
    assert!(sim.plan_path(&arc(0.2525), false).is_ok(), "{:?}", sim.plan_path(&arc(0.2525), false).err());
    let slack = sim.machine.setpoint_slack();
    assert_eq!(sim.plan_path(&arc(0.2525 + 2.0 * slack), false).err().map(|e| e.key), Some("error-path"));
}

/// The slack is for where a path starts, not for where it goes: a line to x = 0.6014, 1.4 mm beyond the range's
/// 0.6 m, is refused as the move there is; one to 0.6 runs. A trajectory too.
#[test]
fn a_path_may_not_use_the_setpoint_slack_away_from_its_start() {
    let mut sim = Sim::new(&ENTRY, &ENTRY.config()).unwrap();
    let line = |x: f64| sim_components::path::parse(&format!("line,{x},0.0,-0.775")).unwrap();
    assert_eq!(
        sim.command(Command::MoveTo(vec![0.6014, 0.0, -0.775])).err().map(|e| e.key),
        Some("error-target-out-of-range")
    );
    for time_optimal in [false, true] {
        assert_eq!(sim.plan_path(&line(0.6014), time_optimal).err().map(|e| e.key), Some("error-path-range"));
        assert!(sim.plan_path(&line(0.6), time_optimal).is_ok());
    }
    let mut points = sim.plan_path(&line(0.6), false).unwrap();
    points.last_mut().unwrap().1[0] = 0.6014;
    assert_eq!(sim.command(Command::Trajectory(points)).err().map(|e| e.key), Some("error-trajectory"));
}

/// An arc along the rim of the range (r = 0.6 m) samples points a hair beyond it by rounding; both planners lay it
/// out (refused as leaving the range with no allowance for rounding, while r = 0.5999999999 ran).
#[test]
fn an_arc_along_the_rim_of_the_range_is_planned() {
    let sim = Sim::new(&ENTRY, &ENTRY.config()).unwrap();
    let rim = sim_components::path::parse("line,0.6,0.0,-0.775\narc,0.0,0.6,-0.775,0.0,0.0,-0.775,ccw").unwrap();
    for time_optimal in [false, true] {
        assert!(
            sim.plan_path(&rim, time_optimal).is_ok(),
            "{time_optimal}: {:?}",
            sim.plan_path(&rim, time_optimal).err()
        );
    }
}

/// The rigid, loss-free model with or without the forearm rods (effects.forearm_inertia).
fn loss_free(rods: bool, more: &[&str]) -> Delta {
    let rods = format!("effects.forearm_inertia = {rods}");
    let mut edits = vec![
        "effects.joint_friction = false",
        "effects.motor_friction = false",
        "effects.torque_ripple = false",
        "effects.gearbox_efficiency = false",
        "effects.joint_elasticity = false",
        &rods,
    ];
    edits.extend(more);
    delta(&edits)
}

/// The height at the centre where the arms are horizontal: the elbows at e + L_a = 0.55 m from the axis, so
/// z = -sqrt(0.95^2 - 0.55^2) = -sqrt(0.6).
const LEVEL: f64 = -0.774_596_669_241_483;

/// The arms horizontal at the centre, a vertical acceleration of 10 m/s^2: the elbows move straight down with the
/// platform (dc/dt = (0, 0, -L_a), and u_i . (p' - c' t') = 0 with u_i = (-0.55 cos phi_i, -0.55 sin phi_i, z) gives
/// p'_z = -L_a t'), so every rod translates and counts with all its mass, in either model: t'' = -a / L_a = -25 rad/s^2
/// and per arm, at the arm, (I_arm + L_a^2 m_elbow) t'' - L_a m_fb a - (L_a / 3)(m_platform + m_payload) a
/// = -0.096 * 25 - 0.4 * 0.5 * 10 - 0.13333 * 4.2 * 10 = -2.4 - 2.0 - 5.6 = -10.0 N m; at the motor over 20 plus
/// the rotor's (1.7e-4 + 4.06e-5) * 20 * (-25): -0.6053 N m. The model given that torque accelerates the platform at
/// 10 m/s^2 straight up.
#[test]
fn with_the_arms_horizontal_a_vertical_acceleration_takes_the_hand_calculated_torque_with_rods() {
    let by_hand: f64 = (-(0.048 + 0.16 * 0.3) * 25.0 - 0.4 * 0.5 * 10.0 - 0.4 / 3.0 * 4.2 * 10.0) / 20.0
        + (1.7e-4 + 4.06e-5) * 20.0 * -25.0;
    assert!((by_hand + 0.6053).abs() < 1e-4, "{by_hand}");
    for rods in [true, false] {
        let d = loss_free(rods, &[]);
        let p = [0.0, 0.0, LEVEL];
        let t = d.legs.inverse(p).unwrap();
        assert!(t.iter().all(|a| a.abs() < 1e-7), "{t:?}");
        let (mut moving, mut still) = ([0.0; 3], [0.0; 3]);
        d.feedforward_torque(&p, &[0.0, 0.0, 10.0], &mut moving);
        d.feedforward_torque(&p, &[0.0; 3], &mut still);
        for k in 0..3 {
            close(moving[k] - still[k], by_hand, 1e-6, "inertia torque");
        }
        let (mut state, mut hold) = (Vec::new(), [0.0; 3]);
        d.equilibrium(&p, &mut state, &mut hold);
        let mut rates = vec![0.0; d.state_len()];
        d.derivatives(&state, &moving, &mut rates);
        let platform = mul(&d.pose(t).1, [rates[3], rates[4], rates[5]]);
        assert!(
            platform[0].abs() < 1e-9 && platform[1].abs() < 1e-9 && (platform[2] - 10.0).abs() < 1e-9,
            "rods {rods}: {platform:?}"
        );
    }
}

/// Kinetic energy at the same pose, the platform moving sideways at 1 m/s: there the rods turn and the two models part.
/// By hand: t'_i = u_i . p' / (u_i . dc_i/dt) = (-0.55 cos phi_i) / (-0.4 z) = 0.55 cos phi_i / (0.4 z), the elbows
/// moving straight up or down at 0.4 t'_i; sum t'^2 = 1.5 (0.55 / (0.4 * 0.7746))^2 = 4.7265 rad^2/s^2, sum of the
/// elbows' speeds squared 0.16 * 4.7265 = 0.75624 m^2/s^2. A slender rod has (m/6)(|c'|^2 + c'.p' + |p'|^2), here
/// c'.p' = 0: (0.5 / 6)(3 + 0.75624) = 0.31302 J for the three pairs; lumped, (0.5 / 2)(2/3 * 0.75624 + 1/3 * 3)
/// = 0.37604 J (0.6667 for 2/3). The rest is the same in both: arms (I_arm + L_a^2 m_elbow + i^2 J_rotor) sum t'^2 / 2
/// = (0.096 + 0.08424) * 4.7265 / 2 = 0.42596 J, the platform with its payload 4.2 / 2 = 2.1 J.
#[test]
fn the_rods_kinetic_energy_is_the_hand_calculated_one() {
    let sum_t2 = 1.5 * (0.55 / (0.4 * -LEVEL)).powi(2);
    let elbows = 0.16 * sum_t2;
    let common = 0.5 * (0.048 + 0.16 * 0.3 + 400.0 * (1.7e-4 + 4.06e-5)) * sum_t2 + 0.5 * 4.2;
    let rods = 0.5 / 6.0 * (3.0 + elbows);
    let lumped = 0.5 * 0.5 * (0.6667 * elbows + (1.0 - 0.6667) * 3.0);
    println!("rods {rods:.5} J, lumped {lumped:.5} J, the rest {common:.5} J");
    assert!((rods - 0.31302).abs() < 1e-4 && (lumped - 0.37604).abs() < 2e-4 && (common - 2.52596).abs() < 1e-4);
    for (on, by_hand) in [(true, rods), (false, lumped)] {
        let d = loss_free(on, &["effects.gravity = false"]);
        let (mut state, mut hold) = (Vec::new(), [0.0; 3]);
        d.equilibrium(&[0.0, 0.0, LEVEL], &mut state, &mut hold);
        d.set_tool_velocity(&mut state, &[1.0, 0.0, 0.0]);
        close(state[3] * state[3] + state[4] * state[4] + state[5] * state[5], sum_t2, 1e-6, "sum of t'^2");
        close(d.energy_parts(&state)[0], common + by_hand, 1e-6, &format!("kinetic energy, rods {on}"));
    }
}

/// Free and loss-free (no friction, damping, backlash or ripple), held by the constant motor torques of the pose and
/// started at (1, -0.5, 0.3) m/s, the energy less the motors' work, E - sum tau_k rotor_k, stays what it was: over 0.1 s
/// of RK4 at 10 us to 1e-7 of the kinetic energy at the start, with the rods, with bending arms (in and out of their
/// planes), with both, on elastic and rigid joints, and with the rods' spin inertia made 0.05 kg m^2 (some 4000 times
/// the config's, so that its share of the energy shows). A velocity term or a gravity that does not belong to the mass
/// matrix shows as a drift.
#[test]
fn a_free_loss_free_motion_keeps_its_energy() {
    for (rods, bending, elastic, spin) in [
        (true, false, true, 1.3e-5),
        (false, true, true, 1.3e-5),
        (true, true, true, 1.3e-5),
        (true, true, false, 1.3e-5),
        (true, false, false, 0.05),
        (true, true, true, 0.05),
    ] {
        let mut cfg = lossless_config();
        for e in [
            format!("effects.forearm_inertia = {rods}"),
            format!("effects.link_elasticity = {bending}"),
            format!("effects.joint_elasticity = {elastic}"),
            format!("mass.forearm_rod_spin_inertia_kgm2 = {spin:?}"),
            "effects.gearbox_backlash = false".into(),
        ] {
            cfg.set_text(&e).unwrap();
        }
        let d = Delta::new(&cfg).unwrap();
        let (mut s, mut torque) = (Vec::new(), [0.0; 3]);
        d.equilibrium(&[0.1, -0.05, -0.85], &mut s, &mut torque);
        d.set_tool_velocity(&mut s, &[1.0, -0.5, 0.3]);
        let budget = |s: &[f64]| {
            let (mut rotor, mut speed) = ([0.0; 3], [0.0; 3]);
            d.motor_state(s, &mut rotor, &mut speed);
            d.energy(s) - (0..3).map(|k| torque[k] * rotor[k]).sum::<f64>()
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
        let (mut at, mut vel) = ([0.0; 3], [0.0; 3]);
        d.tool_state(&s, &mut at, &mut vel);
        let travel = norm(sub(at, [0.1, -0.05, -0.85]));
        println!(
            "rods {rods} (spin {spin}), bending {bending}, elastic {elastic}: drift {drift:.3e} J of {kinetic:.4} J, \
             {travel:.3} m travelled"
        );
        assert!(travel > 0.05, "{travel} m");
        assert!(drift < 1e-7 * kinetic, "rods {rods}, bending {bending}, elastic {elastic}: {drift} J");
    }
}

/// The lowest mode with the motors held, the forearms lumped, at the centre with the arms horizontal. Sideways the
/// platform moves 1 m/s for sum t'^2 = 4.7265 rad^2/s^2 (as in `the_rods_kinetic_energy_is_the_hand_calculated_one`),
/// so a unit turn vector carries M_s = L_a^2 (m_elbow + s m_fb) + m_nt / 4.7265 = 0.16 * 0.63335 + 4.69995 / 4.7265
/// = 0.10134 + 0.99438 = 1.09572 kg m^2 beyond the arm's tube; up and down (all arms together, p'_z = -L_a t' / 3 each)
/// M_v = 0.10134 + m_nt L_a^2 / 3 = 0.35200. The heavier sideways mode is the lowest:
/// bending alone (rigid gearboxes): w^2 = k_b / M_s = 65000 / 1.09572, 38.76 Hz (vertical: 68.39 Hz);
/// gearboxes alone (rigid arms): w^2 = k_g / (I_arm + M_s) = 35752.6 / 1.14372, 28.14 Hz;
/// both, the tube I_arm = 0.048 between the two springs: the lower root of
/// I_arm M_s w^4 - (k_g M_s + k_b M_s + k_b I_arm) w^2 + k_g k_b = 0.
/// The mode's shape moves the platform sideways by 1 m, not up or down.
#[test]
fn the_lowest_mode_is_the_hand_calculated_one() {
    let (kb, kg, i_arm) = (65000.0, 35752.6, 0.048);
    let sum_t2 = 1.5 * (0.55 / (0.4 * -LEVEL)).powi(2);
    let m_nt = 1.2 + 3.0 + 3.0 * (1.0 - 0.6667) * 0.5;
    let tip = 0.16 * (0.3 + 0.6667 * 0.5);
    let (sideways, vertical) = (tip + m_nt / sum_t2, tip + m_nt * 0.16 / 3.0);
    assert!((sideways - 1.09572).abs() < 1e-4 && (vertical - 0.352).abs() < 1e-4);
    let hz = |w2: f64| w2.sqrt() / std::f64::consts::TAU;
    let (b, c) = (kg * sideways + kb * sideways + kb * i_arm, kg * kb);
    let both = (b - (b * b - 4.0 * i_arm * sideways * c).sqrt()) / (2.0 * i_arm * sideways);
    let cases = [
        ("bending", false, true, kb / sideways),
        ("gearboxes", true, false, kg / (i_arm + sideways)),
        ("both", true, true, both),
    ];
    for (what, elastic, bending, w2) in cases {
        let d = delta(&[
            "effects.forearm_inertia = false",
            &format!("effects.joint_elasticity = {elastic}"),
            &format!("effects.link_elasticity = {bending}"),
        ]);
        let found = d.resonance_hz(&[0.0, 0.0, LEVEL]).unwrap();
        println!("{what}: {found:.4} Hz, by hand {:.4} Hz", hz(w2));
        close(found, hz(w2), 1e-6, what);
    }
    assert!(
        (hz(kb / sideways) - 38.76).abs() < 0.01
            && (hz(kb / vertical) - 68.39).abs() < 0.01
            && (hz(kg / (i_arm + sideways)) - 28.14).abs() < 0.01
    );
    assert!(hz(both) < hz(kg / (i_arm + sideways)));
    assert_eq!(delta(&["effects.joint_elasticity = false"]).resonance_hz(&[0.0, 0.0, LEVEL]), None);
    let d = delta(&["effects.forearm_inertia = false", "effects.link_elasticity = true"]);
    let (_, shape) = d.mode_shape(&[0.0, 0.0, LEVEL]).unwrap();
    let (mut s, mut torque) = (Vec::new(), [0.0; 3]);
    d.equilibrium(&[0.0, 0.0, LEVEL], &mut s, &mut torque);
    let moved: Vec<f64> = s.iter().zip(&shape).map(|(a, b)| a + 1e-6 * b).collect();
    let (mut at, mut rest, mut vel) = ([0.0; 3], [0.0; 3], [0.0; 3]);
    d.tool_state(&moved, &mut at, &mut vel);
    d.tool_state(&s, &mut rest, &mut vel);
    let travel: Vec<f64> = (0..3).map(|i| (at[i] - rest[i]) / 1e-6).collect();
    assert!((travel[0].abs().max(travel[1].abs()) - 1.0).abs() < 1e-3 && travel[2].abs() < 1e-3, "{travel:?}");
}

/// The spin of a forearm pair's rods about their own axis, by hand. Leg 1 (along x, its elbow's axis a = (0, 1, 0)), the
/// platform at (0, 0.1, -0.8) moving at 1 m/s at right angles to the forearm u = p - c_1 in the plane y = 0.1,
/// p' = (u_z, 0, -u_x) / sqrt(u_x^2 + u_z^2): then u . p' = 0, so arm 1 stands still (theta_1' = 0, c_1' = 0) and
/// u' = p'. With sigma = a . u = 0.1, u x a = (-u_z, 0, u_x) and n = a . (u' x u) = (u x a) . u' = -sqrt(u_x^2 + u_z^2)
/// = -sqrt(L^2 - 0.01), the spin s = -sigma n / (L (L^2 - sigma^2)) = 0.1 / (L sqrt(L^2 - 0.01))
/// = 0.1 / (0.95 * 0.944722) = 0.111422 rad/s, whatever the arm's angle. Every leg's spin also from its pair's plane
/// turning, by central differences along the motion: the plane's direction across the rod, e2 = a - (a . e) e
/// normalised (e = u / L), turns at e2' . (e x e2) about the rod. The spin's kinetic energy with I = 0.01 kg m^2 a rod
/// (the config's 1.3e-5 would give some 1e-7 J): I s^2 / 2 for each of the six rods, I sum s_i^2, the kinetic energy
/// with that inertia less that without.
#[test]
fn the_rods_spin_is_the_hand_calculated_one() {
    let by_hand = 0.1 / (0.95 * (0.95f64 * 0.95 - 0.01).sqrt());
    assert!((by_hand - 0.111422).abs() < 1e-6, "{by_hand}");
    let spin = |inertia: f64| {
        loss_free(true, &["effects.gravity = false", &format!("mass.forearm_rod_spin_inertia_kgm2 = {inertia:?}")])
    };
    let (d, still) = (spin(0.01), spin(0.0));
    let p = [0.0, 0.1, -0.8];
    let t = d.legs.inverse(p).unwrap();
    let u = sub(p, d.legs.elbow(0, t[0])[0]);
    let pv = [u[2], 0.0, -u[0]].map(|v| v / u[0].hypot(u[2]));
    let w = mul(&inverse(&d.legs.jacobian(t, p).unwrap()).unwrap(), pv);
    assert!(w[0].abs() < 1e-12, "{w:?}");
    close(d.rod_spin(0, t, [0.0; 3], w, [0.0; 3]), by_hand, 1e-9, "leg 1's spin");
    let plane = |i: usize, t: V3| -> (V3, V3) {
        let e = sub(d.legs.forward(t).unwrap(), d.legs.elbow(i, t[i])[0]).map(|v| v / 0.95);
        let a = d.legs.tangent(i);
        let across = sub(a, e.map(|v| v * dot(a, e)));
        (e, across.map(|v| v / norm(across)))
    };
    let (h, mut sum) = (1e-6, 0.0);
    for i in 0..3 {
        let shifted = |sign: f64| plane(i, std::array::from_fn(|k| t[k] + sign * h * w[k])).1;
        let (e, across) = plane(i, t);
        let turning = dot(cross(e, across), sub(shifted(1.0), shifted(-1.0))) / (2.0 * h);
        let spin = d.rod_spin(i, t, [0.0; 3], w, [0.0; 3]);
        println!("leg {}: spin {spin:.6} rad/s, from the plane {turning:.6} rad/s", i + 1);
        close(spin, turning, 1e-6, &format!("leg {}'s spin", i + 1));
        sum += turning * turning;
    }
    let kinetic = |d: &Delta| {
        let (mut s, mut hold) = (Vec::new(), [0.0; 3]);
        d.equilibrium(&p, &mut s, &mut hold);
        d.set_tool_velocity(&mut s, &pv);
        d.energy_parts(&s)[0]
    };
    println!("spin energy {:.6e} J, by hand {:.6e} J", kinetic(&d) - kinetic(&still), 0.01 * sum);
    close(kinetic(&d) - kinetic(&still), 0.01 * sum, 1e-6, "the spin's kinetic energy");
}

/// The arms' bending out of their planes, by hand: at the centre with the arms horizontal every forearm is at right
/// angles to its elbow's axis (sigma = 0), so an elbow moving along its axis moves neither the platform nor the arm,
/// and rings on its own on the tube's lateral stiffness k_l = 400000 N/m with the mass at the elbow: lumped,
/// m_elbow + s m_fb = 0.3 + 0.6667 * 0.5 = 0.63335 kg, sqrt(400000 / 0.63335) / 2 pi = 126.48 Hz, three times. Damped
/// c_l / (2 sqrt(k_l m)) = 15 / (2 * 503.33) = 0.014901. With the rods the sixth of the pair's mass at the elbow and
/// the two thirds at its middle at half the elbow's speed, 0.3 + 0.5 / 6 + (2/3) 0.5 / 4 = 0.46667 kg, 147.35 Hz; but
/// the middle also moves with the platform, which moves sideways along the elbows' axes, so only the mode with all
/// three elbows moving the same way round (the axes summing to nought) keeps it; the other two couple to the sideways
/// modes and ring a little higher (within 0.5 %). All above the in-plane ones (lumped 38.76 Hz sideways twice and
/// 68.39 Hz vertical, as in `the_lowest_mode_is_the_hand_calculated_one`): 6 modes with the bending alone; 3 with the
/// gearboxes alone (one per arm), 9 with both.
#[test]
fn the_arms_bending_out_of_their_planes_rings_at_the_hand_calculated_frequency() {
    let hz = |k: f64, m: f64| (k / m).sqrt() / std::f64::consts::TAU;
    let (lumped, rods) = (0.3 + 0.6667 * 0.5, 0.3 + 0.5 / 6.0 + 2.0 / 3.0 * 0.5 / 4.0);
    assert!((hz(4e5, lumped) - 126.48).abs() < 0.01 && (hz(4e5, rods) - 147.35).abs() < 0.01);
    for (on, m) in [(false, lumped), (true, rods)] {
        let d = delta(&[
            "effects.joint_elasticity = false",
            "effects.link_elasticity = true",
            &format!("effects.forearm_inertia = {on}"),
        ]);
        let modes = d.modes(&[0.0, 0.0, LEVEL]);
        println!("rods {on}: {modes:?}");
        assert_eq!(modes.len(), 6);
        close(modes[3].hz, hz(4e5, m), 1e-6, "out of the plane");
        for mode in &modes[4..] {
            let by = if on { 0.005 } else { 1e-6 };
            assert!(mode.hz >= modes[3].hz * (1.0 - 1e-9) && mode.hz <= hz(4e5, m) * (1.0 + by), "{modes:?}");
        }
        if !on {
            assert!((modes[0].hz - 38.76).abs() < 0.01 && (modes[1].hz - 38.76).abs() < 0.01, "{modes:?}");
            assert!((modes[2].hz - 68.39).abs() < 0.01, "{modes:?}");
            close(modes[3].damping, 15.0 / (2.0 * (4e5 * lumped).sqrt()), 1e-6, "damping out of the plane");
        }
    }
    for (elastic, bending, count) in [(true, false, 3), (false, true, 6), (true, true, 9)] {
        let d =
            delta(&[&format!("effects.joint_elasticity = {elastic}"), &format!("effects.link_elasticity = {bending}")]);
        assert_eq!(d.modes(&[0.1, -0.05, -0.85]).len(), count, "gearboxes {elastic}, bending {bending}");
        assert_eq!(d.resonance_hz(&[0.1, -0.05, -0.85]), d.modes(&[0.1, -0.05, -0.85]).first().map(|m| m.hz));
    }
}

/// A box in the way of an upper arm and of nothing else: at the centre, 0.75 m down, arm 1's middle (on y = 0, the rods
/// of its pair 0.05 m to either side) is where the box is put (0.02 m across). At the start pose (0.1525, 0, -0.75) arm
/// 1 stands at another angle, its boxes some 6 mm from the obstacle; on the traverse to (-0.1525, 0, -0.75) it sweeps
/// through it. Planned, the check
/// finds the contact; run with effects.crash the drives trip. The platform's and the rods' boxes, the bodies until 14
/// November, never touch it.
#[test]
fn an_upper_arm_running_into_an_obstacle_is_a_crash() {
    let d = delta(&[]);
    let t = d.legs.inverse([0.0, 0.0, -0.75]).unwrap();
    let (shoulder, end) = d.arm(0, t, [0.0; 3]);
    let middle: V3 = std::array::from_fn(|i| 0.5 * (shoulder[i] + end[i]));
    println!("arm 1's middle at the centre: {middle:?}, arm angles {t:?}");
    let obstacle = format!(
        "obstacles = [{{ name = \"arm\", centre_m = [{:?}, {:?}, {:?}], size_m = [0.02, 0.02, 0.02] }}]",
        middle[0], middle[1], middle[2]
    );
    let mut cfg = ENTRY.config();
    cfg.set_text(&obstacle).unwrap();
    let start = cfg.f64s("start.pose").unwrap();
    let sim = Sim::new(&ENTRY, &cfg).unwrap();
    let bodies = sim.machine.collision_bodies(&sim.state);
    let rods = 2 + 6 * ROD_PIECES;
    assert_eq!(bodies.len(), rods + 3 * ARM_PIECES);
    let gap = |bodies: &[sim_core::Aabb]| sim_components::collision::nearest(bodies, &sim.obstacles).unwrap().1.2;
    println!("at the start {:.4} m, the rods and platform {:.4} m", gap(&bodies), gap(&bodies[..rods]));
    assert!(gap(&bodies) > 0.004 && gap(&bodies[..rods]) > 0.05, "at the start {} m", gap(&bodies));
    let planned = sim.check_plan(&[start, vec![-0.1525, 0.0, -0.75]]).unwrap();
    assert!(planned.clearance < 0.0 && planned.contact.is_some(), "{planned:?}");
    cfg.set_text("effects.crash = true").unwrap();
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    sim.command(Command::MoveTo(vec![-0.1525, 0.0, -0.75])).unwrap();
    sim.run(0.5, |_| {});
    let contact = sim.contact.as_ref().unwrap();
    println!("contact at {:?}, {:.3} m/s", contact.pose, contact.speed);
    assert!(sim.crashed && !sim.control.enabled() && contact.pose[0] > -0.1525 && contact.pose[0] < 0.1525);
    let bodies = sim.machine.collision_bodies(&sim.state);
    assert!(gap(&bodies[..rods]) > 0.0, "the rods and the platform {} m from it", gap(&bodies[..rods]));
}

/// A box in the way of a forearm and of nothing else: at the centre, 0.75 m down, leg 1's rod on the +y side has its
/// middle where the box is put (0.03 m across). From the start pose (0.1525, 0, -0.75) that middle is some 0.08 m
/// away in x; on the traverse to (-0.1525, 0, -0.75) the rod sweeps through it. Planned, the check finds the contact;
/// run with effects.crash the drives trip. The platform passes far below: the box's bottom lies more than 0.3 m above
/// the platform's top, so the platform's and payload's boxes alone (the bodies until 13 November) never touch it.
#[test]
fn a_forearm_running_into_an_obstacle_is_a_crash() {
    let d = delta(&[]);
    let p = [0.0, 0.0, -0.75];
    let t = d.legs.inverse(p).unwrap();
    let [_, (from, to)] = d.legs.rods(0, t[0], 0.0, p, 0.05, 0.1);
    let middle: V3 = std::array::from_fn(|i| 0.5 * (from[i] + to[i]));
    println!("rod middle at the centre: {middle:?}");
    assert!(middle[1] > 0.0 && middle[2] - 0.015 - (p[2] + 0.015) > 0.3, "{middle:?}");
    let obstacle = format!(
        "obstacles = [{{ name = \"rod\", centre_m = [{:?}, {:?}, {:?}], size_m = [0.03, 0.03, 0.03] }}]",
        middle[0], middle[1], middle[2]
    );
    let mut cfg = ENTRY.config();
    cfg.set_text(&obstacle).unwrap();
    let start = cfg.f64s("start.pose").unwrap();
    let sim = Sim::new(&ENTRY, &cfg).unwrap();
    let bodies = sim.machine.collision_bodies(&sim.state);
    assert_eq!(bodies.len(), 2 + 6 * ROD_PIECES + 3 * ARM_PIECES);
    let gap = |bodies: &[sim_core::Aabb]| sim_components::collision::nearest(bodies, &sim.obstacles).unwrap().1.2;
    assert!(gap(&bodies) > 0.0 && gap(&bodies[..2]) > 0.3, "at the start {} m", gap(&bodies));
    let planned = sim.check_plan(&[start, vec![-0.1525, 0.0, -0.75]]).unwrap();
    assert!(planned.clearance < 0.0 && planned.contact.is_some(), "{planned:?}");
    cfg.set_text("effects.crash = true").unwrap();
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    sim.command(Command::MoveTo(vec![-0.1525, 0.0, -0.75])).unwrap();
    sim.run(0.5, |_| {});
    let contact = sim.contact.as_ref().unwrap();
    println!("contact at {:?}, {:.3} m/s", contact.pose, contact.speed);
    assert!(sim.crashed && !sim.control.enabled() && contact.pose[0] > -0.1525 && contact.pose[0] < 0.1525);
    let bodies = sim.machine.collision_bodies(&sim.state);
    assert!(gap(&bodies[..2]) > 0.3, "the platform {} m from it", gap(&bodies[..2]));
}

/// Transition curves on the delta's three axes, with `profile.path_tolerance_m` = 0.1 mm: a line along x onto a circle
/// of 0.1 m around (0, 0, -0.85) in the plane tilted by 30 degrees about x (along x and along (0, cos 30, sin 30), in
/// no plane of two axes), a quarter of it, and 0.15 m on in its direction. By hand within the axis limits (3 m/s,
/// 50 m/s^2, 2500 m/s^3): 2 m/s on the arc (sqrt(0.8 a r)), whose entry would need a curve of v^3 dk / (0.4 j) = 80 mm;
/// the tolerance allows L = sqrt(24 e r) = 15.49 mm, and the arc moves in by e = L^2 / (24 r) = 0.1 mm, which is the
/// deviation reported. Both planners: every planned pose within the reported deviation (at most 0.1 mm) of the given
/// path, and the path takes less time than without the tolerance.
#[test]
fn a_path_in_a_tilted_plane_gets_its_transition_curves() {
    let (c, r, t) = ([0.0, 0.0, -0.85], 0.1, 30f64.to_radians());
    let (e1, e2) = ([1.0, 0.0, 0.0], [0.0, t.cos(), t.sin()]);
    let at = |a: f64, b: f64| [0, 1, 2].map(|i| c[i] + a * e1[i] + b * e2[i]);
    let (start, onto, off, end) = (at(-0.2, -r), at(0.0, -r), at(r, 0.0), at(r, 0.15));
    let text = format!(
        "line,{:?},{:?},{:?}\nline,{:?},{:?},{:?}\narc,{:?},{:?},{:?},{:?},{:?},{:?},ccw\nline,{:?},{:?},{:?}",
        start[0],
        start[1],
        start[2],
        onto[0],
        onto[1],
        onto[2],
        off[0],
        off[1],
        off[2],
        c[0],
        c[1],
        c[2],
        end[0],
        end[1],
        end[2]
    );
    let pieces = sim_components::path::parse(&text).unwrap();
    let mut cfg = ENTRY.config();
    cfg.set_text("profile.path_tolerance_m = 0.0001").unwrap();
    let (blended, given) = (Sim::new(&ENTRY, &cfg).unwrap(), Sim::new(&ENTRY, &ENTRY.config()).unwrap());
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let sub = |a: &[f64], b: [f64; 3]| [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
    let to_segment = |p: &[f64], a: [f64; 3], b: [f64; 3]| {
        let (ab, ap) = (sub(&b, a), sub(p, a));
        let s = (dot(ab, ap) / dot(ab, ab)).clamp(0.0, 1.0);
        dot(sub(&ap, ab.map(|x| s * x)), sub(&ap, ab.map(|x| s * x))).sqrt()
    };
    // From the arc: out of the plane, and in it from the circle, where the point is beside the quarter.
    let normal = [0.0, -t.sin(), t.cos()];
    let to_arc = |p: &[f64]| {
        let q = sub(p, c);
        let (h, a, b) = (dot(q, normal), dot(q, e1), dot(q, e2));
        if a >= 0.0 && b <= 0.0 { (h * h + ((a * a + b * b).sqrt() - r).powi(2)).sqrt() } else { f64::INFINITY }
    };
    for time_optimal in [false, true] {
        let points = blended.plan_path(&pieces, time_optimal).unwrap();
        let reported = blended.path_deviation(&pieces, time_optimal).unwrap();
        assert!(
            reported > 0.0 && reported <= 1e-4 * (1.0 + 1e-9),
            "time-optimal {time_optimal}: {reported} m reported"
        );
        if !time_optimal {
            close(reported, 1e-4, 1e-9, "e = L^2 / (24 r) within the axis limits");
        }
        let home: [f64; 3] = [points[0].1[0], points[0].1[1], points[0].1[2]];
        let off_path = |p: &[f64]| {
            to_segment(p, home, start).min(to_segment(p, start, onto)).min(to_arc(p)).min(to_segment(p, off, end))
        };
        let furthest = points.iter().map(|p| off_path(&p.1)).fold(0.0, f64::max);
        assert!(
            furthest <= reported + 1e-7 && furthest > 0.5 * reported,
            "time-optimal {time_optimal}: {furthest} m off the path, {reported} m reported"
        );
        let as_given = given.plan_path(&pieces, time_optimal).unwrap();
        let (time, before) = (points[points.len() - 1].0, as_given[as_given.len() - 1].0);
        assert!(
            time < before - 0.01
                && dot(sub(&points[points.len() - 1].1, end), sub(&points[points.len() - 1].1, end)).sqrt() < 1e-9,
            "time-optimal {time_optimal}: {time} s against {before} s"
        );
        println!(
            "time-optimal {time_optimal}: {time} s against {before} s, off by {furthest} m, {reported} m reported"
        );
    }
}

/// Limits that change within a piece: on the delta the motors turn faster per tool speed in one place than in another.
/// From the start pose a line to (0.5, 0, -0.85) and across the range to (-0.5, 0, -0.85), as fast as the motors allow.
/// Along the second line, by the inverse kinematics at 801 places (central differences of 1 µm), the rated speed lets
/// the tool go 7.270 m/s where the motors turn fastest per tool speed (at its end, x = -0.5) and 13.31 m/s where they
/// turn slowest. The plan with one limit per piece kept the whole line below the slowest place's speed and its
/// acceleration below the line's weakest pose, and took 0.4290 s (13 November, before it was replaced). Point by point
/// the plan reaches 8.12 m/s on the second line, more than 1.1 times the slowest place's speed (it passes that place
/// braking, slower), uses the motors' torque up to the reserve, and takes 0.412 s. Every planned pose
/// keeps each motor within `profile.torque_reserve` of its torque-speed curve (rigid model, with the centripetal and
/// Coriolis torque of the speed), within its rated speed, and each axis within its jerk limit, by differences over
/// one cycle (0.1 %), and the path ends on its end point.
#[test]
fn the_time_optimal_plan_takes_the_motor_limits_where_they_are_along_a_line() {
    let cfg = ENTRY.config();
    let sim = Sim::new(&ENTRY, &cfg).unwrap();
    let (from, to) = ([0.5, 0.0, -0.85], [-0.5, 0.0, -0.85]);
    let points =
        sim.plan_path(&sim_components::path::parse("line,0.5,0.0,-0.85\nline,-0.5,0.0,-0.85").unwrap(), true).unwrap();
    let (reserve, rated, c) =
        (cfg.f64("profile.torque_reserve").unwrap(), cfg.f64("drive.rated_speed_rad_s").unwrap(), 1e-3);
    // The tool speed along the second line at which the fastest motor turns at its rated speed, at its places.
    let length = (0..3).map(|i| (to[i] - from[i]) * (to[i] - from[i])).sum::<f64>().sqrt();
    let speed_at = |k: usize| {
        let at = |s: f64| (0..3).map(|i| from[i] + (to[i] - from[i]) * s / length).collect::<Vec<_>>();
        let (mut ahead, mut behind) = ([0.0; 3], [0.0; 3]);
        let s = length * k as f64 / 800.0;
        sim.machine.inverse(&at(s + 1e-6), &mut ahead);
        sim.machine.inverse(&at(s - 1e-6), &mut behind);
        rated / (0..3).map(|m| (ahead[m] - behind[m]).abs() / 2e-6).fold(0.0, f64::max)
    };
    let (slowest, fastest) = (0..=800).map(speed_at).fold((f64::INFINITY, 0f64), |(l, h), v| (l.min(v), h.max(v)));
    assert!((slowest - 7.270).abs() < 1e-3 && (fastest - 13.31).abs() < 0.01, "{slowest} to {fastest} m/s");
    // Speed, motor torque and speed, axis jerk by differences over the planned poses, at rest before and after.
    let pose = |i: isize| &points[i.clamp(0, points.len() as isize - 1) as usize].1;
    let off = |p: &[f64]| (0..3).map(|k| (p[k] - from[k]) * (p[k] - from[k])).sum::<f64>();
    let junction = (0..points.len()).min_by(|i, j| off(&points[*i].1).total_cmp(&off(&points[*j].1))).unwrap() as isize;
    let (mut quickest, mut used) = (0f64, [0f64; 3]);
    for i in 0..points.len() as isize {
        let (a, p, b, b2) = (pose(i - 1), pose(i), pose(i + 1), pose(i + 2));
        let vel: Vec<f64> = (0..3).map(|k| (b[k] - a[k]) / (2.0 * c)).collect();
        let acc: Vec<f64> = (0..3).map(|k| (b[k] - 2.0 * p[k] + a[k]) / (c * c)).collect();
        let (mut torque, mut ahead, mut here) = ([0.0; 3], [0.0; 3], [0.0; 3]);
        sim.machine.feedforward_torque_moving(p, &vel, &acc, &mut torque);
        sim.machine.inverse(b, &mut ahead);
        sim.machine.inverse(p, &mut here);
        if i > junction {
            quickest = quickest.max(vel.iter().map(|v| v * v).sum::<f64>().sqrt());
        }
        for k in 0..3 {
            let speed = (ahead[k] - here[k]) / c;
            let (low, high) = sim.control.torque_curve(speed);
            let jerk = (b2[k] - 3.0 * b[k] + 3.0 * p[k] - a[k]) / (c * c * c);
            used = [
                used[0].max(torque[k] / (reserve * if torque[k] >= 0.0 { high } else { low })),
                used[1].max(speed.abs() / rated),
                used[2].max(jerk.abs() / sim.machine.axes()[k].max_jerk),
            ];
        }
    }
    let (duration, last) = (points[points.len() - 1].0, &points[points.len() - 1].1);
    println!("{duration} s, {quickest} m/s on the second line (its slowest place allows {slowest} m/s), used {used:?}");
    assert!(used.iter().all(|u| *u <= 1.001), "torque, speed and jerk used to {used:?}");
    assert!(quickest > 1.1 * slowest, "{quickest} m/s, the slowest place of the line allows {slowest} m/s");
    assert!(duration < 0.4290 - 0.01, "{duration} s, with one limit per piece 0.4290 s");
    assert!((0..3).all(|k| (last[k] - to[k]).abs() < 1e-9), "ends at {last:?}");
}
