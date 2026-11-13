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

/// Static hold at the centre, 100 mm below the top: the platform's weight shares out evenly. By hand (default
/// config): t = 0.0577 degrees, dz/dt per arm = -0.13343 m/rad, m_nt = 1.2 + 3 + 3 (1/3) 0.5 = 4.7 kg, m_LG =
/// 0.9 * 0.2 + 0.4 (0.3 + 2/3 0.5) = 0.4333 kg m. Each gearbox holds -(g m_LG cos t) + m_nt g dz/dt
/// = -4.249 - 6.150 = -10.40 N m (negative: the arm is held up), the motor that over 20: -0.520 N m.
#[test]
fn the_holding_torque_at_the_centre_is_the_platforms_weight_shared_out_and_the_arms_own() {
    let d = delta(&["effects.torque_ripple = false"]);
    let z = -0.775;
    let (t, dz) = centre_by_hand(&d, z);
    let (m_nt, m_lg) = (1.2 + 3.0 + 0.5 / 3.0 * 3.0, 0.9 * 0.2 + 0.4 * (0.3 + 0.6667 * 0.5));
    close(m_nt, d.m_nt(), 1e-3, "m_nt");
    let by_hand = -G * m_lg * t.cos() + m_nt * G * dz;
    println!("centre: t = {:.4} deg, dz/dt per arm {dz:.5} m/rad, holding {by_hand:.4} N m at each gearbox", t.to_degrees());
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
    let d = delta(&["effects.torque_ripple = false"]);
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
    assert!((by_hand[0] + 22.2).abs() < 0.1 && (by_hand[1] + 1.35).abs() < 0.05 && (by_hand[1] - by_hand[2]).abs() < 1e-6);
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
    let d = delta(&["effects.joint_friction = false", "effects.motor_friction = false", "effects.torque_ripple = false", "effects.gearbox_efficiency = false", "effects.joint_elasticity = false"]);
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
    println!("Adept cycle 25/305/25: {:.4} s simulated, {by_hand:.4} s by hand (traverse {traverse:.5} s, lift {lift:.5} s); ABB IRB 360-3/1130: 0.40 s at 1 kg, 0.52 s at 3 kg", load.duration);
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
    assert!(!sim_components::analysis::verdicts(&tight, &load).unwrap().iter().find(|v| v.check == "singularity").unwrap().ok);
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
    let d = delta(&["effects.joint_friction = false", "effects.motor_friction = false", "effects.torque_ripple = false", "effects.gearbox_efficiency = false", "effects.joint_elasticity = false", "effects.gravity = true"]);
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
    for pose in [[-0.6, 0.0, -0.7], [0.0, 0.0, -0.9], [0.3, 0.4, -0.8], [0.0, -0.55, -0.75], [0.29995, 0.51953, -0.874]] {
        let mut cfg = ENTRY.config();
        cfg.set_text(&format!("start.pose = {pose:?}")).unwrap();
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        sim.run(0.5, |_| {});
        let mut out = Vec::new();
        sim.sample(&mut out);
        let at = |id: &str| out[sim.signal_index(id).unwrap()];
        let off = ["x", "y", "z"].map(|a| at(&format!("axis.{a}.position_setpoint")) - at(&format!("axis.{a}.position")));
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
    assert_eq!(sim.command(Command::MoveTo(vec![0.6014, 0.0, -0.775])).err().map(|e| e.key), Some("error-target-out-of-range"));
    for time_optimal in [false, true] {
        assert_eq!(sim.plan_path(&line(0.6014), time_optimal).err().map(|e| e.key), Some("error-path-range"));
        assert!(sim.plan_path(&line(0.6), time_optimal).is_ok());
    }
    let mut points = sim.plan_path(&line(0.6), false).unwrap();
    points.last_mut().unwrap().1[0] = 0.6014;
    assert_eq!(sim.command(Command::Trajectory(points)).err().map(|e| e.key), Some("error-trajectory"));
}
