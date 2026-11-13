//! The planar delta against hand calculations: kinematics, holding torques, an inertia torque, the working range's
//! edge, the cycle's time and verdicts.

use super::*;
use sim_components::{Command, Sim};

fn delta(edits: &[&str]) -> Delta2 {
    let mut cfg = ENTRY.config();
    for e in edits {
        cfg.set_text(e).unwrap();
    }
    Delta2::new(&cfg).unwrap()
}

fn close(a: f64, b: f64, tolerance: f64, what: &str) {
    assert!((a - b).abs() <= tolerance * b.abs().max(1e-12), "{what}: {a} against {b} by hand");
}

#[test]
fn the_planar_delta_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ENTRY);
}

/// Forward of inverse is the pose to 1e-12 m over a grid of the working range in steps of 10 mm, and the Jacobian is
/// the derivative of the forward kinematics, by central differences of 1e-6 rad, to 1e-7 m/rad.
#[test]
fn the_kinematics_round_trip_and_the_jacobian_is_the_derivative_of_the_forward_kinematics() {
    let d = delta(&[]);
    let mut tried = 0;
    for i in 0..=80 {
        for k in 0..=48 {
            let p = [-0.4 + 0.01 * i as f64, -0.877 + 0.01 * k as f64];
            if d.unreachable(&p, 0.0).is_some() {
                continue;
            }
            tried += 1;
            let t = d.legs.inverse(p).unwrap();
            let back = d.legs.forward(t).unwrap();
            for a in 0..2 {
                assert!((back[a] - p[a]).abs() < 1e-12, "{p:?} back as {back:?}");
            }
            let jac = d.legs.jacobian(t, p).unwrap();
            let h = 1e-6;
            for c in 0..2 {
                let (mut up, mut down) = (t, t);
                up[c] += h;
                down[c] -= h;
                let (pu, pd) = (d.legs.forward(up).unwrap(), d.legs.forward(down).unwrap());
                for r in 0..2 {
                    assert!((jac[r][c] - (pu[r] - pd[r]) / (2.0 * h)).abs() < 1e-7, "{p:?}: J[{r}][{c}]");
                }
            }
        }
    }
    assert!(tried > 2500, "{tried} poses");
}

/// The arm angle at the centre, from the leg's own equation solved by halving: (e + L_a cos t)^2 + (z + L_a sin t)^2 =
/// L_b^2, and the plate's height per arm angle when both turn together: differentiating that, dz/dt = -L_a cos t +
/// (e + L_a cos t) L_a sin t / (z + L_a sin t), half of it per arm.
fn centre_by_hand(d: &Delta2, z: f64) -> (f64, f64) {
    let (e, la, lb) = (d.legs.e, d.legs.la, d.legs.lb);
    let f = |t: f64| (e + la * t.cos()).powi(2) + (z + la * t.sin()).powi(2) - lb * lb;
    let (mut lo, mut hi) = (-60f64.to_radians(), 90f64.to_radians());
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        if f(lo) * f(mid) <= 0.0 { hi = mid } else { lo = mid }
    }
    let t = 0.5 * (lo + hi);
    let dz = -la * t.cos() + (e + la * t.cos()) * la * t.sin() / (z + la * t.sin());
    (t, dz / 2.0)
}

/// Static hold at the centre, 100 mm below the top: the plate's weight shares out evenly. By hand (default config):
/// e = 0.1 m, t = 1.4290 degrees, dz/dt per arm = -0.20503 m/rad, m_nt = 1.0 + 5 + 2 (1/3) 0.6 = 6.4 kg, m_LG =
/// 1.2 * 0.2 + 0.4 (0.4 + 2/3 0.6) = 0.560 kg m. Each gearbox holds -(g m_LG cos t) + m_nt g dz/dt = -5.490 - 12.868
/// = -18.358 N m (negative: the arm is held up), the motor that over 20: -0.9179 N m.
#[test]
fn the_holding_torque_at_the_centre_is_the_plates_weight_shared_out_and_the_arms_own() {
    let d = delta(&["effects.torque_ripple = false"]);
    let z = -0.5;
    let (t, dz) = centre_by_hand(&d, z);
    let (m_nt, m_lg) = (1.0 + 5.0 + 0.6 / 3.0 * 2.0, 1.2 * 0.2 + 0.4 * (0.4 + 0.6667 * 0.6));
    close(m_nt, d.m_nt(), 1e-3, "m_nt");
    close(m_lg, d.arm_moment(), 1e-12, "m_LG");
    let by_hand = -G * m_lg * t.cos() + m_nt * G * dz;
    println!("centre: t = {:.4} deg, dz/dt per arm {dz:.5} m/rad, holding {by_hand:.4} N m at each gearbox", t.to_degrees());
    let (mut state, mut torque) = (Vec::new(), [0.0; 2]);
    d.equilibrium(&[0.0, z], &mut state, &mut torque);
    for k in 0..2 {
        close(state[k], t, 1e-9, "arm angle");
        close(torque[k], by_hand / 20.0, 1e-4, "motor torque");
    }
    assert!((t.to_degrees() - 1.4290).abs() < 1e-3 && (by_hand + 18.358).abs() < 0.002, "{by_hand} N m");
}

/// Static hold at the edge, x = 0.4 m, 100 mm below the top: by the Jacobian's transpose with the Jacobian from central
/// differences of the forward kinematics (not the machine's own), tau_k = -g m_LG cos t_k + m_nt g dz/dt_k, at each
/// gearbox. The arm on the far side (arm 2, at 62.5 degrees) carries little: -25.43 N m on arm 1 (at -29.7 degrees),
/// -4.19 N m on arm 2 (by hand with the script of the hand calculation, the same sum).
#[test]
fn the_holding_torque_at_the_edge_is_the_weight_through_the_jacobians_transpose() {
    let d = delta(&["effects.torque_ripple = false"]);
    let p = [0.4, -0.5];
    let t = d.legs.inverse(p).unwrap();
    let m_lg = 1.2 * 0.2 + 0.4 * (0.4 + 0.6667 * 0.6);
    let (mut state, mut torque) = (Vec::new(), [0.0; 2]);
    d.equilibrium(&p, &mut state, &mut torque);
    let mut by_hand = [0.0; 2];
    for k in 0..2 {
        let (mut up, mut down) = (t, t);
        up[k] += 1e-6;
        down[k] -= 1e-6;
        let dz = (d.legs.forward(up).unwrap()[1] - d.legs.forward(down).unwrap()[1]) / 2e-6;
        by_hand[k] = -G * m_lg * t[k].cos() + d.m_nt() * G * dz;
        close(torque[k] * 20.0, by_hand[k], 1e-6, "gearbox torque");
    }
    println!("edge: {by_hand:?} N m at {:?} degrees", t.map(f64::to_degrees));
    assert!((by_hand[0] + 25.43).abs() < 0.01 && (by_hand[1] + 4.19).abs() < 0.01, "{by_hand:?}");
    // The rigid model held there stays put; so does the elastic one.
    for elastic in [false, true] {
        let d = delta(&[&format!("effects.joint_elasticity = {elastic}")]);
        let (mut state, mut torque) = (Vec::new(), [0.0; 2]);
        d.equilibrium(&p, &mut state, &mut torque);
        let mut rates = vec![0.0; d.state_len()];
        d.derivatives(&state, &torque, &mut rates);
        assert!(rates.iter().all(|r| r.abs() < 1e-6), "elastic {elastic}: {rates:?}");
    }
}

/// A vertical acceleration of 10 m/s^2 at the centre, 500 mm down: the arms turn together, t'' = a / (2 dz/dt per arm)
/// = -24.387 rad/s^2. I_b = 0.064 + 0.16 (0.4 + 2/3 0.6) = 0.1920 kg m^2. At the arm I_b t'' + m_nt a dz/dt = -4.682 -
/// 13.122 = -17.804 N m, at the motor over 20 plus the rotor's own (1.7 + 0.406) 1e-4 * 20 * t'' = -0.1027: -0.9929 N m.
/// The feedforward says so, and the rigid model given that torque accelerates the plate at 10 m/s^2 with no sideways part.
#[test]
fn a_vertical_acceleration_at_the_centre_takes_the_hand_calculated_inertia_torque() {
    let d = delta(&["effects.joint_friction = false", "effects.motor_friction = false", "effects.torque_ripple = false", "effects.gearbox_efficiency = false", "effects.joint_elasticity = false"]);
    let z = -0.5;
    let (_, dz) = centre_by_hand(&d, z);
    let i_b = 0.064 + 0.16 * (0.4 + 0.6667 * 0.6);
    close(i_b, d.arm_inertia(), 1e-12, "I_b");
    let acc = 10.0 / (2.0 * dz);
    let at_arm = i_b * acc + d.m_nt() * 10.0 * dz;
    let at_motor = at_arm / 20.0 + (1.7e-4 + 4.06e-5) * 20.0 * acc;
    println!("t'' = {acc:.3} rad/s^2, {at_arm:.4} N m at the arm, {at_motor:.4} N m at the motor");
    let (mut moving, mut still) = ([0.0; 2], [0.0; 2]);
    d.feedforward_torque(&[0.0, z], &[0.0, 10.0], &mut moving);
    d.feedforward_torque(&[0.0, z], &[0.0; 2], &mut still);
    for k in 0..2 {
        close(moving[k] - still[k], at_motor, 1e-6, "inertia torque");
    }
    assert!((at_motor + 0.9929).abs() < 1e-3, "{at_motor}");
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    d.equilibrium(&[0.0, z], &mut state, &mut hold);
    let mut rates = vec![0.0; d.state_len()];
    d.derivatives(&state, &moving, &mut rates);
    let (_, j) = d.pose([state[0], state[1]]);
    let plate = mul(&j, [rates[2], rates[3]]);
    assert!(plate[0].abs() < 1e-9 && (plate[1] - 10.0).abs() < 1e-9, "{plate:?}");
}

/// The working range's edge at a height, by hand: the bottom is the arc through both edges, 250 mm below the top, and
/// the centre, 477 mm below: radius r = (0.4^2 + 0.227^2) / (2 * 0.227) = 0.465923 m about z_c = -0.4 - 0.477 + r =
/// -0.411077 m. At z = -0.8 the edge is at |x| = sqrt(r^2 - (z - z_c)^2) = 0.256560 m; at z = -0.6, above the edges'
/// height (-0.65), it is the half width, 0.4 m.
#[test]
fn the_edge_of_the_working_range_at_a_height_is_where_the_bottom_arc_cuts_it() {
    let d = delta(&[]);
    let r = (0.4f64 * 0.4 + 0.227 * 0.227) / (2.0 * 0.227);
    let z_c = -0.4 - 0.477 + r;
    let edge = (r * r - (-0.8 - z_c).powi(2)).sqrt();
    assert!((edge - 0.256560).abs() < 1e-6, "{edge}");
    for side in [1.0, -1.0] {
        assert_eq!(d.unreachable(&[side * (edge - 1e-5), -0.8], 0.0), None);
        assert_eq!(d.unreachable(&[side * (edge + 1e-5), -0.8], 0.0), Some("error-delta-outside-envelope"));
        assert_eq!(d.unreachable(&[side * 0.4, -0.6], 0.0), None);
        assert_eq!(d.unreachable(&[side * 0.40001, -0.6], 0.0), Some("error-delta-outside-envelope"));
    }
    // The top, and the bottom at the centre.
    assert_eq!(d.unreachable(&[0.0, -0.39999], 0.0), Some("error-delta-outside-envelope"));
    assert_eq!(d.unreachable(&[0.0, -0.87699], 0.0), None);
    assert_eq!(d.unreachable(&[0.0, -0.87701], 0.0), Some("error-delta-outside-envelope"));
}

/// Targets the delta cannot go to are refused with the reason: outside the band, below the bottom arc, in reach but
/// beyond an arm's stop (arm 2 at 62.5 degrees at the edge), out of reach of a shorter forearm (L_a + L_b = 0.85 m
/// against 0.883 m from the folded shoulder to the bottom's centre), and above the condition number (1.23 at the edge).
#[test]
fn targets_outside_the_working_range_are_refused_with_their_reason() {
    let mut sim = Sim::new(&ENTRY, &ENTRY.config()).unwrap();
    let refused = |sim: &mut Sim, p: [f64; 2]| sim.command(Command::MoveTo(p.to_vec())).err().map(|e| e.key);
    assert_eq!(refused(&mut sim, [0.45, -0.5]), Some("error-target-out-of-range"));
    assert_eq!(refused(&mut sim, [0.4, -0.75]), Some("error-delta-outside-envelope"));
    assert_eq!(refused(&mut sim, [0.0, -0.45]), None);
    assert_eq!(delta(&["joint.max_angle_deg = 60.0"]).unreachable(&[0.4, -0.5], 0.0), Some("error-delta-joint-stop"));
    assert_eq!(delta(&["geometry.forearm_m = 0.45"]).unreachable(&[0.0, -0.877], 0.0), Some("error-delta-out-of-reach"));
    let d = delta(&["workspace.max_condition_number = 1.2"]);
    assert_eq!(d.unreachable(&[0.4, -0.5], 0.0), Some("error-delta-singularity"));
    assert_eq!(d.unreachable(&[0.0, -0.5], 0.0), None);
}

/// The Adept cycle's time in the plane, against the S-curve by hand, at 3 m/s, 50 m/s^2 and 2500 m/s^3 along x and z,
/// the D3's limits and lengths: the 305 mm traverse t = d/v + v/a + a/j = 0.18167 s, the 25 mm lift 4 (d / (2 j))^(1/3)
/// = 0.068399 s. On the 1 ms profile cycle: 4 * 69 + 2 * 182 + 4 * 2 + 2 * (35 + 1) = 720 cycles, 0.720 s.
#[test]
fn the_adept_cycle_takes_the_hand_calculated_time() {
    let (v, a, j): (f64, f64, f64) = (3.0, 50.0, 2500.0);
    let traverse = 0.305 / v + v / a + a / j;
    let lift = 4.0 * (0.025 / (2.0 * j)).cbrt();
    let mut sim = Sim::new(&ENTRY, &ENTRY.config()).unwrap();
    let load = sim_components::analysis::analyse_cycle_with(&mut sim, false).unwrap();
    let cycles = |t: f64| (t / 1e-3 - 1e-9).ceil();
    let ticked = 4.0 * (cycles(lift) + 2.0) + 2.0 * (cycles(traverse) + 2.0) + 2.0 * (35.0 + 1.0 - 2.0);
    println!("Adept cycle 25/305/25 in the plane: {:.4} s simulated, {ticked} cycles by hand", load.duration);
    assert!((load.duration - ticked * 1e-3).abs() < 1e-9 && ticked == 720.0, "{} s, {ticked} cycles", load.duration);
}

/// The default cycle passes every verdict, the singularity check among them: the cycle's highest condition number from
/// the signal, between the 1.02 of the centre and the 1.11 of the pick and place poses by hand (1.136 simulated, with
/// the lifts' tops), against the rating 5. A rating of 1.05 fails it.
#[test]
fn the_default_cycle_passes_its_verdicts() {
    let cfg = ENTRY.config();
    let load = sim_components::analysis::analyse_cycle_with(&mut Sim::new(&ENTRY, &cfg).unwrap(), false).unwrap();
    let verdicts = sim_components::analysis::verdicts(&cfg, &load).unwrap();
    let failed: Vec<_> = verdicts.iter().filter(|v| !v.ok).collect();
    assert!(failed.is_empty(), "{failed:?}");
    let peak = load.condition_peak.unwrap();
    let v = verdicts.iter().find(|v| v.check == "singularity").unwrap();
    assert!(v.value == peak && v.limit == 5.0 && peak > 1.1 && peak < 1.2, "{v:?}");
    let mut tight = cfg.clone();
    tight.set_text("ratings.max_condition_number = 1.05").unwrap();
    assert!(!sim_components::analysis::verdicts(&tight, &load).unwrap().iter().find(|v| v.check == "singularity").unwrap().ok);
}

/// The condition number of a matrix whose singular values are known: diag(1, 4) turned has 4.
#[test]
fn the_condition_number_is_the_ratio_of_the_singular_values() {
    let (c, s) = (0.6f64, 0.8f64);
    let m: M2 = [[c * 1.0, -s * 4.0], [s * 1.0, c * 4.0]];
    close(condition_number(&m), 4.0, 1e-12, "condition");
    close(condition_number(&[[2.0, 0.0], [0.0, 2.0]]), 1.0, 1e-12, "identity");
}

/// Held by constant torques (the holding torques of the pose), the stiffness of the potential gravity less torque
/// times angle, by second differences, in the symmetric mode (both arms together, the plate up and down) a + b and the
/// sideways mode a - b of [[a, b], [b, a]]. At the middle of the default range, 638.5 mm down: a + b = -3.49 N m/rad,
/// unstable; at 788.5 mm, the standard suite's middle, +7.52; a - b = 13.47 and 18.86, stable (by hand with the
/// script of the hand calculation: the same potential, V = m_nt g z - g m_LG (sin t_1 + sin t_2)).
#[test]
fn the_vertical_mode_under_constant_torques_is_unstable_high_up() {
    let mut cfg = lossless_config();
    cfg.set_text("effects.joint_elasticity = false").unwrap();
    let m = Delta2::new(&cfg).unwrap();
    let stiffness = |z: f64| {
        let (mut s, mut hold) = (Vec::new(), [0.0; 2]);
        m.equilibrium(&[0.0, z], &mut s, &mut hold);
        let tau: Vec<f64> = hold.iter().map(|t| t * 20.0).collect();
        let at = |t: V2| {
            let mut st = s.clone();
            st[..2].copy_from_slice(&t);
            m.energy(&st) - (0..2).map(|k| tau[k] * t[k]).sum::<f64>()
        };
        let (t0, h) = ([s[0], s[1]], 1e-4);
        let second = |i: usize, j: usize| {
            let f = |a: f64, b: f64| {
                let mut t = t0;
                t[i] += a;
                t[j] += b;
                at(t)
            };
            (f(h, h) - f(h, -h) - f(-h, h) + f(-h, -h)) / (4.0 * h * h)
        };
        (second(0, 0) + second(0, 1), second(0, 0) - second(0, 1))
    };
    let ((up, side_up), (down, side_down)) = (stiffness(-0.6385), stiffness(-0.7885));
    println!("638.5 mm: {up:.3}, {side_up:.3}; 788.5 mm: {down:.3}, {side_down:.3} N m/rad");
    assert!((up + 3.49).abs() < 0.05 && (down - 7.52).abs() < 0.05, "{up} {down}");
    assert!((side_up - 13.47).abs() < 0.05 && (side_down - 18.86).abs() < 0.05, "{side_up} {side_down}");
}

/// The feedforward with the speed is the rigid, loss-free model's torque: given it at a pose where the plate moves at
/// (2, -1) m/s, the model accelerates the plate at the asked (10, -20) m/s^2 (to 1e-9 m/s^2), the velocity term
/// included. Without the speed it misses by that term's effect, here more than 1 m/s^2.
#[test]
fn the_feedforward_with_the_speed_gives_the_asked_acceleration() {
    let d = delta(&["effects.joint_friction = false", "effects.motor_friction = false", "effects.torque_ripple = false", "effects.gearbox_efficiency = false", "effects.joint_elasticity = false"]);
    let (p, v, a) = ([0.2, -0.6], [2.0, -1.0], [10.0, -20.0]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    d.equilibrium(&p, &mut state, &mut hold);
    d.set_tool_velocity(&mut state, &v);
    let plate = |torque: &[f64]| {
        let mut rates = vec![0.0; d.state_len()];
        d.derivatives(&state, torque, &mut rates);
        let t = [state[0], state[1]];
        let (_, j) = d.pose(t);
        let bend = d.legs.velocity_term(t, p, [state[2], state[3]], v).unwrap();
        let along = mul(&j, [rates[2], rates[3]]);
        [along[0] + bend[0], along[1] + bend[1]]
    };
    let (mut moving, mut still) = ([0.0; 2], [0.0; 2]);
    d.feedforward_torque_moving(&p, &v, &a, &mut moving);
    d.feedforward_torque(&p, &a, &mut still);
    let (with, without) = (plate(&moving), plate(&still));
    assert!((0..2).all(|i| (with[i] - a[i]).abs() < 1e-9), "{with:?}");
    assert!((without[0] - a[0]).hypot(without[1] - a[1]) > 1.0, "{without:?}");
    d.feedforward_torque_moving(&p, &[0.0; 2], &a, &mut moving);
    assert!((0..2).all(|k| (moving[k] - still[k]).abs() < 1e-12));
}

/// The setpoint at rest is where the motors stand; the plate's weight twists the gearboxes and sits them at a side of
/// their backlash. `setpoint_slack` (`rest_sag`) is 0.420 mm by hand (the same grid, twist and half the 1 arcmin
/// either way, plus a tenth) and covers what the simulation shows after 0.5 s at rest at six poses, at most 0.368 mm
/// (at the edge, 650 mm down); without backlash and with stiff gearboxes it is the default 0.1 mm.
#[test]
fn the_setpoint_slack_covers_the_sag_at_rest() {
    let slack = delta(&[]).setpoint_slack();
    assert!((slack - 4.20e-4).abs() < 2e-6, "{slack}");
    let mut most = 0f64;
    for pose in [[0.4, -0.5], [0.0, -0.877], [0.1525, -0.525], [-0.4, -0.65], [0.3, -0.75], [0.2565, -0.8]] {
        let mut cfg = ENTRY.config();
        cfg.set_text(&format!("start.pose = {pose:?}")).unwrap();
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        sim.run(0.5, |_| {});
        let mut out = Vec::new();
        sim.sample(&mut out);
        let at = |id: &str| out[sim.signal_index(id).unwrap()];
        let off = ["x", "z"].map(|a| at(&format!("axis.{a}.position_setpoint")) - at(&format!("axis.{a}.position")));
        println!("{pose:?}: setpoint {:.3} mm off the tool", norm(off) * 1e3);
        most = most.max(norm(off));
    }
    assert!(most <= slack && slack <= 1.4 * most, "{most} m at rest, slack {slack} m");
    assert_eq!(delta(&["effects.gearbox_backlash = false", "gearbox.stiffness_nm_rad = 1e12"]).setpoint_slack(), 1e-4);
}

/// The slack is for where a path starts, not for where it goes: a line to x = 0.4014, 1.4 mm beyond the range's 0.4 m,
/// is refused as the move there is; one to 0.4 runs.
#[test]
fn a_path_may_not_use_the_setpoint_slack_away_from_its_start() {
    let mut sim = Sim::new(&ENTRY, &ENTRY.config()).unwrap();
    let line = |x: f64| sim_components::path::parse(&format!("line,{x},-0.5")).unwrap();
    assert_eq!(sim.command(Command::MoveTo(vec![0.4014, -0.5])).err().map(|e| e.key), Some("error-target-out-of-range"));
    for time_optimal in [false, true] {
        assert_eq!(sim.plan_path(&line(0.4014), time_optimal).err().map(|e| e.key), Some("error-path-range"));
        assert!(sim.plan_path(&line(0.4), time_optimal).is_ok());
    }
}

/// The scene draws the machine in the plane y = 0 and the collision bodies are the plate and the payload, where the
/// scene draws them: the plate's box centred on the tool at (x, 0, z).
#[test]
fn the_collision_bodies_are_the_plate_and_the_payload_at_the_tool() {
    let d = delta(&[]);
    let (mut state, mut torque) = (Vec::new(), [0.0; 2]);
    d.equilibrium(&[0.1525, -0.525], &mut state, &mut torque);
    let bodies = d.collision_bodies(&state);
    assert_eq!(bodies.len(), 2);
    let (mut pos, mut vel) = ([0.0; 2], [0.0; 2]);
    d.tool_state(&state, &mut pos, &mut vel);
    assert!((bodies[0].centre[0] - pos[0]).abs() < 1e-12 && bodies[0].centre[1] == 0.0 && (bodies[0].centre[2] - pos[1]).abs() < 1e-12);
    assert!((bodies[1].centre[2] - (pos[1] - 0.07)).abs() < 1e-12);
    let mut scene = Scene::default();
    d.scene(&state, &mut scene);
    assert_eq!(scene.parts.iter().filter(|p| p.id == "forearm").count(), 4);
}
