//! The linear delta against hand calculations: kinematics, the belts' holding forces, the working range's edges, the
//! feedforward, the default cycle.

use super::*;
use sim_components::Sim;

fn delta(edits: &[&str]) -> LinearDelta {
    let mut cfg = ENTRY.config();
    for e in edits {
        cfg.set_text(e).unwrap();
    }
    LinearDelta::new(&cfg).unwrap()
}

#[track_caller]
fn close(a: f64, b: f64, tolerance: f64, what: &str) {
    sim_core::testkit::record_check(what, b, a, tolerance * b.abs().max(1e-12));
    assert!((a - b).abs() <= tolerance * b.abs().max(1e-12), "{what}: {a} against {b} by hand");
}

#[test]
fn the_linear_delta_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ENTRY);
}

/// Forward of inverse is the pose to 1e-12 m over a grid of the working range, and the Jacobian is the derivative of
/// the forward kinematics, by central differences of 1e-6 m of carriage, to 1e-7.
#[test]
fn the_kinematics_round_trip_and_the_jacobian_is_the_derivative_of_the_forward_kinematics() {
    let d = delta(&[]);
    let mut tried = 0;
    for i in 0..=19 {
        for j in 0..=19 {
            for k in 0..=4 {
                let p = [-0.475 + 0.05 * i as f64, -0.475 + 0.05 * j as f64, -1.22 + 0.025 * k as f64];
                if d.unreachable(&p, 0.0).is_some() {
                    continue;
                }
                tried += 1;
                let h = d.legs.inverse(p).unwrap();
                let back = d.legs.forward(h).unwrap();
                for a in 0..3 {
                    assert!((back[a] - p[a]).abs() < 1e-12, "{p:?} back as {back:?}");
                }
                let jac = d.legs.jacobian(h, p).unwrap();
                let step = 1e-6;
                for c in 0..3 {
                    let (mut up, mut down) = (h, h);
                    up[c] += step;
                    down[c] -= step;
                    let (pu, pd) = (d.legs.forward(up).unwrap(), d.legs.forward(down).unwrap());
                    for r in 0..3 {
                        assert!((jac[r][c] - (pu[r] - pd[r]) / (2.0 * step)).abs() < 1e-7, "{p:?}: J[{r}][{c}]");
                    }
                }
                // All three carriages raised alike lift the plate alike: the Jacobian's z row sums to 1, its x and y
                // rows to 0.
                close(jac[2].iter().sum::<f64>(), 1.0, 1e-12, "sum of dz/dh");
                assert!(jac[0].iter().sum::<f64>().abs() < 1e-12 && jac[1].iter().sum::<f64>().abs() < 1e-12);
            }
        }
    }
    assert!(tried > 1000, "{tried} poses");
}

/// The belts hold each carriage and its share of the plate. By hand (default config): m_c = 1.3 + 0.35 / 2 = 1.475 kg,
/// m_nt = 0.8 + 3 * 0.35 / 2 + 1.0 = 2.325 kg, and the belt's force on carriage k is F_k = g (m_c + m_nt dz/dh_k).
/// At the centre the plate's weight shares out evenly, dz/dh_k = 1/3 (the z row sums to 1): F = 9.80665 * (1.475 +
/// 0.775) = 22.0650 N, at the motor F r = 22.0650 * 0.014324 = 0.31606 N m through the coupling. At (0.3, 0.1, -1.17)
/// with dz/dh from central differences of the forward kinematics (not the machine's Jacobian): 0.80346, 0.21018,
/// -0.01364, so 32.784, 19.257 and 14.154 N; they add up to 3 m_c g + m_nt g = 66.1949 N. The rigid model's holding
/// torques and the elastic one's belt sections (T1 - T2) say so.
/// The guides' loads the time-optimal plan reads (`Machine::rigid_guide_loads`): at rest those of `guide_loads`, the
/// rods' pull holding the plate times the reach; accelerating the plate straight up at a takes m (g + a) through the same
/// rods, so each guide's moving part is its load at rest times a / g (2 / 9.80665 = 0.20394 for 2 m/s^2), with the sign
/// of its pull.
#[test]
fn the_guide_loads_of_the_plan_grow_with_the_plates_upward_acceleration_by_hand() {
    let m = delta(&["effects.belt_stretch = false"]);
    let pose = [0.1, 0.05, -1.13];
    let h = m.legs.inverse(pose).unwrap();
    let (rest, mut loads) = (m.guide_loads(h, [0.0; 3]), Vec::new());
    m.rigid_guide_loads(&pose, &[0.0, 0.0, 2.0], &mut loads);
    let pull = m.legs.rod_pull(h, pose, m.m_nt(), G, [0.0; 3]).unwrap();
    for k in 0..3 {
        close(loads[k][0], rest[k], 1e-9 * rest[k], "load at rest");
        close(loads[k][1], pull[k] * m.legs.reach(k, pose) * 2.0 / G, 1e-9 * rest[k], "moving part");
        close(loads[k][1].abs(), rest[k] * 0.20394, 1e-4 * rest[k], "a / g of the load at rest");
    }
}

#[test]
fn the_belts_hold_the_carriages_and_the_plates_weight_through_the_jacobian() {
    let rigid = delta(&["effects.torque_ripple = false", "effects.belt_stretch = false"]);
    let elastic = delta(&["effects.torque_ripple = false"]);
    let (m_c, m_nt) = (1.3 + 0.35 / 2.0, 0.8 + 3.0 * 0.35 / 2.0 + 1.0);
    close(rigid.m_carriage(), m_c, 1e-12, "m_c");
    close(rigid.m_nt(), m_nt, 1e-12, "m_nt");
    let centre = G * (m_c + m_nt / 3.0);
    assert!((centre - 22.0650).abs() < 1e-4 && (centre * 0.014324 - 0.31606).abs() < 1e-5, "{centre}");
    let off = [0.3, 0.1, -1.17];
    let h = rigid.legs.inverse(off).unwrap();
    let dz: V3 = std::array::from_fn(|k| {
        let (mut up, mut down) = (h, h);
        up[k] += 1e-6;
        down[k] -= 1e-6;
        (rigid.legs.forward(up).unwrap()[2] - rigid.legs.forward(down).unwrap()[2]) / 2e-6
    });
    let by_hand: V3 = dz.map(|d| G * (m_c + m_nt * d));
    println!("dz/dh {dz:?}, belt forces {by_hand:?} N");
    for (k, f) in [32.784, 19.257, 14.154].into_iter().enumerate() {
        assert!((by_hand[k] - f).abs() < 1e-3, "{by_hand:?}");
    }
    close(by_hand.iter().sum(), 3.0 * m_c * G + m_nt * G, 1e-9, "sum");
    for (p, forces) in [([0.0, 0.0, -1.17], [centre; 3]), (off, by_hand)] {
        let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
        rigid.equilibrium(&p, &mut state, &mut torque);
        for k in 0..3 {
            close(torque[k], forces[k] * 0.014324, 1e-6, "holding torque");
        }
        elastic.equilibrium(&p, &mut state, &mut torque);
        let mut signals = Vec::new();
        elastic.base_signals(&state, &torque, &mut signals);
        for k in 0..3 {
            close(signals[6 * k + 1] - signals[6 * k + 2], forces[k], 1e-6, "T1 - T2");
        }
        // Held there, neither model moves.
        for m in [&rigid, &elastic] {
            let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
            m.equilibrium(&p, &mut state, &mut torque);
            let mut rates = vec![0.0; m.state_len()];
            m.derivatives(&state, &torque, &mut rates);
            assert!(rates.iter().all(|r| r.abs() < 1e-6), "{p:?}: {rates:?}");
        }
    }
}

/// The working range's edges at a height, by hand. With e = 0.52 - 0.07 = 0.45 m and L = 1.1 m: at z = -1.17 straight
/// away from column 1 (along -x) the rod of column 1 reaches the 60 degree tilt at x = e - L sin 60 = 0.45 - 0.952628 =
/// -0.502628 m (the range widened to 0.6 m for it; its carriage is then at z + L cos 60 = -0.62 m, within its stroke).
/// At z = -1.09 (the top raised for it) towards column 1, its carriage reaches its upper stop at 0 where
/// sqrt(L^2 - rho^2) = 1.09, rho = sqrt(1.21 - 1.1881) = 0.147986 m: x = 0.302014 m. Beyond each, the target is
/// refused with its reason; before it, not. In the default range the rods stand at most asin(0.925 / 1.1) = 57.2
/// degrees from the vertical.
#[test]
fn the_working_range_ends_where_a_rod_tilts_too_far_or_a_carriage_meets_its_stop() {
    let tilt = 0.45 - 1.1 * 60f64.to_radians().sin();
    assert!((tilt + 0.502628).abs() < 1e-6, "{tilt}");
    let d = delta(&["workspace.radius_m = 0.6"]);
    assert_eq!(d.unreachable(&[tilt + 1e-6, 0.0, -1.17], 0.0), None);
    assert_eq!(d.unreachable(&[tilt - 1e-6, 0.0, -1.17], 0.0), Some("error-delta-linear-rod-angle"));
    let h = d.legs.inverse([tilt, 0.0, -1.17]).unwrap();
    close(h[0], -0.62, 1e-9, "carriage at the tilt edge");
    let stop = 0.45 - (1.1f64 * 1.1 - 1.09 * 1.09).sqrt();
    assert!((stop - 0.302014).abs() < 1e-6, "{stop}");
    let d = delta(&["workspace.top_z_m = -1.0"]);
    assert_eq!(d.unreachable(&[stop - 1e-6, 0.0, -1.09], 0.0), None);
    assert_eq!(d.unreachable(&[stop + 1e-6, 0.0, -1.09], 0.0), Some("error-delta-linear-carriage-stop"));
    // The default range: outside the cylinder, above its top, out of the rods' reach, near a singularity.
    let d = delta(&[]);
    assert_eq!(d.unreachable(&[0.4, 0.3, -1.17], 0.0), Some("error-delta-outside-envelope"));
    assert_eq!(d.unreachable(&[0.0, 0.0, -1.1], 0.0), Some("error-delta-outside-envelope"));
    assert_eq!(d.unreachable(&[-0.475, 0.0, -1.22], 0.0), None);
    assert_eq!(
        delta(&["geometry.rod_m = 0.9"]).unreachable(&[-0.475, 0.0, -1.22], 0.0),
        Some("error-delta-out-of-reach")
    );
    assert_eq!(
        delta(&["workspace.max_condition_number = 3.0"]).unreachable(&[0.0, 0.0, -1.17], 0.0),
        Some("error-delta-singularity")
    );
    let steepest = (0.925f64 / 1.1).asin().to_degrees();
    assert!((steepest - 57.24).abs() < 0.01 && steepest < 60.0);
}

/// The feedforward with the speed is the rigid, loss-free model's force: given it at a pose where the plate moves at
/// (2, -1, 0.5) m/s, the model accelerates the plate at the asked (10, 5, -20) m/s^2, the velocity term
/// m_nt J^T J_x^-1 w included (to 1e-9 m/s^2). Without the speed it misses by that term's effect. At rest a vertical
/// 10 m/s^2 at the centre takes, per column, (m_c + J_pulley / r^2 + J_rotor / r^2) h'' + m_nt a / 3 with h'' = a:
/// (1.475 + (2e-5 + 0.126 * 0.014324^2 + 4e-4) / 0.014324^2) 10 + 2.325 * 10 / 3 = 44.2301 N, at the motor times r:
/// 0.63355 N m; the belt (0.07 kg/m of 4 * 0.1 + 2 * 0.7 = 1.8 m, 0.126 kg) on the pulley included.
#[test]
fn the_feedforward_gives_the_asked_acceleration() {
    let d = delta(&[
        "effects.guide_friction = false",
        "effects.motor_friction = false",
        "effects.torque_ripple = false",
        "effects.belt_stretch = false",
    ]);
    let (p, v, a) = ([0.2, -0.1, -1.17], [2.0, -1.0, 0.5], [10.0, 5.0, -20.0]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 3]);
    d.equilibrium(&p, &mut state, &mut hold);
    d.set_tool_velocity(&mut state, &v);
    let plate = |torque: &[f64]| {
        let mut rates = vec![0.0; d.state_len()];
        d.derivatives(&state, torque, &mut rates);
        let h = [state[0], state[1], state[2]];
        let (_, j) = d.pose(h);
        let hv = [state[3], state[4], state[5]];
        let bend = d.legs.velocity_term(h, p, hv, v).unwrap();
        let along = mul(&j, [rates[3], rates[4], rates[5]]);
        [0, 1, 2].map(|i| along[i] + bend[i])
    };
    let (mut moving, mut still) = ([0.0; 3], [0.0; 3]);
    d.feedforward_torque_moving(&p, &v, &a, &mut moving);
    d.feedforward_torque(&p, &a, &mut still);
    let (with, without) = (plate(&moving), plate(&still));
    assert!((0..3).all(|i| (with[i] - a[i]).abs() < 1e-9), "{with:?}");
    assert!((0..3).map(|i| (without[i] - a[i]).powi(2)).sum::<f64>().sqrt() > 0.1, "{without:?}");
    let (r, belt) = (0.014324f64, 0.07 * (4.0 * 0.1 + 2.0 * 0.7));
    let inertia = (1.475 + (2e-5 + belt * r * r + 4e-4) / (r * r)) * 10.0 + 2.325 * 10.0 / 3.0;
    let (mut up, mut rest) = ([0.0; 3], [0.0; 3]);
    d.feedforward_torque(&[0.0, 0.0, -1.17], &[0.0, 0.0, 10.0], &mut up);
    d.feedforward_torque(&[0.0, 0.0, -1.17], &[0.0; 3], &mut rest);
    for k in 0..3 {
        close(up[k] - rest[k], inertia * r, 1e-9, "inertia torque");
    }
    assert!((inertia * r - 0.63355).abs() < 1e-5, "{}", inertia * r);
}

/// At 3 m/s along y and no tool acceleration the carriages still accelerate (h'' = -J^-1 J_x^-1 w): the carriage part of
/// T1 - T2 = m_c h'' + m_nt J^T p'' - Q_g changes against rest by m_c h'' = (-12.182, -17.920, -17.920) N at (0.3, 0,
/// -1.17) (by hand: h'' a second difference over 0.1 ms of the heights h(p + v t) from the rods' constraint
/// |p - c_i - h_i e_z| = L, times m_c = 1.475 kg); `rigid_tensions` without the speed misses it.
#[test]
fn the_rigid_tensions_have_the_speed_term_of_the_carriage_acceleration() {
    let d = delta(&[]);
    let (p, v) = ([0.3, 0.0, -1.17], [0.0, 3.0, 0.0]);
    let (mut moving, mut rest) = (Vec::new(), Vec::new());
    d.rigid_tensions_moving(&p, &v, &[0.0; 3], &mut moving);
    d.rigid_tensions(&p, &[0.0; 3], &mut rest);
    for (k, want) in [-12.182, -17.920, -17.920].into_iter().enumerate() {
        let diff = (moving[2 * k] - moving[2 * k + 1]) - (rest[2 * k] - rest[2 * k + 1]);
        close(diff, want, 1e-4, "m_c h'' in T1 - T2");
    }
}

/// The default cycle passes every verdict, and takes the D3's hand-calculated 720 profile cycles: the same Adept cycle
/// 25/305/25 mm at the same 3 m/s, 50 m/s^2 and 2500 m/s^3 (see the D3's test).
#[test]
fn the_default_cycle_passes_its_verdicts() {
    let cfg = ENTRY.config();
    let load = sim_components::analysis::analyse_cycle(&mut Sim::new(&ENTRY, &cfg).unwrap()).unwrap();
    assert!(!load.tripped);
    assert!((load.duration - 0.720).abs() < 1e-9, "{} s", load.duration);
    for v in sim_components::analysis::verdicts(&cfg, &load).unwrap() {
        assert!(v.ok, "{} of motor {:?} is {} against {}", v.check, v.motor, v.value, v.limit);
    }
    // The cycle's condition number (3.28 at the lowered place position) against the rating.
    let peak = load.condition_peak.unwrap();
    assert!(peak > 3.2 && peak < 3.35, "{peak}");
}

/// The setpoint at rest is where the motors stand: the belt stretched under what it holds puts the plate off it.
/// `setpoint_slack` covers what the simulation shows after 0.5 s at rest at four poses.
#[test]
fn the_setpoint_slack_covers_the_sag_at_rest() {
    let slack = delta(&[]).setpoint_slack();
    for pose in [[0.0, 0.0, -1.17], [-0.475, 0.0, -1.22], [0.3, 0.3, -1.12], [0.1525, 0.0, -1.145]] {
        let mut cfg = ENTRY.config();
        cfg.set_text(&format!("start.pose = {pose:?}")).unwrap();
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        sim.run(0.5, |_| {});
        let mut out = Vec::new();
        sim.sample(&mut out);
        let at = |id: &str| out[sim.signal_index(id).unwrap()];
        let off = norm(
            ["x", "y", "z"].map(|a| at(&format!("axis.{a}.position_setpoint")) - at(&format!("axis.{a}.position"))),
        );
        println!("{pose:?}: setpoint {:.4} mm off the tool, slack {:.4} mm", off * 1e3, slack * 1e3);
        assert!(off <= slack, "{off} m at rest, slack {slack} m");
    }
}

/// Upright columns (incline 0, the shipped value) are the vertical linear delta of before, to the bit: the carriage at
/// z + sqrt(L^2 - rho^2) with rho the horizontal distance from its joint line (as the vertical delta had it), and the
/// forward kinematics back from there. And the hand-calculated numbers the vertical delta's tests hold (see
/// `the_belts_hold_the_carriages_and_the_plates_weight_through_the_jacobian`,
/// `the_feedforward_gives_the_asked_acceleration`): at the centre (0, 0, -1.17) each belt holds g (m_c + m_nt / 3) =
/// 22.0650 N, 0.31606 N m at the motor; 10 m/s^2 upwards takes 0.63355 N m more; at (0.3, 0.1, -1.17) 32.784, 19.257 and
/// 14.154 N. The guide at the centre carries the rods' horizontal pull at rest: lambda e with lambda = m_nt g / (3
/// sqrt(L^2 - e^2)), sqrt(1.21 - 0.2025) = 1.003743, so 2.325 * 9.80665 * 0.45 / (3 * 1.003743) = 3.40732 N.
#[test]
fn the_inclined_columns_at_0_degrees_are_the_vertical_linear_delta() {
    let d = delta(&["effects.torque_ripple = false", "effects.belt_stretch = false"]);
    assert!(d.legs.dir.iter().all(|v| v[0] == 0.0 && v[1] == 0.0 && v[2] == 1.0), "{:?}", d.legs.dir);
    let e = 0.45;
    for p in [[0.0, 0.0, -1.17], [0.3, 0.1, -1.17], [-0.2, 0.35, -1.2], [0.1525, 0.0, -1.13]] {
        let h = d.legs.inverse(p).unwrap();
        let mut motors = [0.0; 3];
        d.inverse(&p, &mut motors);
        for k in 0..3 {
            let (c, s) = d.legs.azimuth[k];
            let rho = (p[0] - e * c).hypot(p[1] - e * s);
            let vertical = p[2] + (1.1f64 * 1.1 - rho * rho).sqrt();
            assert_eq!(h[k], vertical, "{p:?}: carriage {k}");
            assert_eq!(motors[k], (vertical - (0.0 - 0.7)) / 0.014324, "{p:?}: motor {k}");
        }
        let back = d.legs.forward(h).unwrap();
        assert!((0..3).all(|i| (back[i] - p[i]).abs() < 1e-12), "{p:?} back as {back:?}");
    }
    let (r, centre) = (0.014324, [0.0, 0.0, -1.17]);
    let (mut hold, mut up, mut off) = ([0.0; 3], [0.0; 3], [0.0; 3]);
    d.feedforward_torque(&centre, &[0.0; 3], &mut hold);
    d.feedforward_torque(&centre, &[0.0, 0.0, 10.0], &mut up);
    d.feedforward_torque(&[0.3, 0.1, -1.17], &[0.0; 3], &mut off);
    for k in 0..3 {
        close(hold[k], G * (1.475 + 2.325 / 3.0) * r, 1e-12, "holding torque");
        assert!((hold[k] - 0.31606).abs() < 1e-5 && (up[k] - hold[k] - 0.63355).abs() < 1e-5, "{hold:?} {up:?}");
    }
    for (k, f) in [32.784, 19.257, 14.154].into_iter().enumerate() {
        assert!((off[k] / r - f).abs() < 1e-3, "{off:?}");
    }
    let (mut state, mut torque, mut signals) = (Vec::new(), [0.0; 3], Vec::new());
    d.equilibrium(&centre, &mut state, &mut torque);
    d.base_signals(&state, &torque, &mut signals);
    let lambda = 2.325 * G / (3.0 * (1.21f64 - 0.2025).sqrt());
    for k in 0..3 {
        close(signals[6 * k + 4], lambda * e, 1e-9, "guide load at rest");
        assert!((signals[6 * k + 4] - 3.40732).abs() < 1e-5, "{signals:?}");
    }
}

/// A column inclined by 20 degrees, tops inwards, the plate at rest at the centre (0, 0, -1.17), worked out by hand in
/// the plane of column and axis (r outwards, z up). The column runs along d = (-sin 20, cos 20) = (-0.342020, 0.939693)
/// from the joint line's point (0.45, 0); the plate is at q = (-0.45, -1.17) from it: q . d = 0.153909 - 1.099441 =
/// -0.945532, |q|^2 - (q . d)^2 = 1.5714 - 0.894031 = 0.677369, so the carriage travels h = -0.945532 + sqrt(1.21 -
/// 0.677369) = -0.215717 m and the rod runs u = q - h d = (-0.523779, -0.967293). The three rods share the plate's
/// weight: 3 lambda u_z = -m_nt g, lambda = 2.325 * 9.80665 / (3 * 0.967293) = 7.857140 N/m. On the carriage act the
/// rod, lambda u = (-4.115405, -7.600154) N, and its weight m_c g = 1.475 * 9.80665 = 14.464809 N down. Along the
/// column the belt holds -(lambda u - m_c g e_z) . d = 13.592474 (the weight's cos 20 share) + 5.734256 (the rod's) =
/// 19.326730 N; across it the guide carries |(lambda u - m_c g e_z) . (cos 20, sin 20)| = |-3.867199 - 7.546684| =
/// 11.413880 N, of which the weight's sin 20 share is 4.947256 N.
#[test]
fn an_inclined_column_holds_its_carriage_partly_on_the_belt_and_partly_on_the_guide() {
    let incline = "geometry.column_incline_deg = 20.0";
    let rigid = delta(&["effects.torque_ripple = false", "effects.belt_stretch = false", incline]);
    let elastic = delta(&["effects.torque_ripple = false", incline]);
    let (s, c) = 20f64.to_radians().sin_cos();
    let (m_c, m_nt, e, z) = (1.475, 2.325, 0.45, -1.17);
    let q = [-e, z];
    let along = q[0] * -s + q[1] * c;
    let h = along + (1.21 - (q[0] * q[0] + q[1] * q[1] - along * along)).sqrt();
    let u = [q[0] + h * s, q[1] - h * c];
    let lambda = -m_nt * G / (3.0 * u[1]);
    let force = [lambda * u[0], lambda * u[1] - m_c * G];
    let (belt, guide) = (-(force[0] * -s + force[1] * c), (force[0] * c + force[1] * s).abs());
    assert!((h + 0.215717).abs() < 1e-6 && (lambda - 7.857140).abs() < 1e-6, "{h} m, {lambda} N/m");
    assert!((belt - 19.326730).abs() < 1e-6 && (guide - 11.413880).abs() < 1e-6, "{belt} N, {guide} N");
    close(m_c * G * c, 13.592474, 1e-7, "the weight along the column");
    close(m_c * G * s, 4.947256, 1e-7, "the weight across it");
    let p = [0.0, 0.0, z];
    let hs = rigid.legs.inverse(p).unwrap();
    assert!(hs.iter().all(|x| (x - h).abs() < 1e-12), "{hs:?}");
    let joint = rigid.legs.joint(0, h);
    close(joint[0], e + h * -s, 1e-12, "joint x");
    close(joint[2], h * c, 1e-12, "joint z");
    let back = rigid.legs.forward(hs).unwrap();
    assert!((0..3).all(|i| (back[i] - p[i]).abs() < 1e-12), "{back:?}");
    let (mut state, mut torque, mut signals) = (Vec::new(), [0.0; 3], Vec::new());
    rigid.equilibrium(&p, &mut state, &mut torque);
    rigid.base_signals(&state, &torque, &mut signals);
    for k in 0..3 {
        close(torque[k], belt * 0.014324, 1e-9, "holding torque");
        close(signals[6 * k + 4], guide, 1e-9, "guide load");
    }
    elastic.equilibrium(&p, &mut state, &mut torque);
    signals.clear();
    elastic.base_signals(&state, &torque, &mut signals);
    for k in 0..3 {
        close(signals[6 * k + 1] - signals[6 * k + 2], belt, 1e-6, "T1 - T2");
    }
    // Held off the centre, neither model moves; and the Jacobian is the forward kinematics' derivative there.
    let p = [0.1, -0.2, -1.18];
    for m in [&rigid, &elastic] {
        let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
        m.equilibrium(&p, &mut state, &mut torque);
        let mut rates = vec![0.0; m.state_len()];
        m.derivatives(&state, &torque, &mut rates);
        assert!(rates.iter().all(|r| r.abs() < 1e-6), "{rates:?}");
    }
    let hs = rigid.legs.inverse(p).unwrap();
    let jac = rigid.legs.jacobian(hs, p).unwrap();
    for col in 0..3 {
        let (mut up, mut down) = (hs, hs);
        up[col] += 1e-6;
        down[col] -= 1e-6;
        let (pu, pd) = (rigid.legs.forward(up).unwrap(), rigid.legs.forward(down).unwrap());
        for row in 0..3 {
            assert!((jac[row][col] - (pu[row] - pd[row]) / 2e-6).abs() < 1e-7, "J[{row}][{col}]");
        }
    }
}

/// The guide load follows the motion: the plate accelerating at 10 m/s^2 upwards at the centre, the rods pull with
/// m_nt (g + a) / (3 sqrt(L^2 - e^2)) and their horizontal part is lambda e. Upright, by hand: at rest 3.40732 N (see
/// above), with 10 m/s^2 2.325 * 10 * 0.45 / (3 * 1.003743) = 3.47449 N more, 6.88181 N. The carriage accelerates along
/// its column and adds nothing across it. The rigid, loss-free model is driven with the feedforward that gives that
/// acceleration (`the_feedforward_gives_the_asked_acceleration`); the verdicts then judge the highest of these loads.
#[test]
fn the_guide_load_over_the_motion_adds_the_rods_inertial_pull() {
    let d = delta(&[
        "effects.guide_friction = false",
        "effects.motor_friction = false",
        "effects.torque_ripple = false",
        "effects.belt_stretch = false",
    ]);
    let p = [0.0, 0.0, -1.17];
    let (mut state, mut hold, mut torque) = (Vec::new(), [0.0; 3], [0.0; 3]);
    d.equilibrium(&p, &mut state, &mut hold);
    d.feedforward_torque(&p, &[0.0, 0.0, 10.0], &mut torque);
    let mut rates = vec![0.0; d.state_len()];
    d.derivatives(&state, &torque, &mut rates);
    assert!((3..6).all(|i| (rates[i] - 10.0).abs() < 1e-9), "{rates:?}");
    let (mut still, mut moving) = (Vec::new(), Vec::new());
    d.base_signals(&state, &hold, &mut still);
    d.base_signals(&state, &torque, &mut moving);
    let inertial = 2.325 * 10.0 * 0.45 / (3.0 * (1.21f64 - 0.2025).sqrt());
    assert!((inertial - 3.47449).abs() < 1e-5, "{inertial}");
    for k in 0..3 {
        close(moving[6 * k + 4] - still[6 * k + 4], inertial, 1e-9, "inertial part of the guide load");
        assert!((moving[6 * k + 4] - 6.88181).abs() < 1e-5, "{moving:?}");
    }
    // The verdicts judge each guide's highest load and its life (`rating_checks`).
    let checks = delta(&[]).rating_checks();
    for k in 1..=3 {
        let (load, life) = (format!("carriage.{k}.guide_load"), format!("carriage.{k}.guide_damage_rate"));
        assert!(checks.iter().any(|c| c.check == "guide-load" && c.signal == load && c.limit == 3050.0 && !c.life));
        assert!(checks.iter().any(|c| c.check == "guide-life" && c.signal == life && c.limit == 20000.0 && c.life));
    }
}

/// With effects.part_heating each guide warms with its friction loss through its first-order lag: rise' = (R P - rise)
/// / tau, P = F v the guide's friction force (its Coulomb, viscous and stiction parts) times the carriage's speed, as the
/// friction signals report them; here R 2 K/W and tau 0.02 s along a move of 0.6 m across the working range. Step by
/// step, the loss held over each step: rise += (1 - exp(-dt / tau)) (R P - rise). With effects.thermal_feedback a guide
/// 30 K warm has its viscous friction at exp(4000 (1 / 323.15 - 1 / 293.15)) = 0.281749 of the configured: at 0.5 m/s
/// 20 * 0.5 * 0.281749 = 2.81749 N.
#[test]
fn the_guides_warm_with_their_loss() {
    let parts = delta(&[]).thermal_parts();
    let named: Vec<_> = parts.iter().map(|p| (p.id, p.kind, p.scene_part, p.scene_index)).collect();
    let guides = [
        ("guide_1", "guide", "carriage", 0),
        ("guide_2", "guide", "carriage", 1),
        ("guide_3", "guide", "carriage", 2),
        ("belt_1", "belt", "belt", 0),
        ("belt_2", "belt", "belt", 7),
        ("belt_3", "belt", "belt", 14),
    ];
    assert_eq!(named, guides);
    let mut cfg = ENTRY.config();
    for edit in [
        "effects.part_heating = true",
        "thermal.guide_time_constant_s = 0.02",
        "thermal.guide_resistance_k_w = 2.0",
        "start.pose = [-0.3, 0.0, -1.15]",
    ] {
        cfg.set_text(edit).unwrap();
    }
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    assert_eq!(sim.heated.len(), 6);
    sim.command(sim_components::control::Command::MoveTo(vec![0.3, 0.0, -1.15])).unwrap();
    let ids: Vec<[usize; 4]> = (1..=3)
        .map(|k| {
            ["speed", "coulomb", "viscous", "stiction"]
                .map(|part| sim.signal_index(&format!("carriage.{k}.friction_{part}")).unwrap())
        })
        .collect();
    let (mut row, mut by_hand, mut hottest) = (Vec::new(), [0.0; 3], 0f64);
    let lag = 1.0 - (-sim.dt / 0.02f64).exp();
    while sim.control.busy() {
        sim.step();
        row.clear();
        sim.sample(&mut row);
        for k in 0..3 {
            let [v, coulomb, viscous, stiction] = ids[k].map(|i| row[i]);
            by_hand[k] += lag * (2.0 * (coulomb + viscous + stiction) * v - by_hand[k]);
            hottest = hottest.max(by_hand[k]);
        }
    }
    assert!(hottest > 1.0, "{hottest} K");
    for k in 0..3 {
        let rise = sim.heated[k].rise;
        assert!((rise - by_hand[k]).abs() <= 1e-9 * hottest, "{rise} K against {} K", by_hand[k]);
    }
    let mut d = delta(&["effects.thermal_feedback = true"]);
    d.set_part_temperatures(&[30.0; 3]);
    for k in 0..3 {
        let curve = d.friction_curve(k, 0.5, 1.0);
        assert!((curve[1] - 2.81749).abs() < 1e-5, "{curve:?}");
    }
}

/// Friction feedforward from the machine's own model (`steady_friction`), by hand: the plate on the axis going down at
/// 0.1 m/s (axis.z's speed limit set to it). There each carriage is e = 0.52 - 0.07 = 0.45 m beside the plate and
/// sqrt(1.1^2 - 0.45^2) = 1.003743 m above it, so it moves with the plate, h' = -0.1 m/s. Its guide carries what the
/// rod pair's pull has across the column: the plate's m_nt g = (0.8 + 1.0 + 3 x 0.5 x 0.35) 9.80665 = 22.80046 N shared
/// by three, times 0.45 / 1.003743: 3.40728 N, load scale (200 + 3.40728) / 200 = 1.017036, so it slides at 15 N x
/// 1.017036 + 20 N s/m x 0.1 m/s = 17.25555 N (Stribeck part exp(-100) nothing), times r = 0.014324 m through the
/// coupling (ratio and efficiency 1): 0.247168 N m. The shaft at 0.1 / 0.014324 = 6.98129 rad/s: 0.02 tanh(13.96) +
/// 1e-4 x 6.98129 = 0.020698 N m. Each motor: -0.267867 N m. In the cruise the velocity loop's integrator then carries
/// nothing (the coupling loses nothing on the weight); without the feedforward, the friction.
#[test]
fn the_machine_friction_feedforward_is_the_steady_friction_and_the_integrator_carries_nothing() {
    let cruise = |model: bool| {
        let mut cfg = ENTRY.config();
        cfg.set_text("effects.friction_feedforward = true").unwrap();
        let source = if model { "machine" } else { "characteristic" };
        cfg.set_text(&format!("drive.friction_feedforward_model = \"{source}\"")).unwrap();
        cfg.set_text("start.pose = [0.0, 0.0, -1.125]").unwrap();
        cfg.set_text("axis.z.max_velocity_m_s = 0.1").unwrap();
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        let at = sim.control.set_pos.clone();
        sim.command(sim_components::Command::MoveTo(vec![at[0], at[1], -1.215])).unwrap();
        let (mut ff, mut integrator, mut n) = ([0.0; 3], [0.0; 3], 0);
        sim.run(1.0, |s| {
            if s.time() > 0.3 && s.control.set_acc[2] == 0.0 && (s.control.set_vel[2] + 0.1).abs() < 1e-12 {
                n += 1;
                for k in 0..3 {
                    ff[k] = s.control.drives[k].friction_ff;
                    integrator[k] += s.control.drives[k].controller_torque;
                }
            }
        });
        assert!(n > 1000, "{n} samples in the cruise");
        (ff, integrator.map(|i| i / n as f64))
    };
    let expected = -(0.247168 + 0.020698);
    let ((ff, with), (_, without)) = (cruise(true), cruise(false));
    for k in 0..3 {
        assert!((ff[k] - expected).abs() < 1e-4 * expected.abs(), "motor {k}: feedforward {ff:?}, by hand {expected}");
        assert!(with[k].abs() < 0.02 * expected.abs(), "motor {k}: the integrator carries {with:?}");
        assert!((without[k] - expected).abs() < 0.02 * expected.abs(), "motor {k}: without {without:?}");
    }
}

/// On an inclined column the carriage's weight m_c g sin(alpha) also loads the rail: the plan's guide load at rest is
/// that of `guide_loads`, the part of (lambda u - m_c g e_z) at right angles to the column, not |lambda| times the reach.
/// At (0, 0, -1.17) with the default masses that is 11.414 N at +20 degrees and 2.762 N at -15 (the reach alone: 6.467
/// and 0.982 N). By hand: lambda from the plate's balance sum(-lambda_i u_i) = m_nt g e_z, the rods u_i from the
/// constraint along the inclined columns, then |f - (f . d_i) d_i| with f = lambda_i u_i - m_c g e_z: 11.41388, 2.76205 N.
#[test]
fn the_plans_guide_load_at_rest_holds_the_carriages_weight_on_an_inclined_column() {
    for (deg, expected) in [(20.0, 11.41388), (-15.0, 2.76205)] {
        let m = delta(&["effects.belt_stretch = false", &format!("geometry.column_incline_deg = {deg}")]);
        let pose = [0.0, 0.0, -1.17];
        let h = m.legs.inverse(pose).unwrap();
        let (rest, mut loads) = (m.guide_loads(h, [0.0; 3]), Vec::new());
        m.rigid_guide_loads(&pose, &[0.0; 3], &mut loads);
        for k in 0..3 {
            close(loads[k][0], rest[k], 1e-9, "load at rest as guide_loads");
            close(loads[k][0], expected, 1e-5, "load at rest by hand");
            close(loads[k][1], 0.0, 1e-9, "no moving part at rest");
        }
    }
}

/// The carriages move along their (inclined) columns: with p = (0.1, -0.05, -1.17), p' = (1, 0.5, 0.8) and 20 degrees
/// the linear momentum is m_nt p' + m_c sum h'_i d_i = (1.875, 0.856, 5.693) N s (the carriages at the column radius
/// moving straight up gave (2.325, 1.163, 5.939)); by hand h' by central differences of the heights from the rods'
/// constraint along the inclined columns: (1.87530, 0.85615, 5.69276).
#[test]
fn the_carriages_carry_momentum_along_their_columns() {
    let m = delta(&["effects.belt_stretch = false", "geometry.column_incline_deg = 20.0"]);
    let (p, v) = ([0.1, -0.05, -1.17], [1.0, 0.5, 0.8]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 3]);
    m.equilibrium(&p, &mut state, &mut hold);
    m.set_tool_velocity(&mut state, &v);
    let f = m.frame_momentum(&state).unwrap();
    for (k, by_hand) in [1.87530, 0.85615, 5.69276].into_iter().enumerate() {
        close(f.linear[k], by_hand, 1e-5, "linear momentum");
    }
}

/// The guide's moments from joints standing off the slide: a joint a = 0.05 m along the column from the centre gives
/// M = a d cross f with f the pull on the carriage, so Mx = M.d = 0 and |M| = a F (F the pull's part across the
/// column; the weight acts at the centre). With the ratings My = Mz = 20 N m the equivalent load is
/// P = F + F_max (|My| + |Mz|) / 20, F_max = 3050 N. Without the ratings the load is F as before.
#[test]
fn the_guides_moments_from_the_joints_offset_add_to_its_equivalent_load() {
    let plain = delta(&[]);
    // The moment ratings are keys a config file may add (`analysis::RATINGS_BEYOND_DEFAULT`).
    let rated = "\n[ratings]\nguide_moment_y_nm = 20.0\nguide_moment_z_nm = 20.0\n";
    let mut cfg =
        sim_components::load_config(&ENTRY, &ENTRY.default_config.replacen("\n[ratings]\n", rated, 1)).unwrap();
    cfg.set_text("geometry.rod_joint_along_m = 0.05").unwrap();
    let m = LinearDelta::new(&cfg).unwrap();
    let h = plain.legs.inverse([0.1, 0.05, -1.0]).unwrap();
    let across = plain.guide_loads(h, [0.0; 3]);
    let p = m.pose(h).0;
    let pull = m.legs.rod_pull(h, p, m.m_nt(), m.g, [0.0; 3]).unwrap();
    let parts = m.guide_loads_at(h, p, [0.0; 3]);
    for k in 0..3 {
        let (load, moments) = parts[k];
        let (u, d) = (sub(p, m.legs.joint(k, h[k])), m.legs.dir[k]);
        let rod = u.map(|x| pull[k] * x);
        let along = dot(rod, d);
        let rod_across = norm(std::array::from_fn::<f64, 3, _>(|i| rod[i] - along * d[i]));
        assert!(moments[0].abs() < 1e-9, "Mx {moments:?}");
        close(norm(moments), 0.05 * rod_across, 1e-9, "|M| = a F");
        close(load, across[k] + 3050.0 * (moments[1].abs() + moments[2].abs()) / 20.0, 1e-12, "equivalent load");
        assert!(load > across[k], "moments add");
    }
    assert_eq!(delta(&["geometry.rod_joint_along_m = 0.05"]).guide_loads(h, [0.0; 3]), across);
}

/// The friction's load factor under motion is the fixed point of scale -> acceleration -> load -> scale: put back
/// into the model's acceleration (`carriage_acceleration_with`), it gives the load that gives it back; and it is not the factor at rest, the plate accelerating at
/// (8, 3, 10) m/s^2 (the load along one column falls, along another it rises). Rigid and elastic, with the guides
/// sliding at 0.2 m/s (the bristles at their steady deflection) so that the friction acts. Without load-dependent
/// friction every factor is 1.
#[test]
fn the_frictions_load_factor_is_the_fixed_point_with_the_acceleration() {
    for stretch in [false, true] {
        let d = delta(&[
            "effects.motor_friction = false",
            "effects.torque_ripple = false",
            &format!("effects.belt_stretch = {stretch}"),
        ]);
        let p = [0.0, 0.0, -1.17];
        let (mut state, mut hold, mut torque) = (Vec::new(), [0.0; 3], [0.0; 3]);
        d.equilibrium(&p, &mut state, &mut hold);
        d.set_tool_velocity(&mut state, &[0.0, 0.0, 0.2]);
        let (h, v) = d.heights(&state);
        let at = if stretch { 18 } else { 6 };
        for k in 0..3 {
            state[at + k] = d.friction[k].level(v[k], d.load_scales(h)[k]) / d.friction[k].sigma0 * v[k].signum();
        }
        d.feedforward_torque(&p, &[8.0, 3.0, 10.0], &mut torque);
        let scales = d.scales_at(&state, &torque);
        let plate = d.plate_acceleration(&state, d.carriage_acceleration_with(&state, &torque, scales));
        let loads = d.guide_loads(h, plate);
        let rest = d.load_scales(h);
        for k in 0..3 {
            close(scales[k], d.friction[k].load_scale(loads[k], 0.0), 1e-12, "fixed point");
            assert!((scales[k] - rest[k]).abs() > 1e-4, "stretch {stretch}: {scales:?} at rest {rest:?}");
        }
        let plain = delta(&["effects.load_dependent_friction = false", &format!("effects.belt_stretch = {stretch}")]);
        assert_eq!(plain.scales_at(&state, &torque), [1.0; 3]);
    }
}

/// A belt is a thermal part of the loop's own size: the loop is 4 span + 2 stroke = 0.4 + 1.4 = 1.8 m long (the sections'
/// geometric length less the teeth's), 19.3 mm wide, both sides in the air: A = 2 * 0.0193 * 1.8 = 0.06948 m^2, at
/// 15 W/(m^2 K) R = 1 / 1.0422 = 0.959509 K/W; 0.07 kg/m * 1.8 m = 0.126 kg of 1500 J/(kg K) is C = 189 J/K, so tau = C R
/// = 181.347 s. The belt warms with its damping loss, power.belt_k, as the guides do (rise += (1 - exp(-dt / tau)) (R P -
/// rise)), so under a steady loss P it settles at R P; here with the specific heat at 1.5 J/(kg K) (tau 0.1813 s) so that
/// a move of the carriages follows it. Without its pulleys (effects.belt_pulley_conduction off).
#[test]
fn a_belt_warms_with_its_damping_loss_through_its_surface() {
    let air = "effects.belt_pulley_conduction = false";
    let (r, tau) = delta(&[air]).thermal_values("belt_2").unwrap();
    close(r, 1.0 / (15.0 * 2.0 * 0.0193 * 1.8), 1e-12, "resistance");
    close(r, 0.959509, 1e-6, "resistance by hand");
    close(tau, 0.07 * 1.8 * 1500.0 * r, 1e-12, "time constant");
    close(tau, 181.347, 1e-5, "time constant by hand");
    assert!(delta(&[]).thermal_values("guide_1").is_none());
    // Each loop is drawn as seven strips (the belt parts 0, 7 and 14 are the loops' first), none of zero length: the
    // pulleys stand `span_min_m` beyond the stops. At the working range's top and bottom.
    let d = delta(&[]);
    for z in [-1.12, -1.22] {
        let (mut state, mut torque, mut scene) = (Vec::new(), [0.0; 3], sim_core::Scene::default());
        d.equilibrium(&[0.0, 0.0, z], &mut state, &mut torque);
        d.scene(&state, &mut scene);
        assert_eq!(scene.parts.iter().filter(|p| p.id == "belt").count(), 21, "at z = {z}");
    }
    let mut cfg = ENTRY.config();
    for edit in
        ["effects.part_heating = true", "belt.specific_heat_j_kg_k = 1.5", "start.pose = [-0.3, 0.0, -1.15]", air]
    {
        cfg.set_text(edit).unwrap();
    }
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    let fast = 0.07 * 1.8 * 1.5 * r;
    close(sim.heated[4].resistance, r, 1e-12, "resistance in the model");
    close(sim.heated[4].time_constant, fast, 1e-12, "time constant in the model");
    sim.command(sim_components::control::Command::MoveTo(vec![0.3, 0.0, -1.15])).unwrap();
    let power = sim.signal_index("power.belt_2").unwrap();
    let (mut row, mut by_hand) = (Vec::new(), 0.0);
    let lag = 1.0 - (-sim.dt / fast).exp();
    while sim.control.busy() {
        sim.step();
        row.clear();
        sim.sample(&mut row);
        by_hand += lag * (r * row[power] - by_hand);
    }
    assert!(by_hand > 0.0, "{by_hand} K");
    close(sim.heated[4].rise, by_hand, 1e-9, "the belt's rise");
}

/// With effects.belt_pulley_conduction (as shipped) the belt also gives heat to its two pulleys (radius 14.324 mm) through
/// the half wrap it lies on each, 2 pi r w = 2 pi 0.014324 0.0193 = 1.737046e-3 m^2, from its middle through half of its
/// 3 mm at 0.2 W/(m K): 133.33 W/(m^2 K), G = 0.231601 W/K beside the air's 1.0422 W/K, 22.2 % of it. R = 1 / 1.273801 =
/// 0.785052 K/W, tau = 189 J/K R = 148.375 s: the belt settles 18.2 % cooler under the same loss.
#[test]
fn a_belt_gives_heat_to_its_pulleys_through_the_wrap() {
    let (r, tau) = delta(&[]).thermal_values("belt_2").unwrap();
    let pulleys = std::f64::consts::TAU * 0.014324 * 0.0193 * 0.2 / 1.5e-3;
    close(pulleys, 0.231601, 1e-5, "the pulleys' conductance by hand");
    close(r, 1.0 / (15.0 * 2.0 * 0.0193 * 1.8 + pulleys), 1e-12, "resistance");
    close(r, 0.785052, 1e-5, "resistance by hand");
    close(tau, 189.0 * r, 1e-12, "time constant");
    close(tau, 148.375, 1e-5, "time constant by hand");
    let thicker = delta(&["belt.thickness_m = 6.0e-3"]).thermal_values("belt_2").unwrap().0;
    close(thicker, 1.0 / (1.0422 + pulleys / 2.0), 1e-5, "half the conductance through twice the thickness");
}

/// By hand: at a 20 degree incline the cycle's segment[2] target (-0.1525, 0, -1.13) lies q = (-0.6025, 0, -1.13) from
/// column 0's joint line (through (0.45, 0, 0) along d = (-sin 20, 0, cos 20)): q . d = 0.206067 - 1.061853 =
/// -0.855786, so the rod reaches sqrt(1.639906 - 0.732370) = 0.952648 m across the column, beyond l sin 60 = 0.952628:
/// it would stand at 60.002 degrees, over the 60 its joints allow. The config is refused with the segment and that
/// reason, not as a value out of range.
#[test]
fn a_cycle_target_beyond_the_rods_angle_is_refused_with_the_segment_and_the_reason() {
    let (s, c) = 20f64.to_radians().sin_cos();
    let q = [-0.1525 - 0.45, -1.13];
    let along = q[0] * -s + q[1] * c;
    let reach = (q[0] * q[0] + q[1] * q[1] - along * along).sqrt();
    assert!((along + 0.855786).abs() < 1e-6 && (reach - 0.952648).abs() < 1e-6, "{along} {reach}");
    assert!(reach > 1.1 * 60f64.to_radians().sin());
    let mut cfg = ENTRY.config();
    cfg.set_text("geometry.column_incline_deg = 20.0").unwrap();
    let refused = Sim::new(&ENTRY, &cfg).err().expect("refused");
    assert_eq!((refused.key(), refused.detail()), ("error-delta-linear-rod-angle", "cycle.segment[2]"));
}

/// The suite's pulse takes the carriages into their stops (with the file's damper on, 5000 N s/m, it takes 15 to 17.6
/// mJ of the 21 to 25 mJ put in). Loss-free, on the stops' springs alone, the energy holds to 1e-8 of it: 2e-11 to 9e-11
/// J, the steps that meet a stop cut 16 times finer (`mech::STOP_SUBSTEPS`). Before 22 November 4e-9 to 3.1e-8 J, in
/// steps at each contact.
#[test]
fn the_carriages_meet_their_stops_with_the_energy_at_the_integrators_level() {
    for cfg in super::suite_configs() {
        let m = (super::ENTRY.build)(&cfg).unwrap();
        let (drift, injected) = sim_core::testkit::energy_drift(m.as_ref());
        println!("{drift} J of {injected} J");
        assert!(injected > 0.0 && drift < 1e-8 * injected, "{drift} J of {injected} J");
    }
}
