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

fn close(a: f64, b: f64, tolerance: f64, what: &str) {
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
