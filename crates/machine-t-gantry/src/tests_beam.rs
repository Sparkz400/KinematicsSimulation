//! The beam's vertical mode (effects.beam_vertical_mode), the belt runs' geometric stiffness and the process force's vertical
//! part on the leaning axis.

use super::*;

fn machine(mut cfg: Config, edit: &[&str]) -> TGantry {
    for e in edit {
        cfg.set_text(e).unwrap();
    }
    TGantry::new(&cfg).unwrap()
}

/// The beam's modal mass (`Beam::modal_mass`) is mu times the integral of the static shape squared, by brute force: the
/// shape of a simple span under a point load at a (b = L - a), taken to 1 under it, phi = s (L^2 - b^2 - s^2) / (2 a^2 b)
/// left of the load and s' (L^2 - a^2 - s'^2) / (2 a b^2) right of it (s' = L - s), cut into 200 000 pieces. In the middle
/// of the 1.3 m span it is 17 mu L / 35 = 17 x 3.5 x 1.3 / 35 = 2.2100 kg.
#[test]
fn the_beams_modal_mass_is_the_integral_of_its_shape_squared() {
    let m = machine(lossless_config(), &["effects.beam_vertical_mode = true"]);
    let (beam, mode) = m.vertical().unwrap();
    let (mu, l) = (mode.mass_per_m, beam.span);
    assert!((beam.modal_mass(mu, 0.5) - 17.0 / 35.0 * mu * l).abs() < 1e-12);
    assert!((17.0 / 35.0 * 3.5 * 1.3_f64 - 2.21).abs() < 1e-12);
    for x in [0.2, 0.5, 0.9] {
        let (a, pieces) = (0.15 + x, 200_000);
        let b = l - a;
        let sum: f64 = (0..pieces)
            .map(|i| {
                let s = (i as f64 + 0.5) / pieces as f64 * l;
                let phi = if s < a {
                    s * (l * l - b * b - s * s) / (2.0 * a * a * b)
                } else {
                    (l - s) * (l * l - a * a - (l - s) * (l - s)) / (2.0 * a * b * b)
                };
                mu * phi * phi * l / pieces as f64
            })
            .sum();
        let got = beam.modal_mass(mu, x);
        assert!((got - sum).abs() < 1e-6 * sum, "x = {x}: {got} against the sum {sum}");
    }
}

/// The beam's mode (effects.beam_vertical_mode) against the hand value. The belt stiff (EA = 1e11 N) and the frame and
/// the lean off, the carriage and the Z axis cannot move on the beam or against it and the lowest mode is the beam's
/// bending with them on it: f = sqrt(k / (m_c + m_z + M)) / 2 pi, in the middle (x = 0.5 m, 0.65 m from a post of the 1.3 m
/// span) k = 48 E I / L^3 = 48 x 8.4e4 / 1.3^3 = 1.83523e6 N/m, M = 17 mu L / 35 = 2.2100 kg, m_c + m_z = 4 + 7 kg:
/// 1.83523e6 / 13.2100 = 138 928 1/s^2, 372.73 rad/s, 59.32 Hz. (The old comment's 100 Hz had no carriage mass: the 11 kg
/// of carriage and Z axis make it 59 Hz in the middle, higher towards the posts.) Its damping ratio is the key's.
#[test]
fn the_beams_vertical_mode_is_where_the_hand_value_has_it() {
    let m = machine(
        lossless_config(),
        &[
            "effects.beam_vertical_mode = true",
            "beam.damping_ratio = 0.01",
            "effects.frame_compliance = false",
            "effects.tool_flex = false",
            "effects.gearbox_backlash = false",
            "belt.stiffness_ea_n = 1.0e11",
        ],
    );
    let modes = m.modal(&[0.5, 0.25]);
    let by_hand = (48.0 * 8.4e4 / 1.3f64.powi(3) / (4.0 + 7.0 + 17.0 / 35.0 * 3.5 * 1.3)).sqrt() / std::f64::consts::TAU;
    assert!((by_hand - 59.32).abs() < 0.01, "{by_hand}");
    let (mode, shape) = &modes[0];
    assert!((mode.hz - by_hand).abs() < 1e-3 * by_hand, "{} Hz against {by_hand} Hz", mode.hz);
    assert!((mode.damping - 0.01).abs() < 1e-3, "damping ratio {}", mode.damping);
    assert!(shape[W].abs() > 0.99 * shape.iter().fold(0.0, |a: f64, b| a.max(b.abs())), "{shape:?}");
    // Nearer a post the beam is stiffer: 0.15 m from it (x = 0) the mode is higher.
    assert!(m.modal(&[0.0, 0.25])[0].0.hz > mode.hz);
}

/// The mass matrix of the beam's mode: w carries the carriage, the Z axis with its payload (4 + 7 kg) and the modal mass
/// 17 mu L / 35 = 2.21 kg, and the Z axis's z is relative to the carriage, so its speed over ground z' - w' puts -m_z = -7 kg
/// between z and w. With the frame and the lean off the coordinates are (x, z, w). A push on Z then moves the beam the
/// other way, by about 7 / 13.21 of the Z axis's own acceleration.
#[test]
fn the_z_axis_acceleration_drives_the_beam_through_its_mass() {
    let m = machine(
        lossless_config(),
        &["effects.beam_vertical_mode = true", "effects.frame_compliance = false", "effects.tool_flex = false"],
    );
    assert_eq!(m.coordinates(), vec![X, Z, W]);
    let mass = m.elastic_mass(0.5, 0.25);
    assert!((mass[5] + 7.0).abs() < 1e-12 && (mass[7] + 7.0).abs() < 1e-12, "{mass:?}");
    assert!((mass[8] - (11.0 + 2.21)).abs() < 1e-9, "{}", mass[8]);
    let (mut state, mut torque) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&[0.5, 0.25], &mut state, &mut torque);
    let mut rest = vec![0.0; state.len()];
    m.derivatives(&state, &torque, &mut rest);
    assert!(rest[V + W].abs() < 1e-6, "at rest the beam accelerates at {} m/s^2", rest[V + W]);
    torque[0] += 0.05;
    torque[1] -= 0.05;
    let mut pushed = vec![0.0; state.len()];
    m.derivatives(&state, &torque, &mut pushed);
    let (az, aw) = (pushed[V + Z] - rest[V + Z], pushed[V + W] - rest[V + W]);
    assert!(az > 0.0 && aw < 0.0, "{az} {aw}");
    // The two accelerations share the push: M_zz az + M_zw aw = F, M_zw az + M_ww aw = 0 (the belt's own spring does not
    // change in that instant), so aw / az = -M_zw / M_ww = 7 / 13.21.
    let ratio = -aw / az;
    assert!((ratio - 7.0 / 13.21).abs() < 0.02, "the beam moves {ratio} of the Z axis's push against it, by hand 7 / 13.21");
}

/// The beam at rest at the quasi-static sag: with effects.beam_vertical_mode the state's w is the sag m g a^2 b^2 / (3 E I L)
/// (58.78 um in the middle, `the_beam_sags_under_a_central_carriage_by_f_l3_over_48_ei`), the Z axis stands that much
/// higher on its belt, the tool is at its pose and nothing accelerates; the stiffness is the quasi-static span's,
/// k sag = (m_c + m_z) g.
#[test]
fn the_vertical_mode_rests_at_the_quasi_static_sag() {
    let m = machine(ENTRY.config(), &["effects.beam_vertical_mode = true"]);
    let quasi = machine(ENTRY.config(), &["effects.beam_sag = true"]);
    let (mut state, mut torque, mut other, mut other_torque) = (Vec::new(), [0.0; 2], Vec::new(), [0.0; 2]);
    for pose in [[0.5, 0.25], [0.0, 0.25], [0.8, 0.1]] {
        m.equilibrium(&pose, &mut state, &mut torque);
        quasi.equilibrium(&pose, &mut other, &mut other_torque);
        let sag = quasi.sag(pose[0]).0;
        assert!((state[W] - sag).abs() < 1e-15 && (state[Z] - pose[1] - sag).abs() < 1e-12, "{pose:?}: w {}", state[W]);
        let k = m.beam_mode(pose[0])[0];
        assert!((k * state[W] - 11.0 * G).abs() < 1e-6, "k w = {} N", k * state[W]);
        assert!((state[X] - other[X]).abs() < 1e-15 && (state[Z] - other[Z]).abs() < 1e-15);
        assert!((torque[0] - other_torque[0]).abs() < 1e-12 && (torque[1] - other_torque[1]).abs() < 1e-12);
        let (mut pos, mut vel) = ([0.0; 2], [0.0; 2]);
        m.tool_state(&state, &mut pos, &mut vel);
        assert!((pos[1] - pose[1]).abs() < 1e-12, "the tool stands at {} m", pos[1]);
        let mut rates = vec![0.0; state.len()];
        m.derivatives(&state, &torque, &mut rates);
        assert!(rates[V + Z].abs() < 1e-3 && rates[V + W].abs() < 1e-6, "{} {}", rates[V + Z], rates[V + W]);
    }
}

/// The beam mode's damping row: with the mode lightly damped (zeta = 0.05) and moving at w' = 0.05 m/s, the energy rises by
/// the motors' power less what `beam_loss` books, c w'^2 with c = 2 zeta sqrt(k M) (the other losses are off).
#[test]
fn the_beams_damping_is_booked_as_its_loss() {
    let m = machine(lossless_config(), &["effects.beam_vertical_mode = true", "beam.damping_ratio = 0.05"]);
    let (mut s, mut hold) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&[0.3, 0.1], &mut s, &mut hold);
    s[V + W] = 0.05;
    let torque = [hold[0] + 0.7, hold[1] - 0.3];
    let mut rate = vec![0.0; s.len()];
    m.derivatives(&s, &torque, &mut rate);
    let energy = |h: f64| m.energy(&s.iter().zip(&rate).map(|(x, r)| x + h * r).collect::<Vec<_>>());
    let h = 1e-7;
    let rate_of_energy = (8.0 * (energy(h) - energy(-h)) - (energy(2.0 * h) - energy(-2.0 * h))) / (12.0 * h);
    let (mut angle, mut speed) = ([0.0; 2], [0.0; 2]);
    m.motor_state(&s, &mut angle, &mut speed);
    let power = torque[0] * speed[0] + torque[1] * speed[1];
    let [k, mass, c, ..] = m.beam_mode(0.3);
    assert!((c - 2.0 * 0.05 * (k * mass).sqrt()).abs() < 1e-9 * c);
    let loss = m.beam_loss(&s);
    assert!((loss - c * 0.05 * 0.05).abs() < 1e-12 && loss > 0.1, "{loss} W");
    assert!(
        (rate_of_energy - power + loss).abs() < 1e-6 * (loss + power.abs()),
        "dE/dt {rate_of_energy} W, power {power} W, beam loss {loss} W"
    );
}

/// The modes follow the dynamics with the beam's mode in (and the belt runs' geometric stiffness, always): at rest at the
/// pose the derivatives' accelerations per unit of each coordinate (central differences over 1e-7) are -M^-1 K, and each
/// mode's shape satisfies K v = w^2 M v. With the beam sagged to its static w0 the cross terms K_xw = k' w0 and
/// K_xx = k'' w0^2 / 2 are in it. The mass matrix is taken at the state's z (the pose's with the sag).
#[test]
fn the_modes_with_the_beam_follow_the_linearised_derivatives() {
    let m = machine(
        lossless_config(),
        &["effects.beam_vertical_mode = true", "effects.gearbox_backlash = false", "beam.mass_kg_m = 12.0"],
    );
    for pose in [[0.9, 0.05], [0.2, 0.25]] {
        let (mut rest, mut torque) = (Vec::new(), [0.0; 2]);
        m.equilibrium(&pose, &mut rest, &mut torque);
        let at = m.coordinates();
        assert!(at.contains(&W) && at.contains(&U));
        let n = at.len();
        let mut jacobian = vec![0.0; n * n];
        for (j, &c) in at.iter().enumerate() {
            let acc = |h: f64| {
                let mut s = rest.clone();
                s[c] += h;
                let mut d = vec![0.0; s.len()];
                m.derivatives(&s, &torque, &mut d);
                at.iter().map(|&i| d[V + i]).collect::<Vec<_>>()
            };
            let (ahead, behind) = (acc(1e-7), acc(-1e-7));
            for i in 0..n {
                jacobian[i * n + j] = (ahead[i] - behind[i]) / 2e-7;
            }
        }
        let mass = m.elastic_mass(rest[X], rest[Z]);
        let k: Vec<f64> = (0..n * n)
            .map(|rc| -(0..n).map(|p| mass[rc / n * n + p] * jacobian[p * n + rc % n]).sum::<f64>())
            .collect();
        let modes = m.modal(&pose);
        assert!(modes.len() >= 4, "{pose:?}: {} modes", modes.len());
        for (mode, shape) in &modes {
            let v: Vec<f64> = at.iter().map(|&c| shape[c]).collect();
            let w2 = (std::f64::consts::TAU * mode.hz).powi(2);
            let times = |a: &[f64], i: usize| (0..n).map(|j| a[i * n + j] * v[j]).sum::<f64>();
            let (kv, mv): (Vec<f64>, Vec<f64>) = (0..n).map(|i| (times(&k, i), w2 * times(&mass, i))).unzip();
            let size = mv.iter().fold(0.0, |a: f64, b| a.max(b.abs()));
            assert!(
                kv.iter().zip(&mv).all(|(a, b)| (a - b).abs() < 1e-5 * size),
                "{pose:?} {} Hz: K v {kv:?}, w^2 M v {mv:?}",
                mode.hz
            );
        }
    }
}

/// The belt runs' geometric stiffness. The clamp (s + z above the carriage's idlers) and the Z-bottom idler (s + sz - z
/// below them) stand out sideways by k u g with the lean u, g = 1 / lateral + h d / tilt at the signed distance d below the
/// carriage (the guide's tilt only, no bending); a run under tension T and of length l holds the end back with T delta / l,
/// so the belt's energy has K_uu = (T1 + T3) (k g_clamp)^2 / (s + z) + 2 T2 (k g_bottom)^2 / (s + sz - z) on top of the
/// lean's spring k and the weight's pendulum stiffness. By hand at z = 0.05 m (s = 0.15, sz = 0.5, h = 0.25 m, lateral
/// 2e7, tilt 1.2e4, d = -0.2 and 0.6): k = 1 / (5e-8 + 0.0625 / 1.2e4) = 190.2e3 N/m, k g_clamp = -0.7805, k g_bottom =
/// 2.3796, with the tensions of the rest state (section 2 higher by m_z g / 2). The energy's second derivative in u over
/// 1e-4 m.
#[test]
fn the_belt_runs_hold_the_lean_back_with_t_over_l() {
    let m = machine(lossless_config(), &["effects.frame_compliance = false", "effects.z_axis_bending = false"]);
    let z = 0.05;
    let (mut state, mut torque) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&[0.5, z], &mut state, &mut torque);
    let t = m.sections(&state).map(|f| f.tension);
    let (lateral, tilt, s, sz, h) = (2.0e7, 1.2e4, 0.15, 0.5, 0.3 - z);
    let k = 1.0 / (1.0 / lateral + h * h / tilt);
    let (g_clamp, g_bottom) = (k * (1.0 / lateral - h * (s + z) / tilt), k * (1.0 / lateral + h * (s + sz - z) / tilt));
    assert!((g_clamp + 0.7805).abs() < 1e-3 && (g_bottom - 2.3796).abs() < 1e-3, "{g_clamp} {g_bottom}");
    let by_hand = (t[0] + t[2]) * g_clamp * g_clamp / (s + z) + 2.0 * t[1] * g_bottom * g_bottom / (s + sz - z);
    assert!(by_hand > 1e3, "{by_hand} N/m");
    let energy = |u: f64| {
        let mut at = state.clone();
        at[U] = u;
        m.energy(&at)
    };
    let step = 1e-4;
    let second = (energy(step) + energy(-step) - 2.0 * energy(0.0)) / (step * step);
    let geometric = second - k - m.lean_weight_stiffness(z);
    assert!((geometric - by_hand).abs() < 1e-3 * by_hand, "{geometric} N/m against {by_hand} N/m");
    let runs = m.lean_runs(z);
    let from_runs = (0..3).map(|i| t[i] * runs[i]).sum::<f64>();
    assert!((from_runs - by_hand).abs() < 1e-9 * by_hand, "{from_runs} against {by_hand}");
}

/// The process force's vertical part on the leaning axis. The tip, d = z_tool_drop - z below the carriage, rises as the Z axis
/// leans under the spring force k u at its centre of mass (the guide's tilt only: the whole bar turns by theta = k u h /
/// tilt): by theta^2 d / 2, which is `lean_rise`'s 1/2 (h / tilt)^2 d per F^2. At z = 0.25 m (h = 0.05, d = 0.45, k = 1 / (5e-8
/// + 0.0025 / 1.2e4)) and u = 1 mm: theta = 0.01613, rise 5.85e-5 m. `tool_state` has it in the tool's height, and the
/// force's work on it is in the budget (`the_process_force_works_on_the_tool_it_moves`).
#[test]
fn the_leaning_tip_rises_by_half_theta_squared_times_its_distance() {
    let m = machine(lossless_config(), &["effects.frame_compliance = false", "effects.z_axis_bending = false"]);
    let (z, u) = (0.25, 1e-3);
    let (lateral, tilt, h, d) = (2.0e7_f64, 1.2e4_f64, 0.3 - z, 0.7 - z);
    let k = 1.0 / (1.0 / lateral + h * h / tilt);
    let theta = k * u * h / tilt;
    let by_hand = 0.5 * theta * theta * d;
    assert!((theta - 0.01613).abs() < 1e-4 && (by_hand - 5.85e-5).abs() < 1e-6, "{theta} {by_hand}");
    let a = m.rise_coefficients(z);
    assert!((a[0] * u * u - by_hand).abs() < 1e-9 * by_hand, "{} against {by_hand}", a[0] * u * u);
    let (mut state, mut torque) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&[0.5, z], &mut state, &mut torque);
    state[U] = u;
    let (mut pos, mut vel) = ([0.0; 2], [0.0; 2]);
    m.tool_state(&state, &mut pos, &mut vel);
    assert!((pos[1] - z - by_hand).abs() < 1e-9 * by_hand, "the tool is {} m higher", pos[1] - z);
    // Its rate: d/dt of a(z) u^2 at the lean's speed 0.2 m/s and the Z axis's 0.1 m/s.
    (state[V + U], state[V + Z]) = (0.2, 0.1);
    m.tool_state(&state, &mut pos, &mut vel);
    let rate = 2.0 * a[0] * u * 0.2 + a[1] * u * u * 0.1;
    assert!((vel[1] - 0.1 - rate).abs() < 1e-9 * rate.abs(), "{} against {}", vel[1] - 0.1, rate);
}

/// The process force does work on the tool as it moves (tip and beam included): the loss-free machine's energy rises by the
/// motors' power plus the force times the tool's velocity, F_x x' + F_z z', the tool's velocity being the rate of
/// `tool_state` along the derivatives (central difference over 1e-7). The force of (400, 300) N on the leaning Z axis, with
/// and without the beam's mode; this carries the work on the tip's rise and on the beam's deflection into the dynamics.
#[test]
fn the_process_force_works_on_the_tool_it_moves() {
    let plain = ["effects.z_axis_bending = true"];
    let beam = ["effects.z_axis_bending = true", "effects.beam_vertical_mode = true"];
    for edit in [&plain[..], &beam[..]] {
        let mut m = machine(lossless_config(), edit);
        m.dwelling = true;
        m.process = [400.0, 300.0];
        for (pose, vel) in [([0.3, 0.1], [0.6, 0.2]), ([0.8, 0.4], [-0.5, 0.3])] {
            let (mut s, mut hold) = (Vec::new(), [0.0; 2]);
            m.equilibrium(&pose, &mut s, &mut hold);
            m.set_tool_velocity(&mut s, &vel);
            for (i, x) in s.iter_mut().enumerate() {
                *x += 1e-4 * (1.0 + (i % 5) as f64);
            }
            let torque = [hold[0] + 0.7, hold[1] - 0.3];
            let mut rate = vec![0.0; s.len()];
            m.derivatives(&s, &torque, &mut rate);
            let along = |h: f64| s.iter().zip(&rate).map(|(x, r)| x + h * r).collect::<Vec<_>>();
            let h = 1e-7;
            let energy = |h: f64| m.energy(&along(h));
            let rate_of_energy = (8.0 * (energy(h) - energy(-h)) - (energy(2.0 * h) - energy(-2.0 * h))) / (12.0 * h);
            let tool = |h: f64| {
                let (mut pos, mut v) = ([0.0; 2], [0.0; 2]);
                m.tool_state(&along(h), &mut pos, &mut v);
                pos
            };
            let (ahead, behind) = (tool(h), tool(-h));
            let tool_speed = [(ahead[0] - behind[0]) / (2.0 * h), (ahead[1] - behind[1]) / (2.0 * h)];
            let (mut angle, mut speed) = ([0.0; 2], [0.0; 2]);
            m.motor_state(&s, &mut angle, &mut speed);
            let power = torque[0] * speed[0] + torque[1] * speed[1] + 400.0 * tool_speed[0] + 300.0 * tool_speed[1];
            // The tool's reported velocity is the rate of its position.
            let (mut pos, mut v) = ([0.0; 2], [0.0; 2]);
            m.tool_state(&s, &mut pos, &mut v);
            assert!((v[1] - tool_speed[1]).abs() < 1e-6, "{edit:?}: {} against {}", v[1], tool_speed[1]);
            assert!(
                (rate_of_energy - power).abs() < 1e-6 * power.abs().max(1.0),
                "{edit:?} at {pose:?}: dE/dt {rate_of_energy} W, motors and process {power} W"
            );
        }
    }
}
