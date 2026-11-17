//! Tests of the crane's four further effects (rope mass, hook block, elastic drives, bridge skew: `full.rs`) and of the
//! floor's friction and the rigid rope on the floor. Hand values are worked in each test's comment.

use super::*;
use sim_components::Sim;
use sim_core::{Rk4, advance};

const DT: f64 = 62.5e-6;

fn lossless(edits: &[&str]) -> Config {
    let mut cfg = lossless_config();
    for e in edits {
        cfg.set_text(e).unwrap();
    }
    cfg
}

fn at_rest(m: &Crane, tool: [f64; 3]) -> (Vec<f64>, [f64; 3]) {
    let (mut state, mut hold) = (Vec::new(), [0.0; 3]);
    m.equilibrium(&tool, &mut state, &mut hold);
    (state, hold)
}

/// The load swung aside by (dx, dy) on the same rope length.
fn swing(m: &Crane, s: &mut [f64], dx: f64, dy: f64) {
    let length = m.rope(s).length;
    s[LOAD] += dx;
    s[LOAD + 1] += dy;
    s[LOAD + 2] = m.height - (length * length - dx * dx - dy * dy).sqrt();
}

/// The largest drift over `seconds` of the energy less the work of the constant holding torques on the motor angles (the
/// drives hold, and the losses are off): what the energy must keep.
fn drift(m: &Crane, state: &mut Vec<f64>, hold: &[f64; 3], seconds: f64) -> f64 {
    let mut rk = Rk4::new(m.state_len());
    let (mut pos, mut vel) = ([0.0; 3], [0.0; 3]);
    m.motor_state(state, &mut pos, &mut vel);
    let (p0, e0) = (pos, m.energy(state));
    let mut worst = 0.0f64;
    for step in 0..(seconds / DT) as usize {
        advance(m, &mut rk, state, hold, DT, 4);
        if step % 64 == 0 {
            m.motor_state(state, &mut pos, &mut vel);
            let work: f64 = (0..3).map(|k| hold[k] * (pos[k] - p0[k])).sum();
            worst = worst.max((m.energy(state) - work - e0).abs());
        }
    }
    assert!(state.iter().all(|v| v.is_finite()));
    worst
}

/// With the rope's mass per metre set to 0 the composite model is the plain one: the same accelerations to rounding, on
/// the shipped config (every loss, the rolling, the gearboxes' efficiency) at a swinging, moving state.
#[test]
fn the_composite_model_without_the_rope_s_mass_is_the_plain_model() {
    let plain = Crane::new(&ENTRY.config()).unwrap();
    let mut cfg = ENTRY.config();
    cfg.set_text("effects.rope_mass = true").unwrap();
    cfg.set_text("rope.mass_per_m_kg_m = 0.0").unwrap();
    let full = Crane::new(&cfg).unwrap();
    assert!(full.full && !plain.full);
    let (mut s, hold) = at_rest(&plain, [5.0, 2.0, 2.0]);
    swing(&plain, &mut s, 0.3, -0.2);
    s[LOAD_SPEED] = 0.2;
    s[LOAD_SPEED + 1] = -0.1;
    s[RATES] = 0.05;
    s[RATES + 1] = -0.03;
    s[RATES + 2] = 0.02;
    let torque = [hold[0] + 0.3, hold[1] - 0.2, hold[2] + 0.1];
    let (mut a, mut b) = (vec![0.0; STATE_LEN], vec![0.0; STATE_LEN]);
    plain.derivatives(&s, &torque, &mut a);
    full.derivatives(&s, &torque, &mut b);
    for i in 0..STATE_LEN {
        assert!((a[i] - b[i]).abs() < 1e-8 * (1.0 + a[i].abs()), "{i}: {} against {}", a[i], b[i]);
    }
    assert!((plain.energy(&s) - full.energy(&s)).abs() < 1e-8 * plain.energy(&s).abs());
}

/// effects.rope_mass at rest, by hand. Pose z = 2 m, the rope 3 m, hook and payload m = 265 kg, mu = 0.126 kg/m: the
/// rope's half weight hangs on the load, so the spring's tension is t = m g + mu g l / 2 = 2598.762 + 1.8532 N, and at
/// the drum it is the load's weight plus the whole rope's, (m + mu l) g = 2600.616 N (on a rope stiff enough that its
/// stretch e = t l / EA does not count: the exact drum pull is t (1 + e / 2l) + mu g L / 2). Nothing accelerates at the
/// rest the model finds, and the signal `rope.tension` is the drum's end.
#[test]
fn the_rope_s_tension_at_the_drum_is_the_load_plus_the_rope_s_weight() {
    let m = crane(&lossless(&["effects.rope_mass = true", "rope.ea_n = 1.0e10"]));
    let (s, hold) = at_rest(&m, [5.0, 2.0, 2.0]);
    let (mass, mu, length) = (m.load_mass(), m.rope_mass, s[2]);
    let drum = m.full_motion(&s, &hold).rope_state.drum;
    let hand = (mass + mu * length) * G;
    assert!((drum - hand).abs() < 5e-3, "{drum} N against {hand} N");
    let rope = m.rope(&s);
    let (t, e) = (mass * G + 0.5 * mu * G * length, rope.stretch);
    assert!(
        (rope.tension - t).abs() < 1e-3
            && (drum - (t * (1.0 + e / (2.0 * length)) + 0.5 * mu * G * rope.length)).abs() < 1e-4
    );
    let mut out = vec![0.0; m.state_len()];
    m.derivatives(&s, &hold, &mut out);
    assert!(out[RATES..RATES + 3].iter().chain(&out[LOAD_SPEED..LOAD_SPEED + 3]).all(|a| a.abs() < 1e-7), "{out:?}");
    let mut signals = Vec::new();
    m.base_signals(&s, &hold, &mut signals);
    assert!((signals[0] - hand).abs() < 5e-3, "{signals:?}");
}

fn crane(cfg: &Config) -> Crane {
    Crane::new(cfg).unwrap()
}

/// The small swing of a point load on a rope of mass mu per metre (the trolley and bridge held by their brakes): T = 2 pi
/// sqrt(l (m + mu l / 3) / (g (m + mu l / 2))), from K = (m + mu l / 3) l^2 theta'^2 / 2 and V = (m + mu l / 2) g l (1 -
/// cos theta). Hook only (m = 15 kg) on 3 m of a 1 kg/m rope: mu l = 3 kg, T = T_light sqrt(16 / 16.5), 1.5 % shorter.
#[test]
fn a_massive_rope_swings_with_the_closed_form_period() {
    let mut cfg = lossless(&["effects.rope_mass = true", "rope.mass_per_m_kg_m = 1.0", "effects.payload = false"]);
    cfg.set_text("effects.holding_brake = true").unwrap();
    let mut m = crane(&cfg);
    m.set_brake_engaged(true);
    let (mut state, hold) = at_rest(&m, [5.0, 2.0, 2.0]);
    let length = m.rope(&state).length;
    swing(&m, &mut state, 0.0, 0.01);
    let mut rk = Rk4::new(m.state_len());
    let (mut crossings, mut last) = (Vec::new(), state[LOAD + 1] - state[1]);
    for step in 1..=(10.0 / DT) as usize {
        advance(&m, &mut rk, &mut state, &hold, DT, 4);
        let sway = state[LOAD + 1] - state[1];
        if sway.signum() != last.signum() {
            crossings.push((step as f64 - sway / (sway - last)) * DT);
        }
        last = sway;
    }
    let n = crossings.len() - 1 - (crossings.len() - 1) % 2;
    let period = 2.0 * (crossings[n] - crossings[0]) / n as f64;
    let (mass, rope) = (m.load_mass(), m.rope_mass * length);
    let hand = TAU * (length * (mass + rope / 3.0) / (G * (mass + rope / 2.0))).sqrt();
    assert!((period / hand - 1.0).abs() < 5e-4, "{period} s against {hand} s");
    // `Machine::resonance_hz` (the shaper's period) says the same.
    let hz = m.resonance_hz(&[5.0, 2.0, m.height - length]).unwrap();
    assert!((1.0 / hz / hand - 1.0).abs() < 1e-9);
}

/// The small-swing frequencies of the hook block (m1 = 15 kg on the rope of length l1) and the load (m2 = 250 kg on a
/// sling l2 = 1 m), the trolley held, against the planar double pendulum. Its angles theta1 (rope) and theta2 (sling) from
/// the vertical obey, for small angles, (m1 + m2) l1 theta1'' + m2 l2 theta2'' + (m1 + m2) g theta1 = 0 and l1 theta1'' +
/// l2 theta2'' + g theta2 = 0. With theta'' = -w^2 theta the second gives theta2 / theta1 = l1 w^2 / (g - l2 w^2), and the
/// determinant (m1 + m2)(g - l1 w^2)(g - l2 w^2) - m2 l1 l2 w^4 = 0 is m1 l1 l2 w^4 - (m1 + m2) g (l1 + l2) w^2 +
/// (m1 + m2) g^2 = 0 (for m2 -> 0: w^2 = g / l1 and g / l2). Each root is excited alone in its own shape and its period read
/// from the zero crossings of the hook block's sway.
#[test]
fn the_hook_block_and_its_load_swing_at_the_double_pendulum_s_two_frequencies() {
    let mut cfg = lossless(&["effects.hook_block = true"]);
    cfg.set_text("effects.holding_brake = true").unwrap();
    let a0 = crane(&cfg).layout.sling.unwrap();
    for upper in [false, true] {
        let mut m = crane(&cfg);
        m.set_brake_engaged(true);
        let (mut state, hold) = at_rest(&m, [5.0, 2.0, 2.0]);
        let (l1, l2) = (m.rope(&state).length, m.sling.unwrap());
        let (m1, m2) = (m.hook, m.payload);
        let b = (m1 + m2) * G * (l1 + l2);
        let root = (b + if upper { 1.0 } else { -1.0 } * (b * b - 4.0 * m1 * l1 * l2 * (m1 + m2) * G * G).sqrt())
            / (2.0 * m1 * l1 * l2);
        let ratio = l1 * root / (G - l2 * root);
        let theta1 = 0.01f64;
        // The rope turned by theta1 about the sheave, the sling by theta2 = ratio theta1 (to the y direction).
        state[LOAD + 1] = state[1] + l1 * theta1.sin();
        state[LOAD + 2] = m.height - l1 * theta1.cos();
        state[a0 + 1] = ratio * theta1;
        let mut rk = Rk4::new(m.state_len());
        let (mut crossings, mut last) = (Vec::new(), state[LOAD + 1] - state[1]);
        for step in 1..=(9.0 / DT) as usize {
            advance(&m, &mut rk, &mut state, &hold, DT, 4);
            let sway = state[LOAD + 1] - state[1];
            if sway.signum() != last.signum() {
                crossings.push((step as f64 - sway / (sway - last)) * DT);
            }
            last = sway;
        }
        let n = crossings.len() - 1 - (crossings.len() - 1) % 2;
        let period = 2.0 * (crossings[n] - crossings[0]) / n as f64;
        let hand = TAU / root.sqrt();
        assert!((period / hand - 1.0).abs() < 1e-3, "upper {upper}: {period} s against {hand} s");
    }
}

/// Energy: with the drives held and the losses off, kinetic + potential + elastic less the holding torques' work on the
/// motor angles stays put, for each new effect and for all together (the sling and the rope swinging, the rotors twisted,
/// the girder yawed with the trolley off its middle).
#[test]
fn the_new_effects_conserve_energy_with_the_drives_held() {
    let cases: [(&[&str], [f64; 3]); 5] = [
        (&["effects.rope_mass = true", "rope.mass_per_m_kg_m = 0.5"], [5.0, 2.0, 2.0]),
        (&["effects.hook_block = true"], [5.0, 2.0, 2.0]),
        (&["effects.joint_elasticity = true", "gearbox.damping_nm_s_rad = 0.0"], [5.0, 2.0, 2.0]),
        (&["effects.bridge_skew = true"], [5.0, 4.0, 2.0]),
        (
            &[
                "effects.rope_mass = true",
                "effects.hook_block = true",
                "effects.joint_elasticity = true",
                "effects.bridge_skew = true",
                "gearbox.damping_nm_s_rad = 0.0",
            ],
            [5.0, 4.0, 2.0],
        ),
    ];
    for (edits, tool) in cases {
        let m = crane(&lossless(edits));
        let (mut state, hold) = at_rest(&m, tool);
        swing(&m, &mut state, 0.3, -0.2);
        state[LOAD_SPEED] = 0.1;
        if let Some(a0) = m.layout.sling {
            state[a0] = 0.3;
            state[a0 + 1] = -0.2;
            state[a0 + 2] = 0.1;
        }
        if let Some(r0) = m.layout.rotor {
            state[r0 + 2] += 2e-3;
            state[r0 + 1] -= 1e-3;
        }
        if let Some(k) = m.layout.skew {
            state[k] = 2e-4;
        }
        let worst = drift(&m, &mut state, &hold, 1.5);
        println!("{edits:?}: energy drift {worst:.2e} J");
        assert!(worst < 2e-3, "{edits:?}: {worst} J");
    }
}

/// Each elastic drive's mode, the rotor against the load it moves with the rope held (the rope's stiffness raised so
/// that the hook stays with the drum, the payload off, the gearbox damping off): w^2 = k (1 / J + 1 / M), J = i^2 J_rotor
/// the rotor's inertia at the axis, k the gearbox's stiffness at the axis (2e4 N m/rad over the lever squared), M the mass
/// the axis moves: bridge + trolley, trolley, and for the hoist the drum's J / r^2 and the hook. By hand for the bridge: i
/// = 30 / 0.08 = 375 rad/m, J = 1.34e-4 x 375^2 = 18.84 kg, k = 2e4 / 0.08^2 = 3.125e6 N/m, M = 720 kg: w = sqrt(3.125e6
/// (1/18.84 + 1/720)) = 412.9 rad/s, 65.7 Hz. The twist of the rotor over the axis is followed through ten of its periods.
#[test]
fn each_elastic_drive_rings_at_the_hand_calculated_frequency() {
    for d in 0..3 {
        let m = crane(&lossless(&[
            "effects.joint_elasticity = true",
            "gearbox.damping_nm_s_rad = 0.0",
            "effects.payload = false",
            "rope.ea_n = 1.0e11",
        ]));
        let j = &m.drives[d];
        let (ratio, r0) = (j.ratio(), m.layout.rotor.unwrap());
        let axis = [m.bridge + m.trolley, m.trolley, m.masses[2] + m.hook][d];
        let hand = (j.gearbox.stiffness * (1.0 / (j.rotor_inertia * ratio * ratio) + 1.0 / axis)).sqrt() / TAU;
        let (mut state, hold) = at_rest(&m, [5.0, 2.0, 2.0]);
        let twist = |s: &[f64], m: &Crane| s[r0 + d] / ratio - m.travel(s, d).0;
        let twist0 = twist(&state, &m);
        state[r0 + d] += 2e-3;
        let mut rk = Rk4::new(m.state_len());
        let (mut crossings, mut last) = (Vec::new(), 2e-3 / ratio);
        for step in 1..=(0.3 / DT) as usize {
            advance(&m, &mut rk, &mut state, &hold, DT, 4);
            let x = twist(&state, &m) - twist0;
            if x.signum() != last.signum() {
                crossings.push((step as f64 - x / (x - last)) * DT);
            }
            last = x;
        }
        let n = crossings.len() - 1 - (crossings.len() - 1) % 2;
        let measured = 1.0 / (2.0 * (crossings[n] - crossings[0]) / n as f64);
        assert!((measured / hand - 1.0).abs() < 2e-3, "drive {d}: {measured} Hz against {hand} Hz");
    }
}

/// The bridge on two carriages, the girder 5.6 m between its rails (the trolley's middle at 2.5 m), each carriage taking
/// half the bridge's command T through its own drive (force F / 2 = T i / 2 each). By hand with the rotors' inertia at the
/// axis R = i^2 J = 375^2 x 1.34e-4 = 18.84 kg per carriage, the girder's yaw inertia J_b = m_b g^2 / 12, the trolley m_t
/// at the distance d from the middle (x_t = x_c + psi d, so its acceleration a + d psi''):
///   (m_b + m_t + 2R) a + m_t d psi'' = F,      m_t d a + (J_b + m_t d^2 + R g^2 / 2) psi'' = (F_B - F_A) g / 2 = 0.
/// (a) the trolley in the middle (d = 0): psi'' = 0 to rounding. (b) at d = 2 m: psi'' = -F m_t d / det with det = A C -
/// B^2, A = m_b + m_t + 2R, B = m_t d, C = J_b + m_t d^2 + R g^2 / 2. (c) The wheels hold a steady skew where psi'' = 0
/// under a steady acceleration a = F / A: psi_s = -m_t d a / k_psi, k_psi = k_lat b^2 / 2 = 2e6 x 2^2 / 2 = 4e6 N m/rad.
#[test]
fn the_skew_follows_the_moment_balance_of_the_trolley_s_place() {
    let m = crane(&lossless(&["effects.bridge_skew = true"]));
    let k = m.layout.skew.unwrap();
    let t_cmd = 1.0;
    let ratio = m.drives[0].ratio();
    let r = ratio * ratio * m.drives[0].rotor_inertia;
    let (gauge, f) = (5.6, t_cmd * ratio);
    let (jb, mt, mb) = (m.bridge * gauge * gauge / 12.0, m.trolley, m.bridge);
    let run = |y: f64, psi: f64| {
        let (mut s, mut hold) = at_rest(&m, [5.0, y, 2.0]);
        s[k] = psi;
        hold[0] += t_cmd;
        let mut out = vec![0.0; m.state_len()];
        m.derivatives(&s, &hold, &mut out);
        (out[RATES], out[k + 1])
    };
    let (a, alpha) = run(2.5, 0.0);
    assert!(alpha.abs() < 1e-12 && (a - f / (mb + mt + 2.0 * r)).abs() < 1e-9, "{a} {alpha}");
    let d = 2.0;
    let (am, bm, cm) = (mb + mt + 2.0 * r, mt * d, jb + mt * d * d + 0.5 * r * gauge * gauge);
    let det = am * cm - bm * bm;
    let (a, alpha) = run(2.5 + d, 0.0);
    assert!((a / (f * cm / det) - 1.0).abs() < 1e-9 && (alpha / (-f * bm / det) - 1.0).abs() < 1e-9, "{a} {alpha}");
    let stiffness = 0.5 * 2.0e6 * 2.0 * 2.0;
    let steady = f / am;
    let psi = -mt * d * steady / stiffness;
    let (_, alpha_s) = run(2.5 + d, psi);
    assert!(alpha_s.abs() < 1e-3 * alpha.abs(), "{alpha_s} against {alpha}");
    // The two sides' travels and torques are signals; the skew angle is their difference over the gauge.
    let (mut s, hold) = at_rest(&m, [5.0, 2.5, 2.0]);
    s[k] = 1e-3;
    let mut signals = Vec::new();
    m.base_signals(&s, &hold, &mut signals);
    let n = signals.len();
    let (angle, a_travel, b_travel) = (signals[n - 5], signals[n - 4], signals[n - 3]);
    assert!((angle - 1e-3).abs() < 1e-15 && ((b_travel - a_travel) / gauge - 1e-3).abs() < 1e-12);
}

fn floor_crane(extra: &[&str]) -> Crane {
    let mut edits = vec!["effects.floor_contact = true", "effects.rope_damping = false"];
    edits.extend(extra);
    let mut cfg = ENTRY.config();
    for e in edits {
        cfg.set_text(e).unwrap();
    }
    crane(&cfg)
}

/// A load standing on the floor (0.5 m) 3 m aside of the sheave (8, 4, 5), the rope's free length 2 mm short of its
/// 5.4083 m: t = EA e / l, u = (3, 0, -4.5) / L, so the rope lifts t |u_z| and the floor carries N = m g - t |u_z|
/// (m g = 2598.762 N). Sliding at 1 m/s along +x (tanh(1 / 0.005) = 1) the friction is -0.3 N against the motion: a_x =
/// (-t u_x - 0.3 N) / m, and it takes mu N v of power; at rest it is none and a_x = -t u_x / m.
#[test]
fn the_standing_load_rubs_on_the_floor_with_coulomb_friction() {
    let m = floor_crane(&[]);
    let (mut s, hold) = at_rest(&m, [8.0, 4.0, 1.0]);
    s[LOAD] = 11.0;
    s[LOAD + 2] = 0.5;
    let length = (9.0f64 + 4.5 * 4.5).sqrt();
    s[2] = length - 0.002;
    let weight = m.load_mass() * G;
    let t = 1.44e6 * 0.002 / s[2];
    let (ux, uz) = (3.0 / length, -4.5 / length);
    let n = weight + t * uz;
    let moving = {
        let mut v = s.clone();
        v[LOAD_SPEED] = 1.0;
        m.motion(&v, &hold)
    };
    let want = (-t * ux - 0.3 * n) / m.load_mass();
    assert!((moving.floor_force - n).abs() < 1e-6 && (moving.load_acc[0] - want).abs() < 1e-9, "{:?}", moving.load_acc);
    assert!((moving.floor_power - 0.3 * n).abs() < 1e-6 && moving.load_acc[2] == 0.0);
    let resting = m.motion(&s, &hold);
    assert!((resting.load_acc[0] + t * ux / m.load_mass()).abs() < 1e-9 && resting.floor_power == 0.0);
    // No friction coefficient: the smooth floor of before.
    let smooth = floor_crane(&["crane.floor_friction_coefficient = 0.0"]);
    let mut v = s.clone();
    v[LOAD_SPEED] = 1.0;
    assert!((smooth.motion(&v, &hold).load_acc[0] + t * ux / m.load_mass()).abs() < 1e-9);
}

/// The rigid rope respects the floor. The load on it (0.5 m) under the sheave (5 m) with the drum paying out a free length
/// of 5.2 m: the rope hangs slack, nothing pulls, the floor carries all of m g and the load does not move. With the drum
/// at 4.5 m holding the weight, the rope taut and vertical, it is the drum that carries it and the floor nothing. And a
/// rope that comes taut (the drum 4.0 m, the load 4.5 m away) lifts the load to its length at once (`after_step`).
#[test]
fn the_rigid_rope_stands_the_load_on_the_floor() {
    let mut m = floor_crane(&["effects.rope_stretch = false"]);
    let weight = m.load_mass() * G;
    let (mut s, hold) = at_rest(&m, [8.0, 4.0, 0.5]);
    assert!(s[LOAD + 2] == 0.5 && s[2] == 4.5);
    let taut = m.motion(&s, &hold);
    assert!(taut.floor_force.abs() < 1e-6 && (taut.rope_state.tension - weight).abs() < 1e-6, "{}", taut.floor_force);
    s[2] = 5.2;
    let slack = m.motion(&s, &hold);
    assert!(slack.rope_state.tension == 0.0 && (slack.floor_force - weight).abs() < 1e-9);
    assert!(slack.load_acc == [0.0; 3], "{:?}", slack.load_acc);
    s[2] = 4.0;
    m.after_step(&mut s, 1e-3);
    assert!((s[LOAD + 2] - 1.0).abs() < 1e-12 && s[LOAD] == 8.0, "{s:?}");
}

/// Every new effect builds and runs: the shipped cycle for a moment with each of them and with all together (and the
/// floor), finite, with the signal list as long as the samples.
#[test]
fn the_new_effects_run_the_shipped_cycle_without_a_hitch() {
    let cases: [&[&str]; 5] = [
        &["effects.rope_mass = true"],
        &["effects.hook_block = true"],
        &["effects.joint_elasticity = true"],
        &["effects.bridge_skew = true"],
        &[
            "effects.rope_mass = true",
            "effects.hook_block = true",
            "effects.joint_elasticity = true",
            "effects.bridge_skew = true",
            "effects.floor_contact = true",
        ],
    ];
    for edits in cases {
        let mut cfg = ENTRY.config();
        for e in edits {
            cfg.set_text(e).unwrap();
        }
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        let mut sample = Vec::new();
        sim.run(0.2, |s| {
            s.sample(&mut sample);
            assert!(sample.iter().all(|v| v.is_finite()), "{edits:?}");
        });
        assert!(sim.state.iter().all(|v| v.is_finite()), "{edits:?}");
    }
    // The new effects need the elastic rope.
    let mut cfg = ENTRY.config();
    cfg.set_text("effects.rope_stretch = false").unwrap();
    cfg.set_text("effects.rope_mass = true").unwrap();
    assert!(Crane::new(&cfg).is_err());
}
