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
fn drift(m: &Crane, state: &mut [f64], hold: &[f64; 3], seconds: f64) -> f64 {
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
/// the shipped config (every loss, the rolling, the gearboxes' efficiency) at a swinging, moving state; on the elastic
/// rope and on the rigid one (its tension the constraint's multiplier, the rolling at it to the fixed point).
#[test]
fn the_composite_model_without_the_rope_s_mass_is_the_plain_model() {
    for elastic in [true, false] {
        composite_against_plain(elastic);
    }
}

fn composite_against_plain(elastic: bool) {
    let mut cfg = ENTRY.config();
    cfg.set_text(&format!("effects.rope_stretch = {elastic}")).unwrap();
    let plain = Crane::new(&cfg).unwrap();
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
        assert!((a[i] - b[i]).abs() < 1e-8 * (1.0 + a[i].abs()), "{elastic} {i}: {} against {}", a[i], b[i]);
    }
    assert!((plain.energy(&s) - full.energy(&s)).abs() < 1e-8 * plain.energy(&s).abs());
    let (mut x, mut y) = (Vec::new(), Vec::new());
    plain.base_signals(&s, &torque, &mut x);
    full.base_signals(&s, &torque, &mut y);
    assert!(x.iter().zip(&y).all(|(x, y)| (x - y).abs() < 1e-8 * (1.0 + x.abs())), "{x:?} against {y:?}");
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
    assert!((drum - hand).abs() < 1e-3, "{drum} N against {hand} N");
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
    assert!((signals[0] - hand).abs() < 1e-3, "{signals:?}");
    // The rigid rope: the same at its constraint's tension, without the stretch.
    let m = crane(&lossless(&["effects.rope_mass = true", "effects.rope_stretch = false"]));
    let (s, hold) = at_rest(&m, [5.0, 2.0, 2.0]);
    let motion = m.full_motion(&s, &hold);
    let (hand, t) = ((mass + mu * s[2]) * G, (mass + 0.5 * mu * s[2]) * G);
    assert!(
        (motion.rope_state.drum - hand).abs() < 1e-9 && (motion.rope_state.tension - t).abs() < 1e-9,
        "{} {} {} {}",
        motion.rope_state.drum,
        hand,
        motion.rope_state.tension,
        t
    );
    let mut out = vec![0.0; m.state_len()];
    m.derivatives(&s, &hold, &mut out);
    assert!(out.iter().all(|a| a.abs() < 1e-9), "{out:?}");
    signals.clear();
    m.base_signals(&s, &hold, &mut signals);
    assert!((signals[0] - hand).abs() < 1e-9, "{signals:?}");
}

/// The rope paid in along itself: all of it moves at the drum's speed, the free part straight up with the hook, the
/// wound part with the drum's surface. Gravity off (nothing hangs, so the hanging length's weight does not change as it
/// winds), the trolley and bridge still, a stiff rope (1e9 N) and a heavy one (mu = 5 kg/m, capacity 6 m), the hook
/// only: the drum winds under an extra -0.05 N m at the motor, the axis force F = -0.05 i (i = 40 / 0.08 = 500 rad/m),
/// with the mass at the drum M = J_drum / r^2 + i^2 J_rotor + mu capacity + m_hook = 7.8125 + 112.5 + 30 + 15 = 165.31 kg:
/// l(t) = l0 + F t^2 / (2 M). (A rope whose top moved with the sheave alone, as a linear velocity from v_S to v_P has it,
/// would count 2 mu l / 3 = 10 kg less, 6 % more travel.)
#[test]
fn a_rope_wound_in_moves_all_its_mass_with_the_drum() {
    let m = crane(&lossless(&[
        "effects.rope_mass = true",
        "rope.mass_per_m_kg_m = 5.0",
        "rope.ea_n = 1.0e9",
        "effects.gravity = false",
        "effects.payload = false",
    ]));
    let (mut state, mut hold) = at_rest(&m, [5.0, 2.0, 2.0]);
    let (l0, j) = (state[2], &m.drives[2]);
    hold[2] -= 0.05;
    let mass = m.masses[2] + j.reflected_inertia() + 5.0 * m.capacity + m.hook;
    let a = -0.05 * j.ratio() / mass;
    let mut rk = Rk4::new(m.state_len());
    let seconds = 0.1;
    for _ in 0..(seconds / DT).round() as usize {
        advance(&m, &mut rk, &mut state, &hold, DT, 4);
    }
    let (moved, hand) = (state[2] - l0, 0.5 * a * seconds * seconds);
    // Measured 3.3e-6 off: the stiff rope's ringing as the drum's force steps on.
    assert!((moved / hand - 1.0).abs() < 2e-5, "{moved} m against {hand} m");
}

/// A pick with elastic drives is as quasi-static as the rope's: the servo holds the hoist's rotor, the drum pays out by
/// the change of the gearbox's static twist, some m_p g / k = 250 x 9.80665 / 3.125e6 = 0.7845 mm (k = 2e4 N m/rad /
/// 0.08^2 at the drum; 0.7853 mm with the rope's stretch in the drum's pull), and the crane is then at rest under the new
/// load's holding torques: nothing left to ring.
#[test]
fn a_pick_with_elastic_drives_leaves_the_gearbox_at_its_new_twist() {
    let mut m = crane(&lossless(&["effects.joint_elasticity = true"]));
    m.attached = false;
    let (mut s, _) = at_rest(&m, [5.0, 2.0, 2.0]);
    let (l0, r0) = (s[2], m.layout.rotor.unwrap());
    let rotor = s[r0 + 2];
    m.set_payload_attached(true);
    m.after_step(&mut s, DT);
    // The drum's pull t (1 + e / 2l), e = t l / EA, from t0 = m_hook g to t1 = (m_hook + m_p) g.
    let (k, t0, t1) = (m.drives[2].gearbox.stiffness, m.hook * G, m.load_mass() * G);
    let hand = (t1 - t0 + (t1 * t1 - t0 * t0) / (2.0 * m.ea)) / k;
    assert!(s[r0 + 2] == rotor && ((s[2] - l0) / hand - 1.0).abs() < 1e-9, "{} m against {hand} m", s[2] - l0);
    let (_, hold) = at_rest(&m, [5.0, 2.0, 2.0]);
    let mut out = vec![0.0; m.state_len()];
    m.derivatives(&s, &hold, &mut out);
    assert!(out.iter().all(|a| a.abs() < 1e-6), "{out:?}");
}

/// A pick on a hook block caught swinging is a plastic attach: the payload hangs plumb under the hook block at its
/// speed (the sling at rest), so the pick can only lose energy (the rope's settling to its new stretch), never make it.
/// The sling turned into the rope's line at the rope's rate made it: with this swing (0.36 m aside, 0.22 m/s) the
/// payload's lift of l_s (1 - cos theta) and the faster turn at L + l_s instead of L gave the pick a loss of -16.4 J.
#[test]
fn a_pick_on_a_swinging_hook_block_creates_no_energy() {
    let mut m = crane(&lossless(&["effects.hook_block = true"]));
    m.attached = false;
    let (mut s, _) = at_rest(&m, [5.0, 2.0, 2.0]);
    swing(&m, &mut s, 0.3, -0.2);
    (s[LOAD_SPEED], s[LOAD_SPEED + 1]) = (0.2, 0.1);
    m.set_payload_attached(true);
    let before = m.energy(&s);
    m.after_step(&mut s, DT);
    let lost = before - m.energy(&s);
    assert!(lost >= 0.0 && (m.pick_loss * DT - lost).abs() < 1e-9, "{lost} J lost, {} J booked", m.pick_loss * DT);
    // At its rest stretch under the payload, l_s (1 + m_p g / EA).
    let ((low, low_v), ls) = (m.lower(&s), m.sling.unwrap() * (1.0 + m.payload * G / m.ea));
    let (p1, v1) = ([s[LOAD], s[LOAD + 1], s[LOAD + 2]], [s[LOAD_SPEED], s[LOAD_SPEED + 1], s[LOAD_SPEED + 2]]);
    let off = |a: V3, b: V3| a.iter().zip(b).map(|(x, y)| (x - y).abs()).fold(0.0, f64::max);
    assert!(off(sub(low, p1), [0.0, 0.0, -ls]) < 1e-12 && off(low_v, v1) < 1e-12, "{low:?} {low_v:?}");
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
    // Measured 2.3e-6 off (the swing's amplitude, 3.3e-3 rad, lengthens it by theta^2 / 16 = 7e-7).
    assert!((period / hand - 1.0).abs() < 2e-5, "{period} s against {hand} s");
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
/// from the zero crossings of the hook block's sway; l2 is the sling at its rest stretch, l_s (1 + m2 g / EA). The swing is
/// small, 5e-4 rad of the rope: in the upper mode hook block and load swing against each other and pull the hook block up
/// and down at twice the frequency (30.2 rad/s), close to the load's bounce on the elastic rope and sling in series (5.94
/// Hz, 37.3 rad/s), which lengthens the period by some theta1^2 (measured: with the sling a rod and the bounce at 42.6
/// rad/s, 1.36 % at 0.01 rad, 0.13 % at 3e-3, 1.5e-4 at 1e-3; with the elastic sling 2.1e-4 at 1e-3, 5.3e-5 at 5e-4;
/// the model's linearisation, its Jacobian's eigenvalues, gives both hand periods to 1e-6).
#[test]
fn the_hook_block_and_its_load_swing_at_the_double_pendulum_s_two_frequencies() {
    let mut cfg = lossless(&["effects.hook_block = true"]);
    cfg.set_text("effects.holding_brake = true").unwrap();
    let a0 = crane(&cfg).layout.sling.unwrap();
    for upper in [false, true] {
        let mut m = crane(&cfg);
        m.set_brake_engaged(true);
        let (mut state, hold) = at_rest(&m, [5.0, 2.0, 2.0]);
        // The sling at its rest stretch under the payload.
        let (l1, l2) = (m.rope(&state).length, m.sling.unwrap() * (1.0 + m.payload * G / m.ea));
        let (m1, m2) = (m.hook, m.payload);
        let b = (m1 + m2) * G * (l1 + l2);
        let root = (b + if upper { 1.0 } else { -1.0 } * (b * b - 4.0 * m1 * l1 * l2 * (m1 + m2) * G * G).sqrt())
            / (2.0 * m1 * l1 * l2);
        let ratio = l1 * root / (G - l2 * root);
        let theta1 = 5e-4f64;
        // The rope turned by theta1 about the sheave, the sling by theta2 = ratio theta1 (to the y direction).
        state[LOAD + 1] = state[1] + l1 * theta1.sin();
        state[LOAD + 2] = m.height - l1 * theta1.cos();
        state[a0 + 1] = l2 * (ratio * theta1).sin();
        state[a0 + 2] = -l2 * (ratio * theta1).cos();
        let mut rk = Rk4::new(m.state_len());
        let (mut crossings, mut last) = (Vec::new(), state[LOAD + 1] - state[1]);
        let mut dump = String::new();
        for step in 1..=(9.0 / DT) as usize {
            advance(&m, &mut rk, &mut state, &hold, DT, 4);
            if step % 16 == 0 {
                dump += &format!(
                    "{} {} {} {} {}\n",
                    step as f64 * DT,
                    state[LOAD + 1] - state[1],
                    state[a0 + 1],
                    state[LOAD + 2],
                    state[1]
                );
            }
            let sway = state[LOAD + 1] - state[1];
            if sway.signum() != last.signum() {
                crossings.push((step as f64 - sway / (sway - last)) * DT);
            }
            last = sway;
        }
        let n = crossings.len() - 1 - (crossings.len() - 1) % 2;
        let period = 2.0 * (crossings[n] - crossings[0]) / n as f64;
        let hand = TAU / root.sqrt();
        assert!((period / hand - 1.0).abs() < 2e-4, "upper {upper}: {period} s against {hand} s");
    }
}

/// Energy: with the drives held and the losses off, kinetic + potential + elastic less the holding torques' work on the
/// motor angles stays put, for each new effect and for all together (the sling and the rope swinging, the rotors twisted,
/// the girder yawed with the trolley off its middle, the drum paying out), on the elastic rope and on the rigid one (the
/// load put on it at its length and speed). Measured: 2e-11 J for each effect, 2.2e-10 J
/// for all together, of some 8 kJ (rounding); the rope's mass without its l' du/dq term drifts 2.3e-3 J.
#[test]
fn the_new_effects_conserve_energy_with_the_drives_held() {
    let cases: [(&[&str], [f64; 3]); 7] = [
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
        (&["effects.rope_stretch = false", "effects.rope_mass = true", "rope.mass_per_m_kg_m = 0.5"], [5.0, 2.0, 2.0]),
        (
            &[
                "effects.rope_stretch = false",
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
        // Everything on the move, the drum paying out (the rope's l' terms).
        m.set_tool_velocity(&mut state, &[0.05, 0.04, -0.1]);
        state[LOAD_SPEED] = 0.1;
        if let (Some(a0), Some(rest)) = (m.layout.sling, m.sling_rest()) {
            // The sling turned aside and swinging, taut (its radial speed's swing, 0.03 m/s over 76 rad/s, inside its
            // 1.7 mm rest stretch).
            state[a0] = 0.29;
            state[a0 + 1] = -0.2;
            state[a0 + 2] = -(rest * rest - 0.29 * 0.29 - 0.2 * 0.2).sqrt();
            state[a0 + 3] = 0.1;
        }
        if let Some(r0) = m.layout.rotor {
            state[r0 + 2] += 2e-3;
            state[r0 + 1] -= 1e-3;
        }
        if let Some(k) = m.layout.skew {
            state[k] = 2e-4;
        }
        if !m.elastic {
            // The rigid rope's length and its rate as the drum's (the constraint's own place and speed).
            let l = state[2];
            m.place_on_rope(&mut state, l, 1.0);
        }
        let worst = drift(&m, &mut state, &hold, 1.5);
        assert!(worst < 1e-8, "{edits:?}: {worst} J");
    }
}

/// Each elastic drive's mode, the rotor against the load it moves with the rope held (the rope's stiffness raised so
/// that the hook stays with the drum, the payload off, the gearbox damping off): w^2 = k (1 / J + 1 / M), J = i^2 J_rotor
/// the rotor's inertia at the axis, k the gearbox's stiffness at the axis (2e4 N m/rad over the lever squared), M the mass
/// the axis moves: bridge + trolley, trolley, and for the hoist the drum's J / r^2 and the hook. By hand for the bridge: i
/// = 30 / 0.08 = 375 rad/m, J = 1.34e-4 x 375^2 = 18.84 kg, k = 2e4 / 0.08^2 = 3.125e6 N/m, M = 720 kg: w = sqrt(3.125e6
/// (1/18.84 + 1/720)) = 412.5 rad/s, 65.66 Hz. The twist of the rotor over the axis is followed through 20 of its periods.
/// The rope of 1e11 N carries the hook with the drum: its own mode, sqrt(EA / (l m_hook)) = 58,200 rad/s, is 0.91 rad of
/// the 15.6 us sub-step, inside RK4's stable 2.8, and moves the hoist's mode by 4e-5 (the Jacobian's eigenvalues: 405.896
/// against 405.910 rad/s by hand; with the shipped rope the hoist's mode couples with the hook's bounce, 177 and 700 rad/s).
/// Measured: 2e-9, 1.5e-7 and 2.0e-5 off the hand values.
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
        assert!((measured / hand - 1.0).abs() < 5e-5, "drive {d}: {measured} Hz against {hand} Hz");
    }
}

/// The bridge on two carriages, the girder 5.6 m between its rails (the trolley's middle at 2.5 m), each carriage taking
/// half the bridge's command T through its own drive (force F / 2 = T i / 2 each). By hand with the rotors' inertia at the
/// axis R = i^2 J = 375^2 x 1.34e-4 = 18.84 kg per carriage, the girder's yaw inertia J_b = m_b g^2 / 12, the trolley m_t
/// at the distance d from the middle (x_t = x_c + psi d, so its acceleration a + d psi''):
///   (m_b + m_t + 2R) a + m_t d psi'' = F,      m_t d a + (J_b + m_t d^2 + R g^2 / 2) psi'' = (F_B - F_A) g / 2 = 0.
/// (a) the trolley in the middle (d = 0): psi'' = 0 to rounding. (b) at d = 2 m: psi'' = -F m_t d / det with det = A C -
/// B^2, A = m_b + m_t + 2R, B = m_t d, C = J_b + m_t d^2 + R g^2 / 2. (c) The wheels hold a steady skew where psi'' = 0
/// under a steady acceleration a = F / A: psi_s = -m_t d a / k_psi, k_psi = k_lat b^2 / 2 = 2e6 x 2^2 / 2 = 4e6 N m/rad
/// (the heavier side, B with the trolley at +d, lags: psi = (x_B - x_A) / gauge < 0), but for the rope's pull (7e-4 of
/// (b)).
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
    // What is left is the rope's: the yaw moves the sheave by delta = psi d off the load hanging at rest, and the rope
    // pulls it back with t delta / L, in x and as the moment t delta d / L: psi'' = -t delta d (A - m_t) / (L det).
    let (rope, delta) = (m.rope(&at_rest(&m, [5.0, 2.5 + d, 2.0]).0), psi * d);
    let hand = -rope.tension * delta * d * (am - mt) / (rope.length * det);
    assert!((alpha_s / hand - 1.0).abs() < 1e-6, "{alpha_s} against {hand} ({alpha})");
    // The two sides' travels and torques are signals; the skew angle is their difference over the gauge.
    let (mut s, hold) = at_rest(&m, [5.0, 2.5, 2.0]);
    s[k] = 1e-3;
    let mut signals = Vec::new();
    m.base_signals(&s, &hold, &mut signals);
    let n = signals.len();
    let (angle, a_travel, b_travel) = (signals[n - 5], signals[n - 4], signals[n - 3]);
    assert!((angle - 1e-3).abs() < 1e-15 && ((b_travel - a_travel) / gauge - 1e-3).abs() < 1e-12);
}

/// The scene draws the girder yawed: its long axis, upright along z, turned by `angle` from +x towards +z, (-sin, 0, cos),
/// and laid flat, (x, z, -y), is (sin psi, cos psi, 0): x grows by psi per metre of y, as the trolley's x_c + psi (y - y_c).
#[test]
fn the_scene_draws_the_girder_yawed() {
    let m = crane(&lossless(&["effects.bridge_skew = true"]));
    let (mut s, _) = at_rest(&m, [5.0, 2.0, 2.0]);
    let psi = 0.05;
    s[m.layout.skew.unwrap()] = psi;
    let mut scene = Scene::default();
    m.scene(&s, &mut scene);
    let girder = scene.parts.iter().find(|p| p.id == "bridge").unwrap();
    let axis = girder.oriented([-girder.angle.sin(), 0.0, girder.angle.cos()]);
    assert!(girder.flat && (axis[0] - psi.sin()).abs() < 1e-15 && (axis[1] - psi.cos()).abs() < 1e-15, "{axis:?}");
    assert!(girder.pos == [5.0, 0.5 * m.strokes[1], m.height + 0.5]);
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

/// The hook block, the payload and the floor (effects.hook_block, effects.floor_contact): the payload hanging 1.6 mm over
/// the floor (0.5 m) on its 1 m sling, the drum pays out that and 0.2 m more, its brake then holding it. The payload stands
/// on the floor, the sling, a rope, goes slack and pushes nothing, and the hook block hangs 0.2 m lower on the main rope
/// alone. By hand at rest: the rope's tension t = m_hook g = 147.10 N, its length L = l (1 + t / EA) from the drum's free
/// length l, so the hook block at H - L; the floor carries the payload's whole weight, m_p g = 2451.66 N; the sling's
/// span 0.8 m less than the rope's lowering (`base_signals`: rope.tension and load.floor_force). A sling that was a rod
/// held the hook block up on the standing payload: the main rope slack (0 N), the floor carrying both, 2598.76 N. The
/// rope's damping (c / l on the rope) settles the hook block's drop onto it in the 4 s (after 2 s it still rang by 8e-8 m,
/// 0.03 N).
#[test]
fn a_hook_block_lowered_onto_a_standing_payload_hangs_on_the_rope_alone() {
    let mut m = crane(&lossless(&[
        "effects.hook_block = true",
        "effects.floor_contact = true",
        "effects.rope_damping = true",
        "effects.holding_brake = true",
    ]));
    m.set_brake_engaged(true);
    let floor = m.floor.unwrap();
    let (mut s, hold) = at_rest(&m, [5.0, 2.0, floor + m.sling.unwrap() + 0.01]);
    let gap = m.lower(&s).0[2] - floor;
    assert!(gap > 0.0 && gap < 0.01, "{gap}");
    s[2] += gap + 0.2;
    let mut rk = Rk4::new(m.state_len());
    for _ in 0..(4.0 / DT) as usize {
        advance(&m, &mut rk, &mut s, &hold, DT, 4);
        m.after_step(&mut s, DT);
    }
    let mut signals = Vec::new();
    m.base_signals(&s, &hold, &mut signals);
    let (tension, floor_force) = (signals[0], signals[6]);
    let (t, n) = (m.hook * G, m.payload * G);
    let hook = m.height - s[2] * (1.0 + t / m.ea);
    assert!((tension / t - 1.0).abs() < 1e-6, "rope {tension} N against {t} N");
    assert!((floor_force / n - 1.0).abs() < 1e-9, "floor {floor_force} N against {n} N");
    assert!((s[LOAD + 2] - hook).abs() < 1e-7 && (m.lower(&s).0[2] - floor).abs() < 1e-12, "{s:?}");
    let sling = m.sling_state(&s).unwrap();
    let slack = hook - floor - m.sling.unwrap();
    assert!(sling.tension == 0.0 && (sling.stretch - slack).abs() < 1e-7 && slack < -0.19, "{} m", sling.stretch);
}

/// A pick from the floor on the sling (effects.hook_block, effects.floor_contact): the hook block hangs 0.8 m over the
/// floor, less than the 1 m sling, and the payload goes on. It stands on the floor, the sling slack 0.2 m (the payload
/// where it stands, not plumb at the sling's length under the floor), and the pick makes no energy: what it booked is
/// the energy's change, at least 0. The empty sling's span follows the floor while nothing hangs on it, so the payload
/// is counted where it stands when it goes on; at the sling's length under the floor it was lifted 0.2 m by the floor at
/// the pick, -490 J booked.
#[test]
fn a_pick_from_the_floor_on_a_slack_sling_makes_no_energy() {
    let mut m = crane(&lossless(&["effects.hook_block = true", "effects.floor_contact = true"]));
    m.attached = false;
    let floor = m.floor.unwrap();
    let (mut s, _) = at_rest(&m, [5.0, 2.0, floor + 0.8]);
    m.after_step(&mut s, DT);
    m.set_payload_attached(true);
    let before = m.energy(&s);
    m.after_step(&mut s, DT);
    let lost = before - m.energy(&s);
    assert!(lost >= 0.0 && (m.pick_loss * DT - lost).abs() < 1e-9, "{lost} J lost, {} J booked", m.pick_loss * DT);
    let (low, low_v) = m.lower(&s);
    assert!((low[2] - floor).abs() < 1e-12 && low_v == [s[LOAD_SPEED], s[LOAD_SPEED + 1], s[LOAD_SPEED + 2]]);
    let sling = m.sling_state(&s).unwrap();
    assert!(sling.tension == 0.0 && (sling.stretch + 0.2).abs() < 1e-3, "{} m", sling.stretch);
}

/// Energy through a lift-off from the floor on the sling (effects.hook_block, effects.floor_contact, the losses off): the
/// payload stands, the sling slack 0.2 m, and the hoist's torque rises over 2 s from what holds the hook block to 1.2
/// times what holds both. The drum takes the slack in, the sling comes taut with a jerk (the rope's and the sling's
/// stretch take it), the payload lifts off and may fall back (a plastic fall, booked with `power.other`). The energy less
/// the motor torque's work on the motor angles plus what `after_step` booked and what the rope's and the sling's dampers
/// took (effects.rope_damping on, the shipped setting: their power trapezoid-integrated over each sub-step) stays put; with
/// the damping off the jerk is elastic.
#[test]
fn a_lift_off_on_a_slack_sling_keeps_the_energy() {
    for damping in [false, true] {
        let mut m = crane(&lossless(&[
            "effects.hook_block = true",
            "effects.floor_contact = true",
            "crane.floor_friction_coefficient = 0.0",
            &format!("effects.rope_damping = {damping}"),
        ]));
        let floor = m.floor.unwrap();
        let (mut s, hold) = at_rest(&m, [5.0, 2.0, floor + m.sling.unwrap() + 0.01]);
        let gap = m.lower(&s).0[2] - floor;
        s[2] += gap + 0.2;
        let (m1, mm) = (m.hook, m.load_mass());
        let mut rk = Rk4::new(m.state_len());
        let (mut pos, mut vel, mut powers) = ([0.0; 3], [0.0; 3], Vec::new());
        let e0 = m.energy(&s);
        let (mut work, mut booked, mut worst, mut lift, mut least) = (0.0, 0.0, 0.0f64, 0.0f64, 0.0f64);
        // The dampers' power (`power.rope`): the losses but `power.other`, which carries `after_step`'s.
        let mut damped = |m: &Crane, s: &[f64], torque: &[f64]| {
            powers.clear();
            m.loss_powers(s, torque, &mut powers);
            powers[..10].iter().sum::<f64>()
        };
        for step in 0..(2.0 / DT) as usize {
            let ramp = (step as f64 * DT / 2.0).min(1.0);
            let mut torque = hold;
            torque[2] = hold[2] * (m1 + ramp * (1.2 * mm - m1)) / mm;
            m.motor_state(&s, &mut pos, &mut vel);
            let before = pos;
            // The integrator's four sub-steps one by one, the dampers' power trapezoid-integrated over each.
            for _ in 0..4 {
                let p0 = damped(&m, &s, &torque);
                advance(&m, &mut rk, &mut s, &torque, DT / 4.0, 1);
                booked += 0.5 * (p0 + damped(&m, &s, &torque)) * DT / 4.0;
            }
            m.after_step(&mut s, DT);
            booked += m.pick_loss * DT;
            least = least.min(m.pick_loss * DT);
            m.motor_state(&s, &mut pos, &mut vel);
            work += (0..3).map(|k| torque[k] * (pos[k] - before[k])).sum::<f64>();
            worst = worst.max((m.energy(&s) - work + booked - e0).abs());
            lift = lift.max(m.lower(&s).0[2] - floor);
        }
        assert!(s.iter().all(|v| v.is_finite()) && lift > 0.02, "lifted {lift} m");
        // Measured: 6.4e-4 J undamped (of 302 J of work, 66.6 J booked at the floor); damped 0.032 J of 135 J booked,
        // the trapezoid's error where the dampers' power jumps (a tension clamped at 0 while the rope or sling is still
        // stretched): first order, 0.147 J with one trapezoid per step instead of per sub-step.
        // No step books a negative loss (energy made) beyond rounding: measured -1.1e-12 J. With the sling a rod the floor
        // lifted the hook block on the standing payload each step, booked as made energy (the audit's crane cycle with
        // the hook block on the floor: power.other -1.30 J, 1.30 J unaccounted; now -2.3e-10 J and -9.3e-10 J with
        // the torque ripple, whose work is in that row, off).
        assert!(least > -1e-9, "damping {damping}: a step booked {least} J");
        let tolerance = if damping { 0.05 } else { 1e-3 };
        assert!(worst < tolerance, "damping {damping}: {worst} J of {work} J of work, {booked} J booked");
    }
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
    // And on the rigid rope, all together.
    let mut cfg = ENTRY.config();
    for e in ["effects.rope_stretch = false", "effects.rope_mass = true", "effects.hook_block = true"] {
        cfg.set_text(e).unwrap();
    }
    for e in ["effects.joint_elasticity = true", "effects.bridge_skew = true", "effects.floor_contact = true"] {
        cfg.set_text(e).unwrap();
    }
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    sim.run(0.2, |_| {});
    assert!(sim.state.iter().all(|v| v.is_finite()));
}

/// The shipped gains hold the elastic drives: the default cycle with effects.joint_elasticity runs without a trip and
/// no motor's peak torque nears its limit (tuned for the rigid drive, the bridge's loop ran away above the gearbox's mode
/// and its command beat between +-10 N m: motor-peak 8.1), and every velocity loop's margins are measured, a crossover
/// in the band, and meet the 35 degrees and 6 dB of the shipped loops' test (measured: bridge 51.7 degrees, 9.7 dB at
/// 177 Hz; trolley 51.0 degrees, 9.9 dB at 183 Hz; hoist 78.2 degrees, 20.7 dB at 49.5 Hz).
#[test]
fn the_default_cycle_with_elastic_drives_keeps_its_loops_stable() {
    use sim_components::analysis::{analyse_cycle, velocity_loop_margins, verdicts};
    let mut cfg = ENTRY.config();
    cfg.set_text("effects.joint_elasticity = true").unwrap();
    cfg.set_text("effects.thermal_warm_start = false").unwrap();
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    assert!(!load.tripped);
    for v in verdicts(&cfg, &load).unwrap().iter().filter(|v| v.check == "motor-peak") {
        assert!(v.ok && v.value < 0.5, "motor-peak of {:?}: {}", v.motor, v.value);
    }
    for (k, m) in velocity_loop_margins(&ENTRY, &cfg, &[]).unwrap()[0].1.iter().enumerate() {
        assert!(m.stable() && m.crossover_hz.is_finite(), "motor {k}: {m:?}");
        assert!(m.phase_margin_deg >= 35.0 && m.gain_margin_db >= 6.0, "motor {k}: {m:?}");
    }
}

/// The shipped cycle with the hook block, unshaped, does not repeat itself: the double pendulum carries the swing one pass
/// leaves (0.5 m at the dwells, `residual-sway`) into the next, and the bridge's winding settles at 1.9 to 4.7 K from
/// pass to pass started at about the same rises. The warm start converges neither per pass nor over a period; a last
/// pass from the rises the one before started at settles 0.69 K from it (measured), seven times the tolerance, and the
/// verdict row `thermal-cycle-repeat` says so (`WarmStart::repeat`; before, the message said only "not converged", 3.88
/// K). Shaped (effects.input_shaping, the
/// swing taken out) the same cycle converges (0.015 K) and has no such row.
#[test]
fn the_unshaped_cycle_on_the_hook_block_does_not_repeat_and_the_warm_start_says_so() {
    use sim_components::analysis::{analyse_cycle, verdicts};
    for shaped in [false, true] {
        let mut cfg = ENTRY.config();
        cfg.set_text("effects.hook_block = true").unwrap();
        cfg.set_text(&format!("effects.input_shaping = {shaped}")).unwrap();
        let load = analyse_cycle(&mut Sim::new(&ENTRY, &cfg).unwrap()).unwrap();
        let start = load.warm_start.unwrap();
        let rows = verdicts(&cfg, &load).unwrap();
        let repeat = rows.iter().find(|v| v.check == "thermal-cycle-repeat");
        if shaped {
            assert!(start.converged() && repeat.is_none(), "{start:?}");
        } else {
            assert!(!start.converged() && repeat.is_some_and(|v| !v.ok), "{start:?} {repeat:?}");
        }
    }
}

/// The mode list (`Machine::modes`: the app's marks, `identify`, a shaper following its mode k) has, with the hook block,
/// the double pendulum's upper root beside the lower one of `resonance_hz`: the roots of m1 l1 l2 w^4 - (m1 + m2) g
/// (l1 + l2) w^2 + (m1 + m2) g^2 = 0 whose periods the ringing above measures (it listed the lower alone), the two
/// lowest; the bounces along the rope and the sling follow (`the_bounce_and_the_held_drives_are_among_the_modes`).
/// Without the hook block one swing.
#[test]
fn the_mode_list_has_both_of_the_double_pendulum_s_roots() {
    let m = crane(&lossless(&["effects.hook_block = true"]));
    let pose = [5.0, 2.0, 2.0];
    let (l1, m1, m2) = (m.height - pose[2], m.hook, m.payload);
    let l2 = m.sling.unwrap() * (1.0 + m2 * G / m.ea);
    let b = (m1 + m2) * G * (l1 + l2);
    let root = |sign: f64| {
        ((b + sign * (b * b - 4.0 * m1 * l1 * l2 * (m1 + m2) * G * G).sqrt()) / (2.0 * m1 * l1 * l2)).sqrt() / TAU
    };
    let modes = m.modes(&pose);
    assert!(modes.len() == 4 && modes[0].hz == m.resonance_hz(&pose).unwrap(), "{modes:?}");
    for (mode, hand) in modes.iter().zip([root(-1.0), root(1.0)]) {
        assert!((mode.hz / hand - 1.0).abs() < 1e-12, "{modes:?} against {hand} Hz");
    }
    let plain = crane(&lossless(&[]));
    assert!(plain.modes(&pose).iter().filter(|mode| mode.hz < 1.0).count() == 1);
}

/// The modes along the rope and of the elastic drives, with the motors held (`held_modes`), at the pose z = 2 m: the drum's
/// free length l = 3 / (1 + m_hook g / EA) = 2.99969 m, the rope at rest L = l (1 + m g / EA), m = 265 kg, EA = 1.44e6 N.
/// (a) Rigid drives: the drum held, the load bounces on the rope, w = sqrt(EA / (l m)) = 42.56 rad/s, 6.774 Hz, its
/// damping ratio c / l / (2 sqrt(EA / l m)) = 0.0245 with the rope's c = 1660 N s; the model linearised with the hoist's
/// brake holding its motor has it too. (b) Elastic drives, gearbox damping off: the drum (J / r^2 = 7.8125 kg) on the
/// gearbox's k_g = 2e4 / 0.08^2 = 3.125e6 N/m and the rope, K = [k_g + EA L^2 / l^3, -EA L / l^2; -EA L / l^2, EA / l],
/// M = diag(7.8125, 265): the roots of det(K - w^2 M) = 0, 6.304 and 108.17 Hz; the bridge's 720 kg on its 3.125e6 N/m,
/// 10.485 Hz, the trolley's 120 kg on 2e4 / 0.05^2 = 8e6 N/m, 41.09 Hz. (The model linearised with the motors free has
/// the rotors in: the drives at 65.66, 87.96 and 110.97 Hz, the bounce at 11.39 Hz; rigid, 12.14 Hz with the drum and
/// rotor free.) (c) The hook block
/// (15 kg) on the rope and the payload (250 kg) on the sling EA / l_s: K = [EA / l + k_s, -k_s; -k_s, k_s], 5.939 and
/// 57.90 Hz. (d) The rigid rope on rigid drives: the swing alone.
#[test]
fn the_bounce_and_the_held_drives_are_among_the_modes() {
    use sim_components::shapes::{as_mode, eigen_near, linearised};
    let pose = [5.0, 2.0, 2.0];
    let roots = |m: [f64; 2], k: [f64; 3]| {
        let (a, b, c) = (m[0] * m[1], m[0] * k[2] + m[1] * k[0], k[0] * k[2] - k[1] * k[1]);
        [-1.0, 1.0].map(|sign: f64| ((b + sign * (b * b - 4.0 * a * c).sqrt()) / (2.0 * a)).sqrt() / TAU)
    };
    let near = |modes: &[sim_core::Mode], hz: f64| {
        modes.iter().min_by(|a, b| (a.hz / hz - 1.0).abs().total_cmp(&(b.hz / hz - 1.0).abs())).unwrap().clone()
    };
    let mut m = crane(&lossless(&["effects.rope_damping = true", "effects.holding_brake = true"]));
    let (ea, load) = (m.ea, m.load_mass());
    let l = 3.0 / (1.0 + m.hook * G / ea);
    let length = l * (1.0 + load * G / ea);
    let w = (ea / (l * load)).sqrt();
    let modes = m.modes(&pose);
    let bounce = near(&modes, w / TAU);
    let zeta = 1660.0 / l / (2.0 * (ea / l * load).sqrt());
    assert!(modes.len() == 2 && (bounce.hz / (w / TAU) - 1.0).abs() < 1e-12, "{modes:?} against {} Hz", w / TAU);
    assert!((bounce.damping / zeta - 1.0).abs() < 1e-9, "{} against {zeta}", bounce.damping);
    m.set_brake_engaged(true);
    let (_, jac) = linearised(&m, &pose);
    let linear = as_mode(eigen_near(&jac, m.state_len(), [-zeta * w, w]).unwrap().0).hz;
    assert!((linear / (w / TAU) - 1.0).abs() < 1e-3, "linearised {linear} Hz against {} Hz", w / TAU);
    // (b)
    let e = crane(&lossless(&["effects.joint_elasticity = true", "gearbox.damping_nm_s_rad = 0.0"]));
    let modes = e.modes(&pose);
    let (kg, md) = (e.drives[2].gearbox.stiffness, e.masses[2]);
    let hoist = roots([md, load], [kg + ea * length * length / l.powi(3), -ea * length / (l * l), ea / l]);
    let bridge = (e.drives[0].gearbox.stiffness / (e.bridge + e.trolley)).sqrt() / TAU;
    let trolley = (e.drives[1].gearbox.stiffness / e.trolley).sqrt() / TAU;
    assert!(modes.len() == 5, "{modes:?}");
    for hand in [hoist[0], hoist[1], bridge, trolley] {
        assert!((near(&modes, hand).hz / hand - 1.0).abs() < 1e-12, "{modes:?} against {hand} Hz");
    }
    // (c)
    let h = crane(&lossless(&["effects.hook_block = true"]));
    let ks = ea / h.sling.unwrap();
    let both = roots([h.hook, h.payload], [ea / l + ks, -ks, ks]);
    let modes = h.modes(&pose);
    for hand in both {
        assert!((near(&modes, hand).hz / hand - 1.0).abs() < 1e-12, "{modes:?} against {hand} Hz");
    }
    // (d)
    assert_eq!(crane(&lossless(&["effects.rope_stretch = false"])).modes(&pose).len(), 1);
}

/// With effects.bridge_skew the girder's yaw is among the modes (it listed the swing alone): no loop holds it, the
/// carriages share the bridge's command and its loop holds their mean. By hand with the trolley 2 m off the middle (the
/// skew test's A, B, C): rigid, sqrt(k_psi / C) on k_psi = k_lat b^2 / 2 = 4e6 N m/rad. The model linearised with the
/// motors free, the carriages' mean free too, has it at sqrt(k_psi / (C - B^2 / A)), to the rope's pull. Elastic, the
/// rotors' differential delta on the gearboxes: J R' w^4 - (J k' + R' (k_psi + k')) w^2 + k_psi k' = 0, J = J_b + m_t d^2,
/// R' = R g^2 / 2, k' = k g^2 / 2; its lower root 0.1 % under the rigid one.
#[test]
fn the_bridge_s_skew_is_among_the_modes() {
    use sim_components::shapes::{as_mode, eigen_near, linearised};
    let (gauge, d) = (5.6, 2.0);
    let pose = [5.0, 2.5 + d, 2.0];
    let m = crane(&lossless(&["effects.bridge_skew = true"]));
    let r = m.drives[0].reflected_inertia();
    let (jb, mt, mb) = (m.bridge * gauge * gauge / 12.0, m.trolley, m.bridge);
    let (a, b, c) = (mb + mt + 2.0 * r, mt * d, jb + mt * d * d + 0.5 * r * gauge * gauge);
    let k_psi = 0.5 * 2.0e6 * 2.0 * 2.0;
    let held = (k_psi / c).sqrt() / TAU;
    // Beside the swing and the load's bounce on the rope.
    let modes = m.modes(&pose);
    let near = |modes: &[sim_core::Mode], hz: f64| modes.iter().map(|m| (m.hz / hz - 1.0).abs()).fold(1.0, f64::min);
    assert!(modes.len() == 3 && near(&modes, held) < 1e-12, "{modes:?} against {held} Hz");
    let free = (k_psi / (c - b * b / a)).sqrt();
    let (_, jac) = linearised(&m, &pose);
    let linear = as_mode(eigen_near(&jac, m.state_len(), [0.0, free]).unwrap().0).hz;
    assert!((linear / (free / TAU) - 1.0).abs() < 1e-3, "linearised {linear} Hz against {} Hz", free / TAU);
    let e = crane(&lossless(&["effects.bridge_skew = true", "effects.joint_elasticity = true"]));
    let (k2, r2, j) = (0.5 * e.drives[0].gearbox.stiffness * gauge * gauge, 0.5 * r * gauge * gauge, jb + mt * d * d);
    let (p, q) = (j * r2, j * k2 + r2 * (k_psi + k2));
    let low = ((q - (q * q - 4.0 * p * k_psi * k2).sqrt()) / (2.0 * p)).sqrt() / TAU;
    let modes = e.modes(&pose);
    // Beside the swing, the bounce on the hoist's gearbox and the rope (two), the bridge's and the trolley's.
    assert!(modes.len() == 7 && near(&modes, low) < 1e-9, "{modes:?} against {low} Hz");
    assert!(low < held && low > 0.995 * held, "{low} Hz against {held} Hz");
}
