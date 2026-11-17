//! The leaning Z axis of the T-gantry file and the X beam's vertical mode, against hand values and against the crate:
//! the belt runs' geometric stiffness on the lean, the process force's vertical part on the leaning tip's rise, and
//! effects.beam_vertical_mode (`machines/t-gantry.toml`, `crates/machine-t-gantry`).

use sim_core::{Machine, MachineEntry};
use sim_machines::{find, load_file};
use std::path::{Path, PathBuf};

const G: f64 = 9.80665;

fn file() -> &'static MachineEntry {
    let root: PathBuf = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    load_file(&root.join("machines").join("t-gantry.toml")).unwrap_or_else(|e| panic!("{e}"))
}

fn file_machine(changes: &[&str]) -> Box<dyn Machine> {
    let entry = file();
    let mut cfg = entry.config();
    for change in changes {
        cfg.set_text(change).unwrap();
    }
    (entry.build)(&cfg).unwrap()
}

/// The values of the signals whose ids end with `suffix`, in the machine's order.
fn signals_ending(m: &dyn Machine, state: &[f64], torque: &[f64], suffix: &str) -> Vec<f64> {
    let mut values = Vec::new();
    m.signals(state, torque, &mut values);
    m.signal_infos().iter().zip(values).filter(|(info, _)| info.id.ends_with(suffix)).map(|(_, v)| v).collect()
}

/// The belt runs' geometric stiffness on the lean (the crate's `the_belt_runs_hold_the_lean_back_with_t_over_l`). The
/// clamp (s + z above the carriage's idlers) and the Z-bottom idler (s + sz - z below them) stand out sideways by k u g with
/// the lean u, g = 1 / lateral + h d / tilt at the signed distance d below the carriage; a run of length l under the tension T
/// holds the end back with T delta / l, so the belt's energy has K_uu = (T1 + T3) (k g_clamp)^2 / (s + z) + 2 T2 (k g_bottom)^2 /
/// (s + sz - z) on top of the lean's spring k and the weight's pendulum stiffness c = k^2 m g h^3 / tilt^2 (the body
/// m = 5 + 2 kg hangs h below the carriage, rises by h (k u h / tilt)^2 / 2 as it leans: V = m g F^2 r = c u^2 / 2, c =
/// 2 k^2 m g r). By hand at z = 0.05 m (s = 0.15, sz = 0.5, h = 0.25 m, lateral 2e7, tilt 1.2e4, d = -0.2 and 0.6): k =
/// 1 / (5e-8 + 0.0625 / 1.2e4) = 190.2e3 N/m, k g_clamp = -0.7805, k g_bottom = 2.3796, with the tensions of the rest
/// state. The energy's second derivative in u over 1e-4 m.
#[test]
fn the_belt_runs_hold_the_lean_back_with_t_over_l() {
    let m = file_machine(&["effects.tool_flex = true"]);
    let z = 0.05;
    let (mut state, mut torque) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&[0.5, z], &mut state, &mut torque);
    let t = signals_ending(m.as_ref(), &state, &torque, ".tension");
    assert_eq!(t.len(), 3);
    let (lateral, tilt, s, sz, h, mass) = (2.0e7, 1.2e4, 0.15, 0.5, 0.3 - z, 7.0);
    let k = 1.0 / (1.0 / lateral + h * h / tilt);
    let (g_clamp, g_bottom) = (k * (1.0 / lateral - h * (s + z) / tilt), k * (1.0 / lateral + h * (s + sz - z) / tilt));
    assert!((g_clamp + 0.7805).abs() < 1e-3 && (g_bottom - 2.3796).abs() < 1e-3, "{g_clamp} {g_bottom}");
    let geometric = (t[0] + t[2]) * g_clamp * g_clamp / (s + z) + 2.0 * t[1] * g_bottom * g_bottom / (s + sz - z);
    assert!(geometric > 1e3, "{geometric} N/m");
    let weight = k * k * mass * G * h.powi(3) / (tilt * tilt);
    // The lean is the state after the rotors: x z a b pa pb u.
    let energy = |u: f64| {
        let mut at = state.clone();
        at[6] = u;
        m.energy(&at)
    };
    let step = 1e-4;
    let second = (energy(step) + energy(-step) - 2.0 * energy(0.0)) / (step * step);
    let found = second - k - weight;
    assert!((found - geometric).abs() < 1e-3 * geometric, "{found} N/m against {geometric} N/m");
}

/// The process force's vertical part on the leaning axis (the crate's `the_leaning_tip_rises_by_half_theta_squared_times_its_distance`).
/// The tip, d = z_tool_drop - z below the carriage, rises as the Z axis leans under the spring force k u at its centre of
/// mass (the guide's tilt only: the whole bar turns by theta = k u h / tilt): by theta^2 d / 2. At z = 0.25 m (h = 0.05, d =
/// 0.45, k = 1 / (5e-8 + 0.0025 / 1.2e4)) and u = 1 mm: theta = 0.01613, rise 5.85e-5 m, in the tool's height, and its
/// rate a' u^2 z' + 2 a u u'.
#[test]
fn the_leaning_tip_rises_by_half_theta_squared_times_its_distance() {
    let m = file_machine(&["effects.tool_flex = true"]);
    let (z, u) = (0.25, 1e-3);
    let (lateral, tilt) = (2.0e7_f64, 1.2e4_f64);
    let a = |z: f64| {
        let (h, d) = (0.3 - z, 0.7 - z);
        let k = 1.0 / (1.0 / lateral + h * h / tilt);
        0.5 * d * (k * h / tilt).powi(2)
    };
    let by_hand = a(z) * u * u;
    assert!((by_hand - 5.85e-5).abs() < 1e-6, "{by_hand}");
    let (mut state, mut torque) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&[0.5, z], &mut state, &mut torque);
    state[6] = u;
    let (mut pos, mut vel) = ([0.0; 2], [0.0; 2]);
    m.tool_state(&state, &mut pos, &mut vel);
    assert!((pos[1] - z - by_hand).abs() < 1e-6 * by_hand, "the tool is {} m higher", pos[1] - z);
    // The rates follow the seven positions: x' z' ... u' at 7, 8, 13.
    (state[13], state[8]) = (0.2, 0.1);
    m.tool_state(&state, &mut pos, &mut vel);
    let da = (a(z + 1e-5) - a(z - 1e-5)) / 2e-5;
    let rate = 2.0 * a(z) * u * 0.2 + da * u * u * 0.1;
    assert!((vel[1] - 0.1 - rate).abs() < 1e-4 * rate.abs(), "{} against {}", vel[1] - 0.1, rate);
}

/// The crate's state of the file's: the frame's, tool flex's and beam's states where the file has none are 0, positions
/// then rates.
fn crate_state(s: &[f64], extra: [bool; 3]) -> Vec<f64> {
    let positions = 6 + extra.iter().filter(|e| **e).count();
    let mut out = Vec::new();
    for half in 0..2 {
        let mut next = half * positions;
        out.extend_from_slice(&s[next..next + 6]);
        next += 6;
        for present in extra {
            out.push(if present {
                next += 1;
                s[next - 1]
            } else {
                0.0
            });
        }
    }
    out.extend_from_slice(&s[2 * positions..]);
    out
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * (1.0 + a.abs().max(b.abs()))
}

/// The file and the crate at one state far off rest (the Z axis 1 mm out and moving, the beam 0.1 mm off its sag), with
/// the cycle dwelling in its process force: the derivatives (the accelerations of the lean, of Z and of the beam among
/// them), the energy and every signal both have, to 1e-9.
fn same_leaning_state(beam: bool) {
    let mut changes = vec!["effects.tool_flex = true", "effects.process_force = true"];
    if beam {
        changes.push("effects.beam_vertical_mode = true");
    }
    let (krate, from_file) = (find("t-gantry").unwrap(), file());
    let (mut a, mut b) = (krate.config(), from_file.config());
    for off in ["frame_compliance", "tool_flex"] {
        a.set_text(&format!("effects.{off} = false")).unwrap();
    }
    for change in &changes {
        a.set_text(change).unwrap();
        b.set_text(change).unwrap();
    }
    let (mut ma, mut mb) = ((krate.build)(&a).unwrap(), (from_file.build)(&b).unwrap());
    let extra = [false, true, beam];
    for machine in [&mut ma, &mut mb] {
        machine.set_dwelling(true);
    }
    let pose = [0.6, 0.3];
    let (mut sa, mut sb, mut ta, mut tb) = (Vec::new(), Vec::new(), [0.0; 2], [0.0; 2]);
    ma.equilibrium(&pose, &mut sa, &mut ta);
    mb.equilibrium(&pose, &mut sb, &mut tb);
    mb.set_tool_velocity(&mut sb, &[0.4, -0.3]);
    // The file's states: x z a b pa pb u (w), the rates after them.
    let positions = 7 + usize::from(beam);
    sb[6] += 1e-3;
    sb[positions + 6] = 0.05;
    if beam {
        sb[7] += 1e-4;
        sb[positions + 7] = 0.01;
    }
    let sa = crate_state(&sb, extra);
    let torque = [ta[0] + 0.7, ta[1] - 0.2];
    let (mut da, mut db) = (vec![0.0; sa.len()], vec![0.0; sb.len()]);
    ma.derivatives(&sa, &torque, &mut da);
    mb.derivatives(&sb, &torque, &mut db);
    for (i, (x, y)) in da.iter().zip(crate_state(&db, extra)).enumerate() {
        assert!(close(*x, y, 1e-9), "beam {beam}: derivative {i}: crate {x}, file {y}");
    }
    assert!(close(ma.energy(&sa), mb.energy(&sb), 1e-9), "beam {beam}: energy");
    let (ia, ib) = (ma.signal_infos(), mb.signal_infos());
    let (mut va, mut vb) = (Vec::new(), Vec::new());
    ma.signals(&sa, &torque, &mut va);
    mb.signals(&sb, &torque, &mut vb);
    let mut shared = 0;
    for (info, value) in ib.iter().zip(&vb) {
        if let Some(i) = ia.iter().position(|x| x.id == info.id) {
            assert!(close(va[i], *value, 1e-9), "beam {beam}: signal {}: crate {}, file {value}", info.id, va[i]);
            shared += 1;
        }
    }
    assert!(shared * 10 >= ib.len() * 9, "only {shared} of {} signals are the crate's", ib.len());
    if beam {
        assert!(ib.iter().any(|i| i.id == "beam.deflection") && ib.iter().any(|i| i.id == "power.beam"));
    }
}

#[test]
fn the_file_and_the_crate_agree_on_the_leaning_axis_at_a_leaning_state() {
    same_leaning_state(false);
}

#[test]
fn the_file_and_the_crate_agree_with_the_beam_mode_at_a_leaning_state() {
    same_leaning_state(true);
}

/// The beam's mode (effects.beam_vertical_mode) against the hand value (the crate's `the_beams_vertical_mode_is_where_the_hand_value_has_it`).
/// The belt stiff (EA = 1e11 N), the carriage and the Z axis cannot move on the beam or against it and the lowest mode is
/// the beam's bending with them on it: f = sqrt(k / (m_c + m_z + M)) / 2 pi, in the middle (x = 0.5 m, 0.65 m from a post
/// of the 1.3 m span) k = 48 E I / L^3 = 48 x 8.4e4 / 1.3^3 = 1.83523e6 N/m, M = 17 mu L / 35 = 2.2100 kg, m_c + m_z = 4 + 7
/// kg: 1.83523e6 / 13.2100 = 138 928 1/s^2, 372.73 rad/s, 59.32 Hz.
#[test]
fn the_beams_vertical_mode_is_where_the_hand_value_has_it() {
    let m = file_machine(&["effects.beam_vertical_mode = true", "belt.stiffness_ea_n = 1.0e11"]);
    let by_hand =
        (48.0 * 8.4e4 / 1.3f64.powi(3) / (4.0 + 7.0 + 17.0 / 35.0 * 3.5 * 1.3)).sqrt() / std::f64::consts::TAU;
    assert!((by_hand - 59.32).abs() < 0.01, "{by_hand}");
    let hz = m.resonance_hz(&[0.5, 0.25]).unwrap();
    assert!((hz - by_hand).abs() < 1e-3 * by_hand, "{hz} Hz against {by_hand} Hz");
    // Nearer a post the beam is stiffer: 0.15 m from it (x = 0) the mode is higher.
    assert!(m.resonance_hz(&[0.0, 0.25]).unwrap() > hz);
}

/// The beam at rest at the quasi-static sag m g a^2 b^2 / (3 E I L) = 11 x 9.80665 / 1.83523e6 = 58.78 um in the middle, the
/// Z axis on its belt that much higher, the tool at its pose and nothing accelerating; the loss row is the beam's damping,
/// c w'^2 with c = 2 zeta sqrt(k M) = 2 x 0.01 x sqrt(1.83523e6 x 2.21) = 40.28 N s/m, at w' = 0.1 m/s 0.4028 W.
#[test]
fn the_beam_rests_at_its_sag_and_its_damping_is_in_the_budget() {
    let m = file_machine(&["effects.beam_vertical_mode = true"]);
    let (mut state, mut torque) = (Vec::new(), [0.0; 2]);
    m.equilibrium(&[0.5, 0.25], &mut state, &mut torque);
    let sag = 11.0 * G / (48.0 * 8.4e4 / 1.3f64.powi(3));
    assert!((sag - 58.78e-6).abs() < 1e-8, "{sag}");
    // x z a b pa pb w: the beam's deflection is state 6, the Z axis stands on its belt at 0.25 + sag.
    assert!((state[6] - sag).abs() < 1e-9 * sag && (state[1] - 0.25 - sag).abs() < 1e-12, "{state:?}");
    let mut rest = vec![0.0; state.len()];
    m.derivatives(&state, &torque, &mut rest);
    assert!(rest[7 + 6].abs() < 1e-6, "at rest the beam accelerates at {} m/s^2", rest[7 + 6]);
    let (mut pos, mut vel) = ([0.0; 2], [0.0; 2]);
    m.tool_state(&state, &mut pos, &mut vel);
    assert!((pos[1] - 0.25).abs() < 1e-12, "the tool is at {}", pos[1]);
    state[7 + 6] = 0.1;
    let c = 2.0 * 0.01 * (48.0 * 8.4e4 / 1.3f64.powi(3) * 17.0 / 35.0 * 3.5 * 1.3).sqrt();
    assert!((c - 40.28).abs() < 0.01, "{c}");
    let power = signals_ending(m.as_ref(), &state, &torque, "power.beam");
    assert!((power[0] - c * 0.01).abs() < 1e-9 * c, "{} W against {} W", power[0], c * 0.01);
}
