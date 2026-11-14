//! The linkage node of machine files (crates/sim-netlist/src/linkage.rs and spatial.rs): the planar delta written as a
//! netlist in machines/planar-delta.toml and the spatial one in machines/delta-d3.toml are their crates' machines with
//! every effect the crates have (crate and file build one implementation, read from tables or from the node), their
//! cycles give the crates' signals and verdicts, the planar one passes the standard suite, and its kinematics and
//! holding torque are the hand's.

use sim_components::Sim;
use sim_components::analysis::{analyse_cycle, verdicts};
use sim_components::control::Command;
use sim_core::{Config, ConfigError, Machine, MachineEntry};
use sim_machines::{MACHINES, find, load_file};
use std::path::{Path, PathBuf};

const G: f64 = 9.80665;

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn file() -> &'static MachineEntry {
    load_file(&root().join("machines/planar-delta.toml")).unwrap_or_else(|e| panic!("{e}"))
}

fn spatial_file() -> &'static MachineEntry {
    load_file(&root().join("machines/delta-d3.toml")).unwrap_or_else(|e| panic!("{e}"))
}

/// The crate and the file, each with the same changes.
fn pair(changes: &[&str]) -> [(&'static MachineEntry, Config); 2] {
    pair_of("delta-d2", file(), changes)
}

/// A crate by its id and a file, each with the same changes.
fn pair_of(krate: &str, from_file: &'static MachineEntry, changes: &[&str]) -> [(&'static MachineEntry, Config); 2] {
    let krate = find(krate).unwrap();
    let (mut a, mut b) = (krate.config(), from_file.config());
    for change in changes {
        a.set_text(change).unwrap();
        b.set_text(change).unwrap();
    }
    [(krate, a), (from_file, b)]
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * (1.0 + a.abs().max(b.abs())) || (a.is_nan() && b.is_nan())
}

fn assert_close(what: &str, a: &[f64], b: &[f64], tol: f64) {
    assert_eq!(a.len(), b.len(), "{what}: lengths");
    for (i, (x, y)) in a.iter().zip(b).enumerate() {
        assert!(close(*x, *y, tol), "{what}[{i}]: crate {x}, file {y}\n crate {a:?}\n file  {b:?}");
    }
}

/// Every effect of the crate's config, each switched from its default (the comparison below runs once per entry).
fn flipped() -> Vec<String> {
    flipped_of("delta-d2")
}

fn flipped_of(krate: &str) -> Vec<String> {
    let cfg = find(krate).unwrap().config();
    let effects = cfg.get("effects").unwrap().as_table().unwrap().clone();
    effects.iter().map(|(name, on)| format!("effects.{name} = {}", !on.as_bool().unwrap())).collect()
}

/// The two machines agree to 1e-9: their axes and motors, kinematics, the state at rest and the torque that holds it,
/// the derivatives of a moving state under torques, energy, feedforward, every signal (by id and value), the parts'
/// warmth and what the working range refuses; with the payload held, set down, and the brakes holding, the gearboxes
/// 10 and 20 K warm.
fn same_machine(changes: &[&str], what: &str) {
    let poses = [[0.1525, -0.5], [-0.3, -0.6], [0.05, -0.8]];
    let refused = [[0.0, -0.5], [0.45, -0.5], [0.3, -0.87], [0.0, -1.5], [0.0, -0.39]];
    same_machine_of(pair(changes), what, &poses, [[0.4, -0.3], [3.0, -2.0]], &refused);
}

/// `same_machine` for a pair of N axes: the poses to compare at, a tool velocity and acceleration, and poses to ask
/// the working range about.
fn same_machine_of<const N: usize>(
    pair: [(&'static MachineEntry, Config); 2],
    what: &str,
    poses: &[[f64; N]],
    [vel, acc]: [[f64; N]; 2],
    refused: &[[f64; N]],
) {
    let [(ea, ca), (eb, cb)] = pair;
    let (mut a, mut b) = ((ea.build)(&ca).unwrap(), (eb.build)(&cb).unwrap());
    let ids = |m: &dyn Machine| {
        (
            m.axes().iter().map(|a| (a.id, a.key, a.min, a.max, a.max_vel, a.max_acc, a.max_jerk)).collect::<Vec<_>>(),
            m.motors().iter().map(|m| (m.id, m.key)).collect::<Vec<_>>(),
            m.thermal_parts().iter().map(|p| (p.id, p.owner_key, p.scene_index)).collect::<Vec<_>>(),
            m.state_len(),
            m.setpoint_slack(),
        )
    };
    assert_eq!(ids(a.as_ref()), ids(b.as_ref()), "{what}");
    for machine in [&mut a, &mut b] {
        machine.set_part_temperatures(&[10.0, 20.0, 30.0][..N]);
    }
    for (k, &pose) in poses.iter().enumerate() {
        for machine in [&mut a, &mut b] {
            machine.set_payload_attached(k != 1);
            machine.set_brake_engaged(k == 2);
        }
        let (mut ma, mut mb) = ([0.0; N], [0.0; N]);
        a.inverse(&pose, &mut ma);
        b.inverse(&pose, &mut mb);
        assert_close(&format!("{what}: inverse"), &ma, &mb, 1e-12);
        let mut back = [0.0; N];
        b.forward(&mb, &mut back);
        assert_close(&format!("{what}: forward"), &pose, &back, 1e-12);

        let (mut sa, mut sb, mut ta, mut tb) = (Vec::new(), Vec::new(), [0.0; N], [0.0; N]);
        a.equilibrium(&pose, &mut sa, &mut ta);
        b.equilibrium(&pose, &mut sb, &mut tb);
        assert_close(&format!("{what}: equilibrium state"), &sa, &sb, 1e-9);
        assert_close(&format!("{what}: holding torque"), &ta, &tb, 1e-9);
        a.set_tool_velocity(&mut sa, &vel);
        b.set_tool_velocity(&mut sb, &vel);
        assert_close(&format!("{what}: moving state"), &sa, &sb, 1e-12);
        for (i, x) in sb.iter_mut().enumerate() {
            *x += 1e-5 * (1.0 + (i % 5) as f64);
        }
        sa.clone_from(&sb);
        let torque: [f64; N] = std::array::from_fn(|k| ta[k] + [0.7, 0.2, -0.4][k]);
        let (mut da, mut db) = (vec![0.0; sa.len()], vec![0.0; sb.len()]);
        a.derivatives(&sa, &torque, &mut da);
        b.derivatives(&sb, &torque, &mut db);
        assert_close(&format!("{what}: derivatives"), &da, &db, 1e-9);
        assert!(close(a.energy(&sa), b.energy(&sb), 1e-9), "{what}: energy");

        let (mut fa, mut fb) = ([0.0; N], [0.0; N]);
        a.feedforward_torque(&pose, &acc, &mut fa);
        b.feedforward_torque(&pose, &acc, &mut fb);
        assert_close(&format!("{what}: feedforward"), &fa, &fb, 1e-9);
        a.feedforward_torque_moving(&pose, &vel, &acc, &mut fa);
        b.feedforward_torque_moving(&pose, &vel, &acc, &mut fb);
        assert_close(&format!("{what}: feedforward moving"), &fa, &fb, 1e-9);

        let names = |m: &dyn Machine| m.signal_infos().iter().map(|s| s.id.clone()).collect::<Vec<_>>();
        assert_eq!(names(a.as_ref()), names(b.as_ref()), "{what}: signals");
        let (mut va, mut vb) = (Vec::new(), Vec::new());
        a.signals(&sa, &torque, &mut va);
        b.signals(&sb, &torque, &mut vb);
        assert_close(&format!("{what}: signal values"), &va, &vb, 1e-9);
    }
    // What the working range refuses, and why: inside, outside the band, below the bottom arc, beyond reach.
    for pose in refused {
        assert_eq!(a.unreachable(pose, 0.0), b.unreachable(pose, 0.0), "{what}: {pose:?}");
    }
}

/// Every effect of the planar delta's crate switched from its default, one at a time, and the default: the file's
/// machine is the crate's to 1e-9. Its hand checks are the crate's (crates/machine-delta-d2/src/tests.rs); the
/// linkage node has to give the same numbers.
#[test]
fn the_planar_delta_file_is_its_crate_with_every_effect() {
    same_machine(&[], "default");
    for change in flipped() {
        same_machine(&[&change], &change);
    }
}

/// The spatial delta written as a file (machines/delta-d3.toml) is its crate's machine, with every effect the crate has
/// switched from its default one at a time: the same numbers to 1e-9 (the same code, read from the node in place of the
/// crate's tables). The crate's working range is 1.2 m across, its top 0.675 m below the shoulders, 0.2 m high at the
/// edge and 0.386 m at the centre (its bottom a sphere's cap through z = -1.061 m at the centre): the refused poses lie
/// above the top, outside the cylinder, below the cap (0.907 m down at 0.55 m out), and far below.
#[test]
fn the_spatial_delta_file_is_its_crate_with_every_effect() {
    let check = |changes: &[&str], what: &str| {
        let poses = [[0.1, 0.05, -0.8], [-0.3, 0.2, -0.85], [0.0, -0.4, -0.75]];
        let refused = [[0.0, 0.0, -0.8], [0.0, 0.0, -0.6], [0.65, 0.0, -0.8], [0.55, 0.0, -1.0], [0.0, 0.0, -2.5]];
        let motion = [[0.4, -0.3, 0.2], [3.0, -2.0, 1.5]];
        same_machine_of(pair_of("delta-d3", spatial_file(), changes), what, &poses, motion, &refused);
    };
    check(&[], "default");
    for change in flipped_of("delta-d3") {
        check(&[&change], &change);
    }
}

/// The spatial delta's default cycle run in full by the crate and the file: every signal at every 16th step of the
/// first 0.3 s, and the cycle's analysis and verdicts, to 1e-9.
#[test]
fn the_spatial_delta_file_runs_its_crates_cycle() {
    let [(ea, ca), (eb, cb)] = pair_of("delta-d3", spatial_file(), &[]);
    let (mut a, mut b) = (Sim::new(ea, &ca).unwrap(), Sim::new(eb, &cb).unwrap());
    let ids = |s: &Sim| s.signal_infos().iter().map(|i| i.id.clone()).collect::<Vec<_>>();
    assert_eq!(ids(&a), ids(&b), "signals");
    for sim in [&mut a, &mut b] {
        sim.command(Command::StartCycle).unwrap();
    }
    let (mut va, mut vb) = (Vec::new(), Vec::new());
    for step in 0..(0.3 / ca.f64("sim.dt_s").unwrap()) as usize {
        a.step();
        b.step();
        if step % 16 == 0 {
            a.sample(&mut va);
            b.sample(&mut vb);
            assert_close(&format!("step {step}"), &va, &vb, 1e-9);
            va.clear();
            vb.clear();
        }
    }
    let ((names, crate_values), (file_names, values)) = (cycle_numbers(ea, &ca), cycle_numbers(eb, &cb));
    assert_eq!(names, file_names, "the same numbers, limits and verdicts");
    assert_close("cycle", &crate_values, &values, 1e-9);
}

/// Every number of a cycle's analysis and verdicts: the motor loads, then each verdict's value.
fn cycle_numbers(entry: &'static MachineEntry, cfg: &Config) -> (Vec<String>, Vec<f64>) {
    let load = analyse_cycle(&mut Sim::new(entry, cfg).unwrap()).unwrap();
    let (mut names, mut values) = (vec!["duration".to_string()], vec![load.duration]);
    for (k, m) in load.motors.iter().enumerate() {
        for (name, v) in [
            ("peak torque", m.peak_torque),
            ("rms torque", m.rms_torque),
            ("peak speed", m.peak_speed),
            ("utilisation", m.utilisation),
            ("rms current", m.rms_current),
            ("peak current", m.peak_current),
            ("energy in", m.energy_in),
            ("energy regenerated", m.energy_regenerated),
        ] {
            names.push(format!("motor {k} {name}"));
            values.push(v);
        }
    }
    for v in verdicts(cfg, &load).unwrap() {
        names.push(format!("verdict {} {:?} (limit {}, ok {})", v.check, v.motor, v.limit, v.ok));
        values.push(v.value);
    }
    (names, values)
}

/// The default cycle run in full (controller, drives, DC link, winding heat) by the crate and the file: every signal
/// at every 16th step of the first cycle, and the cycle's analysis and verdicts, to 1e-9. Elastic and rigid, and with
/// the effects that are off by default switched on together (part heating with its feedback, the fieldbus, noise,
/// shaping, the shared DC link, ...). The two compute the same sums in the same order, so they agree to the bit; the
/// encoder's counts stay on.
#[test]
fn the_planar_delta_file_runs_its_crates_cycle() {
    let off_by_default: Vec<String> = flipped().into_iter().filter(|c| c.ends_with("true")).collect();
    let all: Vec<&str> = off_by_default.iter().map(String::as_str).collect();
    let variants: [Vec<&str>; 3] = [vec![], vec!["effects.joint_elasticity = false"], all];
    for changes in &variants {
        let what = format!("{changes:?}");
        let [(ea, ca), (eb, cb)] = pair(changes);
        let (mut a, mut b) = (Sim::new(ea, &ca).unwrap(), Sim::new(eb, &cb).unwrap());
        let ids = |s: &Sim| s.signal_infos().iter().map(|i| i.id.clone()).collect::<Vec<_>>();
        assert_eq!(ids(&a), ids(&b), "{what}: signals");
        for sim in [&mut a, &mut b] {
            sim.command(Command::StartCycle).unwrap();
        }
        let (mut va, mut vb) = (Vec::new(), Vec::new());
        let steps = (0.5 / ca.f64("sim.dt_s").unwrap()) as usize;
        for step in 0..steps {
            a.step();
            b.step();
            if step % 16 == 0 {
                a.sample(&mut va);
                b.sample(&mut vb);
                assert_close(&format!("{what}: step {step}"), &va, &vb, 1e-9);
                va.clear();
                vb.clear();
            }
        }
        let ((names, crate_values), (file_names, values)) = (cycle_numbers(ea, &ca), cycle_numbers(eb, &cb));
        assert_eq!(names, file_names, "{what}: the same numbers, limits and verdicts");
        assert_close(&format!("{what}: cycle"), &crate_values, &values, 1e-9);
    }
}

/// The start pose and the cycle's targets moved by `dz` in z, as the crate's `suite_configs` lowers them.
fn lowered(cfg: &mut Config, dz: f64) {
    let start = cfg.f64s("start.pose").unwrap();
    cfg.set_text(&format!("start.pose = [{:?}, {:?}]", start[0], start[1] + dz)).unwrap();
    let segments: Vec<String> = cfg
        .array("cycle.segment")
        .unwrap()
        .iter()
        .map(|s| {
            let t: Vec<f64> = s["target"].as_array().unwrap().iter().map(|v| v.as_float().unwrap()).collect();
            format!(
                "{{ target = [{:?}, {:?}], dwell_s = {:?}, payload = {} }}",
                t[0],
                t[1] + dz,
                s["dwell_s"].as_float().unwrap(),
                s["payload"].as_bool().unwrap()
            )
        })
        .collect();
    cfg.set_text(&format!("cycle.segment = [{}]", segments.join(", "))).unwrap();
}

/// The standard suite on the file, with the crate's loss-free configs: every loss off, the working range 150 mm lower
/// (where the plate's vertical mode under constant torques is stable, see the crate), rigid and elastic, with and
/// without the payload, the elastic one with the payload also with part heating and its feedback.
#[test]
fn the_planar_delta_file_passes_the_standard_suite() {
    use std::sync::Mutex;
    static LOSSLESS: Mutex<Vec<Config>> = Mutex::new(Vec::new());
    fn lossless() -> Vec<Config> {
        LOSSLESS.lock().unwrap().clone()
    }
    let shipped = file();
    let mut configs = Vec::new();
    for (elastic, payload) in [(false, true), (false, false), (true, true), (true, false)] {
        let mut cfg = shipped.config();
        for effect in sim_netlist::linkage::LOSSY_EFFECTS {
            cfg.set_text(&format!("effects.{effect} = false")).unwrap();
        }
        cfg.set_text("gearbox.damping_nm_s_rad = 0.0").unwrap();
        cfg.set_text("joint.stop_damping_nm_s_rad = 0.0").unwrap();
        cfg.set_text("workspace.top_z_m = -0.55").unwrap();
        lowered(&mut cfg, -0.15);
        cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
        cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
        cfg.set_text(&format!("effects.part_heating = {}", elastic && payload)).unwrap();
        cfg.set_text(&format!("effects.thermal_feedback = {}", elastic && payload)).unwrap();
        configs.push(cfg);
    }
    *LOSSLESS.lock().unwrap() = configs;
    sim_core::testkit::run_standard_suite(&MachineEntry { suite_configs: lossless, ..*shipped });
}

/// The inverse kinematics at (x, z) = (0.1, -0.5) m by hand (e = 0.15 - 0.05 = 0.1 m, L_a = 0.4 m, L_b = 0.7 m).
/// Arm 1 sees x' = x = e, so its equation A cos t + B sin t + C = 0 has A = 0: B = 2 L_a z = -0.4, C = 0.5^2 + 0.4^2 -
/// 0.7^2 = -0.08, sin t = -C / B = -0.2, t_1 = -asin(0.2) = -0.2013579 rad (-11.537 deg; the elbow out, cos t > 0).
/// Arm 2 sees x' = -0.1: A = -2 L_a (x' - e) = 0.16, B = -0.4, C = 0.2^2 + 0.5^2 + 0.4^2 - 0.7^2 = -0.04, so
/// 0.16 cos t - 0.4 sin t = 0.04; with R = sqrt(0.16^2 + 0.4^2) = 0.430813 and phi = atan2(0.4, 0.16) = 1.190290,
/// cos(t + phi) = 0.04 / R = 0.092848, t + phi = 1.477815, t_2 = 0.287525 rad (16.474 deg; the other root, -2.668 rad,
/// is the elbow in). The motors turn 20 times that. Both elbows then lie 0.7 m from the plate.
#[test]
fn the_linkage_inverse_kinematics_at_a_pose_is_the_hand_calculation() {
    let machine = (file().build)(&file().config()).unwrap();
    let mut motors = [0.0; 2];
    machine.inverse(&[0.1, -0.5], &mut motors);
    let t = motors.map(|m| m / 20.0);
    assert!((t[0] + 0.2f64.asin()).abs() < 1e-12, "arm 1: {}", t[0]);
    assert!((t[0] + 0.2013579).abs() < 1e-7 && (t[1] - 0.287525).abs() < 1e-6, "{t:?}");
    let (r, phi) = (0.16f64.hypot(0.4), 0.4f64.atan2(0.16));
    assert!((t[1] - ((0.04 / r).acos() - phi)).abs() < 1e-12, "arm 2: {}", t[1]);
    for (k, side) in [(0, 1.0), (1, -1.0)] {
        let elbow = [side * (0.1 + 0.4 * t[k].cos()), -0.4 * t[k].sin()];
        assert!(((0.1 - elbow[0]).hypot(-0.5 - elbow[1]) - 0.7).abs() < 1e-12, "forearm {k}");
    }
    let mut back = [0.0; 2];
    machine.forward(&motors, &mut back);
    assert!((back[0] - 0.1).abs() < 1e-12 && (back[1] + 0.5).abs() < 1e-12, "{back:?}");
}

/// Static hold at the centre, 100 mm below the top, as the crate's test works it out: the arm angle from
/// (e + L_a cos t)^2 + (z + L_a sin t)^2 = L_b^2 by halving, dz/dt per arm = (-L_a cos t + (e + L_a cos t) L_a sin t /
/// (z + L_a sin t)) / 2; m_nt = 1.0 + 5 + 2 (1/3) 0.6 = 6.4 kg, m_LG = 1.2 * 0.2 + 0.4 (0.4 + 0.6667 * 0.6) =
/// 0.560 kg m. Each gearbox holds -(g m_LG cos t) + m_nt g dz/dt = -5.490 - 12.868 = -18.358 N m at t = 1.4290 deg
/// (negative: the arm is held up), the motor that over 20: -0.9179 N m.
#[test]
fn the_linkage_holds_the_plate_at_the_centre_with_the_hand_calculated_torque() {
    let mut cfg = file().config();
    cfg.set_text("effects.torque_ripple = false").unwrap();
    let machine = (file().build)(&cfg).unwrap();
    let (e, la, lb, z) = (0.1f64, 0.4f64, 0.7f64, -0.5f64);
    let f = |t: f64| (e + la * t.cos()).powi(2) + (z + la * t.sin()).powi(2) - lb * lb;
    let (mut lo, mut hi) = (-60f64.to_radians(), 90f64.to_radians());
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        if f(lo) * f(mid) <= 0.0 { hi = mid } else { lo = mid }
    }
    let t = 0.5 * (lo + hi);
    let dz = (-la * t.cos() + (e + la * t.cos()) * la * t.sin() / (z + la * t.sin())) / 2.0;
    let (m_nt, m_lg) = (1.0 + 5.0 + 2.0 * (1.0 - 0.6667) * 0.6, 1.2 * 0.2 + 0.4 * (0.4 + 0.6667 * 0.6));
    let by_hand = -G * m_lg * t.cos() + m_nt * G * dz;
    assert!((t.to_degrees() - 1.4290).abs() < 1e-3 && (by_hand + 18.358).abs() < 0.002, "{by_hand} N m");
    let (mut state, mut torque) = (Vec::new(), [0.0; 2]);
    machine.equilibrium(&[0.0, z], &mut state, &mut torque);
    for k in 0..2 {
        assert!((state[k] - t).abs() < 1e-9, "arm angle {}", state[k]);
        assert!((torque[k] - by_hand / 20.0).abs() < 1e-6, "motor torque {} against {}", torque[k], by_hand / 20.0);
    }
}

/// A broken linkage is refused with its key and line.
#[test]
fn a_broken_linkage_is_refused_with_its_key_and_line() {
    let text = std::fs::read_to_string(root().join("machines/planar-delta.toml")).unwrap().replace("\r\n", "\n");
    let known: Vec<Config> = MACHINES.iter().map(|m| m.config()).collect();
    let refused = |from: &str, to: &str| -> (ConfigError, Option<usize>) {
        let broken = text.replacen(from, to, 1);
        assert_ne!(broken, text, "the change applies");
        match sim_netlist::from_text(&broken, true, "broken", &known) {
            Ok(_) => panic!("a broken file loaded: {to}"),
            Err(e) => e,
        }
    };
    let line_with = |needle: &str| text.lines().position(|l| l.contains(needle)).unwrap() + 1;

    // A forearm no longer than the upper arm cannot reach under it.
    let (e, line) = refused("forearm_m = 0.7\n", "forearm_m = 0.35\n");
    assert_eq!(e, ConfigError::Invalid("machine.linkage[delta].forearm_m (longer than the upper arm)".into()));
    assert_eq!(line, Some(line_with("forearm_m = \"geometry.forearm_m\"")));
    // An arm's gearbox that is not there, and one arm twice.
    let (e, _) = refused("arms = [\"gearbox_1\", \"gearbox_2\"]", "arms = [\"gearbox_1\", \"gearbox_3\"]");
    assert_eq!(e, ConfigError::Invalid("machine.linkage[delta].arms = \"gearbox_3\"".into()));
    let (e, _) = refused("arms = [\"gearbox_1\", \"gearbox_2\"]", "arms = [\"gearbox_1\", \"gearbox_1\"]");
    assert!(matches!(e, ConfigError::Invalid(d) if d.starts_with("machine.linkage[delta].arms (two")));
    // A stroke on a linkage's axis, and a belt node beside the linkage.
    let (e, line) = refused("id = \"x\"\n", "id = \"x\"\nstroke_m = 0.8\n");
    assert_eq!(
        e,
        ConfigError::Invalid(
            "machine.axis[x].stroke_m (a linkage's axes take their range from the linkage's working range)".into()
        )
    );
    assert_eq!(line, Some(line_with("id = \"x\"") + 1));
    let (e, _) = refused(
        "[[machine.linkage]]",
        "[[machine.body]]\nid = \"plate\"\nmass_kg = 1.0\naxes = [\"x\"]\n\n[[machine.linkage]]",
    );
    assert_eq!(
        e,
        ConfigError::Invalid(
            "machine.body[plate] (a file with a linkage has axes, motors, gearboxes and the linkage only)".into()
        )
    );
    // A belt effect switched on: a linkage has no belt, and the effect would be ignored without a word.
    let (e, line) = refused("[effects]\n", "[effects]\nbelt_wrap = true\n");
    assert_eq!(e, ConfigError::Invalid("effects.belt_wrap (a linkage has no belt: switch it off)".into()));
    assert_eq!(line, Some(line_with("[effects]") + 1));
    // A spatial linkage's value on the planar one.
    let (e, _) = refused("height_edge_m = \"", "radius_m = 0.4\nheight_edge_m = \"");
    assert_eq!(e, ConfigError::Invalid("machine.linkage[delta].radius_m (not for a linkage of two arms)".into()));
    // A misspelt field of the node.
    let (e, _) = refused("upper_arm_kg = \"", "upper_arm_kgs = \"");
    assert_eq!(e, ConfigError::Unknown("machine.linkage[delta].upper_arm_kgs".into()));
}

fn scissor_file() -> &'static MachineEntry {
    load_file(&root().join("machines/scissor-lift.toml")).unwrap_or_else(|e| panic!("{e}"))
}

/// The scissor lift of machines/scissor-lift.toml held at two heights, rigid, without the screw's and gearbox's loss
/// and the ripple, by hand. The weight is the platform, the payload and half the arms: W = (60 + 200 + 20) 9.80665 =
/// 2745.862 N. Two stages of 1 m arms stand s = h / 2 high with the feet u = sqrt(1 - s^2) apart; the screw pulls the
/// foot with F = W n u / s = W 2 cot(alpha):
/// - at h = 0.6 m: u = 0.953939, cot = 3.179797, F = 17462.57 N, the screw (lead 10 mm, c = 1.591549 mm/rad) holds
///   F c = 27.79254 N m, the motor behind ratio 20 1.389627 N m;
/// - at h = 1.4 m: u = 0.714143, cot = 1.020204, F = 5602.68 N, 8.916941 N m, the motor 0.445847 N m.
///
/// The force ratio between the two, 3.116825, is cot over cot: the lift needs three times the force low down.
#[test]
fn the_scissor_lift_holds_its_platform_with_the_hand_calculated_force_at_two_heights() {
    let mut cfg = scissor_file().config();
    for off in ["joint_elasticity", "gearbox_efficiency", "torque_ripple"] {
        cfg.set_text(&format!("effects.{off} = false")).unwrap();
    }
    let machine = (scissor_file().build)(&cfg).unwrap();
    for (h, force, motor) in [(0.6, 17_462.57, 1.389_627), (1.4, 5602.68, 0.445_847)] {
        let (mut state, mut torque) = (Vec::new(), [0.0]);
        machine.equilibrium(&[h], &mut state, &mut torque);
        assert!((torque[0] - motor).abs() < 1e-6, "h = {h}: motor {} N m against {motor}", torque[0]);
        let mut values = Vec::new();
        machine.signals(&state, &torque, &mut values);
        let at = machine.signal_infos().iter().position(|s| s.id == "scissor.scissor.actuator_force").unwrap();
        assert!((values[at] - force).abs() < 0.01, "h = {h}: {} N on the foot against {force}", values[at]);
        let (mut pos, mut vel) = ([0.0], [0.0]);
        machine.tool_state(&state, &mut pos, &mut vel);
        assert!((pos[0] - h).abs() < 1e-12, "{pos:?}");
    }
    assert!((3.179_797 / 1.020_204f64 - 3.116_825).abs() < 1e-5);
}

/// The scissor lift passes the standard suite (round trip, static hold, determinism, energy without losses, the loss
/// budget with them), rigid and elastic, with and without the payload. The loss-free configs leave gravity out: held
/// by a constant torque the lift is unstable at every height, as it needs less torque the higher it stands (the
/// gravity's stiffness -g m_g h'' is negative, h'' < 0); at mid height (s = 0.5 m) that is 2746 N * 4.05e-5 m/rad2 =
/// 0.111 N m/rad against the 0.094 kg m2 the screw sees (mostly the rotor, 20^2 * 2.1e-4), growing at
/// sqrt(0.111 / 0.094) = 1.09 per second, e^11 over the suite's 10 s: out of its range. The drive's position loop holds
/// it; the default config, with gravity, takes the suite's short runs.
#[test]
fn the_scissor_lift_file_passes_the_standard_suite() {
    use std::sync::Mutex;
    static LOSSLESS: Mutex<Vec<Config>> = Mutex::new(Vec::new());
    fn lossless() -> Vec<Config> {
        LOSSLESS.lock().unwrap().clone()
    }
    let shipped = scissor_file();
    let mut configs = Vec::new();
    for (elastic, payload) in [(false, true), (false, false), (true, true), (true, false)] {
        let mut cfg = shipped.config();
        for effect in sim_netlist::scissor::LOSSY_EFFECTS {
            cfg.set_text(&format!("effects.{effect} = false")).unwrap();
        }
        cfg.set_text("gearbox.damping_nm_s_rad = 0.0").unwrap();
        cfg.set_text(&format!("effects.joint_elasticity = {elastic}")).unwrap();
        cfg.set_text(&format!("effects.payload = {payload}")).unwrap();
        cfg.set_text("effects.gravity = false").unwrap();
        configs.push(cfg);
    }
    *LOSSLESS.lock().unwrap() = configs;
    sim_core::testkit::run_standard_suite(&MachineEntry { suite_configs: lossless, ..*shipped });
}
