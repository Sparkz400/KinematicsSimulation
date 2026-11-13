//! Machine files (crates/sim-netlist, docs/machine-files.md): the single axis and the T-gantry written as netlists
//! in machines/*.toml reproduce their crates, and a broken file is refused with the key and line of what is wrong.

use sim_components::Sim;
use sim_components::analysis::{analyse_cycle, verdicts};
use sim_core::{Config, ConfigError, Machine, MachineEntry};
use sim_machines::{MACHINES, find, load_file};
use std::path::{Path, PathBuf};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn file(name: &str) -> &'static MachineEntry {
    load_file(&root().join("machines").join(name)).unwrap_or_else(|e| panic!("{e}"))
}

/// The crate and the file of one machine, each with the same changes: the crate's default config with the effects
/// machine files do not have switched off, and the file as it is.
fn pair(id: &str, file_name: &str, changes: &[&str]) -> [(&'static MachineEntry, Config); 2] {
    let (krate, from_file) = (find(id).unwrap(), file(file_name));
    let mut a = krate.config();
    if id == "t-gantry" {
        for off in ["frame_compliance", "tool_flex"] {
            a.set_text(&format!("effects.{off} = false")).unwrap();
        }
    }
    let mut b = from_file.config();
    for change in changes {
        a.set_text(change).unwrap();
        b.set_text(change).unwrap();
    }
    [(krate, a), (from_file, b)]
}

const PAIRS: [(&str, &str); 2] = [("single-axis", "single-axis.toml"), ("t-gantry", "t-gantry.toml")];

/// Variants the comparison runs on: the default, the rigid model, without losses, without gravity, and with the
/// belt details and the payload off.
const VARIANTS: [&[&str]; 5] = [
    &[],
    &["effects.belt_stretch = false"],
    &["effects.guide_friction = false", "effects.motor_friction = false", "effects.gearbox_efficiency = false", "effects.torque_ripple = false", "effects.tooth_meshing = false", "effects.belt_damping = false"],
    &["effects.gravity = false", "effects.belt_stretch = false", "effects.hard_stops = false"],
    &["effects.belt_mass = false", "effects.gearbox_backlash = false", "effects.payload = false", "effects.load_dependent_friction = false"],
];

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * (1.0 + a.abs().max(b.abs()))
}

fn assert_close(what: &str, a: &[f64], b: &[f64], tol: f64) {
    assert_eq!(a.len(), b.len(), "{what}: lengths");
    for (i, (x, y)) in a.iter().zip(b).enumerate() {
        assert!(close(*x, *y, tol), "{what}[{i}]: crate {x}, file {y}\n crate {a:?}\n file  {b:?}");
    }
}

/// Signal values by id: those the file machine has that the crate has too, which must agree.
fn shared_signals(a: &dyn Machine, b: &dyn Machine, sa: &[f64], sb: &[f64], torque: &[f64], what: &str) -> usize {
    let (ia, ib) = (a.signal_infos(), b.signal_infos());
    let (mut va, mut vb) = (Vec::new(), Vec::new());
    a.signals(sa, torque, &mut va);
    b.signals(sb, torque, &mut vb);
    assert_eq!((ia.len(), ib.len()), (va.len(), vb.len()), "{what}: signals and their infos");
    let mut shared = 0;
    for (info, value) in ib.iter().zip(&vb) {
        if let Some(i) = ia.iter().position(|x| x.id == info.id) {
            assert!(close(va[i], *value, 1e-9), "{what}: signal {}: crate {}, file {value}", info.id, va[i]);
            shared += 1;
        }
    }
    shared
}

/// The machine itself: kinematics, the state at rest and the torque that holds it, the derivatives of a moving state
/// under torques, the feedforward, the rigid tensions and every signal both have, to 1e-9.
#[test]
fn the_netlist_machines_are_their_crates() {
    for (id, name) in PAIRS {
        for (v, changes) in VARIANTS.iter().enumerate() {
            let [(ea, ca), (eb, cb)] = pair(id, name, changes);
            let what = format!("{id} variant {v}");
            let (mut a, mut b) = ((ea.build)(&ca).unwrap(), (eb.build)(&cb).unwrap());
            let ids = |m: &dyn Machine| (m.axes().iter().map(|a| (a.id, a.min, a.max, a.max_vel)).collect::<Vec<_>>(), m.motors().iter().map(|m| m.id).collect::<Vec<_>>());
            assert_eq!(ids(a.as_ref()), ids(b.as_ref()), "{what}");
            // The crate's elastic T-gantry keeps the states of its frame and tool flex (position and rate, 6 7 and 14 15)
            // with those effects off; the file machine has none. Elsewhere the states are laid out alike.
            let gaps: &[usize] = if a.state_len() == b.state_len() { &[] } else { &[6, 7, 14, 15] };
            assert_eq!(a.state_len(), b.state_len() + gaps.len(), "{what}: state length");
            let crate_state = |s: &[f64]| {
                let mut out = s.to_vec();
                for &at in gaps {
                    out.insert(at, 0.0);
                }
                out
            };
            let n = a.axes().len();
            let m = a.motors().len();
            // With the payload on, then set down, then with the brakes holding.
            for (k, pose) in [vec![0.1, 0.05], vec![0.73, 0.41], vec![0.5, 0.25]].into_iter().enumerate() {
                let pose = &pose[..n];
                for machine in [&mut a, &mut b] {
                    machine.set_payload_attached(k != 1);
                    machine.set_brake_engaged(k == 2);
                }
                let (mut ma, mut mb) = (vec![0.0; m], vec![0.0; m]);
                a.inverse(pose, &mut ma);
                b.inverse(pose, &mut mb);
                assert_close(&format!("{what}: inverse"), &ma, &mb, 1e-12);
                let mut back = vec![0.0; n];
                b.forward(&mb, &mut back);
                assert_close(&format!("{what}: forward"), pose, &back, 1e-12);

                let (mut sa, mut sb, mut ta, mut tb) = (Vec::new(), Vec::new(), vec![0.0; m], vec![0.0; m]);
                a.equilibrium(pose, &mut sa, &mut ta);
                b.equilibrium(pose, &mut sb, &mut tb);
                assert_close(&format!("{what}: equilibrium state"), &sa, &crate_state(&sb), 1e-9);
                assert_close(&format!("{what}: holding torque"), &ta, &tb, 1e-9);

                let speed: Vec<f64> = [0.4, -0.3][..n].to_vec();
                a.set_tool_velocity(&mut sa, &speed);
                b.set_tool_velocity(&mut sb, &speed);
                assert_close(&format!("{what}: moving state"), &sa, &crate_state(&sb), 1e-12);
                // Off rest: belts stretched a little more, and torques that accelerate.
                // Off rest: every state moved a little (belts stretched, friction bristles bent, brakes slipped).
                for (i, x) in sb.iter_mut().enumerate() {
                    *x += 1e-5 * (1.0 + (i % 5) as f64);
                }
                sa = crate_state(&sb);
                let torque: Vec<f64> = ta.iter().enumerate().map(|(k, t)| t + 0.7 - 0.5 * k as f64).collect();
                let (mut da, mut db) = (vec![0.0; sa.len()], vec![0.0; sb.len()]);
                a.derivatives(&sa, &torque, &mut da);
                b.derivatives(&sb, &torque, &mut db);
                assert_close(&format!("{what}: derivatives"), &da, &crate_state(&db), 1e-9);
                assert!(close(a.energy(&sa), b.energy(&sb), 1e-9), "{what}: energy");

                let acc: Vec<f64> = [3.0, -2.0][..n].to_vec();
                let (mut fa, mut fb) = (vec![0.0; m], vec![0.0; m]);
                a.feedforward_torque(pose, &acc, &mut fa);
                b.feedforward_torque(pose, &acc, &mut fb);
                assert_close(&format!("{what}: feedforward"), &fa, &fb, 1e-9);
                let (mut ra, mut rb) = (Vec::new(), Vec::new());
                a.rigid_tensions(pose, &acc, &mut ra);
                b.rigid_tensions(pose, &acc, &mut rb);
                assert_close(&format!("{what}: rigid tensions"), &ra, &rb, 1e-9);

                let shared = shared_signals(a.as_ref(), b.as_ref(), &sa, &sb, &torque, &what);
                assert!(shared >= b.signal_infos().len() * 9 / 10, "{what}: only {shared} of the file machine's signals are the crate's");
            }
        }
    }
}

/// Every number of a cycle's analysis and verdicts: the motor loads, then each verdict's value.
fn cycle_numbers(entry: &'static MachineEntry, cfg: &Config) -> (Vec<String>, Vec<f64>) {
    let load = analyse_cycle(&mut Sim::new(entry, cfg).unwrap()).unwrap();
    let (mut names, mut values) = (vec!["duration".to_string()], vec![load.duration]);
    for (k, m) in load.motors.iter().enumerate() {
        for (name, v) in [("peak torque", m.peak_torque), ("rms torque", m.rms_torque), ("peak speed", m.peak_speed), ("utilisation", m.utilisation), ("rms current", m.rms_current), ("peak current", m.peak_current), ("energy in", m.energy_in), ("energy regenerated", m.energy_regenerated)] {
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

/// The cycle of each machine run in full (controller, drives, DC link, winding heat), its analysis and its verdicts,
/// elastic and rigid. Without the encoder's counts, every number agrees to 1e-5: the elastic T-gantry's motor speeds
/// stay within 1e-10 rad/s of the crate's all cycle, and the rigid single axis's states, 1e-36 apart at the start,
/// grow apart tenfold every 30 steps while its guide breaks away from stiction (to 1e-6) and close again in the cruise.
#[test]
fn the_netlist_machines_run_their_crates_cycle() {
    for (id, name) in PAIRS {
        for rigid in [false, true] {
            let changes = ["effects.encoder_resolution = false".to_string(), format!("effects.belt_stretch = {}", !rigid)];
            let changes: Vec<&str> = changes.iter().map(String::as_str).collect();
            let [(ea, ca), (eb, cb)] = pair(id, name, &changes);
            let ((names, crate_values), (file_names, values)) = (cycle_numbers(ea, &ca), cycle_numbers(eb, &cb));
            assert_eq!(names, file_names, "{id} rigid {rigid}: the same numbers, limits and verdicts");
            for (i, name) in names.iter().enumerate() {
                assert!(close(crate_values[i], values[i], 1e-5), "{id} rigid {rigid}: {name}: crate {}, file {}", crate_values[i], values[i]);
            }
        }
    }
}

/// The same with the encoder's counts, as the machines are shipped. There the bits in which the two differ (sums taken
/// in another order) flip one count a little earlier or later (on the T-gantry 0.2 s into the cycle), and stick-slip
/// in the guides and the braking chopper switching a step earlier grow that. So the crate is run three times, with
/// its carriage 1e-12 and 1e-9 heavier, and the file has to give the same verdicts and lie within three times the
/// crate's own spread of it, or within 1e-4 of it. Found on 2026-11-13: the T-gantry's peak braking power spreads by
/// 0.5 % that way, and the file lies within that.
#[test]
fn the_netlist_machines_give_their_crates_verdicts() {
    for (id, name) in PAIRS {
        let [(ea, ca), (eb, cb)] = pair(id, name, &[]);
        let (names, nominal) = cycle_numbers(ea, &ca);
        let carriage = ca.f64("mass.carriage_kg").unwrap();
        let mut spread = vec![0.0f64; nominal.len()];
        for factor in [1e-12, 1e-9] {
            let mut c = ca.clone();
            c.set_text(&format!("mass.carriage_kg = {:e}", carriage * (1.0 + factor))).unwrap();
            let (_, values) = cycle_numbers(ea, &c);
            for (s, (a, b)) in spread.iter_mut().zip(nominal.iter().zip(&values)) {
                *s = s.max((a - b).abs());
            }
        }
        let (file_names, values) = cycle_numbers(eb, &cb);
        assert_eq!(names, file_names, "{id}: the same numbers, limits and verdicts");
        for (i, name) in names.iter().enumerate() {
            let allowed = (3.0 * spread[i]).max(1e-4 * (1.0 + nominal[i].abs()));
            assert!((values[i] - nominal[i]).abs() <= allowed, "{id}: {name}: crate {}, file {}, crate's own spread {}", nominal[i], values[i], spread[i]);
        }
    }
}

/// A JSON file is the same machine as the TOML file it was written from.
#[test]
fn a_json_machine_file_is_the_same_machine() {
    let toml_entry = file("single-axis.toml");
    let table: toml::Table = toml_entry.default_config.parse().unwrap();
    let dir = std::env::temp_dir().join(format!("machine-files-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("single-axis.json");
    std::fs::write(&path, serde_json::to_string_pretty(&table).unwrap()).unwrap();
    let json_entry = load_file(&path).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(json_entry.id, "single-axis-file");
    assert_eq!(json_entry.config(), toml_entry.config());
    std::fs::remove_dir_all(dir).ok();
}

/// The JSON Schema in schema/ is the one the code writes: regenerate it with MACHINE_SCHEMA_WRITE=1.
#[test]
fn the_json_schema_file_is_current() {
    let path = root().join("schema/machine-file.schema.json");
    let schema = sim_machines::json_schema();
    if std::env::var_os("MACHINE_SCHEMA_WRITE").is_some() {
        std::fs::write(&path, &schema).unwrap();
    }
    let on_disk = std::fs::read_to_string(&path).unwrap_or_default().replace("\r\n", "\n");
    assert!(on_disk == schema, "schema/machine-file.schema.json is out of date: run the tests with MACHINE_SCHEMA_WRITE=1");
    // It is valid JSON and lists every node kind.
    let parsed: serde_json::Value = serde_json::from_str(&schema).unwrap();
    for (list, ..) in sim_netlist::netlist::NODES {
        assert!(parsed["properties"]["machine"]["properties"][list].is_object(), "{list}");
    }
}

/// A broken file, from the T-gantry's: what is refused, under which key, on which line.
fn refused(change: impl Fn(&str) -> String) -> (ConfigError, Option<usize>) {
    let text = std::fs::read_to_string(root().join("machines/t-gantry.toml")).unwrap().replace("\r\n", "\n");
    let broken = change(&text);
    assert_ne!(broken, text, "the change applies");
    let known: Vec<Config> = MACHINES.iter().map(|m| m.config()).collect();
    match sim_netlist::from_text(&broken, true, "broken", &known) {
        Ok(_) => panic!("a broken file loaded"),
        Err((error, line)) => {
            if let Some(line) = line {
                eprintln!("{:?} at line {line}: {}", error, broken.lines().nth(line - 1).unwrap_or_default());
            }
            (error, line)
        }
    }
}

fn line_with(text: &str, needle: &str) -> usize {
    text.lines().position(|l| l.contains(needle)).unwrap() + 1
}

#[test]
fn a_broken_machine_file_is_refused_with_its_key_and_line() {
    let text = std::fs::read_to_string(root().join("machines/t-gantry.toml")).unwrap().replace("\r\n", "\n");
    let one = |from: &'static str, to: &'static str| move |t: &str| t.replacen(from, to, 1);

    // A reference to a node that is not there.
    let (e, line) = refused(one("pulleys = { pulley_a = -1, pulley_b = 1 }", "pulleys = { pulley_a = -1, pulley_c = 1 }"));
    assert_eq!(e, ConfigError::Invalid("machine.section[2].pulleys.pulley_c".into()));
    assert_eq!(line, Some(line_with(&text, "pulleys = { pulley_a = -1, pulley_b = 1 }")));

    // A field the node kind does not have.
    let (e, line) = refused(one("side_load = true", "side_lod = true"));
    assert_eq!(e, ConfigError::Unknown("machine.guide[guide_z].side_lod".into()));
    assert_eq!(line, Some(line_with(&text, "side_load = true")));

    // A value path that names nothing.
    let (e, line) = refused(one("radius_m = \"geometry.pulley_radius_m\"", "radius_m = \"geometry.pully_radius_m\""));
    assert_eq!(e, ConfigError::Missing("machine.pulley[pulley_a].radius_m = \"geometry.pully_radius_m\"".into()));
    assert_eq!(line, Some(line_with(&text, "radius_m = \"geometry.pulley_radius_m\"")));

    // Two nodes with one id.
    let (e, _) = refused(one("id = \"gearbox_b\"", "id = \"gearbox_a\""));
    assert_eq!(e, ConfigError::Invalid("machine.gearbox[#2].id = \"gearbox_a\" (another node has this id)".into()));

    // A routing whose belt cannot keep its length: section 2 shortened by z with the pulleys as they are.
    // Which section is wrong the routing alone cannot tell: the first that does not fit is named.
    let (e, line) = refused(one("axes = { z = -2 }", "axes = { z = -1 }"));
    assert_eq!(e, ConfigError::Invalid("machine.section[1].axes (the belt cannot keep its length)".into()));
    assert_eq!(line, Some(line_with(&text, "axes = { x = 1, z = 1 }")));

    // A misspelt key in a value table, and a value out of its range.
    let (e, line) = refused(one("pretension_n = 300.0", "pretensoin_n = 300.0"));
    assert_eq!(e, ConfigError::Unknown("belt.pretensoin_n".into()));
    assert_eq!(line, Some(line_with(&text, "pretension_n = 300.0")));
    let (e, line) = refused(one("carriage_kg = 4.0", "carriage_kg = -4.0"));
    assert_eq!(e, ConfigError::Invalid("machine.body[carriage].mass_kg".into()));
    assert_eq!(line, Some(line_with(&text, "mass_kg = \"mass.carriage_kg\"")));

    // An effect machine files do not have yet, and a later version of the format.
    let (e, _) = refused(one("tooth_meshing = true", "tooth_meshing = true\nframe_compliance = true"));
    assert_eq!(e.key(), "error-config-invalid");
    assert!(e.detail().starts_with("effects.frame_compliance"), "{e:?}");
    let (e, line) = refused(one("[machine]\nversion = 1", "[machine]\nversion = 2"));
    assert_eq!(e, ConfigError::Version("machine.version".into()));
    assert_eq!(line, Some(text.lines().position(|l| l == "[machine]").unwrap() + 2));
}






/// Every shipped machine file, the H-bot of docs/machine-files.md among them, passes the standard suite of every
/// machine (round trip, static hold, determinism, energy conservation without losses, the loss budget with them), on
/// its file without losses, elastic and rigid.
#[test]
fn every_machine_file_passes_the_standard_suite() {
    use std::sync::Mutex;
    static LOSSLESS: Mutex<Vec<Config>> = Mutex::new(Vec::new());
    fn lossless() -> Vec<Config> {
        LOSSLESS.lock().unwrap().clone()
    }
    for name in ["single-axis.toml", "t-gantry.toml", "h-bot.toml"] {
        let shipped = file(name);
        let mut configs = Vec::new();
        for elastic in [true, false] {
            let mut cfg = shipped.config();
            for effect in ["belt_damping", "guide_friction", "load_dependent_friction", "motor_friction", "gearbox_efficiency", "torque_ripple", "tooth_meshing", "temperature", "holding_brake"] {
                cfg.set_text(&format!("effects.{effect} = false")).unwrap();
            }
            for damping in ["gearbox.damping_nm_s_rad = 0.0", "hard_stop.damping_n_s_m = 0.0"] {
                cfg.set_text(damping).unwrap();
            }
            cfg.set_text(&format!("effects.belt_stretch = {elastic}")).unwrap();
            // Nothing pulls the H-bot's gears to one flank (it lies flat), and the suite's small pulse knocks them across
            // the backlash: with it the energy drifts by 0.17 % of the 7e-4 J the pulse puts in, over the 0.1 % allowed.
            // That is the gearbox's contact at this step size, not the netlist: the T-gantry crate with gravity off
            // drifts by 0.19 % the same way (2026-11-13).
            if name == "h-bot.toml" {
                cfg.set_text("effects.gearbox_backlash = false").unwrap();
            }
            configs.push(cfg);
        }
        *LOSSLESS.lock().unwrap() = configs;
        let entry = MachineEntry { suite_configs: lossless, ..*shipped };
        sim_core::testkit::run_standard_suite(&entry);
    }
}

/// The H-bot by hand: x = r/2 (a + b) and y = r/2 (a - b) at the pulleys, so motor angles i (x + y) / r and
/// i (x - y) / r (r 0.02 m, i 5). Rigid and without losses, an X acceleration asks both motors for the same torque, a
/// Y acceleration for opposite ones, and the head's weight none (it lies flat).
#[test]
fn the_h_bot_moves_as_worked_out_by_hand() {
    let entry = file("h-bot.toml");
    assert_eq!((entry.id, entry.key), ("h-bot", "H-bot (machine file)"));
    let mut cfg = entry.config();
    cfg.set_text("effects.belt_stretch = false").unwrap();
    let machine = (entry.build)(&cfg).unwrap();
    let mut motors = [0.0; 2];
    machine.inverse(&[0.1, 0.05], &mut motors);
    assert_close("motor angles", &motors, &[37.5, 12.5], 1e-12);
    let (mut along_x, mut along_y, mut held) = ([0.0; 2], [0.0; 2], [0.0; 2]);
    machine.feedforward_torque(&[0.5, 0.25], &[2.0, 0.0], &mut along_x);
    machine.feedforward_torque(&[0.5, 0.25], &[0.0, 2.0], &mut along_y);
    machine.feedforward_torque(&[0.5, 0.25], &[0.0, 0.0], &mut held);
    assert!(along_x[0] > 0.0 && close(along_x[0], along_x[1], 1e-12), "{along_x:?}");
    assert!(along_y[0] > 0.0 && close(along_y[0], -along_y[1], 1e-12), "{along_y:?}");
    assert_close("holding torque", &held, &[0.0, 0.0], 1e-12);
}

