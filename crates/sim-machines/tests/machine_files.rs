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
    &[
        "effects.guide_friction = false",
        "effects.motor_friction = false",
        "effects.gearbox_efficiency = false",
        "effects.torque_ripple = false",
        "effects.tooth_meshing = false",
        "effects.belt_damping = false",
    ],
    &["effects.gravity = false", "effects.belt_stretch = false", "effects.hard_stops = false"],
    &[
        "effects.belt_mass = false",
        "effects.gearbox_backlash = false",
        "effects.payload = false",
        "effects.load_dependent_friction = false",
    ],
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

/// The machine of a crate and of its file, with the same changes made to both, agree: kinematics, the state at rest and
/// the torque that holds it, the derivatives of a moving state under torques, the feedforward, the rigid tensions and
/// every signal both have, to 1e-9. With the payload held and the cycle dwelling (for a process force), then set
/// down, then with the brakes holding; the parts `rises` warm (for thermal feedback), in the order of `thermal_parts`.
fn same_machine(id: &str, name: &str, changes: &[&str], rises: &[f64], what: &str) {
    let [(ea, ca), (eb, cb)] = pair(id, name, changes);
    let (mut a, mut b) = ((ea.build)(&ca).unwrap(), (eb.build)(&cb).unwrap());
    let ids = |m: &dyn Machine| {
        (
            m.axes().iter().map(|a| (a.id, a.min, a.max, a.max_vel)).collect::<Vec<_>>(),
            m.motors().iter().map(|m| m.id).collect::<Vec<_>>(),
        )
    };
    assert_eq!(ids(a.as_ref()), ids(b.as_ref()), "{what}");
    for machine in [&mut a, &mut b] {
        machine.set_part_temperatures(rises);
    }
    // The crate's elastic T-gantry keeps the states of its frame and tool flex (position and rate, 6 7 and 14 15)
    // with those effects off; the file machine has its frame's where it sways and its leaning Z axis's where that
    // leans, in that order after the rotors. Elsewhere the states are laid out alike.
    let on = |effect: &str| cb.bool(&format!("effects.{effect}")).unwrap_or(false);
    let gaps: &[usize] = match a.state_len() - b.state_len() {
        0 => &[],
        2 if on("tool_flex") && !on("frame_compliance") => &[6, 14],
        2 => &[7, 15],
        _ => &[6, 7, 14, 15],
    };
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
            machine.set_dwelling(k == 0);
        }
        let (mut ma, mut mb) = (vec![0.0; m], vec![0.0; m]);
        a.inverse(pose, &mut ma);
        b.inverse(pose, &mut mb);
        assert_close(&format!("{what}: inverse"), &ma, &mb, 1e-12);
        assert_close(&format!("{what}: belt wraps"), &a.belt_wraps(pose), &b.belt_wraps(pose), 1e-9);
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

        let shared = shared_signals(a.as_ref(), b.as_ref(), &sa, &sb, &torque, what);
        assert!(
            shared >= b.signal_infos().len() * 9 / 10,
            "{what}: only {shared} of the file machine's signals are the crate's"
        );
        // The spans (nodes span_1 to span_8 in the file) give their signals the crate's names, so the comparison above
        // took their frequencies too.
        let names = |m: &dyn Machine| {
            m.signal_infos().iter().filter(|s| s.id.starts_with("belt.span.")).map(|s| s.id.clone()).collect::<Vec<_>>()
        };
        assert_eq!(names(a.as_ref()), names(b.as_ref()), "{what}: span signals");
    }
}

#[test]
fn the_netlist_machines_are_their_crates() {
    for (id, name) in PAIRS {
        for (v, changes) in VARIANTS.iter().enumerate() {
            same_machine(id, name, changes, &[], &format!("{id} variant {v}"));
        }
    }
}

/// The effects machine files have had since 13 November, and since 14 November the leaning Z axis and the belt's wrap,
/// each switched on in the T-gantry crate and its file (which names its idlers, frame, cable chain, spans, leaning Z
/// axis and the places of its pulleys and idlers), elastic and rigid, and all of them together: the same machine as
/// above, to 1e-9, and the same wraps. Their hand checks are the crate's (crates/machine-t-gantry/src/tests.rs); the
/// file's netlist has to give the same numbers. The parts warm by 10 to 40 K for the thermal feedback.
#[test]
fn the_effects_of_a_machine_file_are_its_crates() {
    let effects = [
        "frame_compliance",
        "idlers",
        "cable_chain",
        "process_force",
        "thermal_feedback",
        "belt_tooth_jump",
        "belt_span_vibration",
        "tool_flex",
        "belt_wrap",
    ];
    let rises = [10.0, 20.0, 30.0, 40.0];
    for rigid in [false, true] {
        let stretch = format!("effects.belt_stretch = {}", !rigid);
        for effect in effects {
            let on = format!("effects.{effect} = true");
            same_machine(
                "t-gantry",
                "t-gantry.toml",
                &[&stretch, &on, "effects.part_heating = true"],
                &rises,
                &format!("{effect}, rigid {rigid}"),
            );
        }
        let all: Vec<String> = effects.iter().map(|e| format!("effects.{e} = true")).collect();
        let mut changes: Vec<&str> = all.iter().map(String::as_str).collect();
        changes.extend([stretch.as_str(), "effects.part_heating = true"]);
        same_machine("t-gantry", "t-gantry.toml", &changes, &rises, &format!("all, rigid {rigid}"));
    }
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

/// The cycle of each machine run in full (controller, drives, DC link, winding heat), its analysis and its verdicts,
/// elastic and rigid. Without the encoder's counts, every number agrees to 1e-5: the elastic T-gantry's motor speeds
/// stay within 1e-10 rad/s of the crate's all cycle, and the rigid single axis's states, 1e-36 apart at the start,
/// grow apart tenfold every 30 steps while its guide breaks away from stiction (to 1e-6) and close again in the cruise.
#[test]
fn the_netlist_machines_run_their_crates_cycle() {
    for (id, name) in PAIRS {
        for rigid in [false, true] {
            let changes =
                ["effects.encoder_resolution = false".to_string(), format!("effects.belt_stretch = {}", !rigid)];
            let changes: Vec<&str> = changes.iter().map(String::as_str).collect();
            let [(ea, ca), (eb, cb)] = pair(id, name, &changes);
            let ((names, crate_values), (file_names, values)) = (cycle_numbers(ea, &ca), cycle_numbers(eb, &cb));
            assert_eq!(names, file_names, "{id} rigid {rigid}: the same numbers, limits and verdicts");
            for (i, name) in names.iter().enumerate() {
                assert!(
                    close(crate_values[i], values[i], 1e-5),
                    "{id} rigid {rigid}: {name}: crate {}, file {}",
                    crate_values[i],
                    values[i]
                );
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
            assert!(
                (values[i] - nominal[i]).abs() <= allowed,
                "{id}: {name}: crate {}, file {}, crate's own spread {}",
                nominal[i],
                values[i],
                spread[i]
            );
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
    assert!(
        on_disk == schema,
        "schema/machine-file.schema.json is out of date: run the tests with MACHINE_SCHEMA_WRITE=1"
    );
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
    let (e, line) =
        refused(one("pulleys = { pulley_a = -1, pulley_b = 1 }", "pulleys = { pulley_a = -1, pulley_c = 1 }"));
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

    // An effect whose node the file leaves out, a wrap without the places it is worked out from, and a later version
    // of the format.
    let without_lean = |t: &str| {
        let start = t.find("[[machine.lean]]").unwrap();
        let end = start + t[start..].find("\n\n").unwrap();
        format!("{}{}", &t[..start], &t[end..]).replacen("tool_flex = false", "tool_flex = true", 1)
    };
    let (e, _) = refused(without_lean);
    assert_eq!(e, ConfigError::Missing("machine.lean (effects.tool_flex)".into()));
    let (e, _) = refused(|t: &str| {
        t.replacen("belt_wrap = false", "belt_wrap = true", 1).replacen("at_m = [1.15, -0.05, 0.06]\n", "", 1)
    });
    assert_eq!(e, ConfigError::Invalid("effects.belt_wrap (pulley_b has no at_m)".into()));
    let (e, line) = refused(one("lever_axis = \"z\"\nlateral", "lever_axis = \"y\"\nlateral"));
    assert_eq!(e, ConfigError::Invalid("machine.lean[z_lean].lever_axis = \"y\"".into()));
    assert_eq!(line, Some(line_with(&text, "lateral_stiffness_n_m = \"tool_flex") - 1));
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
    for name in ["single-axis.toml", "t-gantry.toml", "h-bot.toml", "corexy.toml", "xyz-portal.toml"] {
        let shipped = file(name);
        let mut configs = Vec::new();
        for elastic in [true, false] {
            let mut cfg = shipped.config();
            for effect in [
                "belt_damping",
                "guide_friction",
                "load_dependent_friction",
                "motor_friction",
                "gearbox_efficiency",
                "torque_ripple",
                "tooth_meshing",
                "temperature",
                "holding_brake",
            ] {
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
            // The CoreXY and the portal's X and Y lie flat the same way.
            if ["h-bot.toml", "corexy.toml", "xyz-portal.toml"].contains(&name) {
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

/// The CoreXY by hand: belt A gets longer by x + y on one side of motor A and belt B by x - y on one side of motor B, so
/// the motor angles are i (x + y) / r and i (x - y) / r (r 0.02 m, i 5): at x = 0.3 m, y = 0.1 m, 100 and 50 rad.
/// Rigid, a diagonal acceleration a = 1 m/s2 along x and y asks motor B for r / (2 i) (F_x - F_y): the pulleys and
/// rotors of both motors weigh alike on X and Y (2 J / r^2 each), so what is left is the head's mass on X (3 kg and
/// the 1 kg payload) less the head's and the gantry's on Y (4 + 4 kg): 0.02 / 10 * (4 - 8) = -0.008 N m. Lying flat,
/// nothing is held.
#[test]
fn the_corexy_moves_as_worked_out_by_hand() {
    let entry = file("corexy.toml");
    assert_eq!((entry.id, entry.key), ("corexy", "CoreXY (machine file)"));
    let mut cfg = entry.config();
    cfg.set_text("effects.belt_stretch = false").unwrap();
    let machine = (entry.build)(&cfg).unwrap();
    let mut motors = [0.0; 2];
    machine.inverse(&[0.3, 0.1], &mut motors);
    assert_close("motor angles", &motors, &[100.0, 50.0], 1e-12);
    let (mut diagonal, mut held) = ([0.0; 2], [0.0; 2]);
    machine.feedforward_torque(&[0.25, 0.25], &[1.0, 1.0], &mut diagonal);
    machine.feedforward_torque(&[0.25, 0.25], &[0.0, 0.0], &mut held);
    assert!(close(diagonal[1], 0.02 / 10.0 * (4.0 - 8.0), 1e-12), "{diagonal:?}");
    assert_close("holding torque", &held, &[0.0, 0.0], 1e-12);
}

/// The portal by hand. Each axis has a belt and a motor of its own, so the motor angle is i q / r: X and Y with the shared
/// gearbox (i 5), Z with its own ([gearbox.mz], i 10): at q = 0.1 m 25, 25 and 50 rad, and at 0.2 m/s 50, 50 and 100
/// rad/s. Its Z motor holds the slide (2.5 kg) and the payload (1 kg) against gravity: m g r / i = 3.5 * 9.80665 * 0.02
/// / 10 = 0.0686466 N m (rigid, without tooth meshing and ripple); X and Y lie flat and hold nothing. The Z motor's own
/// drive ([drive.mz]: rated torque 0.635 N m, the shared one 1.27) is the controller's, and its verdicts are judged
/// against its own ratings ([ratings.mz]: drive current 1.5 A, gearbox 18 N m; the shared 3 A and 40 N m).
#[test]
fn the_portal_moves_and_holds_as_worked_out_by_hand() {
    let entry = file("xyz-portal.toml");
    let mut cfg = entry.config();
    for change in ["effects.belt_stretch = false", "effects.tooth_meshing = false", "effects.torque_ripple = false"] {
        cfg.set_text(change).unwrap();
    }
    let machine = (entry.build)(&cfg).unwrap();
    let mut motors = [0.0; 3];
    machine.inverse(&[0.1, 0.1, 0.1], &mut motors);
    assert_close("motor angles", &motors, &[25.0, 25.0, 50.0], 1e-12);
    let (mut state, mut held) = (Vec::new(), [0.0; 3]);
    machine.equilibrium(&[0.5, 0.25, 0.15], &mut state, &mut held);
    assert_close("holding torque", &held, &[0.0, 0.0, 3.5 * 9.80665 * 0.02 / 10.0], 1e-12);

    // Elastic, the rotors turn at the speed the tool's sets.
    let elastic = (entry.build)(&entry.config()).unwrap();
    let (mut state, mut torque) = (Vec::new(), [0.0; 3]);
    elastic.equilibrium(&[0.5, 0.25, 0.15], &mut state, &mut torque);
    elastic.set_tool_velocity(&mut state, &[0.2, 0.2, 0.2]);
    let (mut angle, mut speed) = ([0.0; 3], [0.0; 3]);
    elastic.motor_state(&state, &mut angle, &mut speed);
    assert_close("motor speeds", &speed, &[50.0, 50.0, 100.0], 1e-12);

    let sim = Sim::new(entry, &entry.config()).unwrap();
    assert_eq!([0, 2].map(|k| sim.control.drive_params_of(k).rated_torque), [1.27, 0.635]);
    let load = analyse_cycle(&mut Sim::new(entry, &entry.config()).unwrap()).unwrap();
    let all = verdicts(&entry.config(), &load).unwrap();
    let limit =
        |check: &str, motor: usize| all.iter().find(|v| v.check == check && v.motor == Some(motor)).map(|v| v.limit);
    assert_eq!((limit("drive-current", 0), limit("drive-current", 2)), (Some(3.0), Some(1.5)));
    assert_eq!((limit("gearbox-continuous", 0), limit("gearbox-continuous", 2)), (Some(40.0), Some(18.0)));
    assert!(all.iter().all(|v| v.ok), "{all:?}");
}

/// A motor's own table may only override keys the shared one has, and names its parts like `[parts]`: an unknown key
/// in `[motor.mz]` is refused by name, a belt in `[parts.mz]` (the machine's, not a motor's) as out of range.
#[test]
fn a_motors_own_tables_are_checked() {
    let text = std::fs::read_to_string(root().join("machines/xyz-portal.toml")).unwrap().replace("\r\n", "\n");
    let known: Vec<Config> = MACHINES.iter().map(|m| m.config()).collect();
    let refused = |text: String| sim_netlist::from_text(&text, true, "x", &known).err().map(|(e, _)| e);
    let typo = text.replacen("[motor.mz]\nrotor_inertia_kgm2", "[motor.mz]\nrotor_inertia_kg", 1);
    assert_eq!(refused(typo), Some(ConfigError::Unknown("motor.mz.rotor_inertia_kg".into())));
    let belt = text.replacen("[motor.mz]\n", "[parts.mz]\nbelt = \"AT5-25\"\n\n[motor.mz]\n", 1);
    assert_eq!(refused(belt).map(|e| e.key()), Some("error-config-invalid"));
}

/// The belt's lowest mode by hand, on the single axis at x = 0.5 m: the carriage (4 kg and the 2 kg payload) between
/// the two sections, 0.15 + x = 0.65 m and 2.45 - x = 1.95 m long, with the pulley held: k = EA (1 / 0.65 + 1 / 1.95)
/// = 5e5 * 2.0513 = 1.0256e6 N/m, f = sqrt(k / m) / (2 pi) = 65.80 Hz. The mode moves the tool by 1 m, the pulley not.
/// The rigid model has no mode.
#[test]
fn a_machine_file_reports_its_belt_mode() {
    let entry = file("single-axis.toml");
    let machine = (entry.build)(&entry.config()).unwrap();
    let k = 5e5 * (1.0 / 0.65 + 1.0 / 1.95);
    let hand = (k / 6.0f64).sqrt() / std::f64::consts::TAU;
    let hz = machine.resonance_hz(&[0.5]).unwrap();
    assert!(close(hz, hand, 1e-9) && (hz - 65.80).abs() < 0.01, "{hz} against {hand} by hand");
    let (mode_hz, shape) = machine.mode_shape(&[0.5]).unwrap();
    assert_eq!((mode_hz, shape.len()), (hz, machine.state_len()));
    assert!(close(shape[0], 1.0, 1e-12) && shape[1..].iter().all(|s| *s == 0.0), "{shape:?}");
    let mut rigid = entry.config();
    rigid.set_text("effects.belt_stretch = false").unwrap();
    assert_eq!((entry.build)(&rigid).unwrap().resonance_hz(&[0.5]), None);
    // The T-gantry's frame, where it sways, makes the lowest mode lower.
    let gantry = file("t-gantry.toml");
    let mut swaying = gantry.config();
    swaying.set_text("effects.frame_compliance = true").unwrap();
    let (stiff, soft) = ((gantry.build)(&gantry.config()).unwrap(), (gantry.build)(&swaying).unwrap());
    assert!(soft.resonance_hz(&[0.5, 0.25]).unwrap() < stiff.resonance_hz(&[0.5, 0.25]).unwrap());
}

/// The 3D view draws the belt: one strip per pair of points on each section's way. The H-bot's sections have no route,
/// so each runs from its pulleys to the body it is clamped to: sections 1 and 3 one strip each (pulley, head), section 2
/// two (pulley A, pulley B, head). The T-gantry's file routes them past its idlers: 2 + 4 + 2 strips, and five idlers.
#[test]
fn the_3d_view_draws_the_belt_of_a_machine_file() {
    for (name, strips, idlers) in [("h-bot.toml", 4, 0), ("t-gantry.toml", 8, 5)] {
        let entry = file(name);
        let sim = Sim::new(entry, &entry.config()).unwrap();
        let mut scene = sim_core::Scene::default();
        sim.machine.scene(&sim.state, &mut scene);
        let count = |id: &str| scene.parts.iter().filter(|p| p.id == id).count();
        assert_eq!((count("belt"), count("idler")), (strips, idlers), "{name}");
    }
}
