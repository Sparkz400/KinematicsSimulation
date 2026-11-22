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

/// The crate's elastic T-gantry keeps the states of its frame, its tool flex and its beam (each a position and a rate, in
/// that order after the rotors, the positions then the rates) with those effects off; the file machine has the frame's
/// where it sways, the leaning Z axis's where that leans and the beam's where it is a mode. Elsewhere the states are laid
/// out alike: the crate's state of the file machine's `s` under the file's config `cb`.
fn crate_state(id: &str, cb: &Config, s: &[f64]) -> Vec<f64> {
    let on = |effect: &str| cb.bool(&format!("effects.{effect}")).unwrap_or(false);
    if !(on("belt_stretch") && id == "t-gantry") {
        return s.to_vec();
    }
    let extra = [on("frame_compliance"), on("tool_flex"), on("beam_vertical_mode")];
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
    let crate_state = |s: &[f64]| crate_state(id, &cb, s);
    assert_eq!(a.state_len(), crate_state(&vec![0.0; b.state_len()]).len(), "{what}: state length");
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
        // The momentum over ground, which does not depend on where either takes its origin: the bodies, the frame's
        // sway, the belt's runs and (with the beam's mode) the beam's own mass.
        if let (Some(fa), Some(fb)) = (a.frame_momentum(&sa), b.frame_momentum(&sb)) {
            assert_close(&format!("{what}: linear momentum"), &fa.linear, &fb.linear, 1e-9);
        }

        let acc: Vec<f64> = [3.0, -2.0][..n].to_vec();
        let (mut fa, mut fb) = (vec![0.0; m], vec![0.0; m]);
        a.feedforward_torque(pose, &acc, &mut fa);
        b.feedforward_torque(pose, &acc, &mut fb);
        assert_close(&format!("{what}: feedforward"), &fa, &fb, 1e-9);
        // The friction feedforward from the machine's model, at the speed above and slowly (within the guides' fade).
        for scale in [1.0, 1e-3] {
            let slow: Vec<f64> = speed.iter().map(|v| v * scale).collect();
            a.steady_friction(pose, &slow, &mut fa);
            b.steady_friction(pose, &slow, &mut fb);
            assert_close(&format!("{what}: steady friction at {scale} of the speed"), &fa, &fb, 1e-9);
            assert!(
                fa.iter().any(|f| *f != 0.0) || changes.contains(&"effects.guide_friction = false"),
                "{what}: {fa:?}"
            );
        }
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
        "beam_vertical_mode",
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
        // The belt's upper runs beside the leaning Z axis act with it (effects.belt_run_offset).
        same_machine(
            "t-gantry",
            "t-gantry.toml",
            &[&stretch, "effects.tool_flex = true", "effects.belt_run_offset = true", "effects.part_heating = true"],
            &rises,
            &format!("belt_run_offset, rigid {rigid}"),
        );
        let all: Vec<String> =
            effects.iter().chain(&["belt_run_offset"]).map(|e| format!("effects.{e} = true")).collect();
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
    // The limit is a number too: the S1 check's is the S1 curve at the cycle's mean speed, which rounds apart.
    for v in verdicts(cfg, &load).unwrap() {
        names.push(format!("verdict {} {:?} (ok {})", v.check, v.motor, v.ok));
        values.push(v.value);
        names.push(format!("limit {} {:?}", v.check, v.motor));
        values.push(v.limit);
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
/// in the guides and the braking chopper switching a step earlier grow that. So the crate is run again with its
/// carriage 1e-12 and 1e-9 heavier and, since the friction feedforward (15 November), with its X guide's Coulomb level
/// 1e-12 and 1e-9 higher, which reach the drives' torque directly, and with its belt's pretension 1e-12 and 1e-9 higher;
/// the file has to give the same verdicts and lie within three times the crate's own spread of it, or within 1e-4 of it.
/// Found on 2026-11-13: the T-gantry's peak braking power spreads by 0.5 % that way, and the file lies within that. On
/// 17 November the braking resistor's peak took three levels, 2.6191, 2.6196 and 2.6204 (the chopper switching a step
/// earlier or later), under changes of 1e-12 to 1e-8 of the carriage, the Z axis, the guides' Coulomb levels or the
/// pretension: the mass and the guide alone saw 7.8e-5 of it, the pretension 5.1e-4 at 1e-12; the file 4.8e-4.
#[test]
fn the_netlist_machines_give_their_crates_verdicts() {
    for (id, name) in PAIRS {
        let [(ea, ca), (eb, cb)] = pair(id, name, &[]);
        let (names, nominal) = cycle_numbers(ea, &ca);
        let mut spread = vec![0.0f64; nominal.len()];
        for (key, factor) in ["mass.carriage_kg", "friction_x.coulomb_n", "belt.pretension_n"]
            .iter()
            .flat_map(|k| [(*k, 1e-12), (*k, 1e-9)])
        {
            let mut c = ca.clone();
            c.set_text(&format!("{key} = {:e}", ca.f64(key).unwrap() * (1.0 + factor))).unwrap();
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
        t.replacen("belt_wrap = false", "belt_wrap = true", 1).replacen("at_m = [1.15, -0.03, 0.06]\n", "", 1)
    });
    assert_eq!(e, ConfigError::Invalid("effects.belt_wrap (pulley_b has no at_m)".into()));
    let (e, line) = refused(one("lever_axis = \"z\"\nlateral", "lever_axis = \"y\"\nlateral"));
    assert_eq!(e, ConfigError::Invalid("machine.lean[z_lean].lever_axis = \"y\"".into()));
    assert_eq!(line, Some(line_with(&text, "lateral_stiffness_n_m = \"tool_flex") - 1));
    let (e, line) = refused(one("[machine]\nversion = 1", "[machine]\nversion = 2"));
    assert_eq!(e, ConfigError::Version("machine.version".into()));
    assert_eq!(line, Some(text.lines().position(|l| l == "[machine]").unwrap() + 2));
}

/// Every shipped machine file, the H-bot of docs/machine-files.md among them and the stepper's and the induction motor's
/// axes, passes the standard suite of every machine (round trip, static hold, determinism, energy conservation without
/// losses, the loss budget with them), on its file without losses, elastic and rigid.
#[test]
fn every_machine_file_passes_the_standard_suite() {
    use std::sync::Mutex;
    static LOSSLESS: Mutex<Vec<Config>> = Mutex::new(Vec::new());
    fn lossless() -> Vec<Config> {
        LOSSLESS.lock().unwrap().clone()
    }
    // The pneumatic axis's cylinder rests in its end stop through the suite (it has no motor; the suite's pulse is on the
    // servo X): its own energy and step checks are in tests/pneumatic.rs and sim_components::pneumatic.
    let files = [
        "single-axis.toml",
        "t-gantry.toml",
        "h-bot.toml",
        "corexy.toml",
        "xyz-portal.toml",
        "stepper-axis.toml",
        "induction-lift.toml",
        "pneumatic-axis.toml",
    ];
    for name in files {
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

/// The belts' mass run by run (`machine.span` with velocity_x, _y, _z) in the files whose belt runs move over ground,
/// each derived from its belt path in the file's comment. Per file, without losses: rigid and elastic, the motors' power
/// is the rate of the energy along the machine's own derivatives to 2e-8 of it (Richardson's central difference over
/// +-0.1 and 0.2 us, every state moved a little off rest); rigid, the feedforward at speed gives the commanded
/// acceleration back; and the belts' kinetic energy is the sum over their runs by hand, 1/2 mu sum L_s |V_s v|^2: with
/// the belt's mass per metre 0.1 kg/m more the energy grows by 0.05 times the sum below (2T / mu, at the pose and speed
/// given). The H-bot's runs are the T-gantry's with Y for Z: L (vx^2 + vy^2) + 8 (0.65 - y) vx^2 + 4 (2x - 1) vx vy with
/// L = 4.2 m, so at x = 0.3, y = 0.1 the belt weighs mu (4.2 + 8 * 0.55) = 0.86 kg on X where the lumps had 0.42,
/// 0.42 kg on Y, and -0.08 kg between them. The CoreXY's belts move as a whole along X and along Y (0.39 kg, as the
/// lumps had it) and add -0.01 kg between them (belt A's 1.2 m off the clamps against B's 1.3 m). The portals' belts
/// ride on the axes that carry them: X carries every belt (4.8 m on the XYZ portal, 3.8 m on the C portal, the Y belt's
/// 2.0 and 1.4 m on the dual and rack gantries), Y the belts on it.
#[test]
fn the_belts_runs_keep_the_energy_and_give_the_moved_mass_by_hand() {
    type Sum = fn(&[f64], &[f64]) -> f64;
    let cases: [(&str, &[f64], &[f64], Sum); 6] = [
        ("h-bot.toml", &[0.3, 0.1], &[0.6, 0.2], |q, v| {
            4.2 * (v[0] * v[0] + v[1] * v[1])
                + 8.0 * (0.65 - q[1]) * v[0] * v[0]
                + 4.0 * (2.0 * q[0] - 1.0) * v[0] * v[1]
        }),
        ("corexy.toml", &[0.3, 0.1], &[0.6, 0.2], |_, v| 3.9 * (v[0] * v[0] + v[1] * v[1]) - 0.2 * v[0] * v[1]),
        ("xyz-portal.toml", &[0.3, 0.2, 0.1], &[0.6, 0.4, 0.2], |_, v| {
            4.8 * v[0] * v[0] + 2.4 * v[1] * v[1] + 1.0 * v[2] * v[2]
        }),
        ("portal-c.toml", &[0.3, 0.2, 0.1, 0.0], &[0.6, 0.4, 0.2, 0.0], |_, v| 3.8 * v[0] * v[0] + 1.4 * v[1] * v[1]),
        ("dual-gantry.toml", &[1.0, 0.3], &[0.6, 0.4], |_, v| 2.0 * (v[0] * v[0] + v[1] * v[1])),
        ("rack-gantry.toml", &[1.0, 0.2], &[0.6, 0.4], |_, v| 1.4 * (v[0] * v[0] + v[1] * v[1])),
    ];
    for (name, pose, vel, by_hand) in cases {
        let entry = file(name);
        let lossless = |edits: &[&str]| {
            let mut cfg = entry.config();
            for effect in [
                "belt_damping",
                "guide_friction",
                "load_dependent_friction",
                "motor_friction",
                "gearbox_efficiency",
                "gearbox_backlash",
                "torque_ripple",
                "tooth_meshing",
                "holding_brake",
            ] {
                cfg.set_text(&format!("effects.{effect} = false")).unwrap();
            }
            // The motors' own gearboxes ([gearbox.<motor>]) damp too.
            let own = ["mx", "mx1", "mx2", "mz", "mc"].map(|m| format!("gearbox.{m}.damping_nm_s_rad"));
            // And the screws' and racks' own dampers across their springs (`Converter::apply_back`).
            let shared = [
                "gearbox.damping_nm_s_rad",
                "gantry.damping_nm_s_rad",
                "screw_z.axial_damping_ratio",
                "rack_x.damping_ratio",
            ]
            .map(String::from);
            for damping in shared.iter().chain(&own) {
                if cfg.get(damping).is_ok() {
                    cfg.set_text(&format!("{damping} = 0.0")).unwrap();
                }
            }
            for edit in edits {
                cfg.set_text(edit).unwrap();
            }
            (entry.build)(&cfg).unwrap()
        };
        let moving = |m: &dyn Machine| {
            let (mut s, mut hold) = (Vec::new(), vec![0.0; m.motors().len()]);
            m.equilibrium(pose, &mut s, &mut hold);
            m.set_tool_velocity(&mut s, vel);
            (s, hold)
        };
        let off = |elastic: bool, mu: f64| {
            let m = lossless(&[&format!("effects.belt_stretch = {elastic}"), &format!("belt.mass_kg_m = {mu}")]);
            let (mut s, hold) = moving(m.as_ref());
            for (i, x) in s.iter_mut().enumerate() {
                *x += 1e-5 * (1.0 + (i % 5) as f64);
            }
            let torque: Vec<f64> = hold.iter().enumerate().map(|(k, h)| h + 0.7 - 0.4 * k as f64).collect();
            let mut rate = vec![0.0; s.len()];
            m.derivatives(&s, &torque, &mut rate);
            let energy = |h: f64| m.energy(&s.iter().zip(&rate).map(|(x, r)| x + h * r).collect::<Vec<_>>());
            let h = 1e-7;
            let rate_of_energy = (8.0 * (energy(h) - energy(-h)) - (energy(2.0 * h) - energy(-2.0 * h))) / (12.0 * h);
            let (mut angle, mut speed) = (vec![0.0; torque.len()], vec![0.0; torque.len()]);
            m.motor_state(&s, &mut angle, &mut speed);
            let power: f64 = torque.iter().zip(&speed).map(|(t, w)| t * w).sum();
            (rate_of_energy - power) / power
        };
        for elastic in [false, true] {
            let with = off(elastic, 0.1);
            println!("{name}, elastic {elastic}: off {with:.2e}");
            assert!(with.abs() < 2e-8, "{name}, elastic {elastic}: off {with}");
        }
        let acc: Vec<f64> = (0..pose.len()).map(|i| 3.0 - 2.0 * i as f64).collect();
        let reached = |mu: f64| {
            let rigid = lossless(&["effects.belt_stretch = false", &format!("belt.mass_kg_m = {mu}")]);
            let mut torque = vec![0.0; rigid.motors().len()];
            rigid.feedforward_torque_moving(pose, vel, &acc, &mut torque);
            let (s, _) = moving(rigid.as_ref());
            let mut rate = vec![0.0; s.len()];
            rigid.derivatives(&s, &torque, &mut rate);
            let tool_speed = |h: f64| {
                let (mut p, mut v) = (vec![0.0; pose.len()], vec![0.0; pose.len()]);
                rigid.tool_state(&s.iter().zip(&rate).map(|(x, r)| x + h * r).collect::<Vec<_>>(), &mut p, &mut v);
                v
            };
            let (ahead, behind) = (tool_speed(1e-6), tool_speed(-1e-6));
            ahead.iter().zip(&behind).map(|(a, b)| (a - b) / 2e-6).collect::<Vec<f64>>()
        };
        // The C portal and the dual gantry miss by up to 6e-6 with the belt's mass at 0 too (as above): there the
        // belt's mass must change nothing.
        let (with, without) = (reached(0.1), reached(0.0));
        if without.iter().zip(&acc).all(|(a, b)| close(*a, *b, 1e-8)) {
            assert_close(&format!("{name}: acceleration"), &with, &acc, 1e-8);
        } else {
            println!("{name}: acceleration {with:?}, without the belt's mass {without:?}");
            assert_close(&format!("{name}: acceleration with and without the belt"), &with, &without, 1e-9);
            assert_close(&format!("{name}: acceleration"), &with, &acc, 1e-5);
        }
        // Less the energy at rest at the pose: the runs that move with a vertical axis weigh there too.
        let kinetic = |mu: f64| {
            let m = lossless(&["effects.belt_stretch = false", &format!("belt.mass_kg_m = {mu}")]);
            let (mut rest, mut hold) = (Vec::new(), vec![0.0; m.motors().len()]);
            m.equilibrium(pose, &mut rest, &mut hold);
            m.energy(&moving(m.as_ref()).0) - m.energy(&rest)
        };
        let (belt, hand) = (kinetic(0.2) - kinetic(0.1), 0.5 * 0.1 * by_hand(pose, vel));
        assert!(close(belt, hand, 1e-9), "{name}: the belt carries {belt} J more, by hand {hand} J");
    }
}

/// The portal by hand. Each axis has a belt and a motor of its own, so the motor angle is i q / r: X and Y with the
/// shared gearbox (i 5), Z with its own ([gearbox.mz], i 10): at q = 0.1 m 25, 25 and 50 rad, and at 0.2 m/s 50, 50 and
/// 100 rad/s. Its Z motor holds the slide (2.5 kg) and the payload (1 kg) against gravity: m g r / i = 3.5 * 9.80665 *
/// 0.02 / 10 = 0.0686466 N m (rigid, without tooth meshing and ripple), and nothing for its belt: its loop's runs, 0.1 + z
/// and 0.4 - z up with the slide and 0.5 m down from the motor to the far idler, cancel (`runs_height` weighs them where
/// the frame momentum has them; with the sections shared out, 0.1 m down, 0.4 m of the belt's 0.1 kg/m rose with the
/// slide and it held 3.54 kg, 0.0694311 N m). X and Y lie flat and hold nothing. The Z motor's own
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

/// A per-motor table for an id the built machine has no motor of is refused by its name (`motor.zz`); one for a motor
/// it has passes. No shipped config (every crate's `*.toml` and `config/*.toml`) and no machine file is refused by
/// that check: each is either taken by some machine, or refused by every one for something else.
#[test]
fn a_per_motor_table_of_a_motor_the_machine_lacks_is_refused() {
    let entry = &machine_t_gantry::ENTRY;
    let cfg = entry.config();
    let ids: Vec<String> = (entry.build)(&cfg).unwrap().motors().iter().map(|m| m.id.to_string()).collect();
    let with = |id: &str| format!("{}\n[motor.{id}]\nrotor_inertia_kgm2 = 1e-4\n", entry.default_config);
    assert_eq!(sim_components::load_config(entry, &with("zz")).err(), Some(ConfigError::Invalid("motor.zz".into())));
    assert!(sim_components::load_config(entry, &with(&ids[0])).is_ok(), "{}", ids[0]);

    let per_motor = |e: &ConfigError| {
        matches!(e, ConfigError::Invalid(key) if key.split_once('.').is_some_and(|(table, id)|
            sim_components::catalog::PER_MOTOR_TABLES.contains(&table) && !id.contains('.')))
    };
    let mut files: Vec<PathBuf> = Vec::new();
    for dir in std::fs::read_dir(root().join("crates")).unwrap().flatten().map(|d| d.path()) {
        for sub in [dir.clone(), dir.join("config")] {
            let Ok(list) = std::fs::read_dir(&sub) else { continue };
            files.extend(
                list.flatten()
                    .map(|f| f.path())
                    .filter(|p| p.extension().is_some_and(|e| e == "toml") && !p.ends_with("Cargo.toml")),
            );
        }
    }
    assert!(files.len() > 40, "{} configs", files.len());
    for path in &files {
        let text = std::fs::read_to_string(path).unwrap().replace("\r\n", "\n");
        let results: Vec<_> = MACHINES.iter().map(|m| sim_components::load_config(m, &text)).collect();
        if results.iter().all(|r| r.is_err()) {
            let motor = results.iter().filter_map(|r| r.as_ref().err()).find(|e| per_motor(e));
            assert!(motor.is_none(), "{}: {motor:?}", path.display());
        }
    }
    for path in std::fs::read_dir(root().join("machines")).unwrap().flatten().map(|f| f.path()) {
        let Some(name) = path.file_name().and_then(|n| n.to_str()).filter(|n| n.ends_with(".toml")) else { continue };
        // A file that does not load is refused by the netlist's own check, not by this one.
        let Ok(entry) = load_file(&path) else { continue };
        assert!(sim_components::load_config(entry, entry.default_config).is_ok(), "{name}");
    }
}

/// A load-side scale is mounted on the frame: on the T-gantry's file with the frame swaying, through a cycle, it reads
/// the tool less the frame's deflection along X (`frame.deflection`) and the tool's height as it is.
/// The same on the gantry crate (the Z axis's lean off), along X; its Z scale rides on the carriage, so the beam's sag
/// under it is not in what it reads.
#[test]
fn a_load_scale_reads_the_carriage_against_the_swaying_frame() {
    for (entry, file) in [(file("t-gantry.toml"), true), (&machine_t_gantry::ENTRY, false)] {
        let mut cfg = entry.config();
        cfg.set_text("effects.frame_compliance = true").unwrap();
        cfg.set_text("effects.tool_flex = false").unwrap();
        let mut sim = Sim::new(entry, &cfg).unwrap();
        let sway = sim.signal_index("frame.deflection").unwrap();
        sim.command(sim_components::Command::StartCycle).unwrap();
        let (mut tool, mut rate, mut scale, mut scale_rate, mut out) =
            (vec![0.0; 2], vec![0.0; 2], vec![0.0; 2], vec![0.0; 2], Vec::new());
        let mut largest = 0.0f64;
        while sim.time() < 1.5 {
            sim.step();
            sim.machine.tool_state(&sim.state, &mut tool, &mut rate);
            sim.machine.scale_state(&sim.state, &mut scale, &mut scale_rate);
            sim.sample(&mut out);
            let off = tool[0] - scale[0] - out[sway];
            assert!(off.abs() < 1e-12, "{} at {} s: {tool:?} {scale:?} {}", entry.id, sim.time(), out[sway]);
            assert!(!file || tool[1] == scale[1], "{tool:?} {scale:?}");
            largest = largest.max(out[sway].abs());
        }
        assert!(largest > 1e-6, "{}: the frame sways ({largest} m)", entry.id);
    }
}

/// The belt's lowest mode by hand, on the single axis at x = 0.5 m: the carriage (4 kg and the 2 kg payload) between
/// the two sections, 0.15 + x = 0.65 m and 2.45 - x = 1.95 m long, with the pulley held. Each section's stiffness is
/// dT/dL = EA / l with l its unstretched belt (`Belt::section`): the axis lies flat, so both hold the pretension and
/// l = L / (1 + 300 / 5e5). k = EA (1 + 6e-4) (1 / 0.65 + 1 / 1.95) = 1.026257e6 N/m, f = sqrt(k / m) / (2 pi) =
/// 65.8222 Hz, as the crate's (`modes`). The mode moves the tool by 1 m, the pulley not. The rigid model has no mode.
#[test]
fn a_machine_file_reports_its_belt_mode() {
    let entry = file("single-axis.toml");
    let machine = (entry.build)(&entry.config()).unwrap();
    let k = 5e5 * (1.0 + 300.0 / 5e5) * (1.0 / 0.65 + 1.0 / 1.95);
    let hand = (k / 6.0f64).sqrt() / std::f64::consts::TAU;
    let hz = machine.resonance_hz(&[0.5]).unwrap();
    assert!(close(hz, hand, 1e-9) && (hz - 65.8222).abs() < 1e-4, "{hz} against {hand} by hand");
    let krate = find("single-axis").unwrap();
    assert!(close((krate.build)(&krate.config()).unwrap().resonance_hz(&[0.5]).unwrap(), hz, 1e-12));
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

/// The T-gantry's file tells the modes its crate tells: the belt's two modes with the pulleys held, their frequencies and
/// their damping ratios (the belt sections' dampers on each mode's shape), and with the frame's sway, the leaning Z axis
/// or the beam's vertical mode switched on in both, that mode too with its own damper, at the middle and an end of X and
/// two heights. Before the file machine told its modes it gave the lowest
/// alone, undamped.
#[test]
fn the_t_gantry_file_tells_its_crates_modes() {
    let variants: [&[&str]; 6] = [
        &[],
        &["effects.frame_compliance = true"],
        &["effects.tool_flex = true"],
        &["effects.beam_vertical_mode = true"],
        &["effects.tool_flex = true", "effects.belt_run_offset = true"],
        &["effects.tool_flex = true", "effects.belt_run_offset = true", "effects.beam_vertical_mode = true"],
    ];
    for changes in variants {
        let [(ea, ca), (eb, cb)] = pair("t-gantry", "t-gantry.toml", changes);
        let (a, b) = ((ea.build)(&ca).unwrap(), (eb.build)(&cb).unwrap());
        for pose in [[0.5, 0.25], [0.0, 0.25], [0.5, 0.45], [0.9, 0.05]] {
            let (ma, mb) = (a.modes(&pose), b.modes(&pose));
            assert!(ma.len() >= 2 && mb.len() == ma.len(), "{changes:?} {pose:?}: crate {ma:?}, file {mb:?}");
            for (x, y) in ma.iter().zip(&mb) {
                assert!(
                    close(x.hz, y.hz, 1e-9) && close(x.damping, y.damping, 1e-9),
                    "{changes:?} {pose:?}: crate {ma:?}, file {mb:?}"
                );
            }
        }
    }
}

/// The T-gantry's beam mode moving alone (effects.beam_vertical_mode, the state at rest at a pose with the beam's
/// deflection rate w' = 1 m/s and every other rate 0), crate against file. The belt's runs ride on the beam: the vertical
/// ones whole, the four horizontal ones tilting, a point at the share xi of the way from its post's pulley to the carriage
/// going down by xi w. A tilting run's angular momentum about y is then mu w' times the integral of x xi along it, its
/// weight times w' / 2 at the weighted point f + 2/3 (c - f), not at its middle (f its end at the pulley, c at the
/// carriage): the file took the middle, a sixth of the run off (up to 0.025 N m s per m/s of w' with the carriage at an
/// end). Both meet the carriage at its idlers, d = 0.042 m to the left (the two runs from motor A) and right (motor B);
/// until 22 November the crate met it at the carriage's x, (2 / 3) mu d (sx - 2 x) w' less about y (2.8e-3 N m s at
/// x = 0). They agree to rounding; about x and z both have none (the file's wheels in the bodies' plane).
#[test]
fn the_t_gantry_files_tilting_runs_turn_with_the_beam_as_the_crates() {
    let changes = ["effects.beam_vertical_mode = true"];
    let [(ea, ca), (eb, cb)] = pair("t-gantry", "t-gantry.toml", &changes);
    let (krate, file) = ((ea.build)(&ca).unwrap(), (eb.build)(&cb).unwrap());
    // The file's positions: x, z, the two pulleys, the two rotors, the beam's w; then their rates.
    let w_rate = 7 + 6;
    for x in [0.0, 0.5, 1.0] {
        let (mut s, mut torque) = (Vec::new(), vec![0.0; 2]);
        file.equilibrium(&[x, 0.15], &mut s, &mut torque);
        s[w_rate] = 1.0;
        let (fa, fb) =
            (krate.frame_momentum(&crate_state("t-gantry", &cb, &s)).unwrap(), file.frame_momentum(&s).unwrap());
        println!("x = {x}: crate {:?}, file {:?}", fa.angular, fb.angular);
        assert_close("linear momentum", &fa.linear, &fb.linear, 1e-12);
        assert_close("angular momentum", &fa.angular, &fb.angular, 1e-12);
    }
}

/// The T-gantry's angular momentum and first moment, crate against file, over 1 s of the default cycle (the file's run,
/// each state the crate's too; frame and leaning Z off in both, as `pair` has them). Both take moments about the same
/// point: the place of the Z axis's centre of mass at z = 0 under the carriage at x = 0. The bodies are the same there:
/// the Z axis at (x, 0, z), the carriage (and its chain) at (x, 0, 0.3), z_com_drop above it, which the file gives as its
/// carriage's `offset_m`, naming geometry.z_com_drop_m (checked with it moved to 0.45 m too; before it the file had the carriage at 0: its angular momentum about y was 0.3 m * 4 kg * vx
/// less, its first moment 1.2 kg m less). With the belt's mass off the two agree to rounding, the linear momentum and the
/// first moment with it on too (the runs' first moment, mu sum (L0 + c q / 2) V q, has no place in it). The belt's
/// angular momentum too, since 22 November: both take each run's mass at the middle of its straight piece between the
/// wheels where the 3D view draws them (the carriage idlers 0.042 m beside and 0.032 m above and below the carriage's
/// point, the bottom idler 0.08 m under the Z axis's point, the clamp at it, the motor pulleys at the posts; the crate's
/// `runs`), and neither has the belt lumped on the pulleys spinning. Before, the crate took its runs on another path (the
/// horizontal ones at the carriage's centre of mass, the
/// vertical ones at its x up to a clamp s + z above it) and its lumps spinning, and the two stood 1.29e-3 N m s apart about
/// y over the run; now 1.1e-16.
#[test]
fn the_t_gantry_file_has_its_crates_angular_momentum() {
    // The carriage's offset names geometry.z_com_drop_m, which the crate reads too: moved, the two still agree.
    let moved = ["effects.belt_mass = false", "geometry.z_com_drop_m = 0.45"];
    for (belt_mass, changes) in [(false, &moved[..1]), (false, &moved[..]), (true, &[][..])] {
        let [(ea, ca), (eb, cb)] = pair("t-gantry", "t-gantry.toml", changes);
        let krate = (ea.build)(&ca).unwrap();
        let mut sim = Sim::new(eb, &cb).unwrap();
        sim.command(sim_components::Command::StartCycle).unwrap();
        let (mut worst, mut speed) = ([0.0f64; 3], 0.0f64);
        for step in 0..16_000 {
            sim.step();
            if step % 50 != 0 {
                continue;
            }
            let sb = sim.state.clone();
            let fa = krate.frame_momentum(&crate_state("t-gantry", &cb, &sb)).unwrap();
            let fb = sim.machine.frame_momentum(&sb).unwrap();
            assert_close("linear momentum", &fa.linear, &fb.linear, 1e-9);
            assert_close("first moment", &fa.first_moment, &fb.first_moment, 1e-9);
            assert!(close(fa.mass, fb.mass, 1e-12), "{} against {}", fa.mass, fb.mass);
            speed = speed.max(fa.linear[0].hypot(fa.linear[2]) / fa.mass);
            for k in 0..3 {
                worst[k] = worst[k].max((fa.angular[k] - fb.angular[k]).abs());
            }
        }
        eprintln!("belt mass {belt_mass}: angular momentum apart by {worst:?} N m s, speed up to {speed} m/s");
        assert!(worst.iter().all(|w| *w < 1e-12), "{worst:?}");
    }
}

/// The XYZ portal's X and Y belts are loops (machines/xyz-portal.toml): the motor and the far idler 0.1 + stroke + 0.1 m
/// apart (X 1.2 m, Y 0.7 m), the clamp to the motor 0.1 + q, back round the far idler 1.2 + 1.1 - x and 0.7 + 0.6 - y.
/// With the pulleys held each axis rings on its two sections, k = EA (1 + 300 / 5e5) (1 / l1 + 1 / l2) (flat, both at
/// the pretension), carrying what rides on it and its belts' runs beyond the third of the 4.8 m of belt lumped on each
/// pulley: X 13.5 kg + 0.1 (4.8 - 1.6) = 13.82 kg, Y 7.5 kg + 0.1 (2.4 - 1.6) = 7.58 kg. X at its far end (1.1 and 1.3 m):
/// 39.2301 Hz, Y in the middle of its stroke with X at its start (0.6 and 0.8 m): 69.8303 Hz, each the lowest mode there.
/// With the sections shared out as before (the run from the motor to the far idler 0.1 m) X's far end rang at 73.91 Hz
/// on a return section of 0.2 m.
#[test]
fn the_portals_x_and_y_belts_ring_on_their_loops_by_hand() {
    let entry = file("xyz-portal.toml");
    let machine = (entry.build)(&entry.config()).unwrap();
    let hz = |l1: f64, l2: f64, m: f64| {
        (5e5 * (1.0 + 300.0 / 5e5) * (1.0 / l1 + 1.0 / l2) / m).sqrt() / std::f64::consts::TAU
    };
    let x = hz(1.1, 1.3, 13.5 + 0.1 * (4.8 - 1.6));
    let at_x = machine.resonance_hz(&[1.0, 0.25, 0.15]).unwrap();
    assert!(close(at_x, x, 1e-9) && (x - 39.2301).abs() < 1e-4, "X: {at_x} against {x} by hand");
    let y = hz(0.6, 0.8, 7.5 + 0.1 * (2.4 - 1.6));
    let at_y = machine.resonance_hz(&[0.1, 0.5, 0.15]).unwrap();
    assert!(close(at_y, y, 1e-9) && (y - 69.8303).abs() < 1e-4, "Y: {at_y} against {y} by hand");
}

/// The screw portal's family (machines/screw-portal.toml, portal-c.toml, five-axis.toml, five-axis-table.toml) has the
/// XYZ portal's X and Y loops, now with their far idlers in the routes (far_x on the frame at x = 1.1 m, far_y on the
/// bridge 0.7 m from the Y pulley): sections x2 2.3 m and y2 1.3 m, the clamp to the motor 0.1 + q. With the pulleys held
/// each axis rings on its two sections, k = EA (1 + 300 / 5e5) (1 / l1 + 1 / l2), carrying what rides on it: X at its
/// far end (1.1 and 1.3 m) the bridge, the Y carriage, the Z slide and the payload, 13.5 kg, with the C portal's gripper
/// 1.5 kg and its belts' runs beyond the half of the 3.8 m of belt lumped on each pulley, 0.1 (3.8 - 1.9) kg (the other
/// files have no spans), the five-axis head's 3 + 4 kg (the five-axis table's file has the screw portal's belts, its
/// tool's coordinates are the workpiece's on the table, so it is not compared at a pose here). Y in the
/// middle of its stroke with X at its start (0.35 and 1.05 m) on the screw portal: the Y carriage, Z slide and payload,
/// 7.5 kg. Each the lowest mode there. Before the loops (the run from the motor to the far idler 0.1 m) the screw
/// portal's X far end rang at 74.48 Hz on a return section of 0.2 m, its Y in the middle at 92.64 Hz.
#[test]
fn the_screw_portals_belts_ring_on_their_loops_by_hand() {
    let hz = |l1: f64, l2: f64, m: f64| {
        (5e5 * (1.0 + 300.0 / 5e5) * (1.0 / l1 + 1.0 / l2) / m).sqrt() / std::f64::consts::TAU
    };
    for (name, x_mass, y_mass, pose_x, pose_y) in [
        ("screw-portal.toml", 13.5, Some(7.5), vec![1.0, 0.25, 0.1], vec![0.0, 0.25, 0.1]),
        // The C portal's tip is 0.1 m along x off its C axis at C = 0; its lowest mode with X at its start is the
        // gripper turning C with Y (58.9 Hz), and the five-axis head's couples Y with its rotaries (55.7 Hz against the
        // belt's 57.7 alone), so Y is not compared there.
        ("portal-c.toml", 15.0 + 0.19, None, vec![1.1, 0.25, 0.1, 0.0], vec![0.1, 0.25, 0.1, 0.0]),
        ("five-axis.toml", 20.5, None, vec![1.0, 0.25, 0.1, 0.0, 0.0], vec![0.0, 0.25, 0.1, 0.0, 0.0]),
    ] {
        let entry = file(name);
        let machine = (entry.build)(&entry.config()).unwrap();
        let x = hz(1.1, 1.3, x_mass);
        let at_x = machine.resonance_hz(&pose_x).unwrap();
        eprintln!("{name}: X {at_x} against {x} by hand, Y {:?}", machine.resonance_hz(&pose_y));
        assert!(close(at_x, x, 1e-9), "{name} X: {at_x} against {x} by hand");
        if let Some(m) = y_mass {
            let (y, at_y) = (hz(0.35, 1.05, m), machine.resonance_hz(&pose_y).unwrap());
            assert!(close(at_y, y, 1e-9), "{name} Y: {at_y} against {y} by hand");
        }
    }
}

/// The 3D view draws the belt round its wheels: a strip per straight run from tangent to tangent, and 8 round each
/// wheel inside a section's way and each motor pulley one section ends at and another starts from. The T-gantry's file
/// routes its sections past its idlers: 2 + 4 + 2 runs, 8 round each of its 1 + 3 + 1 idlers inside the ways and each
/// motor pulley, which one section ends at and the next starts from: 8 + 40 + 16 = 64 strips, and five idlers. The
/// H-bot's belt takes the T-gantry's path (its routes and idlers, for the belt's runs), so the same.
#[test]
fn the_3d_view_draws_the_belt_of_a_machine_file() {
    for (name, strips, idlers) in [("h-bot.toml", 64, 5), ("t-gantry.toml", 64, 5)] {
        let entry = file(name);
        let sim = Sim::new(entry, &entry.config()).unwrap();
        let mut scene = sim_core::Scene::default();
        sim.machine.scene(&sim.state, &mut scene);
        let count = |id: &str| scene.parts.iter().filter(|p| p.id == id).count();
        assert_eq!((count("belt"), count("idler")), (strips, idlers), "{name}");
    }
}
