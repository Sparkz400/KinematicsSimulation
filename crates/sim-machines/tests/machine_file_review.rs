//! Regression tests for the defects the review of machine files found on 2026-11-13, each with the machine file the
//! review found it with: routings the two shipped netlists of crates cannot show (two and three belt loops, a pulley
//! of another radius), files that loaded where they had to be refused, and error lines that pointed elsewhere.

use sim_core::{Config, MachineEntry};
use sim_machines::MACHINES;
use std::path::Path;

fn hbot() -> String {
    std::fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines/h-bot.toml"))
        .unwrap()
        .replace("\r\n", "\n")
}
fn base() -> String {
    let t = hbot();
    let at = t.find("[machine]\n").unwrap();
    t[..at].to_string()
}
fn known() -> Vec<Config> {
    MACHINES.iter().map(|m| m.config()).collect()
}
fn load(text: &str) -> Result<&'static MachineEntry, (sim_core::ConfigError, Option<usize>)> {
    sim_netlist::from_text(text, true, "review", &known())
}

const HEAD: &str = "[machine]\nversion = 1\n";
const DRIVES2: &str = r#"
[[machine.axis]]
id = "x"
stroke_m = "geometry.stroke_x_m"
[[machine.axis]]
id = "y"
stroke_m = "geometry.stroke_y_m"
INCLINE
[[machine.motor]]
id = "a"
[[machine.motor]]
id = "b"
[[machine.gearbox]]
id = "ga"
motor = "a"
[[machine.gearbox]]
id = "gb"
motor = "b"
[[machine.pulley]]
id = "pa"
gearbox = "ga"
radius_m = 0.02
inertia_kgm2 = 1e-5
[[machine.pulley]]
id = "pb"
gearbox = "gb"
radius_m = 0.02
inertia_kgm2 = 1e-5
"#;
const TWO_LOOPS: &str = r#"
[[machine.section]]
id = "s1"
length_m = 0.3
axes = { x = 1 }
pulleys = { pa = 1 }
[[machine.section]]
id = "s2"
length_m = 1.3
axes = { x = -1 }
pulleys = { pa = -1 }
[[machine.section]]
id = "s3"
length_m = 0.2
axes = { y = 1 }
pulleys = { pb = 1 }
[[machine.section]]
id = "s4"
length_m = 0.9
axes = { y = -1 }
pulleys = { pb = -1 }
"#;
const BODIES: &str = r#"
[[machine.body]]
id = "carriage"
mass_kg = 4.0
axes = ["x"]
[[machine.body]]
id = "head"
mass_kg = 2.0
axes = ["x", "y"]
"#;

fn file(incline: &str, rest: &str) -> String {
    format!("{}{HEAD}{}{rest}", base(), DRIVES2.replace("INCLINE", incline))
}

static LOSSLESS: std::sync::Mutex<Vec<Config>> = std::sync::Mutex::new(Vec::new());
fn lossless() -> Vec<Config> {
    LOSSLESS.lock().unwrap().clone()
}
fn suite(entry: &'static MachineEntry) {
    let mut configs = Vec::new();
    for elastic in [true, false] {
        let mut cfg = entry.config();
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
            "gearbox_backlash",
        ] {
            cfg.set_text(&format!("effects.{effect} = false")).unwrap();
        }
        for damping in ["gearbox.damping_nm_s_rad = 0.0", "hard_stop.damping_n_s_m = 0.0"] {
            cfg.set_text(damping).unwrap();
        }
        cfg.set_text(&format!("effects.belt_stretch = {elastic}")).unwrap();
        configs.push(cfg);
    }
    *LOSSLESS.lock().unwrap() = configs;
    let e = MachineEntry { suite_configs: lossless, ..*entry };
    sim_core::testkit::run_standard_suite(&e);
}

fn derivs_at_rest(entry: &'static MachineEntry, elastic: bool, pose: &[f64]) -> Vec<f64> {
    let mut cfg = entry.config();
    cfg.set_text(&format!("effects.belt_stretch = {elastic}")).unwrap();
    let m = (entry.build)(&cfg).unwrap();
    let (mut s, mut t) = (Vec::new(), vec![0.0; m.motors().len()]);
    m.equilibrium(pose, &mut s, &mut t);
    let mut d = vec![0.0; s.len()];
    m.derivatives(&s, &t, &mut d);
    d
}

#[test]
fn two_loops_vertical_equilibrium() {
    let text = file("incline_deg = 90", &format!("{TWO_LOOPS}{BODIES}"));
    let entry = load(&text).unwrap_or_else(|e| panic!("{e:?}"));
    // Each loop keeps its own belt: the vertical axis's loop holds its weight with its own tensions. Taking all
    // sections as one belt left y accelerating at 0.2 m/s^2 and pulley b at -124 rad/s^2 from rest.
    for elastic in [true, false] {
        let d = derivs_at_rest(entry, elastic, &[0.5, 0.25]);
        assert!(d.iter().all(|x| x.abs() < 1e-6), "elastic {elastic}: {d:?}");
    }
}
#[test]
fn two_loops_vertical_suite() {
    let text = file("incline_deg = 90", &format!("{TWO_LOOPS}{BODIES}"));
    suite(load(&text).unwrap_or_else(|e| panic!("{e:?}")));
}
#[test]
fn two_loops_flat_suite() {
    let text = file("", &format!("{TWO_LOOPS}{BODIES}"));
    suite(load(&text).unwrap_or_else(|e| panic!("{e:?}")));
}

#[test]
fn axis_without_body() {
    let bodies = "\n[[machine.body]]\nid = \"carriage\"\nmass_kg = 4.0\naxes = [\"x\"]\n";
    let text = file("", &format!("{TWO_LOOPS}{bodies}"));
    // It loaded, and the axis accelerated at infinity (its mass 0).
    assert_eq!(
        load(&text).err().map(|e| e.0),
        Some(sim_core::ConfigError::Invalid("machine.axis[y] (no body moves with it)".into()))
    );
}

#[test]
fn hbot_scene() {
    let entry = load(&hbot()).unwrap();
    let m = (entry.build)(&entry.config()).unwrap();
    let (px, py) = (m.scene_point(&[0.2, 0.0]), m.scene_point(&[0.0, 0.2]));
    let mut cfg = entry.config();
    cfg.set_text("effects.gravity = false").unwrap();
    let m2 = (entry.build)(&cfg).unwrap();
    assert_ne!(px, py, "Y motion drawn as X motion");
    // And the picture does not turn with the gravity switch.
    assert_eq!((m2.scene_point(&[0.2, 0.0]), m2.scene_point(&[0.0, 0.2])), (px, py));
}

#[test]
fn friction_under_a_known_scalar() {
    let guide = "\n[[machine.guide]]\nid = \"g\"\naxis = \"x\"\nfriction = \"frame.mass_kg.f\"\n";
    let mut text = file("", &format!("{TWO_LOOPS}{BODIES}{guide}"));
    let fr = base();
    let fx = fr.find("[friction_x]").unwrap();
    let end = fx + fr[fx..].find("\n[").unwrap();
    text.push('\n');
    text.push_str(&fr[fx..end].replace("[friction_x]", "[frame.mass_kg.f]"));
    // A known value lies on the way to the friction table: refused by name, where it panicked.
    let r = std::panic::catch_unwind(|| load(&text).map(|_| ()));
    assert!(matches!(r, Ok(Err((sim_core::ConfigError::Unknown(_), _)))), "{r:?}");
}

#[test]
fn machine_id_validation() {
    let t = hbot();
    use sim_core::ConfigError::{Invalid, Type};
    let err = |text: String| load(&text).err().map(|e| e.0);
    assert_eq!(
        err(t.replace("id = \"h-bot\"", "id = \"bad id/x\"")),
        Some(Invalid("machine.id (letters, digits, _ and - only)".into()))
    );
    assert_eq!(err(t.replace("id = \"h-bot\"", "id = 42")), Some(Type("machine.id".into())));
    assert_eq!(err(t.replace("name = \"H-bot (machine file)\"", "name = [1]")), Some(Type("machine.name".into())));
    // Fine as they were: a misspelt table is refused, and so is a fourth section that pulley A would have to feed
    // with x alone while section 1 takes x + y from it.
    assert!(
        load(&t.replace(
            "[machine]
",
            "[machin]
foo = 1
[machine]
"
        ))
        .is_err()
    );
    let fourth = "pulleys = { pulley_b = -1 }
[[machine.section]]
id = \"4\"
length_m = 1.0
axes = { x = 1 }
pulleys = { pulley_a = 1 }";
    assert_eq!(
        err(t.replace("pulleys = { pulley_b = -1 }", fourth)),
        Some(Invalid("machine.section[1].axes (the belt cannot keep its length)".into()))
    );
}

#[test]
fn line_of_with_machine_id_equal_to_node_id() {
    let t = hbot()
        .replace("id = \"h-bot\"", "id = \"x\"")
        .replace("stroke_m = \"geometry.stroke_x_m\"", "stroke_m = \"geometry.stroke_q_m\"");
    let Err((e, line)) = load(&t) else { panic!("loaded") };
    let want = t.lines().position(|l| l.contains("stroke_q_m")).unwrap() + 1;
    let _ = e;
    assert_eq!(line, Some(want));
}

#[test]
fn line_of_header_with_comment() {
    let t = hbot().replace("[belt]\n", "[belt] # the belt\n").replace("pretension_n =", "pretensoin_n =");
    let Err((e, line)) = load(&t) else { panic!("loaded") };
    let want = t.lines().position(|l| l.contains("pretensoin_n")).unwrap() + 1;
    let _ = e;
    assert_eq!(line, Some(want));
}

#[test]
fn json_float_version() {
    let table: toml::Table = hbot().parse().unwrap();
    let mut j = serde_json::to_value(&table).unwrap();
    j["machine"]["version"] = serde_json::json!(1.0);
    let path = std::env::temp_dir().join(format!("review-{}.json", std::process::id()));
    std::fs::write(&path, j.to_string()).unwrap();
    // JSON has one kind of number, and the JSON Schema takes 1.0 for 1: so does the loader now.
    assert_eq!(sim_machines::load_file(&path).map(|e| e.id).map_err(|e| e.to_string()), Ok("h-bot"));
    std::fs::remove_file(&path).ok();
}

#[test]
fn commanded_acc_vs_true_acc() {
    let t = hbot();
    let t = t.replacen(
        "radius_m = \"geometry.pulley_radius_m\"\ninertia_kgm2 = \"pulley.inertia_kgm2\"\npos = [300, 300]",
        "radius_m = 0.04\ninertia_kgm2 = \"pulley.inertia_kgm2\"\npos = [300, 300]",
        1,
    );
    let t = t.replace("carries = [\"carriage\", \"head\"]\n", "carries = [\"carriage\", \"head\"]\noverhang = \"head\"\noverhang_axis = \"x\"\nlever_m = 0.1\nspacing_m = 0.1\n");
    let entry = load(&t).unwrap_or_else(|e| panic!("{e:?}"));
    let mut cfg = entry.config();
    for e in [
        "belt_stretch",
        "guide_friction",
        "load_dependent_friction",
        "motor_friction",
        "gearbox_efficiency",
        "torque_ripple",
        "tooth_meshing",
        "gravity",
        "holding_brake",
        "gearbox_backlash",
    ] {
        cfg.set_text(&format!("effects.{e} = false")).unwrap();
    }
    let m = (entry.build)(&cfg).unwrap();
    let (mut s, mut hold) = (Vec::new(), vec![0.0; 2]);
    m.equilibrium(&[0.5, 0.25], &mut s, &mut hold);
    let torque = [1.0, 0.0];
    let mut d = vec![0.0; s.len()];
    m.derivatives(&s, &torque, &mut d);
    let acc_x = d[2];
    let infos = m.signal_infos();
    let mut v = Vec::new();
    m.signals(&s, &torque, &mut v);
    let gl = infos.iter().position(|i| i.id == "axis.x.guide_load").unwrap();
    let head = cfg.f64("mass.z_axis_kg").unwrap() + cfg.f64("mass.payload_kg").unwrap();
    let expected = 2.0 * head * acc_x.abs() * 0.1 / 0.1;
    // The tilt from the X acceleration a rigid machine gets: it was 17 % high where the drive mass matrix is not
    // diagonal (a pulley of another radius).
    assert!((v[gl] - expected).abs() < 1e-6 * expected, "guide load {} vs {expected}", v[gl]);
    // The time-optimal plan reads the same load (`Machine::rigid_guide_loads`): nothing at rest without gravity, and
    // 2 m a lever / spacing linear in the X acceleration.
    let mut loads = Vec::new();
    m.rigid_guide_loads(&[0.5, 0.25], &[acc_x, 0.0], &mut loads);
    let x = infos.iter().filter(|i| i.id.ends_with(".guide_load")).position(|i| i.id == "axis.x.guide_load").unwrap();
    assert!(loads[x][0].abs() < 1e-12 && (loads[x][1].abs() - expected).abs() < 1e-6 * expected, "{loads:?}");
}

#[test]
fn three_axes_suite() {
    let extra = r#"
[[machine.axis]]
id = "z"
stroke_m = 0.3
incline_deg = 90
[[machine.motor]]
id = "c"
[[machine.gearbox]]
id = "gc"
motor = "c"
[[machine.pulley]]
id = "pc"
gearbox = "gc"
radius_m = 0.02
inertia_kgm2 = 1e-5
[[machine.section]]
id = "s5"
length_m = 0.5
axes = { z = 1 }
pulleys = { pc = 1 }
[[machine.section]]
id = "s6"
length_m = 0.5
axes = { z = -1 }
pulleys = { pc = -1 }
[[machine.body]]
id = "carriage"
mass_kg = 4.0
axes = ["x"]
[[machine.body]]
id = "head"
mass_kg = 2.0
axes = ["x", "y", "z"]
"#;
    let mut text = file("", &format!("{TWO_LOOPS}{extra}"));
    text = text.replace(
        "[axis.y]",
        "[axis.z]\nmax_velocity_m_s = 0.4\nmax_acceleration_m_s2 = 6.0\nmax_jerk_m_s3 = 100.0\n\n[axis.y]",
    );
    let text = regex_like(&text, |a, b| format!("[{a}, {b}, 0.1]"));
    let entry = load(&text).unwrap_or_else(|e| panic!("{e:?}"));
    let d = derivs_at_rest(entry, true, &[0.5, 0.25, 0.1]);
    assert!(d.iter().all(|x| x.abs() < 1e-6), "{d:?}");
    suite(entry);
}

/// Rewrites every `pose = [a, b]` / `target = [a, b]`.
fn regex_like(text: &str, f: impl Fn(&str, &str) -> String) -> String {
    text.lines()
        .map(|l| {
            for key in ["pose = [", "target = ["] {
                if let Some(rest) = l.strip_prefix(key) {
                    let inner = rest.trim_end_matches(']');
                    let (a, b) = inner.split_once(',').unwrap();
                    return format!("{} = {}", &key[..key.len() - 4], f(a.trim(), b.trim()));
                }
            }
            l.to_string()
        })
        .collect::<Vec<_>>()
        .join("\n")
        + "\n"
}
