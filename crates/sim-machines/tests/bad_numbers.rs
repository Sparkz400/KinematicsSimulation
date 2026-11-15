//! Every shipped config against bad values: each registered machine's default config, every other config shipped
//! beside a machine, and every machine file in machines/. Every number in it (in arrays too) is set, one at a time, to
//! -1, 0, NaN, infinity, 1e30 and 1e-30; and every table and key is taken out, every number made a string and every
//! value made a table. Each such config is refused with a `ConfigError`, or a short move runs with finite signals. It
//! never panics. A run is a fixed number of base steps, so a time step the config makes huge or tiny cannot stretch it.

use sim_components::{Command, Sim, load_config};
use sim_core::{Config, MachineEntry};
use sim_machines::{find, load_file};
use std::path::Path;
use toml::{Table, Value};

/// Base steps of a run: 20 ms at the shipped 62.5 µs, enough to start the cycle's first move.
const STEPS: u64 = 320;

/// The bad numbers, as a float and (for a whole-number key) as the nearest integer where there is one.
const BAD: [(f64, Option<i64>); 6] =
    [(-1.0, Some(-1)), (0.0, Some(0)), (f64::NAN, None), (f64::INFINITY, None), (1e30, Some(i64::MAX)), (1e-30, None)];

/// A place in a TOML document: a key of a table or an index of an array.
#[derive(Clone, Debug)]
enum Seg {
    Key(String),
    Index(usize),
}

fn show(path: &[Seg]) -> String {
    path.iter()
        .map(|s| match s {
            Seg::Key(k) => format!(".{k}"),
            Seg::Index(i) => format!("[{i}]"),
        })
        .collect::<String>()
        .trim_start_matches('.')
        .to_string()
}

/// Every value below `value` with its path, tables and arrays as well as scalars.
fn walk(value: &Value, path: &mut Vec<Seg>, out: &mut Vec<(Vec<Seg>, Value)>) {
    if !path.is_empty() {
        out.push((path.clone(), value.clone()));
    }
    let children: Vec<(Seg, &Value)> = match value {
        Value::Table(t) => t.iter().map(|(k, v)| (Seg::Key(k.clone()), v)).collect(),
        Value::Array(a) => a.iter().enumerate().map(|(i, v)| (Seg::Index(i), v)).collect(),
        _ => return,
    };
    for (seg, child) in children {
        path.push(seg);
        walk(child, path, out);
        path.pop();
    }
}

/// The value at `path`, if `doc` has one there.
fn at<'a>(doc: &'a Table, path: &[Seg]) -> Option<&'a Value> {
    let (Seg::Key(first), rest) = path.split_first()? else { return None };
    rest.iter().try_fold(doc.get(first)?, |v, seg| match seg {
        Seg::Key(k) => v.get(k.as_str()),
        Seg::Index(i) => v.get(*i),
    })
}

/// The document with the value at `path` replaced by `new`, or taken out with `None`.
fn edited(doc: &Table, path: &[Seg], new: Option<Value>) -> Table {
    let mut root = Value::Table(doc.clone());
    let mut at = &mut root;
    for seg in &path[..path.len() - 1] {
        at = match seg {
            Seg::Key(k) => at.get_mut(k.as_str()),
            Seg::Index(i) => at.get_mut(*i),
        }
        .expect("the path was walked in this document");
    }
    match (at, &path[path.len() - 1], new) {
        (Value::Table(t), Seg::Key(k), Some(v)) => drop(t.insert(k.clone(), v)),
        (Value::Table(t), Seg::Key(k), None) => drop(t.remove(k)),
        (Value::Array(a), Seg::Index(i), Some(v)) => a[*i] = v,
        (Value::Array(a), Seg::Index(i), None) => drop(a.remove(*i)),
        _ => unreachable!("a key indexes a table and an index an array"),
    }
    let Value::Table(t) = root else { unreachable!() };
    t
}

#[derive(Default)]
struct Tally {
    refused: usize,
    ran: usize,
    defects: Vec<String>,
}

/// One damaged config: refused, or `STEPS` steps of the cycle with finite signals; a panic is a defect. A damage to
/// `[bounds]` is checked by what reads it, the catalogue's scaling and `analysis::bounds`: the simulation does not.
fn try_config(
    entry: &'static MachineEntry,
    machine_file: bool,
    bounds_only: bool,
    doc: &Table,
    what: &str,
    tally: &mut Tally,
) {
    let text = toml::to_string(doc).expect("a TOML table is written as text");
    let outcome = std::panic::catch_unwind(|| {
        // A machine file is checked by its netlist when the simulation is built; a crate's config against its schema.
        let cfg = if machine_file { Config::parse(&text) } else { load_config(entry, &text) };
        let Ok(cfg) = cfg else { return None };
        if bounds_only {
            let resolved = sim_components::catalog::resolve(entry, &cfg).ok()?;
            return sim_components::analysis::bounds(&resolved).ok().map(|_| true);
        }
        let Ok(mut sim) = Sim::new(entry, &cfg) else { return None };
        if sim.command(Command::StartCycle).is_err() {
            return None;
        }
        let mut out = Vec::new();
        for _ in 0..STEPS {
            sim.step();
            if sim.step % 16 == 0 {
                sim.sample(&mut out);
                if !out.iter().all(|v| v.is_finite()) {
                    return Some(false);
                }
            }
        }
        Some(sim.state.iter().all(|v| v.is_finite()))
    });
    match outcome {
        Err(_) => tally.defects.push(format!("{what}: panics")),
        Ok(Some(false)) => tally.defects.push(format!("{what}: accepted, signals not finite")),
        Ok(Some(true)) => tally.ran += 1,
        Ok(None) => tally.refused += 1,
    }
}

/// Every bad value and every damage to the structure of one config; panics with the list of what was not refused.
/// With `beside`, a config shipped beside the machine: only the values it does not share with the machine's default
/// config, which its own test sweeps through the same readers (the seven-axis arm's 2000 cases took over 15 minutes).
fn check(name: &str, entry: &'static MachineEntry, machine_file: bool, text: &str, beside: bool) {
    let start = std::time::Instant::now();
    // Panics are caught and reported together below, not printed one by one.
    std::panic::set_hook(Box::new(|_| {}));
    let doc: Table = text.parse().unwrap_or_else(|e| panic!("{name}: {e}"));
    let mut all = Vec::new();
    walk(&Value::Table(doc.clone()), &mut Vec::new(), &mut all);
    let default: Table = entry.default_config.parse().expect("a registered machine's config parses");
    let mut tally = Tally::default();
    try_config(entry, machine_file, false, &doc, &format!("{name} as shipped"), &mut tally);
    assert_eq!((tally.ran, tally.refused), (1, 0), "{name}: the config as shipped does not run: {:?}", tally.defects);
    for (path, value) in &all {
        let p = show(path);
        if beside && at(&default, path) == Some(value) {
            continue;
        }
        let mut damage: Vec<(String, Option<Value>)> = vec![("taken out".into(), None)];
        match value {
            Value::Float(_) | Value::Integer(_) => {
                for (f, i) in BAD {
                    let v = match (value, i) {
                        (Value::Integer(_), Some(i)) => Value::Integer(i),
                        _ => Value::Float(f),
                    };
                    damage.push((format!("= {v}"), Some(v)));
                }
                damage.push(("= \"1\"".into(), Some(Value::String("1".into()))));
            }
            Value::Table(_) => {}
            _ => damage.push(("= {}".into(), Some(Value::Table(Table::new())))),
        }
        for (how, new) in damage {
            let bounds_only = matches!(&path[0], Seg::Key(k) if k == "bounds");
            try_config(
                entry,
                machine_file,
                bounds_only,
                &edited(&doc, path, new),
                &format!("{name}: {p} {how}"),
                &mut tally,
            );
        }
    }
    let _ = std::panic::take_hook();
    eprintln!(
        "{name}: {} refused, {} ran, {} defects in {:.1} s",
        tally.refused,
        tally.ran,
        tally.defects.len(),
        start.elapsed().as_secs_f64()
    );
    assert!(tally.defects.is_empty(), "{} defects:\n{}", tally.defects.len(), tally.defects.join("\n"));
}

fn root() -> &'static Path {
    Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../.."))
}

fn read(rel: &str) -> String {
    std::fs::read_to_string(root().join(rel)).unwrap_or_else(|e| panic!("{rel}: {e}"))
}

/// A registered machine's default config.
macro_rules! default_config {
    ($($test:ident: $id:literal;)*) => {$(
        #[test]
        fn $test() {
            let entry = find($id).unwrap();
            check($id, entry, false, entry.default_config, false);
        }
    )*};
}

/// Another config shipped beside a machine, loaded as a front end loads it.
macro_rules! shipped_config {
    ($($test:ident: $id:literal, $file:literal;)*) => {$(
        #[test]
        fn $test() {
            check($file, find($id).unwrap(), false, &read($file), true);
        }
    )*};
}

/// A machine file, its own config.
macro_rules! machine_file {
    ($($test:ident: $file:literal;)*) => {$(
        #[test]
        fn $test() {
            let path = root().join("machines").join($file);
            let entry = load_file(&path).unwrap_or_else(|e| panic!("{e}"));
            check($file, entry, true, entry.default_config, false);
        }
    )*};
}

default_config! {
    default_t_gantry: "t-gantry";
    default_single_axis: "single-axis";
    default_delta_d3: "delta-d3";
    default_delta_d2: "delta-d2";
    default_parallel_scara: "parallel-scara";
    default_delta_linear: "delta-linear";
    default_delta_d4: "delta-d4";
    default_delta_h4: "delta-h4";
    default_screw_axis: "screw-axis";
    default_rack_axis: "rack-axis";
    default_linear_motor_axis: "linear-motor-axis";
    default_rotary_table: "rotary-table";
    default_urdf_arm: "urdf-arm";
    default_scara: "scara";
    default_slider_crank: "slider-crank";
    default_rotary_knife: "rotary-knife";
    default_flying_saw: "flying-saw";
    default_palletiser: "palletiser";
    default_hexapod: "hexapod";
    default_planar_cable_robot: "cable-planar";
    default_spatial_cable_robot: "cable-spatial";
    default_disc_cam: "disc-cam";
    default_tripod: "tripod";
    default_wafer_handler: "wafer-handler";
    default_toggle_press: "toggle-press";
}

shipped_config! {
    config_t_gantry_exct_30: "t-gantry", "crates/machine-t-gantry/exct-30.toml";
    config_t_gantry_conveyor: "t-gantry", "crates/machine-t-gantry/conveyor.toml";
    config_delta_d2_conveyor: "delta-d2", "crates/machine-delta-d2/conveyor.toml";
    config_delta_d3_conveyor: "delta-d3", "crates/machine-delta-d3/conveyor.toml";
    config_delta_d4_conveyor: "delta-d4", "crates/machine-delta-d4/conveyor.toml";
    config_delta_linear_conveyor: "delta-linear", "crates/machine-delta-linear/conveyor.toml";
    config_slider_crank_cam: "slider-crank", "crates/machine-crank-cam/config/slider-crank-cam.toml";
    config_rotary_knife_cam: "rotary-knife", "crates/machine-crank-cam/config/rotary-knife-cam.toml";
    config_flying_saw_cam: "flying-saw", "crates/machine-crank-cam/config/flying-saw-cam.toml";
    config_disc_cam_cam: "disc-cam", "crates/machine-crank-cam/config/disc-cam-cam.toml";
    config_arm_tool: "urdf-arm", "crates/machine-joint-kit/config/arm-tool.toml";
    config_arm5_tool: "urdf-arm", "crates/machine-joint-kit/config/arm5-tool.toml";
    config_arm7_tool: "urdf-arm", "crates/machine-joint-kit/config/arm7-tool.toml";
    config_scara_tool: "scara", "crates/machine-joint-kit/config/scara-tool.toml";
    config_toggle_press_cam: "toggle-press", "crates/machine-crank-cam/config/toggle-press-cam.toml";
}

machine_file! {
    file_corexy: "corexy.toml";
    file_delta_d3: "delta-d3.toml";
    file_dual_gantry: "dual-gantry.toml";
    file_five_axis_table: "five-axis-table.toml";
    file_five_axis: "five-axis.toml";
    file_h_bot: "h-bot.toml";
    file_induction_lift: "induction-lift.toml";
    file_planar_delta: "planar-delta.toml";
    file_portal_c: "portal-c.toml";
    file_rack_gantry: "rack-gantry.toml";
    file_scissor_lift: "scissor-lift.toml";
    file_screw_portal: "screw-portal.toml";
    file_single_axis: "single-axis.toml";
    file_stepper_axis: "stepper-axis.toml";
    file_t_gantry: "t-gantry.toml";
    file_xyz_portal: "xyz-portal.toml";
}
