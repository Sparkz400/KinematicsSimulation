//! Machine files' physics beside the belt: tooth jump beside a screw, the CoreXY's idlers, a table's reach with a head's
//! tool offset, the frame momentum's spin, a gantry yaw's spring, a rotary's bearing at the output and the rigid
//! friction's load (crates/sim-netlist).

use sim_components::mech::JUMP_STATES;
use sim_core::{Config, Machine, Scene};
use sim_machines::load_file;
use std::path::{Path, PathBuf};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

/// A machine file's text with its line ends made plain.
fn text(name: &str) -> String {
    std::fs::read_to_string(root().join("machines").join(name)).unwrap().replace("\r\n", "\n")
}

/// A config with `changes` (TOML lines, `path = value`) set or added.
fn with(cfg: &Config, changes: &[&str]) -> Config {
    let mut all: toml::Table = cfg.to_text().unwrap().parse().unwrap();
    for change in changes {
        let (path, value) = change.split_once('=').unwrap();
        let value: toml::Table = format!("v = {value}").parse().unwrap();
        let parts: Vec<&str> = path.trim().split('.').collect();
        let mut table = &mut all;
        for part in &parts[..parts.len() - 1] {
            table =
                table.entry(*part).or_insert_with(|| toml::Value::Table(toml::Table::new())).as_table_mut().unwrap();
        }
        table.insert(parts[parts.len() - 1].into(), value["v"].clone());
    }
    Config::parse(&toml::to_string(&all).unwrap()).unwrap()
}

fn shipped(name: &str) -> Config {
    load_file(&root().join("machines").join(name)).unwrap_or_else(|e| panic!("{e}")).config()
}

/// The machine at rest at `tool`: its state and holding torques.
fn at_rest(machine: &dyn Machine, tool: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let (mut state, mut torque) = (Vec::new(), vec![0.0; machine.motors().len()]);
    machine.equilibrium(tool, &mut state, &mut torque);
    (state, torque)
}

/// A CoreXY has one idler per belt at each corner both belts pass: two at each back corner and two at each end of the
/// gantry, eight in all (the file's routes name them, so a wrong name would be refused at the build).
#[test]
fn the_corexy_has_an_idler_per_belt_at_each_corner() {
    let machine = sim_netlist::build(&shipped("corexy.toml")).unwrap();
    let (state, _) = at_rest(machine.as_ref(), &[0.3, 0.3]);
    let mut scene = Scene::default();
    machine.scene(&state, &mut scene);
    assert_eq!(scene.parts.iter().filter(|p| p.id == "idler").count(), 8);
}

/// Tooth jump beside a screw: the portal's Z motor drives a screw and has no pulley, so only the two belt motors carry
/// the jump states (JUMP_STATES each, 12 more than without), their signals are two, and the model's rates are finite.
#[test]
fn the_tooth_jump_states_belong_to_the_belt_motors_beside_a_screw() {
    let (off, on) = {
        let cfg = shipped("screw-portal.toml");
        (with(&cfg, &["effects.belt_tooth_jump = false"]), with(&cfg, &["effects.belt_tooth_jump = true"]))
    };
    let (off, on) = (sim_netlist::build(&off).unwrap(), sim_netlist::build(&on).unwrap());
    assert_eq!(on.state_len(), off.state_len() + 2 * JUMP_STATES);
    assert_eq!(on.signal_infos().iter().filter(|s| s.id.ends_with(".tooth_jumps")).count(), 2);
    let (state, torque) = at_rest(on.as_ref(), &[0.3, 0.1, 0.05]);
    let mut rates = vec![0.0; state.len()];
    on.derivatives(&state, &torque, &mut rates);
    assert!(rates.iter().all(|r| r.is_finite()));
}

/// The table's reach with a head's tool offset: the workpiece stays within the round the table can turn under the tool,
/// as wide as the pivot's least distance from an end of the tip's reach in X and Y, which the tool's offset (dx, dy)
/// shifts to [dx, stroke_x + dx] and [dy, stroke_y + dy]. Pivot (0.5, 0.25), strokes 1.0 and 0.5 m: with no offset the
/// half width is 0.25 m; with dx = 0.05 the reach in X is [0.05, 1.05]: min(0.45, 0.55, 0.25, 0.25) = 0.25 still, and
/// with dy = 0.1 the reach in Y is [0.1, 0.6]: min(0.5, 0.5, 0.15, 0.35) = 0.15 m.
#[test]
fn a_tool_offset_narrows_the_tables_reach() {
    let file = text("five-axis-table.toml");
    let half = |offset: &str| {
        let edited = file.replace("tool_m = [0.0, 0.0, -0.2]", &format!("tool_m = {offset}"));
        assert_ne!(edited, file.replace("tool_m = [0.0, 0.0, -0.2]", "tool_m = [9]"));
        let machine = sim_netlist::build(&Config::parse(&edited).unwrap()).unwrap();
        let x = &machine.axes()[0];
        (x.max - x.min) / 2.0
    };
    assert!((half("[0.0, 0.0, -0.2]") - 0.25).abs() < 1e-12);
    assert!((half("[0.05, 0.0, -0.2]") - 0.25).abs() < 1e-12);
    assert!((half("[0.0, 0.1, -0.2]") - 0.15).abs() < 1e-12);
}

/// The single axis with a belt that wraps a pulley and an idler, with a payload on its carriage.
fn belt_axis() -> Config {
    let base = text("single-axis.toml");
    let base = &base[..base.find("\n[machine]\n").unwrap() + 1];
    let netlist = r#"
[machine]
version = 1
id = "wrapped-axis"
axis = [{ id = "x", stroke_m = "geometry.stroke_m" }]
motor = [{ id = "m" }]
gearbox = [{ id = "gearbox", motor = "m" }]
pulley = [{ id = "pulley", gearbox = "gearbox", radius_m = "geometry.pulley_radius_m", inertia_kgm2 = "pulley.inertia_kgm2", at_m = [-0.2, -0.05, 0.03] }]
section = [
    { id = "1", length_m = 0.15, axes = { x = 1 }, pulleys = { pulley = 1 }, route = ["carriage", "pulley"] },
    { id = "2", length_m = 2.45, axes = { x = -1 }, pulleys = { pulley = -1 }, route = ["pulley", "carriage"] },
]
body = [{ id = "carriage", mass_kg = "mass.carriage_kg", payload_kg = "mass.payload_kg", axes = ["x"] }]
guide = [{ id = "guide_x", axis = "x", friction = "friction_x", carries = ["carriage"] }]
"#;
    with(
        &Config::parse(&format!("{base}{netlist}")).unwrap(),
        &["geometry.incline_deg = 0.0", "effects.idlers = false"],
    )
}

/// The spin of the belt drive in the frame momentum, by hand. At the carriage's speed v the pulley (radius r) turns at
/// v / r about the belt's normal and the rotor, i times as fast, at i v / r; a pulley inertia J_p more adds J_p v / r and
/// a rotor inertia J_r more adds J_r i v / r to the angular momentum about that normal (y), nothing about x and z. In
/// both the elastic and the rigid model: here v = 0.5 m/s, r = 0.02 m, J_p + 1e-4 kg m^2 adds 2.5e-3 N m s.
#[test]
fn the_pulleys_and_rotors_spin_is_in_the_frame_momentum() {
    let momentum = |changes: &[&str]| {
        let machine = sim_netlist::build(&with(&belt_axis(), changes)).unwrap();
        let (mut state, _) = at_rest(machine.as_ref(), &[0.4]);
        machine.set_tool_velocity(&mut state, &[0.5]);
        machine.frame_momentum(&state).unwrap().angular
    };
    let cfg = belt_axis();
    let (pulley, rotor, ratio) = (
        cfg.f64("pulley.inertia_kgm2").unwrap(),
        cfg.f64("motor.rotor_inertia_kgm2").unwrap(),
        cfg.f64("gearbox.ratio").unwrap(),
    );
    for stretch in ["true", "false"] {
        let effect = format!("effects.belt_stretch = {stretch}");
        let base = momentum(&[&effect]);
        let heavier = momentum(&[&effect, &format!("pulley.inertia_kgm2 = {:?}", pulley + 1e-4)]);
        let spun = momentum(&[&effect, &format!("motor.rotor_inertia_kgm2 = {:?}", rotor + 1e-4)]);
        for (more, by_hand, what) in
            [(heavier, 1e-4 * 0.5 / 0.02, "pulley"), (spun, 1e-4 * ratio * 0.5 / 0.02, "rotor")]
        {
            let d: Vec<f64> = (0..3).map(|k| more[k] - base[k]).collect();
            assert!(d[0].abs() < 1e-12 && d[2].abs() < 1e-12, "{what}, stretch {stretch}: {d:?}");
            assert!(
                (d[1].abs() - by_hand).abs() < 1e-9 * by_hand,
                "{what}, stretch {stretch}: {d:?} against {by_hand}"
            );
        }
    }
}

/// The rigid friction takes each guide's load: the carriage 4 kg with its payload 2 kg on, over a guide of preload
/// 200 N whose levels were given for the carriage alone, the Coulomb level scales by (P + W) / (P + W_0)
/// = (200 + 6 g) / (200 + 4 g) = 1.0819838 (g = 9.80665 m/s^2) against the same machine without the effect.
#[test]
fn the_rigid_friction_has_the_guides_load_scale() {
    let friction = |on: bool| {
        let cfg = with(&belt_axis(), &[&format!("effects.load_dependent_friction = {on}")]);
        let machine = sim_netlist::build(&cfg).unwrap();
        let mut out = Vec::new();
        machine.rigid_friction(&[0.4], &[1.0], &mut out);
        out
    };
    let (off, on) = (friction(false), friction(true));
    let g = 9.80665;
    let scale = (200.0 + 6.0 * g) / (200.0 + 4.0 * g);
    assert!((scale - 1.081_98_f64).abs() < 1e-5, "{scale}");
    assert!((on[0][0] / off[0][0] - scale).abs() < 1e-9, "{:?} against {:?}", on, off);
    assert!((on[0][1] - off[0][1]).abs() < 1e-12, "the viscous part is not load dependent");
}

/// A gantry's yaw left to its spring: the rack gantry's one rack drives the bridge at one end (0.5 m from its centre),
/// the yaw is on, and the bridge's axis stands vertical so that its weight loads the rack. At rest the sum of the
/// forces on every coordinate vanishes, the yaw's included: the spring holds the drive's moment, so the yaw is not 0
/// and no coordinate (nor pulley nor rotor) accelerates.
#[test]
fn a_yaw_left_to_its_spring_rests_deflected_under_the_drives_moment() {
    let file = text("rack-gantry.toml").replace("gantry_yaw = false", "gantry_yaw = true").replace(
        "id = \"x\"\nstroke_m = \"geometry.stroke_x_m\"",
        "id = \"x\"\nstroke_m = \"geometry.stroke_x_m\"\nincline_deg = 90.0",
    );
    let machine = sim_netlist::build(&Config::parse(&file).unwrap()).unwrap();
    let (state, torque) = at_rest(machine.as_ref(), &[0.3, 0.2]);
    // The coordinates: x, y, the yaw; two motors.
    let (n, m) = (3, 2);
    let v = (state.len() - n - 1 - m) / 2;
    assert!(state[2].abs() > 1e-9, "the yaw stands at {}", state[2]);
    let mut rates = vec![0.0; state.len()];
    machine.derivatives(&state, &torque, &mut rates);
    let worst = rates[v..2 * v].iter().fold(0.0f64, |a, r| a.max(r.abs()));
    assert!(worst < 1e-6, "an acceleration of {worst} at rest, yaw {}", state[2]);
}

/// A rotary's bearing friction at the output (elastic model): the bearing's viscous drag acts on the axis, behind the
/// gearbox's spring, so at the instant the axis turns the rotor's acceleration is the same as without the bearing (the
/// spring has not twisted yet) and the axis's alone is slowed. At the rotor, as the rigid model has it, the rotor's
/// would change.
#[test]
fn a_rotarys_bearing_friction_acts_behind_the_gearbox_spring() {
    let file = text("five-axis-table.toml");
    let bearing = file.replace(
        "inertia_kgm2 = \"mass.a_flange_kgm2\"",
        "inertia_kgm2 = \"mass.a_flange_kgm2\"\nfriction_viscous_nm_s_rad = 2.0",
    );
    assert_ne!(bearing, file);
    let rates = |text: &str| {
        let cfg =
            with(&Config::parse(text).unwrap(), &["effects.guide_friction = true", "effects.belt_stretch = true"]);
        let machine = sim_netlist::build(&cfg).unwrap();
        let id_at = |ids: Vec<&str>, id: &str| ids.iter().position(|x| *x == id).unwrap();
        let axis = id_at(machine.axes().iter().map(|a| a.id).collect(), "a");
        let motor = id_at(machine.motors().iter().map(|a| a.id).collect(), "ma");
        let (mut state, torque) = at_rest(machine.as_ref(), &[0.5, 0.25, -0.1, 0.2, 0.1]);
        machine.set_tool_velocity(&mut state, &[0.0, 0.0, 0.0, 1.0, 0.0]);
        let (n, m) = (machine.axes().len(), machine.motors().len());
        let v = (state.len() - n - 1 - m) / 2;
        let mut d = vec![0.0; state.len()];
        machine.derivatives(&state, &torque, &mut d);
        (d[v + axis], d[v + n + m + motor])
    };
    let ((axis_off, rotor_off), (axis_on, rotor_on)) = (rates(&file), rates(&bearing));
    assert!((rotor_on - rotor_off).abs() < 1e-9 * (1.0 + rotor_off.abs()), "rotor {rotor_on} against {rotor_off}");
    assert!((axis_on - axis_off).abs() > 1e-6, "axis {axis_on} against {axis_off}");
}
