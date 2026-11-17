//! Machine files' physics beside the belt: tooth jump beside a screw, the CoreXY's idlers, a table's reach with a head's
//! tool offset, the frame momentum's spin, a gantry yaw's spring, a rotary's bearing at the output and the rigid
//! friction's load (crates/sim-netlist).

use sim_components::mech::JUMP_STATES;
use sim_core::{Config, Machine, Scene};
use sim_machines::{find, load_file};
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

/// The H-bot standing up (its Y axis vertical) with its bridge's yaw on: the two motors set X and Y, the yaw is left to
/// the spring (k = 5000 N m/rad). The belt that holds the head's weight W = (5 + 2) kg g along Y pulls the bridge's ends
/// with the moment 0.3 m W (the file's comment: the belt's moment is -0.3 m times its pull on the head along Y), so at
/// rest |psi| = 0.3 W / k = 4.119e-3 rad, and no coordinate accelerates.
fn standing_hbot(changes: &[&str]) -> Box<dyn Machine> {
    let file = text("h-bot.toml").replace("gantry_yaw = false", "gantry_yaw = true").replace(
        "id = \"y\"\nstroke_m = \"geometry.stroke_y_m\"",
        "id = \"y\"\nstroke_m = \"geometry.stroke_y_m\"\nincline_deg = 90.0",
    );
    assert!(file.contains("incline_deg = 90.0"));
    sim_netlist::build(&with(&Config::parse(&file).unwrap(), changes)).unwrap()
}

/// The standing H-bot's yaw at rest against the hand value, and the accelerations there zero.
fn rests_on_its_spring(machine: &dyn Machine, velocities: std::ops::Range<usize>) {
    let (state, torque) = at_rest(machine, &[0.5, 0.2]);
    let by_hand = 0.3 * 7.0 * 9.80665 / 5000.0;
    assert!((state[2].abs() - by_hand).abs() < 1e-6 * by_hand, "the yaw stands at {}, by hand {by_hand}", state[2]);
    let mut rates = vec![0.0; state.len()];
    machine.derivatives(&state, &torque, &mut rates);
    let worst = rates[velocities].iter().fold(0.0f64, |a, r| a.max(r.abs()));
    assert!(worst < 1e-6, "an acceleration of {worst} at rest, yaw {}", state[2]);
}

#[test]
fn a_yaw_left_to_its_spring_rests_deflected_under_the_belts_moment() {
    let machine = standing_hbot(&["effects.belt_stretch = true"]);
    // Elastic state: x, y, the yaw, two pulleys, two rotors, then their rates.
    let v = 3 + 2 + 2;
    rests_on_its_spring(machine.as_ref(), v..2 * v);
}

/// A rotary's bearing friction at the output (elastic model): the bearing's viscous drag acts on the axis, behind the
/// gearbox's spring, so at the instant the axis turns the rotor's acceleration is the same as without the bearing (the
/// spring has not twisted yet) and the axis's alone is slowed. At the rotor, as the rigid model has it, the rotor's
/// would change. By hand: the drag b w = 2 N m at w = 1 rad/s takes b w^2 = 2 W from the motion, so the energy's rate
/// (kinetic plus potential, along the derivatives) falls by exactly 2 W, and the motor's shaft friction row books 2 W more.
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
        let h = 1e-6;
        let at = |sign: f64| state.iter().zip(&d).map(|(x, r)| x + sign * h * r).collect::<Vec<_>>();
        let energy_rate = (machine.energy(&at(1.0)) - machine.energy(&at(-1.0))) / (2.0 * h);
        let mut losses = Vec::new();
        machine.loss_powers(&state, &torque, &mut losses);
        (d[v + axis], d[v + n + m + motor], energy_rate, losses)
    };
    let ((axis_off, rotor_off, e_off, l_off), (axis_on, rotor_on, e_on, l_on)) = (rates(&file), rates(&bearing));
    assert!((rotor_on - rotor_off).abs() < 1e-9 * (1.0 + rotor_off.abs()), "rotor {rotor_on} against {rotor_off}");
    assert!((axis_on - axis_off).abs() > 1e-6, "axis {axis_on} against {axis_off}");
    assert!((e_on - e_off + 2.0).abs() < 1e-6, "energy rate {e_on} against {e_off}");
    let more: f64 = l_on.iter().zip(&l_off).map(|(a, b)| a - b).sum();
    assert!((more - 2.0).abs() < 1e-9, "losses {l_on:?} against {l_off:?}");
}

/// The T-shaped belt gantry's file with the effects `on` switched on, everything that loses energy off (damping of the belt,
/// the frame, the lean and the gearboxes, friction, ripple, tooth meshing, the brake).
fn lossless_gantry(on: &[&str]) -> Box<dyn Machine> {
    let mut changes: Vec<String> = [
        "belt_damping",
        "guide_friction",
        "load_dependent_friction",
        "motor_friction",
        "gearbox_efficiency",
        "gearbox_backlash",
        "torque_ripple",
        "tooth_meshing",
        "holding_brake",
    ]
    .iter()
    .map(|e| format!("effects.{e} = false"))
    .collect();
    changes.extend(
        ["frame.damping_n_s_m", "tool_flex.damping_n_s_m", "gearbox.damping_nm_s_rad"].map(|k| format!("{k} = 0.0")),
    );
    changes.extend(on.iter().map(|e| format!("effects.{e} = true")));
    let changes: Vec<&str> = changes.iter().map(String::as_str).collect();
    sim_netlist::build(&with(&shipped("t-gantry.toml"), &changes)).unwrap()
}

/// The idlers' inertia rides on the axes' speeds alone, not on the swaying frame's: the belt's speed past a carriage is the
/// difference of two speeds over ground, in which the sway cancels. With the frame swaying at 0.2 m/s the idlers' share of
/// the kinetic energy is what it is without the sway: 1/2 (J / r^2) sum (s_d . v)^2 over the five idlers, whose belt speeds
/// are vz (twice, the upper ones), 2 vx + vz and 2 vx - vz (the lower ones) and 2 vx (the one at the Z axis's bottom):
/// 12 vx^2 + 4 vz^2 in all, 2.28 at (0.4, -0.3), times J / r^2 / 2 = 2e-6 / 0.012^2 / 2 = 6.9444e-3: 0.0158333 J.
#[test]
fn the_idlers_inertia_rides_on_the_axes_speeds_and_not_on_the_swaying_frame() {
    let energy = |idlers: bool, sway: f64| {
        let machine = lossless_gantry(if idlers { &["frame_compliance", "idlers"] } else { &["frame_compliance"] });
        let (mut state, _) = at_rest(machine.as_ref(), &[0.5, 0.25]);
        machine.set_tool_velocity(&mut state, &[0.4, -0.3]);
        // 7 coordinates (two axes, two pulleys, two rotors, the frame): the frame's rate is the seventh velocity, after
        // the seven positions and the six velocities before it.
        state[7 + 6] = sway;
        machine.energy(&state)
    };
    let hand = 0.5 * 2.0e-6 / 0.012_f64.powi(2) * (12.0 * 0.4 * 0.4 + 4.0 * 0.3 * 0.3);
    assert!((hand - 0.015_833_3).abs() < 1e-6, "{hand}");
    for sway in [0.0, 0.2] {
        let (off, on) = (energy(false, sway), energy(true, sway));
        assert!((on - off - hand).abs() < 1e-9, "sway {sway}: the idlers carry {} J, by hand {hand}", on - off);
    }
}

/// The leaning body's weight rises as it leans: with the spring's force F = k w holding it out its centre of mass rises
/// by F^2 r, r = 1/2 h (h / tilt)^2 for a guide tilting under the force (a bar turned by theta = F h / tilt along its
/// whole length is shorter by theta^2 / 2 per length), so V = m g k^2 w^2 r = 1/2 c w^2, c = m g k^2 h^3 / tilt^2, h the
/// centre of mass's distance below the carriage. The energy of a lean w at rest is then 1/2 (k + c) w^2 with k =
/// 1 / (1 / lateral + h^2 / tilt); at z = 0.1 with h = 0.3 - 0.1 = 0.2 m, the Z axis 5 kg (the payload set down), lateral
/// 2e7 N/m and tilt 1.2e4 N m/rad: k = 295 566 N/m. And the flow keeps that energy (the weight's stiffness and its rate
/// with z are in the equations of motion): the energy's rate along a moving state with the lean out is the drives' power.
#[test]
fn the_leaning_bodys_weight_is_in_the_energy_and_the_flow_conserves_it() {
    let mut machine = lossless_gantry(&["tool_flex"]);
    machine.set_payload_attached(false);
    let (mut state, _) = at_rest(machine.as_ref(), &[0.5, 0.1]);
    let (h, lateral, tilt, m, g) = (0.2_f64, 2.0e7_f64, 1.2e4_f64, 5.0_f64, 9.80665_f64);
    let k = 1.0 / (1.0 / lateral + h * h / tilt);
    let c = m * g * k * k * h.powi(3) / (tilt * tilt);
    let at_rest_energy = machine.energy(&state);
    // Seven coordinates with the lean off the frame: two axes, two pulleys, two rotors, then the lean.
    let w = 1.0e-4;
    state[6] = w;
    let by_hand = 0.5 * (k + c) * w * w;
    let got = machine.energy(&state) - at_rest_energy;
    assert!((got - by_hand).abs() < 1e-9 * by_hand, "{got} against {by_hand} (k {k}, c {c})");
    machine.set_tool_velocity(&mut state, &[0.4, -0.3]);
    let torque = [0.3, -0.2];
    let mut rate = vec![0.0; state.len()];
    machine.derivatives(&state, &torque, &mut rate);
    let energy = |h: f64| machine.energy(&state.iter().zip(&rate).map(|(x, r)| x + h * r).collect::<Vec<_>>());
    let h = 1e-7;
    let rate_of_energy = (8.0 * (energy(h) - energy(-h)) - (energy(2.0 * h) - energy(-2.0 * h))) / (12.0 * h);
    let (mut angle, mut speed) = (vec![0.0; 2], vec![0.0; 2]);
    machine.motor_state(&state, &mut angle, &mut speed);
    let power: f64 = torque.iter().zip(&speed).map(|(t, w)| t * w).sum();
    assert!((rate_of_energy - power).abs() < 2e-8 * power.abs().max(1.0), "{rate_of_energy} against {power}");
}

/// The file's lowest belt mode is the crate's at a pose, with the Z axis leaning (its weight rising, its spring stiffening
/// with z under the process force's lean) and with the frame swaying and the idlers on too, to 1e-7.
#[test]
fn the_files_lowest_mode_is_the_crates() {
    let krate = find("t-gantry").unwrap();
    let entry = load_file(&root().join("machines").join("t-gantry.toml")).unwrap_or_else(|e| panic!("{e}"));
    for on in [vec!["tool_flex"], vec!["frame_compliance", "tool_flex", "idlers"]] {
        let (mut a, mut b) = (krate.config(), entry.config());
        for cfg in [&mut a, &mut b] {
            for effect in ["frame_compliance", "tool_flex", "idlers"] {
                cfg.set_text(&format!("effects.{effect} = {}", on.contains(&effect))).unwrap();
            }
            for text in ["effects.process_force = true", "process.force_x_n = 30.0"] {
                cfg.set_text(text).unwrap();
            }
        }
        let (mut ma, mut mb) = ((krate.build)(&a).unwrap(), (entry.build)(&b).unwrap());
        for machine in [&mut ma, &mut mb] {
            machine.set_payload_attached(true);
            machine.set_dwelling(true);
        }
        for pose in [[0.5, 0.25], [0.9, 0.05]] {
            let (fa, fb) = (ma.resonance_hz(&pose).unwrap(), mb.resonance_hz(&pose).unwrap());
            assert!((fa - fb).abs() < 1e-7 * fa, "{on:?} at {pose:?}: crate {fa} Hz, file {fb} Hz");
        }
    }
}

/// A rack's pinion and a rotary's motor spin in the frame momentum. A rotor of the inertia J turning at w adds J w about its
/// axis: for the rack along x the pinion above it turns about e x up = -y (the rack moves along +x under it), for the
/// rotary C about its own axis, here the vertical (A at 0). 1e-3 kg m^2 more rotor inertia adds 1e-3 w.
#[test]
fn a_racks_pinion_and_a_rotarys_motor_spin_in_the_frame_momentum() {
    let more = |name: &str, speed: &[f64], motor: usize, pose: &[f64]| {
        let angular = |inertia: f64| {
            let cfg = with(&shipped(name), &[&format!("motor.rotor_inertia_kgm2 = {inertia}")]);
            let machine = sim_netlist::build(&cfg).unwrap();
            let (mut state, _) = at_rest(machine.as_ref(), pose);
            machine.set_tool_velocity(&mut state, speed);
            let (mut angle, mut rotor) = (vec![0.0; machine.motors().len()], vec![0.0; machine.motors().len()]);
            machine.motor_state(&state, &mut angle, &mut rotor);
            (machine.frame_momentum(&state).unwrap().angular, rotor[motor])
        };
        let ((low, w), (high, _)) = (angular(0.3e-4), angular(0.3e-4 + 1e-3));
        ([0, 1, 2].map(|k| high[k] - low[k]), w)
    };
    let (d, w) = more("rack-gantry.toml", &[0.5, 0.0], 0, &[0.3, 0.2]);
    assert!(w.abs() > 1.0, "the pinion turns at {w}");
    assert!(d[0].abs() < 1e-12 && d[2].abs() < 1e-12, "{d:?}");
    assert!((d[1] + 1e-3 * w).abs() < 1e-9 * (1e-3 * w).abs(), "{d:?} against {}", -1e-3 * w);
    let (d, w) = more("five-axis-table.toml", &[0.0, 0.0, 0.0, 0.0, 0.8], 4, &[0.5, 0.25, -0.1, 0.0, 0.1]);
    assert!(w.abs() > 0.8, "the rotor turns at {w}");
    let size = d.iter().map(|x| x * x).sum::<f64>().sqrt();
    assert!((size - 1e-3 * w.abs()).abs() < 1e-9 * size, "{d:?} against {}", 1e-3 * w);
    assert!((d[2].abs() - size).abs() < 1e-9 * size, "about the vertical: {d:?}");
}

/// A single belt axis whose pulley stands level with the carriage's clamp, with two spans (a run from the clamp to the
/// pulley and one back) that move with the carriage.
fn runs_axis(belt_mass: bool) -> Config {
    let base = text("single-axis.toml");
    let base = &base[..base.find("\n[machine]\n").unwrap() + 1];
    let netlist = r#"
[machine]
version = 1
id = "runs-axis"
axis = [{ id = "x", stroke_m = "geometry.stroke_m" }]
motor = [{ id = "m" }]
gearbox = [{ id = "gearbox", motor = "m" }]
pulley = [{ id = "pulley", gearbox = "gearbox", radius_m = "geometry.pulley_radius_m", inertia_kgm2 = "pulley.inertia_kgm2", at_m = [-0.2, -0.05, 0.06] }]
section = [
    { id = "1", length_m = 0.15, axes = { x = 1 }, pulleys = { pulley = 1 }, route = ["carriage", "pulley"] },
    { id = "2", length_m = 2.45, axes = { x = -1 }, pulleys = { pulley = -1 }, route = ["pulley", "carriage"] },
]
span = [
    { id = "out", section = "1", length_m = 0.6, velocity_x = { x = 1 } },
    { id = "back", section = "2", length_m = 0.2, velocity_x = { x = 1 } },
]
body = [{ id = "carriage", mass_kg = "mass.carriage_kg", payload_kg = "mass.payload_kg", axes = ["x"] }]
guide = [{ id = "guide_x", axis = "x", friction = "friction_x", carries = ["carriage"] }]
"#;
    with(
        &Config::parse(&format!("{base}{netlist}")).unwrap(),
        &["geometry.incline_deg = 0.0", "effects.idlers = false", &format!("effects.belt_mass = {belt_mass}")],
    )
}

/// The belt's runs carry angular momentum about the origin of the bodies' places, m x (mu L y), each about the middle of its
/// straight piece. The carriage at x = 0.4 (its clamp, a wheel of radius 0), the pulley (radius r = 0.02, anticlockwise) 0.6 m
/// away at the same height: the straight run from the clamp to the pulley leans by alpha, sin alpha = r / 0.6, and meets
/// the wheel at r cos(alpha) above the line (and the run back leaves it r cos(alpha) below), so the runs' middles are
/// +-1/2 r cos(alpha) high. Both move at v = 0.5 m/s along x, the first 0.6 m long, the second 0.2 m, mu = 0.1 kg/m:
/// L_y = mu v (0.6 - 0.2) r cos(alpha) / 2 = 0.1 * 0.5 * 0.4 * 0.02 * 0.99944 / 2 = 1.99889e-4 N m s (m_z y_x only, y_z 0).
#[test]
fn the_belts_runs_have_angular_momentum_about_the_middle_of_their_straight_piece() {
    let angular = |belt: bool, stretch: bool| {
        let cfg = with(&runs_axis(belt), &[&format!("effects.belt_stretch = {stretch}")]);
        let machine = sim_netlist::build(&cfg).unwrap();
        let (mut state, _) = at_rest(machine.as_ref(), &[0.4]);
        machine.set_tool_velocity(&mut state, &[0.5]);
        machine.frame_momentum(&state).unwrap().angular
    };
    let cos = (1.0 - (0.02_f64 / 0.6).powi(2)).sqrt();
    let hand = 0.1 * 0.5 * (0.6 - 0.2) * 0.02 * cos / 2.0;
    for stretch in [false, true] {
        let (with_belt, without) = (angular(true, stretch), angular(false, stretch));
        let d: Vec<f64> = (0..3).map(|k| with_belt[k] - without[k]).collect();
        assert!(d[0].abs() < 1e-12 && d[2].abs() < 1e-12, "stretch {stretch}: {d:?}");
        assert!((d[1] - hand).abs() < 1e-9 * hand, "stretch {stretch}: {} against {hand}", d[1]);
    }
}

/// The standing H-bot's yaw in the rigid model: the motors' torques carry X and Y, K^T P = -gravity there, and what P
/// then puts on the yaw (the same 0.3 m W) the spring holds: the same deflection, and no coordinate accelerates.
#[test]
fn a_yaw_left_to_its_spring_rests_deflected_in_the_rigid_model_too() {
    let machine = standing_hbot(&["effects.belt_stretch = false"]);
    rests_on_its_spring(machine.as_ref(), 3..6);
}

/// The guides' Stribeck rise reaches the planner: the T-gantry's X guide has Coulomb 15 N and stiction 20 N, so along X
/// the rise (20 - 15) is 5 / 15 of the Coulomb level at each motor (both are the guide's force times the same ratio to
/// the motors), and it fades over the guide's Stribeck speed, 10 mm/s, along a path that moves X at 1 per unit.
#[test]
fn the_guides_stribeck_rise_is_offered_to_the_planner() {
    let machine = sim_netlist::build(&shipped("t-gantry.toml")).unwrap();
    let (mut plain, mut rise) = (Vec::new(), Vec::new());
    machine.rigid_friction(&[0.5, 0.25], &[1.0, 0.0], &mut plain);
    machine.rigid_friction_rise(&[0.5, 0.25], &[1.0, 0.0], &mut rise);
    assert_eq!((plain.len(), rise.len()), (2, 2));
    for (c, r) in plain.iter().zip(&rise) {
        assert!(c[0].abs() > 0.0, "{plain:?}");
        assert_eq!(r.len(), 1, "{rise:?}");
        assert!((r[0][0] - c[0] * 5.0 / 15.0).abs() < 1e-12 * c[0].abs(), "{plain:?} against {rise:?}");
        assert!((r[0][1] - 0.01).abs() < 1e-12, "{rise:?}");
    }
}

/// Two guides on one motor keep their own rises: the CoreXY's motors both drive X (friction_z: Coulomb 10 N, stiction 14 N)
/// and Y (friction_x: 15 N, 20 N), Stribeck speed 10 mm/s each. Along a path that moves X at 1 and Y at 0.5 per unit, each motor has the
/// X guide's rise, 4 / 10 of the Coulomb torque X alone gives it, over 0.01 / 1 = 0.01, and the Y guide's, 5 / 15 of
/// what Y alone gives, over 0.01 / 0.5 = 0.02: two Stribeck speeds, which no one pair can stand for.
#[test]
fn each_guide_on_a_motor_keeps_its_own_stribeck_rise() {
    let machine = sim_netlist::build(&shipped("corexy.toml")).unwrap();
    let alone = |dir: [f64; 2]| {
        let mut plain = Vec::new();
        machine.rigid_friction(&[0.25, 0.25], &dir, &mut plain);
        plain
    };
    let (x, y) = (alone([1.0, 0.0]), alone([0.0, 1.0]));
    let mut rise = Vec::new();
    machine.rigid_friction_rise(&[0.25, 0.25], &[1.0, 0.5], &mut rise);
    assert_eq!(rise.len(), 2);
    for kk in 0..2 {
        let mut pairs = rise[kk].clone();
        pairs.sort_by(|a, b| a[1].total_cmp(&b[1]));
        assert_eq!(pairs.len(), 2, "{rise:?}");
        for ([level, speed], (by_hand, at)) in
            pairs.iter().zip([(x[kk][0].abs() * 0.4, 0.01), (y[kk][0].abs() * 5.0 / 15.0, 0.02)])
        {
            assert!(
                by_hand > 0.0 && (level.abs() - by_hand).abs() < 1e-12 * by_hand,
                "motor {kk}: {pairs:?}, {by_hand}"
            );
            assert!((speed - at).abs() < 1e-12, "motor {kk}: {pairs:?}");
        }
    }
}
