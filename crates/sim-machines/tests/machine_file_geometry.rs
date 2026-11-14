//! Machine files with geometry beside their netlist (crates/sim-netlist, docs/machine-files.md): the belt's wrap from
//! the places of pulleys and idlers, worm gears, and the catalogue search for one motor of a file. The leaning Z axis
//! (machine.lean) is checked against the T-gantry crate in machine_files.rs.

use sim_components::catalog::{motor_config, motor_mass, paired_drive, search_motor, with_motor_part};
use sim_core::{Config, ConfigError, Machine, MachineEntry};
use sim_machines::{find, load_file};
use std::f64::consts::PI;
use std::path::{Path, PathBuf};

fn root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..")
}

fn file(name: &str) -> &'static MachineEntry {
    load_file(&root().join("machines").join(name)).unwrap_or_else(|e| panic!("{e}"))
}

fn close(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol * (1.0 + a.abs().max(b.abs()))
}

/// A machine's signals by id.
fn signal(machine: &dyn Machine, state: &[f64], torque: &[f64], id: &str) -> f64 {
    let mut values = Vec::new();
    machine.signals(state, torque, &mut values);
    let at = machine.signal_infos().iter().position(|s| s.id == id).unwrap_or_else(|| panic!("no signal {id}"));
    values[at]
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

/// The single axis of machines/single-axis.toml, level, with its belt's places: the motor pulley (r = 0.02 m) at
/// x = -0.2, z = 0.03; the carriage's clamp at z = 0.06 (where the 3D view puts it), 0.03 above the pulley's centre; and
/// an idler of radius 0.012 m 0.3 along and 0.06 below the pulley, which the belt runs over from the pulley's bottom.
/// The belt runs from the clamp over the pulley's top round it anticlockwise, to the idler and round it clockwise.
const WRAPPED_NETLIST: &str = r#"
[machine]
version = 1
id = "wrapped-axis"
axis = [{ id = "x", stroke_m = "geometry.stroke_m" }]
motor = [{ id = "m" }]
gearbox = [{ id = "gearbox", motor = "m" }]
pulley = [{ id = "pulley", gearbox = "gearbox", radius_m = "geometry.pulley_radius_m", inertia_kgm2 = "pulley.inertia_kgm2", at_m = [-0.2, -0.05, 0.03] }]
section = [
    { id = "1", length_m = 0.15, axes = { x = 1 }, pulleys = { pulley = 1 }, route = ["carriage", "pulley"] },
    { id = "2", length_m = 2.45, axes = { x = -1 }, pulleys = { pulley = -1 }, route = ["pulley", "low", "carriage"] },
]
body = [{ id = "carriage", mass_kg = "mass.carriage_kg", axes = ["x"] }]
guide = [{ id = "guide_x", axis = "x", friction = "friction_x", carries = ["carriage"] }]
idler = [{ id = "low", speed = { x = 1 }, radius_m = 0.012, inertia_kgm2 = 2.0e-6, friction_nm = 0.002, at_m = [0.1, -0.05, -0.03], clockwise = true }]
"#;

/// The wrap at the motor pulley by hand (`mech::wrap_angle`): half a turn less the lean of both spans. To the clamp,
/// d = x + 0.2 along and h = 0.03 up, no radius: atan2(0.03, d) - asin(0.02 / hypot(d, 0.03)); at x = 0 that is
/// 0.1488899 - 0.0990555 = 0.0498344 rad, at x = 0.8 m 0.0299910 - 0.0199923 = 0.0099987 rad. To the idler, an inner
/// tangent 0.3 along and 0.06 down: atan2(0.06, 0.3) - asin(0.032 / hypot(0.3, 0.06)) = 0.1973956 - 0.1047869 =
/// 0.0926086 rad. So 2.9991496 rad at x = 0 and 3.0389854 rad at x = 0.8 m: 12.0 and 12.2 teeth of 5 mm in mesh.
#[test]
fn the_belt_wraps_a_pulley_as_far_as_the_places_of_its_neighbours_say() {
    let text = std::fs::read_to_string(root().join("machines/single-axis.toml")).unwrap().replace("\r\n", "\n");
    let base = &text[..text.find("\n[machine]\n").unwrap() + 1];
    let cfg = Config::parse(&format!("{base}{WRAPPED_NETLIST}")).unwrap();
    let cfg = with(&cfg, &["geometry.incline_deg = 0.0", "effects.belt_wrap = true"]);
    let machine = sim_netlist::build(&cfg).unwrap();
    let lean = |d: f64, h: f64, r2: f64| h.atan2(d) - ((0.02 + r2) / d.hypot(h)).asin();
    for (x, by_hand) in [(0.0, 2.999_149_6), (0.8, 3.038_985_4)] {
        let wraps = machine.belt_wraps(&[x]);
        let formula = PI - lean(x + 0.2, 0.03, 0.0) - lean(0.3, 0.06, 0.012);
        assert!(close(wraps[0], formula, 1e-12), "x = {x}: {wraps:?} against {formula}");
        assert!((wraps[0] - by_hand).abs() < 2e-7, "x = {x}: {wraps:?} against {by_hand}");
    }
    // Off, the machine gives no wrap and the configured teeth in mesh stand.
    let off = sim_netlist::build(&with(&cfg, &["effects.belt_wrap = false"])).unwrap();
    assert!(off.belt_wraps(&[0.3]).is_empty());
    // A route that does not run through the pulley is refused by name.
    let broken = format!(
        "{base}{}",
        WRAPPED_NETLIST.replace(r#"route = ["carriage", "pulley"]"#, r#"route = ["pulley", "carriage"]"#)
    );
    let broken = with(&Config::parse(&broken).unwrap(), &["effects.belt_wrap = true"]);
    assert_eq!(
        sim_netlist::build(&broken).err(),
        Some(ConfigError::Invalid(
            "effects.belt_wrap (pulley pulley has to end the route of one section and start the route of another)"
                .into()
        ))
    );
}

/// The screw portal's belts wrap their pulleys beside its Z screw (effects.belt_wrap), whose motor drives no belt and
/// has no wrap (NaN). Each belt runs from its carriage's clamp round its pulley (r = 0.02 m, anticlockwise) and back to
/// the same clamp, both at z = 0.06 m in the belt's plane: the two runs are the tangents from one point at the distance
/// D, each leaning asin(r / D) off the line of centres, so the belt wraps half a turn plus twice that,
/// pi + 2 asin(r / D). With the tool at x = 0.3 m: X's pulley at x = -0.1 m, D = 0.4 m, pi + 2 asin(0.05) =
/// 3.2416344 rad; Y's at x = 0.5 m, D = 0.2 m, pi + 2 asin(0.1) = 3.3419275 rad.
#[test]
fn the_belts_wrap_their_pulleys_beside_a_screw() {
    let cfg = with(&file("screw-portal.toml").config(), &["effects.belt_wrap = true"]);
    let machine = sim_netlist::build(&cfg).unwrap();
    let wraps = machine.belt_wraps(&[0.3, 0.1, 0.05]);
    assert_eq!(wraps.len(), 3, "{wraps:?}");
    assert!(close(wraps[0], PI + 2.0 * 0.05f64.asin(), 1e-12) && (wraps[0] - 3.241_634_4).abs() < 1e-7, "{wraps:?}");
    assert!(close(wraps[1], PI + 2.0 * 0.1f64.asin(), 1e-12) && (wraps[1] - 3.341_927_5).abs() < 1e-7, "{wraps:?}");
    assert!(wraps[2].is_nan(), "the screw's motor: {wraps:?}");
}

/// A joint kit machine written as a machine file: its default config, the tables a machine file reads that the kit has
/// not taken from machines/single-axis.toml, and `netlist` as its [machine] table.
fn kit_as_file(id: &str, netlist: &str) -> Config {
    fn fill(into: &mut toml::Table, from: &toml::Table) {
        for (key, value) in from {
            match (into.get_mut(key), value) {
                (Some(toml::Value::Table(a)), toml::Value::Table(b)) => fill(a, b),
                (None, v) => {
                    into.insert(key.clone(), v.clone());
                }
                _ => {}
            }
        }
    }
    let mut kit: toml::Table = find(id).unwrap().default_config.parse().unwrap();
    let mut single: toml::Table =
        std::fs::read_to_string(root().join("machines/single-axis.toml")).unwrap().parse().unwrap();
    single.remove("machine");
    fill(&mut kit, &single);
    kit.insert("machine".into(), toml::Value::Table(netlist.parse().unwrap()));
    Config::parse(&toml::to_string(&kit).unwrap()).unwrap()
}

/// The joint kit's rotary table as a netlist, its gear a worm stage of the given ratio (a number or a path) and
/// efficiencies.
fn worm_table(ratio: &str, efficiency: f64, back: f64) -> String {
    format!(
        r#"
version = 1
id = "worm-table-file"
axis = [{{ id = "c", stroke_rad = 4.0 }}]
motor = [{{ id = "1" }}]
gearbox = [{{ id = "gear", motor = "1" }}]
worm = [{{ id = "worm", gearbox = "gear", ratio = {ratio}, efficiency = {efficiency}, back_efficiency = {back} }}]
rotary = [{{ id = "table", gearbox = "gear", axis = "c", inertia_kgm2 = "table.inertia_kgm2", payload_inertia_kgm2 = "table.payload_inertia_kgm2" }}]
"#
    )
}

/// The joint kit's rotary table with its worm gear (table.gear = "worm": the gearbox with the efficiency of [worm],
/// tan 8 deg / tan(8 deg + atan(0.04 / cos 20 deg)) = 0.762938, the same back) and the same table as a file with a worm
/// node of the kit's ratio and that efficiency both ways: the same machine, rigid and elastic, driving forward and driven back. The kit's
/// joint friction and the file's motor friction are off; the states are laid out as for the screw axis
/// (machine_file_mechanisms.rs).
#[test]
fn a_worm_gear_in_a_file_drives_as_the_joint_kits_table() {
    let worm = sim_components::transmission::WormGear {
        lead_angle: 8f64.to_radians(),
        friction: 0.04,
        pressure_angle: 20f64.to_radians(),
    };
    let eta = worm.efficiency();
    assert!((eta - 0.762_938).abs() < 1e-6, "{eta}");
    for elastic in [true, false] {
        let mut kit = find("rotary-table").unwrap().config();
        for change in [
            "table.gear = \"worm\"",
            "effects.joint_friction = false",
            "effects.motor_friction = false",
            &format!("effects.joint_elasticity = {elastic}"),
        ] {
            kit.set_text(change).unwrap();
        }
        let mut from_file = kit_as_file("rotary-table", &worm_table("\"gearbox.ratio\"", eta, eta));
        for change in [&format!("effects.belt_stretch = {elastic}"), "effects.motor_friction = false"] {
            from_file.set_text(change).unwrap();
        }
        let kit = (find("rotary-table").unwrap().build)(&kit).unwrap();
        let from_file = sim_netlist::build(&from_file).unwrap();
        let file_state = |s: &[f64]| {
            if elastic { vec![s[0], s[0], s[2], s[1], s[1], s[3], 0.0, 0.0, s[4]] } else { vec![s[0], s[1], 0.0, s[2]] }
        };
        let rates: &[(usize, usize)] = if elastic { &[(0, 0), (1, 3), (2, 2), (3, 5)] } else { &[(0, 0), (1, 1)] };
        // Driving forward, then driven back (the motor braking against the table's speed).
        for (speed, extra) in [(2.0, 0.7), (2.0, -3.0)] {
            let what = format!("elastic {elastic}, torque {extra}");
            let (mut sa, mut held) = (Vec::new(), [0.0]);
            kit.equilibrium(&[1.0], &mut sa, &mut held);
            kit.set_tool_velocity(&mut sa, &[speed]);
            for (i, x) in sa.iter_mut().enumerate() {
                *x += 1e-5 * (1.0 + i as f64);
            }
            let sb = file_state(&sa);
            let torque = [held[0] + extra];
            let (mut da, mut db) = (vec![0.0; sa.len()], vec![0.0; sb.len()]);
            kit.derivatives(&sa, &torque, &mut da);
            from_file.derivatives(&sb, &torque, &mut db);
            for &(i, j) in rates {
                assert!(close(da[i], db[j], 1e-9), "{what}: rate {i}: crate {}, file {}", da[i], db[j]);
            }
            for id in ["gearbox.1.output_torque", "power.gearbox_1", "motor.1.torque_inertia"] {
                let (x, y) = (signal(kit.as_ref(), &sa, &torque, id), signal(from_file.as_ref(), &sb, &torque, id));
                assert!(close(x, y, 1e-9), "{what}: {id}: crate {x}, file {y}");
            }
        }
    }
}

/// The worm table driven back and held by hand, rigid, the motor without torque or friction, while a process torque of
/// -10 N m acts on the table (effects.process_force, the cycle dwelling). The table and payload turn 3 + 1.5 = 4.5 kg m2,
/// the rotor 1.7e-4 + 1.4e-4 kg m2 behind the worm's ratio 40: R = 40^2 3.1e-4 = 0.496 kg m2 at the table.
///
/// - A worm that turns back (eta_back = 0.5), already turning back at -10 rad/s: the rotor takes the load's torque less
///   the loss, the table's own acceleration a from J a = P + o with the rotor's R a = eta_back o:
///   a = P / (J + R / eta_back) = -10 / (4.5 + 0.992) = -1.820830 rad/s2. At rest it starts as a gear of 1 would:
///   -10 / (4.5 + 0.496) = -2.001601 rad/s2.
/// - A self-locking one (eta_back = -0.2, forward 0.8) holds the 10 N m at rest with no motor torque. Lowering at
///   -10 rad/s with no motor torque it brakes itself: the worm drags by (1 - eta_back) 10 = 12 N m, so the table slows
///   at (12 - 10) / 4.996 = 0.400320 rad/s2, the gear passing 12 - 0.496 * 0.400320 = 11.801441 N m. The motor has to
///   push to lower it: -eta_back 10 / 40 = 0.05 N m against the motion holds the speed; 0.1 N m speeds it up at
///   (12 - 10 - 4) / 4.996 = -0.400320 rad/s2.
/// - From rest it sticks while the worm can take what the motor and load ask of it, from -(1 / 0.8 - 1) 10 = -2.5 N m
///   (lifting: the motor needs 10 / (0.8 * 40) = 0.3125 N m) to 12 N m (lowering: 0.05 N m): -0.04 N m (11.6) and
///   +0.3 N m (-2) hold it; -0.06 N m breaks it loose downwards at (12 - 10 - 2.4) / 4.996 = -0.080064 rad/s2, +0.35 N m
///   upwards at (14 - 10 - 2.5) / 4.996 = 0.300240 rad/s2.
///
/// Elastic (backlash off), at rest with the gear twisted by 10 N m / 67000 N m/rad so that it holds the load: the table
/// stands for both; the rotor of the worm that turns back takes 10 / 40 N m and turns back at 0.25 / 3.1e-4 =
/// 806.4516 rad/s2, the self-locking one's stands.
#[test]
fn a_self_locking_worm_holds_its_load_and_one_that_turns_back_loses_by_its_back_efficiency() {
    let machine = |back: f64, elastic: bool| {
        let cfg = kit_as_file("rotary-table", &worm_table("40", 0.8, back));
        let cfg = with(
            &cfg,
            &[
                &format!("effects.belt_stretch = {elastic}"),
                "effects.gearbox_backlash = false",
                "effects.motor_friction = false",
                "effects.torque_ripple = false",
                "effects.process_force = true",
                "process.force_c_n = -10.0",
            ],
        );
        let mut m = sim_netlist::build(&cfg).unwrap();
        m.set_dwelling(true);
        m
    };
    let torque_acc = |m: &dyn Machine, speed: f64, torque: f64| {
        let state = [1.0, speed, 0.0, 0.0];
        let mut out = [0.0; 4];
        m.derivatives(&state, &[torque], &mut out);
        (out[1], signal(m, &state, &[torque], "gearbox.1.output_torque"))
    };
    let acc = |m: &dyn Machine, speed: f64| torque_acc(m, speed, 0.0);
    let turning = machine(0.5, false);
    assert!(close(acc(turning.as_ref(), -10.0).0, -10.0 / (4.5 + 0.496 / 0.5), 1e-12));
    assert!(close(acc(turning.as_ref(), -10.0).0, -1.820_830, 1e-6));
    assert!(close(acc(turning.as_ref(), 0.0).0, -2.001_601, 1e-6));
    let locking = machine(-0.2, false);
    let near = |(a, o): (f64, f64), (by_hand_a, by_hand_o): (f64, f64), what: &str| {
        assert!((a - by_hand_a).abs() < 1e-6 && (o - by_hand_o).abs() < 1e-6, "{what}: {a} rad/s2, {o} N m");
    };
    near(acc(locking.as_ref(), 0.0), (0.0, 10.0), "held at rest");
    near(acc(locking.as_ref(), -10.0), (0.400_320, 11.801_441), "lowering, no torque");
    near(torque_acc(locking.as_ref(), -10.0, -0.05), (0.0, 10.0), "lowering, pushed");
    near(torque_acc(locking.as_ref(), -10.0, -0.1), (-0.400_320, 8.198_559), "lowering, pushed harder");
    for torque in [-0.04, 0.3] {
        assert!(torque_acc(locking.as_ref(), 0.0, torque).0.abs() < 1e-12, "stuck at {torque} N m");
    }
    assert!((torque_acc(locking.as_ref(), 0.0, -0.06).0 + 0.080_064).abs() < 1e-6);
    assert!((torque_acc(locking.as_ref(), 0.0, 0.35).0 - 0.300_240).abs() < 1e-6);
    // Elastic: [table, its gear output, rotor], their rates, the friction state, slack time, brake.
    let rotor = 40.0 * (1.0 + 10.0 / 67000.0);
    let state = [1.0, 1.0, rotor, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    for (back, rotor_acc) in [(0.5, -806.451_612_9), (-0.2, 0.0)] {
        let m = machine(back, true);
        let mut out = [0.0; 9];
        m.derivatives(&state, &[0.0], &mut out);
        assert!(out[3].abs() < 1e-9, "back {back}: table {}", out[3]);
        assert!((out[5] - rotor_acc).abs() < 1e-4, "back {back}: rotor {} against {rotor_acc}", out[5]);
    }
    // A worm on a gearbox that is not there, and a forward efficiency above 1, are refused by name.
    let refused = |netlist: String| {
        let mut cfg = kit_as_file("rotary-table", &netlist);
        cfg.set_text("effects.belt_stretch = false").unwrap();
        sim_netlist::build(&cfg).err()
    };
    assert_eq!(
        refused(worm_table("40", 0.8, 0.5).replace(r#"gearbox = "gear", ratio"#, r#"gearbox = "gaer", ratio"#)),
        Some(ConfigError::Invalid("machine.worm[worm].gearbox = \"gaer\"".into()))
    );
    assert_eq!(
        refused(worm_table("40", 1.2, 0.5)),
        Some(ConfigError::Invalid("machine.worm[worm].efficiency (above 0, at most 1)".into()))
    );
}

/// The catalogue search for the XYZ portal's Z motor alone: a 1FK2103-2AH (rotor 9.3e-6 kg m2, 1.18 kg without brake,
/// on its paired S210) becomes [parts.mz], X and Y keep the shared motor, and the candidate costs that one motor's
/// weight, 1.18 kg, not three. The Z motor's own velocity gain (0.01 N m s/rad) is retuned down for the lighter rotor.
#[test]
fn the_search_sizes_one_motor_of_a_machine_file() {
    let entry = file("xyz-portal.toml");
    let cfg = entry.config();
    let article = "1FK2103-2AH";
    let drive = paired_drive(article, "400v");
    let sized = with_motor_part(entry, &cfg, "mz", article, "400v", "", drive).unwrap();
    let rotor = |cfg: &Config, id: &str| motor_config(cfg, id).unwrap().f64("motor.rotor_inertia_kgm2").unwrap();
    assert_eq!(rotor(&sized, "mz"), 9.3e-6);
    assert_eq!((rotor(&sized, "mx"), rotor(&sized, "my")), (rotor(&cfg, "mx"), rotor(&cfg, "my")));
    let gain = motor_config(&sized, "mz").unwrap().f64("drive.velocity_kp_nm_s_rad").unwrap();
    assert!(gain > 0.0 && gain < 0.01, "{gain}");
    assert_eq!(motor_config(&sized, "mx").unwrap().f64("drive.velocity_kp_nm_s_rad").unwrap(), 0.04);

    let found = search_motor(entry, &cfg, "400v", &|a: &str| a == article, None, "mz").unwrap();
    assert_eq!(found.len(), 1);
    assert_eq!((found[0].article, found[0].drive, found[0].mass, found[0].cost), (article, drive, 1.18, 1.18));
    assert_eq!(motor_mass(article, "400v"), Some(1.18));
    assert!(found[0].result.is_some() || !found[0].refused.is_empty());
    assert!(search_motor(entry, &cfg, "400v", &|a: &str| a == article, None, "mq").is_err());
}
