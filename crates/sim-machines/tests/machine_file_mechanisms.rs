//! Machine files with mechanisms other than belts (crates/sim-netlist, docs/machine-files.md): ball screws, racks,
//! linear motors and rotary tables beside belts. Each against a hand calculation, the joint kit's screw axis written as
//! a file against its crate, the two shipped files through the standard suite, and broken files refused by key and line.

use sim_components::drive::DriveParams;
use sim_core::{Config, ConfigError, Machine, MachineEntry};
use sim_machines::{MACHINES, find, load_file};
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

/// A shipped file, rigid and without the losses a hand calculation leaves out: guide, motor and nut friction, ripple,
/// the belt's teeth. The gearbox's (and screw's, rack's) efficiency stays.
fn rigid(name: &str) -> Box<dyn Machine> {
    let entry = file(name);
    let mut cfg = entry.config();
    for off in ["belt_stretch", "guide_friction", "motor_friction", "torque_ripple", "tooth_meshing"] {
        cfg.set_text(&format!("effects.{off} = false")).unwrap();
    }
    (entry.build)(&cfg).unwrap()
}

/// The screw portal's Z by hand, rigid: the slide and payload, m = 2.5 + 1 = 3.5 kg, lifted at a = 2 m/s2 take
/// F = m (a + g) = 3.5 * 11.80665 = 41.3233 N from the nut. The screw 16 x 10 turns lever = lead / (2 pi) = 1.59155 mm
/// per radian, at 90 % efficiency; on the rotor side (coupling ratio 1) the rotor 1.5e-5, the coupling 5e-6 and the
/// shaft pi 7850 kg/m3 0.45 m 0.016^4 m^4 / 32 = 2.27280e-5 kg m2: J = 4.27280e-5 kg m2. Driving up,
/// T = J a / lever + F lever / eta = 0.0536936 + 0.0730756 = 0.1267692 N m gives exactly that acceleration; the
/// loss-free feedforward is 0.0536936 + 0.0657680 = 0.1194617 N m. The nut passes F to the slide, the coupling
/// F lever / eta; the screw turns at 0.3 m/s / 0.01 m = 30 r/s = 1800 r/min.
///
/// Its critical speed by hand (root diameter 13.2 mm, 0.45 m, fixed-supported, steel 206 GPa and 7850 kg/m3):
/// sqrt(E I / (rho A)) = d / 4 sqrt(E / rho) = 3.3e-3 * 5122.70 = 16.9049 m2/s, (3.926602 / 0.45 m)^2 = 76.1393 1/m2,
/// omega = 1287.13 rad/s = 12291.2 r/min; the check allows 0.8 of it, 9832.9 r/min.
#[test]
fn a_screw_axis_takes_the_hand_calculated_torque_and_has_its_critical_speed() {
    let machine = rigid("screw-portal.toml");
    let (lever, eta, mass, acc) = (0.01 / (2.0 * PI), 0.9, 3.5, 2.0);
    let shaft = PI * 7850.0 * 0.45 * 0.016f64.powi(4) / 32.0;
    let inertia = 1.5e-5 + 5.0e-6 + shaft;
    let force = mass * (acc + 9.80665);
    let torque = inertia * acc / lever + force * lever / eta;
    assert!(close(shaft, 2.27280e-5, 1e-5) && close(torque, 0.1267692, 1e-6), "{shaft} {torque}");

    let pose = [0.5, 0.25, 0.15];
    let (mut state, mut held) = (Vec::new(), [0.0; 3]);
    machine.equilibrium(&pose, &mut state, &mut held);
    // Held at rest against the weight alone: m g lever = 0.0546265 N m (no efficiency on what stands still).
    assert!(close(held[2], mass * 9.80665 * lever, 1e-12), "{held:?}");
    machine.set_tool_velocity(&mut state, &[0.0, 0.0, 0.3]);
    let torques = [held[0], held[1], torque];
    let mut rates = vec![0.0; state.len()];
    machine.derivatives(&state, &torques, &mut rates);
    assert!(close(rates[3 + 2], acc, 1e-9), "Z acceleration {} against {acc} by hand", rates[5]);
    assert!(close(signal(machine.as_ref(), &state, &torques, "joint.mz.force"), force, 1e-9));
    assert!(close(signal(machine.as_ref(), &state, &torques, "gearbox.mz.output_torque"), force * lever / eta, 1e-9));
    assert!(close(signal(machine.as_ref(), &state, &torques, "screw.mz.speed"), 1800.0, 1e-12));
    let mut feedforward = [0.0; 3];
    machine.feedforward_torque(&pose, &[0.0, 0.0, acc], &mut feedforward);
    assert!(close(feedforward[2], inertia * acc / lever + force * lever, 1e-12) && close(feedforward[2], 0.1194617, 1e-6), "{feedforward:?}");

    let critical = (3.926602f64 / 0.45).powi(2) * 0.0132 / 4.0 * (2.06e11f64 / 7850.0).sqrt() * 60.0 / (2.0 * PI);
    assert!(close(critical, 12291.16, 1e-6), "{critical}");
    let checks = machine.rating_checks();
    let speed = checks.iter().find(|c| c.check == "screw-speed").unwrap();
    assert!(speed.signal == "screw.mz.speed" && speed.motor == Some(2) && close(speed.limit, 0.8 * critical, 1e-6), "{speed:?}");
    let names: Vec<&str> = checks.iter().map(|c| c.check).collect();
    assert_eq!(names, ["screw-speed", "screw-buckling", "screw-static", "screw-dn", "screw-life"]);
}

/// The rack gantry's X by hand, rigid: the pinion (module 2, 30 teeth, 19.5283 degrees helix) has the pitch radius
/// r = 0.002 * 30 / (2 cos 19.5283 deg) = 31.8310 mm, 200 mm per turn; behind ratio 10 the motor turns 10 x / r: at
/// x = 1 m, 314.159 rad (five turns of the pinion, fifty of the motor). The bridge, Y carriage and payload,
/// m = 12 + 4 + 1 = 17 kg, at a = 2 m/s2 take F = 34 N from the rack; the pinion's teeth then carry F / 0.97 =
/// 35.0515 N while driving. The motor's torque for it, i / r = 314.159 rad/m, the gearbox 95 % and the rack 97 %, the
/// rotor 3e-5 and the pinion 1e-4 / 10^2 kg m2 on the rotor side: T = 3.1e-5 * 2 * 314.159 + 34 / (314.159 * 0.95 *
/// 0.97) = 0.0194778 + 0.1174449 = 0.1369228 N m.
#[test]
fn a_rack_axis_turns_its_motor_by_x_over_r_and_feeds_its_force() {
    let machine = rigid("rack-gantry.toml");
    let r = 0.002 * 30.0 / (2.0 * 19.5283f64.to_radians().cos());
    assert!(close(r, 0.0318310, 1e-6));
    let mut motors = [0.0; 2];
    machine.inverse(&[1.0, 0.0], &mut motors);
    assert!(close(motors[0], 10.0 * 1.0 / r, 1e-12) && close(motors[0], 314.159, 1e-6), "{motors:?}");

    let (ratio, mass, acc) = (10.0 / r, 17.0, 2.0);
    let torque = 3.1e-5 * acc * ratio + mass * acc / (ratio * 0.95 * 0.97);
    assert!(close(torque, 0.1369228, 1e-6), "{torque}");
    let (mut state, mut held) = (Vec::new(), [0.0; 2]);
    machine.equilibrium(&[1.0, 0.25], &mut state, &mut held);
    machine.set_tool_velocity(&mut state, &[0.3, 0.0]);
    let torques = [torque, held[1]];
    let mut rates = vec![0.0; state.len()];
    machine.derivatives(&state, &torques, &mut rates);
    assert!(close(rates[2], acc, 1e-9), "X acceleration {} against {acc} by hand", rates[2]);
    assert!(close(signal(machine.as_ref(), &state, &torques, "joint.mx.force"), 34.0, 1e-9));
    assert!(close(signal(machine.as_ref(), &state, &torques, "rack.mx.feed_force"), 34.0 / 0.97, 1e-9));
    let check = machine.rating_checks();
    assert!(check.len() == 1 && check[0].check == "rack-feed-force" && check[0].signal == "rack.mx.feed_force" && check[0].limit == 2400.0, "{check:?}");
}

/// A joint kit machine written as a machine file: its default config, the tables a machine file reads that the kit has
/// not taken from machines/single-axis.toml (the belt, hard stops, a guide's friction, the belt effects' switches), and
/// `netlist` as its [machine] table.
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
    let mut single: toml::Table = std::fs::read_to_string(root().join("machines/single-axis.toml")).unwrap().parse().unwrap();
    single.remove("machine");
    fill(&mut kit, &single);
    kit.insert("machine".into(), toml::Value::Table(netlist.parse().unwrap()));
    Config::parse(&toml::to_string(&kit).unwrap()).unwrap()
}

/// The joint kit's screw axis as a netlist: its motor "1" (so that its signals are named as the kit's joint 1), a
/// coupling, the screw of [screw], the carriage on the axis; stroke and incline are the kit's [slide] values.
const SCREW_NETLIST: &str = r#"
version = 1
id = "screw-axis-file"
axis = [{ id = "x", stroke_m = "slide.stroke_m", incline_deg = "slide.incline_deg" }]
motor = [{ id = "1" }]
gearbox = [{ id = "coupling", motor = "1" }]
screw = [{ id = "screw_x", gearbox = "coupling", axis = "x", values = "screw" }]
body = [{ id = "carriage", mass_kg = "mass.carriage_kg", payload_kg = "mass.payload_kg", axes = ["x"] }]
"#;

/// The kit's screw axis and its file, with the same changes. The kit's joint friction and the file's guide friction
/// (which would hold the nut's preload drag) are off: one is Coulomb at the joint, the other LuGre in a guide. The
/// file's hard stops are the kit's slide's. Vertical, so that gravity counts. `elastic` sets the kit's
/// joint_elasticity and the file's belt_stretch.
fn kit_screw(elastic: bool) -> [Box<dyn Machine>; 2] {
    let mut kit = find("screw-axis").unwrap().config();
    let mut from_file = kit_as_file("screw-axis", SCREW_NETLIST);
    for cfg in [&mut kit, &mut from_file] {
        for change in ["effects.joint_friction = false", "slide.incline_deg = 90.0", &format!("effects.joint_elasticity = {elastic}")] {
            cfg.set_text(change).unwrap();
        }
    }
    for change in ["effects.guide_friction = false", "hard_stop.stiffness_n_m = 2.0e6", "hard_stop.damping_n_s_m = 2000.0", &format!("effects.belt_stretch = {elastic}")] {
        from_file.set_text(change).unwrap();
    }
    [(find("screw-axis").unwrap().build)(&kit).unwrap(), sim_netlist::build(&from_file).unwrap()]
}

/// The joint kit's screw axis (crates/machine-joint-kit, its hand checks there and in transmission.rs) written as a
/// machine file is the same machine, elastic and rigid, with the payload on, set down and with the brake holding: axes,
/// kinematics, the rest state and holding torque, the derivatives of a moving state under torques, energy, feedforward,
/// the screw's signals and the torque breakdown, and the screw's checks, to 1e-9. The states are laid out apart: the
/// kit's elastic [q, q', rotor, rotor', brake] are the file's [q, a, rotor, q', a', rotor', friction, slack, brake] with
/// its pulley a = q; rigid [q, q', brake] are [q, q', friction, brake]. Before writing this, `sim-cli verdicts` of the
/// kit and the file gave the same digits for the screw, rack, linear motor and rotary table axes (encoder counts off).
#[test]
fn the_joint_kits_screw_axis_as_a_machine_file_is_its_crate() {
    for elastic in [true, false] {
        let [mut kit, mut from_file] = kit_screw(elastic);
        let what = format!("elastic {elastic}");
        let axes = |m: &dyn Machine| m.axes().iter().map(|a| (a.id, a.unit, a.min, a.max, a.max_vel)).collect::<Vec<_>>();
        assert_eq!(axes(kit.as_ref()), axes(from_file.as_ref()), "{what}");
        let file_state = |s: &[f64]| if elastic { vec![s[0], s[0], s[2], s[1], s[1], s[3], 0.0, 0.0, s[4]] } else { vec![s[0], s[1], 0.0, s[2]] };
        // (kit, file) indices of the same rate: q', q'' (and rotor', rotor''), brake.
        let rates: &[(usize, usize)] = if elastic { &[(0, 0), (1, 3), (2, 2), (3, 5), (4, 8)] } else { &[(0, 0), (1, 1), (2, 3)] };
        assert_eq!(from_file.state_len(), file_state(&vec![0.0; kit.state_len()]).len(), "{what}");
        for (k, pose) in [0.1, 0.62, 0.4].into_iter().enumerate() {
            for m in [&mut kit, &mut from_file] {
                m.set_payload_attached(k != 1);
                m.set_brake_engaged(k == 2);
            }
            let what = format!("{what}, pose {pose}");
            let (mut a, mut b) = ([0.0], [0.0]);
            kit.inverse(&[pose], &mut a);
            from_file.inverse(&[pose], &mut b);
            assert!(close(a[0], b[0], 1e-12), "{what}: inverse {a:?} {b:?}");
            let (mut sa, mut sb, mut ta, mut tb) = (Vec::new(), Vec::new(), [0.0], [0.0]);
            kit.equilibrium(&[pose], &mut sa, &mut ta);
            from_file.equilibrium(&[pose], &mut sb, &mut tb);
            assert!(close(ta[0], tb[0], 1e-9), "{what}: holding torque {ta:?} {tb:?}");
            assert!(file_state(&sa).iter().zip(&sb).all(|(x, y)| close(*x, *y, 1e-9)), "{what}: rest state {sa:?} {sb:?}");

            kit.set_tool_velocity(&mut sa, &[0.4]);
            // Off rest: every state moved a little (the coupling twisted, the brake slipped).
            for (i, x) in sa.iter_mut().enumerate() {
                *x += 1e-5 * (1.0 + i as f64);
            }
            let sb = file_state(&sa);
            let torque = [ta[0] + 0.7];
            let (mut da, mut db) = (vec![0.0; sa.len()], vec![0.0; sb.len()]);
            kit.derivatives(&sa, &torque, &mut da);
            from_file.derivatives(&sb, &torque, &mut db);
            for &(i, j) in rates {
                assert!(close(da[i], db[j], 1e-9), "{what}: rate {i}: crate {}, file {}", da[i], db[j]);
            }
            assert!(close(kit.energy(&sa), from_file.energy(&sb), 1e-9), "{what}: energy");
            let (mut fa, mut fb) = ([0.0], [0.0]);
            kit.feedforward_torque(&[pose], &[3.0], &mut fa);
            from_file.feedforward_torque(&[pose], &[3.0], &mut fb);
            assert!(close(fa[0], fb[0], 1e-9), "{what}: feedforward {fa:?} {fb:?}");
            for id in ["joint.1.force", "gearbox.1.output_torque", "screw.1.speed", "screw.1.dn", "screw.1.damage_rate", "motor.1.torque_inertia", "motor.1.torque_gravity", "motor.1.torque_friction", "motor.1.torque_rest", "power.shaft_1", "power.gearbox_1", "energy.kinetic", "energy.potential", "energy.elastic"] {
                let (x, y) = (signal(kit.as_ref(), &sa, &torque, id), signal(from_file.as_ref(), &sb, &torque, id));
                assert!(close(x, y, 1e-9), "{what}: {id}: crate {x}, file {y}");
            }
        }
        let checks = |m: &dyn Machine| m.rating_checks().into_iter().map(|c| (c.check, c.signal, c.motor, c.limit, c.life)).collect::<Vec<_>>();
        assert_eq!(checks(kit.as_ref()), checks(from_file.as_ref()), "{what}");
    }
}

/// The joint kit's linear motor axis as a netlist: the motor "1" is the linear motor, with no gearbox node; a guide
/// carries the carriage, so that the attraction shows in its load.
const LINEAR_NETLIST: &str = r#"
version = 1
id = "linear-motor-axis-file"
axis = [{ id = "x", stroke_m = "slide.stroke_m" }]
motor = [{ id = "1" }]
linear_motor = [{ id = "forcer", motor = "1", axis = "x", values = "linear_motor" }]
body = [{ id = "carriage", mass_kg = "mass.carriage_kg", payload_kg = "mass.payload_kg", axes = ["x"] }]
guide = [{ id = "guide_x", axis = "x", friction = "friction_x", carries = ["carriage"] }]
"#;

/// The joint kit's linear motor axis as a file, by hand: carriage 5 kg, payload 5 kg and the forcer's 3 kg (its
/// motor.rotor_inertia_kgm2) at 10 m/s2 take F = 13 * 10 = 130 N, which at the force constant of 65 N/A (saturation
/// off) is 2 A. The guide carries the carriage's and payload's weight, the forcer's and the attraction between forcer
/// and magnets: 2500 + 13 * 9.80665 = 2627.486 N (the kit's `axis.x.guide_load`). No transmission: the "motor angle" is
/// the travel.
#[test]
fn a_linear_motor_axis_draws_the_hand_calculated_current_for_its_force() {
    let mut cfg = kit_as_file("linear-motor-axis", LINEAR_NETLIST);
    cfg.set_text("effects.belt_stretch = false").unwrap();
    cfg.set_text("effects.torque_saturation = false").unwrap();
    let machine = sim_netlist::build(&cfg).unwrap();
    let mut angle = [0.0];
    machine.inverse(&[0.5], &mut angle);
    assert_eq!(angle, [0.5]);
    let mut force = [0.0];
    machine.feedforward_torque(&[0.5], &[10.0], &mut force);
    assert!(close(force[0], 130.0, 1e-12), "{force:?}");
    let drive = DriveParams::from_config(&cfg, cfg.f64("sim.dt_s").unwrap(), cfg.f64("drive.velocity_cycle_s").unwrap()).unwrap();
    assert!(close(drive.current_for(force[0]), 2.0, 1e-12), "{}", drive.current_for(force[0]));
    let (mut state, mut held) = (Vec::new(), [0.0]);
    machine.equilibrium(&[0.5], &mut state, &mut held);
    assert!(close(signal(machine.as_ref(), &state, &held, "axis.x.guide_load"), 2627.48645, 1e-9));
    assert!(machine.rating_checks().is_empty());
}

/// A rotary table as a netlist: the joint kit's table and strain wave gear, its axis in rad.
const TABLE_NETLIST: &str = r#"
version = 1
id = "rotary-table-file"
axis = [{ id = "c", stroke_rad = 4.0 }]
motor = [{ id = "1" }]
gearbox = [{ id = "gear", motor = "1" }]
rotary = [{ id = "table", gearbox = "gear", axis = "c", inertia_kgm2 = "table.inertia_kgm2", payload_inertia_kgm2 = "table.payload_inertia_kgm2", strain_wave = "strain_wave" }]
"#;

/// The joint kit's rotary table as a file, by hand: its axis is in rad (limits axis.c.max_velocity_rad_s), the table
/// 3 kg m2 and its payload 1.5 kg m2 behind the strain wave gear of ratio 100 with the rotor 1.7e-4 + 1.4e-4 kg m2: at
/// 10 rad/s2 the loss-free motor torque is (4.5 * 10) / 100 + 3.1e-4 * 100 * 10 = 0.45 + 0.31 = 0.76 N m, the motor
/// 100 rad per rad. Its strain wave gear is judged for its life against ratings.required_life_h.
#[test]
fn a_rotary_axis_turns_in_rad_with_its_strain_wave_gear() {
    let mut cfg = kit_as_file("rotary-table", TABLE_NETLIST);
    cfg.set_text("effects.belt_stretch = false").unwrap();
    let machine = sim_netlist::build(&cfg).unwrap();
    assert_eq!((machine.axes()[0].unit, machine.axes()[0].max, machine.axes()[0].max_vel), ("rad", 4.0, 3.0));
    let mut angle = [0.0];
    machine.inverse(&[1.0], &mut angle);
    assert!(close(angle[0], 100.0, 1e-12));
    let mut torque = [0.0];
    machine.feedforward_torque(&[1.0], &[10.0], &mut torque);
    assert!(close(torque[0], 0.76, 1e-12), "{torque:?}");
    let checks = machine.rating_checks();
    assert!(checks.len() == 1 && checks[0].check == "strain-wave-life" && checks[0].signal == "strain_wave.1.damage_rate" && checks[0].limit == 20000.0, "{checks:?}");
}

/// Both shipped files pass the standard suite of every machine (round trip, static hold, determinism, energy
/// conservation without losses, the loss budget with them), without losses, elastic and rigid. Their X and Y lie flat:
/// backlash off, as for the portal they come from (machine_files.rs says why).
#[test]
fn the_mechanism_files_pass_the_standard_suite() {
    use std::sync::Mutex;
    static LOSSLESS: Mutex<Vec<Config>> = Mutex::new(Vec::new());
    fn lossless() -> Vec<Config> {
        LOSSLESS.lock().unwrap().clone()
    }
    for name in ["screw-portal.toml", "rack-gantry.toml"] {
        let shipped = file(name);
        let mut configs = Vec::new();
        for elastic in [true, false] {
            let mut cfg = shipped.config();
            for effect in ["belt_damping", "guide_friction", "load_dependent_friction", "motor_friction", "gearbox_efficiency", "torque_ripple", "tooth_meshing", "temperature", "holding_brake", "gearbox_backlash"] {
                cfg.set_text(&format!("effects.{effect} = false")).unwrap();
            }
            for damping in ["gearbox.damping_nm_s_rad = 0.0", "hard_stop.damping_n_s_m = 0.0"] {
                cfg.set_text(damping).unwrap();
            }
            if name == "screw-portal.toml" {
                cfg.set_text("gearbox.mz.damping_nm_s_rad = 0.0").unwrap();
            }
            cfg.set_text(&format!("effects.belt_stretch = {elastic}")).unwrap();
            configs.push(cfg);
        }
        *LOSSLESS.lock().unwrap() = configs;
        let entry = MachineEntry { suite_configs: lossless, ..*shipped };
        sim_core::testkit::run_standard_suite(&entry);
    }
}

fn line_with(text: &str, needle: &str) -> usize {
    text.lines().position(|l| l.contains(needle)).unwrap() + 1
}

/// A broken screw portal, refused by key and line: a field the screw node does not have, a misspelt key in its value
/// table, a screw shorter than its axis's stroke, and a gearbox that turns both a pulley and a screw.
#[test]
fn a_broken_mechanism_is_refused_with_its_key_and_line() {
    let text = std::fs::read_to_string(root().join("machines/screw-portal.toml")).unwrap().replace("\r\n", "\n");
    let known: Vec<Config> = MACHINES.iter().map(|m| m.config()).collect();
    let refused = |from: &str, to: &str| {
        let broken = text.replacen(from, to, 1);
        assert_ne!(broken, text, "the change applies");
        sim_netlist::from_text(&broken, true, "broken", &known).err().expect("a broken file loaded")
    };
    let (e, line) = refused("values = \"screw_z\"", "valeus = \"screw_z\"");
    assert_eq!((e, line), (ConfigError::Unknown("machine.screw[screw_z].valeus".into()), Some(line_with(&text, "values = \"screw_z\""))));
    let (e, line) = refused("lead_m = 0.01", "lead_mm = 10.0");
    assert_eq!((e, line), (ConfigError::Unknown("screw_z.lead_mm".into()), Some(line_with(&text, "lead_m = 0.01"))));
    let (e, line) = refused("length_m = 0.45", "length_m = 0.25");
    assert_eq!((e, line), (ConfigError::Invalid("screw_z.length_m (shorter than the stroke of axis z)".into()), Some(line_with(&text, "length_m = 0.45"))));
    // The screw on the Y gearbox, which turns Y's pulley: the motors are read in order, so Y's gearbox is named first.
    let (e, line) = refused("gearbox = \"gearbox_z\"\naxis = \"z\"", "gearbox = \"gearbox_y\"\naxis = \"z\"");
    assert_eq!((e, line), (ConfigError::Invalid("machine.gearbox[gearbox_y] (one pulley, screw, rack or rotary per gearbox)".into()), Some(line_with(&text, "id = \"gearbox_y\""))));
}
