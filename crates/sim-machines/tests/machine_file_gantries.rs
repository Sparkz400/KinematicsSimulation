//! Machine files with a yawing gantry bridge and with tools on rotary axes (crates/sim-netlist, `machine.gantry` and
//! `machine.tcp`, docs/machine-files.md): the dual-drive gantry's load share and yaw frequency, the H-bot's racking
//! against the CoreXY's, the tool centre point of the C portal and the five-axis head, each against a hand calculation;
//! the new files through the standard suite, and broken files refused.

use sim_core::{Config, ConfigError, Machine, MachineEntry, Shape};
use sim_machines::{MACHINES, load_file};
use sim_netlist::{ConfigMachine, Netlist};
use std::f64::consts::TAU;
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

/// A shipped file's config with changes.
fn config(name: &str, changes: &[&str]) -> Config {
    let mut cfg = file(name).config();
    for change in changes {
        cfg.set_text(change).unwrap();
    }
    cfg
}

fn machine(cfg: &Config) -> ConfigMachine {
    ConfigMachine::new(cfg, &Netlist::parse(cfg).unwrap()).unwrap()
}

/// The pinion of the racks (module 2, 30 teeth, 19.5283 degrees helix): pitch radius 31.8310 mm.
fn pinion_radius() -> f64 {
    0.002 * 30.0 / (2.0 * 19.5283f64.to_radians().cos())
}

/// The dual-drive gantry's load share by hand. Its bridge is driven at both ends, 0.6 m either side of its centre
/// (B = 1.2 m), each rack behind ratio 10: a motor turns i / r = 10 / 0.0318310 = 314.159 rad per metre of its end.
/// Accelerating along X at a = 2 m/s2 with the bridge kept square, the two ends' forces F1 + F2 carry everything that
/// moves along X, and their moment (B / 2) (F2 - F1) turns the Y carriage (4 kg and the 1 kg payload) round the bridge's
/// centre at its lever e = y - 0.4 m (Y's 0.8 m stroke in the middle of the bridge): F2 - F1 = 2 m e a / B, the lever
/// rule (the nearer end takes more). At y = 0, 0.4 and 0.8 m, e = -0.4, 0 and 0.4 m: the torques differ by
/// 2 * 5 * e * 2 / (1.2 * 314.159) = -0.0212207, 0 and 0.0212207 N m, and their sum stays (the rotors and pinions turn
/// alike at every y). With effects.gantry_yaw off the bridge is held square by its guides and the two motors share
/// equally.
#[test]
fn the_dual_gantry_shares_the_bridge_by_the_lever_rule() {
    let ratio = 10.0 / pinion_radius();
    assert!(close(ratio, 314.159, 1e-5));
    for model in ["effects.belt_stretch = true", "effects.belt_stretch = false"] {
        let gantry = machine(&config("dual-gantry.toml", &[model]));
        let mut sums = Vec::new();
        for (y, e) in [(0.0, -0.4), (0.4, 0.0), (0.8, 0.4)] {
            let mut torque = [0.0; 3];
            gantry.feedforward_torque(&[1.5, y], &[2.0, 0.0], &mut torque);
            let hand = 2.0 * 5.0 * e * 2.0 / (1.2 * ratio);
            assert!(close(torque[1] - torque[0], hand, 1e-9), "{model} at y = {y}: {torque:?} against {hand}");
            assert!(torque[0] > 0.0 && torque[1] > 0.0, "{torque:?}");
            sums.push(torque[0] + torque[1]);
            // Two drives set the yaw: nothing is left for the bridge's joints to hold.
            assert!(gantry.racking_moment(&[1.5, y], &[2.0, 0.0]).unwrap().abs() < 1e-9);
        }
        assert!(close(sums[0], sums[1], 1e-12) && close(sums[1], sums[2], 1e-12), "{sums:?}");
    }
    let square = machine(&config("dual-gantry.toml", &["effects.gantry_yaw = false"]));
    for y in [0.0, 0.8] {
        let mut torque = [0.0; 3];
        square.feedforward_torque(&[1.5, y], &[2.0, 0.0], &mut torque);
        assert!(torque[0] > 0.0 && close(torque[0], torque[1], 1e-12), "{torque:?}");
        assert!(square.racking_moment(&[1.5, y], &[2.0, 0.0]).is_none());
    }
}

/// The dual-drive gantry's yaw frequency by hand. Each end's rack holds its end with the gearbox ([gearbox.mx1]:
/// 35752.6 N m/rad at the pinion, r^2 / k = 0.0318310^2 / 35752.6 = 2.83394e-8 m/N) and the rack (1e8 N/m) in series:
/// k_end = 1 / (2.83394e-8 + 1e-8) = 2.60829e7 N/m. Yawing by psi moves the ends by -/+ 0.6 psi, so the drives hold
/// the yaw with 2 k_end 0.6^2 = 1.87797e7 N m/rad, the bridge's joints with 2e5 more. With the carriage in the middle
/// (no lever, so the yaw does not move along X) and the bridge's inertia raised to 20 kg m2 (and the Y belt stiffened)
/// so that the yaw is the lowest mode: f = sqrt(1.89797e7 / 20) / 2 pi = 155.04 Hz.
#[test]
fn the_dual_gantry_yaws_at_the_frequency_of_its_inertia_and_its_ends() {
    let r = pinion_radius();
    let end = 1.0 / (r * r / 35752.6 + 1.0 / 1e8);
    assert!(close(end, 2.60829e7, 1e-5), "{end}");
    let stiffness = 2.0 * end * 0.36 + 2.0e5;
    let hand = (stiffness / 20.0).sqrt() / TAU;
    assert!(close(hand, 155.04, 1e-4), "{hand}");
    let cfg = config("dual-gantry.toml", &["gantry.inertia_kgm2 = 20.0", "belt.stiffness_ea_n = 5.0e8"]);
    let hz = machine(&cfg).resonance_hz(&[1.5, 0.4]).unwrap();
    assert!(close(hz, hand, 1e-9), "{hz} Hz against {hand} by hand");
    // Off, the bridge has no yaw and its lowest mode is another (higher) one.
    let mut square = cfg.clone();
    square.set_text("effects.gantry_yaw = false").unwrap();
    assert!(machine(&square).resonance_hz(&[1.5, 0.4]).unwrap() > hand * 1.2);
}

/// The H-bot's racking by hand (machines/h-bot.toml with effects.gantry_yaw on). Its belt pulls the bridge's ends along
/// x: per rad of yaw section 1 gets longer by -0.3 m, section 2 by 0.6 m and section 3 by -0.3 m, so the belt's moment
/// on the bridge is -sum(c T) = 0.3 (T1 - 2 T2 + T3), and the head's pull along y is -(T1 - 2 T2 + T3). Accelerating
/// the head (5 kg and the 2 kg payload) along y at 2 m/s2 takes 14 N from the belt: the bridge racks with
/// -0.3 * 14 = -4.2 N m, wherever the head is. Accelerating along x at 2 m/s2 with the head off the bridge's centre
/// (y = 0.4 m: e = 0.4 - 0.25 = 0.15 m, the stroke of 0.5 m in the middle) the belt pulls both ends alike, and the
/// head's inertia turns the bridge: -7 * 0.15 * 2 = -2.1 N m, +2.1 at y = 0.1 m; none in the middle. Rigid and
/// elastic alike: the rigid belt lets the bridge rack, as the head can move along y with the motors held.
#[test]
fn the_h_bot_racks_with_the_head_s_pull_and_off_centre() {
    for model in ["effects.belt_stretch = true", "effects.belt_stretch = false"] {
        let hbot = machine(&config("h-bot.toml", &["effects.gantry_yaw = true", model]));
        for y in [0.1, 0.25, 0.4] {
            let along = hbot.racking_moment(&[0.5, y], &[0.0, 2.0]).unwrap();
            assert!(close(along, -4.2, 1e-9), "{model}: along y at y = {y}: {along}");
        }
        for (y, hand) in [(0.4, -2.1), (0.25, 0.0), (0.1, 2.1)] {
            let across = hbot.racking_moment(&[0.5, y], &[2.0, 0.0]).unwrap();
            assert!(close(across, hand, 1e-9), "{model}: along x at y = {y}: {across} against {hand}");
        }
    }
    assert!(machine(&config("h-bot.toml", &[])).racking_moment(&[0.5, 0.25], &[0.0, 2.0]).is_none());
}

/// The CoreXY's cancellation by hand (machines/corexy.toml with effects.gantry_yaw on). Each belt pulls both ends of
/// the gantry: the moment is 0.3 m (T_a1 + T_a2 - T_b1 - T_b2), each belt's tension sum against the other's. The head
/// (3 kg and the 1 kg payload) accelerated along x at 2 m/s2 in the middle (x = y = 0.25 m) takes 8 N: each belt's two
/// sections differ by 4 N, and each closed belt keeps its length, so belt B (1.0 and 1.0 m) moves -2 and +2 N and
/// belt A (0.8 and 1.1 m) -4 * 1.1 / 1.9 and +4 * 0.8 / 1.9 N, a sum of -4 * 0.3 / 1.9 = -0.631579 N: the moment is
/// 0.3 * -0.631579 = -0.189474 N m, against -0.3 * 8 = -2.4 N m for the H-bot's way of routing (a twelfth). The rigid
/// belts hold the yaw: the rigid model has none.
#[test]
fn the_corexy_cancels_what_racks_the_h_bot() {
    let corexy = machine(&config("corexy.toml", &["effects.gantry_yaw = true"]));
    let moment = corexy.racking_moment(&[0.25, 0.25], &[2.0, 0.0]).unwrap();
    let hand = 0.3 * (-4.0 * 0.3 / 1.9);
    assert!(close(moment, hand, 1e-9) && close(hand, -0.189474, 1e-5), "{moment} against {hand}");
    assert!(moment.abs() < 0.1 * 0.3 * 8.0);
    let rigid = machine(&config("corexy.toml", &["effects.gantry_yaw = true", "effects.belt_stretch = false"]));
    assert!(rigid.racking_moment(&[0.25, 0.25], &[2.0, 0.0]).is_none());
}

/// The C portal's tool centre point by hand. The gripper's tip is tool_m = [0.1, 0, -0.15] from the C axis on the Z
/// slide, turned by C about the vertical: the slide stands at x = tip_x - 0.1 cos C, y = tip_y - 0.1 sin C,
/// z = tip_z + 0.15. Along the cycle's carry, the tip from (0.3, 0.2, 0.05) to (0.7, 0.3, 0.05) while C turns from 0.5
/// to 2.0708 rad, X and Y make up for C: the motors stand at 250 rad per metre of X and Y (belt, i 5, r 0.02 m),
/// 2 pi / 0.01 = 628.319 per metre of Z (screw, lead 10 mm, coupling) and 20 per rad of C (ratio 20). The 3D view's
/// tool point is the tip, on the straight line, and its tool part ends there.
#[test]
fn the_c_portal_keeps_the_tip_on_its_line_while_c_turns() {
    let entry = file("portal-c.toml");
    let portal = (entry.build)(&entry.config()).unwrap();
    for s in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let (x, y, z, c) = (0.3 + 0.4 * s, 0.2 + 0.1 * s, 0.05, 0.5 + (2.0708 - 0.5) * s);
        let mut motors = [0.0; 4];
        portal.inverse(&[x, y, z, c], &mut motors);
        let hand = [250.0 * (x - 0.1 * c.cos()), 250.0 * (y - 0.1 * c.sin()), (z + 0.15) * TAU / 0.01, 20.0 * c];
        for k in 0..4 {
            assert!(close(motors[k], hand[k], 1e-12), "s = {s}: {motors:?} against {hand:?}");
        }
        let mut back = [0.0; 4];
        portal.forward(&motors, &mut back);
        assert!(back.iter().zip([x, y, z, c]).all(|(a, b)| close(*a, b, 1e-12)), "{back:?}");
        let point = portal.scene_point(&[x, y, z, c]).unwrap();
        assert!(close(point[0], x, 1e-12) && close(point[1], y - 0.03, 1e-12) && close(point[2], z + 0.06, 1e-12));
        // The 3D view at rest there: the tool part from the slide to the tip.
        let (mut state, mut torque) = (Vec::new(), [0.0; 4]);
        portal.equilibrium(&[x, y, z, c], &mut state, &mut torque);
        let mut scene = sim_core::Scene::default();
        portal.scene(&state, &mut scene);
        let tool = scene.parts.iter().find(|p| p.id == "tool").expect("a tool part");
        let Shape::Segment { to, .. } = tool.shape else { panic!("{:?}", tool.shape) };
        assert!(to.iter().zip(point).all(|(a, b)| close(*a, b, 1e-12)), "{to:?} against {point:?}");
        // Beyond X's stroke the slide cannot follow: the tip at x = 0 with C at 0.5 asks for x = -0.088.
        assert_eq!(portal.unreachable(&[0.0, y, z, 0.5], 1e-4), Some("error-tcp-joint-range"));
    }
}

/// The five-axis head by hand. The tool's tip is 0.2 m below where A (about x) and C (about the vertical) meet:
/// turned, it is (-0.2 sin A sin C, 0.2 sin A cos C, -0.2 cos A) from there, so for the tip held at (0.5, 0.25, -0.05)
/// the axes stand at x = 0.5 + 0.2 sin A sin C, y = 0.25 - 0.2 sin A cos C, z = -0.05 + 0.2 cos A.
///
/// The torque of that compensating motion: with A at 0.6 rad and C turning at 2 rad/s round the still tip, the axes run
/// on a circle of radius 0.2 sin A, at C = 0 accelerating along y by 0.2 sin A * 4 m/s2. The spindle (4 kg and the 1 kg
/// payload) rides on C 0.12 m from the pivot, on the circle's far side: it accelerates by only (0.2 - 0.12) sin A * 4,
/// while the bridge, the carriages and the head go round with the axes. So the Y motor needs 5 * 0.12 sin A * 4 /
/// 250 = 0.0054197 N m less than for the same acceleration of Y with the head standing still, and X and Z the same.
#[test]
fn the_five_axis_head_holds_its_tip_while_a_and_c_turn() {
    let entry = file("five-axis.toml");
    let head = (entry.build)(&entry.config()).unwrap();
    for (a, c) in [(0.1, 0.5), (0.6, 0.5), (0.6, 2.0), (0.9, 4.0)] {
        let mut motors = [0.0; 5];
        head.inverse(&[0.5, 0.25, -0.05, a, c], &mut motors);
        let (x, y, z) = (0.5 + 0.2 * a.sin() * c.sin(), 0.25 - 0.2 * a.sin() * c.cos(), -0.05 + 0.2 * a.cos());
        let hand = [250.0 * x, 250.0 * y, z * TAU / 0.01, 50.0 * a, 50.0 * c];
        for k in 0..5 {
            assert!(close(motors[k], hand[k], 1e-12), "A {a}, C {c}: {motors:?} against {hand:?}");
        }
    }
    let (a, omega) = (0.6f64, 2.0);
    let (mut turning, mut sliding) = ([0.0; 5], [0.0; 5]);
    let tip = [0.5, 0.25, -0.05, a, 0.0];
    head.feedforward_torque_moving(&tip, &[0.0, 0.0, 0.0, 0.0, omega], &[0.0; 5], &mut turning);
    head.feedforward_torque_moving(&tip, &[0.0; 5], &[0.0, 0.2 * a.sin() * omega * omega, 0.0, 0.0, 0.0], &mut sliding);
    let hand = -5.0 * 0.12 * a.sin() * omega * omega / 250.0;
    assert!(close(hand, -0.0054197, 1e-4), "{hand}");
    assert!(close(turning[1] - sliding[1], hand, 1e-6), "{turning:?} against {sliding:?}: {hand}");
    assert!(close(turning[0], sliding[0], 1e-6) && close(turning[2], sliding[2], 1e-6), "{turning:?} {sliding:?}");
    // Tilting A lifts the spindle: held at A = 0.6 rad, A's motor carries 5 kg * g * 0.12 m * sin A / 50 (rigid,
    // without the ripple and the teeth).
    let mut cfg = entry.config();
    for change in ["effects.belt_stretch = false", "effects.torque_ripple = false", "effects.tooth_meshing = false"] {
        cfg.set_text(change).unwrap();
    }
    let rigid = (entry.build)(&cfg).unwrap();
    let (mut state, mut held) = (Vec::new(), [0.0; 5]);
    rigid.equilibrium(&tip, &mut state, &mut held);
    let weight = 5.0 * 9.80665 * 0.12 * a.sin() / 50.0;
    assert!(close(held[3], weight, 1e-6), "{held:?} against {weight}");
}

/// The files of gantries and rotary tools pass the standard suite of every machine (round trip, static hold,
/// determinism, energy conservation without losses, the loss budget with them), elastic and rigid: the new ones as
/// shipped, and the H-bot, CoreXY and rack gantry with their yaw on. X and Y lie flat: backlash off (machine_files.rs
/// says why); the yaw's and the motors' own gearboxes' damping off with the rest.
#[test]
fn the_gantry_and_rotary_files_pass_the_standard_suite() {
    use std::sync::Mutex;
    static LOSSLESS: Mutex<Vec<Config>> = Mutex::new(Vec::new());
    fn lossless() -> Vec<Config> {
        LOSSLESS.lock().unwrap().clone()
    }
    for (name, own) in [
        ("dual-gantry.toml", &["mx1", "mx2"][..]),
        ("portal-c.toml", &["mz", "mc"][..]),
        ("five-axis.toml", &["mz", "mc", "ma"][..]),
        ("h-bot.toml", &[][..]),
        ("corexy.toml", &[][..]),
        ("rack-gantry.toml", &["mx"][..]),
    ] {
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
                "gearbox_backlash",
            ] {
                cfg.set_text(&format!("effects.{effect} = false")).unwrap();
            }
            for damping in ["gearbox.damping_nm_s_rad = 0.0", "hard_stop.damping_n_s_m = 0.0"] {
                cfg.set_text(damping).unwrap();
            }
            for motor in own {
                cfg.set_text(&format!("gearbox.{motor}.damping_nm_s_rad = 0.0")).unwrap();
            }
            if cfg.get("gantry").is_ok() {
                cfg.set_text("effects.gantry_yaw = true").unwrap();
                cfg.set_text("gantry.damping_nm_s_rad = 0.0").unwrap();
            }
            cfg.set_text(&format!("effects.belt_stretch = {elastic}")).unwrap();
            configs.push(cfg);
        }
        *LOSSLESS.lock().unwrap() = configs;
        let entry = MachineEntry { suite_configs: lossless, ..*shipped };
        sim_core::testkit::run_standard_suite(&entry);
    }
}

/// Broken gantries and tools are refused by key: a second rack on X that no gantry names, a body's offset off the
/// rotary axes, a tool centre point whose axes are not x, y and z in that order, and a gantry drive that drives another
/// axis.
#[test]
fn a_broken_gantry_or_tool_is_refused() {
    let known: Vec<Config> = MACHINES.iter().map(|m| m.config()).collect();
    let read = |name: &str| std::fs::read_to_string(root().join("machines").join(name)).unwrap().replace("\r\n", "\n");
    let refused = |text: &str, from: &str, to: &str| {
        let broken = text.replacen(from, to, 1);
        assert_ne!(broken, text, "the change applies");
        match sim_netlist::from_text(&broken, true, "broken", &known).err().expect("a broken file loaded").0 {
            ConfigError::Invalid(detail) => detail,
            other => panic!("{other:?}"),
        }
    };
    let gantry = read("dual-gantry.toml");
    assert!(refused(&gantry, "drives = { mx1 = -0.6, mx2 = 0.6 }\n", "").starts_with("machine.rack[rack_x2].axis"));
    let offset = refused(&gantry, "axes = [\"x\"]\npos = [750, 0]", "axes = [\"x\"]\noffset_m = [0.0, 0.0, 0.1]");
    assert_eq!(offset, "machine.body[bridge].offset_m (only on rotary axes)");
    let other = refused(&gantry, "mx2 = 0.6 }", "mx2 = 0.6, my = 0.0 }");
    assert!(other.starts_with("machine.gantry[bridge_yaw].drives.my"), "{other}");
    let portal = read("portal-c.toml");
    let axes = refused(&portal, "axes = [\"x\", \"y\", \"z\"]\nrotaries", "axes = [\"y\", \"x\", \"z\"]\nrotaries");
    assert!(axes.starts_with("machine.tcp[gripper_tip].axes"), "{axes}");
}
