//! The pneumatic Z beside a servo X (machines/pneumatic-axis.toml, sim_components::pneumatic): its cycle commands the
//! valve and waits for the end-position sensors, the cylinder's stroke runs at the hand-calculated speed, its checks
//! are judged, and halving the integration step leaves the stroke time where it is.

use sim_components::analysis::{analyse_cycle, verdicts};
use sim_components::pneumatic::{Flow, R, RHO0, T0};
use sim_components::{Command, Sim};
use sim_core::{Config, MachineEntry};
use sim_machines::load_file;
use std::path::Path;

fn entry() -> &'static MachineEntry {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines/pneumatic-axis.toml");
    load_file(&path).unwrap_or_else(|e| panic!("{e}"))
}

/// Runs the cycle for `seconds` and records each named signal of `ids` every step.
fn run(cfg: &Config, seconds: f64, ids: &[&str]) -> Vec<Vec<f64>> {
    let mut sim = Sim::new(entry(), cfg).unwrap();
    let at: Vec<usize> = ids.iter().map(|id| sim.signal_index(id).unwrap_or_else(|| panic!("no {id}"))).collect();
    sim.command(Command::StartCycle).unwrap();
    let (mut rows, mut sample) = (Vec::new(), Vec::new());
    sim.run(seconds, |s| {
        s.sample_base(&mut sample);
        rows.push(at.iter().map(|&i| sample[i]).collect());
    });
    rows
}

/// The cycle: X goes to the pick, Z extends by its valve, the segment waits for the extended sensor, the gripper
/// takes the part, Z retracts, X travels, and so on. The servo X moves only while Z is up (the retracted sensor
/// reports), and Z runs down at the meter-out speed by hand: its throttle (C 0.15) in series with valve (0.87) and tube
/// (1.0), C = 0.14943 dm3/(s bar), choked, v = C rho0 R T0 / A2 = 1.4943e-9 * 99719 / 6.9115e-4 = 0.2156 m/s
/// whatever the load, gravity and seals (they set the rod side's pressure, not the choked flow).
#[test]
fn the_servo_x_and_the_pneumatic_z_run_their_cycle() {
    let ids =
        ["axis.x.velocity", "cylinder.z.position", "cylinder.z.velocity", "cylinder.z.sensor_retracted", "payload"];
    let rows = run(&entry().config(), 7.0, &ids);
    let [x_speed, z, z_speed, up, payload] = [0, 1, 2, 3, 4];
    // Z went all the way down and up again more than once, and gripped and let go: a down, up and travel of X takes
    // about 2.4 s.
    let downs = rows.windows(2).filter(|w| w[0][z] < 0.099 && w[1][z] >= 0.099).count();
    assert!(downs >= 3, "Z extended {downs} times in 7 s");
    assert!(rows.iter().any(|r| r[payload] == 0.0) && rows.iter().any(|r| r[payload] == 1.0));
    // X moves only with Z up.
    assert!(rows.iter().all(|r| r[x_speed].abs() < 0.01 || r[up] == 1.0), "X moved with Z down");
    assert!(rows.iter().any(|r| r[x_speed].abs() > 0.5), "X never travelled");
    // The extend speed in the middle of the stroke, against the hand value.
    let c = Flow::series(&[Flow::new(0.87, 0.3), Flow::new(1.0, 0.4), Flow::new(0.15, 0.3)]).c;
    let a2 = std::f64::consts::PI / 4.0 * (0.032f64.powi(2) - 0.012f64.powi(2));
    let expected = c * RHO0 * R * T0 / a2;
    assert!((expected - 0.2156).abs() < 1e-3, "hand speed {expected}");
    let middle: Vec<f64> =
        rows.iter().filter(|r| (0.04..0.06).contains(&r[z]) && r[z_speed] > 0.0).map(|r| r[z_speed]).collect();
    let mean = middle.iter().sum::<f64>() / middle.len() as f64;
    assert!((mean - expected).abs() < 0.03 * expected, "extending at {mean} m/s, by hand {expected}");
}

/// The analysis of the cycle judges the cylinder: the kinetic energy entering the cushion, about 1/2 * 1.0 kg (0.5 kg
/// moved, 0.5 kg payload) * 0.2156^2 = 0.023 J at the steady speed (the piston still swings on its air springs about
/// it, so within 30 %), against the DSBC-32's 0.4 J; the load factor, the weight (1 kg) against the retracting force
/// p A2 = 6e5 * 6.9115e-4 = 414.7 N: 9.807 / 414.7 = 0.0236 against 0.7. A rated energy below the impact fails.
#[test]
fn the_cylinder_s_checks_are_judged() {
    let cfg = entry().config();
    let mut sim = Sim::new(entry(), &cfg).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    let judged = verdicts(&cfg, &load).unwrap();
    let find = |check: &str| judged.iter().find(|v| v.check == check).unwrap_or_else(|| panic!("no {check}")).clone();
    let (cushion, factor) = (find("cushion-energy"), find("load-factor"));
    assert!(cushion.ok && factor.ok, "{cushion:?} {factor:?}");
    assert!((cushion.value - 0.0232).abs() < 0.3 * 0.0232, "impact energy {}", cushion.value);
    assert!((factor.value - 9.80665 / 414.69).abs() < 1e-4, "load factor {}", factor.value);
    let mut tight = cfg.clone();
    tight.set_text("cylinder_z.rated_cushion_energy_j = 0.01").unwrap();
    let mut sim = Sim::new(entry(), &tight).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    assert!(!verdicts(&tight, &load).unwrap().iter().find(|v| v.check == "cushion-energy").unwrap().ok);
}

/// The shipped cycle's figures as docs/mechanisms.md gives them (with the port throttle in series with the cushion's
/// bleed): stroke times, the speed and kinetic energy entering the cushion, the speed at the end stop, the air per
/// double stroke and its energy at the compressor (0.12 kWh/m3: 432 J per normal litre).
#[test]
fn the_shipped_cylinder_gives_its_documented_figures() {
    let ids = [
        "cylinder.z.stroke_time_extend",
        "cylinder.z.stroke_time_retract",
        "cylinder.z.impact_speed",
        "cylinder.z.impact_energy",
        "cylinder.z.end_speed",
        "cylinder.z.air_cycle",
        "cylinder.z.compressor_energy",
    ];
    let rows = run(&entry().config(), 7.0, &ids);
    // Each signal holds its figure of the last stroke: the values it took over the run.
    let taken = |i: usize| {
        let mut v: Vec<f64> = rows.iter().map(|r| r[i]).filter(|x| *x > 0.0).collect();
        v.dedup();
        v
    };
    let range = |i: usize| taken(i).iter().fold((f64::MAX, f64::MIN), |(a, b), x| (a.min(*x), b.max(*x)));
    for (i, id) in ids.iter().enumerate() {
        println!("{id}: {:?}", taken(i));
    }
    let within = |i: usize, low: f64, high: f64| {
        let (a, b) = range(i);
        assert!(a >= low && b <= high, "{}: {a} to {b}, documented {low} to {high}", ids[i]);
    };
    // Run on 16 November 2026: extending 0.5506 to 0.5555 s, retracting 0.6523 to 0.6546 s, entering the cushion at 0.218
    // to 0.241 m/s with 0.0119 to 0.0289 J, at the end stop 0.058 to 0.088 m/s, 1.097 l per double stroke (474 J).
    within(0, 0.545, 0.565);
    within(1, 0.645, 0.655);
    within(2, 0.215, 0.245);
    within(3, 0.0115, 0.0295);
    within(4, 0.055, 0.095);
    // The air per double stroke once the first has ended (before it, the signal counts the stroke under way).
    let air = range(5).1;
    assert!((air - 1.10).abs() < 0.005, "{air} l");
    let energy = rows.last().unwrap()[6];
    assert!(energy > 0.0 && (air * 0.12 * 3.6e6 / 1e3 - 474.0).abs() < 1.0, "{air} l, {energy} J");
}

/// Halving the integration step (four sub-steps to eight) changes the cylinder's stroke times by less than 1 %.
#[test]
fn halving_the_step_leaves_the_stroke_time() {
    let times = |substeps: u32| {
        let mut cfg = entry().config();
        cfg.set_text(&format!("sim.substeps = {substeps}")).unwrap();
        let rows = run(&cfg, 3.0, &["cylinder.z.stroke_time_extend", "cylinder.z.stroke_time_retract"]);
        rows.last().unwrap().clone()
    };
    let (coarse, fine) = (times(4), times(8));
    for (c, f) in coarse.iter().zip(&fine) {
        assert!(*f > 0.1, "no stroke measured: {fine:?}");
        assert!((c - f).abs() < 0.01 * f, "stroke time {c} s, at half the step {f} s");
    }
}

/// A cylinder whose dead volume is too small for the integrator's step is refused through the file with its key.
#[test]
fn a_cylinder_too_stiff_for_the_step_is_refused_in_a_file() {
    let text =
        std::fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines/pneumatic-axis.toml"))
            .unwrap()
            .replace("dead_volume_rod_m3 = 8.0e-6", "dead_volume_rod_m3 = 1.0e-9");
    let Err(error) = sim_machines::load_text(&text, "stiff") else { panic!("the stiff cylinder was taken") };
    assert_eq!(error.key(), "error-cylinder-unstable", "{error:?}");
}

/// Every key the pneumatic Z brings has an English and a German entry: its signals' owner and quantity keys, its
/// checks, its effect switches, its table's parameters and its refusal (the registered machines' test does not load
/// machine files).
#[test]
fn every_key_of_the_cylinder_has_an_entry_in_every_language() {
    let entry = entry();
    let sim = Sim::new(entry, &entry.config()).unwrap();
    let mut keys: Vec<String> = sim
        .signal_infos()
        .into_iter()
        .filter(|s| s.id.starts_with("cylinder."))
        .flat_map(|s| [s.owner_key, s.quantity_key])
        .collect();
    keys.extend(sim.machine.rating_checks().iter().map(|c| format!("check-{}", c.check)));
    keys.extend(sim_components::pneumatic::EFFECTS.map(|e| format!("effect-{e}")));
    let table: toml::Table = sim_components::pneumatic::TABLE.parse().unwrap();
    keys.extend(table.keys().map(|k| format!("param-{k}")));
    keys.push("error-cylinder-unstable".into());
    for lang in sim_machines::LANGUAGES {
        let tr = sim_machines::translations(lang, entry);
        for key in &keys {
            assert!(tr.has(key), "no '{lang}' entry for '{key}'");
        }
    }
}
