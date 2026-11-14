//! The component catalogue in `catalog/` as the simulator uses it: parts by name, the searches, and configs built from
//! it. The data itself (physics and agreement between documents) is checked by the catalogue's validator, `catalog/tools/catalog.py validate`.

use std::fs;
use std::path::{Path, PathBuf};

fn catalog() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog")
}

fn close(a: f64, b: f64, tolerance: f64) -> bool {
    (a - b).abs() <= tolerance * b.abs()
}

/// The rows of a catalogue CSV as (column name -> text) lookups; lines starting with '#' are notes.
fn csv(path: &str) -> Vec<std::collections::HashMap<String, String>> {
    let text = fs::read_to_string(catalog().join(path)).unwrap();
    let mut lines = text.lines().filter(|l| !l.starts_with('#'));
    let header: Vec<&str> = lines.next().unwrap().split(',').collect();
    lines
        .map(|line| {
            let fields: Vec<&str> = line.split(',').collect();
            assert_eq!(fields.len(), header.len(), "{path}: {line}");
            header.iter().zip(fields).map(|(h, f)| (h.to_string(), f.to_string())).collect()
        })
        .collect()
}

/// The real-machine config runs its cycle inside every limit the simulator judges so far.
#[test]
fn exct_30_cycle_stays_within_motor_and_belt_limits() {
    let cfg = sim_core::Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap();
    let mut sim = sim_components::Sim::new(&machine_t_gantry::ENTRY, &cfg).unwrap();
    assert!(sim.limit_violations.is_empty(), "{:?}", sim.limit_violations);
    let load = sim_components::analysis::analyse_cycle(&mut sim).unwrap();
    for motor in &load.motors {
        assert!(
            motor.utilisation < 0.8 && motor.thermal_utilisation < 0.8,
            "{} of the peak curve, {} thermal",
            motor.utilisation,
            motor.thermal_utilisation
        );
    }
    let (slackest, tightest) = load.belt_tension.unwrap();
    assert!(slackest > 0.0, "a belt section goes slack");
    assert!(tightest < 12.5 / 0.01591, "{tightest} N is more than the gantry's torque limit puts on the belt");
}

/// The motor the EXCT-30 config names gives it the values the config has by hand.
#[test]
fn a_motor_by_name_gives_the_values_of_the_hand_written_config() {
    let by_hand = sim_core::Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap();
    let mut cfg = by_hand.clone();
    for wrong in [
        "motor.torque_constant_nm_a = 0.2",
        "motor.rated_current_a = 1.0",
        "drive.peak_torque_nm = 1.0",
        "drive.rated_torque_nm = 0.1",
        "drive.rated_speed_rad_s = 10.0",
        "motor.thermal_time_constant_s = 5.0",
    ] {
        cfg.set_text(wrong).unwrap();
    }
    // The config names its motor in [parts]; that brings the catalogue values back.
    assert_eq!(cfg.get("parts.motor").unwrap().as_str(), Some("1FK2205-4AF"));
    let mut cfg = sim_components::catalog::resolve(&machine_t_gantry::ENTRY, &cfg).unwrap();
    for path in [
        "motor.rotor_inertia_kgm2",
        "motor.torque_constant_nm_a",
        "motor.max_current_a",
        "motor.rated_current_a",
        "motor.torque_constant_at_max_current_nm_a",
        "motor.thermal_time_constant_s",
        "drive.peak_torque_nm",
        "drive.rated_torque_nm",
        "drive.rated_speed_rad_s",
        "drive.no_load_speed_rad_s",
        "drive.bus_voltage_v",
        "ratings.braking_threshold_v",
        "ratings.dc_link_overvoltage_v",
    ] {
        assert!(
            close(cfg.f64(path).unwrap(), by_hand.f64(path).unwrap(), 2e-3),
            "{path}: {} against {} by hand",
            cfg.f64(path).unwrap(),
            by_hand.f64(path).unwrap()
        );
    }
    // The numbers of the file are the motor's own, so there is nothing to retune the velocity gain for.
    assert!(close(cfg.f64("drive.velocity_kp_nm_s_rad").unwrap(), 0.45, 1e-9));
    // The rated current is the catalogue's own, not rated torque over the static torque constant: for the
    // 1FK2204-5AK at 400 V 1.95 A, where 1.5 N m / (2.4 N m / 4.4 A) would be 2.75 A * 0.6 = 1.65 A. The one
    // row whose rated current is a misprint (3.75 A at a stall current of 2.8 A) stops 5 % above the stall current.
    for (article, supply, rated) in [("1FK2204-5AK", "400v", 1.95), ("1FT2206-2AC", "230v", 1.05 * 2.8)] {
        sim_components::catalog::apply_motor(&mut cfg, article, supply).unwrap();
        assert!(
            close(cfg.f64("motor.rated_current_a").unwrap(), rated, 1e-12),
            "{article}: {} A",
            cfg.f64("motor.rated_current_a").unwrap()
        );
    }
    // Not offered at 230 V single-phase data, and an unknown article, are refused.
    assert!(sim_components::catalog::apply_motor(&mut cfg, "1FK2205-4AF", "110v").is_err());
    assert!(sim_components::catalog::apply_motor(&mut cfg, "1FK9999-0AA", "400v").is_err());
}

/// A gearbox named by size, motor frame and ratio sets the gearbox of the config from the catalogue row.
#[test]
fn a_gearbox_by_name_sets_ratio_stiffness_backlash_and_ratings() {
    let mut cfg = machine_t_gantry::ENTRY.config();
    let rotor = cfg.f64("motor.rotor_inertia_kgm2").unwrap();
    assert_eq!(cfg.f64("gearbox.inertia_at_input_kgm2").unwrap(), 0.0);
    sim_components::catalog::apply_gearbox(&mut cfg, "NRK070-04-10").unwrap();
    let row = csv("siemens/gearbox_nrk.csv")
        .into_iter()
        .find(|r| r["size"] == "NRK070" && r["motor_frame"] == "04" && r["ratio"] == "10")
        .unwrap();
    let number = |column: &str| -> f64 { row[column].parse().unwrap() };
    assert_eq!(cfg.f64("gearbox.ratio").unwrap(), 10.0);
    assert!(close(cfg.f64("gearbox.stiffness_nm_rad").unwrap(), number("torsional_stiffness_nm_rad"), 1e-9));
    assert!(close(cfg.f64("gearbox.backlash_rad").unwrap(), number("backlash_rad"), 1e-9));
    assert_eq!(cfg.f64("gearbox.efficiency").unwrap(), number("efficiency_full_load"));
    // The gearbox's inertia has a key of its own and leaves the rotor's alone.
    assert!(
        close(cfg.f64("gearbox.inertia_at_input_kgm2").unwrap(), number("inertia_at_input_kgm2"), 1e-9)
            && cfg.f64("motor.rotor_inertia_kgm2").unwrap() == rotor
    );
    assert_eq!(cfg.f64("ratings.gearbox_max_torque_nm").unwrap(), number("max_output_torque_nm"));
    assert!(sim_components::catalog::apply_gearbox(&mut cfg, "NRK070-04-11").is_err());
}

/// The search on the EXCT-30: the smallest motors cannot run the cycle, the motor the config was written
/// for can, and among the passing ones the lightest comes first.
#[test]
fn the_motor_search_ranks_passing_motors_by_weight_and_fails_the_small_ones() {
    let cfg = sim_core::Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap();
    let wanted =
        |article: &str| ["1FK2102-1AG", "1FK2203-4AK", "1FK2205-2AF", "1FK2205-4AF", "1FK2206-2AF"].contains(&article);
    let found = sim_components::catalog::search_motors(&machine_t_gantry::ENTRY, &cfg, "400v", &wanted, None);
    let place = |article: &str| found.iter().position(|c| c.article == article);
    let by_hand = &found[place("1FK2205-4AF").unwrap()];
    assert!(by_hand.passes(), "{:?}", by_hand.tightest());
    // The 1FK2102 is offered at 230 V only, so it is not among the 400 V candidates at all.
    assert!(place("1FK2102-1AG").is_none());
    let small = &found[place("1FK2203-4AK").unwrap()];
    assert!(!small.passes(), "a 1.27 N m motor passes: {:?}", small.tightest());
    let passing: Vec<&sim_components::catalog::Candidate> = found.iter().take_while(|c| c.passes()).collect();
    assert!(!passing.is_empty() && passing.windows(2).all(|w| w[0].mass <= w[1].mass));
    assert!(found.iter().skip(passing.len()).all(|c| !c.passes()));
}

/// A motor and a gearbox named in `[parts]` give exactly the machine that the same config gives with the
/// numbers put in by `apply_gearbox` and `apply_motor` and the velocity gain retuned for them, when a
/// simulation starts and when a running one is given the config. An unknown article is refused, with a
/// message in every language.
#[test]
fn parts_named_in_a_config_behave_like_their_numbers() {
    use sim_components::catalog::{apply_gearbox, apply_motor, resolve};
    use sim_components::{Command, Sim};
    for entry in sim_machines::MACHINES {
        let mut named = entry.config();
        named.set_text("parts.motor = \"1FK2104-4AK\"").unwrap();
        named.set_text("parts.gearbox = \"NRK070-04-5\"").unwrap();
        let mut numbers = entry.config();
        apply_gearbox(&mut numbers, "NRK070-04-5").unwrap();
        apply_motor(&mut numbers, "1FK2104-4AK", "400v").unwrap();
        let resolved = resolve(entry, &named).unwrap();
        // The rotor is a little heavier than the config's and the gearbox adds its own inertia: more gain.
        let (gain, own) =
            (resolved.f64("drive.velocity_kp_nm_s_rad").unwrap(), numbers.f64("drive.velocity_kp_nm_s_rad").unwrap());
        // Where the named gearbox has the config's ratio (5); another ratio changes the load the motor sees, and the
        // gain with it (a screw on a coupling: far less).
        if entry.config().f64("gearbox.ratio").unwrap() == 5.0 {
            assert!(gain > own && gain < 2.0 * own, "{}: gain {gain}", entry.id);
        }
        numbers.set_text(&format!("drive.velocity_kp_nm_s_rad = {gain:?}")).unwrap();
        assert_eq!(resolved, numbers, "{}", entry.id);
        // Resolved once, a config names no part any more: doing it again changes nothing.
        assert_eq!(resolve(entry, &resolved).unwrap(), resolved, "{}", entry.id);
        assert_eq!(numbers.f64("gearbox.efficiency").unwrap(), 0.97);
        let cycle = |mut sim: Sim| {
            sim.command(Command::StartCycle).unwrap();
            sim.run(0.3, |_| {});
            sim.state.clone()
        };
        let started = cycle(Sim::new(entry, &numbers).unwrap());
        assert_eq!(cycle(Sim::new(entry, &named).unwrap()), started, "{}", entry.id);
        assert_ne!(cycle(Sim::new(entry, &entry.config()).unwrap()), started, "{}: the parts change nothing", entry.id);
        let reconfigured = |cfg: &sim_core::Config| {
            let mut running = Sim::new(entry, &entry.config()).unwrap();
            running.reconfigure(entry, cfg).unwrap();
            cycle(running)
        };
        assert_eq!(reconfigured(&named), reconfigured(&numbers), "{}", entry.id);

        named.set_text("parts.motor = \"1FK9999-0AA\"").unwrap();
        let refused = Sim::new(entry, &named).err().unwrap();
        assert_eq!((refused.key(), refused.detail()), ("error-config-part-motor", "1FK9999-0AA"));
        for lang in sim_machines::LANGUAGES {
            assert!(sim_machines::translations(lang, entry).has(refused.key()), "{lang}");
        }
    }
}

/// Every bound in a config names a value of that config and has it between its low and its high.
#[test]
fn the_bounds_of_every_config_bracket_their_values() {
    let mut configs: Vec<sim_core::Config> = sim_machines::MACHINES.iter().map(|entry| entry.config()).collect();
    configs.push(sim_core::Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap());
    for cfg in configs {
        let bounds = sim_components::analysis::bounds(&cfg).unwrap();
        assert!(bounds.len() >= 14, "{} bounds", bounds.len());
        for (path, low, high) in bounds {
            let value = cfg.f64(&path).unwrap();
            assert!(low < high && low <= value && value <= high, "{path}: {value} is not within {low} to {high}");
        }
    }
}

/// Retuning: the 1FK2204-6AF has a third of the rotor inertia of the EXCT-30 config's motor. With the gain scaled
/// by the geometric mean of the ratio of the inertia the motors turn and the ratio of the rotor inertias it runs
/// the cycle with about the torque of the config's own motor. The 1FK2104-5AF has a ninth of the rotor inertia:
/// with the config's own gain (0.45 N m s/rad) its loop oscillates and the winding overheats, scaled with the
/// inertia the motors turn alone (the rule before) it still oscillates, with the mean it runs calmly. (The config
/// had 0.6 until its velocity loop was tuned for margin; with that the 1FK2204-6AF oscillated as well, with 0.45 it
/// runs as calmly as retuned.)
#[test]
fn the_search_retunes_the_velocity_loop_for_a_lighter_rotor() {
    use sim_components::catalog::{search_motors, with_parts};
    let entry = &machine_t_gantry::ENTRY;
    let cfg = sim_core::Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap();
    let cycle = |cfg: &sim_core::Config| {
        sim_components::analysis::analyse_cycle(&mut sim_components::Sim::new(entry, cfg).unwrap()).unwrap().motors[0]
            .clone()
    };
    let own = cycle(&cfg);
    // The motor's numbers in the config and the gain left as it is: what naming the motor used to give.
    let mut same_gain = cfg.clone();
    sim_components::catalog::apply_motor(&mut same_gain, "1FK2104-5AF", "400v").unwrap();
    same_gain.set_text("parts.motor = \"\"").unwrap();
    let retuned = with_parts(entry, &cfg, "1FK2204-6AF", "400v", "").unwrap();
    // Naming the motor in the config retunes like the search does, so it runs as calmly.
    let mut named = cfg.clone();
    named.set_text("parts.motor = \"1FK2204-6AF\"").unwrap();
    assert_eq!(sim_components::catalog::resolve(entry, &named).unwrap(), retuned);
    assert!(cycle(&named).winding_rise < 50.0, "{} K with the motor named", cycle(&named).winding_rise);
    // The gain by hand. Per unit of acceleration of X or Z each motor needs (J_rotor + J_pulley) / r for
    // itself and m r / 2 for the load, with 14.86 kg moving in X and 7.7 kg in Z (payload attached). Summed
    // over two axes and two motors: 4 (J_rotor + 2e-5) / r + 22.56 r, with r = 15.91 mm. The belt's own mass
    // adds a little to both sums.
    let seen = |rotor: f64| 4.0 * (rotor + 2.0e-5) / 0.01591 + 22.56 * 0.01591;
    let own_gain = cfg.f64("drive.velocity_kp_nm_s_rad").unwrap();
    let (gain, by_hand) = (
        retuned.f64("drive.velocity_kp_nm_s_rad").unwrap(),
        own_gain * (seen(1.6e-4) / seen(5.1e-4) * 1.6 / 5.1).sqrt(),
    );
    assert!(close(gain, by_hand, 0.02), "gain {gain}, by hand {by_hand}");
    let (oscillating, calm) = (cycle(&same_gain), cycle(&retuned));
    assert!(
        oscillating.rms_torque > 2.0 * own.rms_torque && oscillating.winding_rise > 100.0,
        "{} N m, {} K",
        oscillating.rms_torque,
        oscillating.winding_rise
    );
    // The lighter rotor needs a little less torque than the config's own motor (1.74 N m RMS).
    assert!(
        calm.rms_torque > 0.7 * own.rms_torque && calm.rms_torque < 1.05 * own.rms_torque,
        "{} N m against {} N m",
        calm.rms_torque,
        own.rms_torque
    );
    assert!(calm.winding_rise < 50.0, "{} K", calm.winding_rise);
    // The lightest rotor that has the torque for this cycle. The rule before gave it the config's gain times 0.77.
    let lightest = with_parts(entry, &cfg, "1FK2104-5AF", "400v", "").unwrap();
    let mut total_only = lightest.clone();
    total_only.set_text(&format!("drive.velocity_kp_nm_s_rad = {:?}", own_gain * seen(5.6e-5) / seen(5.1e-4))).unwrap();
    let (oscillating, calm) = (cycle(&total_only), cycle(&lightest));
    assert!(
        oscillating.rms_torque > 2.0 * own.rms_torque && oscillating.winding_rise > 100.0,
        "{} N m, {} K",
        oscillating.rms_torque,
        oscillating.winding_rise
    );
    assert!(
        calm.rms_torque > 0.7 * own.rms_torque && calm.rms_torque < 1.05 * own.rms_torque,
        "{} N m against {} N m",
        calm.rms_torque,
        own.rms_torque
    );
    assert!(calm.winding_rise < 50.0, "{} K", calm.winding_rise);
    // The search does the same, and this machine has no gearbox to exchange (ratio 1), so none is tried.
    let found = search_motors(entry, &cfg, "400v", &|article| article == "1FK2204-6AF", Some("NRK"));
    assert!(found.len() == 1 && found[0].passes() && found[0].gearbox.is_empty(), "{:?}", found[0].tightest());
}

/// The gearbox search on the single axis (ratio 5, 1 m/s over a 20 mm pulley radius: 50 rad/s at the pulley).
/// Two motors with the NRK070 gearboxes for their frame, ratios 3 to 10. The 1FK2104-4AF is rated for
/// 3000 r/min = 314 rad/s, so ratios 7 and above (350 rad/s and more) take it beyond its rated speed. Before, the
/// search ruled those out by the rated speed without a simulation; the rated speed is no limit (above it the curve
/// falls until the back-EMF uses up the DC link), its maximum speed is: 7200 r/min = 754 rad/s. Now every candidate is
/// simulated, and the 1FK2104-4AF behind the fast ratios turns above its rated speed, within its maximum, and passes.
#[test]
fn the_gearbox_search_rules_out_by_catalogue_numbers_only_what_would_fail() {
    use sim_components::catalog::search_motors;
    let entry = &machine_single_axis::ENTRY;
    let cfg = entry.config();
    let found = search_motors(
        entry,
        &cfg,
        "400v",
        &|article| ["1FK2104-4AF", "1FK2104-5AK"].contains(&article),
        Some("NRK070"),
    );
    let ratios = ["3", "4", "5", "7", "8", "9", "10"];
    assert_eq!(found.len(), 2 * ratios.len());
    for c in &found {
        let ratio = c.gearbox.strip_prefix("NRK070-04-").unwrap_or_else(|| panic!("{}", c.gearbox));
        let fast = ratios.iter().position(|r| *r == ratio).unwrap() >= 3;
        let Some((load, verdicts)) = &c.result else {
            panic!("{} {} was not simulated: {}", c.article, c.gearbox, c.refused)
        };
        if c.article == "1FK2104-4AF" && fast {
            let speed = verdicts.iter().find(|v| v.check == "motor-speed").unwrap();
            assert!(close(speed.limit, 7200.0 * std::f64::consts::PI / 30.0, 1e-9), "{speed:?}");
            assert!(load.motors[0].peak_speed > 314.16 && speed.ok, "{} {}: {speed:?}", c.article, c.gearbox);
            assert!(c.passes(), "{} {}", c.article, c.gearbox);
        }
    }
    // Passing ones first, lightest motor first (the gearbox files have no mass), then the rest.
    let passing = found.iter().take_while(|c| c.passes()).count();
    assert!(passing >= 2 && found[..passing].windows(2).all(|w| w[0].mass <= w[1].mass));
    assert!(found[passing..].iter().all(|c| !c.passes()));
}

/// A motor is applied with the DC link of its supply, so the search on a default config (written for a DC
/// link of 155 V, 190 V at the winding) judges the catalogue's 400 V motors on 540 V. Before, most of them
/// met their own back-EMF, never reached their targets and failed "settling" five hundred to two thousand times over. Now no candidate
/// fails settling, and what fails does so for what the motor is: too slow for the axis, or too much rotor
/// to brake for the config's braking resistor.
#[test]
fn the_search_on_a_default_config_fails_motors_for_physical_reasons() {
    use sim_components::catalog::{apply_motor, search_motors};
    let physical = [
        "motor-peak",
        "motor-thermal",
        "gearbox-peak",
        "gearbox-continuous",
        "braking-resistor-continuous",
        "braking-resistor-peak",
    ];
    for entry in sim_machines::MACHINES {
        let mut cfg = entry.config();
        // A config written for a catalogue motor's own DC link (540 V) has no placeholder drive to correct.
        if cfg.f64("drive.bus_voltage_v").unwrap() != 155.13 {
            continue;
        }
        // The braking threshold and the trip level keep their distance to the bus: 224.65 V and 257.6 V at 155.13 V.
        for (supply, bus) in [("230v", 325.0), ("400v", 540.0)] {
            apply_motor(&mut cfg, "1FK2104-5AK", supply).unwrap();
            assert_eq!(cfg.f64("drive.bus_voltage_v").unwrap(), bus, "{supply}");
            assert!(close(cfg.f64("ratings.braking_threshold_v").unwrap(), bus * 224.65 / 155.13, 1e-12), "{supply}");
            assert!(close(cfg.f64("ratings.dc_link_overvoltage_v").unwrap(), bus * 257.6 / 155.13, 1e-12), "{supply}");
        }
        // So the verdicts of a default config with a catalogue motor have a DC link with room in it, and
        // the simulated DC link takes the config. (The 250 V threshold of the 190 V placeholder drive was
        // below the motor's 540 V and gave a negative limit that passed.)
        let mut named = entry.config();
        named.set_text("parts.motor = \"1FK2104-5AK\"").unwrap();
        named.set_text("effects.dc_link = true").unwrap();
        let load =
            sim_components::analysis::analyse_cycle(&mut sim_components::Sim::new(entry, &named).unwrap()).unwrap();
        let link = sim_components::analysis::verdicts(&named, &load)
            .unwrap()
            .into_iter()
            .find(|v| v.check == "dc-link-energy")
            .unwrap();
        assert!(link.limit > 0.0 && link.value <= link.limit && link.ok, "{}: {link:?}", entry.id);
        // A threshold below the bus is refused, simulated or not.
        named.set_text("effects.dc_link = false").unwrap();
        named.set_text("parts.motor = \"\"").unwrap();
        named.set_text("drive.bus_voltage_v = 300.0").unwrap();
        assert_eq!(
            sim_components::analysis::verdicts(&named, &load).unwrap_err().detail(),
            "ratings.braking_threshold_v",
            "{}",
            entry.id
        );
        let wanted = |article: &str| ["1FK2104-5AF", "1FK2205-4AF", "1FK2206-4AF", "1FK2208-3AC"].contains(&article);
        let found = search_motors(entry, &entry.config(), "400v", &wanted, None);
        assert_eq!(found.len(), 4, "{}", entry.id);
        for c in &found {
            // Every candidate is simulated. The 1FK2208-3AC, rated for 2000 r/min, was ruled out by its rated speed before
            // (too slow behind the 5:1 gearbox); it turns 200 and 250 rad/s there, within its maximum of 429 rad/s.
            assert!(c.refused.is_empty(), "{} {}: {}", entry.id, c.article, c.refused);
            if c.article == "1FK2208-3AC" {
                assert!(c.passes(), "{} {}", entry.id, c.article);
            }
            let Some((load, verdicts)) = &c.result else { continue };
            let settling = verdicts.iter().find(|v| v.check == "settling").unwrap();
            assert!(
                settling.ok && settling.value < 0.6 * settling.limit,
                "{} {}: {} m from the target",
                entry.id,
                c.article,
                settling.value
            );
            for v in verdicts.iter().filter(|v| !v.ok) {
                assert!(physical.contains(&v.check), "{} {} fails {}", entry.id, c.article, v.check);
            }
            // The current loop is calm on the higher bus with the gains the config has.
            assert!(load.motors.iter().all(|m| m.winding_rise < 50.0), "{} {}", entry.id, c.article);
        }
        // The rotor of the lightest of them was 1500 times the settling tolerance away before.
        assert!(found.iter().find(|c| c.article == "1FK2104-5AF").unwrap().passes(), "{}", entry.id);
    }
}

/// Every motor of the S210 catalogue reaches its rated speed at its rated torque inside the voltage limit
/// that `apply_motor` gives it. The back-EMF there is the static torque constant times the rated speed (the
/// catalogue has no winding resistance, so the few volts of R i are not counted). The winding gets
/// sqrt(3/2) times the DC link voltage; with the DC link voltage itself as the limit, five rows (four
/// motors of 2000 r/min and one of 1500 r/min at 400 V, back-EMF 554 V and 558 V against 540 V) could not.
#[test]
fn every_catalogue_motor_reaches_its_rated_point_inside_the_voltage_limit() {
    use sim_components::drive::DriveParams;
    let (mut rows, mut beyond_the_dc_link) = (0, 0);
    for m in csv("siemens/motors_s210.csv") {
        for (supply, bus) in [("230v", 325.0), ("400v", 540.0)] {
            if m[&format!("rated_speed_{supply}_rad_s")].is_empty() {
                continue;
            }
            let mut cfg = machine_single_axis::ENTRY.config();
            sim_components::catalog::apply_motor(&mut cfg, &m["part_number"], supply).unwrap();
            let p = DriveParams::from_config(&cfg, 62.5e-6, 125e-6).unwrap();
            assert_eq!(p.bus_voltage, bus);
            let back_emf = p.back_emf(0.0, p.rated_speed);
            assert!(
                back_emf < p.max_voltage,
                "{} at {supply}: {back_emf} V of back-EMF, {} V at the winding",
                m["part_number"],
                p.max_voltage
            );
            // The torque-speed curve agrees: full torque at the rated speed, and none where the voltage is used up.
            assert!(p.torque_curve(p.rated_speed).1 == p.peak_torque && p.no_load_speed >= 1.1 * p.rated_speed);
            rows += 1;
            beyond_the_dc_link += (back_emf > bus) as usize;
        }
    }
    assert!(rows > 120 && beyond_the_dc_link == 5, "{rows} rows, {beyond_the_dc_link} beyond the DC link voltage");
}

/// A gearbox named for the whole search and the same gearbox found by the search give the same candidate.
/// The first route puts the gearbox into the config before the search puts each motor in; the gearbox's
/// inertia used to be added to the rotor's, where the motor then overwrote it (0.5581 N m of peak torque
/// against 0.5873 N m for the 1FK2104-5AF behind an NRK070-04-5 on the single axis).
#[test]
fn a_gearbox_named_before_the_search_keeps_its_inertia() {
    use sim_components::catalog::{resolve, with_parts};
    let entry = &machine_single_axis::ENTRY;
    let mut named = entry.config();
    named.set_text("parts.gearbox = \"NRK070-04-5\"").unwrap();
    // As the command line does: the config's parts first, then the search's motor.
    let before = with_parts(entry, &resolve(entry, &named).unwrap(), "1FK2104-5AF", "400v", "").unwrap();
    let by_search = with_parts(entry, &entry.config(), "1FK2104-5AF", "400v", "NRK070-04-5").unwrap();
    let inertia = |cfg: &sim_core::Config| {
        cfg.f64("motor.rotor_inertia_kgm2").unwrap() + cfg.f64("gearbox.inertia_at_input_kgm2").unwrap()
    };
    // Rotor 0.56 kg cm2 and gearbox 0.135 kg cm2 (catalog/siemens/gearbox_nrk.csv).
    let row = csv("siemens/gearbox_nrk.csv")
        .into_iter()
        .find(|r| r["size"] == "NRK070" && r["motor_frame"] == "04" && r["ratio"] == "5")
        .unwrap();
    let by_hand = 0.56e-4 + row["inertia_at_input_kgm2"].parse::<f64>().unwrap();
    assert!(
        close(inertia(&before), by_hand, 1e-12) && close(inertia(&by_search), by_hand, 1e-12),
        "{} and {} kg m2",
        inertia(&before),
        inertia(&by_search)
    );
    for ((path, a), (_, b)) in before.leaves().into_iter().zip(by_search.leaves()) {
        let same = match (a.as_float(), b.as_float()) {
            (Some(a), Some(b)) => close(a, b, 1e-12) || a == b,
            _ => a == b,
        };
        assert!(same, "{path}: {a} with the gearbox named before, {b} with the search's");
    }
    let peak = |cfg: &sim_core::Config| {
        sim_components::analysis::analyse_cycle(&mut sim_components::Sim::new(entry, cfg).unwrap()).unwrap().motors[0]
            .peak_torque
    };
    assert!(close(peak(&before), peak(&by_search), 1e-6), "{} and {} N m", peak(&before), peak(&by_search));
}

/// The search's "tightest check" leaves out a DC link that is full by design. With a braking resistor the
/// simulated DC link fills to the chopper's threshold in every cycle that brakes hard enough, so its energy
/// stood at 1.0000 for every passing motor (and its voltage at threshold over trip level, 0.939 on the
/// EXCT-30) and hid the check that is really the closest.
#[test]
fn the_tightest_check_leaves_out_a_dc_link_that_a_resistor_backs() {
    use sim_components::analysis::{CycleLoad, Verdict};
    use sim_components::catalog::Candidate;
    let verdict =
        |check: &'static str, value: f64, limit: f64, ok: bool| Verdict { check, motor: Some(0), value, limit, ok };
    let load = CycleLoad {
        duration: 1.0,
        belt_tension: None,
        belt_slack_time: None,
        belt_tooth_jumps: None,
        belt_span_detuning: None,
        radial_load_peak: None,
        condition_peak: None,
        guide_load: None,
        guide_speed: None,
        path_error_peak: 0.0,
        path_error_rms: 0.0,
        tracking_error_peak: 0.0,
        dwell_error_peak: 0.0,
        tripped: false,
        motors: Vec::new(),
        budget: None,
        part_rises: Vec::new(),
        checks: Vec::new(),
        clearance: None,
        contact: None,
        braking_power_peak: 0.0,
        warm_start: None,
        conveyor: None,
    };
    let candidate = |verdicts: Vec<Verdict>| Candidate {
        article: "",
        gearbox: String::new(),
        drive: "",
        axis: String::new(),
        motor: String::new(),
        mass: 1.0,
        cost: 1.0,
        result: Some((load.clone(), verdicts)),
        refused: "",
    };
    let full = verdict("dc-link-energy", 6.2, 6.2, true);
    let with_resistor = candidate(vec![
        verdict("motor-peak", 0.7, 1.0, true),
        full.clone(),
        verdict("braking-resistor-continuous", 6.0, 20.0, true),
        verdict("dc-link-overvoltage", 770.0, 820.0, true),
    ]);
    assert_eq!(with_resistor.tightest(), Some(("motor-peak", 0.7)));
    // An S120 line-up's one Braking Module is judged once, without a motor: it backs every axis's DC link.
    let module = Verdict { motor: None, ..verdict("braking-resistor-continuous", 600.0, 1500.0, true) };
    assert_eq!(
        candidate(vec![verdict("motor-peak", 0.7, 1.0, true), full.clone(), module]).tightest(),
        Some(("motor-peak", 0.7))
    );
    // Without a resistor a full DC link is the limit, and one that fails is the reason. The voltage, which
    // starts at the bus level, says nothing while it passes.
    assert_eq!(
        candidate(vec![
            verdict("motor-peak", 0.7, 1.0, true),
            verdict("dc-link-energy", 5.0, 6.2, true),
            verdict("dc-link-overvoltage", 760.0, 820.0, true)
        ])
        .tightest()
        .unwrap()
        .0,
        "dc-link-energy"
    );
    assert_eq!(
        candidate(vec![verdict("motor-peak", 0.7, 1.0, true), verdict("dc-link-overvoltage", 830.0, 820.0, false)])
            .tightest()
            .unwrap()
            .0,
        "dc-link-overvoltage"
    );
    assert_eq!(
        candidate(vec![
            verdict("motor-peak", 0.7, 1.0, true),
            verdict("dc-link-energy", 9.3, 6.2, false),
            verdict("braking-resistor-peak", 1.0, 2.0, true)
        ])
        .tightest(),
        Some(("dc-link-energy", 1.5))
    );
    // A contact with no clearance asked for (`profile.min_clearance_m` 0, as on the single axis) fails and is the
    // tightest; the same check passing at a clearance has no share.
    let contact = Verdict { motor: None, ..verdict("collision", -0.03, 0.0, false) };
    assert_eq!(
        candidate(vec![verdict("gearbox-radial-load", 0.42, 1.0, true), contact]).tightest(),
        Some(("collision", f64::INFINITY))
    );
    let clear = Verdict { motor: None, ..verdict("collision", 0.1, 0.0, true) };
    assert_eq!(
        candidate(vec![verdict("gearbox-radial-load", 0.42, 1.0, true), clear]).tightest(),
        Some(("gearbox-radial-load", 0.42))
    );
    // The belt's slack counts once it fails (a section at -12 N against 0.5 N: infinitely far over), so a candidate
    // that fails it ranks behind every near miss; while it passes it says nothing of a limit and is left out. Before, it
    // was left out either way, and the failing candidate ranked by its motor-peak of 0.7.
    let pushed = Verdict { motor: None, ..verdict("belt-slack", -12.0, 0.5, false) };
    assert_eq!(
        candidate(vec![verdict("motor-peak", 0.7, 1.0, true), pushed]).tightest(),
        Some(("belt-slack", f64::INFINITY))
    );
    let taut = Verdict { motor: None, ..verdict("belt-slack", 200.0, 0.5, true) };
    assert_eq!(candidate(vec![verdict("motor-peak", 0.7, 1.0, true), taut]).tightest(), Some(("motor-peak", 0.7)));
}

/// With --motor-id the search judges one motor: its tightest check is among its own checks (and those of the whole
/// machine where they fail), not another motor's. Two motors, "mx" at index 0 with a gearbox radial load of 0.41 judged
/// for the machine and a motor-peak of 0.9, "mz" at index 1 with a motor-peak of 0.3: for "mz" the tightest is its own
/// 0.3. Before, every Z candidate of the XYZ portal showed the machine's gearbox-radial-load 0.4127.
#[test]
fn the_tightest_check_of_a_one_motor_search_is_that_motors() {
    use sim_components::analysis::{CycleLoad, MotorLoad, Verdict};
    use sim_components::catalog::Candidate;
    let load = CycleLoad {
        duration: 1.0,
        belt_tension: None,
        belt_slack_time: None,
        belt_tooth_jumps: None,
        belt_span_detuning: None,
        radial_load_peak: None,
        condition_peak: None,
        guide_load: None,
        guide_speed: None,
        path_error_peak: 0.0,
        path_error_rms: 0.0,
        tracking_error_peak: 0.0,
        dwell_error_peak: 0.0,
        tripped: false,
        motors: vec![
            MotorLoad { id: "mx".into(), ..MotorLoad::default() },
            MotorLoad { id: "mz".into(), ..MotorLoad::default() },
        ],
        budget: None,
        part_rises: Vec::new(),
        checks: Vec::new(),
        clearance: None,
        contact: None,
        braking_power_peak: 0.0,
        warm_start: None,
        conveyor: None,
    };
    let verdicts = vec![
        Verdict { check: "motor-peak", motor: Some(0), value: 0.9, limit: 1.0, ok: true },
        Verdict { check: "motor-peak", motor: Some(1), value: 0.3, limit: 1.0, ok: true },
        Verdict { check: "gearbox-radial-load", motor: None, value: 0.41, limit: 1.0, ok: true },
    ];
    let candidate = |motor: &str, verdicts: Vec<Verdict>| Candidate {
        article: "",
        gearbox: String::new(),
        drive: "",
        axis: String::new(),
        motor: motor.into(),
        mass: 1.0,
        cost: 1.0,
        result: Some((load.clone(), verdicts)),
        refused: "",
    };
    assert_eq!(candidate("mz", verdicts.clone()).tightest(), Some(("motor-peak", 0.3)));
    assert_eq!(candidate("", verdicts.clone()).tightest(), Some(("motor-peak", 0.9)));
    // A check of the whole machine that fails is the one motor's business too.
    let mut failing = verdicts;
    failing[2] = Verdict { value: 1.2, ok: false, ..failing[2].clone() };
    assert_eq!(candidate("mz", failing).tightest(), Some(("gearbox-radial-load", 1.2)));
}

/// `search --gearboxes` keeps a motor that no gearbox of the series fits, tried with the config's own gearbox (gearbox
/// ""). The 1FK7 04x on S120 have no NRK gearbox for their frame: before, the single axis's search gave 0 candidates
/// of 9; now all 9 are simulated.
#[test]
fn the_gearbox_search_keeps_motors_that_no_gearbox_fits() {
    use sim_components::catalog::search_motors;
    let entry = &machine_single_axis::ENTRY;
    let found = search_motors(entry, &entry.config(), "s120", &|a: &str| a.starts_with("1FK704"), Some("NRK"));
    assert_eq!(found.len(), 9);
    assert!(found.iter().all(|c| c.gearbox.is_empty() && c.result.is_some()), "{found:?}");
}

/// The cost of a search counts the motors that take the candidate: on the XYZ portal X and Y share the motor, Z has its
/// own ([motor.mz]), so a 1FK2103-2AH (1.18 kg) costs 2 * 1.18 = 2.36 kg there, not 3 * 1.18. And a one-motor search's
/// mass is that motor's with its own gearbox (Z has none by name: 1.18 kg).
#[test]
fn the_search_cost_counts_the_motors_that_take_the_candidate() {
    use sim_components::catalog::{search_motor, search_motors};
    let entry = sim_machines::load_file(&Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines/xyz-portal.toml"))
        .unwrap_or_else(|e| panic!("{e}"));
    let article = "1FK2103-2AH";
    let found = search_motors(entry, &entry.config(), "400v", &|a: &str| a == article, None);
    assert!(close(found[0].cost, 2.0 * 1.18, 1e-12), "{}", found[0].cost);
    let one = search_motor(entry, &entry.config(), "400v", &|a: &str| a == article, None, "mz").unwrap();
    assert!(close(one[0].cost, 1.18, 1e-12) && close(one[0].mass, 1.18, 1e-12), "{:?}", (one[0].cost, one[0].mass));
}

/// The EXCT-30 config passes every check with margin: in the worst case of both band modes. One at a time,
/// each of its 67 bounded values at its low and at its high bound (135 cycles); and all of them between their
/// bounds at once, 60 times. Its belt pretension, X jerk limit, input shaper and corner blending were chosen for
/// this (see the comments in the file): the slackest belt section keeps more than 100 N, the pulley shafts
/// carry less than the 1500 N of the lowest shaft rating the file allows for, and the tool is within 0.07 mm
/// in the second half of every dwell, against a tolerance of 0.1 mm.
///
/// The pulley shaft bearings ("pulley-bearing-life") pass too since the life modification factor a_ISO of ISO 281 is in
/// their life: 325800 h as written (31800 h without it), above 73000 h with each value at a bound alone and above
/// 51000 h in the 60 samples. At the worst corner of the bounds they do not (see the test below): that is a finding
/// about the estimates (Festo publishes neither the bearings nor their lubrication), held here so that it shows when
/// they change.
///
/// The bands one at a time are those the worst-corner search below starts with, and judged there.
#[test]
fn the_exct_30_config_passes_every_check_in_the_worst_case_of_both_band_modes() {
    use sim_components::analysis::{analyse_cycle, bounds, verdict_bands_sampled};
    let cfg = sim_components::load_config(&machine_t_gantry::ENTRY, machine_t_gantry::EXCT_30_CONFIG).unwrap();
    let bounded = bounds(&cfg).unwrap();
    assert!(bounded.len() >= 60 && bounded.contains(&("belt.pretension_n".to_string(), 350.0, 550.0)));
    exct_30_passes_with_margin(
        "sampled",
        &verdict_bands_sampled(&machine_t_gantry::ENTRY, &cfg, &bounded, 60, 1).unwrap(),
    );
    let cycle = analyse_cycle(&mut sim_components::Sim::new(&machine_t_gantry::ENTRY, &cfg).unwrap()).unwrap().duration;
    assert!((cycle - 2.048).abs() < 0.01, "cycle {cycle} s");
}

/// The EXCT-30's checks pass with margin in the worst case of the bands of `mode` (the test above).
fn exct_30_passes_with_margin(mode: &str, bands: &[sim_components::analysis::Band]) {
    let band = |check: &str| bands.iter().find(|b| b.verdict.check == check).unwrap().clone();
    let failed: Vec<_> = bands.iter().filter(|b| !b.ok || b.share < 1.0).collect();
    assert!(failed.is_empty(), "{mode}: {failed:?}");
    let bearing = band("pulley-bearing-life");
    assert!(bearing.ok && bearing.low > 40000.0, "{mode}: {bearing:?}");
    let (slack, settling, radial) = (band("belt-slack"), band("settling"), band("gearbox-radial-load"));
    assert!(
        slack.low > 100.0 && settling.high < 0.7e-4 && radial.high < 1500.0,
        "{mode}: slackest {} N, settling {} m, radial {} N",
        slack.low,
        settling.high,
        radial.high
    );
    // The highest belt tension is judged too, against the estimate of 2140 N (a 5 mm pitch belt as stiff as Festo's), and passes in the worst case.
    let tension = band("belt-tension");
    assert!(tension.verdict.limit == 2140.0 && tension.ok && tension.share == 1.0, "{mode}: {tension:?}");
    println!(
        "{mode}: slackest {:.1} N, settling {:.2e} m, radial load {:.0} N, tightest {:.0} N",
        slack.low, settling.high, radial.high, tension.high
    );
}

/// The EXCT-30 passes `settling` at the worst corner of its bounds that `verdict_worst` finds, with the second input
/// shaper at 17.5 Hz (with the 25 Hz shaper alone it was 0.115 mm against 0.1 mm), and so does every other check but
/// the pulley shaft bearings' life: about 10000 h there, with the pretension, the bearings' load rating and fatigue load
/// limit, the lubricant's viscosity and the cleanliness at their bad bounds (see the test above). About 2400 cycles,
/// each with the warm start's passes (it judges the thermal checks too): 85 s on all 16 threads of this machine.
#[test]
fn the_exct_30_config_passes_settling_at_the_worst_corner_of_its_bounds() {
    use sim_components::analysis::{bounds, verdict_worst_and_bands};
    let cfg = sim_components::load_config(&machine_t_gantry::ENTRY, machine_t_gantry::EXCT_30_CONFIG).unwrap();
    // The bands from the runs the search starts with, which are those of `verdict_bands`: 2 n + 1 cycles fewer, here and
    // in the test above, whose judgement of them is made here.
    let (worst, bands, runs) = verdict_worst_and_bands(&machine_t_gantry::ENTRY, &cfg, &bounds(&cfg).unwrap()).unwrap();
    exct_30_passes_with_margin("one at a time", &bands);
    let failed: Vec<_> = worst
        .iter()
        .filter(|w| !w.verdict.ok && w.verdict.check != "pulley-bearing-life")
        .map(|w| &w.verdict)
        .collect();
    assert!(failed.is_empty(), "{failed:?}");
    let settling = worst.iter().find(|w| w.verdict.check == "settling").unwrap();
    assert!(settling.verdict.value < 0.1e-3, "{:?}", settling.verdict);
    println!("{runs} cycles; settling at worst {:.4e} m", settling.verdict.value);
    // The worst is never better than the bands of one value at a time: "dc-link-energy" was reported at 4.47 J (its
    // margin is 0 at every corner where the resistor takes over, and the corner with the least room came out) against
    // bands up to 33 J.
    for band in bands {
        let id = (band.verdict.check, band.verdict.motor);
        let w = &worst.iter().find(|w| (w.verdict.check, w.verdict.motor) == id).unwrap().verdict;
        let below = sim_components::analysis::AT_LEAST.contains(&id.0);
        assert!(
            if below { w.value <= band.low } else { w.value >= band.high } && (band.ok || !w.ok),
            "{id:?}: worst {w:?}, bands {band:?}"
        );
    }
}
