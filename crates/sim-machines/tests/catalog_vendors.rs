//! Other makers' motors and drives (`catalog/<vendor>/motors_*.csv`, `drives_*.csv`) and the S210 with its new 6SL5310
//! converters: the supplies that name them, their values in a config, their pairing and the search across makers.

use sim_components::catalog::{SUPPLIES, apply_drive, apply_motor, motor_articles_for, paired_drive, vendor_of};
use sim_core::Config;

fn exct_30() -> Config {
    Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap()
}

/// D 32 lists a 6SL5310 converter beside each 1FK2 in the columns `paired_drive_new_*` of `motors_s210.csv`: the
/// 1FK2102-0AG has the 1 AC 6SL5310-1BB10-1CF0 at 230 V, the 1FK2103-2AH the 3 AC 6SL5310-1BE10-4DF0 at 400 V. The
/// motor's own values are the same as with the old converters.
#[test]
fn the_new_s210_converters_are_paired_as_d32_lists_them() {
    assert_eq!(paired_drive("1FK2102-0AG", "s210new-230v"), "6SL5310-1BB10-1CF0");
    assert_eq!(paired_drive("1FK2103-2AH", "s210new-400v"), "6SL5310-1BE10-4DF0");
    assert_eq!(paired_drive("1FK2103-2AH", "400v"), "6SL3210-5HE10-4UF0");
    for (old, new) in [("230v", "s210new-230v"), ("400v", "s210new-400v")] {
        assert_eq!(motor_articles_for(old), motor_articles_for(new));
        let (mut a, mut b) = (exct_30(), exct_30());
        apply_motor(&mut a, "1FK2103-2AH", "400v").unwrap();
        apply_motor(&mut b, "1FK2103-2AH", "s210new-400v").unwrap();
        assert_eq!(a.to_text().unwrap(), b.to_text().unwrap(), "{old}");
    }
    // converters_s210.csv: 1.2 A rated, 4.2 A maximum; the base load current is 0.85 of the rated, 1.02 A [estimate].
    let mut cfg = exct_30();
    apply_drive(&mut cfg, "6SL5310-1BE10-4DF0", "s210new-400v").unwrap();
    assert_eq!(cfg.f64("ratings.drive_rated_current_a").unwrap(), 1.2);
    assert_eq!(cfg.f64("ratings.drive_max_current_a").unwrap(), 4.2);
    assert!((cfg.f64("ratings.drive_base_load_current_a").unwrap() - 1.02).abs() < 1e-12);
}

/// Every supply names at least one motor, and every maker's (vendor, line voltage) in its motor tables has its supply
/// "<vendor>-<volts>v" in `SUPPLIES`, so that no table is left out of the search.
#[test]
fn every_supply_has_motors_and_every_vendor_table_a_supply() {
    for supply in SUPPLIES {
        assert!(!motor_articles_for(supply).is_empty(), "{supply}");
    }
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog");
    for maker in std::fs::read_dir(&root).unwrap().flatten().filter(|e| e.path().is_dir()) {
        let name = maker.file_name().to_string_lossy().to_string();
        for file in std::fs::read_dir(maker.path()).unwrap().flatten() {
            let file_name = file.file_name().to_string_lossy().to_string();
            if name == "siemens" || !file_name.starts_with("motors_") || !file_name.ends_with(".csv") {
                continue;
            }
            let text = std::fs::read_to_string(file.path()).unwrap();
            let mut lines = text.lines();
            let header: Vec<&str> = lines.next().unwrap().split(',').collect();
            let at = |column: &str| header.iter().position(|c| *c == column).unwrap();
            for line in lines {
                let fields: Vec<&str> = line.split(',').collect();
                let volts: f64 = fields[at("line_voltage_v")].parse().unwrap();
                let supply = format!("{}-{volts}v", fields[at("vendor")]);
                assert!(SUPPLIES.contains(&supply.as_str()), "{file_name}: {supply}");
                assert_eq!(vendor_of(&supply), name);
            }
        }
    }
    assert_eq!(vendor_of("400v"), "siemens");
    assert_eq!(vendor_of("s210new-400v"), "siemens");
}

/// B&R 8LSA23.ee060 at "br-400v", its row of `br/motors_8ls.csv`: M0 0.2 N m at I0 0.27 A, so Kt = 0.2 / 0.27 =
/// 0.7407 N m/A; rated 0.23 A; 780 s thermal; 4 pole pairs. The bus is 1.35 × 400 V = 540 V. Winding 159 Ω and 0.165 H
/// terminal to terminal, half of it per phase of the star, three phases in the model's one winding: 238.5 Ω and
/// 0.2475 H; the current loop keeps the config's time constant 0.010 H / 40 V/A = 0.25 ms, so Kp = 0.2475 / 0.00025 =
/// 990 V/A and Tn = 0.2475 / 238.5 = 1.0377 ms. B&R pairs it with the ACOPOS P3 8EI2X2HWS10.XXXX-1: 2.2 A rated, 6 A
/// peak, 470 µF, an internal resistor of 100 W continuous and 7 kW peak.
#[test]
fn a_br_motor_and_drive_by_name_bring_their_manuals_values() {
    let (article, supply) = ("8LSA23.ee060", "br-400v");
    let mut cfg = exct_30();
    apply_motor(&mut cfg, article, supply).unwrap();
    let close = |cfg: &Config, path: &str, want: f64| {
        let got = cfg.f64(path).unwrap();
        assert!((got - want).abs() <= 1e-9 * want.abs(), "{path}: {got} against {want}");
    };
    close(&cfg, "motor.torque_constant_nm_a", 0.2 / 0.27);
    close(&cfg, "motor.rated_current_a", 0.23);
    close(&cfg, "motor.thermal_time_constant_s", 780.0);
    close(&cfg, "motor.rotor_inertia_kgm2", 7e-6);
    close(&cfg, "drive.bus_voltage_v", 540.0);
    close(&cfg, "motor.resistance_ohm", 238.5);
    close(&cfg, "motor.inductance_h", 0.2475);
    close(&cfg, "drive.current_kp_v_a", 990.0);
    close(&cfg, "drive.current_tn_s", 0.2475 / 238.5);
    let drive = paired_drive(article, supply);
    assert_eq!(drive, "8EI2X2HWS10.XXXX-1");
    apply_drive(&mut cfg, drive, supply).unwrap();
    close(&cfg, "ratings.drive_rated_current_a", 2.2);
    close(&cfg, "ratings.drive_max_current_a", 6.0);
    close(&cfg, "ratings.drive_base_load_current_a", 0.85 * 2.2);
    close(&cfg, "ratings.dc_link_capacitance_uf", 470.0);
    close(&cfg, "ratings.braking_resistor_continuous_w", 100.0);
    close(&cfg, "ratings.braking_resistor_peak_w", 7000.0);
}

/// Beckhoff publishes no pairing: the AM8041-0D (I0 1.65 A at 400 V) gets the smallest Beckhoff drive at 400 V whose
/// rated current carries 1.65 A, a single-axis one before a two-axis one: not the AX5101 (1.5 A), the AX5103 (3 A)
/// before the AX5203 (3 A, two axes). Kollmorgen's AKD-x00307 at 400 V brings its regen circuit as the manual prints
/// it: switch-on at 760 V, trip at 840 V, 22.1 kW peak, 80 W continuous, 235 µF.
#[test]
fn a_motor_without_the_makers_pairing_gets_the_smallest_drive_that_carries_its_stall_current() {
    assert_eq!(paired_drive("AM8041-0D", "beckhoff-400v"), "AX5103");
    let mut cfg = exct_30();
    apply_drive(&mut cfg, "AKD-x00307", "kollmorgen-400v").unwrap();
    for (path, want) in [
        ("ratings.braking_threshold_v", 760.0),
        ("ratings.dc_link_overvoltage_v", 840.0),
        ("ratings.braking_resistor_peak_w", 22100.0),
        ("ratings.braking_resistor_continuous_w", 80.0),
        ("ratings.dc_link_capacitance_uf", 235.0),
        ("ratings.drive_rated_current_a", 3.0),
        ("ratings.drive_max_current_a", 9.0),
    ] {
        assert_eq!(cfg.f64(path).unwrap(), want, "{path}");
    }
}

/// Mitsubishi prints no peak current of its amplifiers; the MR-J5-100G4 it pairs with the HK-KT634W takes the motor's
/// 9.1 A peak [estimate], put in by the motor before it.
#[test]
fn an_amplifier_without_a_printed_peak_takes_the_paired_motors() {
    let mut cfg = exct_30();
    cfg.set_text("parts.motor = \"HK-KT634W\"").unwrap();
    cfg.set_text("parts.motor_supply = \"mitsubishi-400v\"").unwrap();
    cfg.set_text("parts.drive = \"MR-J5-100G4\"").unwrap();
    let cfg = sim_components::catalog::resolve(&machine_t_gantry::ENTRY, &cfg).unwrap();
    assert_eq!(cfg.f64("ratings.drive_max_current_a").unwrap(), 9.1);
    assert_eq!(cfg.f64("ratings.drive_rated_current_a").unwrap(), 2.8);
}

/// The search across makers ranks the candidates of every supply together and says whose each is: four motors of four
/// makers on the EXCT-30's cycle, each passing there with the drive its maker pairs (`sim-cli search --vendor all`).
#[test]
fn the_search_across_makers_ranks_their_motors_together() {
    let wanted = |a: &str| ["HK-KT634W", "SGM7A-06A", "AKM24C", "8LSAA3.ee030"].contains(&a);
    let found = sim_components::catalog::search_supplies(
        &machine_t_gantry::ENTRY,
        &exct_30(),
        SUPPLIES,
        &wanted,
        None,
        false,
        None,
    )
    .unwrap();
    let mut makers: Vec<&str> = found.iter().filter(|c| c.passes()).map(|c| vendor_of(c.supply)).collect();
    makers.sort();
    makers.dedup();
    assert_eq!(makers, ["br", "kollmorgen", "mitsubishi", "yaskawa"], "{found:?}");
    assert!(found.windows(2).all(|w| w[0].passes() >= w[1].passes()));
}
