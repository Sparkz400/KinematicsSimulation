//! The drive parameter list: the config's drive settings as the SINAMICS S120 servo and S210 take them, against hand
//! calculations from the tables' conversions (`catalog/siemens/parameters_s120.csv`, `parameters_s210.csv`).

use sim_components::parameters::{Basis, Error, Family, Row, SETTINGS, list};
use sim_core::Config;
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// A Motor Module of an S120 line-up and a 400 V converter of the S210.
const S120_DRIVE: &str = "6SL3120-1TE15-0AD0";
const S210_DRIVE: &str = "6SL3210-5HE11-5UF0";

fn close(a: f64, b: f64, tolerance: f64) -> bool {
    (a - b).abs() <= tolerance * b.abs()
}

/// The EXCT-30 (two motors, "a" and "b") on `drive`, tuned the way a commissioning would leave it. Motor a has its own
/// numbers, which the catalogue parts and the retuning of the shared ones leave as they are: Kv 100 1/s, Kp 0.3 N m s/rad,
/// Tn 20 ms, current Tn 4 ms, a notch of 100 Hz and 30 Hz, a speed filter of 1 ms, 10 N m, 628 rad/s with the rated
/// speed 314 rad/s, ramps of 1 s and 0.1 s. Motor b has a gain of 0.5. The notch, the monitoring and the filters are on.
fn tuned(drive: &str) -> Config {
    let mut text = machine_t_gantry::EXCT_30_CONFIG.to_string();
    text.push_str(
        "\n[drive.a]\nkv_1_s = 100.0\nvelocity_kp_nm_s_rad = 0.3\nvelocity_tn_s = 0.02\ncurrent_tn_s = 0.004\n\
         notch_frequency_hz = 100.0\nnotch_bandwidth_hz = 30.0\nvelocity_filter_s = 0.001\npeak_torque_nm = 10.0\n\
         max_speed_rad_s = 628.0\nrated_speed_rad_s = 314.0\noff1_ramp_s = 1.0\noff3_ramp_s = 0.1\n\
         \n[drive.b]\nvelocity_kp_nm_s_rad = 0.5\n",
    );
    let mut cfg = Config::parse(&text).unwrap();
    for set in ["effects.notch_filter = true", "effects.drive_monitoring = true", "effects.velocity_filter = true"] {
        cfg.set_text(set).unwrap();
    }
    cfg.set_text(&format!("parts.drive = {drive:?}")).unwrap();
    cfg
}

fn find<'a>(rows: &'a [Row], motor: &str, key: &str, parameter: &str) -> &'a Row {
    rows.iter()
        .find(|r| r.motor == motor && r.key == key && r.parameter == parameter)
        .unwrap_or_else(|| panic!("no row {motor} {key} {parameter}"))
}

#[track_caller]
fn converts(rows: &[Row], key: &str, parameter: &str, value: f64, converted: f64, unit: &str) {
    let r = find(rows, "a", key, parameter);
    assert!(close(r.value, value, 1e-9), "{key}: {} against {value}", r.value);
    let got = r.parameter_value.unwrap();
    assert!(close(got, converted, 1e-4) || (converted == 0.0 && got == 0.0), "{parameter}: {got} against {converted}");
    assert_eq!(r.parameter_unit, unit, "{parameter}");
}

/// On an S120 Motor Module each setting arrives in its parameter's unit by the hand-worked conversion: Kp (0.3 N m s/rad)
/// unchanged in Nms/rad, Tn 0.02 s as 20 ms, Kv 100 1/s as 100 / 16.667 = 6.0 (1000 rpm), the current Tn 4 ms, the speed
/// filter 1 ms, the torque limits +10 and -10 N m, 628 rad/s as 628 x 60 / (2 pi) = 5996.96 rpm, and the ramps from the
/// rated speed (314 rad/s) to the maximum speed (628 rad/s): twice as long, 2 s and 0.2 s. Without the table row or the
/// factor of any of them its line fails.
#[test]
fn an_s120_motor_module_gets_each_setting_in_its_parameters_unit() {
    let rows = list(&machine_t_gantry::ENTRY, &tuned(S120_DRIVE), None).unwrap();
    converts(&rows, "velocity_kp_nm_s_rad", "p1460", 0.3, 0.3, "Nms/rad");
    assert_eq!(find(&rows, "a", "velocity_kp_nm_s_rad", "p1460").unit, "N m s/rad");
    assert_eq!(find(&rows, "a", "velocity_kp_nm_s_rad", "p1460").basis, Basis::Printed);
    converts(&rows, "velocity_tn_s", "p1462", 0.02, 20.0, "ms");
    converts(&rows, "kv_1_s", "p2538", 100.0, 100.0 / 16.667, "1000 rpm");
    assert_eq!(find(&rows, "a", "kv_1_s", "p2538").basis, Basis::Derived);
    converts(&rows, "current_tn_s", "p1717", 0.004, 4.0, "ms");
    converts(&rows, "velocity_filter_s", "p1441", 0.001, 1.0, "ms");
    converts(&rows, "peak_torque_nm", "p1520", 10.0, 10.0, "Nm");
    converts(&rows, "peak_torque_nm", "p1521", 10.0, -10.0, "Nm");
    converts(&rows, "max_speed_rad_s", "p1082", 628.0, 628.0 * 60.0 / std::f64::consts::TAU, "rpm");
    converts(&rows, "off1_ramp_s", "p1121", 1.0, 2.0, "s");
    converts(&rows, "off3_ramp_s", "p1135", 0.1, 0.2, "s");
}

/// The notch of 100 Hz and a bandwidth of 30 Hz (-3 dB) is a general second order filter (p1657 = 2) with the same
/// natural frequency in numerator and denominator (p1658, p1660 = 100 Hz), the denominator damping from the manual's
/// f_3dB = 2 D f_notch: D = 30 / (2 x 100) = 0.15 (p1659), and the numerator damping 0 (p1661). The row of the damping
/// is the one that needs the rule, not a factor: with the factor alone it would read 30.
#[test]
fn the_notch_becomes_a_current_setpoint_filter_with_the_damping_of_its_bandwidth() {
    let rows = list(&machine_t_gantry::ENTRY, &tuned(S120_DRIVE), None).unwrap();
    converts(&rows, "notch_frequency_hz", "p1657", 100.0, 2.0, "-");
    converts(&rows, "notch_frequency_hz", "p1658", 100.0, 100.0, "Hz");
    converts(&rows, "notch_frequency_hz", "p1660", 100.0, 100.0, "Hz");
    converts(&rows, "notch_bandwidth_hz", "p1659", 30.0, 0.15, "-");
    converts(&rows, "notch_bandwidth_hz", "p1661", 30.0, 0.0, "-");
    assert_eq!(find(&rows, "a", "notch_bandwidth_hz", "p1659").basis, Basis::Derived);
}

/// A setting whose effect is off in the config was not used by the simulation: no row for the notch, the speed filter or
/// the ramps then, as the drive would be given numbers nobody tuned.
#[test]
fn a_setting_of_an_effect_that_is_off_has_no_row() {
    let mut cfg = tuned(S120_DRIVE);
    for off in ["effects.notch_filter = false", "effects.drive_monitoring = false", "effects.velocity_filter = false"] {
        cfg.set_text(off).unwrap();
    }
    let rows = list(&machine_t_gantry::ENTRY, &cfg, None).unwrap();
    for key in ["notch_frequency_hz", "notch_bandwidth_hz", "off1_ramp_s", "blocked_time_s", "velocity_filter_s"] {
        assert!(rows.iter().all(|r| r.key != key), "{key}");
    }
    assert!(rows.iter().any(|r| r.key == "velocity_kp_nm_s_rad"));
}

/// The S210 has no parameter for the current controller, for Kv (its position controller is in the controller above), for
/// the notch or for the position precontrol: those settings are rows without a parameter, and none of the S120's
/// p1717, p2538, p1658 comes up. Its speed controller and limits are rows (p1460, p1462, p1520, p1082, p1121).
#[test]
fn an_s210_has_no_parameter_for_the_current_controller_kv_or_the_notch() {
    let rows = list(&machine_t_gantry::ENTRY, &tuned(S210_DRIVE), None).unwrap();
    for key in
        ["current_kp_v_a", "current_tn_s", "kv_1_s", "notch_frequency_hz", "notch_bandwidth_hz", "precontrol_balance_s"]
    {
        let r = find(&rows, "a", key, "");
        assert_eq!((r.basis, r.parameter_value, r.name.as_str()), (Basis::None, None, ""), "{key}");
    }
    for parameter in ["p1715", "p1717", "p2538", "p1658", "p1659", "p2536"] {
        assert!(rows.iter().all(|r| r.parameter != parameter), "{parameter}");
    }
    converts(&rows, "velocity_kp_nm_s_rad", "p1460", 0.3, 0.3, "Nms/rad");
    converts(&rows, "velocity_tn_s", "p1462", 0.02, 20.0, "ms");
    converts(&rows, "max_speed_rad_s", "p1082", 628.0, 628.0 * 60.0 / std::f64::consts::TAU, "rpm");
    converts(&rows, "off1_ramp_s", "p1121", 1.0, 2.0, "s");
}

/// On an S120 the current controller's gain has no parameter either: the simulator's Kp is in the units of its one
/// winding, three times a phase, which Siemens does not print for p1715; its Tn = L/R does carry over (p1717).
#[test]
fn the_current_gain_has_no_parameter_on_an_s120() {
    let rows = list(&machine_t_gantry::ENTRY, &tuned(S120_DRIVE), None).unwrap();
    let r = find(&rows, "a", "current_kp_v_a", "");
    assert_eq!((r.basis, r.parameter_value), (Basis::None, None));
    assert!(rows.iter().all(|r| r.parameter != "p1715"));
}

/// A gain of its own shows for its motor: motor a's 0.3 and motor b's 0.5 N m s/rad are two rows of p1460, one per motor,
/// each with its own converted value (a shared list would show the same number twice).
#[test]
fn a_gain_of_one_motor_shows_for_that_motor() {
    let rows = list(&machine_t_gantry::ENTRY, &tuned(S120_DRIVE), None).unwrap();
    let kp: Vec<(&str, f64)> = rows
        .iter()
        .filter(|r| r.parameter == "p1460")
        .map(|r| (r.motor.as_str(), r.parameter_value.unwrap()))
        .collect();
    assert_eq!(kp, [("a", 0.3), ("b", 0.5)]);
}

/// A config without a drive from the catalogue is refused (the parts name none), as is a drive that is neither an S120
/// Motor Module nor an S210 converter of the Operating Instructions (the S210 New 6SL5310); naming the family by hand
/// makes the list for the first.
#[test]
fn a_config_without_a_catalogue_drive_is_refused() {
    let mut cfg = tuned(S120_DRIVE);
    cfg.set_text("parts.drive = \"\"").unwrap();
    assert_eq!(list(&machine_t_gantry::ENTRY, &cfg, None).err(), Some(Error::NoDrive("a".into())));
    let by_hand = list(&machine_t_gantry::ENTRY, &cfg, Some(Family::S210)).unwrap();
    assert!(by_hand.iter().any(|r| r.parameter == "p1460"));
    cfg.set_text("parts.drive = \"6SL5310-1BE10-8DF0\"").unwrap();
    assert_eq!(
        list(&machine_t_gantry::ENTRY, &cfg, None).err(),
        Some(Error::UnknownDrive("6SL5310-1BE10-8DF0".into()))
    );
}

fn table(file: &str) -> Vec<HashMap<String, String>> {
    let text = std::fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog/siemens").join(file))
        .unwrap();
    let mut lines = text.lines().filter(|l| !l.trim().is_empty());
    let header: Vec<String> = lines.next().unwrap().split(',').map(String::from).collect();
    lines.map(|l| header.iter().cloned().zip(l.split(',').map(String::from)).collect()).collect()
}

fn config_files(dir: &Path, out: &mut Vec<PathBuf>) {
    for entry in std::fs::read_dir(dir).unwrap().flatten() {
        let path = entry.path();
        if path.is_dir() && path.file_name().is_some_and(|n| n != "target") {
            config_files(&path, out);
        } else if path.extension().is_some_and(|e| e == "toml") && path.file_name().is_some_and(|n| n != "Cargo.toml") {
            out.push(path);
        }
    }
}

/// Every key of the two tables is a [drive] key a shipped config has (so that a renamed key breaks this test, not the list
/// quietly), is one the list reads (`SETTINGS`), and every row says where it comes from: a source of the table's metadata,
/// printed or derived, a parameter number and its name as printed.
#[test]
fn every_table_key_is_a_shipped_drive_key_and_every_row_cites_its_source() {
    let mut files = Vec::new();
    config_files(&Path::new(env!("CARGO_MANIFEST_DIR")).join(".."), &mut files);
    let configs: Vec<String> = files.iter().map(|f| std::fs::read_to_string(f).unwrap()).collect();
    for file in ["parameters_s120", "parameters_s210"] {
        let meta = std::fs::read_to_string(
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog/siemens").join(format!("{file}.meta.toml")),
        )
        .unwrap();
        let sources: Vec<&str> =
            meta.lines().filter_map(|l| l.strip_prefix("id = \"")).filter_map(|l| l.strip_suffix('"')).collect();
        let rows = table(&format!("{file}.csv"));
        assert!(rows.len() >= 8, "{file}");
        for r in &rows {
            let key = &r["config_key"];
            assert!(SETTINGS.iter().any(|s| s.0 == key), "{file}: {key} is not read by the list");
            let line = format!("\n{key} =");
            assert!(configs.iter().any(|c| c.contains(&line)), "{file}: no shipped config has {key}");
            assert!(sources.contains(&r["source"].as_str()), "{file} {key} {}: source {}", r["parameter"], r["source"]);
            assert!(["printed", "derived"].contains(&r["basis"].as_str()), "{file} {key}");
            assert!(r["parameter"].starts_with('p') && !r["name"].is_empty() && !r["unit"].is_empty(), "{file} {key}");
        }
    }
}
