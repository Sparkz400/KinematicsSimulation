//! The 1FK2 files of the component catalogue, from the S120 configuration manual of the motors: the
//! numbers agree with each other and with the S210 catalogue, which another script read from another document.

use std::collections::HashMap;
use std::f64::consts::PI;
use std::fs;
use std::path::Path;

fn close(a: f64, b: f64, tolerance: f64) -> bool {
    (a - b).abs() <= tolerance * b.abs()
}

/// The rows of a catalogue CSV as (column name -> text) lookups; lines starting with '#' are notes.
fn csv(path: &str) -> Vec<HashMap<String, String>> {
    let text = fs::read_to_string(Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog").join(path)).unwrap();
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

#[test]
fn siemens_1fk2_data_agrees_between_the_s120_manual_and_the_s210_catalogue() {
    let (manual, catalogue) = (csv("siemens/motors_1fk2_s120.csv"), csv("siemens/motors_s210.csv"));
    assert_eq!(manual.len(), 33);
    let mut shared = 0;
    for m in &manual {
        let name = &m["article"];
        let number = |row: &HashMap<String, String>, column: &str| -> f64 {
            row[column].parse().unwrap_or_else(|_| panic!("{column} of {name}"))
        };
        assert!((5.0..=120.0).contains(&number(m, "thermal_time_constant_min")), "{name}: thermal time constant");
        let power = number(m, "rated_torque_nm") * number(m, "rated_speed_rpm") * PI / 30e3;
        assert!((number(m, "rated_power_kw") - power).abs() <= 0.08 * power + 0.015, "{name}: rated power");
        assert!(number(m, "rated_torque_nm") <= number(m, "static_torque_nm"), "{name}: rated torque above static torque");
        let Some(c) = catalogue.iter().find(|c| &c["article"] == name) else { continue };
        shared += 1;
        for (column, tolerance) in [("static_torque_nm", 0.05), ("stall_current_a", 0.05), ("max_torque_nm", 0.05), ("max_current_a", 0.05), ("rotor_inertia_kgcm2", 0.06), ("mass_kg", 0.06)] {
            // The manual prints the inertia of the 1FK2308-3AB as "1,60,1"; the file leaves it empty.
            if name == "1FK2308-3AB" && column == "rotor_inertia_kgcm2" {
                assert!(m[column].is_empty(), "{name}: {column} is readable now, check it");
                continue;
            }
            assert!(close(number(m, column), number(c, column), tolerance), "{name}: {column} {} against {}", m[column], c[column]);
        }
    }
    assert!(shared >= 28, "only {shared} motors in both files");
}

#[test]
fn siemens_1fk2_dynamic_braking_fits_the_motors() {
    let (braking, motors) = (csv("siemens/dynamic_braking_1fk2.csv"), csv("siemens/motors_1fk2_s120.csv"));
    assert_eq!(braking.len(), 33);
    for b in &braking {
        let name = &b["article"];
        let number = |column: &str| -> f64 { b[column].parse().unwrap_or_else(|_| panic!("{column} of {name}")) };
        assert!(motors.iter().any(|m| &m["article"] == name), "{name}: not in the motor file");
        assert!(number("optimum_resistance_ohm") > 0.0, "{name}: resistor");
        // The optimum resistor is the one that brakes harder than the bare short circuit, and a peak is above an average.
        let (without, with, max) = (number("braking_torque_rms_nm"), number("braking_torque_rms_with_resistor_nm"), number("max_braking_torque_nm"));
        assert!(without <= with && with <= max, "{name}: torques out of order");
        // A resistor in the circuit lowers the current. For the 1FK2306-2AC the manual prints 6.5 A with
        // and 6.1 A without. The file keeps what is printed, so this one row is not checked.
        let (without, with) = (number("braking_current_rms_a"), number("braking_current_rms_with_resistor_a"));
        assert!(name == "1FK2306-2AC" || (with <= without && with > 0.0), "{name}: currents out of order");
    }
}
