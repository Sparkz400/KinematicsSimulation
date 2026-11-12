//! The Bahr belt axes in `catalog/bahr/belt_axes.csv`: the file reads, and the numbers agree with each
//! other where geometry or the sheet's own logic ties them together. A value that landed in the wrong
//! column or row breaks one of these.

use std::collections::{HashMap, HashSet};
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

fn number(row: &HashMap<String, String>, column: &str) -> f64 {
    row[column].parse().unwrap_or_else(|_| panic!("{column} of {} {}: '{}'", row["series"], row["size"], row[column]))
}

#[test]
fn bahr_rows_are_unique_and_the_gantries_are_there() {
    let rows = csv("bahr/belt_axes.csv");
    assert!(rows.len() >= 60, "{} rows", rows.len());
    let mut seen = HashSet::new();
    for r in &rows {
        assert!(seen.insert((&r["series"], &r["size"])), "{} {} twice", r["series"], r["size"]);
        assert!(r["source_file"].ends_with(&format!("_{}.pdf", r["series"])), "{}", r["source_file"]);
        assert!(r["sheet"].starts_with("21.") && r["edition"].len() == 5, "{} {}", r["series"], r["size"]);
    }
    let count = |series: &str, kind: &str| rows.iter().filter(|r| r["series"] == series && r["kind"] == kind).count();
    assert_eq!((count("ELZI", "xz_gantry"), count("ELZU", "xy_gantry"), count("ELZ", "axis")), (3, 7, 8));
}

#[test]
fn bahr_belt_and_pulley_agree() {
    for r in &csv("bahr/belt_axes.csv") {
        let name = format!("{} {}", r["series"], r["size"]);
        // "5M25": HTD with 5 mm pitch, 25 mm wide.
        let (pitch, width) = r["belt"].split_once('M').unwrap_or_else(|| panic!("{name}: belt {}", r["belt"]));
        assert_eq!(pitch.parse::<f64>().unwrap(), number(r, "belt_pitch_mm"), "{name}: pitch");
        assert_eq!(width.parse::<f64>().unwrap(), number(r, "belt_width_mm"), "{name}: width");
        // One turn of the pulley feeds one tooth pitch per tooth.
        assert_eq!(number(r, "feed_per_rev_mm"), number(r, "pulley_teeth") * number(r, "belt_pitch_mm"), "{name}: feed");
    }
}

#[test]
fn bahr_forces_loads_and_weights_are_plausible() {
    for r in &csv("bahr/belt_axes.csv") {
        let name = format!("{} {}", r["series"], r["size"]);
        let n = |column: &str| number(r, column);
        assert!(n("feed_force_peak_n") >= n("feed_force_continuous_n"), "{name}: peak feed force");
        // The belt pull is the load limit along the travel. The sheets round it differently in places
        // (900 N beside 894 N; ELZU 100: 3600 N beside 4000 N), hence the wide tolerance.
        assert!(close(n("feed_force_continuous_n"), n("fx_static_n"), 0.11), "{name}: feed force and Fx");
        for load in ["fx", "fy", "fz", "mx", "my", "mz"] {
            let unit = if load.starts_with('f') { "n" } else { "nm" };
            let (fixed, moving) = (format!("{load}_static_{unit}"), format!("{load}_dynamic_{unit}"));
            // The X/Z gantry has no Fy.
            if r[&fixed].is_empty() && r[&moving].is_empty() && load == "fy" && r["kind"] == "xz_gantry" {
                continue;
            }
            assert!(n(&moving) > 0.0 && n(&moving) <= n(&fixed), "{name}: {load}");
        }
        assert!(n("no_load_torque_nm") > 0.0 && n("no_load_torque_nm") < 5.0, "{name}: no-load torque");
        // The no-load torque is small beside what the belt pull needs at the pulley.
        let pull_torque = n("feed_force_continuous_n") * n("feed_per_rev_mm") / 1000.0 / std::f64::consts::TAU;
        assert!(n("no_load_torque_nm") < 0.5 * pull_torque, "{name}: no-load torque beside {pull_torque:.1} N m");
        assert!((1.0..=10.0).contains(&n("max_speed_m_s")), "{name}: speed");
        assert!(n("base_weight_kg") > n("weight_per_100_mm_kg") && n("weight_per_100_mm_kg") > 0.0, "{name}: weights");
    }
}

#[test]
fn bahr_gantries_state_their_moved_masses_and_lengths() {
    for r in &csv("bahr/belt_axes.csv") {
        let name = format!("{} {}", r["series"], r["size"]);
        let n = |column: &str| number(r, column);
        // The second axis of each gantry, the one that rides on the first.
        let second = match r["kind"].as_str() {
            "xz_gantry" => "z",
            "xy_gantry" => "y",
            "axis" => {
                for column in ["moved_mass_x_kg", "moved_mass_y_kg", "moved_mass_z_kg", "weight_per_100_mm_z_kg"] {
                    assert!(r[column].is_empty(), "{name}: {column} on a single axis");
                }
                continue;
            }
            other => panic!("{name}: kind {other}"),
        };
        // The first axis moves more mass than the one riding on it, and the whole gantry weighs more than what moves.
        let (carried, riding) = (n("moved_mass_x_kg"), n(&format!("moved_mass_{second}_kg")));
        assert!(riding > 0.0 && carried > riding && n("base_weight_kg") > carried, "{name}: moved masses");
        assert!(n("max_length_x_mm") >= n(&format!("max_length_{second}_mm")), "{name}: lengths");
        // Only the X/Z gantry sheets print "2 x" before the no-load torque.
        assert_eq!(r["no_load_torque_times"] == "2", r["kind"] == "xz_gantry", "{name}: no-load torque times");
        if second == "z" {
            assert!(n("weight_per_100_mm_kg") > n("weight_per_100_mm_z_kg"), "{name}: the double X guide is heavier");
        }
    }
}
