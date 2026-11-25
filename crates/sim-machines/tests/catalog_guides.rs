//! Bosch Rexroth's and THK's profile rail guides beside HIWIN's (`catalog/<maker>/linear_guides.csv`): the tables'
//! bases, their names in a config and in the guide search, and the life of a set against a figure worked out by hand.

use sim_components::catalog::{GUIDE_STATIC_SAFETY, apply_axis, mech_axes};
use sim_core::Config;
use std::collections::HashMap;
use std::path::Path;

fn close(a: f64, b: f64, tolerance: f64) -> bool {
    (a - b).abs() <= tolerance * b.abs()
}

fn linear_axis() -> Config {
    machine_joint_kit::LINEAR_MOTOR.config()
}

/// The rows of `catalog/<maker>/linear_guides.csv` by column name.
fn table(maker: &str) -> Vec<HashMap<String, String>> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog").join(maker).join("linear_guides.csv");
    let text = std::fs::read_to_string(path).unwrap();
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().unwrap().split(',').collect();
    lines
        .map(|l| {
            let fields: Vec<&str> = l.split(',').collect();
            assert_eq!(fields.len(), header.len(), "{l}");
            header.iter().zip(fields).map(|(h, f)| (h.to_string(), f.to_string())).collect()
        })
        .collect()
}

fn n(row: &HashMap<String, String>, column: &str) -> f64 {
    row[column].parse().unwrap_or_else(|_| panic!("{} {column}", row["part_number"]))
}

/// What the Python checks hold the HIWIN table to, for the two new ones: every row on its maker's basis (Rexroth's C
/// is for 100 km, THK's for 50 km, stored as printed), C0 above C, a size's rail width, Rexroth's one ML0 for pitch and
/// yaw, and no part number in two tables (a name finds one carriage).
#[test]
fn the_rexroth_and_thk_tables_state_their_basis_and_share_no_name_with_another_maker() {
    let (hiwin, rexroth, thk) = (table("hiwin"), table("rexroth"), table("thk"));
    assert_eq!((rexroth.len(), thk.len()), (61, 59));
    for r in &rexroth {
        assert_eq!(n(r, "dynamic_load_rating_distance_m"), 100_000.0, "{}", r["part_number"]);
        assert_eq!(n(r, "static_moment_y_nm"), n(r, "static_moment_z_nm"), "{}", r["part_number"]);
        assert!(n(r, "dynamic_moment_x_nm") <= n(r, "static_moment_x_nm"));
    }
    for r in &thk {
        assert_eq!(n(r, "dynamic_load_rating_distance_m"), 50_000.0, "{}", r["part_number"]);
        assert!(!r.contains_key("dynamic_moment_x_nm"), "THK prints no dynamic moments");
    }
    for r in rexroth.iter().chain(&thk) {
        let (c, c0) = (n(r, "dynamic_load_rating_n"), n(r, "static_load_rating_n"));
        assert!(c0 > c && c0 < 2.2 * c, "{}", r["part_number"]);
        let size = n(r, "size") / 1000.0;
        assert!((0.8 * size..=1.25 * size).contains(&n(r, "rail_width_m")), "{}", r["part_number"]);
    }
    let mut names: Vec<&str> = hiwin.iter().chain(&rexroth).chain(&thk).map(|r| r["part_number"].as_str()).collect();
    let all = names.len();
    names.sort_unstable();
    names.dedup();
    assert_eq!(names.len(), all, "a part number is in two tables");
}

/// A config names a Rexroth or a THK carriage like a HIWIN one ("<article>x<carriages>"), and the guide search offers
/// them. Rexroth FNS25 (page 51): C 28600 N for 100 km, C0 35900 N, 0.65 kg; THK SHS25C (A1-99): C 31700 N for 50 km,
/// C0 52400 N, 0.72 kg. Four carriages on the linear motor axis, which has 5 kg carriage mass.
#[test]
fn a_config_can_name_a_rexroth_or_a_thk_carriage_and_the_search_offers_them() {
    let mut rex = linear_axis();
    apply_axis(&mut rex, "FNS25x4", None).unwrap();
    // C for 50 km = 28600 x 2^(1/3) = 36033.74 N a carriage; C0 / 2 = 17950 N; the mass 5 + 4 x 0.65 = 7.6 kg.
    assert!(close(rex.f64("ratings.guide_dynamic_load_rating_n").unwrap(), 4.0 * 36033.74, 1e-6));
    assert!(close(rex.f64("ratings.guide_max_load_n").unwrap(), 4.0 * 35900.0 / GUIDE_STATIC_SAFETY, 1e-12));
    assert!(close(rex.f64("mass.carriage_kg").unwrap(), 7.6, 1e-12));
    let mut thk = linear_axis();
    apply_axis(&mut thk, "SHS25Cx4", None).unwrap();
    assert!(close(thk.f64("ratings.guide_dynamic_load_rating_n").unwrap(), 4.0 * 31700.0, 1e-12));
    assert!(close(thk.f64("ratings.guide_max_load_n").unwrap(), 4.0 * 52400.0 / GUIDE_STATIC_SAFETY, 1e-12));
    assert!(close(thk.f64("mass.carriage_kg").unwrap(), 5.0 + 4.0 * 0.72, 1e-12));
    // A name in no table is refused, as before.
    assert!(apply_axis(&mut linear_axis(), "FNS26x4", None).is_err());
    // The search: the two smallest of each block type, 2.6 kN of load against every smallest carriage's C0 / 2.
    let names: Vec<String> = mech_axes(&linear_axis()).into_iter().map(|(n, ..)| n).collect();
    for expected in ["FNS15x4", "SHS15Cx4", "HSR15Cx4", "HGH15CAx4"] {
        assert!(names.iter().any(|n| n == expected), "{expected} not in {names:?}");
    }
}

/// A seal drag the document does not print is no zero: HIWIN's HGH25CA adds 4 x 6.28 N = 25.12 N to the config's 20 N sliding
/// friction (the seals of its carriages), Rexroth's FNS25 and THK's SHS25C, whose rows have none, leave the 20 N as the
/// config has it.
#[test]
fn a_guide_row_without_seal_drag_leaves_the_configs_friction_as_it_is() {
    let mut hiwin = linear_axis();
    apply_axis(&mut hiwin, "HGH25CAx4", None).unwrap();
    assert!(close(hiwin.f64("slide.friction_coulomb_n").unwrap(), 20.0 + 25.12, 1e-12));
    for name in ["FNS25x4", "SHS25Cx4"] {
        let mut cfg = linear_axis();
        apply_axis(&mut cfg, name, None).unwrap();
        assert_eq!(cfg.f64("slide.friction_coulomb_n").unwrap(), 20.0, "{name}");
        assert_eq!(cfg.f64("slide.friction_breakaway_n").unwrap(), 30.0, "{name}");
    }
}

/// Rexroth states C for 100 km, HIWIN and THK for 50 km: the same bearing is rated 1.26 times higher by HIWIN, and gives
/// the same life. Hand: FNS25, four carriages, a mean load of 20000 N on the set.
///   Rexroth's own formula: L10 = (C / P)^3 x 100 km = (4 x 28600 / 20000)^3 x 100 = 5.72^3 x 100 = 18715.0 km.
///   The simulator takes C for 50 km: 4 x 28600 x 2^(1/3) = 144134.97 N, (144134.97 / 20000)^3 x 50 = 7.2067^3 x 50
///   = 374.30 x 50 = 18715.0 km: the same life.
/// THK SHS25C, four carriages: (4 x 31700 / 20000)^3 x 50 = 6.34^3 x 50 = 254.84 x 50 = 12742.0 km (no conversion).
/// HIWIN's HGH25CA (C 34900 N for 50 km) against the Rexroth FNS25: the lives of the two sets are 2 (28600 / 34900)^3
/// = 1.1006 apart, as the ratings (36033.7 against 34900 N once on one basis) say.
#[test]
fn a_rexroth_and_a_hiwin_carriage_of_equal_rating_have_the_same_life() {
    let life_km =
        |cfg: &Config, load_n: f64| (cfg.f64("ratings.guide_dynamic_load_rating_n").unwrap() / load_n).powi(3) * 50.0;
    let mut rex = linear_axis();
    apply_axis(&mut rex, "FNS25x4", None).unwrap();
    assert!(close(life_km(&rex, 20000.0), 18715.0, 1e-4), "{}", life_km(&rex, 20000.0));
    assert!(close(life_km(&rex, 20000.0), (4.0 * 28600.0_f64 / 20000.0).powi(3) * 100.0, 1e-9));
    let mut thk = linear_axis();
    apply_axis(&mut thk, "SHS25Cx4", None).unwrap();
    assert!(close(life_km(&thk, 20000.0), 12742.0, 1e-4), "{}", life_km(&thk, 20000.0));
    let mut hiwin = linear_axis();
    apply_axis(&mut hiwin, "HGH25CAx4", None).unwrap();
    let ratio = life_km(&rex, 20000.0) / life_km(&hiwin, 20000.0);
    assert!(close(ratio, 2.0 * (28600.0_f64 / 34900.0).powi(3), 1e-9), "{ratio}");
    assert!(close(ratio, 1.1006, 1e-3), "{ratio}");
}
