//! Everything the catalogue gives a config, written out value by value and compared with a snapshot taken before
//! the catalogue moved to its vendor-neutral schema (`tests/catalog_snapshot.txt`). Every motor at every supply, every
//! drive, gearbox, belt and axis, and the lookups the search uses: equal to the last bit, so the simulator's results
//! did not change with the move. `UPDATE_CATALOG_SNAPSHOT=1` rewrites the file; do that only for a change of the data
//! that is meant to change these numbers, and say so in the commit.

use sim_components::catalog::*;
use sim_core::Config;
use std::fmt::Write;

/// The leaves of `after` that differ from `before`, one line each, or the error.
fn diff(out: &mut String, label: &str, before: &Config, after: Result<Config, sim_core::ConfigError>) {
    match after {
        Err(e) => writeln!(out, "{label}: error {} {}", e.key(), e.detail()).unwrap(),
        Ok(after) => {
            let old: std::collections::HashMap<String, String> =
                before.leaves().into_iter().map(|(k, v)| (k, format!("{v:?}"))).collect();
            for (path, value) in after.leaves() {
                let value = format!("{value:?}");
                if old.get(&path) != Some(&value) {
                    writeln!(out, "{label}: {path} = {value}").unwrap();
                }
            }
        }
    }
}

fn snapshot() -> String {
    let mut out = String::new();
    let single = &machine_single_axis::ENTRY;
    let gantry = &machine_t_gantry::ENTRY;
    let base = single.config();
    let exct = Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap();
    let applied = |f: &dyn Fn(&mut Config) -> Result<(), sim_core::ConfigError>| {
        let mut cfg = base.clone();
        f(&mut cfg).map(|()| cfg)
    };
    for supply in SUPPLIES {
        writeln!(out, "motors at {supply}: {:?}", motor_articles_for(supply)).unwrap();
        for article in motor_articles_for(supply) {
            let label = format!("motor {article} {supply}");
            diff(&mut out, &label, &base, applied(&|c| apply_motor(c, article, supply)));
            writeln!(
                out,
                "{label}: paired {:?}, mass {:?}",
                paired_drive(article, supply),
                motor_mass(article, supply)
            )
            .unwrap();
            for series in ["NRB", "NRK", "NLC"] {
                writeln!(out, "{label}: {series} {:?}", fitting_gearboxes(article, series, 5.0)).unwrap();
            }
        }
    }
    let drives: std::collections::BTreeSet<&str> =
        SUPPLIES.iter().flat_map(|s| motor_articles_for(s).into_iter().map(move |a| paired_drive(a, s))).collect();
    for drive in drives
        .iter()
        .chain(["6SL3120-2TE13-0AD0", "6SL5310-1BE10-8DF0", "6SL3210-5HB10-8UF0", "6SL3210-0XX00-0XX0"].iter())
    {
        for supply in SUPPLIES {
            for (name, cfg) in [("single", &base), ("exct", &exct)] {
                let mut with = cfg.clone();
                diff(
                    &mut out,
                    &format!("drive {drive} {supply} {name}"),
                    cfg,
                    apply_drive(&mut with, drive, supply).map(|()| with),
                );
            }
        }
    }
    for series in ["NRB", "NRK", "NLC"] {
        for frame in ["02", "03", "04", "05", "06", "07", "08", "10"] {
            for ratio in [
                "3", "4", "5", "7", "8", "9", "10", "12", "15", "16", "20", "25", "32", "40", "50", "60", "64", "80",
                "100", "120", "160", "200", "256", "320", "11",
            ] {
                for size in ["040", "050", "060", "070", "080", "090", "120", "155", "160"] {
                    let name = format!("{series}{size}-{frame}-{ratio}");
                    let mut cfg = base.clone();
                    if let Ok(()) = apply_gearbox(&mut cfg, &name) {
                        diff(&mut out, &format!("gearbox {name}"), &base, Ok(cfg));
                        writeln!(out, "gearbox {name}: mass {:?}", gearbox_mass(&name)).unwrap();
                    }
                }
            }
        }
    }
    for profile in ["T5", "AT3", "AT5", "AT10", "MTD3", "MTD5", "MTD8", "XL"] {
        for width in
            ["4", "6", "9", "10", "12", "15", "16", "20", "25", "30", "32", "48", "50", "70", "75", "100", "150", "200"]
        {
            let name = format!("{profile}-{width}");
            diff(&mut out, &format!("belt {name}"), &exct, {
                let mut cfg = exct.clone();
                apply_belt(&mut cfg, &name).map(|()| cfg)
            });
        }
    }
    writeln!(out, "axes: {:?}", axis_names()).unwrap();
    for axis in axis_names() {
        for (name, entry, cfg) in [("exct", gantry, &exct), ("single", single, &base)] {
            let mut named = cfg.clone();
            named.set_text(&format!("parts.axis = {axis:?}")).unwrap();
            diff(&mut out, &format!("axis {axis} on {name}"), cfg, resolve(entry, &named));
            diff(&mut out, &format!("axis {axis} on {name}, no machine"), cfg, {
                let mut c = cfg.clone();
                apply_axis(&mut c, &axis, None).map(|()| c)
            });
        }
    }
    out
}

#[test]
fn the_catalogue_gives_every_config_the_values_it_gave_before_the_schema() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/catalog_snapshot.txt");
    let now = snapshot();
    if std::env::var_os("UPDATE_CATALOG_SNAPSHOT").is_some() {
        std::fs::write(&path, &now).unwrap();
    }
    let before = std::fs::read_to_string(&path).unwrap().replace("\r\n", "\n");
    let changed: Vec<(&str, &str)> = before.lines().zip(now.lines()).filter(|(a, b)| a != b).take(10).collect();
    assert!(
        changed.is_empty() && before.lines().count() == now.lines().count(),
        "{} lines before, {} now; first changes: {changed:#?}",
        before.lines().count(),
        now.lines().count()
    );
}
