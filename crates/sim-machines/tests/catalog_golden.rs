//! Everything the simulator takes from the catalogue, written out once and held fixed: every motor at every supply,
//! every drive, gearbox, belt and axis by name, the pairings and the masses, as the values they put into a config.
//! The catalogue's files can change their layout (as in the move to schema 1) without a single result changing.
//!
//! `catalog_golden.txt` was written from the catalogue as it was before schema 1. To write it anew after a change
//! of data that is meant to change results: `UPDATE_GOLDEN=1 cargo test -p sim-machines --test catalog_golden`.

use sim_components::catalog::{
    SUPPLIES, apply_axis, apply_belt, apply_drive, apply_gearbox, apply_motor, axis_names, fitting_gearboxes,
    gearbox_mass, motor_articles, motor_articles_for, motor_mass, paired_drive, resolve,
};
use sim_core::Config;
use std::collections::BTreeSet;
use std::path::Path;
use toml::{Table, Value};

fn flat(prefix: &str, table: &Table, out: &mut Vec<(String, String)>) {
    for (key, value) in table {
        let path = if prefix.is_empty() { key.clone() } else { format!("{prefix}.{key}") };
        match value {
            Value::Table(t) => flat(&path, t, out),
            v => out.push((path, v.to_string())),
        }
    }
}

/// The keys whose values differ from `base`, as "key = value", or the error.
fn diff(base: &Config, result: Result<Config, sim_core::ConfigError>) -> String {
    let leaves = |c: &Config| {
        let mut out = Vec::new();
        flat("", &c.to_text().unwrap().parse::<Table>().unwrap(), &mut out);
        out
    };
    match result {
        Err(e) => format!("error {} {}", e.key(), e.detail()),
        Ok(cfg) => {
            let before: BTreeSet<(String, String)> = leaves(base).into_iter().collect();
            leaves(&cfg)
                .into_iter()
                .filter(|l| !before.contains(l))
                .map(|(k, v)| format!("{k} = {v}"))
                .collect::<Vec<_>>()
                .join("; ")
        }
    }
}

fn with(
    base: &Config,
    f: impl FnOnce(&mut Config) -> Result<(), sim_core::ConfigError>,
) -> Result<Config, sim_core::ConfigError> {
    let mut cfg = base.clone();
    f(&mut cfg).map(|_| cfg)
}

/// Every word in the catalogue's data files that starts with `prefix`: the drives, whatever the files' layout.
fn words(prefix: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let mut stack = vec![Path::new(env!("CARGO_MANIFEST_DIR")).join("../../catalog")];
    while let Some(dir) = stack.pop() {
        for entry in std::fs::read_dir(dir).unwrap().flatten() {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
            } else if path.extension().is_some_and(|e| e == "csv") {
                let text = std::fs::read_to_string(&path).unwrap();
                out.extend(
                    text.lines()
                        .filter(|l| !l.starts_with('#'))
                        .flat_map(|l| l.split(','))
                        .filter(|w| w.starts_with(prefix))
                        .map(String::from),
                );
            }
        }
    }
    out
}

fn snapshot() -> String {
    let gantry = &machine_t_gantry::ENTRY;
    let exct = Config::parse(machine_t_gantry::EXCT_30_CONFIG).unwrap();
    let single = Config::parse(machine_single_axis::ENTRY.default_config).unwrap();
    let mut out = Vec::new();
    let mut motors: BTreeSet<&str> = motor_articles().into_iter().collect();
    for supply in SUPPLIES {
        out.push(format!("articles {supply}: {}", motor_articles_for(supply).join(" ")));
        motors.extend(motor_articles_for(supply));
    }
    for &article in &motors {
        for supply in SUPPLIES {
            out.push(format!(
                "motor {article} {supply}: {}",
                diff(&exct, with(&exct, |c| apply_motor(c, article, supply)))
            ));
            out.push(format!(
                "pair {article} {supply}: {} mass {:?}",
                paired_drive(article, supply),
                motor_mass(article, supply)
            ));
        }
    }
    let mut drives = words("6SL3");
    drives.extend(words("6SL5"));
    for drive in &drives {
        for supply in ["230v", "400v"] {
            out.push(format!(
                "drive {drive} {supply}: {}",
                diff(&exct, with(&exct, |c| apply_drive(c, drive, supply)))
            ));
        }
    }
    let mut gearboxes = BTreeSet::new();
    for &article in &motors {
        for ratio in [4.0, 16.0, 64.0, 256.0] {
            gearboxes.extend(fitting_gearboxes(article, "", ratio));
        }
    }
    for gearbox in &gearboxes {
        out.push(format!(
            "gearbox {gearbox} mass {:?}: {}",
            gearbox_mass(gearbox),
            diff(&exct, with(&exct, |c| apply_gearbox(c, gearbox)))
        ));
    }
    for profile in ["T5", "AT3", "AT5", "AT10", "MTD3", "MTD5", "MTD8", "XX"] {
        for width in [3, 6, 9, 10, 12, 15, 16, 20, 25, 30, 32, 40, 48, 50, 70, 75, 100, 150, 200] {
            let name = format!("{profile}-{width}");
            out.push(format!("belt {name}: {}", diff(&exct, with(&exct, |c| apply_belt(c, &name)))));
        }
    }
    for axis in axis_names() {
        out.push(format!("axis {axis} gantry: {}", diff(&exct, with(&exct, |c| apply_axis(c, &axis, Some(gantry))))));
        out.push(format!(
            "axis {axis} single: {}",
            diff(&single, with(&single, |c| apply_axis(c, &axis, Some(&machine_single_axis::ENTRY))))
        ));
    }
    out.push(format!("resolve exct-30: {}", diff(&exct, resolve(gantry, &exct))));
    out.join("\n") + "\n"
}

/// Two lines alike but for numbers within 1e-12 of each other: a value the catalogue now holds in SI (0.0000093 kg m2)
/// is the last bit off what the simulator computed from the printed unit before (0.093 kg cm2 times 1e-4).
fn same(a: &str, b: &str) -> bool {
    let words = |s: &str| {
        s.split([' ', ',', '[', ']', ';', '(', ')']).filter(|w| !w.is_empty()).map(String::from).collect::<Vec<_>>()
    };
    let (a, b) = (words(a), words(b));
    a.len() == b.len()
        && a.iter().zip(&b).all(|(x, y)| match (x.parse::<f64>(), y.parse::<f64>()) {
            (Ok(x), Ok(y)) => (x - y).abs() <= 1e-12 * x.abs().max(y.abs()),
            _ => x == y,
        })
}

#[test]
fn the_catalogue_gives_every_part_the_values_it_gave_before() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/catalog_golden.txt");
    let now = snapshot();
    if std::env::var_os("UPDATE_GOLDEN").is_some() {
        std::fs::write(&path, &now).unwrap();
        return;
    }
    let before = std::fs::read_to_string(&path).unwrap().replace("\r\n", "\n");
    let changed: Vec<String> = before
        .lines()
        .zip(now.lines())
        .filter(|(a, b)| !same(a, b))
        .take(10)
        .map(|(a, b)| format!("was: {a}\nnow: {b}"))
        .collect();
    assert!(
        changed.is_empty() && before.lines().count() == now.lines().count(),
        "{} lines before, {} now; first changes:\n{}",
        before.lines().count(),
        now.lines().count(),
        changed.join("\n")
    );
}
