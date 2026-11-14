//! Property tests of the drive trace readers: mapping files (`Mapping::parse`), the vendor layouts through them
//! (`import`: CSV, SINAMICS, TwinCAT, CODESYS), traces written by `sim-cli run` (`Trace::parse`) and frequency responses
//! (`identify::parse_sweep`). Whatever the bytes, an `Ok` or a `TraceError`, never a panic; an imported trace has
//! finite values on a time grid from 0 that rises; and what a run writes is read back as written.

#[path = "../../sim-core/tests/edits/mod.rs"]
mod edits;

use proptest::prelude::*;
use sim_components::identify::parse_sweep;
use sim_components::import::{Imported, Mapping, decode, import};
use sim_components::trace::Trace;
use sim_core::MotorInfo;

const TWINCAT: &str = r#"format = "twincat"
[[motor]]
id = "x"
setpoint = { signal = "SetPos", per_rev = 10.0 }
angle = { signal = "MAIN.Axis.NcToPlc.ActPos", per_rev = 10.0 }
speed = { signal = "ActVelo", per_rev = 10.0 }
torque = { signal = "TorqueActual", reference = 2.0 }
"#;

const CODESYS: &str = r#"format = "codesys"
[[motor]]
setpoint = { signal = "Axis1.fSetPosition", unit = "mm", per_rev = 5.0 }
angle = { signal = "Axis1.fActPosition", unit = "mm", per_rev = 5.0 }
speed = { signal = "Axis1.fActVelocity", unit = "mm/s", per_rev = 5.0 }
torque = { signal = "Axis1.fActTorque", unit = "Nm" }
torque_setpoint = { signal = "Axis1.fSetTorque", unit = "Nm" }
"#;

const SINAMICS: &str = r#"format = "sinamics"
[[motor]]
setpoint = { signal = "r2665", per_rev = 10000 }
angle = { signal = "r2521", per_rev = 10000, wrap = 4294967296 }
speed = "r61"
torque = "r80"
torque_setpoint = "r79"
"#;

fn fixture(path: &str) -> String {
    decode(&std::fs::read(edits::root().join(path)).unwrap())
}

/// Each vendor's file with the mapping that reads it: the fixtures of tests/import.rs and the example dataset (its
/// first 200 lines).
fn pairs() -> Vec<(String, String)> {
    let dataset =
        fixture("datasets/example-synthetic-belt-axis/trace.csv").lines().take(200).collect::<Vec<_>>().join("\n");
    let f = |name: &str| fixture(&format!("crates/sim-components/tests/fixtures/{name}"));
    vec![
        (TWINCAT.into(), f("twincat_scope.csv")),
        (CODESYS.into(), f("codesys_trace.trace")),
        (CODESYS.into(), f("codesys_trace.txt")),
        (CODESYS.into(), f("codesys_trace.trace.csv")),
        (SINAMICS.into(), f("sinamics_trace.csv")),
        (fixture("datasets/example-synthetic-belt-axis/map.toml"), dataset),
    ]
}

fn maps() -> Vec<(String, String)> {
    let mut out: Vec<(String, String)> = pairs().into_iter().map(|(map, _)| ("map".into(), map)).collect();
    out.push(("template".into(), fixture("datasets/template/map.toml")));
    out
}

fn check(imported: &Imported) -> Result<(), TestCaseError> {
    let t = &imported.trace;
    prop_assert!(t.time.first().is_none_or(|&t0| t0 == 0.0), "{:?}", &t.time[..1]);
    prop_assert!(t.time.windows(2).all(|w| w[1] > w[0]), "time does not rise");
    for m in &t.motors {
        for column in [&m.setpoint, &m.angle, &m.speed, &m.torque] {
            prop_assert_eq!(column.len(), t.time.len());
            prop_assert!(column.iter().all(|v| v.is_finite()), "{column:?}");
        }
    }
    Ok(())
}

fn motors(ids: &[&'static str]) -> Vec<MotorInfo> {
    ids.iter().map(|&id| MotorInfo { id, key: "" }).collect()
}

proptest! {
    #[test]
    fn any_bytes_are_imported_or_refused(bytes in prop::collection::vec(any::<u8>(), 0..1500), pick in 0..3usize) {
        let text = decode(&bytes);
        let map = Mapping::parse([TWINCAT, CODESYS, SINAMICS][pick]).unwrap();
        if let Ok(imported) = import(&text, &map) {
            check(&imported)?;
        }
        let _ = Mapping::parse(&text);
        let _ = Trace::parse(&text, &motors(&["x", "y"]));
        let _ = parse_sweep(&text);
    }

    #[test]
    fn edited_vendor_files_are_imported_or_refused(
        (map, text) in prop::sample::select(pairs()),
        e in edits::edits(),
    ) {
        if let Ok(imported) = import(&edits::apply(&text, &e), &Mapping::parse(&map).unwrap()) {
            check(&imported)?;
        }
    }

    #[test]
    fn edited_mappings_are_read_or_refused(
        (_, map) in edits::edited(maps()),
        (_, text) in prop::sample::select(pairs()),
    ) {
        if let Ok(map) = Mapping::parse(&map)
            && let Ok(imported) = import(&text, &map)
        {
            check(&imported)?;
        }
    }

    /// A trace as `sim-cli run` writes it (`id[unit]` headers, `{:.9e}` values, columns of other signals between) is
    /// read back with the values written; an edited one is read or refused.
    #[test]
    fn a_trace_written_by_a_run_is_read_back(
        steps in prop::collection::vec(1e-6..1.0f64, 2..40),
        values in prop::collection::vec(-1e12..1e12f64, 9 * 40),
        e in edits::edits(),
    ) {
        let ids = ["x", "y"];
        let mut header = vec!["time[s]".to_string(), "axis.x.position[m]".into()];
        for id in ids {
            header.extend(["angle_setpoint", "angle", "speed", "torque"].map(|q| format!("motor.{id}.{q}[rad]")));
        }
        let mut text = header.join(",") + "\n";
        let mut time = 0.0;
        let mut rows = Vec::new();
        for (r, step) in steps.iter().enumerate() {
            let row: Vec<String> =
                std::iter::once(time).chain(values[9 * r..9 * r + 9].iter().copied()).map(|v| format!("{v:.9e}")).collect();
            text += &(row.join(",") + "\n");
            rows.push(row);
            time += step;
        }
        let trace = Trace::parse(&text, &motors(&ids)).unwrap();
        // Times that round to the same 10 digits do not rise; others are read as written.
        let read = |r: &[String], c: usize| r[c].parse::<f64>().unwrap();
        prop_assert_eq!(trace.time, rows.iter().map(|r| read(r, 0)).collect::<Vec<_>>());
        for (k, m) in trace.motors.iter().enumerate() {
            let column = |q: usize| rows.iter().map(|r| read(r, 2 + 4 * k + q)).collect::<Vec<_>>();
            prop_assert_eq!(&m.setpoint, &column(0));
            prop_assert_eq!(&m.angle, &column(1));
            prop_assert_eq!(&m.speed, &column(2));
            prop_assert_eq!(&m.torque, &column(3));
        }
        if let Ok(t) = Trace::parse(&edits::apply(&text, &e), &motors(&ids)) {
            prop_assert!(t.time.windows(2).all(|w| w[1] > w[0]));
            prop_assert!(t.motors.iter().all(|m| m.angle.len() == t.time.len()));
        }
    }
}

/// A CODESYS `.txt` whose line above the numbers is empty panicked on the header's missing time column.
#[test]
fn a_codesys_text_export_without_a_header_is_refused() {
    let map = Mapping::parse(CODESYS).unwrap();
    let text = "CODESYS V3.5 SP20 Trace: AxisTrace
 
0 0.000 -0.010 50 0.500 0.520
2 0.100 0.090 50 0.510 0.530
";
    assert_eq!(import(text, &map).unwrap_err().key, "error-import-layout");
}

/// A per_rev of 5e-324 makes the factor to rad infinite: the angles were infinite or NaN.
#[test]
fn a_mapping_that_takes_values_beyond_the_numbers_is_refused() {
    let (map, text) = pairs().swap_remove(2);
    let map = Mapping::parse(&map.replacen("per_rev = 5.0", "per_rev = 5e-324", 1)).unwrap();
    assert_eq!(import(&text, &map).unwrap_err().key, "error-import-map");
    let map =
        Mapping::parse(&CODESYS.replacen("unit = \"Nm\"", "unit = \"Nm\", scale = 1e308, offset = 1e10", 1)).unwrap();
    assert_eq!(import(&text, &map).unwrap_err().key, "error-import-map");
}
