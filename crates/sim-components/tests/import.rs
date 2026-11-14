//! The vendor importers against fixtures built after the vendors' documented or observed layouts
//! (tests/fixtures; see the comments in docs/measurement-procedure.md for what each is based on).

use sim_components::import::{Mapping, decode, import};
use std::f64::consts::TAU;

fn fixture(name: &str) -> String {
    decode(&std::fs::read(format!("{}/tests/fixtures/{name}", env!("CARGO_MANIFEST_DIR"))).unwrap())
}

fn close(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-9 * b.abs().max(1.0), "{a} != {b}");
}

#[test]
fn twincat_scope_export_with_decimal_comma_and_channels_of_two_sample_times() {
    let map = Mapping::parse(
        r#"
        format = "twincat"
        [[motor]]
        id = "x"
        setpoint = { signal = "SetPos", per_rev = 10.0 }
        angle = { signal = "MAIN.Axis.NcToPlc.ActPos", per_rev = 10.0 }
        speed = { signal = "ActVelo", per_rev = 10.0 }
        torque = { signal = "TorqueActual", reference = 2.0 }
        "#,
    )
    .unwrap();
    let t = import(&fixture("twincat_scope.csv"), &map).unwrap().trace;
    // The 1 ms channels set the grid; the 0.5 ms torque channel is resampled onto it.
    assert_eq!(t.time.len(), 6);
    close(t.time[3], 0.003);
    let m = &t.motors[0];
    close(m.setpoint[2], 1.0 / 10.0 * TAU);
    close(m.angle[2], 0.75 / 10.0 * TAU);
    close(m.speed[0], 500.0 / 10.0 * TAU);
    // 140 ‰ of 2 N m at 2 ms.
    close(m.torque[2], 0.28);
}

fn codesys_map() -> Mapping {
    Mapping::parse(
        r#"
        format = "codesys"
        [[motor]]
        setpoint = { signal = "Axis1.fSetPosition", unit = "mm", per_rev = 5.0 }
        angle = { signal = "Axis1.fActPosition", unit = "mm", per_rev = 5.0 }
        speed = { signal = "Axis1.fActVelocity", unit = "mm/s", per_rev = 5.0 }
        torque = { signal = "Axis1.fActTorque", unit = "Nm" }
        "#,
    )
    .unwrap()
}

#[test]
fn codesys_exports_read_alike() {
    let xml = import(&fixture("codesys_trace.trace"), &codesys_map()).unwrap().trace;
    let csv = import(&fixture("codesys_trace.trace.csv"), &codesys_map()).unwrap().trace;
    assert_eq!(xml, csv);
    // Timestamps in µs, 1 ms apart.
    close(xml.time[5], 0.005);
    close(xml.motors[0].setpoint[3], 0.3 / 5.0 * TAU);
    close(xml.motors[0].speed[1], 50.0 / 5.0 * TAU);
    close(xml.motors[0].torque[4], 0.54);
    // The .txt in ms, 2 ms apart.
    let mut map = codesys_map();
    map.motors[0].torque_setpoint = Mapping::parse("format = \"codesys\"\n[[motor]]\nsetpoint = \"a\"\nangle = \"a\"\ntorque = \"a\"\ntorque_setpoint = { signal = \"Axis1.fSetTorque\", unit = \"Nm\" }").unwrap().motors[0].torque_setpoint.clone();
    let txt = import(&fixture("codesys_trace.txt"), &map).unwrap();
    close(txt.trace.time[5], 0.010);
    close(txt.trace.motors[0].angle[2], 0.19 / 5.0 * TAU);
    close(txt.torque_setpoints[0].as_ref().unwrap()[1], 0.53);
}

#[test]
fn sinamics_csv_with_metadata_lines_units_in_brackets_and_parameter_numbers() {
    let map = Mapping::parse(
        r#"
        format = "sinamics"
        [[motor]]
        setpoint = { signal = "r2665", per_rev = 10000 }
        angle = { signal = "r2521", per_rev = 10000 }
        speed = "r61"
        torque = "r80"
        torque_setpoint = "r79"
        "#,
    )
    .unwrap();
    let imported = import(&fixture("sinamics_trace.csv"), &map).unwrap();
    let m = &imported.trace.motors[0];
    close(imported.trace.time[4], 0.004);
    close(m.setpoint[2], 0.2 * TAU);
    close(m.angle[2], 0.19 * TAU);
    close(m.speed[0], 600.0 / 60.0 * TAU);
    close(m.torque[3], 1.28);
    close(imported.torque_setpoints[0].as_ref().unwrap()[0], 1.3);
}

#[test]
fn generic_csv_with_a_line_of_units_and_a_time_offset() {
    let text = "t,pos_set,pos,trq\ns,deg,deg,mNm\n0,0,0,100\n0.001,1,0.5,110\n0.002,2,1.5,120\n0.003,3,2.5,130\n0.004,4,3.5,140\n";
    let map = Mapping::parse(
        r#"
        format = "csv"
        time = "t"
        [[motor]]
        setpoint = "pos_set"
        angle = "pos"
        torque = { signal = "trq", time_offset_s = -0.001 }
        "#,
    )
    .unwrap();
    let t = import(text, &map).unwrap().trace;
    // The torque is shifted 1 ms earlier, so the common span is 0 to 3 ms of the file.
    assert_eq!(t.time.len(), 4);
    close(t.motors[0].setpoint[1], 1.0_f64.to_radians());
    close(t.motors[0].torque[0], 0.110);
    // No speed signal: the slope of the angle.
    close(t.motors[0].speed[1], (1.5f64 - 0.0).to_radians() / 0.002);
}

#[test]
fn mistakes_in_the_mapping_say_what_is_wrong() {
    let err = |map: &str| Mapping::parse(map).unwrap_err().detail;
    assert!(err("format = \"excel\"\n[[motor]]\nsetpoint=\"a\"\nangle=\"a\"\ntorque=\"a\"").contains("excel"));
    assert!(err("format = \"csv\"\n[[motor]]\nsetpoint=\"a\"\nangle=\"a\"").contains("torque"));
    assert!(
        err("format = \"csv\"\n[[motor]]\nsetpoint=\"a\"\nangle={signal=\"a\",per_revs=3}\ntorque=\"a\"")
            .contains("per_revs")
    );
    let map = Mapping::parse("format = \"sinamics\"\n[[motor]]\nsetpoint=\"r2665\"\nangle=\"r2521\"\ntorque=\"r80\"")
        .unwrap();
    let e = import(&fixture("sinamics_trace.csv"), &map).unwrap_err();
    assert_eq!((e.key, e.detail.as_str()), ("error-import-per-rev", "r2665: LU"));
    let map = Mapping::parse("format = \"sinamics\"\n[[motor]]\nsetpoint=\"r9999\"\nangle=\"r2521\"\ntorque=\"r80\"")
        .unwrap();
    assert_eq!(import(&fixture("sinamics_trace.csv"), &map).unwrap_err().key, "error-import-signal");
}

#[test]
fn the_dataset_template_mapping_loads() {
    let text =
        std::fs::read_to_string(format!("{}/../../datasets/template/map.toml", env!("CARGO_MANIFEST_DIR"))).unwrap();
    let map = Mapping::parse(&text).unwrap();
    assert_eq!((map.format.as_str(), map.motors.len()), ("sinamics", 1));
    // And it reads the SINAMICS fixture, whose position is in LU at 10000 per turn.
    let imported = import(&fixture("sinamics_trace.csv"), &map).unwrap();
    close(imported.trace.motors[0].setpoint[1], 0.1 * TAU);
}

fn plain_map(extra: &str) -> Mapping {
    Mapping::parse(&format!("format = \"csv\"\n{extra}\n[[motor]]\nsetpoint = {{ signal = \"pos\", unit = \"rad\" }}\nangle = {{ signal = \"pos\", unit = \"rad\" }}\ntorque = {{ signal = \"torque\", unit = \"Nm\" }}\n")).unwrap()
}

/// Found by review: the separator was the commonest character of a line, so decimal commas outvoted
/// semicolons; a metadata line above the names was taken for the names; a trailing separator and quoted
/// numbers failed the whole file.
#[test]
fn real_world_csv_quirks() {
    let rows = |sep: &str, comma: bool, trail: &str, quote: &str| -> String {
        (0..6)
            .map(|i| {
                [i as f64 * 0.125, i as f64 * 2.5, 0.25]
                    .map(|v| {
                        let v = format!("{v}");
                        format!("{quote}{}{quote}", if comma { v.replace('.', ",") } else { v })
                    })
                    .join(sep)
                    + trail
                    + "\n"
            })
            .collect()
    };
    let cases = [
        format!("time [s];pos;torque\n{}", rows(";", true, "", "")),
        format!("Recorder;S210;2026-11\ntime [s];pos;torque\n{}", rows(";", true, "", "")),
        format!("time [s];pos;torque;\n{}", rows(";", true, ";", "")),
        format!("\"time [s]\",\"pos\",\"torque\"\n{}", rows(",", false, "", "\"")),
        format!("time\tpos\ttorque\n{}", rows("\t", true, "", "")),
    ];
    for text in &cases {
        let t = import(text, &plain_map("")).unwrap_or_else(|e| panic!("{e:?} for\n{text}")).trace;
        assert_eq!(t.time.len(), 6, "{text}");
        close(t.time[1], 0.125);
        close(t.motors[0].angle[2], 5.0);
    }
}

/// Found by review: `time_unit` multiplied a time that already had its unit applied.
#[test]
fn time_unit_overrides_the_files_unit_and_is_refused_where_the_format_has_its_own() {
    let text = "time [ms],pos,torque\n0,0,0\n1,1,0\n2,2,0\n3,3,0\n4,4,0\n";
    close(import(text, &plain_map("time_unit = \"ms\"")).unwrap().trace.time[4], 0.004);
    close(import(text, &plain_map("time_unit = \"us\"")).unwrap().trace.time[4], 4e-6);
    assert!(
        Mapping::parse(
            "format = \"twincat\"\ntime_unit = \"ms\"\n[[motor]]\nsetpoint=\"a\"\nangle=\"a\"\ntorque=\"a\""
        )
        .is_err()
    );
}

/// Found by review: a tiny `sample_time_s` asked for a grid that did not fit in memory.
#[test]
fn a_grid_far_finer_than_the_signals_is_refused() {
    let text = "time,pos,torque\n0,0,0\n1,1,0\n2,2,0\n3,3,0\n";
    assert_eq!(import(text, &plain_map("sample_time_s = 1e-10")).unwrap_err().key, "error-import-time");
}

/// Found by the audit of 14 November: a time column without a unit was read as seconds, so a Startdrive export in ms
/// without `time_unit` came out 1000 times too slow. A step that is a whole number of the 125 µs base cycle, from
/// 0.5 on, is a drive cycle in ms: rows 1 apart are 1 ms apart, row 4 at 4 ms, with a note. `time_unit = "s"` in the
/// mapping keeps seconds (row 4 at 4 s), and a step of 0.125 stays seconds (`real_world_csv_quirks`).
#[test]
fn a_time_column_without_unit_that_steps_like_a_drive_cycle_is_read_in_ms() {
    let text = "time;pos;torque\n0;0;0\n1;1;0\n2;2;0\n3;3;0\n4;4;0\n";
    let imported = import(text, &plain_map("")).unwrap();
    close(imported.trace.time[4], 0.004);
    assert_eq!(imported.notes, vec![("note-import-time-ms", "time: 1".to_string())]);
    let seconds = import(text, &plain_map("time_unit = \"s\"")).unwrap();
    close(seconds.trace.time[4], 4.0);
    assert!(seconds.notes.is_empty());
}

/// Found by the audit of 14 November: a line of numbers above the header (`1000;8` under `Sample time;0,001`) was
/// taken for the first data row, and the header under it then failed as a row. The data are the rows of numbers at
/// the end of the file; the two lines above the header are skipped with a note.
#[test]
fn metadata_lines_above_the_header_are_skipped_with_a_note() {
    let text = "Sample time;0,001\n1000;8\ntime [s];pos;torque\n0;0;0\n0,001;1;0\n0,002;2;0\n0,003;3;0\n";
    let imported = import(text, &plain_map("")).unwrap();
    assert_eq!(imported.trace.time.len(), 4);
    close(imported.trace.time[3], 0.003);
    close(imported.trace.motors[0].angle[2], 2.0);
    assert_eq!(imported.notes, vec![("note-import-skipped", "2: Sample time;0,001 | 1000;8".to_string())]);
}

/// Found by the audit of 14 November: times that do not rise reached the fits, where a zero step divided by zero.
/// They are refused naming the signal and the row (row 3 under the header repeats the 1 ms of row 2), here and in a
/// trace in the simulator's own columns.
#[test]
fn times_that_do_not_rise_are_refused_naming_the_row() {
    let text = "time [s];pos;torque\n0;0;0\n0,001;1;0\n0,001;2;0\n0,003;3;0\n";
    let e = import(text, &plain_map("")).unwrap_err();
    assert_eq!((e.key, e.detail.as_str()), ("error-import-time", "pos: row 3: 0.001 s after 0.001 s"));
    let e = sim_components::trace::Trace::parse("time\n0\n0.001\n0.0005\n", &[]).unwrap_err();
    assert_eq!((e.key, e.detail.as_str()), ("error-import-time", "row 3: 0.0005 s after 0.001 s"));
}
