//! `datasets/RESULTS.md`: one row per dataset, made from its `sim-cli fit --dataset --report` report. The test fails
//! when the committed table is not what the reports give now; `UPDATE_RESULTS=1` writes it anew.

use std::path::{Path, PathBuf};
use std::process::Command;

fn datasets() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../datasets")
}

/// Three significant digits, as the table shows them.
fn sig(v: f64) -> String {
    let decimals = (2 - v.abs().log10().floor() as i32).max(0) as usize;
    format!("{v:.decimals$}")
}

fn share(v: f64) -> String {
    if v < 10.0 { format!("{v:.1} %") } else { format!("{v:.0} %") }
}

/// Rows of a report's CSV, header dropped, split at commas.
fn rows(path: &Path) -> Vec<Vec<String>> {
    let text = std::fs::read_to_string(path).unwrap_or_default();
    text.lines().skip(1).map(|l| l.split(',').map(String::from).collect()).collect()
}

/// One cell per recording of `role` in `summary.csv`: its motor with the largest RMS share, RMS and worst sample.
fn torque_cells(summary: &[Vec<String>], role: &str) -> String {
    let mut cells: Vec<(String, f64, f64, f64)> = Vec::new();
    for r in summary.iter().filter(|r| r[1] == role) {
        let number = |i: usize| r[i].parse::<f64>().unwrap();
        let (peak, rms, worst) = (number(4), number(5), number(6));
        match cells.iter_mut().find(|c| c.0 == r[0]) {
            Some(c) if c.1 / c.3 >= rms / peak => {}
            Some(c) => *c = (r[0].clone(), rms, worst, peak),
            None => cells.push((r[0].clone(), rms, worst, peak)),
        }
    }
    let cell = |(file, rms, worst, peak): &(String, f64, f64, f64)| {
        let name = file.trim_end_matches(".csv");
        format!(
            "{name}: {} N m ({}) / {} N m ({})",
            sig(*rms),
            share(100.0 * rms / peak),
            sig(*worst),
            share(100.0 * worst / peak)
        )
    };
    if cells.is_empty() { "-".into() } else { cells.iter().map(cell).collect::<Vec<_>>().join("<br>") }
}

fn table() -> String {
    let mut out = String::from(
        "# Results per dataset\n\n\
         Generated from each dataset's report, `sim-cli fit --dataset datasets/<id> --report <dir>`, by the test \
         `crates/sim-cli/tests/results.rs`; do not edit by hand. To write it anew after a model change:\n\n\
         ```text\nUPDATE_RESULTS=1 cargo test --release -p sim-cli --test results\n```\n\n\
         Torque: the measured motor torque against the simulated one when the recording's position setpoints are \
         replayed, as RMS difference and worst single sample, in N m and as a share of the recording's peak measured \
         torque (on several motors, the motor with the largest RMS share). The fit recording is the dataset's first \
         trace, the one the free values are fitted on; the checks are the others, from which nothing was set. First \
         mode: the resonance of the two-mass model fitted to each measured frequency response against the model's \
         first mode. What each figure means: [docs/validation.md](../docs/validation.md#against-measured-data); how to \
         add a dataset: [docs/measurement-procedure.md](../docs/measurement-procedure.md).\n\n\
         | Dataset | Machine | Data | Licence | Fit recording: RMS / worst | Checks: RMS / worst | First mode, measured → model |\n\
         | --- | --- | --- | --- | --- | --- | --- |\n",
    );
    let mut dirs: Vec<PathBuf> = std::fs::read_dir(datasets())
        .unwrap()
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.is_dir() && p.file_name().unwrap() != "template")
        .collect();
    dirs.sort();
    for dir in dirs {
        let data = sim_components::dataset::contents(&dir).unwrap();
        let link = format!("[{0}]({0}/)", data.id);
        let kind = if data.synthetic { "synthetic" } else { "measured" };
        if data.simulator.is_empty() {
            out += &format!("| {link} | - | {kind} | {} | not replayed: no `machine.simulator` | | |\n", data.license);
            continue;
        }
        let report = Path::new(env!("CARGO_TARGET_TMPDIR")).join("results").join(&data.id);
        let run = Command::new(env!("CARGO_BIN_EXE_sim-cli"))
            .args(["fit", "--dataset", dir.to_str().unwrap(), "--report", report.to_str().unwrap()])
            .output()
            .unwrap();
        assert!(run.status.success(), "{}: {}", data.id, String::from_utf8_lossy(&run.stderr));
        let summary = rows(&report.join("summary.csv"));
        let modes: Vec<String> = rows(&report.join("modes.csv"))
            .iter()
            .map(|r| {
                let number = |i: usize| r[i].parse::<f64>().unwrap();
                let measured =
                    if number(1).is_nan() { "-".into() } else { format!("{:.0} ± {:.0}", number(1), number(2)) };
                format!("{measured} → {:.0} Hz", number(3))
            })
            .collect();
        out += &format!(
            "| {link} | `{}` | {kind} | {} | {} | {} | {} |\n",
            data.simulator,
            data.license,
            torque_cells(&summary, "fit"),
            torque_cells(&summary, "check"),
            if modes.is_empty() { "-".into() } else { modes.join("<br>") }
        );
    }
    out
}

#[test]
fn the_results_table_is_what_the_reports_give() {
    let made = table();
    let path = datasets().join("RESULTS.md");
    if std::env::var_os("UPDATE_RESULTS").is_some() {
        std::fs::write(&path, &made).unwrap();
        return;
    }
    let committed = std::fs::read_to_string(&path).unwrap_or_default().replace("\r\n", "\n");
    assert_eq!(
        committed, made,
        "datasets/RESULTS.md is out of date: UPDATE_RESULTS=1 cargo test -p sim-cli --test results"
    );
}

/// The ISW ball screw's report gives the figures docs/validation.md "Against measured data" states for machine.toml as
/// committed: S-profile 0.430 N m RMS and 2.27 N m worst, the whole sine 0.786 and 30.0, constant speed 0.895 and 3.59;
/// the fit's viscous 779 ± 23 N s/m, Coulomb 322 ± 6 N, moved mass 492 ± 3 kg; the sweeps' resonances 134, 123, 126 Hz
/// against the model's 137 Hz. Each within one unit of the last digit the document gives.
#[test]
fn the_ball_screw_report_gives_the_documented_figures() {
    let report = Path::new(env!("CARGO_TARGET_TMPDIR")).join("isw-report");
    let dir = datasets().join("isw-ball-screw-axis");
    let run = Command::new(env!("CARGO_BIN_EXE_sim-cli"))
        .args(["fit", "--dataset", dir.to_str().unwrap(), "--report", report.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(run.status.success(), "{}", String::from_utf8_lossy(&run.stderr));
    let close = |got: f64, want: f64, digits: f64| (got - want).abs() <= digits;
    let summary = rows(&report.join("summary.csv"));
    for (file, rms, worst) in [
        ("s-profile-v300.csv", 0.430, 2.27),
        ("sine-a100-v100.csv", 0.786, 30.0),
        ("constant-speed-v100.csv", 0.895, 3.59),
    ] {
        let r = summary.iter().find(|r| r[0] == file).unwrap();
        let (got_rms, got_worst) = (r[5].parse::<f64>().unwrap(), r[6].parse::<f64>().unwrap());
        assert!(close(got_rms, rms, 0.001), "{file}: RMS {got_rms}");
        assert!(close(got_worst, worst, if worst >= 10.0 { 0.1 } else { 0.01 }), "{file}: worst {got_worst}");
    }
    let fit = rows(&report.join("fit.csv"));
    let fitted = |q: &str| {
        let r = fit.iter().find(|r| r[0] == q).unwrap();
        (r[3].parse::<f64>().unwrap(), r[4].parse::<f64>().unwrap())
    };
    let (viscous, coulomb, mass) =
        (fitted("viscous_friction_n_s_m"), fitted("coulomb_friction_n"), fitted("moved_mass_kg"));
    assert!(close(viscous.0, 779.0, 1.0) && close(viscous.1, 23.0, 1.0), "{viscous:?}");
    assert!(close(coulomb.0, 322.0, 1.0) && close(coulomb.1, 6.0, 1.0), "{coulomb:?}");
    assert!(close(mass.0, 492.0, 1.0) && close(mass.1, 3.0, 1.0), "{mass:?}");
    let modes = rows(&report.join("modes.csv"));
    for (r, hz) in modes.iter().zip([134.0, 123.0, 126.0]) {
        assert!(close(r[1].parse().unwrap(), hz, 1.0), "{r:?}");
        assert!(close(r[3].parse().unwrap(), 137.0, 1.0), "{r:?}");
    }
    let md = std::fs::read_to_string(report.join("report.md")).unwrap();
    assert!(md.contains("| s-profile-v300.csv | fit | 1 |") && md.contains("![s-profile-v300.csv, motor 1]"), "{md}");
    assert!(report.join("s-profile-v300-torque-1.svg").exists() && report.join("bounds.csv").exists());
}
