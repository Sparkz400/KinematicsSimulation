//! The command line runs every registered machine and writes traces to CSV.

use sim_machines::MACHINES;
use std::process::{Command, Output};

fn cli(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_sim-cli")).args(args).output().unwrap()
}

fn text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes).into_owned()
}

#[test]
fn list_names_every_machine_in_both_languages() {
    let (en, de) = (cli(&["list"]), cli(&["list", "--lang", "de"]));
    assert!(en.status.success() && de.status.success());
    let (en, de) = (text(&en.stdout), text(&de.stdout));
    assert_eq!(en.lines().count(), MACHINES.len());
    for (entry, line) in MACHINES.iter().zip(en.lines()) {
        assert!(line.starts_with(entry.id));
    }
    assert_ne!(en, de, "German and English names are the same");
}

#[test]
fn run_writes_every_signal_as_csv_for_every_machine() {
    for entry in MACHINES {
        let sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
        let target: Vec<String> = sim.machine.axes().iter().map(|a| format!("{}", 0.5 * (a.min + a.max))).collect();
        let path = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{}.csv", entry.id));
        let out = cli(&["run", entry.id, "--move", &target.join(","), "--time", "0.5", "--out", path.to_str().unwrap()]);
        assert!(out.status.success(), "{}", text(&out.stderr));
        let csv = std::fs::read_to_string(&path).unwrap();
        let mut lines = csv.lines();
        let header: Vec<&str> = lines.next().unwrap().split(',').collect();
        assert_eq!(header.len(), sim.signal_infos().len());
        assert_eq!(header[0], "time[s]");
        let rows: Vec<Vec<f64>> = lines.map(|l| l.split(',').map(|v| v.parse().unwrap()).collect()).collect();
        // One row per millisecond, start and end included.
        assert_eq!(rows.len(), 501);
        assert!(rows.iter().all(|r| r.len() == header.len() && r.iter().all(|v| v.is_finite())));
        assert!((rows[500][0] - 0.5).abs() < 1e-12);
        // The first axis moved towards its target.
        let position = sim.signal_index(&format!("axis.{}.position", sim.machine.axes()[0].id)).unwrap();
        assert!((rows[500][position] - rows[0][position]).abs() > 0.01);
    }
}

#[test]
fn set_overrides_a_parameter_and_cycle_runs() {
    let id = MACHINES[0].id;
    let run = |extra: &[&str]| {
        let mut args = vec!["run", id, "--cycle", "--time", "0.3"];
        args.extend(extra);
        let out = cli(&args);
        assert!(out.status.success(), "{}", text(&out.stderr));
        text(&out.stdout)
    };
    assert_ne!(run(&[]), run(&["--set", "mass.payload_kg=5.0"]));
}

#[test]
fn mistakes_are_reported_in_the_chosen_language() {
    let en = cli(&["run", "no-such-machine"]);
    let de = cli(&["run", "no-such-machine", "--lang", "de"]);
    assert_eq!(en.status.code(), Some(2));
    assert!(text(&en.stderr).contains("no-such-machine") && text(&de.stderr).contains("no-such-machine"));
    assert_ne!(text(&en.stderr), text(&de.stderr));
    assert_eq!(cli(&[]).status.code(), Some(2));
    let bad = cli(&["run", MACHINES[0].id, "--set", "mass.no_such_value=1"]);
    assert_eq!(bad.status.code(), Some(2));
    assert!(text(&bad.stderr).contains("mass.no_such_value"));
}
