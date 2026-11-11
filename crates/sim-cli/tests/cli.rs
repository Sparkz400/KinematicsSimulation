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

#[test]
fn analyze_and_sweep_write_csv_for_every_machine() {
    for entry in MACHINES {
        let motors = sim_components::Sim::new(entry, &entry.config()).unwrap().machine.motors().len();
        let points = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{}-points.csv", entry.id));
        let out = cli(&["analyze", entry.id, "--points", points.to_str().unwrap()]);
        assert!(out.status.success(), "{}", text(&out.stderr));
        let table = text(&out.stdout);
        assert_eq!(table.lines().next().unwrap(), "motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation");
        let rows: Vec<Vec<&str>> = table.lines().skip(1).map(|l| l.split(',').collect()).collect();
        assert_eq!(rows.len(), motors);
        for row in &rows {
            let (peak, rms, use_): (f64, f64, f64) = (row[2].parse().unwrap(), row[3].parse().unwrap(), row[5].parse().unwrap());
            assert!(peak > rms && rms > 0.0 && use_ > 0.0 && use_ < 1.0, "{row:?}");
        }
        assert!(std::fs::read_to_string(&points).unwrap().lines().count() > 1000 * motors);

        let out = cli(&["sweep", entry.id, "--from", "10", "--to", "100", "--count", "4"]);
        assert!(out.status.success(), "{}", text(&out.stderr));
        let table = text(&out.stdout);
        assert_eq!(table.lines().count(), 5);
        assert!(table.lines().skip(1).all(|l| l.split(',').all(|v| v.parse::<f64>().is_ok_and(f64::is_finite))));
    }
}

#[test]
fn step_bode_and_tuning_write_csv_for_every_machine() {
    for entry in MACHINES {
        let axes = sim_components::Sim::new(entry, &entry.config()).unwrap().machine.axes().len();
        let table = |args: &[&str]| {
            let out = cli(args);
            assert!(out.status.success(), "{}", text(&out.stderr));
            text(&out.stdout)
        };
        let step = table(&["step", entry.id, "--time", "0.2"]);
        assert_eq!(step.lines().next().unwrap(), "time_s,setpoint,response");
        assert_eq!(step.lines().count(), 2 + 800);
        let bode = table(&["bode", entry.id, "--from", "2", "--to", "50", "--count", "4"]);
        assert_eq!(bode.lines().next().unwrap(), "frequency_hz,gain,gain_db,phase_deg");
        assert_eq!(bode.lines().count(), 5);
        let tuning = table(&["tuning", entry.id]);
        assert_eq!(tuning.lines().count(), 1 + axes);
        for row in tuning.lines().skip(1) {
            let cells: Vec<&str> = row.split(',').collect();
            let band: f64 = cells[4].parse().unwrap();
            // Kv = 50 1/s puts the position loop's bandwidth near 8 Hz.
            assert!(band > 4.0 && band < 20.0, "{}: {row}", entry.id);
        }
    }
}
