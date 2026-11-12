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
        assert_eq!(table.lines().next().unwrap(), "motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,belt_tension_min_n,belt_tension_max_n,rms_current_a,winding_rise_k,energy_in_j,energy_regenerated_j,peak_regenerated_power_w,largest_regeneration_j,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,cubic_mean_torque_nm,braking_resistor_energy_j,braking_resistor_mean_power_w,dc_link_peak_voltage_v,belt_slack_time_s,radial_load_peak_n,guide_load_peak_n,guide_life_km");
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

/// A path file is run and analysed on every machine, timed within the axis limits and as fast as the motors
/// allow: the run ends at the end of the path, and the faster plan takes less time and more torque.
#[test]
fn a_path_file_is_planned_run_and_analysed_for_every_machine() {
    for entry in MACHINES {
        let sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
        let file = |name: &str| std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{}-{name}", entry.id)).to_str().unwrap().to_string();
        // A line to the middle of the travel and one on to three quarters of it: no corner on one axis, none on two.
        let point = |share: f64| sim.machine.axes().iter().map(|a| format!("{}", a.min + share * (a.max - a.min))).collect::<Vec<_>>().join(",");
        let (path, out) = (file("path.csv"), file("path-run.csv"));
        std::fs::write(&path, format!("# two lines\nline,{}\nline,{}\n", point(0.5), point(0.75))).unwrap();
        let run = cli(&["run", entry.id, "--path", &path, "--time", "3", "--out", &out]);
        assert!(run.status.success(), "{}", text(&run.stderr));
        let csv = std::fs::read_to_string(&out).unwrap();
        let last: Vec<f64> = csv.lines().last().unwrap().split(',').map(|v| v.parse().unwrap()).collect();
        let axis = &sim.machine.axes()[0];
        let position = last[sim.signal_index(&format!("axis.{}.position", axis.id)).unwrap()];
        assert!((position - (axis.min + 0.75 * (axis.max - axis.min))).abs() < 1e-3, "{}: ends at {position}", entry.id);

        let analysed = |extra: &[&str]| -> (f64, f64) {
            let out = cli(&[&["analyze", entry.id, "--path", &path], extra].concat());
            assert!(out.status.success(), "{}", text(&out.stderr));
            let table = text(&out.stdout);
            let row: Vec<&str> = table.lines().nth(1).unwrap().split(',').collect();
            (row[1].parse().unwrap(), row[2].parse().unwrap())
        };
        let ((slow, gentle), (fast, hard)) = (analysed(&[]), analysed(&["--time-optimal"]));
        assert!(fast < slow && hard > gentle, "{}: {fast} s and {hard} N m against {slow} s and {gentle} N m", entry.id);

        // A line the file format does not know is named, in the chosen language.
        std::fs::write(&path, "line,0.5\ncircle,1,2\n").unwrap();
        let (en, de) = (cli(&["run", entry.id, "--path", &path]), cli(&["run", entry.id, "--path", &path, "--lang", "de"]));
        assert_eq!(en.status.code(), Some(2));
        assert!(text(&en.stderr).contains("circle,1,2") && text(&de.stderr).contains("circle,1,2") && text(&en.stderr) != text(&de.stderr));
    }
}

/// The sensitivity run varies the parameters of the named config tables for every machine and prints
/// one row of relative changes per parameter, strongest first.
#[test]
fn sensitivity_writes_one_ranked_row_per_parameter() {
    for entry in MACHINES {
        let run = cli(&["sensitivity", entry.id, "--group", "mass", "--delta", "0.1"]);
        assert!(run.status.success(), "{}", text(&run.stderr));
        let out = text(&run.stdout);
        let mut lines = out.lines();
        assert!(lines.next().unwrap().starts_with("parameter,value,low,high,peak_torque_low,peak_torque_high"), "{out}");
        let rows: Vec<Vec<&str>> = lines.map(|l| l.split(',').collect()).collect();
        assert!(rows.len() >= 2 && rows.iter().all(|r| r[0].starts_with("mass.") && r.len() == 12), "{}: {out}", entry.id);
        let strength = |r: &Vec<&str>| r[4..10].iter().map(|v| v.parse::<f64>().unwrap().abs()).fold(0.0, f64::max);
        assert!(rows.windows(2).all(|w| strength(&w[0]) >= strength(&w[1]) - 1e-4), "{}: not ranked: {out}", entry.id);
        // Without bounds in the config the low and high values are the value less and plus 10 %.
        let number = |cell: &str| cell.parse::<f64>().unwrap();
        assert!(rows.iter().all(|r| (number(r[2]) / number(r[1]) - 0.9).abs() < 1e-9 && (number(r[3]) / number(r[1]) - 1.1).abs() < 1e-9), "{out}");
        // With bounds, they are the bounds: 10 N to 20 N for the Coulomb friction of both default configs.
        let run = cli(&["sensitivity", entry.id, "--group", "friction_x", "--delta", "0.1"]);
        let out = text(&run.stdout);
        let coulomb: Vec<&str> = out.lines().find(|l| l.starts_with("friction_x.coulomb_n,")).unwrap_or_else(|| panic!("{out}")).split(',').collect();
        assert_eq!((number(coulomb[2]), number(coulomb[3])), (10.0, 20.0), "{out}");
    }
}

/// Every machine's default cycle passes every check its config has ratings for.
#[test]
fn verdicts_judge_the_default_cycle_of_every_machine() {
    for entry in MACHINES {
        let run = cli(&["verdicts", entry.id]);
        assert!(run.status.success(), "{}", text(&run.stderr));
        let out = text(&run.stdout);
        let mut lines = out.lines();
        assert_eq!(lines.next().unwrap(), "check,motor,value,limit,result");
        let rows: Vec<Vec<&str>> = lines.map(|l| l.split(',').collect()).collect();
        for check in ["motor-peak", "motor-thermal", "gearbox-peak", "dc-link-energy", "belt-tension"] {
            assert!(rows.iter().any(|r| r[0] == check), "{}: no {check} in {out}", entry.id);
        }
        assert!(rows.iter().all(|r| r.len() == 5 && r[4] == "ok"), "{}: {out}", entry.id);
        // Four significant digits for every value: some tens of micrometres off the target do not print as 0.0000.
        let settling = rows.iter().find(|r| r[0] == "settling").unwrap();
        let (value, limit) = (settling[2].parse::<f64>().unwrap(), settling[3].parse::<f64>().unwrap());
        assert!(value > 1e-6 && value < limit && limit == 1e-4 && settling[2].trim_start_matches("0.0").len() >= 4, "{}: {settling:?}", entry.id);

        // With bands: the same checks and values, each with the lowest and highest value over the config's
        // bounds around it. More friction needs more torque, so the bands have a width.
        let run = cli(&["verdicts", entry.id, "--bands"]);
        assert!(run.status.success(), "{}", text(&run.stderr));
        let out = text(&run.stdout);
        assert_eq!(out.lines().next().unwrap(), "check,motor,value,limit,result,low,high,worst_case");
        let banded: Vec<Vec<&str>> = out.lines().skip(1).map(|l| l.split(',').collect()).collect();
        assert!(banded.len() >= rows.len() && banded.iter().zip(&rows).all(|(b, r)| b[..5] == r[..]), "{}: {out}", entry.id);
        let number = |cell: &str| cell.parse::<f64>().unwrap();
        assert!(banded.iter().all(|b| b.len() == 8 && number(b[5]) <= number(b[2]) && number(b[2]) <= number(b[6])), "{}: {out}", entry.id);
        let peak = banded.iter().find(|b| b[0] == "gearbox-peak").unwrap();
        assert!(number(peak[6]) > 1.02 * number(peak[5]) && peak[7] == "ok", "{}: {peak:?}", entry.id);

        // With samples: all bounded values between their bounds at once, and the share of the runs that pass.
        let sampled = |seed: &str| {
            let run = cli(&["verdicts", entry.id, "--bands", "--samples", "6", "--seed", seed]);
            assert!(run.status.success(), "{}", text(&run.stderr));
            text(&run.stdout)
        };
        let out = sampled("1");
        assert_eq!(out.lines().next().unwrap(), "check,motor,value,limit,result,low,high,worst_case,pass_share");
        let rows: Vec<Vec<&str>> = out.lines().skip(1).map(|l| l.split(',').collect()).collect();
        assert!(rows.iter().zip(&banded).all(|(s, b)| s[..5] == b[..5]) && rows.iter().all(|s| s.len() == 9 && (0.0..=1.0).contains(&number(s[8]))), "{}: {out}", entry.id);
        assert!(out == sampled("1") && out != sampled("2"), "{}", entry.id);
        // Samples without bands, no samples at all, half a sample and a seed without samples are refused.
        for bad in [&["--samples", "6"][..], &["--bands", "--samples", "0"], &["--bands", "--samples", "2.5"], &["--bands", "--seed", "3"], &["--bands", "--samples"]] {
            let run = cli(&[&["verdicts", entry.id], bad].concat());
            assert!(run.status.code() == Some(2) && run.stdout.is_empty() && !run.stderr.is_empty(), "{}: {bad:?}", entry.id);
        }
    }
}

/// A trace written by `run` replays onto itself: simulated torque and following error stay within 0.5 % (RMS)
/// and 5 % (peak) of the largest measured value. Fitted against the config that made it, the fit finds the
/// config's own masses and friction, and in a sweep of the config its own resonance. And a file that is no
/// trace is refused in the chosen language.
#[test]
fn a_trace_from_run_replays_onto_itself_and_fits_its_own_config() {
    for entry in MACHINES {
        let sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
        let file = |name: &str| std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{}-{name}.csv", entry.id)).to_str().unwrap().to_string();
        let (trace, overlay) = (file("trace"), file("overlay"));
        let table = |args: &[&str]| -> Vec<Vec<String>> {
            let out = cli(args);
            assert!(out.status.success(), "{}", text(&out.stderr));
            text(&out.stdout).lines().skip(1).map(|l| l.split(',').map(String::from).collect()).collect()
        };
        // One pass of the cycle moves every axis in both directions. The gripper stays empty, because
        // a trace does not say when a payload comes and goes.
        let no_payload = ["--set", "effects.payload=false"];
        table(&[&["run", entry.id, "--cycle", "--time", "6.2", "--out", &trace], &no_payload[..]].concat());

        let summary = table(&[&["replay", entry.id, "--trace", &trace, "--out", &overlay], &no_payload[..]].concat());
        let rows: Vec<Vec<f64>> = std::fs::read_to_string(&overlay).unwrap().lines().skip(1).map(|l| l.split(',').map(|v| v.parse().unwrap()).collect()).collect();
        assert_eq!((summary.len(), rows.len()), (sim.machine.motors().len(), 6201));
        for (k, motor) in summary.iter().enumerate() {
            // Per motor the overlay has measured and simulated torque, speed and following error.
            let largest = |column: usize| rows.iter().map(|r| r[1 + 6 * k + column].abs()).fold(0.0, f64::max);
            let value = |column: usize| motor[column].parse::<f64>().unwrap();
            for (rms, peak, measured) in [(value(1), value(2), largest(0)), (value(3), value(4), largest(4))] {
                assert!(rms < 0.005 * measured && peak < 0.05 * measured, "{} {motor:?}: RMS {rms}, peak {peak}, largest value {measured}", entry.id);
            }
        }

        let fit = table(&[&["fit", entry.id, "--trace", &trace], &no_payload[..]].concat());
        assert_eq!(fit.len(), 4 * sim.machine.axes().len(), "{}: {fit:?}", entry.id);
        for row in fit.iter().filter(|r| r[0] == "moved_mass_kg" || r[0] == "coulomb_friction_n") {
            let (config, fitted): (f64, f64) = (row[2].parse().unwrap(), row[3].parse().unwrap());
            assert!(config > 0.0 && (fitted / config - 1.0).abs() < 0.02, "{}: {row:?}", entry.id);
        }

        // Stage 2 on a sweep of the config itself: the dip is where the config has it.
        let sweep = file("sweep");
        let large = ["--amplitude", "1.5"];
        table(&[&["sweep", entry.id, "--from", "50", "--to", "130", "--count", "30", "--out", &sweep], &large[..]].concat());
        let fit = table(&[&["fit", entry.id, "--sweep", &sweep], &large[..]].concat());
        assert_eq!((fit.len(), fit[0][0].as_str(), fit[1][0].as_str()), (2, "resonance_hz", "resonance_damping"), "{}: {fit:?}", entry.id);
        let (config, fitted): (f64, f64) = (fit[0][2].parse().unwrap(), fit[0][3].parse().unwrap());
        assert!(config > 50.0 && config < 130.0 && (fitted / config - 1.0).abs() < 1e-3, "{}: {fit:?}", entry.id);

        let (en, de) = (cli(&["replay", entry.id, "--trace", &overlay]), cli(&["replay", entry.id, "--trace", &overlay, "--lang", "de"]));
        assert_eq!(en.status.code(), Some(2));
        assert!(text(&en.stderr).contains("angle_setpoint") && text(&de.stderr).contains("angle_setpoint") && text(&en.stderr) != text(&de.stderr));
    }
}

/// `--motor` and `--gearbox` name parts for every command; a part the catalogue does not have is refused in
/// the chosen language.
#[test]
fn parts_are_named_on_the_command_line() {
    for entry in MACHINES {
        let ratio = |extra: &[&str]| -> f64 {
            let out = cli(&[&["verdicts", entry.id], extra].concat());
            assert!(out.status.success(), "{}", text(&out.stderr));
            let table = text(&out.stdout);
            // The torque at the gearbox output over the use of the motor's curve: another motor or ratio changes it.
            let value = |check: &str| table.lines().find(|l| l.starts_with(check)).unwrap().split(',').nth(2).unwrap().parse::<f64>().unwrap();
            value("gearbox-peak") / value("motor-peak")
        };
        let (own, named) = (ratio(&[]), ratio(&["--motor", "1FK2104-5AK", "--gearbox", "NRK070-04-7"]));
        assert!(named > 0.0 && (named / own - 1.0).abs() > 0.1, "{}: {own} and {named}", entry.id);
        let (en, de) = (cli(&["run", entry.id, "--motor", "1FK9999-0AA"]), cli(&["run", entry.id, "--motor", "1FK9999-0AA", "--lang", "de"]));
        assert_eq!(en.status.code(), Some(2));
        assert!(text(&en.stderr).contains("1FK9999-0AA") && text(&de.stderr).contains("1FK9999-0AA") && text(&en.stderr) != text(&de.stderr));
    }
}

/// `run --disable-at` switches the drives off on the way: from then on no motor gives torque and the axes do
/// not go where the same run with the drives on takes them. A machine with holding brakes stands still when
/// disabled at rest, and disabled on the way it stops elsewhere than without them (where an axis coasts or falls).
#[test]
fn a_run_can_disable_the_drives_on_the_way() {
    for entry in MACHINES {
        let sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
        let rows = |args: &[&str]| -> Vec<Vec<f64>> {
            let out = cli(&[&["run", entry.id, "--time", "1.0"], args].concat());
            assert!(out.status.success(), "{}", text(&out.stderr));
            text(&out.stdout).lines().skip(1).map(|l| l.split(',').map(|v| v.parse().unwrap()).collect()).collect()
        };
        let torques: Vec<usize> = sim.machine.motors().iter().map(|m| sim.signal_index(&format!("motor.{}.torque", m.id)).unwrap()).collect();
        let positions: Vec<usize> = sim.machine.axes().iter().map(|a| sim.signal_index(&format!("axis.{}.position", a.id)).unwrap()).collect();
        let (on, off) = (rows(&["--cycle"]), rows(&["--cycle", "--disable-at", "0.3"]));
        // One row per millisecond: the same up to 0.3 s, then no torque.
        assert_eq!(on[..=300], off[..=300], "{}", entry.id);
        assert!(off[..=300].iter().any(|row| torques.iter().any(|&i| row[i] != 0.0)), "{}", entry.id);
        assert!(off[301..].iter().all(|row| torques.iter().all(|&i| row[i] == 0.0)), "{}: torque after disabling", entry.id);
        assert!(positions.iter().any(|&i| (on[1000][i] - off[1000][i]).abs() > 0.01), "{}: disabling changed nothing", entry.id);

        if entry.config().bool("effects.holding_brake").is_ok() {
            // How far the axes are after a second from where another run (or this one at its start) has them.
            let apart = |a: &[f64], b: &[f64]| positions.iter().map(|&i| (a[i] - b[i]).abs()).fold(0.0, f64::max);
            let (at_rest, free) = (rows(&["--disable-at", "0.1"]), rows(&["--cycle", "--disable-at", "0.3", "--set", "effects.holding_brake=false"]));
            let (held, released) = (apart(&at_rest[1000], &at_rest[0]), apart(&off[1000], &free[1000]));
            assert!(held < 1e-4 && released > 0.01, "{}: moved {held} m with brakes, {released} m from there without", entry.id);
        }
    }
}

/// Runs the command line and gives up after a minute: for mistakes that used to make it run for ever.
fn cli_returns(args: &[&str]) -> Output {
    use std::process::Stdio;
    let mut child = Command::new(env!("CARGO_BIN_EXE_sim-cli")).args(args).stdout(Stdio::piped()).stderr(Stdio::piped()).spawn().unwrap();
    let start = std::time::Instant::now();
    while child.try_wait().unwrap().is_none() {
        if start.elapsed() > std::time::Duration::from_secs(60) {
            child.kill().unwrap();
            panic!("{args:?} does not return");
        }
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    child.wait_with_output().unwrap()
}

/// Refused with exit code 2 and a message that names `word`, in both languages, and nothing written.
fn refused(args: &[&str], word: &str) {
    let (en, de) = (cli_returns(args), cli_returns(&[args, &["--lang", "de"]].concat()));
    assert_eq!((en.status.code(), de.status.code()), (Some(2), Some(2)), "{args:?}: {}", text(&en.stderr));
    assert!(text(&en.stderr).contains(word) && text(&de.stderr).contains(word), "{args:?}: {} / {}", text(&en.stderr), text(&de.stderr));
    assert!(text(&en.stderr) != text(&de.stderr) && en.stdout.is_empty(), "{args:?}: {}", text(&en.stderr));
}

/// A misspelt option was ignored (`run MACHINE --cylce` simulated a standing machine and exited 0), of two
/// motions one was dropped, and an option without its value fell back on the default. Each is refused.
#[test]
fn misspelt_conflicting_and_incomplete_options_are_refused() {
    let id = MACHINES[0].id;
    refused(&["run", id, "--cylce"], "--cylce");
    // An option of another command is as unknown as a misspelt one.
    refused(&["verdicts", id, "--cycle"], "--cycle");
    refused(&["list", "--bands"], "--bands");
    refused(&["run", id, "--cycle", "--move", "0.5,0.2"], "--cycle --move");
    refused(&["analyze", id, "--trajectory", "a.csv", "--path", "b.csv"], "--trajectory --path");
    refused(&["run", id, "--cycle", "--time"], "--time");
    refused(&["run", id, "--time", "--cycle"], "--time");
    refused(&["run", id, "--set"], "--set");
    // "inf" parses as a number, and a run for an infinite time never returned.
    refused(&["run", id, "--time", "inf"], "--time inf");
    assert_eq!(cli(&["list", "--lang", "fr"]).status.code(), Some(2));
    // What is spelt right still runs.
    assert!(cli(&["run", id, "--cycle", "--time", "0.01", "--every", "16", "--disable-at", "0.005"]).status.success());
}

/// `sweep --from 0` and `bode --from 0` never returned (a period without end), and a negative frequency
/// gave numbers that mean nothing. A frequency has to be finite and above zero, also in a file for `fit`.
#[test]
fn frequencies_that_are_not_above_zero_are_refused() {
    let id = MACHINES[0].id;
    for command in ["sweep", "bode"] {
        refused(&[command, id, "--from", "0", "--count", "3"], "frequency");
        refused(&[command, id, "--from", "-5", "--to", "-50", "--count", "3"], "frequency");
        refused(&[command, id, "--to", "nan", "--count", "3"], "--to nan");
    }
    let sweep = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("sweep-from-zero.csv");
    std::fs::write(&sweep, "frequency_hz,gain_rad_s_per_nm\n0,1.0\n10,0.1\n").unwrap();
    let fit = cli_returns(&["fit", id, "--sweep", sweep.to_str().unwrap()]);
    assert!(fit.status.code() == Some(2) && text(&fit.stderr).contains("frequency"), "{}", text(&fit.stderr));
}

/// `fit` on a sweep without a resonance dip said so and exited 0. It fails like any other failure, with
/// exit code 2 and the message in the chosen language, after writing what it has.
#[test]
fn fit_without_a_resonance_dip_fails() {
    let id = MACHINES[0].id;
    let sweep = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("sweep-without-dip.csv");
    // A pure inertia: the gain falls with 1 / f and has no dip.
    let rows: Vec<String> = (5..=13).map(|n| format!("{},{}", 10 * n, 1.0 / n as f64)).collect();
    std::fs::write(&sweep, format!("frequency_hz,gain_rad_s_per_nm\n{}\n", rows.join("\n"))).unwrap();
    let (en, de) = (cli(&["fit", id, "--sweep", sweep.to_str().unwrap()]), cli(&["fit", id, "--sweep", sweep.to_str().unwrap(), "--lang", "de"]));
    assert_eq!((en.status.code(), de.status.code()), (Some(2), Some(2)));
    assert!(text(&en.stderr).contains("No stiffness fitted") && text(&de.stderr).contains("Keine Steifigkeit"), "{} / {}", text(&en.stderr), text(&de.stderr));
    assert_eq!(text(&en.stdout).trim(), "quantity,of,config,fitted,standard_error");
}

/// A config file is checked against the machine's default config: a misspelt key was ignored and the
/// default's value used in its place. It is refused by name; the default config as a file passes.
#[test]
fn a_config_file_with_a_key_the_machine_does_not_know_is_refused() {
    for entry in MACHINES {
        let file = |name: &str, text: &str| {
            let path = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{}-{name}.toml", entry.id));
            std::fs::write(&path, text).unwrap();
            path.to_str().unwrap().to_string()
        };
        let good = file("config", entry.default_config);
        assert!(cli(&["run", entry.id, "--config", &good, "--time", "0.01"]).status.success());
        let misspelt = file("misspelt", &entry.default_config.replacen("payload_kg = ", "payload_kq = ", 1));
        refused(&["run", entry.id, "--config", &misspelt, "--time", "0.01"], "mass.payload_kq");
    }
}

/// Transition curves on the command line, where the machine has the two axes an arc needs: a line, a quarter
/// turn of 50 mm radius and a line, towards the middle of the workspace. With `profile.path_tolerance_m` set
/// to 0.1 mm the path takes less time, and the error stream says in the chosen language that the planned path
/// is off the given one by up to 0.1 mm; without the tolerance nothing is said and the path is as given.
#[test]
fn a_path_tolerance_shortens_a_rounded_path_and_the_deviation_is_reported() {
    for entry in MACHINES {
        let sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
        let (axes, home) = (sim.machine.axes(), entry.config().f64s("start.pose").unwrap());
        if axes.len() < 2 {
            continue;
        }
        let towards = |i: usize| (0.5 * (axes[i].min + axes[i].max) - home[i]).signum();
        let (dx, dz) = (towards(0), towards(1));
        let (corner, end) = ([home[0] + 0.15 * dx, home[1]], [home[0] + 0.2 * dx, home[1] + 0.05 * dz]);
        let turn = if dx * dz > 0.0 { "ccw" } else { "cw" };
        let path = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{}-rounded.csv", entry.id)).to_str().unwrap().to_string();
        let pieces = format!("line,{},{}\narc,{},{},{},{},{turn}\nline,{},{}\n", corner[0], corner[1], end[0], end[1], corner[0], end[1], end[0], end[1] + 0.1 * dz);
        std::fs::write(&path, pieces).unwrap();
        let analysed = |extra: &[&str]| -> (f64, String) {
            let out = cli(&[&["analyze", entry.id, "--path", &path], extra].concat());
            assert!(out.status.success(), "{}", text(&out.stderr));
            (text(&out.stdout).lines().nth(1).unwrap().split(',').nth(1).unwrap().parse().unwrap(), text(&out.stderr))
        };
        let tolerance = ["--set", "profile.path_tolerance_m=0.0001"];
        let ((given, silent), (blended, said)) = (analysed(&[]), analysed(&tolerance));
        let (_, gesagt) = analysed(&[&tolerance[..], &["--lang", "de"]].concat());
        assert!(blended < 0.95 * given, "{}: {blended} s with transition curves, {given} s without", entry.id);
        assert!(silent.is_empty() && said.contains("0.1000 mm") && gesagt.contains("0.1000 mm") && said != gesagt, "{}: {silent:?}, {said:?}, {gesagt:?}", entry.id);
        // An arc of 30 degrees whose end is rounded to 0.1 um lies some 10 nm off its circle. That is below what
        // the message can show ("0.0000 mm", which it said), so nothing is said.
        let rounded = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{}-rounded-end.csv", entry.id)).to_str().unwrap().to_string();
        let on_circle = [corner[0] + 0.025 * dx, end[1] - 0.0433013 * dz];
        std::fs::write(&rounded, format!("line,{},{}\narc,{:.7},{:.7},{},{},{turn}\n", corner[0], corner[1], on_circle[0], on_circle[1], corner[0], end[1])).unwrap();
        let off = sim.path_deviation(&sim_components::path::parse(&std::fs::read_to_string(&rounded).unwrap()).unwrap()).unwrap();
        let out = cli(&["analyze", entry.id, "--path", &rounded]);
        assert!(off > 1e-9 && off < 5e-8 && out.status.success() && text(&out.stderr).is_empty(), "{}: off by {off} m, said {:?}", entry.id, text(&out.stderr));
        // A tolerance below zero is no tolerance.
        let refused = cli(&["analyze", entry.id, "--path", &path, "--set", "profile.path_tolerance_m=-0.001"]);
        assert!(refused.status.code() == Some(2) && text(&refused.stderr).contains("profile.path_tolerance_m"), "{}", text(&refused.stderr));
    }
}
