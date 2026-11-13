//! The command line runs every registered machine and writes traces to CSV.

use sim_machines::MACHINES;
use std::process::{Command, Output};

fn cli(args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_sim-cli")).args(args).output().unwrap()
}

fn text(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes).into_owned()
}

/// The config's payload value: `mass.payload_kg` where the machine has a [mass] table, else the arms' payload or the
/// rotary table's payload inertia (the slides have one; the arms and the table have none).
fn payload_key(cfg: &sim_core::Config) -> &'static str {
    ["mass.payload_kg", "arm.payload_kg", "table.payload_inertia_kgm2"].into_iter().find(|k| cfg.get(k).is_ok()).unwrap()
}

/// Whether the config rates a gearbox, so that `verdicts` judges `gearbox-peak` (a screw's or a linear motor's coupling
/// of ratio 1 is not rated).
fn has_gearbox_ratings(cfg: &sim_core::Config) -> bool {
    cfg.get("ratings.gearbox_max_torque_nm").is_ok()
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
        // The middle of each axis; three quarters of the first where it starts in the middle (the SCARA's j1 at 0 rad).
        let home = entry.config().f64s("start.pose").unwrap();
        let target: Vec<String> = sim.machine.axes().iter().enumerate().map(|(i, a)| {
            let middle = 0.5 * (a.min + a.max);
            format!("{}", if i == 0 && (middle - home[0]).abs() < 0.02 { a.min + 0.75 * (a.max - a.min) } else { middle })
        }).collect();
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
        assert!((rows[500][position] - rows[0][position]).abs() > 0.01, "{}: {} to {}", entry.id, rows[0][position], rows[500][position]);
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

/// `--catalog DIR` reads the catalogue from a directory: the repository's own copy runs a search as the built-in one
/// does, and a directory without a catalogue is refused in the chosen language.
#[test]
fn a_catalogue_directory_is_read_and_a_missing_one_refused() {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/../../catalog");
    let args = ["run", MACHINES[0].id, "--motor", "1FK2205-4AF", "--time", "0.01"];
    let (built_in, from_dir) = (cli(&args), cli(&[&args[..], &["--catalog", dir]].concat()));
    assert_eq!(from_dir.status.code(), Some(0), "{}", text(&from_dir.stderr));
    assert_eq!(built_in.stdout, from_dir.stdout);
    let (en, de) = (cli(&[&args[..], &["--catalog", "no-such-dir"]].concat()), cli(&[&args[..], &["--catalog", "no-such-dir", "--lang", "de"]].concat()));
    assert_eq!(en.status.code(), Some(2));
    assert!(text(&en.stderr).contains("catalogue could not be read") && text(&de.stderr).contains("Katalog konnte nicht gelesen"), "{}", text(&en.stderr));
}

#[test]
fn analyze_and_sweep_write_csv_for_every_machine() {
    for entry in MACHINES {
        let motors = sim_components::Sim::new(entry, &entry.config()).unwrap().machine.motors().len();
        let points = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{}-points.csv", entry.id));
        let out = cli(&["analyze", entry.id, "--points", points.to_str().unwrap()]);
        assert!(out.status.success(), "{}", text(&out.stderr));
        let table = text(&out.stdout);
        assert_eq!(table.lines().next().unwrap(), "motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,belt_tension_min_n,belt_tension_max_n,rms_current_a,winding_rise_k,energy_in_j,energy_regenerated_j,peak_regenerated_power_w,largest_regeneration_j,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,cubic_mean_torque_nm,braking_resistor_energy_j,braking_resistor_mean_power_w,dc_link_peak_voltage_v,belt_slack_time_s,radial_load_peak_n,guide_load_peak_n,guide_life_km,mains_energy_j,guide_life_h,pulley_bearing_life_h,gearbox_life_h,belt_tooth_utilisation,gearbox_gearing_life_h,motor_bearing_life_h");
        let rows: Vec<Vec<&str>> = table.lines().skip(1).map(|l| l.split(',').collect()).collect();
        assert_eq!(rows.len(), motors);
        for row in &rows {
            let (peak, rms, use_): (f64, f64, f64) = (row[2].parse().unwrap(), row[3].parse().unwrap(), row[5].parse().unwrap());
            assert!(peak > rms && rms > 0.0 && use_ > 0.0 && use_ < 1.0, "{row:?}");
        }
        // One operating point per millisecond of the cycle (0.72 s on the delta).
        assert!(std::fs::read_to_string(&points).unwrap().lines().count() > 700 * motors);
        // The operating points are those of `analyse_cycle`, which the app shows too.
        let mut sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
        let load = sim_components::analysis::analyse_cycle(&mut sim).unwrap();
        let mut expected = String::from("motor,speed_rad_s,torque_nm
");
        for (motor, m) in sim.machine.motors().iter().zip(&load.motors) {
            m.points.iter().for_each(|(speed, torque)| expected += &format!("{},{speed:.6e},{torque:.6e}
", motor.id));
        }
        assert!(std::fs::read_to_string(&points).unwrap() == expected, "{}: the operating points differ from analyse_cycle's", entry.id);
        // The lives and the tooth load, as the library gives them for the same cycle.
        let cfg = entry.config();
        for (row, m) in rows.iter().zip(&load.motors) {
            let lives = [sim_components::analysis::guide_life_h(&cfg, &load), sim_components::analysis::pulley_bearing_life_h(&cfg, m), sim_components::analysis::gearbox_life_h(&cfg, m), sim_components::analysis::belt_tooth_utilisation(&cfg, m).unwrap(), sim_components::analysis::gearbox_gearing_life_h(&cfg, m), sim_components::analysis::motor_bearing_life_h(&cfg, m)];
            // Empty where the machine has no such part (the delta: no guide, pulley or belt).
            for (cell, life) in row[28..].iter().zip(lives) {
                assert_eq!(*cell, life.map_or(String::new(), |l| format!("{l:.4e}")), "{}: {row:?}", entry.id);
            }
        }

        let out = cli(&["sweep", entry.id, "--from", "10", "--to", "100", "--count", "4"]);
        assert!(out.status.success(), "{}", text(&out.stderr));
        let table = text(&out.stdout);
        assert_eq!(table.lines().count(), 5);
        assert!(table.lines().skip(1).all(|l| l.split(',').all(|v| v.parse::<f64>().is_ok_and(f64::is_finite))));
        // The numbers of `frequency_sweep` at the shared default amplitude, which the app shows too.
        let swept = sim_components::analysis::frequency_sweep(entry, &entry.config(), 0, &sim_components::analysis::log_spaced(10.0, 100.0, 4), sim_components::analysis::default_sweep_amplitude(entry, &entry.config()).unwrap()).unwrap();
        let rows: Vec<String> = swept.iter().map(|p| format!("{:.4},{:.6e},{:.3}", p.frequency, p.gain, p.phase_deg)).collect();
        assert_eq!(table.lines().skip(1).collect::<Vec<_>>(), rows, "{}", entry.id);
    }
}

/// `--axis-index` takes a whole number naming an axis: -1, 1.7 and one past the last axis are refused in words that
/// name the option, on `step` and `bode`, in English and German (-1 ran as 0 and 1.7 as 1, and past the end the
/// error named "axis").
#[test]
fn a_bad_axis_index_is_refused() {
    for entry in MACHINES {
        let axes = sim_components::Sim::new(entry, &entry.config()).unwrap().machine.axes().len();
        for command in ["step", "bode"] {
            for bad in ["-1".to_string(), "1.7".to_string(), axes.to_string()] {
                for (lang, words) in [("en", "whole number from 0 to"), ("de", "ganze Zahl von 0 bis")] {
                    let out = cli(&[command, entry.id, "--axis-index", &bad, "--lang", lang]);
                    let error = text(&out.stderr);
                    assert!(!out.status.success() && error.contains("--axis-index") && error.contains(&format!("{words} {}", axes - 1)), "{} {command} {bad} {lang}: {error}", entry.id);
                }
            }
        }
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
        // The last axis by `--axis-index`; `--axis` names a catalogue axis on every command (it was taken for both,
        // so `step --axis 1` was refused as "The catalogue has no such part: axis 1").
        let last = (axes - 1).to_string();
        let other = table(&["step", entry.id, "--axis-index", &last, "--time", "0.2"]);
        assert_eq!(other.lines().count(), 2 + 800);
        assert!(axes == 1 || other != step, "{}: the last axis steps as the first", entry.id);
        assert_eq!(table(&["bode", entry.id, "--axis-index", &last, "--from", "2", "--to", "50", "--count", "4"]).lines().count(), 5);
        let tuning = table(&["tuning", entry.id]);
        assert_eq!(tuning.lines().count(), 1 + axes);
        // An arm's joints carry their links on elastic gears, six joints of different inertia with one set of drive gains:
        // the band is not Kv's alone (the six-axis arm's j2 has 3.97 Hz, its j1 17 Hz with 27 % overshoot, at velocity
        // loops of some 80 Hz). There half the lower bound is asked.
        let lowest = if entry.config().get("arm.urdf_file").is_ok() { 2.0 } else { 4.0 };
        for row in tuning.lines().skip(1) {
            let cells: Vec<&str> = row.split(',').collect();
            let band: f64 = cells[4].parse().unwrap();
            // Kv = 50 1/s puts the position loop's bandwidth near 8 Hz.
            assert!(band > lowest && band < 20.0, "{}: {row}", entry.id);
        }
        // The margins of the config as it is: one row per motor, the figures of `velocity_loop_margins`.
        let margins = table(&["margins", entry.id]);
        let found = sim_components::analysis::velocity_loop_margins(entry, &entry.config(), &[]).unwrap();
        assert_eq!(margins.lines().next().unwrap(), "run,motor,crossover_hz,phase_margin_deg,gain_margin_db,peak_sensitivity,peak_frequency_hz,result");
        for (row, m) in margins.lines().skip(1).zip(&found[0].1) {
            assert_eq!(row.split(',').nth(3).unwrap(), format!("{:.1}", m.phase_margin_deg), "{}: {row}", entry.id);
            assert!(m.valid && m.phase_margin_deg > 35.0 && m.gain_margin_db > 6.0 && row.ends_with(",ok"), "{}: {row}", entry.id);
        }
        assert_eq!(margins.lines().count(), 1 + found[0].1.len());
    }
    // An unstable loop says so: every machine with a velocity gain of 0.3 N m s/rad (7.5 to 12 times the shipped one).
    // (12 times the shipped one where 0.3 would be less than 7.5 times it: the delta's 0.25 and the screw's, rack's, arms' and table's;
    // at 10 times the screw's loop is still stable, by 6 degrees.)
    for entry in MACHINES {
        let shipped = entry.config().f64("drive.velocity_kp_nm_s_rad").unwrap();
        let gain = if 7.5 * shipped <= 0.3 { 0.3 } else { 12.0 * shipped };
        let out = cli(&["margins", entry.id, "--set", &format!("drive.velocity_kp_nm_s_rad={gain}")]);
        let unstable = text(&out.stdout);
        assert!(out.status.success() && unstable.lines().count() > 1 && unstable.lines().skip(1).all(|row| row.ends_with(",FAIL")), "{}: {unstable}", entry.id);
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
        // The table that holds the payload: [mass], or the rotary table's [table]. An arm's [arm] has one number beside
        // its lists and its URDF, so there its joints' [joint] comes along.
        let cfg = entry.config();
        let groups: &[&str] = match payload_key(&cfg).split('.').next().unwrap() {
            "arm" => &["arm", "joint"],
            group => &[group],
        };
        let run = cli(&[&["sensitivity", entry.id, "--delta", "0.1"][..], &groups.iter().flat_map(|g| ["--group", g]).collect::<Vec<_>>()].concat());
        assert!(run.status.success(), "{}", text(&run.stderr));
        let out = text(&run.stdout);
        let mut lines = out.lines();
        assert!(lines.next().unwrap().starts_with("parameter,value,low,high,peak_torque_low,peak_torque_high"), "{out}");
        let rows: Vec<Vec<&str>> = lines.map(|l| l.split(',').collect()).collect();
        assert!(rows.len() >= 2 && rows.iter().all(|r| groups.contains(&r[0].split('.').next().unwrap()) && r.len() == 12), "{}: {out}", entry.id);
        let strength = |r: &Vec<&str>| r[4..10].iter().map(|v| v.parse::<f64>().unwrap().abs()).fold(0.0, f64::max);
        assert!(rows.windows(2).all(|w| strength(&w[0]) >= strength(&w[1]) - 1e-4), "{}: not ranked: {out}", entry.id);
        // Without bounds in the config the low and high values are the value less and plus 10 %; with bounds, the bounds
        // (the delta bounds its estimated masses).
        let number = |cell: &str| cell.parse::<f64>().unwrap();
        let expected = |r: &Vec<&str>| cfg.f64s(&format!("bounds.{}", r[0])).map_or((0.9 * number(r[1]), 1.1 * number(r[1])), |b| (b[0], b[1]));
        // (An arm's joint angle limits are below zero.)
        assert!(rows.iter().all(|r| (number(r[2]) - expected(r).0).abs() < 1e-9 * number(r[1]).abs() && (number(r[3]) - expected(r).1).abs() < 1e-9 * number(r[1]).abs()), "{out}");
        // With bounds, they are the bounds: 10 N to 20 N for the Coulomb friction of both belt machines' guides.
        if cfg.get("friction_x").is_err() {
            continue;
        }
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
        // The belt's tension where the machine has belts.
        let belted = !sim_components::Sim::new(entry, &entry.config()).unwrap().machine.belt_sections().is_empty();
        // And the gearbox's peak where the config rates a gearbox.
        let geared = has_gearbox_ratings(&entry.config());
        for check in ["motor-peak", "motor-thermal", "gearbox-peak", "dc-link-energy", "belt-tension"].into_iter().filter(|c| (belted || *c != "belt-tension") && (geared || *c != "gearbox-peak")) {
            assert!(rows.iter().any(|r| r[0] == check), "{}: no {check} in {out}", entry.id);
        }
        // As the library judges the cycle: the belt machines pass every check, the delta's catalogue gearboxes fail
        // `settling` (a sizing finding), and its first arm the gearing life (an estimate, see `gearbox_gearing_life_h`).
        let cfg = entry.config();
        let load = sim_components::analysis::analyse_cycle(&mut sim_components::Sim::new(entry, &cfg).unwrap()).unwrap();
        let judged = sim_components::analysis::verdicts(&cfg, &load).unwrap();
        assert_eq!(rows.len(), judged.len(), "{}: {out}", entry.id);
        assert!(rows.iter().zip(&judged).all(|(r, v)| r.len() == 5 && r[0] == v.check && r[4] == if v.ok { "ok" } else { "FAIL" }), "{}: {out}", entry.id);
        // Four significant digits for every value: some tens of micrometres off the target do not print as 0.0000.
        let settling = rows.iter().find(|r| r[0] == "settling").unwrap();
        let (value, limit) = (settling[2].parse::<f64>().unwrap(), settling[3].parse::<f64>().unwrap());
        assert!(value > 1e-6 && (value < limit) == (settling[4] == "ok") && limit == entry.config().f64("ratings.settling_tolerance_m").unwrap() && settling[2].trim_start_matches("0.0").len() >= 4, "{}: {settling:?}", entry.id);

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
        // The gearbox's peak torque, or the motor's where no gearbox is rated.
        let peak = banded.iter().find(|b| b[0] == if geared { "gearbox-peak" } else { "motor-peak" }).unwrap();
        assert!(number(peak[6]) > 1.02 * number(peak[5]) && peak[7] == "ok", "{}: {peak:?}", entry.id);

        // With samples: all bounded values between their bounds at once, and the share of the runs that pass.
        let sampled = |seed: &str| {
            let run = cli(&["verdicts", entry.id, "--bands", "--samples", "6", "--seed", seed]);
            assert!(run.status.success(), "{}", text(&run.stderr));
            text(&run.stdout)
        };
        let out = sampled("1");
        assert_eq!(out.lines().next().unwrap(), "check,motor,value,limit,result,low,high,worst_case,pass_share");
        let samples: Vec<Vec<&str>> = out.lines().skip(1).map(|l| l.split(',').collect()).collect();
        // The rows of the checks the config's own cycle has come first and are its own. A check that only some runs have
        // (the SCARA's motor 3 brakes into its resistor only in some) follows, with the value of the first run that has it.
        assert!(samples.len() >= rows.len() && samples.iter().zip(&rows).all(|(s, r)| s[..5] == r[..]), "{}: {out}", entry.id);
        assert!(samples.iter().all(|s| s.len() == 9 && (0.0..=1.0).contains(&number(s[8]))), "{}: {out}", entry.id);
        assert!(out == sampled("1") && out != sampled("2"), "{}", entry.id);
        // Samples without bands, no samples at all, half a sample and a seed without samples are refused.
        for bad in [&["--samples", "6"][..], &["--bands", "--samples", "0"], &["--bands", "--samples", "2.5"], &["--bands", "--seed", "3"], &["--bands", "--samples"]] {
            let run = cli(&[&["verdicts", entry.id], bad].concat());
            assert!(run.status.code() == Some(2) && run.stdout.is_empty() && !run.stderr.is_empty(), "{}: {bad:?}", entry.id);
        }
    }
}

/// `verdicts --bands --worst` prints per check what `verdict_worst` finds: the worst value, its limit, the result and
/// the bounded values at their high and at their low bound, each named once; the number of cycles goes to the error
/// stream, in German with `--lang de`. Without `--bands`, or with `--samples` or `--seed`, it is refused. On the
/// machine with the fewest bounded values (some 300 cycles).
#[test]
fn verdicts_search_the_worst_corner_of_the_bounds() {
    use sim_components::analysis::{bounds, verdict_worst};
    // The machine with the fewest bounded values, so that the search stays short.
    let entry = MACHINES.iter().copied().min_by_key(|e| bounds(&e.config()).unwrap().len()).unwrap();
    let run = cli(&["verdicts", entry.id, "--bands", "--worst"]);
    assert!(run.status.success(), "{}", text(&run.stderr));
    let (out, cfg) = (text(&run.stdout), entry.config());
    let bounded = bounds(&cfg).unwrap();
    let (worst, runs) = verdict_worst(entry, &cfg, &bounded).unwrap();
    assert_eq!(text(&run.stderr).trim(), format!("Cycles simulated in the search for the worst corners: {runs}"));
    let mut lines = out.lines();
    assert_eq!(lines.next().unwrap(), "check,motor,worst,limit,worst_case,at_high_bound,at_low_bound");
    let rows: Vec<Vec<&str>> = lines.map(|l| l.split(',').collect()).collect();
    assert_eq!(rows.len(), worst.len(), "{out}");
    for (row, w) in rows.iter().zip(&worst) {
        let number = |cell: &str| cell.parse::<f64>().unwrap();
        assert!(row.len() == 7 && row[0] == w.verdict.check && (number(row[2]) / w.verdict.value - 1.0).abs() < 1e-3, "{row:?} against {w:?}");
        assert_eq!(row[4], if w.verdict.ok { "ok" } else { "FAIL" });
        for (bound, high) in bounded.iter().zip(&w.high) {
            let named = |cell: &str| cell.split(';').any(|path| path == bound.0);
            assert!(named(row[5]) == (*high == Some(true)) && named(row[6]) == (*high == Some(false)), "{}: {row:?}", bound.0);
        }
    }
    let german = cli(&["verdicts", entry.id, "--bands", "--worst", "--lang", "de"]);
    assert!(text(&german.stderr).starts_with("Für die Suche nach den ungünstigsten Ecken simulierte Zyklen: ") && text(&german.stdout) == out);
    for bad in [&["--worst"][..], &["--bands", "--worst", "--samples", "6"], &["--bands", "--worst", "--seed", "2"]] {
        let run = cli(&[&["verdicts", entry.id], bad].concat());
        assert!(run.status.code() == Some(2) && run.stdout.is_empty() && text(&run.stderr).contains("--worst"), "{bad:?}: {}", text(&run.stderr));
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
        // Replay and fit take a motor angle per axis travel that is the same everywhere (`identify`): a machine whose
        // transformation is not linear (the delta) is refused, in the chosen language, whatever the trace.
        if !sim_components::trace::linear_transformation(sim.machine.as_ref()) {
            for command in [&["replay", entry.id, "--trace", "any.csv"][..], &["fit", entry.id, "--trace", "any.csv"], &["fit", entry.id, "--sweep", "any.csv"]] {
                for (lang, start) in [("en", "Trace replay and fit need"), ("de", "Nachfahren und Anpassen einer Aufzeichnung brauchen")] {
                    let out = cli(&[command, &["--lang", lang][..]].concat());
                    assert!(!out.status.success() && out.stdout.is_empty() && text(&out.stderr).starts_with(start), "{}: {command:?} {lang}: {}", entry.id, text(&out.stderr));
                }
            }
            continue;
        }
        let file = |name: &str| std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{}-{name}.csv", entry.id)).to_str().unwrap().to_string();
        let (trace, overlay) = (file("trace"), file("overlay"));
        let table = |args: &[&str]| -> Vec<Vec<String>> {
            let out = cli(args);
            assert!(out.status.success(), "{}", text(&out.stderr));
            text(&out.stdout).lines().skip(1).map(|l| l.split(',').map(String::from).collect()).collect()
        };
        // One pass of the cycle moves every axis in both directions. The gripper stays empty, because
        // a trace does not say when a payload comes and goes. And the encoders count without steps: their quantisation noise
        // is not replayed sample by sample (the linear motor's 1 um scale under its stiff velocity loop makes some 10 N of it).
        let no_payload = ["--set", "effects.payload=false", "--set", "effects.encoder_resolution=false"];
        table(&[&["run", entry.id, "--cycle", "--time", "6.2", "--out", &trace], &no_payload[..]].concat());

        let summary = table(&[&["replay", entry.id, "--trace", &trace, "--out", &overlay], &no_payload[..]].concat());
        // The overlay file is `overlay_csv`, which the app exports too.
        let mut cfg = entry.config();
        cfg.set_text("effects.payload = false").unwrap();
        cfg.set_text("effects.encoder_resolution = false").unwrap();
        let measured = sim_components::trace::Trace::parse(&std::fs::read_to_string(&trace).unwrap(), sim.machine.motors()).unwrap();
        let simulated = sim_components::trace::replay(entry, &cfg, &measured).unwrap();
        let ids: Vec<&str> = sim.machine.motors().iter().map(|m| m.id).collect();
        assert!(std::fs::read_to_string(&overlay).unwrap() == sim_components::trace::overlay_csv(&ids, &measured, &simulated), "{}: overlay", entry.id);
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

        // A file that is no trace is refused, in the chosen language.
        let (en, de) = (cli(&["replay", entry.id, "--trace", &overlay]), cli(&["replay", entry.id, "--trace", &overlay, "--lang", "de"]));
        assert_eq!(en.status.code(), Some(2));
        assert!(text(&en.stderr).contains("angle_setpoint") && text(&de.stderr).contains("angle_setpoint") && text(&en.stderr) != text(&de.stderr));

        // Stage 2 on a sweep of the config itself: the dip is where the config has it. Where the first motor has no dip in
        // this range (a belt's is there; the screw's, the table's gear's and a linear motor without transmission are not),
        // stage 2 has nothing to fit.
        let frequencies = sim_components::analysis::log_spaced(50.0, 130.0, 30);
        if sim_components::identify::resonance(&sim_components::analysis::frequency_sweep(entry, &entry.config(), 0, &frequencies, 1.5).unwrap()).is_none() {
            continue;
        }
        let sweep = file("sweep");
        let large = ["--amplitude", "1.5"];
        table(&[&["sweep", entry.id, "--from", "50", "--to", "130", "--count", "30", "--out", &sweep], &large[..]].concat());
        let fit = table(&[&["fit", entry.id, "--sweep", &sweep], &large[..]].concat());
        assert_eq!((fit.len(), fit[0][0].as_str(), fit[1][0].as_str()), (2, "resonance_hz", "resonance_damping"), "{}: {fit:?}", entry.id);
        let (config, fitted): (f64, f64) = (fit[0][2].parse().unwrap(), fit[0][3].parse().unwrap());
        assert!(config > 50.0 && config < 130.0 && (fitted / config - 1.0).abs() < 1e-3, "{}: {fit:?}", entry.id);
        // The numbers of `fit_sweep`, which the app shows too.
        let measured = sim_components::identify::parse_sweep(&std::fs::read_to_string(&sweep).unwrap()).unwrap();
        let shared = sim_components::identify::fit_sweep(entry, &entry.config(), 0, &measured, 1.5).unwrap().unwrap();
        assert_eq!(fit[0][2..], [format!("{:.6e}", shared.config.frequency), format!("{:.6e}", shared.fitted.frequency), format!("{:.3e}", shared.fitted.frequency_error)]);
        assert_eq!(fit[1][2..], [format!("{:.6e}", shared.config.damping), format!("{:.6e}", shared.fitted.damping), String::new()]);
        // The same file with its rows from high to low frequency gives the same fit. It gave a negative damping.
        let written = std::fs::read_to_string(&sweep).unwrap();
        let mut lines: Vec<&str> = written.lines().collect();
        lines[1..].reverse();
        let falling = file("sweep-falling");
        std::fs::write(&falling, lines.join("\n")).unwrap();
        assert_eq!(table(&[&["fit", entry.id, "--sweep", &falling], &large[..]].concat()), fit, "{}", entry.id);
    }
}

/// `--motor` and `--gearbox` name parts for every command; a part the catalogue does not have is refused in
/// the chosen language.
#[test]
fn parts_are_named_on_the_command_line() {
    for entry in MACHINES {
        // A machine without a rated gearbox (a screw's or a linear motor's coupling of ratio 1) has no gearbox to exchange
        // and no `gearbox-peak`: there the motor alone is named, and the use of its curve changes.
        let geared = has_gearbox_ratings(&entry.config());
        let ratio = |extra: &[&str]| -> f64 {
            let out = cli(&[&["verdicts", entry.id], extra].concat());
            assert!(out.status.success(), "{}", text(&out.stderr));
            let table = text(&out.stdout);
            // The torque at the gearbox output over the use of the motor's curve: another motor or ratio changes it.
            let value = |check: &str| table.lines().find(|l| l.starts_with(check)).unwrap().split(',').nth(2).unwrap().parse::<f64>().unwrap();
            if geared { value("gearbox-peak") / value("motor-peak") } else { value("motor-peak") }
        };
        let parts: &[&str] = if geared { &["--motor", "1FK2104-5AK", "--gearbox", "NRK070-04-7"] } else { &["--motor", "1FK2104-5AK"] };
        let (own, named) = (ratio(&[]), ratio(parts));
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
        // The payload's key misspelt, in whichever table the machine has it.
        let key = payload_key(&entry.config());
        let leaf = key.rsplit('.').next().unwrap();
        let misspelt = file("misspelt", &entry.default_config.replacen(&format!("{leaf} = "), &format!("{leaf}x = "), 1));
        refused(&["run", entry.id, "--config", &misspelt, "--time", "0.01"], &format!("{key}x"));
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
        // Transition curves are worked out in the plane: on two axes only.
        if axes.len() != 2 {
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
        let off = sim.path_deviation(&sim_components::path::parse(&std::fs::read_to_string(&rounded).unwrap()).unwrap(), false).unwrap();
        let out = cli(&["analyze", entry.id, "--path", &rounded]);
        assert!(off > 1e-9 && off < 5e-8 && out.status.success() && text(&out.stderr).is_empty(), "{}: off by {off} m, said {:?}", entry.id, text(&out.stderr));
        // A tolerance below zero is no tolerance.
        let refused = cli(&["analyze", entry.id, "--path", &path, "--set", "profile.path_tolerance_m=-0.001"]);
        assert!(refused.status.code() == Some(2) && text(&refused.stderr).contains("profile.path_tolerance_m"), "{}", text(&refused.stderr));
    }
}

/// `analyze --breakdown` and `--budget` write what `analyse_cycle` computes, for every machine: per motor the peak
/// and RMS of each torque part and of the torque, and the energy budget from the supply to what is not accounted
/// for, each with its share of the supply. The budget closes within `BUDGET_TOLERANCE`.
#[test]
fn analyze_writes_the_torque_breakdown_and_the_energy_budget() {
    use sim_components::analysis::{BUDGET_TOLERANCE, analyse_cycle};
    for entry in MACHINES {
        let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR"));
        let (breakdown, budget) = (dir.join(format!("{}-breakdown.csv", entry.id)), dir.join(format!("{}-budget.csv", entry.id)));
        let out = cli(&["analyze", entry.id, "--breakdown", breakdown.to_str().unwrap(), "--budget", budget.to_str().unwrap()]);
        assert!(out.status.success(), "{}: {}", entry.id, text(&out.stderr));
        let mut sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
        let load = analyse_cycle(&mut sim).unwrap();
        let mut expected = String::from("motor,part,peak_torque_nm,rms_torque_nm\n");
        for (motor, m) in sim.machine.motors().iter().zip(&load.motors) {
            for (part, (peak, rms)) in sim_core::TORQUE_PARTS.iter().zip(m.torque_parts.unwrap()) {
                expected += &format!("{},{part},{peak:.6e},{rms:.6e}\n", motor.id);
            }
            expected += &format!("{},torque,{:.6e},{:.6e}\n", motor.id, m.peak_torque, m.rms_torque);
        }
        assert_eq!(std::fs::read_to_string(&breakdown).unwrap(), expected, "{}", entry.id);
        let b = load.budget.unwrap();
        let written = std::fs::read_to_string(&budget).unwrap();
        let lines: Vec<&str> = written.lines().collect();
        assert_eq!(lines.len(), 3 + b.rows.len());
        assert_eq!((lines[0], lines[1]), ("item,energy_j,share", format!("supply,{:.6e},1.000000", b.supply).as_str()));
        for (line, r) in lines[2..].iter().zip(&b.rows) {
            assert_eq!(*line, format!("{},{:.6e},{:.6}", r.id, r.energy, r.energy / b.supply));
        }
        let unaccounted: f64 = lines.last().unwrap().strip_prefix("unaccounted,").unwrap().split(',').next().unwrap().parse().unwrap();
        assert!(unaccounted.abs() < BUDGET_TOLERANCE * b.supply, "{}: {unaccounted} J", entry.id);
        let copper = lines.iter().filter(|l| l.starts_with("copper.")).count();
        let guide = sim.machine.friction_contacts().iter().any(|c| c.id == "axis.x");
        assert!(copper == sim.machine.motors().len() && (!guide || lines.iter().any(|l| l.starts_with("power.guide_x,"))), "{}: {written}", entry.id);
        assert!(!lines.iter().any(|l| l.starts_with("mains,")), "{}: {written}", entry.id);

        // With the regenerative supply, the column `mains_energy_j` of `analyze` is each motor's energy fed back into the mains,
        // and the budget has the rows `mains` and `line_module`.
        let out = cli(&["analyze", entry.id, "--set", "effects.regenerative_supply=true", "--budget", budget.to_str().unwrap()]);
        assert!(out.status.success(), "{}: {}", entry.id, text(&out.stderr));
        let mut cfg = entry.config();
        cfg.set_text("effects.regenerative_supply = true").unwrap();
        let load = analyse_cycle(&mut sim_components::Sim::new(entry, &cfg).unwrap()).unwrap();
        for (line, m) in text(&out.stdout).lines().skip(1).zip(&load.motors) {
            assert!(m.mains_energy > 0.0 && line.split(',').nth(27) == Some(format!("{:.3}", m.mains_energy).as_str()), "{}: {line}", entry.id);
        }
        let written = std::fs::read_to_string(&budget).unwrap();
        let b = load.budget.unwrap();
        for id in ["mains", "line_module"] {
            let energy = b.rows.iter().find(|r| r.id == id).unwrap().energy;
            assert!(written.lines().any(|l| l == format!("{id},{energy:.6e},{:.6}", energy / b.supply)), "{}: {written}", entry.id);
        }
    }
}

/// `compare` writes the cycle with an effect on and off side by side, and the differences as `Comparison` has them.
#[test]
fn compare_writes_the_cycle_with_an_effect_on_and_off() {
    use sim_components::analysis::compare_effect;
    for entry in MACHINES {
        let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR"));
        let (traces, differences) = (dir.join(format!("{}-compare.csv", entry.id)), dir.join(format!("{}-differences.csv", entry.id)));
        let sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
        let motor = sim.machine.motors()[0].id;
        let torque = format!("motor.{motor}.torque");
        // The first axis's position: not every machine has an axis x (the rotary table's c, the arms' j1), nor metres.
        let position = format!("axis.{}.position", sim.machine.axes()[0].id);
        let unit = sim.signal_infos().iter().find(|s| s.id == position).unwrap().unit.clone();
        // The friction the machine has: of its guides, or of its arms' joints.
        let effect = if entry.config().bool("effects.guide_friction").is_ok() { "guide_friction" } else { "joint_friction" };
        let out = cli(&["compare", entry.id, "--effect", effect, "--signal", &torque, "--signal", &position, "--out", traces.to_str().unwrap(), "--differences", differences.to_str().unwrap()]);
        assert!(out.status.success(), "{}: {}", entry.id, text(&out.stderr));
        let found = compare_effect(entry, &entry.config(), effect).unwrap();
        let place = |id: &str| found.signals.iter().position(|s| s.id == id).unwrap();
        let written = std::fs::read_to_string(&traces).unwrap();
        let lines: Vec<&str> = written.lines().collect();
        assert_eq!(lines[0], format!("time[s],{torque}.on[N m],{torque}.off[N m],{position}.on[{unit}],{position}.off[{unit}]"));
        assert_eq!(lines.len(), 1 + found.on.len().min(found.off.len()));
        let (on, off) = (&found.on[100], &found.off[100]);
        let (t, p) = (place(&torque), place(&position));
        assert_eq!(lines[101], format!("{:.4},{:.9e},{:.9e},{:.9e},{:.9e}", on[0], on[t], off[t], on[p], off[p]));
        let (peak, rms) = found.difference(t);
        let written = std::fs::read_to_string(&differences).unwrap();
        assert_eq!(written.lines().nth(1).unwrap(), format!("{torque},N m,{peak:.6e},{rms:.6e}"));
        // An effect the config does not have is refused by name.
        let out = cli(&["compare", entry.id, "--effect", "no_such_effect"]);
        assert!(!out.status.success() && text(&out.stderr).contains("effects.no_such_effect"), "{}", text(&out.stderr));
    }
}

/// The XML elements of a text, checked for balance: (name, attributes) of every opening and empty one.
fn xml_elements(xml: &str) -> Vec<(String, String)> {
    let (mut open, mut out) = (Vec::new(), Vec::new());
    for tag in xml.split('<').skip(1).map(|t| t.split_once('>').expect("every tag closes").0) {
        if tag.starts_with('?') {
            continue;
        }
        if let Some(name) = tag.strip_prefix('/') {
            assert_eq!(open.pop().as_deref(), Some(name), "unbalanced </{name}>");
            continue;
        }
        let (name, attributes) = tag.trim_end_matches('/').split_once(' ').unwrap_or((tag.trim_end_matches('/'), ""));
        out.push((name.to_string(), attributes.to_string()));
        if !tag.ends_with('/') {
            open.push(name.to_string());
        }
    }
    assert!(open.is_empty(), "not closed: {open:?}");
    out
}

fn attribute<'a>(attributes: &'a str, name: &str) -> Option<&'a str> {
    attributes.split_once(&format!(" {name}=\"")).or_else(|| attributes.strip_prefix(&format!("{name}=\"")).map(|rest| ("", rest))).map(|(_, rest)| rest.split('"').next().unwrap())
}

/// The files of a zip archive of stored entries, read through its local headers and checked against its central directory.
fn unzip(bytes: &[u8]) -> Vec<(String, Vec<u8>)> {
    let u16_at = |i: usize| u16::from_le_bytes([bytes[i], bytes[i + 1]]) as usize;
    let u32_at = |i: usize| u32::from_le_bytes(bytes[i..i + 4].try_into().unwrap());
    let (mut files, mut at) = (Vec::new(), 0);
    while u32_at(at) == 0x0403_4b50 {
        assert_eq!(u16_at(at + 8), 0, "stored, not compressed");
        let (crc, size, name_len, extra) = (u32_at(at + 14), u32_at(at + 18) as usize, u16_at(at + 26), u16_at(at + 28));
        let name = String::from_utf8(bytes[at + 30..at + 30 + name_len].to_vec()).unwrap();
        let data = bytes[at + 30 + name_len + extra..at + 30 + name_len + extra + size].to_vec();
        assert_eq!(sim_fmi::crc32(&data), crc, "{name}");
        files.push((name, data));
        at += 30 + name_len + extra + size;
    }
    let end = bytes.len() - 22;
    assert_eq!(u32_at(end), 0x0605_4b50, "end of central directory");
    assert_eq!((u16_at(end + 10), u32_at(end + 16) as usize), (files.len(), at), "central directory");
    let mut central = at;
    for (name, _) in &files {
        assert_eq!(u32_at(central), 0x0201_4b50);
        let name_len = u16_at(central + 28);
        assert_eq!(&bytes[central + 46..central + 46 + name_len], name.as_bytes());
        central += 46 + name_len;
    }
    assert_eq!(central, end);
    files
}

/// `fmu` writes a zip with the FMI layout, and its model description lists every signal with a unique value reference
/// and its unit, the setpoint inputs and the parameters.
#[test]
fn fmu_writes_an_archive_with_the_fmi_layout_and_every_signal() {
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR"));
    let (dll, so) = (dir.join("sim_fmi.dll"), dir.join("libsim_fmi.so"));
    std::fs::write(&dll, b"windows binary").unwrap();
    std::fs::write(&so, b"linux binary").unwrap();
    for entry in MACHINES {
        let path = dir.join(format!("{}.fmu", entry.id));
        let key = payload_key(&entry.config());
        let out = cli(&["fmu", entry.id, "--set", &format!("{key}=2.5"), "--binary", dll.to_str().unwrap(), "--binary", so.to_str().unwrap(), "--out", path.to_str().unwrap()]);
        assert!(out.status.success(), "{}: {}", entry.id, text(&out.stderr));
        let files = unzip(&std::fs::read(&path).unwrap());
        let names: Vec<&str> = files.iter().map(|(n, _)| n.as_str()).collect();
        assert_eq!(names, ["modelDescription.xml", "resources/model.txt", "resources/config.toml", "binaries/win64/sim_fmi.dll", "binaries/linux64/sim_fmi.so"]);
        assert_eq!(files[3].1, b"windows binary");
        assert_eq!(files[2].1, sim_core::Config::parse(&text(&files[2].1)).unwrap().to_text().unwrap().into_bytes());
        assert!(text(&files[2].1).contains(&format!("{} = 2.5", key.rsplit('.').next().unwrap())), "the config as set on the command line");

        let xml = text(&files[0].1);
        let elements = xml_elements(&xml);
        let root = &elements[0];
        assert_eq!((root.0.as_str(), attribute(&root.1, "fmiVersion")), ("fmiModelDescription", Some("2.0")));
        let model = sim_fmi::Model::new(entry, &text(&files[2].1), "en").unwrap();
        assert_eq!(attribute(&root.1, "guid"), Some(model.guid.as_str()));
        let units: Vec<&str> = elements.iter().filter(|(n, _)| n == "Unit").filter_map(|(_, a)| attribute(a, "name")).collect();
        // (name, value reference, causality) of each variable, and the unit of its type element.
        let mut variables = Vec::new();
        for (i, (name, attributes)) in elements.iter().enumerate().filter(|(_, (n, _))| n == "ScalarVariable") {
            let vr: usize = attribute(attributes, "valueReference").unwrap().parse().unwrap();
            let unit = attribute(&elements[i + 1].1, "unit").unwrap_or("");
            assert!(unit.is_empty() || units.contains(&unit), "{name}: unit {unit} not defined");
            variables.push((attribute(attributes, "name").unwrap().to_string(), vr, attribute(attributes, "causality").unwrap().to_string(), unit.to_string()));
        }
        let mut refs: Vec<usize> = variables.iter().map(|v| v.1).collect();
        refs.sort();
        refs.dedup();
        assert_eq!(refs.len(), variables.len(), "value references are unique");
        let sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
        for (k, signal) in sim.signal_infos().iter().enumerate() {
            let v = variables.iter().find(|v| v.0 == signal.id).unwrap_or_else(|| panic!("{}: no variable {}", entry.id, signal.id));
            assert_eq!((v.2.as_str(), v.3.as_str()), (if k == 0 { "independent" } else { "output" }, signal.unit.as_str()), "{}", signal.id);
        }
        for axis in sim.machine.axes() {
            assert!(variables.iter().any(|v| v.0 == format!("command.setpoint.{}", axis.id) && v.2 == "input"));
        }
        assert!(variables.iter().any(|v| v.0 == "config_file" && v.2 == "parameter"));
        // Outputs name exactly the output variables, by their place counted from one.
        let structure = xml.split("<Outputs>").nth(1).unwrap().split("</Outputs>").next().unwrap();
        let outputs: Vec<usize> = xml_elements(structure).iter().map(|(_, a)| attribute(a, "index").unwrap().parse().unwrap()).collect();
        assert_eq!(outputs.len(), sim.signal_infos().len() - 1);
        assert!(outputs.iter().all(|i| variables[i - 1].2 == "output"));
    }
    // Without --binary, the library built next to the program, or a message that says how to build it.
    let path = dir.join("default.fmu");
    let out = cli(&["fmu", MACHINES[0].id, "--out", path.to_str().unwrap()]);
    let beside = std::path::Path::new(env!("CARGO_BIN_EXE_sim-cli")).with_file_name(format!("{}sim_fmi{}", std::env::consts::DLL_PREFIX, std::env::consts::DLL_SUFFIX));
    assert_eq!(out.status.success(), beside.exists(), "{}", text(&out.stderr));
    assert!(out.status.success() || text(&out.stderr).contains("-p sim-fmi"));
    // A binary of no known platform, and no output file.
    refused(&["fmu", MACHINES[0].id, "--binary", "sim_fmi.txt", "--out", path.to_str().unwrap()], "sim_fmi.txt");
    refused(&["fmu", MACHINES[0].id], "--out");
    // Two binaries for one platform, which went into the archive as two entries of one name.
    let other = dir.join("other").join("sim_fmi.dll");
    std::fs::create_dir_all(other.parent().unwrap()).unwrap();
    std::fs::write(&other, b"another windows binary").unwrap();
    let twice = dir.join("twice.fmu");
    let _ = std::fs::remove_file(&twice);
    refused(&["fmu", MACHINES[0].id, "--binary", dll.to_str().unwrap(), "--binary", other.to_str().unwrap(), "--out", twice.to_str().unwrap()], "--binary");
    assert!(!twice.exists());
}

/// `fmu --fmi 3` writes an FMI 3.0 FMU: its model description with the instantiation token, and each binary in the
/// folder of its architecture, read from its header; a binary without a known header and another version are refused.
#[test]
fn fmu_writes_an_fmi3_archive_with_a_folder_per_architecture() {
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("fmi3");
    std::fs::create_dir_all(&dir).unwrap();
    let mut pe = vec![0u8; 0x48];
    pe[..2].copy_from_slice(b"MZ");
    pe[0x3c] = 0x40;
    pe[0x40..0x44].copy_from_slice(b"PE\0\0");
    pe[0x44..0x46].copy_from_slice(&0x8664u16.to_le_bytes());
    let mut elf = b"\x7fELF".to_vec();
    elf.resize(20, 0);
    elf[18] = 62;
    let macho = [[0xcf, 0xfa, 0xed, 0xfe], 0x0100_000cu32.to_le_bytes()].concat();
    let (dll, so, dylib) = (dir.join("sim_fmi.dll"), dir.join("libsim_fmi.so"), dir.join("libsim_fmi.dylib"));
    std::fs::write(&dll, &pe).unwrap();
    std::fs::write(&so, &elf).unwrap();
    std::fs::write(&dylib, &macho).unwrap();
    let path = dir.join("fmi3.fmu");
    let binaries = ["--binary", dll.to_str().unwrap(), "--binary", so.to_str().unwrap(), "--binary", dylib.to_str().unwrap()];
    let out = cli(&[&["fmu", MACHINES[0].id, "--fmi", "3", "--out", path.to_str().unwrap()][..], &binaries].concat());
    assert!(out.status.success(), "{}", text(&out.stderr));
    let files = unzip(&std::fs::read(&path).unwrap());
    let names: Vec<&str> = files.iter().map(|(n, _)| n.as_str()).collect();
    assert_eq!(names[3..], ["binaries/x86_64-windows/sim_fmi.dll", "binaries/x86_64-linux/sim_fmi.so", "binaries/aarch64-darwin/sim_fmi.dylib"]);
    let xml = text(&files[0].1);
    let root = &xml_elements(&xml)[0];
    assert_eq!(attribute(&root.1, "fmiVersion"), Some("3.0"));
    let model = sim_fmi::Model::with_fmi(MACHINES[0], &text(&files[2].1), "en", 3).unwrap();
    assert_eq!(attribute(&root.1, "instantiationToken"), Some(model.guid.as_str()));
    assert_eq!(xml, model.xml);
    // A file that is no binary of a known architecture; a version that does not exist.
    let fake = dir.join("fake").join("sim_fmi.dll");
    std::fs::create_dir_all(fake.parent().unwrap()).unwrap();
    std::fs::write(&fake, b"windows binary").unwrap();
    refused(&["fmu", MACHINES[0].id, "--fmi", "3", "--binary", fake.to_str().unwrap(), "--out", path.to_str().unwrap()], "sim_fmi.dll");
    refused(&["fmu", MACHINES[0].id, "--fmi", "4", "--binary", dll.to_str().unwrap(), "--out", path.to_str().unwrap()], "--fmi 4");
}

/// `--machine-file` takes the place of the machine's id: the H-bot of docs/machine-files.md is run and judged, and a
/// broken file is refused with its file, line and key. That machine files give their crates' results is checked in
/// sim-machines (tests/machine_files.rs).
#[test]
fn a_machine_file_is_run_judged_and_checked() {
    let file = concat!(env!("CARGO_MANIFEST_DIR"), "/../../machines/h-bot.toml");
    let verdicts = cli(&["verdicts", "--machine-file", file]);
    assert_eq!(verdicts.status.code(), Some(0), "{}", text(&verdicts.stderr));
    let rows: Vec<String> = text(&verdicts.stdout).lines().map(str::to_string).collect();
    assert!(rows.len() > 20 && rows.iter().skip(1).all(|r| r.ends_with(",ok")), "{rows:?}");
    let run = cli(&["run", "--machine-file", file, "--move", "0.5,0.2", "--time", "0.01"]);
    assert_eq!(run.status.code(), Some(0), "{}", text(&run.stderr));
    assert!(text(&run.stdout).starts_with("time[s]") && text(&run.stdout).contains("belt.3.tension"));

    let dir = std::env::temp_dir().join(format!("sim-cli-machine-file-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let broken = dir.join("broken.toml");
    let original = std::fs::read_to_string(file).unwrap();
    std::fs::write(&broken, original.replacen("friction = \"friction_z\"", "fricton = \"friction_z\"", 1)).unwrap();
    let line = original.lines().position(|l| l == "friction = \"friction_z\"").unwrap() + 1;
    let refused = cli(&["verdicts", "--machine-file", broken.to_str().unwrap()]);
    assert_eq!(refused.status.code(), Some(2));
    assert!(text(&refused.stderr).contains(&format!("broken.toml:{line}: machine.guide[guide_y].fricton")), "{}", text(&refused.stderr));
    assert_eq!(cli(&["fmu", "--machine-file", file, "--out", dir.join("x.fmu").to_str().unwrap()]).status.code(), Some(2));
    std::fs::remove_dir_all(dir).ok();
}

/// `report` writes the sizing report as HTML or PDF: the verdicts are those `verdicts` prints, and each input says where
/// it comes from.
#[test]
fn report_lists_every_verdict_and_the_source_of_each_input() {
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR"));
    for entry in MACHINES {
        let html = dir.join(format!("{}-report.html", entry.id));
        let out = cli(&["report", entry.id, "--out", html.to_str().unwrap()]);
        assert!(out.status.success(), "{}", text(&out.stderr));
        let html = std::fs::read_to_string(html).unwrap();
        let verdicts = cli(&["verdicts", entry.id]);
        let rows: Vec<Vec<String>> = text(&verdicts.stdout).lines().skip(1).map(|l| l.split(',').map(String::from).collect()).collect();
        assert!(!rows.is_empty());
        for row in &rows {
            // Value, limit and result as `verdicts` has them, in that order, in the row of the report.
            let cells = format!("<td>{}</td><td>{}</td><td>", row[2], row[3]);
            assert!(html.contains(&cells), "{}: {cells} missing", entry.id);
        }
        let failed = rows.iter().filter(|r| r[4] == "FAIL").count();
        assert_eq!(html.matches("<tr class=\"fail\">").count(), failed, "{}", entry.id);
        assert_eq!(html.matches("<svg").count(), 3);
        assert!(!html.contains("<td>check-"), "{}: a check without its label", entry.id);
    }
    // A PDF, in German, with a value set on top.
    let (id, pdf) = (MACHINES[0].id, dir.join("report.pdf"));
    let out = cli(&["report", id, "--set", "mass.payload_kg=2.5", "--lang", "de", "--out", pdf.to_str().unwrap()]);
    assert!(out.status.success(), "{}", text(&out.stderr));
    let pdf = std::fs::read(pdf).unwrap();
    assert!(pdf.starts_with(b"%PDF-1.4") && pdf.ends_with(b"%%EOF\n"));
    assert!(pdf.windows(b"(Freigabe)".len()).any(|w| w == b"(Freigabe)"), "German headings in the PDF");
    assert!(pdf.windows(b"(set for this run)".len()).all(|w| w != b"(set for this run)"));
    let unknown = cli(&["report", id]);
    assert!(!unknown.status.success(), "a report needs --out");
}

/// `modes` writes the reported modes over an axis's stroke for every machine (a header alone after "position" where the
/// machine reports none). The single axis's carriage on its belt, m = 6 kg on k = EA / L1 + EA / L2 with EA = 5e5 N:
/// at x = 0, L = 0.15 m and 2.45 m, k = 3.537415e6 N/m, sqrt(k / m) / (2 pi) = 122.2046 Hz; at x = 0.5 m, 65.8025 Hz;
/// at x = 1 m, L = 1.15 m and 1.45 m, k = 7.796102e5 N/m, 57.3698 Hz. Damping ratio 200 N s/m / (2 m w): 0.021706,
/// 0.040311, 0.046237.
#[test]
fn modes_write_the_resonances_over_the_stroke() {
    for entry in MACHINES {
        let out = cli(&["modes", entry.id, "--points", "4"]);
        assert!(out.status.success(), "{}: {}", entry.id, text(&out.stderr));
        let table = text(&out.stdout);
        assert!(table.starts_with("position") && table.lines().count() == 5, "{}: {table}", entry.id);
    }
    let out = cli(&["modes", "single-axis", "--points", "3"]);
    let rows: Vec<Vec<f64>> = text(&out.stdout).lines().skip(1).map(|l| l.split(',').map(|v| v.parse().unwrap()).collect()).collect();
    for (row, by_hand) in rows.iter().zip([[0.0, 122.2046, 0.021706], [0.5, 65.8025, 0.040311], [1.0, 57.3698, 0.046237]]) {
        assert!(row.iter().zip(by_hand).all(|(a, b)| (a - b).abs() < 1e-3 * b.max(0.01)), "{row:?} against {by_hand:?} by hand");
    }
    assert!(!cli(&["modes", "single-axis", "--points", "1"]).status.success());
}
