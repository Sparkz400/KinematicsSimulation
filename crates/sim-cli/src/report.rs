//! `fit [MACHINE] --dataset DIR --report OUT`: one self-contained report of a dataset folder (datasets/README.md). Every
//! trace recording is replayed on the dataset's machine and config (`machine.simulator`, `machine.config`, or the
//! command line's); the first is the one the free values are fitted on, the others are checks nothing was set from.
//! Every frequency response's resonance is compared with the model's first mode. Written to OUT: `report.md`,
//! `summary.csv` (torque and following error per recording and motor), `fit.csv` (the table of `fit`), `modes.csv`,
//! `bounds.csv`, per recording `<file>-overlay.csv` and per motor `<file>-torque-<motor>.svg`.

use super::{Args, Failure, fit_rows, io, machine_and_config, read};
use sim_components::dataset::contents;
use sim_components::identify::parse_sweep;
use sim_components::import::{Mapping, decode, import};
use sim_components::motor_fit::two_mass;
use sim_components::report::Plot;
use sim_components::trace::{differences, overlay_csv, replay};
use sim_core::Translations;
use std::path::Path;

pub(super) fn run(args: &Args, tr: &Translations) -> Result<(), Failure> {
    let (Some(dir), Some(out)) = (args.option("--dataset"), args.option("--report")) else {
        return Err(Failure("cli-usage", String::new()));
    };
    let (dir, out) = (Path::new(dir), Path::new(out));
    let data = contents(dir).map_err(|e| Failure("dataset-problem", e))?;
    // The machine and config of the command line, else the dataset's.
    let mut words = args.0.clone();
    if args.positional(1).is_none() && args.option("--machine-file").is_none() {
        if data.simulator.is_empty() {
            return Err(Failure("cli-usage", String::new()));
        }
        // A word after the last option's value is the machine, wherever the command stands.
        words.push(data.simulator.clone());
    }
    let config = if args.option("--config").is_none() && !data.config.is_empty() {
        words.extend(["--config".into(), dir.join(&data.config).to_string_lossy().into_owned()]);
        data.config.clone()
    } else {
        args.option("--config").map_or("the machine's default".into(), |c| c.to_string())
    };
    // The dataset's own settings first, so that the command line's come after them and win.
    let at = words.iter().position(|w| w == "--set").unwrap_or(words.len());
    let settings: Vec<String> = data.set.iter().flat_map(|s| ["--set".to_string(), s.clone()]).collect();
    words.splice(at..at, settings);
    let base = Args(words);
    let (entry, cfg) = machine_and_config(&base)?;
    let machine = (entry.build)(&cfg)?;
    let motors = machine.motors().to_vec();
    std::fs::create_dir_all(out).map_err(io)?;
    let write = |name: &str, text: &str| std::fs::write(out.join(name), text).map_err(io);

    let measured_or_not = if data.synthetic { "synthetic (made by a simulator)" } else { "measured" };
    let mut md = format!(
        "# Fit report: {}\n\n{}\n\nDataset `{}`, {measured_or_not}, licence {}. Machine `{}` with the config {config}{}. \
         Written by `sim-cli fit --dataset datasets/{} --report DIR` (sim-cli {}).\n\n",
        data.id,
        data.title,
        data.id,
        data.license,
        entry.id,
        if data.set.is_empty() { String::new() } else { format!(" and `{}`", data.set.join("`, `")) },
        data.id,
        env!("CARGO_PKG_VERSION"),
    );

    // Each trace replayed: its position setpoints through the simulator, the torques compared.
    let traces: Vec<_> = data.recordings.iter().filter(|r| r.test != "frequency-response").collect();
    let mut summary = String::from(
        "recording,role,motor,rows,peak_torque_nm,torque_rms_nm,torque_worst_nm,following_error_rms_rad,following_error_worst_rad\n",
    );
    md += "## Torque, measured against the replay\n\nThe recording's position setpoints replayed (`sim-cli replay`); \
           the difference between measured and simulated motor torque, as RMS and as the worst single sample, and as a \
           share of the recording's peak measured torque. The first recording is the one fitted on, the others are \
           checks: nothing was set from them.\n\n\
           | Recording | Role | Motor | Rows | Peak torque | RMS difference | Worst difference | Following error, RMS difference |\n\
           | --- | --- | --- | --- | --- | --- | --- | --- |\n";
    let mut plots = String::new();
    for (i, r) in traces.iter().enumerate() {
        let role = if i == 0 { "fit" } else { "check" };
        let text =
            decode(&std::fs::read(dir.join(&r.file)).map_err(|e| Failure("cli-error-io", format!("{}: {e}", r.file)))?);
        let measured = import(&text, &Mapping::parse(&read(&dir.join(&r.map).to_string_lossy())?)?)?.trace;
        if measured.motors.len() != motors.len() {
            return Err(Failure(
                "error-import-map",
                format!("{}: {} [[motor]] for {} motors", r.map, measured.motors.len(), motors.len()),
            ));
        }
        let simulated = replay(entry, &cfg, &measured)?;
        let stem = r.file.rsplit_once('.').map_or(r.file.as_str(), |(s, _)| s);
        for (k, d) in differences(&measured, &simulated).into_iter().enumerate() {
            let id = motors[k].id;
            let peak = measured.motors[k].torque.iter().fold(0.0_f64, |a, v| a.max(v.abs()));
            let rows = measured.time.len();
            summary += &format!(
                "{},{role},{id},{rows},{peak:.6e},{:.6e},{:.6e},{:.6e},{:.6e}
",
                r.file, d[0], d[1], d[2], d[3]
            );
            let share = |v: f64| 100.0 * v / peak;
            md += &format!(
                "| {} | {role} | {id} | {rows} | {peak:.3} N m | {:.3} N m, {:.1} % | {:.3} N m, {:.0} % | {:.2e} rad |\n",
                r.file,
                d[0],
                share(d[0]),
                d[1],
                share(d[1]),
                d[2]
            );
            let svg = format!("{stem}-torque-{id}.svg");
            let series = |name: &str, torque: &[f64]| {
                (name.to_string(), measured.time.iter().zip(torque).map(|(t, v)| [*t, *v]).collect(), false)
            };
            let plot = Plot {
                title: format!("{}, motor {id}: torque", r.file),
                x_label: "time [s]".into(),
                y_label: "torque [N m]".into(),
                series: vec![
                    series("measured", &measured.motors[k].torque),
                    series("simulated", &simulated.motors[k].torque),
                ],
            };
            write(&svg, &plot.svg())?;
            plots += &format!("![{}, motor {id}]({svg})\n\n", r.file);
        }
        let ids: Vec<&str> = motors.iter().map(|m| m.id).collect();
        write(&format!("{stem}-overlay.csv"), &overlay_csv(&ids, &measured, &simulated))?;
    }
    md += "\n";
    md += &plots;
    write("summary.csv", &summary)?;

    // The free values fitted on the first recording, against what the config gives on its replay.
    if let Some(first) = traces.first() {
        let mut fit_args = base.0.clone();
        for (option, file) in [("--trace", &first.file), ("--map", &first.map)] {
            fit_args.extend([option.into(), dir.join(file).to_string_lossy().into_owned()]);
        }
        let mut table = Vec::new();
        let (advice, result, _) = fit_rows(&Args(fit_args), tr, &mut table, None)?;
        result?;
        let table = String::from_utf8_lossy(&table).into_owned();
        write("fit.csv", &table)?;
        md += &format!(
            "## Fitted on {}\n\nEach value as the config gives it (the same estimate on the config's replay of the \
             recording) and as the measurement gives it, with the standard error; the last column is how many standard \
             errors apart the two are (within 2: the config agrees with the measurement).\n\n\
             | Quantity | Of | Config | Fitted | Standard error | Apart |\n| --- | --- | --- | --- | --- | --- |\n",
            first.file
        );
        for line in table.lines().skip(1) {
            let cells: Vec<&str> = line.split(',').collect();
            // The sixth column is the unit (`fit_rows`); the table was empty while this took five.
            let [quantity, of, config, fitted, error, _] = cells[..] else { continue };
            let number = |s: &str| s.parse::<f64>().ok();
            let shown = |s: &str| number(s).map_or("-".into(), |v| format!("{v:.4}"));
            let apart = match (number(config), number(fitted), number(error)) {
                (Some(c), Some(f), Some(e)) if e > 0.0 => format!("{:.1}", (f - c).abs() / e),
                _ => "-".into(),
            };
            md += &format!(
                "| {quantity} | {of} | {} | {} | {} | {apart} |\n",
                shown(config),
                shown(fitted),
                number(error).map_or("-".into(), |v| format!("{v:.3}"))
            );
        }
        if !advice.is_empty() {
            md += "\nWhat the fit says to change in the config:\n\n";
            advice.iter().for_each(|line| md += &format!("- {line}\n"));
        }
        md += "\n";
    }

    // The config's error bands: is each value inside the bounds it claims?
    let bounds = sim_components::analysis::bounds(&cfg)?;
    let mut csv = String::from("key,config,low,high,inside\n");
    md += "## The config's bounds\n\nThe `[bounds]` of the config, the error band each value is claimed to lie in. The \
           fitted values above are lumped at the axis (several keys in one), so they are not mapped onto these keys.\n\n\
           | Key | Config | Low | High | Inside |\n| --- | --- | --- | --- | --- |\n";
    for (key, low, high) in &bounds {
        let value = cfg.f64(key)?;
        let inside = if (*low..=*high).contains(&value) { "yes" } else { "**no**" };
        csv += &format!("{key},{value:e},{low:e},{high:e},{}\n", inside == "yes");
        md += &format!("| `{key}` | {value} | {low} | {high} | {inside} |\n");
    }
    if bounds.is_empty() {
        md += "| (none) | | | | |\n";
    }
    write("bounds.csv", &csv)?;
    md += "\n";

    // Each measured frequency response's resonance against the model's first mode at the config's start pose.
    let sweeps: Vec<_> = data.recordings.iter().filter(|r| r.test == "frequency-response").collect();
    if !sweeps.is_empty() {
        let model = machine.modes(&cfg.f64s("start.pose")?).first().map(|m| m.hz);
        let model_text = model.map_or("-".into(), |hz| format!("{hz:.1} Hz"));
        let mut csv = String::from("recording,measured_hz,measured_error_hz,model_hz\n");
        md += "## First mode\n\nThe two-mass model fitted to each measured frequency response (`sim-cli identify \
               --sweep`): its resonance with the standard error, against the model's first mode at the config's start \
               pose (`sim-cli modes`).\n\n| Recording | Measured | Model | Model − measured |\n| --- | --- | --- | --- |\n";
        for r in sweeps {
            let fitted = two_mass(&parse_sweep(&read(&dir.join(&r.file).to_string_lossy())?)?).map(|m| m.resonance);
            let (hz, error) = fitted.map_or((f64::NAN, f64::NAN), |e| (e.value, e.error));
            csv += &format!("{},{hz:.6e},{error:.3e},{:.6e}\n", r.file, model.unwrap_or(f64::NAN));
            let measured = if hz.is_nan() { "no resonance found".into() } else { format!("{hz:.1} ± {error:.1} Hz") };
            let apart = match model {
                Some(m) if !hz.is_nan() => format!("{:+.1} %", 100.0 * (m - hz) / hz),
                _ => "-".into(),
            };
            md += &format!("| {} | {measured} | {model_text} | {apart} |\n", r.file);
        }
        write("modes.csv", &csv)?;
        md += "\n";
    }
    md += "## Files\n\n`summary.csv` the torque table, `fit.csv` the fit, `bounds.csv` the bounds, `modes.csv` the first \
           mode (with frequency responses), per recording `*-overlay.csv` measured and simulated torque, speed and \
           following error row by row, and `*-torque-*.svg` the torque plots.\n";
    write("report.md", &md)?;
    println!("{}", tr.tr_arg("fit-report-written", &out.join("report.md").to_string_lossy()));
    Ok(())
}
