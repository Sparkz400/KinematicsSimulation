//! Batch front end: runs the core as fast as the CPU allows and writes CSV.
//! Every message comes from the translation files; see `cli-usage` there for the commands.

use sim_components::analysis::{analyse_cycle, analyse_trajectory, bandwidth, closed_loop_response, frequency_sweep, log_spaced, step_response};
use sim_components::identify::{fit_rigid, resonance};
use sim_components::trace::{Trace, TraceError, columns, difference};
use sim_components::{Command, Sim};
use sim_core::{Config, ConfigError, MachineEntry, Translations};
use sim_machines::{LANGUAGES, MACHINES, find, translations};
use std::io::Write;

static I18N: &[(&str, &str)] = &[("en", include_str!("../i18n/en.ftl")), ("de", include_str!("../i18n/de.ftl"))];

/// A failure as a translation key and its argument.
struct Failure(&'static str, String);

impl From<ConfigError> for Failure {
    fn from(e: ConfigError) -> Self {
        Failure(e.key(), e.detail().into())
    }
}

impl From<TraceError> for Failure {
    fn from(e: TraceError) -> Self {
        Failure(e.key, e.detail)
    }
}

fn texts(lang: &str, entry: &MachineEntry) -> Translations {
    let mut tr = translations(lang, entry);
    I18N.iter().filter(|(l, _)| *l == lang).for_each(|(_, text)| tr.add(text));
    tr
}

/// Command-line arguments: positional words, and `--name value` options that may repeat.
struct Args(Vec<String>);

/// Options every command takes, each with a value.
const COMMON: [&str; 7] = ["--config", "--motor", "--supply", "--gearbox", "--set", "--out", "--lang"];
/// Options without a value.
const FLAGS: [&str; 3] = ["--cycle", "--time-optimal", "--bands"];
/// The ways to say what the machine should do: one at most.
const MOTIONS: [&str; 4] = ["--cycle", "--move", "--trajectory", "--path"];

impl Args {
    /// Refuses what would otherwise be ignored without a word: an option the command does not know (a
    /// misspelt one), an option without its value, and two motions at once. `known` are the command's own
    /// options beside the common ones.
    fn check(&self, known: &[&str]) -> Result<(), Failure> {
        let mut words = self.0.iter().map(String::as_str).peekable();
        while let Some(word) = words.next() {
            if !word.starts_with("--") {
                continue;
            }
            if !COMMON.contains(&word) && !known.contains(&word) {
                return Err(Failure("cli-error-unknown-option", word.into()));
            }
            if !FLAGS.contains(&word) && words.next_if(|value| !value.starts_with("--")).is_none() {
                return Err(Failure("cli-error-missing-value", word.into()));
            }
        }
        let motions: Vec<&str> = MOTIONS.into_iter().filter(|motion| self.flag(motion)).collect();
        if motions.len() > 1 {
            return Err(Failure("cli-error-conflict", motions.join(" ")));
        }
        match self.option("--lang") {
            Some(lang) if !LANGUAGES.contains(&lang) => Err(Failure("cli-error-bad-argument", format!("--lang {lang}"))),
            _ => Ok(()),
        }
    }

    fn options(&self, name: &str) -> Vec<&str> {
        self.0.windows(2).filter(|w| w[0] == name).map(|w| w[1].as_str()).collect()
    }
    fn option(&self, name: &str) -> Option<&str> {
        self.options(name).pop()
    }
    fn flag(&self, name: &str) -> bool {
        self.0.iter().any(|a| a == name)
    }
    fn number(&self, name: &str) -> Result<Option<f64>, Failure> {
        // "inf" and "nan" parse as numbers, and a run for an infinite time never returns.
        let finite = |v: &str| v.parse().ok().filter(|number: &f64| number.is_finite());
        self.option(name).map(|v| finite(v).ok_or_else(|| Failure("cli-error-bad-argument", format!("{name} {v}")))).transpose()
    }
    /// The n-th word that is neither an option nor an option's value.
    fn positional(&self, n: usize) -> Option<&str> {
        let mut words = Vec::new();
        let mut i = 0;
        while i < self.0.len() {
            if self.0[i].starts_with("--") {
                i += if FLAGS.contains(&self.0[i].as_str()) { 1 } else { 2 };
            } else {
                words.push(self.0[i].as_str());
                i += 1;
            }
        }
        words.get(n).copied()
    }
}

fn main() {
    let args = Args(std::env::args().skip(1).collect());
    let lang = args.option("--lang").filter(|l| LANGUAGES.contains(l)).unwrap_or("en");
    let entry = args.positional(1).and_then(find).unwrap_or(MACHINES[0]);
    let tr = texts(lang, entry);
    if let Err(Failure(key, arg)) = dispatch(&args, &tr) {
        eprintln!("{}", tr.tr_arg(key, &arg));
        std::process::exit(2);
    }
}

fn dispatch(args: &Args, tr: &Translations) -> Result<(), Failure> {
    let known: &[&str] = match args.positional(0) {
        Some("run") => &["--move", "--cycle", "--trajectory", "--path", "--time-optimal", "--time", "--disable-at", "--every"],
        Some("analyze") => &["--trajectory", "--path", "--time-optimal", "--points"],
        Some("sweep") => &["--from", "--to", "--count", "--amplitude"],
        Some("optimise") => &["--runs"],
        Some("search") => &["--series", "--gearboxes"],
        Some("verdicts") => &["--bands", "--samples", "--seed"],
        Some("sensitivity") => &["--delta", "--group"],
        Some("step") => &["--axis", "--size", "--time"],
        Some("bode") => &["--axis", "--from", "--to", "--count", "--amplitude"],
        Some("tuning") => &["--size", "--amplitude"],
        Some("replay") => &["--trace"],
        Some("fit") => &["--trace", "--sweep", "--sweep-motor", "--amplitude"],
        _ => &[],
    };
    args.check(known)?;
    match args.positional(0) {
        Some("list") => {
            let lang = args.option("--lang").unwrap_or("en");
            for entry in MACHINES {
                println!("{}\t{}", entry.id, translations(lang, entry).tr(entry.key));
            }
            Ok(())
        }
        Some("run") => run(args, tr),
        Some("analyze") => analyze(args, tr),
        Some("sweep") => sweep(args),
        Some("sensitivity") => sensitivity(args),
        Some("verdicts") => verdicts(args),
        Some("search") => search(args, tr),
        Some("optimise") => optimise(args),
        Some("step") => step(args),
        Some("bode") => bode(args),
        Some("tuning") => tuning(args),
        Some("replay") => replay(args),
        Some("fit") => fit(args, tr),
        _ => Err(Failure("cli-usage", String::new())),
    }
}

/// The machine named on the command line with its config: the default, or `--config FILE`, with the
/// parts it names or `--motor`, `--supply` and `--gearbox` name put in from the catalogue, then every
/// `--set path=value` applied on top.
fn machine_and_config(args: &Args) -> Result<(&'static MachineEntry, Config), Failure> {
    let id = args.positional(1).ok_or(Failure("cli-usage", String::new()))?;
    let entry = find(id).ok_or(Failure("cli-error-unknown-machine", id.into()))?;
    let mut cfg = match args.option("--config") {
        Some(path) => sim_components::load_config(entry, &read(path)?)?,
        None => entry.config(),
    };
    // `sweep --motor N` picks a motor of the machine by number; an article number is never a plain number.
    let motor = args.option("--motor").filter(|m| m.parse::<usize>().is_err());
    for (key, name) in [("motor", motor), ("motor_supply", args.option("--supply")), ("gearbox", args.option("--gearbox"))] {
        if let Some(name) = name {
            cfg.set_text(&format!("parts.{key} = {name:?}"))?;
        }
    }
    // Parts by name from the catalogue, here and not only in the simulator, so that single values can be set on top.
    let mut cfg = sim_components::catalog::resolve(entry, &cfg)?;
    for assignment in args.options("--set") {
        cfg.set_text(assignment)?;
    }
    Ok((entry, cfg))
}

fn start(args: &Args, tr: &Translations) -> Result<Sim, Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let mut sim = Sim::new(entry, &cfg)?;
    for v in &sim.limit_violations {
        let motor = tr.tr(sim.machine.motors()[v.motor].key);
        eprintln!("{}", tr.tr_arg(v.key, &format!("{motor}: {:.4} > {:.4}", v.needed, v.allowed)));
    }
    let command = if args.flag("--cycle") {
        Some(Command::StartCycle)
    } else if let Some(target) = args.option("--move") {
        let bad = || Failure("cli-error-bad-argument", format!("--move {target}"));
        Some(Command::MoveTo(target.split(',').map(|v| v.trim().parse().map_err(|_| bad())).collect::<Result<_, _>>()?))
    } else {
        poses(args, &sim, tr)?.map(Command::Trajectory)
    };
    if let Some(command) = command {
        sim.command(command).map_err(|e| Failure(e.key, String::new()))?;
    }
    Ok(sim)
}

/// The poses at times to run: those of `--trajectory FILE`, or the lines and arcs of `--path FILE` (format:
/// see `sim_components::path`) with their timing planned here, within the axis limits or with
/// `--time-optimal` as fast as the motors allow. Where transition curves or an arc beside its stated end take the planned path off the lines
/// and arcs of the file (`profile.path_tolerance_m`), the error stream says by how much.
fn poses(args: &Args, sim: &Sim, tr: &Translations) -> Result<Option<Vec<(f64, Vec<f64>)>>, Failure> {
    let unreadable = |file: &str, line: String| Failure("cli-error-bad-argument", format!("{file}: {line}"));
    if let Some(file) = args.option("--trajectory") {
        return sim_components::path::parse_trajectory(&read(file)?).map(Some).map_err(|line| unreadable(file, line));
    }
    let Some(file) = args.option("--path") else { return Ok(None) };
    let pieces = sim_components::path::parse(&read(file)?).map_err(|line| unreadable(file, line))?;
    let refused = |e: sim_components::CommandError| Failure(e.key, String::new());
    let deviation = sim.path_deviation(&pieces).map_err(refused)?;
    if deviation > 0.0 {
        eprintln!("{}", tr.tr_arg("path-deviation", &format!("{:.4}", deviation * 1e3)));
    }
    sim.plan_path(&pieces, args.flag("--time-optimal")).map(Some).map_err(refused)
}

fn read(path: &str) -> Result<String, Failure> {
    std::fs::read_to_string(path).map_err(|e| Failure("cli-error-io", format!("{path}: {e}")))
}

/// `replay <machine> --trace FILE`: the position setpoints of a drive trace run through the simulator.
/// Prints per motor how far simulated torque and following error are from the measured ones; `--out FILE`
/// also writes measured and simulated torque, speed and following error side by side, one line per row
/// of the trace.
fn replay(args: &Args) -> Result<(), Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let motors = (entry.build)(&cfg)?.motors().to_vec();
    let measured = Trace::parse(&read(args.option("--trace").ok_or(Failure("cli-usage", String::new()))?)?, &motors)?;
    let simulated = sim_components::trace::replay(entry, &cfg, &measured)?;
    let pairs = || measured.motors.iter().zip(&simulated.motors);
    let errors: Vec<(Vec<f64>, Vec<f64>)> = pairs().map(|(m, s)| (m.following_error(), s.following_error())).collect();
    println!("motor,torque_rms_nm,torque_peak_nm,following_error_rms_rad,following_error_peak_rad");
    for ((motor, (m, s)), (m_error, s_error)) in motors.iter().zip(pairs()).zip(&errors) {
        let (torque, error) = (difference(&m.torque, &s.torque), difference(m_error, s_error));
        println!("{},{:.6e},{:.6e},{:.6e},{:.6e}", motor.id, torque.0, torque.1, error.0, error.1);
    }
    if args.option("--out").is_some() {
        let mut out = output(args, "--out")?;
        let names = ["torque", "speed", "following_error"];
        let header: Vec<String> = motors.iter().flat_map(|m| names.map(|n| format!("motor.{0}.{n}_measured,motor.{0}.{n}_simulated", m.id))).collect();
        writeln!(out, "time,{}", header.join(",")).map_err(io)?;
        for (row, time) in measured.time.iter().enumerate() {
            let values: Vec<String> = pairs()
                .zip(&errors)
                .flat_map(|((m, s), (m_error, s_error))| [m.torque[row], s.torque[row], m.speed[row], s.speed[row], m_error[row], s_error[row]])
                .map(|v| format!("{v:.9e}"))
                .collect();
            writeln!(out, "{time:.9e},{}", values.join(",")).map_err(io)?;
        }
        out.flush().map_err(io)?;
    }
    Ok(())
}

/// `fit <machine>`: what a measurement says the config should be, as a table of the config's value, the
/// fitted one and its standard error, and on the error stream what to change in the config. With
/// `--trace FILE` the rigid values of each axis; with `--sweep FILE` (a frequency response as `sweep`
/// writes it, taken on motor `--sweep-motor` with `--amplitude`) the load's resonance on the belt.
fn fit(args: &Args, tr: &Translations) -> Result<(), Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let machine = (entry.build)(&cfg)?;
    if args.option("--trace").is_none() && args.option("--sweep").is_none() {
        return Err(Failure("cli-usage", String::new()));
    }
    let mut out = output(args, "--out")?;
    let (mut advice, mut result) = (Vec::new(), Ok(()));
    writeln!(out, "quantity,of,config,fitted,standard_error").map_err(io)?;
    if let Some(path) = args.option("--trace") {
        let measured = Trace::parse(&read(path)?, machine.motors())?;
        for (axis, fit) in machine.axes().iter().zip(fit_rigid(entry, &cfg, &measured)?) {
            let Some(fit) = fit else {
                advice.push(tr.tr_arg("fit-axis-skipped", axis.id));
                continue;
            };
            for (q, name) in ["moved_mass_kg", "viscous_friction_n_s_m", "coulomb_friction_n", "constant_force_n"].iter().enumerate() {
                writeln!(out, "{name},{},{:.6e},{:.6e},{:.3e}", axis.id, fit.config[q], fit.fitted[q], fit.error[q]).map_err(io)?;
            }
            advice.extend(fit.advice(axis.id).iter().map(|(key, detail)| tr.tr_arg(key, detail)));
        }
    }
    if let Some(path) = args.option("--sweep") {
        let table = columns(&read(path)?, &["frequency_hz".to_string(), "gain_rad_s_per_nm".to_string()])?;
        let measured: Vec<_> = table[0].iter().zip(&table[1]).map(|(&frequency, &gain)| sim_components::analysis::SweepPoint { frequency, gain, phase_deg: 0.0 }).collect();
        let motor = number(args, "--sweep-motor", 0.0)? as usize;
        let config = frequency_sweep(entry, &cfg, motor, &table[0], number(args, "--amplitude", 0.05)?)?;
        match (resonance(&config), resonance(&measured)) {
            (Some(config), Some(fitted)) => {
                let id = machine.motors()[motor].id;
                writeln!(out, "resonance_hz,{id},{:.6e},{:.6e},{:.3e}", config.frequency, fitted.frequency, fitted.frequency_error).map_err(io)?;
                writeln!(out, "resonance_damping,{id},{:.6e},{:.6e},", config.damping, fitted.damping).map_err(io)?;
                let stiffness = (fitted.frequency / config.frequency).powi(2);
                advice.push(tr.tr_arg("cli-fit-stiffness", &format!("× {stiffness:.3} ± {:.3}", 2.0 * stiffness * fitted.frequency_error / fitted.frequency)));
            }
            // A failure like any other, after what the trace gave has been written.
            _ => result = Err(Failure("cli-fit-no-resonance", String::new())),
        }
    }
    out.flush().map_err(io)?;
    advice.iter().for_each(|line| eprintln!("{line}"));
    result
}

fn io(e: std::io::Error) -> Failure {
    Failure("cli-error-io", e.to_string())
}

/// Where the CSV goes: the file named by `option`, or standard output.
fn output(args: &Args, option: &str) -> Result<Box<dyn Write>, Failure> {
    Ok(match args.option(option) {
        Some(path) => Box::new(std::io::BufWriter::new(std::fs::File::create(path).map_err(io)?)),
        None => Box::new(std::io::stdout().lock()),
    })
}

/// `optimise <machine>`: a shorter cycle that still passes every verdict, found by trying corner blending
/// and higher axis limits, with input shaping where a faster setting fails on settling alone (`--runs N`
/// cycles at most, 40 by default). Prints the cycle time and what to change in the config.
fn optimise(args: &Args) -> Result<(), Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let runs = args.number("--runs")?.unwrap_or(40.0) as usize;
    let found = sim_components::analysis::optimise_cycle(entry, &cfg, runs)?;
    let mut out = output(args, "--out")?;
    writeln!(out, "setting,before,after").map_err(io)?;
    writeln!(out, "cycle_s,{:.4},{:.4}", found.before.duration, found.after.duration).map_err(io)?;
    writeln!(out, "dwell_error_peak_m,{:.3e},{:.3e}", found.before.dwell_error_peak, found.after.dwell_error_peak).map_err(io)?;
    for (path, before, after) in &found.changes {
        writeln!(out, "{path},{before},{after}").map_err(io)?;
    }
    out.flush().map_err(io)
}

/// `search <machine>`: every motor of the S210 catalogue on the cycle of the config, the passing ones
/// first, lightest first. `--series PREFIX` limits it to article numbers starting that way, `--supply`
/// picks 230v or 400v. `--gearboxes SERIES` (e.g. NRK) also tries each motor with the fitting gearboxes
/// of that series, on a machine that has a gearbox. Says on the error stream how many candidates the
/// catalogue numbers ruled out before any cycle was simulated; they are in the table too.
fn search(args: &Args, tr: &Translations) -> Result<(), Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let prefix = args.option("--series").unwrap_or("");
    let found = sim_components::catalog::search_motors(entry, &cfg, args.option("--supply").unwrap_or("400v"), &|article| article.starts_with(prefix), args.option("--gearboxes"));
    eprintln!("{}", tr.tr_arg("cli-search-screened", &format!("{} / {}", found.iter().filter(|c| c.result.is_none()).count(), found.len())));
    let mut out = output(args, "--out")?;
    writeln!(out, "motor,gearbox,mass_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k").map_err(io)?;
    for c in &found {
        let (check, utilisation) = c.tightest().unwrap_or((c.refused, f64::NAN));
        let (peak, rise) = c.result.as_ref().map_or((f64::NAN, f64::NAN), |(load, _)| {
            (load.motors.iter().map(|m| m.peak_torque).fold(0.0, f64::max), load.motors.iter().map(|m| m.winding_rise).fold(0.0, f64::max))
        });
        writeln!(out, "{},{},{:.2},{},{check},{utilisation:.4},{peak:.4},{rise:.2}", c.article, c.gearbox, c.mass, if c.passes() { "ok" } else { "FAIL" }).map_err(io)?;
    }
    out.flush().map_err(io)
}

/// A value with four significant digits and no exponent: the checks range from fractions of a millimetre
/// to thousands of r/min, and a fixed number of decimals shows the small ones as 0.0000.
fn significant(value: f64) -> String {
    let decimals = if value == 0.0 || !value.is_finite() { 0 } else { (3 - value.abs().log10().floor() as i32).max(0) };
    format!("{value:.*}", decimals as usize)
}

/// `verdicts <machine>`: the cycle judged against the motor data and the `[ratings]` of the config, one
/// line per check and motor. `--bands` adds the lowest and highest value each check takes when the
/// parameters with `[bounds]` in the config are at their bounds, one at a time, and the worst-case result.
/// `--bands --samples N` runs the cycle N times with all of them between their bounds at once instead (see
/// `verdict_bands_sampled`; `--seed S` picks another set of samples, 1 by default) and adds the share of
/// the runs in which each check passed.
fn verdicts(args: &Args) -> Result<(), Failure> {
    use sim_components::analysis::{bounds, verdict_bands, verdict_bands_sampled};
    let (entry, cfg) = machine_and_config(args)?;
    let motors = (entry.build)(&cfg)?.motors().to_vec();
    // Whole numbers: at least one sample, and any seed from 0 on.
    let whole = |name: &str, least: f64| -> Result<Option<f64>, Failure> {
        let value = args.number(name)?;
        let bad = |v: &f64| v.fract() != 0.0 || *v < least || *v > 1e9 || !args.flag("--bands");
        if let Some(v) = value.filter(bad) { Err(Failure("cli-error-bad-argument", format!("{name} {v}"))) } else { Ok(value) }
    };
    let (samples, seed) = (whole("--samples", 1.0)?, whole("--seed", 0.0)?);
    if seed.is_some() && samples.is_none() {
        return Err(Failure("cli-error-missing-value", "--samples".into()));
    }
    let bands = match samples {
        Some(samples) => verdict_bands_sampled(entry, &cfg, &bounds(&cfg)?, samples as usize, seed.unwrap_or(1.0) as u64)?,
        None => verdict_bands(entry, &cfg, &if args.flag("--bands") { bounds(&cfg)? } else { Vec::new() })?,
    };
    let result = |ok: bool| if ok { "ok" } else { "FAIL" };
    let mut out = output(args, "--out")?;
    let share = if samples.is_some() { ",pass_share" } else { "" };
    writeln!(out, "check,motor,value,limit,result{}{share}", if args.flag("--bands") { ",low,high,worst_case" } else { "" }).map_err(io)?;
    for band in &bands {
        let v = &band.verdict;
        let motor = v.motor.map_or("", |k| motors[k].id);
        let worst = if args.flag("--bands") { format!(",{},{},{}", significant(band.low), significant(band.high), result(band.ok)) } else { String::new() };
        let share = if samples.is_some() { format!(",{:.3}", band.share_shown(3)) } else { String::new() };
        writeln!(out, "{},{motor},{},{},{}{worst}{share}", v.check, significant(v.value), significant(v.limit), result(v.ok)).map_err(io)?;
    }
    out.flush().map_err(io)
}

/// `sensitivity <machine>`: the cycle with each parameter at its low and at its high bound of the config,
/// or 20 % (or `--delta`) lower and higher where the config gives it no bounds, one at a
/// time, as relative changes of the results, strongest first. `--group NAME` (repeatable) limits it to
/// config tables; without it every table of machine and motor data is varied.
fn sensitivity(args: &Args) -> Result<(), Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let delta = args.number("--delta")?.unwrap_or(0.2);
    let groups = args.options("--group");
    // Timing, switches, limits, gains and the cycle itself are choices, not uncertain machine data.
    let fixed = ["sim", "effects", "drive", "profile", "start", "axis", "temperature"];
    let paths: Vec<String> = cfg
        .leaves()
        .into_iter()
        .map(|(path, _)| path)
        .filter(|path| path.contains('.'))
        .filter(|path| {
            let group = path.split('.').next().unwrap();
            if groups.is_empty() { !fixed.contains(&group) } else { groups.contains(&group) }
        })
        .collect();
    let (nominal, mut rows) = sim_components::analysis::sensitivity(entry, &cfg, &paths, delta)?;
    let slack = |load: &sim_components::analysis::CycleLoad| load.belt_tension.map_or(0.0, |t| t.0);
    rows.sort_by(|a, b| b.strength(&nominal).total_cmp(&a.strength(&nominal)));
    let mut out = output(args, "--out")?;
    writeln!(out, "parameter,value,low,high,peak_torque_low,peak_torque_high,rms_torque_low,rms_torque_high,cycle_time_low,cycle_time_high,belt_tension_min_low_n,belt_tension_min_high_n")
        .map_err(io)?;
    for row in &rows {
        let n = row.changes(&nominal);
        writeln!(out, "{},{:?},{:?},{:?},{:+.4},{:+.4},{:+.4},{:+.4},{:+.4},{:+.4},{:.2},{:.2}", row.path, row.value, row.bounds.0, row.bounds.1, n[0], n[1], n[2], n[3], n[4], n[5], slack(&row.low), slack(&row.high)).map_err(io)?;
    }
    out.flush().map_err(io)
}

/// `analyze <machine>`: peak and RMS torque, peak speed and use of the torque-speed curve per motor
/// over one pass of the cycle, or over a trajectory or path (see `poses`). `--points FILE` also writes the operating points.
fn analyze(args: &Args, tr: &Translations) -> Result<(), Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let mut sim = Sim::new(entry, &cfg)?;
    let load = match poses(args, &sim, tr)? {
        Some(points) => analyse_trajectory(&mut sim, points),
        None => analyse_cycle(&mut sim),
    }
    .map_err(|e| Failure(e.key, String::new()))?;
    let mut out = output(args, "--out")?;
    writeln!(out, "motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,belt_tension_min_n,belt_tension_max_n,rms_current_a,winding_rise_k,energy_in_j,energy_regenerated_j,peak_regenerated_power_w,largest_regeneration_j,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,cubic_mean_torque_nm,braking_resistor_energy_j,braking_resistor_mean_power_w,dc_link_peak_voltage_v,belt_slack_time_s,radial_load_peak_n,guide_load_peak_n,guide_life_km").map_err(io)?;
    // Empty where the machine does not report the signal or the config has no load rating for the guide.
    let or_empty = |value: Option<f64>| value.map_or(String::new(), |v| format!("{v:.4e}"));
    let machine_wide = format!("{},{},{},{}", or_empty(load.belt_slack_time), or_empty(load.radial_load_peak), or_empty(load.guide_load.map(|g| g.0)), or_empty(sim_components::analysis::guide_life_km(&cfg, &load)));
    for (motor, m) in sim.machine.motors().iter().zip(&load.motors) {
        let belt = load.belt_tension.map_or(String::from(","), |(low, high)| format!("{low:.2},{high:.2}"));
        let electrical = format!("{:.4},{:.2},{:.3},{:.3},{:.2},{:.3}", m.rms_current, m.winding_rise, m.energy_in, m.energy_regenerated, m.peak_regenerated_power, m.largest_regeneration);
        let electrical = format!("{electrical},{:.3e},{:.3e},{:.3e},{:.3e}", load.path_error_peak, load.path_error_rms, load.tracking_error_peak, load.dwell_error_peak);
        let electrical = format!("{electrical},{:.6},{:.3},{:.3},{:.2},{machine_wide}", m.cubic_mean_torque, m.resistor_energy, m.resistor_energy / load.duration, m.peak_dc_link_voltage);
        writeln!(out, "{},{:.6},{:.6},{:.6},{:.4},{:.4},{:.4},{belt},{electrical}", motor.id, load.duration, m.peak_torque, m.rms_torque, m.peak_speed, m.utilisation, m.thermal_utilisation)
            .map_err(io)?;
    }
    out.flush().map_err(io)?;
    if args.option("--points").is_some() {
        let mut out = output(args, "--points")?;
        writeln!(out, "motor,speed_rad_s,torque_nm").map_err(io)?;
        for (motor, m) in sim.machine.motors().iter().zip(&load.motors) {
            for (speed, torque) in &m.points {
                writeln!(out, "{},{speed:.6e},{torque:.6e}", motor.id).map_err(io)?;
            }
        }
        out.flush().map_err(io)?;
    }
    Ok(())
}

/// `sweep <machine>`: mechanical frequency response from one motor's torque to its speed.
fn sweep(args: &Args) -> Result<(), Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let number = |name: &str, default: f64| args.number(name).map(|v| v.unwrap_or(default));
    let frequencies = log_spaced(number("--from", 5.0)?, number("--to", 500.0)?, number("--count", 60.0)? as usize);
    let points = frequency_sweep(entry, &cfg, number("--motor", 0.0)? as usize, &frequencies, number("--amplitude", 0.05)?)?;
    let mut out = output(args, "--out")?;
    writeln!(out, "frequency_hz,gain_rad_s_per_nm,phase_deg").map_err(io)?;
    for p in points {
        writeln!(out, "{:.4},{:.6e},{:.3}", p.frequency, p.gain, p.phase_deg).map_err(io)?;
    }
    out.flush().map_err(io)
}

fn number(args: &Args, name: &str, default: f64) -> Result<f64, Failure> {
    args.number(name).map(|v| v.unwrap_or(default))
}

/// `step <machine>`: the answer of one axis to a small step of its position setpoint.
fn step(args: &Args) -> Result<(), Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let response = step_response(entry, &cfg, number(args, "--axis", 0.0)? as usize, number(args, "--size", 0.001)?, number(args, "--time", 0.5)?)?;
    let mut out = output(args, "--out")?;
    writeln!(out, "time_s,setpoint,response").map_err(io)?;
    for (t, y) in &response.trace {
        writeln!(out, "{t:.6},{:.9e},{y:.9e}", response.size).map_err(io)?;
    }
    out.flush().map_err(io)
}

/// `bode <machine>`: closed-loop frequency response of the position loop of one axis.
fn bode(args: &Args) -> Result<(), Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let frequencies = log_spaced(number(args, "--from", 1.0)?, number(args, "--to", 200.0)?, number(args, "--count", 40.0)? as usize);
    let points = closed_loop_response(entry, &cfg, number(args, "--axis", 0.0)? as usize, &frequencies, number(args, "--amplitude", 0.0005)?)?;
    let mut out = output(args, "--out")?;
    writeln!(out, "frequency_hz,gain,gain_db,phase_deg").map_err(io)?;
    for p in points {
        writeln!(out, "{:.4},{:.6e},{:.3},{:.3}", p.frequency, p.gain, 20.0 * p.gain.log10(), p.phase_deg).map_err(io)?;
    }
    out.flush().map_err(io)
}

/// `tuning <machine>`: one line per axis with what the step and frequency responses say.
fn tuning(args: &Args) -> Result<(), Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let axes: Vec<&str> = (entry.build)(&cfg)?.axes().iter().map(|a| a.id).collect();
    let mut out = output(args, "--out")?;
    writeln!(out, "axis,rise_time_s,overshoot,settling_time_s,bandwidth_hz").map_err(io)?;
    for (i, axis) in axes.iter().enumerate() {
        let step = step_response(entry, &cfg, i, number(args, "--size", 0.001)?, 0.5)?;
        let response = closed_loop_response(entry, &cfg, i, &log_spaced(1.0, 200.0, 30), number(args, "--amplitude", 0.0005)?)?;
        let band = bandwidth(&response).map_or(String::new(), |f| format!("{f:.3}"));
        writeln!(out, "{axis},{:.5},{:.4},{:.5},{band}", step.rise_time, step.overshoot, step.settling_time).map_err(io)?;
    }
    out.flush().map_err(io)
}

/// `run <machine>`: simulate a move or the cycle and write every signal as CSV. `--disable-at SECONDS`
/// switches the drives off at that time, to show the axes held by the brakes, coasting or falling.
fn run(args: &Args, tr: &Translations) -> Result<(), Failure> {
    let mut sim = start(args, tr)?;
    let seconds = args.number("--time")?.unwrap_or(2.0);
    let every = args.number("--every")?.unwrap_or(16.0).max(1.0) as u64;
    let disable_at = args.number("--disable-at")?.map(|seconds| (seconds / sim.dt).round() as u64);
    let mut out = output(args, "--out")?;
    let header: Vec<String> = sim.signal_infos().iter().map(|s| format!("{}[{}]", s.id, s.unit)).collect();
    writeln!(out, "{}", header.join(",")).map_err(io)?;
    let mut sample = Vec::new();
    for step in 0..=(seconds / sim.dt).round() as u64 {
        if step % every == 0 {
            sim.sample(&mut sample);
            let row: Vec<String> = sample.iter().map(|v| format!("{v:.9e}")).collect();
            writeln!(out, "{}", row.join(",")).map_err(io)?;
        }
        if Some(step) == disable_at {
            sim.command(Command::Disable).map_err(|e| Failure(e.key, String::new()))?;
        }
        sim.step();
    }
    out.flush().map_err(io)
}
