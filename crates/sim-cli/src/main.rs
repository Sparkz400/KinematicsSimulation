//! Batch front end: runs the core as fast as the CPU allows and writes CSV.
//! Every message comes from the translation files; see `cli-usage` there for the commands.

use sim_components::analysis::{SWEEP_POINTS, analyse_cycle_with, analyse_trajectory_with, bandwidth, closed_loop_response, default_sweep_amplitude, frequency_sweep, log_spaced, step_response};
use sim_components::identify::{fit_rigid, fit_sweep, parse_sweep};
use sim_components::import::{Imported, Mapping, decode, import};
use sim_components::motor_fit::{Estimate, Identified, TwoMass, frequency_response, ringing, two_mass, two_mass_from_ringing};
use sim_components::trace::{Trace, TraceError, differences, overlay_csv};
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
const COMMON: [&str; 11] = ["--machine-file", "--config", "--motor", "--supply", "--gearbox", "--drive", "--belt", "--axis", "--set", "--out", "--lang"];
/// Options without a value.
const FLAGS: [&str; 5] = ["--cycle", "--time-optimal", "--bands", "--worst", "--axes"];
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
    let entry = machine(&args).unwrap_or(MACHINES[0]);
    let tr = texts(lang, entry);
    if let Err(Failure(key, arg)) = dispatch(&args, &tr) {
        eprintln!("{}", tr.tr_arg(key, &arg));
        std::process::exit(2);
    }
}

fn dispatch(args: &Args, tr: &Translations) -> Result<(), Failure> {
    let known: &[&str] = match args.positional(0) {
        Some("run") => &["--move", "--cycle", "--trajectory", "--path", "--time-optimal", "--time", "--disable-at", "--every"],
        Some("analyze") => &["--trajectory", "--path", "--time-optimal", "--points", "--breakdown", "--budget"],
        Some("sweep") => &["--from", "--to", "--count", "--amplitude"],
        Some("optimise") => &["--runs"],
        Some("search") => &["--series", "--gearboxes", "--axes"],
        Some("verdicts") => &["--bands", "--samples", "--seed", "--worst"],
        Some("sensitivity") => &["--delta", "--group"],
        Some("step") => &["--axis-index", "--size", "--time"],
        Some("bode") => &["--axis-index", "--from", "--to", "--count", "--amplitude"],
        Some("tuning") => &["--size", "--amplitude"],
        Some("margins") => &["--bands"],
        Some("replay") => &["--trace", "--map"],
        Some("fit") => &["--trace", "--map", "--sweep", "--sweep-motor", "--amplitude"],
        Some("identify") => &["--trace", "--map", "--from", "--to", "--sweep", "--step-from", "--step-to", "--motor-inertia", "--overlay"],
        Some("compare") => &["--effect", "--signal", "--differences"],
        Some("fmu") => &["--binary", "--fmi"],
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
        Some("verdicts") => verdicts(args, tr),
        Some("search") => search(args, tr),
        Some("optimise") => optimise(args),
        Some("step") => step(args),
        Some("bode") => bode(args),
        Some("tuning") => tuning(args),
        Some("margins") => margins(args),
        Some("replay") => replay(args),
        Some("fit") => fit(args, tr),
        Some("identify") => identify(args, tr),
        Some("check-dataset") => check_dataset(args, tr),
        Some("compare") => compare(args),
        Some("fmu") => fmu(args),
        _ => Err(Failure("cli-usage", String::new())),
    }
}

/// The machine `--machine-file FILE` defines (docs/machine-files.md), or else the registered one named after the
/// command. A file's errors name the file and the line.
fn machine(args: &Args) -> Result<&'static MachineEntry, Failure> {
    if let Some(path) = args.option("--machine-file") {
        return sim_machines::load_file(std::path::Path::new(path)).map_err(|e| {
            let at = e.line.map_or(String::new(), |line| format!(":{line}"));
            Failure(e.error.key(), format!("{}{at}: {}", e.file.display(), e.error.detail()))
        });
    }
    let id = args.positional(1).ok_or(Failure("cli-usage", String::new()))?;
    find(id).ok_or(Failure("cli-error-unknown-machine", id.into()))
}

/// The machine named on the command line with its config: the default, or `--config FILE`, with the
/// parts it names or `--motor`, `--supply`, `--gearbox`, `--drive`, `--belt` and `--axis` name put in from the catalogue, then every
/// `--set path=value` applied on top.
fn machine_and_config(args: &Args) -> Result<(&'static MachineEntry, Config), Failure> {
    let entry = machine(args)?;
    let mut cfg = match args.option("--config") {
        Some(path) => sim_components::load_config(entry, &read(path)?)?,
        None => entry.config(),
    };
    // `sweep --motor N` picks a motor of the machine by number; an article number is never a plain number.
    let motor = args.option("--motor").filter(|m| m.parse::<usize>().is_err());
    let named = [("motor", motor), ("motor_supply", args.option("--supply")), ("gearbox", args.option("--gearbox")), ("drive", args.option("--drive")), ("belt", args.option("--belt")), ("axis", args.option("--axis"))];
    for (key, name) in named {
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
    let deviation = sim.path_deviation(&pieces, args.flag("--time-optimal")).map_err(refused)?;
    // Not below what the figure can show: an arc with decimally rounded end coordinates is some 10 nm off.
    let shown = format!("{:.4}", deviation * 1e3);
    if shown != "0.0000" {
        eprintln!("{}", tr.tr_arg("path-deviation", &shown));
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
    let machine = (entry.build)(&cfg)?;
    if !sim_components::trace::linear_transformation(machine.as_ref()) {
        return Err(Failure("error-trace-not-linear", String::new()));
    }
    let motors = machine.motors().to_vec();
    let measured = load_trace(args, &motors)?.trace;
    let simulated = sim_components::trace::replay(entry, &cfg, &measured)?;
    println!("motor,torque_rms_nm,torque_peak_nm,following_error_rms_rad,following_error_peak_rad");
    for (motor, d) in motors.iter().zip(differences(&measured, &simulated)) {
        println!("{},{:.6e},{:.6e},{:.6e},{:.6e}", motor.id, d[0], d[1], d[2], d[3]);
    }
    if args.option("--out").is_some() {
        let mut out = output(args, "--out")?;
        let ids: Vec<&str> = motors.iter().map(|m| m.id).collect();
        out.write_all(overlay_csv(&ids, &measured, &simulated).as_bytes()).map_err(io)?;
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
    if !sim_components::trace::linear_transformation(machine.as_ref()) {
        return Err(Failure("error-trace-not-linear", String::new()));
    }
    let mut out = output(args, "--out")?;
    let (mut advice, mut result) = (Vec::new(), Ok(()));
    writeln!(out, "quantity,of,config,fitted,standard_error").map_err(io)?;
    if args.option("--trace").is_some() {
        let measured = load_trace(args, machine.motors())?.trace;
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
        let measured = parse_sweep(&read(path)?)?;
        let motor = number(args, "--sweep-motor", 0.0)? as usize;
        match fit_sweep(entry, &cfg, motor, &measured, number(args, "--amplitude", default_sweep_amplitude(entry, &cfg)?)?)? {
            Some(fit) => {
                let (id, (config, fitted)) = (machine.motors()[motor].id, (fit.config, fit.fitted));
                writeln!(out, "resonance_hz,{id},{:.6e},{:.6e},{:.3e}", config.frequency, fitted.frequency, fitted.frequency_error).map_err(io)?;
                writeln!(out, "resonance_damping,{id},{:.6e},{:.6e},", config.damping, fitted.damping).map_err(io)?;
                let (key, detail) = fit.advice();
                advice.push(tr.tr_arg(key, &detail));
            }
            // A failure like any other, after what the trace gave has been written.
            None => result = Err(Failure("fit-no-resonance", String::new())),
        }
    }
    out.flush().map_err(io)?;
    advice.iter().for_each(|line| eprintln!("{line}"));
    result
}

/// The trace named by `--trace`: through the mapping file `--map` (a vendor export; see
/// docs/measurement-procedure.md), or without one in the simulator's own columns for `motors`. Where a
/// machine is replayed or fitted, the mapping's motors have to be the machine's, in its order.
fn load_trace(args: &Args, motors: &[sim_core::MotorInfo]) -> Result<Imported, Failure> {
    let path = args.option("--trace").ok_or(Failure("cli-usage", String::new()))?;
    let text = decode(&std::fs::read(path).map_err(|e| Failure("cli-error-io", format!("{path}: {e}")))?);
    Ok(match args.option("--map") {
        Some(map) => {
            let imported = import(&text, &Mapping::parse(&read(map)?)?)?;
            if !motors.is_empty() && imported.trace.motors.len() != motors.len() {
                return Err(Failure("error-import-map", format!("{} [[motor]] for {} motors", imported.trace.motors.len(), motors.len())));
            }
            imported
        }
        None => {
            let trace = Trace::parse(&text, motors)?;
            Imported { ids: motors.iter().map(|m| m.id.to_string()).collect(), torque_setpoints: vec![None; trace.motors.len()], trace }
        }
    })
}

/// One line of the `identify` table: quantity, motor, value, standard error and the 95 % interval.
fn estimate_row(out: &mut dyn Write, name: &str, of: &str, e: Estimate) -> Result<(), Failure> {
    let (low, high) = e.interval();
    writeln!(out, "{name},{of},{:.6e},{:.3e},{low:.6e},{high:.6e}", e.value, e.error).map_err(io)
}

/// A value without an error (a fit quality), in the same table.
fn value_row(out: &mut dyn Write, name: &str, of: &str, value: f64) -> Result<(), Failure> {
    writeln!(out, "{name},{of},{value:.6e},,,").map_err(io)
}

fn two_mass_rows(out: &mut dyn Write, of: &str, m: &TwoMass) -> Result<(), Failure> {
    let rows = [
        ("total_inertia_kg_m2", m.total_inertia),
        ("motor_side_inertia_kg_m2", m.motor_inertia),
        ("load_side_inertia_kg_m2", m.load_inertia),
        ("stiffness_nm_rad", m.stiffness),
        ("damping_nm_s_rad", m.damping),
        ("resonance_hz", m.resonance),
        ("anti_resonance_hz", m.anti_resonance),
    ];
    rows.into_iter().try_for_each(|(name, e)| estimate_row(out, name, of, e))
}

/// `identify [MACHINE] --trace FILE [--map FILE]`: one motor's drive train from its own trace, with no
/// machine config: inertia, friction (Coulomb, viscous, Stribeck, LuGre presliding stiffness), constant
/// torque, and the lag and dead time from torque setpoint to torque (where mapped) and from position
/// setpoint to position. `--motor N` picks the motor (default 0). `--from HZ --to HZ` also computes the
/// frequency response torque to speed from the trace (which needs an excitation in it) and fits the
/// two-mass model to it; `--sweep FILE` fits it to a measured response instead. `--step-from S [--step-to S]`
/// fits the ringing of the speed after a torque step, and with `--motor-inertia KGM2` (rotor and what sits on
/// it) and the fitted total inertia the two-mass model. Every value with its standard error and 95 %
/// interval, and how well each model reproduces the trace; `--overlay FILE` writes measured and modelled
/// signals side by side. Without `--map` the trace is in the simulator's columns of MACHINE.
fn identify(args: &Args, tr: &Translations) -> Result<(), Failure> {
    if args.option("--trace").is_none() && args.option("--sweep").is_none() {
        return Err(Failure("cli-usage", String::new()));
    }
    let mut out = output(args, "--out")?;
    writeln!(out, "quantity,of,value,standard_error,ci95_low,ci95_high").map_err(io)?;
    let mut result = Ok(());
    let mut overlay: Vec<(String, Vec<f64>)> = Vec::new();
    let mut time = Vec::new();
    if args.option("--trace").is_some() {
        let motors = match args.option("--map") {
            Some(_) => Vec::new(),
            None => {
                let (entry, cfg) = machine_and_config(args)?;
                (entry.build)(&cfg)?.motors().to_vec()
            }
        };
        let imported = load_trace(args, &motors)?;
        let k = number(args, "--motor", 0.0)? as usize;
        let (Some(m), Some(id)) = (imported.trace.motors.get(k), imported.ids.get(k)) else {
            return Err(Failure("cli-error-bad-argument", format!("--motor {k}")));
        };
        let t = &imported.trace.time;
        time.clone_from(t);
        let found = Identified::of(t, m, imported.torque_setpoints[k].as_deref());
        for (name, e) in found.estimates() {
            estimate_row(&mut *out, name, id, e)?;
        }
        for (name, value) in found.qualities() {
            value_row(&mut *out, name, id, value)?;
        }
        let has_setpoint = imported.torque_setpoints[k].is_some();
        if found.friction.is_none() {
            eprintln!("{}", tr.tr_arg("error-identify-friction", id));
        }
        if found.position_loop.is_none() || (has_setpoint && found.current_loop.is_none()) {
            eprintln!("{}", tr.tr_arg("error-identify-lag", id));
        }
        let total_inertia = found.friction.as_ref().map(|f| f.inertia);
        if let Some(f) = &found.friction {
            overlay.push(("torque_measured".into(), m.torque.clone()));
            overlay.push(("torque_model".into(), f.model.clone()));
        }
        for (name, lag, output) in [("current_loop", &found.current_loop, &m.torque), ("position_loop", &found.position_loop, &m.angle)] {
            if let Some(l) = lag {
                overlay.push((format!("{name}_measured"), output.clone()));
                overlay.push((format!("{name}_model"), l.model.clone()));
            }
        }
        if let (Some(from), Some(to)) = (args.number("--from")?, args.number("--to")?) {
            if !(from > 0.0 && to > from) {
                return Err(Failure("cli-error-bad-argument", format!("--from {from} --to {to}")));
            }
            match frequency_response(t, &m.torque, &m.speed, &log_spaced(from, to, SWEEP_POINTS)) {
                Some(response) => {
                    value_row(&mut *out, "response_lowest_coherence", id, response.iter().map(|(_, c)| *c).fold(1.0, f64::min))?;
                    let points: Vec<_> = response.into_iter().map(|(p, _)| p).collect();
                    match two_mass(&points) {
                        Some(model) => two_mass_rows(&mut *out, id, &model)?,
                        None => result = Err(Failure("error-identify-two-mass", String::new())),
                    }
                }
                None => result = Err(Failure("error-identify-response", String::new())),
            }
        }
        if let Some(from) = args.number("--step-from")? {
            match ringing(t, &m.speed, from, number(args, "--step-to", t[t.len() - 1])?) {
                Some(r) => {
                    estimate_row(&mut *out, "ringing_frequency_hz", id, r.frequency)?;
                    estimate_row(&mut *out, "ringing_damping_ratio", id, r.damping_ratio)?;
                    value_row(&mut *out, "ringing_fit_percent", id, r.quality.fit_percent)?;
                    if let (Some(motor), Some(total)) = (args.number("--motor-inertia")?, total_inertia) {
                        two_mass_rows(&mut *out, id, &two_mass_from_ringing(&r, total, Estimate { value: motor, error: 0.0 }))?;
                    }
                }
                None => result = Err(Failure("error-identify-ringing", String::new())),
            }
        }
    }
    if let Some(path) = args.option("--sweep") {
        match two_mass(&parse_sweep(&read(path)?)?) {
            Some(model) => two_mass_rows(&mut *out, "sweep", &model)?,
            None => result = Err(Failure("error-identify-two-mass", String::new())),
        }
    }
    out.flush().map_err(io)?;
    if let Some(path) = args.option("--overlay") {
        let mut file = std::io::BufWriter::new(std::fs::File::create(path).map_err(io)?);
        let names: Vec<&str> = overlay.iter().map(|(n, _)| n.as_str()).collect();
        writeln!(file, "time,{}", names.join(",")).map_err(io)?;
        for (row, t) in time.iter().enumerate() {
            let values: Vec<String> = overlay.iter().map(|(_, v)| format!("{:.9e}", v[row])).collect();
            writeln!(file, "{t:.9e},{}", values.join(",")).map_err(io)?;
        }
        file.flush().map_err(io)?;
    }
    result
}

/// `check-dataset DIR`: what keeps a dataset folder from being published (datasets/README.md).
fn check_dataset(args: &Args, tr: &Translations) -> Result<(), Failure> {
    let dir = args.positional(1).ok_or(Failure("cli-usage", String::new()))?;
    let problems = sim_components::dataset::check(std::path::Path::new(dir));
    if problems.is_empty() {
        println!("{}", tr.tr_arg("dataset-ok", dir));
        return Ok(());
    }
    problems.iter().for_each(|p| println!("{}", tr.tr_arg("dataset-problem", p)));
    Err(Failure("dataset-problem", format!("{} problems in {dir}", problems.len())))
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
/// and higher axis limits, with input shaping where a faster setting fails on settling alone, lower lifts down to
/// `profile.min_clearance_m` and lifts rounded through a via point (`--runs N` cycles at most, 40 by default). Prints
/// the cycle time and what to change in the config; per changed lift its segments, height before and after, and via point.
fn optimise(args: &Args) -> Result<(), Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let runs = args.number("--runs")?.unwrap_or(40.0) as usize;
    let found = sim_components::analysis::optimise_cycle(entry, &cfg, runs)?;
    let mut out = output(args, "--out")?;
    writeln!(out, "setting,before,after").map_err(io)?;
    writeln!(out, "cycle_s,{:.4},{:.4}", found.before.duration, found.after.duration).map_err(io)?;
    writeln!(out, "dwell_error_peak_m,{:.3e},{:.3e}", found.before.dwell_error_peak, found.after.dwell_error_peak).map_err(io)?;
    // The new cycle itself is TOML with commas; its lifts say what changed in it.
    for (path, before, after) in found.changes.iter().filter(|c| c.0 != "cycle.segment") {
        writeln!(out, "{path},{before},{after}").map_err(io)?;
    }
    for lift in &found.lifts {
        let [a, b, c] = lift.segments;
        writeln!(out, "cycle.segment[{a}-{b}-{c}].lift_m,{:.4},{:.4}", lift.before, lift.after).map_err(io)?;
        if let Some(via) = &lift.via {
            let point: Vec<String> = via.iter().map(|x| format!("{x:.4}")).collect();
            writeln!(out, "cycle.segment[{a}-{b}-{c}].via_m,,{}", point.join(" ")).map_err(io)?;
        }
    }
    out.flush().map_err(io)
}

/// `search <machine>`: every motor of the catalogue on the cycle of the config, each with the drive the catalogue
/// pairs with it, the passing ones first, cheapest first (installed weight stands in for the price). `--series
/// PREFIX` limits it to article numbers starting that way, `--supply` picks 230v or 400v (S210) or s120 (the S120
/// motors and Motor Modules). `--gearboxes SERIES` (e.g. NRK) also tries each motor with the fitting gearboxes of
/// that series, on a machine that has a gearbox. `--axes` tries every motor on every axis of the catalogue of the
/// machine's kind as well, the axis's weight in the cost. Says on the error stream how many candidates the
/// catalogue numbers ruled out before any cycle was simulated (they are in the table too) and how long the search
/// took on how many processors.
fn search(args: &Args, tr: &Translations) -> Result<(), Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let prefix = args.option("--series").unwrap_or("");
    let (supply, gearboxes, wanted) = (args.option("--supply").unwrap_or("400v"), args.option("--gearboxes"), |article: &str| article.starts_with(prefix));
    let started = std::time::Instant::now();
    let found = if args.flag("--axes") {
        sim_components::catalog::search_axes(entry, &cfg, supply, &wanted, gearboxes)
    } else {
        sim_components::catalog::search_motors(entry, &cfg, supply, &wanted, gearboxes)
    };
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get());
    eprintln!("{}", tr.tr_arg("cli-search-screened", &format!("{} / {}", found.iter().filter(|c| c.result.is_none()).count(), found.len())));
    eprintln!("{}", tr.tr_arg("cli-search-time", &format!("{threads}: {:.1} s", started.elapsed().as_secs_f64())));
    let mut out = output(args, "--out")?;
    writeln!(out, "axis,motor,gearbox,drive,mass_kg,cost_kg,result,tightest_check,tightest_utilisation,peak_torque_nm,winding_rise_k").map_err(io)?;
    for c in &found {
        let (check, utilisation) = c.tightest().unwrap_or((c.refused, f64::NAN));
        let (peak, rise) = c.result.as_ref().map_or((f64::NAN, f64::NAN), |(load, _)| {
            (load.motors.iter().map(|m| m.peak_torque).fold(0.0, f64::max), load.motors.iter().map(|m| m.winding_rise).fold(0.0, f64::max))
        });
        let result = if c.passes() { "ok" } else { "FAIL" };
        writeln!(out, "{},{},{},{},{:.2},{:.2},{result},{check},{utilisation:.4},{peak:.4},{rise:.2}", c.axis, c.article, c.gearbox, c.drive, c.mass, c.cost).map_err(io)?;
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
/// the runs in which each check passed. `--bands --worst` searches per check for the worst corner of the bounds
/// instead (`verdict_worst`) and prints its value, its result and which parameters are at their high and which at
/// their low bound there; the number of cycles it simulated goes to the error stream.
fn verdicts(args: &Args, tr: &Translations) -> Result<(), Failure> {
    use sim_components::analysis::{bounds, verdict_bands, verdict_bands_sampled, verdict_worst};
    let (entry, cfg) = machine_and_config(args)?;
    let motors = (entry.build)(&cfg)?.motors().to_vec();
    if args.flag("--worst") {
        if !args.flag("--bands") || args.option("--samples").is_some() || args.option("--seed").is_some() {
            return Err(Failure("cli-error-worst", String::new()));
        }
        let bounded = bounds(&cfg)?;
        let (worst, runs) = verdict_worst(entry, &cfg, &bounded)?;
        eprintln!("{}", tr.tr_arg("cli-worst-runs", &runs.to_string()));
        let mut out = output(args, "--out")?;
        writeln!(out, "check,motor,worst,limit,worst_case,at_high_bound,at_low_bound").map_err(io)?;
        for w in &worst {
            let v = &w.verdict;
            let at = |high: bool| bounded.iter().zip(&w.high).filter(|(_, h)| **h == Some(high)).map(|(b, _)| b.0.as_str()).collect::<Vec<_>>().join(";");
            let result = if v.ok { "ok" } else { "FAIL" };
            writeln!(out, "{},{},{},{},{result},{},{}", v.check, v.motor.map_or("", |k| motors[k].id), significant(v.value), significant(v.limit), at(true), at(false)).map_err(io)?;
        }
        return out.flush().map_err(io);
    }
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
/// over one pass of the cycle, or over a trajectory or path (see `poses`). `--points FILE` also writes the operating points,
/// `--breakdown FILE` the peak and RMS of each part of each motor's torque (inertia, gravity, friction, rest, and the
/// torque itself), `--budget FILE` where the energy drawn from the supply went, in joules and as a share of it.
fn analyze(args: &Args, tr: &Translations) -> Result<(), Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let mut sim = Sim::new(entry, &cfg)?;
    // The effect views cost a machine evaluation per step: only when they are written.
    let views = args.option("--breakdown").is_some() || args.option("--budget").is_some();
    let load = match poses(args, &sim, tr)? {
        Some(points) => analyse_trajectory_with(&mut sim, points, views),
        None => analyse_cycle_with(&mut sim, views),
    }
    .map_err(|e| Failure(e.key, String::new()))?;
    let mut out = output(args, "--out")?;
    writeln!(out, "motor,cycle_s,peak_torque_nm,rms_torque_nm,peak_speed_rad_s,utilisation,thermal_utilisation,belt_tension_min_n,belt_tension_max_n,rms_current_a,winding_rise_k,energy_in_j,energy_regenerated_j,peak_regenerated_power_w,largest_regeneration_j,path_error_peak_m,path_error_rms_m,tracking_error_peak_m,dwell_error_peak_m,cubic_mean_torque_nm,braking_resistor_energy_j,braking_resistor_mean_power_w,dc_link_peak_voltage_v,belt_slack_time_s,radial_load_peak_n,guide_load_peak_n,guide_life_km,mains_energy_j,guide_life_h,pulley_bearing_life_h,gearbox_life_h,belt_tooth_utilisation").map_err(io)?;
    // Empty where the machine does not report the signal or the config has no load rating for the guide.
    let or_empty = |value: Option<f64>| value.map_or(String::new(), |v| format!("{v:.4e}"));
    let machine_wide = format!("{},{},{},{}", or_empty(load.belt_slack_time), or_empty(load.radial_load_peak), or_empty(load.guide_load.map(|g| g.0)), or_empty(sim_components::analysis::guide_life_km(&cfg, &load)));
    for (motor, m) in sim.machine.motors().iter().zip(&load.motors) {
        let belt = load.belt_tension.map_or(String::from(","), |(low, high)| format!("{low:.2},{high:.2}"));
        let electrical = format!("{:.4},{:.2},{:.3},{:.3},{:.2},{:.3}", m.rms_current, m.winding_rise, m.energy_in, m.energy_regenerated, m.peak_regenerated_power, m.largest_regeneration);
        let electrical = format!("{electrical},{:.3e},{:.3e},{:.3e},{:.3e}", load.path_error_peak, load.path_error_rms, load.tracking_error_peak, load.dwell_error_peak);
        let electrical = format!("{electrical},{:.6},{:.3},{:.3},{:.2},{machine_wide},{:.3}", m.cubic_mean_torque, m.resistor_energy, m.resistor_energy / load.duration, m.peak_dc_link_voltage, m.mains_energy);
        // Lives in hours and the belt's tooth load; empty where the machine or the config lacks what they need.
        let tooth = sim_components::analysis::belt_tooth_utilisation(&cfg, m)?;
        let lives = [sim_components::analysis::guide_life_h(&cfg, &load), sim_components::analysis::pulley_bearing_life_h(&cfg, m), sim_components::analysis::gearbox_life_h(&cfg, m), tooth];
        let electrical = format!("{electrical},{}", lives.map(or_empty).join(","));
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
    if args.option("--breakdown").is_some() {
        let mut out = output(args, "--breakdown")?;
        writeln!(out, "motor,part,peak_torque_nm,rms_torque_nm").map_err(io)?;
        for (motor, m) in sim.machine.motors().iter().zip(&load.motors) {
            for (part, (peak, rms)) in sim_core::TORQUE_PARTS.iter().zip(m.torque_parts.iter().flatten()) {
                writeln!(out, "{},{part},{peak:.6e},{rms:.6e}", motor.id).map_err(io)?;
            }
            writeln!(out, "{},torque,{:.6e},{:.6e}", motor.id, m.peak_torque, m.rms_torque).map_err(io)?;
        }
        out.flush().map_err(io)?;
    }
    if args.option("--budget").is_some() {
        let mut out = output(args, "--budget")?;
        writeln!(out, "item,energy_j,share").map_err(io)?;
        if let Some(budget) = &load.budget {
            let rows = budget.rows.iter().map(|r| (r.id.as_str(), r.energy));
            for (item, energy) in std::iter::once(("supply", budget.supply)).chain(rows).chain([("unaccounted", budget.unaccounted())]) {
                writeln!(out, "{item},{energy:.6e},{:.6}", energy / budget.supply).map_err(io)?;
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
    let frequencies = log_spaced(number("--from", 5.0)?, number("--to", 500.0)?, number("--count", SWEEP_POINTS as f64)? as usize);
    let points = frequency_sweep(entry, &cfg, number("--motor", 0.0)? as usize, &frequencies, number("--amplitude", default_sweep_amplitude(entry, &cfg)?)?)?;
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
    let response = step_response(entry, &cfg, number(args, "--axis-index", 0.0)? as usize, number(args, "--size", 0.001)?, number(args, "--time", 0.5)?)?;
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
    let points = closed_loop_response(entry, &cfg, number(args, "--axis-index", 0.0)? as usize, &frequencies, number(args, "--amplitude", 0.0005)?)?;
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

/// `margins <machine> [--bands]`: per motor the stability margins of its velocity loop (`velocity_loop_margins`), for
/// the config as it is and with `--bands` with each value of `[bounds]` at its low and at its high bound. A row whose
/// loop is unstable or whose measurement is not valid (`LoopMargins::stable`) says FAIL.
fn margins(args: &Args) -> Result<(), Failure> {
    use sim_components::analysis::{bounds, velocity_loop_margins};
    let (entry, cfg) = machine_and_config(args)?;
    let motors = (entry.build)(&cfg)?.motors().to_vec();
    let runs = velocity_loop_margins(entry, &cfg, &if args.flag("--bands") { bounds(&cfg)? } else { Vec::new() })?;
    let mut out = output(args, "--out")?;
    writeln!(out, "run,motor,crossover_hz,phase_margin_deg,gain_margin_db,peak_sensitivity,peak_frequency_hz,result").map_err(io)?;
    for (run, margins) in &runs {
        for (motor, m) in motors.iter().zip(margins) {
            let result = if m.stable() { "ok" } else { "FAIL" };
            writeln!(out, "{run},{},{:.1},{:.1},{:.2},{:.3},{:.1},{result}", motor.id, m.crossover_hz, m.phase_margin_deg, m.gain_margin_db, m.peak_sensitivity, m.peak_frequency_hz).map_err(io)?;
        }
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
            for (i, v) in sample.iter().enumerate() {
                write!(out, "{}{v:.9e}", if i == 0 { "" } else { "," }).map_err(io)?;
            }
            writeln!(out).map_err(io)?;
        }
        if Some(step) == disable_at {
            sim.command(Command::Disable).map_err(|e| Failure(e.key, String::new()))?;
        }
        sim.step();
    }
    out.flush().map_err(io)
}

/// `fmu <machine> --out FILE.fmu [--fmi 2|3]`: the machine with its config as an FMI 2.0 (the default) or 3.0
/// Co-Simulation FMU (see `sim_fmi`), with the descriptions in the language of `--lang`. Its binaries are the files
/// `--binary` names (it may repeat: for FMI 2.0 a .dll goes to win64, a .so to linux64, a .dylib to darwin64; for
/// FMI 3.0 to `<architecture>-<system>`, the architecture read from the binary), or else the sim-fmi library built
/// next to this program.
fn fmu(args: &Args) -> Result<(), Failure> {
    // shortcut: the FMU finds its machine by id among the registered ones, so a machine file cannot go into one yet.
    if args.option("--machine-file").is_some() {
        return Err(Failure("cli-error-machine-file-fmu", String::new()));
    }
    let (entry, cfg) = machine_and_config(args)?;
    let path = args.option("--out").ok_or(Failure("cli-error-missing-value", "--out".into()))?;
    let fmi = match args.option("--fmi") {
        None | Some("2") => 2,
        Some("3") => 3,
        Some(other) => return Err(Failure("cli-error-bad-argument", format!("--fmi {other}"))),
    };
    let model = sim_fmi::Model::with_fmi(entry, &cfg.to_text()?, args.option("--lang").unwrap_or("en"), fmi)?;
    let mut binaries: Vec<std::path::PathBuf> = args.options("--binary").into_iter().map(Into::into).collect();
    if binaries.is_empty() {
        let name = format!("{}{}{}", std::env::consts::DLL_PREFIX, sim_fmi::MODEL_IDENTIFIER, std::env::consts::DLL_SUFFIX);
        let beside = std::env::current_exe().map_err(io)?.with_file_name(name);
        binaries.push(beside);
    }
    let mut files = model.files();
    let mut platforms: Vec<(String, &std::path::PathBuf)> = Vec::new();
    for binary in &binaries {
        let file_name = binary.file_name().and_then(|n| n.to_str()).unwrap_or("");
        let bad = || Failure("cli-error-bad-argument", format!("--binary {}", binary.display()));
        let inside = sim_fmi::binary_path(file_name).ok_or_else(bad)?;
        let bytes = std::fs::read(binary).map_err(|e| Failure("cli-error-fmu-binary", format!("{}: {e}", binary.display())))?;
        let inside = if fmi == 3 { sim_fmi::binary_path3(file_name, &bytes).ok_or_else(bad)? } else { inside };
        // Two binaries for one platform would be two entries of one name in the archive.
        if let Some((_, other)) = platforms.iter().find(|(path, _)| *path == inside) {
            return Err(Failure("cli-error-conflict", format!("--binary {} --binary {}", other.display(), binary.display())));
        }
        platforms.push((inside.clone(), binary));
        files.push((inside, bytes));
    }
    std::fs::write(path, sim_fmi::zip(&files)).map_err(|e| Failure("cli-error-io", format!("{path}: {e}")))
}

/// `compare <machine> --effect NAME`: the cycle run with `effects.NAME` on and with it off (`compare_effect`). Writes
/// every signal, or those `--signal ID` names (it may repeat), once per millisecond of the measured pass, on and off
/// side by side; `--differences FILE` the peak and the RMS of on less off per signal.
fn compare(args: &Args) -> Result<(), Failure> {
    let (entry, cfg) = machine_and_config(args)?;
    let effect = args.option("--effect").ok_or(Failure("cli-error-missing-value", "--effect".into()))?;
    let found = sim_components::analysis::compare_effect(entry, &cfg, effect)?;
    let named = args.options("--signal");
    let place = |id: &str| found.signals.iter().position(|s| s.id == id).ok_or(Failure("cli-error-bad-argument", format!("--signal {id}")));
    // Every signal but the time, which is the first column anyway.
    let chosen: Vec<usize> = if named.is_empty() { (1..found.signals.len()).collect() } else { named.into_iter().map(place).collect::<Result<_, _>>()? };
    let mut out = output(args, "--out")?;
    let header: Vec<String> = chosen.iter().map(|&i| &found.signals[i]).map(|s| format!("{0}.on[{1}],{0}.off[{1}]", s.id, s.unit)).collect();
    writeln!(out, "time[s],{}", header.join(",")).map_err(io)?;
    for (on, off) in found.on.iter().zip(&found.off) {
        let cells: Vec<String> = chosen.iter().map(|&i| format!("{:.9e},{:.9e}", on[i], off[i])).collect();
        writeln!(out, "{:.4},{}", on[0], cells.join(",")).map_err(io)?;
    }
    out.flush().map_err(io)?;
    if args.option("--differences").is_some() {
        let mut out = output(args, "--differences")?;
        writeln!(out, "signal,unit,peak_difference,rms_difference").map_err(io)?;
        for &i in &chosen {
            let (peak, rms) = found.difference(i);
            writeln!(out, "{},{},{peak:.6e},{rms:.6e}", found.signals[i].id, found.signals[i].unit).map_err(io)?;
        }
        out.flush().map_err(io)?;
    }
    Ok(())
}
