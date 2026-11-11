//! Batch front end: runs the core as fast as the CPU allows and writes CSV.
//! Every message comes from the translation files; see `cli-usage` there for the commands.

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

fn texts(lang: &str, entry: &MachineEntry) -> Translations {
    let mut tr = translations(lang, entry);
    I18N.iter().filter(|(l, _)| *l == lang).for_each(|(_, text)| tr.add(text));
    tr
}

/// Command-line arguments: positional words, and `--name value` options that may repeat.
struct Args(Vec<String>);

impl Args {
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
        self.option(name).map(|v| v.parse().map_err(|_| Failure("cli-error-bad-argument", format!("{name} {v}")))).transpose()
    }
    /// The n-th word that is neither an option nor an option's value.
    fn positional(&self, n: usize) -> Option<&str> {
        let mut words = Vec::new();
        let mut i = 0;
        while i < self.0.len() {
            if self.0[i].starts_with("--") {
                i += if self.0[i] == "--cycle" { 1 } else { 2 };
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
    match args.positional(0) {
        Some("list") => {
            let lang = args.option("--lang").unwrap_or("en");
            for entry in MACHINES {
                println!("{}\t{}", entry.id, translations(lang, entry).tr(entry.key));
            }
            Ok(())
        }
        Some("run") => run(args, tr),
        _ => Err(Failure("cli-usage", String::new())),
    }
}

/// The machine named on the command line with its config: the default, or `--config FILE`,
/// then every `--set path=value` applied on top.
fn machine_and_config(args: &Args) -> Result<(&'static MachineEntry, Config), Failure> {
    let id = args.positional(1).ok_or(Failure("cli-usage", String::new()))?;
    let entry = find(id).ok_or(Failure("cli-error-unknown-machine", id.into()))?;
    let mut cfg = match args.option("--config") {
        Some(path) => Config::parse(&std::fs::read_to_string(path).map_err(|e| Failure("cli-error-io", format!("{path}: {e}")))?)?,
        None => entry.config(),
    };
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
        None
    };
    if let Some(command) = command {
        sim.command(command).map_err(|e| Failure(e.key, String::new()))?;
    }
    Ok(sim)
}

/// `run <machine>`: simulate a move or the cycle and write every signal as CSV.
fn run(args: &Args, tr: &Translations) -> Result<(), Failure> {
    let mut sim = start(args, tr)?;
    let seconds = args.number("--time")?.unwrap_or(2.0);
    let every = args.number("--every")?.unwrap_or(16.0).max(1.0) as u64;
    let io = |e: std::io::Error| Failure("cli-error-io", e.to_string());
    let mut out: Box<dyn Write> = match args.option("--out") {
        Some(path) => Box::new(std::io::BufWriter::new(std::fs::File::create(path).map_err(io)?)),
        None => Box::new(std::io::stdout().lock()),
    };
    let header: Vec<String> = sim.signal_infos().iter().map(|s| format!("{}[{}]", s.id, s.unit)).collect();
    writeln!(out, "{}", header.join(",")).map_err(io)?;
    let mut sample = Vec::new();
    for step in 0..=(seconds / sim.dt).round() as u64 {
        if step % every == 0 {
            sim.sample(&mut sample);
            let row: Vec<String> = sample.iter().map(|v| format!("{v:.9e}")).collect();
            writeln!(out, "{}", row.join(",")).map_err(io)?;
        }
        sim.step();
    }
    out.flush().map_err(io)
}
