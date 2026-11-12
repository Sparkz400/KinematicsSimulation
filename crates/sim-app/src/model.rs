//! Everything the interface shows and edits, without any drawing: the chosen machine and language,
//! the config being edited, the scope history, tuning and analysis results. The window code only draws
//! this and calls its methods.

use crate::session::{Event, Request, Session, extend_trail};
use sim_components::analysis::{Band, CycleLoad, MotorLoad, Optimised, Sensitivity, Verdict};
use sim_components::analysis::{StepResponse, SweepPoint, bandwidth, closed_loop_response, log_spaced, step_response};
use sim_components::analysis::{analyse_cycle, analyse_trajectory, bounds, guide_life_km, optimise_cycle, sensitivity, verdict_bands, verdict_bands_sampled, verdicts};
use sim_components::catalog::{Candidate, search_motors};
use sim_components::identify::{AxisFit, fit_rigid};
use sim_components::trace::{Trace, TraceError, difference, replay};
use sim_components::{Command, CommandError, Sim, path};
use sim_core::{Config, ConfigError, MachineEntry, Scene, SignalInfo, Translations};
use sim_machines::{MACHINES, translations};
use std::collections::VecDeque;
use std::sync::Mutex;
use std::sync::mpsc::{Receiver, TryRecvError, channel};
use toml::Value;

/// Samples kept for the scopes: one minute at one per millisecond.
pub const HISTORY: usize = 60_000;

/// A message for the user, as a translation key and its argument.
pub type Message = (&'static str, String);

/// One group of the parameter panel: a config table and its scalar values.
pub struct ParamGroup {
    pub key: String,
    /// (config path, label key, value)
    pub params: Vec<(String, String, Value)>,
}

pub struct ScopeLine {
    /// Index into the signal list.
    pub signal: usize,
    /// A setpoint or command, drawn dashed in the colour of the actual value it belongs to.
    pub reference: bool,
}

/// One scope: one quantity for every axis or motor, actual values together with their setpoints.
pub struct Scope {
    pub quantity_key: String,
    pub unit: String,
    pub lines: Vec<ScopeLine>,
    pub visible: bool,
}

/// What a trace did inside the visible time window.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stats {
    pub min: f64,
    pub max: f64,
    pub rms: f64,
}

/// The answer of one axis's position loop to a step and to sines, for choosing gains.
pub struct Tuning {
    pub axis: usize,
    pub step: StepResponse,
    pub response: Vec<SweepPoint>,
    pub bandwidth: Option<f64>,
}

/// What the analysis panel can compute. Each takes seconds, so each runs in a thread of its own.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Analysis {
    /// The cycle's loads, judged against what the parts allow.
    Cycle,
    /// The cycle again with each bounded parameter at its low and its high bound.
    Sensitivity,
    /// A shorter cycle that still passes every check.
    Optimise,
    /// The catalogue motors the inputs ask for, each tried on the cycle.
    Search,
    /// A path from a file: planned, run and judged.
    Path,
    /// A drive trace from a file: replayed, and the config fitted to it.
    Trace,
}

/// What the analysis panel is asked with, as typed.
#[derive(Debug, Clone, PartialEq)]
pub struct Inputs {
    /// The cycle analysis also runs the cycle with every bounded value at its bounds, for the error bands.
    pub bands: bool,
    /// For the error bands: how often the cycle is run with all bounded values between their bounds at once.
    /// 0: each value alone at its low and at its high bound.
    pub samples: usize,
    /// Motor search: "230v" or "400v", how the article numbers start, and a gearbox series or nothing.
    pub supply: String,
    pub series: String,
    pub gearboxes: String,
    /// Path: the file, whether it holds poses at times and not lines and arcs, and whether lines and arcs
    /// are timed as fast as the motors allow and not within the axis limits.
    pub path_file: String,
    pub path_timed: bool,
    pub time_optimal: bool,
    pub trace_file: String,
}

/// The supplies the motor catalogue has data for.
pub const SUPPLIES: [&str; 2] = ["230v", "400v"];

/// A path as it was planned, run on a simulation of its own and judged.
pub struct PathRun {
    pub points: Vec<(f64, Vec<f64>)>,
    /// The planned poses as places in the 3D scene (`Machine::scene_point`), for drawing the path there.
    /// Empty for a machine without that mapping.
    pub scene_line: Vec<[f64; 3]>,
    pub load: CycleLoad,
    pub verdicts: Vec<Verdict>,
    /// How far the planned path is off the given one, at most: transition curves and arcs off their stated end.
    pub deviation: f64,
}

/// A trace replayed and fitted: per motor the RMS and peak difference between the trace and its replay, of
/// torque and of following error, and per axis the fit (none for an axis the trace does not move both ways).
pub struct TraceFit {
    pub differences: Vec<[f64; 4]>,
    pub fits: Vec<Option<AxisFit>>,
}

enum Report {
    /// With the error bands, where they were asked for, and the number of samples they are of (0: each
    /// bounded value alone at its bounds).
    Cycle(CycleLoad, Vec<Verdict>, Option<Vec<Band>>, usize),
    Sensitivity(CycleLoad, Vec<Sensitivity>),
    Optimised(Optimised),
    /// With the supply that was searched.
    Search(Vec<Candidate>, String),
    Path(PathRun),
    Trace(TraceFit),
}

type Job<T> = Mutex<Receiver<Result<T, Message>>>;

/// What a background job has answered, if it has. A job whose thread is gone without an answer (it
/// panicked) answers with a message, so that it does not leave a spinner turning for ever.
fn answer<T>(job: &Job<T>) -> Option<Result<T, Message>> {
    match job.lock().map(|job| job.try_recv()) {
        Ok(Err(TryRecvError::Empty)) => None,
        Ok(Ok(answer)) => Some(answer),
        _ => Some(Err(("ui-job-ended", String::new()))),
    }
}

/// A table for the window to draw as it is, the header first. Every cell is text in the chosen language.
pub type Table = Vec<Vec<String>>;

/// The table headers of the analysis panel, as translation keys.
const CYCLE_KEYS: [&str; 11] = [
    "ui-analysis-cycle-time",
    "ui-analysis-duration",
    "ui-analysis-path-deviation",
    "ui-analysis-path-error",
    "ui-analysis-tracking-error",
    "ui-analysis-dwell-error",
    "ui-analysis-belt-min",
    "ui-analysis-slack-time",
    "ui-analysis-radial-load",
    "ui-analysis-guide-load",
    "ui-analysis-guide-life",
];
/// One row per figure, one column per motor.
const MOTOR_KEYS: [&str; 9] = [
    "ui-analysis-peak-torque",
    "ui-analysis-rms-torque",
    "ui-analysis-cubic-torque",
    "ui-analysis-winding-rise",
    "ui-analysis-energy-in",
    "ui-analysis-energy-back",
    "ui-analysis-resistor-energy",
    "ui-analysis-resistor-power",
    "ui-analysis-dc-link-peak",
];
const VERDICT_KEYS: [&str; 5] = ["ui-analysis-check", "ui-analysis-motor", "ui-analysis-value", "ui-analysis-limit", "ui-analysis-result"];
const BAND_KEYS: [&str; 6] = ["ui-analysis-check", "ui-analysis-motor", "ui-analysis-band-low", "ui-analysis-band-high", "ui-analysis-band-worst", "ui-analysis-band-share"];
const SEARCH_KEYS: [&str; 6] = ["ui-analysis-motor", "ui-search-gearbox", "ui-search-mass", "ui-analysis-result", "ui-search-tightest", "ui-search-share"];
const TRACE_KEYS: [&str; 4] = ["ui-trace-torque-rms", "ui-trace-torque-peak", "ui-trace-error-rms", "ui-trace-error-peak"];
const FIT_KEYS: [&str; 4] = ["ui-trace-quantity", "ui-trace-config", "ui-trace-fitted", "ui-trace-error"];
/// The four values of `AxisFit`, in its order.
const FIT_QUANTITIES: [&str; 4] = ["ui-trace-mass", "ui-trace-viscous", "ui-trace-coulomb", "ui-trace-constant"];
/// Messages of the analysis panel, and the reasons a search gives for a motor it did not run.
const NOTES: [&str; 14] = [
    "ui-simulation-ended",
    "ui-job-ended",
    "ui-analysis-stale",
    "ui-analysis-tripped",
    "ui-drive-tripped",
    "ui-search-bad-supply",
    "ui-search-none",
    "ui-search-no-gearbox",
    "ui-search-refused",
    "ui-search-limit-motor-speed",
    "ui-search-limit-motor-torque",
    "ui-file-unreadable",
    "ui-file-line",
    "ui-analysis-figure",
];
/// The names of `[parts]` that a motor of the search goes into.
const PARTS: [&str; 3] = ["motor", "motor_supply", "gearbox"];
const SENSITIVITY_KEYS: [&str; 6] =
    ["ui-analysis-parameter", "ui-analysis-low", "ui-analysis-high", "ui-analysis-peak-change", "ui-analysis-rms-change", "ui-analysis-time-change"];
const OPTIMISED_KEYS: [&str; 3] = ["ui-analysis-setting", "ui-analysis-before", "ui-analysis-after"];
/// Every check a cycle can be judged by (`verdicts`); each has a label "check-<name>".
const CHECKS: [&str; 15] = [
    "motor-peak",
    "motor-thermal",
    "pulley-torque",
    "gearbox-continuous",
    "gearbox-peak",
    "dc-link-energy",
    "braking-resistor-continuous",
    "braking-resistor-peak",
    "settling",
    "belt-tension",
    "belt-slack",
    "gearbox-input-speed",
    "gearbox-radial-load",
    "guide-load",
    "dc-link-overvoltage",
];
/// Rows of the sensitivity ranking that are shown: the parameters that matter most.
const TOP_ROWS: usize = 8;

/// A number at a readable length, whatever its size: four digits and at most three decimals, so 12.5 reads
/// as 12.50 and 1912.141 as 1912, and 0.0000312 as 3.12e-5. Narrow, so that the tables fit the panel.
fn short(value: f64) -> String {
    match value.abs() {
        size if size == 0.0 => "0.000".into(),
        size if (0.01..1e5).contains(&size) => format!("{value:.*}", (3 - size.log10().floor() as i32).clamp(0, 3) as usize),
        _ => format!("{value:.2e}"),
    }
}

pub struct Model {
    pub entry: &'static MachineEntry,
    pub lang: &'static str,
    pub tr: Translations,
    /// The config as edited. The simulation may yet refuse the last edits.
    pub config: Config,
    /// The last config the simulation took, to go back to when it refuses one, and the configs sent to it
    /// that it has not answered yet, oldest first.
    accepted: Config,
    sent: VecDeque<Config>,
    pub session: Session,
    /// The end of the simulation thread has been reported.
    ended: bool,
    pub scopes: Vec<Scope>,
    pub history: VecDeque<Vec<f64>>,
    /// The history as it was when the scopes were frozen; the simulation runs on meanwhile.
    pub frozen: Option<VecDeque<Vec<f64>>>,
    /// Seconds of history the scopes show.
    pub window_s: f64,
    pub scene: Scene,
    pub paused: bool,
    pub speed: f64,
    /// Target of the next point-to-point move, per axis.
    pub target: Vec<f64>,
    pub messages: Vec<Message>,
    pub tuning_axis: usize,
    pub tuning: Option<Tuning>,
    tuning_job: Option<Job<Tuning>>,
    pub inputs: Inputs,
    /// Results of the analysis panel, each for the config as it was when it was asked for.
    pub cycle_report: Option<(CycleLoad, Vec<Verdict>)>,
    /// The checks of the cycle with their error bands, where the cycle was analysed with them.
    pub bands: Option<Vec<Band>>,
    /// How many samples those bands are of; 0: each bounded value alone at its low and at its high bound.
    pub band_samples: usize,
    /// The cycle as configured and the varied parameters, strongest first.
    pub sensitivity: Option<(CycleLoad, Vec<Sensitivity>)>,
    pub optimised: Option<Optimised>,
    /// The motors of the search as ranked, and the supply they were searched for.
    pub candidates: Option<(Vec<Candidate>, String)>,
    pub path_run: Option<PathRun>,
    /// Where the tool has been in the 3D scene since the machine took the planned path, a point per 0.1 mm of
    /// travel and the newest `session::TRAIL` of them: the path it really took, to draw beside the planned one. `None`
    /// before the machine has taken a path (it may refuse it), and after a reconfigure or the start of a cycle.
    pub trail: Option<Vec<[f64; 3]>>,
    pub trace_fit: Option<TraceFit>,
    /// Per kind of analysis (`Analysis as usize`) the config its result was computed for. A result says
    /// nothing about another config: it is marked (`outdated`), and nothing of it is applied to one.
    computed_for: [Option<Config>; 6],
    /// The jobs under way, each with the config it was started with.
    analysis_jobs: Vec<(Analysis, Config, Job<Report>)>,
}

/// A config without the names of its parts: what a motor search does not depend on, because it puts its
/// own motor (and gearbox) in.
fn without_parts(config: &Config) -> Config {
    let mut out = config.clone();
    for key in PARTS {
        let _ = out.set_text(&format!("parts.{key} = \"\""));
    }
    out
}

fn texts(lang: &str, entry: &MachineEntry) -> Translations {
    let mut tr = translations(lang, entry);
    crate::I18N.iter().filter(|(l, _)| *l == lang).for_each(|(_, text)| tr.add(text));
    tr
}

/// The quantity a setpoint or command belongs to: "position-setpoint" is drawn with "position".
fn actual_quantity(quantity_key: &str) -> Option<&str> {
    quantity_key.strip_suffix("-setpoint").or_else(|| quantity_key.strip_suffix("-command"))
}

/// The scopes follow from the signal list alone: one per quantity, in order of first appearance,
/// with setpoints and commands joined to the quantity they ask for.
fn scopes_for(signals: &[SignalInfo]) -> Vec<Scope> {
    const SHOWN_AT_START: [&str; 4] = ["quantity-position", "quantity-following-error", "quantity-speed", "quantity-torque"];
    let mut scopes: Vec<Scope> = Vec::new();
    for (i, signal) in signals.iter().enumerate().filter(|(_, s)| s.id != "time") {
        let actual = actual_quantity(&signal.quantity_key).filter(|a| signals.iter().any(|s| s.quantity_key == *a && s.unit == signal.unit));
        let key = actual.unwrap_or(&signal.quantity_key);
        let line = ScopeLine { signal: i, reference: actual.is_some() };
        match scopes.iter_mut().find(|s| s.quantity_key == key && s.unit == signal.unit) {
            Some(scope) => scope.lines.push(line),
            None => scopes.push(Scope { quantity_key: key.into(), unit: signal.unit.clone(), lines: vec![line], visible: SHOWN_AT_START.contains(&key) }),
        }
    }
    scopes
}

/// A factor and an SI prefix that bring a value of this size into a readable range:
/// 0.00004 m reads as 40 µm.
pub fn display_scale(peak: f64, unit: &str) -> (f64, &'static str) {
    match peak {
        _ if unit.is_empty() || peak == 0.0 || !peak.is_finite() => (1.0, ""),
        p if p < 1e-3 => (1e6, "µ"),
        p if p < 0.1 => (1e3, "m"),
        p if p >= 1e4 => (1e-3, "k"),
        _ => (1.0, ""),
    }
}

impl Model {
    pub fn new(entry: &'static MachineEntry, lang: &'static str) -> Self {
        let config = entry.config();
        let session = Session::start(entry, &config).expect("default config of a registered machine must run");
        let mut model = Self {
            entry,
            lang,
            tr: texts(lang, entry),
            scopes: scopes_for(&session.signals),
            accepted: config.clone(),
            sent: VecDeque::new(),
            config,
            session,
            ended: false,
            history: VecDeque::new(),
            frozen: None,
            window_s: 5.0,
            scene: Scene::default(),
            paused: false,
            speed: 1.0,
            target: Vec::new(),
            messages: Vec::new(),
            tuning_axis: 0,
            tuning: None,
            tuning_job: None,
            inputs: Inputs { bands: false, samples: 0, supply: SUPPLIES[1].into(), series: String::new(), gearboxes: String::new(), path_file: String::new(), path_timed: false, time_optimal: false, trace_file: String::new() },
            cycle_report: None,
            bands: None,
            band_samples: 0,
            sensitivity: None,
            optimised: None,
            candidates: None,
            path_run: None,
            trail: None,
            trace_fit: None,
            computed_for: Default::default(),
            analysis_jobs: Vec::new(),
        };
        model.target = model.config.f64s("start.pose").unwrap_or_default();
        model
    }

    /// Switching machine restarts the simulation with that machine's default config.
    pub fn set_machine(&mut self, entry: &'static MachineEntry) {
        let (lang, paused, speed, window, inputs) = (self.lang, self.paused, self.speed, self.window_s, self.inputs.clone());
        *self = Self::new(entry, lang);
        (self.window_s, self.inputs) = (window, inputs);
        self.set_paused(paused);
        self.set_speed(speed);
    }

    pub fn set_language(&mut self, lang: &'static str) {
        self.lang = lang;
        self.tr = texts(lang, self.entry);
    }

    pub fn set_paused(&mut self, paused: bool) {
        self.paused = paused;
        self.session.send(Request::Pause(paused));
    }

    pub fn set_speed(&mut self, speed: f64) {
        self.speed = speed;
        self.session.send(Request::Speed(speed));
    }

    /// Adds a message, unless it only repeats the last one: a refusal that comes back with every edit
    /// would otherwise bury everything else.
    fn say(&mut self, message: Message) {
        if self.messages.last() != Some(&message) {
            self.messages.push(message);
        }
    }

    /// The axis limits that ask too much of the motors, in place of those reported before.
    fn say_limits(&mut self, violations: Vec<sim_components::LimitViolation>) {
        self.messages.retain(|(key, _)| !key.starts_with("limit-"));
        for v in violations {
            let motor = self.tr.tr(self.session.motors[v.motor].key).to_string();
            self.say((v.key, format!("{motor}: {:.4} > {:.4}", v.needed, v.allowed)));
        }
    }

    /// Sends the config as edited to the running simulation. Its answer comes with `poll`.
    fn reconfigure(&mut self) {
        self.sent.push_back(self.config.clone());
        self.session.send(Request::Reconfigure(self.config.clone()));
    }

    /// Edits one config value and applies the config to the running simulation.
    pub fn set_param(&mut self, path: &str, value: Value) {
        match self.config.set(path, value) {
            Ok(()) => self.reconfigure(),
            Err(e) => self.say((e.key(), e.detail().into())),
        }
    }

    pub fn command(&mut self, command: Command) {
        self.session.send(Request::Command(command));
    }

    /// Takes in what the simulation and a running tuning job have produced since the last frame.
    pub fn poll(&mut self) {
        for event in self.session.drain() {
            match event {
                Event::Sample(sample) => {
                    if self.history.len() == HISTORY {
                        self.history.pop_front();
                    }
                    self.history.push_back(sample);
                }
                Event::Scene(scene) => self.scene = scene,
                Event::TrailStart(at) => self.trail = at.map(|at| vec![at]),
                // Not after `clear`: the result the trail belonged to is gone.
                Event::Trail(points) => {
                    if let Some(trail) = &mut self.trail {
                        extend_trail(trail, &points);
                    }
                }
                Event::Refused(key, detail) => self.say((key, detail)),
                Event::Limits(violations) => self.say_limits(violations),
                Event::Configured(answer) => {
                    let config = self.sent.pop_front();
                    match answer {
                        Ok(violations) => {
                            self.accepted = config.unwrap_or_else(|| self.accepted.clone());
                            self.say_limits(violations);
                        }
                        // The refused value goes, or it would stay in the config and have every later edit
                        // refused with it.
                        Err(message) => {
                            self.config = self.accepted.clone();
                            self.say(message);
                        }
                    }
                    // With every answer in, the config shown is the one the simulation runs.
                    if self.sent.is_empty() {
                        self.config = self.accepted.clone();
                    }
                }
            }
        }
        if self.session.ended() && !self.ended {
            self.ended = true;
            self.say(("ui-simulation-ended", String::new()));
        }
        if let Some(result) = self.tuning_job.as_ref().and_then(answer) {
            self.tuning_job = None;
            match result {
                Ok(tuning) => self.tuning = Some(tuning),
                Err(message) => self.say(message),
            }
        }
        // A job stays until it has answered.
        let mut reports = Vec::new();
        self.analysis_jobs.retain(|(kind, config, job)| answer(job).map(|report| reports.push((*kind, config.clone(), report))).is_none());
        for (kind, config, report) in reports {
            let report = match report {
                Ok(report) => report,
                Err(message) => {
                    self.say(message);
                    continue;
                }
            };
            let tripped = |load: &CycleLoad| load.tripped.then_some(("ui-analysis-tripped", String::new()));
            let note = match report {
                Report::Cycle(load, verdicts, bands, samples) => {
                    let note = tripped(&load);
                    (self.cycle_report, self.bands, self.band_samples) = (Some((load, verdicts)), bands, samples);
                    note
                }
                Report::Sensitivity(nominal, rows) => {
                    self.sensitivity = Some((nominal, rows));
                    None
                }
                Report::Optimised(found) => {
                    self.optimised = Some(found);
                    None
                }
                Report::Search(found, supply) => {
                    self.candidates = Some((found, supply));
                    None
                }
                Report::Path(run) => {
                    // The machine runs the path as well, if it is still the machine the path was planned
                    // for. It refuses by itself where it has moved since, or is busy.
                    let stale = config != self.config;
                    if !stale {
                        self.command(Command::Trajectory(run.points.clone()));
                    }
                    let note = tripped(&run.load).or(stale.then_some(("ui-analysis-stale", String::new())));
                    self.path_run = Some(run);
                    note
                }
                Report::Trace(fit) => {
                    self.trace_fit = Some(fit);
                    None
                }
            };
            self.computed_for[kind as usize] = Some(config);
            if let Some(note) = note {
                self.say(note);
            }
        }
    }

    /// Where the machine's setpoint stands, by the newest sample.
    fn standing(&self) -> Option<Vec<f64>> {
        let row = self.history.back()?;
        let signal = |axis: &sim_core::AxisInfo| self.session.signals.iter().position(|s| s.id == format!("axis.{}.position_setpoint", axis.id));
        self.session.axes.iter().map(|axis| Some(row[signal(axis)?])).collect()
    }

    // ---------------------------------------------------------------------------------------
    // Scopes

    /// Freezing keeps the traces as they are for a closer look; the simulation runs on.
    pub fn set_frozen(&mut self, frozen: bool) {
        self.frozen = frozen.then(|| self.history.clone());
    }

    /// The rows the scopes show: the last `window_s` seconds of the live or the frozen history.
    pub fn visible_rows(&self) -> impl Iterator<Item = &Vec<f64>> {
        let rows = self.frozen.as_ref().unwrap_or(&self.history);
        let from = rows.back().map_or(0.0, |last| last[0] - self.window_s);
        let first = rows.partition_point(|row| row[0] < from);
        rows.range(first..)
    }

    pub fn stats(&self, signal: usize) -> Option<Stats> {
        let (mut min, mut max, mut sum, mut n) = (f64::INFINITY, f64::NEG_INFINITY, 0.0, 0usize);
        for row in self.visible_rows() {
            let v = row[signal];
            (min, max, sum, n) = (min.min(v), max.max(v), sum + v * v, n + 1);
        }
        (n > 0).then(|| Stats { min, max, rms: (sum / n as f64).sqrt() })
    }

    /// Writes what the scopes show (every signal, the visible time window) as CSV.
    pub fn export_traces(&mut self, path: &std::path::Path) {
        let header: Vec<String> = self.session.signals.iter().map(|s| format!("{}[{}]", s.id, s.unit)).collect();
        let mut text = header.join(",") + "\n";
        for row in self.visible_rows() {
            let cells: Vec<String> = row.iter().map(|v| format!("{v:.9e}")).collect();
            text += &(cells.join(",") + "\n");
        }
        let message = match std::fs::write(path, text) {
            Ok(()) => ("ui-exported", path.display().to_string()),
            Err(e) => ("ui-export-failed", e.to_string()),
        };
        self.say(message);
    }

    // ---------------------------------------------------------------------------------------
    // Tuning

    pub fn tuning_running(&self) -> bool {
        self.tuning_job.is_some()
    }

    /// Measures step and frequency response of one axis with the config as it is now, in the
    /// background on a simulation of its own. The running machine is not disturbed.
    pub fn run_tuning(&mut self, axis: usize) {
        let (entry, config) = (self.entry, self.config.clone());
        let (result, job) = channel();
        std::thread::spawn(move || {
            let measure = || -> Result<Tuning, sim_core::ConfigError> {
                let step = step_response(entry, &config, axis, 0.001, 0.5)?;
                let response = closed_loop_response(entry, &config, axis, &log_spaced(1.0, 200.0, 40), 0.0005)?;
                Ok(Tuning { axis, step, bandwidth: bandwidth(&response), response })
            };
            let _ = result.send(measure().map_err(|e| (e.key(), e.detail().to_string())));
        });
        self.tuning_axis = axis;
        self.tuning_job = Some(Mutex::new(job));
    }

    // ---------------------------------------------------------------------------------------
    // Analysis

    pub fn analysis_running(&self) -> bool {
        !self.analysis_jobs.is_empty()
    }

    pub fn running(&self, kind: Analysis) -> bool {
        self.analysis_jobs.iter().any(|job| job.0 == kind)
    }

    pub fn has_result(&self, kind: Analysis) -> bool {
        self.computed_for[kind as usize].is_some()
    }

    /// The result of this kind was computed for a config that has been changed since. A motor search does
    /// not depend on the parts the config names, so naming one of its motors does not outdate it.
    pub fn outdated(&self, kind: Analysis) -> bool {
        self.computed_for[kind as usize].as_ref().is_some_and(|config| match kind {
            Analysis::Search => without_parts(config) != without_parts(&self.config),
            _ => *config != self.config,
        })
    }

    /// Starts one computation of the analysis panel with the config as it is now, in the background on
    /// simulations of its own, like the tuning measurement. The running machine is not disturbed.
    ///
    /// The search, the path and the trace take what `inputs` holds at this moment. A path is planned from
    /// where the machine stands, and when its analysis is back the machine runs it too (see `poll`).
    pub fn run_analysis(&mut self, kind: Analysis) {
        let (entry, config, inputs) = (self.entry, self.config.clone(), self.inputs.clone());
        if kind == Analysis::Search && !SUPPLIES.contains(&inputs.supply.as_str()) {
            return self.say(("ui-search-bad-supply", inputs.supply));
        }
        let (standing, motors) = (self.standing(), self.session.motors.clone());
        let (result, job) = channel();
        std::thread::spawn(move || {
            let refused = |e: ConfigError| (e.key(), e.detail().to_string());
            let command = |e: CommandError| (e.key, String::new());
            let read = |file: &str| std::fs::read_to_string(file).map_err(|e| ("ui-file-unreadable", format!("{file}: {e}")));
            let run = || -> Result<Report, Message> {
                Ok(match kind {
                    Analysis::Cycle => {
                        let load = analyse_cycle(&mut Sim::new(entry, &config).map_err(refused)?).map_err(command)?;
                        let verdicts = verdicts(&config, &load).map_err(refused)?;
                        // Two more cycles per bounded value, or one per sample (always with the same seed, so
                        // that asking again gives the same bands).
                        let banded = |bounds: Vec<(String, f64, f64)>| match inputs.samples {
                            0 => verdict_bands(entry, &config, &bounds),
                            samples => verdict_bands_sampled(entry, &config, &bounds, samples, 1),
                        };
                        let bands = if inputs.bands { Some(banded(bounds(&config).map_err(refused)?).map_err(refused)?) } else { None };
                        Report::Cycle(load, verdicts, bands, inputs.samples)
                    }
                    Analysis::Sensitivity => {
                        // The values the config calls uncertain: those it gives bounds. Two cycles each.
                        let paths: Vec<String> = bounds(&config).map_err(refused)?.into_iter().map(|bound| bound.0).collect();
                        let (nominal, mut rows) = sensitivity(entry, &config, &paths, 0.2).map_err(refused)?;
                        rows.sort_by(|a, b| b.strength(&nominal).total_cmp(&a.strength(&nominal)));
                        Report::Sensitivity(nominal, rows)
                    }
                    Analysis::Optimise => Report::Optimised(optimise_cycle(entry, &config, 40).map_err(refused)?),
                    Analysis::Search => {
                        let (series, gearboxes) = (inputs.series.trim(), Some(inputs.gearboxes.trim()).filter(|series| !series.is_empty()));
                        let found = search_motors(entry, &config, &inputs.supply, &|article| article.starts_with(series), gearboxes);
                        if found.is_empty() {
                            // Motors there are, so it is the gearbox series that has nothing for them.
                            let motors = sim_components::catalog::motor_articles().iter().any(|article| article.starts_with(series));
                            return Err(match gearboxes.filter(|_| motors) {
                                Some(gearboxes) => ("ui-search-no-gearbox", gearboxes.to_string()),
                                None => ("ui-search-none", series.to_string()),
                            });
                        }
                        Report::Search(found, inputs.supply.clone())
                    }
                    Analysis::Path => {
                        let unreadable = |line: String| ("ui-file-line", format!("{}: {line}", inputs.path_file));
                        let text = read(&inputs.path_file)?;
                        let mut from_here = config.clone();
                        if let Some(pose) = &standing {
                            from_here.set_text(&format!("start.pose = {pose:?}")).map_err(refused)?;
                        }
                        let mut sim = Sim::new(entry, &from_here).map_err(refused)?;
                        let (points, deviation) = if inputs.path_timed {
                            (path::parse_trajectory(&text).map_err(unreadable)?, 0.0)
                        } else {
                            let pieces = path::parse(&text).map_err(unreadable)?;
                            (sim.plan_path(&pieces, inputs.time_optimal).map_err(command)?, sim.path_deviation(&pieces).map_err(command)?)
                        };
                        let scene_line = points.iter().filter_map(|(_, pose)| sim.machine.scene_point(pose)).collect();
                        let load = analyse_trajectory(&mut sim, points.clone()).map_err(command)?;
                        let verdicts = verdicts(&config, &load).map_err(refused)?;
                        Report::Path(PathRun { points, scene_line, load, verdicts, deviation })
                    }
                    Analysis::Trace => {
                        let unfit = |e: TraceError| (e.key, e.detail);
                        let measured = Trace::parse(&read(&inputs.trace_file)?, &motors).map_err(unfit)?;
                        let simulated = replay(entry, &config, &measured).map_err(unfit)?;
                        let apart = |(m, s): (&sim_components::trace::MotorTrace, &sim_components::trace::MotorTrace)| {
                            let (torque, error) = (difference(&m.torque, &s.torque), difference(&m.following_error(), &s.following_error()));
                            [torque.0, torque.1, error.0, error.1]
                        };
                        let differences = measured.motors.iter().zip(&simulated.motors).map(apart).collect();
                        Report::Trace(TraceFit { differences, fits: fit_rigid(entry, &config, &measured).map_err(unfit)? })
                    }
                })
            };
            let _ = result.send(run());
        });
        self.analysis_jobs.push((kind, self.config.clone(), Mutex::new(job)));
    }

    /// Forgets the result of one kind of analysis.
    pub fn clear(&mut self, kind: Analysis) {
        match kind {
            Analysis::Cycle => (self.cycle_report, self.bands) = (None, None),
            Analysis::Sensitivity => self.sensitivity = None,
            Analysis::Optimise => self.optimised = None,
            Analysis::Search => self.candidates = None,
            Analysis::Path => (self.path_run, self.trail) = (None, None),
            Analysis::Trace => self.trace_fit = None,
        }
        self.computed_for[kind as usize] = None;
    }

    pub fn clear_analysis(&mut self) {
        [Analysis::Cycle, Analysis::Sensitivity, Analysis::Optimise].into_iter().for_each(|kind| self.clear(kind));
    }

    /// Names a motor of the search (and its gearbox, where it was searched with one) in `[parts]` of the
    /// config and applies it to the running simulation: the row of `search_table` after the header. The
    /// candidate passed or failed on the config that was searched, so over a config changed since in
    /// anything but its part names nothing is applied.
    pub fn use_candidate(&mut self, row: usize) {
        let (Some((found, supply)), Some(searched)) = (&self.candidates, &self.computed_for[Analysis::Search as usize]) else { return };
        let Some(candidate) = found.get(row) else { return };
        if self.outdated(Analysis::Search) {
            return self.say(("ui-analysis-stale", String::new()));
        }
        // A candidate without a gearbox of its own ran with the one the searched config names.
        let own = searched.get("parts.gearbox").ok().and_then(|name| name.as_str()).unwrap_or("");
        let names = [candidate.article, supply.as_str(), if candidate.gearbox.is_empty() { own } else { &candidate.gearbox }];
        let mut config = self.config.clone();
        let named = PARTS.iter().zip(names).try_for_each(|(key, name)| config.set_text(&format!("parts.{key} = {name:?}")));
        match named {
            Ok(()) => {
                self.config = config;
                self.reconfigure();
            }
            Err(e) => self.say((e.key(), e.detail().into())),
        }
    }

    /// Puts the settings the optimiser found into the config and applies it to the running simulation.
    /// They are absolute values, found for the config the optimiser started with: over a config edited
    /// since they would undo an edit, or fail a check they were found to pass. Then nothing is applied; the
    /// result stays, for when the edit is taken back.
    pub fn apply_optimised(&mut self) {
        if self.optimised.is_some() && self.outdated(Analysis::Optimise) {
            return self.say(("ui-analysis-stale", String::new()));
        }
        let Some(found) = self.optimised.take() else { return };
        for (path, _, after) in &found.changes {
            if let Err(e) = self.config.set_text(&format!("{path} = {after}")) {
                self.say((e.key(), e.detail().into()));
            }
        }
        self.reconfigure();
    }

    fn header(&self, keys: &[&str]) -> Vec<String> {
        keys.iter().map(|key| self.tr.tr(key).to_string()).collect()
    }

    /// "X axis · Highest velocity [m/s]" for `axis.x.max_velocity_m_s`, as the parameter panel names it.
    fn param_label(&self, path: &str) -> String {
        match path.rsplit_once('.') {
            Some(("effects", leaf)) => self.tr.tr(&format!("effect-{leaf}")).to_string(),
            Some((table, leaf)) => format!("{} · {}", self.tr.tr(&format!("group-{}", table.replace('.', "-"))), self.tr.tr(&format!("param-{leaf}"))),
            None => path.to_string(),
        }
    }

    /// "Figure" and the name of every motor: the header of a table with one row per figure.
    fn per_motor(&self) -> Vec<String> {
        [self.header(&["ui-analysis-figure"]), self.session.motors.iter().map(|m| self.tr.tr(m.key).to_string()).collect()].concat()
    }

    fn result(&self, ok: bool) -> String {
        self.tr.tr(if ok { "ui-analysis-ok" } else { "ui-analysis-fail" }).to_string()
    }

    /// A run as tables, each narrow enough for the panel: the figures of the machine, one per row; the load
    /// on the motors, one row per figure and one column per motor; one row per check with its value, its
    /// limit and the result; and where there are error bands, a fourth with the lowest and the highest
    /// value of every check, its worst-case result (also of a check that only a bound brings up) and the
    /// share of the runs it passed, in percent. A figure
    /// the machine does not report is left out. `deviation` is that of a path, which has no cycle time and no dwell.
    fn load_tables(&self, kind: Analysis, load: &CycleLoad, verdicts: &[Verdict], bands: Option<&[Band]>, deviation: Option<f64>) -> Vec<Table> {
        let tr = |key: &str| self.tr.tr(key).to_string();
        let path = deviation.is_some();
        let life = guide_life_km(self.computed_for[kind as usize].as_ref().unwrap_or(&self.config), load);
        // In the order of `CYCLE_KEYS`; lengths in mm.
        let figures = [
            (!path).then_some(load.duration),
            path.then_some(load.duration),
            deviation.map(|d| d * 1e3),
            Some(load.path_error_peak * 1e3),
            path.then_some(load.tracking_error_peak * 1e3),
            (!path).then_some(load.dwell_error_peak * 1e3),
            load.belt_tension.map(|(lowest, _)| lowest),
            load.belt_slack_time,
            load.radial_load_peak,
            load.guide_load.map(|(peak, _)| peak),
            life,
        ];
        let mut summary = vec![self.header(&["ui-analysis-figure", "ui-analysis-value"])];
        summary.extend(CYCLE_KEYS.iter().zip(figures).filter_map(|(key, value)| Some(vec![tr(key), short(value?)])));
        // In the order of `MOTOR_KEYS`.
        let values = |m: &MotorLoad| [m.peak_torque, m.rms_torque, m.cubic_mean_torque, m.winding_rise, m.energy_in, m.energy_regenerated, m.resistor_energy, m.resistor_energy / load.duration, m.peak_dc_link_voltage];
        let mut motors = vec![self.per_motor()];
        for (i, key) in MOTOR_KEYS.iter().enumerate() {
            motors.push([vec![tr(key)], load.motors.iter().map(|m| short(values(m)[i])).collect()].concat());
        }
        let motor = |k: usize| tr(self.session.motors[k].key);
        let named = |v: &Verdict| vec![tr(&format!("check-{}", v.check)), v.motor.map_or(String::new(), motor)];
        let mut checks = vec![self.header(&VERDICT_KEYS)];
        checks.extend(verdicts.iter().map(|v| [named(v), vec![short(v.value), short(v.limit), self.result(v.ok)]].concat()));
        let mut tables = vec![summary, motors, checks];
        if let Some(bands) = bands {
            let mut banded = vec![self.header(&BAND_KEYS)];
            banded.extend(bands.iter().map(|b| [named(&b.verdict), vec![short(b.low), short(b.high), self.result(b.ok), short(100.0 * b.share_shown(4))]].concat()));
            tables.push(banded);
        }
        tables
    }

    /// The analysed cycle as the tables of `load_tables`. Empty until a cycle was analysed.
    pub fn cycle_tables(&self) -> Vec<Table> {
        let Some((load, verdicts)) = &self.cycle_report else { return Vec::new() };
        self.load_tables(Analysis::Cycle, load, verdicts, self.bands.as_deref(), None)
    }

    /// The motors of the search as ranked: passing ones first, lightest first. Per motor its gearbox (empty:
    /// the config's own), mass, result, and the check nearest to its limit with value over limit, or why the
    /// cycle was not run. No more columns than the panel is wide: the cycle analysis has the rest.
    pub fn search_table(&self) -> Option<Table> {
        let (found, _) = self.candidates.as_ref()?;
        let tr = |key: &str| self.tr.tr(key).to_string();
        let mut table = vec![self.header(&SEARCH_KEYS)];
        for c in found {
            let reason = format!("ui-search-{}", c.refused);
            let (check, share) = match (c.tightest(), &c.result) {
                (Some((check, share)), _) => (tr(&format!("check-{check}")), short(share)),
                (None, Some(_)) => (String::new(), String::new()),
                (None, None) => (tr(if self.tr.has(&reason) { &reason } else { "ui-search-refused" }), String::new()),
            };
            table.push(vec![c.article.to_string(), c.gearbox.clone(), short(c.mass), self.result(c.passes()), check, share]);
        }
        Some(table)
    }

    /// The path that was run as the three tables of `load_tables`. Empty until a path was run.
    pub fn path_tables(&self) -> Vec<Table> {
        let Some(run) = &self.path_run else { return Vec::new() };
        self.load_tables(Analysis::Path, &run.load, &run.verdicts, None, Some(run.deviation))
    }

    /// The planned path for a plot, with the labels of its two directions: the second axis over the first,
    /// or on a machine with one axis its position over the time.
    pub fn path_plot(&self) -> Option<(String, String, Vec<[f64; 2]>)> {
        let run = self.path_run.as_ref()?;
        let axis = |i: usize| format!("{} [{}]", self.tr.tr(self.session.axes[i].key), self.session.axes[i].unit);
        Some(match self.session.axes.len() {
            1 => (self.tr.tr("ui-time").to_string(), axis(0), run.points.iter().map(|(time, pose)| [*time, pose[0]]).collect()),
            _ => (axis(0), axis(1), run.points.iter().map(|(_, pose)| [pose[0], pose[1]]).collect()),
        })
    }

    /// The replayed trace as two tables: per motor how far the simulation is from the trace (torque and
    /// following error, RMS and peak of the difference), and per axis and fitted value what the config has,
    /// what the trace says and the standard error of that. Empty until a trace was replayed.
    pub fn trace_tables(&self) -> Vec<Table> {
        let Some(fit) = &self.trace_fit else { return Vec::new() };
        let tr = |key: &str| self.tr.tr(key).to_string();
        let mut replayed = vec![self.per_motor()];
        for (i, key) in TRACE_KEYS.iter().enumerate() {
            replayed.push([vec![tr(key)], fit.differences.iter().map(|d| short(d[i])).collect()].concat());
        }
        let mut fitted = vec![self.header(&FIT_KEYS)];
        for (axis, fit) in self.session.axes.iter().zip(&fit.fits) {
            let Some(fit) = fit else { continue };
            for (q, key) in FIT_QUANTITIES.iter().enumerate() {
                fitted.push(vec![format!("{} · {}", tr(axis.key), tr(key)), short(fit.config[q]), short(fit.fitted[q]), short(fit.error[q])]);
            }
        }
        vec![replayed, fitted]
    }

    /// What the fit says to change in the config, one line per value, and which axis it could not fit.
    pub fn trace_advice(&self) -> Vec<String> {
        let Some(fit) = &self.trace_fit else { return Vec::new() };
        let lines = self.session.axes.iter().zip(&fit.fits).flat_map(|(axis, fit)| match fit {
            Some(fit) => fit.advice(axis.id).to_vec(),
            None => vec![("fit-axis-skipped", axis.id.to_string())],
        });
        lines.map(|(key, detail)| self.tr.tr_arg(key, &detail)).collect()
    }

    /// The parameters the cycle depends on most, strongest first: the low and high value each was run
    /// with, and what that did to peak torque, RMS torque and cycle time, in percent.
    pub fn sensitivity_table(&self) -> Option<Table> {
        let (nominal, rows) = self.sensitivity.as_ref()?;
        let mut table = vec![self.header(&SENSITIVITY_KEYS)];
        for row in rows.iter().take(TOP_ROWS) {
            let c = row.changes(nominal).map(|change| change * 100.0);
            let pair = |low: f64, high: f64| format!("{low:+.1} / {high:+.1}");
            table.push(vec![self.param_label(&row.path), short(row.bounds.0), short(row.bounds.1), pair(c[0], c[1]), pair(c[2], c[3]), pair(c[4], c[5])]);
        }
        Some(table)
    }

    /// The optimiser's result: cycle time and settling before and after, and every setting it would change.
    pub fn optimised_table(&self) -> Option<Table> {
        let found = self.optimised.as_ref()?;
        let mut table = vec![self.header(&OPTIMISED_KEYS)];
        table.push(vec![self.tr.tr("ui-analysis-cycle-time").to_string(), short(found.before.duration), short(found.after.duration)]);
        table.push(vec![self.tr.tr("ui-analysis-dwell-error").to_string(), short(found.before.dwell_error_peak * 1e3), short(found.after.dwell_error_peak * 1e3)]);
        for (path, before, after) in &found.changes {
            table.push(vec![self.param_label(path), before.clone(), after.clone()]);
        }
        Some(table)
    }

    // ---------------------------------------------------------------------------------------
    // Labels and panels

    /// "X axis · Following error [m]"
    pub fn signal_label(&self, signal: &SignalInfo) -> String {
        format!("{} · {} [{}]", self.tr.tr(&signal.owner_key), self.tr.tr(&signal.quantity_key), signal.unit)
    }

    /// The effect switches: (config path, label key, on).
    pub fn effects(&self) -> Vec<(String, String, bool)> {
        let leaves = self.config.leaves().into_iter();
        leaves.filter_map(|(path, value)| Some((path.clone(), format!("effect-{}", path.strip_prefix("effects.")?), value.as_bool()?))).collect()
    }

    /// The parameter panel: every numeric config value, grouped by its table.
    pub fn param_groups(&self) -> Vec<ParamGroup> {
        let mut groups: Vec<ParamGroup> = Vec::new();
        for (path, value) in self.config.leaves() {
            let Some((table, leaf)) = path.rsplit_once('.') else { continue };
            if table == "effects" || !(value.is_float() || value.is_integer()) {
                continue;
            }
            let key = format!("group-{}", table.replace('.', "-"));
            let param = (path.clone(), format!("param-{leaf}"), value);
            match groups.iter_mut().find(|g| g.key == key) {
                Some(group) => group.params.push(param),
                None => groups.push(ParamGroup { key, params: vec![param] }),
            }
        }
        groups
    }

    /// Every translation key the panels and the 3D view can show for the current machine.
    pub fn keys_in_use(&self) -> Vec<String> {
        let mut keys = vec![self.entry.key.to_string()];
        keys.extend(MACHINES.iter().map(|m| m.key.to_string()));
        keys.extend(self.session.signals.iter().flat_map(|s| [s.owner_key.clone(), s.quantity_key.clone()]));
        keys.extend(self.scopes.iter().map(|s| s.quantity_key.clone()));
        keys.extend(self.scene.labels.iter().flat_map(|l| [l.owner_key.to_string(), l.unit_key.to_string()]));
        keys.extend(self.effects().into_iter().map(|(_, key, _)| key));
        for group in self.param_groups() {
            keys.push(group.key);
            keys.extend(group.params.into_iter().map(|(_, key, _)| key));
        }
        let headers = [&CYCLE_KEYS[..], &MOTOR_KEYS, &VERDICT_KEYS, &BAND_KEYS, &SENSITIVITY_KEYS, &OPTIMISED_KEYS, &SEARCH_KEYS, &TRACE_KEYS, &FIT_KEYS, &FIT_QUANTITIES, &NOTES];
        keys.extend(headers.concat().into_iter().map(String::from));
        keys.extend(["ui-analysis-ok", "ui-analysis-fail", "fit-axis-skipped", "fit-mass", "fit-viscous", "fit-coulomb", "fit-constant"].map(String::from));
        keys.extend(CHECKS.iter().map(|check| format!("check-{check}")));
        keys
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    /// A background job whose thread ended without an answer (it panicked) left no message: the analysis
    /// was dropped without a word, and the tuning's spinner turned for ever.
    #[test]
    fn a_job_that_ends_without_an_answer_is_reported() {
        let mut model = Model::new(MACHINES[0], "en");
        // What is left of a thread that panicked: the receiving end of its channel.
        model.analysis_jobs.push((Analysis::Cycle, model.config.clone(), Mutex::new(channel().1)));
        model.tuning_job = Some(Mutex::new(channel().1));
        assert!(model.analysis_running() && model.tuning_running());
        model.poll();
        assert!(!model.analysis_running() && !model.tuning_running());
        assert_eq!(model.messages.last(), Some(&("ui-job-ended", String::new())), "{:?}", model.messages);
        // A job that answers is taken as before.
        let (result, job) = channel();
        result.send(Err(("error-busy", String::new()))).unwrap();
        model.analysis_jobs.push((Analysis::Cycle, model.config.clone(), Mutex::new(job)));
        model.poll();
        assert_eq!((model.analysis_running(), model.messages.last().unwrap().0), (false, "error-busy"));
    }

    /// A simulation thread that is gone (it panicked) left a frozen view and no message. It is reported, once.
    #[test]
    fn a_simulation_thread_that_ends_is_reported_once() {
        let mut model = Model::new(MACHINES[0], "en");
        model.poll();
        let reports = |model: &Model| model.messages.iter().filter(|m| m.0 == "ui-simulation-ended").count();
        assert_eq!(reports(&model), 0);
        // The thread ends while the session lives, which nothing but a panic does to it otherwise.
        model.session.send(Request::Quit);
        let start = Instant::now();
        while !model.session.ended() && start.elapsed() < Duration::from_secs(5) {
            std::thread::sleep(Duration::from_millis(5));
        }
        model.poll();
        assert_eq!(reports(&model), 1, "{:?}", model.messages);
        model.command(Command::MoveTo(Vec::new()));
        model.set_param("no.such_value", Value::Float(1.0));
        model.poll();
        assert_eq!(reports(&model), 1, "{:?}", model.messages);
    }
}
