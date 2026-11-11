//! Everything the interface shows and edits, without any drawing: the chosen machine and language,
//! the config being edited, the scope history, tuning results. The window code only draws this and
//! calls its methods.

use crate::session::{Event, Request, Session};
use sim_components::Command;
use sim_components::analysis::{StepResponse, SweepPoint, bandwidth, closed_loop_response, log_spaced, step_response};
use sim_core::{Config, MachineEntry, Scene, SignalInfo, Translations};
use sim_machines::{MACHINES, translations};
use std::collections::VecDeque;
use std::sync::Mutex;
use std::sync::mpsc::{Receiver, channel};
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

pub struct Model {
    pub entry: &'static MachineEntry,
    pub lang: &'static str,
    pub tr: Translations,
    pub config: Config,
    pub session: Session,
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
    tuning_job: Option<Mutex<Receiver<Result<Tuning, Message>>>>,
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
            config,
            session,
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
        };
        model.target = model.config.f64s("start.pose").unwrap_or_default();
        model
    }

    /// Switching machine restarts the simulation with that machine's default config.
    pub fn set_machine(&mut self, entry: &'static MachineEntry) {
        let (lang, paused, speed, window) = (self.lang, self.paused, self.speed, self.window_s);
        *self = Self::new(entry, lang);
        self.window_s = window;
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

    /// Edits one config value and applies the config to the running simulation.
    pub fn set_param(&mut self, path: &str, value: Value) {
        match self.config.set(path, value) {
            Ok(()) => self.session.send(Request::Reconfigure(self.config.clone())),
            Err(e) => self.messages.push((e.key(), e.detail().into())),
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
                Event::Refused(key, detail) => self.messages.push((key, detail)),
                Event::Limits(violations) => {
                    self.messages.retain(|(key, _)| !key.starts_with("limit-"));
                    for v in violations {
                        let motor = self.tr.tr(self.session.motors[v.motor].key).to_string();
                        self.messages.push((v.key, format!("{motor}: {:.4} > {:.4}", v.needed, v.allowed)));
                    }
                }
            }
        }
        let finished = self.tuning_job.as_ref().and_then(|job| job.lock().ok()?.try_recv().ok());
        if let Some(result) = finished {
            self.tuning_job = None;
            match result {
                Ok(tuning) => self.tuning = Some(tuning),
                Err(message) => self.messages.push(message),
            }
        }
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
        self.messages.push(message);
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
        keys
    }
}
