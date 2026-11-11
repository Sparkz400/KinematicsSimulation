//! Everything the interface shows and edits, without any drawing: the chosen machine and language,
//! the config being edited, the scope history. The window code only draws this and calls its methods.

use crate::session::{Event, Request, Session};
use sim_components::Command;
use sim_core::{Config, MachineEntry, Scene, SignalInfo, Translations};
use sim_machines::{MACHINES, translations};
use std::collections::VecDeque;
use toml::Value;

/// Samples kept for the scopes: ten seconds at one per millisecond.
pub const HISTORY: usize = 10_000;

/// A message for the user, as a translation key and its argument.
pub type Message = (&'static str, String);

/// One group of the parameter panel: a config table and its scalar values.
pub struct ParamGroup {
    pub key: String,
    /// (config path, label key, value)
    pub params: Vec<(String, String, Value)>,
}

/// One scope: every signal of one quantity, e.g. the following error of each axis and motor.
pub struct Scope {
    pub quantity_key: String,
    pub unit: String,
    /// Indices into the signal list.
    pub signals: Vec<usize>,
    pub visible: bool,
}

pub struct Model {
    pub entry: &'static MachineEntry,
    pub lang: &'static str,
    pub tr: Translations,
    pub config: Config,
    pub session: Session,
    pub scopes: Vec<Scope>,
    pub history: VecDeque<Vec<f64>>,
    pub scene: Scene,
    pub paused: bool,
    pub speed: f64,
    /// Target of the next point-to-point move, per axis.
    pub target: Vec<f64>,
    pub messages: Vec<Message>,
}

fn texts(lang: &str, entry: &MachineEntry) -> Translations {
    let mut tr = translations(lang, entry);
    crate::I18N.iter().filter(|(l, _)| *l == lang).for_each(|(_, text)| tr.add(text));
    tr
}

/// The scopes follow from the signal list alone: one per quantity, in order of first appearance.
fn scopes_for(signals: &[SignalInfo]) -> Vec<Scope> {
    const SHOWN_AT_START: [&str; 4] = ["quantity-position", "quantity-following-error", "quantity-speed", "quantity-torque"];
    let mut scopes: Vec<Scope> = Vec::new();
    for (i, signal) in signals.iter().enumerate().filter(|(_, s)| s.id != "time") {
        match scopes.iter_mut().find(|s| s.quantity_key == signal.quantity_key && s.unit == signal.unit) {
            Some(scope) => scope.signals.push(i),
            None => scopes.push(Scope {
                quantity_key: signal.quantity_key.clone(),
                unit: signal.unit.clone(),
                signals: vec![i],
                visible: SHOWN_AT_START.contains(&signal.quantity_key.as_str()),
            }),
        }
    }
    scopes
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
            history: VecDeque::with_capacity(HISTORY),
            scene: Scene::default(),
            paused: false,
            speed: 1.0,
            target: Vec::new(),
            messages: Vec::new(),
        };
        model.target = model.config.f64s("start.pose").unwrap_or_default();
        model
    }

    /// Switching machine restarts the simulation with that machine's default config.
    pub fn set_machine(&mut self, entry: &'static MachineEntry) {
        let (lang, paused, speed) = (self.lang, self.paused, self.speed);
        *self = Self::new(entry, lang);
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

    /// Takes in what the simulation has produced since the last frame.
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
    }

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

    /// Every translation key the panels can show for the current machine.
    pub fn keys_in_use(&self) -> Vec<String> {
        let mut keys = vec![self.entry.key.to_string()];
        keys.extend(MACHINES.iter().map(|m| m.key.to_string()));
        keys.extend(self.session.signals.iter().flat_map(|s| [s.owner_key.clone(), s.quantity_key.clone()]));
        keys.extend(self.effects().into_iter().map(|(_, key, _)| key));
        for group in self.param_groups() {
            keys.push(group.key);
            keys.extend(group.params.into_iter().map(|(_, key, _)| key));
        }
        keys
    }
}
