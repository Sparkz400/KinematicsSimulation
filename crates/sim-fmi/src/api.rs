//! The FMI 2.0 Co-Simulation C functions, and the instance both FMI versions drive (`api3` has the FMI 3.0 ones).
//! The type definitions follow the FMI 2.0 headers (fmi2TypesPlatform.h, fmi2FunctionTypes.h; Copyright Modelica
//! Association Project "FMI", BSD-2-Clause licence), written out in Rust.
//!
//! Supported: instantiation, setup, initialisation, `fmi2DoStep`, get and set of Real, Integer (there are none),
//! Boolean and String variables, reset, terminate, FMU state get, set, free and (de)serialization, directional
//! derivatives. The rest (input derivatives, asynchronous steps, status queries) return `fmi2Error`; the model
//! description says so. A panic inside the simulation returns `fmi2Fatal` instead of taking the importing tool down.
//!
//! An FMU state is what the importer set, the triggers as the last step saw them, and a snapshot of the simulation
//! (`Sim::snapshot`) with its outputs. Setting it builds a fresh simulation from the parameters and restores the snapshot
//! into it, which takes as long as building the machine, however long the simulation ran. Serialized, the snapshot is
//! written as bytes (`postcard`, behind a header with the format and the FMU's version), so a state from
//! `fmi2DeSerializeFMUstate` is set as fast. Either gives bit for bit the simulation that was saved (tested). A state of
//! another format or version of the FMU is refused by name; one that cannot be set (its config file gone, say) leaves
//! the instance as it was.
//!
//! Log categories (`LOG_CATEGORIES`): `logEvents` (the commands the trigger inputs gave, states set) is on when the
//! instance was made with logging on; `logStatusWarning` and `logStatusError` are on from the start; `SetDebugLogging`
//! switches them, `logAll` or no category naming all.
//! The directional derivative of an output with respect to an input is exactly zero at a communication point: the
//! outputs are the signals at the end of the last step, and a new input acts on them only through the next step
//! (the model description says `dependencies=""`).

#![allow(non_snake_case, non_camel_case_types, non_upper_case_globals)]

use crate::api3::fmi3LogMessageCallback;
use crate::{Kind, LOG_CATEGORIES, Model, texts};
use sim_components::{Command, Sim, Snapshot, load_config};
use sim_core::Translations;
use std::ffi::{CStr, CString, c_char, c_int, c_void};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::{Path, PathBuf};

pub type fmi2Component = *mut c_void;
pub type fmi2ComponentEnvironment = *mut c_void;
pub type fmi2FMUstate = *mut c_void;
pub type fmi2ValueReference = u32;
pub type fmi2Real = f64;
pub type fmi2Integer = c_int;
pub type fmi2Boolean = c_int;
pub type fmi2String = *const c_char;
pub type fmi2Byte = c_char;
pub type fmi2Status = c_int;

pub const fmi2OK: fmi2Status = 0;
pub const fmi2Warning: fmi2Status = 1;
pub const fmi2Discard: fmi2Status = 2;
pub const fmi2Error: fmi2Status = 3;
pub const fmi2Fatal: fmi2Status = 4;
/// `fmi2Type`: the one this FMU supports.
pub const fmi2CoSimulation: c_int = 1;

pub type fmi2CallbackLogger =
    unsafe extern "C" fn(fmi2ComponentEnvironment, fmi2String, fmi2Status, fmi2String, fmi2String, ...);

#[repr(C)]
pub struct fmi2CallbackFunctions {
    pub logger: Option<fmi2CallbackLogger>,
    pub allocateMemory: Option<unsafe extern "C" fn(usize, usize) -> *mut c_void>,
    pub freeMemory: Option<unsafe extern "C" fn(*mut c_void)>,
    pub stepFinished: Option<unsafe extern "C" fn(fmi2ComponentEnvironment, fmi2Status)>,
    pub componentEnvironment: fmi2ComponentEnvironment,
}

/// The state of an instance; `Stepping` is FMI 3.0's Step Mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Mode {
    Instantiated,
    Initialising,
    Stepping,
    Terminated,
}

/// A value the importer has set.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum Value {
    Real(f64),
    Bool(bool),
    Text(String),
}

/// A refusal: a translation key and its argument.
pub(crate) type Fail = (&'static str, String);

/// The logger of the FMI version the instance was made by. Status codes are the same numbers in both versions.
#[derive(Clone, Copy)]
pub(crate) enum Logger {
    Fmi2(Option<fmi2CallbackLogger>),
    Fmi3(Option<fmi3LogMessageCallback>),
}

/// What one communication step gives the simulation.
struct Applied {
    steps: u32,
    enable: bool,
    /// The commands the triggers gave at its start: `StartCycle`, `Stop`, `MoveTo`, and `Trajectory` with the path as
    /// it was planned then.
    commands: Vec<Command>,
    /// Per axis, the setpoint input if the importer has set it.
    setpoints: Vec<Option<f64>>,
}

/// The places in `Instance::logging` (and in `LOG_CATEGORIES`).
const EVENTS: usize = 0;
const WARNINGS: usize = 1;
const ERRORS: usize = 2;

/// An FMU state: what the importer set, the triggers as the last step saw them, and in step mode the simulation's
/// snapshot with its outputs.
#[derive(Clone)]
pub struct FmuState {
    guid: String,
    mode: Mode,
    start_time: f64,
    set: Vec<Option<Value>>,
    latched: [bool; 3],
    snapshot: Option<(Snapshot, Vec<f64>)>,
}

pub struct Instance {
    name: CString,
    logger: Logger,
    environment: *mut c_void,
    tr: Translations,
    pub(crate) model: Model,
    /// Per variable, what the importer set; none: the start value.
    set: Vec<Option<Value>>,
    mode: Mode,
    start_time: f64,
    /// Built from the parameters when initialisation needs it, and dropped when a parameter changes before that.
    sim: Option<Sim>,
    sample: Vec<f64>,
    /// What `fmi2GetString` handed out, valid until its next call.
    strings: Vec<CString>,
    /// The triggers `command.cycle`, `command.move`, `command.run_path` as the last communication step saw them.
    latched: [bool; 3],
    /// Per log category (`EVENTS`, `WARNINGS`, `ERRORS`): whether its messages go to the logger.
    logging: [bool; 3],
}

pub(crate) fn c_text(text: &str) -> CString {
    CString::new(text.replace('\0', "")).unwrap_or_default()
}

fn log(logger: Logger, environment: *mut c_void, name: &CStr, status: fmi2Status, category: usize, message: &str) {
    let category = c_text(LOG_CATEGORIES[category].0);
    match logger {
        // FMI 2.0's message is a printf format: a per cent sign in the text is doubled, so that it is no format, and
        // a logger that prints the format as it is (fmpy's Python callbacks) still shows the text.
        Logger::Fmi2(Some(logger)) => unsafe {
            let format = c_text(&message.replace('%', "%%"));
            logger(environment, name.as_ptr(), status, category.as_ptr(), format.as_ptr())
        },
        Logger::Fmi3(Some(logger)) => unsafe {
            let message = c_text(message);
            logger(environment, status, category.as_ptr(), message.as_ptr())
        },
        _ => {}
    }
}

/// The resources folder from its URI (`file:///C:/dir/resources`, `file:/dir`, or a plain path).
fn resource_dir(uri: &str) -> PathBuf {
    let path = uri.strip_prefix("file://").or_else(|| uri.strip_prefix("file:")).unwrap_or(uri);
    let path = path.strip_prefix("localhost").unwrap_or(path);
    let mut bytes = Vec::new();
    let raw = path.as_bytes();
    let mut i = 0;
    while i < raw.len() {
        let hex = |k: usize| raw.get(k).and_then(|c| (*c as char).to_digit(16));
        match (raw[i], hex(i + 1), hex(i + 2)) {
            (b'%', Some(high), Some(low)) => {
                bytes.push((high * 16 + low) as u8);
                i += 3;
            }
            (c, _, _) => {
                bytes.push(c);
                i += 1;
            }
        }
    }
    let path = String::from_utf8_lossy(&bytes).into_owned();
    // `/C:/dir` is a Windows path with the slash of the URI in front.
    let windows = cfg!(windows) && path.starts_with('/') && path.as_bytes().get(2) == Some(&b':');
    PathBuf::from(if windows { &path[1..] } else { &path })
}

/// # Safety
/// The pointer is null or points to a NUL-terminated string.
pub(crate) unsafe fn text<'a>(p: *const c_char) -> &'a str {
    if p.is_null() { "" } else { unsafe { CStr::from_ptr(p) }.to_str().unwrap_or("") }
}

/// `n` values at `p`; refused when `p` is null and `n` is not 0.
///
/// # Safety
/// The pointer is null or points to `n` values, or `n` is 0.
pub(crate) unsafe fn slice<'a, T>(p: *const T, n: usize) -> Result<&'a [T], Fail> {
    match (n, p.is_null()) {
        (0, _) => Ok(&[]),
        (_, true) => Err(("fmi-error-null", n.to_string())),
        _ => Ok(unsafe { std::slice::from_raw_parts(p, n) }),
    }
}

/// # Safety
/// As `slice`, writable.
pub(crate) unsafe fn slice_mut<'a, T>(p: *mut T, n: usize) -> Result<&'a mut [T], Fail> {
    match (n, p.is_null()) {
        (0, _) => Ok(&mut []),
        (_, true) => Err(("fmi-error-null", n.to_string())),
        _ => Ok(unsafe { std::slice::from_raw_parts_mut(p, n) }),
    }
}

/// The outputs: every signal, the time on the importer's clock, which starts at its start time.
fn sample_of(sim: &Sim, start_time: f64) -> Vec<f64> {
    let mut out = Vec::new();
    sim.sample(&mut out);
    out[0] += start_time;
    out
}

/// Gives the simulation one communication step's inputs and runs its base steps; the keys of the commands the
/// controller refused.
fn apply(sim: &mut Sim, applied: &Applied) -> Vec<&'static str> {
    let mut refused = Vec::new();
    if applied.enable != sim.control.enabled()
        && let Err(e) = sim.command(if applied.enable { Command::Enable } else { Command::Disable })
    {
        refused.push(e.key);
    }
    for command in &applied.commands {
        if let Err(e) = sim.command(command.clone()) {
            refused.push(e.key);
        }
    }
    if sim.control.enabled() {
        // An axis the importer has not set stays at the setpoint the controller follows.
        let mut target = sim.control.external_setpoint().to_vec();
        for (t, s) in target.iter_mut().zip(&applied.setpoints) {
            if let Some(s) = s {
                *t = *s;
            }
        }
        if target != sim.control.external_setpoint()
            && let Err(e) = sim.command(Command::Setpoint(target))
        {
            refused.push(e.key);
        }
    }
    for _ in 0..applied.steps {
        sim.step();
    }
    refused
}

impl Instance {
    pub(crate) fn fail(&self, status: fmi2Status, (key, arg): Fail) -> fmi2Status {
        self.note(ERRORS, status, &self.tr.tr_arg(key, &arg));
        status
    }

    /// A message to the logger, if its category is on.
    fn note(&self, category: usize, status: fmi2Status, message: &str) {
        if self.logging[category] {
            log(self.logger, self.environment, &self.name, status, category, message);
        }
    }

    /// `fmi2SetDebugLogging`, `fmi3SetDebugLogging`: the categories named switched on or off; none named, all of them.
    /// An unknown category is refused and changes nothing.
    pub(crate) fn set_debug_logging(&mut self, on: bool, categories: &[&str]) -> Result<fmi2Status, Fail> {
        let mut next = if categories.is_empty() { [on; 3] } else { self.logging };
        for name in categories {
            match LOG_CATEGORIES.iter().position(|(n, _)| n == name) {
                Some(i) if i <= ERRORS => next[i] = on,
                // logAll
                Some(_) => next = [on; 3],
                None => return Err(("fmi-error-category", name.to_string())),
            }
        }
        self.logging = next;
        Ok(fmi2OK)
    }

    /// `call` without its prefix, which is the version's.
    pub(crate) fn in_mode(&self, modes: &[Mode], call: &str) -> Result<(), Fail> {
        let prefix = if matches!(self.logger, Logger::Fmi3(_)) { "fmi3" } else { "fmi2" };
        if modes.contains(&self.mode) {
            Ok(())
        } else {
            Err(("fmi-error-state", format!("{prefix}{call} ({:?})", self.mode)))
        }
    }

    pub(crate) fn variable(&self, vr: u32, type_name: &str) -> Result<usize, Fail> {
        match self.model.variables.get(vr as usize) {
            Some(v) if v.type_name() == type_name => Ok(vr as usize),
            _ => Err(("fmi-error-reference", format!("{type_name} {vr}"))),
        }
    }

    fn start(&self, index: usize) -> &str {
        self.model.variables[index].start.as_deref().unwrap_or("")
    }

    pub(crate) fn text_value(&self, index: usize) -> String {
        self.text_in(&self.set, index)
    }

    fn text_in(&self, set: &[Option<Value>], index: usize) -> String {
        match &set[index] {
            Some(Value::Text(t)) => t.clone(),
            _ => self.start(index).into(),
        }
    }

    pub(crate) fn bool_value(&self, index: usize) -> bool {
        match self.set[index] {
            Some(Value::Bool(b)) => b,
            _ => self.start(index) == "true",
        }
    }

    fn real_value(&self, index: usize) -> f64 {
        match self.set[index] {
            Some(Value::Real(v)) => v,
            _ => self.start(index).parse().unwrap_or(f64::NAN),
        }
    }

    /// The place of the variable of `kind`; 0 (the time) where there is none.
    fn find(&self, kind: &Kind) -> usize {
        self.model.variables.iter().position(|v| &v.kind == kind).unwrap_or(0)
    }

    /// The simulation for the parameters as set, built when it is not there yet.
    fn ensure_sim(&mut self) -> Result<(), Fail> {
        if self.sim.is_none() {
            self.sim = Some(self.build_sim(&self.set)?);
            self.take_sample();
        }
        Ok(())
    }

    /// The simulation for the parameters in `set`: the config of the FMU or of `config_file`, with the effect switches
    /// that were set on top. Refused when it would give other variables than the model description has.
    fn build_sim(&self, set: &[Option<Value>]) -> Result<Sim, Fail> {
        let find = |kind: Kind| self.find(&kind);
        let machine = self.text_in(set, find(Kind::Machine));
        // The FMU's own machine, which may come from a machine file; another one by its id among the registered.
        let entry = if machine == self.model.entry.id {
            self.model.entry
        } else {
            sim_machines::find(&machine).ok_or(("fmi-error-config", machine.clone()))?
        };
        let file = self.text_in(set, find(Kind::ConfigFile));
        let text = if file.is_empty() {
            self.model.config_text.clone()
        } else {
            std::fs::read_to_string(&file).map_err(|e| ("fmi-error-config", format!("{file}: {e}")))?
        };
        let refused = |e: sim_core::ConfigError| ("fmi-error-config", self.tr.tr_arg(e.key(), e.detail()));
        let mut cfg = load_config(entry, &text).map_err(refused)?;
        for (index, v) in self.model.variables.iter().enumerate() {
            if let (Kind::Effect(name), Some(Value::Bool(on))) = (&v.kind, &set[index]) {
                cfg.set_text(&format!("effects.{name} = {on}")).map_err(refused)?;
            }
        }
        let sim = Sim::new(entry, &cfg).map_err(refused)?;
        let signals = sim.signal_infos();
        let expected: Vec<(&str, &str)> = self
            .model
            .variables
            .iter()
            .filter(|v| matches!(v.kind, Kind::Signal(_)))
            .map(|v| (v.name.as_str(), v.unit.as_str()))
            .collect();
        let found: Vec<(&str, &str)> = signals.iter().map(|s| (s.id.as_str(), s.unit.as_str())).collect();
        let setpoints = self.model.variables.iter().filter(|v| matches!(v.kind, Kind::Setpoint(_))).count();
        if expected != found || setpoints != sim.machine.axes().len() {
            let differing =
                expected.iter().zip(&found).find(|(a, b)| a != b).map_or(String::new(), |(a, _)| a.0.into());
            return Err(("fmi-error-variables", differing));
        }
        Ok(sim)
    }

    fn take_sample(&mut self) {
        if let Some(sim) = &self.sim {
            self.sample = sample_of(sim, self.start_time);
        }
    }

    pub(crate) fn get_real(&mut self, vr: u32) -> Result<f64, Fail> {
        // FMI allows getting values from initialisation mode on: before it the start time is not yet set up.
        self.in_mode(&[Mode::Initialising, Mode::Stepping, Mode::Terminated], "Get")?;
        let index = self.variable(vr, "Real")?;
        match self.model.variables[index].kind {
            Kind::Signal(i) => {
                self.ensure_sim()?;
                Ok(self.sample[i])
            }
            Kind::Setpoint(i) => Ok(match (&self.set[index], &self.sim) {
                (Some(Value::Real(v)), _) => *v,
                (_, Some(sim)) => sim.control.external_setpoint()[i],
                _ => self.start(index).parse().unwrap_or(f64::NAN),
            }),
            Kind::Target(_) => Ok(self.real_value(index)),
            _ => unreachable!("only signals, setpoints and targets are Real"),
        }
    }

    pub(crate) fn set_value(&mut self, vr: u32, value: Value) -> Result<(), Fail> {
        let type_name = match value {
            Value::Real(_) => "Real",
            Value::Bool(_) => "Boolean",
            Value::Text(_) => "String",
        };
        let index = self.variable(vr, type_name)?;
        let v = &self.model.variables[index];
        match v.causality() {
            "input" => self.in_mode(&[Mode::Instantiated, Mode::Initialising, Mode::Stepping], "Set")?,
            "parameter" => {
                self.in_mode(&[Mode::Instantiated, Mode::Initialising], "Set")?;
                self.sim = None;
            }
            _ => return Err(("fmi-error-state", format!("Set {}", v.name))),
        }
        self.set[index] = Some(value);
        Ok(())
    }

    /// `fmi2SetupExperiment`, and the experiment part of `fmi3EnterInitializationMode`.
    pub(crate) fn setup(
        &mut self,
        tolerance: Option<f64>,
        start_time: f64,
        stop_time: Option<f64>,
    ) -> Result<(), Fail> {
        self.in_mode(&[Mode::Instantiated], "SetupExperiment")?;
        // A start time that is not a number would pass every later check of the communication point.
        if !start_time.is_finite()
            || stop_time.is_some_and(|t| !t.is_finite())
            || tolerance.is_some_and(|t| !t.is_finite())
        {
            return Err(("fmi-error-experiment", format!("{start_time:?}, {stop_time:?}, {tolerance:?}")));
        }
        self.start_time = start_time;
        Ok(())
    }

    pub(crate) fn enter_initialisation(&mut self) -> Result<fmi2Status, Fail> {
        self.in_mode(&[Mode::Instantiated], "EnterInitializationMode")?;
        self.mode = Mode::Initialising;
        Ok(fmi2OK)
    }

    pub(crate) fn exit_initialisation(&mut self) -> Result<fmi2Status, Fail> {
        self.in_mode(&[Mode::Initialising], "ExitInitializationMode")?;
        self.ensure_sim()?;
        // A fresh sample: the outputs at the start time as it is now.
        self.take_sample();
        self.mode = Mode::Stepping;
        Ok(fmi2OK)
    }

    pub(crate) fn terminate(&mut self) -> Result<fmi2Status, Fail> {
        self.in_mode(&[Mode::Stepping], "Terminate")?;
        self.mode = Mode::Terminated;
        Ok(fmi2OK)
    }

    pub(crate) fn reset(&mut self) -> Result<fmi2Status, Fail> {
        (self.mode, self.start_time, self.sim) = (Mode::Instantiated, 0.0, None);
        self.set.iter_mut().for_each(|v| *v = None);
        self.latched = [false; 3];
        Ok(fmi2OK)
    }

    /// The time the simulation stands at, as the importer counts it.
    pub(crate) fn now(&self) -> f64 {
        self.start_time + self.sim.as_ref().map_or(0.0, |s| s.time())
    }

    pub(crate) fn do_step(&mut self, point: f64, size: f64) -> Result<fmi2Status, Fail> {
        self.in_mode(&[Mode::Stepping], "DoStep")?;
        // An infinite step would loop for ever, and a time that is not a number would pass every comparison.
        if !(point.is_finite() && size.is_finite()) {
            return Err(("fmi-error-step", format!("{point:?}, {size:?}")));
        }
        let now = self.now();
        let sim = self.sim.as_ref().ok_or(("fmi-error-state", "DoStep".to_string()))?;
        let steps = (size / sim.dt).round();
        // More base steps than a u32 holds would run for days: refused as an importer's mistake.
        if !(1.0..=u32::MAX as f64).contains(&steps) || (steps * sim.dt - size).abs() > 1e-9 * size.max(sim.dt) {
            return Err(("fmi-error-step", format!("{size:?} / {:?}", sim.dt)));
        }
        if !((point - now).abs() <= 0.5 * sim.dt) {
            return Err(("fmi-error-time", format!("{point:?} / {now:?}")));
        }
        let mut applied = Applied {
            steps: steps as u32,
            enable: !matches!(self.set[self.find(&Kind::Enable)], Some(Value::Bool(false))),
            commands: Vec::new(),
            setpoints: vec![None; sim.machine.axes().len()],
        };
        for (index, v) in self.model.variables.iter().enumerate() {
            if let (Kind::Setpoint(i), Some(Value::Real(value))) = (&v.kind, &self.set[index]) {
                applied.setpoints[*i] = Some(*value);
            }
        }
        // The triggers that changed since the last communication point.
        let on = [Kind::Cycle, Kind::Move, Kind::RunPath].map(|kind| self.bool_value(self.find(&kind)));
        let mut warnings = Vec::new();
        if on[0] != self.latched[0] {
            applied.commands.push(if on[0] { Command::StartCycle } else { Command::Stop });
        }
        if on[1] && !self.latched[1] {
            let targets = (0..applied.setpoints.len()).map(|i| self.real_value(self.find(&Kind::Target(i))));
            applied.commands.push(Command::MoveTo(targets.collect()));
        }
        if on[2] && !self.latched[2] {
            match self.planned_path(sim) {
                Ok(points) => applied.commands.push(Command::Trajectory(points)),
                Err((key, arg)) => warnings.push(self.tr.tr_arg(key, &arg)),
            }
        }
        for command in &applied.commands {
            let what = match command {
                Command::MoveTo(target) => format!("MoveTo {target:?}"),
                Command::Trajectory(points) => {
                    format!("Trajectory {} {:.4} s", points.len(), points.last().map_or(0.0, |p| p.0))
                }
                other => format!("{other:?}"),
            };
            self.note(EVENTS, fmi2OK, &self.tr.tr_arg("fmi-log-command", &format!("{now:?}: {what}")));
        }
        self.latched = on;
        let refused = apply(self.sim.as_mut().expect("checked above"), &applied);
        warnings.extend(refused.iter().map(|key| self.tr.tr_arg("fmi-refused", self.tr.tr(key))));
        for warning in &warnings {
            self.note(WARNINGS, fmi2Warning, warning);
        }
        self.take_sample();
        Ok(if warnings.is_empty() { fmi2OK } else { fmi2Warning })
    }

    /// The poses of `command.trajectory_file` where it is not empty, else the path or program of `command.path_file`
    /// (where it is not empty) or of `command.path`, planned from where the machine stands as `sim-cli run --path` plans
    /// it: within the axis limits, or with `time_optimal` as fast as the motors allow.
    fn planned_path(&self, sim: &Sim) -> Result<Vec<(f64, Vec<f64>)>, Fail> {
        let read = |file: &str| std::fs::read_to_string(file).map_err(|e| ("fmi-error-path", format!("{file}: {e}")));
        let trajectory = self.text_value(self.find(&Kind::TrajectoryFile));
        if !trajectory.is_empty() {
            return sim_components::path::parse_trajectory(&read(&trajectory)?)
                .map_err(|line| ("fmi-error-path", format!("{trajectory}: {line}")));
        }
        let file = self.text_value(self.find(&Kind::PathFile));
        let text = if file.is_empty() { self.text_value(self.find(&Kind::PathText)) } else { read(&file)? };
        let time_optimal = self.bool_value(self.find(&Kind::TimeOptimal));
        let runs = sim_components::program::read(&file, &text, sim.machine.axes(), sim.control.planned_pos())
            .map_err(|e| ("fmi-error-path", format!("{file}:{}: {}", e.line, self.tr.tr_arg(e.key, &e.detail))))?;
        // A program with feeds is planned run by run; one run at the axes' limits as a path file is.
        match runs.as_slice() {
            [run] if run.feed.is_none() => sim.plan_path(&run.pieces, time_optimal),
            _ => sim.plan_program(&runs, time_optimal),
        }
        .map_err(|e| ("fmi-error-path", self.tr.tr(e.key).to_string()))
    }

    pub(crate) fn get_state(&self) -> FmuState {
        let stepping = matches!(self.mode, Mode::Stepping | Mode::Terminated);
        FmuState {
            guid: self.model.guid.clone(),
            mode: self.mode,
            start_time: self.start_time,
            set: self.set.clone(),
            latched: self.latched,
            snapshot: self.sim.as_ref().filter(|_| stepping).map(|sim| (sim.snapshot(), self.sample.clone())),
        }
    }

    /// Sets a state: in step mode the simulation is built from the state's parameters and its snapshot restored into
    /// it. Everything is built before anything is changed, so a state that cannot be set leaves the instance as it was.
    pub(crate) fn set_state(&mut self, state: &FmuState) -> Result<fmi2Status, Fail> {
        let refused = || ("fmi-error-fmu-state", state.guid.clone());
        if state.guid != self.model.guid || state.set.len() != self.set.len() {
            return Err(refused());
        }
        let (sim, sample) = if matches!(state.mode, Mode::Stepping | Mode::Terminated) {
            let (snapshot, sample) = state.snapshot.as_ref().ok_or_else(refused)?;
            let mut sim = self.build_sim(&state.set)?;
            sim.restore(snapshot).map_err(|_| refused())?;
            (Some(sim), sample.clone())
        } else {
            (None, Vec::new())
        };
        (self.mode, self.start_time, self.set, self.sim) = (state.mode, state.start_time, state.set.clone(), sim);
        (self.sample, self.latched) = (sample, state.latched);
        self.note(EVENTS, fmi2OK, &self.tr.tr_arg("fmi-log-state", &format!("{:?} {:?}", self.mode, self.now())));
        Ok(fmi2OK)
    }

    /// Checks a directional or adjoint derivative's references: unknowns are outputs, knowns inputs, both Real.
    /// The derivative itself is zero (see the module's documentation).
    pub(crate) fn check_derivative(&self, unknowns: &[u32], knowns: &[u32]) -> Result<(), Fail> {
        self.in_mode(&[Mode::Initialising, Mode::Stepping], "GetDirectionalDerivative")?;
        let is = |vr: &u32, causality: &str| {
            self.model
                .variables
                .get(*vr as usize)
                .is_some_and(|v| v.type_name() == "Real" && v.causality() == causality)
        };
        if let Some(vr) =
            unknowns.iter().find(|vr| !is(vr, "output")).or_else(|| knowns.iter().find(|vr| !is(vr, "input")))
        {
            return Err(("fmi-error-reference", format!("Real {vr}")));
        }
        Ok(())
    }
}

/// The serialized state's format; a state of another one (format 2 replayed its steps) is refused by name.
const STATE_MAGIC: &[u8] = b"sim-fmi state 3\0";

impl FmuState {
    /// The state as bytes: the format, the FMU's version, then little-endian numbers, lengths as u64, texts as UTF-8,
    /// and the snapshot as `postcard` writes it (its layout is the version's, hence the version in front).
    pub(crate) fn to_bytes(&self) -> Vec<u8> {
        let mut out = STATE_MAGIC.to_vec();
        let text = |out: &mut Vec<u8>, t: &str| {
            out.extend((t.len() as u64).to_le_bytes());
            out.extend(t.as_bytes());
        };
        text(&mut out, env!("CARGO_PKG_VERSION"));
        text(&mut out, &self.guid);
        out.push(self.mode as u8);
        out.extend(self.start_time.to_le_bytes());
        out.extend(self.latched.map(u8::from));
        out.extend((self.set.len() as u64).to_le_bytes());
        for value in &self.set {
            match value {
                None => out.push(0),
                Some(Value::Real(v)) => {
                    out.push(1);
                    out.extend(v.to_le_bytes());
                }
                Some(Value::Bool(b)) => out.extend([2, *b as u8]),
                Some(Value::Text(t)) => {
                    out.push(3);
                    text(&mut out, t);
                }
            }
        }
        match &self.snapshot {
            None => out.push(0),
            Some((snapshot, sample)) => {
                out.push(1);
                out.extend((sample.len() as u64).to_le_bytes());
                sample.iter().for_each(|x| out.extend(x.to_le_bytes()));
                // Numbers, vectors and enums written to a vector: nothing that can fail.
                let bytes = postcard::to_allocvec(snapshot).expect("a snapshot is plain data");
                out.extend((bytes.len() as u64).to_le_bytes());
                out.extend(bytes);
            }
        }
        out
    }

    /// The state from `to_bytes`. A state of another format or version is refused as such
    /// (`fmi-error-fmu-state-version`), anything else that does not read back whole (cut off, extended) as damaged.
    pub(crate) fn from_bytes(bytes: &[u8]) -> Result<Self, Fail> {
        let damaged = || ("fmi-error-fmu-state", format!("{} bytes", bytes.len()));
        let Some(rest) = bytes.strip_prefix(STATE_MAGIC) else {
            if !bytes.starts_with(b"sim-fmi state ") {
                return Err(damaged());
            }
            let format = bytes.iter().take(STATE_MAGIC.len()).take_while(|b| **b != 0).map(|b| *b as char);
            return Err(("fmi-error-fmu-state-version", format.collect()));
        };
        let mut read = Reader(rest);
        let version = read.text().ok_or_else(damaged)?;
        if version != env!("CARGO_PKG_VERSION") {
            return Err(("fmi-error-fmu-state-version", version));
        }
        Self::read(&mut read).filter(|_| read.0.is_empty()).ok_or_else(damaged)
    }

    fn read(read: &mut Reader) -> Option<Self> {
        let guid = read.text()?;
        let mode = match read.u8()? {
            0 => Mode::Instantiated,
            1 => Mode::Initialising,
            2 => Mode::Stepping,
            3 => Mode::Terminated,
            _ => return None,
        };
        let start_time = read.f64()?;
        let latched = [read.u8()? != 0, read.u8()? != 0, read.u8()? != 0];
        let mut set = Vec::new();
        for _ in 0..read.u64()? {
            set.push(match read.u8()? {
                0 => None,
                1 => Some(Value::Real(read.f64()?)),
                2 => Some(Value::Bool(read.u8()? != 0)),
                3 => Some(Value::Text(read.text()?)),
                _ => return None,
            });
        }
        let snapshot = match read.u8()? {
            0 => None,
            1 => {
                let sample = read.numbers()?;
                let n = usize::try_from(read.u64()?).ok()?;
                Some((postcard::from_bytes(read.take(n)?).ok()?, sample))
            }
            _ => return None,
        };
        Some(Self { guid, mode, start_time, set, latched, snapshot })
    }
}

/// Reads from the front of a byte slice.
struct Reader<'a>(&'a [u8]);

impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> Option<&'a [u8]> {
        let (head, tail) = (self.0.get(..n)?, self.0.get(n..)?);
        self.0 = tail;
        Some(head)
    }
    fn u8(&mut self) -> Option<u8> {
        Some(self.take(1)?[0])
    }
    fn u64(&mut self) -> Option<u64> {
        Some(u64::from_le_bytes(self.take(8)?.try_into().ok()?))
    }
    fn f64(&mut self) -> Option<f64> {
        Some(f64::from_bits(self.u64()?))
    }
    /// A length and as many numbers; a length the bytes cannot hold is refused before anything is allocated.
    fn numbers(&mut self) -> Option<Vec<f64>> {
        let n = usize::try_from(self.u64()?).ok()?;
        let bytes = self.take(n.checked_mul(8)?)?;
        Some(bytes.as_chunks::<8>().0.iter().map(|c| f64::from_le_bytes(*c)).collect())
    }
    fn text(&mut self) -> Option<String> {
        let n = usize::try_from(self.u64()?).ok()?;
        String::from_utf8(self.take(n)?.to_vec()).ok()
    }
}

/// Runs `f` on the instance; a refusal is logged and returned as an error (3 in both versions), a panic as fatal (4).
///
/// # Safety
/// `c` is null or an instance from `fmi2Instantiate` or `fmi3InstantiateCoSimulation` that has not been freed.
pub(crate) unsafe fn with(c: *mut c_void, f: impl FnOnce(&mut Instance) -> Result<fmi2Status, Fail>) -> fmi2Status {
    let Some(instance) = (unsafe { (c as *mut Instance).as_mut() }) else { return fmi2Error };
    match catch_unwind(AssertUnwindSafe(|| f(&mut *instance))) {
        Ok(Ok(status)) => status,
        Ok(Err(fail)) => instance.fail(fmi2Error, fail),
        Err(_) => fmi2Fatal,
    }
}

/// A new instance of the FMU whose resources are in `resources`, for FMI version `fmi`; null, after a logged
/// message, when the type is not co-simulation, the resources cannot be read or `token` (GUID or instantiation token)
/// is not this FMU's.
pub(crate) fn instantiate(
    name: &str,
    logger: Logger,
    environment: *mut c_void,
    resources: &Path,
    token: &str,
    fmi: u8,
    co_simulation: bool,
    logging_on: bool,
) -> *mut c_void {
    let name = c_text(name);
    let made = catch_unwind(AssertUnwindSafe(|| -> Result<Instance, (Translations, Fail)> {
        // Until the FMU's language is known, its messages are in English.
        let mut tr = Translations::default();
        sim_core::i18n::I18N.iter().chain(crate::I18N).filter(|(l, _)| *l == "en").for_each(|(_, t)| tr.add(t));
        if !co_simulation {
            return Err((tr, ("fmi-error-type", String::new())));
        }
        let read = |file: &str| {
            std::fs::read_to_string(resources.join(file))
                .map_err(|e| ("fmi-error-resources", format!("{}: {e}", resources.join(file).display())))
        };
        let (about, config_text) = match (read("model.txt"), read("config.toml")) {
            (Ok(a), Ok(c)) => (a, c),
            (Err(e), _) | (_, Err(e)) => return Err((tr, e)),
        };
        let mut lines = about.lines();
        let (id, lang) = (lines.next().unwrap_or("").trim(), lines.next().unwrap_or("en").trim());
        let Ok(entry) = crate::machine_of(resources, id) else { return Err((tr, ("fmi-error-resources", id.into()))) };
        let tr = texts(lang, entry);
        let model = match Model::with_fmi(entry, &config_text, lang, fmi) {
            Ok(model) => model,
            Err(e) => {
                let detail = tr.tr_arg(e.key(), e.detail());
                return Err((tr, ("fmi-error-config", detail)));
            }
        };
        if model.guid != token {
            return Err((tr, ("fmi-error-guid", token.into())));
        }
        let set = vec![None; model.variables.len()];
        Ok(Instance {
            name: name.clone(),
            logger,
            environment,
            tr,
            model,
            set,
            mode: Mode::Instantiated,
            start_time: 0.0,
            sim: None,
            sample: Vec::new(),
            strings: Vec::new(),
            latched: [false; 3],
            logging: [logging_on, true, true],
        })
    }));
    match made {
        Ok(Ok(instance)) => Box::into_raw(Box::new(instance)) as *mut c_void,
        Ok(Err((tr, (key, arg)))) => {
            log(logger, environment, &name, fmi2Error, ERRORS, &tr.tr_arg(key, &arg));
            std::ptr::null_mut()
        }
        Err(_) => std::ptr::null_mut(),
    }
}

/// The strings of `vr` handed out through `value`, valid until the next call on the instance.
///
/// # Safety
/// See `with`; `vr` and `value` point to `n` values.
pub(crate) unsafe fn get_strings(c: *mut c_void, vr: *const u32, n: usize, value: *mut *const c_char) -> fmi2Status {
    unsafe {
        with(c, |i| {
            i.strings.clear();
            for r in slice(vr, n)? {
                let index = i.variable(*r, "String")?;
                i.strings.push(c_text(&i.text_value(index)));
            }
            for (s, v) in i.strings.iter().zip(slice_mut(value, n)?) {
                *v = s.as_ptr();
            }
            Ok(fmi2OK)
        })
    }
}

// The FMU state functions of both versions. A state is a `Box<FmuState>` behind the importer's pointer.

/// # Safety
/// See `with`; `slot` is null or points to null or to a state from this library.
pub(crate) unsafe fn state_get(c: *mut c_void, slot: *mut *mut c_void) -> fmi2Status {
    unsafe {
        with(c, |i| {
            let slot = slot.as_mut().ok_or(("fmi-error-fmu-state", "null".to_string()))?;
            let state = i.get_state();
            // A state given back is overwritten, as the standard has it.
            match (*slot as *mut FmuState).as_mut() {
                Some(old) => *old = state,
                None => *slot = Box::into_raw(Box::new(state)) as *mut c_void,
            }
            Ok(fmi2OK)
        })
    }
}

/// # Safety
/// See `state_get`; `state` is null or a state from this library.
pub(crate) unsafe fn state_set(c: *mut c_void, state: *mut c_void) -> fmi2Status {
    unsafe {
        with(c, |i| {
            i.set_state((state as *const FmuState).as_ref().ok_or(("fmi-error-fmu-state", "null".to_string()))?)
        })
    }
}

/// # Safety
/// See `state_get`; the state is not used afterwards.
pub(crate) unsafe fn state_free(c: *mut c_void, slot: *mut *mut c_void) -> fmi2Status {
    unsafe {
        with(c, |_| {
            if let Some(slot) = slot.as_mut() {
                if !slot.is_null() {
                    drop(Box::from_raw(*slot as *mut FmuState));
                }
                *slot = std::ptr::null_mut();
            }
            Ok(fmi2OK)
        })
    }
}

/// # Safety
/// See `state_set`; `size` is writable.
pub(crate) unsafe fn state_size(c: *mut c_void, state: *mut c_void, size: *mut usize) -> fmi2Status {
    unsafe {
        with(c, |_| {
            let state = (state as *const FmuState).as_ref().ok_or(("fmi-error-fmu-state", "null".to_string()))?;
            *size.as_mut().ok_or(("fmi-error-fmu-state", "size".to_string()))? = state.to_bytes().len();
            Ok(fmi2OK)
        })
    }
}

/// # Safety
/// See `state_set`; `bytes` points to `size` writable bytes.
pub(crate) unsafe fn state_serialize(c: *mut c_void, state: *mut c_void, bytes: *mut u8, size: usize) -> fmi2Status {
    unsafe {
        with(c, |_| {
            let state = (state as *const FmuState).as_ref().ok_or(("fmi-error-fmu-state", "null".to_string()))?;
            let data = state.to_bytes();
            let out = slice_mut(bytes, size)?;
            if out.len() != data.len() {
                return Err(("fmi-error-fmu-state", format!("{size} / {}", data.len())));
            }
            out.copy_from_slice(&data);
            Ok(fmi2OK)
        })
    }
}

/// # Safety
/// See `state_get`; `bytes` points to `size` bytes.
pub(crate) unsafe fn state_deserialize(
    c: *mut c_void,
    bytes: *const u8,
    size: usize,
    slot: *mut *mut c_void,
) -> fmi2Status {
    unsafe {
        with(c, |i| {
            let state = FmuState::from_bytes(slice(bytes, size)?)?;
            if state.guid != i.model.guid || state.set.len() != i.model.variables.len() {
                return Err(("fmi-error-fmu-state", state.guid));
            }
            let slot = slot.as_mut().ok_or(("fmi-error-fmu-state", "null".to_string()))?;
            match (*slot as *mut FmuState).as_mut() {
                Some(old) => *old = state,
                None => *slot = Box::into_raw(Box::new(state)) as *mut c_void,
            }
            Ok(fmi2OK)
        })
    }
}

/// A directional or adjoint derivative: the references checked, `n` zeros written to `out`.
///
/// # Safety
/// See `with`; the pointers point to as many values as their counts say.
pub(crate) unsafe fn zero_derivative(
    c: *mut c_void,
    unknowns: *const u32,
    n_unknowns: usize,
    knowns: *const u32,
    n_knowns: usize,
    out: *mut f64,
    n: usize,
) -> fmi2Status {
    unsafe {
        with(c, |i| {
            i.check_derivative(slice(unknowns, n_unknowns)?, slice(knowns, n_knowns)?)?;
            slice_mut(out, n)?.fill(0.0);
            Ok(fmi2OK)
        })
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi2GetTypesPlatform() -> *const c_char {
    c"default".as_ptr()
}

#[unsafe(no_mangle)]
pub extern "C" fn fmi2GetVersion() -> *const c_char {
    c"2.0".as_ptr()
}

/// # Safety
/// As the FMI standard demands of the caller: valid strings, `functions` null or valid for the instance's life.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2Instantiate(
    instanceName: fmi2String,
    fmuType: c_int,
    fmuGUID: fmi2String,
    fmuResourceLocation: fmi2String,
    functions: *const fmi2CallbackFunctions,
    _visible: fmi2Boolean,
    loggingOn: fmi2Boolean,
) -> fmi2Component {
    let callbacks = unsafe { functions.as_ref() };
    let (logger, environment) =
        (callbacks.and_then(|f| f.logger), callbacks.map_or(std::ptr::null_mut(), |f| f.componentEnvironment));
    let resources = resource_dir(unsafe { text(fmuResourceLocation) });
    instantiate(
        unsafe { text(instanceName) },
        Logger::Fmi2(logger),
        environment,
        &resources,
        unsafe { text(fmuGUID) },
        2,
        fmuType == fmi2CoSimulation,
        loggingOn != 0,
    )
}

/// `fmi2SetDebugLogging` and `fmi3SetDebugLogging` alike.
///
/// # Safety
/// See `with`; `categories` points to `n` valid strings.
pub(crate) unsafe fn debug_logging(c: *mut c_void, on: bool, n: usize, categories: *const *const c_char) -> fmi2Status {
    unsafe {
        with(c, |i| {
            let names: Vec<&str> = slice(categories, n)?.iter().map(|p| text(*p)).collect();
            i.set_debug_logging(on, &names)
        })
    }
}

/// # Safety
/// `c` is null or an instance from `fmi2Instantiate`; it is not used afterwards.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2FreeInstance(c: fmi2Component) {
    if !c.is_null() {
        drop(unsafe { Box::from_raw(c as *mut Instance) });
    }
}

/// Switches the log categories named in `categories` (all of them where `nCategories` is 0, or with `logAll`).
///
/// # Safety
/// See `with`; `categories` points to `nCategories` valid strings.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SetDebugLogging(
    c: fmi2Component,
    loggingOn: fmi2Boolean,
    nCategories: usize,
    categories: *const fmi2String,
) -> fmi2Status {
    unsafe { debug_logging(c, loggingOn != 0, nCategories, categories) }
}

/// # Safety
/// See `with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SetupExperiment(
    c: fmi2Component,
    toleranceDefined: fmi2Boolean,
    tolerance: fmi2Real,
    startTime: fmi2Real,
    stopTimeDefined: fmi2Boolean,
    stopTime: fmi2Real,
) -> fmi2Status {
    unsafe {
        with(c, |i| {
            i.setup((toleranceDefined != 0).then_some(tolerance), startTime, (stopTimeDefined != 0).then_some(stopTime))
                .map(|_| fmi2OK)
        })
    }
}

/// # Safety
/// See `with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2EnterInitializationMode(c: fmi2Component) -> fmi2Status {
    unsafe { with(c, Instance::enter_initialisation) }
}

/// # Safety
/// See `with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2ExitInitializationMode(c: fmi2Component) -> fmi2Status {
    unsafe { with(c, Instance::exit_initialisation) }
}

/// # Safety
/// See `with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2Terminate(c: fmi2Component) -> fmi2Status {
    unsafe { with(c, Instance::terminate) }
}

/// # Safety
/// See `with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2Reset(c: fmi2Component) -> fmi2Status {
    unsafe { with(c, Instance::reset) }
}

/// # Safety
/// See `with`; `vr` and `value` point to `nvr` values.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2GetReal(
    c: fmi2Component,
    vr: *const fmi2ValueReference,
    nvr: usize,
    value: *mut fmi2Real,
) -> fmi2Status {
    unsafe {
        with(c, |i| {
            for (r, v) in slice(vr, nvr)?.iter().zip(slice_mut(value, nvr)?) {
                *v = i.get_real(*r)?;
            }
            Ok(fmi2OK)
        })
    }
}

/// # Safety
/// See `fmi2GetReal`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SetReal(
    c: fmi2Component,
    vr: *const fmi2ValueReference,
    nvr: usize,
    value: *const fmi2Real,
) -> fmi2Status {
    unsafe {
        with(c, |i| {
            for (r, v) in slice(vr, nvr)?.iter().zip(slice(value, nvr)?) {
                i.set_value(*r, Value::Real(*v))?;
            }
            Ok(fmi2OK)
        })
    }
}

/// The FMU has no Integer variables: any value reference is refused.
///
/// # Safety
/// See `fmi2GetReal`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2GetInteger(
    c: fmi2Component,
    vr: *const fmi2ValueReference,
    nvr: usize,
    _value: *mut fmi2Integer,
) -> fmi2Status {
    unsafe { with(c, |i| slice(vr, nvr)?.first().map_or(Ok(fmi2OK), |r| i.variable(*r, "Integer").map(|_| fmi2OK))) }
}

/// # Safety
/// See `fmi2GetReal`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SetInteger(
    c: fmi2Component,
    vr: *const fmi2ValueReference,
    nvr: usize,
    _value: *const fmi2Integer,
) -> fmi2Status {
    unsafe { with(c, |i| slice(vr, nvr)?.first().map_or(Ok(fmi2OK), |r| i.variable(*r, "Integer").map(|_| fmi2OK))) }
}

/// # Safety
/// See `fmi2GetReal`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2GetBoolean(
    c: fmi2Component,
    vr: *const fmi2ValueReference,
    nvr: usize,
    value: *mut fmi2Boolean,
) -> fmi2Status {
    unsafe {
        with(c, |i| {
            for (r, v) in slice(vr, nvr)?.iter().zip(slice_mut(value, nvr)?) {
                *v = i.bool_value(i.variable(*r, "Boolean")?) as fmi2Boolean;
            }
            Ok(fmi2OK)
        })
    }
}

/// # Safety
/// See `fmi2GetReal`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SetBoolean(
    c: fmi2Component,
    vr: *const fmi2ValueReference,
    nvr: usize,
    value: *const fmi2Boolean,
) -> fmi2Status {
    unsafe {
        with(c, |i| {
            for (r, v) in slice(vr, nvr)?.iter().zip(slice(value, nvr)?) {
                i.set_value(*r, Value::Bool(*v != 0))?;
            }
            Ok(fmi2OK)
        })
    }
}

/// The strings stay valid until the next call of this function on the instance.
///
/// # Safety
/// See `fmi2GetReal`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2GetString(
    c: fmi2Component,
    vr: *const fmi2ValueReference,
    nvr: usize,
    value: *mut fmi2String,
) -> fmi2Status {
    unsafe { get_strings(c, vr, nvr, value) }
}

/// # Safety
/// See `fmi2GetReal`; every string is valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SetString(
    c: fmi2Component,
    vr: *const fmi2ValueReference,
    nvr: usize,
    value: *const fmi2String,
) -> fmi2Status {
    unsafe {
        with(c, |i| {
            for (r, v) in slice(vr, nvr)?.iter().zip(slice(value, nvr)?) {
                i.set_value(*r, Value::Text(text(*v).into()))?;
            }
            Ok(fmi2OK)
        })
    }
}

/// # Safety
/// See `with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2DoStep(
    c: fmi2Component,
    currentCommunicationPoint: fmi2Real,
    communicationStepSize: fmi2Real,
    _noSetFMUStatePriorToCurrentPoint: fmi2Boolean,
) -> fmi2Status {
    unsafe { with(c, |i| i.do_step(currentCommunicationPoint, communicationStepSize)) }
}

/// A copy of the instance's state: what was set and the simulation's snapshot.
///
/// # Safety
/// See `with`; `state` points to null or to a state from this library, which is then overwritten.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2GetFMUstate(c: fmi2Component, state: *mut fmi2FMUstate) -> fmi2Status {
    unsafe { state_get(c, state) }
}

/// Restores a state: a fresh simulation from its parameters, its snapshot restored into it.
///
/// # Safety
/// See `with`; `state` is a state from this library.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SetFMUstate(c: fmi2Component, state: fmi2FMUstate) -> fmi2Status {
    unsafe { state_set(c, state) }
}

/// # Safety
/// See `fmi2GetFMUstate`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2FreeFMUstate(c: fmi2Component, state: *mut fmi2FMUstate) -> fmi2Status {
    unsafe { state_free(c, state) }
}

/// # Safety
/// See `fmi2SetFMUstate`; `size` is writable.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SerializedFMUstateSize(
    c: fmi2Component,
    state: fmi2FMUstate,
    size: *mut usize,
) -> fmi2Status {
    unsafe { state_size(c, state, size) }
}

/// # Safety
/// See `fmi2SetFMUstate`; `bytes` points to `size` bytes, the size `fmi2SerializedFMUstateSize` gave.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SerializeFMUstate(
    c: fmi2Component,
    state: fmi2FMUstate,
    bytes: *mut fmi2Byte,
    size: usize,
) -> fmi2Status {
    unsafe { state_serialize(c, state, bytes as *mut u8, size) }
}

/// # Safety
/// See `fmi2GetFMUstate`; `bytes` points to `size` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2DeSerializeFMUstate(
    c: fmi2Component,
    bytes: *const fmi2Byte,
    size: usize,
    state: *mut fmi2FMUstate,
) -> fmi2Status {
    unsafe { state_deserialize(c, bytes as *const u8, size, state) }
}

/// The derivative of outputs with respect to inputs: zero (see the module's documentation).
///
/// # Safety
/// See `with`; the pointers point to as many values as their counts say.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2GetDirectionalDerivative(
    c: fmi2Component,
    unknown: *const fmi2ValueReference,
    n_unknown: usize,
    known: *const fmi2ValueReference,
    n_known: usize,
    _dv_known: *const fmi2Real,
    dv_unknown: *mut fmi2Real,
) -> fmi2Status {
    unsafe { zero_derivative(c, unknown, n_unknown, known, n_known, dv_unknown, n_unknown) }
}

/// # Safety
/// See `with`.
unsafe fn unsupported(c: fmi2Component, call: &str) -> fmi2Status {
    unsafe { with(c, |_| Err(("fmi-error-unsupported", call.into()))) }
}

macro_rules! unsupported {
    ($($name:ident($($arg:ident: $ty:ty),*);)*) => {$(
        /// Not supported: returns `fmi2Error`.
        ///
        /// # Safety
        /// See `with`.
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn $name(c: fmi2Component, $($arg: $ty),*) -> fmi2Status {
            $(let _ = $arg;)*
            unsafe { unsupported(c, stringify!($name)) }
        }
    )*};
}

unsupported! {
    fmi2SetRealInputDerivatives(vr: *const fmi2ValueReference, nvr: usize, order: *const fmi2Integer, value: *const fmi2Real);
    fmi2GetRealOutputDerivatives(vr: *const fmi2ValueReference, nvr: usize, order: *const fmi2Integer, value: *mut fmi2Real);
    fmi2CancelStep();
    fmi2GetStatus(kind: c_int, value: *mut fmi2Status);
    fmi2GetRealStatus(kind: c_int, value: *mut fmi2Real);
    fmi2GetIntegerStatus(kind: c_int, value: *mut fmi2Integer);
    fmi2GetBooleanStatus(kind: c_int, value: *mut fmi2Boolean);
    fmi2GetStringStatus(kind: c_int, value: *mut fmi2String);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A state of format 2 (which replayed its steps) or of another version of the FMU is refused as such, by its
    /// format or version, not as damaged; a whole state of this version reads back, and one byte more is damage.
    #[test]
    fn a_state_of_another_format_or_version_is_refused_by_name() {
        let old = [b"sim-fmi state 2\0".as_slice(), &[0; 40]].concat();
        assert_eq!(FmuState::from_bytes(&old).err(), Some(("fmi-error-fmu-state-version", "sim-fmi state 2".into())));
        let mut other = STATE_MAGIC.to_vec();
        other.extend(5u64.to_le_bytes());
        other.extend(b"0.9.0");
        assert_eq!(FmuState::from_bytes(&other).err(), Some(("fmi-error-fmu-state-version", "0.9.0".into())));
        assert_eq!(FmuState::from_bytes(b"no state at all").err().map(|e| e.0), Some("fmi-error-fmu-state"));
        let set = vec![None, Some(Value::Real(1.5)), Some(Value::Bool(true)), Some(Value::Text("Bahn ä".into()))];
        let state = FmuState {
            guid: "{guid}".into(),
            mode: Mode::Initialising,
            start_time: 0.25,
            set: set.clone(),
            latched: [true, false, true],
            snapshot: None,
        };
        let bytes = state.to_bytes();
        let back = FmuState::from_bytes(&bytes).unwrap();
        assert_eq!(
            (back.guid.as_str(), back.mode, back.start_time, back.set, back.latched),
            ("{guid}", Mode::Initialising, 0.25, set, [true, false, true])
        );
        assert!(back.snapshot.is_none());
        let longer = [bytes.as_slice(), &[0]].concat();
        assert_eq!(FmuState::from_bytes(&longer).err().map(|e| e.0), Some("fmi-error-fmu-state"));
    }
}
