//! The FMI 2.0 Co-Simulation C functions, and the instance both FMI versions drive (`api3` has the FMI 3.0 ones).
//! The type definitions follow the FMI 2.0 headers (fmi2TypesPlatform.h, fmi2FunctionTypes.h; Copyright Modelica
//! Association Project "FMI", BSD-2-Clause licence), written out in Rust.
//!
//! Supported: instantiation, setup, initialisation, `fmi2DoStep`, get and set of Real, Integer (there are none),
//! Boolean and String variables, reset, terminate, FMU state get, set, free and (de)serialization, directional
//! derivatives. The rest (input derivatives, asynchronous steps, status queries) return `fmi2Error`; the model
//! description says so. A panic inside the simulation returns `fmi2Fatal` instead of taking the importing tool down.
//!
//! An FMU state is what the importer set plus the inputs each communication step applied; setting it builds a fresh
//! simulation and replays those steps, which gives bit for bit the simulation that was saved (tested).
//! The directional derivative of an output with respect to an input is exactly zero at a communication point: the
//! outputs are the signals at the end of the last step, and a new input acts on them only through the next step
//! (the model description says `dependencies=""`).

#![allow(non_snake_case, non_camel_case_types, non_upper_case_globals)]

use crate::api3::fmi3LogMessageCallback;
use crate::{Kind, Model, texts};
use sim_components::{Command, Sim, load_config};
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

pub type fmi2CallbackLogger = unsafe extern "C" fn(fmi2ComponentEnvironment, fmi2String, fmi2Status, fmi2String, fmi2String, ...);

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

/// What one communication step gave the simulation, to replay it.
#[derive(Debug, Clone, PartialEq)]
struct Applied {
    steps: u32,
    enable: bool,
    /// Per axis, the setpoint input if the importer has set it.
    setpoints: Vec<Option<f64>>,
}

/// An FMU state: what the importer set, and every communication step since initialisation.
#[derive(Debug, Clone, PartialEq)]
pub struct FmuState {
    guid: String,
    mode: Mode,
    start_time: f64,
    set: Vec<Option<Value>>,
    log: Vec<Applied>,
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
    /// Every communication step since initialisation, for the FMU state.
    log: Vec<Applied>,
}

pub(crate) fn c_text(text: &str) -> CString {
    CString::new(text.replace('\0', "")).unwrap_or_default()
}

fn log(logger: Logger, environment: *mut c_void, name: &CStr, status: fmi2Status, message: &str) {
    let category = if status == fmi2Warning { c"logStatusWarning" } else { c"logStatusError" };
    let message = c_text(message);
    match logger {
        // The message as an argument of "%s": a per cent sign in it is not a format.
        Logger::Fmi2(Some(logger)) => unsafe { logger(environment, name.as_ptr(), status, category.as_ptr(), c"%s".as_ptr(), message.as_ptr()) },
        Logger::Fmi3(Some(logger)) => unsafe { logger(environment, status, category.as_ptr(), message.as_ptr()) },
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

/// Gives the simulation one communication step's inputs and runs its base steps; the keys of the commands the
/// controller refused.
fn apply(sim: &mut Sim, applied: &Applied) -> Vec<&'static str> {
    let mut refused = Vec::new();
    if applied.enable != sim.control.enabled()
        && let Err(e) = sim.command(if applied.enable { Command::Enable } else { Command::Disable })
    {
        refused.push(e.key);
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
        log(self.logger, self.environment, &self.name, status, &self.tr.tr_arg(key, &arg));
        status
    }

    /// `call` without its prefix, which is the version's.
    pub(crate) fn in_mode(&self, modes: &[Mode], call: &str) -> Result<(), Fail> {
        let prefix = if matches!(self.logger, Logger::Fmi3(_)) { "fmi3" } else { "fmi2" };
        if modes.contains(&self.mode) { Ok(()) } else { Err(("fmi-error-state", format!("{prefix}{call} ({:?})", self.mode))) }
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
        match &self.set[index] {
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

    /// The simulation for the parameters as set: the config of the FMU or of `config_file`, with the effect switches
    /// that were set on top. Refused when it would give other variables than the model description has.
    fn ensure_sim(&mut self) -> Result<(), Fail> {
        if self.sim.is_some() {
            return Ok(());
        }
        let find = |kind: Kind| self.model.variables.iter().position(|v| v.kind == kind).unwrap_or(0);
        let machine = self.text_value(find(Kind::Machine));
        let entry = sim_machines::find(&machine).ok_or(("fmi-error-config", machine.clone()))?;
        let file = self.text_value(find(Kind::ConfigFile));
        let text = if file.is_empty() { self.model.config_text.clone() } else { std::fs::read_to_string(&file).map_err(|e| ("fmi-error-config", format!("{file}: {e}")))? };
        let refused = |e: sim_core::ConfigError| ("fmi-error-config", self.tr.tr_arg(e.key(), e.detail()));
        let mut cfg = load_config(entry, &text).map_err(refused)?;
        for (index, v) in self.model.variables.iter().enumerate() {
            if let (Kind::Effect(name), Some(Value::Bool(on))) = (&v.kind, &self.set[index]) {
                cfg.set_text(&format!("effects.{name} = {on}")).map_err(refused)?;
            }
        }
        let sim = Sim::new(entry, &cfg).map_err(refused)?;
        let signals = sim.signal_infos();
        let expected: Vec<(&str, &str)> = self.model.variables.iter().filter(|v| matches!(v.kind, Kind::Signal(_))).map(|v| (v.name.as_str(), v.unit.as_str())).collect();
        let found: Vec<(&str, &str)> = signals.iter().map(|s| (s.id.as_str(), s.unit.as_str())).collect();
        let setpoints = self.model.variables.iter().filter(|v| matches!(v.kind, Kind::Setpoint(_))).count();
        if expected != found || setpoints != sim.machine.axes().len() {
            let differing = expected.iter().zip(&found).find(|(a, b)| a != b).map_or(String::new(), |(a, _)| a.0.into());
            return Err(("fmi-error-variables", differing));
        }
        self.sim = Some(sim);
        self.take_sample();
        Ok(())
    }

    fn take_sample(&mut self) {
        if let Some(sim) = &self.sim {
            sim.sample(&mut self.sample);
            // The first signal is the time; the importer's clock starts at its start time.
            self.sample[0] += self.start_time;
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
            _ => unreachable!("only signals and setpoints are Real"),
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
    pub(crate) fn setup(&mut self, tolerance: Option<f64>, start_time: f64, stop_time: Option<f64>) -> Result<(), Fail> {
        self.in_mode(&[Mode::Instantiated], "SetupExperiment")?;
        // A start time that is not a number would pass every later check of the communication point.
        if !start_time.is_finite() || stop_time.is_some_and(|t| !t.is_finite()) || tolerance.is_some_and(|t| !t.is_finite()) {
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
        self.log.clear();
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
        let sim = self.sim.as_mut().ok_or(("fmi-error-state", "DoStep".to_string()))?;
        let steps = (size / sim.dt).round();
        // More base steps than a u32 holds would run for days: refused as an importer's mistake.
        if !(1.0..=u32::MAX as f64).contains(&steps) || (steps * sim.dt - size).abs() > 1e-9 * size.max(sim.dt) {
            return Err(("fmi-error-step", format!("{size:?} / {:?}", sim.dt)));
        }
        if !((point - now).abs() <= 0.5 * sim.dt) {
            return Err(("fmi-error-time", format!("{point:?} / {now:?}")));
        }
        let enable = self.model.variables.iter().position(|v| v.kind == Kind::Enable).unwrap_or(0);
        let mut applied = Applied { steps: steps as u32, enable: !matches!(self.set[enable], Some(Value::Bool(false))), setpoints: vec![None; sim.machine.axes().len()] };
        for (index, v) in self.model.variables.iter().enumerate() {
            if let (Kind::Setpoint(i), Some(Value::Real(value))) = (&v.kind, &self.set[index]) {
                applied.setpoints[*i] = Some(*value);
            }
        }
        let refused = apply(sim, &applied);
        self.log.push(applied);
        for key in &refused {
            log(self.logger, self.environment, &self.name, fmi2Warning, &self.tr.tr_arg("fmi-refused", self.tr.tr(key)));
        }
        self.take_sample();
        Ok(if refused.is_empty() { fmi2OK } else { fmi2Warning })
    }

    pub(crate) fn get_state(&self) -> FmuState {
        FmuState { guid: self.model.guid.clone(), mode: self.mode, start_time: self.start_time, set: self.set.clone(), log: self.log.clone() }
    }

    pub(crate) fn set_state(&mut self, state: &FmuState) -> Result<fmi2Status, Fail> {
        if state.guid != self.model.guid || state.set.len() != self.set.len() {
            return Err(("fmi-error-fmu-state", state.guid.clone()));
        }
        (self.mode, self.start_time, self.set, self.sim) = (state.mode, state.start_time, state.set.clone(), None);
        self.log.clear();
        if matches!(state.mode, Mode::Stepping | Mode::Terminated) {
            // shortcut: replays every step since initialisation from a fresh simulation, so restoring costs as long as
            // simulating up to the saved point and the log grows with each step; snapshot the simulation instead once
            // `Machine` and `Sim` can be cloned.
            self.ensure_sim()?;
            let sim = self.sim.as_mut().expect("just built");
            for applied in &state.log {
                apply(sim, applied);
            }
            self.log = state.log.clone();
            self.take_sample();
        }
        Ok(fmi2OK)
    }

    /// Checks a directional or adjoint derivative's references: unknowns are outputs, knowns inputs, both Real.
    /// The derivative itself is zero (see the module's documentation).
    pub(crate) fn check_derivative(&self, unknowns: &[u32], knowns: &[u32]) -> Result<(), Fail> {
        self.in_mode(&[Mode::Initialising, Mode::Stepping], "GetDirectionalDerivative")?;
        let is = |vr: &u32, causality: &str| self.model.variables.get(*vr as usize).is_some_and(|v| v.type_name() == "Real" && v.causality() == causality);
        if let Some(vr) = unknowns.iter().find(|vr| !is(vr, "output")).or_else(|| knowns.iter().find(|vr| !is(vr, "input"))) {
            return Err(("fmi-error-reference", format!("Real {vr}")));
        }
        Ok(())
    }
}

const STATE_MAGIC: &[u8] = b"sim-fmi state 1\0";

impl FmuState {
    /// The state as bytes: little-endian numbers, lengths as u64, texts as UTF-8.
    pub(crate) fn to_bytes(&self) -> Vec<u8> {
        let mut out = STATE_MAGIC.to_vec();
        let text = |out: &mut Vec<u8>, t: &str| {
            out.extend((t.len() as u64).to_le_bytes());
            out.extend(t.as_bytes());
        };
        text(&mut out, &self.guid);
        out.push(self.mode as u8);
        out.extend(self.start_time.to_le_bytes());
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
        out.extend((self.log.len() as u64).to_le_bytes());
        for applied in &self.log {
            out.extend(applied.steps.to_le_bytes());
            out.push(applied.enable as u8);
            out.extend((applied.setpoints.len() as u64).to_le_bytes());
            for s in &applied.setpoints {
                match s {
                    None => out.push(0),
                    Some(v) => {
                        out.push(1);
                        out.extend(v.to_le_bytes());
                    }
                }
            }
        }
        out
    }

    /// The state from `to_bytes`; none for anything else, a truncated or extended state among it.
    pub(crate) fn from_bytes(bytes: &[u8]) -> Option<Self> {
        let mut read = Reader(bytes.strip_prefix(STATE_MAGIC)?);
        let guid = read.text()?;
        let mode = match read.u8()? {
            0 => Mode::Instantiated,
            1 => Mode::Initialising,
            2 => Mode::Stepping,
            3 => Mode::Terminated,
            _ => return None,
        };
        let start_time = read.f64()?;
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
        let mut log = Vec::new();
        for _ in 0..read.u64()? {
            let (steps, enable) = (u32::from_le_bytes(read.take(4)?.try_into().ok()?), read.u8()? != 0);
            let mut setpoints = Vec::new();
            for _ in 0..read.u64()? {
                setpoints.push(match read.u8()? {
                    0 => None,
                    1 => Some(read.f64()?),
                    _ => return None,
                });
            }
            log.push(Applied { steps, enable, setpoints });
        }
        read.0.is_empty().then_some(Self { guid, mode, start_time, set, log })
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
pub(crate) fn instantiate(name: &str, logger: Logger, environment: *mut c_void, resources: &Path, token: &str, fmi: u8, co_simulation: bool) -> *mut c_void {
    let name = c_text(name);
    let made = catch_unwind(AssertUnwindSafe(|| -> Result<Instance, (Translations, Fail)> {
        // Until the FMU's language is known, its messages are in English.
        let mut tr = Translations::default();
        sim_core::i18n::I18N.iter().chain(crate::I18N).filter(|(l, _)| *l == "en").for_each(|(_, t)| tr.add(t));
        if !co_simulation {
            return Err((tr, ("fmi-error-type", String::new())));
        }
        let read = |file: &str| std::fs::read_to_string(resources.join(file)).map_err(|e| ("fmi-error-resources", format!("{}: {e}", resources.join(file).display())));
        let (about, config_text) = match (read("model.txt"), read("config.toml")) {
            (Ok(a), Ok(c)) => (a, c),
            (Err(e), _) | (_, Err(e)) => return Err((tr, e)),
        };
        let mut lines = about.lines();
        let (id, lang) = (lines.next().unwrap_or("").trim(), lines.next().unwrap_or("en").trim());
        let Some(entry) = sim_machines::find(id) else { return Err((tr, ("fmi-error-resources", id.into()))) };
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
        Ok(Instance { name: name.clone(), logger, environment, tr, model, set, mode: Mode::Instantiated, start_time: 0.0, sim: None, sample: Vec::new(), strings: Vec::new(), log: Vec::new() })
    }));
    match made {
        Ok(Ok(instance)) => Box::into_raw(Box::new(instance)) as *mut c_void,
        Ok(Err((tr, (key, arg)))) => {
            log(logger, environment, &name, fmi2Error, &tr.tr_arg(key, &arg));
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
    unsafe { with(c, |i| i.set_state((state as *const FmuState).as_ref().ok_or(("fmi-error-fmu-state", "null".to_string()))?)) }
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
pub(crate) unsafe fn state_deserialize(c: *mut c_void, bytes: *const u8, size: usize, slot: *mut *mut c_void) -> fmi2Status {
    unsafe {
        with(c, |i| {
            let state = FmuState::from_bytes(slice(bytes, size)?).filter(|s| s.guid == i.model.guid && s.set.len() == i.model.variables.len());
            let state = state.ok_or(("fmi-error-fmu-state", format!("{size} bytes")))?;
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
pub(crate) unsafe fn zero_derivative(c: *mut c_void, unknowns: *const u32, n_unknowns: usize, knowns: *const u32, n_knowns: usize, out: *mut f64, n: usize) -> fmi2Status {
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
    _loggingOn: fmi2Boolean,
) -> fmi2Component {
    let callbacks = unsafe { functions.as_ref() };
    let (logger, environment) = (callbacks.and_then(|f| f.logger), callbacks.map_or(std::ptr::null_mut(), |f| f.componentEnvironment));
    let resources = resource_dir(unsafe { text(fmuResourceLocation) });
    instantiate(unsafe { text(instanceName) }, Logger::Fmi2(logger), environment, &resources, unsafe { text(fmuGUID) }, 2, fmuType == fmi2CoSimulation)
}

/// # Safety
/// `c` is null or an instance from `fmi2Instantiate`; it is not used afterwards.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2FreeInstance(c: fmi2Component) {
    if !c.is_null() {
        drop(unsafe { Box::from_raw(c as *mut Instance) });
    }
}

/// # Safety
/// See `with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SetDebugLogging(c: fmi2Component, _loggingOn: fmi2Boolean, _nCategories: usize, _categories: *const fmi2String) -> fmi2Status {
    // Only warnings and errors are ever logged, and those always.
    unsafe { with(c, |_| Ok(fmi2OK)) }
}

/// # Safety
/// See `with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SetupExperiment(c: fmi2Component, toleranceDefined: fmi2Boolean, tolerance: fmi2Real, startTime: fmi2Real, stopTimeDefined: fmi2Boolean, stopTime: fmi2Real) -> fmi2Status {
    unsafe { with(c, |i| i.setup((toleranceDefined != 0).then_some(tolerance), startTime, (stopTimeDefined != 0).then_some(stopTime)).map(|_| fmi2OK)) }
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
pub unsafe extern "C" fn fmi2GetReal(c: fmi2Component, vr: *const fmi2ValueReference, nvr: usize, value: *mut fmi2Real) -> fmi2Status {
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
pub unsafe extern "C" fn fmi2SetReal(c: fmi2Component, vr: *const fmi2ValueReference, nvr: usize, value: *const fmi2Real) -> fmi2Status {
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
pub unsafe extern "C" fn fmi2GetInteger(c: fmi2Component, vr: *const fmi2ValueReference, nvr: usize, _value: *mut fmi2Integer) -> fmi2Status {
    unsafe { with(c, |i| slice(vr, nvr)?.first().map_or(Ok(fmi2OK), |r| i.variable(*r, "Integer").map(|_| fmi2OK))) }
}

/// # Safety
/// See `fmi2GetReal`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SetInteger(c: fmi2Component, vr: *const fmi2ValueReference, nvr: usize, _value: *const fmi2Integer) -> fmi2Status {
    unsafe { with(c, |i| slice(vr, nvr)?.first().map_or(Ok(fmi2OK), |r| i.variable(*r, "Integer").map(|_| fmi2OK))) }
}

/// # Safety
/// See `fmi2GetReal`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2GetBoolean(c: fmi2Component, vr: *const fmi2ValueReference, nvr: usize, value: *mut fmi2Boolean) -> fmi2Status {
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
pub unsafe extern "C" fn fmi2SetBoolean(c: fmi2Component, vr: *const fmi2ValueReference, nvr: usize, value: *const fmi2Boolean) -> fmi2Status {
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
pub unsafe extern "C" fn fmi2GetString(c: fmi2Component, vr: *const fmi2ValueReference, nvr: usize, value: *mut fmi2String) -> fmi2Status {
    unsafe { get_strings(c, vr, nvr, value) }
}

/// # Safety
/// See `fmi2GetReal`; every string is valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SetString(c: fmi2Component, vr: *const fmi2ValueReference, nvr: usize, value: *const fmi2String) -> fmi2Status {
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
pub unsafe extern "C" fn fmi2DoStep(c: fmi2Component, currentCommunicationPoint: fmi2Real, communicationStepSize: fmi2Real, _noSetFMUStatePriorToCurrentPoint: fmi2Boolean) -> fmi2Status {
    unsafe { with(c, |i| i.do_step(currentCommunicationPoint, communicationStepSize)) }
}

/// A copy of the instance's state: what was set and every step so far.
///
/// # Safety
/// See `with`; `state` points to null or to a state from this library, which is then overwritten.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2GetFMUstate(c: fmi2Component, state: *mut fmi2FMUstate) -> fmi2Status {
    unsafe { state_get(c, state) }
}

/// Restores a state by replaying its steps from a fresh simulation: as long as simulating up to it took.
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
pub unsafe extern "C" fn fmi2SerializedFMUstateSize(c: fmi2Component, state: fmi2FMUstate, size: *mut usize) -> fmi2Status {
    unsafe { state_size(c, state, size) }
}

/// # Safety
/// See `fmi2SetFMUstate`; `bytes` points to `size` bytes, the size `fmi2SerializedFMUstateSize` gave.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SerializeFMUstate(c: fmi2Component, state: fmi2FMUstate, bytes: *mut fmi2Byte, size: usize) -> fmi2Status {
    unsafe { state_serialize(c, state, bytes as *mut u8, size) }
}

/// # Safety
/// See `fmi2GetFMUstate`; `bytes` points to `size` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2DeSerializeFMUstate(c: fmi2Component, bytes: *const fmi2Byte, size: usize, state: *mut fmi2FMUstate) -> fmi2Status {
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
