//! The FMI 2.0 Co-Simulation C functions. The type definitions follow the FMI 2.0 headers (fmi2TypesPlatform.h,
//! fmi2FunctionTypes.h; Copyright Modelica Association Project "FMI", BSD-2-Clause licence), written out in Rust.
//!
//! Supported: instantiation, setup, initialisation, `fmi2DoStep`, get and set of Real, Integer (there are none),
//! Boolean and String variables, reset, terminate. The rest (FMU state, directional derivatives, input
//! derivatives, asynchronous steps, status queries) return `fmi2Error`; the model description says so.
//! A panic inside the simulation returns `fmi2Fatal` instead of taking the importing tool down.

#![allow(non_snake_case, non_camel_case_types, non_upper_case_globals)]

use crate::{Kind, Model, texts};
use sim_components::{Command, Sim, load_config};
use sim_core::Translations;
use std::ffi::{CStr, CString, c_char, c_int, c_void};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::path::PathBuf;

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

#[derive(Debug, Clone, Copy, PartialEq)]
enum Mode {
    Instantiated,
    Initialising,
    Stepping,
    Terminated,
}

/// A value the importer has set.
#[derive(Debug, Clone)]
enum Value {
    Real(f64),
    Bool(bool),
    Text(String),
}

/// A refusal: a translation key and its argument.
type Fail = (&'static str, String);

pub struct Instance {
    name: CString,
    logger: Option<fmi2CallbackLogger>,
    environment: fmi2ComponentEnvironment,
    tr: Translations,
    model: Model,
    /// Per variable, what the importer set; none: the start value.
    set: Vec<Option<Value>>,
    mode: Mode,
    start_time: f64,
    /// Built from the parameters when initialisation needs it, and dropped when a parameter changes before that.
    sim: Option<Sim>,
    sample: Vec<f64>,
    /// What `fmi2GetString` handed out, valid until its next call.
    strings: Vec<CString>,
}

fn c_text(text: &str) -> CString {
    CString::new(text.replace('\0', "")).unwrap_or_default()
}

fn log(logger: Option<fmi2CallbackLogger>, environment: fmi2ComponentEnvironment, name: &CStr, status: fmi2Status, message: &str) {
    let Some(logger) = logger else { return };
    let category = if status == fmi2Warning { c"logStatusWarning" } else { c"logStatusError" };
    let message = c_text(message);
    // The message as an argument of "%s": a per cent sign in it is not a format.
    unsafe { logger(environment, name.as_ptr(), status, category.as_ptr(), c"%s".as_ptr(), message.as_ptr()) };
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
unsafe fn text<'a>(p: fmi2String) -> &'a str {
    if p.is_null() { "" } else { unsafe { CStr::from_ptr(p) }.to_str().unwrap_or("") }
}

/// # Safety
/// The pointer points to `n` values, or `n` is 0.
unsafe fn slice<'a, T>(p: *const T, n: usize) -> &'a [T] {
    if n == 0 || p.is_null() { &[] } else { unsafe { std::slice::from_raw_parts(p, n) } }
}

/// # Safety
/// As `slice`, writable.
unsafe fn slice_mut<'a, T>(p: *mut T, n: usize) -> &'a mut [T] {
    if n == 0 || p.is_null() { &mut [] } else { unsafe { std::slice::from_raw_parts_mut(p, n) } }
}

impl Instance {
    fn fail(&self, status: fmi2Status, (key, arg): Fail) -> fmi2Status {
        log(self.logger, self.environment, &self.name, status, &self.tr.tr_arg(key, &arg));
        status
    }

    fn in_mode(&self, modes: &[Mode], call: &str) -> Result<(), Fail> {
        if modes.contains(&self.mode) { Ok(()) } else { Err(("fmi-error-state", format!("{call} ({:?})", self.mode))) }
    }

    fn variable(&self, vr: fmi2ValueReference, type_name: &str) -> Result<usize, Fail> {
        match self.model.variables.get(vr as usize) {
            Some(v) if v.type_name() == type_name => Ok(vr as usize),
            _ => Err(("fmi-error-reference", format!("{type_name} {vr}"))),
        }
    }

    fn start(&self, index: usize) -> &str {
        self.model.variables[index].start.as_deref().unwrap_or("")
    }

    fn text_value(&self, index: usize) -> String {
        match &self.set[index] {
            Some(Value::Text(t)) => t.clone(),
            _ => self.start(index).into(),
        }
    }

    fn bool_value(&self, index: usize) -> bool {
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

    fn get_real(&mut self, vr: fmi2ValueReference) -> Result<f64, Fail> {
        // FMI 2.0 allows getting values from initialisation mode on: before it the start time is not yet set up.
        self.in_mode(&[Mode::Initialising, Mode::Stepping, Mode::Terminated], "fmi2GetReal")?;
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

    fn set_value(&mut self, vr: fmi2ValueReference, value: Value) -> Result<(), Fail> {
        let type_name = match value {
            Value::Real(_) => "Real",
            Value::Bool(_) => "Boolean",
            Value::Text(_) => "String",
        };
        let index = self.variable(vr, type_name)?;
        let v = &self.model.variables[index];
        match v.causality() {
            "input" => self.in_mode(&[Mode::Instantiated, Mode::Initialising, Mode::Stepping], "fmi2Set")?,
            "parameter" => {
                self.in_mode(&[Mode::Instantiated, Mode::Initialising], "fmi2Set")?;
                self.sim = None;
            }
            _ => return Err(("fmi-error-state", format!("fmi2Set {}", v.name))),
        }
        self.set[index] = Some(value);
        Ok(())
    }

    fn do_step(&mut self, point: f64, size: f64) -> Result<fmi2Status, Fail> {
        self.in_mode(&[Mode::Stepping], "fmi2DoStep")?;
        // An infinite step would loop for ever, and a time that is not a number would pass every comparison.
        if !(point.is_finite() && size.is_finite()) {
            return Err(("fmi-error-step", format!("{point:?}, {size:?}")));
        }
        let sim = self.sim.as_mut().expect("built on leaving initialisation");
        let steps = (size / sim.dt).round();
        // More base steps than a u32 holds would run for days: refused as an importer's mistake.
        if !(1.0..=u32::MAX as f64).contains(&steps) || (steps * sim.dt - size).abs() > 1e-9 * size.max(sim.dt) {
            return Err(("fmi-error-step", format!("{size:?} / {:?}", sim.dt)));
        }
        let now = self.start_time + sim.time();
        if !((point - now).abs() <= 0.5 * sim.dt) {
            return Err(("fmi-error-time", format!("{point:?} / {now:?}")));
        }
        let mut status = fmi2OK;
        let mut warn = |key: &'static str| {
            log(self.logger, self.environment, &self.name, fmi2Warning, &self.tr.tr_arg("fmi-refused", self.tr.tr(key)));
            status = fmi2Warning;
        };
        let enable = self.model.variables.iter().position(|v| v.kind == Kind::Enable).unwrap_or(0);
        let enable = !matches!(self.set[enable], Some(Value::Bool(false)));
        if enable != sim.control.enabled() {
            if let Err(e) = sim.command(if enable { Command::Enable } else { Command::Disable }) {
                warn(e.key);
            }
        }
        if sim.control.enabled() {
            // An axis the importer has not set stays at the setpoint the controller follows.
            let mut target = sim.control.external_setpoint().to_vec();
            for (index, v) in self.model.variables.iter().enumerate() {
                if let (Kind::Setpoint(i), Some(Value::Real(value))) = (&v.kind, &self.set[index]) {
                    target[*i] = *value;
                }
            }
            if target != sim.control.external_setpoint() {
                if let Err(e) = sim.command(Command::Setpoint(target)) {
                    warn(e.key);
                }
            }
        }
        for _ in 0..steps as u64 {
            sim.step();
        }
        self.take_sample();
        Ok(status)
    }
}

/// Runs `f` on the instance; a refusal is logged and returned as `fmi2Error`, a panic as `fmi2Fatal`.
///
/// # Safety
/// `c` is null or an instance from `fmi2Instantiate` that has not been freed.
unsafe fn with(c: fmi2Component, f: impl FnOnce(&mut Instance) -> Result<fmi2Status, Fail>) -> fmi2Status {
    let Some(instance) = (unsafe { (c as *mut Instance).as_mut() }) else { return fmi2Error };
    match catch_unwind(AssertUnwindSafe(|| f(&mut *instance))) {
        Ok(Ok(status)) => status,
        Ok(Err(fail)) => instance.fail(fmi2Error, fail),
        Err(_) => fmi2Fatal,
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
    let name = c_text(unsafe { text(instanceName) });
    let (guid, resources) = (unsafe { text(fmuGUID) }.to_string(), resource_dir(unsafe { text(fmuResourceLocation) }));
    let made = catch_unwind(AssertUnwindSafe(|| -> Result<Instance, (Translations, Fail)> {
        // Until the FMU's language is known, its messages are in English.
        let mut tr = Translations::default();
        sim_core::i18n::I18N.iter().chain(crate::I18N).filter(|(l, _)| *l == "en").for_each(|(_, t)| tr.add(t));
        if fmuType != fmi2CoSimulation {
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
        let model = match Model::new(entry, &config_text, lang) {
            Ok(model) => model,
            Err(e) => {
                let detail = tr.tr_arg(e.key(), e.detail());
                return Err((tr, ("fmi-error-config", detail)));
            }
        };
        if model.guid != guid {
            return Err((tr, ("fmi-error-guid", guid.clone())));
        }
        let set = vec![None; model.variables.len()];
        Ok(Instance { name: name.clone(), logger, environment, tr, model, set, mode: Mode::Instantiated, start_time: 0.0, sim: None, sample: Vec::new(), strings: Vec::new() })
    }));
    match made {
        Ok(Ok(instance)) => Box::into_raw(Box::new(instance)) as fmi2Component,
        Ok(Err((tr, (key, arg)))) => {
            log(logger, environment, &name, fmi2Error, &tr.tr_arg(key, &arg));
            std::ptr::null_mut()
        }
        Err(_) => std::ptr::null_mut(),
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
    unsafe {
        with(c, |i| {
            i.in_mode(&[Mode::Instantiated], "fmi2SetupExperiment")?;
            // A start time that is not a number would pass every later check of the communication point.
            if !startTime.is_finite() || (stopTimeDefined != 0 && !stopTime.is_finite()) || (toleranceDefined != 0 && !tolerance.is_finite()) {
                return Err(("fmi-error-experiment", format!("{startTime:?}, {stopTime:?}, {tolerance:?}")));
            }
            i.start_time = startTime;
            Ok(fmi2OK)
        })
    }
}

/// # Safety
/// See `with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2EnterInitializationMode(c: fmi2Component) -> fmi2Status {
    unsafe {
        with(c, |i| {
            i.in_mode(&[Mode::Instantiated], "fmi2EnterInitializationMode")?;
            i.mode = Mode::Initialising;
            Ok(fmi2OK)
        })
    }
}

/// # Safety
/// See `with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2ExitInitializationMode(c: fmi2Component) -> fmi2Status {
    unsafe {
        with(c, |i| {
            i.in_mode(&[Mode::Initialising], "fmi2ExitInitializationMode")?;
            i.ensure_sim()?;
            // A fresh sample: the outputs at the start time as it is now.
            i.take_sample();
            i.mode = Mode::Stepping;
            Ok(fmi2OK)
        })
    }
}

/// # Safety
/// See `with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2Terminate(c: fmi2Component) -> fmi2Status {
    unsafe {
        with(c, |i| {
            i.in_mode(&[Mode::Stepping], "fmi2Terminate")?;
            i.mode = Mode::Terminated;
            Ok(fmi2OK)
        })
    }
}

/// # Safety
/// See `with`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2Reset(c: fmi2Component) -> fmi2Status {
    unsafe {
        with(c, |i| {
            (i.mode, i.start_time, i.sim) = (Mode::Instantiated, 0.0, None);
            i.set.iter_mut().for_each(|v| *v = None);
            Ok(fmi2OK)
        })
    }
}

/// # Safety
/// See `with`; `vr` and `value` point to `nvr` values.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2GetReal(c: fmi2Component, vr: *const fmi2ValueReference, nvr: usize, value: *mut fmi2Real) -> fmi2Status {
    unsafe {
        with(c, |i| {
            for (r, v) in slice(vr, nvr).iter().zip(slice_mut(value, nvr)) {
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
            for (r, v) in slice(vr, nvr).iter().zip(slice(value, nvr)) {
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
    unsafe { with(c, |i| slice(vr, nvr).first().map_or(Ok(fmi2OK), |r| i.variable(*r, "Integer").map(|_| fmi2OK))) }
}

/// # Safety
/// See `fmi2GetReal`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SetInteger(c: fmi2Component, vr: *const fmi2ValueReference, nvr: usize, _value: *const fmi2Integer) -> fmi2Status {
    unsafe { with(c, |i| slice(vr, nvr).first().map_or(Ok(fmi2OK), |r| i.variable(*r, "Integer").map(|_| fmi2OK))) }
}

/// # Safety
/// See `fmi2GetReal`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2GetBoolean(c: fmi2Component, vr: *const fmi2ValueReference, nvr: usize, value: *mut fmi2Boolean) -> fmi2Status {
    unsafe {
        with(c, |i| {
            for (r, v) in slice(vr, nvr).iter().zip(slice_mut(value, nvr)) {
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
            for (r, v) in slice(vr, nvr).iter().zip(slice(value, nvr)) {
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
    unsafe {
        with(c, |i| {
            i.strings.clear();
            for r in slice(vr, nvr) {
                let index = i.variable(*r, "String")?;
                i.strings.push(c_text(&i.text_value(index)));
            }
            for (s, v) in i.strings.iter().zip(slice_mut(value, nvr)) {
                *v = s.as_ptr();
            }
            Ok(fmi2OK)
        })
    }
}

/// # Safety
/// See `fmi2GetReal`; every string is valid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn fmi2SetString(c: fmi2Component, vr: *const fmi2ValueReference, nvr: usize, value: *const fmi2String) -> fmi2Status {
    unsafe {
        with(c, |i| {
            for (r, v) in slice(vr, nvr).iter().zip(slice(value, nvr)) {
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
    fmi2GetFMUstate(state: *mut fmi2FMUstate);
    fmi2SetFMUstate(state: fmi2FMUstate);
    fmi2FreeFMUstate(state: *mut fmi2FMUstate);
    fmi2SerializedFMUstateSize(state: fmi2FMUstate, size: *mut usize);
    fmi2SerializeFMUstate(state: fmi2FMUstate, bytes: *mut fmi2Byte, size: usize);
    fmi2DeSerializeFMUstate(bytes: *const fmi2Byte, size: usize, state: *mut fmi2FMUstate);
    fmi2GetDirectionalDerivative(unknown: *const fmi2ValueReference, n_unknown: usize, known: *const fmi2ValueReference, n_known: usize, dv_known: *const fmi2Real, dv_unknown: *mut fmi2Real);
    fmi2SetRealInputDerivatives(vr: *const fmi2ValueReference, nvr: usize, order: *const fmi2Integer, value: *const fmi2Real);
    fmi2GetRealOutputDerivatives(vr: *const fmi2ValueReference, nvr: usize, order: *const fmi2Integer, value: *mut fmi2Real);
    fmi2CancelStep();
    fmi2GetStatus(kind: c_int, value: *mut fmi2Status);
    fmi2GetRealStatus(kind: c_int, value: *mut fmi2Real);
    fmi2GetIntegerStatus(kind: c_int, value: *mut fmi2Integer);
    fmi2GetBooleanStatus(kind: c_int, value: *mut fmi2Boolean);
    fmi2GetStringStatus(kind: c_int, value: *mut fmi2String);
}
