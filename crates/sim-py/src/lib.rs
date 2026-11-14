//! The Python module `kinematics_sim`: the simulator's machines, configs, runs, analyses, verdicts with their bands
//! and the catalogue sizing, as `sim-cli` has them, for notebooks and scripts. Results come as NumPy arrays and as
//! dicts of equal-length columns, which `pandas.DataFrame(result)` takes as they are. The type stubs are in
//! `kinematics_sim.pyi`; `docs/python.md` has the guide.
//!
//! Every long call releases the GIL, so Python threads can run several at once.

use numpy::{PyArray1, PyArray2};
use pyo3::exceptions::{PyKeyError, PyOverflowError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBool, PyDict, PyList, PyString};
use sim_components::analysis::{
    CycleLoad, SWEEP_POINTS, analyse_cycle_with, analyse_trajectory_with, bounds, default_sweep_amplitude,
    frequency_sweep, log_spaced, verdict_bands, verdict_bands_sampled,
};
use sim_components::identify::{BELT_STIFFNESS_KEY, fit_joints, fit_move_stiffness, fit_rigid, fit_sweep, parse_sweep};
use sim_components::import::{Imported, Mapping, decode, import};
use sim_components::motor_fit::{
    Estimate, Identified, TwoMass, frequency_response, ringing, two_mass, two_mass_from_ringing,
};
use sim_components::trace::{Trace, TraceError, per_axis_fit};
use sim_components::{Command, Sim, load_config};
use sim_core::{ConfigError, MachineEntry, MotorInfo, Translations};
use sim_machines::{LANGUAGES, MACHINES, find, translations};
use std::path::{Path, PathBuf};

fn texts(entry: &MachineEntry) -> Translations {
    translations("en", entry)
}

fn config_error(entry: &MachineEntry, e: ConfigError) -> PyErr {
    PyValueError::new_err(texts(entry).tr_arg(e.key(), e.detail()))
}

fn machine(id: &str) -> PyResult<&'static MachineEntry> {
    find(id).ok_or_else(|| {
        PyKeyError::new_err(format!(
            "unknown machine: {id}; known: {}",
            MACHINES.iter().map(|m| m.id).collect::<Vec<_>>().join(", ")
        ))
    })
}

/// A machine's config: its default, or a config file, with values set on top as `sim-cli --set` sets them.
#[pyclass(name = "Config", module = "kinematics_sim", from_py_object)]
#[derive(Clone)]
struct PyConfig {
    entry: &'static MachineEntry,
    /// The config as read, with the parts named under `[parts]` (from the text or set since) not yet brought in.
    base: sim_core::Config,
    /// Every other value set since, in order.
    sets: Vec<(String, toml::Value)>,
    /// What runs: `base` with its parts from the catalogue, then `sets` on top, as `sim-cli` builds a config from
    /// `--config`, the part options and `--set`, whatever order they are given in.
    cfg: sim_core::Config,
    /// The text the config was read from (the machine's default where none was given) and where it came from (none:
    /// the default), for the report's sources of the values.
    text: String,
    source: Option<String>,
}

impl PyConfig {
    fn rebuild(&mut self) -> PyResult<()> {
        let mut cfg =
            sim_components::catalog::resolve(self.entry, &self.base).map_err(|e| config_error(self.entry, e))?;
        for (path, value) in &self.sets {
            cfg.set(path, value.clone()).map_err(|e| config_error(self.entry, e))?;
        }
        cfg.check_keys(&self.entry.config(), &sim_components::analysis::RATINGS_BEYOND_DEFAULT)
            .map_err(|e| config_error(self.entry, e))?;
        self.cfg = cfg;
        Ok(())
    }

    /// One value set: a part into `base`, anything else onto the list. Nothing changes if the result is refused.
    fn assign(&mut self, path: &str, value: toml::Value) -> PyResult<()> {
        let mut next = self.clone();
        if path.starts_with("parts.") {
            next.base.set(path, value).map_err(|e| config_error(self.entry, e))?;
        } else {
            next.sets.push((path.to_string(), value));
        }
        next.rebuild()?;
        *self = next;
        Ok(())
    }
}

#[pymethods]
impl PyConfig {
    /// The default config of `machine`, or the config in `text` (TOML), or in the file at `path`. Instead of a
    /// registered machine, `machine_file` names a machine file (docs/machine-files.md), as `sim-cli --machine-file`.
    #[new]
    #[pyo3(signature = (machine=None, text=None, path=None, *, machine_file=None))]
    fn new(
        machine: Option<&str>,
        text: Option<&str>,
        path: Option<std::path::PathBuf>,
        machine_file: Option<std::path::PathBuf>,
    ) -> PyResult<Self> {
        let entry = match (machine, machine_file) {
            (Some(id), None) => self::machine(id)?,
            (None, Some(file)) => sim_machines::load_file(&file).map_err(|e| {
                let at = e.line.map_or(String::new(), |line| format!(":{line}"));
                // The core's and the components' messages; any machine brings them.
                let tr = texts(MACHINES[0]);
                PyValueError::new_err(format!(
                    "{}{at}: {}",
                    e.file.display(),
                    tr.tr_arg(e.error.key(), e.error.detail())
                ))
            })?,
            _ => return Err(PyValueError::new_err("give machine or machine_file, one of them")),
        };
        let (text, source) = match (text, path) {
            (Some(_), Some(_)) => return Err(PyValueError::new_err("give text or path, not both")),
            (Some(text), None) => (Some(text.to_string()), Some("text".to_string())),
            (None, Some(path)) => (
                Some(
                    std::fs::read_to_string(&path)
                        .map_err(|e| PyValueError::new_err(format!("{}: {e}", path.display())))?,
                ),
                Some(path.display().to_string()),
            ),
            (None, None) => (None, None),
        };
        let cfg = match &text {
            Some(text) => load_config(entry, text).map_err(|e| config_error(entry, e))?,
            None => entry.config(),
        };
        let text = text.unwrap_or_else(|| entry.default_config.to_string());
        let mut out = Self { entry, base: cfg.clone(), sets: Vec::new(), cfg, text, source };
        out.rebuild()?;
        Ok(out)
    }

    #[getter]
    fn machine(&self) -> &'static str {
        self.entry.id
    }

    /// The value at a dotted path, e.g. `cfg["payload.mass_kg"]`.
    fn __getitem__(&self, py: Python<'_>, path: &str) -> PyResult<Py<PyAny>> {
        let value = self.cfg.get(path).map_err(|_| PyKeyError::new_err(path.to_string()))?;
        from_toml(py, value)
    }

    /// Sets the value at a dotted path, e.g. `cfg["payload.mass_kg"] = 2.5`, as `sim-cli --set` does: a part named under
    /// `[parts]` comes in from the catalogue as with `--motor`, under the other values set.
    fn __setitem__(&mut self, path: &str, value: &Bound<'_, PyAny>) -> PyResult<()> {
        self.assign(path, to_toml(value)?)
    }

    /// One `path = value` assignment with the value in TOML syntax, exactly as `sim-cli --set` takes it.
    fn set(&mut self, assignment: &str) -> PyResult<()> {
        let (path, value) = assignment
            .split_once('=')
            .ok_or_else(|| config_error(self.entry, ConfigError::Parse(assignment.into())))?;
        let parsed: toml::Table = format!("v = {value}")
            .parse()
            .map_err(|_| config_error(self.entry, ConfigError::Parse(assignment.into())))?;
        self.assign(path.trim(), parsed["v"].clone())
    }

    /// A copy, for a sweep that changes it.
    fn copy(&self) -> Self {
        self.clone()
    }

    /// Every value with its dotted path.
    fn leaves(&self, py: Python<'_>) -> PyResult<Py<PyDict>> {
        let out = PyDict::new(py);
        for (path, value) in self.cfg.leaves() {
            out.set_item(path, from_toml(py, &value)?)?;
        }
        Ok(out.unbind())
    }

    /// The config as TOML, which `Config(machine, text=...)` reads back.
    fn to_toml(&self) -> PyResult<String> {
        self.cfg.to_text().map_err(|e| config_error(self.entry, e))
    }

    fn __repr__(&self) -> String {
        format!("Config('{}')", self.entry.id)
    }
}

fn to_toml(value: &Bound<'_, PyAny>) -> PyResult<toml::Value> {
    use toml::Value;
    // bool before int: a Python bool is an int. The extractions take NumPy's scalars too (numpy.bool_, and integers
    // through `__index__`); a NumPy array takes none of them and becomes an array below. An integer beyond TOML's
    // 64 bits is an OverflowError, not a float that has lost its last digits.
    let integer = value.extract::<i64>();
    if let Err(e) = &integer
        && e.is_instance_of::<PyOverflowError>(value.py())
    {
        return Err(PyOverflowError::new_err(format!("{value} does not fit a 64-bit integer")));
    }
    Ok(if let Ok(b) = value.extract::<bool>() {
        Value::Boolean(b)
    } else if let Ok(i) = integer {
        Value::Integer(i)
    } else if let Ok(f) = value.extract::<f64>() {
        Value::Float(f)
    } else if let Ok(s) = value.cast::<PyString>() {
        Value::String(s.to_str()?.to_string())
    } else if let Ok(d) = value.cast::<PyDict>() {
        let mut table = toml::Table::new();
        for (k, v) in d.iter() {
            table.insert(k.extract()?, to_toml(&v)?);
        }
        Value::Table(table)
    } else {
        Value::Array(value.try_iter()?.map(|item| to_toml(&item?)).collect::<PyResult<_>>()?)
    })
}

fn from_toml(py: Python<'_>, value: &toml::Value) -> PyResult<Py<PyAny>> {
    use toml::Value;
    Ok(match value {
        Value::Boolean(b) => PyBool::new(py, *b).to_owned().into_any().unbind(),
        Value::Integer(i) => i.into_pyobject(py)?.into_any().unbind(),
        Value::Float(f) => f.into_pyobject(py)?.into_any().unbind(),
        Value::String(s) => s.into_pyobject(py)?.into_any().unbind(),
        Value::Datetime(d) => d.to_string().into_pyobject(py)?.into_any().unbind(),
        Value::Array(items) => {
            PyList::new(py, items.iter().map(|v| from_toml(py, v)).collect::<PyResult<Vec<_>>>()?)?.into_any().unbind()
        }
        Value::Table(table) => {
            let out = PyDict::new(py);
            for (k, v) in table {
                out.set_item(k, from_toml(py, v)?)?;
            }
            out.into_any().unbind()
        }
    })
}

/// Every machine of the registry: `[{"id": ..., "name": ...}]`, the names in `lang` ("en" or "de").
#[pyfunction]
#[pyo3(signature = (lang="en"))]
fn machines(py: Python<'_>, lang: &str) -> PyResult<Py<PyList>> {
    if !LANGUAGES.contains(&lang) {
        return Err(PyValueError::new_err(format!("unknown language: {lang}")));
    }
    let rows = PyList::empty(py);
    for m in MACHINES {
        let row = PyDict::new(py);
        row.set_item("id", m.id)?;
        row.set_item("name", translations(lang, m).tr(m.key))?;
        rows.append(row)?;
    }
    Ok(rows.unbind())
}

/// Every signal of a run of this config, in the order `run` gives them: `{"id": [...], "unit": [...]}`.
#[pyfunction]
fn signals(py: Python<'_>, config: &PyConfig) -> PyResult<Py<PyDict>> {
    let sim = Sim::new(config.entry, &config.cfg).map_err(|e| config_error(config.entry, e))?;
    let (ids, units): (Vec<String>, Vec<String>) = sim.signal_infos().into_iter().map(|s| (s.id, s.unit)).unzip();
    let out = PyDict::new(py);
    out.set_item("id", ids)?;
    out.set_item("unit", units)?;
    Ok(out.unbind())
}

/// What the machine is told to do: the cycle, a move, a path (text of a path file), or poses at times (text of a
/// trajectory file); at most one.
fn motion(
    entry: &MachineEntry,
    sim: &Sim,
    cycle: bool,
    move_to: Option<Vec<f64>>,
    path: Option<&str>,
    trajectory: Option<&str>,
    time_optimal: bool,
) -> Result<Option<Command>, String> {
    let given = [cycle, move_to.is_some(), path.is_some(), trajectory.is_some()].iter().filter(|g| **g).count();
    if given > 1 {
        return Err("give at most one of cycle, move_to, path, trajectory".into());
    }
    Ok(if cycle {
        Some(Command::StartCycle)
    } else if let Some(target) = move_to {
        Some(Command::MoveTo(target))
    } else {
        poses(entry, sim, path, trajectory, time_optimal)?.map(Command::Trajectory)
    })
}

fn poses(
    entry: &MachineEntry,
    sim: &Sim,
    path: Option<&str>,
    trajectory: Option<&str>,
    time_optimal: bool,
) -> Result<Option<Vec<(f64, Vec<f64>)>>, String> {
    if let Some(text) = trajectory {
        return sim_components::path::parse_trajectory(text).map(Some).map_err(|line| format!("trajectory: {line}"));
    }
    let Some(text) = path else { return Ok(None) };
    let pieces = sim_components::path::parse(text).map_err(|line| format!("path: {line}"))?;
    sim.plan_path(&pieces, time_optimal).map(Some).map_err(|e| texts(entry).tr(e.key).to_string())
}

/// The longest run `run` takes: 1e8 base steps (some 10^4 s simulated at the usual 0.1 ms, minutes to hours of
/// computing), and 2.5e8 values sampled (2 GB of arrays). Beyond, it would seem to hang or exhaust the memory.
const MAX_RUN_STEPS: f64 = 1e8;
const MAX_RUN_VALUES: f64 = 2.5e8;

/// Simulates `time` seconds and returns every signal, sampled every `every` base steps, as
/// `{signal id: numpy array}` (the first is "time"). `disable_at` switches the drives off at that time.
#[pyfunction]
#[pyo3(signature = (config, *, cycle=false, move_to=None, path=None, trajectory=None, time_optimal=false, time=2.0, every=16, disable_at=None))]
#[allow(clippy::too_many_arguments)]
fn run(
    py: Python<'_>,
    config: &PyConfig,
    cycle: bool,
    move_to: Option<Vec<f64>>,
    path: Option<String>,
    trajectory: Option<String>,
    time_optimal: bool,
    time: f64,
    every: i64,
    disable_at: Option<f64>,
) -> PyResult<Py<PyDict>> {
    // `every` as i64, so that a negative one is refused with ValueError like the rest, not OverflowError.
    if !(time.is_finite() && time >= 0.0) || every < 1 || disable_at.is_some_and(|t| !(t.is_finite() && t >= 0.0)) {
        return Err(PyValueError::new_err("time and disable_at must be finite and not negative, every at least 1"));
    }
    let every = every as u64;
    let (entry, cfg) = (config.entry, config.cfg.clone());
    let (ids, columns) = py
        .detach(|| -> Result<_, String> {
            let tr = texts(entry);
            let mut sim = Sim::new(entry, &cfg).map_err(|e| tr.tr_arg(e.key(), e.detail()))?;
            if let Some(command) =
                motion(entry, &sim, cycle, move_to, path.as_deref(), trajectory.as_deref(), time_optimal)?
            {
                sim.command(command).map_err(|e| tr.tr(e.key).to_string())?;
            }
            let ids: Vec<String> = sim.signal_infos().into_iter().map(|s| s.id).collect();
            let steps = (time / sim.dt).round();
            let values = (steps / every as f64 + 1.0) * ids.len() as f64;
            if steps > MAX_RUN_STEPS || values > MAX_RUN_VALUES {
                return Err(format!(
                    "time = {time} s is {steps:.3e} base steps of {} s and {values:.3e} values; at most {MAX_RUN_STEPS:.0e} \
                     steps and {MAX_RUN_VALUES:.0e} values (raise every, or run in pieces)",
                    sim.dt
                ));
            }
            let mut columns: Vec<Vec<f64>> = vec![Vec::new(); ids.len()];
            let disable = disable_at.map(|t| (t / sim.dt).round() as u64);
            let mut sample = Vec::new();
            for step in 0..=(time / sim.dt).round() as u64 {
                if step % every == 0 {
                    sim.sample(&mut sample);
                    columns.iter_mut().zip(&sample).for_each(|(c, v)| c.push(*v));
                }
                if Some(step) == disable {
                    sim.command(Command::Disable).map_err(|e| tr.tr(e.key).to_string())?;
                }
                sim.step();
            }
            Ok((ids, columns))
        })
        .map_err(PyValueError::new_err)?;
    let out = PyDict::new(py);
    for (id, column) in ids.into_iter().zip(columns) {
        out.set_item(id, PyArray1::from_vec(py, column))?;
    }
    Ok(out.unbind())
}

/// The load per motor over one pass of the cycle, or over a path or trajectory, as `sim-cli analyze` gives it: a dict
/// of columns, one row per motor. `points=True` adds "points", per motor an (n, 2) array of speed (rad/s) and torque
/// (N m) at its operating points.
#[pyfunction]
#[pyo3(signature = (config, *, path=None, trajectory=None, time_optimal=false, points=false))]
fn analyze(
    py: Python<'_>,
    config: &PyConfig,
    path: Option<String>,
    trajectory: Option<String>,
    time_optimal: bool,
    points: bool,
) -> PyResult<Py<PyDict>> {
    let (entry, cfg) = (config.entry, config.cfg.clone());
    let (motors, load) = py
        .detach(|| -> Result<(Vec<&'static str>, CycleLoad), String> {
            let tr = texts(entry);
            let mut sim = Sim::new(entry, &cfg).map_err(|e| tr.tr_arg(e.key(), e.detail()))?;
            let motors = sim.machine.motors().iter().map(|m| m.id).collect();
            let load = match poses(entry, &sim, path.as_deref(), trajectory.as_deref(), time_optimal)? {
                Some(points) => analyse_trajectory_with(&mut sim, points, false),
                None => analyse_cycle_with(&mut sim, false),
            }
            .map_err(|e| tr.tr(e.key).to_string())?;
            Ok((motors, load))
        })
        .map_err(PyValueError::new_err)?;
    let out = PyDict::new(py);
    let column = |f: fn(&sim_components::analysis::MotorLoad) -> f64| {
        PyArray1::from_vec(py, load.motors.iter().map(f).collect())
    };
    out.set_item("motor", motors)?;
    out.set_item("cycle_s", PyArray1::from_vec(py, vec![load.duration; load.motors.len()]))?;
    out.set_item("peak_torque_nm", column(|m| m.peak_torque))?;
    out.set_item("rms_torque_nm", column(|m| m.rms_torque))?;
    out.set_item("cubic_mean_torque_nm", column(|m| m.cubic_mean_torque))?;
    out.set_item("peak_speed_rad_s", column(|m| m.peak_speed))?;
    out.set_item("utilisation", column(|m| m.utilisation))?;
    out.set_item("thermal_utilisation", column(|m| m.thermal_utilisation))?;
    out.set_item("rms_current_a", column(|m| m.rms_current))?;
    out.set_item("peak_current_a", column(|m| m.peak_current))?;
    out.set_item("winding_rise_k", column(|m| m.winding_rise))?;
    out.set_item("energy_in_j", column(|m| m.energy_in))?;
    out.set_item("energy_regenerated_j", column(|m| m.energy_regenerated))?;
    out.set_item("peak_regenerated_power_w", column(|m| m.peak_regenerated_power))?;
    out.set_item("braking_resistor_energy_j", column(|m| m.resistor_energy))?;
    out.set_item("dc_link_peak_voltage_v", column(|m| m.peak_dc_link_voltage))?;
    let (low, high) = load.belt_tension.unwrap_or((f64::NAN, f64::NAN));
    out.set_item("belt_tension_min_n", PyArray1::from_vec(py, vec![low; load.motors.len()]))?;
    out.set_item("belt_tension_max_n", PyArray1::from_vec(py, vec![high; load.motors.len()]))?;
    for (name, value) in [
        ("path_error_peak_m", load.path_error_peak),
        ("path_error_rms_m", load.path_error_rms),
        ("tracking_error_peak_m", load.tracking_error_peak),
        ("dwell_error_peak_m", load.dwell_error_peak),
    ] {
        out.set_item(name, PyArray1::from_vec(py, vec![value; load.motors.len()]))?;
    }
    if points {
        let list = PyList::empty(py);
        for m in &load.motors {
            let rows: Vec<Vec<f64>> = m.points.iter().map(|(s, t)| vec![*s, *t]).collect();
            list.append(if rows.is_empty() {
                PyArray2::zeros(py, [0, 2], false)
            } else {
                PyArray2::from_vec2(py, &rows)?
            })?;
        }
        out.set_item("points", list)?;
    }
    Ok(out.unbind())
}

/// The cycle judged against the motor data and the `[ratings]` of the config, as `sim-cli verdicts` judges it: a dict
/// of columns, one row per check and motor. `bands=True` adds "low", "high" and "worst_ok" over the parameters with
/// `[bounds]`, one at a time at their bounds, or with `samples=N` N runs with all of them drawn between their bounds
/// at once (seed `seed`), which also adds "pass_share". `bands=True, worst=True` searches per check for the worst corner
/// of the bounds instead, as `sim-cli verdicts --bands --worst`: columns "check", "motor", "worst", "limit", "worst_ok",
/// "at_high_bound" and "at_low_bound" (per row the parameters at that bound there), and "runs", the cycles simulated.
#[pyfunction]
#[pyo3(signature = (config, *, bands=false, samples=None, seed=1, worst=false))]
fn verdicts(
    py: Python<'_>,
    config: &PyConfig,
    bands: bool,
    samples: Option<usize>,
    seed: u64,
    worst: bool,
) -> PyResult<Py<PyDict>> {
    if samples.is_some() && !bands {
        return Err(PyValueError::new_err("samples needs bands=True"));
    }
    if samples == Some(0) {
        return Err(PyValueError::new_err("samples must be at least 1"));
    }
    if worst && (!bands || samples.is_some()) {
        return Err(PyValueError::new_err("worst needs bands=True and no samples"));
    }
    let (entry, cfg) = (config.entry, config.cfg.clone());
    if worst {
        let (motors, bounded, (found, runs)) = py
            .detach(|| -> Result<_, ConfigError> {
                let motors: Vec<&'static str> = (entry.build)(&cfg)?.motors().iter().map(|m| m.id).collect();
                let bounded = bounds(&cfg)?;
                let found = sim_components::analysis::verdict_worst(entry, &cfg, &bounded)?;
                Ok((motors, bounded, found))
            })
            .map_err(|e| config_error(entry, e))?;
        let at = |w: &sim_components::analysis::Worst, high: bool| -> Vec<String> {
            bounded.iter().zip(&w.high).filter(|(_, h)| **h == Some(high)).map(|(b, _)| b.0.clone()).collect()
        };
        let out = PyDict::new(py);
        out.set_item("check", found.iter().map(|w| w.verdict.check).collect::<Vec<_>>())?;
        out.set_item("motor", found.iter().map(|w| w.verdict.motor.map(|k| motors[k])).collect::<Vec<_>>())?;
        out.set_item("worst", PyArray1::from_vec(py, found.iter().map(|w| w.verdict.value).collect()))?;
        out.set_item("limit", PyArray1::from_vec(py, found.iter().map(|w| w.verdict.limit).collect()))?;
        out.set_item("worst_ok", PyArray1::from_vec(py, found.iter().map(|w| w.verdict.ok).collect()))?;
        out.set_item("at_high_bound", found.iter().map(|w| at(w, true)).collect::<Vec<_>>())?;
        out.set_item("at_low_bound", found.iter().map(|w| at(w, false)).collect::<Vec<_>>())?;
        out.set_item("runs", runs)?;
        return Ok(out.unbind());
    }
    let (motors, found) = py
        .detach(|| -> Result<_, ConfigError> {
            let motors: Vec<&'static str> = (entry.build)(&cfg)?.motors().iter().map(|m| m.id).collect();
            let bounded = if bands { bounds(&cfg)? } else { Vec::new() };
            let found = match samples {
                Some(n) => verdict_bands_sampled(entry, &cfg, &bounded, n, seed)?,
                None => verdict_bands(entry, &cfg, &bounded)?,
            };
            Ok((motors, found))
        })
        .map_err(|e| config_error(entry, e))?;
    let out = PyDict::new(py);
    out.set_item("check", found.iter().map(|b| b.verdict.check).collect::<Vec<_>>())?;
    out.set_item("motor", found.iter().map(|b| b.verdict.motor.map(|k| motors[k])).collect::<Vec<_>>())?;
    out.set_item("value", PyArray1::from_vec(py, found.iter().map(|b| b.verdict.value).collect()))?;
    out.set_item("limit", PyArray1::from_vec(py, found.iter().map(|b| b.verdict.limit).collect()))?;
    out.set_item("ok", PyArray1::from_vec(py, found.iter().map(|b| b.verdict.ok).collect()))?;
    if bands {
        out.set_item("low", PyArray1::from_vec(py, found.iter().map(|b| b.low).collect()))?;
        out.set_item("high", PyArray1::from_vec(py, found.iter().map(|b| b.high).collect()))?;
        out.set_item("worst_ok", PyArray1::from_vec(py, found.iter().map(|b| b.ok).collect()))?;
    }
    if samples.is_some() {
        out.set_item("pass_share", PyArray1::from_vec(py, found.iter().map(|b| b.share).collect()))?;
    }
    Ok(out.unbind())
}

/// Every motor of the catalogue on the cycle of the config, each with the drive the catalogue pairs with it, the
/// passing ones first, cheapest first, as `sim-cli search` gives them: a dict of columns, one row per candidate.
/// `series` keeps article numbers starting with it, `supply` is one of the catalogue's supplies ("230v", "400v", "s120",
/// "s210new-400v", "beckhoff-400v", ...), `vendor` keeps that maker's supplies ("all": every maker's; all of them where
/// `supply` is left out; without either the config's own `parts.motor_supply`, else "400v"), `gearboxes` names a gearbox
/// series to try as well, and `axes=True` tries every axis of the catalogue too. `motor_id` sizes one motor of a machine
/// with several (a machine file's): each candidate becomes that motor's own `[parts.ID]`, and the cost, the tightest
/// check, the peak torque and the winding rise are that motor's alone; not with `axes`. The candidates of several
/// supplies are ranked together; `vendor` and `supply` say whose each is.
#[pyfunction]
#[pyo3(signature = (config, *, supply=None, vendor=None, series="", gearboxes=None, axes=false, motor_id=None))]
fn sizing(
    py: Python<'_>,
    config: &PyConfig,
    supply: Option<&str>,
    vendor: Option<&str>,
    series: &str,
    gearboxes: Option<&str>,
    axes: bool,
    motor_id: Option<&str>,
) -> PyResult<Py<PyDict>> {
    use sim_components::catalog::{Parts, SUPPLIES, search_supplies, vendor_of};
    if motor_id.is_some() && axes {
        return Err(PyValueError::new_err("give motor_id or axes=True, not both"));
    }
    let (entry, cfg) = (config.entry, config.cfg.clone());
    let own = Parts::of(&cfg).map(|p| p.supply.to_string()).unwrap_or_default();
    let supplies: Vec<&str> = match (supply, vendor) {
        (None, Some(_)) => SUPPLIES.to_vec(),
        (supply, _) => vec![supply.or(Some(own.as_str()).filter(|s| !s.is_empty())).unwrap_or("400v")],
    };
    let maker = vendor.filter(|v| *v != "all");
    let supplies: Vec<&str> = supplies.into_iter().filter(|s| maker.is_none_or(|v| vendor_of(s) == v)).collect();
    if supplies.is_empty() || !supplies.iter().all(|s| SUPPLIES.contains(s)) {
        return Err(PyValueError::new_err(format!("no supply of the catalogue: {supply:?}, vendor {vendor:?}")));
    }
    let found = py
        .detach(|| {
            let wanted = |article: &str| article.starts_with(series);
            search_supplies(entry, &cfg, &supplies, &wanted, gearboxes, axes, motor_id)
        })
        .map_err(|e| config_error(entry, e))?;
    let out = PyDict::new(py);
    let numbers =
        |f: &dyn Fn(&sim_components::catalog::Candidate) -> f64| PyArray1::from_vec(py, found.iter().map(f).collect());
    out.set_item("axis", found.iter().map(|c| c.axis.clone()).collect::<Vec<_>>())?;
    out.set_item("motor", found.iter().map(|c| c.article).collect::<Vec<_>>())?;
    out.set_item("gearbox", found.iter().map(|c| c.gearbox.clone()).collect::<Vec<_>>())?;
    out.set_item("drive", found.iter().map(|c| c.drive).collect::<Vec<_>>())?;
    out.set_item(
        "vendor",
        found.iter().map(|c| if c.supply.is_empty() { "" } else { vendor_of(c.supply) }).collect::<Vec<_>>(),
    )?;
    out.set_item("supply", found.iter().map(|c| c.supply).collect::<Vec<_>>())?;
    out.set_item("mass_kg", numbers(&|c| c.mass))?;
    out.set_item("cost_kg", numbers(&|c| c.cost))?;
    out.set_item("passes", PyArray1::from_vec(py, found.iter().map(|c| c.passes()).collect()))?;
    out.set_item("tightest_check", found.iter().map(|c| c.tightest().map_or(c.refused, |t| t.0)).collect::<Vec<_>>())?;
    out.set_item("tightest_utilisation", numbers(&|c| c.tightest().map_or(f64::NAN, |t| t.1)))?;
    // Of the searched motor alone with `motor_id` (the others are the config's, alike for every candidate).
    let most = |c: &sim_components::catalog::Candidate, f: fn(&sim_components::analysis::MotorLoad) -> f64| {
        c.result.as_ref().map_or(f64::NAN, |(load, _)| {
            load.motors.iter().filter(|m| c.motor.is_empty() || m.id == c.motor).map(f).fold(0.0, f64::max)
        })
    };
    out.set_item("peak_torque_nm", numbers(&|c| most(c, |m| m.peak_torque)))?;
    out.set_item("winding_rise_k", numbers(&|c| most(c, |m| m.winding_rise)))?;
    Ok(out.unbind())
}

/// The mechanical frequency response from motor `motor`'s torque to its speed, as `sim-cli sweep` gives it: columns
/// "frequency_hz", "gain_rad_s_per_nm", "phase_deg" at `count` frequencies spaced logarithmically from `from_hz` to
/// `to_hz`, with a torque amplitude of `amplitude` N m (the default as the command line's).
#[pyfunction]
#[pyo3(signature = (config, *, motor=0, from_hz=5.0, to_hz=500.0, count=SWEEP_POINTS, amplitude=None))]
fn sweep(
    py: Python<'_>,
    config: &PyConfig,
    motor: usize,
    from_hz: f64,
    to_hz: f64,
    count: usize,
    amplitude: Option<f64>,
) -> PyResult<Py<PyDict>> {
    if !(from_hz > 0.0 && to_hz > from_hz && to_hz.is_finite()) || count < 2 {
        return Err(PyValueError::new_err("from_hz must be above 0 and below to_hz, count at least 2"));
    }
    let (entry, cfg) = (config.entry, config.cfg.clone());
    let points = py
        .detach(|| -> Result<_, ConfigError> {
            let amplitude = match amplitude {
                Some(a) => a,
                None => default_sweep_amplitude(entry, &cfg)?,
            };
            frequency_sweep(entry, &cfg, motor, &log_spaced(from_hz, to_hz, count), amplitude)
        })
        .map_err(|e| config_error(entry, e))?;
    let out = PyDict::new(py);
    out.set_item("frequency_hz", PyArray1::from_vec(py, points.iter().map(|p| p.frequency).collect()))?;
    out.set_item("gain_rad_s_per_nm", PyArray1::from_vec(py, points.iter().map(|p| p.gain).collect()))?;
    out.set_item("phase_deg", PyArray1::from_vec(py, points.iter().map(|p| p.phase_deg).collect()))?;
    Ok(out.unbind())
}

/// A shorter cycle that still passes every verdict, as `sim-cli optimise` finds it (`runs` cycles at most; judged in
/// the error bands of `[bounds]` unless `nominal`, at their worst corner with `worst`): "cycle_s" and
/// "dwell_error_peak_m" as (before, after), "runs", "changes" (columns "setting", "before", "after": the config values
/// to change, and per changed lift its height and via points), "robust" (what the bands cost, or None) and with
/// `pareto=True` "pareto", the settings no other beats in cycle time, energy and peak utilisation at once.
#[pyfunction]
#[pyo3(signature = (config, *, runs=40, nominal=false, worst=false, pareto=false))]
fn optimise(
    py: Python<'_>,
    config: &PyConfig,
    runs: usize,
    nominal: bool,
    worst: bool,
    pareto: bool,
) -> PyResult<Py<PyDict>> {
    let (entry, cfg) = (config.entry, config.cfg.clone());
    let options = sim_components::analysis::OptimiseOptions { max_runs: runs, robust: !nominal, pareto, worst };
    let found = py
        .detach(|| sim_components::analysis::optimise_cycle_with(entry, &cfg, &options))
        .map_err(|e| config_error(entry, e))?;
    let out = PyDict::new(py);
    out.set_item("cycle_s", (found.before.duration, found.after.duration))?;
    out.set_item("dwell_error_peak_m", (found.before.dwell_error_peak, found.after.dwell_error_peak))?;
    out.set_item("runs", found.runs)?;
    // The new cycle itself is TOML; its lifts say what changed in it, as the command line writes them.
    let mut rows: Vec<(String, String, String)> =
        found.changes.iter().filter(|c| c.0 != "cycle.segment").cloned().collect();
    for lift in &found.lifts {
        let [a, b, c] = lift.segments;
        rows.push((
            format!("cycle.segment[{a}-{b}-{c}].lift_m"),
            format!("{:.4}", lift.before),
            format!("{:.4}", lift.after),
        ));
        if let Some(via) = &lift.via {
            let points: Vec<String> =
                via.iter().map(|p| p.iter().map(|x| format!("{x:.4}")).collect::<Vec<_>>().join(" ")).collect();
            rows.push((format!("cycle.segment[{a}-{b}-{c}].via_m"), String::new(), points.join("; ")));
        }
    }
    let changes = PyDict::new(py);
    changes.set_item("setting", rows.iter().map(|r| r.0.clone()).collect::<Vec<_>>())?;
    changes.set_item("before", rows.iter().map(|r| r.1.clone()).collect::<Vec<_>>())?;
    changes.set_item("after", rows.iter().map(|r| r.2.clone()).collect::<Vec<_>>())?;
    out.set_item("changes", changes)?;
    match &found.robust {
        Some(r) => {
            let robust = PyDict::new(py);
            robust.set_item("judged", if r.worst { "worst" } else { "bands" })?;
            robust.set_item("fastest_cycle_s", r.nominal_duration)?;
            robust.set_item("fastest_fails", r.nominal_fails.clone())?;
            robust.set_item("steps_back", r.steps_back)?;
            robust.set_item("excused", r.excused.clone())?;
            robust.set_item("runs", r.runs)?;
            out.set_item("robust", robust)?;
        }
        None => out.set_item("robust", py.None())?,
    }
    if pareto {
        let front = PyDict::new(py);
        let column = |f: fn(&sim_components::analysis::FrontPoint) -> f64| {
            PyArray1::from_vec(py, found.front.iter().map(f).collect())
        };
        front.set_item("cycle_s", column(|p| p.duration))?;
        front.set_item("energy_j", column(|p| p.energy))?;
        front.set_item("peak_utilisation", column(|p| p.utilisation))?;
        front.set_item("velocity_factor", column(|p| p.factors[0]))?;
        front.set_item("acceleration_factor", column(|p| p.factors[1]))?;
        front.set_item("jerk_factor", column(|p| p.factors[2]))?;
        front.set_item("corner_blending", PyArray1::from_vec(py, found.front.iter().map(|p| p.blending).collect()))?;
        front.set_item("input_shaping", PyArray1::from_vec(py, found.front.iter().map(|p| p.shaping).collect()))?;
        out.set_item("pareto", front)?;
    }
    Ok(out.unbind())
}

fn read_text(path: &Path) -> Result<String, String> {
    std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))
}

/// A drive trace: through the mapping file `map` (a vendor export, docs/measurement-procedure.md), or without one in the
/// simulator's own columns for `motors`; with a mapping and `motors`, its motors have to be as many.
fn load_trace(tr: &Translations, path: &Path, map: Option<&Path>, motors: &[MotorInfo]) -> Result<Imported, String> {
    let traced = |e: TraceError| tr.tr_arg(e.key, &e.detail).to_string();
    let text = decode(&std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?);
    match map {
        Some(map) => {
            let imported = import(&text, &Mapping::parse(&read_text(map)?).map_err(traced)?).map_err(traced)?;
            if !motors.is_empty() && imported.trace.motors.len() != motors.len() {
                let detail = format!("{} [[motor]] for {} motors", imported.trace.motors.len(), motors.len());
                return Err(tr.tr_arg("error-import-map", &detail).to_string());
            }
            Ok(imported)
        }
        None => {
            let trace = Trace::parse(&text, motors).map_err(traced)?;
            Ok(Imported {
                ids: motors.iter().map(|m| m.id.to_string()).collect(),
                torque_setpoints: vec![None; trace.motors.len()],
                trace,
            })
        }
    }
}

/// Rows of a table of quantities: name, of what, and as many numbers per row as the table has.
struct Rows<const N: usize> {
    quantity: Vec<String>,
    of: Vec<String>,
    numbers: Vec<[f64; N]>,
}

impl<const N: usize> Rows<N> {
    fn new() -> Self {
        Self { quantity: Vec::new(), of: Vec::new(), numbers: Vec::new() }
    }
    fn push(&mut self, quantity: &str, of: &str, numbers: [f64; N]) {
        self.quantity.push(quantity.into());
        self.of.push(of.into());
        self.numbers.push(numbers);
    }
    /// The table as a dict of columns: "quantity", "of", then the numbers under `names` (NaN where a row has none).
    fn into_dict<'py>(self, py: Python<'py>, names: [&str; N]) -> PyResult<Bound<'py, PyDict>> {
        let out = PyDict::new(py);
        out.set_item("quantity", self.quantity)?;
        out.set_item("of", self.of)?;
        for (k, name) in names.iter().enumerate() {
            out.set_item(*name, PyArray1::from_vec(py, self.numbers.iter().map(|r| r[k]).collect()))?;
        }
        Ok(out)
    }
}

/// What a measurement says the config should be, as `sim-cli fit` says it: columns "quantity", "of", "config",
/// "fitted", "standard_error" (NaN where the command line leaves a cell empty), "advice", what to change in the
/// config, and "errors": the fits that found nothing (no resonance in the sweep, a stiffness the moves do not tell),
/// after which the command line goes on with the others and fails at the end. Their rows are missing; an empty list:
/// every fit asked for is there. What keeps any fit from starting (a config refused, a trace unreadable) raises
/// ValueError. `trace_file` (through `map_file` for a vendor export) gives the rigid values per axis, or per motor and link
/// where the motors do not turn in proportion to the axes; `stiffness=True` also the factor on the stiffness at
/// `stiffness_key` (the belts' by default) from the trace's moves; `sweep_file` (a frequency response as `sweep` writes
/// it as CSV, taken on motor `sweep_motor` with `amplitude` N m) the load's resonance on the belt.
#[pyfunction]
#[pyo3(signature = (config, *, trace_file=None, map_file=None, stiffness=false, stiffness_key=None, sweep_file=None, sweep_motor=0, amplitude=None))]
#[allow(clippy::too_many_arguments)]
fn fit(
    py: Python<'_>,
    config: &PyConfig,
    trace_file: Option<PathBuf>,
    map_file: Option<PathBuf>,
    stiffness: bool,
    stiffness_key: Option<String>,
    sweep_file: Option<PathBuf>,
    sweep_motor: usize,
    amplitude: Option<f64>,
) -> PyResult<Py<PyDict>> {
    if trace_file.is_none() && sweep_file.is_none() {
        return Err(PyValueError::new_err("give trace_file or sweep_file"));
    }
    if stiffness && trace_file.is_none() {
        return Err(PyValueError::new_err("stiffness needs trace_file"));
    }
    let (entry, cfg) = (config.entry, config.cfg.clone());
    let (rows, advice, errors) = py
        .detach(|| -> Result<_, String> {
            let tr = texts(entry);
            let refused = |e: ConfigError| tr.tr_arg(e.key(), e.detail()).to_string();
            let traced = |e: TraceError| tr.tr_arg(e.key, &e.detail).to_string();
            let machine = (entry.build)(&cfg).map_err(refused)?;
            let (mut rows, mut advice) = (Rows::<3>::new(), Vec::new());
            let measured = match &trace_file {
                Some(path) => Some(load_trace(&tr, path, map_file.as_deref(), machine.motors())?.trace),
                None => None,
            };
            if let Some(measured) = &measured {
                if !per_axis_fit(machine.as_ref()) {
                    match fit_joints(entry, &cfg, measured).map_err(traced)? {
                        Some(fit) => {
                            for v in &fit.values {
                                rows.push(v.quantity, &v.of, [v.config, v.fitted, v.error]);
                            }
                            if let Some((time, error)) = fit.payload_step {
                                rows.push("payload_step_s", "payload", [f64::NAN, time, error]);
                            }
                            advice.extend(fit.advice().iter().map(|(key, detail)| tr.tr_arg(key, detail).to_string()));
                        }
                        None => advice.push(tr.tr("fit-joints-skipped").to_string()),
                    }
                } else {
                    for (axis, fit) in machine.axes().iter().zip(fit_rigid(entry, &cfg, measured).map_err(traced)?) {
                        let Some(fit) = fit else {
                            advice.push(tr.tr_arg("fit-axis-skipped", axis.id).to_string());
                            continue;
                        };
                        let names =
                            ["moved_mass_kg", "viscous_friction_n_s_m", "coulomb_friction_n", "constant_force_n"];
                        for (q, name) in names.iter().enumerate() {
                            rows.push(name, axis.id, [fit.config[q], fit.fitted[q], fit.error[q]]);
                        }
                        advice
                            .extend(fit.advice(axis.id).iter().map(|(key, detail)| tr.tr_arg(key, detail).to_string()));
                    }
                }
            }
            // A fit that finds nothing is an error beside what the others found, as the command line goes on after it.
            let mut errors = Vec::new();
            if let (true, Some(measured)) = (stiffness, &measured) {
                let key = stiffness_key.as_deref().unwrap_or(BELT_STIFFNESS_KEY);
                match fit_move_stiffness(entry, &cfg, measured, key) {
                    Ok(fit) => {
                        rows.push("stiffness_factor", key, [1.0, fit.factor, fit.error]);
                        rows.push("ringing_rms_nm", key, [f64::NAN, fit.ringing, f64::NAN]);
                        rows.push("ringing_left_rms_nm", key, [f64::NAN, fit.residual, f64::NAN]);
                        let (key, detail) = fit.advice();
                        advice.push(tr.tr_arg(key, &detail).to_string());
                    }
                    Err(e) => errors.push(traced(e)),
                }
            }
            if let Some(path) = &sweep_file {
                let measured = parse_sweep(&read_text(path)?).map_err(traced)?;
                let id = machine.motors().get(sweep_motor).ok_or(format!("sweep_motor {sweep_motor}"))?.id;
                let amplitude = match amplitude {
                    Some(a) => a,
                    None => default_sweep_amplitude(entry, &cfg).map_err(refused)?,
                };
                match fit_sweep(entry, &cfg, sweep_motor, &measured, amplitude).map_err(refused)? {
                    Some(fit) => {
                        let (config, fitted) = (fit.config, fit.fitted);
                        rows.push("resonance_hz", id, [config.frequency, fitted.frequency, fitted.frequency_error]);
                        rows.push("resonance_damping", id, [config.damping, fitted.damping, f64::NAN]);
                        let (key, detail) = fit.advice();
                        advice.push(tr.tr_arg(key, &detail).to_string());
                    }
                    None => errors.push(tr.tr("fit-no-resonance").to_string()),
                }
            }
            Ok((rows, advice, errors))
        })
        .map_err(PyValueError::new_err)?;
    let out = rows.into_dict(py, ["config", "fitted", "standard_error"])?;
    out.set_item("advice", advice)?;
    out.set_item("errors", errors)?;
    Ok(out.unbind())
}

/// One motor's drive train from its own trace, with no machine config, as `sim-cli identify` gives it: columns
/// "quantity", "of", "value", "standard_error", "ci95_low", "ci95_high" (NaN for a fit quality, which has no error),
/// "notes" (what the trace could not tell) and "overlay" ({"time": ..., measured and modelled signals}). Without
/// `map_file` the trace is in the simulator's columns of `config`'s machine. `motor` picks the motor; `from_hz` and
/// `to_hz` also fit the two-mass model to the frequency response from the trace, `sweep_file` to a measured one;
/// `step_from` (and `step_to`) fits the ringing of the speed after a torque step, and with `motor_inertia` (kg m²) the
/// two-mass model from it. "errors" lists the fits that found nothing (no response, no two-mass model, no ringing),
/// whose rows are missing, as `fit` has it.
#[pyfunction]
#[pyo3(signature = (*, trace_file=None, map_file=None, config=None, motor=0, from_hz=None, to_hz=None, step_from=None, step_to=None, motor_inertia=None, sweep_file=None))]
#[allow(clippy::too_many_arguments)]
fn identify(
    py: Python<'_>,
    trace_file: Option<PathBuf>,
    map_file: Option<PathBuf>,
    config: Option<PyConfig>,
    motor: usize,
    from_hz: Option<f64>,
    to_hz: Option<f64>,
    step_from: Option<f64>,
    step_to: Option<f64>,
    motor_inertia: Option<f64>,
    sweep_file: Option<PathBuf>,
) -> PyResult<Py<PyDict>> {
    if trace_file.is_none() && sweep_file.is_none() {
        return Err(PyValueError::new_err("give trace_file or sweep_file"));
    }
    if trace_file.is_some() && map_file.is_none() && config.is_none() {
        return Err(PyValueError::new_err(
            "a trace without map_file is in the columns of config's machine: give config",
        ));
    }
    if from_hz.is_some() != to_hz.is_some() || from_hz.zip(to_hz).is_some_and(|(f, t)| !(f > 0.0 && t > f)) {
        return Err(PyValueError::new_err("give from_hz and to_hz together, 0 < from_hz < to_hz"));
    }
    let entry = config.as_ref().map_or(MACHINES[0], |c| c.entry);
    let (rows, notes, overlay, errors) = py
        .detach(|| -> Result<_, String> {
            let tr = texts(entry);
            let (mut rows, mut notes, mut errors) = (Rows::<4>::new(), Vec::new(), Vec::new());
            let estimate = |rows: &mut Rows<4>, name: &str, of: &str, e: Estimate| {
                let (low, high) = e.interval();
                rows.push(name, of, [e.value, e.error, low, high]);
            };
            let two_mass_rows = |rows: &mut Rows<4>, of: &str, m: &TwoMass| {
                for (name, e) in [
                    ("total_inertia_kg_m2", m.total_inertia),
                    ("motor_side_inertia_kg_m2", m.motor_inertia),
                    ("load_side_inertia_kg_m2", m.load_inertia),
                    ("stiffness_nm_rad", m.stiffness),
                    ("damping_nm_s_rad", m.damping),
                    ("resonance_hz", m.resonance),
                    ("anti_resonance_hz", m.anti_resonance),
                ] {
                    estimate(rows, name, of, e);
                }
            };
            let mut overlay: Vec<(String, Vec<f64>)> = Vec::new();
            if let Some(path) = &trace_file {
                let motors = match (&map_file, &config) {
                    (None, Some(c)) => (c.entry.build)(&c.cfg)
                        .map_err(|e| tr.tr_arg(e.key(), e.detail()).to_string())?
                        .motors()
                        .to_vec(),
                    _ => Vec::new(),
                };
                let imported = load_trace(&tr, path, map_file.as_deref(), &motors)?;
                let (Some(m), Some(id)) = (imported.trace.motors.get(motor), imported.ids.get(motor)) else {
                    return Err(format!("motor {motor}: the trace has {}", imported.trace.motors.len()));
                };
                let t = &imported.trace.time;
                overlay.push(("time".into(), t.clone()));
                let found = Identified::of(t, m, imported.torque_setpoints[motor].as_deref());
                for (name, e) in found.estimates() {
                    estimate(&mut rows, name, id, e);
                }
                for (name, value) in found.qualities() {
                    rows.push(name, id, [value, f64::NAN, f64::NAN, f64::NAN]);
                }
                if found.friction.is_none() {
                    notes.push(tr.tr_arg("error-identify-friction", id).to_string());
                }
                if found.position_loop.is_none()
                    || (imported.torque_setpoints[motor].is_some() && found.current_loop.is_none())
                {
                    notes.push(tr.tr_arg("error-identify-lag", id).to_string());
                }
                if let Some(f) = &found.friction {
                    overlay.push(("torque_measured".into(), m.torque.clone()));
                    overlay.push(("torque_model".into(), f.model.clone()));
                }
                for (name, lag, output) in [
                    ("current_loop", &found.current_loop, &m.torque),
                    ("position_loop", &found.position_loop, &m.angle),
                ] {
                    if let Some(l) = lag {
                        overlay.push((format!("{name}_measured"), output.clone()));
                        overlay.push((format!("{name}_model"), l.model.clone()));
                    }
                }
                if let (Some(from), Some(to)) = (from_hz, to_hz) {
                    match frequency_response(t, &m.torque, &m.speed, &log_spaced(from, to, SWEEP_POINTS)) {
                        Some(response) => {
                            let coherence = response.iter().map(|(_, c)| *c).fold(1.0, f64::min);
                            rows.push("response_lowest_coherence", id, [coherence, f64::NAN, f64::NAN, f64::NAN]);
                            let points: Vec<_> = response.into_iter().map(|(p, _)| p).collect();
                            match two_mass(&points) {
                                Some(model) => two_mass_rows(&mut rows, id, &model),
                                None => errors.push(tr.tr("error-identify-two-mass").to_string()),
                            }
                        }
                        None => errors.push(tr.tr("error-identify-response").to_string()),
                    }
                }
                if let Some(from) = step_from {
                    match ringing(t, &m.speed, from, step_to.unwrap_or(t[t.len() - 1])) {
                        Some(r) => {
                            estimate(&mut rows, "ringing_frequency_hz", id, r.frequency);
                            estimate(&mut rows, "ringing_damping_ratio", id, r.damping_ratio);
                            rows.push("ringing_fit_percent", id, [r.quality.fit_percent, f64::NAN, f64::NAN, f64::NAN]);
                            if let (Some(inertia), Some(total)) =
                                (motor_inertia, found.friction.as_ref().map(|f| f.inertia))
                            {
                                let model = two_mass_from_ringing(&r, total, Estimate { value: inertia, error: 0.0 });
                                two_mass_rows(&mut rows, id, &model);
                            }
                        }
                        None => errors.push(tr.tr("error-identify-ringing").to_string()),
                    }
                }
            }
            if let Some(path) = &sweep_file {
                let points = parse_sweep(&read_text(path)?).map_err(|e| tr.tr_arg(e.key, &e.detail).to_string())?;
                match two_mass(&points) {
                    Some(model) => two_mass_rows(&mut rows, "sweep", &model),
                    None => errors.push(tr.tr("error-identify-two-mass").to_string()),
                }
            }
            Ok((rows, notes, overlay, errors))
        })
        .map_err(PyValueError::new_err)?;
    let out = rows.into_dict(py, ["value", "standard_error", "ci95_low", "ci95_high"])?;
    out.set_item("notes", notes)?;
    out.set_item("errors", errors)?;
    let signals = PyDict::new(py);
    for (name, values) in overlay {
        signals.set_item(name, PyArray1::from_vec(py, values))?;
    }
    out.set_item("overlay", signals)?;
    Ok(out.unbind())
}

/// The machine of the config, built: refused as ValueError.
fn built(config: &PyConfig) -> PyResult<Box<dyn sim_core::Machine>> {
    (config.entry.build)(&config.cfg).map_err(|e| config_error(config.entry, e))
}

/// `axis_index` checked against the machine's axes, as `sim-cli --axis-index` checks it.
fn axis(config: &PyConfig, axis_index: usize) -> PyResult<usize> {
    let count = built(config)?.axes().len();
    if axis_index < count {
        Ok(axis_index)
    } else {
        Err(PyValueError::new_err(format!("axis_index {axis_index}: the machine has {count} axes")))
    }
}

fn column<'py>(py: Python<'py>, values: impl Iterator<Item = f64>) -> Bound<'py, PyArray1<f64>> {
    PyArray1::from_vec(py, values.collect())
}

/// The answer of axis `axis_index` to a step of `size` (m or rad) of its position setpoint, over `time` seconds, as
/// `sim-cli step` gives it: columns "time_s", "setpoint", "response" (the tool position relative to where it was),
/// with "rise_time_s", "overshoot" and "settling_time_s" as numbers.
#[pyfunction]
#[pyo3(signature = (config, *, axis_index=0, size=0.001, time=0.5))]
fn step(py: Python<'_>, config: &PyConfig, axis_index: usize, size: f64, time: f64) -> PyResult<Py<PyDict>> {
    if !(size.is_finite() && size != 0.0 && time.is_finite() && time > 0.0) {
        return Err(PyValueError::new_err("size must be finite and not 0, time finite and above 0"));
    }
    let (entry, cfg, i) = (config.entry, config.cfg.clone(), axis(config, axis_index)?);
    let r = py
        .detach(|| sim_components::analysis::step_response(entry, &cfg, i, size, time))
        .map_err(|e| config_error(entry, e))?;
    let out = PyDict::new(py);
    out.set_item("time_s", column(py, r.trace.iter().map(|p| p.0)))?;
    out.set_item("setpoint", column(py, r.trace.iter().map(|_| r.size)))?;
    out.set_item("response", column(py, r.trace.iter().map(|p| p.1)))?;
    out.set_item("rise_time_s", r.rise_time)?;
    out.set_item("overshoot", r.overshoot)?;
    out.set_item("settling_time_s", r.settling_time)?;
    Ok(out.unbind())
}

/// The closed-loop frequency response of the position loop of axis `axis_index`, as `sim-cli bode` gives it: columns
/// "frequency_hz", "gain", "gain_db", "phase_deg" at `count` frequencies from `from_hz` to `to_hz`, spaced
/// logarithmically, with a setpoint amplitude of `amplitude` (m or rad).
#[pyfunction]
#[pyo3(signature = (config, *, axis_index=0, from_hz=1.0, to_hz=200.0, count=40, amplitude=0.0005))]
fn bode(
    py: Python<'_>,
    config: &PyConfig,
    axis_index: usize,
    from_hz: f64,
    to_hz: f64,
    count: usize,
    amplitude: f64,
) -> PyResult<Py<PyDict>> {
    if !(from_hz > 0.0 && to_hz > from_hz && to_hz.is_finite()) || count < 2 || !amplitude.is_finite() {
        return Err(PyValueError::new_err("from_hz must be above 0 and below to_hz, count at least 2"));
    }
    let (entry, cfg, i) = (config.entry, config.cfg.clone(), axis(config, axis_index)?);
    let points = py
        .detach(|| {
            sim_components::analysis::closed_loop_response(
                entry,
                &cfg,
                i,
                &log_spaced(from_hz, to_hz, count),
                amplitude,
            )
        })
        .map_err(|e| config_error(entry, e))?;
    let out = PyDict::new(py);
    out.set_item("frequency_hz", column(py, points.iter().map(|p| p.frequency)))?;
    out.set_item("gain", column(py, points.iter().map(|p| p.gain)))?;
    out.set_item("gain_db", column(py, points.iter().map(|p| 20.0 * p.gain.log10())))?;
    out.set_item("phase_deg", column(py, points.iter().map(|p| p.phase_deg)))?;
    Ok(out.unbind())
}

/// One row per axis with what its step response (a step of `size`) and its closed-loop frequency response (setpoint
/// amplitude `amplitude`) say, as `sim-cli tuning` gives it: columns "axis", "rise_time_s", "overshoot",
/// "settling_time_s", "bandwidth_hz" (NaN where the response stays above -3 dB up to 200 Hz).
#[pyfunction]
#[pyo3(signature = (config, *, size=0.001, amplitude=0.0005))]
fn tuning(py: Python<'_>, config: &PyConfig, size: f64, amplitude: f64) -> PyResult<Py<PyDict>> {
    use sim_components::analysis::{bandwidth, closed_loop_response, step_response};
    if !(size.is_finite() && size != 0.0 && amplitude.is_finite()) {
        return Err(PyValueError::new_err("size must be finite and not 0, amplitude finite"));
    }
    let (entry, cfg) = (config.entry, config.cfg.clone());
    let axes: Vec<&'static str> = built(config)?.axes().iter().map(|a| a.id).collect();
    let rows = py
        .detach(|| -> Result<Vec<[f64; 4]>, ConfigError> {
            // As the command line: a batch of frequencies per processor, lowest first, up to the batch with the
            // first point below -3 dB.
            let batch = std::thread::available_parallelism().map_or(1, |n| n.get());
            let mut rows = Vec::new();
            for i in 0..axes.len() {
                let step = step_response(entry, &cfg, i, size, 0.5)?;
                let mut response = Vec::new();
                for frequencies in log_spaced(1.0, 200.0, 30).chunks(batch) {
                    response.extend(closed_loop_response(entry, &cfg, i, frequencies, amplitude)?);
                    if bandwidth(&response).is_some() {
                        break;
                    }
                }
                let band = bandwidth(&response).unwrap_or(f64::NAN);
                rows.push([step.rise_time, step.overshoot, step.settling_time, band]);
            }
            Ok(rows)
        })
        .map_err(|e| config_error(entry, e))?;
    let out = PyDict::new(py);
    out.set_item("axis", axes)?;
    for (k, name) in ["rise_time_s", "overshoot", "settling_time_s", "bandwidth_hz"].iter().enumerate() {
        out.set_item(*name, column(py, rows.iter().map(|r| r[k])))?;
    }
    Ok(out.unbind())
}

/// Per motor the stability margins of its velocity loop, as `sim-cli margins` gives them: columns "run" ("nominal",
/// and with `bands=True` each value of `[bounds]` at its low and its high bound), "motor", "crossover_hz",
/// "phase_margin_deg", "gain_margin_db", "peak_sensitivity", "peak_frequency_hz" and "ok" (false: the loop is unstable
/// or its measurement not valid).
#[pyfunction]
#[pyo3(signature = (config, *, bands=false))]
fn margins(py: Python<'_>, config: &PyConfig, bands: bool) -> PyResult<Py<PyDict>> {
    let (entry, cfg) = (config.entry, config.cfg.clone());
    let motors: Vec<&'static str> = built(config)?.motors().iter().map(|m| m.id).collect();
    let runs = py
        .detach(|| {
            let bounded = if bands { bounds(&cfg)? } else { Vec::new() };
            sim_components::analysis::velocity_loop_margins(entry, &cfg, &bounded)
        })
        .map_err(|e| config_error(entry, e))?;
    let rows: Vec<(&str, &str, &sim_components::analysis::LoopMargins)> = runs
        .iter()
        .flat_map(|(run, margins)| motors.iter().zip(margins).map(move |(motor, m)| (run.as_str(), *motor, m)))
        .collect();
    let out = PyDict::new(py);
    out.set_item("run", rows.iter().map(|r| r.0).collect::<Vec<_>>())?;
    out.set_item("motor", rows.iter().map(|r| r.1).collect::<Vec<_>>())?;
    out.set_item("crossover_hz", column(py, rows.iter().map(|r| r.2.crossover_hz)))?;
    out.set_item("phase_margin_deg", column(py, rows.iter().map(|r| r.2.phase_margin_deg)))?;
    out.set_item("gain_margin_db", column(py, rows.iter().map(|r| r.2.gain_margin_db)))?;
    out.set_item("peak_sensitivity", column(py, rows.iter().map(|r| r.2.peak_sensitivity)))?;
    out.set_item("peak_frequency_hz", column(py, rows.iter().map(|r| r.2.peak_frequency_hz)))?;
    out.set_item("ok", PyArray1::from_vec(py, rows.iter().map(|r| r.2.stable()).collect()))?;
    Ok(out.unbind())
}

/// The modes the machine reports over the stroke of axis `axis_index` at `points` places, the other axes at the start
/// pose, as `sim-cli modes` gives them: "position", then per mode, lowest first, "mode<i>_hz" and "mode<i>_damping"
/// (NaN where a place does not have it).
#[pyfunction]
#[pyo3(signature = (config, *, axis_index=0, points=21))]
fn modes(py: Python<'_>, config: &PyConfig, axis_index: usize, points: usize) -> PyResult<Py<PyDict>> {
    if points < 2 {
        return Err(PyValueError::new_err("points must be at least 2"));
    }
    let (entry, cfg, i) = (config.entry, config.cfg.clone(), axis(config, axis_index)?);
    let rows = py
        .detach(|| -> Result<_, ConfigError> {
            let sim = Sim::new(entry, &cfg)?;
            Ok(sim_components::analysis::modes_along(sim.machine.as_ref(), &cfg.f64s("start.pose")?, i, points))
        })
        .map_err(|e| config_error(entry, e))?;
    let count = rows.iter().map(|(_, modes)| modes.len()).max().unwrap_or(0);
    let out = PyDict::new(py);
    out.set_item("position", column(py, rows.iter().map(|r| r.0)))?;
    for k in 0..count {
        let at = |f: fn(&sim_core::Mode) -> f64| column(py, rows.iter().map(move |r| r.1.get(k).map_or(f64::NAN, f)));
        out.set_item(format!("mode{}_hz", k + 1), at(|m| m.hz))?;
        out.set_item(format!("mode{}_damping", k + 1), at(|m| m.damping))?;
    }
    Ok(out.unbind())
}

/// The cycle with each parameter at its low and at its high bound of the config, or `delta` lower and higher where it
/// has none, one at a time, strongest first, as `sim-cli sensitivity` gives it: columns "parameter", "value", "low",
/// "high", the relative changes "peak_torque_low", "peak_torque_high", "rms_torque_low", "rms_torque_high",
/// "cycle_time_low", "cycle_time_high", and "belt_tension_min_low_n", "belt_tension_min_high_n". `groups` limits it to
/// those config tables; without it every table of machine and motor data is varied.
#[pyfunction]
#[pyo3(signature = (config, *, delta=0.2, groups=None))]
fn sensitivity(py: Python<'_>, config: &PyConfig, delta: f64, groups: Option<Vec<String>>) -> PyResult<Py<PyDict>> {
    if !(delta.is_finite() && delta > 0.0) {
        return Err(PyValueError::new_err("delta must be finite and above 0"));
    }
    let (entry, cfg) = (config.entry, config.cfg.clone());
    let groups = groups.unwrap_or_default();
    // As the command line: timing, switches, limits, gains and the cycle are choices, not uncertain machine data.
    let fixed = ["sim", "effects", "drive", "profile", "start", "axis", "temperature"];
    let paths: Vec<String> = cfg
        .leaves()
        .into_iter()
        .map(|(path, _)| path)
        .filter(|path| path.contains('.'))
        .filter(|path| {
            let group = path.split('.').next().unwrap_or("");
            if groups.is_empty() { !fixed.contains(&group) } else { groups.iter().any(|g| g == group) }
        })
        .collect();
    let (nominal, mut rows) = py
        .detach(|| sim_components::analysis::sensitivity(entry, &cfg, &paths, delta))
        .map_err(|e| config_error(entry, e))?;
    rows.sort_by(|a, b| b.strength(&nominal).total_cmp(&a.strength(&nominal)));
    let changes: Vec<[f64; 6]> = rows.iter().map(|r| r.changes(&nominal)).collect();
    let slack = |load: &CycleLoad| load.belt_tension.map_or(0.0, |t| t.0);
    let out = PyDict::new(py);
    out.set_item("parameter", rows.iter().map(|r| r.path.clone()).collect::<Vec<_>>())?;
    out.set_item("value", column(py, rows.iter().map(|r| r.value)))?;
    out.set_item("low", column(py, rows.iter().map(|r| r.bounds.0)))?;
    out.set_item("high", column(py, rows.iter().map(|r| r.bounds.1)))?;
    let names = ["peak_torque_low", "peak_torque_high", "rms_torque_low", "rms_torque_high", "cycle_time_low"];
    for (k, name) in names.iter().chain(&["cycle_time_high"]).enumerate() {
        out.set_item(*name, column(py, changes.iter().map(|c| c[k])))?;
    }
    out.set_item("belt_tension_min_low_n", column(py, rows.iter().map(|r| slack(&r.low))))?;
    out.set_item("belt_tension_min_high_n", column(py, rows.iter().map(|r| slack(&r.high))))?;
    Ok(out.unbind())
}

/// The position setpoints of a drive trace run through the simulator, as `sim-cli replay` runs them: per motor how far
/// simulated torque and following error are from the measured ones (columns "motor", "torque_rms_nm",
/// "torque_peak_nm", "following_error_rms_rad", "following_error_peak_rad"), and "overlay": measured and simulated
/// torque, speed and following error side by side ({"time": ..., "motor.<id>.<quantity>_measured": ..., ...}).
/// `map_file` maps a vendor export, as for `fit`.
#[pyfunction]
#[pyo3(signature = (config, *, trace_file, map_file=None))]
fn replay(py: Python<'_>, config: &PyConfig, trace_file: PathBuf, map_file: Option<PathBuf>) -> PyResult<Py<PyDict>> {
    let (entry, cfg) = (config.entry, config.cfg.clone());
    let (ids, measured, simulated) = py
        .detach(|| -> Result<_, String> {
            let tr = texts(entry);
            let machine = (entry.build)(&cfg).map_err(|e| tr.tr_arg(e.key(), e.detail()).to_string())?;
            let measured = load_trace(&tr, &trace_file, map_file.as_deref(), machine.motors())?.trace;
            let simulated = sim_components::trace::replay(entry, &cfg, &measured)
                .map_err(|e| tr.tr_arg(e.key, &e.detail).to_string())?;
            let ids: Vec<&'static str> = machine.motors().iter().map(|m| m.id).collect();
            Ok((ids, measured, simulated))
        })
        .map_err(PyValueError::new_err)?;
    let found = sim_components::trace::differences(&measured, &simulated);
    let out = PyDict::new(py);
    out.set_item("motor", ids.clone())?;
    let names = ["torque_rms_nm", "torque_peak_nm", "following_error_rms_rad", "following_error_peak_rad"];
    for (k, name) in names.iter().enumerate() {
        out.set_item(*name, column(py, found.iter().map(|d| d[k])))?;
    }
    let overlay = PyDict::new(py);
    overlay.set_item("time", PyArray1::from_vec(py, measured.time.clone()))?;
    for ((id, m), s) in ids.iter().zip(&measured.motors).zip(&simulated.motors) {
        let (m_error, s_error) = (m.following_error(), s.following_error());
        for (name, a, b) in
            [("torque", &m.torque, &s.torque), ("speed", &m.speed, &s.speed), ("following_error", &m_error, &s_error)]
        {
            overlay.set_item(format!("motor.{id}.{name}_measured"), PyArray1::from_vec(py, a.clone()))?;
            overlay.set_item(format!("motor.{id}.{name}_simulated"), PyArray1::from_vec(py, b.clone()))?;
        }
    }
    out.set_item("overlay", overlay)?;
    Ok(out.unbind())
}

/// The cycle run with `effects.<effect>` on and with it off, as `sim-cli compare` runs it: "time" and per signal
/// "<id>.on" and "<id>.off", once per millisecond of the measured pass (every signal, or those `signals` names), and
/// "differences": columns "signal", "unit", "peak_difference", "rms_difference" of on less off.
#[pyfunction]
#[pyo3(signature = (config, *, effect, signals=None))]
fn compare(py: Python<'_>, config: &PyConfig, effect: &str, signals: Option<Vec<String>>) -> PyResult<Py<PyDict>> {
    let (entry, cfg) = (config.entry, config.cfg.clone());
    let found = py
        .detach(|| sim_components::analysis::compare_effect(entry, &cfg, effect))
        .map_err(|e| config_error(entry, e))?;
    // Every signal but the time, which is the first column anyway.
    let chosen: Vec<usize> = match signals {
        None => (1..found.signals.len()).collect(),
        Some(ids) => ids
            .iter()
            .map(|id| found.signals.iter().position(|s| &s.id == id).ok_or_else(|| PyKeyError::new_err(id.clone())))
            .collect::<PyResult<_>>()?,
    };
    let out = PyDict::new(py);
    out.set_item("time", column(py, found.on.iter().map(|row| row[0])))?;
    for &i in &chosen {
        out.set_item(format!("{}.on", found.signals[i].id), column(py, found.on.iter().map(|row| row[i])))?;
        out.set_item(format!("{}.off", found.signals[i].id), column(py, found.off.iter().map(|row| row[i])))?;
    }
    let differences = PyDict::new(py);
    let apart: Vec<(f64, f64)> = chosen.iter().map(|&i| found.difference(i)).collect();
    differences.set_item("signal", chosen.iter().map(|&i| found.signals[i].id.clone()).collect::<Vec<_>>())?;
    differences.set_item("unit", chosen.iter().map(|&i| found.signals[i].unit.clone()).collect::<Vec<_>>())?;
    differences.set_item("peak_difference", column(py, apart.iter().map(|d| d.0)))?;
    differences.set_item("rms_difference", column(py, apart.iter().map(|d| d.1)))?;
    out.set_item("differences", differences)?;
    Ok(out.unbind())
}

/// The sizing report a customer signs, written to `path` as `sim-cli report` writes it: PDF for a path ending in .pdf,
/// HTML otherwise, in `lang`; `bands=True` adds the error bands. True where every check passed.
#[pyfunction]
#[pyo3(signature = (config, path, *, bands=false, lang="en"))]
fn report(py: Python<'_>, config: &PyConfig, path: PathBuf, bands: bool, lang: &str) -> PyResult<bool> {
    if !LANGUAGES.contains(&lang) {
        return Err(PyValueError::new_err(format!("unknown language: {lang}")));
    }
    let (entry, config) = (config.entry, config.clone());
    let lang = lang.to_string();
    py.detach(|| -> Result<bool, String> {
        let tr = translations(&lang, entry);
        let source = config.source.clone().unwrap_or_else(|| tr.tr("report-default-config").to_string());
        let report = sim_components::report::make(entry, &config.text, &source, &config.base, &config.cfg, bands, &tr)
            .map_err(|e| tr.tr_arg(e.key(), e.detail()).to_string())?;
        let pdf = path.extension().is_some_and(|e| e.eq_ignore_ascii_case("pdf"));
        let bytes = if pdf { report.pdf() } else { report.html(&lang).into_bytes() };
        std::fs::write(&path, bytes).map_err(|e| format!("{}: {e}", path.display()))?;
        Ok(report.passed)
    })
    .map_err(PyValueError::new_err)
}

/// What keeps the dataset folder `path` from being published (datasets/README.md), as `sim-cli check-dataset` lists it:
/// one text per problem, none for a folder that is ready.
#[pyfunction]
fn check_dataset(py: Python<'_>, path: PathBuf) -> Vec<String> {
    py.detach(|| sim_components::dataset::check(&path))
}

/// The machine with its config as an FMI 2.0 (`fmi=2`) or 3.0 (`fmi=3`) Co-Simulation FMU written to `path`, as
/// `sim-cli fmu` writes it, with the descriptions in `lang`. `binaries` are the sim-fmi libraries it carries (a .dll,
/// .so or .dylib each, one per platform), which Python has none of beside it: build them with
/// `cargo build -p sim-fmi --release`.
#[pyfunction]
#[pyo3(signature = (config, path, *, binaries, fmi=2, lang="en"))]
fn fmu(config: &PyConfig, path: PathBuf, binaries: Vec<PathBuf>, fmi: u8, lang: &str) -> PyResult<()> {
    if fmi != 2 && fmi != 3 {
        return Err(PyValueError::new_err(format!("fmi {fmi}: 2 or 3")));
    }
    if !LANGUAGES.contains(&lang) {
        return Err(PyValueError::new_err(format!("unknown language: {lang}")));
    }
    if binaries.is_empty() {
        return Err(PyValueError::new_err("give the sim-fmi binaries the FMU carries"));
    }
    let entry = config.entry;
    let text = config.cfg.to_text().map_err(|e| config_error(entry, e))?;
    let model = sim_fmi::Model::with_fmi(entry, &text, lang, fmi).map_err(|e| config_error(entry, e))?;
    let mut files = model.files();
    for binary in &binaries {
        let name = binary.file_name().and_then(|n| n.to_str()).unwrap_or("");
        let bytes = std::fs::read(binary).map_err(|e| PyValueError::new_err(format!("{}: {e}", binary.display())))?;
        let inside = if fmi == 3 { sim_fmi::binary_path3(name, &bytes) } else { sim_fmi::binary_path(name) };
        let inside =
            inside.ok_or_else(|| PyValueError::new_err(format!("not a sim-fmi binary: {}", binary.display())))?;
        // Two binaries for one platform would be two entries of one name in the archive.
        if files.iter().any(|(p, _)| *p == inside) {
            return Err(PyValueError::new_err(format!("two binaries for {inside}")));
        }
        files.push((inside, bytes));
    }
    std::fs::write(&path, sim_fmi::zip(&files)).map_err(|e| PyValueError::new_err(format!("{}: {e}", path.display())))
}

#[pymodule]
fn kinematics_sim(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyConfig>()?;
    m.add_function(wrap_pyfunction!(machines, m)?)?;
    m.add_function(wrap_pyfunction!(signals, m)?)?;
    m.add_function(wrap_pyfunction!(run, m)?)?;
    m.add_function(wrap_pyfunction!(analyze, m)?)?;
    m.add_function(wrap_pyfunction!(verdicts, m)?)?;
    m.add_function(wrap_pyfunction!(sizing, m)?)?;
    m.add_function(wrap_pyfunction!(sweep, m)?)?;
    m.add_function(wrap_pyfunction!(optimise, m)?)?;
    m.add_function(wrap_pyfunction!(fit, m)?)?;
    m.add_function(wrap_pyfunction!(identify, m)?)?;
    m.add_function(wrap_pyfunction!(step, m)?)?;
    m.add_function(wrap_pyfunction!(bode, m)?)?;
    m.add_function(wrap_pyfunction!(tuning, m)?)?;
    m.add_function(wrap_pyfunction!(margins, m)?)?;
    m.add_function(wrap_pyfunction!(modes, m)?)?;
    m.add_function(wrap_pyfunction!(sensitivity, m)?)?;
    m.add_function(wrap_pyfunction!(replay, m)?)?;
    m.add_function(wrap_pyfunction!(compare, m)?)?;
    m.add_function(wrap_pyfunction!(report, m)?)?;
    m.add_function(wrap_pyfunction!(check_dataset, m)?)?;
    m.add_function(wrap_pyfunction!(fmu, m)?)?;
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
