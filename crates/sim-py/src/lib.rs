//! The Python module `kinematics_sim`: the simulator's machines, configs, runs, analyses, verdicts with their bands
//! and the catalogue sizing, as `sim-cli` has them, for notebooks and scripts. Results come as NumPy arrays and as
//! dicts of equal-length columns, which `pandas.DataFrame(result)` takes as they are. The type stubs are in
//! `kinematics_sim.pyi`; `docs/python.md` has the guide.
//!
//! Every long call releases the GIL, so Python threads can run several at once.

use numpy::{PyArray1, PyArray2};
use pyo3::exceptions::{PyKeyError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBool, PyDict, PyList, PyString};
use sim_components::analysis::{
    CycleLoad, analyse_cycle_with, analyse_trajectory_with, bounds, verdict_bands, verdict_bands_sampled,
};
use sim_components::{Command, Sim, load_config};
use sim_core::{ConfigError, MachineEntry, Translations};
use sim_machines::{LANGUAGES, MACHINES, find, translations};

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
    /// The default config of `machine`, or the config in `text` (TOML), or in the file at `path`.
    #[new]
    #[pyo3(signature = (machine, text=None, path=None))]
    fn new(machine: &str, text: Option<&str>, path: Option<std::path::PathBuf>) -> PyResult<Self> {
        let entry = self::machine(machine)?;
        let text = match (text, path) {
            (Some(_), Some(_)) => return Err(PyValueError::new_err("give text or path, not both")),
            (Some(text), None) => Some(text.to_string()),
            (None, Some(path)) => Some(
                std::fs::read_to_string(&path)
                    .map_err(|e| PyValueError::new_err(format!("{}: {e}", path.display())))?,
            ),
            (None, None) => None,
        };
        let cfg = match text {
            Some(text) => load_config(entry, &text).map_err(|e| config_error(entry, e))?,
            None => entry.config(),
        };
        let mut out = Self { entry, base: cfg.clone(), sets: Vec::new(), cfg };
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
    // through `__index__`); a NumPy array takes none of them and becomes an array below.
    Ok(if let Ok(b) = value.extract::<bool>() {
        Value::Boolean(b)
    } else if let Ok(i) = value.extract::<i64>() {
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
/// at once (seed `seed`), which also adds "pass_share".
#[pyfunction]
#[pyo3(signature = (config, *, bands=false, samples=None, seed=1))]
fn verdicts(py: Python<'_>, config: &PyConfig, bands: bool, samples: Option<usize>, seed: u64) -> PyResult<Py<PyDict>> {
    if samples.is_some() && !bands {
        return Err(PyValueError::new_err("samples needs bands=True"));
    }
    if samples == Some(0) {
        return Err(PyValueError::new_err("samples must be at least 1"));
    }
    let (entry, cfg) = (config.entry, config.cfg.clone());
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
/// `series` keeps article numbers starting with it, `supply` is "230v", "400v" or "s120", `gearboxes` names a gearbox
/// series to try as well, and `axes=True` tries every axis of the catalogue too.
#[pyfunction]
#[pyo3(signature = (config, *, supply="400v", series="", gearboxes=None, axes=false))]
fn sizing(
    py: Python<'_>,
    config: &PyConfig,
    supply: &str,
    series: &str,
    gearboxes: Option<&str>,
    axes: bool,
) -> PyResult<Py<PyDict>> {
    let (entry, cfg) = (config.entry, config.cfg.clone());
    let found = py.detach(|| {
        let wanted = |article: &str| article.starts_with(series);
        if axes {
            sim_components::catalog::search_axes(entry, &cfg, supply, &wanted, gearboxes)
        } else {
            sim_components::catalog::search_motors(entry, &cfg, supply, &wanted, gearboxes)
        }
    });
    let out = PyDict::new(py);
    let numbers =
        |f: &dyn Fn(&sim_components::catalog::Candidate) -> f64| PyArray1::from_vec(py, found.iter().map(f).collect());
    out.set_item("axis", found.iter().map(|c| c.axis.clone()).collect::<Vec<_>>())?;
    out.set_item("motor", found.iter().map(|c| c.article).collect::<Vec<_>>())?;
    out.set_item("gearbox", found.iter().map(|c| c.gearbox.clone()).collect::<Vec<_>>())?;
    out.set_item("drive", found.iter().map(|c| c.drive).collect::<Vec<_>>())?;
    out.set_item("mass_kg", numbers(&|c| c.mass))?;
    out.set_item("cost_kg", numbers(&|c| c.cost))?;
    out.set_item("passes", PyArray1::from_vec(py, found.iter().map(|c| c.passes()).collect()))?;
    out.set_item("tightest_check", found.iter().map(|c| c.tightest().map_or(c.refused, |t| t.0)).collect::<Vec<_>>())?;
    out.set_item("tightest_utilisation", numbers(&|c| c.tightest().map_or(f64::NAN, |t| t.1)))?;
    let peak = |c: &sim_components::catalog::Candidate| {
        c.result.as_ref().map_or(f64::NAN, |(load, _)| load.motors.iter().map(|m| m.peak_torque).fold(0.0, f64::max))
    };
    out.set_item("peak_torque_nm", numbers(&peak))?;
    Ok(out.unbind())
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
    m.add("__version__", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}
