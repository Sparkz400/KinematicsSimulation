//! The app's side of the views of what the machine is (`sim_components::shapes`): what the workspace map and the path
//! in the 3D view are coloured by, the path's values, and the cycle run with its bounded values at their bounds, kept
//! as traces. Nothing here draws.

pub use sim_components::shapes::*;
use sim_core::{Config, ConfigError, MachineEntry};

/// What the workspace map colours each place by.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum WorkspaceColour {
    /// The condition number of the kinematics (`Machine::condition`).
    #[default]
    Condition,
    /// The largest tool acceleration in the worst direction (`worst_acceleration`).
    Acceleration,
}

/// What the path in the 3D view is coloured by along its length (`Model::path_colours`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PathQuantity {
    /// The tool's distance to the path, as the path's analysis measured it (`Run::off_path`), m.
    PathError,
    /// The tool's distance to where its setpoint is at the same moment (the axes' following errors in m), m.
    TrackingError,
    /// The tool's speed over the axes in m, m/s.
    Speed,
    /// Motor k's torque over what its drive gives at that speed (`charts::torque_utilisation`), 1 at the curve.
    Utilisation(usize),
    /// The highest tension of the machine's belt sections (`Machine::belt_sections`), N.
    BeltTension,
}

impl PathQuantity {
    /// Its label's translation key.
    pub fn key(self) -> &'static str {
        match self {
            Self::PathError => "ui-colour-path-error",
            Self::TrackingError => "ui-colour-tracking-error",
            Self::Speed => "ui-colour-speed",
            Self::Utilisation(_) => "ui-colour-utilisation",
            Self::BeltTension => "ui-colour-belt-tension",
        }
    }

    /// The unit its values are in.
    pub fn unit(self) -> &'static str {
        match self {
            Self::PathError | Self::TrackingError => "m",
            Self::Speed => "m/s",
            Self::Utilisation(_) => "",
            Self::BeltTension => "N",
        }
    }

    /// Its values over `run`, one per sample, where the run has what it needs: `belts` are the ids of the machine's
    /// belt sections, whose tension signals are `<id>.tension`.
    pub fn values(self, run: &crate::runs::Run, belts: &[&str]) -> Option<Vec<f64>> {
        use crate::runs::{Channel, Quantity};
        let time = run.time();
        let length = |columns: &[&[f64]]| -> Vec<f64> {
            (0..time.len()).map(|i| columns.iter().map(|c| c[i] * c[i]).sum::<f64>().sqrt()).collect()
        };
        match self {
            Self::PathError => {
                let off = &run.off_path;
                (!off.is_empty()).then(|| time.iter().map(|&t| off[nearest(off, t)][1]).collect())
            }
            Self::TrackingError => run.channel(Channel { quantity: Quantity::ToolError, motor: 0 }).map(|c| c.to_vec()),
            Self::Speed => {
                let speeds: Vec<&[f64]> = run
                    .signals
                    .iter()
                    .zip(&run.columns)
                    .filter(|(s, _)| s.id.starts_with("axis.") && s.id.ends_with(".velocity") && s.unit == "m/s")
                    .map(|(_, c)| c.as_slice())
                    .collect();
                (!speeds.is_empty()).then(|| length(&speeds))
            }
            Self::Utilisation(k) => {
                let (drive, id) = (run.drives.get(k)?, run.motors.get(k)?);
                let speed = run.column(&format!("motor.{id}.speed"))?;
                let torque = run.column(&format!("motor.{id}.torque"))?;
                Some(
                    speed
                        .iter()
                        .zip(torque)
                        .map(|(w, t)| sim_components::charts::torque_utilisation(drive, *w, *t))
                        .collect(),
                )
            }
            Self::BeltTension => {
                let tensions: Vec<&[f64]> =
                    belts.iter().filter_map(|id| run.column(&format!("{id}.tension"))).collect();
                (!tensions.is_empty()).then(|| {
                    (0..time.len()).map(|i| tensions.iter().map(|c| c[i]).fold(f64::NEG_INFINITY, f64::max)).collect()
                })
            }
        }
    }
}

/// The place in `series` (sorted by its first entry, the time) whose time is nearest `t`.
fn nearest(series: &[[f64; 3]], t: f64) -> usize {
    let after = series.partition_point(|p| p[0] < t).min(series.len() - 1);
    if after > 0 && (t - series[after - 1][0]).abs() <= (series[after][0] - t).abs() { after - 1 } else { after }
}

/// The path that was run in the 3D view, coloured by one quantity (`Model::path_colours`): per sample of the run the
/// tool's place in the scene, the time and the value; and the sample with the highest value.
#[derive(Debug, Clone)]
pub struct PathColours {
    pub quantity: PathQuantity,
    pub points: Vec<[f64; 3]>,
    pub time: Vec<f64>,
    pub values: Vec<f64>,
    pub worst: usize,
}

/// One traced signal of the cycle with the bounds runs' envelope (`BandRuns`): its signal, the time from the measured
/// pass's start, the nominal trace and, at each sample, the lowest and highest value of the bounds runs.
#[derive(Debug, Clone)]
pub struct BandTrace {
    pub signal: sim_core::SignalInfo,
    pub time: Vec<f64>,
    pub nominal: Vec<f64>,
    pub low: Vec<f64>,
    pub high: Vec<f64>,
}

/// The cycle run with each bounded value at its low and at its high bound, one at a time, as `verdict_bands` runs it,
/// kept as traces: each motor's torque and following error and each belt section's tension, nominal and enveloped; and
/// per motor the operating points (speed in rad/s, torque) of the bounds runs, every other millisecond.
#[derive(Debug, Clone)]
pub struct BandRuns {
    pub runs: usize,
    pub traces: Vec<BandTrace>,
    pub points: Vec<Vec<[f64; 2]>>,
}

/// The configs of the bounds runs: `config` with its parts in and each bounded value (`analysis::bounds` of that) at
/// its low and its high bound, one at a time. A bound whose low is its high moves nothing and runs nothing.
pub fn bound_configs(entry: &MachineEntry, config: &Config) -> Result<Vec<Config>, ConfigError> {
    let resolved = sim_components::catalog::resolve(entry, config)?;
    let mut out = Vec::new();
    for (path, low, high) in sim_components::analysis::bounds(&resolved)? {
        for value in if low == high { vec![] } else { vec![low, high] } {
            let mut changed = resolved.clone();
            sim_components::analysis::set_bounded(&mut changed, &path, value)?;
            out.push(changed);
        }
    }
    Ok(out)
}

/// The bounds runs of `config`'s cycle (`BandRuns`), each recorded as `analysis::record_cycle` records the cycle, on all
/// processors.
pub fn band_runs(entry: &MachineEntry, config: &Config) -> Result<BandRuns, ConfigError> {
    use sim_components::analysis::record_cycle;
    let configs = bound_configs(entry, config)?;
    let machine = build(entry, config)?;
    let signals = sim_components::Sim::new(entry, config)?.signal_infos();
    let mut ids: Vec<String> = Vec::new();
    for motor in machine.motors() {
        ids.extend(["torque", "following_error"].map(|q| format!("motor.{}.{q}", motor.id)));
    }
    ids.extend(machine.belt_sections().iter().map(|b| format!("{}.tension", b.id)));
    let traced: Vec<usize> = ids.iter().filter_map(|id| signals.iter().position(|s| s.id == *id)).collect();
    let motor_columns: Vec<Option<(usize, usize)>> = machine
        .motors()
        .iter()
        .map(|m| {
            let at = |q: &str| signals.iter().position(|s| s.id == format!("motor.{}.{q}", m.id));
            Some((at("speed")?, at("torque")?))
        })
        .collect();
    let nominal = record_cycle(entry, config)?;
    let mut traces: Vec<BandTrace> = traced
        .iter()
        .map(|&i| BandTrace {
            signal: signals[i].clone(),
            time: nominal.iter().map(|row| row[0]).collect(),
            nominal: nominal.iter().map(|row| row[i]).collect(),
            low: Vec::new(),
            high: Vec::new(),
        })
        .collect();
    let mut points = vec![Vec::new(); motor_columns.len()];
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()).max(1);
    for chunk in configs.chunks(threads) {
        let recorded: Vec<Result<Vec<Vec<f64>>, ConfigError>> = std::thread::scope(|scope| {
            let runs: Vec<_> = chunk.iter().map(|cfg| scope.spawn(move || record_cycle(entry, cfg))).collect();
            runs.into_iter()
                .map(|run| run.join().unwrap_or_else(|_| Err(ConfigError::Invalid("bounds".into()))))
                .collect()
        });
        for rows in recorded {
            let rows = rows?;
            for (trace, &i) in traces.iter_mut().zip(&traced) {
                let column: Vec<f64> = rows.iter().map(|row| row[i]).collect();
                (trace.low, trace.high) =
                    (envelope([&trace.low[..], &column[..]]).0, envelope([&trace.high[..], &column[..]]).1);
            }
            for (cloud, columns) in points.iter_mut().zip(&motor_columns) {
                if let Some((speed, torque)) = columns {
                    cloud.extend(rows.iter().step_by(2).map(|row| [row[*speed], row[*torque]]));
                }
            }
        }
    }
    Ok(BandRuns { runs: configs.len(), traces, points })
}

/// What the model keeps for these views: the mode list with the config it was found for, the mode picked for the 3D
/// view, and the bounds runs with the config they are of, and their job.
#[derive(Default)]
pub struct Insight {
    pub modes: Option<(Config, Vec<ModeView>)>,
    pub picked: Option<usize>,
    pub band_runs: Option<(Config, BandRuns)>,
    #[allow(clippy::type_complexity)] // the model's `Job` of what `band_runs` answers, with its config
    pub(crate) band_job:
        Option<std::sync::Mutex<std::sync::mpsc::Receiver<Result<(Config, BandRuns), (&'static str, String)>>>>,
}

/// The labels these views take from the model.
pub const KEYS: [&str; 22] = [
    "ui-colour-utilisation",
    "ui-modes",
    "ui-modes-hint",
    "ui-modes-find",
    "ui-modes-mode",
    "ui-modes-hz",
    "ui-modes-damping",
    "ui-modes-linear",
    "ui-modes-shape",
    "ui-modes-from-machine",
    "ui-modes-from-linear",
    "ui-modes-stiffness",
    "ui-modes-stiffness-belt",
    "ui-modes-stiffness-gearbox",
    "ui-modes-stiffness-guide",
    "ui-modes-stiffness-frame",
    "ui-modes-stiffness-base",
    "ui-modes-stiffness-link",
    "ui-colour-path-error",
    "ui-colour-tracking-error",
    "ui-colour-speed",
    "ui-colour-belt-tension",
];
