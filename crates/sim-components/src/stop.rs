//! How far a machine travels after a stop (`Command::Halt`), triggered at many moments of its cycle or path (`study`):
//! where each axis is fastest, where the kinetic plus potential energy is largest, and on a grid between. Per moment the
//! distances the tool and each axis travel, the stopping time, the peak deceleration, whether an axis went beyond its
//! stroke into a hard stop, and the DC link's peak voltage and the braking resistor's energy; the worst moment judged.

use crate::analysis::Verdict;
use crate::control::{Command, StopCategory};
use crate::sim::{Sim, Snapshot};
use sim_core::{Config, ConfigError, MachineEntry};

/// Why a moment was chosen.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Moment {
    /// On the even grid over the pass.
    Grid,
    /// Where tool axis k is fastest.
    Fastest(usize),
    /// Where the machine's kinetic plus potential energy is largest.
    Energy,
}

/// One stop: when it was triggered (s from the start of the run) and what it came to.
#[derive(Debug, Clone, PartialEq)]
pub struct Stop {
    pub at: f64,
    pub moment: Moment,
    /// How far the tool got from where it was at the trigger, over the axes in metres (over all axes on a machine without
    /// one), and each tool axis, in its unit: the farthest, not the path, so that ringing at the end does not add up.
    pub tool_distance: f64,
    pub axis_distance: Vec<f64>,
    /// From the trigger until every axis stays below a hundredth of its speed limit, s.
    pub time: f64,
    /// The tool's largest deceleration over 1 ms, m/s^2 (over the axes as `tool_distance`).
    pub peak_deceleration: f64,
    /// An axis went beyond its stroke (into its hard stop).
    pub hard_stop: bool,
    /// The highest DC link voltage of the drives, V, and the energy their braking resistors took, J.
    pub peak_dc_voltage: f64,
    pub resistor_energy: f64,
    /// A drive tripped on DC link overvoltage and the machine coasted from then on.
    pub tripped: bool,
}

/// The stops of `study`, in the order of their moments, the index of the one with the longest tool distance, and the
/// verdicts on it: `stopping-distance` against `ratings.max_stopping_distance_m` (where the config has it) and, with the
/// DC link simulated, `stop-dc-link`, its peak voltage against the trip level, failed when a stop tripped a drive.
#[derive(Debug, Clone, PartialEq)]
pub struct Study {
    pub stops: Vec<Stop>,
    pub worst: usize,
    pub verdicts: Vec<Verdict>,
}

/// The longest a stop is followed, s.
const LONGEST_STOP_S: f64 = 3.0;
/// How long every axis has to stay slow for the machine to count as stopped, s.
const STANDING_S: f64 = 0.1;
/// The longest pass of a cycle, s.
const LONGEST_PASS_S: f64 = 60.0;

/// The config's cycle, or its `[path]` (`sim::with_path`), run once; stops of `category` triggered at the moments where
/// each tool axis is fastest, where the energy is largest, and at `grid` moments evenly over the pass (from its start).
pub fn study(entry: &MachineEntry, cfg: &Config, category: StopCategory, grid: usize) -> Result<Study, ConfigError> {
    let started = || -> Result<Sim, ConfigError> {
        let mut sim = Sim::new(entry, cfg)?;
        let command = match sim.plan_configured_path() {
            Some(points) => Command::Trajectory(points.map_err(|e| ConfigError::Invalid(e.key.into()))?),
            None => Command::StartCycle,
        };
        sim.command(command).map_err(|e| ConfigError::Invalid(e.key.into()))?;
        Ok(sim)
    };
    // The pass: until the cycle starts again, or the path has run.
    let mut sim = started()?;
    let axes = sim.machine.axes().to_vec();
    let every = (1e-3 / sim.dt).round().max(1.0) as u64;
    let (mut pose, mut rate) = (vec![0.0; axes.len()], vec![0.0; axes.len()]);
    let mut fastest = vec![(0.0f64, 0u64); axes.len()];
    let mut energy = (f64::NEG_INFINITY, 0u64);
    let path = sim.path.is_some();
    while sim.time() < LONGEST_PASS_S && if path { sim.control.busy() } else { sim.control.cycle_passes() < 2 } {
        sim.step();
        if sim.step % every == 0 {
            sim.machine.tool_state(&sim.state, &mut pose, &mut rate);
            for (best, v) in fastest.iter_mut().zip(&rate) {
                if v.abs() > best.0 {
                    *best = (v.abs(), sim.step);
                }
            }
            let e = sim.machine.energy(&sim.state);
            if e > energy.0 {
                energy = (e, sim.step);
            }
        }
    }
    let pass = sim.step;
    let mut moments: Vec<(u64, Moment)> = (0..grid).map(|i| (pass * i as u64 / grid as u64, Moment::Grid)).collect();
    moments.extend(fastest.iter().enumerate().filter(|(_, f)| f.0 > 0.0).map(|(k, f)| (f.1, Moment::Fastest(k))));
    moments.push((energy.1, Moment::Energy));
    moments.sort_by_key(|m| m.0);
    moments.dedup_by_key(|m| m.0);
    // The run again, kept at each moment.
    let mut sim = started()?;
    let mut kept: Vec<Snapshot> = Vec::new();
    for (step, _) in &moments {
        while sim.step < *step {
            sim.step();
        }
        kept.push(sim.snapshot());
    }
    let stops: Vec<Stop> =
        moments.iter().zip(&kept).map(|((_, moment), at)| stop(&mut sim, at, *moment, category)).collect();
    let worst =
        (0..stops.len()).max_by(|&a, &b| stops[a].tool_distance.total_cmp(&stops[b].tool_distance)).unwrap_or(0);
    let mut verdicts = Vec::new();
    // The tool distance is in rad on a machine without an axis in m: it has its own limit.
    let key = if axes.iter().any(|a| a.unit == "m") { "ratings.max_stopping_distance_m" } else { "ratings.max_stopping_angle_rad" };
    if let (Ok(limit), Some(w)) = (cfg.f64(key), stops.get(worst)) {
        let value = w.tool_distance;
        verdicts.push(Verdict { check: "stopping-distance", motor: None, value, limit, ok: value <= limit });
    }
    let p = sim.control.drive_params();
    if p.dc_link {
        let value = stops.iter().map(|s| s.peak_dc_voltage).fold(0.0, f64::max);
        let ok = !stops.iter().any(|s| s.tripped);
        verdicts.push(Verdict { check: "stop-dc-link", motor: None, value, limit: p.overvoltage, ok });
    }
    Ok(Study { stops, worst, verdicts })
}

/// One stop of `category` from `at`, followed until the machine stands (or `LONGEST_STOP_S`).
fn stop(sim: &mut Sim, at: &Snapshot, moment: Moment, category: StopCategory) -> Stop {
    sim.restore(at).expect("a snapshot of this simulation");
    let axes = sim.machine.axes().to_vec();
    let metres: Vec<bool> = if axes.iter().any(|a| a.unit == "m") {
        axes.iter().map(|a| a.unit == "m").collect()
    } else {
        vec![true; axes.len()]
    };
    let slack = sim.machine.setpoint_slack();
    let shared = sim.control.drive_params().shared_dc_link;
    let start = sim.time();
    // A stop the controller refuses (the drives already off or stopping) leaves the machine as it goes.
    let _ = sim.command(Command::Halt(category));
    let (mut pose, mut rate) = (vec![0.0; axes.len()], vec![0.0; axes.len()]);
    sim.machine.tool_state(&sim.state, &mut pose, &mut rate);
    let mut result = Stop {
        at: start,
        moment,
        tool_distance: 0.0,
        axis_distance: vec![0.0; axes.len()],
        time: 0.0,
        peak_deceleration: 0.0,
        hard_stop: false,
        peak_dc_voltage: 0.0,
        resistor_energy: 0.0,
        tripped: false,
    };
    let speed = |rate: &[f64]| rate.iter().zip(&metres).filter(|(_, m)| **m).map(|(v, _)| v * v).sum::<f64>().sqrt();
    let every = (1e-3 / sim.dt).round().max(1.0) as u64;
    let (from, mut last) = (pose.clone(), rate.clone());
    let (mut steps, mut still) = (0u64, 0.0);
    while still < STANDING_S && (steps as f64) * sim.dt < LONGEST_STOP_S {
        sim.step();
        steps += 1;
        sim.machine.tool_state(&sim.state, &mut pose, &mut rate);
        let away: Vec<f64> = pose.iter().zip(&from).map(|(x, x0)| x - x0).collect();
        result.tool_distance = result.tool_distance.max(speed(&away));
        for (d, a) in result.axis_distance.iter_mut().zip(&away) {
            *d = d.max(a.abs());
        }
        if steps % every == 0 {
            let change: Vec<f64> = rate.iter().zip(&last).map(|(v, l)| v - l).collect();
            result.peak_deceleration = result.peak_deceleration.max(speed(&change) / (every as f64 * sim.dt));
            last.clone_from(&rate);
        }
        // A hundredth: a carriage ringing on its belt and brake after the torque is off is not travelling any more.
        let moving = rate.iter().zip(&axes).any(|(v, a)| v.abs() >= 1e-2 * a.max_vel);
        if moving {
            (result.time, still) = (steps as f64 * sim.dt, 0.0);
        } else {
            still += sim.dt;
        }
        result.hard_stop |= pose.iter().zip(&axes).any(|(x, a)| *x < a.min - slack || *x > a.max + slack);
        let drives = &sim.control.drives;
        result.peak_dc_voltage = drives.iter().map(|d| d.dc_voltage).fold(result.peak_dc_voltage, f64::max);
        let burned: f64 = if shared {
            drives.first().map_or(0.0, |d| d.resistor_power)
        } else {
            drives.iter().map(|d| d.resistor_power).sum()
        };
        result.resistor_energy += burned * sim.dt;
        result.tripped |= drives.iter().any(|d| d.tripped);
    }
    result
}
