//! Analysis for motor sizing: peak and RMS torque per cycle, operating points against the
//! torque-speed curve, and the mechanical frequency response.

use crate::control::{Command, CommandError};
use crate::sim::Sim;
use sim_core::{Config, ConfigError, MachineEntry, Rk4, advance};

/// The load on one motor over one pass of the cycle.
#[derive(Debug, Clone, Default)]
pub struct MotorLoad {
    pub peak_torque: f64,
    pub rms_torque: f64,
    pub peak_speed: f64,
    /// Highest ratio of torque to the torque the drive has available at that speed. Above 1 the
    /// cycle leaves the torque-speed curve.
    pub utilisation: f64,
    /// (speed, torque) once per millisecond, for plotting over the torque-speed curve.
    pub points: Vec<(f64, f64)>,
}

#[derive(Debug, Clone)]
pub struct CycleLoad {
    pub duration: f64,
    pub motors: Vec<MotorLoad>,
}

/// Runs the configured cycle and measures its second pass, when the machine has settled into it.
pub fn analyse_cycle(sim: &mut Sim) -> Result<CycleLoad, CommandError> {
    sim.command(Command::StartCycle)?;
    let motors = sim.machine.motors().len();
    let (mut pos, mut speed) = (vec![0.0; motors], vec![0.0; motors]);
    let mut loads = vec![MotorLoad::default(); motors];
    let (mut passes, mut last, mut start, mut steps) = (0, None, 0.0, 0u64);
    let every = (1e-3 / sim.dt).round().max(1.0) as u64;
    loop {
        sim.step();
        let now = sim.control.cycle_segment();
        if now != last && now == Some(0) {
            passes += 1;
            if passes == 2 {
                start = sim.time();
            } else if passes == 3 {
                break;
            }
        }
        last = now;
        if sim.time() > 600.0 {
            return Err(CommandError { key: "error-cycle-timeout" });
        }
        if passes < 2 {
            continue;
        }
        sim.machine.motor_state(&sim.state, &mut pos, &mut speed);
        for (k, load) in loads.iter_mut().enumerate() {
            let torque = sim.control.drives[k].torque;
            load.peak_torque = load.peak_torque.max(torque.abs());
            load.rms_torque += torque * torque;
            load.peak_speed = load.peak_speed.max(speed[k].abs());
            let (low, high) = sim.control.torque_curve(speed[k]);
            let available = if torque >= 0.0 { high } else { -low };
            load.utilisation = load.utilisation.max(torque.abs() / available.max(1e-9));
            if steps % every == 0 {
                load.points.push((speed[k], torque));
            }
        }
        steps += 1;
    }
    for load in &mut loads {
        load.rms_torque = (load.rms_torque / steps as f64).sqrt();
    }
    let _ = sim.command(Command::Stop);
    Ok(CycleLoad { duration: sim.time() - start, motors: loads })
}

/// One point of the frequency response from a motor's torque to its own speed.
#[derive(Debug, Clone, Copy)]
pub struct SweepPoint {
    pub frequency: f64,
    /// Speed amplitude per torque amplitude, rad/s per N m.
    pub gain: f64,
    pub phase_deg: f64,
}

/// Mechanical frequency response without control: a sine torque on one motor on top of the holding
/// torques, measured at that motor's speed once the response has settled. Belt resonances show as
/// peaks, the load swinging against the held motor as a dip.
pub fn frequency_sweep(entry: &MachineEntry, cfg: &Config, motor: usize, frequencies: &[f64], amplitude: f64) -> Result<Vec<SweepPoint>, ConfigError> {
    let machine = (entry.build)(cfg)?;
    let (dt, substeps) = (cfg.positive("sim.dt_s")?, cfg.positive("sim.substeps")? as u32);
    let motors = machine.motors().len();
    if motor >= motors {
        return Err(ConfigError::Invalid("motor".into()));
    }
    let (mut start, mut hold) = (Vec::new(), vec![0.0; motors]);
    machine.equilibrium(&cfg.f64s("start.pose")?, &mut start, &mut hold);
    let mut rk = Rk4::new(start.len());
    let (mut pos, mut speed) = (vec![0.0; motors], vec![0.0; motors]);
    let mut out = Vec::new();
    for &frequency in frequencies {
        let omega = std::f64::consts::TAU * frequency;
        // Whole periods: at least 0.3 s to settle, then at least 0.2 s to measure.
        let period = 1.0 / frequency;
        let settle = (0.3 / period).ceil().max(5.0) * period;
        let measure = (0.2 / period).ceil().max(5.0) * period;
        let (settle_steps, measure_steps) = ((settle / dt).round() as u64, (measure / dt).round() as u64);
        let (mut state, mut torque) = (start.clone(), hold.clone());
        let (mut in_phase, mut quadrature) = (0.0, 0.0);
        for step in 0..settle_steps + measure_steps {
            let t = (step as f64 + 0.5) * dt;
            torque[motor] = hold[motor] + amplitude * (omega * t).sin();
            advance(machine.as_ref(), &mut rk, &mut state, &torque, dt, substeps);
            if step >= settle_steps {
                machine.motor_state(&state, &mut pos, &mut speed);
                let t = (step + 1) as f64 * dt;
                in_phase += speed[motor] * (omega * t).sin();
                quadrature += speed[motor] * (omega * t).cos();
            }
        }
        let (a, b) = (2.0 * in_phase / measure_steps as f64, 2.0 * quadrature / measure_steps as f64);
        out.push(SweepPoint { frequency, gain: a.hypot(b) / amplitude, phase_deg: b.atan2(a).to_degrees() });
    }
    Ok(out)
}

/// Frequencies spaced evenly on a logarithmic axis, both ends included.
pub fn log_spaced(from: f64, to: f64, points: usize) -> Vec<f64> {
    (0..points).map(|i| from * (to / from).powf(i as f64 / (points.max(2) - 1) as f64)).collect()
}

// ---------------------------------------------------------------------------------------------
// Tuning: how the closed position loop answers a test signal on its setpoint.

/// The answer of one axis to a small step of its position setpoint.
#[derive(Debug, Clone)]
pub struct StepResponse {
    pub size: f64,
    /// (time since the step, tool position relative to where it was), four times per millisecond.
    pub trace: Vec<(f64, f64)>,
    /// Time from 10 % to 90 % of the step.
    pub rise_time: f64,
    /// Highest excursion beyond the step, as a fraction of it.
    pub overshoot: f64,
    /// After this time the position stays within 2 % of the step.
    pub settling_time: f64,
}

/// Tool position of one axis, after letting the machine come to rest at its start pose.
fn settled(entry: &MachineEntry, cfg: &Config, axis: usize) -> Result<(Sim, f64), ConfigError> {
    let mut sim = Sim::new(entry, cfg)?;
    if axis >= sim.machine.axes().len() {
        return Err(ConfigError::Invalid("axis".into()));
    }
    sim.run(0.05, |_| {});
    let position = tool_position(&sim, axis);
    Ok((sim, position))
}

fn tool_position(sim: &Sim, axis: usize) -> f64 {
    let n = sim.machine.axes().len();
    let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
    sim.machine.tool_state(&sim.state, &mut pos, &mut vel);
    pos[axis]
}

/// Steps the position setpoint of one axis by `size` (without feedforward, so only the feedback
/// loops answer) and records the tool position. With friction on, a small step shows stiction too.
pub fn step_response(entry: &MachineEntry, cfg: &Config, axis: usize, size: f64, seconds: f64) -> Result<StepResponse, ConfigError> {
    let (mut sim, start) = settled(entry, cfg, axis)?;
    sim.control.setpoint_offset[axis] = size;
    let (t0, every) = (sim.time(), (0.25e-3 / sim.dt).round().max(1.0) as u64);
    let mut trace = vec![(0.0, 0.0)];
    sim.run(seconds, |s| {
        if s.step % every == 0 {
            trace.push((s.time() - t0, tool_position(s, axis) - start));
        }
    });
    let relative = |fraction: f64| trace.iter().find(|(_, y)| y / size >= fraction).map_or(f64::NAN, |(t, _)| *t);
    let overshoot = trace.iter().map(|(_, y)| y / size - 1.0).fold(0.0, f64::max);
    let settling_time = trace.iter().rev().find(|(_, y)| (y / size - 1.0).abs() > 0.02).map_or(0.0, |(t, _)| *t);
    Ok(StepResponse { size, rise_time: relative(0.9) - relative(0.1), overshoot, settling_time, trace })
}

/// Closed-loop frequency response of the position loop of one axis: a sine on the position
/// setpoint, measured at the tool. Gain is tool amplitude over setpoint amplitude.
pub fn closed_loop_response(entry: &MachineEntry, cfg: &Config, axis: usize, frequencies: &[f64], amplitude: f64) -> Result<Vec<SweepPoint>, ConfigError> {
    let mut out = Vec::new();
    for &frequency in frequencies {
        let (mut sim, start) = settled(entry, cfg, axis)?;
        let (omega, t0, dt) = (std::f64::consts::TAU * frequency, sim.time(), sim.dt);
        // Whole periods: at least 0.3 s to settle, then at least 0.3 s to measure.
        let period = 1.0 / frequency;
        let whole = |seconds: f64| ((seconds / period).ceil().max(4.0) * period / dt).round() as u64;
        let (settle_steps, measure_steps) = (whole(0.3), whole(0.3));
        let (mut in_phase, mut quadrature) = (0.0, 0.0);
        for step in 0..settle_steps + measure_steps {
            sim.control.setpoint_offset[axis] = amplitude * (omega * (sim.time() - t0)).sin();
            sim.step();
            if step >= settle_steps {
                let (t, y) = (sim.time() - t0, tool_position(&sim, axis) - start);
                in_phase += y * (omega * t).sin();
                quadrature += y * (omega * t).cos();
            }
        }
        let (a, b) = (2.0 * in_phase / measure_steps as f64, 2.0 * quadrature / measure_steps as f64);
        out.push(SweepPoint { frequency, gain: a.hypot(b) / amplitude, phase_deg: b.atan2(a).to_degrees() });
    }
    Ok(out)
}

/// The -3 dB bandwidth of a closed-loop response: where the gain first falls below 1 / sqrt(2),
/// interpolated between the measured points on the logarithmic frequency axis.
pub fn bandwidth(points: &[SweepPoint]) -> Option<f64> {
    let limit = std::f64::consts::FRAC_1_SQRT_2;
    let below = points.iter().position(|p| p.gain < limit)?;
    let (a, b) = (points.get(below.checked_sub(1)?)?, &points[below]);
    let share = (a.gain - limit) / (a.gain - b.gain);
    Some(a.frequency * (b.frequency / a.frequency).powf(share))
}
