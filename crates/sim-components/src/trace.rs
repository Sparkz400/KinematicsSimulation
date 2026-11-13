//! Drive traces: what a drive's scope exports, read from CSV and replayed through the simulator.

use crate::control::Command;
use crate::sim::Sim;
use sim_core::{Config, ConfigError, MachineEntry, MotorInfo};

/// A trace that cannot be read or replayed; `key` is a translation key, `detail` its argument.
#[derive(Debug, Clone, PartialEq)]
pub struct TraceError {
    pub key: &'static str,
    pub detail: String,
}

impl From<ConfigError> for TraceError {
    fn from(e: ConfigError) -> Self {
        Self { key: e.key(), detail: e.detail().into() }
    }
}

/// Whether the motors turn by the same angle per travel of each axis everywhere: at 40 % and at 60 % of every range,
/// 1 mm along each axis, to 1e-9. Replay and fit take the motor angles per axis travel as constant (`identify`), so
/// they refuse a machine whose transformation is not linear (the delta's arms), which gave numbers without meaning.
pub fn linear_transformation(machine: &dyn sim_core::Machine) -> bool {
    let (axes, motors) = (machine.axes(), machine.motors().len());
    let turned = |pose: &[f64], axis: usize| {
        let (mut a, mut b) = (vec![0.0; motors], vec![0.0; motors]);
        let ahead: Vec<f64> = pose.iter().enumerate().map(|(i, p)| if i == axis { p + 1e-3 } else { *p }).collect();
        machine.inverse(pose, &mut a);
        machine.inverse(&ahead, &mut b);
        b.iter().zip(&a).map(|(b, a)| b - a).collect::<Vec<f64>>()
    };
    let at = |share: f64| axes.iter().map(|a| a.min + share * (a.max - a.min)).collect::<Vec<f64>>();
    (0..axes.len()).all(|axis| turned(&at(0.4), axis).iter().zip(turned(&at(0.6), axis)).all(|(a, b)| (a - b).abs() <= 1e-9 * a.abs().max(1e-6)))
}

/// One motor's columns of a trace, all at the motor shaft: position setpoint, position, speed, torque.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct MotorTrace {
    pub setpoint: Vec<f64>,
    pub angle: Vec<f64>,
    pub speed: Vec<f64>,
    pub torque: Vec<f64>,
}

impl MotorTrace {
    pub fn following_error(&self) -> Vec<f64> {
        self.setpoint.iter().zip(&self.angle).map(|(s, a)| s - a).collect()
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Trace {
    pub time: Vec<f64>,
    pub motors: Vec<MotorTrace>,
}

/// The named columns of a CSV table with a header line. A unit in brackets after a column's name is
/// dropped; columns that are not asked for are ignored.
pub fn columns(text: &str, names: &[String]) -> Result<Vec<Vec<f64>>, TraceError> {
    let mut lines = text.lines().filter(|l| !l.trim().is_empty());
    let header: Vec<&str> = lines.next().unwrap_or("").split(',').map(|h| h.split('[').next().unwrap_or("").trim()).collect();
    let mut at = Vec::new();
    for name in names {
        at.push(header.iter().position(|h| h == name).ok_or_else(|| TraceError { key: "error-trace-column", detail: name.clone() })?);
    }
    let mut out = vec![Vec::new(); names.len()];
    for line in lines {
        let fields: Vec<&str> = line.split(',').collect();
        for (column, &i) in out.iter_mut().zip(&at) {
            let value = fields.get(i).and_then(|f| f.trim().parse::<f64>().ok()).filter(|v| v.is_finite());
            column.push(value.ok_or_else(|| TraceError { key: "error-trace-row", detail: line.into() })?);
        }
    }
    Ok(out)
}

impl Trace {
    /// Reads a trace whose columns are named like the simulator's signals: `time` and per motor
    /// `motor.<id>.angle_setpoint`, `.angle`, `.speed` and `.torque`. So a file written by a run is a trace.
    pub fn parse(text: &str, motors: &[MotorInfo]) -> Result<Self, TraceError> {
        let mut names = vec!["time".to_string()];
        for motor in motors {
            names.extend(["angle_setpoint", "angle", "speed", "torque"].map(|n| format!("motor.{}.{n}", motor.id)));
        }
        let mut found = columns(text, &names)?.into_iter();
        let time = found.next().unwrap_or_default();
        let mut next = || found.next().unwrap_or_default();
        let motors = motors.iter().map(|_| MotorTrace { setpoint: next(), angle: next(), speed: next(), torque: next() }).collect();
        Ok(Self { time, motors })
    }

    /// Adds the present state of a simulation as one row, with the values a run writes to CSV.
    pub fn record(&mut self, sim: &Sim) {
        let n = sim.control.drives.len();
        let (mut angle, mut speed) = (vec![0.0; n], vec![0.0; n]);
        sim.machine.motor_state(&sim.state, &mut angle, &mut speed);
        self.motors.resize(n, MotorTrace::default());
        self.time.push(sim.time());
        for (k, motor) in self.motors.iter_mut().enumerate() {
            let drive = &sim.control.drives[k];
            motor.setpoint.push(drive.pos_set);
            motor.angle.push(angle[k]);
            motor.speed.push(speed[k]);
            motor.torque.push(drive.torque);
        }
    }
}

/// Plays the position setpoints of a trace through the simulator and returns what the simulated drives
/// did at the times of the trace's rows. The config's start pose has to be where the trace starts.
///
/// The setpoints become tool poses (the machine's forward transformation) and run as `Command::Trajectory`,
/// so the simulator's own controller makes the speed and torque feedforward from them, as it does for any
/// path. With a row per profile cycle the position setpoints are reproduced exactly; the speed and
/// acceleration setpoints are slopes between rows.
pub fn replay(entry: &MachineEntry, cfg: &Config, trace: &Trace) -> Result<Trace, TraceError> {
    let mut sim = Sim::new(entry, cfg)?;
    if !linear_transformation(sim.machine.as_ref()) {
        return Err(TraceError { key: "error-trace-not-linear", detail: String::new() });
    }
    let rows = trace.time.len();
    // The first row is the start, and carrying on behind the last one (below) takes three more.
    if rows < 4 {
        return Err(TraceError { key: "error-trace-short", detail: String::new() });
    }
    // A row holds the setpoint the drive worked with up to the row's time (a run writes a row before it
    // computes the step at that time). So each row is commanded one row earlier, and the first row, the
    // start, is left out.
    let mut points: Vec<(f64, Vec<f64>)> = (1..rows)
        .map(|i| {
            let setpoint: Vec<f64> = trace.motors.iter().map(|m| m.setpoint[i]).collect();
            let mut pose = vec![0.0; sim.machine.axes().len()];
            sim.machine.forward(&setpoint, &mut pose);
            (trace.time[i] - trace.time[1], pose)
        })
        .collect();
    // A trajectory ends at rest, a trace may end in motion. Two more points, carrying on with the last
    // rows' speed and acceleration, put the stop and the piece of curve that bends towards it behind
    // the last row.
    for _ in 0..2 {
        let [a, b, c] = [1, 2, 3].map(|back| &points[points.len() - back]);
        points.push((2.0 * a.0 - b.0, (0..a.1.len()).map(|i| 3.0 * a.1[i] - 3.0 * b.1[i] + c.1[i]).collect()));
    }
    sim.command(Command::Trajectory(points)).map_err(|e| TraceError { key: e.key, detail: String::new() })?;
    let mut out = Trace::default();
    for time in &trace.time {
        while sim.step < ((time - trace.time[0]) / sim.dt).round() as u64 {
            sim.step();
        }
        out.record(&sim);
    }
    out.time.clone_from(&trace.time);
    Ok(out)
}

/// Per motor how far a replay is from its trace: RMS and peak of the difference of the torques, then of the
/// following errors.
pub fn differences(measured: &Trace, simulated: &Trace) -> Vec<[f64; 4]> {
    let apart = |(m, s): (&MotorTrace, &MotorTrace)| {
        let (torque, error) = (difference(&m.torque, &s.torque), difference(&m.following_error(), &s.following_error()));
        [torque.0, torque.1, error.0, error.1]
    };
    measured.motors.iter().zip(&simulated.motors).map(apart).collect()
}

/// Measured and simulated torque, speed and following error side by side, per motor named in `ids`, one line
/// per row of the trace, as CSV with a header.
pub fn overlay_csv(ids: &[&str], measured: &Trace, simulated: &Trace) -> String {
    let names = ["torque", "speed", "following_error"];
    let header: Vec<String> = ids.iter().flat_map(|id| names.map(|n| format!("motor.{id}.{n}_measured,motor.{id}.{n}_simulated"))).collect();
    let mut out = format!("time,{}\n", header.join(","));
    let pairs = || measured.motors.iter().zip(&simulated.motors);
    let errors: Vec<(Vec<f64>, Vec<f64>)> = pairs().map(|(m, s)| (m.following_error(), s.following_error())).collect();
    for (row, time) in measured.time.iter().enumerate() {
        let values: Vec<String> = pairs()
            .zip(&errors)
            .flat_map(|((m, s), (m_error, s_error))| [m.torque[row], s.torque[row], m.speed[row], s.speed[row], m_error[row], s_error[row]])
            .map(|v| format!("{v:.9e}"))
            .collect();
        out += &format!("{time:.9e},{}\n", values.join(","));
    }
    out
}

/// RMS and peak of the difference between two columns.
pub fn difference(a: &[f64], b: &[f64]) -> (f64, f64) {
    let squares: f64 = a.iter().zip(b).map(|(a, b)| (a - b).powi(2)).sum();
    ((squares / a.len().max(1) as f64).sqrt(), a.iter().zip(b).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max))
}
