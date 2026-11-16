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
/// 1 mm along each axis, to 1e-9. The fit per axis (`identify::fit_rigid`) takes the motor angles per axis travel as
/// constant, so it refuses a machine whose transformation is not linear (the delta's arms, a SCARA), which gave numbers
/// without meaning; those are fitted in the motors' own coordinates (`identify::fit_joints`). Replay works for both.
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
    (0..axes.len()).all(|axis| {
        turned(&at(0.4), axis)
            .iter()
            .zip(turned(&at(0.6), axis))
            .all(|(a, b)| (a - b).abs() <= 1e-9 * a.abs().max(1e-6))
    })
}

/// Whether the fit per axis (`identify::fit_rigid`) applies: a linear transformation, and a rigid model whose torques
/// per unit of each axis's acceleration are the same at 10 %, 20 % ... 90 % of every range (a SCARA's inertia, even in its
/// elbow angle, agrees at poses symmetric about the middle; a cam's dwells hold two probes alike), to `PER_AXIS_SPREAD` of
/// the largest. A SCARA's tool pose is its joints, linear, but the inertia an arm turns changes with the pose.
pub fn per_axis_fit(machine: &dyn sim_core::Machine) -> bool {
    let (axes, motors) = (machine.axes(), machine.motors().len());
    let at = |share: f64| axes.iter().map(|a| a.min + share * (a.max - a.min)).collect::<Vec<f64>>();
    let inertia = |pose: &[f64], axis: usize| {
        let (mut a, mut b) = (vec![0.0; motors], vec![0.0; motors]);
        let acc: Vec<f64> = (0..axes.len()).map(|i| if i == axis { 1.0 } else { 0.0 }).collect();
        machine.feedforward_torque(pose, &vec![0.0; axes.len()], &mut a);
        machine.feedforward_torque(pose, &acc, &mut b);
        b.iter().zip(&a).map(|(b, a)| b - a).collect::<Vec<f64>>()
    };
    // Nine poses across the range, 10 % to 90 %: two of them can both fall in a dwell of a cam (the Geneva drive's and the
    // indexer's reflected inertia varies 5 to 7 times, yet agreed at 35 % and 60 %).
    let same = |axis: usize| {
        let low = inertia(&at(0.1), axis);
        let largest = low.iter().fold(0.0, |m: f64, v| m.max(v.abs()));
        (2..=9).all(|k| {
            let high = inertia(&at(k as f64 / 10.0), axis);
            low.iter().zip(&high).all(|(a, b)| (a - b).abs() <= PER_AXIS_SPREAD * largest)
        })
    };
    linear_transformation(machine) && (0..axes.len()).all(same)
}

/// How far the torques per unit of an axis's acceleration may differ across the range for the fit per axis
/// (`per_axis_fit`), of the largest: the fitted mass is then their mean over the trace, and what differs from it is in
/// the trace's and the replay's residual alike. The T-gantry's belt (its runs on the vertical posts and its share in the
/// Z axis's moves) changes its torques by up to 3.9 % over the range (4.5 % without the payload); the other machines
/// fitted per axis by 0; the disc cam by 6.2 % (4.1 % without the payload, so its config without one is fitted per
/// axis); the slider-crank, the SCARA, the indexers' cams and the arm by 40 % and more.
pub const PER_AXIS_SPREAD: f64 = 0.05;

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
    let header: Vec<&str> =
        lines.next().unwrap_or("").split(',').map(|h| h.split('[').next().unwrap_or("").trim()).collect();
    let mut at = Vec::new();
    for name in names {
        at.push(
            header
                .iter()
                .position(|h| h == name)
                .ok_or_else(|| TraceError { key: "error-trace-column", detail: name.clone() })?,
        );
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

/// Refuses times that do not rise, naming the first data row (1 = the row under the header) that does not.
pub fn rising(time: &[f64]) -> Result<(), TraceError> {
    match time.windows(2).position(|p| !(p[1] > p[0])) {
        Some(i) => Err(TraceError {
            key: "error-import-time",
            detail: format!("row {}: {} s after {} s", i + 2, time[i + 1], time[i]),
        }),
        None => Ok(()),
    }
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
        rising(&time)?;
        let mut next = || found.next().unwrap_or_default();
        let motors = motors
            .iter()
            .map(|_| MotorTrace { setpoint: next(), angle: next(), speed: next(), torque: next() })
            .collect();
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

/// The shortest run-up of a replay (`replay`), s: long enough for the loops' integrators and the friction to settle
/// at the trace's first speed before its first row.
pub const RUN_UP_S: f64 = 0.25;

/// Plays the position setpoints of a trace through the simulator and returns what the simulated drives
/// did at the times of the trace's rows. The replay starts from the trace's own state, whatever the config's start pose:
/// at rest at a pose before it, it runs up to the first row's position setpoint at the first row's measured speed, so a
/// recording cut out of a longer motion replays without the jolt of a start from rest. The run-up is a smootherstep of
/// the speed (rest to that speed, zero acceleration at both ends, `RUN_UP_S` or more, so that its acceleration stays
/// within a quarter of each axis's limit), the way covered half the speed times its time; the pose it starts at has to
/// be within the axes' ranges; the run-up starts where the machine's setpoint stands at rest there (its motors, a sag
/// away from the tool) and takes that difference up on the way. A trace that starts at rest where the config's own
/// machine stands at rest (a trace written by a run of the config) is replayed from that state with no run-up, the
/// first row the machine at rest, as before the run-up existed: it replays onto itself to rounding.
///
/// The setpoints become tool poses (the machine's forward transformation) and run as `Command::Trajectory`,
/// so the simulator's own controller makes the speed and torque feedforward from them, as it does for any
/// path. With a row per profile cycle the position setpoints are reproduced exactly; the speed and
/// acceleration setpoints are slopes between rows. Behind the last row the motion carries on (below): the trace's end
/// is not forced to rest.
pub fn replay(entry: &MachineEntry, cfg: &Config, trace: &Trace) -> Result<Trace, TraceError> {
    // The rows are a stream, interpolated between neighbours as a drive does (the Hermite cubic, whatever the config's
    // `profile.trajectory_spline`): the C2 spline, clamped at rest at the end of the run-out below, rang back over
    // the rows before it, the last ones off by 4 N m of torque on a belt axis and 800 N m on a linear motor.
    let mut cfg = cfg.clone();
    cfg.set_text("profile.trajectory_spline = \"hermite\"")?;
    let rows = trace.time.len();
    // The first row is the start, and carrying on behind the last one (below) takes three more.
    if rows < 4 {
        return Err(TraceError { key: "error-trace-short", detail: String::new() });
    }
    let machine = (entry.build)(&crate::catalog::resolve(entry, &cfg)?)?;
    let pose_of = |angles: &[f64]| {
        let mut pose = vec![0.0; machine.axes().len()];
        machine.forward(angles, &mut pose);
        pose
    };
    // The trace's state at its first row: the position setpoint's pose, and the measured speed in tool units (the pose
    // 1 ms ahead at that speed).
    let first: Vec<f64> = trace.motors.iter().map(|m| m.setpoint[0]).collect();
    let start = pose_of(&first);
    let ahead = pose_of(&trace.motors.iter().map(|m| m.setpoint[0] + 1e-3 * m.speed[0]).collect::<Vec<f64>>());
    let speed: Vec<f64> = ahead.iter().zip(&start).map(|(a, s)| (a - s) / 1e-3).collect();
    let step = trace.time[1] - trace.time[0];
    // A trace that starts at rest where the config's own machine stands at rest (its drives' setpoints those of the
    // first row: a trace written by a run of this config) is replayed from that very state, with no run-up: so it
    // replays onto itself to rounding, where the gear mesh sticks at rest too (an arm, a machine on cables).
    let at_rest = trace.motors.iter().all(|m| m.speed[0].abs() <= 1e-9);
    let own = Sim::new(entry, &cfg)?;
    let same_start =
        own.control.drives.iter().zip(&first).all(|(d, s)| (d.pos_set - s).abs() <= 1e-9 * s.abs().max(1.0));
    // Peak acceleration of the smootherstep: 15/8 of speed over time.
    let long =
        machine.axes().iter().zip(&speed).map(|(a, v)| 1.875 * v.abs() / (0.25 * a.max_acc)).fold(RUN_UP_S, f64::max);
    let run_up = if at_rest && same_start { 0 } else { (long / step).ceil() as usize + 1 };
    let span = run_up.saturating_sub(1) as f64 * step;
    let from: Vec<f64> = start.iter().zip(&speed).map(|(p, v)| p - 0.5 * v * span).collect();
    let mut sim = if run_up == 0 {
        own
    } else {
        cfg.set("start.pose", toml::Value::Array(from.iter().map(|p| toml::Value::Float(*p)).collect()))?;
        Sim::new(entry, &cfg)?
    };
    // The setpoint of a machine at rest is where its motors stand, some hundredths of a millimetre from the tool where
    // gravity twists its gears or stretches its belts or cables (`Controller::command`): the run-up starts there and
    // takes the difference up along a smoothstep, so the setpoint does not step.
    let rest = sim.control.planned_pos().to_vec();
    // Smootherstep of the speed, integrated: s(u) = 2.5 u^4 - 3 u^5 + u^6, 1/2 at u = 1.
    let mut points: Vec<(f64, Vec<f64>)> = (0..run_up)
        .map(|k| {
            let u = k as f64 / (run_up - 1) as f64;
            let way = span * (2.5 * u.powi(4) - 3.0 * u.powi(5) + u.powi(6));
            let blend = 1.0 - u * u * u * (10.0 - 15.0 * u + 6.0 * u * u);
            (k as f64 * step, (0..from.len()).map(|i| from[i] + speed[i] * way + (rest[i] - from[i]) * blend).collect())
        })
        .collect();
    // A row holds the setpoint the drive worked with up to the row's time (a run writes a row before it
    // computes the step at that time). So each row is commanded one row earlier: the first row, the start, ends the
    // run-up, and row i follows at the run-up's end plus the time from row 1 to row i + 1. Without a run-up, row 1 is
    // commanded at once and row 0 is the machine at rest.
    let offset = if run_up == 0 { 0.0 } else { span + step };
    points.extend((1..rows).map(|i| {
        let setpoint: Vec<f64> = trace.motors.iter().map(|m| m.setpoint[i]).collect();
        (offset + trace.time[i] - trace.time[1], pose_of(&setpoint))
    }));
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
        while sim.step < ((time - trace.time[0] + offset) / sim.dt).round() as u64 {
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
        let (torque, error) =
            (difference(&m.torque, &s.torque), difference(&m.following_error(), &s.following_error()));
        [torque.0, torque.1, error.0, error.1]
    };
    measured.motors.iter().zip(&simulated.motors).map(apart).collect()
}

/// Measured and simulated torque, speed and following error side by side, per motor named in `ids`, one line
/// per row of the trace, as CSV with a header.
pub fn overlay_csv(ids: &[&str], measured: &Trace, simulated: &Trace) -> String {
    let names = ["torque", "speed", "following_error"];
    let header: Vec<String> =
        ids.iter().flat_map(|id| names.map(|n| format!("motor.{id}.{n}_measured,motor.{id}.{n}_simulated"))).collect();
    let mut out = format!("time,{}\n", header.join(","));
    let pairs = || measured.motors.iter().zip(&simulated.motors);
    let errors: Vec<(Vec<f64>, Vec<f64>)> = pairs().map(|(m, s)| (m.following_error(), s.following_error())).collect();
    for (row, time) in measured.time.iter().enumerate() {
        let values: Vec<String> = pairs()
            .zip(&errors)
            .flat_map(|((m, s), (m_error, s_error))| {
                [m.torque[row], s.torque[row], m.speed[row], s.speed[row], m_error[row], s_error[row]]
            })
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
