//! Motion controller: plans in tool coordinates, transforms to motor setpoints, and runs one drive
//! per motor. It works on any machine through the machine interface.

use crate::drive::{Drive, DriveParams};
use crate::profile::Move;
use sim_core::{Config, ConfigError, Machine};

type Result<T> = std::result::Result<T, ConfigError>;

#[derive(Debug, Clone, PartialEq)]
pub enum Command {
    /// Point-to-point move of all axes, synchronised.
    MoveTo(Vec<f64>),
    /// Run the configured pick-and-place cycle until stopped.
    StartCycle,
    /// Finish the current segment, then stay.
    Stop,
    /// A position setpoint from outside (the PLC). The controller interpolates towards it over one
    /// setpoint interval, so slow and jittery updates do not arrive as steps.
    Setpoint(Vec<f64>),
}

/// A command the controller refuses; `key` is a translation key.
#[derive(Debug, Clone, PartialEq)]
pub struct CommandError {
    pub key: &'static str,
}

#[derive(Debug, Clone)]
struct Segment {
    target: Vec<f64>,
    dwell_ticks: u64,
    /// Whether the payload is attached once this segment has arrived.
    payload: bool,
}

#[derive(Debug, Clone)]
enum Motion {
    Idle,
    /// `tail` is the previous move still finishing while this one has already started (corner
    /// blending): its remaining distance is added on top.
    Moving { mv: Move, start_tick: u64, tail: Option<(Move, u64)> },
    Dwell { until_tick: u64 },
    /// Linear interpolation from `from` to `to` between two ticks.
    Following { from: Vec<f64>, to: Vec<f64>, start_tick: u64 },
}

pub struct Controller {
    params: DriveParams,
    /// Loop cycles in base steps.
    n_velocity: u64,
    n_position: u64,
    n_profile: u64,
    profile_cycle: f64,
    setpoint_ticks: u64,
    /// How long before the end of a segment the next one may start; 0 with blending off.
    blend_time: f64,
    segments: Vec<Segment>,
    motion: Motion,
    /// Index of the cycle segment under way, while the cycle runs.
    cycle: Option<usize>,
    stopping: bool,
    pub payload_attached: bool,
    pub drives: Vec<Drive>,
    /// Tool-side setpoints and following error.
    pub set_pos: Vec<f64>,
    pub set_vel: Vec<f64>,
    pub set_acc: Vec<f64>,
    pub tool_error: Vec<f64>,
    /// Added to the position setpoint only, not to the feedforward: a test signal for the closed
    /// loop (step response, frequency response).
    pub setpoint_offset: Vec<f64>,
    motor_pos: Vec<f64>,
    motor_vel: Vec<f64>,
}

fn steps(cfg: &Config, path: &str, dt: f64) -> Result<u64> {
    let n = (cfg.positive(path)? / dt).round();
    // A loop cycle must be a whole number of base steps.
    if n >= 1.0 && (n * dt - cfg.f64(path)?).abs() < 1e-6 * dt { Ok(n as u64) } else { Err(ConfigError::Invalid(path.into())) }
}

impl Controller {
    /// A controller at rest at the machine's current state.
    pub fn new(cfg: &Config, machine: &dyn Machine, state: &[f64]) -> Result<Self> {
        let (axes, motors) = (machine.axes().len(), machine.motors().len());
        let mut c = Self {
            params: DriveParams::from_config(cfg, 1.0, 1.0)?,
            n_velocity: 1,
            n_position: 1,
            n_profile: 1,
            profile_cycle: 1.0,
            setpoint_ticks: 1,
            blend_time: 0.0,
            segments: Vec::new(),
            motion: Motion::Idle,
            cycle: None,
            stopping: false,
            payload_attached: true,
            drives: Vec::new(),
            set_pos: vec![0.0; axes],
            set_vel: vec![0.0; axes],
            set_acc: vec![0.0; axes],
            tool_error: vec![0.0; axes],
            setpoint_offset: vec![0.0; axes],
            motor_pos: vec![0.0; motors],
            motor_vel: vec![0.0; motors],
        };
        c.apply_config(cfg, machine)?;
        machine.motor_state(state, &mut c.motor_pos, &mut c.motor_vel);
        // The setpoint starts where the motors are. With a stretched belt and a loaded gearbox that
        // is a few micrometres from the tool pose, and starting there avoids a jolt at the first step.
        machine.forward(&c.motor_pos, &mut c.set_pos);
        // Start holding the load: the torque that keeps the pose, and the feedforward's share of it.
        let (mut hold, mut ff) = (vec![0.0; motors], vec![0.0; motors]);
        machine.equilibrium(&c.set_pos, &mut Vec::new(), &mut hold);
        machine.feedforward_torque(&c.set_pos, &c.set_acc, &mut ff);
        c.drives = (0..motors).map(|k| Drive::at_rest(&c.params, c.motor_pos[k], hold[k], ff[k])).collect();
        Ok(c)
    }

    /// Re-reads gains, switches, cycle times and the cycle; keeps the loops' running state.
    /// A config with a refused value changes nothing.
    pub fn apply_config(&mut self, cfg: &Config, machine: &dyn Machine) -> Result<()> {
        let dt = cfg.positive("sim.dt_s")?;
        if steps(cfg, "drive.current_cycle_s", dt)? != 1 {
            // The current loop is the base step.
            return Err(ConfigError::Invalid("drive.current_cycle_s".into()));
        }
        let n_velocity = steps(cfg, "drive.velocity_cycle_s", dt)?;
        let n_position = steps(cfg, "drive.position_cycle_s", dt)?;
        let n_profile = steps(cfg, "profile.cycle_s", dt)?;
        let profile_cycle = n_profile as f64 * dt;
        let setpoint_ticks = steps(cfg, "profile.setpoint_interval_s", profile_cycle)?;
        let params = DriveParams::from_config(cfg, dt, n_velocity as f64 * dt)?;
        let blend_time = if cfg.bool("effects.corner_blending")? { cfg.f64("profile.blend_time_s")?.max(0.0) } else { 0.0 };
        let mut segments = Vec::new();
        for (i, segment) in cfg.array("cycle.segment")?.iter().enumerate() {
            let bad = || ConfigError::Invalid(format!("cycle.segment[{i}]"));
            let number = |v: &toml::Value| v.as_float().or_else(|| v.as_integer().map(|i| i as f64));
            let target: Vec<f64> = segment.get("target").and_then(|t| t.as_array()).ok_or_else(bad)?.iter().filter_map(number).collect();
            if !in_range(machine, &target) {
                return Err(bad());
            }
            segments.push(Segment {
                target,
                dwell_ticks: (segment.get("dwell_s").and_then(number).ok_or_else(bad)? / profile_cycle).round() as u64,
                payload: segment.get("payload").and_then(|p| p.as_bool()).ok_or_else(bad)?,
            });
        }
        (self.n_velocity, self.n_position, self.n_profile) = (n_velocity, n_position, n_profile);
        (self.profile_cycle, self.setpoint_ticks, self.params, self.segments) = (profile_cycle, setpoint_ticks, params, segments);
        self.blend_time = blend_time;
        Ok(())
    }

    pub fn command(&mut self, command: Command, machine: &dyn Machine) -> std::result::Result<(), CommandError> {
        match command {
            Command::MoveTo(target) => {
                if !in_range(machine, &target) {
                    return Err(CommandError { key: "error-target-out-of-range" });
                }
                if !matches!(self.motion, Motion::Idle) || self.cycle.is_some() {
                    return Err(CommandError { key: "error-busy" });
                }
                // The tick is filled in when the profile generator picks the move up.
                self.motion = Motion::Moving { mv: Move::new(&self.set_pos, &target, machine.axes()), start_tick: u64::MAX, tail: None };
            }
            Command::StartCycle => {
                if !matches!(self.motion, Motion::Idle) || self.cycle.is_some() {
                    return Err(CommandError { key: "error-busy" });
                }
                if self.segments.is_empty() {
                    return Err(CommandError { key: "error-no-cycle" });
                }
                self.cycle = Some(self.segments.len() - 1);
                self.stopping = false;
            }
            Command::Stop => self.stopping = true,
            Command::Setpoint(target) => {
                if !in_range(machine, &target) {
                    return Err(CommandError { key: "error-target-out-of-range" });
                }
                if self.cycle.is_some() || matches!(self.motion, Motion::Moving { .. } | Motion::Dwell { .. }) {
                    return Err(CommandError { key: "error-busy" });
                }
                self.motion = Motion::Following { from: self.set_pos.clone(), to: target, start_tick: u64::MAX };
            }
        }
        Ok(())
    }

    /// True while a move, a dwell or the cycle is under way.
    pub fn busy(&self) -> bool {
        self.cycle.is_some() || matches!(self.motion, Motion::Moving { .. } | Motion::Dwell { .. })
    }

    pub fn cycle_segment(&self) -> Option<usize> {
        self.cycle
    }

    /// The torque the drives can give at a motor speed: (lowest, highest).
    pub fn torque_curve(&self, speed: f64) -> (f64, f64) {
        self.params.torque_curve(speed)
    }

    /// One base step: runs whichever loops are due and writes the motor torques.
    pub fn update(&mut self, step: u64, machine: &dyn Machine, state: &[f64], torque: &mut [f64]) {
        machine.motor_state(state, &mut self.motor_pos, &mut self.motor_vel);
        if step % self.n_profile == 0 {
            self.profile_tick(step / self.n_profile, machine);
        }
        if step % self.n_position == 0 {
            for (drive, pos) in self.drives.iter_mut().zip(&self.motor_pos) {
                drive.position_loop(&self.params, *pos);
            }
            // Following error per motor, transformed back to the tool axes for display.
            let (set, actual): (Vec<f64>, Vec<f64>) = self.drives.iter().map(|d| (d.pos_set, d.pos_set - d.following_error)).unzip();
            let (mut tool_set, mut tool_actual) = (self.set_pos.clone(), self.set_pos.clone());
            machine.forward(&set, &mut tool_set);
            machine.forward(&actual, &mut tool_actual);
            for i in 0..self.tool_error.len() {
                self.tool_error[i] = tool_set[i] - tool_actual[i];
            }
        }
        for (k, drive) in self.drives.iter_mut().enumerate() {
            if step % self.n_velocity == 0 {
                drive.velocity_loop(&self.params, self.motor_pos[k], self.motor_vel[k]);
            }
            drive.current_loop(&self.params, self.motor_vel[k]);
            torque[k] = drive.torque;
        }
    }

    /// Profile generator: new tool setpoints, then the inverse transformation to motor setpoints.
    fn profile_tick(&mut self, tick: u64, machine: &dyn Machine) {
        let dt = self.profile_cycle;
        if matches!(self.motion, Motion::Idle) {
            self.next_cycle_segment(machine);
        }
        match std::mem::replace(&mut self.motion, Motion::Idle) {
            Motion::Idle => {
                self.set_vel.fill(0.0);
                self.set_acc.fill(0.0);
            }
            Motion::Moving { mv, start_tick, mut tail } => {
                let start_tick = if start_tick == u64::MAX { tick } else { start_tick };
                let t = (tick - start_tick) as f64 * dt;
                mv.at(t, &mut self.set_pos, &mut self.set_vel, &mut self.set_acc);
                if let Some((old, old_start)) = &tail {
                    // The previous move's remaining way to the corner, on top of the new move.
                    let n = self.set_pos.len();
                    let (mut pos, mut vel, mut acc, mut end) = (vec![0.0; n], vec![0.0; n], vec![0.0; n], vec![0.0; n]);
                    let t_old = (tick - old_start) as f64 * dt;
                    old.at(t_old, &mut pos, &mut vel, &mut acc);
                    old.at(f64::INFINITY, &mut end, &mut vec![0.0; n], &mut vec![0.0; n]);
                    for i in 0..n {
                        self.set_pos[i] += pos[i] - end[i];
                        self.set_vel[i] += vel[i];
                        self.set_acc[i] += acc[i];
                    }
                    if t_old >= old.duration() {
                        tail = None;
                    }
                }
                self.motion = if t >= mv.duration() && tail.is_none() {
                    match self.cycle {
                        Some(i) => {
                            self.payload_attached = self.segments[i].payload;
                            Motion::Dwell { until_tick: tick + self.segments[i].dwell_ticks }
                        }
                        None => Motion::Idle,
                    }
                } else {
                    match self.blend_into_next(&mv, t, tail.is_some(), machine) {
                        Some(next) => Motion::Moving { mv: next, start_tick: tick, tail: Some((mv, start_tick)) },
                        None => Motion::Moving { mv, start_tick, tail },
                    }
                };
            }
            Motion::Dwell { until_tick } => {
                if tick >= until_tick {
                    self.next_cycle_segment(machine);
                } else {
                    self.motion = Motion::Dwell { until_tick };
                }
            }
            Motion::Following { from, to, start_tick } => {
                let start_tick = if start_tick == u64::MAX { tick } else { start_tick };
                let n = self.setpoint_ticks as f64;
                let done = ((tick - start_tick) as f64 / n).min(1.0);
                for i in 0..to.len() {
                    self.set_pos[i] = from[i] + (to[i] - from[i]) * done;
                    self.set_vel[i] = if done < 1.0 { (to[i] - from[i]) / (n * dt) } else { 0.0 };
                }
                self.set_acc.fill(0.0);
                if done < 1.0 {
                    self.motion = Motion::Following { from, to, start_tick };
                }
            }
        }

        let motors = self.drives.len();
        let (mut pos, mut ahead, mut behind, mut ff) = (vec![0.0; motors], vec![0.0; motors], vec![0.0; motors], vec![0.0; motors]);
        let offset_pos: Vec<f64> = self.set_pos.iter().zip(&self.setpoint_offset).map(|(p, o)| p + o).collect();
        machine.inverse(&offset_pos, &mut pos);
        // Motor speed setpoints: the inverse transformation applied a moment ahead and behind.
        let h = 1e-3;
        let shifted = |sign: f64| -> Vec<f64> { self.set_pos.iter().zip(&self.set_vel).map(|(p, v)| p + sign * h * v).collect() };
        machine.inverse(&shifted(1.0), &mut ahead);
        machine.inverse(&shifted(-1.0), &mut behind);
        machine.feedforward_torque(&self.set_pos, &self.set_acc, &mut ff);
        // Motor acceleration setpoints the same way, for the drives' fine interpolation.
        let (mut acc_ahead, mut acc_behind) = (vec![0.0; motors], vec![0.0; motors]);
        let shifted = |sign: f64| -> Vec<f64> { self.set_pos.iter().zip(&self.set_acc).map(|(p, a)| p + sign * h * a).collect() };
        machine.inverse(&shifted(1.0), &mut acc_ahead);
        machine.inverse(&shifted(-1.0), &mut acc_behind);
        for (k, drive) in self.drives.iter_mut().enumerate() {
            drive.pos_set = pos[k];
            drive.vel_set = (ahead[k] - behind[k]) / (2.0 * h);
            drive.acc_set = (acc_ahead[k] - acc_behind[k]) / (2.0 * h);
            drive.since_setpoint = 0.0;
            drive.torque_ff = ff[k];
        }
    }

    /// Corner blending: the next cycle segment, if it should start now, while `current` still has
    /// time `t` behind it and its braking ahead. The corner is rounded instead of stopped at.
    ///
    /// Only where the machine does not have to stop anyway (no dwell, no gripper action), where no
    /// axis reverses, and only while both moves are on their ramps, so the speeds add up to no
    /// more than either cruise speed.
    fn blend_into_next(&mut self, current: &Move, t: f64, blending_already: bool, machine: &dyn Machine) -> Option<Move> {
        let i = self.cycle?;
        let segment = &self.segments[i];
        if self.blend_time <= 0.0 || self.stopping || blending_already || segment.dwell_ticks > 0 || segment.payload != self.payload_attached {
            return None;
        }
        let next_index = (i + 1) % self.segments.len();
        let next = Move::new(&segment.target, &self.segments[next_index].target, machine.axes());
        if current.delta().iter().zip(next.delta()).any(|(a, b)| a * b < 0.0) || next.duration() == 0.0 {
            return None;
        }
        let lead = self.blend_time.min(current.ramp()).min(next.ramp());
        if t < current.duration() - lead {
            return None;
        }
        self.cycle = Some(next_index);
        Some(next)
    }

    fn next_cycle_segment(&mut self, machine: &dyn Machine) {
        let Some(last) = self.cycle else { return };
        if self.stopping {
            self.cycle = None;
            self.stopping = false;
            return;
        }
        let next = (last + 1) % self.segments.len();
        self.cycle = Some(next);
        self.motion = Motion::Moving { mv: Move::new(&self.set_pos, &self.segments[next].target, machine.axes()), start_tick: u64::MAX, tail: None };
    }
}

fn in_range(machine: &dyn Machine, target: &[f64]) -> bool {
    let axes = machine.axes();
    target.len() == axes.len() && target.iter().zip(axes).all(|(t, a)| t.is_finite() && *t >= a.min && *t <= a.max)
}

/// An axis limit that asks more of a motor than the drive allows; `key` is a translation key.
#[derive(Debug, Clone, PartialEq)]
pub struct LimitViolation {
    pub key: &'static str,
    pub motor: usize,
    pub needed: f64,
    pub allowed: f64,
}

/// Checks that the axis limits do not exceed the motor limits. A diagonal move loads one motor
/// with both axes, so every combination of axis directions is tried.
pub fn check_limits(machine: &dyn Machine, cfg: &Config) -> Result<Vec<LimitViolation>> {
    let (axes, motors) = (machine.axes(), machine.motors().len());
    let pose = sim_core::testkit::mid_pose(machine);
    let (max_speed, max_torque) = (cfg.positive("drive.rated_speed_rad_s")?, cfg.positive("drive.peak_torque_nm")?);
    let (mut speed, mut torque) = (vec![0f64; motors], vec![0f64; motors]);
    let (mut a, mut b) = (vec![0.0; motors], vec![0.0; motors]);
    for combo in 0..1u32 << axes.len() {
        let sign = |i: usize| if combo >> i & 1 == 1 { 1.0 } else { -1.0 };
        let h = 1e-3;
        let shifted = |s: f64| -> Vec<f64> { axes.iter().enumerate().map(|(i, ax)| pose[i] + s * h * sign(i) * ax.max_vel).collect() };
        machine.inverse(&shifted(1.0), &mut a);
        machine.inverse(&shifted(-1.0), &mut b);
        let acc: Vec<f64> = axes.iter().enumerate().map(|(i, ax)| sign(i) * ax.max_acc).collect();
        for k in 0..motors {
            speed[k] = speed[k].max(((a[k] - b[k]) / (2.0 * h)).abs());
        }
        machine.feedforward_torque(&pose, &acc, &mut a);
        for k in 0..motors {
            torque[k] = torque[k].max(a[k].abs());
        }
    }
    let mut out = Vec::new();
    for k in 0..motors {
        if speed[k] > max_speed {
            out.push(LimitViolation { key: "limit-motor-speed", motor: k, needed: speed[k], allowed: max_speed });
        }
        if torque[k] > max_torque {
            out.push(LimitViolation { key: "limit-motor-torque", motor: k, needed: torque[k], allowed: max_torque });
        }
    }
    Ok(out)
}
