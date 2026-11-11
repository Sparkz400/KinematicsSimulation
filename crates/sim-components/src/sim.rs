//! A machine, its controller and the integrator, stepped together. Every front end runs this.

use crate::control::{Command, CommandError, Controller, LimitViolation, check_limits};
use sim_core::{Config, ConfigError, Machine, MachineEntry, Rk4, Scene, SignalInfo, advance};

pub struct Sim {
    pub machine: Box<dyn Machine>,
    pub control: Controller,
    pub state: Vec<f64>,
    pub torque: Vec<f64>,
    pub step: u64,
    pub dt: f64,
    substeps: u32,
    rk: Rk4,
    payload_attached: bool,
    /// Axis limits that exceed what the drives allow, found when the config was applied.
    pub limit_violations: Vec<LimitViolation>,
}

fn timing(cfg: &Config) -> Result<(f64, u32), ConfigError> {
    let substeps = cfg.positive("sim.substeps")?;
    if substeps.fract() != 0.0 {
        return Err(ConfigError::Invalid("sim.substeps".into()));
    }
    Ok((cfg.positive("sim.dt_s")?, substeps as u32))
}

impl Sim {
    /// The machine at rest at its configured start pose.
    pub fn new(entry: &MachineEntry, cfg: &Config) -> Result<Self, ConfigError> {
        let machine = (entry.build)(cfg)?;
        let (dt, substeps) = timing(cfg)?;
        let start = cfg.f64s("start.pose")?;
        if start.len() != machine.axes().len() {
            return Err(ConfigError::Invalid("start.pose".into()));
        }
        let mut state = Vec::new();
        let mut torque = vec![0.0; machine.motors().len()];
        machine.equilibrium(&start, &mut state, &mut torque);
        let control = Controller::new(cfg, machine.as_ref(), &state)?;
        let limit_violations = check_limits(machine.as_ref(), cfg)?;
        Ok(Self { rk: Rk4::new(state.len()), machine, control, state, torque, step: 0, dt, substeps, payload_attached: true, limit_violations })
    }

    /// Applies a changed config to the running simulation: parameters and effect switches take
    /// effect from the next step, without a restart.
    pub fn reconfigure(&mut self, entry: &MachineEntry, cfg: &Config) -> Result<(), ConfigError> {
        let mut machine = (entry.build)(cfg)?;
        let (dt, substeps) = timing(cfg)?;
        if dt != self.dt {
            // The step counter is the clock; a new base step would make time jump.
            return Err(ConfigError::Invalid("sim.dt_s".into()));
        }
        self.control.apply_config(cfg, machine.as_ref())?;
        machine.set_payload_attached(self.payload_attached);
        self.state = machine.adopt_state(self.machine.as_ref(), &self.state);
        self.rk = Rk4::new(self.state.len());
        self.limit_violations = check_limits(machine.as_ref(), cfg)?;
        self.machine = machine;
        self.substeps = substeps;
        Ok(())
    }

    pub fn time(&self) -> f64 {
        self.step as f64 * self.dt
    }

    pub fn command(&mut self, command: Command) -> Result<(), CommandError> {
        self.control.command(command, self.machine.as_ref())
    }

    /// One base step: controller, then physics with the torques held.
    pub fn step(&mut self) {
        self.control.update(self.step, self.machine.as_ref(), &self.state, &mut self.torque);
        if self.control.payload_attached != self.payload_attached {
            self.payload_attached = self.control.payload_attached;
            self.machine.set_payload_attached(self.payload_attached);
        }
        advance(self.machine.as_ref(), &mut self.rk, &mut self.state, &self.torque, self.dt, self.substeps);
        self.step += 1;
    }

    pub fn run(&mut self, seconds: f64, mut each: impl FnMut(&Self)) {
        for _ in 0..(seconds / self.dt).round() as u64 {
            self.step();
            each(self);
        }
    }

    /// Every signal of the simulation. Scopes, CSV columns and PLC variables are generated from
    /// this list, never written per machine.
    pub fn signal_infos(&self) -> Vec<SignalInfo> {
        let mut out = vec![
            SignalInfo::new("time", "group-sim", "quantity-time", "s"),
            SignalInfo::new("payload", "effect-payload", "quantity-attached", ""),
        ];
        for axis in self.machine.axes() {
            let (unit, rate, rate2) = (axis.unit, format!("{}/s", axis.unit), format!("{}/s²", axis.unit));
            for (name, unit) in [
                ("position", unit),
                ("velocity", rate.as_str()),
                ("position-setpoint", unit),
                ("velocity-setpoint", rate.as_str()),
                ("acceleration-setpoint", rate2.as_str()),
                ("following-error", unit),
            ] {
                out.push(SignalInfo::new(format!("axis.{}.{}", axis.id, name.replace('-', "_")), axis.key, &format!("quantity-{name}"), unit));
            }
        }
        for motor in self.machine.motors() {
            for (name, unit) in [
                ("angle", "rad"),
                ("speed", "rad/s"),
                ("torque", "N m"),
                ("torque-command", "N m"),
                ("current", "A"),
                ("voltage", "V"),
                ("speed-command", "rad/s"),
                ("following-error", "rad"),
            ] {
                out.push(SignalInfo::new(format!("motor.{}.{}", motor.id, name.replace('-', "_")), motor.key, &format!("quantity-{name}"), unit));
            }
        }
        out.extend(self.machine.signal_infos());
        out
    }

    /// The current value of every signal, in the order of `signal_infos`.
    pub fn sample(&self, out: &mut Vec<f64>) {
        let (axes, motors) = (self.machine.axes().len(), self.machine.motors().len());
        let (mut pos, mut vel) = (vec![0.0; axes], vec![0.0; axes]);
        let (mut angle, mut speed) = (vec![0.0; motors], vec![0.0; motors]);
        self.machine.tool_state(&self.state, &mut pos, &mut vel);
        self.machine.motor_state(&self.state, &mut angle, &mut speed);
        let c = &self.control;
        out.clear();
        out.extend([self.time(), if self.payload_attached { 1.0 } else { 0.0 }]);
        for i in 0..axes {
            out.extend([pos[i], vel[i], c.set_pos[i], c.set_vel[i], c.set_acc[i], c.tool_error[i]]);
        }
        for (k, d) in c.drives.iter().enumerate() {
            out.extend([angle[k], speed[k], d.torque, d.torque_command, d.current, d.voltage, d.speed_command, d.following_error]);
        }
        self.machine.signals(&self.state, &self.torque, out);
    }

    /// Position of a signal in a sample.
    pub fn signal_index(&self, id: &str) -> Option<usize> {
        self.signal_infos().iter().position(|s| s.id == id)
    }

    pub fn scene(&self, out: &mut Scene) {
        self.machine.scene(&self.state, out);
    }
}
