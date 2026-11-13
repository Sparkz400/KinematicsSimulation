//! A machine, its controller and the integrator, stepped together. Every front end runs this.

use crate::control::{Command, CommandError, Controller, LimitViolation, check_limits, in_range};
use sim_core::{Config, ConfigError, LOSS_PREFIX, Machine, MachineEntry, Rk4, Scene, SignalInfo, TEMPERATURE_PREFIX, ThermalPart, advance};

/// A part of the machine that warms with its loss power (`Machine::thermal_parts`, effects.part_heating): one thermal
/// mass with a thermal resistance R to its surroundings, so that its temperature rise follows
/// tau d(rise)/dt = R P - rise and settles at R P under a steady loss P.
#[derive(Debug, Clone)]
pub struct HeatedPart {
    pub part: ThermalPart,
    /// Where its loss power is among the machine's loss powers (`Machine::loss_powers`).
    loss: usize,
    /// Its loss power at the end of the last step, W; kept up to date only while it warms (effects.part_heating).
    pub loss_power: f64,
    /// K/W, s, and the rise it is rated for, K.
    pub resistance: f64,
    pub time_constant: f64,
    pub max_rise: f64,
    /// Temperature above the surroundings, K.
    pub rise: f64,
}

/// The parts of `machine` that warm, with their values from the config; the rises start at zero.
fn heated_parts(cfg: &Config, machine: &dyn Machine) -> Result<Vec<HeatedPart>, ConfigError> {
    let infos: Vec<SignalInfo> = machine.signal_infos().into_iter().filter(|s| s.id.starts_with(LOSS_PREFIX)).collect();
    let mut parts = Vec::new();
    for part in machine.thermal_parts() {
        let key = |name: &str| format!("thermal.{}_{name}", part.kind);
        let loss = infos.iter().position(|s| s.id == format!("{LOSS_PREFIX}{}", part.id)).ok_or_else(|| ConfigError::Invalid(format!("{LOSS_PREFIX}{}", part.id)))?;
        let (resistance, time_constant, max_rise) = (cfg.non_negative(&key("resistance_k_w"))?, cfg.positive(&key("time_constant_s"))?, cfg.positive(&key("max_rise_k"))?);
        parts.push(HeatedPart { part, loss, loss_power: 0.0, resistance, time_constant, max_rise, rise: 0.0 });
    }
    Ok(parts)
}

/// Per drive in the integrated state: winding current, and since the start of the step the electrical energy, the
/// copper loss and the work on the shaft.
const WINDING_STATES: usize = 4;

pub struct Sim {
    pub machine: Box<dyn Machine>,
    pub control: Controller,
    pub state: Vec<f64>,
    pub torque: Vec<f64>,
    pub step: u64,
    pub dt: f64,
    substeps: u32,
    rk: Rk4,
    /// For the steps with the drives enabled: the machine's state and per drive its winding current and the energies
    /// of the step (`DriveParams::winding_rates`), integrated together; with the motors' angles and speeds and
    /// torques at a stage of the integration.
    rk_joint: Rk4,
    joint: Vec<f64>,
    motor_buffers: [Vec<f64>; 3],
    payload_attached: bool,
    /// What switching the payload has added to the machine's energy since the start: a payload picked up brings
    /// its energy along, one put down takes it away. For the energy budget.
    pub payload_energy: f64,
    /// Axis limits that exceed what the drives allow, found when the config was applied.
    pub limit_violations: Vec<LimitViolation>,
    /// The parts that warm with their losses, and whether they do (effects.part_heating, or effects.thermal_feedback,
    /// which feeds back what this computes: `heats`). Their rises are signals
    /// whether or not; switched off they stay as they are.
    pub heated: Vec<HeatedPart>,
    pub part_heating: bool,
    /// Whether the machine's friction and losses follow the parts' rises (effects.thermal_feedback).
    pub thermal_feedback: bool,
    /// The machine's loss powers, for the heated parts.
    losses: Vec<f64>,
}

/// Whether the parts warm with their losses: effects.part_heating, or effects.thermal_feedback, which needs their
/// temperatures.
pub fn heats(cfg: &Config) -> Result<bool, ConfigError> {
    Ok(cfg.bool("effects.part_heating")? || cfg.bool("effects.thermal_feedback")?)
}

fn timing(cfg: &Config) -> Result<(f64, u32), ConfigError> {
    let substeps = cfg.positive("sim.substeps")?;
    if substeps.fract() != 0.0 {
        return Err(ConfigError::Invalid("sim.substeps".into()));
    }
    Ok((cfg.positive("sim.dt_s")?, substeps as u32))
}

/// A config file for a machine, as a front end loads it. The machine's default config is the schema: a key
/// it does not have is refused by name, so that a misspelt key cannot pass silently. `[bounds]` has its own
/// check (`analysis::bounds`), and a rating that the default config leaves out may be there.
pub fn load_config(entry: &MachineEntry, text: &str) -> Result<Config, ConfigError> {
    let cfg = Config::parse(text)?;
    cfg.check_keys(&entry.config(), &crate::analysis::RATINGS_BEYOND_DEFAULT)?;
    Ok(cfg)
}

/// The configured start pose: one finite position per axis, within its range. A machine set up beyond its
/// stroke stands in its end stops, and one set up nowhere (NaN) has no state at all.
pub(crate) fn start_pose(cfg: &Config, machine: &dyn Machine) -> Result<Vec<f64>, ConfigError> {
    let pose = cfg.f64s("start.pose")?;
    if in_range(machine, &pose) { Ok(pose) } else { Err(ConfigError::Invalid("start.pose".into())) }
}

impl Sim {
    /// The machine at rest at its configured start pose.
    pub fn new(entry: &MachineEntry, cfg: &Config) -> Result<Self, ConfigError> {
        // Parts the config names bring their catalogue values; the caller's config keeps the names.
        let cfg = &crate::catalog::resolve(entry, cfg)?;
        let machine = (entry.build)(cfg)?;
        let (dt, substeps) = timing(cfg)?;
        let start = start_pose(cfg, machine.as_ref())?;
        let mut state = Vec::new();
        let mut torque = vec![0.0; machine.motors().len()];
        machine.equilibrium(&start, &mut state, &mut torque);
        let control = Controller::new(cfg, machine.as_ref(), &state)?;
        let limit_violations = check_limits(machine.as_ref(), cfg)?;
        let (heated, part_heating) = (heated_parts(cfg, machine.as_ref())?, heats(cfg)?);
        let motors = torque.len();
        Ok(Self {
            rk: Rk4::new(state.len()),
            rk_joint: Rk4::new(state.len() + WINDING_STATES * motors),
            joint: Vec::new(),
            motor_buffers: std::array::from_fn(|_| vec![0.0; motors]),
            machine,
            control,
            state,
            torque,
            step: 0,
            dt,
            substeps,
            payload_attached: true,
            payload_energy: 0.0,
            limit_violations,
            heated,
            part_heating,
            thermal_feedback: cfg.bool("effects.thermal_feedback")?,
            losses: Vec::new(),
        })
    }

    /// Applies a changed config to the running simulation: parameters and effect switches take
    /// effect from the next step, without a restart.
    pub fn reconfigure(&mut self, entry: &MachineEntry, cfg: &Config) -> Result<(), ConfigError> {
        let cfg = &crate::catalog::resolve(entry, cfg)?;
        let mut machine = (entry.build)(cfg)?;
        let (dt, substeps) = timing(cfg)?;
        if dt != self.dt {
            // The step counter is the clock; a new base step would make time jump.
            return Err(ConfigError::Invalid("sim.dt_s".into()));
        }
        // Not used by a running simulation, but a config it would not start with is refused all the same.
        start_pose(cfg, machine.as_ref())?;
        self.control.apply_config(cfg, machine.as_ref())?;
        machine.set_payload_attached(self.payload_attached);
        let motors = machine.motors().len();
        let (mut before, mut after, mut speed) = (vec![0.0; motors], vec![0.0; motors], vec![0.0; motors]);
        self.machine.motor_state(&self.state, &mut before, &mut speed);
        self.state = machine.adopt_state(self.machine.as_ref(), &self.state);
        machine.motor_state(&self.state, &mut after, &mut speed);
        for k in 0..motors {
            after[k] -= before[k];
        }
        self.control.motors_shifted(&after);
        self.rk = Rk4::new(self.state.len());
        self.rk_joint = Rk4::new(self.state.len() + WINDING_STATES * motors);
        self.motor_buffers = std::array::from_fn(|_| vec![0.0; motors]);
        self.limit_violations = check_limits(machine.as_ref(), cfg)?;
        // A part that is still there keeps its warmth.
        let mut heated = heated_parts(cfg, machine.as_ref())?;
        for part in &mut heated {
            part.rise = self.heated.iter().find(|old| old.part.id == part.part.id).map_or(0.0, |old| old.rise);
        }
        (self.heated, self.part_heating) = (heated, heats(cfg)?);
        self.thermal_feedback = cfg.bool("effects.thermal_feedback")?;
        self.machine = machine;
        self.feed_temperatures_back();
        self.substeps = substeps;
        Ok(())
    }

    pub fn time(&self) -> f64 {
        self.step as f64 * self.dt
    }

    pub fn command(&mut self, command: Command) -> Result<(), CommandError> {
        self.control.command(command, self.machine.as_ref())
    }

    /// The timing for a path from where the machine stands, to run with `Command::Trajectory`.
    pub fn plan_path(&self, pieces: &[crate::path::Piece], time_optimal: bool) -> Result<Vec<(f64, Vec<f64>)>, CommandError> {
        self.control.plan_path(self.machine.as_ref(), pieces, time_optimal)
    }

    /// How far that planned path is from the pieces as given, at most.
    pub fn path_deviation(&self, pieces: &[crate::path::Piece], time_optimal: bool) -> Result<f64, CommandError> {
        self.control.path_deviation(self.machine.as_ref(), pieces, time_optimal)
    }

    /// One base step: the controller sets the windings' voltages, then windings and mechanics go through the step
    /// together (with the drives disabled, the mechanics alone without torque).
    pub fn step(&mut self) {
        self.control.update(self.step, self.machine.as_ref(), &self.state, &mut self.torque);
        if self.control.payload_attached != self.payload_attached {
            self.payload_attached = self.control.payload_attached;
            let before = self.machine.energy(&self.state);
            self.machine.set_payload_attached(self.payload_attached);
            self.payload_energy += self.machine.energy(&self.state) - before;
        }
        self.machine.set_dwelling(self.control.dwell_progress().is_some());
        self.machine.set_brake_engaged(!self.control.enabled());
        // The temperatures the last step ended with; fed back here and not after the parts warm, so that the losses
        // the parts warmed with are those the machine reports after the step.
        self.feed_temperatures_back();
        if self.control.enabled() {
            self.advance_with_windings();
        } else {
            advance(self.machine.as_ref(), &mut self.rk, &mut self.state, &self.torque, self.dt, self.substeps);
        }
        self.machine.after_step(&mut self.state, self.dt);
        if self.part_heating && !self.heated.is_empty() {
            // The loss at the end of the step, held over it: the exact answer of the first-order lag to a held input.
            self.losses.clear();
            self.machine.loss_powers(&self.state, &self.torque, &mut self.losses);
            for part in &mut self.heated {
                part.loss_power = self.losses[part.loss];
                let settled = part.resistance * part.loss_power;
                part.rise += (1.0 - (-self.dt / part.time_constant).exp()) * (settled - part.rise);
            }
        }
        self.step += 1;
    }

    /// With effects.thermal_feedback, tells the machine its parts' rises (`Machine::set_part_temperatures`).
    pub fn feed_temperatures_back(&mut self) {
        if self.thermal_feedback {
            let rises: Vec<f64> = self.heated.iter().map(|h| h.rise).collect();
            self.machine.set_part_temperatures(&rises);
        }
    }

    /// Windings and mechanics over one base step in one integration, the voltages held as the controller set them: the
    /// motor torque follows the winding current within the step, and the electrical power, the copper loss and the
    /// shaft's work come from the same stages as the motion. The core's integrator sees one state vector.
    fn advance_with_windings(&mut self) {
        let Self { machine, control, state, torque, rk_joint, joint, motor_buffers: [angle, speed, stage_torque], dt, substeps, .. } = self;
        let (n, p, drives) = (state.len(), control.drive_params(), &control.drives);
        joint.clear();
        joint.extend_from_slice(state);
        for d in drives {
            joint.extend([d.current, 0.0, 0.0, 0.0]);
        }
        let h = *dt / *substeps as f64;
        for _ in 0..*substeps {
            rk_joint.step(
                |y, dy| {
                    let ((mech, windings), (dmech, dwindings)) = (y.split_at(n), dy.split_at_mut(n));
                    machine.motor_state(mech, angle, speed);
                    for (k, d) in drives.iter().enumerate() {
                        let i = windings[WINDING_STATES * k];
                        stage_torque[k] = p.torque_of(i);
                        dwindings[WINDING_STATES * k..WINDING_STATES * (k + 1)].copy_from_slice(&p.winding_rates(d.voltage, d.winding_resistance(p), i, speed[k]));
                    }
                    machine.derivatives(mech, stage_torque, dmech);
                },
                joint,
                h,
            );
        }
        state.copy_from_slice(&joint[..n]);
        control.winding_stepped(&joint[n..]);
        for (torque, d) in torque.iter_mut().zip(&control.drives) {
            *torque = d.torque;
        }
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
                ("electrical-power", "W"),
                ("winding-temperature", "K"),
                ("dc-link-voltage", "V"),
                ("braking-resistor-power", "W"),
                ("speed-command", "rad/s"),
                ("following-error", "rad"),
                ("angle-setpoint", "rad"),
            ] {
                out.push(SignalInfo::new(format!("motor.{}.{}", motor.id, name.replace('-', "_")), motor.key, &format!("quantity-{name}"), unit));
            }
        }
        out.extend(self.machine.signal_infos());
        out.extend(self.heated.iter().map(|h| SignalInfo::new(format!("{TEMPERATURE_PREFIX}{}", h.part.id), h.part.owner_key, &format!("quantity-{}-temperature", h.part.kind), "K")));
        out
    }

    /// The current value of every signal, in the order of `signal_infos`.
    pub fn sample(&self, out: &mut Vec<f64>) {
        self.sample_base(out);
        self.machine.effect_signals(&self.state, &self.torque, out);
        out.extend(self.heated.iter().map(|h| h.rise));
    }

    /// The signals of `sample` up to the end of the machine's `base_signals`, in the same places: without those of the
    /// effect views (which cost about one more evaluation of the machine) and the parts' temperatures after them.
    pub fn sample_base(&self, out: &mut Vec<f64>) {
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
            out.extend([angle[k], speed[k], d.torque, d.torque_command, d.current, d.voltage, d.power, d.temperature_rise, d.dc_voltage, d.resistor_power, d.speed_command, d.following_error, d.pos_set]);
        }
        self.machine.base_signals(&self.state, &self.torque, out);
    }

    /// Position of a signal in a sample.
    pub fn signal_index(&self, id: &str) -> Option<usize> {
        self.signal_infos().iter().position(|s| s.id == id)
    }

    /// The machine's scene, with where the tool is in it.
    /// Where the windings warm (effects.winding_heating), the k-th motor housing (`scene::MOTOR_PART`) is coloured by
    /// motor k's winding rise over its rise at rated current; where the parts warm, each heated part by its rise over
    /// its rated rise.
    pub fn scene(&self, out: &mut Scene) {
        self.machine.scene(&self.state, out);
        out.tool = self.tool_point();
        let mut colour = |id: &str, index: usize, heat: f64| {
            if let Some(part) = out.parts.iter_mut().filter(|p| p.id == id).nth(index) {
                part.heat = Some(heat as f32);
            }
        };
        let drive = self.control.drive_params();
        if drive.heating {
            for (k, d) in self.control.drives.iter().enumerate() {
                colour(sim_core::scene::MOTOR_PART, k, d.temperature_rise / drive.rated_rise);
            }
        }
        if self.part_heating {
            for h in &self.heated {
                colour(h.part.scene_part, h.part.scene_index, h.rise / h.max_rise);
            }
        }
    }

    /// Where the tool is in the scene right now; `None` where the machine has no mapping.
    pub fn tool_point(&self) -> Option<[f64; 3]> {
        let axes = self.machine.axes().len();
        let (mut pos, mut vel) = (vec![0.0; axes], vec![0.0; axes]);
        self.machine.tool_state(&self.state, &mut pos, &mut vel);
        self.machine.scene_point(&pos)
    }
}
