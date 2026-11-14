//! A machine, its controller and the integrator, stepped together. Every front end runs this.

use crate::control::{Command, CommandError, Controller, LimitViolation, check_limits, in_range};
use sim_core::{
    Config, ConfigError, LOSS_PREFIX, Machine, MachineEntry, Obstacle, Rk4, Scene, SignalInfo, TEMPERATURE_PREFIX,
    ThermalPart, advance,
};

/// A part of the machine that warms with its loss power (`Machine::thermal_parts`, effects.part_heating): one thermal
/// mass with a thermal resistance R to its surroundings, so that its temperature rise follows
/// tau d(rise)/dt = R P - rise and settles at R P under a steady loss P.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct HeatedPart {
    /// The machine's, not written: a simulation built from the same config has it (`Sim::restore`).
    #[serde(skip)]
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
        let loss = infos
            .iter()
            .position(|s| s.id == format!("{LOSS_PREFIX}{}", part.id))
            .ok_or_else(|| ConfigError::Invalid(format!("{LOSS_PREFIX}{}", part.id)))?;
        let (resistance, time_constant, max_rise) = (
            cfg.non_negative(&key("resistance_k_w"))?,
            cfg.positive(&key("time_constant_s"))?,
            cfg.positive(&key("max_rise_k"))?,
        );
        parts.push(HeatedPart { part, loss, loss_power: 0.0, resistance, time_constant, max_rise, rise: 0.0 });
    }
    Ok(parts)
}

/// How near an obstacle a part has to come before the obstacle's push is worked out (effects.crash, `Sim::pushes`), m:
/// the push acts on the motors through the rigid kinematics, whose parts may be some millimetres from the real ones
/// (a carriage swinging on its belt).
const PUSH_RANGE: f64 = 0.01;

/// The step in a motor's angle (or a linear motor's position) by which the slope of a clearance is worked out, by
/// central differences.
const SLOPE_STEP: f64 = 1e-6;

/// An obstacle's push on the machine over one base step (effects.crash), linear in the motors' angles about where the
/// step starts: the depth of the machine in it there (the clearance of the rigid kinematics at those angles, negated),
/// how the clearance grows with each motor's angle (m/rad), and the obstacle's stiffness and damping.
#[derive(Debug, Clone, PartialEq)]
struct Push {
    depth: f64,
    slope: Vec<f64>,
    stiffness: f64,
    damping: f64,
}

impl Push {
    /// The obstacle's force at the motor angles `angle` and speeds `speed`, N: the stiffness times the depth there plus
    /// the damping times the speed into it, pushing only (an obstacle does not pull a part back).
    fn force(&self, from: &[f64], angle: &[f64], speed: &[f64]) -> f64 {
        let moved: f64 = self.slope.iter().zip(angle.iter().zip(from)).map(|(g, (a, f))| g * (a - f)).sum();
        let into = -self.slope.iter().zip(speed).map(|(g, w)| g * w).sum::<f64>();
        let depth = self.depth - moved;
        if depth > 0.0 { (self.stiffness * depth + self.damping * into).max(0.0) } else { 0.0 }
    }
}

/// The motor torques of the pushes, added to `out`: each force times its clearance's slope at the motor, which is the
/// force's virtual work per radian. Returns the sum of the forces, N.
fn push_torques(pushes: &[Push], from: &[f64], angle: &[f64], speed: &[f64], out: &mut [f64]) -> f64 {
    let mut total = 0.0;
    for push in pushes {
        let force = push.force(from, angle, speed);
        total += force;
        for (torque, g) in out.iter_mut().zip(&push.slope) {
            *torque += force * g;
        }
    }
    total
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
    motor_buffers: [Vec<f64>; 4],
    payload_attached: bool,
    /// What switching the payload has added to the machine's energy since the start: a payload picked up brings
    /// its energy along, one put down takes it away. For the energy budget.
    pub payload_energy: f64,
    /// The work the obstacles' pushes took out of the machine since the start (effects.crash, `pushes`): what the
    /// contacts' damping lost and their springs hold. For the energy budget.
    pub crash_energy: f64,
    /// Axis limits that exceed what the drives allow, found when the config was applied.
    pub limit_violations: Vec<LimitViolation>,
    /// The parts that warm with their losses, and whether they do (effects.part_heating, or effects.thermal_feedback,
    /// which feeds back what this computes: `heats`). Their rises are signals
    /// whether or not; switched off they stay as they are.
    pub heated: Vec<HeatedPart>,
    pub part_heating: bool,
    /// Whether the machine's friction and losses follow the parts' rises (effects.thermal_feedback).
    pub thermal_feedback: bool,
    /// effects.thermal_warm_start: the cycle analysis measures its pass with the windings and parts at the rises they
    /// settle at (`analysis::analyse_cycle_with`), not from cold.
    pub warm_start: bool,
    /// The machine's loss powers, for the heated parts.
    losses: Vec<f64>,
    /// The obstacles of the config (`collision::obstacles`), each marked while a part of the machine touches it.
    pub obstacles: Vec<Obstacle>,
    /// effects.crash: an obstacle pushes back on a part that goes into it, with its contact stiffness and damping
    /// (`pushes`), so that the axes in contact are blocked; what trips the drives then is their monitoring
    /// (effects.drive_monitoring).
    pub crash: bool,
    /// `profile.min_clearance_m`: the least clearance to the obstacles a time-optimal path may keep (`plan_path`).
    pub min_clearance: f64,
    /// The clearance to the nearest obstacle after the last step, and the least since the start, m; infinite without
    /// obstacles. Checked every base step.
    pub clearance: f64,
    pub least_clearance: f64,
    /// The first contact with an obstacle since the start.
    pub contact: Option<Contact>,
    /// Whether the drives tripped (effects.drive_monitoring) while a part touched an obstacle, and have not been
    /// enabled since. Which monitor, and the stopping distances, are in `control.fault`.
    pub crashed: bool,
    /// The obstacles' force on the machine over the last step (at its end), N.
    pub contact_force: f64,
    /// Per obstacle the clearance after the last step (`check_obstacles`).
    gaps: Vec<f64>,
    /// The obstacles' pushes over the step under way, and the motor angles they are linear about.
    pushes: Vec<Push>,
    push_from: Vec<f64>,
    /// The work the pushes have taken since the first contact, J.
    absorbed: f64,
}

/// The first contact of a run with an obstacle: when (the end of the base step that found it, s), at which tool pose,
/// how fast the tool went then (the length of its speed vector, axis units per second) and which obstacle. With
/// effects.crash, from then on: the largest force of the obstacles on the machine (N) and the impact energy, the most
/// work their pushes had taken out of the machine at any time (J; at the deepest point of a blocked impact, the
/// kinetic energy the impact took).
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Contact {
    pub time: f64,
    pub pose: Vec<f64>,
    pub speed: f64,
    pub obstacle: usize,
    pub peak_force: f64,
    pub impact_energy: f64,
}

/// What a running simulation has come to apart from its machine (`Sim::snapshot`): the integrated state, the controller
/// with its drives (loops, filters, windings, thermal and overload images), the parts' warmth and the obstacle record.
/// `Sim::restore` takes it back into a simulation built from the same machine and config; what the machine itself keeps
/// between steps (payload, brakes, dwelling, the parts' temperatures) is set again from it.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Snapshot {
    control: Controller,
    state: Vec<f64>,
    torque: Vec<f64>,
    step: u64,
    payload_attached: bool,
    payload_energy: f64,
    crash_energy: f64,
    heated: Vec<HeatedPart>,
    obstacles: Vec<Obstacle>,
    clearance: f64,
    least_clearance: f64,
    contact: Option<Contact>,
    crashed: bool,
    contact_force: f64,
    gaps: Vec<f64>,
    absorbed: f64,
}

/// Whether the parts warm with their losses: effects.part_heating, or effects.thermal_feedback, which needs their
/// temperatures.
pub fn heats(cfg: &Config) -> Result<bool, ConfigError> {
    Ok(cfg.bool("effects.part_heating")? || cfg.bool("effects.thermal_feedback")?)
}

/// effects.thermal_warm_start; a config without the key (one written before 13 November 2026) starts cold.
fn warm_start(cfg: &Config) -> Result<bool, ConfigError> {
    Ok(cfg.get("effects.thermal_warm_start").is_ok() && cfg.bool("effects.thermal_warm_start")?)
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
    cfg.check_keys(&schema(entry, &cfg)?, &crate::analysis::RATINGS_BEYOND_DEFAULT)?;
    Ok(cfg)
}

/// The keys a config for `entry` may have: its default config's, and a motor's own table (`[motor.<id>]` and the rest
/// of `catalog::PER_MOTOR_TABLES`) for any motor `cfg` gives one, with the shared table's keys, though the default has
/// none for that motor (a seven-joint URDF on the arm whose default has six).
/// shortcut: any id passes here, a motor's that the machine does not have too (it is then not read); check the ids
/// against the built machine's motors where a misspelt one has to be refused.
pub fn schema(entry: &MachineEntry, cfg: &Config) -> Result<Config, ConfigError> {
    let mut text = entry.default_config.to_string();
    let default = entry.config();
    for table in crate::catalog::PER_MOTOR_TABLES {
        let (Some(shared), Some(own)) =
            (default.get(table).ok().and_then(|v| v.as_table()), cfg.get(table).ok().and_then(|v| v.as_table()))
        else {
            continue;
        };
        for (id, value) in own {
            if !value.is_table() || shared.contains_key(id) {
                continue;
            }
            text.push_str(&format!(
                "
[{table}.{id}]
"
            ));
            for (key, value) in shared.iter().filter(|(_, v)| !v.is_table()) {
                text.push_str(&format!(
                    "{key} = {value}
"
                ));
            }
        }
    }
    Config::parse(&text)
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
        let limit_violations = check_limits(machine.as_ref(), &control.motor_drives(machine.as_ref()));
        let (heated, part_heating) = (heated_parts(cfg, machine.as_ref())?, heats(cfg)?);
        let motors = torque.len();
        Ok(Self {
            rk: Rk4::new(state.len()),
            rk_joint: Rk4::new(state.len() + WINDING_STATES * motors + 1),
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
            crash_energy: 0.0,
            limit_violations,
            heated,
            part_heating,
            thermal_feedback: cfg.bool("effects.thermal_feedback")?,
            warm_start: warm_start(cfg)?,
            losses: Vec::new(),
            obstacles: crate::collision::obstacles(cfg)?,
            crash: cfg.bool("effects.crash")?,
            min_clearance: cfg.non_negative("profile.min_clearance_m")?,
            clearance: f64::INFINITY,
            least_clearance: f64::INFINITY,
            contact: None,
            crashed: false,
            contact_force: 0.0,
            gaps: Vec::new(),
            pushes: Vec::new(),
            push_from: Vec::new(),
            absorbed: 0.0,
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
        // Everything that can refuse the config runs before anything changes: a refused reload leaves the old sim.
        let mut heated = heated_parts(cfg, machine.as_ref())?;
        let part_heating = heats(cfg)?;
        let thermal_feedback = cfg.bool("effects.thermal_feedback")?;
        let warm_start = warm_start(cfg)?;
        let (obstacles, crash, min_clearance) = (
            crate::collision::obstacles(cfg)?,
            cfg.bool("effects.crash")?,
            cfg.non_negative("profile.min_clearance_m")?,
        );
        // The last fallible step; it changes the controller only once it has checked all it reads.
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
        self.rk_joint = Rk4::new(self.state.len() + WINDING_STATES * motors + 1);
        self.motor_buffers = std::array::from_fn(|_| vec![0.0; motors]);
        self.limit_violations = check_limits(machine.as_ref(), &self.control.motor_drives(machine.as_ref()));
        // A part that is still there keeps its warmth.
        for part in &mut heated {
            part.rise = self.heated.iter().find(|old| old.part.id == part.part.id).map_or(0.0, |old| old.rise);
        }
        (self.heated, self.part_heating) = (heated, part_heating);
        self.thermal_feedback = thermal_feedback;
        self.warm_start = warm_start;
        (self.obstacles, self.crash, self.min_clearance) = (obstacles, crash, min_clearance);
        self.machine = machine;
        self.feed_temperatures_back();
        self.substeps = substeps;
        Ok(())
    }

    pub fn time(&self) -> f64 {
        self.step as f64 * self.dt
    }

    /// Everything but the machine, to go back to with `restore`: a copy, as long as the simulation's state takes to copy.
    pub fn snapshot(&self) -> Snapshot {
        Snapshot {
            control: self.control.clone(),
            state: self.state.clone(),
            torque: self.torque.clone(),
            step: self.step,
            payload_attached: self.payload_attached,
            payload_energy: self.payload_energy,
            crash_energy: self.crash_energy,
            heated: self.heated.clone(),
            obstacles: self.obstacles.clone(),
            clearance: self.clearance,
            least_clearance: self.least_clearance,
            contact: self.contact.clone(),
            crashed: self.crashed,
            contact_force: self.contact_force,
            gaps: self.gaps.clone(),
            absorbed: self.absorbed,
        }
    }

    /// Back to a snapshot of a simulation of the same machine and config (this one, or one built as it was): its next
    /// steps are bit for bit those the snapshot's simulation took. A snapshot of another machine (other state, motor,
    /// part or obstacle counts) is refused and changes nothing. What a machine works out per step for its signals alone
    /// (a belt tooth jump's loss) reads as the machine has it until the next step: sample before restoring if the
    /// signals at the snapshot are wanted.
    pub fn restore(&mut self, s: &Snapshot) -> Result<(), ConfigError> {
        if s.state.len() != self.state.len()
            || s.torque.len() != self.torque.len()
            || s.control.drives.len() != self.control.drives.len()
            || s.heated.len() != self.heated.len()
            || s.obstacles.len() != self.obstacles.len()
        {
            return Err(ConfigError::Invalid("snapshot".into()));
        }
        self.control = s.control.clone();
        self.state.clone_from(&s.state);
        self.torque.clone_from(&s.torque);
        (self.step, self.payload_energy, self.crash_energy) = (s.step, s.payload_energy, s.crash_energy);
        // Each part as this machine has it, warmed as the snapshot's (a deserialized one carries no part).
        for (mine, theirs) in self.heated.iter_mut().zip(&s.heated) {
            *mine = HeatedPart { part: std::mem::take(&mut mine.part), ..theirs.clone() };
        }
        self.obstacles.clone_from(&s.obstacles);
        (self.clearance, self.least_clearance, self.crashed) = (s.clearance, s.least_clearance, s.crashed);
        self.contact.clone_from(&s.contact);
        self.gaps.clone_from(&s.gaps);
        (self.contact_force, self.absorbed) = (s.contact_force, s.absorbed);
        // What the machine keeps between steps, as `step` sets it.
        self.payload_attached = s.payload_attached;
        self.machine.set_payload_attached(s.payload_attached);
        self.machine.set_dwelling(self.control.dwell_progress().is_some());
        self.machine.set_brake_engaged(!self.control.enabled());
        self.feed_temperatures_back();
        Ok(())
    }

    pub fn command(&mut self, command: Command) -> Result<(), CommandError> {
        self.control.command(command, self.machine.as_ref())
    }

    /// The timing for a path from where the machine stands, to run with `Command::Trajectory`.
    /// A time-optimal path that comes nearer to an obstacle than `profile.min_clearance_m`, or touches one, is refused
    /// (`error-path-collision`); one timed within the axis limits is planned all the same, and `check_plan` tells.
    pub fn plan_path(
        &self,
        pieces: &[crate::path::Piece],
        time_optimal: bool,
    ) -> Result<Vec<(f64, Vec<f64>)>, CommandError> {
        let points = self.control.plan_path(self.machine.as_ref(), pieces, time_optimal)?;
        if time_optimal
            && let Some(check) = self.check_plan(&points.iter().map(|(_, pose)| pose.clone()).collect::<Vec<_>>())
            && (check.clearance < self.min_clearance || check.contact.is_some())
        {
            return Err(CommandError { key: "error-path-collision" });
        }
        Ok(points)
    }

    /// The timing for a program (`program::read`) from where the machine stands (`Controller::plan_program`), refused
    /// as `plan_path` refuses a time-optimal path near an obstacle.
    pub fn plan_program(
        &self,
        runs: &[crate::program::Run],
        time_optimal: bool,
    ) -> Result<Vec<(f64, Vec<f64>)>, CommandError> {
        let points = self.control.plan_program(self.machine.as_ref(), runs, time_optimal)?;
        if time_optimal
            && let Some(check) = self.check_plan(&points.iter().map(|(_, pose)| pose.clone()).collect::<Vec<_>>())
            && (check.clearance < self.min_clearance || check.contact.is_some())
        {
            return Err(CommandError { key: "error-path-collision" });
        }
        Ok(points)
    }

    /// The path through `poses` checked against the obstacles before it runs, with the payload as it is now
    /// (`collision::check_path`). `None` without obstacles.
    pub fn check_plan(&self, poses: &[Vec<f64>]) -> Option<crate::collision::PlanCheck> {
        if self.obstacles.is_empty() {
            return None;
        }
        let mut check = crate::collision::PlanCheck::default();
        crate::collision::check_path(self.machine.as_ref(), &self.obstacles, poses, &mut check);
        Some(check)
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
        self.pushes();
        if self.control.enabled() {
            self.advance_with_windings();
        } else if self.pushes.is_empty() {
            advance(self.machine.as_ref(), &mut self.rk, &mut self.state, &self.torque, self.dt, self.substeps);
        } else {
            self.advance_pushed();
        }
        self.machine.after_step(&mut self.state, self.dt);
        let taken = self.pushed();
        if !self.obstacles.is_empty() {
            self.check_obstacles();
        }
        if let Some(contact) = &mut self.contact {
            self.absorbed += taken;
            contact.peak_force = contact.peak_force.max(self.contact_force);
            contact.impact_energy = contact.impact_energy.max(self.absorbed);
        }
        self.crashed = self.control.fault.is_some() && (self.crashed || self.obstacles.iter().any(|o| o.contact));
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

    /// The clearance to the obstacles after a base step, which obstacles a part touches, and the first contact.
    fn check_obstacles(&mut self) {
        let bodies = crate::collision::Bodies::of(self.machine.as_ref(), &self.state);
        let mut nearest = (f64::INFINITY, 0);
        self.gaps.clear();
        for (k, obstacle) in self.obstacles.iter_mut().enumerate() {
            let gap = bodies.gap(&obstacle.aabb).map_or(f64::INFINITY, |g| g.2);
            obstacle.contact = gap <= 0.0;
            if gap < nearest.0 {
                nearest = (gap, k);
            }
            self.gaps.push(gap);
        }
        self.clearance = nearest.0;
        self.least_clearance = self.least_clearance.min(nearest.0);
        if nearest.0 <= 0.0 && self.contact.is_none() {
            let n = self.machine.axes().len();
            let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
            self.machine.tool_state(&self.state, &mut pos, &mut vel);
            let speed = vel.iter().map(|v| v * v).sum::<f64>().sqrt();
            // The step counter moves on after this check.
            let time = (self.step + 1) as f64 * self.dt;
            let (peak_force, impact_energy) = (self.contact_force, 0.0);
            self.contact = Some(Contact { time, pose: pos, speed, obstacle: nearest.1, peak_force, impact_energy });
        }
    }

    /// effects.crash: the obstacles' pushes over the coming step (`Push`), for each obstacle a part comes within
    /// `PUSH_RANGE` of. The depth and its slopes are those of the rigid kinematics at the motors' angles (`forward`,
    /// then `equilibrium`), so that the push is a spring on the motors' angles, which the step integrates with the rest.
    // shortcut: the push acts on the motors, not on the part that touches; a load on a soft belt or gearbox swings on
    // into the obstacle by its transmission's give (millimetres on a belt at 1 m/s) instead of stopping at its face.
    // Push on the part itself once `Machine` can take an external force at a body.
    fn pushes(&mut self) {
        self.pushes.clear();
        let near: Vec<usize> =
            (0..self.obstacles.len()).filter(|&k| self.gaps.get(k).is_some_and(|g| *g < PUSH_RANGE)).collect();
        if !self.crash || near.is_empty() {
            return;
        }
        let machine = self.machine.as_ref();
        let (motors, axes) = (machine.motors().len(), machine.axes().len());
        let (mut angle, mut speed) = (vec![0.0; motors], vec![0.0; motors]);
        machine.motor_state(&self.state, &mut angle, &mut speed);
        let (mut tool, mut state, mut torque) = (vec![0.0; axes], Vec::new(), vec![0.0; motors]);
        let mut gaps_at = |at: &[f64]| -> Vec<f64> {
            machine.forward(at, &mut tool);
            machine.equilibrium(&tool, &mut state, &mut torque);
            let bodies = crate::collision::Bodies::of(machine, &state);
            near.iter().map(|&k| bodies.gap(&self.obstacles[k].aabb).map_or(f64::INFINITY, |g| g.2)).collect()
        };
        let depths: Vec<f64> = gaps_at(&angle).into_iter().map(|g| -g).collect();
        let mut slopes = vec![vec![0.0; motors]; near.len()];
        for m in 0..motors {
            let mut at = angle.clone();
            at[m] += SLOPE_STEP;
            let up = gaps_at(&at);
            at[m] -= 2.0 * SLOPE_STEP;
            let down = gaps_at(&at);
            for (j, slope) in slopes.iter_mut().enumerate() {
                slope[m] = (up[j] - down[j]) / (2.0 * SLOPE_STEP);
            }
        }
        for ((&k, depth), slope) in near.iter().zip(depths).zip(slopes) {
            let (stiffness, damping) = (self.obstacles[k].stiffness, self.obstacles[k].damping);
            if depth.is_finite() && slope.iter().all(|g| g.is_finite()) {
                self.pushes.push(Push { depth, slope, stiffness, damping });
            }
        }
        self.push_from = angle;
    }

    /// The step done: the obstacles' force at its end (`contact_force`), and the work their pushes took out of the
    /// machine over it (J), integrated with the motion (the last entry of `joint`), which goes to `crash_energy`.
    fn pushed(&mut self) -> f64 {
        if self.pushes.is_empty() {
            self.contact_force = 0.0;
            return 0.0;
        }
        let motors = self.push_from.len();
        let (mut angle, mut speed, mut torque) = (vec![0.0; motors], vec![0.0; motors], vec![0.0; motors]);
        self.machine.motor_state(&self.state, &mut angle, &mut speed);
        self.contact_force = push_torques(&self.pushes, &self.push_from, &angle, &speed, &mut torque);
        let taken = self.joint.last().copied().unwrap_or(0.0);
        self.crash_energy += taken;
        taken
    }

    /// A step with the drives disabled and the obstacles pushing: the mechanics alone, the pushes' torques worked out at
    /// every stage of the integration, and the work they take out of the machine integrated with it (as with the
    /// windings, whose entries rest at zero here).
    fn advance_pushed(&mut self) {
        let Self {
            machine,
            rk_joint,
            joint,
            state,
            torque,
            pushes,
            push_from,
            motor_buffers: [angle, speed, stage, push],
            dt,
            substeps,
            ..
        } = self;
        let n = state.len();
        joint.clear();
        joint.extend_from_slice(state);
        joint.resize(n + WINDING_STATES * torque.len() + 1, 0.0);
        let h = *dt / *substeps as f64;
        for _ in 0..*substeps {
            rk_joint.step(
                |y, dy| {
                    let (mech, (dmech, rest)) = (&y[..n], dy.split_at_mut(n));
                    machine.motor_state(mech, angle, speed);
                    push.fill(0.0);
                    push_torques(pushes, push_from, angle, speed, push);
                    for k in 0..stage.len() {
                        stage[k] = torque[k] + push[k];
                    }
                    rest.fill(0.0);
                    rest[rest.len() - 1] = -push.iter().zip(speed.iter()).map(|(t, w)| t * w).sum::<f64>();
                    machine.derivatives(mech, stage, dmech);
                },
                joint,
                h,
            );
        }
        state.copy_from_slice(&joint[..n]);
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
        let Self {
            machine,
            control,
            state,
            torque,
            rk_joint,
            joint,
            motor_buffers: [angle, speed, stage_torque, push],
            dt,
            substeps,
            pushes,
            push_from,
            ..
        } = self;
        let (n, drives) = (state.len(), &control.drives);
        joint.clear();
        joint.extend_from_slice(state);
        for d in drives {
            joint.extend([d.current, 0.0, 0.0, 0.0]);
        }
        // The work the obstacles' pushes take out of the machine over the step (`pushed`).
        joint.push(0.0);
        let h = *dt / *substeps as f64;
        for _ in 0..*substeps {
            rk_joint.step(
                |y, dy| {
                    let ((mech, windings), (dmech, dwindings)) = (y.split_at(n), dy.split_at_mut(n));
                    machine.motor_state(mech, angle, speed);
                    for (k, d) in drives.iter().enumerate() {
                        // A drive whose pulses are off after a trip: no current, no torque (`Controller::react`).
                        if d.pulses_off {
                            stage_torque[k] = 0.0;
                            dwindings[WINDING_STATES * k..WINDING_STATES * (k + 1)].fill(0.0);
                            continue;
                        }
                        let (i, p) = (windings[WINDING_STATES * k], control.drive_params_of(k));
                        stage_torque[k] = p.torque_with(i, d.d_current, d.flux(p));
                        dwindings[WINDING_STATES * k..WINDING_STATES * (k + 1)].copy_from_slice(&p.winding_rates(
                            d.voltage,
                            d.winding_resistance(p),
                            i,
                            speed[k],
                            d.d_current,
                            d.flux(p),
                        ));
                    }
                    push.fill(0.0);
                    push_torques(pushes, push_from, angle, speed, push);
                    for (stage, push) in stage_torque.iter_mut().zip(push.iter()) {
                        *stage += push;
                    }
                    let work = dwindings.len() - 1;
                    dwindings[work] = -push.iter().zip(speed.iter()).map(|(t, w)| t * w).sum::<f64>();
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
                out.push(SignalInfo::new(
                    format!("axis.{}.{}", axis.id, name.replace('-', "_")),
                    axis.key,
                    &format!("quantity-{name}"),
                    unit,
                ));
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
                ("torque-feedforward", "N m"),
                ("torque-feedback", "N m"),
                ("speed-from-position", "rad/s"),
                // The electrical and thermal states, 0 where their effect is off.
                ("d-current", "A"),
                ("housing-temperature", "K"),
                ("drive-overload-image", ""),
                ("drive-overload-chip", ""),
                ("iron-loss", "W"),
                ("braking-resistor-temperature", "K"),
            ] {
                out.push(SignalInfo::new(
                    format!("motor.{}.{}", motor.id, name.replace('-', "_")),
                    motor.key,
                    &format!("quantity-{name}"),
                    unit,
                ));
            }
        }
        out.extend(self.machine.signal_infos());
        out.extend(self.heated.iter().map(|h| {
            SignalInfo::new(
                format!("{TEMPERATURE_PREFIX}{}", h.part.id),
                h.part.owner_key,
                &format!("quantity-{}-temperature", h.part.kind),
                "K",
            )
        }));
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
            out.extend([
                angle[k],
                speed[k],
                d.torque,
                d.torque_command,
                d.current,
                d.voltage,
                d.power,
                d.temperature_rise,
                d.dc_voltage,
                d.resistor_power,
                d.speed_command,
                d.following_error,
                d.position_setpoint,
                d.feedforward_torque,
                d.controller_torque,
                d.position_part,
                d.d_current,
                d.housing_rise,
                d.overload_image,
                d.overload_chip,
                d.iron_loss,
                d.resistor_rise,
            ]);
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
        out.obstacles.clone_from(&self.obstacles);
        let bodies = crate::collision::Bodies::of(self.machine.as_ref(), &self.state);
        out.clearance = crate::collision::nearest(&bodies, &self.obstacles).map(|(_, gap)| gap);
        out.first_contact = self.contact.as_ref().map(|c| c.time);
        let mut colour = |id: &str, index: usize, heat: f64| {
            if let Some(part) = out.parts.iter_mut().filter(|p| p.id == id).nth(index) {
                part.heat = Some(heat as f32);
            }
        };
        if self.control.drive_params().heating {
            for (k, d) in self.control.drives.iter().enumerate() {
                colour(sim_core::scene::MOTOR_PART, k, d.temperature_rise / self.control.drive_params_of(k).rated_rise);
            }
        }
        if self.part_heating {
            for h in &self.heated {
                colour(h.part.scene_part, h.part.scene_index, h.rise / h.max_rise);
            }
        }
        self.conveyor_scene(out);
    }

    /// With `[conveyor]`: the belt as a band under the grip poses, slats across it that run with it, and the parts on
    /// it, where the machine maps tool poses into the scene.
    fn conveyor_scene(&self, out: &mut Scene) {
        use sim_core::scene::{Part, Shape};
        let Some((conveyor, pass)) = self.control.conveyor() else { return };
        let along =
            |s: f64| -> Vec<f64> { conveyor.origin.iter().zip(&conveyor.direction).map(|(o, d)| o + d * s).collect() };
        let (Some(start), Some(end)) =
            (self.machine.scene_point(&along(0.0)), self.machine.scene_point(&along(conveyor.length)))
        else {
            return;
        };
        // The parts are 50 mm cubes under the grip pose; the band lies under them, 300 mm wide.
        let (part, width) = (0.05, 0.3);
        let below = |p: [f64; 3], by: f64| [p[0], p[1], p[2] - by];
        out.parts.push(Part::new(
            "conveyor-belt",
            Shape::Segment { to: below(end, part), width },
            below(start, part),
            [0.15, 0.15, 0.17],
        ));
        // Each slat a strip 10 mm wide square across the belt, however the belt runs in the floor's plane.
        let unit = [end[0] - start[0], end[1] - start[1]];
        let norm = unit[0].hypot(unit[1]).max(1e-9);
        let across = [-unit[1] / norm * width / 2.0, unit[0] / norm * width / 2.0];
        let spacing = 0.1;
        let shift = conveyor.belt(self.time()) % spacing;
        for i in 0..(conveyor.length / spacing).floor() as usize {
            if let Some(p) = self.machine.scene_point(&along(shift + i as f64 * spacing)) {
                let [x, y, z] = below(p, part);
                let (from, to) = ([x - across[0], y - across[1], z], [x + across[0], y + across[1], z]);
                out.parts.push(Part::new(
                    "conveyor-slat",
                    Shape::Segment { to, width: 0.01 },
                    from,
                    [0.35, 0.35, 0.38],
                ));
            }
        }
        for (k, _) in conveyor.parts_on_belt(self.time(), pass) {
            if let Some(p) = self.machine.scene_point(&conveyor.part_pose(k, self.time())) {
                let size = [part; 3];
                out.parts.push(Part::new("conveyor-part", Shape::Box { size }, below(p, part / 2.0), [0.9, 0.55, 0.1]));
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
