//! A machine, its controller and the integrator, stepped together. Every front end runs this.

use crate::control::{Command, CommandError, Controller, LimitViolation, check_limits, in_range};
use sim_core::{
    Config, ConfigError, LOSS_PREFIX, Machine, MachineEntry, Obstacle, Rk4, Scene, SignalInfo, TEMPERATURE_PREFIX,
    ThermalPart, advance,
};

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

/// How far a part in contact with an obstacle may go further into it, m, in all, before it counts as moving in
/// (effects.crash, `Sim::check_obstacles`): drives enabled again at rest against the obstacle settle some 25 nm into
/// it (and rounding moves it by 1e-16 m), and they tripped on that. A drive that pushes on goes further.
const CREEP: f64 = 1e-6;

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
    /// The machine's energy that the stops at obstacles took since the start (effects.crash, `check_obstacles`): lost
    /// in the impact. For the energy budget.
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
    /// effects.crash: the obstacles are rigid; a part moving into one stops dead and the drives go off
    /// (`check_obstacles`).
    pub crash: bool,
    /// `profile.min_clearance_m`: the least clearance to the obstacles a time-optimal path may keep (`plan_path`).
    pub min_clearance: f64,
    /// The clearance to the nearest obstacle after the last step, and the least since the start, m; infinite without
    /// obstacles. Checked every base step.
    pub clearance: f64,
    pub least_clearance: f64,
    /// The first contact with an obstacle since the start.
    pub contact: Option<Contact>,
    /// Whether the drives went off at an obstacle (effects.crash) and have not been enabled since.
    pub crashed: bool,
    /// Per obstacle the clearance, and the tool's pose, after the last step: whether a part moves into an obstacle,
    /// and where along the step it met its face (`check_obstacles`).
    gaps: Vec<f64>,
    last_pose: Vec<f64>,
}

/// The first contact of a run with an obstacle: when (the end of the base step that found it, s), at which tool pose,
/// how fast the tool went then (the length of its speed vector, axis units per second) and which obstacle.
#[derive(Debug, Clone, PartialEq)]
pub struct Contact {
    pub time: f64,
    pub pose: Vec<f64>,
    pub speed: f64,
    pub obstacle: usize,
}

/// What a running simulation has come to apart from its machine (`Sim::snapshot`): the integrated state, the controller
/// with its drives (loops, filters, windings, thermal and overload images), the parts' warmth and the obstacle record.
/// `Sim::restore` takes it back into a simulation built from the same machine and config; what the machine itself keeps
/// between steps (payload, brakes, dwelling, the parts' temperatures) is set again from it.
#[derive(Clone)]
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
    gaps: Vec<f64>,
    last_pose: Vec<f64>,
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
        let limit_violations = check_limits(machine.as_ref(), &control.motor_drives(machine.as_ref()));
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
            gaps: Vec::new(),
            last_pose: Vec::new(),
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
        self.rk_joint = Rk4::new(self.state.len() + WINDING_STATES * motors);
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
            gaps: self.gaps.clone(),
            last_pose: self.last_pose.clone(),
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
        self.heated.clone_from(&s.heated);
        self.obstacles.clone_from(&s.obstacles);
        (self.clearance, self.least_clearance, self.crashed) = (s.clearance, s.least_clearance, s.crashed);
        self.contact.clone_from(&s.contact);
        self.gaps.clone_from(&s.gaps);
        self.last_pose.clone_from(&s.last_pose);
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
        self.crashed &= !self.control.enabled();
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
        if !self.obstacles.is_empty() {
            self.check_obstacles();
        }
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

    /// The clearance to the obstacles after a base step, which obstacles a part touches, and the first contact. With
    /// effects.crash the obstacles are rigid: a part that touches one and moves further into it (its clearance more
    /// than `CREEP` less than after the step before) stops dead, a perfectly plastic impact, whether the drives are on
    /// or off. The
    /// machine is put at rest (`Machine::equilibrium`, so belts and gears hold what they held at rest there) at the
    /// pose along the step where the part met the face, and drives that were on go off, as a real drive trips on the
    /// following error of a blocked axis; holding brakes engage. A part in contact that stands or moves out of the
    /// obstacle is left to move, so drives enabled again after a crash can take the machine away. The energy the stop
    /// takes (the kinetic energy, and what springs held beyond their rest) is lost in the impact: `crash_energy`.
    fn check_obstacles(&mut self) {
        let bodies = self.machine.collision_bodies(&self.state);
        let gap_to = |bodies: &[sim_core::Aabb], o: &Obstacle| {
            bodies.iter().map(|b| b.gap(&o.aabb).2).fold(f64::INFINITY, f64::min)
        };
        let mut gaps: Vec<f64> = self.obstacles.iter().map(|o| gap_to(&bodies, o)).collect();
        let (mut nearest, mut stop) = ((f64::INFINITY, 0), None::<f64>);
        let (known, mut held) = (self.gaps.len() == gaps.len(), false);
        for (k, (obstacle, gap)) in self.obstacles.iter_mut().zip(&mut gaps).enumerate() {
            obstacle.contact = *gap <= 0.0;
            if *gap < nearest.0 {
                nearest = (*gap, k);
            }
            let Some(&before) = self.gaps.get(k).filter(|_| known) else { continue };
            if *gap <= 0.0 && *gap < before - CREEP {
                // Where the clearance reached 0, linearly from the pose it was measured at (the last step's, or where
                // the part stood in contact before it crept in); at that pose if it was in contact already.
                let at = (before / (before - *gap)).clamp(0.0, 1.0);
                stop = Some(stop.map_or(at, |s| s.min(at)));
            } else if *gap <= 0.0 && *gap < before {
                // In contact and a little further in: judged from where it was, so that creeping in adds up.
                (*gap, held) = (before, true);
            }
        }
        self.clearance = nearest.0;
        self.least_clearance = self.least_clearance.min(nearest.0);
        let n = self.machine.axes().len();
        let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
        self.machine.tool_state(&self.state, &mut pos, &mut vel);
        if nearest.0 <= 0.0 && self.contact.is_none() {
            let speed = vel.iter().map(|v| v * v).sum::<f64>().sqrt();
            // The step counter moves on after this check.
            self.contact =
                Some(Contact { time: (self.step + 1) as f64 * self.dt, pose: pos.clone(), speed, obstacle: nearest.1 });
        }
        self.gaps = gaps;
        if self.crash
            && let Some(at) = stop
            && self.last_pose.len() == n
        {
            pos = self.last_pose.iter().zip(&pos).map(|(from, to)| from + at * (to - from)).collect();
            let mut torque = vec![0.0; self.machine.motors().len()];
            let before = self.machine.energy(&self.state);
            self.machine.equilibrium(&pos, &mut self.state, &mut torque);
            self.crash_energy += before - self.machine.energy(&self.state);
            if self.control.enabled() {
                let _ = self.control.command(Command::Disable, self.machine.as_ref());
                self.crashed = true;
            }
            // Moving on from the face is judged from where the stop left the parts.
            let bodies = self.machine.collision_bodies(&self.state);
            for (gap, o) in self.gaps.iter_mut().zip(&self.obstacles) {
                *gap = gap_to(&bodies, o);
            }
            held = false;
        }
        if !held || self.last_pose.len() != n {
            self.last_pose = pos;
        }
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
            motor_buffers: [angle, speed, stage_torque],
            dt,
            substeps,
            ..
        } = self;
        let (n, drives) = (state.len(), &control.drives);
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
        out.clearance =
            crate::collision::nearest(&self.machine.collision_bodies(&self.state), &self.obstacles).map(|(_, gap)| gap);
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
        // shortcut: the slats are boxes along the scene's axes, right for a belt along x or y and too wide on a slant;
        // turned boxes if a slanted belt is ever drawn.
        let unit = [end[0] - start[0], end[1] - start[1]];
        let norm = unit[0].hypot(unit[1]).max(1e-9);
        let (dx, dy) = (unit[0].abs() / norm, unit[1].abs() / norm);
        let size = [0.01 * dx + width * dy, 0.01 * dy + width * dx, 0.006];
        let spacing = 0.1;
        let shift = conveyor.belt(self.time()) % spacing;
        for i in 0..(conveyor.length / spacing).floor() as usize {
            if let Some(p) = self.machine.scene_point(&along(shift + i as f64 * spacing)) {
                out.parts.push(Part::new("conveyor-slat", Shape::Box { size }, below(p, part), [0.35, 0.35, 0.38]));
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
