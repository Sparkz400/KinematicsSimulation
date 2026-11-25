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
        let (resistance, time_constant) = match machine.thermal_values(part.id) {
            Some(own) => own,
            None => (cfg.non_negative(&key("resistance_k_w"))?, cfg.positive(&key("time_constant_s"))?),
        };
        let max_rise = cfg.positive(&key("max_rise_k"))?;
        parts.push(HeatedPart { part, loss, loss_power: 0.0, resistance, time_constant, max_rise, rise: 0.0 });
    }
    Ok(parts)
}

/// How near an obstacle a part has to come before the obstacle's push is worked out (effects.crash, `Sim::pushes`), m:
/// the push acts on the motors through the rigid kinematics, whose parts may be some millimetres from the real ones
/// (a carriage swinging on its belt).
const PUSH_RANGE: f64 = 0.01;

/// How many times finer the integration's sub-steps are over a base step with a push at the tool: the contact's
/// spring then acts on the load's own coordinates (not through the transmission's give), the highest frequency of the
/// machine, and with `sim.substeps` = 4 the fourth-order steps took 3.5e-4 of the supply out of the budget of a belt
/// gantry's move into a slab (2.8e-4 on a six-leg platform; falling with the third and second power of the sub-step),
/// 5e-6 and 1.9e-5 four times finer (`sim-machines`' collision tests).
const PUSH_SUBSTEPS: u32 = 4;
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

/// The pushes' force on the contact coordinates (`Machine::contact_state`, the tool's by default) at `state`, one per
/// coordinate (the generalised force: each push's force times its clearance's slope along it), the coordinates they
/// were worked out at, and the sum of the forces, N.
fn tool_push_forces(machine: &dyn Machine, pushes: &[Push], from: &[f64], state: &[f64]) -> (Vec<f64>, Vec<f64>, f64) {
    let n = machine.contact_axes();
    let (mut pos, mut vel, mut force, mut total) = (vec![0.0; n], vec![0.0; n], vec![0.0; n], 0.0);
    machine.contact_state(state, &mut pos, &mut vel);
    for push in pushes {
        let f = push.force(from, &pos, &vel);
        total += f;
        force.iter_mut().zip(&push.slope).for_each(|(t, g)| *t += f * g);
    }
    (force, pos, total)
}

/// At one stage of the integration: the pushes' force on the tool at the mechanical state `mech`, given to the machine
/// (`Machine::set_tool_force`), as the motors' pushes are worked out at every stage; returns the power they take out of
/// the machine, -F . v, W. `pos`, `vel` and `force` are buffers as long as the contact coordinates.
fn tool_push_stage(
    machine: &mut dyn Machine,
    pushes: &[Push],
    from: &[f64],
    mech: &[f64],
    [pos, vel, force]: [&mut Vec<f64>; 3],
) -> f64 {
    machine.contact_state(mech, pos, vel);
    force.fill(0.0);
    for push in pushes {
        let f = push.force(from, pos, vel);
        force.iter_mut().zip(&push.slope).for_each(|(t, g)| *t += f * g);
    }
    machine.set_tool_force(force);
    -force.iter().zip(vel.iter()).map(|(f, v)| f * v).sum::<f64>()
}

// Per drive in the integrated state: winding current, and since the start of the step the electrical energy, the
// copper loss and the work on the shaft; then a stepper's or an induction motor's further states.
use crate::drive::WINDING_STATES;

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
    /// The tool's pose, speed and push force at a stage of the integration (`tool_push_stage`), as long as the axes.
    tool_buffers: [Vec<f64>; 3],
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
    /// The path the config's `[path]` gives in place of the cycle (`with_path`): what the analysis of the cycle runs
    /// instead (`analysis::analyse_cycle_with`), and so the verdicts, the search and the optimiser. `None`: the cycle.
    pub path: Option<PathRun>,
    /// The clearance to the nearest obstacle after the last step, and the least since the start, m; infinite without
    /// obstacles. Checked every base step.
    pub clearance: f64,
    pub least_clearance: f64,
    /// The first contact with an obstacle since the start.
    pub contact: Option<Contact>,
    /// The machine's own parts against each other (`collision::PlanCheck::own`): joined as they are at the start pose,
    /// looked at after the first base step and then every `OWN_PARTS_EVERY_S`, in the simulated state (a load that
    /// swings, a machine that coasts); its least since the start is the verdict "self-collision".
    pub own_parts: crate::collision::PlanCheck,
    /// When the own parts were last looked at (s), `None` before the first look.
    own_at: Option<f64>,
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
    /// Whether the pushes of the step under way act on the tool's coordinates (`Machine::set_tool_force`; `push_from` and
    /// the slopes are then about the tool pose) and not on the motors.
    tool_push: bool,
    /// The work the pushes have taken since the first contact, J.
    absorbed: f64,
    /// How many parts each sub-step of the step under way is cut into (`Machine::substeps_wanted`, at most `MOST_CUTS`).
    cuts: u32,
    /// How many base steps were cut finer for the machine (`cuts` above 1) since the start.
    pub refined_steps: u64,
}

/// The first contact of a run with an obstacle: when (the end of the base step that found it, s), at which tool pose,
/// how fast the tool went into it (the length of its speed vector at the start of that step, before a push braked it,
/// axis units per second) and which obstacle. With
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
/// between steps (payload, brakes, dwelling, the parts' temperatures, its `memory`) is set again from it.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Snapshot {
    control: Controller,
    state: Vec<f64>,
    /// `Machine::memory`: the gearboxes' hysteresis anchors, a tool-space arm's last inverse solution. Without them a
    /// restored delta ran on some micrometres off the run it was taken from, the palletiser by rounding.
    memory: Vec<f64>,
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
    #[serde(default)]
    own: Option<crate::collision::SelfClearance>,
    #[serde(default)]
    own_at: Option<f64>,
    crashed: bool,
    contact_force: f64,
    gaps: Vec<f64>,
    absorbed: f64,
}

/// A path in place of the cycle (`[path]` of a config, see `with_path`): the text of a path file or a program
/// (`program::read`), the file name that says which reader takes it, and whether it is timed as fast as the motors allow.
#[derive(Debug, Clone, PartialEq)]
pub struct PathRun {
    pub file: String,
    pub program: String,
    pub time_optimal: bool,
    /// `program.orientation_speed_rad_s`: how fast a robot program turns the tool (rad/s), 500 degrees per second where
    /// the config has no key (`program::ORIENTATION_SPEED`).
    pub orientation_speed: f64,
}

/// The config with `[path]` set: the analysis of the cycle runs this path (file name and text, as `--path FILE` reads
/// it) instead, timed within the axis limits or with `time_optimal` as fast as the motors allow. An optional table
/// (`analysis::RATINGS_BEYOND_DEFAULT`): a config without it runs its cycle.
pub fn with_path(cfg: &Config, file: &str, program: &str, time_optimal: bool) -> Result<Config, ConfigError> {
    let mut table: toml::Table =
        cfg.to_text()?.parse().map_err(|e: toml::de::Error| ConfigError::Parse(e.to_string()))?;
    let mut path = toml::Table::new();
    path.insert("file".into(), file.into());
    path.insert("program".into(), program.into());
    path.insert("time_optimal".into(), time_optimal.into());
    table.insert("path".into(), toml::Value::Table(path));
    Config::parse(&toml::to_string(&table).map_err(|e| ConfigError::Parse(e.to_string()))?)
}

/// The config's `[path]` (`with_path`), `None` without it or with no program text.
fn path_run(cfg: &Config) -> Result<Option<PathRun>, ConfigError> {
    let text = |key: &str| -> Result<String, ConfigError> {
        let path = format!("path.{key}");
        Ok(cfg
            .get(&path)
            .ok()
            .map(|v| v.as_str().map(str::to_string).ok_or(ConfigError::Type(path)))
            .transpose()?
            .unwrap_or_default())
    };
    let program = text("program")?;
    if program.trim().is_empty() {
        return Ok(None);
    }
    let time_optimal = cfg.get("path.time_optimal").is_ok() && cfg.bool("path.time_optimal")?;
    let orientation_speed = crate::program::orientation_speed_of(cfg)?;
    Ok(Some(PathRun { file: text("file")?, program, time_optimal, orientation_speed }))
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
    // The configs ship 2 to 4; a huge count (1e30 saturated to 4e9 a step) made every step hang.
    if substeps.fract() != 0.0 || substeps > 1000.0 {
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
/// An id of a table that the machine built from `cfg` has no motor of is refused (`ConfigError::Invalid` with
/// `<table>.<id>`), so a misspelt one is not silently unread. Where `cfg` does not build, the ids are not checked here:
/// building it reports that error itself.
pub fn schema(entry: &MachineEntry, cfg: &Config) -> Result<Config, ConfigError> {
    let mut text = entry.default_config.to_string();
    let default = entry.config();
    let motors: Option<Vec<String>> = crate::catalog::resolve(entry, cfg)
        .and_then(|resolved| (entry.build)(&resolved))
        .ok()
        .map(|machine| machine.motors().iter().map(|m| m.id.to_string()).collect());
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
            if motors.as_ref().is_some_and(|m| !m.contains(id)) {
                return Err(ConfigError::Invalid(format!("{table}.{id}")));
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

/// How often the run looks at the machine's own parts (`Sim::own_parts`), s: a look draws the machine's parts, which on
/// an arm costs about as much as a step. At 1 m/s the machine moves a millimetre in this time, the planned path's
/// spacing.
pub const OWN_PARTS_EVERY_S: f64 = 1e-3;

impl Sim {
    /// The machine at rest at its configured start pose.
    pub fn new(entry: &MachineEntry, cfg: &Config) -> Result<Self, ConfigError> {
        // Parts the config names bring their catalogue values; the caller's config keeps the names.
        let cfg = &crate::catalog::resolve(entry, cfg)?;
        let machine = (entry.build)(cfg)?;
        let (dt, substeps) = timing(cfg)?;
        let start = start_pose(cfg, machine.as_ref())?;
        let min_clearance = cfg.non_negative("profile.min_clearance_m")?;
        let own_parts = crate::collision::PlanCheck::at_start(min_clearance, (entry.build)(cfg)?.as_mut(), &start);
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
            tool_buffers: Default::default(),
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
            min_clearance,
            path: path_run(cfg)?,
            clearance: f64::INFINITY,
            least_clearance: f64::INFINITY,
            contact: None,
            own_parts,
            own_at: None,
            crashed: false,
            contact_force: 0.0,
            gaps: Vec::new(),
            pushes: Vec::new(),
            push_from: Vec::new(),
            tool_push: false,
            absorbed: 0.0,
            cuts: 1,
            refined_steps: 0,
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
        // Not used by a running simulation but for the own parts' joints, and a config it would not start with is
        // refused all the same.
        let start = start_pose(cfg, machine.as_ref())?;
        // Everything that can refuse the config runs before anything changes: a refused reload leaves the old sim.
        let mut heated = heated_parts(cfg, machine.as_ref())?;
        let part_heating = heats(cfg)?;
        let thermal_feedback = cfg.bool("effects.thermal_feedback")?;
        let warm_start = warm_start(cfg)?;
        let path = path_run(cfg)?;
        let (obstacles, crash, min_clearance) = (
            crate::collision::obstacles(cfg)?,
            cfg.bool("effects.crash")?,
            cfg.non_negative("profile.min_clearance_m")?,
        );
        let mut own_parts = crate::collision::PlanCheck::at_start(min_clearance, (entry.build)(cfg)?.as_mut(), &start);
        // The last fallible step; it changes the controller only once it has checked all it reads.
        self.control.apply_config(cfg, machine.as_ref())?;
        own_parts.own = self.own_parts.own.take();
        self.own_parts = own_parts;
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
        (self.warm_start, self.path) = (warm_start, path);
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
            memory: self.machine.memory(),
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
            own: self.own_parts.own.clone(),
            own_at: self.own_at,
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
            || s.memory.len() != self.machine.memory().len()
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
        self.own_parts.own.clone_from(&s.own);
        self.own_at = s.own_at;
        self.gaps.clone_from(&s.gaps);
        (self.contact_force, self.absorbed) = (s.contact_force, s.absorbed);
        // The pushes are those of the next step, which works them out afresh; until then the restored state has none.
        self.pushes.clear();
        // What the machine keeps between steps, as `step` sets it.
        self.payload_attached = s.payload_attached;
        self.machine.set_memory(&s.memory);
        self.machine.set_payload_attached(s.payload_attached);
        self.machine.set_dwelling(self.control.dwell_progress().is_some());
        self.machine.set_brake_engaged(self.control.brake_engaged());
        self.feed_temperatures_back();
        Ok(())
    }

    pub fn command(&mut self, command: Command) -> Result<(), CommandError> {
        self.control.command(command, self.machine.as_ref())
    }

    /// Puts motor `k`'s drive in `mode`: torque or speed setpoints from an outside controller (`Controller::set_drive_mode`).
    pub fn set_drive_mode(&mut self, k: usize, mode: crate::DriveMode) -> Result<(), CommandError> {
        self.control.set_drive_mode(k, mode, self.machine.as_ref(), &self.state)
    }

    /// The config's `[path]` (`with_path`) planned from where the machine stands, as `sim-cli run --path` plans a file:
    /// one run at the axes' limits as a path, a program with feeds run by run (`plan_program`). `None` without one.
    pub fn plan_configured_path(&self) -> Option<Result<Vec<(f64, Vec<f64>)>, CommandError>> {
        let run = self.path.as_ref()?;
        let start = self.control.planned_pos();
        let runs =
            match crate::program::read_with(&run.file, &run.program, self.machine.axes(), start, run.orientation_speed)
            {
                Ok(runs) => runs,
                Err(e) => return Some(Err(CommandError { key: e.key })),
            };
        Some(match crate::program::as_path(&runs) {
            Some(pieces) => self.plan_path(pieces, run.time_optimal),
            None => self.plan_program(&runs, run.time_optimal),
        })
    }

    /// The timing for a path from where the machine stands, to run with `Command::Trajectory`.
    /// A time-optimal path that comes nearer to an obstacle than `profile.min_clearance_m`, or touches one, is refused
    /// (`error-path-collision`), and so is one that brings two of the machine's own parts as near
    /// (`error-path-self-collision`); one timed within the axis limits is planned all the same, and `check_plan` tells.
    pub fn plan_path(
        &self,
        pieces: &[crate::path::Piece],
        time_optimal: bool,
    ) -> Result<Vec<(f64, Vec<f64>)>, CommandError> {
        let points = self.control.plan_path(self.machine.as_ref(), pieces, time_optimal)?;
        self.refuse_near(&points, time_optimal)?;
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
        self.refuse_near(&points, time_optimal)?;
        Ok(points)
    }

    /// `plan_path`'s refusal of a time-optimal plan that comes nearer than `profile.min_clearance_m` to an obstacle or
    /// brings two of the machine's own parts that near, or touches; nothing to look at without obstacles or own parts.
    fn refuse_near(&self, points: &[(f64, Vec<f64>)], time_optimal: bool) -> Result<(), CommandError> {
        if !time_optimal || (self.obstacles.is_empty() && !self.own_parts.has_pairs()) {
            return Ok(());
        }
        let Some(check) = self.check_plan(&points.iter().map(|(_, pose)| pose.clone()).collect::<Vec<_>>()) else {
            return Ok(());
        };
        if check.clearance < self.min_clearance || check.contact.is_some() {
            return Err(CommandError { key: "error-path-collision" });
        }
        if check.own_fails(self.min_clearance) {
            return Err(CommandError { key: "error-path-self-collision" });
        }
        Ok(())
    }

    /// The path through `poses` checked against the obstacles before it runs, with the payload as it is now
    /// (`collision::check_path`), and the machine's own parts against each other (`PlanCheck::own`, joined as at the
    /// start pose). `None` without obstacles where nothing of the machine's own was checked.
    pub fn check_plan(&self, poses: &[Vec<f64>]) -> Option<crate::collision::PlanCheck> {
        if self.obstacles.is_empty() && !self.own_parts.has_pairs() {
            return None;
        }
        let mut check = self.own_parts.anew();
        crate::collision::check_path(self.machine.as_ref(), &self.obstacles, poses, &mut check);
        (!self.obstacles.is_empty() || check.own.is_some()).then_some(check)
    }

    /// How far that planned path is from the pieces as given, at most.
    pub fn path_deviation(&self, pieces: &[crate::path::Piece], time_optimal: bool) -> Result<f64, CommandError> {
        self.control.path_deviation(self.machine.as_ref(), pieces, time_optimal)
    }

    /// `path_deviation` per run of a program (`Controller::program_deviations`).
    pub fn program_deviations(
        &self,
        runs: &[crate::program::Run],
        time_optimal: bool,
    ) -> Result<Vec<f64>, CommandError> {
        self.control.program_deviations(self.machine.as_ref(), runs, time_optimal)
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
        self.control.step_brake(self.dt);
        self.machine.set_brake_engaged(self.control.brake_engaged());
        for (k, &command) in self.control.valves.iter().enumerate() {
            self.machine.set_valve(k, command);
        }
        // The temperatures the last step ended with; fed back here and not after the parts warm, so that the losses
        // the parts warmed with are those the machine reports after the step.
        self.feed_temperatures_back();
        // The tool's speed before the step, the impact's if it finds the first contact (its push may brake the tool in
        // the step already, where it acts at the tool).
        let approach = (self.contact.is_none() && !self.obstacles.is_empty()).then(|| {
            let n = self.machine.axes().len();
            let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
            self.machine.tool_state(&self.state, &mut pos, &mut vel);
            vel.iter().map(|v| v * v).sum::<f64>().sqrt()
        });
        self.pushes();
        self.cuts = (self.machine.substeps_wanted(&self.state, &self.torque, self.dt, self.substeps))
            .clamp(1, sim_core::MOST_CUTS);
        self.refined_steps += u64::from(self.cuts > 1);
        if self.control.enabled() {
            self.advance_with_windings();
        } else {
            // A disabled drive's torque (the iron's drag, `Drive::disabled_step`) is held over the step: its work is that
            // torque times the angle turned, which the drive's books take in place of the step start's speed.
            let dragged = self.control.drives.iter().any(|d| d.torque != 0.0).then(|| {
                let [angle, speed, ..] = &mut self.motor_buffers;
                self.machine.motor_state(&self.state, angle, speed);
                angle.clone()
            });
            if self.pushes.is_empty() {
                let substeps = self.substeps * self.cuts;
                advance(self.machine.as_ref(), &mut self.rk, &mut self.state, &self.torque, self.dt, substeps);
            } else {
                self.advance_pushed();
            }
            if let Some(before) = dragged {
                let [angle, speed, ..] = &mut self.motor_buffers;
                self.machine.motor_state(&self.state, angle, speed);
                for ((d, a), b) in self.control.drives.iter_mut().zip(angle.iter()).zip(&before) {
                    if d.torque != 0.0 {
                        let work = d.torque * (a - b) / self.dt;
                        (d.shaft_power, d.emf_loss) = (work, -work);
                    }
                }
            }
        }
        self.machine.after_step(&mut self.state, self.dt);
        let taken = self.pushed();
        if !self.obstacles.is_empty() {
            self.check_obstacles(approach.unwrap_or(0.0));
        }
        if self.own_parts.has_pairs() {
            self.look_at_own_parts();
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
            let torque = self.machine_torque();
            self.machine.loss_powers(&self.state, &torque, &mut self.losses);
            for part in &mut self.heated {
                part.loss_power = self.losses[part.loss];
                let settled = part.resistance * part.loss_power;
                part.rise += (1.0 - (-self.dt / part.time_constant).exp()) * (settled - part.rise);
            }
        }
        self.step += 1;
    }

    /// The machine's own parts after a base step (`own_parts`), where `OWN_PARTS_EVERY_S` has passed since the last look
    /// or there was none. The pose said with a finding is the setpoint's.
    fn look_at_own_parts(&mut self) {
        let now = self.time();
        if self.own_at.is_some_and(|at| now - at < OWN_PARTS_EVERY_S) {
            return;
        }
        let boxes = self.machine.collision_bodies(&self.state);
        self.own_parts.look_at_own(self.machine.as_ref(), &self.state, &boxes, &self.control.set_pos);
        self.own_at = Some(now);
    }

    /// The clearance to the obstacles after a base step, which obstacles a part touches, and the first contact, the tool
    /// at `speed` before the step.
    fn check_obstacles(&mut self, speed: f64) {
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
            // The step counter moves on after this check.
            let time = (self.step + 1) as f64 * self.dt;
            let (peak_force, impact_energy) = (self.contact_force, 0.0);
            self.contact = Some(Contact { time, pose: pos, speed, obstacle: nearest.1, peak_force, impact_energy });
        }
    }

    /// effects.crash: the obstacles' pushes over the coming step (`Push`), for each obstacle a part comes within
    /// `PUSH_RANGE` of. The depth and its slopes are those of the rigid kinematics at the motors' angles (`forward`,
    /// then `equilibrium`), so that the push is a spring on the motors' angles, which the step integrates with the rest.
    /// A machine that takes a force at its tool (`Machine::set_tool_force`) is pushed there: the push's slopes are then
    /// the clearance's along the tool's coordinates, its depth the clearance of the parts where they are, its force the
    /// generalised force of the contact on them (its virtual work per unit of each, wherever on the moving parts it
    /// touches), worked out at every stage of the integration, and a load on a soft belt stops at the obstacle's face.
    /// A machine without it is pushed at its
    /// motors through the rigid kinematics, so that a load on a soft transmission swings on into the obstacle by the
    /// transmission's give. Either way the work the pushes take is the budget's crash row.
    fn pushes(&mut self) {
        self.pushes.clear();
        let (axes, contact) = (self.machine.axes().len(), self.machine.contact_axes());
        // The probe also clears the force of the last step.
        let tool_route = self.crash && self.machine.set_tool_force(&vec![0.0; contact]);
        self.tool_push = false;
        let near: Vec<usize> =
            (0..self.obstacles.len()).filter(|&k| self.gaps.get(k).is_some_and(|g| *g < PUSH_RANGE)).collect();
        if !self.crash || near.is_empty() {
            return;
        }
        let machine = self.machine.as_ref();
        let motors = machine.motors().len();
        let dim = if tool_route { contact } else { motors };
        let (mut angle, mut speed) = (vec![0.0; dim.max(motors)], vec![0.0; dim.max(motors)]);
        if tool_route {
            machine.contact_state(&self.state, &mut angle[..contact], &mut speed[..contact]);
        } else {
            machine.motor_state(&self.state, &mut angle, &mut speed);
        }
        angle.truncate(dim);
        let (mut tool, mut state, mut torque) = (vec![0.0; axes], Vec::new(), vec![0.0; motors]);
        let mut gaps_at = |at: &[f64]| -> Vec<f64> {
            if tool_route {
                machine.contact_placed(&self.state, at, &mut state);
            } else {
                machine.forward(at, &mut tool);
                machine.equilibrium(&tool, &mut state, &mut torque);
            }
            let bodies = crate::collision::Bodies::of(machine, &state);
            near.iter().map(|&k| bodies.gap(&self.obstacles[k].aabb).map_or(f64::INFINITY, |g| g.2)).collect()
        };
        // At the tool the depth is that of the parts where they are (the clearance after the last step), not where the
        // rigid kinematics put them about the tool: a payload on a leaning axis stands back from its tool point by the
        // lean, and is held at its face where it touches.
        let depths: Vec<f64> = if tool_route {
            near.iter().map(|&k| -self.gaps[k]).collect()
        } else {
            gaps_at(&angle).into_iter().map(|g| -g).collect()
        };
        let mut slopes = vec![vec![0.0; dim]; near.len()];
        for m in 0..dim {
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
        self.tool_push = tool_route && !self.pushes.is_empty();
        if self.tool_push {
            self.tool_buffers = std::array::from_fn(|_| vec![0.0; contact]);
        }
    }

    /// The step done: the obstacles' force at its end (`contact_force`), and the work their pushes took out of the
    /// machine over it (J), integrated with the motion (the last entry of `joint`), which goes to `crash_energy`.
    fn pushed(&mut self) -> f64 {
        if self.pushes.is_empty() {
            self.contact_force = 0.0;
            return 0.0;
        }
        self.contact_force = if self.tool_push {
            let (force, _, total) = tool_push_forces(self.machine.as_ref(), &self.pushes, &self.push_from, &self.state);
            // What is sampled after the step (the losses of a gearbox or a guide the push loads) sees the push at its
            // end, not at the integration's last stage.
            self.machine.set_tool_force(&force);
            total
        } else {
            let motors = self.push_from.len();
            let (mut angle, mut speed, mut torque) = (vec![0.0; motors], vec![0.0; motors], vec![0.0; motors]);
            self.machine.motor_state(&self.state, &mut angle, &mut speed);
            push_torques(&self.pushes, &self.push_from, &angle, &speed, &mut torque)
        };
        let taken = self.joint.last().copied().unwrap_or(0.0);
        self.crash_energy += taken;
        taken
    }

    /// The torques the machine's mechanics take at the end of the last step: the drives' plus the obstacles' pushes
    /// (`push_torques`), as the step integrated them. What a machine works out of its torques (a guide's load from the
    /// acceleration they command, its friction and so its loss) is to be sampled with these, or the losses booked and
    /// those the motion had part ways while a part touches an obstacle.
    pub fn machine_torque(&self) -> Vec<f64> {
        let mut out = self.torque.clone();
        // (Pushes at the tool are the machine's own force, not torques at its motors.)
        if !self.pushes.is_empty() && !self.tool_push {
            let motors = out.len();
            let (mut angle, mut speed) = (vec![0.0; motors], vec![0.0; motors]);
            self.machine.motor_state(&self.state, &mut angle, &mut speed);
            push_torques(&self.pushes, &self.push_from, &angle, &speed, &mut out);
        }
        out
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
            tool_push,
            cuts,
            tool_buffers,
            motor_buffers: [angle, speed, stage, push],
            dt,
            substeps,
            ..
        } = self;
        let (on_tool, motor_pushes): (bool, &[Push]) = if *tool_push { (true, &[]) } else { (false, pushes) };
        let n = state.len();
        joint.clear();
        joint.extend_from_slice(state);
        joint.resize(n + WINDING_STATES * torque.len() + 1, 0.0);
        let finer = if *tool_push { PUSH_SUBSTEPS } else { 1 };
        let substeps = *substeps * finer.max(*cuts);
        let h = *dt / substeps as f64;
        let [pos, vel, force] = tool_buffers;
        for _ in 0..substeps {
            rk_joint.step(
                |y, dy| {
                    let (mech, (dmech, rest)) = (&y[..n], dy.split_at_mut(n));
                    machine.motor_state(mech, angle, speed);
                    push.fill(0.0);
                    push_torques(motor_pushes, push_from, angle, speed, push);
                    for k in 0..stage.len() {
                        stage[k] = torque[k] + push[k];
                    }
                    rest.fill(0.0);
                    rest[rest.len() - 1] = -push.iter().zip(speed.iter()).map(|(t, w)| t * w).sum::<f64>();
                    if on_tool {
                        let buffers = [&mut *pos, &mut *vel, &mut *force];
                        let last = rest.len() - 1;
                        rest[last] += tool_push_stage(machine.as_mut(), pushes, push_from, mech, buffers);
                    }
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
            tool_push,
            cuts,
            tool_buffers: [pos, vel, force],
            ..
        } = self;
        let (on_tool, motor_pushes): (bool, &[Push]) = if *tool_push { (true, &[]) } else { (false, pushes) };
        let (n, drives) = (state.len(), &control.drives);
        joint.clear();
        joint.extend_from_slice(state);
        for (k, d) in drives.iter().enumerate() {
            joint.extend(d.winding_states(control.drive_params_of(k)));
        }
        // The work the obstacles' pushes take out of the machine over the step (`pushed`).
        joint.push(0.0);
        let finer = if *tool_push { PUSH_SUBSTEPS } else { 1 };
        let substeps = *substeps * finer.max(*cuts);
        let h = *dt / substeps as f64;
        for _ in 0..substeps {
            rk_joint.step(
                |y, dy| {
                    let ((mech, windings), (dmech, dwindings)) = (y.split_at(n), dy.split_at_mut(n));
                    machine.motor_state(mech, angle, speed);
                    for (k, d) in drives.iter().enumerate() {
                        // A drive whose pulses are off after a trip: no current, no torque (`Controller::react`).
                        if d.pulses_off {
                            // Only the iron's drag acts (`DriveParams::iron_drag`), and a stepper's magnets hold their
                            // detent without current, at the stage's angle; their work is integrated in the winding's
                            // work slot, which `Controller::winding_stepped` books as the shaft's and what the magnets
                            // and the iron gave, as with the drives disabled.
                            let p = control.drive_params_of(k);
                            stage_torque[k] = -p.iron_drag(speed[k]);
                            if let crate::drive::Motor::Stepper(s) = &p.motor {
                                stage_torque[k] += s.detent_torque(angle[k], d.phases.origin);
                            }
                            dwindings[WINDING_STATES * k..WINDING_STATES * (k + 1)].fill(0.0);
                            dwindings[WINDING_STATES * k + 3] = stage_torque[k] * speed[k];
                            continue;
                        }
                        let range = WINDING_STATES * k..WINDING_STATES * (k + 1);
                        stage_torque[k] = control.drive_params_of(k).motor_rates(
                            d,
                            &windings[range.clone()],
                            angle[k],
                            speed[k],
                            &mut dwindings[range],
                        );
                    }
                    push.fill(0.0);
                    push_torques(motor_pushes, push_from, angle, speed, push);
                    for (stage, push) in stage_torque.iter_mut().zip(push.iter()) {
                        *stage += push;
                    }
                    let work = dwindings.len() - 1;
                    dwindings[work] = -push.iter().zip(speed.iter()).map(|(t, w)| t * w).sum::<f64>();
                    if on_tool {
                        let buffers = [&mut *pos, &mut *vel, &mut *force];
                        dwindings[work] += tool_push_stage(machine.as_mut(), pushes, push_from, mech, buffers);
                    }
                    machine.derivatives(mech, stage_torque, dmech);
                },
                joint,
                h,
            );
        }
        state.copy_from_slice(&joint[..n]);
        // The rotors where the step leaves them, for a stepper's torque at its end.
        machine.motor_state(state, angle, speed);
        control.winding_stepped(&joint[n..], angle, speed);
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
            // How long the axes' setpoints have been held: the plan now is `position_setpoint` run on by it.
            SignalInfo::new("setpoint_age", "group-sim", "quantity-setpoint-age", "s"),
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
        for (k, motor) in self.machine.motors().iter().enumerate() {
            // A linear motor's angle is a position (m), its speed m/s, its torque a force (N).
            let (pos, rate, load) =
                if self.machine.motor_linear(k) { ("m", "m/s", "N") } else { ("rad", "rad/s", "N m") };
            for (name, unit) in [
                ("angle", pos),
                ("speed", rate),
                ("torque", load),
                ("torque-command", load),
                ("current", "A"),
                ("voltage", "V"),
                ("electrical-power", "W"),
                ("winding-temperature", "K"),
                ("dc-link-voltage", "V"),
                ("braking-resistor-power", "W"),
                ("speed-command", rate),
                ("following-error", pos),
                ("angle-setpoint", pos),
                ("torque-feedforward", load),
                ("torque-feedback", load),
                ("speed-from-position", rate),
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
        self.machine.effect_signals(&self.state, &self.machine_torque(), out);
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
        out.extend([self.time(), if self.payload_attached { 1.0 } else { 0.0 }, c.setpoint_age(self.time())]);
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
                d.phase_voltage(c.drive_params_of(k)),
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
        self.machine.base_signals(&self.state, &self.machine_torque(), out);
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
