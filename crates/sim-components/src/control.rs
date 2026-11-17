//! Motion controller: plans in tool coordinates, transforms to motor setpoints, and runs one drive
//! per motor. It works on any machine through the machine interface.

use crate::conveyor::{Conveyor, Pass};
use crate::drive::{Drive, DriveMode, DriveParams, Motor, WINDING_STATES, quantise, shared_dc_link};
use crate::fault::{self, Fault, Monitor, Watch};
use crate::path::{self, Piece};
use crate::profile::{MotionLaw, Move, Shape};
use crate::shaper;
use sim_core::{CamMaster, Config, ConfigError, Machine};

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
    /// A path given as poses at times (seconds from its start, the first at 0 and at the present
    /// setpoint, the last at rest). The controller runs a smooth curve through them.
    Trajectory(Vec<(f64, Vec<f64>)>),
    /// `Trajectory` with the speed and acceleration at every point given (`trace::replay` of a trace that carries the
    /// tool's setpoints): a quintic through them (Hermite interpolation of position, speed and acceleration), so that at
    /// the points the setpoints are exactly those given.
    RatedTrajectory(Vec<(f64, Vec<f64>)>, Vec<Vec<f64>>, Vec<Vec<f64>>),
    /// Switch the drives off: no torque, and a move or cycle under way is dropped. Machines with holding
    /// brakes engage them. Until `Enable`, every other command is refused.
    Disable,
    /// Switch the drives on again, holding where the motors stand.
    Enable,
    /// A stop as a servo drive makes it, by its stop category (`StopCategory`); a move or cycle under way is dropped.
    /// (`Stop` above ends the cycle after its segment.)
    Halt(StopCategory),
}

/// The stop categories of IEC 60204-1 as servo drives make them (SINAMICS: STO / OFF2, OFF3, and a stop that holds).
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum StopCategory {
    /// 0: the torque off at once, as `Command::Disable`; the holding brakes engage after `brake.engage_delay_s`.
    Zero,
    /// 1: every drive brakes on its OFF3 ramp (`drive.off3_ramp_s`) within its torque and current limits; once all stand,
    /// the brakes engage with the torque on, and after their engagement delay the torque goes off. When
    /// `drive.ss1_time_s` runs out first, the torque goes off then (as SS1's timer).
    One,
    /// 2: the plan brakes along the direction it moves at the axes' acceleration limits and holds there, torque on.
    Two,
}

impl StopCategory {
    /// 0, 1 or 2.
    pub fn from_number(n: u8) -> Option<Self> {
        [Self::Zero, Self::One, Self::Two].get(n as usize).copied()
    }
}

/// A category 1 stop under way (`StopCategory::One`): the time left on the SS1 timer, s, and how long all drives have
/// stood with their brakes engaging, s (`None` while one still moves).
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
struct Halt {
    left: f64,
    closing: Option<f64>,
}

/// A command the controller refuses; `key` is a translation key.
#[derive(Debug, Clone, PartialEq)]
pub struct CommandError {
    pub key: &'static str,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
struct Segment {
    target: Vec<f64>,
    dwell_ticks: u64,
    /// Whether the payload is attached once this segment has arrived.
    payload: bool,
    /// With effects.gripper_time: how long the gripper takes to come to `payload` (`Gripper`), ticks; 0 without.
    #[serde(default)]
    grip_ticks: u64,
    /// With `via`: the path to the target, planned where the segment starts (see `via_pieces`): by `path::plan`, or
    /// with `profile.cycle_timing = "time_optimal"` by `path::plan_time_optimal`.
    path: Option<Vec<Piece>>,
    /// With `corners_m`: its via points joined by lines (`lines_through`) whose corners are rounded within this, in
    /// place of `profile.corner_tolerance_m`.
    #[serde(default)]
    corner: Option<f64>,
    /// The motion law of its move: `profile.shape`, or the segment's own `shape`.
    law: MotionLaw,
    /// `valves`: the commands it gives valves (`Machine::valves`, by index) once it has arrived, as the gripper acts.
    #[serde(default)]
    valves: Vec<(usize, f64)>,
    /// `wait`: the end-position sensors ([valve, 0 retracted or 1 extended]) its dwell waits for before it counts.
    #[serde(default)]
    wait: Vec<(usize, usize)>,
}

/// One mode an input shaper cancels: its impulses (`shaper::impulses`, times in half damped periods), half its damped
/// period in profile cycles as configured (0: none, the shaper of a single impulse), the mode of the machine it
/// follows with `effects.shaper_follows_pose` (0 the lowest resonance, `Machine::resonance_hz`; k the k-th of
/// `Machine::modes`) with half the damped period in profile cycles times the frequency, and the axes it acts on.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
struct ShaperMode {
    impulses: Vec<(f64, f64)>,
    half: f64,
    follows: Option<(usize, f64)>,
    axes: Vec<bool>,
}

/// The fastest the delays of a shaper change while its period goes over to the one of a changed machine (a payload
/// gripped or let go): a tenth of the time passing. Each copy of the plan then runs at 0.9 to 1.1 times the plan's pace.
const SHAPER_RAMP_RATE: f64 = 0.1;

/// The motion that made the planned setpoints from the profile tick `first` on, its start ticks as it took them (on a
/// conveyor with its pass and the simulated time at `first`, on a cam with the machine's cam curve then): the input
/// shaper reads the plan off it between two cycles. `key` tells it from the one before without comparing their curves
/// (`plan_key`).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
struct Plan {
    first: u64,
    key: [u64; 4],
    motion: Motion,
    pass: Option<(Pass, f64)>,
    #[serde(skip)]
    cam: KeptCurve,
    /// Noted after a changed config: the profile tick, between two cycles, from which it holds, the moment of the
    /// change. Before it the plan noted before holds, also in the cycle that ends with `first`.
    #[serde(default)]
    from: Option<f64>,
}

/// A cam's curve kept with a `Plan` (`Machine::cam_curve`), shared with every copy of the controller. A closure is not
/// saved with the controller's state: a plan read back from it reads the machine's cam as it is then.
#[derive(Clone, Default)]
struct KeptCurve(Option<sim_core::CamCurve>);

impl std::fmt::Debug for KeptCurve {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(if self.0.is_some() { "KeptCurve(Some)" } else { "KeptCurve(None)" })
    }
}

/// The path of a cycle segment through its via points (`via` in `[[cycle.segment]]`): a line to the first, a smooth
/// curve (`Piece::Spline`) through the others, and a line from the last to the target. So the path leaves its start and
/// arrives at its target on straight lines, and rounds the corners in between: with via points straight above the start
/// and the target, a lift, traverse and lower without a stop. One via point alone is two lines, with a stop between.
pub fn via_pieces(via: &[Vec<f64>], target: &[f64]) -> Vec<Piece> {
    let mut pieces: Vec<Piece> = via
        .iter()
        .enumerate()
        .map(|(i, point)| if i == 0 { Piece::Line(point.clone()) } else { Piece::Spline(point.clone()) })
        .collect();
    pieces.push(Piece::Line(target.to_vec()));
    pieces
}

/// The path of a cycle segment with `corners_m`: lines through its via points to the target, whose corners the planner
/// rounds within that tolerance (`path::round_corners`): a lift, traverse and lower through the two corners at the
/// lift's height, passed without a stop.
pub fn lines_through(via: &[Vec<f64>], target: &[f64]) -> Vec<Piece> {
    via.iter().chain([&target.to_vec()]).map(|point| Piece::Line(point.clone())).collect()
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
enum Motion {
    Idle,
    /// `tail` is the previous move still finishing while this one has already started (corner
    /// blending): its remaining distance is added on top.
    Moving {
        mv: Move,
        start_tick: u64,
        tail: Option<(Move, u64)>,
    },
    Dwell {
        until_tick: u64,
    },
    /// Linear interpolation from `from` to `to` between two ticks.
    Following {
        from: Vec<f64>,
        to: Vec<f64>,
        start_tick: u64,
    },
    /// A pass of the conveyor cycle (`Controller::pass`), waiting for its start and then run.
    Conveyor,
    /// The electronic cam of `Machine::cam` (`crate::cam`): the master started from `from` at `start_tick`, its speed
    /// ramped from `from_speed` to `speed` over `ramp` seconds (`cam::master_change`), and runs down from `stop` seconds
    /// after that; the slaves are the cam at the master plus `offset`. The next pass begins where the master reaches
    /// `next_pass`. A config with another master speed or ramp starts a new ramp from where the master is.
    Cam {
        start_tick: u64,
        stop: Option<f64>,
        from: f64,
        offset: Vec<f64>,
        next_pass: f64,
        from_speed: f64,
        speed: f64,
        ramp: f64,
    },
    /// A cubic curve through the points with the given speeds at them (Hermite interpolation; with the speeds of
    /// `c2_speeds`, the cubic spline whose acceleration is continuous).
    Trajectory {
        points: Vec<(f64, Vec<f64>)>,
        speeds: Vec<Vec<f64>>,
        /// Given (`Command::RatedTrajectory`): the accelerations at the points, and a quintic in place of the cubic.
        accelerations: Option<Vec<Vec<f64>>>,
        start_tick: u64,
    },
    /// A category 2 stop (`StopCategory::Two`): from `from` at the speeds `vel`, every axis decelerating evenly to rest in
    /// `time` seconds, so that the tool keeps the direction it moved in.
    Halting {
        from: Vec<f64>,
        vel: Vec<f64>,
        time: f64,
        start_tick: u64,
    },
}

#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Controller {
    params: DriveParams,
    /// Each motor's drive: `params` with that motor's own values (`catalog::motor_config`).
    motor_params: Vec<DriveParams>,
    /// Loop cycles in base steps.
    n_velocity: u64,
    n_position: u64,
    n_profile: u64,
    profile_cycle: f64,
    setpoint_ticks: u64,
    /// How long before the end of a segment the next one may start; 0 with blending off.
    blend_time: f64,
    /// For a path timed as fast as the motors allow: the share of the torque limits it may use
    /// (`profile.torque_reserve`), the belt tensions it may need (`ratings.belt_min_tension_n`, `belt_max_tension_n`, or the machine's
    /// own `tension_band`, less `profile.tension_margin_n` on both; unbounded where the config has none) and the share of the room to them
    /// (`profile.tension_reserve`), each gearbox's output torque rating and the guides' load rating (`path::PlanLimits`).
    plan_limits: path::PlanLimits,
    /// Each motor's DC link room above the bus, J (`dc_link_room`).
    #[serde(default)]
    dc_link_rooms: Vec<Option<f64>>,
    /// Per motor where the machine's signals hold the friction its own model takes of that motor's torque at the
    /// machine's state (`motor.<id>.torque_friction`, `TORQUE_PARTS`): what the torque-band monitor expects of friction.
    #[serde(default)]
    friction_signals: Vec<Option<usize>>,
    /// How far a planned path may leave its lines and arcs for transition curves (`path`) and to round a corner between
    /// two lines (`corner`).
    path_tolerance: path::Tolerance,
    /// `profile.cycle_timing = "time_optimal"`: every cycle segment is timed as fast as the motors allow
    /// (`path::plan_time_optimal`), rest to rest, not by the axis limits.
    cycle_time_optimal: bool,
    segments: Vec<Segment>,
    motion: Motion,
    /// Index of the cycle segment under way, while the cycle runs.
    cycle: Option<usize>,
    /// How often the cycle has started its first segment.
    passes: u64,
    stopping: bool,
    enabled: bool,
    pub payload_attached: bool,
    /// The command of each valve of `Machine::valves` (+1 extend, -1 retract, 0 centre), set by the cycle's segments;
    /// the simulation hands them to the machine every step.
    #[serde(default)]
    pub valves: Vec<f64>,
    /// effects.gripper_time: the tick at which the gripper under way is done, and whether it then holds the payload.
    #[serde(default)]
    gripping: Option<(u64, bool)>,
    /// The ticks of the dwell under way (`dwell_progress`): its segment's `dwell_s`, or the gripper's time if longer.
    #[serde(default)]
    dwell_ticks: u64,
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
    /// The setpoints as planned. With input shaping, `set_pos` and its rates are these spread over time;
    /// without, the same.
    raw_pos: Vec<f64>,
    raw_vel: Vec<f64>,
    raw_acc: Vec<f64>,
    /// Input shaper: the modes it cancels, the first and the second (`profile.shaper_frequency_hz`,
    /// `shaper2_frequency_hz`; half period 0 without one) and those of `profile.shaper_modes`; on each axis the shapers
    /// of the modes acting on it are convolved. `None` with input shaping off.
    shaper: Option<Vec<ShaperMode>>,
    /// A shaper setting from a changed config, waiting for the plan to stand still: taken under a moving plan it would
    /// make the setpoint jump.
    pending_shaper: Option<Option<Vec<ShaperMode>>>,
    /// How many cycles of `history` the shaper has needed so far.
    shaper_reach: usize,
    /// Where the machine itself changed its modes (a payload gripped or let go), the shaper's half periods go over to
    /// the new ones on a ramp: per change the half periods' step (old less new, cycles, one per mode), the tick it was
    /// made at and the ramp's length in cycles (`shape`).
    #[serde(default)]
    shaper_ramps: Vec<(Vec<f64>, f64, f64)>,
    /// The half periods the machine reported for the newest stored setpoint, to tell a change of the machine from
    /// one of the pose.
    #[serde(default)]
    shaper_targets: Option<Vec<f64>>,
    /// The motions that made the stored setpoints (`history`), oldest first.
    #[serde(default)]
    plans: std::collections::VecDeque<Plan>,
    /// A config changed between two profile cycles: the moment, in profile ticks, from which the plan's next motion is
    /// noted anew (`Plan::from`).
    #[serde(default)]
    plan_changed_at: Option<f64>,
    /// The base step `update` ran last.
    #[serde(default)]
    base_step: u64,
    /// The motion law of moves (`profile.shape`, `deceleration_ratio`, `vertical_axis`, `up_acceleration_ratio`).
    law: MotionLaw,
    /// A trajectory's curve: the C2 cubic spline (`profile.trajectory_spline = "c2"`), or the Hermite cubic with the
    /// slopes of the neighbours ("hermite", as before 14 November 2026).
    spline_c2: bool,
    /// Fieldbus: profile cycles between computing the motor setpoints and the drives using them, and the
    /// setpoints on their way, oldest first, each with the cycles it takes: per motor position, speed,
    /// acceleration, torque feedforward and friction feedforward from the machine's model (`Drive::friction_ff`).
    fieldbus_delay: usize,
    fieldbus: std::collections::VecDeque<(usize, Vec<[f64; 5]>)>,
    /// Frames pushed so far, and the number of the one the drives hold: a late frame leaves them the old one, whose
    /// setpoint the drive carries on from (`since_setpoint` runs on).
    fieldbus_pushed: u64,
    fieldbus_held: u64,
    /// Fieldbus jitter: a frame takes up to this many cycles more than the delay, drawn evenly from 0 to it per frame
    /// from its own pseudo-random stream; 0 with the jitter off.
    fieldbus_jitter: usize,
    jitter: sim_core::testkit::Noise,
    /// The profile cycle under way, counted here: the base step counter would count in another unit once
    /// `profile.cycle_s` changes.
    tick: u64,
    /// Planned setpoints of the last cycles, newest last, as far back as the shaper reaches.
    history: std::collections::VecDeque<[Vec<f64>; 3]>,
    /// With `[conveyor]`: the cycle picks from the belt instead of running `segments`, one pass per part, and the pass
    /// under way or last run.
    conveyor: Option<Conveyor>,
    pass: Option<Pass>,
    /// The simulated time at the profile cycle under way, s.
    clock: f64,
    /// Each motor's moment of inertia in the rigid model at the start pose, for the constant-inertia torque
    /// feedforward (`rigid_inertias`); empty while no drive uses it.
    inertias: Vec<f64>,
    /// effects.load_encoder: the step of the load-side scales (`drive.load_encoder_step`, in the tool axes' units, 0
    /// ideal) on which the position loops close; `None` off, on the motor encoders.
    load_encoder: Option<f64>,
    /// `Machine::cam`: the cycle is the machine's electronic cam, and where its master stands (kept between runs).
    cam: Option<sim_core::CamMaster>,
    cam_master: f64,
    /// effects.drive_monitoring: each drive's monitors (`fault::Watch`) trip all drives, which then stop by their
    /// fault reactions. Per drive its settings and its monitors' running state.
    monitoring: bool,
    fault_settings: Vec<fault::Settings>,
    watch: Vec<Watch>,
    /// While the drives stop after a trip: per drive the speed setpoint of its stop ramp, `None` once its pulses are
    /// off. The controller counts as enabled until the last is off, and refuses commands but `Enable` and `Disable`.
    reacting: Option<Vec<Option<f64>>>,
    /// The first trip since the drives were last enabled (effects.drive_monitoring).
    pub fault: Option<Fault>,
    /// A dual-drive gantry's synchronisation (`gantry.sync`, `Machine::gantry_motors`); `None` for the plain scheme,
    /// in which each drive follows its own setpoint.
    gantry_sync: Option<GantrySync>,
    /// effects.brake_delay: the holding brakes' engagement and release delays, s (`brake.engage_delay_s`,
    /// `release_delay_s`); both 0 with the effect off. Whether the brakes are engaged now, and how long they have been
    /// asked to change (`step_brake`).
    #[serde(default)]
    brake_delays: [f64; 2],
    #[serde(default)]
    brake_on: bool,
    #[serde(default)]
    brake_timer: f64,
    /// The SS1 timer of a category 1 stop, s (`drive.ss1_time_s`; infinite without the key), and such a stop under way.
    #[serde(default)]
    ss1_time: f64,
    #[serde(default)]
    halt: Option<Halt>,
}

/// How the two drives of a dual-drive gantry are kept in step, beyond each following the common setpoint.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GantrySync {
    pub master: usize,
    pub slave: usize,
    /// The slave motor's angle per unit of the master's (both drive the same axis).
    pub ratio: f64,
    /// `None`: master/slave, the slave's position loop follows the master's actual position (its setpoint less the
    /// master's following error). `Some(k)`: cross-coupled, the synchronisation error (the slave's actual position
    /// less the master's, in the slave's units) is fed back to both speed commands with the gain k, 1/s: the slave's
    /// takes k e, the master's gets k e / ratio.
    pub cross_gain: Option<f64>,
}

/// The gantry's synchronisation from `gantry.sync` ("plain", "master_slave" or "cross_coupled") and, cross-coupled,
/// `gantry.sync_gain_1_s`; read only where the machine has a dual-drive gantry. The ratio of the two motors is that of
/// their setpoints at the start pose.
fn gantry_sync(cfg: &Config, machine: &dyn Machine) -> Result<Option<GantrySync>> {
    let Some([master, slave]) = machine.gantry_motors() else { return Ok(None) };
    let cross_gain = match text(cfg, "gantry.sync")? {
        "plain" => return Ok(None),
        "master_slave" => None,
        "cross_coupled" => Some(cfg.non_negative("gantry.sync_gain_1_s")?),
        _ => return Err(ConfigError::Invalid("gantry.sync".into())),
    };
    let pose = cfg.f64s("start.pose")?;
    let (mut a, mut b) = (vec![0.0; machine.motors().len()], vec![0.0; machine.motors().len()]);
    let h = 1e-4;
    machine.inverse(&pose.iter().map(|p| p + h).collect::<Vec<_>>(), &mut a);
    machine.inverse(&pose.iter().map(|p| p - h).collect::<Vec<_>>(), &mut b);
    let ratio = (a[slave] - b[slave]) / (a[master] - b[master]);
    if !ratio.is_finite() || ratio == 0.0 {
        return Err(ConfigError::Invalid("gantry.sync".into()));
    }
    Ok(Some(GantrySync { master, slave, ratio, cross_gain }))
}

/// Each motor's moment of inertia in the rigid, loss-free model at a tool pose: the torque it needs for a unit
/// acceleration of its own while the others stand, less what it needs at rest (gravity). What a drive's constant-inertia
/// precontrol is set to (S120 p1498, the load's inertia, plus the motor's).
pub fn rigid_inertias(machine: &dyn Machine, pose: &[f64]) -> Vec<f64> {
    let motors = machine.motors().len();
    let (mut angle, zero) = (vec![0.0; motors], vec![0.0; motors]);
    let (mut rest, mut moved) = (vec![0.0; motors], vec![0.0; motors]);
    machine.inverse(pose, &mut angle);
    machine.rigid_motor_torque(&angle, &zero, &zero, &mut rest);
    (0..motors)
        .map(|k| {
            let unit: Vec<f64> = (0..motors).map(|j| if j == k { 1.0 } else { 0.0 }).collect();
            machine.rigid_motor_torque(&angle, &zero, &unit, &mut moved);
            moved[k] - rest[k]
        })
        .collect()
}

fn steps(cfg: &Config, path: &str, dt: f64) -> Result<u64> {
    let n = (cfg.positive(path)? / dt).round();
    // A loop cycle must be a whole number of base steps.
    if n >= 1.0 && (n * dt - cfg.f64(path)?).abs() < 1e-6 * dt {
        Ok(n as u64)
    } else {
        Err(ConfigError::Invalid(path.into()))
    }
}

impl Controller {
    /// A controller at rest at the machine's current state.
    pub fn new(cfg: &Config, machine: &dyn Machine, state: &[f64]) -> Result<Self> {
        let (axes, motors) = (machine.axes().len(), machine.motors().len());
        let mut c = Self {
            params: DriveParams::from_config(cfg, 1.0, 1.0)?,
            motor_params: Vec::new(),
            n_velocity: 1,
            n_position: 1,
            n_profile: 1,
            profile_cycle: 1.0,
            setpoint_ticks: 1,
            blend_time: 0.0,
            plan_limits: path::PlanLimits::new(1.0, [f64::NEG_INFINITY, f64::INFINITY, 1.0]),
            dc_link_rooms: Vec::new(),
            friction_signals: Vec::new(),
            path_tolerance: path::Tolerance::default(),
            cycle_time_optimal: false,
            segments: Vec::new(),
            motion: Motion::Idle,
            cycle: None,
            passes: 0,
            stopping: false,
            enabled: true,
            payload_attached: true,
            valves: machine.valves().into_iter().map(|(_, command)| command).collect(),
            gripping: None,
            dwell_ticks: 0,
            drives: Vec::new(),
            set_pos: vec![0.0; axes],
            set_vel: vec![0.0; axes],
            set_acc: vec![0.0; axes],
            tool_error: vec![0.0; axes],
            setpoint_offset: vec![0.0; axes],
            motor_pos: vec![0.0; motors],
            motor_vel: vec![0.0; motors],
            raw_pos: vec![0.0; axes],
            raw_vel: vec![0.0; axes],
            raw_acc: vec![0.0; axes],
            shaper: None,
            pending_shaper: None,
            shaper_reach: 0,
            shaper_ramps: Vec::new(),
            shaper_targets: None,
            plans: Default::default(),
            plan_changed_at: None,
            base_step: 0,
            law: MotionLaw::default(),
            spline_c2: true,
            fieldbus_delay: 0,
            fieldbus: Default::default(),
            fieldbus_pushed: 0,
            fieldbus_held: u64::MAX,
            fieldbus_jitter: 0,
            jitter: Default::default(),
            tick: 0,
            conveyor: None,
            pass: None,
            clock: 0.0,
            history: Default::default(),
            inertias: Vec::new(),
            load_encoder: None,
            cam: None,
            cam_master: 0.0,
            monitoring: false,
            fault_settings: Vec::new(),
            watch: Vec::new(),
            reacting: None,
            fault: None,
            gantry_sync: None,
            brake_delays: [0.0; 2],
            brake_on: false,
            brake_timer: 0.0,
            ss1_time: f64::INFINITY,
            halt: None,
        };
        c.apply_config(cfg, machine)?;
        // Nothing moves yet: the shaper is in use from the first cycle on.
        c.shaper = c.pending_shaper.take().flatten();
        machine.motor_state(state, &mut c.motor_pos, &mut c.motor_vel);
        let (mut tool, mut speed) = (vec![0.0; axes], vec![0.0; axes]);
        machine.tool_state(state, &mut tool, &mut speed);
        c.hold_here(machine, Some(&tool));
        Ok(c)
    }

    /// Setpoints and drives at rest where the motors stand; holding the load at the tool pose `at` where it is known
    /// (the start), else at the setpoint (`Command::Enable`).
    fn hold_here(&mut self, machine: &dyn Machine, at: Option<&[f64]>) {
        let motors = self.motor_pos.len();
        // The setpoint starts where the motors are. With a stretched belt and a loaded gearbox that
        // is a few micrometres from the tool pose, and starting there avoids a jolt at the first step.
        machine.forward(&self.motor_pos, &mut self.set_pos);
        self.raw_pos.clone_from(&self.set_pos);
        for rate in [&mut self.set_vel, &mut self.set_acc, &mut self.raw_vel, &mut self.raw_acc] {
            rate.fill(0.0);
        }
        self.history.clear();
        self.plans.clear();
        self.fieldbus.clear();
        self.watch = vec![Watch::default(); motors];
        // Start holding the load: the torque that keeps the pose, and the feedforward's share of it. At the tool's pose,
        // not the setpoint's: the setpoint is where the motors' angles put the tool without the gears' twist (4 mm from
        // it on the palletiser), and the hold there had its gravity, and its ripple and cogging at the rotor angle, of
        // another pose; a machine built at rest moved 1.2 um in its first 0.2 s while its drives settled.
        let (mut hold, mut ff) = (vec![0.0; motors], vec![0.0; motors]);
        machine.equilibrium(at.unwrap_or(&self.set_pos), &mut Vec::new(), &mut hold);
        self.torque_feedforward(machine, &self.set_pos, &self.set_vel, &self.set_acc, &vec![0.0; motors], &mut ff);
        // A winding that was warm stays warm, and a DC link keeps its charge. The current that holds the load comes
        // out of that charge (L i^2 / 2); below the bus level the rectifier makes it up at the next step. Without the DC
        // link simulated the drive draws it from the supply in its first step (`Drive::enable_energy`).
        // So does a housing (effects.motor_thermal_nodes), a drive's thermal image (effects.drive_overload) and a braking
        // resistor's warmth (effects.braking_resistor_heating).
        let kept: Vec<_> = self
            .drives
            .iter()
            .map(|d| {
                let heat = (d.temperature_rise, d.housing_rise, d.overload_image, d.overload_chip, d.overloaded);
                (heat, d.resistor_rise, d.dc_voltage)
            })
            .collect();
        // A stepper's electrical grid stays where it was across an enable (`Stepper::re_enabled`).
        let origins: Vec<f64> = self.drives.iter().map(|d| d.phases.origin).collect();
        self.drives =
            (0..motors).map(|k| Drive::at_rest(&self.motor_params[k], self.motor_pos[k], hold[k], ff[k])).collect();
        for (k, origin) in origins.into_iter().enumerate().take(motors) {
            if let Motor::Stepper(s) = &self.motor_params[k].motor {
                s.re_enabled(&mut self.drives[k], &self.motor_params[k], self.motor_pos[k], origin);
            }
        }
        let held = self.holding_torque(machine, at.unwrap_or(&self.set_pos));
        for (k, (drive, (heat, resistor_rise, voltage))) in self.drives.iter_mut().zip(kept).enumerate() {
            let p = &self.motor_params[k];
            drive.torque_hold = held[k];
            (drive.temperature_rise, drive.housing_rise, drive.overload_image, drive.overload_chip, drive.overloaded) =
                heat;
            drive.resistor_rise = resistor_rise;
            drive.hold_current(p);
            let stored = 0.5 * p.inductance * drive.current * drive.current;
            if p.dc_link {
                drive.dc_voltage = (voltage * voltage - 2.0 * stored / p.dc_capacitance).max(0.0).sqrt();
            } else {
                drive.enable_energy = stored;
            }
        }
        // Every drive has noise of its own; the multiplier spreads neighbouring seeds over the generator's cycle.
        for (k, drive) in self.drives.iter_mut().enumerate() {
            drive.noise.0 = self.params.noise_seed.wrapping_add(k as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15);
        }
        // The fieldbus jitter's stream, beside the drives' (no machine has a million motors).
        self.jitter.0 = self.params.noise_seed.wrapping_add(1 << 20).wrapping_mul(0x9E37_79B9_7F4A_7C15);
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
        let mut motor_params = machine
            .motors()
            .iter()
            .map(|m| DriveParams::from_config(&crate::catalog::motor_config(cfg, m.id)?, dt, n_velocity as f64 * dt))
            .collect::<Result<Vec<_>>>()?;
        // The DC links are the machine's (`catalog::motor_config`): a drive named per motor (an S210 switches the shared
        // link off, an S120 on) leaves them as the shared config has them, else its own link is stepped as well.
        for p in &mut motor_params {
            p.shared_dc_link = params.shared_dc_link;
        }
        let fault_settings = machine
            .motors()
            .iter()
            .zip(&motor_params)
            .map(|(m, p)| fault::Settings::from_config(&crate::catalog::motor_config(cfg, m.id)?, p))
            .collect::<Result<Vec<_>>>()?;
        // A config written before 14 November 2026 has no drive monitoring.
        let monitoring = cfg.get("effects.drive_monitoring").is_ok() && cfg.bool("effects.drive_monitoring")?;
        // A config written before 15 November 2026 has neither brake delays nor an SS1 timer.
        let brake_delays = if cfg.get("effects.brake_delay").is_ok() && cfg.bool("effects.brake_delay")? {
            [cfg.non_negative("brake.engage_delay_s")?, cfg.non_negative("brake.release_delay_s")?]
        } else {
            [0.0; 2]
        };
        let ss1_time =
            if cfg.get("drive.ss1_time_s").is_ok() { cfg.positive("drive.ss1_time_s")? } else { f64::INFINITY };
        let inertias = if motor_params.iter().any(|p| p.precontrol.constant_inertia) {
            rigid_inertias(machine, &cfg.f64s("start.pose")?)
        } else {
            Vec::new()
        };
        let load_encoder = if cfg.get("effects.load_encoder").is_ok() && cfg.bool("effects.load_encoder")? {
            Some(cfg.non_negative("drive.load_encoder_step")?)
        } else {
            None
        };
        let shaper = shaper_modes(cfg, machine, profile_cycle)?;
        let law = motion_law(cfg, machine)?;
        let sync = gantry_sync(cfg, machine)?;
        let spline_c2 = match text(cfg, "profile.trajectory_spline")? {
            "c2" => true,
            "hermite" => false,
            _ => return Err(ConfigError::Invalid("profile.trajectory_spline".into())),
        };
        let fieldbus_delay = cfg.f64("profile.fieldbus_delay_cycles")?;
        if fieldbus_delay < 0.0 || fieldbus_delay.fract() != 0.0 || fieldbus_delay > 1000.0 {
            return Err(ConfigError::Invalid("profile.fieldbus_delay_cycles".into()));
        }
        let fieldbus_delay = if cfg.bool("effects.fieldbus_delay")? { fieldbus_delay as usize } else { 0 };
        let fieldbus_jitter = cfg.f64("profile.fieldbus_jitter_cycles")?;
        if fieldbus_jitter < 0.0 || fieldbus_jitter.fract() != 0.0 || fieldbus_jitter > 1000.0 {
            return Err(ConfigError::Invalid("profile.fieldbus_jitter_cycles".into()));
        }
        let fieldbus_jitter = if cfg.bool("effects.fieldbus_jitter")? { fieldbus_jitter as usize } else { 0 };
        let blend_time = cfg.non_negative("profile.blend_time_s")?;
        let blend_time = if cfg.bool("effects.corner_blending")? { blend_time } else { 0.0 };
        let torque_reserve = cfg.positive("profile.torque_reserve")?;
        if torque_reserve > 1.0 {
            return Err(ConfigError::Invalid("profile.torque_reserve".into()));
        }
        let path_tolerance = path::Tolerance {
            path: cfg.non_negative("profile.path_tolerance_m")?,
            corner: cfg.non_negative("profile.corner_tolerance_m")?,
        };
        let cycle_time_optimal = match cfg.get("profile.cycle_timing")?.as_str() {
            Some("axis_limits") => false,
            Some("time_optimal") => true,
            _ => return Err(ConfigError::Invalid("profile.cycle_timing".into())),
        };
        let tension_reserve = cfg.positive("profile.tension_reserve")?;
        if tension_reserve > 1.0 {
            return Err(ConfigError::Invalid("profile.tension_reserve".into()));
        }
        let tension_margin = cfg.non_negative("profile.tension_margin_n")?;
        // The ratings the time-optimal plan keeps beside the motors': each gearbox's output torque where the machine
        // reports it, and the guides' load where it reports theirs (the verdicts judge the same signals); and what each
        // gearbox loses, which its motor gives on top.
        let reported = machine.signal_infos();
        let reports = |id: &str| reported.iter().any(|s| s.id == id);
        let gearbox = machine
            .motors()
            .iter()
            .map(|m| -> Result<Option<[f64; 4]>> {
                if !reports(&format!("gearbox.{}.output_torque", m.id)) {
                    return Ok(None);
                }
                let own = crate::catalog::motor_config(cfg, m.id)?;
                let rated = ["gearbox_max_torque_nm", "pulley_max_torque_nm"]
                    .iter()
                    .filter_map(|key| own.f64(&format!("ratings.{key}")).ok())
                    .fold(f64::INFINITY, f64::min);
                let inertia = own.f64("motor.rotor_inertia_kgm2").unwrap_or(0.0)
                    + own.f64("gearbox.inertia_at_input_kgm2").unwrap_or(0.0);
                let efficiency = match own.get("gearbox.efficiency") {
                    Ok(_) => crate::mech::Gearbox::configured_efficiency(&own)?,
                    Err(_) => 1.0,
                };
                Ok((rated.is_finite() || efficiency < 1.0).then_some([
                    own.f64("gearbox.ratio").unwrap_or(1.0),
                    inertia,
                    rated,
                    efficiency,
                ]))
            })
            .collect::<Result<Vec<_>>>()?;
        let guides = reported.iter().any(|s| s.id.ends_with(".guide_load"));
        let guide = if guides { cfg.f64("ratings.guide_max_load_n").unwrap_or(f64::INFINITY) } else { f64::INFINITY };
        let mut segments = Vec::new();
        let gripper = crate::gripper::Gripper::from_config(cfg)?;
        for (i, segment) in cfg.array("cycle.segment")?.iter().enumerate() {
            let bad = || ConfigError::Invalid(format!("cycle.segment[{i}]"));
            let number = |v: &toml::Value| v.as_float().or_else(|| v.as_integer().map(|i| i as f64));
            let target: Vec<f64> =
                segment.get("target").and_then(|t| t.as_array()).ok_or_else(bad)?.iter().filter_map(number).collect();
            if !in_range(machine, &target) {
                return Err(bad());
            }
            let corner = match segment.get("corners_m") {
                None => None,
                Some(c) => Some(
                    number(c)
                        .filter(|c| c.is_finite() && *c >= 0.0)
                        .ok_or_else(|| ConfigError::Invalid(format!("cycle.segment[{i}].corners_m")))?,
                ),
            };
            let via = match segment.get("via") {
                None => None,
                Some(points) => {
                    let bad = || ConfigError::Invalid(format!("cycle.segment[{i}].via"));
                    let points: Vec<Vec<f64>> = points
                        .as_array()
                        .ok_or_else(bad)?
                        .iter()
                        .map(|p| Some(p.as_array()?.iter().filter_map(number).collect()))
                        .collect::<Option<_>>()
                        .ok_or_else(bad)?;
                    if points.is_empty() || !points.iter().all(|p| in_range(machine, p)) {
                        return Err(bad());
                    }
                    Some(if corner.is_some() { lines_through(&points, &target) } else { via_pieces(&points, &target) })
                }
            };
            let shape = match segment.get("shape") {
                None => law.shape,
                Some(name) => name
                    .as_str()
                    .and_then(Shape::from_name)
                    .ok_or_else(|| ConfigError::Invalid(format!("cycle.segment[{i}].shape")))?,
            };
            // Valves by the id `Machine::valves` gives them: `valves = { z = "extend" }`, `wait = ["z.extended"]`.
            let ids: Vec<String> = machine.valves().into_iter().map(|(id, _)| id).collect();
            let valve = |id: &str| ids.iter().position(|v| v == id);
            let mut valves = Vec::new();
            if let Some(table) = segment.get("valves") {
                let bad = || ConfigError::Invalid(format!("cycle.segment[{i}].valves"));
                for (id, command) in table.as_table().ok_or_else(bad)? {
                    let command = match command.as_str() {
                        Some("extend") => 1.0,
                        Some("retract") => -1.0,
                        Some("centre") => 0.0,
                        _ => return Err(bad()),
                    };
                    valves.push((valve(id).ok_or_else(bad)?, command));
                }
            }
            let mut wait = Vec::new();
            if let Some(list) = segment.get("wait") {
                let bad = || ConfigError::Invalid(format!("cycle.segment[{i}].wait"));
                for sensor in list.as_array().ok_or_else(bad)? {
                    let (id, end) = sensor.as_str().and_then(|s| s.rsplit_once('.')).ok_or_else(bad)?;
                    let end = ["retracted", "extended"].iter().position(|e| *e == end).ok_or_else(bad)?;
                    wait.push((valve(id).ok_or_else(bad)?, end));
                }
            }
            let payload = segment.get("payload").and_then(|p| p.as_bool()).ok_or_else(bad)?;
            // Whole profile cycles, at least the gripper's time.
            let grip_ticks = gripper
                .map_or(0.0, |g| ((if payload { g.close_s } else { g.open_s }) / profile_cycle - 1e-9).ceil())
                as u64;
            segments.push(Segment {
                valves,
                wait,
                target,
                grip_ticks,
                dwell_ticks: (segment
                    .get("dwell_s")
                    .and_then(number)
                    .filter(|d| d.is_finite() && *d >= 0.0)
                    .ok_or_else(bad)?
                    / profile_cycle)
                    .round() as u64,
                payload,
                path: via,
                corner,
                law: MotionLaw { shape, ..law.clone() },
            });
        }
        // The margin comes off both ratings before the share is taken: it is for what the rigid model leaves out
        // whatever the room (friction, the belt's vibration), which is roughly absolute, not a share of the room.
        // A machine with a band of its own (cables between winches and a platform) gives it; the ratings otherwise.
        let band = machine.tension_band().map_or_else(
            || [cfg.f64("ratings.belt_min_tension_n").ok(), cfg.f64("ratings.belt_max_tension_n").ok()],
            |[low, high]| [Some(low), Some(high)],
        );
        let tension = [
            band[0].map_or(f64::NEG_INFINITY, |t| t + tension_margin),
            band[1].map_or(f64::INFINITY, |t| t - tension_margin),
            tension_reserve,
        ];
        // Each motor's shaft friction and gearbox drag as its own config has them (`mech::ShaftFriction`); none where the
        // config lacks their keys (an arm's joints, whose friction is the joint's).
        let friction = machine
            .motors()
            .iter()
            .map(|m| -> Result<[f64; 5]> {
                let own = crate::catalog::motor_config(cfg, m.id)?;
                Ok(crate::mech::ShaftFriction::from_config(&own)
                    .map_or([0.0; 5], |f| [f.coulomb, f.viscous, f.smoothing, f.drag.torque, f.drag.speed]))
            })
            .collect::<Result<Vec<_>>>()?;
        // 1 where a config (an older machine file) leaves it out.
        let margin = if cfg.get("profile.time_optimal_margin").is_ok() {
            cfg.positive("profile.time_optimal_margin")?
        } else {
            1.0
        };
        if margin > 1.0 {
            return Err(ConfigError::Invalid("profile.time_optimal_margin".into()));
        }
        // Each motor's drive current, gearbox input speed and braking power as the verdicts judge them
        // (`path::PlanLimits::ratings`); a line-up's one Braking Module shared out evenly over its motors.
        let line_up = cfg.get("drive.braking_module").is_ok() && cfg.bool("drive.braking_module")?;
        let ratings = machine
            .motors()
            .iter()
            .map(|m| -> Result<[f64; 3]> {
                let own = crate::catalog::motor_config(cfg, m.id)?;
                let rating = |key: &str| own.f64(&format!("ratings.{key}")).unwrap_or(f64::INFINITY);
                let dc_link = own.get("ratings.dc_link_capacitance_uf").is_ok();
                let braking = if dc_link { rating("braking_resistor_peak_w") } else { f64::INFINITY };
                let share = if line_up { machine.motors().len() as f64 } else { 1.0 };
                Ok([
                    rating("drive_max_current_a"),
                    rating("gearbox_max_input_speed_rpm") * std::f64::consts::PI / 30.0,
                    braking / share,
                ])
            })
            .collect::<Result<Vec<_>>>()?;
        // Each motor's DC link room above the bus, J, as the verdicts take it ("dc-link-energy"), for the tally's running
        // account (`analysis::LinkAccount`); None without the capacitance or the threshold.
        let dc_link_rooms = machine
            .motors()
            .iter()
            .map(|m| -> Result<Option<f64>> {
                let own = crate::catalog::motor_config(cfg, m.id)?;
                let rating = |key: &str| own.f64(&format!("ratings.{key}")).ok();
                let bus = own.f64("drive.bus_voltage_v")?;
                Ok(rating("dc_link_capacitance_uf")
                    .zip(rating("braking_threshold_v"))
                    .map(|(capacitance, threshold)| 0.5 * capacitance * 1e-6 * (threshold * threshold - bus * bus)))
            })
            .collect::<Result<Vec<_>>>()?;
        // The share of each speed rating the plan keeps back for the velocity loops (`path::speed_kept_of`): the most over
        // the motors, each at the inertia its precontrol is tuned for (the rigid model's at the start pose), the most
        // acceleration its torque reserve gives it and the top speed it is held to. Not less than the fixed share.
        let speed_kept = rigid_inertias(machine, &cfg.f64s("start.pose")?)
            .iter()
            .zip(&motor_params)
            .filter(|(inertia, _)| **inertia > 0.0)
            .map(|(&inertia, p)| {
                let acc = torque_reserve * p.peak_torque / inertia;
                path::speed_kept_of(p.ki, inertia, p.torque_ff, acc, p.max_speed)
            })
            .fold(path::PlanLimits::new(1.0, [0.0; 3]).speed_kept, f64::max);
        let plan_limits = path::PlanLimits {
            reserve: torque_reserve,
            tension,
            gearbox,
            guide,
            friction,
            ratings,
            margin,
            hermite: !spline_c2,
            speed_kept,
        };
        // A path through via points has to be one the planner times, from where the segment before ends; timed as fast as
        // the motors allow, so has the straight way of every other segment.
        for (i, segment) in segments.iter().enumerate() {
            let from = &segments[(i + segments.len() - 1) % segments.len()].target;
            let (straight, key) = (vec![Piece::Line(segment.target.clone())], format!("cycle.segment[{i}].via"));
            let (pieces, key) = match &segment.path {
                Some(pieces) => (pieces, key),
                None if cycle_time_optimal && !at_target(from, &segment.target) => {
                    (&straight, "profile.cycle_timing".to_string())
                }
                None => continue,
            };
            let tolerance =
                path::Tolerance { corner: segment.corner.unwrap_or(path_tolerance.corner), ..path_tolerance };
            let planned = if cycle_time_optimal {
                path::plan_time_optimal(
                    machine,
                    &motor_params.iter().collect::<Vec<_>>(),
                    &plan_limits,
                    from,
                    pieces,
                    profile_cycle,
                    tolerance,
                    path::ARC_SLACK,
                )
            } else {
                path::plan(machine.axes(), from, pieces, profile_cycle, tolerance, path::ARC_SLACK)
            };
            if planned.map_or(true, |points| !points.iter().all(|(_, pose)| near_start(machine, from, pose))) {
                return Err(ConfigError::Invalid(key));
            }
        }
        // The belt goes on where it was: the parts picked and missed so far stay so.
        let mut conveyor = Conveyor::from_config(cfg, machine.axes().len())?;
        // Nothing fails from here on: a refused config leaves the controller as it was.
        if profile_cycle != self.profile_cycle {
            self.rescale_ticks(self.profile_cycle / profile_cycle);
        }
        if let (Some(new), Some(old)) = (&mut conveyor, &self.conveyor) {
            new.carry_on(old);
        }
        // A cycle under way goes on with a segment the new cycle has: the move in progress finishes where it
        // was going, and the next one starts from there. A cycle that lost all its segments ends; a conveyor cycle
        // goes on with the new conveyor's next pass.
        self.cycle = if conveyor.is_some() {
            self.cycle.map(|_| 0)
        } else {
            self.cycle.and_then(|i| Some(i.min(segments.len().checked_sub(1)?)))
        };
        if conveyor.is_none() && matches!(self.motion, Motion::Conveyor) {
            self.motion = Motion::Idle;
        }
        self.conveyor = conveyor;
        // A cam under way with another master speed or ramp: a ramp from the master's speed now to the new one, from
        // where it stands (a run-down under way goes on as it was).
        if let (Some(new), Motion::Cam { start_tick, stop: None, from, from_speed, speed, ramp, .. }) =
            (machine.cam(), &mut self.motion)
            && (new.speed != *speed || new.ramp_s != *ramp)
            && *start_tick != u64::MAX
        {
            let t = self.tick.saturating_sub(*start_tick) as f64 * self.profile_cycle;
            let now = crate::cam::master_change(*from_speed, *speed, *ramp, t, None);
            (*start_tick, *from, *from_speed) = (self.tick, *from + now[0], now[1]);
            (*speed, *ramp) = (new.speed, new.ramp_s);
        }
        self.cam = machine.cam();
        // The plan of the next cycle is noted anew with the new machine, as from the next base step: a cam's curve may
        // have changed under the same motion. (The profile cycle under way began at the base step of `clock`; in new
        // profile ticks with a changed cycle.) A controller being built has no plan yet.
        if !self.plans.is_empty() {
            let (old_steps, cycle_ratio) = (self.n_profile as f64, self.profile_cycle / profile_cycle);
            let tick_step = (self.clock * old_steps / self.profile_cycle).round();
            let into = ((self.base_step + 1) as f64 - tick_step).max(0.0);
            self.plan_changed_at = Some(self.tick as f64 + into / old_steps * cycle_ratio);
        }
        // The valves keep their commands; a machine with other valves starts them as it says.
        let valves = machine.valves();
        if valves.len() != self.valves.len() {
            self.valves = valves.into_iter().map(|(_, command)| command).collect();
        }
        self.gantry_sync = sync;
        (self.n_velocity, self.n_position, self.n_profile) = (n_velocity, n_position, n_profile);
        (self.profile_cycle, self.setpoint_ticks, self.params, self.segments) =
            (profile_cycle, setpoint_ticks, params, segments);
        self.motor_params = motor_params;
        (self.fault_settings, self.monitoring) = (fault_settings, monitoring);
        (self.brake_delays, self.ss1_time) = (brake_delays, ss1_time);
        self.watch.resize(self.motor_params.len(), Watch::default());
        (self.inertias, self.load_encoder) = (inertias, load_encoder);
        self.blend_time = blend_time;
        (self.path_tolerance, self.cycle_time_optimal, self.plan_limits) =
            (path_tolerance, cycle_time_optimal, plan_limits);
        self.dc_link_rooms = dc_link_rooms;
        self.friction_signals = machine
            .motors()
            .iter()
            .map(|m| reported.iter().position(|s| s.id == format!("motor.{}.torque_friction", m.id)))
            .collect();
        (self.law, self.spline_c2) = (law, spline_c2);
        (self.pending_shaper, self.fieldbus_delay, self.fieldbus_jitter) =
            (Some(shaper), fieldbus_delay, fieldbus_jitter);
        Ok(())
    }

    /// A new profile cycle, `ratio` of them to an old one: every stored tick keeps its distance in time from
    /// now, to the nearest new cycle, and the shaper's past is read off at the new spacing. Without this a
    /// motion under way would jump to another point of its plan.
    fn rescale_ticks(&mut self, ratio: f64) {
        let now = self.tick;
        // The shaper in use counts its period in cycles too. (A pending one was worked out for the new cycle.)
        for mode in self.shaper.iter_mut().flatten() {
            mode.half *= ratio;
            if let Some((_, per_hz)) = &mut mode.follows {
                *per_hz *= ratio;
            }
        }
        for half in self.shaper_targets.iter_mut().flatten() {
            *half *= ratio;
        }
        self.tick = (now as f64 * ratio).round() as u64;
        let new_now = self.tick as f64;
        for (step, at, length) in &mut self.shaper_ramps {
            step.iter_mut().for_each(|half| *half *= ratio);
            (*at, *length) = (new_now + (*at - now as f64) * ratio, *length * ratio);
        }
        // A tick of u64::MAX is a motion not yet picked up.
        let scale = |tick: &mut u64| {
            if *tick != u64::MAX {
                *tick = (new_now + (*tick as f64 - now as f64) * ratio).round().max(0.0) as u64;
            }
        };
        let scale_motion = |motion: &mut Motion| match motion {
            // A conveyor pass runs on the simulated time, which a new profile cycle does not change.
            Motion::Idle | Motion::Conveyor => {}
            Motion::Moving { start_tick, tail, .. } => {
                scale(start_tick);
                if let Some((_, old_start)) = tail {
                    scale(old_start);
                }
            }
            Motion::Dwell { until_tick } => scale(until_tick),
            Motion::Following { start_tick, .. }
            | Motion::Trajectory { start_tick, .. }
            | Motion::Cam { start_tick, .. }
            | Motion::Halting { start_tick, .. } => scale(start_tick),
        };
        scale_motion(&mut self.motion);
        let scale_at = |at: &mut f64| *at = new_now + (*at - now as f64) * ratio;
        self.plan_changed_at.iter_mut().for_each(scale_at);
        for plan in &mut self.plans {
            plan.from.iter_mut().for_each(scale_at);
            scale(&mut plan.first);
            scale_motion(&mut plan.motion);
            plan.key = plan_key(&plan.motion, plan.pass.as_ref().map(|(pass, _)| pass), plan.first);
        }
        if let Some((at, _)) = &mut self.gripping {
            scale(at);
        }
        self.dwell_ticks = (self.dwell_ticks as f64 * ratio).round() as u64;
        if let Some(newest) = self.history.len().checked_sub(1) {
            let old = std::mem::take(&mut self.history);
            // From the oldest stored moment to now, in steps of the new cycle; between two stored cycles, in proportion.
            for back in (0..=(newest as f64 * ratio).floor() as usize).rev() {
                let (whole, part) = ((back as f64 / ratio).floor() as usize, (back as f64 / ratio).fract());
                let (a, b) = (&old[newest - whole.min(newest)], &old[newest - (whole + 1).min(newest)]);
                self.history.push_back(std::array::from_fn(|which| {
                    a[which].iter().zip(&b[which]).map(|(a, b)| a * (1.0 - part) + b * part).collect()
                }));
            }
            self.shaper_reach = 0;
        }
        // The fieldbus queue is resampled the same way, its delays (in cycles) scaled with it: the frames under way
        // arrive when they would have; the configured delay in the new cycles holds for the frames sent from now on.
        if !self.fieldbus.is_empty() {
            self.fieldbus = resample_fieldbus(&self.fieldbus, ratio);
            self.fieldbus_held = u64::MAX;
        }
    }

    /// The setpoints `t` into a ramp down (`ramp_down`) from `from` at speeds `vel` to rest after `time`.
    fn halt_at(&mut self, from: &[f64], vel: &[f64], time: f64, t: f64) {
        halt(from, vel, time, t, [&mut self.set_pos, &mut self.set_vel, &mut self.set_acc]);
    }

    /// The ramp down to rest from the setpoint's speed now (a category 2 stop, or a cam taken away under way).
    fn ramp_down(&self, machine: &dyn Machine) -> Motion {
        let axes = machine.axes();
        // The axis that needs longest to stop at its limit sets the time; the others decelerate in proportion.
        let mut time = self.raw_vel.iter().zip(axes).map(|(v, a)| v.abs() / a.max_acc).fold(0.0, f64::max);
        // A stop that would end beyond the range (a plan decelerating at its limit into an end) ends shorter; one that
        // starts beyond it (a cam's slave turning on past its range, as a rotary knife's does) at the limit.
        for _ in 0..if in_range(machine, &self.raw_pos) { 8 } else { 0 } {
            let end: Vec<f64> = self.raw_pos.iter().zip(&self.raw_vel).map(|(x, v)| x + 0.5 * v * time).collect();
            if in_range(machine, &end) {
                break;
            }
            time *= 0.5;
        }
        if time > 0.0 {
            Motion::Halting { from: self.raw_pos.clone(), vel: self.raw_vel.clone(), time, start_tick: u64::MAX }
        } else {
            Motion::Idle
        }
    }

    pub fn command(&mut self, command: Command, machine: &dyn Machine) -> std::result::Result<(), CommandError> {
        let off = matches!(command, Command::Enable | Command::Disable | Command::Halt(StopCategory::Zero));
        if !self.enabled && !off {
            return Err(CommandError { key: "error-disabled" });
        }
        if self.reacting.is_some() && !off {
            return Err(CommandError { key: "error-fault" });
        }
        // A plan would act on the drives in position mode only, and be dropped when the others come back to it.
        let planning = !off && !matches!(command, Command::Stop | Command::Halt(_));
        if planning && self.drives.iter().any(|d| d.mode() != DriveMode::Position) {
            return Err(CommandError { key: "error-outside-mode" });
        }
        match command {
            Command::Disable | Command::Halt(StopCategory::Zero) => self.disable(),
            Command::Halt(StopCategory::One) => {
                (self.motion, self.cycle, self.stopping) = (Motion::Idle, None, false);
                let ramps = self.drives.iter_mut().map(|d| {
                    d.pulses_off = false;
                    Some(d.measured_speed())
                });
                self.reacting = Some(ramps.collect());
                self.halt = Some(Halt { left: self.ss1_time, closing: None });
            }
            Command::Halt(StopCategory::Two) => {
                (self.cycle, self.stopping) = (None, false);
                self.motion = self.ramp_down(machine);
            }
            Command::Enable => {
                // Enabling acknowledges a trip; drives still stopping are taken off and on again, holding where
                // they stand.
                if !self.enabled || self.reacting.is_some() {
                    (self.enabled, self.reacting, self.fault, self.halt) = (true, None, None, None);
                    self.hold_here(machine, None);
                }
            }
            Command::MoveTo(target) => {
                refuse_target(machine, &target)?;
                if !matches!(self.motion, Motion::Idle) || self.cycle.is_some() {
                    return Err(CommandError { key: "error-busy" });
                }
                // The tick is filled in when the profile generator picks the move up.
                self.motion = Motion::Moving {
                    mv: Move::with_law(&self.raw_pos, &target, machine.axes(), &self.law),
                    start_tick: u64::MAX,
                    tail: None,
                };
            }
            Command::StartCycle => {
                if !matches!(self.motion, Motion::Idle) || self.cycle.is_some() {
                    return Err(CommandError { key: "error-busy" });
                }
                if self.segments.is_empty() && self.conveyor.is_none() && self.cam.is_none() {
                    return Err(CommandError { key: "error-no-cycle" });
                }
                self.cycle = Some(self.segments.len().saturating_sub(1));
                self.stopping = false;
            }
            Command::Stop => self.stopping = true,
            Command::Setpoint(target) => {
                refuse_target(machine, &target)?;
                if self.busy() {
                    return Err(CommandError { key: "error-busy" });
                }
                // Further than an axis may travel in one setpoint interval: carried out, that is a jump, not a
                // setpoint to follow (a PLC that sends a stale or a wrong value).
                // Measured from the last setpoint, not from where the plan stands on its way there: a stream
                // whose setpoints come a cycle early would otherwise be refused from 10/11 of the speed limit on.
                let reach = self.setpoint_ticks as f64 * self.profile_cycle * (1.0 + 1e-9);
                if target
                    .iter()
                    .zip(self.external_setpoint())
                    .zip(machine.axes())
                    .any(|((to, from), axis)| (to - from).abs() > axis.max_vel * reach)
                {
                    return Err(CommandError { key: "error-setpoint-too-far" });
                }
                self.motion = Motion::Following { from: self.raw_pos.clone(), to: target, start_tick: u64::MAX };
            }
            Command::Trajectory(points) => self.command_trajectory(points, None, machine)?,
            Command::RatedTrajectory(points, speeds, accelerations) => {
                let fits = |rates: &[Vec<f64>]| {
                    rates.len() == points.len() && rates.iter().all(|r| r.len() == self.set_pos.len())
                };
                if !fits(&speeds) || !fits(&accelerations) {
                    return Err(CommandError { key: "error-trajectory" });
                }
                self.command_trajectory(points, Some((speeds, accelerations)), machine)?
            }
        }
        Ok(())
    }

    /// `Command::Trajectory` and, with the speeds and accelerations at its points, `Command::RatedTrajectory`.
    fn command_trajectory(
        &mut self,
        points: Vec<(f64, Vec<f64>)>,
        rates: Option<(Vec<Vec<f64>>, Vec<Vec<f64>>)>,
        machine: &dyn Machine,
    ) -> std::result::Result<(), CommandError> {
        if !matches!(self.motion, Motion::Idle) || self.cycle.is_some() {
            return Err(CommandError { key: "error-busy" });
        }
        let rising = points.windows(2).all(|w| w[1].0 > w[0].0);
        // The setpoint of a machine at rest is where its motors stand, which under belt stretch and gear
        // twist is some hundredths of a millimetre from the tool: at an end of a stroke, beyond it. A
        // path from there starts that far outside the range (`near_start`). Judged from the path's own start,
        // which may be up to that slack off this setpoint (below): a path planned by another simulation from
        // where this machine's tool stands starts where that one's motors stand, on an arm some mrad beyond a
        // stop where this setpoint is on it, and was refused.
        if points.len() < 2
            || points[0].0 != 0.0
            || !rising
            || !points.iter().all(|(t, pose)| t.is_finite() && near_start(machine, &points[0].1, pose))
        {
            return Err(CommandError { key: "error-trajectory" });
        }
        // A path that starts elsewhere would make the setpoint jump.
        if points[0].1.iter().zip(&self.raw_pos).any(|(p, s)| (p - s).abs() > machine.setpoint_slack()) {
            return Err(CommandError { key: "error-trajectory-start" });
        }
        self.motion = match rates {
            Some((speeds, accelerations)) => {
                Motion::Trajectory { points, speeds, accelerations: Some(accelerations), start_tick: u64::MAX }
            }
            None => trajectory(points, self.spline_c2),
        };
        Ok(())
    }

    /// Puts motor `k`'s drive in `mode` (`DriveMode`), `state` the machine's. Refused while the drives are off or
    /// stopping after a trip, and out of position mode while a move, cycle or path runs; while a drive is out of position
    /// mode, the plan commands are refused. A drive coming back to position mode takes the plan with it: the plan stands
    /// where the motors stand, at rest, and every drive starts its PI from the torque it asked for last
    /// (`Drive::set_mode`), so the torque does not jump; at speed, the position loop then brings the motors back to
    /// where they were at the switch. Enabling the drives puts them all in position mode.
    pub fn set_drive_mode(
        &mut self,
        k: usize,
        mode: DriveMode,
        machine: &dyn Machine,
        state: &[f64],
    ) -> std::result::Result<(), CommandError> {
        if !self.enabled {
            return Err(CommandError { key: "error-disabled" });
        }
        if self.reacting.is_some() {
            return Err(CommandError { key: "error-fault" });
        }
        // A stepper's step generator and a V/f converter take no torque setpoint.
        if matches!(mode, DriveMode::Torque(_)) && self.motor_params[k].motor.open_loop() {
            return Err(CommandError { key: "error-open-loop" });
        }
        let was = self.drives[k].mode();
        if mode != DriveMode::Position && was == DriveMode::Position && self.busy() {
            return Err(CommandError { key: "error-busy" });
        }
        self.drives[k].set_mode(mode);
        if mode == DriveMode::Position && was != DriveMode::Position {
            machine.motor_state(state, &mut self.motor_pos, &mut self.motor_vel);
            machine.forward(&self.motor_pos, &mut self.set_pos);
            self.raw_pos.clone_from(&self.set_pos);
            for rate in [&mut self.set_vel, &mut self.set_acc, &mut self.raw_vel, &mut self.raw_acc] {
                rate.fill(0.0);
            }
            (self.motion, self.cycle, self.stopping) = (Motion::Idle, None, false);
            self.history.clear();
            self.plans.clear();
            self.fieldbus.clear();
            for (drive, pos) in self.drives.iter_mut().zip(&self.motor_pos) {
                if drive.mode() == DriveMode::Position {
                    (drive.pos_set, drive.position_setpoint, drive.vel_set, drive.acc_set) = (*pos, *pos, 0.0, 0.0);
                }
                drive.retune();
            }
        }
        Ok(())
    }

    /// Drives off: a move or cycle under way is dropped.
    fn disable(&mut self) {
        (self.motion, self.cycle, self.stopping, self.enabled) = (Motion::Idle, None, false, false);
        (self.reacting, self.halt) = (None, None);
    }

    /// Whether the holding brakes are asked to hold: with the drives off, and at the end of a category 1 stop once all
    /// drives stand (`StopCategory::One`).
    fn brake_wanted(&self) -> bool {
        !self.enabled || self.halt.is_some_and(|h| h.closing.is_some())
    }

    /// One base step of `dt` of the holding brakes' command: they engage `brake.engage_delay_s` after they are asked to
    /// hold and release `release_delay_s` after they are asked to let go (effects.brake_delay; at once without it).
    pub fn step_brake(&mut self, dt: f64) {
        let wanted = self.brake_wanted();
        if wanted == self.brake_on {
            self.brake_timer = 0.0;
            return;
        }
        self.brake_timer += dt;
        // To the nearest step: the summed steps come a rounding short of a delay of whole steps.
        if self.brake_timer >= self.brake_delays[usize::from(!wanted)] - 0.5 * dt {
            (self.brake_on, self.brake_timer) = (wanted, 0.0);
        }
    }

    /// Whether the holding brakes hold (`step_brake`).
    pub fn brake_engaged(&self) -> bool {
        self.brake_on
    }

    /// Whether the drives are stopping after a trip (`react`).
    pub fn reacting(&self) -> bool {
        self.reacting.is_some()
    }

    /// Whether a category 2 stop (`StopCategory::Two`) is braking the plan.
    pub fn halting(&self) -> bool {
        matches!(self.motion, Motion::Halting { .. })
    }

    /// Each drive's fault reaction (effects.drive_monitoring).
    pub fn fault_reaction(&self, k: usize) -> fault::Reaction {
        self.fault_settings[k].reaction
    }

    /// A monitor of drive `k` has tripped at `time`: the motion is dropped and every drive stops by its own fault
    /// reaction (`react`). The first trip since the last enable is kept in `fault`.
    fn trip(&mut self, monitor: Monitor, k: usize, time: f64, machine: &dyn Machine, state: &[f64]) {
        if self.fault.is_none() {
            let axes = self.set_pos.len();
            let (mut pose, mut rate) = (vec![0.0; axes], vec![0.0; axes]);
            machine.tool_state(state, &mut pose, &mut rate);
            let stopping = vec![0.0; axes];
            self.fault = Some(Fault { monitor, drive: k, time, pose, stopping, stopped: None });
        }
        (self.motion, self.cycle, self.stopping) = (Motion::Idle, None, false);
        let ramps = self.drives.iter_mut().zip(&self.fault_settings).map(|(d, s)| {
            d.pulses_off = s.deceleration().is_none();
            (!d.pulses_off).then(|| d.measured_speed())
        });
        self.reacting = Some(ramps.collect());
    }

    /// One base step of the drives stopping after a trip: each with a ramp has its speed setpoint taken down by it and
    /// its velocity and current loops follow (the position loop open); once the ramp is down and the motor below the
    /// pulse-off speed, its pulses go off, as an STO drive's did at the trip. A drive with its pulses off gives no
    /// torque (`disabled_step`; its winding is left out of the integration). When the last is off, the drives are
    /// disabled and the holding brakes engage.
    fn react(&mut self, step: u64, machine: &dyn Machine, state: &[f64], torque: &mut [f64]) {
        let Some(mut ramps) = self.reacting.take() else { return };
        let dt = self.profile_cycle / self.n_profile as f64;
        let velocity_due = step.is_multiple_of(self.n_velocity);
        // A category 1 stop: every drive on its OFF3 ramp, its pulses off only together, once all stand and their brakes
        // have had their engagement delay, or when the SS1 timer runs out.
        if let Some(halt) = &mut self.halt {
            halt.left -= dt;
            let standing = self
                .drives
                .iter()
                .zip(&ramps)
                .zip(&self.fault_settings)
                .all(|((d, r), s)| r.is_none_or(|v| v == 0.0) && d.measured_speed().abs() < s.pulse_off_speed);
            if standing {
                halt.closing = Some(halt.closing.unwrap_or(0.0) + dt);
            }
            let done = halt.closing.is_some_and(|t| t >= self.brake_delays[0] - 0.5 * dt);
            if halt.left <= 0.0 || done {
                ramps.iter_mut().for_each(|r| *r = None);
                self.drives.iter_mut().for_each(|d| d.pulses_off = true);
            }
        }
        for (k, drive) in self.drives.iter_mut().enumerate() {
            let p = &self.motor_params[k];
            if let Some(speed) = ramps[k] {
                let deceleration = match self.halt {
                    Some(_) => self.fault_settings[k].off3_deceleration,
                    None => self.fault_settings[k].deceleration().unwrap_or(f64::INFINITY),
                };
                let ramped = fault::ramp(speed, deceleration, dt);
                if self.halt.is_none()
                    && ramped == 0.0
                    && drive.measured_speed().abs() < self.fault_settings[k].pulse_off_speed
                {
                    (ramps[k], drive.pulses_off) = (None, true);
                } else {
                    (ramps[k], drive.fault_speed) = (Some(ramped), Some(ramped));
                    if velocity_due {
                        drive.read_encoder(p, self.motor_pos[k]);
                        drive.velocity_loop(p, self.motor_vel[k]);
                    }
                    drive.current_control(p, drive.measured_speed());
                    continue;
                }
            }
            drive.disabled_step(p, self.motor_vel[k]);
            torque[k] = drive.torque;
        }
        if self.params.shared_dc_link {
            shared_dc_link(&self.params, &mut self.drives);
        }
        self.track_stop(step, machine, state);
        if ramps.iter().all(Option::is_none) {
            self.disable();
        } else {
            self.reacting = Some(ramps);
        }
    }

    /// After a trip, until the machine stands: how far each tool axis has gone since (`Fault::stopping`), and when
    /// every drive is off and every motor below its pulse-off speed (`Fault::stopped`). An STO drive's motor coasts
    /// until then (or its holding brake stops it, once all drives are off).
    fn track_stop(&mut self, step: u64, machine: &dyn Machine, state: &[f64]) {
        let Some(fault) = self.fault.as_mut().filter(|f| f.stopped.is_none()) else { return };
        let axes = fault.pose.len();
        let (mut pose, mut rate) = (vec![0.0; axes], vec![0.0; axes]);
        machine.tool_state(state, &mut pose, &mut rate);
        fault.stopping = pose.iter().zip(&fault.pose).map(|(now, then)| (now - then).abs()).collect();
        let standing = self.motor_vel.iter().zip(&self.fault_settings).all(|(w, s)| w.abs() < s.pulse_off_speed);
        if !self.enabled && standing {
            fault.stopped = Some(step as f64 * self.profile_cycle / self.n_profile as f64);
        }
    }

    /// The timing for a path of lines and arcs from where the setpoint stands, as the poses that
    /// `Command::Trajectory` runs: within the axis limits, or as fast as the motors allow (see `path`).
    pub fn plan_path(
        &self,
        machine: &dyn Machine,
        pieces: &[Piece],
        time_optimal: bool,
    ) -> std::result::Result<Vec<(f64, Vec<f64>)>, CommandError> {
        self.plan_from(machine, &self.raw_pos, pieces, time_optimal, None)
    }

    /// The timing for a program (`program::read`) from where the setpoint stands, as the poses `Command::Trajectory`
    /// runs: each run planned from where the one before ends, at rest between them. A run with a feed is planned within
    /// the axis limits with the speed along the path of each piece capped at its own feed (`path::plan_feeds`); one without as `plan_path`
    /// plans it, as fast as the motors allow with `time_optimal`. A program without a move is refused (`error-path`).
    pub fn plan_program(
        &self,
        machine: &dyn Machine,
        runs: &[crate::program::Run],
        time_optimal: bool,
    ) -> std::result::Result<Vec<(f64, Vec<f64>)>, CommandError> {
        let mut points = vec![(0.0, self.raw_pos.clone())];
        for run in runs.iter().filter(|run| !run.pieces.is_empty()) {
            let (t0, start) = points[points.len() - 1].clone();
            let planned = self.plan_from(
                machine,
                &start,
                &run.pieces,
                time_optimal && run.feed.is_none(),
                run.caps().as_deref(),
            )?;
            // Each plan starts at its start at 0 s, which the one before has as its end.
            points.extend(planned.into_iter().skip(1).map(|(t, pose)| (t0 + t, pose)));
        }
        if points.len() < 2 {
            return Err(CommandError { key: "error-path" });
        }
        Ok(points)
    }

    /// `plan_path` from `start`, with the speed along the path of piece i capped at `feeds[i]` where there are any (then
    /// within the axis limits, also with `time_optimal`).
    fn plan_from(
        &self,
        machine: &dyn Machine,
        start: &[f64],
        pieces: &[Piece],
        time_optimal: bool,
        feeds: Option<&[f64]>,
    ) -> std::result::Result<Vec<(f64, Vec<f64>)>, CommandError> {
        let time_optimal = time_optimal && feeds.is_none();
        // An arc or a spline can swing out of the range although every point given is inside. Said here,
        // because `Command::Trajectory` refuses such poses with a message about poses at times.
        let reachable = |points: &[(f64, Vec<f64>)]| {
            if points.iter().all(|(_, pose)| near_start(machine, start, pose)) {
                Ok(())
            } else {
                Err(CommandError { key: "error-path-range" })
            }
        };
        let within_axis_limits = || {
            let (cycle, slack) = (self.profile_cycle, machine.setpoint_slack());
            let (feeds, spin) = (feeds.unwrap_or(&[]), self.law.spin);
            path::plan_within(machine.axes(), spin, start, pieces, feeds, cycle, self.path_tolerance, slack)
        };
        if time_optimal {
            // A pose on the way that the machine cannot go to (an arm's singularity, where the motors would have to
            // turn infinitely fast) would time the plan to a standstill there: so the same path is checked first as
            // planned within the axis limits, where the timing does not depend on the kinematics (what that plan
            // refuses otherwise is left to the time-optimal one).
            if let Ok(points) = within_axis_limits() {
                reachable(&points)?;
            }
        }
        let points = if time_optimal {
            self.plan_time_optimal_from(machine, start, pieces, self.path_tolerance)
        } else {
            within_axis_limits()
        }?;
        reachable(&points)?;
        Ok(points)
    }

    /// `path::plan_time_optimal` from `from` with this controller's drive, limits and cycle, within `tolerance`.
    fn plan_time_optimal_from(
        &self,
        machine: &dyn Machine,
        from: &[f64],
        pieces: &[Piece],
        tolerance: path::Tolerance,
    ) -> std::result::Result<Vec<(f64, Vec<f64>)>, CommandError> {
        path::plan_time_optimal(
            machine,
            &self.motor_drives(machine),
            &self.plan_limits,
            from,
            pieces,
            self.profile_cycle,
            tolerance,
            machine.setpoint_slack(),
        )
    }

    /// How far the path `plan_path` plans for these pieces is from the pieces as given, at most (see
    /// `path::deviation`): zero unless `profile.path_tolerance_m` allows transition curves. The time-optimal plan's
    /// curves are as long as the motors' speeds need, so its deviation can differ. A path `plan_path` refuses is
    /// refused here the same way (it is planned for that).
    pub fn path_deviation(
        &self,
        machine: &dyn Machine,
        pieces: &[Piece],
        time_optimal: bool,
    ) -> std::result::Result<f64, CommandError> {
        self.plan_path(machine, pieces, time_optimal)?;
        if time_optimal {
            path::deviation_time_optimal(
                machine,
                &self.motor_drives(machine),
                &self.plan_limits,
                &self.raw_pos,
                pieces,
                self.path_tolerance,
                machine.setpoint_slack(),
            )
        } else {
            path::deviation(machine.axes(), &self.raw_pos, pieces, self.path_tolerance, machine.setpoint_slack())
        }
    }

    /// `path_deviation` per run of a program (`plan_program`: each run from where the one before ends, a run with a
    /// feed within the axis limits). A run with a feed reads as without its feeds (`path::deviation`): never lower
    /// than its own path's.
    pub fn program_deviations(
        &self,
        machine: &dyn Machine,
        runs: &[crate::program::Run],
        time_optimal: bool,
    ) -> std::result::Result<Vec<f64>, CommandError> {
        let mut start = self.raw_pos.clone();
        let mut out = Vec::new();
        for run in runs.iter().filter(|run| !run.pieces.is_empty()) {
            let optimal = time_optimal && run.feed.is_none();
            let planned = self.plan_from(machine, &start, &run.pieces, optimal, run.caps().as_deref())?;
            out.push(if optimal {
                path::deviation_time_optimal(
                    machine,
                    &self.motor_drives(machine),
                    &self.plan_limits,
                    &start,
                    &run.pieces,
                    self.path_tolerance,
                    machine.setpoint_slack(),
                )?
            } else {
                path::deviation(machine.axes(), &start, &run.pieces, self.path_tolerance, machine.setpoint_slack())?
            });
            if let Some((_, end)) = planned.last() {
                start.clone_from(end);
            }
        }
        Ok(out)
    }

    /// True while a move, a dwell or the cycle is under way.
    pub fn busy(&self) -> bool {
        self.cycle.is_some()
            || matches!(
                self.motion,
                Motion::Moving { .. }
                    | Motion::Dwell { .. }
                    | Motion::Trajectory { .. }
                    | Motion::Conveyor
                    | Motion::Cam { .. }
                    | Motion::Halting { .. }
            )
    }

    /// False between `Command::Disable` and `Command::Enable`.
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn cycle_segment(&self) -> Option<usize> {
        self.cycle
    }

    /// The conveyor of `[conveyor]`, where the config has one, and the pass of its cycle under way or last run.
    pub fn conveyor(&self) -> Option<(&Conveyor, Option<&Pass>)> {
        self.conveyor.as_ref().map(|c| (c, self.pass.as_ref()))
    }

    /// How often the cycle has started its first segment since the controller was made. The segment index
    /// alone does not tell one pass from the next when the cycle has a single segment.
    pub fn cycle_passes(&self) -> u64 {
        self.passes
    }

    /// Where an electronic cam's master stands (`Machine::cam`), in its units; kept between runs.
    pub fn cam_master(&self) -> f64 {
        self.cam_master
    }

    /// The torque the drives can give at a motor speed: (lowest, highest).
    pub fn torque_curve(&self, speed: f64) -> (f64, f64) {
        self.params.torque_curve(speed)
    }

    /// The torque motor `k`'s drive can give at a motor speed: (lowest, highest).
    pub fn motor_torque_curve(&self, k: usize, speed: f64) -> (f64, f64) {
        self.drive_params_of(k).torque_curve(speed)
    }

    /// How far a dwell of the cycle has run, from 0 to 1; `None` outside a dwell.
    pub fn dwell_progress(&self) -> Option<f64> {
        let (Motion::Dwell { until_tick }, Some(_)) = (&self.motion, self.cycle) else { return None };
        let total = self.dwell_ticks;
        (total > 0).then(|| 1.0 - until_tick.saturating_sub(self.tick) as f64 / total as f64)
    }

    /// Segment `i` has arrived at `tick`: the dwell starts. Where the gripper changes its state, with
    /// effects.gripper_time it closes or opens for its time (`Segment::grip_ticks`), the dwell lasts at least that long
    /// and the payload comes or goes when the gripper is done; without, at once.
    fn arrive(&mut self, i: usize, tick: u64) -> Motion {
        let s = &self.segments[i];
        let grip = if s.payload != self.payload_attached { s.grip_ticks } else { 0 };
        if grip == 0 {
            self.payload_attached = s.payload;
        } else {
            self.gripping = Some((tick + grip, s.payload));
        }
        self.dwell_ticks = s.dwell_ticks.max(grip);
        self.arrived(i, tick)
    }

    /// Whether the gripper is closing or opening (effects.gripper_time).
    pub fn gripper_acting(&self) -> bool {
        self.gripping.is_some()
    }

    /// The setpoint as planned, before input shaping: during a dwell, the target.
    pub fn planned_pos(&self) -> &[f64] {
        &self.raw_pos
    }

    /// Where a setpoint from outside would leave from: the target of the one the controller still
    /// interpolates towards, else where the plan stands. A link to a PLC shows this as its setpoint, so that
    /// an axis the PLC does not write stays where it is.
    pub fn external_setpoint(&self) -> &[f64] {
        match &self.motion {
            Motion::Following { to, .. } => to,
            _ => &self.raw_pos,
        }
    }

    /// The torque the motors can give continuously.
    pub fn rated_torque(&self) -> f64 {
        self.params.rated_torque
    }

    /// Motor and drive data, for the analysis.
    pub fn drive_params(&self) -> &DriveParams {
        &self.params
    }

    /// The limits the time-optimal plan keeps beside the motors' (`path::PlanLimits`), as the config gives them.
    pub fn plan_limits(&self) -> &path::PlanLimits {
        &self.plan_limits
    }

    /// Motor k's DC link room above the bus, J: 1/2 C (U_threshold^2 - U_bus^2) of its own ratings; None without them.
    pub fn dc_link_room(&self, k: usize) -> Option<f64> {
        self.dc_link_rooms.get(k).copied().flatten()
    }

    /// Motor and drive data of motor `k`: `drive_params` with that motor's own values (`catalog::motor_config`).
    pub fn drive_params_of(&self, k: usize) -> &DriveParams {
        self.motor_params.get(k).unwrap_or(&self.params)
    }

    /// Each motor's own drive and motor data (`drive_params_of`), in the order of `Machine::motors`.
    pub fn motor_drives(&self, machine: &dyn Machine) -> Vec<&DriveParams> {
        (0..machine.motors().len()).map(|k| self.drive_params_of(k)).collect()
    }

    /// The motors stand `shift` from where they stood without having turned there: the machine's model was
    /// exchanged, and belt stretch and gear twist came or went with it, or its ratio changed. The encoders'
    /// last readings move along, so that the step does not read as speed, and so do the position setpoints
    /// at the drives and on the fieldbus, so that it does not read as following error. The next profile
    /// cycle works them out anew.
    pub fn motors_shifted(&mut self, shift: &[f64]) {
        for (k, (drive, by)) in self.drives.iter_mut().zip(shift).enumerate() {
            drive.shift_encoder(*by);
            drive.shift_setpoint(*by);
            for (_, setpoints) in &mut self.fieldbus {
                setpoints[k][0] += by;
            }
        }
    }

    /// One base step: runs whichever loops are due and sets the voltages the windings get over the step. Enabled, the
    /// simulation then takes windings and mechanics through the step together and hands the windings back
    /// (`winding_stepped`). Disabled, the drives give no torque, their windings cool and their DC links discharge,
    /// and `torque` is zero.
    pub fn update(&mut self, step: u64, machine: &dyn Machine, state: &[f64], torque: &mut [f64]) {
        self.base_step = step;
        machine.motor_state(state, &mut self.motor_pos, &mut self.motor_vel);
        if !self.enabled {
            for (k, (drive, torque)) in self.drives.iter_mut().zip(torque).enumerate() {
                drive.disabled_step(&self.motor_params[k], self.motor_vel[k]);
                *torque = drive.torque;
            }
            if self.params.shared_dc_link {
                shared_dc_link(&self.params, &mut self.drives);
            }
            self.track_stop(step, machine, state);
            return;
        }
        if self.reacting.is_some() {
            self.react(step, machine, state, torque);
            return;
        }
        if step.is_multiple_of(self.n_profile) {
            self.clock = step as f64 * self.profile_cycle / self.n_profile as f64;
            self.profile_tick(self.tick + 1, machine);
        }
        let (position_due, velocity_due) = (step.is_multiple_of(self.n_position), step.is_multiple_of(self.n_velocity));
        // One encoder sample per base step, which both loops use.
        if position_due || velocity_due {
            for (k, (drive, pos)) in self.drives.iter_mut().zip(&self.motor_pos).enumerate() {
                drive.read_encoder(&self.motor_params[k], *pos);
            }
        }
        if position_due {
            // With a load-side encoder (effects.load_encoder) the position loop measures the tool axes, in whole
            // steps of the scale, as motor angles through the rigid kinematics; the speed loop keeps the motor's.
            // The scale is mounted on the frame: it reads `Machine::scale_state`, the carriage against the frame.
            let load = self.load_encoder.map(|scale| {
                let axes = self.set_pos.len();
                let (mut tool, mut rate, mut motors) = (vec![0.0; axes], vec![0.0; axes], vec![0.0; self.drives.len()]);
                machine.scale_state(state, &mut tool, &mut rate);
                tool.iter_mut().for_each(|x| *x = quantise(*x, scale));
                machine.inverse(&tool, &mut motors);
                motors
            });
            let cycle = self.n_position as f64 * self.profile_cycle / self.n_profile as f64;
            let measured = |k: usize, drive: &Drive| load.as_ref().map_or(drive.encoder(), |l| l[k]);
            // A gantry's master first: master/slave, the slave measures itself against the master's actual position.
            let sync = self.gantry_sync;
            let mut lead = 0.0;
            if let Some(g) = sync {
                let drive = &mut self.drives[g.master];
                drive.position_loop(&self.motor_params[g.master], measured(g.master, drive), cycle);
                if g.cross_gain.is_none() {
                    lead = g.ratio * drive.following_error;
                }
            }
            for (k, drive) in self.drives.iter_mut().enumerate() {
                if sync.is_some_and(|g| g.master == k) {
                    continue;
                }
                let behind = if sync.is_some_and(|g| g.slave == k) { lead } else { 0.0 };
                drive.position_loop(&self.motor_params[k], measured(k, drive) + behind, cycle);
            }
            if let Some(GantrySync { master, slave, ratio, cross_gain: Some(gain) }) = sync {
                // The slave's actual less the master's, in the slave's units, from the following errors (their
                // setpoints stand in the ratio): ratio e_master - e_slave.
                let error = ratio * self.drives[master].following_error - self.drives[slave].following_error;
                for (k, by) in [(slave, -gain * error), (master, gain * error / ratio)] {
                    self.drives[k].position_part += by;
                    self.drives[k].speed_command += by;
                }
            }
            // Following error per motor, transformed back to the tool axes for display.
            let (set, actual): (Vec<f64>, Vec<f64>) =
                self.drives.iter().map(|d| (d.position_setpoint, d.position_setpoint - d.following_error)).unzip();
            let (mut tool_set, mut tool_actual) = (self.set_pos.clone(), self.set_pos.clone());
            machine.forward(&set, &mut tool_set);
            machine.forward(&actual, &mut tool_actual);
            for i in 0..self.tool_error.len() {
                self.tool_error[i] = tool_set[i] - tool_actual[i];
            }
        }
        let dt = self.profile_cycle / self.n_profile as f64;
        for (k, drive) in self.drives.iter_mut().enumerate() {
            if velocity_due {
                drive.velocity_loop(&self.motor_params[k], self.motor_vel[k]);
            }
            // The back-EMF the current loop adds, and the field weakening, at the speed the drive has measured.
            let speed = drive.measured_speed();
            drive.current_control(&self.motor_params[k], speed);
            drive.since_setpoint += dt;
        }
        if self.monitoring {
            // The friction the torque-band monitor expects: what the machine's own friction model takes of each motor's
            // torque at the machine's state (`friction_signals`), so with the guides' bristles and breakaway, the motor's
            // Stribeck curve and the gearbox's mesh friction as they act through a start or a reversal.
            let mut friction = vec![0.0; self.drives.len()];
            let banded = |k: usize| {
                self.fault_settings[k].torque_band > 0.0 && self.friction_signals.get(k).copied().flatten().is_some()
            };
            if (0..self.drives.len()).any(banded) {
                let torques: Vec<f64> = self.drives.iter().map(|d| d.torque).collect();
                let mut signals = Vec::new();
                machine.signals(state, &torques, &mut signals);
                for (k, f) in friction.iter_mut().enumerate().filter(|(k, _)| banded(*k)) {
                    *f = self.friction_signals[k].and_then(|i| signals.get(i).copied()).unwrap_or(0.0);
                }
            }
            let tripped = (0..self.drives.len()).find_map(|k| {
                let (s, peak) = (&self.fault_settings[k], self.motor_params[k].peak_torque);
                self.watch[k].check(s, &self.drives[k], friction[k], peak, dt).map(|m| (m, k))
            });
            if let Some((monitor, k)) = tripped {
                self.trip(monitor, k, step as f64 * dt, machine, state);
            }
        }
    }

    /// The tool setpoint at `time`: the last profile cycle's run on by its speed and acceleration, as the drives' fine
    /// interpolation runs the motors' on (at most one profile cycle). What the analysis measures the tool against.
    pub fn plan_now(&self, time: f64, out: &mut [f64]) {
        let t = self.setpoint_age(time);
        for (i, x) in out.iter_mut().enumerate() {
            *x = self.set_pos[i] + (self.set_vel[i] + 0.5 * self.set_acc[i] * t) * t;
        }
    }

    /// How long ago, at `time`, the profile cycle set `set_pos`, `set_vel` and `set_acc` (the signal `setpoint_age`):
    /// `plan_now` is those run on by it.
    pub fn setpoint_age(&self, time: f64) -> f64 {
        (time - self.clock).clamp(0.0, self.profile_cycle)
    }

    /// The base step done (see `update`): per drive the current at its end and the electrical energy, copper loss and
    /// work on the shaft over it, `WINDING_STATES` numbers per drive in their order, and the rotors' angles and speeds at
    /// its end. Then the shared DC link, and the trip.
    pub fn winding_stepped(&mut self, windings: &[f64], angle: &[f64], speed: &[f64]) {
        for (k, (drive, w)) in self.drives.iter_mut().zip(windings.as_chunks::<WINDING_STATES>().0).enumerate() {
            // A drive whose pulses are off after a trip has had its disabled step (`react`); the iron's drag work over
            // the step is the integrated one (`Sim::advance_with_windings`), not the step start's speed times the step.
            if drive.pulses_off {
                let dt = self.motor_params[k].current_cycle;
                (drive.shaft_power, drive.emf_loss) = (w[3] / dt, -w[3] / dt);
            } else {
                (drive.phases.x, drive.phases.angle, drive.phases.speed) =
                    ([w[0], w[4], w[5], w[6]], angle[k], speed[k]);
                drive.winding_stepped(&self.motor_params[k], w[0], [w[1], w[2], w[3]]);
            }
        }
        if self.params.shared_dc_link {
            shared_dc_link(&self.params, &mut self.drives);
        }
        // A drive that trips on DC link overvoltage takes all drives off, as `Command::Disable` does. Its
        // `tripped` stays until `Command::Enable`; its DC link discharges as a disabled drive's does, and an enable
        // keeps what charge is left (`hold_here`).
        if self.drives.iter().any(|d| d.tripped) {
            self.disable();
        }
    }

    /// Profile generator: new tool setpoints, then the inverse transformation to motor setpoints.
    fn profile_tick(&mut self, tick: u64, machine: &dyn Machine) {
        let dt = self.profile_cycle;
        self.tick = tick;
        // Plan on the setpoints as planned so far, not on their shaped version.
        self.set_pos.clone_from(&self.raw_pos);
        self.set_vel.clone_from(&self.raw_vel);
        self.set_acc.clone_from(&self.raw_acc);
        // effects.gripper_time: the grip is made or lost once the gripper has had its time (`arrive`).
        if let Some((at, holds)) = self.gripping
            && tick >= at
        {
            (self.payload_attached, self.gripping) = (holds, None);
        }
        if matches!(self.motion, Motion::Idle) {
            self.next_cycle_segment(machine);
        }
        let motion = std::mem::replace(&mut self.motion, Motion::Idle);
        self.record_plan(tick, &motion, machine);
        match motion {
            Motion::Idle => {
                self.set_vel.fill(0.0);
                self.set_acc.fill(0.0);
            }
            Motion::Moving { mv, start_tick, mut tail } => {
                let start_tick = if start_tick == u64::MAX { tick } else { start_tick };
                let t = (tick - start_tick) as f64 * dt;
                let old = tail.as_ref().map(|(old, old_start)| (old, (tick - old_start) as f64 * dt));
                move_at(&mv, t, old, [&mut self.set_pos, &mut self.set_vel, &mut self.set_acc]);
                if old.is_some_and(|(old, t_old)| t_old >= old.duration()) {
                    tail = None;
                }
                // With input shaping the setpoint arrives after the plan. Where the gripper acts or the cycle
                // dwells, both wait for the setpoint: the tool is not there before.
                let waits = self.cycle.is_some_and(|i| {
                    self.segments[i].dwell_ticks > 0 || self.segments[i].payload != self.payload_attached
                });
                // The shaper's oldest copy is its whole length old, read between two stored cycles.
                let lag = if waits { self.shaper_span(machine, &self.set_pos).ceil() } else { 0.0 };
                let arrival = mv.duration() + lag * dt;
                self.motion = if t >= arrival && tail.is_none() {
                    match self.cycle {
                        Some(i) => self.arrive(i, tick),
                        None => Motion::Idle,
                    }
                } else {
                    match self.blend_into_next(&mv, t, tail.is_some(), machine) {
                        Some(next) => Motion::Moving { mv: next, start_tick: tick, tail: Some((mv, start_tick)) },
                        None => Motion::Moving { mv, start_tick, tail },
                    }
                };
            }
            Motion::Conveyor => {
                // The pass was planned for the shaped setpoint (`Conveyor::plan`): its grip begins once the shaped
                // setpoint is on the part, so the gripper acts on the plan's time.
                if let Some(pass) = &self.pass {
                    let t = self.clock - pass.start;
                    if t < 0.0 {
                        // Waiting for the part, at rest where the last pass ended.
                        self.set_vel.fill(0.0);
                        self.set_acc.fill(0.0);
                        self.motion = Motion::Conveyor;
                    } else {
                        pass.at(t, &mut self.set_pos, &mut self.set_vel, &mut self.set_acc);
                        self.payload_attached = pass.holds(t);
                        if t < pass.duration() {
                            self.motion = Motion::Conveyor;
                        } else {
                            self.set_vel.fill(0.0);
                            self.set_acc.fill(0.0);
                        }
                    }
                }
            }
            Motion::Cam { start_tick, mut stop, from, offset, mut next_pass, from_speed, speed, ramp } => {
                let start_tick = if start_tick == u64::MAX { tick } else { start_tick };
                let t = (tick - start_tick) as f64 * dt;
                if let Some(master) = self.cam {
                    if self.stopping && stop.is_none() {
                        stop = Some(t);
                    }
                    let run = crate::cam::master_change(from_speed, speed, ramp, t, stop);
                    self.cam_master = from + run[0];
                    // Passes count whole periods of the master from where it is up to speed.
                    if t >= ramp && next_pass.is_nan() {
                        next_pass = self.cam_master + master.period;
                    } else if self.cam_master >= next_pass && stop.is_none() {
                        next_pass += master.period;
                        self.passes += 1;
                    }
                    let out = [&mut self.set_pos, &mut self.set_vel, &mut self.set_acc];
                    cam_slaves(&|m, cam| machine.cam_at(m, cam), self.cam_master, run, &offset, out);
                    if stop.is_some_and(|s| t >= s.max(ramp) + ramp) {
                        // Run down: at rest where the cam has brought the slaves.
                        self.set_vel.fill(0.0);
                        self.set_acc.fill(0.0);
                        (self.cycle, self.stopping) = (None, false);
                    } else {
                        self.motion =
                            Motion::Cam { start_tick, stop, from, offset, next_pass, from_speed, speed, ramp };
                    }
                } else {
                    // A config without the cam now: the slaves ramp down from the speeds they have, every axis within
                    // its acceleration limit, as in a category 2 stop, instead of a step in their speed. (The cam's
                    // curve went with the config, so they decelerate along their last direction.) Its first cycle is
                    // this one, so the setpoint does not stand still for a cycle.
                    (self.cycle, self.stopping) = (None, false);
                    self.set_acc.fill(0.0);
                    if let Motion::Halting { from, vel, time, .. } = self.ramp_down(machine) {
                        self.halt_at(&from, &vel, time, dt.min(time));
                        // This cycle is the stop's, not the cam's.
                        let halting =
                            Motion::Halting { from: from.clone(), vel: vel.clone(), time, start_tick: tick - 1 };
                        self.record_plan(tick, &halting, machine);
                        if dt < time {
                            self.motion = Motion::Halting { from, vel, time, start_tick: tick - 1 };
                        }
                    }
                }
            }
            Motion::Dwell { mut until_tick } => {
                // Waiting for end-position sensors (`arrived`): the dwell counts from when they all report.
                if until_tick == u64::MAX
                    && let Some(i) = self.cycle
                    && self.segments[i].wait.iter().all(|&(valve, end)| machine.end_sensors(valve)[end])
                {
                    until_tick = tick + self.segments[i].dwell_ticks;
                }
                if tick >= until_tick {
                    self.next_cycle_segment(machine);
                } else {
                    self.motion = Motion::Dwell { until_tick };
                }
            }
            Motion::Trajectory { points, speeds, accelerations, start_tick } => {
                let start_tick = if start_tick == u64::MAX { tick } else { start_tick };
                let t = (tick - start_tick) as f64 * dt;
                let last = points.len() - 1;
                let out = [&mut self.set_pos, &mut self.set_vel, &mut self.set_acc];
                path_at(&points, &speeds, accelerations.as_deref(), t, dt, out);
                if t >= points[last].0 {
                    // A cycle segment's path arrives as a move does (see above): the gripper acts and the dwell starts
                    // once the shaped setpoint is there, where the segment waits for it.
                    if let Some(i) = self.cycle {
                        let waits =
                            self.segments[i].dwell_ticks > 0 || self.segments[i].payload != self.payload_attached;
                        let lag = if waits { self.shaper_span(machine, &self.set_pos).ceil() } else { 0.0 };
                        self.motion = if t >= points[last].0 + lag * dt {
                            self.arrive(i, tick)
                        } else {
                            Motion::Trajectory { points, speeds, accelerations, start_tick }
                        };
                    }
                } else {
                    self.motion = Motion::Trajectory { points, speeds, accelerations, start_tick };
                }
            }
            Motion::Following { from, to, start_tick } => {
                // The setpoint came during the cycle before this one, and the way to it starts there. Counted
                // from this cycle, the plan stood still for a cycle at every setpoint of a stream and made up
                // for it a ninth faster than the stream.
                let start_tick = if start_tick == u64::MAX { tick - 1 } else { start_tick };
                let n = self.setpoint_ticks as f64;
                let done = ((tick - start_tick) as f64 / n).min(1.0);
                line_at(&from, &to, done, n * dt, [&mut self.set_pos, &mut self.set_vel, &mut self.set_acc]);
                if done < 1.0 {
                    self.motion = Motion::Following { from, to, start_tick };
                }
            }
            Motion::Halting { from, vel, time, start_tick } => {
                // From the cycle the stop came in, as a setpoint from outside is (`Following`).
                let start_tick = if start_tick == u64::MAX { tick - 1 } else { start_tick };
                let t = ((tick - start_tick) as f64 * dt).min(time);
                self.halt_at(&from, &vel, time, t);
                if t < time {
                    self.motion = Motion::Halting { from, vel, time, start_tick };
                }
            }
        }

        // A changed shaper is taken once the plan stands still and the shaper's store holds nothing but where
        // it stands: then the setpoint is the plan, with the old shaper and with the new, and does not step.
        if self.raw_pos == self.set_pos
            && self.history.iter().all(|past| past[0] == self.raw_pos)
            && let Some(next) = self.pending_shaper.take()
        {
            self.shaper = next;
            (self.shaper_ramps, self.shaper_targets) = (Vec::new(), None);
        }
        self.raw_pos.clone_from(&self.set_pos);
        self.raw_vel.clone_from(&self.set_vel);
        self.raw_acc.clone_from(&self.set_acc);
        self.shape(machine);

        let motors = self.drives.len();
        let (mut pos, mut ahead, mut behind, mut ff) =
            (vec![0.0; motors], vec![0.0; motors], vec![0.0; motors], vec![0.0; motors]);
        let offset_pos: Vec<f64> = self.set_pos.iter().zip(&self.setpoint_offset).map(|(p, o)| p + o).collect();
        machine.inverse(&offset_pos, &mut pos);
        // Motor speed and acceleration setpoints: the inverse transformation along the plan a moment ahead and behind,
        // x(s) = x + v s + a s^2 / 2. The second difference is the motors' whole acceleration, J^-1 a and the part a
        // kinematics that is not linear adds at speed (d/dt J^-1 times v), for the drives' fine interpolation.
        let (h, mut here) = (1e-3, vec![0.0; motors]);
        let along = |s: f64| -> Vec<f64> {
            (0..self.set_pos.len())
                .map(|i| self.set_pos[i] + s * self.set_vel[i] + 0.5 * s * s * self.set_acc[i])
                .collect()
        };
        machine.inverse(&along(h), &mut ahead);
        machine.inverse(&along(-h), &mut behind);
        machine.inverse(&self.set_pos, &mut here);
        let motor_acc: Vec<f64> = (0..motors).map(|k| (ahead[k] - 2.0 * here[k] + behind[k]) / (h * h)).collect();
        self.torque_feedforward(machine, &self.set_pos, &self.set_vel, &self.set_acc, &motor_acc, &mut ff);
        let hold = self.holding_torque(machine, &self.set_pos);
        // The friction of the machine's own model at the planned speed, for the drives that take it and for the
        // torque-band monitor of a machine that does not report its friction torque (`friction_signals`; one that does
        // has the monitor take it at its state every step, `update`), which expects it as it expects the held load.
        let mut friction = vec![0.0; motors];
        let banded: Vec<bool> = (0..motors)
            .map(|k| {
                self.monitoring
                    && self.fault_settings.get(k).is_some_and(|s| s.torque_band > 0.0)
                    && self.friction_signals.get(k).is_none_or(|s| s.is_none())
            })
            .collect();
        if self.motor_params.iter().any(|p| p.precontrol.machine_friction) || banded.contains(&true) {
            machine.steady_friction(&self.set_pos, &self.set_vel, &mut friction);
        }
        for k in 0..motors {
            let p = &self.motor_params[k];
            // The friction the config's feedforward model gives (its characteristic where it has one), else the machine's.
            let expected = if p.precontrol.friction.is_empty() {
                friction[k]
            } else {
                p.precontrol.friction_at((ahead[k] - behind[k]) / (2.0 * h))
            };
            self.drives[k].torque_hold = hold[k] + if banded[k] { expected } else { 0.0 };
            if !p.precontrol.machine_friction {
                friction[k] = 0.0;
            }
        }
        let setpoints = (0..motors)
            .map(|k| [pos[k], (ahead[k] - behind[k]) / (2.0 * h), motor_acc[k], ff[k], friction[k]])
            .collect();
        // Fieldbus: the drives get the newest frame that has arrived, one the controller computed some cycles ago. With
        // jitter each frame takes its own number of cycles, and a late one leaves the drives the frame before. Frames
        // older than the longest delay have arrived and are dropped; before any has arrived, the oldest.
        let late = if self.fieldbus_jitter > 0 {
            (self.jitter.uniform() * (self.fieldbus_jitter + 1) as f64) as usize
        } else {
            0
        };
        self.fieldbus.push_back((self.fieldbus_delay + late, setpoints));
        self.fieldbus_pushed += 1;
        // (The longest stamp in the queue, not the configured delay: after a cycle change the queue drains by itself.)
        let longest =
            self.fieldbus.iter().map(|f| f.0).max().unwrap_or(0).max(self.fieldbus_delay + self.fieldbus_jitter);
        while self.fieldbus.len() > longest + 1 {
            self.fieldbus.pop_front();
        }
        let newest = self.fieldbus.len() - 1;
        let arrived = (0..=newest).find(|&age| self.fieldbus[newest - age].0 <= age).map_or(0, |age| newest - age);
        let number = self.fieldbus_pushed - (self.fieldbus.len() - arrived) as u64;
        let fresh = number != self.fieldbus_held;
        self.fieldbus_held = number;
        for (drive, &[pos, vel, acc, ff, friction]) in self.drives.iter_mut().zip(&self.fieldbus[arrived].1) {
            (drive.pos_set, drive.vel_set, drive.acc_set, drive.torque_ff) = (pos, vel, acc, ff);
            drive.friction_ff = friction;
            if fresh {
                drive.since_setpoint = 0.0;
            }
        }
    }

    /// What the torque-band monitor expects besides the feedforward at `pose`: for a motor with the constant-inertia
    /// feedforward (no gravity in it) the rigid model's torque at rest there, the weight; 0 for the others and without
    /// the monitor.
    fn holding_torque(&self, machine: &dyn Machine, pose: &[f64]) -> Vec<f64> {
        let wanted = |k: usize| {
            self.monitoring
                && self.motor_params[k].precontrol.constant_inertia
                && self.fault_settings.get(k).is_some_and(|s| s.torque_band > 0.0)
        };
        let mut out = vec![0.0; self.drives.len()];
        if (0..out.len()).any(wanted) {
            let still = vec![0.0; pose.len()];
            machine.feedforward_torque_moving(pose, &still, &still, &mut out);
            for (k, torque) in out.iter_mut().enumerate() {
                if !wanted(k) {
                    *torque = 0.0;
                }
            }
        }
        out
    }

    /// The torque feedforward per motor (before its weight): the rigid model's inverse dynamics, or, for a motor whose
    /// drive has `drive.torque_feedforward_model = "inertia"`, its constant moment of inertia (`rigid_inertias`) times
    /// its acceleration setpoint.
    fn torque_feedforward(
        &self,
        machine: &dyn Machine,
        pos: &[f64],
        vel: &[f64],
        acc: &[f64],
        motor_acc: &[f64],
        out: &mut [f64],
    ) {
        machine.feedforward_torque_moving(pos, vel, acc, out);
        for (k, torque) in out.iter_mut().enumerate() {
            if self.motor_params[k].precontrol.constant_inertia {
                *torque = self.inertias[k] * motor_acc[k];
            }
        }
    }

    /// Half the period in profile cycles of the first shaper and of the second (0 without one) with the plan at `pose`:
    /// as configured, or, when the shaper follows the pose, that of the resonance the machine reports there for the
    /// first and that of its second mode (`Machine::modes`) for the second; one the machine does not report stays as
    /// configured. `None` with the shaper off.
    pub fn shaper_halves(&self, machine: &dyn Machine, pose: &[f64]) -> Option<(f64, f64)> {
        let halves = self.mode_halves(machine, pose)?;
        Some((halves[0], halves[1]))
    }

    /// Half the period in profile cycles of every mode of the shaper with the plan at `pose` (see `shaper_halves`; the
    /// modes of `profile.shaper_modes` keep theirs).
    fn mode_halves(&self, machine: &dyn Machine, pose: &[f64]) -> Option<Vec<f64>> {
        let modes = self.shaper.as_ref()?;
        let following = modes.iter().any(|m| m.follows.is_some());
        let reported = if following { machine.modes(pose) } else { Vec::new() };
        Some(
            modes
                .iter()
                .map(|m| match m.follows {
                    Some((0, per_hz)) if m.half > 0.0 => machine.resonance_hz(pose).map_or(m.half, |f| per_hz / f),
                    Some((k, per_hz)) if m.half > 0.0 => reported.get(k).map_or(m.half, |mode| per_hz / mode.hz),
                    _ => m.half,
                })
                .collect(),
        )
    }

    /// Per axis the impulses of the shaper with the modes at the half periods `halves`: the modes acting on the axis
    /// convolved, times in profile cycles, amplitudes adding up to one.
    fn trains(&self, halves: &[f64]) -> Vec<Vec<(f64, f64)>> {
        let modes = self.shaper.as_deref().unwrap_or_default();
        (0..self.set_pos.len())
            .map(|axis| {
                modes.iter().zip(halves).filter(|(m, half)| m.axes[axis] && **half > 0.0).fold(
                    vec![(0.0, 1.0)],
                    |train, (m, half)| {
                        let scaled: Vec<(f64, f64)> = m.impulses.iter().map(|(t, a)| (t * half, *a)).collect();
                        shaper::convolve(&train, &scaled)
                    },
                )
            })
            .collect()
    }

    /// How many profile cycles the shaped setpoint arrives after the plan with the plan at `pose`: the longest of the
    /// axes' shapers; on the ramp to a changed machine's period (`shape`) with the longer of the two. 0 with the shaper
    /// off.
    pub fn shaper_span(&self, machine: &dyn Machine, pose: &[f64]) -> f64 {
        self.mode_halves(machine, pose).map_or(0.0, |halves| {
            let ramp = self.ramp_offset(self.tick as f64);
            span(&self.trains(&halves.iter().zip(ramp).map(|(h, e)| h.max(h + e)).collect::<Vec<_>>()))
        })
    }

    /// With the plan at `pose`: per axis how far the shaped setpoint lags a plan at a constant speed (the mean of the
    /// shaper's delays, s), and how long after the plan it arrives (`shaper_span`, s). Zeros with the shaper off.
    pub fn shaper_delays(&self, machine: &dyn Machine, pose: &[f64]) -> (Vec<f64>, f64) {
        let Some(halves) = self.mode_halves(machine, pose) else { return (vec![0.0; self.set_pos.len()], 0.0) };
        let dt = self.profile_cycle;
        let trains = self.trains(&halves);
        let mean = trains.iter().map(|train| train.iter().map(|(t, a)| t * a).sum::<f64>() * dt).collect();
        (mean, self.shaper_span(machine, pose) * dt)
    }

    /// Per mode what the ramps to a changed machine's period (`shaper_ramps`) still add to the half period at the
    /// profile tick `x`: each step from where the period was to where it is now, going over by `smoothstep`.
    fn ramp_offset(&self, x: f64) -> Vec<f64> {
        let mut offset = vec![0.0; self.shaper.as_ref().map_or(0, Vec::len)];
        for (step, at, length) in &self.shaper_ramps {
            let left = 1.0 - smoothstep((x - at) / length);
            offset.iter_mut().zip(step).for_each(|(o, s)| *o += s * left);
        }
        offset
    }

    /// Notes the motion that makes the setpoints of the profile tick `tick` (`plans`), unless it is the one before.
    fn record_plan(&mut self, tick: u64, motion: &Motion, machine: &dyn Machine) {
        if self.shaper.is_none() {
            self.plan_changed_at = None;
            return;
        }
        let pass = self.pass.as_ref().filter(|_| matches!(motion, Motion::Conveyor));
        let key = plan_key(motion, pass, tick);
        let from = self.plan_changed_at.take();
        if from.is_none() && self.plans.back().is_some_and(|plan| plan.key == key) {
            return;
        }
        let mut motion = motion.clone();
        // Its start as it takes it (`profile_tick`).
        match &mut motion {
            Motion::Moving { start_tick, .. }
            | Motion::Cam { start_tick, .. }
            | Motion::Trajectory { start_tick, .. }
                if *start_tick == u64::MAX =>
            {
                *start_tick = tick
            }
            Motion::Following { start_tick, .. } | Motion::Halting { start_tick, .. } if *start_tick == u64::MAX => {
                *start_tick = tick - 1
            }
            _ => {}
        }
        let pass = pass.map(|pass| (pass.clone(), self.clock));
        let cam = KeptCurve(if matches!(motion, Motion::Cam { .. }) { machine.cam_curve() } else { None });
        self.plans.push_back(Plan { first: tick, key, motion, pass, cam, from });
    }

    /// The plan as `plan` makes it at the profile tick `x`, also between two cycles: position, speed and acceleration,
    /// worked out as the profile does at a cycle (`profile_tick`). `None` where the plan stands where the cycle before
    /// left it (idle, a dwell, waiting for a part).
    fn plan_at(&self, machine: &dyn Machine, plan: &Plan, x: f64) -> Option<[Vec<f64>; 3]> {
        let dt = self.profile_cycle;
        let n = self.set_pos.len();
        let (mut pos, mut vel, mut acc) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
        let since = |start: u64| (x - start as f64) * dt;
        match &plan.motion {
            Motion::Idle | Motion::Dwell { .. } => return None,
            Motion::Moving { mv, start_tick, tail } => {
                let old = tail.as_ref().map(|(old, start)| (old, since(*start)));
                move_at(mv, since(*start_tick), old, [&mut pos, &mut vel, &mut acc]);
            }
            Motion::Conveyor => {
                // The simulated time runs with the profile cycles.
                let (pass, clock) = plan.pass.as_ref()?;
                let t = clock + since(plan.first) - pass.start;
                if t < 0.0 {
                    return None;
                }
                pass.at(t, &mut pos, &mut vel, &mut acc);
                if t >= pass.duration() {
                    vel.fill(0.0);
                    acc.fill(0.0);
                }
            }
            Motion::Cam { start_tick, stop, from, offset, from_speed, speed, ramp, .. } => {
                let t = since(*start_tick);
                let run = crate::cam::master_change(*from_speed, *speed, *ramp, t, *stop);
                // The cam as it was when the plan was noted, also after a config has changed it.
                let curve = |m: f64, cam: &mut [[f64; 4]]| match &plan.cam.0 {
                    Some(curve) => curve(m, cam),
                    None => machine.cam_at(m, cam),
                };
                cam_slaves(&curve, from + run[0], run, offset, [&mut pos, &mut vel, &mut acc]);
                if stop.is_some_and(|s| t >= s.max(*ramp) + ramp) {
                    vel.fill(0.0);
                    acc.fill(0.0);
                }
            }
            Motion::Trajectory { points, speeds, accelerations, start_tick } => {
                let t = since(*start_tick);
                path_at(points, speeds, accelerations.as_deref(), t, dt, [&mut pos, &mut vel, &mut acc]);
            }
            Motion::Following { from, to, start_tick } => {
                let ticks = self.setpoint_ticks as f64;
                let done = ((x - *start_tick as f64) / ticks).clamp(0.0, 1.0);
                line_at(from, to, done, ticks * dt, [&mut pos, &mut vel, &mut acc]);
            }
            Motion::Halting { from, vel: speed, time, start_tick } => {
                halt(from, speed, *time, since(*start_tick).clamp(0.0, *time), [&mut pos, &mut vel, &mut acc]);
            }
        }
        Some([pos, vel, acc])
    }

    /// Input shaping: on each axis the setpoints become a weighted sum of copies of the plan at the delays of its
    /// shaper (`shaper::impulses`): ZVD three copies, now, half a period and a whole period ago, weighted 1 : 2K : K^2;
    /// ZV, ZVDD, EI and the two-hump EI likewise with their own (`profile.shaper_type`). The vibration each copy starts
    /// is cancelled by the others, also when the resonance is some percent off the frequency the shaper was set to.
    /// Every further mode's shaper is convolved with it (the second shaper, `profile.shaper_modes`): copies at the sums
    /// of the delays with the products of the weights, so that each mode's vibration is cancelled. An axis takes the
    /// modes that act on it (`profile.shaper_axes`, a mode's `axes`). A copy between two cycles is the plan itself
    /// there (`plan_at`), not an interpolation of the cycles around it.
    ///
    /// Following the pose, the half period is that of the resonance the machine reports where the plan
    /// stands, and the second shaper's that of the machine's second mode there. The weights always add up to one, so
    /// once the plan has stood still for a whole period the setpoint is the plan exactly, whatever the period did on the
    /// way. Where the machine itself changes its modes (the gripper takes its payload), the period does not step with
    /// it: a step in the delays would move the copies to other points of the plan at once, a step in the setpoint
    /// wherever the plan has not stood still for the whole period. The step goes over on a quintic smoothstep instead,
    /// long enough that no delay changes faster than `SHAPER_RAMP_RATE` of the time passing.
    fn shape(&mut self, machine: &dyn Machine) {
        // The shaped position is x_s(t) = sum w_i x(t - D_i(t)) / sum w_i (D_i the impulses' delays in seconds, which
        // change with the period). Its exact rates are x_s' = sum w_i v (1 - D_i') / W and x_s'' = sum w_i (a (1 - D_i')^2
        // - v D_i'') / W, v and a the plan's D_i ago. With steady delays this is the plain weighted sum of the plan's rates.
        let Some(target) = self.mode_halves(machine, &self.raw_pos) else {
            self.history.clear();
            self.plans.clear();
            self.shaper_reach = 0;
            (self.shaper_ramps, self.shaper_targets) = (Vec::new(), None);
            return;
        };
        let following = self.shaper.iter().flatten().any(|m| m.follows.is_some());
        let now = self.tick as f64;
        // The machine itself changed since the last cycle where its modes at the last cycle's pose did: from that cycle
        // on, the half periods go over from where they were. (At a pose that stands, the modes now are `target`.)
        if following
            && let (Some(before), Some(past)) = (self.shaper_targets.take(), self.history.back())
            && let Some(after) =
                if past[0] == self.raw_pos { Some(target.clone()) } else { self.mode_halves(machine, &past[0]) }
            && after.len() == before.len()
            && before != after
        {
            let step: Vec<f64> = before.iter().zip(&after).map(|(b, a)| b - a).collect();
            // The longest delay's change bounds every delay's; the smoothstep's steepest slope is 1.875.
            let reach = span(&self.trains(&step.iter().map(|s| s.abs()).collect::<Vec<_>>()));
            self.shaper_ramps.push((step, now - 1.0, 1.875 * reach / SHAPER_RAMP_RATE));
        }
        self.shaper_ramps.retain(|(_, at, length)| now - 1.0 - at < *length);
        let ramped = |halves: Vec<f64>, x: f64| -> Vec<f64> {
            halves.iter().zip(self.ramp_offset(x)).map(|(h, e)| h + e).collect()
        };
        let trains = self.trains(&ramped(target.clone(), now));
        // D_i' and D_i'' where the period follows the pose: the delays (cycles) with the plan one cycle ahead and one
        // behind on its own Taylor expansion and the ramps there, central differences: D' = (d+ - d-) / 2,
        // D'' = (d+ - 2 d + d-) / T (1/s).
        let cycle = self.profile_cycle;
        let zeros = || trains.iter().map(|train| [vec![0.0; train.len()], vec![0.0; train.len()]]).collect();
        let changes: Vec<[Vec<f64>; 2]> = if !following {
            zeros()
        } else {
            // A plan at rest is there a cycle ahead and behind too.
            let resting = self.raw_vel.iter().chain(&self.raw_acc).all(|r| *r == 0.0);
            let off = |sign: f64| -> Option<Vec<Vec<(f64, f64)>>> {
                if resting {
                    return Some(self.trains(&ramped(target.clone(), now + sign)));
                }
                let pose: Vec<f64> = (self.raw_pos.iter().zip(&self.raw_vel).zip(&self.raw_acc))
                    .map(|((x, v), a)| x + sign * v * cycle + 0.5 * a * cycle * cycle)
                    .collect();
                Some(self.trains(&ramped(self.mode_halves(machine, &pose)?, now + sign)))
            };
            match (off(1.0), off(-1.0)) {
                (Some(ahead), Some(behind)) => (trains.iter().zip(ahead.iter().zip(&behind)))
                    .map(|(now, (ahead, behind))| {
                        if ahead.len() != now.len() || behind.len() != now.len() {
                            return [vec![0.0; now.len()], vec![0.0; now.len()]];
                        }
                        let d = |k: usize| (ahead[k].0, now[k].0, behind[k].0);
                        let rate = (0..now.len()).map(|k| (d(k).0 - d(k).2) / 2.0).collect();
                        [rate, (0..now.len()).map(|k| (d(k).0 - 2.0 * d(k).1 + d(k).2) / cycle).collect()]
                    })
                    .collect(),
                _ => zeros(),
            }
        };
        if following {
            self.shaper_targets = Some(target);
        }
        self.history.push_back([self.raw_pos.clone(), self.raw_vel.clone(), self.raw_acc.clone()]);
        // The store keeps the longest span met so far: a period that grows again finds its past.
        self.shaper_reach = self.shaper_reach.max(span(&trains).ceil() as usize + 2);
        while self.history.len() > self.shaper_reach {
            self.history.pop_front();
        }
        let newest = self.history.len() - 1;
        // The motions of the stored cycles but the oldest, which is read only as stored.
        let oldest = self.tick - newest as u64;
        while self.plans.len() > 1
            && self.plans[1].first <= oldest + 1
            && self.plans[1].from.is_none_or(|from| from <= oldest as f64)
        {
            self.plans.pop_front();
        }
        // The plan `delay` cycles ago: at a whole cycle as stored, between two the plan itself there; before the store
        // reaches back that far, its oldest.
        let ago = |delay: f64| -> [Vec<f64>; 3] {
            let whole = delay.floor() as usize;
            if whole >= newest {
                return self.history[0].clone();
            }
            let owner = newest - whole;
            if delay == delay.floor() {
                return self.history[owner].clone();
            }
            let (tick, x) = (self.tick - whole as u64, now - delay);
            let plan =
                self.plans.iter().rev().find(|plan| plan.first <= tick && plan.from.is_none_or(|from| from <= x));
            plan.and_then(|plan| self.plan_at(machine, plan, x)).unwrap_or_else(|| {
                let n = self.set_pos.len();
                [self.history[owner - 1][0].clone(), vec![0.0; n], vec![0.0; n]]
            })
        };
        // Each delay read once: the axes that share their modes share their delays.
        let mut read: Vec<(f64, [Vec<f64>; 3])> = Vec::new();
        for (delay, _) in trains.iter().flatten() {
            if !read.iter().any(|(d, _)| d == delay) {
                read.push((*delay, ago(*delay)));
            }
        }
        let at = |delay: f64| &read.iter().find(|(d, _)| *d == delay).expect("every delay is read").1;
        let shaped = |which: usize, axis: usize| {
            let train = &trains[axis];
            let [rate, second] = &changes[axis];
            // The plan's position, speed and acceleration `delay` ago, and what the delay's own change does to each.
            let term = |k: usize, delay: f64| match which {
                0 => at(delay)[0][axis],
                1 => at(delay)[1][axis] * (1.0 - rate[k]),
                _ => at(delay)[2][axis] * (1.0 - rate[k]).powi(2) - at(delay)[1][axis] * second[k],
            };
            train.iter().enumerate().map(|(k, (delay, weight))| weight * term(k, *delay)).sum::<f64>()
                / train.iter().map(|(_, weight)| weight).sum::<f64>()
        };
        let axes = self.set_pos.len();
        let (pos, vel, acc): (Vec<f64>, Vec<f64>, Vec<f64>) = (
            (0..axes).map(|a| shaped(0, a)).collect(),
            (0..axes).map(|a| shaped(1, a)).collect(),
            (0..axes).map(|a| shaped(2, a)).collect(),
        );
        (self.set_pos, self.set_vel, self.set_acc) = (pos, vel, acc);
    }

    /// Corner blending: the next cycle segment, if it should start now, while `current` still has
    /// time `t` behind it and its braking ahead. The corner is rounded instead of stopped at.
    ///
    /// Only where the machine does not have to stop anyway (no dwell, no gripper action), where no
    /// axis reverses, and only while both moves are on their ramps, so the speeds add up to no
    /// more than either cruise speed.
    fn blend_into_next(
        &mut self,
        current: &Move,
        t: f64,
        blending_already: bool,
        machine: &dyn Machine,
    ) -> Option<Move> {
        let i = self.cycle?;
        let segment = &self.segments[i];
        if self.blend_time <= 0.0
            || self.stopping
            || blending_already
            || segment.dwell_ticks > 0
            || segment.payload != self.payload_attached
        {
            return None;
        }
        let next_index = (i + 1) % self.segments.len();
        if self.segments[next_index].path.is_some() {
            return None;
        }
        let next = Move::with_law(
            &segment.target,
            &self.segments[next_index].target,
            machine.axes(),
            &self.segments[next_index].law,
        );
        if current.delta().iter().zip(next.delta()).any(|(a, b)| a * b < 0.0) || next.duration() == 0.0 {
            return None;
        }
        let lead = self.blend_time.min(current.brake()).min(next.ramp());
        if t < current.duration() - lead {
            return None;
        }
        self.cycle = Some(next_index);
        self.passes += (next_index == 0) as u64;
        Some(next)
    }

    /// Cycle segment `i` has arrived at tick `tick`: its valve commands go out, and its dwell starts, or waits for its
    /// end-position sensors first (a dwell until `u64::MAX`, see `Motion::Dwell`).
    fn arrived(&mut self, i: usize, tick: u64) -> Motion {
        for &(valve, command) in &self.segments[i].valves {
            if let Some(v) = self.valves.get_mut(valve) {
                *v = command;
            }
        }
        // The dwell as `arrive` set it: the segment's, or the gripper's time if longer.
        let until_tick = if self.segments[i].wait.is_empty() { tick + self.dwell_ticks } else { u64::MAX };
        Motion::Dwell { until_tick }
    }

    fn next_cycle_segment(&mut self, machine: &dyn Machine) {
        let Some(last) = self.cycle else { return };
        if self.stopping {
            self.cycle = None;
            self.stopping = false;
            return;
        }
        if let Some(CamMaster { speed, ramp_s: ramp, .. }) = self.cam {
            // The cam starts from where its master stands, the slaves from where they stand: the offset keeps them there.
            let mut cam = vec![[0.0; 4]; self.set_pos.len()];
            machine.cam_at(self.cam_master, &mut cam);
            let offset = self.set_pos.iter().zip(&cam).map(|(p, c)| p - c[0]).collect();
            self.cycle = Some(0);
            self.passes += 1;
            self.motion = Motion::Cam {
                start_tick: u64::MAX,
                stop: None,
                from: self.cam_master,
                offset,
                next_pass: f64::NAN,
                from_speed: 0.0,
                speed,
                ramp,
            };
            return;
        }
        if self.conveyor.is_some() {
            // Planned for the shaped setpoint: the part each axis's mean shaper delay ahead, and its grip and release
            // longer by the shaper's span (see `Conveyor::plan`).
            let shaping = self.shaper_delays(machine, &self.set_pos);
            let reachable = |pose: &[f64]| in_range(machine, pose);
            let Some(conveyor) = &mut self.conveyor else { return };
            self.pass = conveyor.plan(machine.axes(), self.clock, &self.set_pos, &reachable, &shaping);
            if self.pass.is_some() {
                self.passes += 1;
                self.motion = Motion::Conveyor;
            } else {
                // No part can be picked: the cycle ends.
                self.cycle = None;
            }
            return;
        }
        let next = (last + 1) % self.segments.len();
        self.cycle = Some(next);
        self.passes += (next == 0) as u64;
        // A path through via points, planned from where the plan stands; were it refused from there (the first segment,
        // from the start pose), the straight move. Timed as fast as the motors allow, the straight way too; refused, as
        // the axis limits time it.
        let segment = &self.segments[next];
        let straight = [Piece::Line(segment.target.clone())];
        let tolerance =
            path::Tolerance { corner: segment.corner.unwrap_or(self.path_tolerance.corner), ..self.path_tolerance };
        let time_optimal = (self.cycle_time_optimal && !at_target(&self.set_pos, &segment.target))
            .then(|| {
                let pieces = segment.path.as_deref().unwrap_or(&straight);
                self.plan_time_optimal_from(machine, &self.set_pos, pieces, tolerance)
            })
            .and_then(|planned| planned.ok());
        let planned = time_optimal.or_else(|| {
            self.segments[next].path.as_ref().and_then(|pieces| {
                let (cycle, slack) = (self.profile_cycle, machine.setpoint_slack());
                let spin = self.segments[next].law.spin;
                path::plan_within(machine.axes(), spin, &self.set_pos, pieces, &[], cycle, tolerance, slack).ok()
            })
        });
        self.motion = match planned {
            Some(points) if points.iter().all(|(_, pose)| near_start(machine, &self.set_pos, pose)) => {
                trajectory(points, self.spline_c2)
            }
            _ => Motion::Moving {
                mv: Move::with_law(
                    &self.set_pos,
                    &self.segments[next].target,
                    machine.axes(),
                    &self.segments[next].law,
                ),
                start_tick: u64::MAX,
                tail: None,
            },
        };
    }
}

/// A path given as poses at times as the controller runs it: a cubic through them, at rest at both ends. With `c2`
/// the cubic spline whose acceleration is continuous at every point (`c2_speeds`); without, the speed at each point is
/// the slope between its neighbours (of the four around it where they are evenly spaced), and the acceleration steps at
/// each point.
fn trajectory(points: Vec<(f64, Vec<f64>)>, c2: bool) -> Motion {
    let speeds = if c2 { c2_speeds(&points) } else { neighbour_speeds(&points) };
    Motion::Trajectory { points, speeds, accelerations: None, start_tick: u64::MAX }
}

/// The speeds at the points of the Hermite cubics through them that make one C2 spline, clamped at rest at both ends:
/// with h the spacing before and h' after point i, the acceleration of the cubic before (6 (p_i-1 - p_i) / h^2 +
/// (2 m_i-1 + 4 m_i) / h) equals that of the cubic after (6 (p_i+1 - p_i) / h'^2 - (4 m_i + 2 m_i+1) / h'), which is
/// m_i-1 / h + 2 m_i (1/h + 1/h') + m_i+1 / h' = 3 ((p_i - p_i-1) / h^2 + (p_i+1 - p_i) / h'^2) for every inner point:
/// a tridiagonal system, diagonally dominant, solved by elimination (Thomas).
pub(crate) fn c2_speeds(points: &[(f64, Vec<f64>)]) -> Vec<Vec<f64>> {
    let (n, axes) = (points.len(), points[0].1.len());
    let mut speeds = vec![vec![0.0; axes]; n];
    if n < 3 {
        return speeds;
    }
    let h = |i: usize| points[i + 1].0 - points[i].0;
    for a in 0..axes {
        let p = |i: usize| points[i].1[a];
        // Forward elimination over the inner points 1 ..= n - 2: c' and d' of the Thomas algorithm.
        let (mut c, mut d) = (vec![0.0; n], vec![0.0; n]);
        for i in 1..n - 1 {
            let (lower, upper) = (1.0 / h(i - 1), 1.0 / h(i));
            let diagonal = 2.0 * (lower + upper);
            let rhs = 3.0 * ((p(i) - p(i - 1)) * lower * lower + (p(i + 1) - p(i)) * upper * upper);
            // Point 0 is at rest: no lower term for i = 1.
            let (l, c_prev, d_prev) = if i == 1 { (0.0, 0.0, 0.0) } else { (lower, c[i - 1], d[i - 1]) };
            let pivot = diagonal - l * c_prev;
            c[i] = upper / pivot;
            d[i] = (rhs - l * d_prev) / pivot;
        }
        // Back substitution; the last point is at rest.
        let mut next = 0.0;
        for i in (1..n - 1).rev() {
            next = d[i] - c[i] * next;
            speeds[i][a] = next;
        }
    }
    speeds
}

/// The speeds as before 14 November 2026 (`profile.trajectory_spline = "hermite"`): at each inner point the slope
/// between its neighbours, of the five points around it where they are evenly spaced.
fn neighbour_speeds(points: &[(f64, Vec<f64>)]) -> Vec<Vec<f64>> {
    let axes = points[0].1.len();
    let mut speeds = vec![vec![0.0; axes]; points.len()];
    let n = points.len();
    for i in 1..n - 1 {
        // Evenly spaced on both sides (rows of a trace, a plan's profile cycles): the slope of the five points around it,
        // exact for a cubic, so that the acceleration at each point is the curve's own. The slope of the two neighbours
        // is off by jerk x spacing^2 / 6, which put the acceleration off by jerk x spacing.
        let even = i >= 2 && i + 2 < n && {
            let h = points[i + 1].0 - points[i].0;
            (1..=4).all(|k| ((points[i + k - 2].0 - points[i + k - 3].0) - h).abs() <= 1e-6 * h)
        };
        for a in 0..axes {
            speeds[i][a] = if even {
                let p = |k: usize| points[k].1[a];
                (p(i - 2) - 8.0 * p(i - 1) + 8.0 * p(i + 1) - p(i + 2)) / (3.0 * (points[i + 2].0 - points[i - 2].0))
            } else {
                (points[i + 1].1[a] - points[i - 1].1[a]) / (points[i + 1].0 - points[i - 1].0)
            };
        }
    }
    speeds
}

/// Position, speed and acceleration of axis `a` at time `t` on the cubic from point i to point i + 1 with the speeds
/// there (Hermite interpolation).
pub(crate) fn hermite(points: &[(f64, Vec<f64>)], speeds: &[Vec<f64>], i: usize, a: usize, t: f64) -> [f64; 3] {
    let h = points[i + 1].0 - points[i].0;
    let s = (t - points[i].0) / h;
    let basis = [
        [
            2.0 * s * s * s - 3.0 * s * s + 1.0,
            s * s * s - 2.0 * s * s + s,
            -2.0 * s * s * s + 3.0 * s * s,
            s * s * s - s * s,
        ],
        [6.0 * s * s - 6.0 * s, 3.0 * s * s - 4.0 * s + 1.0, -6.0 * s * s + 6.0 * s, 3.0 * s * s - 2.0 * s],
        [12.0 * s - 6.0, 6.0 * s - 4.0, -12.0 * s + 6.0, 6.0 * s - 2.0],
    ];
    let at = |b: [f64; 4]| {
        b[0] * points[i].1[a] + b[1] * h * speeds[i][a] + b[2] * points[i + 1].1[a] + b[3] * h * speeds[i + 1][a]
    };
    [at(basis[0]), at(basis[1]) / h, at(basis[2]) / (h * h)]
}

/// The fieldbus queue (frames of setpoints newest last, each with the cycles it takes to arrive) for a profile cycle
/// `ratio` of the old ones to a new one: at age `b` new cycles stands what the old queue had at age `b / ratio` old
/// cycles, between two frames in proportion (as the shaper's history), and the delay is scaled with it, so every frame
/// arrives at the same time in seconds. Interpolated, not repeated: the drives take every frame as a fresh setpoint.
fn resample_fieldbus(
    old: &std::collections::VecDeque<(usize, Vec<[f64; 5]>)>,
    ratio: f64,
) -> std::collections::VecDeque<(usize, Vec<[f64; 5]>)> {
    let Some(newest) = old.len().checked_sub(1) else { return Default::default() };
    (0..=(newest as f64 * ratio).floor() as usize)
        .rev()
        .map(|back| {
            let (whole, part) = ((back as f64 / ratio).floor() as usize, (back as f64 / ratio).fract());
            let (a, b) = (&old[newest - whole.min(newest)], &old[newest - (whole + 1).min(newest)]);
            let stamp = ((a.0 as f64 * (1.0 - part) + b.0 as f64 * part) * ratio).round() as usize;
            let frames = a.1.iter().zip(&b.1).map(|(a, b)| std::array::from_fn(|k| a[k] * (1.0 - part) + b[k] * part));
            (stamp, frames.collect())
        })
        .collect()
}

/// A move `t` s after its start; with corner blending the previous move's remaining way to the corner on top, `old` with
/// its own time.
fn move_at(mv: &Move, t: f64, old: Option<(&Move, f64)>, [pos, vel, acc]: [&mut Vec<f64>; 3]) {
    mv.at(t, pos, vel, acc);
    if let Some((old, t_old)) = old {
        let n = pos.len();
        let (mut p, mut v, mut a, mut end) = (vec![0.0; n], vec![0.0; n], vec![0.0; n], vec![0.0; n]);
        old.at(t_old, &mut p, &mut v, &mut a);
        old.at(f64::INFINITY, &mut end, &mut vec![0.0; n], &mut vec![0.0; n]);
        for i in 0..n {
            pos[i] += p[i] - end[i];
            vel[i] += v[i];
            acc[i] += a[i];
        }
    }
}

/// The ramp to rest from `from` at the speeds `vel` over `time` s, `t` s into it.
fn halt(from: &[f64], vel: &[f64], time: f64, t: f64, [pos, speed, acc]: [&mut Vec<f64>; 3]) {
    for i in 0..from.len() {
        pos[i] = from[i] + vel[i] * (t - 0.5 * t * t / time);
        speed[i] = vel[i] * (1.0 - t / time);
        acc[i] = if t < time { -vel[i] / time } else { 0.0 };
    }
}

/// The cam's slaves (`curve`, as `Machine::cam_at`) with the master at `master` and its speed and acceleration in `run`,
/// each from its `offset`.
fn cam_slaves(
    curve: &dyn Fn(f64, &mut [[f64; 4]]),
    master: f64,
    run: [f64; 3],
    offset: &[f64],
    [pos, vel, acc]: [&mut Vec<f64>; 3],
) {
    let mut cam = vec![[0.0; 4]; pos.len()];
    curve(master, &mut cam);
    for (i, c) in cam.iter().enumerate() {
        let [p, v, a] = crate::cam::slave_motion(*c, run);
        (pos[i], vel[i], acc[i]) = (p + offset[i], v, a);
    }
}

/// A trajectory through `points` `t` s after its start (profile cycle `dt`), at its end its last point at rest.
fn path_at(
    points: &[(f64, Vec<f64>)],
    speeds: &[Vec<f64>],
    accelerations: Option<&[Vec<f64>]>,
    t: f64,
    dt: f64,
    [pos, vel, acc]: [&mut Vec<f64>; 3],
) {
    let last = points.len() - 1;
    if t >= points[last].0 {
        pos.copy_from_slice(&points[last].1);
        vel.fill(0.0);
        acc.fill(0.0);
        return;
    }
    // A point a rounding error after `t` counts as at `t`: at a point the cubic that starts there is taken, not by the
    // rounding of the two times the end of the one before, whose acceleration differs. Before the start, the start.
    let i = points.partition_point(|(time, _)| *time <= t + 1e-6 * dt).saturating_sub(1).min(last - 1);
    let t = t.max(points[0].0);
    for a in 0..pos.len() {
        [pos[a], vel[a], acc[a]] = match accelerations {
            Some(accelerations) => quintic(points, speeds, accelerations, i, a, t),
            None => hermite(points, speeds, i, a, t),
        };
    }
}

/// A setpoint from outside approached on a straight line, `done` of the way from `from` to `to` in `time` s.
fn line_at(from: &[f64], to: &[f64], done: f64, time: f64, [pos, vel, acc]: [&mut Vec<f64>; 3]) {
    for i in 0..to.len() {
        // At the end the target itself, not from + (to - from), which can be a rounding off it.
        pos[i] = if done < 1.0 { from[i] + (to[i] - from[i]) * done } else { to[i] };
        // The speed of the cycle that ends here, also on arrival: in a stream the next setpoint is on its way, and a
        // speed of zero for this one cycle would be a dip in every interval.
        vel[i] = (to[i] - from[i]) / time;
    }
    acc.fill(0.0);
}

/// What tells a motion that makes the setpoints from one taken over by another (`Plan`): its kind and its start ticks
/// (`u64::MAX`, not yet taken, counts as `tick`), a cam's stop, a conveyor's pass by its start. Idle and a dwell are
/// the same: the plan stands.
fn plan_key(motion: &Motion, pass: Option<&Pass>, tick: u64) -> [u64; 4] {
    let at = |start: u64, before: u64| if start == u64::MAX { tick - before } else { start };
    match motion {
        Motion::Idle | Motion::Dwell { .. } => [0, 0, 0, 0],
        Motion::Moving { start_tick, tail, .. } => {
            [1, at(*start_tick, 0), tail.as_ref().map_or(u64::MAX, |(_, start)| *start), 0]
        }
        Motion::Following { start_tick, .. } => [2, at(*start_tick, 1), 0, 0],
        Motion::Conveyor => [3, pass.map_or(u64::MAX, |pass| pass.start.to_bits()), 0, 0],
        Motion::Cam { start_tick, stop, .. } => [4, at(*start_tick, 0), stop.map_or(u64::MAX, f64::to_bits), 0],
        Motion::Trajectory { start_tick, .. } => [5, at(*start_tick, 0), 0, 0],
        Motion::Halting { start_tick, .. } => [6, at(*start_tick, 1), 0, 0],
    }
}

/// The longest delay of per-axis impulse trains (`Controller::trains`), cycles.
fn span(trains: &[Vec<(f64, f64)>]) -> f64 {
    trains.iter().flatten().fold(0.0, |m: f64, (t, _)| m.max(*t))
}

/// The quintic smoothstep 10 u^3 - 15 u^4 + 6 u^5 from 0 to 1, held at its ends: no step in its slope or its
/// curvature, the steepest slope 1.875 at u = 1/2.
fn smoothstep(u: f64) -> f64 {
    let u = u.clamp(0.0, 1.0);
    u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)
}

/// Position, speed and acceleration of axis `a` at time `t` on the quintic from point i to point i + 1 with the speeds
/// and accelerations there (Hermite interpolation of the fifth degree): at the points exactly those given.
fn quintic(
    points: &[(f64, Vec<f64>)],
    speeds: &[Vec<f64>],
    accelerations: &[Vec<f64>],
    i: usize,
    a: usize,
    t: f64,
) -> [f64; 3] {
    let h = points[i + 1].0 - points[i].0;
    let s = (t - points[i].0) / h;
    let (s2, s3, s4, s5) = (s * s, s * s * s, s.powi(4), s.powi(5));
    // The basis functions of p0, h v0, h² a0, h² a1, h v1, p1 and their first and second derivatives in s.
    let basis = [
        [
            1.0 - 10.0 * s3 + 15.0 * s4 - 6.0 * s5,
            s - 6.0 * s3 + 8.0 * s4 - 3.0 * s5,
            0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5,
            0.5 * s3 - s4 + 0.5 * s5,
            -4.0 * s3 + 7.0 * s4 - 3.0 * s5,
            10.0 * s3 - 15.0 * s4 + 6.0 * s5,
        ],
        [
            -30.0 * s2 + 60.0 * s3 - 30.0 * s4,
            1.0 - 18.0 * s2 + 32.0 * s3 - 15.0 * s4,
            s - 4.5 * s2 + 6.0 * s3 - 2.5 * s4,
            1.5 * s2 - 4.0 * s3 + 2.5 * s4,
            -12.0 * s2 + 28.0 * s3 - 15.0 * s4,
            30.0 * s2 - 60.0 * s3 + 30.0 * s4,
        ],
        [
            -60.0 * s + 180.0 * s2 - 120.0 * s3,
            -36.0 * s + 96.0 * s2 - 60.0 * s3,
            1.0 - 9.0 * s + 18.0 * s2 - 10.0 * s3,
            3.0 * s - 12.0 * s2 + 10.0 * s3,
            -24.0 * s + 84.0 * s2 - 60.0 * s3,
            60.0 * s - 180.0 * s2 + 120.0 * s3,
        ],
    ];
    let given = [
        points[i].1[a],
        h * speeds[i][a],
        h * h * accelerations[i][a],
        h * h * accelerations[i + 1][a],
        h * speeds[i + 1][a],
        points[i + 1].1[a],
    ];
    let at = |b: &[f64; 6]| b.iter().zip(&given).map(|(b, g)| b * g).sum::<f64>();
    [at(&basis[0]), at(&basis[1]) / h, at(&basis[2]) / (h * h)]
}

/// A string of the config.
fn text<'a>(cfg: &'a Config, path: &str) -> Result<&'a str> {
    cfg.get(path)?.as_str().ok_or_else(|| ConfigError::Type(path.into()))
}

/// The axes a list of axis ids names (`profile.shaper_axes`, a mode's `axes`): every axis for an empty list.
fn named_axes(machine: &dyn Machine, ids: Option<&toml::Value>, path: &str) -> Result<Vec<bool>> {
    let bad = || ConfigError::Invalid(path.into());
    let axes = machine.axes();
    let ids: Vec<&str> = match ids {
        None => Vec::new(),
        Some(list) => {
            list.as_array().ok_or_else(bad)?.iter().map(|v| v.as_str()).collect::<Option<_>>().ok_or_else(bad)?
        }
    };
    if ids.iter().any(|id| !axes.iter().any(|a| a.id == *id)) {
        return Err(bad());
    }
    Ok(axes.iter().map(|a| ids.is_empty() || ids.contains(&a.id)).collect())
}

/// The input shaper of a config (see `Controller::shape`), `None` with `effects.input_shaping` off: the first mode
/// (`profile.shaper_frequency_hz`, `shaper_damping`), the second (`shaper2_frequency_hz`, `shaper2_damping`; half period
/// 0 at 0 Hz), both of `profile.shaper_type` on `profile.shaper_axes`, following the pose with
/// `effects.shaper_follows_pose`; then each entry of `profile.shaper_modes` (`frequency_hz`, `damping`, and `type` and
/// `axes` where it has them), fixed. A value out of its range is refused also with shaping off.
fn shaper_modes(cfg: &Config, machine: &dyn Machine, profile_cycle: f64) -> Result<Option<Vec<ShaperMode>>> {
    let kind_of = |name: Option<&str>, path: &str| {
        name.and_then(shaper::Kind::from_name).ok_or_else(|| ConfigError::Invalid(path.into()))
    };
    let kind = kind_of(Some(text(cfg, "profile.shaper_type")?), "profile.shaper_type")?;
    let tolerance = cfg.positive("profile.shaper_ei_tolerance")?;
    if tolerance >= 1.0 {
        return Err(ConfigError::Invalid("profile.shaper_ei_tolerance".into()));
    }
    let axes = named_axes(machine, Some(cfg.get("profile.shaper_axes")?), "profile.shaper_axes")?;
    let (frequency, damping) =
        (cfg.positive("profile.shaper_frequency_hz")?, cfg.non_negative("profile.shaper_damping")?);
    if damping >= 1.0 {
        return Err(ConfigError::Invalid("profile.shaper_damping".into()));
    }
    let (frequency2, damping2) =
        (cfg.non_negative("profile.shaper2_frequency_hz")?, cfg.non_negative("profile.shaper2_damping")?);
    if damping2 >= 1.0 {
        return Err(ConfigError::Invalid("profile.shaper2_damping".into()));
    }
    let follows = cfg.bool("effects.shaper_follows_pose")?;
    // Half the damped period in profile cycles, per Hz.
    let per_hz = |damping: f64| 0.5 / (1.0 - damping * damping).sqrt() / profile_cycle;
    let mode = |kind, frequency: f64, damping: f64, axes: Vec<bool>, follow: Option<usize>| ShaperMode {
        impulses: shaper::impulses(kind, damping, tolerance),
        half: if frequency > 0.0 { per_hz(damping) / frequency } else { 0.0 },
        follows: follow.filter(|_| follows).map(|k| (k, per_hz(damping))),
        axes,
    };
    let mut modes =
        vec![mode(kind, frequency, damping, axes.clone(), Some(0)), mode(kind, frequency2, damping2, axes, Some(1))];
    for (i, entry) in cfg.array("profile.shaper_modes")?.iter().enumerate() {
        let path = format!("profile.shaper_modes[{i}]");
        let bad = || ConfigError::Invalid(path.clone());
        let number = |key: &str| entry.get(key).and_then(|v| v.as_float().or_else(|| v.as_integer().map(|i| i as f64)));
        let frequency = number("frequency_hz").filter(|f| f.is_finite() && *f > 0.0).ok_or_else(bad)?;
        let damping = number("damping").filter(|d| (0.0..1.0).contains(d)).ok_or_else(bad)?;
        let kind = match entry.get("type") {
            None => kind,
            Some(name) => kind_of(name.as_str(), &path)?,
        };
        modes.push(mode(kind, frequency, damping, named_axes(machine, entry.get("axes"), &path)?, None));
    }
    Ok(cfg.bool("effects.input_shaping")?.then_some(modes))
}

/// The motion law of a config's moves (`profile.shape`, `deceleration_ratio`, and `vertical_axis` with
/// `up_acceleration_ratio`: an axis id, "-" before it where up is its minus direction, "" for none). The ratios are
/// shares of the axes' limits, above 0 and at most 1.
fn motion_law(cfg: &Config, machine: &dyn Machine) -> Result<MotionLaw> {
    let shape =
        Shape::from_name(text(cfg, "profile.shape")?).ok_or_else(|| ConfigError::Invalid("profile.shape".into()))?;
    let share = |path: &str| {
        let v = cfg.positive(path)?;
        if v <= 1.0 { Ok(v) } else { Err(ConfigError::Invalid(path.into())) }
    };
    let (deceleration_ratio, up_ratio) =
        (share("profile.deceleration_ratio")?, share("profile.up_acceleration_ratio")?);
    let vertical = match text(cfg, "profile.vertical_axis")? {
        "" => None,
        name => {
            let (up, id) = name.strip_prefix('-').map_or((1.0, name), |id| (-1.0, id));
            let axis = machine.axes().iter().position(|a| a.id == id);
            Some((axis.ok_or_else(|| ConfigError::Invalid("profile.vertical_axis".into()))?, up, up_ratio))
        }
    };
    // The flange's own limits, where the machine has them as limits of the plan (an arm in tool space).
    let checks = machine.rating_checks();
    let limit = |name: &str| checks.iter().find(|c| c.check == name && c.signal.starts_with("plan.")).map(|c| c.limit);
    let spin = limit("tool-angular-speed").zip(limit("tool-angular-acceleration")).map(|(w, a)| [w, a]);
    Ok(MotionLaw { shape, deceleration_ratio, vertical, spin })
}

/// A segment that starts on its target (within 1 nm on every axis) has no way to time: it is a move of no length.
fn at_target(from: &[f64], target: &[f64]) -> bool {
    from.iter().zip(target).all(|(f, t)| (f - t).abs() < 1e-9)
}

pub(crate) fn in_range(machine: &dyn Machine, target: &[f64]) -> bool {
    near_range(machine, target, 0.0)
}

/// A pose a path from `start` may go to. The setpoint at rest is where the motors stand, up to
/// `Machine::setpoint_slack` from the tool (at an end of a stroke, beyond it), so a path may start that far outside
/// the range or the envelope, and no further: a pose within the slack of the start is judged with the slack; any other
/// within the range, except that on an axis whose start lies beyond an end it may stay as far beyond as the start is
/// (a path along that end), and within the envelope, with the slack only if the start is outside it. Until 13 November 2026 every pose had the slack,
/// which let a delta's path run 1.4 mm beyond its 0.6 m range where a move there was refused.
fn near_start(machine: &dyn Machine, start: &[f64], pose: &[f64]) -> bool {
    let slack = machine.setpoint_slack();
    if pose.len() == start.len() && pose.iter().zip(start).map(|(p, s)| (p - s) * (p - s)).sum::<f64>().sqrt() <= slack
    {
        return near_range(machine, pose, slack);
    }
    let axes = machine.axes();
    let beyond = |s: f64, a: &sim_core::AxisInfo| ((a.min - s).clamp(0.0, slack), (s - a.max).clamp(0.0, slack));
    pose.len() == axes.len()
        && start.len() == axes.len()
        && pose.iter().zip(start).zip(axes).all(|((t, s), a)| {
            t.is_finite() && *t >= a.min - beyond(*s, a).0 - ROUNDING && *t <= a.max + beyond(*s, a).1 + ROUNDING
        })
        && machine
            .unreachable(pose, ROUNDING + if machine.unreachable(start, 0.0).is_some() { slack } else { 0.0 })
            .is_none()
}

/// What a pose away from the start may be off the range by rounding alone: an arc along the rim of a cylindrical
/// range samples points a hair beyond it, and was refused with no allowance.
const ROUNDING: f64 = 1e-9;

/// `in_range`, with every axis range wider by `slack` at both ends, and the machine's own envelope too
/// (`Machine::unreachable`).
fn near_range(machine: &dyn Machine, target: &[f64], slack: f64) -> bool {
    let axes = machine.axes();
    target.len() == axes.len()
        && target.iter().zip(axes).all(|(t, a)| t.is_finite() && *t >= a.min - slack && *t <= a.max + slack)
        && machine.unreachable(target, slack).is_none()
}

/// A target that a move or a setpoint cannot go to, with the reason: outside an axis range, or where the machine says
/// it cannot reach (out of reach, outside its envelope, beyond a joint's stop), in its own words. Up to `ROUNDING`
/// beyond is rounding: a pose read back through the kinematics at an end of a stroke lies that far out (the planar
/// delta's tool at rest at x = 0.4 m reads 0.40000000000000013), and a move back there was refused.
fn refuse_target(machine: &dyn Machine, target: &[f64]) -> std::result::Result<(), CommandError> {
    let axes = machine.axes();
    if target.len() != axes.len()
        || !target.iter().zip(axes).all(|(t, a)| t.is_finite() && *t >= a.min - ROUNDING && *t <= a.max + ROUNDING)
    {
        return Err(CommandError { key: "error-target-out-of-range" });
    }
    machine.unreachable(target, ROUNDING).map_or(Ok(()), |key| Err(CommandError { key }))
}

/// With `Machine::cam`: per tool axis (a slave) the highest speed, acceleration and jerk the cam asks of it at the
/// master's steady speed (the cam's first three rates over the master times v, v^2 and v^3) over a period, at 4000 points; empty
/// without a cam. `check_limits` reports them against the axis limits, the verdicts judge them (`analysis`).
pub fn cam_peaks(machine: &dyn Machine) -> Vec<[f64; 3]> {
    let Some(master) = machine.cam() else { return Vec::new() };
    let axes = machine.axes().len();
    let n = 4000;
    let mut peak = vec![[0f64; 3]; axes];
    let mut cam = vec![[0.0; 4]; axes];
    for i in 0..=n {
        machine.cam_at(master.period * i as f64 / n as f64, &mut cam);
        for (p, c) in peak.iter_mut().zip(&cam) {
            for k in 0..3 {
                p[k] = p[k].max(c[k + 1].abs() * master.speed.powi(k as i32 + 1));
            }
        }
    }
    peak
}

/// An axis limit that asks more of a motor than the drive allows; `key` is a translation key.
#[derive(Debug, Clone, PartialEq)]
pub struct LimitViolation {
    pub key: &'static str,
    pub motor: usize,
    pub needed: f64,
    pub allowed: f64,
}

/// Checks that the axis limits do not ask more of a motor than its drive and motor allow (`drives`, each motor's own, in
/// the order of `Machine::motors`): the motor speed at the highest axis speeds against the motor's maximum speed
/// (`DriveParams::max_speed`), and the torque of the rigid machine at the highest axis accelerations against its
/// torque-speed curve at that speed (or the torque of the drive's maximum current, where lower). A diagonal move loads
/// one motor with both axes, so every combination of axis directions is tried, at the middle pose.
///
/// A warning about the axis limits, not a verdict: it takes every axis at its highest speed and acceleration at once,
/// which a cycle need not ask. So nothing is refused by it; the search simulates every candidate and the verdicts judge
/// what the cycle asks ("motor-peak" against the curve, "motor-speed" against the maximum speed).
pub fn check_limits(machine: &dyn Machine, drives: &[&DriveParams]) -> Vec<LimitViolation> {
    let (axes, motors) = (machine.axes(), machine.motors().len());
    let pose = sim_core::testkit::mid_pose(machine);
    let (mut speed, mut torque) = (vec![0f64; motors], vec![0f64; motors]);
    let (mut a, mut b) = (vec![0.0; motors], vec![0.0; motors]);
    for combo in 0..1u32 << axes.len() {
        let sign = |i: usize| if combo >> i & 1 == 1 { 1.0 } else { -1.0 };
        let h = 1e-3;
        let shifted = |s: f64| -> Vec<f64> {
            axes.iter().enumerate().map(|(i, ax)| pose[i] + s * h * sign(i) * ax.max_vel).collect()
        };
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
    // An electronic cam asks of its slaves what it asks at the master's speed, whatever the axis limits say.
    for (i, (p, axis)) in cam_peaks(machine).iter().zip(axes).enumerate() {
        let keys = ["limit-cam-speed", "limit-cam-acceleration", "limit-cam-jerk"];
        for (k, allowed) in [axis.max_vel, axis.max_acc, axis.max_jerk].into_iter().enumerate() {
            if p[k] > allowed * (1.0 + 1e-9) {
                out.push(LimitViolation { key: keys[k], motor: i.min(motors - 1), needed: p[k], allowed });
            }
        }
    }
    for (k, drive) in drives.iter().enumerate().take(motors) {
        if speed[k] > drive.max_speed {
            out.push(LimitViolation { key: "limit-motor-speed", motor: k, needed: speed[k], allowed: drive.max_speed });
        }
        // Accelerating at that speed: the curve there, driving.
        let curve = drive.torque_curve(speed[k].min(drive.max_speed)).1;
        let allowed = if drive.drive_max_current.is_finite() {
            curve.min(drive.torque_of(drive.drive_max_current))
        } else {
            curve
        };
        if torque[k] > allowed {
            out.push(LimitViolation { key: "limit-motor-torque", motor: k, needed: torque[k], allowed });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Acceleration of axis 0 just before and just after point i of a curve through `points` with `speeds`.
    fn around(points: &[(f64, Vec<f64>)], speeds: &[Vec<f64>], i: usize) -> (f64, f64) {
        (hermite(points, speeds, i - 1, 0, points[i].0)[2], hermite(points, speeds, i, 0, points[i].0)[2])
    }

    /// The fieldbus queue of four frames, oldest first setpoints 0, 1, 2, 3 and delays 3, 2, 1, 0 cycles, after the
    /// profile cycle halves (ratio 2): seven frames (3 * 2 + 1); newest first the new ages 0..6 read the old ages 0,
    /// 0.5, .., 3, so the setpoints are 3, 2.5, .., 0 and the delays (old 0, 0.5, .., 3 doubled) 0, 1, .., 6: every
    /// frame arrives at the same time in seconds, and none repeats its neighbour (the drives take each as fresh).
    #[test]
    fn the_fieldbus_queue_keeps_its_delay_in_seconds_across_a_cycle_change() {
        let queue: std::collections::VecDeque<(usize, Vec<[f64; 5]>)> =
            (0..4).map(|k| (3 - k, vec![[k as f64; 5]])).collect();
        let new = resample_fieldbus(&queue, 2.0);
        assert_eq!(new.iter().rev().map(|f| f.0).collect::<Vec<_>>(), [0, 1, 2, 3, 4, 5, 6]);
        let set: Vec<f64> = new.iter().rev().map(|f| f.1[0][0]).collect();
        assert_eq!(set, [3.0, 2.5, 2.0, 1.5, 1.0, 0.5, 0.0]);
        // The same ratio back: the original stamps and frames (new ages 0, 2, 4, 6).
        let back = resample_fieldbus(&new, 0.5);
        assert_eq!(back, queue);
    }

    /// The quintic of `Command::RatedTrajectory` is a polynomial of the fifth degree where the points' positions, speeds
    /// and accelerations are its own: p(t) = 1 - 2 t + 3 t^2 + t^3 - 4 t^4 + 0.5 t^5, points at 0.2 s and 0.7 s, by hand
    /// at 0.45 s (position, speed and acceleration of the polynomial), and the given values at both points.
    #[test]
    fn the_quintic_reproduces_a_fifth_degree_polynomial() {
        let c = [1.0, -2.0, 3.0, 1.0, -4.0, 0.5];
        let p = |t: f64, d: usize| -> f64 {
            (d..6).map(|k| c[k] * (k - d + 1..=k).map(|f| f as f64).product::<f64>() * t.powi((k - d) as i32)).sum()
        };
        let points = vec![(0.2, vec![p(0.2, 0)]), (0.7, vec![p(0.7, 0)])];
        let (speeds, accelerations) = (vec![vec![p(0.2, 1)], vec![p(0.7, 1)]], vec![vec![p(0.2, 2)], vec![p(0.7, 2)]]);
        for t in [0.2, 0.45, 0.7] {
            let got = quintic(&points, &speeds, &accelerations, 0, 0, t);
            for d in 0..3 {
                assert!((got[d] - p(t, d)).abs() < 1e-12, "at {t} s, derivative {d}: {} against {}", got[d], p(t, d));
            }
        }
    }

    /// The C2 spline through (0 s, 0 m), (1 s, 1 m) and (3 s, 1 m), at rest at both ends. Its one inner speed m solves
    /// m 2 (1/1 + 1/2) = 3 ((1 - 0) / 1^2 + (1 - 1) / 2^2): m = 1 m/s. The acceleration before the point,
    /// 6 (0 - 1) / 1 + (2 * 0 + 4 * 1) / 1 = -2 m/s^2, equals that after it, 6 (1 - 1) / 4 - (4 * 1 + 0) / 2 = -2 m/s^2.
    /// The slope of the neighbours (1 - 0) / 3 = 1/3 m/s gives -4.667 before and -0.667 after: a step of 4 m/s^2.
    #[test]
    fn the_c2_import_has_a_continuous_acceleration_at_a_knot() {
        let points = vec![(0.0, vec![0.0]), (1.0, vec![1.0]), (3.0, vec![1.0])];
        let c2 = c2_speeds(&points);
        assert!((c2[1][0] - 1.0).abs() < 1e-12 && c2[0][0] == 0.0 && c2[2][0] == 0.0, "{c2:?}");
        let (before, after) = around(&points, &c2, 1);
        assert!((before + 2.0).abs() < 1e-12 && (after + 2.0).abs() < 1e-12, "{before} {after}");
        let old = neighbour_speeds(&points);
        let (before, after) = around(&points, &old, 1);
        assert!((before + 14.0 / 3.0).abs() < 1e-12 && (after + 2.0 / 3.0).abs() < 1e-12, "{before} {after}");
        // Uneven spacing and two axes: continuous at every inner point, and through every point.
        let points: Vec<(f64, Vec<f64>)> = [(0.0, 0.0), (0.3, 0.2), (0.5, 0.25), (1.2, -0.1), (1.3, 0.0), (2.0, 0.4)]
            .iter()
            .map(|&(t, x)| (t, vec![x, 2.0 * x * x]))
            .collect();
        let c2 = c2_speeds(&points);
        for i in 1..points.len() - 1 {
            for a in 0..2 {
                let (before, after) =
                    (hermite(&points, &c2, i - 1, a, points[i].0)[2], hermite(&points, &c2, i, a, points[i].0)[2]);
                assert!((before - after).abs() < 1e-9 * (1.0 + before.abs()), "point {i}, axis {a}: {before} {after}");
                assert!((hermite(&points, &c2, i, a, points[i].0)[0] - points[i].1[a]).abs() < 1e-15);
            }
        }
    }
}
