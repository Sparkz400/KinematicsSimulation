//! Motion controller: plans in tool coordinates, transforms to motor setpoints, and runs one drive
//! per motor. It works on any machine through the machine interface.

use crate::conveyor::{Conveyor, Pass};
use crate::drive::{Drive, DriveParams, quantise, shared_dc_link};
use crate::path::{self, Piece};
use crate::profile::{MotionLaw, Move, Shape};
use crate::shaper;
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
    /// A path given as poses at times (seconds from its start, the first at 0 and at the present
    /// setpoint, the last at rest). The controller runs a smooth curve through them.
    Trajectory(Vec<(f64, Vec<f64>)>),
    /// Switch the drives off: no torque, and a move or cycle under way is dropped. Machines with holding
    /// brakes engage them. Until `Enable`, every other command is refused.
    Disable,
    /// Switch the drives on again, holding where the motors stand.
    Enable,
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
    /// With `via`: the path to the target, planned by `path::plan` where the segment starts (see `via_pieces`).
    path: Option<Vec<Piece>>,
    /// The motion law of its move: `profile.shape`, or the segment's own `shape`.
    law: MotionLaw,
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
    /// A cubic curve through the points with the given speeds at them (Hermite interpolation; with the speeds of
    /// `c2_speeds`, the cubic spline whose acceleration is continuous).
    Trajectory {
        points: Vec<(f64, Vec<f64>)>,
        speeds: Vec<Vec<f64>>,
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
    /// (`profile.torque_reserve`), the belt tensions it may need (`ratings.belt_min_tension_n`, `belt_max_tension_n` less
    /// `profile.tension_margin_n` on both; unbounded where the config has none) and the share of the room to them
    /// (`profile.tension_reserve`), each gearbox's output torque rating and the guides' load rating (`path::PlanLimits`).
    plan_limits: path::PlanLimits,
    /// How far a planned path may leave its lines and arcs for transition curves.
    path_tolerance: f64,
    segments: Vec<Segment>,
    motion: Motion,
    /// Index of the cycle segment under way, while the cycle runs.
    cycle: Option<usize>,
    /// How often the cycle has started its first segment.
    passes: u64,
    stopping: bool,
    enabled: bool,
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
    /// The half periods, one per mode, the shaper used in the last cycle, kept while the plan stands still.
    shaper_held: Option<Vec<f64>>,
    /// The motion law of moves (`profile.shape`, `deceleration_ratio`, `vertical_axis`, `up_acceleration_ratio`).
    law: MotionLaw,
    /// A trajectory's curve: the C2 cubic spline (`profile.trajectory_spline = "c2"`), or the Hermite cubic with the
    /// slopes of the neighbours ("hermite", as before 14 November 2026).
    spline_c2: bool,
    /// Fieldbus: profile cycles between computing the motor setpoints and the drives using them, and the
    /// setpoints on their way, oldest first, each with the cycles it takes: per motor position, speed,
    /// acceleration and torque feedforward.
    fieldbus_delay: usize,
    fieldbus: std::collections::VecDeque<(usize, Vec<[f64; 4]>)>,
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
            path_tolerance: 0.0,
            segments: Vec::new(),
            motion: Motion::Idle,
            cycle: None,
            passes: 0,
            stopping: false,
            enabled: true,
            payload_attached: true,
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
            shaper_held: None,
            law: MotionLaw::default(),
            spline_c2: true,
            fieldbus_delay: 0,
            fieldbus: Default::default(),
            fieldbus_jitter: 0,
            jitter: Default::default(),
            tick: 0,
            conveyor: None,
            pass: None,
            clock: 0.0,
            history: Default::default(),
            inertias: Vec::new(),
            load_encoder: None,
        };
        c.apply_config(cfg, machine)?;
        // Nothing moves yet: the shaper is in use from the first cycle on.
        c.shaper = c.pending_shaper.take().flatten();
        machine.motor_state(state, &mut c.motor_pos, &mut c.motor_vel);
        c.hold_here(machine);
        Ok(c)
    }

    /// Setpoints and drives at rest where the motors stand.
    fn hold_here(&mut self, machine: &dyn Machine) {
        let motors = self.motor_pos.len();
        // The setpoint starts where the motors are. With a stretched belt and a loaded gearbox that
        // is a few micrometres from the tool pose, and starting there avoids a jolt at the first step.
        machine.forward(&self.motor_pos, &mut self.set_pos);
        self.raw_pos.clone_from(&self.set_pos);
        for rate in [&mut self.set_vel, &mut self.set_acc, &mut self.raw_vel, &mut self.raw_acc] {
            rate.fill(0.0);
        }
        self.history.clear();
        self.fieldbus.clear();
        // Start holding the load: the torque that keeps the pose, and the feedforward's share of it.
        let (mut hold, mut ff) = (vec![0.0; motors], vec![0.0; motors]);
        machine.equilibrium(&self.set_pos, &mut Vec::new(), &mut hold);
        self.torque_feedforward(machine, &self.set_pos, &self.set_vel, &self.set_acc, &vec![0.0; motors], &mut ff);
        // A winding that was warm stays warm, and a DC link keeps its charge. The current that holds the load comes
        // out of that charge (L i^2 / 2); below the bus level the rectifier makes it up at the next step.
        // shortcut: without the DC link simulated that L i^2 / 2 (about 1 mJ) comes from nowhere; book it on the supply
        // if a budget across an enable ever needs to close there.
        // So does a housing (effects.motor_thermal_nodes), and a drive's thermal image (effects.drive_overload).
        let kept: Vec<_> = self
            .drives
            .iter()
            .map(|d| ((d.temperature_rise, d.housing_rise, d.overload_image, d.overloaded), d.dc_voltage))
            .collect();
        self.drives =
            (0..motors).map(|k| Drive::at_rest(&self.motor_params[k], self.motor_pos[k], hold[k], ff[k])).collect();
        for (k, (drive, (heat, voltage))) in self.drives.iter_mut().zip(kept).enumerate() {
            let p = &self.motor_params[k];
            (drive.temperature_rise, drive.housing_rise, drive.overload_image, drive.overloaded) = heat;
            if p.dc_link {
                drive.dc_voltage = (voltage * voltage
                    - p.inductance * drive.current * drive.current / p.dc_capacitance)
                    .max(0.0)
                    .sqrt();
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
        let motor_params = machine
            .motors()
            .iter()
            .map(|m| DriveParams::from_config(&crate::catalog::motor_config(cfg, m.id)?, dt, n_velocity as f64 * dt))
            .collect::<Result<Vec<_>>>()?;
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
        let path_tolerance = cfg.non_negative("profile.path_tolerance_m")?;
        let tension_reserve = cfg.positive("profile.tension_reserve")?;
        if tension_reserve > 1.0 {
            return Err(ConfigError::Invalid("profile.tension_reserve".into()));
        }
        let tension_margin = cfg.non_negative("profile.tension_margin_n")?;
        // The ratings the time-optimal plan keeps beside the motors': each gearbox's output torque where the machine
        // reports it, and the guides' load where it reports theirs (the verdicts judge the same signals).
        let reported = machine.signal_infos();
        let reports = |id: &str| reported.iter().any(|s| s.id == id);
        let gearbox = machine
            .motors()
            .iter()
            .map(|m| -> Result<Option<[f64; 3]>> {
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
                Ok(rated.is_finite().then_some([own.f64("gearbox.ratio").unwrap_or(1.0), inertia, rated]))
            })
            .collect::<Result<Vec<_>>>()?;
        let guides = reported.iter().any(|s| s.id.starts_with("axis.") && s.id.ends_with(".guide_load"));
        let guide = if guides { cfg.f64("ratings.guide_max_load_n").unwrap_or(f64::INFINITY) } else { f64::INFINITY };
        let mut segments = Vec::new();
        for (i, segment) in cfg.array("cycle.segment")?.iter().enumerate() {
            let bad = || ConfigError::Invalid(format!("cycle.segment[{i}]"));
            let number = |v: &toml::Value| v.as_float().or_else(|| v.as_integer().map(|i| i as f64));
            let target: Vec<f64> =
                segment.get("target").and_then(|t| t.as_array()).ok_or_else(bad)?.iter().filter_map(number).collect();
            if !in_range(machine, &target) {
                return Err(bad());
            }
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
                    Some(via_pieces(&points, &target))
                }
            };
            let shape = match segment.get("shape") {
                None => law.shape,
                Some(name) => name
                    .as_str()
                    .and_then(Shape::from_name)
                    .ok_or_else(|| ConfigError::Invalid(format!("cycle.segment[{i}].shape")))?,
            };
            segments.push(Segment {
                target,
                dwell_ticks: (segment
                    .get("dwell_s")
                    .and_then(number)
                    .filter(|d| d.is_finite() && *d >= 0.0)
                    .ok_or_else(bad)?
                    / profile_cycle)
                    .round() as u64,
                payload: segment.get("payload").and_then(|p| p.as_bool()).ok_or_else(bad)?,
                path: via,
                law: MotionLaw { shape, ..law.clone() },
            });
        }
        // A path through via points has to be one the planner times, from where the segment before ends.
        for (i, segment) in segments.iter().enumerate() {
            let from = &segments[(i + segments.len() - 1) % segments.len()].target;
            if let Some(pieces) = &segment.path
                && path::plan(machine.axes(), from, pieces, profile_cycle, path_tolerance, path::ARC_SLACK)
                    .map_or(true, |points| !points.iter().all(|(_, pose)| near_start(machine, from, pose)))
            {
                return Err(ConfigError::Invalid(format!("cycle.segment[{i}].via")));
            }
        }
        if profile_cycle != self.profile_cycle {
            self.rescale_ticks(self.profile_cycle / profile_cycle);
        }
        // The belt goes on where it was: the parts picked and missed so far stay so.
        let mut conveyor = Conveyor::from_config(cfg, machine.axes().len())?;
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
        (self.n_velocity, self.n_position, self.n_profile) = (n_velocity, n_position, n_profile);
        (self.profile_cycle, self.setpoint_ticks, self.params, self.segments) =
            (profile_cycle, setpoint_ticks, params, segments);
        self.motor_params = motor_params;
        (self.inertias, self.load_encoder) = (inertias, load_encoder);
        self.blend_time = blend_time;
        self.path_tolerance = path_tolerance;
        (self.law, self.spline_c2) = (law, spline_c2);
        // The margin comes off both ratings before the share is taken: it is for what the rigid model leaves out
        // whatever the room (friction, the belt's vibration), which is roughly absolute, not a share of the room.
        let tension = [
            cfg.f64("ratings.belt_min_tension_n").map_or(f64::NEG_INFINITY, |t| t + tension_margin),
            cfg.f64("ratings.belt_max_tension_n").map_or(f64::INFINITY, |t| t - tension_margin),
            tension_reserve,
        ];
        self.plan_limits = path::PlanLimits { reserve: torque_reserve, tension, gearbox, guide };
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
        for half in self.shaper_held.iter_mut().flatten() {
            *half *= ratio;
        }
        self.tick = (now as f64 * ratio).round() as u64;
        let new_now = self.tick as f64;
        // A tick of u64::MAX is a motion not yet picked up.
        let scale = |tick: &mut u64| {
            if *tick != u64::MAX {
                *tick = (new_now + (*tick as f64 - now as f64) * ratio).round().max(0.0) as u64;
            }
        };
        match &mut self.motion {
            // A conveyor pass runs on the simulated time, which a new profile cycle does not change.
            Motion::Idle | Motion::Conveyor => {}
            Motion::Moving { start_tick, tail, .. } => {
                scale(start_tick);
                if let Some((_, old_start)) = tail {
                    scale(old_start);
                }
            }
            Motion::Dwell { until_tick } => scale(until_tick),
            Motion::Following { start_tick, .. } | Motion::Trajectory { start_tick, .. } => scale(start_tick),
        }
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
        // shortcut: setpoints on the fieldbus keep their count of cycles, so its delay changes with the cycle
        // at once instead of after the queue has run through; resample the queue if that step ever matters.
    }

    pub fn command(&mut self, command: Command, machine: &dyn Machine) -> std::result::Result<(), CommandError> {
        if !self.enabled && !matches!(command, Command::Enable | Command::Disable) {
            return Err(CommandError { key: "error-disabled" });
        }
        match command {
            Command::Disable => self.disable(),
            Command::Enable => {
                if !self.enabled {
                    self.enabled = true;
                    self.hold_here(machine);
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
                if self.segments.is_empty() && self.conveyor.is_none() {
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
            Command::Trajectory(points) => {
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
                self.motion = trajectory(points, self.spline_c2);
            }
        }
        Ok(())
    }

    /// Drives off: a move or cycle under way is dropped.
    fn disable(&mut self) {
        (self.motion, self.cycle, self.stopping, self.enabled) = (Motion::Idle, None, false, false);
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
    /// the axis limits with every axis's speed capped at the feed; one without as `plan_path` plans it, as fast as the
    /// motors allow with `time_optimal`. A program without a move is refused (`error-path`).
    pub fn plan_program(
        &self,
        machine: &dyn Machine,
        runs: &[crate::program::Run],
        time_optimal: bool,
    ) -> std::result::Result<Vec<(f64, Vec<f64>)>, CommandError> {
        let mut points = vec![(0.0, self.raw_pos.clone())];
        for run in runs.iter().filter(|run| !run.pieces.is_empty()) {
            let (t0, start) = points[points.len() - 1].clone();
            let planned = self.plan_from(machine, &start, &run.pieces, time_optimal && run.feed.is_none(), run.feed)?;
            // Each plan starts at its start at 0 s, which the one before has as its end.
            points.extend(planned.into_iter().skip(1).map(|(t, pose)| (t0 + t, pose)));
        }
        if points.len() < 2 {
            return Err(CommandError { key: "error-path" });
        }
        Ok(points)
    }

    /// `plan_path` from `start`, with every axis's speed capped at `feed` where there is one (then within the axis
    /// limits, also with `time_optimal`).
    fn plan_from(
        &self,
        machine: &dyn Machine,
        start: &[f64],
        pieces: &[Piece],
        time_optimal: bool,
        feed: Option<f64>,
    ) -> std::result::Result<Vec<(f64, Vec<f64>)>, CommandError> {
        let time_optimal = time_optimal && feed.is_none();
        let capped: Vec<sim_core::AxisInfo> = machine
            .axes()
            .iter()
            .map(|a| sim_core::AxisInfo { max_vel: feed.map_or(a.max_vel, |f| a.max_vel.min(f)), ..a.clone() })
            .collect();
        // An arc or a spline can swing out of the range although every point given is inside. Said here,
        // because `Command::Trajectory` refuses such poses with a message about poses at times.
        let reachable = |points: &[(f64, Vec<f64>)]| {
            if points.iter().all(|(_, pose)| near_start(machine, start, pose)) {
                Ok(())
            } else {
                Err(CommandError { key: "error-path-range" })
            }
        };
        let within_axis_limits =
            || path::plan(&capped, start, pieces, self.profile_cycle, self.path_tolerance, machine.setpoint_slack());
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
            path::plan_time_optimal(
                machine,
                &self.motor_drives(machine),
                &self.plan_limits,
                start,
                pieces,
                self.profile_cycle,
                self.path_tolerance,
                machine.setpoint_slack(),
            )
        } else {
            within_axis_limits()
        }?;
        reachable(&points)?;
        Ok(points)
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

    /// True while a move, a dwell or the cycle is under way.
    pub fn busy(&self) -> bool {
        self.cycle.is_some()
            || matches!(
                self.motion,
                Motion::Moving { .. } | Motion::Dwell { .. } | Motion::Trajectory { .. } | Motion::Conveyor
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
        let (Motion::Dwell { until_tick }, Some(i)) = (&self.motion, self.cycle) else { return None };
        let total = self.segments[i].dwell_ticks;
        (total > 0).then(|| 1.0 - until_tick.saturating_sub(self.tick) as f64 / total as f64)
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
        machine.motor_state(state, &mut self.motor_pos, &mut self.motor_vel);
        if !self.enabled {
            for (k, (drive, torque)) in self.drives.iter_mut().zip(torque).enumerate() {
                drive.disabled_step(&self.motor_params[k]);
                *torque = 0.0;
            }
            if self.params.shared_dc_link {
                shared_dc_link(&self.params, &mut self.drives);
            }
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
            // shortcut: the scale reads the tool as `tool_state` reports it, in the room (a compliant frame and a leaning
            // tool inside the loop); add a `Machine` reading of the carriage against the frame when a machine needs it.
            let load = self.load_encoder.map(|scale| {
                let axes = self.set_pos.len();
                let (mut tool, mut rate, mut motors) = (vec![0.0; axes], vec![0.0; axes], vec![0.0; self.drives.len()]);
                machine.tool_state(state, &mut tool, &mut rate);
                tool.iter_mut().for_each(|x| *x = quantise(*x, scale));
                machine.inverse(&tool, &mut motors);
                motors
            });
            let cycle = self.n_position as f64 * self.profile_cycle / self.n_profile as f64;
            for (k, drive) in self.drives.iter_mut().enumerate() {
                let measured = load.as_ref().map_or(drive.encoder(), |l| l[k]);
                drive.position_loop(&self.motor_params[k], measured, cycle);
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
    }

    /// The tool setpoint at `time`: the last profile cycle's run on by its speed and acceleration, as the drives' fine
    /// interpolation runs the motors' on (at most one profile cycle). What the analysis measures the tool against.
    pub fn plan_now(&self, time: f64, out: &mut [f64]) {
        let t = (time - self.clock).clamp(0.0, self.profile_cycle);
        for (i, x) in out.iter_mut().enumerate() {
            *x = self.set_pos[i] + (self.set_vel[i] + 0.5 * self.set_acc[i] * t) * t;
        }
    }

    /// The base step done (see `update`): per drive the current at its end and the electrical energy, copper loss and
    /// work on the shaft over it, four numbers per drive in their order. Then the shared DC link, and the trip.
    pub fn winding_stepped(&mut self, windings: &[f64]) {
        for (k, (drive, w)) in self.drives.iter_mut().zip(windings.as_chunks::<4>().0).enumerate() {
            drive.winding_stepped(&self.motor_params[k], w[0], [w[1], w[2], w[3]]);
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
            Motion::Dwell { until_tick } => {
                if tick >= until_tick {
                    self.next_cycle_segment(machine);
                } else {
                    self.motion = Motion::Dwell { until_tick };
                }
            }
            Motion::Trajectory { points, speeds, start_tick } => {
                let start_tick = if start_tick == u64::MAX { tick } else { start_tick };
                let t = (tick - start_tick) as f64 * dt;
                let last = points.len() - 1;
                if t >= points[last].0 {
                    self.set_pos.copy_from_slice(&points[last].1);
                    self.set_vel.fill(0.0);
                    self.set_acc.fill(0.0);
                    // A cycle segment's path arrives as a move does (see above): the gripper acts and the dwell starts
                    // once the shaped setpoint is there, where the segment waits for it.
                    if let Some(i) = self.cycle {
                        let waits =
                            self.segments[i].dwell_ticks > 0 || self.segments[i].payload != self.payload_attached;
                        let lag = if waits { self.shaper_span(machine, &self.set_pos).ceil() } else { 0.0 };
                        self.motion = if t >= points[last].0 + lag * dt {
                            self.payload_attached = self.segments[i].payload;
                            Motion::Dwell { until_tick: tick + self.segments[i].dwell_ticks }
                        } else {
                            Motion::Trajectory { points, speeds, start_tick }
                        };
                    }
                } else {
                    // A point a rounding error after `t` counts as at `t`: at a point the cubic that starts there is
                    // taken, not by the rounding of the two times the end of the one before, whose acceleration differs.
                    let i = (points.partition_point(|(time, _)| *time <= t + 1e-6 * dt) - 1).min(last - 1);
                    for a in 0..self.set_pos.len() {
                        [self.set_pos[a], self.set_vel[a], self.set_acc[a]] = hermite(&points, &speeds, i, a, t);
                    }
                    self.motion = Motion::Trajectory { points, speeds, start_tick };
                }
            }
            Motion::Following { from, to, start_tick } => {
                // The setpoint came during the cycle before this one, and the way to it starts there. Counted
                // from this cycle, the plan stood still for a cycle at every setpoint of a stream and made up
                // for it a ninth faster than the stream.
                let start_tick = if start_tick == u64::MAX { tick - 1 } else { start_tick };
                let n = self.setpoint_ticks as f64;
                let done = ((tick - start_tick) as f64 / n).min(1.0);
                for i in 0..to.len() {
                    // At the end the target itself, not from + (to - from), which can be a rounding off it.
                    self.set_pos[i] = if done < 1.0 { from[i] + (to[i] - from[i]) * done } else { to[i] };
                    // The speed of the cycle that ends here, also on arrival: in a stream the next setpoint is
                    // on its way, and a speed of zero for this one cycle would be a dip in every interval.
                    self.set_vel[i] = (to[i] - from[i]) / (n * dt);
                }
                self.set_acc.fill(0.0);
                if done < 1.0 {
                    self.motion = Motion::Following { from, to, start_tick };
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
            self.shaper_held = None;
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
        let setpoints =
            (0..motors).map(|k| [pos[k], (ahead[k] - behind[k]) / (2.0 * h), motor_acc[k], ff[k]]).collect();
        // Fieldbus: the drives get the newest frame that has arrived, one the controller computed some cycles ago. With
        // jitter each frame takes its own number of cycles, and a late one leaves the drives the frame before. Frames
        // older than the longest delay have arrived and are dropped; before any has arrived, the oldest.
        let late = if self.fieldbus_jitter > 0 {
            (self.jitter.uniform() * (self.fieldbus_jitter + 1) as f64) as usize
        } else {
            0
        };
        self.fieldbus.push_back((self.fieldbus_delay + late, setpoints));
        while self.fieldbus.len() > self.fieldbus_delay + self.fieldbus_jitter + 1 {
            self.fieldbus.pop_front();
        }
        let newest = self.fieldbus.len() - 1;
        let arrived = (0..=newest).find(|&age| self.fieldbus[newest - age].0 <= age).map_or(0, |age| newest - age);
        for (drive, &[pos, vel, acc, ff]) in self.drives.iter_mut().zip(&self.fieldbus[arrived].1) {
            (drive.pos_set, drive.vel_set, drive.acc_set, drive.torque_ff) = (pos, vel, acc, ff);
            drive.since_setpoint = 0.0;
        }
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
    /// axes' shapers. 0 with the shaper off.
    pub fn shaper_span(&self, machine: &dyn Machine, pose: &[f64]) -> f64 {
        self.mode_halves(machine, pose)
            .map_or(0.0, |halves| self.trains(&halves).iter().flatten().fold(0.0, |m: f64, (t, _)| m.max(*t)))
    }

    /// With the plan at `pose`: per axis how far the shaped setpoint lags a plan at a constant speed (the mean of the
    /// shaper's delays, s), and how long after the plan it arrives (`shaper_span`, s). Zeros with the shaper off.
    pub fn shaper_delays(&self, machine: &dyn Machine, pose: &[f64]) -> (Vec<f64>, f64) {
        let Some(halves) = self.mode_halves(machine, pose) else { return (vec![0.0; self.set_pos.len()], 0.0) };
        let dt = self.profile_cycle;
        let trains = self.trains(&halves);
        let mean = trains.iter().map(|train| train.iter().map(|(t, a)| t * a).sum::<f64>() * dt).collect();
        (mean, trains.iter().flatten().fold(0.0, |m: f64, (t, _)| m.max(*t)) * dt)
    }

    /// Input shaping: on each axis the setpoints become a weighted sum of copies of the plan at the delays of its
    /// shaper (`shaper::impulses`): ZVD three copies, now, half a period and a whole period ago, weighted 1 : 2K : K^2;
    /// ZV, ZVDD, EI and the two-hump EI likewise with their own (`profile.shaper_type`). The vibration each copy starts
    /// is cancelled by the others, also when the resonance is some percent off the frequency the shaper was set to.
    /// Every further mode's shaper is convolved with it (the second shaper, `profile.shaper_modes`): copies at the sums
    /// of the delays with the products of the weights, so that each mode's vibration is cancelled. An axis takes the
    /// modes that act on it (`profile.shaper_axes`, a mode's `axes`).
    ///
    /// Following the pose, the half period is that of the resonance the machine reports where the plan
    /// stands, and the second shaper's that of the machine's second mode there. The weights always add up to one, so
    /// once the plan has stood still for a whole period the setpoint is the plan exactly, whatever the period did on the
    /// way. While the plan stands still the period stays as it was: a resonance that changes at rest (the gripper takes
    /// its payload) would reach back into the move that has just arrived and take the setpoint off its target.
    fn shape(&mut self, machine: &dyn Machine) {
        // shortcut: while the period changes, the speed and acceleration setpoints are the delayed plan's,
        // not the exact rates of the shaped position (off by the rate of the delay, a few percent in a Z
        // move); they are feedforward only. Shape the position alone and difference it if that matters.
        let Some(halves) = self.mode_halves(machine, &self.raw_pos) else {
            self.history.clear();
            self.shaper_reach = 0;
            return;
        };
        let standing = self.history.back().is_some_and(|past| past[0] == self.raw_pos);
        let halves = self.shaper_held.take().filter(|held| standing && held.len() == halves.len()).unwrap_or(halves);
        let trains = self.trains(&halves);
        self.shaper_held = Some(halves);
        self.history.push_back([self.raw_pos.clone(), self.raw_vel.clone(), self.raw_acc.clone()]);
        // The store keeps the longest span met so far: a period that grows again finds its past.
        let span = trains.iter().flatten().fold(0.0, |m: f64, (t, _)| m.max(*t));
        self.shaper_reach = self.shaper_reach.max(span.ceil() as usize + 2);
        while self.history.len() > self.shaper_reach {
            self.history.pop_front();
        }
        let newest = self.history.len() - 1;
        // The plan `delay` cycles ago, between two stored cycles; before the store reaches back that far, its oldest.
        let ago = |which: usize, axis: usize, delay: f64| -> f64 {
            let (whole, part) = (delay.floor() as usize, delay.fract());
            let at = |n: usize| self.history[newest - n.min(newest)][which][axis];
            at(whole) * (1.0 - part) + at(whole + 1) * part
        };
        let shaped = |which: usize, axis: usize| {
            let train = &trains[axis];
            train.iter().map(|(delay, weight)| weight * ago(which, axis, *delay)).sum::<f64>()
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

    fn next_cycle_segment(&mut self, machine: &dyn Machine) {
        let Some(last) = self.cycle else { return };
        if self.stopping {
            self.cycle = None;
            self.stopping = false;
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
        // from the start pose), the straight move.
        let planned = self.segments[next].path.as_ref().and_then(|pieces| {
            path::plan(
                machine.axes(),
                &self.set_pos,
                pieces,
                self.profile_cycle,
                self.path_tolerance,
                machine.setpoint_slack(),
            )
            .ok()
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
    Motion::Trajectory { points, speeds, start_tick: u64::MAX }
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
    Ok(MotionLaw { shape, deceleration_ratio, vertical })
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
