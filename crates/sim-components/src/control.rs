//! Motion controller: plans in tool coordinates, transforms to motor setpoints, and runs one drive
//! per motor. It works on any machine through the machine interface.

use crate::drive::{Drive, DriveParams, shared_dc_link};
use crate::path::{self, Piece};
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

#[derive(Debug, Clone)]
struct Segment {
    target: Vec<f64>,
    dwell_ticks: u64,
    /// Whether the payload is attached once this segment has arrived.
    payload: bool,
    /// With `via`: the path to the target, planned by `path::plan` where the segment starts (see `via_pieces`).
    path: Option<Vec<Piece>>,
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

#[derive(Debug, Clone)]
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
    /// A cubic curve through the points with the given speeds at them (Hermite interpolation).
    Trajectory {
        points: Vec<(f64, Vec<f64>)>,
        speeds: Vec<Vec<f64>>,
        start_tick: u64,
    },
}

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
    /// For a path timed as fast as the motors allow: the share of the torque-speed curve it may use.
    torque_reserve: f64,
    /// How far a planned path may leave its lines and arcs for transition curves.
    path_tolerance: f64,
    /// For a path timed as fast as the motors allow: the lowest and highest belt tension it may need
    /// (`ratings.belt_min_tension_n`, `belt_max_tension_n` less `profile.tension_margin_n` on both; unbounded where the
    /// config has none), and the share of the room to them it may use (`profile.tension_reserve`).
    belt_tension: [f64; 3],
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
    /// Input shaper: (amplitude ratio K of successive steps, half the damped period in profile cycles), and the same
    /// of a second shaper convolved with it, (0, 0) without one.
    shaper: Option<(f64, f64, f64, f64)>,
    /// With the shaper following the pose: half the damped period in profile cycles, times the frequency, of the first
    /// shaper and of the second.
    shaper_follows: Option<(f64, f64)>,
    /// A shaper setting (`shaper`, `shaper_follows`) from a changed config, waiting for the plan to stand
    /// still: taken under a moving plan it would make the setpoint jump.
    pending_shaper: Option<(Option<(f64, f64, f64, f64)>, Option<(f64, f64)>)>,
    /// How many cycles of `history` the shaper has needed so far.
    shaper_reach: usize,
    /// The half periods (first, second) the shaper used in the last cycle, kept while the plan stands still.
    shaper_held: Option<(f64, f64)>,
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
            torque_reserve: 1.0,
            path_tolerance: 0.0,
            belt_tension: [f64::NEG_INFINITY, f64::INFINITY, 1.0],
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
            shaper_follows: None,
            pending_shaper: None,
            shaper_reach: 0,
            shaper_held: None,
            fieldbus_delay: 0,
            fieldbus: Default::default(),
            fieldbus_jitter: 0,
            jitter: Default::default(),
            tick: 0,
            history: Default::default(),
        };
        c.apply_config(cfg, machine)?;
        // Nothing moves yet: the shaper is in use from the first cycle on.
        (c.shaper, c.shaper_follows) = c.pending_shaper.take().unwrap_or_default();
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
        machine.feedforward_torque(&self.set_pos, &self.set_acc, &mut ff);
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
        let (mut shaper, mut shaper_follows) = (None, None);
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
        // K and half the damped period in profile cycles of a ZVD shaper.
        let zvd = |frequency: f64, damping: f64| {
            let root = (1.0 - damping * damping).sqrt();
            ((-damping * std::f64::consts::PI / root).exp(), 0.5 / (frequency * root) / profile_cycle)
        };
        if cfg.bool("effects.input_shaping")? {
            let per_hz = |damping: f64| 0.5 / (1.0 - damping * damping).sqrt() / profile_cycle;
            let ((k, half), (k2, half2)) =
                (zvd(frequency, damping), if frequency2 > 0.0 { zvd(frequency2, damping2) } else { (0.0, 0.0) });
            shaper = Some((k, half, k2, half2));
            shaper_follows = cfg.bool("effects.shaper_follows_pose")?.then_some((per_hz(damping), per_hz(damping2)));
        }
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
        // A cycle under way goes on with a segment the new cycle has: the move in progress finishes where it
        // was going, and the next one starts from there. A cycle that lost all its segments ends.
        self.cycle = self.cycle.and_then(|i| Some(i.min(segments.len().checked_sub(1)?)));
        (self.n_velocity, self.n_position, self.n_profile) = (n_velocity, n_position, n_profile);
        (self.profile_cycle, self.setpoint_ticks, self.params, self.segments) =
            (profile_cycle, setpoint_ticks, params, segments);
        self.motor_params = motor_params;
        self.blend_time = blend_time;
        (self.torque_reserve, self.path_tolerance) = (torque_reserve, path_tolerance);
        // The margin comes off both ratings before the share is taken: it is for what the rigid model leaves out
        // whatever the room (friction, the belt's vibration), which is roughly absolute, not a share of the room.
        self.belt_tension = [
            cfg.f64("ratings.belt_min_tension_n").map_or(f64::NEG_INFINITY, |t| t + tension_margin),
            cfg.f64("ratings.belt_max_tension_n").map_or(f64::INFINITY, |t| t - tension_margin),
            tension_reserve,
        ];
        (self.pending_shaper, self.fieldbus_delay, self.fieldbus_jitter) =
            (Some((shaper, shaper_follows)), fieldbus_delay, fieldbus_jitter);
        Ok(())
    }

    /// A new profile cycle, `ratio` of them to an old one: every stored tick keeps its distance in time from
    /// now, to the nearest new cycle, and the shaper's past is read off at the new spacing. Without this a
    /// motion under way would jump to another point of its plan.
    fn rescale_ticks(&mut self, ratio: f64) {
        let now = self.tick;
        // The shaper in use counts its period in cycles too. (A pending one was worked out for the new cycle.)
        self.shaper = self.shaper.map(|(k, half, k2, half2)| (k, half * ratio, k2, half2 * ratio));
        self.shaper_follows = self.shaper_follows.map(|(per_hz, per_hz2)| (per_hz * ratio, per_hz2 * ratio));
        self.shaper_held = self.shaper_held.map(|(half, half2)| (half * ratio, half2 * ratio));
        self.tick = (now as f64 * ratio).round() as u64;
        let new_now = self.tick as f64;
        // A tick of u64::MAX is a motion not yet picked up.
        let scale = |tick: &mut u64| {
            if *tick != u64::MAX {
                *tick = (new_now + (*tick as f64 - now as f64) * ratio).round().max(0.0) as u64;
            }
        };
        match &mut self.motion {
            Motion::Idle => {}
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
                    mv: Move::new(&self.raw_pos, &target, machine.axes()),
                    start_tick: u64::MAX,
                    tail: None,
                };
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
                self.motion = trajectory(points);
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
        let points = if time_optimal {
            path::plan_time_optimal(
                machine,
                &self.params,
                self.torque_reserve,
                self.belt_tension,
                &self.raw_pos,
                pieces,
                self.profile_cycle,
                self.path_tolerance,
                machine.setpoint_slack(),
            )
        } else {
            path::plan(
                machine.axes(),
                &self.raw_pos,
                pieces,
                self.profile_cycle,
                self.path_tolerance,
                machine.setpoint_slack(),
            )
        }?;
        // An arc or a spline can swing out of the range although every point given is inside. Said here,
        // because `Command::Trajectory` refuses such poses with a message about poses at times.
        if !points.iter().all(|(_, pose)| near_start(machine, &self.raw_pos, pose)) {
            return Err(CommandError { key: "error-path-range" });
        }
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
                &self.params,
                self.torque_reserve,
                self.belt_tension,
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
            || matches!(self.motion, Motion::Moving { .. } | Motion::Dwell { .. } | Motion::Trajectory { .. })
    }

    /// False between `Command::Disable` and `Command::Enable`.
    pub fn enabled(&self) -> bool {
        self.enabled
    }

    pub fn cycle_segment(&self) -> Option<usize> {
        self.cycle
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

    /// The motors stand `shift` from where they stood without having turned there: the machine's model was
    /// exchanged, and belt stretch and gear twist came or went with it, or its ratio changed. The encoders'
    /// last readings move along, so that the step does not read as speed, and so do the position setpoints
    /// at the drives and on the fieldbus, so that it does not read as following error. The next profile
    /// cycle works them out anew.
    pub fn motors_shifted(&mut self, shift: &[f64]) {
        for (k, (drive, by)) in self.drives.iter_mut().zip(shift).enumerate() {
            drive.shift_encoder(*by);
            drive.pos_set += by;
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
            self.profile_tick(self.tick + 1, machine);
        }
        if step.is_multiple_of(self.n_position) {
            for (k, (drive, pos)) in self.drives.iter_mut().zip(&self.motor_pos).enumerate() {
                drive.position_loop(&self.motor_params[k], *pos);
            }
            // Following error per motor, transformed back to the tool axes for display.
            let (set, actual): (Vec<f64>, Vec<f64>) =
                self.drives.iter().map(|d| (d.pos_set, d.pos_set - d.following_error)).unzip();
            let (mut tool_set, mut tool_actual) = (self.set_pos.clone(), self.set_pos.clone());
            machine.forward(&set, &mut tool_set);
            machine.forward(&actual, &mut tool_actual);
            for i in 0..self.tool_error.len() {
                self.tool_error[i] = tool_set[i] - tool_actual[i];
            }
        }
        for (k, drive) in self.drives.iter_mut().enumerate() {
            if step.is_multiple_of(self.n_velocity) {
                drive.velocity_loop(&self.motor_params[k], self.motor_pos[k], self.motor_vel[k]);
            }
            drive.current_control(&self.motor_params[k], self.motor_vel[k]);
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
                // The shaper's oldest copy is a whole period old, read between two stored cycles.
                let lag = if waits {
                    self.shaper_halves(machine, &self.set_pos)
                        .map_or(0.0, |(half, half2)| (2.0 * half + 2.0 * half2).ceil())
                } else {
                    0.0
                };
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
                        let lag = if waits {
                            self.shaper_halves(machine, &self.set_pos)
                                .map_or(0.0, |(half, half2)| (2.0 * half + 2.0 * half2).ceil())
                        } else {
                            0.0
                        };
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
                    let h = points[i + 1].0 - points[i].0;
                    let s = (t - points[i].0) / h;
                    // Cubic through both points with the speeds there: position, and its first and second rate.
                    let basis = [
                        [
                            2.0 * s * s * s - 3.0 * s * s + 1.0,
                            s * s * s - 2.0 * s * s + s,
                            -2.0 * s * s * s + 3.0 * s * s,
                            s * s * s - s * s,
                        ],
                        [
                            6.0 * s * s - 6.0 * s,
                            3.0 * s * s - 4.0 * s + 1.0,
                            -6.0 * s * s + 6.0 * s,
                            3.0 * s * s - 2.0 * s,
                        ],
                        [12.0 * s - 6.0, 6.0 * s - 4.0, -12.0 * s + 6.0, 6.0 * s - 2.0],
                    ];
                    for a in 0..self.set_pos.len() {
                        let at = |b: [f64; 4]| {
                            b[0] * points[i].1[a]
                                + b[1] * h * speeds[i][a]
                                + b[2] * points[i + 1].1[a]
                                + b[3] * h * speeds[i + 1][a]
                        };
                        self.set_pos[a] = at(basis[0]);
                        self.set_vel[a] = at(basis[1]) / h;
                        self.set_acc[a] = at(basis[2]) / (h * h);
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
            (self.shaper, self.shaper_follows) = next;
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
        // Motor speed setpoints: the inverse transformation applied a moment ahead and behind.
        let h = 1e-3;
        let shifted =
            |sign: f64| -> Vec<f64> { self.set_pos.iter().zip(&self.set_vel).map(|(p, v)| p + sign * h * v).collect() };
        machine.inverse(&shifted(1.0), &mut ahead);
        machine.inverse(&shifted(-1.0), &mut behind);
        machine.feedforward_torque_moving(&self.set_pos, &self.set_vel, &self.set_acc, &mut ff);
        // Motor acceleration setpoints the same way, for the drives' fine interpolation.
        let (mut acc_ahead, mut acc_behind) = (vec![0.0; motors], vec![0.0; motors]);
        let shifted =
            |sign: f64| -> Vec<f64> { self.set_pos.iter().zip(&self.set_acc).map(|(p, a)| p + sign * h * a).collect() };
        machine.inverse(&shifted(1.0), &mut acc_ahead);
        machine.inverse(&shifted(-1.0), &mut acc_behind);
        let setpoints = (0..motors)
            .map(|k| [pos[k], (ahead[k] - behind[k]) / (2.0 * h), (acc_ahead[k] - acc_behind[k]) / (2.0 * h), ff[k]])
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

    /// Half the period in profile cycles of the first shaper and of the second (0 without one) with the plan at `pose`:
    /// as configured, or, when the shaper follows the pose, that of the resonance the machine reports there for the
    /// first and that of its second mode (`Machine::modes`) for the second; one the machine does not report stays as
    /// configured. `None` with the shaper off.
    pub fn shaper_halves(&self, machine: &dyn Machine, pose: &[f64]) -> Option<(f64, f64)> {
        let (_, half, _, half2) = self.shaper?;
        let Some((per_hz, per_hz2)) = self.shaper_follows else { return Some((half, half2)) };
        let second = if half2 > 0.0 { machine.modes(pose).get(1).map_or(half2, |mode| per_hz2 / mode.hz) } else { 0.0 };
        Some((machine.resonance_hz(pose).map_or(half, |resonance| per_hz / resonance), second))
    }

    /// Input shaping (ZVD): the setpoints become the sum of three copies of the plan, now, half a period
    /// and a whole period ago, weighted 1 : 2K : K^2. The vibration each copy starts is cancelled by the
    /// others, also when the resonance is some percent off the frequency the shaper was set to. A second shaper
    /// is convolved with the first: nine copies, at the sums of the delays with the products of the weights, so
    /// that the vibration of each of the two resonances is cancelled. Without one its weights are 1 : 0 : 0.
    ///
    /// Following the pose, the half period is that of the resonance the machine reports where the plan
    /// stands, and the second shaper's that of the machine's second mode there. The weights always add up to one, so once the plan has stood still for a whole period the
    /// setpoint is the plan exactly, whatever the period did on the way. While the plan stands still the period stays
    /// as it was: a resonance that changes at rest (the gripper takes its payload) would reach back into the move that
    /// has just arrived and take the setpoint off its target.
    fn shape(&mut self, machine: &dyn Machine) {
        // shortcut: while the period changes, the speed and acceleration setpoints are the delayed plan's,
        // not the exact rates of the shaped position (off by the rate of the delay, a few percent in a Z
        // move); they are feedforward only. Shape the position alone and difference it if that matters.
        let (Some((k, _, k2, _)), Some(halves)) = (self.shaper, self.shaper_halves(machine, &self.raw_pos)) else {
            self.history.clear();
            self.shaper_reach = 0;
            return;
        };
        let standing = self.history.back().is_some_and(|past| past[0] == self.raw_pos);
        let (half, half2) = self.shaper_held.filter(|_| standing).unwrap_or(halves);
        self.shaper_held = Some((half, half2));
        self.history.push_back([self.raw_pos.clone(), self.raw_vel.clone(), self.raw_acc.clone()]);
        // The store keeps the longest period met so far: a period that grows again finds its past.
        self.shaper_reach = self.shaper_reach.max((2.0 * half + 2.0 * half2).ceil() as usize + 2);
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
        let sum = (1.0 + k) * (1.0 + k) * (1.0 + k2) * (1.0 + k2);
        let (first, second) = (
            [(0.0, 1.0), (half, 2.0 * k), (2.0 * half, k * k)],
            [(0.0, 1.0), (half2, 2.0 * k2), (2.0 * half2, k2 * k2)],
        );
        let shaped = |which: usize, axis: usize| {
            first
                .iter()
                .flat_map(|a| second.iter().map(move |b| (a.0 + b.0, a.1 * b.1)))
                .map(|(delay, weight)| weight * ago(which, axis, delay))
                .sum::<f64>()
                / sum
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
        let next = Move::new(&segment.target, &self.segments[next_index].target, machine.axes());
        if current.delta().iter().zip(next.delta()).any(|(a, b)| a * b < 0.0) || next.duration() == 0.0 {
            return None;
        }
        let lead = self.blend_time.min(current.ramp()).min(next.ramp());
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
                trajectory(points)
            }
            _ => Motion::Moving {
                mv: Move::new(&self.set_pos, &self.segments[next].target, machine.axes()),
                start_tick: u64::MAX,
                tail: None,
            },
        };
    }
}

/// A path given as poses at times as the controller runs it: a cubic through them, with the speed at each point the
/// slope between its neighbours (of the four around it where they are evenly spaced), at rest at both ends.
fn trajectory(points: Vec<(f64, Vec<f64>)>) -> Motion {
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
    Motion::Trajectory { points, speeds, start_tick: u64::MAX }
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
