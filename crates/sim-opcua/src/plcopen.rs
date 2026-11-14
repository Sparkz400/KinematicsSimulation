//! One axis as PLCopen motion sees it: the function blocks MC_Power, MC_Home, MC_MoveAbsolute, MC_MoveRelative,
//! MC_Stop and MC_Reset, and of synchronised motion MC_GearIn, MC_GearOut, MC_CamIn and MC_CamOut, as handshake
//! variables, and the axis state machine behind them.
//!
//! Each axis runs its own profile here and the link streams the positions to the controller with
//! `Command::Setpoint`, one per setpoint interval, as a PLC drives servo drives over a fieldbus. So axes move
//! independently and at the same time, MC_Stop brakes in the middle of a move, and a new move takes over from
//! the one under way at the position and speed the axis has.
//!
//! The profile is a trapezoid: velocity, acceleration and deceleration are kept, jerk is not. The controller
//! interpolates linearly between the streamed setpoints, so within one interval the speed is constant anyway.
//! Parameters above the axis limits are cut to the limits.
//!
//! Synchronised motion (PLCopen Motion Control Part 1): the blocks sit in the slave's folder and name their master by
//! its axis id (`GearIn.Master`, `CamIn.Master`). The slave follows the master's setpoint (MasterValueSource
//! mcSetValue) in the same setpoint interval, the master moved first (`advance_all`). MC_GearIn brings the slave's
//! speed to the ratio times the master's with its Acceleration (its Deceleration where the slave has to slow down),
//! then holds the slave on the master's travel times the ratio: `InGear`. MC_CamIn holds the slave on the cam table at
//! the master's position from its start: `InSync`. The table is the client's (`CamIn.Table`, CamTableID 0) or the
//! machine's own electronic cam for this axis (CamTableID 1, `Machine::cam`), whose master may also be the machine's
//! virtual one (`CamIn.Master` empty), run up to the cam's master speed. MC_GearOut and MC_CamOut end the coupling:
//! the slave goes on at its speed (ContinuousMotion) until another block takes it over. Another block started on the
//! slave aborts the coupling; a master that goes into ErrorStop or Disabled ends it with an error, the slave braking.

use opcua::types::Variant;
use sim_components::cam::{Cam, Law, Segment};
use sim_core::{AxisInfo, CamMaster};

/// PLCopen axis states, numbered as in TwinCAT's `MC_AxisStates`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum State {
    Disabled = 1,
    Standstill = 2,
    ErrorStop = 3,
    Stopping = 4,
    Homing = 5,
    DiscreteMotion = 6,
    ContinuousMotion = 7,
    SynchronizedMotion = 8,
}

/// The function blocks with an `Execute` input, in this order, and their numeric inputs.
pub const FBS: [&str; 9] =
    ["Home", "MoveAbsolute", "MoveRelative", "Stop", "Reset", "GearIn", "GearOut", "CamIn", "CamOut"];
const HOME: usize = 0;
const MOVE_ABSOLUTE: usize = 1;
const STOP: usize = 3;
const RESET: usize = 4;
const GEAR_IN: usize = 5;
const GEAR_OUT: usize = 6;
const CAM_IN: usize = 7;
const CAM_OUT: usize = 8;
/// The block of a motion no block reports on any more: the braking left by a refused re-trigger.
const UNOWNED: usize = usize::MAX;
const PARAMS: [&[&str]; 9] = [
    &[],
    &["Position", "Velocity", "Acceleration", "Deceleration", "Jerk"],
    &["Distance", "Velocity", "Acceleration", "Deceleration", "Jerk"],
    &["Deceleration"],
    &[],
    &["RatioNumerator", "RatioDenominator", "Acceleration", "Deceleration", "Jerk"],
    &[],
    &["MasterOffset", "SlaveOffset", "MasterScaling", "SlaveScaling", "StartMode", "CamTableID"],
    &[],
];
/// The text inputs, block and name: the master's axis id and the client's cam table.
pub const TEXTS: [(usize, &str); 3] = [(GEAR_IN, "Master"), (CAM_IN, "Master"), (CAM_IN, "Table")];

/// `ErrorID` values.
pub const ERR_NOT_POWERED: u16 = 1;
pub const ERR_STATE: u16 = 2;
pub const ERR_PARAMETER: u16 = 3;
pub const ERR_RANGE: u16 = 4;
pub const ERR_BUSY: u16 = 5;
pub const ERR_REFUSED: u16 = 6;
pub const ERR_DRIVES_OFF: u16 = 7;
/// The drives went off because the machine ran into an obstacle (effects.crash).
pub const ERR_CRASH: u16 = 8;
/// MC_GearIn, MC_CamIn: no axis of that id, the slave itself, a master that is a slave itself, or no virtual master
/// (the machine has no cam); or the master went into ErrorStop or Disabled while coupled.
pub const ERR_MASTER: u16 = 9;
/// MC_CamIn: the client's cam table cannot be read, or the machine has no cam of its own (CamTableID 1).
pub const ERR_TABLE: u16 = 10;

/// A function block's inputs and outputs. `Active` reads the same as `Busy`: there is no buffering. `done` is
/// MC_GearIn's `InGear` and MC_CamIn's `InSync`, true beside `busy` while the slave is coupled.
#[derive(Debug, Clone, Default)]
pub struct Fb {
    pub execute: bool,
    pub done: bool,
    pub busy: bool,
    pub aborted: bool,
    pub error: bool,
    pub error_id: u16,
    pub params: [f64; 6],
}

/// A slave's master: an axis by its index, or the machine's virtual cam master, run up `t` seconds ago from `from`.
#[derive(Debug, Clone)]
enum Master {
    Axis(usize),
    Virtual { run: CamMaster, t: f64, from: f64 },
}

/// How a slave follows its master.
#[derive(Debug, Clone)]
enum Coupling {
    /// MC_GearIn: the ratio, whether the slave runs at the master's speed times it, and where the master stood.
    Gear { ratio: f64, in_gear: bool, master_at: f64 },
    /// MC_CamIn: the slave at `slave_ref + slave_scale * cam(master_scale * (m - master_ref) + master_offset)`.
    Cam { cam: Cam, master_ref: f64, master_offset: f64, master_scale: f64, slave_ref: f64, slave_scale: f64 },
}

#[derive(Debug, Clone)]
struct Motion {
    fb: usize,
    /// `None` brakes to a stop.
    target: Option<f64>,
    vel: f64,
    acc: f64,
    dec: f64,
    /// Synchronised motion: the master and the coupling. The profile above is not used, but `dec` brakes where the
    /// coupling ends with an error.
    sync: Option<(Master, Coupling)>,
    /// After MC_GearOut or MC_CamOut: the slave goes on at its speed (ContinuousMotion).
    keeps: bool,
}

impl Motion {
    fn profile(fb: usize, target: Option<f64>, vel: f64, acc: f64, dec: f64) -> Self {
        Self { fb, target, vel, acc, dec, sync: None, keeps: false }
    }
}

/// A cam table as the client writes it (`CamIn.Table`): lines of segments `master span, slave rise, law` (the laws of
/// `Law::from_name`), or of points `master, slave[, slope]` (`sim_components::cam::parse_table`). `None` where it
/// cannot be read.
pub fn cam_table(text: &str) -> Option<Cam> {
    let rows: Vec<Vec<&str>> = text
        .lines()
        .map(|l| l.split(',').map(str::trim).collect::<Vec<_>>())
        .filter(|f| f[0].parse::<f64>().is_ok())
        .collect();
    if rows.iter().any(|f| f.len() == 3 && Law::from_name(f[2]).is_some()) {
        let segments = rows
            .iter()
            .map(|f| match f[..] {
                [master, slave, law] => Some(Segment {
                    master: master.parse().ok()?,
                    slave: slave.parse().ok()?,
                    law: Law::from_name(law)?,
                }),
                _ => None,
            })
            .collect::<Option<Vec<_>>>()?;
        return Cam::from_segments(&segments).ok();
    }
    Cam::from_table(&sim_components::cam::parse_table(text).ok()?).ok()
}

#[derive(Debug, Clone)]
pub struct Axis {
    info: AxisInfo,
    home: f64,
    /// MC_Power.Enable.
    pub enable: bool,
    pub fbs: [Fb; 9],
    pub homed: bool,
    /// The axis error; not 0 is ErrorStop.
    pub error_id: u16,
    /// The setpoint streamed to the controller, and its speed.
    pub pos: f64,
    pub vel: f64,
    motion: Option<Motion>,
    /// MC_Stop has been executed and its Execute is still held, or it still brakes.
    stopping: bool,
    /// The text inputs of `TEXTS`, in their order.
    pub texts: [String; 3],
    /// The machine's own electronic cam with this axis its slave (`Machine::cam`): the table over one master period,
    /// and its master's speed and run-up. MC_CamIn takes it with CamTableID 1.
    pub own_cam: Option<(Cam, CamMaster)>,
}

impl Axis {
    /// An axis standing at `home`, the position MC_Home moves to. The numeric inputs start at the axis limits.
    pub fn new(info: &AxisInfo, home: f64) -> Self {
        let mut fbs: [Fb; 9] = Default::default();
        for (fb, params) in fbs.iter_mut().zip(PARAMS) {
            for (value, name) in fb.params.iter_mut().zip(params) {
                *value = match *name {
                    "Velocity" => info.max_vel,
                    "Acceleration" | "Deceleration" => info.max_acc,
                    "Jerk" => info.max_jerk,
                    "Position" => home,
                    "RatioNumerator" | "RatioDenominator" | "MasterScaling" | "SlaveScaling" | "StartMode" => 1.0,
                    _ => 0.0,
                };
            }
        }
        Self {
            info: info.clone(),
            home,
            enable: false,
            fbs,
            homed: false,
            error_id: 0,
            pos: home,
            vel: 0.0,
            motion: None,
            stopping: false,
            texts: Default::default(),
            own_cam: None,
        }
    }

    pub fn state(&self) -> State {
        match &self.motion {
            _ if self.error_id != 0 => State::ErrorStop,
            _ if !self.enable => State::Disabled,
            Some(m) if m.fb == HOME => State::Homing,
            // MC_Stop's own braking, or MC_Stop done with its Execute held; never with another block's motion.
            _ if self.stopping => State::Stopping,
            Some(m) if m.sync.is_some() => State::SynchronizedMotion,
            Some(m) if m.keeps => State::ContinuousMotion,
            Some(_) => State::DiscreteMotion,
            None => State::Standstill,
        }
    }

    /// The axis this one follows as a slave, where its master is an axis.
    pub fn master(&self) -> Option<usize> {
        match self.motion.as_ref()?.sync.as_ref()? {
            (Master::Axis(i), _) => Some(*i),
            (Master::Virtual { .. }, _) => None,
        }
    }

    /// A text input by its name after the block's (`GearIn.Master`, `CamIn.Master`, `CamIn.Table`), taken on the
    /// block's rising edge. False for no such input.
    pub fn set_text(&mut self, name: &str, value: String) -> bool {
        let Some(k) = TEXTS.iter().position(|(fb, text)| name.split_once('.') == Some((FBS[*fb], *text))) else {
            return false;
        };
        self.texts[k] = value;
        true
    }

    pub fn moving(&self) -> bool {
        self.motion.is_some()
    }

    /// The axis's variables as (name, value, writable), always in the same order. `drives_on` says whether the
    /// controller's drives are on, `actual` is the simulated axis's position and velocity.
    pub fn variables(&self, drives_on: bool, actual: (f64, f64)) -> Vec<(String, Variant, bool)> {
        let status = self.enable && drives_on;
        // Also with Enable off: the axis stays in ErrorStop until MC_Reset (PLCopen state diagram), and MC_Power says so.
        let error = self.error_id != 0;
        let mut out: Vec<(String, Variant, bool)> = vec![
            ("Power.Enable".into(), self.enable.into(), true),
            ("Power.Status".into(), status.into(), false),
            ("Power.Valid".into(), (status && self.error_id == 0).into(), false),
            ("Power.Error".into(), error.into(), false),
            ("Power.ErrorID".into(), (if error { self.error_id } else { 0 }).into(), false),
        ];
        for (k, ((name, fb), params)) in FBS.iter().zip(&self.fbs).zip(PARAMS).enumerate() {
            out.push((format!("{name}.Execute"), fb.execute.into(), true));
            for (param, value) in params.iter().zip(fb.params) {
                out.push((format!("{name}.{param}"), value.into(), true));
            }
            for ((_, text), value) in TEXTS.iter().zip(&self.texts).filter(|((fb, _), _)| *fb == k) {
                out.push((format!("{name}.{text}"), value.as_str().into(), true));
            }
            let done = match k {
                GEAR_IN => "InGear",
                CAM_IN => "InSync",
                _ => "Done",
            };
            for (output, value) in [
                (done, fb.done),
                ("Busy", fb.busy),
                ("Active", fb.busy),
                ("CommandAborted", fb.aborted),
                ("Error", fb.error),
            ] {
                out.push((format!("{name}.{output}"), value.into(), false));
            }
            out.push((format!("{name}.ErrorID"), fb.error_id.into(), false));
        }
        let state = self.state();
        out.extend([
            ("State".into(), format!("{state:?}").into(), false),
            ("StateID".into(), (state as i32).into(), false),
            ("ActualPosition".into(), actual.0.into(), false),
            ("ActualVelocity".into(), actual.1.into(), false),
            ("IsHomed".into(), self.homed.into(), false),
            ("Error".into(), (self.error_id != 0).into(), false),
            ("ErrorID".into(), self.error_id.into(), false),
        ]);
        out
    }

    /// Ends the motion under way: its function block reports `CommandAborted`, or `Error` with `error`.
    fn end_motion(&mut self, error: u16) {
        if let Some(m) = self.motion.take()
            && let Some(fb) = self.fbs.get_mut(m.fb)
        {
            (fb.busy, fb.aborted, fb.error, fb.error_id) = (false, error == 0, error != 0, error);
        }
        self.vel = 0.0;
    }

    /// An axis error: ErrorStop until MC_Reset.
    pub fn fault(&mut self, error: u16) {
        self.end_motion(error);
        (self.error_id, self.stopping) = (error, false);
    }

    /// MC_Power.Enable. Switched off, a motion under way is aborted.
    pub fn power(&mut self, on: bool) {
        if !on {
            self.end_motion(0);
            self.stopping = false;
        }
        self.enable = on;
    }

    /// A numeric input, by its name after the function block's (`MoveAbsolute.Position`). False for no such input.
    pub fn set_param(&mut self, name: &str, value: f64) -> bool {
        let Some((fb, param)) = name.split_once('.') else { return false };
        let Some(k) = FBS.iter().position(|f| *f == fb) else { return false };
        let Some(p) = PARAMS[k].iter().position(|n| *n == param) else { return false };
        self.fbs[k].params[p] = value;
        true
    }

    /// `Execute` of function block `k` written. A rising edge starts it with the inputs it has then; `setpoint` is where
    /// the controller's setpoint stands, `busy` whether the controller runs a move, path or cycle of the link's other
    /// commands. A falling edge clears the outputs of a block that has finished; one still busy runs on and keeps its
    /// outputs until the next rising edge.
    /// A block of synchronised motion has no master here (`execute_in` gives it one).
    pub fn execute(&mut self, k: usize, on: bool, setpoint: f64, busy: bool) {
        self.execute_with(k, on, setpoint, busy, Err(ERR_MASTER));
    }

    /// `execute` with the master MC_GearIn and MC_CamIn would couple to: an axis (its index and where its setpoint
    /// stands), `None` for the machine's virtual cam master, or why there is none.
    fn execute_with(
        &mut self,
        k: usize,
        on: bool,
        setpoint: f64,
        busy: bool,
        master: Result<Option<(usize, f64)>, u16>,
    ) {
        let rising = on && !self.fbs[k].execute;
        self.fbs[k].execute = on;
        if !on {
            if !self.fbs[k].busy {
                let fb = &mut self.fbs[k];
                (fb.done, fb.aborted, fb.error, fb.error_id) = (false, false, false, 0);
            }
            if k == STOP && self.motion.as_ref().is_none_or(|m| m.fb != STOP) {
                self.stopping = false;
            }
            return;
        }
        if !rising {
            return;
        }
        let fb = &mut self.fbs[k];
        (fb.done, fb.aborted, fb.error, fb.error_id) = (false, false, false, 0);
        if k == RESET {
            self.error_id = 0;
            self.fbs[k].done = true;
            return;
        }
        let state = self.state();
        let refuse = match state {
            State::ErrorStop => ERR_STATE,
            State::Disabled => ERR_NOT_POWERED,
            State::Stopping if k != STOP => ERR_STATE,
            State::Homing if k != STOP => ERR_STATE,
            State::DiscreteMotion | State::ContinuousMotion | State::SynchronizedMotion if k == HOME => ERR_STATE,
            _ if k == GEAR_OUT && !self.coupled_by(GEAR_IN) => ERR_STATE,
            _ if k == CAM_OUT && !self.coupled_by(CAM_IN) => ERR_STATE,
            _ if busy => ERR_BUSY,
            _ => 0,
        };
        if k == GEAR_OUT || k == CAM_OUT {
            self.decouple(k, refuse);
            return;
        }
        if k == GEAR_IN || k == CAM_IN {
            self.couple(k, refuse, setpoint, master);
            return;
        }
        let p = self.fbs[k].params;
        let a = &self.info;
        // `min` would turn NaN into the limit: what is not a number stays one, to be refused below.
        let cut = |v: f64, max: f64| if v.is_finite() { v.min(max) } else { f64::NAN };
        let (target, vel, acc, dec) = match k {
            HOME => (Some(self.home), 0.5 * a.max_vel, 0.5 * a.max_acc, 0.5 * a.max_acc),
            STOP => (None, a.max_vel, a.max_acc, cut(p[0], a.max_acc)),
            _ => {
                let start = if self.moving() { self.pos } else { setpoint };
                (
                    Some(if k == MOVE_ABSOLUTE { p[0] } else { start + p[0] }),
                    cut(p[1], a.max_vel),
                    cut(p[2], a.max_acc),
                    cut(p[3], a.max_acc),
                )
            }
        };
        let valid = [vel, acc, dec].iter().all(|v| v.is_finite() && *v > 0.0) && target.is_none_or(f64::is_finite);
        let refuse = match target {
            _ if refuse != 0 => refuse,
            _ if !valid => ERR_PARAMETER,
            Some(t) if t < a.min || t > a.max => ERR_RANGE,
            _ => 0,
        };
        if refuse != 0 {
            self.refused(k, refuse);
            return;
        }
        self.take_over(k, setpoint, k == STOP, Motion::profile(k, target, vel, acc, dec));
    }

    /// Block `k` refused with `error`. Re-triggered while busy, the block gives up its motion, so that it reports Error
    /// alone (Done, Busy, Error and CommandAborted are exclusive), and the axis brakes to a stop with the motion's
    /// deceleration, a motion no block reports on, which a new move may take over (PLCopen Part 1 leaves this case
    /// open). A coupling given up so brakes the slave too.
    fn refused(&mut self, k: usize, error: u16) {
        if self.fbs[k].busy
            && let Some(m) = &mut self.motion
        {
            (m.fb, m.target, m.sync, m.keeps) = (UNOWNED, None, None, false);
        }
        let fb = &mut self.fbs[k];
        (fb.busy, fb.done, fb.error, fb.error_id) = (false, false, true, error);
    }

    /// A new motion of block `k` takes over the one under way at its position and speed.
    fn take_over(&mut self, k: usize, setpoint: f64, stopping: bool, motion: Motion) {
        let (pos, vel_now) = if self.moving() { (self.pos, self.vel) } else { (setpoint, 0.0) };
        self.end_motion(0);
        (self.pos, self.vel, self.stopping) = (pos, vel_now, stopping);
        // A block started again while busy is not aborted by itself.
        (self.fbs[k].busy, self.fbs[k].aborted) = (true, false);
        self.motion = Some(motion);
    }

    /// Whether the axis is coupled by block `k` (MC_GearIn or MC_CamIn).
    fn coupled_by(&self, k: usize) -> bool {
        self.motion.as_ref().is_some_and(|m| m.fb == k && m.sync.is_some())
    }

    /// MC_GearIn or MC_CamIn started: the coupling to `master` (see `execute_with`).
    fn couple(&mut self, k: usize, refuse: u16, setpoint: f64, master: Result<Option<(usize, f64)>, u16>) {
        let (p, a) = (self.fbs[k].params, &self.info);
        let pos = if self.moving() { self.pos } else { setpoint };
        let cut = |v: f64, max: f64| if v.is_finite() { v.min(max) } else { f64::NAN };
        let positive = |v: f64| v.is_finite() && v > 0.0;
        let finite_nonzero = |v: f64| v.is_finite() && v != 0.0;
        // The master and where it stands, the coupling, and the deceleration that brakes the slave where it ends.
        let coupled = || -> Result<(Master, Coupling, [f64; 2]), u16> {
            if refuse != 0 {
                return Err(refuse);
            }
            let master = master?;
            if k == GEAR_IN {
                let (ratio, acc, dec) = (p[0] / p[1], cut(p[2], a.max_acc), cut(p[3], a.max_acc));
                let (Some((m, at)), true) = (master, finite_nonzero(ratio) && positive(acc) && positive(dec)) else {
                    return Err(if master.is_none() { ERR_MASTER } else { ERR_PARAMETER });
                };
                return Ok((Master::Axis(m), Coupling::Gear { ratio, in_gear: false, master_at: at }, [acc, dec]));
            }
            let [master_offset, slave_offset, master_scale, slave_scale, mode, table] = p;
            if !(master_offset.is_finite() && slave_offset.is_finite())
                || !finite_nonzero(master_scale)
                || !finite_nonzero(slave_scale)
                || !(mode == 0.0 || mode == 1.0)
            {
                return Err(ERR_PARAMETER);
            }
            let cam = match table {
                0.0 => cam_table(&self.texts[2]).ok_or(ERR_TABLE)?,
                1.0 => self.own_cam.as_ref().ok_or(ERR_TABLE)?.0.clone(),
                _ => return Err(ERR_PARAMETER),
            };
            let (master, at) = match master {
                Some((m, at)) => (Master::Axis(m), at),
                // The machine's virtual master, from rest at 0.
                None => {
                    let run = self.own_cam.as_ref().ok_or(ERR_MASTER)?.1;
                    (Master::Virtual { run, t: 0.0, from: 0.0 }, 0.0)
                }
            };
            // Absolute: where the table puts the slave now, where it has to stand (no ramp-in); relative: from where
            // master and slave stand.
            let (master_ref, slave_ref) = if mode == 0.0 {
                let wanted = slave_offset + slave_scale * cam.at(master_scale * at + master_offset)[0];
                if (wanted - pos).abs() > 1e-6 {
                    return Err(ERR_RANGE);
                }
                (0.0, slave_offset)
            } else {
                (at, pos - slave_scale * cam.at(master_offset)[0])
            };
            let coupling = Coupling::Cam { cam, master_ref, master_offset, master_scale, slave_ref, slave_scale };
            Ok((master, coupling, [a.max_acc, a.max_acc]))
        };
        match coupled() {
            Err(error) => self.refused(k, error),
            Ok((master, coupling, [acc, dec])) => {
                let motion =
                    Motion { sync: Some((master, coupling)), ..Motion::profile(k, None, self.info.max_vel, acc, dec) };
                self.take_over(k, setpoint, false, motion);
                // A cam is in sync from its start; a gear once the slave's speed has come to the master's.
                self.fbs[k].done = k == CAM_IN;
            }
        }
    }

    /// MC_GearOut or MC_CamOut: the coupling ends, the slave goes on at its speed until another block takes it over;
    /// the coupling block is no longer busy, and not aborted.
    fn decouple(&mut self, k: usize, refuse: u16) {
        if refuse != 0 {
            let fb = &mut self.fbs[k];
            (fb.error, fb.error_id) = (true, refuse);
            return;
        }
        if let Some(m) = &mut self.motion {
            let from = m.fb;
            (m.fb, m.sync, m.keeps) = (UNOWNED, None, true);
            (self.fbs[from].busy, self.fbs[from].done) = (false, false);
        }
        self.fbs[k].done = true;
    }

    /// A slave's interval `t` at its master's new place and speed; `lost` where the master went into ErrorStop or
    /// Disabled: the coupling ends with ErrorID 9 and the slave brakes with the coupling's deceleration.
    fn follow(&mut self, t: f64, (at, speed): (f64, f64), lost: bool) {
        let Some(m) = &mut self.motion else { return };
        let k = m.fb;
        if lost {
            (m.fb, m.target, m.sync) = (UNOWNED, None, None);
            let fb = &mut self.fbs[k];
            (fb.busy, fb.done, fb.error, fb.error_id) = (false, false, true, ERR_MASTER);
            return self.advance(t, self.pos);
        }
        let (acc, dec) = (m.acc, m.dec);
        match m.sync.as_mut().map(|(_, c)| c) {
            Some(Coupling::Gear { ratio, in_gear, master_at }) => {
                let wanted = *ratio * speed;
                if *in_gear {
                    // Geared: the master's travel times the ratio, exactly.
                    self.pos += *ratio * (at - *master_at);
                    self.vel = wanted;
                } else {
                    let braking = wanted.abs() < self.vel.abs() || wanted * self.vel < 0.0;
                    let dv = if braking { dec } else { acc } * t;
                    self.vel = wanted.clamp(self.vel - dv, self.vel + dv);
                    self.pos += self.vel * t;
                    *in_gear = self.vel == wanted;
                    self.fbs[k].done = *in_gear;
                }
                *master_at = at;
            }
            Some(Coupling::Cam { cam, master_ref, master_offset, master_scale, slave_ref, slave_scale }) => {
                let s = cam.at(*master_scale * (at - *master_ref) + *master_offset);
                self.pos = *slave_ref + *slave_scale * s[0];
                self.vel = *slave_scale * s[1] * *master_scale * speed;
            }
            None => {}
        }
    }

    /// One setpoint interval `t` of the profile. Without a motion the axis follows `setpoint`, where the controller's
    /// setpoint stands, so that the link's other commands and other axes' moves do not leave it behind.
    pub fn advance(&mut self, t: f64, setpoint: f64) {
        let Some(m) = &mut self.motion else {
            (self.pos, self.vel) = (setpoint, 0.0);
            return;
        };
        if m.keeps {
            self.pos += self.vel * t;
            return;
        }
        // The virtual master runs on; an axis master's slave follows in `advance_all`.
        if let Some((Master::Virtual { run, t: since, from }, _)) = &mut m.sync {
            *since += t;
            let now = sim_components::cam::master_change(0.0, run.speed, run.ramp_s, *since, None);
            let master = (*from + now[0], now[1]);
            return self.follow(t, master, false);
        }
        if m.sync.is_some() {
            return;
        }
        let m = &*m;
        let v_wanted = match m.target {
            None => 0.0,
            Some(target) => {
                // The highest speed from which steps of this grid still brake to a stop at the target, and no step
                // beyond it.
                let s = target - self.pos;
                let brake = m.dec * t * ((0.25 + 2.0 * s.abs() / (m.dec * t * t)).sqrt() - 0.5);
                s.signum() * m.vel.min(brake).min(s.abs() / t)
            }
        };
        let braking = v_wanted.abs() < self.vel.abs() || v_wanted * self.vel < 0.0;
        let dv = if braking { m.dec } else { m.acc } * t;
        let vel = v_wanted.clamp(self.vel - dv, self.vel + dv);
        // Braking ends at standstill: going on the other way is acceleration, from the next interval on.
        self.vel = if vel * self.vel < 0.0 { 0.0 } else { vel };
        self.pos += self.vel * t;
        if let Some(target) = m.target
            && (target - self.pos).abs() < 1e-12
        {
            self.pos = target;
        }
        if self.vel == 0.0 && m.target.is_none_or(|target| self.pos == target) {
            let k = m.fb;
            self.motion = None;
            if let Some(fb) = self.fbs.get_mut(k) {
                (fb.busy, fb.done) = (false, true);
            }
            self.homed |= k == HOME;
            self.stopping = k == STOP && self.fbs[k].execute;
        }
    }
}

/// `Execute` of block `k` of axis `i` written (`Axis::execute`), with MC_GearIn's and MC_CamIn's master found among
/// `axes` by its id: not the slave itself, and no slave of an axis itself (`ERR_MASTER` else); an empty id is the
/// machine's virtual cam master (MC_CamIn only).
pub fn execute_in(axes: &mut [Axis], i: usize, k: usize, on: bool, setpoint: f64, busy: bool) {
    let name = match k {
        GEAR_IN => &axes[i].texts[0],
        CAM_IN => &axes[i].texts[1],
        _ => return axes[i].execute(k, on, setpoint, busy),
    };
    let master = match axes.iter().position(|a| a.info.id == name.trim()) {
        _ if name.trim().is_empty() => {
            if k == CAM_IN {
                Ok(None)
            } else {
                Err(ERR_MASTER)
            }
        }
        Some(j) if j != i && axes[j].master().is_none() => Ok(Some((j, axes[j].pos))),
        _ => Err(ERR_MASTER),
    };
    axes[i].execute_with(k, on, setpoint, busy, master);
}

/// One setpoint interval `t` of every axis (`Axis::advance`, `setpoints` where the controller's setpoint stands for
/// each): the axes that follow no axis first, then the slaves at their masters' new places.
pub fn advance_all(axes: &mut [Axis], t: f64, setpoints: &[f64]) {
    for (axis, at) in axes.iter_mut().zip(setpoints) {
        if axis.master().is_none() {
            axis.advance(t, *at);
        }
    }
    for i in 0..axes.len() {
        if let Some(j) = axes[i].master() {
            let lost = matches!(axes[j].state(), State::ErrorStop | State::Disabled);
            let master = (axes[j].pos, axes[j].vel);
            axes[i].follow(t, master, lost);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axis() -> Axis {
        let info =
            AxisInfo { id: "x", key: "", unit: "m", min: 0.0, max: 1.0, max_vel: 1.0, max_acc: 10.0, max_jerk: 100.0 };
        let mut axis = Axis::new(&info, 0.1);
        axis.power(true);
        axis
    }

    /// Runs until the motion is over and returns the steps taken; checks speed and acceleration on the way.
    fn run(axis: &mut Axis, vel: f64, acc: f64) -> usize {
        let t = 0.01;
        for n in 0..10_000 {
            if !axis.moving() {
                return n;
            }
            let (before, at) = (axis.vel, axis.pos);
            axis.advance(t, 0.0);
            assert!(axis.vel.abs() <= vel + 1e-12, "{}", axis.vel);
            assert!((axis.vel - before).abs() <= acc * t + 1e-9, "{} after {before}", axis.vel);
            assert!((axis.pos - at - axis.vel * t).abs() < 1e-12);
        }
        panic!("the motion did not end");
    }

    /// A move arrives exactly, from rest to rest, within the limits, in about the time a trapezoid takes.
    #[test]
    fn moves_arrive_exactly_within_the_limits() {
        for target in [0.9, 0.1001, 0.5, 0.0] {
            let mut a = axis();
            a.set_param("MoveAbsolute.Position", target);
            a.set_param("MoveAbsolute.Velocity", 0.5);
            a.set_param("MoveAbsolute.Acceleration", 5.0);
            a.set_param("MoveAbsolute.Deceleration", 2.0);
            a.execute(MOVE_ABSOLUTE, true, 0.1, false);
            assert_eq!(a.state(), State::DiscreteMotion);
            let n = run(&mut a, 0.5, 5.0);
            assert_eq!((a.pos, a.vel), (target, 0.0));
            assert!(a.fbs[1].done && !a.fbs[1].busy && a.state() == State::Standstill);
            // A trapezoid's time: with or without reaching 0.5 m/s.
            let (d, r) = ((target - 0.1f64).abs(), 1.0 / 5.0 + 1.0 / 2.0);
            let ideal = if d > 0.5 * 0.5 * r / 2.0 { d / 0.5 + 0.5 * r / 2.0 } else { (2.0 * d * r).sqrt() };
            assert!((n as f64 * 0.01 - ideal).abs() < 0.05, "{target}: {n} steps, {ideal} s");
        }
    }

    /// MC_Stop brakes a move with its deceleration and the move reports CommandAborted; the axis stays in Stopping
    /// while Execute is held, and a move in Stopping is refused.
    #[test]
    fn stop_brakes_and_aborts_the_move() {
        let mut a = axis();
        a.set_param("MoveAbsolute.Position", 0.9);
        a.execute(MOVE_ABSOLUTE, true, 0.1, false);
        for _ in 0..30 {
            a.advance(0.01, 0.0);
        }
        let v = a.vel;
        a.set_param("Stop.Deceleration", 4.0);
        a.execute(STOP, true, 0.0, false);
        assert!(a.fbs[1].aborted && !a.fbs[1].busy && a.fbs[STOP].busy);
        let n = run(&mut a, 1.0, 4.0);
        assert!((n as f64 * 0.01 - v / 4.0).abs() < 0.02, "{n} steps from {v} m/s");
        assert!(a.fbs[STOP].done && a.state() == State::Stopping);
        a.execute(MOVE_ABSOLUTE, false, 0.0, false);
        a.execute(MOVE_ABSOLUTE, true, 0.0, false);
        assert_eq!((a.fbs[1].error, a.fbs[1].error_id), (true, ERR_STATE));
        a.execute(STOP, false, 0.0, false);
        assert_eq!(a.state(), State::Standstill);
    }

    /// A new move takes over a move under way at its speed, also one that must turn round.
    #[test]
    fn a_new_move_takes_over_at_speed() {
        let mut a = axis();
        a.set_param("MoveAbsolute.Position", 0.9);
        a.execute(MOVE_ABSOLUTE, true, 0.1, false);
        for _ in 0..30 {
            a.advance(0.01, 0.0);
        }
        let (v, p) = (a.vel, a.pos);
        a.set_param("MoveRelative.Distance", -0.2);
        a.execute(2, true, 0.0, false);
        assert_eq!((a.vel, a.pos), (v, p));
        assert!(a.fbs[1].aborted && a.fbs[2].busy);
        run(&mut a, 1.0, 10.0);
        assert!((a.pos - (p - 0.2)).abs() < 1e-12 && a.fbs[2].done);
    }

    /// Execute starts on its rising edge only; a block that finished clears its outputs on the falling edge.
    #[test]
    fn execute_is_edge_triggered() {
        let mut a = axis();
        a.set_param("MoveRelative.Distance", 0.1);
        a.execute(2, true, 0.1, false);
        run(&mut a, 1.0, 10.0);
        a.execute(2, true, a.pos, false);
        assert!(!a.moving() && a.fbs[2].done, "held Execute started it again");
        a.execute(2, false, a.pos, false);
        assert!(!a.fbs[2].done);
        a.execute(2, true, a.pos, false);
        run(&mut a, 1.0, 10.0);
        assert!((a.pos - 0.3).abs() < 1e-12);
    }

    /// Refusals: not powered, out of range, bad parameters; an axis error holds until MC_Reset.
    #[test]
    fn refusals_and_reset() {
        let mut a = axis();
        a.power(false);
        a.execute(MOVE_ABSOLUTE, true, 0.1, false);
        assert_eq!(a.fbs[1].error_id, ERR_NOT_POWERED);
        a.power(true);
        a.execute(MOVE_ABSOLUTE, false, 0.1, false);
        a.set_param("MoveAbsolute.Position", 2.0);
        a.execute(MOVE_ABSOLUTE, true, 0.1, false);
        assert_eq!(a.fbs[1].error_id, ERR_RANGE);
        a.execute(MOVE_ABSOLUTE, false, 0.1, false);
        a.set_param("MoveAbsolute.Position", 0.5);
        a.set_param("MoveAbsolute.Velocity", 0.0);
        a.execute(MOVE_ABSOLUTE, true, 0.1, false);
        assert_eq!(a.fbs[1].error_id, ERR_PARAMETER);
        a.fault(ERR_REFUSED);
        assert_eq!(a.state(), State::ErrorStop);
        a.execute(RESET, true, 0.1, false);
        assert!(a.fbs[RESET].done && a.state() == State::Standstill);
    }

    /// NaN and infinite numeric inputs are refused, not cut to the axis limits.
    #[test]
    fn nan_and_infinite_inputs_are_refused() {
        for (fb, name) in
            [(MOVE_ABSOLUTE, "Velocity"), (MOVE_ABSOLUTE, "Acceleration"), (2, "Deceleration"), (STOP, "Deceleration")]
        {
            for bad in [f64::NAN, f64::INFINITY] {
                let mut a = axis();
                a.set_param("MoveAbsolute.Position", 0.5);
                a.set_param(&format!("{}.{name}", FBS[fb]), bad);
                a.execute(fb, true, 0.1, false);
                assert_eq!((a.fbs[fb].error_id, a.moving()), (ERR_PARAMETER, false), "{}.{name} = {bad}", FBS[fb]);
            }
        }
    }

    /// Execute falling while busy and rising again restarts the block: it does not abort itself.
    #[test]
    fn a_block_started_again_does_not_abort_itself() {
        let mut a = axis();
        a.set_param("MoveRelative.Distance", 0.3);
        a.execute(2, true, 0.1, false);
        for _ in 0..10 {
            a.advance(0.01, 0.0);
        }
        a.execute(2, false, 0.0, false);
        assert!(a.fbs[2].busy);
        let at = a.pos;
        a.execute(2, true, 0.0, false);
        assert!(a.fbs[2].busy && !a.fbs[2].aborted);
        run(&mut a, 1.0, 10.0);
        assert!(a.fbs[2].done && !a.fbs[2].aborted);
        assert!((a.pos - at - 0.3).abs() < 1e-12);
    }

    /// A reversal from a speed residue near zero accelerates the other way with Acceleration, not Deceleration.
    #[test]
    fn a_reversal_through_zero_accelerates_with_acceleration() {
        let mut a = axis();
        a.set_param("MoveAbsolute.Position", 0.9);
        a.execute(MOVE_ABSOLUTE, true, 0.1, false);
        a.set_param("MoveRelative.Distance", -0.1);
        a.set_param("MoveRelative.Acceleration", 1.0);
        a.set_param("MoveRelative.Deceleration", 10.0);
        a.execute(2, true, 0.0, false);
        a.vel = 1e-15;
        a.advance(0.01, 0.0);
        assert!(a.vel <= 0.0 && a.vel >= -0.01 - 1e-12, "{} m/s after one step", a.vel);
    }

    /// Takeover by another move, MC_Stop in the middle of a move, a block started again and a reversal, one after
    /// the other: every step keeps the speed and acceleration of the motion under way, and each ends exactly.
    #[test]
    fn takeover_stop_restart_and_reversal_keep_the_limits() {
        let t = 0.01;
        let mut a = axis();
        // Each step: the motion's velocity, acceleration and deceleration bound it; braking through zero is two parts.
        let step = |a: &mut Axis, n: usize, (vel, acc, dec): (f64, f64, f64)| {
            // Without a motion the axis would follow the setpoint 0 given here.
            for _ in 0..n {
                if !a.moving() {
                    break;
                }
                let before = a.vel;
                a.advance(t, 0.0);
                assert!(a.vel.abs() <= vel + 1e-12, "{} m/s", a.vel);
                let dv = a.vel - before;
                let limit = if a.vel * before < 0.0 || a.vel.abs() > before.abs() { acc } else { dec };
                assert!(dv.abs() <= limit * t + 1e-9, "{before} -> {} m/s", a.vel);
            }
        };
        let params = |a: &mut Axis, fb: &str, v: f64, acc: f64, dec: f64| {
            a.set_param(&format!("{fb}.Velocity"), v);
            a.set_param(&format!("{fb}.Acceleration"), acc);
            a.set_param(&format!("{fb}.Deceleration"), dec);
        };
        params(&mut a, "MoveAbsolute", 0.8, 4.0, 4.0);
        a.set_param("MoveAbsolute.Position", 0.9);
        a.execute(MOVE_ABSOLUTE, true, 0.1, false);
        step(&mut a, 25, (0.8, 4.0, 4.0));
        // Taken over by a relative move backwards: brakes with its deceleration, turns with its acceleration.
        params(&mut a, "MoveRelative", 0.5, 1.0, 8.0);
        a.set_param("MoveRelative.Distance", -0.2);
        a.execute(2, true, 0.0, false);
        assert!(a.fbs[1].aborted && a.fbs[2].busy);
        step(&mut a, 20, (0.8, 1.0, 8.0));
        assert!(a.vel < 0.0, "turned round");
        // MC_Stop in the middle of it.
        a.set_param("Stop.Deceleration", 2.0);
        a.execute(STOP, true, 0.0, false);
        assert!(a.fbs[2].aborted && a.state() == State::Stopping);
        step(&mut a, 200, (0.5, 2.0, 2.0));
        assert!(!a.moving() && a.vel == 0.0 && a.fbs[STOP].done);
        a.execute(STOP, false, 0.0, false);
        // Started again in the middle of its move, MoveRelative counts from where the axis is then.
        a.execute(2, false, 0.0, false);
        a.set_param("MoveRelative.Distance", 0.3);
        a.execute(2, true, a.pos, false);
        step(&mut a, 15, (0.5, 1.0, 8.0));
        a.execute(2, false, 0.0, false);
        let at = a.pos;
        a.execute(2, true, 0.0, false);
        assert!(a.fbs[2].busy && !a.fbs[2].aborted);
        step(&mut a, 300, (0.5, 1.0, 8.0));
        assert!(
            !a.moving() && a.fbs[2].done && !a.fbs[2].aborted && (a.pos - at - 0.3).abs() < 1e-12,
            "{} from {at}, {:?}",
            a.pos,
            a.fbs[2]
        );
    }

    /// Two powered axes on a stroke of 0 to 10 m, up to 1 m/s and 100 m/s2: x the master at 1, y the slave at 2.
    fn pair() -> Vec<Axis> {
        let info =
            |id| AxisInfo { id, key: "", unit: "m", min: 0.0, max: 10.0, max_vel: 1.0, max_acc: 100.0, max_jerk: 1e4 };
        let mut axes = vec![Axis::new(&info("x"), 1.0), Axis::new(&info("y"), 2.0)];
        axes.iter_mut().for_each(|a| a.power(true));
        axes
    }

    /// The master x moving at `speed` (up to it within one interval of 10 ms), slave y standing.
    fn master_at_speed(axes: &mut [Axis], speed: f64) {
        axes[0].set_param("MoveAbsolute.Position", 9.0);
        axes[0].set_param("MoveAbsolute.Velocity", speed);
        execute_in(axes, 0, MOVE_ABSOLUTE, true, 1.0, false);
        advance_all(axes, 0.01, &[1.0, 2.0]);
        assert_eq!(axes[0].vel, speed);
    }

    /// MC_GearIn at ratio 2 (RatioNumerator 2, RatioDenominator 1) to a master running at 0.1 m/s, Acceleration 1 m/s2:
    /// the slave's speed rises by 1 * 0.01 = 0.01 m/s per interval of 10 ms to 2 * 0.1 = 0.2 m/s, which it reaches in
    /// the 20th interval, 0.2 s = 0.2 / 1 after the start: InGear then, Busy and Active on. On the way it has gone
    /// 0.01 * (0.01 + 0.02 + ... + 0.2) = 0.021 m (a continuous ramp would go 0.2^2 / 2 = 0.02 m); geared, it goes
    /// twice the master's travel, 0.1 m over the master's 0.05 m.
    #[test]
    fn gear_in_reaches_in_gear_at_ratio_2_after_its_acceleration() {
        let mut axes = pair();
        master_at_speed(&mut axes, 0.1);
        axes[1].set_text("GearIn.Master", "x".into());
        axes[1].set_param("GearIn.RatioNumerator", 2.0);
        axes[1].set_param("GearIn.RatioDenominator", 1.0);
        axes[1].set_param("GearIn.Acceleration", 1.0);
        execute_in(&mut axes, 1, GEAR_IN, true, 2.0, false);
        assert!(axes[1].fbs[GEAR_IN].busy && !axes[1].fbs[GEAR_IN].done);
        assert_eq!((axes[1].state(), axes[1].master()), (State::SynchronizedMotion, Some(0)));
        let mut intervals = 0;
        while !axes[1].fbs[GEAR_IN].done {
            advance_all(&mut axes, 0.01, &[0.0, 0.0]);
            intervals += 1;
            assert!(intervals < 100);
        }
        assert_eq!(intervals, 20);
        assert!((axes[1].vel - 0.2).abs() < 1e-12 && axes[1].fbs[GEAR_IN].busy, "{}", axes[1].vel);
        assert!((axes[1].pos - 2.021).abs() < 1e-12, "{}", axes[1].pos);
        let (m, s) = (axes[0].pos, axes[1].pos);
        for _ in 0..50 {
            advance_all(&mut axes, 0.01, &[0.0, 0.0]);
        }
        assert!((axes[1].pos - s - 2.0 * (axes[0].pos - m)).abs() < 1e-12 && (axes[0].pos - m - 0.05).abs() < 1e-12);
        let names: Vec<String> = axes[1].variables(true, (0.0, 0.0)).into_iter().map(|v| v.0).collect();
        assert!(names.contains(&"GearIn.InGear".into()) && names.contains(&"GearIn.Master".into()));
        assert!(!names.contains(&"GearIn.Done".into()) && names.contains(&"GearOut.Done".into()));

        // MC_GearOut: the slave goes on at 0.2 m/s, ContinuousMotion, GearIn no longer busy and not aborted.
        execute_in(&mut axes, 1, GEAR_OUT, true, 0.0, false);
        assert!(axes[1].fbs[GEAR_OUT].done && !axes[1].fbs[GEAR_IN].busy && !axes[1].fbs[GEAR_IN].aborted);
        assert_eq!(axes[1].state(), State::ContinuousMotion);
        let at = axes[1].pos;
        advance_all(&mut axes, 0.01, &[0.0, 0.0]);
        assert!((axes[1].pos - at - 0.002).abs() < 1e-12);
        // A GearOut with nothing geared is refused in that state; MC_Stop takes the slave over.
        execute_in(&mut axes, 1, GEAR_OUT, false, 0.0, false);
        execute_in(&mut axes, 1, GEAR_OUT, true, 0.0, false);
        assert_eq!(axes[1].fbs[GEAR_OUT].error_id, ERR_STATE);
        execute_in(&mut axes, 1, STOP, true, 0.0, false);
        assert_eq!(axes[1].state(), State::Stopping);
    }

    /// MC_CamIn, relative, with the client's table of a 3-4-5 rise of 0.1 m over 0.5 m of master and the 3-4-5 back
    /// over the next 0.5 m, the master at 0.2 m/s: InSync from the start; after 0.25 m of master (125 intervals of 10 ms)
    /// the slave has risen by half, 0.05 m, at 1.875 * 0.1 / 0.5 * 0.2 = 0.075 m/s; after 0.5 m by 0.1 m, at rest; a
    /// period on (1 m), back where it began. Started with a master that is no axis, or with a table that cannot be
    /// read, it is refused with ErrorID 9 and 10.
    #[test]
    fn cam_in_follows_a_3_4_5_table_at_the_master_s_speed() {
        let mut axes = pair();
        master_at_speed(&mut axes, 0.2);
        let m0 = axes[0].pos;
        axes[1].set_text("CamIn.Master", "x".into());
        axes[1].set_text("CamIn.Table", "span, rise, law\n0.5, 0.1, 3-4-5\n0.5, -0.1, 3-4-5\n".into());
        execute_in(&mut axes, 1, CAM_IN, true, 2.0, false);
        let fb = &axes[1].fbs[CAM_IN];
        assert!(fb.busy && fb.done && !fb.error, "{fb:?}");
        let ticks = |axes: &mut Vec<Axis>, n: usize| (0..n).for_each(|_| advance_all(axes, 0.01, &[0.0, 0.0]));
        ticks(&mut axes, 125);
        assert!((axes[0].pos - m0 - 0.25).abs() < 1e-12);
        let (pos, vel) = (axes[1].pos, axes[1].vel);
        assert!((pos - 2.05).abs() < 1e-12 && (vel - 0.075).abs() < 1e-12, "{pos} {vel}");
        ticks(&mut axes, 125);
        assert!((axes[1].pos - 2.1).abs() < 1e-12 && axes[1].vel.abs() < 1e-12);
        ticks(&mut axes, 250);
        assert!((axes[1].pos - 2.0).abs() < 1e-12);
        // MC_CamOut; then refusals.
        execute_in(&mut axes, 1, CAM_OUT, true, 0.0, false);
        assert!(axes[1].fbs[CAM_OUT].done && axes[1].state() == State::ContinuousMotion);
        let mut b = pair();
        b[1].set_text("CamIn.Master", "z".into());
        b[1].set_text("CamIn.Table", "0.5, 0.1, 3-4-5\n0.5, -0.1, 3-4-5".into());
        execute_in(&mut b, 1, CAM_IN, true, 2.0, false);
        assert_eq!(b[1].fbs[CAM_IN].error_id, ERR_MASTER);
        b[1].set_text("CamIn.Master", "x".into());
        b[1].set_text("CamIn.Table", "0.5, 0.1, three".into());
        execute_in(&mut b, 1, CAM_IN, false, 2.0, false);
        execute_in(&mut b, 1, CAM_IN, true, 2.0, false);
        assert_eq!(b[1].fbs[CAM_IN].error_id, ERR_TABLE);
    }

    /// A master switched off while its slave is geared ends the coupling with ErrorID 9: the slave brakes with GearIn's
    /// Deceleration (5 m/s2, from 0.5 m/s by 0.05 m/s in the first interval of 10 ms) to a stop. A GearIn started again
    /// on the slave with a ratio of 0 is refused, reports Error alone and gives up its coupling: the slave brakes the
    /// same way, in DiscreteMotion, owned by no block.
    #[test]
    fn a_lost_master_and_a_refused_gear_in_end_the_coupling_with_an_error() {
        for lose in [true, false] {
            let mut axes = pair();
            master_at_speed(&mut axes, 0.5);
            axes[1].set_text("GearIn.Master", "x".into());
            axes[1].set_param("GearIn.Deceleration", 5.0);
            execute_in(&mut axes, 1, GEAR_IN, true, 2.0, false);
            (0..100).for_each(|_| advance_all(&mut axes, 0.01, &[0.0, 0.0]));
            assert!(axes[1].fbs[GEAR_IN].done && (axes[1].vel - 0.5).abs() < 1e-12);
            if lose {
                axes[0].power(false);
            } else {
                execute_in(&mut axes, 1, GEAR_IN, false, 0.0, false);
                axes[1].set_param("GearIn.RatioNumerator", 0.0);
                execute_in(&mut axes, 1, GEAR_IN, true, 0.0, false);
            }
            advance_all(&mut axes, 0.01, &[0.0, 0.0]);
            let fb = &axes[1].fbs[GEAR_IN];
            let id = if lose { ERR_MASTER } else { ERR_PARAMETER };
            assert!(fb.error && fb.error_id == id && !fb.busy && !fb.done && !fb.aborted, "{fb:?}");
            assert_eq!(axes[1].state(), State::DiscreteMotion);
            assert!((axes[1].vel - 0.45).abs() < 1e-12, "{}", axes[1].vel);
            (0..20).for_each(|_| advance_all(&mut axes, 0.01, &[0.0, 0.0]));
            assert!(!axes[1].moving() && axes[1].state() == State::Standstill);
        }
    }

    /// MC_CamIn on the machine's own cam (CamTableID 1) with its virtual master (`CamIn.Master` empty): a slave that
    /// runs one to one with a master of 2 m/s, run up over 0.5 s by the sin^2 ramp. After 1.5 s the master has gone
    /// 2 * (1.5 - 0.5 / 2) = 2.5 m and the slave with it, at 2 m/s. Without a cam of its own the axis refuses it.
    #[test]
    fn cam_in_runs_the_machine_s_own_cam_on_its_virtual_master() {
        let mut axes = pair();
        let one_to_one = Cam::from_segments(&[Segment { master: 1.0, slave: 1.0, law: Law::Linear }]).unwrap();
        axes[1].set_param("CamIn.CamTableID", 1.0);
        execute_in(&mut axes, 1, CAM_IN, true, 2.0, false);
        assert_eq!(axes[1].fbs[CAM_IN].error_id, ERR_TABLE);
        execute_in(&mut axes, 1, CAM_IN, false, 2.0, false);
        axes[1].own_cam = Some((one_to_one, CamMaster { speed: 2.0, period: 1.0, ramp_s: 0.5 }));
        execute_in(&mut axes, 1, CAM_IN, true, 2.0, false);
        assert!(axes[1].fbs[CAM_IN].done && axes[1].master().is_none());
        assert_eq!(axes[1].state(), State::SynchronizedMotion);
        (0..150).for_each(|_| advance_all(&mut axes, 0.01, &[1.0, 2.0]));
        assert!((axes[1].pos - 4.5).abs() < 1e-9 && (axes[1].vel - 2.0).abs() < 1e-12, "{}", axes[1].pos);
    }
}
