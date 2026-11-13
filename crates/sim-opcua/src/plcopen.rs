//! One axis as PLCopen motion sees it: the function blocks MC_Power, MC_Home, MC_MoveAbsolute, MC_MoveRelative,
//! MC_Stop and MC_Reset as handshake variables, and the axis state machine behind them.
//!
//! Each axis runs its own profile here and the link streams the positions to the controller with
//! `Command::Setpoint`, one per setpoint interval, as a PLC drives servo drives over a fieldbus. So axes move
//! independently and at the same time, MC_Stop brakes in the middle of a move, and a new move takes over from
//! the one under way at the position and speed the axis has.
//!
//! The profile is a trapezoid: velocity, acceleration and deceleration are kept, jerk is not. The controller
//! interpolates linearly between the streamed setpoints, so within one interval the speed is constant anyway.
//! Parameters above the axis limits are cut to the limits.

use opcua::types::Variant;
use sim_core::AxisInfo;

/// PLCopen axis states, numbered as in TwinCAT's `MC_AxisStates`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum State {
    Disabled = 1,
    Standstill = 2,
    ErrorStop = 3,
    Stopping = 4,
    Homing = 5,
    DiscreteMotion = 6,
}

/// The function blocks with an `Execute` input, in this order, and their numeric inputs.
pub const FBS: [&str; 5] = ["Home", "MoveAbsolute", "MoveRelative", "Stop", "Reset"];
const HOME: usize = 0;
const MOVE_ABSOLUTE: usize = 1;
const STOP: usize = 3;
const RESET: usize = 4;
const PARAMS: [&[&str]; 5] = [
    &[],
    &["Position", "Velocity", "Acceleration", "Deceleration", "Jerk"],
    &["Distance", "Velocity", "Acceleration", "Deceleration", "Jerk"],
    &["Deceleration"],
    &[],
];

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

/// A function block's inputs and outputs. `Active` reads the same as `Busy`: there is no buffering.
#[derive(Debug, Clone, Default)]
pub struct Fb {
    pub execute: bool,
    pub done: bool,
    pub busy: bool,
    pub aborted: bool,
    pub error: bool,
    pub error_id: u16,
    pub params: [f64; 5],
}

#[derive(Debug, Clone)]
struct Motion {
    fb: usize,
    /// `None` brakes to a stop.
    target: Option<f64>,
    vel: f64,
    acc: f64,
    dec: f64,
}

#[derive(Debug, Clone)]
pub struct Axis {
    info: AxisInfo,
    home: f64,
    /// MC_Power.Enable.
    pub enable: bool,
    pub fbs: [Fb; 5],
    pub homed: bool,
    /// The axis error; not 0 is ErrorStop.
    pub error_id: u16,
    /// The setpoint streamed to the controller, and its speed.
    pub pos: f64,
    pub vel: f64,
    motion: Option<Motion>,
    /// MC_Stop has been executed and its Execute is still held, or it still brakes.
    stopping: bool,
}

impl Axis {
    /// An axis standing at `home`, the position MC_Home moves to. The numeric inputs start at the axis limits.
    pub fn new(info: &AxisInfo, home: f64) -> Self {
        let mut fbs: [Fb; 5] = Default::default();
        for (fb, params) in fbs.iter_mut().zip(PARAMS) {
            for (value, name) in fb.params.iter_mut().zip(params) {
                *value = match *name {
                    "Velocity" => info.max_vel,
                    "Acceleration" | "Deceleration" => info.max_acc,
                    "Jerk" => info.max_jerk,
                    "Position" => home,
                    _ => 0.0,
                };
            }
        }
        Self { info: info.clone(), home, enable: false, fbs, homed: false, error_id: 0, pos: home, vel: 0.0, motion: None, stopping: false }
    }

    pub fn state(&self) -> State {
        match &self.motion {
            _ if self.error_id != 0 => State::ErrorStop,
            _ if !self.enable => State::Disabled,
            Some(m) if m.fb == HOME => State::Homing,
            Some(m) if m.fb != STOP => State::DiscreteMotion,
            _ if self.stopping => State::Stopping,
            _ => State::Standstill,
        }
    }

    pub fn moving(&self) -> bool {
        self.motion.is_some()
    }

    /// The axis's variables as (name, value, writable), always in the same order. `drives_on` says whether the
    /// controller's drives are on, `actual` is the simulated axis's position and velocity.
    pub fn variables(&self, drives_on: bool, actual: (f64, f64)) -> Vec<(String, Variant, bool)> {
        let status = self.enable && drives_on;
        let error = self.enable && self.error_id != 0;
        let mut out: Vec<(String, Variant, bool)> = vec![
            ("Power.Enable".into(), self.enable.into(), true),
            ("Power.Status".into(), status.into(), false),
            ("Power.Valid".into(), (status && self.error_id == 0).into(), false),
            ("Power.Error".into(), error.into(), false),
            ("Power.ErrorID".into(), (if error { self.error_id } else { 0 }).into(), false),
        ];
        for ((name, fb), params) in FBS.iter().zip(&self.fbs).zip(PARAMS) {
            out.push((format!("{name}.Execute"), fb.execute.into(), true));
            for (param, value) in params.iter().zip(fb.params) {
                out.push((format!("{name}.{param}"), value.into(), true));
            }
            for (output, value) in [("Done", fb.done), ("Busy", fb.busy), ("Active", fb.busy), ("CommandAborted", fb.aborted), ("Error", fb.error)] {
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
        if let Some(m) = self.motion.take() {
            let fb = &mut self.fbs[m.fb];
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
    pub fn execute(&mut self, k: usize, on: bool, setpoint: f64, busy: bool) {
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
            State::DiscreteMotion if k == HOME => ERR_STATE,
            _ if busy => ERR_BUSY,
            _ => 0,
        };
        let p = self.fbs[k].params;
        let a = &self.info;
        // `min` would turn NaN into the limit: what is not a number stays one, to be refused below.
        let cut = |v: f64, max: f64| if v.is_finite() { v.min(max) } else { f64::NAN };
        let (target, vel, acc, dec) = match k {
            HOME => (Some(self.home), 0.5 * a.max_vel, 0.5 * a.max_acc, 0.5 * a.max_acc),
            STOP => (None, a.max_vel, a.max_acc, cut(p[0], a.max_acc)),
            _ => {
                let start = if self.moving() { self.pos } else { setpoint };
                (Some(if k == MOVE_ABSOLUTE { p[0] } else { start + p[0] }), cut(p[1], a.max_vel), cut(p[2], a.max_acc), cut(p[3], a.max_acc))
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
            let fb = &mut self.fbs[k];
            (fb.error, fb.error_id) = (true, refuse);
            return;
        }
        // A new motion takes over the one under way at its position and speed.
        let (pos, vel_now) = if self.moving() { (self.pos, self.vel) } else { (setpoint, 0.0) };
        self.end_motion(0);
        (self.pos, self.vel, self.stopping) = (pos, vel_now, k == STOP);
        // A block started again while busy is not aborted by itself.
        (self.fbs[k].busy, self.fbs[k].aborted) = (true, false);
        self.motion = Some(Motion { fb: k, target, vel, acc, dec });
    }

    /// One setpoint interval `t` of the profile. Without a motion the axis follows `setpoint`, where the controller's
    /// setpoint stands, so that the link's other commands and other axes' moves do not leave it behind.
    pub fn advance(&mut self, t: f64, setpoint: f64) {
        let Some(m) = &self.motion else {
            (self.pos, self.vel) = (setpoint, 0.0);
            return;
        };
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
            (self.fbs[k].busy, self.fbs[k].done) = (false, true);
            self.homed |= k == HOME;
            self.stopping = k == STOP && self.fbs[k].execute;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn axis() -> Axis {
        let info = AxisInfo { id: "x", key: "", unit: "m", min: 0.0, max: 1.0, max_vel: 1.0, max_acc: 10.0, max_jerk: 100.0 };
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
        for (fb, name) in [(MOVE_ABSOLUTE, "Velocity"), (MOVE_ABSOLUTE, "Acceleration"), (2, "Deceleration"), (STOP, "Deceleration")] {
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
        assert!(a.vel <= 0.0 && a.vel >= -1.0 * 0.01 - 1e-12, "{} m/s after one step", a.vel);
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
        assert!(!a.moving() && a.fbs[2].done && !a.fbs[2].aborted && (a.pos - at - 0.3).abs() < 1e-12, "{} from {at}, {:?}", a.pos, a.fbs[2]);
    }
}
