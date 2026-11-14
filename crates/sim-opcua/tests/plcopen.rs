//! The PLCopen axis state machine on its own, without a server or a wall clock: every step is one call, so these tests
//! give the same result on any host.

use opcua::types::Variant;
use sim_core::AxisInfo;
use sim_opcua::plcopen::{Axis, ERR_DRIVES_OFF, ERR_PARAMETER, State};

const MOVE_RELATIVE: usize = 2;
const RESET: usize = 4;
const T: f64 = 0.01;

/// An axis from 0 to 1 m at 1 m/s and 10 m/s², standing at 0.1 m, powered.
fn axis() -> Axis {
    let info =
        AxisInfo { id: "x", key: "", unit: "m", min: 0.0, max: 1.0, max_vel: 1.0, max_acc: 10.0, max_jerk: 100.0 };
    let mut axis = Axis::new(&info, 0.1);
    axis.power(true);
    axis
}

/// An output of the axis as a PLC reads it.
fn output(axis: &Axis, name: &str) -> Variant {
    axis.variables(true, (0.0, 0.0)).into_iter().find(|(n, ..)| n == name).unwrap_or_else(|| panic!("{name}")).1
}

/// How many of Done, Busy, CommandAborted and Error a block shows: PLCopen allows one at most.
fn shown(axis: &Axis, k: usize) -> usize {
    let fb = &axis.fbs[k];
    [fb.done, fb.busy, fb.aborted, fb.error].iter().filter(|on| **on).count()
}

/// A block re-triggered while busy with an input it refuses (Velocity 0) used to stay Busy beside Error and report Done
/// with Error when its old motion arrived. Now it shows Error alone from the refusal on, and the axis brakes with the
/// motion's deceleration, 10 m/s², to a stop short of the old target.
///
/// By hand: from rest the speed rises by 10 · 0.01 = 0.1 m/s a step and reaches the 1 m/s limit after 10 steps, having
/// gone (0.1 + 0.2 + … + 1.0) · 0.01 = 0.055 m. Braking from 1 m/s at 10 m/s² takes 1 / 10 = 0.1 s, 10 steps, and
/// 1² / (2 · 10) = 0.05 m (0.045 m on the grid of steps); the axis stops near 0.1 + 0.055 + 0.045 = 0.2 m, not at the
/// old target 0.4 m.
#[test]
fn a_refused_retrigger_shows_error_alone_and_brakes_the_axis_to_a_stop() {
    let mut a = axis();
    a.set_param("MoveRelative.Distance", 0.3);
    a.execute(MOVE_RELATIVE, true, 0.1, false);
    for _ in 0..10 {
        a.advance(T, 0.0);
    }
    assert!((a.vel - 1.0).abs() < 1e-9 && (a.pos - 0.155).abs() < 1e-9, "{} m/s at {} m", a.vel, a.pos);
    a.execute(MOVE_RELATIVE, false, 0.0, false);
    assert!(a.fbs[MOVE_RELATIVE].busy, "a block runs on when Execute falls while busy");
    a.set_param("MoveRelative.Velocity", 0.0);
    a.execute(MOVE_RELATIVE, true, 0.0, false);
    let fb = &a.fbs[MOVE_RELATIVE];
    assert!(fb.error && fb.error_id == ERR_PARAMETER && shown(&a, MOVE_RELATIVE) == 1, "{fb:?}");
    let mut steps = 0;
    while a.moving() {
        let before = a.vel;
        a.advance(T, 0.0);
        steps += 1;
        assert!(a.vel <= before && before - a.vel <= 10.0 * T + 1e-9, "{before} -> {} m/s", a.vel);
        assert_eq!(shown(&a, MOVE_RELATIVE), 1, "{:?}", a.fbs[MOVE_RELATIVE]);
        assert!(steps <= 11, "still moving after {steps} steps");
    }
    // 10 steps, or 11 where rounding leaves a speed residue of some 1e-16 m/s.
    assert!((10..=11).contains(&steps), "{steps} steps");
    assert!((a.pos - 0.2).abs() < 1e-9, "stopped at {} m", a.pos);
    assert!(a.fbs[MOVE_RELATIVE].error && !a.fbs[MOVE_RELATIVE].done);
    assert_eq!(a.state(), State::Standstill);
    // A falling Execute clears the error; a corrected re-trigger moves again.
    a.execute(MOVE_RELATIVE, false, a.pos, false);
    assert_eq!(shown(&a, MOVE_RELATIVE), 0);
    a.set_param("MoveRelative.Velocity", 0.5);
    a.execute(MOVE_RELATIVE, true, a.pos, false);
    assert!(a.fbs[MOVE_RELATIVE].busy && shown(&a, MOVE_RELATIVE) == 1);
}

/// While the refused block's motion brakes, the axis is in DiscreteMotion and another move takes it over at speed.
#[test]
fn the_braking_after_a_refused_retrigger_can_be_taken_over() {
    let mut a = axis();
    a.set_param("MoveRelative.Distance", 0.3);
    a.execute(MOVE_RELATIVE, true, 0.1, false);
    for _ in 0..10 {
        a.advance(T, 0.0);
    }
    a.execute(MOVE_RELATIVE, false, 0.0, false);
    a.set_param("MoveRelative.Velocity", f64::NAN);
    a.execute(MOVE_RELATIVE, true, 0.0, false);
    a.advance(T, 0.0);
    assert_eq!(a.state(), State::DiscreteMotion);
    let v = a.vel;
    a.set_param("MoveAbsolute.Position", 0.6);
    a.execute(1, true, 0.0, false);
    assert!(a.fbs[1].busy && a.vel == v && !a.fbs[MOVE_RELATIVE].aborted, "{:?}", a.fbs[MOVE_RELATIVE]);
    while a.moving() {
        a.advance(T, 0.0);
    }
    assert!(a.fbs[1].done && (a.pos - 0.6).abs() < 1e-12 && shown(&a, MOVE_RELATIVE) == 1);
}

/// PLCopen's state diagram: an axis error holds through MC_Power off (ErrorStop, Power.Error with its ErrorID), and
/// MC_Reset leads to Disabled when the axis is not powered, to Standstill when it is. State, StateID and MC_Power's
/// outputs agree at every step.
#[test]
fn error_stop_holds_through_power_off_and_reset_leads_to_disabled_or_standstill() {
    let check = |a: &Axis, state: State, error: u16| {
        assert_eq!(a.state(), state);
        assert_eq!(output(a, "StateID"), Variant::Int32(state as i32));
        assert_eq!(output(a, "State"), Variant::from(format!("{state:?}")));
        assert_eq!(output(a, "Power.Error"), Variant::Boolean(error != 0), "{state:?}");
        assert_eq!(output(a, "Power.ErrorID"), Variant::UInt16(error), "{state:?}");
        assert_eq!(output(a, "ErrorID"), Variant::UInt16(error), "{state:?}");
        assert_eq!(output(a, "Power.Valid"), Variant::Boolean(a.enable && error == 0), "{state:?}");
    };
    for powered_at_reset in [false, true] {
        let mut a = axis();
        check(&a, State::Standstill, 0);
        a.fault(ERR_DRIVES_OFF);
        check(&a, State::ErrorStop, ERR_DRIVES_OFF);
        a.power(false);
        check(&a, State::ErrorStop, ERR_DRIVES_OFF);
        a.power(powered_at_reset);
        a.execute(RESET, true, 0.1, false);
        assert!(a.fbs[RESET].done);
        check(&a, if powered_at_reset { State::Standstill } else { State::Disabled }, 0);
        a.power(true);
        check(&a, State::Standstill, 0);
    }
}
