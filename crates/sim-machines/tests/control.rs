//! Control and profiles on every registered machine, through the same controller:
//! following error, feedforward, the pick-and-place cycle, limits, determinism, convergence.

use sim_components::{Command, Sim, check_limits};
use sim_core::{Config, MachineEntry};
use sim_machines::MACHINES;

fn config(entry: &MachineEntry, edit: &[&str]) -> Config {
    let mut cfg = entry.config();
    for e in edit {
        cfg.set_text(e).unwrap();
    }
    cfg
}

/// A long move of the first axis, the others staying where they start.
fn long_move(sim: &Sim) -> Vec<f64> {
    let axis = &sim.machine.axes()[0];
    let mut target = sim.control.set_pos.clone();
    target[0] = axis.min + 0.9 * (axis.max - axis.min);
    target
}

/// Mean following error of the first axis while it cruises at its speed limit.
fn cruise_following_error(entry: &MachineEntry, edit: &[&str]) -> (f64, f64) {
    let cfg = config(entry, edit);
    let mut sim = Sim::new(entry, &cfg).unwrap();
    let v_max = sim.machine.axes()[0].max_vel;
    sim.command(Command::MoveTo(long_move(&sim))).unwrap();
    let (mut sum, mut n, mut cruise_since) = (0.0, 0u32, None);
    while sim.control.busy() || sim.step == 0 {
        sim.step();
        if sim.control.set_vel[0] < v_max * (1.0 - 1e-9) {
            cruise_since = None;
            continue;
        }
        // Let the loops settle into the cruise before measuring.
        let since = *cruise_since.get_or_insert(sim.time());
        if sim.time() - since > 0.15 {
            sum += sim.control.tool_error[0];
            n += 1;
        }
    }
    assert!(n > 1000, "{}: the move has no long cruise to measure on", entry.id);
    (sum / n as f64, v_max / cfg.f64("drive.kv_1_s").unwrap())
}

/// Following error at constant speed, no feedforward: equals v / Kv.
#[test]
fn following_error_at_constant_speed_is_v_over_kv() {
    for entry in MACHINES {
        let off = ["effects.velocity_feedforward = false", "effects.torque_feedforward = false"];
        let (measured, expected) = cruise_following_error(entry, &off);
        assert!((measured - expected).abs() < 0.01 * expected, "{}: {measured} m, v/Kv = {expected} m", entry.id);
        // It scales with 1 / Kv.
        let (doubled, expected) = cruise_following_error(entry, &[off[0], off[1], "drive.kv_1_s = 100.0"]);
        assert!((doubled - expected).abs() < 0.01 * expected, "{}: {doubled} m, v/Kv = {expected} m", entry.id);
    }
}

#[test]
fn velocity_feedforward_removes_the_following_error() {
    for entry in MACHINES {
        let (measured, v_over_kv) = cruise_following_error(entry, &[]);
        assert!(measured.abs() < 0.02 * v_over_kv, "{}: {measured} m left of {v_over_kv} m", entry.id);
        // The weighting factor takes away its share.
        let (half, _) = cruise_following_error(entry, &["drive.velocity_feedforward_factor = 0.5"]);
        assert!((half - 0.5 * v_over_kv).abs() < 0.02 * v_over_kv, "{}: {half} m", entry.id);
    }
}

#[test]
fn torque_feedforward_cuts_the_following_error_while_accelerating() {
    for entry in MACHINES {
        let peak = |edit: &[&str]| {
            let mut sim = Sim::new(entry, &config(entry, edit)).unwrap();
            sim.command(Command::MoveTo(long_move(&sim))).unwrap();
            let mut peak = 0f64;
            sim.run(0.15, |s| peak = peak.max(s.control.tool_error[0].abs()));
            peak
        };
        let (with, without) = (peak(&[]), peak(&["effects.torque_feedforward = false"]));
        // Friction and ripple are not in the feedforward model, so part of the error stays.
        assert!(with < 0.75 * without, "{}: {with} m with, {without} m without", entry.id);
    }
}

#[test]
fn a_move_arrives_and_comes_to_rest() {
    for entry in MACHINES {
        let mut sim = Sim::new(entry, &entry.config()).unwrap();
        let target = long_move(&sim);
        sim.command(Command::MoveTo(target.clone())).unwrap();
        while sim.control.busy() || sim.step == 0 {
            sim.step();
        }
        sim.run(0.3, |_| {});
        let position = sim.signal_index(&format!("axis.{}.position", sim.machine.axes()[0].id)).unwrap();
        let (mut low, mut high, mut sample) = (f64::INFINITY, f64::NEG_INFINITY, Vec::new());
        sim.run(0.2, |s| {
            s.sample(&mut sample);
            low = low.min(sample[position]);
            high = high.max(sample[position]);
        });
        // The drive controls the motor; belt stretch and friction leave a small standstill error.
        assert!((0.5 * (low + high) - target[0]).abs() < 1e-4, "{}: rests at {low}..{high}, target {}", entry.id, target[0]);
        assert!(high - low < 2e-5, "{}: still moving by {} m at rest", entry.id, high - low);
    }
}

#[test]
fn the_cycle_visits_every_segment_and_switches_the_payload() {
    for entry in MACHINES {
        let cfg = entry.config();
        let mut sim = Sim::new(entry, &cfg).unwrap();
        let segments = cfg.array("cycle.segment").unwrap().len();
        sim.command(Command::StartCycle).unwrap();
        let payload = sim.signal_index("payload").unwrap();
        let (mut visited, mut payload_seen, mut worst_error) = (vec![0u32; segments], [false; 2], 0f64);
        let (mut last, mut sample) = (None, Vec::new());
        sim.run(8.0, |s| {
            let now = s.control.cycle_segment();
            if now != last {
                if let Some(i) = now {
                    visited[i] += 1;
                }
                last = now;
            }
            s.sample(&mut sample);
            payload_seen[sample[payload] as usize] = true;
            worst_error = s.control.tool_error.iter().fold(worst_error, |w, e| w.max(e.abs()));
        });
        // Every segment ran, and the cycle started over.
        assert!(visited.iter().all(|n| *n >= 1) && visited[0] >= 2, "{}: segments run {visited:?} times in 8 s", entry.id);
        assert!(payload_seen == [true, true], "{}: the payload never switched", entry.id);
        assert!(worst_error < 5e-4, "{}: following error up to {worst_error} m in the cycle", entry.id);
        // Stop ends the cycle after the segment under way.
        sim.command(Command::Stop).unwrap();
        sim.run(3.0, |_| {});
        assert!(!sim.control.busy(), "{}: the cycle did not stop", entry.id);
    }
}

#[test]
fn bad_commands_are_refused() {
    for entry in MACHINES {
        let mut sim = Sim::new(entry, &entry.config()).unwrap();
        let mut outside = long_move(&sim);
        outside[0] = sim.machine.axes()[0].max + 0.01;
        assert_eq!(sim.command(Command::MoveTo(outside)).unwrap_err().key, "error-target-out-of-range");
        assert_eq!(sim.command(Command::MoveTo(vec![])).unwrap_err().key, "error-target-out-of-range");
        sim.command(Command::MoveTo(long_move(&sim))).unwrap();
        assert_eq!(sim.command(Command::MoveTo(long_move(&sim))).unwrap_err().key, "error-busy");
        assert_eq!(sim.command(Command::StartCycle).unwrap_err().key, "error-busy");
    }
}

/// External setpoints are interpolated over the setpoint interval, so they never arrive as a step.
#[test]
fn external_setpoints_are_interpolated() {
    for entry in MACHINES {
        let mut sim = Sim::new(entry, &entry.config()).unwrap();
        let start = sim.control.set_pos.clone();
        let mut target = start.clone();
        target[0] += 0.002;
        sim.command(Command::Setpoint(target.clone())).unwrap();
        let (mut biggest_jump, mut last) = (0f64, start[0]);
        sim.run(0.02, |s| {
            biggest_jump = biggest_jump.max((s.control.set_pos[0] - last).abs());
            last = s.control.set_pos[0];
        });
        // 2 mm over 10 ms at a 1 ms profile cycle: ten steps of 0.2 mm.
        assert!((biggest_jump - 0.0002).abs() < 1e-9, "{}: setpoint jumped by {biggest_jump} m", entry.id);
        assert!((sim.control.set_pos[0] - target[0]).abs() < 1e-12 && !sim.control.busy());
    }
}

/// A diagonal move loads one motor with both axes: the check catches limits the drive cannot meet.
#[test]
fn axis_limits_are_checked_against_motor_limits() {
    for entry in MACHINES {
        let violations = |edit: &[&str]| {
            let cfg = config(entry, edit);
            check_limits((entry.build)(&cfg).unwrap().as_ref(), &cfg).unwrap()
        };
        assert!(violations(&[]).is_empty(), "{}: the default config violates its own limits", entry.id);
        // A drive with 10 % headroom on the fastest move stops being enough when every axis runs 20 % faster.
        let sim = Sim::new(entry, &entry.config()).unwrap();
        let mut faster: Vec<String> = Vec::new();
        for axis in sim.machine.axes() {
            faster.push(format!("axis.{}.max_velocity_m_s = {:?}", axis.id, axis.max_vel * 1.5));
        }
        let faster: Vec<&str> = faster.iter().map(String::as_str).collect();
        assert!(violations(&faster).iter().any(|v| v.key == "limit-motor-speed"), "{}", entry.id);
        assert!(violations(&["drive.peak_torque_nm = 0.2"]).iter().any(|v| v.key == "limit-motor-torque"), "{}", entry.id);
    }
}

/// Determinism: the same config and the same commands give bit-identical traces.
#[test]
fn the_same_run_twice_is_bit_identical() {
    for entry in MACHINES {
        let trace = || {
            let mut sim = Sim::new(entry, &entry.config()).unwrap();
            sim.command(Command::StartCycle).unwrap();
            let (mut bits, mut sample) = (Vec::new(), Vec::new());
            sim.run(0.5, |s| {
                s.sample(&mut sample);
                bits.extend(sample.iter().map(|v| v.to_bits()));
            });
            bits
        };
        assert!(trace() == trace(), "{}: two identical runs differ", entry.id);
    }
}

/// Step-size convergence with control and every effect on: halving the integration step changes
/// the results by less than 1 %.
#[test]
fn halving_the_integration_step_changes_a_controlled_move_by_less_than_one_percent() {
    for entry in MACHINES {
        let run = |substeps: u32| {
            let cfg = config(entry, &[&format!("sim.substeps = {substeps}"), "effects.velocity_feedforward = false"]);
            let mut sim = Sim::new(entry, &cfg).unwrap();
            let start = sim.control.set_pos[0];
            sim.command(Command::MoveTo(long_move(&sim))).unwrap();
            let torque = sim.signal_index(&format!("motor.{}.torque", sim.machine.motors()[0].id)).unwrap();
            let (mut peak_torque, mut sum_sq, mut peak_error, mut n, mut sample) = (0f64, 0.0, 0f64, 0u32, Vec::new());
            sim.run(1.0, |s| {
                s.sample(&mut sample);
                peak_torque = peak_torque.max(sample[torque].abs());
                sum_sq += sample[torque] * sample[torque];
                peak_error = peak_error.max(s.control.tool_error[0].abs());
                n += 1;
            });
            let mut pos = vec![0.0; sim.machine.axes().len()];
            sim.machine.tool_state(&sim.state, &mut pos, &mut vec![0.0; sim.machine.axes().len()]);
            [pos[0] - start, peak_torque, (sum_sq / n as f64).sqrt(), peak_error]
        };
        let (coarse, fine) = (run(4), run(8));
        for (c, f) in coarse.iter().zip(&fine) {
            assert!((c - f).abs() < 0.01 * f.abs(), "{}: {coarse:?} at 4 sub-steps, {fine:?} at 8", entry.id);
        }
    }
}

#[test]
fn loop_cycles_must_be_whole_multiples_of_the_base_step() {
    for entry in MACHINES {
        let cfg = config(entry, &["drive.velocity_cycle_s = 100.0e-6"]);
        assert!(Sim::new(entry, &cfg).is_err(), "{}", entry.id);
    }
}

/// One switch per effect, and each one does something: flipping it changes the traces of a move.
#[test]
fn every_effect_switch_changes_the_traces() {
    for entry in MACHINES {
        // Low pretension, so that accelerating takes a belt section to zero tension and the slack
        // switch matters; a weak drive, so that the torque limit matters.
        let base = config(entry, &["belt.pretension_n = 20.0", "drive.peak_torque_nm = 0.3"]);
        let traces = |cfg: &Config| {
            let mut sim = Sim::new(entry, cfg).unwrap();
            sim.command(Command::MoveTo(long_move(&sim))).unwrap();
            let (mut all, mut sample) = (Vec::new(), Vec::new());
            sim.run(0.4, |s| {
                if s.step % 16 == 0 {
                    s.sample(&mut sample);
                    all.extend(sample.iter().map(|v| v.to_bits()));
                }
            });
            all
        };
        let reference = traces(&base);
        let effects: Vec<String> = base.leaves().into_iter().map(|(p, _)| p).filter(|p| p.starts_with("effects.")).collect();
        assert!(effects.len() >= 20, "{}: only {} effect switches", entry.id, effects.len());
        for effect in effects {
            let mut flipped = base.clone();
            flipped.set_text(&format!("{effect} = {}", !base.bool(&effect).unwrap())).unwrap();
            assert!(traces(&flipped) != reference, "{}: flipping {effect} changed nothing", entry.id);
        }
    }
}
