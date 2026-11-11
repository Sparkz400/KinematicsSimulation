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

/// The machine's default config with one more cycle segment: a pass through the middle of the
/// workspace without stopping. It gives every machine's cycle a corner that blending can round.
fn with_via_point(entry: &MachineEntry) -> Config {
    let cfg = entry.config();
    let machine = (entry.build)(&cfg).unwrap();
    let middle: Vec<String> = machine.axes().iter().map(|a| format!("{:?}", 0.5 * (a.min + a.max))).collect();
    let gripper = cfg.array("cycle.segment").unwrap().last().unwrap().get("payload").unwrap().as_bool().unwrap();
    let segment = format!("\n[[cycle.segment]]\ntarget = [{}]\ndwell_s = 0.0\npayload = {gripper}\n", middle.join(", "));
    Config::parse(&format!("{}{segment}", entry.default_config)).unwrap()
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

/// One switch per effect, and each one does something: flipping it changes the traces of the cycle.
#[test]
fn every_effect_switch_changes_the_traces() {
    for entry in MACHINES {
        // Low pretension, so that accelerating takes a belt section to zero tension and the slack
        // switch matters; a weak drive and a low bus voltage, so that the torque and voltage limits
        // matter; a via point, so that corner blending matters.
        let mut base = with_via_point(entry);
        for e in ["belt.pretension_n = 20.0", "drive.peak_torque_nm = 0.3", "drive.bus_voltage_v = 100.0"] {
            base.set_text(e).unwrap();
        }
        let traces = |cfg: &Config| {
            let mut sim = Sim::new(entry, cfg).unwrap();
            sim.command(Command::StartCycle).unwrap();
            let (mut all, mut sample) = (Vec::new(), Vec::new());
            sim.run(4.0, |s| {
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

/// What one pass of the cycle looks like on the setpoint side.
struct CyclePass {
    duration: f64,
    /// How often every axis setpoint came to rest.
    stops: u32,
    /// Highest setpoint speed and acceleration per axis, relative to the axis limit.
    peak_vel: f64,
    peak_acc: f64,
    /// For every segment that dwells: how close the setpoint came to its target.
    dwell_miss: f64,
}

fn cycle_pass(entry: &MachineEntry, cfg: &Config) -> CyclePass {
    let mut sim = Sim::new(entry, cfg).unwrap();
    let axes = sim.machine.axes().to_vec();
    let segments: Vec<(Vec<f64>, f64)> = cfg
        .array("cycle.segment")
        .unwrap()
        .iter()
        .map(|s| (s["target"].as_array().unwrap().iter().map(|v| v.as_float().unwrap()).collect(), s["dwell_s"].as_float().unwrap()))
        .collect();
    let mut closest = vec![f64::INFINITY; segments.len()];
    sim.command(Command::StartCycle).unwrap();
    let (mut starts, mut last, mut pass) = (Vec::new(), None, CyclePass { duration: 0.0, stops: 0, peak_vel: 0.0, peak_acc: 0.0, dwell_miss: 0.0 });
    let mut moving = false;
    while starts.len() < 3 {
        sim.step();
        assert!(sim.time() < 60.0, "{}: the cycle does not repeat", entry.id);
        let now = sim.control.cycle_segment();
        if now != last && now == Some(0) {
            starts.push(sim.time());
        }
        last = now;
        if starts.len() != 2 {
            continue;
        }
        let c = &sim.control;
        let now_moving = c.set_vel.iter().any(|v| *v != 0.0);
        if moving && !now_moving {
            pass.stops += 1;
        }
        moving = now_moving;
        for (i, axis) in axes.iter().enumerate() {
            pass.peak_vel = pass.peak_vel.max(c.set_vel[i].abs() / axis.max_vel);
            pass.peak_acc = pass.peak_acc.max(c.set_acc[i].abs() / axis.max_acc);
        }
        for (k, (target, _)) in segments.iter().enumerate() {
            let miss = target.iter().zip(&c.set_pos).map(|(t, p)| (t - p).abs()).fold(0.0, f64::max);
            closest[k] = closest[k].min(miss);
        }
    }
    pass.duration = starts[2] - starts[1];
    pass.dwell_miss = segments.iter().zip(&closest).filter(|((_, dwell), _)| *dwell > 0.0).map(|(_, c)| *c).fold(0.0, f64::max);
    pass
}

/// Corner blending rounds the corners where the machine need not stop: the cycle gets shorter,
/// the axes stop less often, and the places where it dwells are still reached exactly.
#[test]
fn corner_blending_shortens_the_cycle_and_still_reaches_every_dwell() {
    for entry in MACHINES {
        let stopping = with_via_point(entry);
        let mut blending = stopping.clone();
        blending.set_text("effects.corner_blending = true").unwrap();
        let (off, on) = (cycle_pass(entry, &stopping), cycle_pass(entry, &blending));
        assert!(on.duration < off.duration - 0.05, "{}: {} s with blending, {} s without", entry.id, on.duration, off.duration);
        assert!(on.stops < off.stops, "{}: {} stops with blending, {} without", entry.id, on.stops, off.stops);
        assert!(off.dwell_miss < 1e-12 && on.dwell_miss < 1e-12, "{}: a dwell position was missed by {} m", entry.id, on.dwell_miss);
        // A longer lead time than the ramps allow changes nothing more: blending stays on the ramps.
        let mut greedy = blending.clone();
        greedy.set_text("profile.blend_time_s = 10.0").unwrap();
        let greedy = cycle_pass(entry, &greedy);
        assert!(greedy.dwell_miss < 1e-12 && greedy.duration <= on.duration + 1e-9, "{}", entry.id);
    }
}

/// On the machines' own cycles the blended setpoints stay within every axis limit.
#[test]
fn blended_default_cycles_stay_within_the_axis_limits() {
    for entry in MACHINES {
        let mut cfg = entry.config();
        cfg.set_text("effects.corner_blending = true").unwrap();
        cfg.set_text("profile.blend_time_s = 10.0").unwrap();
        let pass = cycle_pass(entry, &cfg);
        assert!(pass.peak_vel <= 1.0 + 1e-9 && pass.peak_acc <= 1.0 + 1e-9, "{}: {} of v max, {} of a max", entry.id, pass.peak_vel, pass.peak_acc);
        assert!(pass.dwell_miss < 1e-12, "{}", entry.id);
    }
}
