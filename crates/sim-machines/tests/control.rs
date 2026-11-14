//! Control and profiles on every registered machine, through the same controller:
//! following error, feedforward, the pick-and-place cycle, limits, determinism, convergence.

use sim_components::{Command, Sim, check_limits};
use sim_core::{Config, MachineEntry};
use sim_machines::MACHINES;

/// The switch between a machine's elastic model and its rigid reference model: belt stretch on a belt machine, the
/// gearboxes' elasticity on one of arms.
fn model_switch(cfg: &Config) -> &'static str {
    ["effects.belt_stretch", "effects.joint_elasticity"]
        .into_iter()
        .find(|key| cfg.bool(key).is_ok())
        .expect("a machine with an elastic and a rigid model")
}

/// `check` on every machine, each on a thread of its own named after the machine (a failure says "thread 'scara'
/// panicked"): the tests that went through the machines one after the other took up to a minute alone.
fn on_every_machine(check: impl Fn(&'static MachineEntry) + Sync) {
    let check = &check;
    std::thread::scope(|scope| {
        for entry in MACHINES.iter().copied() {
            std::thread::Builder::new().name(entry.id.into()).spawn_scoped(scope, move || check(entry)).unwrap();
        }
    });
}

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
    let segment =
        format!("\n[[cycle.segment]]\ntarget = [{}]\ndwell_s = 0.0\npayload = {gripper}\n", middle.join(", "));
    Config::parse(&format!("{}{segment}", entry.default_config)).unwrap()
}

/// A long move of the first axis, the others staying where they start.
fn long_move(sim: &Sim) -> Vec<f64> {
    let axis = &sim.machine.axes()[0];
    let mut target = sim.control.set_pos.clone();
    target[0] = axis.min + 0.9 * (axis.max - axis.min);
    target
}

/// Moves the first axis to near the low end of its range (5 % of it in), the others staying, and waits there: so that
/// the long move that follows has a long cruise also where the start pose is in the middle of the range (the delta's).
/// The gantries start near that end already: they move 50 mm, from 0.1 m to 0.05 m.
fn to_low_end(sim: &mut Sim) {
    let axis = sim.machine.axes()[0].clone();
    let mut low = sim.control.set_pos.clone();
    low[0] = low[0].min(axis.min + 0.05 * (axis.max - axis.min));
    sim.command(Command::MoveTo(low)).unwrap();
    while sim.control.busy() {
        sim.step();
    }
}

/// The config with the first axis's speed limit lowered, where its stroke is too short, to one at which the long move
/// from the low end (85 % of the range) cruises for `cruise` seconds. At 3 m/s, 50 m/s2 and 2500 m/s3 a cruise of 0.25 s
/// takes a move of 0.99 m, 85 % of 1.16 m: the D3's range of 1.2 m has it, the D2's 0.8 m (0.147 s of cruise) and the
/// linear delta's 0.95 m (0.19 s) have not, and a test on a cruise needs it at a lower speed there. The S-curve's cruise
/// d / v - a / j - v / a solved for v, with ramps that reach the acceleration limit; where they do not, they are shorter
/// and the cruise longer.
fn cruising(entry: &MachineEntry, mut cfg: Config, cruise: f64) -> Config {
    let axis = (entry.build)(&cfg).unwrap().axes()[0].clone();
    let (d, a, lead) = (0.85 * (axis.max - axis.min), axis.max_acc, cruise + axis.max_acc / axis.max_jerk);
    let v = 0.5 * a * ((lead * lead + 4.0 * d / a).sqrt() - lead);
    if v < axis.max_vel {
        cfg.set_text(&format!("axis.{}.max_velocity_{}_s = {v:?}", axis.id, axis.unit)).unwrap();
    }
    cfg
}

/// Mean following error of the first axis while it cruises at its speed limit, and what each motor's position loop makes
/// of it at the tool: forward(motor setpoints) - forward(motor setpoints through a first-order lag of 1 / Kv, as a
/// position loop on an ideal velocity loop follows them), which is v / Kv where the kinematics are linear (the gantries)
/// and a little more or less where they are not: the delta's lag of 60 mm at 3 m/s spans a curved part of its
/// kinematics, and its motors accelerate while the tool cruises. That lag's first term alone, forward(motor setpoints -
/// motor speeds / Kv), was 2.2 % short of the D2's following error at Kv = 50, whose motors accelerate more in its cruise
/// across 0.68 m; the whole lag is within 0.13 % of it.
fn cruise_following_error(entry: &MachineEntry, edit: &[&str]) -> (f64, f64) {
    // 0.15 s to settle and 1600 base steps to measure on.
    let cfg = cruising(entry, config(entry, edit), 0.25);
    let mut sim = Sim::new(entry, &cfg).unwrap();
    let v_max = sim.machine.axes()[0].max_vel;
    let kv = cfg.f64("drive.kv_1_s").unwrap();
    to_low_end(&mut sim);
    sim.command(Command::MoveTo(long_move(&sim))).unwrap();
    let (mut sum, mut n, mut cruise_since, mut lag) = (0.0, 0u32, None, 0.0);
    let (mut set, mut behind) = (vec![0.0; sim.machine.axes().len()], vec![0.0; sim.machine.axes().len()]);
    let mut lagging: Vec<f64> = sim.control.drives.iter().map(|d| d.pos_set - d.following_error).collect();
    while sim.control.busy() || sim.step == 0 {
        sim.step();
        for (m, d) in lagging.iter_mut().zip(&sim.control.drives) {
            *m += sim.dt * kv * (d.pos_set - *m);
        }
        if sim.control.set_vel[0] < v_max * (1.0 - 1e-9) {
            cruise_since = None;
            continue;
        }
        // Let the loops settle into the cruise before measuring.
        let since = *cruise_since.get_or_insert(sim.time());
        if sim.time() - since > 0.15 {
            sum += sim.control.tool_error[0];
            n += 1;
            let motors: Vec<f64> = sim.control.drives.iter().map(|d| d.pos_set).collect();
            sim.machine.forward(&motors, &mut set);
            sim.machine.forward(&lagging, &mut behind);
            lag += set[0] - behind[0];
        }
    }
    assert!(n > 1000, "{}: the move has no long cruise to measure on", entry.id);
    println!("{}: lag at the tool {} m, v / Kv {} m", entry.id, lag / n as f64, v_max / kv);
    (sum / n as f64, lag / n as f64)
}

/// Following error at constant speed, no feedforward: equals v / Kv.
#[test]
fn following_error_at_constant_speed_is_v_over_kv() {
    for entry in MACHINES {
        let off = ["effects.velocity_feedforward = false", "effects.torque_feedforward = false"];
        // Within 1 %, and what the gears sag beyond the default slack of 0.1 mm (the delta's with 9 arcmin backlash: up
        // to 2.2 mm more; 0.52 mm seen at Kv = 100), which the tool's error carries along.
        let sag = ((entry.build)(&entry.config()).unwrap().setpoint_slack() - 1e-4).max(0.0);
        let (measured, expected) = cruise_following_error(entry, &off);
        assert!((measured - expected).abs() < 0.01 * expected + sag, "{}: {measured} m, v/Kv = {expected} m", entry.id);
        // It scales with 1 / Kv.
        let (doubled, expected) = cruise_following_error(entry, &[off[0], off[1], "drive.kv_1_s = 100.0"]);
        assert!((doubled - expected).abs() < 0.01 * expected + sag, "{}: {doubled} m, v/Kv = {expected} m", entry.id);
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
            sim.command(Command::MoveTo(long_move(&sim))).unwrap_or_else(|e| panic!("{}: {}", entry.id, e.key));
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
        sim.command(Command::MoveTo(target.clone())).unwrap_or_else(|e| panic!("{}: {}", entry.id, e.key));
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
        // The drive controls the motor; belt stretch, gear twist, backlash and friction leave a standstill error, up to
        // the machine's setpoint slack (the delta's 9 arcmin: 0.21 mm here).
        assert!(
            (0.5 * (low + high) - target[0]).abs() < sim.machine.setpoint_slack().max(1e-4),
            "{}: rests at {low}..{high}, target {}",
            entry.id,
            target[0]
        );
        assert!(high - low < 2e-5, "{}: still moving by {} m at rest", entry.id, high - low);
    }
}

#[test]
fn the_cycle_visits_every_segment_and_switches_the_payload() {
    on_every_machine(|entry| {
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
        assert!(
            visited.iter().all(|n| *n >= 1) && visited[0] >= 2,
            "{}: segments run {visited:?} times in 8 s",
            entry.id
        );
        assert!(payload_seen == [true, true], "{}: the payload never switched", entry.id);
        // 0.5 mm, and on a fast machine 0.03 mm per m/s2 of its fastest axis: the delta (50 m/s2, 1.5 mm) has up to
        // 1.23 mm at the motors in its Adept cycle, from the gearboxes' springs that the rigid feedforward leaves out.
        let fastest = sim.machine.axes().iter().map(|a| a.max_acc).fold(0.0, f64::max);
        let limit = f64::max(5e-4, 3e-5 * fastest);
        assert!(
            worst_error < limit,
            "{}: following error up to {worst_error} m in the cycle, limit {limit} m",
            entry.id
        );
        // Stop ends the cycle after the segment under way.
        sim.command(Command::Stop).unwrap();
        sim.run(3.0, |_| {});
        assert!(!sim.control.busy(), "{}: the cycle did not stop", entry.id);
    });
}

#[test]
fn bad_commands_are_refused() {
    for entry in MACHINES {
        let mut sim = Sim::new(entry, &entry.config()).unwrap();
        let mut outside = long_move(&sim);
        outside[0] = sim.machine.axes()[0].max + 0.01;
        assert_eq!(sim.command(Command::MoveTo(outside)).unwrap_err().key, "error-target-out-of-range");
        assert_eq!(sim.command(Command::MoveTo(vec![])).unwrap_err().key, "error-target-out-of-range");
        sim.command(Command::MoveTo(long_move(&sim))).unwrap_or_else(|e| panic!("{}: {}", entry.id, e.key));
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
        sim.command(Command::Setpoint(target.clone())).unwrap_or_else(|e| panic!("{}: {}", entry.id, e.key));
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
            faster.push(format!("axis.{}.max_velocity_{}_s = {:?}", axis.id, axis.unit, axis.max_vel * 1.5));
        }
        let faster: Vec<&str> = faster.iter().map(String::as_str).collect();
        assert!(violations(&faster).iter().any(|v| v.key == "limit-motor-speed"), "{}", entry.id);
        assert!(
            violations(&["drive.peak_torque_nm = 0.2"]).iter().any(|v| v.key == "limit-motor-torque"),
            "{}",
            entry.id
        );
    }
}

/// Determinism: the same config and the same commands give bit-identical traces, also with sensor noise,
/// which is pseudo-random from the seed in the config.
#[test]
fn the_same_run_twice_is_bit_identical() {
    for entry in MACHINES {
        let trace = |edit: &[&str]| {
            let mut sim = Sim::new(entry, &config(entry, edit)).unwrap();
            sim.command(Command::StartCycle).unwrap();
            let (mut bits, mut sample) = (Vec::new(), Vec::new());
            sim.run(0.5, |s| {
                s.sample(&mut sample);
                bits.extend(sample.iter().map(|v| v.to_bits()));
            });
            bits
        };
        assert!(trace(&[]) == trace(&[]), "{}: two identical runs differ", entry.id);
        let noisy = trace(&["effects.sensor_noise = true"]);
        assert!(noisy == trace(&["effects.sensor_noise = true"]), "{}: two identical runs with noise differ", entry.id);
        assert!(
            noisy != trace(&["effects.sensor_noise = true", "drive.noise_seed = 2"]),
            "{}: the seed changes nothing",
            entry.id
        );
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
            sim.command(Command::MoveTo(long_move(&sim))).unwrap_or_else(|e| panic!("{}: {}", entry.id, e.key));
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
    on_every_machine(|entry| {
        // Low pretension, so that accelerating takes a belt section to zero tension and the slack
        // switch matters; a weak drive and a low bus voltage (100 V at the winding), so that the torque and
        // voltage limits matter; a via point, so that corner blending matters.
        let mut base = with_via_point(entry);
        // The pretension where there is a belt. The weak drive keeps half as much again as holding the start pose takes
        // (0.3 N m on the gantries; the delta's arms need 0.52 N m held up, and with less they lie on their stops, where
        // no motor turns fast enough to meet the voltage limit), and at least 0.3 / 3.8 of its peak torque (0.3 N m on
        // the gantries' 3.8): enough to move an axis against its friction (a screw's preload drags with 0.34 N m).
        let hold = {
            let machine = (entry.build)(&base).unwrap();
            let mut torque = vec![0.0; machine.motors().len()];
            machine.equilibrium(&base.f64s("start.pose").unwrap(), &mut Vec::new(), &mut torque);
            torque.iter().fold(0.0f64, |m, t| m.max(t.abs()))
        };
        let peak = format!(
            "drive.peak_torque_nm = {:?}",
            f64::max(0.3, 1.5 * hold).max(0.3 / 3.8 * base.f64("drive.peak_torque_nm").unwrap())
        );
        // The low bus at 81.65 V for a rotary motor; a linear motor's force constant (N/A) is tens of times a rotary
        // motor's torque constant, and its bus is lowered with it (65 N/A: 37.7 V, 0.71 m/s at no load).
        let bus = format!(
            "drive.bus_voltage_v = {:?}",
            81.65 * f64::min(1.0, 30.0 / base.f64("motor.torque_constant_nm_a").unwrap())
        );
        for e in ["belt.pretension_n = 20.0", peak.as_str(), bus.as_str()] {
            if base.get(e.split(' ').next().unwrap()).is_ok() {
                base.set_text(e).unwrap();
            }
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
        // Whether the traces with `cfg` differ from the reference, as `traces(cfg) != reference` says, but it stops at
        // the first sample that differs: most effects show within milliseconds, and only a switch that changes nothing
        // runs the whole 4 s.
        let differs = |cfg: &Config, reference: &[u64]| {
            let mut sim = Sim::new(entry, cfg).unwrap();
            sim.command(Command::StartCycle).unwrap();
            let (mut at, mut sample) = (0, Vec::new());
            for _ in 0..(4.0 / sim.dt).round() as u64 {
                sim.step();
                if sim.step.is_multiple_of(16) {
                    sim.sample(&mut sample);
                    let rest = &reference[at.min(reference.len())..];
                    if sample.len() > rest.len() || sample.iter().zip(rest).any(|(v, r)| v.to_bits() != *r) {
                        return true;
                    }
                    at += sample.len();
                }
            }
            at != reference.len()
        };
        let effects: Vec<String> =
            base.leaves().into_iter().map(|(p, _)| p).filter(|p| p.starts_with("effects.")).collect();
        // Hard stops act only beyond the stroke and holding brakes only while the drives are disabled. A cycle
        // meets neither. The shaper follows the pose only where input shaping is on, which it is not here.
        // A crash needs an obstacle, and the configs have none. The thermal warm start acts on the cycle analysis, not on
        // a run. Their own tests show them working.
        let skipped = [
            "effects.hard_stops",
            "effects.holding_brake",
            "effects.shaper_follows_pose",
            "effects.crash",
            "effects.thermal_warm_start",
        ];
        // A machine with one drive has nothing to share a DC link with.
        let alone = Sim::new(entry, &base).unwrap().machine.motors().len() < 2;
        // An effect whose values are all zero (or all one) in this config has nothing to do (a backlash-free coupling and nut, a
        // linear motor's forcer without bearings), nor gravity where it loads no motor (a table turning about the
        // vertical).
        let zero = |keys: &[&str]| keys.iter().all(|k| base.f64(k).map_or(true, |v| v == 0.0));
        let hold = |cfg: &Config| {
            let m = (entry.build)(cfg).unwrap();
            let (mut state, mut hold) = (Vec::new(), vec![0.0; m.motors().len()]);
            m.equilibrium(&cfg.f64s("start.pose").unwrap(), &mut state, &mut hold);
            hold
        };
        let unloaded = hold(&base) == hold(&config_of(&base, &["effects.gravity = false"]));
        // A machine with no part that warms but its motors (a screw on a coupling, a linear motor).
        let cold = (entry.build)(&base).unwrap().thermal_parts().is_empty();
        let idle = |e: &str| match e {
            "effects.gearbox_backlash" => zero(&["gearbox.backlash_rad", "rack.backlash_m"]),
            "effects.motor_friction" => zero(&["motor.friction_coulomb_nm", "motor.friction_viscous_nm_s_rad"]),
            // Every efficiency 1 (a linear motor's forcer on its mounting).
            "effects.gearbox_efficiency" => ["gearbox.efficiency", "screw.efficiency", "rack.efficiency"]
                .iter()
                .all(|k| base.f64(k).map_or(true, |v| v == 1.0)),
            // A coupling has no no-load torque; a breakaway at the Coulomb level breaks away at nothing more; a gearbox
            // without a lost-motion band or hysteresis has none to lose; a planetary gearbox's "curve" is one straight line.
            "effects.gearbox_drag" => zero(&["gearbox.no_load_torque_nm"]) || base.f64("gearbox.ratio").unwrap() == 1.0,
            "effects.motor_stiction" => {
                base.f64("motor.friction_breakaway_nm").unwrap() <= base.f64("motor.friction_coulomb_nm").unwrap()
            }
            "effects.gearbox_lost_motion" => zero(&["gearbox.lost_motion_band_rad", "gearbox.hysteresis_rad"]),
            "effects.gearbox_stiffness_curve" => ["gearbox.stiffness_k2_nm_rad", "gearbox.stiffness_k3_nm_rad"]
                .iter()
                .all(|k| base.f64(k).unwrap() == base.f64("gearbox.stiffness_nm_rad").unwrap()),
            "effects.gravity" => unloaded,
            "effects.part_heating" | "effects.thermal_feedback" => cold,
            _ => false,
        };
        let effects: Vec<String> = effects
            .into_iter()
            .filter(|e| !skipped.contains(&e.as_str()) && !(alone && e == "effects.shared_dc_link") && !idle(e))
            .collect();
        assert!(effects.len() >= 20, "{}: only {} effect switches", entry.id, effects.len());
        // A switch that acts only in a setting the base has not is flipped from that setting, in both runs: the belt's
        // wrap sets how many teeth hold against a tooth jump, and the drive's current limit acts where it gives less
        // torque than the motor (the gantries' drives give 10.9 A, 4.9 N m, far above the weak drive's 0.3 N m).
        // The drive's overload acts where the current stays above its rating for the image's time (10 s; the weak
        // drive's 0.3 N m take 0.3 A of the gantries' rated 3 A): a drive with the weak drive's peak current I_p as its
        // maximum, rated a quarter of it, I_p for 10 ms from the rated load in an image of 50 ms trips at
        // 16 - 15 exp(-0.2) = 3.719, which I_p (16) reaches from cold in -0.05 ln(1 - 3.719 / 16) = 13 ms at the peak;
        // then it gives 0.85 of its rated current.
        let current = |share: f64| {
            share * base.f64("drive.peak_torque_nm").unwrap() / base.f64("motor.torque_constant_nm_a").unwrap()
        };
        let limit = [format!("ratings.drive_max_current_a = {:?}", current(0.8))];
        let peak = current(1.0);
        let overload = [
            format!("ratings.drive_max_current_a = {peak:?}"),
            format!("ratings.drive_rated_current_a = {:?}", 0.25 * peak),
            format!("ratings.drive_base_load_current_a = {:?}", 0.85 * 0.25 * peak),
            "ratings.drive_overload_time_constant_s = 0.05".into(),
            "ratings.drive_max_current_time_s = 0.01".into(),
        ];
        let through: [(&str, &[String]); 3] = [
            ("effects.belt_wrap", &["effects.belt_tooth_jump = true".into()]),
            ("effects.drive_current_limit", &limit),
            ("effects.drive_overload", &overload),
        ];
        for effect in effects {
            let (from, own);
            let reference = match through.iter().find(|(e, _)| *e == effect) {
                Some((_, setting)) => {
                    from = config_of(&base, setting);
                    own = traces(&from);
                    &own
                }
                None => {
                    from = base.clone();
                    &reference
                }
            };
            let mut flipped = from.clone();
            flipped.set_text(&format!("{effect} = {}", !from.bool(&effect).unwrap())).unwrap();
            assert!(differs(&flipped, reference), "{}: flipping {effect} changed nothing", entry.id);
        }
    });
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
        .map(|s| {
            (
                s["target"].as_array().unwrap().iter().map(|v| v.as_float().unwrap()).collect(),
                s["dwell_s"].as_float().unwrap(),
            )
        })
        .collect();
    let mut closest = vec![f64::INFINITY; segments.len()];
    sim.command(Command::StartCycle).unwrap();
    let (mut starts, mut last, mut pass) =
        (Vec::new(), None, CyclePass { duration: 0.0, stops: 0, peak_vel: 0.0, peak_acc: 0.0, dwell_miss: 0.0 });
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
    pass.dwell_miss =
        segments.iter().zip(&closest).filter(|((_, dwell), _)| *dwell > 0.0).map(|(_, c)| *c).fold(0.0, f64::max);
    pass
}

/// Corner blending rounds the corners where the machine need not stop: the cycle gets shorter,
/// the axes stop less often, and the places where it dwells are still reached exactly.
#[test]
fn corner_blending_shortens_the_cycle_and_still_reaches_every_dwell() {
    on_every_machine(|entry| {
        let stopping = with_via_point(entry);
        let mut blending = stopping.clone();
        blending.set_text("effects.corner_blending = true").unwrap();
        let (off, on) = (cycle_pass(entry, &stopping), cycle_pass(entry, &blending));
        assert!(
            on.duration < off.duration - 0.05,
            "{}: {} s with blending, {} s without",
            entry.id,
            on.duration,
            off.duration
        );
        assert!(on.stops < off.stops, "{}: {} stops with blending, {} without", entry.id, on.stops, off.stops);
        assert!(
            off.dwell_miss < 1e-12 && on.dwell_miss < 1e-12,
            "{}: a dwell position was missed by {} m",
            entry.id,
            on.dwell_miss
        );
        // A longer lead time than the ramps allow changes nothing more: blending stays on the ramps.
        let mut greedy = blending.clone();
        greedy.set_text("profile.blend_time_s = 10.0").unwrap();
        let greedy = cycle_pass(entry, &greedy);
        assert!(greedy.dwell_miss < 1e-12 && greedy.duration <= on.duration + 1e-9, "{}", entry.id);
    });
}

/// On the machines' own cycles the blended setpoints stay within every axis limit.
#[test]
fn blended_default_cycles_stay_within_the_axis_limits() {
    for entry in MACHINES {
        let mut cfg = entry.config();
        cfg.set_text("effects.corner_blending = true").unwrap();
        cfg.set_text("profile.blend_time_s = 10.0").unwrap();
        let pass = cycle_pass(entry, &cfg);
        assert!(
            pass.peak_vel <= 1.0 + 1e-9 && pass.peak_acc <= 1.0 + 1e-9,
            "{}: {} of v max, {} of a max",
            entry.id,
            pass.peak_vel,
            pass.peak_acc
        );
        assert!(pass.dwell_miss < 1e-12, "{}", entry.id);
    }
}

/// `profile.cycle_s` changed while the cycle runs. The controller counts its own profile cycles and
/// converts what it has stored in them, so the setpoint goes on from where it is: it stays within what an
/// axis travels in three of the longer cycles of the setpoint of an untouched run (the two generators no
/// longer tick together), and no single step is larger than one cycle's travel. With the cycle counted from
/// the step counter, the default T-gantry cycle reconfigured at 2.4 s to 2 ms jumped 0.44 m in X, and a
/// debug build stopped on a subtraction below zero. With input shaping on, the shaper's stored past is
/// converted too.
#[test]
fn changing_the_profile_cycle_on_the_way_does_not_move_the_setpoint() {
    on_every_machine(|entry| {
        for (shaping, cycle) in [(false, 2.0e-3), (false, 0.5e-3), (true, 2.0e-3), (true, 0.5e-3)] {
            let cfg = config(entry, &[&format!("effects.input_shaping = {shaping}")]);
            let (mut sim, mut plain) = (Sim::new(entry, &cfg).unwrap(), Sim::new(entry, &cfg).unwrap());
            for s in [&mut sim, &mut plain] {
                s.command(Command::StartCycle).unwrap();
                s.run(2.4, |_| {});
            }
            // Further on to where an axis moves at more than half its speed limit.
            let fast = |s: &Sim| s.control.set_vel.iter().zip(s.machine.axes()).any(|(v, a)| v.abs() > 0.5 * a.max_vel);
            while !fast(&sim) {
                sim.step();
                plain.step();
            }
            let mut changed = cfg.clone();
            changed.set_text(&format!("profile.cycle_s = {cycle:?}")).unwrap();
            sim.reconfigure(entry, &changed).unwrap();
            let (mut apart, mut step) = (0f64, 0f64);
            for _ in 0..(0.2 / sim.dt) as usize {
                let before = sim.control.set_pos.clone();
                sim.step();
                plain.step();
                for (i, axis) in sim.machine.axes().iter().enumerate() {
                    let per_cycle = axis.max_vel * f64::max(cycle, 1e-3);
                    apart = apart.max((sim.control.set_pos[i] - plain.control.set_pos[i]).abs() / per_cycle);
                    step = step.max((sim.control.set_pos[i] - before[i]).abs() / per_cycle);
                }
            }
            assert!(
                apart < 3.0 && step < 1.01,
                "{}, shaping {shaping}, to {cycle} s: {apart} cycles of travel from the untouched run, {step} in one step",
                entry.id
            );
            // The cycle goes on to its end with the new profile cycle.
            let segment = sim.control.cycle_segment();
            sim.run(3.0, |_| {});
            assert!(
                sim.control.cycle_segment().is_some()
                    && (sim.control.cycle_segment() != segment || sim.control.cycle_passes() > 1),
                "{}",
                entry.id
            );
        }
    });
}

/// With input shaping the setpoint arrives one period of the shaper after the plan. The dwell (and with it
/// the process force) and the gripper wait for the setpoint: while the cycle dwells and whenever the payload
/// switches, the setpoint is on the target. They used to start with the plan, 40 ms early at 25 Hz, with
/// the setpoint still half a millimetre away. Where the shaper follows the pose, the gripper closing lowers the
/// resonance (the delta's from 41.5 to 27.2 Hz); the period stays as it was while the plan stands still, else
/// the longer one reached back into the lift and took the setpoint 0.1 mm off the target.
#[test]
fn with_input_shaping_a_dwell_starts_when_the_setpoint_has_arrived() {
    on_every_machine(|entry| {
        for follows in [false, true] {
            let cfg =
                config(entry, &["effects.input_shaping = true", &format!("effects.shaper_follows_pose = {follows}")]);
            let mut sim = Sim::new(entry, &cfg).unwrap();
            sim.command(Command::StartCycle).unwrap();
            let on_target = |s: &Sim| {
                s.control.set_pos.iter().zip(s.control.planned_pos()).all(|(set, plan)| (set - plan).abs() < 1e-6)
            };
            let (mut dwells, mut switches, mut attached, mut dwelling) = (0, 0, sim.control.payload_attached, false);
            while sim.control.cycle_passes() < 3 {
                sim.step();
                let now = sim.control.dwell_progress().is_some();
                if now || sim.control.payload_attached != attached {
                    assert!(
                        on_target(&sim),
                        "{} at {} s: setpoint {:?}, plan {:?}",
                        entry.id,
                        sim.time(),
                        sim.control.set_pos,
                        sim.control.planned_pos()
                    );
                }
                dwells += (now && !dwelling) as usize;
                switches += (sim.control.payload_attached != attached) as usize;
                (attached, dwelling) = (sim.control.payload_attached, now);
                assert!(sim.time() < 60.0, "{}: the cycle does not go on", entry.id);
            }
            assert!(dwells >= 4 && switches >= 3, "{}: {dwells} dwells, {switches} gripper actions", entry.id);
        }
    });
}

/// A cycle of a single segment is analysed like any other: go there once, then dwell there every pass, so
/// one pass lasts the dwell. The segment index never changes in such a cycle, and the analysis that
/// waited for it to change ran into its 600 s timeout.
#[test]
fn a_cycle_of_one_segment_is_analysed() {
    for entry in MACHINES {
        let text = entry.default_config;
        let first = text.find("[[cycle.segment]]").unwrap();
        let second = first + 1 + text[first + 1..].find("[[cycle.segment]]").unwrap();
        let cfg = Config::parse(&text[..second]).unwrap();
        let dwell = cfg.array("cycle.segment").unwrap()[0]["dwell_s"].as_float().unwrap();
        let mut sim = Sim::new(entry, &cfg).unwrap();
        let load = sim_components::analysis::analyse_cycle(&mut sim).unwrap();
        assert!(
            dwell > 0.0 && (load.duration - dwell).abs() < 2.1e-3 && sim.time() < 10.0,
            "{}: a pass of {} s, analysed after {} s",
            entry.id,
            load.duration,
            sim.time()
        );
    }
}

/// Switching between the rigid and the elastic model while an axis moves carries the motion over: the tool
/// has the speed it had, the drives go on without running into their torque limit, and the tool stays with
/// its setpoint. The new model used to start at rest under a setpoint that went on moving.
#[test]
fn switching_the_model_at_speed_carries_the_motion_over() {
    for entry in MACHINES {
        for elastic in [true, false] {
            let switch = model_switch(&entry.config());
            // 50 ms before the switch and 200 ms after it, all in the cruise (with 50 ms to spare): on the D2 at its
            // 3 m/s the braking at the end of its short cruise fell into them, 8.6 N m with no switch at all.
            let cfg = cruising(entry, config(entry, &[&format!("{switch} = {elastic}")]), 0.3);
            let mut sim = Sim::new(entry, &cfg).unwrap();
            to_low_end(&mut sim);
            sim.command(Command::MoveTo(long_move(&sim))).unwrap();
            let axes = sim.machine.axes().to_vec();
            while sim.control.set_vel[0] < axes[0].max_vel - 1e-9 {
                sim.step();
            }
            let tool = |s: &Sim| {
                let (mut pos, mut vel) = (vec![0.0; axes.len()], vec![0.0; axes.len()]);
                s.machine.tool_state(&s.state, &mut pos, &mut vel);
                (pos, vel)
            };
            // At this speed the tool is behind its setpoint by up to what it travels in a profile cycle.
            let mut behind = 0f64;
            sim.run(0.05, |s| behind = behind.max((tool(s).0[0] - s.control.set_pos[0]).abs()));
            let (before, peak) = (tool(&sim), cfg.f64("drive.peak_torque_nm").unwrap());
            let quiet = sim.control.drives.iter().map(|d| d.torque_command.abs()).fold(0.0, f64::max);
            let mut other = cfg.clone();
            other.set_text(&format!("{switch} = {}", !elastic)).unwrap();
            sim.reconfigure(entry, &other).unwrap();
            let after = tool(&sim);
            for i in 0..axes.len() {
                assert!(
                    (after.0[i] - before.0[i]).abs() < 1e-12 && (after.1[i] - before.1[i]).abs() < 1e-12,
                    "{}: {before:?} to {after:?}",
                    entry.id
                );
            }
            assert!(before.1[0] > 0.95 * axes[0].max_vel);
            let (mut torque, mut off) = (0f64, 0f64);
            sim.run(0.2, |s| {
                torque = s.control.drives.iter().map(|d| d.torque_command.abs()).fold(torque, f64::max);
                off = off.max((tool(s).0[0] - s.control.set_pos[0]).abs());
            });
            assert!(
                torque < 0.5 * peak && off < behind + 2e-4,
                "{}, from elastic {elastic}: {torque} N m (before {quiet} N m, limit {peak} N m), {off} m off the setpoint (before {behind} m)",
                entry.id
            );
        }
    }
}

/// A shared DC link on a whole machine: over a cycle every drive reports the same DC link voltage and the
/// same resistor power at every moment, and the highest voltage is no higher than the highest of the
/// separate links, because one motor's braking feeds another's driving. Machines with one drive are left out.
#[test]
fn a_shared_dc_link_is_one_voltage_for_all_drives_of_a_machine() {
    for entry in MACHINES {
        let peaks = |shared: bool| {
            let mut cfg = entry.config();
            cfg.set_text(&format!("effects.shared_dc_link = {shared}")).unwrap();
            let mut sim = Sim::new(entry, &cfg).unwrap();
            sim.command(Command::StartCycle).unwrap();
            let (mut peak, mut apart) = (0f64, 0f64);
            sim.run(4.0, |s| {
                let drives = &s.control.drives;
                peak = drives.iter().map(|d| d.dc_voltage).fold(peak, f64::max);
                apart = drives
                    .iter()
                    .map(|d| {
                        (d.dc_voltage - drives[0].dc_voltage)
                            .abs()
                            .max((d.resistor_power - drives[0].resistor_power).abs())
                    })
                    .fold(apart, f64::max);
            });
            (peak, apart, sim.control.drives.len())
        };
        let ((separate, apart, drives), (shared, together, _)) = (peaks(false), peaks(true));
        if drives < 2 {
            continue;
        }
        let bus = entry.config().f64("drive.bus_voltage_v").unwrap();
        assert!(
            apart > 1.0 && together == 0.0,
            "{}: links {apart} V apart when separate, {together} when shared",
            entry.id
        );
        assert!(
            shared > bus && shared <= separate,
            "{}: {shared} V shared, {separate} V separate, bus {bus} V",
            entry.id
        );
        println!("{}: highest DC link voltage {separate} V separate, {shared} V shared", entry.id);
    }
}

/// The configs whose velocity loops are checked: every registered machine's default config, and the EXCT-30.
fn shipped_configs() -> Vec<(String, Config, &'static MachineEntry)> {
    let mut configs: Vec<(String, Config, &'static MachineEntry)> =
        MACHINES.iter().map(|e| (e.id.to_string(), e.config(), *e)).collect();
    let exct = &machine_t_gantry::ENTRY;
    configs.push((
        "EXCT-30".into(),
        sim_components::load_config(exct, machine_t_gantry::EXCT_30_CONFIG).unwrap(),
        exct,
    ));
    configs
}

/// The shipped configs named keep their velocity loops' stability margins, with friction as configured and with motor
/// and guide friction off (in a cruise friction adds no damping to the loop): as configured, and with `across` also over
/// the whole range of their `[bounds]` (each value at its low and at its high bound). All were tuned for at least 35
/// degrees and 6 dB (measured at least 37.8 degrees and 7.3 dB on the default configs, 38.6 degrees and 8.3 dB on the
/// EXCT-30 at 0.45 N m s/rad). Every measurement has to be valid and the loop stable (`LoopMargins::stable`).
fn loops_keep_their_margins(names: &[&str], across: bool) {
    use sim_components::analysis::{bounds, velocity_loop_margins};
    let configs: Vec<_> = shipped_configs().into_iter().filter(|(name, ..)| names.contains(&name.as_str())).collect();
    assert_eq!(configs.len(), names.len(), "{names:?}");
    let (phase, gain) = (35.0, 6.0);
    let passes = configs.iter().flat_map(|(name, cfg, entry)| {
        // As configured, friction on, a registered machine's loops are judged to the same 35 degrees and 6 dB by sim-cli's
        // `step_bode_and_tuning_write_csv_for_every_machine` (its `margins`): not again here, but across the bounds.
        let frictions: &[bool] = if across || name.as_str() != entry.id { &[true, false] } else { &[false] };
        frictions.iter().map(move |&friction| {
            // Motor friction, and the guides' or the joints' (whichever the machine has).
            let other = if cfg.bool("effects.guide_friction").is_ok() {
                "effects.guide_friction"
            } else {
                "effects.joint_friction"
            };
            let cfg =
                config_of(cfg, &[&format!("effects.motor_friction = {friction}"), &format!("{other} = {friction}")]);
            // With friction off, the friction values act on nothing: their runs gave the margins of the config as it is
            // to the bit (13 November 2026, on all four configs), so they are left out there. Not with the cable chain on:
            // its drag takes the guide's Stribeck speed (`friction_x.stribeck_m_s`) whether guide friction is on or not.
            let skip = !friction && !cfg.bool("effects.cable_chain").unwrap_or(false);
            let mut bounded = if across { bounds(&cfg).unwrap() } else { Vec::new() };
            bounded.retain(|(path, ..)| {
                !skip
                    || !["friction_", "motor.friction_", "joint.friction_"].iter().any(|start| path.starts_with(start))
            });
            (name, friction, cfg, *entry, bounded)
        })
    });
    // All passes at once: each spreads its runs over every core, and one waiting for its last run leaves the cores to
    // the others instead of idle (the 24 passes, one after the other, ended 24 times on one run).
    std::thread::scope(|scope| {
        let workers: Vec<_> = passes
            .map(|(name, friction, cfg, entry, bounded)| {
                scope.spawn(move || (name, friction, velocity_loop_margins(entry, &cfg, &bounded).unwrap()))
            })
            .collect();
        for (name, friction, runs) in workers.into_iter().map(|w| w.join().unwrap()) {
            for (run, margins) in runs {
                for (k, m) in margins.iter().enumerate() {
                    assert!(
                        m.stable() && m.phase_margin_deg >= phase && m.gain_margin_db >= gain,
                        "{name}, friction {friction}, run '{run}', motor {k}: {m:?}"
                    );
                }
            }
        }
    });
}

/// Every shipped config keeps its velocity loops' margins as configured, without friction and, the EXCT-30, with it
/// (`loops_keep_their_margins`); over the bounds in `across_the_bounds`.
#[test]
fn the_shipped_velocity_loops_keep_their_margins() {
    let configs = shipped_configs();
    loops_keep_their_margins(&configs.iter().map(|(name, ..)| name.as_str()).collect::<Vec<_>>(), false);
}

/// The shipped velocity loops keep their margins across the bounds (`loops_keep_their_margins`), a test per config: in all
/// 8600 s of processor time in a full run, 10 minutes alone on the 16 threads of the 8-core development machine. The
/// default filter of `.config/nextest.toml` leaves them out; CI runs them one at a time after the other tests.
mod across_the_bounds {
    macro_rules! swept {
        ($($test:ident: $name:literal),* $(,)?) => {
            /// The configs swept here, in the order of `shipped_configs`.
            pub const SWEPT: &[&str] = &[$($name),*];
            $(
                #[test]
                fn $test() {
                    super::loops_keep_their_margins(&[$name], true);
                }
            )*
        };
    }
    swept!(
        t_gantry: "t-gantry",
        single_axis: "single-axis",
        delta_d3: "delta-d3",
        delta_d2: "delta-d2",
        delta_linear: "delta-linear",
        delta_d4: "delta-d4",
        screw_axis: "screw-axis",
        rack_axis: "rack-axis",
        linear_motor_axis: "linear-motor-axis",
        rotary_table: "rotary-table",
        urdf_arm: "urdf-arm",
        scara: "scara",
        exct_30: "EXCT-30",
    );
}

/// A machine registered later gets its sweep across the bounds too: `across_the_bounds` names every shipped config.
#[test]
fn every_shipped_config_is_swept_across_its_bounds() {
    assert_eq!(shipped_configs().iter().map(|(name, ..)| name.as_str()).collect::<Vec<_>>(), across_the_bounds::SWEPT);
}

fn config_of(cfg: &Config, edit: &[impl AsRef<str>]) -> Config {
    let mut cfg = cfg.clone();
    for e in edit {
        cfg.set_text(e.as_ref()).unwrap();
    }
    cfg
}

/// The time-optimal plan and its deviation refuse the same paths: with a torque reserve of 0.1 % of the curve, where a
/// motor holds the load against gravity it has no torque left for the path, and the plan was refused with
/// `error-path-torque` while the deviation was reported for it all the same.
#[test]
fn the_time_optimal_deviation_refuses_what_the_plan_refuses() {
    let mut refused = 0;
    for entry in MACHINES {
        let sim =
            Sim::new(entry, &config(entry, &["profile.torque_reserve = 0.001", "profile.path_tolerance_m = 1e-4"]))
                .unwrap();
        let middle: Vec<f64> = sim.machine.axes().iter().map(|a| 0.5 * (a.min + a.max)).collect();
        let pieces = [sim_components::path::Piece::Line(middle)];
        let (plan, deviation) =
            (sim.plan_path(&pieces, true).err().map(|e| e.key), sim.path_deviation(&pieces, true).err().map(|e| e.key));
        assert_eq!(plan, deviation, "{}", entry.id);
        refused += (plan == Some("error-path-torque")) as usize;
    }
    assert!(refused > 0, "no machine refused the path");
}
