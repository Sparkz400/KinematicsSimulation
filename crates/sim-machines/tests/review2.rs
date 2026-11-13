//! Regression tests for the defects of the second review, each with the scenario the review found it in.
//! They run on every registered machine where the scenario does not need a particular one.

use sim_components::analysis::{analyse_cycle, verdicts};
use sim_components::catalog::Candidate;
use sim_components::trace::{MotorTrace, Trace, replay};
use sim_components::{Command, Sim, load_config};
use sim_core::{Config, ConfigError, MachineEntry};
use sim_machines::{MACHINES, find};

fn config(entry: &MachineEntry, edit: &[&str]) -> Config {
    let mut cfg = entry.config();
    for e in edit {
        cfg.set_text(e).unwrap_or_else(|_| panic!("{}: cannot set {e}", entry.id));
    }
    cfg
}

fn refusal(entry: &MachineEntry, cfg: &Config) -> Option<ConfigError> {
    Sim::new(entry, cfg).err()
}

fn sample(sim: &Sim) -> Vec<f64> {
    let mut out = Vec::new();
    sim.sample(&mut out);
    out
}

/// A long move of the first axis, the others staying where they start.
fn long_move(sim: &Sim) -> Vec<f64> {
    let axis = &sim.machine.axes()[0];
    let mut target = sim.control.set_pos.clone();
    target[0] = axis.min + 0.9 * (axis.max - axis.min);
    target
}

/// Moves the first axis to near the low end of its range (5 % of it in) and waits there (see control.rs): 50 mm on the
/// gantries, which start at 0.1 m.
fn to_low_end(sim: &mut Sim) {
    let axis = sim.machine.axes()[0].clone();
    let mut low = sim.control.set_pos.clone();
    low[0] = low[0].min(axis.min + 0.05 * (axis.max - axis.min));
    sim.command(Command::MoveTo(low)).unwrap();
    while sim.control.busy() {
        sim.step();
    }
}

fn tool(sim: &Sim) -> Vec<f64> {
    let n = sim.machine.axes().len();
    let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
    sim.machine.tool_state(&sim.state, &mut pos, &mut vel);
    pos
}

fn tensions(sim: &Sim) -> Vec<f64> {
    let row = sample(sim);
    sim.signal_infos().iter().zip(row).filter(|(s, _)| s.id.starts_with("belt.") && s.id.ends_with(".tension")).map(|(_, v)| v).collect()
}

// ---------------------------------------------------------------------------------------------
// 1. Config values read unchecked

/// The review's values, and their like: each is refused with its key, by a new simulation and by a running
/// one, and none panics. They used to panic in `f64::clamp`, or to fill the simulation with NaN.
#[test]
fn bad_config_values_are_refused_with_their_key() {
    for entry in MACHINES.iter().copied() {
        let axes = Sim::new(entry, &entry.config()).unwrap().machine.axes().len();
        let pose = |first: &str| format!("start.pose = [{}]", [&[first][..], &vec!["0.2"; axes - 1]].concat().join(", "));
        let (not_a_number, beyond) = (pose("nan"), pose("5.0"));
        let table = [
            ("gearbox.backlash_rad = -0.001", "gearbox.backlash_rad"),
            ("gearbox.inertia_at_input_kgm2 = -0.3e-4", "gearbox.inertia_at_input_kgm2"),
            ("belt.pretension_n = -5e5", "belt.pretension_n"),
            ("mass.payload_kg = -100", "mass.payload_kg"),
            (not_a_number.as_str(), "start.pose"),
            (beyond.as_str(), "start.pose"),
            ("gearbox.efficiency = 1.5", "gearbox.efficiency"),
            ("gearbox.damping_nm_s_rad = -0.2", "gearbox.damping_nm_s_rad"),
            ("motor.ripple_fraction = 1.0", "motor.ripple_fraction"),
            ("motor.start_temperature_rise_k = -300.0", "motor.start_temperature_rise_k"),
            ("drive.no_load_speed_rad_s = 314.0", "drive.no_load_speed_rad_s"),
            ("drive.kv_1_s = inf", "drive.kv_1_s"),
            ("profile.shaper_damping = 1.0", "profile.shaper_damping"),
            ("profile.blend_time_s = -0.1", "profile.blend_time_s"),
            ("friction_x.viscous_n_s_m = nan", "friction_x.viscous_n_s_m"),
            ("hard_stop.damping_n_s_m = -5.0", "hard_stop.damping_n_s_m"),
        ];
        // Keys of belt and guide on the machines that have them.
        for (edit, key) in table.into_iter().filter(|(_, key)| *key == "start.pose" || entry.config().get(key).is_ok()) {
            let cfg = config(entry, &[edit]);
            let refused = refusal(entry, &cfg).unwrap_or_else(|| panic!("{}: {edit} was accepted", entry.id));
            assert_eq!((refused.key(), refused.detail()), ("error-config-invalid", key), "{}: {edit}", entry.id);
            // A running simulation refuses it the same way and runs on as it was.
            let mut sim = Sim::new(entry, &entry.config()).unwrap();
            sim.command(Command::StartCycle).unwrap();
            sim.run(0.02, |_| {});
            let before = sim.state.clone();
            assert_eq!(sim.reconfigure(entry, &cfg).err().map(|e| e.detail().to_string()).as_deref(), Some(key), "{}: {edit}", entry.id);
            assert_eq!(sim.state, before);
            sim.run(0.02, |_| {});
            assert!(sample(&sim).iter().all(|v| v.is_finite()), "{}: {edit}", entry.id);
        }
        // A rating that is no number is refused where it is read, not passed over as if it were left out.
        let load = analyse_cycle(&mut Sim::new(entry, &entry.config()).unwrap()).unwrap();
        let rating = if entry.config().get("ratings.belt_max_tension_n").is_ok() { "ratings.belt_max_tension_n" } else { "ratings.settling_tolerance_m" };
        let no_number = config(entry, &[&format!("{rating} = nan")]);
        assert_eq!(verdicts(&no_number, &load).err(), Some(ConfigError::Invalid(rating.into())), "{}", entry.id);
        // A cycle segment that dwells for a negative or an endless time is refused too.
        let first_dwell = |text: &str| {
            let at = text.find("dwell_s = ").unwrap() + "dwell_s = ".len();
            text[at..at + text[at..].find('\n').unwrap()].to_string()
        };
        for dwell in ["-1.0", "inf", "nan"] {
            let text = entry.default_config.replacen(&format!("dwell_s = {}", first_dwell(entry.default_config)), &format!("dwell_s = {dwell}"), 1);
            let refused = refusal(entry, &Config::parse(&text).unwrap()).unwrap_or_else(|| panic!("{}: a dwell of {dwell} was accepted", entry.id));
            assert!(refused.detail().starts_with("cycle.segment["), "{}: {refused:?}", entry.id);
        }
    }
}

/// Every numeric value of every machine's default config, set to -1, 0, NaN and infinity, one at a time:
/// the simulation either refuses the config or runs the cycle with finite signals. It never panics.
#[test]
fn no_single_bad_number_panics_the_simulation_or_fills_it_with_nan() {
    for entry in MACHINES.iter().copied() {
        let numeric: Vec<String> = entry.config().leaves().into_iter().filter(|(_, v)| v.is_float() || v.is_integer()).map(|(path, _)| path).collect();
        assert!(numeric.len() > 100, "{}: only {} numbers found", entry.id, numeric.len());
        let (mut refused, mut ran) = (0, 0);
        for path in &numeric {
            for value in ["-1", "0", "nan", "inf"] {
                let mut cfg = entry.config();
                // A number of the wrong kind (NaN for a whole number) is refused before it is in the config.
                if cfg.set_text(&format!("{path} = {value}")).is_err() {
                    refused += 1;
                    continue;
                }
                let outcome = std::panic::catch_unwind(|| {
                    let Ok(mut sim) = Sim::new(entry, &cfg) else { return None };
                    sim.command(Command::StartCycle).unwrap();
                    let mut finite = true;
                    sim.run(0.05, |sim| finite &= sim.step % 16 != 0 || sample(sim).iter().all(|v| v.is_finite()));
                    Some(finite && sim.state.iter().all(|v| v.is_finite()))
                });
                match outcome {
                    Err(_) => panic!("{}: {path} = {value} panics", entry.id),
                    Ok(Some(false)) => panic!("{}: {path} = {value} is accepted and gives signals that are not finite", entry.id),
                    Ok(Some(true)) => ran += 1,
                    Ok(None) => refused += 1,
                }
            }
        }
        // NaN and infinity are always refused, so at least half; and some zeros are good values.
        assert!(refused >= 2 * numeric.len() && ran > 10, "{}: {refused} refused, {ran} ran", entry.id);
    }
}

/// `[parts]` with a name that is not text was taken as no name. And the default config is the schema of a
/// config file: a key it does not have is refused by name, except under `[bounds]`.
#[test]
fn a_part_that_is_no_name_and_a_key_the_machine_does_not_know_are_refused() {
    for entry in MACHINES.iter().copied() {
        let cfg = Config::parse(&entry.default_config.replacen("motor = \"\"", "motor = 5", 1)).unwrap();
        assert_eq!(refusal(entry, &cfg), Some(ConfigError::Type("parts.motor".into())), "{}", entry.id);
        assert_eq!(sim_components::catalog::resolve(entry, &cfg).err(), Some(ConfigError::Type("parts.motor".into())));

        assert!(load_config(entry, entry.default_config).is_ok());
        let misspelt = entry.default_config.replacen("\n[gearbox]\nratio = ", "\n[gearbox]\nratoi = ", 1);
        assert_eq!(load_config(entry, &misspelt).err(), Some(ConfigError::Unknown("gearbox.ratoi".into())), "{}", entry.id);
        let table = format!("{}\n[gearbxo]\nratio = 3.0\n", entry.default_config);
        assert_eq!(load_config(entry, &table).err(), Some(ConfigError::Unknown("gearbxo".into())), "{}", entry.id);
        let rating = entry.default_config.replacen("\n[ratings]\n", "\n[ratings]\nbelt_max_tensoin_n = 1.0\n", 1);
        assert_eq!(load_config(entry, &rating).err(), Some(ConfigError::Unknown("ratings.belt_max_tensoin_n".into())), "{}", entry.id);
        // Bounds have their own check, which names the bound.
        let bound = entry.default_config.replacen("\n[bounds]\n", "\n[bounds]\nbelt.pretention_n = [1.0, 2.0]\n", 1);
        assert!(load_config(entry, &bound).is_ok());
        assert_eq!(sim_components::analysis::bounds(&Config::parse(&bound).unwrap()).err(), Some(ConfigError::Missing("belt.pretention_n".into())));
    }
    // The real machine's config file passes: it has a rating more than the default config, and no other key.
    assert_eq!(load_config(&machine_t_gantry::ENTRY, machine_t_gantry::EXCT_30_CONFIG).err(), None);
}

// ---------------------------------------------------------------------------------------------
// 2. A shaper changed during a motion

/// Largest step of the first axis's position setpoint from one base step to the next, and the highest torque
/// the velocity loops ask for, over a long move. `change` is applied 0.4 s into it, at speed.
fn setpoint_steps(entry: &MachineEntry, change: Option<&str>) -> (f64, f64, Sim) {
    let shaped = config(entry, &["effects.input_shaping = true"]);
    let mut sim = Sim::new(entry, &shaped).unwrap();
    to_low_end(&mut sim);
    let begun = sim.time();
    sim.command(Command::MoveTo(long_move(&sim))).unwrap();
    let (mut last, mut step, mut demand) = (sim.control.set_pos[0], 0f64, 0f64);
    let mut watch = |sim: &Sim| {
        step = step.max((sim.control.set_pos[0] - last).abs());
        last = sim.control.set_pos[0];
        demand = sim.control.drives.iter().fold(demand, |d, drive| d.max(drive.torque_demand.abs()));
    };
    // 0.4 s in, or as soon as the first axis cruises where a move is shorter (the delta's 3 m/s over a metre).
    let v_max = sim.machine.axes()[0].max_vel;
    while sim.time() - begun < 0.4 && !(sim.time() - begun > 0.1 && sim.control.set_vel[0] >= v_max * (1.0 - 1e-9)) {
        sim.step();
        watch(&sim);
    }
    assert!(sim.control.set_vel[0] > 0.5 * v_max, "{}: not at speed at the moment of the change", entry.id);
    if let Some(change) = change {
        let mut changed = shaped.clone();
        changed.set_text(change).unwrap();
        sim.reconfigure(entry, &changed).unwrap();
    }
    while sim.control.busy() {
        sim.step();
        watch(&sim);
    }
    sim.run(0.2, &mut watch);
    (step, demand, sim)
}

/// Switching input shaping off, or changing the shaper's frequency, at speed: the setpoint used to jump
/// (16 mm, and a torque demand of 14.85 N m against 0.60 N m). The change now waits until the plan stands
/// still, so the largest setpoint step stays the one of the undisturbed move: speed times profile cycle,
/// 0.8 mm on the gantry's X axis.
#[test]
fn a_shaper_changed_at_speed_does_not_make_the_setpoint_jump() {
    for entry in MACHINES.iter().copied() {
        let (step, demand, sim) = setpoint_steps(entry, None);
        let v_max = sim.machine.axes()[0].max_vel;
        assert!(step > 0.5e-3 * v_max && step <= 1e-3 * v_max * (1.0 + 1e-9), "{}: the undisturbed move steps by {step} m", entry.id);
        for change in ["effects.input_shaping = false", "profile.shaper_frequency_hz = 40.0", "effects.shaper_follows_pose = true"] {
            let (changed_step, changed_demand, mut sim) = setpoint_steps(entry, Some(change));
            assert!(changed_step <= step * (1.0 + 1e-9), "{}: {change} makes the setpoint step by {changed_step} m, undisturbed {step} m", entry.id);
            assert!(changed_demand <= 1.05 * demand, "{}: {change} asks for {changed_demand} N m, undisturbed {demand} N m", entry.id);
            // At rest the change was taken: without the shaper the setpoint is the plan again, also on the way.
            if change.ends_with("false") {
                let back = sim.control.planned_pos().iter().zip(sim.machine.axes()).map(|(p, a)| p - 0.2 * (a.max - a.min)).collect();
                sim.command(Command::MoveTo(back)).unwrap();
                // On the way and to its end (the gantries' moves take longer than 0.2 s, the delta's not).
                sim.step();
                assert!(sim.control.busy());
                while sim.control.busy() {
                    sim.step();
                    assert_eq!(sim.control.set_pos, sim.control.planned_pos());
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// 3. A reconfigure that changes the kinematics

/// A gear ratio or a pulley radius changed by 1 %, at standstill and during a move. The old motor and pulley
/// angles were kept and belonged to another pose with the new kinematics: 5323 N of belt tension against
/// 300 N of pretension. Now the state is rebuilt at the tool's pose and speed, with the belt as it is at rest
/// there: the belt tensions stay within 10 % of the pretension of what they are in a run left alone (at speed
/// the belt carried the friction, 16 N per section, which it has to take up anew), and the tool within
/// 0.1 mm of where it is there.
#[test]
fn a_changed_ratio_or_pulley_radius_does_not_tear_the_machine() {
    for entry in MACHINES.iter().copied() {
        // The pulley radius where there are pulleys.
        for path in ["gearbox.ratio", "geometry.pulley_radius_m"].into_iter().filter(|path| entry.config().get(path).is_ok()) {
            for moving in [false, true] {
                // With the fieldbus delay, so that setpoints worked out with the old kinematics are on their way.
                let cfg = config(entry, &["effects.fieldbus_delay = true"]);
                let pretension = cfg.f64("belt.pretension_n").unwrap_or(0.0);
                let mut sims = [Sim::new(entry, &cfg).unwrap(), Sim::new(entry, &cfg).unwrap()];
                for sim in &mut sims {
                    if moving {
                        to_low_end(sim);
                        sim.command(Command::MoveTo(long_move(sim))).unwrap();
                    }
                    // 0.4 s in, or in the middle of a move too short for that (the delta's 0.42 s), at speed.
                    let duration = sim_components::profile::Move::new(&sim.control.set_pos, &long_move(sim), sim.machine.axes()).duration();
                    sim.run(if moving { f64::min(0.4, 0.5 * duration) } else { 0.4 }, |_| {});
                }
                assert_eq!(sims[0].control.busy(), moving);
                let mut changed = cfg.clone();
                changed.set_text(&format!("{path} = {:?}", 1.01 * cfg.f64(path).unwrap())).unwrap();
                sims[1].reconfigure(entry, &changed).unwrap();
                let (mut tension_off, mut tool_off) = (0f64, 0f64);
                for _ in 0..(0.5 / sims[0].dt) as usize {
                    sims.iter_mut().for_each(Sim::step);
                    let off = |a: Vec<f64>, b: Vec<f64>| a.iter().zip(&b).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
                    tension_off = tension_off.max(off(tensions(&sims[0]), tensions(&sims[1])));
                    tool_off = tool_off.max(off(tool(&sims[0]), tool(&sims[1])));
                }
                let what = format!("{}: {path} changed by 1 % {}", entry.id, if moving { "during a move" } else { "at standstill" });
                assert!(tension_off <= 0.1 * pretension, "{what}: belt tension off by {tension_off} N");
                // 0.1 mm, or a tenth of what the machine's gears sag at rest where that is more (the delta's backlash
                // takes up anew after the change: 0.13 mm with 9 arcmin).
                assert!(tool_off < (0.1 * sims[0].machine.setpoint_slack()).max(1e-4), "{what}: tool off by {tool_off} m");
                // The motors stand where the new kinematics put them for this pose.
                let (mut wanted, mut speed) = (vec![0.0; sims[1].machine.motors().len()], vec![0.0; sims[1].machine.motors().len()]);
                sims[1].machine.inverse(&tool(&sims[1]), &mut wanted);
                let mut angle = wanted.clone();
                sims[1].machine.motor_state(&sims[1].state, &mut angle, &mut speed);
                assert!(wanted.iter().zip(&angle).all(|(w, a)| (w - a).abs() < 0.05), "{what}: motors at {angle:?}, the pose asks for {wanted:?}");
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------
// d. Fewer cycle segments during a running cycle

/// A cycle that loses segments while it runs: the segment index used to point past the end (a panic), and
/// a cycle left without segments divided by zero. The cycle goes on within the segments it has, or ends.
#[test]
fn a_running_cycle_survives_losing_segments() {
    for entry in MACHINES.iter().copied() {
        let cfg = entry.config();
        let segments = cfg.array("cycle.segment").unwrap().clone();
        let running = || {
            let mut sim = Sim::new(entry, &cfg).unwrap();
            sim.command(Command::StartCycle).unwrap();
            // Into the last segment.
            while sim.control.cycle_segment() != Some(segments.len() - 1) {
                sim.step();
                assert!(sim.time() < 60.0, "{}: the cycle never reaches its last segment", entry.id);
            }
            sim
        };
        let mut sim = running();
        let mut fewer = cfg.clone();
        fewer.set_text(&format!("cycle.segment = [{}]", segments[0])).unwrap();
        sim.reconfigure(entry, &fewer).unwrap();
        assert_eq!(sim.control.cycle_segment(), Some(0));
        let passes = sim.control.cycle_passes();
        sim.run(3.0, |sim| assert_eq!(sim.control.cycle_segment(), Some(0)));
        assert!(sim.control.cycle_passes() > passes && sample(&sim).iter().all(|v| v.is_finite()), "{}: the cycle of one segment does not run", entry.id);

        let mut sim = running();
        let mut none = cfg.clone();
        none.set_text("cycle.segment = []").unwrap();
        sim.reconfigure(entry, &none).unwrap();
        assert_eq!(sim.control.cycle_segment(), None);
        sim.run(3.0, |_| {});
        // The move under way was finished, and nothing follows it.
        assert!(!sim.control.busy() && sample(&sim).iter().all(|v| v.is_finite()), "{}", entry.id);
        assert_eq!(sim.command(Command::StartCycle).err().map(|e| e.key), Some("error-no-cycle"));
    }
}

// ---------------------------------------------------------------------------------------------
// 4. A setpoint from outside that asks for a jump

/// A setpoint further away than the axis may travel in one setpoint interval (a PLC sending a stale value:
/// 0.25 m in 10 ms, a torque demand of 698 N m) is refused; one within reach is followed.
#[test]
fn an_external_setpoint_beyond_the_speed_limit_is_refused() {
    for entry in MACHINES.iter().copied() {
        let mut sim = Sim::new(entry, &entry.config()).unwrap();
        let (start, axis) = (sim.control.set_pos.clone(), sim.machine.axes()[0].clone());
        let interval = entry.config().f64("profile.setpoint_interval_s").unwrap();
        let at = |offset: f64| [&[start[0] + offset][..], &start[1..]].concat();
        assert_eq!(sim.command(Command::Setpoint(at(0.25))).err().map(|e| e.key), Some("error-setpoint-too-far"), "{}", entry.id);
        assert_eq!(sim.command(Command::Setpoint(at(1.01 * axis.max_vel * interval))).err().map(|e| e.key), Some("error-setpoint-too-far"));
        sim.run(0.05, |_| {});
        assert_eq!(sim.control.set_pos, start, "{}: a refused setpoint moved the axis", entry.id);
        // Exactly at the limit is within it.
        sim.command(Command::Setpoint(at(axis.max_vel * interval))).unwrap();
        sim.run(0.05, |_| {});
        // Once followed, the plan stands at the value sent, to the last bit: a link shows it as its setpoint.
        // Near zero, where one setpoint is several times the last: there from + (to - from) is not always to.
        for step in 1..=50 {
            let sent = [&[axis.min + 1e-3 + 0.137e-3 * step as f64 + 5e-3 * (step % 2) as f64][..], &start[1..]].concat();
            if step == 1 {
                sim.command(Command::MoveTo(sent.clone())).unwrap();
                sim.run(3.0, |_| {});
            }
            sim.command(Command::Setpoint(sent.clone())).unwrap();
            assert_eq!(sim.control.external_setpoint(), sent);
            sim.run(interval + 2e-3, |_| {});
            assert_eq!(sim.control.planned_pos(), sent, "{}: setpoint {step}", entry.id);
        }
    }
}

// ---------------------------------------------------------------------------------------------
// 5. A cycle that trips the DC link before the measured pass

/// The review's scenario: the single axis with a payload of 120 kg trips on DC link overvoltage in its
/// first pass. The analysis measured the second pass only, so every motor figure was 0 and "ok". Now the
/// figures are those up to the trip, and such a run is never the nearest to passing in a search.
#[test]
fn a_cycle_that_trips_before_the_measured_pass_reports_the_run_up_to_the_trip() {
    let entry = find("single-axis").unwrap();
    let cfg = config(entry, &["mass.payload_kg = 120.0"]);
    let mut sim = Sim::new(entry, &cfg).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    assert!(load.tripped && !sim.control.enabled() && sim.control.cycle_passes() < 2);
    let motor = &load.motors[0];
    assert!(load.duration > 0.1 && motor.peak_torque > 1.0 && motor.rms_torque > 0.1 && motor.utilisation > 0.5 && motor.peak_speed > 10.0, "{motor:?} in {} s", load.duration);
    let judged = verdicts(&cfg, &load).unwrap();
    let peak = judged.iter().find(|v| v.check == "motor-peak").unwrap();
    assert!(peak.value > 0.5, "motor-peak judges {}", peak.value);
    assert!(judged.iter().any(|v| v.check == "dc-link-overvoltage" && !v.ok));
    let candidate = Candidate { article: "", gearbox: String::new(), drive: "", axis: String::new(), mass: 1.0, cost: 1.0, result: Some((load, judged)), refused: "" };
    assert!(!candidate.passes());
    assert_eq!(candidate.tightest(), Some(("dc-link-overvoltage", f64::INFINITY)));
    // A cycle that does not trip is measured as before, over its second pass.
    let mut sim = Sim::new(entry, &entry.config()).unwrap();
    let load = analyse_cycle(&mut sim).unwrap();
    assert!(!load.tripped && sim.control.cycle_passes() == 3);
}

// ---------------------------------------------------------------------------------------------
// 8. Replay of a very short trace

/// A trace of exactly three rows used to index out of bounds. Fewer than four rows are refused, four replay.
#[test]
fn replay_refuses_a_trace_too_short_to_replay() {
    for entry in MACHINES.iter().copied() {
        let cfg = entry.config();
        let sim = Sim::new(entry, &cfg).unwrap();
        let linear = sim_components::trace::linear_transformation(sim.machine.as_ref());
        for rows in 0..=4usize {
            let motors = sim.control.drives.iter().map(|d| MotorTrace { setpoint: vec![d.pos_set; rows], angle: vec![d.pos_set; rows], speed: vec![0.0; rows], torque: vec![0.0; rows] });
            let trace = Trace { time: (0..rows).map(|i| i as f64 * 1e-3).collect(), motors: motors.collect() };
            let replayed = replay(entry, &cfg, &trace);
            if !linear {
                // Replay refuses a transformation that is not linear (the delta), whatever the trace.
                assert_eq!(replayed.err().map(|e| e.key), Some("error-trace-not-linear"), "{}", entry.id);
            } else if rows < 4 {
                assert_eq!(replayed.err().map(|e| e.key), Some("error-trace-short"), "{}: {rows} rows", entry.id);
            } else {
                assert_eq!(replayed.unwrap().time.len(), rows, "{}", entry.id);
            }
        }
    }
}
