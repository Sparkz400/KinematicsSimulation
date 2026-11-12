//! The app without its window. Phase 4 is done when switching machine or language needs no code
//! change, and switching one effect off changes the traces within one cycle.

use sim_app::model::{Analysis, Model};
use sim_app::session::{Event, Request, Runner, Session};
use sim_components::Command;
use sim_core::{Config, MachineEntry};
use sim_machines::{LANGUAGES, MACHINES};
use std::time::{Duration, Instant};

/// Polls until the simulation thread has delivered samples and a scene.
fn wait_for_data(model: &mut Model) {
    let start = Instant::now();
    while (model.history.len() < 20 || model.scene.parts.is_empty()) && start.elapsed() < Duration::from_secs(5) {
        model.poll();
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// Switching machine needs no code change: every registered machine runs in the same model, with
/// scopes, scene, effect switches and parameters generated from what the machine supplies.
#[test]
fn every_machine_runs_in_the_same_app_model() {
    let mut model = Model::new(MACHINES[0], "en");
    for entry in MACHINES.iter().chain([&MACHINES[0]]) {
        model.set_machine(entry);
        assert_eq!(model.entry.id, entry.id);
        wait_for_data(&mut model);
        assert!(model.history.len() >= 20, "{}: no samples arrived", entry.id);
        assert!(model.scene.parts.len() >= 3, "{}: no scene arrived", entry.id);
        let signals = model.session.signals.len();
        assert!(model.history.iter().all(|row| row.len() == signals));
        // Every signal except time is on exactly one scope.
        let mut on_scopes: Vec<usize> = model.scopes.iter().flat_map(|s| s.lines.iter().map(|l| l.signal)).collect();
        on_scopes.sort();
        assert_eq!(on_scopes, (1..signals).collect::<Vec<_>>(), "{}", entry.id);
        assert!(model.scopes.iter().any(|s| s.visible));
        // Setpoints and commands share a scope with the value they ask for, as dashed references.
        for (quantity, references) in [("quantity-position", model.session.axes.len()), ("quantity-torque", model.session.motors.len())] {
            let scope = model.scopes.iter().find(|s| s.quantity_key == quantity).unwrap();
            assert_eq!(scope.lines.iter().filter(|l| l.reference).count(), references, "{}: {quantity}", entry.id);
            assert_eq!(scope.lines.iter().filter(|l| !l.reference).count(), references, "{}: {quantity}", entry.id);
        }
        assert!(!model.scene.labels.is_empty(), "{}: no live values in the 3D view", entry.id);
        assert_eq!(model.target.len(), model.session.axes.len());
        assert!(model.effects().len() >= 20 && model.param_groups().len() >= 8, "{}", entry.id);
    }
}

fn string_literals(source: &str) -> Vec<String> {
    let (mut out, mut chars) = (Vec::new(), source.chars().peekable());
    while let Some(c) = chars.next() {
        match c {
            '/' if chars.peek() == Some(&'/') => while chars.next().is_some_and(|c| c != '\n') {},
            // A char literal or a lifetime: skip what follows the quote, so '"' cannot open a string.
            '\'' => {
                if chars.next() == Some('\\') {
                    chars.next();
                }
            }
            '"' => {
                let mut text = String::new();
                while let Some(c) = chars.next() {
                    match c {
                        '\\' => text.extend(chars.next()),
                        '"' => break,
                        c => text.push(c),
                    }
                }
                out.push(text);
            }
            _ => {}
        }
    }
    out
}

fn window_sources() -> String {
    ["src/main.rs", "src/ui.rs"].map(|f| std::fs::read_to_string(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(f)).unwrap()).concat()
}

/// No text for the user is written in the window code: every string there is a translation key,
/// an identifier, or punctuation around placeholders.
#[test]
fn window_code_holds_no_text_for_the_user() {
    let literals = string_literals(&window_sources());
    assert!(literals.len() > 30, "the scan found only {} strings", literals.len());
    for literal in literals {
        // Drop {placeholders}; what is left may be a key fragment or an environment variable name.
        let mut rest = String::new();
        let mut depth = 0;
        for c in literal.chars() {
            match c {
                '{' => depth += 1,
                '}' => depth -= 1,
                c if depth == 0 => rest.push(c),
                _ => {}
            }
        }
        let key_like = rest.chars().all(|c| c.is_ascii_lowercase() || c.is_ascii_digit() || "-._".contains(c));
        let env_like = rest.chars().all(|c| c.is_ascii_uppercase() || c == '_');
        let no_letters = !rest.chars().any(char::is_alphabetic);
        assert!(key_like || env_like || no_letters, "text in window code: {literal:?}");
    }
}

/// Switching language needs no code change: every key the window can show has an entry in every
/// language, for every machine.
#[test]
fn every_label_exists_in_every_language() {
    let ui_keys: Vec<String> =
        string_literals(&window_sources()).into_iter().filter(|l| (l.starts_with("ui-") || l.starts_with("lang-")) && !l.contains('{')).collect();
    assert!(ui_keys.len() >= 15);
    let mut model = Model::new(MACHINES[0], "en");
    for entry in MACHINES {
        model.set_machine(entry);
        let mut seen = Vec::new();
        for lang in LANGUAGES {
            model.set_language(lang);
            assert_eq!(model.lang, *lang);
            let mut keys = model.keys_in_use();
            keys.extend(ui_keys.iter().cloned());
            keys.extend(LANGUAGES.iter().map(|l| format!("lang-{l}")));
            for key in &keys {
                assert!(model.tr.has(key), "{}: no '{lang}' entry for '{key}'", entry.id);
            }
            seen.push((model.tr.tr(entry.key).to_string(), model.tr.tr("ui-parameters").to_string(), model.signal_label(&model.session.signals[2])));
        }
        assert!(seen.windows(2).all(|w| w[0] != w[1]), "{}: the languages read the same: {seen:?}", entry.id);
    }
}

#[test]
fn editing_a_parameter_reaches_the_running_simulation() {
    let mut model = Model::new(MACHINES[0], "en");
    model.set_param("mass.payload_kg", toml::Value::Float(7.5));
    assert_eq!(model.config.f64("mass.payload_kg").unwrap(), 7.5);
    assert!(model.messages.is_empty());
    // A value the simulation refuses comes back as a message, and the simulation keeps running.
    model.set_param("drive.velocity_cycle_s", toml::Value::Float(1.0e-4));
    model.set_param("no.such_value", toml::Value::Float(1.0));
    let start = Instant::now();
    while model.messages.len() < 2 && start.elapsed() < Duration::from_secs(5) {
        model.poll();
        std::thread::sleep(Duration::from_millis(5));
    }
    let keys: Vec<&str> = model.messages.iter().map(|m| m.0).collect();
    assert!(keys.contains(&"error-config-missing") && keys.contains(&"error-config-invalid"), "{keys:?}");
    let before = model.history.len();
    wait_for_data(&mut model);
    std::thread::sleep(Duration::from_millis(50));
    model.poll();
    assert!(model.history.len() > before);
}

fn samples(events: &[Event]) -> Vec<&Vec<f64>> {
    events.iter().filter_map(|e| if let Event::Sample(s) = e { Some(s) } else { None }).collect()
}

/// A config in which every effect has something to act on: low pretension so the belt can go
/// slack, a weak drive so the torque limit is reached, a low bus voltage so the voltage limit is, a via point so corner blending has a corner.
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

fn demanding(entry: &MachineEntry) -> Config {
    let mut cfg = with_via_point(entry);
    cfg.set_text("belt.pretension_n = 20.0").unwrap();
    cfg.set_text("drive.peak_torque_nm = 0.3").unwrap();
    // 100 V at the winding.
    cfg.set_text("drive.bus_voltage_v = 81.65").unwrap();
    cfg
}

/// Seconds for one pass through the pick-and-place cycle.
fn cycle_time(entry: &'static MachineEntry, cfg: &Config) -> f64 {
    let mut runner = Runner::new(entry, cfg).unwrap();
    runner.handle(Request::Command(Command::StartCycle), &mut Vec::new());
    let (mut starts, mut last) = (Vec::new(), None);
    while starts.len() < 2 && runner.sim.time() < 60.0 {
        runner.sim.step();
        let now = runner.sim.control.cycle_segment();
        if now != last && now == Some(0) {
            starts.push(runner.sim.time());
        }
        last = now;
    }
    assert_eq!(starts.len(), 2, "{}: the cycle does not repeat within a minute", entry.id);
    starts[1] - starts[0]
}

/// Switching one effect off (or on) changes the traces within one cycle, without a restart.
/// Checked for every switch of every machine, against a run that was left alone.
#[test]
fn switching_an_effect_changes_the_traces_within_one_cycle() {
    for entry in MACHINES.iter().copied() {
        let base = demanding(entry);
        let limit = cycle_time(entry, &base);
        let effects: Vec<String> = base.leaves().into_iter().map(|(p, _)| p).filter(|p| p.starts_with("effects.")).collect();
        // Hard stops act only beyond the stroke and holding brakes only while the drives are disabled. A cycle
        // meets neither. The shaper follows the pose only where input shaping is on, which it is not here.
        // Their own tests show them working.
        let skipped = ["effects.hard_stops", "effects.holding_brake", "effects.shaper_follows_pose"];
        // A machine with one drive has nothing to share a DC link with.
        let alone = Runner::new(entry, &base).unwrap().sim.machine.motors().len() < 2;
        let effects: Vec<String> = effects.into_iter().filter(|e| !skipped.contains(&e.as_str()) && !(alone && e == "effects.shared_dc_link")).collect();
        for effect in effects {
            let mut runners = [Runner::new(entry, &base).unwrap(), Runner::new(entry, &base).unwrap()];
            let mut events = [Vec::new(), Vec::new()];
            for (runner, events) in runners.iter_mut().zip(&mut events) {
                runner.handle(Request::Command(Command::StartCycle), events);
                runner.advance(0.5, events);
            }
            assert!(samples(&events[0]) == samples(&events[1]) && samples(&events[0]).len() == 500);

            let mut flipped = base.clone();
            flipped.set_text(&format!("{effect} = {}", !base.bool(&effect).unwrap())).unwrap();
            let mut answer = Vec::new();
            runners[1].handle(Request::Reconfigure(flipped), &mut answer);
            assert!(matches!(answer[..], [Event::Configured(Ok(_))]), "{}: {effect} was refused", entry.id);

            let mut changed_after = None;
            while changed_after.is_none() && runners[0].sim.time() < 0.5 + limit {
                let mut step = [Vec::new(), Vec::new()];
                runners[0].advance(1e-3, &mut step[0]);
                runners[1].advance(1e-3, &mut step[1]);
                if samples(&step[0]) != samples(&step[1]) {
                    changed_after = Some(runners[0].sim.time() - 0.5);
                }
            }
            assert!(changed_after.is_some(), "{}: switching {effect} changed nothing within one cycle of {limit:.2} s", entry.id);
        }
    }
}

/// Effects that act all the time show up at once: within one position-loop cycle of 1 ms.
#[test]
fn switching_friction_off_changes_the_traces_within_a_millisecond() {
    for entry in MACHINES.iter().copied() {
        let cfg = entry.config();
        let mut runner = Runner::new(entry, &cfg).unwrap();
        let mut events = Vec::new();
        runner.handle(Request::Command(Command::StartCycle), &mut events);
        runner.advance(0.5, &mut events);
        let friction = runner.sim.signal_infos().iter().position(|s| s.id.ends_with(".friction")).unwrap();
        assert!(samples(&events).last().unwrap()[friction].abs() > 1.0, "{}: not sliding at the moment of the switch", entry.id);

        let mut off = cfg.clone();
        off.set_text("effects.guide_friction = false").unwrap();
        let state_before = runner.sim.state.clone();
        runner.handle(Request::Reconfigure(off), &mut events);
        assert_eq!(runner.sim.state, state_before, "{}: the switch disturbed the running state", entry.id);
        let mut next = Vec::new();
        runner.advance(1e-3, &mut next);
        assert_eq!(samples(&next).len(), 1);
        assert_eq!(samples(&next)[0][friction], 0.0, "{}", entry.id);
    }
}

/// Pause and slow motion: the thread paces the simulation, the core never sees the wall clock.
#[test]
fn the_session_runs_in_real_time_and_can_pause_and_slow_down() {
    let session = Session::start(MACHINES[0], &MACHINES[0].config()).unwrap();
    let sim_time = |wall: Duration| {
        std::thread::sleep(wall);
        let events = session.drain();
        let times: Vec<f64> = samples(&events).iter().map(|s| s[0]).collect();
        assert!(times.windows(2).all(|w| w[1] > w[0]), "time went backwards");
        (times.len(), times.last().copied())
    };
    // Real time: about 0.4 s of samples in 0.4 s (generous bounds, the test machine may be busy).
    let (n, last) = sim_time(Duration::from_millis(400));
    assert!(n > 100 && n < 500, "{n} samples in 0.4 s at real time");
    let t0 = last.unwrap();

    session.send(Request::Speed(0.1));
    sim_time(Duration::from_millis(100));
    let (slow, _) = sim_time(Duration::from_millis(400));
    assert!(slow > 10 && slow < 100, "{slow} samples in 0.4 s at one tenth speed");

    session.send(Request::Pause(true));
    let (_, at_pause) = sim_time(Duration::from_millis(100));
    let (paused, _) = sim_time(Duration::from_millis(200));
    assert_eq!(paused, 0, "samples arrived while paused");
    session.send(Request::Pause(false));
    session.send(Request::Speed(1.0));
    let (resumed, after) = sim_time(Duration::from_millis(200));
    assert!(resumed > 30 && after.unwrap() > at_pause.unwrap_or(t0));

    session.send(Request::Command(Command::MoveTo(vec![])));
    std::thread::sleep(Duration::from_millis(100));
    assert!(session.drain().iter().any(|e| matches!(e, Event::Refused("error-target-out-of-range", _))));
}

/// Better traces: a time window, freezing, statistics per trace, readable units and CSV export.
#[test]
fn scopes_have_a_window_freeze_statistics_and_export() {
    use sim_app::model::display_scale;
    let mut model = Model::new(MACHINES[0], "en");
    model.command(Command::StartCycle);
    let start = Instant::now();
    while model.history.len() < 1500 && start.elapsed() < Duration::from_secs(10) {
        model.poll();
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(model.history.len() >= 1500);

    // The window shows the last second only.
    model.window_s = 1.0;
    let rows: Vec<f64> = model.visible_rows().map(|r| r[0]).collect();
    assert!(rows.len() > 900 && rows.len() <= 1002, "{} rows in a one-second window", rows.len());
    assert!(rows.last().unwrap() - rows[0] <= 1.0 + 1e-9);

    // Statistics are those of the visible rows.
    let torque = model.session.signals.iter().position(|s| s.quantity_key == "quantity-torque").unwrap();
    model.set_frozen(true);
    let values: Vec<f64> = model.visible_rows().map(|r| r[torque]).collect();
    let stats = model.stats(torque).unwrap();
    assert_eq!(stats.min, values.iter().copied().fold(f64::INFINITY, f64::min));
    assert_eq!(stats.max, values.iter().copied().fold(f64::NEG_INFINITY, f64::max));
    assert!((stats.rms - (values.iter().map(|v| v * v).sum::<f64>() / values.len() as f64).sqrt()).abs() < 1e-12);

    // Frozen traces stay as they are while the simulation runs on.
    let frozen_end = *rows.last().unwrap();
    std::thread::sleep(Duration::from_millis(200));
    model.poll();
    assert!(model.visible_rows().last().unwrap()[0] <= frozen_end + 0.2 && model.history.back().unwrap()[0] > model.visible_rows().last().unwrap()[0]);

    // Export writes what the scopes show.
    let path = std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join("traces.csv");
    model.export_traces(&path);
    let csv = std::fs::read_to_string(&path).unwrap();
    assert_eq!(csv.lines().count(), 1 + model.visible_rows().count());
    assert_eq!(csv.lines().next().unwrap().split(',').count(), model.session.signals.len());
    assert_eq!(model.messages.last().unwrap().0, "ui-exported");
    model.set_frozen(false);
    assert!(model.visible_rows().last().unwrap()[0] > frozen_end);

    // 40 micrometres read as 40 um, not as 0.00004 m.
    assert_eq!(display_scale(4e-5, "m"), (1e6, "µ"));
    assert_eq!(display_scale(0.016, "m"), (1e3, "m"));
    assert_eq!(display_scale(250.0, "rad/s"), (1.0, ""));
    assert_eq!(display_scale(0.5, ""), (1.0, ""));
}

/// Tuning help in the app: step and frequency response of an axis, measured in the background
/// with the parameters as they are, and changing when a gain is changed.
#[test]
fn tuning_measures_the_axis_with_the_current_gains() {
    let mut model = Model::new(MACHINES[0], "en");
    let measure = |model: &mut Model| {
        model.run_tuning(0);
        assert!(model.tuning_running());
        let start = Instant::now();
        while model.tuning_running() && start.elapsed() < Duration::from_secs(60) {
            model.poll();
            std::thread::sleep(Duration::from_millis(20));
        }
        let tuning = model.tuning.take().expect("the tuning job returned nothing");
        (tuning.step.rise_time, tuning.bandwidth.unwrap())
    };
    let (rise, band) = measure(&mut model);
    // Kv = 50 1/s: a bandwidth near 8 Hz.
    assert!(band > 4.0 && band < 16.0 && rise > 0.0, "bandwidth {band} Hz, rise time {rise} s");
    model.set_param("drive.kv_1_s", toml::Value::Float(100.0));
    let (faster_rise, wider_band) = measure(&mut model);
    assert!(faster_rise < 0.8 * rise && wider_band > 1.3 * band, "Kv doubled: {rise} -> {faster_rise} s, {band} -> {wider_band} Hz");
}

/// Polls until the analysis jobs have answered, whatever they say.
fn wait_for_jobs(model: &mut Model) {
    let start = Instant::now();
    while model.analysis_running() && start.elapsed() < Duration::from_secs(300) {
        model.poll();
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(!model.analysis_running(), "the analysis did not finish");
}

/// Polls until the analysis jobs have answered, with a result and no complaint.
fn wait_for_analysis(model: &mut Model) {
    wait_for_jobs(model);
    assert!(model.messages.iter().all(|m| m.0.starts_with("limit-")), "{:?}", model.messages);
}

fn number(cell: &str) -> f64 {
    cell.parse().unwrap_or_else(|_| panic!("not a number: {cell}"))
}

/// The cells of the row that starts with `label`, after the label.
fn row<'a>(table: &'a [Vec<String>], label: &str) -> &'a [String] {
    &table.iter().find(|row| row[0] == label).unwrap_or_else(|| panic!("no row {label:?} in {table:?}"))[1..]
}

/// A cell shows this value, to the digits the tables give: four digits and at most three decimals, and
/// three digits for a value below 0.01 or from 100 000 up.
fn shows(cell: &str, value: f64) -> bool {
    let digits = value.abs() < 0.01 || value.abs() >= 1e5;
    (number(cell) - value).abs() <= if digits { 0.01 * value.abs() } else { 0.00051 * value.abs().max(1.0) }
}

/// Polls while the machine in the app runs for `seconds` of its own time.
fn simulate(model: &mut Model, seconds: f64) {
    let until = model.history.back().map_or(0.0, |row| row[0]) + seconds;
    let start = Instant::now();
    while model.history.back().is_none_or(|row| row[0] < until) && start.elapsed() < Duration::from_secs(30) {
        model.poll();
        std::thread::sleep(Duration::from_millis(5));
    }
}

/// The newest value of a signal in the app.
fn latest(model: &Model, id: &str) -> f64 {
    model.history.back().unwrap()[model.session.signals.iter().position(|s| s.id == id).unwrap_or_else(|| panic!("no signal {id}"))]
}

fn temp_file(entry: &MachineEntry, name: &str) -> String {
    std::path::Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("{}-app-{name}", entry.id)).to_str().unwrap().to_string()
}

/// The app shows what the command line computes: the analysed cycle with its headline numbers, the load on
/// each motor and one row per check, for every machine, in the background while the machine runs on, and
/// in both languages. The numbers are those of the analysis functions.
#[test]
fn the_cycle_is_analysed_and_judged_in_the_app() {
    for entry in MACHINES.iter().copied() {
        let mut model = Model::new(entry, "en");
        assert!(model.cycle_tables().is_empty());
        model.run_analysis(Analysis::Cycle);
        assert!(model.analysis_running());
        let before = model.history.len();
        wait_for_analysis(&mut model);
        assert!(model.history.len() > before, "{}: the simulation stood still meanwhile", entry.id);

        let mut sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
        let load = sim_components::analysis::analyse_cycle(&mut sim).unwrap();
        let verdicts = sim_components::analysis::verdicts(&entry.config(), &load).unwrap();
        let tables = model.cycle_tables();
        let [summary, motors, checks] = &tables[..] else { panic!("{} tables", tables.len()) };
        // One figure per row: cycle time, path error and dwell error in mm, lowest belt tension, and what
        // `sim-cli analyze` prints beside them: slack time, radial load, guide load and guide life.
        assert_eq!(summary[0], ["Figure", "Value"]);
        let life = sim_components::analysis::guide_life_km(&entry.config(), &load);
        let expected = [
            ("Cycle time [s]", Some(load.duration)),
            ("Path error, peak [mm]", Some(load.path_error_peak * 1e3)),
            ("Off target while dwelling, peak [mm]", Some(load.dwell_error_peak * 1e3)),
            ("Lowest belt tension [N]", load.belt_tension.map(|t| t.0)),
            ("Time with a slack belt section [s]", load.belt_slack_time),
            ("Radial load on a pulley shaft, peak [N]", load.radial_load_peak),
            ("Highest guide load [N]", load.guide_load.map(|g| g.0)),
            ("Guide life by the catalogue formula [km]", life),
        ];
        assert!(expected.iter().all(|(_, value)| value.is_some()) && load.radial_load_peak.unwrap() > 100.0 && life.unwrap() > 1.0, "{}: {expected:?}", entry.id);
        assert_eq!(summary.len(), 1 + expected.len(), "{}: {summary:?}", entry.id);
        for (label, value) in expected {
            assert!(shows(&row(summary, label)[0], value.unwrap()), "{}: {label} {value:?} in {summary:?}", entry.id);
        }
        // One row per figure and one column per motor: peak, RMS and cubic-mean torque, winding rise, energy
        // drawn and fed back, the braking resistor's energy and mean power and the highest DC link voltage.
        assert_eq!(motors[0][0], "Figure");
        assert_eq!(motors[0][1..], model.session.motors.iter().map(|info| model.tr.tr(info.key).to_string()).collect::<Vec<_>>());
        assert_eq!(motors.len(), 10);
        for (k, m) in load.motors.iter().enumerate() {
            let figures = [
                ("Peak torque [N m]", m.peak_torque),
                ("RMS torque [N m]", m.rms_torque),
                ("Cubic-mean torque [N m]", m.cubic_mean_torque),
                ("Winding rise [K]", m.winding_rise),
                ("Energy drawn [J]", m.energy_in),
                ("Energy fed back [J]", m.energy_regenerated),
                ("Highest DC link voltage [V]", m.peak_dc_link_voltage),
            ];
            for (label, value) in figures {
                assert!(value > 0.0 && shows(&row(motors, label)[k], value), "{}: {label} {value} in {motors:?}", entry.id);
            }
            assert!(m.cubic_mean_torque > m.rms_torque && m.peak_dc_link_voltage > entry.config().f64("drive.bus_voltage_v").unwrap());
            assert!(shows(&row(motors, "Braking resistor, energy [J]")[k], m.resistor_energy) && shows(&row(motors, "Braking resistor, mean power [W]")[k], m.resistor_energy / load.duration));
        }
        // One row per check: label, motor, value, limit, result. The default cycles pass every check.
        assert_eq!(checks.len(), 1 + verdicts.len());
        assert_eq!(checks[0], ["Check", "Motor", "Value", "Limit", "Result"]);
        for (row, v) in checks[1..].iter().zip(&verdicts) {
            assert_eq!(row[0], model.tr.tr(&format!("check-{}", v.check)));
            assert!(model.tr.has(&format!("check-{}", v.check)), "no label for {}", v.check);
            assert_eq!(row[1].is_empty(), v.motor.is_none());
            assert!((number(&row[2]) - v.value).abs() <= 0.01 * v.value.abs() && (number(&row[3]) - v.limit).abs() <= 0.01 * v.limit.abs(), "{row:?} against {v:?}");
            assert_eq!(row[4], "ok");
        }
        assert!(checks.iter().any(|row| row[0] == "Winding rise, settled [K]") && checks.iter().any(|row| row[0] == "Off target while dwelling [m]"));

        // A motor too weak for the cycle fails its check, in the words of the chosen language.
        model.set_language("de");
        model.set_param("motor.rated_current_a", toml::Value::Float(0.02));
        model.run_analysis(Analysis::Cycle);
        wait_for_analysis(&mut model);
        let checks = &model.cycle_tables()[2];
        assert_eq!(checks[0][0], "Prüfung");
        assert!(checks.iter().any(|row| row[0].starts_with("Übertemperatur der Wicklung") && row[4] == "NICHT BESTANDEN"), "{checks:?}");
        model.clear_analysis();
        assert!(model.cycle_tables().is_empty());
    }
}

/// The sensitivity ranking in the app: the values the config gives bounds, each run at its low and its high
/// bound, strongest first, the top rows shown with the parameter panel's names.
#[test]
fn the_app_ranks_the_uncertain_parameters() {
    for entry in MACHINES.iter().copied() {
        let mut model = Model::new(entry, "en");
        assert!(model.sensitivity_table().is_none());
        model.run_analysis(Analysis::Sensitivity);
        wait_for_analysis(&mut model);
        let bounds = sim_components::analysis::bounds(&model.config).unwrap();
        let (nominal, rows) = model.sensitivity.as_ref().unwrap();
        assert_eq!(rows.len(), bounds.len(), "{}", entry.id);
        assert!(rows.windows(2).all(|w| w[0].strength(nominal) >= w[1].strength(nominal)) && rows[0].strength(nominal) > 0.01, "{}", entry.id);
        let table = model.sensitivity_table().unwrap();
        assert_eq!((table.len(), table[0].len()), (9, 6), "{}: {table:?}", entry.id);
        assert_eq!(table[0][0], "Parameter");
        for (shown, row) in table[1..].iter().zip(rows) {
            // "Belt · Pretension [N]": group and parameter as the parameter panel names them.
            let (group, leaf) = row.path.rsplit_once('.').unwrap();
            assert_eq!(shown[0], format!("{} · {}", model.tr.tr(&format!("group-{group}")), model.tr.tr(&format!("param-{leaf}"))));
            assert_eq!((number(&shown[1]), number(&shown[2])), (row.bounds.0, row.bounds.1));
            let peak = row.changes(nominal);
            assert_eq!(shown[3], format!("{:+.1} / {:+.1}", peak[0] * 100.0, peak[1] * 100.0));
        }
    }
}

/// The optimiser in the app: cycle time before and after and the settings it would change; applying them
/// puts them into the config and the running simulation, whose cycle is then as short as promised.
#[test]
fn the_app_optimises_the_cycle_and_applies_the_settings() {
    for entry in MACHINES.iter().copied() {
        let mut model = Model::new(entry, "en");
        model.run_analysis(Analysis::Optimise);
        wait_for_analysis(&mut model);
        let table = model.optimised_table().unwrap();
        let found = model.optimised.clone().unwrap();
        assert_eq!(table[0], ["Setting", "Before", "After"]);
        assert_eq!(table[1][0], "Cycle time [s]");
        let (before, after) = (number(&table[1][1]), number(&table[1][2]));
        assert!(after < 0.95 * before && (before - cycle_time(entry, &entry.config())).abs() < 2e-3, "{}: {before} s to {after} s", entry.id);
        assert!(!found.changes.is_empty() && table.len() == 3 + found.changes.len());
        assert!(table[3..].iter().all(|row| row[1] != row[2] && !row[0].contains("param-") && !row[0].contains("effect-")), "{table:?}");

        // An edit made since the optimiser ran: its settings, absolute values, used to be written over it
        // without a word. Now nothing is applied, with a message, until the edit is taken back.
        let payload = model.config.f64("mass.payload_kg").unwrap();
        model.set_param("mass.payload_kg", toml::Value::Float(payload + 0.5));
        model.apply_optimised();
        assert_eq!(model.messages.last().map(|m| m.0), Some("ui-analysis-stale"), "{}", entry.id);
        assert!(model.optimised_table().is_some());
        for (path, before, _) in &found.changes {
            assert_eq!(&model.config.get(path).unwrap().to_string(), before, "{path} was applied over a changed config");
        }
        model.set_param("mass.payload_kg", toml::Value::Float(payload));

        model.apply_optimised();
        assert!(model.optimised_table().is_none());
        for (path, _, value) in &found.changes {
            assert_eq!(&model.config.get(path).unwrap().to_string(), value, "{path}");
        }
        assert!((cycle_time(entry, &model.config) - after).abs() < 2e-3, "{}", entry.id);
        // The running simulation took the config: no refusal comes back.
        std::thread::sleep(Duration::from_millis(100));
        model.poll();
        assert!(model.messages.iter().all(|m| m.0.starts_with("limit-") || m.0 == "ui-analysis-stale"), "{:?}", model.messages);
        for (path, _, value) in &found.changes {
            assert_eq!(&model.config.get(path).unwrap().to_string(), value, "{path}");
        }
    }
}

/// Disable and Enable from the app: disabled, no motor gives torque and a move is refused with the reason;
/// enabled again, the drives hold and the move runs.
#[test]
fn the_drives_can_be_disabled_and_enabled_from_the_app() {
    for entry in MACHINES.iter().copied() {
        let mut model = Model::new(entry, "en");
        let torques: Vec<usize> = model.session.signals.iter().enumerate().filter(|(_, s)| s.id.ends_with(".torque")).map(|(i, _)| i).collect();
        assert_eq!(torques.len(), model.session.motors.len());
        let run = |model: &mut Model, seconds: f64| {
            let until = model.history.back().map_or(0.0, |row| row[0]) + seconds;
            let start = Instant::now();
            while model.history.back().is_none_or(|row| row[0] < until) && start.elapsed() < Duration::from_secs(10) {
                model.poll();
                std::thread::sleep(Duration::from_millis(5));
            }
        };
        let pushing = |model: &Model, last: usize| model.history.iter().rev().take(last).any(|row| torques.iter().any(|&i| row[i] != 0.0));
        model.command(Command::StartCycle);
        run(&mut model, 0.5);
        assert!(pushing(&model, 100), "{}: no torque in the cycle", entry.id);

        model.command(Command::Disable);
        run(&mut model, 0.3);
        assert!(!pushing(&model, 100), "{}: torque while disabled", entry.id);
        model.command(Command::MoveTo(model.target.clone()));
        run(&mut model, 0.1);
        assert_eq!(model.messages.iter().filter(|m| m.0 == "error-disabled").count(), 1, "{}: {:?}", entry.id, model.messages);

        // Enabled, the move back to the start pose is accepted and takes torque.
        model.command(Command::Enable);
        model.command(Command::MoveTo(model.target.clone()));
        run(&mut model, 0.3);
        assert!(pushing(&model, 100), "{}: no torque after enabling", entry.id);
        assert_eq!(model.messages.iter().filter(|m| m.0.starts_with("error-")).count(), 1, "{}: {:?}", entry.id, model.messages);
    }
}

/// A config value the simulation refuses went into the model's config and stayed there: every later edit was
/// sent with it and refused too, each with the same message again. Now the config goes back to the last one
/// the simulation took, and a message that repeats the last one is not added.
#[test]
fn a_refused_config_value_is_rolled_back_and_reported_once() {
    let mut model = Model::new(MACHINES[0], "en");
    let good = model.config.f64("drive.velocity_cycle_s").unwrap();
    let wait = |model: &mut Model| {
        let start = Instant::now();
        while start.elapsed() < Duration::from_millis(300) {
            model.poll();
            std::thread::sleep(Duration::from_millis(5));
        }
    };
    let errors = |model: &Model| model.messages.iter().filter(|m| m.0.starts_with("error-")).count();
    model.set_param("drive.velocity_cycle_s", toml::Value::Float(1.0e-4));
    wait(&mut model);
    assert_eq!(model.config.f64("drive.velocity_cycle_s").unwrap(), good, "the refused value stayed in the config");
    assert_eq!(errors(&model), 1, "{:?}", model.messages);
    // The next edit is taken.
    model.set_param("mass.payload_kg", toml::Value::Float(3.5));
    wait(&mut model);
    assert_eq!((model.config.f64("mass.payload_kg").unwrap(), errors(&model)), (3.5, 1), "{:?}", model.messages);
    // The same refusal again goes back to the config with that edit, and does not pile up.
    model.set_param("drive.velocity_cycle_s", toml::Value::Float(1.0e-4));
    wait(&mut model);
    assert_eq!((model.config.f64("mass.payload_kg").unwrap(), model.config.f64("drive.velocity_cycle_s").unwrap()), (3.5, good));
    assert_eq!(errors(&model), 1, "{:?}", model.messages);
    // Two edits on the way at once, the first of them refused: the second goes with it, as it was sent with
    // the refused value in it, and the config is again the one the simulation runs.
    model.set_param("drive.velocity_cycle_s", toml::Value::Float(1.0e-4));
    model.set_param("mass.payload_kg", toml::Value::Float(4.5));
    wait(&mut model);
    assert_eq!((model.config.f64("mass.payload_kg").unwrap(), model.config.f64("drive.velocity_cycle_s").unwrap()), (3.5, good));
    // And a second edit that puts the refused value right before the refusal is back: it is taken, and the
    // config shows it, not the config the refusal went back to.
    model.set_param("drive.velocity_cycle_s", toml::Value::Float(1.0e-4));
    model.set_param("drive.velocity_cycle_s", toml::Value::Float(2.0 * good));
    wait(&mut model);
    assert_eq!((model.config.f64("mass.payload_kg").unwrap(), model.config.f64("drive.velocity_cycle_s").unwrap()), (3.5, 2.0 * good));
    // A refused command twice over is one message as well.
    model.command(Command::MoveTo(Vec::new()));
    model.command(Command::MoveTo(Vec::new()));
    wait(&mut model);
    assert_eq!(model.messages.iter().filter(|m| m.0 == "error-target-out-of-range").count(), 1, "{:?}", model.messages);
}

/// Verdicts with error bands, as `sim-cli verdicts --bands` gives them: with the option on, the cycle analysis
/// also runs the cycle with every bounded value at its bounds, and every check gets the lowest and highest
/// value it took and the worst-case result. A result is marked when the config has changed since.
#[test]
fn the_checks_get_error_bands_in_the_app() {
    use sim_components::analysis::{bounds, verdict_bands};
    for entry in MACHINES.iter().copied() {
        let mut model = Model::new(entry, "en");
        model.inputs.bands = true;
        model.run_analysis(Analysis::Cycle);
        assert!(model.running(Analysis::Cycle) && !model.running(Analysis::Sensitivity) && !model.has_result(Analysis::Cycle));
        wait_for_analysis(&mut model);
        assert!(model.has_result(Analysis::Cycle));
        let cfg = entry.config();
        let bands = verdict_bands(entry, &cfg, &bounds(&cfg).unwrap()).unwrap();
        // A fourth table beside the three of the cycle: per check and motor the band and the worst-case result.
        let tables = model.cycle_tables();
        let [_, _, checks, banded] = &tables[..] else { panic!("{} tables", tables.len()) };
        assert_eq!(checks[0].len(), 5);
        assert_eq!(banded[0], ["Check", "Motor", "Lowest", "Highest", "Worst case", "Runs passed [%]"]);
        assert_eq!(banded.len(), 1 + bands.len(), "{}", entry.id);
        for (row, band) in banded[1..].iter().zip(&bands) {
            assert_eq!(row[0], model.tr.tr(&format!("check-{}", band.verdict.check)));
            assert_eq!((row[1].is_empty(), row[4].as_str()), (band.verdict.motor.is_none(), if band.ok { "ok" } else { "FAIL" }));
            assert!(shows(&row[2], band.low) && shows(&row[3], band.high) && shows(&row[5], 100.0 * band.share), "{row:?} against {band:?}");
            assert!(band.low <= band.verdict.value && band.verdict.value <= band.high);
        }
        // The bounds move the checks: a band is not a line.
        assert!(bands.iter().any(|band| band.high > 1.05 * band.low), "{}: {bands:?}", entry.id);
        model.set_language("de");
        assert_eq!(model.cycle_tables()[3][0][2..], ["Kleinster Wert", "Größter Wert", "Ungünstigster Fall", "Bestandene Läufe [%]"]);
        model.set_language("en");
        assert_eq!(model.band_samples, 0);

        // The result is of the config it was computed for, and says so once that has changed.
        assert!(!model.outdated(Analysis::Cycle));
        let payload = model.config.f64("mass.payload_kg").unwrap();
        model.set_param("mass.payload_kg", toml::Value::Float(payload + 0.5));
        assert!(model.outdated(Analysis::Cycle) && !model.outdated(Analysis::Search));
        model.set_param("mass.payload_kg", toml::Value::Float(payload));
        assert!(!model.outdated(Analysis::Cycle));
        // With a number of samples the bands are those of `verdict_bands_sampled` with the seed 1.
        model.inputs.samples = 6;
        model.run_analysis(Analysis::Cycle);
        wait_for_analysis(&mut model);
        let sampled = sim_components::analysis::verdict_bands_sampled(entry, &cfg, &bounds(&cfg).unwrap(), 6, 1).unwrap();
        let tables = model.cycle_tables();
        assert_eq!((model.band_samples, tables[3].len()), (6, 1 + sampled.len()), "{}", entry.id);
        for (row, band) in tables[3][1..].iter().zip(&sampled) {
            assert!(shows(&row[2], band.low) && shows(&row[3], band.high) && shows(&row[5], 100.0 * band.share), "{row:?} against {band:?}");
        }
        assert!(sampled.iter().zip(&bands).any(|(sampled, alone)| sampled.high != alone.high), "{}", entry.id);
        // Without the option there are the three tables again.
        model.inputs.bands = false;
        model.run_analysis(Analysis::Cycle);
        wait_for_analysis(&mut model);
        assert_eq!(model.cycle_tables().len(), 3);
    }
}

/// The motor search in the app, as `sim-cli search`: the motors whose article number starts as asked, each
/// on the cycle, ranked as the library ranks them; a button per row that names the motor in the running
/// config; and inputs that cannot be searched are refused with a message.
#[test]
fn the_app_searches_motors_and_names_one_in_the_config() {
    use sim_components::catalog::search_motors;
    let series = "1FK2104";
    let part = |model: &Model, key: &str| model.config.get(&format!("parts.{key}")).unwrap().as_str().unwrap().to_string();
    let settle = |model: &mut Model| {
        std::thread::sleep(Duration::from_millis(150));
        model.poll();
    };
    for entry in MACHINES.iter().copied() {
        let mut model = Model::new(entry, "en");
        assert!(model.search_table().is_none());
        model.inputs.supply = "110v".into();
        model.run_analysis(Analysis::Search);
        assert!(!model.analysis_running());
        assert_eq!(model.messages.last(), Some(&("ui-search-bad-supply", "110v".to_string())));
        model.inputs.supply = "400v".into();
        model.inputs.series = "9XX".into();
        model.run_analysis(Analysis::Search);
        wait_for_jobs(&mut model);
        assert_eq!(model.messages.last(), Some(&("ui-search-none", "9XX".to_string())));
        assert!(model.search_table().is_none() && !model.has_result(Analysis::Search));
        model.messages.clear();

        model.inputs.series = series.into();
        model.run_analysis(Analysis::Search);
        wait_for_analysis(&mut model);
        let found = search_motors(entry, &entry.config(), "400v", &|article| article.starts_with(series), None);
        let table = model.search_table().unwrap();
        assert_eq!(table[0], ["Motor", "Gearbox", "Mass [kg]", "Result", "Tightest check", "Value / limit"]);
        assert!(found.len() >= 3 && table.len() == 1 + found.len(), "{}: {table:?}", entry.id);
        for (row, c) in table[1..].iter().zip(&found) {
            assert_eq!((row[0].as_str(), row[1].as_str(), row[3].as_str()), (c.article, "", if c.passes() { "ok" } else { "FAIL" }));
            assert!(shows(&row[2], c.mass), "{row:?}");
            match c.tightest() {
                Some((check, share)) => assert!(row[4] == model.tr.tr(&format!("check-{check}")) && shows(&row[5], share), "{row:?}"),
                // Not run: the reason in words, and no figure.
                None => assert!(row[4].contains(' ') && row[5].is_empty(), "{row:?}"),
            }
        }
        // A 2000 r/min motor is too slow for the axis speed by its catalogue numbers alone: it is listed as
        // failed with that reason, and no cycle is simulated for it.
        model.inputs.series = "1FK2208-3AC".into();
        model.run_analysis(Analysis::Search);
        wait_for_analysis(&mut model);
        assert_eq!(model.search_table().unwrap()[1][3..], ["FAIL", "Motor speed at the highest axis speed", ""]);
        model.inputs.series = series.into();
        model.run_analysis(Analysis::Search);
        wait_for_analysis(&mut model);

        // The button of the first row: its motor is named in the config, and the running simulation takes it.
        model.use_candidate(0);
        settle(&mut model);
        assert_eq!((part(&model, "motor"), part(&model, "motor_supply"), part(&model, "gearbox")), (found[0].article.to_string(), "400v".to_string(), String::new()));
        assert!(model.messages.iter().all(|m| m.0.starts_with("limit-")), "{:?}", model.messages);
        // Naming a motor does not outdate the search, which put its own motors in: another row can follow.
        assert!(!model.outdated(Analysis::Search));
        model.use_candidate(1);
        settle(&mut model);
        assert_eq!(part(&model, "motor"), found[1].article);
        // A row that is not there does nothing.
        model.use_candidate(found.len());
        assert_eq!(part(&model, "motor"), found[1].article);
        // Any other change does: the motors passed or failed on another machine, so none of them is applied.
        let payload = model.config.f64("mass.payload_kg").unwrap();
        model.set_param("mass.payload_kg", toml::Value::Float(payload + 0.5));
        assert!(model.outdated(Analysis::Search));
        model.use_candidate(0);
        settle(&mut model);
        assert_eq!((model.messages.last().map(|m| m.0), part(&model, "motor")), (Some("ui-analysis-stale"), found[1].article.to_string()));
        model.clear(Analysis::Search);
        assert!(model.search_table().is_none() && !model.outdated(Analysis::Search));
    }

    // With a gearbox series every motor is tried with the gearboxes of the series that fit it, and the
    // button names both.
    let entry = MACHINES[0];
    let mut model = Model::new(entry, "de");
    (model.inputs.series, model.inputs.gearboxes) = ("1FK2104-5AK".into(), "NRK070".into());
    model.run_analysis(Analysis::Search);
    wait_for_analysis(&mut model);
    let found = search_motors(entry, &entry.config(), "400v", &|article| article == "1FK2104-5AK", Some("NRK070"));
    let table = model.search_table().unwrap();
    assert_eq!((table[0][1].as_str(), table[0][4].as_str()), ("Getriebe", "Knappste Prüfung"));
    assert!(found.len() >= 3 && table.len() == 1 + found.len(), "{table:?}");
    for (row, c) in table[1..].iter().zip(&found) {
        assert!(row[1] == c.gearbox && row[1].starts_with("NRK070-04-"), "{row:?}");
    }
    model.use_candidate(0);
    settle(&mut model);
    assert_eq!((part(&model, "motor"), part(&model, "gearbox")), ("1FK2104-5AK".to_string(), found[0].gearbox.clone()));
    assert!(model.messages.iter().all(|m| m.0.starts_with("limit-")), "{:?}", model.messages);
    // A gearbox series the catalogue does not have: the message names the series. It used to say that the
    // catalogue had no motor whose article number starts with 1FK2104-5AK, which it has.
    model.inputs.gearboxes = "XYZ".into();
    model.run_analysis(Analysis::Search);
    wait_for_jobs(&mut model);
    assert_eq!(model.messages.last(), Some(&("ui-search-no-gearbox", "XYZ".to_string())));
}

/// Paths in the app, as `sim-cli run` and `analyze` with `--path` and `--trajectory`: a file of lines is
/// planned from where the machine stands, within the axis limits or as fast as the motors allow, analysed
/// like a cycle, and run by the machine in the app; a file of poses at times is run as it is. The planned
/// path can be drawn. Files that are missing or cannot be read, a path that starts elsewhere and a config
/// changed while the plan was made are answered with a message, and then the machine does not move.
#[test]
fn a_path_is_planned_run_and_analysed_in_the_app() {
    use sim_components::analysis::{analyse_trajectory, verdicts};
    for entry in MACHINES.iter().copied() {
        let mut model = Model::new(entry, "en");
        wait_for_data(&mut model);
        let axes = model.session.axes.clone();
        let point = |share: f64| -> Vec<f64> { axes.iter().map(|a| a.min + share * (a.max - a.min)).collect() };
        let text = |pose: &[f64]| pose.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(",");
        let position = format!("axis.{}.position", axes[0].id);
        let home = entry.config().f64s("start.pose").unwrap();
        let (lines, timed) = (temp_file(entry, "path.csv"), temp_file(entry, "timed.csv"));
        std::fs::write(&lines, format!("# two lines\nline,{}\nline,{}\n", text(&point(0.5)), text(&point(0.75)))).unwrap();
        assert!(model.path_tables().is_empty() && model.path_plot().is_none());
        model.inputs.path_file = lines.clone();
        model.run_analysis(Analysis::Path);
        wait_for_analysis(&mut model);

        // The same by the library, from the start pose, where the machine in the app stands.
        let mut sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
        let pieces = sim_components::path::parse(&std::fs::read_to_string(&lines).unwrap()).unwrap();
        let points = sim.plan_path(&pieces, false).unwrap();
        let load = analyse_trajectory(&mut sim, points.clone()).unwrap();
        let judged = verdicts(&entry.config(), &load).unwrap();
        let tables = model.path_tables();
        let [summary, motors, checks] = &tables[..] else { panic!("{} tables", tables.len()) };
        let slow = number(&row(summary, "Duration [s]")[0]);
        // The app plans from where its machine stands, a few micrometres from the config's start pose, so
        // the plan can be a cycle longer and what depends on the first milliseconds differs in the last digits.
        assert!((slow - load.duration).abs() < 2e-3 && load.duration > 1.0, "{}: {} s in {summary:?}", entry.id, load.duration);
        let behind = number(&row(summary, "Behind the setpoint, peak [mm]")[0]);
        assert!(load.tracking_error_peak > 1e-5 && (behind / (load.tracking_error_peak * 1e3) - 1.0).abs() < 0.05, "{}: {} m in {summary:?}", entry.id, load.tracking_error_peak);
        assert_eq!(row(summary, "Planned path off the given one, at most [mm]")[0], "0.000");
        // A path has no cycle time and no dwell.
        assert!(summary.iter().all(|row| row[0] != "Cycle time [s]" && !row[0].starts_with("Off target")));
        for (k, m) in load.motors.iter().enumerate() {
            assert!((number(&row(motors, "Peak torque [N m]")[k]) / m.peak_torque - 1.0).abs() < 0.02, "{}: {} N m in {motors:?}", entry.id, m.peak_torque);
        }
        assert_eq!(checks.len(), 1 + judged.len());
        for (row, v) in checks[1..].iter().zip(&judged) {
            assert_eq!((row[0].as_str(), row[4].as_str()), (model.tr.tr(&format!("check-{}", v.check)), if v.ok { "ok" } else { "FAIL" }));
        }
        // The planned path for the plot: every pose, from where the machine stood to the end of the file.
        let (across, up, line) = model.path_plot().unwrap();
        assert!(!across.is_empty() && up.ends_with("[m]") && line.len().abs_diff(points.len()) <= 1, "{across}, {up}, {} points", line.len());
        let (first, last) = (line[0], line[line.len() - 1]);
        if axes.len() == 1 {
            assert!(first[0] == 0.0 && (first[1] - home[0]).abs() < 1e-4 && (last[1] - point(0.75)[0]).abs() < 1e-9, "{first:?} to {last:?}");
        } else {
            assert!((first[0] - home[0]).abs() < 1e-4 && (last[0] - point(0.75)[0]).abs() < 1e-9 && (last[1] - point(0.75)[1]).abs() < 1e-9, "{first:?} to {last:?}");
        }
        // The same path as places in the 3D scene, from where the scene draws the tool now to where the
        // machine's own mapping puts the end of the file.
        let run = model.path_run.as_ref().unwrap();
        let apart = |a: [f64; 3], b: [f64; 3]| (0..3).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>().sqrt();
        assert_eq!(run.scene_line.len(), run.points.len(), "{}", entry.id);
        let (planned_start, planned_end) = (run.scene_line[0], run.scene_line[run.scene_line.len() - 1]);
        assert!(apart(planned_end, sim.machine.scene_point(&point(0.75)).unwrap()) < 1e-9 && apart(planned_start, planned_end) > 0.1, "{}", entry.id);
        // The trail starts once the machine has taken the path, not when it is sent.
        let sent = Instant::now();
        while model.trail.is_none() && sent.elapsed() < Duration::from_secs(5) {
            model.poll();
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(model.trail.is_some(), "{}", entry.id);
        // And the machine in the app has been sent on its way: it arrives.
        simulate(&mut model, load.duration);
        assert!((latest(&model, &position) - point(0.75)[0]).abs() < 1e-3, "{}: the machine stands at {}", entry.id, latest(&model, &position));
        // The path the tool took, as the scenes reported it on the way: from the start of the planned line to
        // its end, never far from it.
        let trail = model.trail.clone().unwrap();
        let line = &model.path_run.as_ref().unwrap().scene_line;
        let off = trail.iter().map(|p| line.iter().map(|q| apart(*p, *q)).fold(f64::INFINITY, f64::min)).fold(0.0, f64::max);
        // A point per 0.1 mm of travel whatever the frame rate: no two further apart than that and one step.
        let widest = trail.windows(2).map(|w| apart(w[0], w[1])).fold(0.0, f64::max);
        assert!(widest < 3e-4, "{}: {widest} m between two points of the trail", entry.id);
        assert!(trail.len() >= 3 && apart(trail[0], planned_start) < 1e-3 && apart(trail[trail.len() - 1], planned_end) < 2e-3 && off < 5e-3, "{}: {} points, {off} m off the planned line, {} m from its start, {} m from its end", entry.id, trail.len(), apart(trail[0], planned_start), apart(trail[trail.len() - 1], planned_end));

        // Poses at times: the same path backwards, from where the machine stands now.
        let back: String = points.iter().zip(points.iter().rev()).map(|((time, _), (_, pose))| format!("{time:?},{}\n", text(pose))).collect();
        std::fs::write(&timed, format!("time,pose\n{back}")).unwrap();
        (model.inputs.path_file, model.inputs.path_timed) = (timed.clone(), true);
        model.run_analysis(Analysis::Path);
        wait_for_analysis(&mut model);
        assert!((number(&row(&model.path_tables()[0], "Duration [s]")[0]) - load.duration).abs() < 2e-3, "{}: {:?}", entry.id, model.path_tables()[0]);
        simulate(&mut model, load.duration);
        assert!((latest(&model, &position) - home[0]).abs() < 1e-3, "{}: the machine stands at {}", entry.id, latest(&model, &position));

        // What cannot be run says why, and leaves the last result and the machine alone: poses that start
        // where the machine is not, a file that is not there, and a line that is no piece of a path.
        model.run_analysis(Analysis::Path);
        wait_for_jobs(&mut model);
        assert_eq!(model.messages.last().map(|m| m.0), Some("error-trajectory-start"), "{:?}", model.messages);
        (model.inputs.path_file, model.inputs.path_timed) = (temp_file(entry, "no-such-file.csv"), false);
        model.run_analysis(Analysis::Path);
        wait_for_jobs(&mut model);
        let message = model.messages.last().unwrap();
        assert!(message.0 == "ui-file-unreadable" && message.1.contains("no-such-file.csv"), "{message:?}");
        std::fs::write(&timed, format!("line,{}\ncircle,1,2\n", text(&point(0.5)))).unwrap();
        model.inputs.path_file = timed.clone();
        model.run_analysis(Analysis::Path);
        wait_for_jobs(&mut model);
        let message = model.messages.last().unwrap();
        assert!(message.0 == "ui-file-line" && message.1.ends_with("circle,1,2"), "{message:?}");
        assert!(model.tr.tr_arg(message.0, &message.1).starts_with("This line of the file cannot be read: "));
        simulate(&mut model, 0.2);
        assert!(model.path_tables().len() == 3 && (latest(&model, &position) - home[0]).abs() < 1e-3);

        // As fast as the motors allow, and the config changed while the plan was made: the analysis is shown,
        // marked as of an earlier config, and the machine does not run a path planned for another config.
        (model.inputs.path_file, model.inputs.time_optimal) = (lines.clone(), true);
        model.messages.clear();
        model.run_analysis(Analysis::Path);
        let payload = model.config.f64("mass.payload_kg").unwrap();
        model.set_param("mass.payload_kg", toml::Value::Float(payload + 0.5));
        wait_for_jobs(&mut model);
        assert_eq!(model.messages.last().map(|m| m.0), Some("ui-analysis-stale"), "{:?}", model.messages);
        let fast = number(&row(&model.path_tables()[0], "Duration [s]")[0]);
        assert!(fast < slow && model.outdated(Analysis::Path), "{}: {fast} s as fast as the motors allow, {slow} s within the axis limits", entry.id);
        simulate(&mut model, 0.5);
        assert!((latest(&model, &position) - home[0]).abs() < 1e-3, "{}: the machine ran a path planned for another config", entry.id);
        model.set_param("mass.payload_kg", toml::Value::Float(payload));
        assert!(!model.outdated(Analysis::Path));
        model.clear(Analysis::Path);
        assert!(model.path_tables().is_empty() && model.path_plot().is_none());

        // From an end of the strokes, where a move has taken the machine: the plan starts where the motors
        // of a machine at rest there stand, some hundredths of a millimetre beyond the travel range, and was
        // refused for that (`error-trajectory`). It is run, and so is the path back onto the ends.
        let ends: Vec<f64> = axes.iter().map(|a| a.max).collect();
        model.messages.clear();
        model.set_speed(20.0);
        model.command(Command::MoveTo(ends.clone()));
        simulate(&mut model, 4.0);
        assert!((latest(&model, &position) - ends[0]).abs() < 1e-3);
        for (name, to) in [("from-the-ends.csv", point(0.5)), ("onto-the-ends.csv", ends.clone())] {
            let file = temp_file(entry, name);
            std::fs::write(&file, format!("line,{}
", text(&to))).unwrap();
            (model.inputs.path_file, model.inputs.time_optimal) = (file, false);
            model.run_analysis(Analysis::Path);
            wait_for_analysis(&mut model);
            simulate(&mut model, 4.0);
            assert!((latest(&model, &position) - to[0]).abs() < 1e-3, "{}: {name}: the machine stands at {}", entry.id, latest(&model, &position));
        }

        // Transition curves, where the machine has the two axes an arc needs: a line, a quarter turn of
        // 50 mm radius and a line, towards the middle of the workspace. With a tolerance of 0.1 mm the
        // planned path is off by the 0.1 mm the library reports, which the table shows, and takes less time.
        if axes.len() < 2 {
            continue;
        }
        let towards = |i: usize| (0.5 * (axes[i].min + axes[i].max) - home[i]).signum();
        let (dx, dz) = (towards(0), towards(1));
        let corner = [home[0] + 0.15 * dx, home[1]];
        let arc_end = [corner[0] + 0.05 * dx, corner[1] + 0.05 * dz];
        let turn = if dx * dz > 0.0 { "ccw" } else { "cw" };
        let rounded = temp_file(entry, "rounded.csv");
        let pieces = format!("line,{}\narc,{},{},{turn}\nline,{}\n", text(&corner), text(&arc_end), text(&[corner[0], arc_end[1]]), text(&[arc_end[0], arc_end[1] + 0.1 * dz]));
        std::fs::write(&rounded, &pieces).unwrap();
        let planned = |tolerance: f64| -> (f64, f64) {
            let mut model = Model::new(entry, "en");
            wait_for_data(&mut model);
            model.set_param("profile.path_tolerance_m", toml::Value::Float(tolerance));
            model.inputs.path_file = rounded.clone();
            model.run_analysis(Analysis::Path);
            wait_for_analysis(&mut model);
            let summary = &model.path_tables()[0];
            (number(&row(summary, "Duration [s]")[0]), number(&row(summary, "Planned path off the given one, at most [mm]")[0]))
        };
        let mut blended = entry.config();
        blended.set_text("profile.path_tolerance_m = 1.0e-4").unwrap();
        let reported = sim_components::Sim::new(entry, &blended).unwrap().path_deviation(&sim_components::path::parse(&pieces).unwrap()).unwrap();
        let ((given, none), (shorter, off)) = (planned(0.0), planned(1e-4));
        assert!(none == 0.0 && (reported - 1e-4).abs() < 1e-9 && (off - reported * 1e3).abs() < 1e-3, "{}: off by {off} mm, reported {reported} m", entry.id);
        assert!(shorter < 0.95 * given, "{}: {shorter} s with transition curves, {given} s without", entry.id);
    }
}

/// Trace replay and fit in the app, as `sim-cli replay` and `fit --trace`: a trace the simulator wrote itself
/// replays onto itself and fits the config's own values, with the advice in the chosen language. A file
/// that is no trace is refused with a message.
#[test]
fn a_trace_is_replayed_and_fitted_in_the_app() {
    for entry in MACHINES.iter().copied() {
        // A trace as a run writes it: one pass of the cycle, a row per millisecond. The gripper stays
        // empty, because a trace does not say when a payload comes and goes.
        let mut cfg = entry.config();
        cfg.set_text("effects.payload = false").unwrap();
        let mut sim = sim_components::Sim::new(entry, &cfg).unwrap();
        sim.command(Command::StartCycle).unwrap();
        let header: Vec<String> = sim.signal_infos().iter().map(|s| format!("{}[{}]", s.id, s.unit)).collect();
        let (mut rows, mut sample, every) = (vec![header.join(",")], Vec::new(), (1e-3 / sim.dt).round() as u64);
        for step in 0..=(6.2 / sim.dt).round() as u64 {
            if step % every == 0 {
                sim.sample(&mut sample);
                rows.push(sample.iter().map(|v| format!("{v:.9e}")).collect::<Vec<_>>().join(","));
            }
            sim.step();
        }
        let file = temp_file(entry, "trace.csv");
        std::fs::write(&file, rows.join("\n")).unwrap();
        let measured = sim_components::trace::Trace::parse(&rows.join("\n"), sim.machine.motors()).unwrap();

        let mut model = Model::new(entry, "en");
        assert!(model.trace_tables().is_empty() && model.trace_advice().is_empty());
        model.set_param("effects.payload", toml::Value::Boolean(false));
        model.inputs.trace_file = file.clone();
        model.run_analysis(Analysis::Trace);
        wait_for_analysis(&mut model);
        let tables = model.trace_tables();
        let [replayed, fitted] = &tables[..] else { panic!("{} tables", tables.len()) };
        // Per motor the difference between trace and replay: within 0.5 % (RMS) and 5 % (peak) of the largest torque.
        assert_eq!((replayed.len(), replayed[0][0].as_str(), replayed[0].len()), (5, "Figure", 1 + measured.motors.len()));
        for (k, motor) in measured.motors.iter().enumerate() {
            let largest = motor.torque.iter().fold(0.0, |a: f64, b| a.max(b.abs()));
            let (rms, peak) = (number(&row(replayed, "Torque, RMS [N m]")[k]), number(&row(replayed, "Torque, peak [N m]")[k]));
            assert!(largest > 0.1 && rms > 0.0 && rms < 0.005 * largest && peak < 0.05 * largest, "{}: RMS {rms}, peak {peak} of {largest} N m", entry.id);
            assert!(number(&row(replayed, "Following error, peak [rad]")[k]) < 0.01, "{}: {replayed:?}", entry.id);
        }
        // Per axis the four fitted values next to the config's own: on its own trace the fit finds them.
        assert_eq!(fitted[0], ["Quantity", "Configuration", "Fitted", "Standard error"]);
        assert_eq!(fitted.len(), 1 + 4 * model.session.axes.len(), "{}: {fitted:?}", entry.id);
        for axis in &model.session.axes {
            for quantity in ["Moved mass [kg]", "Coulomb friction [N]"] {
                let cells = row(fitted, &format!("{} · {quantity}", model.tr.tr(axis.key)));
                let (config, fit) = (number(&cells[0]), number(&cells[1]));
                assert!(config > 0.0 && (fit / config - 1.0).abs() < 0.02 && number(&cells[2]) > 0.0, "{}: {quantity} {cells:?}", entry.id);
            }
        }
        // What to change in the config, four lines per axis, in the chosen language.
        let advice = model.trace_advice();
        assert!(advice.len() == 4 * model.session.axes.len() && advice[0].starts_with("Moved mass, axis ") && advice[0].contains(" kg ± "), "{advice:?}");
        model.set_language("de");
        assert!(model.trace_advice()[0].starts_with("Bewegte Masse, Achse ") && model.trace_tables()[1][0][0] == "Größe" && model.trace_tables()[0][1][0] == "Moment, Effektivwert [N m]");
        assert!(!model.outdated(Analysis::Trace));
        model.set_param("effects.payload", toml::Value::Boolean(true));
        assert!(model.outdated(Analysis::Trace));

        // A file that is not there, a file that is no trace and a trace too short to replay.
        let refused = |model: &mut Model, file: String| {
            model.inputs.trace_file = file;
            model.run_analysis(Analysis::Trace);
            wait_for_jobs(model);
            model.messages.last().unwrap().clone()
        };
        assert_eq!(refused(&mut model, temp_file(entry, "no-such-trace.csv")).0, "ui-file-unreadable");
        let other = temp_file(entry, "no-trace.csv");
        std::fs::write(&other, "line,0.5\n").unwrap();
        assert_eq!(refused(&mut model, other.clone()).0, "error-trace-column");
        std::fs::write(&other, rows[..4].join("\n")).unwrap();
        assert_eq!(refused(&mut model, other).0, "error-trace-short");
        // The result that was there stays.
        assert_eq!(model.trace_tables().len(), 2);
    }
}

/// A drive that trips on DC link overvoltage takes all drives off, which used to show only as a machine that
/// stops. Now it is said, once per trip, for the machine in the app, and for an analysed cycle that trips.
/// The DC link here has a hundredth of its capacitors and a resistor that takes 5 W, so the first hard
/// braking is too much for it.
#[test]
fn a_drive_that_trips_is_reported() {
    let weak = [("ratings.dc_link_capacitance_uf", 4.7), ("ratings.braking_resistor_peak_w", 5.0)];
    let trips = |events: &[Event]| events.iter().filter(|e| matches!(e, Event::Refused("ui-drive-tripped", _))).count();
    for entry in MACHINES.iter().copied() {
        let mut cfg = entry.config();
        weak.iter().for_each(|(path, value)| cfg.set_text(&format!("{path} = {value:?}")).unwrap());
        let mut runner = Runner::new(entry, &cfg).unwrap();
        let mut events = Vec::new();
        runner.handle(Request::Command(Command::StartCycle), &mut events);
        runner.advance(5.0, &mut events);
        assert!(trips(&events) == 1 && !runner.sim.control.enabled(), "{}: {} trips reported", entry.id, trips(&events));
        // Enabled and sent off again, the next trip is a new one.
        events.clear();
        runner.handle(Request::Command(Command::Enable), &mut events);
        runner.advance(0.5, &mut events);
        assert!(trips(&events) == 0 && runner.sim.control.enabled(), "{}: {events:?}", entry.id);
        runner.handle(Request::Command(Command::StartCycle), &mut events);
        runner.advance(5.0, &mut events);
        assert_eq!(trips(&events), 1, "{}", entry.id);

        let mut model = Model::new(entry, "en");
        weak.iter().for_each(|(path, value)| model.set_param(path, toml::Value::Float(*value)));
        model.run_analysis(Analysis::Cycle);
        wait_for_jobs(&mut model);
        assert_eq!(model.messages.iter().filter(|m| m.0 == "ui-analysis-tripped").count(), 1, "{}: {:?}", entry.id, model.messages);
        assert!(model.cycle_tables()[2].iter().any(|row| row[0] == "Highest DC link voltage [V]" && row[4] == "FAIL"), "{}: {:?}", entry.id, model.cycle_tables()[2]);
        model.set_speed(10.0);
        model.command(Command::StartCycle);
        simulate(&mut model, 8.0);
        assert_eq!(model.messages.iter().filter(|m| m.0 == "ui-drive-tripped").count(), 1, "{}: {:?}", entry.id, model.messages);
        for lang in LANGUAGES {
            model.set_language(lang);
            assert!(model.tr.has("ui-drive-tripped") && model.tr.has("ui-analysis-tripped"));
        }
    }
}

/// The travelled path for the 3D view. From the simulation steps, not from the frames: a point per 0.1 mm of
/// travel however long a frame takes (it was one point per frame, 13 mm apart at speed). It starts when the
/// machine takes a trajectory and not when it refuses one, ends on a reconfigure and on the start of a
/// cycle, and at its cap of points drops the oldest instead of standing still, in the queue and in the model.
#[test]
fn the_travelled_path_follows_every_step_and_starts_and_ends_with_the_path() {
    use sim_app::session::{Outbox, TRAIL, extend_trail};
    let apart = |a: [f64; 3], b: [f64; 3]| (0..3).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>().sqrt();
    let starts = |events: &[Event]| events.iter().filter_map(|e| if let Event::TrailStart(at) = e { Some(*at) } else { None }).collect::<Vec<_>>();
    let trailed = |events: &[Event]| events.iter().filter_map(|e| if let Event::Trail(points) = e { Some(points.clone()) } else { None }).flatten().collect::<Vec<_>>();
    for entry in MACHINES.iter().copied() {
        let mut runner = Runner::new(entry, &entry.config()).unwrap();
        let axes = runner.sim.machine.axes().to_vec();
        let far: Vec<f64> = axes.iter().map(|a| a.min + 0.75 * (a.max - a.min)).collect();
        let points = runner.sim.plan_path(&[sim_components::path::Piece::Line(far.clone())], false).unwrap();
        let duration = points[points.len() - 1].0;

        // Refused (it starts elsewhere): no trail, and none while the machine stands.
        let mut events = Vec::new();
        let elsewhere = points.iter().map(|(t, _)| (*t, far.clone())).collect();
        runner.handle(Request::Command(Command::Trajectory(elsewhere)), &mut events);
        runner.advance(0.05, &mut events);
        assert!(matches!(events[0], Event::Refused("error-trajectory-start", _)) && starts(&events).is_empty() && trailed(&events).is_empty(), "{}", entry.id);

        // Taken: the trail starts where the tool stands and follows it in frames of 16 ms.
        events.clear();
        let from = runner.sim.tool_point().unwrap();
        runner.handle(Request::Command(Command::Trajectory(points)), &mut events);
        assert_eq!(starts(&events), [Some(from)], "{}", entry.id);
        for _ in 0..((duration + 0.3) / 0.016) as usize {
            runner.advance(0.016, &mut events);
        }
        let (trail, to) = (trailed(&events), runner.sim.machine.scene_point(&far).unwrap());
        let widest = std::iter::once(&from).chain(&trail).zip(&trail).map(|(a, b)| apart(*a, *b)).fold(0.0, f64::max);
        // 0.1 mm and what the tool travels in one step.
        assert!(widest > 1e-4 && widest < 2e-4, "{}: {widest} m between two points", entry.id);
        assert!(trail.len() as f64 > 0.9 * apart(from, to) / 2e-4 && apart(trail[trail.len() - 1], to) < 1e-3, "{}: {} points over {} m", entry.id, trail.len(), apart(from, to));

        // The trail ends with the path: a move after it adds no points and says nothing, and the finished
        // trail stays until the reconfigure below.
        events.clear();
        let back: Vec<f64> = axes.iter().map(|a| a.min + 0.25 * (a.max - a.min)).collect();
        runner.handle(Request::Command(Command::MoveTo(back)), &mut events);
        runner.advance(1.0, &mut events);
        assert!(apart(runner.sim.tool_point().unwrap(), to) > 0.01, "{}: the move did not run", entry.id);
        assert!(starts(&events).is_empty() && trailed(&events).is_empty(), "{}: {} points from a move after the path", entry.id, trailed(&events).len());

        // A reconfigure ends it, and so does a cycle after the next path; a cycle with no trail says nothing.
        events.clear();
        runner.handle(Request::Reconfigure(entry.config()), &mut events);
        assert_eq!(starts(&events), [None], "{}", entry.id);
        events.clear();
        runner.handle(Request::Command(Command::StartCycle), &mut events);
        runner.advance(0.5, &mut events);
        assert!(starts(&events).is_empty() && trailed(&events).is_empty(), "{}: {:?}", entry.id, starts(&events));
        let mut runner = Runner::new(entry, &entry.config()).unwrap();
        let points = runner.sim.plan_path(&[sim_components::path::Piece::Line(far.clone())], false).unwrap();
        events.clear();
        runner.handle(Request::Command(Command::Trajectory(points)), &mut events);
        runner.advance(duration + 0.3, &mut events);
        events.clear();
        runner.handle(Request::Command(Command::StartCycle), &mut events);
        runner.advance(0.5, &mut events);
        assert!(starts(&events) == [None] && trailed(&events).is_empty(), "{}: {:?}", entry.id, starts(&events));
    }

    // At the cap the oldest points go: in a trail, and in the queue of a window that does not read.
    let numbered = |from: usize, to: usize| (from..to).map(|n| [n as f64, 0.0, 0.0]).collect::<Vec<_>>();
    let mut trail = numbered(0, TRAIL - 10);
    extend_trail(&mut trail, &numbered(TRAIL - 10, TRAIL + 25));
    assert_eq!((trail.len(), trail[0][0], trail[TRAIL - 1][0]), (TRAIL, 25.0, (TRAIL + 24) as f64));
    let mut outbox = Outbox::default();
    outbox.push(Event::Trail(numbered(0, 5)));
    outbox.push(Event::TrailStart(Some([0.0; 3])));
    for chunk in 0..TRAIL / 1000 + 2 {
        outbox.push(Event::Sample(vec![chunk as f64]));
        outbox.push(Event::Trail(numbered(1000 * chunk, 1000 * (chunk + 1))));
    }
    let events = outbox.drain();
    // The points of the path before the start are gone, the rest is one event after the start.
    assert!(matches!(events[0], Event::TrailStart(Some(_))) && events.iter().filter(|e| matches!(e, Event::Trail(_))).count() == 1);
    let waiting = trailed(&events);
    assert_eq!((waiting.len(), waiting[0][0], waiting[TRAIL - 1][0]), (TRAIL, 2000.0, (TRAIL + 1999) as f64));
}
