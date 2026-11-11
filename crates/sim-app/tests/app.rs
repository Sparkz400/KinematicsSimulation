//! The app without its window. Phase 4 is done when switching machine or language needs no code
//! change, and switching one effect off changes the traces within one cycle.

use sim_app::model::Model;
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
    cfg.set_text("drive.bus_voltage_v = 100.0").unwrap();
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
            assert!(!answer.iter().any(|e| matches!(e, Event::Refused(..))), "{}: {effect} was refused", entry.id);

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
