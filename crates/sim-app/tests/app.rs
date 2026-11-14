//! The app without its window. Phase 4 is done when switching machine or language needs no code
//! change, and switching one effect off changes the traces within one cycle.

use sim_app::model::{Analysis, Model};
use sim_app::session::{Event, Request, Runner, Session};
use sim_components::Command;
use sim_core::{Config, MachineEntry};
use sim_machines::{LANGUAGES, MACHINES};
use std::time::{Duration, Instant};

/// The config's payload value: `mass.payload_kg` where the machine has a [mass] table, else the arms' payload or the
/// rotary table's payload inertia (the slides have one; the arms and the table have none).
fn payload_key(cfg: &Config) -> &'static str {
    ["mass.payload_kg", "arm.payload_kg", "table.payload_inertia_kgm2"]
        .into_iter()
        .find(|k| cfg.get(k).is_ok())
        .unwrap()
}

/// Polls until the simulation thread has delivered samples and a scene.
fn wait_for_data(model: &mut Model) {
    let start = Instant::now();
    while (model.history.len() < 20 || model.scene.parts.is_empty()) && start.elapsed() < Duration::from_secs(60) {
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
        // A quantity has a scope per unit (the SCARA's positions in rad and its quill's in m), so they are counted over all.
        for (quantity, references) in
            [("quantity-position", model.session.axes.len()), ("quantity-torque", model.session.motors.len())]
        {
            let lines: Vec<_> =
                model.scopes.iter().filter(|s| s.quantity_key == quantity).flat_map(|s| &s.lines).collect();
            assert_eq!(lines.iter().filter(|l| l.reference).count(), references, "{}: {quantity}", entry.id);
            assert_eq!(lines.iter().filter(|l| !l.reference).count(), references, "{}: {quantity}", entry.id);
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
    ["src/main.rs", "src/ui.rs"]
        .map(|f| std::fs::read_to_string(std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(f)).unwrap())
        .concat()
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
    let ui_keys: Vec<String> = string_literals(&window_sources())
        .into_iter()
        .filter(|l| (l.starts_with("ui-") || l.starts_with("lang-")) && !l.contains('{'))
        .collect();
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
            seen.push((
                model.tr.tr(entry.key).to_string(),
                model.tr.tr("ui-parameters").to_string(),
                model.signal_label(&model.session.signals[2]),
            ));
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
    while model.messages.len() < 2 && start.elapsed() < Duration::from_secs(60) {
        model.poll();
        std::thread::sleep(Duration::from_millis(5));
    }
    let keys: Vec<&str> = model.messages.iter().map(|m| m.0).collect();
    assert!(keys.contains(&"error-config-missing") && keys.contains(&"error-config-invalid"), "{keys:?}");
    let before = model.history.len();
    wait_for_data(&mut model);
    let start = Instant::now();
    while model.history.len() <= before && start.elapsed() < Duration::from_secs(60) {
        model.poll();
        std::thread::sleep(Duration::from_millis(5));
    }
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
    let segment =
        format!("\n[[cycle.segment]]\ntarget = [{}]\ndwell_s = 0.0\npayload = {gripper}\n", middle.join(", "));
    Config::parse(&format!("{}{segment}", entry.default_config)).unwrap()
}

fn demanding(entry: &MachineEntry) -> Config {
    let mut cfg = with_via_point(entry);
    // The pretension where there is a belt. The weak drive keeps half as much again as holding the start pose takes
    // (0.3 N m on the belt machines; the delta's arms need 0.52 N m held up, with less they lie on their stops).
    if cfg.get("belt.pretension_n").is_ok() {
        cfg.set_text("belt.pretension_n = 20.0").unwrap();
    }
    let hold = {
        let machine = (entry.build)(&cfg).unwrap();
        let mut torque = vec![0.0; machine.motors().len()];
        machine.equilibrium(&cfg.f64s("start.pose").unwrap(), &mut Vec::new(), &mut torque);
        torque.iter().fold(0.0f64, |m, t| m.max(t.abs()))
    };
    // And at least half the smallest motor peak of the default cycle, so that the drive still moves a machine whose
    // friction takes more than 0.3 N m (the screw's nut and guides) or whose motor values are in N (the linear motor).
    let load = sim_components::analysis::analyse_cycle(&mut sim_components::Sim::new(entry, &entry.config()).unwrap())
        .unwrap();
    let peak = load.motors.iter().map(|m| m.peak_torque).fold(f64::INFINITY, f64::min);
    cfg.set_text(&format!("drive.peak_torque_nm = {:?}", 0.3f64.max(1.5 * hold).max(0.5 * peak))).unwrap();
    // 100 V at the winding; or the DC link that gives three quarters of the back EMF at the cycle's top motor speed, where
    // 100 V would hold the motor far below it all the time (the screw's 251 rad/s need 220 V), so that no switch shows.
    let top = load.motors.iter().map(|m| m.peak_speed).fold(0.0, f64::max);
    let emf = cfg.f64("motor.torque_constant_nm_a").unwrap() * top / 1.5f64.sqrt();
    cfg.set_text(&format!("drive.bus_voltage_v = {:?}", 81.65f64.max(0.75 * emf))).unwrap();
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
        let alone = Runner::new(entry, &base).unwrap().sim.machine.motors().len() < 2;
        // Nor does backlash or a gearbox's loss act where the config has none (the screw's and the linear motor's couplings;
        // the screw's own loss is under the same switch), motor friction where the motor has none (the linear motor: no
        // bearings of its own), gravity along a horizontal slide (the screw, rack and linear motor axes) or on the table
        // about its vertical axis, or part heating and its feedback where no part warms but the motors (a coupling of
        // ratio 1; the linear delta's belt axes, whose guides' heat is not modelled), as sim-machines' control test has it.
        let machine = (entry.build)(&base).unwrap();
        let tight = base.f64("gearbox.backlash_rad").is_ok_and(|b| b == 0.0);
        let lossless = base.f64("gearbox.efficiency").is_ok_and(|e| e == 1.0)
            && base.get("screw").is_err()
            && base.get("rack").is_err();
        let level = base.f64("slide.incline_deg").is_ok_and(|i| i == 0.0) || base.get("table").is_ok();
        let frictionless = ["motor.friction_coulomb_nm", "motor.friction_viscous_nm_s_rad"]
            .iter()
            .all(|k| base.f64(k).is_ok_and(|f| f == 0.0));
        let cold = machine.thermal_parts().is_empty();
        let effects: Vec<String> = effects
            .into_iter()
            .filter(|e| {
                !skipped.contains(&e.as_str())
                    && !(alone && e == "effects.shared_dc_link")
                    && !(tight && e == "effects.gearbox_backlash")
                    && !(lossless && e == "effects.gearbox_efficiency")
                    && !(frictionless && e == "effects.motor_friction")
                    && !(level && e == "effects.gravity")
                    && !(cold && (e == "effects.part_heating" || e == "effects.thermal_feedback"))
            })
            .collect();
        // A switch that acts only in a setting the base has not is switched from that setting, in both runs: the belt's
        // wrap sets how many teeth hold against a tooth jump, and the drive's current limit acts where it gives less
        // torque than the motor (the gantries' drives give 10.9 A, 4.9 N m, far above the weak drive's). The drive's
        // overload acts where the current stays above its rating for the image's time (10 s against a cycle of a few,
        // with the weak drive far below its rating), so its drive has the weak drive's peak current I_p as its maximum,
        // a quarter of it rated, I_p for 10 ms from the rated load in an image of 50 ms: it trips at
        // 16 - 15 exp(-0.2) = 3.719, which I_p (16) reaches from cold in -0.05 ln(1 - 3.719 / 16) = 13 ms at the peak,
        // as in sim-machines' control test.
        let current = |share: f64| {
            share * base.f64("drive.peak_torque_nm").unwrap() / base.f64("motor.torque_constant_nm_a").unwrap()
        };
        let capped = [format!("ratings.drive_max_current_a = {:?}", current(0.8))];
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
            ("effects.drive_current_limit", &capped),
            ("effects.drive_overload", &overload),
        ];
        for effect in effects {
            let mut base = base.clone();
            for setting in through.iter().filter(|(e, _)| *e == effect).flat_map(|(_, s)| s.iter()) {
                base.set_text(setting).unwrap();
            }
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
            assert!(
                changed_after.is_some(),
                "{}: switching {effect} changed nothing within one cycle of {limit:.2} s",
                entry.id
            );
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
        // The friction of a guide (the delta has none: its joints' friction acts at the arms).
        let Some(friction) = runner.sim.signal_infos().iter().position(|s| s.id.ends_with(".friction")) else {
            continue;
        };
        assert!(
            samples(&events).last().unwrap()[friction].abs() > 1.0,
            "{}: not sliding at the moment of the switch",
            entry.id
        );

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

/// Pause and slow motion: the thread paces the simulation, the core never sees the wall clock. Only what holds on a
/// busy machine is asserted: the simulation never runs ahead of the wall clock (at speed 1 at most the wall time, at
/// 0.1 a tenth of it, each plus 50 ms for the pacing step and a sample in flight), it sends nothing while paused, and it
/// goes on after the pause; how much it does in a given wall time is left out (a loaded machine gave 0 samples in 0.4 s).
#[test]
fn the_session_runs_in_real_time_and_can_pause_and_slow_down() {
    let session = Session::start(MACHINES[0], &MACHINES[0].config()).unwrap();
    let times = || -> Vec<f64> {
        let times: Vec<f64> = samples(&session.drain()).iter().map(|s| s[0]).collect();
        assert!(times.windows(2).all(|w| w[1] > w[0]), "time went backwards");
        times
    };
    // Waits up to 10 s for a sample later than `after`; its time.
    let first_after = |after: f64| -> f64 {
        let start = Instant::now();
        loop {
            if let Some(&t) = times().last().filter(|t| **t > after) {
                return t;
            }
            assert!(start.elapsed() < Duration::from_secs(10), "no sample after {after} s within 10 s");
            std::thread::sleep(Duration::from_millis(10));
        }
    };
    let mut last = first_after(f64::NEG_INFINITY);
    let mut mark = Instant::now();
    // Over a wall interval the simulation goes at most `speed` times as far.
    let ahead_check = |speed: f64, wall: Duration, last: &mut f64, mark: &mut Instant| {
        std::thread::sleep(wall);
        let got = times();
        let elapsed = mark.elapsed().as_secs_f64();
        *mark = Instant::now();
        if let Some(&now) = got.last() {
            assert!(
                now - *last <= speed * elapsed + 0.05,
                "{} s simulated in {elapsed} s of wall time at speed {speed}",
                now - *last
            );
            *last = now;
        }
    };
    ahead_check(1.0, Duration::from_millis(400), &mut last, &mut mark);

    session.send(Request::Speed(0.1));
    std::thread::sleep(Duration::from_millis(100));
    if let Some(&t) = times().last() {
        last = t;
    }
    mark = Instant::now();
    ahead_check(0.1, Duration::from_millis(400), &mut last, &mut mark);

    session.send(Request::Pause(true));
    std::thread::sleep(Duration::from_millis(100));
    let at_pause = times().last().copied().unwrap_or(last);
    std::thread::sleep(Duration::from_millis(200));
    assert!(times().is_empty(), "samples arrived while paused");
    session.send(Request::Pause(false));
    session.send(Request::Speed(1.0));
    assert!(first_after(at_pause) > at_pause);

    session.send(Request::Command(Command::MoveTo(vec![])));
    let start = Instant::now();
    while !session.drain().iter().any(|e| matches!(e, Event::Refused("error-target-out-of-range", _))) {
        assert!(start.elapsed() < Duration::from_secs(10), "no refusal within 10 s");
        std::thread::sleep(Duration::from_millis(10));
    }
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
    // On a busy machine 0.2 s of wall time may bring no sample: wait for one.
    let start = Instant::now();
    while model.history.back().unwrap()[0] <= frozen_end && start.elapsed() < Duration::from_secs(60) {
        std::thread::sleep(Duration::from_millis(5));
        model.poll();
    }
    assert!(
        model.visible_rows().last().unwrap()[0] <= frozen_end + 0.2
            && model.history.back().unwrap()[0] > model.visible_rows().last().unwrap()[0]
    );

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
/// with the parameters as they are, and changing when a gain is changed; with them the modes over the axis's stroke.
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
        // The modes over X at the start height (z = 0.35 m), as the machine reports them there: both at every place,
        // the middle place at x = 0.5 m.
        let machine = (model.entry.build)(&model.config).unwrap();
        let (at, modes) = &tuning.modes[sim_app::model::MODE_POINTS / 2];
        assert!(
            tuning.modes.len() == sim_app::model::MODE_POINTS && tuning.modes.iter().all(|(_, m)| m.len() == 2),
            "{:?}",
            tuning.modes
        );
        assert!((at - 0.5).abs() < 1e-12 && *modes == machine.modes(&[0.5, 0.35]), "{at}: {modes:?}");
        (tuning.step.rise_time, tuning.bandwidth.unwrap())
    };
    let (rise, band) = measure(&mut model);
    // Kv = 50 1/s: a bandwidth near 8 Hz.
    assert!(band > 4.0 && band < 16.0 && rise > 0.0, "bandwidth {band} Hz, rise time {rise} s");
    model.set_param("drive.kv_1_s", toml::Value::Float(100.0));
    let (faster_rise, wider_band) = measure(&mut model);
    assert!(
        faster_rise < 0.8 * rise && wider_band > 1.3 * band,
        "Kv doubled: {rise} -> {faster_rise} s, {band} -> {wider_band} Hz"
    );
}

/// Polls until the analysis jobs have answered, whatever they say. The bound only stops a job that never ends: an
/// optimiser run takes minutes on a machine busy with the rest of the suite, and 300 s were too few for it once.
#[track_caller]
fn wait_for_jobs(model: &mut Model) {
    let start = Instant::now();
    while model.analysis_running() && start.elapsed() < Duration::from_secs(1200) {
        model.poll();
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(!model.analysis_running(), "the analysis did not finish");
}

/// Polls until the analysis jobs have answered, with a result and no complaint.
#[track_caller]
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
    model.history.back().unwrap()
        [model.session.signals.iter().position(|s| s.id == id).unwrap_or_else(|| panic!("no signal {id}"))]
}

fn temp_file(entry: &MachineEntry, name: &str) -> String {
    std::path::Path::new(env!("CARGO_TARGET_TMPDIR"))
        .join(format!("{}-app-{name}", entry.id))
        .to_str()
        .unwrap()
        .to_string()
}

/// `check` on every machine, each on a thread of its own named after the machine (a failure says "thread 'scara'
/// panicked"). One machine after the other, a test took up to five minutes of the full run, mostly waiting for a
/// processor, some of them in one of the four places of the group `real-time` (.config/nextest.toml).
fn on_every_machine(check: impl Fn(&'static MachineEntry) + Sync) {
    let check = &check;
    std::thread::scope(|scope| {
        for entry in MACHINES.iter().copied() {
            std::thread::Builder::new().name(entry.id.into()).spawn_scoped(scope, move || check(entry)).unwrap();
        }
    });
}

/// The app shows what the command line computes: the analysed cycle with its headline numbers, the load on
/// each motor and one row per check, for every machine, in the background while the machine runs on, and
/// in both languages. The numbers are those of the analysis functions.
#[test]
fn the_cycle_is_analysed_and_judged_in_the_app() {
    on_every_machine(|entry| {
        let mut model = Model::new(entry, "en");
        assert!(model.cycle_tables().is_empty());
        model.run_analysis(Analysis::Cycle);
        assert!(model.analysis_running());
        let before = model.history.len();
        wait_for_analysis(&mut model);
        // The simulation goes on beside the analysis (on a loaded machine its next sample can come after it).
        let start = Instant::now();
        while model.history.len() <= before && start.elapsed() < Duration::from_secs(10) {
            model.poll();
            std::thread::sleep(Duration::from_millis(10));
        }
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
            (
                "Guide life at the cycle's mean speed [h]",
                sim_components::analysis::guide_life_h(&entry.config(), &load),
            ),
        ];
        // The belt machines have every belt figure, and the guide figures where their config rates the parts (guides,
        // pulley bearings, belt teeth: the gantries' do, the linear delta's EGC-80 axes are not in the catalogue, and it
        // tells its guides' load per carriage only); a machine without belts, pulleys and guides (the delta) has the first
        // three.
        let belted = !sim.machine.belt_sections().is_empty();
        let rated = entry.config().get("ratings.guide_dynamic_load_rating_n").is_ok();
        if belted {
            assert!(
                expected[..6].iter().all(|(_, value)| value.is_some()) && load.radial_load_peak.unwrap() > 100.0,
                "{}: {expected:?}",
                entry.id
            );
            assert!(
                !rated || (expected[6..].iter().all(|(_, value)| value.is_some()) && life.unwrap() > 1.0),
                "{}: {expected:?}",
                entry.id
            );
        }
        let expected: Vec<(&str, Option<f64>)> = expected.into_iter().filter(|(_, value)| value.is_some()).collect();
        assert!(expected.len() >= 3);
        assert_eq!(summary.len(), 1 + expected.len(), "{}: {summary:?}", entry.id);
        for (label, value) in expected {
            assert!(shows(&row(summary, label)[0], value.unwrap()), "{}: {label} {value:?} in {summary:?}", entry.id);
        }
        // One row per figure and one column per motor: peak, RMS and cubic-mean torque, winding rise, energy
        // drawn and fed back, the braking resistor's energy and mean power, the highest DC link voltage, the energy
        // fed back into the mains (zero without the regenerative supply), and since the motor and drive model of
        // 14 November the housing's rise and the drive's overload image (zero with their switches off, as shipped):
        // 1 + 12 rows.
        assert_eq!(motors[0][0], "Figure");
        assert_eq!(
            motors[0][1..],
            model.session.motors.iter().map(|info| model.tr.tr(info.key).to_string()).collect::<Vec<_>>()
        );
        // And the lives in hours and the belt's tooth load, which both gantries' default configs have (not the
        // gearing's, which their cycles never take above its rated torque, nor the motor bearings', as their belts pull on a
        // gearbox); the delta has the gearbox's lives alone, a machine without a rated gearbox (the screw's coupling, the
        // linear delta's direct drives) none.
        let lives = {
            let (cfg, m) = (entry.config(), &load.motors[0]);
            let figures = [
                sim_components::analysis::pulley_bearing_life_h(&cfg, m),
                sim_components::analysis::gearbox_life_h(&cfg, m),
                sim_components::analysis::gearbox_gearing_life_h(&cfg, m),
                sim_components::analysis::motor_bearing_life_h(&cfg, m),
                sim_components::analysis::belt_tooth_utilisation(&cfg, m).unwrap(),
            ];
            figures.iter().filter(|l| l.is_some()).count()
        };
        assert_eq!(motors.len(), if belted && rated { 16 } else { 13 + lives }, "{}", entry.id);
        assert!(row(motors, "Fed back into the mains [J]").iter().all(|cell| shows(cell, 0.0)), "{motors:?}");
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
                assert!(
                    value > 0.0 && shows(&row(motors, label)[k], value),
                    "{}: {label} {value} in {motors:?}",
                    entry.id
                );
            }
            assert!(
                m.cubic_mean_torque > m.rms_torque
                    && m.peak_dc_link_voltage > entry.config().f64("drive.bus_voltage_v").unwrap()
            );
            assert!(
                shows(&row(motors, "Braking resistor, energy [J]")[k], m.resistor_energy)
                    && shows(&row(motors, "Braking resistor, mean power [W]")[k], m.resistor_energy / load.duration)
            );
            let cfg = entry.config();
            let lives = [
                (
                    "Pulley shaft bearing, modified rating life [h]",
                    sim_components::analysis::pulley_bearing_life_h(&cfg, m),
                ),
                ("Gearbox planet bearings, life [h]", sim_components::analysis::gearbox_life_h(&cfg, m)),
                (
                    "Gearbox gearing, fatigue life above the rated torque [h]",
                    sim_components::analysis::gearbox_gearing_life_h(&cfg, m),
                ),
                (
                    "Belt tooth load, share of its rating",
                    sim_components::analysis::belt_tooth_utilisation(&cfg, m).unwrap(),
                ),
            ];
            let gearing = lives[2].0;
            for (label, value) in
                lives.into_iter().filter(|(label, value)| (belted && rated && *label != gearing) || value.is_some())
            {
                assert!(shows(&row(motors, label)[k], value.unwrap()), "{}: {label} {value:?} in {motors:?}", entry.id);
            }
        }
        // One row per check: label, motor, value, limit, result, as the library judges it (the delta's catalogue
        // gearboxes fail `settling`, a sizing finding; the belt machines pass every check).
        assert_eq!(checks.len(), 1 + verdicts.len());
        assert_eq!(checks[0], ["Check", "Motor", "Value", "Limit", "Result"]);
        for (row, v) in checks[1..].iter().zip(&verdicts) {
            assert_eq!(row[0], model.tr.tr(&format!("check-{}", v.check)));
            assert!(model.tr.has(&format!("check-{}", v.check)), "no label for {}", v.check);
            assert_eq!(row[1].is_empty(), v.motor.is_none());
            // To the digits the tables give (an arm's wrist motor uses 0.011 of its curve, shown with three decimals).
            assert!(shows(&row[2], v.value) && shows(&row[3], v.limit), "{row:?} against {v:?}");
            assert_eq!(row[4], if v.ok { "ok" } else { "FAIL" });
        }
        assert!(
            checks.iter().any(|row| row[0] == "Winding rise, settled [K]")
                && checks.iter().any(|row| row[0] == "Off target while dwelling [m]")
        );

        // A motor too weak for the cycle fails its check, in the words of the chosen language.
        model.set_language("de");
        model.set_param("motor.rated_current_a", toml::Value::Float(0.02));
        model.run_analysis(Analysis::Cycle);
        wait_for_analysis(&mut model);
        let checks = &model.cycle_tables()[2];
        assert_eq!(checks[0][0], "Prüfung");
        assert!(
            checks.iter().any(|row| row[0].starts_with("Übertemperatur der Wicklung") && row[4] == "NICHT BESTANDEN"),
            "{checks:?}"
        );
        model.clear_analysis();
        assert!(model.cycle_tables().is_empty());
    });
}

/// With the regenerative supply switched on in the app, its tables show what the analysis computes: per motor the energy
/// fed back into the mains, the two line module checks, and the rows "fed back into the mains" and "line module loss"
/// of the energy budget; in German too.
#[test]
fn the_regenerative_supply_shows_in_the_apps_tables() {
    for entry in MACHINES.iter().copied() {
        let mut model = Model::new(entry, "en");
        model.set_param("effects.regenerative_supply", toml::Value::Boolean(true));
        model.run_analysis(Analysis::Cycle);
        wait_for_analysis(&mut model);
        let cfg = model.config.clone();
        let load =
            sim_components::analysis::analyse_cycle(&mut sim_components::Sim::new(entry, &cfg).unwrap()).unwrap();
        let tables = model.cycle_tables();
        let (motors, checks) = (&tables[1], &tables[2]);
        for (k, m) in load.motors.iter().enumerate() {
            assert!(
                m.mains_energy > 0.0 && shows(&row(motors, "Fed back into the mains [J]")[k], m.mains_energy),
                "{}: {motors:?}",
                entry.id
            );
        }
        assert!(
            ["Line module, mean power either way [W]", "Line module, highest power either way [W]"].iter().all(
                |label| checks.iter().filter(|row| row[0] == *label && row[4] == "ok").count() == load.motors.len()
            ),
            "{}: {checks:?}",
            entry.id
        );
        let budget = load.budget.unwrap();
        let shown = model.budget_table().unwrap();
        for (id, label) in [
            ("mains", "Drive · Fed back into the mains"),
            ("line_module", "Drive · Line module loss while feeding back"),
        ] {
            let energy = budget.rows.iter().find(|r| r.id == id).unwrap().energy;
            assert!(
                energy > 0.0 && shows(&row(&shown, label)[0], energy),
                "{}: {label} {energy} in {shown:?}",
                entry.id
            );
        }
        model.set_language("de");
        assert!(row(&model.cycle_tables()[1], "Ins Netz zurückgespeist [J]").len() == load.motors.len());
        assert!(model.budget_table().unwrap().iter().any(|cells| cells[0].ends_with("In das Netz zurückgespeist")));
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
        assert!(
            rows.windows(2).all(|w| w[0].strength(nominal) >= w[1].strength(nominal))
                && rows[0].strength(nominal) > 0.01,
            "{}",
            entry.id
        );
        let table = model.sensitivity_table().unwrap();
        assert_eq!((table.len(), table[0].len()), (9, 6), "{}: {table:?}", entry.id);
        assert_eq!(table[0][0], "Parameter");
        for (shown, row) in table[1..].iter().zip(rows) {
            // "Belt · Pretension [N]": group and parameter as the parameter panel names them.
            let (group, leaf) = row.path.rsplit_once('.').unwrap();
            assert_eq!(
                shown[0],
                format!("{} · {}", model.tr.tr(&format!("group-{group}")), model.tr.tr(&format!("param-{leaf}")))
            );
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
    on_every_machine(optimised_in_the_app);
}

/// `the_app_optimises_the_cycle_and_applies_the_settings` on one machine. From cold, in the app and here: the cycle
/// times and how the settings are shown and applied are judged, nothing thermal, and the warm start runs each of the
/// optimiser's cycles once more.
fn optimised_in_the_app(entry: &'static MachineEntry) {
    let mut model = Model::new(entry, "en");
    model.set_param("effects.thermal_warm_start", toml::Value::Boolean(false));
    model.run_analysis(Analysis::Optimise);
    // The optimiser starts only from a cycle that passes every verdict, which the delta's does not (`settling`).
    let mut cfg = entry.config();
    cfg.set_text("effects.thermal_warm_start = false").unwrap();
    let load = sim_components::analysis::analyse_cycle(&mut sim_components::Sim::new(entry, &cfg).unwrap()).unwrap();
    let judged = sim_components::analysis::verdicts(&cfg, &load).unwrap();
    if !judged.iter().all(|v| v.ok) {
        wait_for_jobs(&mut model);
        assert!(model.messages.iter().any(|m| m.0 == "error-config-invalid"), "{}: {:?}", entry.id, model.messages);
        return;
    }
    // At least 5 % shorter; 2 % where the cycle as configured leaves less than a tenth of the settling tolerance, which
    // a faster move uses up (the screw's 46 of 50 um: 2.9 % shorter by acceleration and jerk alone).
    let settling = judged.iter().find(|v| v.check == "settling").unwrap();
    let shorter = if settling.value > 0.9 * settling.limit { 0.98 } else { 0.95 };
    wait_for_analysis(&mut model);
    let table = model.optimised_table().unwrap();
    let found = model.optimised.clone().unwrap();
    assert_eq!(table[0], ["Setting", "Before", "After"]);
    assert_eq!(table[1][0], "Cycle time [s]");
    let (before, after) = (number(&table[1][1]), number(&table[1][2]));
    assert!(
        after < shorter * before && (before - cycle_time(entry, &entry.config())).abs() < 2e-3,
        "{}: {before} s to {after} s",
        entry.id
    );
    let cycle = found.changes.iter().filter(|c| c.0 == "cycle.segment").count();
    let lift_rows = found.lifts.len() + found.lifts.iter().filter(|l| l.via.is_some()).count();
    assert!(!found.changes.is_empty() && table.len() == 3 + found.changes.len() - cycle + lift_rows);
    assert!(
        table[3..].iter().all(|row| row[1] != row[2] && !row[0].contains("param-") && !row[0].contains("effect-")),
        "{table:?}"
    );

    // An edit made since the optimiser ran: its settings, absolute values, used to be written over it
    // without a word. Now nothing is applied, with a message, until the edit is taken back.
    let key = payload_key(&model.config);
    let payload = model.config.f64(key).unwrap();
    model.set_param(key, toml::Value::Float(payload + 0.5));
    model.apply_optimised();
    assert_eq!(model.messages.last().map(|m| m.0), Some("ui-analysis-stale"), "{}", entry.id);
    assert!(model.optimised_table().is_some());
    for (path, before, _) in &found.changes {
        assert_eq!(&model.config.get(path).unwrap().to_string(), before, "{path} was applied over a changed config");
    }
    model.set_param(key, toml::Value::Float(payload));

    model.apply_optimised();
    assert!(model.optimised_table().is_none());
    for (path, _, value) in &found.changes {
        assert_eq!(&model.config.get(path).unwrap().to_string(), value, "{path}");
    }
    assert!((cycle_time(entry, &model.config) - after).abs() < 2e-3, "{}", entry.id);
    // The running simulation took the config: no refusal comes back.
    std::thread::sleep(Duration::from_millis(100));
    model.poll();
    assert!(
        model.messages.iter().all(|m| m.0.starts_with("limit-") || m.0 == "ui-analysis-stale"),
        "{:?}",
        model.messages
    );
    for (path, _, value) in &found.changes {
        assert_eq!(&model.config.get(path).unwrap().to_string(), value, "{path}");
    }
}

/// The optimiser's lower lifts in the app: with `profile.min_clearance_m` = 0.1 m the two lifts of the machine with two axes go from
/// 0.35 m to 0.15 m (see `the_optimiser_lowers_the_lifts_to_the_clearance_and_saves_the_hand_calculated_time`); the table
/// has a row per lift, in both languages, and "Apply" writes the new cycle into the config.
#[test]
fn the_app_shows_and_applies_lower_lifts() {
    let entry = MACHINES.iter().copied().find(|e| e.config().f64s("start.pose").unwrap().len() == 2).unwrap();
    let mut model = Model::new(entry, "en");
    model.set_param("profile.min_clearance_m", toml::Value::Float(0.1));
    // From cold: the lifts are judged, nothing thermal, and the warm start runs each of the optimiser's cycles once more.
    model.set_param("effects.thermal_warm_start", toml::Value::Boolean(false));
    model.run_analysis(Analysis::Optimise);
    wait_for_analysis(&mut model);
    let table = model.optimised_table().unwrap();
    for segments in ["2, 3, 4", "5, 6, 1"] {
        let row = table
            .iter()
            .find(|row| row[0] == format!("Lift height, segments {segments} [m]"))
            .unwrap_or_else(|| panic!("{table:?}"));
        assert!(number(&row[1]) == 0.35 && (number(&row[2]) - 0.15).abs() < 1e-9, "{row:?}");
    }
    model.set_language("de");
    assert!(model.optimised_table().unwrap().iter().any(|row| row[0] == "Hubhöhe, Abschnitte 2, 3, 4 [m]"));
    model.apply_optimised();
    let lifted = model
        .config
        .array("cycle.segment")
        .unwrap()
        .iter()
        .filter(|s| (s["target"][1].as_float().unwrap() - 0.15).abs() < 1e-12)
        .count();
    assert_eq!(lifted, 4);
}

/// Disable and Enable from the app: disabled, no motor gives torque and a move is refused with the reason;
/// enabled again, the drives hold and the move runs.
#[test]
fn the_drives_can_be_disabled_and_enabled_from_the_app() {
    for entry in MACHINES.iter().copied() {
        let mut model = Model::new(entry, "en");
        let torques: Vec<usize> = model
            .session
            .signals
            .iter()
            .enumerate()
            .filter(|(_, s)| s.id.ends_with(".torque"))
            .map(|(i, _)| i)
            .collect();
        assert_eq!(torques.len(), model.session.motors.len());
        let run = |model: &mut Model, seconds: f64| {
            let until = model.history.back().map_or(0.0, |row| row[0]) + seconds;
            let start = Instant::now();
            while model.history.back().is_none_or(|row| row[0] < until) && start.elapsed() < Duration::from_secs(10) {
                model.poll();
                std::thread::sleep(Duration::from_millis(5));
            }
        };
        let pushing = |model: &Model, last: usize| {
            model.history.iter().rev().take(last).any(|row| torques.iter().any(|&i| row[i] != 0.0))
        };
        model.command(Command::StartCycle);
        run(&mut model, 0.5);
        assert!(pushing(&model, 100), "{}: no torque in the cycle", entry.id);

        model.command(Command::Disable);
        run(&mut model, 0.3);
        assert!(!pushing(&model, 100), "{}: torque while disabled", entry.id);
        model.command(Command::MoveTo(model.target.clone()));
        run(&mut model, 0.1);
        assert_eq!(
            model.messages.iter().filter(|m| m.0 == "error-disabled").count(),
            1,
            "{}: {:?}",
            entry.id,
            model.messages
        );

        // Enabled, the move back to the start pose is accepted and takes torque.
        model.command(Command::Enable);
        model.command(Command::MoveTo(model.target.clone()));
        run(&mut model, 0.3);
        assert!(pushing(&model, 100), "{}: no torque after enabling", entry.id);
        assert_eq!(
            model.messages.iter().filter(|m| m.0.starts_with("error-")).count(),
            1,
            "{}: {:?}",
            entry.id,
            model.messages
        );
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
    // The simulation answers from its own thread: on a loaded machine later than any fixed wait.
    let until = |model: &mut Model, done: &dyn Fn(&Model) -> bool| {
        let start = Instant::now();
        while !done(model) {
            assert!(start.elapsed() < Duration::from_secs(10), "no answer within 10 s: {:?}", model.messages);
            model.poll();
            std::thread::sleep(Duration::from_millis(5));
        }
    };
    model.set_param("drive.velocity_cycle_s", toml::Value::Float(1.0e-4));
    until(&mut model, &|model| errors(model) == 1);
    wait(&mut model);
    assert_eq!(model.config.f64("drive.velocity_cycle_s").unwrap(), good, "the refused value stayed in the config");
    assert_eq!(errors(&model), 1, "{:?}", model.messages);
    // The next edit is taken.
    model.set_param("mass.payload_kg", toml::Value::Float(3.5));
    wait(&mut model);
    assert_eq!((model.config.f64("mass.payload_kg").unwrap(), errors(&model)), (3.5, 1), "{:?}", model.messages);
    // The same refusal again goes back to the config with that edit, and does not pile up.
    model.set_param("drive.velocity_cycle_s", toml::Value::Float(1.0e-4));
    until(&mut model, &|model| model.config.f64("drive.velocity_cycle_s").unwrap() == good);
    wait(&mut model);
    assert_eq!(
        (model.config.f64("mass.payload_kg").unwrap(), model.config.f64("drive.velocity_cycle_s").unwrap()),
        (3.5, good)
    );
    assert_eq!(errors(&model), 1, "{:?}", model.messages);
    // Two edits on the way at once, the first of them refused: the second goes with it, as it was sent with
    // the refused value in it, and the config is again the one the simulation runs.
    model.set_param("drive.velocity_cycle_s", toml::Value::Float(1.0e-4));
    model.set_param("mass.payload_kg", toml::Value::Float(4.5));
    wait(&mut model);
    assert_eq!(
        (model.config.f64("mass.payload_kg").unwrap(), model.config.f64("drive.velocity_cycle_s").unwrap()),
        (3.5, good)
    );
    // And a second edit that puts the refused value right before the refusal is back: it is taken, and the
    // config shows it, not the config the refusal went back to.
    model.set_param("drive.velocity_cycle_s", toml::Value::Float(1.0e-4));
    model.set_param("drive.velocity_cycle_s", toml::Value::Float(2.0 * good));
    wait(&mut model);
    assert_eq!(
        (model.config.f64("mass.payload_kg").unwrap(), model.config.f64("drive.velocity_cycle_s").unwrap()),
        (3.5, 2.0 * good)
    );
    // A refused command twice over is one message as well.
    model.command(Command::MoveTo(Vec::new()));
    model.command(Command::MoveTo(Vec::new()));
    wait(&mut model);
    assert_eq!(model.messages.iter().filter(|m| m.0 == "error-target-out-of-range").count(), 1, "{:?}", model.messages);
}

/// The worst corner of the bounds, as `sim-cli verdicts --bands --worst` gives it: with both options on, the cycle
/// analysis also searches it (`verdict_worst`) and shows per check the worst value found, its limit, the result
/// and which bounded values are at their high and which at their low bound; in both languages. Without the option,
/// or without the bands, there is no such table. On the machine with the fewest bounded values (some 300 cycles).
/// The bands beside it come from the search's first runs (`verdict_worst_and_bands`, whose bands are those of
/// `verdict_bands`: `the_worst_corner_search_finds_the_hand_calculated_corner` checks that).
#[test]
fn the_app_searches_the_worst_corner_of_the_bounds() {
    use sim_components::analysis::{bounds, verdict_worst_and_bands};
    // The machine with the fewest bounded values, so that the search stays short.
    let entry = MACHINES.iter().copied().min_by_key(|e| bounds(&e.config()).unwrap().len()).unwrap();
    let mut model = Model::new(entry, "en");
    // From cold, in the app and below: how the search is shown is judged here, nothing thermal, and the warm start runs
    // each of its cycles once more.
    model.set_param("effects.thermal_warm_start", toml::Value::Boolean(false));
    model.inputs.worst = true;
    model.run_analysis(Analysis::Cycle);
    wait_for_analysis(&mut model);
    assert!(model.worst_table().is_none(), "searched without the bands");
    model.inputs.bands = true;
    model.run_analysis(Analysis::Cycle);
    wait_for_analysis(&mut model);
    let mut cfg = entry.config();
    cfg.set_text("effects.thermal_warm_start = false").unwrap();
    let bounded = bounds(&cfg).unwrap();
    let (worst, bands, _) = verdict_worst_and_bands(entry, &cfg, &bounded).unwrap();
    let table = model.worst_table().unwrap();
    assert_eq!(table[0], ["Check", "Motor", "Worst value", "Limit", "Result", "At the high bound", "At the low bound"]);
    assert_eq!(table.len(), 1 + worst.len());
    for (row, w) in table[1..].iter().zip(&worst) {
        assert_eq!(row[0], model.tr.tr(&format!("check-{}", w.verdict.check)));
        assert!(
            shows(&row[2], w.verdict.value)
                && shows(&row[3], w.verdict.limit)
                && row[4] == if w.verdict.ok { "ok" } else { "FAIL" },
            "{row:?} against {w:?}"
        );
        // Every bounded value at a bound is named once, at that bound; one as configured is not named.
        for (bound, high) in bounded.iter().zip(&w.high) {
            let named = |cell: &str| cell.split(", ").any(|path| path == bound.0);
            assert!(
                named(&row[5]) == (*high == Some(true)) && named(&row[6]) == (*high == Some(false)),
                "{}: {row:?}",
                bound.0
            );
        }
    }
    let tables = model.cycle_tables();
    assert_eq!(tables[3].len(), 1 + bands.len());
    for (row, band) in tables[3][1..].iter().zip(&bands) {
        // The share as the app shows it, cut at four decimals (`Band::share_shown`): 1 of 45 is 2.22 %.
        assert!(
            shows(&row[2], band.low) && shows(&row[3], band.high) && shows(&row[5], 100.0 * band.share_shown(4)),
            "{row:?} against {band:?}"
        );
    }
    model.set_language("de");
    assert_eq!(
        model.worst_table().unwrap()[0][2..],
        ["Ungünstigster Wert", "Grenze", "Ergebnis", "An der oberen Grenze", "An der unteren Grenze"]
    );
    model.inputs.worst = false;
    model.run_analysis(Analysis::Cycle);
    wait_for_analysis(&mut model);
    assert!(model.worst_table().is_none());
}

/// Verdicts with error bands, as `sim-cli verdicts --bands` gives them: with the option on, the cycle analysis
/// also runs the cycle with every bounded value at its bounds, and every check gets the lowest and highest
/// value it took and the worst-case result. A result is marked when the config has changed since.
#[test]
fn the_checks_get_error_bands_in_the_app() {
    use sim_components::analysis::bounds;
    // The app runs the same code for every machine: that its bands are those of the library is checked on the machine
    // with the fewest bounded values, and on every machine that the tables show the bands the app found.
    let fewest = MACHINES.iter().copied().min_by_key(|e| bounds(&e.config()).unwrap().len()).unwrap();
    on_every_machine(|entry| banded_in_the_app(entry, entry.id == fewest.id));
}

/// `the_checks_get_error_bands_in_the_app` on one machine, with `library` against `verdict_bands` and
/// `verdict_bands_sampled`. From cold, in the app and here: how the bands are shown is judged, nothing thermal, and
/// the warm start runs each of their cycles once more.
fn banded_in_the_app(entry: &'static MachineEntry, library: bool) {
    use sim_components::analysis::{bounds, verdict_bands, verdict_bands_sampled};
    let mut model = Model::new(entry, "en");
    model.set_param("effects.thermal_warm_start", toml::Value::Boolean(false));
    model.inputs.bands = true;
    model.run_analysis(Analysis::Cycle);
    assert!(
        model.running(Analysis::Cycle) && !model.running(Analysis::Sensitivity) && !model.has_result(Analysis::Cycle)
    );
    wait_for_analysis(&mut model);
    assert!(model.has_result(Analysis::Cycle));
    let mut cfg = entry.config();
    cfg.set_text("effects.thermal_warm_start = false").unwrap();
    let bands = if library {
        verdict_bands(entry, &cfg, &bounds(&cfg).unwrap()).unwrap()
    } else {
        model.bands.clone().unwrap()
    };
    // A fourth table beside the three of the cycle: per check and motor the band and the worst-case result.
    let tables = model.cycle_tables();
    let [_, _, checks, banded] = &tables[..] else { panic!("{} tables", tables.len()) };
    assert_eq!(checks[0].len(), 5);
    assert_eq!(banded[0], ["Check", "Motor", "Lowest", "Highest", "Worst case", "Runs passed [%]"]);
    assert_eq!(banded.len(), 1 + bands.len(), "{}", entry.id);
    for (row, band) in banded[1..].iter().zip(&bands) {
        assert_eq!(row[0], model.tr.tr(&format!("check-{}", band.verdict.check)));
        assert_eq!(
            (row[1].is_empty(), row[4].as_str()),
            (band.verdict.motor.is_none(), if band.ok { "ok" } else { "FAIL" })
        );
        // The share as the app shows it, cut at four decimals (`Band::share_shown`): 1 of 45 is 2.22 %.
        assert!(
            shows(&row[2], band.low) && shows(&row[3], band.high) && shows(&row[5], 100.0 * band.share_shown(4)),
            "{row:?} against {band:?}"
        );
        assert!(band.low <= band.verdict.value && band.verdict.value <= band.high);
    }
    // The bounds move the checks: a band is not a line.
    assert!(bands.iter().any(|band| band.high > 1.05 * band.low), "{}: {bands:?}", entry.id);
    // The gearbox's peak torque, or the motor's where no gearbox is rated: more friction needs more torque, so its band
    // has a width, and it passes in the worst case (checked on the command line's bands until 14 November 2026).
    let peak = if cfg.get("ratings.gearbox_max_torque_nm").is_ok() { "gearbox-peak" } else { "motor-peak" };
    let band = bands.iter().find(|b| b.verdict.check == peak).unwrap();
    assert!(band.high > 1.02 * band.low && band.ok, "{}: {band:?}", entry.id);
    model.set_language("de");
    assert_eq!(
        model.cycle_tables()[3][0][2..],
        ["Kleinster Wert", "Größter Wert", "Ungünstigster Fall", "Bestandene Läufe [%]"]
    );
    model.set_language("en");
    assert_eq!(model.band_samples, 0);

    // The result is of the config it was computed for, and says so once that has changed.
    assert!(!model.outdated(Analysis::Cycle));
    let key = payload_key(&model.config);
    let payload = model.config.f64(key).unwrap();
    model.set_param(key, toml::Value::Float(payload + 0.5));
    assert!(model.outdated(Analysis::Cycle) && !model.outdated(Analysis::Search));
    model.set_param(key, toml::Value::Float(payload));
    assert!(!model.outdated(Analysis::Cycle));
    // With a number of samples the bands are those of `verdict_bands_sampled` with the seed 1.
    model.inputs.samples = 6;
    model.run_analysis(Analysis::Cycle);
    wait_for_analysis(&mut model);
    let sampled = if library {
        verdict_bands_sampled(entry, &cfg, &bounds(&cfg).unwrap(), 6, 1).unwrap()
    } else {
        model.bands.clone().unwrap()
    };
    let tables = model.cycle_tables();
    assert_eq!((model.band_samples, tables[3].len()), (6, 1 + sampled.len()), "{}", entry.id);
    for (row, band) in tables[3][1..].iter().zip(&sampled) {
        assert!(
            shows(&row[2], band.low) && shows(&row[3], band.high) && shows(&row[5], 100.0 * band.share_shown(4)),
            "{row:?} against {band:?}"
        );
    }
    assert!(sampled.iter().zip(&bands).any(|(sampled, alone)| sampled.high != alone.high), "{}", entry.id);
    // Without the option there are the three tables again.
    model.inputs.bands = false;
    model.run_analysis(Analysis::Cycle);
    wait_for_analysis(&mut model);
    assert_eq!(model.cycle_tables().len(), 3);
}

/// The motor search in the app, as `sim-cli search`: the motors whose article number starts as asked, each
/// on the cycle, ranked as the library ranks them; a button per row that names the motor in the running
/// config; and inputs that cannot be searched are refused with a message.
#[test]
fn the_app_searches_motors_and_names_one_in_the_config() {
    use sim_components::catalog::search_motors;
    let series = "1FK2104";
    let part =
        |model: &Model, key: &str| model.config.get(&format!("parts.{key}")).unwrap().as_str().unwrap().to_string();
    let settle = |model: &mut Model| {
        std::thread::sleep(Duration::from_millis(150));
        model.poll();
    };
    // What the running simulation says of a motor too weak for the machine: its limits, and on an arm that falls with it
    // the drive's trip (its DC link takes what the falling arm feeds back).
    let weak = |m: &(&str, String)| m.0.starts_with("limit-") || m.0 == "ui-drive-tripped";
    on_every_machine(|entry| {
        let mut model = Model::new(entry, "en");
        // From cold, in the app and in the library's search below: what is judged is how the motors are ranked, shown and
        // named, nothing thermal, and the warm start runs each candidate's cycle once more.
        model.set_param("effects.thermal_warm_start", toml::Value::Boolean(false));
        let mut cold = entry.config();
        cold.set_text("effects.thermal_warm_start = false").unwrap();
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
        let found = search_motors(entry, &cold, "400v", &|article| article.starts_with(series), None);
        let table = model.search_table().unwrap();
        assert_eq!(
            table[0],
            [
                "Motor",
                "Gearbox",
                "Axis",
                "Motor and gearbox [kg]",
                "Installed mass [kg]",
                "Result",
                "Tightest check",
                "Value / limit"
            ]
        );
        assert!(found.len() >= 3 && table.len() == 1 + found.len(), "{}: {table:?}", entry.id);
        for (row, c) in table[1..].iter().zip(&found) {
            assert_eq!(
                (row[0].as_str(), row[1].as_str(), row[2].as_str(), row[5].as_str()),
                (c.article, "", "", if c.passes() { "ok" } else { "FAIL" })
            );
            assert!(shows(&row[3], c.mass) && shows(&row[4], c.cost), "{row:?}");
            match c.tightest() {
                Some((check, share)) => {
                    assert!(row[6] == model.tr.tr(&format!("check-{check}")) && shows(&row[7], share), "{row:?}")
                }
                // Not run: the reason in words, and no figure.
                None => assert!(row[6].contains(' ') && row[7].is_empty(), "{row:?}"),
            }
        }
        // A 2000 r/min motor is too slow for the axis speed by its catalogue numbers alone: it is listed as
        // failed with that reason, and no cycle is simulated for it. Not on the linear motor, whose motor values are in N
        // and m: the catalogue's rotary motors are no motors for it (the search tries them all the same).
        model.inputs.series = "1FK2208-3AC".into();
        model.run_analysis(Analysis::Search);
        wait_for_analysis(&mut model);
        if model.config.get("linear_motor").is_err() {
            assert_eq!(
                model.search_table().unwrap()[1][5..],
                ["FAIL", "Motor speed at the highest axis speed", ""],
                "{}",
                entry.id
            );
        }
        model.inputs.series = series.into();
        model.run_analysis(Analysis::Search);
        wait_for_analysis(&mut model);

        // The button of the first row: its motor is named in the config, and the running simulation takes it.
        model.use_candidate(0);
        settle(&mut model);
        assert_eq!(
            (part(&model, "motor"), part(&model, "motor_supply"), part(&model, "gearbox")),
            (found[0].article.to_string(), "400v".to_string(), String::new())
        );
        assert!(model.messages.iter().all(weak), "{}: {:?}", entry.id, model.messages);
        // Naming a motor does not outdate the search, which put its own motors in: another row can follow.
        assert!(!model.outdated(Analysis::Search));
        model.use_candidate(1);
        settle(&mut model);
        assert_eq!(part(&model, "motor"), found[1].article);
        // A row that is not there does nothing.
        model.use_candidate(found.len());
        assert_eq!(part(&model, "motor"), found[1].article);
        // Any other change does: the motors passed or failed on another machine, so none of them is applied.
        let key = payload_key(&model.config);
        let payload = model.config.f64(key).unwrap();
        model.set_param(key, toml::Value::Float(payload + 0.5));
        assert!(model.outdated(Analysis::Search));
        model.use_candidate(0);
        settle(&mut model);
        // (Not counting what the running simulation says about the motor's limits meanwhile: the delta's 1FK2104 is too
        // weak for its axis limits.)
        let said = model.messages.iter().rev().find(|m| !weak(m)).map(|m| m.0);
        assert_eq!((said, part(&model, "motor")), (Some("ui-analysis-stale"), found[1].article.to_string()));
        model.clear(Analysis::Search);
        assert!(model.search_table().is_none() && !model.outdated(Analysis::Search));
    });

    // With a gearbox series every motor is tried with the gearboxes of the series that fit it, and the
    // button names both.
    let entry = MACHINES[0];
    let mut model = Model::new(entry, "de");
    (model.inputs.series, model.inputs.gearboxes) = ("1FK2104-5AK".into(), "NRK070".into());
    model.run_analysis(Analysis::Search);
    wait_for_analysis(&mut model);
    let found = search_motors(entry, &entry.config(), "400v", &|article| article == "1FK2104-5AK", Some("NRK070"));
    let table = model.search_table().unwrap();
    assert_eq!((table[0][1].as_str(), table[0][6].as_str()), ("Getriebe", "Knappste Prüfung"));
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
        std::fs::write(&lines, format!("# two lines\nline,{}\nline,{}\n", text(&point(0.5)), text(&point(0.75))))
            .unwrap();
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
        assert!(
            (slow - load.duration).abs() < 2e-3 && load.duration > 0.5,
            "{}: {} s in {summary:?}",
            entry.id,
            load.duration
        );
        let behind = number(&row(summary, "Behind the setpoint, peak [mm]")[0]);
        assert!(
            load.tracking_error_peak > 1e-5 && (behind / (load.tracking_error_peak * 1e3) - 1.0).abs() < 0.05,
            "{}: {} m in {summary:?}",
            entry.id,
            load.tracking_error_peak
        );
        assert_eq!(row(summary, "Planned path off the given one, at most [mm]")[0], "0.000");
        // A path has no cycle time and no dwell.
        assert!(summary.iter().all(|row| row[0] != "Cycle time [s]" && !row[0].starts_with("Off target")));
        for (k, m) in load.motors.iter().enumerate() {
            assert!(
                (number(&row(motors, "Peak torque [N m]")[k]) / m.peak_torque - 1.0).abs() < 0.02,
                "{}: {} N m in {motors:?}",
                entry.id,
                m.peak_torque
            );
        }
        assert_eq!(checks.len(), 1 + judged.len());
        for (row, v) in checks[1..].iter().zip(&judged) {
            assert_eq!(
                (row[0].as_str(), row[4].as_str()),
                (model.tr.tr(&format!("check-{}", v.check)), if v.ok { "ok" } else { "FAIL" })
            );
        }
        // The operating points of the path, as its analysis has them, over the curves the cycle's plot has.
        let run = model.path_run.as_ref().unwrap();
        let (points_of_path, curves) = model.points_plot(Analysis::Path).unwrap();
        assert_eq!(curves, sim_app::model::torque_speed_curves(sim.control.drive_params()));
        for (k, m) in run.load.motors.iter().enumerate() {
            assert!(
                m.points.len() > 100 && points_of_path[k] == m.points.iter().map(|(s, t)| [*s, *t]).collect::<Vec<_>>(),
                "{}: motor {k}",
                entry.id
            );
        }
        // The planned path for the plot: every pose, from where the machine stood to the end of the file.
        let (across, up, line) = model.path_plot().unwrap();
        // In the unit of the axis drawn upwards: m, or rad on the rotary table.
        assert!(
            !across.is_empty()
                && (up.ends_with("[m]") || up.ends_with("[rad]"))
                && line.len().abs_diff(points.len()) <= 1,
            "{across}, {up}, {} points",
            line.len()
        );
        let (first, last) = (line[0], line[line.len() - 1]);
        // The plan starts at the setpoint at rest, up to the machine's slack from the start pose (the delta's sag).
        let slack = (entry.build)(&entry.config()).unwrap().setpoint_slack();
        if axes.len() == 1 {
            assert!(
                first[0] == 0.0 && (first[1] - home[0]).abs() < slack && (last[1] - point(0.75)[0]).abs() < 1e-9,
                "{first:?} to {last:?}"
            );
        } else {
            assert!(
                (first[0] - home[0]).abs() < slack
                    && (last[0] - point(0.75)[0]).abs() < 1e-9
                    && (last[1] - point(0.75)[1]).abs() < 1e-9,
                "{first:?} to {last:?}"
            );
        }
        // The same path as places in the 3D scene, from where the scene draws the tool now to where the
        // machine's own mapping puts the end of the file. A machine without that mapping has no line and no trail (the
        // rotary table: its tool pose is an angle, no point).
        let run = model.path_run.as_ref().unwrap();
        let apart = |a: [f64; 3], b: [f64; 3]| (0..3).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>().sqrt();
        if let Some(end) = sim.machine.scene_point(&point(0.75)) {
            assert_eq!(run.scene_line.len(), run.points.len(), "{}", entry.id);
            let (planned_start, planned_end) = (run.scene_line[0], run.scene_line[run.scene_line.len() - 1]);
            // The line spans the way from the start pose to the end, not a point: half of it at least, not 0.1 m, which the
            // planar delta's 51 mm from its start to 3/4 of its strokes are not.
            let way = apart(sim.machine.scene_point(&home).unwrap(), end);
            assert!(apart(planned_end, end) < 1e-9 && apart(planned_start, planned_end) > 0.5 * way, "{}", entry.id);
            let twist = twist_in_scene(sim.machine.as_ref(), &home);
            // The trail starts once the machine has taken the path, not when it is sent.
            let sent = Instant::now();
            while model.trail.is_none() && sent.elapsed() < Duration::from_secs(60) {
                model.poll();
                std::thread::sleep(Duration::from_millis(1));
            }
            assert!(model.trail.is_some(), "{}", entry.id);
            // And the machine in the app has been sent on its way: it arrives.
            simulate(&mut model, load.duration);
            assert!(
                (latest(&model, &position) - point(0.75)[0]).abs() < 1e-3,
                "{}: the machine stands at {}",
                entry.id,
                latest(&model, &position)
            );
            // The path the tool took, as the scenes reported it on the way: from the start of the planned line to
            // its end, never far from it.
            let trail = model.trail.clone().unwrap();
            let line = &model.path_run.as_ref().unwrap().scene_line;
            let off = trail
                .iter()
                .map(|p| line.iter().map(|q| apart(*p, *q)).fold(f64::INFINITY, f64::min))
                .fold(0.0, f64::max);
            // A point per 0.1 mm of travel whatever the frame rate: no two further apart than that and one step.
            let widest = trail.windows(2).map(|w| apart(w[0], w[1])).fold(0.0, f64::max);
            assert!(widest < 3e-4, "{}: {widest} m between two points of the trail", entry.id);
            assert!(
                trail.len() >= 3
                    && apart(trail[0], planned_start) < slack.max(1e-3) + twist
                    && apart(trail[trail.len() - 1], planned_end) < 2e-3 + twist
                    && off < 5e-3 + twist,
                "{}: {} points, {off} m off the planned line, {} m from its start, {} m from its end",
                entry.id,
                trail.len(),
                apart(trail[0], planned_start),
                apart(trail[trail.len() - 1], planned_end)
            );
        } else {
            assert!(run.scene_line.is_empty(), "{}", entry.id);
            simulate(&mut model, load.duration);
            assert!(
                model.trail.is_none() && (latest(&model, &position) - point(0.75)[0]).abs() < 1e-3,
                "{}: the machine stands at {}",
                entry.id,
                latest(&model, &position)
            );
        }

        // Poses at times: the same path backwards, from where the machine stands now.
        let back: String = points
            .iter()
            .zip(points.iter().rev())
            .map(|((time, _), (_, pose))| format!("{time:?},{}\n", text(pose)))
            .collect();
        std::fs::write(&timed, format!("time,pose\n{back}")).unwrap();
        (model.inputs.path_file, model.inputs.path_timed) = (timed.clone(), true);
        model.run_analysis(Analysis::Path);
        wait_for_analysis(&mut model);
        assert!(
            (number(&row(&model.path_tables()[0], "Duration [s]")[0]) - load.duration).abs() < 2e-3,
            "{}: {:?}",
            entry.id,
            model.path_tables()[0]
        );
        simulate(&mut model, load.duration);
        assert!(
            (latest(&model, &position) - home[0]).abs() < 1e-3,
            "{}: the machine stands at {}",
            entry.id,
            latest(&model, &position)
        );

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
        let key = payload_key(&model.config);
        let payload = model.config.f64(key).unwrap();
        model.set_param(key, toml::Value::Float(payload + 0.5));
        wait_for_jobs(&mut model);
        assert_eq!(model.messages.last().map(|m| m.0), Some("ui-analysis-stale"), "{:?}", model.messages);
        let fast = number(&row(&model.path_tables()[0], "Duration [s]")[0]);
        // Sooner where the motors bind before the belts: where the axis limits take a belt section below the tension the
        // time-optimal plan keeps, that plan is slower (the linear delta; cli.rs checks that it keeps the belts above it).
        let cfg = entry.config();
        let floor = cfg.f64("ratings.belt_min_tension_n").unwrap_or(f64::NEG_INFINITY)
            + cfg.f64("profile.tension_margin_n").unwrap_or(0.0);
        let belts_bind = load.belt_tension.is_some_and(|(slackest, _)| slackest < floor);
        // Likewise where the axis limits take a gearbox's or pulley's output torque above the share of its rating the
        // time-optimal plan keeps (profile.torque_reserve; a limit of that plan since 14 November): the planar delta's
        // lines within the axis limits take its gearbox 1 to 248 N m of its 192 N m, the plan keeps 0.8 * 192 = 153.6 N m
        // in its rigid model and takes 0.588 s against 0.581 s (sim-cli's test checks it is gentler then).
        let reserve = cfg.f64("profile.torque_reserve").unwrap();
        let gears_bind =
            judged.iter().any(|v| matches!(v.check, "gearbox-peak" | "pulley-torque") && v.value > reserve * v.limit);
        assert!(
            (fast < slow || belts_bind || gears_bind) && model.outdated(Analysis::Path),
            "{}: {fast} s as fast as the motors allow, {slow} s within the axis limits",
            entry.id
        );
        simulate(&mut model, 0.5);
        assert!(
            (latest(&model, &position) - home[0]).abs() < 1e-3,
            "{}: the machine ran a path planned for another config",
            entry.id
        );
        model.set_param(key, toml::Value::Float(payload));
        assert!(!model.outdated(Analysis::Path));
        model.clear(Analysis::Path);
        assert!(model.path_tables().is_empty() && model.path_plot().is_none());

        // From an end of the strokes, where a move has taken the machine: the plan starts where the motors
        // of a machine at rest there stand, some hundredths of a millimetre beyond the travel range, and was
        // refused for that (`error-trajectory`). It is run, and so is the path back onto the ends.
        let mut ends: Vec<f64> = axes.iter().map(|a| a.max).collect();
        // Where the corner of the strokes is outside the working range (the delta's cylinder), the ends of the first
        // and the last axis with the others in the middle: on the rim of its top.
        if sim.machine.unreachable(&ends, 0.0).is_some() {
            for i in 1..axes.len() - 1 {
                ends[i] = 0.5 * (axes[i].min + axes[i].max);
            }
        }
        model.messages.clear();
        model.set_speed(20.0);
        model.command(Command::MoveTo(ends.clone()));
        simulate(&mut model, 4.0);
        assert!((latest(&model, &position) - ends[0]).abs() < 1e-3);
        for (name, to) in [("from-the-ends.csv", point(0.5)), ("onto-the-ends.csv", ends.clone())] {
            let file = temp_file(entry, name);
            std::fs::write(
                &file,
                format!(
                    "line,{}
",
                    text(&to)
                ),
            )
            .unwrap();
            (model.inputs.path_file, model.inputs.time_optimal) = (file, false);
            model.run_analysis(Analysis::Path);
            wait_for_analysis(&mut model);
            simulate(&mut model, 4.0);
            assert!(
                (latest(&model, &position) - to[0]).abs() < 1e-3,
                "{}: {name}: the machine stands at {} ({:?})",
                entry.id,
                latest(&model, &position),
                model.messages
            );
        }

        // Transition curves, where the machine has the two axes they are worked out in: a line, a quarter turn of
        // 50 mm radius and a line, towards the middle of the workspace. With a tolerance of 0.1 mm the
        // planned path is off by the 0.1 mm the library reports, which the table shows, and takes less time.
        if axes.len() != 2 {
            continue;
        }
        let towards = |i: usize| (0.5 * (axes[i].min + axes[i].max) - home[i]).signum();
        let (dx, dz) = (towards(0), towards(1));
        let corner = [home[0] + 0.15 * dx, home[1]];
        let arc_end = [corner[0] + 0.05 * dx, corner[1] + 0.05 * dz];
        let turn = if dx * dz > 0.0 { "ccw" } else { "cw" };
        let rounded = temp_file(entry, "rounded.csv");
        // First onto the start pose, so that the line after it is the arc's tangent: the plan starts at the setpoint at
        // rest, which the planar delta's sagging arms hold some tenths of a millimetre above its start pose, and from there
        // the line to the corner would meet the arc more than 1 mrad off its tangent, a corner, where the path stops with
        // transition curves or without (as in cli.rs).
        let pieces = format!(
            "line,{}\nline,{}\narc,{},{},{turn}\nline,{}\n",
            text(&home),
            text(&corner),
            text(&arc_end),
            text(&[corner[0], arc_end[1]]),
            text(&[arc_end[0], arc_end[1] + 0.1 * dz])
        );
        std::fs::write(&rounded, &pieces).unwrap();
        let planned = |tolerance: f64| -> (f64, f64) {
            let mut model = Model::new(entry, "en");
            wait_for_data(&mut model);
            model.set_param("profile.path_tolerance_m", toml::Value::Float(tolerance));
            model.inputs.path_file = rounded.clone();
            model.run_analysis(Analysis::Path);
            wait_for_analysis(&mut model);
            let summary = &model.path_tables()[0];
            (
                number(&row(summary, "Duration [s]")[0]),
                number(&row(summary, "Planned path off the given one, at most [mm]")[0]),
            )
        };
        let mut blended = entry.config();
        blended.set_text("profile.path_tolerance_m = 1.0e-4").unwrap();
        let reported = sim_components::Sim::new(entry, &blended)
            .unwrap()
            .path_deviation(&sim_components::path::parse(&pieces).unwrap(), false)
            .unwrap();
        let ((given, none), (shorter, off)) = (planned(0.0), planned(1e-4));
        assert!(
            none == 0.0 && (reported - 1e-4).abs() < 1e-9 && (off - reported * 1e3).abs() < 1e-3,
            "{}: off by {off} mm, reported {reported} m",
            entry.id
        );
        assert!(shorter < 0.95 * given, "{}: {shorter} s with transition curves, {given} s without", entry.id);
    }
}

/// Where an arm's tool is drawn off its pose by the gears' twist under its weight: the distance in the scene of the tool
/// at `pose` and with every axis the machine's setpoint slack further. An arm's poses are joint angles, so its slack of
/// some mrad is some mm at its tool. 0 on a machine whose axes are all in metres, where the slack is a distance already.
fn twist_in_scene(machine: &dyn sim_core::Machine, pose: &[f64]) -> f64 {
    if machine.axes().iter().all(|a| a.unit == "m") {
        return 0.0;
    }
    let off: Vec<f64> = pose.iter().map(|p| p + machine.setpoint_slack()).collect();
    let (a, b) = (machine.scene_point(pose).unwrap(), machine.scene_point(&off).unwrap());
    (0..3).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>().sqrt()
}

/// Trace replay and fit in the app, as `sim-cli replay` and `fit --trace`: a trace the simulator wrote itself
/// replays onto itself and fits the config's own values, with the advice in the chosen language. A file
/// that is no trace is refused with a message. Since 13 November also the delta, the SCARA and the arm, which are
/// fitted in their motors' coordinates (`identify::fit_joints`).
#[test]
fn a_trace_is_replayed_and_fitted_in_the_app() {
    on_every_machine(|entry| {
        let per_axis = sim_components::trace::per_axis_fit((entry.build)(&entry.config()).unwrap().as_ref());
        // A trace as a run writes it: one pass of the cycle, a row per millisecond. The gripper stays
        // empty, because a trace does not say when a payload comes and goes.
        // And the encoders count without steps: their quantisation noise is not replayed sample by sample (the linear
        // motor's 1 um scale under its stiff velocity loop makes some 10 N of it).
        let mut cfg = entry.config();
        cfg.set_text("effects.payload = false").unwrap();
        cfg.set_text("effects.encoder_resolution = false").unwrap();
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
        model.set_param("effects.encoder_resolution", toml::Value::Boolean(false));
        model.inputs.trace_file = file.clone();
        model.run_analysis(Analysis::Trace);
        wait_for_analysis(&mut model);
        let tables = model.trace_tables();
        let [replayed, fitted, _identified] = &tables[..] else { panic!("{} tables", tables.len()) };
        // Per motor the difference between trace and replay: within 0.5 % (RMS) and 5 % (peak) of the largest torque.
        assert_eq!(
            (replayed.len(), replayed[0][0].as_str(), replayed[0].len()),
            (5, "Figure", 1 + measured.motors.len())
        );
        for (k, motor) in measured.motors.iter().enumerate() {
            let largest = motor.torque.iter().fold(0.0, |a: f64, b| a.max(b.abs()));
            let (rms, peak) =
                (number(&row(replayed, "Torque, RMS [N m]")[k]), number(&row(replayed, "Torque, peak [N m]")[k]));
            assert!(
                largest > 0.1 && rms > 0.0 && rms < 0.005 * largest && peak < 0.05 * largest,
                "{}: RMS {rms}, peak {peak} of {largest} N m",
                entry.id
            );
            assert!(number(&row(replayed, "Following error, peak [rad]")[k]) < 0.01, "{}: {replayed:?}", entry.id);
        }
        // Per axis the four fitted values next to the config's own: on its own trace the fit finds them.
        assert_eq!(fitted[0], ["Quantity", "Configuration", "Fitted", "Standard error"]);
        // Per axis four values; in the motors' coordinates four per motor (no payload's factor: the gripper is empty), and
        // on the SCARA and the arm the links' values the trace tells apart, by the link's name.
        let (owners, quantities, first) = if per_axis {
            (
                model.session.axes.iter().map(|a| a.key).collect::<Vec<_>>(),
                ["Moved mass [kg]", "Coulomb friction [N]"],
                ("Moved mass, axis ", "Bewegte Masse, Achse "),
            )
        } else {
            (
                model.session.motors.iter().map(|m| m.key).collect(),
                ["Coulomb friction [N m]"; 2],
                ("Inertia at motor ", "Trägheit an Motor "),
            )
        };
        let link_labels: Vec<String> = sim_components::identify::LINK_QUANTITIES
            .iter()
            .map(|q| model.tr.tr(&format!("ui-trace-{}", q.replace('_', "-"))).to_string())
            .collect();
        let own_rows =
            fitted.iter().skip(1).filter(|r| !link_labels.iter().any(|l| r[0].ends_with(l.as_str()))).count();
        assert_eq!(own_rows, 4 * owners.len(), "{}: {fitted:?}", entry.id);
        for owner in &owners {
            for quantity in quantities {
                let cells = row(fitted, &format!("{} · {quantity}", model.tr.tr(owner)));
                let (config, fit) = (number(&cells[0]), number(&cells[1]));
                assert!(
                    config > 0.0 && (fit / config - 1.0).abs() < 0.02 && number(&cells[2]) > 0.0,
                    "{}: {quantity} {cells:?}",
                    entry.id
                );
            }
        }
        // What to change in the config, four lines per axis or motor (then the links' and what the values combine), in the
        // chosen language.
        let advice = model.trace_advice();
        assert!(
            advice.len() >= 4 * owners.len() && advice[0].starts_with(first.0) && advice[0].contains(" ± "),
            "{advice:?}"
        );
        model.set_language("de");
        assert!(
            model.trace_advice()[0].starts_with(first.1)
                && model.trace_tables()[1][0][0] == "Größe"
                && model.trace_tables()[0][1][0] == "Moment, Effektivwert [N m]"
        );
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
        assert_eq!(model.trace_tables().len(), 3);
    });
}

/// A drive that trips on DC link overvoltage takes all drives off, which used to show only as a machine that
/// stops. Now it is said, once per trip, for the machine in the app, and for an analysed cycle that trips.
/// The DC link here has a hundredth of its capacitors and a resistor that takes 5 W, so the first hard
/// braking is too much for it.
#[test]
fn a_drive_that_trips_is_reported() {
    let weak = [("ratings.dc_link_capacitance_uf", 4.7), ("ratings.braking_resistor_peak_w", 5.0)];
    let trips = |events: &[Event]| events.iter().filter(|e| matches!(e, Event::Refused("ui-drive-tripped", _))).count();
    on_every_machine(|entry| {
        let mut cfg = entry.config();
        weak.iter().for_each(|(path, value)| cfg.set_text(&format!("{path} = {value:?}")).unwrap());
        let mut runner = Runner::new(entry, &cfg).unwrap();
        let mut events = Vec::new();
        runner.handle(Request::Command(Command::StartCycle), &mut events);
        runner.advance(5.0, &mut events);
        assert!(
            trips(&events) == 1 && !runner.sim.control.enabled(),
            "{}: {} trips reported",
            entry.id,
            trips(&events)
        );
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
        assert_eq!(
            model.messages.iter().filter(|m| m.0 == "ui-analysis-tripped").count(),
            1,
            "{}: {:?}",
            entry.id,
            model.messages
        );
        assert!(
            model.cycle_tables()[2].iter().any(|row| row[0] == "Highest DC link voltage [V]" && row[4] == "FAIL"),
            "{}: {:?}",
            entry.id,
            model.cycle_tables()[2]
        );
        model.set_speed(10.0);
        model.command(Command::StartCycle);
        simulate(&mut model, 8.0);
        assert_eq!(
            model.messages.iter().filter(|m| m.0 == "ui-drive-tripped").count(),
            1,
            "{}: {:?}",
            entry.id,
            model.messages
        );
        for lang in LANGUAGES {
            model.set_language(lang);
            assert!(model.tr.has("ui-drive-tripped") && model.tr.has("ui-analysis-tripped"));
        }
    });
}

/// The travelled path for the 3D view. From the simulation steps, not from the frames: a point per 0.1 mm of
/// travel however long a frame takes (it was one point per frame, 13 mm apart at speed). It starts when the
/// machine takes a trajectory and not when it refuses one, ends on a reconfigure and on the start of a
/// cycle, and at its cap of points drops the oldest instead of standing still, in the queue and in the model.
#[test]
fn the_travelled_path_follows_every_step_and_starts_and_ends_with_the_path() {
    use sim_app::session::{Outbox, TRAIL, extend_trail};
    let apart = |a: [f64; 3], b: [f64; 3]| (0..3).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>().sqrt();
    let starts = |events: &[Event]| {
        events.iter().filter_map(|e| if let Event::TrailStart(at) = e { Some(*at) } else { None }).collect::<Vec<_>>()
    };
    let trailed = |events: &[Event]| {
        events
            .iter()
            .filter_map(|e| if let Event::Trail(points) = e { Some(points.clone()) } else { None })
            .flatten()
            .collect::<Vec<_>>()
    };
    for entry in MACHINES.iter().copied() {
        let mut runner = Runner::new(entry, &entry.config()).unwrap();
        // A trail needs the tool as a point in the scene, which a machine may not have (the rotary table: an angle).
        if runner.sim.tool_point().is_none() {
            continue;
        }
        let axes = runner.sim.machine.axes().to_vec();
        let far: Vec<f64> = axes.iter().map(|a| a.min + 0.75 * (a.max - a.min)).collect();
        // The tool at rest is off its pose by what the gears give way (an arm's twist, some mm at its tool). On a machine
        // in metres that is its setpoint slack, a distance already: 0.1 mm on the belts, 2.32 mm on the delta (its
        // gearboxes' twist under the arms' weight and half their backlash). There, with the forearms as rods since
        // 13 November, the tool rests 1.48 mm off `far` (1.22 mm with the lumped forearms) and the trail ends 1.006 mm
        // off it (0.884 mm), past the 1 mm this bound had without the slack.
        let twist = if axes.iter().all(|a| a.unit == "m") {
            runner.sim.machine.setpoint_slack()
        } else {
            twist_in_scene(runner.sim.machine.as_ref(), &far)
        };
        let points = runner.sim.plan_path(&[sim_components::path::Piece::Line(far.clone())], false).unwrap();
        let duration = points[points.len() - 1].0;

        // Refused (it starts elsewhere): no trail, and none while the machine stands.
        let mut events = Vec::new();
        let elsewhere = points.iter().map(|(t, _)| (*t, far.clone())).collect();
        runner.handle(Request::Command(Command::Trajectory(elsewhere)), &mut events);
        runner.advance(0.05, &mut events);
        assert!(
            matches!(events[0], Event::Refused("error-trajectory-start", _))
                && starts(&events).is_empty()
                && trailed(&events).is_empty(),
            "{}",
            entry.id
        );

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
        // 0.1 mm and what the tool travels in one step: below 0.2 mm on the belt machines, 0.1 mm + 3 m/s × 62.5 us on
        // the delta.
        let step = axes.iter().map(|a| a.max_vel * a.max_vel).sum::<f64>().sqrt() * runner.sim.dt;
        assert!(widest > 1e-4 && widest < f64::max(2e-4, 1e-4 + step), "{}: {widest} m between two points", entry.id);
        assert!(
            trail.len() as f64 > 0.9 * apart(from, to) / 2e-4 && apart(trail[trail.len() - 1], to) < 1e-3 + twist,
            "{}: {} points over {} m, {} m from its end",
            entry.id,
            trail.len(),
            apart(from, to),
            apart(trail[trail.len() - 1], to)
        );

        // The trail ends with the path: a move after it adds no points and says nothing, and the finished
        // trail stays until the reconfigure below.
        events.clear();
        let back: Vec<f64> = axes.iter().map(|a| a.min + 0.25 * (a.max - a.min)).collect();
        runner.handle(Request::Command(Command::MoveTo(back)), &mut events);
        runner.advance(1.0, &mut events);
        assert!(apart(runner.sim.tool_point().unwrap(), to) > 0.01, "{}: the move did not run", entry.id);
        assert!(
            starts(&events).is_empty() && trailed(&events).is_empty(),
            "{}: {} points from a move after the path",
            entry.id,
            trailed(&events).len()
        );

        // A reconfigure ends it, and so does a cycle after the next path; a cycle with no trail says nothing.
        events.clear();
        runner.handle(Request::Reconfigure(entry.config()), &mut events);
        assert_eq!(starts(&events), [None], "{}", entry.id);

        // Commands the machine takes while the path runs: a stop after the segment and an enable leave it
        // running, and the trail grows to the path's end (it stopped at the command). Drives off ends both.
        for (command, runs_on) in [(Command::Stop, true), (Command::Enable, true), (Command::Disable, false)] {
            let mut runner = Runner::new(entry, &entry.config()).unwrap();
            let points = runner.sim.plan_path(&[sim_components::path::Piece::Line(far.clone())], false).unwrap();
            events.clear();
            runner.handle(Request::Command(Command::Trajectory(points)), &mut events);
            runner.advance(0.5 * duration, &mut events);
            let before = trailed(&events).len();
            runner.handle(Request::Command(command.clone()), &mut events);
            // Past the first step, in which a tool that is let go still moves.
            runner.advance(0.01, &mut events);
            let at_command = trailed(&events).len();
            runner.advance(0.5 * duration + 0.3, &mut events);
            let trail = trailed(&events);
            assert!(before > 10 && starts(&events).len() == 1, "{}: {command:?}", entry.id);
            if runs_on {
                assert!(
                    apart(trail[trail.len() - 1], to) < 1e-3 + twist,
                    "{}: the trail ends {} m before the path's end after {command:?}",
                    entry.id,
                    apart(trail[trail.len() - 1], to)
                );
            } else {
                assert!(
                    trail.len() == at_command && apart(trail[trail.len() - 1], to) > 0.01,
                    "{}: {} points after {command:?}",
                    entry.id,
                    trail.len() - at_command
                );
            }
        }
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
    assert!(
        matches!(events[0], Event::TrailStart(Some(_)))
            && events.iter().filter(|e| matches!(e, Event::Trail(_))).count() == 1
    );
    let waiting = trailed(&events);
    assert_eq!((waiting.len(), waiting[0][0], waiting[TRAIL - 1][0]), (TRAIL, 2000.0, (TRAIL + 1999) as f64));
}

/// A config file in the app, as `--config FILE` on the command line: opened, it replaces the running config
/// as switching machine does; saved, it holds every value changed in the panels, with the comments and the
/// layout of the file it was read from; opened again, it is the same config. A file that cannot be read, or
/// that the machine refuses, is reported in words and the running config stays.
#[test]
fn a_config_file_is_opened_changed_saved_and_opened_again() {
    for entry in MACHINES.iter().copied() {
        let mut model = Model::new(entry, "en");
        let (original, saved) = (temp_file(entry, "config.toml"), temp_file(entry, "saved.toml"));
        std::fs::write(&original, entry.default_config).unwrap();
        model.open_config(&original);
        assert_eq!((model.config == entry.config(), model.inputs.config_file.as_str()), (true, original.as_str()));
        assert_eq!(model.messages.last(), Some(&("ui-config-opened", original.clone())));
        wait_for_data(&mut model);

        // A value and an effect switch changed in the panels.
        let key = payload_key(&model.config);
        let payload = model.config.f64(key).unwrap() + 0.75;
        model.set_param(key, toml::Value::Float(payload));
        let (switch, _, on) = model.effects().into_iter().next().unwrap();
        model.set_param(&switch, toml::Value::Boolean(!on));
        let changed = model.config.clone();
        model.save_config(&saved);
        assert_eq!(model.messages.last(), Some(&("ui-config-saved", saved.clone())));
        // Line for line the file it was read from, but for the two values.
        let text = std::fs::read_to_string(&saved).unwrap();
        assert_eq!(text.lines().count(), entry.default_config.lines().count(), "{text}");
        let differ: Vec<(&str, &str)> =
            entry.default_config.lines().zip(text.lines()).filter(|(a, b)| a != b).collect();
        assert_eq!(differ.len(), 2, "{}: {differ:?}", entry.id);
        assert!(
            differ.iter().any(|(_, line)| line.starts_with(&format!("{} = ", key.rsplit('.').next().unwrap()))),
            "{differ:?}"
        );

        // Opened again, it is the config as it was changed.
        model.open_config(&saved);
        assert!(model.config == changed && model.config.f64(key).unwrap() == payload, "{}", entry.id);
        wait_for_data(&mut model);
        assert!(
            model.messages.iter().all(|m| m.0.starts_with("limit-") || m.0.starts_with("ui-config-")),
            "{:?}",
            model.messages
        );

        // A changed array is not written over the old text: the file is written plainly and still reads back.
        let mut moved = model.config.clone();
        // 10 mm towards the middle of each axis: a start pose may be 10 mm from an end (the linear delta's at its top),
        // where -1.13 + 0.01 lies a rounding beyond -1.12.
        let middle = |a: &sim_core::AxisInfo| 0.5 * (a.min + a.max);
        let pose: Vec<f64> = moved
            .f64s("start.pose")
            .unwrap()
            .iter()
            .zip(&model.session.axes)
            .map(|(p, a)| p + 0.01 * (middle(a) - p).signum())
            .collect();
        moved.set_text(&format!("start.pose = {pose:?}")).unwrap();
        std::fs::write(&original, moved.to_text().unwrap()).unwrap();
        model.open_config(&original);
        assert!(model.config == moved, "{}: {:?}", entry.id, model.messages);

        // Files that do not load: a misspelt key, a value the machine refuses, a file that is not there. The
        // message says why in the user's language, and the config that runs stays.
        let mut refused = entry.config();
        refused.set_text(&format!("start.pose = {:?}", vec![1000.0; pose.len()])).unwrap();
        let bad_files = [
            ("format_version = 1\n[mass]\npayload_kgg = 1.0\n".to_string(), "error-config-unknown"),
            (refused.to_text().unwrap(), "error-config-invalid"),
            (String::new(), "ui-file-unreadable"),
        ];
        for (text, key) in bad_files {
            let broken = temp_file(entry, "broken.toml");
            let _ = std::fs::remove_file(&broken);
            if !text.is_empty() {
                std::fs::write(&broken, text).unwrap();
            }
            model.open_config(&broken);
            assert_eq!(model.messages.last().map(|m| m.0), Some(key), "{:?}", model.messages);
            assert!(model.config == moved && model.inputs.config_file == original, "{}", entry.id);
            for lang in LANGUAGES {
                model.set_language(lang);
                assert!(model.tr.has(key));
            }
            model.set_language("en");
        }
        // The simulation that was running runs on.
        wait_for_data(&mut model);
        let before = model.history.back().unwrap()[0];
        simulate(&mut model, 0.05);
        assert!(model.history.back().unwrap()[0] > before);
    }
}

/// Switching machine forgets the config file: Save wrote the new machine's config over the file of the machine
/// left. And a file that holds another machine's config is not overwritten, also when it is typed in: the
/// message says so in words and the file stays as it was.
#[test]
fn saving_after_a_machine_switch_does_not_overwrite_the_other_machines_file() {
    let (first, second) = (MACHINES[0], MACHINES[1]);
    let file = temp_file(first, "own-config.toml");
    std::fs::write(&file, first.default_config).unwrap();
    let mut model = Model::new(first, "en");
    model.open_config(&file);
    assert_eq!(model.inputs.config_file, file);
    model.set_machine(second);
    assert_eq!(model.inputs.config_file, "", "the file of the machine left is still in the field");
    model.save_config(&file);
    assert_eq!(
        std::fs::read_to_string(&file).unwrap(),
        first.default_config,
        "the other machine's file was overwritten"
    );
    assert_eq!(model.messages.last().map(|m| m.0), Some("ui-config-other-machine"), "{:?}", model.messages);
    for lang in LANGUAGES {
        model.set_language(lang);
        assert!(model.tr.has("ui-config-other-machine"));
    }
    // A file of the machine's own, or one that is no config, is written as before.
    let own = temp_file(second, "own-config.toml");
    std::fs::write(&own, "not a config").unwrap();
    model.save_config(&own);
    assert_eq!(model.messages.last().map(|m| m.0), Some("ui-config-saved"));
    model.save_config(&own);
    assert_eq!(model.messages.last().map(|m| m.0), Some("ui-config-saved"));
}

/// The motor a measured response was taken on is a number of the machine shown: switched to a machine with
/// fewer motors, it is one that machine has. It stayed at the old number, and the fit was refused with
/// `error-config-invalid` "motor".
#[test]
fn switching_to_a_machine_with_fewer_motors_keeps_the_sweep_motor_in_range() {
    let motors = |entry: &'static sim_core::MachineEntry| Model::new(entry, "en").session.motors.len();
    let many = MACHINES.iter().copied().max_by_key(|m| motors(m)).unwrap();
    let few = MACHINES.iter().copied().min_by_key(|m| motors(m)).unwrap();
    assert!(motors(many) > motors(few));
    let mut model = Model::new(many, "en");
    model.inputs.sweep_motor = motors(many) - 1;
    model.set_machine(few);
    assert!(
        model.inputs.sweep_motor < model.session.motors.len(),
        "motor {} of {}",
        model.inputs.sweep_motor,
        model.session.motors.len()
    );
}

/// Parts typed by name, as `--motor`, `--supply` and `--gearbox` on the command line: the config they give
/// is the one the command line runs, and the one the motor search's button gives for the same motor. Names
/// the catalogue does not have are refused in words, and so is any name in a config without `[parts]`.
#[test]
fn typed_part_names_give_the_config_of_the_search_button_and_the_command_line() {
    use sim_components::catalog::resolve;
    let entry = MACHINES[0];
    let motor = "1FK2104-5AK";
    let mut typed = Model::new(entry, "en");
    assert_eq!(typed.inputs.parts, ["", "400v", "", "", "", ""].map(String::from));
    // With the converter the search pairs with this motor, which its button names too.
    typed.inputs.parts = [motor, "400v", "", "6SL3210-5HE11-5UF0", "", ""].map(String::from);
    typed.apply_parts();
    // The command line: `--motor 1FK2104-5AK --supply 400v`, the parts put in by `resolve`.
    let mut cli = entry.config();
    cli.set_text(&format!("parts.motor = {motor:?}")).unwrap();
    cli.set_text(r#"parts.motor_supply = "400v""#).unwrap();
    cli.set_text(r#"parts.drive = "6SL3210-5HE11-5UF0""#).unwrap();
    assert!(resolve(entry, &typed.config).unwrap() == resolve(entry, &cli).unwrap());
    // The search's button, for the same motor.
    let mut searched = Model::new(entry, "en");
    searched.inputs.series = motor.into();
    searched.run_analysis(Analysis::Search);
    wait_for_analysis(&mut searched);
    assert_eq!(searched.search_table().unwrap()[1][0], motor);
    searched.use_candidate(0);
    assert!(searched.config == typed.config, "{:?}", searched.messages);
    assert_eq!(searched.inputs.parts, typed.inputs.parts);
    // The running simulation takes it.
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(300) {
        typed.poll();
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(typed.messages.iter().all(|m| m.0.starts_with("limit-")), "{:?}", typed.messages);
    // A gearbox too, and then none again.
    typed.inputs.parts[2] = "NRK070-04-5".into();
    typed.apply_parts();
    cli.set_text(r#"parts.gearbox = "NRK070-04-5""#).unwrap();
    assert!(resolve(entry, &typed.config).unwrap() == resolve(entry, &cli).unwrap());
    typed.inputs.parts[2] = String::new();
    typed.apply_parts();
    assert_eq!(typed.config.get("parts.gearbox").unwrap().as_str(), Some(""));

    // Names the catalogue does not have, and a supply it has no data for: refused, the config stays.
    let before = typed.config.clone();
    for (parts, (key, name)) in [
        (["1FK9999-9XX", "400v", "", "", "", ""], ("error-config-part-motor", "1FK9999-9XX")),
        ([motor, "400v", "NRK999-04-5", "", "", ""], ("error-config-part-gearbox", "NRK999-04-5")),
        ([motor, "110v", "", "", "", ""], ("ui-search-bad-supply", "110v")),
        ([motor, "400v", "", "6SL0000-0XX00-0XX0", "", ""], ("error-config-part-drive", "6SL0000-0XX00-0XX0")),
        ([motor, "400v", "", "", "XT5-25", ""], ("error-config-part-belt", "XT5-25")),
        ([motor, "400v", "", "", "", "EXCT-99"], ("error-config-part-axis", "EXCT-99")),
    ] {
        typed.inputs.parts = parts.map(String::from);
        typed.apply_parts();
        assert_eq!(typed.messages.last(), Some(&(key, name.to_string())));
        assert!(typed.config == before && typed.tr.tr_arg(key, name).contains(name));
    }
    // A config without `[parts]` takes no names, as on the command line.
    let mut table: toml::Table = entry.default_config.parse().unwrap();
    table.remove("parts");
    let file = temp_file(entry, "no-parts.toml");
    std::fs::write(&file, toml::to_string(&table).unwrap()).unwrap();
    typed.open_config(&file);
    assert_eq!(typed.inputs.parts, ["", "", "", "", "", ""].map(String::from));
    let opened = typed.config.clone();
    typed.inputs.parts[0] = motor.into();
    typed.apply_parts();
    assert_eq!(typed.messages.last(), Some(&("error-config-missing", "parts.motor".to_string())));
    assert!(typed.config == opened);
}

/// The overlay of a replay in the app, as `sim-cli replay --out` writes it: per quantity and motor the trace and
/// the simulation of its setpoints, and exported as CSV the same file the command line writes
/// (`trace::overlay_csv`, which `cli.rs` checks against the command line's file).
#[test]
fn the_replayed_trace_is_overlaid_and_exported_in_the_app() {
    // Since 13 November the delta too: replay takes a transformation that is not linear.
    for entry in MACHINES.iter().copied() {
        let mut cfg = entry.config();
        cfg.set_text("effects.payload = false").unwrap();
        let mut sim = sim_components::Sim::new(entry, &cfg).unwrap();
        sim.command(Command::StartCycle).unwrap();
        let mut measured = sim_components::trace::Trace::default();
        for step in 0..=(3.0 / sim.dt).round() as u64 {
            if step % 4 == 0 {
                measured.record(&sim);
            }
            sim.step();
        }
        let ids: Vec<&str> = sim.machine.motors().iter().map(|m| m.id).collect();
        let file = temp_file(entry, "overlay-trace.csv");
        let mut text = String::from("time");
        for id in &ids {
            text += &["angle_setpoint", "angle", "speed", "torque"].map(|n| format!(",motor.{id}.{n}")).concat();
        }
        for (row, time) in measured.time.iter().enumerate() {
            text += &format!("\n{time:.9e}");
            for m in &measured.motors {
                text +=
                    &[m.setpoint[row], m.angle[row], m.speed[row], m.torque[row]].map(|v| format!(",{v:.9e}")).concat();
            }
        }
        std::fs::write(&file, &text).unwrap();
        let measured = sim_components::trace::Trace::parse(&text, sim.machine.motors()).unwrap();
        let simulated = sim_components::trace::replay(entry, &cfg, &measured).unwrap();

        let mut model = Model::new(entry, "en");
        assert!(model.overlay().is_empty());
        model.set_param("effects.payload", toml::Value::Boolean(false));
        model.inputs.trace_file = file;
        model.run_analysis(Analysis::Trace);
        wait_for_analysis(&mut model);
        let overlay = model.overlay();
        assert_eq!(
            overlay.iter().map(|(key, _)| *key).collect::<Vec<_>>(),
            ["ui-overlay-torque", "ui-overlay-speed", "ui-overlay-error"]
        );
        for (q, (_, motors)) in overlay.iter().enumerate() {
            assert_eq!(motors.len(), ids.len());
            for (k, [m, s]) in motors.iter().enumerate() {
                let column = |t: &sim_components::trace::MotorTrace| {
                    [t.torque.clone(), t.speed.clone(), t.following_error()][q].clone()
                };
                let line =
                    |values: Vec<f64>| measured.time.iter().zip(values).map(|(t, v)| [*t, v]).collect::<Vec<_>>();
                assert!(
                    *m == line(column(&measured.motors[k])) && *s == line(column(&simulated.motors[k])),
                    "{}: quantity {q}, motor {k}",
                    entry.id
                );
            }
        }
        // The replay is close to the trace, and not the same: the drawn lines are two.
        assert!(overlay[0].1[0][0] != overlay[0].1[0][1]);
        let exported = temp_file(entry, "overlay.csv");
        model.export_overlay(std::path::Path::new(&exported));
        assert_eq!(model.messages.last().unwrap().0, "ui-overlay-exported");
        let written = std::fs::read_to_string(&exported).unwrap();
        assert!(written == sim_components::trace::overlay_csv(&ids, &measured, &simulated), "{}", entry.id);
        assert!(
            written.starts_with(&format!(
                "time,motor.{0}.torque_measured,motor.{0}.torque_simulated,motor.{0}.speed_measured",
                ids[0]
            )) && written.lines().count() == 1 + measured.time.len()
        );
        model.clear(Analysis::Trace);
        assert!(model.overlay().is_empty());
    }
}

/// The operating points in the app, as `sim-cli analyze --points` writes them (`MotorLoad::points`, which
/// `cli.rs` checks against the command line's file), over the torque-speed curves. The peak curve by hand: the peak
/// torque both ways up to the rated speed, falling straight to nothing at the no-load speed in the direction
/// of motion, and the peak torque against the motion at every speed. The S1 curve: the rated torque both ways from
/// the rated speed backwards to the rated speed forwards (on the gantry 1.27 N m up to 314 rad/s).
#[test]
fn the_operating_points_are_shown_over_the_torque_speed_curve() {
    for entry in MACHINES.iter().copied() {
        let mut model = Model::new(entry, "en");
        assert!(
            model.points_plot(Analysis::Cycle).is_none()
                && model.points_plot(Analysis::Path).is_none()
                && model.points_table(0..10).is_none()
        );
        model.run_analysis(Analysis::Cycle);
        wait_for_analysis(&mut model);
        let load =
            sim_components::analysis::analyse_cycle(&mut sim_components::Sim::new(entry, &entry.config()).unwrap())
                .unwrap();
        let all: Vec<(usize, (f64, f64))> =
            load.motors.iter().enumerate().flat_map(|(k, m)| m.points.iter().map(move |p| (k, *p))).collect();
        let (count, table) = model.points_table(0..all.len()).unwrap();
        // One per millisecond of the cycle (0.72 s on the delta).
        assert!(count == all.len() && count > 700 * load.motors.len(), "{}: {count} points", entry.id);
        assert_eq!(table[0], ["Motor", "Motor speed [rad/s]", "Motor torque [N m]"]);
        for (row, (k, (speed, torque))) in table[1..].iter().zip(&all) {
            assert!(
                row[0] == model.tr.tr(model.session.motors[*k].key)
                    && shows(&row[1], *speed)
                    && shows(&row[2], *torque),
                "{}: {row:?} against {speed}, {torque}",
                entry.id
            );
        }
        // A window of the rows, as the window draws them: the header and the rows asked for.
        let (_, window) = model.points_table(count - 3..count + 5).unwrap();
        assert!(window.len() == 4 && shows(&window[3][2], all[count - 1].1.1), "{window:?}");

        let (points, [high, low, s1_high, s1_low]) = model.points_plot(Analysis::Cycle).unwrap();
        for (k, m) in load.motors.iter().enumerate() {
            assert_eq!(points[k], m.points.iter().map(|(s, t)| [*s, *t]).collect::<Vec<_>>());
        }
        let resolved = sim_components::catalog::resolve(entry, &entry.config()).unwrap();
        let [peak, rated, no_load] = ["drive.peak_torque_nm", "drive.rated_speed_rad_s", "drive.no_load_speed_rad_s"]
            .map(|path| resolved.f64(path).unwrap());
        assert_eq!(high, [[-no_load, peak], [-rated, peak], [0.0, peak], [rated, peak], [no_load, 0.0]]);
        assert_eq!(low, [[-no_load, 0.0], [-rated, -peak], [0.0, -peak], [rated, -peak], [no_load, -peak]]);
        let continuous = resolved.f64("drive.rated_torque_nm").unwrap();
        assert!(
            s1_high == [[-rated, continuous], [rated, continuous]]
                && s1_low == [[-rated, -continuous], [rated, -continuous]],
            "{}",
            entry.id
        );
        // Nothing of a path before one was run.
        assert!(model.points_plot(Analysis::Path).is_none());
        // The cycle runs inside the curve.
        assert!(all.iter().all(|(_, (speed, torque))| speed.abs() < rated && torque.abs() <= peak), "{}", entry.id);
        model.set_language("de");
        assert_eq!(model.points_table(0..1).unwrap().1[0][1], "Motordrehzahl [rad/s]");
        model.clear_analysis();
        assert!(model.points_plot(Analysis::Cycle).is_none());
    }
}

/// A frequency response as `sim-cli sweep` writes it.
fn sweep_csv(points: &[sim_components::analysis::SweepPoint]) -> String {
    let rows: Vec<String> =
        points.iter().map(|p| format!("{:.4},{:.6e},{:.3}", p.frequency, p.gain, p.phase_deg)).collect();
    format!("frequency_hz,gain_rad_s_per_nm,phase_deg\n{}\n", rows.join("\n"))
}

/// The open-loop frequency response in the app, as `sim-cli sweep` gives it (`frequency_sweep`, which `cli.rs`
/// checks against the command line's output): every motor at the frequencies and amplitude typed, the numbers
/// the same, and the dip of the load on the belt read off each (`identify::resonance`). A sweep from 0 Hz is
/// refused.
#[test]
fn the_frequency_response_is_measured_in_the_app() {
    for entry in MACHINES.iter().copied() {
        let mut model = Model::new(entry, "en");
        assert!(model.sweep_table().is_none());
        // The defaults of `sim-cli sweep`.
        let defaults = (
            5.0,
            500.0,
            sim_components::analysis::SWEEP_POINTS,
            sim_components::analysis::default_sweep_amplitude(entry, &entry.config()).unwrap(),
        );
        assert_eq!(
            (model.inputs.sweep_from, model.inputs.sweep_to, model.inputs.sweep_count, model.inputs.sweep_amplitude),
            defaults
        );
        (model.inputs.sweep_from, model.inputs.sweep_to, model.inputs.sweep_count, model.inputs.sweep_amplitude) =
            (50.0, 130.0, 30, 1.5);
        model.run_analysis(Analysis::Sweep);
        assert!(model.running(Analysis::Sweep));
        let before = model.history.len();
        wait_for_analysis(&mut model);
        // The simulation goes on beside the analysis (on a loaded machine its next sample can come after it).
        let start = Instant::now();
        while model.history.len() <= before && start.elapsed() < Duration::from_secs(10) {
            model.poll();
            std::thread::sleep(Duration::from_millis(10));
        }
        assert!(model.history.len() > before, "{}: the simulation stood still meanwhile", entry.id);
        let sweeps = model.sweeps.as_ref().unwrap();
        assert_eq!(sweeps.len(), model.session.motors.len());
        let frequencies = sim_components::analysis::log_spaced(50.0, 130.0, 30);
        let table = model.sweep_table().unwrap();
        assert_eq!(table[0][0], "Figure");
        // The numbers are those of `frequency_sweep`, checked on the first machine (the app runs the same code for every
        // one); on every machine the table shows the dip of the sweep the app measured.
        let (library, mut first) = (entry.id == MACHINES[0].id, Vec::new());
        for (k, points) in sweeps.iter().enumerate() {
            let expected = if library {
                sim_components::analysis::frequency_sweep(entry, &entry.config(), k, &frequencies, 1.5).unwrap()
            } else {
                points.clone()
            };
            if k == 0 {
                first = expected.clone();
            }
            assert!(
                points.len() == 30
                    && points
                        .iter()
                        .zip(&expected)
                        .all(|(a, b)| (a.frequency, a.gain, a.phase_deg) == (b.frequency, b.gain, b.phase_deg)),
                "{}: motor {k}",
                entry.id
            );
            match sim_components::identify::resonance(&expected) {
                Some(r) => assert!(
                    shows(&row(&table, "Resonance [Hz]")[k], r.frequency)
                        && shows(&row(&table, "Damping ratio")[k], r.damping),
                    "{}: {table:?}",
                    entry.id
                ),
                None => assert!(row(&table, "Resonance [Hz]")[k].is_empty()),
            }
        }
        // The first motor of both belt machines has its dip in this range (59.9 Hz and 84.5 Hz); the delta's gearboxes
        // put no dip there, and the rest is about one.
        let Some(rising) = sim_components::identify::resonance(&first) else {
            assert!(
                row(&table, "Resonance [Hz]")[0].is_empty()
                    && (entry.build)(&entry.config()).unwrap().belt_sections().is_empty(),
                "{}",
                entry.id
            );
            continue;
        };
        assert!(number(&row(&table, "Resonance [Hz]")[0]) > 55.0, "{}: {table:?}", entry.id);
        // The range typed from high to low gives the same dip. Its damping came out negative.
        (model.inputs.sweep_from, model.inputs.sweep_to) = (130.0, 50.0);
        model.run_analysis(Analysis::Sweep);
        wait_for_analysis(&mut model);
        let falling = model.sweep_table().unwrap();
        assert!(
            shows(&row(&falling, "Resonance [Hz]")[0], rising.frequency)
                && shows(&row(&falling, "Damping ratio")[0], rising.damping),
            "{}: {falling:?}",
            entry.id
        );
        model.set_language("de");
        assert_eq!(model.sweep_table().unwrap()[1][0], "Resonanz [Hz]");
        let stiffness = if model.config.get("belt.stiffness_ea_n").is_ok() {
            "belt.stiffness_ea_n"
        } else {
            "gearbox.stiffness_nm_rad"
        };
        model.set_param(stiffness, toml::Value::Float(4.0e5));
        assert!(model.outdated(Analysis::Sweep));

        model.inputs.sweep_from = 0.0;
        model.run_analysis(Analysis::Sweep);
        wait_for_jobs(&mut model);
        assert_eq!(model.messages.last().unwrap(), &("error-config-invalid", "frequency".to_string()));
        model.clear(Analysis::Sweep);
        assert!(model.sweep_table().is_none());
    }
}

/// Stage 2 of the fit in the app, as `sim-cli fit --sweep` gives it (`identify::fit_sweep`, which `cli.rs` checks
/// against the command line's output). The measured response is that of the default machine, whose belt has
/// EA = 500 kN (the linear delta's 600 kN); the app's config says 0.8 of it, 400 kN (480 kN). By hand the stiffness is
/// off by 1 / 0.8 = 1.25, and the fit says so within its standard error, half a step of the sweep at the dip's flanks.
/// The sweep has 30 points from 50 to 130 Hz, 3.4 % apart, and 40 (2.5 %) for a broad dip (damping above 0.3; the
/// gantries' is 0.08): the linear delta's (damping 0.35, flanks near 62 and 123 Hz) came to a standard error of
/// 2 * 1.270 * 0.5 * 0.0335 * 123 / 87.6 = 0.060 on 30 points (above the 0.05 asked), 0.044 (1.268) on 40.
/// A file without a dip, a missing one and one that is no sweep are refused.
#[test]
fn the_belt_stiffness_is_fitted_to_a_measured_response_in_the_app() {
    for entry in MACHINES.iter().copied() {
        // A machine with a belt.
        if (entry.build)(&entry.config()).unwrap().belt_sections().is_empty() {
            continue;
        }
        let sweep = |count| {
            sim_components::analysis::frequency_sweep(
                entry,
                &entry.config(),
                0,
                &sim_components::analysis::log_spaced(50.0, 130.0, count),
                1.5,
            )
            .unwrap()
        };
        let mut points = sweep(30);
        if sim_components::identify::resonance(&points).is_some_and(|dip| dip.damping > 0.3) {
            points = sweep(40);
        }
        let file = temp_file(entry, "sweep.csv");
        let text = sweep_csv(&points);
        std::fs::write(&file, &text).unwrap();

        let mut model = Model::new(entry, "en");
        assert!(model.sweep_fit_table().is_none() && model.sweep_fit_advice().is_none());
        model.set_param(
            "belt.stiffness_ea_n",
            toml::Value::Float(0.8 * entry.config().f64("belt.stiffness_ea_n").unwrap()),
        );
        (model.inputs.sweep_file, model.inputs.sweep_amplitude, model.inputs.sweep_motor) = (file.clone(), 1.5, 0);
        model.run_analysis(Analysis::SweepFit);
        wait_for_analysis(&mut model);
        let measured = sim_components::identify::parse_sweep(&text).unwrap();
        let fit = sim_components::identify::fit_sweep(entry, &model.config, 0, &measured, 1.5).unwrap().unwrap();
        let table = model.sweep_fit_table().unwrap();
        assert_eq!(table[0], ["Quantity", "Configuration", "Fitted", "Standard error"]);
        let motor = model.tr.tr(model.session.motors[0].key).to_string();
        let resonance = row(&table, &format!("{motor} · Resonance [Hz]"));
        assert!(
            shows(&resonance[0], fit.config.frequency)
                && shows(&resonance[1], fit.fitted.frequency)
                && shows(&resonance[2], fit.fitted.frequency_error),
            "{table:?}"
        );
        let damping = row(&table, &format!("{motor} · Damping ratio"));
        assert!(
            shows(&damping[0], fit.config.damping) && shows(&damping[1], fit.fitted.damping) && damping[2].is_empty(),
            "{table:?}"
        );
        let (factor, error) = fit.stiffness();
        assert!((factor - 1.25).abs() < error && error < 0.05, "{}: x {factor} +- {error}", entry.id);
        let advice = model.sweep_fit_advice().unwrap();
        assert!(
            advice == model.tr.tr_arg("fit-stiffness", &format!("× {factor:.3} ± {error:.3}"))
                && advice.starts_with("Stiffness between motor and load: × 1.2"),
            "{advice}"
        );
        model.set_language("de");
        assert!(model.sweep_fit_advice().unwrap().starts_with("Steifigkeit zwischen Motor und Last: × 1.2"));

        let refused = |model: &mut Model, file: String| {
            model.inputs.sweep_file = file;
            model.run_analysis(Analysis::SweepFit);
            wait_for_jobs(model);
            model.messages.last().unwrap().0
        };
        // The same response from 90 Hz on has no lower flank to its dip.
        let other = temp_file(entry, "sweep-without-dip.csv");
        std::fs::write(&other, sweep_csv(&measured.iter().copied().filter(|p| p.frequency > 90.0).collect::<Vec<_>>()))
            .unwrap();
        assert_eq!(refused(&mut model, other.clone()), "fit-no-resonance");
        assert_eq!(refused(&mut model, temp_file(entry, "no-such-sweep.csv")), "ui-file-unreadable");
        std::fs::write(&other, "line,0.5\n").unwrap();
        assert_eq!(refused(&mut model, other), "error-trace-column");
        // The result that was there stays.
        assert!(model.sweep_fit_table().is_some());
    }
}

/// The effect views on every machine, against the analysis and the machine they come from. Live: per motor the
/// four parts of its torque and the torque itself over the scopes' window, the parts adding up to the torque at
/// every point. Over the analysed cycle: the peak and RMS of each part as `analyse_cycle` has them, and the energy
/// budget row by row, with what is not accounted for below `BUDGET_TOLERANCE` of the supply. Per friction contact:
/// the curve is the machine's `friction_curve` at the guide load of the newest sample, over the axis's speed limit
/// both ways, and the point is the newest speed and friction force, which the three parts add up to.
#[test]
fn the_effect_views_show_torque_parts_energy_budget_and_friction() {
    use sim_components::analysis::{BUDGET_TOLERANCE, analyse_cycle};
    on_every_machine(|entry| {
        let mut model = Model::new(entry, "en");
        assert!(model.breakdown_table().is_none() && model.budget_table().is_none());
        model.command(Command::StartCycle);
        simulate(&mut model, 1.0);
        let lines = model.breakdown_lines();
        assert_eq!(lines.len(), model.session.motors.len());
        for ((title, parts), motor) in lines.iter().zip(&model.session.motors) {
            assert_eq!(title, &format!("{} [N m]", model.tr.tr(motor.key)));
            assert_eq!(
                parts.iter().map(|p| p.0.as_str()).collect::<Vec<_>>(),
                [
                    "Torque to inertia",
                    "Torque to gravity",
                    "Torque to friction",
                    "Torque to belt and the rest",
                    "Torque"
                ]
            );
            let torque = &parts[4].1;
            assert!(torque.len() > 500, "{}: {} points", entry.id, torque.len());
            for (j, [time, total]) in torque.iter().enumerate() {
                let sum: f64 = parts[..4].iter().map(|p| p.1[j][1]).sum();
                assert!(
                    parts[..4].iter().all(|p| p.1[j][0] == *time) && (sum - total).abs() < 1e-9 * total.abs().max(1.0),
                    "{} at {time} s: {sum} against {total}",
                    entry.id
                );
            }
            assert_eq!(torque.last().unwrap()[1], latest(&model, &format!("motor.{}.torque", motor.id)));
        }

        // The friction view, from the same newest sample.
        let machine = (entry.build)(&entry.config()).unwrap();
        let views = model.friction_views();
        assert_eq!(views.len(), machine.friction_contacts().len());
        // Every belt machine has guides; the delta offers no contact to this view (its joints' friction is a torque).
        assert!(!views.is_empty() || machine.belt_sections().is_empty());
        for (i, (view, contact)) in views.iter().zip(machine.friction_contacts()).enumerate() {
            let get = |part: &str| latest(&model, &format!("{}.friction_{part}", contact.id));
            let scale = get("load_scale");
            // A guide on no axis of its own (a delta's carriage) spans the fastest axis's limit.
            let fastest = model.session.axes.iter().map(|a| a.max_vel).fold(0.0, f64::max);
            let top = model.session.axes.iter().find(|a| a.key == contact.owner_key).map_or(fastest, |a| a.max_vel);
            assert_eq!(view.title, model.tr.tr(contact.owner_key));
            let curve = &view.curves[0].1;
            assert!(
                curve.len() == 200
                    && curve[0][0] == -top
                    && curve[199][0] == top
                    && curve
                        .windows(2)
                        .all(|w| w[0][0] < w[1][0] && w[0][0] * w[1][0] > 0.0 || w[0][0] < 0.0 && w[1][0] > 0.0)
            );
            for (k, [v, force]) in curve.iter().enumerate() {
                let parts = machine.friction_curve(i, *v, scale);
                assert!(
                    (force - parts.iter().sum::<f64>()).abs() < 1e-12
                        && (0..3).all(|p| view.curves[p + 1].1[k][1] == parts[p]),
                    "{} at {v} m/s",
                    entry.id
                );
            }
            let split = ["coulomb", "viscous", "stiction"].map(get);
            // The parts add up to the total, where the machine reports one (a guide's; a joint's friction has its parts alone).
            let total = format!("{}.friction", contact.id);
            assert!(
                model.session.signals.iter().all(|s| s.id != total)
                    || (split.iter().sum::<f64>() - latest(&model, &total)).abs() < 1e-9
            );
            assert_eq!(view.now, Some([get("speed"), split.iter().sum::<f64>()]));
            assert!(
                shows(&row(&view.table, "Coulomb friction [N]")[0], split[0])
                    && shows(&row(&view.table, "Friction force [N]")[0], split.iter().sum()),
                "{:?}",
                view.table
            );
        }

        // Over the analysed cycle.
        model.run_analysis(Analysis::Cycle);
        wait_for_analysis(&mut model);
        let load = analyse_cycle(&mut sim_components::Sim::new(entry, &entry.config()).unwrap()).unwrap();
        let table = model.breakdown_table().unwrap();
        let motor = |k: usize| model.tr.tr(model.session.motors[k].key).to_string();
        assert_eq!(table[0][..3], ["Part".to_string(), format!("{} · peak", motor(0)), format!("{} · RMS", motor(0))]);
        assert_eq!(table.len(), 6);
        for (k, m) in load.motors.iter().enumerate() {
            let parts = m.torque_parts.unwrap();
            for (i, (peak, rms)) in parts.iter().chain([&(m.peak_torque, m.rms_torque)]).enumerate() {
                assert!(
                    shows(&table[1 + i][1 + 2 * k], *peak) && shows(&table[1 + i][2 + 2 * k], *rms),
                    "{}: {:?} against {peak}, {rms}",
                    entry.id,
                    table[1 + i]
                );
            }
        }
        let budget = load.budget.unwrap();
        let shown = model.budget_table().unwrap();
        assert_eq!(shown[0], ["Where it went", "Energy [J]", "Share of the supply [%]"]);
        assert!(
            shown[1][0] == "Drawn from the supply" && shows(&shown[1][1], budget.supply) && shown[1][2] == "100.0",
            "{:?}",
            shown[1]
        );
        let rows: Vec<_> = budget.rows.iter().filter(|r| r.energy != 0.0).collect();
        assert_eq!(shown.len(), 3 + rows.len());
        for (cells, r) in shown[2..].iter().zip(rows) {
            assert!(
                cells[0] == format!("{} · {}", model.tr.tr(&r.owner_key), model.tr.tr(&r.quantity_key))
                    && shows(&cells[1], r.energy),
                "{cells:?} against {r:?}"
            );
        }
        let last = shown.last().unwrap();
        assert!(
            last[0] == "Not accounted for"
                && shows(&last[1], budget.unaccounted())
                && budget.unaccounted().abs() < BUDGET_TOLERANCE * budget.supply,
            "{last:?}"
        );
        assert!(
            rows_named(&shown, "Copper loss") == model.session.motors.len() && rows_named(&shown, "Friction loss") >= 1,
            "{shown:?}"
        );
        // The flow diagram: the first column (the supply and what stores gave back) adds up to the second, the shaft
        // work in the second's last bar passes on to the third, and the bars of the second and third, less what the
        // first has beyond the supply, are the supply, as the budget's rows and what it leaves unaccounted are. One
        // bar per row that is not zero, and one per side for what that side leaves unaccounted.
        let [first, second, third] = model.budget_flows().unwrap();
        let sum = |bars: &[(String, f64)]| bars.iter().map(|(_, e)| e).sum::<f64>();
        let close = |a: f64, b: f64| (a - b).abs() < 1e-9 * budget.supply;
        assert_eq!(first[0], ("Drawn from the supply".to_string(), budget.supply));
        assert_eq!(second.last().unwrap().0, "Shaft work");
        assert!(
            close(sum(&first), sum(&second)) && close(second.last().unwrap().1, sum(&third)),
            "{}: {first:?} {second:?} {third:?}",
            entry.id
        );
        assert!(
            close(sum(&second[..second.len() - 1]) + sum(&third) - (sum(&first) - budget.supply), budget.supply),
            "{}",
            entry.id
        );
        let electrical_rest = budget.electrical_unaccounted();
        let rests = [electrical_rest, budget.unaccounted() - electrical_rest].iter().filter(|e| **e != 0.0).count();
        let nonzero = budget.rows.iter().filter(|r| r.energy != 0.0).count();
        assert!(
            first.len() + second.len() + third.len() == 2 + nonzero + rests
                && [&first, &second, &third].iter().all(|c| c.iter().all(|(_, e)| *e > 0.0)),
            "{}",
            entry.id
        );

        model.set_language("de");
        assert_eq!(model.breakdown_table().unwrap()[0][0], "Anteil");
        assert_eq!(model.budget_table().unwrap()[1][0], "Aus der Einspeisung bezogen");
        assert_eq!(model.budget_flows().unwrap()[1].last().unwrap().0, "Wellenarbeit");
        assert_eq!(model.breakdown_lines()[0].1[0].0, "Moment für die Trägheit");
        if !views.is_empty() {
            assert_eq!(model.friction_views()[0].curves[0].0, "Reibkraft");
        }
        model.clear_analysis();
        assert!(model.breakdown_table().is_none() && model.budget_table().is_none());
    });
}

/// The workspace map of the delta, a slice at z = -1.0 m, ends where the hand-calculated reach ends. Along the row
/// through the middle (y = 0), away from arm 1 (towards -x), the reach ends where arm 1's forearm stands in line with
/// its upper arm: the platform's centre la + lb = 0.4 + 0.95 = 1.35 m from the arm's shoulder, which in the legs'
/// frame (the platform's radius taken off the base's) is at x = e = 0.2 - 0.05 = 0.15 m, so x = 0.15 - sqrt(1.35^2 -
/// 1.0^2) = 0.15 - 0.9069 = -0.7569 m. The other two arms reach there: in arm 2's frame that point is at x' = 0.378,
/// y' = 0.656, whose distance to its elbow circle (radius 0.4 about x' = 0.15) is at most 0.906 m, within its
/// forearm's 0.95 m. The grid spans the travel range of +-0.6 m and half its width beyond each end, +-1.2 m, in steps
/// of 2.4 / 240 = 0.01 m: the first node reached is x = -0.75 m, the 45th. The working envelope's bottom sphere (radius
/// (0.6^2 + 0.186^2) / (2 * 0.186) = 1.0607 m about z = -0.675 - 0.386 + 1.0607 = -0.0003 m) is 0.3545 m wide at
/// z = -1.0: the machine goes to x = 0.3 m but not to x = 0.5 m, which its arms reach (at 7.3 and 66.4 degrees). The
/// delta picker is the first machine registered with a map; machines that move their tool as their axes have none.
#[test]
fn the_workspace_map_ends_at_the_hand_calculated_reach_of_the_delta() {
    assert!(MACHINES.iter().any(|e| Model::new(e, "en").workspace_slice(5).is_none()));
    let entry = MACHINES.iter().copied().find(|e| Model::new(e, "en").workspace_slice(5).is_some()).unwrap();
    let mut model = Model::new(entry, "en");
    // The middle of the z range by default: -0.675 - 0.386 / 2.
    assert!((model.workspace_slice(5).unwrap().z + 0.868).abs() < 1e-12);
    model.inputs.workspace_z = Some(-1.0);
    let slice = model.workspace_slice(241).unwrap();
    assert_eq!((slice.x, slice.y, slice.z, slice.limit), ((-1.2, 1.2), (-1.2, 1.2), -1.0, 5.0));
    assert!((slice.heights.0 + 1.061).abs() < 1e-12 && slice.heights.1 == -0.675, "{:?}", slice.heights);
    let middle = &slice.nodes[120 * 241..121 * 241];
    let first = middle.iter().position(|(condition, _)| condition.is_finite()).unwrap();
    let reach = 0.15 - (1.35f64.powi(2) - 1.0).sqrt();
    let x = |i: usize| -1.2 + 0.01 * i as f64;
    assert!(first == 45 && x(first) >= reach && x(first - 1) < reach, "{first}");
    // Beyond arm 1's stop at 95 degrees there (124.7), so reached but not gone to.
    assert!(!middle[first].1);
    // The centre: gone to, the condition the signal kinematics.condition has there.
    let machine = (entry.build)(&entry.config()).unwrap();
    assert!(
        middle[120].1
            && middle[120].0 == machine.condition(&[0.0, 0.0, -1.0]).unwrap()
            && middle[120].0 >= 1.0
            && middle[120].0 < 3.6,
        "{:?}",
        middle[120]
    );
    assert!(middle[150].1 && middle[170].0.is_finite() && !middle[170].1, "{:?} {:?}", middle[150], middle[170]);
    // A height beyond the z range is taken at its end.
    model.inputs.workspace_z = Some(0.0);
    assert_eq!(model.workspace_slice(5).unwrap().z, -0.675);
}

/// How many rows of a table have a label that ends with `quantity`.
fn rows_named(table: &[Vec<String>], quantity: &str) -> usize {
    table.iter().filter(|row| row[0].ends_with(quantity)).count()
}

/// The thermal, belt and comparison views show what the simulation has, in its own numbers; the 3D view's colours
/// follow the heat and tension scales the views draw as their legends.
#[test]
fn the_thermal_belt_and_comparison_views_show_the_simulation() {
    use sim_app::model::{heat_color, part_color};
    use toml::Value;
    // Belt: its own colour at pretension, red at twice it, blue at none. Heat: blue cold, yellow at half the rated
    // rise, red at it, magenta beyond; it takes the place of the part's own colour.
    let close = |a: [f32; 3], b: [f32; 3]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-6);
    let own = [0.1, 0.2, 0.3];
    assert!(
        close(part_color(own, 0.0, None), own)
            && close(part_color(own, 1.0, None), [1.0, 0.1, 0.1])
            && close(part_color(own, -1.0, None), [0.1, 0.3, 1.0])
    );
    for (heat, color) in
        [(0.0, [0.15, 0.3, 0.75]), (0.5, [0.95, 0.8, 0.1]), (1.0, [0.95, 0.1, 0.05]), (1.5, [0.9, 0.2, 0.9])]
    {
        assert!(close(heat_color(heat), color), "{heat}: {:?}", heat_color(heat));
    }
    assert_eq!(part_color(own, 0.7, Some(0.5)), heat_color(0.5));
    for entry in MACHINES.iter().copied() {
        let mut model = Model::new(entry, "en");
        // Without heating the view says that nothing warms.
        model.set_param("effects.winding_heating", Value::Boolean(false));
        assert!(model.thermal_view().off);
        // The temperature feedback warms the parts too, so they are shown with it alone.
        model.set_param("effects.thermal_feedback", Value::Boolean(true));
        let view = model.thermal_view();
        let parts = (entry.build)(&model.config).unwrap().thermal_parts().len();
        assert!(
            !view.off && view.lines.len() == model.session.motors.len() + parts,
            "{}: {} lines",
            entry.id,
            view.lines.len()
        );
        model.set_param("effects.thermal_feedback", Value::Boolean(false));
        model.set_param("effects.winding_heating", Value::Boolean(true));
        model.set_param("effects.part_heating", Value::Boolean(true));
        if model.config.get("thermal.guide_time_constant_s").is_ok() {
            model.set_param("thermal.guide_time_constant_s", Value::Float(0.5));
        }
        model.command(Command::StartCycle);
        simulate(&mut model, 1.5);
        let machine = (entry.build)(&model.config).unwrap();
        let parts = machine.thermal_parts();
        let view = model.thermal_view();
        assert!(
            !view.off
                && view.lines.len() == model.session.motors.len() + parts.len()
                && view.table.len() == 1 + view.lines.len(),
            "{}: {:?}",
            entry.id,
            view.table
        );
        assert_eq!(
            view.table[0],
            ["Part", "Now [K]", "Settles at on the cycle [K]", "Rated rise [K]", "Settled, of the rating [%]"]
        );
        for (k, motor) in model.session.motors.iter().enumerate() {
            let id = format!("motor.{}.winding_temperature", motor.id);
            let cells = row(&view.table, &format!("{} · Winding temperature rise", model.tr.tr(motor.key)));
            assert!(
                shows(&cells[0], latest(&model, &id))
                    && cells[2] == "100.0"
                    && view.lines[k].1.last().unwrap()[1] == latest(&model, &id),
                "{cells:?}"
            );
        }
        for part in &parts {
            let id = format!("temperature.{}", part.id);
            let rise = latest(&model, &id);
            let cells = row(
                &view.table,
                &format!(
                    "{} · {}",
                    model.tr.tr(part.owner_key),
                    model.tr.tr(&format!("quantity-{}-temperature", part.kind))
                ),
            );
            assert!(
                shows(&cells[0], rise)
                    && shows(&cells[2], model.config.f64(&format!("thermal.{}_max_rise_k", part.kind)).unwrap()),
                "{cells:?}"
            );
            if part.kind == "guide" {
                assert!(rise > 0.0, "{}: the guide {} has not warmed", entry.id, part.id);
            }
        }

        // The belt sections: tension and stretch now, against pretension and rating (where there is a belt).
        let sections = machine.belt_sections();
        let belted = !sections.is_empty();
        if belted {
            let belt = model.belt_view().unwrap();
            // The config's own pretension and rating: the belt machines differ in the rating (1500 N, 1000 N).
            let (pretension, rating) = (
                model.config.f64("belt.pretension_n").unwrap(),
                model.config.f64("ratings.belt_max_tension_n").unwrap(),
            );
            assert!(
                belt.tension.len() == sections.len()
                    && belt.stretch.len() == sections.len()
                    && belt.pretension == pretension
                    && belt.rating == Some(rating),
                "{}",
                entry.id
            );
            assert_eq!(
                (belt.titles.clone(), belt.scale.clone()),
                (
                    ["Tension [N]".to_string(), "Stretch [mm]".into()],
                    ["slack 0 N".to_string(), format!("twice the pretension {:.0} N", 2.0 * pretension)]
                )
            );
            assert_eq!(
                belt.table[0],
                ["Section", "Tension [N]", "Pretension [N]", "Allowed [N]", "Of the rating [%]", "Stretch [mm]"]
            );
            for (i, section) in sections.iter().enumerate() {
                let (tension, stretch) = (
                    latest(&model, &format!("{}.tension", section.id)),
                    latest(&model, &format!("{}.stretch", section.id)),
                );
                let cells = row(&belt.table, model.tr.tr(section.owner_key));
                assert!(
                    shows(&cells[0], tension)
                        && shows(&cells[1], pretension)
                        && shows(&cells[3], 100.0 * tension / rating)
                        && shows(&cells[4], stretch * 1e3),
                    "{cells:?}"
                );
                assert!(
                    belt.tension[i].1.last().unwrap()[1] == tension
                        && belt.stretch[i].1.last().unwrap()[1] == stretch * 1e3
                );
            }
        }
        // The friction the machine has: of its guides, or of its arms' joints.
        let effect =
            if model.config.get("effects.guide_friction").is_ok() { "guide_friction" } else { "joint_friction" };

        // Over the analysed cycle: where windings and parts settle.
        model.run_analysis(Analysis::Cycle);
        model.inputs.compare_effect = effect.into();
        model.run_analysis(Analysis::Compare);
        wait_for_analysis(&mut model);
        let view = model.thermal_view();
        let (load, ..) = model.cycle_report.as_ref().unwrap();
        for (k, motor) in model.session.motors.iter().enumerate() {
            assert!(shows(
                &row(&view.table, &format!("{} · Winding temperature rise", model.tr.tr(motor.key)))[1],
                load.motors[k].winding_rise
            ));
        }
        let budget = load.budget.as_ref().unwrap();
        for part in &parts {
            let lost =
                budget.rows.iter().find(|r| r.id == format!("power.{}", part.id)).unwrap().energy / load.duration;
            let settles = lost * model.config.f64(&format!("thermal.{}_resistance_k_w", part.kind)).unwrap();
            let cells = row(
                &view.table,
                &format!(
                    "{} · {}",
                    model.tr.tr(part.owner_key),
                    model.tr.tr(&format!("quantity-{}-temperature", part.kind))
                ),
            );
            assert!(shows(&cells[1], settles), "{}: {cells:?} against {settles} K", entry.id);
        }

        // The comparison: one line per motor for the torque, on and off, and the difference of each.
        let compare = model.compare_view().unwrap();
        let (compared, label) = (effect, model.tr.tr(&format!("effect-{effect}")).to_string());
        let (effect, found) = model.comparison.as_ref().unwrap();
        assert!(
            effect == compared
                && compare.effect == label
                && (compared != "guide_friction" || label == "Guide friction")
                && compare.title == "Torque [N m]",
            "{} {}",
            compare.effect,
            compare.title
        );
        assert_eq!(compare.lines.len(), model.session.motors.len());
        assert_eq!(compare.table[0], ["Signal", "Peak of on − off [N m]", "RMS of on − off [N m]"]);
        for ((name, on, off), motor) in compare.lines.iter().zip(&model.session.motors) {
            let i = found.signals.iter().position(|s| s.id == format!("motor.{}.torque", motor.id)).unwrap();
            let (peak, rms) = found.difference(i);
            let cells = row(&compare.table, name);
            assert!(shows(&cells[0], peak) && shows(&cells[1], rms) && peak > 0.0 && on != off, "{cells:?}");
        }
        model.set_language("de");
        assert_eq!(model.thermal_view().table[0][0], "Bauteil");
        if belted {
            assert_eq!(model.belt_view().unwrap().table[0][0], "Abschnitt");
            assert_eq!(model.compare_view().unwrap().effect, "Führungsreibung");
        }
        model.clear(Analysis::Compare);
        assert!(model.compare_view().is_none());
    }
}

/// The thermal table's share of the rating is that of the rise where a part settles on the analysed cycle, not of
/// the rise now (minutes into a climb of many minutes it is a few percent), and it is marked above 100 %; the checks
/// table judges the same settled rise. A guide of 20 K/W settles far above its rating.
#[test]
fn the_thermal_share_is_that_of_the_settled_rise_and_marked_above_the_rating() {
    use toml::Value;
    for entry in MACHINES.iter().copied() {
        // A machine with a guide that warms (the delta's arms have none).
        if !(entry.build)(&entry.config()).unwrap().thermal_parts().iter().any(|part| part.id == "guide_x") {
            continue;
        }
        let mut model = Model::new(entry, "en");
        model.set_param("effects.part_heating", Value::Boolean(true));
        model.set_param("thermal.guide_resistance_k_w", Value::Float(20.0));
        model.command(Command::StartCycle);
        simulate(&mut model, 0.5);
        model.run_analysis(Analysis::Cycle);
        wait_for_analysis(&mut model);
        let view = model.thermal_view();
        let (load, ..) = model.cycle_report.as_ref().unwrap();
        let rated = model.config.f64("thermal.guide_max_rise_k").unwrap();
        let (_, settles) = load.part_rises.iter().find(|(part, _)| part.id == "guide_x").unwrap();
        let owner = (entry.build)(&model.config)
            .unwrap()
            .thermal_parts()
            .into_iter()
            .find(|part| part.id == "guide_x")
            .unwrap()
            .owner_key;
        let cells = row(&view.table, &format!("{} · Guide temperature rise", model.tr.tr(owner)));
        let share = 100.0 * settles / rated;
        assert!(share > 100.0 && number(&cells[0]) < 0.1 * settles, "{}: {cells:?}", entry.id);
        assert!(
            shows(&cells[1], *settles)
                && cells[3] == format!("{} (above the rating)", cells[3].split(' ').next().unwrap())
                && shows(cells[3].split(' ').next().unwrap(), share),
            "{}: {cells:?} against {share} %",
            entry.id
        );
        // The checks table: the hottest guide where it settles, against its rated rise.
        let checks = &model.cycle_tables()[2];
        let check = row(checks, "Guide temperature rise, settled [K]");
        let hottest =
            load.part_rises.iter().filter(|(part, _)| part.kind == "guide").map(|(_, rise)| *rise).fold(0.0, f64::max);
        assert!(shows(&check[1], hottest) && shows(&check[2], rated) && check[3] == "FAIL", "{}: {check:?}", entry.id);
    }
}

/// A drive, a belt and an axis typed by name give the config of `--drive`, `--belt` and `--axis` on the command line;
/// and the search over every catalogue axis in the app is `search_axes`, its button naming the axis and the drive the
/// motor ran with as well.
#[test]
fn drives_belts_and_axes_are_named_in_the_app_and_searched_over() {
    use sim_components::catalog::{resolve, search_axes};
    // The machine with a Z stroke, which the X/Z gantries of the catalogue fit.
    let entry = MACHINES.iter().copied().find(|e| e.config().get("geometry.stroke_z_m").is_ok()).unwrap();
    let mut typed = Model::new(entry, "en");
    typed.inputs.parts = ["", "400v", "", "6SL3120-1TE15-0AD0", "MTD5-25", "EXCT-30"].map(String::from);
    typed.apply_parts();
    assert!(typed.messages.iter().all(|m| m.0.starts_with("limit-")), "{:?}", typed.messages);
    let mut cli = entry.config();
    for (key, name) in [("drive", "6SL3120-1TE15-0AD0"), ("belt", "MTD5-25"), ("axis", "EXCT-30")] {
        cli.set_text(&format!("parts.{key} = {name:?}")).unwrap();
    }
    let resolved = resolve(entry, &typed.config).unwrap();
    assert!(resolved == resolve(entry, &cli).unwrap());
    // The belt named after the axis is the one in the config: MTD5 at 25 mm, 1805 N.
    assert_eq!(
        (resolved.f64("ratings.belt_max_tension_n").unwrap(), resolved.f64("ratings.drive_max_current_a").unwrap()),
        (1805.0, 15.0)
    );

    let mut model = Model::new(entry, "en");
    (model.inputs.series, model.inputs.axes) = ("1FK2205-4AF".into(), true);
    model.run_analysis(Analysis::Search);
    wait_for_analysis(&mut model);
    let found = search_axes(entry, &entry.config(), "400v", &|article| article == "1FK2205-4AF", None);
    let table = model.search_table().unwrap();
    assert!(found.len() >= 4 && table.len() == 1 + found.len(), "{table:?}");
    for (row, c) in table[1..].iter().zip(&found) {
        assert_eq!((row[0].as_str(), row[2].as_str()), (c.article, c.axis.as_str()));
        // An axis ruled out has no motor, no masses and its reason in words.
        assert!(!c.article.is_empty() || (row[3].is_empty() && row[6].contains(' ')), "{row:?}");
    }
    let row = found.iter().position(|c| !c.article.is_empty()).unwrap();
    model.use_candidate(row);
    let part = |key: &str| model.config.get(&format!("parts.{key}")).unwrap().as_str().unwrap().to_string();
    assert_eq!(
        (part("motor"), part("axis"), part("drive")),
        (found[row].article.to_string(), found[row].axis.clone(), found[row].drive.to_string())
    );
}

/// A config file of another machine than the one shown switches to that machine (the screenshot hook's SIM_APP_CONFIG
/// could not open a config of any machine but the first, and SIM_APP_MACHINE now names one); a file no machine takes is
/// answered with the first machine's reason, and the model stays as it was.
#[test]
fn a_config_of_another_machine_opens_that_machine() {
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR"));
    for other in MACHINES.iter().copied().skip(1) {
        let text = other.default_config;
        if sim_components::load_config(MACHINES[0], text).is_ok() {
            continue;
        }
        let path = dir.join(format!("other-machine-{}.toml", other.id));
        std::fs::write(&path, text).unwrap();
        let mut model = Model::new(MACHINES[0], "en");
        model.open_config(path.to_str().unwrap());
        assert_eq!(model.entry.id, other.id, "{}", other.id);
        assert_eq!(model.inputs.config_file, path.to_str().unwrap());
        let bad = dir.join("no-machine.toml");
        std::fs::write(&bad, format!("{text}\n[nonsense]\nkey = 1.0\n")).unwrap();
        let mut model = Model::new(MACHINES[0], "en");
        model.open_config(bad.to_str().unwrap());
        assert_eq!(model.entry.id, MACHINES[0].id);
    }
}

/// A machine file (docs/machine-files.md) opened like a config brings its machine: the H-bot runs in the app, Save
/// writes back over its file, and a broken one is reported with its line while the machine shown stays.
#[test]
fn a_machine_file_opens_with_its_machine() {
    let shipped = concat!(env!("CARGO_MANIFEST_DIR"), "/../../machines/h-bot.toml");
    let file = temp_file(MACHINES[0], "machine-file.toml");
    std::fs::copy(shipped, &file).unwrap();
    let mut model = Model::new(MACHINES[0], "en");
    model.open_config(&file);
    assert_eq!(model.messages.last().map(|m| m.0), Some("ui-machine-file-opened"), "{:?}", model.messages);
    assert_eq!((model.entry.id, model.inputs.config_file.as_str()), ("h-bot", file.as_str()));
    wait_for_data(&mut model);
    model.save_config(&file);
    assert_eq!(model.messages.last().map(|m| m.0), Some("ui-config-saved"), "{:?}", model.messages);

    let original = std::fs::read_to_string(shipped).unwrap();
    std::fs::write(&file, original.replacen("friction = \"friction_z\"", "fricton = \"friction_z\"", 1)).unwrap();
    let mut model = Model::new(MACHINES[1], "de");
    model.open_config(&file);
    let line = original.lines().position(|l| l == "friction = \"friction_z\"").unwrap() + 1;
    let (key, detail) = model.messages.last().cloned().unwrap();
    assert_eq!((key, model.entry.id), ("error-config-unknown", MACHINES[1].id));
    assert!(detail.ends_with(&format!(":{line}: machine.guide[guide_y].fricton")), "{detail}");
}

/// A vendor-style trace (semicolons, decimal commas, encoder counts, 1/min) through a mapping file in
/// "Replay and fit": the same replay as the simulator's own columns, and a table of what the trace says by
/// itself per motor, the values `Identified` gives on the imported trace.
#[test]
fn a_vendor_trace_through_a_mapping_is_replayed_and_identified_per_motor() {
    // The machine with one motor, by the registry.
    let entry = MACHINES.iter().copied().find(|m| (m.build)(&m.config()).unwrap().motors().len() == 1).unwrap();
    let mut cfg = entry.config();
    cfg.set_text("effects.payload = false").unwrap();
    let mut sim = sim_components::Sim::new(entry, &cfg).unwrap();
    sim.command(Command::StartCycle).unwrap();
    let mut recorded = sim_components::trace::Trace::default();
    for _ in 0..=(3.0 / sim.dt).round() as u64 {
        recorded.record(&sim);
        sim.step();
    }
    let counts = 1048576.0;
    let comma = |v: f64| format!("{v}").replace('.', ",");
    let mut text = String::from(
        "time [ms];position setpoint [inc];position actual [inc];speed actual [1/min];torque actual [Nm]\n",
    );
    let m = &recorded.motors[0];
    for (row, t) in recorded.time.iter().enumerate() {
        let cells = [
            t * 1e3,
            (m.setpoint[row] / std::f64::consts::TAU * counts).round(),
            (m.angle[row] / std::f64::consts::TAU * counts).round(),
            m.speed[row] * 60.0 / std::f64::consts::TAU,
            m.torque[row],
        ];
        text += &(cells.map(comma).join(";") + "\n");
    }
    let (file, map) = (temp_file(entry, "vendor-trace.csv"), temp_file(entry, "vendor-map.toml"));
    std::fs::write(&file, &text).unwrap();
    let mapping = format!(
        "format = \"csv\"\ntime = \"time\"\n[[motor]]\nsetpoint = {{ signal = \"position setpoint\", per_rev = {counts} }}\nangle = {{ signal = \"position actual\", per_rev = {counts} }}\nspeed = \"speed actual\"\ntorque = \"torque actual\"\n"
    );
    std::fs::write(&map, &mapping).unwrap();

    let mut model = Model::new(entry, "de");
    model.set_param("effects.payload", toml::Value::Boolean(false));
    model.inputs.trace_file = file;
    model.inputs.trace_map = map;
    model.run_analysis(Analysis::Trace);
    wait_for_analysis(&mut model);
    let tables = model.trace_tables();
    assert_eq!(tables.len(), 3, "{:?}", model.messages);
    let imported =
        sim_components::import::import(&text, &sim_components::import::Mapping::parse(&mapping).unwrap()).unwrap();
    let expected = sim_components::motor_fit::Identified::of(&imported.trace.time, &imported.trace.motors[0], None);
    let inertia = expected.estimates()[0].1;
    let identified = &tables[2];
    assert_eq!(identified[0][2], "Wert");
    let row = identified.iter().find(|r| r[1] == model.tr.tr("identify-inertia_kg_m2")).expect("an inertia row");
    assert_eq!(
        (row[2].as_str(), row[3].as_str()),
        (sim_app::model::short(inertia.value).as_str(), sim_app::model::short(inertia.error).as_str())
    );
    // Values and qualities, all labelled.
    assert_eq!(identified.len() - 1, expected.estimates().len() + expected.qualities().len());
    assert!(identified[1..].iter().all(|r| !r[1].starts_with("identify-")), "{identified:?}");
}

/// The sizing report is written in the background, as PDF or HTML by the file's name, with an edited value marked as
/// set for this run, and says so when done.
#[test]
fn the_sizing_report_is_written_from_the_config_as_edited() {
    let mut model = Model::new(MACHINES[0], "en");
    model.set_param("mass.payload_kg", toml::Value::Float(2.5));
    let dir = std::path::Path::new(env!("CARGO_TARGET_TMPDIR"));
    for name in ["app-report.html", "app-report.pdf"] {
        let path = dir.join(name);
        let _ = std::fs::remove_file(&path);
        model.export_report(path.to_str().unwrap());
        assert!(model.report_running());
        let start = Instant::now();
        while model.report_running() && start.elapsed() < Duration::from_secs(120) {
            model.poll();
            std::thread::sleep(Duration::from_millis(20));
        }
        assert_eq!(model.messages.last().map(|m| m.0), Some("ui-report-written"), "{:?}", model.messages.last());
        let bytes = std::fs::read(&path).unwrap();
        if name.ends_with(".pdf") {
            assert!(bytes.starts_with(b"%PDF"));
        } else {
            let html = String::from_utf8(bytes).unwrap();
            assert!(
                html.contains("<td>mass.payload_kg</td><td>2.5</td><td>set for this run</td>"),
                "the edit is marked"
            );
        }
    }
}

/// The belt axis that can stand upright, found by its key `geometry.incline_deg` (no test outside a machine's crate names
/// the machine).
fn inclinable_axis() -> &'static MachineEntry {
    MACHINES.iter().copied().find(|e| e.config().get("geometry.incline_deg").is_ok()).unwrap()
}

/// `sim-app --set` at start (`Model::set_at_start`): the inclinable belt axis stood upright, its frictions, gearbox loss
/// and ripple off, holds its moved masses with the motor torque m g r / i. 5 kg more payload set at start needs
/// 5 kg * 9.80665 m/s^2 * 0.02 m / 5 = 0.196133 N m more. The value is in the config, and a key the config does not
/// have stops the start with the CLI's refusal, with nothing applied.
#[test]
fn values_set_at_start_change_the_config_and_the_run() {
    let entry = inclinable_axis();
    let upright = [
        "geometry.incline_deg = 90.0",
        "effects.guide_friction = false",
        "effects.load_dependent_friction = false",
        "effects.motor_friction = false",
        "effects.gearbox_efficiency = false",
        "effects.torque_ripple = false",
        "effects.tooth_meshing = false",
    ];
    let holding = |payload: f64| {
        let mut model = Model::new(entry, "en");
        let mut sets: Vec<String> = upright.map(String::from).to_vec();
        sets.push(format!("mass.payload_kg = {payload:?}"));
        model.set_at_start(&sets).unwrap();
        assert_eq!(model.config.f64("mass.payload_kg").unwrap(), payload);
        wait_for_data(&mut model);
        simulate(&mut model, 0.3);
        latest(&model, "motor.m.torque")
    };
    // (4 kg carriage + 2 kg payload) * 9.80665 * 0.02 / 5 = 0.235360 N m; `sim-cli run` with the same `--set`s gives
    // 0.235361 and, with 7 kg, 0.43129 (some 2e-4 N m of ringing left at 0.3 s).
    let (light, heavy) = (holding(2.0), holding(7.0));
    assert!((light - 0.235360).abs() < 1e-3, "{light} N m for 2 kg");
    assert!((heavy - light - 0.196133).abs() < 2e-3, "{} N m more for 5 kg", heavy - light);
    let mut model = Model::new(entry, "en");
    assert_eq!(
        model.set_at_start(&["mass.payload_kg = 3.0".into(), "mass.nope_kg = 1.0".into()]),
        Err(("error-config-missing", "mass.nope_kg".to_string()))
    );
    assert_eq!(model.config.f64("mass.payload_kg").unwrap(), 2.0);
    assert_eq!(
        model.set_at_start(&["mass.payload_kg = true".into()]),
        Err(("error-config-type", "mass.payload_kg".to_string()))
    );
}

/// The parameter panel's "key = value" field (`Model::set_value`) applies any key the config has and refuses as
/// `sim-cli --set` does: an unknown key, a value of another type and text that is no assignment with the messages of
/// the config's errors, in English and German; a value the simulation refuses (a negative mass) comes back and is
/// rolled back like any edit.
#[test]
fn the_panel_field_sets_any_key_and_refuses_as_the_command_line_does() {
    let mut model = Model::new(inclinable_axis(), "en");
    wait_for_data(&mut model);
    let answered = |model: &mut Model, done: &dyn Fn(&Model) -> bool| {
        let start = Instant::now();
        while !done(model) {
            assert!(start.elapsed() < Duration::from_secs(10), "no answer within 10 s: {:?}", model.messages);
            model.poll();
            std::thread::sleep(Duration::from_millis(5));
        }
    };
    model.set_value("geometry.span_min_m = 0.2");
    assert_eq!(model.config.f64("geometry.span_min_m").unwrap(), 0.2);
    model.set_value("mass.nope_kg = 1.0");
    let shown = |model: &Model| {
        let (key, detail) = model.messages.last().unwrap();
        model.tr.tr_arg(key, detail).to_string()
    };
    assert_eq!(model.messages.last().unwrap(), &("error-config-missing", "mass.nope_kg".to_string()));
    assert_eq!(shown(&model), "Configuration value is missing: mass.nope_kg");
    model.set_value("mass.payload_kg = \"heavy\"");
    assert_eq!(model.messages.last().unwrap(), &("error-config-type", "mass.payload_kg".to_string()));
    model.set_value("mass.payload_kg");
    assert_eq!(model.messages.last().unwrap().0, "error-config-parse");
    model.set_language("de");
    model.set_value("mass.nope_kg = 1.0");
    assert!(
        shown(&model).ends_with(": mass.nope_kg") && shown(&model) != "Configuration value is missing: mass.nope_kg"
    );
    model.set_value("mass.carriage_kg = -1.0");
    answered(&mut model, &|model| model.config.f64("mass.carriage_kg").unwrap() == 4.0);
    assert_eq!(model.messages.last().unwrap().0, "error-config-invalid", "{:?}", model.messages);
}

/// The app plans a path from where its machine's setpoint stands: the planned path's first pose is the live
/// setpoint within 1 um (1 urad on an arm's joints), on every machine. The planning model's tool is put one rest sag
/// below it (0.26 mm on the planar delta) so that its setpoint lands there.
#[test]
fn a_planned_path_starts_at_the_live_setpoint() {
    on_every_machine(|entry| {
        let mut model = Model::new(entry, "en");
        wait_for_data(&mut model);
        simulate(&mut model, 0.2);
        let axes = model.session.axes.clone();
        // The lines of `a_path_is_planned_run_and_analysed_in_the_app`, inside every machine's workspace.
        let point = |share: f64| -> String {
            axes.iter().map(|a| format!("{:?}", a.min + share * (a.max - a.min))).collect::<Vec<_>>().join(",")
        };
        let file = temp_file(entry, "from-setpoint.csv");
        std::fs::write(&file, format!("line,{}\nline,{}\n", point(0.5), point(0.75))).unwrap();
        let live: Vec<f64> = axes.iter().map(|a| latest(&model, &format!("axis.{}.position_setpoint", a.id))).collect();
        model.inputs.path_file = file;
        model.run_analysis(Analysis::Path);
        wait_for_analysis(&mut model);
        let first = &model.path_run.as_ref().unwrap().points[0].1;
        let off = first.iter().zip(&live).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
        assert!(off < 1e-6, "{}: the plan starts {off} off the live setpoint {live:?}: {first:?}", entry.id);
    });
}
