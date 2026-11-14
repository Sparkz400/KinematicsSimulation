//! The browser module's requests, run natively: the same code the page runs once built for wasm32.

use sim_wasm::handle;

/// Every machine of the registry is listed, in the language asked for.
#[test]
fn lists_every_machine() {
    let list = handle(0, "de").unwrap();
    for entry in sim_machines::MACHINES {
        assert!(list.contains(&format!("\"id\":\"{}\"", entry.id)), "{list}");
    }
}

/// The sizing check gives what `sim-cli verdicts` gives: the same checks, values and results.
#[test]
fn sizing_matches_the_command_line_verdicts() {
    let entry = sim_machines::MACHINES[0];
    let json = handle(2, &format!("{}\nen\n", entry.id)).unwrap();
    let cfg = entry.config();
    let mut sim = sim_components::Sim::new(entry, &cfg).unwrap();
    let load = sim_components::analysis::analyse_cycle_with(&mut sim, false).unwrap();
    let judged = sim_components::analysis::verdicts(&cfg, &load).unwrap();
    assert!(!judged.is_empty());
    for v in &judged {
        let row = format!("\"check\":\"{}\"", v.check);
        assert!(json.contains(&row), "{row} missing in {json}");
        assert!(json.contains(&format!("\"value\":{:?}", v.value)), "{json}");
    }
    assert!(json.contains(&format!("\"cycle_s\":{:?}", load.duration)), "{json}");
    // Labels come from the translations, not the check ids.
    assert!(json.contains("Winding rise"), "{json}");
}

/// A config the page sends is read as a config file: a changed value counts, a misspelt key is refused by name.
#[test]
fn a_config_from_the_page_is_checked_and_used() {
    let entry = sim_machines::MACHINES[0];
    let base = handle(2, &format!("{}\nen\n", entry.id)).unwrap();
    let mut cfg = entry.config();
    let scale = cfg.f64("sim.dt_s").unwrap();
    cfg.set_text(&format!("sim.dt_s = {scale:?}")).unwrap();
    assert_eq!(handle(2, &format!("{}\nen\n{}", entry.id, cfg.to_text().unwrap())).unwrap(), base);
    let error = handle(2, &format!("{}\nen\n{}\n[no_such_table]\nno_such_key = 1\n", entry.id, entry.default_config))
        .unwrap_err();
    assert!(error.contains("no_such"), "{error}");
    assert!(handle(2, "no-such-machine\nen\n").unwrap_err().contains("no-such-machine"));
    assert!(handle(1, entry.id).unwrap().contains("[sim]"));
}

/// The C functions as the page calls them: a request in memory from `alloc`, the answer at `out_ptr`.
#[test]
fn the_c_functions_hand_back_json() {
    let request = b"en";
    let ptr = sim_wasm::alloc(request.len());
    unsafe { std::ptr::copy_nonoverlapping(request.as_ptr(), ptr, request.len()) };
    assert_eq!(unsafe { sim_wasm::call(0, ptr, request.len()) }, 0);
    let answer = unsafe { std::slice::from_raw_parts(sim_wasm::out_ptr(), sim_wasm::out_len()) };
    assert!(std::str::from_utf8(answer).unwrap().starts_with("[{\"id\":"));
    let ptr = sim_wasm::alloc(0);
    assert_eq!(unsafe { sim_wasm::call(9, ptr, 0) }, 1);
}

/// The 3D view's frame for every machine, read as the page reads it, against the machine's own scene from a native
/// simulation: at the start pose and after 0.5 s of its cycle, part by part the mesh, where the part's centre is, how
/// long the mesh is stretched along each axis of the part, and its colour.
#[test]
fn the_frame_draws_the_native_scene_of_every_machine_at_the_start_and_after_half_a_second() {
    for entry in sim_machines::MACHINES {
        let view = handle(3, &format!("{}\nen\n", entry.id)).unwrap();
        assert!(view.contains("\"radius\":"), "{view}");
        let mut sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
        sim.command(sim_components::Command::StartCycle).unwrap();
        for seconds in [0.0, 0.5] {
            let ptr = sim_wasm::frame(seconds);
            let frame = unsafe { std::slice::from_raw_parts(ptr, sim_wasm::frame_len()) };
            sim.run(seconds, |_| {});
            let mut scene = sim_core::Scene::default();
            sim.scene(&mut scene);
            let (head, parts) = frame.split_at(sim_wasm::HEAD);
            assert_eq!(head[0], sim.time() as f32, "{}", entry.id);
            assert_eq!(head[5] as usize, scene.parts.len(), "{}", entry.id);
            let samples = head[7] as usize * sim_wasm::sample_len(sim.machine.motors().len());
            let obstacles = head[6] as usize * sim_wasm::RECORD;
            assert_eq!(parts.len(), sim_wasm::RECORD * scene.parts.len() + obstacles + samples, "{}", entry.id);
            assert_eq!(head[6] as usize, scene.obstacles.len(), "{}", entry.id);
            let parts = &parts[..sim_wasm::RECORD * scene.parts.len()];
            if let Some(tool) = scene.tool {
                assert_eq!(&head[2..5], &tool.map(|v| v as f32), "{}", entry.id);
            }
            for (part, r) in scene.parts.iter().zip(parts.as_chunks::<{ sim_wasm::RECORD }>().0) {
                let column = |c: usize| (r[1 + 4 * c..4 + 4 * c].iter().map(|v| v * v).sum::<f32>()).sqrt() as f64;
                let (mesh, centre, lengths) = match part.shape {
                    sim_core::Shape::Box { size } => (0.0, part.pos, size.map(f64::abs)),
                    sim_core::Shape::Cylinder { radius, length } => {
                        (1.0, part.pos, [2.0 * radius, length, 2.0 * radius])
                    }
                    sim_core::Shape::Segment { to, width } => {
                        let length = (0..3).map(|i| (to[i] - part.pos[i]).powi(2)).sum::<f64>().sqrt();
                        (0.0, std::array::from_fn(|i| (part.pos[i] + to[i]) / 2.0), [length, width, 0.004])
                    }
                };
                let what = format!("{} {} at {seconds} s", entry.id, part.id);
                assert_eq!(r[0], mesh, "{what}");
                for i in 0..3 {
                    assert!((r[13 + i] as f64 - centre[i]).abs() < 1e-5, "{what}: {r:?}");
                    assert!((column(i) - lengths[i].max(1e-6)).abs() < 1e-5 * (1.0 + lengths[i]), "{what}: {r:?}");
                }
                assert_eq!(&r[17..20], &sim_core::scene::part_color(part.color, part.highlight, part.heat), "{what}");
            }
        }
    }
}

/// By hand: a box 0.2 × 0.4 × 0.6 m at (1, 2, 3) turned a quarter turn from +x towards +z has its x side along +z,
/// its y side along y and its z side along -x. It then reaches 0.3 m in x (half of 0.6), 0.2 m in y and 0.1 m in z
/// out from its centre. A belt strip from the origin to (3, 0, 4), 0.02 m wide: 5 m long along (0.6, 0, 0.8), its
/// width along y, its 4 mm thickness along (0.6, 0, 0.8) × (0, 1, 0) = (-0.8, 0, 0.6), centred at (1.5, 0, 2).
#[test]
fn records_and_bounds_by_hand() {
    use sim_core::scene::{Part, Shape};
    let mut out = Vec::new();
    let part = Part::new("x", Shape::Box { size: [0.2, 0.4, 0.6] }, [1.0, 2.0, 3.0], [0.5, 0.5, 0.5]);
    sim_wasm::record(&part.turned(std::f64::consts::FRAC_PI_2), &mut out);
    let strip = Part::new("belt", Shape::Segment { to: [3.0, 0.0, 4.0], width: 0.02 }, [0.0; 3], [0.1, 0.1, 0.1]);
    sim_wasm::record(&strip, &mut out);
    let expected: [[f32; 20]; 2] = [
        [0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.4, 0.0, 0.0, -0.6, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 1.0, 0.5, 0.5, 0.5],
        [0.0, 3.0, 0.0, 4.0, 0.0, 0.0, 0.02, 0.0, 0.0, -0.0032, 0.0, 0.0024, 0.0, 1.5, 0.0, 2.0, 1.0, 0.1, 0.1, 0.1],
    ];
    for (got, want) in out.as_chunks::<{ sim_wasm::RECORD }>().0.iter().zip(expected) {
        assert!(got.iter().zip(want).all(|(g, w)| (g - w).abs() < 1e-6), "{got:?}");
    }
    let (lo, hi) = sim_wasm::bounds(&out[..sim_wasm::RECORD]);
    let near = |a: [f32; 3], b: [f32; 3]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-6);
    assert!(near(lo, [0.7, 1.8, 2.9]) && near(hi, [1.3, 2.2, 3.1]), "{lo:?} {hi:?}");
}

/// The cycle time in the 3D view's legend is a pass from one start of the cycle to the next, which is the pass the
/// sizing check measures: the two agree to one base step once the view has run three passes.
#[test]
fn the_view_times_the_cycle_as_the_sizing_check_does() {
    let entry = sim_machines::MACHINES[0];
    let cfg = entry.config();
    let mut sim = sim_components::Sim::new(entry, &cfg).unwrap();
    let load = sim_components::analysis::analyse_cycle_with(&mut sim, false).unwrap();
    handle(3, &format!("{}\nen\n", entry.id)).unwrap();
    let ptr = sim_wasm::frame(3.0 * load.duration + 1.0);
    let cycle = unsafe { *ptr.add(1) } as f64;
    assert!((cycle - load.duration).abs() <= 1.01 * sim.dt, "{cycle} against {}", load.duration);
}

/// The error the page reads when `frame` refuses: the answer at `out_ptr`, as JSON; `None` where it ran.
fn refused(seconds: f64) -> Option<String> {
    sim_wasm::frame(seconds).is_null().then(|| {
        let answer = unsafe { std::slice::from_raw_parts(sim_wasm::out_ptr(), sim_wasm::out_len()) };
        String::from_utf8(answer.to_vec()).unwrap()
    })
}

/// An infinite, a NaN, a negative and an over-long time are refused with an error, and the view runs on afterwards;
/// before the fix an infinite time ran the step count to `u64::MAX` and hung the page.
#[test]
fn frame_refuses_times_that_are_not_finite_or_too_long() {
    handle(3, "single-axis\nen\n").unwrap();
    for seconds in [f64::INFINITY, f64::NAN, -1.0, sim_wasm::MAX_FRAME_S + 1.0, 1e300] {
        let error = refused(seconds).unwrap_or_else(|| panic!("{seconds} was run"));
        assert!(error.starts_with("{\"error\":\"frame: "), "{error}");
    }
    assert!(refused(0.01).is_none());
    // 0.01 s at 62.5 µs a step: 160 steps, and nothing more for 0 s.
    let ptr = sim_wasm::frame(0.0);
    assert!((unsafe { *ptr } as f64 - 0.01).abs() < 1e-6);
}

/// The machine files of machines/ are listed and run; one handed in is checked as a file (an error names the file
/// and the line) and is then a machine like the others.
#[test]
fn machine_files_are_listed_handed_in_and_run() {
    let list = handle(0, "en").unwrap();
    let names =
        ["corexy", "h-bot", "planar-delta", "rack-gantry", "screw-portal", "single-axis", "t-gantry", "xyz-portal"];
    for name in names {
        assert!(
            list.contains(&format!("\"id\":\"file:{name}.toml\",\"name\":\"{name}.toml\",\"file\":true")),
            "{list}"
        );
    }
    let text = include_str!("../../../machines/single-axis.toml");
    assert!(handle(1, "file:single-axis.toml").unwrap().contains("[machine]"));
    let shipped = handle(2, "file:single-axis.toml\nen\n").unwrap();
    assert!(shipped.contains("\"verdicts\":[{"), "{shipped}");
    // Handed in under a name of its own: the same machine and config give the same sizing check.
    let loaded = handle(4, &format!("mine.toml\nen\n{text}")).unwrap();
    assert!(loaded.starts_with("{\"id\":\"upload:mine.toml\""), "{loaded}");
    assert!(handle(0, "en").unwrap().contains("\"id\":\"upload:mine.toml\""));
    assert_eq!(handle(2, "upload:mine.toml\nen\n").unwrap(), shipped);
    assert!(handle(3, "upload:mine.toml\nen\n").unwrap().contains("\"radius\":"));
    // A misspelt key in the file: refused, by file, line and key.
    let broken = text.replacen("[sim]\n", "[sim]\nno_such_key = 1\n", 1);
    let line = broken.lines().position(|l| l.starts_with("no_such_key")).unwrap() + 1;
    let error = handle(4, &format!("broken.toml\nen\n{broken}")).unwrap_err();
    assert!(error.contains(&format!("broken.toml:{line}")) && error.contains("no_such_key"), "{error}");
}

/// The single axis's path from 0.1 m (its start pose) to 0.5 m and back to 0.2 m: the sizing check of the path is
/// `analyse_trajectory` of the same planned poses, and the 3D view runs it to its end and stands where a native run
/// of it stands.
#[test]
fn a_path_runs_in_the_sizing_check_and_the_view() {
    let entry = sim_machines::find("single-axis").unwrap();
    let path = "line,0.5\n# back\nline,0.2\n";
    let json = handle(2, &format!("single-axis\nen path\n\n---\n{path}")).unwrap();
    let cfg = entry.config();
    let mut sim = sim_components::Sim::new(entry, &cfg).unwrap();
    let pieces = sim_components::path::parse(path).unwrap();
    let points = sim.plan_path(&pieces, false).unwrap();
    let end = points.last().unwrap().0;
    let load = sim_components::analysis::analyse_trajectory_with(&mut sim, points, false).unwrap();
    assert!(json.contains(&format!("\"cycle_s\":{:?}", load.duration)), "{json}");
    assert!(json.contains(&format!("\"peak_torque_nm\":{:?}", load.motors[0].peak_torque)), "{json}");
    assert!(handle(2, &format!("single-axis\nen path time-optimal\n\n---\n{path}")).unwrap().contains("\"verdicts\""));
    handle(3, &format!("single-axis\nen path\n\n---\n{path}")).unwrap();
    let x = unsafe { *sim_wasm::frame(end + 1.0).add(2) };
    let mut native = sim_components::Sim::new(entry, &cfg).unwrap();
    native.command(sim_components::Command::Trajectory(native.plan_path(&pieces, false).unwrap())).unwrap();
    native.run(end + 1.0, |_| {});
    assert_eq!(x, native.tool_point().unwrap()[0] as f32);
    assert!(handle(2, "single-axis\nen path\n\n---\nline,oops\n").unwrap_err().contains("oops"));
    assert!(handle(2, "single-axis\nen path\n\n---\n# nothing\n").is_err());
}

/// The samples of 0.5 s of the T-gantry's cycle: 62.5 µs a step and 1 ms a sample make 16 steps a sample and 500
/// samples, the first at 1 ms and the last at 0.5 s; and the highest torque and the lowest following error of motor 0
/// over the samples are those of a native run, step by step.
#[test]
fn samples_keep_every_peak_of_the_steps() {
    let entry = sim_machines::find("t-gantry").unwrap();
    let view = handle(3, "t-gantry\nen\n").unwrap();
    let mut sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
    let size = sim_wasm::sample_len(sim.machine.motors().len());
    assert!(view.contains(&format!("\"sample\":{size}")), "{view}");
    sim.command(sim_components::Command::StartCycle).unwrap();
    let (mut peak, mut lowest) = (f64::MIN, f64::MAX);
    sim.run(0.5, |s| {
        peak = peak.max(s.control.drives[0].torque);
        lowest = lowest.min(s.control.drives[0].following_error);
    });
    let ptr = sim_wasm::frame(0.5);
    let frame = unsafe { std::slice::from_raw_parts(ptr, sim_wasm::frame_len()) };
    assert_eq!(frame[7], 500.0);
    let at = sim_wasm::HEAD + sim_wasm::RECORD * (frame[5] + frame[6]) as usize;
    let samples: Vec<&[f32]> = frame[at..].chunks(size).collect();
    assert_eq!(samples.len(), 500);
    assert!((samples[0][0] - 0.001).abs() < 1e-6 && (samples[499][0] - 0.5).abs() < 1e-6);
    let head = sim_wasm::SAMPLE_HEAD;
    // Motor 0: torque lowest, highest at head, head + 1; following error at head + 8, head + 9.
    let high = samples.iter().map(|s| s[head + 1]).fold(f32::MIN, f32::max);
    let low = samples.iter().map(|s| s[head + 8]).fold(f32::MAX, f32::min);
    assert_eq!((high, low), (peak as f32, lowest as f32));
    assert!(samples.iter().all(|s| s[head] <= s[head + 1] && s[4] >= 0.0));
}

/// By hand: an obstacle with its centre at (0.5, 0.2, 0.06) and 0.1 m on each side is the cube stretched by 0.1 along
/// each axis, moved to its centre, grey; touched, red. The single axis's config with it draws it in its frame.
#[test]
fn obstacles_are_drawn() {
    let mut obstacle = sim_core::scene::Obstacle {
        name: "fixture".into(),
        aabb: sim_core::Aabb { centre: [0.5, 0.2, 0.06], size: [0.1, 0.1, 0.1] },
        contact: false,
        stiffness: 1.0e7,
        damping: 1.0e3,
    };
    let mut out = Vec::new();
    sim_wasm::record_obstacle(&obstacle, &mut out);
    obstacle.contact = true;
    sim_wasm::record_obstacle(&obstacle, &mut out);
    let want = [2.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.5, 0.2, 0.06, 1.0];
    assert!(out[..17].iter().zip(want).all(|(g, w)| (g - w).abs() < 1e-7), "{out:?}");
    assert_eq!(&out[17..20], &[0.55, 0.57, 0.62]);
    assert_eq!(&out[37..40], &[0.9, 0.15, 0.1]);
    let entry = sim_machines::find("single-axis").unwrap();
    let cfg = entry.default_config.replacen(
        "\nobstacles = []",
        "\nobstacles = [{ name = \"fixture\", centre_m = [0.5, 0.2, 0.06], size_m = [0.1, 0.1, 0.1] }]",
        1,
    );
    handle(3, &format!("single-axis\nen\n{cfg}")).unwrap();
    let frame = unsafe { std::slice::from_raw_parts(sim_wasm::frame(0.0), sim_wasm::frame_len()) };
    assert_eq!(frame[6], 1.0);
    let at = sim_wasm::HEAD + sim_wasm::RECORD * frame[5] as usize;
    assert_eq!(frame[at], sim_wasm::OBSTACLE_MESH);
    assert_eq!(&frame[at + 13..at + 16], &[0.5, 0.2, 0.06]);
}

/// The sizing check has every column of `sim-cli analyze`: housing rise, drive overload, current and energy among
/// them, the values of the native analysis; and per motor the operating points and the torque-speed curve.
#[test]
fn sizing_has_the_columns_of_analyze() {
    let entry = sim_machines::find("t-gantry").unwrap();
    let json = handle(2, "t-gantry\nen\n").unwrap();
    let mut sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
    let load = sim_components::analysis::analyse_cycle_with(&mut sim, false).unwrap();
    let m = &load.motors[0];
    for (key, value) in [
        ("rms_current_a", m.rms_current),
        ("housing_rise_k", m.housing_rise),
        ("drive_overload", m.drive_overload),
        ("energy_in_j", m.energy_in),
        ("mains_energy_j", m.mains_energy),
        ("path_error_peak_m", load.path_error_peak),
    ] {
        assert!(json.contains(&format!("\"{key}\":{value:?}")), "{key} {value:?} missing in {json}");
    }
    for key in
        ["belt_tension_min_n", "guide_life_h", "motor_bearing_life_h", "\"points\":[[", "\"curve\":[[", "\"s1\":[["]
    {
        assert!(json.contains(key), "{key}");
    }
}

/// By hand from the T-gantry's motor 0 drive: the curve starts and ends at the no-load speed backwards, passes 0 rad/s
/// at the peak torque both ways, has the rated speed both ways as corners, and the S1 box has the rated torque at
/// ± the rated speed.
#[test]
fn torque_speed_curve_by_hand() {
    let entry = sim_machines::find("t-gantry").unwrap();
    let sim = sim_components::Sim::new(entry, &entry.config()).unwrap();
    let drive = sim.control.drive_params_of(0);
    let (curve, s1) = sim_wasm::torque_speed_curve(drive);
    assert_eq!(curve[0].0, -drive.no_load_speed);
    assert_eq!(curve.last().unwrap().0, -drive.no_load_speed);
    let at_rest: Vec<f64> = curve.iter().filter(|p| p.0 == 0.0).map(|p| p.1).collect();
    assert_eq!(at_rest, [drive.peak_torque, -drive.peak_torque]);
    for s in [drive.rated_speed, -drive.rated_speed] {
        assert!(curve.iter().any(|p| p.0 == s), "{s}");
    }
    let (r, t) = (drive.rated_speed, drive.rated_torque);
    assert_eq!(s1, [(-r, t), (r, t), (r, -t), (-r, -t), (-r, t)]);
}

/// The page's sweep is `frequency_sweep` at the same frequencies: 3 from 10 Hz to 1000 Hz are 10, 100 and 1000 Hz.
#[test]
fn sweep_is_the_command_lines() {
    let entry = sim_machines::find("single-axis").unwrap();
    let json = handle(5, "single-axis\nen motor=0 from=10 to=1000 count=3\n").unwrap();
    let cfg = entry.config();
    let amplitude = sim_components::analysis::default_sweep_amplitude(entry, &cfg).unwrap();
    let points = sim_components::analysis::frequency_sweep(entry, &cfg, 0, &[10.0, 100.0, 1000.0], amplitude).unwrap();
    assert_eq!(points.len(), 3);
    for p in points {
        assert!(json.contains(&format!("[{:?},{:?},{:?}]", p.frequency, p.gain, p.phase_deg)), "{json}");
    }
    assert!(handle(5, "single-axis\nen motor=0.5\n").is_err());
    assert!(handle(5, "single-axis\nen count=nope\n").unwrap_err().contains("count=nope"));
}

/// A trace that lacks the simulator's columns is refused with the trace's error, translated.
#[test]
fn fit_refuses_a_trace_without_its_columns() {
    let error = handle(6, "single-axis\nen\n\n---\ntime,something\n0,1\n").unwrap_err();
    assert!(error.contains("motor."), "{error}");
}

/// The page's texts come in the language asked for, with the simulator's quantity names asked for by key; every
/// English text has a German one.
#[test]
fn page_texts_in_both_languages() {
    let de = handle(7, "de\nquantity-torque\n").unwrap();
    assert!(de.contains("\"web-title\":\"Maschinensimulator\""), "{de}");
    assert!(de.contains("\"quantity-torque\":\"Drehmoment\""), "{de}");
    let en = handle(7, "en").unwrap();
    assert!(en.contains("\"web-col-housing_rise_k\":\"Housing rise [K]\""), "{en}");
    let keys = |ftl: &str| -> Vec<String> {
        let lines = ftl.lines().filter(|l| !l.starts_with('#'));
        lines.filter_map(|l| l.split_once(" = ").map(|(k, _)| k.to_string())).collect()
    };
    assert_eq!(keys(include_str!("../i18n/en.ftl")), keys(include_str!("../i18n/de.ftl")));
}
