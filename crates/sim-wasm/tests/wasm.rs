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
            assert_eq!(parts.len(), sim_wasm::RECORD * scene.parts.len());
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
