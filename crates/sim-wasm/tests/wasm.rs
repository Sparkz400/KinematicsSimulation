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
    let error = handle(2, &format!("{}\nen\n{}\n[no_such_table]\nno_such_key = 1\n", entry.id, entry.default_config)).unwrap_err();
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
