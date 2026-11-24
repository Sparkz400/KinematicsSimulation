//! A machine file loaded again is the same entry, a changed one a new entry, and a refused one keeps nothing: a sweep or
//! an FMU made in a loop must not grow memory. One test, as the count of kept entries is the process's.

use sim_machines::load_text;
use std::path::Path;

#[test]
fn the_same_text_gives_the_same_entry_a_changed_one_a_new_one_and_a_refused_one_nothing() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../machines/single-axis.toml");
    let text = std::fs::read_to_string(path).unwrap().replace("\r\n", "\n");
    let start = sim_netlist::kept_entries();

    let first = load_text(&text, "cache").unwrap();
    let again = load_text(&text, "cache").unwrap();
    assert!(std::ptr::eq(first, again), "the same text is the same entry");
    // Loaded from several threads at once, still one entry.
    let from_threads: Vec<usize> = std::thread::scope(|s| {
        let loads: Vec<_> =
            (0..4).map(|_| s.spawn(|| load_text(&text, "cache").unwrap() as *const _ as usize)).collect();
        loads.into_iter().map(|l| l.join().unwrap()).collect()
    });
    assert!(from_threads.iter().all(|&p| p == first as *const _ as usize));
    assert_eq!(sim_netlist::kept_entries(), start + 1);

    // A changed text is a new entry with the changed value, never the stale one.
    let changed = text.replace("payload_kg = 2.0", "payload_kg = 3.0");
    assert_ne!(changed, text);
    let other = load_text(&changed, "cache").unwrap();
    assert!(!std::ptr::eq(first, other));
    assert_eq!(first.config().f64("mass.payload_kg").unwrap(), 2.0);
    assert_eq!(other.config().f64("mass.payload_kg").unwrap(), 3.0);
    assert_eq!(sim_netlist::kept_entries(), start + 2);

    // A refused text (a payload mass that is no number) adds nothing, however often it is tried.
    let broken = text.replace("payload_kg = 2.0", "payload_kg = \"heavy\"");
    for _ in 0..3 {
        assert!(load_text(&broken, "cache").is_err());
    }
    assert_eq!(sim_netlist::kept_entries(), start + 2);
}
