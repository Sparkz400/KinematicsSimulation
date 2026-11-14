//! Property test of a program from the reader to the planned poses, as `sim-cli run --path` and `verdicts --path` take
//! it (`sim::with_path`, `Sim::plan_configured_path`): whatever the program, the poses at times or an error, within a
//! few seconds and with a bounded number of poses, on a gantry, a delta and an arm. Seeds: fuzz/seeds/program.

#[path = "../../sim-core/tests/edits/mod.rs"]
mod edits;

use proptest::prelude::*;
use sim_components::Sim;
use sim_components::sim::with_path;
use std::sync::mpsc;
use std::time::Duration;

const MACHINES: [&str; 3] = ["t-gantry", "delta-d3", "urdf-arm"];

/// The number of poses the plan has, or why it was refused; `None` where it took longer than `limit`.
fn plan(
    id: &'static str,
    name: String,
    text: String,
    time_optimal: bool,
    limit: Duration,
) -> Option<Result<usize, String>> {
    let (send, receive) = mpsc::channel();
    std::thread::spawn(move || {
        let entry = sim_machines::find(id).unwrap();
        let result = with_path(&entry.config(), &name, &text, time_optimal)
            .map_err(|e| e.key().to_string())
            .and_then(|cfg| Sim::new(entry, &cfg).map_err(|e| e.key().to_string()))
            .and_then(|sim| match sim.plan_configured_path() {
                Some(Ok(points)) => Ok(points.len()),
                Some(Err(e)) => Err(e.key.to_string()),
                None => Err("none".into()),
            });
        let _ = send.send(result);
    });
    receive.recv_timeout(limit).ok()
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(48))]

    #[test]
    fn edited_programs_are_planned_or_refused_in_bounded_time(
        (name, text) in edits::edited(edits::files("fuzz/seeds/program", "")),
        id in prop::sample::select(MACHINES.to_vec()),
        time_optimal in any::<bool>(),
    ) {
        let planned = plan(id, name.clone(), text.clone(), time_optimal, Duration::from_secs(60));
        prop_assert!(planned.is_some(), "{id}, {name}: not planned within 60 s:\n{text}");
        if let Some(Ok(points)) = planned {
            prop_assert!(points < 10_000_000, "{id}, {name}: {points} poses");
        }
    }
}

/// Programs whose plan, if made, would have to hold more poses than memory has.
#[test]
fn programs_beyond_any_plan_are_refused_in_bounded_time() {
    for (id, name, text) in [
        ("t-gantry", "p.path", "line,1e300,0"),
        ("t-gantry", "p.nc", "F1e-300\nG1 X0.1"),
        ("t-gantry", "p.nc", "F0.0001\nG1 X100"),
        ("t-gantry", "p.nc", "F0.000001\nG1 X100"),
        ("t-gantry", "p.src", "DEF d()\n$VEL.CP = 0.000000001\nLIN {X 100}\nEND"),
    ] {
        for time_optimal in [false, true] {
            let planned = plan(id, name.into(), text.into(), time_optimal, Duration::from_secs(60));
            assert!(
                planned.as_ref().is_some_and(|p| p.as_ref().is_ok_and(|&n| n < 10_000_000) || p.is_err()),
                "{text}: {planned:?}"
            );
        }
    }
    // 0.1 m at 1e-6 mm/min (1.7e-11 m/s) is 190 years: 6e12 poses of 1 ms. Planned, it ran until memory was full.
    let planned = plan("t-gantry", "p.nc".into(), "F0.000001\nG1 X100".into(), false, Duration::from_secs(60));
    assert_eq!(planned, Some(Err("error-path-long".into())));
}
