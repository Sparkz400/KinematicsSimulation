//! The stop analysis (`sim_components::stop::study`) on every registered machine, each on a thread of its own: it
//! reports stops where the axes are fastest and where the energy is largest (no grid here), the worst of them with the
//! longest tool distance, with finite figures.

use sim_components::StopCategory;
use sim_components::stop::{Moment, study};
use sim_machines::MACHINES;

#[test]
fn the_stop_study_reports_the_worst_stop_on_every_machine() {
    std::thread::scope(|scope| {
        for entry in MACHINES {
            scope.spawn(move || {
                let found = study(entry, &entry.config(), StopCategory::One, 0).unwrap();
                let id = entry.id;
                // Where an axis is fastest and the energy largest at the same moment, the stop there is named once.
                let chosen = |s: &&sim_components::stop::Stop| matches!(s.moment, Moment::Fastest(_) | Moment::Energy);
                assert!(!found.stops.is_empty() && found.stops.iter().all(|s| chosen(&s)), "{id}: {:?}", found.stops);
                let worst = &found.stops[found.worst];
                assert!(found.stops.iter().all(|s| s.tool_distance <= worst.tool_distance), "{id}: not the worst");
                for s in &found.stops {
                    let figures = [s.tool_distance, s.time, s.peak_deceleration, s.peak_dc_voltage, s.resistor_energy];
                    assert!(figures.iter().chain(&s.axis_distance).all(|v| v.is_finite() && *v >= 0.0), "{id}: {s:?}");
                }
                assert!(worst.tool_distance > 0.0 && worst.time > 0.0, "{id}: {worst:?}");
                assert!(found.verdicts.iter().all(|v| v.check != "stopping-distance"), "{id}: no rating, no verdict");
            });
        }
    });
}
