//! The analyses run their cycles side by side (`analysis::on_all_cores`, the optimiser's candidates a batch at a time):
//! what they find must not depend on how many threads run them (`analysis::set_max_threads`), to the bit.

use sim_components::analysis::{
    OptimiseOptions, bounds, optimise_cycle_with, sensitivity, set_max_threads, verdict_bands, verdict_bands_sampled,
};
use sim_machines::MACHINES;

#[test]
fn the_analyses_find_the_same_on_one_thread_as_on_many() {
    let entry = MACHINES.iter().find(|e| e.id == "single-axis").unwrap();
    let cfg = entry.config();
    // Four of the bounds and three of their parameters: enough runs for every thread, few enough for one.
    let bounded: Vec<_> = bounds(&cfg).unwrap().into_iter().take(4).collect();
    let paths: Vec<String> = bounded.iter().take(3).map(|b| b.0.clone()).collect();
    let run = |threads: usize| -> Vec<String> {
        set_max_threads(threads);
        // Over the search's 40 runs batches of several candidates run, some with a faster one inside (counting a batch's
        // tries in another order failed this test).
        let options = OptimiseOptions { max_runs: 40, robust: false, pareto: true, worst: false };
        vec![
            format!("{:?}", verdict_bands(entry, &cfg, &bounded).unwrap()),
            format!("{:?}", verdict_bands_sampled(entry, &cfg, &bounded, 6, 1).unwrap()),
            format!("{:?}", sensitivity(entry, &cfg, &paths, 0.2).unwrap()),
            format!("{:?}", optimise_cycle_with(entry, &cfg, &options).unwrap()),
        ]
    };
    let (one, many) = (run(1), run(5));
    set_max_threads(0);
    for (what, (one, many)) in ["bands", "sampled bands", "sensitivity", "optimiser"].iter().zip(one.iter().zip(&many))
    {
        let at = one.chars().zip(many.chars()).take_while(|(a, b)| a == b).count();
        let around = |s: &str| s.chars().skip(at.saturating_sub(80)).take(200).collect::<String>();
        assert!(
            one == many,
            "{what} differ from character {at}: one thread\n{}\nfive threads\n{}",
            around(one),
            around(many)
        );
    }
}
