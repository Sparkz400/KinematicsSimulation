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

/// A cancelled job leaves nothing behind that a later analysis trips on (a lock poisoned by the `Cancelled` unwind, a
/// job left installed on a thread): a sampled bands run of 2000 samples cancelled from another thread once runs are
/// done, then the plain bands on this thread and on another give what they gave before, to the bit.
#[test]
fn an_analysis_after_a_cancelled_one_gives_what_it_gave_before() {
    use sim_components::analysis::{Cancelled, Progress, with_progress};
    use std::sync::{Arc, atomic::Ordering::Relaxed};
    let entry = MACHINES.iter().find(|e| e.id == "single-axis").unwrap();
    let cfg = entry.config();
    let bounded = bounds(&cfg).unwrap();
    let before = format!("{:?}", verdict_bands(entry, &cfg, &bounded).unwrap());
    let job = Arc::new(Progress::default());
    let watcher = {
        let job = job.clone();
        std::thread::spawn(move || {
            while job.done.load(Relaxed) < 3 {
                std::thread::yield_now();
            }
            job.cancel.store(true, Relaxed);
        })
    };
    let cancelled = with_progress(&job, || verdict_bands_sampled(entry, &cfg, &bounded, 2000, 1));
    watcher.join().unwrap();
    assert_eq!(cancelled.err(), Some(Cancelled));
    assert!(job.done.load(Relaxed) < job.total.load(Relaxed));
    assert_eq!(format!("{:?}", verdict_bands(entry, &cfg, &bounded).unwrap()), before);
    let elsewhere =
        std::thread::scope(|s| s.spawn(|| format!("{:?}", verdict_bands(entry, &cfg, &bounded).unwrap())).join());
    assert_eq!(elsewhere.unwrap(), before);
}

/// Work that runs on the job's own thread, not in a batch of runs, stops at its next check too: the cycle's analysis
/// (`analyse_cycle_with`, where every sequential batch of cycles passes) and a step response (`settled`) in a job
/// cancelled before they start end with `Cancelled`, and run as before outside a job.
#[test]
fn sequential_work_in_a_cancelled_job_ends_with_cancelled() {
    use sim_components::analysis::{Cancelled, Progress, analyse_cycle, step_response, with_progress};
    use std::sync::{Arc, atomic::Ordering::Relaxed};
    let entry = MACHINES.iter().find(|e| e.id == "single-axis").unwrap();
    let cfg = entry.config();
    let cancelled = Arc::new(Progress::default());
    cancelled.cancel.store(true, Relaxed);
    let cycle = || analyse_cycle(&mut sim_components::Sim::new(entry, &cfg).unwrap()).map(|_| ());
    assert_eq!(with_progress(&cancelled, cycle).err(), Some(Cancelled));
    assert!(cycle().is_ok());
    let step = || step_response(entry, &cfg, 0, 0.001, 0.05).map(|_| ());
    assert_eq!(with_progress(&cancelled, step).err(), Some(Cancelled));
    assert!(step().is_ok());
}
