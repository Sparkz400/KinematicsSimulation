//! Benchmarks per registered machine with its default config: one integrator step (`Sim::step`, the control and the
//! drives with it) while the cycle runs, one whole pass of the default cycle from a fresh simulation, and the cycle
//! analysis with its verdicts (`analyse_cycle`, `verdicts`). The steps and cycles run on main in
//! .github/workflows/bench.yml, which warns when a median gets over 15 % slower (tools/bench_compare.py).
//! `cargo bench -p sim-machines`, or one machine: `cargo bench -p sim-machines -- t-gantry`. Before the benchmarks it
//! prints each machine's base step and the simulated length of one pass, for the ratio to real time.

use criterion::{BatchSize, Criterion, criterion_group, criterion_main};
use sim_components::analysis::{analyse_cycle, verdicts};
use sim_components::{Command, Sim};
use sim_core::MachineEntry;
use sim_machines::MACHINES;
use std::hint::black_box;
use std::time::Duration;

/// A pass of the cycle longer than this (simulated) is cut off: no default cycle is near it.
const LONGEST_PASS_S: f64 = 120.0;

fn sim(entry: &MachineEntry) -> Sim {
    Sim::new(entry, &entry.config()).unwrap_or_else(|e| panic!("{}: {e:?}", entry.id))
}

/// The first pass of the default cycle from the start, its way from the start pose in: the simulated seconds it took.
/// The controller counts a pass on the cycle's first step, so the pass ends at the second count (as `analyse_cycle`
/// reads it).
fn pass(sim: &mut Sim) -> f64 {
    sim.command(Command::StartCycle).unwrap();
    let (mut counted, mut last) = (0, sim.control.cycle_passes());
    while counted < 2 && sim.time() < LONGEST_PASS_S {
        sim.step();
        if sim.control.cycle_passes() != last {
            (counted, last) = (counted + 1, sim.control.cycle_passes());
        }
    }
    sim.time()
}

fn benches(c: &mut Criterion) {
    for entry in MACHINES {
        eprintln!("{}: dt {:e} s, one pass {:.4} s", entry.id, sim(entry).dt, pass(&mut sim(entry)));
    }

    let mut step = c.benchmark_group("step");
    step.warm_up_time(Duration::from_millis(500)).measurement_time(Duration::from_secs(2));
    for entry in MACHINES {
        let mut sim = sim(entry);
        sim.command(Command::StartCycle).unwrap();
        step.bench_function(entry.id, |b| b.iter(|| black_box(&mut sim).step()));
    }
    step.finish();

    let mut cycle = c.benchmark_group("cycle");
    cycle.sample_size(10).warm_up_time(Duration::from_millis(500)).measurement_time(Duration::from_secs(5));
    for entry in MACHINES {
        cycle.bench_function(entry.id, |b| {
            b.iter_batched(|| sim(entry), |mut sim| pass(&mut sim), BatchSize::LargeInput)
        });
    }
    cycle.finish();

    let mut analysis = c.benchmark_group("analysis");
    analysis.sample_size(10).warm_up_time(Duration::from_millis(500)).measurement_time(Duration::from_secs(5));
    for entry in MACHINES {
        let cfg = entry.config();
        analysis.bench_function(entry.id, |b| {
            b.iter_batched(
                || sim(entry),
                |mut sim| verdicts(&cfg, &analyse_cycle(&mut sim).unwrap()).unwrap(),
                BatchSize::LargeInput,
            )
        });
    }
    analysis.finish();
}

criterion_group!(simulate, benches);
criterion_main!(simulate);
