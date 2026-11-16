//! Morris and Sobol' on a machine (`sim_components::sensitivity`): the same seed gives the same ranking on one thread as
//! on many, and a value the cycle never reads moves no figure of the cycle. The methods' numbers themselves are checked on
//! analytic functions in sim-components.

use sim_components::analysis::{bounds, set_max_threads};
use sim_components::sensitivity::{morris, sobol};
use sim_machines::MACHINES;

/// Coulomb and viscous friction of the slide, and the lubricant's viscosity, which only the bearings' life reads.
fn picked() -> (&'static sim_core::MachineEntry, sim_core::Config, Vec<(String, f64, f64)>) {
    let entry = MACHINES.iter().find(|e| e.id == "single-axis").unwrap();
    let cfg = entry.config();
    let wanted = ["friction_x.coulomb_n", "friction_x.viscous_n_s_m", "ratings.bearing_lubricant_viscosity_mm2_s"];
    let bounded: Vec<_> = bounds(&cfg).unwrap().into_iter().filter(|b| wanted.contains(&b.0.as_str())).collect();
    assert_eq!(bounded.len(), 3);
    (entry, cfg, bounded)
}

#[test]
fn the_same_seed_gives_the_same_ranking_on_one_thread_as_on_many() {
    let (entry, cfg, bounded) = picked();
    let run = |threads: usize| {
        set_max_threads(threads);
        (
            format!("{:?}", morris(entry, &cfg, &bounded, 2, 4).unwrap()),
            format!("{:?}", sobol(entry, &cfg, &bounded, 3, 4).unwrap()),
        )
    };
    let (one, many) = (run(1), run(5));
    set_max_threads(0);
    assert!(one == many, "one thread and five differ");
    assert_ne!(one.0, format!("{:?}", morris(entry, &cfg, &bounded, 2, 5).unwrap()), "another seed, the same points");
}

#[test]
fn friction_moves_the_rms_torque_and_the_lubricant_does_not() {
    let (entry, cfg, bounded) = picked();
    let rankings = morris(entry, &cfg, &bounded, 3, 1).unwrap();
    let rms = rankings.iter().find(|r| r.result.starts_with("rms-torque:")).expect("an RMS torque result");
    // More friction, more torque: every effect of either friction is positive, so mu = mu*.
    for row in &rms.rows[..2] {
        assert!(row.parameter.starts_with("friction_x.") && row.mu_star > 0.0 && row.mu == row.mu_star, "{row:?}");
    }
    // The lubricant enters no equation of motion: the same torque to the bit, mu* exactly 0, ranked last.
    assert_eq!(rms.rows[2].parameter, "ratings.bearing_lubricant_viscosity_mm2_s");
    assert_eq!((rms.rows[2].mu_star, rms.rows[2].sigma), (0.0, 0.0));
    let sobol = sobol(entry, &cfg, &bounded, 4, 1).unwrap();
    let rms = sobol.iter().find(|r| r.result == rms.result).unwrap();
    let lubricant = rms.rows.iter().find(|r| r.parameter.starts_with("ratings.")).unwrap();
    assert_eq!((lubricant.total, lubricant.total_ci), (0.0, (0.0, 0.0)));
}
