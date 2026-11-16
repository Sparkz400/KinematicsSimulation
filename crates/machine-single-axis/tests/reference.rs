//! The closed loop of the servo drive against an independent implementation: `reference/drive_reference.py`, written
//! again in Python from docs/model-reference.md (not from the Rust source), with the mechanics and the winding between
//! drive samples integrated by SciPy's DOP853 (rtol 1e-10) instead of RK4. Each `reference/stage<N>.csv` holds the
//! switches and values it ran with (`# set` lines), the move, and the trace; the simulator runs the same here and every
//! column has to agree within the stage's tolerance, relative to the column's peak.
//!
//! The drive is discrete (cycle times, zero-order hold), so where the mechanics are smooth the two agree to the
//! integrators' error: about 1e-10 in stages 1 and 2 (rigid axis, the CSV's twelve digits), 1e-8 in stage 3 away from
//! the gearbox's backlash. The backlash's contact is a kink in the equations, through which the fixed RK4 step (15.6 µs)
//! is less exact than an adaptive one: up to 7e-5 of the peak where the torque reverses, in stages 3 and 4.

use machine_single_axis::ENTRY;
use sim_components::{Command, Sim};

/// Runs the simulator as the reference file says and returns the largest difference of each column over its peak.
fn worst(reference: &str) -> Vec<(String, f64)> {
    let mut cfg = ENTRY.config();
    let (mut target, mut seconds, mut every) = (None, None, None);
    for line in reference.lines().filter_map(|l| l.strip_prefix("# ")) {
        if let Some(set) = line.strip_prefix("set ") {
            cfg.set_text(set).unwrap_or_else(|e| panic!("{set}: {e:?}"));
        } else if let Some(v) = line.strip_prefix("move ") {
            target = Some(v.trim().parse::<f64>().unwrap());
        } else if let Some(v) = line.strip_prefix("time ") {
            seconds = Some(v.trim().parse::<f64>().unwrap());
        } else if let Some(v) = line.strip_prefix("every ") {
            every = Some(v.trim().parse::<u64>().unwrap());
        }
    }
    let (target, seconds, every) = (target.unwrap(), seconds.unwrap(), every.unwrap());
    let mut rows = reference.lines().filter(|l| !l.starts_with('#'));
    let columns: Vec<&str> = rows.next().unwrap().trim().split(',').collect();
    let expected: Vec<Vec<f64>> = rows.map(|l| l.trim().split(',').map(|v| v.parse().unwrap()).collect()).collect();

    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    sim.command(Command::MoveTo(vec![target])).unwrap();
    let index: Vec<usize> = columns.iter().map(|c| sim.signal_index(c).unwrap_or_else(|| panic!("{c}"))).collect();
    let (mut sample, mut ours) = (Vec::new(), Vec::new());
    for step in 0..=(seconds / sim.dt).round() as u64 {
        if step % every == 0 {
            sim.sample_base(&mut sample);
            ours.push(index.iter().map(|&i| sample[i]).collect::<Vec<f64>>());
        }
        sim.step();
    }
    assert_eq!(ours.len(), expected.len(), "the reference trace has a different length");
    (0..columns.len())
        .map(|c| {
            let peak = expected.iter().map(|r| r[c].abs()).fold(0.0, f64::max).max(1e-300);
            let diff = ours.iter().zip(&expected).map(|(a, b)| (a[c] - b[c]).abs()).fold(0.0, f64::max);
            (columns[c].to_string(), diff / peak)
        })
        .collect()
}

fn agrees(reference: &str, tolerance: f64) {
    let worst = worst(reference);
    for (column, d) in &worst {
        sim_core::testkit::record_check(&format!("{column}: largest difference over its peak"), 0.0, *d, tolerance);
    }
    let failed: Vec<_> = worst.iter().filter(|(_, d)| !(*d <= tolerance)).collect();
    assert!(failed.is_empty(), "beyond {tolerance:e} of the peak: {failed:?}");
}

/// Rigid loss-free axis: position, velocity and current loop with the back-EMF.
#[test]
fn the_rigid_closed_loop_matches_the_python_reference() {
    agrees(include_str!("../reference/stage1.csv"), 1e-8);
}

/// Plus delays, filters, speed and torque feedforward, torque saturation, and the torque, drive current and voltage
/// limits, each of which acts within the move.
#[test]
fn the_drive_with_delays_filters_feedforward_and_limits_matches_the_python_reference() {
    agrees(include_str!("../reference/stage2.csv"), 1e-8);
}

/// Plus the elastic belt with slack, damping and mass, the gearbox with backlash, stiffness and efficiency, guide
/// friction (LuGre), motor friction, torque ripple, cogging and tooth meshing.
#[test]
fn the_drive_on_the_elastic_belt_axis_matches_the_python_reference() {
    agrees(include_str!("../reference/stage3.csv"), 2e-4);
}

/// Plus the winding's heating (started warm) and the DC link with its braking chopper, which the move's braking
/// reaches.
#[test]
fn the_drive_with_winding_heating_and_dc_link_matches_the_python_reference() {
    agrees(include_str!("../reference/stage4.csv"), 2e-4);
}
