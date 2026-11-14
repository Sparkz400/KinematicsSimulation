//! The belt's stiffness from an ordinary move's trace, no sweep (`identify::fit_move_stiffness`).

use sim_components::Command;
use sim_components::identify::{BELT_STIFFNESS_KEY, MOVE_STIFFNESS_LIMIT, fit_move_stiffness};
use sim_components::sim::Sim;
use sim_components::trace::Trace;
use sim_core::testkit::Noise;
use sim_core::{Config, MachineEntry};

/// Point-to-point moves to `targets`, `rows` milliseconds each, recorded every millisecond, with the noise the single
/// axis's fit test puts on torque, speed and angle.
fn move_trace(entry: &MachineEntry, cfg: &Config, targets: &[f64], rows: usize, noise: &mut Noise) -> Trace {
    let mut sim = Sim::new(entry, cfg).unwrap();
    let mut trace = Trace::default();
    for &target in targets {
        sim.command(Command::MoveTo(vec![target])).unwrap();
        for _ in 0..rows {
            trace.record(&sim);
            sim.run(1e-3, |_| {});
        }
    }
    let m = &mut trace.motors[0];
    for row in 0..m.torque.len() {
        m.torque[row] += 0.005 * noise.next();
        m.speed[row] += 0.1 * noise.next();
        m.angle[row] += 1e-5 * noise.next();
    }
    trace
}

/// The single axis with its belt 0.7 times as stiff as the config's (EA 350 kN for 500 kN), two moves of 0.8 m at the
/// config's limits (1 m/s, 10 m/s², 100 m/s³), noise on torque, speed and angle as in the stage-1 test. The fit scales
/// the belt's own key, so it has to find 0.7 itself (a sweep's dip gives 0.72, the gearbox being in series with the
/// belt). Found (13 November): 0.708 ± 0.015; the ringing 7.8 mN m RMS, of which the replay leaves 4.9 mN m, which is
/// the noise: 0.005 N m less the window's mean, sqrt(20 / 21) of it, 4.88 mN m.
///
/// What rings the belt in a move is the jerk at the ramps and, at speed, the motor's ripple sweeping through the mode
/// (24 per turn of the motor, 5 turns per turn of the pulley of 20 mm: 70 Hz at 0.073 m/s). A move of 2 mm at 1 m/s³
/// and 0.2 m/s² rings neither: refused, the sum of squares having no minimum the replays resolve (the criterion: a
/// standard error above a tenth of the factor, or none).
#[test]
fn the_belt_stiffness_is_fitted_to_a_moves_trace_and_a_tiny_move_is_refused() {
    let entry = sim_machines::find("single-axis").unwrap();
    let cfg = entry.config();
    let mut truth = cfg.clone();
    truth.set_text("belt.stiffness_ea_n = 3.5e5").unwrap();
    let trace = move_trace(entry, &truth, &[0.9, 0.1], 1300, &mut Noise(1));
    let fit = fit_move_stiffness(entry, &cfg, &trace, BELT_STIFFNESS_KEY).unwrap();
    assert!((fit.factor - 0.7).abs() < 3.0 * fit.error && fit.error < 0.03, "{fit:?}");
    // What the fitted replay leaves is the noise: 0.005 N m times sqrt(20 / 21).
    let noise = 0.005 * (20.0f64 / 21.0).sqrt();
    assert!(fit.residual < 1.05 * noise && fit.ringing > 1.4 * noise, "{fit:?}");
    assert_eq!(fit.advice().0, "fit-move-stiffness");

    // The start is at 0.1 m (`start.pose`); 2 mm out and back at 1 m/s³ and 0.2 m/s², config and machine alike.
    let gentle = |cfg: &Config| {
        let mut cfg = cfg.clone();
        cfg.set_text("axis.x.max_jerk_m_s3 = 1.0").unwrap();
        cfg.set_text("axis.x.max_acceleration_m_s2 = 0.2").unwrap();
        cfg
    };
    let tiny = move_trace(entry, &gentle(&truth), &[0.102, 0.1], 1300, &mut Noise(2));
    let refused = fit_move_stiffness(entry, &gentle(&cfg), &tiny, BELT_STIFFNESS_KEY).unwrap_err();
    assert_eq!(refused.key, "fit-move-not-excited", "{refused:?}");
    assert_eq!(MOVE_STIFFNESS_LIMIT, 0.1);
    // A config without the key is refused before anything is replayed.
    assert_eq!(fit_move_stiffness(entry, &cfg, &trace, "belt.no_such_key").unwrap_err().key, "fit-move-no-key");
}
