//! Regression tests for the defects of the third review, each with the scenario the review found it in.
//! They run on every registered machine.

use sim_components::path::Piece;
use sim_components::{Command, Sim};
use sim_machines::MACHINES;

// ---------------------------------------------------------------------------------------------
// A path from an end of a stroke

/// A machine that stands on the end of a stroke has its setpoint up to 44 um beyond it (the setpoint is where
/// the motors stand, and belt stretch and gear twist put them there). A path planned from there starts that
/// far outside the travel range, and the controller refused it with `error-trajectory`, from every corner of
/// the gantry's workspace. A point-to-point move from the same pose always ran. Now the path runs too; a
/// path that leaves the range by a millimetre is refused as before.
#[test]
fn a_path_from_an_end_of_a_stroke_is_run() {
    for entry in MACHINES.iter().copied() {
        let axes = Sim::new(entry, &entry.config()).unwrap().machine.axes().to_vec();
        let middle: Vec<f64> = axes.iter().map(|a| 0.5 * (a.min + a.max)).collect();
        let mut beyond_range = 0;
        for corner in 0..1 << axes.len() {
            let pose: Vec<f64> = axes.iter().enumerate().map(|(i, a)| if corner >> i & 1 == 1 { a.max } else { a.min }).collect();
            let mut cfg = entry.config();
            cfg.set_text(&format!("start.pose = {pose:?}")).unwrap();
            let mut sim = Sim::new(entry, &cfg).unwrap();
            beyond_range += sim.control.set_pos.iter().zip(&axes).filter(|(p, a)| **p < a.min || **p > a.max).count();
            let path = sim.plan_path(&[Piece::Line(middle.clone())], false).unwrap();
            let duration = path[path.len() - 1].0;
            assert_eq!(sim.command(Command::Trajectory(path)).err().map(|e| e.key), None, "{}: path from {pose:?}", entry.id);
            sim.run(duration + 0.3, |_| {});
            let (mut tool, mut speed) = (middle.clone(), middle.clone());
            sim.machine.tool_state(&sim.state, &mut tool, &mut speed);
            assert!(tool.iter().zip(&middle).all(|(t, m)| (t - m).abs() < 1e-3), "{}: from {pose:?} the tool arrives at {tool:?}", entry.id);

            let mut sim = Sim::new(entry, &cfg).unwrap();
            let outside: Vec<f64> = pose.iter().zip(&axes).map(|(p, a)| if *p == a.max { p + 1e-3 } else { p - 1e-3 }).collect();
            let leaves = vec![(0.0, sim.control.set_pos.clone()), (0.5, outside), (1.0, pose.clone())];
            assert_eq!(sim.command(Command::Trajectory(leaves)).err().map(|e| e.key), Some("error-trajectory"), "{}", entry.id);
        }
        // The scenario is there: some setpoint did start beyond its range.
        assert!(beyond_range > 0, "{}: no setpoint starts beyond its range", entry.id);
    }
}

// ---------------------------------------------------------------------------------------------
// A stream of setpoints near the speed limit

/// A PLC that streams one setpoint per setpoint interval, each further by what the axis travels at its speed
/// limit in that time. Two things were wrong. The way to a setpoint was counted from the profile cycle after
/// it came, so it ended a cycle late: the next setpoint found the plan a tenth short, the plan stood still
/// for a cycle and then ran a ninth faster than the stream. And the speed guard measured from where the plan
/// stood, so from 10/11 of the speed limit on it refused every setpoint after the first (39 of 40).
/// Now the stream is followed at its own speed, to the last bit, also when the setpoints come a cycle and a
/// half early and late in turn; a setpoint that jumps is refused as before.
#[test]
fn a_stream_of_setpoints_at_the_speed_limit_is_followed() {
    for entry in MACHINES.iter().copied() {
        let cfg = entry.config();
        let interval = cfg.f64("profile.setpoint_interval_s").unwrap();
        for (share, jitter) in [(1.0, 0usize), (0.95, 0), (1.0, 24), (0.5, 24)] {
            let mut sim = Sim::new(entry, &cfg).unwrap();
            let axis = sim.machine.axes()[0].clone();
            let (per, step) = ((interval / sim.dt).round() as usize, share * axis.max_vel * interval);
            let mut target = sim.control.set_pos.clone();
            let (mut fastest, mut slowest) = (0f64, f64::INFINITY);
            for k in 0..40 {
                target[0] += step;
                assert_eq!(sim.command(Command::Setpoint(target.clone())).err().map(|e| e.key), None, "{}: setpoint {k} at {share} of the speed limit, jitter {jitter}", entry.id);
                // Early and late in turn, by `jitter` base steps.
                for _ in 0..if k % 2 == 0 { per - jitter } else { per + jitter } {
                    sim.step();
                    if k > 0 && jitter == 0 {
                        (fastest, slowest) = (fastest.max(sim.control.set_vel[0]), slowest.min(sim.control.set_vel[0]));
                    }
                }
            }
            sim.run(2.0 * interval, |_| {});
            assert_eq!(sim.control.planned_pos()[0], target[0], "{}: the stream ends elsewhere", entry.id);
            if jitter == 0 {
                let speed = share * axis.max_vel;
                assert!(fastest <= speed * (1.0 + 1e-9) && slowest >= speed * (1.0 - 1e-9), "{}: a stream at {speed} m/s is followed at {slowest} to {fastest} m/s", entry.id);
            }
            // Twice as far as the axis may go in one interval is still refused.
            target[0] += 2.0 * axis.max_vel * interval;
            assert_eq!(sim.command(Command::Setpoint(target)).err().map(|e| e.key), Some("error-setpoint-too-far"), "{}", entry.id);
        }
    }
}

/// `verdicts --bands --samples N` and the app's column showed the share of runs passed rounded: one failed run
/// in 2001 and more read as 1.000 (100.0 %). The share is cut off instead, so only a check that passed every
/// run shows as 1.000; a share that is a whole number of thousandths keeps its digits.
#[test]
fn a_pass_share_below_one_never_shows_as_one() {
    use sim_components::analysis::{Band, Verdict};
    let shown = |passed: f64, runs: f64, decimals: i32| {
        let verdict = Verdict { check: "settling", motor: None, value: 0.0, limit: 1.0, ok: true };
        Band { verdict, low: 0.0, high: 0.0, ok: false, share: passed / runs }.share_shown(decimals)
    };
    // As the command line prints it, and as the app does in percent.
    assert_eq!(format!("{:.3}", shown(2000.0, 2001.0, 3)), "0.999");
    assert_eq!(format!("{:.2}", 100.0 * shown(49_999.0, 50_000.0, 4)), "99.99");
    assert_eq!(format!("{:.3}", shown(2001.0, 2001.0, 3)), "1.000");
    for passed in 0..=200 {
        let thousandths = passed * 5;
        assert_eq!(format!("{:.3}", shown(passed as f64, 200.0, 3)), format!("{}.{:03}", thousandths / 1000, thousandths % 1000), "{passed} of 200 passed");
    }
}
