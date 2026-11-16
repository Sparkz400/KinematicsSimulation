use super::*;
use sim_components::Sim;
use sim_components::analysis::{analyse_cycle_with, verdicts};

/// The shipped config with edits.
fn config(edits: &[&str]) -> Config {
    let mut cfg = ENTRY.config();
    for edit in edits {
        cfg.set_text(edit).unwrap();
    }
    cfg
}

fn track(edits: &[&str]) -> MoverTrack {
    MoverTrack::new(&config(edits)).unwrap()
}

/// The cam cycle judged (cam.run on); `edits` on top of the config, the warm start off (the windings from cold).
fn judged(base: &str, edits: &[&str]) -> Vec<sim_components::analysis::Verdict> {
    let mut cfg = Config::parse(base).unwrap();
    for edit in [&["effects.thermal_warm_start = false", "cam.run = true"], edits].concat() {
        cfg.set_text(edit).unwrap();
    }
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    let load = analyse_cycle_with(&mut sim, false).unwrap();
    verdicts(&cfg, &load).unwrap()
}

#[test]
fn passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ENTRY);
}

/// The 180 degree curve: arc R = 0.1303 m, a clothoid of 0.1 m each end. Its length pi R + Lc = 0.4093495 + 0.1 =
/// 0.5093495 m, the loop 2 (1 + 0.5093495) = 3.018699 m. A clothoid shifts the arc out by about Lc^2 / (24 R) = 0.003198 m,
/// so the straights are 2 (R + 0.003198) = 0.26700 m apart: Beckhoff's 267 mm. The loop closes on itself, its heading
/// turns by 2 pi per loop and is continuous, and its segments are 4 modules a straight and one per curve.
#[test]
fn the_track_matches_the_hand_calculation_and_closes() {
    let m = track(&[]);
    let t = &m.movers().track;
    assert!((t.curve - 0.5093495).abs() < 1e-6 && (t.length - 3.018699).abs() < 1e-6, "{} {}", t.curve, t.length);
    assert!((t.width - 0.267).abs() < 5e-4, "{}", t.width);
    let (start, end) = (t.point(0.0), t.point(t.length - 1e-9));
    assert!((start[0] - end[0]).hypot(start[1] - end[1]) < 1e-6, "{start:?} {end:?}");
    assert!((t.heading(t.length) - TAU).abs() < 1e-12);
    let steps = 30000;
    for i in 0..steps {
        let (a, b) = (t.length * i as f64 / steps as f64, t.length * (i + 1) as f64 / steps as f64);
        assert!((t.heading(b) - t.heading(a)).abs() < 1e-3, "heading jumps at {a}");
        let (p, q) = (t.point(a), t.point(b));
        assert!(((p[0] - q[0]).hypot(p[1] - q[1]) - (b - a)).abs() < 1e-5, "the centre line stretches at {a}");
    }
    assert_eq!(t.segments.len(), 10);
    assert!(t.is_curve(4) && t.is_curve(9) && !t.is_curve(0) && !t.is_curve(5));
}

/// A mover in the middle of the arc at 2 m/s: its whole wheelbase on the arc, k = 1 / 0.1303. Whole mover 0.451 + 0.6 +
/// 0.2 = 1.251 kg; lateral acceleration 4 / 0.1303 = 30.6984 m/s^2, centrifugal force 1.251 * 30.6984 = 38.4037 N; the
/// rollers carry 300 - 38.4037 = 261.5963 N across and 1.251 * 9.80665 = 12.2681 N of weight: guide load
/// sqrt(261.5963^2 + 12.2681^2) = 261.8838 N, lift-off ratio 38.4037 / 300 = 0.128012. On a straight, 300 N and 12.268 N:
/// 300.251 N.
#[test]
fn the_guide_load_in_the_curve_matches_the_hand_calculation() {
    let m = track(&[]);
    let mv = m.movers();
    let mid = mv.track.straight + 0.5 * mv.track.curve;
    let [lateral, jerk, guide, lift] = mv.curve_loads(mid, 2.0, 0.0);
    assert!((lateral - 30.698).abs() < 1e-3, "{lateral}");
    assert!(jerk.abs() < 1e-9, "{jerk}");
    assert!((guide - 261.8838).abs() < 1e-3, "{guide}");
    assert!((lift - 0.128012).abs() < 1e-5, "{lift}");
    let [_, _, straight, _] = mv.curve_loads(0.5, 2.0, 0.0);
    assert!((straight - 300.251).abs() < 1e-3, "{straight}");
}

/// The lateral jerk entering a curve at a steady 2 m/s, v^3 dk/ds: with a constant radius the curvature steps to 1 / R
/// over the rollers' 0.05 m, 8 / (0.1303 * 0.05) = 1227.9 m/s^3; with the 0.1 m clothoid it rises over the clothoid, 8 /
/// (0.1303 * 0.1) = 614.0 m/s^3 (the wheelbase inside it). The clothoid halves it here.
#[test]
fn a_clothoid_takes_the_lateral_jerk_down_by_the_hand_calculation() {
    let circle = track(&["track.clothoid_m = 0.0"]);
    let c = circle.movers();
    let [_, jerk, ..] = c.curve_loads(c.track.straight, 2.0, 0.0);
    assert!((jerk - 1227.9).abs() < 0.1, "{jerk}");
    let clothoid = track(&[]);
    let k = clothoid.movers();
    let [_, jerk, ..] = k.curve_loads(k.track.straight + 0.05, 2.0, 0.0);
    assert!((jerk - 614.0).abs() < 0.1, "{jerk}");
}

/// The force at a current: k_t = 10 N/A, so 30 N is 3 A and R I^2 = 1 * 9 = 9 W. A mover in the middle of the first
/// module (0.125 m, its 0.06 m plate wholly on it) puts both on segment 1; one at the module's end (0.25 m) half on
/// each: 1.5 A and 4.5 W. Two movers on one module at 30 N each: 6 A, 18 W.
#[test]
fn the_segment_current_and_loss_match_the_hand_calculation() {
    let m = track(&[]);
    let (current, loss) = m.segment_load(&[0.125, 1.0, 1.5, 2.0, 2.2, 2.6], &[30.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    assert!((current[0] - 3.0).abs() < 1e-12 && (loss[0] - 9.0).abs() < 1e-12, "{current:?} {loss:?}");
    assert!(current[1..].iter().all(|c| *c == 0.0));
    let (current, loss) = m.segment_load(&[0.25, 1.0, 1.5, 2.0, 2.2, 2.6], &[-30.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
    assert!((current[0] - 1.5).abs() < 1e-12 && (current[1] - 1.5).abs() < 1e-12, "{current:?}");
    assert!((loss[0] - 4.5).abs() < 1e-12 && (loss[1] - 4.5).abs() < 1e-12, "{loss:?}");
    // On the loop's next turn the same.
    let l = m.movers().track.length;
    let (current, _) = m.segment_load(&[0.125 + l, 0.18 + l, 1.5, 2.0, 2.2, 2.6], &[30.0, 30.0, 0.0, 0.0, 0.0, 0.0]);
    assert!((current[0] - 6.0).abs() < 1e-12, "{current:?}");
}

/// Two movers standing on the first module, each holding 30 N (3 A, 9 W): 18 W into 2 K/W settles at 36 K. Over one
/// time constant (600 s, held loss) it rises to 36 (1 - 1/e) = 22.757 K; the settled rise, over its window of 1 s (no
/// cam running), is at 36 K. The other segments stay cold.
#[test]
fn a_segment_with_two_movers_stopping_on_it_heats_by_the_hand_calculation() {
    let mut m = track(&["start.pose = [0.06, 0.18, 1.131233, 1.634349, 2.137465, 2.640582]"]);
    let start = m.movers().start.clone();
    let (mut state, mut torque) = (Vec::new(), vec![0.0; 6]);
    m.equilibrium(&start, &mut state, &mut torque);
    let mut rate = vec![0.0; state.len()];
    m.derivatives(&state, &[30.0, 30.0, 0.0, 0.0, 0.0, 0.0], &mut rate);
    m.after_step(&mut state, 600.0);
    assert!((m.rises[0] - 22.757).abs() < 1e-3, "{:?}", m.rises);
    assert!((m.settled[0] - 36.0).abs() < 1e-9, "{:?}", m.settled);
    assert!(m.rises[1..].iter().all(|r| *r == 0.0));
    // And it comes back with a snapshot.
    let memory = m.memory();
    let mut other = track(&["start.pose = [0.06, 0.18, 1.131233, 1.634349, 2.137465, 2.640582]"]);
    other.set_memory(&memory);
    assert_eq!(other.rises, m.rises);
}

/// The index cam moves every mover one station pitch (the loop over six, 0.503116 m) per period and stands it still for
/// the first 0.4 of it; the flying cam runs a mover at the conveyor's speed (slope 1) over the process and goes once round
/// in six product pitches.
#[test]
fn the_cams_index_and_fly_as_configured() {
    let m = track(&["cam.run = true"]);
    let mv = m.movers();
    let p = mv.track.length / 6.0;
    let mut out = vec![[0.0; 4]; 6];
    m.cam_at(0.2, &mut out);
    assert!(out.iter().zip(&mv.start).all(|(c, s)| (c[0] - s).abs() < 1e-12 && c[1] == 0.0));
    m.cam_at(3.0, &mut out);
    assert!(out.iter().zip(&mv.start).all(|(c, s)| (c[0] - s - 3.0 * p).abs() < 1e-9));
    let fly = MoverTrack::new(&Config::parse(FLYING_CONFIG).unwrap()).unwrap();
    let f = fly.movers();
    fly.cam_at(0.1, &mut out);
    assert!((out[0][1] - 1.0).abs() < 1e-12, "{:?}", out[0]);
    fly.cam_at(1.8, &mut out);
    assert!((out[0][0] - f.start[0] - f.track.length).abs() < 1e-9);
}

/// flying.toml is the shipped config with the flying cycle, its cam on and its start poses: each mover a product pitch of the cam
/// behind the next, mover 1 at the start of the process (0.3 m), so that each enters the process as the cam has it.
#[test]
fn the_flying_config_is_the_default_with_its_cycle_and_matching_start_poses() {
    let (default, flying) = (ENTRY.config(), Config::parse(FLYING_CONFIG).unwrap());
    let mut changed = default.clone();
    changed.set_text("track.cycle = \"flying\"").unwrap();
    changed.set_text("cam.run = true").unwrap();
    changed.set_text(&format!("start.pose = {:?}", flying.f64s("start.pose").unwrap())).unwrap();
    assert_eq!(changed, flying, "flying.toml differs from default.toml beyond its cycle and start");
    let fly = MoverTrack::new(&flying).unwrap();
    let f = fly.movers();
    let cycle = f.cycle.as_ref().unwrap();
    for k in 0..6 {
        let want = 0.3 + cycle.cam.at(k as f64 * cycle.pitch)[0];
        assert!((f.start[k] - want).abs() < 1e-6, "mover {}: {} not {want}", k + 1, f.start[k]);
    }
}

/// Every verdict of the index cycle (the shipped config with its cam on) and of the flying one passes: per mover its force and speed (the motor
/// checks), the segments' current and warmth, the rollers' load and the lift-off in the curves, the least gap, the
/// movers' error at their stations; the own checks are there.
#[test]
fn the_shipped_cycles_pass_their_verdicts() {
    for (name, text) in [("index", ENTRY.default_config), ("flying", FLYING_CONFIG)] {
        let all = judged(text, &[]);
        let failed: Vec<_> = all.iter().filter(|v| !v.ok).collect();
        assert!(failed.is_empty(), "{name}: {failed:?}");
        for check in ["mover-guide-load", "curve-holding", "mover-gap", "segment-current", "segment-temperature"] {
            assert!(all.iter().any(|v| v.check == check), "{name}: no {check}");
        }
        assert_eq!(all.iter().any(|v| v.check == "station-settling"), name == "index");
    }
}

/// Movers too close fail the gap: on the index cycle the movers keep 0.503 - 0.07 = 0.433 m; asked for more, it fails.
#[test]
fn the_gap_verdict_fails_where_the_movers_come_too_close() {
    let all = judged(ENTRY.default_config, &["ratings.min_gap_m = 0.45"]);
    let gap = all.iter().find(|v| v.check == "mover-gap").unwrap();
    assert!(!gap.ok && (gap.value - 0.45 / 0.433116).abs() < 1e-3, "{gap:?}");
}
