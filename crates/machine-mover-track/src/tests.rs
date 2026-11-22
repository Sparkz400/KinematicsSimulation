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

/// The movers' modes with the motors held (`Machine::modes`' convention), by hand as two masses on the coupling k = 2e7
/// N/m: the plate (0.2 kg) held, the body and payload (1.051 kg) ring at sqrt(k / 1.051) / 2 pi = 694.2786 Hz; with the
/// plate free sqrt(k (1 / 1.051 + 1 / 0.2)) / 2 pi = 1736.3906 Hz. The six shipped movers six times the held mode, and
/// so one mover alone, its resonance (what the shaper follows) and its train (what the plan keeps back, the load's share
/// 1.051 / 1.251): until 21 November one mover alone reported the free mode, its train the held one.
#[test]
fn the_movers_modes_are_the_held_two_mass_mode() {
    let (k, body, plate): (f64, f64, f64) = (2e7, 1.051, 0.2);
    let held = (k / body).sqrt() / TAU;
    let free = (k * (1.0 / body + 1.0 / plate)).sqrt() / TAU;
    assert!((held - 694.2786).abs() < 1e-4 && (free - 1736.3906).abs() < 1e-4, "{held} {free}");
    let six = track(&[]);
    let pose = six.movers().start.clone();
    let modes = six.modes(&pose);
    assert!(modes.len() == 6 && modes.iter().all(|m| (m.hz / held - 1.0).abs() < 1e-12), "{modes:?}");
    for edits in [&[][..], &["start.pose = [0.125]"]] {
        let m = track(edits);
        let pose = m.movers().start.clone();
        let (modes, trains) = (m.modes(&pose), m.trains(&pose));
        assert!(modes.iter().all(|m| (m.hz / held - 1.0).abs() < 1e-12), "{modes:?}");
        // Its damping as the six's (all on the straights here): c w / 2k with the coupling's damper.
        assert!((modes[0].damping / six.modes(&six.movers().start)[0].damping - 1.0).abs() < 1e-12, "{modes:?}");
        assert_eq!(m.resonance_hz(&pose), Some(modes[0].hz));
        for train in trains {
            let train = train.unwrap();
            assert!((train.hz / held - 1.0).abs() < 1e-12 && (train.load_share - body / (body + plate)).abs() < 1e-12);
        }
        // Rigid couplings: no mode, one mover as six.
        let rigid = track(&[edits, &["effects.joint_elasticity = false"]].concat());
        assert!(rigid.modes(&pose).is_empty() && rigid.resonance_hz(&pose).is_none(), "{:?}", rigid.modes(&pose));
    }
}

/// What the app's mode list shows beside the held 694.28 Hz (`shapes::mode_views`' `linear`). Until 21 November it took
/// the motor-free model's eigenvalue nearest it: 101.8 Hz, damping 1, no mode but a real pole, the rollers' smoothed
/// Coulomb friction acting as a damper at rest. Its slope (0.1 + 0.005 * 300 N of attraction) / 0.002 m/s = 800 N s/m and
/// the viscous 0.5 N s/m on the whole mover (1.251 kg): lambda = -800.5 / 1.251 = -639.9 1/s, |lambda| / 2 pi =
/// 101.84 Hz, nearer the held mode's eigenvalue (4362 rad/s) than the plate-free mode at sqrt(2e7 (1 / 1.051 + 1
/// / 0.2)) = 1736.39 Hz, which the motor-free model has (damped, 1735.8 Hz). Now a real eigenvalue is no partner, and the
/// model with the rotors held (`linearised_held`) is asked too: the body on its coupling, |lambda| = sqrt(2e7 / 1.051),
/// 694.28 Hz to rounding (the friction's slope damps it, it does not move its natural frequency); one mover and six.
#[test]
fn the_mode_list_pairs_the_held_mode_with_the_held_model_not_the_friction_s_pole() {
    for edits in [&["start.pose = [0.125]"][..], &[]] {
        let cfg = config(edits);
        let views = sim_components::shapes::mode_views(&ENTRY, &cfg, &cfg.f64s("start.pose").unwrap()).unwrap();
        let held = (2e7f64 / 1.051).sqrt() / TAU;
        for view in &views {
            let linear = view.linear.unwrap();
            assert!((linear.hz / held - 1.0).abs() < 1e-6 && (view.hz / held - 1.0).abs() < 1e-12, "{linear:?}");
        }
    }
    use sim_components::shapes::{as_mode, eigen_near, linearised};
    let cfg = config(&["start.pose = [0.125]"]);
    let m = (ENTRY.build)(&cfg).unwrap();
    let (rest, a) = linearised(m.as_ref(), &[0.125]);
    let pole = -(0.1 + 0.005 * 300.0) / 0.002 - 0.5;
    let (lambda, _) = eigen_near(&a, rest.len(), [pole / 1.251, 0.0]).unwrap();
    assert!((lambda[0] / (pole / 1.251) - 1.0).abs() < 1e-3 && lambda[1].abs() < 1e-6, "{lambda:?}");
    let free = (2e7f64 * (1.0 / 1.051 + 1.0 / 0.2)).sqrt();
    let (lambda, _) = eigen_near(&a, rest.len(), [0.0, free]).unwrap();
    assert!((as_mode(lambda).hz / (free / TAU) - 1.0).abs() < 1e-3, "{lambda:?}");
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
/// each: 1.5 A and 4.5 W. Two movers on one module at 30 N each: 6 A, 18 W. (Saturation off: the force constant is linear.)
#[test]
fn the_segment_current_and_loss_match_the_hand_calculation() {
    let m = track(&["effects.torque_saturation = false"]);
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

/// The drive saturates: k_t = 10 N/A falls to 9.5 N/A at the maximum current 11 A, so the force at 11 A is
/// 10 * 11 * (1 - 0.05) = 104.5 N (F = k_t I (1 - 0.05 (I / 11)^2), the whole 0.05 lost at the
/// maximum). A mover holding 104.5 N wholly on the first module needs 11 A, not the linear 10.45 A, and loses
/// R I^2 = 1 * 121 = 121 W.
#[test]
fn the_segment_current_is_that_of_the_saturated_drive() {
    let m = track(&[]);
    let (current, loss) = m.segment_load(&[0.125, 1.0, 1.5, 2.0, 2.2, 2.6], &[104.5, 0.0, 0.0, 0.0, 0.0, 0.0]);
    assert!((current[0] - 11.0).abs() < 1e-9 && (loss[0] - 121.0).abs() < 1e-7, "{current:?} {loss:?}");
}

/// Two movers standing on the first module, each holding 30 N (3 A, 9 W): 18 W into 2 K/W settles at 36 K. Over one
/// time constant (600 s, held loss) it rises to 36 (1 - 1/e) = 22.757 K; the settled rise, over its window of 1 s (no
/// cam running), is at 36 K. The other segments stay cold.
#[test]
fn a_segment_with_two_movers_stopping_on_it_heats_by_the_hand_calculation() {
    let sat = "effects.torque_saturation = false";
    let mut m = track(&[sat, "start.pose = [0.06, 0.18, 1.131233, 1.634349, 2.137465, 2.640582]"]);
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
    let mut other = track(&[sat, "start.pose = [0.06, 0.18, 1.131233, 1.634349, 2.137465, 2.640582]"]);
    other.set_memory(&memory);
    assert_eq!(other.rises, m.rises);
}

/// The settled rise is R_th times the loss averaged over the last pass (window 1 s here): 18 W for half a second and
/// nothing for the next half second average to 9 W, 18 K at 2 K/W (a low-pass of one pass would read about 22 K).
#[test]
fn the_settled_rise_is_the_loss_averaged_over_one_pass() {
    let mut m = track(&[
        "effects.torque_saturation = false",
        "start.pose = [0.06, 0.18, 1.131233, 1.634349, 2.137465, 2.640582]",
    ]);
    let start = m.movers().start.clone();
    let (mut state, mut torque) = (Vec::new(), vec![0.0; 6]);
    m.equilibrium(&start, &mut state, &mut torque);
    let mut rate = vec![0.0; state.len()];
    for held in [30.0, 0.0] {
        m.derivatives(&state, &[held, held, 0.0, 0.0, 0.0, 0.0], &mut rate);
        for _ in 0..5 {
            m.after_step(&mut state, 0.1);
        }
    }
    // The ring holds the window whole (its oldest bin by what is left of it).
    assert!((m.settled[0] - 18.0).abs() < 1e-6, "{:?}", m.settled);
}

/// A mass centre 35 mm outside a constant-radius curve (R = 0.1303 m, no clothoid) runs on R + e at v (1 + e / R): the
/// mass m (1 + e / R)^2 and the lateral acceleration v^2 (1 / R) (1 + e / R); e = 0 leaves both at the centre line's.
#[test]
fn a_mass_centre_outside_the_centre_line_runs_the_longer_way_round() {
    let (r, e): (f64, f64) = (0.1303, 0.035);
    let m = track(&["track.clothoid_m = 0.0", "mass.centre_offset_m = 0.035"]);
    let mv = m.movers();
    let mid = mv.track.straight + 0.5 * mv.track.curve;
    let (mut mass, mut bias) = (vec![0.0; 36], vec![0.0; 6]);
    mv.dynamics(&[mid; 6], &[1.0; 6], &mut mass, &mut bias);
    assert!((mass[0] - mv.moving() * (1.0 + e / r).powi(2)).abs() < 1e-9, "{}", mass[0]);
    assert!(bias[0].abs() < 1e-9, "no change of curvature mid-arc: {}", bias[0]);
    let lateral = mv.curve_loads(mid, 1.0, 0.0)[0];
    assert!((lateral - (1.0 / r) * (1.0 + e / r)).abs() < 1e-9, "{lateral}");
    // With the shipped clothoid, mid-arc at 2 m/s: 4 / 0.1303 * (1 + 0.035 / 0.1303) = 38.94 m/s^2 (30.70 at e = 0).
    let shipped = track(&["mass.centre_offset_m = 0.035"]);
    let s = shipped.movers();
    let lateral = s.curve_loads(s.track.straight + 0.5 * s.track.curve, 2.0, 0.0)[0];
    assert!((lateral - 38.94).abs() < 0.01, "{lateral}");
}

/// The frame momentum (effects.base_compliance) by hand. A mover on the second straight at 1.5 m/s runs along -x at the
/// track's width d = 0.267 m from the first: the whole mover, 1.251 kg with its plate, -1.8765 N s along x and
/// d * 1.8765 N m s about z. In the arc's middle (no clothoid, R = 0.1303 m) with its mass centre e = 35 mm outside the
/// centre line its body and payload (1.051 kg) run on R + e at v (1 + e / R) = 1.5 * 1.268611 m/s, the plate (0.2 kg)
/// on the centre line at v, both square to the radius: about the arc's centre 1.051 * 1.5 * (R + e)^2 / R + 0.2 * 1.5 R
/// = 0.36968 N m s (0.33059 + 0.03909), as m v r of a point running round a circle. The others stand.
#[test]
fn the_frame_momentum_is_the_hand_calculated_one() {
    let (v, spin) = (1.5, 0.2 * 1.5);
    let m = track(&[]);
    let mv = m.movers();
    let mut q = mv.start.clone();
    q[0] = mv.track.straight + mv.track.curve + 0.5;
    let mut qd = vec![0.0; q.len()];
    qd[0] = v;
    let mut rotor = vec![0.0; q.len()];
    rotor[0] = spin;
    let f = mv.frame_momentum(&q, &qd, &rotor).unwrap();
    let d = mv.track.width;
    assert!((d - 0.267).abs() < 5e-4);
    assert!((f.linear[0] + 1.8765).abs() < 1e-12 && f.linear[1].abs() < 1e-12, "{:?}", f.linear);
    assert!((f.angular[2] - d * 1.8765).abs() < 1e-12, "{:?}", f.angular);
    let (r, e) = (0.1303, 0.035);
    let arc = track(&["track.clothoid_m = 0.0", "mass.centre_offset_m = 0.035"]);
    let mv = arc.movers();
    let mid = mv.track.straight + 0.5 * mv.track.curve;
    q[0] = mid;
    let f = mv.frame_momentum(&q, &qd, &rotor).unwrap();
    // The arc's centre: the curve's apex less R across it.
    let apex = mv.track.point(mid);
    let centre = [apex[0] - r, apex[1]];
    let about = f.angular[2] - (centre[0] * f.linear[1] - centre[1] * f.linear[0]);
    let hand = 1.051 * v * (r + e) * (r + e) / r + 0.2 * v * r;
    assert!((hand - 0.36968).abs() < 1e-5, "{hand}");
    // The curve's table turns its chord by 1 / 2000 of pi between two points: the heading within that of the hand's.
    assert!((about / hand - 1.0).abs() < 1e-6, "{about} N m s about the arc's centre against {hand}");
    let speed = f.linear[0].hypot(f.linear[1]);
    assert!((speed / (1.051 * v * (1.0 + e / r) + 0.2 * v) - 1.0).abs() < 1e-6, "{speed}");
    // Its first moment moves at its momentum with the mass centre outside, at the middle of the axes, and on the shipped
    // clothoid where the rollers straddle its start (by central differences over +-1 us at 1.5 m/s).
    for at in [&[][..], &["mass.centre_offset_m = 0.035"]] {
        sim_core::testkit::frame_momentum_moves_its_first_moment(&MoverTrack::new(&config(at)).unwrap());
    }
    let clothoid = track(&["mass.centre_offset_m = 0.035"]);
    let mv = clothoid.movers();
    for s in [mv.track.straight + 0.01, mv.track.straight + 0.5 * mv.track.clothoid] {
        let (h, mut q) = (1e-6, mv.start.clone());
        let at = |s: f64, q: &mut Vec<f64>| {
            q[0] = s;
            mv.frame_momentum(q, &qd, &rotor).unwrap()
        };
        let (before, now, after) = (at(s - h * v, &mut q), at(s, &mut q), at(s + h * v, &mut q));
        for k in 0..2 {
            let moved = (after.first_moment[k] - before.first_moment[k]) / (2.0 * h);
            assert!((moved - now.linear[k]).abs() < 1e-6 * now.linear[k].abs().max(1.0), "{s}, {k}: {moved} {now:?}");
        }
    }
}

/// The loss-free configs keep their energy with the mass centre 35 mm out: the clothoid's changing mass m (1 + e k)^2
/// has its tangential force in the dynamics (`sim_core::testkit::energy_conservation`).
#[test]
fn a_mass_centre_outside_the_centre_line_keeps_the_energy() {
    for mut cfg in suite_configs() {
        cfg.set_text("mass.centre_offset_m = 0.035").unwrap();
        sim_core::testkit::energy_conservation(&MoverTrack::new(&cfg).unwrap());
    }
}

/// On the index cycle the settled rise of each segment is R_th times its copper loss averaged over the last pass, as the
/// signals give it (the old low-pass of one pass read 23 to 26 % above it: segment 1 5.15 K where the mean gives 4.18 K).
#[test]
fn the_settled_rise_on_the_index_cycle_is_the_mean_loss_over_a_pass() {
    let cfg = config(&["cam.run = true", "effects.thermal_warm_start = false"]);
    let own = MoverTrack::new(&cfg).unwrap();
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    sim.command(sim_components::Command::StartCycle).unwrap();
    let n = own.rises.len();
    let index = |what: &str| -> Vec<usize> {
        (1..=n).map(|j| sim.signal_index(&format!("segment.{j}.{what}")).unwrap()).collect()
    };
    let (loss, settled) = (index("copper_loss"), index("settled_rise"));
    let pass = (own.window / sim.dt).round() as usize;
    let (mut sum, mut values) = (vec![0.0; n], Vec::new());
    for step in 0..3 * pass {
        sim.step();
        if step >= 2 * pass {
            sim.sample(&mut values);
            sum.iter_mut().zip(&loss).for_each(|(s, &i)| *s += values[i]);
        }
    }
    let mean: Vec<f64> = sum.iter().map(|s| own.thermal_resistance * s / pass as f64).collect();
    let rise: Vec<f64> = settled.iter().map(|&i| values[i]).collect();
    println!("settled rise {rise:?} K, R_th times the mean loss {mean:?} K");
    let top = mean.iter().fold(0.0f64, |a, b| a.max(*b));
    for (a, b) in rise.iter().zip(&mean) {
        assert!((a - b).abs() < 0.02 * top, "{rise:?} against {mean:?}");
    }
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
