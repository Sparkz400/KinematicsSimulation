use super::*;
use sim_components::analysis::{analyse_cycle_with, verdicts};
use sim_components::{Command, Sim, StopCategory};
use sim_core::{Rk4, advance};

/// The shipped config with edits.
fn config(edits: &[&str]) -> Config {
    let mut cfg = ENTRY.config();
    for edit in edits {
        cfg.set_text(edit).unwrap();
    }
    cfg
}

fn winder(edits: &[&str]) -> Winder {
    Winder::new(&config(edits)).unwrap()
}

/// The values of signals `ids` in the simulation's last sample.
fn signals(sim: &Sim, ids: &[&str]) -> Vec<f64> {
    let mut sample = Vec::new();
    sim.sample(&mut sample);
    ids.iter().map(|id| sample[sim.signal_index(id).unwrap_or_else(|| panic!("{id}"))]).collect()
}

#[test]
fn passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ENTRY);
}

/// Core r_c = 45 mm, full r_f = 150 mm, web h = 50 um: the roll holds pi (r_f^2 - r_c^2) / h = pi (0.0225 - 0.002025)
/// / 5e-5 = 1286.482 m. After L = 640 m the rewind has r^2 = r_c^2 + h L / pi = 0.002025 + 0.0101859 = 0.0122109, r =
/// 0.1105030 m, and the unwind r^2 = 0.0225 - 0.0101859, r = 0.1109688 m; the rewind has turned 2 L / (r_c + r) =
/// 8231.352 rad, the unwind 2 L / (r_f + r) = 4904.800 rad, and its radius is r_c + h phi / (2 pi) = 0.1105030 m again.
#[test]
fn the_radius_after_a_wound_length_matches_the_hand_calculation() {
    let w = winder(&[]);
    assert!((w.roll.capacity() - 1286.4822).abs() < 1e-3, "{}", w.roll.capacity());
    // The axis runs over the shipped stretch of the roll, its first 6 m; one beyond the roll's end is refused.
    assert!(w.axes()[0].min == 0.0 && w.axes()[0].max == 6.0);
    assert!(Winder::new(&config(&["roll.run_from_m = 1281.0"])).is_err());
    let [ru, rr] = w.radii_at(640.0);
    assert!((rr - 0.1105030).abs() < 1e-7 && (ru - 0.1109688).abs() < 1e-7, "{ru} {rr}");
    let [pu, pr] = w.angles_at(640.0);
    assert!((pr - 8231.352).abs() < 1e-3 && (pu - 4904.800).abs() < 1e-3, "{pu} {pr}");
    assert!((w.rewind_radius(pr) - rr).abs() < 1e-12 && (w.unwind_radius(pu) - ru).abs() < 1e-12);
    assert!((w.wound_up(pr) - 640.0).abs() < 1e-9 && (w.paid_out(pu) - 640.0).abs() < 1e-9);
    // The roll's radius from its web: the same law.
    assert!((w.roll.radius(640.0) - rr).abs() < 1e-12);
    // Forward and back through the motors, at the trim's shift too.
    let mut w = w;
    w.trim_length = 0.5;
    let (mut motors, mut back) = ([0.0; 2], [0.0]);
    w.inverse(&[640.0], &mut motors);
    w.forward(&motors, &mut back);
    let paid = w.paid_out(motors[0] / 7.0);
    assert!((back[0] - 640.0).abs() < 1e-9 && (paid - 640.5).abs() < 1e-9 && motors[0] > 7.0 * pu, "{paid}");
}

/// The web on a roll, a thick-walled cylinder: J = 1/2 rho w pi (r^4 - r_c^4) plus core and shaft (0.01 kg m^2), rho w =
/// 1390 * 0.4 = 556 kg/m^2. At r = 0.1105030 m: 873.363 (1.491074e-4 - 4.100625e-6) + 0.01 = 0.1366427 kg m^2; full,
/// at 0.15 m: 873.363 (5.0625e-4 - 4.100625e-6) + 0.01 = 0.4485586 kg m^2, 36.4 kg of film. The motor (rotor 7.8 kg cm^2,
/// gearbox 1 kg cm^2) sees it over i^2 = 49: inertia ratio 0.4485586 / (49 * 8.8e-4) = 10.4026 at the full roll and
/// 0.01 / 0.04312 = 0.2319 on the core, a range of 45.
#[test]
fn the_roll_inertia_at_a_radius_matches_the_hand_calculation() {
    let w = winder(&[]);
    assert!((w.roll.inertia(0.1105030152) - 0.1366427).abs() < 1e-7, "{}", w.roll.inertia(0.1105030152));
    assert!((w.roll.inertia(0.15) - 0.4485586).abs() < 1e-7);
    assert!((w.roll.web_mass(0.15) - 556.0 * std::f64::consts::PI * 0.020475).abs() < 1e-9);
    assert!((w.inertia_ratio(0.15) - 10.4026).abs() < 1e-4 && (w.inertia_ratio(0.045) - 0.2319).abs() < 1e-4);
    assert!((w.roll_inertia(0.15) - (0.4485586 + 49.0 * 8.8e-4)).abs() < 1e-7);
}

/// The torque that holds a tension: T r / i. At L = 0 the unwind is full (0.15 m) and the rewind on its core (45 mm),
/// where the taper has the tension at its setpoint T0 = 100 N: -100 * 0.15 / 7 = -2.142857 N m on the unwind (it holds the
/// web back), 100 * 0.045 / 7 = 0.642857 N m on the rewind. At the roll's end the other way round, with the taper at
/// T0 (1 - 0.3 (1 - 0.045 / 0.15)) = 79 N: -79 * 0.045 / 7 = -0.507857 and 79 * 0.15 / 7 = 1.692857 N m.
#[test]
fn the_steady_torque_for_a_tension_through_the_ratio_matches_the_hand_calculation() {
    let w = winder(&[]);
    let cap = w.roll.capacity();
    for (l, expected) in [(0.0, [-2.142857, 0.642857]), (cap, [-0.507857, 1.692857])] {
        let (mut state, mut hold, mut ff) = (Vec::new(), [0.0; 2], [0.0; 2]);
        w.equilibrium(&[l], &mut state, &mut hold);
        w.feedforward_torque(&[l], &[0.0], &mut ff);
        for k in 0..2 {
            assert!((hold[k] - expected[k]).abs() < 1e-6 && (ff[k] - expected[k]).abs() < 1e-6, "{l}: {hold:?} {ff:?}");
        }
    }
}

/// The line speeding up by a = 2 m/s^2 at L = 640 m (rewind 0.1105030 m, unwind 0.1109688 m), at rest and at 2 m/s, with
/// the tension setpoint there 100 (1 - 0.3 (1 - 0.045 / 0.1105030)) = 82.21686 N. The roller (2e-3 kg m^2, R = 40 mm)
/// takes J_d a / R^2 = 2.5 N between the spans: T1 = 80.96686 N, T2 = 83.46686 N. Rewind: w' = (a - h w^2 / (2 pi)) / r,
/// tau = ((J + i^2 J_m) w' + T2 r) / i: at rest (0.1366427 + 0.04312) * 2 / 0.1105030 / 7 + 83.46686 * 0.1105030 / 7 =
/// 1.7824107 N m, at 2 m/s 1.7818049 N m (the growing radius slows the roll). Unwind: tau = ((J + i^2 J_m) w' - T1 r) / i
/// with w' = (a + h w^2 / (2 pi)) / r: -0.8150135 N m at rest, -0.8144080 N m at 2 m/s.
///
/// In the rigid model these torques from rest move the line at 2 m/s^2 with the dancer at rest: its own dynamics
/// against the feedforward's formula.
#[test]
fn the_torque_to_accelerate_the_line_matches_the_hand_calculation() {
    let w = winder(&[]);
    let mut ff = [0.0; 2];
    w.feedforward_torque_moving(&[640.0], &[0.0], &[2.0], &mut ff);
    assert!((ff[1] - 1.7824107).abs() < 1e-6 && (ff[0] + 0.8150135).abs() < 1e-6, "{ff:?}");
    w.feedforward_torque_moving(&[640.0], &[2.0], &[2.0], &mut ff);
    assert!((ff[1] - 1.7818049).abs() < 1e-6 && (ff[0] + 0.8144080).abs() < 1e-6, "{ff:?}");
    let rigid =
        winder(&["effects.web_stretch = false", "effects.dancer_friction = false", "effects.motor_friction = false"]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    rigid.equilibrium(&[640.0], &mut state, &mut hold);
    rigid.feedforward_torque_moving(&[640.0], &[0.0], &[2.0], &mut ff);
    let e = rigid.eval(&state, &ff);
    let line = e.r[1] * e.acc[1];
    assert!((line - 2.0).abs() < 1e-9 && (e.r[0] * e.acc[0] - 2.0).abs() < 1e-9 && e.ydd.abs() < 1e-9, "{e:?}");
    assert!((e.t[0] - 80.96686).abs() < 1e-5 && (e.t[1] - 83.46686).abs() < 1e-5, "{:?}", e.t);
}

/// A span's steady tension for a speed difference: the unwind gives off unstretched web (wound at no tension) at
/// v1 = 1 m/s, roller and rewind take it at v2 = 1.0005 m/s. Steady, what enters a span leaves it: v1 / (1 + 0) =
/// v2 / (1 + eps), eps = v2 / v1 - 1 = 5e-4, T = EA eps = 80 kN * 5e-4 = 40.000 N in both spans (the second takes it at
/// the strain it comes with). The span equation to first order, 0 = EA (v2 - v1) + v1 T1 - v2 T2 with T1 = 0, gives
/// EA (v2 - v1) / v2 = 39.980 N. The spans fill with the time constant L / v = 1 s, the second behind the first (t e^-t);
/// after 25 s both are within 1e-6 N.
#[test]
fn a_span_settles_at_the_tension_of_its_speed_difference() {
    let w = winder(&["roll.wound_tension_n = 0.0"]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    w.equilibrium(&[640.0], &mut state, &mut hold);
    let e = w.eval(&state, &hold);
    // The speeds held, the web in the spans let free; unstretched to begin with.
    state[3] = 1.0 / e.r[0];
    state[4] = 1.0005 / e.r[1];
    state[6] = 1.0005 / w.roller_radius;
    state[7] = w.spans[0];
    state[8] = w.spans[1];
    let mut rk = Rk4::new(2);
    let mut material = [state[7], state[8]];
    for _ in 0..25_000 {
        rk.step(
            |m, rate| {
                let mut s = state.clone();
                (s[7], s[8]) = (m[0], m[1]);
                rate.copy_from_slice(&w.eval(&s, &hold).material_rate);
            },
            &mut material,
            1e-3,
        );
    }
    (state[7], state[8]) = (material[0], material[1]);
    let t = w.eval(&state, &hold).t;
    assert!((t[0] - 40.0).abs() < 1e-6 && (t[1] - 40.0).abs() < 1e-6, "{t:?}");
}

/// The dancer's static position for a tension and a load: the cylinder set for 120 N (its force 2 * 120 N less the
/// dancer's weight) and the rolls held at the torques of 100 N (T r / i). The rolls turn until the web pulls them with
/// 100 N; the dancer then stands where its load less the air spring is twice that: 2 T = 240 N - k_air y, y = (240 - 200)
/// / 1000 N/m = 40 mm down, short of its stops at 50 mm. The dancer's position measures the tension. One step finer: each
/// roll gives the 40 mm of web the span gains, its radius shrinks by h s / (2 pi r) = 50e-6 * 0.04 / (2 pi 0.1105) =
/// 2.88 um, so the torques hold T = 100 r0 / (r0 - 2.88e-6) = 100.0026 N and the dancer stands at (240 - 200.0052) / 1000
/// = 39.9948 mm. Without the
/// motors' and the slide's Coulomb friction (their band would hold it anywhere within it), the slide's viscous friction
/// 500 N s/m against the dancer and the rolls (35 kg at the web) on the air spring settles it.
#[test]
fn the_dancer_settles_where_the_web_carries_its_load() {
    let after = winder(&[
        "effects.tension_taper = false",
        "effects.motor_friction = false",
        "effects.gearbox_efficiency = false",
        "dancer.friction_coulomb_n = 0.0",
        "dancer.friction_viscous_n_s_m = 500.0",
        "web.tension_n = 120.0",
    ]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
    after.equilibrium(&[640.0], &mut state, &mut hold);
    let r = after.radii_at(640.0);
    let torque = [-100.0 * r[0] / 7.0, 100.0 * r[1] / 7.0];
    let mut rk = Rk4::new(state.len());
    for _ in 0..(15.0 / 62.5e-6) as usize {
        advance(&after, &mut rk, &mut state, &torque, 62.5e-6, 4);
    }
    let e = after.eval(&state, &torque);
    assert!((e.y - 0.0399948).abs() < 2e-7 && e.vy.abs() < 1e-9, "{} {} {:?} {:?}", e.y, e.vy, e.t, e.r);
    assert!((e.t[0] - 100.0026).abs() < 2e-4 && (e.t[1] - 100.0026).abs() < 2e-4, "{:?}", e.t);
}

/// The dancer on the web's stiffness, the rolls held: k = EA / L1 + EA / L2 + k_air = 80 kN/m * 2 + 1 kN/m = 161 kN/m on
/// 4 kg of dancer and 2 kg of weight: f = sqrt(161000 / 6) / (2 pi) = 26.071 Hz; the roller turning on the spans,
/// R^2 * 160 kN/m on 2e-3 kg m^2: f = sqrt(0.0016 * 160000 / 0.002) / (2 pi) = 56.941 Hz. Damped by the web's
/// 2e-4 s (32 N s/m over both spans) and the slide's 20 N s/m: 52 / (2 * 6 * 163.81) = 0.0265.
#[test]
fn the_dancer_frequency_on_the_web_stiffness_matches_the_hand_calculation() {
    let w = winder(&[]);
    let modes = w.modes(&[640.0]);
    assert!((modes[0].hz - 26.0710).abs() < 1e-4 && (modes[1].hz - 56.9410).abs() < 1e-4, "{modes:?}");
    assert!((modes[0].damping - 0.026454).abs() < 1e-5, "{modes:?}");
    assert!(winder(&["effects.web_stretch = false"]).modes(&[640.0]).is_empty());
    assert_eq!(w.resonance_hz(&[640.0]), Some(modes[0].hz));
}

/// Conservation of web length over the shipped move forward (3 m): what leaves the unwind, its paid-out length over
/// 1 + eps_0 (the strain it was wound at), arrives at the rewind (the web it takes up, summed over the run) plus what the
/// spans hold more. Summed by the trapezoid over 62.5 us steps of a smooth flow: within 1e-7 m.
#[test]
fn the_web_that_leaves_the_unwind_arrives_at_the_rewind_or_stays_in_the_spans() {
    let cfg = config(&[]);
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    let w = Winder::new(&cfg).unwrap();
    let start = sim.state.clone();
    sim.command(Command::MoveTo(vec![5.0])).unwrap();
    let flow = |sim: &Sim| w.eval(&sim.state, &[0.0, 0.0]).flow;
    let (mut taken, mut last) = (0.0, flow(&sim)[2]);
    for _ in 0..(3.5 / sim.dt) as usize {
        sim.step();
        let now = flow(&sim)[2];
        taken += 0.5 * (now + last) * sim.dt;
        last = now;
    }
    let paid = (w.paid_out(sim.state[0]) - w.paid_out(start[0])) / (1.0 + w.wound_strain);
    let stored = sim.state[7] + sim.state[8] - start[7] - start[8];
    assert!(paid > 2.99 && (paid - taken - stored).abs() < 1e-7, "{paid} {taken} {stored}");
}

/// Round trip and static hold have a meaning at a radius: at both ends of the roll (L = 0, unwind full and rewind on its
/// core; and the other way round) and at a quarter, the shipped model at rest under its holding torques stays put for a
/// second: the line within 1e-7 m, the dancer within 1e-9 m.
#[test]
fn the_winder_holds_at_rest_at_both_ends_of_the_roll() {
    let cfg = config(&[]);
    let w = Winder::new(&cfg).unwrap();
    let cap = w.roll.capacity();
    for l in [0.0, 0.25 * cap, cap] {
        let (mut state, mut hold) = (Vec::new(), [0.0; 2]);
        w.equilibrium(&[l], &mut state, &mut hold);
        let mut rk = Rk4::new(state.len());
        for _ in 0..16_000 {
            advance(&w, &mut rk, &mut state, &hold, 62.5e-6, 4);
        }
        let (mut pos, mut vel) = ([0.0], [0.0]);
        w.tool_state(&state, &mut pos, &mut vel);
        assert!((pos[0] - l).abs() < 1e-7 && state[2].abs() < 1e-9, "{l}: {pos:?} {}", state[2]);
        let (mut motors, mut back) = ([0.0; 2], [0.0]);
        w.inverse(&[l], &mut motors);
        w.forward(&motors, &mut back);
        assert!((back[0] - l).abs() < 1e-9);
    }
}

/// The dancer's position loop takes out the draw. The unwind was wound at 100 N (eps_0 = 1.25e-3), the web runs at 80 N
/// (eps = 1e-3, taper off): with both rolls geared to the same surface speed v the spans lose web at v (1 / (1 + eps_0) -
/// 1 / (1 + eps)) and the dancer rises at y' = v / 2 (eps - eps_0) / (1 + eps_0) = -0.24969 mm/s at 2 m/s. With the
/// loop the unwind pays out faster by what makes up for it, 2 |y'| = 0.49938 mm/s, and the dancer stays in the middle.
/// Without the air spring, whose k y would raise the tension as the dancer rises, and without the roller's drag, which
/// would part the two spans' tensions by its torque over R.
#[test]
fn the_dancer_loop_takes_out_the_draw_of_a_roll_wound_at_another_tension() {
    let run = |position_loop: bool| {
        let cfg = config(&[
            "effects.tension_taper = false",
            "effects.dancer_friction = false",
            "dancer.air_spring_n_m = 0.0",
            "web.tension_n = 80.0",
            "roll.run_m = 40.0",
            &format!("dancer.position_loop = {position_loop}"),
        ]);
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        sim.command(Command::MoveTo(vec![32.0])).unwrap();
        // Up to speed by 1.2 s; cruising until 15 s.
        sim.run(5.0, |_| {});
        let before = signals(&sim, &["dancer.position"])[0];
        sim.run(5.0, |_| {});
        let [after, trim] = signals(&sim, &["dancer.position", "dancer.trim_speed"])[..] else { unreachable!() };
        ((after - before) / 5.0, after, trim)
    };
    let (rate, _, _) = run(false);
    assert!((rate + 0.24969e-3).abs() < 0.01 * 0.24969e-3, "{rate}");
    let (rate, at, trim) = run(true);
    assert!(
        rate.abs() < 1e-6 && at.abs() < 1e-5 && (trim - 0.49938e-3).abs() < 0.01 * 0.49938e-3,
        "{rate} {at} {trim}"
    );
}

/// The own verdicts on the shipped cycle: the web's highest tension against what it may carry, its lowest against the
/// minimum, the dancer's travel; per motor the inertia ratio at the full roll (10.4026), the speed at the core at the
/// line's 2 m/s (7 * 2 / 0.045 = 311.11 rad/s), and the tension's torque at the full roll. The unwind is full while the
/// rewind is on its core, where the taper leaves the setpoint at 100 N: 100 * 0.15 / 7 = 2.1429 N m; the rewind is full
/// at 79 N: 1.6929 N m, each plus what the span's tension is off its setpoint now, at most the roller's J_d a / (2 R^2) =
/// 2.5 N at the line's 4 m/s^2 and its drag: (100 + 2.5) * 0.15 / 7 = 2.1964 N m and (79 + 2.5) * 0.15 / 7 = 1.7464 N m
/// and a little more. Each passes, and fails when its rating is set just below the value.
#[test]
fn the_own_verdicts_judge_the_cycle_and_fail_at_their_limits() {
    let judged = |edits: &[&str]| {
        let cfg = config(&[&["effects.thermal_warm_start = false"], edits].concat());
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        let load = analyse_cycle_with(&mut sim, false).unwrap();
        verdicts(&cfg, &load).unwrap()
    };
    let all = judged(&[]);
    assert!(all.iter().all(|v| v.ok), "{all:?}");
    let value = |check: &str, motor: Option<usize>| {
        all.iter().find(|v| v.check == check && v.motor == motor).unwrap_or_else(|| panic!("{check}")).value
    };
    assert!((value("inertia-ratio", Some(0)) - 10.4026).abs() < 1e-4);
    let speed = value("speed-at-core", Some(1));
    assert!((speed - 311.11).abs() < 0.05, "{speed}");
    let (unwind, rewind) =
        (value("tension-torque-at-full-roll", Some(0)), value("tension-torque-at-full-roll", Some(1)));
    assert!((unwind - 2.1964).abs() < 0.01 && (rewind - 1.7464).abs() < 0.01, "{unwind} {rewind}");
    // At the roll's start the setpoint is 100 N, the roller's 2.5 N and its drag on top.
    let tension = value("web-tension", None);
    assert!(tension > 102.5 && tension < 104.0, "{tension}");
    let below = judged(&[
        &format!("ratings.web_max_tension_n = {:?}", tension - 0.01),
        "ratings.max_inertia_ratio = 10.40",
        &format!("drive.max_speed_rad_s = {:?}", speed - 0.01),
        &format!("ratings.dancer_max_travel_m = {:?}", 0.99 * value("dancer-travel", None)),
        &format!("ratings.web_min_tension_n = {:?}", 1.01 * 30.0 / value("web-slack", None)),
    ]);
    for check in ["web-tension", "inertia-ratio", "speed-at-core", "dancer-travel", "web-slack"] {
        assert!(below.iter().any(|v| v.check == check && !v.ok), "{check}: {below:?}");
    }
}

/// A stop of `category` from full line speed 1.5 s into a 10 m run from the line position `start` (backwards near the
/// roll's end), the axis over the 20 m of the roll about it: the spans' lowest and highest tension, the dancer's largest
/// travel over the next 6 s, when the line stood (when the dancer met its stop where it did), and how long a span was
/// slack.
fn stop_from_full_speed(start: f64, category: StopCategory) -> (f64, f64, f64, f64, f64) {
    let mut cfg = config(&[]);
    on_the_roll(&mut cfg, if start > 1000.0 { start - 15.0 } else { (start - 5.0).max(0.0) });
    cfg.set_text(&format!("start.pose = [{start:?}]")).unwrap();
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    sim.command(Command::MoveTo(vec![start + if start > 1000.0 { -10.0 } else { 10.0 }])).unwrap();
    sim.run(1.5, |_| {});
    assert!(signals(&sim, &["axis.web.velocity"])[0].abs() > 1.99);
    sim.command(Command::Halt(category)).unwrap();
    let ids = ["axis.web.velocity", "web.span.1.tension", "web.span.2.tension", "dancer.position"];
    let (mut low, mut high, mut travel, mut stood) = (f64::INFINITY, 0f64, 0f64, f64::NAN);
    for step in 1..=6000 {
        sim.run(1e-3, |_| {});
        let v = signals(&sim, &ids);
        (low, high, travel) = (low.min(v[1]).min(v[2]), high.max(v[1]).max(v[2]), travel.max(v[3].abs()));
        if stood.is_nan() && (v[0].abs() < 1e-3 || v[3].abs() > 0.05) {
            stood = step as f64 * 1e-3;
        }
    }
    (low, high, travel, stood, signals(&sim, &["web.slack_time"])[0])
}

/// An emergency stop from 2 m/s that brakes the line along its path (category 2, the drives' setpoints run down by the
/// axis's 4 m/s^2 in 0.5 s, torque on) keeps the web between its 30 N and 250 N and the dancer within its 40 mm, at both
/// ends of the roll and in the middle; one in which each drive brakes on its own OFF3 ramp (category 1) does so in the
/// middle of the roll, where the radii are equal (0.1105 and 0.1110 m: both surfaces slow by 62.83 rad/s^2 / 7 * r =
/// 0.99 m/s^2 and stand after about 2 s), the holding brakes then keeping the tension.
#[test]
fn an_emergency_stop_along_the_line_keeps_the_web_within_its_limits() {
    for (start, category) in
        [(5.0, StopCategory::Two), (640.0, StopCategory::Two), (1270.0, StopCategory::Two), (640.0, StopCategory::One)]
    {
        let (low, high, travel, stood, _) = stop_from_full_speed(start, category);
        println!("{start} {category:?}: {low:.2} to {high:.2} N, dancer {travel:.5} m, stood after {stood} s");
        assert!(low > 30.0 && high < 250.0 && travel < 0.04, "{start} {category:?}: {low} {high} {travel}");
        let time = if category == StopCategory::Two { 0.5 } else { 2.0 };
        assert!((stood - time).abs() < 0.05, "{start} {category:?}: stood after {stood} s");
    }
}

/// What the other stops do to the web. Category 1 at the roll's start: each drive brakes its motor by the same OFF3 ramp,
/// 314.16 rad/s in 5 s = 62.83 rad/s^2, 8.976 rad/s^2 at the rolls; the full unwind's surface (0.15 m) slows by 1.346
/// m/s^2, the rewind's on its core (45 mm) by 0.404 m/s^2. They part at 0.942 m/s^2, the dancer rises at half that and
/// meets its stop at 50 mm after sqrt(2 * 0.05 / 0.471) = 0.46 s, and the smaller roll, which would stand only after
/// 2 m/s / (8.976 rad/s^2 r) = 4.8 s, stretches the web far beyond its 250 N. Category 0 in the middle: the torque goes off and the holding brakes take the
/// rolls (10 N m at the motor, 70 N m at the roll) within some 50 ms, the dancer's roller runs on and the web goes slack.
#[test]
fn a_stop_on_each_drives_own_ramp_overloads_the_web_where_the_radii_differ() {
    for start in [5.0, 1270.0] {
        let (_, high, travel, met, _) = stop_from_full_speed(start, StopCategory::One);
        assert!(high > 250.0 && travel > 0.05 && (met - 0.46).abs() < 0.05, "{start}: {high} {travel} {met}");
    }
    let (low, _, _, stood, slack) = stop_from_full_speed(640.0, StopCategory::Zero);
    assert!(low < 1.0 && slack > 0.0 && stood < 0.1, "{low} {slack} {stood}");
}
