use super::*;
use sim_components::analysis::{analyse_cycle, verdicts};
use sim_components::{Command, Sim};
use sim_core::{Rk4, advance};

const DT: f64 = 62.5e-6;

#[test]
fn the_overhead_crane_passes_the_standard_suite() {
    sim_core::testkit::run_standard_suite(&ENTRY);
}

fn crane(cfg: &Config) -> Crane {
    Crane::new(cfg).unwrap()
}

fn with(edits: &[&str]) -> Config {
    let mut cfg = ENTRY.config();
    for e in edits {
        cfg.set_text(e).unwrap();
    }
    cfg
}

/// The rope at rest, by hand, at the pose z = 1 m (4 m below the sheave at 5 m) with hook and payload, m = 15 + 250 =
/// 265 kg (m g = 2598.762 N). The motors stand for the pose with the empty hook's stretch, s_h = 1 + 15 g / EA =
/// 1.00010215: the drum's free length l = 4 / s_h = 3.999591 m. The payload stretches it by s = 1 + 265 g / EA =
/// 1.00180469 to L = l s = 4.006809 m, so the load hangs 6.8095 mm below the pose (within `setpoint_slack`, 4.5 m x
/// 250 g / EA + 0.1 mm = 7.762 mm), the rope stretched by e = L - l = 7.2180 mm at the tension m g. It holds V = EA e^2 /
/// (2 l) = t e / 2 = 9.3790 J. The drum feels t (1 + e / (2 l)) = 2601.107 N, at the hoist's motor 2601.107 / (40 /
/// 0.08) = 5.2022 N m, which the motor holds (with no ripple at rest it is the holding torque, pulling in).
#[test]
fn the_static_stretch_tension_and_holding_torque_are_the_hand_calculated_ones() {
    let m = crane(&with(&["effects.torque_ripple = false"]));
    let (mut state, mut hold) = (Vec::new(), [0.0; 3]);
    m.equilibrium(&[8.0, 4.0, 1.0], &mut state, &mut hold);
    let rope = m.rope(&state);
    let close = |a: f64, b: f64, tol: f64| (a - b).abs() < tol;
    assert!(close(rope.tension, 2598.762, 1e-3), "{rope:?}");
    assert!(close(state[2], 3.999591, 1e-6) && close(rope.stretch, 7.2180e-3, 1e-7), "{rope:?}");
    assert!(close(1.0 - state[LOAD + 2], 6.8095e-3, 1e-7), "{state:?}");
    assert!(close(m.energy_parts(&state)[2], 9.3790, 1e-4));
    assert!(close(rope.drum, 2601.107, 1e-3) && close(hold[2], -5.20221, 1e-5), "{hold:?}");
    assert!(hold[0] == 0.0 && hold[1] == 0.0);
    // The brake's check: that torque against 12 N m / 1.6 = 7.5 N m.
    assert!(close(m.brake_load(rope.drum), 5.20221, 1e-5) && m.brake_limit == 7.5);
    // The motors stand for the pose, the tool is where they put the hook.
    let (mut motors, mut speed, mut pose, mut rate) = ([0.0; 3], [0.0; 3], [0.0; 3], [0.0; 3]);
    m.motor_state(&state, &mut motors, &mut speed);
    m.forward(&motors, &mut pose);
    assert!(close(pose[2], 1.0, 1e-12) && pose[0] == 8.0 && pose[1] == 4.0, "{pose:?}");
    m.tool_state(&state, &mut pose, &mut rate);
    assert!(close(pose[2], 1.0, 1e-12), "{pose:?}");
    assert!(close(m.setpoint_slack(), 7.762e-3, 1e-6));
    // A rope shorter than its free length is slack: no tension, the load falls freely.
    let mut slack = state.clone();
    slack[LOAD + 2] += 0.01;
    let rope = m.rope(&slack);
    assert!(rope.tension == 0.0 && rope.drum == 0.0 && rope.stretch < 0.0);
    let mut rate = vec![0.0; STATE_LEN];
    m.derivatives(&slack, &hold, &mut rate);
    assert!((rate[LOAD_SPEED + 2] + G).abs() < 1e-12);
}

/// The period of the load's sway on its rope, small amplitude. By hand, at the pose z = 2 m the payload hangs on
/// L = 3 s / s_h = 3.005107 m of rope (s and s_h the stretch factors of 265 kg and 15 kg, see the static test):
/// T = 2 pi sqrt(L / g) = 2 pi sqrt(3.005107 / 9.80665) = 3.47816 s (an amplitude of 10 mm, 3.3 mrad, lengthens it by
/// theta^2 / 16 = 7e-7). With the trolley held by its brake that is the period; with the trolley free to roll it is a
/// two-mass pendulum, w^2 = g / L (1 + m / M): M the trolley and its motor's rotor and gearbox at the rail,
/// 120 kg + (8.4e-5 + 5e-5) kg m^2 x (25 / 0.05)^2 = 153.5 kg, so T = 3.47816 / sqrt(1 + 265 / 153.5) = 2.10647 s.
#[test]
fn the_sway_period_is_the_hand_calculated_pendulum_s() {
    let period = |braked: bool| {
        let mut m = crane(&{
            let mut cfg = lossless_config();
            cfg.set_text("effects.holding_brake = true").unwrap();
            cfg
        });
        m.set_brake_engaged(braked);
        let (mut state, mut hold) = (Vec::new(), [0.0; 3]);
        m.equilibrium(&[5.0, 2.0, 2.0], &mut state, &mut hold);
        // 10 mm aside in y, on the same rope length.
        let length = m.rope(&state).length;
        state[LOAD + 1] += 0.01;
        state[LOAD + 2] = m.height - (length * length - 1e-4).sqrt();
        let mut rk = Rk4::new(STATE_LEN);
        let (mut crossings, mut last) = (Vec::new(), state[LOAD + 1] - state[1]);
        for step in 1..=(12.0 / DT) as usize {
            advance(&m, &mut rk, &mut state, &hold, DT, 4);
            let sway = state[LOAD + 1] - state[1];
            if sway.signum() != last.signum() {
                // Between the two steps, linearly.
                crossings.push((step as f64 - sway / (sway - last)) * DT);
            }
            last = sway;
        }
        // Every second crossing is a whole period apart.
        let n = crossings.len() - 1 - (crossings.len() - 1) % 2;
        2.0 * (crossings[n] - crossings[0]) / n as f64
    };
    let (held, free) = (period(true), period(false));
    assert!((held / 3.47816 - 1.0).abs() < 2e-4, "{held} s");
    assert!((free / 2.10647 - 1.0).abs() < 2e-4, "{free} s");
}

/// The energy budget of a lift, by hand: the hoist lifts hook and payload, 265 kg (m g = 2598.762 N), from the pose
/// z = 1 m to the pose z = 3 m with every loss off. The motors count on the empty hook's stretch (s_h = 1 + 15 g / EA =
/// 1.00010215), the payload's is s = 1 + 265 g / EA = 1.00180469: the load hangs L (s / s_h - 1) below the pose, 6.8095
/// mm on 4 m and 3.4047 mm on 2 m, so it rises by 2.00340474 m and gains m g h = 5206.373 J; the rope gives back
/// t e / 2 = 9.3790 J at 4 m less 4.6895 J at 2 m: 5201.683 J in all. The motors' work, the time integral of torque times speed, is
/// that (and what still moves at the end, which the machine's energy holds), to 0.1 %.
#[test]
fn a_loss_free_lift_takes_the_hand_calculated_work() {
    let cfg = {
        let mut cfg = lossless_config();
        cfg.set_text("start.pose = [8.0, 4.0, 1.0]").unwrap();
        cfg.set_text("effects.input_shaping = false").unwrap();
        cfg
    };
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    let ids: Vec<usize> = ["torque", "speed"]
        .iter()
        .flat_map(|q| MOTOR_IDS.map(|id| sim.signal_index(&format!("motor.{id}.{q}")).unwrap()))
        .collect();
    let start = sim.machine.energy(&sim.state);
    sim.command(Command::MoveTo(vec![8.0, 4.0, 3.0])).unwrap();
    let (mut sample, mut work, mut last) = (Vec::new(), 0.0, None::<Vec<f64>>);
    let mut tally = |s: &Sim| {
        s.sample(&mut sample);
        let now: Vec<f64> = ids.iter().map(|i| sample[*i]).collect();
        if let Some(before) = &last {
            // Trapezoid of torque x speed over the step.
            work += (0..3).map(|k| 0.5 * (now[k] * now[3 + k] + before[k] * before[3 + k])).sum::<f64>() * s.dt;
        }
        last = Some(now);
    };
    while sim.control.busy() {
        sim.step();
        tally(&sim);
    }
    sim.run(2.0, &mut tally);
    let gained = sim.machine.energy(&sim.state) - start;
    assert!((gained / 5201.683 - 1.0).abs() < 1e-4, "{gained} J");
    assert!((work / gained - 1.0).abs() < 1e-3, "work {work} J, energy {gained} J");
}

/// The trolley's torque steps over its holding torque, N m at its motor (as in `reference/scipy_reference.py`).
fn reference_step(t: f64) -> f64 {
    [(0.0, 0.40), (1.5, 0.05), (3.5, -0.30), (5.0, 0.0)].iter().rev().find(|(from, _)| t >= *from).map_or(0.0, |s| s.1)
}

/// Independent reference: the sway after a trapezoidal trolley move (torque steps open loop: speed up, cruise, brake,
/// let go) matches a SciPy implementation of the same equations (`reference/scipy_reference.py`, LSODA, from the same
/// config), the load's y and its sway at every 50 ms over 10 s.
#[test]
fn a_trapezoidal_trolley_move_sways_as_scipy_says() {
    // The SciPy implementation has no torque ripple.
    let m = crane(&with(&["effects.torque_ripple = false"]));
    assert_eq!(m.drives[1].gearbox.efficiency, 0.94, "the reference takes the configured efficiency");
    let (mut state, mut hold) = (Vec::new(), [0.0; 3]);
    m.equilibrium(&[5.0, 1.0, 2.0], &mut state, &mut hold);
    let mut rk = Rk4::new(STATE_LEN);
    let mut trace = vec![(0.0, state[LOAD + 1], state[LOAD + 1] - state[1])];
    for step in 0..(10.0 / DT).round() as usize {
        let torque = [hold[0], hold[1] + reference_step(step as f64 * DT + 1e-9), hold[2]];
        advance(&m, &mut rk, &mut state, &torque, DT, 4);
        if (step + 1) % 800 == 0 {
            trace.push(((step + 1) as f64 * DT, state[LOAD + 1], state[LOAD + 1] - state[1]));
        }
    }
    let reference: Vec<Vec<f64>> = include_str!("../reference/scipy_trace.csv")
        .lines()
        .filter(|l| !l.starts_with('#') && !l.starts_with('t'))
        .map(|l| l.split(',').map(|v| v.trim().parse().unwrap()).collect())
        .collect();
    assert_eq!(reference.len(), trace.len(), "the reference trace has a different length");
    let sway = reference.iter().map(|r| r[2].abs()).fold(0.0, f64::max);
    let travel = reference.iter().map(|r| (r[1] - 1.0).abs()).fold(0.0, f64::max);
    assert!(sway > 0.2 && travel > 1.0, "the reference move is too small: {sway} m sway, {travel} m travel");
    for (ours, theirs) in trace.iter().zip(&reference) {
        assert!((ours.0 - theirs[0]).abs() < 1e-9);
        assert!((ours.1 - theirs[1]).abs() < 1e-4 * travel, "y at t={}: {} vs SciPy {}", ours.0, ours.1, theirs[1]);
        assert!((ours.2 - theirs[2]).abs() < 1e-3 * sway, "sway at t={}: {} vs SciPy {}", ours.0, ours.2, theirs[2]);
    }
}

/// The rigid rope is the elastic one's limit: on the reference move (the rope's damper off, which the rigid rope has
/// not) a rope a hundred times stiffer sways within 1e-4 of the largest sway of the rigid one, the shipped rope 5e-3 off
/// (measured 4.5e-5 and 4.75e-3): at the same pose its stretch hangs the payload 5.1 mm lower, on a longer pendulum.
#[test]
fn a_stiff_rope_converges_to_the_rigid_reference() {
    let run = |edits: &[&str]| {
        let mut all = vec!["effects.torque_ripple = false", "effects.rope_damping = false"];
        all.extend(edits);
        let m = crane(&with(&all));
        let (mut state, mut hold) = (Vec::new(), [0.0; 3]);
        m.equilibrium(&[5.0, 1.0, 2.0], &mut state, &mut hold);
        let mut rk = Rk4::new(STATE_LEN);
        let mut sway = Vec::new();
        for step in 0..(10.0 / DT).round() as usize {
            let torque = [hold[0], hold[1] + reference_step(step as f64 * DT + 1e-9), hold[2]];
            advance(&m, &mut rk, &mut state, &torque, DT, 4);
            if (step + 1) % 800 == 0 {
                sway.push(state[LOAD + 1] - state[1]);
            }
        }
        sway
    };
    let rigid = run(&["effects.rope_stretch = false"]);
    let (stiff, shipped) = (run(&["rope.ea_n = 1.44e8"]), run(&[]));
    let largest = rigid.iter().fold(0f64, |m, v| m.max(v.abs()));
    let off = |other: &[f64]| rigid.iter().zip(other).fold(0f64, |m, (a, b)| m.max((a - b).abs())) / largest;
    println!("off the rigid rope's sway: stiff {:.2e}, shipped {:.2e}", off(&stiff), off(&shipped));
    assert!(off(&stiff) < 1e-4 && off(&shipped) < 1e-2 && off(&stiff) < 0.02 * off(&shipped));
}

/// The largest sway (the load's horizontal offset from under the sheave) over 4 s after a move from `from` to `to` has
/// arrived, shaped or not as `edits` say; the move's shaped tail is let run out first.
fn sway_after(from: [f64; 3], to: [f64; 3], edits: &[&str]) -> f64 {
    let mut cfg = with(edits);
    cfg.set_text(&format!("start.pose = {from:?}")).unwrap();
    let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
    let sway = sim.signal_index("load.sway").unwrap();
    sim.command(Command::MoveTo(to.to_vec())).unwrap();
    while sim.control.busy() {
        sim.step();
    }
    // The longest shaper here, ZV at a 4.5 m rope, spans half a period, 2.1 s; ZVD's whole one, 4.3 s.
    sim.run(4.5, |_| {});
    let (mut sample, mut largest) = (Vec::new(), 0f64);
    sim.run(4.0, |s| {
        s.sample(&mut sample);
        largest = largest.max(sample[sway]);
    });
    largest
}

/// The input shaper against the sway. A travel at the travel height (a 2 m rope, 0.35242 Hz): unshaped the load swings
/// on after the trolley stops; shaped (the shipped ZV) less than a tenth of that is left. Then the empty hook's way back,
/// which hoists from a 4 m rope to a 2 m one while it travels: the sway period falls from 4.01 s to 2.84 s under the
/// move. A shaper that follows the rope's length (effects.shaper_follows_pose, the shipped setting) leaves the least,
/// under a tenth of the unshaped sway; one set for the length at either end of the move leaves three times as much or
/// more (measured with ZV: unshaped 0.434 m, following 0.017 m, set for the end 0.124 m, for the start 0.105 m; with
/// ZVD 0.012, 0.047 and 0.041 m).
#[test]
fn the_shaper_cancels_the_sway_and_following_the_rope_length_beats_a_fixed_period() {
    let travel =
        |shaping: bool| sway_after([1.0, 1.0, 3.0], [8.0, 4.0, 3.0], &[&format!("effects.input_shaping = {shaping}")]);
    let (plain, shaped) = (travel(false), travel(true));
    println!("travel at 2 m: unshaped {plain:.4} m, shaped {shaped:.4} m");
    assert!(plain > 0.05 && shaped < 0.1 * plain, "unshaped {plain} m, shaped {shaped} m");

    let back = |edits: &[&str]| {
        let all: Vec<&str> = ["effects.input_shaping = true"].into_iter().chain(edits.iter().copied()).collect();
        sway_after([8.0, 4.0, 1.0], [1.0, 1.0, 3.0], &all)
    };
    let following = back(&[]);
    let fixed = |hz: f64| {
        let frequency = format!("profile.shaper_frequency_hz = {hz:?}");
        back(&["effects.shaper_follows_pose = false", &frequency])
    };
    // sqrt(9.80665 / L) / (2 pi) at L = 2 m and 4 m.
    let (at_end, at_start) = (fixed(0.35242), fixed(0.24921));
    let plain = back(&["effects.input_shaping = false"]);
    println!("hoist and travel: unshaped {plain:.4}, following {following:.4}, end {at_end:.4}, start {at_start:.4} m");
    assert!(following < 0.1 * plain, "{following} {plain}");
    assert!(3.0 * following < at_end.min(at_start), "{following} {at_end} {at_start}");
}

/// The shipped cycle (travel while lowering, back, pick, hoist) passes every verdict with the shaper; without it the load is
/// left swinging at the dwells and residual-sway fails (the drives settle; the sway is not in the tool pose). Pick and
/// set-down neither snatch nor slacken the rope (`after_step`): its tension stays within 7 % of the payload's weight
/// above and above the least tension below.
#[test]
fn the_default_cycle_passes_with_the_shaper_and_leaves_the_load_swinging_without() {
    let run = |shaping: bool| {
        let cfg = with(&[&format!("effects.input_shaping = {shaping}"), "effects.thermal_warm_start = false"]);
        let mut sim = Sim::new(&ENTRY, &cfg).unwrap();
        let load = analyse_cycle(&mut sim).unwrap();
        assert!(!load.tripped);
        verdicts(&cfg, &load).unwrap()
    };
    let (shaped, plain) = (run(true), run(false));
    for v in &shaped {
        assert!(v.ok, "{} of motor {:?} is {} against {}", v.check, v.motor, v.value, v.limit);
    }
    let find = |list: &[sim_components::analysis::Verdict], check: &str| {
        list.iter().find(|v| v.check == check).map(|v| (v.ok, v.value)).unwrap()
    };
    assert!(find(&shaped, "rope-max-tension").1 < 1.07 * 2598.762);
    assert_eq!(find(&shaped, "rope-min-tension"), (true, 0.0));
    assert!(find(&plain, "residual-sway").1 > 5.0 * find(&shaped, "residual-sway").1);
    assert!(!find(&plain, "residual-sway").0);
}

/// The torque parts add up and say what they should at rest: the hoist's motor holds the rope's pull (gravity), the
/// travel drives nothing; the rope's own share in the energy is the elastic part.
#[test]
fn the_signals_at_rest_are_the_hand_calculated_ones() {
    let mut sim = Sim::new(&ENTRY, &with(&["effects.torque_ripple = false"])).unwrap();
    let id = |s: &Sim, name: &str| s.signal_index(name).unwrap();
    let mut sample = Vec::new();
    sim.run(0.01, |_| {});
    sim.sample(&mut sample);
    // The start pose: z = 3 m, a 2 m rope, the payload on: 2598.76 N, sway none, the brake's load 5.2 N m.
    assert!((sample[id(&sim, "rope.tension")] - 2598.76).abs() < 0.5);
    // 3.4047 mm more than the pose.
    assert!((sample[id(&sim, "rope.length")] - 2.0034047).abs() < 1e-6);
    assert!(sample[id(&sim, "load.sway")] < 1e-6 && sample[id(&sim, "load.residual_sway")] == 0.0);
    assert!((sample[id(&sim, "hoist.brake_load")] - 5.2).abs() < 0.01);
    assert!((sample[id(&sim, "motor.hoist.torque_gravity")] + 5.2).abs() < 0.01);
    assert!(sample[id(&sim, "motor.bridge.torque_gravity")].abs() < 1e-6);
    // Elastic: the rope's t e / 2 at the pose 2 m below the sheave, 4.6895 J.
    assert!((sample[id(&sim, "energy.elastic")] - 4.6895).abs() < 0.01);
}

/// The mesh friction's bristle on the hoist is capped by the lighter of its rotor and its drum, as a cable winch's
/// winch is (`Converter::apply_to`): with effects.gearbox_mesh_friction and a gearbox far stiffer than any shipped
/// (1e12 N m/rad), the drum's 0.05 kg m2 at 0.08 m, 7.8125 kg at the rope, against the rotor's 1e-4 kg m2 and more times
/// (40 / 0.08)^2, 25 kg and more: 7.8125 * (1e5 / 4)^2 = 4.8828125e9 N/m. Before, the rotor alone set it.
#[test]
fn the_hoist_s_mesh_bristle_is_capped_by_its_drum() {
    // The crane has no switch for it (its drives are rigid): added to its [effects].
    let text = with(&["gearbox.stiffness_nm_rad = 1.0e12"]).to_text().unwrap();
    let meshing = text.replacen("[effects]\n", "[effects]\ngearbox_mesh_friction = true\n", 1);
    assert_ne!(meshing, text);
    let m = crane(&Config::parse(&meshing).unwrap());
    let hoist = &m.drives[2];
    let (drum, rotor) = (0.05 / (0.08 * 0.08), hoist.rotor_inertia * hoist.ratio() * hoist.ratio());
    assert!(drum == 7.8125 && rotor >= 25.0, "{rotor}");
    let cap = drum * (1e5f64 / 4.0).powi(2);
    assert!((hoist.gearbox.mesh_stiffness / cap - 1.0).abs() < 1e-12 && (cap - 4.8828125e9).abs() < 1.0, "{cap}");
}

/// The rigid rope's rolling resistance takes the tension the same pass returns (fixed point), not one of an earlier pass:
/// F_roll,k = mu (M_k g + t (-u_z)) tanh(v_k / v_s), M the bridge with the trolley and the trolley. By hand from the
/// motion's own tension t, to 1e-9 N (the loop stops at 1e-12 of t).
#[test]
fn the_rigid_ropes_rolling_resistance_takes_its_own_tension() {
    let m = crane(&with(&["effects.rope_stretch = false"]));
    let (mut state, mut hold) = (Vec::new(), [0.0; 3]);
    m.equilibrium(&[5.0, 2.0, 2.0], &mut state, &mut hold);
    // A swing: the load 0.4 m aside with a speed, the carriages rolling at 0.01 m/s.
    state[LOAD] += 0.4;
    state[LOAD_SPEED] = 0.3;
    state[RATES] = 0.01;
    state[RATES + 1] = 0.02;
    let motion = m.motion(&state, &hold);
    let (t, u) = (motion.rope_state.tension, motion.rope_state.u);
    let down = t * -u[2];
    let wheels = [(m.bridge + m.trolley) * m.g + down, m.trolley * m.g + down];
    for k in 0..2 {
        let want = m.rolling * wheels[k] * (state[RATES + k] / m.rolling_smoothing).tanh();
        assert!((motion.rolling[k] - want).abs() < 1e-9, "{k}: {} against {want}", motion.rolling[k]);
    }
}

fn floor_crane(extra: &[&str]) -> Crane {
    let mut edits = vec!["effects.floor_contact = true", "effects.rope_damping = false"];
    edits.extend(extra);
    crane(&with(&edits))
}

/// The load (hook and payload, m = 265 kg, m g = 2598.762 N) standing on the floor at 0.5 m under the sheave at 5 m, the
/// drum's free length l: the rope's stretch e = 4.5 - l, its tension t = EA e / l. Slack (l = 5): the free load would
/// fall with -g, the floor carries all of m g and the load's acceleration is 0. At l = 4.496, e = 4 mm,
/// t = 1.44e6 x 0.004 / 4.496 = 1281.14 N: the floor carries the rest, m g - t = 1317.62 N. At l = 4.49, e = 10 mm,
/// t = 3207.1 N > m g: no floor force, the load rises with (t - m g) / m = 2.30 m/s2. Without the effect it falls.
#[test]
fn the_load_stands_on_the_floor_until_the_rope_carries_it() {
    let m = floor_crane(&[]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 3]);
    m.equilibrium(&[8.0, 4.0, 1.0], &mut state, &mut hold);
    state[LOAD + 2] = 0.5;
    let weight = m.load_mass() * G;
    let run = |l: f64| {
        let mut s = state.clone();
        s[2] = l;
        let motion = m.motion(&s, &hold);
        (motion.floor_force, motion.load_acc[2], m.rope(&s).tension)
    };
    let (n, a, t) = run(5.0);
    assert!(t == 0.0 && (n - weight).abs() < 1e-9 && a == 0.0, "{n} {a}");
    let (n, a, t) = run(4.496);
    assert!((t - 1.44e6 * 0.004 / 4.496).abs() < 1e-6 && (n - (weight - t)).abs() < 1e-6 && a == 0.0, "{n} {t}");
    assert!((n - 1317.62).abs() < 0.01, "{n}");
    let (n, a, t) = run(4.49);
    assert!(n == 0.0 && (a - (t - weight) / m.load_mass()).abs() < 1e-9 && (a - 2.30).abs() < 0.01, "{n} {a}");
    // Off, the same slack rope lets the load fall freely.
    let free = crane(&with(&["effects.rope_damping = false"]));
    let mut s = state.clone();
    s[2] = 5.0;
    assert!((free.motion(&s, &hold).load_acc[2] + G).abs() < 1e-12);
}

/// The fall onto the floor is plastic: a load arriving at the floor at 1 m/s loses m v^2 / 2 = 132.5 J (265 kg), booked
/// as `pick_loss` over the step (J / dt), and stays at the floor height with no speed; a payload put on while it stands
/// is not moved up the rope (the quasi-static pick is for the load above the floor).
#[test]
fn the_fall_onto_the_floor_loses_its_kinetic_energy() {
    let mut m = floor_crane(&[]);
    let (mut state, mut hold) = (Vec::new(), [0.0; 3]);
    m.equilibrium(&[8.0, 4.0, 1.0], &mut state, &mut hold);
    state[2] = 5.0;
    state[LOAD + 2] = 0.5;
    state[LOAD_SPEED + 2] = -1.0;
    m.after_step(&mut state, 1e-3);
    assert!(state[LOAD + 2] == 0.5 && state[LOAD_SPEED + 2] == 0.0);
    assert!((m.pick_loss * 1e-3 - 132.5).abs() < 1e-9, "{}", m.pick_loss);
    m.set_payload_attached(false);
    m.set_payload_attached(true);
    let before = state.clone();
    m.after_step(&mut state, 1e-3);
    assert!(state == before && m.pick_loss == 0.0);
}
