//! Temperature feedback, belt tooth jump and the sideways vibration of the belt spans, against hand calculations.

use sim_components::analysis::{BUDGET_TOLERANCE, analyse_cycle, verdicts};
use sim_components::{Command, Sim};
use sim_core::{Config, MachineEntry};
use sim_machines::MACHINES;

fn edited(entry: &MachineEntry, edits: &[&str]) -> Config {
    let mut cfg = entry.config();
    for edit in edits {
        cfg.set_text(edit).unwrap();
    }
    cfg
}

fn signal(sim: &Sim, id: &str) -> f64 {
    let mut sample = Vec::new();
    sim.sample(&mut sample);
    sample[sim.signal_index(id).unwrap_or_else(|| panic!("no signal {id}"))]
}

/// Viscosity factor of the configs' law (B 4000 K, ambient and reference 20 °C) at a rise.
fn viscosity(rise: f64) -> f64 {
    (4000.0 * (1.0 / (293.15 + rise) - 1.0 / 293.15)).exp()
}

/// With effects.thermal_feedback, at a fixed rise of 30 K the guides' viscous friction is exp(4000 (1/323.15 -
/// 1/293.15)) = 0.281749 of the configured one, their Coulomb level stays, and effects.temperature's hand-set factor
/// (1.5) is not used; at 0 K they are as configured. A gearbox's loss in a cruise falls with the share of it that is
/// viscous: (1 - 0.967956) / (1 - 0.95) = 0.64088 of what it was (the same share whatever the efficiency at 20 °C).
/// The guides are the friction contacts with a `[friction_<axis>]` table; a machine without effects.temperature (it has
/// no hand-set factors) is run without it.
#[test]
fn with_thermal_feedback_friction_and_gearbox_loss_follow_the_temperature() {
    for entry in MACHINES.iter().copied() {
        let mut cfg = edited(entry, &["effects.part_heating = true", "effects.thermal_feedback = true"]);
        if cfg.bool("effects.temperature").is_ok() {
            cfg.set_text("effects.temperature = true").unwrap();
        }
        let mut sim = Sim::new(entry, &cfg).unwrap();
        let guide = |id: &str| cfg.get(&format!("friction_{}", id.trim_start_matches("axis."))).is_ok();
        let contacts: Vec<_> = sim.machine.friction_contacts().into_iter().enumerate().filter(|(_, c)| guide(c.id)).collect();
        for (i, contact) in contacts.iter().map(|(i, c)| (*i, c)) {
            let table = format!("friction_{}", contact.id.trim_start_matches("axis."));
            let [coulomb, viscous, _] = sim.machine.friction_curve(i, 0.5, 1.0);
            assert_eq!((coulomb, viscous), (cfg.f64(&format!("{table}.coulomb_n")).unwrap(), 0.5 * cfg.f64(&format!("{table}.viscous_n_s_m")).unwrap()), "{}", entry.id);
        }
        // Into a cruise, then the same state with every part 30 K warm.
        let axis = sim.machine.axes()[0].clone();
        let mut target: Vec<f64> = cfg.f64s("start.pose").unwrap();
        target[0] = axis.max - 0.05 * axis.max;
        sim.command(Command::MoveTo(target)).unwrap();
        while (signal(&sim, &format!("axis.{}.velocity_setpoint", axis.id)) - axis.max_vel).abs() > 1e-9 {
            sim.step();
        }
        sim.run(0.1, |_| {});
        let gearbox = sim.heated.iter().position(|h| h.part.kind == "gearbox").map(|i| format!("power.{}", sim.heated[i].part.id));
        let cold = gearbox.as_ref().map(|id| signal(&sim, id));
        for part in &mut sim.heated {
            part.rise = 30.0;
        }
        sim.feed_temperatures_back();
        for (i, contact) in contacts.iter().map(|(i, c)| (*i, c)) {
            let table = format!("friction_{}", contact.id.trim_start_matches("axis."));
            let [coulomb, viscous, _] = sim.machine.friction_curve(i, 0.5, 1.0);
            let by_hand = 0.5 * cfg.f64(&format!("{table}.viscous_n_s_m")).unwrap() * 0.281749;
            assert!(coulomb == cfg.f64(&format!("{table}.coulomb_n")).unwrap() && (viscous - by_hand).abs() < 1e-5, "{}: {viscous} N against {by_hand} N", entry.id);
        }
        if let (Some(id), Some(cold)) = (gearbox, cold) {
            let warm = signal(&sim, &id);
            // The gearbox's own shortfall 1 - g falls to 0.64088 of itself; a screw's or rack's behind it (efficiency e)
            // stays. The loss is T w (1 - eta) with the torque into the gearbox set, T w (1 - eta) / eta with the torque
            // out of it set, eta = g e; the ratio warm to cold lies between the two (by hand 0.6409 and 0.6338 for a
            // gearbox of 97 %, 0.6409 and 0.5724 for a strain wave gear's 75 %).
            let g = cfg.f64("gearbox.efficiency").unwrap();
            // An arm's heated gearboxes are those of its turning joints, with nothing behind them.
            let e = if cfg.get("arm.gear_ratios").is_ok() { 1.0 } else { ["rack.efficiency", "screw.efficiency"].iter().find_map(|k| cfg.f64(k).ok()).unwrap_or(1.0) };
            let warm_eta = (1.0 - 0.64088 * (1.0 - g)) * e;
            let (input_set, output_set) = ((1.0 - warm_eta) / (1.0 - g * e), (1.0 - warm_eta) / warm_eta / ((1.0 - g * e) / (g * e)));
            let ratio = warm / cold;
            assert!(ratio > output_set.min(input_set) - 0.005 && ratio < output_set.max(input_set) + 0.005, "{}: {warm} W against {cold} W, {ratio} not within {output_set} to {input_set}", entry.id);
        }
    }
}

/// A guide at constant speed with the feedback on settles where its heat flow balances its loss, which now falls as
/// it warms: rise = R (lambda F_c + b nu(rise) / nu_ref v) v. Single axis, R 2 K/W, at 1 m/s with lambda =
/// (200 + 6 g) / (200 + 4 g) = 1.08199: rise = 2 (16.230 + 20 nu(rise) / nu_ref), solved by hand by iteration from 72.46 K
/// (the rise without the feedback) to 40.18 K, where nu / nu_ref = 0.1943. The run's own speed and load scale are used
/// for the balance; it is solved here by bisection.
#[test]
fn a_guide_settles_where_its_loss_and_heat_flow_balance() {
    let entry = &machine_single_axis::ENTRY;
    let mut cfg = edited(entry, &["effects.part_heating = true", "effects.thermal_feedback = true", "thermal.guide_time_constant_s = 0.02", "thermal.guide_resistance_k_w = 2.0"]);
    cfg.set_text("start.pose = [0.05]").unwrap();
    let mut sim = Sim::new(entry, &cfg).unwrap();
    let part = sim.heated.iter().position(|h| h.part.id == "guide_x").unwrap();
    sim.command(Command::MoveTo(vec![0.95])).unwrap();
    while (signal(&sim, "axis.x.velocity_setpoint") - 1.0).abs() > 1e-9 {
        sim.step();
    }
    // Fourteen time constants of cruise.
    sim.run(0.28, |sim| assert!((signal(sim, "axis.x.velocity_setpoint") - 1.0).abs() < 1e-9, "the move stopped cruising"));
    let (scale, v) = (signal(&sim, "axis.x.friction_load_scale"), signal(&sim, "axis.x.friction_speed"));
    let balance = |rise: f64| 2.0 * (scale * 15.0 + 20.0 * viscosity(rise) * v) * v - rise;
    let (mut low, mut high) = (0.0, 100.0);
    for _ in 0..60 {
        let mid = 0.5 * (low + high);
        if balance(mid) > 0.0 { low = mid } else { high = mid }
    }
    let rise = sim.heated[part].rise;
    assert!((rise - low).abs() < 0.01 * low, "{rise} K against {low} K");
    assert!((low - 40.18).abs() < 0.5, "the balance at the run's speed {v} m/s and scale {scale}: {low} K");
}

/// Single axis standing upright (incline 90 degrees) and holding at x = 0.1 m: the carriage hangs on section 2,
/// T2 - T1 = W, and the sections share their belt, so (to first order in the strain) T1 = P - W L2 / (L1 + L2) with
/// L1 = 0.25 m, L2 = 2.35 m. The teeth climb out at W > c T1, c = 2.4 * 10 * 0.005 / 0.02 = 6:
/// W = 6 P / (1 + 6 L2 / (L1 + L2)) = 1800 / 6.42308 = 280.24 N, 28.576 kg. At 2 % less nothing jumps in a second of
/// holding; at 2 % more the belt slides towards the tight side and the first jump ends exactly one pitch on, u = -5 mm.
#[test]
fn the_belt_teeth_jump_at_the_hand_calculated_pull() {
    let entry = &machine_single_axis::ENTRY;
    let hold = |share: f64| {
        let payload = share * 28.5765 - 4.0;
        let cfg = edited(entry, &[
            "effects.belt_tooth_jump = true",
            "effects.tooth_meshing = false",
            "effects.torque_ripple = false",
            "geometry.incline_deg = 90.0",
            &format!("mass.payload_kg = {payload}"),
        ]);
        let mut sim = Sim::new(entry, &cfg).unwrap();
        let (t1, t2) = (signal(&sim, "belt.1.tension"), signal(&sim, "belt.2.tension"));
        assert!((t2 - t1 - share * 280.24).abs() < 0.01 * 280.0 && (t1 - (300.0 - share * 280.24 * 2.35 / 2.6)).abs() < 1.0, "{t1} N, {t2} N");
        let mut first = None;
        for _ in 0..16000 {
            sim.step();
            if first.is_none() && signal(&sim, "pulley.m.tooth_jumps") >= 1.0 {
                first = Some(signal(&sim, "pulley.m.tooth_slip"));
            }
        }
        first
    };
    assert_eq!(hold(0.98), None);
    assert_eq!(hold(1.02), Some(-0.005));
}

/// Teeth that jump in a cycle fail "belt-tooth-jump", and what they take closes the energy budget like every other
/// loss: the single axis with 90 N of pretension, where its acceleration pulls the slack side below a sixth of the
/// pull (with 100 N the slack side keeps 20.9 N against 119.5 N of pull, just above a sixth, and nothing jumps).
/// Once they jump, they go on jumping (240 jumps, 11.3 J). The budget closes to 0.5 % here, not to the 0.1 % of the
/// other configs: a jump is found at the end of a base step, so it lands up to a step late. Measured 0.33 %, 0.21 %
/// and 0.14 % at the base step, its half and its quarter (with the current cycle along).
#[test]
fn jumping_teeth_fail_the_check_and_close_the_energy_budget() {
    let entry = &machine_single_axis::ENTRY;
    let cfg = edited(entry, &["effects.belt_tooth_jump = true", "belt.pretension_n = 90.0"]);
    let load = analyse_cycle(&mut Sim::new(entry, &cfg).unwrap()).unwrap();
    let jumps = load.belt_tooth_jumps.unwrap();
    assert!(jumps > 0.0, "no jump");
    let budget = load.budget.as_ref().unwrap();
    let lost = budget.rows.iter().find(|r| r.id == "power.tooth_jump").unwrap().energy;
    assert!(lost > 0.0 && budget.unaccounted().abs() < 5.0 * BUDGET_TOLERANCE * budget.supply, "{jumps} jumps, {lost} J in them, {} J of {} J not accounted for", budget.unaccounted(), budget.supply);
    let v = verdicts(&cfg, &load).unwrap().into_iter().find(|v| v.check == "belt-tooth-jump").unwrap();
    assert!(!v.ok && v.value == jumps && v.limit == 0.0);
    // The default config's teeth hold: the check passes.
    let cfg = edited(entry, &["effects.belt_tooth_jump = true"]);
    let load = analyse_cycle(&mut Sim::new(entry, &cfg).unwrap()).unwrap();
    assert_eq!(load.belt_tooth_jumps, Some(0.0));
}

/// The spans at rest, single axis at x = 0.1 m with 300 N and 0.1 kg/m: f1 = sqrt(3000) / (2 L), L = 0.25, 1.3 and
/// 1.05 m: 109.545, 21.066 and 26.082 Hz; nothing excites them at rest. The T-gantry's first span (clamp down to the
/// carriage idler, 0.15 m + z) at its start pose by the same formula.
///
/// In the cycle's cruise at 1 m/s the teeth mesh at 1 / 0.005 = 200 Hz. With 1.109e-3 kg/m the span of 1.3 m has
/// 300 N / 1.109e-3 = sqrt -> 520.1 m/s, f1 = 200.0 Hz: "belt-span-resonance" fails; with the 0.1 kg/m of the config
/// every span stays below 110 Hz and the check passes with the cap of 1.
#[test]
fn a_span_vibrates_at_its_string_frequency_and_a_cruise_on_resonance_fails() {
    let entry = &machine_single_axis::ENTRY;
    let sim = Sim::new(entry, &edited(entry, &["effects.belt_span_vibration = true"])).unwrap();
    for (i, by_hand) in [(1, 109.5445), (2, 21.06625), (3, 26.08203)] {
        let f = signal(&sim, &format!("belt.span.{i}.frequency"));
        assert!((f - by_hand).abs() < 1e-3, "span {i}: {f} Hz against {by_hand} Hz");
    }
    assert_eq!(signal(&sim, "belt.span_detuning"), 1.0);
    let gantry = &machine_t_gantry::ENTRY;
    let cfg = edited(gantry, &["effects.belt_span_vibration = true"]);
    let sim = Sim::new(gantry, &cfg).unwrap();
    let (z, t1) = (cfg.f64s("start.pose").unwrap()[1], signal(&sim, "belt.1.tension"));
    let by_hand = (t1 / 0.1f64).sqrt() / (2.0 * (0.15 + z));
    assert!((signal(&sim, "belt.span.1.frequency") - by_hand).abs() < 1e-9 * by_hand);
    let judged = |mass: &str| {
        let cfg = edited(entry, &["effects.belt_span_vibration = true", mass]);
        let load = analyse_cycle(&mut Sim::new(entry, &cfg).unwrap()).unwrap();
        verdicts(&cfg, &load).unwrap().into_iter().find(|v| v.check == "belt-span-resonance").unwrap()
    };
    let on = judged("belt.mass_kg_m = 1.109e-3");
    assert!(!on.ok && on.value < 0.1 && on.limit == 0.1, "{on:?}");
    let off = judged("belt.mass_kg_m = 0.1");
    assert!(off.ok && off.value == 1.0, "{off:?}");
}
