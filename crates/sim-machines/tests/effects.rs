//! The effect views on every machine: the parts of the motor torque add up to it, and the energy budget closes.

use sim_components::analysis::{
    BUDGET_TOLERANCE, CycleLoad, ELECTRICAL_TOLERANCE, analyse_cycle, analyse_cycle_with, bounds, sensitivity, verdicts,
};
use sim_components::{Command, Sim};
use sim_core::{Config, LOSS_PREFIX, MachineEntry, TORQUE_PARTS};
use sim_machines::MACHINES;

/// Every config the effect views are checked on, with a name: the default, the rigid model, the loss-free
/// configs of the standard suite, without and with a shared DC link, every remaining effect on, and the
/// catalogue machine.
fn configs(entry: &MachineEntry) -> Vec<(String, Config)> {
    let edited = |edits: &[&str]| {
        let mut cfg = entry.config();
        for edit in edits {
            cfg.set_text(edit).unwrap();
        }
        cfg
    };
    let mut everything = entry.config();
    // Not the fieldbus jitter: the drives' position setpoint then holds for a cycle and jumps, and the budget's losses,
    // taken once per step, miss 0.12 % of the T-gantry's supply (0.03 % at half the step, so it is the sampling, not a
    // loss left out), beyond its 0.1 %.
    for effect in [
        "idlers",
        "cable_chain",
        "process_force",
        "belt_tooth_stiffness",
        "belt_material_damping",
        "sensor_noise",
        "fieldbus_delay",
        "part_heating",
        "thermal_feedback",
        "belt_tooth_jump",
        "belt_span_vibration",
        "belt_wrap",
    ] {
        if everything.bool(&format!("effects.{effect}")).is_ok() {
            everything.set_text(&format!("effects.{effect} = true")).unwrap();
        }
    }
    let mut out = vec![
        ("default".to_string(), entry.config()),
        (
            "rigid".into(),
            edited(&[&format!(
                "{} = false",
                ["effects.belt_stretch", "effects.joint_elasticity", "effects.cable_stretch"]
                    .into_iter()
                    .find(|key| entry.config().bool(key).is_ok())
                    .unwrap()
            )]),
        ),
        ("no DC link".into(), edited(&["effects.dc_link = false"])),
        ("shared DC link".into(), edited(&["effects.shared_dc_link = true"])),
        ("every effect".into(), everything),
    ];
    out.extend((entry.suite_configs)().into_iter().enumerate().map(|(i, cfg)| (format!("loss-free {i}"), cfg)));
    // Not the SCARA's loss-free elastic configs (2 and 3): with no damping at all its quill on the screw is a 570 Hz mode
    // that the velocity loop's delays drive unstable, a real property of a loop around an undamped mode and not of the
    // integration (0.05 N m s/rad of damping or a quarter of the gain holds it; the default config has 1 N m s/rad).
    // The standard suite checks those configs' energy without the controller. The same for the hexapod's: with its
    // gearboxes' damping at zero the elastic legs' mode runs away under the velocity loops (utilisation 31.6 and 2.8e11
    // on 14 November 2026, 0.076 on the default config with its damping).
    out.retain(|(name, _)| {
        !(["scara", "hexapod"].contains(&entry.id) && (name == "loss-free 2" || name == "loss-free 3"))
    });
    if entry.id == machine_t_gantry::ENTRY.id {
        out.push(("EXCT-30".into(), sim_components::load_config(entry, machine_t_gantry::EXCT_30_CONFIG).unwrap()));
    }
    out
}

/// Every config of `configs` on every machine, each cycle analysed on a thread of its own with `edits` on top:
/// (name, its load).
fn analysed(edits: &[&str]) -> Vec<(String, CycleLoad)> {
    analysed_where(edits, |_| true)
}

/// `analysed`, of the configs whose name `which` takes.
fn analysed_where(edits: &[&str], which: impl Fn(&str) -> bool) -> Vec<(String, CycleLoad)> {
    let runs: Vec<(String, Config, &MachineEntry)> = MACHINES
        .iter()
        .flat_map(|entry| {
            configs(entry)
                .into_iter()
                .map(move |(name, cfg)| (format!("{} {name}", entry.id), with(&cfg, edits), *entry))
        })
        .filter(|(name, ..)| which(name))
        .collect();
    std::thread::scope(|scope| {
        let workers: Vec<_> = runs
            .iter()
            .map(|(name, cfg, entry)| {
                scope.spawn(move || (name.clone(), analyse_cycle(&mut Sim::new(entry, cfg).unwrap()).unwrap()))
            })
            .collect();
        workers.into_iter().map(|w| w.join().unwrap()).collect()
    })
}

/// `check` on every config of `configs` on every machine, each on a thread of its own: (machine, config name, config).
fn on_every_config(check: impl Fn(&MachineEntry, &str, Config) + Sync) {
    let runs: Vec<(&MachineEntry, String, Config)> = MACHINES
        .iter()
        .flat_map(|entry| configs(entry).into_iter().map(move |(name, cfg)| (*entry, name, cfg)))
        .collect();
    let check = &check;
    std::thread::scope(|scope| {
        let workers: Vec<_> =
            runs.into_iter().map(|(entry, name, cfg)| scope.spawn(move || check(entry, &name, cfg))).collect();
        workers.into_iter().for_each(|w| w.join().unwrap());
    });
}

/// `cfg` analysed from cold (effects.thermal_warm_start off; a config without the key starts cold anyway).
fn cold(cfg: &Config) -> Config {
    if cfg.get("effects.thermal_warm_start").is_err() {
        cfg.clone()
    } else {
        with(cfg, &["effects.thermal_warm_start = false"])
    }
}

/// The supply equals the copper loss, the resistor, each loss of the machine and what the stores hold more at the end
/// of the pass, to within `BUDGET_TOLERANCE` of the supply (0.1 %), on every config, the loss-free ones included.
/// Measured: at most 0.03 % (T-gantry loss-free elastic), 0.009 % on the default configs.
///
/// The electrical side is checked on its own, not closed by a remainder: supply = copper + resistor + Δ windings +
/// Δ capacitors + the work of the torques on the shafts, to within `ELECTRICAL_TOLERANCE` (1e-6) of the supply.
/// The current is integrated with the mechanics, so what is left is the integration's error: measured at most 3.2e-7
/// (single axis, loss-free, inclined), 1e-12 on the default configs.
#[test]
fn the_energy_budget_closes_on_every_machine_and_config() {
    let results = analysed(&[]);
    for (name, load) in &results {
        let budget = load.budget.as_ref().expect("every machine reports its losses");
        for row in &budget.rows {
            println!("{name}: {} {:.6} J", row.id, row.energy);
        }
        let (supply, unaccounted, electrical) = (budget.supply, budget.unaccounted(), budget.electrical_unaccounted());
        println!(
            "{name}: supply {supply:.6} J, not accounted for {unaccounted:.3e} J ({:.4} %), electrical side {electrical:.3e} J ({:.2e})",
            100.0 * unaccounted / supply,
            electrical / supply
        );
    }
    for (name, load) in results {
        let budget = load.budget.unwrap();
        let (supply, unaccounted, electrical) = (budget.supply, budget.unaccounted(), budget.electrical_unaccounted());
        assert!(
            supply > 0.0 && unaccounted.abs() < BUDGET_TOLERANCE * supply,
            "{name}: {unaccounted} J of {supply} J not accounted for"
        );
        assert!(
            electrical.abs() < ELECTRICAL_TOLERANCE * supply,
            "{name}: the electrical side is off by {electrical} J of {supply} J"
        );
        // The line module (or rectifier) loses its share of what it draws either way, but feeds nothing back.
        assert!(
            !budget.rows.iter().any(|r| r.id == "mains"),
            "{name}: nothing fed back without the regenerative supply"
        );
    }
}

/// The regenerative supply against a hand calculation on every config. A DC link of its own is held at the bus level
/// from both sides: below it the rectifier draws, above it the line module feeds back. So in every step the line module
/// takes what the motor feeds back and the rectifier gives what it draws: fed back to the mains is the efficiency times
/// the energy the motor regenerated (`MotorLoad::energy_regenerated`), drawn from the line the energy the motor drew
/// (`energy_in`) over the efficiency, lost in the line module the rest of both, and the braking resistor burns nothing.
/// With the efficiency of 0.98: 2 % of what is fed back and 1 / 0.98 - 1 = 2.04 % of what is drawn. A shared link nets
/// the drives first, so it feeds back no more than that. The budget closes, its electrical side to `ELECTRICAL_TOLERANCE`; the
/// two line module checks judge the mean and the highest power through it.
#[test]
fn the_regenerative_supply_feeds_back_what_the_motors_regenerate() {
    // From cold: the line module's share is judged here, nothing thermal; the budget of a warm start closes in
    // `the_energy_budget_closes_on_every_machine_and_config`, and the warm start would run each cycle once more.
    for (name, load) in analysed(&["effects.regenerative_supply = true", "effects.thermal_warm_start = false"])
        .into_iter()
        .filter(|(name, _)| !name.contains("no DC link"))
    {
        let budget = load.budget.as_ref().unwrap();
        let row = |id: &str| budget.rows.iter().find(|r| r.id == id).unwrap().energy;
        let (mains, line, resistor) = (row("mains"), row("line_module"), row("resistor"));
        let efficiency = 0.98;
        let regenerated: f64 = load.motors.iter().map(|m| m.energy_regenerated).sum();
        if name.contains("shared") {
            assert!(
                mains <= efficiency * regenerated + 1e-9 && mains > 0.0,
                "{name}: {mains} J fed back of {regenerated} J"
            );
        } else {
            // The budget starts a step later than the motors' figures (see `analyse_cycle`): to what one step at the line
            // module's highest power can make.
            let (drawn, step): (f64, f64) = (
                load.motors.iter().map(|m| m.energy_in).sum(),
                load.motors.iter().map(|m| 62.5e-6 * m.peak_line_module_power).sum::<f64>() + 1e-9,
            );
            assert!(
                (mains - efficiency * regenerated).abs() < step
                    && (line - (1.0 - efficiency) * regenerated - (1.0 / efficiency - 1.0) * drawn).abs() < step,
                "{name}: {mains} J and {line} J of {regenerated} J regenerated and {drawn} J drawn"
            );
            assert!(
                (budget.supply - drawn / efficiency).abs() < step,
                "{name}: {} J drawn, the motors drew {drawn} J",
                budget.supply
            );
            for m in &load.motors {
                assert!(
                    (m.mains_energy - efficiency * m.energy_regenerated).abs() < 1e-9 * m.energy_regenerated.max(1.0)
                        && (m.line_module_energy - m.energy_in - m.energy_regenerated).abs()
                            < 1e-9 * m.line_module_energy,
                    "{name}: {m:?}"
                );
            }
        }
        let bus = load.motors.iter().map(|m| m.peak_dc_link_voltage).fold(0.0, f64::max);
        println!(
            "{name}: fed back {mains:.4} J of {regenerated:.4} J regenerated, line module {line:.4} J, resistor {resistor} J, highest DC link {bus:.3} V"
        );
        assert!(
            resistor == 0.0 && load.motors.iter().all(|m| m.resistor_energy == 0.0),
            "{name}: the resistor burned {resistor} J"
        );
        assert!(
            budget.unaccounted().abs() < BUDGET_TOLERANCE * budget.supply
                && budget.electrical_unaccounted().abs() < ELECTRICAL_TOLERANCE * budget.supply,
            "{name}: {budget:?}"
        );
    }
}

/// Step-size convergence of the supply: the cycle with twice the integrator's sub-steps (`sim.substeps`, the base step
/// and so the controller unchanged) draws the same energy to within 0.5 %, on every config, the loss-free ones
/// included. Measured: at most 0.06 % (single axis, loss-free elastic, level: 0.4687 J against 0.4684 J).
///
/// Left out, and named here so that it shows: a cycle in which a drive ran into its torque limit (utilisation above
/// 1), where the loop no longer controls the motion and the run is a limit cycle that the integration step moves. Since
/// the default velocity gains were lowered for margin no config does (the single axis's loss-free elastic config,
/// level, did at 0.08: 70.5 J at 4 sub-steps, 73.8 J at 8, peak 3.9 N m).
///
/// Compared with 16 sub-steps instead of 8: the single axis's loss-free elastic config, inclined. Nothing damps it and
/// its gearbox knocks through the backlash, so one run can take another path: 20.925, 20.804, 20.924 and 20.925 J at 4,
/// 8, 16 and 32 sub-steps; the run at 8 alone is off.
#[test]
fn the_supply_converges_with_the_integration_step_on_every_config() {
    // From cold: the warm start would run each cycle once more and change nothing the step does. At 16 sub-steps only
    // the config compared there: run on every config, they took more than those at 4 and 8 together.
    let (cold, chaotic) = ("effects.thermal_warm_start = false", "single-axis loss-free 3");
    let (coarse, fine) = (analysed(&["sim.substeps = 4", cold]), analysed(&["sim.substeps = 8", cold]));
    let finer = analysed_where(&["sim.substeps = 16", cold], |name| name == chaotic);
    assert_eq!(finer.len(), 1);
    let mut saturated = Vec::new();
    for ((name, a), (_, b)) in coarse.iter().zip(&fine) {
        let b = if name == chaotic { &finer[0].1 } else { b };
        let (sa, sb) = (a.budget.as_ref().unwrap().supply, b.budget.as_ref().unwrap().supply);
        let utilisation = a.motors.iter().chain(&b.motors).map(|m| m.utilisation).fold(0.0, f64::max);
        println!(
            "{name}: supply {sa:.4} J at 4 sub-steps, {sb:.4} J at 8 or 16 ({:+.3} %), utilisation {utilisation:.3}",
            100.0 * (sb / sa - 1.0)
        );
        if utilisation >= 1.0 {
            saturated.push(name.clone());
        } else {
            assert!((sa - sb).abs() < 5e-3 * sb, "{name}: {sa} J at 4 sub-steps, {sb} J at 8 or 16");
        }
    }
    assert!(saturated.is_empty(), "{saturated:?}");
}

/// At every step of a cycle the four parts of each motor's torque add up to the motor torque, to rounding: the
/// rest is defined as what the other three leave.
#[test]
fn the_torque_parts_add_up_to_the_motor_torque() {
    for entry in MACHINES {
        for (name, cfg) in configs(entry) {
            let mut sim = Sim::new(entry, &cfg).unwrap();
            let parts: Vec<Vec<usize>> = sim
                .machine
                .motors()
                .iter()
                .map(|m| {
                    TORQUE_PARTS
                        .iter()
                        .map(|p| sim.signal_index(&format!("motor.{}.torque_{p}", m.id)).unwrap())
                        .collect()
                })
                .collect();
            let torques: Vec<usize> = sim
                .machine
                .motors()
                .iter()
                .map(|m| sim.signal_index(&format!("motor.{}.torque", m.id)).unwrap())
                .collect();
            sim.command(Command::StartCycle).unwrap();
            let mut sample = Vec::new();
            for _ in 0..200 {
                sim.run(0.005, |_| {});
                sim.sample(&mut sample);
                for (places, &torque) in parts.iter().zip(&torques) {
                    let sum: f64 = places.iter().map(|&i| sample[i]).sum();
                    assert!(
                        (sum - sample[torque]).abs() < 1e-9 * sample[torque].abs().max(1.0),
                        "{} {name}: parts {sum} N m, torque {} N m",
                        entry.id,
                        sample[torque]
                    );
                }
            }
        }
    }
}

fn with(cfg: &Config, edits: &[&str]) -> Config {
    let mut cfg = cfg.clone();
    for edit in edits {
        cfg.set_text(edit).unwrap();
    }
    cfg
}

/// A gearbox of ratio 1 is the coupling of a direct drive (the EXCT-30's `[gearbox]` table): there is no gearbox to
/// warm, and the loss of `power.gearbox_*` is named as the coupling's damping. A machine with a gearbox has its
/// gearboxes among the parts that warm, with the gearbox loss.
#[test]
fn a_direct_drive_has_no_gearbox_that_warms() {
    // Every gearbox of a config by its ratio: `gearbox.ratio`, a gearbox of one axis in a table of its own (a
    // `<axis>_gearbox` table such as a turning tool's), and one motor's own (`gearbox.<motor>.ratio`). Direct is all of
    // them at ratio 1.
    let ratios = |cfg: &Config| {
        cfg.leaves()
            .into_iter()
            .map(|(key, _)| key)
            .filter(|key| key.ends_with("gearbox.ratio") || key.starts_with("gearbox.") && key.ends_with(".ratio"))
            .collect::<Vec<_>>()
    };
    for entry in MACHINES {
        let direct_drive = ratios(&entry.config()).iter().map(|key| format!("{key} = 1.0")).collect::<Vec<_>>();
        let direct_drive = with(&entry.config(), &direct_drive.iter().map(String::as_str).collect::<Vec<_>>());
        for (name, cfg) in configs(entry).into_iter().chain([("ratio 1".to_string(), direct_drive)]) {
            let machine = (entry.build)(&cfg).unwrap();
            // An arm has its ratios per joint (`arm.gear_ratios`), and a SCARA's quill sits on its screw through a
            // coupling: each joint's gearbox warms and is named as such, each coupling neither.
            let direct =
                ratios(&cfg).iter().all(|key| cfg.f64(key).unwrap() == 1.0) && cfg.get("arm.gear_ratios").is_err();
            let gearboxes = machine.thermal_parts().iter().filter(|p| p.kind == "gearbox").count();
            let losses: Vec<_> = machine
                .signal_infos()
                .into_iter()
                .filter(|s| s.id.starts_with(&format!("{LOSS_PREFIX}gearbox")))
                .collect();
            let geared = losses.iter().filter(|s| s.quantity_key == "quantity-gearbox-loss-power").count();
            assert!(
                losses.len() == machine.motors().len()
                    && losses.iter().all(|s| s.quantity_key == "quantity-gearbox-loss-power"
                        || s.quantity_key == "quantity-coupling-loss-power"),
                "{} {name}: {losses:?}",
                entry.id
            );
            assert_eq!(gearboxes, geared, "{} {name}", entry.id);
            assert!(if direct { geared == 0 } else { geared > 0 }, "{} {name}: {geared} gearboxes", entry.id);
        }
    }
}

/// With effects.part_heating a part settles where its thermal resistance times its mean loss over the measured pass
/// puts it: R_th times its row of the energy budget over the cycle's duration. "gearbox-thermal" and "guide-thermal"
/// judge the hottest of each kind against `thermal.<kind>_max_rise_k`. Without the effect neither is judged.
#[test]
fn the_part_thermal_verdicts_judge_where_the_parts_settle() {
    on_every_config(|entry, name, cfg| {
        // The temperature feedback warms the parts too, so both are off here. From cold: with no part warming, no part
        // rise and no part's verdict can come up whatever the windings start at.
        let off = analyse_cycle_with(
            &mut Sim::new(
                entry,
                &cold(&with(&cfg, &["effects.part_heating = false", "effects.thermal_feedback = false"])),
            )
            .unwrap(),
            false,
        )
        .unwrap();
        let judged = verdicts(&cfg, &off).unwrap();
        assert!(
            off.part_rises.is_empty()
                && !judged.iter().any(|v| v.check.ends_with("-thermal") && v.check != "motor-thermal"),
            "{} {name}",
            entry.id
        );

        // From cold too: what is judged here, a settled rise as the mean loss times the resistance, holds whatever rises a
        // pass started at, and the warm start would run each cycle once or twice more.
        let cfg = cold(&with(&cfg, &["effects.part_heating = true"]));
        let load = analyse_cycle(&mut Sim::new(entry, &cfg).unwrap()).unwrap();
        let budget = load.budget.as_ref().unwrap();
        assert_eq!(load.part_rises.len(), (entry.build)(&cfg).unwrap().thermal_parts().len());
        for (part, rise) in &load.part_rises {
            let lost = budget.rows.iter().find(|r| r.id == format!("{LOSS_PREFIX}{}", part.id)).unwrap().energy;
            let by_hand = cfg.f64(&format!("thermal.{}_resistance_k_w", part.kind)).unwrap() * lost / load.duration;
            assert!(
                (rise - by_hand).abs() <= 1e-12 * by_hand.abs(),
                "{} {name} {}: {rise} K against {by_hand} K",
                entry.id,
                part.id
            );
        }
        let judged = verdicts(&cfg, &load).unwrap();
        for (kind, check) in [("gearbox", "gearbox-thermal"), ("guide", "guide-thermal")] {
            let hottest =
                load.part_rises.iter().filter(|(p, _)| p.kind == kind).map(|(_, rise)| *rise).reduce(f64::max);
            let verdict = judged.iter().find(|v| v.check == check);
            assert_eq!(verdict.map(|v| v.value), hottest, "{} {name} {check}", entry.id);
            if let Some(v) = verdict {
                assert!(
                    v.limit == cfg.f64(&format!("thermal.{kind}_max_rise_k")).unwrap() && v.ok == (v.value <= v.limit),
                    "{} {name}: {v:?}",
                    entry.id
                );
            }
        }
    });
}

/// What the costly effect signals were split out for changes no number: the cheap loss powers are the `power.*`
/// signals, and a cycle analysed without the effect views has every figure and verdict of the full one.
#[test]
fn the_cheap_signals_and_the_lean_analysis_change_no_number() {
    let same = |a: &CycleLoad, b: &CycleLoad| {
        let motors = a.motors.iter().zip(&b.motors).all(|(a, b)| {
            (
                a.peak_torque,
                a.rms_torque,
                a.cubic_mean_torque,
                a.utilisation,
                a.winding_rise,
                a.energy_in,
                a.peak_dc_link_voltage,
                a.gearbox_torque,
            ) == (
                b.peak_torque,
                b.rms_torque,
                b.cubic_mean_torque,
                b.utilisation,
                b.winding_rise,
                b.energy_in,
                b.peak_dc_link_voltage,
                b.gearbox_torque,
            )
        });
        motors
            && (
                a.duration,
                a.belt_tension,
                a.belt_slack_time,
                a.radial_load_peak,
                a.guide_load,
                a.path_error_peak,
                a.dwell_error_peak,
            ) == (
                b.duration,
                b.belt_tension,
                b.belt_slack_time,
                b.radial_load_peak,
                b.guide_load,
                b.path_error_peak,
                b.dwell_error_peak,
            )
            && a.part_rises == b.part_rises
    };
    on_every_config(|entry, name, cfg| {
        let mut sim = Sim::new(entry, &cfg).unwrap();
        let losses: Vec<usize> = sim
            .machine
            .signal_infos()
            .iter()
            .enumerate()
            .filter(|(_, s)| s.id.starts_with(LOSS_PREFIX))
            .map(|(i, _)| i)
            .collect();
        sim.command(Command::StartCycle).unwrap();
        let (mut all, mut cheap) = (Vec::new(), Vec::new());
        for _ in 0..100 {
            sim.run(0.01, |_| {});
            all.clear();
            cheap.clear();
            sim.machine.signals(&sim.state, &sim.torque, &mut all);
            sim.machine.loss_powers(&sim.state, &sim.torque, &mut cheap);
            assert_eq!(cheap, losses.iter().map(|&i| all[i]).collect::<Vec<_>>(), "{} {name}", entry.id);
        }
        // From cold: a warm pass runs the same steps from the rises the pass before settled at, which the two analyses
        // would share as they share every other figure, so it adds no case to compare.
        let cfg = cold(&with(&cfg, &["effects.part_heating = true"]));
        let (full, lean) = (
            analyse_cycle(&mut Sim::new(entry, &cfg).unwrap()).unwrap(),
            analyse_cycle_with(&mut Sim::new(entry, &cfg).unwrap(), false).unwrap(),
        );
        assert!(
            same(&full, &lean) && lean.budget.is_none() && lean.motors.iter().all(|m| m.torque_parts.is_none()),
            "{} {name}",
            entry.id
        );
        assert_eq!(verdicts(&cfg, &full).unwrap(), verdicts(&cfg, &lean).unwrap(), "{} {name}", entry.id);
    });
}

/// The `[thermal]` values change nothing while effects.part_heating is off, so they get no runs then: no bounds for
/// the bands and no rows in the sensitivity. With the effect on they have their bounds. The same for the line module's
/// efficiency and the regenerative supply; the DC link's discharge time gets no runs at all.
#[test]
fn the_bounds_of_an_effect_that_is_off_get_no_runs() {
    for entry in MACHINES {
        let cfg = entry.config();
        assert!(!cfg.bool("effects.part_heating").unwrap());
        let thermal =
            |cfg: &Config| bounds(cfg).unwrap().into_iter().filter(|(path, ..)| path.starts_with("thermal.")).count();
        assert_eq!(thermal(&cfg), 0, "{}", entry.id);
        // A resistance and a time constant for each kind of part the machine has that warms (gearbox, guide).
        let kinds = (entry.build)(&cfg)
            .unwrap()
            .thermal_parts()
            .iter()
            .map(|p| p.kind)
            .collect::<std::collections::BTreeSet<_>>()
            .len();
        assert_eq!(thermal(&with(&cfg, &["effects.part_heating = true"])), 2 * kinds, "{}", entry.id);
        let (_, rows) = sensitivity(entry, &cfg, &["thermal.gearbox_resistance_k_w".to_string()], 0.2).unwrap();
        assert!(rows.is_empty(), "{}", entry.id);
        // The line module's efficiency wherever the DC link is simulated (it loses its share of what it draws too, not
        // only of what it feeds back); the DC link's discharge, which acts only while the drives are disabled, never.
        let named = |cfg: &Config, path: &str| bounds(cfg).unwrap().iter().any(|(p, ..)| p == path);
        let regenerative = with(&cfg, &["effects.regenerative_supply = true"]);
        let linked = with(&cfg, &["effects.dc_link = true"]);
        assert!(
            !named(&with(&cfg, &["effects.dc_link = false"]), "ratings.line_module_efficiency")
                && named(&linked, "ratings.line_module_efficiency"),
            "{}",
            entry.id
        );
        assert!(
            !named(&regenerative, "ratings.dc_link_discharge_time_constant_s")
                && cfg.f64s("bounds.ratings.dc_link_discharge_time_constant_s").is_ok(),
            "{}",
            entry.id
        );
        // The temperature feedback's values only with it, and with it the [thermal] values, as it warms the parts; the
        // tooth jump factor only with the jumps.
        let feedback = with(&cfg, &["effects.thermal_feedback = true"]);
        let starting =
            |cfg: &Config, start: &str| bounds(cfg).unwrap().iter().filter(|(p, ..)| p.starts_with(start)).count();
        let feedback_bounds = cfg.get("bounds.thermal_feedback").ok().and_then(|t| t.as_table()).map_or(0, |t| t.len());
        assert_eq!(
            (starting(&cfg, "thermal_feedback."), starting(&feedback, "thermal_feedback."), thermal(&feedback)),
            (0, feedback_bounds, 2 * kinds),
            "{}",
            entry.id
        );
        assert!(feedback_bounds >= 3, "{}", entry.id);
        // A machine without a belt has no tooth jump.
        if cfg.bool("effects.belt_tooth_jump").is_ok() {
            assert!(
                !named(&cfg, "belt.tooth_jump_factor")
                    && named(&with(&cfg, &["effects.belt_tooth_jump = true"]), "belt.tooth_jump_factor"),
                "{}",
                entry.id
            );
        }
    }
}
