//! The electrical side of the drives on every machine, against hand calculations: a disabled drive's winding cools and
//! its DC link discharges, and the regenerative supply is judged against its line module's ratings.

use sim_components::analysis::{analyse_cycle_with, verdicts};
use sim_components::{Command, Sim};
use sim_core::Config;
use sim_machines::MACHINES;

/// The config with no motor named per motor (`parts.<k>.motor`): each motor takes the tables' values.
fn unnamed(cfg: &Config) -> Config {
    let named: Vec<String> =
        cfg.get("parts").ok().and_then(|t| t.as_table()).map_or(Vec::new(), |t| {
            t.iter().filter(|(_, v)| v.get("motor").is_some()).map(|(id, _)| id.clone()).collect()
        });
    let mut cfg = cfg.clone();
    for id in named {
        cfg.set_text(&format!("parts.{id}.motor = \"\"")).unwrap();
    }
    cfg
}

/// `edits` on the shared tables, and on every motor's own table that sets the same key (`motor.3.thermal_time_constant_s`
/// over `motor.thermal_time_constant_s`), so that they reach every motor.
fn with(cfg: &Config, edits: &[&str]) -> Config {
    let mut cfg = cfg.clone();
    for edit in edits {
        cfg.set_text(edit).unwrap();
        let (path, value) = edit.split_once('=').unwrap();
        let Some((table, key)) = path.trim().split_once('.') else { continue };
        let own: Vec<String> = cfg.get(table).ok().and_then(|t| t.as_table()).map_or(Vec::new(), |t| {
            t.iter()
                .filter(|(_, v)| v.as_table().is_some_and(|m| m.contains_key(key)))
                .map(|(id, _)| id.clone())
                .collect()
        });
        for id in own {
            cfg.set_text(&format!("{table}.{id}.{key} ={value}")).unwrap();
        }
    }
    cfg
}

/// A disabled drive's winding cools with the model it warms with: no current, so tau dT/dt = -T, and T falls by the
/// factor e every thermal time constant. With tau = 0.5 s and 40 K at the start, by hand 40 / e = 14.715 K after
/// 0.5 s and 40 / e^2 = 5.413 K after 1 s (the model steps it as (1 - dt / tau)^n, 6e-5 relative below the
/// exponential at 8000 steps). Without winding heating it keeps its temperature, as before.
#[test]
fn a_disabled_winding_cools_by_the_exponential() {
    for entry in MACHINES {
        // A motor named per motor (`[parts.<k>]`) brings the catalogue's time constant over any table: none is named.
        let cfg = with(
            &unnamed(&entry.config()),
            &["motor.thermal_time_constant_s = 0.5", "motor.start_temperature_rise_k = 40.0"],
        );
        let mut sim = Sim::new(entry, &cfg).unwrap();
        sim.command(Command::Disable).unwrap();
        for (seconds, by_hand) in [(0.5, 14.715), (1.0, 5.413)] {
            sim.run(seconds - sim.time(), |_| {});
            for d in &sim.control.drives {
                assert!(
                    (d.temperature_rise - by_hand).abs() < 1e-3 * by_hand,
                    "{}: {} K after {seconds} s, by hand {by_hand} K",
                    entry.id,
                    d.temperature_rise
                );
            }
        }
        // Enabled again, it stays where it cooled to and warms from there.
        sim.command(Command::Enable).unwrap();
        assert!(sim.control.drives.iter().all(|d| (d.temperature_rise - 5.413).abs() < 1e-2), "{}", entry.id);
        let mut cold = Sim::new(entry, &with(&cfg, &["effects.winding_heating = false"])).unwrap();
        cold.command(Command::Disable).unwrap();
        cold.run(0.5, |_| {});
        assert!(cold.control.drives.iter().all(|d| d.temperature_rise == 0.0), "{}", entry.id);
    }
}

/// A disabled drive's DC link discharges through its bleeder, C dU/dt = -U / R_b: U = U0 exp(-t / (R_b C)), down to
/// the bus level, where the rectifier holds it. With tau = R_b C = 0.2 s and 220 V at the moment the drives go off (below
/// the chopper's threshold, which would burn the excess first), by hand 220 V exp(-0.25) = 171.336 V after 0.05 s, and
/// the bus level of 155.13 V after tau ln(220 / 155.13) = 69.9 ms, from then on held. A shared link discharges the same
/// way. A regenerative supply takes what is above the bus level at once (from 160 V, 0.36 J, in one step). On a config
/// with another bus level (540 V, a catalogue motor's) every voltage is scaled with it, 220 V to 765.8 V: the times stay.
#[test]
fn a_disabled_drives_dc_link_discharges_by_the_rc_exponential() {
    for entry in MACHINES {
        for shared in [false, true] {
            let cfg = with(
                &entry.config(),
                &["ratings.dc_link_discharge_time_constant_s = 0.2", &format!("effects.shared_dc_link = {shared}")],
            );
            let mut sim = Sim::new(entry, &cfg).unwrap();
            let bus = cfg.f64("drive.bus_voltage_v").unwrap();
            let scale = bus / 155.13;
            sim.command(Command::Disable).unwrap();
            // The first disabled step hands the holding current's energy to the link; from then on it only discharges.
            sim.step();
            sim.control.drives.iter_mut().for_each(|d| d.dc_voltage = 220.0 * scale);
            sim.run(0.05, |_| {});
            for d in &sim.control.drives {
                assert!(
                    (d.dc_voltage - 171.336 * scale).abs() < 1e-3 * scale,
                    "{} shared {shared}: {} V after 0.05 s",
                    entry.id,
                    d.dc_voltage
                );
            }
            sim.run(0.019, |_| {});
            assert!(
                sim.control.drives.iter().all(|d| d.dc_voltage > bus),
                "{} shared {shared}: at the bus level before 69.9 ms",
                entry.id
            );
            sim.run(0.0015, |_| {});
            assert!(
                sim.control.drives.iter().all(|d| d.dc_voltage == bus),
                "{} shared {shared}: not at the bus level after 70.5 ms",
                entry.id
            );
            sim.run(0.1, |_| {});
            assert!(
                sim.control.drives.iter().all(|d| d.dc_voltage == bus && !d.tripped),
                "{} shared {shared}",
                entry.id
            );
        }
        let mut regenerative =
            Sim::new(entry, &with(&entry.config(), &["effects.regenerative_supply = true"])).unwrap();
        regenerative.command(Command::Disable).unwrap();
        let bus = entry.config().f64("drive.bus_voltage_v").unwrap();
        let start = 160.0 * bus / 155.13;
        regenerative.control.drives.iter_mut().for_each(|d| d.dc_voltage = start);
        // At once, or as fast as the line module's highest power takes the charge above the bus: a 48 V track's 2.3 kW
        // take its six movers' shared 6 x 4.7 mF from 49.5 V in 15 steps (2.06 J), the 540 V machines' 35 kW theirs in
        // one. A shared link has one line module for the capacitors of all its drives.
        let cfg = entry.config();
        let rating = |key: &str| cfg.f64(&format!("ratings.{key}"));
        let links =
            if cfg.bool("effects.shared_dc_link").unwrap_or(false) { regenerative.control.drives.len() } else { 1 };
        let charge =
            links as f64 * 0.5 * rating("dc_link_capacitance_uf").unwrap_or(0.0) * 1e-6 * (start * start - bus * bus);
        let most = (charge / (0.98 * rating("line_module_max_power_w").unwrap_or(f64::INFINITY) * regenerative.dt))
            .ceil()
            .max(1.0) as usize;
        for _ in 0..most {
            regenerative.step();
            if regenerative.control.drives.iter().all(|d| d.dc_voltage <= bus) {
                break;
            }
        }
        assert!(
            regenerative
                .control
                .drives
                .iter()
                .all(|d| d.dc_voltage == entry.config().f64("drive.bus_voltage_v").unwrap() && d.mains_power > 0.0),
            "{}",
            entry.id
        );
    }
}

/// The energy in the windings (½ L i²) and the DC link capacitors (½ C U²) across Disable and Enable, on every machine,
/// own and shared link: the current in the windings when the pulses stop goes into the DC link, and the link keeps its
/// charge across Enable, less what the holding current takes. Mid-move (current in the windings) with the link at bus
/// + 20 V (below the chopper, above the rectifier) and no bleeder, nothing goes anywhere else: conserved to 1e-9.
#[test]
fn disable_and_enable_conserve_the_electrical_energy() {
    for entry in MACHINES {
        for shared in [false, true] {
            let cfg = with(
                &entry.config(),
                &[
                    "effects.dc_link = true",
                    "ratings.dc_link_discharge_time_constant_s = 1e12",
                    &format!("effects.shared_dc_link = {shared}"),
                ],
            );
            let mut sim = Sim::new(entry, &cfg).unwrap();
            let bus = cfg.f64("drive.bus_voltage_v").unwrap();
            sim.command(Command::StartCycle).unwrap();
            sim.run(0.05, |_| {});
            // On into the move until the windings hold some energy (the parallel SCARA's 7.6 uJ at 0.05 s were not).
            let energy = |sim: &Sim| {
                (0..sim.control.drives.len())
                    .map(|k| 0.5 * sim.control.drive_params_of(k).inductance * sim.control.drives[k].current.powi(2))
                    .sum::<f64>()
            };
            while energy(&sim) <= 1e-5 && sim.time() < 2.0 {
                sim.step();
            }
            sim.control.drives.iter_mut().for_each(|d| d.dc_voltage = bus + 20.0);
            // Each motor's own inductance and capacitance (a motor of its own table has its own).
            let p: Vec<_> = (0..sim.control.drives.len()).map(|k| sim.control.drive_params_of(k).clone()).collect();
            let stored = |sim: &Sim| {
                sim.control
                    .drives
                    .iter()
                    .zip(&p)
                    .map(|(d, p)| {
                        0.5 * p.inductance * d.current * d.current
                            + 0.5 * p.dc_capacitance * d.dc_voltage * d.dc_voltage
                    })
                    .sum::<f64>()
            };
            let windings: f64 =
                sim.control.drives.iter().zip(&p).map(|(d, p)| 0.5 * p.inductance * d.current * d.current).sum();
            let before = stored(&sim);
            sim.command(Command::Disable).unwrap();
            sim.step();
            let off = stored(&sim);
            sim.command(Command::Enable).unwrap();
            let on = stored(&sim);
            assert!(
                windings > 1e-5 && sim.control.drives.iter().all(|d| d.resistor_power == 0.0),
                "{} shared {shared}: {windings} J in the windings",
                entry.id
            );
            assert!(
                (off - before).abs() < 1e-9 * before && (on - off).abs() < 1e-9 * before,
                "{} shared {shared}: {before} J, disabled {off} J, enabled {on} J",
                entry.id
            );
        }
    }
}

/// The two line module checks, only with the regenerative supply: the mean power through the line module either way
/// (drawn and fed back) over the cycle against its rated power, the highest against its maximum power (the config's: 16 kW
/// and 35 kW for a 540 V machine, a 48 V track's own module less). A line module rated at 1 W fails the first.
#[test]
fn the_line_module_is_judged_against_its_ratings() {
    for entry in MACHINES {
        let judged = |cfg: &Config| {
            let load = analyse_cycle_with(&mut Sim::new(entry, cfg).unwrap(), false).unwrap();
            let all = verdicts(cfg, &load).unwrap();
            (load, all)
        };
        let (_, off) = judged(&entry.config());
        assert!(!off.iter().any(|v| v.check.starts_with("line-module")), "{}", entry.id);
        let cfg = with(&entry.config(), &["effects.regenerative_supply = true"]);
        let (load, all) = judged(&cfg);
        let rating = |key: &str| cfg.f64(&format!("ratings.line_module_{key}_power_w")).unwrap();
        for (k, m) in load.motors.iter().enumerate() {
            let find = |check: &str| all.iter().find(|v| v.check == check && v.motor == Some(k)).unwrap();
            let (continuous, peak) = (find("line-module-continuous"), find("line-module-peak"));
            assert!(
                continuous.value == m.line_module_energy / load.duration
                    && continuous.limit == rating("rated")
                    && continuous.ok,
                "{}: {continuous:?}",
                entry.id
            );
            assert!(
                peak.value == m.peak_line_module_power
                    && peak.limit == rating("max")
                    && peak.ok
                    && peak.value > continuous.value,
                "{}: {peak:?}",
                entry.id
            );
            println!(
                "{} motor {k}: {:.2} W mean, {:.1} W highest through the line module, {:.3} J fed back",
                entry.id, continuous.value, peak.value, m.mains_energy
            );
        }
        let (_, weak) = judged(&with(&cfg, &["ratings.line_module_rated_power_w = 1.0"]));
        assert!(weak.iter().any(|v| v.check == "line-module-continuous" && !v.ok), "{}", entry.id);
    }
}

/// The braking resistor's mean power estimated without the simulated DC link (each braking fills the link's capacitors,
/// what is above their room goes to the resistor, what the motor draws between empties them) stays on the safe side of
/// the simulated link's: on the T-gantry's motor b the audit's run gave 0.90 W simulated, where the old estimate (all
/// that is fed back) said 2.56 W. Now (16 November 2026) both give 0.9577 W, to 1e-13: the simulated link keeps the same
/// energy account (it has no losses of its own), so the estimate is that link's figure; all of it fed back is 2.61 W.
#[test]
fn the_estimated_braking_resistor_power_is_not_below_the_simulated_one() {
    let entry = sim_machines::find("t-gantry").unwrap();
    let mean = |dc_link: bool| {
        let cfg =
            with(&entry.config(), &[&format!("effects.dc_link = {dc_link}"), "effects.thermal_warm_start = false"]);
        let load = analyse_cycle_with(&mut Sim::new(entry, &cfg).unwrap(), false).unwrap();
        let all = verdicts(&cfg, &load).unwrap();
        let found = all.iter().find(|v| v.check == "braking-resistor-continuous" && v.motor == Some(1));
        (found.map_or(0.0, |v| v.value), load.motors[1].energy_regenerated / load.duration)
    };
    let ((estimated, fed_back), (simulated, _)) = (mean(false), mean(true));
    println!("motor b: resistor {estimated} W estimated, {simulated} W simulated, {fed_back} W fed back");
    assert!(simulated > 0.0 && estimated >= 0.97 * simulated && estimated < fed_back, "{estimated} {simulated}");
}
