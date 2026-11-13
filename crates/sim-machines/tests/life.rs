//! The life estimates on every machine: what the analysis adds up for them is what the signals say, and the verdicts
//! judge the lives against the required life.

use sim_components::analysis::{SPEED_BAND, analyse_trajectory, sensitivity, belt_tooth_utilisation, gearbox_life_h, guide_life_h, pulley_bearing_life_h, verdicts};
use sim_components::path::Piece;
use sim_components::{Command, Sim};
use sim_machines::MACHINES;

/// A move from the start pose to the middle of every stroke, analysed; then the same move again, its signals added up
/// here: per motor the mean speed (every step), the cubic mean of the pulley shaft's radial load weighted with the speed
/// (once per millisecond), the mean of the gearbox output torque with the exponent 10/3 weighted with the speed and its
/// peak per speed band (every step); per guide the mean speed of its axis (once per millisecond). They agree to 1e-9.
#[test]
fn the_life_loads_are_the_signals_added_up() {
    for entry in MACHINES.iter().copied() {
        let cfg = entry.config();
        let mut sim = Sim::new(entry, &cfg).unwrap();
        let middle: Vec<f64> = sim.machine.axes().iter().map(|a| 0.5 * (a.min + a.max)).collect();
        let path = sim.plan_path(&[Piece::Line(middle)], false).unwrap();
        let load = analyse_trajectory(&mut sim, path.clone()).unwrap();

        let mut sim = Sim::new(entry, &cfg).unwrap();
        let end = path[path.len() - 1].0 + 0.3;
        sim.command(Command::Trajectory(path)).unwrap();
        let infos = sim.signal_infos();
        let find = |id: String| infos.iter().position(|s| s.id == id);
        let motors = sim.machine.motors().to_vec();
        let axes = sim.machine.axes().to_vec();
        // A machine without pulleys (the delta's arms) reports no radial load, one without guides no guide load.
        let radial: Vec<Option<usize>> = motors.iter().map(|m| find(format!("pulley.{}.radial_load", m.id))).collect();
        let gear: Vec<usize> = motors.iter().map(|m| find(format!("gearbox.{}.output_torque", m.id)).unwrap()).collect();
        let guides: Vec<(usize, usize)> = axes.iter().enumerate().filter_map(|(a, axis)| Some((find(format!("axis.{}.guide_load", axis.id))?, a))).collect();
        let every = (1e-3 / sim.dt).round().max(1.0) as u64;
        let n = motors.len();
        let (mut speed_sum, mut cubed, mut weight, mut life, mut bands) = (vec![0.0; n], vec![0.0; n], vec![0.0; n], vec![0.0; n], vec![vec![0.0f64; 4001]; n]);
        let (mut guide_cubed, mut guide_speed, mut samples) = (vec![0.0; guides.len()], vec![0.0; guides.len()], 0.0);
        let (mut pos, mut speed, mut values) = (vec![0.0; n], vec![0.0; n], Vec::new());
        let (mut tool, mut rate) = (vec![0.0; axes.len()], vec![0.0; axes.len()]);
        let (start, mut steps) = (sim.time(), 0u64);
        while sim.time() - start < end {
            sim.step();
            sim.machine.motor_state(&sim.state, &mut pos, &mut speed);
            sim.sample(&mut values);
            for k in 0..n {
                let (w, torque) = (speed[k].abs(), values[gear[k]].abs());
                speed_sum[k] += w;
                life[k] += torque.powf(10.0 / 3.0) * w;
                let band = ((w / SPEED_BAND) as usize).min(4000);
                bands[k][band] = bands[k][band].max(torque);
                if let (0, Some(r)) = (steps % every, radial[k]) {
                    (cubed[k], weight[k]) = (cubed[k] + values[r].powi(3) * w, weight[k] + w);
                }
            }
            if steps % every == 0 {
                sim.machine.tool_state(&sim.state, &mut tool, &mut rate);
                for (g, &(signal, axis)) in guides.iter().enumerate() {
                    guide_cubed[g] += values[signal].powi(3);
                    guide_speed[g] += rate[axis].abs();
                }
                samples += 1.0;
            }
            steps += 1;
        }
        let close = |a: f64, b: f64, what: &str| assert!((a - b).abs() <= 1e-9 * b.abs().max(1e-12), "{}: {what} {a} against {b}", entry.id);
        for k in 0..n {
            let m = &load.motors[k];
            close(m.mean_speed, speed_sum[k] / steps as f64, "mean speed");
            assert_eq!(m.radial_load_equivalent.is_some(), radial[k].is_some(), "{}", entry.id);
            if radial[k].is_some() {
                close(m.radial_load_equivalent.unwrap(), (cubed[k] / weight[k]).cbrt(), "equivalent radial load");
            }
            close(m.gearbox_life_torque.unwrap(), (life[k] / speed_sum[k]).powf(0.3), "equivalent gearbox torque");
            let peaks = &m.gearbox_torque_by_speed;
            assert!(!peaks.is_empty() && bands[k][peaks.len()..].iter().all(|&t| t == 0.0), "{}: bands", entry.id);
            for (i, &peak) in peaks.iter().enumerate() {
                close(peak, bands[k][i], "torque in a speed band");
            }
        }
        if let Some(g) = (0..guides.len()).max_by(|&a, &b| guide_cubed[a].total_cmp(&guide_cubed[b])) {
            close(load.guide_load.unwrap().1, (guide_cubed[g] / samples).cbrt(), "guide load");
            close(load.guide_speed.unwrap(), guide_speed[g] / samples, "guide speed");
        } else {
            assert!(load.guide_load.is_none() && load.guide_speed.is_none(), "{}", entry.id);
        }
        println!("{}: motor speed {:.2} rad/s, radial {:?} N, guide {:?} m/s", entry.id, load.motors[0].mean_speed, load.motors[0].radial_load_equivalent, load.guide_speed);
    }
}

/// On every shipped config the verdicts carry the lives and the tooth load as the functions give them, and a life
/// passes from its required life upwards: required just above the life fails it, just below passes.
#[test]
fn the_life_verdicts_judge_the_lives_against_the_required_life() {
    let mut configs: Vec<_> = MACHINES.iter().map(|e| (*e, e.config())).collect();
    configs.push((&machine_t_gantry::ENTRY, sim_components::load_config(&machine_t_gantry::ENTRY, machine_t_gantry::EXCT_30_CONFIG).unwrap()));
    for (entry, cfg) in configs {
        let load = sim_components::analysis::analyse_cycle_with(&mut Sim::new(entry, &cfg).unwrap(), false).unwrap();
        let judged = verdicts(&cfg, &load).unwrap();
        let find = |v: &[sim_components::analysis::Verdict], check: &str, motor: Option<usize>| v.iter().find(|v| v.check == check && v.motor == motor).cloned();
        let required = cfg.f64("ratings.required_life_h").unwrap();
        // Each life is judged where the machine reports what it needs: a delta has no guide, pulley or belt.
        let guide = guide_life_h(&cfg, &load);
        assert_eq!(find(&judged, "guide-life", None).map(|v| (v.value, v.limit)), guide.map(|g| (g, required)), "{}", entry.id);
        for (k, m) in load.motors.iter().enumerate() {
            // The EXCT-30 has no gearbox and no gearbox rating: its gearbox life is not judged.
            assert_eq!(find(&judged, "gearbox-life", Some(k)).map(|v| v.value), gearbox_life_h(&cfg, m), "{}", entry.id);
            let Some(bearing) = pulley_bearing_life_h(&cfg, m) else {
                assert!(find(&judged, "pulley-bearing-life", Some(k)).is_none() && belt_tooth_utilisation(&cfg, m).unwrap().is_none(), "{}", entry.id);
                let life = gearbox_life_h(&cfg, m).unwrap();
                for (factor, ok) in [(1.001, false), (0.999, true)] {
                    let mut edited = cfg.clone();
                    edited.set_text(&format!("ratings.required_life_h = {:?}", life * factor)).unwrap();
                    assert_eq!(find(&verdicts(&edited, &load).unwrap(), "gearbox-life", Some(k)).unwrap().ok, ok, "{}", entry.id);
                }
                continue;
            };
            assert_eq!(find(&judged, "pulley-bearing-life", Some(k)).map(|v| v.value), Some(bearing), "{}", entry.id);
            let tooth = belt_tooth_utilisation(&cfg, m).unwrap().unwrap();
            assert_eq!(find(&judged, "belt-tooth-load", Some(k)).map(|v| v.value), Some(tooth), "{}", entry.id);
            for (factor, ok) in [(1.001, false), (0.999, true)] {
                let mut edited = cfg.clone();
                edited.set_text(&format!("ratings.required_life_h = {:?}", bearing * factor)).unwrap();
                assert_eq!(find(&verdicts(&edited, &load).unwrap(), "pulley-bearing-life", Some(k)).unwrap().ok, ok, "{}", entry.id);
            }
        }
        // A reliability the standard (ISO 281) has no life factor for is refused by name.
        let mut unreliable = cfg.clone();
        unreliable.set_text("ratings.bearing_reliability_percent = 99.99").unwrap();
        assert_eq!(verdicts(&unreliable, &load).err(), Some(sim_core::ConfigError::Invalid("ratings.bearing_reliability_percent".into())), "{}", entry.id);
        println!("{}: guide {guide:?} h, pulley bearing {:?} h, gearbox {:?} h, tooth load {:?}", entry.id, pulley_bearing_life_h(&cfg, &load.motors[0]), gearbox_life_h(&cfg, &load.motors[0]), belt_tooth_utilisation(&cfg, &load.motors[0]).unwrap());
    }
}

/// `sensitivity` leaves out the reliability of the life estimates: 20 % either way of the shipped 90 % is 72 % and 108 %,
/// which ISO 281 has no life factor for (`verdicts` refuses them), and it is a choice of how to judge, not machine data.
#[test]
fn sensitivity_leaves_the_bearing_reliability_alone() {
    let entry = MACHINES[0];
    let paths = ["ratings.bearing_reliability_percent".to_string()];
    let (_, rows) = sensitivity(entry, &entry.config(), &paths, 0.2).unwrap();
    assert!(rows.is_empty(), "{:?}", rows.iter().map(|r| (r.path.clone(), r.bounds)).collect::<Vec<_>>());
}
