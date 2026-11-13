//! The effect views, part 2, against hand calculations: gearboxes and guides warming with their losses, the belt
//! sections at rest and in a static hold, and the comparison of the cycle with an effect on and off.

use sim_components::Sim;
use sim_components::analysis::{compare_configs, compare_effect};
use sim_core::{Config, MachineEntry, Scene};
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

/// A guide at constant speed loses P = (lambda F_c + b v) v; its rise settles at R P and closes the gap to it by
/// the factor e every time constant. With effects.part_heating off nothing warms, and the parts keep a rise of 0.
#[test]
fn a_guide_at_constant_speed_warms_to_resistance_times_its_loss() {
    for entry in MACHINES.iter().copied() {
        // On a machine with an X guide that warms (not the delta's arms).
        if !(entry.build)(&entry.config()).unwrap().thermal_parts().iter().any(|p| p.id == "guide_x") {
            continue;
        }
        // A short time constant, so that the guide settles within the cruise of a move.
        let mut cfg = edited(entry, &["effects.part_heating = true", "thermal.guide_time_constant_s = 0.02", "thermal.guide_resistance_k_w = 2.0"]);
        let machine = (entry.build)(&cfg).unwrap();
        let axis = &machine.axes()[0];
        let mut start: Vec<f64> = machine.axes().iter().map(|a| 0.5 * (a.min + a.max)).collect();
        start[0] = axis.min + 0.05 * (axis.max - axis.min);
        cfg.set_text(&format!("start.pose = {start:?}")).unwrap();
        let mut sim = Sim::new(entry, &cfg).unwrap();
        let part = sim.heated.iter().position(|h| h.part.id == "guide_x").expect("an X guide that warms");
        let mut target = start.clone();
        target[0] = axis.max - 0.05 * (axis.max - axis.min);
        sim.command(sim_components::Command::MoveTo(target)).unwrap();
        let (top, tau) = (axis.max_vel, 0.02);
        let cruise = |sim: &mut Sim, seconds: f64| {
            for _ in 0..(seconds / sim.dt).round() as usize {
                sim.step();
                assert!((signal(sim, "axis.x.velocity_setpoint") - top).abs() < 1e-9, "{}: the move stopped cruising", entry.id);
            }
        };
        // Up to the cruise, and 0.15 s into it.
        while (signal(&sim, "axis.x.velocity_setpoint") - top).abs() > 1e-9 {
            sim.step();
        }
        cruise(&mut sim, 0.15);
        // From cold, one time constant later the guide has (1 - 1/e) of R P.
        sim.heated[part].rise = 0.0;
        cruise(&mut sim, tau);
        let by_hand = 2.0 * signal(&sim, "power.guide_x") * (1.0 - (-1f64).exp());
        let rise = signal(&sim, "temperature.guide_x");
        assert!((rise - by_hand).abs() < 0.01 * by_hand, "{}: {rise} K after one time constant against {by_hand} K", entry.id);
        // Twelve time constants later it has settled at R P, P = (lambda F_c + b v) v by hand from the config.
        cruise(&mut sim, 12.0 * tau);
        let (scale, v) = (signal(&sim, "axis.x.friction_load_scale"), signal(&sim, "axis.x.friction_speed"));
        let force = scale * cfg.f64("friction_x.coulomb_n").unwrap() + cfg.f64("friction_x.viscous_n_s_m").unwrap() * v;
        let settled = 2.0 * force * v;
        let rise = signal(&sim, "temperature.guide_x");
        assert!((rise - settled).abs() < 0.01 * settled, "{}: {rise} K against 2 K/W * {force} N * {v} m/s = {settled} K", entry.id);
        assert_eq!(rise, sim.heated[part].rise);
        // Switched off, nothing warms.
        let mut sim = Sim::new(entry, &entry.config()).unwrap();
        sim.command(sim_components::Command::StartCycle).unwrap();
        sim.run(0.5, |_| {});
        assert!(!sim.part_heating && sim.heated.iter().all(|h| h.rise == 0.0));
    }
}

/// The 3D view colours a heated part and a motor housing by rise over rated rise.
#[test]
fn the_scene_colours_warm_parts_by_their_rise_over_the_rating() {
    for entry in MACHINES.iter().copied() {
        let cfg = edited(entry, &["effects.part_heating = true", "motor.start_temperature_rise_k = 25.0", "motor.rated_temperature_rise_k = 100.0"]);
        let mut sim = Sim::new(entry, &cfg).unwrap();
        // A screw on a coupling, a linear motor: no part but the motor warms.
        let parts = !sim.heated.is_empty();
        if parts {
            sim.heated[0].rise = 0.5 * sim.heated[0].max_rise;
        }
        let mut scene = Scene::default();
        sim.scene(&mut scene);
        let motors: Vec<_> = scene.parts.iter().filter(|p| p.id == sim_core::scene::MOTOR_PART).collect();
        assert_eq!(motors.len(), sim.machine.motors().len());
        assert!(motors.iter().all(|p| p.heat == Some(0.25)), "{}: {:?}", entry.id, motors.iter().map(|p| p.heat).collect::<Vec<_>>());
        if parts {
            let first = &sim.heated[0].part;
            assert_eq!(scene.parts.iter().filter(|p| p.id == first.scene_part).nth(first.scene_index).unwrap().heat, Some(0.5));
        }
        // Without the heating effects the parts keep their colours.
        let mut scene = Scene::default();
        Sim::new(entry, &edited(entry, &["effects.winding_heating = false"])).unwrap().scene(&mut scene);
        assert!(scene.parts.iter().all(|p| p.heat.is_none()));
    }
}

/// At rest without gravity every section has the pretension, and is stretched by L P / (EA + P) over its length L.
/// Held against gravity, the sections balance the held weight: on the single axis inclined by 30 degrees
/// T2 - T1 = m g sin 30; on the T-gantry, whose Z axis hangs in a loop of section 2, 2 T2 - T1 - T3 = m_z g and
/// T1 = T3. The 3D view colours each section by (T - P) / P, from the same tensions.
#[test]
fn belt_sections_have_the_pretension_at_rest_and_carry_the_weight_in_a_static_hold() {
    for entry in MACHINES.iter().copied() {
        let sim = Sim::new(entry, &edited(entry, &["effects.gravity = false"])).unwrap();
        let sections = sim.machine.belt_sections();
        // A machine without belts reports no sections. The hand calculation below is the single axis's and the
        // T-gantry's; the linear delta's belts are checked in its own tests.
        if sections.is_empty() || entry.id == "delta-linear" {
            continue;
        }
        assert!(sections.len() >= 2, "{}", entry.id);
        let (p, ea) = (300.0, 5.0e5);
        for section in &sections {
            assert_eq!(section.pretension, p);
            let tension = signal(&sim, &format!("{}.tension", section.id));
            assert!((tension - p).abs() < 1e-6, "{} {}: {tension} N", entry.id, section.id);
        }
        // The stretch, for the section from the motor to the carriage or clamp: on both machines 2 s_min + start x + z.
        let first = signal(&sim, "belt.1.stretch");
        let pose = sim.machine.axes().iter().map(|a| a.id).collect::<Vec<_>>();
        let start = entry.config().f64s("start.pose").unwrap();
        let length = if pose.len() == 1 { 0.15 + start[0] } else { 2.0 * 0.15 + start[0] + start[1] };
        assert!((first - length * p / (ea + p)).abs() < 1e-12, "{}: {first} m against {} m", entry.id, length * p / (ea + p));

        let held = if pose.len() == 1 { edited(entry, &["geometry.incline_deg = 30.0"]) } else { entry.config() };
        let sim = Sim::new(entry, &held).unwrap();
        let t: Vec<f64> = sections.iter().map(|s| signal(&sim, &format!("{}.tension", s.id))).collect();
        let g = 9.80665;
        if pose.len() == 1 {
            // Carriage and payload, 4 + 2 kg.
            let weight = 6.0 * g * 0.5;
            assert!((t[1] - t[0] - weight).abs() < 1e-6, "{}: {t:?} against {weight} N", entry.id);
        } else {
            // Z axis and payload, 5 + 2 kg: 68.6466 N.
            let weight = 7.0 * g;
            assert!((2.0 * t[1] - t[0] - t[2] - weight).abs() < 1e-6 && (t[0] - t[2]).abs() < 1e-6, "{}: {t:?} against {weight} N", entry.id);
        }
        let mut scene = Scene::default();
        sim.scene(&mut scene);
        let shades: Vec<f32> = scene.parts.iter().filter(|p| p.id == "belt").map(|p| p.highlight).collect();
        for tension in &t {
            let shade = ((tension - p) / p) as f32;
            assert!(shades.contains(&shade), "{}: no strip with {shade} among {shades:?}", entry.id);
        }
        assert!(shades.iter().all(|s| t.iter().any(|t| ((t - p) / p) as f32 == *s)));
    }
}

/// The cycle against itself differs in nothing; guide friction on against off at constant speed differs in the motor
/// torque by r / i times the friction force (lambda F_c + b v), divided by the gearbox efficiency the motor drives
/// it through (the mean over 0.1 s of the cruise).
#[test]
fn comparing_friction_on_and_off_gives_the_friction_torque() {
    for entry in MACHINES.iter().copied() {
        let cfg = entry.config();
        let same = compare_configs(entry, &cfg, &cfg).unwrap();
        assert!(same.on.len() > 500 && same.on == same.off, "{}", entry.id);
        // The friction of an X guide (not the delta's joints, whose friction its own tests check).
        if !(entry.build)(&cfg).unwrap().friction_contacts().iter().any(|c| c.id == "axis.x") {
            continue;
        }
        // Without torque ripple and tooth meshing, whose periodic torques the controller answers within a sample.
        let cfg = edited(entry, &["effects.torque_ripple = false", "effects.tooth_meshing = false"]);
        assert!(same.on.len() > 500 && same.on == same.off, "{}", entry.id);
        assert!((0..same.signals.len()).all(|i| same.difference(i) == (0.0, 0.0)));

        let found = compare_effect(entry, &cfg, "guide_friction").unwrap();
        let place = |id: &str| found.signals.iter().position(|s| s.id == id).unwrap();
        let machine = (entry.build)(&cfg).unwrap();
        let (set, acc) = (place("axis.x.velocity_setpoint"), place("axis.x.acceleration_setpoint"));
        let (scale, speed) = (place("axis.x.friction_load_scale"), place("axis.x.friction_speed"));
        let top = machine.axes()[0].max_vel;
        // The middle of the first stretch of 0.2 s at which X cruises at its top speed with every other axis at rest,
        // on both runs.
        let cruising = |row: &Vec<f64>| {
            let others = found.signals.iter().zip(row).filter(|(s, _)| s.id.ends_with(".velocity_setpoint") && s.id != "axis.x.velocity_setpoint");
            (row[set].abs() - top).abs() < 1e-9 && row[acc] == 0.0 && others.into_iter().all(|(_, v)| *v == 0.0)
        };
        let rows: Vec<usize> = (0..found.on.len().min(found.off.len())).filter(|&i| cruising(&found.on[i]) && cruising(&found.off[i])).collect();
        let run = rows.windows(200).find(|w| w[199] - w[0] == 199).unwrap_or_else(|| panic!("{}: no cruise of 0.2 s", entry.id));
        let (on, off) = (&found.on[run[150]], &found.off[run[150]]);
        let (r, ratio, eta) = (cfg.f64("geometry.pulley_radius_m").unwrap(), cfg.f64("gearbox.ratio").unwrap(), cfg.f64("gearbox.efficiency").unwrap());
        let v = on[speed];
        let force = (on[scale] * cfg.f64("friction_x.coulomb_n").unwrap() + cfg.f64("friction_x.viscous_n_s_m").unwrap() * v.abs()) * v.signum();
        // The T-gantry moves X with both motors, each giving half: k_r = r / (2 i).
        let share = 1.0 / machine.motors().len() as f64;
        // On the T-gantry the Z guide's friction is switched with it. Z stands still, so its bristles hold what they
        // held when Z stopped: that force is read from its signal, and goes to motor A with +k_r and to B with -k_r.
        let held_z = found.signals.iter().position(|s| s.id == "axis.z.friction").map_or(0.0, |i| on[i]);
        let mut checked = 0;
        for (k, motor) in machine.motors().iter().enumerate() {
            let (torque, turn) = (place(&format!("motor.{}.torque", motor.id)), place(&format!("motor.{}.speed", motor.id)));
            // Through the gearbox: divided by the efficiency where the motor drives it, times it where it is driven back.
            let factor = match (on[torque] * on[turn] > 0.0, off[torque] * off[turn] > 0.0) {
                (true, true) => 1.0 / eta,
                (false, false) => eta,
                _ => continue,
            };
            let by_hand = share * r / ratio * (force + [1.0, -1.0][k] * held_z) * factor;
            // Over the second half of the cruise's first 0.2 s: the difference swings by about 3 % around its mean with
            // the belt mode (about 22 Hz on the T-gantry), which the velocity loop no longer damps away within 0.15 s.
            let difference = run[100..200].iter().map(|&i| found.on[i][torque] - found.off[i][torque]).sum::<f64>() / 100.0;
            println!("{} motor {}: {difference:.4} against {by_hand:.4} N m", entry.id, motor.id);
            assert!((difference - by_hand).abs() < 0.02 * by_hand.abs(), "{} motor {} at {} s: {difference} N m against {by_hand} N m (v {v} m/s, F_z {held_z} N)", entry.id, motor.id, on[0]);
            let (peak, rms) = found.difference(torque);
            assert!(peak >= difference.abs() && rms > 0.0 && rms <= peak);
            checked += 1;
        }
        assert!(checked > 0, "{}: no motor kept its direction of power", entry.id);
        assert!(compare_effect(entry, &cfg, "no_such_effect").is_err());
    }
}
