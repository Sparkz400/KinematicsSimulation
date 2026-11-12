//! Analysis for motor sizing: peak and RMS torque per cycle, operating points against the
//! torque-speed curve, and the mechanical frequency response.

use crate::control::{Command, CommandError};
use crate::sim::Sim;
use sim_core::{Config, ConfigError, MachineEntry, Rk4, advance};

/// The load on one motor over one pass of the cycle.
#[derive(Debug, Clone, Default)]
pub struct MotorLoad {
    pub peak_torque: f64,
    pub rms_torque: f64,
    /// The torque gearbox makers size a gearbox's continuous rating with: the cube root of the mean of the
    /// torque cubed, weighted with the speed, so that a turn under load counts and standing still does not.
    pub cubic_mean_torque: f64,
    pub peak_speed: f64,
    /// Highest ratio of the torque the velocity loop asks for to the torque the drive has available at
    /// that speed. Above 1 the cycle asks for more than the torque-speed curve gives.
    pub utilisation: f64,
    /// RMS torque over the torque the motor can give continuously. Above 1 the motor overheats on this cycle.
    pub thermal_utilisation: f64,
    pub rms_current: f64,
    /// Temperature rise the winding settles at when the cycle repeats for long.
    pub winding_rise: f64,
    /// Electrical energy over the cycle: drawn from the DC link, and fed back into it while braking.
    pub energy_in: f64,
    pub energy_regenerated: f64,
    /// Highest power fed back, and the most energy fed back in one go: what the DC link capacitors or a
    /// braking resistor have to take.
    pub peak_regenerated_power: f64,
    pub largest_regeneration: f64,
    /// With the DC link simulated: the energy the braking resistor burned (over the cycle's duration its
    /// mean power), the most power it took and the highest DC link voltage. For a drive that tripped on
    /// overvoltage, the voltage it tripped at.
    pub resistor_energy: f64,
    pub peak_resistor_power: f64,
    pub peak_dc_link_voltage: f64,
    /// The torque at the gearbox output, where the machine reports it (signal `gearbox.<motor>.output_torque`): its
    /// peak, and its cubic mean weighted with the speed. Not the motor torque times the ratio: the rotor's
    /// own acceleration does not pass through the gearbox, and its losses come off or on top.
    pub gearbox_torque: Option<(f64, f64)>,
    /// (speed, torque) once per millisecond, for plotting over the torque-speed curve.
    pub points: Vec<(f64, f64)>,
}

#[derive(Debug, Clone)]
pub struct CycleLoad {
    pub duration: f64,
    /// Lowest and highest tension of any belt section during the cycle, for machines that report
    /// belt tensions. A lowest tension of zero means a section went slack.
    pub belt_tension: Option<(f64, f64)>,
    /// Seconds any belt section spent without tension, where the machine counts them.
    pub belt_slack_time: Option<f64>,
    /// The highest radial load a belt put on a pulley shaft, where the machine reports it.
    pub radial_load_peak: Option<f64>,
    /// Of the guide with the highest load, where the machine reports guide loads: its highest load and
    /// its cubic mean over the cycle, the equivalent load of the guide makers' life formula.
    pub guide_load: Option<(f64, f64)>,
    /// How far the tool was from the path its setpoint drew: the most and the RMS. And the most it was
    /// from where the setpoint was at the same moment, which also counts lagging behind along the path.
    pub path_error_peak: f64,
    pub path_error_rms: f64,
    pub tracking_error_peak: f64,
    /// The most the tool was from its target in the second half of a dwell: by then it should have settled,
    /// because that is when the gripper acts.
    pub dwell_error_peak: f64,
    /// A drive tripped on DC link overvoltage and took all drives off: the run ended there, and the figures
    /// are those up to the trip.
    pub tripped: bool,
    pub motors: Vec<MotorLoad>,
}

/// What a run adds up step by step, for a cycle or a trajectory.
struct Tally {
    loads: Vec<MotorLoad>,
    pos: Vec<f64>,
    speed: Vec<f64>,
    steps: u64,
    /// Steps per millisecond: belt tensions, operating points and the path are sampled that often.
    every: u64,
    tensions: Vec<usize>,
    /// Signals of the pulley shafts' radial loads and of the guide loads, and the slack-time counter.
    radial: Vec<usize>,
    guides: Vec<usize>,
    slack: Option<usize>,
    sample: Vec<f64>,
    belt_tension: Option<(f64, f64)>,
    radial_peak: f64,
    /// Per guide: highest load and the sum of the load cubed, one per sample.
    guide_loads: Vec<(f64, f64)>,
    /// The slack-time counter when the measurement began.
    slack_start: f64,
    /// Per motor: sums of |torque|^3 |speed| and of |speed|.
    cubic: Vec<(f64, f64)>,
    /// Per motor, where the machine reports the torque at the gearbox output: its place among the machine's
    /// own signals, its peak and the sum of |torque|^3 |speed|. Looked at every step: a gear rings faster
    /// than once per millisecond.
    gear: Vec<Option<(usize, f64, f64)>>,
    machine_signals: Vec<f64>,
    /// Energy fed back since each motor last drew power.
    braking: Vec<f64>,
    /// The setpoints of the last half second: the piece of the path the tool can be near.
    path: std::collections::VecDeque<Vec<f64>>,
    path_error: (f64, f64, u64),
    tracking_error: f64,
    dwell_error: f64,
}

/// Distance from `p` to the piece of line between `a` and `b`.
fn distance_to_piece(p: &[f64], a: &[f64], b: &[f64]) -> f64 {
    let dot = |u: &dyn Fn(usize) -> f64, v: &dyn Fn(usize) -> f64| (0..p.len()).map(|i| u(i) * v(i)).sum::<f64>();
    let (ab, ap) = (|i: usize| b[i] - a[i], |i: usize| p[i] - a[i]);
    let length = dot(&ab, &ab);
    let along = if length > 0.0 { (dot(&ap, &ab) / length).clamp(0.0, 1.0) } else { 0.0 };
    (0..p.len()).map(|i| (ap(i) - along * ab(i)).powi(2)).sum::<f64>().sqrt()
}

impl Tally {
    fn new(sim: &Sim) -> Self {
        let motors = sim.machine.motors().len();
        let infos = sim.signal_infos();
        let named = |start: &str, end: &str| -> Vec<usize> { infos.iter().enumerate().filter(|(_, s)| s.id.starts_with(start) && s.id.ends_with(end)).map(|(i, _)| i).collect() };
        let guides = named("axis.", ".guide_load");
        let own = sim.machine.signal_infos();
        let gear = |motor: &sim_core::MotorInfo| Some((own.iter().position(|s| s.id == format!("gearbox.{}.output_torque", motor.id))?, 0.0, 0.0));
        Self {
            gear: sim.machine.motors().iter().map(gear).collect(),
            machine_signals: Vec::new(),
            radial: named("pulley.", ".radial_load"),
            slack: infos.iter().position(|s| s.id == "belt.slack_time"),
            radial_peak: 0.0,
            guide_loads: vec![(0.0, 0.0); guides.len()],
            guides,
            slack_start: 0.0,
            cubic: vec![(0.0, 0.0); motors],
            loads: vec![MotorLoad::default(); motors],
            pos: vec![0.0; motors],
            speed: vec![0.0; motors],
            steps: 0,
            every: (1e-3 / sim.dt).round().max(1.0) as u64,
            tensions: named("belt.", ".tension"),
            sample: Vec::new(),
            belt_tension: None,
            braking: vec![0.0; motors],
            path: std::collections::VecDeque::new(),
            path_error: (0.0, 0.0, 0),
            tracking_error: 0.0,
            dwell_error: 0.0,
        }
    }

    /// Adds the step the simulation has just made.
    fn step(&mut self, sim: &Sim) {
        sim.machine.motor_state(&sim.state, &mut self.pos, &mut self.speed);
        let sampled = self.steps % self.every == 0;
        if sampled {
            sim.sample(&mut self.sample);
            for &i in &self.tensions {
                let (low, high) = self.belt_tension.unwrap_or((self.sample[i], self.sample[i]));
                self.belt_tension = Some((low.min(self.sample[i]), high.max(self.sample[i])));
            }
            for &i in &self.radial {
                self.radial_peak = self.radial_peak.max(self.sample[i]);
            }
            for (&i, (peak, cubed)) in self.guides.iter().zip(&mut self.guide_loads) {
                (*peak, *cubed) = (peak.max(self.sample[i]), *cubed + self.sample[i].powi(3));
            }
            if let (0, Some(i)) = (self.steps, self.slack) {
                self.slack_start = self.sample[i];
            }
            // Path error: how far the tool is from the path the setpoint has drawn, wherever along it.
            // Tracking error: how far it is from where the setpoint is right now.
            let axes = sim.machine.axes().len();
            let (mut tool, mut rate) = (vec![0.0; axes], vec![0.0; axes]);
            sim.machine.tool_state(&sim.state, &mut tool, &mut rate);
            self.path.push_back(sim.control.set_pos.clone());
            if self.path.len() > 500 {
                self.path.pop_front();
            }
            let now = self.path.back().unwrap();
            let off_path = self.path.iter().zip(self.path.iter().skip(1)).map(|(a, b)| distance_to_piece(&tool, a, b)).fold(distance_to_piece(&tool, now, now), f64::min);
            self.path_error = (self.path_error.0.max(off_path), self.path_error.1 + off_path * off_path, self.path_error.2 + 1);
            self.tracking_error = self.tracking_error.max(distance_to_piece(&tool, now, now));
            if sim.control.dwell_progress().is_some_and(|done| done >= 0.5) {
                let target = sim.control.planned_pos();
                self.dwell_error = self.dwell_error.max(distance_to_piece(&tool, target, target));
            }
        }
        if self.gear.iter().any(Option::is_some) {
            self.machine_signals.clear();
            sim.machine.signals(&sim.state, &sim.torque, &mut self.machine_signals);
        }
        for (k, (i, peak, cubed)) in self.gear.iter_mut().enumerate().filter_map(|(k, gear)| Some((k, gear.as_mut()?))) {
            let torque = self.machine_signals[*i].abs();
            (*peak, *cubed) = (peak.max(torque), *cubed + torque.powi(3) * self.speed[k].abs());
        }
        for (k, load) in self.loads.iter_mut().enumerate() {
            let torque = sim.control.drives[k].torque;
            load.peak_torque = load.peak_torque.max(torque.abs());
            let (current, power) = (sim.control.drives[k].current, sim.control.drives[k].power);
            load.rms_current += current * current;
            if power >= 0.0 {
                load.energy_in += power * sim.dt;
                self.braking[k] = 0.0;
            } else {
                load.energy_regenerated -= power * sim.dt;
                load.peak_regenerated_power = load.peak_regenerated_power.max(-power);
                self.braking[k] -= power * sim.dt;
                load.largest_regeneration = load.largest_regeneration.max(self.braking[k]);
            }
            load.rms_torque += torque * torque;
            self.cubic[k] = (self.cubic[k].0 + torque.abs().powi(3) * self.speed[k].abs(), self.cubic[k].1 + self.speed[k].abs());
            let drive = &sim.control.drives[k];
            load.resistor_energy += drive.resistor_power * sim.dt;
            load.peak_resistor_power = load.peak_resistor_power.max(drive.resistor_power);
            load.peak_dc_link_voltage = load.peak_dc_link_voltage.max(drive.dc_voltage);
            load.peak_speed = load.peak_speed.max(self.speed[k].abs());
            // What the velocity loop asks for, not what the drive delivers: a drive on its torque limit
            // delivers the curve and no more, so the delivered torque would never show a cycle beyond it.
            let (low, high) = sim.control.torque_curve(self.speed[k]);
            let demand = drive.torque_demand;
            let available = if demand >= 0.0 { high } else { -low };
            load.utilisation = load.utilisation.max(demand.abs() / available.max(1e-9));
            if sampled {
                load.points.push((self.speed[k], torque));
            }
        }
        self.steps += 1;
    }

    fn finish(mut self, sim: &Sim, duration: f64) -> CycleLoad {
        let steps = self.steps.max(1) as f64;
        for (k, load) in self.loads.iter_mut().enumerate() {
            load.rms_torque = (load.rms_torque / steps).sqrt();
            load.rms_current = (load.rms_current / steps).sqrt();
            load.cubic_mean_torque = if self.cubic[k].1 > 0.0 { (self.cubic[k].0 / self.cubic[k].1).cbrt() } else { 0.0 };
            load.gearbox_torque = self.gear[k].map(|(_, peak, cubed)| (peak, if self.cubic[k].1 > 0.0 { (cubed / self.cubic[k].1).cbrt() } else { 0.0 }));
            load.winding_rise = sim.control.drive_params().settled_rise(load.rms_current);
            load.thermal_utilisation = load.rms_torque / sim.control.rated_torque();
            // A tripped drive's DC link stays where it tripped, also when that was before the measured pass.
            load.peak_dc_link_voltage = load.peak_dc_link_voltage.max(sim.control.drives[k].dc_voltage);
        }
        sim.sample(&mut self.sample);
        let samples = self.path_error.2.max(1) as f64;
        // The guide that the life estimate is about: the one with the highest equivalent load.
        let guide_load = self.guide_loads.iter().map(|(peak, cubed)| (*peak, (cubed / samples).cbrt())).max_by(|a, b| a.1.total_cmp(&b.1));
        CycleLoad {
            duration,
            belt_tension: self.belt_tension,
            belt_slack_time: self.slack.map(|i| self.sample[i] - self.slack_start),
            radial_load_peak: (!self.radial.is_empty()).then_some(self.radial_peak),
            guide_load,
            path_error_peak: self.path_error.0,
            path_error_rms: (self.path_error.1 / self.path_error.2.max(1) as f64).sqrt(),
            tracking_error_peak: self.tracking_error,
            dwell_error_peak: self.dwell_error,
            tripped: sim.control.drives.iter().any(|d| d.tripped),
            motors: self.loads,
        }
    }
}

/// Runs the configured cycle and measures its second pass, when the machine has settled into it. If a
/// drive trips before, the cycle never gets there: what is measured then is the run up to the trip.
pub fn analyse_cycle(sim: &mut Sim) -> Result<CycleLoad, CommandError> {
    sim.command(Command::StartCycle)?;
    let mut tally = Tally::new(sim);
    let (mut passes, mut last, mut start) = (0, sim.control.cycle_passes(), 0.0);
    loop {
        sim.step();
        // The controller counts the passes: a cycle of one segment never changes its segment index.
        let now = sim.control.cycle_passes();
        if now != last {
            passes += 1;
            if passes == 2 {
                // The measured pass begins: what was added up on the way here goes.
                (start, tally) = (sim.time(), Tally::new(sim));
            } else if passes == 3 {
                break;
            }
        }
        last = now;
        if sim.time() > 600.0 {
            return Err(CommandError { key: "error-cycle-timeout" });
        }
        tally.step(sim);
        // Drives that tripped on DC link overvoltage never finish the cycle; the verdicts say why.
        if !sim.control.enabled() {
            break;
        }
    }
    let _ = sim.command(Command::Stop);
    let duration = sim.time() - start;
    Ok(tally.finish(sim, duration))
}

/// Runs a path given as poses at times (see `Command::Trajectory`) and measures it, with 0.3 s of rest
/// after its end so that the settling of the tool counts.
pub fn analyse_trajectory(sim: &mut Sim, points: Vec<(f64, Vec<f64>)>) -> Result<CycleLoad, CommandError> {
    let end = points.last().map_or(0.0, |p| p.0) + 0.3;
    sim.command(Command::Trajectory(points))?;
    let (mut tally, start) = (Tally::new(sim), sim.time());
    while sim.time() - start < end {
        sim.step();
        tally.step(sim);
    }
    Ok(tally.finish(sim, end))
}

/// One judgement of a cycle: a value the cycle reached against what a part allows.
#[derive(Debug, Clone, PartialEq)]
pub struct Verdict {
    /// What is judged, e.g. "motor-thermal" or "belt-tension".
    pub check: &'static str,
    /// The motor it concerns, where it concerns one.
    pub motor: Option<usize>,
    pub value: f64,
    pub limit: f64,
    pub ok: bool,
}

/// Ratings that `verdicts` judges and no default config has: a config file may add them (see `load_config`).
pub const RATINGS_BEYOND_DEFAULT: [&str; 1] = ["ratings.pulley_max_torque_nm"];

fn at_most(check: &'static str, motor: Option<usize>, value: f64, limit: f64) -> Verdict {
    Verdict { check, motor, value, limit, ok: value <= limit }
}

/// Judges a cycle against the motor and drive data and the `[ratings]` of the config. A rating that the
/// config leaves out is not judged. Gearbox and pulley torques are those the machine reports at the gearbox
/// output (`MotorLoad::gearbox_torque`), so what the rotor's own inertia takes is not in them, and a load
/// that drives the gearbox back counts with what it puts on the output. On a machine that does not report
/// them, these three checks are not made.
///
/// "motor-peak" judges the torque the velocity loop asks for before the torque limit (`Drive::torque_demand`),
/// so a cycle that sits on the limit fails it.
///
/// "gearbox-continuous" judges the cubic mean of that torque against the gearbox's rated torque, as gearbox
/// makers do; it used to judge the RMS torque, which is for motors and their heat.
/// "belt-slack" also fails when any belt section spent time without tension.
/// With `effects.dc_link` the braking checks judge the simulated DC link: what the capacitors held at the
/// highest voltage, the mean and peak power the resistor burned, and the highest voltage against the trip
/// level. Without it they are estimated from the energy the motor fed back, as before.
pub fn verdicts(cfg: &Config, load: &CycleLoad) -> Result<Vec<Verdict>, ConfigError> {
    let cfg = &crate::catalog::parts_applied(cfg)?;
    // A rating that is no number (NaN) would otherwise count as left out, and not be judged.
    for (name, _) in cfg.get("ratings").ok().and_then(|ratings| ratings.as_table()).into_iter().flatten() {
        cfg.f64(&format!("ratings.{name}"))?;
    }
    let rating = |name: &str| cfg.f64(&format!("ratings.{name}")).ok();
    let simulated = cfg.bool("effects.dc_link")?;
    let (bus, rated_rise) = (cfg.positive("drive.bus_voltage_v")?, cfg.positive("motor.rated_temperature_rise_k")?);
    let mut out = Vec::new();
    for (k, m) in load.motors.iter().enumerate() {
        // Peak torque against the torque-speed curve, and the settled winding temperature against the rated rise.
        out.push(at_most("motor-peak", Some(k), m.utilisation, 1.0));
        out.push(at_most("motor-thermal", Some(k), m.winding_rise, rated_rise));
        if let (Some((peak, _)), Some(limit)) = (m.gearbox_torque, rating("pulley_max_torque_nm")) {
            out.push(at_most("pulley-torque", Some(k), peak, limit));
        }
        if let (Some((_, cubic_mean)), Some(limit)) = (m.gearbox_torque, rating("gearbox_rated_torque_nm")) {
            out.push(at_most("gearbox-continuous", Some(k), cubic_mean, limit));
        }
        if let Some(limit) = rating("gearbox_max_input_speed_rpm") {
            out.push(at_most("gearbox-input-speed", Some(k), m.peak_speed * 30.0 / std::f64::consts::PI, limit));
        }
        if let (Some((peak, _)), Some(limit)) = (m.gearbox_torque, rating("gearbox_max_torque_nm")) {
            out.push(at_most("gearbox-peak", Some(k), peak, limit));
        }
        // Braking: what the DC link capacitors take between the bus voltage and the threshold. If one
        // braking gives back more, the braking resistor has to take the mean and the peak power.
        if let (Some(capacitance), Some(threshold)) = (rating("dc_link_capacitance_uf"), rating("braking_threshold_v")) {
            // A threshold at or below the bus is a rating for another bus, not a DC link without room.
            if threshold <= bus {
                return Err(ConfigError::Invalid("ratings.braking_threshold_v".into()));
            }
            let room = 0.5 * capacitance * 1e-6 * (threshold * threshold - bus * bus);
            let (continuous, peak) = (rating("braking_resistor_continuous_w"), rating("braking_resistor_peak_w"));
            // (energy in the capacitors above the bus level, whether the resistor is needed, its mean and peak power)
            let (stored, resistor, mean_power, peak_power) = if simulated {
                let stored = 0.5 * capacitance * 1e-6 * (m.peak_dc_link_voltage.powi(2) - bus * bus);
                (stored, m.resistor_energy > 0.0, m.resistor_energy / load.duration, m.peak_resistor_power)
            } else {
                (m.largest_regeneration, m.largest_regeneration > room, m.energy_regenerated / load.duration, m.peak_regenerated_power)
            };
            // With a resistor to fall back on, a full DC link is no failure in itself.
            out.push(Verdict { check: "dc-link-energy", motor: Some(k), value: stored, limit: room, ok: !resistor || continuous.is_some() });
            if resistor {
                if let Some(limit) = continuous {
                    out.push(at_most("braking-resistor-continuous", Some(k), mean_power, limit));
                }
                if let Some(limit) = peak {
                    out.push(at_most("braking-resistor-peak", Some(k), peak_power, limit));
                }
            }
        }
        // The simulated resistor never takes more than its peak power; what it cannot take shows here.
        if let (true, Some(limit)) = (simulated, rating("dc_link_overvoltage_v")) {
            out.push(at_most("dc-link-overvoltage", Some(k), m.peak_dc_link_voltage, limit));
        }
    }
    if let Some(limit) = rating("settling_tolerance_m") {
        out.push(at_most("settling", None, load.dwell_error_peak, limit));
    }
    if let Some((slackest, tightest)) = load.belt_tension {
        if let Some(limit) = rating("belt_max_tension_n") {
            out.push(Verdict { check: "belt-tension", motor: None, value: tightest, limit, ok: tightest <= limit });
        }
        if let Some(limit) = rating("belt_min_tension_n") {
            // The one check where the value has to stay above its limit. The tension is looked at once per
            // millisecond; the slack time also counts what falls between.
            let never_slack = load.belt_slack_time.is_none_or(|time| time <= 0.0);
            out.push(Verdict { check: "belt-slack", motor: None, value: slackest, limit, ok: slackest > limit && never_slack });
        }
    }
    if let (Some(value), Some(limit)) = (load.radial_load_peak, rating("shaft_max_radial_load_n")) {
        out.push(at_most("gearbox-radial-load", None, value, limit));
    }
    if let (Some((peak, _)), Some(limit)) = (load.guide_load, rating("guide_max_load_n")) {
        out.push(at_most("guide-load", None, peak, limit));
    }
    Ok(out)
}

/// Nominal life of the most loaded guide in km, where the machine reports guide loads and the config has
/// `ratings.guide_dynamic_load_rating_n`: L10 = (C / P)^3 * 50 km, the catalogue formula for ball guides,
/// with the dynamic load rating C and for P the cubic mean of the guide load over the cycle's time.
///
/// This is the catalogue formula and nothing more, and it is not validated: no load factors for shock,
/// temperature or hardness, no moments on single blocks, the mean taken over time and not over distance,
/// and a guide load that is itself an estimate from the rigid model.
pub fn guide_life_km(cfg: &Config, load: &CycleLoad) -> Option<f64> {
    let rating = cfg.f64("ratings.guide_dynamic_load_rating_n").ok()?;
    Some((rating / load.guide_load?.1).powi(3) * 50.0)
}

/// The low and high bound of every uncertain parameter of a config, as (path, low, high).
///
/// The table `[bounds]` mirrors the config: where the config has `[friction_x] coulomb_n = 15.0`, it has
/// `friction_x.coulomb_n = [10.0, 20.0]`, in the unit of the value. A pair and not a relative width,
/// because the parameter panel and the translation check skip arrays (`Config::leaves`), so bounds need
/// no labels, and because a value of zero can have bounds this way. A bound on a value the config does
/// not have, or one that is not a low and a high number, is refused.
pub fn bounds(cfg: &Config) -> Result<Vec<(String, f64, f64)>, ConfigError> {
    fn walk(path: &str, value: &toml::Value, cfg: &Config, out: &mut Vec<(String, f64, f64)>) -> Result<(), ConfigError> {
        if let Some(table) = value.as_table() {
            return table.iter().try_for_each(|(key, v)| walk(&if path.is_empty() { key.clone() } else { format!("{path}.{key}") }, v, cfg, out));
        }
        cfg.f64(path)?;
        match cfg.f64s(&format!("bounds.{path}"))?[..] {
            [low, high] if low <= high => out.push((path.into(), low, high)),
            _ => return Err(ConfigError::Invalid(format!("bounds.{path}"))),
        }
        Ok(())
    }
    let mut out = Vec::new();
    if let Ok(table) = cfg.get("bounds") {
        walk("", table, cfg, &mut out)?;
    }
    Ok(out)
}

/// A verdict with what the uncertain parameters do to it: the lowest and the highest value it took, and
/// whether it passed every time. `verdict` is the one of the config as it is; for a check that only a
/// bound brings up (the braking resistor, once the DC link is full) it is the first one seen.
#[derive(Debug, Clone, PartialEq)]
pub struct Band {
    pub verdict: Verdict,
    pub low: f64,
    pub high: f64,
    /// The worst case passes. Only then does the verdict pass with margin.
    pub ok: bool,
    /// The share of the runs in which the check passed, the config as it is among them. A run in which the
    /// check did not come up has passed it.
    pub share: f64,
}

impl Band {
    /// `share` for showing with this many decimals: cut off, not rounded, so that a share below 1 never
    /// reads as 1 (one failed run in 2001 is 0.999, not 1.000).
    pub fn share_shown(&self, decimals: i32) -> f64 {
        // The 1e-9 keeps a share of whole digits whose product comes out a rounding below them on its digit.
        let scale = 10f64.powi(decimals);
        (self.share * scale + 1e-9).floor() / scale
    }
}

/// The cycle of a config, judged.
fn judged(entry: &MachineEntry, cfg: &Config) -> Result<Vec<Verdict>, ConfigError> {
    let load = analyse_cycle(&mut Sim::new(entry, cfg)?).map_err(|e| ConfigError::Invalid(e.key.into()))?;
    verdicts(cfg, &load)
}

/// The bands of the cycle of `cfg` (already with its parts in) and of the configs `changed(0..count)` makes of
/// it. The runs are independent, so they share the processors; the bands come out the same in any case.
fn envelope(entry: &MachineEntry, cfg: &Config, count: usize, changed: impl Fn(usize) -> Result<Config, ConfigError> + Sync) -> Result<Vec<Band>, ConfigError> {
    let next = std::sync::atomic::AtomicUsize::new(0);
    let work = || -> Vec<(usize, Result<Vec<Verdict>, ConfigError>)> {
        let mut done = Vec::new();
        loop {
            let run = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if run > count {
                return done;
            }
            // Run 0 is the config as it is.
            done.push((run, if run == 0 { judged(entry, cfg) } else { changed(run - 1).and_then(|cfg| judged(entry, &cfg)) }));
        }
    };
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()).min(count + 1);
    let mut runs: Vec<_> = std::thread::scope(|scope| {
        let workers: Vec<_> = (0..threads).map(|_| scope.spawn(work)).collect();
        workers.into_iter().flat_map(|worker| worker.join().expect("a run of the cycle panicked")).collect()
    });
    runs.sort_by_key(|(run, _)| *run);
    let (total, mut bands) = (runs.len() as f64, Vec::<Band>::new());
    for (_, run) in runs {
        for v in run? {
            let failed = if v.ok { 0.0 } else { 1.0 / total };
            match bands.iter_mut().find(|b| b.verdict.check == v.check && b.verdict.motor == v.motor) {
                Some(b) => (b.low, b.high, b.ok, b.share) = (b.low.min(v.value), b.high.max(v.value), b.ok && v.ok, b.share - failed),
                None => bands.push(Band { low: v.value, high: v.value, ok: v.ok, share: 1.0 - failed, verdict: v }),
            }
        }
    }
    Ok(bands)
}

/// The verdicts of the cycle with error bands: the cycle is run with the config as it is and with each
/// parameter of `bounds` (path, low, high; see `bounds`) at its low and at its high value, one at a
/// time, two runs per parameter. A band is the envelope of all these runs.
///
/// One at a time ignores interactions: two parameters that are both off can take a verdict beyond its
/// band. The band says which verdicts are safe against any single uncertain value, no more;
/// `verdict_bands_sampled` moves them all at once.
pub fn verdict_bands(entry: &MachineEntry, cfg: &Config, bounds: &[(String, f64, f64)]) -> Result<Vec<Band>, ConfigError> {
    let cfg = &crate::catalog::resolve(entry, cfg)?;
    envelope(entry, cfg, 2 * bounds.len(), |run| {
        let ((path, low, high), mut changed) = (&bounds[run / 2], cfg.clone());
        changed.set_text(&format!("{path} = {:?}", if run % 2 == 0 { low } else { high }))?;
        Ok(changed)
    })
}

/// The verdicts of the cycle with error bands that count interactions: the cycle is run with the config as
/// it is and `samples` times with every parameter of `bounds` somewhere between its bounds, all at once. A
/// band is the envelope of these runs, and `Band::share` says in how many of them the check passed.
///
/// The values are a Latin hypercube: the range of each parameter is cut into `samples` equal parts, each
/// part is used in exactly one run (at a random place inside it), and each parameter has its parts in a
/// random order of its own. So every parameter is spread evenly over its range however few the runs, and
/// reaches to within a `samples`-th of the range of both bounds. Pseudo-random from `seed`: the same call
/// gives the same bands.
///
/// What it does not do: put a parameter exactly on a bound (`verdict_bands` does), or search for the worst
/// corner. With n parameters there are 2^n corners, and a few dozen runs meet none of them: a check that
/// fails only with several values near their bounds at once can pass every sample.
pub fn verdict_bands_sampled(entry: &MachineEntry, cfg: &Config, bounds: &[(String, f64, f64)], samples: usize, seed: u64) -> Result<Vec<Band>, ConfigError> {
    if samples == 0 {
        return Err(ConfigError::Invalid("samples".into()));
    }
    let cfg = &crate::catalog::resolve(entry, cfg)?;
    let mut random = sim_core::testkit::Noise(seed);
    let columns: Vec<Vec<f64>> = bounds
        .iter()
        .map(|(_, low, high)| {
            let mut values: Vec<f64> = (0..samples).map(|part| low + (high - low) * (part as f64 + random.uniform()) / samples as f64).collect();
            for i in (1..samples).rev() {
                values.swap(i, (random.uniform() * (i + 1) as f64) as usize);
            }
            values
        })
        .collect();
    envelope(entry, cfg, samples, |run| {
        let mut changed = cfg.clone();
        for ((path, ..), values) in bounds.iter().zip(&columns) {
            changed.set_text(&format!("{path} = {:?}", values[run]))?;
        }
        Ok(changed)
    })
}

/// A cycle made faster: the cycle before and after, the settings that changed (path, before, after) and
/// the number of cycles that were simulated on the way.
#[derive(Debug, Clone)]
pub struct Optimised {
    pub before: CycleLoad,
    pub after: CycleLoad,
    pub changes: Vec<(String, String, String)>,
    pub runs: usize,
}

/// Shortens the cycle of a config without breaking a verdict. Tries corner blending, then raises speed,
/// acceleration and jerk of all axes together in steps for as long as the cycle gets shorter and every
/// verdict of `verdicts` still passes, among them the settling tolerance, so a faster move that leaves
/// the tool swinging at the pick does not count. Input shaping makes no cycle shorter by itself (every
/// stop waits one period of the shaper), so it is tried where it can help: a faster setting that fails
/// on "settling" alone is run again with the shaper, and with the shaper following the pose where the
/// machine reports a resonance, and is taken if it then passes and is shorter. Where the shaper is on
/// already, switching it off is tried too. Simulates at most `max_runs` cycles.
///
/// A search by trial, not a proof of the optimum: it stops at the first setting from which no single step
/// is an improvement.
pub fn optimise_cycle(entry: &MachineEntry, cfg: &Config, max_runs: usize) -> Result<Optimised, ConfigError> {
    let probe = Sim::new(entry, cfg)?;
    let axes: Vec<&'static str> = probe.machine.axes().iter().map(|a| a.id).collect();
    let tells_resonance = probe.machine.resonance_hz(probe.control.planned_pos()).is_some();
    let limits = ["max_velocity_m_s", "max_acceleration_m_s2", "max_jerk_m_s3"];
    // A candidate: the factor on each of the three limits, and the three switches.
    #[derive(Clone, Copy, PartialEq)]
    struct Setting {
        factors: [f64; 3],
        blending: bool,
        shaping: bool,
        follows: bool,
    }
    let build = |setting: Setting| -> Result<Config, ConfigError> {
        let mut changed = cfg.clone();
        for axis in &axes {
            for (limit, factor) in limits.iter().zip(setting.factors) {
                let path = format!("axis.{axis}.{limit}");
                changed.set_text(&format!("{path} = {:?}", cfg.positive(&path)? * factor))?;
            }
        }
        changed.set_text(&format!("effects.corner_blending = {}", setting.blending))?;
        changed.set_text(&format!("effects.input_shaping = {}", setting.shaping))?;
        changed.set_text(&format!("effects.shaper_follows_pose = {}", setting.follows))?;
        Ok(changed)
    };
    let runs = std::cell::Cell::new(0usize);
    // The cycle of a setting and the checks it fails, if it runs.
    let judged = |setting: Setting| -> Option<(CycleLoad, Vec<&'static str>)> {
        runs.set(runs.get() + 1);
        let changed = build(setting).ok()?;
        let mut sim = Sim::new(entry, &changed).ok()?;
        if !sim.limit_violations.is_empty() {
            return None;
        }
        let load = analyse_cycle(&mut sim).ok()?;
        let failed = verdicts(&changed, &load).ok()?.iter().filter(|v| !v.ok).map(|v| v.check).collect();
        Some((load, failed))
    };
    let start = Setting { factors: [1.0; 3], blending: cfg.bool("effects.corner_blending")?, shaping: cfg.bool("effects.input_shaping")?, follows: cfg.bool("effects.shaper_follows_pose")? };
    let before = match judged(start) {
        Some((load, failed)) if failed.is_empty() => load,
        _ => return Err(ConfigError::Invalid("the cycle as configured fails a verdict".into())),
    };
    let (mut best, mut best_load) = (start, before.clone());
    let mut improved = true;
    while improved {
        improved = false;
        let mut candidates = vec![Setting { blending: !best.blending, ..best }];
        if best.shaping {
            candidates.push(Setting { shaping: false, ..best });
        }
        for i in 0..3 {
            for step in [1.5, 1.2] {
                let mut faster = best;
                faster.factors[i] *= step;
                candidates.push(faster);
            }
        }
        'candidates: for candidate in candidates {
            // A candidate that only leaves the tool swinging gets its tries with the shaper right after it.
            let mut tries = std::collections::VecDeque::from([candidate]);
            while let Some(setting) = tries.pop_front() {
                if runs.get() >= max_runs {
                    break 'candidates;
                }
                let Some((load, failed)) = judged(setting) else { continue };
                if failed.is_empty() && load.duration < best_load.duration - 1e-4 {
                    (best, best_load, improved) = (setting, load, true);
                    break 'candidates;
                }
                if !setting.shaping && !failed.is_empty() && failed.iter().all(|check| *check == "settling") {
                    tries.push_back(Setting { shaping: true, follows: false, ..setting });
                    if tells_resonance {
                        tries.push_back(Setting { shaping: true, follows: true, ..setting });
                    }
                }
            }
        }
    }
    let (from, to) = (build(start)?, build(best)?);
    let changes = to
        .leaves()
        .into_iter()
        .zip(from.leaves())
        .filter(|((_, after), (_, before))| after != before)
        .map(|((path, after), (_, before))| (path, before.to_string(), after.to_string()))
        .collect();
    Ok(Optimised { before, after: best_load, changes, runs: runs.get() })
}

/// What one parameter does to the cycle when it is lower and when it is higher than in the config.
#[derive(Debug, Clone)]
pub struct Sensitivity {
    pub path: String,
    pub value: f64,
    /// The low and the high value the cycle was run with.
    pub bounds: (f64, f64),
    pub low: CycleLoad,
    pub high: CycleLoad,
}

impl Sensitivity {
    /// Relative change of the peak torque and the RMS torque (each of the motor it changes most) and of the
    /// cycle time, at the low and at the high value: [peak low, peak high, RMS low, RMS high, time low, time high].
    pub fn changes(&self, nominal: &CycleLoad) -> [f64; 6] {
        let change = |a: f64, b: f64| if b.abs() > 1e-12 { a / b - 1.0 } else { 0.0 };
        let per_motor = |load: &CycleLoad, pick: fn(&MotorLoad) -> f64| -> f64 {
            load.motors.iter().zip(&nominal.motors).map(|(m, n)| change(pick(m), pick(n))).fold(0.0, |a: f64, b| if b.abs() > a.abs() { b } else { a })
        };
        [
            per_motor(&self.low, |m| m.peak_torque),
            per_motor(&self.high, |m| m.peak_torque),
            per_motor(&self.low, |m| m.rms_torque),
            per_motor(&self.high, |m| m.rms_torque),
            change(self.low.duration, nominal.duration),
            change(self.high.duration, nominal.duration),
        ]
    }

    /// The largest of these changes: how much the results depend on this parameter. Front ends rank by it.
    pub fn strength(&self, nominal: &CycleLoad) -> f64 {
        self.changes(nominal).iter().fold(0.0, |a: f64, b| a.max(b.abs()))
    }
}

/// The cycle with the config as it is, and with each of `paths` at a low and at a high value, one at a
/// time: the bounds the config gives the parameter (see `bounds`), or the value scaled by 1 - delta and
/// by 1 + delta where it gives none. Shows which of the uncertain parameters the results depend on, and
/// so which are worth measuring. A parameter that is not a number, or whose changed value the machine
/// refuses, is left out.
pub fn sensitivity(entry: &MachineEntry, cfg: &Config, paths: &[String], delta: f64) -> Result<(CycleLoad, Vec<Sensitivity>), ConfigError> {
    let cfg = &crate::catalog::resolve(entry, cfg)?;
    let configured = bounds(cfg)?;
    let run = |cfg: &Config| -> Option<CycleLoad> { analyse_cycle(&mut Sim::new(entry, cfg).ok()?).ok() };
    let nominal = run(cfg).ok_or_else(|| ConfigError::Invalid("cycle".into()))?;
    let mut out = Vec::new();
    for path in paths {
        let Ok(value) = cfg.f64(path) else { continue };
        let bounds = configured.iter().find(|b| b.0 == *path).map_or((value * (1.0 - delta), value * (1.0 + delta)), |b| (b.1, b.2));
        let at = |value: f64| -> Option<CycleLoad> {
            let mut changed = cfg.clone();
            changed.set_text(&format!("{path} = {value:?}")).ok()?;
            run(&changed)
        };
        if let (Some(low), Some(high)) = (at(bounds.0), at(bounds.1)) {
            out.push(Sensitivity { path: path.clone(), value, bounds, low, high });
        }
    }
    Ok((nominal, out))
}

/// One point of the frequency response from a motor's torque to its own speed.
#[derive(Debug, Clone, Copy)]
pub struct SweepPoint {
    pub frequency: f64,
    /// Speed amplitude per torque amplitude, rad/s per N m.
    pub gain: f64,
    pub phase_deg: f64,
}

/// Mechanical frequency response without control: a sine torque on one motor on top of the holding
/// torques, measured at that motor's speed once the response has settled. Belt resonances show as
/// peaks, the load swinging against the held motor as a dip.
pub fn frequency_sweep(entry: &MachineEntry, cfg: &Config, motor: usize, frequencies: &[f64], amplitude: f64) -> Result<Vec<SweepPoint>, ConfigError> {
    let cfg = &crate::catalog::resolve(entry, cfg)?;
    let machine = (entry.build)(cfg)?;
    let (dt, substeps) = (cfg.positive("sim.dt_s")?, cfg.positive("sim.substeps")? as u32);
    let motors = machine.motors().len();
    if motor >= motors {
        return Err(ConfigError::Invalid("motor".into()));
    }
    check_frequencies(frequencies)?;
    let (mut start, mut hold) = (Vec::new(), vec![0.0; motors]);
    machine.equilibrium(&crate::sim::start_pose(cfg, machine.as_ref())?, &mut start, &mut hold);
    let mut rk = Rk4::new(start.len());
    let (mut pos, mut speed) = (vec![0.0; motors], vec![0.0; motors]);
    let mut out = Vec::new();
    for &frequency in frequencies {
        let omega = std::f64::consts::TAU * frequency;
        // Whole periods: at least 0.3 s to settle, then at least 0.2 s to measure.
        let period = 1.0 / frequency;
        let settle = (0.3 / period).ceil().max(5.0) * period;
        let measure = (0.2 / period).ceil().max(5.0) * period;
        let (settle_steps, measure_steps) = ((settle / dt).round() as u64, (measure / dt).round() as u64);
        let (mut state, mut torque) = (start.clone(), hold.clone());
        let (mut in_phase, mut quadrature) = (0.0, 0.0);
        for step in 0..settle_steps + measure_steps {
            let t = (step as f64 + 0.5) * dt;
            torque[motor] = hold[motor] + amplitude * (omega * t).sin();
            advance(machine.as_ref(), &mut rk, &mut state, &torque, dt, substeps);
            if step >= settle_steps {
                machine.motor_state(&state, &mut pos, &mut speed);
                let t = (step + 1) as f64 * dt;
                in_phase += speed[motor] * (omega * t).sin();
                quadrature += speed[motor] * (omega * t).cos();
            }
        }
        let (a, b) = (2.0 * in_phase / measure_steps as f64, 2.0 * quadrature / measure_steps as f64);
        out.push(SweepPoint { frequency, gain: a.hypot(b) / amplitude, phase_deg: b.atan2(a).to_degrees() });
    }
    Ok(out)
}

/// A response is measured over whole periods: at 0 Hz that never ends, and below it means nothing.
fn check_frequencies(frequencies: &[f64]) -> Result<(), ConfigError> {
    if frequencies.iter().all(|f| f.is_finite() && *f > 0.0) { Ok(()) } else { Err(ConfigError::Invalid("frequency".into())) }
}

/// Frequencies spaced evenly on a logarithmic axis, both ends included.
pub fn log_spaced(from: f64, to: f64, points: usize) -> Vec<f64> {
    (0..points).map(|i| from * (to / from).powf(i as f64 / (points.max(2) - 1) as f64)).collect()
}

// ---------------------------------------------------------------------------------------------
// Tuning: how the closed position loop answers a test signal on its setpoint.

/// The answer of one axis to a small step of its position setpoint.
#[derive(Debug, Clone)]
pub struct StepResponse {
    pub size: f64,
    /// (time since the step, tool position relative to where it was), four times per millisecond.
    pub trace: Vec<(f64, f64)>,
    /// Time from 10 % to 90 % of the step.
    pub rise_time: f64,
    /// Highest excursion beyond the step, as a fraction of it.
    pub overshoot: f64,
    /// After this time the position stays within 2 % of the step.
    pub settling_time: f64,
}

/// Tool position of one axis, after letting the machine come to rest at its start pose.
fn settled(entry: &MachineEntry, cfg: &Config, axis: usize) -> Result<(Sim, f64), ConfigError> {
    let mut sim = Sim::new(entry, cfg)?;
    if axis >= sim.machine.axes().len() {
        return Err(ConfigError::Invalid("axis".into()));
    }
    sim.run(0.05, |_| {});
    let position = tool_position(&sim, axis);
    Ok((sim, position))
}

fn tool_position(sim: &Sim, axis: usize) -> f64 {
    let n = sim.machine.axes().len();
    let (mut pos, mut vel) = (vec![0.0; n], vec![0.0; n]);
    sim.machine.tool_state(&sim.state, &mut pos, &mut vel);
    pos[axis]
}

/// Steps the position setpoint of one axis by `size` (without feedforward, so only the feedback
/// loops answer) and records the tool position. With friction on, a small step shows stiction too.
pub fn step_response(entry: &MachineEntry, cfg: &Config, axis: usize, size: f64, seconds: f64) -> Result<StepResponse, ConfigError> {
    let (mut sim, start) = settled(entry, cfg, axis)?;
    sim.control.setpoint_offset[axis] = size;
    let (t0, every) = (sim.time(), (0.25e-3 / sim.dt).round().max(1.0) as u64);
    let mut trace = vec![(0.0, 0.0)];
    sim.run(seconds, |s| {
        if s.step % every == 0 {
            trace.push((s.time() - t0, tool_position(s, axis) - start));
        }
    });
    let relative = |fraction: f64| trace.iter().find(|(_, y)| y / size >= fraction).map_or(f64::NAN, |(t, _)| *t);
    let overshoot = trace.iter().map(|(_, y)| y / size - 1.0).fold(0.0, f64::max);
    let settling_time = trace.iter().rev().find(|(_, y)| (y / size - 1.0).abs() > 0.02).map_or(0.0, |(t, _)| *t);
    Ok(StepResponse { size, rise_time: relative(0.9) - relative(0.1), overshoot, settling_time, trace })
}

/// Closed-loop frequency response of the position loop of one axis: a sine on the position
/// setpoint, measured at the tool. Gain is tool amplitude over setpoint amplitude.
pub fn closed_loop_response(entry: &MachineEntry, cfg: &Config, axis: usize, frequencies: &[f64], amplitude: f64) -> Result<Vec<SweepPoint>, ConfigError> {
    check_frequencies(frequencies)?;
    let mut out = Vec::new();
    for &frequency in frequencies {
        let (mut sim, start) = settled(entry, cfg, axis)?;
        let (omega, t0, dt) = (std::f64::consts::TAU * frequency, sim.time(), sim.dt);
        // Whole periods: at least 0.3 s to settle, then at least 0.3 s to measure.
        let period = 1.0 / frequency;
        let whole = |seconds: f64| ((seconds / period).ceil().max(4.0) * period / dt).round() as u64;
        let (settle_steps, measure_steps) = (whole(0.3), whole(0.3));
        let (mut in_phase, mut quadrature) = (0.0, 0.0);
        for step in 0..settle_steps + measure_steps {
            sim.control.setpoint_offset[axis] = amplitude * (omega * (sim.time() - t0)).sin();
            sim.step();
            if step >= settle_steps {
                let (t, y) = (sim.time() - t0, tool_position(&sim, axis) - start);
                in_phase += y * (omega * t).sin();
                quadrature += y * (omega * t).cos();
            }
        }
        let (a, b) = (2.0 * in_phase / measure_steps as f64, 2.0 * quadrature / measure_steps as f64);
        out.push(SweepPoint { frequency, gain: a.hypot(b) / amplitude, phase_deg: b.atan2(a).to_degrees() });
    }
    Ok(out)
}

/// The -3 dB bandwidth of a closed-loop response: where the gain first falls below 1 / sqrt(2),
/// interpolated between the measured points on the logarithmic frequency axis.
pub fn bandwidth(points: &[SweepPoint]) -> Option<f64> {
    let limit = std::f64::consts::FRAC_1_SQRT_2;
    let below = points.iter().position(|p| p.gain < limit)?;
    let (a, b) = (points.get(below.checked_sub(1)?)?, &points[below]);
    let share = (a.gain - limit) / (a.gain - b.gain);
    Some(a.frequency * (b.frequency / a.frequency).powf(share))
}
