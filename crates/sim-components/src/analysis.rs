//! Analysis for motor sizing: peak and RMS torque per cycle, operating points against the
//! torque-speed curve, and the mechanical frequency response.

use crate::control::{Command, CommandError};
use crate::sim::Sim;
use sim_core::{Config, ConfigError, LOSS_PREFIX, MachineEntry, RatingCheck, Rk4, SignalInfo, TORQUE_PARTS, ThermalPart, advance};

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
    /// Highest |current| over the pass, A: against the drive's maximum output current.
    pub peak_current: f64,
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
    /// With the regenerative supply (effects.regenerative_supply): the energy fed back into the mains (what arrives
    /// there), and the line module's load: the energy through it either way (drawn from the line, and fed back as
    /// the DC link gives it) and its highest power either way.
    pub mains_energy: f64,
    pub line_module_energy: f64,
    pub peak_line_module_power: f64,
    /// The torque at the gearbox output, where the machine reports it (signal `gearbox.<motor>.output_torque`): its
    /// peak, and its cubic mean weighted with the speed. Not the motor torque times the ratio: the rotor's
    /// own acceleration does not pass through the gearbox, and its losses come off or on top.
    pub gearbox_torque: Option<(f64, f64)>,
    /// Mean of the motor's |speed| over the pass, rad/s: for the lives in hours.
    pub mean_speed: f64,
    /// Where the machine reports the gearbox output torque: its mean with the exponent 10/3, weighted with the
    /// speed, the equivalent torque of Siemens' life formula for the planet bearings (`gearbox_life_h`).
    pub gearbox_life_torque: Option<f64>,
    /// Where the machine reports the gearbox output torque: its peak in each band of `SPEED_BAND` rad/s of the motor's
    /// |speed| (band i from i to i + 1 times it), every step. For the belt's tooth load, whose rating falls with speed.
    pub gearbox_torque_by_speed: Vec<f64>,
    /// Where the machine reports the radial load on this motor's pulley shaft (`pulley.<motor>.radial_load`): its cubic
    /// mean weighted with the speed, the equivalent load P of the bearings' rating life (ISO 281), once per millisecond.
    pub radial_load_equivalent: Option<f64>,
    /// Peak and RMS of each part of the motor torque, in the order of `TORQUE_PARTS`, where the machine reports
    /// them (signals `motor.<id>.torque_<part>`). Looked at every step. The parts add up to the torque at every
    /// moment; their peaks and RMS values do not add up, as they come at different times.
    pub torque_parts: Option<[(f64, f64); 4]>,
    /// (speed, torque) once per millisecond, for plotting over the torque-speed curve.
    pub points: Vec<(f64, f64)>,
}

/// One line of an energy budget.
#[derive(Debug, Clone, PartialEq)]
pub struct BudgetRow {
    /// `copper.<motor>`, `resistor`, `mains`, `line_module`, a machine's loss signal (`power.<place>`), `windings`,
    /// `capacitors`, a machine's energy signal (`energy.<part>`) or `payload`.
    pub id: String,
    /// The label, as two translation keys like a signal's.
    pub owner_key: String,
    pub quantity_key: String,
    /// Energy over the measured pass, J: lost, or for a store what it holds more at the end than at the start.
    pub energy: f64,
}

/// Where the energy drawn from the supply over the measured pass went (see `analyse_cycle`).
#[derive(Debug, Clone, PartialEq)]
pub struct EnergyBudget {
    /// Energy from the supply. Without the DC link simulated, what the drives drew less what they fed back; with it,
    /// what the rectifier (or the line module) drew from the line into the DC link.
    pub supply: f64,
    pub rows: Vec<BudgetRow>,
    /// The work the motor torques did on the mechanics, integrated with the windings: where the electrical side hands
    /// over to the machine. Not a row: the machine's rows say where it went.
    pub shaft_work: f64,
}

impl EnergyBudget {
    /// What the rows leave of the supply. Below `BUDGET_TOLERANCE` of the supply on every shipped machine and
    /// config (`the_energy_budget_closes_on_every_machine_and_config`).
    pub fn unaccounted(&self) -> f64 {
        self.supply - self.rows.iter().map(|r| r.energy).sum::<f64>()
    }

    /// The electrical side on its own: the supply less copper loss, braking resistor, bleeder discharge, mains and line module, the
    /// windings' and the capacitors' stored energy and the shaft work. Below `ELECTRICAL_TOLERANCE` of the supply on
    /// every machine and config with the back-EMF on, loss-free ones included; the rest of `unaccounted` is the
    /// machine's own balance.
    pub fn electrical_unaccounted(&self) -> f64 {
        let electrical = |id: &str| id.starts_with("copper.") || matches!(id, "resistor" | "discharge" | "mains" | "line_module" | "windings" | "capacitors");
        self.supply - self.shaft_work - self.rows.iter().filter(|r| electrical(&r.id)).map(|r| r.energy).sum::<f64>()
    }
}

/// How far an energy budget may be off, as a share of the supply. The electrical side closes to
/// `ELECTRICAL_TOLERANCE`, so what is left is the machine's own balance: its losses are taken at the end of each
/// step and summed times the step, the trapezoid rule but for the ends (see `testkit::loss_budget`).
pub const BUDGET_TOLERANCE: f64 = 1e-3;

/// How far the electrical side of an energy budget may be off (`EnergyBudget::electrical_unaccounted`), as a share
/// of the supply: the error of the fourth-order integration of the winding's energy over a base step.
pub const ELECTRICAL_TOLERANCE: f64 = 1e-6;

#[derive(Debug, Clone)]
pub struct CycleLoad {
    pub duration: f64,
    /// Lowest and highest tension of any belt section during the cycle, for machines that report
    /// belt tensions. A lowest tension of zero means a section went slack.
    pub belt_tension: Option<(f64, f64)>,
    /// Seconds any belt section spent without tension, where the machine counts them.
    pub belt_slack_time: Option<f64>,
    /// With effects.belt_tooth_jump: how often the belt's teeth jumped over a driven pulley during the cycle.
    pub belt_tooth_jumps: Option<f64>,
    /// With effects.belt_span_vibration: how close any free belt span's first sideways mode came to an excitation of
    /// the drive while the axes ran at a steady speed (the signal `belt.span_detuning`, sampled once per millisecond);
    /// 1 where they never did.
    pub belt_span_detuning: Option<f64>,
    /// The highest radial load a belt put on a pulley shaft, where the machine reports it.
    pub radial_load_peak: Option<f64>,
    /// The highest condition number of the kinematics' Jacobian during the cycle, where the machine reports it (the
    /// signal `kinematics.condition`, sampled once per millisecond): how near the tool came to a singularity.
    pub condition_peak: Option<f64>,
    /// Of the guide with the highest load, where the machine reports guide loads: its highest load and
    /// its cubic mean over the cycle, the equivalent load of the guide makers' life formula.
    pub guide_load: Option<(f64, f64)>,
    /// The mean |speed| of the axis of that guide over the cycle, m/s: the distance it runs per hour.
    pub guide_speed: Option<f64>,
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
    /// Where the energy drawn from the supply went, for a machine that reports its losses. `None` also where the
    /// effect views were not asked for (`analyse_cycle_with`).
    pub budget: Option<EnergyBudget>,
    /// With effects.part_heating, each part that warms with its loss (`Sim::heated`) and the rise it settles at when
    /// the cycle repeats for long: its thermal resistance times its mean loss over the measured pass. Empty without.
    pub part_rises: Vec<(ThermalPart, f64)>,
    /// The machine's own rating checks (`Machine::rating_checks`) and what the pass reached in each: the highest
    /// |signal|, or for a life check the life in hours, 1 / the mean damage rate (infinite where nothing wore).
    pub checks: Vec<(RatingCheck, f64)>,
    /// Where the config has obstacles: the least clearance to them over the whole run, the way to the cycle and its
    /// first pass included (`Sim::least_clearance`), m; and the run's first contact with one.
    pub clearance: Option<f64>,
    pub contact: Option<crate::Contact>,
    /// The highest braking power of all drives at once, W: with the DC link simulated, what the braking resistors
    /// took (a shared one counted once); without, the power the drives fed back. For an S120 line-up, which brakes
    /// through one Braking Module (`verdicts`).
    pub braking_power_peak: f64,
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
    /// The jump counters of the driven pulleys and their sum when the measurement began; the spans' detuning signal
    /// and its lowest value at steady speed.
    jumps: Vec<usize>,
    jumps_start: f64,
    detuning: Option<usize>,
    /// The signal of the Jacobian's condition number, and its highest value.
    condition: Option<usize>,
    condition_peak: f64,
    lowest_detuning: f64,
    /// The machine's rating checks, the places of their signals, and per check the highest |signal| and the sum of the
    /// signal, one per sample.
    checks: Vec<(RatingCheck, usize, f64, f64)>,
    sample: Vec<f64>,
    /// Every signal looked at in `sample` is one of `Sim::sample_base`: the effect views need not be sampled.
    base_only: bool,
    belt_tension: Option<(f64, f64)>,
    radial_peak: f64,
    /// Per guide: highest load and the sum of the load cubed, one per sample.
    guide_loads: Vec<(f64, f64)>,
    /// Per guide the place of its axis among the machine's axes, and the sum of that axis's |speed|, one per sample.
    guide_axes: Vec<Option<usize>>,
    guide_speeds: Vec<f64>,
    /// Per motor the signal of its pulley shaft's radial load, and the sums of load^3 |speed| and |speed| per sample.
    pulley_radial: Vec<Option<usize>>,
    radial_sums: Vec<(f64, f64)>,
    /// Per motor with a gearbox output torque: the sum of |torque|^(10/3) |speed|, and its peak per speed band.
    gear_life: Vec<f64>,
    gear_bands: Vec<Vec<f64>>,
    /// The slack-time counter when the measurement began.
    slack_start: f64,
    /// Per motor: sums of |torque|^3 |speed| and of |speed|.
    cubic: Vec<(f64, f64)>,
    /// Per motor, where the machine reports the torque at the gearbox output: its place among the machine's
    /// own signals, its peak and the sum of |torque|^3 |speed|. Looked at every step: a gear rings faster
    /// than once per millisecond.
    gear: Vec<Option<(usize, f64, f64)>>,
    machine_signals: Vec<f64>,
    /// Per motor, where the machine reports the parts of its torque: their places among the machine's own signals,
    /// and per part its peak and the sum of its square.
    parts: Vec<Option<[usize; 4]>>,
    part_sums: Vec<[(f64, f64); 4]>,
    /// The energy budget: the places of the machine's loss and energy signals, the energy lost in each, the energy
    /// parts, the windings' and the DC link's stored energy and the payload energy when the measurement began, and
    /// the drives' electrical energy, copper loss per motor, the energies of resistor, rectifier, mains and line module
    /// since. The first `budget_skip` steps are not counted.
    losses: Vec<usize>,
    lost: Vec<f64>,
    energies: Vec<usize>,
    start: (Vec<f64>, f64, f64, f64),
    /// The windings', capacitors' and payload energy after the last step counted, where the budget ends: the stores
    /// are read where the flows stop (the machine's from `machine_signals` of that step).
    end: (f64, f64, f64),
    /// `Sim::crash_energy` when the measurement began and after the last step counted.
    crash: (f64, f64),
    supply: f64,
    copper: Vec<f64>,
    resistor: f64,
    /// What the DC links of drives switched off lost through their bleeders (`Drive::discharge_power`).
    discharge: f64,
    drawn: f64,
    mains: f64,
    line_loss: f64,
    /// The work of the motor torques on the mechanics, as the drives integrated it.
    work: f64,
    budget_skip: u64,
    /// Per heated part the energy it lost since the measurement began, while the parts warm (effects.part_heating).
    heat: Vec<f64>,
    /// Whether the effect views (torque breakdown, energy budget) are added up: they need the machine's
    /// `effect_signals` every step.
    views: bool,
    /// Energy fed back since each motor last drew power.
    braking: Vec<f64>,
    /// `CycleLoad::braking_power_peak`.
    braking_peak: f64,
    /// The setpoints of the last half second: the piece of the path the tool can be near.
    path: std::collections::VecDeque<Vec<f64>>,
    path_error: (f64, f64, u64),
    tracking_error: f64,
    dwell_error: f64,
}

/// Distance from `p` to the piece of line between `a` and `b`.
fn distance_to_piece(p: &[f64], a: &[f64], b: &[f64]) -> f64 {
    // Without closures behind references: the path error asks this 500 times per millisecond.
    let (ab, ap) = (|i: usize| b[i] - a[i], |i: usize| p[i] - a[i]);
    let length = (0..p.len()).map(|i| ab(i) * ab(i)).sum::<f64>();
    let along = if length > 0.0 { ((0..p.len()).map(|i| ap(i) * ab(i)).sum::<f64>() / length).clamp(0.0, 1.0) } else { 0.0 };
    (0..p.len()).map(|i| (ap(i) - along * ab(i)).powi(2)).sum::<f64>().sqrt()
}

impl Tally {
    fn new(sim: &Sim, views: bool) -> Self {
        let motors = sim.machine.motors().len();
        let infos = sim.signal_infos();
        let named = |start: &str, end: &str| -> Vec<usize> { infos.iter().enumerate().filter(|(_, s)| s.id.starts_with(start) && s.id.ends_with(end)).map(|(i, _)| i).collect() };
        let guides = named("axis.", ".guide_load");
        let own = sim.machine.signal_infos();
        let gear = |motor: &sim_core::MotorInfo| Some((own.iter().position(|s| s.id == format!("gearbox.{}.output_torque", motor.id))?, 0.0, 0.0));
        let parts = |motor: &sim_core::MotorInfo| -> Option<[usize; 4]> {
            let found: Vec<usize> = TORQUE_PARTS.iter().filter_map(|part| own.iter().position(|s| s.id == format!("motor.{}.torque_{part}", motor.id))).collect();
            found.try_into().ok()
        };
        // Without the views, only what `base_signals` gives is looked at.
        let own_named = |prefix: &str| own.iter().enumerate().filter(|(_, s)| views && s.id.starts_with(prefix)).map(|(i, _)| i).collect::<Vec<_>>();
        let (losses, energies) = (own_named(LOSS_PREFIX), own_named("energy."));
        let mut machine_signals = Vec::new();
        if views {
            sim.machine.signals(&sim.state, &sim.torque, &mut machine_signals);
        } else {
            sim.machine.base_signals(&sim.state, &sim.torque, &mut machine_signals);
        }
        let (windings, capacitors) = stored_electrically(sim);
        let (tensions, radial, slack, jumps, detuning) =
            (named("belt.", ".tension"), named("pulley.", ".radial_load"), infos.iter().position(|s| s.id == "belt.slack_time"), named("pulley.", ".tooth_jumps"), infos.iter().position(|s| s.id == "belt.span_detuning"));
        let mut base = Vec::new();
        sim.sample_base(&mut base);
        let condition = infos.iter().position(|s| s.id == "kinematics.condition");
        let checks: Vec<(RatingCheck, usize, f64, f64)> = sim.machine.rating_checks().into_iter().filter_map(|c| Some((infos.iter().position(|s| s.id == c.signal)?, c))).map(|(i, c)| (c, i, 0.0, 0.0)).collect();
        let check_places: Vec<usize> = checks.iter().map(|c| c.1).collect();
        let base_only = tensions.iter().chain(&radial).chain(&guides).chain(&jumps).chain(&slack).chain(&detuning).chain(&condition).chain(&check_places).all(|&i| i < base.len());
        Self {
            gear: sim.machine.motors().iter().map(gear).collect(),
            parts: sim.machine.motors().iter().map(|m| if views { parts(m) } else { None }).collect(),
            part_sums: vec![[(0.0, 0.0); 4]; motors],
            lost: vec![0.0; losses.len()],
            start: (energies.iter().map(|&i| machine_signals[i]).collect(), windings, capacitors, sim.payload_energy),
            end: (windings, capacitors, sim.payload_energy),
            crash: (sim.crash_energy, sim.crash_energy),
            losses,
            energies,
            supply: 0.0,
            copper: vec![0.0; motors],
            resistor: 0.0,
            discharge: 0.0,
            drawn: 0.0,
            mains: 0.0,
            line_loss: 0.0,
            work: 0.0,
            budget_skip: 0,
            heat: vec![0.0; if sim.part_heating { sim.heated.len() } else { 0 }],
            views,
            machine_signals,
            radial,
            slack,
            jumps,
            jumps_start: 0.0,
            detuning,
            lowest_detuning: 1.0,
            condition,
            condition_peak: 0.0,
            checks,
            radial_peak: 0.0,
            guide_loads: vec![(0.0, 0.0); guides.len()],
            guide_axes: guides.iter().map(|&i| sim.machine.axes().iter().position(|a| infos[i].id == format!("axis.{}.guide_load", a.id))).collect(),
            guide_speeds: vec![0.0; guides.len()],
            pulley_radial: sim.machine.motors().iter().map(|m| infos.iter().position(|s| s.id == format!("pulley.{}.radial_load", m.id))).collect(),
            radial_sums: vec![(0.0, 0.0); motors],
            gear_life: vec![0.0; motors],
            gear_bands: vec![Vec::new(); motors],
            guides,
            slack_start: 0.0,
            cubic: vec![(0.0, 0.0); motors],
            loads: vec![MotorLoad::default(); motors],
            pos: { let (mut pos, mut speed) = (vec![0.0; motors], vec![0.0; motors]); sim.machine.motor_state(&sim.state, &mut pos, &mut speed); pos },
            speed: vec![0.0; motors],
            steps: 0,
            every: (1e-3 / sim.dt).round().max(1.0) as u64,
            tensions,
            sample: Vec::new(),
            base_only,
            belt_tension: None,
            braking: vec![0.0; motors],
            braking_peak: 0.0,
            path: std::collections::VecDeque::new(),
            path_error: (0.0, 0.0, 0),
            tracking_error: 0.0,
            dwell_error: 0.0,
        }
    }

    fn sample_now(&mut self, sim: &Sim) {
        if self.base_only { sim.sample_base(&mut self.sample) } else { sim.sample(&mut self.sample) }
    }

    /// Adds the step the simulation has just made.
    fn step(&mut self, sim: &Sim) {
        sim.machine.motor_state(&sim.state, &mut self.pos, &mut self.speed);
        let sampled = self.steps % self.every == 0;
        if sampled {
            self.sample_now(sim);
            for &i in &self.tensions {
                let (low, high) = self.belt_tension.unwrap_or((self.sample[i], self.sample[i]));
                self.belt_tension = Some((low.min(self.sample[i]), high.max(self.sample[i])));
            }
            for &i in &self.radial {
                self.radial_peak = self.radial_peak.max(self.sample[i]);
            }
            if let Some(i) = self.condition {
                self.condition_peak = self.condition_peak.max(self.sample[i]);
            }
            for (_, i, peak, sum) in &mut self.checks {
                (*peak, *sum) = (peak.max(self.sample[*i].abs()), *sum + self.sample[*i]);
            }
            for (&i, (peak, cubed)) in self.guides.iter().zip(&mut self.guide_loads) {
                (*peak, *cubed) = (peak.max(self.sample[i]), *cubed + self.sample[i].powi(3));
            }
            if let (0, Some(i)) = (self.steps, self.slack) {
                self.slack_start = self.sample[i];
            }
            if self.steps == 0 {
                self.jumps_start = self.jumps.iter().map(|&i| self.sample[i]).sum();
            }
            // Steady speed: no axis accelerates by its setpoint, and one moves. A sweep through a resonance while
            // speeding up is passed in moments; a cruise on it is not.
            let steady = sim.control.set_acc.iter().all(|a| a.abs() <= 1e-9) && sim.control.set_vel.iter().any(|v| v.abs() > 1e-9);
            if let (true, Some(i)) = (steady, self.detuning) {
                self.lowest_detuning = self.lowest_detuning.min(self.sample[i]);
            }
            // Path error: how far the tool is from the path the setpoint has drawn, wherever along it.
            // Tracking error: how far it is from where the setpoint is right now.
            let axes = sim.machine.axes().len();
            let (mut tool, mut rate) = (vec![0.0; axes], vec![0.0; axes]);
            sim.machine.tool_state(&sim.state, &mut tool, &mut rate);
            for (axis, sum) in self.guide_axes.iter().zip(&mut self.guide_speeds) {
                *sum += axis.map_or(0.0, |a| rate[a].abs());
            }
            for (k, (signal, (cubed, weight))) in self.pulley_radial.iter().zip(&mut self.radial_sums).enumerate() {
                if let Some(i) = *signal {
                    (*cubed, *weight) = (*cubed + self.sample[i].powi(3) * self.speed[k].abs(), *weight + self.speed[k].abs());
                }
            }
            // The oldest setpoint's place is reused for the newest.
            let mut newest = if self.path.len() >= 500 { self.path.pop_front().unwrap_or_default() } else { Vec::new() };
            newest.clone_from(&sim.control.set_pos);
            self.path.push_back(newest);
            let now = self.path.back().unwrap();
            // The nearest piece, newest first. A piece is at least as far as the tool is beyond its extent along any one
            // axis; one that is further than the nearest so far by more than a picometre (far above rounding) cannot be
            // the nearest, and is skipped. The smallest distance is the same as over all pieces.
            let mut off_path = distance_to_piece(&tool, now, now);
            for (b, a) in self.path.iter().rev().zip(self.path.iter().rev().skip(1)) {
                let beyond = (0..axes).map(|i| (tool[i] - a[i]).abs() - (b[i] - a[i]).abs()).fold(f64::NEG_INFINITY, f64::max);
                if !(beyond > off_path + 1e-12) {
                    off_path = off_path.min(distance_to_piece(&tool, a, b));
                }
            }
            self.path_error = (self.path_error.0.max(off_path), self.path_error.1 + off_path * off_path, self.path_error.2 + 1);
            self.tracking_error = self.tracking_error.max(distance_to_piece(&tool, now, now));
            if sim.control.dwell_progress().is_some_and(|done| done >= 0.5) {
                let target = sim.control.planned_pos();
                self.dwell_error = self.dwell_error.max(distance_to_piece(&tool, target, target));
            }
        }
        if self.views && (self.parts.iter().any(Option::is_some) || !self.losses.is_empty()) {
            self.machine_signals.clear();
            sim.machine.signals(&sim.state, &sim.torque, &mut self.machine_signals);
        } else if self.gear.iter().any(Option::is_some) {
            self.machine_signals.clear();
            sim.machine.base_signals(&sim.state, &sim.torque, &mut self.machine_signals);
        }
        for (places, sums) in self.parts.iter().zip(&mut self.part_sums).filter_map(|(p, s)| Some((p.as_ref()?, s))) {
            for (&i, (peak, squares)) in places.iter().zip(sums) {
                let value = self.machine_signals[i];
                (*peak, *squares) = (peak.max(value.abs()), *squares + value * value);
            }
        }
        if self.steps >= self.budget_skip {
            for (&i, lost) in self.losses.iter().zip(&mut self.lost) {
                *lost += self.machine_signals[i] * sim.dt;
            }
            // `Sim::step` has worked out the parts' losses of this step already.
            for (part, heat) in sim.heated.iter().zip(&mut self.heat) {
                *heat += part.loss_power * sim.dt;
            }
            let drives = &sim.control.drives;
            self.supply += drives.iter().map(|d| d.power).sum::<f64>() * sim.dt;
            self.work += drives.iter().map(|d| d.shaft_power).sum::<f64>() * sim.dt;
            for (copper, d) in self.copper.iter_mut().zip(drives) {
                *copper += d.copper_loss * sim.dt;
            }
            // A shared DC link has one resistor, rectifier and line module, which every drive reports as its own.
            let shared = sim.control.drive_params().shared_dc_link;
            let link = |power: fn(&crate::drive::Drive) -> f64| -> f64 {
                let total: f64 = if shared { drives.first().map_or(0.0, power) } else { drives.iter().map(power).sum() };
                total * sim.dt
            };
            self.resistor += link(|d| d.resistor_power);
            // Each drive discharges its own share of a shared link (`Drive::disabled_step`), so these add up.
            self.discharge += drives.iter().map(|d| d.discharge_power).sum::<f64>() * sim.dt;
            self.drawn += link(|d| d.rectifier_power);
            self.mains += link(|d| d.mains_power);
            self.line_loss += link(|d| d.line_loss);
            if !self.losses.is_empty() {
                let (windings, capacitors) = stored_electrically(sim);
                self.end = (windings, capacitors, sim.payload_energy);
                self.crash.1 = sim.crash_energy;
            }
        }
        for (k, (i, peak, cubed)) in self.gear.iter_mut().enumerate().filter_map(|(k, gear)| Some((k, gear.as_mut()?))) {
            let (torque, speed) = (self.machine_signals[*i].abs(), self.speed[k].abs());
            (*peak, *cubed) = (peak.max(torque), *cubed + torque.powi(3) * speed);
            self.gear_life[k] += torque.powf(10.0 / 3.0) * speed;
            // Capped, so that a run gone unstable cannot ask for a band per rad/s up to infinity.
            let (bands, band) = (&mut self.gear_bands[k], ((speed / SPEED_BAND) as usize).min(4000));
            if bands.len() <= band {
                bands.resize(band + 1, 0.0);
            }
            bands[band] = bands[band].max(torque);
        }
        for (k, load) in self.loads.iter_mut().enumerate() {
            let torque = sim.control.drives[k].torque;
            load.peak_torque = load.peak_torque.max(torque.abs());
            let (current, power) = (sim.control.drives[k].current, sim.control.drives[k].power);
            load.rms_current += current * current;
            load.peak_current = load.peak_current.max(current.abs());
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
            // The line module's load either way: drawn, or fed back as the DC link gives it (before its losses).
            let through = drive.rectifier_power + drive.mains_power + drive.line_loss;
            load.mains_energy += drive.mains_power * sim.dt;
            load.line_module_energy += through * sim.dt;
            load.peak_line_module_power = load.peak_line_module_power.max(through);
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
        let (drives, p) = (&sim.control.drives, sim.control.drive_params());
        let braking: f64 = match (p.dc_link, p.shared_dc_link) {
            (true, true) => drives.first().map_or(0.0, |d| d.resistor_power),
            (true, false) => drives.iter().map(|d| d.resistor_power).sum(),
            (false, _) => drives.iter().map(|d| (-d.power).max(0.0)).sum(),
        };
        self.braking_peak = self.braking_peak.max(braking);
        self.steps += 1;
    }

    fn finish(mut self, sim: &Sim, duration: f64) -> CycleLoad {
        let steps = self.steps.max(1) as f64;
        for (k, load) in self.loads.iter_mut().enumerate() {
            load.rms_torque = (load.rms_torque / steps).sqrt();
            load.rms_current = (load.rms_current / steps).sqrt();
            load.cubic_mean_torque = if self.cubic[k].1 > 0.0 { (self.cubic[k].0 / self.cubic[k].1).cbrt() } else { 0.0 };
            load.gearbox_torque = self.gear[k].map(|(_, peak, cubed)| (peak, if self.cubic[k].1 > 0.0 { (cubed / self.cubic[k].1).cbrt() } else { 0.0 }));
            load.mean_speed = self.cubic[k].1 / steps;
            load.gearbox_life_torque = self.gear[k].map(|_| if self.cubic[k].1 > 0.0 { (self.gear_life[k] / self.cubic[k].1).powf(0.3) } else { 0.0 });
            load.gearbox_torque_by_speed = std::mem::take(&mut self.gear_bands[k]);
            let (cubed, weight) = self.radial_sums[k];
            load.radial_load_equivalent = self.pulley_radial[k].map(|_| if weight > 0.0 { (cubed / weight).cbrt() } else { 0.0 });
            load.winding_rise = sim.control.drive_params().settled_rise(load.rms_current);
            load.thermal_utilisation = load.rms_torque / sim.control.rated_torque();
            load.torque_parts = self.parts[k].map(|_| self.part_sums[k].map(|(peak, squares)| (peak, (squares / steps).sqrt())));
            // A tripped drive's DC link stays where it tripped, also when that was before the measured pass.
            load.peak_dc_link_voltage = load.peak_dc_link_voltage.max(sim.control.drives[k].dc_voltage);
        }
        self.sample_now(sim);
        let budget = self.budget(sim);
        let part_rises = sim.heated.iter().zip(&self.heat).map(|(h, heat)| (h.part.clone(), h.resistance * heat / duration)).collect();
        let samples = self.path_error.2.max(1) as f64;
        // The guide that the life estimate is about: the one with the highest equivalent load.
        let guide = (0..self.guide_loads.len()).max_by(|&a, &b| self.guide_loads[a].1.total_cmp(&self.guide_loads[b].1));
        let guide_load = guide.map(|g| (self.guide_loads[g].0, (self.guide_loads[g].1 / samples).cbrt()));
        let guide_speed = guide.and_then(|g| self.guide_axes[g].map(|_| self.guide_speeds[g] / samples));
        CycleLoad {
            duration,
            belt_tension: self.belt_tension,
            belt_slack_time: self.slack.map(|i| self.sample[i] - self.slack_start),
            belt_tooth_jumps: (!self.jumps.is_empty()).then(|| self.jumps.iter().map(|&i| self.sample[i]).sum::<f64>() - self.jumps_start),
            belt_span_detuning: self.detuning.map(|_| self.lowest_detuning),
            radial_load_peak: (!self.radial.is_empty()).then_some(self.radial_peak),
            condition_peak: self.condition.map(|_| self.condition_peak),
            guide_load,
            guide_speed,
            path_error_peak: self.path_error.0,
            path_error_rms: (self.path_error.1 / self.path_error.2.max(1) as f64).sqrt(),
            tracking_error_peak: self.tracking_error,
            dwell_error_peak: self.dwell_error,
            tripped: sim.control.drives.iter().any(|d| d.tripped),
            motors: self.loads,
            budget,
            part_rises,
            checks: std::mem::take(&mut self.checks).into_iter().map(|(c, _, peak, sum)| { let value = if c.life { samples / sum } else { peak }; (c, value) }).collect(),
            clearance: (!sim.obstacles.is_empty()).then_some(sim.least_clearance),
            contact: sim.contact.clone(),
            braking_power_peak: self.braking_peak,
        }
    }

    /// The energy budget of the steps counted, where the machine reports its losses.
    fn budget(&self, sim: &Sim) -> Option<EnergyBudget> {
        if self.losses.is_empty() {
            return None;
        }
        let own = sim.machine.signal_infos();
        // The stores where the flows stop, after the last step counted: a cycle's measured pass ends with the step in
        // which the next pass begins, which is not counted, as the step in which it began is not.
        let (now, (windings, capacitors, payload)) = (&self.machine_signals, self.end);
        let row = |id: &str, owner_key: &str, quantity_key: &str, energy: f64| BudgetRow { id: id.into(), owner_key: owner_key.into(), quantity_key: quantity_key.into(), energy };
        let mut rows: Vec<BudgetRow> = sim.machine.motors().iter().zip(&self.copper).map(|(m, copper)| row(&format!("copper.{}", m.id), m.key, "budget-copper", *copper)).collect();
        let p = sim.control.drive_params();
        if p.dc_link {
            rows.push(row("resistor", "group-drive", "budget-resistor", self.resistor));
            rows.push(row("discharge", "group-drive", "budget-discharge", self.discharge));
        }
        if p.regenerative {
            rows.push(row("mains", "group-drive", "budget-mains", self.mains));
            rows.push(row("line_module", "group-drive", "budget-line-module", self.line_loss));
        }
        rows.extend(self.losses.iter().zip(&self.lost).map(|(&i, lost)| row(&own[i].id, &own[i].owner_key, &own[i].quantity_key, *lost)));
        rows.push(row("windings", "group-drive", "budget-windings", windings - self.start.1));
        if p.dc_link {
            rows.push(row("capacitors", "group-drive", "budget-capacitors", capacitors - self.start.2));
        }
        rows.extend(self.energies.iter().zip(&self.start.0).map(|(&i, start)| row(&own[i].id, &own[i].owner_key, &own[i].quantity_key, now[i] - start)));
        // A payload picked up brings energy into the machine, one put down takes it away.
        rows.push(row("payload", "group-energy", "budget-payload", self.start.3 - payload));
        // What the stops at obstacles took out of the machine (effects.crash).
        if sim.crash {
            rows.push(row("crash", "group-energy", "budget-crash", self.crash.1 - self.crash.0));
        }
        // With the DC link simulated the supply is what the rectifier drew, counted on its own: that the capacitors,
        // the resistor, the line module and the drives account for it is the check of `electrical_unaccounted`.
        let supply = if p.dc_link { self.drawn } else { self.supply };
        Some(EnergyBudget { supply, rows, shaft_work: self.work })
    }
}

/// Energy in the windings' inductance, and in the DC link capacitors where the DC link is simulated.
fn stored_electrically(sim: &Sim) -> (f64, f64) {
    let p = sim.control.drive_params();
    let drives = &sim.control.drives;
    let windings = drives.iter().map(|d| 0.5 * p.inductance * d.current * d.current).sum();
    (windings, if p.dc_link { drives.iter().map(|d| 0.5 * p.dc_capacitance * d.dc_voltage * d.dc_voltage).sum() } else { 0.0 })
}

/// Runs the configured cycle and measures its second pass, when the machine has settled into it. If a
/// drive trips before, the cycle never gets there: what is measured then is the run up to the trip.
pub fn analyse_cycle(sim: &mut Sim) -> Result<CycleLoad, CommandError> {
    analyse_cycle_with(sim, true)
}

/// `analyse_cycle`, with the effect views (`MotorLoad::torque_parts`, `CycleLoad::budget`) only where `views` asks
/// for them: they cost one more evaluation of the machine per step, and no verdict needs them.
pub fn analyse_cycle_with(sim: &mut Sim, views: bool) -> Result<CycleLoad, CommandError> {
    sim.command(Command::StartCycle)?;
    let mut tally = Tally::new(sim, views);
    let (mut passes, mut last, mut start) = (0, sim.control.cycle_passes(), 0.0);
    loop {
        sim.step();
        // The controller counts the passes: a cycle of one segment never changes its segment index.
        let now = sim.control.cycle_passes();
        if now != last {
            passes += 1;
            if passes == 2 {
                // The measured pass begins: what was added up on the way here goes. The energy budget starts from
                // the state after this step, so it leaves the step out.
                (start, tally) = (sim.time(), Tally::new(sim, views));
                tally.budget_skip = 1;
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
    analyse_trajectory_with(sim, points, true)
}

/// `analyse_trajectory`, with the effect views only where `views` asks for them (see `analyse_cycle_with`).
pub fn analyse_trajectory_with(sim: &mut Sim, points: Vec<(f64, Vec<f64>)>, views: bool) -> Result<CycleLoad, CommandError> {
    let end = points.last().map_or(0.0, |p| p.0) + 0.3;
    sim.command(Command::Trajectory(points))?;
    let (mut tally, start) = (Tally::new(sim, views), sim.time());
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

fn at_least(check: &'static str, motor: Option<usize>, value: f64, limit: f64) -> Verdict {
    Verdict { check, motor, value, limit, ok: value >= limit }
}

/// The checks whose value has to stay at or above its limit ("belt-slack": above); every other check's value has to
/// stay at or below it.
pub const AT_LEAST: [&str; 8] = ["belt-slack", "pulley-bearing-life", "gearbox-life", "guide-life", "belt-span-resonance", "screw-life", "strain-wave-life", "collision"];

/// How close a verdict is to its limit, as a share: value over limit, or for the checks of `AT_LEAST` limit over value.
/// Above 1 it fails (but "belt-slack" at exactly 1). A contact ("collision" at no clearance) is infinitely far over.
pub fn utilisation(v: &Verdict) -> f64 {
    if v.check == "collision" && v.value <= 0.0 {
        return f64::INFINITY;
    }
    if AT_LEAST.contains(&v.check) { v.limit / v.value } else { v.value / v.limit }
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
/// "gearbox-thermal" and "guide-thermal" judge, with effects.part_heating (or thermal_feedback, which warms the parts
/// too), the hottest gearbox and guide where they settle on the cycle (`CycleLoad::part_rises`) against
/// `thermal.<kind>_max_rise_k`. Without either they are not judged.
///
/// "gearbox-continuous" judges the cubic mean of that torque against the gearbox's rated torque, as gearbox
/// makers do; it used to judge the RMS torque, which is for motors and their heat.
/// "belt-slack" also fails when any belt section spent time without tension.
/// With `effects.dc_link` the braking checks judge the simulated DC link: what the capacitors held at the
/// highest voltage, the mean and peak power the resistor burned, and the highest voltage against the trip
/// level. Without it they are estimated from the energy the motor fed back, as before.
///
/// "line-module-continuous" and "line-module-peak" judge, with effects.regenerative_supply (and the DC link), the line
/// module's mean and highest power either way against `ratings.line_module_rated_power_w` and `_max_power_w`.
///
/// "pulley-bearing-life", "gearbox-life" and "guide-life" judge `pulley_bearing_life_h`, `gearbox_life_h` and
/// `guide_life_h` against `ratings.required_life_h` (they pass at or above it); "belt-tooth-load" judges
/// `belt_tooth_utilisation` against 1.
///
/// "singularity" judges the highest condition number of the kinematics' Jacobian in the cycle
/// (`CycleLoad::condition_peak`, where the machine reports it) against `ratings.max_condition_number`: near a singularity
/// the motors' torques and speeds for a tool force or speed grow without bound, and a tool force or speed for the motors'
/// shrinks.
///
/// "belt-tooth-jump" judges, with effects.belt_tooth_jump, the jumps of the belt's teeth during the cycle against 0;
/// "belt-span-resonance", with effects.belt_span_vibration, how close a span's first sideways mode came to an
/// excitation at steady speed (`CycleLoad::belt_span_detuning`) against `ratings.belt_span_resonance_band` (it passes
/// at or above it).
///
/// "collision" judges, where the config has obstacles, the least clearance of the run to them (`CycleLoad::clearance`)
/// against `profile.min_clearance_m`: it passes at or above it, never at a contact. Without obstacles it is not judged,
/// as a rating left out is not, so it cannot fail there.
pub fn verdicts(cfg: &Config, load: &CycleLoad) -> Result<Vec<Verdict>, ConfigError> {
    let cfg = &crate::catalog::parts_applied(cfg)?;
    // A rating that is no number (NaN) would otherwise count as left out, and not be judged.
    // The belt's tooth load table is two arrays; `belt_tooth_utilisation` checks them.
    for (name, value) in cfg.get("ratings").ok().and_then(|ratings| ratings.as_table()).into_iter().flatten() {
        if !value.is_array() {
            cfg.f64(&format!("ratings.{name}"))?;
        }
    }
    let rating = |name: &str| cfg.f64(&format!("ratings.{name}")).ok();
    let simulated = cfg.bool("effects.dc_link")?;
    let (bus, rated_rise) = (cfg.positive("drive.bus_voltage_v")?, cfg.positive("motor.rated_temperature_rise_k")?);
    let mut out = Vec::new();
    // An S120 line-up brakes through one Braking Module for all its axes (`drive.braking_module`): their braking power
    // is judged together, after the axes. An S210 has a resistor of its own and is judged per axis.
    let line_up = cfg.get("drive.braking_module").is_ok() && cfg.bool("drive.braking_module")?;
    let mut line_up_brakes = false;
    for (k, m) in load.motors.iter().enumerate() {
        // Peak torque against the torque-speed curve, and the settled winding temperature against the rated rise.
        out.push(at_most("motor-peak", Some(k), m.utilisation, 1.0));
        out.push(at_most("motor-thermal", Some(k), m.winding_rise, rated_rise));
        // The drive's output current: the cycle's RMS against its rated current (a cycle of seconds against a power
        // unit that warms in seconds to minutes), its peak against the maximum current.
        if let Some(limit) = rating("drive_rated_current_a") {
            out.push(at_most("drive-current", Some(k), m.rms_current, limit));
        }
        if let Some(limit) = rating("drive_max_current_a") {
            out.push(at_most("drive-peak-current", Some(k), m.peak_current, limit));
        }
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
        // Lives against the required life, and the belt's teeth against their speed-dependent rating.
        if reliability_factor(cfg).is_none() {
            return Err(ConfigError::Invalid("ratings.bearing_reliability_percent".into()));
        }
        if let Some(required) = rating("required_life_h") {
            if let Some(life) = pulley_bearing_life_h(cfg, m) {
                out.push(at_least("pulley-bearing-life", Some(k), life, required));
            }
            if let Some(life) = gearbox_life_h(cfg, m) {
                out.push(at_least("gearbox-life", Some(k), life, required));
            }
        }
        if let Some(utilisation) = belt_tooth_utilisation(cfg, m)? {
            out.push(at_most("belt-tooth-load", Some(k), utilisation, 1.0));
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
            line_up_brakes |= resistor;
            if resistor && !line_up {
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
        // The line module of a regenerative supply, through which the drive draws and feeds back: its mean power
        // either way against its rated power, its highest against its maximum power.
        if simulated && cfg.bool("effects.regenerative_supply")? {
            if let Some(limit) = rating("line_module_rated_power_w") {
                out.push(at_most("line-module-continuous", Some(k), m.line_module_energy / load.duration, limit));
            }
            if let Some(limit) = rating("line_module_max_power_w") {
                out.push(at_most("line-module-peak", Some(k), m.peak_line_module_power, limit));
            }
        }
    }
    // The line-up's mean braking power is the sum of the axes' means; its peak the peak of their sum at each step.
    if line_up && line_up_brakes {
        let shared = simulated && cfg.bool("effects.shared_dc_link")?;
        let energy: f64 = match (simulated, shared) {
            (true, true) => load.motors.first().map_or(0.0, |m| m.resistor_energy),
            (true, false) => load.motors.iter().map(|m| m.resistor_energy).sum(),
            (false, _) => load.motors.iter().map(|m| m.energy_regenerated).sum(),
        };
        if let Some(limit) = rating("braking_resistor_continuous_w") {
            out.push(at_most("braking-resistor-continuous", None, energy / load.duration, limit));
        }
        if let Some(limit) = rating("braking_resistor_peak_w") {
            out.push(at_most("braking-resistor-peak", None, load.braking_power_peak, limit));
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
    if let Some(jumps) = load.belt_tooth_jumps {
        out.push(at_most("belt-tooth-jump", None, jumps, 0.0));
    }
    if let (Some(detuning), Some(band)) = (load.belt_span_detuning, rating("belt_span_resonance_band")) {
        out.push(at_least("belt-span-resonance", None, detuning, band));
    }
    // The machine's own parts (`Machine::rating_checks`): a peak at most its limit, a life (an L10) times the life
    // factor a1 at least its required life.
    for (check, value) in &load.checks {
        let a1 = reliability_factor(cfg).ok_or_else(|| ConfigError::Invalid("ratings.bearing_reliability_percent".into()))?;
        out.push(if check.life { at_least(check.check, check.motor, value * a1, check.limit) } else { at_most(check.check, check.motor, *value, check.limit) });
    }
    // The least clearance to the obstacles against `profile.min_clearance_m`; a contact fails whatever the limit.
    if let Some(clearance) = load.clearance {
        let limit = cfg.non_negative("profile.min_clearance_m")?;
        out.push(Verdict { check: "collision", motor: None, value: clearance, limit, ok: clearance >= limit && clearance > 0.0 });
    }
    if let (Some(value), Some(limit)) = (load.condition_peak, rating("max_condition_number")) {
        out.push(at_most("singularity", None, value, limit));
    }
    if let (Some(value), Some(limit)) = (load.radial_load_peak, rating("shaft_max_radial_load_n")) {
        out.push(at_most("gearbox-radial-load", None, value, limit));
    }
    if let (Some((peak, _)), Some(limit)) = (load.guide_load, rating("guide_max_load_n")) {
        out.push(at_most("guide-load", None, peak, limit));
    }
    if let (Some(life), Some(required)) = (guide_life_h(cfg, load), rating("required_life_h")) {
        out.push(at_least("guide-life", None, life, required));
    }
    // The hottest part of each kind where it settles (`CycleLoad::part_rises`), against its rated rise; only where
    // the parts warm, as a rating left out is not judged either.
    if crate::sim::heats(cfg)? {
        for (kind, check) in [("gearbox", "gearbox-thermal"), ("guide", "guide-thermal")] {
            let hottest = load.part_rises.iter().filter(|(part, _)| part.kind == kind).map(|(_, rise)| *rise).reduce(f64::max);
            if let Some(rise) = hottest {
                out.push(at_most(check, None, rise, cfg.positive(&format!("thermal.{kind}_max_rise_k"))?));
            }
        }
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

/// `guide_life_km` in hours: the distance over the mean speed of the guide's axis over the cycle
/// (`CycleLoad::guide_speed`).
pub fn guide_life_h(cfg: &Config, load: &CycleLoad) -> Option<f64> {
    Some(guide_life_km(cfg, load)? * 1e3 / (load.guide_speed? * 3600.0))
}

/// The width of the speed bands of `MotorLoad::gearbox_torque_by_speed`, rad/s of the motor.
pub const SPEED_BAND: f64 = 5.0;

/// The basic rating life of a ball bearing in hours (ISO 281): L10h = (C / P)^3 · 10^6 / (60 n), with the dynamic load
/// rating C and the equivalent load P in the same unit and the mean speed n in r/min.
pub fn rating_life_h(c: f64, p: f64, rpm: f64) -> f64 {
    (c / p).powi(3) * 1e6 / (60.0 * rpm)
}

/// The life factor a1 of ISO 281 for the reliability R of `ratings.bearing_reliability_percent`, which turns the L10 of
/// a bearing into the life R % of bearings reach: a1 = 0.95 · (ln(100 / R) / ln(100 / 90))^(2/3) + 0.05, the Weibull
/// form behind the standard's table (1 at 90 %, 0.64 at 95 %, 0.25 at 99 %). 1 where the config has no reliability;
/// `None` for one outside 90 to 99.95 %, where the standard gives no factor.
pub fn reliability_factor(cfg: &Config) -> Option<f64> {
    let Ok(r) = cfg.f64("ratings.bearing_reliability_percent") else { return Some(1.0) };
    (90.0..=99.95).contains(&r).then(|| 0.95 * ((100.0 / r).ln() / (100.0_f64 / 90.0).ln()).powf(2.0 / 3.0) + 0.05)
}

/// The mean speed of the gearbox output (the pulley) in r/min: the motor's over `gearbox.ratio`.
fn output_rpm(cfg: &Config, m: &MotorLoad) -> f64 {
    m.mean_speed * 30.0 / std::f64::consts::PI / cfg.f64("gearbox.ratio").unwrap_or(1.0)
}

/// The rating life of the bearing that takes the belt's pull on this motor's pulley shaft, in hours, where the machine
/// reports that radial load and the config has `ratings.pulley_bearing_load_rating_n`: `rating_life_h` with that C, P the
/// cubic mean of the radial load weighted with the speed (`MotorLoad::radial_load_equivalent`) and the pulley's mean
/// speed, times the life factor a1 (`reliability_factor`). One bearing, all of the pull on it, no axial load, no a_ISO.
pub fn pulley_bearing_life_h(cfg: &Config, m: &MotorLoad) -> Option<f64> {
    let c = cfg.f64("ratings.pulley_bearing_load_rating_n").ok()?;
    Some(reliability_factor(cfg)? * rating_life_h(c, m.radial_load_equivalent?, output_rpm(cfg, m)))
}

/// The life of a planetary gearbox's planet bearings in hours by Siemens' formula (configuration manual "Planetary
/// gearboxes for S-1FK2 and S-1FT2 servomotors", 01/2023, section 6.3.3): L10h = 20000 h · (n2B / n2) · (M2N / M2eq)^(10/3),
/// with the rated torque M2N (`ratings.gearbox_rated_torque_nm`), the reference output speed n2B
/// (`ratings.gearbox_reference_speed_rpm`), the mean output speed n2 and M2eq the mean of the output torque with the
/// exponent 10/3 weighted with the speed (`MotorLoad::gearbox_life_torque`). Where the machine reports the output torque
/// and the config has both ratings. Times the life factor a1 (`reliability_factor`): Siemens' figure is an L10 too.
pub fn gearbox_life_h(cfg: &Config, m: &MotorLoad) -> Option<f64> {
    let (rated, reference) = (cfg.f64("ratings.gearbox_rated_torque_nm").ok()?, cfg.f64("ratings.gearbox_reference_speed_rpm").ok()?);
    Some(reliability_factor(cfg)? * 20000.0 * reference / output_rpm(cfg, m) * (rated / m.gearbox_life_torque?).powf(10.0 / 3.0))
}

/// The tooth load of the belt on this motor's pulley against its rating, by the method of Megadyne's MEGALINEAR handbook
/// (02.2026, page 9): the effective pull Fu = output torque / `geometry.pulley_radius_m` times the service factor
/// `ratings.belt_service_factor`, against the specific tooth load the teeth allow at the pulley's speed
/// (`ratings.belt_tooth_load_n_cm` at `ratings.belt_tooth_load_rpm`, interpolated, held beyond the ends) times the width in
/// cm (`ratings.belt_width_mm` / 10) and the teeth in mesh (`ratings.belt_teeth_in_mesh`, at most 12). The highest ratio
/// over the cycle, each speed band (`MotorLoad::gearbox_torque_by_speed`) at its upper speed, where the rating is lowest.
/// `None` where the machine reports no output torque or the config lacks one of these values. A rating, not a life: no
/// belt maker publishes a life in hours or load cycles for these belts.
pub fn belt_tooth_utilisation(cfg: &Config, m: &MotorLoad) -> Result<Option<f64>, ConfigError> {
    let Ok(rpm) = cfg.f64s("ratings.belt_tooth_load_rpm") else { return Ok(None) };
    let get = |key: &str| cfg.f64(key).ok();
    let (Some(width), Some(teeth), Some(factor), Some(radius)) = (get("ratings.belt_width_mm"), get("ratings.belt_teeth_in_mesh"), get("ratings.belt_service_factor"), get("geometry.pulley_radius_m")) else { return Ok(None) };
    let load = cfg.f64s("ratings.belt_tooth_load_n_cm")?;
    if load.len() != rpm.len() || rpm.is_empty() || rpm.windows(2).any(|w| w[0] >= w[1]) {
        return Err(ConfigError::Invalid("ratings.belt_tooth_load_rpm".into()));
    }
    if m.gearbox_torque.is_none() {
        return Ok(None);
    }
    let allowed = |n: f64| -> f64 {
        let i = rpm.partition_point(|&r| r <= n);
        if i == 0 {
            load[0]
        } else if i == rpm.len() {
            load[i - 1]
        } else {
            load[i - 1] + (load[i] - load[i - 1]) * (n - rpm[i - 1]) / (rpm[i] - rpm[i - 1])
        }
    };
    let ratio = cfg.f64("gearbox.ratio").unwrap_or(1.0);
    let capacity = |n: f64| allowed(n) * width / 10.0 * teeth.min(12.0);
    let band_rpm = |i: usize| (i + 1) as f64 * SPEED_BAND * 30.0 / std::f64::consts::PI / ratio;
    Ok(Some(m.gearbox_torque_by_speed.iter().enumerate().map(|(i, torque)| torque / radius * factor / capacity(band_rpm(i))).fold(0.0, f64::max)))
}

/// The low and high bound of every uncertain parameter of a config, as (path, low, high).
///
/// The table `[bounds]` mirrors the config: where the config has `[friction_x] coulomb_n = 15.0`, it has
/// `friction_x.coulomb_n = [10.0, 20.0]`, in the unit of the value. A pair and not a relative width,
/// because the parameter panel and the translation check skip arrays (`Config::leaves`), so bounds need
/// no labels, and because a value of zero can have bounds this way. A bound on a value the config does
/// not have, or one that is not a low and a high number, is refused. A bound on a value that only an effect that
/// is off uses (`PATH_EFFECTS`, e.g. `thermal.*` without effects.part_heating), or that no cycle uses (the DC link's
/// discharge while disabled), is checked and left out.
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
    out.retain(|(path, ..)| !effect_off(cfg, path));
    Ok(out)
}

/// Values (by the start of their path) that only some effects use, and those effects: they change nothing while all
/// of them are off. None: a value no cycle uses, the discharge of a disabled drive's DC link. The parts warm with
/// effects.thermal_feedback too (`sim::heats`).
const PATH_EFFECTS: [(&str, &[&str]); 5] = [
    ("thermal.", &["part_heating", "thermal_feedback"]),
    ("thermal_feedback.", &["thermal_feedback"]),
    ("belt.tooth_jump_factor", &["belt_tooth_jump"]),
    ("ratings.line_module_efficiency", &["regenerative_supply"]),
    ("ratings.dc_link_discharge_", &[]),
];

/// Whether `path` is a value only an effect uses (`PATH_EFFECTS`), and that effect is off in `cfg`, or a value no cycle
/// uses: varying it would only run the cycle again for nothing.
fn effect_off(cfg: &Config, path: &str) -> bool {
    PATH_EFFECTS.iter().any(|(start, effects)| path.starts_with(start) && !effects.iter().any(|effect| cfg.bool(&format!("effects.{effect}")).unwrap_or(false)))
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
    let load = analyse_cycle_with(&mut Sim::new(entry, cfg)?, false).map_err(|e| ConfigError::Invalid(e.key.into()))?;
    verdicts(cfg, &load)
}

/// The cycle of each config `make(0..count)` makes, judged; the runs are independent, so they share the processors.
/// In the order of `make`.
fn judge_each(entry: &MachineEntry, count: usize, make: impl Fn(usize) -> Result<Config, ConfigError> + Sync) -> Vec<Result<Vec<Verdict>, ConfigError>> {
    on_all_cores(count, |run| make(run).and_then(|cfg| judged(entry, &cfg)))
}

/// `work(0..count)` on all processors, in the order of the index: for runs that do not depend on each other.
fn on_all_cores<T: Send>(count: usize, work: impl Fn(usize) -> T + Sync) -> Vec<T> {
    let next = std::sync::atomic::AtomicUsize::new(0);
    let worker = || -> Vec<(usize, T)> {
        let mut done = Vec::new();
        loop {
            let run = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            if run >= count {
                return done;
            }
            done.push((run, work(run)));
        }
    };
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()).min(count.max(1));
    // One processor, or none known (WebAssembly, which cannot start threads): on this thread.
    let mut runs: Vec<_> = if threads <= 1 {
        worker()
    } else {
        std::thread::scope(|scope| {
            let workers: Vec<_> = (0..threads).map(|_| scope.spawn(worker)).collect();
            workers.into_iter().flat_map(|worker| worker.join().expect("a run of the cycle panicked")).collect()
        })
    };
    runs.sort_by_key(|(run, _)| *run);
    runs.into_iter().map(|(_, run)| run).collect()
}

/// The bands of the cycle of `cfg` (already with its parts in) and of the configs `changed(0..count)` makes of
/// it. The runs are independent, so they share the processors; the bands come out the same in any case.
fn envelope(entry: &MachineEntry, cfg: &Config, count: usize, changed: impl Fn(usize) -> Result<Config, ConfigError> + Sync) -> Result<Vec<Band>, ConfigError> {
    // Run 0 is the config as it is.
    bands_of(judge_each(entry, count + 1, |run| if run == 0 { Ok(cfg.clone()) } else { changed(run - 1) }))
}

/// The bands of these judged runs: per check the envelope, whether every run passed, and the share that did.
fn bands_of(runs: Vec<Result<Vec<Verdict>, ConfigError>>) -> Result<Vec<Band>, ConfigError> {
    // The failed runs are counted and divided once: subtracting 1 / total per failure left -5e-16 where all failed.
    let (total, mut bands, mut failed) = (runs.len(), Vec::<Band>::new(), Vec::<usize>::new());
    for run in runs {
        for v in run? {
            let fails = usize::from(!v.ok);
            match bands.iter().position(|b| b.verdict.check == v.check && b.verdict.motor == v.motor) {
                Some(k) => {
                    let b = &mut bands[k];
                    (b.low, b.high, b.ok) = (b.low.min(v.value), b.high.max(v.value), b.ok && v.ok);
                    failed[k] += fails;
                }
                None => {
                    bands.push(Band { low: v.value, high: v.value, ok: v.ok, share: 1.0, verdict: v });
                    failed.push(fails);
                }
            }
        }
    }
    for (band, failed) in bands.iter_mut().zip(failed) {
        band.share = (total - failed) as f64 / total as f64;
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
/// corner (`verdict_worst` does). With n parameters there are 2^n corners, and a few dozen runs meet none of
/// them: a check that fails only with several values near their bounds at once can pass every sample.
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

/// The worst combination of the bounds found for one verdict: the verdict as judged there, and per parameter of the
/// bounds (in their order) whether it is at its high bound (`Some(true)`), its low one (`Some(false)`) or as
/// configured (`None`: where a run with one parameter alone at a bound, or the config as it is, was the worst).
#[derive(Debug, Clone, PartialEq)]
pub struct Worst {
    pub verdict: Verdict,
    pub high: Vec<Option<bool>>,
}

/// How many passes of flipping one parameter at a time `verdict_worst` makes at most.
pub const WORST_PASSES: usize = 3;

/// How bad a verdict is: how far its value is beyond its limit, or how much room it leaves below as a negative
/// number (for the checks of `AT_LEAST` the value has to stay above its limit). The margin and not the value,
/// because some limits are bounded themselves (the DC link's capacitance and threshold): a corner that raises the
/// threshold raises the highest voltage too, but leaves more room.
///
/// Not so "dc-link-energy": it fails only where the braking resistor is needed and has no rating, and with a resistor
/// the link fills up to its room at every corner that brakes hard (the margin is 0 at all of them). It is ranked by the
/// energy in the capacitors, any failing run above any passing one.
fn badness(v: &Verdict) -> f64 {
    match v.check {
        // J: no braking stores a gigajoule.
        "dc-link-energy" => v.value + if v.ok { 0.0 } else { 1e9 },
        _ if AT_LEAST.contains(&v.check) => v.limit - v.value,
        _ => v.value - v.limit,
    }
}

/// A search for the worst combination of the uncertain parameters, per verdict. With n parameters the bounds have
/// 2^n corners, too many to run; the search looks at few of them:
///
/// 1. The cycle with each parameter alone at its low and its high bound (as `verdict_bands`, 2 n runs).
/// 2. Per verdict the corner where each parameter is at the bound that made that verdict worse alone (a parameter
///    that moved it not at all goes to its high bound). Where the verdict is a sum of effects that do not depend on
///    each other (a monotone function of each parameter), this is the worst corner.
/// 3. Up to `WORST_PASSES` passes per verdict: from its corner every parameter that moved the verdict alone is
///    flipped to its other bound, one at a time; the flips that made it worse are taken together, and the worse of
///    that corner and the best single flip becomes the corner, if it is worse than before. The search ends when a
///    pass finds nothing worse. This catches parameters whose sign changes with the others (interactions).
///
/// The same corner is run once for all verdicts that land on it, and the runs share the processors. What is
/// reported per verdict is the worst value it took in any run, the runs of step 1 included, with where the parameters
/// were: never better than `verdict_bands`. Returns the
/// verdicts in the order they came up (as `verdict_bands`) and the number of cycles simulated.
///
/// It is a search, not a proof: a verdict that is worst at a corner which no single flip from the sign corner
/// leads to, or inside the bounds and not at a corner at all (a resonance hit by a stiffness between its bounds),
/// can be missed. `verdict_bands_sampled` looks inside.
pub fn verdict_worst(entry: &MachineEntry, cfg: &Config, bounds: &[(String, f64, f64)]) -> Result<(Vec<Worst>, usize), ConfigError> {
    verdict_worst_and_bands(entry, cfg, bounds).map(|(worst, _, runs)| (worst, runs))
}

/// `verdict_worst`, and `verdict_bands` of the same bounds from the runs the search starts with, which are those of the
/// bands (the config as it is, then each parameter at its low and at its high bound): the bands cost no cycle more.
pub fn verdict_worst_and_bands(entry: &MachineEntry, cfg: &Config, bounds: &[(String, f64, f64)]) -> Result<(Vec<Worst>, Vec<Band>, usize), ConfigError> {
    let cfg = &crate::catalog::resolve(entry, cfg)?;
    let n = bounds.len();
    let set = |cfg: &mut Config, i: usize, high: bool| cfg.set_text(&format!("{} = {:?}", bounds[i].0, if high { bounds[i].2 } else { bounds[i].1 }));
    // Run 0 is the config as it is, then each parameter at its low and at its high bound.
    let alone = judge_each(entry, 2 * n + 1, |run| {
        let mut changed = cfg.clone();
        if run > 0 {
            set(&mut changed, (run - 1) / 2, run % 2 == 0)?;
        }
        Ok(changed)
    });
    let bands = bands_of(alone.clone())?;
    let alone = alone.into_iter().collect::<Result<Vec<_>, _>>()?;
    let mut ids: Vec<(&'static str, Option<usize>)> = Vec::new();
    for v in alone.iter().flatten() {
        if !ids.contains(&(v.check, v.motor)) {
            ids.push((v.check, v.motor));
        }
    }
    // How bad a verdict is in a run; a run in which it did not come up counts as the best.
    let bad = |run: &[Verdict], id: (&str, Option<usize>)| run.iter().find(|v| (v.check, v.motor) == id).map_or(f64::NEG_INFINITY, badness);
    let mut corners: Vec<Vec<bool>> = ids.iter().map(|&id| (0..n).map(|i| bad(&alone[2 * i + 2], id) >= bad(&alone[2 * i + 1], id)).collect()).collect();
    let moves: Vec<Vec<usize>> = ids.iter().map(|&id| (0..n).filter(|&i| bad(&alone[2 * i + 2], id) != bad(&alone[2 * i + 1], id)).collect()).collect();

    let mut seen: std::collections::HashMap<Vec<bool>, Vec<Verdict>> = std::collections::HashMap::new();
    let mut runs = alone.len();
    // Runs the corners not run before.
    let mut run_corners = |wanted: Vec<Vec<bool>>, seen: &mut std::collections::HashMap<Vec<bool>, Vec<Verdict>>| -> Result<(), ConfigError> {
        let mut new: Vec<Vec<bool>> = Vec::new();
        for corner in wanted {
            if !seen.contains_key(&corner) && !new.contains(&corner) {
                new.push(corner);
            }
        }
        let judged = judge_each(entry, new.len(), |k| {
            let mut changed = cfg.clone();
            for (i, high) in new[k].iter().enumerate() {
                set(&mut changed, i, *high)?;
            }
            Ok(changed)
        });
        runs += new.len();
        for (corner, verdicts) in new.into_iter().zip(judged) {
            seen.insert(corner, verdicts?);
        }
        Ok(())
    };
    run_corners(corners.clone(), &mut seen)?;
    let flipped = |corner: &[bool], flips: &[usize]| {
        let mut out = corner.to_vec();
        for &i in flips {
            out[i] = !out[i];
        }
        out
    };
    for _ in 0..WORST_PASSES {
        let singles: Vec<Vec<bool>> = (0..ids.len()).flat_map(|k| moves[k].iter().map(|&i| flipped(&corners[k], &[i])).collect::<Vec<_>>()).collect();
        run_corners(singles, &mut seen)?;
        // Per verdict the flips that made it worse, and all of them together.
        let worse: Vec<Vec<usize>> = (0..ids.len()).map(|k| moves[k].iter().copied().filter(|&i| bad(&seen[&flipped(&corners[k], &[i])], ids[k]) > bad(&seen[&corners[k]], ids[k])).collect()).collect();
        run_corners((0..ids.len()).filter(|&k| worse[k].len() > 1).map(|k| flipped(&corners[k], &worse[k])).collect(), &mut seen)?;
        let mut changed = false;
        for k in 0..ids.len() {
            let candidates = worse[k].iter().map(|&i| flipped(&corners[k], &[i])).chain((worse[k].len() > 1).then(|| flipped(&corners[k], &worse[k])));
            if let Some(best) = candidates.max_by(|a, b| bad(&seen[a], ids[k]).total_cmp(&bad(&seen[b], ids[k]))) {
                corners[k] = best;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    // Per verdict the worst it was in any run: at a corner, or with one parameter at a bound or none (so that the worst
    // is never better than `verdict_bands` reports).
    let alone_at = |run: usize| -> Vec<Option<bool>> { (0..n).map(|i| (run > 0 && (run - 1) / 2 == i).then_some(run % 2 == 0)).collect() };
    let worst = ids
        .iter()
        .filter_map(|&id| {
            let corners = seen.iter().map(|(corner, run)| (corner.iter().map(|&h| Some(h)).collect::<Vec<_>>(), run));
            let candidates = corners.chain(alone.iter().enumerate().map(|(r, run)| (alone_at(r), run)));
            let (high, run) = candidates.max_by(|a, b| bad(a.1, id).total_cmp(&bad(b.1, id)).then_with(|| b.0.cmp(&a.0)))?;
            let verdict = run.iter().find(|v| (v.check, v.motor) == id)?.clone();
            Some(Worst { verdict, high })
        })
        .collect();
    Ok((worst, bands, runs))
}

/// A cycle made faster: the cycle before and after, the settings that changed (path, before, after) and
/// the number of cycles that were simulated on the way. Where the optimiser changed the cycle's geometry, `changes`
/// has the whole new cycle as the entry "cycle.segment" (TOML, for `Config::set_text`), and `lifts` says what changed.
#[derive(Debug, Clone)]
pub struct Optimised {
    pub before: CycleLoad,
    pub after: CycleLoad,
    pub changes: Vec<(String, String, String)>,
    pub lifts: Vec<LiftChange>,
    pub runs: usize,
}

/// A lift, traverse and lower of the cycle that the optimiser changed: the three segments of the configured cycle
/// (counted from 0), the height of the traverse (the last axis) before and after, and the via point the path is rounded
/// through, where it took one (then the three segments are one: a path up, through the via point and down, see
/// `control::via_pieces`).
#[derive(Debug, Clone, PartialEq)]
pub struct LiftChange {
    pub segments: [usize; 3],
    pub before: f64,
    pub after: f64,
    pub via: Option<Vec<f64>>,
}

/// A lift, traverse and lower of a configured cycle (`lifts`): segment `lift` goes from `from` straight along the last
/// axis to `height`, `traverse` along the other axes at that height, `lower` straight along the last axis to `to`.
/// `up` is the side (+1 or -1 on the last axis) both ends lie away from; `lowest` the height nearest them that keeps
/// `profile.min_clearance_m` to the farther of the two.
struct Lift {
    segments: [usize; 3],
    from: Vec<f64>,
    to: Vec<f64>,
    height: f64,
    up: f64,
    lowest: f64,
}

/// The lifts of a cycle: three segments in a row (round the end of the cycle too) of which the first moves along the
/// last axis only, away from both the start and the end of the three, the second along the other axes only, at that
/// height, and the third along the last axis only. The first two neither dwell nor change the payload, none has via
/// points. A lift already as low as the clearance allows is a lift all the same (it can still be rounded).
fn lifts(cfg: &Config, clearance: f64) -> Result<Vec<Lift>, ConfigError> {
    let segments = cfg.array("cycle.segment")?;
    let n = segments.len();
    let target = |i: usize| -> Option<Vec<f64>> { segments[i].get("target")?.as_array()?.iter().map(|v| v.as_float().or_else(|| v.as_integer().map(|i| i as f64))).collect() };
    let plain = |i: usize| segments[i].get("via").is_none();
    let quiet = |i: usize| segments[i].get("dwell_s").and_then(|d| d.as_float().or_else(|| d.as_integer().map(|i| i as f64))) == Some(0.0) && segments[i].get("payload") == segments[(i + n - 1) % n].get("payload");
    let mut out = Vec::new();
    if n < 3 {
        return Ok(out);
    }
    for a in 0..n {
        let (b, c) = ((a + 1) % n, (a + 2) % n);
        let (Some(from), Some(lift), Some(traverse), Some(to)) = (target((a + n - 1) % n), target(a), target(b), target(c)) else { continue };
        let z = from.len().wrapping_sub(1);
        if from.len() < 2 || [&lift, &traverse, &to].iter().any(|p| p.len() != from.len()) {
            continue;
        }
        let along_z = |p: &[f64], q: &[f64]| p[..z] == q[..z] && p[z] != q[z];
        let height = lift[z];
        let up = (height - from[z]).signum();
        if !(along_z(&from, &lift) && traverse[z] == height && traverse[..z] != lift[..z] && along_z(&traverse, &to) && (height - to[z]).signum() == up) {
            continue;
        }
        if !(quiet(a) && quiet(b) && plain(a) && plain(b) && plain(c)) {
            continue;
        }
        let lowest = if up > 0.0 { from[z].max(to[z]) + clearance } else { from[z].min(to[z]) - clearance };
        // A clearance that is the lift as configured, but for rounding (0.05 + 0.3 is 0.35 and a bit), is that lift.
        let lowest = if (lowest - height).abs() < 1e-9 { height } else { lowest };
        // A lift that is lower than the clearance asks for as configured is left as it is.
        if up * (height - lowest) >= 0.0 {
            out.push(Lift { segments: [a, b, c], from, to, height, up, lowest });
        }
    }
    Ok(out)
}

/// The cycle with each lift at `heights` and, where `vias` has one, rounded through that via point (see `LiftChange`).
fn cycle_with(cfg: &Config, lifts: &[Lift], heights: &[f64], vias: &[Option<Vec<f64>>]) -> Result<toml::Value, ConfigError> {
    let mut segments = cfg.array("cycle.segment")?.clone();
    let number_list = |p: &[f64]| toml::Value::Array(p.iter().map(|x| toml::Value::Float(*x)).collect());
    let mut dropped = Vec::new();
    for ((lift, &height), via) in lifts.iter().zip(heights).zip(vias) {
        let [a, b, c] = lift.segments;
        let at_height = |p: &[f64]| {
            let mut p = p.to_vec();
            *p.last_mut().unwrap_or(&mut 0.0) = height;
            p
        };
        match via {
            None => {
                for i in [a, b] {
                    let old = segments[i].get("target").and_then(|t| t.as_array()).cloned().unwrap_or_default();
                    let pose: Vec<f64> = old.iter().filter_map(|v| v.as_float().or_else(|| v.as_integer().map(|i| i as f64))).collect();
                    if let Some(t) = segments[i].as_table_mut() {
                        t.insert("target".into(), number_list(&at_height(&pose)));
                    }
                }
            }
            Some(point) => {
                let via = toml::Value::Array(vec![number_list(&at_height(&lift.from)), number_list(point), number_list(&at_height(&lift.to))]);
                if let Some(t) = segments[c].as_table_mut() {
                    t.insert("via".into(), via);
                }
                dropped.extend([a, b]);
            }
        }
    }
    let kept = segments.into_iter().enumerate().filter(|(i, _)| !dropped.contains(i)).map(|(_, s)| s).collect();
    Ok(toml::Value::Array(kept))
}

/// Whether the path of a lift rounded through `via` keeps its clearance: every planned pose that lies between the two
/// ends along the other axes is at `lowest` or beyond, less the path tolerance the transition curves may use.
fn clears(axes: &[sim_core::AxisInfo], cycle: f64, tolerance: f64, lift: &Lift, height: f64, via: &[f64]) -> bool {
    let z = lift.from.len() - 1;
    let at_height = |p: &[f64]| {
        let mut p = p.to_vec();
        p[z] = height;
        p
    };
    let pieces = crate::control::via_pieces(&[at_height(&lift.from), via.to_vec(), at_height(&lift.to)], &lift.to);
    let Ok(points) = crate::path::plan(axes, &lift.from, &pieces, cycle, tolerance, crate::path::ARC_SLACK) else { return false };
    let across: Vec<f64> = (0..z).map(|i| lift.to[i] - lift.from[i]).collect();
    let width = across.iter().map(|d| d * d).sum::<f64>();
    points.iter().all(|(_, pose)| {
        let along = (0..z).map(|i| (pose[i] - lift.from[i]) * across[i]).sum::<f64>() / width;
        along <= 0.0 || along >= 1.0 || lift.up * (pose[z] - lift.lowest) >= -tolerance - 1e-9
    })
}

/// Shortens the cycle of a config without breaking a verdict. Tries corner blending, then raises speed,
/// acceleration and jerk of all axes together in steps for as long as the cycle gets shorter and every
/// verdict of `verdicts` still passes, among them the settling tolerance, so a faster move that leaves
/// the tool swinging at the pick does not count. Input shaping makes no cycle shorter by itself (every
/// stop waits one period of the shaper), so it is tried where it can help: a faster setting that fails
/// on "settling" alone is run again with the shaper, and with the shaper following the pose where the
/// machine reports a resonance, and is taken if it then passes and is shorter. A setting with the shaper that still
/// fails on "settling" alone is run again with a second shaper convolved with it, at `profile.shaper2_frequency_hz`
/// where the config has one, else at the resonance the machine reports at the start pose where that is more than 10 %
/// off the first shaper's frequency (no second shaper is tried where neither is). Where a shaper is on already,
/// switching it off is tried too. Simulates at most `max_runs` cycles.
///
/// The geometry of the cycle is a variable too. Each lift, traverse and lower (see `lifts`) is tried first with all
/// lifts as low as `profile.min_clearance_m` allows (that far beyond the farther of pick and place), then each alone at
/// that height and halfway down to it. Last, once no other step helps, the corners of a lift are rounded: the three
/// segments become one path up to the lift height, through a via point and down (`control::via_pieces`, timed by the
/// path planner with the transition curves of `profile.path_tolerance_m`), the via point at the middle and a half, a
/// quarter and an eighth of the width above the lift height; then moved by an eighth and a thirty-second of the width
/// along each axis, or the rounding dropped. A via point whose planned path comes nearer to pick and place than the
/// clearance between them is not run. Where the config has obstacles, a setting whose run comes nearer to one than the
/// clearance fails the verdict "collision" and is not taken; the cycle as configured is refused if it does.
///
/// A search by trial, not a proof of the optimum: it stops at the first setting from which no single step
/// is an improvement.
pub fn optimise_cycle(entry: &MachineEntry, cfg: &Config, max_runs: usize) -> Result<Optimised, ConfigError> {
    let probe = Sim::new(entry, cfg)?;
    let lifts = lifts(cfg, cfg.non_negative("profile.min_clearance_m")?)?;
    let (cycle_s, tolerance) = (cfg.positive("profile.cycle_s")?, cfg.non_negative("profile.path_tolerance_m")?);
    let axis_infos = probe.machine.axes().to_vec();
    let axes: Vec<&'static str> = probe.machine.axes().iter().map(|a| a.id).collect();
    let resonance = probe.machine.resonance_hz(probe.control.planned_pos());
    let (first, configured) = (cfg.positive("profile.shaper_frequency_hz")?, cfg.non_negative("profile.shaper2_frequency_hz")?);
    let second_frequency = if configured > 0.0 { Some(configured) } else { resonance.filter(|f| (f / first - 1.0).abs() > 0.1) };
    let limits = ["max_velocity_m_s", "max_acceleration_m_s2", "max_jerk_m_s3"];
    // A candidate: the factor on each of the three limits, the switches, and per lift its height and via point.
    #[derive(Clone, PartialEq)]
    struct Setting {
        factors: [f64; 3],
        blending: bool,
        shaping: bool,
        follows: bool,
        second: bool,
        heights: Vec<f64>,
        vias: Vec<Option<Vec<f64>>>,
    }
    let build = |setting: &Setting| -> Result<Config, ConfigError> {
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
        changed.set_text(&format!("profile.shaper2_frequency_hz = {:?}", if setting.second { second_frequency.unwrap_or(0.0) } else { 0.0 }))?;
        if setting.heights.iter().zip(&lifts).any(|(h, lift)| *h != lift.height) || setting.vias.iter().any(Option::is_some) {
            changed.set("cycle.segment", cycle_with(cfg, &lifts, &setting.heights, &setting.vias)?)?;
        }
        Ok(changed)
    };
    let runs = std::cell::Cell::new(0usize);
    // The cycle of a setting and the checks it fails, if it runs.
    let judged = |setting: &Setting| -> Option<(CycleLoad, Vec<&'static str>)> {
        runs.set(runs.get() + 1);
        let changed = build(setting).ok()?;
        let mut sim = Sim::new(entry, &changed).ok()?;
        if !sim.limit_violations.is_empty() {
            return None;
        }
        let load = analyse_cycle_with(&mut sim, false).ok()?;
        let failed = verdicts(&changed, &load).ok()?.iter().filter(|v| !v.ok).map(|v| v.check).collect();
        Some((load, failed))
    };
    let start = Setting {
        factors: [1.0; 3],
        blending: cfg.bool("effects.corner_blending")?,
        shaping: cfg.bool("effects.input_shaping")?,
        follows: cfg.bool("effects.shaper_follows_pose")?,
        second: configured > 0.0,
        heights: lifts.iter().map(|lift| lift.height).collect(),
        vias: vec![None; lifts.len()],
    };
    let before = match judged(&start) {
        Some((load, failed)) if failed.is_empty() => load,
        _ => return Err(ConfigError::Invalid("the cycle as configured fails a verdict".into())),
    };
    let (mut best, mut best_load) = (start.clone(), before.clone());
    let mut improved = true;
    while improved {
        improved = false;
        // The lifts lower, all at once and each alone, to the clearance and halfway there.
        let mut candidates = Vec::new();
        let lower = |setting: &Setting, k: usize, height: f64| {
            let mut lower = setting.clone();
            lower.heights[k] = height;
            lower
        };
        if lifts.iter().zip(&best.heights).filter(|(lift, h)| **h != lift.lowest).count() > 1 {
            candidates.push(Setting { heights: lifts.iter().map(|lift| lift.lowest).collect(), ..best.clone() });
        }
        for (k, lift) in lifts.iter().enumerate().filter(|(k, lift)| best.heights[*k] != lift.lowest) {
            candidates.push(lower(&best, k, lift.lowest));
            candidates.push(lower(&best, k, 0.5 * (best.heights[k] + lift.lowest)));
        }
        candidates.push(Setting { blending: !best.blending, ..best.clone() });
        if best.shaping {
            candidates.push(Setting { shaping: false, ..best.clone() });
        }
        if best.second {
            candidates.push(Setting { second: false, ..best.clone() });
        }
        for i in 0..3 {
            for step in [1.5, 1.2] {
                let mut faster = best.clone();
                faster.factors[i] *= step;
                candidates.push(faster);
            }
        }
        // Rounded lifts: a via point to start with, or one moved, or none again.
        for (k, lift) in lifts.iter().enumerate() {
            let z = lift.from.len() - 1;
            let width = (0..z).map(|i| (lift.to[i] - lift.from[i]).powi(2)).sum::<f64>().sqrt();
            let with_via = |via: Option<Vec<f64>>| {
                let mut out = best.clone();
                out.vias[k] = via;
                out
            };
            match &best.vias[k] {
                None => {
                    for above in [0.5, 0.25, 0.125] {
                        let mut via: Vec<f64> = (0..z).map(|i| 0.5 * (lift.from[i] + lift.to[i])).collect();
                        via.push(best.heights[k] + lift.up * above * width);
                        candidates.push(with_via(Some(via)));
                    }
                }
                Some(via) => {
                    for step in [0.125, 1.0 / 32.0] {
                        for axis in 0..=z {
                            for sign in [-1.0, 1.0] {
                                let mut moved = via.clone();
                                moved[axis] += sign * step * width;
                                candidates.push(with_via(Some(moved)));
                            }
                        }
                    }
                    candidates.push(with_via(None));
                }
            }
        }
        // A via point whose path would come too near to pick and place is not run.
        candidates.retain(|c| lifts.iter().enumerate().all(|(k, lift)| c.vias[k].as_ref().is_none_or(|via| clears(&axis_infos, cycle_s, tolerance, lift, c.heights[k], via))));
        'candidates: for candidate in candidates {
            // A candidate that only leaves the tool swinging gets its tries with the shaper right after it.
            let mut tries = std::collections::VecDeque::from([candidate]);
            while let Some(setting) = tries.pop_front() {
                if runs.get() >= max_runs {
                    break 'candidates;
                }
                let Some((load, failed)) = judged(&setting) else { continue };
                if failed.is_empty() && load.duration < best_load.duration - 1e-4 {
                    (best, best_load, improved) = (setting, load, true);
                    break 'candidates;
                }
                if !failed.is_empty() && failed.iter().all(|check| *check == "settling") {
                    if !setting.shaping {
                        tries.push_back(Setting { shaping: true, follows: false, ..setting.clone() });
                        if resonance.is_some() {
                            tries.push_back(Setting { shaping: true, follows: true, ..setting.clone() });
                        }
                    } else if second_frequency.is_some() && !setting.second {
                        tries.push_back(Setting { second: true, ..setting.clone() });
                    }
                }
            }
        }
    }
    let (from, to) = (build(&start)?, build(&best)?);
    let mut changes: Vec<(String, String, String)> = to
        .leaves()
        .into_iter()
        .zip(from.leaves())
        .filter(|((_, after), (_, before))| after != before)
        .map(|((path, after), (_, before))| (path, before.to_string(), after.to_string()))
        .collect();
    let (old_cycle, new_cycle) = (from.get("cycle.segment")?, to.get("cycle.segment")?);
    if old_cycle != new_cycle {
        changes.push(("cycle.segment".into(), old_cycle.to_string(), new_cycle.to_string()));
    }
    let lifts = lifts
        .iter()
        .enumerate()
        .filter(|(k, lift)| best.heights[*k] != lift.height || best.vias[*k].is_some())
        .map(|(k, lift)| LiftChange { segments: lift.segments, before: lift.height, after: best.heights[k], via: best.vias[k].clone() })
        .collect();
    Ok(Optimised { before, after: best_load, changes, lifts, runs: runs.get() })
}

/// Keys of `[ratings]` that are a choice of how to judge, not uncertain data of the machine, with a range of their own
/// that 20 % either way leaves: the reliability of the life estimates (90 to 99.95 %, `verdicts` refuses 72 and 108).
const NOT_MACHINE_DATA: [&str; 1] = ["ratings.bearing_reliability_percent"];

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
/// so which are worth measuring. A parameter that is not a number, whose changed value the machine
/// refuses, or that only an effect that is off uses (see `bounds`), is left out, and so are the analysis' own choices
/// in `NOT_MACHINE_DATA`.
pub fn sensitivity(entry: &MachineEntry, cfg: &Config, paths: &[String], delta: f64) -> Result<(CycleLoad, Vec<Sensitivity>), ConfigError> {
    let cfg = &crate::catalog::resolve(entry, cfg)?;
    let configured = bounds(cfg)?;
    let run = |cfg: &Config| -> Option<CycleLoad> { analyse_cycle_with(&mut Sim::new(entry, cfg).ok()?, false).ok() };
    // (path, value, low, high) of each parameter that is varied; the runs are independent and share the processors.
    let varied: Vec<(&String, f64, (f64, f64))> = paths
        .iter()
        .filter(|path| !effect_off(cfg, path) && !NOT_MACHINE_DATA.contains(&path.as_str()))
        .filter_map(|path| {
            let value = cfg.f64(path).ok()?;
            Some((path, value, configured.iter().find(|b| b.0 == *path).map_or((value * (1.0 - delta), value * (1.0 + delta)), |b| (b.1, b.2))))
        })
        .collect();
    // Run 0 is the config as it is, then each parameter at its low and at its high value.
    let mut runs = on_all_cores(2 * varied.len() + 1, |k| {
        if k == 0 {
            return run(cfg);
        }
        let (path, _, bounds) = varied[(k - 1) / 2];
        let mut changed = cfg.clone();
        changed.set_text(&format!("{path} = {:?}", if k % 2 == 1 { bounds.0 } else { bounds.1 })).ok()?;
        run(&changed)
    })
    .into_iter();
    let nominal = runs.next().flatten().ok_or_else(|| ConfigError::Invalid("cycle".into()))?;
    let mut out = Vec::new();
    for (path, value, bounds) in varied {
        if let (Some(low), Some(high)) = (runs.next().flatten(), runs.next().flatten()) {
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

/// How many frequencies a sweep has unless told otherwise. The first default machine's lowest dip is about
/// 2 Hz wide at 22 Hz; 120 points from 5 to 500 Hz put two points into it, 60 points one.
pub const SWEEP_POINTS: usize = 120;

/// The torque amplitude of a sweep unless told otherwise: 1.5 N m, or 40 % of the drive's peak torque where
/// that is less. At 0.05 N m the stiction of the default machines holds the motors and there is no dip,
/// and up to 1 N m friction still pulls the dip of the second default machine low (70 Hz at 0.5 N m, 81 at
/// 1, 84 at 1.5); above 1.5 N m that machine shows a second, false dip (62 Hz at 2 N m) from its
/// nonlinearities.
/// The 40 % leave a small motor (0.56 N m peak) most of its torque to hold the axis with.
pub fn default_sweep_amplitude(entry: &MachineEntry, cfg: &Config) -> Result<f64, ConfigError> {
    let cfg = crate::catalog::resolve(entry, cfg)?;
    Ok((0.4 * cfg.positive("drive.peak_torque_nm")?).min(1.5))
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

/// One point of the open-loop response of a motor's velocity loop (`velocity_loop_response`): the loop gain L
/// as a complex number (real, imaginary part) and the sensitivity |S| = |1 / (1 + L)| as measured. L is NaN where the
/// point is not a measurement of the linear loop: the torque limit was hit, or the loop moved on its own.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopPoint {
    pub frequency: f64,
    pub re: f64,
    pub im: f64,
    pub sensitivity: f64,
}

/// The open-loop response of the velocity loop of one motor, measured on the closed loop as a drive's own
/// measurement function does: a sine torque d is added to what the loop asks for (u), before the limit, with the
/// machine holding its start pose. The torque that reaches the motor is e = u + d, the loop answers u = -L e, so
/// L = -U / E and S = E / D at the frequency. Everything between the two is in L: velocity filter, PI, torque
/// filter, delays, current loop, the elastic mechanics, and the position loop around it. Sampled at the velocity
/// cycle, over whole periods: at least 0.1 s to settle and 0.1 s to measure.
pub fn velocity_loop_response(entry: &MachineEntry, cfg: &Config, motor: usize, frequencies: &[f64], amplitude: f64) -> Result<Vec<LoopPoint>, ConfigError> {
    check_frequencies(frequencies)?;
    let mut out = Vec::new();
    for &frequency in frequencies {
        let (mut sim, _) = settled(entry, cfg, 0)?;
        if motor >= sim.control.drives.len() {
            return Err(ConfigError::Invalid("motor".into()));
        }
        let every = (sim.control.drive_params().velocity_cycle / sim.dt).round() as u64;
        let (omega, t0, period) = (std::f64::consts::TAU * frequency, sim.time(), 1.0 / frequency);
        let whole = |seconds: f64| ((seconds / period).ceil().max(5.0) * period / sim.dt).round() as u64;
        let (settle_steps, total) = (whole(0.1), whole(0.1) + whole(0.1));
        let (mut u_ab, mut e_ab, mut samples) = ([0.0; 2], [0.0; 2], 0.0);
        let (mut seen, mut limited, mut pos, mut speed) = (Vec::new(), false, vec![0.0; sim.control.drives.len()], vec![0.0; sim.control.drives.len()]);
        for step in 0..total {
            let due = sim.step % every == 0;
            let (s, c) = (omega * (sim.time() - t0)).sin_cos();
            if due {
                sim.control.drives[motor].torque_injection = amplitude * s;
            }
            sim.step();
            if due {
                // What reaches the motor clipped by the torque limit is no longer the linear loop being measured.
                sim.machine.motor_state(&sim.state, &mut pos, &mut speed);
                let (low, high) = sim.control.drive_params().torque_range(speed[motor]);
                let drive = &sim.control.drives[motor];
                let e = drive.torque_demand + drive.torque_injection;
                limited |= !(low < e && e < high);
                if step >= settle_steps {
                    let u = drive.torque_demand;
                    (u_ab[0], u_ab[1], e_ab[0], e_ab[1]) = (u_ab[0] + u * s, u_ab[1] + u * c, e_ab[0] + e * s, e_ab[1] + e * c);
                    samples += 1.0;
                    seen.push((s, c, e));
                }
            }
        }
        // Phasors (sine part, cosine part) of u and e; L = -U / E.
        let (u, e) = ([2.0 * u_ab[0] / samples, 2.0 * u_ab[1] / samples], [2.0 * e_ab[0] / samples, 2.0 * e_ab[1] / samples]);
        // What of e is not the sine of the injection and not a constant: a loop that moves on its own (unstable, or in a
        // limit cycle) shows here. More of it than the injection itself, and the point is not a measurement of L.
        let mean = seen.iter().map(|(_, _, x)| x).sum::<f64>() / samples;
        let rest = (seen.iter().map(|(s, c, x)| (x - mean - e[0] * s - e[1] * c).powi(2)).sum::<f64>() / samples).sqrt();
        let e2 = e[0] * e[0] + e[1] * e[1];
        let (re, im) = if limited || !(rest < amplitude) { (f64::NAN, f64::NAN) } else { (-(u[0] * e[0] + u[1] * e[1]) / e2, -(u[1] * e[0] - u[0] * e[1]) / e2) };
        out.push(LoopPoint { frequency, re, im, sensitivity: e2.sqrt() / amplitude });
    }
    Ok(out)
}

/// What an open-loop response says about the stability of the loop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopMargins {
    /// The gain crossover (|L| = 1) with the smallest phase margin; NaN without one.
    pub crossover_hz: f64,
    /// 180 degrees plus the phase of L at that crossover, the phase unwrapped along the frequency axis and taken
    /// between -360 and 0 degrees at the first crossover; infinite without a crossover. Negative: the loop is unstable.
    pub phase_margin_deg: f64,
    /// By how much the gain may rise before L reaches -1, in dB, at the worst crossing of the negative real axis
    /// where the phase falls through -180 degrees and that a later rising crossing beyond -1 does not undo;
    /// infinite without one. Negative where |L| > 1 there: the loop encircles -1 and is unstable. A falling crossing
    /// beyond -1 that a rising one undoes (the integrators turn the phase past -180 degrees below the crossover and
    /// it comes back) is the conditionally stable kind: a gain that falls would be the danger there.
    pub gain_margin_db: f64,
    /// False when a point of the measurement was not valid (`LoopPoint` with NaN: the torque limit was hit or the
    /// loop moved on its own); the margins are then NaN, and a check fails.
    pub valid: bool,
    /// The highest sensitivity |S| measured, and where. 1 / |S| is the shortest distance of L from -1: |S| <= 2
    /// guarantees a gain margin of at least 6 dB and a phase margin of at least 29 degrees.
    pub peak_sensitivity: f64,
    pub peak_frequency_hz: f64,
}

impl LoopMargins {
    /// A valid measurement, both margins positive: the loop is stable.
    pub fn stable(&self) -> bool {
        self.valid && self.phase_margin_deg > 0.0 && self.gain_margin_db > 0.0
    }
}

/// Margins from an open-loop response, between neighbouring points interpolated linearly in log |L| and in the
/// unwrapped phase. Points too far apart (more than 180 degrees of phase between them) can miss a sharp resonance.
/// A point without a measurement (NaN) makes the margins not valid (and NaN).
pub fn loop_margins(points: &[LoopPoint]) -> LoopMargins {
    let measured = |p: &&LoopPoint| p.re.is_finite() && p.im.is_finite();
    let mut m = LoopMargins { crossover_hz: f64::NAN, phase_margin_deg: f64::INFINITY, gain_margin_db: f64::INFINITY, valid: points.iter().all(|p| measured(&p)), peak_sensitivity: 0.0, peak_frequency_hz: f64::NAN };
    let points: Vec<&LoopPoint> = points.iter().filter(measured).collect();
    for p in &points {
        if p.sensitivity > m.peak_sensitivity {
            (m.peak_sensitivity, m.peak_frequency_hz) = (p.sensitivity, p.frequency);
        }
    }
    if !m.valid {
        (m.phase_margin_deg, m.gain_margin_db) = (f64::NAN, f64::NAN);
        return m;
    }
    // Phase in degrees, unwrapped along the frequency axis, then shifted by whole turns (below).
    let mut phase: Vec<f64> = Vec::with_capacity(points.len());
    for p in &points {
        let raw = p.im.atan2(p.re).to_degrees();
        phase.push(phase.last().map_or(raw, |last| raw + 360.0 * ((last - raw) / 360.0).round()));
    }
    let gain: Vec<f64> = points.iter().map(|p| p.re.hypot(p.im).ln()).collect();
    // Gain crossovers: (segment, share of the way along it).
    let crossovers: Vec<(usize, f64)> = (1..points.len()).filter(|&i| gain[i - 1].signum() != gain[i].signum()).map(|i| (i, gain[i - 1] / (gain[i - 1] - gain[i]))).collect();
    // The turn is fixed by the phase at the first crossover, taken in (-360, 0]; without a crossover by the first
    // point, taken in (-270, 90] (a loop with up to three integrators, or a lead, there).
    let shift = match crossovers.first() {
        Some(&(i, share)) => -360.0 * ((phase[i - 1] + (phase[i] - phase[i - 1]) * share) / 360.0).ceil(),
        None => phase.first().map_or(0.0, |p| -360.0 * ((p - 90.0) / 360.0).ceil()),
    };
    phase.iter_mut().for_each(|p| *p += shift);
    for &(i, share) in &crossovers {
        let margin = 180.0 + phase[i - 1] + (phase[i] - phase[i - 1]) * share;
        if margin < m.phase_margin_deg {
            (m.phase_margin_deg, m.crossover_hz) = (margin, points[i - 1].frequency * (points[i].frequency / points[i - 1].frequency).powf(share));
        }
    }
    // Crossings of the negative real axis (the phase through an odd multiple of -180 degrees), in frequency order,
    // from a phase of 0 at the gain of the first point: below the measured range each integrator has turned the
    // phase by -90 degrees from there (the small circle of the Nyquist contour around s = 0). Beyond -1 (|L| > 1) a
    // falling crossing is an encirclement of -1 and a later rising one beyond -1 undoes it (Nyquist: the loop is
    // stable when none is left). The gain margin is the worst falling crossing not undone: negative where |L| > 1.
    let (mut open, mut worst) = (Vec::new(), f64::INFINITY);
    let samples: Vec<(f64, f64)> = points.first().map(|_| (0.0, gain[0])).into_iter().chain(phase.iter().copied().zip(gain.iter().copied())).collect();
    for w in samples.windows(2) {
        let ((pa, ga), (pb, gb)) = (w[0], w[1]);
        let at = |level: f64| ga + (gb - ga) * (pa - level) / (pa - pb);
        let mut level = -180.0 + 360.0 * ((pa + 180.0) / 360.0).floor();
        while pb < level && level <= pa {
            let g = at(level);
            if g > 0.0 { open.push(g) } else { worst = worst.min(-g) }
            level -= 360.0;
        }
        level += 360.0;
        while pa < level && level <= pb {
            // shortcut: a rising crossing beyond -1 with nothing to undo (an unstable open loop) is not counted.
            if at(level) > 0.0 {
                open.pop();
            }
            level += 360.0;
        }
    }
    m.gain_margin_db = open.iter().fold(worst, |w, g| w.min(-g)) * 20.0 / std::f64::consts::LN_10;
    m
}

/// Frequencies of the velocity-loop measurement of `velocity_loop_margins`: 10 Hz to
/// a quarter of the velocity loop's sampling rate (2 kHz at 125 us; above it a period has fewer than four samples), 60 points.
pub fn loop_frequencies(cfg: &Config) -> Result<Vec<f64>, ConfigError> {
    Ok(log_spaced(10.0, 0.25 / cfg.positive("drive.velocity_cycle_s")?, 60))
}

/// The amplitude of the test torque: a tenth of the motor's rated torque.
pub fn loop_amplitude(entry: &MachineEntry, cfg: &Config) -> Result<f64, ConfigError> {
    Ok(0.1 * crate::catalog::resolve(entry, cfg)?.positive("drive.rated_torque_nm")?)
}

/// The margins of every motor's velocity loop for the config as it is and with each value of `bounds` at its low
/// and at its high bound, one at a time (as `verdict_bands`): per run its name ("" for the config as it is,
/// "path=value" otherwise) and the margins per motor. The runs share the processors.
pub fn velocity_loop_margins(entry: &MachineEntry, cfg: &Config, bounds: &[(String, f64, f64)]) -> Result<Vec<(String, Vec<LoopMargins>)>, ConfigError> {
    let cfg = &crate::catalog::resolve(entry, cfg)?;
    let (frequencies, amplitude, motors) = (loop_frequencies(cfg)?, loop_amplitude(entry, cfg)?, (entry.build)(cfg)?.motors().len());
    let mut runs = vec![(String::new(), cfg.clone())];
    for (path, low, high) in bounds {
        for value in [low, high] {
            let mut changed = cfg.clone();
            changed.set_text(&format!("{path} = {value:?}"))?;
            runs.push((format!("{path}={value:?}"), changed));
        }
    }
    let jobs: Vec<(usize, usize)> = (0..runs.len()).flat_map(|r| (0..motors).map(move |k| (r, k))).collect();
    let next = std::sync::atomic::AtomicUsize::new(0);
    let work = || {
        let mut done = Vec::new();
        while let Some(&(run, motor)) = jobs.get(next.fetch_add(1, std::sync::atomic::Ordering::Relaxed)) {
            done.push((run, motor, velocity_loop_response(entry, &runs[run].1, motor, &frequencies, amplitude).map(|p| loop_margins(&p))));
        }
        done
    };
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()).min(jobs.len());
    let found: Vec<_> = if threads <= 1 {
        work()
    } else {
        std::thread::scope(|scope| {
            let workers: Vec<_> = (0..threads).map(|_| scope.spawn(work)).collect();
            workers.into_iter().flat_map(|w| w.join().expect("a loop measurement panicked")).collect()
        })
    };
    let mut out: Vec<(String, Vec<LoopMargins>)> = runs.iter().map(|(name, _)| (name.clone(), vec![loop_margins(&[]); motors])).collect();
    for (run, motor, margins) in found {
        out[run].1[motor] = margins?;
    }
    Ok(out)
}

/// The same cycle run twice, once with an effect on and once with it off (`compare_effect`), for overlaying the two.
#[derive(Debug, Clone)]
pub struct Comparison {
    pub signals: Vec<SignalInfo>,
    /// Every signal once per millisecond over the measured pass, the time counted from its start: with the effect on,
    /// and with it off.
    pub on: Vec<Vec<f64>>,
    pub off: Vec<Vec<f64>>,
}

impl Comparison {
    /// Peak and RMS of (on - off) of one signal over the rows both runs have. Where one pass is longer (an effect
    /// that changes the timing, such as corner blending), the rest of it is not compared.
    pub fn difference(&self, signal: usize) -> (f64, f64) {
        let (mut peak, mut sum, mut n) = (0f64, 0.0, 0usize);
        for (a, b) in self.on.iter().zip(&self.off) {
            let d = a[signal] - b[signal];
            (peak, sum, n) = (peak.max(d.abs()), sum + d * d, n + 1);
        }
        (peak, if n > 0 { (sum / n as f64).sqrt() } else { 0.0 })
    }
}

/// Runs the configured cycle and keeps every signal once per millisecond over its second pass, as `analyse_cycle`
/// measures it, with the time counted from the start of that pass.
pub(crate) fn record_cycle(entry: &MachineEntry, cfg: &Config) -> Result<Vec<Vec<f64>>, ConfigError> {
    let mut sim = Sim::new(entry, cfg)?;
    let refused = |e: CommandError| ConfigError::Invalid(e.key.into());
    sim.command(Command::StartCycle).map_err(refused)?;
    let every = ((1e-3 / sim.dt).round() as u64).max(1);
    let (mut passes, mut last, mut start, mut rows, mut sample) = (0, sim.control.cycle_passes(), None, Vec::new(), Vec::new());
    loop {
        sim.step();
        let now = sim.control.cycle_passes();
        if now != last {
            passes += 1;
            if passes == 2 {
                start = Some(sim.step);
            } else if passes == 3 {
                break;
            }
        }
        last = now;
        if sim.time() > 600.0 {
            return Err(refused(CommandError { key: "error-cycle-timeout" }));
        }
        if let Some(first) = start.filter(|first| (sim.step - first) % every == 0) {
            sim.sample(&mut sample);
            sample[0] = (sim.step - first) as f64 * sim.dt;
            rows.push(sample.clone());
        }
        // Tripped drives never finish the cycle.
        if !sim.control.enabled() {
            break;
        }
    }
    Ok(rows)
}

/// The cycle of `on` against that of `off`: both run on simulations of their own; they are expected to be of the same
/// machine, with the same signals.
pub fn compare_configs(entry: &MachineEntry, on: &Config, off: &Config) -> Result<Comparison, ConfigError> {
    let signals = Sim::new(entry, on)?.signal_infos();
    let (with, without) = if std::thread::available_parallelism().is_err() {
        (record_cycle(entry, on), record_cycle(entry, off))
    } else {
        std::thread::scope(|scope| {
            let with = scope.spawn(|| record_cycle(entry, on));
            let without = record_cycle(entry, off);
            (with.join().expect("a run of the cycle panicked"), without)
        })
    };
    Ok(Comparison { signals, on: with?, off: without? })
}

/// The cycle of `cfg` with `effects.<effect>` on and with it off, whatever the config has it at.
pub fn compare_effect(entry: &MachineEntry, cfg: &Config, effect: &str) -> Result<Comparison, ConfigError> {
    let path = format!("effects.{effect}");
    cfg.bool(&path)?;
    let (mut on, mut off) = (cfg.clone(), cfg.clone());
    on.set_text(&format!("{path} = true"))?;
    off.set_text(&format!("{path} = false"))?;
    compare_configs(entry, &on, &off)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The lives and the tooth load from given loads, by hand. Pulley bearing (ISO 281): C 8469 N, P 600 N, the motor at
    /// 500 r/min behind a ratio of 5, so the pulley at 100 r/min: (8469 / 600)^3 * 10^6 / (60 * 100) = 468696 h. Gearbox
    /// (Siemens): 20000 h * (175 / 100) * (40 N m / 20 N m)^(10/3) = 352778 h. Guide: (5000 / 1000)^3 * 50 km = 6250 km,
    /// at 0.25 m/s 6944.4 h. Belt teeth (Megadyne): 2 N m at a motor speed up to 50 rad/s, the pulley at up to
    /// 95.49 r/min, where the table allows 35 - 95.49 / 100 = 34.045 N/cm; 100 N * 1.4 / (34.045 * 2.5 cm * 12 teeth,
    /// 14 capped) = 0.13707. Beyond the table's end (1000 r/min) its last value holds: 1.6 N m at up to 1000 rad/s,
    /// 80 N * 1.4 / (26 * 30) = 0.14359, the highest of the bands.
    #[test]
    fn the_lives_and_the_tooth_load_are_the_hand_calculated_ones() {
        let cfg = Config::parse(
            "format_version = 1\n[gearbox]\nratio = 5.0\n[geometry]\npulley_radius_m = 0.02\n[ratings]\npulley_bearing_load_rating_n = 8469.0\ngearbox_rated_torque_nm = 40.0\n\
             gearbox_reference_speed_rpm = 175.0\nguide_dynamic_load_rating_n = 5000.0\nbelt_tooth_load_rpm = [0.0, 100.0, 1000.0]\n\
             belt_tooth_load_n_cm = [35.0, 34.0, 26.0]\nbelt_width_mm = 25.0\nbelt_service_factor = 1.4\nbelt_teeth_in_mesh = 14.0\nbearing_reliability_percent = 90.0\n",
        )
        .unwrap();
        let mut bands = vec![0.0; 200];
        bands[9] = 2.0;
        let m = MotorLoad {
            mean_speed: 500.0 * std::f64::consts::PI / 30.0,
            radial_load_equivalent: Some(600.0),
            gearbox_life_torque: Some(20.0),
            gearbox_torque: Some((2.0, 1.0)),
            gearbox_torque_by_speed: bands.clone(),
            ..MotorLoad::default()
        };
        let close = |a: f64, b: f64, what: &str| assert!((a / b - 1.0).abs() < 1e-5, "{what}: {a} against {b} by hand");
        close(rating_life_h(8469.0, 600.0, 100.0), 468696.16, "rating life");
        close(pulley_bearing_life_h(&cfg, &m).unwrap(), 468696.16, "pulley bearing");
        close(gearbox_life_h(&cfg, &m).unwrap(), 352777.89, "gearbox");
        close(belt_tooth_utilisation(&cfg, &m).unwrap().unwrap(), 0.137073, "tooth load");
        bands[199] = 1.6;
        close(belt_tooth_utilisation(&cfg, &MotorLoad { gearbox_torque_by_speed: bands, ..m.clone() }).unwrap().unwrap(), 112.0 / 780.0, "tooth load beyond the table");
        let load = CycleLoad {
            duration: 1.0,
            belt_tension: None,
            belt_slack_time: None,
            belt_tooth_jumps: None,
            belt_span_detuning: None,
            radial_load_peak: None,
            condition_peak: None,
            guide_load: Some((1500.0, 1000.0)),
            guide_speed: Some(0.25),
            path_error_peak: 0.0,
            path_error_rms: 0.0,
            tracking_error_peak: 0.0,
            dwell_error_peak: 0.0,
            tripped: false,
            motors: vec![m.clone()],
            budget: None,
            part_rises: Vec::new(),
            checks: Vec::new(),
            clearance: None,
            contact: None,
            braking_power_peak: 0.0,
        };
        close(guide_life_km(&cfg, &load).unwrap(), 6250.0, "guide km");
        close(guide_life_h(&cfg, &load).unwrap(), 6944.444, "guide h");
        // What a config or a machine lacks gives no figure: no output torque, no radial load, no rating.
        assert_eq!(belt_tooth_utilisation(&cfg, &MotorLoad { gearbox_torque: None, ..m.clone() }).unwrap(), None);
        assert_eq!(pulley_bearing_life_h(&cfg, &MotorLoad { radial_load_equivalent: None, ..m.clone() }), None);
        assert_eq!(gearbox_life_h(&Config::parse("format_version = 1\n[ratings]\ngearbox_rated_torque_nm = 40.0\n").unwrap(), &m), None);
        // Life factor a1 (ISO 281), by hand: 0.95 * (ln(100/99) / ln(100/90))^(2/3) + 0.05 = 0.248332 at 99 %
        // (the table's 0.25); 0.637912 at 95 % (0.64); 0.076832 at 99.95 % (0.077). Both bearing lives scale with it.
        let mut reliable = cfg.clone();
        for (percent, a1) in [(95.0, 0.637912), (99.0, 0.248332), (99.95, 0.076832)] {
            reliable.set_text(&format!("ratings.bearing_reliability_percent = {percent:?}")).unwrap();
            close(reliability_factor(&reliable).unwrap(), a1, "a1");
            close(pulley_bearing_life_h(&reliable, &m).unwrap(), a1 * 468696.16, "pulley bearing with a1");
            close(gearbox_life_h(&reliable, &m).unwrap(), a1 * 352777.89, "gearbox with a1");
        }
        assert_eq!(reliability_factor(&cfg), Some(1.0));
        assert_eq!(reliability_factor(&Config::parse("format_version = 1\n").unwrap()), Some(1.0));
        // Outside 90 to 99.95 % the standard has no factor: no life (the verdicts refuse it, `sim-machines/tests/life.rs`).
        reliable.set_text("ratings.bearing_reliability_percent = 89.0").unwrap();
        assert_eq!(pulley_bearing_life_h(&reliable, &m), None);
        // A table out of order is refused.
        let mut bad = cfg.clone();
        bad.set_text("ratings.belt_tooth_load_rpm = [0.0, 1000.0, 100.0]").unwrap();
        assert!(belt_tooth_utilisation(&bad, &m).is_err());
        // A life is judged from below: 6944 h against 20000 h is 2.88 times too short, and the search's badness says so.
        let v = at_least("guide-life", None, 6944.444, 20000.0);
        assert!(!v.ok && (utilisation(&v) - 2.88).abs() < 0.001 && (badness(&v) - 13055.556).abs() < 0.001);
    }

    /// L(jw) = (wc / jw) e^(-jw tau), an integrator with a dead time, by hand: |L| = 1 at wc, where the phase is
    /// -90 degrees - wc tau, so the phase margin is 90 - 360 fc tau degrees (100 Hz, 1 ms: 54); the phase reaches
    /// -180 degrees at w = pi / (2 tau) (250 Hz), where |L| = fc / 250 Hz, a gain margin of 20 log 2.5 = 7.959 dB.
    #[test]
    fn margins_of_an_integrator_with_dead_time_are_the_hand_calculated_ones() {
        let (fc, tau) = (100.0, 1e-3);
        let points: Vec<LoopPoint> = log_spaced(10.0, 1000.0, 400)
            .into_iter()
            .map(|f| {
                let w = std::f64::consts::TAU * f;
                let (gain, phase) = (fc / f, -std::f64::consts::FRAC_PI_2 - w * tau);
                let (re, im) = (gain * phase.cos(), gain * phase.sin());
                LoopPoint { frequency: f, re, im, sensitivity: 1.0 / (1.0 + re).hypot(im) }
            })
            .collect();
        let m = loop_margins(&points);
        assert!((m.crossover_hz - fc).abs() < 0.1, "crossover {} Hz", m.crossover_hz);
        assert!((m.phase_margin_deg - 54.0).abs() < 0.1, "phase margin {}", m.phase_margin_deg);
        assert!((m.gain_margin_db - 20.0 * 2.5f64.log10()).abs() < 0.01, "gain margin {}", m.gain_margin_db);
        // Without a crossover and without a crossing of the negative real axis, both margins are infinite.
        let m = loop_margins(&points[..10].iter().map(|p| LoopPoint { re: 2.0 + p.re.abs(), ..*p }).collect::<Vec<_>>());
        assert!(m.phase_margin_deg.is_infinite() && m.gain_margin_db.is_infinite() && m.crossover_hz.is_nan());
    }

    /// The same loop, stable and unstable: phase margin 90 - 360 fc tau degrees, gain margin 20 log(1 / (4 fc tau)) dB,
    /// both negative once the loop is unstable (|L| > 1 where the phase falls through -180 degrees). By hand:
    /// 100 Hz, 0.5 ms: 72 degrees, 13.979 dB; 200 Hz, 0.5 ms: 54 degrees, 7.959 dB; 300 Hz, 1 ms: -18 degrees,
    /// -1.584 dB; 400 Hz, 1 ms: -54 degrees, -4.082 dB. Points without a measurement (NaN) make the margins invalid.
    #[test]
    fn margins_are_signed_and_say_an_unstable_loop_is_unstable() {
        let response = |fc: f64, tau: f64| -> Vec<LoopPoint> {
            log_spaced(10.0, 2000.0, 1000)
                .into_iter()
                .map(|f| {
                    let (gain, phase) = (fc / f, -std::f64::consts::FRAC_PI_2 - std::f64::consts::TAU * f * tau);
                    let (re, im) = (gain * phase.cos(), gain * phase.sin());
                    LoopPoint { frequency: f, re, im, sensitivity: 1.0 / (1.0 + re).hypot(im) }
                })
                .collect()
        };
        for (fc, tau, phase, gain) in [(100.0, 0.5e-3, 72.0, 13.979), (200.0, 0.5e-3, 54.0, 7.959), (300.0, 1e-3, -18.0, -1.584), (400.0, 1e-3, -54.0, -4.082)] {
            let m = loop_margins(&response(fc, tau));
            assert!((m.phase_margin_deg - phase).abs() < 0.1 && (m.gain_margin_db - gain).abs() < 0.01 && m.valid, "{fc} Hz, {tau} s: {m:?}");
        }
        let mut points = response(100.0, 0.5e-3);
        points[500].re = f64::NAN;
        let m = loop_margins(&points);
        assert!(!m.valid && !m.stable() && m.phase_margin_deg.is_nan() && m.gain_margin_db.is_nan(), "{m:?}");
    }

    /// Points of L(jw) from its gain and phase (radians) at w.
    fn loop_points(frequencies: Vec<f64>, l: impl Fn(f64) -> (f64, f64)) -> Vec<LoopPoint> {
        frequencies
            .into_iter()
            .map(|f| {
                let (gain, phase) = l(std::f64::consts::TAU * f);
                let (re, im) = (gain * phase.cos(), gain * phase.sin());
                LoopPoint { frequency: f, re, im, sensitivity: 1.0 / (1.0 + re).hypot(im) }
            })
            .collect()
    }

    /// L = K (1 + s/a)^2 / (s (1 + s/b)^2 (1 + s/c)), a = 2 pi 300, b = 2 pi 30, c = 2 pi 3000: the phase falls
    /// through -180 degrees between b and a with |L| > 1 and rises back through it before the crossover, so L
    /// does not encircle -1. The closed loops are stable (largest real parts of their poles -977, -1355, -1459 1/s,
    /// found from the characteristic polynomial); phase margins 30.7, 41.6, 42.4 degrees at 589, 1002, 1710 Hz,
    /// and no crossing above the crossover: an infinite gain margin.
    #[test]
    fn a_conditionally_stable_loop_is_stable() {
        let (a, b, c) = (std::f64::consts::TAU * 300.0, std::f64::consts::TAU * 30.0, std::f64::consts::TAU * 3000.0);
        for (k, phase, hz) in [(3e5, 30.7, 589.0), (6.1e5, 41.6, 1002.0), (1.2e6, 42.4, 1710.0)] {
            let m = loop_margins(&loop_points(log_spaced(10.0, 2000.0, 60), |w| {
                let gain = k * (1.0 + (w / a).powi(2)) / (w * (1.0 + (w / b).powi(2)) * (1.0 + (w / c).powi(2)).sqrt());
                (gain, 2.0 * (w / a).atan() - std::f64::consts::FRAC_PI_2 - 2.0 * (w / b).atan() - (w / c).atan())
            }));
            assert!(m.stable() && m.gain_margin_db.is_infinite() && (m.phase_margin_deg - phase).abs() < 0.3 && (m.crossover_hz / hz - 1.0).abs() < 0.01, "K {k}: {m:?}");
        }
    }

    /// A type-0 lead loop, L = 3 (1 + s/w5) / ((1 + s/w50) (1 + s/w200) (1 + s/w400)), wn = 2 pi n: the phase is
    /// positive at the first point. By hand at the crossover, 1516 Hz: 89.81 - 88.11 - 82.49 - 75.22 = -156.0
    /// degrees, a phase margin of 24.0 degrees; the phase tends to -180 degrees and never crosses it.
    #[test]
    fn a_lead_with_positive_phase_at_the_first_point_keeps_its_phase_margin() {
        let w = |n: f64| std::f64::consts::TAU * n;
        let m = loop_margins(&loop_points(log_spaced(1.0, 10000.0, 200), |x| {
            let lag = |n: f64| (1.0 + (x / w(n)).powi(2)).sqrt();
            (3.0 * lag(5.0) / (lag(50.0) * lag(200.0) * lag(400.0)), (x / w(5.0)).atan() - (x / w(50.0)).atan() - (x / w(200.0)).atan() - (x / w(400.0)).atan())
        }));
        assert!(m.stable() && m.gain_margin_db.is_infinite() && (m.phase_margin_deg - 24.0).abs() < 0.2 && (m.crossover_hz / 1516.0 - 1.0).abs() < 0.01, "{m:?}");
    }
}
