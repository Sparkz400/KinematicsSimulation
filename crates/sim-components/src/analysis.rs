//! Analysis for motor sizing: peak and RMS torque per cycle, operating points against the
//! torque-speed curve, and the mechanical frequency response.

use crate::control::{Command, CommandError};
use crate::sim::Sim;
use sim_core::{
    Config, ConfigError, LOSS_PREFIX, Machine, MachineEntry, Mode, RatingCheck, Rk4, SignalInfo, TORQUE_PARTS,
    ThermalPart, advance,
};

/// The load on one motor over one pass of the cycle.
#[derive(Debug, Clone, Default)]
pub struct MotorLoad {
    /// The motor's id, whose own values (`catalog::motor_config`) its verdicts are judged with; "" for the shared ones.
    pub id: String,
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
    /// Highest |current| over the pass, A: against the drive's maximum output current. With the drive's limit
    /// simulated (effects.drive_current_limit) the current the velocity loop's torque asks for (`Drive::torque_demand`)
    /// where that is higher: a drive held at its limit delivers no more, and the check would never see the cycle beyond
    /// it ("motor-peak" judges the demand for the same reason).
    pub peak_current: f64,
    /// Temperature rise the winding settles at when the cycle repeats for long.
    pub winding_rise: f64,
    /// RMS of the current that heats the winding (Drive::thermal_current): the phase current's, an induction motor's
    /// with its rotor's copper loss.
    pub thermal_current: f64,
    /// A stepper's highest |load angle| over the pass, electrical rad (drive::Phases::load_angle): beyond pi / 2 it
    /// is past its pull-out angle, beyond pi it has lost steps. None for the other motors.
    pub load_angle: Option<f64>,
    /// With effects.iron_loss: the stator's mean iron loss over the pass, W (`DriveParams::iron_loss_at`); 0 without.
    pub iron_loss: f64,
    /// With effects.motor_thermal_nodes: the rise the stator and housing settle at, K, the winding's times the housing's
    /// share of the thermal resistance (`drive::ThermalNodes`); 0 without.
    pub housing_rise: f64,
    /// With effects.drive_overload: the highest the drive's thermal image reaches when the cycle repeats for long
    /// (`drive::Overload`, 1 at the rated current held), judged against its trip level ("drive-overload"); 0 without.
    pub drive_overload: f64,
    /// Electrical energy over the cycle: drawn from the DC link, and fed back into it while braking.
    pub energy_in: f64,
    pub energy_regenerated: f64,
    /// Highest power fed back, and the most energy fed back in one go: what the DC link capacitors or a
    /// braking resistor have to take.
    pub peak_regenerated_power: f64,
    pub largest_regeneration: f64,
    /// The cycle's electrical energy in stretches of one sign, J, in their order: positive for a braking event (one run of
    /// negative power) fed back, negative for the energy drawn between two, which empties the DC link's capacitors first.
    /// What the resistor takes above the link's room (`resistor_excess`), where the link is not simulated. For a load
    /// built by hand: a run keeps its account online instead (`dc_link_account`) and leaves this empty.
    pub regeneration_events: Vec<f64>,
    /// A run's running account of these stretches for the motor's DC link room (`LinkAccount`), where its config gives
    /// the capacitance and the braking threshold; judged instead of `regeneration_events` where the room is the same.
    pub dc_link_account: Option<LinkAccount>,
    /// With the DC link simulated: the energy the braking resistor burned (over the cycle's duration its
    /// mean power), the most power it took and the highest DC link voltage. For a drive that tripped on
    /// overvoltage, the voltage it tripped at.
    pub resistor_energy: f64,
    pub peak_resistor_power: f64,
    pub peak_dc_link_voltage: f64,
    /// With the regenerative supply (effects.regenerative_supply): the energy fed back into the mains (what arrives
    /// there), and the line module's load: the energy through it either way (drawn into the DC link, and fed back as
    /// the DC link gives it) and its highest power either way.
    pub mains_energy: f64,
    pub line_module_energy: f64,
    pub peak_line_module_power: f64,
    /// With the DC link simulated: what the drive's bleeder took out of the DC link while disabled (`Drive::disabled_step`;
    /// each drive its own share of a shared link), J.
    pub discharge_energy: f64,
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
    /// Where the machine reports the gearbox output torque: the gearing's load spectrum, the motor's |speed| in rad/s
    /// while the output torque was in each band (band i from `torque_band_floor(i)` to `torque_band_floor(i + 1)`),
    /// summed every step over the steps: what each band adds to `mean_speed`. For `gearbox_gearing_life_h`.
    pub gearbox_torque_spectrum: Vec<f64>,
    /// Where the machine reports the radial load on this motor's pulley shaft (`pulley.<motor>.radial_load`): its cubic
    /// mean weighted with the speed, the equivalent load P of the bearings' rating life (ISO 281), once per millisecond.
    pub radial_load_equivalent: Option<f64>,
    /// Where the machine reports that radial load: its highest over the pass, N, sampled as `CycleLoad::radial_load_peak`
    /// is. Judged per motor against that motor's `ratings.shaft_max_radial_load_n` ("gearbox-radial-load").
    pub radial_load_peak: Option<f64>,
    /// With effects.belt_wrap, where the machine gives its belt geometry (`Machine::belt_wraps`): the smallest angle by
    /// which the belt wrapped this motor's pulley over the pass, rad, once per millisecond. For the teeth in mesh.
    pub belt_wrap: Option<f64>,
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
    /// `copper.<motor>`, `emf.<motor>` (only with a back-EMF constant other than the torque constant), `resistor`, `mains`, `line_module`, a machine's loss signal (`power.<place>`), `windings`,
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
        let electrical = |id: &str| {
            id.starts_with("copper.")
                || id.starts_with("emf.")
                || matches!(id, "resistor" | "discharge" | "mains" | "line_module" | "windings" | "capacitors")
        };
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

/// One guide's load over the cycle (`CycleLoad::guides`).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GuideLoad {
    /// Its highest load, N.
    pub peak: f64,
    /// Its equivalent load: the cubic mean over the distance, N.
    pub equivalent: f64,
    /// The mean |speed| of its axis, m/s; `None` where the axis is not known.
    pub speed: Option<f64>,
}

#[derive(Debug, Clone, Default)]
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
    /// Every guide the machine reports, each with its own figures: `guide_life_km` and `guide_life_h` take the lowest
    /// life over them. `guide_load` and `guide_speed` above are those of the one that wears out first.
    pub guides: Vec<GuideLoad>,
    /// How far the tool was from the path its setpoint drew: the most and the RMS. And the most it was
    /// from where the setpoint was at the same moment, which also counts lagging behind along the path. Of the
    /// axes in m alone (its place); the axes in rad (its turn, an arm's orientation, a turning table) have their own
    /// figures below, in rad, so that metres and radians are never added.
    pub path_error_peak: f64,
    pub path_error_rms: f64,
    pub tracking_error_peak: f64,
    /// The most the tool was from its target in the second half of a dwell: by then it should have settled,
    /// because that is when the gripper acts. Of the axes in m.
    pub dwell_error_peak: f64,
    /// The same four of the axes in rad, rad: 0 on a machine without one.
    pub path_angle_error_peak: f64,
    pub path_angle_error_rms: f64,
    pub tracking_angle_error_peak: f64,
    pub dwell_angle_error_peak: f64,
    /// The tool's distance to the path every millisecond of the measured pass, of which `path_error_peak` is the most:
    /// [time from the pass's start in s, place in m, turn in rad], for the error views (`charts::tool_error`).
    pub path_error_series: Vec<[f64; 3]>,
    /// A drive tripped on DC link overvoltage and took all drives off: the run ended there, and the figures
    /// are those up to the trip.
    /// A drive tripped on DC link overvoltage, or a drive's monitoring tripped (effects.drive_monitoring), and took all
    /// drives off: the run ended there, and the figures are those up to the trip and the stop.
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
    /// With effects.thermal_warm_start, and windings or parts whose warmth acts back on the run: how the measured pass
    /// was started warm. `None`: from cold.
    pub warm_start: Option<WarmStart>,
    /// With `[conveyor]`: what the measured pass, one pick, did on the belt.
    pub conveyor: Option<ConveyorLoad>,
    /// While the gripper held the payload (and was not closing or opening): the tool's acceleration upwards (along z)
    /// and sideways (|a_x, a_y|), m/s^2, once per millisecond from its speed a millisecond apart, for the verdict
    /// "gripper-hold" (`gripper::demand`). Empty on a machine without tool axes x, y or z in m.
    pub held_accelerations: Vec<[f64; 2]>,
}

/// With an electronic cam (`Machine::cam`): per slave axis what its cam asks at the master's steady speed against the
/// axis limits, as rating checks "cam-speed", "cam-acceleration" and "cam-jerk" (signal: the axis's id); the motor is
/// the axis's, where there are as many motors as axes. A cam that asks more than its slave may do fails them.
fn cam_checks(machine: &dyn sim_core::Machine) -> Vec<(RatingCheck, f64)> {
    let (axes, motors) = (machine.axes(), machine.motors().len());
    let checks = ["cam-speed", "cam-acceleration", "cam-jerk"];
    crate::control::cam_peaks(machine)
        .into_iter()
        .zip(axes)
        .enumerate()
        .flat_map(|(i, (peak, axis))| {
            let limits = [axis.max_vel, axis.max_acc, axis.max_jerk];
            (0..3).map(move |k| {
                let motor = (axes.len() == motors).then_some(i);
                (
                    RatingCheck { check: checks[k], signal: axis.id.to_string(), motor, limit: limits[k], life: false },
                    peak[k],
                )
            })
        })
        .collect()
}

/// One pick from the conveyor (`[conveyor]`), the measured pass of the cycle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConveyorLoad {
    /// 60 / the pass's duration: one part per pass.
    pub picks_per_minute: f64,
    /// The most the tool was from the part along the belt while the gripper closed, m (the verdict `pick-tracking`).
    pub tracking_peak: f64,
    /// The parts that left the window unpicked from the measured pick's plan to the next's.
    pub missed: u64,
    /// Parts a minute that pass the window unpicked (the verdict `parts-missed`): those the belt brings beyond the picks a
    /// minute, which one pass cannot see when the misses come in bursts, or those missed in the pass, whichever is more.
    pub missed_per_minute: f64,
}

/// How far the rises a warm pass starts at may lie from those it settles at for the warm start to count as converged,
/// K: a tenth of a kelvin moves a winding's resistance by 0.04 % and a guide's viscosity by 0.5 %.
pub const WARM_START_TOLERANCE_K: f64 = 0.1;

/// The most warm passes of a warm start (`analyse_cycle_with`) before the last is measured, converged or not; as many
/// again of periods, where the passes it took repeat every few (`period_of`).
pub const WARM_START_PASSES: usize = 4;

/// The most passes a cycle's period may span for the warm start to find it (`period_of`).
pub const WARM_START_PERIOD_MOST: usize = 3;

/// A cycle analysis started warm (effects.thermal_warm_start): the measured pass ran with the windings (with
/// effects.winding_heating) and the heated parts (with effects.thermal_feedback) at the rises the pass before settled
/// at, by `passes` passes of cold → settled → warm → settled; `change` is the most a rise the measured pass started at
/// lies from the one it settles at, K (infinite where a rise does not settle, or the run stopped). `period`: the passes
/// the measured span covers, more than one where the cycle repeats only every so many passes (a pick that alternates
/// between two places loads the motors differently pass by pass): the rises then settle over the period, not per pass.
/// `repeat`: where the passes converged neither per pass nor over a period, how far apart (K) the rises settle of two
/// passes started at the same rises, the measured one and one more run after it (`None` where not measured). Beyond the
/// tolerance the cycle's motion does not repeat itself: what one pass leaves moving (a load still swinging) carries into
/// the next and loads the motors differently, which no start temperature can settle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WarmStart {
    pub passes: usize,
    pub change: f64,
    pub period: usize,
    pub repeat: Option<f64>,
}

impl WarmStart {
    pub fn converged(&self) -> bool {
        self.change <= WARM_START_TOLERANCE_K
    }
}

/// The rises that act back on the run: per winding with effects.winding_heating (its resistance), per heated part
/// with effects.thermal_feedback (friction and losses).
fn fed_back_rises(sim: &Sim) -> Vec<f64> {
    let windings = if sim.control.drive_params().heating { &sim.control.drives[..] } else { &[] };
    let parts = if sim.thermal_feedback { &sim.heated[..] } else { &[] };
    windings.iter().map(|d| d.temperature_rise).chain(parts.iter().map(|h| h.rise)).collect()
}

/// The same rises where the cycle of `load` settles them (`MotorLoad::winding_rise`, `CycleLoad::part_rises`).
fn settled_rises(sim: &Sim, load: &CycleLoad) -> Vec<f64> {
    let windings = if sim.control.drive_params().heating { &load.motors[..] } else { &[] };
    let parts = if sim.thermal_feedback { &load.part_rises[..] } else { &[] };
    windings.iter().map(|m| m.winding_rise).chain(parts.iter().map(|(_, rise)| *rise)).collect()
}

/// Puts the windings and parts at `rises` (in the order of `fed_back_rises`).
fn set_rises(sim: &mut Sim, rises: &[f64]) {
    let mut rises = rises.iter();
    if sim.control.drive_params().heating {
        let shares: Vec<f64> = (0..sim.control.drives.len())
            .map(|k| sim.control.drive_params_of(k).thermal_nodes.map_or(1.0, |n| n.winding_share))
            .collect();
        for ((drive, rise), share) in sim.control.drives.iter_mut().zip(&mut rises).zip(shares) {
            drive.temperature_rise = *rise;
            // With effects.motor_thermal_nodes the housing where that rise settles it (0 without).
            drive.housing_rise = rise * (1.0 - share);
        }
    }
    if sim.thermal_feedback {
        for (part, rise) in sim.heated.iter_mut().zip(rises) {
            part.rise = *rise;
        }
    }
    sim.feed_temperatures_back();
}

/// The warm start's next move after a measured span (one period of `period` passes) that began at `started_at` and
/// settles where `load` says: `None` to measure another span from the next rises (already set), or the result.
/// `before` is the span before's start and settled rises, which this one replaces; `settled` the settled rises of every
/// span so far of one pass, from which, where the passes do not converge, `period_of` finds a period of a few passes:
/// the spans then cover it, from the mean of its passes' settled rises, again up to `WARM_START_PASSES` of them.
///
/// The next start is a secant step per rise: s(r), the rise a pass settles at for the rise it starts at, falls with r
/// where the warmth thins a lubricant (a warmer guide loses less), so plain substitution r = s oscillates and converges
/// slowly (about -0.4 on the EXCT-30's guide: still 0.3 K off after four passes). With the slope m of the last two passes the
/// fixed point lies at r + (s - r) / (1 - m); m is clamped to [-20, 0.5], and taken as 0 (substitution) on the first
/// warm pass and where the start did not move. A winding's settled rise depends on its current alone: m = 0.
///
/// Where the spans converge neither per pass nor over a period, and `repeat` is given (a cycle that runs on from pass to
/// pass), one more span is measured from the rises the last one started at (`repeat` holds the last one's settled rises
/// and result meanwhile): the result is the last one's, with `WarmStart::repeat` how far apart the two settle, the
/// temperatures alike, what differs being the motion. The caller keeps the last span's load as the measured one.
#[allow(clippy::too_many_arguments)]
fn warm_pass(
    sim: &mut Sim,
    started_at: &[f64],
    load: &CycleLoad,
    passes: usize,
    before: &mut Option<(Vec<f64>, Vec<f64>)>,
    period: &mut usize,
    history: &mut Vec<Vec<f64>>,
    mut repeat: Option<&mut Option<(Vec<f64>, WarmStart)>>,
) -> Option<WarmStart> {
    let settled = settled_rises(sim, load);
    if let Some((first, done)) = repeat.as_mut().and_then(|r| r.take()) {
        // This span started where the one before did: how far apart the two settle.
        let apart = settled.iter().zip(&first).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max);
        return Some(WarmStart { repeat: Some(apart), ..done });
    }
    let change = settled
        .iter()
        .zip(started_at)
        .map(|(s, r)| if s.is_finite() { (s - r).abs() } else { f64::INFINITY })
        .fold(0.0, f64::max);
    let done = WarmStart { passes, change, period: *period, repeat: None };
    // A rise that does not settle (a winding loaded beyond what its copper can shed) cannot be started at.
    if done.converged() || change.is_infinite() {
        return Some(done);
    }
    if *period == 1 {
        history.push(settled.clone());
    }
    if passes == WARM_START_PASSES {
        let found = (*period == 1).then(|| period_of(history)).flatten();
        let Some(p) = found else {
            let Some(r) = repeat else { return Some(done) };
            // No period: one more span from the same rises tells whether the passes repeat at all.
            *r = Some((settled, done));
            set_rises(sim, started_at);
            return None;
        };
        // The next span covers the period, from the mean of its passes' settled rises.
        let last = &history[history.len() - p..];
        let mean: Vec<f64> = (0..settled.len()).map(|i| last.iter().map(|s| s[i]).sum::<f64>() / p as f64).collect();
        (*period, *before) = (p, None);
        set_rises(sim, &mean);
        return None;
    }
    let next: Vec<f64> = (0..settled.len())
        .map(|i| {
            let (r, s) = (started_at[i], settled[i]);
            let slope = before
                .as_ref()
                .filter(|(r0, _)| (r - r0[i]).abs() > 1e-9)
                .map_or(0.0, |(r0, s0)| ((s - s0[i]) / (r - r0[i])).clamp(-20.0, 0.5));
            (r + (s - r) / (1.0 - slope)).max(0.0)
        })
        .collect();
    *before = Some((started_at.to_vec(), settled));
    set_rises(sim, &next);
    None
}

/// The period, in passes, of a warm start whose passes do not converge: from the settled rises of its passes in order
/// (the first from cold), the fewest passes p (2 to `WARM_START_PERIOD_MOST`) such that the last pass settles within a
/// quarter as far from the one p before as from the one before it, the rises it starts at being about the same. `None`
/// where none does: the passes drift or alternate as a convergence would, with no period.
fn period_of(settled: &[Vec<f64>]) -> Option<usize> {
    let apart = |a: &[f64], b: &[f64]| a.iter().zip(b).map(|(x, y)| (x - y).abs()).fold(0.0, f64::max);
    let (k, warm) = (settled.len().checked_sub(1)?, 1);
    let next = apart(&settled[k], &settled[k.checked_sub(1)?]);
    (2..=WARM_START_PERIOD_MOST).find(|&p| k >= warm + p && apart(&settled[k], &settled[k - p]) < 0.25 * next)
}

/// The highest a drive's thermal image (`drive::Overload`) reaches when a pass repeats for long. The pass ran from the
/// nodes [slow, chips] `start` and ended at `end` (time, nodes), with `samples` (time, nodes) between. Each node is linear
/// in its start: one started at x0 lies (x0 - start) exp(-t / tau) above the pass's at every t. The start a repeating
/// pass returns to is x0 = f / (1 - exp(-T / tau)), where f = end - start exp(-T / tau) is what the pass itself added.
/// The image is the nodes' sum.
///
/// shortcut: where the image tripped in the pass the drive lowered its current and the image is no longer linear in its
/// start; the peak is then that of the pass as it ran, corrected as if it were. It is at or above the trip anyway.
pub fn repeating_peak(
    o: &crate::drive::Overload,
    start: [f64; 2],
    samples: &[(f64, [f64; 2])],
    (t_end, end): (f64, [f64; 2]),
) -> f64 {
    let decay = |node: usize, t: f64| (-t / o.node_time_constant(node)).exp();
    let x0: [f64; 2] = std::array::from_fn(|n| {
        let added = end[n] - start[n] * decay(n, t_end);
        if t_end > 0.0 { added / (1.0 - decay(n, t_end)) } else { start[n] }
    });
    let at = |t: f64, x: [f64; 2]| (0..2).map(|n| x[n] + (x0[n] - start[n]) * decay(n, t)).sum::<f64>();
    samples.iter().chain([(t_end, end)].iter()).map(|&(t, x)| at(t, x)).fold(x0[0] + x0[1], f64::max)
}

/// What a run adds up step by step, for a cycle or a trajectory, or for any run stepped from outside (a controller of
/// the user's own, `kinematics_sim.Sim`): `step` after each `Sim::step`, then `finish` gives the load the verdicts judge.
/// The path, tracking and dwell errors are measured against the controller's plan.
#[derive(Clone)]
pub struct Tally {
    loads: Vec<MotorLoad>,
    pos: Vec<f64>,
    speed: Vec<f64>,
    steps: u64,
    /// Steps per millisecond: belt tensions, operating points and the path are sampled that often.
    every: u64,
    /// Per motor with effects.drive_overload: its drive's thermal image when the pass began, and (time, image) once per
    /// millisecond, for `repeating_peak`.
    overload: Vec<([f64; 2], Vec<(f64, [f64; 2])>)>,
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
    /// The checks on `plan.<signal>` (a limit the plan keeps: an arm's flange spin), their places among
    /// `Machine::rating_checks` (and so in `Machine::rigid_checks`), and the highest |value| on the setpoint's rigid
    /// motion, once per sample.
    plan_checks: Vec<(RatingCheck, usize, f64)>,
    sample: Vec<f64>,
    /// Every signal looked at in `sample` is one of `Sim::sample_base`: the effect views need not be sampled.
    base_only: bool,
    belt_tension: Option<(f64, f64)>,
    radial_peak: f64,
    /// Per guide: highest load and the sum of the load cubed times its axis's |speed| (rolling damage goes with distance;
    /// time where the axis is unknown), one per sample; and the sum of that weight.
    guide_loads: Vec<(f64, f64)>,
    guide_weights: Vec<f64>,
    /// Per guide the place of its axis among the machine's axes, and the sum of that axis's |speed|, one per sample.
    guide_axes: Vec<Option<usize>>,
    guide_speeds: Vec<f64>,
    /// Per motor the signal of its pulley shaft's radial load, and the sums of load^3 |speed| and |speed| per sample.
    pulley_radial: Vec<Option<usize>>,
    radial_sums: Vec<(f64, f64)>,
    /// Per motor the highest radial load on its pulley shaft, sampled as `radial_peak`.
    radial_peaks: Vec<f64>,
    /// Per motor with a gearbox output torque: the sum of |torque|^(10/3) |speed|, and its peak per speed band.
    gear_life: Vec<f64>,
    gear_bands: Vec<Vec<f64>>,
    /// Per motor with a gearbox output torque: the sum of |speed| per torque band (`MotorLoad::gearbox_torque_spectrum`).
    gear_spectrum: Vec<Vec<f64>>,
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
    /// What each servo winding took beyond copper, shaft and stored energy where its back-EMF constant is not its torque
    /// constant (`Drive::emf_loss`).
    emf: Vec<f64>,
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
    /// Energy fed back (positive) since each motor last drew power, or drawn (negative) since it last fed back.
    braking: Vec<f64>,
    /// `CycleLoad::braking_power_peak`.
    braking_peak: f64,
    /// The setpoints of the last half second: the piece of the path the tool can be near.
    path: std::collections::VecDeque<Vec<f64>>,
    /// Of the axes in m and of those in rad (`groups`): peak, sum of squares and count; tracking; dwell.
    path_error: [(f64, f64, u64); 2],
    /// [time, place, turn] per sample (`CycleLoad::path_error_series`), the time as the simulation counts it.
    path_series: Vec<[f64; 3]>,
    tracking_error: [f64; 2],
    dwell_error: [f64; 2],
    /// The tool's axes in m and those in rad, by index.
    groups: [Vec<usize>; 2],
    /// With `[conveyor]`: the most the tool was from the gripped part along the belt, and the parts missed before.
    pick_tracking: f64,
    missed_before: u64,
    /// The tool axes x, y, z in m (by index; `None` where the machine lacks one), the tool's speed along them at the
    /// last sample, and `CycleLoad::held_accelerations`.
    cartesian: [Option<usize>; 3],
    last_speed: Option<[f64; 3]>,
    held: Vec<[f64; 2]>,
}

/// Distance from `p` to the piece of line between `a` and `b`, in the axes `axes` alone.
fn distance_to_piece(p: &[f64], a: &[f64], b: &[f64], axes: &[usize]) -> f64 {
    // Without closures behind references: the path error asks this 500 times per millisecond.
    let (ab, ap) = (|i: usize| b[i] - a[i], |i: usize| p[i] - a[i]);
    let length = axes.iter().map(|&i| ab(i) * ab(i)).sum::<f64>();
    let along =
        if length > 0.0 { (axes.iter().map(|&i| ap(i) * ab(i)).sum::<f64>() / length).clamp(0.0, 1.0) } else { 0.0 };
    // (+ 0: an empty sum is -0.)
    axes.iter().map(|&i| (ap(i) - along * ab(i)).powi(2)).sum::<f64>().sqrt() + 0.0
}

impl Tally {
    pub fn new(sim: &Sim, views: bool) -> Self {
        let motors = sim.machine.motors().len();
        let infos = sim.signal_infos();
        let named = |start: &str, end: &str| -> Vec<usize> {
            infos
                .iter()
                .enumerate()
                .filter(|(_, s)| s.id.starts_with(start) && s.id.ends_with(end))
                .map(|(i, _)| i)
                .collect()
        };
        let guides = named("axis.", ".guide_load");
        let own = sim.machine.signal_infos();
        let gear = |motor: &sim_core::MotorInfo| {
            Some((own.iter().position(|s| s.id == format!("gearbox.{}.output_torque", motor.id))?, 0.0, 0.0))
        };
        let parts = |motor: &sim_core::MotorInfo| -> Option<[usize; 4]> {
            let found: Vec<usize> = TORQUE_PARTS
                .iter()
                .filter_map(|part| own.iter().position(|s| s.id == format!("motor.{}.torque_{part}", motor.id)))
                .collect();
            found.try_into().ok()
        };
        // Without the views, only what `base_signals` gives is looked at.
        let own_named = |prefix: &str| {
            own.iter()
                .enumerate()
                .filter(|(_, s)| views && s.id.starts_with(prefix))
                .map(|(i, _)| i)
                .collect::<Vec<_>>()
        };
        let (losses, energies) = (own_named(LOSS_PREFIX), own_named("energy."));
        let mut machine_signals = Vec::new();
        if views {
            sim.machine.signals(&sim.state, &sim.machine_torque(), &mut machine_signals);
        } else {
            sim.machine.base_signals(&sim.state, &sim.machine_torque(), &mut machine_signals);
        }
        let (windings, capacitors) = stored_electrically(sim);
        let (tensions, radial, slack, jumps, detuning) = (
            named("belt.", ".tension"),
            named("pulley.", ".radial_load"),
            infos.iter().position(|s| s.id == "belt.slack_time"),
            named("pulley.", ".tooth_jumps"),
            infos.iter().position(|s| s.id == "belt.span_detuning"),
        );
        let mut base = Vec::new();
        sim.sample_base(&mut base);
        let condition = infos.iter().position(|s| s.id == "kinematics.condition");
        let checks: Vec<(RatingCheck, usize, f64, f64)> = sim
            .machine
            .rating_checks()
            .into_iter()
            .filter_map(|c| Some((infos.iter().position(|s| s.id == c.signal)?, c)))
            .map(|(i, c)| (c, i, 0.0, 0.0))
            .collect();
        let check_places: Vec<usize> = checks.iter().map(|c| c.1).collect();
        let plan_checks = sim
            .machine
            .rating_checks()
            .into_iter()
            .enumerate()
            .filter(|(_, c)| c.signal.starts_with("plan.") && !c.life)
            .map(|(i, c)| (c, i, 0.0))
            .collect();
        let base_only = tensions
            .iter()
            .chain(&radial)
            .chain(&guides)
            .chain(&jumps)
            .chain(&slack)
            .chain(&detuning)
            .chain(&condition)
            .chain(&check_places)
            .all(|&i| i < base.len());
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
            emf: vec![0.0; motors],
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
            plan_checks,
            radial_peak: 0.0,
            guide_loads: vec![(0.0, 0.0); guides.len()],
            guide_weights: vec![0.0; guides.len()],
            guide_axes: guides
                .iter()
                .map(|&i| sim.machine.axes().iter().position(|a| infos[i].id == format!("axis.{}.guide_load", a.id)))
                .collect(),
            guide_speeds: vec![0.0; guides.len()],
            pulley_radial: sim
                .machine
                .motors()
                .iter()
                .map(|m| infos.iter().position(|s| s.id == format!("pulley.{}.radial_load", m.id)))
                .collect(),
            radial_sums: vec![(0.0, 0.0); motors],
            radial_peaks: vec![0.0; motors],
            gear_life: vec![0.0; motors],
            gear_bands: vec![Vec::new(); motors],
            gear_spectrum: vec![Vec::new(); motors],
            guides,
            slack_start: 0.0,
            cubic: vec![(0.0, 0.0); motors],
            loads: sim
                .machine
                .motors()
                .iter()
                .enumerate()
                .map(|(k, m)| MotorLoad {
                    id: m.id.to_string(),
                    dc_link_account: sim.control.dc_link_room(k).map(LinkAccount::new),
                    ..MotorLoad::default()
                })
                .collect(),
            pos: {
                let (mut pos, mut speed) = (vec![0.0; motors], vec![0.0; motors]);
                sim.machine.motor_state(&sim.state, &mut pos, &mut speed);
                pos
            },
            speed: vec![0.0; motors],
            steps: 0,
            every: (1e-3 / sim.dt).round().max(1.0) as u64,
            overload: sim.control.drives.iter().map(|d| (d.overload_nodes(), Vec::new())).collect(),
            tensions,
            sample: Vec::new(),
            base_only,
            belt_tension: None,
            braking: vec![0.0; motors],
            braking_peak: 0.0,
            path: std::collections::VecDeque::new(),
            path_error: [(0.0, 0.0, 0); 2],
            path_series: Vec::new(),
            tracking_error: [0.0; 2],
            dwell_error: [0.0; 2],
            groups: [false, true].map(|turn| {
                sim.machine
                    .axes()
                    .iter()
                    .enumerate()
                    .filter(|(_, a)| (a.unit == "rad") == turn)
                    .map(|(i, _)| i)
                    .collect()
            }),
            pick_tracking: 0.0,
            missed_before: sim.control.conveyor().map_or(0, |(c, _)| c.missed),
            cartesian: ["x", "y", "z"].map(|id| sim.machine.axes().iter().position(|a| a.id == id && a.unit == "m")),
            last_speed: None,
            held: Vec::new(),
        }
    }

    fn sample_now(&mut self, sim: &Sim) {
        if self.base_only { sim.sample_base(&mut self.sample) } else { sim.sample(&mut self.sample) }
    }

    /// Adds the step the simulation has just made.
    pub fn step(&mut self, sim: &Sim) {
        sim.machine.motor_state(&sim.state, &mut self.pos, &mut self.speed);
        if let Some((conveyor, Some(pass))) = sim.control.conveyor()
            && let Some(part) = conveyor.gripping(pass, sim.time())
        {
            let n = part.len();
            let (mut tool, mut vel) = (vec![0.0; n], vec![0.0; n]);
            sim.machine.tool_state(&sim.state, &mut tool, &mut vel);
            let along: f64 = tool.iter().zip(&part).zip(&conveyor.direction).map(|((t, p), d)| (t - p) * d).sum();
            self.pick_tracking = self.pick_tracking.max(along.abs());
        }
        let sampled = self.steps.is_multiple_of(self.every);
        if sampled {
            self.sample_now(sim);
            for &i in &self.tensions {
                let (low, high) = self.belt_tension.unwrap_or((self.sample[i], self.sample[i]));
                self.belt_tension = Some((low.min(self.sample[i]), high.max(self.sample[i])));
            }
            for &i in &self.radial {
                self.radial_peak = self.radial_peak.max(self.sample[i]);
            }
            for (peak, signal) in self.radial_peaks.iter_mut().zip(&self.pulley_radial) {
                if let Some(i) = *signal {
                    *peak = peak.max(self.sample[i]);
                }
            }
            if let Some(i) = self.condition {
                self.condition_peak = self.condition_peak.max(self.sample[i]);
            }
            let axes = sim.machine.axes().len();
            let (mut tool, mut rate) = (vec![0.0; axes], vec![0.0; axes]);
            sim.machine.tool_state(&sim.state, &mut tool, &mut rate);
            // A motor that drives no belt has no wrap (NaN).
            for (load, wrap) in self.loads.iter_mut().zip(sim.machine.belt_wraps(&tool)).filter(|(_, w)| w.is_finite())
            {
                load.belt_wrap = Some(load.belt_wrap.map_or(wrap, |least| least.min(wrap)));
            }
            for (_, i, peak, sum) in &mut self.checks {
                (*peak, *sum) = (peak.max(self.sample[*i].abs()), *sum + self.sample[*i]);
            }
            // A limit of the plan on the setpoint as the rigid model moves along it: the simulated machine's own
            // signal counts what the plan cannot hold (the gears' ringing, the servo's steps).
            if !self.plan_checks.is_empty() {
                let mut rigid = Vec::new();
                let c = &sim.control;
                sim.machine.rigid_checks(&c.set_pos, &c.set_vel, &c.set_acc, &mut rigid);
                for (_, i, peak) in &mut self.plan_checks {
                    if let Some(v) = rigid.get(*i).filter(|v| v.is_finite()) {
                        *peak = peak.max(v.abs());
                    }
                }
            }
            for (k, (&i, (peak, cubed))) in self.guides.iter().zip(&mut self.guide_loads).enumerate() {
                let weight = self.guide_axes[k].map_or(1.0, |a| rate[a].abs());
                (*peak, *cubed) = (peak.max(self.sample[i]), *cubed + self.sample[i].powi(3) * weight);
                self.guide_weights[k] += weight;
            }
            if let (0, Some(i)) = (self.steps, self.slack) {
                self.slack_start = self.sample[i];
            }
            if self.steps == 0 {
                self.jumps_start = self.jumps.iter().map(|&i| self.sample[i]).sum();
            }
            // Steady speed: no axis accelerates by its setpoint, and one moves. A sweep through a resonance while
            // speeding up is passed in moments; a cruise on it is not.
            let steady = sim.control.set_acc.iter().all(|a| a.abs() <= 1e-9)
                && sim.control.set_vel.iter().any(|v| v.abs() > 1e-9);
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
            if self.cartesian.iter().any(Option::is_some) {
                let speed = self.cartesian.map(|i| i.map_or(0.0, |i| rate[i]));
                let held = sim.control.payload_attached && !sim.control.gripper_acting();
                if let (true, Some(last)) = (held, self.last_speed) {
                    let a: [f64; 3] = std::array::from_fn(|k| (speed[k] - last[k]) / (self.every as f64 * sim.dt));
                    self.held.push([a[2], a[0].hypot(a[1])]);
                }
                self.last_speed = Some(speed);
            }
            for (k, (signal, (cubed, weight))) in self.pulley_radial.iter().zip(&mut self.radial_sums).enumerate() {
                if let Some(i) = *signal {
                    (*cubed, *weight) =
                        (*cubed + self.sample[i].powi(3) * self.speed[k].abs(), *weight + self.speed[k].abs());
                }
            }
            // The oldest setpoint's place is reused for the newest: the plan at this moment, not the setpoint held from
            // the last profile cycle, which would count a tool where the plan is now as off it by up to a cycle's way.
            let mut newest =
                if self.path.len() >= 500 { self.path.pop_front().unwrap_or_default() } else { Vec::new() };
            newest.resize(axes, 0.0);
            sim.control.plan_now(sim.time(), &mut newest);
            self.path.push_back(newest);
            let now = self.path.back().unwrap();
            // The nearest piece, newest first. A piece is at least as far as the tool is beyond its extent along any one
            // axis; one that is further than the nearest so far by more than a picometre (far above rounding) cannot be
            // the nearest, and is skipped. The smallest distance is the same as over all pieces. The place (axes in m)
            // and the turn (axes in rad) each by itself: each its own nearest piece.
            let dwelling = sim.control.dwell_progress().is_some_and(|done| done >= 0.5);
            let mut series = [sim.time(), 0.0, 0.0];
            for (g, group) in self.groups.iter().enumerate() {
                let mut off_path = distance_to_piece(&tool, now, now, group);
                for (b, a) in self.path.iter().rev().zip(self.path.iter().rev().skip(1)) {
                    let beyond = group
                        .iter()
                        .map(|&i| (tool[i] - a[i]).abs() - (b[i] - a[i]).abs())
                        .fold(f64::NEG_INFINITY, f64::max);
                    if !(beyond > off_path + 1e-12) {
                        off_path = off_path.min(distance_to_piece(&tool, a, b, group));
                    }
                }
                let (peak, squares, count) = self.path_error[g];
                self.path_error[g] = (peak.max(off_path), squares + off_path * off_path, count + 1);
                series[g + 1] = off_path;
                self.tracking_error[g] = self.tracking_error[g].max(distance_to_piece(&tool, now, now, group));
                if dwelling {
                    let target = sim.control.planned_pos();
                    self.dwell_error[g] = self.dwell_error[g].max(distance_to_piece(&tool, target, target, group));
                }
            }
            self.path_series.push(series);
        }
        if self.views && (self.parts.iter().any(Option::is_some) || !self.losses.is_empty()) {
            self.machine_signals.clear();
            sim.machine.signals(&sim.state, &sim.machine_torque(), &mut self.machine_signals);
        } else if self.gear.iter().any(Option::is_some) {
            self.machine_signals.clear();
            sim.machine.base_signals(&sim.state, &sim.machine_torque(), &mut self.machine_signals);
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
            for ((copper, emf), d) in self.copper.iter_mut().zip(&mut self.emf).zip(drives) {
                *copper += d.copper_loss * sim.dt;
                *emf += d.emf_loss * sim.dt;
            }
            // A shared DC link has one resistor, rectifier and line module, which every drive reports as its own.
            let shared = sim.control.drive_params().shared_dc_link;
            let link = |power: fn(&crate::drive::Drive) -> f64| -> f64 {
                let total: f64 =
                    if shared { drives.first().map_or(0.0, power) } else { drives.iter().map(power).sum() };
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
        for (k, (i, peak, cubed)) in self.gear.iter_mut().enumerate().filter_map(|(k, gear)| Some((k, gear.as_mut()?)))
        {
            let (torque, speed) = (self.machine_signals[*i].abs(), self.speed[k].abs());
            (*peak, *cubed) = (peak.max(torque), *cubed + torque.powi(3) * speed);
            self.gear_life[k] += torque.powf(10.0 / 3.0) * speed;
            // Capped, so that a run gone unstable cannot ask for a band per rad/s up to infinity.
            let (bands, band) = (&mut self.gear_bands[k], ((speed / SPEED_BAND) as usize).min(4000));
            if bands.len() <= band {
                bands.resize(band + 1, 0.0);
            }
            bands[band] = bands[band].max(torque);
            if torque >= TORQUE_FLOOR && speed > 0.0 {
                let (spectrum, band) = (
                    &mut self.gear_spectrum[k],
                    (((torque / TORQUE_FLOOR).ln() / TORQUE_BAND.ln()) as usize).min(4000),
                );
                if spectrum.len() <= band {
                    spectrum.resize(band + 1, 0.0);
                }
                spectrum[band] += speed;
            }
        }
        for (k, load) in self.loads.iter_mut().enumerate() {
            let torque = sim.control.drives[k].torque;
            load.peak_torque = load.peak_torque.max(torque.abs());
            // The RMS phase current: with field weakening the d current flows too.
            let (current, power) = (sim.control.drives[k].phase_current(), sim.control.drives[k].power);
            load.rms_current += current * current;
            let heating = sim.control.drives[k].thermal_current(sim.control.drive_params_of(k));
            load.thermal_current += heating * heating;
            if let crate::drive::Motor::Stepper(_) = sim.control.drive_params_of(k).motor {
                let angle = sim.control.drives[k].phases.load_angle.abs();
                load.load_angle = Some(load.load_angle.unwrap_or(0.0).max(angle));
            }
            load.peak_current = load.peak_current.max(current.abs());
            // Each motor's own drive and motor (`[drive.<id>]`, `[motor.<id>]`, `[parts.<id>]`).
            let own = sim.control.drive_params_of(k);
            if own.drive_max_current.is_finite() {
                load.peak_current = load.peak_current.max(own.current_for(sim.control.drives[k].torque_demand).abs());
            }
            // The running stretch: energy fed back (positive) or drawn (negative), pushed as one event when the sign turns.
            let fed = -power * sim.dt;
            if self.braking[k] * fed < 0.0 {
                if let Some(account) = &mut load.dc_link_account {
                    account.push(self.braking[k]);
                }
                self.braking[k] = 0.0;
            }
            self.braking[k] += fed;
            if power >= 0.0 {
                load.energy_in += power * sim.dt;
            } else {
                load.energy_regenerated -= power * sim.dt;
                load.peak_regenerated_power = load.peak_regenerated_power.max(-power);
                load.largest_regeneration = load.largest_regeneration.max(self.braking[k]);
            }
            load.rms_torque += torque * torque;
            load.iron_loss += sim.control.drives[k].iron_loss;
            self.cubic[k] =
                (self.cubic[k].0 + torque.abs().powi(3) * self.speed[k].abs(), self.cubic[k].1 + self.speed[k].abs());
            let drive = &sim.control.drives[k];
            load.resistor_energy += drive.resistor_power * sim.dt;
            load.discharge_energy += drive.discharge_power * sim.dt;
            load.peak_resistor_power = load.peak_resistor_power.max(drive.resistor_power);
            load.peak_dc_link_voltage = load.peak_dc_link_voltage.max(drive.dc_voltage);
            // The line module's load either way, on the DC link's side: drawn into it (the line gives 1 / efficiency of
            // that, `rectifier_power`), or fed back as the DC link gives it (before its losses).
            let efficiency = sim.control.drive_params_of(k).line_efficiency;
            let through = efficiency * drive.rectifier_power + drive.mains_power / efficiency;
            load.mains_energy += drive.mains_power * sim.dt;
            load.line_module_energy += through * sim.dt;
            load.peak_line_module_power = load.peak_line_module_power.max(through);
            load.peak_speed = load.peak_speed.max(self.speed[k].abs());
            // What the velocity loop asks for, not what the drive delivers: a drive on its torque limit
            // delivers the curve and no more, so the delivered torque would never show a cycle beyond it.
            let (low, high) = sim.control.motor_torque_curve(k, self.speed[k]);
            let demand = drive.torque_demand;
            let available = if demand >= 0.0 { high } else { -low };
            load.utilisation = load.utilisation.max(demand.abs() / available.max(1e-9));
            if sampled {
                load.points.push((self.speed[k], torque));
                if sim.control.drive_params_of(k).overload.is_some() {
                    self.overload[k].1.push(((self.steps + 1) as f64 * sim.dt, drive.overload_nodes()));
                }
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

    pub fn finish(mut self, sim: &Sim, duration: f64) -> CycleLoad {
        let steps = self.steps.max(1) as f64;
        for (k, load) in self.loads.iter_mut().enumerate() {
            load.rms_torque = (load.rms_torque / steps).sqrt();
            load.rms_current = (load.rms_current / steps).sqrt();
            if let (Some(account), true) = (&mut load.dc_link_account, self.braking[k] != 0.0) {
                account.push(self.braking[k]);
            }
            load.thermal_current = (load.thermal_current / steps).sqrt();
            load.iron_loss /= steps;
            load.cubic_mean_torque =
                if self.cubic[k].1 > 0.0 { (self.cubic[k].0 / self.cubic[k].1).cbrt() } else { 0.0 };
            load.gearbox_torque = self.gear[k].map(|(_, peak, cubed)| {
                (peak, if self.cubic[k].1 > 0.0 { (cubed / self.cubic[k].1).cbrt() } else { 0.0 })
            });
            load.mean_speed = self.cubic[k].1 / steps;
            load.gearbox_life_torque = self.gear[k]
                .map(|_| if self.cubic[k].1 > 0.0 { (self.gear_life[k] / self.cubic[k].1).powf(0.3) } else { 0.0 });
            load.gearbox_torque_by_speed = std::mem::take(&mut self.gear_bands[k]);
            load.gearbox_torque_spectrum = self.gear_spectrum[k].iter().map(|speed| speed / steps).collect();
            let (cubed, weight) = self.radial_sums[k];
            load.radial_load_equivalent =
                self.pulley_radial[k].map(|_| if weight > 0.0 { (cubed / weight).cbrt() } else { 0.0 });
            load.radial_load_peak = self.pulley_radial[k].map(|_| self.radial_peaks[k]);
            let p = sim.control.drive_params_of(k);
            load.winding_rise = p.settled_rise(load.thermal_current, load.iron_loss);
            // The housing carries all of the loss through its share of the resistance: the winding's rise less what the
            // winding's own share adds to it of the copper loss, which is the winding's rise less the iron's part.
            if let (true, Some(n)) = (p.heating, &p.thermal_nodes) {
                let (s, iron) = (n.winding_share, p.iron_heating(load.iron_loss));
                load.housing_rise = load.winding_rise - s * (load.winding_rise - (1.0 - s) * iron);
            }
            if let Some(o) = &p.overload {
                let (start, samples) = &self.overload[k];
                let end = (self.steps as f64 * sim.dt, sim.control.drives[k].overload_nodes());
                load.drive_overload = repeating_peak(o, *start, samples, end);
            }
            load.thermal_utilisation = load.rms_torque / sim.control.drive_params_of(k).rated_torque;
            load.torque_parts =
                self.parts[k].map(|_| self.part_sums[k].map(|(peak, squares)| (peak, (squares / steps).sqrt())));
            // A tripped drive's DC link stays where it tripped, also when that was before the measured pass.
            load.peak_dc_link_voltage = load.peak_dc_link_voltage.max(sim.control.drives[k].dc_voltage);
        }
        self.sample_now(sim);
        let budget = self.budget(sim);
        let part_rises = sim
            .heated
            .iter()
            .zip(&self.heat)
            .map(|(h, heat)| (h.part.clone(), h.resistance * heat / duration))
            .collect();
        let samples = self.path_error[0].2.max(1) as f64;
        // Per guide: its equivalent load (cubic mean over distance) and mean speed. The life estimate is about the guide
        // that wears out first: life in hours goes with 1/(P^3 v); the peak is the highest of all guides.
        let equivalent = |g: usize| equivalent_load(self.guide_loads[g].1, self.guide_weights[g]);
        let speed = |g: usize| self.guide_axes[g].map(|_| self.guide_speeds[g] / samples);
        let wear = |g: usize| equivalent(g).powi(3) * speed(g).unwrap_or(1.0);
        let guide = (0..self.guide_loads.len()).max_by(|&a, &b| wear(a).total_cmp(&wear(b)));
        let peak = self.guide_loads.iter().map(|g| g.0).reduce(f64::max);
        let guide_load = guide.zip(peak).map(|(g, peak)| (peak, equivalent(g)));
        let guide_speed = guide.and_then(speed);
        let guides = (0..self.guide_loads.len())
            .map(|g| GuideLoad { peak: self.guide_loads[g].0, equivalent: equivalent(g), speed: speed(g) })
            .collect();
        CycleLoad {
            duration,
            belt_tension: self.belt_tension,
            belt_slack_time: self.slack.map(|i| self.sample[i] - self.slack_start),
            belt_tooth_jumps: (!self.jumps.is_empty())
                .then(|| self.jumps.iter().map(|&i| self.sample[i]).sum::<f64>() - self.jumps_start),
            belt_span_detuning: self.detuning.map(|_| self.lowest_detuning),
            radial_load_peak: (!self.radial.is_empty()).then_some(self.radial_peak),
            condition_peak: self.condition.map(|_| self.condition_peak),
            guide_load,
            guide_speed,
            guides,
            path_error_peak: self.path_error[0].0,
            path_error_rms: (self.path_error[0].1 / self.path_error[0].2.max(1) as f64).sqrt(),
            tracking_error_peak: self.tracking_error[0],
            dwell_error_peak: self.dwell_error[0],
            path_angle_error_peak: self.path_error[1].0,
            path_angle_error_rms: (self.path_error[1].1 / self.path_error[1].2.max(1) as f64).sqrt(),
            tracking_angle_error_peak: self.tracking_error[1],
            dwell_angle_error_peak: self.dwell_error[1],
            path_error_series: {
                let start = self.path_series.first().map_or(0.0, |s| s[0]);
                self.path_series.iter().map(|&[t, place, turn]| [t - start, place, turn]).collect()
            },
            tripped: sim.control.drives.iter().any(|d| d.tripped) || sim.control.fault.is_some(),
            motors: self.loads,
            budget,
            part_rises,
            checks: std::mem::take(&mut self.checks)
                .into_iter()
                .map(|(c, _, peak, sum)| {
                    let value = if c.life { samples / sum } else { peak };
                    (c, value)
                })
                .chain(std::mem::take(&mut self.plan_checks).into_iter().map(|(c, _, peak)| (c, peak)))
                .chain(cam_checks(sim.machine.as_ref()))
                .collect(),
            clearance: (!sim.obstacles.is_empty()).then_some(sim.least_clearance),
            contact: sim.contact.clone(),
            braking_power_peak: self.braking_peak,
            warm_start: None,
            conveyor: sim.control.conveyor().map(|(c, _)| {
                let missed = c.missed - self.missed_before;
                ConveyorLoad {
                    picks_per_minute: 60.0 / duration,
                    tracking_peak: self.pick_tracking,
                    missed,
                    missed_per_minute: (missed as f64 * 60.0 / duration)
                        .max(c.offered_per_minute() - 60.0 / duration)
                        .max(0.0),
                }
            }),
            held_accelerations: self.held,
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
        let row = |id: &str, owner_key: &str, quantity_key: &str, energy: f64| BudgetRow {
            id: id.into(),
            owner_key: owner_key.into(),
            quantity_key: quantity_key.into(),
            energy,
        };
        let mut rows: Vec<BudgetRow> = sim
            .machine
            .motors()
            .iter()
            .zip(&self.copper)
            .map(|(m, copper)| row(&format!("copper.{}", m.id), m.key, "budget-copper", *copper))
            .collect();
        // The magnets' and iron's share of a back-EMF constant that is not the torque constant: only where there is one.
        rows.extend(
            sim.machine
                .motors()
                .iter()
                .zip(&self.emf)
                .filter(|(_, emf)| **emf != 0.0)
                .map(|(m, emf)| row(&format!("emf.{}", m.id), m.key, "budget-emf", *emf)),
        );
        let p = sim.control.drive_params();
        if p.dc_link {
            rows.push(row("resistor", "group-drive", "budget-resistor", self.resistor));
            rows.push(row("discharge", "group-drive", "budget-discharge", self.discharge));
        }
        if p.regenerative {
            rows.push(row("mains", "group-drive", "budget-mains", self.mains));
        }
        // The line module (or an S210's rectifier) loses its share of what it draws whether or not it feeds back.
        if p.dc_link {
            rows.push(row("line_module", "group-drive", "budget-line-module", self.line_loss));
        }
        rows.extend(
            self.losses
                .iter()
                .zip(&self.lost)
                .map(|(&i, lost)| row(&own[i].id, &own[i].owner_key, &own[i].quantity_key, *lost)),
        );
        rows.push(row("windings", "group-drive", "budget-windings", windings - self.start.1));
        if p.dc_link {
            rows.push(row("capacitors", "group-drive", "budget-capacitors", capacitors - self.start.2));
        }
        rows.extend(
            self.energies
                .iter()
                .zip(&self.start.0)
                .map(|(&i, start)| row(&own[i].id, &own[i].owner_key, &own[i].quantity_key, now[i] - start)),
        );
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
    let windings = drives.iter().enumerate().map(|(k, d)| d.magnetic_energy(sim.control.drive_params_of(k))).sum();
    // A link of its own has its drive's capacitance (a drive named per motor brings its own); a shared one each drive's
    // share of the shared `[drive]`'s, as `drive::shared_dc_link` steps it.
    let capacitance =
        |k: usize| if p.shared_dc_link { p.dc_capacitance } else { sim.control.drive_params_of(k).dc_capacitance };
    let capacitors = drives.iter().enumerate().map(|(k, d)| 0.5 * capacitance(k) * d.dc_voltage * d.dc_voltage).sum();
    (windings, if p.dc_link { capacitors } else { 0.0 })
}

/// Runs the configured cycle and measures its second pass, when the machine has settled into it. If a
/// drive trips before, the cycle never gets there: what is measured then is the run up to the trip.
///
/// With effects.thermal_warm_start, where windings (effects.winding_heating) or parts (effects.thermal_feedback) act
/// back on the run with their warmth: the cycle repeats from cold until a pass that starts with them at the rises it
/// settles at, within `WARM_START_TOLERANCE_K`; each pass ends with them put at the rises it settles at, and the pass
/// after it is measured. At most `WARM_START_PASSES` warm passes; the last is the measured one, converged or not
/// (`CycleLoad::warm_start`). With thermal time constants of minutes to an hour, a cycle of seconds would otherwise
/// see every part at the temperature it started with.
///
/// Where the config gives a path in place of the cycle (`[path]`, `sim::with_path`), that path is planned and measured
/// instead, as `analyse_trajectory` measures it: so the verdicts, the catalogue search and the optimiser judge the path's
/// run, repeated with the windings and parts at the rises it settles at under effects.thermal_warm_start.
pub fn analyse_cycle(sim: &mut Sim) -> Result<CycleLoad, CommandError> {
    analyse_cycle_with(sim, true)
}

/// `analyse_cycle`, with the effect views (`MotorLoad::torque_parts`, `CycleLoad::budget`) only where `views` asks
/// for them: they cost one more evaluation of the machine per step, and no verdict needs them.
pub fn analyse_cycle_with(sim: &mut Sim, views: bool) -> Result<CycleLoad, CommandError> {
    if let Some(points) = sim.plan_configured_path() {
        return analyse_trajectory_with(sim, points?, views);
    }
    sim.command(Command::StartCycle)?;
    let warm = sim.warm_start && !fed_back_rises(sim).is_empty();
    let mut tally = Tally::new(sim, views);
    let (mut passes, mut last, mut start) = (0usize, sim.control.cycle_passes(), 0.0);
    let (mut started_at, mut before) = (Vec::new(), None);
    // The warm spans measured so far, of `period` passes each, from the pass `span_from`; the settled rises per pass.
    let (mut spans, mut period, mut span_from, mut history) = (0usize, 1usize, 2usize, Vec::new());
    let (mut repeat, mut measured) = (None, None);
    loop {
        sim.step();
        // The controller counts the passes: a cycle of one segment never changes its segment index.
        let now = sim.control.cycle_passes();
        if now != last {
            passes += 1;
            let span_ends = passes == span_from + period;
            if warm && passes >= 3 && span_ends {
                // A measured span ended: it is the result, or the next starts at the rises it settles at.
                let load = std::mem::replace(&mut tally, Tally::new(sim, views)).finish(sim, sim.time() - start);
                let was = period;
                let next = warm_pass(
                    sim,
                    &started_at,
                    &load,
                    spans,
                    &mut before,
                    &mut period,
                    &mut history,
                    Some(&mut repeat),
                );
                if let Some(warm_start) = next {
                    let _ = sim.command(Command::Stop);
                    // After the repeat span (`warm_pass`) the span before it is the measured one.
                    let load = if warm_start.repeat.is_some() { measured.take().unwrap_or(load) } else { load };
                    return Ok(CycleLoad { warm_start: Some(warm_start), ..load });
                }
                if repeat.is_some() {
                    measured = Some(load);
                }
                spans = if period == was { spans + 1 } else { 0 };
                span_from = passes;
            }
            if passes == 2 || warm && passes > 2 && span_from == passes {
                started_at = fed_back_rises(sim);
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
        // A conveyor cycle ends where no part can be picked any more.
        if !sim.control.busy() && sim.control.enabled() {
            return Err(CommandError { key: "error-conveyor-unreachable" });
        }
        tally.step(sim);
        // Drives that tripped on DC link overvoltage never finish the cycle; the verdicts say why.
        if !sim.control.enabled() {
            break;
        }
    }
    let _ = sim.command(Command::Stop);
    // Stopped in the repeat span: the span before it stays the measured one, the repeat not told.
    if let (Some((_, done)), Some(load)) = (repeat, measured) {
        return Ok(CycleLoad { warm_start: Some(done), ..load });
    }
    let duration = sim.time() - start;
    // Stopped before a measured pass ended (a trip, or a crash): not converged, where a warm pass had begun; before
    // that the run was measured from cold.
    let warm_start =
        (warm && passes > 2).then(|| WarmStart { passes: passes - 2, change: f64::INFINITY, period: 1, repeat: None });
    Ok(CycleLoad { warm_start, ..tally.finish(sim, duration) })
}

/// Runs a path given as poses at times (see `Command::Trajectory`) and measures it, with 0.3 s of rest
/// after its end so that the settling of the tool counts. With effects.thermal_warm_start as `analyse_cycle`: the
/// path counts as a cycle that repeats, and every pass starts from a copy of the machine's start state
/// (`Sim::snapshot`) with the rises the previous pass settled, so nothing that still rings carries over.
pub fn analyse_trajectory(sim: &mut Sim, points: Vec<(f64, Vec<f64>)>) -> Result<CycleLoad, CommandError> {
    analyse_trajectory_with(sim, points, true)
}

/// `analyse_trajectory`, with the effect views only where `views` asks for them (see `analyse_cycle_with`).
pub fn analyse_trajectory_with(
    sim: &mut Sim,
    points: Vec<(f64, Vec<f64>)>,
    views: bool,
) -> Result<CycleLoad, CommandError> {
    let end = points.last().map_or(0.0, |p| p.0) + 0.3;
    let warm = sim.warm_start && !fed_back_rises(sim).is_empty();
    // Every pass starts from the machine's start state (a copy taken here), with the rises the previous one settled.
    let start_state = sim.snapshot();
    let mut before = None;
    for passes in 0.. {
        let started_at = fed_back_rises(sim);
        sim.command(Command::Trajectory(points.clone()))?;
        let (mut tally, start) = (Tally::new(sim, views), sim.time());
        while sim.time() - start < end {
            sim.step();
            tally.step(sim);
        }
        let load = tally.finish(sim, end);
        if !warm {
            return Ok(load);
        }
        if !sim.control.enabled() {
            return Ok(CycleLoad {
                warm_start: (passes > 0).then_some(WarmStart {
                    passes,
                    change: f64::INFINITY,
                    period: 1,
                    repeat: None,
                }),
                ..load
            });
        }
        // Every pass starts from the same state: the passes cannot differ but by their rises, so there is no period.
        let (mut period, mut history) = (1, Vec::new());
        if let Some(warm_start) =
            warm_pass(sim, &started_at, &load, passes, &mut before, &mut period, &mut history, None)
        {
            return Ok(CycleLoad { warm_start: Some(warm_start), ..load });
        }
        let rises = fed_back_rises(sim);
        sim.restore(&start_state).expect("a snapshot of this very simulation");
        set_rises(sim, &rises);
    }
    unreachable!()
}

/// The motor's S1 torque at a speed, as Siemens sizes a cycle (RMS torque at the mean speed against it): from the static
/// torque kt I_0 at standstill (`motor.stall_current_a`) straight to the rated torque at the rated speed, and on along
/// that line above it. `None` where the config has no stall current. Where it has the maker's printed curve
/// (`motor.s1_curve_speed_rad_s` and `motor.s1_curve_torque_nm`, digitised from the graph) the torque is read from it,
/// linearly between its points (`drive::s1_interpolated`), with the bend the voltage limit gives above the rated speed.
pub fn s1_torque(cfg: &Config, speed: f64) -> Result<Option<f64>, ConfigError> {
    let curve = crate::drive::s1_curve_of(cfg)?;
    if !curve.is_empty() {
        return Ok(Some(crate::drive::s1_interpolated(&curve, speed)));
    }
    if cfg.get("motor.stall_current_a").is_err() {
        return Ok(None);
    }
    let still = cfg.positive("motor.torque_constant_nm_a")? * cfg.positive("motor.stall_current_a")?;
    let (rated, rated_speed) = (cfg.positive("drive.rated_torque_nm")?, cfg.positive("drive.rated_speed_rad_s")?);
    Ok(Some(crate::drive::s1_line(still, rated, rated_speed, speed)))
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

/// Keys that no default config has and a config file may add (see `load_config`): ratings that `verdicts` judges, the
/// `[conveyor]` table (`conveyor::Conveyor`), whose cycle picks from a belt, the `[gripper]` table and the motor kinds'
/// own keys (`drive::Motor`: a stepper's and an induction motor's).
pub const RATINGS_BEYOND_DEFAULT: [&str; 84] = [
    // A guide's moment ratings (Mx, My, Mz), which its equivalent load takes where a machine gives the moments.
    "ratings.guide_moment_x_nm",
    "ratings.guide_moment_y_nm",
    "ratings.guide_moment_z_nm",
    // Optional end stops of a joint that turns on without stops of its own.
    "joint.stop_min_deg",
    "joint.stop_max_deg",
    // A guide set's carriages and rail length for the catalogue's guides, and the share of the moved mass and of the guide
    // friction that the config's own guide carriages are (`catalog::mechanics`).
    "ratings.guide_carriages",
    "ratings.guide_rail_length_m",
    "mass.guide_carriages_kg",
    "friction_x.seal_drag_n",
    "friction.seal_drag_n",
    "slide.seal_drag_n",
    // The gear's presliding stiffnesses over K1 and the rack's pressure angle (`mech::Gearbox`, `transmission::RackPinion`).
    "gearbox.hysteresis_stiffness_factor",
    "gearbox.mesh_stiffness_factor",
    "rack.pressure_angle_deg",
    // The housing's highest rise (`charts::utilisation`).
    "ratings.housing_max_rise_k",
    // The maker's printed S1 curve (`s1_torque`).
    "motor.s1_curve_speed_rad_s",
    "motor.s1_curve_torque_nm",
    // What `stop::study` judges the worst stop against.
    "ratings.max_stopping_distance_m",
    "ratings.max_stopping_angle_rad",
    // The gripper's time and hold (`gripper::Gripper`), and the hold's safety factor.
    "effects.gripper_time",
    "gripper",
    "gripper.kind",
    "gripper.close_s",
    "gripper.open_s",
    "gripper.cup_volume_m3",
    "gripper.generator_flow_m3_s",
    "gripper.vacuum_pa",
    "gripper.holding_pressure_pa",
    "gripper.vent_flow_m3_s",
    "gripper.release_pressure_pa",
    "gripper.cup_area_m2",
    "gripper.friction_coefficient",
    "ratings.gripper_safety_factor",
    // A path in place of the cycle (`sim::with_path`).
    "path",
    "path.file",
    "path.program",
    "path.time_optimal",
    // How fast a robot program turns the tool (`program::read_with`).
    "program",
    "program.orientation_speed_rad_s",
    "ratings.settling_tolerance_rad",
    "ratings.pulley_max_torque_nm",
    "ratings.motor_bearing_load_rating_n",
    "conveyor",
    "conveyor.enabled",
    "conveyor.direction",
    "conveyor.speed_m_s",
    "conveyor.ramp_s",
    "conveyor.origin",
    "conveyor.length_m",
    "conveyor.pitch_s",
    "conveyor.arrivals_s",
    "conveyor.window_m",
    "conveyor.lift",
    "conveyor.grip_s",
    "conveyor.release_s",
    "conveyor.place",
    "conveyor.place_direction",
    "conveyor.place_speed_m_s",
    "conveyor.tracking_tolerance_m",
    "motor.kind",
    "motor.rotor_teeth",
    "motor.detent_torque_nm",
    "effects.detent_torque",
    "drive.microsteps",
    "drive.stepper_current_a",
    "motor.rotor_resistance_ohm",
    "motor.magnetizing_inductance_h",
    "motor.stator_leakage_inductance_h",
    "motor.rotor_leakage_inductance_h",
    "motor.rated_voltage_v",
    "motor.rated_frequency_hz",
    "drive.induction_control",
    "drive.vf_boost_v",
    "drive.vf_slip_compensation",
    // The positioning accuracy test (`accuracy::study`): its two effects, the axes they act on, and its ratings.
    "effects.thermal_growth",
    "effects.lead_error",
    "accuracy",
    "accuracy.axes",
    "accuracy.screw_expansion_per_k",
    "accuracy.belt_expansion_per_k",
    "ratings.max_positioning_error_m",
    "ratings.max_repeatability_m",
    "ratings.max_positioning_error_rad",
    "ratings.max_repeatability_rad",
];

fn at_most(check: &'static str, motor: Option<usize>, value: f64, limit: f64) -> Verdict {
    Verdict { check, motor, value, limit, ok: value <= limit }
}

fn at_least(check: &'static str, motor: Option<usize>, value: f64, limit: f64) -> Verdict {
    Verdict { check, motor, value, limit, ok: value >= limit }
}

/// The checks whose value has to stay at or above its limit ("belt-least-tension": above); every other check's value has to
/// stay at or below it.
pub const AT_LEAST: [&str; 10] = [
    "belt-least-tension",
    "pulley-bearing-life",
    "gearbox-life",
    "gearbox-gearing-life",
    "motor-bearing-life",
    "guide-life",
    "belt-span-resonance",
    "screw-life",
    "strain-wave-life",
    "collision",
];

/// Above 1 it fails (but "belt-least-tension" at exactly 1). A check of `AT_LEAST` at or below zero (a contact, "collision" at no
/// clearance; a belt section pushed, "belt-least-tension" with a negative tension) is infinitely far over: limit over value would
/// be negative there, and read as a margin above 100 %.
pub fn utilisation(v: &Verdict) -> f64 {
    if AT_LEAST.contains(&v.check) && v.value <= 0.0 {
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
/// "belt-least-tension" also fails when any belt section spent time without tension.
/// With `effects.dc_link` the braking checks judge the simulated DC link: what the capacitors held at the
/// highest voltage, the mean and peak power the resistor burned, and the highest voltage against the trip
/// level. Without it they are estimated from the energy the motor fed back, as before.
///
/// "line-module-continuous" and "line-module-peak" judge, with effects.regenerative_supply (and the DC link), the line
/// module's mean and highest power either way against `ratings.line_module_rated_power_w` and `_max_power_w`.
///
/// "pulley-bearing-life", "gearbox-life", "gearbox-gearing-life", "motor-bearing-life" and "guide-life" judge
/// `pulley_bearing_life_h`, `gearbox_life_h`, `gearbox_gearing_life_h`, `motor_bearing_life_h` and `guide_life_h` against
/// `ratings.required_life_h` (they pass at or above it); "belt-tooth-load" judges `belt_tooth_utilisation` against 1.
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
    // A table there is a motor's own ratings (`catalog::motor_config`), checked with that motor.
    for (name, value) in cfg.get("ratings").ok().and_then(|ratings| ratings.as_table()).into_iter().flatten() {
        if !value.is_array() && !value.is_table() {
            cfg.f64(&format!("ratings.{name}"))?;
        }
    }
    let rating = |name: &str| cfg.f64(&format!("ratings.{name}")).ok();
    let simulated = cfg.bool("effects.dc_link")?;
    let mut out = Vec::new();
    // An S120 line-up brakes through one Braking Module for all its axes (`drive.braking_module`): their braking power
    // is judged together, after the axes. An S210 has a resistor of its own and is judged per axis.
    let line_up = cfg.get("drive.braking_module").is_ok() && cfg.bool("drive.braking_module")?;
    let mut line_up_brakes = false;
    let mut regenerated_excess = 0.0;
    let shared = cfg;
    for (k, m) in load.motors.iter().enumerate() {
        // Each motor against its own motor, drive and ratings.
        let cfg = &crate::catalog::motor_config(shared, &m.id)?;
        let rating = |name: &str| cfg.f64(&format!("ratings.{name}")).ok();
        let (bus, rated_rise) = (cfg.positive("drive.bus_voltage_v")?, cfg.positive("motor.rated_temperature_rise_k")?);
        // Peak torque against the torque-speed curve, and the settled winding temperature against the rated rise.
        out.push(at_most("motor-peak", Some(k), m.utilisation, 1.0));
        out.push(at_most("motor-thermal", Some(k), m.winding_rise, rated_rise));
        // A stepper's load angle against its pull-out angle, pi / 2 electrical: beyond it the rotor falls out of step.
        if let Some(angle) = m.load_angle {
            out.push(at_most("stepper-synchronism", Some(k), angle, std::f64::consts::FRAC_PI_2));
        }
        // Siemens' S1 check: the RMS torque at the mean speed against the S1 curve (`s1_torque`), for a servo motor; a
        // stepper's and an induction motor's heat is judged by "motor-thermal" alone.
        let servo = cfg.get("motor.kind").ok().and_then(|v| v.as_str()).is_none_or(|kind| kind == "servo");
        if let (true, Some(limit)) = (servo, s1_torque(cfg, m.mean_speed)?) {
            out.push(at_most("motor-s1", Some(k), m.rms_torque, limit));
        }
        // The peak speed against the motor's maximum speed, where the config gives it (a catalogue motor does).
        if cfg.get("drive.max_speed_rad_s").is_ok() {
            out.push(at_most("motor-speed", Some(k), m.peak_speed, cfg.positive("drive.max_speed_rad_s")?));
        }
        // The drive's output current: the cycle's RMS against its rated current (a cycle of seconds against a power
        // unit that warms in seconds to minutes), its peak against the maximum current.
        if let Some(limit) = rating("drive_rated_current_a") {
            out.push(at_most("drive-current", Some(k), m.rms_current, limit));
        }
        if let Some(limit) = rating("drive_max_current_a") {
            out.push(at_most("drive-peak-current", Some(k), m.peak_current, limit));
        }
        // effects.drive_overload: the thermal image's peak, the cycle repeated, against the level at which the drive
        // lowers its current to the base load current.
        if let Some(o) = crate::drive::Overload::from_config(cfg)? {
            out.push(at_most("drive-overload", Some(k), m.drive_overload, o.trip));
        }
        if let (Some((peak, _)), Some(limit)) = (m.gearbox_torque, rating("pulley_max_torque_nm")) {
            out.push(at_most("pulley-torque", Some(k), peak, limit));
        }
        // The radial load on this motor's shaft against its own gearbox's rating (a smaller Z gearbox has its own).
        if let (Some(peak), Some(limit)) = (m.radial_load_peak, rating("shaft_max_radial_load_n")) {
            out.push(at_most("gearbox-radial-load", Some(k), peak, limit));
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
        lubrication(cfg)?;
        if let Some(required) = rating("required_life_h") {
            if let Some(life) = pulley_bearing_life_h(cfg, m) {
                out.push(at_least("pulley-bearing-life", Some(k), life, required));
            }
            if let Some(life) = gearbox_life_h(cfg, m) {
                out.push(at_least("gearbox-life", Some(k), life, required));
            }
            if let Some(life) = gearbox_gearing_life_h(cfg, m) {
                out.push(at_least("gearbox-gearing-life", Some(k), life, required));
            }
            if let Some(life) = motor_bearing_life_h(cfg, m) {
                out.push(at_least("motor-bearing-life", Some(k), life, required));
            }
        }
        if let Some(utilisation) = belt_tooth_utilisation(cfg, m)? {
            out.push(at_most("belt-tooth-load", Some(k), utilisation, 1.0));
        }
        // Braking: what the DC link capacitors take between the bus voltage and the threshold. If one
        // braking gives back more, the braking resistor has to take the mean and the peak power.
        if let (Some(capacitance), Some(threshold)) = (rating("dc_link_capacitance_uf"), rating("braking_threshold_v"))
        {
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
                // The capacitors take the braking up to their room; only what is above it goes to the resistor.
                // The run's own account where it kept one for this room, else the stretches of a load built by hand.
                let (excess, fill) = match &m.dc_link_account {
                    Some(account) if account.room == room => account.finish(),
                    _ => resistor_excess(&m.regeneration_events, room),
                };
                regenerated_excess += excess;
                (fill, fill > room, excess / load.duration, m.peak_regenerated_power)
            };
            // With a resistor to fall back on, a full DC link is no failure in itself.
            out.push(Verdict {
                check: "dc-link-energy",
                motor: Some(k),
                value: stored,
                limit: room,
                ok: !resistor || continuous.is_some(),
            });
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
            (false, _) => regenerated_excess,
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
    // The turn's (an arm's orientation, a turning tool or table), on its own: no metres and radians added.
    if let Some(limit) = rating("settling_tolerance_rad") {
        out.push(at_most("settling-angle", None, load.dwell_angle_error_peak, limit));
    }
    // A vacuum gripper (effects.gripper_time): the most the held part asked of the cup against its hold over the
    // safety factor (`gripper`).
    if let (Some(crate::gripper::Gripper { hold: Some((hold, mu)), .. }), Some(factor)) =
        (crate::gripper::Gripper::from_config(cfg)?, rating("gripper_safety_factor"))
    {
        let (mass, g) = (crate::gripper::payload_mass(cfg)?, crate::gripper::gravity(cfg)?);
        let asked = load
            .held_accelerations
            .iter()
            .map(|&[up, side]| crate::gripper::demand(mass, [side, 0.0, up], g, mu))
            .fold(0.0, f64::max);
        out.push(at_most("gripper-hold", None, asked, hold / factor));
    }
    if let Some((slackest, tightest)) = load.belt_tension {
        if let Some(limit) = rating("belt_max_tension_n") {
            out.push(Verdict { check: "belt-tension", motor: None, value: tightest, limit, ok: tightest <= limit });
        }
        if let Some(limit) = rating("belt_min_tension_n") {
            // The one check where the value has to stay above its limit. The tension is looked at once per
            // millisecond; the slack time also counts what falls between.
            let never_slack = load.belt_slack_time.is_none_or(|time| time <= 0.0);
            out.push(Verdict {
                check: "belt-least-tension",
                motor: None,
                value: slackest,
                limit,
                ok: slackest > limit && never_slack,
            });
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
        let a1 = reliability_factor(cfg)
            .ok_or_else(|| ConfigError::Invalid("ratings.bearing_reliability_percent".into()))?;
        out.push(if check.life {
            at_least(check.check, check.motor, value * a1, check.limit)
        } else {
            at_most(check.check, check.motor, *value, check.limit)
        });
    }
    // The least clearance to the obstacles against `profile.min_clearance_m`; a contact fails whatever the limit.
    if let Some(clearance) = load.clearance {
        let limit = cfg.non_negative("profile.min_clearance_m")?;
        out.push(Verdict {
            check: "collision",
            motor: None,
            value: clearance,
            limit,
            ok: clearance >= limit && clearance > 0.0,
        });
    }
    if let (Some(value), Some(limit)) = (load.condition_peak, rating("max_condition_number")) {
        out.push(at_most("singularity", None, value, limit));
    }
    // Once for the machine only where no motor's shaft reports its own (judged per motor above); before 14 November 2026
    // always so, the highest of all shafts against the shared rating.
    let per_motor = load.motors.iter().any(|m| m.radial_load_peak.is_some());
    if let (Some(value), Some(limit), false) = (load.radial_load_peak, rating("shaft_max_radial_load_n"), per_motor) {
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
            let hottest =
                load.part_rises.iter().filter(|(part, _)| part.kind == kind).map(|(_, rise)| *rise).reduce(f64::max);
            if let Some(rise) = hottest {
                out.push(at_most(check, None, rise, cfg.positive(&format!("thermal.{kind}_max_rise_k"))?));
            }
        }
    }
    // How the measured pass was started (effects.thermal_warm_start): the most a rise it started at lies from the one it
    // settles at, against the warm start's tolerance. No row: from cold, or a change without a value (a run that stopped,
    // a winding that does not settle), whose cause fails a verdict of its own (the trip's, the crash's, motor-thermal).
    if let Some(warm) = load.warm_start.filter(|w| w.change.is_finite()) {
        out.push(at_most("thermal-warm-start", None, warm.change, WARM_START_TOLERANCE_K));
        // Where it did not converge: whether two passes started at the same rises settle alike (`WarmStart::repeat`).
        if let Some(apart) = warm.repeat {
            out.push(at_most("thermal-cycle-repeat", None, apart, WARM_START_TOLERANCE_K));
        }
    }
    // A pick from the conveyor: the tool on the part along the belt while the gripper closes, and no part left unpicked.
    if let Some(c) = load.conveyor {
        out.push(at_most("pick-tracking", None, c.tracking_peak, cfg.positive("conveyor.tracking_tolerance_m")?));
        out.push(at_most("parts-missed", None, c.missed_per_minute, 0.0));
    }
    Ok(out)
}

/// The equivalent load of a guide: the cubic mean cbrt(sum F^3 w / sum w) with the weight w (|speed|: over distance);
/// 0 where nothing moved.
fn equivalent_load(cubed_sum: f64, weight_sum: f64) -> f64 {
    if weight_sum > 0.0 { (cubed_sum / weight_sum).cbrt() } else { 0.0 }
}

/// The energy a braking resistor takes per cycle, J, and the most the DC link's capacitors would have to hold, J, from the
/// cycle's stretches `events` (`MotorLoad::regeneration_events`: fed back positive, drawn negative) when the capacitors
/// hold `room` J above the bus: the link fills with what is fed back, spills what is above its room into the resistor and
/// empties by what is drawn (to the bus, no lower; its own losses left out, so the estimate errs high). Braking events
/// with too little drawn between them add up. The cycle repeats: the link enters it as the cycle before left it, so the
/// figures are those of a second pass.
fn resistor_excess(events: &[f64], room: f64) -> (f64, f64) {
    let mut account = LinkAccount::new(room);
    events.iter().for_each(|&e| account.push(e));
    account.finish()
}

/// The DC link's running account of `resistor_excess`, kept as the stretches come (`Tally`, where the motor's room is
/// known at the run: `Controller::dc_link_room`). Only the stretches until the link's fill no longer depends on how full
/// it entered the cycle are kept: until the pass entered empty and the one entered full meet, which they do the first
/// time the link empties or fills whatever it held (a hanging axis at rest draws, so empties it at once). After that one
/// fill stands for every entry, and what it spills and the most it holds are summed as they come. At most the stretches
/// up to that point, so on a run at rest with noisy power the list stays at the few stretches before the link first
/// emptied, where before it grew by one per sign change of the power (about 90 per cycle). Same figures as the two passes
/// over the whole list, to rounding (the spills are summed in another order).
///
/// A link that neither empties nor fills (what is fed back and drawn stays within less than its room for the whole run)
/// never lets the passes meet. Past `LINK_PREFIX_CAP` stretches the account keeps, instead of them, `LINK_GRID` passes
/// entered at fills spread evenly over the voltage window, 0 to the room, and reads the cycle's figures between the two
/// next to its entry: fill, spill and most are each at most 1:1 in the entry fill (what one enters with more is spilled,
/// drawn less or held), so the figures are off by at most half the grid's step, room / (2 (LINK_GRID − 1)).
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LinkAccount {
    /// The room it was kept for, J.
    pub room: f64,
    /// The stretches before the passes entered empty and full met.
    prefix: Vec<f64>,
    /// Past `LINK_PREFIX_CAP` stretches in `prefix`: per entry fill j room / (LINK_GRID − 1) its fill, spill and most
    /// over the stretches until the passes met, in place of them.
    grid: Vec<[f64; 3]>,
    /// The fill of the pass entered empty, and of the one entered full, J; one fill once they have met.
    low: f64,
    high: f64,
    met: bool,
    /// After they met: what the link spilled into the resistor, J, and the most it would have held.
    spilled: f64,
    most: f64,
}

impl LinkAccount {
    pub fn new(room: f64) -> Self {
        Self { room, high: room, ..Default::default() }
    }

    /// One stretch: fed back positive, drawn negative, J.
    pub fn push(&mut self, e: f64) {
        let room = self.room;
        if self.met {
            let fill = (self.low + e).max(0.0);
            (self.spilled, self.most, self.low) =
                (self.spilled + (fill - room).max(0.0), self.most.max(fill), fill.min(room));
            return;
        }
        if self.grid.is_empty() {
            self.prefix.push(e);
            if self.prefix.len() > LINK_PREFIX_CAP {
                self.grid = (0..LINK_GRID)
                    .map(|j| {
                        let mut pass = [j as f64 * room / (LINK_GRID - 1) as f64, 0.0, 0.0];
                        self.prefix.iter().for_each(|&e| link_step(&mut pass, e, room));
                        pass
                    })
                    .collect();
                self.prefix = Vec::new();
            }
        } else {
            self.grid.iter_mut().for_each(|pass| link_step(pass, e, room));
        }
        let step = |fill: f64| (fill + e).max(0.0).min(room);
        (self.low, self.high) = (step(self.low), step(self.high));
        // Both at 0 or both at the room, exactly.
        self.met = self.low == self.high;
    }

    /// What the resistor takes per cycle, J, and the most the link would have to hold, J (`resistor_excess`): the cycle
    /// entered as it left, which is as the pass entered empty left it.
    pub fn finish(&self) -> (f64, f64) {
        let mut pass = [self.low, self.spilled, self.most];
        self.prefix.iter().for_each(|&e| link_step(&mut pass, e, self.room));
        if self.grid.len() >= 2 {
            // Between the two passes of the grid next to the entry fill.
            let at = self.low / self.room * (self.grid.len() - 1) as f64;
            let j = (at.floor() as usize).min(self.grid.len() - 2);
            let (u, [a, b]) = (at - j as f64, [self.grid[j], self.grid[j + 1]]);
            let between = |q: usize| a[q] + u * (b[q] - a[q]);
            (pass[1], pass[2]) = (pass[1] + between(1), pass[2].max(between(2)));
        }
        (pass[1], pass[2])
    }
}

/// Stretches a link account keeps before it keeps passes over the voltage window in their place (`LinkAccount`): some
/// 1500 cycles of a machine at rest whose power changes sign about 90 times per cycle.
pub const LINK_PREFIX_CAP: usize = 1 << 17;

/// Entry fills over the voltage window a link account keeps passes for past `LINK_PREFIX_CAP` (`LinkAccount`).
pub const LINK_GRID: usize = 257;

/// One stretch `e` on a pass of a link of `room` J: its fill, what it spilled and the most it held, J.
fn link_step(pass: &mut [f64; 3], e: f64, room: f64) {
    let fill = (pass[0] + e).max(0.0);
    pass[2] = pass[2].max(fill);
    pass[1] += (fill - room).max(0.0);
    pass[0] = fill.min(room);
}

/// Nominal life of the guide that has the least, in km, where the machine reports guide loads and the config has
/// `ratings.guide_dynamic_load_rating_n`: L10 = (C / P)^3 * 50 km, the catalogue formula for ball guides,
/// with the dynamic load rating C and for P the cubic mean of the guide load over the cycle's distance (weighted by
/// the axis's |speed|, as the pulley's radial load is; over time where the guide's axis is unknown). The lowest over
/// all guides (`CycleLoad::guides`).
///
/// This is the catalogue formula and nothing more, and it is not validated: no load factors for shock,
/// temperature or hardness, no moments on single blocks,
/// and a guide load that is itself an estimate from the rigid model.
pub fn guide_life_km(cfg: &Config, load: &CycleLoad) -> Option<f64> {
    let rating = cfg.f64("ratings.guide_dynamic_load_rating_n").ok()?;
    guides_of(load).iter().map(|g| (rating / g.equivalent).powi(3) * 50.0).reduce(f64::min)
}

/// `guide_life_km` in hours: the distance over the mean speed of the guide's axis over the cycle (`GuideLoad::speed`),
/// times the reliability factor a1 as every other life (`reliability_factor`); the lowest over the guides whose axis is
/// known.
pub fn guide_life_h(cfg: &Config, load: &CycleLoad) -> Option<f64> {
    let rating = cfg.f64("ratings.guide_dynamic_load_rating_n").ok()?;
    let a1 = reliability_factor(cfg)?;
    let hours = |g: &GuideLoad| -> Option<f64> {
        Some((rating / g.equivalent).powi(3) * 50.0 * 1e3 / (g.speed? * 3600.0) * a1)
    };
    guides_of(load).iter().filter_map(hours).reduce(f64::min)
}

/// The guides of a load: `CycleLoad::guides`, or the one pair of `guide_load` and `guide_speed` where a load was built
/// without them.
fn guides_of(load: &CycleLoad) -> Vec<GuideLoad> {
    if !load.guides.is_empty() {
        return load.guides.clone();
    }
    load.guide_load
        .map(|(peak, equivalent)| GuideLoad { peak, equivalent, speed: load.guide_speed })
        .into_iter()
        .collect()
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

/// This motor's own `gearbox.ratio` (`catalog::motor_config`: a `[gearbox.<id>]` table over the shared one), so a machine
/// whose motors have different ratios gets each gearbox's own speed. Applying it to a config that is already the
/// motor's own changes nothing.
fn own_ratio(cfg: &Config, m: &MotorLoad) -> f64 {
    let own = crate::catalog::motor_config(cfg, &m.id);
    own.as_ref().ok().unwrap_or(cfg).f64("gearbox.ratio").unwrap_or(1.0)
}

/// The mean speed of the gearbox output (the pulley) in r/min: the motor's over its own `gearbox.ratio`.
fn output_rpm(cfg: &Config, m: &MotorLoad) -> f64 {
    m.mean_speed * 30.0 / std::f64::consts::PI / own_ratio(cfg, m)
}

/// The rating life of the bearing that takes the belt's pull on this motor's pulley shaft, in hours, where the machine
/// reports that radial load and the config has `ratings.pulley_bearing_load_rating_n`: `rating_life_h` with that C, P the
/// cubic mean of the radial load weighted with the speed (`MotorLoad::radial_load_equivalent`) and the pulley's mean
/// speed, times the life factor a1 (`reliability_factor`) and, where the config has the lubrication (`lubrication`),
/// the life modification factor a_ISO (`life_modification_factor`): L_nm = a1 · a_ISO · L10. One bearing, all of the
/// pull on it, no axial load.
pub fn pulley_bearing_life_h(cfg: &Config, m: &MotorLoad) -> Option<f64> {
    let c = cfg.f64("ratings.pulley_bearing_load_rating_n").ok()?;
    let (p, rpm) = (m.radial_load_equivalent?, output_rpm(cfg, m));
    Some(reliability_factor(cfg)? * pulley_bearing_a_iso(cfg, p, rpm)? * rating_life_h(c, p, rpm))
}

/// a_ISO of the pulley shaft bearing at the equivalent load `p` and the mean speed `rpm`: `life_modification_factor` of
/// κ = ν / ν1 (`reference_viscosity`) and e_C · C_u / P. 1 where the config has no `lubrication`; `None` where it is
/// invalid. κ is that of the mean speed.
pub fn pulley_bearing_a_iso(cfg: &Config, p: f64, rpm: f64) -> Option<f64> {
    Some(match lubrication(cfg).ok()? {
        Some(l) => life_modification_factor(
            l.viscosity / reference_viscosity(rpm, l.mean_diameter_mm),
            l.contamination * l.fatigue_limit / p,
        ),
        None => 1.0,
    })
}

/// What a_ISO of the pulley shaft bearing is worked out from: the lubricant's kinematic viscosity at operating
/// temperature ν (mm²/s), the contamination factor e_C (ISO 281:2007, Table 13, for a mean diameter below 100 mm:
/// 1 extreme cleanliness, 0.8 to 0.6 high (typical of bearings sealed and greased for life), 0.6 to 0.5 normal, 0.5 to
/// 0.3 slight, 0.3 to 0.1 typical, 0.1 to 0 severe, 0 very severe contamination), the bearing's fatigue load limit C_u
/// (N) and its mean diameter D_pw (mm).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Lubrication {
    pub viscosity: f64,
    pub contamination: f64,
    pub fatigue_limit: f64,
    pub mean_diameter_mm: f64,
}

/// The keys of `Lubrication`, in its order.
pub const LUBRICATION_KEYS: [&str; 4] = [
    "ratings.bearing_lubricant_viscosity_mm2_s",
    "ratings.bearing_contamination_factor",
    "ratings.pulley_bearing_fatigue_limit_n",
    "ratings.pulley_bearing_mean_diameter_mm",
];

/// The pulley shaft bearing's `Lubrication`, `None` where the config has none of its keys (then a_ISO is 1). A config
/// with some of them must have all: ν and D_pw above 0, e_C from 0 to 1, C_u not negative.
pub fn lubrication(cfg: &Config) -> Result<Option<Lubrication>, ConfigError> {
    if LUBRICATION_KEYS.iter().all(|key| cfg.get(key).is_err()) {
        return Ok(None);
    }
    let contamination = cfg.non_negative(LUBRICATION_KEYS[1])?;
    if contamination > 1.0 {
        return Err(ConfigError::Invalid(LUBRICATION_KEYS[1].into()));
    }
    let (viscosity, fatigue_limit, mean_diameter_mm) = (
        cfg.positive(LUBRICATION_KEYS[0])?,
        cfg.non_negative(LUBRICATION_KEYS[2])?,
        cfg.positive(LUBRICATION_KEYS[3])?,
    );
    Ok(Some(Lubrication { viscosity, contamination, fatigue_limit, mean_diameter_mm }))
}

/// The rated viscosity ν1 of ISO 281:2007 (section 6.3.3.2), mm²/s: what the lubricant needs at operating temperature
/// for a bearing of mean diameter D_pw (`mean_diameter_mm`) at n (`rpm`, r/min): 45000 · n^-0.83 · D_pw^-0.5 below
/// 1000 r/min, 4500 · n^-0.5 · D_pw^-0.5 from 1000 r/min. The viscosity ratio is κ = ν / ν1.
pub fn reference_viscosity(rpm: f64, mean_diameter_mm: f64) -> f64 {
    if rpm < 1000.0 {
        45000.0 * rpm.powf(-0.83) / mean_diameter_mm.sqrt()
    } else {
        4500.0 / rpm.sqrt() / mean_diameter_mm.sqrt()
    }
}

/// The life modification factor a_ISO of ISO 281:2007 (section 6.3.3.3, radial ball bearings) for the viscosity ratio κ
/// and `load_ratio` = e_C · C_u / P: a_ISO = 0.1 · [1 - x · (e_C · C_u / P)^(1/3)]^-9.3, x = (2.5671 - 2.2649 /
/// κ^0.054381)^0.83 for 0.1 <= κ < 0.4, (2.5671 - 1.9987 / κ^0.19087)^0.83 for 0.4 <= κ < 1 and
/// (2.5671 - 1.9987 / κ^0.071739)^0.83 for 1 <= κ <= 4; at most 50, as the standard limits it.
pub fn life_modification_factor(kappa: f64, load_ratio: f64) -> f64 {
    // shortcut: κ below 0.1, where the standard gives no factor, is taken as 0.1 (a_ISO 0.1); above 4 as 4, as the
    // standard says. Upgrade if a machine runs its bearings that slowly with that thin a lubricant.
    let kappa = kappa.clamp(0.1, 4.0);
    let x = if kappa < 0.4 {
        2.5671 - 2.2649 / kappa.powf(0.054381)
    } else if kappa < 1.0 {
        2.5671 - 1.9987 / kappa.powf(0.19087)
    } else {
        2.5671 - 1.9987 / kappa.powf(0.071739)
    };
    let inner = 1.0 - x.max(0.0).powf(0.83) * load_ratio.cbrt();
    if inner <= 0.0 { 50.0 } else { (0.1 * inner.powf(-9.3)).min(50.0) }
}

/// The life of a planetary gearbox's planet bearings in hours by Siemens' formula (configuration manual "Planetary
/// gearboxes for S-1FK2 and S-1FT2 servomotors", 01/2023, section 6.3.3): L10h = 20000 h · (n2B / n2) · (M2N / M2eq)^(10/3),
/// with the rated torque M2N (`ratings.gearbox_rated_torque_nm`), the reference output speed n2B
/// (`ratings.gearbox_reference_speed_rpm`), the mean output speed n2 and M2eq the mean of the output torque with the
/// exponent 10/3 weighted with the speed (`MotorLoad::gearbox_life_torque`). Where the machine reports the output torque
/// and the config has both ratings. Times the life factor a1 (`reliability_factor`): Siemens' figure is an L10 too.
pub fn gearbox_life_h(cfg: &Config, m: &MotorLoad) -> Option<f64> {
    let (rated, reference) =
        (cfg.f64("ratings.gearbox_rated_torque_nm").ok()?, cfg.f64("ratings.gearbox_reference_speed_rpm").ok()?);
    Some(
        reliability_factor(cfg)? * 20000.0 * reference / output_rpm(cfg, m)
            * (rated / m.gearbox_life_torque?).powf(10.0 / 3.0),
    )
}

/// The lowest torque of `MotorLoad::gearbox_torque_spectrum`, N m, and the ratio of a band's upper torque to its lower:
/// bands 1 % wide, each judged at its upper torque, which overstates its damage by at most 1.01^k - 1 (9 % at k = 8.7).
const TORQUE_FLOOR: f64 = 0.1;
const TORQUE_BAND: f64 = 1.01;

/// The lower torque of band `i` of `MotorLoad::gearbox_torque_spectrum`, N m: 0.1 N m · 1.01^i.
pub fn torque_band_floor(i: usize) -> f64 {
    TORQUE_FLOOR * TORQUE_BAND.powi(i as i32)
}

/// The output revolutions a Siemens planetary gearbox takes at its maximum torque M2max (NRB, NRK, NLC: "for at most
/// 30 000 output revolutions", equipment manual 04/2025, chapter 6; `catalog/siemens/gearbox_nrb.meta.toml`).
pub const MAX_TORQUE_REVOLUTIONS: f64 = 30000.0;

/// The life of a gearing in hours by Palmgren and Miner's damage sum over a load spectrum of (output torque, output
/// revolutions per hour), on a Wöhler line through the maximum torque M2max at `MAX_TORQUE_REVOLUTIONS` with the
/// exponent k: a torque M above the rated torque M2N allows N = 30000 · (M2max / M)^k revolutions, one at or below M2N
/// any number (the knee of the line at M2N). The damage per hour is the sum of the revolutions over the allowed ones, the
/// life its inverse; infinite where no torque is above M2N.
pub fn gearing_life_h(spectrum: impl IntoIterator<Item = (f64, f64)>, rated: f64, max: f64, exponent: f64) -> f64 {
    // A fold, not `sum`: an empty sum of f64 is -0.0, whose inverse is -inf.
    1.0 / spectrum
        .into_iter()
        .filter(|&(torque, _)| torque > rated)
        .map(|(torque, revolutions)| revolutions / (MAX_TORQUE_REVOLUTIONS * (max / torque).powf(exponent)))
        .fold(0.0, |sum, d| sum + d)
}

/// The fatigue life of the gearbox's gearing in hours, where the machine reports the output torque and the config has
/// `ratings.gearbox_rated_torque_nm` (M2N), `ratings.gearbox_max_torque_nm` (M2max) and
/// `ratings.gearbox_gearing_wohler_exponent` (k): `gearing_life_h` over the cycle's load spectrum
/// (`MotorLoad::gearbox_torque_spectrum`), each band at its upper torque, with the output's revolutions per hour in it.
/// Siemens' own damage sum over overload partial cycles is in its gearbox configuration manual, which is not in the
/// repository; this is the textbook method on Siemens' two figures (M2N continuous, M2max for 30000 revolutions) with an
/// estimated exponent (the configs' comments say how). Torque at or below M2N does no damage: a cycle that never exceeds
/// it has no figure (`None`), as a rating left out is not judged; the life would be infinite.
pub fn gearbox_gearing_life_h(cfg: &Config, m: &MotorLoad) -> Option<f64> {
    let rating = |key: &str| cfg.f64(&format!("ratings.{key}")).ok();
    let (rated, max, exponent) = (
        rating("gearbox_rated_torque_nm")?,
        rating("gearbox_max_torque_nm")?,
        rating("gearbox_gearing_wohler_exponent")?,
    );
    m.gearbox_torque?;
    let per_hour = 3600.0 / (2.0 * std::f64::consts::PI) / own_ratio(cfg, m);
    let life = gearing_life_h(
        m.gearbox_torque_spectrum.iter().enumerate().map(|(i, speed)| (torque_band_floor(i + 1), speed * per_hour)),
        rated,
        max,
        exponent,
    );
    life.is_finite().then_some(life)
}

/// The rating life of the motor's own bearings in hours, where the belt's pulley sits on the motor shaft: where the
/// config has `ratings.motor_bearing_load_rating_n` and no gearbox (`gearbox.ratio` 1), `rating_life_h` with that C, the
/// pulley shaft's radial load (`MotorLoad::radial_load_equivalent`) and the motor's mean speed, times a1. Behind a
/// gearbox the motor's bearings carry no belt pull, the gearbox's output bearing takes it (`pulley_bearing_life_h`), and
/// there is no figure. Siemens gives no load rating for the 1FK2's bearings, only the radial force they allow for
/// L10h = 25000 h at a mean speed; C is worked out from that by the same formula (`catalog/siemens/motor_bearings_1fk2.csv`).
/// No a_ISO: Siemens' 25000 h are for its own grease and seals.
pub fn motor_bearing_life_h(cfg: &Config, m: &MotorLoad) -> Option<f64> {
    let c = cfg.f64("ratings.motor_bearing_load_rating_n").ok()?;
    if own_ratio(cfg, m) != 1.0 {
        return None;
    }
    Some(reliability_factor(cfg)? * rating_life_h(c, m.radial_load_equivalent?, output_rpm(cfg, m)))
}

/// The tooth load of the belt on this motor's pulley against its rating, by the method of Megadyne's MEGALINEAR handbook
/// (02.2026, page 9): the effective pull Fu = output torque / `geometry.pulley_radius_m` times the service factor
/// `ratings.belt_service_factor`, against the specific tooth load the teeth allow at the pulley's speed
/// (`ratings.belt_tooth_load_n_cm` at `ratings.belt_tooth_load_rpm`, interpolated, held beyond the ends) times the width in
/// cm (`ratings.belt_width_mm` / 10) and the teeth in mesh (at most 12): `ratings.belt_teeth_in_mesh`, or with
/// effects.belt_wrap those of the smallest wrap over the pass (`MotorLoad::belt_wrap`), wrap r / `belt.tooth_pitch_m`
/// (wrap / (2 pi) of the pulley's 2 pi r / p teeth). The highest ratio
/// over the cycle, each speed band (`MotorLoad::gearbox_torque_by_speed`) at its upper speed, where the rating is lowest.
/// `None` where the machine reports no output torque or the config lacks one of these values, and for a motor that turns
/// no pulley (the machine reports neither its pulley's radial load nor its wrap: a screw, rack or linear motor in a
/// machine file whose other motors drive belts). A rating, not a life: no belt maker publishes a life in hours or load
/// cycles for these belts.
pub fn belt_tooth_utilisation(cfg: &Config, m: &MotorLoad) -> Result<Option<f64>, ConfigError> {
    let Ok(rpm) = cfg.f64s("ratings.belt_tooth_load_rpm") else { return Ok(None) };
    let get = |key: &str| cfg.f64(key).ok();
    let (Some(width), Some(teeth), Some(factor), Some(radius)) = (
        get("ratings.belt_width_mm"),
        get("ratings.belt_teeth_in_mesh"),
        get("ratings.belt_service_factor"),
        get("geometry.pulley_radius_m"),
    ) else {
        return Ok(None);
    };
    let load = cfg.f64s("ratings.belt_tooth_load_n_cm")?;
    if load.len() != rpm.len() || rpm.is_empty() || rpm.windows(2).any(|w| w[0] >= w[1]) {
        return Err(ConfigError::Invalid("ratings.belt_tooth_load_rpm".into()));
    }
    if m.gearbox_torque.is_none() || (m.radial_load_equivalent.is_none() && m.belt_wrap.is_none()) {
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
    let ratio = own_ratio(cfg, m);
    let teeth = match m.belt_wrap {
        Some(wrap) => wrap * radius / cfg.positive("belt.tooth_pitch_m")?,
        None => teeth,
    };
    let capacity = |n: f64| allowed(n) * width / 10.0 * teeth.min(12.0);
    let band_rpm = |i: usize| (i + 1) as f64 * SPEED_BAND * 30.0 / std::f64::consts::PI / ratio;
    Ok(Some(
        m.gearbox_torque_by_speed
            .iter()
            .enumerate()
            .map(|(i, torque)| torque / radius * factor / capacity(band_rpm(i)))
            .fold(0.0, f64::max),
    ))
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
/// A part by name (`[parts]`, `--motor`, `--drive`, ...) re-centres the bounds of the values it puts in: they are/// scaled by the part's value over the config's own (`catalog::set`), so that a band written for the config's motor and/// drive keeps its relative width around the named part's value. A list taken from a config before its parts were/// applied is read anew from the resolved config by the functions that run the bounds (`followed`).
pub fn bounds(cfg: &Config) -> Result<Vec<(String, f64, f64)>, ConfigError> {
    fn walk(
        path: &str,
        value: &toml::Value,
        cfg: &Config,
        out: &mut Vec<(String, f64, f64)>,
    ) -> Result<(), ConfigError> {
        if let Some(table) = value.as_table() {
            return table.iter().try_for_each(|(key, v)| {
                walk(&if path.is_empty() { key.clone() } else { format!("{path}.{key}") }, v, cfg, out)
            });
        }
        cfg.f64(path)?;
        // A value written as an integer is a count (pole pairs, teeth): its bounds are whole numbers too.
        let whole = |v: f64| !cfg.get(path).is_ok_and(toml::Value::is_integer) || v.fract() == 0.0;
        match cfg.f64s(&format!("bounds.{path}"))?[..] {
            [low, high] if low <= high && whole(low) && whole(high) => out.push((path.into(), low, high)),
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

/// `bounds` as the config resolved from `cfg` has them, where they are `cfg`'s own `[bounds]`: a part by name has
/// re-centred those of the values it set (see `bounds`). A bound given otherwise (the caller's own range), or one the
/// resolved config has none for, is kept.
pub(crate) fn followed(cfg: &Config, resolved: &Config, bounds: &[(String, f64, f64)]) -> Vec<(String, f64, f64)> {
    let own = |c: &Config, path: &str| match c.f64s(&format!("bounds.{path}")).as_deref() {
        Ok([low, high]) => Some((*low, *high)),
        _ => None,
    };
    bounds
        .iter()
        .map(|(path, low, high)| {
            let (low, high) = match (own(cfg, path), own(resolved, path)) {
                (Some(given), Some(now)) if given == (*low, *high) => now,
                _ => (*low, *high),
            };
            (path.clone(), low, high)
        })
        .collect()
}

/// Sets a bounded value in the type its key has. `set_text("{path} = {value:?}")` wrote a float, which an integer key
/// refused with a Type error that ended the whole band run. An integer key takes the value rounded (`bounds` keeps its
/// bounds whole; a sampled value between them is rounded to a count).
pub fn set_bounded(cfg: &mut Config, path: &str, value: f64) -> Result<(), ConfigError> {
    let value = if cfg.get(path)?.is_integer() {
        toml::Value::Integer(value.round() as i64)
    } else {
        toml::Value::Float(value)
    };
    cfg.set(path, value)
}

/// Values (by the start of their path) that only some effects use, and those effects: they change nothing while all
/// of them are off. None: a value no cycle uses, the discharge of a disabled drive's DC link. The parts warm with
/// effects.thermal_feedback too (`sim::heats`).
const PATH_EFFECTS: [(&str, &[&str]); 38] = [
    ("follower.roller_width_m", &["follower_lift_off"]),
    ("follower.contact_", &["follower_lift_off"]),
    ("follower.guide_friction_n", &["follower_lift_off"]),
    ("thermal.", &["part_heating", "thermal_feedback"]),
    ("motor.pole_pairs", &["field_weakening"]),
    ("motor.d_inductance_h", &["field_weakening"]),
    ("motor.winding_resistance_share", &["motor_thermal_nodes"]),
    ("motor.winding_time_constant_s", &["motor_thermal_nodes"]),
    ("ratings.drive_base_load_current_a", &["drive_overload"]),
    ("ratings.drive_overload_time_constant_s", &["drive_overload"]),
    ("ratings.drive_max_current_time_s", &["drive_overload"]),
    ("ratings.drive_chip_", &["drive_overload"]),
    ("motor.magnet_temperature_coefficient_1_k", &["magnet_temperature"]),
    ("motor.stall_current_a", &["iron_loss"]),
    ("ratings.braking_resistor_time_constant_s", &["braking_resistor_heating"]),
    ("ratings.braking_resistor_rated_rise_k", &["braking_resistor_heating"]),
    ("thermal_feedback.", &["thermal_feedback"]),
    ("belt.tooth_jump_factor", &["belt_tooth_jump"]),
    ("ratings.line_module_efficiency", &["dc_link"]),
    ("ratings.dc_link_discharge_", &[]),
    // The brakes' delays act only while the drives stop or start, which a cycle never does.
    ("brake.", &[]),
    ("motor.friction_breakaway_nm", &["motor_stiction"]),
    ("motor.friction_stribeck_rad_s", &["motor_stiction"]),
    ("slide.friction_breakaway_n", &["slide_presliding"]),
    ("slide.friction_stribeck_m_s", &["slide_presliding"]),
    ("slide.presliding_stiffness_n_m", &["slide_presliding"]),
    ("screw.friction_coefficient", &["screw_lead_efficiency"]),
    ("screw.fixed_bearing_at_m", &["screw_stiffness_with_position"]),
    ("gearbox.no_load_", &["gearbox_drag"]),
    ("gearbox.lost_motion_band_rad", &["gearbox_lost_motion"]),
    ("gearbox.hysteresis_rad", &["gearbox_lost_motion"]),
    // The knees and the upper stiffnesses; not stiffness_nm_rad, which every model uses.
    ("gearbox.stiffness_t", &["gearbox_stiffness_curve"]),
    ("gearbox.stiffness_k", &["gearbox_stiffness_curve"]),
    ("base.", &["base_compliance"]),
    ("link_bending.", &["link_bending"]),
    ("beam.", &["beam_sag"]),
    ("tool_flex.z_axis_bending_stiffness_nm2", &["z_axis_bending"]),
    ("mass.elbow_twist_inertia_kgm2", &["arm_tube_inertia"]),
];

/// Whether `path` is a value only an effect uses (`PATH_EFFECTS`), and that effect is off in `cfg`, or a value no cycle
/// uses: varying it would only run the cycle again for nothing.
fn effect_off(cfg: &Config, path: &str) -> bool {
    PATH_EFFECTS.iter().any(|(start, effects)| {
        path.starts_with(start) && !effects.iter().any(|effect| cfg.bool(&format!("effects.{effect}")).unwrap_or(false))
    })
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
fn judge_each(
    entry: &MachineEntry,
    count: usize,
    make: impl Fn(usize) -> Result<Config, ConfigError> + Sync,
) -> Vec<Result<Vec<Verdict>, ConfigError>> {
    on_all_cores(count, |run| make(run).and_then(|cfg| judged(entry, &cfg)))
}

/// The most threads `on_all_cores` starts; 0: one per processor.
static MAX_THREADS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

/// Limits the threads the analyses run their cycles on to `count` (0: one per processor, as by default). The results
/// do not depend on it, only the time they take.
pub fn set_max_threads(count: usize) {
    MAX_THREADS.store(count, std::sync::atomic::Ordering::Relaxed);
}

/// How many threads `on_all_cores` starts at most: one per processor, or `set_max_threads`.
fn threads() -> usize {
    match MAX_THREADS.load(std::sync::atomic::Ordering::Relaxed) {
        0 => std::thread::available_parallelism().map_or(1, |n| n.get()),
        count => count,
    }
}

/// `work(0..count)` on all processors, in the order of the index: for runs that do not depend on each other.
pub(crate) fn on_all_cores<T: Send>(count: usize, work: impl Fn(usize) -> T + Sync) -> Vec<T> {
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
    let threads = threads().min(count.max(1));
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
fn envelope(
    entry: &MachineEntry,
    cfg: &Config,
    count: usize,
    changed: impl Fn(usize) -> Result<Config, ConfigError> + Sync,
) -> Result<Vec<Band>, ConfigError> {
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
pub fn verdict_bands(
    entry: &MachineEntry,
    cfg: &Config,
    bounds: &[(String, f64, f64)],
) -> Result<Vec<Band>, ConfigError> {
    let resolved = &crate::catalog::resolve(entry, cfg)?;
    let (bounds, cfg) = (&followed(cfg, resolved, bounds), resolved);
    envelope(entry, cfg, 2 * bounds.len(), |run| {
        let ((path, low, high), mut changed) = (&bounds[run / 2], cfg.clone());
        set_bounded(&mut changed, path, if run % 2 == 0 { *low } else { *high })?;
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
pub fn verdict_bands_sampled(
    entry: &MachineEntry,
    cfg: &Config,
    bounds: &[(String, f64, f64)],
    samples: usize,
    seed: u64,
) -> Result<Vec<Band>, ConfigError> {
    if samples == 0 {
        return Err(ConfigError::Invalid("samples".into()));
    }
    let resolved = &crate::catalog::resolve(entry, cfg)?;
    let (bounds, cfg) = (&followed(cfg, resolved, bounds), resolved);
    let mut random = sim_core::testkit::Noise(seed);
    let columns: Vec<Vec<f64>> = bounds
        .iter()
        .map(|(_, low, high)| {
            let mut values: Vec<f64> = (0..samples)
                .map(|part| low + (high - low) * (part as f64 + random.uniform()) / samples as f64)
                .collect();
            for i in (1..samples).rev() {
                values.swap(i, (random.uniform() * (i + 1) as f64) as usize);
            }
            values
        })
        .collect();
    envelope(entry, cfg, samples, |run| {
        let mut changed = cfg.clone();
        for ((path, ..), values) in bounds.iter().zip(&columns) {
            set_bounded(&mut changed, path, values[run])?;
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
pub fn verdict_worst(
    entry: &MachineEntry,
    cfg: &Config,
    bounds: &[(String, f64, f64)],
) -> Result<(Vec<Worst>, usize), ConfigError> {
    verdict_worst_and_bands(entry, cfg, bounds).map(|(worst, _, runs)| (worst, runs))
}

/// `verdict_worst`, and `verdict_bands` of the same bounds from the runs the search starts with, which are those of the
/// bands (the config as it is, then each parameter at its low and at its high bound): the bands cost no cycle more.
pub fn verdict_worst_and_bands(
    entry: &MachineEntry,
    cfg: &Config,
    bounds: &[(String, f64, f64)],
) -> Result<(Vec<Worst>, Vec<Band>, usize), ConfigError> {
    worst_search(entry, cfg, bounds, &[], &|_| false)
}

/// `verdict_worst_and_bands`, started from the worst corners `from` found for another config with the same bounds (the
/// optimiser's previous step): per verdict, where `from` has one, its corner (a parameter it left as configured at the
/// sign corner's bound) is run beside the sign corner, and the worse of the two starts the passes. And the search ends
/// after the first batch of runs in which a verdict `stop` picks out came up: the worst it reports is then only what was
/// found so far (enough to say that a check fails).
fn worst_search(
    entry: &MachineEntry,
    cfg: &Config,
    bounds: &[(String, f64, f64)],
    from: &[Worst],
    stop: &(dyn Fn(&Verdict) -> bool + Sync),
) -> Result<(Vec<Worst>, Vec<Band>, usize), ConfigError> {
    let resolved = &crate::catalog::resolve(entry, cfg)?;
    let (bounds, cfg) = (&followed(cfg, resolved, bounds), resolved);
    let n = bounds.len();
    let set = |cfg: &mut Config, i: usize, high: bool| {
        set_bounded(cfg, &bounds[i].0, if high { bounds[i].2 } else { bounds[i].1 })
    };
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
    let bad = |run: &[Verdict], id: (&str, Option<usize>)| {
        run.iter().find(|v| (v.check, v.motor) == id).map_or(f64::NEG_INFINITY, badness)
    };
    let mut corners: Vec<Vec<bool>> = ids
        .iter()
        .map(|&id| (0..n).map(|i| bad(&alone[2 * i + 2], id) >= bad(&alone[2 * i + 1], id)).collect())
        .collect();
    let moves: Vec<Vec<usize>> = ids
        .iter()
        .map(|&id| (0..n).filter(|&i| bad(&alone[2 * i + 2], id) != bad(&alone[2 * i + 1], id)).collect())
        .collect();

    // The previous worst corners, where there are any: a parameter left as configured there takes the sign corner's bound.
    let warm: Vec<Option<Vec<bool>>> = ids
        .iter()
        .zip(&corners)
        .map(|(&id, sign)| {
            let w = from.iter().find(|w| (w.verdict.check, w.verdict.motor) == id && w.high.len() == n)?;
            Some(w.high.iter().zip(sign).map(|(h, s)| h.unwrap_or(*s)).collect())
        })
        .collect();

    let mut seen: std::collections::HashMap<Vec<bool>, Vec<Verdict>> = std::collections::HashMap::new();
    let mut runs = alone.len();
    let mut stopped = alone.iter().flatten().any(stop);
    // Runs the corners not run before; whether one of them has a verdict `stop` picks out.
    let mut run_corners = |wanted: Vec<Vec<bool>>,
                           seen: &mut std::collections::HashMap<Vec<bool>, Vec<Verdict>>|
     -> Result<bool, ConfigError> {
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
        let mut hit = false;
        for (corner, verdicts) in new.into_iter().zip(judged) {
            let verdicts = verdicts?;
            hit |= verdicts.iter().any(stop);
            seen.insert(corner, verdicts);
        }
        Ok(hit)
    };
    if !stopped {
        stopped = run_corners(corners.iter().cloned().chain(warm.iter().flatten().cloned()).collect(), &mut seen)?;
        for (k, warm) in warm.into_iter().enumerate() {
            if let Some(warm) = warm.filter(|w| bad(&seen[w], ids[k]) > bad(&seen[&corners[k]], ids[k])) {
                corners[k] = warm;
            }
        }
    }
    let flipped = |corner: &[bool], flips: &[usize]| {
        let mut out = corner.to_vec();
        for &i in flips {
            out[i] = !out[i];
        }
        out
    };
    for _ in 0..WORST_PASSES {
        if stopped {
            break;
        }
        let singles: Vec<Vec<bool>> = (0..ids.len())
            .flat_map(|k| moves[k].iter().map(|&i| flipped(&corners[k], &[i])).collect::<Vec<_>>())
            .collect();
        if run_corners(singles, &mut seen)? {
            break;
        }
        // Per verdict the flips that made it worse, and all of them together.
        let worse: Vec<Vec<usize>> = (0..ids.len())
            .map(|k| {
                moves[k]
                    .iter()
                    .copied()
                    .filter(|&i| bad(&seen[&flipped(&corners[k], &[i])], ids[k]) > bad(&seen[&corners[k]], ids[k]))
                    .collect()
            })
            .collect();
        stopped = run_corners(
            (0..ids.len()).filter(|&k| worse[k].len() > 1).map(|k| flipped(&corners[k], &worse[k])).collect(),
            &mut seen,
        )?;
        let mut changed = false;
        for k in 0..ids.len() {
            let candidates = worse[k]
                .iter()
                .map(|&i| flipped(&corners[k], &[i]))
                .chain((worse[k].len() > 1).then(|| flipped(&corners[k], &worse[k])));
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
    let alone_at = |run: usize| -> Vec<Option<bool>> {
        (0..n).map(|i| (run > 0 && (run - 1) / 2 == i).then_some(run.is_multiple_of(2))).collect()
    };
    let worst = ids
        .iter()
        .filter_map(|&id| {
            let corners = seen.iter().map(|(corner, run)| (corner.iter().map(|&h| Some(h)).collect::<Vec<_>>(), run));
            let candidates = corners.chain(alone.iter().enumerate().map(|(r, run)| (alone_at(r), run)));
            let (high, run) =
                candidates.max_by(|a, b| bad(a.1, id).total_cmp(&bad(b.1, id)).then_with(|| b.0.cmp(&a.0)))?;
            let verdict = run.iter().find(|v| (v.check, v.motor) == id)?.clone();
            Some(Worst { verdict, high })
        })
        .collect();
    Ok((worst, bands, runs))
}

/// A cycle made faster: the cycle before and after, the settings that changed (path, before, after) and
/// the number of cycles that were simulated on the way. Where the optimiser changed the cycle's geometry, `changes`
/// has the whole new cycle as the entry "cycle.segment" (TOML, for `Config::set_text`), and `lifts` says what changed.
/// `runs` counts the search's cycles, the sweep of `OptimiseOptions::pareto` among them; the bands' are in `robust`.
#[derive(Debug, Clone)]
pub struct Optimised {
    pub before: CycleLoad,
    pub after: CycleLoad,
    pub changes: Vec<(String, String, String)>,
    pub lifts: Vec<LiftChange>,
    pub runs: usize,
    /// With `OptimiseOptions::robust`: how the result stands in the error bands.
    pub robust: Option<Robust>,
    /// With `OptimiseOptions::pareto`: the settings run that passed every verdict and that no other one beats in cycle
    /// time, energy and peak utilisation at once, fastest first. Empty without.
    pub front: Vec<FrontPoint>,
}

/// What to optimise for: at most `max_runs` cycles in the search; `robust`: the result has to pass every verdict in the
/// error bands too (`verdict_bands`, one bound of `[bounds]` at a time), not only as configured; `pareto`: the
/// non-dominated set of cycle time, energy and peak utilisation (`Optimised::front`), with a sweep of slower settings;
/// `worst` (with `robust`): judged at the worst corner of all bounds at once (`verdict_worst`) instead of the bands.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OptimiseOptions {
    pub max_runs: usize,
    pub robust: bool,
    pub pareto: bool,
    pub worst: bool,
}

/// The fastest setting the search found on the verdicts as configured, judged in the error bands: its cycle time and
/// the checks it fails there (none where it passes); the checks the cycle as configured fails in the bands already,
/// which the result is not judged on (it cannot take back a margin the config does not have); how many accepted steps
/// the result went back from the fastest to the fastest one that passes the other checks in the bands; and the cycles
/// the bands took (2 n + 1 per setting judged, n the bounds). With `worst`, all of it at the worst corner instead: there
/// the search stops at the first batch of runs that fails a check judged on, so `nominal_fails` are the checks found
/// failing by then (not every one), and `runs` counts the worst-corner searches' cycles.
#[derive(Debug, Clone, PartialEq)]
pub struct Robust {
    pub nominal_duration: f64,
    pub nominal_fails: Vec<&'static str>,
    pub excused: Vec<&'static str>,
    pub steps_back: usize,
    pub runs: usize,
    pub worst: bool,
}

/// A setting on the energy-against-time front: cycle time (s), energy per cycle from the supply (`supply_energy`, J),
/// the highest utilisation of any motor's torque-speed curve (`MotorLoad::utilisation`), and the setting: factors on the
/// axes' speed, acceleration and jerk, corner blending and input shaping. The lifts are not shown.
#[derive(Debug, Clone, PartialEq)]
pub struct FrontPoint {
    pub duration: f64,
    pub energy: f64,
    pub utilisation: f64,
    pub factors: [f64; 3],
    pub blending: bool,
    pub shaping: bool,
}

/// The energy a cycle takes from the supply, J: what the drives drew less what they fed back, plus what the braking
/// resistor burned of what they fed back (with the DC link simulated; a shared one counted once). What the capacitors
/// hold more at the end than at the start counts as fed back. With the DC link simulated also what the bleeders took
/// out of it and the line module's (or an S210's rectifier's) own loss, `effects.dc_link`'s
/// `ratings.line_module_efficiency` eta both ways: it draws `drawn / eta` from the line for the `drawn` it puts into the
/// DC link and gives the mains `eta * fed` of the `fed` it takes out (`effects.regenerative_supply`), so it loses
/// `(1/eta - 1) drawn + (1 - eta) fed`; with the load's `line_module_energy = drawn + fed` and `mains_energy = eta * fed`,
/// `fed = mains_energy / eta`. The DC link's balance, drawn - fed = net + burned + bleeders (+ what the capacitors gain),
/// makes this the energy drawn from the line less what the mains got back. A shared link is counted once, with the
/// shared config's efficiency; a link of its own with its motor's (`catalog::motor_config`).
pub fn supply_energy(cfg: &Config, load: &CycleLoad) -> f64 {
    let net: f64 = load.motors.iter().map(|m| m.energy_in - m.energy_regenerated).sum();
    let shared = cfg.bool("effects.shared_dc_link").unwrap_or(false);
    // Every drive of a shared link reports the link's resistor and line module as its own.
    let links: Vec<&MotorLoad> =
        if shared { load.motors.iter().take(1).collect() } else { load.motors.iter().collect() };
    let burned: f64 = links.iter().map(|m| m.resistor_energy).sum();
    if !cfg.bool("effects.dc_link").unwrap_or(false) {
        return net + burned;
    }
    let bled: f64 = load.motors.iter().map(|m| m.discharge_energy).sum();
    let line: f64 = links
        .iter()
        .map(|m| {
            let own = if shared { None } else { crate::catalog::motor_config(cfg, &m.id).ok() };
            let eta = own.as_ref().unwrap_or(cfg).positive("ratings.line_module_efficiency").unwrap_or(1.0);
            let fed = m.mains_energy / eta;
            (1.0 / eta - 1.0) * (m.line_module_energy - fed) + (1.0 - eta) * fed
        })
        .sum();
    net + burned + bled + line
}

/// The points that no other one beats: none other is as good in all three figures and better in one. Fastest first;
/// a point that is there twice is kept once.
pub fn non_dominated(points: &[FrontPoint]) -> Vec<FrontPoint> {
    let figures: Vec<Vec<f64>> = points.iter().map(|p| vec![p.duration, p.energy, p.utilisation]).collect();
    let mut front: Vec<FrontPoint> = non_dominated_indices(&figures).into_iter().map(|i| points[i].clone()).collect();
    front.sort_by(|a, b| a.duration.total_cmp(&b.duration));
    front
}

/// The indices of the rows of figures (each the same number of figures, lower better) that no other row beats: none
/// other is as good in every figure and better in one. A row that is there twice is kept once, the first; in the order
/// of the rows. For the optimiser's front (`non_dominated`) and the catalogue search's (`search::front`).
pub fn non_dominated_indices(figures: &[Vec<f64>]) -> Vec<usize> {
    let beats = |a: &[f64], b: &[f64]| a.iter().zip(b).all(|(x, y)| x <= y) && a != b;
    let mut front: Vec<usize> = Vec::new();
    for (i, p) in figures.iter().enumerate() {
        if !figures.iter().any(|q| beats(q, p)) && !front.iter().any(|&k| figures[k] == *p) {
            front.push(i);
        }
    }
    front
}

/// A lift, traverse and lower of the cycle that the optimiser changed: the three segments of the configured cycle
/// (counted from 0), the height of the traverse (the last axis) before and after, and the via points the path is rounded
/// through, where it took them (then the three segments are one: a path up, through the via points and down, see
/// `control::via_pieces`): one over the middle or over the nearest obstacle, or two per obstacle in the way, the height
/// profile (`lift_profiles`).
#[derive(Debug, Clone, PartialEq)]
pub struct LiftChange {
    pub segments: [usize; 3],
    pub before: f64,
    pub after: f64,
    pub via: Option<Vec<Vec<f64>>>,
    /// Where the lift's corners are rounded instead (`corners_m` of the segment, `control::lines_through`): the corner
    /// tolerance, its via points then none but the two corners at the lift's height.
    pub corners: Option<f64>,
}

/// A lift, traverse and lower of a configured cycle (`lifts`): segment `lift` goes from `from` straight along the last
/// axis to `height`, `traverse` along the other axes at that height, `lower` straight along the last axis to `to`.
/// `up` is the side (+1 or -1 on the last axis) both ends lie away from; `lowest` the height nearest them that keeps
/// `profile.min_clearance_m` to the farther of the two. `needed` is the lowest height at which the traverse keeps that
/// clearance to the obstacles as well (`over_obstacles`; `lowest` without obstacles in the way, the height as configured
/// where none clears them), `over` the pose on the traverse nearest to an obstacle there, where one raised it. `payload`:
/// whether the payload is on during the three segments.
struct Lift {
    segments: [usize; 3],
    from: Vec<f64>,
    to: Vec<f64>,
    height: f64,
    up: f64,
    lowest: f64,
    needed: f64,
    over: Option<Vec<f64>>,
    profile: Option<Vec<Vec<f64>>>,
    payload: bool,
}

/// The lifts of a cycle: three segments in a row (round the end of the cycle too) of which the first moves along the
/// last axis only, away from both the start and the end of the three, the second along the other axes only, at that
/// height, and the third along the last axis only. The first two neither dwell nor change the payload, none has via
/// points. A lift already as low as the clearance allows is a lift all the same (it can still be rounded).
fn lifts(cfg: &Config, clearance: f64) -> Result<Vec<Lift>, ConfigError> {
    let segments = cfg.array("cycle.segment")?;
    let n = segments.len();
    let target = |i: usize| -> Option<Vec<f64>> {
        segments[i]
            .get("target")?
            .as_array()?
            .iter()
            .map(|v| v.as_float().or_else(|| v.as_integer().map(|i| i as f64)))
            .collect()
    };
    let plain = |i: usize| segments[i].get("via").is_none();
    let quiet = |i: usize| {
        segments[i].get("dwell_s").and_then(|d| d.as_float().or_else(|| d.as_integer().map(|i| i as f64))) == Some(0.0)
            && segments[i].get("payload") == segments[(i + n - 1) % n].get("payload")
    };
    let mut out = Vec::new();
    if n < 3 {
        return Ok(out);
    }
    for a in 0..n {
        let (b, c) = ((a + 1) % n, (a + 2) % n);
        let (Some(from), Some(lift), Some(traverse), Some(to)) =
            (target((a + n - 1) % n), target(a), target(b), target(c))
        else {
            continue;
        };
        let z = from.len().wrapping_sub(1);
        if from.len() < 2 || [&lift, &traverse, &to].iter().any(|p| p.len() != from.len()) {
            continue;
        }
        let along_z = |p: &[f64], q: &[f64]| p[..z] == q[..z] && p[z] != q[z];
        let height = lift[z];
        let up = (height - from[z]).signum();
        if !(along_z(&from, &lift)
            && traverse[z] == height
            && traverse[..z] != lift[..z]
            && along_z(&traverse, &to)
            && (height - to[z]).signum() == up)
        {
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
            // The segments run with the payload as the one before left it, which `quiet` keeps.
            let payload = segments[a].get("payload").and_then(|p| p.as_bool()).unwrap_or(true);
            out.push(Lift {
                segments: [a, b, c],
                from,
                to,
                height,
                up,
                lowest,
                needed: lowest,
                over: None,
                profile: None,
                payload,
            });
        }
    }
    Ok(out)
}

/// The lowest height of a lift's traverse at which the machine's collision bodies keep `clearance` to the obstacles, at
/// rest at every millimetre of it (`collision::check_path`), with the pose nearest to an obstacle there where one raised
/// it above `lowest`. From `lowest`, the height is moved each time by what the clearance lacks (up) or has too much
/// (down, never below `lowest`) until it is the clearance to 1 nm. The bodies move with the last axis, so the clearance
/// changes by no more than the height: a box across the traverse is cleared in one step, to the clearance exactly. `None`
/// where 40 steps within the axis's range find no height that clears (a box beside the traverse that reaches above it).
fn over_obstacles(
    machine: &mut dyn sim_core::Machine,
    obstacles: &[sim_core::Obstacle],
    lift: &Lift,
    clearance: f64,
    axis: &sim_core::AxisInfo,
) -> Option<(f64, Option<Vec<f64>>)> {
    let z = lift.from.len() - 1;
    machine.set_payload_attached(lift.payload);
    let machine: &dyn sim_core::Machine = machine;
    let gap_at = |height: f64| {
        let at = |p: &[f64]| -> Vec<f64> { p[..z].iter().copied().chain([height]).collect() };
        let mut check = crate::collision::PlanCheck::default();
        crate::collision::check_path(machine, obstacles, &[at(&lift.from), at(&lift.to)], &mut check);
        (check.clearance, check.nearest)
    };
    let (mut height, mut best) = (lift.lowest, None::<(f64, Option<Vec<f64>>)>);
    for _ in 0..40 {
        let (gap, nearest) = gap_at(height);
        if gap >= clearance - 1e-9 && best.as_ref().is_none_or(|(b, _)| lift.up * (height - b) < 0.0) {
            best = Some((height, nearest));
        }
        // How far beyond `lowest` the next height is (on the side `up`): what it is now, plus what the clearance lacks.
        let next = lift.lowest + lift.up * (lift.up * (height - lift.lowest) + clearance - gap).max(0.0);
        if !gap.is_finite()
            || (gap - clearance).abs() <= 1e-9
            || next == height
            || !(axis.min..=axis.max).contains(&next)
        {
            break;
        }
        height = next;
    }
    best.map(|(height, nearest)| (height, if height == lift.lowest { None } else { nearest }))
}

/// The height profile of a lift over the obstacles in its way: per obstacle that raises the traverse (`over_obstacles`
/// with it alone), two via points at its own height, where the traverse at `lowest` begins and ends to come nearer to it
/// than `clearance` (looked at every millimetre, the collision bodies at rest with the payload as the lift carries it).
/// Obstacles whose stretches overlap are passed as one, at the higher of their heights. `None` with fewer than two such
/// stretches (one box: the lift at `needed`, as without a profile), or where an obstacle cannot be cleared.
fn profile(
    machine: &mut dyn sim_core::Machine,
    obstacles: &[sim_core::Obstacle],
    lift: &Lift,
    clearance: f64,
    axis: &sim_core::AxisInfo,
) -> Option<Vec<Vec<f64>>> {
    let z = lift.from.len() - 1;
    let at = |s: f64, height: f64| -> Vec<f64> {
        (0..z).map(|i| lift.from[i] + s * (lift.to[i] - lift.from[i])).chain([height]).collect()
    };
    let width = (0..z).map(|i| (lift.to[i] - lift.from[i]).powi(2)).sum::<f64>().sqrt();
    let steps = (width / 1e-3).ceil().max(1.0) as usize;
    let mut stretches: Vec<(f64, f64, f64)> = Vec::new();
    for one in obstacles.chunks(1) {
        let (height, over) = over_obstacles(machine, one, lift, clearance, axis)?;
        if over.is_none() {
            continue;
        }
        machine.set_payload_attached(lift.payload);
        let near = (0..=steps).map(|k| k as f64 / steps as f64).filter(|&s| {
            let mut check = crate::collision::PlanCheck::default();
            crate::collision::check_path(&*machine, one, &[at(s, lift.lowest)], &mut check);
            check.clearance < clearance - 1e-9
        });
        let (first, last) = near.fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), s| (a.min(s), b.max(s)));
        if first <= last {
            stretches.push((first, last, height));
        }
    }
    stretches.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut merged: Vec<(f64, f64, f64)> = Vec::new();
    for (first, last, height) in stretches {
        match merged.last_mut() {
            Some(m) if first <= m.1 => {
                (m.1, m.2) = (m.1.max(last), if lift.up > 0.0 { m.2.max(height) } else { m.2.min(height) })
            }
            _ => merged.push((first, last, height)),
        }
    }
    (merged.len() >= 2)
        .then(|| merged.iter().flat_map(|&(first, last, height)| [at(first, height), at(last, height)]).collect())
}

/// The points with the last axis moved by `by`.
fn raised(points: &[Vec<f64>], by: f64) -> Vec<Vec<f64>> {
    points
        .iter()
        .map(|p| p.iter().enumerate().map(|(i, x)| if i + 1 == p.len() { x + by } else { *x }).collect())
        .collect()
}

/// A lift's height profile (`profile`) and how far its points have to be raised (on the side `up`) for the planner's
/// spline through them to keep the clearance: the spline sags between the points, so they are raised by what the
/// planned path lacks and 1 um, three times at most. `None` where there is no profile or it still does not clear.
/// `planned`: the axes, the profile cycle and the path tolerance the planner times it with.
fn smoothed_profile(
    machine: &mut dyn sim_core::Machine,
    obstacles: &[sim_core::Obstacle],
    lift: &Lift,
    clearance: f64,
    planned: (&[sim_core::AxisInfo], f64, f64),
) -> Option<(Vec<Vec<f64>>, f64)> {
    let (axes, cycle, tolerance) = planned;
    let points = profile(machine, obstacles, lift, clearance, &axes[lift.from.len() - 1])?;
    let mut by = 0.0;
    for _ in 0..4 {
        let via = raised(&points, lift.up * by);
        let at = (lift.lowest, None);
        let gap = planned_clearance(axes, cycle, tolerance, lift, at, &via, (&mut *machine, obstacles, clearance))?;
        if gap >= clearance - 1e-9 {
            return Some((points, by));
        }
        by += clearance - gap + 1e-6;
    }
    None
}

/// Per lift of the cycle with two or more obstacles in its way (as the optimiser finds them, `optimise_cycle_with`): its
/// three segments, its height profile as `profile` builds it, and how far the optimiser raises those points for the
/// planned spline through them to keep the clearance (`smoothed_profile`). Empty without obstacles; lifts with fewer
/// than two obstacles in the way, or whose profile does not clear, are left out.
pub fn lift_profiles(entry: &MachineEntry, cfg: &Config) -> Result<Vec<([usize; 3], Vec<Vec<f64>>, f64)>, ConfigError> {
    let mut probe = Sim::new(entry, cfg)?;
    let clearance = cfg.non_negative("profile.min_clearance_m")?;
    let (cycle, tolerance) = (cfg.positive("profile.cycle_s")?, cfg.non_negative("profile.path_tolerance_m")?);
    let (axes, obstacles) = (probe.machine.axes().to_vec(), probe.obstacles.clone());
    let mut out = Vec::new();
    for lift in lifts(cfg, clearance)?.iter().filter(|_| !obstacles.is_empty()) {
        let planned = (axes.as_slice(), cycle, tolerance);
        if let Some((points, by)) = smoothed_profile(probe.machine.as_mut(), &obstacles, lift, clearance, planned) {
            out.push((lift.segments, points, by));
        }
    }
    Ok(out)
}

/// The cycle with each lift at `heights` and, where `vias` has some, rounded through those via points, or where
/// `corners` has a tolerance, its two corners rounded within it (see `LiftChange`).
fn cycle_with(
    cfg: &Config,
    lifts: &[Lift],
    heights: &[f64],
    vias: &[Option<Vec<Vec<f64>>>],
    corners: &[Option<f64>],
) -> Result<toml::Value, ConfigError> {
    let mut segments = cfg.array("cycle.segment")?.clone();
    let number_list = |p: &[f64]| toml::Value::Array(p.iter().map(|x| toml::Value::Float(*x)).collect());
    let mut dropped = Vec::new();
    for (((lift, &height), via), corner) in lifts.iter().zip(heights).zip(vias).zip(corners) {
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
                    let pose: Vec<f64> =
                        old.iter().filter_map(|v| v.as_float().or_else(|| v.as_integer().map(|i| i as f64))).collect();
                    if let Some(t) = segments[i].as_table_mut() {
                        t.insert("target".into(), number_list(&at_height(&pose)));
                    }
                }
            }
            Some(points) => {
                let via = toml::Value::Array(
                    std::iter::once(at_height(&lift.from))
                        .chain(points.iter().cloned())
                        .chain([at_height(&lift.to)])
                        .map(|p| number_list(&p))
                        .collect(),
                );
                if let Some(t) = segments[c].as_table_mut() {
                    t.insert("via".into(), via);
                    if let Some(corner) = corner {
                        t.insert("corners_m".into(), toml::Value::Float(*corner));
                    }
                }
                dropped.extend([a, b]);
            }
        }
    }
    let kept = segments.into_iter().enumerate().filter(|(i, _)| !dropped.contains(i)).map(|(_, s)| s).collect();
    Ok(toml::Value::Array(kept))
}

/// Whether the path of a lift rounded through `via` keeps its clearance: every planned pose that lies between the two
/// ends along the other axes is at `lowest` or beyond, less the path tolerance the transition curves may use; and where
/// there are obstacles, the machine's collision bodies keep `clearance` to them along the whole path, at rest at each
/// planned pose (`collision::check_path`).
fn clears(
    axes: &[sim_core::AxisInfo],
    cycle: f64,
    tolerance: f64,
    lift: &Lift,
    height: (f64, Option<f64>),
    via: &[Vec<f64>],
    bodies: (&mut dyn sim_core::Machine, &[sim_core::Obstacle], f64),
) -> bool {
    let clearance = bodies.2;
    planned_clearance(axes, cycle, tolerance, lift, height, via, bodies).is_some_and(|gap| gap >= clearance - 1e-9)
}

/// The least clearance to the obstacles of the planned path of a lift rounded through `via` (infinite without
/// obstacles); `None` where it is not planned or comes nearer to pick and place than `lowest` (see `clears`). `height`:
/// the lift's, and the corner tolerance where its corners are rounded (lines through the via points,
/// `control::lines_through`) instead of a curve through them.
fn planned_clearance(
    axes: &[sim_core::AxisInfo],
    cycle: f64,
    tolerance: f64,
    lift: &Lift,
    (height, corner): (f64, Option<f64>),
    via: &[Vec<f64>],
    bodies: (&mut dyn sim_core::Machine, &[sim_core::Obstacle], f64),
) -> Option<f64> {
    let points = lift_path(axes, cycle, tolerance, lift, (height, corner), via)?;
    let (machine, obstacles, _) = bodies;
    if obstacles.is_empty() {
        return Some(f64::INFINITY);
    }
    machine.set_payload_attached(lift.payload);
    let mut check = crate::collision::PlanCheck::default();
    let poses: Vec<Vec<f64>> =
        std::iter::once(lift.from.clone()).chain(points.into_iter().map(|(_, pose)| pose)).collect();
    crate::collision::check_path(&*machine, obstacles, &poses, &mut check);
    Some(check.clearance)
}

/// The planned path of a lift rounded through `via` (or with its corners rounded, see `planned_clearance`), `None`
/// where it is not planned or comes nearer to pick and place than `lowest` (see `clears`).
fn lift_path(
    axes: &[sim_core::AxisInfo],
    cycle: f64,
    tolerance: f64,
    lift: &Lift,
    (height, corner): (f64, Option<f64>),
    via: &[Vec<f64>],
) -> Option<Vec<(f64, Vec<f64>)>> {
    let z = lift.from.len() - 1;
    let at_height = |p: &[f64]| {
        let mut p = p.to_vec();
        p[z] = height;
        p
    };
    let via: Vec<Vec<f64>> =
        std::iter::once(at_height(&lift.from)).chain(via.iter().cloned()).chain([at_height(&lift.to)]).collect();
    let (pieces, rounded) = match corner {
        Some(corner) => {
            (crate::control::lines_through(&via, &lift.to), crate::path::Tolerance { path: tolerance, corner })
        }
        None => (crate::control::via_pieces(&via, &lift.to), tolerance.into()),
    };
    let points = crate::path::plan(axes, &lift.from, &pieces, cycle, rounded, crate::path::ARC_SLACK).ok()?;
    let across: Vec<f64> = (0..z).map(|i| lift.to[i] - lift.from[i]).collect();
    let width = across.iter().map(|d| d * d).sum::<f64>();
    let above = points.iter().all(|(_, pose)| {
        let along = (0..z).map(|i| (pose[i] - lift.from[i]) * across[i]).sum::<f64>() / width;
        along <= 0.0 || along >= 1.0 || lift.up * (pose[z] - lift.lowest) >= -tolerance - 1e-9
    });
    above.then_some(points)
}

/// Shortens the cycle of a config without breaking a verdict. Tries corner blending, then raises speed,
/// acceleration and jerk of all axes together in steps for as long as the cycle gets shorter and every
/// verdict of `verdicts` still passes, among them the settling tolerance, so a faster move that leaves
/// the tool swinging at the pick does not count. Input shaping makes no cycle shorter by itself (every
/// stop waits one period of the shaper), so it is tried where it can help: a faster setting that fails
/// on "settling" alone is run again with the shaper, and with the shaper following the pose where the
/// machine reports a resonance, and is taken if it then passes and is shorter. A setting with the shaper that still
/// fails on "settling" alone is run again with a second shaper convolved with it, at `profile.shaper2_frequency_hz`
/// where the config has one, else at the machine's second mode at the start pose (`Machine::modes`), else at the one
/// resonance it reports there where that is more than 10 % off the first shaper's frequency (no second shaper is tried
/// where none of these is). Where a shaper is on already,
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
/// is an improvement. Judged on the verdicts as configured, at most `max_runs` cycles: `optimise_cycle_with` for the
/// error bands and the energy front.
pub fn optimise_cycle(entry: &MachineEntry, cfg: &Config, max_runs: usize) -> Result<Optimised, ConfigError> {
    optimise_cycle_with(entry, cfg, &OptimiseOptions { max_runs, robust: false, pareto: false, worst: false })
}

/// The fastest of the steps 1 to `top` that passes, judged by `pass` from the fastest down and the first that passes
/// taken; `None` where none does (step 0, the cycle as configured, passes by definition and is not judged). Pass and fail
/// need not follow the steps (a faster step may pass where a slower one fails), so no step above the result is left out:
/// the cost is one judgement per step above the result and the result's own, never more than a bisection followed by
/// judging the steps above its result would take.
fn fastest_passing<T, E>(
    top: usize,
    mut pass: impl FnMut(usize) -> Result<Option<T>, E>,
) -> Result<Option<(usize, T)>, E> {
    for k in (1..=top).rev() {
        if let Some(passed) = pass(k)? {
            return Ok(Some((k, passed)));
        }
    }
    Ok(None)
}

/// The second input shaper's frequency the cycle optimiser tries (Hz) with the plan at `pose`: `profile.shaper2_frequency_hz`
/// where set (above 0); otherwise the second of the modes the shaper acts on (`Machine::sway_modes`: a crane's upper swing
/// with the hook block, not the load's bounce on its rope), or, where the machine tells only one, its resonance where
/// that is more than 10 % off `profile.shaper_frequency_hz`. `None`: no second shaper to try.
pub fn second_shaper_frequency(machine: &dyn Machine, pose: &[f64], cfg: &Config) -> Result<Option<f64>, ConfigError> {
    let (first, configured) =
        (cfg.positive("profile.shaper_frequency_hz")?, cfg.non_negative("profile.shaper2_frequency_hz")?);
    if configured > 0.0 {
        return Ok(Some(configured));
    }
    let second = machine.sway_modes(pose).get(1).map(|mode| mode.hz);
    Ok(second.or(machine.resonance_hz(pose).filter(|f| (f / first - 1.0).abs() > 0.1)))
}

/// `optimise_cycle`, and with `options.robust` a result that passes in the error bands: the fastest setting found is
/// judged with `verdict_bands` over the config's `[bounds]`; where it fails, the accepted steps of the search are gone
/// back one at a time, each judged in the bands, until one passes (2 n + 1 cycles each, n the bounds). With
/// `options.pareto`, every setting run that passes every verdict, and a sweep of slower ones from the result (its
/// acceleration and jerk, and all three limits, at 0.8, 0.6 and 0.4 times: six cycles more), give the front of cycle
/// time against energy and peak utilisation (`Optimised::front`).
///
/// Where the config has obstacles, a lift is lowered no further than its traverse keeps the clearance to them
/// (`over_obstacles`): a box under the traverse puts the lift at its top plus the clearance plus how far the machine's
/// lowest collision body reaches below the tool. A lift that an obstacle raises is tried rounded through a via point
/// over that obstacle as well, its ends at the clearance above pick and place; every via point's path is checked
/// against the obstacles before it runs.
///
/// With `options.worst` the accepted steps are judged at the worst corner from the fastest down (`fastest_passing`).
pub fn optimise_cycle_with(
    entry: &MachineEntry,
    cfg: &Config,
    options: &OptimiseOptions,
) -> Result<Optimised, ConfigError> {
    let max_runs = options.max_runs;
    let mut probe = Sim::new(entry, cfg)?;
    let clearance = cfg.non_negative("profile.min_clearance_m")?;
    // A conveyor's cycle picks from the belt (`[conveyor]`), and a path runs in place of the cycle (`[path]`):
    // `[[cycle.segment]]` does not run, so its lifts are left.
    let conveyor = probe.control.conveyor().is_some() || probe.path.is_some();
    let mut lifts = if conveyor { Vec::new() } else { lifts(cfg, clearance)? };
    // How far a path may leave the given one: the larger of the path and the corner tolerance as configured. With a
    // `[path]` the search tries both at it (transition curves and corners rounded within it, `path::geometry`).
    let corner_s = cfg.non_negative("profile.corner_tolerance_m")?;
    let allowed = if probe.path.is_some() { cfg.non_negative("profile.path_tolerance_m")?.max(corner_s) } else { 0.0 };
    let (cycle_s, tolerance) = (cfg.positive("profile.cycle_s")?, cfg.non_negative("profile.path_tolerance_m")?);
    let axis_infos = probe.machine.axes().to_vec();
    let obstacles = probe.obstacles.clone();
    if !obstacles.is_empty() {
        for lift in &mut lifts {
            let axis = &axis_infos[lift.from.len() - 1];
            (lift.needed, lift.over) = over_obstacles(probe.machine.as_mut(), &obstacles, lift, clearance, axis)
                .unwrap_or((lift.height, None));
            let planned = (axis_infos.as_slice(), cycle_s, tolerance);
            lift.profile = smoothed_profile(probe.machine.as_mut(), &obstacles, lift, clearance, planned)
                .map(|(points, by)| raised(&points, lift.up * by));
        }
    }
    let resonance = probe.machine.resonance_hz(probe.control.planned_pos());
    let configured = cfg.non_negative("profile.shaper2_frequency_hz")?;
    let second_frequency = second_shaper_frequency(probe.machine.as_ref(), probe.control.planned_pos(), cfg)?;
    // In the axis's own unit: `axis.x.max_velocity_m_s`, `axis.c.max_velocity_rad_s` (with metres alone, a machine with an
    // axis in rad could not be built and its cycle was said to fail a verdict).
    let limits = ["max_velocity_{unit}_s", "max_acceleration_{unit}_s2", "max_jerk_{unit}_s3"];
    // A candidate: the factor on each of the three limits, the switches, and per lift its height and via point.
    #[derive(Clone, PartialEq)]
    struct Setting {
        factors: [f64; 3],
        blending: bool,
        shaping: bool,
        follows: bool,
        second: bool,
        heights: Vec<f64>,
        vias: Vec<Option<Vec<Vec<f64>>>>,
        /// Per lift whose corners are rounded (its via points the two corners alone): the corner tolerance.
        corners: Vec<Option<f64>>,
        /// `profile.path_tolerance_m` and `corner_tolerance_m`.
        tolerances: [f64; 2],
    }
    let build = |setting: &Setting| -> Result<Config, ConfigError> {
        let mut changed = cfg.clone();
        let [path, corner] = setting.tolerances;
        changed.set_text(&format!("profile.path_tolerance_m = {path:?}"))?;
        changed.set_text(&format!("profile.corner_tolerance_m = {corner:?}"))?;
        for axis in &axis_infos {
            for (limit, factor) in limits.iter().zip(setting.factors) {
                let path = format!("axis.{}.{}", axis.id, limit.replace("{unit}", axis.unit));
                changed.set_text(&format!("{path} = {:?}", cfg.positive(&path)? * factor))?;
            }
        }
        changed.set_text(&format!("effects.corner_blending = {}", setting.blending))?;
        changed.set_text(&format!("effects.input_shaping = {}", setting.shaping))?;
        changed.set_text(&format!("effects.shaper_follows_pose = {}", setting.follows))?;
        changed.set_text(&format!(
            "profile.shaper2_frequency_hz = {:?}",
            if setting.second { second_frequency.unwrap_or(0.0) } else { 0.0 }
        ))?;
        if setting.heights.iter().zip(&lifts).any(|(h, lift)| *h != lift.height)
            || setting.vias.iter().any(Option::is_some)
        {
            changed
                .set("cycle.segment", cycle_with(cfg, &lifts, &setting.heights, &setting.vias, &setting.corners)?)?;
        }
        Ok(changed)
    };
    let runs = std::cell::Cell::new(0usize);
    // Every setting run that passed every verdict, for the front.
    let passed = std::cell::RefCell::new(Vec::<FrontPoint>::new());
    // The cycle of a setting, the checks it fails, and its point of the front where it fails none; if it runs. Touches
    // nothing shared, so that settings can run side by side.
    let evaluate = |setting: &Setting| -> Option<(CycleLoad, Vec<&'static str>, Option<FrontPoint>)> {
        let changed = build(setting).ok()?;
        let mut sim = Sim::new(entry, &changed).ok()?;
        let load = analyse_cycle_with(&mut sim, false).ok()?;
        let failed: Vec<&'static str> =
            verdicts(&changed, &load).ok()?.iter().filter(|v| !v.ok).map(|v| v.check).collect();
        let point = failed.is_empty().then(|| {
            let utilisation = load.motors.iter().fold(0.0, |a: f64, m| a.max(m.utilisation));
            let (factors, blending, shaping) = (setting.factors, setting.blending, setting.shaping);
            FrontPoint {
                duration: load.duration,
                energy: supply_energy(&changed, &load),
                utilisation,
                factors,
                blending,
                shaping,
            }
        });
        Some((load, failed, point))
    };
    // A setting's run counted, and its point put on the front.
    let count = |result: Option<(CycleLoad, Vec<&'static str>, Option<FrontPoint>)>| {
        runs.set(runs.get() + 1);
        let (load, failed, point) = result?;
        passed.borrow_mut().extend(point);
        Some((load, failed))
    };
    let judged = |setting: &Setting| -> Option<(CycleLoad, Vec<&'static str>)> { count(evaluate(setting)) };
    let start = Setting {
        factors: [1.0; 3],
        blending: cfg.bool("effects.corner_blending")?,
        shaping: cfg.bool("effects.input_shaping")?,
        follows: cfg.bool("effects.shaper_follows_pose")?,
        second: configured > 0.0,
        heights: lifts.iter().map(|lift| lift.height).collect(),
        vias: vec![None; lifts.len()],
        corners: vec![None; lifts.len()],
        tolerances: [tolerance, corner_s],
    };
    let before = match judged(&start) {
        Some((load, failed)) if failed.is_empty() => load,
        Some((_, failed)) => {
            return Err(ConfigError::Invalid(format!("the cycle as configured fails {}", failed.join(", "))));
        }
        // Said by what stops it: the config (its own error), or the cycle that does not run (its key).
        None => {
            let changed = build(&start)?;
            let why =
                |e: CommandError| ConfigError::Invalid(format!("the cycle as configured does not run: {}", e.key));
            verdicts(&changed, &analyse_cycle_with(&mut Sim::new(entry, &changed)?, false).map_err(why)?)?;
            return Err(ConfigError::Invalid("the cycle as configured cannot be judged".into()));
        }
    };
    // The setting with the lifts the obstacles raised (where it changed them) higher by `lack`, their via points with them.
    let raised_by = |setting: &Setting, lack: f64| {
        let mut higher = setting.clone();
        for (k, lift) in
            lifts.iter().enumerate().filter(|(k, lift)| lift.over.is_some() && setting.heights[*k] != lift.height)
        {
            higher.heights[k] += lift.up * lack;
            for z in higher.vias[k].iter_mut().flatten().filter_map(|point| point.last_mut()) {
                *z += lift.up * lack;
            }
        }
        higher
    };
    let (mut best, mut best_load) = (start.clone(), before.clone());
    // The settings the search took, one after the other, each faster than the one before: where the robust result steps
    // back to.
    let mut accepted = vec![(start.clone(), before.clone())];
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
        if lifts.iter().zip(&best.heights).filter(|(lift, h)| **h != lift.needed).count() > 1 {
            candidates.push(Setting { heights: lifts.iter().map(|lift| lift.needed).collect(), ..best.clone() });
        }
        for (k, lift) in lifts.iter().enumerate().filter(|(k, lift)| best.heights[*k] != lift.needed) {
            candidates.push(lower(&best, k, lift.needed));
            candidates.push(lower(&best, k, 0.5 * (best.heights[k] + lift.needed)));
        }
        candidates.push(Setting { blending: !best.blending, ..best.clone() });
        // A path's corners rounded and its transition curves laid within the deviation the config allows.
        for k in (0..2).filter(|&k| best.tolerances[k] < allowed) {
            let mut tolerances = best.tolerances;
            tolerances[k] = allowed;
            candidates.push(Setting { tolerances, ..best.clone() });
        }
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
            let with_via = |via: Option<Vec<Vec<f64>>>| {
                let mut out = best.clone();
                (out.vias[k], out.corners[k]) = (via, None);
                out
            };
            match &best.vias[k] {
                None => {
                    // The lift's two corners rounded, so that the curve starts at `lowest` on the way up and ends there
                    // on the way down: for a right angle it starts 16 / 3 of the tolerance before the corner
                    // (`path::round_corners`), so the tolerance is 3 / 16 of the height above `lowest`.
                    let above = lift.up * (best.heights[k] - lift.lowest);
                    if above > 0.0 {
                        let mut rounded = with_via(Some(Vec::new()));
                        rounded.corners[k] = Some(3.0 * above / 16.0);
                        candidates.push(rounded);
                    }
                    for above in [0.5, 0.25, 0.125] {
                        let mut via: Vec<f64> = (0..z).map(|i| 0.5 * (lift.from[i] + lift.to[i])).collect();
                        via.push(best.heights[k] + lift.up * above * width);
                        candidates.push(with_via(Some(vec![via])));
                    }
                    // Over the obstacle that raised the lift, the ends as low as pick and place allow.
                    for above in [0.0, 0.125] {
                        if let Some(over) = &lift.over {
                            let via =
                                over[..z].iter().copied().chain([lift.needed + lift.up * above * width]).collect();
                            candidates.push(Setting {
                                heights: lower(&best, k, lift.lowest).heights,
                                ..with_via(Some(vec![via]))
                            });
                        }
                    }
                    // The height profile over the obstacles in the way, the ends as low as pick and place allow.
                    if let Some(points) = &lift.profile {
                        candidates.push(Setting {
                            heights: lower(&best, k, lift.lowest).heights,
                            ..with_via(Some(points.clone()))
                        });
                    }
                }
                // One via point is moved; a profile is kept or dropped.
                Some(points) => {
                    for via in points.iter().filter(|_| points.len() == 1) {
                        for step in [0.125, 1.0 / 32.0] {
                            for axis in 0..=z {
                                for sign in [-1.0, 1.0] {
                                    let mut moved = via.clone();
                                    moved[axis] += sign * step * width;
                                    candidates.push(with_via(Some(vec![moved])));
                                }
                            }
                        }
                    }
                    candidates.push(with_via(None));
                }
            }
        }
        // A via point whose path would come too near to pick and place, or to an obstacle, is not run.
        let machine = probe.machine.as_mut();
        candidates.retain(|c| {
            lifts.iter().enumerate().all(|(k, lift)| {
                c.vias[k].as_ref().is_none_or(|via| {
                    clears(
                        &axis_infos,
                        cycle_s,
                        tolerance,
                        lift,
                        (c.heights[k], c.corners[k]),
                        via,
                        (&mut *machine, &obstacles, clearance),
                    )
                })
            })
        });
        // A candidate's tries, at most `budget`, up to the first that is faster than `best` by more than 0.1 ms: its
        // own run, and where that only leaves the tool swinging its tries with the shaper right after it. They depend on
        // the candidate's own runs alone, so the candidates' tries can run side by side.
        let tries_of = |candidate: Setting, best: f64, budget: usize| {
            let mut done = Vec::new();
            let mut tries = std::collections::VecDeque::from([candidate]);
            let mut raised = 0;
            while let Some(setting) = tries.pop_front() {
                if done.len() >= budget {
                    break;
                }
                let result = evaluate(&setting);
                let Some((load, failed, _)) = &result else {
                    done.push((setting, result));
                    continue;
                };
                if failed.is_empty() && load.duration < best - 1e-4 {
                    done.push((setting, result));
                    break;
                }
                if !failed.is_empty() && failed.iter().all(|check| *check == "settling" || *check == "settling-angle") {
                    if !setting.shaping {
                        tries.push_back(Setting { shaping: true, follows: false, ..setting.clone() });
                        if resonance.is_some() {
                            tries.push_back(Setting { shaping: true, follows: true, ..setting.clone() });
                        }
                    } else if second_frequency.is_some() && !setting.second {
                        tries.push_back(Setting { second: true, ..setting.clone() });
                    }
                }
                // The lifts placed over the obstacles keep the clearance at rest; the run sways by some hundredths of a
                // millimetre (0.05 mm on the gantry). A setting that fails on that alone, by less than a millimetre, is
                // run again (twice at most) with those lifts higher by what the run lacked and 1 um.
                let lack = load.clearance.map_or(f64::INFINITY, |run| clearance - run + 1e-6);
                if *failed == ["collision"] && raised < 2 && lack < 1e-3 {
                    let higher = raised_by(&setting, lack);
                    if higher != setting {
                        raised += 1;
                        tries.push_back(higher);
                    }
                }
                done.push((setting, result));
            }
            done
        };
        // The candidates are taken in order, the first faster one accepted: a batch at a time, their tries then counted
        // in order as if they had run one after the other, those after the accepted one (or beyond `max_runs`)
        // dropped. The first candidate runs alone (it is often taken, and runs beside it would only slow it down), then
        // the batch doubles while candidates fail, up to one per processor. The result is the same with any batch.
        let (mut from, mut batch) = (0, 1);
        'candidates: while from < candidates.len() {
            let chunk = &candidates[from..(from + batch).min(candidates.len())];
            (from, batch) = (from + chunk.len(), (2 * batch).min(threads().max(1)));
            let budget = max_runs.saturating_sub(runs.get());
            let best_duration = best_load.duration;
            let tried = on_all_cores(chunk.len(), |i| tries_of(chunk[i].clone(), best_duration, budget));
            for (setting, result) in tried.into_iter().flatten() {
                if runs.get() >= max_runs {
                    break 'candidates;
                }
                let Some((load, failed)) = count(result) else { continue };
                if failed.is_empty() && load.duration < best_load.duration - 1e-4 {
                    accepted.push((setting.clone(), load.clone()));
                    (best, best_load, improved) = (setting, load, true);
                    break 'candidates;
                }
            }
        }
    }
    let mut robust = None;
    if options.robust && options.worst {
        let (mut corner_runs, last) = (0, accepted.len() - 1);
        // The checks a setting fails at the worst corner, the least clearance to the obstacles there, and its worst
        // corners. With `ignore`, the search stops at the first batch of runs that fails a check not in it.
        let mut at_worst = |setting: &Setting,
                            ignore: Option<&[&'static str]>,
                            from: &[Worst]|
         -> Result<(Vec<&'static str>, f64, Vec<Worst>), ConfigError> {
            let changed = build(setting)?;
            let stop = |v: &Verdict| ignore.is_some_and(|ignore| !v.ok && !ignore.contains(&v.check));
            let (worst, _, runs) = worst_search(entry, &changed, &bounds(&changed)?, from, &stop)?;
            corner_runs += runs;
            let least =
                worst.iter().find(|w| w.verdict.check == "collision").map_or(f64::INFINITY, |w| w.verdict.value);
            let mut out: Vec<&'static str> = worst.iter().filter(|w| !w.verdict.ok).map(|w| w.verdict.check).collect();
            out.sort();
            out.dedup();
            Ok((out, least, worst))
        };
        let (nominal_fails, _, mut from) = at_worst(&best, Some(&[]), &[])?;
        let excused = match (nominal_fails.is_empty(), last) {
            (true, _) => Vec::new(),
            (false, 0) => nominal_fails.clone(),
            (false, _) => at_worst(&accepted[0].0, None, &from)?.0,
        };
        let judged_on = |checks: &[&'static str]| -> Vec<&'static str> {
            checks.iter().copied().filter(|c| !excused.contains(c)).collect()
        };
        // Whether accepted step k passes what it is judged on at the worst corner, or with its lifts over the obstacles
        // higher by what the corner lacked (less than a millimetre); and its worst corners, where the next search starts.
        let mut pass = |k: usize, from: &[Worst]| -> Result<(Option<(Setting, CycleLoad)>, Vec<Worst>), ConfigError> {
            let (fails, least, worst) = at_worst(&accepted[k].0, Some(&excused), from)?;
            let failing = judged_on(&fails);
            if failing.is_empty() {
                return Ok((Some(accepted[k].clone()), worst));
            }
            if failing == ["collision"] && clearance - least < 1e-3 {
                let higher = raised_by(&accepted[k].0, clearance - least + 1e-6);
                if higher != accepted[k].0
                    && let Some((load, _)) = judged(&higher).filter(|(_, failed)| failed.is_empty())
                    && judged_on(&at_worst(&higher, Some(&excused), &worst)?.0).is_empty()
                {
                    return Ok((Some((higher, load)), worst));
                }
            }
            Ok((None, worst))
        };
        // The accepted steps from the fastest down, the first that passes what it is judged on taken (`fastest_passing`);
        // step 0 (the cycle as configured) passes by definition. Each search starts from the worst corners of the one
        // before. The fastest is not judged again where the nominal search already failed it on a check not excused.
        let (lo, result) = if nominal_fails.is_empty() {
            (last, (best.clone(), best_load.clone()))
        } else {
            let top =
                if matches!(judged_on(&nominal_fails)[..], [] | ["collision"]) { last } else { last.saturating_sub(1) };
            let found = fastest_passing(top, |k| {
                let (passed, worst) = pass(k, &from)?;
                from = worst;
                Ok(passed)
            })?;
            found.unwrap_or_else(|| (0, accepted[0].clone()))
        };
        let (nominal_duration, steps_back) = (best_load.duration, last - lo);
        (best, best_load) = result;
        robust = Some(Robust { nominal_duration, nominal_fails, excused, steps_back, runs: corner_runs, worst: true });
    } else if options.robust {
        let mut band_runs = 0;
        // The checks a setting fails in the bands, each once, and the least clearance to the obstacles there.
        let mut fails = |setting: &Setting| -> Result<(Vec<&'static str>, f64), ConfigError> {
            let changed = build(setting)?;
            let bounded = bounds(&changed)?;
            band_runs += 2 * bounded.len() + 1;
            let bands = verdict_bands(entry, &changed, &bounded)?;
            let least = bands.iter().find(|b| b.verdict.check == "collision").map_or(f64::INFINITY, |b| b.low);
            let mut out: Vec<&'static str> = bands.into_iter().filter(|b| !b.ok).map(|b| b.verdict.check).collect();
            out.sort();
            out.dedup();
            Ok((out, least))
        };
        let last = accepted.len() - 1;
        let (nominal_fails, mut least) = fails(&best)?;
        let excused = match (nominal_fails.is_empty(), last) {
            (true, _) => Vec::new(),
            (false, 0) => nominal_fails.clone(),
            (false, _) => fails(&accepted[0].0)?.0,
        };
        let judged_on = |checks: &[&'static str]| -> Vec<&'static str> {
            checks.iter().copied().filter(|c| !excused.contains(c)).collect()
        };
        // Pass or fail in the bands need not follow the cycle time, so the steps are judged from the fastest down, and
        // no further than the first that passes; the cycle as configured passes what it is judged on by definition.
        let (mut k, mut failing) = (last, judged_on(&nominal_fails));
        let result = loop {
            if failing.is_empty() {
                break accepted[k].clone();
            }
            // A step whose lifts over the obstacles the bounds sway nearer than the clearance, by less than a millimetre:
            // those lifts higher by that and 1 um, judged as written and in the bands.
            if failing == ["collision"] && clearance - least < 1e-3 {
                let higher = raised_by(&accepted[k].0, clearance - least + 1e-6);
                if higher != accepted[k].0
                    && let Some((load, _)) = judged(&higher).filter(|(_, failed)| failed.is_empty())
                    && judged_on(&fails(&higher)?.0).is_empty()
                {
                    break (higher, load);
                }
            }
            if k <= 1 {
                k = 0;
                break accepted[0].clone();
            }
            k -= 1;
            let (checks, at_k) = fails(&accepted[k].0)?;
            (failing, least) = (judged_on(&checks), at_k);
        };
        let (nominal_duration, steps_back) = (best_load.duration, last - k);
        (best, best_load) = result;
        robust = Some(Robust { nominal_duration, nominal_fails, excused, steps_back, runs: band_runs, worst: false });
    }
    let mut front = Vec::new();
    if options.pareto {
        let [v, a, j] = best.factors;
        let slower: Vec<Setting> = [0.8, 0.6, 0.4]
            .into_iter()
            .flat_map(|f| [[v, a * f, j * f], [v * f, a * f, j * f]])
            .map(|factors| Setting { factors, ..best.clone() })
            .collect();
        // Side by side, counted in their order.
        for result in on_all_cores(slower.len(), |i| evaluate(&slower[i])) {
            count(result);
        }
        front = non_dominated(&passed.borrow());
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
        .map(|(k, lift)| LiftChange {
            segments: lift.segments,
            before: lift.height,
            after: best.heights[k],
            via: best.vias[k].clone().filter(|_| best.corners[k].is_none()),
            corners: best.corners[k],
        })
        .collect();
    Ok(Optimised { before, after: best_load, changes, lifts, runs: runs.get(), robust, front })
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
            load.motors
                .iter()
                .zip(&nominal.motors)
                .map(|(m, n)| change(pick(m), pick(n)))
                .fold(0.0, |a: f64, b| if b.abs() > a.abs() { b } else { a })
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
/// so which are worth measuring. A parameter of value 0 without bounds (nothing to scale), one that is not a number, whose changed value the machine
/// refuses, or that only an effect that is off uses (see `bounds`), is left out, and so are the analysis' own choices
/// in `NOT_MACHINE_DATA`.
pub fn sensitivity(
    entry: &MachineEntry,
    cfg: &Config,
    paths: &[String],
    delta: f64,
) -> Result<(CycleLoad, Vec<Sensitivity>), ConfigError> {
    let cfg = &crate::catalog::resolve(entry, cfg)?;
    let configured = bounds(cfg)?;
    let run = |cfg: &Config| -> Option<CycleLoad> { analyse_cycle_with(&mut Sim::new(entry, cfg).ok()?, false).ok() };
    // (path, value, low, high) of each parameter that is varied; the runs are independent and share the processors.
    let varied: Vec<(&String, f64, (f64, f64))> = paths
        .iter()
        .filter(|path| !effect_off(cfg, path) && !NOT_MACHINE_DATA.contains(&path.as_str()))
        .filter_map(|path| {
            let value = cfg.f64(path).ok()?;
            let bound = configured.iter().find(|b| b.0 == *path).map(|b| (b.1, b.2));
            // A value of 0 scaled by 1 -+ delta stays 0: without a range of its own it is not varied (left out).
            if bound.is_none() && value == 0.0 {
                return None;
            }
            Some((path, value, bound.unwrap_or((value * (1.0 - delta), value * (1.0 + delta)))))
        })
        .collect();
    // Run 0 is the config as it is, then each parameter at its low and at its high value.
    let mut runs = on_all_cores(2 * varied.len() + 1, |k| {
        if k == 0 {
            return run(cfg);
        }
        let (path, _, bounds) = varied[(k - 1) / 2];
        let mut changed = cfg.clone();
        set_bounded(&mut changed, path, if k % 2 == 1 { bounds.0 } else { bounds.1 }).ok()?;
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
/// peaks, the load swinging against the held motor as a dip. Each frequency starts from the same rest, on a machine
/// of its own, so they share the processors.
pub fn frequency_sweep(
    entry: &MachineEntry,
    cfg: &Config,
    motor: usize,
    frequencies: &[f64],
    amplitude: f64,
) -> Result<Vec<SweepPoint>, ConfigError> {
    let cfg = &crate::catalog::resolve(entry, cfg)?;
    let (dt, substeps) = (cfg.positive("sim.dt_s")?, cfg.positive("sim.substeps")? as u32);
    // The machine at rest at its start pose under the holding torques.
    let at_rest = || -> Result<_, ConfigError> {
        let machine = (entry.build)(cfg)?;
        let motors = machine.motors().len();
        if motor >= motors {
            return Err(ConfigError::Invalid("motor".into()));
        }
        let (mut start, mut hold) = (Vec::new(), vec![0.0; motors]);
        machine.equilibrium(&crate::sim::start_pose(cfg, machine.as_ref())?, &mut start, &mut hold);
        Ok((machine, start, hold))
    };
    at_rest()?;
    check_frequencies(frequencies)?;
    on_all_cores(frequencies.len(), |i| {
        let (frequency, (machine, start, hold)) = (frequencies[i], at_rest()?);
        let motors = hold.len();
        let mut rk = Rk4::new(start.len());
        let (mut pos, mut speed) = (vec![0.0; motors], vec![0.0; motors]);
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
        Ok(SweepPoint { frequency, gain: a.hypot(b) / amplitude, phase_deg: b.atan2(a).to_degrees() })
    })
    .into_iter()
    .collect()
}

/// A response is measured over whole periods: at 0 Hz that never ends, and below it means nothing.
pub(crate) fn check_frequencies(frequencies: &[f64]) -> Result<(), ConfigError> {
    if frequencies.iter().all(|f| f.is_finite() && *f > 0.0) {
        Ok(())
    } else {
        Err(ConfigError::Invalid("frequency".into()))
    }
}

/// The modes the machine reports (`Machine::modes`) at `points` places spread evenly over the range of axis `axis`, both
/// ends included, the other axes where `pose` has them: the resonances over a stroke, to choose shaper frequencies by
/// (`sim-cli modes`, the app's plot of them). Each place with its modes, which may be fewer at one place than at another.
pub fn modes_along(machine: &dyn Machine, pose: &[f64], axis: usize, points: usize) -> Vec<(f64, Vec<Mode>)> {
    let info = &machine.axes()[axis];
    (0..points)
        .map(|i| {
            let mut at = pose.to_vec();
            at[axis] = info.min + (info.max - info.min) * i as f64 / (points.max(2) - 1) as f64;
            (at[axis], machine.modes(&at))
        })
        .collect()
}

/// The small-amplitude modes (`Machine::modes_small_amplitude`) at the places of `modes_along`'s `rows`, or `None` where
/// they are the large-amplitude ones everywhere (no gearbox hysteresis): the front ends' second column.
pub fn small_amplitude_along(
    machine: &dyn Machine,
    pose: &[f64],
    axis: usize,
    rows: &[(f64, Vec<Mode>)],
) -> Option<Vec<Vec<Mode>>> {
    let small: Vec<Vec<Mode>> = rows
        .iter()
        .map(|(position, _)| {
            let mut at = pose.to_vec();
            at[axis] = *position;
            machine.modes_small_amplitude(&at)
        })
        .collect();
    let same = |a: &[Mode], b: &[Mode]| a.len() == b.len() && a.iter().zip(b).all(|(a, b)| a.hz == b.hz);
    (!rows.iter().zip(&small).all(|((_, large), small)| same(large, small))).then_some(small)
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
pub fn step_response(
    entry: &MachineEntry,
    cfg: &Config,
    axis: usize,
    size: f64,
    seconds: f64,
) -> Result<StepResponse, ConfigError> {
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
/// setpoint, measured at the tool. Gain is tool amplitude over setpoint amplitude. Each frequency is measured on a
/// simulation of its own, so they share the processors.
pub fn closed_loop_response(
    entry: &MachineEntry,
    cfg: &Config,
    axis: usize,
    frequencies: &[f64],
    amplitude: f64,
) -> Result<Vec<SweepPoint>, ConfigError> {
    check_frequencies(frequencies)?;
    on_all_cores(frequencies.len(), |i| {
        let frequency = frequencies[i];
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
        Ok(SweepPoint { frequency, gain: a.hypot(b) / amplitude, phase_deg: b.atan2(a).to_degrees() })
    })
    .into_iter()
    .collect()
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
    /// The mechanics from the motor's air-gap torque to its speed in the same measurement, (real, imaginary part) in
    /// rad/s per N m: the plant the loop works on, without the current loop, filters and delays. NaN where L is.
    pub plant: [f64; 2],
}

/// The open-loop response of the velocity loop of one motor, measured on the closed loop as a drive's own
/// measurement function does: a sine torque d is added to what the loop asks for (u), before the limit, with the
/// machine holding its start pose. The torque that reaches the motor is e = u + d, the loop answers u = -L e, so
/// L = -U / E and S = E / D at the frequency. Everything between the two is in L: velocity filter, PI, torque
/// filter, delays, current loop, the elastic mechanics, and the position loop around it. Sampled at the velocity
/// cycle, over whole periods: at least 0.1 s to settle and 0.1 s to measure.
///
/// Measured with `effects.gearbox_efficiency` off: at standstill the efficiency's fade with the transmitted power
/// (`Gearbox::loss_factor`) would act as a large damper on the small-signal torque and hide the loop (a crane hoist read
/// no gain crossover); the small-signal loop is that of the lossless gearbox.
pub fn velocity_loop_response(
    entry: &MachineEntry,
    cfg: &Config,
    motor: usize,
    frequencies: &[f64],
    amplitude: f64,
) -> Result<Vec<LoopPoint>, ConfigError> {
    check_frequencies(frequencies)?;
    let cfg = &lossless(cfg)?;
    frequencies.iter().map(|&frequency| loop_point(settled(entry, cfg, 0)?.0, motor, frequency, amplitude)).collect()
}

/// The config a velocity loop is measured with (`velocity_loop_response`): effects.gearbox_efficiency off.
fn lossless(cfg: &Config) -> Result<Config, ConfigError> {
    let mut lossless = cfg.clone();
    if lossless.get("effects.gearbox_efficiency").is_ok() {
        lossless.set_text("effects.gearbox_efficiency = false")?;
    }
    Ok(lossless)
}

/// One point of `velocity_loop_response`, measured on `sim` settled at its start pose.
fn loop_point(mut sim: Sim, motor: usize, frequency: f64, amplitude: f64) -> Result<LoopPoint, ConfigError> {
    if motor >= sim.control.drives.len() {
        return Err(ConfigError::Invalid("motor".into()));
    }
    let every = (sim.control.drive_params().velocity_cycle / sim.dt).round() as u64;
    let (omega, t0, period) = (std::f64::consts::TAU * frequency, sim.time(), 1.0 / frequency);
    let whole = |seconds: f64| ((seconds / period).ceil().max(5.0) * period / sim.dt).round() as u64;
    let (settle_steps, total) = (whole(0.1), whole(0.1) + whole(0.1));
    let (mut u_ab, mut e_ab, mut samples) = ([0.0; 2], [0.0; 2], 0.0);
    let (mut torque_ab, mut speed_ab) = ([0.0; 2], [0.0; 2]);
    let (mut seen, mut limited, mut pos, mut speed) =
        (Vec::new(), false, vec![0.0; sim.control.drives.len()], vec![0.0; sim.control.drives.len()]);
    for step in 0..total {
        let due = sim.step.is_multiple_of(every);
        let (s, c) = (omega * (sim.time() - t0)).sin_cos();
        if due {
            sim.control.drives[motor].torque_injection = amplitude * s;
        }
        sim.step();
        if due {
            // What reaches the motor clipped by the torque limit is no longer the linear loop being measured.
            sim.machine.motor_state(&sim.state, &mut pos, &mut speed);
            let (p, drive) = (sim.control.drive_params_of(motor), &sim.control.drives[motor]);
            let (low, high) = p.torque_range_at(speed[motor], drive.max_voltage(p));
            let e = drive.torque_demand + drive.torque_injection;
            limited |= !(low < e && e < high);
            if step >= settle_steps {
                let u = drive.torque_demand;
                (u_ab[0], u_ab[1], e_ab[0], e_ab[1]) =
                    (u_ab[0] + u * s, u_ab[1] + u * c, e_ab[0] + e * s, e_ab[1] + e * c);
                samples += 1.0;
                seen.push((s, c, e));
                let (t, v) = (drive.torque, speed[motor]);
                (torque_ab[0], torque_ab[1], speed_ab[0], speed_ab[1]) =
                    (torque_ab[0] + t * s, torque_ab[1] + t * c, speed_ab[0] + v * s, speed_ab[1] + v * c);
            }
        }
    }
    // Phasors (sine part, cosine part) of u and e; L = -U / E.
    let (u, e) =
        ([2.0 * u_ab[0] / samples, 2.0 * u_ab[1] / samples], [2.0 * e_ab[0] / samples, 2.0 * e_ab[1] / samples]);
    // What of e is not the sine of the injection and not a constant: a loop that moves on its own (unstable, or in a
    // limit cycle) shows here. More of it than the injection itself, and the point is not a measurement of L.
    let mean = seen.iter().map(|(_, _, x)| x).sum::<f64>() / samples;
    let rest = (seen.iter().map(|(s, c, x)| (x - mean - e[0] * s - e[1] * c).powi(2)).sum::<f64>() / samples).sqrt();
    let e2 = e[0] * e[0] + e[1] * e[1];
    let (re, im) = if limited || !(rest < amplitude) {
        (f64::NAN, f64::NAN)
    } else {
        (-(u[0] * e[0] + u[1] * e[1]) / e2, -(u[1] * e[0] - u[0] * e[1]) / e2)
    };
    // Plant = speed / torque, phasors as (sine, cosine) parts; NaN with L.
    let (t, v) = (torque_ab, speed_ab);
    let t2 = (t[0] * t[0] + t[1] * t[1]) * if re.is_nan() { f64::NAN } else { 1.0 };
    let plant = [(v[0] * t[0] + v[1] * t[1]) / t2, (v[1] * t[0] - v[0] * t[1]) / t2];
    Ok(LoopPoint { frequency, re, im, sensitivity: e2.sqrt() / amplitude, plant })
}

/// What an open-loop response says about the stability of the loop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LoopMargins {
    /// The gain crossover (|L| = 1) with the smallest phase margin; NaN without one.
    pub crossover_hz: f64,
    /// 180 degrees plus the phase of L at that crossover, the phase unwrapped along the frequency axis and taken
    /// between -270 and 90 degrees at the first point; NaN without a crossover in the measured band (not measurable,
    /// not infinite: nothing is known beyond the band). Negative: the loop is unstable.
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
    /// The loop hammers across its gearbox's play (`backlash_hammering`), which the response does not show; `None`
    /// from `loop_margins` alone, `velocity_loop_margins` runs the check.
    pub hammering: Option<Hammering>,
    /// With a gearbox's play in the loop: by how much the gain may rise before the loop hammers across it, in dB, the
    /// describing function's margin (`play_margin`). NaN without play acting on the loop and from `loop_margins` alone;
    /// negative: the loop crosses -1 at some amplitude of the swing across the play.
    pub play_margin_db: f64,
}

impl LoopMargins {
    /// A valid measurement, both margins positive (a phase margin that is NaN has no crossover to judge), and no
    /// hammering across a play: stable. The margin across the play (`play_margin_db`) is told, not judged: the time run
    /// is the verdict.
    pub fn stable(&self) -> bool {
        self.valid && !(self.phase_margin_deg <= 0.0) && self.gain_margin_db > 0.0 && self.hammering.is_none()
    }
}

/// Margins from an open-loop response, between neighbouring points interpolated linearly in log |L| and in the
/// unwrapped phase. Points too far apart (more than 180 degrees of phase between them) can miss a sharp resonance.
/// A point without a measurement (NaN) makes the margins not valid (and NaN).
pub fn loop_margins(points: &[LoopPoint]) -> LoopMargins {
    let measured = |p: &&LoopPoint| p.re.is_finite() && p.im.is_finite();
    let mut m = LoopMargins {
        crossover_hz: f64::NAN,
        phase_margin_deg: f64::NAN,
        gain_margin_db: f64::INFINITY,
        valid: points.iter().all(|p| measured(&p)),
        peak_sensitivity: 0.0,
        peak_frequency_hz: f64::NAN,
        hammering: None,
        play_margin_db: f64::NAN,
    };
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
    let crossovers: Vec<(usize, f64)> = (1..points.len())
        .filter(|&i| gain[i - 1].signum() != gain[i].signum())
        .map(|i| (i, gain[i - 1] / (gain[i - 1] - gain[i])))
        .collect();
    // The turn is fixed by the phase at the first point, taken in (-270, 90] (a loop with up to three integrators, or
    // a lead, there). Not at the first crossover: one rising out of an anti-resonance has a phase lead above 0 (the
    // arm 2 of a five-bar laid flat, +8 degrees at 18 Hz) and turned every phase after it by -360 degrees.
    let shift = phase.first().map_or(0.0, |p| -360.0 * ((p - 90.0) / 360.0).ceil());
    phase.iter_mut().for_each(|p| *p += shift);
    for &(i, share) in &crossovers {
        let margin = 180.0 + phase[i - 1] + (phase[i] - phase[i - 1]) * share;
        if !(margin >= m.phase_margin_deg) {
            (m.phase_margin_deg, m.crossover_hz) =
                (margin, points[i - 1].frequency * (points[i].frequency / points[i - 1].frequency).powf(share));
        }
    }
    // Crossings of the negative real axis (the phase through an odd multiple of -180 degrees), in frequency order,
    // from a phase of 0 at the gain of the first point: below the measured range each integrator has turned the
    // phase by -90 degrees from there (the small circle of the Nyquist contour around s = 0). Beyond -1 (|L| > 1) a
    // falling crossing is an encirclement of -1 and a later rising one beyond -1 undoes it (Nyquist: the loop is
    // stable when none is left). The gain margin is the worst falling crossing not undone: negative where |L| > 1.
    let (mut open, mut worst) = (Vec::new(), f64::INFINITY);
    let samples: Vec<(f64, f64)> = points
        .first()
        .map(|_| (0.0, gain[0]))
        .into_iter()
        .chain(phase.iter().copied().zip(gain.iter().copied()))
        .collect();
    for w in samples.windows(2) {
        let ((pa, ga), (pb, gb)) = (w[0], w[1]);
        let at = |level: f64| ga + (gb - ga) * (pa - level) / (pa - pb);
        let mut level = -180.0 + 360.0 * ((pa + 180.0) / 360.0).floor();
        while pb < level && level <= pa {
            let g = at(level);
            if g > 0.0 {
                open.push(g)
            } else {
                worst = worst.min(-g)
            }
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

/// The lowest frequency `velocity_loop_responses` follows a loop down to when its gain crossover lies below
/// `loop_frequencies` (a loop with less band than this is no servo loop to judge).
pub const LOOP_FLOOR_HZ: f64 = 1.0;

/// Frequencies of the velocity-loop measurement of `velocity_loop_margins`: 10 Hz to
/// a quarter of the velocity loop's sampling rate (2 kHz at 125 us; above it a period has fewer than four samples), 60 points.
pub fn loop_frequencies(cfg: &Config) -> Result<Vec<f64>, ConfigError> {
    Ok(log_spaced(10.0, 0.25 / cfg.positive("drive.velocity_cycle_s")?, 60))
}

/// The amplitude of the test torque: a tenth of the rated torque of the motor `id` (its own `[drive.<id>]`, not the
/// shared one).
pub fn loop_amplitude(entry: &MachineEntry, cfg: &Config, id: &str) -> Result<f64, ConfigError> {
    let own = crate::catalog::motor_config(&crate::catalog::resolve(entry, cfg)?, id)?;
    Ok(0.1 * own.positive("drive.rated_torque_nm")?)
}

/// Where neighbouring points of a response differ by more than this turn (degrees) or this change of magnitude (dB),
/// a sharp resonance may lie between them: `refinement` measures their middle too. A first-order lag turns by at
/// most 5 degrees between two of the 60 points of `loop_frequencies` (9.4 % apart), the velocity loop's delays by
/// about 17 degrees at 2 kHz; a resonance with 1 % damping turns by 180 degrees within 2 % of its frequency.
pub const REFINE_TURN_DEG: f64 = 30.0;
pub const REFINE_GAIN_DB: f64 = 3.0;
/// Points closer than this ratio of frequencies are not split again (0.1 %, a twentieth of the half-power width of a
/// mode with 1 % damping).
pub const REFINE_CLOSEST: f64 = 1.001;
/// At most this many rounds of splitting: 9.4 % halved eight times is 0.04 %.
pub const REFINE_ROUNDS: usize = 8;

/// The frequencies to measure next between neighbours of a response, `points` (frequency, complex value) in
/// frequency order: the geometric middle of each two that differ by more than `REFINE_TURN_DEG` or `REFINE_GAIN_DB`
/// and are further apart than `REFINE_CLOSEST`. Points with a value that is not a number are not split.
pub fn refinement(points: &[(f64, [f64; 2])]) -> Vec<f64> {
    points
        .windows(2)
        .filter_map(|w| {
            let ((fa, a), (fb, b)) = (w[0], w[1]);
            // The turn is the angle of b / a, the change of magnitude that of |b| / |a|.
            let turn = (a[0] * b[1] - a[1] * b[0]).atan2(a[0] * b[0] + a[1] * b[1]).to_degrees().abs();
            let change = 20.0 * (b[0].hypot(b[1]) / a[0].hypot(a[1])).log10().abs();
            ((turn > REFINE_TURN_DEG || change > REFINE_GAIN_DB) && fb / fa > REFINE_CLOSEST).then(|| (fa * fb).sqrt())
        })
        .collect()
}

/// A long measurement's progress, read from another thread: measurements done, measurements planned so far (the
/// refinement adds to them), and a switch that stops it (`velocity_loop_responses` then answers `cancelled`).
#[derive(Debug, Default)]
pub struct Progress {
    pub done: std::sync::atomic::AtomicUsize,
    pub total: std::sync::atomic::AtomicUsize,
    pub cancel: std::sync::atomic::AtomicBool,
}

/// The key of the error a cancelled measurement answers with (`ConfigError::Invalid`).
pub const CANCELLED: &str = "cancelled";

/// Per config of `configs` (of the same machine) per motor the open-loop response of its velocity loop
/// (`velocity_loop_response`), in frequency order: at `loop_frequencies` and the machine's resonances at the start pose
/// (`Machine::modes`) of the first config, below the band down to the gain crossover where it lies there (to
/// `LOOP_FLOOR_HZ`), and where the loop gain or the plant changes fast between two points at
/// their middle (`refinement`), in up to `REFINE_ROUNDS` rounds. Every point is measured on a simulation of its own:
/// they share the processors. `progress` counts them and can stop the measurement.
pub fn velocity_loop_responses(
    entry: &MachineEntry,
    configs: &[Config],
    progress: &Progress,
) -> Result<Vec<Vec<Vec<LoopPoint>>>, ConfigError> {
    use std::sync::atomic::Ordering::Relaxed;
    let first = &crate::catalog::resolve(entry, configs.first().ok_or_else(|| ConfigError::Invalid("config".into()))?)?;
    let mut frequencies = loop_frequencies(first)?;
    let spacing = frequencies[1] / frequencies[0];
    let machine = (entry.build)(first)?;
    let amplitudes =
        machine.motors().iter().map(|m| loop_amplitude(entry, first, m.id)).collect::<Result<Vec<_>, _>>()?;
    let range = frequencies[0]..=frequencies[frequencies.len() - 1];
    frequencies.extend(machine.modes(&first.f64s("start.pose")?).iter().map(|m| m.hz).filter(|f| range.contains(f)));
    let motors = machine.motors().len();
    // Each run settled once, each point started from a copy of it (`Sim::snapshot`): the same start, bit for bit, as
    // settling it anew, without the settling's share of the sweep's time.
    let lossless = configs.iter().map(lossless).collect::<Result<Vec<_>, _>>()?;
    let starts = on_all_cores(configs.len(), |run| settled(entry, &lossless[run], 0).map(|(sim, _)| sim.snapshot()))
        .into_iter()
        .collect::<Result<Vec<_>, _>>()?;
    let measure = |jobs: &[(usize, usize, f64)]| -> Result<Vec<LoopPoint>, ConfigError> {
        progress.total.fetch_add(jobs.len(), Relaxed);
        on_all_cores(jobs.len(), |j| {
            if progress.cancel.load(Relaxed) {
                return Err(ConfigError::Invalid(CANCELLED.into()));
            }
            let (run, motor, frequency) = jobs[j];
            check_frequencies(&[frequency])?;
            let mut sim = Sim::new(entry, &lossless[run])?;
            sim.restore(&starts[run])?;
            let point = loop_point(sim, motor, frequency, amplitudes[motor]);
            progress.done.fetch_add(1, Relaxed);
            point
        })
        .into_iter()
        .collect()
    };
    let mut curves = vec![vec![Vec::<LoopPoint>::new(); motors]; configs.len()];
    let mut jobs = Vec::new();
    for run in 0..configs.len() {
        for motor in 0..motors {
            jobs.extend(frequencies.iter().map(|&f| (run, motor, f)));
        }
    }
    for round in 0..=REFINE_ROUNDS {
        for (&(run, motor, _), point) in jobs.iter().zip(measure(&jobs)?) {
            curves[run][motor].push(point);
        }
        // Below the band, once: where |L| < 1 at the lowest point the integrators put the gain crossover lower, and
        // the loop is followed down by the band's own spacing (its points stay where they were) until |L| > 1 or
        // `LOOP_FLOOR_HZ`. A loop with its crossover in the band is measured as before.
        if round == 0 {
            loop {
                jobs.clear();
                for (run, per_motor) in curves.iter().enumerate() {
                    for (motor, curve) in per_motor.iter().enumerate() {
                        let low = curve.iter().min_by(|a, b| a.frequency.total_cmp(&b.frequency)).unwrap();
                        if low.re.hypot(low.im) < 1.0 && low.frequency / spacing >= LOOP_FLOOR_HZ {
                            jobs.push((run, motor, low.frequency / spacing));
                        }
                    }
                }
                if jobs.is_empty() {
                    break;
                }
                for (&(run, motor, _), point) in jobs.iter().zip(measure(&jobs)?) {
                    curves[run][motor].push(point);
                }
            }
        }
        jobs.clear();
        for (run, per_motor) in curves.iter_mut().enumerate() {
            for (motor, curve) in per_motor.iter_mut().enumerate() {
                curve.sort_by(|a, b| a.frequency.total_cmp(&b.frequency));
                if round < REFINE_ROUNDS {
                    let of = |value: fn(&LoopPoint) -> [f64; 2]| {
                        refinement(&curve.iter().map(|p| (p.frequency, value(p))).collect::<Vec<_>>())
                    };
                    let mut more = of(|p| [p.re, p.im]);
                    more.extend(of(|p| p.plant));
                    more.sort_by(f64::total_cmp);
                    more.dedup();
                    jobs.extend(more.into_iter().map(|f| (run, motor, f)));
                }
            }
        }
        if jobs.is_empty() {
            break;
        }
    }
    Ok(curves)
}

/// The margins of every motor's velocity loop for the config as it is and with each value of `bounds` at its low
/// and at its high bound, one at a time (as `verdict_bands`): per run its name ("" for the config as it is,
/// "path=value" otherwise) and the margins per motor, from the refined responses of `velocity_loop_responses`.
pub fn velocity_loop_margins(
    entry: &MachineEntry,
    cfg: &Config,
    bounds: &[(String, f64, f64)],
) -> Result<Vec<(String, Vec<LoopMargins>)>, ConfigError> {
    let resolved = &crate::catalog::resolve(entry, cfg)?;
    let (bounds, cfg) = (&followed(cfg, resolved, bounds), resolved);
    let mut runs = vec![(String::new(), cfg.clone())];
    for (path, low, high) in bounds {
        for value in [low, high] {
            let mut changed = cfg.clone();
            set_bounded(&mut changed, path, *value)?;
            runs.push((format!("{path}={value:?}"), changed));
        }
    }
    let mut configs: Vec<Config> = runs.iter().map(|(_, c)| c.clone()).collect();
    // With the play: each run once more with the flanks held together, for the margin across the play.
    let count = configs.len();
    let play = cfg.bool("effects.gearbox_backlash").unwrap_or(false);
    if play {
        for run in 0..count {
            let mut closed = configs[run].clone();
            closed.set_text("effects.gearbox_backlash = false")?;
            configs.push(closed);
        }
    }
    let responses = velocity_loop_responses(entry, &configs, &Progress::default())?;
    let motors = responses.first().map_or(0, Vec::len);
    let hammering = on_all_cores(count * motors, |j| backlash_hammering(entry, &configs[j / motors], j % motors))
        .into_iter()
        .collect::<Result<Vec<_>, _>>()?;
    Ok(runs
        .into_iter()
        .zip(&responses)
        .enumerate()
        .map(|(run, ((name, _), curves))| {
            let margins = curves.iter().enumerate().map(|(motor, points)| LoopMargins {
                hammering: hammering[run * motors + motor],
                play_margin_db: if play { play_margin(points, &responses[count + run][motor]) } else { f64::NAN },
                ..loop_margins(points)
            });
            (name, margins.collect())
        })
        .collect())
}

/// The describing function's gain margin of a velocity loop across its gearbox's play, dB, from the loop measured with
/// the play (`play`: small signals, the rotor swinging inside it) and with the flanks held together (`closed`, the
/// config with `effects.gearbox_backlash` off). Two things in the loop depend on the swing's amplitude A. Across the
/// play the flanks pass their spring's and their damper's force only while in contact (`Gearbox::torque`): over a swing
/// about it the first harmonic of both is kappa(A) = 1 - (2 / pi) (asin(b / A) + (b / A) sqrt(1 - (b / A)^2)) of the
/// gear's impedance y (b the half play), 0 inside the play, 1 far beyond it. And the friction on the rotor (its seals,
/// the gearbox's drag, the mesh), which damps the small-signal measurement (Re(1/P) some 0.2 N m s/rad on the five-bar's
/// tool turn, three times its J w at 650 Hz), is a Coulomb force whose first harmonic, 4 F / (pi A w), fades as the
/// swing grows. The margin is taken with it faded: the rotor alone, J_m s (J_m the median of Im(1/P) / w over the upper
/// half of the play's band), on kappa y to the load J_L (J_m + J_L from the closed plant at its lowest frequency),
/// 1 / P(kappa) = J_m s + kappa y J_L s / (kappa y + J_L s), y at each frequency from the closed plant (which so holds
/// the friction's small-signal share, scaled with kappa), and the rest of the loop, C = L / P, from the closed
/// measurement. The least gain margin (`loop_margins`) of L(kappa) = C P(kappa) over kappa from 1e-3 to 1 (301 values,
/// log spaced), each on 3000 frequencies over the band (C's log gain and unwrapped phase and y interpolated in log f).
/// A swing that the friction still damps is quieter: the five-bar's tool turn reads -3.07 dB at 0.08 N m s/rad, so it
/// would hammer from 0.0562, where the time run (`backlash_hammering`, bisected) hammers from 0.0782 with its friction
/// and from 0.0478 with it off (the impacts' harmonics and the damper that cannot pull, which the first harmonic leaves
/// out). NaN where the play does not change the loop (none on that motor: the two measurements within 1e-3 at every
/// common frequency), where a measurement is not valid, or the inertias do not come out positive.
pub fn play_margin(play: &[LoopPoint], closed: &[LoopPoint]) -> f64 {
    type Cx = [f64; 2];
    let mul = |a: Cx, b: Cx| [a[0] * b[0] - a[1] * b[1], a[0] * b[1] + a[1] * b[0]];
    let div = |a: Cx, b: Cx| {
        let n = b[0] * b[0] + b[1] * b[1];
        [(a[0] * b[0] + a[1] * b[1]) / n, (a[1] * b[0] - a[0] * b[1]) / n]
    };
    let add = |a: Cx, b: Cx| [a[0] + b[0], a[1] + b[1]];
    let sub = |a: Cx, b: Cx| [a[0] - b[0], a[1] - b[1]];
    let valid = |p: &LoopPoint| [p.re, p.im, p.plant[0], p.plant[1]].iter().all(|x| x.is_finite());
    if play.len() < 2 || closed.len() < 2 || !play.iter().chain(closed).all(valid) {
        return f64::NAN;
    }
    let same = closed.iter().all(|c| {
        play.iter()
            .find(|p| p.frequency == c.frequency)
            .is_none_or(|p| (p.re - c.re).hypot(p.im - c.im) <= 1e-3 * c.re.hypot(c.im))
    });
    if same {
        return f64::NAN;
    }
    let tau = std::f64::consts::TAU;
    let s = |f: f64| [0.0, tau * f];
    // The impedance 1 / P = J s of an inertia: J = Im(1 / P) / w.
    let inertia = |p: &LoopPoint| div([1.0, 0.0], p.plant)[1] / (tau * p.frequency);
    let mut upper: Vec<f64> = play[play.len() / 2..].iter().map(inertia).collect();
    upper.sort_by(f64::total_cmp);
    let rotor = upper[upper.len() / 2];
    let load = inertia(closed.iter().min_by(|a, b| a.frequency.total_cmp(&b.frequency)).unwrap()) - rotor;
    if !(rotor > 0.0 && load > 0.0) {
        return f64::NAN;
    }
    // Per measured frequency: ln f, C's ln gain and unwrapped phase, and y.
    let mut rows: Vec<(f64, f64, f64, Cx)> = Vec::with_capacity(closed.len());
    for p in closed {
        let (sv, plant) = (s(p.frequency), p.plant);
        let c = div([p.re, p.im], plant);
        let ls = mul([load, 0.0], sv);
        let load_side = sub(div([1.0, 0.0], plant), mul([rotor, 0.0], sv));
        let y = div(mul(load_side, ls), sub(ls, load_side));
        let raw = c[1].atan2(c[0]);
        let phase = rows.last().map_or(raw, |r| raw + tau * ((r.2 - raw) / tau).round());
        rows.push((p.frequency.ln(), c[0].hypot(c[1]).ln(), phase, y));
    }
    let (low, high) = (rows[0].0, rows[rows.len() - 1].0);
    const FINE: usize = 3000;
    let fine: Vec<(f64, Cx, Cx)> = (0..FINE)
        .map(|i| {
            let x = low + (high - low) * i as f64 / (FINE - 1) as f64;
            let k = rows.partition_point(|r| r.0 <= x).clamp(1, rows.len() - 1);
            let (a, b) = (&rows[k - 1], &rows[k]);
            let t = if b.0 > a.0 { ((x - a.0) / (b.0 - a.0)).clamp(0.0, 1.0) } else { 0.0 };
            let (gain, phase) = ((a.1 + (b.1 - a.1) * t).exp(), a.2 + (b.2 - a.2) * t);
            (x.exp(), [gain * phase.cos(), gain * phase.sin()], add(a.3, mul([t, 0.0], sub(b.3, a.3))))
        })
        .collect();
    (0..=300)
        .map(|j| {
            let kappa = 10f64.powf(-3.0 + 3.0 * j as f64 / 300.0);
            let points: Vec<LoopPoint> = fine
                .iter()
                .map(|&(f, c, y)| {
                    let (sv, ky) = (s(f), mul([kappa, 0.0], y));
                    let ls = mul([load, 0.0], sv);
                    let impedance = add(mul([rotor, 0.0], sv), div(mul(ky, ls), add(ky, ls)));
                    let plant = div([1.0, 0.0], impedance);
                    let l = mul(c, plant);
                    LoopPoint { frequency: f, re: l[0], im: l[1], sensitivity: 1.0 / (1.0 + l[0]).hypot(l[1]), plant }
                })
                .collect();
            loop_margins(&points).gain_margin_db
        })
        .fold(f64::INFINITY, f64::min)
}

/// A velocity loop that keeps oscillating on its own across a gearbox's play after a kick at rest
/// (`backlash_hammering`): the frequency, and the peak of the torque the loop asks for about its mean (N m).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Hammering {
    pub frequency_hz: f64,
    pub torque_nm: f64,
}

/// How long `backlash_hammering` sweeps, and the windows it watches the loop in afterwards.
pub const HAMMER_SWEEP_S: f64 = 0.2;
pub const HAMMER_WINDOW_S: f64 = 0.1;

/// The torque one motor's loop asks for after a sweep at rest: a sine of `amplitude` (N m) added to it as the margin
/// measurement adds its own, its frequency rising logarithmically over `loop_frequencies`' band (10 Hz to a quarter of
/// the velocity loop's rate) in `HAMMER_SWEEP_S`, then taken off. Per window of `HAMMER_WINDOW_S` after it (three):
/// the RMS about the window's mean, the number of times the torque crosses that mean, and its peak about it.
fn after_a_sweep(sim: &mut Sim, motor: usize, amplitude: f64) -> [(f64, usize, f64); 3] {
    let every = (sim.control.drive_params().velocity_cycle / sim.dt).round() as u64;
    let (f0, f1) = (10.0, 0.25 / sim.control.drive_params().velocity_cycle);
    let (t0, k) = (sim.time(), (f1 / f0).ln());
    let mut windows: [Vec<f64>; 3] = Default::default();
    loop {
        let t = sim.time() - t0;
        let window = ((t - HAMMER_SWEEP_S) / HAMMER_WINDOW_S).floor();
        if window >= 3.0 {
            break;
        }
        let due = sim.step.is_multiple_of(every);
        if due {
            // A logarithmic sweep: the phase is the integral of f0 e^(k t / T).
            let phase = std::f64::consts::TAU * f0 * HAMMER_SWEEP_S / k * ((k * t / HAMMER_SWEEP_S).exp() - 1.0);
            sim.control.drives[motor].torque_injection = if t < HAMMER_SWEEP_S { amplitude * phase.sin() } else { 0.0 };
        }
        sim.step();
        if due && window >= 0.0 {
            windows[window as usize].push(sim.control.drives[motor].torque_demand);
        }
    }
    windows.map(|seen| {
        let mean = seen.iter().sum::<f64>() / seen.len() as f64;
        let rms = (seen.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / seen.len() as f64).sqrt();
        let crossings = seen.windows(2).filter(|w| (w[0] > mean) != (w[1] > mean)).count();
        (rms, crossings, seen.iter().map(|x| (x - mean).abs()).fold(0.0, f64::max))
    })
}

/// Whether the velocity loop of `motor` hammers across its gearbox's play: a time run, as the small-signal measurement
/// (`velocity_loop_response`) cannot tell. Inside the play the measurement sees the rotor alone; a loop that is stable
/// there can still be unstable with the flanks in contact, and then the rotor swings from flank to flank in a limit
/// cycle (the describing function of the play rises from 0 with the rotor inside it to the flanks' stiffness with it far
/// beyond, so the loop crosses -1 at some amplitude). The motor's rated torque, swept over the band at rest
/// (`after_a_sweep`), takes the rotor across the play at each frequency in turn. The loop hammers where, 0.1 to 0.3 s
/// after the sweep, the torque it asks for still swings by more in RMS than the margin measurement's test torque
/// (`loop_amplitude`, beyond which `loop_point` says a loop moves on its own) and has not fallen to half from the
/// one window to the next, and where the same run with `effects.gearbox_backlash` off is quiet (otherwise the play is
/// not the cause). On the measurement's config (`lossless`). `None` without the play.
pub fn backlash_hammering(entry: &MachineEntry, cfg: &Config, motor: usize) -> Result<Option<Hammering>, ConfigError> {
    if !cfg.bool("effects.gearbox_backlash").unwrap_or(false) {
        return Ok(None);
    }
    let cfg = lossless(&crate::catalog::resolve(entry, cfg)?)?;
    let run = |cfg: &Config| -> Result<_, ConfigError> {
        let (mut sim, _) = settled(entry, cfg, 0)?;
        let id = sim.machine.motors().get(motor).ok_or_else(|| ConfigError::Invalid("motor".into()))?.id;
        let rated = 10.0 * loop_amplitude(entry, cfg, id)?;
        let [_, early, late] = after_a_sweep(&mut sim, motor, rated);
        Ok((early.0 > 0.1 * rated && late.0 > 0.1 * rated && late.0 > 0.5 * early.0).then_some(late))
    };
    let Some((_, crossings, peak)) = run(&cfg)? else { return Ok(None) };
    let mut without = cfg;
    without.set_text("effects.gearbox_backlash = false")?;
    if run(&without)?.is_some() {
        return Ok(None);
    }
    Ok(Some(Hammering { frequency_hz: crossings as f64 / (2.0 * HAMMER_WINDOW_S), torque_nm: peak }))
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
pub fn record_cycle(entry: &MachineEntry, cfg: &Config) -> Result<Vec<Vec<f64>>, ConfigError> {
    let mut sim = Sim::new(entry, cfg)?;
    let refused = |e: CommandError| ConfigError::Invalid(e.key.into());
    sim.command(Command::StartCycle).map_err(refused)?;
    let every = ((1e-3 / sim.dt).round() as u64).max(1);
    let (mut passes, mut last, mut start, mut rows, mut sample) =
        (0, sim.control.cycle_passes(), None, Vec::new(), Vec::new());
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

    /// By hand: a winding that settles at 102 K on every odd pass and 92 K on every even one (a pick alternating between
    /// two places) repeats every 2 passes; one at 100, 90, 80, 100, 90, 80 every 3; one that closes in on 95 K by halves
    /// (100, 97.5, 96.25, ...) and one that swings about it by halves (100, 92.5, 96.25, 94.375) have no period.
    #[test]
    fn the_warm_start_finds_the_period_of_a_cycle() {
        let rises = |r: &[f64]| r.iter().map(|&x| vec![x, 50.0]).collect::<Vec<_>>();
        assert_eq!(period_of(&rises(&[90.0, 102.3, 92.2, 101.9, 92.4])), Some(2));
        assert_eq!(period_of(&rises(&[60.0, 100.0, 90.0, 80.0, 100.0])), Some(3));
        assert_eq!(period_of(&rises(&[60.0, 105.0, 100.0, 97.5, 96.25])), None);
        assert_eq!(period_of(&rises(&[60.0, 110.0, 100.0, 92.5, 96.25])), None);
        // Too few warm passes to tell a period of 3 from the cold one's.
        assert_eq!(period_of(&rises(&[100.0, 90.0, 80.0, 100.0])), None);
    }

    /// Steps 1 to 5 where 5, 4 and 2 fail and 3 and 1 pass: the fastest that passes is 3, after judging 5, 4 and 3 (a
    /// bisection from 0 and 5 would judge 2, fail, and end at step 1). None pass: `None`, all five judged.
    #[test]
    fn the_fastest_passing_step_is_found_where_pass_and_fail_do_not_follow_the_steps() {
        let mut judged = Vec::new();
        let passes = [true, true, false, true, false, false];
        let found = fastest_passing(5, |k| -> Result<_, ()> {
            judged.push(k);
            Ok(passes[k].then_some(k * 10))
        });
        assert_eq!((found, judged), (Ok(Some((3, 30))), vec![5, 4, 3]));
        let mut count = 0;
        assert_eq!(fastest_passing(5, |_| -> Result<Option<()>, ()> { Ok((count += 1, None).1) }), Ok(None));
        assert_eq!(count, 5);
    }

    /// The line module's loss in the supply energy: efficiency eta = 0.9, the drive drew 100 J out of the DC link and fed
    /// 40 J into it, the bleeder took 2 J, so the module put 102 J in and took the 40 J out (mains 36 J arrive, through the
    /// module 142 J). The line gives 102 / 0.9 = 113.33 J and gets 36 J back, 77.33 J from the supply: the DC side's net
    /// 60 J, the bleeder's 2 J and the loss (1 / 0.9 - 1) 102 + (1 - 0.9) 40 = 15.33 J. Without the DC link simulated the
    /// net 60 J alone.
    #[test]
    fn the_supply_energy_has_the_line_modules_loss() {
        let cfg = Config::parse(
            "format_version = 1
[effects]
dc_link = true
regenerative_supply = true
[ratings]
line_module_efficiency = 0.9
",
        )
        .unwrap();
        let motor = MotorLoad {
            energy_in: 100.0,
            energy_regenerated: 40.0,
            mains_energy: 36.0,
            line_module_energy: 142.0,
            discharge_energy: 2.0,
            ..MotorLoad::default()
        };
        let load = CycleLoad { motors: vec![motor], ..CycleLoad::default() };
        assert!((supply_energy(&cfg, &load) - (102.0 / 0.9 - 36.0)).abs() < 1e-9, "{}", supply_energy(&cfg, &load));
        let off = Config::parse("format_version = 1\n[effects]\ndc_link = false\n").unwrap();
        assert!((supply_energy(&off, &load) - 60.0).abs() < 1e-12);
    }

    /// The refinement finds a sharp resonance that the 60 points of the loop measurement miss. Two masses on a spring,
    /// the motor's J1 = 1e-4 kg m², the load's J2 = 3e-4 kg m², k = 300 N m/rad, a damper c = 0.003 N m s/rad between
    /// them. The motor's speed per torque: P(s) = (J2 s² + c s + k) / (s (J1 J2 s² + c (J1 + J2) s + k (J1 + J2))).
    /// Anti-resonance sqrt(k / J2) = 1000 rad/s = 159.155 Hz; resonance sqrt(k (J1 + J2) / (J1 J2)) = sqrt(4e6) =
    /// 2000 rad/s = 318.310 Hz, damping ratio c (J1 + J2) / (J1 J2) / (2 * 2000) = 40 / 4000 = 0.01. At s = j2000 the
    /// numerator is -1200 + 300 + j6 = -900 + j6, the denominator j2000 (-0.12 + j0.0024 + 0.12) = -4.8: |P| = 900.02 /
    /// 4.8 = 187.50 rad/s per N m, 150 times the rigid 1 / ((J1 + J2) 2000) = 1.25. The 60 points from 10 to 2000 Hz
    /// lie 9.4 % apart, the nearest to the resonance at 303.5 and 331.9 Hz; the higher, the larger, has |P| = 44.7 by
    /// the same formula, a quarter of the peak.
    #[test]
    fn refinement_finds_a_sharp_resonance_the_coarse_sweep_misses() {
        let (j1, j2, k, c) = (1e-4, 3e-4, 300.0, 0.003);
        let plant = |f: f64| {
            let w = std::f64::consts::TAU * f;
            let (nr, ni) = (k - j2 * w * w, c * w);
            // j w (a + j b) = -w b + j w a
            let (a, b) = (k * (j1 + j2) - j1 * j2 * w * w, c * (j1 + j2) * w);
            let (dr, di) = (-w * b, w * a);
            let d2 = dr * dr + di * di;
            (f, [(nr * dr + ni * di) / d2, (ni * dr - nr * di) / d2])
        };
        let magnitude = |p: &(f64, [f64; 2])| p.1[0].hypot(p.1[1]);
        let peak =
            |points: &[(f64, [f64; 2])]| *points.iter().max_by(|a, b| magnitude(a).total_cmp(&magnitude(b))).unwrap();
        let close = |value: f64, expected: f64, share: f64, what: &str| {
            assert!((value / expected - 1.0).abs() < share, "{what}: {value} against {expected}")
        };
        close(magnitude(&plant(2000.0 / std::f64::consts::TAU)), 187.50, 1e-3, "|P| at the resonance");
        let mut points: Vec<_> = log_spaced(10.0, 2000.0, 60).into_iter().map(plant).collect();
        assert!(magnitude(&peak(&points)) < 0.25 * 187.5, "the coarse sweep sees the peak: {:?}", peak(&points));
        for _ in 0..REFINE_ROUNDS {
            let more = refinement(&points);
            points.extend(more.into_iter().map(plant));
            points.sort_by(|a, b| a.0.total_cmp(&b.0));
        }
        let top = peak(&points);
        close(top.0, 318.310, 2e-3, "resonance");
        close(magnitude(&top), 187.50, 0.03, "peak");
        let bottom = *points.iter().min_by(|a, b| magnitude(a).total_cmp(&magnitude(b))).unwrap();
        close(bottom.0, 159.155, 2e-3, "anti-resonance");
        // Only around the two: the rigid stretches (0.78 dB and no turn per step) are not split.
        assert!(points.len() < 60 + 80, "{} points", points.len());
        assert!(
            points.iter().filter(|p| p.0 < 100.0 || p.0 > 700.0).count()
                == log_spaced(10.0, 2000.0, 60).iter().filter(|f| **f < 100.0 || **f > 700.0).count()
        );
    }

    /// A lift whose corners are rounded keeps to `lowest` where the tolerance is 3 / 16 of its height above it: from
    /// pick and place at 0 up to 0.3 m and across 0.4 m, `lowest` 0.25 m, the corners rounded within 3 * 0.05 / 16 =
    /// 9.375 mm start d = 16 / 3 of that = 0.05 m before each corner (`path::round_corners`, a right angle), at 0.25 m on
    /// the way up and down: the path stays at `lowest` or above. Half as much again starts 0.075 m before the corners, at
    /// 0.225 m, below `lowest`: refused.
    #[test]
    fn a_lift_with_rounded_corners_keeps_above_the_lowest_height() {
        let axis = |id| sim_core::AxisInfo {
            id,
            key: "",
            unit: "m",
            min: -1.0,
            max: 1.0,
            max_vel: 1.0,
            max_acc: 10.0,
            max_jerk: 200.0,
        };
        let axes = [axis("x"), axis("z")];
        let lift = Lift {
            segments: [0, 1, 2],
            from: vec![0.0, 0.0],
            to: vec![0.4, 0.0],
            height: 0.3,
            up: 1.0,
            lowest: 0.25,
            needed: 0.25,
            over: None,
            profile: None,
            payload: true,
        };
        let path = |corner: f64| lift_path(&axes, 1e-3, 0.0, &lift, (0.3, Some(corner)), &[]);
        let lowest = |points: &[(f64, Vec<f64>)]| {
            points.iter().filter(|(_, p)| p[0] > 1e-9 && p[0] < 0.4 - 1e-9).map(|(_, p)| p[1]).fold(1.0, f64::min)
        };
        let rounded = path(3.0 * 0.05 / 16.0).expect("keeps to lowest");
        assert!((lowest(&rounded) - 0.25).abs() < 1e-3, "{}", lowest(&rounded));
        assert!(path(1.5 * 3.0 * 0.05 / 16.0).is_none());
    }

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
        close(
            belt_tooth_utilisation(&cfg, &MotorLoad { gearbox_torque_by_speed: bands, ..m.clone() }).unwrap().unwrap(),
            112.0 / 780.0,
            "tooth load beyond the table",
        );
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
            guides: Vec::new(),
            path_error_peak: 0.0,
            path_error_rms: 0.0,
            tracking_error_peak: 0.0,
            dwell_error_peak: 0.0,
            path_angle_error_peak: 0.0,
            path_angle_error_rms: 0.0,
            tracking_angle_error_peak: 0.0,
            dwell_angle_error_peak: 0.0,
            path_error_series: Vec::new(),
            tripped: false,
            motors: vec![m.clone()],
            budget: None,
            part_rises: Vec::new(),
            checks: Vec::new(),
            clearance: None,
            contact: None,
            braking_power_peak: 0.0,
            warm_start: None,
            conveyor: None,
            held_accelerations: Vec::new(),
        };
        close(guide_life_km(&cfg, &load).unwrap(), 6250.0, "guide km");
        close(guide_life_h(&cfg, &load).unwrap(), 6944.444, "guide h");
        // Two guides, C = 5000 N: 2000 N at 0.1 m/s lives (5000/2000)^3 * 50 = 781.25 km = 2170.14 h; 1000 N at 3 m/s
        // lives 6250 km = 578.70 h. The km life is the first's, the hours the second's: each the lowest over the guides.
        let guide = |equivalent: f64, speed: f64| GuideLoad { peak: equivalent, equivalent, speed: Some(speed) };
        let two = CycleLoad { guides: vec![guide(2000.0, 0.1), guide(1000.0, 3.0)], ..load.clone() };
        close(guide_life_km(&cfg, &two).unwrap(), 781.25, "lowest guide km");
        close(guide_life_h(&cfg, &two).unwrap(), 578.704, "lowest guide h");
        // What a config or a machine lacks gives no figure: no output torque, no radial load, no rating.
        assert_eq!(belt_tooth_utilisation(&cfg, &MotorLoad { gearbox_torque: None, ..m.clone() }).unwrap(), None);
        assert_eq!(pulley_bearing_life_h(&cfg, &MotorLoad { radial_load_equivalent: None, ..m.clone() }), None);
        assert_eq!(
            gearbox_life_h(
                &Config::parse("format_version = 1\n[ratings]\ngearbox_rated_torque_nm = 40.0\n").unwrap(),
                &m
            ),
            None
        );
        // Life factor a1 (ISO 281), by hand: 0.95 * (ln(100/99) / ln(100/90))^(2/3) + 0.05 = 0.248332 at 99 %
        // (the table's 0.25); 0.637912 at 95 % (0.64); 0.076832 at 99.95 % (0.077). Both bearing lives scale with it.
        let mut reliable = cfg.clone();
        for (percent, a1) in [(95.0, 0.637912), (99.0, 0.248332), (99.95, 0.076832)] {
            reliable.set_text(&format!("ratings.bearing_reliability_percent = {percent:?}")).unwrap();
            close(reliability_factor(&reliable).unwrap(), a1, "a1");
            close(pulley_bearing_life_h(&reliable, &m).unwrap(), a1 * 468696.16, "pulley bearing with a1");
            close(gearbox_life_h(&reliable, &m).unwrap(), a1 * 352777.89, "gearbox with a1");
            close(guide_life_h(&reliable, &load).unwrap(), a1 * 6944.444, "guide with a1");
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

    /// A check judged from below at or below zero is infinitely far over: a belt section pushed (belt-least-tension with a
    /// tension of -12 N against 0.5 N) would otherwise have limit over value -0.0417, and the report a margin of
    /// +104.2 % beside "not ok". Above zero it stays limit over value: 0.5 / 2 = 0.25.
    #[test]
    fn an_at_least_check_at_or_below_zero_is_infinitely_far_over() {
        let pushed = Verdict { check: "belt-least-tension", motor: None, value: -12.0, limit: 0.5, ok: false };
        assert_eq!(utilisation(&pushed), f64::INFINITY);
        assert_eq!(utilisation(&Verdict { value: 0.0, ..pushed.clone() }), f64::INFINITY);
        assert_eq!(utilisation(&Verdict { value: 2.0, ok: true, ..pushed }), 0.25);
    }

    /// a_ISO of ISO 281:2007 by hand from the standard's formulae (worked out separately in Python). Rated viscosity
    /// ν1 = 45000 * 100^-0.83 / sqrt(36) = 164.082 mm²/s at 100 r/min and D_pw 36 mm; 4500 / sqrt(1000) / sqrt(36) =
    /// 23.717 mm²/s at 1000 r/min. With e_C * C_u / P = 0.5 * 150 N / 600 N = 0.125, whose cube root is 0.5: at κ = 1,
    /// x = (2.5671 - 1.9987)^0.83 = 0.62568, a_ISO = 0.1 * (1 - 0.62568 * 0.5)^-9.3 = 3.27647; at κ = 0.2 (first
    /// branch) 0.198152, at 0.5 (second) 0.610291, at 2 (third) 6.03577, at 4 and above 11.0606; at κ 0.05, below the
    /// standard, as at 0.1: 0.100176; with e_C * C_u / P = 2 the bracket is below 0 and a_ISO is capped at 50. The
    /// pulley bearing of the life test with ν 120 mm²/s at 100 r/min: κ = 120 / 164.082 = 0.731341, a_ISO 1.555262,
    /// L_nm = 1.555262 * 468696.16 = 728945 h.
    #[test]
    fn a_iso_is_the_hand_calculated_one_of_iso_281() {
        let close = |a: f64, b: f64, what: &str| assert!((a / b - 1.0).abs() < 1e-5, "{what}: {a} against {b} by hand");
        close(reference_viscosity(100.0, 36.0), 164.082122, "ν1 below 1000 r/min");
        close(reference_viscosity(1000.0, 36.0), 23.717082, "ν1 from 1000 r/min");
        for (kappa, a_iso) in [
            (1.0, 3.276474),
            (0.2, 0.198152),
            (0.5, 0.610291),
            (2.0, 6.035768),
            (4.0, 11.060568),
            (8.0, 11.060568),
            (0.05, 0.100176),
        ] {
            close(life_modification_factor(kappa, 0.125), a_iso, &format!("a_ISO at κ {kappa}"));
        }
        // The three branches meet: the standard's curves are continuous.
        assert!(
            (life_modification_factor(0.4 - 1e-9, 0.125) / life_modification_factor(0.4, 0.125) - 1.0).abs() < 1e-3
        );
        assert!(
            (life_modification_factor(1.0 - 1e-9, 0.125) / life_modification_factor(1.0, 0.125) - 1.0).abs() < 1e-5
        );
        assert_eq!(life_modification_factor(4.0, 2.0), 50.0);
        let text = "format_version = 1\n[gearbox]\nratio = 5.0\n[ratings]\npulley_bearing_load_rating_n = 8469.0\nbearing_lubricant_viscosity_mm2_s = 120.0\n\
                    bearing_contamination_factor = 0.5\npulley_bearing_fatigue_limit_n = 150.0\npulley_bearing_mean_diameter_mm = 36.0\n";
        let cfg = Config::parse(text).unwrap();
        let m = MotorLoad {
            mean_speed: 500.0 * std::f64::consts::PI / 30.0,
            radial_load_equivalent: Some(600.0),
            ..MotorLoad::default()
        };
        close(pulley_bearing_life_h(&cfg, &m).unwrap(), 728945.29, "pulley bearing with a_ISO");
        // Some of the four values and not all, or e_C above 1, is refused by name; none of them is a_ISO = 1.
        let partial = Config::parse(&text.replace("pulley_bearing_mean_diameter_mm = 36.0\n", "")).unwrap();
        assert!(
            matches!(lubrication(&partial), Err(ConfigError::Missing(key)) if key == "ratings.pulley_bearing_mean_diameter_mm")
        );
        assert_eq!(pulley_bearing_life_h(&partial, &m), None);
        let mut dirty = cfg.clone();
        dirty.set_text("ratings.bearing_contamination_factor = 1.5").unwrap();
        assert!(
            matches!(lubrication(&dirty), Err(ConfigError::Invalid(key)) if key == "ratings.bearing_contamination_factor")
        );
        let bare = Config::parse(
            "format_version = 1\n[gearbox]\nratio = 5.0\n[ratings]\npulley_bearing_load_rating_n = 8469.0\n",
        )
        .unwrap();
        close(pulley_bearing_life_h(&bare, &m).unwrap(), 468696.16, "pulley bearing without a_ISO");
    }

    /// Palmgren-Miner over a two-level spectrum above the rated torque, by hand: M2N 40 N m, M2max 64 N m for 30000
    /// revolutions, Wöhler exponent 6. 1000 rev/h at 48 N m, which allows 30000 * (64 / 48)^6 = 30000 * 5.6186557 =
    /// 168560 rev, and 100 rev/h at 60 N m, 30000 * (64 / 60)^6 = 30000 * 1.4728969 = 44187 rev; 5000 rev/h at 30 N m do no
    /// damage. Damage 1000 / 168560 + 100 / 44187 = 0.0059326 + 0.0022631 = 0.0081957 per hour, life 122.0147 h. Through
    /// a cycle's spectrum (ratio 5, so 1000 rev/h of the output are 1000 * 2 pi * 5 / 3600 = 8.72665 rad/s of the motor):
    /// 48 N m lies in band 620 (ln(480) / ln(1.01) = 620.5), judged at its upper torque 0.1 * 1.01^621 = 48.2584 N m,
    /// 60 N m in band 642 at 60.0680 N m, 30 N m in band 573 (30.23 N m, below M2N): damage
    /// 1000 / (30000 * 5.4405290) + 100 / (30000 * 1.4629173) = 0.0061269 + 0.0022786 per hour, life 118.971 h.
    #[test]
    fn the_gearing_life_is_the_hand_calculated_damage_sum() {
        let close = |a: f64, b: f64, what: &str| assert!((a / b - 1.0).abs() < 1e-6, "{what}: {a} against {b} by hand");
        close(
            gearing_life_h([(30.0, 5000.0), (48.0, 1000.0), (60.0, 100.0)], 40.0, 64.0, 6.0),
            122.014741,
            "two levels",
        );
        assert_eq!(gearing_life_h([(30.0, 5000.0), (40.0, 5000.0)], 40.0, 64.0, 6.0), f64::INFINITY);
        // At M2max the line gives Siemens' 30000 revolutions: 30000 rev/h last one hour.
        close(gearing_life_h([(64.0, 30000.0)], 40.0, 64.0, 6.0), 1.0, "at M2max");
        let text = "format_version = 1\n[gearbox]\nratio = 5.0\n[ratings]\ngearbox_rated_torque_nm = 40.0\ngearbox_max_torque_nm = 64.0\n\
                    gearbox_gearing_wohler_exponent = 6.0\n";
        let cfg = Config::parse(text).unwrap();
        let rad_s = |rev_h: f64| rev_h * std::f64::consts::TAU * 5.0 / 3600.0;
        let mut spectrum = vec![0.0; 700];
        (spectrum[573], spectrum[620], spectrum[642]) = (rad_s(5000.0), rad_s(1000.0), rad_s(100.0));
        let m =
            MotorLoad { gearbox_torque: Some((60.0, 40.0)), gearbox_torque_spectrum: spectrum, ..MotorLoad::default() };
        close(torque_band_floor(621), 48.258422, "band 620's upper torque");
        close(gearbox_gearing_life_h(&cfg, &m).unwrap(), 118.971035, "through the spectrum");
        // Never above M2N: no figure, as the life would be infinite.
        let below = MotorLoad { gearbox_torque_spectrum: m.gearbox_torque_spectrum[..600].to_vec(), ..m.clone() };
        assert_eq!(gearbox_gearing_life_h(&cfg, &below), None);
        // No output torque reported, or a key missing: no figure.
        assert_eq!(gearbox_gearing_life_h(&cfg, &MotorLoad { gearbox_torque: None, ..m.clone() }), None);
        let without = Config::parse(&text.replace("gearbox_gearing_wohler_exponent = 6.0\n", "")).unwrap();
        assert_eq!(gearbox_gearing_life_h(&without, &m), None);
    }

    /// The motor's bearings, by hand from Siemens' figure for the 1FK2205: 1000 N at 1500 r/min for L10h = 25000 h is
    /// C = 1000 N * (25000 * 60 * 1500 / 10^6)^(1/3) = 1000 N * 2250^(1/3) = 13103.7 N. With the pulley on the shaft, 600 N
    /// at a mean 1000 r/min: 25000 h * (1000 / 600)^3 * (1500 / 1000) = 173611 h. Behind a gearbox there is no figure.
    #[test]
    fn the_motor_bearing_life_is_the_hand_calculated_one() {
        let cfg = Config::parse(
            "format_version = 1\n[gearbox]\nratio = 1.0\n[ratings]\nmotor_bearing_load_rating_n = 13103.7\n",
        )
        .unwrap();
        let m = MotorLoad {
            mean_speed: 1000.0 * std::f64::consts::PI / 30.0,
            radial_load_equivalent: Some(600.0),
            ..MotorLoad::default()
        };
        let life = motor_bearing_life_h(&cfg, &m).unwrap();
        assert!((life / 173611.11 - 1.0).abs() < 1e-5, "{life} against 173611 h by hand");
        let mut geared = cfg.clone();
        geared.set_text("gearbox.ratio = 5.0").unwrap();
        assert_eq!(motor_bearing_life_h(&geared, &m), None);
        assert_eq!(motor_bearing_life_h(&cfg, &MotorLoad { radial_load_equivalent: None, ..m }), None);
    }

    /// Two motors of one machine with gearbox ratios 5 and 10 at 500 r/min: the pulleys turn at 100 and 50 r/min, so the
    /// pulley bearing (C 8469 N, P 600 N, L10 = (C/P)^3 * 1e6 / (60 n) h) lives 468696.16 h and twice that.
    #[test]
    fn each_motor_gets_its_own_gearbox_ratio() {
        let cfg = Config::parse(
            "format_version = 1\n[gearbox]\nratio = 5.0\n[gearbox.b]\nratio = 10.0\n[ratings]\npulley_bearing_load_rating_n = 8469.0\nbearing_reliability_percent = 90.0\n",
        )
        .unwrap();
        let motor = |id: &str| MotorLoad {
            id: id.into(),
            mean_speed: 500.0 * std::f64::consts::PI / 30.0,
            radial_load_equivalent: Some(600.0),
            ..MotorLoad::default()
        };
        assert!((output_rpm(&cfg, &motor("a")) - 100.0).abs() < 1e-9);
        assert!((output_rpm(&cfg, &motor("b")) - 50.0).abs() < 1e-9);
        let (a, b) =
            (pulley_bearing_life_h(&cfg, &motor("a")).unwrap(), pulley_bearing_life_h(&cfg, &motor("b")).unwrap());
        assert!((a / 468696.16 - 1.0).abs() < 1e-5 && (b / a - 2.0).abs() < 1e-9, "{a} and {b}");
    }

    /// A guide at 100 N for 1 s at 1 m/s and at 200 N for 1 s at 3 m/s: the speed-weighted cubic mean is
    /// cbrt((100^3 * 1 + 200^3 * 3) / (1 + 3)) = cbrt(6.25e6) = 184.20 N (time-weighted: 165.1 N). A braking resistor, with the
    /// capacitors holding 2 J: of 5, 1.5 and 3 J fed back with 10 J drawn between each it takes (5 - 2) + 0 + (3 - 2) = 4 J.
    /// With 1 J drawn between, the link enters the 1.5 J and the 3 J 1 J full: the resistor takes 3 + 0.5 + 2 = 5.5 J and
    /// the link would have to hold 5 J. A cycle that ends on 1 J fed back and begins on 1.5 J
    /// enters its first braking 1 J full: 0.5 J spills (the link would hold 2.5 J).
    #[test]
    fn a_guide_load_is_weighted_with_distance_and_the_resistor_takes_what_the_link_cannot() {
        assert!((equivalent_load(100f64.powi(3) + 200f64.powi(3) * 3.0, 4.0) - 184.2).abs() < 0.01);
        assert_eq!(equivalent_load(0.0, 0.0), 0.0);
        let close = |(a, b): (f64, f64), (c, d): (f64, f64)| assert!((a - c).abs() < 1e-12 && (b - d).abs() < 1e-12);
        close(resistor_excess(&[5.0, -10.0, 1.5, -10.0, 3.0, -10.0], 2.0), (4.0, 5.0));
        close(resistor_excess(&[5.0, -1.0, 1.5, -1.0, 3.0, -10.0], 2.0), (5.5, 5.0));
        close(resistor_excess(&[1.5, -3.0, 1.0], 2.0), (0.5, 2.5));
        assert_eq!(resistor_excess(&[], 2.0), (0.0, 0.0));
    }

    /// The running account (`LinkAccount`) gives the two passes over the whole list, and keeps only the stretches until
    /// the link first emptied or filled: here a braking, then 100000 sign changes of noise at rest that draw a little more
    /// than they feed back (as a held axis does), then two more brakings. A 0.5 J and a 2 J link fill on the 2.5 J braking
    /// (the third stretch), and keep those three.
    #[test]
    fn the_dc_link_account_matches_two_passes_over_the_list_and_stays_bounded() {
        let two_passes = |events: &[f64], room: f64| {
            let (mut fill, mut excess, mut most) = (0.0f64, 0.0, 0.0f64);
            for (i, &e) in events.iter().chain(events).enumerate() {
                fill = (fill + e).max(0.0);
                if i >= events.len() {
                    most = most.max(fill);
                    excess += (fill - room).max(0.0);
                }
                fill = fill.min(room);
            }
            (excess, most)
        };
        let mut events = vec![0.3, -0.05, 2.5, -0.2];
        events.extend((0..100_000).map(|i| if i % 2 == 0 { 1e-4 } else { -1.2e-4 }));
        events.extend([1.7, -0.4, 3.1, -5.0]);
        for room in [0.5, 2.0, 10.0] {
            let mut account = LinkAccount::new(room);
            events.iter().for_each(|&e| account.push(e));
            let ((a, b), (c, d)) = (account.finish(), two_passes(&events, room));
            assert!((a - c).abs() < 1e-9 && (b - d).abs() < 1e-12, "room {room}: {a} {b} against {c} {d}");
            // A link of 10 J never fills nor empties here: it keeps them all, as the two passes did.
            assert!(room > 2.0 || account.prefix.len() == 3, "room {room}: {} stretches kept", account.prefix.len());
        }
    }

    /// A link of 10 J that neither empties nor fills over 400 000 stretches of noise (±1e-4 J, no net) keeps the passes
    /// over its voltage window, not the stretches, and gives the two passes' figures within half the grid's step
    /// (10 / 512 J). At both ends of the window: after a 0.3 J braking it sits near empty (by hand: nothing spilled, at
    /// most 0.6001 J held, the cycle entered 0.3 J full), after a 9.99 J one near full (entered 9.99 J full, the braking
    /// spills 9.98 J and the noise's first 1e-4 J: 9.9801 J, at most 19.98 J held). And once the passes meet after the grid
    /// was taken (a 12 J braking and 20 J drawn), the figures are the two passes' to rounding again (2.3 J, 12.3 J).
    #[test]
    fn a_dc_link_that_never_empties_nor_fills_keeps_a_bounded_account_at_both_ends() {
        let two_passes = |events: &[f64], room: f64| {
            let mut pass = [0.0, 0.0, 0.0];
            events.iter().for_each(|&e| link_step(&mut pass, e, room));
            pass[1..].fill(0.0);
            events.iter().for_each(|&e| link_step(&mut pass, e, room));
            (pass[1], pass[2])
        };
        let room = 10.0;
        let noise = (0..400_000).map(|i| if i % 2 == 0 { 1e-4 } else { -1e-4 });
        let cases = [
            (0.3, vec![], false, (0.0, 0.6001)),
            (9.99, vec![], false, (9.9801, 19.98)),
            (0.3, vec![12.0, -20.0], true, (2.3, 12.3)),
        ];
        for (start, end, meets, by_hand) in cases {
            let events: Vec<f64> = std::iter::once(start).chain(noise.clone()).chain(end).collect();
            let mut account = LinkAccount::new(room);
            events.iter().for_each(|&e| account.push(e));
            assert!(account.prefix.is_empty() && account.grid.len() == LINK_GRID && account.met == meets);
            let ((a, b), (c, d)) = (account.finish(), two_passes(&events, room));
            assert!((c - by_hand.0).abs() < 1e-9 && (d - by_hand.1).abs() < 1e-9, "{start}: {c} {d}");
            let tolerance = if meets { 1e-9 } else { room / (2.0 * (LINK_GRID - 1) as f64) };
            assert!((a - c).abs() <= tolerance && (b - d).abs() <= tolerance, "{start}: {a} {b} against {c} {d}");
        }
    }

    /// Without a gain crossover in the band the phase margin is NaN (not measurable) and `stable` judges the gain margin
    /// alone; a valid measurement with a negative margin, or one not valid, is not stable, nor one that hammers across a
    /// gearbox's play whatever its margins.
    #[test]
    fn a_loop_without_a_gain_crossover_is_judged_on_its_gain_margin() {
        let m = |phase: f64, gain: f64, valid: bool| LoopMargins {
            crossover_hz: f64::NAN,
            phase_margin_deg: phase,
            gain_margin_db: gain,
            valid,
            peak_sensitivity: 1.0,
            peak_frequency_hz: 10.0,
            hammering: None,
            play_margin_db: f64::NAN,
        };
        assert!(m(f64::NAN, 20.0, true).stable());
        let hammering = Some(Hammering { frequency_hz: 645.0, torque_nm: 0.7 });
        assert!(!LoopMargins { hammering, ..m(40.0, 6.0, true) }.stable());
        assert!(!m(f64::NAN, -1.0, true).stable());
        assert!(!m(f64::NAN, f64::NAN, true).stable());
        assert!(!m(-5.0, 20.0, true).stable() && m(40.0, 6.0, true).stable() && !m(40.0, 6.0, false).stable());
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
                LoopPoint { frequency: f, re, im, sensitivity: 1.0 / (1.0 + re).hypot(im), plant: [0.0; 2] }
            })
            .collect();
        let m = loop_margins(&points);
        assert!((m.crossover_hz - fc).abs() < 0.1, "crossover {} Hz", m.crossover_hz);
        assert!((m.phase_margin_deg - 54.0).abs() < 0.1, "phase margin {}", m.phase_margin_deg);
        assert!((m.gain_margin_db - 20.0 * 2.5f64.log10()).abs() < 0.01, "gain margin {}", m.gain_margin_db);
        // Without a crossover and without a crossing of the negative real axis, the gain margin is infinite and the
        // phase margin is not measurable (NaN), though the loop counts as stable.
        let m =
            loop_margins(&points[..10].iter().map(|p| LoopPoint { re: 2.0 + p.re.abs(), ..*p }).collect::<Vec<_>>());
        assert!(m.phase_margin_deg.is_nan() && m.gain_margin_db.is_infinite() && m.crossover_hz.is_nan() && m.stable());
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
                    LoopPoint { frequency: f, re, im, sensitivity: 1.0 / (1.0 + re).hypot(im), plant: [0.0; 2] }
                })
                .collect()
        };
        for (fc, tau, phase, gain) in [
            (100.0, 0.5e-3, 72.0, 13.979),
            (200.0, 0.5e-3, 54.0, 7.959),
            (300.0, 1e-3, -18.0, -1.584),
            (400.0, 1e-3, -54.0, -4.082),
        ] {
            let m = loop_margins(&response(fc, tau));
            assert!(
                (m.phase_margin_deg - phase).abs() < 0.1 && (m.gain_margin_db - gain).abs() < 0.01 && m.valid,
                "{fc} Hz, {tau} s: {m:?}"
            );
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
                LoopPoint { frequency: f, re, im, sensitivity: 1.0 / (1.0 + re).hypot(im), plant: [0.0; 2] }
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
            assert!(
                m.stable()
                    && m.gain_margin_db.is_infinite()
                    && (m.phase_margin_deg - phase).abs() < 0.3
                    && (m.crossover_hz / hz - 1.0).abs() < 0.01,
                "K {k}: {m:?}"
            );
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
            (
                3.0 * lag(5.0) / (lag(50.0) * lag(200.0) * lag(400.0)),
                (x / w(5.0)).atan() - (x / w(50.0)).atan() - (x / w(200.0)).atan() - (x / w(400.0)).atan(),
            )
        }));
        assert!(
            m.stable()
                && m.gain_margin_db.is_infinite()
                && (m.phase_margin_deg - 24.0).abs() < 0.2
                && (m.crossover_hz / 1516.0 - 1.0).abs() < 0.01,
            "{m:?}"
        );
    }

    /// A flat five-bar's arm 2 as measured: the main crossover just below the first point, then an anti-resonance
    /// (zeros at 14 Hz) and a resonance (poles at 40 Hz), both damped 0.05, and 1 ms of dead time: L = (2 pi 9.7 / s)
    /// (1 + 0.1 s/wz + s^2/wz^2) / (1 + 0.1 s/wp + s^2/wp^2) e^(-s 1 ms). Its first crossover, rising out of the
    /// notch at 22.5 Hz, has a phase of +71.4 degrees (a margin of 251.4); the one that counts, at 94.3 Hz, -121.9
    /// (by hand from the formula on a fine grid): 58.1 degrees, not 58.1 - 360 as the phase turned at that first one.
    #[test]
    fn a_crossover_with_phase_lead_before_the_one_that_counts_does_not_turn_the_phase() {
        let m = loop_margins(&loop_points(log_spaced(10.0, 2000.0, 60), |w| {
            let pair = |wn: f64| {
                let (re, im) = (1.0 - (w / wn).powi(2), 0.1 * w / wn);
                (re.hypot(im), im.atan2(re))
            };
            let ((zg, zp), (pg, pp)) = (pair(std::f64::consts::TAU * 14.0), pair(std::f64::consts::TAU * 40.0));
            (std::f64::consts::TAU * 9.7 / w * zg / pg, -std::f64::consts::FRAC_PI_2 + zp - pp - w * 1e-3)
        }));
        assert!((m.phase_margin_deg - 58.1).abs() < 0.5 && (m.crossover_hz / 94.3 - 1.0).abs() < 0.02, "{m:?}");
    }

    /// With the belt's wrap known (effects.belt_wrap) the teeth in mesh are wrap r / p in place of
    /// `ratings.belt_teeth_in_mesh`: 2 rad on a pulley of r = 0.02 m with p = 5 mm, 8 teeth. The load of the lives' test,
    /// 2 N m at up to 95.49 r/min of the pulley: 100 N * 1.4 / (34.045 N/cm * 2.5 cm * 8) = 0.205610; half a turn,
    /// pi r / p = 12.57 teeth, is capped at 12 like the configured 14: 0.137073.
    #[test]
    fn the_tooth_load_counts_the_teeth_in_mesh_from_the_wrap() {
        let cfg = Config::parse(
            "format_version = 1
[gearbox]
ratio = 5.0
[geometry]
pulley_radius_m = 0.02
[belt]
tooth_pitch_m = 0.005
[ratings]
             belt_tooth_load_rpm = [0.0, 100.0, 1000.0]
belt_tooth_load_n_cm = [35.0, 34.0, 26.0]
belt_width_mm = 25.0
belt_service_factor = 1.4
             belt_teeth_in_mesh = 14.0
",
        )
        .unwrap();
        let mut bands = vec![0.0; 200];
        bands[9] = 2.0;
        let m = MotorLoad {
            gearbox_torque: Some((2.0, 1.0)),
            gearbox_torque_by_speed: bands,
            belt_wrap: Some(2.0),
            ..MotorLoad::default()
        };
        let close = |a: f64, b: f64| assert!((a / b - 1.0).abs() < 1e-5, "{a} against {b} by hand");
        close(belt_tooth_utilisation(&cfg, &m).unwrap().unwrap(), 0.205610);
        close(
            belt_tooth_utilisation(&cfg, &MotorLoad { belt_wrap: Some(std::f64::consts::PI), ..m.clone() })
                .unwrap()
                .unwrap(),
            0.137073,
        );
    }

    /// The front keeps what no other point beats in all three figures at once. By hand, as (cycle time, energy,
    /// utilisation): A (1, 3, 0.5) and B (2, 2, 0.5) beat each other in one figure each, so both stay; C (2, 3, 0.5) is
    /// beaten by both; D (3, 1, 0.9) is the slowest but takes the least energy, so it stays; E is A again and is kept
    /// once; F (1, 3, 0.6) is A with a higher utilisation, beaten by A alone.
    #[test]
    fn the_front_keeps_the_points_no_other_beats_in_every_figure() {
        let point = |duration: f64, energy: f64, utilisation: f64| FrontPoint {
            duration,
            energy,
            utilisation,
            factors: [1.0; 3],
            blending: false,
            shaping: false,
        };
        let (a, b, c, d, f) = (
            point(1.0, 3.0, 0.5),
            point(2.0, 2.0, 0.5),
            point(2.0, 3.0, 0.5),
            point(3.0, 1.0, 0.9),
            point(1.0, 3.0, 0.6),
        );
        let front = non_dominated(&[d.clone(), c, b.clone(), a.clone(), a.clone(), f]);
        assert_eq!(front, [a, b, d]);
    }
}
