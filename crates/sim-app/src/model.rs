//! Everything the interface shows and edits, without any drawing: the chosen machine and language,
//! the config being edited, the scope history, tuning and analysis results. The window code only draws
//! this and calls its methods.

use crate::session::{Event, Request, Session, extend_trail};
use sim_components::analysis::{
    Band, Comparison, CycleLoad, MotorLoad, OptimiseOptions, Optimised, Sensitivity, Verdict, compare_effect,
};
use sim_components::analysis::{
    SWEEP_POINTS, StepResponse, SweepPoint, bandwidth, closed_loop_response, default_sweep_amplitude, frequency_sweep,
    log_spaced, modes_along, step_response,
};
use sim_components::analysis::{
    Worst, analyse_cycle, analyse_trajectory, belt_tooth_utilisation, bounds, gearbox_gearing_life_h, gearbox_life_h,
    guide_life_h, guide_life_km, motor_bearing_life_h, optimise_cycle_with, pulley_bearing_life_h, sensitivity,
    verdict_bands, verdict_bands_sampled, verdict_worst_and_bands, verdicts,
};
use sim_components::catalog::{Candidate, search_axes, search_motors};
use sim_components::identify::{
    AxisFit, BELT_STIFFNESS_KEY, JointFit, MoveStiffness, SweepFit, fit_joints, fit_move_stiffness, fit_rigid,
    fit_sweep, parse_sweep, resonance,
};
use sim_components::import::{Mapping, decode, import};
use sim_components::motor_fit::{Identified, QUANTITIES};
use sim_components::trace::{Trace, TraceError, differences, overlay_csv, per_axis_fit, replay};
use sim_components::{Command, CommandError, Sim, path};
use sim_core::{
    Config, ConfigError, Machine, MachineEntry, Mode, Scene, SignalInfo, TEMPERATURE_PREFIX, TORQUE_PARTS, Translations,
};
use sim_machines::{MACHINES, translations};
use std::collections::VecDeque;
use std::sync::Mutex;
use std::sync::mpsc::{Receiver, TryRecvError, channel};
use toml::Value;

/// Samples kept for the scopes: one minute at one per millisecond.
pub const HISTORY: usize = 60_000;

/// A message for the user, as a translation key and its argument.
pub type Message = (&'static str, String);

/// One group of the parameter panel: a config table and its scalar values.
pub struct ParamGroup {
    pub key: String,
    /// (config path, label key, value)
    pub params: Vec<(String, String, Value)>,
}

pub struct ScopeLine {
    /// Index into the signal list.
    pub signal: usize,
    /// A setpoint or command, drawn dashed in the colour of the actual value it belongs to.
    pub reference: bool,
}

/// One scope: one quantity for every axis or motor, actual values together with their setpoints.
pub struct Scope {
    pub quantity_key: String,
    pub unit: String,
    pub lines: Vec<ScopeLine>,
    pub visible: bool,
}

/// What a trace did inside the visible time window.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stats {
    pub min: f64,
    pub max: f64,
    pub rms: f64,
}

/// The answer of one axis's position loop to a step and to sines, for choosing gains.
pub struct Tuning {
    pub axis: usize,
    pub step: StepResponse,
    pub response: Vec<SweepPoint>,
    pub bandwidth: Option<f64>,
    /// The modes the machine reports at `MODE_POINTS` places over the axis's stroke, the other axes at the start pose
    /// (`analysis::modes_along`): empty lists where it reports none.
    pub modes: Vec<(f64, Vec<Mode>)>,
}

/// How many places over the stroke the tuning help asks the machine for its modes.
pub const MODE_POINTS: usize = 41;

/// What the analysis panel can compute. Each takes seconds, so each runs in a thread of its own.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Analysis {
    /// The cycle's loads, judged against what the parts allow.
    Cycle,
    /// The cycle again with each bounded parameter at its low and its high bound.
    Sensitivity,
    /// A shorter cycle that still passes every check.
    Optimise,
    /// The catalogue motors the inputs ask for, each tried on the cycle.
    Search,
    /// A path from a file: planned, run and judged.
    Path,
    /// A drive trace from a file: replayed, and the config fitted to it.
    Trace,
    /// The open-loop frequency response of the mechanics, from each motor's torque to its speed.
    Sweep,
    /// The belt stiffness fitted to a measured frequency response from a file.
    SweepFit,
    /// The cycle with one effect on and off, overlaid.
    Compare,
}

/// What the analysis panel is asked with, as typed.
#[derive(Debug, Clone, PartialEq)]
pub struct Inputs {
    /// The cycle analysis also runs the cycle with every bounded value at its bounds, for the error bands.
    pub bands: bool,
    /// For the error bands: how often the cycle is run with all bounded values between their bounds at once.
    /// 0: each value alone at its low and at its high bound.
    pub samples: usize,
    /// With the error bands: also search per check for the worst corner of the bounds (`verdict_worst`).
    pub worst: bool,
    /// Motor search: a supply of `SUPPLIES`, how the article numbers start, a gearbox series or nothing, and whether
    /// every axis of the catalogue of the machine's kind is tried too (`search_axes`).
    pub supply: String,
    pub series: String,
    pub gearboxes: String,
    pub axes: bool,
    /// Path: the file, whether it holds poses at times and not lines and arcs, and whether lines and arcs
    /// are timed as fast as the motors allow and not within the axis limits.
    pub path_file: String,
    pub path_timed: bool,
    pub time_optimal: bool,
    pub trace_file: String,
    /// A mapping file for a vendor's trace export (`sim-cli --map`); "" for a trace in the simulator's columns.
    pub trace_map: String,
    /// Also the belts' stiffness from the trace's moves (`sim-cli fit --stiffness`): some twenty replays more.
    pub trace_stiffness: bool,
    /// The config file to open or save.
    pub config_file: String,
    /// Part names as typed, in the order of `PARTS`: motor article, supply, gearbox, drive, belt and axis; "" for none.
    pub parts: [String; 6],
    /// Frequency response: lowest and highest frequency in Hz, number of frequencies and torque amplitude in
    /// N m, as `sim-cli sweep` takes them. The amplitude is also the one a measured sweep was taken with.
    pub sweep_from: f64,
    pub sweep_to: f64,
    pub sweep_count: usize,
    pub sweep_amplitude: f64,
    /// A measured frequency response (`frequency_hz`, `gain_rad_s_per_nm`) and the motor it was taken on.
    pub sweep_file: String,
    pub sweep_motor: usize,
    /// The effect comparison: the effect switched (its name in `[effects]`), and the quantity of the scope overlaid.
    pub compare_effect: String,
    pub compare_quantity: String,
    /// The height of the workspace map's slice, in the unit of the third axis; `None`: the middle of its range.
    pub workspace_z: Option<f64>,
    /// The parameter panel's "path = value" field, applied by `set_value`.
    pub assignment: String,
}

/// The supplies the motor catalogue has data for: the S210 at 230 V and 400 V, and the S120.
pub const SUPPLIES: [&str; 3] = sim_components::catalog::SUPPLIES;

/// A path as it was planned, run on a simulation of its own and judged.
pub struct PathRun {
    pub points: Vec<(f64, Vec<f64>)>,
    /// The planned poses as places in the 3D scene (`Machine::scene_point`), for drawing the path there.
    /// Empty for a machine without that mapping.
    pub scene_line: Vec<[f64; 3]>,
    pub load: CycleLoad,
    pub verdicts: Vec<Verdict>,
    /// How far the planned path is off the given one, at most: transition curves and arcs off their stated end.
    pub deviation: f64,
    /// The motors' torque-speed curves, for its operating points (`torque_speed_curves`).
    pub curves: Curves,
}

/// The motors' torque-speed curves as lines of (speed in rad/s, torque in N m): the highest and the lowest torque the
/// drives give (peak), and the same of the continuous (S1) rating.
pub type Curves = [Vec<[f64; 2]>; 4];

/// The torque-speed curves of `Curves` from the drive's data. The peak curve is straight between its corners
/// (`DriveParams::torque_curve`): the peak torque both ways up to the rated speed, falling to nothing at the no-load
/// speed in the direction of motion. The S1 curve is the rated torque up to the rated speed both ways.
// shortcut: the S1 line ends at the rated speed, where the motor data says nothing more; upgrade when the catalogue
// gives the S1 curve above it.
pub fn torque_speed_curves(drive: &sim_components::drive::DriveParams) -> Curves {
    let corners = [-drive.no_load_speed, -drive.rated_speed, 0.0, drive.rated_speed, drive.no_load_speed];
    let peak = |high: bool| {
        corners
            .iter()
            .map(|&speed| [speed, if high { drive.torque_curve(speed).1 } else { drive.torque_curve(speed).0 }])
            .collect()
    };
    let s1 = |sign: f64| {
        [-drive.rated_speed, drive.rated_speed].iter().map(|&speed| [speed, sign * drive.rated_torque]).collect()
    };
    [peak(true), peak(false), s1(1.0), s1(-1.0)]
}

/// A trace replayed and fitted: per motor the RMS and peak difference between the trace and its replay, of
/// torque and of following error, and per axis the fit (none for an axis the trace does not move both ways), or
/// on a machine that is fitted in its motors' coordinates (`per_axis_fit` false) that fit in `joints` and no `fits`.
/// With the trace and its replay, for the overlay, and per motor what the trace says by itself
/// (`sim-cli identify`: inertia, friction, the loops' lags). Where it was asked for, the belts' stiffness from the
/// trace's moves, or why the trace gives none.
pub struct TraceFit {
    pub differences: Vec<[f64; 4]>,
    pub fits: Vec<Option<AxisFit>>,
    pub joints: Option<Option<JointFit>>,
    pub stiffness: Option<Result<MoveStiffness, TraceError>>,
    pub identified: Vec<Identified>,
    pub measured: Trace,
    pub simulated: Trace,
}

#[allow(clippy::large_enum_variant)] // one per finished job, sent once over a channel, never stored in bulk
enum Report {
    /// With the error bands, where they were asked for, and the number of samples they are of (0: each
    /// bounded value alone at its bounds).
    /// And the worst corners, where they were searched, with the paths of the bounds they name.
    Cycle(CycleLoad, Vec<Verdict>, Option<Vec<Band>>, usize, Curves, Option<(Vec<Worst>, Vec<String>)>),
    Sensitivity(CycleLoad, Vec<Sensitivity>),
    Optimised(Optimised),
    /// With the supply that was searched.
    Search(Vec<Candidate>, String),
    Path(PathRun),
    Trace(TraceFit),
    Sweep(Vec<Vec<SweepPoint>>),
    /// With the motor the sweep was taken on.
    SweepFit(usize, SweepFit),
    /// With the effect that was switched.
    Compare(String, Comparison),
}

type Job<T> = Mutex<Receiver<Result<T, Message>>>;

/// What a background job has answered, if it has. A job whose thread is gone without an answer (it
/// panicked) answers with a message, so that it does not leave a spinner turning for ever.
fn answer<T>(job: &Job<T>) -> Option<Result<T, Message>> {
    match job.lock().map(|job| job.try_recv()) {
        Ok(Err(TryRecvError::Empty)) => None,
        Ok(Ok(answer)) => Some(answer),
        _ => Some(Err(("ui-job-ended", String::new()))),
    }
}

/// A table for the window to draw as it is, the header first. Every cell is text in the chosen language.
pub type Table = Vec<Vec<String>>;

/// The table headers of the analysis panel, as translation keys.
const CYCLE_KEYS: [&str; 12] = [
    "ui-analysis-cycle-time",
    "ui-analysis-duration",
    "ui-analysis-path-deviation",
    "ui-analysis-path-error",
    "ui-analysis-tracking-error",
    "ui-analysis-dwell-error",
    "ui-analysis-belt-min",
    "ui-analysis-slack-time",
    "ui-analysis-radial-load",
    "ui-analysis-guide-load",
    "ui-analysis-guide-life",
    "ui-analysis-guide-life-h",
];
/// One row per figure, one column per motor.
const MOTOR_KEYS: [&str; 12] = [
    "ui-analysis-peak-torque",
    "ui-analysis-rms-torque",
    "ui-analysis-cubic-torque",
    "ui-analysis-winding-rise",
    "ui-analysis-energy-in",
    "ui-analysis-energy-back",
    "ui-analysis-resistor-energy",
    "ui-analysis-resistor-power",
    "ui-analysis-dc-link-peak",
    "ui-analysis-mains-energy",
    "ui-analysis-housing-rise",
    "ui-analysis-drive-overload",
];
/// Rows of the motor table that a machine or a config may lack: the lives in hours and the belt's tooth load.
const LIFE_KEYS: [&str; 5] = [
    "ui-analysis-pulley-bearing-life",
    "ui-analysis-gearbox-life",
    "ui-analysis-gearbox-gearing-life",
    "ui-analysis-motor-bearing-life",
    "ui-analysis-belt-tooth",
];
const VERDICT_KEYS: [&str; 5] =
    ["ui-analysis-check", "ui-analysis-motor", "ui-analysis-value", "ui-analysis-limit", "ui-analysis-result"];
const BAND_KEYS: [&str; 6] = [
    "ui-analysis-check",
    "ui-analysis-motor",
    "ui-analysis-band-low",
    "ui-analysis-band-high",
    "ui-analysis-band-worst",
    "ui-analysis-band-share",
];
const WORST_KEYS: [&str; 7] = [
    "ui-analysis-check",
    "ui-analysis-motor",
    "ui-analysis-worst-value",
    "ui-analysis-limit",
    "ui-analysis-result",
    "ui-analysis-worst-high",
    "ui-analysis-worst-low",
];
const SEARCH_KEYS: [&str; 8] = [
    "ui-analysis-motor",
    "ui-search-gearbox",
    "ui-search-axis",
    "ui-search-mass",
    "ui-search-cost",
    "ui-analysis-result",
    "ui-search-tightest",
    "ui-search-share",
];
const TRACE_KEYS: [&str; 4] =
    ["ui-trace-torque-rms", "ui-trace-torque-peak", "ui-trace-error-rms", "ui-trace-error-peak"];
const FIT_KEYS: [&str; 4] = ["ui-trace-quantity", "ui-trace-config", "ui-trace-fitted", "ui-trace-error"];
/// The label of a quantity of `JointFit`: `ui-trace-` and its name with dashes.
fn joint_label(quantity: &str) -> String {
    format!("ui-trace-{}", quantity.replace('_', "-"))
}
/// The four values of `AxisFit`, in its order.
const FIT_QUANTITIES: [&str; 4] = ["ui-trace-mass", "ui-trace-viscous", "ui-trace-coulomb", "ui-trace-constant"];
/// The table of what the trace says per motor by itself.
const IDENTIFY_KEYS: [&str; 5] =
    ["ui-analysis-motor", "ui-trace-quantity", "ui-identify-value", "ui-trace-error", "ui-identify-interval"];
/// Rows of the frequency response's table and of its fit.
const SWEEP_KEYS: [&str; 2] = ["ui-sweep-resonance", "ui-sweep-damping"];
/// The operating points: motor, speed, torque.
const POINTS_KEYS: [&str; 3] = ["ui-analysis-motor", "ui-points-speed", "ui-points-torque"];
/// The overlay's quantities, in the order of `Model::overlay`.
const OVERLAY_KEYS: [&str; 3] = ["ui-overlay-torque", "ui-overlay-speed", "ui-overlay-error"];
/// Messages of the analysis panel, and the reasons a search gives for a motor it did not run.
const NOTES: [&str; 23] = [
    "ui-overlay-exported",
    "ui-report-written",
    "ui-report-failed",
    "ui-config-opened",
    "ui-config-saved",
    "ui-config-save-failed",
    "ui-simulation-ended",
    "ui-job-ended",
    "ui-analysis-stale",
    "ui-analysis-tripped",
    "ui-drive-tripped",
    "ui-search-bad-supply",
    "ui-search-none",
    "ui-search-no-gearbox",
    "ui-search-refused",
    "ui-search-limit-motor-speed",
    "ui-search-limit-motor-torque",
    "ui-search-axis-stroke",
    "ui-search-axis-payload",
    "ui-search-axis-speed",
    "ui-file-unreadable",
    "ui-file-line",
    "ui-analysis-figure",
];
/// The names of `[parts]`; a motor of the search goes into the first three, its drive and axis into two more.
const PARTS: [&str; 6] = sim_components::catalog::PART_KEYS;
const SENSITIVITY_KEYS: [&str; 6] = [
    "ui-analysis-parameter",
    "ui-analysis-low",
    "ui-analysis-high",
    "ui-analysis-peak-change",
    "ui-analysis-rms-change",
    "ui-analysis-time-change",
];
const OPTIMISED_KEYS: [&str; 3] = ["ui-analysis-setting", "ui-analysis-before", "ui-analysis-after"];
/// Every check a cycle can be judged by (`verdicts`); each has a label "check-<name>".
const CHECKS: [&str; 43] = [
    "motor-peak",
    "motor-thermal",
    "motor-speed",
    "pulley-torque",
    "gearbox-continuous",
    "gearbox-peak",
    "dc-link-energy",
    "braking-resistor-continuous",
    "braking-resistor-peak",
    "settling",
    "belt-tension",
    "belt-slack",
    "gearbox-input-speed",
    "gearbox-radial-load",
    "guide-load",
    "dc-link-overvoltage",
    "gearbox-thermal",
    "guide-thermal",
    "line-module-continuous",
    "line-module-peak",
    "pulley-bearing-life",
    "gearbox-life",
    "gearbox-gearing-life",
    "motor-bearing-life",
    "guide-life",
    "belt-tooth-load",
    "belt-tooth-jump",
    "belt-span-resonance",
    "drive-current",
    "drive-peak-current",
    "drive-overload",
    "singularity",
    "screw-speed",
    "screw-dn",
    "screw-buckling",
    "screw-static",
    "screw-life",
    "rack-feed-force",
    "strain-wave-life",
    "collision",
    "thermal-warm-start",
    "pick-tracking",
    "parts-missed",
];
/// The parts of a motor's torque in the effect view, in the order of `TORQUE_PARTS`, and the torque itself.
const BREAKDOWN_KEYS: [&str; 5] = [
    "quantity-torque-inertia",
    "quantity-torque-gravity",
    "quantity-torque-friction",
    "quantity-torque-rest",
    "quantity-torque",
];
/// The parts of a friction force, in the order of `Machine::friction_curve`.
const FRICTION_KEYS: [&str; 3] =
    ["quantity-friction-coulomb", "quantity-friction-viscous", "quantity-friction-stiction"];
/// The labels of the effect view's tables and of the rows of an energy budget that the machine does not name.
const EFFECT_KEYS: [&str; 21] = [
    "ui-effects-part",
    "ui-effects-peak",
    "ui-effects-rms",
    "ui-effects-item",
    "ui-effects-energy",
    "ui-effects-share",
    "ui-effects-now",
    "ui-effects-total",
    "quantity-friction-speed",
    "quantity-friction-load-scale",
    "budget-supply",
    "budget-unaccounted",
    "budget-copper",
    "budget-resistor",
    "budget-windings",
    "budget-capacitors",
    "budget-mains",
    "budget-line-module",
    "budget-payload",
    "budget-crash",
    "budget-discharge",
];

/// The labels of the thermal, belt and comparison views.
const VIEW_KEYS: [&str; 30] = [
    "ui-effects-component",
    "ui-effects-signal",
    "ui-effects-thermal",
    "ui-effects-thermal-hint",
    "ui-effects-thermal-off",
    "ui-effects-rise",
    "ui-effects-settles",
    "ui-effects-rated",
    "ui-effects-of-rated",
    "ui-effects-over-rating",
    "ui-effects-settled-of-rated",
    "ui-effects-cold",
    "ui-effects-above",
    "ui-effects-belt",
    "ui-effects-belt-hint",
    "ui-effects-section",
    "ui-effects-tension",
    "ui-effects-pretension",
    "ui-effects-rating",
    "ui-effects-stretch",
    "ui-effects-slack",
    "ui-effects-double",
    "ui-effects-compare",
    "ui-effects-compare-hint",
    "ui-effects-effect",
    "ui-effects-quantity",
    "ui-effects-run-compare",
    "ui-effects-difference-peak",
    "ui-effects-difference-rms",
    "ui-effects-off",
];

/// The part colours live in the core, so that the browser's 3D view (sim-wasm) colours as this one does.
pub use sim_core::scene::{heat_color, part_color};

/// The temperatures in the effect view: one line per winding and, with effects.part_heating or thermal_feedback, per
/// heated part, over the scopes' window, and the table of where each is now, where it settles on the analysed cycle and
/// what it is rated for.
pub struct ThermalView {
    /// Per motor winding and heated part its name and the points (time, temperature rise in K).
    pub lines: Vec<(String, Vec<[f64; 2]>)>,
    pub table: Table,
    /// Neither the windings nor the parts warm (effects.winding_heating, part_heating and thermal_feedback off).
    pub off: bool,
}

/// The belt sections in the effect view: tension and stretch per section over the scopes' window, the pretension and
/// the rating to compare them with, and the table of the newest sample.
pub struct BeltView {
    /// Per section its name and the points (time, tension in N) and (time, stretch in mm).
    pub tension: Vec<(String, Vec<[f64; 2]>)>,
    pub stretch: Vec<(String, Vec<[f64; 2]>)>,
    pub pretension: f64,
    /// `ratings.belt_max_tension_n`, where the config has it.
    pub rating: Option<f64>,
    pub table: Table,
    /// The titles of the two plots with their units, and the ends of the 3D view's colour scale (no tension, twice
    /// the pretension) with their numbers.
    pub titles: [String; 2],
    pub scale: [String; 2],
}

/// The effect comparison: one scope's quantity with the effect on (solid) and off (dashed), and per line the peak and
/// RMS of the difference.
pub struct CompareView {
    /// The effect's label, and the quantity with its unit.
    pub effect: String,
    pub title: String,
    /// Per signal of the quantity its owner's name and the points (time, value) on and off.
    pub lines: Vec<(String, Vec<[f64; 2]>, Vec<[f64; 2]>)>,
    pub table: Table,
}

/// A horizontal slice through what the machine can reach, for the workspace map (`Model::workspace_slice`).
pub struct WorkspaceSlice {
    /// The span of the first two axes the grid covers, (low, high) each, and the slice's height on the third.
    pub x: (f64, f64),
    pub y: (f64, f64),
    pub z: f64,
    /// Nodes per side: node (i, j) is at x.0 + i (x.1 - x.0) / (n - 1), and likewise in y with j.
    pub n: usize,
    /// Per node, row after row (j outer): the condition number there, `INFINITY` out of reach of the kinematics, and
    /// whether the machine goes there (`Machine::unreachable` has nothing against it: within the envelope and the
    /// joints' stops, and not near a singularity).
    pub nodes: Vec<(f64, bool)>,
    /// Where the colour scale ends: `ratings.max_condition_number`, or 10 where the config has none.
    pub limit: f64,
    /// The range the height can be chosen in: the third axis's.
    pub heights: (f64, f64),
}

/// One friction contact in the effect view.
pub struct FrictionView {
    /// Whose it is, translated.
    pub title: String,
    /// Steady sliding over the speed, with the levels at the guide load of the newest sample: the friction force
    /// and its Coulomb, viscous and stiction parts, each with its label.
    pub curves: [(String, Vec<[f64; 2]>); 4],
    /// Where the contact is now, by the newest sample: (speed, friction force).
    pub now: Option<[f64; 2]>,
    /// The newest sample as a table: speed, the three parts, the friction force and the load factor.
    pub table: Table,
}

/// Rows of the sensitivity ranking that are shown: the parameters that matter most.
const TOP_ROWS: usize = 8;

/// A number at a readable length, whatever its size: four digits and at most three decimals, so 12.5 reads
/// as 12.50 and 1912.141 as 1912, and 0.0000312 as 3.12e-5; at least three digits, as the exponent has, so 0.0146
/// reads as 0.0146 and not 0.015. Narrow, so that the tables fit the panel.
pub fn short(value: f64) -> String {
    match value.abs() {
        0.0 => "0.000".into(),
        size if (0.01..1e5).contains(&size) => {
            let exponent = size.log10().floor() as i32;
            format!("{value:.*}", (3 - exponent).clamp(0, 3).max(2 - exponent) as usize)
        }
        _ => format!("{value:.2e}"),
    }
}

pub struct Model {
    pub entry: &'static MachineEntry,
    pub lang: &'static str,
    pub tr: Translations,
    /// The config as edited. The simulation may yet refuse the last edits.
    pub config: Config,
    /// The text the config was read from (the machine's default or a file), whose comments and layout a
    /// saved file keeps.
    config_text: String,
    /// The last config the simulation took, to go back to when it refuses one, and the configs sent to it
    /// that it has not answered yet, oldest first.
    accepted: Config,
    sent: VecDeque<Config>,
    pub session: Session,
    /// The end of the simulation thread has been reported.
    ended: bool,
    pub scopes: Vec<Scope>,
    pub history: VecDeque<Vec<f64>>,
    /// The history as it was when the scopes were frozen; the simulation runs on meanwhile.
    pub frozen: Option<VecDeque<Vec<f64>>>,
    /// Seconds of history the scopes show.
    pub window_s: f64,
    pub scene: Scene,
    pub paused: bool,
    pub speed: f64,
    /// How many times larger the 3D view draws the parts' deflections (`Scene::exaggeration`).
    pub exaggeration: f64,
    /// Target of the next point-to-point move, per axis.
    pub target: Vec<f64>,
    pub messages: Vec<Message>,
    pub tuning_axis: usize,
    pub tuning: Option<Tuning>,
    tuning_job: Option<Job<Tuning>>,
    /// The sizing report being written; answers with the message to show.
    report_job: Option<Job<Message>>,
    pub inputs: Inputs,
    /// Results of the analysis panel, each for the config as it was when it was asked for.
    /// With the motors' torque-speed curves, to plot the operating points over.
    pub cycle_report: Option<(CycleLoad, Vec<Verdict>, Curves)>,
    /// The checks of the cycle with their error bands, where the cycle was analysed with them.
    pub bands: Option<Vec<Band>>,
    /// How many samples those bands are of; 0: each bounded value alone at its low and at its high bound.
    pub band_samples: usize,
    /// Per check the worst corner of the bounds found, and the paths of the bounds in the order its `high` has them.
    pub worst: Option<(Vec<Worst>, Vec<String>)>,
    /// The cycle as configured and the varied parameters, strongest first.
    pub sensitivity: Option<(CycleLoad, Vec<Sensitivity>)>,
    pub optimised: Option<Optimised>,
    /// The motors of the search as ranked, and the supply they were searched for.
    pub candidates: Option<(Vec<Candidate>, String)>,
    pub path_run: Option<PathRun>,
    /// Where the tool has been in the 3D scene since the machine took the planned path, a point per 0.1 mm of
    /// travel and the newest `session::TRAIL` of them: the path it really took, to draw beside the planned one. `None`
    /// before the machine has taken a path (it may refuse it), and after a reconfigure or the start of a cycle.
    pub trail: Option<Vec<[f64; 3]>>,
    pub trace_fit: Option<TraceFit>,
    /// Per motor the frequency response, at the frequencies `inputs` gave when it was asked for.
    pub sweeps: Option<Vec<Vec<SweepPoint>>>,
    /// The measured response's fit, and the motor it was taken on.
    pub sweep_fit: Option<(usize, SweepFit)>,
    /// The effect comparison, with the effect that was switched.
    pub comparison: Option<(String, Comparison)>,
    /// Per kind of analysis (`Analysis as usize`) the config its result was computed for. A result says
    /// nothing about another config: it is marked (`outdated`), and nothing of it is applied to one.
    computed_for: [Option<Config>; 9],
    /// The jobs under way, each with the config it was started with.
    analysis_jobs: Vec<(Analysis, Config, Job<Report>)>,
}

/// A config without the names of its motor, gearbox and drive: what a motor search does not depend on, because it puts its
/// own in (the drive the catalogue pairs with each motor). The belt and axis the config names are part of what it
/// searches on, so naming an axis of the axis search marks the search as computed for an earlier configuration.
fn without_parts(config: &Config) -> Config {
    let mut out = config.clone();
    for key in &PARTS[..4] {
        let _ = out.set_text(&format!("parts.{key} = \"\""));
    }
    out
}

/// The config as a file: `text`, the file it was read from, with every value the config has changed written
/// in place, so that its comments and layout stay. Where that does not give the config back (an array was
/// changed), the config is written plainly, without comments.
fn saved_text(config: &Config, text: &str) -> Result<String, Message> {
    let plain = config.to_text().map_err(|e| (e.key(), e.detail().to_string()))?;
    let (Ok(doc), Ok(old)) = (toml_edit::Document::parse(text), Config::parse(text)) else { return Ok(plain) };
    // Each new value replaces the old one's text: a document written out anew would group dotted keys that the file
    // has apart (`ratings.a`, `thermal.b`, `ratings.c` in `[bounds]`), and so change its order.
    let mut edits = Vec::new();
    for (path, value) in config.leaves() {
        if old.get(&path).ok() == Some(&value) {
            continue;
        }
        let new = match value {
            Value::Float(x) => toml_edit::Value::from(x),
            Value::Integer(n) => toml_edit::Value::from(n),
            Value::Boolean(on) => toml_edit::Value::from(on),
            Value::String(name) => toml_edit::Value::from(name),
            _ => return Ok(plain),
        };
        // The span is the value's own text: the comment behind it stays.
        let span = path
            .split('.')
            .try_fold(doc.as_item(), |item, key| item.get(key))
            .and_then(toml_edit::Item::as_value)
            .and_then(toml_edit::Value::span);
        let Some(span) = span else { return Ok(plain) };
        edits.push((span, new.to_string()));
    }
    edits.sort_by_key(|(span, _)| std::cmp::Reverse(span.start));
    let mut kept = text.to_string();
    for (span, new) in edits {
        kept.replace_range(span, &new);
    }
    Ok(if Config::parse(&kept).as_ref() == Ok(config) { kept } else { plain })
}

fn texts(lang: &str, entry: &MachineEntry) -> Translations {
    let mut tr = translations(lang, entry);
    crate::I18N.iter().filter(|(l, _)| *l == lang).for_each(|(_, text)| tr.add(text));
    tr
}

/// The quantity a setpoint or command belongs to: "position-setpoint" is drawn with "position".
fn actual_quantity(quantity_key: &str) -> Option<&str> {
    quantity_key.strip_suffix("-setpoint").or_else(|| quantity_key.strip_suffix("-command"))
}

/// The scopes follow from the signal list alone: one per quantity, in order of first appearance,
/// with setpoints and commands joined to the quantity they ask for.
fn scopes_for(signals: &[SignalInfo]) -> Vec<Scope> {
    const SHOWN_AT_START: [&str; 4] =
        ["quantity-position", "quantity-following-error", "quantity-speed", "quantity-torque"];
    let mut scopes: Vec<Scope> = Vec::new();
    for (i, signal) in signals.iter().enumerate().filter(|(_, s)| s.id != "time") {
        let actual = actual_quantity(&signal.quantity_key)
            .filter(|a| signals.iter().any(|s| s.quantity_key == *a && s.unit == signal.unit));
        let key = actual.unwrap_or(&signal.quantity_key);
        let line = ScopeLine { signal: i, reference: actual.is_some() };
        match scopes.iter_mut().find(|s| s.quantity_key == key && s.unit == signal.unit) {
            Some(scope) => scope.lines.push(line),
            None => scopes.push(Scope {
                quantity_key: key.into(),
                unit: signal.unit.clone(),
                lines: vec![line],
                visible: SHOWN_AT_START.contains(&key),
            }),
        }
    }
    scopes
}

/// A factor and an SI prefix that bring a value of this size into a readable range:
/// 0.00004 m reads as 40 µm.
pub fn display_scale(peak: f64, unit: &str) -> (f64, &'static str) {
    match peak {
        _ if unit.is_empty() || peak == 0.0 || !peak.is_finite() => (1.0, ""),
        p if p < 1e-3 => (1e6, "µ"),
        p if p < 0.1 => (1e3, "m"),
        p if p >= 1e4 => (1e-3, "k"),
        _ => (1.0, ""),
    }
}

impl Model {
    pub fn new(entry: &'static MachineEntry, lang: &'static str) -> Self {
        Self::with_config(entry, lang, entry.config(), entry.default_config.into())
            .expect("default config of a registered machine must run")
    }

    /// The machine running `config`, which was read from `text`.
    fn with_config(
        entry: &'static MachineEntry,
        lang: &'static str,
        config: Config,
        text: String,
    ) -> Result<Self, ConfigError> {
        let session = Session::start(entry, &config)?;
        let sweep_amplitude = default_sweep_amplitude(entry, &config)?;
        let mut model = Self {
            entry,
            lang,
            tr: texts(lang, entry),
            scopes: scopes_for(&session.signals),
            accepted: config.clone(),
            sent: VecDeque::new(),
            config,
            config_text: text,
            session,
            ended: false,
            history: VecDeque::new(),
            frozen: None,
            window_s: 5.0,
            scene: Scene::default(),
            paused: false,
            speed: 1.0,
            exaggeration: 1.0,
            target: Vec::new(),
            messages: Vec::new(),
            tuning_axis: 0,
            tuning: None,
            tuning_job: None,
            report_job: None,
            inputs: Inputs {
                bands: false,
                samples: 0,
                worst: false,
                supply: SUPPLIES[1].into(),
                series: String::new(),
                gearboxes: String::new(),
                axes: false,
                path_file: String::new(),
                path_timed: false,
                time_optimal: false,
                trace_file: String::new(),
                trace_map: String::new(),
                trace_stiffness: false,
                config_file: String::new(),
                parts: Default::default(),
                sweep_from: 5.0,
                sweep_to: 500.0,
                sweep_count: SWEEP_POINTS,
                sweep_amplitude,
                sweep_file: String::new(),
                sweep_motor: 0,
                compare_effect: String::new(),
                compare_quantity: "quantity-torque".into(),
                workspace_z: None,
                assignment: String::new(),
            },
            cycle_report: None,
            bands: None,
            band_samples: 0,
            worst: None,
            sensitivity: None,
            optimised: None,
            candidates: None,
            path_run: None,
            trail: None,
            trace_fit: None,
            sweeps: None,
            sweep_fit: None,
            comparison: None,
            computed_for: Default::default(),
            analysis_jobs: Vec::new(),
        };
        model.target = model.config.f64s("start.pose").unwrap_or_default();
        model.show_parts();
        // Guide friction to compare where the config has it, else the first effect.
        let effects: Vec<String> =
            model.effects().into_iter().map(|(path, ..)| path["effects.".len()..].to_string()).collect();
        model.inputs.compare_effect =
            effects.iter().find(|e| *e == "guide_friction").or(effects.first()).cloned().unwrap_or_default();
        Ok(model)
    }

    /// Switching machine restarts the simulation with that machine's default config.
    pub fn set_machine(&mut self, entry: &'static MachineEntry) {
        let fresh = Self::new(entry, self.lang);
        self.restart(fresh);
    }

    /// Puts a freshly started model in place of this one, keeping how the window is set up and the fields as
    /// typed, except those that belong to the machine and config left: the config file (Save would write the
    /// new config over it), a motor number the new machine may not have, and the sweep amplitude where it is
    /// still the default for the old motors.
    fn restart(&mut self, fresh: Self) {
        let (paused, speed, exaggeration, window, inputs) =
            (self.paused, self.speed, self.exaggeration, self.window_s, self.inputs.clone());
        let old_amplitude = default_sweep_amplitude(self.entry, &self.config).ok();
        *self = fresh;
        let parts = std::mem::take(&mut self.inputs.parts);
        let sweep_motor = inputs.sweep_motor.min(self.session.motors.len().saturating_sub(1));
        let sweep_amplitude = if old_amplitude == Some(inputs.sweep_amplitude) {
            self.inputs.sweep_amplitude
        } else {
            inputs.sweep_amplitude
        };
        let config_file = String::new();
        let compare_effect = if self.config.bool(&format!("effects.{}", inputs.compare_effect)).is_ok() {
            inputs.compare_effect.clone()
        } else {
            self.inputs.compare_effect.clone()
        };
        (self.window_s, self.inputs) =
            (window, Inputs { parts, config_file, sweep_motor, sweep_amplitude, compare_effect, ..inputs });
        self.set_paused(paused);
        self.set_speed(speed);
        self.set_exaggeration(exaggeration);
    }

    /// The part fields show the parts the config names.
    fn show_parts(&mut self) {
        for (field, key) in self.inputs.parts.iter_mut().zip(PARTS) {
            *field =
                self.config.get(&format!("parts.{key}")).ok().and_then(|name| name.as_str()).unwrap_or("").to_string();
        }
    }

    /// Opens a config file for the machine shown: it replaces the running config, as switching machine does.
    /// A file that cannot be read, or that the machine refuses, is reported and the running config stays.
    /// A config of another machine (one this machine refuses and that one reads) switches to that machine.
    /// A machine file (one with a `[machine]` table, or any `.json`; see docs/machine-files.md) brings its own
    /// machine and replaces the machine shown too.
    pub fn open_config(&mut self, path: &str) {
        let json = path.to_ascii_lowercase().ends_with(".json");
        let machine_file = json
            || std::fs::read_to_string(path)
                .ok()
                .and_then(|text| Config::parse(&text).ok())
                .is_some_and(|c| c.get("machine").is_ok_and(|m| m.is_table()));
        if machine_file {
            let opened = sim_machines::load_file(std::path::Path::new(path))
                .map_err(|e| {
                    (
                        e.error.key(),
                        format!("{}{}: {}", path, e.line.map_or(String::new(), |l| format!(":{l}")), e.error.detail()),
                    )
                })
                .and_then(|entry| {
                    Self::with_config(entry, self.lang, entry.config(), entry.default_config.to_string())
                        .map_err(|e| (e.key(), e.detail().to_string()))
                });
            return match opened {
                Ok(fresh) => {
                    self.restart(fresh);
                    // A JSON file is held as TOML: Save does not write that over it.
                    self.inputs.config_file = if json { String::new() } else { path.into() };
                    self.say(("ui-machine-file-opened", path.into()));
                }
                Err(message) => self.say(message),
            };
        }
        let opened = std::fs::read_to_string(path)
            .map_err(|e| ("ui-file-unreadable", format!("{path}: {e}")))
            .and_then(|text| {
                let entry = match sim_components::load_config(self.entry, &text) {
                    Ok(_) => self.entry,
                    Err(e) => *MACHINES
                        .iter()
                        .find(|m| sim_components::load_config(m, &text).is_ok())
                        .ok_or_else(|| (e.key(), e.detail().to_string()))?,
                };
                let config =
                    sim_components::load_config(entry, &text).map_err(|e| (e.key(), e.detail().to_string()))?;
                Self::with_config(entry, self.lang, config, text).map_err(|e| (e.key(), e.detail().to_string()))
            });
        match opened {
            Ok(fresh) => {
                self.restart(fresh);
                self.inputs.config_file = path.into();
                self.say(("ui-config-opened", path.into()));
            }
            Err(message) => self.say(message),
        }
    }

    /// Writes the config as edited to a file: over the text it was read from, so that the comments and layout
    /// of that file stay, wherever only values have changed.
    /// A file that holds a config of another machine is not overwritten.
    pub fn save_config(&mut self, path: &str) {
        let other_machine = std::fs::read_to_string(path)
            .ok()
            .filter(|old| sim_components::load_config(self.entry, old).is_err())
            .and_then(|old| {
                MACHINES.iter().find(|m| m.id != self.entry.id && sim_components::load_config(m, &old).is_ok())
            });
        if let Some(other) = other_machine {
            let name = self.tr.tr(other.key).to_string();
            return self.say(("ui-config-other-machine", format!("{path}: {name}")));
        }
        let written = saved_text(&self.config, &self.config_text).and_then(|text| {
            std::fs::write(path, &text).map_err(|e| ("ui-config-save-failed", format!("{path}: {e}")))?;
            Ok(text)
        });
        match written {
            Ok(text) => {
                self.config_text = text;
                self.inputs.config_file = path.into();
                self.say(("ui-config-saved", path.into()));
            }
            Err(message) => self.say(message),
        }
    }

    /// Names the parts typed in `inputs.parts` in the config and applies it to the running simulation, as
    /// the motor search's button does. A name the catalogue does not have is refused, and so is any name in a
    /// config without `[parts]`, as on the command line.
    pub fn apply_parts(&mut self) {
        let names = self.inputs.parts.clone().map(|name| name.trim().to_string());
        if !names[1].is_empty() && !SUPPLIES.contains(&names[1].as_str()) {
            return self.say(("ui-search-bad-supply", names[1].clone()));
        }
        self.name_parts(names.each_ref().map(String::as_str));
    }

    pub fn set_language(&mut self, lang: &'static str) {
        self.lang = lang;
        self.tr = texts(lang, self.entry);
    }

    pub fn set_paused(&mut self, paused: bool) {
        self.paused = paused;
        self.session.send(Request::Pause(paused));
    }

    pub fn set_speed(&mut self, speed: f64) {
        self.speed = speed;
        self.session.send(Request::Speed(speed));
    }

    pub fn set_exaggeration(&mut self, factor: f64) {
        self.exaggeration = factor.clamp(1.0, 1.0e4);
        self.session.send(Request::Exaggeration(self.exaggeration));
    }

    /// Adds a message, unless it only repeats the last one: a refusal that comes back with every edit
    /// would otherwise bury everything else.
    fn say(&mut self, message: Message) {
        if self.messages.last() != Some(&message) {
            self.messages.push(message);
        }
    }

    /// The axis limits that ask too much of the motors, in place of those reported before.
    fn say_limits(&mut self, violations: Vec<sim_components::LimitViolation>) {
        self.messages.retain(|(key, _)| !key.starts_with("limit-"));
        for v in violations {
            let motor = self.tr.tr(self.session.motors[v.motor].key).to_string();
            self.say((v.key, format!("{motor}: {:.4} > {:.4}", v.needed, v.allowed)));
        }
    }

    /// Sends the config as edited to the running simulation. Its answer comes with `poll`.
    fn reconfigure(&mut self) {
        self.sent.push_back(self.config.clone());
        self.session.send(Request::Reconfigure(self.config.clone()));
    }

    /// Edits one config value and applies the config to the running simulation.
    pub fn set_param(&mut self, path: &str, value: Value) {
        match self.config.set(path, value) {
            Ok(()) => self.reconfigure(),
            Err(e) => self.say((e.key(), e.detail().into())),
        }
    }

    /// Applies one `path = value` (TOML value) to the config and the running simulation, as `sim-cli --set` does:
    /// a key the config does not have, a value of another type or text that is no assignment is refused with the
    /// CLI's message; a value the simulation refuses (outside a bound) comes back with `poll`, as with any edit.
    pub fn set_value(&mut self, assignment: &str) {
        match self.config.set_text(assignment) {
            Ok(()) => self.reconfigure(),
            Err(e) => self.say((e.key(), e.detail().into())),
        }
    }

    /// `sim-app --set path=value`, each in turn, before the window opens: the machine restarts with them, and a
    /// saved file still has the comments and order of the one read. The first refusal is returned, with nothing
    /// applied.
    pub fn set_at_start(&mut self, assignments: &[String]) -> Result<(), Message> {
        if assignments.is_empty() {
            return Ok(());
        }
        let refused = |e: ConfigError| (e.key(), e.detail().to_string());
        let mut config = self.config.clone();
        for assignment in assignments {
            config.set_text(assignment).map_err(refused)?;
        }
        let fresh = Self::with_config(self.entry, self.lang, config, self.config_text.clone()).map_err(refused)?;
        let file = self.inputs.config_file.clone();
        self.restart(fresh);
        self.inputs.config_file = file;
        Ok(())
    }

    pub fn command(&mut self, command: Command) {
        self.session.send(Request::Command(command));
    }

    /// Takes in what the simulation and a running tuning job have produced since the last frame.
    pub fn poll(&mut self) {
        for event in self.session.drain() {
            match event {
                Event::Sample(sample) => {
                    if self.history.len() == HISTORY {
                        self.history.pop_front();
                    }
                    self.history.push_back(sample);
                }
                Event::Scene(scene) => self.scene = scene,
                Event::TrailStart(at) => self.trail = at.map(|at| vec![at]),
                // Not after `clear`: the result the trail belonged to is gone.
                Event::Trail(points) => {
                    if let Some(trail) = &mut self.trail {
                        extend_trail(trail, &points);
                    }
                }
                Event::Refused(key, detail) => self.say((key, detail)),
                Event::Limits(violations) => self.say_limits(violations),
                Event::Configured(answer) => {
                    let config = self.sent.pop_front();
                    match answer {
                        Ok(violations) => {
                            self.accepted = config.unwrap_or_else(|| self.accepted.clone());
                            self.say_limits(violations);
                        }
                        // The refused value goes, or it would stay in the config and have every later edit
                        // refused with it.
                        Err(message) => {
                            self.config = self.accepted.clone();
                            self.say(message);
                        }
                    }
                    // With every answer in, the config shown is the one the simulation runs.
                    if self.sent.is_empty() {
                        self.config = self.accepted.clone();
                    }
                }
            }
        }
        if self.session.ended() && !self.ended {
            self.ended = true;
            self.say(("ui-simulation-ended", String::new()));
        }
        if let Some(result) = self.report_job.as_ref().and_then(answer) {
            self.report_job = None;
            self.say(result.unwrap_or_else(|message| message));
        }
        if let Some(result) = self.tuning_job.as_ref().and_then(answer) {
            self.tuning_job = None;
            match result {
                Ok(tuning) => self.tuning = Some(tuning),
                Err(message) => self.say(message),
            }
        }
        // A job stays until it has answered.
        let mut reports = Vec::new();
        self.analysis_jobs.retain(|(kind, config, job)| {
            answer(job).map(|report| reports.push((*kind, config.clone(), report))).is_none()
        });
        for (kind, config, report) in reports {
            let report = match report {
                Ok(report) => report,
                Err(message) => {
                    self.say(message);
                    continue;
                }
            };
            let tripped = |load: &CycleLoad| load.tripped.then_some(("ui-analysis-tripped", String::new()));
            let note = match report {
                Report::Cycle(load, verdicts, bands, samples, curve, worst) => {
                    let note = tripped(&load);
                    (self.cycle_report, self.bands, self.band_samples, self.worst) =
                        (Some((load, verdicts, curve)), bands, samples, worst);
                    note
                }
                Report::Sensitivity(nominal, rows) => {
                    self.sensitivity = Some((nominal, rows));
                    None
                }
                Report::Optimised(found) => {
                    self.optimised = Some(found);
                    None
                }
                Report::Search(found, supply) => {
                    self.candidates = Some((found, supply));
                    None
                }
                Report::Path(run) => {
                    // The machine runs the path as well, if it is still the machine the path was planned
                    // for. It refuses by itself where it has moved since, or is busy.
                    let stale = config != self.config;
                    if !stale {
                        self.command(Command::Trajectory(run.points.clone()));
                    }
                    let note = tripped(&run.load).or(stale.then_some(("ui-analysis-stale", String::new())));
                    self.path_run = Some(run);
                    note
                }
                Report::Trace(fit) => {
                    self.trace_fit = Some(fit);
                    None
                }
                Report::Sweep(sweeps) => {
                    self.sweeps = Some(sweeps);
                    None
                }
                Report::SweepFit(motor, fit) => {
                    self.sweep_fit = Some((motor, fit));
                    None
                }
                Report::Compare(effect, comparison) => {
                    self.comparison = Some((effect, comparison));
                    None
                }
            };
            self.computed_for[kind as usize] = Some(config);
            if let Some(note) = note {
                self.say(note);
            }
        }
    }

    /// Where the machine's setpoint stands, by the newest sample.
    fn standing(&self) -> Option<Vec<f64>> {
        let row = self.history.back()?;
        let signal = |axis: &sim_core::AxisInfo| {
            self.session.signals.iter().position(|s| s.id == format!("axis.{}.position_setpoint", axis.id))
        };
        self.session.axes.iter().map(|axis| Some(row[signal(axis)?])).collect()
    }

    // ---------------------------------------------------------------------------------------
    // Scopes

    /// Freezing keeps the traces as they are for a closer look; the simulation runs on.
    pub fn set_frozen(&mut self, frozen: bool) {
        self.frozen = frozen.then(|| self.history.clone());
    }

    /// The rows the scopes show: the last `window_s` seconds of the live or the frozen history.
    pub fn visible_rows(&self) -> impl Iterator<Item = &Vec<f64>> {
        let rows = self.frozen.as_ref().unwrap_or(&self.history);
        let from = rows.back().map_or(0.0, |last| last[0] - self.window_s);
        let first = rows.partition_point(|row| row[0] < from);
        rows.range(first..)
    }

    pub fn stats(&self, signal: usize) -> Option<Stats> {
        let (mut min, mut max, mut sum, mut n) = (f64::INFINITY, f64::NEG_INFINITY, 0.0, 0usize);
        for row in self.visible_rows() {
            let v = row[signal];
            (min, max, sum, n) = (min.min(v), max.max(v), sum + v * v, n + 1);
        }
        (n > 0).then(|| Stats { min, max, rms: (sum / n as f64).sqrt() })
    }

    /// Writes what the scopes show (every signal, the visible time window) as CSV.
    pub fn export_traces(&mut self, path: &std::path::Path) {
        let header: Vec<String> = self.session.signals.iter().map(|s| format!("{}[{}]", s.id, s.unit)).collect();
        let mut text = header.join(",") + "\n";
        for row in self.visible_rows() {
            let cells: Vec<String> = row.iter().map(|v| format!("{v:.9e}")).collect();
            text += &(cells.join(",") + "\n");
        }
        let message = match std::fs::write(path, text) {
            Ok(()) => ("ui-exported", path.display().to_string()),
            Err(e) => ("ui-export-failed", e.to_string()),
        };
        self.say(message);
    }

    pub fn report_running(&self) -> bool {
        self.report_job.is_some()
    }

    /// Writes the sizing report of the config as it is now (`sim_components::report`) to `path`, as PDF for a name
    /// ending in .pdf and as HTML otherwise, with the error bands where the analysis panel asks for them. In the
    /// background: the cycle and the bands take a while.
    pub fn export_report(&mut self, path: &str) {
        let (entry, config, text, lang, tr) =
            (self.entry, self.config.clone(), self.config_text.clone(), self.lang, self.tr.clone());
        let source = if self.inputs.config_file.is_empty() {
            tr.tr("report-default-config").to_string()
        } else {
            self.inputs.config_file.clone()
        };
        let (bands, path) = (self.inputs.bands, path.to_string());
        let (result, job) = channel();
        std::thread::spawn(move || {
            let write = || -> Result<Message, Message> {
                let report = sim_components::report::make(entry, &text, &source, &config, &config, bands, &tr)
                    .map_err(|e| (e.key(), e.detail().to_string()))?;
                let bytes =
                    if path.to_lowercase().ends_with(".pdf") { report.pdf() } else { report.html(lang).into_bytes() };
                std::fs::write(&path, bytes).map_err(|e| ("ui-report-failed", format!("{path}: {e}")))?;
                Ok(("ui-report-written", path.clone()))
            };
            let _ = result.send(write());
        });
        self.report_job = Some(Mutex::new(job));
    }

    // ---------------------------------------------------------------------------------------
    // Tuning

    pub fn tuning_running(&self) -> bool {
        self.tuning_job.is_some()
    }

    /// Measures step and frequency response of one axis with the config as it is now, in the
    /// background on a simulation of its own. The running machine is not disturbed.
    pub fn run_tuning(&mut self, axis: usize) {
        let (entry, config) = (self.entry, self.config.clone());
        let (result, job) = channel();
        std::thread::spawn(move || {
            let measure = || -> Result<Tuning, sim_core::ConfigError> {
                let step = step_response(entry, &config, axis, 0.001, 0.5)?;
                let response = closed_loop_response(entry, &config, axis, &log_spaced(1.0, 200.0, 40), 0.0005)?;
                let sim = Sim::new(entry, &config)?;
                // At the configured start pose, where the tool stands: the plan at rest is the motors' pose by the
                // rigid kinematics, off it by the belt's stretch (46 um in Z on the shipped gantry's belts).
                let modes = modes_along(sim.machine.as_ref(), &config.f64s("start.pose")?, axis, MODE_POINTS);
                Ok(Tuning { axis, step, bandwidth: bandwidth(&response), response, modes })
            };
            let _ = result.send(measure().map_err(|e| (e.key(), e.detail().to_string())));
        });
        self.tuning_axis = axis;
        self.tuning_job = Some(Mutex::new(job));
    }

    // ---------------------------------------------------------------------------------------
    // Analysis

    pub fn analysis_running(&self) -> bool {
        !self.analysis_jobs.is_empty()
    }

    pub fn running(&self, kind: Analysis) -> bool {
        self.analysis_jobs.iter().any(|job| job.0 == kind)
    }

    pub fn has_result(&self, kind: Analysis) -> bool {
        self.computed_for[kind as usize].is_some()
    }

    /// The result of this kind was computed for a config that has been changed since. A motor search does
    /// not depend on the parts the config names, so naming one of its motors does not outdate it.
    pub fn outdated(&self, kind: Analysis) -> bool {
        self.computed_for[kind as usize].as_ref().is_some_and(|config| match kind {
            Analysis::Search => without_parts(config) != without_parts(&self.config),
            _ => *config != self.config,
        })
    }

    /// Starts one computation of the analysis panel with the config as it is now, in the background on
    /// simulations of its own, like the tuning measurement. The running machine is not disturbed.
    ///
    /// The search, the path and the trace take what `inputs` holds at this moment. A path is planned from
    /// where the machine stands, and when its analysis is back the machine runs it too (see `poll`).
    pub fn run_analysis(&mut self, kind: Analysis) {
        let (entry, config, inputs) = (self.entry, self.config.clone(), self.inputs.clone());
        if kind == Analysis::Search && !SUPPLIES.contains(&inputs.supply.as_str()) {
            return self.say(("ui-search-bad-supply", inputs.supply));
        }
        let (standing, motors) = (self.standing(), self.session.motors.clone());
        let (result, job) = channel();
        std::thread::spawn(move || {
            let refused = |e: ConfigError| (e.key(), e.detail().to_string());
            let command = |e: CommandError| (e.key, String::new());
            let read =
                |file: &str| std::fs::read_to_string(file).map_err(|e| ("ui-file-unreadable", format!("{file}: {e}")));
            let run = || -> Result<Report, Message> {
                Ok(match kind {
                    Analysis::Cycle => {
                        let mut sim = Sim::new(entry, &config).map_err(refused)?;
                        let load = analyse_cycle(&mut sim).map_err(command)?;
                        let curve = torque_speed_curves(sim.control.drive_params());
                        let verdicts = verdicts(&config, &load).map_err(refused)?;
                        // Two more cycles per bounded value, or one per sample (always with the same seed, so
                        // that asking again gives the same bands).
                        let banded = |bounds: Vec<(String, f64, f64)>| match inputs.samples {
                            0 => verdict_bands(entry, &config, &bounds),
                            samples => verdict_bands_sampled(entry, &config, &bounds, samples, 1),
                        };
                        // The search for the worst corners, beside the bands. Its first runs are those of the bands at
                        // their bounds, which it gives back, so that they are not run twice.
                        let (bands, worst) = if inputs.bands && inputs.worst {
                            let bounded = bounds(&config).map_err(refused)?;
                            let (worst, at_bounds, _) =
                                verdict_worst_and_bands(entry, &config, &bounded).map_err(refused)?;
                            let bands = if inputs.samples == 0 {
                                at_bounds
                            } else {
                                banded(bounded.clone()).map_err(refused)?
                            };
                            (Some(bands), Some((worst, bounded.into_iter().map(|bound| bound.0).collect())))
                        } else {
                            (
                                if inputs.bands {
                                    Some(banded(bounds(&config).map_err(refused)?).map_err(refused)?)
                                } else {
                                    None
                                },
                                None,
                            )
                        };
                        Report::Cycle(load, verdicts, bands, inputs.samples, curve, worst)
                    }
                    Analysis::Sensitivity => {
                        // The values the config calls uncertain: those it gives bounds. Two cycles each.
                        let paths: Vec<String> =
                            bounds(&config).map_err(refused)?.into_iter().map(|bound| bound.0).collect();
                        let (nominal, mut rows) = sensitivity(entry, &config, &paths, 0.2).map_err(refused)?;
                        rows.sort_by(|a, b| b.strength(&nominal).total_cmp(&a.strength(&nominal)));
                        Report::Sensitivity(nominal, rows)
                    }
                    // With the error bands ticked the result has to pass in them too; the front comes with six cycles more.
                    Analysis::Optimise => Report::Optimised(
                        optimise_cycle_with(
                            entry,
                            &config,
                            &OptimiseOptions {
                                max_runs: 40,
                                robust: inputs.bands,
                                pareto: true,
                                worst: inputs.bands && inputs.worst,
                            },
                        )
                        .map_err(refused)?,
                    ),
                    Analysis::Search => {
                        let (series, gearboxes) =
                            (inputs.series.trim(), Some(inputs.gearboxes.trim()).filter(|series| !series.is_empty()));
                        let wanted = |article: &str| article.starts_with(series);
                        let found = if inputs.axes {
                            search_axes(entry, &config, &inputs.supply, &wanted, gearboxes)
                        } else {
                            search_motors(entry, &config, &inputs.supply, &wanted, gearboxes)
                        };
                        if found.iter().all(|c| c.article.is_empty()) {
                            // Motors there are, so it is the gearbox series that has nothing for them.
                            let motors = sim_components::catalog::motor_articles_for(&inputs.supply)
                                .iter()
                                .any(|article| article.starts_with(series));
                            return Err(match gearboxes.filter(|_| motors) {
                                Some(gearboxes) => ("ui-search-no-gearbox", gearboxes.to_string()),
                                None => ("ui-search-none", series.to_string()),
                            });
                        }
                        Report::Search(found, inputs.supply.clone())
                    }
                    Analysis::Path => {
                        let unreadable = |line: String| ("ui-file-line", format!("{}: {line}", inputs.path_file));
                        let text = read(&inputs.path_file)?;
                        let mut from_here = config.clone();
                        if let Some(pose) = &standing {
                            from_here.set_text(&format!("start.pose = {pose:?}")).map_err(refused)?;
                        }
                        let mut sim = Sim::new(entry, &from_here).map_err(refused)?;
                        // The planning model puts the tool at `start.pose` and its setpoint one rest sag off it (0.26 mm
                        // on the planar delta): the tool is moved by what is left until the setpoint is the live one.
                        if let Some(pose) = &standing {
                            let mut tool = pose.clone();
                            for _ in 0..8 {
                                let off: Vec<f64> =
                                    pose.iter().zip(sim.control.planned_pos()).map(|(want, is)| want - is).collect();
                                if off.iter().all(|d| d.abs() < 1e-10) {
                                    break;
                                }
                                tool.iter_mut().zip(&off).for_each(|(t, d)| *t += d);
                                from_here.set_text(&format!("start.pose = {tool:?}")).map_err(refused)?;
                                // shortcut: a tool pose pushed past a stroke's end keeps the plan of the last one that
                                // started, up to one sag off; a setpoint within a sag of the end is rare.
                                match Sim::new(entry, &from_here) {
                                    Ok(closer) => sim = closer,
                                    Err(_) => break,
                                }
                            }
                        }
                        let (points, deviation) = if inputs.path_timed {
                            (path::parse_trajectory(&text).map_err(unreadable)?, 0.0)
                        } else {
                            let pieces = path::parse(&text).map_err(unreadable)?;
                            (
                                sim.plan_path(&pieces, inputs.time_optimal).map_err(command)?,
                                sim.path_deviation(&pieces, inputs.time_optimal).map_err(command)?,
                            )
                        };
                        let scene_line = points.iter().filter_map(|(_, pose)| sim.machine.scene_point(pose)).collect();
                        let load = analyse_trajectory(&mut sim, points.clone()).map_err(command)?;
                        let verdicts = verdicts(&config, &load).map_err(refused)?;
                        Report::Path(PathRun {
                            points,
                            scene_line,
                            load,
                            verdicts,
                            deviation,
                            curves: torque_speed_curves(sim.control.drive_params()),
                        })
                    }
                    Analysis::Trace => {
                        let unfit = |e: TraceError| (e.key, e.detail);
                        // A vendor's export through its mapping, whose motors are the machine's in its order.
                        let (measured, setpoints) = if inputs.trace_map.is_empty() {
                            let measured = Trace::parse(&read(&inputs.trace_file)?, &motors).map_err(unfit)?;
                            let none = vec![None; measured.motors.len()];
                            (measured, none)
                        } else {
                            let bytes = std::fs::read(&inputs.trace_file)
                                .map_err(|e| ("ui-file-unreadable", format!("{}: {e}", inputs.trace_file)))?;
                            let imported =
                                import(&decode(&bytes), &Mapping::parse(&read(&inputs.trace_map)?).map_err(unfit)?)
                                    .map_err(unfit)?;
                            if imported.trace.motors.len() != motors.len() {
                                return Err((
                                    "error-import-map",
                                    format!("{} [[motor]] for {} motors", imported.trace.motors.len(), motors.len()),
                                ));
                            }
                            (imported.trace, imported.torque_setpoints)
                        };
                        let simulated = replay(entry, &config, &measured).map_err(unfit)?;
                        let per_axis = per_axis_fit((entry.build)(&config).map_err(refused)?.as_ref());
                        let fits =
                            if per_axis { fit_rigid(entry, &config, &measured).map_err(unfit)? } else { Vec::new() };
                        let joints =
                            if per_axis { None } else { Some(fit_joints(entry, &config, &measured).map_err(unfit)?) };
                        let stiffness = inputs
                            .trace_stiffness
                            .then(|| fit_move_stiffness(entry, &config, &measured, BELT_STIFFNESS_KEY));
                        let identified = measured
                            .motors
                            .iter()
                            .zip(&setpoints)
                            .map(|(m, s)| Identified::of(&measured.time, m, s.as_deref()))
                            .collect();
                        Report::Trace(TraceFit {
                            differences: differences(&measured, &simulated),
                            fits,
                            joints,
                            stiffness,
                            identified,
                            measured,
                            simulated,
                        })
                    }
                    Analysis::Sweep => {
                        let frequencies = log_spaced(inputs.sweep_from, inputs.sweep_to, inputs.sweep_count);
                        let sweep =
                            |motor: usize| frequency_sweep(entry, &config, motor, &frequencies, inputs.sweep_amplitude);
                        Report::Sweep((0..motors.len()).map(sweep).collect::<Result<_, _>>().map_err(refused)?)
                    }
                    Analysis::SweepFit => {
                        let measured = parse_sweep(&read(&inputs.sweep_file)?).map_err(|e| (e.key, e.detail))?;
                        let fit = fit_sweep(entry, &config, inputs.sweep_motor, &measured, inputs.sweep_amplitude)
                            .map_err(refused)?;
                        Report::SweepFit(inputs.sweep_motor, fit.ok_or(("fit-no-resonance", String::new()))?)
                    }
                    Analysis::Compare => Report::Compare(
                        inputs.compare_effect.clone(),
                        compare_effect(entry, &config, &inputs.compare_effect).map_err(refused)?,
                    ),
                })
            };
            let _ = result.send(run());
        });
        self.analysis_jobs.push((kind, self.config.clone(), Mutex::new(job)));
    }

    /// Forgets the result of one kind of analysis.
    pub fn clear(&mut self, kind: Analysis) {
        match kind {
            Analysis::Cycle => (self.cycle_report, self.bands, self.worst) = (None, None, None),
            Analysis::Sensitivity => self.sensitivity = None,
            Analysis::Optimise => self.optimised = None,
            Analysis::Search => self.candidates = None,
            Analysis::Path => (self.path_run, self.trail) = (None, None),
            Analysis::Trace => self.trace_fit = None,
            Analysis::Sweep => self.sweeps = None,
            Analysis::SweepFit => self.sweep_fit = None,
            Analysis::Compare => self.comparison = None,
        }
        self.computed_for[kind as usize] = None;
    }

    pub fn clear_analysis(&mut self) {
        [Analysis::Cycle, Analysis::Sensitivity, Analysis::Optimise].into_iter().for_each(|kind| self.clear(kind));
    }

    /// Names a motor of the search (and its gearbox, where it was searched with one, its drive and its axis, where it
    /// was tried on one) in `[parts]` of the config and applies it to the running simulation: the row of `search_table`
    /// after the header. A row of an axis ruled out has no motor and names nothing. The
    /// candidate passed or failed on the config that was searched, so over a config changed since in
    /// anything but its part names nothing is applied.
    pub fn use_candidate(&mut self, row: usize) {
        let (Some((found, supply)), Some(searched)) = (&self.candidates, &self.computed_for[Analysis::Search as usize])
        else {
            return;
        };
        let Some(candidate) = found.get(row).filter(|c| !c.article.is_empty()) else { return };
        if self.outdated(Analysis::Search) {
            return self.say(("ui-analysis-stale", String::new()));
        }
        // A candidate without a gearbox of its own ran with the one the searched config names.
        // Where it has none of its own (gearbox, drive, axis), it ran with the one the searched config names.
        let own = |key: &str| {
            searched.get(&format!("parts.{key}")).ok().and_then(|name| name.as_str()).unwrap_or("").to_string()
        };
        let or_own = |name: &str, key: &str| if name.is_empty() { own(key) } else { name.to_string() };
        let names = [
            candidate.article.to_string(),
            supply.clone(),
            or_own(&candidate.gearbox, "gearbox"),
            or_own(candidate.drive, "drive"),
            own("belt"),
            or_own(&candidate.axis, "axis"),
        ];
        self.name_parts(names.each_ref().map(String::as_str));
    }

    /// Names motor, supply, gearbox, drive, belt and axis in `[parts]` of the config and applies it to the running
    /// simulation. Names the catalogue does not have are refused here, before the config changes.
    fn name_parts(&mut self, names: [&str; 6]) {
        let mut config = self.config.clone();
        let named =
            PARTS.iter().zip(names).try_for_each(|(key, name)| config.set_text(&format!("parts.{key} = {name:?}")));
        match named.and_then(|()| sim_components::catalog::resolve(self.entry, &config)) {
            Ok(_) => {
                self.config = config;
                self.reconfigure();
                self.show_parts();
            }
            Err(e) => self.say((e.key(), e.detail().into())),
        }
    }

    /// Puts the settings the optimiser found into the config and applies it to the running simulation.
    /// They are absolute values, found for the config the optimiser started with: over a config edited
    /// since they would undo an edit, or fail a check they were found to pass. Then nothing is applied; the
    /// result stays, for when the edit is taken back.
    pub fn apply_optimised(&mut self) {
        if self.optimised.is_some() && self.outdated(Analysis::Optimise) {
            return self.say(("ui-analysis-stale", String::new()));
        }
        let Some(found) = self.optimised.take() else { return };
        for (path, _, after) in &found.changes {
            if let Err(e) = self.config.set_text(&format!("{path} = {after}")) {
                self.say((e.key(), e.detail().into()));
            }
        }
        self.reconfigure();
    }

    fn header(&self, keys: &[&str]) -> Vec<String> {
        keys.iter().map(|key| self.tr.tr(key).to_string()).collect()
    }

    /// "X axis · Highest velocity [m/s]" for `axis.x.max_velocity_m_s`, as the parameter panel names it.
    fn param_label(&self, path: &str) -> String {
        match path.rsplit_once('.') {
            Some(("effects", leaf)) => self.tr.tr(&format!("effect-{leaf}")).to_string(),
            Some((table, leaf)) => format!(
                "{} · {}",
                self.tr.tr(&format!("group-{}", table.replace('.', "-"))),
                self.tr.tr(&format!("param-{leaf}"))
            ),
            None => path.to_string(),
        }
    }

    /// "Figure" and the name of every motor: the header of a table with one row per figure.
    fn per_motor(&self) -> Vec<String> {
        [
            self.header(&["ui-analysis-figure"]),
            self.session.motors.iter().map(|m| self.tr.tr(m.key).to_string()).collect(),
        ]
        .concat()
    }

    fn result(&self, ok: bool) -> String {
        self.tr.tr(if ok { "ui-analysis-ok" } else { "ui-analysis-fail" }).to_string()
    }

    /// A run as tables, each narrow enough for the panel: the figures of the machine, one per row; the load
    /// on the motors, one row per figure and one column per motor; one row per check with its value, its
    /// limit and the result; and where there are error bands, a fourth with the lowest and the highest
    /// value of every check, its worst-case result (also of a check that only a bound brings up) and the
    /// share of the runs it passed, in percent. A figure
    /// the machine does not report is left out. `deviation` is that of a path, which has no cycle time and no dwell.
    fn load_tables(
        &self,
        kind: Analysis,
        load: &CycleLoad,
        verdicts: &[Verdict],
        bands: Option<&[Band]>,
        deviation: Option<f64>,
    ) -> Vec<Table> {
        let tr = |key: &str| self.tr.tr(key).to_string();
        let path = deviation.is_some();
        // The lives read the ratings of the parts the config names, as the verdicts do.
        let resolved = sim_components::catalog::resolve(
            self.entry,
            self.computed_for[kind as usize].as_ref().unwrap_or(&self.config),
        )
        .ok();
        let life = resolved.as_ref().and_then(|cfg| guide_life_km(cfg, load));
        // In the order of `CYCLE_KEYS`; lengths in mm.
        let figures = [
            (!path).then_some(load.duration),
            path.then_some(load.duration),
            deviation.map(|d| d * 1e3),
            Some(load.path_error_peak * 1e3),
            path.then_some(load.tracking_error_peak * 1e3),
            (!path).then_some(load.dwell_error_peak * 1e3),
            load.belt_tension.map(|(lowest, _)| lowest),
            load.belt_slack_time,
            load.radial_load_peak,
            load.guide_load.map(|(peak, _)| peak),
            life,
            resolved.as_ref().and_then(|cfg| guide_life_h(cfg, load)),
        ];
        let mut summary = vec![self.header(&["ui-analysis-figure", "ui-analysis-value"])];
        summary.extend(CYCLE_KEYS.iter().zip(figures).filter_map(|(key, value)| Some(vec![tr(key), short(value?)])));
        // In the order of `MOTOR_KEYS`.
        let values = |m: &MotorLoad| {
            [
                m.peak_torque,
                m.rms_torque,
                m.cubic_mean_torque,
                m.winding_rise,
                m.energy_in,
                m.energy_regenerated,
                m.resistor_energy,
                m.resistor_energy / load.duration,
                m.peak_dc_link_voltage,
                m.mains_energy,
                m.housing_rise,
                m.drive_overload,
            ]
        };
        let mut motors = vec![self.per_motor()];
        for (i, key) in MOTOR_KEYS.iter().enumerate() {
            motors.push([vec![tr(key)], load.motors.iter().map(|m| short(values(m)[i])).collect()].concat());
        }
        // In the order of `LIFE_KEYS`; a row only where some motor has the figure.
        let lives = |m: &MotorLoad| -> [Option<f64>; 5] {
            let Some(cfg) = &resolved else { return [None; 5] };
            [
                pulley_bearing_life_h(cfg, m),
                gearbox_life_h(cfg, m),
                gearbox_gearing_life_h(cfg, m),
                motor_bearing_life_h(cfg, m),
                belt_tooth_utilisation(cfg, m).ok().flatten(),
            ]
        };
        for (i, key) in LIFE_KEYS.iter().enumerate() {
            let row: Vec<Option<f64>> = load.motors.iter().map(|m| lives(m)[i]).collect();
            if row.iter().any(Option::is_some) {
                motors
                    .push([vec![tr(key)], row.into_iter().map(|v| v.map_or(String::new(), short)).collect()].concat());
            }
        }
        let motor = |k: usize| tr(self.session.motors[k].key);
        let named = |v: &Verdict| vec![tr(&format!("check-{}", v.check)), v.motor.map_or(String::new(), motor)];
        let mut checks = vec![self.header(&VERDICT_KEYS)];
        checks.extend(
            verdicts.iter().map(|v| [named(v), vec![short(v.value), short(v.limit), self.result(v.ok)]].concat()),
        );
        let mut tables = vec![summary, motors, checks];
        if let Some(bands) = bands {
            let mut banded = vec![self.header(&BAND_KEYS)];
            banded.extend(bands.iter().map(|b| {
                [
                    named(&b.verdict),
                    vec![short(b.low), short(b.high), self.result(b.ok), short(100.0 * b.share_shown(4))],
                ]
                .concat()
            }));
            tables.push(banded);
        }
        tables
    }

    /// The analysed cycle as the tables of `load_tables`. Empty until a cycle was analysed.
    pub fn cycle_tables(&self) -> Vec<Table> {
        let Some((load, verdicts, _)) = &self.cycle_report else { return Vec::new() };
        self.load_tables(Analysis::Cycle, load, verdicts, self.bands.as_deref(), None)
    }

    /// The worst corners of the bounds, where the cycle was analysed with that search: per check and motor the worst
    /// value found, its limit there and the result, and which bounded values are at their high and which at their
    /// low bound (their config paths, separated by commas).
    pub fn worst_table(&self) -> Option<Table> {
        let (worst, paths) = self.worst.as_ref()?;
        let tr = |key: &str| self.tr.tr(key).to_string();
        let mut rows = vec![self.header(&WORST_KEYS)];
        for w in worst {
            let v = &w.verdict;
            let at = |high: bool| {
                paths
                    .iter()
                    .zip(&w.high)
                    .filter(|(_, h)| **h == Some(high))
                    .map(|(path, _)| path.as_str())
                    .collect::<Vec<_>>()
                    .join(", ")
            };
            let motor = v.motor.map_or(String::new(), |k| tr(self.session.motors[k].key));
            rows.push(vec![
                tr(&format!("check-{}", v.check)),
                motor,
                short(v.value),
                short(v.limit),
                self.result(v.ok),
                at(true),
                at(false),
            ]);
        }
        Some(rows)
    }

    /// The motors of the search as ranked: passing ones first, cheapest first. Per motor its gearbox and axis (empty:
    /// the config's own), the mass of motor and gearbox, the installed mass that stands in for the price
    /// (`Candidate::cost`), result, and the check nearest to its limit with value over limit, or why the cycle was
    /// not run. An axis ruled out by its catalogue numbers has a row without motor. No more columns than the panel is
    /// wide: the cycle analysis has the rest.
    pub fn search_table(&self) -> Option<Table> {
        let (found, _) = self.candidates.as_ref()?;
        let tr = |key: &str| self.tr.tr(key).to_string();
        let mut table = vec![self.header(&SEARCH_KEYS)];
        for c in found {
            let reason = format!("ui-search-{}", c.refused);
            let (check, share) = match (c.tightest(), &c.result) {
                (Some((check, share)), _) => (tr(&format!("check-{check}")), short(share)),
                (None, Some(_)) => (String::new(), String::new()),
                (None, None) => (tr(if self.tr.has(&reason) { &reason } else { "ui-search-refused" }), String::new()),
            };
            let mass = |kg: f64| if kg.is_finite() { short(kg) } else { String::new() };
            table.push(vec![
                c.article.to_string(),
                c.gearbox.clone(),
                c.axis.clone(),
                mass(c.mass),
                mass(c.cost),
                self.result(c.passes()),
                check,
                share,
            ]);
        }
        Some(table)
    }

    /// The path that was run as the three tables of `load_tables`. Empty until a path was run.
    pub fn path_tables(&self) -> Vec<Table> {
        let Some(run) = &self.path_run else { return Vec::new() };
        self.load_tables(Analysis::Path, &run.load, &run.verdicts, None, Some(run.deviation))
    }

    /// The planned path for a plot, with the labels of its two directions: the second axis over the first,
    /// or on a machine with one axis its position over the time.
    pub fn path_plot(&self) -> Option<(String, String, Vec<[f64; 2]>)> {
        let run = self.path_run.as_ref()?;
        let axis = |i: usize| format!("{} [{}]", self.tr.tr(self.session.axes[i].key), self.session.axes[i].unit);
        Some(match self.session.axes.len() {
            1 => (
                self.tr.tr("ui-time").to_string(),
                axis(0),
                run.points.iter().map(|(time, pose)| [*time, pose[0]]).collect(),
            ),
            _ => (axis(0), axis(1), run.points.iter().map(|(_, pose)| [pose[0], pose[1]]).collect()),
        })
    }

    /// The replayed trace as two tables: per motor how far the simulation is from the trace (torque and
    /// following error, RMS and peak of the difference), and per axis and fitted value what the config has,
    /// what the trace says and the standard error of that. Empty until a trace was replayed.
    pub fn trace_tables(&self) -> Vec<Table> {
        let Some(fit) = &self.trace_fit else { return Vec::new() };
        let tr = |key: &str| self.tr.tr(key).to_string();
        let mut replayed = vec![self.per_motor()];
        for (i, key) in TRACE_KEYS.iter().enumerate() {
            replayed.push([vec![tr(key)], fit.differences.iter().map(|d| short(d[i])).collect()].concat());
        }
        let mut fitted = vec![self.header(&FIT_KEYS)];
        for (axis, fit) in self.session.axes.iter().zip(&fit.fits) {
            let Some(fit) = fit else { continue };
            for (q, key) in FIT_QUANTITIES.iter().enumerate() {
                fitted.push(vec![
                    format!("{} · {}", tr(axis.key), tr(key)),
                    short(fit.config[q]),
                    short(fit.fitted[q]),
                    short(fit.error[q]),
                ]);
            }
        }
        // In the motors' coordinates: the payload's factor (or its step), then per motor, then per link by its name.
        for v in fit.joints.iter().flatten().flat_map(|j| &j.values) {
            let of = match self.session.motors.iter().find(|m| m.id == v.of) {
                Some(m) => tr(m.key),
                None if v.of == "payload" => tr("ui-trace-payload"),
                None => v.of.clone(),
            };
            fitted.push(vec![
                format!("{of} · {}", tr(&joint_label(v.quantity))),
                short(v.config),
                short(v.fitted),
                short(v.error),
            ]);
        }
        if let Some((time, error)) = fit.joints.iter().flatten().find_map(|j| j.payload_step) {
            fitted.push(vec![tr("ui-trace-payload-step"), String::new(), short(time), short(error)]);
        }
        if let Some(Ok(s)) = &fit.stiffness {
            fitted.push(vec![tr("ui-trace-stiffness-factor"), short(1.0), short(s.factor), short(s.error)]);
        }
        let mut identified = vec![self.header(&IDENTIFY_KEYS)];
        for (motor, found) in self.session.motors.iter().zip(&fit.identified) {
            let named = |name: &str| (tr(motor.key), tr(&format!("identify-{name}")));
            for (name, e) in found.estimates() {
                let (low, high) = e.interval();
                let (motor, quantity) = named(name);
                identified.push(vec![
                    motor,
                    quantity,
                    short(e.value),
                    short(e.error),
                    format!("{} … {}", short(low), short(high)),
                ]);
            }
            for (name, value) in found.qualities() {
                let (motor, quantity) = named(name);
                identified.push(vec![motor, quantity, short(value), String::new(), String::new()]);
            }
        }
        vec![replayed, fitted, identified]
    }

    /// What the fit says to change in the config, one line per value, and which axis it could not fit.
    pub fn trace_advice(&self) -> Vec<String> {
        let Some(fit) = &self.trace_fit else { return Vec::new() };
        let mut lines: Vec<(&str, String)> = self
            .session
            .axes
            .iter()
            .zip(&fit.fits)
            .flat_map(|(axis, fit)| match fit {
                Some(fit) => fit.advice(axis.id).to_vec(),
                None => vec![("fit-axis-skipped", axis.id.to_string())],
            })
            .collect();
        match &fit.joints {
            Some(Some(joints)) => lines.extend(joints.advice()),
            Some(None) => lines.push(("fit-joints-skipped", String::new())),
            None => {}
        }
        match &fit.stiffness {
            Some(Ok(s)) => lines.push(s.advice()),
            Some(Err(e)) => lines.push((e.key, e.detail.clone())),
            None => {}
        }
        lines.into_iter().map(|(key, detail)| self.tr.tr_arg(key, &detail)).collect()
    }

    /// Measured against simulated, per quantity of `OVERLAY_KEYS` (torque, speed, following error at the motor)
    /// and per motor: (time, measured value) and (time, simulated value). Empty until a trace was replayed.
    pub fn overlay(&self) -> Vec<(&'static str, Vec<[Vec<[f64; 2]>; 2]>)> {
        let Some(fit) = &self.trace_fit else { return Vec::new() };
        let time = &fit.measured.time;
        let line = |values: &[f64]| time.iter().zip(values).map(|(t, v)| [*t, *v]).collect::<Vec<_>>();
        let columns = |m: &sim_components::trace::MotorTrace| [m.torque.clone(), m.speed.clone(), m.following_error()];
        let pairs: Vec<_> =
            fit.measured.motors.iter().zip(&fit.simulated.motors).map(|(m, s)| (columns(m), columns(s))).collect();
        OVERLAY_KEYS
            .iter()
            .enumerate()
            .map(|(q, key)| (*key, pairs.iter().map(|(m, s)| [line(&m[q]), line(&s[q])]).collect()))
            .collect()
    }

    /// Writes the overlay as `sim-cli replay --out` does.
    pub fn export_overlay(&mut self, path: &std::path::Path) {
        let Some(fit) = &self.trace_fit else { return };
        let ids: Vec<&str> = self.session.motors.iter().map(|m| m.id).collect();
        let message = match std::fs::write(path, overlay_csv(&ids, &fit.measured, &fit.simulated)) {
            Ok(()) => ("ui-overlay-exported", path.display().to_string()),
            Err(e) => ("ui-export-failed", e.to_string()),
        };
        self.say(message);
    }

    /// The operating points of the analysed cycle, as `sim-cli analyze --points` writes them, motor after motor:
    /// how many there are, and the header with the rows in `rows`. `None` until a cycle was analysed.
    pub fn points_table(&self, rows: std::ops::Range<usize>) -> Option<(usize, Table)> {
        let (load, _, _) = self.cycle_report.as_ref()?;
        let all = self
            .session
            .motors
            .iter()
            .zip(&load.motors)
            .flat_map(|(motor, m)| m.points.iter().map(move |p| (motor, p)));
        let shown = all
            .clone()
            .skip(rows.start)
            .take(rows.len())
            .map(|(motor, (speed, torque))| vec![self.tr.tr(motor.key).to_string(), short(*speed), short(*torque)]);
        Some((all.count(), [vec![self.header(&POINTS_KEYS)], shown.collect()].concat()))
    }

    /// The operating points per motor as (speed, torque), and the torque-speed curves (`Curves`): of the analysed
    /// cycle (`Analysis::Cycle`) or of the path that was run (`Analysis::Path`). `None` until there is one.
    pub fn points_plot(&self, kind: Analysis) -> Option<(Vec<Vec<[f64; 2]>>, Curves)> {
        let (load, curves) = match kind {
            Analysis::Path => self.path_run.as_ref().map(|run| (&run.load, &run.curves))?,
            _ => self.cycle_report.as_ref().map(|(load, _, curves)| (load, curves))?,
        };
        let points =
            load.motors.iter().map(|m| m.points.iter().map(|(speed, torque)| [*speed, *torque]).collect()).collect();
        Some((points, curves.clone()))
    }

    /// The parts of each motor's torque over the scopes' time window: per motor its name with the unit, and per
    /// part of `TORQUE_PARTS` and then the torque itself the label and the points (time, torque). Empty for a machine
    /// that does not report the parts.
    pub fn breakdown_lines(&self) -> Vec<(String, Vec<(String, Vec<[f64; 2]>)>)> {
        let signals = &self.session.signals;
        let rows: Vec<&Vec<f64>> = self.visible_rows().collect();
        // At most about three thousand points per line, as in the scopes.
        let stride = (rows.len() / 3000).max(1);
        let lines = |motor: &sim_core::MotorInfo| -> Option<(String, Vec<(String, Vec<[f64; 2]>)>)> {
            let ids = TORQUE_PARTS
                .iter()
                .map(|part| format!("motor.{}.torque_{part}", motor.id))
                .chain([format!("motor.{}.torque", motor.id)]);
            let places: Vec<usize> = ids.map(|id| signals.iter().position(|s| s.id == id)).collect::<Option<_>>()?;
            let title = format!("{} [{}]", self.tr.tr(motor.key), signals[places[4]].unit);
            Some((
                title,
                places
                    .iter()
                    .zip(BREAKDOWN_KEYS)
                    .map(|(&i, key)| {
                        (self.tr.tr(key).to_string(), rows.iter().step_by(stride).map(|row| [row[0], row[i]]).collect())
                    })
                    .collect(),
            ))
        };
        self.session.motors.iter().filter_map(lines).collect()
    }

    /// The parts of each motor's torque over the analysed cycle: one row per part and one for the torque itself, per
    /// motor its peak and its RMS in N m. `None` until a cycle was analysed, and for a machine that does not report the
    /// parts.
    pub fn breakdown_table(&self) -> Option<Table> {
        let (load, _, _) = self.cycle_report.as_ref()?;
        let parts: Vec<[(f64, f64); 4]> = load.motors.iter().map(|m| m.torque_parts).collect::<Option<_>>()?;
        let tr = |key: &str| self.tr.tr(key).to_string();
        let mut header = vec![tr("ui-effects-part")];
        for motor in &self.session.motors {
            header.extend(["ui-effects-peak", "ui-effects-rms"].map(|key| format!("{} · {}", tr(motor.key), tr(key))));
        }
        let mut table = vec![header];
        for (i, key) in BREAKDOWN_KEYS.iter().enumerate() {
            let cells = load.motors.iter().zip(&parts).flat_map(|(m, p)| {
                let (peak, rms) = p.get(i).copied().unwrap_or((m.peak_torque, m.rms_torque));
                [short(peak), short(rms)]
            });
            table.push([vec![tr(key)], cells.collect()].concat());
        }
        Some(table)
    }

    /// Where the energy drawn from the supply over the analysed cycle went: the supply, every row of the budget that
    /// is not exactly zero (an effect that is off), and what is not accounted for; in J and in percent of the supply.
    /// `None` until a cycle was analysed, and for a machine that does not report its losses.
    pub fn budget_table(&self) -> Option<Table> {
        let budget = self.cycle_report.as_ref()?.0.budget.as_ref()?;
        let tr = |key: &str| self.tr.tr(key).to_string();
        let line = |label: String, energy: f64| vec![label, short(energy), short(100.0 * energy / budget.supply)];
        let mut table = vec![
            self.header(&["ui-effects-item", "ui-effects-energy", "ui-effects-share"]),
            line(tr("budget-supply"), budget.supply),
        ];
        table.extend(
            budget
                .rows
                .iter()
                .filter(|r| r.energy != 0.0)
                .map(|r| line(format!("{} · {}", tr(&r.owner_key), tr(&r.quantity_key)), r.energy)),
        );
        table.push(line(tr("budget-unaccounted"), budget.unaccounted()));
        Some(table)
    }

    /// The energy budget of the analysed cycle as a flow diagram, in three columns of (label, energy in J), every
    /// energy above zero. The first holds the supply and every store that gave energy back over the cycle (a row of the
    /// budget below zero); the second what the electrical side lost or stored (the rows `EnergyBudget::
    /// electrical_unaccounted` counts, and what that leaves), and last the shaft work, with what the machine's stores
    /// gave back; the third what the machine lost or stored, and what its own balance leaves. So the first column adds
    /// up to the second, the second's last to the third, and the second and third less what the first has beyond the
    /// supply are the supply, as the budget's rows and what it leaves unaccounted are. Rows that are exactly zero (an
    /// effect that is off) are left out, as in `budget_table`. `None` where that has nothing.
    pub fn budget_flows(&self) -> Option<[Vec<(String, f64)>; 3]> {
        let budget = self.cycle_report.as_ref()?.0.budget.as_ref()?;
        let tr = |key: &str| self.tr.tr(key).to_string();
        let electrical = |id: &str| {
            id.starts_with("copper.")
                || matches!(id, "resistor" | "discharge" | "mains" | "line_module" | "windings" | "capacitors")
        };
        let unaccounted = |energy: f64| (tr("budget-unaccounted"), energy);
        let stage = |side: bool, rest: f64| -> Vec<(String, f64)> {
            let rows = budget.rows.iter().filter(|r| r.energy != 0.0 && electrical(&r.id) == side);
            rows.map(|r| (format!("{} · {}", tr(&r.owner_key), tr(&r.quantity_key)), r.energy))
                .chain([unaccounted(rest)])
                .collect()
        };
        let electrical_rest = budget.electrical_unaccounted();
        let (supplied, machine) = (stage(true, electrical_rest), stage(false, budget.unaccounted() - electrical_rest));
        let given_back = |items: &[(String, f64)]| {
            items.iter().filter(|(_, e)| *e < 0.0).map(|(label, e)| (label.clone(), -e)).collect::<Vec<_>>()
        };
        let taken = |items: &[(String, f64)]| items.iter().filter(|(_, e)| *e > 0.0).cloned().collect::<Vec<_>>();
        let back: f64 = given_back(&machine).iter().map(|(_, e)| e).sum();
        let first = [vec![(tr("budget-supply"), budget.supply)], given_back(&supplied), given_back(&machine)].concat();
        let second = [taken(&supplied), vec![(tr("ui-effects-shaft-work"), budget.shaft_work + back)]].concat();
        Some([first, second, taken(&machine)])
    }

    /// A horizontal slice through what the machine can reach, at the height `inputs.workspace_z` (the middle of the
    /// third axis's range where it is `None`, and within that range), for the workspace map: per node of a grid the
    /// condition number of the kinematics there (`Machine::condition`). The grid spans the first two axes' ranges and
    /// half of their width beyond each end, so that the reach shows where it goes past the travel range. `None` for a
    /// machine with fewer than three axes or no condition to tell, and for a config it does not build.
    pub fn workspace_slice(&self, n: usize) -> Option<WorkspaceSlice> {
        let machine = self.machine()?;
        let axes = machine.axes();
        let heights = (axes.get(2)?.min, axes[2].max);
        let z = self.inputs.workspace_z.unwrap_or((heights.0 + heights.1) / 2.0).clamp(heights.0, heights.1);
        machine.condition(&[0.0, 0.0, z])?;
        let span = |a: &sim_core::AxisInfo| (a.min - (a.max - a.min) / 2.0, a.max + (a.max - a.min) / 2.0);
        let (x, y) = (span(&axes[0]), span(&axes[1]));
        let at = |(low, high): (f64, f64), i: usize| low + (high - low) * i as f64 / (n - 1) as f64;
        let mut pose = vec![0.0; axes.len()];
        let mut nodes = Vec::with_capacity(n * n);
        for j in 0..n {
            for i in 0..n {
                (pose[0], pose[1], pose[2]) = (at(x, i), at(y, j), z);
                nodes.push((
                    machine.condition(&pose).unwrap_or(f64::INFINITY),
                    machine.unreachable(&pose, 0.0).is_none(),
                ));
            }
        }
        let limit = self.config.f64("ratings.max_condition_number").unwrap_or(10.0);
        Some(WorkspaceSlice { x, y, z, n, nodes, limit, heights })
    }

    /// Per friction contact of the machine its curve of steady sliding and where it is now (see `FrictionView`). The
    /// curve runs over the axis's speed limit both ways, with its points closer together near standstill, where the
    /// Stribeck drop is. Empty for a machine that tells no contacts, or a config it does not build.
    pub fn friction_views(&self) -> Vec<FrictionView> {
        let Ok(machine) =
            sim_components::catalog::resolve(self.entry, &self.config).and_then(|cfg| (self.entry.build)(&cfg))
        else {
            return Vec::new();
        };
        let tr = |key: &str| self.tr.tr(key).to_string();
        let value = |id: String| -> Option<f64> {
            Some(self.history.back()?[self.session.signals.iter().position(|s| s.id == id)?])
        };
        let views = machine.friction_contacts().into_iter().enumerate().map(|(i, contact)| {
            let get = |part: &str| value(format!("{}.friction_{part}", contact.id));
            let scale = get("load_scale").unwrap_or(1.0);
            // shortcut: a contact on no axis of its own (a delta's carriage) spans the fastest axis's limit, which a
            // carriage can exceed; a speed limit per contact from the machine would be exact.
            let fastest = self.session.axes.iter().map(|a| a.max_vel).fold(0.0, f64::max);
            let top = self.session.axes.iter().find(|a| a.key == contact.owner_key).map_or(fastest, |a| a.max_vel);
            let speeds = (1..=100)
                .rev()
                .map(|j| -top * (j as f64 / 100.0).powi(3))
                .chain((1..=100).map(|j| top * (j as f64 / 100.0).powi(3)));
            let mut points: [Vec<[f64; 2]>; 4] = Default::default();
            for v in speeds {
                let parts = machine.friction_curve(i, v, scale);
                points[0].push([v, parts.iter().sum()]);
                (0..3).for_each(|k| points[k + 1].push([v, parts[k]]));
            }
            let labels = [tr("ui-effects-total")].into_iter().chain(FRICTION_KEYS.map(tr));
            let split: Option<Vec<f64>> = ["coulomb", "viscous", "stiction"].iter().map(|part| get(part)).collect();
            let (speed, total) = (get("speed"), split.as_ref().map(|s| s.iter().sum::<f64>()));
            let mut table = vec![self.header(&["ui-effects-part", "ui-effects-now"])];
            let cell = |value: Option<f64>| value.map_or(String::new(), short);
            table.push(vec![format!("{} [m/s]", tr("quantity-friction-speed")), cell(speed)]);
            for (k, key) in FRICTION_KEYS.iter().enumerate() {
                table.push(vec![format!("{} [N]", tr(key)), cell(split.as_ref().map(|s| s[k]))]);
            }
            table.push(vec![format!("{} [N]", tr("ui-effects-total")), cell(total)]);
            table.push(vec![tr("quantity-friction-load-scale"), cell(get("load_scale"))]);
            let mut curves = labels.zip(points);
            FrictionView {
                title: tr(contact.owner_key),
                curves: std::array::from_fn(|_| curves.next().unwrap()),
                now: speed.zip(total).map(|(v, f)| [v, f]),
                table,
            }
        });
        views.collect()
    }

    /// The machine as the config builds it, for what it tells the views; `None` for a config it does not build.
    fn machine(&self) -> Option<Box<dyn Machine>> {
        sim_components::catalog::resolve(self.entry, &self.config).and_then(|cfg| (self.entry.build)(&cfg)).ok()
    }

    /// The newest sample of a signal, by its id.
    fn latest(&self, id: &str) -> Option<f64> {
        Some(self.history.back()?[self.session.signals.iter().position(|s| s.id == id)?])
    }

    /// The points (time, value times `scale`) of a signal over the scopes' window, at most about three thousand.
    fn line_of(&self, id: &str, scale: f64) -> Vec<[f64; 2]> {
        let Some(i) = self.session.signals.iter().position(|s| s.id == id) else { return Vec::new() };
        let rows: Vec<&Vec<f64>> = self.visible_rows().collect();
        rows.iter().step_by((rows.len() / 3000).max(1)).map(|row| [row[0], row[i] * scale]).collect()
    }

    /// The thermal view (see `ThermalView`). The windings are rated for `motor.rated_temperature_rise_k`, the parts
    /// for `thermal.<kind>_max_rise_k`; a part settles at its thermal resistance times its mean loss over the analysed
    /// cycle (`CycleLoad::part_rises`), a winding at `MotorLoad::winding_rise`. The share of the rating is that of the
    /// settled rise, which is what the verdicts judge (the rise now is a climb of many minutes); above 100 % it is
    /// marked.
    pub fn thermal_view(&self) -> ThermalView {
        let tr = |key: &str| self.tr.tr(key).to_string();
        let on = |effect: &str| self.config.bool(&format!("effects.{effect}")).unwrap_or(false);
        let load = self.cycle_report.as_ref().map(|r| &r.0);
        let kelvin = |key: &str, unit: &str| format!("{} [{unit}]", tr(key));
        let mut table = vec![vec![
            tr("ui-effects-component"),
            kelvin("ui-effects-now", "K"),
            kelvin("ui-effects-settles", "K"),
            kelvin("ui-effects-rated", "K"),
            kelvin("ui-effects-settled-of-rated", "%"),
        ]];
        let mut lines = Vec::new();
        let mut add = |name: String, id: String, settles: Option<f64>, rated: Option<f64>| {
            let now = self.latest(&id);
            let share = settles.zip(rated).map(|(settles, rated)| 100.0 * settles / rated);
            let mut row: Vec<String> = [name.clone()]
                .into_iter()
                .chain([now, settles, rated, share].map(|v| v.map_or(String::new(), short)))
                .collect();
            if share.is_some_and(|share| share > 100.0) {
                row[4] = format!("{} {}", row[4], tr("ui-effects-over-rating"));
            }
            table.push(row);
            lines.push((name, self.line_of(&id, 1.0)));
        };
        let winding_rated = self.config.positive("motor.rated_temperature_rise_k").ok();
        for (k, motor) in self.session.motors.iter().enumerate() {
            let settles = load.and_then(|l| l.motors.get(k)).map(|m| m.winding_rise);
            add(
                format!("{} · {}", tr(motor.key), tr("quantity-winding-temperature")),
                format!("motor.{}.winding_temperature", motor.id),
                settles,
                winding_rated,
            );
        }
        // The temperature feedback warms the parts too (`sim::heats`).
        let parts = on("part_heating") || on("thermal_feedback");
        if parts {
            for part in self.machine().map(|m| m.thermal_parts()).unwrap_or_default() {
                let key = |name: &str| format!("thermal.{}_{name}", part.kind);
                let settles =
                    load.and_then(|l| l.part_rises.iter().find(|(p, _)| p.id == part.id)).map(|(_, rise)| *rise);
                let name = format!("{} · {}", tr(part.owner_key), tr(&format!("quantity-{}-temperature", part.kind)));
                add(
                    name,
                    format!("{TEMPERATURE_PREFIX}{}", part.id),
                    settles,
                    self.config.positive(&key("max_rise_k")).ok(),
                );
            }
        }
        ThermalView { lines, table, off: !on("winding_heating") && !parts }
    }

    /// The belt view (see `BeltView`); `None` for a machine that tells no belt sections. Stretch in mm.
    pub fn belt_view(&self) -> Option<BeltView> {
        let sections = self.machine()?.belt_sections();
        let pretension = sections.first()?.pretension;
        let rating = self.config.positive("ratings.belt_max_tension_n").ok();
        let tr = |key: &str| self.tr.tr(key).to_string();
        let unit = |key: &str, unit: &str| format!("{} [{unit}]", tr(key));
        let mut table = vec![vec![
            tr("ui-effects-section"),
            unit("ui-effects-tension", "N"),
            unit("ui-effects-pretension", "N"),
            unit("ui-effects-rating", "N"),
            unit("ui-effects-of-rated", "%"),
            unit("ui-effects-stretch", "mm"),
        ]];
        let (mut tension, mut stretch) = (Vec::new(), Vec::new());
        for section in &sections {
            let (pull, long) = (format!("{}.tension", section.id), format!("{}.stretch", section.id));
            let now = self.latest(&pull);
            let cells = [
                now,
                Some(section.pretension),
                rating,
                now.zip(rating).map(|(t, r)| 100.0 * t / r),
                self.latest(&long).map(|e| e * 1e3),
            ];
            table.push(
                [tr(section.owner_key)].into_iter().chain(cells.map(|v| v.map_or(String::new(), short))).collect(),
            );
            tension.push((tr(section.owner_key), self.line_of(&pull, 1.0)));
            stretch.push((tr(section.owner_key), self.line_of(&long, 1e3)));
        }
        let titles = [unit("ui-effects-tension", "N"), unit("ui-effects-stretch", "mm")];
        let scale = [
            format!("{} 0 N", tr("ui-effects-slack")),
            format!("{} {:.0} N", tr("ui-effects-double"), 2.0 * pretension),
        ];
        Some(BeltView { tension, stretch, pretension, rating, table, titles, scale })
    }

    /// The effect comparison (see `CompareView`) for the quantity `inputs.compare_quantity`; `None` before one was run,
    /// and for a quantity the comparison has no signal of.
    pub fn compare_view(&self) -> Option<CompareView> {
        let (effect, found) = self.comparison.as_ref()?;
        let tr = |key: &str| self.tr.tr(key).to_string();
        let scope = scopes_for(&found.signals).into_iter().find(|s| s.quantity_key == self.inputs.compare_quantity)?;
        let unit = |key: &str| format!("{} [{}]", tr(key), scope.unit);
        let mut table =
            vec![vec![tr("ui-effects-signal"), unit("ui-effects-difference-peak"), unit("ui-effects-difference-rms")]];
        let mut lines = Vec::new();
        let stride = (found.on.len() / 3000).max(1);
        for line in scope.lines.iter().filter(|l| !l.reference) {
            let i = line.signal;
            let points =
                |rows: &Vec<Vec<f64>>| rows.iter().step_by(stride).map(|row| [row[0], row[i]]).collect::<Vec<_>>();
            let name = tr(&found.signals[i].owner_key);
            let (peak, rms) = found.difference(i);
            table.push(vec![name.clone(), short(peak), short(rms)]);
            lines.push((name, points(&found.on), points(&found.off)));
        }
        Some(CompareView {
            effect: tr(&format!("effect-{effect}")),
            title: format!("{} [{}]", tr(&scope.quantity_key), scope.unit),
            lines,
            table,
        })
    }

    /// One row per figure, one column per motor: the resonance of the load on the belt in each frequency
    /// response (`identify::resonance`), and its damping ratio. Empty where a response has no dip with both
    /// flanks. `None` until a sweep was run.
    pub fn sweep_table(&self) -> Option<Table> {
        let sweeps = self.sweeps.as_ref()?;
        let found: Vec<_> = sweeps.iter().map(|points| resonance(points)).collect();
        let cells = |value: fn(&sim_components::identify::Resonance) -> f64| {
            found.iter().map(move |r| r.as_ref().map_or(String::new(), |r| short(value(r))))
        };
        let rows = [
            (SWEEP_KEYS[0], cells(|r| r.frequency).collect::<Vec<_>>()),
            (SWEEP_KEYS[1], cells(|r| r.damping).collect()),
        ];
        Some(
            [
                vec![self.per_motor()],
                rows.into_iter().map(|(key, cells)| [vec![self.tr.tr(key).to_string()], cells].concat()).collect(),
            ]
            .concat(),
        )
    }

    /// The fit of a measured frequency response, as `sim-cli fit --sweep` writes it: resonance and damping of
    /// the config's own response and of the measured one, and the standard error of the frequency.
    pub fn sweep_fit_table(&self) -> Option<Table> {
        let (motor, fit) = self.sweep_fit.as_ref()?;
        let name = |key: &str| format!("{} · {}", self.tr.tr(self.session.motors[*motor].key), self.tr.tr(key));
        Some(vec![
            self.header(&FIT_KEYS),
            vec![
                name(SWEEP_KEYS[0]),
                short(fit.config.frequency),
                short(fit.fitted.frequency),
                short(fit.fitted.frequency_error),
            ],
            vec![name(SWEEP_KEYS[1]), short(fit.config.damping), short(fit.fitted.damping), String::new()],
        ])
    }

    /// What the fit of a measured frequency response says to change in the config.
    pub fn sweep_fit_advice(&self) -> Option<String> {
        let (key, detail) = self.sweep_fit.as_ref()?.1.advice();
        Some(self.tr.tr_arg(key, &detail))
    }

    /// The parameters the cycle depends on most, strongest first: the low and high value each was run
    /// with, and what that did to peak torque, RMS torque and cycle time, in percent.
    pub fn sensitivity_table(&self) -> Option<Table> {
        let (nominal, rows) = self.sensitivity.as_ref()?;
        let mut table = vec![self.header(&SENSITIVITY_KEYS)];
        for row in rows.iter().take(TOP_ROWS) {
            let c = row.changes(nominal).map(|change| change * 100.0);
            let pair = |low: f64, high: f64| format!("{low:+.1} / {high:+.1}");
            table.push(vec![
                self.param_label(&row.path),
                short(row.bounds.0),
                short(row.bounds.1),
                pair(c[0], c[1]),
                pair(c[2], c[3]),
                pair(c[4], c[5]),
            ]);
        }
        Some(table)
    }

    /// The optimiser's result: cycle time and settling before and after, and every setting it would change.
    pub fn optimised_table(&self) -> Option<Table> {
        let found = self.optimised.as_ref()?;
        let mut table = vec![self.header(&OPTIMISED_KEYS)];
        table.push(vec![
            self.tr.tr("ui-analysis-cycle-time").to_string(),
            short(found.before.duration),
            short(found.after.duration),
        ]);
        table.push(vec![
            self.tr.tr("ui-analysis-dwell-error").to_string(),
            short(found.before.dwell_error_peak * 1e3),
            short(found.after.dwell_error_peak * 1e3),
        ]);
        // The new cycle itself is applied whole; its lifts say what changed in it (segments counted from 1).
        for (path, before, after) in found.changes.iter().filter(|c| c.0 != "cycle.segment") {
            table.push(vec![self.param_label(path), before.clone(), after.clone()]);
        }
        for lift in &found.lifts {
            let segments = lift.segments.map(|i| (i + 1).to_string()).join(", ");
            table.push(vec![self.tr.tr_arg("optimised-lift", &segments), short(lift.before), short(lift.after)]);
            if let Some(via) = &lift.via {
                table.push(vec![
                    self.tr.tr_arg("optimised-via", &segments),
                    String::new(),
                    via.iter()
                        .map(|point| point.iter().map(|x| short(*x)).collect::<Vec<_>>().join(", "))
                        .collect::<Vec<_>>()
                        .join("; "),
                ]);
            }
        }
        // Judged in the error bands: the fastest setting on the config as written, what it fails there, and what the
        // cycle as configured fails there already (not judged).
        if let Some(robust) = &found.robust {
            let checks = |checks: &[&str]| {
                checks.iter().map(|c| self.tr.tr(&format!("check-{c}")).to_string()).collect::<Vec<_>>().join(", ")
            };
            table.push(vec![
                self.tr.tr("optimised-fastest").to_string(),
                String::new(),
                short(robust.nominal_duration),
            ]);
            table.push(vec![
                self.tr
                    .tr(if robust.worst { "optimised-fastest-fails-worst" } else { "optimised-fastest-fails" })
                    .to_string(),
                String::new(),
                checks(&robust.nominal_fails),
            ]);
            let excused = if robust.worst { "optimised-excused-worst" } else { "optimised-excused" };
            table.push(vec![self.tr.tr(excused).to_string(), String::new(), checks(&robust.excused)]);
        }
        Some(table)
    }

    /// The optimiser's front of energy against cycle time (`Optimised::front`): (cycle time, energy per cycle) of each
    /// point, and the result's cycle time.
    pub fn optimised_front(&self) -> Option<(Vec<[f64; 2]>, f64)> {
        let found = self.optimised.as_ref().filter(|found| !found.front.is_empty())?;
        Some((found.front.iter().map(|p| [p.duration, p.energy]).collect(), found.after.duration))
    }

    // ---------------------------------------------------------------------------------------
    // Labels and panels

    /// "X axis · Following error [m]"
    pub fn signal_label(&self, signal: &SignalInfo) -> String {
        format!("{} · {} [{}]", self.tr.tr(&signal.owner_key), self.tr.tr(&signal.quantity_key), signal.unit)
    }

    /// The effect switches: (config path, label key, on).
    pub fn effects(&self) -> Vec<(String, String, bool)> {
        let leaves = self.config.leaves().into_iter();
        leaves
            .filter_map(|(path, value)| {
                Some((path.clone(), format!("effect-{}", path.strip_prefix("effects.")?), value.as_bool()?))
            })
            .collect()
    }

    /// The parameter panel: every numeric config value, grouped by its table.
    pub fn param_groups(&self) -> Vec<ParamGroup> {
        let mut groups: Vec<ParamGroup> = Vec::new();
        for (path, value) in self.config.leaves() {
            let Some((table, leaf)) = path.rsplit_once('.') else { continue };
            if table == "effects" || !(value.is_float() || value.is_integer()) {
                continue;
            }
            let key = format!("group-{}", table.replace('.', "-"));
            let param = (path.clone(), format!("param-{leaf}"), value);
            match groups.iter_mut().find(|g| g.key == key) {
                Some(group) => group.params.push(param),
                None => groups.push(ParamGroup { key, params: vec![param] }),
            }
        }
        groups
    }

    /// Every translation key the panels and the 3D view can show for the current machine.
    pub fn keys_in_use(&self) -> Vec<String> {
        let mut keys = vec![self.entry.key.to_string()];
        keys.extend(MACHINES.iter().map(|m| m.key.to_string()));
        keys.extend(self.session.signals.iter().flat_map(|s| [s.owner_key.clone(), s.quantity_key.clone()]));
        keys.extend(self.scopes.iter().map(|s| s.quantity_key.clone()));
        keys.extend(self.scene.labels.iter().flat_map(|l| [l.owner_key.to_string(), l.unit_key.to_string()]));
        keys.extend(self.effects().into_iter().map(|(_, key, _)| key));
        for group in self.param_groups() {
            keys.push(group.key);
            keys.extend(group.params.into_iter().map(|(_, key, _)| key));
        }
        let headers = [
            &CYCLE_KEYS[..],
            &MOTOR_KEYS,
            &LIFE_KEYS,
            &VERDICT_KEYS,
            &BAND_KEYS,
            &WORST_KEYS,
            &SENSITIVITY_KEYS,
            &OPTIMISED_KEYS,
            &SEARCH_KEYS,
            &TRACE_KEYS,
            &FIT_KEYS,
            &FIT_QUANTITIES,
            &IDENTIFY_KEYS,
            &NOTES,
            &SWEEP_KEYS,
            &POINTS_KEYS,
            &OVERLAY_KEYS,
            &BREAKDOWN_KEYS,
            &FRICTION_KEYS,
            &EFFECT_KEYS,
            &VIEW_KEYS,
        ];
        keys.extend(headers.concat().into_iter().map(String::from));
        keys.extend(
            [
                "ui-analysis-ok",
                "ui-analysis-fail",
                "fit-axis-skipped",
                "fit-mass",
                "fit-viscous",
                "fit-coulomb",
                "fit-constant",
                "fit-stiffness",
                "fit-no-resonance",
            ]
            .map(String::from),
        );
        keys.extend(
            [
                "fit-joints-skipped",
                "fit-payload",
                "fit-joint-inertia",
                "fit-joint-viscous",
                "fit-joint-coulomb",
                "fit-joint-constant",
                "fit-alike",
                "fit-payload-step",
                "fit-link-mass",
                "fit-link-moment",
                "fit-combined",
                "fit-no-effect",
                "ui-trace-payload-step",
            ]
            .map(String::from),
        );
        keys.extend(
            [
                "fit-move-not-excited",
                "fit-move-no-key",
                "fit-move-stiffness",
                "ui-trace-payload",
                "ui-trace-stiffness-factor",
                "ui-trace-stiffness",
                "ui-trace-stiffness-hint",
            ]
            .map(String::from),
        );
        keys.extend(
            ["payload_factor"]
                .iter()
                .chain(&sim_components::identify::PAYLOAD_STEP_QUANTITIES)
                .chain(&sim_components::identify::JOINT_QUANTITIES)
                .chain(&sim_components::identify::LINK_QUANTITIES)
                .map(|q| joint_label(q)),
        );
        keys.extend(CHECKS.iter().map(|check| format!("check-{check}")));
        keys.extend(QUANTITIES.iter().map(|name| format!("identify-{name}")));
        keys.extend(["error-import-map", "error-import-signal", "error-import-unit"].map(String::from));
        let budget = self.cycle_report.as_ref().and_then(|(load, ..)| load.budget.as_ref());
        keys.extend(
            budget.into_iter().flat_map(|b| b.rows.iter().flat_map(|r| [r.owner_key.clone(), r.quantity_key.clone()])),
        );
        keys
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    /// A background job whose thread ended without an answer (it panicked) left no message: the analysis
    /// was dropped without a word, and the tuning's spinner turned for ever.
    #[test]
    fn a_job_that_ends_without_an_answer_is_reported() {
        let mut model = Model::new(MACHINES[0], "en");
        // What is left of a thread that panicked: the receiving end of its channel.
        model.analysis_jobs.push((Analysis::Cycle, model.config.clone(), Mutex::new(channel().1)));
        model.tuning_job = Some(Mutex::new(channel().1));
        assert!(model.analysis_running() && model.tuning_running());
        model.poll();
        assert!(!model.analysis_running() && !model.tuning_running());
        assert_eq!(model.messages.last(), Some(&("ui-job-ended", String::new())), "{:?}", model.messages);
        // A job that answers is taken as before.
        let (result, job) = channel();
        result.send(Err(("error-busy", String::new()))).unwrap();
        model.analysis_jobs.push((Analysis::Cycle, model.config.clone(), Mutex::new(job)));
        model.poll();
        assert_eq!((model.analysis_running(), model.messages.last().unwrap().0), (false, "error-busy"));
    }

    /// A simulation thread that is gone (it panicked) left a frozen view and no message. It is reported, once.
    #[test]
    fn a_simulation_thread_that_ends_is_reported_once() {
        let mut model = Model::new(MACHINES[0], "en");
        model.poll();
        let reports = |model: &Model| model.messages.iter().filter(|m| m.0 == "ui-simulation-ended").count();
        assert_eq!(reports(&model), 0);
        // The thread ends while the session lives, which nothing but a panic does to it otherwise.
        model.session.send(Request::Quit);
        let start = Instant::now();
        while !model.session.ended() && start.elapsed() < Duration::from_secs(5) {
            std::thread::sleep(Duration::from_millis(5));
        }
        model.poll();
        assert_eq!(reports(&model), 1, "{:?}", model.messages);
        model.command(Command::MoveTo(Vec::new()));
        model.set_param("no.such_value", Value::Float(1.0));
        model.poll();
        assert_eq!(reports(&model), 1, "{:?}", model.messages);
    }
}
