//! The search workflow in the app beside the ranked table of the search tab: every option `sim-cli search` takes
//! (`SearchInputs`), the figures of each candidate as a heat map of its checks (`heatmap`), cost against margin with the
//! front (`scatter`), the top candidates' operating points on their torque-speed curves (`Found::curves`) and why each
//! failed (`failures`); the sweep of a gear ratio or pulley radius (`sim-cli transmission`) and the comparison of
//! kinematics on one task (`sim-cli compare-kinematics`); and for the path tab maps of peak torque and following error
//! over a plane of the workspace, from a grid of short moves (`move_map`). The window draws them in `ui/search_views.rs`.

use crate::model::{Curves, Inputs, Message, Table, short, torque_speed_curves};
use sim_components::analysis::{Verdict, utilisation};
use sim_components::catalog::{self, Candidate, Drives, Options, SUPPLIES, vendor_of};
use sim_components::search::{self, KinematicsRow, Task, TransmissionSweep};
use sim_components::{Command, Sim};
use sim_core::{Config, ConfigError, MachineEntry, Translations};
use sim_machines::MACHINES;
use std::sync::Mutex;
use std::sync::mpsc::{Receiver, TryRecvError, channel};

/// The search's options beyond those of `Inputs` (supply, series, gearbox series, axes), as `sim-cli search` takes them,
/// and the inputs of the ratio sweep and of the kinematics comparison.
#[derive(Debug, Clone, PartialEq)]
pub struct SearchInputs {
    /// A maker (`vendor_of`): every supply of that maker is searched, ranked together; "all" every supply of every
    /// maker; "" the supply of `Inputs` alone.
    pub vendor: String,
    /// The drives tried with each motor (`--drives`).
    pub drives: Drives,
    /// The ranking of the passing ones (`--rank-by`, `search::parse_ranking`): "cost", or "cost=1,energy=0.5".
    pub rank_by: String,
    /// The duty mix beside the config shown, which is its first duty with weight 1: config files of the machine with
    /// their weights, "file.toml=2; other.toml" (weight 1 without); "" for the config's cycle alone.
    pub duties: String,
    /// One motor of a machine with several (`--motor-id`); "" for every motor.
    pub motor: String,
    /// How many of the best candidates' operating points are drawn on their torque-speed curves.
    pub top: usize,
    /// Ratio sweep: `gearbox.ratio` or `geometry.pulley_radius_m`, swept from `from` to `to` (0 each: half and twice
    /// the config's) in `count` values; with `snap` (a gearbox series) the catalogue gearboxes of it around the best ratio.
    pub sweep_key: String,
    pub sweep_from: f64,
    pub sweep_to: f64,
    pub sweep_count: usize,
    pub sweep_snap: String,
    /// Kinematics comparison: the task file (`search::Task`) and the machines' ids, separated by ","; "" for the
    /// machine shown.
    pub task_file: String,
    pub machines: String,
}

impl Default for SearchInputs {
    fn default() -> Self {
        Self {
            vendor: String::new(),
            drives: Drives::Paired,
            rank_by: "cost".into(),
            duties: String::new(),
            motor: String::new(),
            top: 3,
            sweep_key: "gearbox.ratio".into(),
            sweep_from: 0.0,
            sweep_to: 0.0,
            sweep_count: 9,
            sweep_snap: String::new(),
            task_file: String::new(),
            machines: String::new(),
        }
    }
}

/// The workspace maps of the path tab: the plane (two axes), the axis the short move goes along, how far (in its
/// unit), and the nodes per side of the grid.
#[derive(Debug, Clone, PartialEq)]
pub struct MapInputs {
    pub plane: [usize; 2],
    pub axis: usize,
    pub distance: f64,
    pub n: usize,
}

impl Default for MapInputs {
    fn default() -> Self {
        Self { plane: [0, 1], axis: 0, distance: 0.0, n: 7 }
    }
}

/// The makers of the supplies, each once, in the order of `SUPPLIES`.
pub fn vendors() -> Vec<&'static str> {
    let mut out: Vec<&'static str> = Vec::new();
    for supply in SUPPLIES {
        let vendor = vendor_of(supply);
        if !out.contains(&vendor) {
            out.push(vendor);
        }
    }
    out
}

/// The supplies a search runs at: with a maker every one of that maker's ("all": every one), else `supply` alone.
/// Empty where neither names one of `SUPPLIES`.
pub fn supplies(vendor: &str, supply: &str) -> Vec<&'static str> {
    SUPPLIES
        .iter()
        .copied()
        .filter(|s| match vendor {
            "" => *s == supply,
            "all" => true,
            vendor => vendor_of(s) == vendor,
        })
        .collect()
}

/// A search's result: the candidates ranked (`search::rank_by`), and for the first `SearchInputs::top` of them that ran
/// their motor's torque-speed curves (`None` where its config does not build again).
pub struct Found {
    pub candidates: Vec<Candidate>,
    pub curves: Vec<Option<Curves>>,
}

fn refused(e: ConfigError) -> Message {
    (e.key(), e.detail().to_string())
}

/// The duty mix of `SearchInputs::duties` beside `config`, the first duty: each file read as a config of the machine
/// with its parts resolved. Empty for none.
fn duty_mix(entry: &MachineEntry, config: &Config, text: &str) -> Result<Vec<(Config, f64)>, Message> {
    let files: Vec<&str> = text.split(';').map(str::trim).filter(|f| !f.is_empty()).collect();
    if files.is_empty() {
        return Ok(Vec::new());
    }
    let mut out = vec![(catalog::resolve(entry, config).map_err(refused)?, 1.0)];
    for part in files {
        let (file, weight) = part.rsplit_once('=').map_or((part, "1"), |(f, w)| (f.trim(), w.trim()));
        let weight: f64 = weight
            .parse()
            .ok()
            .filter(|w: &f64| *w > 0.0 && w.is_finite())
            .ok_or_else(|| ("ui-search-bad-duty", part.to_string()))?;
        let bytes = std::fs::read(file).map_err(|e| ("ui-file-unreadable", format!("{file}: {e}")))?;
        let text = sim_components::import::decode(&bytes);
        let cfg = sim_components::load_config(entry, &text).map_err(refused)?;
        out.push((catalog::resolve(entry, &cfg).map_err(refused)?, weight));
    }
    Ok(out)
}

/// The search of the search tab, as `sim-cli search` runs it with the same options: at the supplies of `supplies`,
/// with `inputs.series` and `inputs.gearboxes`, every axis (`inputs.axes`) or one motor, the drives and the duty mix of
/// `SearchInputs`, ranked by its `rank_by`. A search that finds nothing to try says why.
pub fn run_search(entry: &MachineEntry, config: &Config, inputs: &Inputs) -> Result<Found, Message> {
    let s = &inputs.search;
    let at = supplies(&s.vendor, &inputs.supply);
    if at.is_empty() {
        return Err(("ui-search-bad-supply", format!("{} {}", s.vendor, inputs.supply).trim().to_string()));
    }
    let by = search::parse_ranking(&s.rank_by).ok_or_else(|| ("ui-search-bad-rank", s.rank_by.clone()))?;
    let motor = Some(s.motor.trim()).filter(|m| !m.is_empty());
    if motor.is_some() && inputs.axes {
        return Err(("ui-search-motor-axes", String::new()));
    }
    let duties = duty_mix(entry, config, &s.duties)?;
    // With a mix the first duty's config is searched on, as the command line does.
    let config = duties.first().map_or(config, |(first, _)| first);
    let (series, gearboxes) = (inputs.series.trim(), Some(inputs.gearboxes.trim()).filter(|g| !g.is_empty()));
    let wanted = |article: &str| article.starts_with(series);
    let options = Options { gearboxes, axes: inputs.axes, motor, drives: s.drives, duties: &duties };
    let mut found = catalog::search(entry, config, &at, &wanted, &options).map_err(refused)?;
    if found.iter().all(|c| c.article.is_empty()) {
        // Motors there are, so it is the gearbox series that has nothing for them.
        let motors = at
            .iter()
            .any(|supply| catalog::motor_articles_for(supply).iter().any(|article| article.starts_with(series)));
        return Err(match gearboxes.filter(|_| motors) {
            Some(gearboxes) => ("ui-search-no-gearbox", gearboxes.to_string()),
            None => ("ui-search-none", series.to_string()),
        });
    }
    search::rank_by(&mut found, &by);
    let curves = found
        .iter()
        .filter(|c| c.result.is_some())
        .take(s.top)
        .map(|c| {
            let sim = Sim::new(entry, &catalog::candidate_config(entry, config, c).ok()?).ok()?;
            // One motor's search: that motor's own drive and motor data.
            let k = sim.machine.motors().iter().position(|m| m.id == c.motor).unwrap_or(0);
            Some(torque_speed_curves(sim.control.drive_params_of(k)))
        })
        .collect();
    Ok(Found { candidates: found, curves })
}

/// The candidates × checks heat map: every check of any candidate in the order they first come up, and per candidate
/// in the order of the ranking its utilisation of each (`Figures::checks`, above 1 it fails), NaN where it has none.
pub struct Heatmap {
    pub checks: Vec<&'static str>,
    pub rows: Vec<Vec<f64>>,
}

pub fn heatmap(found: &[Candidate]) -> Heatmap {
    let mut checks: Vec<&'static str> = Vec::new();
    for (check, _) in found.iter().filter_map(|c| c.figures.as_ref()).flat_map(|f| &f.checks) {
        if !checks.contains(check) {
            checks.push(check);
        }
    }
    let rows = found
        .iter()
        .map(|c| {
            let of = |check: &str| c.figures.as_ref()?.checks.iter().find(|(k, _)| *k == check).map(|(_, u)| *u);
            checks.iter().map(|check| of(check).unwrap_or(f64::NAN)).collect()
        })
        .collect();
    Heatmap { checks, rows }
}

/// One candidate in the scatter of cost against margin: its place in the ranking, its cost (installed mass, kg), its
/// worst margin (1 less the tightest check's utilisation: below 0 it fails), its energy per cycle (J, NaN without
/// figures), whether it passes, and whether it is on the front (`search::front`).
#[derive(Debug, Clone, PartialEq)]
pub struct ScatterPoint {
    pub index: usize,
    pub cost: f64,
    pub margin: f64,
    pub energy: f64,
    pub passes: bool,
    pub front: bool,
}

/// The candidates that ran, finite in cost and margin (a tripped drive is infinitely far beyond), as `ScatterPoint`s.
pub fn scatter(found: &[Candidate]) -> Vec<ScatterPoint> {
    let front = search::front(found);
    found
        .iter()
        .enumerate()
        .filter_map(|(index, c)| {
            let margin = 1.0 - c.tightest()?.1;
            let energy = c.figures.as_ref().map_or(f64::NAN, |f| f.energy_j);
            (margin.is_finite() && c.cost.is_finite()).then_some(ScatterPoint {
                index,
                cost: c.cost,
                margin,
                energy,
                passes: c.passes(),
                front: front[index],
            })
        })
        .collect()
}

/// Why a candidate failed: its checks that fail, with value, limit and utilisation; empty for one that passes or did
/// not run.
pub fn failures(c: &Candidate) -> Vec<&Verdict> {
    c.result.as_ref().map_or(Vec::new(), |(_, verdicts)| verdicts.iter().filter(|v| !v.ok).collect())
}

/// A failing check as a line: the check, the motor it concerns, value, limit and utilisation.
pub fn failure_line(tr: &Translations, v: &Verdict, motor_keys: &[&str]) -> String {
    let motor = v.motor.and_then(|m| motor_keys.get(m)).map_or(String::new(), |key| format!(" ({})", tr.tr(key)));
    format!(
        "{}{motor}: {} / {} = {}",
        tr.tr(&format!("check-{}", v.check)),
        short(v.value),
        short(v.limit),
        short(utilisation(v))
    )
}

/// The ratio sweep of the search tab, as `sim-cli transmission` runs it: `SearchInputs::sweep_key` from `sweep_from` to
/// `sweep_to` (half and twice the config's where 0) in `sweep_count` values, and with `sweep_snap` the gearboxes of
/// that series that fit the config's motor around the best ratio.
pub fn ratio_sweep(entry: &MachineEntry, config: &Config, s: &SearchInputs) -> Result<TransmissionSweep, Message> {
    let key = s.sweep_key.as_str();
    let own = catalog::resolve(entry, config).and_then(|c| c.positive(key)).map_err(refused)?;
    let (from, to) = (
        if s.sweep_from > 0.0 { s.sweep_from } else { own / 2.0 },
        if s.sweep_to > 0.0 { s.sweep_to } else { own * 2.0 },
    );
    let parts = catalog::Parts::of(config).map_err(refused)?;
    let snap = Some(s.sweep_snap.trim())
        .filter(|series| !series.is_empty() && !parts.motor.is_empty())
        .map(|series| (series, parts.motor, parts.supply));
    search::transmission_sweep(entry, config, key, (from, to, s.sweep_count), snap).map_err(refused)
}

/// The kinematics comparison of the search tab, as `sim-cli compare-kinematics` runs it: the task file mapped onto each
/// machine of `SearchInputs::machines` (the one shown with `config`, the others with their default config), the search
/// at the supplies of `supplies` with the series, gearbox series, drives and ranking of the search tab.
pub fn compare(entry: &'static MachineEntry, config: &Config, inputs: &Inputs) -> Result<Vec<KinematicsRow>, Message> {
    let s = &inputs.search;
    let file = s.task_file.trim();
    let bytes = std::fs::read(file).map_err(|e| ("ui-file-unreadable", format!("{file}: {e}")))?;
    let task =
        Task::parse(&sim_components::import::decode(&bytes)).map_err(|e| ("ui-file-line", format!("{file}: {e}")))?;
    let ids: Vec<&str> = s.machines.split(',').map(str::trim).filter(|id| !id.is_empty()).collect();
    let machines: Vec<(&MachineEntry, Config)> = if ids.is_empty() {
        vec![(entry, config.clone())]
    } else {
        ids.iter()
            .map(|id| match MACHINES.iter().copied().find(|m| m.id == *id) {
                Some(m) if m.id == entry.id => Ok((m, config.clone())),
                Some(m) => Ok((m, m.config())),
                None => Err(("ui-search-unknown-machine", id.to_string())),
            })
            .collect::<Result<_, _>>()?
    };
    let at = supplies(&s.vendor, &inputs.supply);
    if at.is_empty() {
        return Err(("ui-search-bad-supply", inputs.supply.clone()));
    }
    let by = search::parse_ranking(&s.rank_by).ok_or_else(|| ("ui-search-bad-rank", s.rank_by.clone()))?;
    let series = inputs.series.trim();
    let gearboxes = Some(inputs.gearboxes.trim()).filter(|g| !g.is_empty());
    let options = Options { gearboxes, drives: s.drives, ..Options::default() };
    let rows = search::compare_kinematics(
        &task,
        &machines,
        &|_| at.clone(),
        &|article: &str| article.starts_with(series),
        &options,
        &by,
    );
    Ok(rows)
}

/// The kinematics comparison side by side: a column per machine, a row per figure of its best candidate.
pub fn kinematics_table(tr: &Translations, rows: &[KinematicsRow]) -> Table {
    let keys = [
        "ui-kinematics-machine",
        "ui-analysis-result",
        "ui-analysis-motor",
        "ui-search-gearbox",
        "ui-parts-drive",
        "ui-search-supply",
        "ui-search-cost",
        "ui-kinematics-cycle",
        "ui-kinematics-target",
        "ui-kinematics-energy",
        "ui-search-tightest",
        "ui-search-share",
        "ui-kinematics-thermal",
        "ui-kinematics-error",
        "ui-kinematics-inertia",
        "ui-kinematics-passing",
    ];
    let mut columns: Vec<Vec<String>> = vec![keys.iter().map(|k| tr.tr(k).to_string()).collect()];
    for row in rows {
        let machine =
            MACHINES.iter().find(|m| m.id == row.machine).map_or(row.machine.clone(), |m| tr.tr(m.key).into());
        let target = row.target_s.map_or(String::new(), short);
        let mut column = vec![machine];
        match (&row.refused, &row.best) {
            (Some(r), _) => column.push(format!("{}: {}", tr.tr(r.key), r.detail)),
            (None, None) => column.push(tr.tr("ui-search-refused").into()),
            (None, Some(c)) => {
                let f = c.figures.clone().unwrap_or_default();
                let (check, u) =
                    c.tightest().map_or((String::new(), f64::NAN), |(k, u)| (tr.tr(&format!("check-{k}")).into(), u));
                column.extend([
                    tr.tr(if row.meets() { "ui-analysis-ok" } else { "ui-analysis-fail" }).to_string(),
                    c.article.to_string(),
                    c.gearbox.clone(),
                    c.drive.to_string(),
                    c.supply.to_string(),
                    short(c.cost),
                    short(f.cycle_s),
                    target.clone(),
                    short(f.energy_j),
                    check,
                    short(u),
                    short(f.thermal),
                    short(f.following_error_m * 1e6),
                    short(f.inertia_ratio),
                    format!("{} / {}", row.passing, row.tried),
                ]);
            }
        }
        column.resize(keys.len(), String::new());
        columns.push(column);
    }
    (0..keys.len()).map(|i| columns.iter().map(|c| c[i].clone()).collect()).collect()
}

/// The workspace maps of the path tab: over the plane of the two axes `axes`, from `x` and `y` (low, high) in `n` nodes
/// per side (node (i, j) at x.0 + i (x.1 - x.0) / (n - 1), j outer), per node the peak motor torque (N m, the highest of
/// the motors) and the peak following error (of the axes in the unit of the moved axis) of a short move from there;
/// NaN where the machine cannot start or make it.
#[derive(Debug, Clone)]
pub struct MoveMap {
    pub axes: [usize; 2],
    pub x: (f64, f64),
    pub y: (f64, f64),
    pub n: usize,
    pub torque: Vec<f64>,
    pub error: Vec<f64>,
    /// The axis moved, and how far.
    pub axis: usize,
    pub distance: f64,
}

/// How long a short move may take, s, beyond which it counts as not made.
const MOVE_LIMIT_S: f64 = 10.0;
/// How long after the setpoint has arrived the move is watched, s: the settling belongs to it.
const SETTLE_S: f64 = 0.1;

/// One short move: the machine at rest at `from` (`start.pose`), moved point to point to `to` (`Command::MoveTo`)
/// and watched until `SETTLE_S` after it has arrived. The peak motor torque, the highest of the motors (N m), and the
/// peak following error of the axes in the unit of axis `unit_of`.
pub fn short_move(
    entry: &MachineEntry,
    config: &Config,
    from: &[f64],
    to: &[f64],
    unit_of: usize,
) -> Result<(f64, f64), Message> {
    let mut cfg = config.clone();
    cfg.set_text(&format!("start.pose = {from:?}")).map_err(refused)?;
    let mut sim = Sim::new(entry, &cfg).map_err(refused)?;
    let infos = sim.signal_infos();
    let unit = sim.machine.axes().get(unit_of).map(|a| a.unit.to_string()).unwrap_or_default();
    let torques: Vec<usize> =
        (0..infos.len()).filter(|&i| infos[i].id.starts_with("motor.") && infos[i].id.ends_with(".torque")).collect();
    let errors: Vec<usize> = (0..infos.len())
        .filter(|&i| {
            infos[i].id.starts_with("axis.") && infos[i].id.ends_with(".following_error") && infos[i].unit == unit
        })
        .collect();
    sim.command(Command::MoveTo(to.to_vec())).map_err(|e| (e.key, String::new()))?;
    let (start, mut arrived, mut row) = (sim.time(), None, Vec::new());
    let (mut torque, mut error) = (0.0f64, 0.0f64);
    loop {
        sim.step();
        row.clear();
        sim.sample(&mut row);
        torque = torques.iter().map(|&i| row[i].abs()).fold(torque, f64::max);
        error = errors.iter().map(|&i| row[i].abs()).fold(error, f64::max);
        let now = sim.time();
        if !sim.control.busy() {
            let since = *arrived.get_or_insert(now);
            if now - since >= SETTLE_S {
                return Ok((torque, error));
            }
        }
        if now - start > MOVE_LIMIT_S {
            return Err(("ui-map-move-too-long", String::new()));
        }
    }
}

/// The nodes of a move map: the plane's axes over their travel, the other axes where the config starts them.
fn map_nodes(
    config: &Config,
    axes: &[sim_core::AxisInfo],
    m: &MapInputs,
) -> Result<(Vec<f64>, (f64, f64), (f64, f64)), Message> {
    let [a, b] = m.plane;
    if a == b || a >= axes.len() || b >= axes.len() || m.axis >= axes.len() || m.n < 2 {
        return Err(("ui-map-bad-plane", String::new()));
    }
    let base = config.f64s("start.pose").map_err(refused)?;
    Ok((base, (axes[a].min, axes[a].max), (axes[b].min, axes[b].max)))
}

/// The distance a map's move goes when `MapInputs::distance` is 0: a tenth of the moved axis's travel.
pub fn default_distance(axis: &sim_core::AxisInfo) -> f64 {
    (axis.max - axis.min) / 10.0
}

/// The move map of `MapInputs`: from each node of the grid a short move (`short_move`) along `axis` by `distance` (a
/// tenth of its travel where 0), backwards where forwards would leave the travel. The moves share the processors.
pub fn move_map(entry: &MachineEntry, config: &Config, m: &MapInputs) -> Result<MoveMap, Message> {
    let resolved = catalog::resolve(entry, config).map_err(refused)?;
    let machine = (entry.build)(&resolved).map_err(refused)?;
    let axes = machine.axes();
    let (base, x, y) = map_nodes(config, axes, m)?;
    let moved = &axes[m.axis];
    let distance = if m.distance > 0.0 { m.distance } else { default_distance(moved) };
    let n = m.n;
    let at = |(low, high): (f64, f64), i: usize| low + (high - low) * i as f64 / (n - 1) as f64;
    let poses: Vec<(Vec<f64>, Vec<f64>)> = (0..n * n)
        .map(|k| {
            let mut from = base.clone();
            (from[m.plane[0]], from[m.plane[1]]) = (at(x, k % n), at(y, k / n));
            let mut to = from.clone();
            let ahead = from[m.axis] + distance;
            to[m.axis] = if ahead <= moved.max { ahead } else { from[m.axis] - distance };
            (from, to)
        })
        .collect();
    let results = on_cores(poses.len(), |k| short_move(entry, config, &poses[k].0, &poses[k].1, m.axis).ok());
    let (torque, error) = results.into_iter().map(|r| r.unwrap_or((f64::NAN, f64::NAN))).unzip();
    Ok(MoveMap { axes: m.plane, x, y, n, torque, error, axis: m.axis, distance })
}

/// `work` for 0..count on every processor, the results in order.
fn on_cores<T: Send>(count: usize, work: impl Fn(usize) -> T + Sync) -> Vec<T> {
    let threads = std::thread::available_parallelism().map_or(1, |n| n.get()).min(count.max(1));
    let next = std::sync::atomic::AtomicUsize::new(0);
    let mut out: Vec<(usize, T)> = std::thread::scope(|scope| {
        let handles: Vec<_> = (0..threads)
            .map(|_| {
                scope.spawn(|| {
                    let mut done = Vec::new();
                    loop {
                        let k = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                        if k >= count {
                            return done;
                        }
                        done.push((k, work(k)));
                    }
                })
            })
            .collect();
        handles.into_iter().flat_map(|h| h.join().unwrap_or_default()).collect()
    });
    out.sort_by_key(|(k, _)| *k);
    out.into_iter().map(|(_, t)| t).collect()
}

/// The computations of these views beside the analysis panel's, each in a thread of its own.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ViewJob {
    Sweep,
    Kinematics,
    Map,
}

enum Done {
    Sweep(TransmissionSweep),
    Kinematics(Vec<KinematicsRow>),
    Map(MoveMap),
}

/// What these views keep: the top candidates' curves of the last search, and the last ratio sweep, kinematics
/// comparison and move map, each with the config it was computed for; and the jobs under way.
#[derive(Default)]
pub struct Views {
    pub curves: Vec<Option<Curves>>,
    pub sweep: Option<(Config, TransmissionSweep)>,
    pub kinematics: Option<(Config, Vec<KinematicsRow>)>,
    pub map: Option<(Config, MoveMap)>,
    jobs: Vec<(ViewJob, Config, Mutex<Receiver<Result<Done, Message>>>)>,
}

impl Views {
    /// Starts one computation with the config and the inputs as they are now.
    pub fn start(&mut self, kind: ViewJob, entry: &'static MachineEntry, config: &Config, inputs: &Inputs) {
        let (config, inputs, (send, receive)) = (config.clone(), inputs.clone(), channel());
        self.jobs.push((kind, config.clone(), Mutex::new(receive)));
        std::thread::spawn(move || {
            let done = match kind {
                ViewJob::Sweep => ratio_sweep(entry, &config, &inputs.search).map(Done::Sweep),
                ViewJob::Kinematics => compare(entry, &config, &inputs).map(Done::Kinematics),
                ViewJob::Map => move_map(entry, &config, &inputs.map).map(Done::Map),
            };
            let _ = send.send(done);
        });
    }

    pub fn running(&self, kind: ViewJob) -> bool {
        self.jobs.iter().any(|job| job.0 == kind)
    }

    pub fn any_running(&self) -> bool {
        !self.jobs.is_empty()
    }

    /// Takes in what the jobs have answered; the messages of those that failed. A job whose thread is gone without an
    /// answer (it panicked) answers with a message.
    pub fn poll(&mut self) -> Vec<Message> {
        let mut said = Vec::new();
        let mut done = Vec::new();
        self.jobs.retain(|(_, config, job)| {
            let answer = match job.lock().map(|job| job.try_recv()) {
                Ok(Err(TryRecvError::Empty)) => return true,
                Ok(Ok(answer)) => answer,
                _ => Err(("ui-job-ended", String::new())),
            };
            done.push((config.clone(), answer));
            false
        });
        for (config, answer) in done {
            match answer {
                Ok(Done::Sweep(s)) => self.sweep = Some((config, s)),
                Ok(Done::Kinematics(rows)) => self.kinematics = Some((config, rows)),
                Ok(Done::Map(map)) => self.map = Some((config, map)),
                Err(message) => said.push(message),
            }
        }
        said
    }
}
